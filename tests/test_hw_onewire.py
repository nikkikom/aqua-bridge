"""Tests for aqua_bridge.hw.onewire against a fake w1 sysfs tree (no hardware).

PROJECT.md section 3 (Track B) / the DAS plan section 1 "Read timing vs dt".
"""

from __future__ import annotations

import threading
import time
from pathlib import Path

import pytest

from aqua_bridge.config import load_config
from aqua_bridge.hw.onewire import W1Source, build_onewire_from_config
from aqua_bridge.hw.w1_netlink import W1Netlink
from aqua_bridge.hw.w1_therm_netlink import reg_num_from_rom_name
from aqua_bridge.model import ConfigError
from conftest import EXAMPLE_CONFIG, EXAMPLE_DAS_CONFIG
from w1_netlink_fakes import (
    CAPTURED_SCRATCHPAD,
    FakeSocket,
    convert_exchange,
    list_masters_reply,
    scratchpad,
    scratchpad_exchange,
)


class _FakeClock:
    """A clock that only moves when the test tells it to -- safe wherever the
    code under test never spins on it (the happy poll path returns before
    consulting the clock a second time; see ``_patch_trigger`` below for the
    one path that does spin)."""

    def __init__(self, t: float = 1000.0) -> None:
        self.t = t

    def __call__(self) -> float:
        return self.t


def _make_bus(root: Path, bus_name: str) -> Path:
    bus = root / bus_name
    bus.mkdir(parents=True)
    (bus / "therm_bulk_read").write_text("1")
    return bus


def _make_slave(
    bus: Path,
    rom: str,
    milli: str | None,
    *,
    resolution: str | None = None,
    conv_time: str | None = None,
    identifier: bytes | None = None,
) -> Path:
    slave = bus / rom
    slave.mkdir(parents=True)
    if milli is not None:
        (slave / "temperature").write_text(milli)
    if resolution is not None:
        (slave / "resolution").write_text(resolution)
    if conv_time is not None:
        (slave / "conv_time").write_text(conv_time)
    if identifier is not None:
        # The slave's ``id`` attribute: the raw struct w1_reg_num a netlink
        # slave command addresses it by (8 bytes, not text).
        (slave / "id").write_bytes(identifier)
    return slave


class _TriggerLog:
    """What the code under test did to one ``therm_bulk_read`` attribute."""

    def __init__(self) -> None:
        self.writes: list[bytes] = []
        self.reads: list[str] = []


def _patch_trigger(
    monkeypatch: pytest.MonkeyPatch, trigger_path: Path, statuses: list[str]
) -> _TriggerLog:
    """Makes ``trigger_path.read_text()`` return ``statuses`` in order (repeating
    the last entry once exhausted) and swallows writes to it, recording both.

    A real ``therm_bulk_read`` attribute is a kernel command/status channel,
    not a byte store: writing "trigger" does not make a later read return
    "trigger" back, and the reader's decision depends on the *sequence* of
    values the driver reports around the write (before it, immediately after
    it, then while polling -- ``hw/onewire.py``, "Reading strategy"). A plain
    file can't reproduce that on its own, so this patches only reads/writes of
    this one path (everything else -- the ``temperature`` / ``resolution`` /
    ``conv_time`` files -- goes through the real filesystem unpatched). The
    recorded writes are bytes on purpose: the kernel takes the trigger only at
    exactly eight of them.
    """
    original_write = Path.write_bytes
    original_read = Path.read_text
    log = _TriggerLog()

    def fake_write(self: Path, data: bytes, *a: object, **kw: object) -> int:
        if self == trigger_path:
            log.writes.append(bytes(data))
            return len(data)
        return original_write(self, data, *a, **kw)

    def fake_read(self: Path, *a: object, **kw: object) -> str:
        if self == trigger_path:
            idx = min(len(log.reads), len(statuses) - 1)
            log.reads.append(statuses[idx])
            return statuses[idx]
        return original_read(self, *a, **kw)

    monkeypatch.setattr(Path, "write_bytes", fake_write)
    monkeypatch.setattr(Path, "read_text", fake_read)
    return log


# --- discovery ---------------------------------------------------------------------


def test_discover_buses_finds_bus_master_dirs_sorted(tmp_path: Path) -> None:
    root = tmp_path / "w1"
    _make_bus(root, "w1_bus_master2")
    _make_bus(root, "w1_bus_master1")
    (root / "not_a_bus").mkdir()

    src = W1Source({"a": "28-000000000001"}, max_age_s=10.0, root=root)

    assert [p.name for p in src.discover_buses()] == ["w1_bus_master1", "w1_bus_master2"]


def test_discover_buses_missing_root_is_empty(tmp_path: Path) -> None:
    src = W1Source({"a": "28-000000000001"}, max_age_s=10.0, root=tmp_path / "nope")
    assert src.discover_buses() == []


def test_missing_roms_reports_undiscovered_sensors(tmp_path: Path) -> None:
    root = tmp_path / "w1"
    bus = _make_bus(root, "w1_bus_master1")
    _make_slave(bus, "28-000000000001", "23000")

    src = W1Source(
        {"present": "28-000000000001", "absent": "28-000000000002"}, max_age_s=10.0, root=root
    )
    assert src.missing_roms() == ["28-000000000002"]


# --- run_bus_cycle: the trigger/poll/read cycle -------------------------------------


def test_run_bus_cycle_reads_present_sensors(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    root = tmp_path / "w1"
    bus = _make_bus(root, "w1_bus_master1")
    _make_slave(bus, "28-000000000001", "23562")
    _patch_trigger(monkeypatch, bus / "therm_bulk_read", ["1"])

    src = W1Source({"prox_b01": "28-000000000001"}, max_age_s=10.0, root=root, clock=_FakeClock())
    result = src.run_bus_cycle(bus)

    assert result == {"prox_b01": pytest.approx(23.562)}


def test_run_bus_cycle_ignores_sensors_of_another_bus(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    root = tmp_path / "w1"
    bus1 = _make_bus(root, "w1_bus_master1")
    bus2 = _make_bus(root, "w1_bus_master2")
    _make_slave(bus1, "28-000000000001", "20000")
    _make_slave(bus2, "28-000000000002", "21000")
    _patch_trigger(monkeypatch, bus1 / "therm_bulk_read", ["1"])

    src = W1Source(
        {"a": "28-000000000001", "b": "28-000000000002"},
        max_age_s=10.0,
        root=root,
        clock=_FakeClock(),
    )
    assert src.run_bus_cycle(bus1) == {"a": pytest.approx(20.0)}


def test_run_bus_cycle_with_no_declared_sensor_present_is_empty(tmp_path: Path) -> None:
    root = tmp_path / "w1"
    bus = _make_bus(root, "w1_bus_master1")
    _make_slave(bus, "28-0000000000ff", "20000")  # not declared

    src = W1Source({"a": "28-000000000001"}, max_age_s=10.0, root=root)
    assert src.run_bus_cycle(bus) == {}


def test_redundant_sensors_share_one_rom_and_both_get_the_reading(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    root = tmp_path / "w1"
    bus = _make_bus(root, "w1_bus_master1")
    _make_slave(bus, "28-000000000001", "22000")
    _patch_trigger(monkeypatch, bus / "therm_bulk_read", ["1"])

    src = W1Source(
        {"prox_b01": "28-000000000001", "prox_b01b": "28-000000000001"},
        max_age_s=10.0,
        root=root,
        clock=_FakeClock(),
    )
    result = src.run_bus_cycle(bus)
    assert result == {"prox_b01": pytest.approx(22.0), "prox_b01b": pytest.approx(22.0)}


# --- a read that produces no value -> None, counted by why ---------------------------


def test_bad_temperature_content_is_none_and_counted_as_rejected(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    root = tmp_path / "w1"
    bus = _make_bus(root, "w1_bus_master1")
    _make_slave(bus, "28-000000000001", "not-a-number")
    _patch_trigger(monkeypatch, bus / "therm_bulk_read", ["1"])

    src = W1Source({"a": "28-000000000001"}, max_age_s=10.0, root=root, clock=_FakeClock())
    result = src.run_bus_cycle(bus)

    assert result == {"a": None}
    stats = src.read_stats()["28-000000000001"]
    assert (stats.attempts, stats.rejected, stats.failed) == (1, 1, 1)
    assert (stats.errors, stats.empty) == (0, 0)


def test_empty_temperature_is_none_and_counted_apart_from_an_error(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    """A slave that answers with nothing at all is its own category.

    The kernel does that while a bulk conversion it started is still in flight,
    i.e. when a second reader is on this bus master -- not a bad sensor and not
    a bad value (``hw/onewire.py``, "One cycle at a time per bus master";
    PROJECT.md item 39). The sensor reads ``None`` for this cycle either way and
    its siblings on the bus are unaffected.
    """
    root = tmp_path / "w1"
    bus = _make_bus(root, "w1_bus_master1")
    _make_slave(bus, "28-000000000001", "")  # empty temperature attribute
    _make_slave(bus, "28-000000000002", "21500")
    _patch_trigger(monkeypatch, bus / "therm_bulk_read", ["1"])

    src = W1Source(
        {"a": "28-000000000001", "b": "28-000000000002"},
        max_age_s=10.0,
        root=root,
        clock=_FakeClock(),
    )
    assert src.run_bus_cycle(bus) == {"a": None, "b": pytest.approx(21.5)}
    stats = src.read_stats()
    empty = stats["28-000000000001"]
    assert (empty.attempts, empty.empty, empty.failed) == (1, 1, 1)
    assert (empty.errors, empty.rejected) == (0, 0)
    good = stats["28-000000000002"]
    assert (good.attempts, good.failed) == (1, 0)


def test_missing_temperature_file_is_none_and_counted_as_an_error(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    root = tmp_path / "w1"
    bus = _make_bus(root, "w1_bus_master1")
    _make_slave(bus, "28-000000000001", None)  # no temperature file: kernel EIO equivalent
    _patch_trigger(monkeypatch, bus / "therm_bulk_read", ["1"])

    src = W1Source({"a": "28-000000000001"}, max_age_s=10.0, root=root, clock=_FakeClock())
    result = src.run_bus_cycle(bus)

    assert result == {"a": None}
    stats = src.read_stats()["28-000000000001"]
    assert (stats.attempts, stats.errors, stats.failed) == (1, 1, 1)
    assert (stats.empty, stats.rejected) == (0, 0)


def test_a_sensor_no_cycle_reached_has_no_attempts_and_so_no_rate(tmp_path: Path) -> None:
    """A declared ROM id that is not under any bus master must not read as 0 % failed.

    It is a binding or wiring report and nothing was ever asked of it; the
    denominator that used to be printed for it (``cycles * len(buses)``) belonged
    to no sensor at all (PROJECT.md item 39).
    """
    root = tmp_path / "w1"
    bus = _make_bus(root, "w1_bus_master1")
    _make_slave(bus, "28-000000000001", "22000")

    src = W1Source(
        {"here": "28-000000000001", "nowhere": "28-00000000ffff"},
        max_age_s=10.0,
        root=root,
        clock=_FakeClock(),
    )
    src.run_bus_cycle(bus)

    stats = src.read_stats()
    assert stats["28-000000000001"].attempts == 1
    absent = stats["28-00000000ffff"]
    assert (absent.attempts, absent.failed) == (0, 0)
    assert src.missing_roms() == ["28-00000000ffff"]


def test_bulk_read_never_completing_is_bounded_and_falls_back_to_serial(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    """An implementation that reports a conversion still running and never
    finishes it must not block the reader (it used to report ``None`` for the
    whole bus after the timeout; since section 8 item 38 the bus reads serially
    in the same cycle, so the sensor keeps a reading instead of going
    untrusted). The kernel's bulk read converts inside the ``write()`` and can
    never show ``-1`` to a single-threaded caller, but the ABI allows it, and
    the wait for it stays bounded by ``bulk_timeout_s`` -- which is what is
    asserted here, on a real advancing clock.
    """
    root = tmp_path / "w1"
    bus = _make_bus(root, "w1_bus_master1")
    _make_slave(bus, "28-000000000001", "20000")
    log = _patch_trigger(monkeypatch, bus / "therm_bulk_read", ["0", "-1", "-1"])  # never "1"

    src = W1Source(
        {"a": "28-000000000001"},
        max_age_s=10.0,
        root=root,
        clock=time.monotonic,  # a real, advancing clock so the deadline is actually reached
        poll_interval_s=0.005,
        bulk_timeout_s=0.05,
        # One failing cycle gives the tier up here, so this test stays about the
        # bounded wait; how much evidence a demotion needs by default has its
        # own tests below ("giving up a tier").
        tier_failures_before_demote=1,
    )
    t0 = time.monotonic()
    result = src.run_bus_cycle(bus)
    elapsed = time.monotonic() - t0

    assert result == {"a": pytest.approx(20.0)}  # serial read in the same cycle
    assert elapsed < 1.0, "the bounded wait outlived bulk_timeout_s by more than a factor 20"
    assert src.bulk_read_modes() == {"w1_bus_master1": "serial"}

    writes_after_demotion = len(log.writes)
    src.run_bus_cycle(bus)
    assert log.writes == log.writes[:writes_after_demotion], (
        "a bus that just dropped to serial reads was triggered again inside bulk_retry_s"
    )


def test_unreadable_trigger_file_falls_back_to_serial_without_raising(tmp_path: Path) -> None:
    """``therm_bulk_read`` that cannot even be read (here a directory; on the
    board, the same OSError shape as the EACCES of a missing udev rule) is not a
    reason to lose the bus: no exception, no wait, serial reads."""
    root = tmp_path / "w1"
    bus = _make_bus(root, "w1_bus_master1")
    _make_slave(bus, "28-000000000001", "20000")
    (bus / "therm_bulk_read").unlink()
    (bus / "therm_bulk_read").mkdir()  # reading/writing it now raises IsADirectoryError

    src = W1Source(
        {"a": "28-000000000001"},
        max_age_s=10.0,
        root=root,
        clock=_FakeClock(),
        tier_failures_before_demote=1,  # see the note in the bounded-wait test above
    )
    assert src.run_bus_cycle(bus) == {"a": pytest.approx(20.0)}
    assert src.read_tiers() == {"w1_bus_master1": "serial"}
    assert src.bulk_read_modes() == {"w1_bus_master1": "serial"}


# --- which read path a bus ends up on (section 8 item 38) -----------------------------


def test_bulk_path_is_used_when_the_driver_answers_the_trigger(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    """The kernel's own behaviour: ``therm_bulk_read`` reads "0" with nothing
    pending and "1" by the time the ``write()`` returns, because the conversion
    happens inside that syscall. That bus keeps the cheaper path -- one
    conversion for every slave -- and the status is looked at once, not polled.

    The trigger is **eight bytes**: ``therm_bulk_read_store()`` compares the
    write size against ``sizeof("trigger")``, NUL included, and silently ignores
    a seven-byte one (section 8 item 38 -- this module used to write seven).
    """
    root = tmp_path / "w1"
    bus = _make_bus(root, "w1_bus_master1")
    _make_slave(bus, "28-000000000001", "20000")
    _make_slave(bus, "28-000000000002", "21000")
    log = _patch_trigger(monkeypatch, bus / "therm_bulk_read", ["0", "1"])

    src = W1Source(
        {"a": "28-000000000001", "b": "28-000000000002"},
        max_age_s=10.0,
        root=root,
        clock=_FakeClock(),  # never advances: any polling here would hang the test
    )
    result = src.run_bus_cycle(bus)

    assert result == {"a": pytest.approx(20.0), "b": pytest.approx(21.0)}
    assert log.writes == [b"trigger\n"]
    assert len(log.writes[0]) == 8
    assert log.reads == ["0", "1"], "the status is read once before and once after the trigger"
    assert src.bulk_read_modes() == {"w1_bus_master1": "bulk"}


def test_a_trigger_that_registers_nothing_is_detected_on_the_first_cycle(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    """A trigger the kernel refuses is silent -- the store function returns the
    write size either way and only the kernel log says ``err=-22`` / ``-ENODEV``
    -- so the only evidence is that ``therm_bulk_read`` is still at the ABI's
    "no bulk conversion pending". The reader must notice at once, with no wait at
    all, read serially instead, and not trigger again for ``bulk_retry_s``.

    The retry matters: the usual cause is a family-``00`` phantom on the master
    (section 8 item 38), and phantoms come and go, so the bus must be able to
    get the fast path back."""
    root = tmp_path / "w1"
    bus = _make_bus(root, "w1_bus_master1")
    _make_slave(bus, "28-000000000001", "20000")
    log = _patch_trigger(monkeypatch, bus / "therm_bulk_read", ["0"])  # "0" forever

    clock = _FakeClock()  # never advances on its own: a wait here would hang the test
    src = W1Source(
        {"a": "28-000000000001"},
        max_age_s=10.0,
        root=root,
        clock=clock,
        bulk_timeout_s=30.0,
        bulk_retry_s=300.0,
        tier_failures_before_demote=1,  # see the note in the bounded-wait test above
    )
    assert src.run_bus_cycle(bus) == {"a": pytest.approx(20.0)}
    assert src.read_tiers() == {"w1_bus_master1": "serial"}, (
        "a refused trigger means this cycle converted per sensor, whatever the tier's standing"
    )
    assert src.bulk_read_modes() == {"w1_bus_master1": "serial"}
    assert log.writes == [b"trigger\n"]

    clock.t += 299.0
    assert src.run_bus_cycle(bus) == {"a": pytest.approx(20.0)}
    assert log.writes == [b"trigger\n"], "triggered again inside bulk_retry_s"

    clock.t += 2.0  # past bulk_retry_s
    assert src.run_bus_cycle(bus) == {"a": pytest.approx(20.0)}
    assert len(log.writes) == 2, "the bus was never probed again"
    assert src.bulk_read_modes() == {"w1_bus_master1": "serial"}


def test_a_bus_that_honours_the_trigger_again_gets_the_fast_path_back(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    """The phantom that killed the trigger is gone by the next search: after
    ``bulk_retry_s`` the probe succeeds and the bus is back on the bulk path."""
    root = tmp_path / "w1"
    bus = _make_bus(root, "w1_bus_master1")
    _make_slave(bus, "28-000000000001", "20000")
    # refused, refused, then accepted (pre-trigger "0", post-trigger "1")
    log = _patch_trigger(monkeypatch, bus / "therm_bulk_read", ["0", "0", "0", "1"])

    clock = _FakeClock()
    src = W1Source(
        {"a": "28-000000000001"},
        max_age_s=10.0,
        root=root,
        clock=clock,
        bulk_retry_s=60.0,
        tier_failures_before_demote=1,  # see the note in the bounded-wait test above
    )
    src.run_bus_cycle(bus)
    assert src.bulk_read_modes() == {"w1_bus_master1": "serial"}

    clock.t += 61.0
    assert src.run_bus_cycle(bus) == {"a": pytest.approx(20.0)}
    assert src.bulk_read_modes() == {"w1_bus_master1": "bulk"}
    assert len(log.writes) == 2


def test_bulk_attribute_absent_reads_serially(tmp_path: Path) -> None:
    """``therm_bulk_read`` only exists on a master once its first ``w1_therm``
    slave attached, and a master may never get one. No attribute, no trigger."""
    root = tmp_path / "w1"
    bus = root / "w1_bus_master1"
    bus.mkdir(parents=True)  # deliberately no therm_bulk_read
    _make_slave(bus, "28-000000000001", "19750")

    src = W1Source({"a": "28-000000000001"}, max_age_s=10.0, root=root, clock=_FakeClock())
    assert src.run_bus_cycle(bus) == {"a": pytest.approx(19.75)}
    assert src.bulk_read_modes() == {"w1_bus_master1": "serial"}


def test_a_value_outside_the_abi_falls_back_to_serial(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    root = tmp_path / "w1"
    bus = _make_bus(root, "w1_bus_master1")
    _make_slave(bus, "28-000000000001", "20000")
    log = _patch_trigger(monkeypatch, bus / "therm_bulk_read", ["trigger"])

    src = W1Source(
        {"a": "28-000000000001"},
        max_age_s=10.0,
        root=root,
        clock=_FakeClock(),
        tier_failures_before_demote=1,  # see the note in the bounded-wait test above
    )
    assert src.run_bus_cycle(bus) == {"a": pytest.approx(20.0)}
    assert src.bulk_read_modes() == {"w1_bus_master1": "serial"}
    assert log.writes == [], "an attribute that is not a w1_therm state must not be written"


def test_bulk_read_off_never_touches_the_trigger_attribute(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    root = tmp_path / "w1"
    bus = _make_bus(root, "w1_bus_master1")
    _make_slave(bus, "28-000000000001", "20000")
    log = _patch_trigger(monkeypatch, bus / "therm_bulk_read", ["0", "-1", "1"])

    src = W1Source(
        {"a": "28-000000000001"},
        max_age_s=10.0,
        root=root,
        clock=_FakeClock(),
        bulk_read="off",
    )
    assert src.run_bus_cycle(bus) == {"a": pytest.approx(20.0)}
    assert log.writes == [] and log.reads == []
    assert src.bulk_read_modes() == {}  # nothing probed, nothing to report


def test_default_resolution_fits_the_planned_sensor_count(tmp_path: Path) -> None:
    """The arithmetic behind the ``resolution_bits`` default (section 8 item 39),
    executable: the per-bus budget is ``max_age_s / 2`` so one lost cycle never
    ages a sensor out, the plan is 24 sensors on two buses read in parallel, and
    the default is the finest step that fits the two *bus-wide* tiers -- which
    the netlink tier made available on every bus, including the one the kernel
    gives no ``therm_bulk_read`` to.

    The serial tier is asserted here too, and asserted *not* to fit: it is the
    floor a demoted bus lands on, and the day something makes it affordable this
    assertion is what says to re-derive the default rather than assume it.
    """
    # Measured on the board through this module: cycle time per tier, fitted
    # against the sensor count over n = 2, 4, 6, 8 (section 8 items 38, 39).
    fixed_ms = {"netlink": {12: 764, 10: 203}, "sysfs_bulk": {12: 759, 10: 211}}
    per_sensor_ms = {"netlink": 16.6, "sysfs_bulk": 20.5}
    serial_ms = {12: 800, 11: 416, 10: 228, 9: 132}  # per sensor, conv_time + ~40 ms
    dt, sensors_per_bus = 5.0, 12
    budget_s = (1.5 * dt) / 2  # default max_age_s = 1.5 * dt

    src = W1Source({"a": "28-000000000001"}, max_age_s=1.5 * dt, root=tmp_path)
    chosen = src._resolution_bits
    assert chosen == 12, "the default must be the value section 8 item 39 argues for"
    # It is the finest step the sensor has, so "the finest that fits" is only a
    # claim about the tiers it has to fit: both bus-wide ones, with room to spare.
    for tier, fixed in fixed_ms.items():
        cycle_s = (fixed[chosen] + sensors_per_bus * per_sensor_ms[tier]) / 1000
        assert cycle_s <= budget_s / 2, f"{tier} at {chosen} bit: {cycle_s:.2f} s"
    # And it does not fit the tier below them: a bus demoted to reading one
    # sensor at a time cannot keep 12 fresh at 12 bit -- the cycle is longer than
    # max_age_s itself, so its sensors read as missing part of the time (which
    # raises cooling and never lowers it) until it is promoted back.
    demoted_s = sensors_per_bus * serial_ms[chosen] / 1000
    assert demoted_s > 2 * budget_s
    assert demoted_s > 1.5 * dt  # ... longer than max_age_s
    # The coarse alternatives a config can reach for, and what each one buys on
    # that same demoted bus: 10 bit fits the budget, 11 bit does not.
    assert sensors_per_bus * serial_ms[10] / 1000 <= budget_s
    assert sensors_per_bus * serial_ms[11] / 1000 > budget_s


# --- resolution: set once, never rewritten -------------------------------------------


def test_resolution_written_once_on_first_sighting(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    root = tmp_path / "w1"
    bus = _make_bus(root, "w1_bus_master1")
    _make_slave(bus, "28-000000000001", "20000")
    _patch_trigger(monkeypatch, bus / "therm_bulk_read", ["1", "1"])

    src = W1Source(
        {"a": "28-000000000001"}, max_age_s=10.0, root=root, resolution_bits=11, clock=_FakeClock()
    )
    src.run_bus_cycle(bus)
    assert (bus / "28-000000000001" / "resolution").read_text() == "11"


def test_resolution_not_rewritten_on_later_cycles(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    root = tmp_path / "w1"
    bus = _make_bus(root, "w1_bus_master1")
    _make_slave(bus, "28-000000000001", "20000")
    _patch_trigger(monkeypatch, bus / "therm_bulk_read", ["1", "1"])

    src = W1Source(
        {"a": "28-000000000001"}, max_age_s=10.0, root=root, resolution_bits=12, clock=_FakeClock()
    )
    src.run_bus_cycle(bus)
    (bus / "28-000000000001" / "resolution").write_text("9")  # simulate an external reset
    src.run_bus_cycle(bus)
    assert (bus / "28-000000000001" / "resolution").read_text() == "9"  # not forced back to 12


def test_conv_time_is_read_back_after_the_resolution_write(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    """The driver recomputes ``conv_time`` from the resolution it accepted, so
    reading it back is how the reader reports what a cycle will actually cost
    (``tools/w1_commission.py --check``) instead of assuming the datasheet."""
    root = tmp_path / "w1"
    bus = _make_bus(root, "w1_bus_master1")
    _make_slave(bus, "28-000000000001", "20000", conv_time="190")
    _patch_trigger(monkeypatch, bus / "therm_bulk_read", ["1"])

    src = W1Source(
        {"a": "28-000000000001"}, max_age_s=10.0, root=root, resolution_bits=10, clock=_FakeClock()
    )
    src.run_bus_cycle(bus)
    assert src.conv_time_ms() == {"28-000000000001": 190}


def test_resolution_write_refused_is_a_warning_and_the_sensor_still_reads(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path, caplog: pytest.LogCaptureFixture
) -> None:
    """The root-owned attribute without the udev rule of
    ``deploy/99-w1-therm.rules``: the write fails, the sensor keeps whatever
    resolution it has and is still read (a permission problem must not cost
    cooling -- PROJECT.md section 2)."""
    root = tmp_path / "w1"
    bus = _make_bus(root, "w1_bus_master1")
    slave = _make_slave(bus, "28-000000000001", "20000")
    (slave / "resolution").mkdir()  # writing to it now raises IsADirectoryError
    _patch_trigger(monkeypatch, bus / "therm_bulk_read", ["1"])

    src = W1Source({"a": "28-000000000001"}, max_age_s=10.0, root=root, clock=_FakeClock())
    with caplog.at_level("WARNING"):
        assert src.run_bus_cycle(bus) == {"a": pytest.approx(20.0)}
    assert any("cannot set resolution" in rec.message for rec in caplog.records)


def test_a_sensor_reporting_another_resolution_is_warned(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path, caplog: pytest.LogCaptureFixture
) -> None:
    """Written, read back, and the sensor says something else: the estimator's
    ``quant_c`` is sized on the configured resolution, so a silent disagreement
    is worth naming."""
    root = tmp_path / "w1"
    bus = _make_bus(root, "w1_bus_master1")
    slave = _make_slave(bus, "28-000000000001", "20000", conv_time="750")
    _patch_trigger(monkeypatch, bus / "therm_bulk_read", ["1"])
    original_write = Path.write_text

    def stubborn(self: Path, data: str, *a: object, **kw: object) -> int:
        if self == slave / "resolution":
            return original_write(self, "12", *a, **kw)  # ignores what it was asked for
        return original_write(self, data, *a, **kw)

    monkeypatch.setattr(Path, "write_text", stubborn)

    src = W1Source(
        {"a": "28-000000000001"}, max_age_s=10.0, root=root, resolution_bits=10, clock=_FakeClock()
    )
    with caplog.at_level("WARNING"):
        src.run_bus_cycle(bus)
    assert any("reports resolution 12 bits" in rec.message for rec in caplog.records)


# --- read(): non-blocking, staleness ---------------------------------------------------


def test_read_never_reported_sensor_is_none(tmp_path: Path) -> None:
    src = W1Source({"a": "28-000000000001"}, max_age_s=10.0, root=tmp_path / "w1")
    assert src.read() == {"a": None}


def test_read_reports_published_sample(monkeypatch: pytest.MonkeyPatch, tmp_path: Path) -> None:
    root = tmp_path / "w1"
    bus = _make_bus(root, "w1_bus_master1")
    _make_slave(bus, "28-000000000001", "18500")
    _patch_trigger(monkeypatch, bus / "therm_bulk_read", ["1"])
    clock = _FakeClock(100.0)

    src = W1Source({"a": "28-000000000001"}, max_age_s=10.0, root=root, clock=clock)
    src._publish(src.run_bus_cycle(bus))

    assert src.read() == {"a": pytest.approx(18.5)}


def test_read_ages_out_a_stale_sample(monkeypatch: pytest.MonkeyPatch, tmp_path: Path) -> None:
    root = tmp_path / "w1"
    bus = _make_bus(root, "w1_bus_master1")
    _make_slave(bus, "28-000000000001", "18500")
    _patch_trigger(monkeypatch, bus / "therm_bulk_read", ["1"])
    clock = _FakeClock(100.0)

    src = W1Source({"a": "28-000000000001"}, max_age_s=5.0, root=root, clock=clock)
    src._publish(src.run_bus_cycle(bus))
    assert src.read() == {"a": pytest.approx(18.5)}

    clock.t = 106.0  # 6 s later, past max_age_s = 5
    assert src.read() == {"a": None}


def test_read_does_not_touch_the_filesystem(tmp_path: Path) -> None:
    """Non-blocking on the loop thread: read() must not stat/open anything."""
    root = tmp_path / "w1"
    src = W1Source({"a": "28-000000000001"}, max_age_s=10.0, root=root)

    def _boom(*_a: object, **_kw: object) -> None:
        raise AssertionError("read() touched the filesystem")

    orig_is_dir = Path.is_dir
    try:
        Path.is_dir = _boom  # type: ignore[method-assign]
        assert src.read() == {"a": None}
    finally:
        Path.is_dir = orig_is_dir  # type: ignore[method-assign]


# --- construction validation -----------------------------------------------------------


def test_empty_sensors_is_config_error(tmp_path: Path) -> None:
    with pytest.raises(ConfigError, match="onewire.sensors must not be empty"):
        W1Source({}, max_age_s=10.0, root=tmp_path)


@pytest.mark.parametrize("bits", [8, 13, 0, -1])
def test_bad_resolution_bits_is_config_error(tmp_path: Path, bits: int) -> None:
    with pytest.raises(ConfigError, match="resolution_bits"):
        W1Source({"a": "28-1"}, max_age_s=10.0, root=tmp_path, resolution_bits=bits)


@pytest.mark.parametrize("max_age_s", [0.0, -1.0])
def test_bad_max_age_is_config_error(tmp_path: Path, max_age_s: float) -> None:
    with pytest.raises(ConfigError, match="max_age_s"):
        W1Source({"a": "28-1"}, max_age_s=max_age_s, root=tmp_path)


@pytest.mark.parametrize("bulk_retry_s", [0.0, -1.0])
def test_bad_bulk_retry_is_config_error(tmp_path: Path, bulk_retry_s: float) -> None:
    with pytest.raises(ConfigError, match="bulk_retry_s"):
        W1Source({"a": "28-1"}, max_age_s=10.0, root=tmp_path, bulk_retry_s=bulk_retry_s)


@pytest.mark.parametrize("bulk_timeout_s", [0.0, -1.0])
def test_bad_bulk_timeout_is_config_error(tmp_path: Path, bulk_timeout_s: float) -> None:
    """The only wait in a cycle is bounded and its bound is a config key: a
    non-positive one would mean no wait at all on a kernel where bulk works."""
    with pytest.raises(ConfigError, match="bulk_timeout_s"):
        W1Source({"a": "28-1"}, max_age_s=10.0, root=tmp_path, bulk_timeout_s=bulk_timeout_s)


@pytest.mark.parametrize("mode", ["on", "true", "", "Auto"])
def test_bad_bulk_read_mode_is_config_error(tmp_path: Path, mode: str) -> None:
    with pytest.raises(ConfigError, match="bulk_read"):
        W1Source({"a": "28-1"}, max_age_s=10.0, root=tmp_path, bulk_read=mode)


# --- start()/stop(): real reader threads ------------------------------------------------


def test_start_spawns_a_thread_per_bus_and_publishes(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    root = tmp_path / "w1"
    bus = _make_bus(root, "w1_bus_master1")
    _make_slave(bus, "28-000000000001", "21000")
    _patch_trigger(monkeypatch, bus / "therm_bulk_read", ["1"])  # repeats "1" forever

    src = W1Source(
        {"a": "28-000000000001"},
        max_age_s=10.0,
        root=root,
        clock=time.monotonic,
        poll_interval_s=0.005,
    )
    src.start()
    try:
        deadline = time.monotonic() + 2.0
        while src.read()["a"] is None and time.monotonic() < deadline:
            time.sleep(0.01)
        assert src.read()["a"] == pytest.approx(21.0)
        assert src.cycle_counts().get("w1_bus_master1", 0) >= 1
    finally:
        src.stop()
    assert all(not t.is_alive() for t in src._threads) or src._threads == []


def test_start_is_idempotent(monkeypatch: pytest.MonkeyPatch, tmp_path: Path) -> None:
    root = tmp_path / "w1"
    bus = _make_bus(root, "w1_bus_master1")
    _make_slave(bus, "28-000000000001", "21000")
    _patch_trigger(monkeypatch, bus / "therm_bulk_read", ["1"])

    src = W1Source({"a": "28-000000000001"}, max_age_s=10.0, root=root, poll_interval_s=0.005)
    src.start()
    threads_first = list(src._threads)
    src.start()
    assert src._threads == threads_first
    src.stop()


def test_stop_without_start_is_a_noop(tmp_path: Path) -> None:
    src = W1Source({"a": "28-000000000001"}, max_age_s=10.0, root=tmp_path / "w1")
    src.stop()  # must not raise


def test_start_with_no_bus_master_logs_and_does_not_raise(tmp_path: Path) -> None:
    src = W1Source({"a": "28-000000000001"}, max_age_s=10.0, root=tmp_path / "empty")
    src.start()  # no bus masters at all: no threads, no exception
    src.stop()


def test_start_with_missing_rom_is_a_warning_not_fatal(
    caplog: pytest.LogCaptureFixture, tmp_path: Path
) -> None:
    root = tmp_path / "w1"
    _make_bus(root, "w1_bus_master1")  # no slaves at all
    src = W1Source({"a": "28-000000000001"}, max_age_s=10.0, root=root, poll_interval_s=0.01)
    with caplog.at_level("WARNING"):
        src.start()
    try:
        assert any("28-000000000001" in rec.message for rec in caplog.records)
    finally:
        src.stop()


# --- build_onewire_from_config ---------------------------------------------------------


def test_build_onewire_returns_none_when_sensors_absent() -> None:
    assert build_onewire_from_config({}, default_max_age_s=7.5) is None


def test_build_onewire_returns_none_for_legacy_empty_list_placeholder() -> None:
    # config.example.yaml's placeholder is `onewire: {enabled: false, sensors: []}`.
    assert (
        build_onewire_from_config({"enabled": False, "sensors": []}, default_max_age_s=7.5) is None
    )


def test_build_onewire_rejects_non_empty_non_mapping_sensors() -> None:
    with pytest.raises(ConfigError, match="onewire.sensors must be a mapping"):
        build_onewire_from_config({"sensors": ["28-1", "28-2"]}, default_max_age_s=7.5)


def test_build_onewire_builds_a_source(tmp_path: Path) -> None:
    section = {
        "sensors": {"prox_b01": "28-0316a27a0aff"},
        "resolution_bits": 11,
        "root": str(tmp_path),
    }
    src = build_onewire_from_config(section, default_max_age_s=7.5)
    assert src is not None
    assert src.sensors == {"prox_b01": "28-0316a27a0aff"}


def test_build_onewire_defaults_the_read_strategy_keys(tmp_path: Path) -> None:
    """One documented default each, and they are the ones PROJECT.md quotes."""
    src = build_onewire_from_config(
        {"sensors": {"a": "28-1"}, "root": str(tmp_path)}, default_max_age_s=7.5
    )
    assert src is not None
    assert src._resolution_bits == 12
    assert src._bulk_read == "auto"
    assert src._bulk_timeout_s == pytest.approx(2.0)
    assert src._bulk_retry_s == pytest.approx(300.0)
    assert src._poll_interval_s == pytest.approx(0.02)
    assert src._read_tier == "auto"
    assert src._netlink_timeout_s == pytest.approx(1.0)
    assert src._netlink_retry_s == pytest.approx(300.0)
    assert src._tier_failures_before_demote == 3
    assert src._slow_cycle_log_interval_s == pytest.approx(60.0)


def test_build_onewire_passes_the_read_strategy_keys_through(tmp_path: Path) -> None:
    src = build_onewire_from_config(
        {
            "sensors": {"a": "28-1"},
            "root": str(tmp_path),
            "bulk_read": "off",
            "bulk_timeout_s": 3.5,
            "bulk_retry_s": 30.0,
            "poll_interval_s": 0.05,
            "read_tier": "sysfs_bulk",
            "netlink_timeout_s": 0.25,
            "netlink_retry_s": 60.0,
            "tier_failures_before_demote": 1,
            "slow_cycle_log_interval_s": 15.0,
        },
        default_max_age_s=7.5,
    )
    assert src is not None
    assert src._bulk_read == "off"
    assert src._bulk_timeout_s == pytest.approx(3.5)
    assert src._bulk_retry_s == pytest.approx(30.0)
    assert src._poll_interval_s == pytest.approx(0.05)
    assert src._read_tier == "sysfs_bulk"
    assert src._netlink_timeout_s == pytest.approx(0.25)
    assert src._netlink_retry_s == pytest.approx(60.0)
    assert src._tier_failures_before_demote == 1
    assert src._slow_cycle_log_interval_s == pytest.approx(15.0)


def test_build_onewire_uses_default_max_age_when_absent(tmp_path: Path) -> None:
    section = {"sensors": {"a": "28-1"}, "root": str(tmp_path)}
    src = build_onewire_from_config(section, default_max_age_s=12.5)
    assert src is not None
    assert src._max_age_s == pytest.approx(12.5)


def test_build_onewire_explicit_max_age_overrides_default(tmp_path: Path) -> None:
    section = {"sensors": {"a": "28-1"}, "max_age_s": 3.0, "root": str(tmp_path)}
    src = build_onewire_from_config(section, default_max_age_s=99.0)
    assert src is not None
    assert src._max_age_s == pytest.approx(3.0)


@pytest.mark.parametrize(
    ("section", "match"),
    [
        ("not-a-mapping", "onewire section must be a mapping"),
        ({"sensors": {"": "28-1"}}, "non-empty string"),
        ({"sensors": {"a": ""}}, "ROM id"),
        ({"sensors": {"a": "28-1"}, "resolution_bits": "x"}, "resolution_bits must be an int"),
        ({"sensors": {"a": "28-1"}, "max_age_s": "x"}, "max_age_s must be a number"),
        ({"sensors": {"a": "28-1"}, "bulk_read": 1}, "bulk_read must be a string"),
        ({"sensors": {"a": "28-1"}, "bulk_read": "on"}, "bulk_read must be one of"),
        ({"sensors": {"a": "28-1"}, "bulk_timeout_s": "x"}, "bulk_timeout_s must be a number"),
        ({"sensors": {"a": "28-1"}, "bulk_timeout_s": 0}, "bulk_timeout_s must be > 0"),
        ({"sensors": {"a": "28-1"}, "bulk_retry_s": "x"}, "bulk_retry_s must be a number"),
        ({"sensors": {"a": "28-1"}, "bulk_retry_s": -5}, "bulk_retry_s must be > 0"),
        ({"sensors": {"a": "28-1"}, "poll_interval_s": "x"}, "poll_interval_s must be a number"),
        ({"sensors": {"a": "28-1"}, "poll_interval_s": -1}, "poll_interval_s must be > 0"),
        ({"sensors": {"a": "28-1"}, "read_tier": 1}, "read_tier must be a string"),
        ({"sensors": {"a": "28-1"}, "read_tier": "bulk"}, "read_tier must be one of"),
        (
            {"sensors": {"a": "28-1"}, "netlink_timeout_s": "x"},
            "netlink_timeout_s must be a number",
        ),
        ({"sensors": {"a": "28-1"}, "netlink_timeout_s": 0}, "netlink_timeout_s must be > 0"),
        ({"sensors": {"a": "28-1"}, "netlink_retry_s": "x"}, "netlink_retry_s must be a number"),
        ({"sensors": {"a": "28-1"}, "netlink_retry_s": -1}, "netlink_retry_s must be > 0"),
        (
            {"sensors": {"a": "28-1"}, "tier_failures_before_demote": 1.5},
            "tier_failures_before_demote must be an int",
        ),
        (
            {"sensors": {"a": "28-1"}, "tier_failures_before_demote": 0},
            "tier_failures_before_demote must be >= 1",
        ),
        (
            {"sensors": {"a": "28-1"}, "slow_cycle_log_interval_s": "x"},
            "slow_cycle_log_interval_s must be a number",
        ),
        (
            {"sensors": {"a": "28-1"}, "slow_cycle_log_interval_s": 0},
            "slow_cycle_log_interval_s must be > 0",
        ),
        ({"sensors": {"a": "28-1"}, "enabled": "true"}, "onewire.enabled must be true or false"),
        # item 57: the check runs even when it would otherwise return None (no sensors) --
        # a mistyped enabled: is a config mistake regardless of whether anything reads it.
        ({"enabled": "yes"}, "onewire.enabled must be true or false"),
        ({"enabled": 1}, "onewire.enabled must be true or false"),
    ],
)
def test_build_onewire_malformed_section_is_config_error(section: object, match: str) -> None:
    with pytest.raises(ConfigError, match=match):
        build_onewire_from_config(section, default_max_age_s=7.5)  # type: ignore[arg-type]


# -- the tier ladder: netlink, then the kernel's bulk read, then one at a time ------
#
# These drive the real transport and the real framing over a fake socket, so a
# cycle's requests are the bytes the board would see; nothing opens a socket
# (tests/conftest.py takes the netlink family away as well). A sensor's sysfs
# ``temperature`` file and its scratchpad are given *different* readings on
# purpose: the value alone then says which tier produced it.

_SYSFS_MILLI = "20000"  # what the sysfs tiers would report: 20.0 C
_NETLINK_C = 22.1875  # what the captured scratchpad decodes to
_OTHER_C = 24.0


def _netlink_bus(
    root: Path, roms: list[str], *, conv_time: str = "750"
) -> tuple[Path, list[bytes]]:
    """A one-master fake tree whose slaves carry an ``id`` attribute."""
    bus = _make_bus(root, "w1_bus_master1")
    ids = []
    for rom in roms:
        identifier = reg_num_from_rom_name(rom)
        ids.append(identifier)
        _make_slave(
            bus, rom, _SYSFS_MILLI, resolution="12", conv_time=conv_time, identifier=identifier
        )
    return bus, ids


def _cycle_replies(seq: int, scratchpads: dict[bytes, bytes], *, masters: tuple[int, ...] = (1,)):
    """Every datagram one netlink cycle is answered with, in order."""
    out = [list_masters_reply(seq, masters), *convert_exchange(seq + 1)]
    for index, (identifier, data) in enumerate(scratchpads.items()):
        out += scratchpad_exchange(seq + 2 + index, identifier, data)
    return out


def _source(
    root: Path, roms: dict[str, str], sock: FakeSocket | None, **kwargs: object
) -> W1Source:
    factory = None
    if sock is not None:
        factory = lambda: W1Netlink(  # noqa: E731 - one expression, and it needs this socket
            timeout_s=0.5, socket_factory=lambda *a, **kw: sock
        )
    return W1Source(
        roms,
        resolution_bits=12,
        max_age_s=10.0,
        root=root,
        clock=kwargs.pop("clock", _FakeClock()),  # type: ignore[arg-type]
        netlink_factory=factory,
        **kwargs,  # type: ignore[arg-type]
    )


def _probe_count(sock: FakeSocket) -> int:
    """How many ``W1_LIST_MASTERS`` requests were sent (one per netlink attempt)."""
    return sum(1 for packet in sock.sent if packet[36] == 6)


def test_a_bus_reads_over_netlink_when_the_connector_answers(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """The preferred tier: one conversion for the bus, one scratchpad per sensor,
    and the kernel's own bulk attribute never touched."""
    root = tmp_path / "w1"
    bus, ids = _netlink_bus(root, ["28-000000000001", "28-000000000002"])
    log = _patch_trigger(monkeypatch, bus / "therm_bulk_read", ["1"])
    sock = FakeSocket(_cycle_replies(1, {ids[0]: CAPTURED_SCRATCHPAD, ids[1]: scratchpad(0x0180)}))
    src = _source(root, {"a": "28-000000000001", "b": "28-000000000002"}, sock)

    result = src.run_bus_cycle(bus)

    assert result == {"a": pytest.approx(_NETLINK_C), "b": pytest.approx(_OTHER_C)}, (
        "the readings are the scratchpads', not the sysfs temperature files'"
    )
    assert src.read_tiers() == {"w1_bus_master1": "netlink"}
    assert log.writes == [], "the kernel's bulk trigger was not written at all"
    assert _probe_count(sock) == 1
    stats = src.read_stats()
    assert [(s.attempts, s.failed) for s in stats.values()] == [(1, 0), (1, 0)]


def test_a_bus_the_connector_does_not_answer_falls_to_the_kernels_bulk_read(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, caplog: pytest.LogCaptureFixture
) -> None:
    """A kernel without the w1 connector answers nothing, which is a timeout. The
    same cycle still finishes one tier down, so no sample is lost."""
    root = tmp_path / "w1"
    bus, _ids = _netlink_bus(root, ["28-000000000001"])
    log = _patch_trigger(monkeypatch, bus / "therm_bulk_read", ["0", "1"])
    sock = FakeSocket([])  # every recv times out
    # One timeout is enough to give the tier up here; that a timeout on its own is
    # not, by default, is the counting policy's own test below.
    src = _source(root, {"a": "28-000000000001"}, sock, tier_failures_before_demote=1)

    with caplog.at_level("WARNING"):
        result = src.run_bus_cycle(bus)

    assert result == {"a": pytest.approx(20.0)}, "read through sysfs in the same cycle"
    assert src.read_tiers() == {"w1_bus_master1": "bulk"}
    assert log.writes == [b"trigger\n"]
    assert "does not read over netlink" in caplog.text


def test_a_bus_can_fall_all_the_way_to_one_sensor_at_a_time(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """Both fast tiers gone -- no connector, and a bulk trigger the kernel refuses
    (the phantom case) -- and the sensor still reports."""
    root = tmp_path / "w1"
    bus, _ids = _netlink_bus(root, ["28-000000000001"])
    log = _patch_trigger(monkeypatch, bus / "therm_bulk_read", ["0", "0"])  # refused, silently
    src = _source(root, {"a": "28-000000000001"}, FakeSocket([]))

    assert src.run_bus_cycle(bus) == {"a": pytest.approx(20.0)}
    assert src.read_tiers() == {"w1_bus_master1": "serial"}
    assert log.writes == [b"trigger\n"]


def test_a_master_the_connector_does_not_list_is_not_addressed_by_netlink(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """The tier is chosen from what the kernel says, not from a directory name: a
    master id missing from ``W1_LIST_MASTERS`` drops the bus a tier instead of
    sending commands to a master id nobody confirmed."""
    root = tmp_path / "w1"
    bus, ids = _netlink_bus(root, ["28-000000000001"])
    _patch_trigger(monkeypatch, bus / "therm_bulk_read", ["0", "1"])
    sock = FakeSocket(_cycle_replies(1, {ids[0]: CAPTURED_SCRATCHPAD}, masters=(7, 9)))
    src = _source(root, {"a": "28-000000000001"}, sock)

    assert src.run_bus_cycle(bus) == {"a": pytest.approx(20.0)}
    assert src.read_tiers() == {"w1_bus_master1": "bulk"}
    assert len(sock.sent) == 1, "it asked once and sent no command to an unlisted master"


def test_a_bus_that_lost_netlink_is_left_alone_until_netlink_retry_s(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """What takes the tier away can come back (a sensor pulled out, a kernel busy
    elsewhere), so the drop is timed, not permanent -- and inside the window not
    one request is sent."""
    root = tmp_path / "w1"
    bus, ids = _netlink_bus(root, ["28-000000000001"])
    _patch_trigger(monkeypatch, bus / "therm_bulk_read", ["0", "1"])
    clock = _FakeClock()
    sock = FakeSocket([])
    src = _source(
        root,
        {"a": "28-000000000001"},
        sock,
        clock=clock,
        netlink_retry_s=300.0,
        tier_failures_before_demote=1,
    )

    src.run_bus_cycle(bus)
    assert _probe_count(sock) == 1
    clock.t += 299.0
    src.run_bus_cycle(bus)
    assert _probe_count(sock) == 1, "probed again inside netlink_retry_s"

    clock.t += 2.0
    # The demotion closed that socket, so the next attempt opens a fresh
    # transport and its sequence numbers start again at 1.
    sock.replies = _cycle_replies(1, {ids[0]: CAPTURED_SCRATCHPAD})
    assert src.run_bus_cycle(bus) == {"a": pytest.approx(_NETLINK_C)}
    assert _probe_count(sock) == 2
    assert src.read_tiers() == {"w1_bus_master1": "netlink"}


@pytest.mark.parametrize(
    ("tier", "expected_tier", "expected_value"),
    [
        pytest.param("sysfs_bulk", "bulk", 20.0, id="the kernel's bulk read only"),
        pytest.param("serial", "serial", 20.0, id="one sensor at a time only"),
    ],
)
def test_a_forced_lower_tier_never_touches_the_socket(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    tier: str,
    expected_tier: str,
    expected_value: float,
) -> None:
    """``read_tier`` is for debugging, and a pinned tier has to really be the only
    one used -- otherwise a measurement cannot tell the tiers apart."""
    root = tmp_path / "w1"
    bus, _ids = _netlink_bus(root, ["28-000000000001"])
    log = _patch_trigger(monkeypatch, bus / "therm_bulk_read", ["0", "1"])

    def refuse() -> W1Netlink:
        raise AssertionError(f"read_tier={tier} must not build a netlink transport")

    src = W1Source(
        {"a": "28-000000000001"},
        resolution_bits=12,
        max_age_s=10.0,
        root=root,
        clock=_FakeClock(),
        read_tier=tier,
        netlink_factory=refuse,
    )
    assert src.run_bus_cycle(bus) == {"a": pytest.approx(expected_value)}
    assert src.read_tiers() == {"w1_bus_master1": expected_tier}
    assert log.writes == ([b"trigger\n"] if tier == "sysfs_bulk" else [])


def test_a_bus_pinned_to_netlink_publishes_nothing_rather_than_read_another_way(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """The other half of pinning a tier. A sensor with no sample reads as missing,
    which the control loop turns into more cooling and never less, so this is safe
    -- but it is a debugging setting and the default stays ``auto``."""
    root = tmp_path / "w1"
    bus, _ids = _netlink_bus(root, ["28-000000000001"])
    log = _patch_trigger(monkeypatch, bus / "therm_bulk_read", ["0", "1"])
    src = _source(root, {"a": "28-000000000001"}, FakeSocket([]), read_tier="netlink")

    assert src.run_bus_cycle(bus) == {}
    assert src.read_tiers() == {"w1_bus_master1": "none"}
    assert log.writes == []
    assert src.read() == {"a": None}


def test_one_sensor_with_a_bad_crc_loses_its_reading_and_the_bus_keeps_the_tier(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """The CRC is this module's job now (the driver only checks it on its own
    ``temperature`` path), and a sensor that fails it is counted where
    ``w1_commission.py --check`` looks."""
    root = tmp_path / "w1"
    bus, ids = _netlink_bus(root, ["28-000000000001", "28-000000000002"])
    _patch_trigger(monkeypatch, bus / "therm_bulk_read", ["1"])
    corrupt = CAPTURED_SCRATCHPAD[:8] + bytes([CAPTURED_SCRATCHPAD[8] ^ 0xFF])
    sock = FakeSocket(_cycle_replies(1, {ids[0]: corrupt, ids[1]: CAPTURED_SCRATCHPAD}))
    src = _source(root, {"a": "28-000000000001", "b": "28-000000000002"}, sock)

    result = src.run_bus_cycle(bus)

    assert result == {"a": None, "b": pytest.approx(_NETLINK_C)}
    assert src.read_tiers() == {"w1_bus_master1": "netlink"}
    stats = src.read_stats()
    bad = stats["28-000000000001"]
    assert (bad.attempts, bad.errors, bad.failed) == (1, 1, 1)
    # No "empty" on this tier: a netlink read carries nine bytes or raises.
    assert (bad.empty, bad.rejected) == (0, 0)
    assert (stats["28-000000000002"].attempts, stats["28-000000000002"].failed) == (1, 0)


def test_a_bus_where_every_scratchpad_is_zeros_drops_a_tier_and_still_reads(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """Nine zero bytes pass CRC-8 and are what a family-00 phantom answers with on
    the board. Every sensor failing is the bus's symptom, not the sensors', so the
    bus drops a tier -- and the same cycle still reads them the slow way."""
    root = tmp_path / "w1"
    bus, ids = _netlink_bus(root, ["28-000000000001"])
    _patch_trigger(monkeypatch, bus / "therm_bulk_read", ["0", "1"])
    sock = FakeSocket(_cycle_replies(1, {ids[0]: bytes(9)}))
    src = _source(root, {"a": "28-000000000001"}, sock)

    assert src.run_bus_cycle(bus) == {"a": pytest.approx(20.0)}
    assert src.read_tiers() == {"w1_bus_master1": "bulk"}


def test_the_identifier_comes_from_the_id_attribute_and_falls_back_to_the_name(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """The slave's own ``id`` attribute is the authority, checked against the
    directory name; a slave whose ``id`` cannot be read is still addressable,
    because a ROM name plus its CRC-8 *is* the identifier."""
    root = tmp_path / "w1"
    bus = _make_bus(root, "w1_bus_master1")
    _patch_trigger(monkeypatch, bus / "therm_bulk_read", ["1"])
    with_id = reg_num_from_rom_name("28-000000000001")
    _make_slave(bus, "28-000000000001", _SYSFS_MILLI, conv_time="750", identifier=with_id)
    _make_slave(bus, "28-000000000002", _SYSFS_MILLI, conv_time="750")  # no id attribute
    derived = reg_num_from_rom_name("28-000000000002")
    sock = FakeSocket(
        _cycle_replies(1, {with_id: CAPTURED_SCRATCHPAD, derived: CAPTURED_SCRATCHPAD})
    )
    src = _source(root, {"a": "28-000000000001", "b": "28-000000000002"}, sock)

    assert src.run_bus_cycle(bus) == {
        "a": pytest.approx(_NETLINK_C),
        "b": pytest.approx(_NETLINK_C),
    }
    addressed = [packet[40:48] for packet in sock.sent if packet[36] == 5]
    assert addressed == [with_id, derived]


class _RecordingStop:
    """``threading.Event``'s two methods the reader uses, with the waits recorded."""

    def __init__(self) -> None:
        self.waits: list[float] = []

    def wait(self, timeout: float | None = None) -> bool:
        self.waits.append(float(timeout or 0.0))
        return False

    def is_set(self) -> bool:
        return False


def test_the_conversion_wait_is_the_slowest_sensor_the_driver_reports(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """One conversion covers the bus, so it has to be long enough for the slowest
    sensor on it -- 750 ms for a 12-bit one next to a 10-bit one."""
    root = tmp_path / "w1"
    bus = _make_bus(root, "w1_bus_master1")
    _patch_trigger(monkeypatch, bus / "therm_bulk_read", ["1"])
    ids = []
    for rom, conv_time in (("28-000000000001", "190"), ("28-000000000002", "750")):
        identifier = reg_num_from_rom_name(rom)
        ids.append(identifier)
        _make_slave(bus, rom, _SYSFS_MILLI, conv_time=conv_time, identifier=identifier)
    sock = FakeSocket(_cycle_replies(1, dict.fromkeys(ids, CAPTURED_SCRATCHPAD)))
    src = _source(root, {"a": "28-000000000001", "b": "28-000000000002"}, sock)
    stop = _RecordingStop()
    monkeypatch.setattr(src, "_stop", stop)

    src.run_bus_cycle(bus)

    assert stop.waits == [0.750]


def test_a_sensor_that_reports_a_finer_resolution_than_the_driver_lengthens_the_wait(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """A ``resolution`` write that silently failed leaves the driver believing 10 bit
    while the sensor converts for 750 ms, and reading too early returns the
    conversion before last. The sensor's own config register settles it."""
    root = tmp_path / "w1"
    bus = _make_bus(root, "w1_bus_master1")
    _patch_trigger(monkeypatch, bus / "therm_bulk_read", ["1"])
    identifier = reg_num_from_rom_name("28-000000000001")
    _make_slave(bus, "28-000000000001", _SYSFS_MILLI, conv_time="190", identifier=identifier)
    sock = FakeSocket(
        _cycle_replies(1, {identifier: CAPTURED_SCRATCHPAD})  # config 0x7f: 12 bit
        + _cycle_replies(4, {identifier: CAPTURED_SCRATCHPAD})
    )
    src = _source(root, {"a": "28-000000000001"}, sock)
    stop = _RecordingStop()
    monkeypatch.setattr(src, "_stop", stop)

    src.run_bus_cycle(bus)
    src.run_bus_cycle(bus)

    assert stop.waits == [0.190, 0.750], "the second cycle waits for what the sensor reported"


def test_a_sensor_with_no_conv_time_keeps_the_bus_on_the_netlink_tier(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, caplog: pytest.LogCaptureFixture
) -> None:
    """One unreadable attribute on one sensor is not evidence about the bus.

    It used to be: the wait "had to come from a fact", so a sensor with no
    ``conv_time`` dropped the whole bus off its fastest tier for
    ``netlink_retry_s`` -- which is what cost the owner's 14-sensor bus its top
    tier, on a sensor that answers ``conv_time=750`` when asked directly
    (PROJECT.md item 39). There *is* a fact to fall back on: the conversion time
    of the resolution this reader configured, which is an upper bound for any
    sensor at or below it.
    """
    root = tmp_path / "w1"
    bus = _make_bus(root, "w1_bus_master1")
    log = _patch_trigger(monkeypatch, bus / "therm_bulk_read", ["0", "1"])
    identifier = reg_num_from_rom_name("28-000000000001")
    _make_slave(bus, "28-000000000001", _SYSFS_MILLI, identifier=identifier)  # no conv_time
    sock = FakeSocket(_cycle_replies(1, {identifier: CAPTURED_SCRATCHPAD}))
    src = _source(root, {"a": "28-000000000001"}, sock)
    stop = _RecordingStop()
    monkeypatch.setattr(src, "_stop", stop)

    with caplog.at_level("INFO"):
        result = src.run_bus_cycle(bus)

    assert result == {"a": pytest.approx(_NETLINK_C)}, "read over netlink, not a tier down"
    assert src.read_tiers() == {"w1_bus_master1": "netlink"}
    assert log.writes == [], "the kernel's bulk trigger was not needed"
    assert stop.waits == [0.750], "the configured 12-bit conversion time, the safe upper bound"
    assert "has not reported conv_time yet" in caplog.text
    assert "does not read over netlink" not in caplog.text


def test_an_unreadable_conv_time_is_asked_again_on_the_next_cycle(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """A transient failure to read an attribute must not be cached as "absent".

    ``resolution`` is written once per sensor because that is scratchpad traffic;
    ``conv_time`` is a value the driver already holds, so it is asked again every
    cycle until it answers -- and once it does, the wait is the sensor's own
    again (PROJECT.md item 39).
    """
    root = tmp_path / "w1"
    bus = _make_bus(root, "w1_bus_master1")
    _patch_trigger(monkeypatch, bus / "therm_bulk_read", ["0", "1"])
    identifier = reg_num_from_rom_name("28-000000000001")
    slave = _make_slave(bus, "28-000000000001", _SYSFS_MILLI, identifier=identifier)
    nine_bit = scratchpad(0x0180, 0x1F)  # config register: 9 bit, so 95 ms of its own
    sock = FakeSocket(
        _cycle_replies(1, {identifier: nine_bit}) + _cycle_replies(4, {identifier: nine_bit})
    )
    src = _source(root, {"a": "28-000000000001"}, sock)
    stop = _RecordingStop()
    monkeypatch.setattr(src, "_stop", stop)

    src.run_bus_cycle(bus)
    assert src.conv_time_ms() == {}, "nothing to report yet"

    (slave / "conv_time").write_text("190")  # the attribute answers this time
    src.run_bus_cycle(bus)

    assert stop.waits == [0.750, 0.190], "the retried read-back replaced the configured bound"
    assert src.conv_time_ms() == {"28-000000000001": 190}
    assert src.read_tiers() == {"w1_bus_master1": "netlink"}


# -- giving up a tier: whose failure, and how many (PROJECT.md item 39) --------------


def test_one_failing_cycle_does_not_cost_the_bus_its_tier(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, caplog: pytest.LogCaptureFixture
) -> None:
    """A tier costs five minutes to give up and one command to retry, so the
    evidence is ``tier_failures_before_demote`` consecutive failing cycles -- and
    every one of those cycles still finishes on the tier below, so no sample is
    lost while the evidence is collected."""
    root = tmp_path / "w1"
    bus, _ids = _netlink_bus(root, ["28-000000000001"])
    _patch_trigger(monkeypatch, bus / "therm_bulk_read", ["0", "1"])
    sock = FakeSocket([])  # every recv times out: a kernel that answers nothing
    clock = _FakeClock()
    src = _source(root, {"a": "28-000000000001"}, sock, clock=clock, netlink_retry_s=300.0)
    assert src._tier_failures_before_demote == 3

    with caplog.at_level("WARNING"):
        for cycle in (1, 2):
            assert src.run_bus_cycle(bus) == {"a": pytest.approx(20.0)}
            assert _probe_count(sock) == cycle, "the tier was given up too early"
            assert "does not read over netlink" not in caplog.text

    assert src.run_bus_cycle(bus) == {"a": pytest.approx(20.0)}
    assert _probe_count(sock) == 3
    assert "does not read over netlink" in caplog.text

    # And now it is left alone: the demotion window is what keeps the probe from
    # costing every cycle.
    clock.t += 299.0
    src.run_bus_cycle(bus)
    assert _probe_count(sock) == 3


def test_a_tier_this_kernel_cannot_have_is_given_up_at_once(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, caplog: pytest.LogCaptureFixture
) -> None:
    """Counting is for evidence that can change. A kernel with no netlink at all
    (which is how the whole suite runs -- ``tests/conftest.py`` takes the family
    away) cannot answer differently next cycle, so it is not asked twice more."""
    root = tmp_path / "w1"
    bus, _ids = _netlink_bus(root, ["28-000000000001"])
    _patch_trigger(monkeypatch, bus / "therm_bulk_read", ["0", "1"])
    src = _source(root, {"a": "28-000000000001"}, None)  # the real transport, no family

    with caplog.at_level("WARNING"):
        assert src.run_bus_cycle(bus) == {"a": pytest.approx(20.0)}

    assert "does not read over netlink" in caplog.text
    assert "no AF_NETLINK" in caplog.text
    assert src.read_tiers() == {"w1_bus_master1": "bulk"}


def test_a_refused_trigger_is_retried_before_the_bus_loses_the_bulk_tier(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """Same policy on the kernel's bulk read, and the same cheap retry: a phantom
    slave silences the trigger and phantoms come and go (section 8 item 38), so
    two more cycles are asked for before the tier is given up. Every one of them
    still reads every sensor -- and reports itself as the serial cycle it was,
    because a refused trigger leaves each read converting on its own."""
    root = tmp_path / "w1"
    bus = _make_bus(root, "w1_bus_master1")
    _make_slave(bus, "28-000000000001", "20000")
    log = _patch_trigger(monkeypatch, bus / "therm_bulk_read", ["0"])  # "0" forever: refused
    clock = _FakeClock()
    src = W1Source(
        {"a": "28-000000000001"}, max_age_s=10.0, root=root, clock=clock, bulk_retry_s=300.0
    )

    for cycle in (1, 2, 3):
        assert src.run_bus_cycle(bus) == {"a": pytest.approx(20.0)}
        assert len(log.writes) == cycle, "a trigger that may yet work was not tried again"
        assert src.read_tiers() == {"w1_bus_master1": "serial"}
    assert src.bulk_read_modes() == {"w1_bus_master1": "serial"}

    clock.t += 299.0
    assert src.run_bus_cycle(bus) == {"a": pytest.approx(20.0)}
    assert len(log.writes) == 3, "triggered again inside bulk_retry_s"


def test_a_cycle_that_works_again_forgets_the_failures_before_it(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, caplog: pytest.LogCaptureFixture
) -> None:
    """The count is *consecutive* failures. Two bad cycles and a good one leave no
    credit toward a demotion, or a bus that fails every other cycle would lose its
    tier on a fault that never actually stopped it working."""
    root = tmp_path / "w1"
    bus, ids = _netlink_bus(root, ["28-000000000001"])
    _patch_trigger(monkeypatch, bus / "therm_bulk_read", ["0", "1"])
    good = _cycle_replies(1, {ids[0]: CAPTURED_SCRATCHPAD})
    sock = FakeSocket([])
    src = _source(root, {"a": "28-000000000001"}, sock)
    stop = _RecordingStop()
    monkeypatch.setattr(src, "_stop", stop)

    src.run_bus_cycle(bus)
    src.run_bus_cycle(bus)  # two failures: one short of the default
    sock.replies = list(good)
    assert src.run_bus_cycle(bus) == {"a": pytest.approx(_NETLINK_C)}

    with caplog.at_level("WARNING"):
        for _ in range(2):
            sock.replies = []
            src.run_bus_cycle(bus)
        assert "does not read over netlink" not in caplog.text, (
            "the failures before the good cycle were still being counted"
        )


def test_stop_closes_every_netlink_socket(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    root = tmp_path / "w1"
    bus, ids = _netlink_bus(root, ["28-000000000001"])
    _patch_trigger(monkeypatch, bus / "therm_bulk_read", ["1"])
    sock = FakeSocket(_cycle_replies(1, {ids[0]: CAPTURED_SCRATCHPAD}))
    src = _source(root, {"a": "28-000000000001"}, sock)

    src.run_bus_cycle(bus)
    assert not sock.closed
    src.stop()
    assert sock.closed


def test_a_cycle_abandoned_by_stop_publishes_nothing(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """``stop()`` landing inside the conversion leaves the scratchpads that follow
    one conversion stale, and nothing is going to use them anyway."""
    root = tmp_path / "w1"
    bus, ids = _netlink_bus(root, ["28-000000000001"])
    _patch_trigger(monkeypatch, bus / "therm_bulk_read", ["1"])
    sock = FakeSocket(_cycle_replies(1, {ids[0]: CAPTURED_SCRATCHPAD}))
    src = _source(root, {"a": "28-000000000001"}, sock)

    class _StoppedDuringConversion(_RecordingStop):
        def wait(self, timeout: float | None = None) -> bool:
            super().wait(timeout)
            return True

    monkeypatch.setattr(src, "_stop", _StoppedDuringConversion())
    assert src.run_bus_cycle(bus) == {}
    assert src.read() == {"a": None}


# -- stop() reaching a serial or bulk cycle, not just a netlink one -----------------
#
# PR #73's proposed item: a 12-bit serial cycle (9.6 s for 12 sensors) outlives
# stop()'s 5 s join because run_bus_cycle had no stop check between serial reads.
# These cover the fix: the check between per-sensor reads, the check before a new
# bulk-wide conversion is started, and that the real reader thread's join returns
# promptly rather than riding a slow cycle out.


def test_stop_before_the_sysfs_tiers_skips_starting_a_new_bulk_conversion(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """The bulk trigger is one blocking write the kernel sleeps a whole
    conversion out inside (module docstring, "Shutdown"), so the only place to
    act on a stop request already set is before writing it, not during."""
    root = tmp_path / "w1"
    bus = _make_bus(root, "w1_bus_master1")
    _make_slave(bus, "28-000000000001", "20000")
    log = _patch_trigger(monkeypatch, bus / "therm_bulk_read", ["0", "1"])

    src = W1Source({"a": "28-000000000001"}, max_age_s=10.0, root=root, clock=_FakeClock())
    src._stop.set()

    assert src.run_bus_cycle(bus) == {}
    assert log.writes == [], "no new bus-wide conversion was started after stop()"


def test_stop_during_a_serial_cycle_abandons_the_remaining_sensors(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """The check runs between sensors, not inside one (module docstring,
    "Shutdown"): the sensor already being read finishes, the rest do not, and
    the cycle publishes nothing -- the same contract an interrupted netlink
    cycle already has, not a partial result."""
    root = tmp_path / "w1"
    bus = _make_bus(root, "w1_bus_master1")
    _make_slave(bus, "28-000000000001", "20000")
    _make_slave(bus, "28-000000000002", "21000")
    _patch_trigger(monkeypatch, bus / "therm_bulk_read", ["0", "0"])  # refused -> serial

    src = W1Source(
        {"a": "28-000000000001", "b": "28-000000000002"},
        max_age_s=10.0,
        root=root,
        clock=_FakeClock(),
    )
    read_order: list[str] = []
    original = src._read_temperature

    def recording(slave_dir: Path, rom: str) -> float | None:
        read_order.append(rom)
        src._stop.set()  # stop() lands right after this sensor's own read
        return original(slave_dir, rom)

    monkeypatch.setattr(src, "_read_temperature", recording)

    assert src.run_bus_cycle(bus) == {}
    assert read_order == ["28-000000000001"], "the second sensor was never read"
    assert src.read() == {"a": None, "b": None}, "an abandoned cycle publishes nothing"


def test_stop_during_a_serial_cycle_returns_promptly(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    """A serial cycle that would take several seconds is abandoned within
    about one sensor's read, not ridden out to completion or to the 5 s join
    bound -- real reader thread, real clock, real (short) sleeps standing in
    for a sensor's conversion."""
    root = tmp_path / "w1"
    bus = _make_bus(root, "w1_bus_master1")
    roms = [f"28-0000000000{i:02x}" for i in range(1, 9)]  # 8 sensors
    for rom in roms:
        _make_slave(bus, rom, "20000")
    _patch_trigger(monkeypatch, bus / "therm_bulk_read", ["0", "0"])  # refused -> serial

    per_sensor = 0.4
    presleep = 1.5 * per_sensor  # midway into the second sensor's read
    src = W1Source(
        {f"s{i}": rom for i, rom in enumerate(roms)},
        max_age_s=100.0,  # budget well out of the way; this test is about timing
        root=root,
        poll_interval_s=0.01,
    )
    original = src._read_temperature

    def slow(slave_dir: Path, rom: str) -> float | None:
        time.sleep(per_sensor)  # stand-in for a real conversion
        return original(slave_dir, rom)

    monkeypatch.setattr(src, "_read_temperature", slow)

    src.start()
    time.sleep(presleep)
    started = time.monotonic()
    try:
        src.stop()
    finally:
        elapsed = time.monotonic() - started

    uninterrupted = per_sensor * len(roms)
    remaining = uninterrupted - presleep  # riding the cycle out from here would cost this much
    # Comfortably above the ~0.5 * per_sensor this should actually take (finishing the
    # sensor already in flight, plus scheduling slack on a loaded CI runner) and
    # comfortably below `remaining`, so this only passes if the cycle was abandoned.
    threshold = 1.5
    assert threshold < remaining, "test numbers must actually separate fixed from unfixed"
    assert elapsed < threshold, (
        f"stop() took {elapsed:.2f} s; finishing the cycle from here would still take "
        f"about {remaining:.1f} s (of {uninterrupted:.1f} s total) and the join bound is 5 s"
    )
    assert all(not t.is_alive() for t in src._threads) or src._threads == []


# -- the slow-cycle warning: item 39's "a bus that silently reads as missing" ------


def test_a_slow_cycle_logs_a_rate_limited_warning(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path, caplog: pytest.LogCaptureFixture
) -> None:
    """Past the budget, a cycle that ran to completion logs once immediately,
    stays quiet inside ``slow_cycle_log_interval_s``, and logs again once past
    it, naming the measured cycle, the budget, the sensor count and the tier."""
    root = tmp_path / "w1"
    bus = _make_bus(root, "w1_bus_master1")
    _make_slave(bus, "28-000000000001", "20000")
    _patch_trigger(monkeypatch, bus / "therm_bulk_read", ["0", "0"])  # refused -> serial

    clock = _FakeClock(1000.0)
    src = W1Source(
        {"a": "28-000000000001"},
        max_age_s=2.0,  # budget = max_age_s / 2 = 1.0 s
        root=root,
        clock=clock,
        slow_cycle_log_interval_s=30.0,
    )
    original = src._read_temperature

    def slow(slave_dir: Path, rom: str) -> float | None:
        clock.t += 1.5  # over the 1.0 s budget
        return original(slave_dir, rom)

    monkeypatch.setattr(src, "_read_temperature", slow)

    with caplog.at_level("WARNING"):
        assert src.run_bus_cycle(bus) == {"a": pytest.approx(20.0)}
    assert "w1_bus_master1 cycle took 1.50 s" in caplog.text
    assert "budget of 1.00 s" in caplog.text
    assert "1 sensor(s)" in caplog.text
    assert "serial tier" in caplog.text
    caplog.clear()

    clock.t += 5.0  # well inside slow_cycle_log_interval_s
    with caplog.at_level("WARNING"):
        src.run_bus_cycle(bus)
    assert "cycle took" not in caplog.text, "rate limited: no second line yet"
    caplog.clear()

    clock.t += 30.0  # past slow_cycle_log_interval_s since the first line
    with caplog.at_level("WARNING"):
        src.run_bus_cycle(bus)
    assert "cycle took 1.50 s" in caplog.text
    assert "2 exceedance(s) since the last line" in caplog.text


def test_a_cycle_inside_budget_does_not_warn(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path, caplog: pytest.LogCaptureFixture
) -> None:
    root = tmp_path / "w1"
    bus = _make_bus(root, "w1_bus_master1")
    _make_slave(bus, "28-000000000001", "20000")
    _patch_trigger(monkeypatch, bus / "therm_bulk_read", ["1"])

    src = W1Source({"a": "28-000000000001"}, max_age_s=2.0, root=root, clock=_FakeClock(1000.0))

    with caplog.at_level("WARNING"):
        result = src.run_bus_cycle(bus)
    assert result == {"a": pytest.approx(20.0)}
    assert "cycle took" not in caplog.text


def test_a_cycle_abandoned_by_stop_does_not_warn_even_if_slow(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, caplog: pytest.LogCaptureFixture
) -> None:
    """A cycle stop() cut short is shutdown, not a slow bus -- warning about it
    on the way out would only be log noise at the moment it matters least."""
    root = tmp_path / "w1"
    bus = _make_bus(root, "w1_bus_master1")
    _make_slave(bus, "28-000000000001", "20000")
    _make_slave(bus, "28-000000000002", "21000")
    _patch_trigger(monkeypatch, bus / "therm_bulk_read", ["0", "0"])  # refused -> serial

    clock = _FakeClock(1000.0)
    src = W1Source(
        {"a": "28-000000000001", "b": "28-000000000002"},
        max_age_s=2.0,  # budget = 1.0 s
        root=root,
        clock=clock,
    )
    original = src._read_temperature

    def recording(slave_dir: Path, rom: str) -> float | None:
        clock.t += 1.5  # over budget on its own, if it were ever checked
        src._stop.set()
        return original(slave_dir, rom)

    monkeypatch.setattr(src, "_read_temperature", recording)

    with caplog.at_level("WARNING"):
        assert src.run_bus_cycle(bus) == {}
    assert "cycle took" not in caplog.text


# -- one cycle at a time per bus master (PROJECT.md item 39) -------------------------


def _hold_one_read(
    src: W1Source, monkeypatch: pytest.MonkeyPatch, rom_to_hold: str
) -> tuple[threading.Event, threading.Event, list[str]]:
    """Makes every read of ``rom_to_hold`` block until released, recording the order.

    A cycle's reads are where two overlapping cycles do their damage on the real
    bus, so that is where a cycle is held open here. Reads of any other sensor
    pass straight through, so a cycle on another bus master is not held with it.
    """
    inside, release = threading.Event(), threading.Event()
    order: list[str] = []
    original = src._read_temperature

    def blocking(slave_dir: Path, rom: str) -> float | None:
        if rom != rom_to_hold:
            return original(slave_dir, rom)
        order.append(f"enter {rom}")
        inside.set()
        assert release.wait(5.0), "the test never released the held cycle"
        order.append(f"leave {rom}")
        return original(slave_dir, rom)

    monkeypatch.setattr(src, "_read_temperature", blocking)
    return inside, release, order


def test_two_cycles_on_one_bus_master_never_overlap(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """The defect of PROJECT.md item 39, as a test.

    A cycle owns its bus master from the conversion to the last scratchpad: the
    kernel marks each slave "value not read yet" and the first read consumes it,
    so a second cycle in the middle of the first one takes readings away from it
    (800 ms of its own conversion each) and its own reads come back empty while
    the other conversion is in flight -- measured on the board at 6.4 s per cycle
    and a third of the reads lost. Two cycles therefore queue, and the second one
    says so.
    """
    root = tmp_path / "w1"
    bus = _make_bus(root, "w1_bus_master1")
    _make_slave(bus, "28-000000000001", "20000")
    _patch_trigger(monkeypatch, bus / "therm_bulk_read", ["0", "1"])
    src = W1Source({"a": "28-000000000001"}, max_age_s=10.0, root=root, clock=_FakeClock())
    inside, release, order = _hold_one_read(src, monkeypatch, "28-000000000001")
    contended = threading.Event()
    original_log = src._log_contention

    def note_contention(bus_name: str) -> None:
        original_log(bus_name)
        contended.set()

    monkeypatch.setattr(src, "_log_contention", note_contention)

    results: dict[str, dict[str, float | None]] = {}
    first = threading.Thread(target=lambda: results.__setitem__("first", src.run_bus_cycle(bus)))
    first.start()
    assert inside.wait(5.0), "the first cycle never reached a read"
    second = threading.Thread(target=lambda: results.__setitem__("second", src.run_bus_cycle(bus)))
    second.start()

    assert contended.wait(5.0), "the second cycle did not find the bus busy"
    assert order == ["enter 28-000000000001"], "the second cycle read while the first was inside"
    release.set()
    first.join(timeout=5.0)
    second.join(timeout=5.0)

    assert not first.is_alive() and not second.is_alive()
    assert order == ["enter 28-000000000001", "leave 28-000000000001"] * 2
    assert results["first"] == results["second"] == {"a": pytest.approx(20.0)}
    stats = src.read_stats()["28-000000000001"]
    assert (stats.attempts, stats.failed) == (2, 0), "both cycles got their reading"


def test_two_cycles_on_one_bus_master_are_logged_once_for_that_bus(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, caplog: pytest.LogCaptureFixture
) -> None:
    """Two readers on one bus is a programming mistake whose symptom looks exactly
    like bad wiring, so it is named -- once per bus, not once per cycle, because a
    tool run against a live daemon would otherwise fill the log."""
    root = tmp_path / "w1"
    bus = _make_bus(root, "w1_bus_master1")
    _make_slave(bus, "28-000000000001", "20000")
    _patch_trigger(monkeypatch, bus / "therm_bulk_read", ["0", "1"])
    src = W1Source({"a": "28-000000000001"}, max_age_s=10.0, root=root, clock=_FakeClock())

    with caplog.at_level("WARNING"):
        src._log_contention("w1_bus_master1")
        src._log_contention("w1_bus_master1")

    assert caplog.text.count("two read cycles overlapped") == 1
    assert "w1_bus_master1" in caplog.text


def test_two_bus_masters_run_their_cycles_at_the_same_time(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """The lock is per bus master, which is the whole point of a thread per bus: two
    buses are two independent 1-Wire wires and the kernel serialises each on its
    own ``bus_mutex``. A global lock would halve the throughput of a two-bus board
    for nothing."""
    root = tmp_path / "w1"
    bus1 = _make_bus(root, "w1_bus_master1")
    bus2 = _make_bus(root, "w1_bus_master2")
    _make_slave(bus1, "28-000000000001", "20000")
    _make_slave(bus2, "28-000000000002", "21000")
    _patch_trigger(monkeypatch, bus1 / "therm_bulk_read", ["0", "1"])
    src = W1Source(
        {"a": "28-000000000001", "b": "28-000000000002"},
        max_age_s=10.0,
        root=root,
        clock=_FakeClock(),
    )
    inside, release, _order = _hold_one_read(src, monkeypatch, "28-000000000001")

    held = threading.Thread(target=lambda: src.run_bus_cycle(bus1))
    held.start()
    assert inside.wait(5.0)
    try:
        # bus2's cycle runs to completion here, with bus1's cycle held open.
        assert src.run_bus_cycle(bus2) == {"b": pytest.approx(21.0)}, (
            "a cycle on one bus master waited for a cycle on another one"
        )
    finally:
        release.set()
        held.join(timeout=5.0)
    assert not held.is_alive()


# -- the read path's keys, in both example configs ----------------------------------

#: Every ``onewire:`` key that chooses a read path, with the one default the code
#: uses for it. Both example configs have to account for all of them (the DAS one
#: by carrying them at these values, the legacy one by naming them in the comment
#: over its placeholder section), and this list is the gate: a new tunable that
#: nobody documented fails here.
READ_PATH_DEFAULTS: dict[str, object] = {
    "read_tier": "auto",
    "netlink_timeout_s": 1.0,
    "netlink_retry_s": 300.0,
    "tier_failures_before_demote": 3,
    "bulk_read": "auto",
    "bulk_timeout_s": 2.0,
    "bulk_retry_s": 300.0,
    "poll_interval_s": 0.02,
    "resolution_bits": 12,
    "slow_cycle_log_interval_s": 60.0,
}


def test_the_das_example_shows_every_read_path_key_at_its_default() -> None:
    """Every operator tunable is a documented key with one default, and the example
    the owner's enclosure runs on shows all of them at it."""
    section = load_config(EXAMPLE_DAS_CONFIG).section("onewire")
    for key, default in READ_PATH_DEFAULTS.items():
        assert key in section, f"config.example-das.yaml does not document onewire.{key}"
        if isinstance(default, float):
            assert section[key] == pytest.approx(default), key
        else:
            assert section[key] == default, key
    built = build_onewire_from_config(section, default_max_age_s=7.5)
    assert built is not None
    assert built._read_tier == READ_PATH_DEFAULTS["read_tier"]
    assert built._netlink_timeout_s == pytest.approx(READ_PATH_DEFAULTS["netlink_timeout_s"])
    assert built._netlink_retry_s == pytest.approx(READ_PATH_DEFAULTS["netlink_retry_s"])


#: The step one LSB is worth at each resolution -- what ``sensors.<name>.quant_c``
#: has to say about a DS18B20 the daemon writes that resolution to (PROJECT.md
#: section 3, the ``onewire:`` block; section 8 item 39).
QUANT_C_PER_RESOLUTION: dict[int, float] = {12: 0.0625, 11: 0.125, 10: 0.25, 9: 0.5}


def test_the_das_example_quantises_its_1_wire_sensors_at_the_resolution_it_asks_for() -> None:
    """Nothing cross-checks ``onewire.resolution_bits`` against
    ``mpc.sensors.<name>.quant_c`` at runtime -- the sections are validated apart
    and a mismatch only mis-sizes ``R`` and the Stuck band -- so the example the
    owner's enclosure runs on is where the two are held together (PROJECT.md
    section 8 item 39: whoever moves one moves the other)."""
    cfg = load_config(EXAMPLE_DAS_CONFIG)
    bits = cfg.onewire["resolution_bits"]
    expected = QUANT_C_PER_RESOLUTION[bits]
    for name in cfg.onewire["sensors"]:
        params = cfg.mpc.sensors[name]
        assert params.quant_c == pytest.approx(expected), (
            f"{name} is a DS18B20 at {bits} bit (one LSB = {expected} C) "
            f"but quant_c says {params.quant_c}"
        )
        # The Stuck band is 1.5 * quant_c: below one LSB it would fire on a
        # sensor idling on one code instead of being cleared by its own dither.
        assert params.stuck_eps_c is not None
        assert params.stuck_eps_c > expected


def test_the_legacy_example_names_the_read_path_keys_it_leaves_at_their_defaults() -> None:
    """The legacy single-controller config has no 1-Wire sensors, so its section is a
    placeholder -- but a reader still has to be able to find out what the defaults
    are without reading the source."""
    text = EXAMPLE_CONFIG.read_text()
    section = load_config(EXAMPLE_CONFIG).section("onewire")
    assert not section.get("sensors")
    for key in READ_PATH_DEFAULTS:
        assert key in text, f"config.example.yaml does not name onewire.{key}"


def test_a_config_written_before_these_keys_existed_still_loads(tmp_path: Path) -> None:
    """A new key must never refuse an older config: the ladder's default is the same
    behaviour as before on a bus the netlink tier cannot serve."""
    src = build_onewire_from_config(
        {"resolution_bits": 12, "max_age_s": 7.5, "sensors": {"a": "28-000000000001"}},
        default_max_age_s=7.5,
    )
    assert src is not None
    assert src._read_tier == "auto"
