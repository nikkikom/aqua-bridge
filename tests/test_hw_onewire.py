"""Tests for aqua_bridge.hw.onewire against a fake w1 sysfs tree (no hardware).

PROJECT.md section 3 (Track B) / the DAS plan section 1 "Read timing vs dt".
"""

from __future__ import annotations

import time
from pathlib import Path

import pytest

from aqua_bridge.hw.onewire import W1Source, build_onewire_from_config
from aqua_bridge.model import ConfigError


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


def _make_slave(bus: Path, rom: str, milli: str | None, *, resolution: str | None = None) -> Path:
    slave = bus / rom
    slave.mkdir(parents=True)
    if milli is not None:
        (slave / "temperature").write_text(milli)
    if resolution is not None:
        (slave / "resolution").write_text(resolution)
    return slave


def _patch_trigger(
    monkeypatch: pytest.MonkeyPatch, trigger_path: Path, statuses: list[str]
) -> None:
    """Makes ``trigger_path.read_text()`` return ``statuses`` in order (repeating
    the last entry once exhausted) and swallows writes to it.

    A real ``therm_bulk_read`` attribute is a kernel command/status channel,
    not a byte store: writing "trigger" does not make a later read return
    "trigger" back. A plain file can't reproduce that on its own, so this
    patches only reads/writes of this one path (everything else -- the
    ``temperature``/``resolution`` files -- goes through the real
    filesystem unpatched).
    """
    original_write = Path.write_text
    original_read = Path.read_text
    state = {"n": 0}

    def fake_write(self: Path, data: str, *a: object, **kw: object) -> int:
        if self == trigger_path:
            return len(data)
        return original_write(self, data, *a, **kw)

    def fake_read(self: Path, *a: object, **kw: object) -> str:
        if self == trigger_path:
            idx = min(state["n"], len(statuses) - 1)
            state["n"] += 1
            return statuses[idx]
        return original_read(self, *a, **kw)

    monkeypatch.setattr(Path, "write_text", fake_write)
    monkeypatch.setattr(Path, "read_text", fake_read)


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


# --- CRC failure -> None -------------------------------------------------------------


def test_bad_temperature_content_is_none_and_counted_as_crc_error(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    root = tmp_path / "w1"
    bus = _make_bus(root, "w1_bus_master1")
    _make_slave(bus, "28-000000000001", "not-a-number")
    _patch_trigger(monkeypatch, bus / "therm_bulk_read", ["1"])

    src = W1Source({"a": "28-000000000001"}, max_age_s=10.0, root=root, clock=_FakeClock())
    result = src.run_bus_cycle(bus)

    assert result == {"a": None}
    assert src.crc_error_counts() == {"28-000000000001": 1}


def test_missing_temperature_file_is_none_crc_error(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    root = tmp_path / "w1"
    bus = _make_bus(root, "w1_bus_master1")
    _make_slave(bus, "28-000000000001", None)  # no temperature file: kernel EIO equivalent
    _patch_trigger(monkeypatch, bus / "therm_bulk_read", ["1"])

    src = W1Source({"a": "28-000000000001"}, max_age_s=10.0, root=root, clock=_FakeClock())
    result = src.run_bus_cycle(bus)

    assert result == {"a": None}
    assert src.crc_error_counts()["28-000000000001"] == 1


def test_bulk_read_never_completing_reports_none_without_raising(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    root = tmp_path / "w1"
    bus = _make_bus(root, "w1_bus_master1")
    _make_slave(bus, "28-000000000001", "20000")
    _patch_trigger(monkeypatch, bus / "therm_bulk_read", ["0", "-1", "-1"])  # never "1"

    src = W1Source(
        {"a": "28-000000000001"},
        max_age_s=10.0,
        root=root,
        clock=time.monotonic,  # a real, advancing clock so the deadline is actually reached
        poll_interval_s=0.005,
        bulk_timeout_s=0.03,
    )
    result = src.run_bus_cycle(bus)
    assert result == {"a": None}


def test_bulk_trigger_write_failure_is_none_without_raising(tmp_path: Path) -> None:
    root = tmp_path / "w1"
    bus = _make_bus(root, "w1_bus_master1")
    _make_slave(bus, "28-000000000001", "20000")
    (bus / "therm_bulk_read").unlink()
    (bus / "therm_bulk_read").mkdir()  # writing to it now raises IsADirectoryError

    src = W1Source({"a": "28-000000000001"}, max_age_s=10.0, root=root, clock=_FakeClock())
    assert src.run_bus_cycle(bus) == {"a": None}


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
    ],
)
def test_build_onewire_malformed_section_is_config_error(section: object, match: str) -> None:
    with pytest.raises(ConfigError, match=match):
        build_onewire_from_config(section, default_max_age_s=7.5)  # type: ignore[arg-type]
