"""Tests for tools/w1_commission.py against a fake w1 sysfs tree (no hardware).

PROJECT.md section 3 (Track B) / the DAS plan section 10 "How to run".
``tools/`` is not on ``pythonpath`` (only ``src``/``tests`` are, per
``pyproject.toml``), so this file adds it to ``sys.path`` itself, the same
way a standalone script is expected to be run.
"""

from __future__ import annotations

import sys
from pathlib import Path

import pytest
import yaml

from aqua_bridge.hw.onewire import W1Source

TOOLS_DIR = Path(__file__).resolve().parent.parent / "tools"
if str(TOOLS_DIR) not in sys.path:
    sys.path.insert(0, str(TOOLS_DIR))

import w1_commission  # noqa: E402 -- must follow the sys.path tweak above

REPO_ROOT = Path(__file__).resolve().parent.parent
EXAMPLE_CONFIG = REPO_ROOT / "config.example.yaml"


def _make_bus(root: Path, bus_name: str) -> Path:
    bus = root / bus_name
    bus.mkdir(parents=True)
    (bus / "therm_bulk_read").write_text("1")
    return bus


def _make_slave(bus: Path, rom: str, milli: str) -> Path:
    slave = bus / rom
    slave.mkdir(parents=True)
    (slave / "temperature").write_text(milli)
    return slave


def _make_trigger_always_done(monkeypatch: pytest.MonkeyPatch, trigger_path: Path) -> None:
    """A real ``therm_bulk_read`` file is a kernel status channel: writing
    "trigger" does not change what a later read returns. A plain file
    can't do that on its own, so writes to this one path are swallowed,
    leaving its pre-seeded "1" content (bulk read already done) in place.
    """
    trigger_path.write_text("1")
    original_write = Path.write_text

    def fake_write(self: Path, data: str, *a: object, **kw: object) -> int:
        if self == trigger_path:
            return len(data)
        return original_write(self, data, *a, **kw)

    monkeypatch.setattr(Path, "write_text", fake_write)


# --- discover_all / rank_by_warming_rate (pure) ------------------------------------


def test_discover_all_lists_rom_shaped_subdirs_only(tmp_path: Path) -> None:
    root = tmp_path / "w1"
    bus = _make_bus(root, "w1_bus_master1")
    _make_slave(bus, "28-000000000001", "20000")
    (bus / "not_a_rom").mkdir()

    found = w1_commission.discover_all(root)
    assert found == {"w1_bus_master1": ["28-000000000001"]}


def test_discover_all_skips_the_phantoms_of_an_unterminated_bus(tmp_path: Path) -> None:
    """The kernel's periodic search reads family-00 garbage off an unterminated
    bus (PROJECT.md section 8 item 38: three of them, a different set every
    search, on the board's unwired second bus). Those are ROM-shaped and would
    otherwise be offered as sensors to bind."""
    root = tmp_path / "w1"
    wired = _make_bus(root, "w1_bus_master2")
    _make_slave(wired, "28-000000000001", "20000")
    unwired = root / "w1_bus_master1"
    unwired.mkdir(parents=True)
    (unwired / "00-b00000000000").mkdir()
    (unwired / "00-700000000000").mkdir()

    assert w1_commission.discover_all(root) == {
        "w1_bus_master1": [],
        "w1_bus_master2": ["28-000000000001"],
    }
    assert w1_commission.discover_other_families(root) == {
        "w1_bus_master1": ["00-700000000000", "00-b00000000000"]
    }


def test_cmd_list_names_the_phantom_family_and_what_it_means(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    root = tmp_path / "w1"
    bus = _make_bus(root, "w1_bus_master2")
    _make_slave(bus, "28-000000000001", "20000")
    _make_trigger_always_done(monkeypatch, bus / "therm_bulk_read")
    unwired = root / "w1_bus_master1"
    unwired.mkdir(parents=True)
    (unwired / "00-b00000000000").mkdir()

    assert w1_commission.cmd_list(root) == 0
    out = capsys.readouterr().out
    assert "28-000000000001" in out
    assert "00-b00000000000" in out and "another family" in out
    assert "unterminated" in out


def test_discover_all_missing_root_is_empty(tmp_path: Path) -> None:
    assert w1_commission.discover_all(tmp_path / "nope") == {}


def test_rank_by_warming_rate_orders_fastest_first() -> None:
    series = {
        "slow": [20.0, 20.1, 20.2],
        "fast": [20.0, 22.0, 25.0],
        "flat": [20.0, 20.0, 20.0],
        "too_short": [20.0],
        "had_a_gap": [20.0, None, 21.0],
    }
    ranked = w1_commission.rank_by_warming_rate(series)
    names = [name for name, _ in ranked]
    assert names[0] == "fast"
    assert "too_short" not in names
    assert ("had_a_gap", pytest.approx(1.0)) in ranked


# --- cmd_list ------------------------------------------------------------------------


def test_cmd_list_prints_readings(
    tmp_path: Path, capsys: pytest.CaptureFixture[str], monkeypatch: pytest.MonkeyPatch
) -> None:
    root = tmp_path / "w1"
    bus = _make_bus(root, "w1_bus_master1")
    _make_slave(bus, "28-000000000001", "23562")
    _make_trigger_always_done(monkeypatch, bus / "therm_bulk_read")

    rc = w1_commission.cmd_list(root)

    assert rc == 0
    out = capsys.readouterr().out
    assert "w1_bus_master1" in out
    assert "28-000000000001" in out
    assert "23.562" in out


def test_cmd_list_no_bus_master(tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    rc = w1_commission.cmd_list(tmp_path / "nope")
    assert rc == 1
    assert "no w1 bus master" in capsys.readouterr().out


def test_cmd_list_bus_with_no_roms(tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    root = tmp_path / "w1"
    _make_bus(root, "w1_bus_master1")
    rc = w1_commission.cmd_list(root)
    assert rc == 1
    assert "no DS18B20" in capsys.readouterr().out


# --- cmd_check -----------------------------------------------------------------------


def _example_mpc_and_xt6() -> dict:
    """The example config's sections; building the composite opens no controller."""
    return yaml.safe_load(EXAMPLE_CONFIG.read_text())


def test_cmd_check_reports_cycle_time_and_per_sensor_read_outcomes(
    tmp_path: Path, capsys: pytest.CaptureFixture[str], monkeypatch: pytest.MonkeyPatch
) -> None:
    data = _example_mpc_and_xt6()
    w1_root = tmp_path / "w1"
    bus = _make_bus(w1_root, "w1_bus_master1")
    _make_slave(bus, "28-000000000001", "21000")
    _make_trigger_always_done(monkeypatch, bus / "therm_bulk_read")
    data["mpc"]["temps"] = ["coolant", "air", "prox_b01"]
    data["onewire"] = {"sensors": {"prox_b01": "28-000000000001"}, "root": str(w1_root)}
    config_path = tmp_path / "config.yaml"
    config_path.write_text(yaml.safe_dump(data))

    rc = w1_commission.cmd_check(str(config_path), cycles=3)

    assert rc == 0
    out = capsys.readouterr().out
    assert "binding check" in out
    assert "w1_bus_master1" in out
    assert "ms/cycle over 3 cycles" in out
    # The tier the number was measured on, without which the number cannot be
    # read at all (PROJECT.md section 8 item 38). Neither fast tier exists
    # under a fake tree: no netlink connector in the suite, and this fake's
    # therm_bulk_read is a plain file, so the bus ends up reading one at a time.
    assert "(serial reads)" in out
    # Every sensor's reads against the attempts that sensor actually got -- three
    # cycles on the one bus it lives on, not cycles * len(buses) (PROJECT.md item 39).
    assert "28-000000000001: 0/3 reads failed (0.0%)" in out


def test_cmd_check_is_the_only_reader_on_the_bus_while_it_measures(
    tmp_path: Path, capsys: pytest.CaptureFixture[str], monkeypatch: pytest.MonkeyPatch
) -> None:
    """The defect of PROJECT.md item 39: this command used to build the composite
    the daemon builds -- which starts a reader thread per bus -- and then drive
    cycles itself, so every measurement was taken with two cycles overlapping on
    one bus master. On the board that read 6.4 s per cycle against 1.0 s and lost
    a third of the reads. The binding check still happens; the reader threads do
    not start.
    """
    data = _example_mpc_and_xt6()
    w1_root = tmp_path / "w1"
    bus = _make_bus(w1_root, "w1_bus_master1")
    _make_slave(bus, "28-000000000001", "21000")
    _make_trigger_always_done(monkeypatch, bus / "therm_bulk_read")
    data["mpc"]["temps"] = ["coolant", "air", "prox_b01"]
    data["onewire"] = {"sensors": {"prox_b01": "28-000000000001"}, "root": str(w1_root)}
    config_path = tmp_path / "config.yaml"
    config_path.write_text(yaml.safe_dump(data))

    started: list[str] = []
    monkeypatch.setattr(W1Source, "start", lambda self: started.append("start"))

    rc = w1_commission.cmd_check(str(config_path), cycles=2)

    assert rc == 0
    assert started == [], "--check started the daemon's reader threads and then read the bus too"
    out = capsys.readouterr().out
    assert "binding check" in out, "the binding check is the other half of what --check is for"
    assert "ms/cycle over 2 cycles" in out


def test_cmd_check_does_not_report_a_rate_for_a_rom_that_is_on_no_bus(
    tmp_path: Path, capsys: pytest.CaptureFixture[str], monkeypatch: pytest.MonkeyPatch
) -> None:
    """A placeholder ROM id used to print ``0/40 failed reads (0.0%)`` -- the most
    reassuring line in the report, about a sensor nothing had ever asked anything
    of (PROJECT.md item 39). It has no attempts, so it gets no rate.
    """
    data = _example_mpc_and_xt6()
    w1_root = tmp_path / "w1"
    bus = _make_bus(w1_root, "w1_bus_master1")
    _make_slave(bus, "28-000000000001", "21000")
    _make_trigger_always_done(monkeypatch, bus / "therm_bulk_read")
    data["mpc"]["temps"] = ["coolant", "air", "prox_b01", "prox_b02"]
    data["onewire"] = {
        "sensors": {"prox_b01": "28-000000000001", "prox_b02": "28-00000000ffff"},
        "root": str(w1_root),
    }
    config_path = tmp_path / "config.yaml"
    config_path.write_text(yaml.safe_dump(data))

    rc = w1_commission.cmd_check(str(config_path), cycles=2)

    assert rc == 0
    out = capsys.readouterr().out
    assert "28-00000000ffff: no read attempted (not under any bus master)" in out
    assert "28-00000000ffff: 0/" not in out
    assert "28-000000000001: 0/2 reads failed (0.0%)" in out


def test_cmd_check_tells_an_empty_read_from_a_failed_one(
    tmp_path: Path, capsys: pytest.CaptureFixture[str], monkeypatch: pytest.MonkeyPatch
) -> None:
    """The three ways a read produces no number are three different faults, and one
    of them -- a read that answers nothing at all -- means a second reader was
    converting on this bus. Printing them as one "failed reads" number hid the
    difference that mattered (PROJECT.md item 39).
    """
    data = _example_mpc_and_xt6()
    w1_root = tmp_path / "w1"
    bus = _make_bus(w1_root, "w1_bus_master1")
    _make_slave(bus, "28-000000000001", "")  # answers nothing: a conversion is in flight
    _make_slave(bus, "28-000000000002", "not-a-number")  # answers something impossible
    _make_trigger_always_done(monkeypatch, bus / "therm_bulk_read")
    data["mpc"]["temps"] = ["coolant", "air", "prox_b01", "prox_b02"]
    data["onewire"] = {
        "sensors": {"prox_b01": "28-000000000001", "prox_b02": "28-000000000002"},
        "root": str(w1_root),
    }
    config_path = tmp_path / "config.yaml"
    config_path.write_text(yaml.safe_dump(data))

    rc = w1_commission.cmd_check(str(config_path), cycles=2)

    assert rc == 0
    out = capsys.readouterr().out
    assert "28-000000000001: 2/2 reads failed (100.0%) [2 empty]  WARNING: > 1%" in out
    assert "28-000000000002: 2/2 reads failed (100.0%) [2 rejected]  WARNING: > 1%" in out
    assert "answered nothing at all" in out and "stop the daemon" in out


def test_cmd_check_no_onewire_sensors_is_fine(
    tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    data = _example_mpc_and_xt6()
    config_path = tmp_path / "config.yaml"
    config_path.write_text(yaml.safe_dump(data))

    rc = w1_commission.cmd_check(str(config_path))

    assert rc == 0
    assert "nothing more to check" in capsys.readouterr().out


def test_cmd_check_missing_rom_is_a_warning_not_a_failure(
    tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    data = _example_mpc_and_xt6()
    data["mpc"]["temps"] = ["coolant", "air", "prox_b01"]
    data["onewire"] = {"sensors": {"prox_b01": "28-absent"}, "root": str(tmp_path / "w1_empty")}
    config_path = tmp_path / "config.yaml"
    config_path.write_text(yaml.safe_dump(data))

    rc = w1_commission.cmd_check(str(config_path), cycles=1)

    assert rc == 0
    assert "WARNING" in capsys.readouterr().out


def test_cmd_check_binding_error_exits_2(
    tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    data = _example_mpc_and_xt6()
    data["mpc"]["temps"] = ["coolant", "air", "unbound_temp"]
    config_path = tmp_path / "config.yaml"
    config_path.write_text(yaml.safe_dump(data))

    rc = w1_commission.cmd_check(str(config_path))

    assert rc == 2
    assert "config error" in capsys.readouterr().err


def test_cmd_check_bad_config_path_exits_2(capsys: pytest.CaptureFixture[str]) -> None:
    rc = w1_commission.cmd_check("/nonexistent/config.yaml")
    assert rc == 2
    assert "config error" in capsys.readouterr().err


# --- CLI parsing -----------------------------------------------------------------------


def test_parser_requires_exactly_one_mode() -> None:
    with pytest.raises(SystemExit):
        w1_commission.build_parser().parse_args([])
    with pytest.raises(SystemExit):
        w1_commission.build_parser().parse_args(["--list", "--identify"])


def test_main_check_without_config_exits_2(capsys: pytest.CaptureFixture[str]) -> None:
    rc = w1_commission.main(["--check"])
    assert rc == 2
    assert "requires --config" in capsys.readouterr().err


def test_main_list_dispatches(tmp_path: Path) -> None:
    rc = w1_commission.main(["--list", "--root", str(tmp_path / "nope")])
    assert rc == 1
