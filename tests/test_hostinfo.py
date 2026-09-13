"""Tests for aqua_bridge.hostinfo against fake sysfs/procfs trees."""

from __future__ import annotations

from pathlib import Path

import pytest

from aqua_bridge.hostinfo import (
    collect_hostinfo,
    read_cpu_temp_c,
    read_disk,
    read_loadavg,
    read_memory,
    read_uptime_s,
    read_wifi_rssi,
)


def _write(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text)


# --- read_cpu_temp_c ---------------------------------------------------


def test_cpu_temp_prefers_zone_named_cpu(tmp_path: Path) -> None:
    root = tmp_path / "thermal"
    _write(root / "thermal_zone0" / "type", "gpu-thermal\n")
    _write(root / "thermal_zone0" / "temp", "50000\n")
    _write(root / "thermal_zone1" / "type", "cpu-thermal\n")
    _write(root / "thermal_zone1" / "temp", "42500\n")
    assert read_cpu_temp_c(root) == pytest.approx(42.5)


def test_cpu_temp_falls_back_to_first_zone(tmp_path: Path) -> None:
    root = tmp_path / "thermal"
    _write(root / "thermal_zone0" / "type", "soc-thermal\n")
    _write(root / "thermal_zone0" / "temp", "39000\n")
    assert read_cpu_temp_c(root) == pytest.approx(39.0)


def test_cpu_temp_missing_dir_is_none(tmp_path: Path) -> None:
    assert read_cpu_temp_c(tmp_path / "nope") is None


def test_cpu_temp_garbage_value_is_none(tmp_path: Path) -> None:
    root = tmp_path / "thermal"
    _write(root / "thermal_zone0" / "type", "cpu-thermal\n")
    _write(root / "thermal_zone0" / "temp", "not-a-number\n")
    assert read_cpu_temp_c(root) is None


# --- read_loadavg --------------------------------------------------------


def test_loadavg_parses(tmp_path: Path) -> None:
    p = tmp_path / "loadavg"
    _write(p, "0.10 0.05 0.01 1/234 5678\n")
    assert read_loadavg(p) == pytest.approx((0.10, 0.05, 0.01))


def test_loadavg_missing_is_none(tmp_path: Path) -> None:
    assert read_loadavg(tmp_path / "nope") is None


def test_loadavg_malformed_is_none(tmp_path: Path) -> None:
    p = tmp_path / "loadavg"
    _write(p, "garbage\n")
    assert read_loadavg(p) is None


# --- read_memory ----------------------------------------------------------


def test_memory_parses(tmp_path: Path) -> None:
    p = tmp_path / "meminfo"
    _write(
        p,
        "MemTotal:        1000000 kB\nMemFree:          200000 kB\nMemAvailable:     400000 kB\n",
    )
    mem = read_memory(p)
    assert mem is not None
    assert mem["total_kb"] == pytest.approx(1_000_000)
    assert mem["available_kb"] == pytest.approx(400_000)
    assert mem["used_pct"] == pytest.approx(60.0)


def test_memory_missing_available_is_none(tmp_path: Path) -> None:
    p = tmp_path / "meminfo"
    _write(p, "MemTotal:        1000000 kB\n")
    assert read_memory(p) is None


def test_memory_missing_file_is_none(tmp_path: Path) -> None:
    assert read_memory(tmp_path / "nope") is None


# --- read_uptime_s ----------------------------------------------------------


def test_uptime_parses(tmp_path: Path) -> None:
    p = tmp_path / "uptime"
    _write(p, "12345.67 999.0\n")
    assert read_uptime_s(p) == pytest.approx(12345.67)


def test_uptime_missing_is_none(tmp_path: Path) -> None:
    assert read_uptime_s(tmp_path / "nope") is None


# --- read_disk --------------------------------------------------------------


def test_disk_parses_real_root() -> None:
    # "/" always exists; just check the shape and sane ranges.
    disk = read_disk("/")
    assert disk is not None
    assert 0.0 <= disk["used_pct"] <= 100.0
    assert disk["total_gb"] > 0
    assert disk["free_gb"] >= 0


def test_disk_missing_path_is_none(tmp_path: Path) -> None:
    assert read_disk(tmp_path / "does" / "not" / "exist") is None


# --- read_wifi_rssi ----------------------------------------------------------


_WIRELESS_SAMPLE = (
    "Inter-| sta-|   Quality        |   Discarded packets               | Missed | WE\n"
    " face | tus | link level noise |  nwid  crypt   frag  retry   misc | beacon | 22\n"
    " wlan0: 0000   50.  -60.  -256        0      0      0      0      0        0\n"
)


def test_wifi_rssi_parses_first_interface(tmp_path: Path) -> None:
    p = tmp_path / "wireless"
    _write(p, _WIRELESS_SAMPLE)
    assert read_wifi_rssi(p) == pytest.approx(-60.0)


def test_wifi_rssi_selects_named_interface(tmp_path: Path) -> None:
    p = tmp_path / "wireless"
    extra_line = " wlan1: 0000   30.  -80.  -256        0      0      0      0      0        0\n"
    _write(p, _WIRELESS_SAMPLE + extra_line)
    assert read_wifi_rssi(p, iface="wlan1") == pytest.approx(-80.0)


def test_wifi_rssi_missing_file_is_none(tmp_path: Path) -> None:
    assert read_wifi_rssi(tmp_path / "nope") is None


def test_wifi_rssi_no_interfaces_is_none(tmp_path: Path) -> None:
    p = tmp_path / "wireless"
    _write(p, _WIRELESS_SAMPLE.splitlines()[0] + "\n" + _WIRELESS_SAMPLE.splitlines()[1] + "\n")
    assert read_wifi_rssi(p) is None


# --- collect_hostinfo --------------------------------------------------------


def test_collect_hostinfo_all_missing_is_all_none(tmp_path: Path) -> None:
    info = collect_hostinfo(
        thermal_root=tmp_path / "thermal",
        loadavg_path=tmp_path / "loadavg",
        meminfo_path=tmp_path / "meminfo",
        uptime_path=tmp_path / "uptime",
        disk_path=tmp_path / "no" / "such" / "path",
        wireless_path=tmp_path / "wireless",
    )
    assert info == {
        "cpu_temp_c": None,
        "load1": None,
        "load5": None,
        "load15": None,
        "mem_used_pct": None,
        "mem_total_kb": None,
        "disk_used_pct": None,
        "disk_free_gb": None,
        "wifi_rssi_dbm": None,
        "uptime_s": None,
    }


def test_collect_hostinfo_all_present(tmp_path: Path) -> None:
    _write(tmp_path / "thermal" / "thermal_zone0" / "type", "cpu-thermal\n")
    _write(tmp_path / "thermal" / "thermal_zone0" / "temp", "45000\n")
    _write(tmp_path / "loadavg", "1.0 2.0 3.0 1/1 1\n")
    _write(
        tmp_path / "meminfo",
        "MemTotal:        1000000 kB\nMemAvailable:     500000 kB\n",
    )
    _write(tmp_path / "uptime", "100.0 0.0\n")
    _write(tmp_path / "wireless", _WIRELESS_SAMPLE)

    info = collect_hostinfo(
        thermal_root=tmp_path / "thermal",
        loadavg_path=tmp_path / "loadavg",
        meminfo_path=tmp_path / "meminfo",
        uptime_path=tmp_path / "uptime",
        disk_path="/",
        wireless_path=tmp_path / "wireless",
    )
    assert info["cpu_temp_c"] == pytest.approx(45.0)
    assert info["load1"] == pytest.approx(1.0)
    assert info["load5"] == pytest.approx(2.0)
    assert info["load15"] == pytest.approx(3.0)
    assert info["mem_used_pct"] == pytest.approx(50.0)
    assert info["uptime_s"] == pytest.approx(100.0)
    assert info["wifi_rssi_dbm"] == pytest.approx(-60.0)
    assert info["disk_used_pct"] is not None
