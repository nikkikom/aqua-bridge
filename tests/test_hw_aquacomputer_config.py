"""Tests for the device entry of ``xt6:`` / ``aquacomputer:`` (aqua_bridge.hw.aquacomputer_adapter).

Startup cross-check against the mpc section (review finding F4), channel ranges
per device kind, timing keys and the hints for keys of the hwmon era.
PROJECT.md section 3 (Track B).
"""

from __future__ import annotations

import pytest
import yaml

from aqua_bridge.hw.aquacomputer import AQUAERO, QUADRO
from aqua_bridge.hw.aquacomputer_adapter import (
    ENTRY_KEYS,
    TIMING_KEYS,
    AquacomputerAdapter,
    AquacomputerTiming,
    build_adapter_from_config,
    parse_device_section,
)
from aqua_bridge.model import ConfigError, Mode, MpcCommand
from aquacomputer_fakes import FakeBus, FakeClock, FakeController

_SECTION = {
    "device": "aquaero",
    "fans": {
        "radiator": {"pwm": "pwm1", "rpm": "fan1"},
        "intake": {"pwm": "pwm2"},
    },
    "temp_map": {"coolant": "temp1", "air": "temp2"},
}
_CHANNELS = ("radiator", "intake")
_TEMPS = ("coolant", "air")


def _parse(section, **kwargs):
    return parse_device_section(section, label="xt6", ignored_keys=("prefer",), **kwargs)


def test_matching_mpc_section_is_accepted() -> None:
    binding = _parse(_SECTION, channels=_CHANNELS, temps=_TEMPS)
    assert binding.kind is AQUAERO and binding.serial is None
    assert binding.pwm_map == {"radiator": 1, "intake": 2}
    assert binding.fan_map == {"radiator": 1}  # rpm is optional per fan
    assert binding.temp_map == {"coolant": 1, "air": 2}
    assert binding.timing == AquacomputerTiming()
    # without the mpc tuples the check is skipped (mapping-only callers)
    assert _parse(_SECTION).pwm_map == binding.pwm_map


def test_one_line_yaml_flow_mappings_and_every_optional_key() -> None:
    section = yaml.safe_load(
        """
device: quadro
serial: 12345-67890
prefer: hid
fans:
  radiator: {pwm: pwm4, rpm: fan5}
  intake:   {pwm: pwm2, rpm: fan2}
temp_map: {coolant: temp20, air: temp5}
status_max_age_s: 2.5
ctrl_gap_ms: 150
ctrl_retries: 3
ctrl_refresh_s: 0
duty_mismatch_tolerance: 250
duty_mismatch_s: 12
"""
    )
    binding = _parse(section, channels=_CHANNELS, temps=_TEMPS)
    assert binding.kind is QUADRO and binding.serial == "12345-67890"
    assert binding.pwm_map == {"radiator": 4, "intake": 2}
    assert binding.fan_map == {"radiator": 5, "intake": 2}
    assert binding.temp_map == {"coolant": 20, "air": 5}
    assert binding.timing == AquacomputerTiming(
        status_max_age_s=2.5,
        ctrl_gap_ms=150,
        ctrl_retries=3,
        ctrl_refresh_s=0,
        duty_mismatch_tolerance=250,
        duty_mismatch_s=12,
    )
    assert set(TIMING_KEYS) <= set(ENTRY_KEYS)


def test_build_adapter_from_config_opens_nothing() -> None:
    clock = FakeClock()
    device = FakeController(AQUAERO, clock)
    bus = FakeBus(device)
    adapter = build_adapter_from_config(
        dict(_SECTION, prefer="hwmon"), channels=_CHANNELS, temps=_TEMPS, clock=clock, opener=bus
    )
    assert isinstance(adapter, AquacomputerAdapter) and bus.opened == []
    adapter.apply(MpcCommand(pwm={"radiator": 0.5, "intake": 0.5}, mode=Mode.AUTO))
    assert bus.opened == [device.node]


@pytest.mark.parametrize(
    ("key", "match"),
    [
        ("hwmon_name", r"xt6.hwmon_name is no longer supported.*hidraw.*'device: aquaero'"),
        ("root", r"xt6.root is no longer supported.*/sys/class/hidraw.*serial"),
        ("name", r"xt6.name was renamed to 'device'"),
    ],
)
def test_hwmon_era_keys_are_rejected_with_a_hint(key: str, match: str) -> None:
    section = {**_SECTION, key: "/sys/class/hwmon" if key == "root" else "aquaero"}
    with pytest.raises(ConfigError, match=match):
        _parse(section)


@pytest.mark.parametrize("legacy", ["map", "fan_map"])
def test_legacy_map_keys_are_rejected_with_a_hint(legacy: str) -> None:
    section = dict(_SECTION, **{legacy: {"radiator": "pwm1"}})
    with pytest.raises(ConfigError, match=f"xt6.{legacy} is no longer supported.*xt6.fans"):
        _parse(section, channels=_CHANNELS, temps=_TEMPS)


def test_unknown_key_is_rejected_so_a_misspelt_timing_key_cannot_hide() -> None:
    with pytest.raises(ConfigError, match=r"unknown key\(s\) \['ctrl_gap_s'\]"):
        _parse(dict(_SECTION, ctrl_gap_s=0.2))
    with pytest.raises(ConfigError, match=r"unknown key\(s\) \['prefer'\]"):
        parse_device_section(dict(_SECTION, prefer="hid"), label="aquacomputer[0]")


@pytest.mark.parametrize(
    ("section", "match"),
    [
        ({k: v for k, v in _SECTION.items() if k != "device"}, r"xt6.device is required"),
        (dict(_SECTION, device="octo"), r"xt6.device must be one of \['aquaero', 'quadro'\]"),
        (dict(_SECTION, device=1), "xt6.device must be one of"),
        (dict(_SECTION, serial=12345), "xt6.serial must be a non-empty string"),
        (dict(_SECTION, serial=" "), "xt6.serial must be a non-empty string"),
        ("not a mapping", "xt6 must be a mapping"),
    ],
    ids=["no_device", "unknown_device", "device_not_string", "serial_int", "serial_blank", "type"],
)
def test_malformed_device_or_serial_is_config_error(section, match: str) -> None:
    with pytest.raises(ConfigError, match=match):
        _parse(section)


@pytest.mark.parametrize(
    ("fans", "match"),
    [
        (None, "xt6.fans is required"),
        (["radiator"], "xt6.fans must be a mapping"),
        ({"radiator": "pwm1", "intake": {"pwm": "pwm2"}}, "xt6.fans.radiator must be a mapping"),
        (
            {"radiator": {"rpm": "fan1"}, "intake": {"pwm": "pwm2"}},
            "xt6.fans.radiator.pwm is required",
        ),
        (
            {"radiator": {"pwm": "pwm1", "rmp": "fan1"}, "intake": {"pwm": "pwm2"}},
            "unknown key\\(s\\) \\['rmp'\\]",
        ),
        (
            {"radiator": {"pwm": "fan1"}, "intake": {"pwm": "pwm2"}},
            "xt6.fans.radiator.pwm must be one of the aquaero's inputs pwm1..pwm4",
        ),
        (
            {"radiator": {"pwm": "pwm1", "rpm": "pwm1"}, "intake": {"pwm": "pwm2"}},
            "xt6.fans.radiator.rpm must be one of the aquaero's inputs fan1..fan6",
        ),
        (
            {"radiator": {"pwm": "pwm1", "rpm": 1}, "intake": {"pwm": "pwm2"}},
            "xt6.fans.radiator.rpm must be",
        ),
        (
            {"radiator": {"pwm": "pwm5"}, "intake": {"pwm": "pwm2"}},
            "pwm1..pwm4, got 'pwm5'",
        ),
        (
            {"radiator": {"pwm": "pwm1", "rpm": "fan7"}, "intake": {"pwm": "pwm2"}},
            "fan1..fan6, got 'fan7'",
        ),
        (
            {"radiator": {"pwm": "pwm1"}, "intake": {"pwm": "pwm1"}},
            "radiator and intake name the same input 'pwm1'",
        ),
        (
            {"radiator": {"pwm": "pwm1", "rpm": "fan1"}, "intake": {"pwm": "pwm2", "rpm": "fan1"}},
            "name the same input 'fan1'",
        ),
    ],
    ids=[
        "missing",
        "not_mapping",
        "entry_is_bare_string",
        "no_pwm",
        "typo_in_key",
        "pwm_names_a_tachometer",
        "rpm_names_a_pwm",
        "rpm_not_string",
        "pwm_out_of_range",
        "fan_out_of_range",
        "two_fans_one_pwm",
        "two_fans_one_tachometer",
    ],
)
def test_malformed_fans_entry_is_config_error(fans, match: str) -> None:
    section = {k: v for k, v in _SECTION.items() if k != "fans"}
    if fans is not None:
        section["fans"] = fans
    with pytest.raises(ConfigError, match=match):
        _parse(section, channels=_CHANNELS, temps=_TEMPS)


def test_quadro_ranges_differ_from_the_aquaero() -> None:
    quadro = dict(_SECTION, device="quadro")
    with pytest.raises(ConfigError, match="quadro's inputs fan1..fan5, got 'fan6'"):
        _parse(dict(quadro, fans={"radiator": {"pwm": "pwm1", "rpm": "fan6"}}))
    with pytest.raises(ConfigError, match="temp1..temp20, got 'temp21'"):
        _parse(dict(quadro, temp_map={"coolant": "temp21"}))
    _parse(dict(_SECTION, fans={"radiator": {"pwm": "pwm1", "rpm": "fan6"}}))


@pytest.mark.parametrize(
    ("temp_map", "match"),
    [
        (["temp1"], "xt6.temp_map must be a mapping"),
        ({"coolant": "fan1", "air": "temp2"}, "xt6.temp_map.coolant must be"),
        ({"coolant": "temp0", "air": "temp2"}, "xt6.temp_map.coolant must be"),
        ({"coolant": "temp1", "air": "temp1"}, "coolant and air name the same input 'temp1'"),
    ],
    ids=["not_mapping", "wrong_role", "zero", "duplicate"],
)
def test_malformed_temp_map_is_config_error(temp_map, match: str) -> None:
    with pytest.raises(ConfigError, match=match):
        _parse(dict(_SECTION, temp_map=temp_map))


def test_channel_missing_from_fans_is_a_startup_error() -> None:
    """A channel in mpc.channels absent from xt6.fans would be silently never written."""
    section = dict(_SECTION, fans={"radiator": {"pwm": "pwm1"}})
    with pytest.raises(ConfigError, match="xt6.fans.*missing \\['intake'\\]"):
        _parse(section, channels=_CHANNELS, temps=_TEMPS)


def test_extra_channel_in_fans_is_a_startup_error() -> None:
    section = dict(_SECTION, fans={**_SECTION["fans"], "pump": {"pwm": "pwm3"}})
    with pytest.raises(ConfigError, match="extra \\['pump'\\]"):
        _parse(section, channels=_CHANNELS, temps=_TEMPS)


def test_misspelt_channel_in_fans_is_a_startup_error() -> None:
    section = dict(
        _SECTION, fans={"radaitor": {"pwm": "pwm1", "rpm": "fan1"}, "intake": {"pwm": "pwm2"}}
    )
    with pytest.raises(ConfigError, match="missing \\['radiator'\\], extra \\['radaitor'\\]"):
        _parse(section, channels=_CHANNELS, temps=_TEMPS)


@pytest.mark.parametrize(
    "temp_map",
    [
        {"coolant": "temp1", "air": "temp2", "ambient": "temp3"},  # extra -> every tick untrusted
        {"coolant": "temp1"},  # missing -> every tick untrusted
    ],
    ids=["extra", "missing"],
)
def test_temp_map_not_equal_to_mpc_temps_is_a_startup_error(temp_map) -> None:
    """Every observation would carry an unknown / missing key: permanent fallback with no
    startup error (section 3 gate rule 2)."""
    with pytest.raises(ConfigError, match="xt6.temp_map"):
        _parse(dict(_SECTION, temp_map=temp_map), channels=_CHANNELS, temps=_TEMPS)


@pytest.mark.parametrize(
    ("key", "value", "match"),
    [
        ("status_max_age_s", 0, "status_max_age_s must be a finite number > 0"),
        ("status_max_age_s", float("inf"), "status_max_age_s must be a finite number > 0"),
        ("status_max_age_s", "3", "status_max_age_s must be a finite number > 0"),
        ("ctrl_gap_ms", -1, "ctrl_gap_ms must be a finite number >= 0"),
        ("ctrl_gap_ms", True, "ctrl_gap_ms must be a finite number >= 0"),
        ("ctrl_retries", -1, "ctrl_retries must be an integer >= 0"),
        ("ctrl_retries", 1.0, "ctrl_retries must be an integer >= 0"),
        ("ctrl_refresh_s", float("nan"), "ctrl_refresh_s must be a finite number >= 0"),
        (
            "duty_mismatch_tolerance",
            10001,
            "duty_mismatch_tolerance must be an integer in 0..10000",
        ),
        ("duty_mismatch_tolerance", 1.5, "duty_mismatch_tolerance must be an integer"),
        ("duty_mismatch_s", 0.0, "duty_mismatch_s must be a finite number > 0"),
        ("duty_mismatch_s", None, "duty_mismatch_s must be a finite number > 0"),
    ],
)
def test_bad_timing_value_is_config_error_naming_the_key(key: str, value, match: str) -> None:
    with pytest.raises(ConfigError, match=f"xt6.{match}"):
        _parse(dict(_SECTION, **{key: value}))


def test_zero_is_allowed_where_documented() -> None:
    timing = _parse(dict(_SECTION, ctrl_gap_ms=0, ctrl_retries=0, ctrl_refresh_s=0.0)).timing
    assert (timing.ctrl_gap_ms, timing.ctrl_retries, timing.ctrl_refresh_s) == (0, 0, 0.0)
    assert _parse(dict(_SECTION, duty_mismatch_tolerance=0)).timing.duty_mismatch_tolerance == 0
