"""Tests for the device entry of ``xt6:`` / ``aquacomputer:`` (aqua_bridge.hw.aquacomputer_adapter).

Startup cross-check against the mpc section (review finding F4), channel ranges
per device kind, timing keys and the hints for keys of the hwmon era.
PROJECT.md section 3 (Track B).
"""

from __future__ import annotations

import dataclasses
import re
from pathlib import Path

import pytest
import yaml

from aqua_bridge.hw.aquacomputer import AQUAERO, KINDS, QUADRO
from aqua_bridge.hw.aquacomputer_adapter import (
    ENTRY_KEYS,
    KIND_TIMING_DEFAULTS,
    TIMING_KEYS,
    AquacomputerAdapter,
    AquacomputerTiming,
    DeviceBinding,
    aquabus_binding_problem,
    build_adapter_from_config,
    check_watchdog,
    parse_device_section,
)
from aqua_bridge.hw.hidraw import USB_CTRL_TIMEOUT_S
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
    assert binding.temp_map == {"coolant": "temp1", "air": "temp2"}
    assert binding.timing == AquacomputerTiming.for_kind(AQUAERO)
    # without the mpc tuples the check is skipped (mapping-only callers)
    assert _parse(_SECTION).pwm_map == binding.pwm_map


def test_one_line_yaml_flow_mappings_and_every_optional_key() -> None:
    section = yaml.safe_load(
        """
device: quadro
serial: 12345-54321
prefer: hid
fans:
  radiator: {pwm: pwm4, rpm: fan4}
  intake:   {pwm: pwm2, rpm: fan2}
temp_map: {coolant: temp1, air: temp4}
status_max_age_s: 2.5
ctrl_gap_ms: 150
ctrl_retries: 3
ctrl_budget_s: 4
ctrl_refresh_s: 0
duty_mismatch_tolerance: 250
duty_mismatch_s: 12
write_min_interval_s: 0
write_deadband: 0
"""
    )
    binding = _parse(section, channels=_CHANNELS, temps=_TEMPS)
    assert binding.kind is QUADRO and binding.serial == "12345-54321"
    assert binding.pwm_map == {"radiator": 4, "intake": 2}
    assert binding.fan_map == {"radiator": 4, "intake": 2}
    assert binding.temp_map == {"coolant": "temp1", "air": "temp4"}
    assert binding.timing == AquacomputerTiming(
        status_max_age_s=2.5,
        ctrl_gap_ms=150,
        ctrl_retries=3,
        ctrl_budget_s=4,
        ctrl_refresh_s=0,
        duty_mismatch_tolerance=250,
        duty_mismatch_s=12,
        write_min_interval_s=0,
        write_deadband=0,
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
            "xt6.fans.radiator.pwm must be one of the aquaero's outputs pwm1..pwm8",
        ),
        (
            {"radiator": {"pwm": "pwm1", "rpm": "pwm1"}, "intake": {"pwm": "pwm2"}},
            "xt6.fans.radiator.rpm must be one of the aquaero's tachometers fan1..fan8",
        ),
        (
            {"radiator": {"pwm": "pwm1", "rpm": 1}, "intake": {"pwm": "pwm2"}},
            "xt6.fans.radiator.rpm must be",
        ),
        (
            {"radiator": {"pwm": "pwm9"}, "intake": {"pwm": "pwm2"}},
            "pwm1..pwm8, got 'pwm9'",
        ),
        (
            {"radiator": {"pwm": "pwm1", "rpm": "fan9"}, "intake": {"pwm": "pwm2"}},
            "fan1..fan8, got 'fan9'",
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
    with pytest.raises(ConfigError, match="quadro's tachometers fan1..fan4, got 'fan6'"):
        _parse(dict(quadro, fans={"radiator": {"pwm": "pwm1", "rpm": "fan6"}}))
    with pytest.raises(ConfigError, match="quadro's outputs pwm1..pwm4, got 'pwm5'"):
        _parse(dict(quadro, fans={"radiator": {"pwm": "pwm5"}}))
    with pytest.raises(
        # softN is not offered as a choice: nothing may bind one (item 113)
        ConfigError,
        match="quadro's temperature inputs temp1..temp4, got 'bus1'",
    ):
        _parse(dict(quadro, temp_map={"coolant": "bus1"}))
    with pytest.raises(ConfigError, match="got 'virt1'"):
        _parse(dict(quadro, temp_map={"coolant": "virt1"}))
    _parse(dict(_SECTION, fans={"radiator": {"pwm": "pwm6", "rpm": "fan6"}}))


def test_every_input_name_of_both_kinds_is_accepted() -> None:
    """aquaero: outputs and tachometers 1-8 (5-8: a Quadro on its aquabus), physical
    sensors temp1..8, aquabus slots bus1..8, software sensors soft1..8, virtual
    sensors virt1..4; Quadro: 1-4, temp1..4, soft1..16 (PROJECT.md section 3, Track B)."""
    for kind in (AQUAERO, QUADRO):
        section = {
            "device": kind.name,
            "fans": {
                f"ch{n}": {"pwm": f"pwm{n}", "rpm": f"fan{n}"} for n in range(1, kind.pwm_count + 1)
            },
            # softN is never accepted (item 113); it is refused below.
            "temp_map": {
                f"t_{name}": name for name in kind.temp_names if name not in kind.soft_sensor_names
            },
        }
        binding = _parse(section)
        assert binding.pwm_map == {f"ch{n}": n for n in range(1, kind.pwm_count + 1)}
        assert binding.fan_map == binding.pwm_map
        assert set(binding.temp_map.values()) == set(kind.temp_names) - set(kind.soft_sensor_names)
    assert {"bus8", "soft8", "virt4"} <= set(AQUAERO.temp_names)
    assert {"soft16"} <= set(QUADRO.temp_names)


@pytest.mark.parametrize(
    ("device", "name"),
    [("aquaero", "soft1"), ("aquaero", "soft8"), ("quadro", "soft1"), ("quadro", "soft16")],
)
def test_no_software_sensor_can_be_bound_as_a_temperature(device: str, name: str) -> None:
    """Item 113. A ``softN`` slot holds whatever a host wrote into it and, once that
    host stops for the configured timeout, the configured fallback -- for ever, as a
    steady number that never reads 0x7FFF. Nothing in a status report separates that
    from a measurement, so the estimator may never be handed one. The daemon writes at
    most one software sensor itself (``heartbeat_sensor``), and that one carries
    ``heartbeat_value_c``: its own constant, which is no better."""
    with pytest.raises(ConfigError, match="is a software sensor, which cannot be bound"):
        _parse(dict(_SECTION, device=device, temp_map={"coolant": name}))


def test_the_refusal_holds_below_the_config_parser_too() -> None:
    """The binding itself refuses, so no code path -- a test, a tool, a future caller --
    can build one that would reach the estimator (PROJECT.md section 8 item 113)."""
    with pytest.raises(ValueError, match="'soft1' is a software sensor"):
        DeviceBinding(kind=AQUAERO, pwm_map={}, temp_map={"beat": "soft1"})
    # bus2 needs one of that device's own aquabus outputs bound alongside it (item 92).
    DeviceBinding(kind=AQUAERO, pwm_map={"qd2": 6}, temp_map={"air": "temp1", "coolant": "bus2"})


@pytest.mark.parametrize(
    ("device", "field", "value", "match"),
    [
        (
            "aquaero",
            "temp",
            "temp9",
            r"numbering 'temp9' is 'soft1', and 'soft1' is a software sensor, which "
            r"cannot be bound as a temperature",
        ),
        ("aquaero", "temp", "temp16", r"'temp16' is 'soft8', and 'soft8' is a software sensor"),
        (
            "aquaero",
            "temp",
            "temp17",
            r"'temp17' is 'virt1' \(the aquaero's virtual sensors\): use 'virt1'",
        ),
        ("aquaero", "temp", "temp20", r"use 'virt4'"),
        (
            "quadro",
            "temp",
            "temp5",
            r"'temp5' is 'soft1', and 'soft1' is a software sensor, which cannot be bound",
        ),
        ("quadro", "temp", "temp20", r"'temp20' is 'soft16', and 'soft16' is a software sensor"),
        (
            "quadro",
            "rpm",
            "fan5",
            r"'fan5' was the hwmon driver's name of the quadro's flow sensor flow1, which is "
            r"not a tachometer \(quadro: fan1..fan4\); flow sensors cannot be bound in the "
            r"config \(PROJECT.md section 8 item 91\)",
        ),
        (
            "aquaero",
            "rpm",
            "flow1",
            r"aquaero's tachometers fan1..fan8, got 'flow1'; 'flow1' is a flow sensor, and flow "
            r"sensors cannot be bound in the config \(PROJECT.md section 8 item 91\)",
        ),
        ("quadro", "temp", "flow1", r"got 'flow1'; 'flow1' is a flow sensor"),
        ("aquaero", "pwm", "flow1", r"outputs pwm1..pwm8, got 'flow1'; 'flow1' is a flow sensor"),
    ],
)
def test_hwmon_era_input_names_are_rejected_with_the_new_name(
    device: str, field: str, value: str, match: str
) -> None:
    """The Linux driver numbered the temperatures in one run and the flow sensors after the
    fans; those names mean other inputs now (aquabus slots, aquabus tachometers)."""
    section = dict(_SECTION, device=device)
    if field == "temp":
        section["temp_map"] = {"coolant": value}
    elif field == "pwm":
        section["fans"] = {"radiator": {"pwm": value}}
    else:
        section["fans"] = {"radiator": {"pwm": "pwm1", "rpm": value}}
    with pytest.raises(ConfigError, match=match):
        _parse(section)


@pytest.mark.parametrize("field", ["pwm", "rpm", "temp"])
def test_a_flow_name_is_rejected_everywhere_naming_the_aquaero_aquabus_tachometers(
    field: str,
) -> None:
    """Item 91 (owner decision 2026-09-16): flow cannot be bound anywhere in a device
    entry, and the message says so and that an aquaero's hwmon-era fan5/fan6 mean
    aquabus tachometers here, not the flow sensors the driver called by those names."""
    section = dict(_SECTION, device="aquaero")
    if field == "temp":
        section["temp_map"] = {"coolant": "flow2"}
    elif field == "pwm":
        section["fans"] = {"radiator": {"pwm": "flow2"}}
    else:
        section["fans"] = {"radiator": {"pwm": "pwm1", "rpm": "flow2"}}
    with pytest.raises(ConfigError) as excinfo:
        _parse(section)
    message = str(excinfo.value)
    assert "flow sensors cannot be bound in the config (PROJECT.md section 8 item 91)" in message
    assert "publishes them with the device health" in message
    assert "fan5..fan8 are the aquaero's aquabus tachometers" in message


def test_the_quadro_hwmon_flow_name_also_names_the_aquaero_difference() -> None:
    section = dict(_SECTION, device="quadro", fans={"radiator": {"pwm": "pwm1", "rpm": "fan5"}})
    with pytest.raises(ConfigError, match="are aquabus tachometers now"):
        _parse(section)


def test_any_aquaero_tachometer_may_be_bound_to_any_output() -> None:
    """Review finding: fan5/fan6 were the hwmon driver's flow sensors, but on the aquaero
    they are aquabus tachometers now, so they are accepted with any output, like fan7."""
    binding = _parse(
        dict(
            _SECTION,
            fans={
                "a": {"pwm": "pwm5", "rpm": "fan5"},
                "b": {"pwm": "pwm6", "rpm": "fan6"},
                "c": {"pwm": "pwm1", "rpm": "fan7"},
                "d": {"pwm": "pwm2", "rpm": "fan8"},
            },
        )
    )
    assert binding.fan_map == {"a": 5, "b": 6, "c": 7, "d": 8}
    binding = _parse(dict(_SECTION, fans={"a": {"pwm": "pwm6", "rpm": "fan5"}}))
    assert (binding.pwm_map, binding.fan_map) == ({"a": 6}, {"a": 5})


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
        ("ctrl_retries", -1, "ctrl_retries must be an integer in 0..5"),
        ("ctrl_retries", 1.0, "ctrl_retries must be an integer in 0..5"),
        ("ctrl_retries", 6, "ctrl_retries must be an integer in 0..5"),
        ("ctrl_budget_s", 0, "ctrl_budget_s must be a finite number > 0"),
        ("ctrl_budget_s", float("inf"), "ctrl_budget_s must be a finite number > 0"),
        ("write_min_interval_s", -0.5, "write_min_interval_s must be a finite number >= 0"),
        ("write_min_interval_s", "30", "write_min_interval_s must be a finite number >= 0"),
        ("write_deadband", -1, "write_deadband must be an integer in 0..10000"),
        ("write_deadband", 10001, "write_deadband must be an integer in 0..10000"),
        ("write_deadband", 0.5, "write_deadband must be an integer in 0..10000"),
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
    writes = _parse(dict(_SECTION, write_min_interval_s=0, write_deadband=0)).timing
    assert (writes.write_min_interval_s, writes.write_deadband) == (0, 0)


def test_write_limiting_is_off_by_default_and_still_configurable() -> None:
    """Item 86: writes are not saved, so every change is written by default; the keys
    remain to limit USB traffic."""
    assert (
        _parse(_SECTION).timing.write_min_interval_s,
        _parse(_SECTION).timing.write_deadband,
    ) == (0, 0)
    limited = _parse(dict(_SECTION, write_min_interval_s=30, write_deadband=50)).timing
    assert (limited.write_min_interval_s, limited.write_deadband) == (30, 50)


def test_ctrl_gap_default_depends_on_the_device_kind() -> None:
    """Owner decision 2026-09-15: 100 ms for the aquaero (EPIPE at 0 and 25 ms on the
    Pi), no gap for the Quadro; declared once, in KIND_TIMING_DEFAULTS."""
    assert set(KIND_TIMING_DEFAULTS) == set(KINDS)
    assert _parse(_SECTION).timing.ctrl_gap_ms == KIND_TIMING_DEFAULTS["aquaero"]["ctrl_gap_ms"]
    quadro = _parse(dict(_SECTION, device="quadro")).timing
    assert quadro.ctrl_gap_ms == KIND_TIMING_DEFAULTS["quadro"]["ctrl_gap_ms"]
    assert (AquacomputerTiming.for_kind(AQUAERO).ctrl_gap_ms, quadro.ctrl_gap_ms) == (100, 0)
    assert _parse(dict(_SECTION, device="quadro", ctrl_gap_ms=40)).timing.ctrl_gap_ms == 40
    # Everything else is the same for both kinds.
    aquaero = AquacomputerTiming.for_kind(AQUAERO)
    assert dataclasses.replace(aquaero, ctrl_gap_ms=0) == AquacomputerTiming.for_kind(QUADRO)
    with pytest.raises(TypeError):
        AquacomputerTiming()  # type: ignore[call-arg]  # no kind-independent gap default


def test_worst_case_tick_and_the_watchdog_check() -> None:
    """Review finding: the per-device bound is budget + one transfer + the gap the
    retry sleeps before it finds the budget spent, and pings are dt + step apart
    besides."""
    aquaero = AquacomputerTiming.for_kind(AQUAERO)
    quadro = AquacomputerTiming.for_kind(QUADRO)
    worst = aquaero.status_max_age_s + aquaero.ctrl_budget_s + USB_CTRL_TIMEOUT_S
    assert aquaero.worst_case_tick_s() == pytest.approx(worst + aquaero.ctrl_gap_ms / 1000)
    assert quadro.worst_case_tick_s() == pytest.approx(worst)  # no gap
    slow_gap = AquacomputerTiming.for_kind(AQUAERO, ctrl_gap_ms=3000)
    assert slow_gap.worst_case_tick_s() == pytest.approx(worst + 3.0)
    pair = [("aquacomputer[0]", aquaero), ("aquacomputer[1]", quadro)]
    dt, step = 5.0, 0.75
    total = dt + step + aquaero.worst_case_tick_s() + quadro.worst_case_tick_s()
    check_watchdog(pair, None)
    check_watchdog(pair, total + 0.001, dt=dt, step_bound_s=step)
    with pytest.raises(ConfigError, match=r"mpc.dt 5 s .*aquacomputer\[0\] .*not below"):
        check_watchdog(pair, total, dt=dt, step_bound_s=step)
    with pytest.raises(ValueError, match="needs dt and step_bound_s"):
        check_watchdog(pair, total + 1)
    single = dt + step + aquaero.worst_case_tick_s()
    with pytest.raises(ConfigError, match="WatchdogSec"):
        build_adapter_from_config(_SECTION, watchdog_s=single, dt=dt, step_bound_s=step)
    build_adapter_from_config(_SECTION, watchdog_s=single + 1, dt=dt, step_bound_s=step)


def test_a_retry_after_a_timeout_blocks_no_longer_than_the_bound() -> None:
    """The reviewer's case: ctrl_gap_ms 3000, ctrl_retries 2, a control operation that
    starts just before the budget runs out and times out after the usbhid timeout."""
    from aqua_bridge.hw.aquacomputer_adapter import AquacomputerAdapter, DeviceBinding
    from aqua_bridge.hw.hidraw import DeviceUnavailable, FeatureReportError
    from aqua_bridge.model import Mode, MpcCommand
    from aquacomputer_fakes import FakeBus, FakeClock, FakeController, FakeSleep

    timing = AquacomputerTiming.for_kind(AQUAERO, ctrl_gap_ms=3000, ctrl_retries=2)
    clock = FakeClock()
    device = FakeController(AQUAERO, clock, op_delay_s=USB_CTRL_TIMEOUT_S)
    device.failures = [FeatureReportError("ETIMEDOUT") for _ in range(3)]
    adapter = AquacomputerAdapter(
        DeviceBinding(kind=AQUAERO, pwm_map={"xt1": 1}, timing=timing),
        clock=clock,
        sleep=FakeSleep(clock),
        opener=FakeBus(device),
    )
    start = clock()
    with pytest.raises(DeviceUnavailable):
        adapter.apply(MpcCommand(pwm={"xt1": 0.5}, mode=Mode.AUTO))
    blocked = clock() - start
    assert blocked <= timing.worst_case_tick_s() - timing.status_max_age_s + 1e-9


def test_the_unit_watchdog_fits_the_das_example_as_it_ships_and_with_two_controllers() -> None:
    """deploy/aqua-bridge.service's WatchdogSec against config.example-das.yaml's dt and
    step bound plus the controllers *the example declares*, and plus the aquaero and a
    Quadro on its own USB port.

    The timings are built from the example's own entries, the way config.py builds them
    (``from_section``), not from the per-kind defaults: the example spells its timing
    keys out, so defaults that equal them today would make this test blind to the day
    someone raises one of them."""
    from aqua_bridge.config import load_config

    root = Path(__file__).resolve().parent.parent
    unit = (root / "deploy" / "aqua-bridge.service").read_text()
    match = re.search(r"^WatchdogSec=(\d+)$", unit, re.MULTILINE)
    assert match is not None
    app = load_config(root / "config.example-das.yaml")
    example = [
        (
            entry["device"],
            AquacomputerTiming.from_section(entry, f"aquacomputer[{i}]", KINDS[entry["device"]]),
        )
        for i, entry in enumerate(app.aquacomputer)
    ]
    assert [name for name, _ in example] == ["aquaero"]  # the Quadro on its aquabus
    # The alternative the example describes, the Quadro on its own USB port, fits too.
    both = [*example, ("quadro", AquacomputerTiming.for_kind("quadro"))]
    for pair in (example, both):
        check_watchdog(
            pair,
            float(match.group(1)),
            dt=app.mpc.dt,
            step_bound_s=app.mpc.budget_alarm_ms / 1000.0,
        )


# --- the bus-absent window (item 92) and the aquabus temperature slots ---------------------


def test_bus_absent_s_defaults_and_is_bounded_only_by_being_a_positive_time() -> None:
    """The key that says how long every aquabus block must read "no device" before the
    daemon reports the bus device lost (item 92). Nothing about the aquabus floors it
    (item 115): an aquabus block's electrical fields are sampled inside the PWM cycle
    and read 0 mA in most reports at a low duty, while presence is read from the speed
    field every report carries, so no report can look like a departure however short the
    window is. A short window only risks reporting a re-enumeration blip, which costs a
    log line and never cooling -- the temperatures go missing from the first empty report
    either way."""
    assert _parse(_SECTION).timing.bus_absent_s == 10.0
    assert "bus_absent_s" in TIMING_KEYS and "bus_absent_s" in ENTRY_KEYS
    assert _parse(dict(_SECTION, bus_absent_s=4.0)).timing.bus_absent_s == 4.0
    assert _parse(dict(_SECTION, bus_absent_s=1.0)).timing.bus_absent_s == 1.0
    with pytest.raises(ConfigError, match="bus_absent_s must be a finite number > 0"):
        _parse(dict(_SECTION, bus_absent_s=0))
    assert AquacomputerTiming.for_kind(QUADRO, bus_absent_s=0.5).bus_absent_s == 0.5
    quadro = _parse(dict(_SECTION, device="quadro", bus_absent_s=0.5))
    assert quadro.timing.bus_absent_s == 0.5


def test_a_busn_no_longer_needs_an_aquabus_output_bound_on_the_aquaero() -> None:
    """Item 92's teeth, and item 130's loosening of them. A ``busN`` slot keeps the last
    value it read when the device on aquabus leaves, so it is a reading only while a
    device answers there -- judged, on the aquaero, from the aquabus fan blocks **or**
    the aquabus flow slot (item 130): the flow slot needs no fan behind any block, so a
    bus device with no fan outputs is no longer indistinguishable from an empty bus, and
    an aquaero entry may bind a ``busN`` with no aquabus output bound alongside it."""
    fans = {"radiator": {"pwm": "pwm1", "rpm": "fan1"}, "intake": {"pwm": "pwm2"}}
    sensor_only = dict(_SECTION, fans=fans, temp_map={"coolant": "temp1", "air": "bus2"})
    assert _parse(sensor_only).temp_map == {"coolant": "temp1", "air": "bus2"}
    # One of the bus device's own outputs commanded (the supported topology) still works,
    with_output = dict(sensor_only, fans=dict(fans, quadro1={"pwm": "pwm7", "rpm": "fan7"}))
    assert _parse(with_output).temp_map == {"coolant": "temp1", "air": "bus2"}
    # ... and so is the same entry with no aquabus temperature bound at all.
    assert _parse(dict(_SECTION, fans=fans)).temp_map == {"coolant": "temp1", "air": "temp2"}


def test_a_busn_still_needs_a_witness_on_a_kind_without_the_aquabus_flow_slot() -> None:
    """Item 130's fallback: a kind with neither a bound aquabus output nor an aquabus
    flow witness (``aquabus_flow_index=None``) still has the binding refused, naming the
    entry -- dead in the supported topology (the aquaero always has the flow witness),
    exercised directly against a synthetic kind since no shipped kind lacks one."""
    no_witness = dataclasses.replace(AQUAERO, aquabus_flow_index=None)
    problem = aquabus_binding_problem(no_witness, {}, {}, {"coolant": "temp1", "air": "bus2"})
    assert problem is not None
    assert "no aquabus output" in problem
    assert "no aquabus flow witness" in problem
    with pytest.raises(ValueError, match="no aquabus flow witness"):
        DeviceBinding(kind=no_witness, pwm_map={}, temp_map={"air": "bus2"})
    # One of the bus device's own outputs bound is still accepted, same as before item 130.
    DeviceBinding(kind=no_witness, pwm_map={"qd1": 5}, temp_map={"air": "bus2"})


# --- the software-sensor heartbeat (item 84) -----------------------------------------------


def test_the_heartbeat_is_off_by_default_and_configured_per_device() -> None:
    default = _parse(_SECTION).timing
    assert (default.heartbeat_sensor, default.heartbeat_value_c) == (0, 20.0)
    assert not default.heartbeat_on
    on = _parse(dict(_SECTION, heartbeat_sensor=1, heartbeat_value_c=25.5)).timing
    assert (on.heartbeat_sensor, on.heartbeat_value_c) == (1, 25.5)
    assert on.heartbeat_on
    assert _parse(dict(_SECTION, heartbeat_sensor=8)).timing.heartbeat_sensor == 8
    assert {"heartbeat_sensor", "heartbeat_value_c"} <= set(TIMING_KEYS) <= set(ENTRY_KEYS)


@pytest.mark.parametrize(
    ("value", "match"),
    [
        (9, "one of the aquaero's software sensors 1..8"),
        (-1, "heartbeat_sensor must be an integer >= 0"),
        (1.0, "heartbeat_sensor must be an integer >= 0"),
        ("1", "heartbeat_sensor must be an integer >= 0"),
        (True, "heartbeat_sensor must be an integer >= 0"),
    ],
)
def test_bad_heartbeat_sensor_is_config_error(value, match: str) -> None:
    with pytest.raises(ConfigError, match=re.escape(match)):
        _parse(dict(_SECTION, heartbeat_sensor=value))


@pytest.mark.parametrize("value", [400.0, -400.0, float("nan"), float("inf"), "20", None, True])
def test_a_heartbeat_value_the_report_cannot_carry_is_config_error(value) -> None:
    with pytest.raises(ConfigError, match="heartbeat_value_c must be a finite number in"):
        _parse(dict(_SECTION, heartbeat_sensor=1, heartbeat_value_c=value))


def test_the_quadro_has_no_known_software_sensor_report() -> None:
    """Only the aquaero's output report 0x07 is verified, so a heartbeat on a Quadro
    entry is refused at startup instead of writing bytes nobody has seen work."""
    quadro = dict(
        _SECTION, device="quadro", fans={"radiator": {"pwm": "pwm1"}, "intake": {"pwm": "pwm2"}}
    )
    assert _parse(quadro).timing.heartbeat_sensor == 0  # off is fine
    with pytest.raises(ConfigError, match="not supported on the quadro"):
        _parse(dict(quadro, heartbeat_sensor=1))
    with pytest.raises(ConfigError, match="not supported on the quadro"):
        AquacomputerTiming.for_kind(QUADRO, heartbeat_sensor=1)


def test_a_binding_built_in_code_validates_the_heartbeat_against_its_kind() -> None:
    with pytest.raises(ConfigError, match="software sensors 1..8"):
        DeviceBinding(
            kind=AQUAERO,
            pwm_map={"radiator": 1},
            timing=dataclasses.replace(AquacomputerTiming.for_kind(AQUAERO), heartbeat_sensor=99),
        )


def test_both_example_configs_show_the_heartbeat_keys() -> None:
    """Every operator-tunable key is in both example files with its default
    (PROJECT.md section 3, Track B)."""
    from aqua_bridge.config import load_config

    root = Path(__file__).resolve().parent.parent
    legacy = load_config(root / "config.example.yaml").xt6
    (aquaero,) = load_config(root / "config.example-das.yaml").aquacomputer
    for entry in (legacy, aquaero):
        assert entry["heartbeat_sensor"] == 0
        assert entry["heartbeat_value_c"] == AquacomputerTiming.for_kind(AQUAERO).heartbeat_value_c
