"""Fan health, device health and the board's own health (PROJECT.md section 8 items
79, 83 and 97).

The readings the rules judge come from the captured status reports in
``tests/fixtures/aquacomputer/``, decoded by the real adapter against the real
device layouts, so a rule is exercised on the numbers the owner's hardware
actually produced: the aquaero reporting 0 mA and 0 W for its own outputs in PWM
mode, the Quadro on aquabus reporting 27 mA / 0.32 W at 100 % duty, and an empty
aquabus slot reading rpm 0xFFFF, 0 V.
"""

from __future__ import annotations

import copy
import logging
from pathlib import Path
from typing import Any

import pytest

from aqua_bridge.config import load_config
from aqua_bridge.health import (
    FANHEALTH_KEYS,
    HOSTHEALTH_KEYS,
    FanHealthConfig,
    HealthMonitor,
    HostHealth,
    HostHealthConfig,
    default_air_temps,
    expected_rpm,
)
from aqua_bridge.hostinfo import decode_throttled, read_rpi_volt_hwmon
from aqua_bridge.hw.aquacomputer import AQUAERO, QUADRO, decode_status
from aqua_bridge.model import ConfigError, FanModel, FanSpec, MpcConfig, PlantObservation
from aquacomputer_fakes import fixture_bytes

# --- config ---------------------------------------------------------------------------


def test_every_threshold_is_a_key_with_one_default() -> None:
    """Item 79's thresholds are config keys, declared once in FanHealthConfig."""
    defaults = FanHealthConfig()
    assert set(FANHEALTH_KEYS) == {
        "enabled",
        "min_duty",
        "settle_s",
        "rpm_tolerance_frac",
        "rpm_fault_s",
        "rail_min_v",
        "rail_max_v",
        "rail_fault_s",
        "power_tolerance_frac",
        "power_exponent",
        "power_min_w",
        "power_fault_s",
        "log_interval_s",
    }
    assert defaults.enabled is True
    assert (defaults.rail_min_v, defaults.rail_max_v) == (11.0, 13.0)
    assert FanHealthConfig.from_section(None) == defaults
    assert FanHealthConfig.from_section({}) == defaults


def test_from_section_overrides_and_keeps_the_other_defaults() -> None:
    settings = FanHealthConfig.from_section({"rpm_fault_s": 30.0, "enabled": False})
    assert settings.rpm_fault_s == 30.0 and settings.enabled is False
    assert settings.rail_max_v == FanHealthConfig().rail_max_v


@pytest.mark.parametrize(
    ("section", "match"),
    [
        ({"rpm_fault_sec": 5}, r"unknown key\(s\) \['rpm_fault_sec'\]"),
        ({"enabled": "true"}, "fan_health.enabled must be true or false"),
        ({"min_duty": 1.5}, "fan_health.min_duty must be <= 1"),
        ({"min_duty": "0.3"}, "fan_health.min_duty must be a number"),
        ({"rpm_fault_s": 0}, "fan_health.rpm_fault_s must be >="),
        ({"rail_min_v": 13.0, "rail_max_v": 12.0}, "must be above fan_health.rail_min_v"),
        ({"power_exponent": 0}, "fan_health.power_exponent must be >="),
        ({"settle_s": float("inf")}, "fan_health.settle_s must be finite"),
    ],
)
def test_a_bad_fan_health_key_is_a_config_error(section: dict[str, Any], match: str) -> None:
    with pytest.raises(ConfigError, match=match):
        FanHealthConfig.from_section(section)


def test_power_w_at_max_is_optional_and_must_be_positive(das_example_cfg: MpcConfig) -> None:
    """Without it the power rule is off for that model; a bad value is a config error."""
    assert das_example_cfg.fan_models["case120"].power_w_at_max is None
    raw = das_example_cfg.to_dict()
    raw["fan_models"]["case120"]["power_w_at_max"] = 1.9
    assert MpcConfig.from_mapping(raw).fan_models["case120"].power_w_at_max == 1.9
    raw["fan_models"]["case120"]["power_w_at_max"] = 0
    with pytest.raises(ConfigError, match="power_w_at_max must be a finite number > 0"):
        MpcConfig.from_mapping(raw)


# --- the curve ------------------------------------------------------------------------


def _cfg(**models: FanModel) -> Any:
    """A minimal stand-in for MpcConfig with just what the rules read.

    ``temps``/``sensors`` are the empty pair a legacy config has, so the board's air
    reference resolves to nothing and the divergence rule is off unless a test names
    its own ``air_temps``. The reference's own resolution is tested against real
    configs (:func:`default_air_temps`, below).
    """

    class _Cfg:
        fans = {"qd3": FanSpec(model="case120"), "xt2": FanSpec(model="case120")}
        fan_models = dict(models or {"case120": FanModel(rpm_max=1200.0, deadband=0.1)})
        temps: tuple[str, ...] = ()
        sensors: dict[str, Any] = {}

    return _Cfg()


#: What the board's rules publish with no host reader at all: every measurement
#: missing, nothing to judge, no problem. The payload tests below pin this instead of
#: comparing the payload's ``host`` key against itself, so a verdict that started
#: reporting a problem for a board with no readings would fail them.
_EMPTY_BOARD: dict[str, Any] = {
    "cpu_temp_c": None,
    "air_c": None,
    "air_temps": [],
    "divergence_c": None,
    "load1": None,
    "idle": None,
    "throttled": None,
    "faults": [],
    "hints": [],
    "problems": [],
    "ok": True,
}


def test_expected_rpm_follows_the_fitted_curve_and_is_none_without_a_model() -> None:
    cfg = _cfg(case120=FanModel(rpm_max=1200.0, deadband=0.1, exponent=1.0))
    assert expected_rpm(cfg, "qd3", 1.0) == pytest.approx(1200.0)
    assert expected_rpm(cfg, "qd3", 0.1) == pytest.approx(0.0)  # inside the deadband
    assert expected_rpm(cfg, "qd3", 0.55) == pytest.approx(600.0)
    assert expected_rpm(cfg, "nowhere", 1.0) is None


def test_expected_rpm_is_none_for_a_legacy_config(cfg: MpcConfig) -> None:
    """A legacy config has no mpc.fans / fan_models, so the rpm rule is simply off."""
    assert not cfg.fans and not cfg.fan_models
    assert expected_rpm(cfg, "radiator", 1.0) is None


# --- the rules ------------------------------------------------------------------------


def _reading(**over: Any) -> dict[str, Any]:
    base = {
        "device": "aquaero",
        "output": "pwm7",
        "rpm": 1100.0,
        "duty": 1.0,
        "voltage_v": 12.1,
        "current_ma": 27.0,
        "power_w": 0.32,
        "power_reported": True,
        "aquabus": True,
    }
    base.update(over)
    return base


def _monitor(settings: FanHealthConfig | None = None, **models: FanModel) -> HealthMonitor:
    return HealthMonitor(_cfg(**models), settings or FanHealthConfig())


def _settle(mon: HealthMonitor, reading: dict[str, Any], t: float) -> float:
    """Feed one reading and wait out settle_s, so the next check is judged."""
    mon.check_channel("qd3", reading, t)
    return t + mon.settings.settle_s + 1.0


def test_a_healthy_fan_at_full_duty_has_no_problem() -> None:
    mon = _monitor(case120=FanModel(rpm_max=1100.0, deadband=0.1))
    t = _settle(mon, _reading(), 0.0)
    verdict = mon.check_channel("qd3", _reading(), t + 600.0)
    assert verdict["problems"] == []
    assert verdict["expected_rpm"] == pytest.approx(1100.0)


def test_rpm_far_below_the_curve_is_reported_only_after_rpm_fault_s() -> None:
    mon = _monitor(case120=FanModel(rpm_max=1100.0, deadband=0.1))
    t = _settle(mon, _reading(rpm=0.0), 0.0)
    assert mon.check_channel("qd3", _reading(rpm=0.0), t)["problems"] == []
    held = mon.settings.rpm_fault_s
    assert mon.check_channel("qd3", _reading(rpm=0.0), t + held - 1.0)["problems"] == []
    problems = mon.check_channel("qd3", _reading(rpm=0.0), t + held)["problems"]
    assert len(problems) == 1
    assert "0 rpm at 100 % duty" in problems[0] and "1100 rpm expected" in problems[0]


def test_a_recovering_fan_clears_the_deviation() -> None:
    mon = _monitor(case120=FanModel(rpm_max=1100.0, deadband=0.1))
    t = _settle(mon, _reading(rpm=0.0), 0.0)
    mon.check_channel("qd3", _reading(rpm=0.0), t + mon.settings.rpm_fault_s)
    assert mon.check_channel("qd3", _reading(), t + mon.settings.rpm_fault_s + 1)["problems"] == []
    # ... and the window starts again from scratch
    later = t + 2 * mon.settings.rpm_fault_s
    assert mon.check_channel("qd3", _reading(rpm=0.0), later)["problems"] == []


def test_a_duty_step_widens_the_band_so_the_aquabus_lag_never_fires() -> None:
    """An aquabus fan's rpm in the aquaero's status report lags its own report by
    several seconds (PROJECT.md section 2): the step to 100 % must not read as drift."""
    settings = FanHealthConfig(settle_s=10.0, rpm_fault_s=5.0)
    mon = _monitor(settings, case120=FanModel(rpm_max=1100.0, deadband=0.1))
    mon.check_channel("qd3", _reading(duty=0.3, rpm=330.0), 0.0)
    t = 20.0  # a full settle_s of readings at 30 %
    assert mon.check_channel("qd3", _reading(duty=0.3, rpm=330.0), t)["problems"] == []
    # the duty steps to 100 % and the reported rpm has not caught up yet: the band
    # still covers the 30 % the fan may still be turning at
    for dt in (0.0, 2.0, 4.0, 6.0, 8.0, 9.9):
        assert mon.check_channel("qd3", _reading(rpm=330.0), t + dt)["problems"] == []
    # once the band has narrowed to 100 %, the same stale speed is a deviation
    assert mon.check_channel("qd3", _reading(rpm=330.0), t + 11.0)["problems"] == []
    assert mon.check_channel("qd3", _reading(rpm=330.0), t + 17.0)["problems"]


def test_a_duty_that_moves_every_tick_is_still_judged() -> None:
    """A creeping duty must not defer every rule for ever: during a fallback ramp or an
    ident experiment is exactly when a sagging rail or a stalled fan matters. The rail
    does not depend on the duty at all; rpm is judged against the band the duty spanned.
    """
    mon = _monitor(case120=FanModel(rpm_max=1100.0, deadband=0.1))
    duty = 0.3
    rail: list[str] = []
    rpm: list[str] = []
    for step in range(200):  # 2 s ticks, the duty creeping 3 % a tick (past settle_duty)
        duty = 0.3 + 0.03 * step
        reading = _reading(duty=min(duty, 1.0), rpm=0.0, voltage_v=9.5)
        for text in mon.check_channel("qd3", reading, 2.0 * step)["problems"]:
            (rail if " V, outside " in text else rpm).append(text)
    assert rail and "9.50 V, outside 11..13 V" in rail[0]
    assert rpm and "0 rpm at" in rpm[0]


def test_a_gap_in_the_readings_restarts_every_window() -> None:
    """read() raised for ten minutes (an aquabus link loss, item 92) and the loop fed
    blank observations: the deviation from before the outage is not evidence held
    across it, and the first reading back is not judged against a stale band."""
    settings = FanHealthConfig(settle_s=10.0, rail_fault_s=30.0, rpm_fault_s=30.0)
    mon = _monitor(settings, case120=FanModel(rpm_max=1100.0, deadband=0.1))
    sagging = {"qd3": _reading(voltage_v=10.8, rpm=0.0)}
    for step in range(6):  # 10 s of a sagging rail and a dead fan: under both windows
        assert mon.update(sagging, 2.0 * step)["problems"] == []
    for step in range(300):  # ten minutes of ticks with no readings at all
        mon.update({}, 12.0 + 2.0 * step)
    back = mon.update(sagging, 620.0)
    assert back["problems"] == []  # nothing is "held for 610 s"
    assert mon.update(sagging, 640.0)["problems"] == []  # and the band is not settled
    assert mon.update(sagging, 700.0)["problems"]  # only live evidence counts


def test_a_channel_that_missed_one_tick_starts_its_windows_again() -> None:
    """Per channel, not only per source: an aquabus slot that went away and came back
    leaves the others' windows alone."""
    mon = _monitor(FanHealthConfig(settle_s=0.0, rail_fault_s=20.0))
    sagging = _reading(voltage_v=10.0)
    for step in range(3):
        mon.update({"qd3": sagging, "xt2": sagging}, 10.0 * step)
    mon.update({"xt2": sagging}, 30.0)  # qd3 missing for one tick
    problems = mon.update({"qd3": sagging, "xt2": sagging}, 40.0)["problems"]
    assert [p.split(":")[0] for p in problems] == ["xt2"]


def test_below_min_duty_no_rpm_rule_fires() -> None:
    """At 9.02 % duty the captured aquabus fan reported 128 rpm: inside the deadband
    region the curve says nothing, so nothing is judged there."""
    settings = FanHealthConfig(min_duty=0.25, settle_s=0.0)
    mon = _monitor(settings, case120=FanModel(rpm_max=1100.0, deadband=0.1))
    t = 0.0
    for step in range(20):
        verdict = mon.check_channel("qd3", _reading(duty=0.0902, rpm=128.0), t + 30.0 * step)
        assert verdict["problems"] == []


def test_a_sagging_rail_is_reported_after_rail_fault_s_at_any_duty() -> None:
    mon = _monitor()
    t = _settle(mon, _reading(voltage_v=10.2, duty=0.1), 0.0)
    assert mon.check_channel("qd3", _reading(voltage_v=10.2, duty=0.1), t)["problems"] == []
    problems = mon.check_channel(
        "qd3", _reading(voltage_v=10.2, duty=0.1), t + mon.settings.rail_fault_s
    )["problems"]
    assert len(problems) == 1 and "10.20 V, outside 11..13 V" in problems[0]


def test_an_empty_aquabus_slot_reading_0_v_is_no_rail_fault() -> None:
    """An aquaero's empty aquabus slot reads 0 V, which is absence, not a dead rail."""
    mon = _monitor()
    t = _settle(mon, _reading(voltage_v=0.0), 0.0)
    for step in range(10):
        assert mon.check_channel("qd3", _reading(voltage_v=0.0), t + 60.0 * step)["problems"] == []


def test_zero_power_on_an_aquaero_own_output_is_not_a_fault() -> None:
    """The aquaero reports 0 mA and 0 W for its own outputs in PWM mode however fast
    the fan turns (PROJECT.md section 8 item 79), so power_reported gates the rule."""
    mon = _monitor(case120=FanModel(rpm_max=1100.0, deadband=0.1, power_w_at_max=1.5))
    own = _reading(power_reported=False, aquabus=False, current_ma=0.0, power_w=0.0)
    t = _settle(mon, own, 0.0)
    for step in range(10):
        assert mon.check_channel("qd3", own, t + 60.0 * step)["problems"] == []


def test_power_out_of_line_with_the_duty_is_reported_where_power_is_reported() -> None:
    mon = _monitor(case120=FanModel(rpm_max=1100.0, deadband=0.1, power_w_at_max=1.5))
    dead = _reading(current_ma=0.0, power_w=0.0)
    t = _settle(mon, dead, 0.0)
    verdict = mon.check_channel("qd3", dead, t)
    assert verdict["expected_power_w"] == pytest.approx(1.5)
    assert verdict["problems"] == []
    problems = mon.check_channel("qd3", dead, t + mon.settings.power_fault_s)["problems"]
    assert len(problems) == 1 and "0.00 W at 100 % duty, 1.50 W expected" in problems[0]


def test_without_power_w_at_max_the_power_rule_is_off() -> None:
    mon = _monitor(case120=FanModel(rpm_max=1100.0, deadband=0.1))
    dead = _reading(current_ma=0.0, power_w=0.0)
    t = _settle(mon, dead, 0.0)
    verdict = mon.check_channel("qd3", dead, t + 10 * mon.settings.power_fault_s)
    assert verdict["expected_power_w"] is None and verdict["problems"] == []


def test_expected_power_follows_the_fan_law_and_the_fan_count() -> None:
    mon = HealthMonitor(_cfg(case120=FanModel(rpm_max=1100.0, deadband=0.0)), FanHealthConfig())
    mon.cfg.fans["qd3"] = FanSpec(model="case120", count=2)
    mon.cfg.fan_models["case120"] = FanModel(rpm_max=1100.0, deadband=0.0, power_w_at_max=1.0)
    assert mon._expected_power("qd3", 1.0) == pytest.approx(2.0)
    assert mon._expected_power("qd3", 0.5) == pytest.approx(2.0 * 0.5**3)


# --- the tick -------------------------------------------------------------------------


class _FakeSource:
    def __init__(self, health: dict[str, Any]) -> None:
        self.health = health
        self.calls = 0

    def device_health(self) -> dict[str, Any]:
        self.calls += 1
        return self.health


def _tick(obs: PlantObservation) -> Any:
    class _Result:
        pass

    result = _Result()
    result.obs = obs  # type: ignore[attr-defined]
    return result


def test_on_tick_publishes_the_fan_verdicts_and_the_device_health() -> None:
    source = _FakeSource(
        {
            "devices": [{"label": "aquaero", "stuck_channels": ["qd3"]}],
            "problems": ["aquaero: qd3 do not follow the written duty"],
            "ok": False,
        }
    )
    published: list[dict[str, Any]] = []
    mon = HealthMonitor(
        _cfg(case120=FanModel(rpm_max=1100.0, deadband=0.1)),
        FanHealthConfig(settle_s=0.0, rpm_fault_s=0.001),
        source=source,
        publish=published.append,
    )

    def obs(ts: float) -> PlantObservation:
        return PlantObservation(
            temps={}, rpm={}, pwm={"qd3": 1.0}, ts=ts, inputs={"fans": {"qd3": _reading(rpm=0.0)}}
        )

    for ts in (5.0, 10.0, 15.0):  # the observation clock is the rules' clock
        mon.on_tick(_tick(obs(ts)))
    payload = published[-1]
    assert payload["devices"][0]["label"] == "aquaero"
    assert payload["fans"]["qd3"]["rpm"] == 0.0
    assert payload["ok"] is False
    assert payload["problems"][0].startswith("aquaero: qd3")  # device problems come first
    assert any("rpm at 100 % duty" in p for p in payload["problems"])
    assert source.calls == 3


def test_on_tick_without_a_device_health_source_still_publishes_the_fan_verdicts() -> None:
    published: list[dict[str, Any]] = []
    mon = HealthMonitor(_cfg(), FanHealthConfig(), source=object(), publish=published.append)
    obs = PlantObservation(temps={}, rpm={}, pwm={}, ts=1.0, inputs={"fans": {"qd3": _reading()}})
    mon.on_tick(_tick(obs))
    assert published[-1] == {
        "devices": [],
        "fans": published[-1]["fans"],
        "host": _EMPTY_BOARD,
        "problems": [],
        "ok": True,
    }
    assert published[-1]["fans"]["qd3"]["power_w"] == pytest.approx(0.32)


def test_a_single_controller_source_is_published_as_one_device() -> None:
    """--source xt6 (the argparse default, and what deploy/install-pi.sh installs)
    hands the loop a bare adapter, whose device_health() is that one controller's own
    dict, not a {devices, problems} one. It must reach /api/state, the page and Home
    Assistant in the same shape a composite does (PROJECT.md section 8 items 83, 91)."""
    source = _FakeSource(
        {
            "label": "aquaero",
            "device": "aquaero",
            "serial": "12345-54321",
            "firmware": 2104,
            "stuck_channels": [],
            "absent_channels": ["qd3"],
            "not_pwm_channels": [],
            "unconfigured_channels": [],
            "flows": {"flow1": 0, "flow2": None},
            "problems": ["aquaero: no device on aquabus behind qd3"],
        }
    )
    mon = HealthMonitor(_cfg(), FanHealthConfig(), source=source)
    payload = mon.update({}, 0.0)
    assert [d["label"] for d in payload["devices"]] == ["aquaero"]
    assert payload["devices"][0]["flows"] == {"flow1": 0, "flow2": None}
    assert payload["devices"][0]["absent_channels"] == ["qd3"]
    assert payload["problems"] == ["aquaero: no device on aquabus behind qd3"]
    assert payload["ok"] is False


def test_a_source_answering_with_neither_shape_contributes_nothing() -> None:
    mon = HealthMonitor(_cfg(), FanHealthConfig(), source=_FakeSource({}))
    payload = mon.update({}, 0.0)
    assert payload == {
        "devices": [],
        "fans": {},
        "host": _EMPTY_BOARD,
        "problems": [],
        "ok": True,
    }


def test_disabled_still_publishes_device_health_but_runs_no_rule() -> None:
    source = _FakeSource({"devices": [{"label": "aquaero"}], "problems": [], "ok": True})
    mon = HealthMonitor(
        _cfg(case120=FanModel(rpm_max=1100.0, deadband=0.1)),
        FanHealthConfig(enabled=False),
        source=source,
    )
    payload = mon.update({"qd3": _reading(rpm=0.0)}, 10_000.0)
    assert payload["fans"] == {} and payload["devices"][0]["label"] == "aquaero"


def test_on_tick_never_raises_on_a_broken_result_or_a_broken_source() -> None:
    class _Angry:
        def device_health(self) -> dict[str, Any]:
            raise RuntimeError("no")

    mon = HealthMonitor(_cfg(), FanHealthConfig(), source=_Angry(), publish=lambda _p: None)
    mon.on_tick(None)
    mon.on_tick(_tick(PlantObservation(temps={}, rpm={}, pwm={}, ts=0.0)))
    assert mon.last["ok"] is True

    def boom(_payload: dict[str, Any]) -> None:
        raise RuntimeError("publisher")

    HealthMonitor(_cfg(), FanHealthConfig(), publish=boom).on_tick(
        _tick(PlantObservation(temps={}, rpm={}, pwm={}, ts=0.0))
    )


def test_a_repeated_problem_is_logged_at_most_once_per_log_interval_s(
    caplog: pytest.LogCaptureFixture,
) -> None:
    mon = _monitor(
        FanHealthConfig(settle_s=0.0, rpm_fault_s=0.0001, log_interval_s=100.0),
        case120=FanModel(rpm_max=1100.0, deadband=0.1),
    )
    with caplog.at_level(logging.WARNING, logger="aqua_bridge.health"):
        for step in range(6):  # 6 ticks, 10 s apart: one line
            mon.update({"qd3": _reading(rpm=0.0)}, 10.0 * step)
        assert len(caplog.records) == 1
        mon.update({"qd3": _reading(rpm=0.0)}, 200.0)
        assert len(caplog.records) == 2


# --- against the captured reports -----------------------------------------------------


def test_the_captured_aquaero_own_outputs_report_no_power_and_the_aquabus_one_does() -> None:
    status = decode_status(AQUAERO, fixture_bytes("aquaero-status-aquabus-fan7-100.bin"))
    assert not AQUAERO.reports_power(1) and AQUAERO.reports_power(7)
    assert (status.fans[0].current_ma, status.fans[0].power_cw) == (0, 0)  # own output, 349 rpm
    assert (status.fans[6].current_ma, status.fans[6].power_w) == (27, pytest.approx(0.32))
    assert QUADRO.reports_power(1)


def test_the_captured_rails_sit_inside_the_default_window() -> None:
    defaults = FanHealthConfig()
    for name, kind in (
        ("aquaero-status.bin", AQUAERO),
        ("aquaero-status-aquabus-fan7-100.bin", AQUAERO),
        ("quadro-status.bin", QUADRO),
    ):
        for fan in decode_status(kind, fixture_bytes(name)).fans:
            if fan.present and fan.voltage_v > 0.0:
                assert defaults.rail_min_v <= fan.voltage_v <= defaults.rail_max_v, name


# --- the board itself (item 97) ---------------------------------------------------------


def test_every_host_threshold_is_a_key_with_one_default() -> None:
    """Item 97's thresholds are config keys, declared once in HostHealthConfig."""
    defaults = HostHealthConfig()
    assert set(HOSTHEALTH_KEYS) == {
        "enabled",
        "temp_limit_c",
        "temp_fault_s",
        "divergence_c",
        "divergence_fault_s",
        "idle_load1_max",
        "air_temps",
        "log_interval_s",
        "vcgencmd_interval_s",
        "vcgencmd_timeout_s",
    }
    assert defaults.enabled is True and defaults.air_temps == ()
    assert defaults.temp_limit_c == 75.0 and defaults.divergence_c == 40.0
    assert defaults.vcgencmd_timeout_s == 2.0 and defaults.vcgencmd_interval_s == 60.0
    assert HostHealthConfig.from_section(None) == defaults
    assert HostHealthConfig.from_section({}) == defaults


def test_host_from_section_overrides_and_keeps_the_other_defaults() -> None:
    settings = HostHealthConfig.from_section({"temp_limit_c": 70.0, "air_temps": ["inlet_a"]})
    assert settings.temp_limit_c == 70.0 and settings.air_temps == ("inlet_a",)
    assert settings.divergence_c == HostHealthConfig().divergence_c


@pytest.mark.parametrize(
    ("section", "match"),
    [
        ({"temp_limit": 70}, r"unknown key\(s\) \['temp_limit'\]"),
        ({"enabled": "true"}, "host_health.enabled must be true or false"),
        ({"temp_limit_c": "70"}, "host_health.temp_limit_c must be a number"),
        ({"temp_fault_s": 0}, "host_health.temp_fault_s must be >="),
        ({"divergence_c": 0}, "host_health.divergence_c must be >="),
        ({"idle_load1_max": -1.0}, "host_health.idle_load1_max must be >="),
        ({"divergence_fault_s": float("nan")}, "host_health.divergence_fault_s must be finite"),
        ({"vcgencmd_timeout_s": 0}, "host_health.vcgencmd_timeout_s must be >="),
        ({"vcgencmd_interval_s": 0}, "host_health.vcgencmd_interval_s must be >="),
        ({"vcgencmd_interval_s": "60"}, "host_health.vcgencmd_interval_s must be a number"),
        ({"air_temps": "inlet_a"}, "host_health.air_temps must be a list"),
        ({"air_temps": [""]}, "air_temps entries must be non-empty strings"),
        ({"air_temps": [3]}, "air_temps entries must be non-empty strings"),
    ],
)
def test_a_bad_host_health_key_is_a_config_error(section: dict[str, Any], match: str) -> None:
    with pytest.raises(ConfigError, match=match):
        HostHealthConfig.from_section(section)


def test_air_temps_naming_an_unknown_temperature_is_a_config_error(
    das_example_cfg: MpcConfig,
) -> None:
    with pytest.raises(ConfigError, match="air_temps names temperature"):
        HealthMonitor(
            das_example_cfg,
            FanHealthConfig(),
            host_settings=HostHealthConfig(air_temps=("nowhere",)),
        )


def test_default_air_temps_prefers_the_zone_air_sensors(das_example_cfg: MpcConfig) -> None:
    """The air the drives and the board sit in, not the intake and not a bay probe."""
    names = default_air_temps(das_example_cfg)
    assert names, "the DAS example config has zone_air sensors"
    assert all(das_example_cfg.sensors[n].role == "zone_air" for n in names)


def test_default_air_temps_falls_back_to_inlet_then_to_every_temperature(
    das_example_cfg: MpcConfig, cfg: MpcConfig
) -> None:
    """The inlet step is a backstop: ``mpc.sensors`` validation gives every zone a
    non-redundant ``zone_air`` sensor today, so it is reached only by a config shape
    that validation does not currently allow. It is exercised on a copy of the real
    :class:`MpcConfig` -- the same object, its zone-air sensors removed after
    construction -- rather than a stub with hand-written attributes, so a renamed
    field breaks this test the way it breaks the daemon."""
    keep = tuple(t for t in das_example_cfg.temps if das_example_cfg.sensors[t].role != "zone_air")
    no_zone_air = copy.copy(das_example_cfg)
    object.__setattr__(no_zone_air, "temps", keep)
    object.__setattr__(no_zone_air, "sensors", {t: das_example_cfg.sensors[t] for t in keep})

    inlets = default_air_temps(no_zone_air)
    assert inlets and all(das_example_cfg.sensors[n].role == "inlet" for n in inlets)
    # a legacy config declares no roles at all: every configured temperature
    assert default_air_temps(cfg) == cfg.temps


def _hwmon(root: Path, devices: dict[str, dict[str, str]]) -> Path:
    """A fake ``/sys/class/hwmon`` tree, read back by the real reader.

    The board's under-voltage-only source (item 97): built here rather than
    hand-writing the partial reading it produces, so these rules are exercised on
    exactly what ``hostinfo`` hands them.
    """
    for entry, files in devices.items():
        for name, text in files.items():
            path = root / "hwmon" / entry / name
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(text)
    return root / "hwmon"


def _host_info(**over: Any) -> dict[str, Any]:
    """The board at the idle the owner measured: 47.2 degC, nothing throttling."""
    base: dict[str, Any] = {"cpu_temp_c": 47.2, "load1": 0.1, "throttled": decode_throttled(0)}
    base.update(over)
    return base


def _board(settings: HostHealthConfig | None = None) -> HostHealth:
    return HostHealth(settings or HostHealthConfig(), air_temps=("air_z1", "air_z2"))


def test_an_idle_board_next_to_the_air_has_no_problem() -> None:
    board = _board()
    verdict = board.check(_host_info(), {"air_z1": 27.0, "air_z2": 29.0}, 0.0)
    assert verdict["ok"] is True and verdict["problems"] == []
    assert verdict["air_c"] == pytest.approx(28.0)
    assert verdict["divergence_c"] == pytest.approx(19.2)
    assert verdict["idle"] is True


def test_the_owners_own_idle_board_against_room_air_stands_clear_of_the_default() -> None:
    """The shipped divergence_c must not sit permanently tripped on a healthy board.

    The owner's Zero 2 W reads 47.2 degC at idle, and an un-heatsinked Zero 2 W idles
    roughly 20-25 degC above the air around it -- so against a room-temperature air
    reference (an operator who names the intake, or a config whose only air role is
    inlet) a *healthy* board is already ~25 degC away. The default must clear that by
    a real margin, for as long as the board stays where it is.
    """
    board = _board()  # the shipped defaults
    air = {"air_z1": 22.0, "air_z2": 22.0}  # room air at the intake
    for t in (0.0, 900.0, 86_400.0):
        verdict = board.check(_host_info(), air, t)
        assert verdict["divergence_c"] == pytest.approx(25.2)
        assert verdict["idle"] is True
        assert verdict["problems"] == [], "a healthy idle board must not be a standing alarm"
    assert HostHealthConfig().divergence_c - 25.2 >= 10.0


def test_a_hot_board_is_reported_only_after_temp_fault_s_and_clears() -> None:
    board = _board(HostHealthConfig(temp_limit_c=75.0, temp_fault_s=120.0))
    hot = _host_info(cpu_temp_c=82.0)
    assert board.check(hot, {}, 0.0)["problems"] == []
    assert board.check(hot, {}, 119.0)["problems"] == []
    problems = board.check(hot, {}, 120.0)["problems"]
    assert len(problems) == 1 and "82.0 degC, above the 75 degC limit" in problems[0]
    # back under the limit: the window is forgotten, not merely paused
    assert board.check(_host_info(), {}, 121.0)["problems"] == []
    assert board.check(hot, {}, 240.0)["problems"] == []


def test_a_missing_board_temperature_never_fires_the_temperature_rule() -> None:
    board = _board(HostHealthConfig(temp_fault_s=1.0))
    verdict = board.check(_host_info(cpu_temp_c=None), {}, 0.0)
    assert verdict["cpu_temp_c"] is None
    assert board.check(_host_info(cpu_temp_c=None), {}, 1000.0)["problems"] == []


def test_throttling_now_is_reported_the_tick_it_is_seen_and_clears() -> None:
    """The firmware has already latched it; a sustained window would only delay a fact."""
    board = _board()
    verdict = board.check(_host_info(throttled=decode_throttled(0x4)), {}, 0.0)
    assert len(verdict["problems"]) == 1
    assert "throttling now (throttled" in verdict["problems"][0]
    assert verdict["ok"] is False
    assert board.check(_host_info(), {}, 1.0)["problems"] == []


def test_the_under_voltage_alarm_alone_raises_the_rule(tmp_path: Path) -> None:
    """The hwmon source sees one condition, and that is enough to report it.

    On a board with no ``get_throttled`` sysfs attribute and no ``vcgencmd`` this
    partial reading is the whole evidence there is; the rule must fire on it, and
    say plainly which conditions it could not see.
    """
    board = _board()
    partial = read_rpi_volt_hwmon(
        _hwmon(tmp_path, {"hwmon2": {"name": "rpi_volt\n", "in0_lcrit_alarm": "1\n"}})
    )
    verdict = board.check(_host_info(throttled=partial), {}, 0.0)
    assert len(verdict["problems"]) == 1 and verdict["ok"] is False
    message = verdict["problems"][0]
    assert "throttling now (under_voltage" in message
    assert "source hwmon" in message
    assert "freq_capped, throttled, soft_temp_limit not read" in message


def test_a_condition_nobody_read_is_never_reported_as_absent(tmp_path: Path) -> None:
    """An unknown bit must not read as false: the same partial source, alarm clear."""
    board = _board()
    partial = read_rpi_volt_hwmon(
        _hwmon(tmp_path, {"hwmon2": {"name": "rpi_volt\n", "in0_lcrit_alarm": "0\n"}})
    )
    verdict = board.check(_host_info(throttled=partial), {}, 0.0)
    assert verdict["problems"] == [], "one clear bit is not evidence that the rest are clear"
    published = verdict["throttled"]
    assert published["now"] is None, "not False -- three conditions were not read"
    assert published["throttled_now"] is None and published["partial"] is True


def test_a_cached_vcgencmd_word_says_how_old_it_is(tmp_path: Path) -> None:
    """The word may be up to vcgencmd_interval_s old; the message says so."""
    board = _board()
    aged = decode_throttled(0x1, source="vcgencmd", age_s=45.0)
    message = board.check(_host_info(throttled=aged), {}, 0.0)["problems"][0]
    assert "source vcgencmd" in message and "read 45 s ago" in message


def test_the_since_boot_half_alone_never_warns() -> None:
    """An under-voltage during boot is history, published but not a live problem."""
    board = _board()
    verdict = board.check(_host_info(throttled=decode_throttled(0x50000)), {}, 0.0)
    assert verdict["problems"] == [] and verdict["ok"] is True
    assert verdict["throttled"]["under_voltage_since_boot"] is True


def test_an_unreadable_throttled_word_never_fires_the_rule() -> None:
    board = _board()
    verdict = board.check(_host_info(throttled=None), {}, 0.0)
    assert verdict["throttled"] is None and verdict["problems"] == []


def test_a_divergence_at_idle_is_reported_after_divergence_fault_s_and_clears() -> None:
    board = _board()  # the defaults: 40 degC for 900 s
    far = _host_info(cpu_temp_c=70.0)  # 44 degC above the air, idle
    air = {"air_z1": 26.0, "air_z2": 26.0}
    assert board.check(far, air, 0.0)["problems"] == []
    assert board.check(far, air, 899.0)["problems"] == []
    problems = board.check(far, air, 900.0)["problems"]
    assert len(problems) == 1 and "hint about the air sensors" in problems[0]
    assert "not a verdict" in problems[0]
    assert board.check(_host_info(), air, 901.0)["problems"] == []


def test_the_divergence_rule_is_judged_on_the_absolute_difference() -> None:
    """Air reading above the board is evidence too -- of the sensor, most likely."""
    board = _board(HostHealthConfig(divergence_fault_s=1.0))
    air = {"air_z1": 90.0, "air_z2": 90.0}
    board.check(_host_info(), air, 0.0)
    verdict = board.check(_host_info(), air, 1.0)
    assert verdict["divergence_c"] == pytest.approx(-42.8)
    assert (
        len(verdict["problems"]) == 1
        and "-42.8 degC from the enclosure air" in (verdict["problems"][0])
    )


def test_a_busy_cpu_gates_the_divergence_rule_entirely() -> None:
    """The board's reading is dominated by its own self-heating, which moves with load."""
    board = _board(HostHealthConfig(divergence_fault_s=1.0, idle_load1_max=0.5))
    air = {"air_z1": 26.0, "air_z2": 26.0}
    busy = _host_info(cpu_temp_c=70.0, load1=2.0)
    for t in (0.0, 10.0, 1000.0):
        verdict = board.check(busy, air, t)
        assert verdict["idle"] is False and verdict["problems"] == []
    # and a busy tick in the middle starts the window again
    idle = _host_info(cpu_temp_c=70.0)
    board.check(idle, air, 1001.0)
    board.check(busy, air, 1002.0)
    assert board.check(idle, air, 1003.0)["problems"] == []
    assert len(board.check(idle, air, 1004.0)["problems"]) == 1


def test_an_unknown_load_average_gates_the_divergence_rule() -> None:
    board = _board(HostHealthConfig(divergence_fault_s=0.001))
    air = {"air_z1": 26.0, "air_z2": 26.0}
    verdict = board.check(_host_info(cpu_temp_c=70.0, load1=None), air, 0.0)
    assert verdict["idle"] is None and verdict["problems"] == []


def test_without_an_air_reading_the_divergence_rule_is_off() -> None:
    board = _board(HostHealthConfig(divergence_fault_s=0.001))
    verdict = board.check(_host_info(cpu_temp_c=70.0), {"somewhere_else": 26.0}, 0.0)
    assert verdict["air_c"] is None and verdict["divergence_c"] is None
    assert board.check(_host_info(cpu_temp_c=70.0), {}, 10.0)["problems"] == []


def test_air_temps_from_the_config_override_the_default_reference() -> None:
    settings = HostHealthConfig(air_temps=("air_z3",))
    board = HostHealth(settings, air_temps=("air_z1",))
    assert board.air_temps == ("air_z3",)
    assert board.check(_host_info(), {"air_z1": 5.0, "air_z3": 30.0}, 0.0)["air_c"] == 30.0


def test_host_health_disabled_still_publishes_the_numbers_but_runs_no_rule() -> None:
    board = _board(HostHealthConfig(enabled=False, temp_fault_s=0.001))
    verdict = board.check(_host_info(cpu_temp_c=95.0, throttled=decode_throttled(0x7)), {}, 10.0)
    assert verdict["cpu_temp_c"] == 95.0 and verdict["throttled"]["throttled_now"] is True
    assert verdict["problems"] == [] and verdict["ok"] is True


def test_a_repeated_board_problem_is_logged_at_most_once_per_log_interval_s(
    caplog: pytest.LogCaptureFixture,
) -> None:
    board = _board(HostHealthConfig(log_interval_s=300.0))
    hot = _host_info(throttled=decode_throttled(0x4))
    with caplog.at_level(logging.WARNING, logger="aqua_bridge.health"):
        for t in (0.0, 10.0, 20.0, 400.0):
            board.check(hot, {}, t)
    lines = [r for r in caplog.records if "host health" in r.getMessage()]
    assert len(lines) == 2
    assert "item 97" in lines[0].getMessage()


def test_on_tick_publishes_the_board_verdict_next_to_the_fans() -> None:
    """The host verdict rides the same payload, and its problems join the same list."""
    published: list[dict[str, Any]] = []
    mon = HealthMonitor(
        _cfg(),
        FanHealthConfig(),
        publish=published.append,
        host_settings=HostHealthConfig(temp_fault_s=0.001),
        hostinfo=lambda: _host_info(cpu_temp_c=88.0),
    )
    obs = PlantObservation(temps={"air_z1": 26.0}, rpm={}, pwm={}, ts=0.0, inputs={})
    mon.on_tick(_tick(obs))
    mon.on_tick(_tick(PlantObservation(temps={"air_z1": 26.0}, rpm={}, pwm={}, ts=5.0, inputs={})))
    payload = published[-1]
    assert payload["host"]["cpu_temp_c"] == 88.0
    assert payload["host"]["ok"] is False
    assert payload["problems"] == payload["host"]["problems"]
    assert payload["host"]["faults"] == payload["host"]["problems"]
    assert payload["ok"] is False


def test_the_divergence_hint_does_not_make_the_daemon_not_ok() -> None:
    """A hint says where to look; it must not read like a controller that is broken.

    It belongs to the board's own verdict (and to the host_problem sensor templated
    off it), never to the daemon-wide problems list behind /api/health and Home
    Assistant's device_problem -- where it would be indistinguishable from an aquabus
    device that has gone missing.
    """
    published: list[dict[str, Any]] = []
    mon = HealthMonitor(
        _cfg(),
        FanHealthConfig(),
        publish=published.append,
        host_settings=HostHealthConfig(divergence_fault_s=0.001),
        hostinfo=lambda: _host_info(cpu_temp_c=90.0),
    )
    mon.host.air_temps = ("air_z1",)  # what a zone_air role gives on a real config
    for ts in (0.0, 5.0):
        mon.on_tick(_tick(PlantObservation(temps={"air_z1": 26.0}, rpm={}, pwm={}, ts=ts)))
    board = published[-1]["host"]
    assert board["faults"] == [] and len(board["hints"]) == 1
    assert board["problems"] == board["hints"] and board["ok"] is False
    assert published[-1]["problems"] == [] and published[-1]["ok"] is True


def test_a_hot_board_is_a_daemon_problem_and_the_hint_rides_along() -> None:
    """The facts (hot, throttling now) do join the one problems list."""
    board = _board(HostHealthConfig(temp_fault_s=0.001, divergence_fault_s=0.001))
    hot = _host_info(cpu_temp_c=90.0)
    air = {"air_z1": 26.0, "air_z2": 26.0}
    board.check(hot, air, 0.0)
    verdict = board.check(hot, air, 1.0)
    assert len(verdict["faults"]) == 1 and "above the 75 degC limit" in verdict["faults"][0]
    assert len(verdict["hints"]) == 1 and "not a verdict" in verdict["hints"][0]
    assert verdict["problems"] == [*verdict["faults"], *verdict["hints"]]


def test_on_tick_without_a_hostinfo_reader_publishes_an_empty_board_verdict() -> None:
    published: list[dict[str, Any]] = []
    mon = HealthMonitor(_cfg(), FanHealthConfig(), publish=published.append)
    mon.on_tick(_tick(PlantObservation(temps={}, rpm={}, pwm={}, ts=0.0, inputs={})))
    host = published[-1]["host"]
    assert host["cpu_temp_c"] is None and host["throttled"] is None and host["ok"] is True


def test_on_tick_survives_a_hostinfo_reader_that_raises() -> None:
    def boom() -> dict[str, Any]:
        raise RuntimeError("no /proc here")

    published: list[dict[str, Any]] = []
    mon = HealthMonitor(_cfg(), FanHealthConfig(), publish=published.append, hostinfo=boom)
    mon.on_tick(_tick(PlantObservation(temps={}, rpm={}, pwm={}, ts=0.0, inputs={})))
    assert published[-1]["host"]["cpu_temp_c"] is None


def test_the_board_never_reaches_the_observation_or_the_solver_diagnostics() -> None:
    """Item 97: a health signal, never a model input. The observation the monitor was
    given must come back untouched, and nothing the board reports may be in it."""
    obs = PlantObservation(temps={"air_z1": 26.0}, rpm={}, pwm={}, ts=0.0, inputs={})
    before = obs.to_dict()
    mon = HealthMonitor(
        _cfg(),
        FanHealthConfig(),
        host_settings=HostHealthConfig(temp_fault_s=0.001),
        hostinfo=lambda: _host_info(cpu_temp_c=95.0),
    )
    mon.on_tick(_tick(obs))
    assert obs.to_dict() == before
    assert "cpu_temp_c" not in obs.temps and "host" not in obs.inputs


# --- the example configs --------------------------------------------------------------


@pytest.mark.parametrize("name", ["config.example.yaml", "config.example-das.yaml"])
def test_both_example_configs_show_every_key_at_its_default(name: str) -> None:
    """Every operator tunable is a documented key with one default: both examples must
    list all of them, and list the values the code actually uses."""
    app = load_config(Path(__file__).resolve().parent.parent / name)
    section = app.section("fan_health")
    assert set(section) == set(FANHEALTH_KEYS), name
    assert FanHealthConfig.from_section(section) == FanHealthConfig(), name
    host_section = app.section("host_health")
    assert set(host_section) == set(HOSTHEALTH_KEYS), name
    assert HostHealthConfig.from_section(host_section) == HostHealthConfig(), name
