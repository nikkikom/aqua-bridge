"""Contract tests for ``aqua_bridge.model`` and ``aqua_bridge.config``.

One test per config rejection rule in PROJECT.md section 4.3 ("Config /
solver"), malformed-observation construction (section 4.5), and
to_dict/from_dict round-trips.
"""

from __future__ import annotations

import dataclasses
import json
import logging
import math

import pytest
import yaml

from aqua_bridge.config import AppConfig, load_config
from aqua_bridge.control.intents import (
    ClearOverride,
    ControlMode,
    ControlSnapshot,
    ControlSurface,
    IntentConflict,
    IntentInvalid,
    Preset,
    SetMode,
    SetPreset,
    SetPwm,
    SetSetpoint,
    SolverStatus,
    parse_intent,
)
from aqua_bridge.model import (
    ConfigError,
    FaultReason,
    Mode,
    MpcCommand,
    MpcConfig,
    MpcState,
    PlantObservation,
    SolverKind,
    WindowSample,
)
from invariants import (
    assert_command_safe,
    assert_state_finite,
    obs_structurally_untrusted,
    resolve_prev_pwm,
)

# ---------------------------------------------------------------------------
# Config: example file and derived quantities
# ---------------------------------------------------------------------------


def test_example_config_loads_and_validates(example_config_path):
    app = load_config(example_config_path)
    assert isinstance(app, AppConfig)
    assert isinstance(app.mpc, MpcConfig)
    app.mpc.validate()
    assert app.mpc.channels == ("radiator", "intake")
    assert app.mpc.temps == ("coolant", "air")
    assert app.mpc.solver is SolverKind.PI
    assert app.xt6["device"] == "aquaero"
    assert "port" in app.section("http")  # validated in tests/test_http_auth.py
    assert app.source == str(example_config_path)


def test_example_config_has_every_mpc_field_documented(example_config_path):
    raw = yaml.safe_load(example_config_path.read_text())["mpc"]
    assert set(raw) == {f.name for f in dataclasses.fields(MpcConfig)}


def test_derived_tick_quantities(cfg):
    assert cfg.dT_max_tick == pytest.approx(cfg.dT_max_c_per_s * cfg.dt)
    assert cfg.confirm_ticks == max(2, math.ceil(cfg.confirm_s / cfg.dt))
    assert cfg.stuck_ticks == max(2, math.ceil(cfg.stuck_s / cfg.dt))
    small = dataclasses.replace(cfg, dt=1.0, confirm_s=2.0, stuck_s=2.0)
    assert small.confirm_ticks == 2
    assert small.stuck_ticks == 2
    assert small.dT_max_tick == pytest.approx(cfg.dT_max_c_per_s)


def test_temps_for_channel_default_and_explicit(cfg):
    plain = dataclasses.replace(cfg, channel_temps={})
    assert plain.temps_for_channel("radiator") == ("coolant",)
    both = dataclasses.replace(
        cfg,
        setpoints={"coolant": 35.0, "air": 30.0},
        channel_temps={"radiator": ["coolant"], "intake": ["air", "coolant"]},
    )
    assert both.temps_for_channel("intake") == ("air", "coolant")
    with pytest.raises(KeyError):
        cfg.temps_for_channel("nope")
    assert cfg.weight_for("air") == 1.0


def test_config_to_dict_round_trip(cfg):
    d = cfg.to_dict()
    json.dumps(d)
    assert MpcConfig.from_mapping(d) == cfg


# ---------------------------------------------------------------------------
# Config: rejection rules (section 4.3 "Config / solver" + new fields)
# ---------------------------------------------------------------------------

REJECT_CASES = [
    pytest.param({"dt": 0.0}, id="dt_zero"),
    pytest.param({"dt": -1.0}, id="dt_negative"),
    pytest.param({"dt": float("nan")}, id="dt_nan"),
    pytest.param({"dt": "2.0"}, id="dt_string"),
    pytest.param({"horizon": 0}, id="horizon_zero"),
    pytest.param({"horizon": 2.5}, id="horizon_float"),
    pytest.param({"horizon": True}, id="horizon_bool"),
    pytest.param({"pwm_min": 0.9, "pwm_max": 0.5}, id="pwm_min_gt_pwm_max"),
    pytest.param({"pwm_min": 0.5, "pwm_max": 0.5}, id="pwm_min_eq_pwm_max"),
    pytest.param({"pwm_min": -0.1}, id="pwm_min_below_zero"),
    pytest.param({"pwm_max": 1.1}, id="pwm_max_above_one"),
    pytest.param({"d_pwm_max": 0.0}, id="d_pwm_max_zero"),
    pytest.param({"d_pwm_max": -0.1}, id="d_pwm_max_negative"),
    pytest.param({"channels": []}, id="empty_channels"),
    pytest.param({"channels": ["radiator", "radiator"]}, id="duplicate_channels"),
    pytest.param({"channels": "radiator"}, id="channels_string"),
    pytest.param({"fallback_pwm": {"radiator": 0.8}}, id="fallback_pwm_missing_channel"),
    pytest.param(
        {"fallback_pwm": {"radiator": 0.8, "intake": 0.8, "exhaust": 0.8}},
        id="fallback_pwm_extra_channel",
    ),
    pytest.param({"fallback_pwm": {"radiator": 0.15, "intake": 0.8}}, id="fallback_pwm_eq_min"),
    pytest.param({"fallback_pwm": {"radiator": 0.1, "intake": 0.8}}, id="fallback_pwm_below_min"),
    pytest.param({"fallback_pwm": {"radiator": 1.01, "intake": 0.8}}, id="fallback_pwm_above_max"),
    pytest.param({"fallback_hold_s": -1.0}, id="fallback_hold_negative"),
    pytest.param({"confirm_s": 3.9}, id="confirm_s_below_2dt"),
    pytest.param({"confirm_s": 12.0}, id="confirm_s_gt_fallback_hold"),
    pytest.param({"dT_max_c_per_s": 0.0}, id="dT_max_zero"),
    pytest.param({"dT_max_c_per_s": -2.0}, id="dT_max_negative"),
    pytest.param({"stuck_s": 3.9}, id="stuck_s_below_2dt"),
    pytest.param({"temps": []}, id="empty_temps"),
    pytest.param({"temps": ["coolant", "coolant"]}, id="duplicate_temps"),
    pytest.param({"setpoints": {"water": 35.0}}, id="setpoint_not_in_temps"),
    pytest.param({"setpoints": {}}, id="empty_setpoints"),
    pytest.param({"setpoints": {"coolant": 200.0}}, id="setpoint_outside_valid_range"),
    pytest.param({"setpoints": {"coolant": "35"}}, id="setpoint_string"),
    pytest.param({"stuck_eps_c": 0.0}, id="stuck_eps_zero"),
    pytest.param({"stuck_pwm_net": 0.0}, id="stuck_pwm_net_zero"),
    pytest.param({"stuck_pwm_net": 1.5}, id="stuck_pwm_net_above_one"),
    pytest.param({"stuck_sibling_dT_c": 0.0}, id="stuck_sibling_zero"),
    pytest.param({"median3": "false"}, id="median3_string"),
    pytest.param({"median3": 0}, id="median3_int"),
    pytest.param({"weights": {"water": 1.0}}, id="weight_unknown_temp"),
    pytest.param({"weights": {"coolant": -1.0}}, id="weight_negative"),
    pytest.param({"weights": {"coolant": float("inf")}}, id="weight_inf"),
    pytest.param({"weight_pwm": -0.1}, id="weight_pwm_negative"),
    pytest.param({"weight_dpwm": float("nan")}, id="weight_dpwm_nan"),
    pytest.param({"solver": "casadi"}, id="solver_unknown"),
    pytest.param({"pi_kp": 0.0}, id="pi_kp_zero"),
    pytest.param({"pi_ki": -1.0}, id="pi_ki_negative"),
    pytest.param({"channel_temps": {"radiator": ["coolant"]}}, id="channel_temps_missing_channel"),
    pytest.param(
        {"channel_temps": {"radiator": ["coolant"], "intake": ["air"]}},
        id="channel_temps_temp_without_setpoint",
    ),
    pytest.param(
        {"channel_temps": {"radiator": [], "intake": ["coolant"]}}, id="channel_temps_empty"
    ),
    pytest.param({"temp_min_c": 130.0}, id="temp_range_inverted"),
    pytest.param({"solver_max_iter": 0}, id="solver_max_iter_zero"),
]


@pytest.mark.parametrize("changes", REJECT_CASES)
def test_config_rejects(cfg, changes):
    with pytest.raises(ConfigError):
        dataclasses.replace(cfg, **changes)


def test_config_error_is_value_error():
    assert issubclass(ConfigError, ValueError)


def test_config_accepts_large_horizon_and_horizon_one(cfg):
    assert dataclasses.replace(cfg, horizon=1).horizon == 1
    assert dataclasses.replace(cfg, horizon=500).horizon == 500


def test_config_coerces_lists_and_ints(cfg):
    d = cfg.to_dict()
    d["temps"] = ["coolant", "air"]
    d["fallback_hold_s"] = 10  # int in YAML
    d["solver"] = "mpc"
    c = MpcConfig.from_mapping(d)
    assert c.temps == ("coolant", "air")
    assert isinstance(c.fallback_hold_s, float)
    assert c.solver is SolverKind.MPC


def test_from_mapping_rejects_unknown_and_missing_keys(cfg):
    d = cfg.to_dict()
    d["confirm_ticks"] = 3
    with pytest.raises(ConfigError, match="unknown mpc keys"):
        MpcConfig.from_mapping(d)
    d = cfg.to_dict()
    del d["dt"]
    with pytest.raises(ConfigError, match="missing mpc keys"):
        MpcConfig.from_mapping(d)
    with pytest.raises(ConfigError):
        MpcConfig.from_mapping(["not", "a", "mapping"])  # type: ignore[arg-type]


def test_app_config_from_mapping_and_errors(cfg):
    app = AppConfig.from_mapping({"mpc": cfg.to_dict(), "http": {"port": 1}, "custom": {"a": 1}})
    assert app.mpc == cfg
    assert app.http == {"port": 1}
    assert app.mqtt == {}
    assert app.section("custom") == {"a": 1}
    with pytest.raises(ConfigError):
        AppConfig.from_mapping({"http": {}})
    with pytest.raises(ConfigError):
        AppConfig.from_mapping({"mpc": cfg.to_dict(), "http": "not a mapping"})
    with pytest.raises(ConfigError):
        AppConfig.from_mapping([])  # type: ignore[arg-type]


def test_aquacomputer_section_is_a_list_of_mappings(cfg):
    entries = [{"device": "aquaero", "fans": {}}, {"device": "quadro"}]
    app = AppConfig.from_mapping({"mpc": cfg.to_dict(), "aquacomputer": entries})
    assert app.aquacomputer == tuple(entries)
    assert "aquacomputer" not in app.extra
    assert AppConfig.from_mapping({"mpc": cfg.to_dict()}).aquacomputer == ()
    for bad, match in (
        ({"device": "aquaero"}, "'aquacomputer' must be a list"),
        (["aquaero"], r"'aquacomputer'\[0\] must be a mapping"),
        ([{1: "x"}], r"'aquacomputer'\[0\] has a non-string key"),
    ):
        with pytest.raises(ConfigError, match=match):
            AppConfig.from_mapping({"mpc": cfg.to_dict(), "aquacomputer": bad})


def test_former_hwmon_section_names_its_replacement(cfg):
    with pytest.raises(ConfigError, match="'hwmon' was renamed to 'aquacomputer'.*'device:'"):
        AppConfig.from_mapping({"mpc": cfg.to_dict(), "hwmon": [{"name": "aquaero"}]})


def test_example_configs_show_every_timing_key_at_its_default(
    example_config_path, example_das_config_path
):
    """The example files document each controller timing key with the one default
    the config model holds for that device kind (AquacomputerTiming.for_kind)."""
    from aqua_bridge.hw.aquacomputer_adapter import TIMING_KEYS, AquacomputerTiming

    legacy = load_config(example_config_path).xt6
    (aquaero,) = load_config(example_das_config_path).aquacomputer  # the Quadro on aquabus
    for entry in (legacy, aquaero):
        defaults = dataclasses.asdict(AquacomputerTiming.for_kind(entry["device"]))
        shown = {key: entry[key] for key in TIMING_KEYS if key in entry}
        assert shown == {key: defaults[key] for key in shown}
    assert set(TIMING_KEYS) <= set(legacy) and set(TIMING_KEYS) <= set(aquaero)
    # The Quadro-on-USB alternative is commented out; its gap default is shown there.
    text = example_das_config_path.read_text()
    quadro_gap = AquacomputerTiming.for_kind("quadro").ctrl_gap_ms
    assert f"#   ctrl_gap_ms: {quadro_gap:g}" in text


def test_digole_enabled_bad_type_logs_a_warning_and_does_not_raise(cfg, caplog):
    """Item 57: digole: has no owner module yet, so a bad enabled: is logged, not
    fatal -- a typo there must never stop the daemon from controlling the fans."""
    with caplog.at_level(logging.WARNING):
        app = AppConfig.from_mapping({"mpc": cfg.to_dict(), "digole": {"enabled": "true"}})
    assert app.digole == {"enabled": "true"}  # unchanged: still nobody's job to fix it up
    assert any(
        "digole.enabled" in r.getMessage() and "'true'" in r.getMessage() for r in caplog.records
    )


def test_digole_enabled_proper_bool_is_silent(cfg, caplog):
    with caplog.at_level(logging.WARNING):
        AppConfig.from_mapping({"mpc": cfg.to_dict(), "digole": {"enabled": True}})
    assert not caplog.records


def test_load_config_errors(tmp_path):
    with pytest.raises(ConfigError):
        load_config(tmp_path / "missing.yaml")
    bad = tmp_path / "bad.yaml"
    bad.write_text("mpc: [unclosed")
    with pytest.raises(ConfigError):
        load_config(bad)
    empty = tmp_path / "empty.yaml"
    empty.write_text("")
    with pytest.raises(ConfigError):
        load_config(empty)


def test_a_key_written_twice_is_refused_instead_of_silently_dropped(
    tmp_path, example_das_config_path
):
    """Item 73: plain YAML keeps the *last* of repeated keys and drops the rest without
    a word, so a raised ``mpc.budget_ms`` written next to the shipped one would load
    cleanly and change nothing. The loader names the key and the line instead."""
    text = example_das_config_path.read_text()
    assert "  budget_ms: 600.0" in text
    doubled = tmp_path / "doubled.yaml"
    doubled.write_text(
        text.replace("  budget_ms: 600.0", "  budget_ms: 1000.0\n  budget_ms: 600.0", 1)
    )
    with pytest.raises(ConfigError, match="'budget_ms' is written twice"):
        load_config(doubled)
    # A repeated *section* is refused the same way, and one copy of a key still loads.
    twice = tmp_path / "twice.yaml"
    twice.write_text(text + "\nmpc: {}\n")
    with pytest.raises(ConfigError, match="'mpc' is written twice"):
        load_config(twice)
    assert load_config(example_das_config_path).mpc.budget_ms == 600.0


# ---------------------------------------------------------------------------
# PlantObservation construction
# ---------------------------------------------------------------------------


def good_obs(ts: float = 0.0) -> PlantObservation:
    return PlantObservation(
        temps={"coolant": 35.0, "air": 25.0},
        rpm={"radiator": 900.0, "intake": 800.0},
        pwm={"radiator": 0.5, "intake": 0.5},
        ts=ts,
    )


def test_observation_accepts_none_and_nan_values():
    obs = PlantObservation(
        temps={"coolant": None, "air": float("nan")}, rpm={}, pwm={"radiator": 1}, ts=1
    )
    assert obs.temps["coolant"] is None
    assert math.isnan(obs.temps["air"])
    assert obs.pwm["radiator"] == 1.0 and isinstance(obs.pwm["radiator"], float)
    assert obs.ts == 1.0 and isinstance(obs.ts, float)


@pytest.mark.parametrize(
    "kwargs",
    [
        pytest.param({"temps": ["coolant", 35.0]}, id="temps_list"),
        pytest.param({"temps": None}, id="temps_none"),
        pytest.param({"temps": {1: 35.0}}, id="temps_int_key"),
        pytest.param({"temps": {"coolant": "35"}}, id="temps_string_value"),
        pytest.param({"temps": {"coolant": True}}, id="temps_bool_value"),
        pytest.param({"temps": {"coolant": [35.0]}}, id="temps_list_value"),
        pytest.param({"rpm": "900"}, id="rpm_string"),
        pytest.param({"pwm": {"radiator": {"v": 0.5}}}, id="pwm_dict_value"),
        pytest.param({"ts": None}, id="ts_none"),
        pytest.param({"ts": "0"}, id="ts_string"),
        pytest.param({"ts": True}, id="ts_bool"),
    ],
)
def test_observation_rejects_malformed_structure_type(kwargs):
    base = {"temps": {"coolant": 35.0}, "rpm": {}, "pwm": {}, "ts": 0.0}
    base.update(kwargs)
    with pytest.raises(TypeError):
        PlantObservation(**base)


@pytest.mark.parametrize("ts", [float("nan"), float("inf"), float("-inf")])
def test_observation_rejects_non_finite_ts(ts):
    with pytest.raises(ValueError):
        PlantObservation(temps={}, rpm={}, pwm={}, ts=ts)


def test_observation_rejects_extra_fields():
    with pytest.raises(TypeError):
        PlantObservation(temps={}, rpm={}, pwm={}, ts=0.0, extra=1)  # type: ignore[call-arg]


def test_observation_is_frozen_and_copies_input():
    src = {"coolant": 35.0}
    obs = PlantObservation(temps=src, rpm={}, pwm={}, ts=0.0)
    src["coolant"] = 99.0
    assert obs.temps["coolant"] == 35.0
    with pytest.raises(dataclasses.FrozenInstanceError):
        obs.ts = 1.0  # type: ignore[misc]


# ---------------------------------------------------------------------------
# MpcCommand / MpcState
# ---------------------------------------------------------------------------


def test_command_mode_coercion_and_rejection():
    cmd = MpcCommand(pwm={"radiator": 0.5}, mode="auto")
    assert cmd.mode is Mode.AUTO
    assert cmd.diagnostics == {}
    with pytest.raises(ValueError):
        MpcCommand(pwm={"radiator": 0.5}, mode="manual")
    with pytest.raises(TypeError):
        MpcCommand(pwm={"radiator": None}, mode=Mode.AUTO)  # type: ignore[dict-item]
    with pytest.raises(TypeError):
        MpcCommand(pwm=[0.5], mode=Mode.AUTO)  # type: ignore[arg-type]
    with pytest.raises(TypeError):
        MpcCommand(pwm={"radiator": 0.5}, mode=Mode.AUTO, diagnostics={1: 2})  # type: ignore[dict-item]


def test_command_round_trip():
    cmd = MpcCommand(
        pwm={"radiator": 0.5, "intake": 0.6},
        mode=Mode.SATURATED,
        diagnostics={"err": {"coolant": 1.5}, "iters": 3, "note": "x", "flags": [1, 2]},
    )
    text = json.dumps(cmd.to_dict())
    assert MpcCommand.from_dict(json.loads(text)) == cmd


def test_state_cold():
    s = MpcState.cold()
    assert s.last_cmd is None and s.last_good_obs is None and s.last_raw_temps is None
    assert s.window == () and s.fault_since_ts is None and s.fault_reason is None
    assert s.trusted_streak == 0 and s.integrator == {} and s.solver_memory == {}
    assert not s.in_fault
    assert_state_finite(s)
    assert MpcState.from_dict(json.loads(json.dumps(s.to_dict()))) == s


def test_state_round_trip_full():
    obs = good_obs(4.0)
    cmd = MpcCommand(pwm={"radiator": 0.7, "intake": 0.6}, mode="fallback", diagnostics={"k": 1})
    s = MpcState(
        last_cmd=cmd,
        last_good_obs=obs,
        last_raw_temps={"coolant": 36.0, "air": None},
        window=(
            WindowSample(raw_temps={"coolant": 35.0, "air": 25.0}, cmd_pwm=cmd.pwm),
            WindowSample(raw_temps={"coolant": None, "air": 25.5}, cmd_pwm=cmd.pwm),
        ),
        fault_since_ts=2.0,
        fault_reason="sensor_gate",
        trusted_streak=1,
        integrator={"radiator": 0.01, "intake": -0.02},
        solver_memory={"warm_start": [0.1, 0.2], "iters": 4},
    )
    assert s.fault_reason is FaultReason.SENSOR_GATE
    assert s.in_fault
    text = json.dumps(s.to_dict())
    back = MpcState.from_dict(json.loads(text))
    assert back == s
    assert_state_finite(back)


def test_state_rejects_bad_fields():
    with pytest.raises(TypeError):
        MpcState(last_cmd={"pwm": {}})  # type: ignore[arg-type]
    with pytest.raises(TypeError):
        MpcState(window=[("a", "b")])  # type: ignore[list-item]
    with pytest.raises(ValueError):
        MpcState(fault_reason="cosmic_rays")
    with pytest.raises(ValueError):
        MpcState(trusted_streak=-1)
    with pytest.raises(TypeError):
        MpcState(trusted_streak=1.5)  # type: ignore[arg-type]
    with pytest.raises(TypeError):
        MpcState(integrator={"radiator": None})  # type: ignore[dict-item]


def test_state_is_immutable_and_replaceable():
    s = MpcState.cold()
    with pytest.raises(dataclasses.FrozenInstanceError):
        s.trusted_streak = 3  # type: ignore[misc]
    s2 = dataclasses.replace(s, trusted_streak=3)
    assert s2.trusted_streak == 3 and s.trusted_streak == 0


# ---------------------------------------------------------------------------
# Invariant helpers
# ---------------------------------------------------------------------------


def test_resolve_prev_pwm_order(cfg):
    obs = good_obs()
    cmd = MpcCommand(pwm={"radiator": 0.9, "intake": 0.4}, mode=Mode.AUTO)
    assert resolve_prev_pwm(MpcState(last_cmd=cmd), obs, cfg) == cmd.pwm
    assert resolve_prev_pwm(MpcState.cold(), obs, cfg) == {"radiator": 0.5, "intake": 0.5}
    no_pwm = PlantObservation(temps=obs.temps, rpm={}, pwm={}, ts=0.0)
    assert resolve_prev_pwm(MpcState.cold(), no_pwm, cfg) == cfg.fallback_pwm
    bad_pwm = PlantObservation(temps=obs.temps, rpm={}, pwm={"radiator": 1.5, "intake": 0.5}, ts=0)
    assert resolve_prev_pwm(MpcState.cold(), bad_pwm, cfg) == cfg.fallback_pwm
    none_pwm = PlantObservation(
        temps=obs.temps, rpm={}, pwm={"radiator": None, "intake": 0.5}, ts=0
    )
    assert resolve_prev_pwm(MpcState.cold(), none_pwm, cfg) == cfg.fallback_pwm


def test_assert_command_safe_accepts_good_command(cfg):
    obs = good_obs()
    prev = {"radiator": 0.5, "intake": 0.5}
    cmd = MpcCommand(pwm={"radiator": 0.6, "intake": 0.4}, mode=Mode.AUTO, diagnostics={"e": 1.0})
    assert_command_safe(obs, cfg, cmd, prev)
    assert_command_safe(obs, cfg, cmd, MpcCommand(pwm=prev, mode=Mode.AUTO))
    fb = MpcCommand(pwm=dict(cfg.fallback_pwm), mode=Mode.FALLBACK)
    assert_command_safe(obs, cfg, fb, cfg.fallback_pwm)


@pytest.mark.parametrize(
    ("pwm", "mode", "diag"),
    [
        pytest.param({"radiator": 0.6}, "auto", {}, id="missing_channel"),
        pytest.param({"radiator": 0.6, "intake": 0.5, "x": 0.5}, "auto", {}, id="extra_channel"),
        pytest.param({"radiator": 0.1, "intake": 0.5}, "auto", {}, id="below_pwm_min"),
        pytest.param({"radiator": 0.6, "intake": 1.05}, "auto", {}, id="above_pwm_max"),
        pytest.param({"radiator": 0.61, "intake": 0.5}, "auto", {}, id="rate_limit"),
        pytest.param({"radiator": float("nan"), "intake": 0.5}, "auto", {}, id="nan_pwm"),
        pytest.param({"radiator": 0.5, "intake": 0.5}, "auto", {"a": float("inf")}, id="inf_diag"),
        pytest.param(
            {"radiator": 0.5, "intake": 0.5}, "auto", {"a": [1, {"b": float("nan")}]}, id="nested"
        ),
    ],
)
def test_assert_command_safe_rejects(cfg, pwm, mode, diag):
    prev = {"radiator": 0.5, "intake": 0.5}
    cmd = MpcCommand(pwm=pwm, mode=mode, diagnostics=diag)
    with pytest.raises(AssertionError):
        assert_command_safe(good_obs(), cfg, cmd, prev)


def test_assert_command_safe_tolerates_float_slack(cfg):
    prev = {"radiator": 0.5, "intake": 0.5}
    cmd = MpcCommand(pwm={"radiator": 0.5 + cfg.d_pwm_max + 5e-10, "intake": 0.5}, mode="auto")
    assert_command_safe(good_obs(), cfg, cmd, prev)


def test_structurally_untrusted_obs_requires_fallback(cfg):
    prev = {"radiator": 0.5, "intake": 0.5}
    cases = [
        PlantObservation(temps={"coolant": 35.0}, rpm={}, pwm={}, ts=0),  # missing air
        PlantObservation(temps={"coolant": 35.0, "air": None}, rpm={}, pwm={}, ts=0),
        PlantObservation(temps={"coolant": 35.0, "air": float("nan")}, rpm={}, pwm={}, ts=0),
        PlantObservation(temps={"coolant": 35.0, "air": 25.0, "x": 1.0}, rpm={}, pwm={}, ts=0),
        PlantObservation(temps={"coolant": 32767.0, "air": 25.0}, rpm={}, pwm={}, ts=0),
        PlantObservation(temps={}, rpm={}, pwm={}, ts=0),
    ]
    for obs in cases:
        assert obs_structurally_untrusted(obs, cfg)
        auto = MpcCommand(pwm=prev, mode=Mode.AUTO)
        with pytest.raises(AssertionError):
            assert_command_safe(obs, cfg, auto, prev)
        assert_command_safe(obs, cfg, MpcCommand(pwm=prev, mode=Mode.FALLBACK), prev)
    assert not obs_structurally_untrusted(good_obs(), cfg)


def test_assert_state_finite_rejects_nan_and_half_fault():
    with pytest.raises(AssertionError):
        assert_state_finite(MpcState(integrator={"radiator": float("nan")}))
    with pytest.raises(AssertionError):
        assert_state_finite(MpcState(last_raw_temps={"coolant": float("inf")}))
    with pytest.raises(AssertionError):
        assert_state_finite(MpcState(fault_since_ts=1.0))
    with pytest.raises(AssertionError):
        assert_state_finite(MpcState(fault_reason=FaultReason.SOLVER))


# ---------------------------------------------------------------------------
# Intents
# ---------------------------------------------------------------------------


def test_intent_construction_and_coercion():
    assert SetMode("mixed").mode is ControlMode.MIXED
    assert SetPreset("cool").name is Preset.COOL
    assert SetSetpoint("coolant", 35).celsius == 35.0
    assert SetPwm("radiator", 1).pwm == 1.0
    assert ClearOverride().channel is None
    assert ClearOverride("intake").channel == "intake"


@pytest.mark.parametrize(
    "make",
    [
        lambda: SetMode("turbo"),
        lambda: SetMode(1),
        lambda: SetPreset("loud"),
        lambda: SetSetpoint("", 35.0),
        lambda: SetSetpoint("coolant", "35"),
        lambda: SetSetpoint("coolant", float("nan")),
        lambda: SetSetpoint("coolant", True),
        lambda: SetPwm("radiator", 1.5),
        lambda: SetPwm("radiator", -0.1),
        lambda: SetPwm("radiator", "0.5"),
        lambda: SetPwm(None, 0.5),
        lambda: ClearOverride(""),
        lambda: ClearOverride(3),
    ],
)
def test_intent_rejects_invalid(make):
    with pytest.raises(IntentInvalid):
        make()


def test_intent_errors_are_distinct():
    assert issubclass(IntentInvalid, ValueError)
    assert not issubclass(IntentConflict, IntentInvalid)
    assert not issubclass(IntentInvalid, IntentConflict)


def test_parse_intent_routes_and_rejects():
    assert parse_intent("mode", {"mode": "auto"}) == SetMode(ControlMode.AUTO)
    assert parse_intent("setpoint", {"channel": "coolant", "celsius": 35}) == SetSetpoint(
        "coolant", 35.0
    )
    assert parse_intent("pwm", {"channel": "radiator", "pwm": 0.4}) == SetPwm("radiator", 0.4)
    assert parse_intent("preset", {"name": "quiet"}) == SetPreset(Preset.QUIET)
    assert parse_intent("auto", {}) == ClearOverride()
    assert parse_intent("auto", None) == ClearOverride()
    assert parse_intent("auto", {"channel": "radiator"}) == ClearOverride("radiator")
    for kind, body in [
        ("reboot", {}),
        ("pwm", []),
        ("pwm", "radiator"),
        ("pwm", {"channel": "radiator"}),
        ("pwm", {"channel": "radiator", "pwm": 0.4, "force": True}),
        ("mode", {}),
        ("setpoint", {"channel": "coolant", "celsius": [35]}),
    ]:
        with pytest.raises(IntentInvalid):
            parse_intent(kind, body)


def test_control_snapshot_payloads_and_protocol(cfg):
    obs = good_obs(10.0)
    cmd = MpcCommand(pwm={"radiator": 0.5, "intake": 0.5}, mode=Mode.FALLBACK)
    snap = ControlSnapshot(
        obs=obs,
        last_cmd=cmd,
        control_mode=ControlMode.AUTO,
        setpoints=dict(cfg.setpoints),
        overrides={},
        preset=Preset.NORMAL,
        channels=cfg.channels,
        temps=cfg.temps,
        pwm_min=cfg.pwm_min,
        pwm_max=cfg.pwm_max,
        solver_status=ControlSnapshot.solver_status_for(cmd),
        fault_reason=FaultReason.SENSOR_GATE,
        fault_since_ts=8.0,
        usb_present=True,
        mqtt_connected=None,
        uptime_s=123.0,
        version="0.1.0",
    )
    assert snap.solver_status is SolverStatus.FALLBACK
    assert ControlSnapshot.solver_status_for(None) is SolverStatus.FAULT
    assert (
        ControlSnapshot.solver_status_for(MpcCommand(pwm={}, mode=Mode.SATURATED))
        is SolverStatus.OK
    )
    d = json.loads(snap.to_json())
    assert d["obs"] == obs.to_dict()
    assert d["cmd"] == cmd.to_dict()
    assert d["mode"] == "auto"
    assert d["health"]["solver"] == "fallback"
    assert d["health"]["usb_present"] is True
    assert d["health"]["mqtt_connected"] is None
    assert snap.state_payload()["setpoints"] == {"coolant": 35.0}
    assert snap.health_payload()["uptime_s"] == 123.0

    class Stub:
        def __init__(self) -> None:
            self.seen = []

        def snapshot(self) -> ControlSnapshot:
            return snap

        def submit(self, intent) -> None:
            if isinstance(intent, SetPwm):
                raise IntentConflict("auto mode")
            self.seen.append(intent)

    stub = Stub()
    assert isinstance(stub, ControlSurface)
    stub.submit(SetMode("manual"))
    with pytest.raises(IntentConflict):
        stub.submit(SetPwm("radiator", 0.4))
    assert stub.seen == [SetMode(ControlMode.MANUAL)]
