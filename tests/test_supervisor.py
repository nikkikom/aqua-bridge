"""Supervisor: every section 6 control rule, section 4.8 "auto -> 409", compose."""

from __future__ import annotations

import contextlib
import dataclasses
import json
import threading

import pytest
from hypothesis import HealthCheck, given, settings
from hypothesis import strategies as st

from aqua_bridge.control.intents import (
    ClearOverride,
    ControlMode,
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
from aqua_bridge.control.mpc import step
from aqua_bridge.control.supervisor import PRESETS, Supervisor, TickPlan, apply_preset
from aqua_bridge.model import Mode, MpcCommand, MpcState
from invariants import assert_command_safe, make_obs


@pytest.fixture
def sup(cfg):
    return Supervisor(cfg, clock=lambda: 100.0, version="test")


def auto_cmd(cfg, pwm=0.5, mode=Mode.AUTO):
    return MpcCommand(pwm=dict.fromkeys(cfg.channels, pwm), mode=mode, diagnostics={"k": 1})


def plan_for(sup: Supervisor) -> TickPlan:
    return sup.plan_tick()


# --- construction / protocol --------------------------------------------------------


def test_implements_control_surface(sup):
    assert isinstance(sup, ControlSurface)


def test_rejects_non_config():
    with pytest.raises(TypeError):
        Supervisor({"dt": 1})  # type: ignore[arg-type]


def test_initial_snapshot(sup, cfg):
    s = sup.snapshot()
    assert s.control_mode is ControlMode.AUTO
    assert s.preset is Preset.NORMAL
    assert s.setpoints == cfg.setpoints
    assert s.overrides == {}
    assert s.channels == tuple(cfg.channels) and s.temps == tuple(cfg.temps)
    assert (s.pwm_min, s.pwm_max) == (cfg.pwm_min, cfg.pwm_max)
    assert s.solver_status is SolverStatus.FAULT  # no command yet
    assert s.obs is None and s.last_cmd is None
    assert s.usb_present is False and s.mqtt_connected is None
    assert s.version == "test"
    assert s.uptime_s == 0.0
    json.loads(s.to_json())  # serialisable, no NaN


# --- SetPwm -------------------------------------------------------------------------


def test_raw_pwm_in_auto_is_409(sup):
    with pytest.raises(IntentConflict):
        sup.submit(SetPwm("radiator", 0.4))
    assert sup.overrides == {}
    assert sup.control_mode is ControlMode.AUTO


def test_raw_pwm_unknown_channel_is_invalid_even_in_manual(sup):
    sup.submit(SetMode(ControlMode.MIXED))
    with pytest.raises(IntentInvalid):
        sup.submit(SetPwm("nosuch", 0.4))


def test_raw_pwm_outside_pwm_limits_is_invalid(sup, cfg):
    sup.submit(SetMode(ControlMode.MIXED))
    with pytest.raises(IntentInvalid):
        sup.submit(SetPwm("radiator", cfg.pwm_min - 0.01))
    with pytest.raises(IntentInvalid):
        sup.submit(SetPwm("radiator", 0.0))
    assert sup.overrides == {}


def test_range_check_precedes_mode_conflict(sup, cfg):
    # An out-of-range PWM is invalid (4xx) whatever the mode; only a legal one is a 409.
    with pytest.raises(IntentInvalid):
        sup.submit(SetPwm("radiator", cfg.pwm_min / 2))


def test_raw_pwm_at_limits_accepted_in_mixed(sup, cfg):
    sup.submit(SetMode(ControlMode.MIXED))
    sup.submit(SetPwm("radiator", cfg.pwm_min))
    sup.submit(SetPwm("intake", cfg.pwm_max))
    assert sup.overrides == {"radiator": cfg.pwm_min, "intake": cfg.pwm_max}
    assert sup.control_mode is ControlMode.MIXED
    assert sup.snapshot().overrides == sup.overrides


def test_raw_pwm_via_parse_intent(sup):
    sup.submit(parse_intent("mode", {"mode": "manual"}))
    sup.submit(parse_intent("pwm", {"channel": "radiator", "pwm": 0.6}))
    assert sup.overrides["radiator"] == 0.6


# --- SetMode ------------------------------------------------------------------------


def test_manual_seeds_overrides_from_fallback_when_nothing_applied(sup, cfg):
    sup.submit(SetMode(ControlMode.MANUAL))
    assert sup.overrides == dict(cfg.fallback_pwm)
    assert sup.control_mode is ControlMode.MANUAL


def test_manual_seeds_overrides_from_last_applied(sup, cfg):
    cmd = MpcCommand(pwm={"radiator": 0.33, "intake": 0.44}, mode=Mode.AUTO)
    sup.record_tick(obs=None, mpc_cmd=cmd, cmd=cmd, state=None, applied=True, usb_present=True)
    sup.submit(SetMode(ControlMode.MANUAL))
    assert sup.overrides == {"radiator": 0.33, "intake": 0.44}


def test_manual_keeps_existing_override(sup):
    sup.submit(SetMode(ControlMode.MIXED))
    sup.submit(SetPwm("radiator", 0.5))
    sup.submit(SetMode(ControlMode.MANUAL))
    assert sup.overrides["radiator"] == 0.5
    assert set(sup.overrides) == {"radiator", "intake"}


def test_auto_clears_overrides_and_releases_channels(sup):
    sup.submit(SetMode(ControlMode.MANUAL))
    sup.plan_tick()  # consume the empty release set
    sup.submit(SetMode(ControlMode.AUTO))
    assert sup.overrides == {}
    plan = sup.plan_tick()
    assert plan.released == {"radiator", "intake"}
    assert plan.control_mode is ControlMode.AUTO
    assert sup.plan_tick().released == frozenset()  # consumed once


def test_mixed_keeps_overrides(sup):
    sup.submit(SetMode(ControlMode.MANUAL))
    sup.submit(SetMode(ControlMode.MIXED))
    assert set(sup.overrides) == {"radiator", "intake"}
    assert sup.control_mode is ControlMode.MIXED


# --- ClearOverride ------------------------------------------------------------------


def test_clear_one_channel_in_manual_becomes_mixed(sup):
    sup.submit(SetMode(ControlMode.MANUAL))
    sup.plan_tick()
    sup.submit(ClearOverride("radiator"))
    assert sup.control_mode is ControlMode.MIXED
    assert set(sup.overrides) == {"intake"}
    assert sup.plan_tick().released == {"radiator"}


def test_clear_last_override_returns_to_auto(sup):
    sup.submit(SetMode(ControlMode.MIXED))
    sup.submit(SetPwm("radiator", 0.5))
    sup.submit(ClearOverride("radiator"))
    assert sup.control_mode is ControlMode.AUTO
    assert sup.overrides == {}


def test_clear_all_returns_to_auto(sup):
    sup.submit(SetMode(ControlMode.MANUAL))
    sup.submit(parse_intent("auto", {}))
    assert sup.control_mode is ControlMode.AUTO
    assert sup.overrides == {}


def test_clear_unknown_channel_invalid(sup):
    with pytest.raises(IntentInvalid):
        sup.submit(ClearOverride("nosuch"))


def test_clear_without_override_is_harmless(sup):
    sup.submit(SetMode(ControlMode.MIXED))
    sup.submit(ClearOverride("radiator"))
    assert sup.control_mode is ControlMode.AUTO  # nothing overridden -> auto
    assert sup.plan_tick().released == frozenset()


def test_reoverride_after_clear_cancels_release(sup):
    sup.submit(SetMode(ControlMode.MIXED))
    sup.submit(SetPwm("radiator", 0.5))
    sup.submit(ClearOverride("radiator"))
    sup.submit(SetMode(ControlMode.MIXED))
    sup.submit(SetPwm("radiator", 0.6))
    assert sup.plan_tick().released == frozenset()


# --- SetSetpoint --------------------------------------------------------------------


def test_setpoint_updates_effective_config(sup, cfg):
    sup.submit(SetSetpoint("coolant", 33.0))
    assert sup.setpoints["coolant"] == 33.0
    assert sup.effective_config().setpoints["coolant"] == 33.0
    assert sup.snapshot().setpoints["coolant"] == 33.0
    assert cfg.setpoints["coolant"] == 35.0  # base config untouched


def test_setpoint_unknown_temperature_invalid(sup):
    with pytest.raises(IntentInvalid):
        sup.submit(SetSetpoint("gpu", 40.0))


def test_setpoint_for_temperature_without_setpoint_invalid(sup, cfg):
    assert "air" in cfg.temps and "air" not in cfg.setpoints
    with pytest.raises(IntentInvalid):
        sup.submit(SetSetpoint("air", 30.0))


def test_setpoint_outside_gate_range_invalid_and_state_unchanged(sup, cfg):
    with pytest.raises(IntentInvalid):
        sup.submit(SetSetpoint("coolant", cfg.temp_max_c))
    with pytest.raises(IntentInvalid):
        sup.submit(SetSetpoint("coolant", cfg.temp_min_c - 5))
    assert sup.setpoints == cfg.setpoints
    assert sup.effective_config() == cfg


def test_setpoint_via_parse_intent(sup):
    sup.submit(parse_intent("setpoint", {"channel": "coolant", "celsius": 36}))
    assert sup.setpoints["coolant"] == 36.0


# --- SetPreset ----------------------------------------------------------------------


@pytest.mark.parametrize("preset", list(Preset))
def test_preset_effect_matches_table(sup, cfg, preset):
    sup.submit(SetPreset(preset))
    eff = sup.effective_config()
    effect = PRESETS[preset]
    assert sup.preset is preset
    assert eff.setpoints["coolant"] == pytest.approx(
        cfg.setpoints["coolant"] + effect.setpoint_offset_c
    )
    assert eff.pi_kp == pytest.approx(cfg.pi_kp * effect.gain_scale)
    assert eff.pi_ki == pytest.approx(cfg.pi_ki * effect.gain_scale)
    assert eff.weight_dpwm == pytest.approx(cfg.weight_dpwm * effect.move_penalty_scale)
    # Structure and limits never change.
    assert eff.channels == cfg.channels and eff.temps == cfg.temps
    assert (eff.pwm_min, eff.pwm_max, eff.d_pwm_max) == (cfg.pwm_min, cfg.pwm_max, cfg.d_pwm_max)
    assert eff.fallback_pwm == cfg.fallback_pwm


def test_preset_offset_stacks_on_user_setpoint_and_snapshot_reports_both(sup):
    sup.submit(SetSetpoint("coolant", 30.0))
    sup.submit(SetPreset(Preset.COOL))
    s = sup.snapshot()
    assert s.setpoints["coolant"] == 30.0
    assert s.extra["effective_setpoints"]["coolant"] == pytest.approx(28.0)


def test_normal_preset_restores_base_config(sup, cfg):
    sup.submit(SetPreset(Preset.QUIET))
    sup.submit(SetPreset(Preset.NORMAL))
    assert sup.effective_config() == cfg


def test_preset_that_invalidates_config_is_rejected(sup, cfg):
    sup.submit(SetSetpoint("coolant", cfg.temp_max_c - 1.0))  # valid on its own
    with pytest.raises(IntentInvalid):
        sup.submit(SetPreset(Preset.QUIET))  # +2 C would leave the gate range
    assert sup.preset is Preset.NORMAL
    assert sup.effective_config().setpoints["coolant"] == cfg.temp_max_c - 1.0


def test_preset_via_parse_intent(sup):
    sup.submit(parse_intent("preset", {"name": "cool"}))
    assert sup.preset is Preset.COOL


def test_apply_preset_is_pure(cfg):
    out = apply_preset(cfg, {"coolant": 40.0}, Preset.QUIET)
    assert out.setpoints == {"coolant": 42.0}
    assert cfg.setpoints == {"coolant": 35.0}


def test_effective_config_steps(sup, cfg):
    sup.submit(SetPreset(Preset.COOL))
    cmd, _ = step(make_obs(cfg, 0.0), sup.effective_config(), MpcState.cold())
    assert set(cmd.pwm) == set(cfg.channels)


def test_unsupported_intent_invalid(sup):
    with pytest.raises(IntentInvalid):
        sup.submit(object())  # type: ignore[arg-type]


# --- compose ------------------------------------------------------------------------


def test_compose_without_overrides_is_solver_command(sup, cfg):
    mpc = auto_cmd(cfg, 0.5)
    out = sup.compose(mpc, plan_for(sup), dict.fromkeys(cfg.channels, 0.5))
    assert out.pwm == mpc.pwm and out.mode is mpc.mode
    assert out.diagnostics["k"] == 1
    assert out.diagnostics["supervisor"]["overrides_applied"] is False


def test_compose_override_rate_limited_toward_target(sup, cfg):
    sup.submit(SetMode(ControlMode.MIXED))
    sup.submit(SetPwm("radiator", 0.9))
    prev = {"radiator": 0.5, "intake": 0.5}
    mpc = auto_cmd(cfg, 0.5)
    out = sup.compose(mpc, plan_for(sup), prev)
    assert out.pwm["radiator"] == pytest.approx(0.5 + cfg.d_pwm_max)
    assert out.pwm["intake"] == 0.5  # solver keeps the non-overridden channel
    assert out.diagnostics["supervisor"]["override_rate_limited"] == {"radiator": True}
    assert_command_safe(make_obs(cfg, 0.0), cfg, out, prev)


def test_compose_override_reaches_target_when_within_step(sup, cfg):
    sup.submit(SetMode(ControlMode.MIXED))
    sup.submit(SetPwm("intake", 0.55))
    prev = {"radiator": 0.5, "intake": 0.5}
    out = sup.compose(auto_cmd(cfg, 0.5), plan_for(sup), prev)
    assert out.pwm["intake"] == 0.55


def test_compose_override_downward_is_rate_limited_too(sup, cfg):
    sup.submit(SetMode(ControlMode.MANUAL))
    sup.submit(SetPwm("radiator", cfg.pwm_min))
    prev = {"radiator": 0.8, "intake": 0.8}
    out = sup.compose(auto_cmd(cfg, 0.8), plan_for(sup), prev)
    assert out.pwm["radiator"] == pytest.approx(0.8 - cfg.d_pwm_max)


def test_fallback_wins_over_overrides(sup, cfg):
    sup.submit(SetMode(ControlMode.MANUAL))
    sup.submit(SetPwm("radiator", cfg.pwm_min))
    sup.submit(SetPwm("intake", cfg.pwm_min))
    fb = auto_cmd(cfg, 0.8, mode=Mode.FALLBACK)
    out = sup.compose(fb, plan_for(sup), dict.fromkeys(cfg.channels, 0.8))
    assert out.pwm == fb.pwm
    assert out.mode is Mode.FALLBACK
    assert out.diagnostics["supervisor"]["overrides_applied"] is False
    assert out.diagnostics["supervisor"]["overrides"] == {"radiator": 0.15, "intake": 0.15}
    # Overrides survive the fault and resume afterwards.
    out2 = sup.compose(auto_cmd(cfg, 0.8), plan_for(sup), dict.fromkeys(cfg.channels, 0.8))
    assert out2.pwm["radiator"] == pytest.approx(0.8 - cfg.d_pwm_max)


def test_compose_keeps_saturated_mode_and_does_not_mutate_input(sup, cfg):
    sup.submit(SetMode(ControlMode.MIXED))
    sup.submit(SetPwm("radiator", 0.5))
    mpc = auto_cmd(cfg, 1.0, mode=Mode.SATURATED)
    before = mpc.to_dict()
    out = sup.compose(mpc, plan_for(sup), dict.fromkeys(cfg.channels, 1.0))
    assert out.mode is Mode.SATURATED
    assert mpc.to_dict() == before


def test_compose_is_deterministic(sup, cfg):
    sup.submit(SetMode(ControlMode.MIXED))
    sup.submit(SetPwm("radiator", 0.7))
    plan = plan_for(sup)
    prev = dict.fromkeys(cfg.channels, 0.5)
    a = sup.compose(auto_cmd(cfg, 0.5), plan, prev)
    b = sup.compose(auto_cmd(cfg, 0.5), plan, prev)
    assert a.to_dict() == b.to_dict()


@pytest.mark.fuzzy
@settings(max_examples=100, suppress_health_check=[HealthCheck.function_scoped_fixture])
@given(
    prev=st.fixed_dictionaries({"radiator": st.floats(0.15, 1.0), "intake": st.floats(0.15, 1.0)}),
    over=st.dictionaries(st.sampled_from(["radiator", "intake"]), st.floats(0.15, 1.0), max_size=2),
    solver=st.floats(0.15, 1.0),
)
def test_compose_respects_bounds_and_rate_for_any_override(cfg, prev, over, solver):
    sup = Supervisor(cfg)
    sup.submit(SetMode(ControlMode.MIXED))
    for ch, v in over.items():
        sup.submit(SetPwm(ch, v))
    mpc = MpcCommand(pwm=dict.fromkeys(cfg.channels, solver), mode=Mode.AUTO)
    # The solver's own command is rate limited against prev already; emulate that.
    lim = {
        ch: min(max(solver, prev[ch] - cfg.d_pwm_max), prev[ch] + cfg.d_pwm_max)
        for ch in cfg.channels
    }
    mpc = dataclasses.replace(mpc, pwm=lim)
    out = sup.compose(mpc, sup.plan_tick(), prev)
    assert_command_safe(make_obs(cfg, 0.0), cfg, out, prev)
    for ch in cfg.channels:
        if ch in over:
            # Moves toward the override, never past it.
            assert abs(out.pwm[ch] - over[ch]) <= abs(prev[ch] - over[ch]) + 1e-9
        else:
            assert out.pwm[ch] == lim[ch]


# --- record_tick / snapshot ---------------------------------------------------------


def test_record_tick_feeds_snapshot(sup, cfg):
    obs = make_obs(cfg, 5.0)
    mpc = auto_cmd(cfg, 0.5, mode=Mode.FALLBACK)
    cmd = auto_cmd(cfg, 0.5, mode=Mode.FALLBACK)
    state = dataclasses.replace(MpcState.cold(), fault_since_ts=5.0, fault_reason="sensor_gate")
    sup.record_tick(
        obs=obs,
        mpc_cmd=mpc,
        cmd=cmd,
        state=state,
        applied=False,
        usb_present=False,
        extra={"apply_error": "boom"},
    )
    s = sup.snapshot()
    assert s.obs == obs and s.last_cmd == cmd
    assert s.solver_status is SolverStatus.FALLBACK
    assert s.fault_since_ts == 5.0 and s.fault_reason.value == "sensor_gate"
    assert s.usb_present is False
    assert s.extra["apply_error"] == "boom"
    assert s.extra["applied_pwm"] is None  # not applied -> not on the fans
    assert s.health_payload()["solver"] == "fallback"
    json.loads(s.to_json())


def test_solver_status_follows_solver_command_not_composed_one(sup, cfg):
    mpc = auto_cmd(cfg, 0.5, mode=Mode.SATURATED)
    cmd = auto_cmd(cfg, 0.5)
    sup.record_tick(
        obs=None, mpc_cmd=mpc, cmd=cmd, state=MpcState.cold(), applied=True, usb_present=True
    )
    s = sup.snapshot()
    assert s.solver_status is SolverStatus.OK
    assert s.extra["mpc_mode"] == "saturated"
    assert s.extra["applied_pwm"] == cmd.pwm
    assert s.usb_present is True


def test_mqtt_and_usb_flags(sup):
    sup.set_mqtt_connected(True)
    sup.set_usb_present(True)
    s = sup.snapshot()
    assert s.mqtt_connected is True and s.usb_present is True


def test_device_health_is_a_view_only_snapshot_field(sup):
    """Items 79 and 83: the health monitor publishes here; nothing in the control path
    reads it back, and both payloads carry it."""
    assert sup.snapshot().device_health == {}
    assert sup.snapshot().health_payload()["device_health"] == {"ok": True, "problems": []}

    health = {"devices": [{"label": "aquaero"}], "fans": {}, "problems": ["boom"], "ok": False}
    sup.set_device_health(health)
    s = sup.snapshot()
    assert s.device_health == health
    assert s.state_payload()["device_health"] == health
    assert s.health_payload()["device_health"] == {"ok": False, "problems": ["boom"]}
    json.loads(s.to_json())

    # the snapshot hands out its own top-level dict, so a reader cannot add keys to it
    s.device_health["extra"] = 1
    assert "extra" not in sup.snapshot().device_health

    sup.set_device_health(None)
    assert sup.snapshot().device_health == {}


def test_uptime_uses_injected_clock(cfg):
    now = [10.0]
    sup = Supervisor(cfg, clock=lambda: now[0])
    now[0] = 25.5
    assert sup.snapshot().uptime_s == pytest.approx(15.5)


# --- thread safety ----------------------------------------------------------------


@pytest.mark.slow
def test_concurrent_http_and_loop_threads(cfg):
    sup = Supervisor(cfg)
    errors: list[BaseException] = []
    stop = threading.Event()

    def http_side() -> None:
        i = 0
        try:
            while not stop.is_set():
                sup.submit(SetMode(ControlMode.MIXED))
                with contextlib.suppress(IntentConflict):
                    sup.submit(SetPwm("radiator", 0.15 + (i % 80) / 100))
                sup.submit(SetPreset(list(Preset)[i % 3]))
                sup.snapshot().to_json()
                if i % 7 == 0:
                    sup.submit(ClearOverride())
                i += 1
        except BaseException as exc:  # noqa: BLE001 - collected for the assertion below
            errors.append(exc)

    def loop_side() -> None:
        prev = dict.fromkeys(cfg.channels, 0.5)
        try:
            for _ in range(2000):
                plan = sup.plan_tick()
                mpc = MpcCommand(pwm=dict(prev), mode=Mode.AUTO)
                out = sup.compose(mpc, plan, prev)
                assert_command_safe(make_obs(cfg, 0.0), plan.cfg, out, prev)
                prev = dict(out.pwm)
                sup.record_tick(
                    obs=None, mpc_cmd=mpc, cmd=out, state=None, applied=True, usb_present=True
                )
        except BaseException as exc:  # noqa: BLE001
            errors.append(exc)

    t1 = threading.Thread(target=http_side)
    t2 = threading.Thread(target=loop_side)
    t1.start()
    t2.start()
    t2.join(30)
    stop.set()
    t1.join(30)
    assert not errors, errors


# --- identification experiments (DAS plan section 5) -----------------------


def test_compose_applies_experiment_levels_like_an_override_and_fallback_beats_them():
    from aqua_bridge.control.intents import Ident
    from test_ident_experiment import Rig, ident_cfg

    rig = Rig(ident_cfg())
    rig.ticks(8)
    rig.sup.submit(Ident("start", channel="fb1"))
    plan = rig.sup.plan_tick()
    assert plan.control_mode is ControlMode.AUTO
    assert set(plan.overrides) == {"fb1"} and plan.experiment["running"] is True
    cfg = plan.cfg
    prev = dict.fromkeys(cfg.channels, 0.2)
    solver = MpcCommand(pwm=dict.fromkeys(cfg.channels, 0.2), mode=Mode.AUTO, diagnostics={})
    out = rig.sup.compose(solver, plan, prev)
    # the experiment wants base + A = 0.65; one step of d_pwm_max from 0.2
    assert out.pwm["fb1"] == pytest.approx(0.2 + cfg.d_pwm_max)
    assert out.diagnostics["supervisor"]["override_rate_limited"]["fb1"] is True
    assert out.diagnostics["supervisor"]["experiment"]["target"]["name"] == "fb1"
    for mode, diag in (
        (Mode.FALLBACK, {}),
        (Mode.DEGRADED, {"fallback_channels": ["fb1"]}),
    ):
        blind = MpcCommand(pwm=dict.fromkeys(cfg.channels, 0.8), mode=mode, diagnostics=diag)
        out = rig.sup.compose(blind, plan, prev)
        assert out.pwm == blind.pwm
    # the human view: auto, no override
    snap = rig.sup.snapshot()
    assert snap.control_mode is ControlMode.AUTO and snap.overrides == {}
    assert snap.extra["experiment"]["overrides"] == {"fb1": pytest.approx(0.65)}
