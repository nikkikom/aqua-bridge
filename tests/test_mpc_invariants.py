"""Section 4.1 invariants on ``mpc.step`` (PROJECT.md).

Every call goes through :func:`invariants.checked_step`, which asserts the
single-step bullets. This file adds the multi-step ones: the first-tick
``prev`` rules (section 3 items 3-4), determinism, no NaN anywhere,
fallback-never-toward-``pwm_min``, honest saturation, and invariants along
a long nominal closed loop and along random observation sequences.
"""

from __future__ import annotations

import dataclasses
import json
import math
from pathlib import Path

import pytest
from hypothesis import given, settings
from hypothesis import strategies as st

from aqua_bridge.config import load_config
from aqua_bridge.control import mpc
from aqua_bridge.control.mpc import step
from aqua_bridge.model import FaultReason, Mode, MpcConfig, MpcState, PlantObservation, SolverKind
from aqua_bridge.sim.plant import Plant, PlantParams, run_closed_loop
from invariants import (
    TOL,
    assert_command_safe,
    assert_no_non_finite,
    checked_step,
    make_obs,
    obs_pwm_trusted,
    resolve_prev_pwm,
)

SP = 35.0


@pytest.fixture
def cfg(cfg: MpcConfig, solver_kind) -> MpcConfig:
    """Section 8: every scenario in this module runs for the PI and the MPC solver."""
    return dataclasses.replace(cfg, solver=solver_kind)


# Hypothesis forbids function-scoped fixtures inside @given; load the example config once
# and parametrise the solver directly (section 8: the same tests for PI and MPC).
EXAMPLE_CFG = load_config(Path(__file__).resolve().parent.parent / "config.example.yaml").mpc
SOLVER_CFGS = [dataclasses.replace(EXAMPLE_CFG, solver=k) for k in (SolverKind.PI, SolverKind.MPC)]
SOLVER_IDS = [c.solver.value for c in SOLVER_CFGS]


def nominal_plant(cfg, **overrides) -> Plant:
    params = PlantParams(dt=cfg.dt, heat_w=overrides.pop("heat_w", 100.0), **overrides)
    return Plant(params, initial_pwm=0.5, t_coolant=40.0, t_air=30.0)


# ---------------------------------------------------------------------------
# first tick: prev rules (section 3 items 3-4, section 4.3 last bullet)
# ---------------------------------------------------------------------------


def test_cold_untrusted_first_tick_commands_fallback_pwm_exactly(cfg):
    obs = make_obs(cfg, 0.0, coolant=None, pwm={}, rpm={})
    assert resolve_prev_pwm(MpcState.cold(), obs, cfg) == cfg.fallback_pwm
    cmd, state = checked_step(obs, cfg, MpcState.cold())
    assert cmd.pwm == cfg.fallback_pwm
    assert cmd.mode is Mode.FALLBACK
    assert cmd.diagnostics["prev_source"] == "fallback_pwm"
    assert state.fault_reason is FaultReason.SENSOR_GATE
    assert state.fault_since_ts == 0.0
    assert state.trusted_streak == 0


def test_cold_untrusted_temps_but_usable_obs_pwm_holds_obs_pwm(cfg):
    obs = make_obs(cfg, 0.0, coolant=None, pwm=dict.fromkeys(cfg.channels, 0.4))
    cmd, _ = checked_step(obs, cfg, MpcState.cold())
    assert cmd.mode is Mode.FALLBACK
    assert cmd.pwm == pytest.approx(dict.fromkeys(cfg.channels, 0.4))  # hold, delta = 0


def test_cold_trusted_first_tick_prev_is_obs_pwm_and_bumpless(cfg):
    obs = make_obs(cfg, 0.0, coolant=SP + 5.0, pwm=dict.fromkeys(cfg.channels, 0.3))
    cmd, state = checked_step(obs, cfg, MpcState.cold())
    assert cmd.mode is Mode.AUTO
    assert cmd.diagnostics["prev_source"] == "obs_pwm"
    # bumpless start: the first output equals prev before the rate limit
    assert cmd.diagnostics["target_pwm"] == pytest.approx(dict.fromkeys(cfg.channels, 0.3))
    assert cmd.pwm == pytest.approx(dict.fromkeys(cfg.channels, 0.3))
    assert state.last_good_obs == obs
    assert state.trusted_streak == 1
    assert not state.in_fault


def test_cold_trusted_temps_with_unusable_obs_pwm_starts_from_fallback(cfg):
    obs = make_obs(cfg, 0.0, pwm={"radiator": 1.5, "intake": math.nan})
    cmd, _ = checked_step(obs, cfg, MpcState.cold())
    assert cmd.mode is Mode.AUTO
    assert cmd.pwm == pytest.approx(cfg.fallback_pwm)
    assert cmd.diagnostics["prev_source"] == "fallback_pwm"
    assert cmd.diagnostics["obs_pwm_usable"] is False


def test_cold_start_obs_pwm_zero_below_reach_uses_fallback(cfg):
    """pwm_min - d_pwm_max > 0 in the example config: 0.0 cannot reach the range in one step."""
    assert cfg.pwm_min - cfg.d_pwm_max > 0
    obs = make_obs(cfg, 0.0, coolant=50.0, pwm=dict.fromkeys(cfg.channels, 0.0))
    assert not obs_pwm_trusted(obs, cfg)
    cmd, _ = checked_step(obs, cfg, MpcState.cold())
    assert cmd.pwm == pytest.approx(cfg.fallback_pwm)


def test_cold_start_obs_pwm_zero_within_reach_ramps_without_skipping(cfg):
    low = dataclasses.replace(cfg, pwm_min=0.1, fallback_pwm=dict.fromkeys(cfg.channels, 0.8))
    obs = make_obs(low, 0.0, coolant=50.0, pwm=dict.fromkeys(low.channels, 0.0))
    assert obs_pwm_trusted(obs, low)
    cmd, state = checked_step(obs, low, MpcState.cold())
    assert all(low.pwm_min <= v <= low.d_pwm_max + TOL for v in cmd.pwm.values())
    # and the following ticks keep ramping up by at most d_pwm_max
    prev = cmd
    for i in range(1, 8):
        cmd, state = checked_step(make_obs(low, i * low.dt, coolant=50.0), low, state)
        for ch in low.channels:
            assert cmd.pwm[ch] >= prev.pwm[ch] - TOL
        prev = cmd


def test_step_rejects_wrong_types_only(cfg):
    with pytest.raises(TypeError):
        step({"temps": {}}, cfg, MpcState.cold())  # type: ignore[arg-type]
    with pytest.raises(TypeError):
        step(make_obs(cfg, 0.0), cfg, {})  # type: ignore[arg-type]
    with pytest.raises(TypeError):
        step(make_obs(cfg, 0.0), cfg.to_dict(), MpcState.cold())  # type: ignore[arg-type]


# ---------------------------------------------------------------------------
# no step toward pwm_min because of a fault; mode <-> fault agreement
# ---------------------------------------------------------------------------


def test_untrusted_tick_never_lowers_pwm(fast_cfg):
    cfg = fast_cfg
    state = MpcState.cold()
    # warm up at a high command
    for i in range(3):
        _, state = checked_step(
            make_obs(cfg, float(i), coolant=SP, pwm=dict.fromkeys(cfg.channels, 0.7)), cfg, state
        )
    last = state.last_cmd
    assert last is not None
    for i in range(3, 20):
        cmd, state = checked_step(make_obs(cfg, float(i), coolant=None), cfg, state)
        assert cmd.mode is Mode.FALLBACK
        for ch in cfg.channels:
            assert cmd.pwm[ch] >= last.pwm[ch] - TOL
        last = cmd


def test_structurally_untrusted_variants_all_fallback(cfg):
    state = MpcState.cold()
    bad = [
        make_obs(cfg, 0.0, temps={}),
        make_obs(cfg, 2.0, temps={"coolant": SP}),
        make_obs(cfg, 4.0, temps={"coolant": SP, "air": 30.0, "ghost": 1.0}),
        make_obs(cfg, 6.0, coolant=math.nan),
        make_obs(cfg, 8.0, coolant=-25.0),
        make_obs(cfg, 10.0, air=121.0),
        make_obs(cfg, 12.0, coolant=None),
    ]
    for obs in bad:
        cmd, state = checked_step(obs, cfg, state)
        assert cmd.mode is Mode.FALLBACK
        assert state.fault_reason is FaultReason.SENSOR_GATE
    assert state.fault_since_ts == 0.0  # one timer, never restarted


# ---------------------------------------------------------------------------
# determinism and serialisation
# ---------------------------------------------------------------------------


def warm_state(cfg, ticks: int = 5) -> MpcState:
    state = MpcState.cold()
    for i in range(ticks):
        _, state = checked_step(make_obs(cfg, i * cfg.dt, coolant=SP + 1.0), cfg, state)
    return state


def test_same_inputs_same_outputs(cfg):
    state = warm_state(cfg)
    obs = make_obs(cfg, 100.0, coolant=SP + 2.0)
    c1, s1 = step(obs, cfg, state)
    c2, s2 = step(obs, cfg, state)
    assert c1.to_dict() == c2.to_dict()
    assert s1.to_dict() == s2.to_dict()
    assert c1 == c2 and s1 == s2


def test_state_survives_json_round_trip_and_steps_identically(cfg):
    state = warm_state(cfg)
    text = json.dumps(state.to_dict(), allow_nan=False)
    restored = MpcState.from_dict(json.loads(text))
    assert restored == state
    obs = make_obs(cfg, 100.0, coolant=SP - 3.0)
    c1, s1 = step(obs, cfg, state)
    c2, s2 = step(obs, cfg, restored)
    assert c1.to_dict() == c2.to_dict()
    assert s1.to_dict() == s2.to_dict()


def test_command_and_state_are_json_serialisable_without_nan(cfg):
    state = warm_state(cfg)
    for obs in (
        make_obs(cfg, 100.0, coolant=SP),
        make_obs(cfg, 102.0, coolant=math.nan),
        make_obs(cfg, 104.0, temps={}),
    ):
        cmd, state = checked_step(obs, cfg, state)
        json.dumps(cmd.to_dict(), allow_nan=False)
        json.dumps(state.to_dict(), allow_nan=False)
        assert_no_non_finite(cmd.diagnostics, "diagnostics")


def test_diagnostics_have_the_documented_keys(cfg):
    cmd, _ = checked_step(make_obs(cfg, 0.0), cfg, MpcState.cold())
    for key in (
        "trusted",
        "gate",
        "time",
        "prev_source",
        "prev_pwm",
        "target_pwm",
        "rate_limited",
        "saturated",
        "policy",
        "fault_reason",
        "fault_since_ts",
        "fault_elapsed_s",
        "trusted_streak",
        "confirm_ticks",
        "returning_to_auto",
        "solver",
        "solver_ran",
        "solver_error",
        "solver_diag",
        "fan_stall",
    ):
        assert key in cmd.diagnostics, key
    assert cmd.diagnostics["solver"] == cfg.solver.value
    assert cmd.diagnostics["time"]["status"] == "first"


# ---------------------------------------------------------------------------
# saturation is honest
# ---------------------------------------------------------------------------


def test_saturation_pins_at_pwm_max_and_reports_it(cfg):
    state = MpcState.cold()
    modes = []
    for i in range(12):
        obs = make_obs(cfg, i * cfg.dt, coolant=SP + 25.0, pwm=dict.fromkeys(cfg.channels, 0.5))
        cmd, state = checked_step(obs, cfg, state)
        modes.append(cmd.mode)
        if cmd.mode is Mode.SATURATED:
            assert all(v == pytest.approx(cfg.pwm_max) for v in cmd.pwm.values())
            assert all(cmd.diagnostics["target_pwm"][ch] > cfg.pwm_max for ch in cfg.channels)
        else:
            assert cmd.mode is Mode.AUTO  # still ramping toward the rail
    assert modes[-1] is Mode.SATURATED
    assert modes[0] is Mode.AUTO  # first tick is bumpless (output == prev), not yet pinned
    # integrator is capped: no windup while pinned
    assert all(v <= cfg.pwm_max + TOL for v in state.integrator.values())


def test_at_setpoint_output_is_steady(cfg):
    state = MpcState.cold()
    cmds = []
    for i in range(30):
        cmd, state = checked_step(make_obs(cfg, i * cfg.dt, coolant=SP), cfg, state)
        cmds.append(cmd)
    first = cmds[0].pwm
    for cmd in cmds:
        assert cmd.mode is Mode.AUTO
        assert cmd.pwm == pytest.approx(first, abs=1e-9)


# ---------------------------------------------------------------------------
# closed loop: invariants on every tick, PI regulates
# ---------------------------------------------------------------------------


def test_closed_loop_nominal_holds_invariants_every_tick(cfg):
    plant = nominal_plant(cfg)
    recs = run_closed_loop(plant, cfg, checked_step, ticks=900)
    modes = {r.cmd.mode for r in recs}
    assert Mode.FALLBACK not in modes
    err = recs[-1].obs.temps["coolant"] - SP
    assert abs(err) < 1.0, f"PI did not settle near the setpoint: error {err:+.2f} C"
    # no PWM chatter at equilibrium
    tail = recs[-50:]
    for a, b in zip(tail, tail[1:], strict=False):
        for ch in cfg.channels:
            assert abs(a.cmd.pwm[ch] - b.cmd.pwm[ch]) < 0.02


def test_closed_loop_with_sensor_noise_keeps_mode_auto(cfg):
    plant = nominal_plant(cfg, noise_sigma_c=0.2)
    recs = run_closed_loop(plant, cfg, checked_step, ticks=600)
    untrusted = sum(1 for r in recs if not r.cmd.diagnostics["trusted"])
    assert untrusted / len(recs) < 0.01
    assert all(r.cmd.mode is not Mode.FALLBACK for r in recs)


# ---------------------------------------------------------------------------
# random sequences (small, deterministic Hypothesis run; the full fuzz suite is 4.5)
# ---------------------------------------------------------------------------


def _lie(draw, obs: PlantObservation) -> PlantObservation:
    kind = draw(st.sampled_from(["none", "nan", "spike", "drop_key", "extra", "garbage", "swap"]))
    temps = dict(obs.temps)
    if kind == "none":
        temps["coolant"] = None
    elif kind == "nan":
        temps["air"] = math.nan
    elif kind == "spike":
        temps["coolant"] = draw(st.sampled_from([-40.0, 125.0, 32767.0]))
    elif kind == "drop_key":
        temps.pop("coolant")
    elif kind == "extra":
        temps["gpu"] = 40.0
    elif kind == "garbage":
        temps["coolant"] = 65535.0
    elif kind == "swap":
        temps["coolant"], temps["air"] = temps["air"], temps["coolant"]
    return dataclasses.replace(obs, temps=temps)


@pytest.mark.fuzzy
@pytest.mark.parametrize("cfg", SOLVER_CFGS, ids=SOLVER_IDS)
@settings(max_examples=30, deadline=None)
@given(data=st.data())
def test_random_sequences_with_lies_hold_invariants(cfg, data):
    n = data.draw(st.integers(min_value=10, max_value=60))
    p_lie = data.draw(st.floats(min_value=0.0, max_value=0.5))
    coolant = data.draw(st.floats(min_value=20.0, max_value=60.0))
    air = data.draw(st.floats(min_value=15.0, max_value=45.0))
    pwm0 = data.draw(st.floats(min_value=0.0, max_value=1.0))
    state = MpcState.cold()
    for i in range(n):
        coolant += data.draw(st.floats(min_value=-1.0, max_value=1.0))
        air += data.draw(st.floats(min_value=-1.0, max_value=1.0))
        pwm = dict.fromkeys(cfg.channels, pwm0) if i == 0 else state.last_cmd.pwm
        obs = make_obs(cfg, i * cfg.dt, coolant=coolant, air=air, pwm=pwm)
        if data.draw(st.floats(min_value=0.0, max_value=1.0)) < p_lie:
            obs = _lie(data.draw, obs)
        cmd, state = checked_step(obs, cfg, state)
        if not cmd.diagnostics["trusted"]:
            assert cmd.mode is Mode.FALLBACK


def test_prev_helper_and_step_agree_on_obs_pwm_band(cfg):
    for v in (0.0, 0.04, 0.05, 0.06, 0.5, 1.0):
        obs = make_obs(cfg, 0.0, pwm=dict.fromkeys(cfg.channels, v))
        assert obs_pwm_trusted(obs, cfg) == mpc.obs_pwm_usable(obs, cfg)
        cmd, _ = step(obs, cfg, MpcState.cold())
        assert_command_safe(obs, cfg, cmd, resolve_prev_pwm(MpcState.cold(), obs, cfg))
