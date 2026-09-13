"""Section 4.6 closed-loop simulation with ``aqua_bridge.sim.plant``.

Model mismatch, actuator delay, fan stall, a long run without drift to
NaN, and bumpless return after a long fallback. Written from PROJECT.md;
every tick goes through :func:`invariants.checked_step`.
"""

from __future__ import annotations

import dataclasses
import json
import math

import pytest

from aqua_bridge.model import Mode, MpcConfig, MpcState
from aqua_bridge.sim.plant import Plant, PlantParams, TickRecord, run_closed_loop
from invariants import TOL, assert_no_non_finite, checked_step

# Legacy-shaped scenarios (coolant setpoint): the DAS cases run in test_das_core.py.
pytestmark = pytest.mark.solver_cases("pi", "mpc")

SP = 35.0


@pytest.fixture
def cfg(cfg: MpcConfig, solver_kind) -> MpcConfig:
    """Section 8: every scenario in this module runs for the PI and the MPC solver."""
    return dataclasses.replace(cfg, solver=solver_kind)


COOLANT = "coolant"
AIR = "air"


def loop(plant: Plant, cfg: MpcConfig, ticks: int, **kw) -> list[TickRecord]:
    return run_closed_loop(plant, cfg, checked_step, ticks, **kw)


def assert_bounded_and_finite(recs: list[TickRecord], cfg: MpcConfig) -> None:
    for r in recs:
        for ch in cfg.channels:
            assert cfg.pwm_min - TOL <= r.cmd.pwm[ch] <= cfg.pwm_max + TOL
        for v in r.obs.temps.values():
            assert v is not None and math.isfinite(v), "plant produced a non-finite temperature"
        assert_no_non_finite(r.state.to_dict(), "state")
        assert_no_non_finite(r.cmd.to_dict(), "cmd")


# ---------------------------------------------------------------------------
# controller model != plant
# ---------------------------------------------------------------------------


MISMATCHES = {
    "fast_small_loop": dict(c_coolant_j_per_k=1200.0, c_air_j_per_k=200.0),
    "slow_big_loop": dict(c_coolant_j_per_k=16000.0, c_air_j_per_k=2500.0),
    "weak_fans": dict(radiator_fans={"radiator": 8.0}, intake_fans={"intake": 6.0}, heat_w=60.0),
    "strong_fans": dict(
        radiator_fans={"radiator": 70.0}, intake_fans={"intake": 50.0}, heat_w=130.0
    ),
    "high_gain_fast": dict(
        c_coolant_j_per_k=1500.0, radiator_fans={"radiator": 60.0}, intake_fans={"intake": 40.0}
    ),
}


@pytest.mark.parametrize("name", sorted(MISMATCHES))
def test_model_mismatch_keeps_pwm_bounded_and_never_crashes(cfg, name):
    params = dict(MISMATCHES[name])
    heat = params.pop("heat_w", 100.0)
    plant = Plant(PlantParams(dt=cfg.dt, heat_w=heat, **params), initial_pwm=0.5)
    recs = loop(plant, cfg, 900)
    assert_bounded_and_finite(recs, cfg)
    # the plant is real (no lies): a gain/time-constant mismatch must not look like a fault
    fb = [i for i, r in enumerate(recs) if r.cmd.mode is Mode.FALLBACK]
    assert not fb, f"{name}: mismatch produced fallback ticks {fb[:5]}"
    # bounded behaviour: either regulating within a few degrees or honestly saturated
    tail = recs[-100:]
    err = abs(sum(r.obs.temps[COOLANT] for r in tail) / len(tail) - SP)
    saturated = all(r.cmd.mode is Mode.SATURATED for r in tail)
    reachable = plant.equilibrium(dict.fromkeys(plant.channels, cfg.pwm_max))[0] < SP
    if reachable:
        assert err < 3.0, f"{name}: mean tail error {err:.2f} C"
    else:
        assert saturated or err < 3.0, f"{name}: unreachable setpoint but not saturated"


# ---------------------------------------------------------------------------
# actuator delay
# ---------------------------------------------------------------------------


@pytest.mark.parametrize("delay", [1, 2])
def test_actuator_delay_is_tolerated(cfg, delay):
    """Commanded PWM appears in obs.pwm 1-2 ticks late: still regulates, no fault, no chatter."""
    plant = Plant(
        PlantParams(dt=cfg.dt, heat_w=100.0, delay_ticks=delay),
        initial_pwm=0.5,
        t_coolant=40.0,
        t_air=30.0,
    )
    recs = loop(plant, cfg, 1200)
    assert_bounded_and_finite(recs, cfg)
    assert all(r.cmd.mode is not Mode.FALLBACK for r in recs), "delay looked like a fault"
    assert all(r.cmd.mode is Mode.AUTO for r in recs[-200:])  # settled: no longer at the rail
    # the lag is visible: obs.pwm trails the command by exactly ``delay`` ticks
    for k in range(delay + 5, 40):
        for ch in cfg.channels:
            assert recs[k].obs.pwm[ch] == pytest.approx(recs[k - delay - 1].cmd.pwm[ch], abs=1e-12)
    assert abs(recs[-1].obs.temps[COOLANT] - SP) < 1.0
    tail = recs[-100:]
    for ch in cfg.channels:
        pwm = [r.cmd.pwm[ch] for r in tail]
        assert max(pwm) - min(pwm) < 0.15, (
            f"{ch}: limit cycle with delay {delay}: ripple {max(pwm) - min(pwm):.3f}"
        )
        assert max(abs(b - a) for a, b in zip(pwm, pwm[1:], strict=False)) < 0.05


# ---------------------------------------------------------------------------
# fan stall
# ---------------------------------------------------------------------------


def test_fan_stall_from_start_no_windup_and_diagnosed(cfg):
    """Intake RPM 0 regardless of PWM: coolant cannot reach the setpoint, the controller
    saturates honestly, the integrator never winds beyond pwm_max, and the stall is
    reported in diagnostics."""
    plant = Plant(
        PlantParams(dt=cfg.dt, heat_w=100.0, stalled=frozenset({"intake"})), initial_pwm=0.5
    )
    recs = loop(plant, cfg, 600)
    assert_bounded_and_finite(recs, cfg)
    assert all(r.obs.rpm["intake"] == 0.0 for r in recs)
    assert all(r.obs.rpm["radiator"] > 0.0 for r in recs)
    for r in recs:
        for ch, v in r.state.integrator.items():
            assert v <= cfg.pwm_max + TOL, f"integrator wound up on stalled fan: {ch}={v}"
    tail = recs[-50:]
    assert all(r.cmd.mode is Mode.SATURATED for r in tail), "unreachable setpoint must be honest"
    assert all(r.cmd.pwm["intake"] == pytest.approx(cfg.pwm_max, abs=TOL) for r in tail)
    assert all(r.cmd.diagnostics["fan_stall"]["intake"] is True for r in tail)
    assert all(r.cmd.diagnostics["fan_stall"]["radiator"] is False for r in tail)


def test_fan_stall_mid_run_is_flagged_and_bounded(cfg):
    """A fan that stalls while regulating: flag appears, PWM bounded, no fault from the gate."""
    plant = Plant(PlantParams(dt=cfg.dt, heat_w=60.0), initial_pwm=0.5)
    settled = loop(plant, cfg, 600)
    assert all(r.cmd.mode is Mode.AUTO for r in settled[-50:])
    plant.stalled.add("radiator")
    recs = loop(plant, cfg, 400, state=settled[-1].state)
    assert_bounded_and_finite(recs, cfg)
    assert all(r.obs.rpm["radiator"] == 0.0 for r in recs)
    flagged = next(
        (i for i, r in enumerate(recs) if r.cmd.diagnostics["fan_stall"]["radiator"]), None
    )
    assert flagged is not None, "a fan with RPM 0 at high PWM was never diagnosed"
    assert all(r.cmd.diagnostics["fan_stall"]["radiator"] for r in recs[-20:])
    for r in recs:
        assert r.cmd.mode is not Mode.FALLBACK  # the readings are true: no sensor fault
        for v in r.state.integrator.values():
            assert v <= cfg.pwm_max + TOL


# ---------------------------------------------------------------------------
# long run
# ---------------------------------------------------------------------------


def test_fifteen_minutes_at_dt2_no_drift_to_nan(cfg):
    assert cfg.dt == 2.0
    ticks = int(15 * 60 / cfg.dt)
    plant = Plant(
        PlantParams(dt=cfg.dt, heat_w=100.0, noise_sigma_c=0.2, delay_ticks=1),
        initial_pwm=0.5,
        t_coolant=40.0,
        t_air=30.0,
        seed=15,
    )
    recs = loop(plant, cfg, ticks, heat_schedule=[(150, 130.0), (300, 90.0)])
    assert len(recs) == ticks
    assert_bounded_and_finite(recs, cfg)
    for r in recs:
        json.dumps(r.state.to_dict(), allow_nan=False)
        json.dumps(r.cmd.to_dict(), allow_nan=False)
    assert all(r.cmd.mode is not Mode.FALLBACK for r in recs)
    # state stays bounded: window trimmed, integrator inside the actuator range
    last = recs[-1].state
    assert len(last.window) <= cfg.stuck_ticks
    assert all(cfg.pwm_min - TOL <= v <= cfg.pwm_max + TOL for v in last.integrator.values())
    # the state is a faithful JSON round trip that keeps stepping identically
    restored = MpcState.from_dict(json.loads(json.dumps(last.to_dict())))
    obs = plant.observe()
    cmd_a, st_a = checked_step(obs, cfg, last)
    cmd_b, st_b = checked_step(obs, cfg, restored)
    assert cmd_a.to_dict() == cmd_b.to_dict()
    assert st_a.to_dict() == st_b.to_dict()


def test_long_run_at_the_rail_does_not_leak_growth(cfg):
    """An unreachable setpoint for 15 minutes: nothing in the state grows without bound."""
    ticks = int(15 * 60 / cfg.dt)
    plant = Plant(PlantParams(dt=cfg.dt, heat_w=220.0), initial_pwm=0.5)
    recs = loop(plant, cfg, ticks)
    assert_bounded_and_finite(recs, cfg)
    sizes = [len(json.dumps(r.state.to_dict())) for r in recs]
    assert max(sizes[-100:]) <= max(sizes[: cfg.stuck_ticks + 10]) + 512, "state keeps growing"


# ---------------------------------------------------------------------------
# bumpless after a long fallback
# ---------------------------------------------------------------------------


def test_bumpless_return_after_long_fallback(cfg):
    """Long fallback (ramped to fallback_pwm) then return: the first auto PWM equals the
    last command and the integrator did not wind while the output was unused."""
    plant = Plant(PlantParams(dt=cfg.dt, heat_w=100.0), initial_pwm=0.5, t_coolant=40.0)
    settle, dropout, restore = 600, 150, 60

    def hook(i, obs):
        if settle <= i < settle + dropout:
            return dataclasses.replace(obs, temps={**obs.temps, COOLANT: None})
        return obs

    recs = loop(plant, cfg, settle + dropout + restore, observe_hook=hook)
    before = recs[settle - 1]
    fb = recs[settle : settle + dropout]
    assert all(r.cmd.mode is Mode.FALLBACK for r in fb)
    assert all(r.state.integrator == before.state.integrator for r in fb), (
        "integrator moved in fallback"
    )
    assert fb[-1].cmd.pwm == pytest.approx(cfg.fallback_pwm, abs=TOL)
    assert {r.state.fault_since_ts for r in fb} == {fb[0].obs.ts}
    after = recs[settle + dropout :]
    first_auto = next(i for i, r in enumerate(after) if r.cmd.mode is not Mode.FALLBACK)
    assert first_auto == cfg.confirm_ticks - 1, "auto must return after exactly confirm_ticks"
    ret = after[first_auto]
    last_cmd = after[first_auto - 1].cmd.pwm if first_auto else fb[-1].cmd.pwm
    assert ret.cmd.pwm == pytest.approx(last_cmd, abs=1e-9), "first auto PWM != last_cmd"
    assert ret.state.fault_since_ts is None and ret.state.fault_reason is None
    # no rail, no jump: the integrator was re-initialised, not stale and not wound
    assert all(cfg.pwm_min - TOL <= v <= cfg.pwm_max + TOL for v in ret.state.integrator.values())
    for a, b in zip(after[first_auto:], after[first_auto + 1 :], strict=False):
        for ch in cfg.channels:
            assert abs(b.cmd.pwm[ch] - a.cmd.pwm[ch]) <= cfg.d_pwm_max + TOL
    # and it regulates again: PWM heads back down from fallback_pwm toward the working point
    assert after[-1].cmd.pwm["radiator"] < cfg.fallback_pwm["radiator"] - 0.05
