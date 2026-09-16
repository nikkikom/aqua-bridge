"""Section 4.3 failures on ``mpc.step``: everything that is not a lying sensor.

One case per bullet ("Observation / time", "Values", "Config / solver"),
plus the hold-then-ramp-high policy, Flicker, recovery through
``confirm_ticks`` and bumpless transfer after a long fallback (section
4.6 last bullet). Every step goes through :func:`invariants.checked_step`.
"""

from __future__ import annotations

import dataclasses
import math
from typing import Any

import pytest

from aqua_bridge.control import mpc
from aqua_bridge.control.mpc import GAP_TICKS_MAX, STALL_TICKS, step
from aqua_bridge.control.solver_pi import PiSolver, Solver, SolverRequest, SolverResult
from aqua_bridge.model import ConfigError, FaultReason, Mode, MpcConfig, MpcState, SolverKind
from aqua_bridge.sim.plant import Plant, PlantParams, run_closed_loop
from invariants import TOL, checked_step, make_obs

# Legacy-shaped scenarios (coolant setpoint): the DAS cases run in test_das_core.py.
pytestmark = pytest.mark.solver_cases("pi", "mpc")

SP = 35.0


@pytest.fixture
def cfg(cfg: MpcConfig, solver_kind) -> MpcConfig:
    """Section 8: every scenario in this module runs for the PI and the MPC solver."""
    return dataclasses.replace(cfg, solver=solver_kind)


def warm(cfg: MpcConfig, ticks: int = 3, pwm: float = 0.5, coolant: float = SP) -> MpcState:
    """A few trusted ticks from cold so ``last_cmd`` exists and mode is auto."""
    state = MpcState.cold()
    for i in range(ticks):
        obs = make_obs(cfg, i * cfg.dt, coolant=coolant, pwm=dict.fromkeys(cfg.channels, pwm))
        cmd, state = checked_step(obs, cfg, state)
        assert cmd.mode is not Mode.FALLBACK
    return state


def next_ts(state: MpcState, cfg: MpcConfig) -> float:
    return float(state.solver_memory["last_ts"]) + cfg.dt


# ---------------------------------------------------------------------------
# Observation / time
# ---------------------------------------------------------------------------


@pytest.mark.parametrize(
    "temps",
    [
        {"air": 30.0},  # missing key
        {"coolant": SP, "air": 30.0, "gpu": 40.0},  # extra unknown key
        {},  # empty
    ],
    ids=["missing", "extra", "empty"],
)
def test_temps_keys_mismatch_untrusted_never_crash(fast_cfg, temps):
    state = warm(fast_cfg)
    obs = make_obs(fast_cfg, next_ts(state, fast_cfg), temps=temps)
    cmd, nxt = checked_step(obs, fast_cfg, state)
    assert cmd.mode is Mode.FALLBACK
    assert nxt.fault_reason is FaultReason.SENSOR_GATE
    assert cmd.pwm == state.last_cmd.pwm  # hold


def test_rpm_and_pwm_missing_for_a_channel_still_steps(fast_cfg):
    state = warm(fast_cfg)
    obs = make_obs(fast_cfg, next_ts(state, fast_cfg), rpm={"radiator": 900.0}, pwm={"intake": 0.5})
    cmd, nxt = checked_step(obs, fast_cfg, state)
    assert cmd.mode is Mode.AUTO  # temperatures are fine; PWM/RPM are not gate inputs
    assert cmd.diagnostics["obs_pwm_usable"] is False
    assert nxt.solver_memory["stall_ticks"] == {"radiator": 0, "intake": 0}


def test_rpm_and_pwm_missing_on_cold_state_uses_fallback_prev(fast_cfg):
    obs = make_obs(fast_cfg, 0.0, rpm={}, pwm={})
    cmd, _ = checked_step(obs, fast_cfg, MpcState.cold())
    assert cmd.diagnostics["prev_source"] == "fallback_pwm"
    assert cmd.pwm == pytest.approx(fast_cfg.fallback_pwm)


def test_ts_not_advancing_is_untrusted(fast_cfg):
    state = warm(fast_cfg)
    ts = float(state.solver_memory["last_ts"])
    cmd, nxt = checked_step(make_obs(fast_cfg, ts), fast_cfg, state)
    assert cmd.mode is Mode.FALLBACK
    assert cmd.diagnostics["time"]["status"] == "not_advancing"
    assert cmd.diagnostics["trusted"] is False
    assert nxt.trusted_streak == 0


def test_duplicate_step_with_identical_ts_is_deterministic_and_untrusted(fast_cfg):
    state = warm(fast_cfg)
    obs = make_obs(fast_cfg, next_ts(state, fast_cfg))
    cmd1, s1 = checked_step(obs, fast_cfg, state)
    assert cmd1.mode is Mode.AUTO
    cmd2, s2 = checked_step(obs, fast_cfg, s1)  # same ts again
    assert cmd2.mode is Mode.FALLBACK
    assert cmd2.pwm == cmd1.pwm
    # replaying the same call from the same state is bit-identical
    cmd3, s3 = step(obs, fast_cfg, s1)
    assert cmd3.to_dict() == cmd2.to_dict() and s3.to_dict() == s2.to_dict()


def test_ts_going_backwards_then_recovering(fast_cfg):
    cfg = fast_cfg
    state = warm(cfg)
    ts = float(state.solver_memory["last_ts"])
    cmd, state = checked_step(make_obs(cfg, ts - 50.0), cfg, state)
    assert cmd.mode is Mode.FALLBACK
    assert cmd.diagnostics["time"]["status"] == "not_advancing"
    # the new timeline is accepted: confirm_ticks trusted ticks return to auto
    modes = []
    for k in range(1, cfg.confirm_ticks + 1):
        cmd, state = checked_step(make_obs(cfg, ts - 50.0 + k * cfg.dt), cfg, state)
        modes.append(cmd.mode)
    assert modes[-1] is Mode.AUTO
    assert all(m is Mode.FALLBACK for m in modes[:-1])


def test_permanently_stuck_clock_ramps_high_by_tick_count(fast_cfg):
    cfg = fast_cfg
    state = warm(cfg, pwm=0.3)
    ts = float(state.solver_memory["last_ts"])
    last = state.last_cmd.pwm
    for _ in range(40):
        cmd, state = checked_step(make_obs(cfg, ts), cfg, state)
        assert cmd.mode is Mode.FALLBACK
        for ch in cfg.channels:
            assert cmd.pwm[ch] >= last[ch] - TOL
        last = cmd.pwm
    assert cmd.pwm == pytest.approx(cfg.fallback_pwm)


def test_gap_much_larger_than_dt_drops_history_and_recovers(fast_cfg):
    cfg = fast_cfg
    state = warm(cfg, ticks=6)
    assert len(state.window) == cfg.stuck_ticks
    ts = float(state.solver_memory["last_ts"]) + (GAP_TICKS_MAX + 2) * cfg.dt
    cmd, state = checked_step(make_obs(cfg, ts), cfg, state)
    assert cmd.mode is Mode.FALLBACK
    assert cmd.diagnostics["time"]["status"] == "gap"
    assert len(state.window) == 1  # history dropped, this sample pushed
    assert state.last_good_obs is not None  # kept
    for k in range(1, cfg.confirm_ticks + 1):
        cmd, state = checked_step(make_obs(cfg, ts + k * cfg.dt), cfg, state)
    assert cmd.mode is Mode.AUTO


def test_moderate_gap_scales_slew_limit(fast_cfg):
    cfg = fast_cfg
    state = warm(cfg)
    ts = float(state.solver_memory["last_ts"])
    jump = 1.5 * cfg.dT_max_tick  # too fast for one tick, fine for two
    # without a gap: rejected
    cmd, _ = checked_step(make_obs(cfg, ts + cfg.dt, coolant=SP + jump), cfg, state)
    assert cmd.mode is Mode.FALLBACK
    # after a 2*dt gap the same move is plausible
    cmd, _ = checked_step(make_obs(cfg, ts + 2 * cfg.dt, coolant=SP + jump), cfg, state)
    assert cmd.mode is Mode.AUTO
    assert cmd.diagnostics["time"]["dT_limit"] == pytest.approx(2 * cfg.dT_max_tick)


# ---------------------------------------------------------------------------
# Values
# ---------------------------------------------------------------------------


def test_none_in_dict_value_is_fallback(fast_cfg):
    state = warm(fast_cfg)
    cmd, nxt = checked_step(make_obs(fast_cfg, next_ts(state, fast_cfg), air=None), fast_cfg, state)
    assert cmd.mode is Mode.FALLBACK
    assert nxt.last_raw_temps["air"] is None


@pytest.mark.parametrize("value", [-25.0, 125.0, 32767.0, 65535.0, 35000.0])
def test_out_of_range_temps_are_invalid_not_a_plant(fast_cfg, value):
    state = warm(fast_cfg)
    cmd, nxt = checked_step(
        make_obs(fast_cfg, next_ts(state, fast_cfg), coolant=value), fast_cfg, state
    )
    assert cmd.mode is Mode.FALLBACK
    assert "range" in cmd.diagnostics["gate"]["reasons"]["coolant"]
    assert nxt.last_good_obs.temps["coolant"] == SP  # last good unchanged


@pytest.mark.parametrize("value", [-0.5, 1.5, math.nan, math.inf])
def test_pwm_obs_outside_unit_interval(fast_cfg, value):
    # cold: cannot serve as prev -> fallback_pwm
    obs = make_obs(fast_cfg, 0.0, pwm=dict.fromkeys(fast_cfg.channels, value))
    cmd, _ = checked_step(obs, fast_cfg, MpcState.cold())
    assert cmd.diagnostics["prev_source"] == "fallback_pwm"
    assert cmd.pwm == pytest.approx(fast_cfg.fallback_pwm)
    # warm: ignored, last_cmd rules
    state = warm(fast_cfg)
    obs = make_obs(fast_cfg, next_ts(state, fast_cfg), pwm=dict.fromkeys(fast_cfg.channels, value))
    cmd, _ = checked_step(obs, fast_cfg, state)
    assert cmd.mode is Mode.AUTO
    assert cmd.diagnostics["prev_source"] == "last_cmd"


def test_fan_stall_flagged_and_integrator_capped(fast_cfg):
    cfg = fast_cfg
    state = MpcState.cold()
    flagged_at = None
    for i in range(STALL_TICKS + 15):
        # The demand stays high; the wobble (> stuck_eps_c) keeps both sensors
        # alive, otherwise a reading frozen while the PWM ramps to the rail is
        # the gate's Stuck case (section 3 rule 3), not a fan stall.
        wobble = 0.05 if i % 2 else 0.0
        obs = make_obs(
            cfg,
            i * cfg.dt,
            coolant=SP + 20.0 + wobble,
            air=30.0 + wobble,
            pwm=dict.fromkeys(cfg.channels, 0.7),
            rpm=dict.fromkeys(cfg.channels, 0.0),
        )
        cmd, state = checked_step(obs, cfg, state)
        if flagged_at is None and all(cmd.diagnostics["fan_stall"].values()):
            flagged_at = i
    assert flagged_at is not None and flagged_at <= STALL_TICKS + 2
    assert all(v <= cfg.pwm_max + TOL for v in state.integrator.values())
    assert cmd.mode is Mode.SATURATED
    assert cmd.pwm == pytest.approx(dict.fromkeys(cfg.channels, cfg.pwm_max))
    # RPM back: counters reset
    obs = make_obs(
        cfg, next_ts(state, cfg), coolant=SP + 20.0, rpm=dict.fromkeys(cfg.channels, 1500.0)
    )
    cmd, state = checked_step(obs, cfg, state)
    assert not any(cmd.diagnostics["fan_stall"].values())


def test_stall_only_counts_when_command_is_high(fast_cfg):
    cfg = fast_cfg
    state = warm(cfg, pwm=0.2, coolant=SP - 10.0)  # cold plant, low command
    for _ in range(STALL_TICKS + 5):
        obs = make_obs(
            cfg, next_ts(state, cfg), coolant=SP - 10.0, rpm=dict.fromkeys(cfg.channels, 0.0)
        )
        cmd, state = checked_step(obs, cfg, state)
    assert not any(cmd.diagnostics["fan_stall"].values())


# ---------------------------------------------------------------------------
# Config / solver
# ---------------------------------------------------------------------------


@pytest.mark.parametrize("horizon", [1, 200])
def test_horizon_extremes_still_satisfy_invariants(cfg, horizon):
    c = dataclasses.replace(cfg, horizon=horizon)
    state = MpcState.cold()
    for i in range(10):
        _, state = checked_step(make_obs(c, i * c.dt, coolant=SP + 3.0), c, state)


@pytest.mark.parametrize(
    "patch",
    [
        {"dt": 0.0},
        {"dt": -1.0},
        {"pwm_min": 0.9, "pwm_max": 0.5},
        {"channels": ()},
        {"fallback_pwm": {"radiator": 0.8}},
        {"fallback_pwm": {"radiator": 0.8, "intake": 0.8, "extra": 0.8}},
        {"fallback_pwm": {"radiator": 0.15, "intake": 0.8}},  # == pwm_min
        {"fallback_hold_s": -1.0},
        {"confirm_s": 1.0},  # < 2 * dt
        {"confirm_s": 30.0},  # > fallback_hold_s
        {"dT_max_c_per_s": 0.0},
        {"stuck_s": 1.0},  # < 2 * dt
        {"temps": ()},
        {"setpoints": {"gpu": 40.0}},
        {"stuck_eps_c": 0.0},
        {"stuck_pwm_net": 0.0},
        {"stuck_pwm_net": 1.5},
        {"stuck_sibling_dT_c": 0.0},
        {"stuck_airflow_net": 0.0},
        {"stuck_airflow_net": 1.5},
        {"stuck_air_oppose_c": 0.0},
        {"stuck_air_oppose_c": -0.3},
        {"stuck_air_oppose_max_c": 0.3},  # == stuck_air_oppose_c
        {"stuck_air_oppose_max_c": 0.1},  # < stuck_air_oppose_c
        {"stuck_zone_air_dT_c": 0.0},
        {"stuck_zone_air_dT_c": -1.5},
        {"median3": "false"},
        {"median3": 0},
    ],
    ids=lambda p: "-".join(f"{k}={v!r}" for k, v in p.items()),
)
def test_config_rejections(cfg, patch):
    with pytest.raises(ConfigError):
        dataclasses.replace(cfg, **patch)


# -- solver faults -----------------------------------------------------------


class ExplodingSolver:
    name = "boom"

    def initialise(self, cfg, req):
        return PiSolver().initialise(cfg, req)

    def solve(self, cfg, req):
        raise RuntimeError("solver exploded")


class NaNSolver(PiSolver):
    name = "nan"

    def solve(self, cfg, req):
        r = super().solve(cfg, req)
        return dataclasses.replace(r, pwm={ch: math.nan for ch in cfg.channels})


class CapSolver(PiSolver):
    name = "cap"

    def solve(self, cfg, req):
        return dataclasses.replace(super().solve(cfg, req), converged=False, iterations=999)


class TooManyIterSolver(PiSolver):
    name = "iters"

    def solve(self, cfg, req):
        return dataclasses.replace(super().solve(cfg, req), iterations=cfg.solver_max_iter + 1)


class WrongKeysSolver(PiSolver):
    name = "keys"

    def solve(self, cfg, req):
        return dataclasses.replace(super().solve(cfg, req), pwm={"nope": 0.5})


class InfIntegratorSolver(PiSolver):
    name = "inf"

    def solve(self, cfg, req):
        r = super().solve(cfg, req)
        return dataclasses.replace(r, integrator={ch: math.inf for ch in cfg.channels})


class NotAResultSolver(PiSolver):
    name = "dict"

    def solve(self, cfg, req):  # type: ignore[override]
        return {"pwm": dict.fromkeys(cfg.channels, 0.5)}


BROKEN: list[Any] = [
    ExplodingSolver(),
    NaNSolver(),
    CapSolver(),
    TooManyIterSolver(),
    WrongKeysSolver(),
    InfIntegratorSolver(),
    NotAResultSolver(),
]


@pytest.mark.parametrize("bad", BROKEN, ids=lambda s: s.name)
def test_broken_solver_is_a_fault_not_an_exception(fast_cfg, bad):
    cfg = fast_cfg
    state = warm(cfg)
    cmd, nxt = checked_step(make_obs(cfg, next_ts(state, cfg)), cfg, state, solver=bad)
    assert cmd.mode is Mode.FALLBACK
    assert nxt.fault_reason is FaultReason.SOLVER
    assert cmd.diagnostics["solver_error"]
    assert cmd.pwm == state.last_cmd.pwm  # hold
    assert nxt.integrator == state.integrator  # nothing half-updated
    assert nxt.trusted_streak == 0


def test_solver_fault_on_cold_state_commands_fallback_pwm(fast_cfg):
    obs = make_obs(fast_cfg, 0.0, pwm={})
    cmd, nxt = checked_step(obs, fast_cfg, MpcState.cold(), solver=ExplodingSolver())
    assert cmd.pwm == pytest.approx(fast_cfg.fallback_pwm)
    assert nxt.fault_reason is FaultReason.SOLVER


def test_persistent_solver_fault_keeps_one_timer_and_ramps_high(fast_cfg):
    cfg = fast_cfg
    state = warm(cfg, pwm=0.3)
    since = None
    last = state.last_cmd.pwm
    for _ in range(25):
        cmd, state = checked_step(
            make_obs(cfg, next_ts(state, cfg)), cfg, state, solver=ExplodingSolver()
        )
        assert cmd.mode is Mode.FALLBACK
        since = state.fault_since_ts if since is None else since
        assert state.fault_since_ts == since, "a failed retry must not restart the hold timer"
        for ch in cfg.channels:
            assert cmd.pwm[ch] >= last[ch] - TOL
        last = cmd.pwm
    assert cmd.pwm == pytest.approx(cfg.fallback_pwm)
    assert cmd.diagnostics["policy"] == "ramp_high"


def test_solver_recovers_after_confirm_ticks(fast_cfg):
    cfg = fast_cfg
    state = warm(cfg)
    cmd, state = checked_step(
        make_obs(cfg, next_ts(state, cfg)), cfg, state, solver=ExplodingSolver()
    )
    assert cmd.mode is Mode.FALLBACK
    modes = []
    for _ in range(cfg.confirm_ticks):
        cmd, state = checked_step(make_obs(cfg, next_ts(state, cfg)), cfg, state)
        modes.append(cmd.mode)
    assert modes[-1] is Mode.AUTO and all(m is Mode.FALLBACK for m in modes[:-1])
    assert not state.in_fault


def test_missing_solver_registration_is_a_solver_fault(fast_cfg, monkeypatch):
    monkeypatch.delitem(mpc.SOLVERS, fast_cfg.solver)
    cmd, nxt = checked_step(make_obs(fast_cfg, 0.0), fast_cfg, MpcState.cold())
    assert cmd.mode is Mode.FALLBACK
    assert nxt.fault_reason is FaultReason.SOLVER
    assert "no solver registered" in cmd.diagnostics["solver_error"]


def test_pi_solver_satisfies_protocol():
    assert isinstance(PiSolver(), Solver)
    assert isinstance(ExplodingSolver(), Solver)


def test_pi_bumpless_initialise_reproduces_prev(cfg):
    pi = PiSolver()
    req = SolverRequest(
        temps={"coolant": SP + 4.0, "air": 30.0}, prev_pwm=dict.fromkeys(cfg.channels, 0.42)
    )
    integ, mem = pi.initialise(cfg, req)
    res = pi.solve(cfg, dataclasses.replace(req, integrator=integ, memory=mem))
    assert isinstance(res, SolverResult)
    assert res.pwm == pytest.approx(dict.fromkeys(cfg.channels, 0.42), abs=1e-12)


# ---------------------------------------------------------------------------
# hold -> ramp high, Flicker, recovery, bumpless
# ---------------------------------------------------------------------------


def test_hold_then_ramp_high_never_toward_pwm_min(fast_cfg):
    cfg = fast_cfg
    state = warm(cfg, pwm=0.3)
    held = state.last_cmd.pwm
    t0 = next_ts(state, cfg)
    trace = []
    for k in range(20):
        cmd, state = checked_step(make_obs(cfg, t0 + k * cfg.dt, coolant=None), cfg, state)
        trace.append(cmd)
        assert cmd.mode is Mode.FALLBACK
        assert state.fault_since_ts == t0
    # hold for fallback_hold_s (elapsed 0..hold inclusive), then ramp
    hold_ticks = int(cfg.fallback_hold_s / cfg.dt) + 1
    for cmd in trace[:hold_ticks]:
        assert cmd.pwm == pytest.approx(held)
        assert cmd.diagnostics["policy"] == "hold"
    ramp = trace[hold_ticks:]
    assert ramp[0].diagnostics["policy"] == "ramp_high"
    prev = held
    for cmd in ramp:
        for ch in cfg.channels:
            assert cmd.pwm[ch] >= prev[ch] - TOL
            assert cmd.pwm[ch] - prev[ch] <= cfg.d_pwm_max + TOL
        prev = cmd.pwm
    assert trace[-1].pwm == pytest.approx(cfg.fallback_pwm)


@pytest.mark.parametrize("start", [1.0, 0.9], ids=["at_pwm_max", "between"])
@pytest.mark.parametrize("fault", ["sensor", "solver"])
def test_fault_above_fallback_pwm_never_lowers_the_fans(fast_cfg, start, fault):
    """Review finding F1: a fault that begins with the fans above ``fallback_pwm``
    (hot plant, saturated) must hold them there, not ramp them down to
    ``fallback_pwm`` while the controller is blind (section 4.1, "never a step
    toward pwm_min because of the fault")."""
    cfg = fast_cfg
    assert start > max(cfg.fallback_pwm.values())
    # One warm tick: the bumpless first output equals prev, so last_cmd == start;
    # more ticks would ramp a hot plant on to pwm_max anyway.
    state = warm(cfg, ticks=1, pwm=start, coolant=SP + 20.0)
    held = dict(state.last_cmd.pwm)
    assert held == pytest.approx(dict.fromkeys(cfg.channels, start))
    for _ in range(int(cfg.fallback_hold_s / cfg.dt) + 8):
        if fault == "sensor":
            obs = make_obs(cfg, next_ts(state, cfg), coolant=None)
            cmd, state = checked_step(obs, cfg, state)
        else:
            obs = make_obs(cfg, next_ts(state, cfg), coolant=SP + 20.0)
            cmd, state = checked_step(obs, cfg, state, solver=ExplodingSolver())
        assert cmd.mode is Mode.FALLBACK
        assert cmd.pwm == pytest.approx(held), "a fault lowered the fans"
    assert cmd.diagnostics["policy"] == "ramp_high"
    assert cmd.diagnostics["target_pwm"] == pytest.approx(held)


def test_ramp_high_target_is_max_of_prev_and_fallback_per_channel(fast_cfg):
    """One channel above fallback_pwm is held, the other is raised to it."""
    cfg = fast_cfg
    state = warm(cfg, pwm=0.3)
    # Pin the radiator high by hand: an applied command the loop would feed back.
    pinned = dict(state.last_cmd.pwm)
    pinned["radiator"] = 1.0
    state = dataclasses.replace(state, last_cmd=dataclasses.replace(state.last_cmd, pwm=pinned))
    for _ in range(int(cfg.fallback_hold_s / cfg.dt) + 10):
        cmd, state = checked_step(make_obs(cfg, next_ts(state, cfg), coolant=None), cfg, state)
        assert cmd.pwm["radiator"] == pytest.approx(1.0)
    assert cmd.pwm["intake"] == pytest.approx(cfg.fallback_pwm["intake"])
    assert cmd.diagnostics["target_pwm"] == {
        "radiator": 1.0,
        "intake": cfg.fallback_pwm["intake"],
    }


def test_flicker_keeps_fallback_and_does_not_reset_timer_or_chatter(fast_cfg):
    cfg = fast_cfg
    state = warm(cfg, pwm=0.3)
    t0 = next_ts(state, cfg)
    since = None
    prev = state.last_cmd.pwm
    for k in range(30):
        lie = k % 2 == 0
        obs = make_obs(cfg, t0 + k * cfg.dt, coolant=None if lie else SP)
        cmd, state = checked_step(obs, cfg, state)
        assert cmd.mode is Mode.FALLBACK
        since = state.fault_since_ts if since is None else since
        assert state.fault_since_ts == since
        assert state.trusted_streak < cfg.confirm_ticks
        for ch in cfg.channels:  # monotonic: hold, then climb; never down, never oscillating
            assert cmd.pwm[ch] >= prev[ch] - TOL
        prev = cmd.pwm
    assert cmd.pwm == pytest.approx(cfg.fallback_pwm)


def test_recovery_requires_confirm_ticks_and_is_bumpless(fast_cfg):
    cfg = fast_cfg
    state = warm(cfg)
    t0 = next_ts(state, cfg)
    cmd, state = checked_step(make_obs(cfg, t0, coolant=125.0), cfg, state)  # Spike
    assert cmd.mode is Mode.FALLBACK
    good_cmds = []
    for k in range(1, cfg.confirm_ticks + 1):
        cmd, state = checked_step(make_obs(cfg, t0 + k * cfg.dt, coolant=SP + 1.0), cfg, state)
        good_cmds.append(cmd)
    assert [c.mode for c in good_cmds] == [Mode.FALLBACK] * (cfg.confirm_ticks - 1) + [Mode.AUTO]
    back = good_cmds[-1]
    assert back.diagnostics["returning_to_auto"] is True
    assert back.diagnostics["target_pwm"] == pytest.approx(good_cmds[-2].pwm, abs=1e-9)
    assert back.pwm == pytest.approx(good_cmds[-2].pwm, abs=1e-9)
    assert not state.in_fault and state.trusted_streak == cfg.confirm_ticks


def test_spike_does_not_move_last_good(fast_cfg):
    cfg = fast_cfg
    state = warm(cfg)
    good = state.last_good_obs
    cmd, state = checked_step(make_obs(cfg, next_ts(state, cfg), coolant=125.0), cfg, state)
    assert cmd.mode is Mode.FALLBACK
    assert state.last_good_obs == good
    # rule 5: the raw value is stored even though untrusted (only NaN/inf/missing become None)
    assert state.last_raw_temps["coolant"] == 125.0
    assert state.window[-1].raw_temps["coolant"] == 125.0


def test_bumpless_after_long_fallback(cfg):
    """Closed loop: settle, drop the coolant sensor for a long time, restore.

    While in fallback the integrator must not move; on the tick that returns
    to auto the solver output must equal the last fallback command (before
    the rate limit) and the integrator must be re-initialised, not stale.
    """
    plant = Plant(PlantParams(dt=cfg.dt, heat_w=100.0), initial_pwm=0.5, t_coolant=40.0)
    settle = 400
    dropout = 120
    restore = 40

    def hook(i, obs):
        if settle <= i < settle + dropout:
            temps = dict(obs.temps)
            temps["coolant"] = None
            return dataclasses.replace(obs, temps=temps)
        return obs

    recs = run_closed_loop(
        plant, cfg, checked_step, ticks=settle + dropout + restore, observe_hook=hook
    )
    before = recs[settle - 1]
    fb = recs[settle : settle + dropout]
    assert all(r.cmd.mode is Mode.FALLBACK for r in fb)
    assert all(r.state.integrator == before.state.integrator for r in fb), "integrator wound"
    assert fb[-1].cmd.pwm == pytest.approx(cfg.fallback_pwm)  # ramped high and stayed
    after = recs[settle + dropout :]
    first_auto = next(i for i, r in enumerate(after) if r.cmd.mode is not Mode.FALLBACK)
    assert first_auto == cfg.confirm_ticks - 1
    ret = after[first_auto]
    last_fb = after[first_auto - 1].cmd.pwm
    assert ret.cmd.diagnostics["target_pwm"] == pytest.approx(last_fb, abs=1e-9)
    assert ret.cmd.pwm == pytest.approx(last_fb, abs=1e-9)
    # the integrator was re-initialised: output == prev exactly with the current error
    err = ret.obs.temps["coolant"] - cfg.setpoints["coolant"]
    diag = ret.cmd.diagnostics["solver_diag"]
    if cfg.solver is SolverKind.PI:
        for ch in cfg.channels:
            expected_i = last_fb[ch] - cfg.pi_kp * err
            # solve() then advances I by one ki*e*dt step (and clamps); check the pre-step value
            assert diag["i_term"][ch] == pytest.approx(expected_i)
    else:
        # MPC: the disturbance estimate was re-initialised (fresh) so that the constrained
        # first move is exactly prev, and the estimator did not run on that tick
        assert diag["fresh"] is True and diag["residual"] is None
        assert diag["error"]["coolant"] == pytest.approx(err)
    # and regulation continues without a jump: the following ticks move by <= d_pwm_max
    for a, b in zip(after[first_auto:], after[first_auto + 1 :], strict=False):
        for ch in cfg.channels:
            assert abs(b.cmd.pwm[ch] - a.cmd.pwm[ch]) <= cfg.d_pwm_max + TOL


def test_median3_variant_of_step_handles_spike_without_fallback(fast_cfg):
    cfg = dataclasses.replace(fast_cfg, median3=True)
    state = warm(cfg, ticks=4)
    cmd, state = checked_step(make_obs(cfg, next_ts(state, cfg), coolant=125.0), cfg, state)
    assert cmd.mode is Mode.AUTO  # the median removed the spike
    cmd, state = checked_step(make_obs(cfg, next_ts(state, cfg), coolant=SP), cfg, state)
    assert cmd.mode is Mode.AUTO
    assert state.last_good_obs.temps["coolant"] == SP  # filtered value, not the raw 125
