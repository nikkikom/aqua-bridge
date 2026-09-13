"""Unit tests for the linear MPC solver (``aqua_bridge.control.solver_mpc``).

The section 4 scenarios run for both solvers through the ``cfg`` override
in every ``test_mpc_*`` module; this file checks what is specific to the
MPC itself: the box QP against a brute-force reference, the iteration cap,
warm start, offset-free tracking, honest saturation, bumpless
initialisation and the protocol contract.
"""

from __future__ import annotations

import dataclasses
import json
import math

import numpy as np
import pytest

from aqua_bridge.control import solver_mpc
from aqua_bridge.control.mpc import SOLVERS, step
from aqua_bridge.control.solver_mpc import (
    BUMPLESS_TOL,
    MpcSolver,
    build_problem,
    solve_box_qp,
)
from aqua_bridge.control.solver_pi import Solver, SolverRequest, SolverResult
from aqua_bridge.model import ConfigError, Mode, MpcConfig, MpcState, SolverKind
from invariants import TOL, checked_step, make_obs

SP = 35.0


@pytest.fixture
def mcfg(cfg: MpcConfig) -> MpcConfig:
    return dataclasses.replace(cfg, solver=SolverKind.MPC)


# ---------------------------------------------------------------------------
# box QP
# ---------------------------------------------------------------------------


def _random_pd(rng: np.random.Generator, n: int) -> np.ndarray:
    a = rng.normal(size=(n, n))
    return a @ a.T + 0.1 * np.eye(n)


def _brute_force_box_qp(H: np.ndarray, f: np.ndarray, lo, hi) -> np.ndarray:
    """Projected gradient to convergence: slow but independent of the active-set code."""
    L = float(np.linalg.eigvalsh(H)[-1])
    x = np.clip(np.zeros_like(f), lo, hi)
    for _ in range(200_000):
        g = H @ x + f
        nxt = np.clip(x - g / L, lo, hi)
        if np.max(np.abs(nxt - x)) < 1e-13:
            return nxt
        x = nxt
    return x


@pytest.mark.parametrize("seed", range(12))
def test_box_qp_matches_projected_gradient_reference(seed):
    rng = np.random.default_rng(seed)
    n = int(rng.integers(2, 14))
    H = _random_pd(rng, n)
    f = rng.normal(scale=3.0, size=n)
    lo = np.full(n, 0.15)
    hi = np.full(n, 1.0)
    x0 = rng.uniform(0.0, 1.2, size=n)
    res = solve_box_qp(H, f, lo, hi, x0, float(np.linalg.eigvalsh(H)[-1]), max_iter=200)
    assert res.converged
    ref = _brute_force_box_qp(H, f, lo, hi)
    assert res.x == pytest.approx(ref, abs=1e-8)
    assert np.all(res.x >= lo - 1e-12) and np.all(res.x <= hi + 1e-12)
    # KKT: free variables have zero gradient, bounds have the right multiplier sign
    g = H @ res.x + f
    for i in range(n):
        if res.side[i] == 0:
            assert abs(g[i]) < 1e-8
        elif res.side[i] < 0:
            assert g[i] >= -1e-8
        else:
            assert g[i] <= 1e-8


def test_box_qp_iteration_cap_reports_non_convergence():
    rng = np.random.default_rng(3)
    n = 10
    H = _random_pd(rng, n)
    f = rng.normal(scale=5.0, size=n)
    lo, hi = np.zeros(n), np.ones(n)
    res = solve_box_qp(H, f, lo, hi, np.full(n, 0.5), float(np.linalg.eigvalsh(H)[-1]), 1)
    full = solve_box_qp(H, f, lo, hi, np.full(n, 0.5), float(np.linalg.eigvalsh(H)[-1]), 200)
    assert full.converged
    if not res.converged:
        assert res.iterations == 1
        assert np.all(res.x >= lo) and np.all(res.x <= hi)  # still a feasible point


def test_box_qp_warm_start_at_optimum_is_a_fixed_point():
    rng = np.random.default_rng(11)
    n = 8
    H = _random_pd(rng, n)
    f = rng.normal(scale=2.0, size=n)
    lo, hi = np.full(n, 0.15), np.ones(n)
    L = float(np.linalg.eigvalsh(H)[-1])
    first = solve_box_qp(H, f, lo, hi, np.full(n, 0.5), L, 200)
    again = solve_box_qp(H, f, lo, hi, first.x, L, 200)
    assert again.converged and again.iterations <= 2
    assert again.x == pytest.approx(first.x, abs=1e-14)


# ---------------------------------------------------------------------------
# problem matrices
# ---------------------------------------------------------------------------


def test_problem_matrices_predict_the_model(mcfg):
    prob = build_problem(mcfg)
    assert prob.temps == ("coolant",) and prob.channels == mcfg.channels
    N, m = mcfg.horizon, prob.m
    rng = np.random.default_rng(0)
    T0 = np.array([40.0])
    d = np.array([0.3])
    U = rng.uniform(0.15, 1.0, size=N * m)
    # step the model by hand
    T = T0.copy()
    expected = []
    for k in range(N):
        T = prob.a * T + prob.B @ U[k * m : (k + 1) * m] + d
        expected.append(T.copy())
    pred = prob.Phi @ T0 + prob.S @ U + prob.Gamma @ d
    assert pred == pytest.approx(np.concatenate(expected), abs=1e-12)
    # steady state: T_ss = (B u + d) / (1 - a); u_ss inverts it at the setpoint
    u = np.array([0.6, 0.6])
    d_eq = prob.d_equilibrium(np.array([SP]), u)
    assert prob.u_ss(d_eq, mcfg.pwm_min, mcfg.pwm_max) == pytest.approx(u, abs=1e-12)
    assert math.exp(-mcfg.dt / mcfg.mpc_tau_s) == pytest.approx(prob.a)
    assert prob.B[0, 0] == pytest.approx(-(1 - prob.a) * mcfg.mpc_gain_c_per_pwm)
    assert np.allclose(prob.H, prob.H.T)
    assert np.linalg.eigvalsh(prob.H)[0] > 0


def test_problem_is_cached_per_config(mcfg):
    a = build_problem(mcfg)
    assert build_problem(mcfg) is a
    other = dataclasses.replace(mcfg, setpoints={"coolant": 36.0})
    assert build_problem(other) is not a
    assert build_problem(dataclasses.replace(mcfg, median3=True)) is a  # irrelevant field


def test_channel_temps_give_a_block_diagonal_gain(mcfg):
    c = dataclasses.replace(
        mcfg,
        setpoints={"coolant": SP, "air": 29.0},
        channel_temps={"radiator": ("coolant",), "intake": ("air",)},
    )
    prob = build_problem(c)
    assert prob.temps == ("coolant", "air")
    assert prob.B[0, 1] == 0.0 and prob.B[1, 0] == 0.0
    assert prob.B[0, 0] < 0 and prob.B[1, 1] < 0


# ---------------------------------------------------------------------------
# solver protocol, bumpless, offset-free, saturation
# ---------------------------------------------------------------------------


def test_mpc_solver_is_registered_and_satisfies_protocol():
    assert isinstance(MpcSolver(), Solver)
    assert isinstance(SOLVERS[SolverKind.MPC], MpcSolver)
    assert SOLVERS[SolverKind.MPC].name == "mpc"


@pytest.mark.parametrize("prev", [0.15, 0.3, 0.42, 0.8, 1.0])
@pytest.mark.parametrize("coolant", [SP - 6.0, SP, SP + 4.0, SP + 25.0])
def test_mpc_bumpless_initialise_reproduces_prev(mcfg, prev, coolant):
    solver = MpcSolver()
    req = SolverRequest(
        temps={"coolant": coolant, "air": 30.0}, prev_pwm=dict.fromkeys(mcfg.channels, prev)
    )
    integ, mem = solver.initialise(mcfg, req)
    assert set(integ) == set(mcfg.channels)
    assert all(mcfg.pwm_min - TOL <= v <= mcfg.pwm_max + TOL for v in integ.values())
    assert mem["fresh"] is True
    json.dumps(mem, allow_nan=False)
    res = solver.solve(mcfg, dataclasses.replace(req, integrator=integ, memory=mem))
    assert isinstance(res, SolverResult) and res.converged
    # The constrained first move (what the plan applies) is prev. At a rail the plant
    # may push *into* the rail -- a hot loop at pwm_max, a cold loop at pwm_min -- and
    # the reported demand then carries honest pressure past it, which ``step`` clamps
    # back to prev; that pressure is the equilibrium disturbance telling the truth,
    # not a fictitious one the estimator would have to unwind later.
    first = {ch: v for ch, v in res.pwm.items()}
    for ch in mcfg.channels:
        assert res.memory["plan"][mcfg.channels.index(ch)] == pytest.approx(prev, abs=1e-9)
        if prev >= mcfg.pwm_max and coolant > SP:
            assert first[ch] >= prev - BUMPLESS_TOL
        elif prev <= mcfg.pwm_min and coolant < SP:
            assert first[ch] <= prev + BUMPLESS_TOL
        else:
            assert first[ch] == pytest.approx(prev, abs=1e-9)
    assert res.memory["fresh"] is False and res.diagnostics["residual"] is None


def _open_loop(cfg: MpcConfig, coolant: float, pwm0: float, ticks: int) -> list[float]:
    """Radiator PWM per tick with the plant held still at ``coolant`` (fans ``pwm0`` at t=0)."""
    state = MpcState.cold()
    obs = make_obs(cfg, 0.0, coolant=coolant, pwm=dict.fromkeys(cfg.channels, pwm0))
    cmd, state = checked_step(obs, cfg, state)
    series = [cmd.pwm["radiator"]]
    for i in range(1, ticks + 1):
        obs = make_obs(cfg, i * cfg.dt, coolant=coolant, pwm=dict(cmd.pwm))
        cmd, state = checked_step(obs, cfg, state)
        series.append(cmd.pwm["radiator"])
    return series


def test_mpc_bumpless_cold_loop_clamped_up_to_pwm_min_keeps_the_equilibrium_disturbance(mcfg):
    """Regression (tests/test_mpc_fuzzy.py, ``test_random_setpoints_in_a_sane_band``).

    Fans at 0.0625 (below ``pwm_min`` but within ``d_pwm_max`` of it, so ``prev =
    obs.pwm``), loop 5 degC *below* setpoint. The first command is clamped up to
    ``pwm_min``; the bumpless ``d`` must be the equilibrium disturbance for the
    plant as observed, not some point further along the half-line on which the
    QP's first move is pinned. The old search accepted ``d = 1.56`` degC/tick (a
    ~90 degC equilibrium) there, and the next ticks *raised* the PWM to 0.19 while
    the estimator unwound that fiction: the wrong direction on a cold loop.
    """
    c = dataclasses.replace(mcfg, setpoints={"coolant": 25.0})
    solver = MpcSolver()
    req = SolverRequest(
        temps={"coolant": 20.0, "air": 10.0}, prev_pwm=dict.fromkeys(c.channels, 0.0625)
    )
    integ, mem = solver.initialise(c, req)
    prob = build_problem(c)
    d_eq = prob.d_equilibrium(np.array([20.0]), np.full(len(c.channels), 0.0625))
    assert mem["d"]["coolant"] == pytest.approx(float(d_eq[0]), abs=1e-9)
    assert integ == pytest.approx(dict.fromkeys(c.channels, c.pwm_min))
    res = solver.solve(c, dataclasses.replace(req, integrator=integ, memory=mem))
    assert res.memory["plan"][: len(c.channels)] == pytest.approx([c.pwm_min] * len(c.channels))
    assert all(v <= c.pwm_min + BUMPLESS_TOL for v in res.pwm.values())  # pressure downward
    series = _open_loop(c, coolant=20.0, pwm0=0.0625, ticks=12)
    assert series[0] == pytest.approx(c.pwm_min)
    assert all(b <= a + TOL for a, b in zip(series, series[1:], strict=False)), series
    assert max(series) == pytest.approx(c.pwm_min)


def test_mpc_bumpless_hot_loop_at_pwm_min_lands_on_the_knee(mcfg):
    """The other side: hot loop, fans at ``pwm_min``. Bumpless needs a ``d`` under which
    holding ``pwm_min`` is optimal (a fictitious self-cooling); the search must return
    the *nearest* such ``d`` -- the knee, where the first move leaves the bound with
    zero pressure -- so the estimator unwinds as little fiction as possible, and the
    PWM then rises tick after tick (never falls) on the hot loop."""
    solver = MpcSolver()
    req = SolverRequest(
        temps={"coolant": SP + 25.0, "air": 30.0},
        prev_pwm=dict.fromkeys(mcfg.channels, mcfg.pwm_min),
    )
    integ, mem = solver.initialise(mcfg, req)
    prob = build_problem(mcfg)
    d_eq = prob.d_equilibrium(np.array([SP + 25.0]), np.full(len(mcfg.channels), mcfg.pwm_min))
    d = mem["d"]["coolant"]
    assert d < float(d_eq[0])  # the model has to believe the loop cools by itself
    res = solver.solve(mcfg, dataclasses.replace(req, integrator=integ, memory=mem))
    assert res.pwm == pytest.approx(req.prev_pwm, abs=1e-9)  # at the knee: no pressure
    # one step past the knee toward d_eq the first move has already left the bound
    past = dict(mem, d={"coolant": d + 1e-3})
    res2 = solver.solve(mcfg, dataclasses.replace(req, integrator=integ, memory=past))
    assert all(v > mcfg.pwm_min + 1e-6 for v in res2.memory["plan"][: len(mcfg.channels)])
    series = _open_loop(mcfg, coolant=SP + 25.0, pwm0=mcfg.pwm_min, ticks=12)
    assert all(b >= a - TOL for a, b in zip(series, series[1:], strict=False)), series
    assert series[-1] > series[0] + 0.1


def test_mpc_bumpless_with_asymmetric_channels_is_exact_for_shared_gain(mcfg):
    solver = MpcSolver()
    req = SolverRequest(
        temps={"coolant": SP + 2.0, "air": 30.0}, prev_pwm={"radiator": 0.3, "intake": 0.9}
    )
    integ, mem = solver.initialise(mcfg, req)
    res = solver.solve(mcfg, dataclasses.replace(req, integrator=integ, memory=mem))
    assert res.pwm == pytest.approx(req.prev_pwm, abs=1e-9)


def test_mpc_bumpless_prev_outside_box_lands_on_the_bound(mcfg):
    solver = MpcSolver()
    req = SolverRequest(
        temps={"coolant": SP + 10.0, "air": 30.0}, prev_pwm={"radiator": 0.0, "intake": 0.0}
    )
    integ, mem = solver.initialise(mcfg, req)
    res = solver.solve(mcfg, dataclasses.replace(req, integrator=integ, memory=mem))
    # the plan cannot go below pwm_min; the demand may still report pressure downward
    plan0 = res.memory["plan"][: len(mcfg.channels)]
    assert plan0 == pytest.approx([mcfg.pwm_min] * len(mcfg.channels), abs=1e-9)


def test_mpc_offset_free_in_closed_loop_with_wrong_model(mcfg):
    """Model gain and time constant far from the plant: the estimator removes the offset."""
    from aqua_bridge.sim.plant import Plant, PlantParams, run_closed_loop

    c = dataclasses.replace(mcfg, mpc_gain_c_per_pwm=3.0, mpc_tau_s=400.0)
    plant = Plant(PlantParams(dt=c.dt, heat_w=100.0), initial_pwm=0.5, t_coolant=40.0)
    recs = run_closed_loop(plant, c, checked_step, 1200)
    assert all(r.cmd.mode is not Mode.FALLBACK for r in recs)
    tail = recs[-100:]
    assert abs(sum(r.obs.temps["coolant"] for r in tail) / len(tail) - SP) < 0.1
    d = [r.cmd.diagnostics["solver_diag"]["disturbance"]["coolant"] for r in tail]
    assert max(d) - min(d) < 1e-3  # the estimate has converged, not drifting


def test_mpc_reports_pressure_above_pwm_max_when_hot(mcfg):
    state = MpcState.cold()
    cmd = None
    for i in range(15):
        obs = make_obs(mcfg, i * mcfg.dt, coolant=SP + 25.0, pwm=dict.fromkeys(mcfg.channels, 0.5))
        cmd, state = checked_step(obs, mcfg, state)
    assert cmd is not None and cmd.mode is Mode.SATURATED
    assert all(cmd.diagnostics["target_pwm"][ch] > mcfg.pwm_max for ch in mcfg.channels)
    assert all(v > 0 for v in cmd.diagnostics["solver_diag"]["pressure"].values())
    assert all(v <= mcfg.pwm_max + TOL for v in state.integrator.values())


def test_mpc_iteration_cap_is_a_solver_fault(mcfg):
    """solver_max_iter=1 cannot finish a plan that needs several bound changes."""
    tight = dataclasses.replace(mcfg, solver_max_iter=1)
    state = MpcState.cold()
    faults = 0
    for i in range(6):
        obs = make_obs(
            tight, i * tight.dt, coolant=SP + 25.0, pwm=dict.fromkeys(tight.channels, 0.5)
        )
        cmd, state = checked_step(obs, tight, state)
        if cmd.diagnostics["solver_error"]:
            faults += 1
            assert cmd.mode is Mode.FALLBACK
            assert (
                "converge" in cmd.diagnostics["solver_error"]
                or "iteration" in (cmd.diagnostics["solver_error"])
            )
    assert faults > 0


def test_mpc_memory_garbage_is_tolerated(mcfg):
    solver = MpcSolver()
    req = SolverRequest(
        temps={"coolant": SP + 1.0, "air": 30.0},
        prev_pwm=dict.fromkeys(mcfg.channels, 0.5),
        integrator=dict.fromkeys(mcfg.channels, 0.5),
        memory={"d": "nope", "plan": [1, 2], "fresh": "yes", "last_temps": {"coolant": None}},
    )
    res = solver.solve(mcfg, req)
    assert res.converged
    json.dumps(res.memory, allow_nan=False)
    json.dumps(res.diagnostics, allow_nan=False)


def test_mpc_rejects_non_finite_inputs(mcfg):
    solver = MpcSolver()
    with pytest.raises(ValueError):
        solver.solve(
            mcfg,
            SolverRequest(
                temps={"coolant": math.nan, "air": 30.0}, prev_pwm={"radiator": 0.5, "intake": 0.5}
            ),
        )
    with pytest.raises(ValueError):
        solver.initialise(
            mcfg, SolverRequest(temps={"air": 30.0}, prev_pwm={"radiator": 0.5, "intake": 0.5})
        )
    with pytest.raises(ValueError):
        solver.initialise(
            mcfg,
            SolverRequest(temps={"coolant": SP, "air": 30.0}, prev_pwm={"radiator": math.inf}),
        )


def test_mpc_config_rules(mcfg):
    with pytest.raises(ConfigError):
        dataclasses.replace(mcfg, weight_pwm=0.0, weight_dpwm=0.0)
    with pytest.raises(ConfigError):
        dataclasses.replace(mcfg, weights={"coolant": 0.0})
    with pytest.raises(ConfigError):
        dataclasses.replace(mcfg, mpc_tau_s=0.0)
    with pytest.raises(ConfigError):
        dataclasses.replace(mcfg, mpc_gain_c_per_pwm=-1.0)
    with pytest.raises(ConfigError):
        dataclasses.replace(mcfg, mpc_estimator_gain=0.0)
    with pytest.raises(ConfigError):
        dataclasses.replace(mcfg, mpc_estimator_gain=1.5)
    # the same rules do not bind the PI solver
    pi = dataclasses.replace(mcfg, solver=SolverKind.PI, weight_pwm=0.0, weight_dpwm=0.0)
    assert pi.solver is SolverKind.PI


def test_mpc_step_is_deterministic_and_json_round_trips(mcfg):
    state = MpcState.cold()
    for i in range(5):
        _, state = step(make_obs(mcfg, i * mcfg.dt, coolant=SP + 2.0), mcfg, state)
    restored = MpcState.from_dict(json.loads(json.dumps(state.to_dict())))
    obs = make_obs(mcfg, 5 * mcfg.dt, coolant=SP + 1.5)
    a, sa = step(obs, mcfg, state)
    b, sb = step(obs, mcfg, restored)
    assert a.to_dict() == b.to_dict() and sa.to_dict() == sb.to_dict()
    assert a.diagnostics["solver"] == "mpc"


@pytest.mark.parametrize("horizon", [1, 40, 120])
@pytest.mark.parametrize("coolant", [SP - 8.0, SP + 3.0, SP + 30.0])
def test_mpc_horizon_extremes_stay_under_the_iteration_cap(mcfg, horizon, coolant):
    """A long horizon rails many plan entries at once; the active set must not need one
    iteration per entry (``solver_max_iter`` is 50 in the example config)."""
    c = dataclasses.replace(mcfg, horizon=horizon)
    state = MpcState.cold()
    worst = 0
    for i in range(8):
        cmd, state = checked_step(make_obs(c, i * c.dt, coolant=coolant), c, state)
        assert cmd.mode is not Mode.FALLBACK, cmd.diagnostics["solver_error"]
        worst = max(worst, cmd.diagnostics["solver_diag"]["iterations"])
    assert len(state.solver_memory["mpc"]["plan"]) == horizon * len(c.channels)
    assert worst <= c.solver_max_iter // 2, f"horizon {horizon}: {worst} active-set iterations"


def test_mpc_multi_temperature_decoupled_channels(mcfg):
    """Two setpoints, one channel each: the hot temperature's fan rises, the other holds."""
    c = dataclasses.replace(
        mcfg,
        setpoints={"coolant": SP, "air": 29.0},
        channel_temps={"radiator": ("coolant",), "intake": ("air",)},
    )
    state = MpcState.cold()
    cmds = []
    for i in range(12):
        obs = make_obs(c, i * c.dt, coolant=SP + 3.0, air=29.0, pwm=dict.fromkeys(c.channels, 0.5))
        cmd, state = checked_step(obs, c, state)
        cmds.append(cmd)
    assert cmds[-1].pwm["radiator"] > cmds[0].pwm["radiator"] + 0.05
    assert all(c_.pwm["intake"] == pytest.approx(0.5, abs=1e-9) for c_ in cmds)


def test_module_constants_are_sane():
    assert 0 < BUMPLESS_TOL < 1e-6
    assert solver_mpc._CACHE_SIZE >= 2
