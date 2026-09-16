"""The DAS MPC (``control/solver_das.py``) and its noise model (``control/noise.py``).

Plan section 9, ``tests/test_solver_das.py``: the active-piece SQP against a
projected-gradient reference on random convex piecewise-quadratic problems, a
monotone objective, terminal rows, forbidden-band snapping and hysteresis, bumpless
initialisation, pressure at the rail, fixed channels, horizon / block extremes under
``solver_max_iter`` / ``solver_outer_max``; plus the validity gate and its PI-like DAS
fallback, ``mpc_every_ticks``, the linearisation memo, malformed memory and the
config keys.
"""

from __future__ import annotations

import copy
import dataclasses
import json
import math

import numpy as np
import pytest
from hypothesis import given, settings
from hypothesis import strategies as st

from aqua_bridge.config import load_config
from aqua_bridge.control import estimates as est
from aqua_bridge.control import noise, solver_das, thermal
from aqua_bridge.control.mpc import DAS_SOLVERS, SOLVERS, solver_for, step
from aqua_bridge.control.solver_das import (
    DIST_TAU_S,
    DasMpcSolver,
    PenaltyQp,
    check_model,
    objective,
    projected_newton,
    snap_bands,
    solve_penalty_qp,
    zoh,
)
from aqua_bridge.control.solver_mpc import MpcSolver
from aqua_bridge.control.solver_pi import PiSolver, SolverRequest
from aqua_bridge.model import ConfigError, Mode, MpcConfig, MpcState, SolverKind
from conftest import EXAMPLE_DAS_CONFIG
from das_fixtures import das_cfg, das_obs
from invariants import TOL, assert_no_non_finite, checked_step

# ---------------------------------------------------------------------------
# helpers
# ---------------------------------------------------------------------------


def base_cfg() -> MpcConfig:
    return load_config(EXAMPLE_DAS_CONFIG).mpc


def mpc_cfg(**changes) -> MpcConfig:
    """The DAS example config with the DAS MPC acting on the prior model, every shipped
    limit kept -- the validity gate's air-disturbance check included.

    The synthetic observations of this module hold every zone-air reading at 35 degC and
    every drive at its given temperature while the fans move, which no plant does: the
    estimator has to grow its per-zone air disturbance without bound to explain air that
    never answers the airflow. The check (``model_max_air_dist_c_per_min``, section 8
    item 66) does not fire on that here, because a case shorter than
    ``model_air_dist_tau_s`` has no reference level yet and the check reports nothing --
    which is the same thing a daemon reports for its first 15 minutes. The cases that do
    establish a level drive ``d_air`` themselves
    (:func:`test_the_air_disturbance_check_sees_airflow_the_model_does_not_have` and the
    two clock cases next to it), and ``tests/test_model_fallback_sim.py`` covers the check
    on the truth plant.
    """
    return dataclasses.replace(
        base_cfg(),
        **{"solver": SolverKind.MPC, "model_accept_prior": True, **changes},
    )


def prox_for(t_drive: float, t_air: float = 35.0) -> float:
    return (1.0 - est.PRIOR_BETA) * t_drive + est.PRIOR_BETA * t_air + est.PRIOR_OFFSET_C


def temps_for(cfg: MpcConfig, i: int, drive: float | dict[str, float] = 38.0) -> dict:
    """Proximal readings for a drive temperature (per bay or one for all), zone air
    flickering by two LSBs like a live thermistor, the inlet at 25 degC."""
    out: dict[str, float] = {}
    for name in cfg.temps:
        spec = cfg.sensors[name]
        if spec.role == "drive_proximal":
            t = drive if isinstance(drive, int | float) else drive.get(spec.bay, 38.0)
            out[name] = prox_for(float(t))
        elif spec.role == "zone_air":
            out[name] = 35.0 + 0.02 * (i % 2)
    return out


class Recorder:
    """A DAS MPC that records every request it is given (``step(..., solver=)``)."""

    name = "mpc"

    def __init__(self) -> None:
        self.inner = DasMpcSolver()
        self.requests: list[SolverRequest] = []
        self.results: list = []

    def initialise(self, cfg, req):
        self.requests.append(req)
        return self.inner.initialise(cfg, req)

    def solve(self, cfg, req):
        self.requests.append(req)
        res = self.inner.solve(cfg, req)
        self.results.append(res)
        return res


def run(cfg: MpcConfig, ticks: int, drive=38.0, pwm=0.5, state=None, solver=None, t0=0.0):
    """``ticks`` checked steps on synthetic observations; ``(cmd, state)`` of the last."""
    state = MpcState.cold() if state is None else state
    cmd = None
    for i in range(ticks):
        ts = t0 + i * cfg.dt
        prev = pwm if state.last_cmd is None else state.last_cmd.pwm
        obs = das_obs(cfg, ts, pwm=prev, **temps_for(cfg, i, drive))
        kwargs = {} if solver is None else {"solver": solver}
        cmd, state = checked_step(obs, cfg, state, **kwargs)
    return cmd, state


def recorded_request(cfg: MpcConfig, drive=38.0, ticks: int = 3) -> SolverRequest:
    rec = Recorder()
    run(cfg, ticks, drive=drive, solver=rec)
    return rec.requests[-1]


def exempt(plant: dict, *bays: str, reason: str = "occupancy") -> dict:
    """``plant`` with exactly ``bays`` reported ``model_exempt`` by the estimator (item 100).

    The estimator owns that verdict and recomputes it every tick; a recorded request carries
    the one it published on the tick it was recorded, so a test that replays that request on
    another clock -- or that wants one particular bay excused -- says so here rather than
    leaving a stale flag to decide.
    """
    out = copy.deepcopy(plant)
    for bay, info in out["bays"].items():
        info["model_exempt"] = bay in bays
        info["model_exempt_reason"] = reason if bay in bays else None
    return out


def full_integrator(cfg: MpcConfig, value: float = 0.5) -> dict[str, float]:
    return dict.fromkeys(cfg.channels, value)


def fresh_req(req: SolverRequest, **changes) -> SolverRequest:
    """``req`` with an empty solver memory and every channel driven (no bumpless offset)."""
    base = {"memory": {}, "integrator": dict.fromkeys(req.prev_pwm, 0.5)}
    base.update(changes)
    return dataclasses.replace(req, **base)


# ---------------------------------------------------------------------------
# the active-piece SQP
# ---------------------------------------------------------------------------


def random_qp(data, *, rows_max: int = 24) -> PenaltyQp:
    n = data.draw(st.integers(1, 10), label="n")
    r = data.draw(st.integers(0, rows_max), label="rows")
    seed = data.draw(st.integers(0, 2**31 - 1), label="seed")
    rng = np.random.default_rng(seed)
    a = rng.normal(size=(n, n))
    h0 = a @ a.T * rng.uniform(0.01, 1.0) + rng.uniform(0.01, 1.0) * np.eye(n)
    f0 = rng.normal(size=n)
    s = rng.normal(size=(r, n)) * rng.uniform(0.1, 3.0)
    y0 = rng.normal(size=r) * 2.0
    soft = rng.normal(size=r)
    hard = soft + rng.uniform(0.0, 3.0, size=r)
    rho_soft = float(rng.uniform(0.1, 20.0))
    rho_hard = rho_soft * float(rng.uniform(1.0, 50.0))
    return PenaltyQp(
        h0=h0,
        f0=f0,
        s=s,
        y0=y0,
        soft=soft,
        hard=hard,
        rho_soft=rho_soft,
        rho_hard=rho_hard,
        lo=np.full(n, 0.2),
        hi=np.full(n, 1.0),
    )


def projected_gradient_reference(qp: PenaltyQp, iters: int = 20000) -> np.ndarray:
    """FISTA with a fixed step 1/L on the true objective (reference, slow and simple)."""
    n = qp.f0.shape[0]
    lip = float(np.linalg.eigvalsh(qp.h0)[-1])
    if qp.s.size:
        lip += 2.0 * qp.rho_hard * float(np.linalg.norm(qp.s, 2) ** 2)
    x = np.clip(np.full(n, 0.6), qp.lo, qp.hi)
    z = x.copy()
    t = 1.0
    for _ in range(iters):
        x_new = np.clip(z - solver_das.gradient(qp, z) / lip, qp.lo, qp.hi)
        t_new = 0.5 * (1.0 + math.sqrt(1.0 + 4.0 * t * t))
        z = x_new + ((t - 1.0) / t_new) * (x_new - x)
        done = float(np.max(np.abs(x_new - x))) < 1e-13
        x, t = x_new, t_new
        if done and projected_gradient_norm(qp, x) < 1e-10:
            break
    return x


def projected_gradient_norm(qp: PenaltyQp, x: np.ndarray) -> float:
    g = solver_das.gradient(qp, x)
    return float(np.max(np.abs(np.clip(x - g, qp.lo, qp.hi) - x)))


@pytest.mark.fuzzy
@settings(max_examples=60, deadline=None)
@given(data=st.data())
def test_sqp_matches_a_projected_gradient_reference(data):
    qp = random_qp(data)
    res = solve_penalty_qp(qp, np.full(qp.f0.shape[0], 0.5), max_iter=500, outer_max=60)
    assert res.converged
    ref = projected_gradient_reference(qp)
    f_sqp, f_ref = objective(qp, res.x), objective(qp, ref)
    assert f_sqp <= f_ref + 1e-6 * (1.0 + abs(f_ref)), (f_sqp, f_ref)
    assert projected_gradient_norm(qp, res.x) <= 1e-6 * (1.0 + float(np.max(np.abs(qp.f0))))
    assert np.all(res.x >= qp.lo - TOL) and np.all(res.x <= qp.hi + TOL)


@pytest.mark.fuzzy
@settings(max_examples=60, deadline=None)
@given(data=st.data(), outer_max=st.integers(1, 6))
def test_sqp_objective_never_increases(data, outer_max):
    qp = random_qp(data)
    x0 = np.full(qp.f0.shape[0], data.draw(st.floats(0.2, 1.0)))
    res = solve_penalty_qp(qp, x0, max_iter=500, outer_max=outer_max)
    history = res.history
    assert history[0] == pytest.approx(objective(qp, np.clip(x0, qp.lo, qp.hi)))
    assert all(b <= a for a, b in zip(history, history[1:], strict=False))
    assert res.objective == history[-1] <= history[0]
    assert res.outer <= outer_max
    if res.stop == "unchanged":  # a consistent piece: the global minimiser (KKT)
        assert projected_gradient_norm(qp, res.x) <= 1e-6 * (1.0 + float(np.max(np.abs(qp.f0))))


@pytest.mark.fuzzy
@settings(max_examples=40, deadline=None)
@given(data=st.data())
def test_projected_newton_warm_start_never_increases_the_cost(data):
    qp = random_qp(data, rows_max=0)
    x0 = np.full(qp.f0.shape[0], 0.5)
    x, k = projected_newton(qp.h0, qp.f0, qp.lo, qp.hi, x0)
    assert 0 <= k <= solver_das.PN_ITER
    assert objective(qp, x) <= objective(qp, x0) + 1e-12
    assert np.all(x >= qp.lo) and np.all(x <= qp.hi)


def test_a_box_qp_at_the_iteration_cap_is_not_converged():
    rng = np.random.default_rng(3)
    n = 30
    a = rng.normal(size=(n, n))
    qp = PenaltyQp(
        h0=a @ a.T + np.eye(n),
        f0=rng.normal(size=n) * 50,
        s=np.zeros((0, n)),
        y0=np.zeros(0),
        soft=np.zeros(0),
        hard=np.zeros(0),
        rho_soft=1.0,
        rho_hard=1.0,
        lo=np.full(n, 0.2),
        hi=np.full(n, 1.0),
    )
    res = solve_penalty_qp(qp, np.full(n, 0.5), max_iter=1, outer_max=4)
    if res.converged:  # the warm start alone may have found the solution: then exact
        assert projected_gradient_norm(qp, res.x) <= 1e-6
    else:
        assert res.stop == "qp_cap" and res.iterations == 1


# ---------------------------------------------------------------------------
# forbidden bands
# ---------------------------------------------------------------------------


def test_a_demand_inside_a_band_snaps_to_its_upper_edge():
    bands = ((0.4, 0.5),)
    assert snap_bands(0.45, bands, None, 0.02) == (0.5, 0)
    assert snap_bands(0.4, bands, None, 0.02) == (0.5, 0)
    assert snap_bands(0.5, bands, None, 0.02) == (0.5, 0)
    assert snap_bands(0.39, bands, None, 0.02) == (0.39, None)
    assert snap_bands(0.51, bands, None, 0.02) == (0.51, None)


def test_a_held_band_is_left_downward_only_below_the_hysteresis():
    bands = ((0.4, 0.5),)
    assert snap_bands(0.385, bands, 0, 0.02) == (0.5, 0)  # inside lo - hysteresis
    assert snap_bands(0.38, bands, 0, 0.02) == (0.5, 0)  # at the boundary: still held
    assert snap_bands(0.379, bands, 0, 0.02) == (0.379, None)
    assert snap_bands(0.55, bands, 0, 0.02) == (0.55, None)  # upward: past the edge
    assert snap_bands(0.3, bands, 7, 0.02) == (0.3, None)  # a stale band index is ignored


def test_overlapping_bands_never_leave_the_output_inside_one():
    bands = ((0.3, 0.45), (0.4, 0.6), (0.7, 0.8))
    out, band = snap_bands(0.35, bands, None, 0.02)
    assert out == 0.6 and band == 1
    assert not any(lo <= out < hi for lo, hi in bands)
    assert snap_bands(0.75, bands, None, 0.02) == (0.8, 2)


# ---------------------------------------------------------------------------
# discretisation and the noise model
# ---------------------------------------------------------------------------


def test_zoh_matches_the_eigen_discretisation_and_fine_euler():
    rng = np.random.default_rng(11)
    n = 6
    cap = rng.uniform(50.0, 500.0, size=n)
    k = np.abs(rng.normal(size=(n, n)))
    k = k + k.T
    np.fill_diagonal(k, 0.0)
    a = (k - np.diag(k.sum(axis=1) + rng.uniform(0.1, 1.0, size=n))) / cap[:, None]
    b = rng.normal(size=(n, 2))
    c = rng.normal(size=n)
    ad, m_int = zoh(a, 30.0)
    ref = thermal.discretise(a, b, c, 30.0)
    assert ref.method == "eig"
    assert np.allclose(ad, ref.ad, atol=1e-10) and np.allclose(m_int @ b, ref.bd, atol=1e-9)
    assert np.allclose(m_int @ c, ref.cd, atol=1e-9)


def test_zoh_with_the_inverse_matches_the_block_exponential():
    rng = np.random.default_rng(5)
    a = -np.diag(rng.uniform(0.01, 2.0, size=5)) + 0.01 * rng.normal(size=(5, 5))
    ad, m_int = zoh(a, 30.0)
    ad2, m_int2 = zoh(a, 30.0, np.linalg.inv(a))
    assert np.allclose(ad, ad2, atol=1e-12) and np.allclose(m_int, m_int2, rtol=1e-8, atol=1e-10)


def test_the_prediction_matches_the_thermal_jacobians():
    """``build_prediction`` builds ``B`` from its affine structure; it must equal
    ``thermal.jacobians`` at the linearisation command and the exact state."""
    cfg = mpc_cfg()
    st_ = thermal.cached_structure(cfg)
    rng = np.random.default_rng(2)
    theta = thermal.prior_theta(cfg)
    theta.update({k: v * rng.uniform(0.7, 1.3) for k, v in theta.items() if v > 0})
    occupancy = {b: ("empty" if b in ("b03", "b11") else "occupied") for b in st_.bays}
    params = thermal.model_params(cfg, theta, st=st_, occupancy=occupancy)
    x_air = {z: 30.0 + rng.uniform(0, 3) for z in st_.zones}
    x_drive = {b: 38.0 + rng.uniform(0, 8) for b in st_.bays if params.occupied[b]}
    t_in = {z: 25.0 + rng.uniform(0, 1) for z in st_.zones}
    u = {ch: rng.uniform(0.25, 0.9) for ch in cfg.channels}
    pred, _ = solver_das.build_prediction(
        cfg, st_, params, x_air=x_air, x_drive=x_drive, u=u, t_in=t_in
    )
    x_full = np.zeros(st_.n_states)
    for z in st_.zones:
        x_full[st_.i_air(z)] = x_air[z]
    for b, bs in st_.bays.items():
        x_full[st_.i_drive(b)] = x_drive.get(b, x_air[bs.zone])
        x_full[st_.i_sensor(b)] = x_air[bs.zone]
    lin = thermal.jacobians(st_, params, x_full, np.array(pred.u_lin), t_in=t_in)
    idx = [st_.i_air(z) for z in pred.zones] + [st_.i_drive(b) for b in pred.drives]
    assert "b03" not in pred.drives and len(pred.drives) == 13
    assert np.allclose(pred.a, lin.a[np.ix_(idx, idx)], rtol=1e-12, atol=1e-15)
    assert np.allclose(pred.b, lin.b[idx], rtol=1e-12, atol=1e-15)
    assert all(
        abs(q - u[ch]) <= solver_das.QUANT_U
        for q, ch in zip(pred.u_lin, pred.channels, strict=True)
    )
    ad = solver_das._expm(pred.a * pred.h)
    assert np.allclose(pred.ad, ad, atol=1e-12)
    # a command within one step of the previous linearisation keeps it (no rebuild)
    nudged = {ch: v + 0.4 * solver_das.QUANT_U for ch, v in u.items()}
    last = dict(zip(pred.channels, pred.u_lin, strict=True))
    again, hit = solver_das.build_prediction(
        cfg, st_, params, x_air=x_air, x_drive=x_drive, u=nudged, t_in=t_in, u_lin=last
    )
    assert again.u_lin == pred.u_lin and hit


def test_zoh_handles_repeated_eigenvalues():
    a = np.diag([-0.1, -0.1, -0.1]) + np.array([[0, 0.01, 0], [0.01, 0, 0], [0, 0, 0]])
    ad, m_int = zoh(a, 30.0)
    x = np.array([1.0, 2.0, 3.0])
    h = 0.001
    y = x.copy()
    for _ in range(30000):
        y = y + h * (a @ y + 1.0)
    assert np.allclose(ad @ x + m_int @ np.ones(3), y, atol=1e-3)
    # thermal.discretise no longer raises here (a conjugate pair with ~0 imaginary parts)
    disc = thermal.discretise(np.diag([-0.1, -0.1]), np.zeros((2, 1)), np.zeros(2), 1.0)
    assert np.all(np.isfinite(disc.ad))


def test_noise_index_follows_the_fan_law_with_counts_and_levels():
    cfg = mpc_cfg()
    full = noise.noise_db(cfg, dict.fromkeys(cfg.channels, 1.0))
    fans = sum(cfg.fans[ch].count for ch in cfg.channels)  # every fan at 30 dB
    assert full == pytest.approx(30.0 + 10.0 * math.log10(fans))
    half = noise.noise_db(cfg, dict.fromkeys(cfg.channels, 0.5))
    assert half == pytest.approx(full + 10.0 * cfg.noise.exponent * math.log10(0.5))
    assert noise.noise_db(cfg, {}) == noise.NOISE_FLOOR_DB
    assert noise.rpm_model(cfg, "xt1", 0.1) == 0.0 and noise.rpm_model(cfg, "xt1", 1.0) == 1500.0


def test_noise_surrogate_derivatives_and_curvature_floor():
    cfg = mpc_cfg()
    n = cfg.noise.exponent
    ref = noise.reference_power(cfg)
    for u in (0.35, 0.6, 0.9):
        sur = noise.surrogate(cfg, dict.fromkeys(cfg.channels, u))
        ch = "xt1"
        eps = 1e-6

        def cost(v, ch=ch):
            model = cfg.fan_models[cfg.fans[ch].model]
            r = noise.rpm_frac(v, model.deadband)
            return cfg.fans[ch].count * 10 ** (model.noise_db_at_max / 10) * r**n / ref

        g_fd = (cost(u + eps) - cost(u - eps)) / (2 * eps)
        h_fd = (cost(u + eps) - 2 * cost(u) + cost(u - eps)) / eps**2
        assert sur.g[ch] == pytest.approx(g_fd, rel=1e-5)
        model = cfg.fan_models[cfg.fans[ch].model]
        span = 1.0 - model.deadband
        h_max = cfg.fans[ch].count * 10 ** (model.noise_db_at_max / 10) * n * (n - 1)
        h_max /= span * span * ref
        assert sur.h[ch] == pytest.approx(max(h_fd, noise.CURVATURE_FLOOR * h_max), rel=1e-3)
    top = noise.surrogate(cfg, dict.fromkeys(cfg.channels, 1.0))
    assert top.g["xt1"] > 0  # the left derivative at full speed: lowering the fan helps
    low = noise.surrogate(cfg, dict.fromkeys(cfg.channels, 0.05))
    assert low.u_now["xt1"] == cfg.pwm_min  # clamped into the box


def test_noise_diagnostics_use_the_tach_where_one_reports():
    cfg = mpc_cfg()
    prev = dict.fromkeys(cfg.channels, 0.55)
    rpm = {ch: 750.0 for ch in cfg.channels if ch != "qd4"}
    rpm["qd3"] = float("nan")
    out = noise.noise_diagnostics(cfg, prev=prev, pwm=prev, rpm=rpm)
    assert out["channels"]["xt1"] == {
        "rpm": 750.0,
        "source": "tach",
        "rpm_cmd": pytest.approx(750.0),
    }
    assert out["channels"]["qd4"]["source"] == "model"
    assert out["channels"]["qd3"]["source"] == "model"
    assert out["db_index"] == pytest.approx(out["db_index_cmd"])  # 750 rpm is the curve at 0.55


# ---------------------------------------------------------------------------
# registry, config, legacy
# ---------------------------------------------------------------------------


def test_registry_picks_the_das_mpc_only_for_drive_limit_regulation():
    assert isinstance(SOLVERS[SolverKind.MPC], MpcSolver)
    assert isinstance(DAS_SOLVERS[SolverKind.MPC], DasMpcSolver)
    assert DAS_SOLVERS[SolverKind.PI] is SOLVERS[SolverKind.PI]
    assert isinstance(solver_for(mpc_cfg()), DasMpcSolver)
    zoned_setpoints = das_cfg(solver="mpc", weight_dpwm=60.0)
    assert isinstance(solver_for(zoned_setpoints), MpcSolver)
    legacy = load_config(EXAMPLE_DAS_CONFIG.parent / "config.example.yaml").mpc
    assert isinstance(solver_for(dataclasses.replace(legacy, solver=SolverKind.MPC)), MpcSolver)


def test_das_mpc_config_rules():
    cfg = mpc_cfg()
    assert cfg.blocks() == (1, 1, 2, 4, 6, 6)
    with pytest.raises(ConfigError, match="sum to horizon"):
        dataclasses.replace(cfg, mpc_blocks=(1, 2))
    with pytest.raises(ConfigError, match="mpc_blocks entries"):
        dataclasses.replace(cfg, mpc_blocks=(0, 20))
    with pytest.raises(ConfigError, match="mpc_pred_dt_s must be >= dt"):
        dataclasses.replace(cfg, mpc_pred_dt_s=1.0)
    with pytest.raises(ConfigError, match="mpc_every_ticks"):
        dataclasses.replace(cfg, mpc_every_ticks=0)
    with pytest.raises(ConfigError, match="rho_hard"):
        dataclasses.replace(cfg, rho_hard=1.0)
    with pytest.raises(ConfigError, match="solver_outer_max"):
        dataclasses.replace(cfg, solver_outer_max=0)
    with pytest.raises(ConfigError, match="model_max_drift"):
        dataclasses.replace(cfg, model_max_drift_c_per_min=0.0)
    assert (cfg.model_return_factor, cfg.model_return_dwell_s) == (0.5, 300.0)
    assert cfg.model_drift_rate_tau_s == 120.0
    for bad in (0.0, -0.5, 1.01):
        with pytest.raises(ConfigError, match="model_return_factor"):
            dataclasses.replace(cfg, model_return_factor=bad)
    assert dataclasses.replace(cfg, model_return_factor=1.0).model_return_factor == 1.0
    with pytest.raises(ConfigError, match="model_return_dwell_s"):
        dataclasses.replace(cfg, model_return_dwell_s=-1.0)
    assert dataclasses.replace(cfg, model_return_dwell_s=0.0).model_return_dwell_s == 0.0
    for bad in (0.0, -1.0):
        with pytest.raises(ConfigError, match="model_drift_rate_tau_s"):
            dataclasses.replace(cfg, model_drift_rate_tau_s=bad)
    for key in ("model_return_factor", "model_return_dwell_s", "model_drift_rate_tau_s"):
        for bad in (float("nan"), float("inf"), "0.5", True):
            with pytest.raises(ConfigError, match=key):
                MpcConfig.from_mapping({**cfg.to_dict(), key: bad})
    with pytest.raises(ConfigError, match="weight_noise"):
        dataclasses.replace(
            cfg, weight_dpwm=0.0, noise=dataclasses.replace(cfg.noise, weight_noise=0.0)
        )
    with pytest.raises(ConfigError, match="model_accept_prior"):
        legacy = load_config(EXAMPLE_DAS_CONFIG.parent / "config.example.yaml").mpc
        dataclasses.replace(legacy, model_accept_prior=True)
    legacy = load_config(EXAMPLE_DAS_CONFIG.parent / "config.example.yaml").mpc
    assert legacy.blocks() == (1, 1, 2, 4) and legacy.horizon == 8  # inert, never invalid
    restored = MpcConfig.from_mapping(json.loads(json.dumps(cfg.to_dict())))
    assert restored == cfg


def test_legacy_step_diagnostics_have_no_noise_and_the_das_ones_do(cfg):
    from invariants import make_obs

    cmd, _ = step(make_obs(cfg, 0.0), cfg, MpcState.cold())
    assert "noise" not in cmd.diagnostics
    das = mpc_cfg()
    cmd, _ = run(das, 2)
    assert set(cmd.diagnostics["noise"]) == {"db_index", "db_index_cmd", "channels"}


def test_home_assistant_publishes_noise_db_in_das_mode_only(cfg):
    from aqua_bridge.control.intents import ControlMode
    from aqua_bridge.publishers.mqtt_ha import build_discovery_entities

    das = build_discovery_entities(
        mpc_cfg(), node_id="n", discovery_prefix="homeassistant", control_mode=ControlMode.AUTO
    )
    entity = {e.object_id: e for e in das}["noise_db"]
    assert "diagnostics.noise.db_index" in entity.payload["value_template"]
    assert entity.payload["unit_of_measurement"] == "dB"
    legacy = build_discovery_entities(
        cfg, node_id="n", discovery_prefix="homeassistant", control_mode=ControlMode.AUTO
    )
    assert "noise_db" not in {e.object_id for e in legacy}


def test_current_model_reads_status_and_theta():
    cfg = mpc_cfg(model_shadow=True)
    status, theta = thermal.current_model(None, cfg)
    assert status == "off" and theta == thermal.prior_theta(cfg)
    mem = thermal.fresh_memory(cfg, status="converged")
    mem["bays"]["b01"]["theta"][2] = 0.9  # k.b01
    status, theta = thermal.current_model(mem, cfg)
    assert status == "converged" and theta["k.b01"] == 0.9
    assert thermal.current_model({"v": 0}, cfg)[0] == "prior"  # malformed: the fresh prior


# ---------------------------------------------------------------------------
# the solver on the example config
# ---------------------------------------------------------------------------


def test_first_tick_is_bumpless_and_the_mpc_acts():
    cfg = mpc_cfg()
    cmd, state = run(cfg, 1, pwm=0.37)
    assert cmd.diagnostics["target_pwm"] == dict.fromkeys(cfg.channels, 0.37)
    model = cmd.diagnostics["solver_diag"]["model"]
    assert model["active"] == "mpc" and model["status"] == "off"
    assert set(state.integrator) == set(cfg.channels)


def test_initialise_then_solve_returns_prev_exactly_on_new_channels():
    cfg = mpc_cfg()
    req = recorded_request(cfg, drive=44.0, ticks=4)
    solver = DasMpcSolver()
    prev = {ch: 0.3 + 0.07 * i for i, ch in enumerate(cfg.channels)}
    partial = {ch: v for ch, v in req.integrator.items() if ch not in ("xt2", "qd3")}
    for integrator in ({}, partial):
        r0 = dataclasses.replace(req, prev_pwm=prev, integrator=integrator)
        init_i, init_m = solver.initialise(cfg, r0)
        merged = {**integrator, **{ch: init_i[ch] for ch in cfg.channels if ch not in integrator}}
        res = solver.solve(cfg, dataclasses.replace(r0, integrator=merged, memory=init_m))
        for ch in cfg.channels:
            if ch not in integrator:
                assert res.pwm[ch] == prev[ch], ch
        assert json.loads(json.dumps(res.memory, allow_nan=False)) == res.memory


def test_cool_drives_let_every_fan_slow_down():
    """Drives far below their targets: only the noise term acts. Its slope vanishes like
    ``r^4`` at low speed, so the fans approach ``pwm_min`` ever more slowly (a PWM of 0.3
    instead of 0.2 is inaudible beside the curve at 0.6)."""
    cfg = mpc_cfg(mpc_every_ticks=1)
    start, _ = run(cfg, 1, drive=30.0, pwm=0.6)
    cmd, _ = run(cfg, 120, drive=30.0, pwm=0.6)
    assert cmd.mode is Mode.AUTO
    assert all(cfg.pwm_min <= v <= 0.35 for v in cmd.pwm.values()), cmd.pwm
    diag = cmd.diagnostics["solver_diag"]
    assert diag["model"]["active"] == "mpc"
    assert diag["rows_over_soft"] == 0
    assert cmd.diagnostics["noise"]["db_index_cmd"] < start.diagnostics["noise"]["db_index"] - 15


def test_hot_drives_pin_the_rail_with_honest_pressure():
    cfg = mpc_cfg(mpc_every_ticks=1)
    cmd, state = run(cfg, 60, drive=60.0, pwm=0.5)
    assert cmd.mode is Mode.SATURATED
    diag = cmd.diagnostics["solver_diag"]
    assert all(v > cfg.pwm_max for v in diag["demand"].values())
    assert all(p > 0 for p in diag["pressure"].values())
    assert all(v == pytest.approx(cfg.pwm_max) for v in cmd.pwm.values())
    assert all(v <= cfg.pwm_max + TOL for v in state.integrator.values())


def test_terminal_rows_raise_the_fans_before_the_drive_heats():
    """Drives below their soft target with a heat load whose steady state is above it: the
    first rows are clear, only the terminal equilibrium rows see the violation."""
    cfg = mpc_cfg(model_max_drift_c_per_min=100.0)  # the heat load is far from equilibrium
    req = recorded_request(cfg, drive=36.0, ticks=4)
    plant = copy.deepcopy(req.plant)
    solver = DasMpcSolver()
    base = solver.solve(cfg, fresh_req(req))
    for bay in ("b01", "b02", "b03", "b04"):
        plant["bays"][bay]["q_w"] = 14.0  # steady state far above soft, T_d still at 36
    hot = solver.solve(cfg, fresh_req(req, plant=plant))
    assert all(req.estimates[b]["t"] < req.estimates[b]["soft"] for b in ("b01", "b02"))
    for ch in cfg.topology.zones["z0"].channels:
        assert hot.diagnostics["demand"][ch] > base.diagnostics["demand"][ch] + 0.1, ch
    assert hot.diagnostics["worst_bay"] in ("b01", "b02", "b03", "b04")
    assert hot.diagnostics["model"]["active"] == base.diagnostics["model"]["active"] == "mpc"


def test_fixed_channels_are_known_inputs_and_only_add_cooling():
    cfg = mpc_cfg()
    req = recorded_request(cfg, drive=45.0, ticks=4)
    solver = DasMpcSolver()
    trust = {z: z != "z3" for z in cfg.zone_layout.zones}
    estimates = {b: e for b, e in req.estimates.items() if e["zone"] != "z3"}
    results = {}
    for level in (0.5, 1.0):
        fixed = {ch: level for ch in ("xt4", "qd4")}
        r = fresh_req(req, fixed_channels=fixed, zone_trust=trust, estimates=estimates)
        integ = {ch: 0.5 for ch in cfg.channels if ch not in fixed}
        res = solver.solve(cfg, dataclasses.replace(r, integrator=integ))
        assert res.pwm["xt4"] == level and res.pwm["qd4"] == level
        assert set(res.integrator) <= set(cfg.channels) - set(fixed)
        plan = np.array(res.memory["plan"]).reshape(len(cfg.blocks()), len(cfg.channels))
        assert np.all(plan[:, cfg.channels.index("xt4")] == level)
        results[level] = res.diagnostics["demand"]
    # z2 exchanges air with z3 and shares qd3: more air through z3 never asks z2 for more
    for ch in ("xt3", "qd2", "qd3"):
        assert results[1.0][ch] <= results[0.5][ch] + 1e-6, ch


def test_a_forbidden_band_holds_the_output_at_its_upper_edge():
    cfg = mpc_cfg()
    free, _ = run(cfg, 120, drive=30.0, pwm=0.6)  # cool drives: the demand drifts to ~0.3
    assert 0.25 < free.diagnostics["solver_diag"]["demand"]["xt1"] < 0.4
    fans = dict(cfg.fans)
    fans["xt1"] = dataclasses.replace(fans["xt1"], forbidden_pwm=((0.25, 0.4),))
    banded = dataclasses.replace(cfg, fans=fans)
    cmd, state = run(banded, 120, drive=30.0, pwm=0.6)
    diag = cmd.diagnostics["solver_diag"]
    assert diag["demand"]["xt1"] < 0.4
    assert cmd.pwm["xt1"] == pytest.approx(0.4, abs=1e-4)  # up to the decaying offset
    assert diag["band"]["xt1"] == [0.25, 0.4]
    assert state.solver_memory["mpc"]["band"] == {"xt1": 0}


def test_every_n_ticks_replays_the_plan_and_a_new_fault_solves_at_once():
    cfg = mpc_cfg(mpc_every_ticks=3)
    rec = Recorder()
    state = MpcState.cold()
    solved = []
    for i in range(9):
        obs = das_obs(cfg, i * cfg.dt, pwm=0.5, **temps_for(cfg, i, 44.0))
        cmd, state = checked_step(obs, cfg, state, solver=rec)
        solved.append(bool(cmd.diagnostics["solver_diag"].get("solved")))
    # tick 0 replays initialise's solve; then every third call solves
    assert solved == [True, False, False, True, False, False, True, False, False]
    # a zone in fault changes the fixed channels: the next call solves at once
    temps = {**temps_for(cfg, 9, 44.0), "air_z3": None}
    cmd, state = checked_step(das_obs(cfg, 9 * cfg.dt, **temps), cfg, state, solver=rec)
    assert cmd.mode is Mode.DEGRADED
    assert cmd.diagnostics["solver_diag"]["solved"] is True


def test_the_linearisation_memo_is_a_pure_memo():
    cfg = mpc_cfg()
    req = fresh_req(recorded_request(cfg, drive=43.0, ticks=4))
    solver_das._DYN_CACHE.clear()
    cold = DasMpcSolver().solve(cfg, req)
    warm = DasMpcSolver().solve(cfg, req)
    assert cold.diagnostics["cache_hit"] is False and warm.diagnostics["cache_hit"] is True
    cold_d = {k: v for k, v in cold.diagnostics.items() if k not in ("cache_hit", "model")}
    warm_d = {k: v for k, v in warm.diagnostics.items() if k not in ("cache_hit", "model")}
    assert cold.pwm == warm.pwm and cold_d == warm_d
    assert cold.memory["plan"] == warm.memory["plan"]


@pytest.mark.parametrize(
    "memory",
    [
        None,
        "garbage",
        {"v": 1, "plan": [1, 2]},
        {"v": 1, "active": "nope"},
        {"v": 1, "bias": {"xt1": "x"}},
        {"v": 1, "fresh": {"pwm": {}}},
        {"v": 1, "dist": {"q": {"b01": None}, "d": {}}},
        {"v": 1, "pred": {"ts": "later"}},
        {"v": 1, "tick": -3},
    ],
)
def test_malformed_memory_starts_over_bumplessly(memory):
    cfg = mpc_cfg()
    req = recorded_request(cfg, drive=41.0, ticks=3)
    res = DasMpcSolver().solve(cfg, dataclasses.replace(req, memory=memory))
    assert res.pwm == req.prev_pwm  # a cold memory switches in bumplessly
    json.dumps(res.memory, allow_nan=False)


@pytest.mark.parametrize(
    "changes",
    [
        {"horizon": 1, "mpc_blocks": ()},
        {"horizon": 40, "mpc_blocks": ()},
        {"mpc_blocks": (1,) * 20},
        {"mpc_blocks": (20,)},
        {"solver_outer_max": 1},
        {"mpc_pred_dt_s": 5.0},
        {"weight_dpwm": 0.0},
    ],
)
@pytest.mark.parametrize("drive", [30.0, 46.0, 60.0])
def test_horizon_and_block_extremes_solve_within_the_caps(changes, drive):
    cfg = mpc_cfg(**changes)
    rec = Recorder()
    cmd, _ = run(cfg, 6, drive=drive, solver=rec)
    assert cmd.diagnostics["solver_error"] is None
    for res in rec.results:
        assert res.converged and res.iterations <= cfg.solver_max_iter
        assert res.diagnostics.get("outer", 0) <= cfg.solver_outer_max
    assert_no_non_finite(cmd.diagnostics, "diagnostics")


def test_a_box_qp_at_solver_max_iter_is_a_solver_fault(monkeypatch):
    monkeypatch.setattr(solver_das, "PN_ITER", 0)  # no warm start: the exact method must work
    cfg = mpc_cfg(solver_max_iter=1, mpc_every_ticks=1)
    state = MpcState.cold()
    faults = 0
    for i in range(6):
        obs = das_obs(cfg, i * cfg.dt, pwm=0.5, **temps_for(cfg, i, 47.0))
        cmd, state = checked_step(obs, cfg, state)
        if cmd.diagnostics["solver_error"]:
            faults += 1
            assert cmd.mode is Mode.FALLBACK
            assert "solver_max_iter" in cmd.diagnostics["solver_error"] or (
                "iteration cap" in cmd.diagnostics["solver_error"]
            )
    assert faults > 0


# ---------------------------------------------------------------------------
# validity gate and model fallback
# ---------------------------------------------------------------------------


def test_without_an_accepted_model_it_regulates_exactly_like_pi_das():
    """Status ``off`` (no shadow) without ``model_accept_prior``: the PI-like DAS fallback,
    mode ``auto``, and the fans move exactly as with ``solver: pi``."""
    mpc = dataclasses.replace(base_cfg(), solver=SolverKind.MPC)
    pi = dataclasses.replace(base_cfg(), solver=SolverKind.PI)
    s_mpc, s_pi = MpcState.cold(), MpcState.cold()
    for i in range(40):
        drive = 40.0 + 0.2 * i
        o = das_obs(mpc, i * mpc.dt, pwm=0.5, **temps_for(mpc, i, drive))
        c_mpc, s_mpc = checked_step(o, mpc, s_mpc)
        c_pi, s_pi = checked_step(o, pi, s_pi)
        assert c_mpc.pwm == pytest.approx(c_pi.pwm, abs=1e-12)
        assert c_mpc.mode is c_pi.mode
    model = c_mpc.diagnostics["solver_diag"]["model"]
    assert model == {**model, "active": "pi_das", "reason": "status:off"}


def test_a_converged_thermal_model_is_accepted_without_the_opt_in():
    cfg = dataclasses.replace(base_cfg(), solver=SolverKind.MPC, model_shadow=True)
    req = recorded_request(mpc_cfg(), drive=40.0)
    for status, active in (("converged", "mpc"), ("learning", "pi_das"), ("error", "pi_das")):
        memory = thermal.fresh_memory(cfg, status=status)
        res = DasMpcSolver().solve(cfg, fresh_req(req, thermal=memory))
        assert res.diagnostics["model"]["active"] == active, status


def test_check_model_verdicts():
    cfg = mpc_cfg()
    solver = DasMpcSolver()
    req = recorded_request(cfg, drive=40.0)
    st_ = thermal.cached_structure(cfg)
    rows = list(req.estimates)
    model = solver._model(cfg, req, rows, {"q": {}, "d": {}}, set())
    theta = dict(model.theta)
    ok = check_model(
        cfg,
        status="prior",
        theta=theta,
        pred=model.pred,
        rows=rows,
        pred_err_c=0.2,
        drift_c_per_min=0.1,
    )
    assert ok.ok and ok.reason is None
    strict = dataclasses.replace(cfg, model_accept_prior=False)
    assert (
        check_model(
            strict,
            status="prior",
            theta=theta,
            pred=model.pred,
            rows=rows,
            pred_err_c=None,
            drift_c_per_min=None,
        ).reason
        == "status:prior"
    )
    bad_theta = {**theta, "k.b01": 9.0}
    assert (
        check_model(
            cfg,
            status="converged",
            theta=bad_theta,
            pred=model.pred,
            rows=rows,
            pred_err_c=None,
            drift_c_per_min=None,
        ).reason
        == "theta_out_of_bounds"
    )
    nan_theta = {**theta, "g0.b02": float("nan")}
    assert not check_model(
        cfg,
        status="converged",
        theta=nan_theta,
        pred=model.pred,
        rows=rows,
        pred_err_c=None,
        drift_c_per_min=None,
    ).ok
    v = check_model(
        cfg,
        status="converged",
        theta=theta,
        pred=model.pred,
        rows=rows,
        pred_err_c=0.8,
        drift_c_per_min=0.1,
        relax=0.5,
    )
    assert v.reason == "pred_err:0.8"  # the hysteresis halves the limit to 0.5
    v = check_model(
        cfg,
        status="converged",
        theta=theta,
        pred=model.pred,
        rows=rows,
        pred_err_c=0.1,
        drift_c_per_min=0.6,
    )
    assert v.reason == "drift:0.6"
    assert v.reasons == ("drift:0.6",) and v.rate_only  # the entry holds it for the dwell
    air = dataclasses.replace(cfg, model_max_air_dist_c_per_min=8.0)  # the shipped limit
    v = check_model(
        air,
        status="converged",
        theta=theta,
        pred=model.pred,
        rows=rows,
        pred_err_c=0.1,
        drift_c_per_min=0.6,
        air_dist_c_per_min=99.0,
    )
    assert v.reasons == ("drift:0.6", "air_dist:99") and v.rate_only
    assert v.checks["max_air_dist_c_per_min"] == 8.0
    v = check_model(
        air,
        status="converged",
        theta=theta,
        pred=model.pred,
        rows=rows,
        pred_err_c=2.0,
        drift_c_per_min=0.6,
        air_dist_c_per_min=99.0,
    )
    assert not v.rate_only  # a failing prediction error faults the model at once
    assert (
        check_model(
            cfg,
            status="converged",
            theta=theta,
            pred=None,
            rows=rows,
            pred_err_c=None,
            drift_c_per_min=None,
            error="boom",
        ).reason
        == "model:boom"
    )
    assert st_.n_states == 34


def test_a_model_without_gain_or_a_closed_zone_is_refused():
    cfg = mpc_cfg()
    solver = DasMpcSolver()
    req = recorded_request(cfg, drive=40.0)
    rows = list(req.estimates)
    prior = thermal.prior_theta(cfg)
    # z0 without any airflow or exchange with z1: nothing the fans do reaches its drives
    cut = ("E.z0.", "kappa.z0.", "kappa.z1.z0")
    no_air = {k: (0.0 if k.startswith(cut) else v) for k, v in prior.items()}
    memory = thermal.fresh_memory(dataclasses.replace(cfg, model_shadow=True), status="converged")
    structure = thermal.cached_structure(cfg)
    for zone in ("z0", "z1"):
        memory["zones"][zone]["air"]["theta"] = [no_air[k] for k in structure.zones[zone].air_keys]
    shadow = dataclasses.replace(cfg, model_shadow=True)
    res = solver.solve(shadow, fresh_req(req, thermal=memory))
    assert res.diagnostics["model"]["active"] == "pi_das"
    assert res.diagnostics["model"]["reason"].startswith("no_gain:b0")
    closed = {
        k: (0.0 if k.split(".")[0] in ("E", "leak", "kappa") else v) for k, v in prior.items()
    }
    model = solver._model(cfg, req, rows, {"q": {}, "d": {}}, set())
    model.theta = closed
    solver._build(cfg, req, model, {"q": {}, "d": {}}, set())
    verdict = check_model(
        cfg,
        status="converged",
        theta=closed,
        pred=model.pred,
        rows=rows,
        pred_err_c=None,
        drift_c_per_min=None,
    )
    assert verdict.reason == "eigenvalues"


def test_fallback_is_bumpless_and_returns_after_the_dwell_with_hysteresis():
    cfg = mpc_cfg(mpc_every_ticks=1)
    cmd, state = run(cfg, 10, drive=43.0, pwm=0.5)
    assert cmd.diagnostics["solver_diag"]["model"]["active"] == "mpc"
    solver = DasMpcSolver()
    t0 = 10 * cfg.dt

    def tick(i, err2):
        nonlocal state
        mem = dict(state.solver_memory)
        slot = dict(mem["mpc"])
        slot["err2"] = err2
        slot["pred"] = None
        mem["mpc"] = slot
        state = dataclasses.replace(state, solver_memory=mem)
        ts = t0 + i * cfg.dt
        obs = das_obs(cfg, ts, pwm=state.last_cmd.pwm, **temps_for(cfg, i, 43.0))
        prev = dict(state.last_cmd.pwm)
        c, state = checked_step(obs, cfg, state, solver=solver)
        return c, prev

    c, prev = tick(0, 4.0)  # error 2 degC > 1: fall back on this tick, bumplessly
    assert c.diagnostics["solver_diag"]["model"]["active"] == "pi_das"
    assert c.diagnostics["solver_diag"]["model"]["reason"] == "pred_err:2"
    assert c.diagnostics["target_pwm"] == pytest.approx(prev, abs=1e-12)
    assert c.mode is Mode.AUTO
    c, _ = tick(1, 0.6**2)  # 0.6 degC: within the limit, above the hysteresis limit 0.5
    assert c.diagnostics["solver_diag"]["model"]["reason"] == "pred_err:0.6"
    back = None
    for i in range(2, 120):
        c, prev = tick(i, 0.1**2)
        if c.diagnostics["solver_diag"]["model"]["active"] == "mpc":
            back = i
            break
    assert back is not None
    elapsed = (back - 2) * cfg.dt
    assert cfg.model_return_dwell_s <= elapsed <= cfg.model_return_dwell_s + cfg.dt
    assert c.diagnostics["target_pwm"] == pytest.approx(prev, abs=1e-12)  # bumpless back


def test_a_clock_stepped_back_keeps_the_prediction_error_guard_and_the_dwell():
    """A wall clock stepped back (NTP) after the solver ran: a prediction pending in the
    old future must not silence the prediction-error guard until the clock catches up,
    and a fallback must not wait that long to count its dwell."""
    cfg = mpc_cfg(mpc_every_ticks=1, model_max_drift_c_per_min=100.0)
    req = fresh_req(recorded_request(cfg, drive=43.0, ticks=4))
    solver = DasMpcSolver()

    def ticks(memory, ts, n, warming=0.0):
        res = None
        for i in range(n):
            # the estimator excuses no bay on these clocks: the run's occupancy changes
            # are a day away from every timestamp below (item 100)
            plant = exempt(req.plant)
            for info in plant["bays"].values():
                info["t"] += warming * i  # drives the model does not see coming
            changes = {"ts": ts + i * cfg.dt, "memory": memory, "plant": plant}
            res = solver.solve(cfg, dataclasses.replace(req, **changes))
            memory = res.memory
        return res, memory

    t0 = 100_000.0
    res, memory = ticks(req.memory, t0, 20)
    assert res.diagnostics["model"]["active"] == "mpc" and memory["pred"] is not None
    back = t0 + 20 * cfg.dt - 86_400.0
    res, memory = ticks(memory, back, 40, warming=0.25)  # 1.5 degC per prediction step
    assert res.diagnostics["model"]["active"] == "pi_das"
    assert res.diagnostics["model"]["reason"].startswith("pred_err:")
    # in fallback, the clock steps back again: the dwell counts from the new clock
    later = back + 40 * cfg.dt
    memory = {**memory, "err2": 0.0, "pred": None}
    res, memory = ticks(memory, later - 86_400.0, int(2 * cfg.model_return_dwell_s / cfg.dt))
    assert res.diagnostics["model"]["active"] == "mpc"


def test_the_plant_view_carries_the_estimators_own_exemption():
    """Item 100: ``mpc.step`` hands the solver the estimator's verdict per bay, and the
    validity gate's exempt set is exactly the bays it marks -- no second rule here."""
    cfg = mpc_cfg()
    req = recorded_request(cfg, ticks=4)
    flags = {bay: info.get("model_exempt") for bay, info in req.plant["bays"].items()}
    assert flags and all(isinstance(v, bool) for v in flags.values()), flags
    assert DasMpcSolver._settling_bays(req) == {bay for bay, v in flags.items() if v}
    reasons = {info.get("model_exempt_reason") for info in req.plant["bays"].values()}
    assert reasons <= {None, "occupancy", "uncertain", "calibration"}, reasons


def test_a_swap_the_estimator_follows_as_a_jump_is_left_out_of_the_drift_check():
    cfg = mpc_cfg()
    req = recorded_request(cfg, drive=40.0, ticks=4)
    solver = DasMpcSolver()
    # the estimator reports the bay ``uncertain``: its fast-swap variance is over
    # ``bay_uncertain_var_c2``, and it is the estimator that says so (item 100)
    plant = exempt(req.plant, "b06", reason="uncertain")
    plant["bays"]["b06"]["sigma"] = 5.0  # the estimator's fast-swap variance
    plant["bays"]["b06"]["q_w"] = 60.0  # a transient far from equilibrium
    mem = {"v": 1}
    res = solver.solve(cfg, fresh_req(req, plant=plant, memory=mem))
    checks = res.diagnostics["model"]["checks"]
    assert res.diagnostics["model"]["active"] == "mpc"
    assert checks["drift_c_per_min"] is None or checks["drift_c_per_min"] < 0.5
    calm = exempt(req.plant)
    calm["bays"]["b06"]["q_w"] = 60.0
    res = solver.solve(cfg, fresh_req(req, plant=calm))
    assert res.diagnostics["model"]["reason"].startswith("drift:")
    # a fresh track has no filter state, so the drift is the plain one; a cold solver
    # starts on whichever model passes, so this one starts in the fallback
    assert res.diagnostics["model"]["active"] == "pi_das"


#: A bay's heat (W) that settles the model's drift just above ``model_max_drift_c_per_min``
#: (about 0.8 degC/min) while its one-step prediction error stays inside its own limit, so
#: the drift is the only failing check.
HOT_W = 8.0
#: A bay's heat (W) big enough that the model's rate steps far past every limit at once.
STEP_W = 300.0
#: Calm ticks before an injected fault, so the solver is on the MPC when it arrives (a
#: cold solver starts on whichever model passes, without the entry dwell).
WARM_TICKS = 2
#: One tick of observed drive rate (degC/min) injected into a failing drift every
#: :data:`DIP_EVERY` ticks: enough to put the ~0.8 degC/min drift of :data:`HOT_W` under
#: its limit for that one tick, the shape sensor noise gives a residual sitting just over
#: the limit.
DIP_C_PER_MIN = 0.6
DIP_EVERY = 5


def ticks_with(cfg: MpcConfig, req: SolverRequest, plants, t0: float = 0.0):
    """Solve once per entry of ``plants`` (a plant dict per tick), carrying the memory."""
    solver = DasMpcSolver()
    memory: object = {}
    out = []
    for i, plant in enumerate(plants):
        changes = {"ts": t0 + i * cfg.dt, "memory": memory, "plant": plant}
        res = solver.solve(cfg, dataclasses.replace(fresh_req(req), **changes))
        memory = res.memory
        out.append(res)
    return out


def fast_rate_cfg(**changes) -> MpcConfig:
    """``mpc_cfg`` with a rate filter shorter than a tick, so the drift is the plain one
    and the entry dwell is the only thing between a failing check and the fallback."""
    return mpc_cfg(mpc_every_ticks=1, model_drift_rate_tau_s=0.1, **changes)


def first_failing(results) -> int:
    """Index of the first tick whose drift is over the gate's limit."""
    for i, res in enumerate(results):
        checks = res.diagnostics["model"]["checks"]
        drift, limit = checks["drift_c_per_min"], checks["max_drift_c_per_min"]
        if drift is not None and drift > limit:
            return i
    raise AssertionError("the drift never failed")


def test_a_drift_faults_the_model_only_after_the_entry_dwell():
    """Section 8 items 64 and 65: a rate the plant itself moves is named at once and
    faults the model only when it keeps failing for ``model_drift_dwell_s``."""
    cfg = fast_rate_cfg()
    req = recorded_request(cfg, drive=40.0, ticks=4)
    plant = copy.deepcopy(req.plant)
    plant["bays"]["b06"]["q_w"] = HOT_W  # away from the model's equilibrium, drives steady
    n = int((4 * DIST_TAU_S + cfg.model_drift_dwell_s) / cfg.dt)
    warm = [copy.deepcopy(req.plant) for _ in range(WARM_TICKS)]
    results = ticks_with(cfg, req, warm + [copy.deepcopy(plant) for _ in range(n)])
    actives = [r.diagnostics["model"]["active"] for r in results]
    assert set(actives[:WARM_TICKS]) == {"mpc"}
    fails = first_failing(results)
    switch = actives.index("pi_das")
    held_s = (switch - fails) * cfg.dt
    assert cfg.model_drift_dwell_s <= held_s <= cfg.model_drift_dwell_s + cfg.dt, actives
    assert set(actives[:switch]) == {"mpc"}
    held = results[switch - 1].diagnostics["model"]
    assert held["reason"].startswith("drift:")  # named while the model still acts
    assert held["checks"]["drift_since_ts"] == pytest.approx(fails * cfg.dt)
    assert results[switch].diagnostics["model"]["reason"].startswith("drift:")
    # the drift is the only failing check: the prediction error stays inside its limit
    assert results[switch].diagnostics["model"]["checks"]["pred_err_c"] < 1.0


def test_a_drift_that_stops_failing_restarts_the_entry_dwell():
    cfg = fast_rate_cfg()
    req = recorded_request(cfg, drive=40.0, ticks=4)
    hot = copy.deepcopy(req.plant)
    hot["bays"]["b06"]["q_w"] = HOT_W
    spell = int((2 * DIST_TAU_S + 0.5 * cfg.model_drift_dwell_s) / cfg.dt)
    cool = int(4 * DIST_TAU_S / cfg.dt)  # the disturbance decays out of the drift
    plants = [copy.deepcopy(req.plant) for _ in range(WARM_TICKS)]
    plants += [copy.deepcopy(hot) for _ in range(spell)]
    plants += [copy.deepcopy(req.plant) for _ in range(cool)]
    plants += [copy.deepcopy(hot) for _ in range(spell)]
    results = ticks_with(cfg, req, plants)
    assert all(r.diagnostics["model"]["active"] == "mpc" for r in results)
    # the first spell failed, the calm block cleared it, the second spell starts over
    assert first_failing(results) < WARM_TICKS + spell
    second = (WARM_TICKS + spell + cool) * cfg.dt
    assert results[-1].diagnostics["model"]["checks"]["drift_since_ts"] >= second


def over_the_limit(results) -> list[bool]:
    """Whether each tick's entry drift is over the gate's limit."""
    out = []
    for res in results:
        checks = res.diagnostics["model"]["checks"]
        drift, limit = checks["drift_c_per_min"], checks["max_drift_c_per_min"]
        out.append(drift is not None and drift > limit)
    return out


def longest_run(flags: list[bool]) -> int:
    best = run = 0
    for flag in flags:
        run = run + 1 if flag else 0
        best = max(best, run)
    return best


def test_a_drift_that_only_dips_under_its_limit_still_faults_the_model():
    """The entry dwell leaks: a residual that sits just over its limit and dips under it
    for single ticks -- the shape a moderate parameter error takes under sensor noise --
    faults the model, where a dwell that had to be contiguous would restart for ever and
    never fault it at all (measured on the truth simulator with bay gains 1.5x)."""
    cfg = fast_rate_cfg()
    req = recorded_request(cfg, drive=40.0, ticks=4)
    bump = DIP_C_PER_MIN * cfg.dt / 60.0  # one tick of observed rate, and back the next
    n = int((4 * DIST_TAU_S + 3 * cfg.model_drift_dwell_s) / cfg.dt)
    plants = [copy.deepcopy(req.plant) for _ in range(WARM_TICKS)]
    for i in range(n):
        plant = copy.deepcopy(req.plant)
        plant["bays"]["b06"]["q_w"] = HOT_W
        if i % DIP_EVERY == DIP_EVERY - 1:
            plant["bays"]["b06"]["t"] += bump
        plants.append(plant)
    results = ticks_with(cfg, req, plants)
    actives = [r.diagnostics["model"]["active"] for r in results]
    assert "pi_das" in actives, "the flickering drift never faulted the model"
    switch = actives.index("pi_das")
    fails = first_failing(results)
    assert (switch - fails) * cfg.dt >= cfg.model_drift_dwell_s  # the dwell still held
    # and it never failed for the dwell together: a contiguous dwell would never fire
    flags = over_the_limit(results[: switch + 1])
    assert longest_run(flags) * cfg.dt < cfg.model_drift_dwell_s
    assert not all(flags[fails : switch + 1])
    assert results[switch].diagnostics["model"]["reason"].startswith("drift:")


def test_the_models_own_rate_goes_through_the_drives_rate_filter():
    """Section 8 item 64: the model's equilibrium rate answers a command or a load change
    at once while the drives' observed rate lags by ``model_drift_rate_tau_s``. Both sides
    now go through that filter, so a step in the model's rate enters the drift slowly
    instead of all at once."""
    cfg = mpc_cfg(mpc_every_ticks=1)
    req = recorded_request(cfg, drive=40.0, ticks=4)
    hot = copy.deepcopy(req.plant)
    hot["bays"]["b06"]["q_w"] = STEP_W
    n = int(cfg.model_drift_rate_tau_s / cfg.dt)
    plants = [copy.deepcopy(req.plant) for _ in range(WARM_TICKS)]
    plants += [copy.deepcopy(hot) for _ in range(n + 1)]
    checks = [r.diagnostics["model"]["checks"] for r in ticks_with(cfg, req, plants)]
    first = checks[WARM_TICKS]
    assert first["drift_abs_c_per_min"] > 2.0 * cfg.model_max_drift_c_per_min
    # the model's rate is there at once, and barely in the drift on the tick it happens
    assert first["drift_c_per_min"] < 0.2 * first["drift_abs_c_per_min"]
    last = checks[-1]  # one time constant later the filter has followed most of the way
    assert last["drift_c_per_min"] > 0.35 * last["drift_abs_c_per_min"]
    assert last["drift_c_per_min"] > 4.0 * first["drift_c_per_min"]


def air_plant(req: SolverRequest, d_air: float):
    """The recorded plant with every zone's air disturbance held at ``d_air``."""
    plant = copy.deepcopy(req.plant)
    for info in plant["zones"].values():
        info["d_air"] = d_air
    return plant


def settled_ticks(cfg: MpcConfig) -> int:
    """Ticks after which a zone's slow level has run for ``model_air_dist_tau_s`` and is a
    reference the check can measure a move against."""
    return int(cfg.model_air_dist_tau_s / cfg.dt)


def test_the_air_disturbance_check_sees_airflow_the_model_does_not_have():
    """Section 8 item 66: the fan gains ``E`` only move the air node, whose model the
    estimator's per-zone air disturbance re-balances, so the drive rows stay clean. A
    disturbance that steps away from its own slow level is the evidence that is left."""
    cfg = mpc_cfg(mpc_every_ticks=1)
    req = recorded_request(cfg, drive=40.0, ticks=4)
    settled = settled_ticks(cfg)
    n = int((cfg.model_drift_dwell_s + 4 * DIST_TAU_S) / cfg.dt)
    # 0.5 degC/s is 30 degC/min, and steady: no evidence of anything
    results = ticks_with(cfg, req, [air_plant(req, 0.5) for _ in range(settled + n)])
    air = [r.diagnostics["model"]["checks"]["air_dist_c_per_min"] for r in results]
    assert {r.diagnostics["model"]["active"] for r in results} == {"mpc"}
    # no reference level for its own time constant: nothing to report, which is not a move
    assert all(v is None for v in air[: settled - 1])
    assert all(v is not None for v in air[settled + 1 :])
    assert max(v for v in air[settled + 1 :]) < 1e-6

    plants = [air_plant(req, 0.0) for _ in range(settled + 4)]
    plants += [air_plant(req, 0.5) for _ in range(n)]
    results = ticks_with(cfg, req, plants)
    actives = [r.diagnostics["model"]["active"] for r in results]
    assert set(actives[: settled + 4]) == {"mpc"}
    switch = actives.index("pi_das")
    assert results[switch].diagnostics["model"]["reason"].startswith("air_dist:")
    assert switch * cfg.dt >= (settled + 4) * cfg.dt + cfg.model_drift_dwell_s
    checks = results[switch].diagnostics["model"]["checks"]
    assert checks["drift_c_per_min"] < checks["max_drift_c_per_min"]  # the drives look fine
    assert checks["max_air_dist_c_per_min"] == cfg.model_max_air_dist_c_per_min


def air_ticks(cfg: MpcConfig, solver, req: SolverRequest, memory, ts: float, n: int, d_air: float):
    """``n`` solves from ``ts`` with every zone's air disturbance at ``d_air``."""
    res = None
    for i in range(n):
        changes = {"ts": ts + i * cfg.dt, "memory": memory, "plant": air_plant(req, d_air)}
        res = solver.solve(cfg, dataclasses.replace(req, **changes))
        memory = res.memory
    return res, memory


def test_a_clock_stepped_back_keeps_the_air_disturbances_reference_level():
    """A wall clock stepped back (NTP, or a daemon that restarts) must not re-reference the
    air-disturbance check to a disturbance that has already moved: the levels and their
    ages are kept and only the filter refuses to advance, so a fouling caught mid-dwell is
    still caught. The entry dwell itself restarts from the new clock."""
    cfg = mpc_cfg(mpc_every_ticks=1)
    req = fresh_req(recorded_request(cfg, drive=40.0, ticks=4))
    solver = DasMpcSolver()
    settled = settled_ticks(cfg)
    t0 = 100_000.0
    res, memory = air_ticks(cfg, solver, req, req.memory, t0, settled + 1, 0.0)
    assert res.diagnostics["model"]["checks"]["air_dist_c_per_min"] == pytest.approx(0.0, abs=1e-6)

    t1 = t0 + (settled + 1) * cfg.dt
    half = int(0.5 * cfg.model_drift_dwell_s / cfg.dt)
    res, memory = air_ticks(cfg, solver, req, memory, t1, half, 0.5)
    moved = res.diagnostics["model"]["checks"]["air_dist_c_per_min"]
    assert moved > cfg.model_max_air_dist_c_per_min
    assert res.diagnostics["model"]["active"] == "mpc"  # still inside the entry dwell
    assert res.diagnostics["model"]["reason"].startswith("air_dist:")

    back = t1 + half * cfg.dt - 86_400.0
    res, memory = air_ticks(cfg, solver, req, memory, back, 1, 0.5)
    checks = res.diagnostics["model"]["checks"]
    assert checks["air_dist_c_per_min"] == pytest.approx(moved, rel=0.05)  # still the move
    assert checks["drift_since_ts"] == pytest.approx(back)  # the dwell counts from now
    n = int(cfg.model_drift_dwell_s / cfg.dt) + 1
    res, memory = air_ticks(cfg, solver, req, memory, back + cfg.dt, n, 0.5)
    assert res.diagnostics["model"]["active"] == "pi_das"
    assert res.diagnostics["model"]["reason"].startswith("air_dist:")


def test_a_gap_leaves_the_air_disturbance_check_without_a_reference_level():
    """After a gap the slow level cannot bridge (a daemon an hour offline) the check has no
    reference: it reports nothing until the level has run again for its own time constant,
    rather than 'no move' against whatever the disturbance happens to be now."""
    cfg = mpc_cfg(mpc_every_ticks=1)
    req = fresh_req(recorded_request(cfg, drive=40.0, ticks=4))
    solver = DasMpcSolver()
    settled = settled_ticks(cfg)
    t0 = 100_000.0
    res, memory = air_ticks(cfg, solver, req, req.memory, t0, settled + 1, 0.0)
    assert res.diagnostics["model"]["checks"]["air_dist_c_per_min"] == pytest.approx(0.0, abs=1e-6)

    after = t0 + (settled + 1) * cfg.dt + 3601.0
    res, memory = air_ticks(cfg, solver, req, memory, after, 1, 0.5)
    assert res.diagnostics["model"]["checks"]["air_dist_c_per_min"] is None
    assert res.diagnostics["model"]["active"] == "mpc"
    res, memory = air_ticks(cfg, solver, req, memory, after + cfg.dt, settled, 0.5)
    # the level is a reference again, and the disturbance it re-started from is steady
    assert res.diagnostics["model"]["checks"]["air_dist_c_per_min"] == pytest.approx(0.0, abs=1e-6)
    assert res.diagnostics["model"]["active"] == "mpc"


def test_the_prediction_error_guard_skips_a_bay_whose_zone_is_in_fault():
    """Section 8 item 11: a drive the solver has no row for is not evidence about the
    model -- the guard neither predicts it nor scores it."""
    cfg = mpc_cfg(mpc_every_ticks=1)
    req = recorded_request(cfg, drive=40.0, ticks=4)
    zone = "z3"
    bays = [b for b, spec in cfg.topology.bays.items() if spec.zone == zone]
    later = copy.deepcopy(req.plant)
    for bay in bays:
        later["bays"][bay]["t"] += 20.0  # a drive the model never predicted
    step_ts = cfg.mpc_pred_dt_s

    def run_two(trusted: bool):
        solver = DasMpcSolver()
        first = solver.solve(cfg, fresh_req(req, ts=0.0))
        assert set(bays) <= set(first.memory["pred"]["t"]) if trusted else True
        changes: dict = {"ts": step_ts, "memory": first.memory, "plant": later}
        if not trusted:
            changes["zone_trust"] = {z: z != zone for z in cfg.zone_layout.zones}
            changes["estimates"] = {b: e for b, e in req.estimates.items() if e["zone"] != zone}
        return solver.solve(cfg, dataclasses.replace(fresh_req(req), **changes))

    scored = run_two(True)
    assert scored.diagnostics["pred_err_c"] > 1.0
    skipped = run_two(False)
    assert skipped.diagnostics["pred_err_c"] is None or skipped.diagnostics["pred_err_c"] < 1.0


# ---------------------------------------------------------------------------
# random lies through step with the DAS MPC
# ---------------------------------------------------------------------------


@pytest.mark.fuzzy
@settings(max_examples=15, deadline=None)
@given(data=st.data())
def test_random_lies_hold_invariants_with_the_das_mpc(data):
    cfg = mpc_cfg()
    n = data.draw(st.integers(3, 20))
    drive = data.draw(st.floats(28.0, 62.0))
    state = MpcState.cold()
    ts = 0.0
    for i in range(n):
        drive += data.draw(st.floats(-1.0, 1.0))
        prev = 0.5 if state.last_cmd is None else state.last_cmd.pwm
        temps = temps_for(cfg, i, drive)
        kind = data.draw(st.sampled_from(["none", "air", "prox", "nan", "spike", "ts"]))
        if kind == "air":
            temps[f"air_z{data.draw(st.integers(0, 3))}"] = None
        elif kind == "prox":
            temps[data.draw(st.sampled_from([t for t in cfg.temps if "prox" in t]))] = None
        elif kind == "nan":
            temps["inlet_a"] = float("nan")
        elif kind == "spike":
            temps["air_z1"] = 125.0
        obs = das_obs(cfg, ts - (cfg.dt if kind == "ts" else 0.0), pwm=prev, **temps)
        cmd, state = checked_step(obs, cfg, state)
        ts += cfg.dt
        if cmd.mode is Mode.FALLBACK:
            assert set(cmd.diagnostics["zones_in_fault"]) == set(cfg.zone_layout.zones)


def test_pi_solver_stays_the_pi_like_das_form_in_das_mode():
    cfg = dataclasses.replace(base_cfg(), solver=SolverKind.PI)
    assert isinstance(solver_for(cfg), PiSolver)
    cmd, _ = run(cfg, 3)
    assert cmd.diagnostics["solver_diag"]["form"] == "margin_deficit"
