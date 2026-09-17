"""The zoned thermal model (``aqua_bridge.control.thermal``, plan sections 3 and 5).

Structure and parameter table, the continuous model and its Jacobians against
finite differences, exact discretisation against a reference matrix exponential
and the Euler fallback, the identification regressors against a hand computation,
the constrained RLS (bounds, positive semi-definite covariance, no windup at
equilibrium), the status machine, determinism and JSON round trips, and the
shadow integration in ``step`` (learns and predicts, never acts). Identifiability
on the DAS truth simulator is ``tests/test_thermal_ident.py``.
"""

from __future__ import annotations

import dataclasses
import json
import math
from typing import Any

import numpy as np
import pytest
from hypothesis import given, settings
from hypothesis import strategies as st

from aqua_bridge.control import thermal
from aqua_bridge.control.mpc import step
from aqua_bridge.model import ConfigError, MpcConfig, MpcState
from aqua_bridge.sim.das import build_das_plant, run_das_closed_loop, topology_from_config
from das_fixtures import das_cfg, das_mapping
from invariants import assert_no_non_finite, checked_step


@pytest.fixture
def dcfg(das_example_cfg: MpcConfig) -> MpcConfig:
    return dataclasses.replace(das_example_cfg, model_shadow=True)


@pytest.fixture
def small() -> MpcConfig:
    """Three zones, a fan group of two channels, fan models with dead band and exponent."""
    return das_cfg(model_shadow=True, model_window_s=4.0)


# ---------------------------------------------------------------------------
# structure and parameter table
# ---------------------------------------------------------------------------


def test_structure_of_the_example_config(dcfg):
    st_ = thermal.structure(dcfg)
    assert tuple(st_.zones) == ("z0", "z1", "z2", "z3")
    z0 = st_.zones["z0"]
    by_key = {gr.key: gr for gr in z0.groups}
    # in-zone channels at 33 W/K per fan / zones listing the channel
    assert by_key["E.z0.xt1"].prior == pytest.approx(66.0) and not by_key["E.z0.xt1"].weak
    assert by_key["E.z0.qd1"].prior == pytest.approx(16.5) and not by_key["E.z0.qd1"].weak
    # channels of the coupled zone z1 reach z0 with the weak 0.1x prior
    assert by_key["E.z0.xt2"].weak and by_key["E.z0.xt2"].prior == pytest.approx(6.6)
    assert by_key["E.z0.qd2"].weak and by_key["E.z0.qd2"].prior == pytest.approx(1.65)
    assert "E.z0.xt3" not in by_key  # z2 is not coupled to z0: no entry at all
    assert z0.air_keys[-3:] == ("leak.z0", "kappa.z0.z1", "p_air.z0")
    assert st_.bays["b01"].keys == ("q_s.b01", "g0.b01", "k.b01")
    assert st_.n_states == 4 + 2 * 15
    assert st_.i_air("z1") == 1 and st_.i_drive("b01") == 4 and st_.i_sensor("b01") == 19


def test_fan_groups_share_one_coefficient_per_zone(small):
    st_ = thermal.structure(small)
    za = {gr.key: gr for gr in st_.zones["za"].groups}
    front = za["E.za.front"]
    assert front.weights == {"fa1": pytest.approx(66.0), "fa2": pytest.approx(33.0)}
    assert not front.weak and front.prior == pytest.approx(99.0)
    assert za["E.za.fb1"].weak and za["E.za.fb1"].prior == pytest.approx(3.3)
    zb = {gr.key: gr for gr in st_.zones["zb"].groups}
    assert zb["E.zb.front"].weak and zb["E.zb.front"].prior == pytest.approx(9.9)
    zc = {gr.key for gr in st_.zones["zc"].groups}
    assert zc == {"E.zc.fc1"}  # no coupling: no weak entries
    # equal phi on the group's channels gives the group that phi
    phis = {"fa1": 0.4, "fa2": 0.4, "fb1": 0.0, "fc1": 0.0}
    assert thermal._group_phi(front, phis) == pytest.approx(0.4)


def test_legacy_config_has_no_thermal_model(cfg):
    with pytest.raises(ValueError, match="zoned config"):
        thermal.structure(cfg)


def test_priors_lie_inside_their_bounds_and_project_clamps(dcfg, small):
    for c in (dcfg, small):
        prior = thermal.prior_theta(c)
        assert set(prior) == set(thermal.parameter_keys(thermal.structure(c)))
        for key, value in prior.items():
            spec = thermal.PARAMETERS[key.split(".")[0]]
            assert spec.lo <= value <= spec.hi, key
    clamped = thermal.project({"E.z0.xt1": 1e6, "leak.z0": -3.0, "k.b01": 0.2})
    assert clamped == {"E.z0.xt1": 200.0, "leak.z0": 0.0, "k.b01": 0.2}


def test_parameter_table_has_units_bounds_and_sources():
    for kind, spec in thermal.PARAMETERS.items():
        entry = spec.to_dict()
        assert set(entry) == {"unit", "lo", "hi", "prior", "identified_from"}, kind
        assert entry["lo"] < entry["hi"] and entry["unit"] and entry["identified_from"]
    json.dumps({k: v.to_dict() for k, v in thermal.PARAMETERS.items()}, allow_nan=False)


def test_sensor_lag_from_config_or_sensor_type(dcfg, small):
    assert thermal.sensor_lag_s(dcfg, "prox_b01") == 15.0  # configured
    assert thermal.sensor_lag_s(dcfg, "air_z0") == thermal.LAG_THERMISTOR_S  # quant 0.01
    assert thermal.sensor_lag_s(dcfg, "prox_b10b") == thermal.LAG_DS18B20_S  # quant 0.0625
    assert thermal.structure(dcfg).zones["z0"].tau_air == thermal.LAG_THERMISTOR_S
    assert thermal.structure(small).bays["a1"].tau_s == 15.0


# ---------------------------------------------------------------------------
# continuous model, Jacobians, discretisation
# ---------------------------------------------------------------------------


def _random_point(
    c: MpcConfig, rng: np.random.Generator, *, empty: tuple[str, ...] = ()
) -> dict[str, Any]:
    st_ = thermal.structure(c)
    prior = thermal.prior_theta(c, st_)
    theta = {}
    for key, value in prior.items():
        spec = thermal.PARAMETERS[key.split(".")[0]]
        lo, hi = max(spec.lo, 0.5 * value if value > 0 else spec.lo), spec.hi
        if key.startswith(("p_air", "q_s")):
            theta[key] = float(rng.uniform(-0.5, 0.5) * (5.0 if key.startswith("p_air") else 0.01))
        else:
            theta[key] = float(rng.uniform(lo, min(hi, 1.5 * value if value > 0 else hi)))
    occupancy = {b: ("empty" if b in empty else "occupied") for b in st_.bays}
    maps = {b: (float(rng.uniform(0.4, 0.9)), float(rng.uniform(-3, 1))) for b in st_.bays}
    params = thermal.model_params(c, theta, st=st_, occupancy=occupancy, maps=maps)
    x = np.concatenate(
        [
            rng.uniform(26, 32, st_.n_zones),
            rng.uniform(33, 48, st_.n_bays),
            rng.uniform(30, 45, st_.n_bays),
        ]
    )
    # interior of every fan curve: away from the dead band and full speed
    u = {}
    for ch in st_.channels:
        u0, _ = params.fan[ch]
        u[ch] = float(rng.uniform(u0 + 0.05, 0.95))
    return {
        "st": st_,
        "p": params,
        "x": x,
        "u": u,
        "t_in": rng.uniform(20, 26, st_.n_zones),
        "q": rng.uniform(0.0, 0.02, st_.n_bays),
        "d_air": rng.uniform(-0.01, 0.01, st_.n_zones),
    }


def _fd_jacobians(pt: dict[str, Any], eps: float = 1e-6) -> tuple[np.ndarray, np.ndarray]:
    st_, p = pt["st"], pt["p"]
    kw = {"t_in": pt["t_in"], "q": pt["q"], "d_air": pt["d_air"]}
    x0 = pt["x"]
    u0 = np.array([pt["u"][ch] for ch in st_.channels])
    n, m = len(x0), len(u0)
    a = np.zeros((n, n))
    b = np.zeros((n, m))
    for i in range(n):
        dx = np.zeros(n)
        dx[i] = eps
        a[:, i] = (
            thermal.derivatives(st_, p, x0 + dx, u0, **kw)
            - thermal.derivatives(st_, p, x0 - dx, u0, **kw)
        ) / (2 * eps)
    for i in range(m):
        du = np.zeros(m)
        du[i] = eps
        b[:, i] = (
            thermal.derivatives(st_, p, x0, u0 + du, **kw)
            - thermal.derivatives(st_, p, x0, u0 - du, **kw)
        ) / (2 * eps)
    return a, b


@pytest.mark.parametrize("which", ["example", "small", "example_empty_bays", "small_empty_bay"])
@pytest.mark.parametrize("seed", [0, 1, 2])
def test_jacobians_match_central_finite_differences(dcfg, small, which, seed):
    rng = np.random.default_rng(seed)
    if which.startswith("example"):
        empty = ("b02", "b11") if which.endswith("empty_bays") else ()
        pt = _random_point(dcfg, rng, empty=empty)
    else:
        pt = _random_point(small, rng, empty=("c1",) if which.endswith("empty_bay") else ())
    lin = thermal.jacobians(
        pt["st"], pt["p"], pt["x"], pt["u"], t_in=pt["t_in"], q=pt["q"], d_air=pt["d_air"]
    )
    a_fd, b_fd = _fd_jacobians(pt)
    np.testing.assert_allclose(lin.a, a_fd, rtol=1e-6, atol=1e-9)
    np.testing.assert_allclose(lin.b, b_fd, rtol=1e-5, atol=1e-8)
    u0 = np.array([pt["u"][ch] for ch in pt["st"].channels])
    np.testing.assert_allclose(lin.a @ pt["x"] + lin.b @ u0 + lin.c, lin.f, rtol=1e-12, atol=1e-12)


def test_more_airflow_cools_every_warm_drive(dcfg):
    pt = _random_point(dcfg, np.random.default_rng(4))
    lin = thermal.jacobians(pt["st"], pt["p"], pt["x"], pt["u"], t_in=pt["t_in"])
    st_ = pt["st"]
    for bay in st_.bays:
        row = lin.b[st_.i_drive(bay)]
        assert np.all(row <= 0.0)
        assert row.min() < 0.0  # some channel of its zone reaches it


def test_derivatives_vanish_at_the_model_equilibrium(dcfg):
    pt = _random_point(dcfg, np.random.default_rng(7))
    kw = {"t_in": pt["t_in"], "q": pt["q"], "d_air": pt["d_air"]}
    lin = thermal.jacobians(pt["st"], pt["p"], pt["x"], pt["u"], **kw)
    u0 = np.array([pt["u"][ch] for ch in pt["st"].channels])
    x_eq = -np.linalg.solve(lin.a, lin.b @ u0 + lin.c)  # f is affine in x at a fixed command
    f_eq = thermal.derivatives(pt["st"], pt["p"], x_eq, u0, **kw)
    assert np.max(np.abs(f_eq)) < 1e-9


def test_an_empty_bay_freezes_its_drive_and_its_sensor_follows_the_air(small):
    pt = _random_point(small, np.random.default_rng(3), empty=("c1",))
    st_ = pt["st"]
    lin = thermal.jacobians(st_, pt["p"], pt["x"], pt["u"], t_in=pt["t_in"])
    di, si, ai = st_.i_drive("c1"), st_.i_sensor("c1"), st_.i_air("zc")
    assert not lin.a[di].any() and not lin.b[di].any() and lin.c[di] == 0.0
    tau = pt["p"].tau_s["c1"]
    assert lin.a[si, si] == pytest.approx(-1.0 / tau) and lin.a[si, ai] == pytest.approx(1.0 / tau)
    assert lin.a[ai, di] == 0.0  # no heat from a missing drive


def _reference_zoh(a: np.ndarray, b: np.ndarray, c: np.ndarray, h: float):
    """Exact ZOH by the augmented matrix exponential, scaling and squaring, order 24."""
    n, m = a.shape[0], b.shape[1]
    big = np.zeros((n + m + 1, n + m + 1))
    big[:n, :n] = a * h
    big[:n, n : n + m] = b * h
    big[:n, -1] = c * h
    norm = np.abs(big).sum(axis=1).max()
    squarings = max(0, math.ceil(math.log2(max(norm, 1e-300) / 0.1)))
    x = big / 2.0**squarings
    e = np.eye(big.shape[0])
    for k in range(24, 0, -1):
        e = np.eye(big.shape[0]) + x @ e / k
    for _ in range(squarings):
        e = e @ e
    return e[:n, :n], e[:n, n : n + m], e[:n, -1]


@pytest.mark.parametrize("h", [5.0, 30.0])
@pytest.mark.parametrize("seed", [0, 5])
def test_eig_discretisation_matches_the_matrix_exponential(dcfg, h, seed):
    pt = _random_point(dcfg, np.random.default_rng(seed))
    lin = thermal.jacobians(pt["st"], pt["p"], pt["x"], pt["u"], t_in=pt["t_in"], q=pt["q"])
    disc = thermal.discretise(lin.a, lin.b, lin.c, h)
    assert disc.method == "eig"
    ad, bd, cd = _reference_zoh(lin.a, lin.b, lin.c, h)
    np.testing.assert_allclose(disc.ad, ad, rtol=1e-7, atol=1e-9)
    np.testing.assert_allclose(disc.bd, bd, rtol=1e-6, atol=1e-8)
    np.testing.assert_allclose(disc.cd, cd, rtol=1e-6, atol=1e-8)


def test_eig_discretisation_handles_frozen_empty_bays(small):
    pt = _random_point(small, np.random.default_rng(9), empty=("c1",))
    lin = thermal.jacobians(pt["st"], pt["p"], pt["x"], pt["u"], t_in=pt["t_in"], q=pt["q"])
    disc = thermal.discretise(lin.a, lin.b, lin.c, 30.0)
    ad, bd, cd = _reference_zoh(lin.a, lin.b, lin.c, 30.0)
    np.testing.assert_allclose(disc.ad, ad, rtol=1e-7, atol=1e-9)
    np.testing.assert_allclose(disc.bd, bd, rtol=1e-6, atol=1e-8)
    np.testing.assert_allclose(disc.cd, cd, rtol=1e-6, atol=1e-8)
    di = pt["st"].i_drive("c1")
    assert disc.ad[di, di] == pytest.approx(1.0)  # a frozen row stays put


@settings(max_examples=40, deadline=None)
@given(seed=st.integers(0, 2**31 - 1), h=st.sampled_from([2.0, 5.0, 30.0]))
def test_discretised_eigenvalues_are_real_and_inside_the_unit_interval(seed, h):
    from aqua_bridge.config import load_config
    from conftest import EXAMPLE_DAS_CONFIG

    c = load_config(EXAMPLE_DAS_CONFIG).mpc
    pt = _random_point(c, np.random.default_rng(seed))
    lin = thermal.jacobians(pt["st"], pt["p"], pt["x"], pt["u"], t_in=pt["t_in"])
    disc = thermal.discretise(lin.a, lin.b, lin.c, h)
    assert disc.method == "eig"
    vals = np.linalg.eigvals(disc.ad)
    assert np.max(np.abs(vals.imag)) < 1e-8
    assert np.all(vals.real > 0.0) and np.all(vals.real < 1.0)
    assert all(np.all(np.isfinite(arr)) for arr in (disc.ad, disc.bd, disc.cd))


def test_euler_fallback_on_complex_eigenvalues_is_stable_and_close():
    a = np.array([[-0.2, 1.0], [-1.0, -0.2]])  # not an RC network: a damped rotation
    b = np.array([[0.5], [0.0]])
    c = np.array([0.1, -0.1])
    disc = thermal.discretise(a, b, c, 1.0)
    assert disc.method == "euler" and disc.substeps >= thermal.EULER_MIN_SUBSTEPS
    ad, bd, cd = _reference_zoh(a, b, c, 1.0)
    np.testing.assert_allclose(disc.ad, ad, atol=0.12)  # four substeps: first order
    np.testing.assert_allclose(disc.bd, bd, atol=0.12)
    assert max(abs(np.linalg.eigvals(disc.ad))) < 1.0


def test_euler_fallback_substeps_grow_with_stiffness(dcfg, monkeypatch):
    pt = _random_point(dcfg, np.random.default_rng(11))
    lin = thermal.jacobians(pt["st"], pt["p"], pt["x"], pt["u"], t_in=pt["t_in"], q=pt["q"])
    monkeypatch.setattr(thermal, "EIG_COND_MAX", 0.0)  # force the fallback
    disc = thermal.discretise(lin.a, lin.b, lin.c, 30.0)
    assert disc.method == "euler"
    stiffness = np.abs(lin.a).sum(axis=1).max()
    assert disc.substeps >= 30.0 * stiffness / thermal.EULER_STEP_NORM
    ad, bd, cd = _reference_zoh(lin.a, lin.b, lin.c, 30.0)
    assert max(abs(np.linalg.eigvals(disc.ad))) < 1.0  # stable at the MPC's prediction step
    # the slow drive and sensor responses the MPC relies on stay within a few percent
    slow = [pt["st"].i_drive(b) for b in pt["st"].bays]
    np.testing.assert_allclose(disc.bd[slow], bd[slow], rtol=0.05, atol=1e-4)
    np.testing.assert_allclose(disc.cd[slow], cd[slow], rtol=0.05, atol=1e-3)


def test_discretise_rejects_a_non_positive_step():
    with pytest.raises(ValueError):
        thermal.discretise(np.eye(1) * -1.0, np.zeros((1, 1)), np.zeros(1), 0.0)


def test_phi_and_its_derivative():
    assert thermal.phi(0.05, 0.1, 1.0) == 0.0
    assert thermal.phi(1.2, 0.1, 1.0) == 1.0
    assert thermal.phi(0.55, 0.1, 1.0) == pytest.approx(0.5)
    assert thermal.phi(0.55, 0.1, 0.8) == pytest.approx(0.5**0.8)
    assert thermal.dphi(0.05, 0.1, 1.0) == 0.0 and thermal.dphi(1.0, 0.1, 1.0) == 0.0
    eps = 1e-6
    fd = (thermal.phi(0.6 + eps, 0.2, 1.1) - thermal.phi(0.6 - eps, 0.2, 1.1)) / (2 * eps)
    assert thermal.dphi(0.6, 0.2, 1.1) == pytest.approx(fd, rel=1e-6)


# ---------------------------------------------------------------------------
# identification: regressors, RLS, status machine
# ---------------------------------------------------------------------------


def _temps(c: MpcConfig, ts: float, *, ripple: float = 0.0) -> dict[str, float]:
    out = {}
    for name in c.temps:
        role = c.sensors[name].role
        base = {"inlet": 24.0, "zone_air": 26.0, "drive_proximal": 33.0, "exhaust": 27.0}[role]
        out[name] = base + ripple * math.sin(0.01 * ts + len(name))
    return out


def test_proximal_window_matches_a_hand_computation(small, monkeypatch):
    """Five samples, window 4 s: the row the RLS receives equals the weighted integrals."""
    rows: list[tuple[tuple[str, ...], np.ndarray, float]] = []
    real = thermal._rls_window

    def spy(block, spec, x, y, fan, c, **kwargs):
        rows.append((spec.keys, np.array(x), float(y)))
        return real(block, spec, x, y, fan, c, **kwargs)

    monkeypatch.setattr(thermal, "_rls_window", spy)
    c = small
    zones_ok = set(c.zone_layout.zones)
    u = dict.fromkeys(c.channels, 0.6)
    ta = [26.0, 26.1, 25.9, 26.05, 26.2]
    tp = [33.0, 33.2, 33.1, 33.4, 33.3]
    mem = None
    for k in range(5):
        temps = _temps(c, float(k))
        temps["air_a"] = temps["air_a2"] = ta[k]
        temps["prox_a1"] = temps["prox_a1b"] = tp[k]
        mem = thermal.update(mem, c, temps=temps, u=u, ts=float(k), zones_ok=zones_ok).memory
    got = [r for r in rows if r[0] == ("q_s.a1", "g0.a1", "k.a1")]
    assert len(got) == 1
    _, x, y = got[0]

    st_ = thermal.structure(c)
    params = thermal.model_params(c, st=st_)
    theta = thermal.prior_theta(c, st_)
    phis = {ch: thermal.phi(0.6, *params.fan[ch]) for ch in c.channels}
    zone = st_.zones["za"]
    qn = sum(theta[g.key] * thermal._group_phi(g, phis) for g in zone.groups) / sum(
        theta[g.key] for g in zone.groups
    )
    s_map, b_map, tau = params.s["a1"], params.b["a1"], params.tau_s["a1"]
    tau_a, cd, window = zone.tau_air, params.c_drive["a1"], c.model_window_s

    def w(t):
        a = math.pi * t / window
        return (
            math.sin(a) ** 2,
            math.pi / window * math.sin(2 * a),
            2 * (math.pi / window) ** 2 * math.cos(2 * a),
        )

    xq = xg = yy = 0.0
    for k in range(1, 5):
        (w0, dw0, ddw0), (w1, dw1, ddw1) = w(k - 1.0), w(float(k))
        drive = 0.5 * (w0 * (tp[k - 1] - ta[k - 1] - b_map) + w1 * (tp[k] - ta[k] - b_map))
        drive += 0.5 * (w0 + w1) * (tau * (tp[k] - tp[k - 1]) - tau_a * (ta[k] - ta[k - 1]))
        xq += 0.5 * (w0 + w1)
        xg -= drive / cd
        yy += -0.5 * (dw0 * tp[k - 1] + dw1 * tp[k]) + tau * 0.5 * (ddw0 * tp[k - 1] + ddw1 * tp[k])
        yy += (1 - s_map) * (
            0.5 * (dw0 * ta[k - 1] + dw1 * ta[k]) - tau_a * 0.5 * (ddw0 * ta[k - 1] + ddw1 * ta[k])
        )
    assert x == pytest.approx([xq, xg, qn * xg], rel=1e-12)
    assert y == pytest.approx(yy, rel=1e-12, abs=1e-12)


def _drive(c: MpcConfig, ticks: int, *, seed: int = 0, excite: bool = True, mem: Any = None):
    """Synthetic data: sinusoidal temperatures with noise, PRBS commands."""
    rng = np.random.default_rng(seed)
    u = dict.fromkeys(c.channels, 0.6)
    zones_ok = set(c.zone_layout.zones)
    out = None
    for k in range(ticks):
        ts = k * c.dt
        if excite and k % 7 == 0:
            u = {ch: float(rng.choice([0.3, 0.9])) for ch in c.channels}
        temps = {n: v + float(rng.normal(0, 0.05)) for n, v in _temps(c, ts, ripple=1.0).items()}
        out = thermal.update(mem, c, temps=temps, u=u, ts=ts, zones_ok=zones_ok)
        mem = out.memory
    return out


def _check_blocks(c: MpcConfig, memory: dict[str, Any]) -> None:
    d = thermal._derived(c)
    blocks = [(memory["zones"][z]["air"], d.zone_specs[z]) for z in d.st.zones]
    blocks += [(memory["bays"][b], d.bay_specs[b]) for b in d.st.bays]
    for block, spec in blocks:
        theta = np.array(block["theta"])
        assert np.all(theta >= spec.lo) and np.all(theta <= spec.hi)
        p = np.array(block["P"])
        np.testing.assert_allclose(p, p.T, atol=1e-12)
        assert np.linalg.eigvalsh(p)[0] > -1e-9
        assert np.trace(p) <= c.model_p_trace_max * (1 + 1e-9)
        assert 0.0 <= block["pe"] <= 1.0


def test_update_is_deterministic_and_round_trips_through_json(small):
    a = _drive(small, 60, seed=3)
    b = _drive(small, 60, seed=3)
    assert a.memory == b.memory and a.summary == b.summary
    text = json.dumps(a.memory, allow_nan=False)
    json.dumps(a.summary, allow_nan=False)
    rng_temps = _temps(small, 60.0, ripple=1.0)
    u = dict.fromkeys(small.channels, 0.5)
    ok = set(small.zone_layout.zones)
    direct = thermal.update(a.memory, small, temps=rng_temps, u=u, ts=60.0, zones_ok=ok)
    loaded = thermal.update(json.loads(text), small, temps=rng_temps, u=u, ts=60.0, zones_ok=ok)
    assert direct.memory == loaded.memory and direct.summary == loaded.summary


def test_update_does_not_mutate_the_memory_it_was_given(small):
    first = _drive(small, 30, seed=1)
    frozen = json.dumps(first.memory, sort_keys=True)
    temps = _temps(small, 30.0, ripple=1.0)
    ok = set(small.zone_layout.zones)
    thermal.update(first.memory, small, temps=temps, u={}, ts=30.0, zones_ok=ok)
    assert json.dumps(first.memory, sort_keys=True) == frozen


@pytest.mark.parametrize(
    "corrupt",
    [
        lambda m: None,
        lambda m: [],
        lambda m: {"v": 1},
        lambda m: {**m, "v": 99},
        lambda m: {**m, "fp": "other"},
        lambda m: {**m, "zones": {}},
        lambda m: {**m, "zones": {**m["zones"], "za": {**m["zones"]["za"], "status": "bogus"}}},
        lambda m: _set_block(m, "a1", "P", [[float("nan")] * 3] * 3),
        lambda m: _set_block(m, "a1", "theta", [1.0, 2.0]),
        lambda m: _set_block(m, "a1", "n", True),
        lambda m: _set_block(m, "a1", "acc", {"x": "no"}),
        lambda m: _set_block(m, "a1", "rel", "x"),
        lambda m: {**m, "zones": {**m["zones"], "za": {**m["zones"]["za"], "prev": 3}}},
    ],
)
def test_malformed_memory_starts_over_without_raising(small, corrupt):
    good = _drive(small, 20, seed=2).memory
    bad = corrupt(json.loads(json.dumps(good)))
    out = thermal.update(
        bad, small, temps=_temps(small, 20.0), u={}, ts=20.0, zones_ok=set(small.zone_layout.zones)
    )
    assert out.summary["status"] == "prior"
    assert all(z["windows"] == 0 for z in out.summary["zones"].values())
    json.dumps(out.memory, allow_nan=False)


def _set_block(memory: dict[str, Any], bay: str, key: str, value: Any) -> dict[str, Any]:
    memory["bays"][bay][key] = value
    return memory


@settings(max_examples=25, deadline=None)
@given(
    seed=st.integers(0, 2**31 - 1),
    offsets=st.lists(st.floats(-15.0, 15.0), min_size=11, max_size=11),
    flicker=st.floats(0.0, 0.5),
)
def test_adversarial_data_keeps_theta_in_bounds_and_the_covariance_psd(seed, offsets, flicker):
    c = das_cfg(model_shadow=True, model_window_s=6.0)
    rng = np.random.default_rng(seed)
    zones = list(c.zone_layout.zones)
    mem = None
    for k in range(150):
        ts = float(k) + (5.0 if rng.random() < 0.02 else 0.0)  # occasional gaps
        temps = {
            name: 30.0 + offsets[i] + float(rng.normal(0, 2.0)) * (k % 5 == 0)
            for i, name in enumerate(c.temps)
            if rng.random() > flicker
        }
        u = {ch: float(rng.uniform(0.0, 1.0)) for ch in c.channels}
        ok = {z for z in zones if rng.random() > flicker}
        occupancy = {b: str(rng.choice(["occupied", "empty", "unknown"])) for b in c.topology.bays}
        out = thermal.update(mem, c, temps=temps, u=u, ts=ts, zones_ok=ok, occupancy=occupancy)
        mem = out.memory
    _check_blocks(c, mem)
    json.dumps(mem, allow_nan=False)
    assert_no_non_finite(out.summary, "thermal summary")


def test_steady_data_never_winds_up_the_covariance(small):
    """At equilibrium only the constants learn: no forgetting, no excited windows."""
    ok = set(small.zone_layout.zones)
    u = dict.fromkeys(small.channels, 0.6)
    mem = None
    first_p = None
    for k in range(200):
        out = thermal.update(mem, small, temps=_temps(small, 0.0), u=u, ts=float(k), zones_ok=ok)
        mem = out.memory
        if k == 10:
            first_p = np.array(mem["bays"]["a1"]["P"])
    block = mem["bays"]["a1"]
    assert block["w"] > 10 and block["n"] == 0
    p = np.array(block["P"])
    # the fan-gain coordinate is only ever reduced, never inflated
    assert p[2, 2] <= first_p[2, 2] + 1e-12
    assert out.summary["status"] == "prior"


@pytest.mark.parametrize("ticks", [[0, 1, 2, 3, 6, 9, 10, 11, 12, 13], [0, 1, 2, 3, 5, 7, 9]])
def test_a_window_without_weight_mass_is_dropped_not_learned(ticks, monkeypatch):
    """A window as short as the rules allow (2 dt) spanned by one allowed interval (up to
    ``GAP_TICKS`` dt) has only its zero-weight endpoints: its row carries no information
    and must never reach the RLS (normalised by a weight of ~1e-32 it was a 1e36 row),
    must not raise and must leave finite JSON."""
    c = das_cfg(model_shadow=True, model_window_s=2.0)
    rows: list[tuple[str, np.ndarray, float]] = []
    real = thermal._rls_window

    def spy(block, spec, x, y, fan, cc, **kwargs):
        rows.append((spec.keys[-1], np.array(x), float(y)))
        return real(block, spec, x, y, fan, cc, **kwargs)

    monkeypatch.setattr(thermal, "_rls_window", spy)
    ok = set(c.zone_layout.zones)
    u = dict.fromkeys(c.channels, 0.6)
    mem = None
    for ts in ticks:
        temps = _temps(c, float(ts), ripple=1.0)
        with np.errstate(all="raise"):
            out = thermal.update(mem, c, temps=temps, u=u, ts=float(ts), zones_ok=ok)
        mem = out.memory
        json.dumps(mem, allow_nan=False)
        assert_no_non_finite(out.summary, "thermal summary")
    assert rows  # the well-sampled windows did close
    for key, x, y in rows:
        assert np.all(np.abs(x) < 1e3) and abs(y) < 1e3, (key, x, y)
        if key.startswith("k."):
            # the weight mass of a well-sampled window is int sin^2 = T / 2
            assert x[0] == pytest.approx(0.5 * c.model_window_s, rel=0.1), (key, x)


def test_windows_restart_when_a_zone_is_not_ok(small):
    u = dict.fromkeys(small.channels, 0.6)
    all_ok = set(small.zone_layout.zones)
    mem = None
    for k in range(3):
        mem = thermal.update(
            mem, small, temps=_temps(small, 0.0), u=u, ts=float(k), zones_ok=all_ok
        ).memory
    assert mem["bays"]["a1"]["acc"] is not None and mem["zones"]["za"]["air"]["acc"] is not None
    mem = thermal.update(
        mem, small, temps=_temps(small, 0.0), u=u, ts=3.0, zones_ok=all_ok - {"za"}
    ).memory
    assert mem["bays"]["a1"]["acc"] is None and mem["zones"]["za"]["air"]["acc"] is None
    assert mem["bays"]["b1"]["acc"] is not None  # the other zones keep their windows
    # a gap longer than GAP_TICKS restarts every window
    mem = thermal.update(mem, small, temps=_temps(small, 0.0), u=u, ts=30.0, zones_ok=all_ok).memory
    assert all(b["acc"] is None for b in mem["bays"].values())


@pytest.mark.parametrize(
    ("dropped", "restarts", "keeps"),
    [
        ("prox_a1b", {"a1", "air"}, {"a2"}),  # a redundant proximal sensor: its bay and the air
        ("air_a2", {"a1", "a2", "air"}, set()),  # a redundant zone-air sensor: the whole zone
    ],
)
def test_a_window_restarts_when_the_sensors_behind_a_mean_change(small, dropped, restarts, keeps):
    """Module docstring: a window accumulates only while every sensor its regression reads
    stays trusted. Redundant sensors disagree by their placement offsets (1 degC here), so
    averaging one tick without one of them is a step in the mean, and the sensor-lag
    correction ``tau (X_k - X_k-1)`` turns that step into a spike of ``tau`` times the offset
    inside the window. The window restarts instead."""
    ok = set(small.zone_layout.zones)
    u = dict.fromkeys(small.channels, 0.6)
    mem = None
    for k in range(3):
        temps = _temps(small, float(k))
        temps.update(prox_a1=33.0, prox_a1b=34.0, air_a=26.0, air_a2=27.0)
        if k == 2:
            del temps[dropped]
        mem = thermal.update(mem, small, temps=temps, u=u, ts=float(k), zones_ok=ok).memory

    def t0(name: str) -> float | None:
        block = mem["zones"]["za"]["air"] if name == "air" else mem["bays"][name]
        return None if block["acc"] is None else block["acc"]["t0"]

    for name in restarts:
        assert t0(name) in (None, 2.0), name  # nothing from before the change is kept
    for name in keeps:
        assert t0(name) == 0.0, name


@pytest.mark.parametrize(
    "bad", [(float("nan"), -2.1), (0.7, float("inf")), (0.0, -2.1), (-0.5, -2.1), ("x", 1.0)]
)
def test_an_unusable_sensor_map_falls_back_to_the_prior_map(small, bad):
    """A sensor map that is not a finite slope in (0, 1] and a finite offset never puts a
    non-finite value into the memory or the summary (the state must stay finite JSON):
    the bay is identified on the prior map instead."""
    ok = set(small.zone_layout.zones)
    u = dict.fromkeys(small.channels, 0.6)
    mem = ref = None
    for k in range(12):
        temps = _temps(small, float(k), ripple=1.0)
        out = thermal.update(
            mem, small, temps=temps, u=u, ts=float(k), zones_ok=ok, maps={"a1": bad}
        )
        mem = out.memory
        json.dumps(mem, allow_nan=False)
        assert_no_non_finite(out.summary, "thermal summary")
        ref = thermal.update(ref, small, temps=temps, u=u, ts=float(k), zones_ok=ok).memory
    assert mem == ref
    assert mem["bays"]["a1"]["w"] > 0


def test_an_empty_bay_learns_nothing(small):
    occupancy = {"a1": "occupied", "a2": "empty", "b1": "occupied", "c1": "empty"}
    out = _drive(small, 1, seed=0)
    mem = out.memory
    u = dict.fromkeys(small.channels, 0.6)
    for k in range(1, 30):
        mem = thermal.update(
            mem,
            small,
            temps=_temps(small, float(k), ripple=1.0),
            u=u,
            ts=float(k),
            zones_ok=set(small.zone_layout.zones),
            occupancy=occupancy,
        ).memory
    assert mem["bays"]["a2"]["w"] == 0 and mem["bays"]["a2"]["acc"] is None
    assert mem["bays"]["a1"]["w"] > 0


def test_use_rpm_takes_airflow_from_the_tachometer():
    c = das_cfg(model_shadow=True, model_use_rpm=True)
    st_ = thermal.structure(c)
    u = dict.fromkeys(c.channels, 0.6)
    phis = thermal._channel_phi(c, st_, u, {"fa1": 900.0, "fa2": None})
    assert phis["fa1"] == pytest.approx(0.5)  # 900 / rpm_max 1800
    assert phis["fa2"] == pytest.approx(thermal.phi(0.6, 0.1, 1.0))  # no reading: PWM curve
    plain = das_cfg(model_shadow=True)
    assert thermal._channel_phi(plain, st_, u, {"fa1": 900.0})["fa1"] == pytest.approx(
        thermal.phi(0.6, 0.1, 1.0)
    )


def _zone_state(**overrides: Any) -> dict[str, Any]:
    zm = {"status": "learning", "conv": None, "bad": 0, "err2": 0.01, "prev": None}
    zm.update(overrides)
    return zm


def test_status_machine_transitions(small, monkeypatch):
    c = small
    d = thermal._derived(c)
    params = thermal.model_params(c, st=d.st)
    mem = thermal.fresh_memory(c)

    def advance(zm, err, excited, converged):
        monkeypatch.setattr(thermal, "_zone_blocks_converged", lambda *a, **k: converged)
        thermal._advance_status(
            zm, mem, c, d.st, "za", err, excited, params, d.zone_specs, d.bay_specs
        )
        return zm["status"]

    zm = _zone_state(status="prior")
    assert advance(zm, 0.05, False, True) == "prior"  # nothing excited yet
    assert advance(zm, 0.05, True, False) == "learning"
    assert advance(zm, 0.05, True, False) == "learning"  # criteria not met
    assert advance(zm, 0.05, True, True) == "converged"
    assert zm["conv"] == pytest.approx(max(0.1, thermal.PRED_ERR_FLOOR_C))
    for _ in range(thermal.SUSPECT_WINDOWS - 1):
        assert advance(zm, 1.0, False, True) == "converged"
    assert advance(zm, 0.01, False, True) == "converged" and zm["bad"] == 0  # one good resets
    for _ in range(thermal.SUSPECT_WINDOWS):
        status = advance(zm, 1.0, False, True)
    assert status == "suspect"
    assert advance(zm, 1.0, False, True) == "suspect"  # stays without excitation
    assert advance(zm, 1.0, True, False) == "learning"
    err = _zone_state(status="error")
    assert advance(err, 0.05, False, True) == "error"
    assert advance(err, 0.05, True, False) == "learning"
    high = _zone_state(status="learning", err2=4.0)
    assert advance(high, 2.0, True, True) == "learning"  # prediction error above the maximum


def test_model_freeze_enters_a_converged_zone_frozen(small, monkeypatch):
    """PROJECT.md section 8 item 16: the switch freezes a converged model in place. A zone
    that meets the ``converged`` rule is entered ``frozen``, which ``update`` never learns
    on; a model the data later contradicts still goes ``suspect`` and learns again."""
    c = dataclasses.replace(small, model_freeze=True)
    d = thermal._derived(c)
    params = thermal.model_params(c, st=d.st)
    mem = thermal.fresh_memory(c)

    def advance(zm, err, excited, converged):
        monkeypatch.setattr(thermal, "_zone_blocks_converged", lambda *a, **k: converged)
        thermal._advance_status(
            zm, mem, c, d.st, "za", err, excited, params, d.zone_specs, d.bay_specs
        )
        return zm["status"]

    zm = _zone_state(status="learning")
    assert advance(zm, 0.05, True, True) == "frozen"
    assert zm["conv"] == pytest.approx(max(0.1, thermal.PRED_ERR_FLOOR_C))
    for _ in range(thermal.SUSPECT_WINDOWS):
        status = advance(zm, 1.0, False, True)
    assert status == "suspect"  # a frozen model the data contradicts is not held
    assert advance(zm, 1.0, True, False) == "learning"
    assert advance(zm, 0.05, True, True) == "frozen"


def test_model_freeze_turned_on_later_freezes_an_already_converged_zone(small, monkeypatch):
    """The case the switch is for: the owner watches ``/api/model`` until the zones read
    ``converged``, then sets ``model_freeze`` and reloads the config. Without a restart
    and without a store file, the zone must stop adapting -- so ``converged`` enters
    ``frozen`` at its next closing window, not only ``learning``."""
    d = thermal._derived(small)
    params = thermal.model_params(small, st=d.st)
    mem = thermal.fresh_memory(small)

    def advance(cfg, zm, err, excited, converged):
        monkeypatch.setattr(thermal, "_zone_blocks_converged", lambda *a, **k: converged)
        thermal._advance_status(
            zm, mem, cfg, d.st, "za", err, excited, params, d.zone_specs, d.bay_specs
        )
        return zm["status"]

    zm = _zone_state(status="learning")
    assert advance(small, zm, 0.05, True, True) == "converged"  # the switch is still off
    assert advance(small, zm, 0.05, True, True) == "converged"

    frozen_cfg = dataclasses.replace(small, model_freeze=True)
    assert advance(frozen_cfg, zm, 0.05, True, True) == "frozen"
    # and it still never holds a model the data contradicts
    for _ in range(thermal.SUSPECT_WINDOWS):
        status = advance(frozen_cfg, zm, 1.0, False, True)
    assert status == "suspect"


def test_model_freeze_holds_a_frozen_zone_theta_while_its_windows_close(small):
    """The freeze is the same hold a fresh store file gets: windows keep closing and
    scoring the prediction error, ``theta`` never moves."""
    c = dataclasses.replace(small, model_freeze=True)
    mem = thermal.fresh_memory(c, status="frozen")
    before = json.dumps({z: zm["air"]["theta"] for z, zm in mem["zones"].items()}, sort_keys=True)
    zones_ok = set(c.zone_layout.zones)
    for i in range(40):
        ts = float(i) * c.dt
        temps = _temps(c, ts, ripple=0.5)
        u = dict.fromkeys(c.channels, 0.4 if i % 8 < 4 else 0.8)
        mem = thermal.update(mem, c, temps=temps, u=u, ts=ts, zones_ok=zones_ok).memory
    after = json.dumps({z: zm["air"]["theta"] for z, zm in mem["zones"].items()}, sort_keys=True)
    assert after == before
    assert any(zm["air"]["w"] > 0 for zm in mem["zones"].values())


def test_overall_status_precedence():
    assert thermal.overall_status([]) == "prior"
    assert thermal.overall_status(["prior", "prior"]) == "prior"
    assert thermal.overall_status(["converged", "prior"]) == "learning"
    assert thermal.overall_status(["converged", "converged"]) == "converged"
    assert thermal.overall_status(["converged", "suspect", "learning"]) == "suspect"
    assert thermal.overall_status(["suspect", "error"]) == "error"


def test_fresh_memory_with_error_status_reports_it(small):
    mem = thermal.fresh_memory(small, status="error", error="FloatingPointError: x")
    summary = thermal.summary(mem, small)
    assert summary["status"] == "error" and summary["error"] == "FloatingPointError: x"
    assert summary["pred_err_c"] is None
    json.dumps(summary, allow_nan=False)


# ---------------------------------------------------------------------------
# config
# ---------------------------------------------------------------------------


@pytest.mark.parametrize(
    ("changes", "message"),
    [
        ({"model_window_s": 0.0}, "model_window_s"),
        ({"model_window_s": 601.0}, "model_window_s"),
        ({"model_shadow": True, "model_window_s": 1.5}, "2 \\* dt"),
        ({"model_lambda": 0.99}, "model_lambda"),
        ({"model_lambda": 1.01}, "model_lambda"),
        ({"model_p_trace_max": 0.0}, "model_p_trace_max"),
        ({"model_converged_rel_se": 1.0}, "model_converged_rel_se"),
        ({"model_max_pred_err_c": -1.0}, "model_max_pred_err_c"),
        ({"model_shadow": "yes"}, "model_shadow"),
        ({"model_use_rpm": 1}, "model_use_rpm"),
    ],
)
def test_model_keys_are_validated(changes, message):
    data = das_mapping()
    data.update(changes)
    with pytest.raises(ConfigError, match=message):
        MpcConfig.from_mapping(data)


def test_model_shadow_and_use_rpm_need_a_topology(cfg):
    for key in ("model_shadow", "model_use_rpm"):
        data = {**cfg.to_dict(), key: True}
        with pytest.raises(ConfigError, match="requires mpc.topology"):
            MpcConfig.from_mapping(data)
    # a long legacy dt never trips the window rule through the default
    assert dataclasses.replace(cfg, dt=300.0, confirm_s=600.0, fallback_hold_s=600.0, stuck_s=600.0)


# ---------------------------------------------------------------------------
# shadow integration in step
# ---------------------------------------------------------------------------


def _closed_loop(c: MpcConfig, ticks: int, controller=None):
    plant = build_das_plant(topology_from_config(c), preset="basic", seed=5, dt=c.dt)
    return run_das_closed_loop(
        plant, c, controller or (lambda o, cc, s: checked_step(o, cc, s)), ticks
    )


def test_without_model_shadow_nothing_runs(das_example_cfg, cfg):
    run = _closed_loop(das_example_cfg, 5)
    for rec in run.records:
        assert "thermal" not in rec.cmd.diagnostics
        assert "thermal" not in rec.state.solver_memory
    from das_fixtures import das_obs

    cmd, state = step(das_obs(das_cfg(), 0.0), das_cfg(), MpcState.cold())
    assert "thermal" not in cmd.diagnostics and "thermal" not in state.solver_memory


def test_shadow_learns_and_predicts_without_acting(das_example_cfg):
    off = _closed_loop(das_example_cfg, 400)
    on_cfg = dataclasses.replace(das_example_cfg, model_shadow=True, model_window_s=60.0)
    on = _closed_loop(on_cfg, 400)
    assert [r.cmd.pwm for r in on.records] == [r.cmd.pwm for r in off.records]
    assert [r.cmd.mode for r in on.records] == [r.cmd.mode for r in off.records]
    last = on.records[-1]
    summary = last.cmd.diagnostics["thermal"]
    assert summary["status"] in thermal.STATUSES
    assert all(z["windows"] > 0 for z in summary["zones"].values())
    assert summary["pred_err_c"] is not None and math.isfinite(summary["pred_err_c"])
    _check_blocks(on_cfg, last.state.solver_memory["thermal"])
    # the diagnostics are the pure summary of the memory in the state
    assert summary == thermal.summary(
        last.state.solver_memory["thermal"],
        on_cfg,
        occupancy={b: i["occupancy"] for b, i in last.cmd.diagnostics["bays"].items()},
    )


# ---------------------------------------------------------------------------
# why a zone is still learning (section 8 items 110, 111)
# ---------------------------------------------------------------------------


def test_pe_diag_is_the_diagonal_of_the_matrix_pe_min_is_the_eigenvalue_of(das_example_cfg):
    """``pe_min`` says *some* direction is not excited; ``pe_diag`` says which group. They
    come from one matrix, so the eigenvalue can never exceed the smallest diagonal -- a
    group whose own entry sits at or under ``PE_MIN`` shuts its zone's gate by itself."""
    on_cfg = dataclasses.replace(das_example_cfg, model_shadow=True, model_window_s=60.0)
    run = _closed_loop(on_cfg, 400)
    summary = run.records[-1].cmd.diagnostics["thermal"]
    mem = run.records[-1].state.solver_memory["thermal"]
    st = thermal.structure(on_cfg)
    for z, zone in summary["zones"].items():
        strong = [gr.key for gr in st.zones[z].groups if not gr.weak]
        assert list(zone["pe_diag"]) == strong
        diag = list(zone["pe_diag"].values())
        assert diag == thermal.pe_diagonal(mem["zones"][z]["air"])
        assert all(0.0 <= v <= 1.0 for v in diag)
        assert zone["pe_min"] <= min(diag) + 1e-9
    json.dumps(summary, allow_nan=False)


def test_a_block_with_no_windows_reads_as_no_variation_rather_than_as_nothing(das_example_cfg):
    fresh = thermal.fresh_memory(das_example_cfg)
    summary = thermal.summary(fresh, das_example_cfg)
    for zone in summary["zones"].values():
        assert set(zone["pe_diag"].values()) == {0.0}
    assert thermal.pe_diagonal(fresh["bays"]["b01"]) == [0.0]


def test_blocked_names_the_gates_the_converged_rule_still_fails(das_example_cfg):
    """One function decides and publishes, so the list cannot drift from the decision.
    A zone that sits in ``learning`` for hours says which group is not moving and which
    bay's gain has not been pinned down, instead of leaving the owner to guess."""
    on_cfg = dataclasses.replace(das_example_cfg, model_shadow=True, model_window_s=60.0)
    run = _closed_loop(on_cfg, 400)
    summary = run.records[-1].cmd.diagnostics["thermal"]
    st = thermal.structure(on_cfg)
    kinds = ("windows:", "pe:", "rel_se:", "pred_err")
    for z, zone in summary["zones"].items():
        blocked = zone["blocked"]
        assert all(any(r.startswith(k) for k in kinds) for r in blocked), blocked
        # empty exactly when the zone is converged or frozen; non-empty otherwise
        assert bool(blocked) is (zone["status"] not in ("converged", "frozen"))
        if zone["pe_min"] <= thermal.PE_MIN:
            assert f"pe:{z}" in blocked
        if zone["excited_windows"] < thermal.MIN_WINDOWS:
            assert f"windows:{z}" in blocked
        if zone["pred_err_c"] is None or zone["pred_err_c"] >= on_cfg.model_max_pred_err_c:
            assert "pred_err" in blocked
        # exactly the gains of the rule -- the zone's strong ``E`` and every occupied
        # bay's ``k`` -- and exactly the ones whose relative standard error is short
        gains = {gr.key: zone["rel_se"][gr.key] for gr in st.zones[z].groups if not gr.weak}
        gains.update({f"k.{b}": summary["bays"][b]["rel_se"][f"k.{b}"] for b in st.zones[z].bays})
        want = {
            key for key, rel in gains.items() if rel is None or rel >= on_cfg.model_converged_rel_se
        }
        assert {r.split(":", 1)[1] for r in blocked if r.startswith("rel_se:")} == want


def test_a_bay_publishes_the_absolute_standard_error_beside_the_relative_one(das_example_cfg):
    """``rel_se(k)`` is ``se / |k|``, so a bay whose airflow sensitivity is genuinely small
    fails the relative gate on a fit no worse than its neighbours' (section 8 item 111)."""
    on_cfg = dataclasses.replace(das_example_cfg, model_shadow=True, model_window_s=60.0)
    run = _closed_loop(on_cfg, 400)
    summary = run.records[-1].cmd.diagnostics["thermal"]
    for b, bay in summary["bays"].items():
        assert set(bay["se"]) == set(bay["rel_se"])
        for key, se in bay["se"].items():
            assert se >= 0.0 and math.isfinite(se)
            value = abs(bay["theta"][key])
            if bay["rel_se"][key] is None:
                assert value < 1e-9
            else:
                assert bay["rel_se"][key] == pytest.approx(se / value)
        assert bay["se"][f"k.{b}"] > 0.0


def test_a_thermal_exception_is_status_error_and_never_touches_the_command(
    das_example_cfg, monkeypatch
):
    on_cfg = dataclasses.replace(das_example_cfg, model_shadow=True)
    reference = _closed_loop(das_example_cfg, 30)
    calls = {"n": 0}
    real = thermal.update

    def flaky(*args, **kwargs):
        calls["n"] += 1
        if 10 <= calls["n"] < 20:
            raise FloatingPointError("thermal: boom")
        return real(*args, **kwargs)

    monkeypatch.setattr(thermal, "update", flaky)
    run = _closed_loop(on_cfg, 30)
    assert [r.cmd.pwm for r in run.records] == [r.cmd.pwm for r in reference.records]
    broken = run.records[12].cmd.diagnostics["thermal"]
    assert broken["status"] == "error" and "boom" in broken["error"]
    assert run.records[12].state.solver_memory["thermal"]["error"]
    assert run.records[-1].cmd.diagnostics["thermal"]["status"] in ("error", "learning", "prior")
    assert run.records[-1].cmd.diagnostics["thermal"]["error"] == broken["error"]


def test_malformed_estimator_bay_info_never_raises_out_of_step(das_example_cfg, monkeypatch):
    """Step 8b reads the estimator's per-bay occupancy, class and calibration map. Like
    everything else of the shadow update, a surprise there is ``status: error``, never a
    raise out of ``step`` and never a change of the command."""
    from aqua_bridge.control import estimator

    on_cfg = dataclasses.replace(das_example_cfg, model_shadow=True)
    reference = _closed_loop(das_example_cfg, 12)
    real = estimator.update

    def odd(*args, **kwargs):
        out = real(*args, **kwargs)
        bays = {b: dict(info) for b, info in out.bays.items()}
        bays["b01"]["serial"] = "S1"
        bays["b01"]["calibration"] = {"accepted_once": True, "slope": None, "offset_c": 0.0}
        return dataclasses.replace(out, bays=bays)

    monkeypatch.setattr(estimator, "update", odd)
    run = _closed_loop(on_cfg, 12)
    assert [r.cmd.pwm for r in run.records] == [r.cmd.pwm for r in reference.records]
    last = run.records[-1].cmd.diagnostics["thermal"]
    assert last["status"] == "error" and "TypeError" in last["error"]
