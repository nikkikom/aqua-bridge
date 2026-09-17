"""Drive temperature estimator (``aqua_bridge.control.estimator``, plan sections 2 and 6).

Config section, numerics (transition, Joseph form), the filter against the DAS
truth simulator (:mod:`aqua_bridge.sim.das`) with and without SMART, SMART
calibration (convergence, windup, serial keying, expiry), the occupancy
machine, determinism and the integration in ``mpc.step``.
"""

from __future__ import annotations

import copy
import dataclasses
import json
import math
import random
from typing import Any

import numpy as np
import pytest
from hypothesis import given, settings
from hypothesis import strategies as st

from aqua_bridge.config import load_config
from aqua_bridge.control import estimates as est
from aqua_bridge.control import estimator as E
from aqua_bridge.control import zones
from aqua_bridge.control.mpc import step
from aqua_bridge.model import (
    DAS_SECTIONS,
    ESTIMATOR_DEFAULTS,
    ConfigError,
    EstimatorSpec,
    FaultReason,
    Mode,
    MpcConfig,
    MpcState,
)
from aqua_bridge.sim.das import SENSOR_TYPES, DasPlant, build_das_plant, topology_from_config
from conftest import EXAMPLE_DAS_CONFIG
from das_fixtures import PROX_C, SP, das_cfg, das_mapping, das_obs, default_temps
from invariants import checked_step

# ---------------------------------------------------------------------------
# helpers
# ---------------------------------------------------------------------------

#: The uncalibrated sigma floor as the config model defaults it (``estimator`` item 71).
FLOOR_C = ESTIMATOR_DEFAULTS["sigma_uncalibrated_c"]


def lcfg(**estimator: Any) -> MpcConfig:
    """The small zoned fixture without setpoints (limit regulation), estimator overrides."""
    return das_cfg(setpoints={}, **({"estimator": estimator} if estimator else {}))


def trusted(cfg: MpcConfig, **overrides: float | None) -> dict[str, float]:
    values = {k: v for k, v in default_temps(cfg).items() if v is not None}
    values.update(overrides)
    return {k: float(v) for k, v in values.items() if v is not None}


def tick(cfg: MpcConfig, mem, ts: float, *, u: float = 0.5, smart=None, calibration=None, **temps):
    return E.update(
        mem,
        cfg,
        temps=trusted(cfg, **temps),
        u=dict.fromkeys(cfg.channels, u),
        ts=ts,
        smart=smart,
        calibration=calibration,
    )


def run_ticks(cfg: MpcConfig, n: int, *, mem=None, t0: float = 0.0, **kw):
    ups = []
    for i in range(n):
        up = tick(cfg, mem, t0 + i * cfg.dt, **kw)
        mem = up.memory
        ups.append(up)
    return ups


def example_cfg(**bay_serials: str) -> MpcConfig:
    base = load_config(EXAMPLE_DAS_CONFIG).mpc
    if not bay_serials:
        return base
    m = base.to_dict()
    for bay, serial in bay_serials.items():
        m["topology"]["bays"][bay]["serial"] = serial
    return MpcConfig.from_mapping(m)


def truth_plant(cfg: MpcConfig, *, prior_placement: bool = False, **kw: Any) -> DasPlant:
    """The DAS truth plant for ``cfg`` with realistic sensor noise and a serial per drive.

    ``prior_placement`` puts every proximal sensor where the prior map assumes it
    (placement offset ``-2.1 degC``), so the uncalibrated map has no bias.
    """
    topo = topology_from_config(cfg)
    for i, bay in enumerate(topo["bays"]):
        topo["bays"][bay]["serial"] = f"SN{i + 1:04d}"
    for entry in topo["sensors"].values():
        entry["noise_sigma_c"] = SENSOR_TYPES[entry["type"]].noise_sigma_c
        if prior_placement and entry["role"] == "drive_proximal":
            entry["offset_c"] = est.PRIOR_OFFSET_C
    kw.setdefault("initial_pwm", 0.5)
    return build_das_plant(topo, dt=cfg.dt, **kw)


def run_truth(cfg: MpcConfig, plant: DasPlant, ticks: int, *, smart: bool, mem=None, on_tick=None):
    u = dict.fromkeys(cfg.channels, 0.5)
    up = None
    for i in range(ticks):
        if i % 120 == 0:  # the controller would move the fans; so do we
            u = dict.fromkeys(cfg.channels, 0.35 + 0.1 * ((i // 120) % 3))
        obs = plant.observe()
        temps = {k: v for k, v in obs.temps.items() if v is not None}
        up = E.update(
            mem, cfg, temps=temps, u=u, ts=obs.ts, smart=plant.observe_smart() if smart else None
        )
        mem = up.memory
        if on_tick is not None:
            on_tick(i, plant, up)
        plant.apply(u)
        plant.advance()
    return up


# ---------------------------------------------------------------------------
# config
# ---------------------------------------------------------------------------


def test_estimator_section_defaults_with_topology_and_absent_in_legacy(cfg):
    assert "estimator" in DAS_SECTIONS
    assert cfg.estimator is None
    dcfg = das_cfg()
    assert dcfg.estimator == EstimatorSpec() and dcfg.estimator.to_dict() == ESTIMATOR_DEFAULTS
    assert load_config(EXAMPLE_DAS_CONFIG).mpc.estimator == EstimatorSpec()


def test_estimator_section_parses_and_round_trips():
    m = das_mapping()
    m["estimator"] = {"k_sigma": 1.5, "calibration_max_age_days": 7}
    c = MpcConfig.from_mapping(m)
    assert c.estimator.k_sigma == 1.5 and c.estimator.calibration_max_age_days == 7.0
    assert c.estimator.associate_window_s == 3600.0
    assert MpcConfig.from_mapping(json.loads(json.dumps(c.to_dict()))) == c


@pytest.mark.parametrize(
    "section",
    [
        {"k_sigma": 4.5},
        {"k_sigma": -0.1},
        {"q_heat": 0.0},
        {"q_offset": 0.0},
        {"sensor_noise_c": -1.0},
        {"proximal_offset_c": -0.1},
        {"air_blind_fault_s": -1.0},
        {"smart_max_age_s": 0.5},  # < dt
        {"occupied_dT_c": 0.5},  # not above empty_dT_c
        {"empty_dT_c": 0.0},
        {"empty_confirm_s": 1.0},  # < 2 dt
        {"bay_settle_s": -1.0},
        {"bay_settle_max_s": -1.0},
        {"bay_settle_max_s": 599.0},  # below bay_settle_s: a window it could not pay for
        {"calibration_max_age_days": 0.0},
        {"associate_window_s": 599.0},
        {"associate_min_corr": 1.0},
        {"associate_margin": 0.0},
        {"sigma_uncalibrated_c": 0.0},
        {"p0_t_air": 0.0},
        {"p0_d_air": -1e-6},
        {"p0_t_drive": 0.0},
        {"p0_t_sensor": 0.0},
        {"p0_heat": 0.0},
        {"reset_drive_var": 0.0},
        {"jump_min_c": 0.0},
        {"jump_sigmas": -1.0},
        {"occupancy_hold_s": -1.0},
        {"occupancy_hold_s": 301.0},  # above smart_max_age_s
        {"associate_drop_checks": 0},
        {"associate_drop_checks": 2.5},  # a count of evaluations, not a fraction
        {"associate_drop_corr": 0.0},
        {"associate_drop_corr": 0.8},  # not below associate_min_corr
        {"k_sigma": "wide"},
        {"window": 1},
        [],
    ],
)
def test_estimator_section_rejects_bad_values(section):
    m = das_mapping()
    m["estimator"] = section
    with pytest.raises(ConfigError):
        MpcConfig.from_mapping(m)


def test_the_filter_reads_its_tunables_from_the_config(cfg):
    """Item 71: no estimator tunable is a module constant any more."""
    assert not hasattr(E, "RESET_DRIVE_VAR") and not hasattr(E, "JUMP_MIN_C")
    assert not hasattr(E, "JUMP_SIGMAS") and not hasattr(E, "P0_T_AIR")
    assert not hasattr(est, "SIGMA_UNCALIBRATED_C")
    wide = lcfg(p0_t_air=4.0, p0_t_drive=9.0, p0_t_sensor=1.0, p0_heat=1e-3, p0_d_air=1e-3)
    plain = lcfg()
    p_wide = np.array(tick(wide, None, 0.0).memory["zones"]["za"]["P"])
    p_plain = np.array(tick(plain, None, 0.0).memory["zones"]["za"]["P"])
    assert np.diag(p_wide)[0] > np.diag(p_plain)[0]  # T_a starts wider
    assert (
        tick(wide, None, 0.0).estimates["a1"]["sigma"]
        > tick(plain, None, 0.0).estimates["a1"]["sigma"]
    )
    # the uncalibrated floor is the config's, not a literal
    high = lcfg(sigma_uncalibrated_c=3.0)
    assert tick(high, None, 0.0).bays["a1"]["sigma_cal_c"] == 3.0
    assert tick(plain, None, 0.0).bays["a1"]["sigma_cal_c"] == FLOOR_C


def test_estimator_section_requires_topology(cfg):
    m = cfg.to_dict()
    m["estimator"] = {}
    with pytest.raises(ConfigError, match="requires mpc.topology"):
        MpcConfig.from_mapping(m)


def test_a_serial_may_be_declared_on_one_bay_only():
    m = das_mapping()
    m["topology"]["bays"]["a1"]["serial"] = "S1"
    m["topology"]["bays"]["b1"]["serial"] = "S1"
    with pytest.raises(ConfigError, match="declared on both"):
        MpcConfig.from_mapping(m)


# ---------------------------------------------------------------------------
# numerics
# ---------------------------------------------------------------------------


@settings(max_examples=30, deadline=None)
@given(seed=st.integers(0, 10_000), scale=st.floats(0.01, 20.0))
def test_matrix_exponential_matches_the_eigen_decomposition(seed, scale):
    rng = np.random.default_rng(seed)
    n = 6
    a = rng.normal(size=(n, n))
    a = -(a @ a.T) * scale / n - np.eye(n) * 0.01  # symmetric, stable, stiff at large scale
    w, v = np.linalg.eigh(a)
    reference = v @ np.diag(np.exp(w)) @ v.T
    assert np.allclose(E._expm(a), reference, atol=1e-9, rtol=1e-7)


def test_discretisation_matches_the_affine_solution():
    a = np.array([[-0.5, 0.2], [0.1, -0.05]])
    c = np.array([10.0, 1.5])
    x0 = np.array([30.0, 40.0])
    h = 5.0
    phi, gamma = E._discretise(a, c, h)
    # exact: x(h) = e^{Ah} x0 + A^-1 (e^{Ah} - I) c
    eah = E._expm(a * h)
    exact = eah @ x0 + np.linalg.solve(a, (eah - np.eye(2)) @ c)
    assert np.allclose(phi @ x0 + gamma, exact, atol=1e-9)
    # an equilibrium stays put
    xe = np.linalg.solve(a, -c)
    assert np.allclose(phi @ xe + gamma, xe, atol=1e-9)


@settings(max_examples=40, deadline=None)
@given(seed=st.integers(0, 10_000))
def test_the_pair_update_is_the_joseph_form_with_h_on_two_states(seed):
    """A proximal member with a placement offset measures ``T_s + c``: the same update a
    textbook ``H = e_i + e_j`` gives, evaluated through the rank-one structure."""
    rng = np.random.default_rng(seed)
    n = int(rng.integers(3, 9))
    a = rng.normal(size=(n, n))
    p = a @ a.T + np.eye(n) * 0.01
    x = rng.normal(size=n) * 10.0
    i, j = (int(v) for v in rng.choice(n, size=2, replace=False))
    z, r = float(rng.normal() * 10.0), float(abs(rng.normal()) + 0.01)

    h = np.zeros(n)
    h[i] = h[j] = 1.0
    k = (p @ h) / (float(h @ p @ h) + r)
    ikh = np.eye(n) - np.outer(k, h)
    x_ref = x + k * (z - float(h @ x))
    p_ref = ikh @ p @ ikh.T + r * np.outer(k, k)

    xc, pc = x.copy(), p.copy()
    E._pair_update(xc, pc, i, j, z, r)
    assert np.allclose(xc, x_ref, atol=1e-12)
    assert np.allclose(pc, 0.5 * (p_ref + p_ref.T), atol=1e-12)
    assert np.allclose(pc, pc.T, atol=1e-12)
    assert np.linalg.eigvalsh(pc).min() >= -1e-9


@settings(max_examples=40, deadline=None)
@given(data=st.data())
def test_joseph_form_keeps_every_covariance_symmetric_and_psd(data):
    cfg = lcfg()
    floor = cfg.estimator.sigma_uncalibrated_c
    names = list(cfg.temps)
    mem = None
    ts = 0.0
    for _ in range(data.draw(st.integers(5, 40))):
        ts += data.draw(st.sampled_from([0.0, 1.0, 1.0, 1.0, 2.5, 10.0, 400.0]))
        kept = data.draw(st.lists(st.sampled_from(names), unique=True))
        temps = {n: data.draw(st.floats(15.0, 70.0)) for n in kept}
        u = {ch: data.draw(st.floats(0.0, 1.0)) for ch in cfg.channels}
        up = E.update(mem, cfg, temps=temps, u=u, ts=ts)
        mem = up.memory
        for zone in mem["zones"].values():
            p = np.array(zone["P"])
            assert np.all(np.isfinite(p)) and np.all(np.isfinite(zone["x"]))
            assert np.allclose(p, p.T, atol=1e-12)
            eig = np.linalg.eigvalsh(p)
            assert eig.min() >= -1e-9 * max(1.0, eig.max()), eig.min()
        for entry in up.estimates.values():
            assert math.isfinite(entry["t"]) and entry["sigma"] >= floor
        json.dumps(mem, allow_nan=False)


# ---------------------------------------------------------------------------
# the filter on fixed readings
# ---------------------------------------------------------------------------


def test_first_tick_inverts_the_prior_map_and_constant_readings_hold():
    cfg = lcfg()
    ups = run_ticks(cfg, 60)
    first = ups[0].estimates["a1"]
    assert first["t"] == pytest.approx(est.prior_drive_temp(PROX_C, SP))
    assert first["source"] == est.SOURCE_ESTIMATOR and not first["calibrated"]
    sigmas = [up.estimates["a1"]["sigma"] for up in ups]
    for up in ups:
        assert up.estimates["a1"]["t"] == pytest.approx(first["t"], abs=1e-6)
        assert up.zones["za"]["drift_c_per_min"] < 1e-6
    assert all(b <= a + 1e-4 for a, b in zip(sigmas, sigmas[1:], strict=False))  # settles
    assert sigmas[-1] < sigmas[0] and sigmas[-1] >= cfg.estimator.sigma_uncalibrated_c
    assert set(ups[-1].estimates) == {"a1", "a2", "b1"}  # c1 is declared empty


def test_sigma_grows_while_a_bay_has_no_trusted_sensor_and_shrinks_back():
    cfg = lcfg()
    ups = run_ticks(cfg, 30)
    mem = ups[-1].memory
    lost = run_ticks(cfg, 40, mem=mem, t0=30.0, prox_a2=None)
    sig = [up.estimates["a2"]["sigma"] for up in lost]
    assert all(b > a for a, b in zip(sig, sig[1:], strict=False))
    assert all(up.bays["a2"]["occupancy"] == "occupied" for up in lost)  # declared occupied
    back = run_ticks(cfg, 20, mem=lost[-1].memory, t0=70.0)
    assert back[-1].estimates["a2"]["sigma"] < sig[-1]


def test_a_redundant_member_keeps_the_bay_observed():
    cfg = lcfg(occupancy_hold_s=0.0)  # no debounce: the old rule
    mem = run_ticks(cfg, 30)[-1].memory
    ups = run_ticks(cfg, 30, mem=mem, t0=30.0, prox_a1=None)
    assert all(up.bays["a1"]["occupancy"] == "occupied" for up in ups)
    ups = run_ticks(cfg, 5, mem=ups[-1].memory, t0=60.0, prox_a1=None, prox_a1b=None)
    assert all(up.bays["a1"]["occupancy"] == "unknown" for up in ups)  # observability lost
    assert "a1" in ups[-1].estimates  # still constrained


def test_occupancy_debounce_rides_out_a_short_proximal_dropout(as_empty):
    """Item 19: an empty bay stays empty through a dropout shorter than the hold."""
    cfg, mem = as_empty(occupancy_hold_s=6.0)  # 6 ticks at dt = 1
    blind = run_ticks(cfg, 5, mem=mem, t0=100.0, prox_b1=None)
    assert [up.bays["b1"]["occupancy"] for up in blind] == ["empty"] * 5
    assert [up.bays["b1"]["pending_unknown_s"] for up in blind] == [1.0, 2.0, 3.0, 4.0, 5.0]
    assert "b1" not in blind[-1].estimates  # still unconstrained: no 25 degC^2 margin
    # the sensor comes back: the count restarts, a second dropout gets the full hold again
    back = run_ticks(cfg, 1, mem=blind[-1].memory, t0=105.0, prox_b1=SP + 0.1)
    assert back[-1].bays["b1"]["pending_unknown_s"] == 0.0
    again = run_ticks(cfg, 5, mem=back[-1].memory, t0=106.0, prox_b1=None)
    assert [up.bays["b1"]["occupancy"] for up in again] == ["empty"] * 5


def test_occupancy_debounce_gives_up_after_the_hold(as_empty):
    """A dropout that lasts is still a loss of observability, and once the verdict is in
    nothing is pending any more, however long the blindness goes on."""
    cfg, mem = as_empty(occupancy_hold_s=6.0)
    blind = run_ticks(cfg, 8, mem=mem, t0=100.0, prox_b1=None)
    seq = [up.bays["b1"]["occupancy"] for up in blind]
    assert seq == ["empty"] * 5 + ["unknown"] * 3
    entry = blind[5].estimates["b1"]  # the insert reset, one hold later than before
    assert entry["sigma"] > 4.0
    assert blind[5].bays["b1"]["pending_unknown_s"] == 0.0
    # blind for three more holds: pending_unknown_s is a pending transition, not an uptime
    more = run_ticks(cfg, 20, mem=blind[-1].memory, t0=108.0, prox_b1=None)
    assert all(up.bays["b1"]["occupancy"] == "unknown" for up in more)
    assert [up.bays["b1"]["pending_unknown_s"] for up in more] == [0.0] * 20


def test_occupancy_debounce_zero_is_the_old_rule(as_empty):
    cfg, mem = as_empty(occupancy_hold_s=0.0)
    blind = run_ticks(cfg, 3, mem=mem, t0=100.0, prox_b1=None)
    assert [up.bays["b1"]["occupancy"] for up in blind] == ["unknown"] * 3


# ---------------------------------------------------------------------------
# placement offsets (plan section 8 item 67)
# ---------------------------------------------------------------------------

#: How far apart the two proximal sensors of bay a1 sit in these tests, degC.
PLACEMENT_GAP_C = 4.0


def test_two_proximal_sensors_at_different_placements_are_an_offset_not_a_swap():
    """The second member reads PLACEMENT_GAP_C below the first on every tick: the filter
    carries that as its placement offset, and the bay's sigma stays at the uncalibrated
    floor instead of the fast-swap rule inflating it every tick (item 67)."""
    cfg = lcfg()
    ups = run_ticks(cfg, 60, prox_a1b=PROX_C - PLACEMENT_GAP_C)
    sigmas = [up.estimates["a1"]["sigma"] for up in ups]
    assert max(sigmas) < est.sigma_uncalibrated_c(cfg) + 0.05, max(sigmas)
    offsets = [up.bays["a1"]["offsets_c"]["prox_a1b"] for up in ups]
    assert offsets[0] == 0.0
    assert offsets[-1] == pytest.approx(-PLACEMENT_GAP_C, abs=0.05)
    # the node itself still follows the anchor, so the drive estimate is the anchor's
    assert ups[-1].estimates["a1"]["t"] == pytest.approx(est.prior_drive_temp(PROX_C, SP), abs=0.1)


def test_one_node_for_both_members_is_what_item_67_reported():
    """``proximal_offset_c: 0`` is the fusion before item 67: the same disagreement trips
    the fast-swap rule and the bay's sigma stays far above ``sigma_fault_c``."""
    cfg = lcfg(proximal_offset_c=0.0)
    ups = run_ticks(cfg, 60, prox_a1b=PROX_C - PLACEMENT_GAP_C)
    sigmas = [up.estimates["a1"]["sigma"] for up in ups]
    assert max(sigmas) > 4.0, max(sigmas)
    assert ups[-1].bays["a1"]["offsets_c"] == {}


def test_a_bay_with_one_proximal_sensor_carries_no_offset():
    cfg = lcfg()
    up = run_ticks(cfg, 5)[-1]
    assert up.bays["a1"]["offsets_c"] == {"prox_a1b": pytest.approx(0.0, abs=0.2)}
    for bay in ("a2", "b1", "c1"):
        assert up.bays[bay]["offsets_c"] == {}
    # one offset state in za (bay a1), none in zb or zc
    assert len(up.memory["zones"]["za"]["x"]) == 2 + 3 * 2 + 1
    assert len(up.memory["zones"]["zb"]["x"]) == 2 + 3 * 1


def test_a_swap_still_widens_a_bay_with_two_proximal_sensors():
    """A swap moves the drive, so both members step together: the fast-swap rule fires and
    the margin widens, exactly as on a bay with one sensor."""
    cfg = lcfg()
    mem = run_ticks(cfg, 60, prox_a1b=PROX_C - PLACEMENT_GAP_C)[-1].memory
    before = run_ticks(cfg, 1, mem=mem, t0=60.0, prox_a1b=PROX_C - PLACEMENT_GAP_C)[-1]
    hot = run_ticks(
        cfg, 1, mem=before.memory, t0=61.0, prox_a1=PROX_C + 8.0, prox_a1b=PROX_C + 4.0
    )[-1]
    assert hot.estimates["a1"]["sigma"] > 4.0 > before.estimates["a1"]["sigma"]
    assert hot.bays["a1"]["settling"] is True and hot.bays["a1"]["observed"] is True


def test_the_anchor_alone_is_enough_and_so_is_the_other_member():
    """Either member keeps the bay observed; the one carrying an offset costs the offset's
    own uncertainty, which is small once it has converged."""
    cfg = lcfg()
    mem = run_ticks(cfg, 120, prox_a1b=PROX_C - PLACEMENT_GAP_C)[-1].memory
    both = run_ticks(cfg, 5, mem=mem, t0=120.0, prox_a1b=PROX_C - PLACEMENT_GAP_C)[-1]
    anchor = run_ticks(cfg, 5, mem=mem, t0=120.0, prox_a1b=None)[-1]
    other = run_ticks(cfg, 5, mem=mem, t0=120.0, prox_a1=None, prox_a1b=PROX_C - PLACEMENT_GAP_C)[
        -1
    ]
    for up in (anchor, other):
        assert up.estimates["a1"]["sigma"] < est.sigma_uncalibrated_c(cfg) + 0.1
        assert up.estimates["a1"]["t"] == pytest.approx(both.estimates["a1"]["t"], abs=0.3)


# ---------------------------------------------------------------------------
# a node per proximal sensor (plan section 8 item 101)
# ---------------------------------------------------------------------------

#: How much less of the drive the second member of bay a1 sees in these tests. A *slope*
#: difference: the disagreement it makes is proportional to the drive-to-air rise, which
#: is exactly what one node with a constant offset per member cannot carry.
MEMBER_SLOPE_DELTA = -0.2
#: The two drive-to-air rises the tests alternate between, degC. A constant offset fits
#: either one on its own; only a rise that moves tells an offset from a slope.
RISES_C = (5.0, 20.0)
#: A prior spread wide enough for :data:`MEMBER_SLOPE_DELTA`.
SLOPE_SPREAD = 0.25


def member_reading(t_drive: float, t_air: float, slope_delta: float = 0.0) -> float:
    """The steady reading of a proximal sensor whose slope is the prior's plus
    ``slope_delta``: ``s T_d + (1 - s) T_a + b`` with the prior offset ``b``."""
    s = (1.0 - est.PRIOR_BETA) + slope_delta
    return s * t_drive + (1.0 - s) * t_air + est.PRIOR_OFFSET_C


def pair_cfg(spread: float = 0.0) -> MpcConfig:
    """The small fixture at one of the two layouts, with a short settling window: these
    tests move the load between two plateaus, and every move is a transient the fast-swap
    rule fires on, so ``settling`` has to be able to expire inside a plateau."""
    return lcfg(proximal_slope_spread=spread, bay_settle_s=5.0)


def rise_profile(cycles: int, *, ramp: int = 100, hold: int = 200) -> list[float]:
    """A drive-to-air rise sweeping between :data:`RISES_C` and back, ``cycles`` times.

    Ramped between long plateaus: the readings here are the steady values of each rise, so
    while the rise moves the filter's own drive estimate lags them and the rows it takes
    are biased. What identifies the slope is the plateaus, where it does not.
    """
    lo, hi = RISES_C
    out: list[float] = []
    for i in range(cycles):
        a, b = (lo, hi) if i % 2 == 0 else (hi, lo)
        out += [a + (b - a) * k / ramp for k in range(ramp)] + [b] * hold
    return out


def pair_ticks(cfg: MpcConfig, rises, *, mem=None, t0: float = 0.0):
    """Both members of bay a1 reporting at each rise of ``rises``: (updates, memory)."""
    ups = []
    t = t0
    for rise in rises:
        up = tick(
            cfg,
            mem,
            t,
            prox_a1=member_reading(SP + rise, SP),
            prox_a1b=member_reading(SP + rise, SP, MEMBER_SLOPE_DELTA),
        )
        mem, t = up.memory, t + cfg.dt
        ups.append(up)
    return ups, mem


def truth_gap(rise: float) -> float:
    """What the two members really differ by at this rise."""
    return MEMBER_SLOPE_DELTA * rise


def test_a_node_per_sensor_learns_the_slope_between_two_placements():
    """Item 101: with ``proximal_slope_spread`` every proximal member of a bay is its own
    node with its own map, learned as a difference from the anchor's out of the two
    readings alone -- no SMART, because both members see the same drive and the same air."""
    cfg = pair_cfg(SLOPE_SPREAD)
    profile = rise_profile(6)
    ups, _ = pair_ticks(cfg, profile)
    learned = ups[-1].bays["a1"]["proximal_map"]["prox_a1b"]
    assert learned["slope_delta"] == pytest.approx(MEMBER_SLOPE_DELTA, abs=0.05)
    assert learned["offset_delta_c"] == pytest.approx(0.0, abs=0.5)
    assert learned["samples"] > 400
    # the anchor still holds the bay's own map, so the drive estimate is the anchor's
    assert ups[-1].estimates["a1"]["t"] == pytest.approx(SP + profile[-1], abs=0.3)


def silent_member_run(cfg: MpcConfig, ramp: int = 120, hold: int = 30):
    """Train at the high rise, lose the redundant member, let the load fall to the low one
    while only the anchor reports, then hand the member back its own (correct) reading."""
    lo, hi = RISES_C
    train = rise_profile(5)  # an odd number of cycles ends at the high rise
    _, mem = pair_ticks(cfg, train)
    t = float(len(train))
    for rise in [hi + (lo - hi) * k / ramp for k in range(ramp)] + [lo] * hold:
        up = tick(cfg, mem, t, prox_a1=member_reading(SP + rise, SP), prox_a1b=None)
        mem, t = up.memory, t + cfg.dt
    return tick(
        cfg,
        mem,
        t,
        prox_a1=member_reading(SP + lo, SP),
        prox_a1b=member_reading(SP + lo, SP, MEMBER_SLOPE_DELTA),
    )


def test_a_member_that_returns_after_the_load_moved_is_a_swap_only_on_the_fused_node():
    """The alternative item 101 weighs this against: one node and a random-walk offset.

    While both members report, the offset is *measured* every tick and follows a slope
    almost as well as a map would. What it cannot do is **predict**: a member that goes
    quiet leaves its offset at whatever rise it last saw, so when the load moves under it
    and the member comes back, the reading it brings is a jump. That is what keeps the
    example's redundant pairs out of ``trust_rule: sigma`` (item 67's leftover). A node of
    its own carries the member through the same drive and air the anchor sees, so the
    return is an ordinary reading.
    """
    fused = silent_member_run(pair_cfg())
    per_sensor = silent_member_run(pair_cfg(SLOPE_SPREAD))
    assert fused.bays["a1"]["settling"] is True
    assert fused.bays["a1"]["settling_reason"] == "jump"
    assert fused.estimates["a1"]["sigma"] > est.sigma_uncalibrated_c(pair_cfg()) + 2.0
    assert per_sensor.bays["a1"]["settling"] is False
    assert per_sensor.estimates["a1"]["sigma"] == pytest.approx(
        est.sigma_uncalibrated_c(pair_cfg(SLOPE_SPREAD)), abs=0.05
    )
    # The map predicts the gap at a rise it has not seen the member at. Not exactly: the
    # offset half of it is a random walk (``q_offset``, so a placement that *drifts* can
    # still be followed), which is what keeps the ramps' biased rows nudging it for ever
    # instead of letting P collapse on them. A few tenths of a degC is what that costs.
    assert per_sensor.bays["a1"]["offsets_c"]["prox_a1b"] == pytest.approx(
        truth_gap(RISES_C[0]), abs=0.5
    )


def test_a_node_per_sensor_costs_no_extra_state():
    """The offset row of item 67 becomes the further member's own node, so the filter has
    exactly the states it had -- which is what keeps the step budget where it was."""
    fused = run_ticks(lcfg(), 3)[-1]
    per_sensor = run_ticks(lcfg(proximal_slope_spread=SLOPE_SPREAD), 3)[-1]
    for zone in ("za", "zb", "zc"):
        assert len(per_sensor.memory["zones"][zone]["x"]) == len(fused.memory["zones"][zone]["x"])
    assert len(per_sensor.memory["zones"]["za"]["x"]) == 2 + 3 * 2 + 1


def _no_redundant_cfg(**estimator):
    """The small fixture with bay a1's redundant member removed: one sensor per bay."""
    m = das_mapping()
    m["temps"] = [t for t in m["temps"] if t != "prox_a1b"]
    del m["sensors"]["prox_a1b"]
    m["setpoints"] = {}
    if estimator:
        m["estimator"] = estimator
    return MpcConfig.from_mapping(m)


def test_one_sensor_per_bay_is_the_same_arithmetic_either_way():
    """A config without a redundant proximal pair has nothing for the new layout to do, so
    it must be bit-identical -- which is what lets the key default to the fused layout
    without splitting the estimator into two filters to maintain."""
    fused = run_ticks(_no_redundant_cfg(), 40)
    per_sensor = run_ticks(_no_redundant_cfg(proximal_slope_spread=SLOPE_SPREAD), 40)
    for a, b in zip(fused, per_sensor, strict=True):
        assert a.estimates == b.estimates
        for zone in a.memory["zones"]:
            assert a.memory["zones"][zone]["x"] == b.memory["zones"][zone]["x"]
            assert a.memory["zones"][zone]["P"] == b.memory["zones"][zone]["P"]


@pytest.mark.parametrize("absent", ["prox_a1", "prox_a1b"])
def test_either_member_alone_keeps_the_bay_on_its_own_node(absent):
    """One member silent: the bay stays observed and its estimate stays where both members
    had it, whichever one is missing -- the anchor's node is predicted through the drive,
    the other member's through its own learned map."""
    cfg = pair_cfg(SLOPE_SPREAD)
    train = rise_profile(6)
    _, mem = pair_ticks(cfg, train)
    rise = train[-1]
    both = tick(
        cfg,
        mem,
        float(len(train)),
        prox_a1=member_reading(SP + rise, SP),
        prox_a1b=member_reading(SP + rise, SP, MEMBER_SLOPE_DELTA),
    )
    values = {
        "prox_a1": member_reading(SP + rise, SP),
        "prox_a1b": member_reading(SP + rise, SP, MEMBER_SLOPE_DELTA),
    }
    values[absent] = None
    alone = tick(cfg, mem, float(len(train)), **values)
    assert alone.bays["a1"]["observed"] is True
    assert alone.bays["a1"]["settling"] is False
    assert alone.estimates["a1"]["t"] == pytest.approx(both.estimates["a1"]["t"], abs=0.4)
    assert alone.estimates["a1"]["sigma"] < est.sigma_uncalibrated_c(cfg) + 0.1


def test_a_member_that_lies_the_other_way_is_not_a_swap_on_either_layout():
    """The agreement test (item 17) still decides: one bay holds one drive, so a swap moves
    both members the same way and a disagreement is placement, not a swap."""
    for cfg in (pair_cfg(), pair_cfg(SLOPE_SPREAD)):
        train = rise_profile(4)
        _, mem = pair_ticks(cfg, train)
        rise = train[-1]
        lying = tick(
            cfg,
            mem,
            float(len(train)),
            prox_a1=member_reading(SP + rise, SP) + 6.0,
            prox_a1b=member_reading(SP + rise, SP, MEMBER_SLOPE_DELTA) - 6.0,
        )
        assert lying.bays["a1"]["settling"] is False, cfg.estimator.proximal_slope_spread


def test_a_hot_swap_widens_the_bay_on_either_layout():
    """Both members step together, which is what a swap does; the rule fires and the bay
    reports ``settling`` with ``jump`` as its reason (item 100)."""
    for cfg in (pair_cfg(), pair_cfg(SLOPE_SPREAD)):
        train = rise_profile(4)
        _, mem = pair_ticks(cfg, train)
        rise = train[-1]
        hot = tick(
            cfg,
            mem,
            float(len(train)),
            prox_a1=member_reading(SP + rise + 12.0, SP),
            prox_a1b=member_reading(SP + rise + 12.0, SP, MEMBER_SLOPE_DELTA),
        )
        assert hot.estimates["a1"]["sigma"] > 4.0
        assert hot.bays["a1"]["settling"] is True
        assert hot.bays["a1"]["settling_reason"] == "jump"


@pytest.mark.parametrize("missing", [("prox_a1",), ("prox_a1b",), ("prox_a1", "prox_a1b")])
def test_a_member_lost_and_returned_is_judged_like_any_other_reading(missing):
    """A drive may be changed while a member is silent, so its return is an innovation --
    and a return that agrees with the member that never left is not a swap."""
    cfg = pair_cfg(SLOPE_SPREAD)
    train = rise_profile(4)
    _, mem = pair_ticks(cfg, train)
    rise = train[-1]
    values = {
        "prox_a1": member_reading(SP + rise, SP),
        "prox_a1b": member_reading(SP + rise, SP, MEMBER_SLOPE_DELTA),
    }
    gone = tick(cfg, mem, float(len(train)), **{**values, **dict.fromkeys(missing)})
    back = tick(cfg, gone.memory, float(len(train)) + 1.0, **values)
    assert back.bays["a1"]["seeded"] is True
    assert back.bays["a1"]["settling"] is False
    assert back.estimates["a1"]["sigma"] < est.sigma_uncalibrated_c(cfg) + 0.2


def test_proximal_slope_spread_is_a_config_key():
    assert ESTIMATOR_DEFAULTS["proximal_slope_spread"] == 0.0
    for bad in (-0.1, 0.6):
        with pytest.raises(ConfigError, match="proximal_slope_spread"):
            lcfg(proximal_slope_spread=bad)


#: How far the redundant member of bay a1 wanders in :func:`drift_run`, degC. Fouling, a
#: sensor working loose, a thermistor ageing: a *placement* moving, which is exactly the
#: quantity ``q_offset`` makes a random walk.
MEMBER_DRIFT_C = -6.0
DRIFT_TICKS = 900


def drift_run(cfg: MpcConfig, noise: float = 0.0, seed: int = 1):
    """Both members reporting at a fixed drive while the redundant one slides
    :data:`MEMBER_DRIFT_C` away from the anchor over :data:`DRIFT_TICKS` ticks."""
    rnd = random.Random(seed)
    truth_d = SP + RISES_C[1]
    mem, up = None, None
    for i in range(DRIFT_TICKS):
        gone = MEMBER_DRIFT_C * i / (DRIFT_TICKS - 1)
        a = member_reading(truth_d, SP) + (rnd.gauss(0.0, noise) if noise else 0.0)
        b = member_reading(truth_d, SP) + gone + (rnd.gauss(0.0, noise) if noise else 0.0)
        up = tick(cfg, mem, i * cfg.dt, prox_a1=a, prox_a1b=b)
        mem = up.memory
    return up, truth_d


@pytest.mark.parametrize("noise", [0.0, 0.15])
def test_a_drifting_member_is_followed_by_its_map_and_not_by_the_shared_drive(noise):
    """A placement that drifts must land in the member's own map, as it landed in the
    random-walk offset state of the fused layout (item 67).

    Without a process-noise term the RLS has no forgetting: ``P`` collapses, the learned
    map freezes, and the disagreement the member keeps bringing has nowhere to go but the
    drive both members share -- a silent *under*-estimate of the drive, with ``sigma`` at
    the uncalibrated floor and so no margin covering it. That is a failure reducing
    cooling (plan section 2), which is the one thing the estimator may not do.
    """
    fused, truth_d = drift_run(pair_cfg(), noise)
    per_sensor, _ = drift_run(pair_cfg(SLOPE_SPREAD), noise)
    assert fused.estimates["a1"]["t"] == pytest.approx(truth_d, abs=0.5)
    assert per_sensor.estimates["a1"]["t"] == pytest.approx(truth_d, abs=0.5)
    # the drift is carried by the member's learned offset, not by its slope
    learned = per_sensor.bays["a1"]["proximal_map"]["prox_a1b"]
    assert learned["offset_delta_c"] < MEMBER_DRIFT_C / 2.0
    assert learned["slope_delta"] == pytest.approx(0.0, abs=SLOPE_SPREAD)


def test_the_learned_map_cannot_run_away_along_the_direction_its_rows_cannot_see():
    """A row ``z = ds * rise + db`` identifies only the combination, so ``(ds, db)`` is free
    to walk along ``(1, -rise)`` for ever and still fit every reading. Bounded, it cannot:
    the slope stays inside the prior spread the operator declared, the offset inside what
    keeps the member's map usable, and the clip in ``member_map`` never has to bite."""
    cfg = pair_cfg(SLOPE_SPREAD)
    for noise in (0.0, 0.15):
        up, _ = drift_run(cfg, noise)
        learned = up.bays["a1"]["proximal_map"]["prox_a1b"]
        assert abs(learned["slope_delta"]) <= SLOPE_SPREAD + 1e-9, learned
        assert E.CAL_SLOPE_BOUNDS[0] <= learned["slope"] <= E.CAL_SLOPE_BOUNDS[1], learned
        assert E.CAL_OFFSET_BOUNDS[0] <= learned["offset_c"] <= E.CAL_OFFSET_BOUNDS[1], learned
        assert learned["clipped"] is False, learned


def test_the_diagnostics_publish_the_map_the_filter_predicts_with():
    """``proximal_map`` carries the effective ``(slope, offset_c)``, not only the learned
    difference: a reader working out why a bay under-reads must see what the filter is
    running, and ``clipped`` says when the two have parted company."""
    cfg = pair_cfg(SLOPE_SPREAD)
    ups, _ = pair_ticks(cfg, rise_profile(4))
    learned = ups[-1].bays["a1"]["proximal_map"]["prox_a1b"]
    s_prior, b_prior = E.CAL_PRIOR
    assert learned["slope"] == pytest.approx(s_prior + learned["slope_delta"])
    assert learned["offset_c"] == pytest.approx(b_prior + learned["offset_delta_c"])
    assert learned["clipped"] is False
    # the fused layout has no per-member map at all, so it publishes none
    fused, _ = pair_ticks(pair_cfg(), rise_profile(1))
    assert fused[-1].bays["a1"]["proximal_map"] == {}


# ---------------------------------------------------------------------------
# per-bay seeding, settling and the blind air clock (items 69 and 70)
# ---------------------------------------------------------------------------


def test_a_bay_missing_on_the_first_tick_is_seeded_by_its_first_reading():
    """Item 69: the bay's node starts at the zone air while nothing reads it, so the
    reading that arrives later would look like a 5 degC jump. It seeds the bay instead."""
    cfg = lcfg()
    first = run_ticks(cfg, 3, prox_b1=None)[-1]
    assert first.bays["b1"]["seeded"] is False
    assert "b1" in first.estimates  # still constrained, on the prior
    back = run_ticks(cfg, 3, mem=first.memory, t0=3.0)
    assert back[0].bays["b1"]["seeded"] is True
    assert max(up.estimates["b1"]["sigma"] for up in back) < est.sigma_uncalibrated_c(cfg) + 0.05
    assert back[-1].estimates["b1"]["t"] == pytest.approx(est.prior_drive_temp(PROX_C, SP), abs=0.5)
    assert not any(up.bays["b1"]["settling"] for up in back)


def test_a_sensor_that_returns_after_a_later_loss_still_widens_its_bay():
    """The counterpart: a bay is seeded once. A drive can be changed while nothing watches,
    so the reading that comes back after a loss is judged, not trusted blindly."""
    cfg = lcfg()
    mem = run_ticks(cfg, 30)[-1].memory
    lost = run_ticks(cfg, 30, mem=mem, t0=30.0, prox_b1=None)[-1]
    back = run_ticks(cfg, 1, mem=lost.memory, t0=60.0, prox_b1=PROX_C + 10.0)[-1]
    assert back.estimates["b1"]["sigma"] > lost.estimates["b1"]["sigma"]
    assert back.bays["b1"]["settling"] is True


def test_settling_expires_after_bay_settle_s():
    cfg = lcfg(bay_settle_s=10.0)
    mem = run_ticks(cfg, 30)[-1].memory
    jump = run_ticks(cfg, 1, mem=mem, t0=30.0, prox_b1=PROX_C + 10.0)[-1]
    assert jump.bays["b1"]["settling"] is True
    later = run_ticks(cfg, 12, mem=jump.memory, t0=31.0, prox_b1=PROX_C + 10.0)
    assert [up.bays["b1"]["settling"] for up in later][-1] is False


def flap(cfg: MpcConfig, mem, n: int, *, t0: float, cadence: int, jump_c: float = 10.0):
    """``n`` ticks with ``prox_b1`` stepping ``jump_c`` away every ``cadence`` ticks:
    (settling flags, sigmas, the memory after the last tick)."""
    flags, sigmas = [], []
    for i in range(n):
        step_c = jump_c if cadence and i % cadence == 0 else 0.0
        up = run_ticks(cfg, 1, mem=mem, t0=t0 + i * cfg.dt, prox_b1=PROX_C + step_c)[-1]
        mem = up.memory
        flags.append(up.bays["b1"]["settling"])
        sigmas.append(up.estimates["b1"]["sigma"])
    return flags, sigmas, mem


@pytest.mark.parametrize("cadence", [2, 5, 12, 20])
def test_repeated_jumps_spend_the_settling_budget_and_then_stop_exempting(cadence):
    """A bay's settling windows may suspend the sigma check for at most ``bay_settle_max_s``
    in total. Nothing about the *cadence* of the jumps buys more: a bay's sigma falls back
    within a tick or two of each jump, so a rule that re-armed on a recovered sigma renewed
    the window for ever at every cadence but the fastest, and the zone never faulted."""
    cfg = lcfg(bay_settle_s=10.0, bay_settle_max_s=20.0)
    mem = run_ticks(cfg, 30)[-1].memory
    flags, sigmas, _ = flap(cfg, mem, 80, t0=30.0, cadence=cadence)
    assert flags[0] is True
    # the budget, plus at most the window that was still open when it ran out
    exempt_s = sum(flags) * cfg.dt
    assert exempt_s <= cfg.estimator.bay_settle_max_s + cfg.estimator.bay_settle_s, exempt_s
    assert not any(flags[40:]), (cadence, flags)
    # ...and the jumps did keep firing, so the zone faults on those ticks as it did before
    pairs = zip(sigmas, flags, strict=True)
    faulting = [s > cfg.estimator.sigma_fault_c and not f for s, f in pairs]
    assert any(faulting[40:]), max(sigmas[40:])


def test_a_clean_run_earns_the_settling_budget_back():
    """The budget is not spent for good: a bay that runs ``bay_settle_max_s`` with neither a
    window nor a sigma over its threshold is a bay whose next swap is a swap again."""
    cfg = lcfg(bay_settle_s=10.0, bay_settle_max_s=20.0)
    mem = run_ticks(cfg, 30)[-1].memory
    flags, _, mem = flap(cfg, mem, 60, t0=30.0, cadence=2)
    assert flags[-1] is False  # the budget is gone
    quiet, _, mem = flap(cfg, mem, 40, t0=90.0, cadence=0)
    assert not any(quiet)
    again, _, _ = flap(cfg, mem, 1, t0=130.0, cadence=1)
    assert again == [True]


def test_bay_settle_max_s_zero_grants_no_exemption_at_all():
    cfg = lcfg(bay_settle_s=0.0, bay_settle_max_s=0.0)
    mem = run_ticks(cfg, 30)[-1].memory
    flags, sigmas, _ = flap(cfg, mem, 10, t0=30.0, cadence=1)
    assert not any(flags)
    assert max(sigmas) > cfg.estimator.sigma_fault_c  # the widening is still there


def test_an_empty_bay_spends_no_settling_budget():
    """A bay the trust rule skips outright (``empty``) is charged nothing while it waits:
    the exemption it is not using must not be spent, or a slow hot swap -- a drive pulled,
    the bay confirmed empty, a drive inserted minutes later -- would run out mid-way."""
    cfg = lcfg(bay_settle_s=600.0, bay_settle_max_s=900.0)
    mem = run_ticks(cfg, 30)[-1].memory
    pulled = run_ticks(cfg, 600, mem=mem, t0=30.0, prox_b1=SP)  # the drive is out
    first = next(i for i, up in enumerate(pulled) if up.bays["b1"]["occupancy"] == "empty")
    spent = pulled[first].memory["bays"]["b1"]["spent"]
    assert 0.0 < spent < cfg.estimator.bay_settle_max_s  # the occupied part did cost
    assert pulled[-1].memory["bays"]["b1"]["spent"] == spent  # the empty part did not
    assert len(pulled) - first > 200  # over 200 empty ticks of it
    back = run_ticks(cfg, 3, mem=pulled[-1].memory, t0=630.0, prox_b1=PROX_C + 10.0)
    assert back[-1].bays["b1"]["settling"] is True


def test_air_blind_s_counts_the_time_without_a_trusted_zone_air_reading():
    cfg = lcfg()
    ups = run_ticks(cfg, 5)
    assert all(up.zones["za"]["air_blind_s"] == 0.0 for up in ups)
    blind = run_ticks(cfg, 6, mem=ups[-1].memory, t0=5.0, air_a=None, air_a2=None)
    assert [up.zones["za"]["air_blind_s"] for up in blind] == [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]
    assert blind[-1].zones["zb"]["air_blind_s"] == 0.0  # zb kept its own sensor
    back = run_ticks(cfg, 1, mem=blind[-1].memory, t0=11.0)[-1]
    assert back.zones["za"]["air_blind_s"] == 0.0


def test_air_blind_s_counts_a_tick_gap_in_full():
    """Wall clock, not the occupancy horizon (which caps at 3 dt): under-counting a gap
    delays the ``sigma:zone_air_blind`` fault, and the air sigma cannot make it up --
    that the variance stays small with nothing reading the air is item 70's premise."""
    cfg = lcfg()
    mem = run_ticks(cfg, 5)[-1].memory
    blind = run_ticks(cfg, 1, mem=mem, t0=5.0, air_a=None, air_a2=None)[-1]
    assert blind.zones["za"]["air_blind_s"] == 1.0
    gap = tick(cfg, blind.memory, 605.0, air_a=None, air_a2=None)
    assert gap.zones["za"]["air_blind_s"] == pytest.approx(601.0)  # not 1 + 3 * dt
    later = tick(cfg, gap.memory, 1205.0, air_a=None, air_a2=None)
    assert later.zones["za"]["air_blind_s"] == pytest.approx(1201.0)
    assert later.zones["za"]["sigma_air_c"] < 1.0  # the variance never says it


# ---------------------------------------------------------------------------
# the two notions of settling (plan section 8 item 100)
# ---------------------------------------------------------------------------


def test_a_jump_is_named_in_the_diagnostics_with_the_time_it_has_left():
    """One owner, and it says which exemption and for how long: the ``sigma`` trust rule's
    (``settling``, a deliberate widening, bounded by ``bay_settle_max_s``) and the DAS
    MPC validity gate's (``model_exempt``, which also covers what the trust rule must not
    excuse). ``/api/state`` carries both per bay."""
    cfg = lcfg(bay_settle_s=10.0)
    mem = run_ticks(cfg, 30)[-1].memory
    jump = run_ticks(cfg, 1, mem=mem, t0=30.0, prox_b1=PROX_C + 10.0)[-1]
    info = jump.bays["b1"]
    assert info["settling"] is True and info["settling_reason"] == "jump"
    assert info["settling_until_s"] == pytest.approx(cfg.estimator.bay_settle_s)
    assert info["settling_spent_s"] == 0.0  # charged from the next tick on
    assert info["settling_budget_s"] == cfg.estimator.bay_settle_max_s
    # the model gate excuses the same tick, but for the level, not the event
    assert info["model_exempt"] is True and info["model_exempt_reason"] == "uncertain"
    assert info["model_exempt_until_s"] == pytest.approx(cfg.estimator.bay_settle_s)
    later = run_ticks(cfg, 6, mem=jump.memory, t0=31.0, prox_b1=PROX_C)[-1]
    assert later.bays["b1"]["settling_until_s"] < info["settling_until_s"]
    assert later.bays["b1"]["settling_spent_s"] > 0.0


def test_an_occupancy_change_is_its_own_reason():
    cfg = lcfg(empty_confirm_s=10.0, bay_settle_s=60.0)
    pulled = run_ticks(cfg, 100, prox_b1=SP + 0.1)
    gone = next(up for up in pulled if up.bays["b1"]["occupancy"] == "empty")
    assert gone.bays["b1"]["settling_reason"] == "occupancy"
    assert gone.bays["b1"]["model_exempt_reason"] == "occupancy"
    # an empty bay carries no estimate, so nothing marks ``uncertain`` for it either
    assert gone.bays["b1"]["model_exempt"] is True
    assert "b1" not in gone.estimates


def test_a_bay_nobody_reads_is_out_of_the_model_checks_but_still_faults_its_zone():
    """The untangling: what the solver used to infer from a wide sigma was **two** facts.
    A bay the filter widened on purpose is following a swap, and the trust rule may excuse
    it; a bay whose sigma is wide because nothing has read it for a while is an
    observability loss, and the trust rule must not. Both keep it out of the model checks,
    because neither is evidence about a model -- so the model gate honours ``uncertain``
    and the trust rule does not."""
    cfg = lcfg(bay_settle_s=600.0)
    mem = run_ticks(cfg, 30)[-1].memory
    blind = run_ticks(cfg, 900, mem=mem, t0=30.0, prox_b1=None)[-1]
    info = blind.bays["b1"]
    assert info["observed"] is False
    assert info["settling"] is False and info["settling_reason"] is None
    assert info["model_exempt"] is True and info["model_exempt_reason"] == "uncertain"
    assert blind.estimates["b1"]["sigma"] > cfg.estimator.sigma_fault_c
    (reason,) = zones.sigma_reasons("zb", cfg, blind)
    assert reason.startswith("sigma:bay:b1=")


def test_an_accepted_calibration_is_a_model_exemption_and_not_a_trust_one():
    """An accepted map re-maps the drive estimate and moves the ``sigma_cal`` floor under
    it, which the model's own prediction cannot be blamed for -- and which is no reason to
    stop checking the bay's sigma, because the filter did not widen anything."""
    cfg = das_cfg(setpoints={})
    ups, mem = [], None
    for k in range(1, 61):
        ts = float(k) * 10.0
        up = tick(
            cfg,
            mem,
            ts,
            calibration=_manual("a1", 44.0 + 3.0 * math.sin(0.7 * k), ts),
            prox_a1=PROX_C,
        )
        mem = up.memory
        ups.append(up)
    step = next(
        up
        for up, before in zip(ups[1:], ups, strict=False)
        if up.bays["a1"]["calibrated"] and not before.bays["a1"]["calibrated"]
    )
    assert step.bays["a1"]["sigma_cal_c"] != FLOOR_C  # the floor moved on acceptance
    after = next(up for up in ups if up.bays["a1"]["model_exempt_reason"] == "calibration")
    assert after.bays["a1"]["settling"] is False
    assert after.bays["a1"]["model_exempt"] is True


def test_the_model_exemption_keys_are_config_keys():
    assert ESTIMATOR_DEFAULTS["bay_uncertain_var_c2"] == 1.0
    assert ESTIMATOR_DEFAULTS["bay_cal_step_c"] == 0.05
    for key in ("bay_uncertain_var_c2", "bay_cal_step_c"):
        with pytest.raises(ConfigError, match=key):
            lcfg(**{key: 0.0})
    # a bay whose variance stays under a raised threshold is no longer excused
    cfg = lcfg(bay_settle_s=10.0, bay_uncertain_var_c2=1000.0)
    mem = run_ticks(cfg, 30)[-1].memory
    jump = run_ticks(cfg, 1, mem=mem, t0=30.0, prox_b1=PROX_C + 10.0)[-1]
    assert jump.bays["b1"]["settling"] is True  # the trust rule still excuses the jump
    assert jump.bays["b1"]["model_exempt"] is False


def test_a_redundant_air_sensor_keeps_the_zone_from_going_blind():
    cfg = lcfg()
    mem = run_ticks(cfg, 5)[-1].memory
    ups = run_ticks(cfg, 5, mem=mem, t0=5.0, air_a=None)
    assert all(up.zones["za"]["air_blind_s"] == 0.0 for up in ups)


# ---------------------------------------------------------------------------
# fast-swap rule: the members of a bay have to agree (section 8 item 17)
# ---------------------------------------------------------------------------


def _disagreeing_run(ticks: int, gap_c: float):
    """``a1``'s two proximal members ``gap_c`` apart for good, with SMART every tick."""
    cfg = das_cfg(setpoints={}, topology=_serial_topology("a1", "A"))
    mem, ups = None, []
    for i in range(ticks):
        ts = float(i) * cfg.dt
        up = tick(
            cfg,
            mem,
            ts,
            prox_a1=PROX_C,
            prox_a1b=PROX_C - gap_c,
            smart={"A": {"temp_c": 44.0, "age_s": 0.0, "model": "M"}},
        )
        mem = up.memory
        ups.append(up)
    return cfg, ups


def test_two_proximal_members_that_disagree_never_latch_the_bays_smart_out():
    # Before the fix the standing disagreement re-armed the fast-swap rule on every
    # tick and ``jumped`` gated SMART out of the calibration for the life of the run.
    _, ups = _disagreeing_run(60, 4.0)
    assert ups[-1].summary["smart_used"] >= 25
    assert ups[-1].summary["smart_rejected"] == 0
    assert ups[-1].bays["a1"]["calibration"]["samples"] >= 25
    # (a constant stream does not identify the slope, so the entry is not accepted here;
    # the accuracy this buys is measured on the truth simulator, see the rich sweep)
    assert ups[-1].bays["a1"]["calibration"]["fresh_samples"] >= 25
    # the other bays, one sensor each, are unaffected
    assert ups[-1].bays["a2"]["occupancy"] == "occupied"


def test_a_step_both_members_see_together_still_trips_the_fast_swap_rule():
    cfg, ups = _disagreeing_run(30, 4.0)
    before = ups[-1].estimates["a1"]["sigma"]
    swap = tick(
        cfg,
        ups[-1].memory,
        30.0 * cfg.dt,
        prox_a1=PROX_C + 12.0,
        prox_a1b=PROX_C - 4.0 + 12.0,
        smart={"A": {"temp_c": 44.0, "age_s": 0.0, "model": "M"}},
    )
    assert swap.estimates["a1"]["sigma"] > before + 1.0  # the margin opens at once
    assert swap.summary["smart_used"] == ups[-1].summary["smart_used"]  # that tick's SMART is held


# ---------------------------------------------------------------------------
# occupancy
# ---------------------------------------------------------------------------


@pytest.fixture
def as_empty():
    """``(cfg, memory)`` with bay ``b1`` settled ``empty`` (its sensor reads the air)."""

    def build(**estimator: Any):
        cfg = lcfg(empty_confirm_s=10.0, **estimator)
        ups = run_ticks(cfg, 100, prox_b1=SP + 0.1)
        assert ups[-1].bays["b1"]["occupancy"] == "empty"
        return cfg, ups[-1].memory

    return build


def test_occupancy_unknown_to_occupied_at_once_on_a_warm_sensor():
    cfg = lcfg()
    first = tick(cfg, None, 0.0)
    assert first.bays["a1"]["occupancy"] == "occupied"  # 5 degC above air
    assert first.bays["b1"]["occupancy"] == "occupied"
    assert first.bays["a2"]["declared"] is True and first.bays["c1"]["occupancy"] == "empty"


def test_occupancy_empty_after_confirm_then_insert_raises_sigma():
    cfg = lcfg(empty_confirm_s=10.0)
    air = SP
    ups = run_ticks(cfg, 30, prox_b1=air + 0.1)  # nothing in b1: its sensor reads air
    states = [up.bays["b1"]["occupancy"] for up in ups]
    assert states[0] == "unknown" and states[-1] == "empty"
    assert "b1" not in ups[-1].estimates  # an empty bay carries no constraint
    empty_since = states.index("empty")
    assert empty_since * cfg.dt >= cfg.estimator.empty_confirm_s
    # a drive goes in: the sensor warms; three ticks above occupied_dT_c
    ins = run_ticks(cfg, 6, mem=ups[-1].memory, t0=30.0, prox_b1=air + 4.0)
    seq = [up.bays["b1"]["occupancy"] for up in ins]
    assert seq[:2] == ["empty", "empty"] and seq[2] == "occupied"
    entry = ins[2].estimates["b1"]
    assert entry["sigma"] > 4.0  # reset variance 25 degC^2: a wide margin at once
    assert entry["margin"] == pytest.approx(cfg.estimator.k_sigma * entry["sigma"])


def test_occupancy_declarations_are_fixed():
    m = das_mapping()
    m["setpoints"] = {}
    m["topology"]["bays"]["b1"]["occupied"] = True
    m["estimator"] = {"empty_confirm_s": 2.0}
    cfg = MpcConfig.from_mapping(m)
    ups = run_ticks(cfg, 20, prox_b1=SP, prox_c1=SP + 6.0)
    assert all(up.bays["b1"]["occupancy"] == "occupied" for up in ups)
    assert all(up.bays["c1"]["occupancy"] == "empty" for up in ups)  # declared false


def test_a_bay_never_turns_empty_while_its_zone_air_is_unobserved():
    """Review finding: with the zone-air sensor lost, ``T_s - T_a`` is measured against a
    *predicted* air node. A rising inlet pulled that prediction onto the proximal reading,
    a warm idle drive (1.5 degC above air) was confirmed empty, and once the air sensor
    came back the bay stayed empty (1.5 < occupied_dT_c): its constraint was gone for good."""
    cfg = lcfg(empty_confirm_s=60.0)
    ups = run_ticks(cfg, 30, prox_b1=PROX_C)
    ups = run_ticks(cfg, 200, mem=ups[-1].memory, t0=30.0, prox_b1=SP + 1.5)
    assert ups[-1].bays["b1"]["occupancy"] == "occupied"
    lost = run_ticks(
        cfg, 300, mem=ups[-1].memory, t0=230.0, prox_b1=SP + 1.5, air_b=None, inlet=30.0
    )
    assert all(up.bays["b1"]["occupancy"] != "empty" for up in lost)
    back = run_ticks(cfg, 100, mem=lost[-1].memory, t0=530.0, prox_b1=SP + 1.5)
    assert all(up.bays["b1"]["occupancy"] != "empty" for up in back)
    assert "b1" in back[-1].estimates


def test_a_correlation_association_never_relaxes_the_drive_class():
    """Review finding: an association found by correlation is a statistical guess (a wrong
    pair is possible). Its SMART model moved an hdd bay to ssd_sata, raising the limit from
    50 to 65 degC. A guessed serial may only make the class stricter; a declared serial
    (the owner's statement) may relax it."""
    m = das_mapping()
    m["setpoints"] = {}
    m["drive_classes"] = {
        "hdd": {"limit_c": 50.0, "comfort_c": 5.0, "tau_d_s": 720.0, "models": ["^WD"]},
        "ssd_sata": {"limit_c": 65.0, "comfort_c": 10.0, "tau_d_s": 200.0, "models": ["^SSD"]},
    }
    m["topology"]["bays"]["b1"]["class"] = "hdd"
    m["topology"]["bays"]["a1"]["class"] = "ssd_sata"
    m["topology"]["bays"]["a2"]["class"] = "ssd_sata"
    cfg = MpcConfig.from_mapping(m)
    up = tick(cfg, None, 0.0)
    mem = json.loads(json.dumps(up.memory))
    mem["bays"]["b1"]["assoc"] = "GUESS"
    mem["bays"]["a2"]["assoc"] = "GUESS2"
    smart = {
        "GUESS": {"temp_c": 44.0, "age_s": 0.0, "model": "SSD 870"},
        "GUESS2": {"temp_c": 44.0, "age_s": 0.0, "model": "WD80EFZX"},
    }
    up = tick(cfg, mem, 1.0, smart=smart)
    assert up.bays["b1"]["association"] == "correlation"
    assert up.bays["b1"]["class"] == "hdd"
    assert up.estimates["b1"]["limit"] == 50.0
    assert up.bays["a2"]["association"] == "correlation"  # stricter: follows the drive
    assert up.bays["a2"]["class"] == "hdd" and up.estimates["a2"]["limit"] == 50.0


def _serial_topology(bay: str, serial: str) -> dict:
    topo = das_mapping()["topology"]
    topo["bays"][bay]["serial"] = serial
    return topo


def test_fresh_smart_of_an_associated_serial_keeps_a_bay_occupied():
    m = das_mapping()
    m["setpoints"] = {}
    m["topology"]["bays"]["b1"]["serial"] = "S1"
    m["estimator"] = {"empty_confirm_s": 2.0}
    cfg = MpcConfig.from_mapping(m)
    smart = {"S1": {"temp_c": 37.0, "age_s": 0.5, "model": "M"}}
    ups = run_ticks(cfg, 20, prox_b1=SP + 0.1, smart=smart)  # an idle drive at air temperature
    assert all(up.bays["b1"]["occupancy"] != "empty" for up in ups)
    silent = run_ticks(cfg, 20, mem=ups[-1].memory, t0=20.0, prox_b1=SP + 0.1)
    assert silent[-1].bays["b1"]["occupancy"] == "empty"


# ---------------------------------------------------------------------------
# SMART calibration
# ---------------------------------------------------------------------------


def test_calibration_rls_converges_on_excited_rows():
    rng = np.random.default_rng(3)
    entry = E._fresh_calibration(FLOOR_C)
    for k in range(200):
        x = 8.0 + 4.0 * math.sin(k / 7.0)
        y = 0.62 * x + 0.4 + rng.normal(0.0, 0.3)
        entry, _ = E.calibration_update(entry, x, y, float(k))
    assert entry["th"][0] == pytest.approx(0.62, abs=0.03)
    assert entry["th"][1] == pytest.approx(0.4, abs=0.3)
    assert entry["P"][0][0] < E.CAL_SLOPE_VAR_MAX and entry["fresh"] == 200
    assert E._calibrated(entry, 200.0, 86400.0)
    assert math.sqrt(entry["rms2"]) < 1.0


def test_calibration_rls_does_not_wind_up_on_a_constant_input_stream():
    entry = E._fresh_calibration(FLOOR_C)
    for k in range(2000):
        entry, _ = E.calibration_update(entry, 10.0, 0.7 * 10.0 - 2.1 + 0.5, float(k))
    slope, offset = entry["th"]
    assert E.CAL_SLOPE_BOUNDS[0] <= slope <= E.CAL_SLOPE_BOUNDS[1]
    assert abs(offset) < 10.0 and slope * 10.0 + offset == pytest.approx(5.4, abs=0.05)
    assert entry["P"][0][0] > E.CAL_SLOPE_VAR_MAX  # not identifiable: never accepted
    assert not E._calibrated(entry, 2000.0, 86400.0)
    p = np.array(entry["P"])
    assert np.all(np.linalg.eigvalsh(p) > 0)


def test_calibration_is_keyed_by_serial_within_the_bay():
    cfg_a = das_cfg(setpoints={}, topology=_serial_topology("a1", "A"))
    up = tick(cfg_a, None, 0.0)
    mem = json.loads(json.dumps(up.memory))
    entry = E._fresh_calibration(FLOOR_C)
    for k in range(40):
        x = 4.0 + 3.0 * math.sin(k)
        entry, _ = E.calibration_update(entry, x, 0.7 * x - 2.1, 0.0)
    entry["used"] = True
    mem["cal"]["a1"] = {"A": entry}
    up = tick(cfg_a, mem, 0.5)  # a declared serial needs no SMART this tick
    assert up.bays["a1"]["calibrated"] and up.bays["a1"]["sigma_cal_c"] < 1.5
    cfg_b = das_cfg(setpoints={}, topology=_serial_topology("a1", "B"))  # another drive
    up_b = tick(cfg_b, up.memory, 1.0)
    assert not up_b.bays["a1"]["calibrated"] and up_b.bays["a1"]["calibration"] is None
    assert up_b.bays["a1"]["sigma_cal_c"] == cfg_b.estimator.sigma_uncalibrated_c
    up_a = tick(cfg_a, up_b.memory, 1.5)  # the first drive is back in the bay
    assert up_a.bays["a1"]["calibrated"]


# ---------------------------------------------------------------------------
# manual calibration (POST /api/calibrate, PROJECT.md section 8 item 23)
# ---------------------------------------------------------------------------


def _manual(bay: str, temp: float, ts: float) -> dict:
    """``obs.inputs["calibration"]`` for one bay, the shape TickPlan.calibrations has."""
    return {bay: {"temp_c": temp, "ts": ts}}


def test_a_manual_calibration_is_the_same_sample_as_smart():
    """Item 23's contract: the same values through either door leave the estimator in
    the same state. The SMART run declares a serial on a1; the manual run declares
    none (the case the endpoint exists for) and names the bay instead."""
    readings = [(float(k) * 10.0, 44.0 + 0.4 * math.sin(k)) for k in range(1, 25)]

    cfg_smart = das_cfg(setpoints={}, topology=_serial_topology("a1", "A"))
    mem = None
    for sample_ts, temp in readings:
        up = tick(
            cfg_smart, mem, sample_ts, smart={"A": {"temp_c": temp, "age_s": 0.0, "model": None}}
        )
        mem = up.memory
    by_smart = up

    cfg_manual = das_cfg(setpoints={})
    mem = None
    for sample_ts, temp in readings:
        up = tick(cfg_manual, mem, sample_ts, calibration=_manual("a1", temp, sample_ts))
        mem = up.memory
    by_manual = up

    smart_entry = by_smart.memory["cal"]["a1"]["A"]
    manual_entry = by_manual.memory["manual"]["a1"]["cal"]
    assert manual_entry == smart_entry
    assert by_smart.summary["smart_used"] == by_manual.summary["manual_used"] == len(readings)
    # ... told apart in the running totals, so the SMART counters stay a SMART diagnostic
    assert (by_manual.summary["smart_used"], by_manual.summary["smart_rejected"]) == (0, 0)
    assert (by_smart.summary["manual_used"], by_smart.summary["manual_rejected"]) == (0, 0)
    # ... and so is everything the views show, apart from the serial and the source.
    shown = ("calibrated", "sigma_cal_c", "calibration", "occupancy")
    manual_view = {k: by_manual.bays["a1"][k] for k in shown}
    assert manual_view == {k: by_smart.bays["a1"][k] for k in shown}
    assert by_manual.bays["a1"]["calibration_source"] == "manual"
    assert by_smart.bays["a1"]["calibration_source"] == "smart"
    assert by_manual.bays["a1"]["serial"] is None
    assert by_manual.estimates["a1"] == by_smart.estimates["a1"]


def _manual_run(cfg: MpcConfig, ticks: int = 60, *, calibrate: bool = True):
    """``ticks`` ticks, each offering one handheld reading for bay a1 (or none)."""
    mem, up = None, None
    for k in range(1, ticks + 1):  # a moving reading identifies the slope
        ts = float(k) * 10.0
        temp = 44.0 + 3.0 * math.sin(0.7 * k)
        up = tick(
            cfg,
            mem,
            ts,
            calibration=_manual("a1", temp, ts) if calibrate else None,
            prox_a1=PROX_C,
        )
        mem = up.memory
    return up


def test_an_accepted_manual_calibration_is_the_map_the_filter_uses():
    cfg = das_cfg(setpoints={})
    up = _manual_run(cfg)
    info = up.bays["a1"]
    assert info["calibrated"] and info["calibration_source"] == "manual"
    assert info["calibration"]["accepted_once"] is True
    assert up.memory["manual"]["a1"]["cal"]["used"] is True
    assert up.memory["bays"]["a1"]["map"] == E.MANUAL_MAP  # the map in force, not the prior
    # sigma_cal is the calibration's own residual now, not the uncalibrated floor
    assert info["sigma_cal_c"] == pytest.approx(math.sqrt(info["calibration"]["rms_c"] ** 2))
    assert info["sigma_cal_c"] != FLOOR_C
    # ... and the estimate is a different number from the one the prior map gives
    bare = _manual_run(cfg, calibrate=False)
    assert bare.bays["a1"]["calibrated"] is False
    assert bare.bays["a1"]["sigma_cal_c"] == FLOOR_C
    assert abs(up.estimates["a1"]["t"] - bare.estimates["a1"]["t"]) > 1.0
    # the entry is the bay's own, never one of the model store's per-serial entries
    assert up.memory["cal"] == {}


def test_a_manual_calibration_far_from_the_estimate_is_rejected_and_counted():
    cfg = das_cfg(setpoints={})
    up = tick(cfg, None, 0.0)
    up = tick(cfg, up.memory, 10.0, calibration=_manual("a1", 90.0, 10.0))
    assert up.summary["manual_rejected"] == 1 and up.summary["manual_used"] == 0
    assert up.bays["a1"]["calibration"] is None
    assert up.bays["a1"]["calibration_source"] is None
    up = tick(cfg, up.memory, 20.0, calibration=_manual("a1", 45.0, 20.0))
    assert up.summary["manual_used"] == 1 and up.bays["a1"]["calibration"]["samples"] == 1
    # the same reading offered again on later ticks is not a new sample
    for ts in (30.0, 40.0):
        up = tick(cfg, up.memory, ts, calibration=_manual("a1", 45.0, 20.0))
    assert up.summary["manual_used"] == 1
    # neither door ever touches the other's totals
    assert (up.summary["smart_used"], up.summary["smart_rejected"]) == (0, 0)


def test_a_stale_or_malformed_manual_calibration_is_ignored_and_never_raises():
    cfg = das_cfg(setpoints={})
    assert cfg.estimator is not None
    up = tick(cfg, None, 0.0)
    ts = cfg.estimator.smart_max_age_s + 100.0
    for payload in (
        "text",
        [1, 2],
        {"a1": 41.0},
        {"nope": {"temp_c": 41.0, "ts": ts}},  # unknown bay
        {"a1": {"temp_c": float("nan"), "ts": ts}},
        {"a1": {"temp_c": 41.0}},  # no sample time
        {"a1": {"temp_c": 41.0, "ts": ts + 1.0}},  # in the future
        _manual("a1", 41.0, 0.0),  # older than smart_max_age_s
    ):
        out = tick(cfg, up.memory, ts, calibration=payload)
        assert out.summary["manual_fresh"] == []
        assert out.bays["a1"]["calibration"] is None


def test_a_manual_calibration_is_presence_evidence_like_smart():
    """An empty bay whose drive was measured by hand is occupied again at once."""
    cfg = das_cfg(setpoints={})
    mem = run_ticks(cfg, 3, prox_a1=None, prox_a1b=None)[-1].memory
    mem = copy.deepcopy(mem)
    mem["bays"]["a1"]["occ"] = E.EMPTY
    up = tick(cfg, mem, 100.0, calibration=_manual("a1", 44.0, 100.0))
    assert up.bays["a1"]["occupancy"] == E.OCCUPIED


def test_a_manual_calibration_expires_like_a_smart_one():
    cfg = das_cfg(setpoints={})
    m = cfg.to_dict()
    m["estimator"] = dict(m["estimator"], calibration_max_age_days=600.0 / 86400.0)
    short = MpcConfig.from_mapping(m)
    mem, up = None, None
    for k in range(1, 61):
        ts = float(k) * 10.0
        up = tick(short, mem, ts, calibration=_manual("a1", 44.0 + 3.0 * math.sin(0.7 * k), ts))
        mem = up.memory
    assert up.bays["a1"]["calibrated"]
    slope = up.bays["a1"]["calibration"]["slope"]
    silent = tick(short, up.memory, 60.0 * 10.0 + 700.0)
    assert not silent.bays["a1"]["calibrated"]
    assert silent.bays["a1"]["sigma_cal_c"] == FLOOR_C
    assert silent.bays["a1"]["calibration"]["slope"] == pytest.approx(slope)  # theta kept
    assert silent.bays["a1"]["calibration"]["fresh_samples"] == 0


def test_a_serials_calibration_wins_over_the_bays_manual_one():
    cfg = das_cfg(setpoints={}, topology=_serial_topology("a1", "A"))
    mem, up = None, None
    for k in range(1, 41):
        ts = float(k) * 10.0
        temp = 44.0 + 3.0 * math.sin(0.7 * k)
        up = tick(
            cfg,
            mem,
            ts,
            smart={"A": {"temp_c": temp, "age_s": 0.0, "model": None}},
            calibration=_manual("a1", temp + 6.0, ts),
        )
        mem = up.memory
    assert up.memory["manual"]["a1"]["cal"] is not None  # the manual entry is still kept
    assert up.bays["a1"]["calibration_source"] == "smart"
    assert up.bays["a1"]["calibration"]["slope"] == up.memory["cal"]["a1"]["A"]["th"][0]


def test_a_hand_calibrated_bay_keeps_its_map_until_the_serials_one_is_accepted():
    """A SMART entry in its first samples must not displace an accepted manual map.

    The trigger is ordinary -- the PC-side agent comes online, or POST /api/bay
    declares a serial, on a bay the owner has hand-calibrated. Falling back to the
    prior there would re-map the drive state through a map nothing fitted, on a bay
    whose proximal sensor is weakly coupled (which is why it was calibrated by hand).
    """
    plain = das_cfg(setpoints={})
    up = _manual_run(plain)  # 60 handheld readings: the manual map is in force
    assert up.bays["a1"]["calibration_source"] == "manual" and up.bays["a1"]["calibrated"]
    manual_slope = up.bays["a1"]["calibration"]["slope"]
    t_before = up.estimates["a1"]["t"]

    declared = das_cfg(setpoints={}, topology=_serial_topology("a1", "A"))
    mem, seen = up.memory, None
    for k in range(61, 76):  # the serial's own calibration starts from the prior
        ts = float(k) * 10.0
        temp = 44.0 + 3.0 * math.sin(0.7 * k)
        seen = tick(
            declared,
            mem,
            ts,
            smart={"A": {"temp_c": temp, "age_s": 0.0, "model": None}},
            prox_a1=PROX_C,
        )
        mem = seen.memory
        info = seen.bays["a1"]
        assert info["calibration_source"] == "manual" and info["calibrated"]
        assert info["sigma_cal_c"] != FLOOR_C
        assert info["calibration"]["slope"] == pytest.approx(manual_slope)
        assert mem["bays"]["a1"]["map"] == E.MANUAL_MAP  # never re-mapped through the prior
    assert seen is not None
    assert abs(seen.estimates["a1"]["t"] - t_before) < 1.0  # no step on the estimate
    assert mem["cal"]["a1"]["A"]["used"] is False  # the SMART entry is still collecting

    for k in range(76, 101):  # ... and once it is accepted the serial's map takes over
        ts = float(k) * 10.0
        temp = 44.0 + 3.0 * math.sin(0.7 * k)
        seen = tick(
            declared,
            mem,
            ts,
            smart={"A": {"temp_c": temp, "age_s": 0.0, "model": None}},
            prox_a1=PROX_C,
        )
        mem = seen.memory
    assert mem["cal"]["a1"]["A"]["used"] is True
    assert seen.bays["a1"]["calibration_source"] == "smart"
    assert seen.bays["a1"]["calibration"]["slope"] == mem["cal"]["a1"]["A"]["th"][0]
    assert mem["bays"]["a1"]["map"] == "A"


def test_smart_far_from_the_estimate_is_rejected_and_counted():
    cfg = das_cfg(setpoints={}, topology=_serial_topology("a1", "A"))
    up = tick(cfg, None, 0.0)
    far = {"A": {"temp_c": 90.0, "age_s": 0.0, "model": "M"}}
    up = tick(cfg, up.memory, 1.0, smart=far)
    assert up.summary["smart_rejected"] == 1 and up.summary["smart_used"] == 0
    assert up.bays["a1"]["calibration"] is None
    near = {"A": {"temp_c": 45.0, "age_s": 0.0, "model": "M"}}
    up = tick(cfg, up.memory, 5.0, smart=near)
    assert up.summary["smart_used"] == 1 and up.bays["a1"]["calibration"]["samples"] == 1
    # the same sample seen again is not a new one
    up = tick(cfg, up.memory, 6.0, smart={"A": dict(near["A"], age_s=1.0)})
    assert up.summary["smart_used"] == 1


def test_smart_model_regex_sets_the_class():
    m = das_mapping()
    m["setpoints"] = {}
    m["topology"]["bays"]["b1"]["serial"] = "S1"
    m["drive_classes"] = {
        "hdd": {"limit_c": 50.0, "comfort_c": 5.0, "tau_d_s": 720.0},
        "ssd_sata": {"limit_c": 65.0, "comfort_c": 10.0, "tau_d_s": 200.0, "models": ["^SSD"]},
    }
    cfg = MpcConfig.from_mapping(m)
    up = tick(cfg, None, 0.0)
    assert up.bays["b1"]["class"] == "hdd" and up.bays["b1"]["class_source"] == "default"
    up = tick(cfg, up.memory, 1.0, smart={"S1": {"temp_c": 44.0, "age_s": 0.0, "model": "SSD 870"}})
    assert up.bays["b1"]["class"] == "ssd_sata" and up.bays["b1"]["class_source"] == "smart_model"
    assert up.estimates["b1"]["limit"] == 65.0 and up.estimates["b1"]["class"] == "ssd_sata"
    hdd = tick(cfg, up.memory, 2.0, smart={"S1": {"temp_c": 44.0, "age_s": 0.0, "model": "WD80"}})
    assert hdd.bays["b1"]["class"] == "hdd"


# ---------------------------------------------------------------------------
# against the DAS truth simulator
# ---------------------------------------------------------------------------

BURSTS = {"burst_prob": 0.3, "burst_window_s": 300.0}


def test_uncalibrated_estimates_are_covered_by_k_sigma():
    """No SMART. With the sensors where the prior map assumes them the estimate is
    unbiased and covered by k * sigma; with them nearer the drive (no case offset, the
    basic truth preset) it errs high, never low beyond the margin."""
    cfg = example_cfg()
    for prior_placement in (True, False):
        plant = truth_plant(cfg, prior_placement=prior_placement, seed=4, **BURSTS)
        stats = {"n": 0, "covered": 0, "under": 0, "err": []}

        def on_tick(i, plant, up, stats=stats):
            if i < 24:
                return
            truth = plant.t_drive()
            for bay, entry in up.estimates.items():
                err = entry["t"] - truth[bay]
                stats["n"] += 1
                stats["covered"] += abs(err) <= entry["margin"]
                stats["under"] += err < -entry["margin"]
                stats["err"].append(err)

        run_truth(cfg, plant, 480, smart=False, on_tick=on_tick)
        mean = float(np.mean(stats["err"]))
        assert stats["under"] == 0, prior_placement
        if prior_placement:
            assert stats["covered"] >= 0.95 * stats["n"]
            assert abs(mean) < 0.5
        else:
            assert 2.0 < mean < 4.0  # the case offset the prior assumes but the truth lacks


def test_offset_free_estimates_settle_without_drift_under_model_mismatch():
    """Truth physics differ from the priors (conductances, capacities, fan strength); the
    integrating disturbances absorb it, so the model is stationary at the estimate."""
    cfg = example_cfg()
    topo = topology_from_config(cfg)
    for bay in topo["bays"].values():
        bay.update(g0_w_per_k=0.42, k_w_per_k=0.3, c_j_per_k=380.0)
    for fan in topo["fans"].values():
        fan["e_w_per_k"] = 26.0
    for zone in topo["zones"].values():
        zone.update(c_air_j_per_k=260.0, leak_w_per_k=2.0)
    plant = build_das_plant(topo, dt=cfg.dt, initial_pwm=0.5)
    u = dict.fromkeys(cfg.channels, 0.5)
    mem = None
    up = None
    for _ in range(600):
        obs = plant.observe()
        up = E.update(mem, cfg, temps=dict(obs.temps), u=u, ts=obs.ts)
        mem = up.memory
        plant.apply(u)
        plant.advance()
    assert max(z["drift_c_per_min"] for z in up.zones.values()) < 0.02
    truth = plant.t_drive()
    for bay, entry in up.estimates.items():
        assert entry["t"] - truth[bay] == pytest.approx(3.0, abs=1.0), bay  # the prior's offset


@pytest.fixture(scope="module")
def calibrated_run():
    """Declared serials on every bay, SMART on, activity bursts: 40 minutes."""
    cfg = example_cfg(**{f"b{i:02d}": f"SN{i:04d}" for i in range(1, 16)})
    plant = truth_plant(cfg, preset="basic", seed=9, **BURSTS)
    errs: list[float] = []

    def on_tick(i, plant, up):
        truth = plant.t_drive()
        for bay, entry in up.estimates.items():
            if entry["calibrated"]:
                drive = plant.drives[plant._bi[bay]]
                errs.append(entry["t"] - (truth[bay] + drive.smart_offset_c))

    up = run_truth(cfg, plant, 480, smart=True, on_tick=on_tick)
    return cfg, plant, up, errs


def test_smart_calibration_converges_and_calibrated_estimates_are_within_1c(calibrated_run):
    cfg, _, up, errs = calibrated_run
    calibrated = [b for b, info in up.bays.items() if info["calibrated"]]
    assert len(calibrated) >= 12, {b: info["calibration"] for b, info in up.bays.items()}
    for bay in calibrated:
        info = up.bays[bay]
        assert info["association"] == "declared"
        assert E.SIGMA_CAL_FLOOR_C <= info["sigma_cal_c"] < 1.0
        assert up.estimates[bay]["sigma"] < cfg.estimator.sigma_uncalibrated_c
    errs_arr = np.abs(np.array(errs))
    assert errs_arr.size > 1000
    assert np.mean(errs_arr <= 1.0) >= 0.95


def test_calibration_expires_without_samples_and_comes_back_with_them(calibrated_run):
    cfg, plant, up, _ = calibrated_run
    m = cfg.to_dict()
    m["estimator"] = dict(m["estimator"], calibration_max_age_days=600.0 / 86400.0)
    short = MpcConfig.from_mapping(m)
    floor = short.estimator.sigma_uncalibrated_c
    mem = copy.deepcopy(up.memory)
    slopes = {b: info["calibration"]["slope"] for b, info in up.bays.items() if info["calibrated"]}
    # the agent stops: no SMART for 15 minutes
    silent = run_truth(short, plant, 180, smart=False, mem=mem)
    for bay in slopes:
        info = silent.bays[bay]
        assert not info["calibrated"] and info["sigma_cal_c"] == floor
        assert info["calibration"]["slope"] == pytest.approx(slopes[bay])  # theta kept
        assert info["calibration"]["fresh_samples"] == 0
        assert silent.estimates[bay]["sigma"] >= floor
    # the agent is back: fresh samples confirm the calibration again
    back = run_truth(short, plant, 240, smart=True, mem=silent.memory)
    assert sum(1 for b in slopes if back.bays[b]["calibrated"]) >= len(slopes) // 2


# ---------------------------------------------------------------------------
# determinism, memory
# ---------------------------------------------------------------------------


def test_update_is_deterministic_json_round_trips_and_never_mutates_memory():
    cfg = lcfg()
    smart = {"X": {"temp_c": 41.0, "age_s": 3.0, "model": "M"}}
    mem = run_ticks(cfg, 10, smart=smart)[-1].memory
    frozen = json.loads(json.dumps(mem))
    a = tick(cfg, mem, 10.0, smart=smart, prox_b1=41.5)
    b = tick(cfg, mem, 10.0, smart=smart, prox_b1=41.5)
    assert json.loads(json.dumps(mem)) == frozen
    restored = tick(cfg, json.loads(json.dumps(mem)), 10.0, smart=smart, prox_b1=41.5)
    for other in (b, restored):
        assert json.dumps(other.memory, sort_keys=True) == json.dumps(a.memory, sort_keys=True)
        assert other.estimates == a.estimates and other.bays == a.bays


@pytest.mark.parametrize(
    "garbage",
    [
        None,
        {},
        {"v": 99},
        {"v": 1, "fp": "other"},
        "text",
        [1, 2],
    ],
)
def test_malformed_memory_starts_over(garbage):
    cfg = lcfg()
    up = tick(cfg, garbage, 0.0)
    assert set(up.estimates) == {"a1", "a2", "b1"}


def test_corrupt_values_inside_a_valid_memory_start_over():
    cfg = lcfg()
    mem = run_ticks(cfg, 3)[-1].memory
    for path, value in (
        (("zones", "za", "x"), [1.0]),
        (("zones", "za", "P"), "nope"),
        (("bays", "a1", "occ"), "maybe"),
        (("cal",), {"a1": {"S": {"th": [1]}}}),
        (("smart",), {"S": {"ts": "x"}}),
    ):
        bad = json.loads(json.dumps(mem))
        node = bad
        for key in path[:-1]:
            node = node[key]
        node[path[-1]] = value
        up = tick(cfg, bad, 3.0)
        assert set(up.estimates) == {"a1", "a2", "b1"}


# ---------------------------------------------------------------------------
# in mpc.step
# ---------------------------------------------------------------------------


def test_step_runs_the_estimator_on_fault_ticks_too():
    cfg = lcfg()
    state = MpcState.cold()
    for i in range(3):
        _, state = checked_step(das_obs(cfg, float(i)), cfg, state)
    cmd, state = checked_step(das_obs(cfg, 3.0, air_b=None), cfg, state)
    assert cmd.mode is Mode.DEGRADED
    assert cmd.diagnostics["estimator"]["status"] == "ok"
    assert state.solver_memory["estimator"]["ts"] == 3.0
    assert cmd.diagnostics["estimates"]["b1"]["zone_trusted"] is False


def test_step_passes_smart_inputs_to_the_estimator():
    m = das_mapping()
    m["setpoints"] = {}
    m["topology"]["bays"]["b1"]["serial"] = "S1"
    cfg = MpcConfig.from_mapping(m)
    obs = das_obs(cfg, 0.0)
    obs = type(obs)(
        temps=obs.temps,
        rpm=obs.rpm,
        pwm=obs.pwm,
        ts=0.0,
        inputs={"smart": {"S1": {"temp_c": 44.0, "age_s": 2.0, "model": "M"}}},
    )
    cmd, _ = checked_step(obs, cfg, MpcState.cold())
    assert cmd.diagnostics["bays"]["b1"]["serial"] == "S1"
    assert cmd.diagnostics["estimator"]["smart_used"] == 1


def test_estimator_error_faults_the_zones_it_serves_and_resets(monkeypatch):
    cfg = lcfg()
    state = MpcState.cold()
    for i in range(3):
        _, state = checked_step(das_obs(cfg, float(i)), cfg, state)
    assert "estimator" in state.solver_memory

    def broken(*args, **kwargs):
        raise FloatingPointError("boom")

    monkeypatch.setattr(E, "update", broken)
    cmd, state = checked_step(das_obs(cfg, 3.0), cfg, state)
    assert cmd.diagnostics["estimator"] == {
        "status": "error",
        "error": "FloatingPointError: boom",
        "zones": {},
    }
    faulted = set(cmd.diagnostics["zones_in_fault"])
    assert faulted == {"za", "zb"}  # zc holds only a declared-empty bay
    assert cmd.mode is Mode.DEGRADED
    assert state.zone_faults["za"].reason is FaultReason.SOLVER
    assert "estimator:FloatingPointError: boom" in cmd.diagnostics["zones"]["za"]["reasons"]
    assert "estimator" not in state.solver_memory
    assert cmd.diagnostics["estimates"]["a1"]["source"] == est.SOURCE_PRIOR_MAP
    monkeypatch.undo()
    for i in range(4, 4 + cfg.confirm_ticks + 1):
        cmd, state = checked_step(das_obs(cfg, float(i), pwm=cmd.pwm), cfg, state)
    assert cmd.mode is Mode.AUTO and cmd.diagnostics["estimator"]["status"] == "ok"


def test_estimator_error_on_a_setpoint_config_is_only_reported(monkeypatch):
    cfg = das_cfg()  # zoned, still regulating on setpoints

    def broken(*args, **kwargs):
        raise ValueError("boom")

    monkeypatch.setattr(E, "update", broken)
    cmd, _ = step(das_obs(cfg, 0.0), cfg, MpcState.cold())
    assert cmd.mode is Mode.AUTO and cmd.diagnostics["estimator"]["status"] == "error"


# ---------------------------------------------------------------------------
# hot swap: which bays report ``swapped`` (item 12)
# ---------------------------------------------------------------------------


def test_swapped_is_a_step_of_the_bay_s_mean_reading_not_of_one_sensor():
    """Two members at different placements disagree, and the per-sensor fast-swap test is
    sequential, so one of them jumps on almost every tick; the mean of the pair does not
    move, and it is the mean that says a drive changed (item 12)."""
    cfg = lcfg()
    mem = run_ticks(cfg, 30)[-1].memory
    one = tick(cfg, mem, 30.0, prox_a1=PROX_C + 1.0)  # a disagreement, not a swap
    assert one.memory["bays"]["a1"]["occ"] == "occupied"
    assert one.bays["a1"]["swapped"] is False
    both = tick(cfg, mem, 30.0, prox_a1=PROX_C + 1.0, prox_a1b=PROX_C + 1.0)
    assert both.bays["a1"]["swapped"] is True  # both members moved: a drive changed
    assert all(not up.bays[b]["swapped"] for up in (one, both) for b in ("a2", "b1"))


def test_a_standing_gap_between_two_members_is_not_a_swap_on_any_tick():
    """Item 109. The mean of the readings is only *where the drive is* once every member
    is predicted where the filter says it sits: on the fused layout that is the bay's node
    plus the member's own placement offset. Comparing the mean reading with the bare node
    left half the pair's disagreement in the statistic, so a pair more than
    ``2 * jump_min_c`` apart was reported swapped on every settled tick -- and the gap is
    ``ds * (T_d - T_a) + db``, which grows with the load.
    """
    _, ups = _disagreeing_run(60, 4.0)
    assert not any(up.bays["a1"]["swapped"] for up in ups)
    assert not any(up.bays[b]["swapped"] for up in ups for b in ("a2", "b1"))


def test_a_step_both_members_of_a_disagreeing_pair_see_is_still_a_swap():
    """The other direction of item 109: the gap cancels in the mean, a step both members
    take does not. Same tick, same run, no waiting."""
    cfg, ups = _disagreeing_run(30, 4.0)
    swap = tick(
        cfg,
        ups[-1].memory,
        30.0 * cfg.dt,
        prox_a1=PROX_C + 6.0,
        prox_a1b=PROX_C - 4.0 + 6.0,
        smart={"A": {"temp_c": 44.0, "age_s": 0.0, "model": "M"}},
    )
    assert swap.bays["a1"]["swapped"] is True


#: ``rich`` seeds on which at least one of the DAS example's two redundant proximal pairs
#: (b03, b10) sits far enough apart for the placement gap to cross the fast-swap rule's
#: thresholds. Before item 109 that bay was reported ``swapped`` on 1073 to 1199 of these
#: 1200 ticks, so ``model_reset_on_swap`` restarted its zone's regression on nearly every
#: tick and the zone could never converge (item 102's dead zones).
REDUNDANT_SEEDS = (2, 3, 4, 5, 7)


@pytest.mark.parametrize("seed", REDUNDANT_SEEDS)
def test_a_redundant_proximal_pair_is_never_reported_swapped(seed):
    """Item 109 on the truth simulator: realistic placements (``beta`` 0.3 +- 0.05, offset
    -2.1 +- 0.3 degC drawn per sensor), the fans stepped every 120 ticks so the
    drive-to-air rise moves, and no drive ever leaves a bay. Nothing is a swap."""
    cfg = example_cfg()
    plant = truth_plant(cfg, preset="rich", seed=seed)
    counts: dict[str, int] = {}

    def on_tick(i, plant, up, counts=counts):
        for bay, block in up.bays.items():
            if block["swapped"]:
                counts[bay] = counts.get(bay, 0) + 1

    run_truth(cfg, plant, 1200, smart=True, on_tick=on_tick)
    assert counts == {}


def _hot_swap(bay: str, *, seed: int = 3, swap_at: int = 600, step_c: float = 12.0):
    """Replace the drive in ``bay`` in place at ``swap_at`` by one ``step_c`` hotter, on
    the same plant the false-positive test uses, and report every tick the bay called
    ``swapped``: those before the step, which are false, and those after, the first of
    which is the detection latency."""
    cfg = example_cfg()
    plant = truth_plant(cfg, preset="rich", seed=seed)
    before: list[int] = []
    after: list[int] = []

    def on_tick(i, plant, up):
        if up.bays[bay]["swapped"]:
            (after if i > swap_at else before).append(i - swap_at)
        if i == swap_at:
            drive = plant.drives[plant._bi[bay]]
            plant.insert(bay, drive, temp_c=plant.t_drive()[bay] + step_c)

    run_truth(cfg, plant, swap_at + 30, smart=True, on_tick=on_tick)
    return before, after


def test_a_hot_swap_is_caught_as_fast_with_a_redundant_pair_as_without():
    """The cost side of item 109, measured as a comparison and not against a constant: a
    drive replaced in place by one 12 degC hotter is caught no later on a bay with a
    redundant pair at different placements (b03, b10) than on the single-sensor b02 of
    the same run. Each bay is swapped in its own run of the same plant, and each must be
    *silent until the swap* -- a bay standing in alarm would otherwise "detect" it on the
    first tick without detecting anything, which is exactly what the defect did and what
    the latency of a pre-fix b03 was.
    """
    latency: dict[str, int] = {}
    for bay in ("b02", "b03", "b10"):
        before, after = _hot_swap(bay)
        assert not before, (bay, len(before), before[:8])  # nothing but the swap is one
        assert after, bay
        latency[bay] = after[0]
    assert latency["b02"] <= 2, latency  # the reference is itself prompt
    assert latency["b03"] <= latency["b02"], latency
    assert latency["b10"] <= latency["b02"], latency


def test_swapped_on_the_edge_of_empty(as_empty):
    cfg, mem = as_empty()
    quiet = run_ticks(cfg, 3, mem=mem, t0=100.0, prox_b1=SP + 0.1)
    assert all(not up.bays["b1"]["swapped"] for up in quiet)
    ins = run_ticks(cfg, 5, mem=quiet[-1].memory, t0=103.0, prox_b1=SP + 4.0)
    states = [up.bays["b1"]["occupancy"] for up in ins]
    swaps = [up.bays["b1"]["swapped"] for up in ins]
    assert "occupied" in states and states.index("occupied") == swaps.index(True)
    assert sum(swaps) == 1  # the one tick the bay filled


def test_a_hot_swap_resets_only_that_bay_s_thermal_coefficients():
    """The whole chain through ``step``: the estimator's ``swapped`` reaches
    ``thermal.update`` and the bay's identification starts over (item 12)."""
    cfg = dataclasses.replace(lcfg(empty_confirm_s=10.0), model_shadow=True)
    state = MpcState.cold()
    ts = 0.0
    for _ in range(40):  # nothing in b1: its sensor reads the air, the bay turns empty
        _, state = step(das_obs(cfg, ts, prox_b1=SP + 0.1), cfg, state)
        ts += cfg.dt
    for b in ("b1", "a1"):
        state.solver_memory["thermal"]["bays"][b]["w"] = 11
    swapped = []
    for k in range(6):  # a drive goes in: the sensor warms within the gate's slew limit
        value = min(SP + 0.1 + 1.5 * (k + 1), SP + 4.5)
        cmd, state = step(das_obs(cfg, ts, prox_b1=value), cfg, state)
        swapped.append(cmd.diagnostics["bays"]["b1"]["swapped"])
        ts += cfg.dt
    assert cmd.diagnostics["bays"]["b1"]["occupancy"] == "occupied"
    assert swapped[0] is False and any(swapped), swapped  # the insert, not the quiet bay
    assert cmd.diagnostics["thermal"]["bays"]["b1"]["windows"] == 0
    assert cmd.diagnostics["thermal"]["bays"]["a1"]["windows"] == 11


def test_a_swap_drops_the_bays_manual_calibration():
    """Items 12 and 104: a hand-measured map is keyed by *bay*, so on a swap it would
    follow the slot rather than the drive and nothing later would notice. The tick the
    swap rule fires drops it, and the bay falls back to the prior map and the
    uncalibrated sigma -- more margin, never less cooling."""
    cfg = das_cfg(setpoints={})
    up = _manual_run(cfg)
    assert up.bays["a1"]["calibration_source"] == "manual"
    assert up.memory["bays"]["a1"]["occ"] == E.OCCUPIED

    # the drive is pulled: the owner declares the bay empty, so the occupancy crosses
    # the ``empty`` boundary on the next tick and that is a swap
    data = das_mapping()
    data["setpoints"] = {}
    data["topology"]["bays"]["a1"]["occupied"] = False
    after = MpcConfig.from_mapping(data)
    out = tick(after, up.memory, 10_000.0, prox_a1=PROX_C)
    assert out.bays["a1"]["swapped"] is True
    assert out.memory["manual"] == {}
    assert out.bays["a1"]["calibration"] is None
    assert out.bays["a1"]["calibration_source"] is None

    # and a bay the swap rule never fired on keeps what it had
    out2 = tick(cfg, up.memory, 10_000.0, prox_a1=PROX_C)
    assert out2.bays["a1"]["swapped"] is False
    assert set(out2.memory["manual"]) == {"a1"}


def test_a_bare_proximal_step_inflates_the_manual_map_instead_of_dropping_it():
    """The riskier half of the swap rule (items 12 and 104). ``stepped`` is one tick's
    innovation against the predicted node -- a spin-up, an I/O burst or a fan step raises
    it as readily as a swap does -- so it must not delete twenty readings the owner took
    by hand. It inflates them instead: ``sigma_cal`` doubled until ``confirm`` further
    accepted hand readings clear it, which is still the conservative direction."""
    cfg = das_cfg(setpoints={})
    up = _manual_run(cfg)
    before = up.bays["a1"]["sigma_cal_c"]
    assert up.bays["a1"]["calibration_source"] == "manual"
    assert "inflate" not in up.memory["manual"]["a1"]["cal"]

    out = tick(cfg, up.memory, 610.0, prox_a1=PROX_C + 6.0)
    assert out.bays["a1"]["swapped"] is True  # the fast-swap step fired ...
    assert out.bays["a1"]["occupancy"] == "occupied"  # ... with no crossing of ``empty``
    entry = out.memory["manual"]["a1"]["cal"]  # the map is still there
    assert entry["inflate"] == E.STALE_SIGMA_CAL_FACTOR
    assert entry["confirm"] == E.CAL_MIN_SAMPLES
    assert out.bays["a1"]["calibration_source"] == "manual"
    assert out.bays["a1"]["sigma_cal_c"] == pytest.approx(before * E.STALE_SIGMA_CAL_FACTOR)


def test_restore_manual_calibration_is_pure_and_never_raises_on_rubbish(cfg: MpcConfig):
    """The estimator half of item 104: a legacy config is the only thing that raises."""
    das = das_cfg(setpoints={})
    for rubbish in (None, 5, "x", [1], {"a1": 7}, {"nope": {}}):
        mem, warnings = E.restore_manual_calibration(None, rubbish, das, ts=0.0)
        assert mem["manual"] == {}
        assert isinstance(warnings, list)
    with pytest.raises(ValueError, match="topology"):
        E.restore_manual_calibration(None, {}, cfg, ts=0.0)


def test_a_live_manual_entry_wins_over_a_stored_one():
    """A restore only fills gaps: whatever this run has already measured stays."""
    cfg = das_cfg(setpoints={})
    up = _manual_run(cfg)
    stored = {
        "a1": {
            "th": [0.4, 9.0],
            "P": [[0.004, 0.0], [0.0, 0.5]],
            "n": 40,
            "fresh": 30,
            "rms2": 0.36,
            "used": True,
            "age_s": 60.0,
            "declared": cfg.bay_declaration("a1"),
        }
    }
    mem, warnings = E.restore_manual_calibration(up.memory, stored, cfg, ts=1000.0)
    assert mem["manual"]["a1"]["cal"]["th"] != [0.4, 9.0]
    assert warnings == []
