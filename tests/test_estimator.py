"""Drive temperature estimator (``aqua_bridge.control.estimator``, plan sections 2 and 6).

Config section, numerics (transition, Joseph form), the filter against the DAS
truth simulator (:mod:`aqua_bridge.sim.das`) with and without SMART, SMART
calibration (convergence, windup, serial keying, expiry), the occupancy
machine, determinism and the integration in ``mpc.step``.
"""

from __future__ import annotations

import copy
import json
import math
from typing import Any

import numpy as np
import pytest
from hypothesis import given, settings
from hypothesis import strategies as st

from aqua_bridge.config import load_config
from aqua_bridge.control import estimates as est
from aqua_bridge.control import estimator as E
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


def lcfg(**estimator: Any) -> MpcConfig:
    """The small zoned fixture without setpoints (limit regulation), estimator overrides."""
    return das_cfg(setpoints={}, **({"estimator": estimator} if estimator else {}))


def trusted(cfg: MpcConfig, **overrides: float | None) -> dict[str, float]:
    values = {k: v for k, v in default_temps(cfg).items() if v is not None}
    values.update(overrides)
    return {k: float(v) for k, v in values.items() if v is not None}


def tick(cfg: MpcConfig, mem, ts: float, *, u: float = 0.5, smart=None, **temps):
    return E.update(
        mem, cfg, temps=trusted(cfg, **temps), u=dict.fromkeys(cfg.channels, u), ts=ts, smart=smart
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
        {"sensor_noise_c": -1.0},
        {"smart_max_age_s": 0.5},  # < dt
        {"occupied_dT_c": 0.5},  # not above empty_dT_c
        {"empty_dT_c": 0.0},
        {"empty_confirm_s": 1.0},  # < 2 dt
        {"bay_settle_s": -1.0},
        {"calibration_max_age_days": 0.0},
        {"associate_window_s": 599.0},
        {"associate_min_corr": 1.0},
        {"associate_margin": 0.0},
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
@given(data=st.data())
def test_joseph_form_keeps_every_covariance_symmetric_and_psd(data):
    cfg = lcfg()
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
            assert math.isfinite(entry["t"]) and entry["sigma"] >= est.SIGMA_UNCALIBRATED_C
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
    assert sigmas[-1] < sigmas[0] and sigmas[-1] >= est.SIGMA_UNCALIBRATED_C
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
    cfg = lcfg()
    mem = run_ticks(cfg, 30)[-1].memory
    ups = run_ticks(cfg, 30, mem=mem, t0=30.0, prox_a1=None)
    assert all(up.bays["a1"]["occupancy"] == "occupied" for up in ups)
    ups = run_ticks(cfg, 5, mem=ups[-1].memory, t0=60.0, prox_a1=None, prox_a1b=None)
    assert all(up.bays["a1"]["occupancy"] == "unknown" for up in ups)  # observability lost
    assert "a1" in ups[-1].estimates  # still constrained


# ---------------------------------------------------------------------------
# occupancy
# ---------------------------------------------------------------------------


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
    entry = E._fresh_calibration()
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
    entry = E._fresh_calibration()
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
    entry = E._fresh_calibration()
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
    assert up_b.bays["a1"]["sigma_cal_c"] == est.SIGMA_UNCALIBRATED_C
    up_a = tick(cfg_a, up_b.memory, 1.5)  # the first drive is back in the bay
    assert up_a.bays["a1"]["calibrated"]


def _serial_topology(bay: str, serial: str) -> dict:
    topo = das_mapping()["topology"]
    topo["bays"][bay]["serial"] = serial
    return topo


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
    _, _, up, errs = calibrated_run
    calibrated = [b for b, info in up.bays.items() if info["calibrated"]]
    assert len(calibrated) >= 12, {b: info["calibration"] for b, info in up.bays.items()}
    for bay in calibrated:
        info = up.bays[bay]
        assert info["association"] == "declared"
        assert E.SIGMA_CAL_FLOOR_C <= info["sigma_cal_c"] < 1.0
        assert up.estimates[bay]["sigma"] < est.SIGMA_UNCALIBRATED_C
    errs_arr = np.abs(np.array(errs))
    assert errs_arr.size > 1000
    assert np.mean(errs_arr <= 1.0) >= 0.95


def test_calibration_expires_without_samples_and_comes_back_with_them(calibrated_run):
    cfg, plant, up, _ = calibrated_run
    m = cfg.to_dict()
    m["estimator"] = dict(m["estimator"], calibration_max_age_days=600.0 / 86400.0)
    short = MpcConfig.from_mapping(m)
    mem = copy.deepcopy(up.memory)
    slopes = {b: info["calibration"]["slope"] for b, info in up.bays.items() if info["calibrated"]}
    # the agent stops: no SMART for 15 minutes
    silent = run_truth(short, plant, 180, smart=False, mem=mem)
    for bay in slopes:
        info = silent.bays[bay]
        assert not info["calibrated"] and info["sigma_cal_c"] == est.SIGMA_UNCALIBRATED_C
        assert info["calibration"]["slope"] == pytest.approx(slopes[bay])  # theta kept
        assert info["calibration"]["fresh_samples"] == 0
        assert silent.estimates[bay]["sigma"] >= est.SIGMA_UNCALIBRATED_C
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
