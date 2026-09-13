"""Serial -> bay association by correlation (``aqua_bridge.control.associate``, plan section 1).

Unit tests of the pure helpers (series recording, detrended correlation, greedy
assignment with margin, confirmation), the drop rules inside the estimator, and
one run on the DAS truth simulator (:mod:`aqua_bridge.sim.das`): every bay's
drive reports SMART by serial and nothing says which bay a serial sits in.
"""

from __future__ import annotations

import copy
import json
import math

import numpy as np
import pytest

from aqua_bridge.config import load_config
from aqua_bridge.control import associate as A
from aqua_bridge.control import estimator as E
from aqua_bridge.model import MpcConfig
from aqua_bridge.sim.das import SENSOR_TYPES, build_das_plant, topology_from_config
from conftest import EXAMPLE_DAS_CONFIG
from das_fixtures import das_cfg, default_temps

# ---------------------------------------------------------------------------
# series and SMART histories
# ---------------------------------------------------------------------------


def test_record_series_samples_every_period_and_trims_to_the_window():
    series = None
    for k in range(200):
        ts = 5.0 * k
        before = copy.deepcopy(series)
        series = A.record_series(series, ts, {"b1": float(k), "b2": None}, {"z": 30.0}, 600.0)
        if before is not None:
            assert json.dumps(before) == json.dumps(before)  # still serialisable
    t = series["t"]
    assert all(b - a == pytest.approx(A.SAMPLE_S) for a, b in zip(t, t[1:], strict=False))
    assert t[-1] - t[0] <= 600.0
    assert len(series["y"]["b1"]) == len(t) == len(series["a"]["z"])
    assert series["y"]["b2"] == [None] * len(t)


def test_record_series_never_mutates_its_input_and_restarts_when_inconsistent():
    series = A.record_series(None, 0.0, {"b1": 1.0}, {"z": 30.0}, 3600.0)
    frozen = copy.deepcopy(series)
    same = A.record_series(series, 10.0, {"b1": 2.0}, {"z": 30.0}, 3600.0)
    assert same is series and series == frozen  # nothing due: handed back unchanged
    grown = A.record_series(series, 40.0, {"b1": 2.0}, {"z": 30.0}, 3600.0)
    assert series == frozen and grown["t"] == [0.0, 40.0]
    other_bays = A.record_series(grown, 80.0, {"b9": 2.0}, {"z": 30.0}, 3600.0)
    assert other_bays["t"] == [80.0] and set(other_bays["y"]) == {"b9"}
    backwards = A.record_series(grown, 10.0, {"b1": 2.0}, {"z": 30.0}, 3600.0)
    assert backwards["t"] == [10.0]
    garbage = A.record_series({"t": "x", "y": 3}, 10.0, {"b1": 2.0}, {"z": 30.0}, 3600.0)
    assert garbage["t"] == [10.0]


def test_record_smart_trims_and_drops_malformed_points():
    hist = [[0.0, 40.0], ["bad", 1.0], [100.0, math.nan], [3000.0, 41.0]]
    out = A.record_smart(hist, 3700.0, 42.0, now=3700.0, window_s=3600.0)
    assert out == [[3000.0, 41.0], [3700.0, 42.0]]
    assert hist[0] == [0.0, 40.0]


# ---------------------------------------------------------------------------
# correlation and assignment
# ---------------------------------------------------------------------------


def test_correlation_is_taken_after_removing_linear_trends():
    t = np.arange(40.0) * 60.0
    burst = np.sin(t / 300.0)
    assert A.correlation(burst + 0.01 * t, 2.0 * burst - 0.03 * t, t) == pytest.approx(1.0)
    assert A.correlation(burst, -burst, t) == pytest.approx(-1.0)
    # two pure trends carry no evidence
    assert A.correlation(0.01 * t, 0.02 * t, t) is None
    assert A.correlation(burst[:10], burst[:10], t[:10]) is None  # too few samples


def test_assign_accepts_clear_pairs_and_refuses_ambiguous_ones():
    scores = {
        "S1": {"b1": 0.95, "b2": 0.30, "b3": 0.10},
        "S2": {"b1": 0.20, "b2": 0.91, "b3": 0.85},  # b2 vs b3 too close
        "S3": {"b3": 0.70},  # below min_corr
    }
    assert A.assign(scores, 0.8, 0.15) == {"b1": "S1"}
    # the bay's runner-up counts too: S4 correlates with b1 almost as well as S1
    scores["S4"] = {"b1": 0.9}
    assert A.assign(scores, 0.8, 0.15) == {}
    # a lone candidate needs only min_corr (the runner-up floor is 0)
    assert A.assign({"S": {"b": 0.82}}, 0.8, 0.15) == {"b": "S"}


def test_assign_is_deterministic_under_ties():
    scores = {"S2": {"b1": 0.9}, "S1": {"b2": 0.9}}
    assert A.assign(scores, 0.8, 0.15) == {"b1": "S2", "b2": "S1"}
    assert A.assign(dict(reversed(list(scores.items()))), 0.8, 0.15) == {"b1": "S2", "b2": "S1"}


def test_confirm_needs_consecutive_acceptances():
    pending: dict = {}
    for k in range(A.CONFIRM_EVALUATIONS - 1):
        pending, confirmed = A.confirm(pending, {"b1": "S1"})
        assert confirmed == {} and pending == {"b1": ["S1", k + 1]}
    pending, confirmed = A.confirm(pending, {"b1": "S1"})
    assert confirmed == {"b1": "S1"} and pending == {}
    pending, _ = A.confirm({"b1": ["S1", 2]}, {"b1": "S2"})  # another serial starts over
    assert pending == {"b1": ["S2", 1]}
    pending, _ = A.confirm({"b1": ["S1", 2]}, {})  # not accepted now: forgotten
    assert pending == {}


def test_score_matrix_needs_a_history_that_spans_the_window():
    t_rec = [30.0 * k for k in range(130)]
    y = [math.sin(t / 400.0) for t in t_rec]
    series = {"t": t_rec, "y": {"b1": y}, "a": {"z": [30.0] * len(t_rec)}}
    hist = [[t + 1.0, 30.0 + 5.0 * math.sin((t + 1.0) / 400.0)] for t in t_rec[::2]]
    full = A.score_matrix(series, {"S": hist}, ["S"], ["b1"], {"b1": "z"}, min_span_s=3240.0)
    assert full["S"]["b1"] > 0.99
    short = A.score_matrix(series, {"S": hist[-25:]}, ["S"], ["b1"], {"b1": "z"}, min_span_s=3240.0)
    assert short == {}
    broken = dict(series, y={"b1": ["x"] * len(t_rec)})
    assert A.score_matrix(broken, {"S": hist}, ["S"], ["b1"], {"b1": "z"}) == {}


# ---------------------------------------------------------------------------
# drop rules inside the estimator
# ---------------------------------------------------------------------------


def _tick(cfg: MpcConfig, mem, ts: float, smart, **temps):
    values = {k: v for k, v in default_temps(cfg).items() if v is not None}
    values.update(temps)
    values = {k: v for k, v in values.items() if v is not None}
    return E.update(mem, cfg, temps=values, u=dict.fromkeys(cfg.channels, 0.5), ts=ts, smart=smart)


def _with_association(cfg: MpcConfig, bay: str, serial: str):
    """An estimator memory in which ``bay`` holds ``serial`` by correlation."""
    up = _tick(cfg, None, 0.0, {serial: {"temp_c": 40.0, "age_s": 0.0, "model": "M"}})
    mem = json.loads(json.dumps(up.memory))
    mem["bays"][bay]["assoc"] = serial
    mem["bays"][bay]["occ"] = "occupied"
    return mem


def test_association_drops_when_the_serial_goes_silent():
    cfg = das_cfg(setpoints={})
    mem = _with_association(cfg, "a1", "S1")
    up = _tick(cfg, mem, 1.0, {"S1": {"temp_c": 40.0, "age_s": 1.0, "model": "M"}})
    assert up.bays["a1"]["serial"] == "S1" and up.bays["a1"]["association"] == "correlation"
    silent = {"S1": {"temp_c": 40.0, "age_s": cfg.estimator.smart_max_age_s + 1.0, "model": "M"}}
    up = _tick(cfg, up.memory, 2.0, silent)
    assert up.bays["a1"]["serial"] is None and up.memory["bays"]["a1"]["assoc"] is None


def test_association_drops_when_the_proximal_sensor_jumps():
    cfg = das_cfg(setpoints={})
    mem = _with_association(cfg, "a1", "S1")
    smart = {"S1": {"temp_c": 40.0, "age_s": 1.0, "model": "M"}}
    up = _tick(cfg, mem, 1.0, smart)
    assert up.bays["a1"]["serial"] == "S1"
    up = _tick(cfg, up.memory, 2.0, smart, prox_a1=33.0, prox_a1b=33.0)  # drive pulled
    assert up.bays["a1"]["serial"] is None


def test_association_drops_when_the_bay_becomes_empty():
    """A fresh associated serial keeps an ``auto`` bay from turning empty (SMART presence
    is evidence), so the change into ``empty`` that drops it is a declaration."""
    cfg = das_cfg(setpoints={})
    mem = _with_association(cfg, "b1", "S1")
    smart = {"S1": {"temp_c": 40.0, "age_s": 1.0, "model": "M"}}
    up = _tick(cfg, mem, 1.0, smart)
    assert up.bays["b1"]["serial"] == "S1"
    m = cfg.to_dict()
    m["topology"]["bays"]["b1"]["occupied"] = False
    declared_empty = MpcConfig.from_mapping(m)
    up = _tick(declared_empty, up.memory, 2.0, smart)
    assert up.bays["b1"]["occupancy"] == "empty" and up.memory["bays"]["b1"]["assoc"] is None
    up = _tick(cfg, up.memory, 3.0, smart)  # back to auto: no association until re-scored
    assert up.bays["b1"]["serial"] is None


def test_a_declared_serial_wins_and_is_never_correlated():
    m = das_cfg(setpoints={}).to_dict()
    m["topology"]["bays"]["a2"]["serial"] = "S1"
    cfg = MpcConfig.from_mapping(m)
    mem = _with_association(cfg, "a1", "S1")  # a stale correlation says a1
    up = _tick(cfg, mem, 1.0, {"S1": {"temp_c": 40.0, "age_s": 1.0, "model": "M"}})
    assert up.bays["a2"]["serial"] == "S1" and up.bays["a2"]["association"] == "declared"
    assert up.bays["a1"]["serial"] is None
    assert "S1" not in up.summary["unassigned"]


# ---------------------------------------------------------------------------
# the DAS truth simulator
# ---------------------------------------------------------------------------

#: b09 and b10 (zone z2, identical sensors) run the same activity schedule, so their
#: SMART series cannot be told apart; b05's drive is pulled at REMOVE_AT.
SAME_SCHEDULE = [(0.0, 0.0), (900.0, 1.0), (1500.0, 0.0), (2400.0, 1.0), (3000.0, 0.0)]
SAME_SCHEDULE += [(3900.0, 1.0), (4500.0, 0.0)]
REMOVE_AT = 4700.0


@pytest.fixture(scope="module")
def truth_run():
    base = load_config(EXAMPLE_DAS_CONFIG).mpc
    m = base.to_dict()
    m["topology"]["bays"]["b15"]["serial"] = "SN0015"
    cfg = MpcConfig.from_mapping(m)
    topo = topology_from_config(cfg)
    for i, bay in enumerate(topo["bays"]):
        topo["bays"][bay]["serial"] = f"SN{i + 1:04d}"
    for entry in topo["sensors"].values():
        entry["noise_sigma_c"] = SENSOR_TYPES[entry["type"]].noise_sigma_c
    plant = build_das_plant(
        topo,
        preset="basic",
        dt=cfg.dt,
        seed=5,
        initial_pwm=0.5,
        burst_prob=0.3,
        burst_window_s=300.0,
        heat_schedule={"b09": SAME_SCHEDULE, "b10": SAME_SCHEDULE},
        bay_schedule=[{"t_s": REMOVE_AT, "bay": "b05", "action": "remove"}],
    )
    truth = {f"SN{i + 1:04d}": bay for i, bay in enumerate(topo["bays"])}
    mem = None
    u = dict.fromkeys(cfg.channels, 0.5)
    history = []
    for _ in range(1100):
        obs = plant.observe()
        temps = {k: v for k, v in obs.temps.items() if v is not None}
        up = E.update(mem, cfg, temps=temps, u=u, ts=obs.ts, smart=plant.observe_smart())
        history.append((obs.ts, {b: (v["serial"], v["association"]) for b, v in up.bays.items()}))
        mem = up.memory
        plant.apply(u)
        plant.advance()
    return truth, history, up


def test_truth_sim_association_finds_the_right_bays(truth_run):
    truth, history, _ = truth_run
    for ts, bays in history:
        for bay, (serial, source) in bays.items():
            if source == "correlation":
                assert truth[serial] == bay, f"t={ts}: {serial} wrongly placed in {bay}"
    before_removal = [bays for ts, bays in history if ts < REMOVE_AT][-1]
    found = [b for b, (_, source) in before_removal.items() if source == "correlation"]
    assert len(found) >= 10, before_removal
    # nothing is associated before the window is (almost) full
    early = [bays for ts, bays in history if ts < A.MIN_SPAN_FRACTION * 3600.0]
    assert all(source != "correlation" for bays in early for _, source in bays.values())


def test_truth_sim_association_refuses_indistinguishable_bays(truth_run):
    _, history, last = truth_run
    for _, bays in history:
        assert bays["b09"][1] is None and bays["b10"][1] is None
    shown = {serial for serial, _ in last.bays["b09"]["candidates"]}
    assert {"SN0009", "SN0010"} <= shown  # both candidates are listed with their scores


def test_truth_sim_association_honours_the_declared_serial(truth_run):
    _, history, _ = truth_run
    assert all(bays["b15"] == ("SN0015", "declared") for _, bays in history)


def test_truth_sim_association_drops_on_hot_swap(truth_run):
    _, history, last = truth_run
    held = [bays["b05"] for ts, bays in history if REMOVE_AT - 300.0 < ts < REMOVE_AT]
    assert held and all(entry == ("SN0005", "correlation") for entry in held)
    after = [bays["b05"] for ts, bays in history if ts >= REMOVE_AT + 60.0]
    assert after and all(entry == (None, None) for entry in after)
    assert last.bays["b05"]["occupancy"] == "empty"
