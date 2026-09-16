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


def _series_of(cfg: MpcConfig, bay: str, ts: float, *, matching: bool):
    """A recorded series over every bay and zone, plus a SMART history that does (or does
    not) correlate with ``bay``'s own trace."""
    window_s = cfg.estimator.associate_window_s
    step = A.SAMPLE_S
    n = int(window_s / step) + 1
    t_rec = [ts - window_s + step * k for k in range(n)]
    wave = [math.sin(t / 400.0) for t in t_rec]
    series = {
        "t": t_rec,
        "y": {b: (wave if b == bay else [0.0] * n) for b in cfg.topology.bays},
        "a": {z: [30.0] * n for z in cfg.topology.zones},
    }
    hist = [
        [t, 30.0 + 5.0 * (math.sin(t / 400.0) if matching else math.sin(t / 97.0))] for t in t_rec
    ]
    return series, hist


def _held(cfg: MpcConfig, bay: str, serial: str, ts: float, *, matching: bool):
    """A memory whose ``bay`` holds ``serial`` by correlation, with a full window of
    evidence that either keeps correlating or does not (item 18)."""
    mem = _with_association(cfg, bay, serial)
    series, hist = _series_of(cfg, bay, ts, matching=matching)
    mem["series"] = series
    mem["smart"][serial] = {"ts": ts, "t": 40.0, "model": "M", "hist": hist}
    mem["next_assoc"] = None  # the next tick is an evaluation
    return mem


def test_a_correlation_pair_that_stops_correlating_is_dropped():
    """Item 18: the statistic that accepted a pair has to keep holding."""
    cfg = das_cfg(setpoints={})
    ts = 4000.0
    smart = {"S1": {"temp_c": 40.0, "age_s": 1.0, "model": "M"}}
    mem = _held(cfg, "a1", "S1", ts, matching=False)
    fails = []
    for i in range(int(cfg.estimator.associate_drop_checks)):
        up = _tick(cfg, mem, ts + A.EVERY_S * (i + 1), smart)
        mem = json.loads(json.dumps(up.memory))
        fails.append(up.bays["a1"]["assoc_check_fails"])
    assert fails[:-1] == list(range(1, len(fails)))  # counted up ...
    assert fails[-1] == 0 and up.bays["a1"]["serial"] is None  # ... and then dropped
    assert mem["smart"]["S1"]["hist"] == []  # the evidence is forgotten: earn it again


def test_a_correlation_pair_that_keeps_correlating_is_kept_and_may_calibrate():
    cfg = das_cfg(setpoints={})
    ts = 4000.0
    smart = {"S1": {"temp_c": 40.0, "age_s": 1.0, "model": "M"}}
    mem = _held(cfg, "a1", "S1", ts, matching=True)
    for i in range(4):
        up = _tick(cfg, mem, ts + A.EVERY_S * (i + 1), smart)
        mem = json.loads(json.dumps(up.memory))
        assert up.bays["a1"]["serial"] == "S1", i
        assert up.bays["a1"]["assoc_check_fails"] == 0
    assert mem["bays"]["a1"]["ver"] is True


def test_a_guessed_pair_does_not_calibrate_before_its_first_re_check():
    """Until a correlation pair has passed one re-check its map is not used, however
    many samples it has; a declared serial is used as before (item 18)."""
    cfg = das_cfg(setpoints={})
    mem = _with_association(cfg, "a1", "S1")
    entry = E._fresh_calibration(cfg.estimator.sigma_uncalibrated_c)
    for k in range(40):  # a converged calibration of the wrong drive
        x = 4.0 + 3.0 * math.sin(k)
        entry, _ = E.calibration_update(entry, x, 0.7 * x - 2.1, 0.0)
    entry["used"] = True
    mem["cal"]["a1"] = {"S1": entry}
    up = _tick(cfg, mem, 1.0, {"S1": {"temp_c": 40.0, "age_s": 1.0, "model": "M"}})
    assert up.bays["a1"]["association"] == "correlation"
    assert up.bays["a1"]["calibrated"] is False
    assert up.bays["a1"]["sigma_cal_c"] == cfg.estimator.sigma_uncalibrated_c
    mem["bays"]["a1"]["ver"] = True  # one re-check passed
    ok = _tick(cfg, mem, 1.0, {"S1": {"temp_c": 40.0, "age_s": 1.0, "model": "M"}})
    assert ok.bays["a1"]["calibrated"] is True


def _smart_on_the_wave(ts: float):
    """The SMART reading of the serial whose history :func:`_series_of` built."""
    return {"S1": {"temp_c": 30.0 + 5.0 * math.sin(ts / 400.0), "age_s": 1.0, "model": "M"}}


def _correlating(mem, cfg: MpcConfig, bay: str, serial: str, ts: float):
    """Give ``mem`` a window in which ``serial``'s SMART history correlates with
    ``bay``'s own series, so the next tick's evaluation scores the pair at 1.0."""
    series, hist = _series_of(cfg, bay, ts, matching=True)
    mem["series"] = series
    mem["smart"][serial] = {"ts": ts, "t": 40.0, "model": "M", "hist": hist}
    mem["next_assoc"] = None  # the next tick is an evaluation
    return mem


def test_a_confirmed_pair_starts_unverified_with_no_evidence_behind_it():
    """Item 18: the window that accepts a pair may not sit as the pair's own first
    re-check, so a correlator-confirmed pair rebuilds a full window before it may
    calibrate its bay -- exactly what a pair dropped and re-earned has to do."""
    cfg = das_cfg(setpoints={})
    ts = 4000.0
    mem = json.loads(json.dumps(_tick(cfg, None, ts, _smart_on_the_wave(ts)).memory))
    for i in range(A.CONFIRM_EVALUATIONS):
        t = ts + A.EVERY_S * (i + 1)
        up = _tick(cfg, _correlating(mem, cfg, "a1", "S1", t), t, _smart_on_the_wave(t))
        mem = json.loads(json.dumps(up.memory))
    assert mem["bays"]["a1"]["assoc"] == "S1"  # the correlator confirmed it ...
    assert mem["bays"]["a1"]["ver"] is False  # ... unverified, with its evidence gone
    assert mem["smart"]["S1"]["hist"] == []
    # an evaluation later there is still nothing to score, so the pair stays unverified
    t = ts + A.EVERY_S * (A.CONFIRM_EVALUATIONS + 1)
    up = _tick(cfg, mem, t, _smart_on_the_wave(t))
    assert up.bays["a1"]["serial"] == "S1" and up.bays["a1"]["association"] == "correlation"
    assert up.memory["bays"]["a1"]["ver"] is False
    assert up.bays["a1"]["calibrated"] is False


def test_a_declaration_clears_the_bay_s_verification():
    """``ver`` belongs to a pair, not to a bay: declaring a serial (and undeclaring it
    again) must not leave the next correlation pair verified from its first tick."""
    cfg = das_cfg(setpoints={})
    mem = _with_association(cfg, "a1", "S1")
    mem["bays"]["a1"]["ver"] = True
    m = cfg.to_dict()
    m["topology"]["bays"]["a1"]["serial"] = "S1"
    declared = MpcConfig.from_mapping(m)
    smart = {"S1": {"temp_c": 40.0, "age_s": 1.0, "model": "M"}}
    up = _tick(declared, mem, 1.0, smart)
    assert up.bays["a1"]["association"] == "declared"
    assert up.memory["bays"]["a1"]["ver"] is False
    mem = json.loads(json.dumps(up.memory))  # the declaration is removed again
    mem["bays"]["a1"]["assoc"] = "S2"  # and the correlator confirms a different serial
    up = _tick(cfg, mem, 2.0, {"S2": {"temp_c": 40.0, "age_s": 1.0, "model": "M"}})
    assert up.bays["a1"]["serial"] == "S2" and up.bays["a1"]["calibrated"] is False
    assert up.memory["bays"]["a1"]["ver"] is False


def test_an_unverified_pair_is_not_an_anchor_for_the_thermal_identification():
    """``calibration.accepted_once`` is what ``mpc._thermal_shadow`` builds its maps
    from, so it says what the filter itself does with the map (item 18)."""
    cfg = das_cfg(setpoints={})
    mem = _with_association(cfg, "a1", "S1")
    entry = E._fresh_calibration(cfg.estimator.sigma_uncalibrated_c)
    for k in range(40):
        x = 4.0 + 3.0 * math.sin(k)
        entry, _ = E.calibration_update(entry, x, 0.7 * x - 2.1, 0.0)
    entry["used"] = True  # a map accepted while the pair was verified
    mem["cal"]["a1"] = {"S1": entry}
    smart = {"S1": {"temp_c": 40.0, "age_s": 1.0, "model": "M"}}
    up = _tick(cfg, mem, 1.0, smart)
    assert up.bays["a1"]["calibrated"] is False
    assert up.bays["a1"]["calibration"]["accepted_once"] is False
    mem["bays"]["a1"]["ver"] = True  # one re-check passed: both agree again
    ok = _tick(cfg, mem, 1.0, smart)
    assert ok.bays["a1"]["calibrated"] is True
    assert ok.bays["a1"]["calibration"]["accepted_once"] is True


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
