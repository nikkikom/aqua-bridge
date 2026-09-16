"""Return from the DAS MPC's model fallback after a load step (PROJECT.md section 8 item 10).

The validity gate's drift check used to hold the PI-like DAS fallback for tens of minutes
after a load step: the drives warm at 0.26-0.36 degC/min, which the model predicts, and
the plain drift had to fall below ``model_return_factor * model_max_drift_c_per_min``
(0.25 degC/min) before the dwell even started. The return now checks the drift relative
to the drives' observed rate (``control/solver_das.py``, validity gate), so a sound model
returns after ``model_return_dwell_s``, and a model whose equilibrium is wrong still
enters the fallback and stays there.

Scenario: the example DAS config with the DAS MPC acting on the prior thermal model
(``model_accept_prior``; on the ``basic`` simulator the prior matches the truth), every
bay idle, then at :data:`T_STEP` every bay of one zone at full load. A sound model is
pushed into the fallback at the step by a thermal status the gate rejects for two ticks
(the check under test is the return, not the cause); a broken model has its bay gains
``g0`` and ``k`` scaled from the step on and reports ``converged``. The runs call
``mpc.step`` directly, as ``tests/test_stuck_sim.py`` does. ``nightly`` sweeps zones and
seeds, the ``rich`` simulator (drawn physics the prior does not match) and more broken
models.
"""

from __future__ import annotations

import dataclasses
import json
from types import SimpleNamespace
from typing import Any

import pytest

from aqua_bridge.config import load_config
from aqua_bridge.control import thermal
from aqua_bridge.control.mpc import step
from aqua_bridge.control.solver_das import DasMpcSolver, _fresh_memory, _parse_memory
from aqua_bridge.model import MpcConfig, SolverKind
from aqua_bridge.sim.das import (
    SENSOR_TYPES,
    DasRun,
    build_das_plant,
    run_das_closed_loop,
    topology_from_config,
)
from conftest import EXAMPLE_DAS_CONFIG

#: Bay settling (``estimator.bay_settle_s``, 600 s) and a steady idle enclosure first.
T_STEP = 1800.0
T_END = 3600.0
#: Ticks the thermal status is rejected at the step (a sound model's fallback).
KICK_TICKS = 2
#: A broken model is detected within this long after it breaks (bay gains 0.3x, 2x and
#: 3x: measured 0-150 s after the load step).
DETECT_BOUND_S = 300.0
#: Bay gains at half their value are detected only when the warming drives push the plain
#: drift over ``model_max_drift_c_per_min`` (measured 830-860 s after the step on the
#: basic simulator; the entry is unchanged by this item).
SLOW_GAINS = (0.5,)
DETECT_SLOW_BOUND_S = 1200.0
#: Bay gains 1.5x: the band where the relative drift sits just over its limit instead of
#: far above it (0.49-0.54 degC/min against 0.5) and dips under it every few ticks, so the
#: model is faulted only because the entry dwell leaks. Caught 220-530 s after the step on
#: the combinations below; where the drift never reaches its limit at all (``basic`` z2:
#: two ticks over in a whole run) no threshold on it can see the error.
MODERATE_GAIN = 1.5
DETECT_MODERATE_BOUND_S = 900.0

#: Fouling scenario (section 8 item 66): the enclosure keeps a steady load, then every
#: fan's ``e`` drops to :data:`FOUL_FACTOR` of it -- a blocked filter or a dust mat, not a
#: fan fault: the tachometers read the same. At ``model_max_air_dist_c_per_min`` 8.0 the
#: gate catches a drop to 0.15x or below on every seed and load of both presets (and
#: nothing healthy); 0.25x on about half the runs, 0.3-0.5x on ``rich`` only.
T_FOUL = 1800.0
FOUL_END = 4800.0
FOUL_FACTOR = 0.15
FOUL_BOUND_S = 900.0
FOUL_HEAT = {
    "b01": [(0.0, 0.6)],
    "b02": [(0.0, 1.0)],
    "b06": [(0.0, 0.8)],
    "b10": [(0.0, 1.0)],
    "b13": [(0.0, 0.7)],
}


def scenario_cfg(**changes: Any) -> MpcConfig:
    data = load_config(EXAMPLE_DAS_CONFIG).mpc.to_dict()
    data.update(solver=SolverKind.MPC.value, model_accept_prior=True, **changes)
    return MpcConfig.from_mapping(data)


def return_bound_s(cfg: MpcConfig) -> float:
    """The documented bound: the dwell plus two solve periods (the first passing check
    after the kick and the solve tick at which the dwell has run out)."""
    return cfg.model_return_dwell_s + 2 * cfg.mpc_every_ticks * cfg.dt


def build_plant(cfg: MpcConfig, *, preset: str, seed: int, heat_schedule: Any = None):
    """The truth plant of every scenario here: the config's topology, sensor noise of the
    real types, a 25 degC inlet."""
    topology = topology_from_config(cfg)
    for entry in topology["sensors"].values():
        entry["noise_sigma_c"] = SENSOR_TYPES[entry["type"]].noise_sigma_c
    topology["inlet"] = {"base_c": 25.0}
    return build_das_plant(
        topology,
        preset=preset,
        dt=cfg.dt,
        initial_pwm=0.5,
        seed=seed,
        heat_schedule=heat_schedule,
    )


def load_step_run(
    cfg: MpcConfig,
    monkeypatch: pytest.MonkeyPatch,
    *,
    zone: str = "z0",
    seed: int = 1,
    preset: str = "basic",
    gain: float | None = None,
) -> DasRun:
    """The scenario of the module docstring; ``gain`` scales the model's bay gains from
    :data:`T_STEP` on (a broken model), ``None`` kicks a sound model into the fallback."""
    zones = {bay: spec.zone for bay, spec in cfg.topology.bays.items()}
    schedule = {bay: [(0.0, 0.0), (T_STEP, 1.0 if z == zone else 0.0)] for bay, z in zones.items()}
    plant = build_plant(cfg, preset=preset, seed=seed, heat_schedule=schedule)
    clock = {"ts": 0.0}
    current_model = thermal.current_model

    def model_at_ts(memory: object, c: MpcConfig) -> tuple[str, dict[str, float]]:
        status, theta = current_model(memory, c)
        if clock["ts"] < T_STEP:
            return status, theta
        if gain is None:
            return ("error", theta) if clock["ts"] < T_STEP + KICK_TICKS * c.dt else (status, theta)
        scaled = {k: v * gain if k.split(".")[0] in ("g0", "k") else v for k, v in theta.items()}
        return "converged", scaled

    monkeypatch.setattr(thermal, "current_model", model_at_ts)

    def controller(obs, c, state):
        clock["ts"] = float(obs.ts)
        return step(obs, c, state)

    return run_das_closed_loop(plant, cfg, controller, int(T_END / cfg.dt))


def model_view(run: DasRun) -> list[tuple[float, dict[str, Any]]]:
    return [
        (ts, rec.cmd.diagnostics["solver_diag"]["model"])
        for ts, rec in zip(run.series["ts"], run.records, strict=True)
    ]


def switches(run: DasRun) -> list[tuple[float, str]]:
    view = model_view(run)
    return [
        (ts, m["active"])
        for (_, before), (ts, m) in zip(view, view[1:], strict=False)
        if m["active"] != before["active"]
    ]


def assert_bumpless(run: DasRun) -> None:
    """On a switch of the active model the output is the previous command."""
    for before, rec in zip(run.records, run.records[1:], strict=False):
        m0 = before.cmd.diagnostics["solver_diag"]["model"]["active"]
        m1 = rec.cmd.diagnostics["solver_diag"]["model"]["active"]
        if m0 != m1:
            target = rec.cmd.diagnostics["target_pwm"]
            for ch, value in before.cmd.pwm.items():
                assert target[ch] == pytest.approx(value, abs=1e-9), (ch, m0, m1)


def assert_safe_and_bumpless(run: DasRun) -> None:
    """Every drive within its limit, and every switch bumpless."""
    assert run.violations() == 0, run.worst_margin_c()
    assert_bumpless(run)


def assert_at_the_rail_when_over(run: DasRun, cfg: MpcConfig) -> None:
    """Where a drive is over its limit, every channel is at ``pwm_max``: the loop is out
    of enclosure, not out of will."""
    for i, rec in enumerate(run.records):
        over = [
            b for b, seq in run.series["margin_c"].items() if seq[i] is not None and seq[i] < 0.0
        ]
        if not over:
            continue
        for ch, value in rec.cmd.pwm.items():
            assert value == pytest.approx(cfg.pwm_max, abs=1e-9), (i, over, ch, value)


def first_return_after_step(run: DasRun) -> float | None:
    back = [ts for ts, active in switches(run) if active == "mpc" and ts >= T_STEP]
    return back[0] - T_STEP if back else None


def assert_returns_in_bound(run: DasRun, cfg: MpcConfig) -> None:
    sw = switches(run)
    assert sw and sw[0] == (T_STEP, "pi_das"), sw
    elapsed = first_return_after_step(run)
    assert elapsed is not None, sw
    assert cfg.model_return_dwell_s <= elapsed <= return_bound_s(cfg), sw
    # section 8 item 64: the MPC's own quieter move after the return is not a model fault
    assert [active for _, active in sw] == ["pi_das", "mpc"], sw


def assert_caught_and_held(run: DasRun, bound_s: float = DETECT_BOUND_S) -> None:
    sw = [(ts, a) for ts, a in switches(run) if ts >= T_STEP]
    assert sw and sw[0][1] == "pi_das", sw
    assert sw[0][0] - T_STEP <= bound_s, sw
    assert len(sw) == 1, f"a broken model came back: {sw}"
    reason = model_view(run)[-1][1]["reason"]
    assert reason.startswith(("drift:", "pred_err:")), reason


# ---------------------------------------------------------------------------
# observed rate of the drive estimates
# ---------------------------------------------------------------------------


def _rate_request(bays: dict[str, dict[str, Any]]) -> Any:
    return SimpleNamespace(plant={"bays": bays})


def _track(cfg: MpcConfig, mem: dict[str, Any], ts: float, bays: dict[str, dict[str, Any]]):
    DasMpcSolver._track_rates(cfg, _rate_request(bays), mem, ts)
    return mem["rate"]["r"]


def test_observed_rate_follows_a_ramp_and_restarts_at_zero():
    cfg = scenario_cfg()
    mem = _fresh_memory()
    slope = 0.3  # degC/min
    r = _track(cfg, mem, 0.0, {"b01": {"t": 40.0, "since_ts": 0.0}})
    assert r == {"b01": 0.0}  # a fresh track checks the plain drift
    ts = 0.0
    for _ in range(int(10 * cfg.model_drift_rate_tau_s / cfg.dt)):
        ts += cfg.dt
        r = _track(cfg, mem, ts, {"b01": {"t": 40.0 + slope * ts / 60.0, "since_ts": 0.0}})
    assert r["b01"] == pytest.approx(slope, abs=1e-3)
    t_now = 40.0 + slope * ts / 60.0
    # one tau after the ramp stops, the rate has decayed by about 1/e
    for _ in range(int(cfg.model_drift_rate_tau_s / cfg.dt)):
        ts += cfg.dt
        r = _track(cfg, mem, ts, {"b01": {"t": t_now, "since_ts": 0.0}})
    assert r["b01"] == pytest.approx(slope * 2.718281828**-1, rel=0.05)
    json.loads(json.dumps(mem))  # plain JSON
    restored = _parse_memory(json.loads(json.dumps(mem)), cfg)
    assert restored["rate"] == mem["rate"]

    def restarted(ts_next: float, since: float) -> float:
        probe = json.loads(json.dumps(mem))
        return _track(cfg, probe, ts_next, {"b01": {"t": t_now + 1.0, "since_ts": since}})["b01"]

    assert restarted(ts + cfg.dt, 0.0) != 0.0
    assert restarted(ts + cfg.dt, ts) == 0.0  # occupancy changed
    assert restarted(ts + 3601.0, 0.0) == 0.0  # a gap over an hour
    assert restarted(ts - cfg.dt, 0.0) == 0.0  # a clock stepped back
    assert restarted(ts, 0.0) == 0.0  # a clock that did not advance
    probe = json.loads(json.dumps(mem))
    assert _track(cfg, probe, ts + cfg.dt, {"b01": {"t": None}, "b02": {"t": 30.0}}) == {"b02": 0.0}


# ---------------------------------------------------------------------------
# the slow level of the estimator's air disturbances
# ---------------------------------------------------------------------------


def _track_air(cfg: MpcConfig, mem: dict[str, Any], ts: float, d: dict[str, float]):
    mem["dist"] = {**mem["dist"], "d": dict(d)}
    DasMpcSolver._track_air_dist(cfg, mem, ts)
    return mem["dslow"]


def test_the_air_disturbance_level_holds_over_a_clock_step_and_restarts_after_a_gap():
    """Section 8 item 66: the level the air-disturbance check measures a move against is a
    reference only once it has run for ``model_air_dist_tau_s``, and a clock that steps
    back must not re-reference it to a disturbance that has already moved."""
    cfg = scenario_cfg()
    mem = _fresh_memory()
    tau = cfg.model_air_dist_tau_s
    slow = _track_air(cfg, mem, 0.0, {"z0": 1.0})
    assert slow["d"] == {"z0": 1.0} and slow["age"] == {"z0": 0.0}  # no reference yet
    ts = 0.0
    while ts < tau:
        ts += cfg.dt
        slow = _track_air(cfg, mem, ts, {"z0": 1.0})
    assert slow["age"]["z0"] == tau  # a reference now, and it followed a steady disturbance
    assert slow["d"]["z0"] == pytest.approx(1.0)
    ts += cfg.dt
    slow = _track_air(cfg, mem, ts, {"z0": 3.0})  # the disturbance moves, the level crawls
    assert 1.0 < slow["d"]["z0"] < 1.1 and slow["age"]["z0"] == tau
    level = slow["d"]["z0"]

    json.loads(json.dumps(mem))  # plain JSON
    restored = _parse_memory(json.loads(json.dumps(mem)), cfg)
    assert restored["dslow"] == mem["dslow"]
    assert restored["mrate"] == mem["mrate"]
    assert (restored["drift_since"], restored["drift_last"]) == (None, None)

    stepped = _parse_memory(json.loads(json.dumps(mem)), cfg)
    back = _track_air(cfg, stepped, ts - 86_400.0, {"z0": 3.0})
    assert back["d"]["z0"] == pytest.approx(level) and back["age"]["z0"] == tau
    assert back["ts"] == ts - 86_400.0  # the filter resumes on the new clock

    gapped = _parse_memory(json.loads(json.dumps(mem)), cfg)
    after = _track_air(cfg, gapped, ts + 3601.0, {"z0": 3.0})
    assert after["d"]["z0"] == 3.0 and after["age"]["z0"] == 0.0  # no reference again


def test_the_entry_dwell_times_survive_the_memory():
    mem = {**_fresh_memory(), "drift_since": 120.0, "drift_last": 300.0}
    restored = _parse_memory(json.loads(json.dumps(mem)), scenario_cfg())
    assert (restored["drift_since"], restored["drift_last"]) == (120.0, 300.0)


@pytest.mark.parametrize(
    "rate",
    [
        {"t": {}, "r": {}, "since": []},
        {"t": {}, "since": {}},
        {"t": {"b01": "x"}, "r": {}, "since": {}},
    ],
)
def test_a_malformed_rate_memory_starts_over(rate):
    cfg = scenario_cfg()
    mem = {**_fresh_memory(), "active": "mpc", "rate": rate}
    assert _parse_memory(mem, cfg) == _fresh_memory()


# ---------------------------------------------------------------------------
# closed loop on the truth simulator
# ---------------------------------------------------------------------------


def test_a_sound_model_returns_after_a_load_step_within_the_bound(monkeypatch):
    cfg = scenario_cfg()
    run = load_step_run(cfg, monkeypatch)
    assert_safe_and_bumpless(run)
    assert_returns_in_bound(run, cfg)
    # the load step is the case of the item: the plain drift stayed above the return
    # limit during the dwell, the relative one below it
    limit = cfg.model_return_factor * cfg.model_max_drift_c_per_min
    dwell = [
        m["checks"]
        for ts, m in model_view(run)
        if T_STEP < ts < T_STEP + cfg.model_return_dwell_s and m["active"] == "pi_das"
    ]
    assert max(c["drift_abs_c_per_min"] or 0.0 for c in dwell) > limit
    assert all((c["drift_rel_c_per_min"] or 0.0) <= limit for c in dwell[KICK_TICKS:])


@pytest.mark.nightly
def test_the_plain_drift_would_hold_the_fallback_after_the_same_step(monkeypatch):
    """The same run with a rate filter too slow to follow the drives checks the plain
    drift on the return, which is what held the fallback before."""
    cfg = scenario_cfg(model_drift_rate_tau_s=1e9)
    run = load_step_run(cfg, monkeypatch)
    assert_safe_and_bumpless(run)
    elapsed = first_return_after_step(run)
    assert elapsed is None or elapsed > 2 * return_bound_s(cfg), switches(run)


def test_a_broken_model_is_caught_and_held_in_the_fallback(monkeypatch):
    cfg = scenario_cfg()
    run = load_step_run(cfg, monkeypatch, gain=2.0)
    assert_safe_and_bumpless(run)
    assert_caught_and_held(run)


# ---------------------------------------------------------------------------
# nightly sweep
# ---------------------------------------------------------------------------


@pytest.mark.nightly
@pytest.mark.parametrize("zone", ["z0", "z1", "z2", "z3"])
@pytest.mark.parametrize(
    ("preset", "seed"),
    [("basic", 2), ("basic", 3), ("rich", 1), ("rich", 2), ("rich", 3), ("rich", 4)],
)
def test_return_sweep(monkeypatch, preset, seed, zone):
    cfg = scenario_cfg()
    run = load_step_run(cfg, monkeypatch, zone=zone, seed=seed, preset=preset)
    assert_safe_and_bumpless(run)
    assert_returns_in_bound(run, cfg)


@pytest.mark.nightly
@pytest.mark.parametrize("zone", ["z0", "z2"])
@pytest.mark.parametrize(
    ("preset", "gain"),
    [("basic", 0.3), ("basic", 0.5), ("basic", 2.0), ("basic", 3.0), ("rich", 0.3), ("rich", 2.0)],
)
def test_broken_model_sweep(monkeypatch, preset, gain, zone):
    cfg = scenario_cfg()
    run = load_step_run(cfg, monkeypatch, zone=zone, seed=2, preset=preset, gain=gain)
    assert_safe_and_bumpless(run)
    assert_caught_and_held(run, DETECT_SLOW_BOUND_S if gain in SLOW_GAINS else DETECT_BOUND_S)


@pytest.mark.nightly
@pytest.mark.parametrize(
    ("preset", "zone", "seed"),
    [("basic", "z0", 2), ("basic", "z0", 3), ("rich", "z0", 2), ("rich", "z2", 1)],
)
def test_a_moderate_parameter_error_is_caught_and_held(monkeypatch, preset, zone, seed):
    """The band between a sound model and the sweep's 2x: bay gains :data:`MODERATE_GAIN`
    hold the drift just over its limit and sensor noise dips it under every few ticks. The
    entry dwell leaks, so it is a model fault -- with a dwell that had to be contiguous
    three of these four combinations never entered the fallback at all (``basic`` z0
    seed 2: 112 ticks over the limit over 18 minutes, no fallback). The zones and seeds are
    the ones where a 1.5x error reaches the drift at all; on ``basic`` z2 it does not, and
    the run stays on the MPC (the drives stay 10.8 degC inside their limits either way)."""
    cfg = scenario_cfg()
    run = load_step_run(cfg, monkeypatch, zone=zone, seed=seed, preset=preset, gain=MODERATE_GAIN)
    assert_safe_and_bumpless(run)
    assert_caught_and_held(run, DETECT_MODERATE_BOUND_S)


# ---------------------------------------------------------------------------
# a healthy enclosure stays on the MPC (section 8 item 65)
# ---------------------------------------------------------------------------


def steady_run(cfg: MpcConfig, *, preset: str, seed: int) -> DasRun:
    """No fault of any kind: idle bays, the prior model, the truth plant."""
    plant = build_plant(cfg, preset=preset, seed=seed)
    return run_das_closed_loop(plant, cfg, step, int(T_END / cfg.dt))


@pytest.mark.parametrize("preset", ["basic", "rich"])
def test_a_healthy_enclosure_never_reaches_the_model_fallback(preset):
    """Section 8 item 65: the drives' physical warm-up right after ``bay_settle_s`` used to
    push the plain entry drift over its limit (0.50-0.52 degC/min against the limit 0.5) on
    three of eight ``rich`` seeds, holding the fallback for 5 minutes each time."""
    for seed in (1, 5, 7):
        run = steady_run(scenario_cfg(), preset=preset, seed=seed)
        assert switches(run) == [], f"{preset} seed {seed}"
        assert_safe_and_bumpless(run)


@pytest.mark.nightly
@pytest.mark.parametrize("preset", ["basic", "rich"])
@pytest.mark.parametrize("seed", [1, 2, 3, 4, 5, 6, 7, 8])
def test_healthy_sweep(preset, seed):
    run = steady_run(scenario_cfg(), preset=preset, seed=seed)
    assert switches(run) == []
    assert_safe_and_bumpless(run)


# ---------------------------------------------------------------------------
# a fouling jump the drive rows cannot see (section 8 item 66)
# ---------------------------------------------------------------------------


def fouling_run(
    cfg: MpcConfig, *, factor: float | None, preset: str = "basic", seed: int = 1
) -> DasRun:
    """The enclosure loses airflow at :data:`T_FOUL`: every fan's ``e`` scaled by
    ``factor``, the fans themselves unchanged. ``None`` is the same run left healthy."""
    plant = build_plant(cfg, preset=preset, seed=seed, heat_schedule=FOUL_HEAT)
    done = {"v": False}

    def on_tick(_rec: Any) -> None:
        if factor is None or done["v"] or plant.ts < T_FOUL:
            return
        done["v"] = True
        plant.params = dataclasses.replace(
            plant.params,
            fans=tuple(
                dataclasses.replace(f, e_w_per_k=f.e_w_per_k * factor) for f in plant.params.fans
            ),
        )

    return run_das_closed_loop(plant, cfg, step, int(FOUL_END / cfg.dt), on_tick=on_tick)


def assert_fouling_caught(run: DasRun, cfg: MpcConfig) -> tuple[float, dict[str, Any]]:
    sw = [(ts, a) for ts, a in switches(run) if ts >= T_FOUL]
    assert sw and sw[0][1] == "pi_das", switches(run)
    assert sw[0][0] - T_FOUL <= FOUL_BOUND_S, sw
    entered = next(m for ts, m in model_view(run) if ts == sw[0][0])
    assert entered["reason"].startswith("air_dist:"), entered["reason"]
    assert_bumpless(run)
    # 0.15x airflow is past what the enclosure can carry on some `rich` seeds (its fans
    # already run near the rail at full airflow), so the drives may cross their limits --
    # with every channel at pwm_max, which is all any solver could do
    assert_at_the_rail_when_over(run, cfg)
    return sw[0][0] - T_FOUL, entered


@pytest.mark.parametrize("preset", ["basic", "rich"])
def test_a_fouling_jump_enters_the_model_fallback(preset):
    """Section 8 item 66: the fan gains ``E`` only move the air node, so airflow the
    enclosure no longer has leaves the drive rows' prediction error and drift inside their
    limits. The air disturbance the estimator has to carry is the evidence that sees it."""
    cfg = scenario_cfg()
    _, entered = assert_fouling_caught(fouling_run(cfg, factor=FOUL_FACTOR, preset=preset), cfg)
    checks = entered["checks"]
    assert checks["pred_err_c"] < checks["max_pred_err_c"]
    assert checks["drift_c_per_min"] < checks["max_drift_c_per_min"]


@pytest.mark.parametrize("preset", ["basic", "rich"])
def test_the_same_run_without_the_fouling_jump_stays_on_the_mpc(preset):
    run = fouling_run(scenario_cfg(), factor=None, preset=preset)
    assert switches(run) == []
    assert_safe_and_bumpless(run)


@pytest.mark.nightly
@pytest.mark.parametrize("preset", ["basic", "rich"])
@pytest.mark.parametrize("seed", [1, 2, 3, 4])
def test_fouling_sweep(preset, seed):
    cfg = scenario_cfg()
    assert switches(fouling_run(cfg, factor=None, preset=preset, seed=seed)) == []
    assert_fouling_caught(fouling_run(cfg, factor=FOUL_FACTOR, preset=preset, seed=seed), cfg)
