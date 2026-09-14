"""Stuck evidence on the DAS truth simulator (PROJECT.md section 8 item 3, section 3).

A healthy enclosure must not fault a zone over an idle drive's plateau: before
the fix, about one in eight 75-minute ``rich`` runs with the PI-like DAS solver
branded an idle bay's DS18B20 Stuck -- its reading sat inside its 1.5-LSB band
for ``stuck_s`` while a neighbouring bay's activity burst moved that bay's
reading by degrees (seed 8), or while the zone's fans rose against a warming
inlet that kept the reading still (seed 22). A frozen reading must still be
flagged once its zone's airflow moves enough to move it, on a DS18B20 and on a
thermistor, and fault only its own zone.

The runs call ``mpc.step`` directly (the section 4.1 invariants are asserted
by the DAS core suites; here they would quadruple the time). The PR cases are
the two seeds of the report; ``nightly`` sweeps 48 seeds with the PI-like DAS
solver and a few 2.5-hour runs of both DAS solvers that exercised the airflow
rules while they were tuned (channels moved apart by the solver, a short fan dip
of the DAS MPC at the start of a window). ``tests/test_gate.py`` pins each rule
and the detection time on scripted samples.
"""

from __future__ import annotations

import dataclasses
from collections import Counter

import pytest

from aqua_bridge.control.gate import stuck_pwm_lag
from aqua_bridge.control.mpc import step
from aqua_bridge.model import Mode, MpcConfig, PlantObservation
from aqua_bridge.sim.das import (
    SENSOR_TYPES,
    DasRun,
    build_das_plant,
    run_das_closed_loop,
    topology_from_config,
)

#: 75 minutes at the example's dt = 5 s: the length of the runs in the report.
RUN_TICKS = 900
#: The load step on the neighbouring bay and when the reading freezes (test scenario).
FREEZE_S = 300.0
LOAD_S = 600.0


def rich_run(cfg: MpcConfig, seed: int, ticks: int) -> DasRun:
    """The example config on the ``rich`` simulator (drawn physics, activity bursts)."""
    plant = build_das_plant(
        topology_from_config(cfg), preset="rich", seed=seed, dt=cfg.dt, initial_pwm=0.5
    )
    return run_das_closed_loop(plant, cfg, step, ticks)


def assert_no_false_stuck(run: DasRun) -> None:
    flagged = sorted(
        {
            name
            for rec in run.records
            for name, stuck in rec.cmd.diagnostics["gate"]["stuck"].items()
            if stuck
        }
    )
    assert flagged == [], f"healthy sensors flagged Stuck: {flagged}"
    modes = Counter(rec.cmd.mode for rec in run.records)
    assert modes[Mode.DEGRADED] == 0 and modes[Mode.FALLBACK] == 0, modes


@pytest.mark.parametrize("seed", [8, 22], ids=["neighbour-burst", "inlet-against-fans"])
def test_healthy_rich_runs_fault_no_zone(das_example_cfg, seed):
    assert_no_false_stuck(rich_run(das_example_cfg, seed, RUN_TICKS))


@pytest.mark.nightly
@pytest.mark.parametrize("seed", range(48))
def test_healthy_rich_runs_fault_no_zone_sweep(das_example_cfg, seed):
    assert_no_false_stuck(rich_run(das_example_cfg, seed, RUN_TICKS))


@pytest.mark.nightly
@pytest.mark.parametrize(
    ("solver", "seed", "ticks"),
    [("pi", 71, 1800), ("pi", 91, 1800), ("pi", 252, 1800), ("mpc", 130, 1800), ("mpc", 150, 1800)],
)
def test_long_healthy_runs_of_both_solvers_fault_no_zone(das_example_cfg, solver, seed, ticks):
    cfg = dataclasses.replace(das_example_cfg, solver=solver, model_accept_prior=solver == "mpc")
    assert_no_false_stuck(rich_run(cfg, seed, ticks))


# ---------------------------------------------------------------------------
# a frozen reading is still flagged
# ---------------------------------------------------------------------------


def frozen_run(cfg: MpcConfig, sensor: str, ticks: int) -> DasRun:
    """``basic`` physics with each sensor type's noise; ``sensor`` frozen from ``FREEZE_S``
    on, and bay b02 (zone z0) at full load from ``LOAD_S`` on, so the zone's fans rise."""
    topology = topology_from_config(cfg)
    for entry in topology["sensors"].values():
        entry["noise_sigma_c"] = SENSOR_TYPES[entry["type"]].noise_sigma_c
    plant = build_das_plant(
        topology,
        preset="basic",
        dt=cfg.dt,
        initial_pwm=0.5,
        heat_schedule={"b02": [(LOAD_S, 1.0)]},
    )
    held: dict[str, float | None] = {}

    def freeze(i: int, obs: PlantObservation) -> PlantObservation:
        if obs.ts < FREEZE_S:
            return obs
        temps = dict(obs.temps)
        temps[sensor] = held.setdefault(sensor, temps[sensor])
        return dataclasses.replace(obs, temps=temps)

    return run_das_closed_loop(plant, cfg, step, ticks, observe_hook=freeze)


@pytest.mark.parametrize(
    ("sensor", "faults_zone"),
    [("prox_b01", True), ("prox_b03b", False)],
    ids=["ds18b20", "thermistor-redundant"],
)
def test_frozen_proximal_reading_is_flagged_once_its_zone_airflow_moves(
    das_example_cfg, sensor, faults_zone
):
    cfg = das_example_cfg
    params = cfg.stuck_params(sensor)
    expected_type = "ds18b20" if sensor == "prox_b01" else "thermistor"
    assert topology_from_config(cfg)["sensors"][sensor]["type"] == expected_type
    window_s = params.ticks * cfg.dt
    # The flag comes at the latest at max(freeze + window, t1 + window / 2) plus two
    # decimation intervals, t1 being when the zone's airflow has moved (section 3, pinned
    # on scripted samples in test_gate.py). The load raises z0's fans within minutes: allow
    # t1 up to three quarters of a window after the load.
    lag_s = stuck_pwm_lag(params.samples) * params.decimate * cfg.dt
    bound_s = LOAD_S + window_s + lag_s
    run = frozen_run(cfg, sensor, int(bound_s / cfg.dt) + 2)
    flags = [r.obs.ts for r in run.records if r.cmd.diagnostics["gate"]["stuck"][sensor]]
    assert flags, f"{sensor} frozen from {FREEZE_S} s was never flagged"
    assert flags[0] <= bound_s
    assert all(
        r.cmd.diagnostics["gate"]["stuck"][sensor] for r in run.records if r.obs.ts >= flags[0]
    )
    in_fault = {z for r in run.records for z in r.cmd.diagnostics["zones_in_fault"]}
    if faults_zone:  # the bay's only proximal sensor: its zone holds, then ramps high
        assert in_fault == {"z0"}
        assert run.records[-1].cmd.mode is Mode.DEGRADED
    else:  # prox_b03 still covers bay b03
        assert in_fault == set()
    assert run.violations() == 0
