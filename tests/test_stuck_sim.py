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


def rich_run(
    cfg: MpcConfig, seed: int, ticks: int, freeze: str | None = None, freeze_s: float = 0.0
) -> DasRun:
    """The example config on the ``rich`` simulator (drawn physics, activity bursts).

    ``freeze`` names a sensor whose reading is held at its value from ``freeze_s`` on.
    """
    plant = build_das_plant(
        topology_from_config(cfg), preset="rich", seed=seed, dt=cfg.dt, initial_pwm=0.5
    )
    held: dict[str, float | None] = {}

    def hold(i: int, obs: PlantObservation) -> PlantObservation:
        if obs.ts < freeze_s or obs.temps.get(freeze) is None:
            return obs
        temps = dict(obs.temps)
        temps[freeze] = held.setdefault(freeze, temps[freeze])  # type: ignore[index]
        return dataclasses.replace(obs, temps=temps)

    hook = None if freeze is None else hold
    return run_das_closed_loop(plant, cfg, step, ticks, observe_hook=hook)


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
    lag_s = stuck_pwm_lag(params.samples, cfg.stuck_pwm_lag_fraction) * params.decimate * cfg.dt
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


# ---------------------------------------------------------------------------
# a frozen reading with no airflow move to measure (item 58)
# ---------------------------------------------------------------------------

#: Ambient step and when it comes, in the pinned-airflow scenario below.
AMBIENT_C = 25.0
AMBIENT_STEP_C = 6.0
AMBIENT_S = 600.0


def pinned_run(cfg: MpcConfig, sensor: str, ticks: int) -> DasRun:
    """``basic`` physics, the fans held where they started by a rate limit that cannot move
    them across a window, and the ambient stepped so the zone air rises on its own."""
    topology = topology_from_config(cfg)
    for entry in topology["sensors"].values():
        entry["noise_sigma_c"] = SENSOR_TYPES[entry["type"]].noise_sigma_c
    topology["inlet"] = {
        "base_c": AMBIENT_C,
        "schedule": [[AMBIENT_S, AMBIENT_C + AMBIENT_STEP_C]],
    }
    plant = build_das_plant(topology, preset="basic", dt=cfg.dt, initial_pwm=0.5)
    held: dict[str, float | None] = {}

    def freeze(i: int, obs: PlantObservation) -> PlantObservation:
        if obs.ts < FREEZE_S:
            return obs
        temps = dict(obs.temps)
        temps[sensor] = held.setdefault(sensor, temps[sensor])
        return dataclasses.replace(obs, temps=temps)

    return run_das_closed_loop(plant, cfg, step, ticks, observe_hook=freeze)


def test_a_frozen_reading_is_flagged_by_its_zone_air_while_the_airflow_stays_put(
    das_example_cfg,
):
    """Item 58: with the fans pinned there is no airflow move to be evidence, and before
    this rule the reading stayed trusted for as long as the run lasted. The zone air rises
    with the ambient at constant airflow, which a healthy proximal reading has to follow."""
    cfg = dataclasses.replace(das_example_cfg, d_pwm_max=1e-4)
    sensor = "prox_b01"
    params = cfg.stuck_params(sensor)
    window_s = params.ticks * cfg.dt
    run = pinned_run(cfg, sensor, int((AMBIENT_S + 2 * window_s) / cfg.dt))
    # the rate limit cannot move a channel by stuck_airflow_net inside one window, and the
    # run really stays under it: no airflow evidence exists anywhere in it
    assert params.ticks * cfg.d_pwm_max < cfg.stuck_airflow_net
    pwm = [r.cmd.pwm for r in run.records]
    span = max(
        abs(pwm[i][ch] - pwm[i - params.ticks][ch])
        for i in range(params.ticks, len(pwm))
        for ch in cfg.channels
    )
    assert span < cfg.stuck_airflow_net, span
    flags = [r.obs.ts for r in run.records if r.cmd.diagnostics["gate"]["stuck"][sensor]]
    assert flags, f"{sensor} frozen from {FREEZE_S} s was never flagged"
    assert flags[0] <= AMBIENT_S + 2 * window_s
    others = sorted(
        {
            name
            for r in run.records
            for name, stuck in r.cmd.diagnostics["gate"]["stuck"].items()
            if stuck and name != sensor
        }
    )
    assert others == [], f"healthy sensors flagged Stuck: {others}"
    assert {z for r in run.records for z in r.cmd.diagnostics["zones_in_fault"]} == {"z0"}
    # and the flag is this rule's: with a threshold the zone air never reaches (the rule
    # as it stood before item 58) the dead sensor stays trusted for the whole run
    without = pinned_run(
        dataclasses.replace(cfg, stuck_zone_air_dT_c=90.0), sensor, len(run.records)
    )
    assert not any(r.cmd.diagnostics["gate"]["stuck"][sensor] for r in without.records)


#: Drift injected on one zone-air sensor below, and when it starts (section 4.4 Drift).
DRIFT_C_PER_MIN = 0.3
DRIFT_S = 300.0


def test_a_drifting_zone_air_sensor_never_brands_its_zones_readings_stuck(das_example_cfg):
    """Section 4.4 Drift on a zone-air sensor, the false positive item 58's evidence could
    have invented: air_z0 walks away from the truth while the enclosure is healthy, so every
    proximal reading of z0 is correctly still. Only the drifting sensor's own rules may
    answer it -- the zone-air move is evidence against a reading only once another bay's
    reading has followed it, and here none does."""
    cfg = das_example_cfg
    plant = build_das_plant(topology_from_config(cfg), preset="basic", dt=cfg.dt, initial_pwm=0.5)

    def drift(i: int, obs: PlantObservation) -> PlantObservation:
        if obs.ts < DRIFT_S or obs.temps.get("air_z0") is None:
            return obs
        temps = dict(obs.temps)
        temps["air_z0"] += DRIFT_C_PER_MIN * (obs.ts - DRIFT_S) / 60.0
        return dataclasses.replace(obs, temps=temps)

    run = run_das_closed_loop(plant, cfg, step, 3000, observe_hook=drift)
    # the lie really is far past the threshold the rule reads
    assert DRIFT_C_PER_MIN * cfg.stuck_params("prox_b01").ticks * cfg.dt / 60.0 > (
        cfg.stuck_zone_air_dT_c
    )
    proximals = {t for t in cfg.temps if cfg.sensors[t].role == "drive_proximal"}
    flagged = sorted(
        {
            name
            for r in run.records
            for name, stuck in r.cmd.diagnostics["gate"]["stuck"].items()
            if stuck and name in proximals
        }
    )
    assert flagged == [], f"healthy proximal readings flagged Stuck: {flagged}"
    assert {z for r in run.records for z in r.cmd.diagnostics["zones_in_fault"]} == set()
    assert {r.cmd.mode for r in run.records} == {Mode.AUTO}


#: Freeze point and length of the coverage runs below: 20 minutes into 2.5 hours.
RICH_FREEZE_S = 1200.0
RICH_TICKS = 1800
#: Proximal readings the rules flag per seed, of the 17 the example config has. With the
#: airflow move as the only zoned evidence (before item 58) the counts were 6, 7 and 4, so
#: the whole gain is seed 0's: its drawn inlet drifts far enough that every zone's air
#: moves past stuck_zone_air_dT_c inside a window. Seed 1's zone air does move, but stays
#: under 1.5 degC within a window -- at stuck_zone_air_dT_c=1.25 this sweep gives 17, 16, 4
#: (37 of 51), so seed 1 is entirely a question of that margin, not of a flat ambient (item
#: 97 asks the owner to measure it on the enclosure). Seed 2 gains nothing. The rule only
#: adds evidence and takes none away.
RICH_FROZEN_FLAGGED = {0: 17, 1: 7, 2: 4}


@pytest.mark.nightly
@pytest.mark.parametrize("seed", sorted(RICH_FROZEN_FLAGGED))
def test_frozen_proximal_readings_are_flagged_on_rich_runs(das_example_cfg, seed):
    """Coverage of the Stuck evidence: freeze each proximal reading in turn 20 minutes into
    a 2.5-hour run and count the ones the rules catch (PROJECT.md section 3, item 58). No
    other sensor of those runs may be flagged: a fault must not be invented on a healthy
    one, whatever the frozen sensor does to the zone it is in."""
    cfg = das_example_cfg
    proximals = [t for t in cfg.temps if cfg.sensors[t].role == "drive_proximal"]
    flagged, false_flags = [], set()
    for name in proximals:
        run = rich_run(cfg, seed, RICH_TICKS, freeze=name, freeze_s=RICH_FREEZE_S)
        stuck = {
            other
            for rec in run.records
            for other, is_stuck in rec.cmd.diagnostics["gate"]["stuck"].items()
            if is_stuck
        }
        if name in stuck:
            flagged.append(name)
        false_flags |= stuck - {name}
    assert false_flags == set(), f"healthy sensors flagged Stuck: {sorted(false_flags)}"
    assert len(flagged) >= RICH_FROZEN_FLAGGED[seed], (seed, flagged)
