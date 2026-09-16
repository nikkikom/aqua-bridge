"""Identifiability of the zoned thermal model on the DAS truth simulator (plan section 9).

The truth (``aqua_bridge.sim.das``) is built from the example DAS config with
parameters the controller does not know: every output's effectiveness and every
drive's airflow-dependent conductance ``k`` spread by up to +-30 %, a leak 50 %
above the prior, the declared weak cross-zone airflow present in the truth,
realistic sensing (thermistor 0.01 degC / DS18B20 0.0625 degC quantisation with
white noise, 5 s / 15 s sensor lags) and realistic placement of the proximal
sensors (offset -2.1 +- 0.3 degC and air fraction 0.3 +- 0.05 around the prior
map). The zone-air and inlet sensors agree with each other: a relative offset of
0.1 degC against a DAS zone's 0.3-1 degC air rise moves ``E`` by 15-35 % (module
docstring of :mod:`aqua_bridge.control.thermal`), which the nightly sweep records.

PR cases (the plan's four): with group experiments (independent two-level
pseudo-random sequences on every output, holds of 60/120/180 s, after 30 min
settling, 8 h in total, one fixed seed) every in-zone ``E`` is within 15 % and every
per-bay ``k`` within 25 % of the truth, ``leak`` and ``kappa`` stay at their prior,
and the model reaches ``converged``; with regulation only (PI-DAS through ``step``,
heat bursts and inlet steps, 3 h) it never reaches ``converged``. The nightly sweep
repeats the experiment over 8 seeds (``E`` within 25 %: observed 9-22 % at 8 h, the worst a
channel shared by two zones in a zone with three groups), the
air/inlet offset case (``k`` still within 25 %) and regulation only on the ``rich``
preset.
"""

from __future__ import annotations

import dataclasses
from collections import Counter
from typing import Any

import numpy as np
import pytest

from aqua_bridge.control import thermal
from aqua_bridge.control.mpc import step
from aqua_bridge.model import MpcConfig
from aqua_bridge.sim.das import (
    DasPlant,
    build_das_plant,
    run_das_closed_loop,
    topology_from_config,
)

PR_SEED = 2
EXPERIMENT_HOURS = 8.0
SETTLE_S = 1800.0
LEVELS = (0.35, 0.8)
HOLDS_S = (60.0, 120.0, 180.0)


def _config(example: MpcConfig) -> MpcConfig:
    return dataclasses.replace(example, model_shadow=True)


def truth_topology(
    cfg: MpcConfig, seed: int, *, spread: float = 0.3, air_offset_c: float = 0.0
) -> dict[str, Any]:
    """The simulator topology described in the module docstring."""
    topo = topology_from_config(cfg)
    zones = cfg.topology.zones
    rng = np.random.default_rng(seed)
    for ch, fan in topo["fans"].items():
        listed = [z for z, spec in zones.items() if ch in spec.channels]
        shares = dict.fromkeys(listed, 1.0 / len(listed))
        for z in listed:
            for other in zones[z].coupled_to:
                if other not in listed:
                    shares[other] = shares.get(other, 0.0) + thermal.WEAK_E_FACTOR / len(listed)
        fan["zones"] = shares
        fan["e_w_per_k"] = 33.0 * float(rng.uniform(1 - spread, 1 + spread))
    for bay in topo["bays"].values():
        bay["k_w_per_k"] = 0.5 * float(rng.uniform(1 - spread, 1 + spread))
        bay["c_j_per_k"] = 576.0 if bay["class"] == "hdd" else 160.0  # the class prior C_d
    for sensor in topo["sensors"].values():
        if sensor["role"] == "drive_proximal":
            sensor["offset_c"] = -2.1 + float(rng.uniform(-0.3, 0.3))
            sensor["beta"] = 0.3 + float(rng.uniform(-0.05, 0.05))
        elif sensor["role"] == "zone_air":
            sensor["offset_c"] = float(rng.uniform(-air_offset_c, air_offset_c))
        sensor["noise_sigma_c"] = 0.02 if sensor["type"] == "thermistor" else 0.03
    for zone in topo["zones"].values():
        zone["leak_w_per_k"] = 1.5
    return topo


def run_experiment(
    cfg: MpcConfig, seed: int, hours: float = EXPERIMENT_HOURS, **truth: Any
) -> tuple[dict[str, Any], DasPlant, Counter[str]]:
    """Open-loop group experiments with :func:`thermal.update` fed like ``step`` feeds it."""
    plant = build_das_plant(
        truth_topology(cfg, seed, **truth), preset="basic", seed=seed, dt=cfg.dt, initial_pwm=0.6
    )
    rng = np.random.default_rng(1000 + seed)
    level = dict.fromkeys(cfg.channels, 0.6)
    applied = dict(level)
    next_switch = dict.fromkeys(cfg.channels, SETTLE_S)
    zones_ok = set(cfg.zone_layout.zones)
    memory = None
    statuses: Counter[str] = Counter()
    summary: dict[str, Any] = {}
    for _ in range(int(hours * 3600.0 / cfg.dt)):
        ts = plant.ts
        for ch in cfg.channels:
            if ts >= next_switch[ch]:
                level[ch] = float(rng.choice(LEVELS))
                next_switch[ch] = ts + float(rng.choice(HOLDS_S))
        obs = plant.observe()
        temps = {name: value for name, value in obs.temps.items() if value is not None}
        out = thermal.update(memory, cfg, temps=temps, u=applied, ts=ts, zones_ok=zones_ok)
        memory, summary = out.memory, out.summary
        statuses[summary["status"]] += 1
        plant.apply(level)
        applied = dict(level)
        plant.advance()
    return summary, plant, statuses


def e_errors(cfg: MpcConfig, summary: dict[str, Any], plant: DasPlant) -> dict[str, float]:
    """Relative error of every in-zone effectiveness against the truth."""
    fans = {f.name: f for f in plant.params.fans}
    out = {}
    for z, zone in summary["zones"].items():
        for key, value in zone["theta"].items():
            if not key.startswith("E."):
                continue
            ch = key.split(".")[2]
            if ch not in cfg.topology.zones[z].channels:
                continue
            f = fans[ch]
            truth = f.e_w_per_k * f.count * f.shares[z]
            out[key] = (value - truth) / truth
    return out


def k_errors(summary: dict[str, Any], plant: DasPlant) -> dict[str, float]:
    drives = {b.name: b.drive for b in plant.params.bays}
    return {
        b: (bay["theta"][f"k.{b}"] - drives[b].k_w_per_k) / drives[b].k_w_per_k
        for b, bay in summary["bays"].items()
    }


@pytest.fixture(scope="module")
def experiment() -> tuple[MpcConfig, dict[str, Any], DasPlant, Counter[str]]:
    from aqua_bridge.config import load_config
    from conftest import EXAMPLE_DAS_CONFIG

    cfg = _config(load_config(EXAMPLE_DAS_CONFIG).mpc)
    summary, plant, statuses = run_experiment(cfg, PR_SEED)
    return cfg, summary, plant, statuses


def test_group_experiments_identify_every_in_zone_effectiveness_within_15_percent(experiment):
    cfg, summary, plant, _ = experiment
    errors = e_errors(cfg, summary, plant)
    assert len(errors) == sum(len(z.channels) for z in cfg.topology.zones.values())
    worst = max(errors, key=lambda k: abs(errors[k]))
    assert abs(errors[worst]) < 0.15, (worst, errors)


def test_group_experiments_identify_every_per_bay_k_within_25_percent(experiment):
    _, summary, plant, _ = experiment
    errors = k_errors(summary, plant)
    worst = max(errors, key=lambda k: abs(errors[k]))
    assert abs(errors[worst]) < 0.25, (worst, errors)


def test_leak_and_kappa_stay_at_their_prior(experiment):
    cfg, summary, _, _ = experiment
    prior = thermal.prior_theta(cfg)
    for zone in summary["zones"].values():
        for key, value in zone["theta"].items():
            if key.startswith(("leak.", "kappa.")):
                # the truth's leak is 1.5x the prior: the ridge holds the weak direction
                assert value == pytest.approx(prior[key], rel=0.1), key


def test_group_experiments_converge(experiment):
    cfg, summary, _, statuses = experiment
    assert summary["status"] == "converged", {z: v["status"] for z, v in summary["zones"].items()}
    assert statuses["converged"] > 0 and statuses["error"] == 0 and statuses["suspect"] == 0
    assert summary["pred_err_c"] < cfg.model_max_pred_err_c
    for zone in summary["zones"].values():
        assert zone["pe_min"] > thermal.PE_MIN


def _regulation_run(cfg: MpcConfig, *, preset: str, seed: int, hours: float):
    topo = topology_from_config(cfg)
    bays = list(cfg.topology.bays)
    rng = np.random.default_rng(seed)
    heat = {
        b: [(float(t), float(rng.choice([0.0, 1.0]))) for t in range(0, int(hours * 3600), 900)]
        for b in bays
    }
    if preset == "basic":  # the rich preset draws its own bursts and inlet drift
        topo["inlet"] = {"schedule": [[3600.0, 27.0], [7200.0, 24.0]]}
    plant = build_das_plant(
        topo, preset=preset, seed=seed, dt=cfg.dt, heat_schedule=heat if preset == "basic" else None
    )
    statuses: Counter[str] = Counter()

    def controller(obs, c, state):
        cmd, state = step(obs, c, state)
        statuses[cmd.diagnostics["thermal"]["status"]] += 1
        return cmd, state

    run = run_das_closed_loop(plant, cfg, controller, int(hours * 3600.0 / cfg.dt))
    return run, statuses


def test_regulation_only_never_converges(das_example_cfg):
    cfg = _config(das_example_cfg)
    run, statuses = _regulation_run(cfg, preset="basic", seed=3, hours=3.0)
    assert statuses["converged"] == 0, statuses
    assert statuses["learning"] > 0  # the loop did move the fans
    last = run.records[-1].cmd.diagnostics["thermal"]
    assert last["status"] in ("prior", "learning")
    assert run.violations() == 0


# ---------------------------------------------------------------------------
# nightly sweeps
# ---------------------------------------------------------------------------


@pytest.mark.nightly
@pytest.mark.parametrize("seed", range(1, 9))
def test_identification_sweep_over_seeds(das_example_cfg, seed):
    cfg = _config(das_example_cfg)
    summary, plant, statuses = run_experiment(cfg, seed)
    e = e_errors(cfg, summary, plant)
    k = k_errors(summary, plant)
    assert max(abs(v) for v in e.values()) < 0.25, e
    assert max(abs(v) for v in k.values()) < 0.25, k
    assert statuses["error"] == 0


@pytest.mark.nightly
@pytest.mark.parametrize("seed", [1, 2, 3])
def test_air_and_inlet_sensor_offsets_leave_k_identifiable(das_example_cfg, seed):
    cfg = _config(das_example_cfg)
    summary, plant, statuses = run_experiment(cfg, seed, air_offset_c=0.1)
    k = k_errors(summary, plant)
    assert max(abs(v) for v in k.values()) < 0.25, k
    for zone in summary["zones"].values():
        for key, value in zone["theta"].items():
            spec = thermal.PARAMETERS[key.split(".")[0]]
            assert spec.lo <= value <= spec.hi
    assert statuses["error"] == 0


@pytest.mark.nightly
@pytest.mark.parametrize("seed", [1, 2, 3])
def test_regulation_only_never_converges_on_the_rich_preset(das_example_cfg, seed):
    cfg = _config(das_example_cfg)
    run, statuses = _regulation_run(cfg, preset="rich", seed=seed, hours=4.0)
    assert statuses["converged"] == 0, statuses
    assert statuses["error"] == 0


# ---------------------------------------------------------------------------
# hot swap: the bay's coefficients start over (item 12)
# ---------------------------------------------------------------------------


def _example(**replace: Any) -> MpcConfig:
    from aqua_bridge.config import load_config
    from conftest import EXAMPLE_DAS_CONFIG

    return dataclasses.replace(_config(load_config(EXAMPLE_DAS_CONFIG).mpc), **replace)


def _reset_tick(cfg: MpcConfig, memory, bays: set[str]):
    return thermal.update(
        memory,
        cfg,
        temps={},
        u=dict.fromkeys(cfg.channels, 0.5),
        ts=10.0,
        zones_ok=(),
        reset_bays=bays,
    ).memory


def test_a_reset_bay_starts_over_and_leaves_every_other_block_alone():
    """``reset_bays`` restores one bay's prior; the zone's status and the other blocks
    (including the air block) are untouched."""
    cfg = _example()
    memory = thermal.fresh_memory(cfg)
    memory["zones"]["z1"]["status"] = "converged"
    for b in ("b06", "b05"):
        block = memory["bays"][b]
        block["theta"] = [v + 0.2 for v in block["theta"]]
        block["w"], block["n"], block["rel"] = 12, 9, [0.1] * len(block["theta"])
    air = memory["zones"]["z1"]["air"]
    air["w"] = 7
    before_air = list(air["theta"])
    prior = thermal.fresh_memory(cfg)["bays"]["b06"]["theta"]

    mem = _reset_tick(cfg, memory, {"b06"})
    assert mem["bays"]["b06"]["theta"] == prior
    assert mem["bays"]["b06"]["w"] == 0 and mem["bays"]["b06"]["rel"] is None
    assert mem["bays"]["b05"]["w"] == 12  # the other bay of the zone is untouched
    assert mem["zones"]["z1"]["status"] == "converged"  # the zone is not demoted
    assert mem["zones"]["z1"]["air"]["theta"] == before_air and mem["zones"]["z1"]["air"]["w"] == 7


def test_model_reset_on_swap_false_keeps_the_coefficients():
    cfg = _example(model_reset_on_swap=False)
    memory = thermal.fresh_memory(cfg)
    memory["bays"]["b06"]["w"] = 12
    assert _reset_tick(cfg, memory, {"b06"})["bays"]["b06"]["w"] == 12


def test_a_frozen_zone_is_never_reset():
    """A frozen zone never moves its coefficients, so a reset would strand the bay."""
    cfg = _example()
    memory = thermal.fresh_memory(cfg)
    memory["zones"]["z1"]["status"] = "frozen"
    memory["bays"]["b06"]["w"] = 12
    assert _reset_tick(cfg, memory, {"b06"})["bays"]["b06"]["w"] == 12
