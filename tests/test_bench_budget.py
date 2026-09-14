"""Step budget of the DAS MPC (plan sections 4 and 9, ``tests/test_bench_budget.py``).

Two gates on ``mpc.step`` in a closed loop, timed like ``tools/bench_step.py``:

* **relative, every CI run**: the p99 step time of the DAS MPC on
  ``config.example-das.yaml`` (15 bays, 25 sensors, 8 channels, the estimator every
  tick, the MPC every ``mpc_every_ticks``) against the DAS truth plant, divided by
  the p99 of the legacy MPC on ``config.example.yaml`` against the RC plant,
  measured in the same process. A ratio is what a CI runner can check; it tracks the
  Zero W's absolute numbers only as far as the plan's 100x factor holds for both.

  Method (item 6, robust on a slow shared runner): :data:`REPEATS` repeats, each
  timing both sides back to back with the garbage collector paused; the two sides
  alternate which one goes first every repeat (``legacy, das`` on even repeats,
  ``das, legacy`` on odd ones) so a runner hiccup during one repeat does not
  systematically favour either side. The first :data:`WARMUP_REPEATS` repeats
  (import / cache / branch-predictor warm-up) are discarded, and the gate compares
  the :data:`RELATIVE_PERCENTILE` percentile of the remaining per-repeat ratios
  (das p99 / legacy p99) against the named constant :data:`RELATIVE_FACTOR` -- a
  high percentile rather than the single worst repeat, so one outlier repeat does
  not flake the gate while a real regression across most repeats still trips it.

* **absolute, only on the Pi** (``armv6l``, marker ``pi``): the DAS MPC's p99 step
  time is at most ``mpc.budget_ms`` of ``config.example-das.yaml`` (the per-tick gate
  at ``dt = 5 s``; §8.1 owner decision 2026-09-14, was 500 ms). Run it there with
  ``pytest tests/test_bench_budget.py -m pi``.

The plan's second relative line (the gate with 30 sensors within 3x the solver-free step)
is not a separate test here: the gate is part of both steps measured above, a gate alone
is always cheaper than a step that contains it, and ``tests/test_gate.py`` checks that the
decimated Stuck windows keep their storage bounded. ``tools/bench_step.py --sim-plant das``
prints the same numbers for both DAS solvers.
"""

from __future__ import annotations

import dataclasses
import gc
import importlib.util
import json
import platform
import time

import pytest

from aqua_bridge.config import load_config
from aqua_bridge.control.mpc import step
from aqua_bridge.model import MpcConfig, MpcState, SolverKind
from aqua_bridge.sim.das import SENSOR_TYPES, build_das_plant, topology_from_config
from aqua_bridge.sim.plant import Plant, PlantParams
from conftest import EXAMPLE_CONFIG, EXAMPLE_DAS_CONFIG, REPO_ROOT

#: Plan section 9: DAS step p99 <= 12x the legacy MPC step p99.
#: Test parameter, not an operator setting (PROJECT.md "No hardcoded tunables"
#: applies to config the controller reads, not to a CI gate's own threshold).
RELATIVE_FACTOR = 12.0
#: Repeats of the relative comparison, and how many leading ones to discard as warm-up.
REPEATS = 5
WARMUP_REPEATS = 1
#: Percentile of the (REPEATS - WARMUP_REPEATS) per-repeat ratios the gate checks.
RELATIVE_PERCENTILE = 75.0
TICKS = 240
WARMUP = 20


def p99(samples: list[float]) -> float:
    ordered = sorted(samples)
    return ordered[min(len(ordered) - 1, round(0.99 * (len(ordered) - 1)))]


def legacy_mpc_times(ticks: int = TICKS) -> list[float]:
    cfg = dataclasses.replace(load_config(EXAMPLE_CONFIG).mpc, solver=SolverKind.MPC)
    plant = Plant(
        PlantParams(dt=cfg.dt, heat_w=100.0, noise_sigma_c=0.2, delay_ticks=1),
        initial_pwm=0.5,
        t_coolant=40.0,
        t_air=30.0,
        seed=7,
    )
    return _timed(cfg, plant, ticks)


def das_mpc_config() -> MpcConfig:
    return dataclasses.replace(
        load_config(EXAMPLE_DAS_CONFIG).mpc, solver=SolverKind.MPC, model_accept_prior=True
    )


def das_mpc_times(cfg: MpcConfig, ticks: int = TICKS) -> list[float]:
    topology = topology_from_config(cfg)
    for entry in topology["sensors"].values():
        entry["noise_sigma_c"] = SENSOR_TYPES[entry["type"]].noise_sigma_c
    plant = build_das_plant(
        topology,
        preset="basic",
        dt=cfg.dt,
        initial_pwm=0.5,
        seed=7,
        heat_schedule={"b02": [(0.0, 1.0)], "b10": [(300.0, 1.0)], "b13": [(0.0, 0.5)]},
    )
    return _timed(cfg, plant, ticks)


def _timed(cfg: MpcConfig, plant, ticks: int) -> list[float]:
    state = MpcState.cold()
    times: list[float] = []
    enabled = gc.isenabled()
    gc.disable()
    try:
        for i in range(ticks):
            obs = plant.observe()
            t0 = time.perf_counter()
            cmd, state = step(obs, cfg, state)
            elapsed = (time.perf_counter() - t0) * 1e3
            if i >= WARMUP:
                times.append(elapsed)
            plant.apply(cmd.pwm)
            plant.advance()
    finally:
        if enabled:
            gc.enable()
    return times


def percentile(samples: list[float], q: float) -> float:
    """Nearest-rank percentile (``q`` in [0, 100]); used on the per-repeat ratios."""
    ordered = sorted(samples)
    k = max(0, min(len(ordered) - 1, round(q / 100.0 * (len(ordered) - 1))))
    return ordered[k]


def test_das_mpc_step_p99_within_the_relative_budget():
    """Method: module docstring "relative, every CI run"."""
    cfg = das_mpc_config()
    ratios: list[float] = []
    runs: list[tuple[float, float]] = []  # (das_p99, legacy_p99), every repeat, for the message
    for i in range(REPEATS):
        if i % 2 == 0:
            legacy_ms = p99(legacy_mpc_times())
            das_ms = p99(das_mpc_times(cfg))
        else:
            das_ms = p99(das_mpc_times(cfg))
            legacy_ms = p99(legacy_mpc_times())
        runs.append((das_ms, legacy_ms))
        if i >= WARMUP_REPEATS:
            ratios.append(das_ms / legacy_ms)
    ratio = percentile(ratios, RELATIVE_PERCENTILE)
    assert ratio <= RELATIVE_FACTOR, (
        f"DAS MPC step p99 is {RELATIVE_PERCENTILE:.0f}th percentile {ratio:.1f}x the legacy "
        f"MPC step p99 (budget {RELATIVE_FACTOR}x); per-repeat (das_ms, legacy_ms): {runs}"
    )


@pytest.mark.pi
@pytest.mark.skipif(platform.machine() != "armv6l", reason="absolute budget: Raspberry Pi only")
def test_das_mpc_step_p99_within_budget_ms_on_the_pi():
    cfg = das_mpc_config()
    times = das_mpc_times(cfg, ticks=200)
    assert p99(times) <= cfg.budget_ms, f"DAS MPC step p99 {p99(times):.0f} ms > {cfg.budget_ms} ms"


def test_bench_tool_runs_both_das_solvers_on_the_das_plant(capsys):
    cfg = das_mpc_config()
    spec = importlib.util.spec_from_file_location(
        "bench_step", REPO_ROOT / "tools" / "bench_step.py"
    )
    assert spec is not None and spec.loader is not None
    tool = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(tool)
    assert tool.main(["--sim-plant", "das", "--ticks", "12"]) == 0
    report = json.loads(capsys.readouterr().out)
    assert report["sim_plant"] == "das" and set(report["results"]) == {"pi", "mpc"}
    mpc = report["results"]["mpc"]
    assert mpc["model_active_fraction"] == 1.0 and mpc["solve_ticks"] >= 6
    assert mpc["budget_ms"] == cfg.budget_ms and report["results"]["pi"]["solve_ticks"] == 0
    assert mpc["budget_alarm_ms"] == cfg.budget_alarm_ms
