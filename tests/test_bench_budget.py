"""Step budget of the DAS MPC (plan sections 4 and 9, ``tests/test_bench_budget.py``).

Two gates on ``mpc.step`` in a closed loop, timed like ``tools/bench_step.py``:

* **relative, every CI run**: the p99 step time of the DAS MPC on
  ``config.example-das.yaml`` (15 bays, 25 sensors, 8 channels, the estimator every
  tick, the MPC every ``mpc_every_ticks``) against the DAS truth plant is at most
  :data:`RELATIVE_FACTOR` times the p99 of the legacy MPC on ``config.example.yaml``
  against the RC plant, measured in the same process. A ratio is what a CI runner can
  check; it tracks the Zero W's absolute numbers only as far as the plan's 100x
  factor holds for both. Each side is the best of :data:`REPEATS` interleaved
  measurements with the garbage collector paused, so a scheduler hiccup on a shared
  runner does not decide the verdict.
* **absolute, only on the Pi** (``armv6l``, marker ``pi``): the DAS MPC's p99 step time
  is at most :data:`BUDGET_MS` (the plan's hard per-tick gate at ``dt = 5 s``, 10 % of
  the tick). Run it there with ``pytest tests/test_bench_budget.py -m pi``.

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
RELATIVE_FACTOR = 12.0
#: Plan section 4: per-tick gate at dt = 5 s on the Zero W, ms.
BUDGET_MS = 500.0
REPEATS = 3
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


def test_das_mpc_step_p99_within_the_relative_budget():
    cfg = das_mpc_config()
    legacy: list[float] = []
    das: list[float] = []
    for _ in range(REPEATS):
        legacy.append(p99(legacy_mpc_times()))
        das.append(p99(das_mpc_times(cfg)))
    ratio = min(das) / min(legacy)
    assert ratio <= RELATIVE_FACTOR, (
        f"DAS MPC step p99 {min(das):.2f} ms is {ratio:.1f}x the legacy MPC step p99 "
        f"{min(legacy):.3f} ms (budget {RELATIVE_FACTOR}x); runs: das {das}, legacy {legacy}"
    )


@pytest.mark.pi
@pytest.mark.skipif(platform.machine() != "armv6l", reason="absolute budget: Raspberry Pi only")
def test_das_mpc_step_p99_within_budget_ms_on_the_pi():
    times = das_mpc_times(das_mpc_config(), ticks=200)
    assert p99(times) <= BUDGET_MS, f"DAS MPC step p99 {p99(times):.0f} ms > {BUDGET_MS} ms"


def test_bench_tool_runs_both_das_solvers_on_the_das_plant(capsys):
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
    assert mpc["budget_ms"] == BUDGET_MS and report["results"]["pi"]["solve_ticks"] == 0
    assert tool.BUDGET_MS == BUDGET_MS
