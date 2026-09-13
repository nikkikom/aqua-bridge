#!/usr/bin/env python3
"""Benchmark ``mpc.step`` per solver in a closed loop against the RC plant.

Runs ``N`` closed-loop ticks for every solver (``pi`` and ``mpc``) with the
``mpc:`` section of a config file and prints one JSON document with the
mean and p99 milliseconds per ``step()`` and the config used. No pytest,
no hardware; only numpy and PyYAML (the Pi's apt packages).

Usage::

    python tools/bench_step.py [--config config.example.yaml] [--ticks 600]
                               [--solver pi --solver mpc] [--noise 0.2]

The loop includes the sensor gate, the fault bookkeeping and the solver,
i.e. exactly what runs on the Pi once per ``dt``; the plant simulation is
timed separately and excluded from the ``step()`` numbers.
"""

from __future__ import annotations

import argparse
import dataclasses
import json
import platform
import statistics
import sys
import time
from pathlib import Path

import numpy as np

from aqua_bridge.config import load_config
from aqua_bridge.control.mpc import step
from aqua_bridge.model import Mode, MpcConfig, MpcState, SolverKind
from aqua_bridge.sim.plant import Plant, PlantParams

REPO_ROOT = Path(__file__).resolve().parent.parent


def percentile(samples: list[float], q: float) -> float:
    """Nearest-rank percentile (``q`` in [0, 100])."""
    ordered = sorted(samples)
    k = max(0, min(len(ordered) - 1, int(round(q / 100.0 * (len(ordered) - 1)))))
    return ordered[k]


def bench_solver(
    cfg: MpcConfig, ticks: int, *, noise: float, seed: int, heat_w: float
) -> dict[str, object]:
    """Closed loop of ``ticks`` steps; returns timing and behaviour summary."""
    plant = Plant(
        PlantParams(dt=cfg.dt, heat_w=heat_w, noise_sigma_c=noise, delay_ticks=1),
        initial_pwm=0.5,
        t_coolant=40.0,
        t_air=30.0,
        seed=seed,
    )
    state = MpcState.cold()
    times_ms: list[float] = []
    modes = dict.fromkeys((m.value for m in Mode), 0)
    iterations: list[int] = []
    for i in range(ticks):
        if i == ticks // 3:
            plant.heat_w = heat_w * 1.3  # disturbance: the solver has to work
        if i == 2 * ticks // 3:
            plant.heat_w = heat_w
        obs = plant.observe()
        t0 = time.perf_counter()
        cmd, state = step(obs, cfg, state)
        times_ms.append((time.perf_counter() - t0) * 1e3)
        modes[cmd.mode.value] += 1
        it = cmd.diagnostics.get("solver_diag", {}).get("iterations")
        if isinstance(it, int):
            iterations.append(it)
        plant.apply(cmd.pwm)
        plant.advance()
    return {
        "ticks": ticks,
        "mean_ms": statistics.fmean(times_ms),
        "p50_ms": percentile(times_ms, 50),
        "p99_ms": percentile(times_ms, 99),
        "max_ms": max(times_ms),
        "first_ms": times_ms[0],  # includes matrix building / cache warm-up
        "modes": modes,
        "solver_iterations_max": max(iterations) if iterations else None,
        "final_coolant_c": plant.t_coolant,
        "final_pwm": dict(state.last_cmd.pwm) if state.last_cmd else None,
        "budget_fraction_of_dt_p99": percentile(times_ms, 99) / (cfg.dt * 1e3),
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument(
        "--config", default=str(REPO_ROOT / "config.example.yaml"), help="YAML with an mpc: section"
    )
    parser.add_argument("--ticks", type=int, default=600, help="closed-loop ticks per solver")
    parser.add_argument(
        "--solver",
        action="append",
        choices=[k.value for k in SolverKind],
        help="solver(s) to benchmark; default: all",
    )
    parser.add_argument("--noise", type=float, default=0.2, help="sensor noise sigma, degrees C")
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--heat-w", type=float, default=100.0, help="plant heat load, W")
    args = parser.parse_args(argv)

    base = load_config(args.config).mpc
    kinds = [SolverKind(k) for k in args.solver] if args.solver else list(SolverKind)
    results: dict[str, object] = {}
    for kind in kinds:
        cfg = dataclasses.replace(base, solver=kind)
        results[kind.value] = bench_solver(
            cfg, args.ticks, noise=args.noise, seed=args.seed, heat_w=args.heat_w
        )
    report = {
        "python": platform.python_version(),
        "numpy": np.__version__,
        "machine": platform.machine(),
        "config_path": str(args.config),
        "config": base.to_dict(),
        "plant": {"noise_sigma_c": args.noise, "heat_w": args.heat_w, "delay_ticks": 1},
        "results": results,
    }
    json.dump(report, sys.stdout, indent=2, sort_keys=True)
    sys.stdout.write("\n")
    return 0


if __name__ == "__main__":
    sys.exit(main())
