#!/usr/bin/env python3
"""Benchmark ``mpc.step`` per solver in a closed loop against a simulated plant.

Runs ``N`` closed-loop ticks for every solver (``pi`` and ``mpc``) with the
``mpc:`` section of a config file and prints one JSON document with the
mean and p99 milliseconds per ``step()`` and the config used. No pytest,
no hardware; only numpy and PyYAML (the Pi's apt packages).

Usage::

    python tools/bench_step.py [--config config.example.yaml] [--ticks 600]
                               [--solver pi --solver mpc] [--noise 0.2]
    python tools/bench_step.py --sim-plant das [--config config.example-das.yaml]
                               [--ticks 600] [--solver pi --solver mpc]
                               [--sim-preset basic|rich]

The loop includes the sensor gate, the fault bookkeeping and the solver,
i.e. exactly what runs on the Pi once per ``dt``; the plant simulation is
timed separately and excluded from the ``step()`` numbers.

``--sim-plant basic`` (default) is the legacy RC plant with a legacy config.
``--sim-plant das`` runs a zoned config (default ``config.example-das.yaml``)
against the DAS truth plant (``aqua_bridge.sim.das``, ``--sim-preset`` (default
``basic``) with each sensor type's white noise, activity bursts in three bays
and an inlet step; ``rich`` additionally draws the unknown physical
parameters -- placement offsets, drive/fan spread, fouling, inlet drift and
more -- from the run's seed instead of using nominal values, PROJECT.md
section 8 item 17): the ``pi`` solver is the PI-like DAS form and ``mpc``
the DAS MPC (``control/solver_das.py``), benchmarked with
``model_accept_prior: true`` so its MPC path is timed rather than its PI-like
fallback (``model_active_fraction`` reports the share of ticks the MPC drove
the fans). The step then includes the estimator; ``solve_p99_ms`` is the p99
over the ticks on which the MPC solved (``mpc_every_ticks``), ``budget_ms`` /
``budget_alarm_ms`` the config's per-tick gate (``mpc.budget_ms`` /
``mpc.budget_alarm_ms``, at ``dt = 5 s`` in the DAS example) -- read from the
loaded config, same as the runtime alarm in ``control/loop.py``, and not
enforced in ``step`` itself.
"""

from __future__ import annotations

import argparse
import dataclasses
import json
import math
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


def das_plant(
    cfg: MpcConfig, ticks: int, seed: int, *, preset: str = "basic"
):  # -> DasPlant (lazy import)
    """The DAS truth plant of the benchmark (module docstring)."""
    from aqua_bridge.sim.das import SENSOR_TYPES, build_das_plant, topology_from_config

    topology = topology_from_config(cfg)
    for entry in topology["sensors"].values():
        entry["noise_sigma_c"] = SENSOR_TYPES[entry["type"]].noise_sigma_c
    span = ticks * cfg.dt
    topology["inlet"] = {"base_c": 25.0, "schedule": [[0.5 * span, 28.0]]}
    bays = list(topology["bays"])
    heat = {
        bays[1 % len(bays)]: [(0.1 * span, 1.0), (0.6 * span, 0.0)],
        bays[len(bays) // 2]: [(0.3 * span, 1.0)],
        bays[-1]: [(0.0, 0.5), (0.7 * span, 1.0)],
    }
    return build_das_plant(
        topology, preset=preset, dt=cfg.dt, initial_pwm=0.5, seed=seed, heat_schedule=heat
    )


def bench_das_solver(
    cfg: MpcConfig, ticks: int, *, seed: int, preset: str = "basic"
) -> dict[str, object]:
    """Closed loop of ``ticks`` steps on the DAS truth plant (module docstring)."""
    plant = das_plant(cfg, ticks, seed, preset=preset)
    state = MpcState.cold()
    times_ms: list[float] = []
    solve_ms: list[float] = []
    modes = dict.fromkeys((m.value for m in Mode), 0)
    iterations: list[int] = []
    active = 0
    noise_power = 0.0
    worst_margin = math.inf
    for _ in range(ticks):
        obs = plant.observe()
        smart = plant.observe_smart()
        if smart:
            obs = dataclasses.replace(obs, inputs={"smart": smart})
        t0 = time.perf_counter()
        cmd, state = step(obs, cfg, state)
        elapsed = (time.perf_counter() - t0) * 1e3
        times_ms.append(elapsed)
        modes[cmd.mode.value] += 1
        diag = cmd.diagnostics.get("solver_diag", {})
        if diag.get("solved"):
            solve_ms.append(elapsed)
        it = diag.get("iterations")
        if isinstance(it, int):
            iterations.append(it)
        if diag.get("model", {}).get("active") == "mpc":
            active += 1
        noise_power += 10.0 ** (plant.noise_db() / 10.0)
        margins = [m for m in plant.margins().values() if m is not None]
        if margins:
            worst_margin = min(worst_margin, min(margins))
        plant.apply(cmd.pwm)
        plant.advance()
    p99 = percentile(times_ms, 99)
    return {
        "ticks": ticks,
        "mean_ms": statistics.fmean(times_ms),
        "p50_ms": percentile(times_ms, 50),
        "p99_ms": p99,
        "max_ms": max(times_ms),
        "first_ms": times_ms[0],
        "solve_ticks": len(solve_ms),
        "solve_p99_ms": percentile(solve_ms, 99) if solve_ms else None,
        "modes": modes,
        "solver_iterations_max": max(iterations) if iterations else None,
        "model_active_fraction": active / ticks if cfg.solver is SolverKind.MPC else None,
        "noise_db_mean": 10.0 * math.log10(noise_power / ticks),
        "worst_true_margin_c": worst_margin,
        "final_pwm": dict(state.last_cmd.pwm) if state.last_cmd else None,
        "budget_fraction_of_dt_p99": p99 / (cfg.dt * 1e3),
        "budget_ms": cfg.budget_ms,
        "budget_alarm_ms": cfg.budget_alarm_ms,
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--config", default=None, help="YAML with an mpc: section")
    parser.add_argument("--ticks", type=int, default=600, help="closed-loop ticks per solver")
    parser.add_argument(
        "--solver",
        action="append",
        choices=[k.value for k in SolverKind],
        help="solver(s) to benchmark; default: all",
    )
    parser.add_argument(
        "--sim-plant",
        choices=("basic", "das"),
        default="basic",
        help="basic: legacy RC plant and config; das: DAS truth plant and a zoned config",
    )
    parser.add_argument(
        "--sim-preset",
        default="basic",
        help="--sim-plant das only: aqua_bridge.sim.das preset, 'basic' or 'rich' (default: basic)",
    )
    parser.add_argument("--noise", type=float, default=0.2, help="sensor noise sigma, degrees C")
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--heat-w", type=float, default=100.0, help="plant heat load, W")
    args = parser.parse_args(argv)

    das = args.sim_plant == "das"
    default_config = "config.example-das.yaml" if das else "config.example.yaml"
    config_path = args.config or str(REPO_ROOT / default_config)
    base = load_config(config_path).mpc
    if das and not base.regulates_drive_limits:
        parser.error("--sim-plant das needs a zoned config without setpoints (mpc.topology)")
    if das:
        from aqua_bridge.sim.das import PRESETS

        if args.sim_preset not in PRESETS:
            parser.error(f"--sim-preset must be one of {PRESETS}, got {args.sim_preset!r}")
    kinds = [SolverKind(k) for k in args.solver] if args.solver else list(SolverKind)
    results: dict[str, object] = {}
    for kind in kinds:
        cfg = dataclasses.replace(base, solver=kind)
        if das:
            if kind is SolverKind.MPC:
                cfg = dataclasses.replace(cfg, model_accept_prior=True)
            results[kind.value] = bench_das_solver(
                cfg, args.ticks, seed=args.seed, preset=args.sim_preset
            )
        else:
            results[kind.value] = bench_solver(
                cfg, args.ticks, noise=args.noise, seed=args.seed, heat_w=args.heat_w
            )
    report = {
        "python": platform.python_version(),
        "numpy": np.__version__,
        "machine": platform.machine(),
        "config_path": str(config_path),
        "config": base.to_dict(),
        "sim_plant": args.sim_plant,
        "plant": (
            {"preset": args.sim_preset, "sensor_noise": "per type", "seed": args.seed}
            if das
            else {"noise_sigma_c": args.noise, "heat_w": args.heat_w, "delay_ticks": 1}
        ),
        "results": results,
    }
    json.dump(report, sys.stdout, indent=2, sort_keys=True)
    sys.stdout.write("\n")
    return 0


if __name__ == "__main__":
    sys.exit(main())
