#!/usr/bin/env python3
"""Benchmark ``model.json`` writes (``aqua_bridge.modelstore``) at the size the
store actually reaches, against a **scratch path only** (PROJECT.md section 8
item 48).

Builds one realistic snapshot -- a DAS closed loop of ``--warm-ticks`` steps
against ``config.example-das.yaml`` (or ``--config``), so the estimator's
calibration, the thermal model and (with ``--sim-preset rich``) more of what a
running daemon actually accumulates are in ``solver_memory`` before anything
is timed, not an empty cold-start document -- then serialises it exactly as
:meth:`aqua_bridge.modelstore.ModelPersister.save` does
(:func:`aqua_bridge.modelstore.build_document`, then
``json.dumps(doc, allow_nan=False, separators=(",", ":"))``) and times
:func:`aqua_bridge.modelstore.write_atomic` -- open, write, ``fsync``,
``os.replace``, ``fsync`` of the directory, the same call the daemon makes --
against ``--path`` for ``--repeats`` repeats.

Reports the size in bytes, the min / mean / median / p99 / max write time in
milliseconds, the spread (max - min) over the repeats, and that p99 as a
fraction of ``dt`` (the tick period) and of ``mpc.model_store_interval_s``
(how often the daemon actually writes; a write happens on at most one tick out
of every ``model_store_interval_s / dt`` of them, and the write itself never
delays the fan command -- ``control/loop.py`` runs ``on_tick`` observers,
model.json's persister included, only after ``apply()`` has already sent that
tick's PWM and after the step-budget gate has already been measured and
recorded on ``step()`` alone -- but a slow write still runs inside ``tick()``
and so delays the *next* tick's read, which is what the fraction of
``dt`` is checking).

**Safety**: refuses to write anywhere but under a scratch directory
(``tempfile.gettempdir()`` or ``/tmp``) -- never the daemon's own
``$STATE_DIRECTORY/model.json`` or a path under ``/opt/aqua-bridge`` /
``/etc/aqua-bridge``. The default ``--path`` is already such a scratch file
and is removed again at the end of the run unless ``--keep`` is given.

Usage::

    python tools/bench_model_store.py
    python tools/bench_model_store.py --config config.example-das.yaml \\
        --warm-ticks 1200 --sim-preset rich --repeats 30 --path /tmp/model-bench.json
"""

from __future__ import annotations

import argparse
import dataclasses
import importlib.util
import json
import os
import platform
import statistics
import sys
import tempfile
import time
from pathlib import Path

from aqua_bridge.config import load_config
from aqua_bridge.control.loop import TickResult
from aqua_bridge.control.mpc import step
from aqua_bridge.model import MpcState
from aqua_bridge.modelstore import build_document, write_atomic

REPO_ROOT = Path(__file__).resolve().parent.parent

_SCRATCH_ROOTS = tuple(
    dict.fromkeys(  # de-duplicated, in order: gettempdir() usually already is /tmp
        p.resolve() for p in (Path(tempfile.gettempdir()), Path("/tmp"))
    )
)


def percentile(samples: list[float], q: float) -> float:
    """Nearest-rank percentile (``q`` in [0, 100]); same method as ``tools/bench_step.py``."""
    ordered = sorted(samples)
    k = max(0, min(len(ordered) - 1, int(round(q / 100.0 * (len(ordered) - 1)))))
    return ordered[k]


def _load_bench_step():
    """``tools/bench_step.py`` as a module (``tools`` is not a package): reused for its
    ``das_plant`` builder so the warm-up closed loop here matches item 95's bench exactly,
    rather than a second, silently-drifting copy of the same plant construction."""
    spec = importlib.util.spec_from_file_location(
        "bench_step", REPO_ROOT / "tools" / "bench_step.py"
    )
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def ensure_scratch_path(path: Path) -> Path:
    """Refuse anything but a path under a scratch directory (module docstring, *Safety*)."""
    resolved = path.resolve()
    if not any(resolved.is_relative_to(root) for root in _SCRATCH_ROOTS):
        roots = " or ".join(str(r) for r in _SCRATCH_ROOTS)
        raise SystemExit(
            f"refusing --path {resolved}: not under {roots}. This tool never writes over "
            "the daemon's own model store (PROJECT.md section 8 item 48) -- pass a path "
            "under a scratch directory."
        )
    return resolved


def warm_snapshot(cfg, *, ticks: int, seed: int, preset: str) -> TickResult:
    """A ``TickResult`` after ``ticks`` DAS closed-loop steps (module docstring):
    ``solver_memory`` at roughly the size a running daemon's store reaches, not the
    empty document a cold start would write."""
    bench_step = _load_bench_step()
    plant = bench_step.das_plant(cfg, ticks, seed, preset=preset)
    state = MpcState.cold()
    result: TickResult | None = None
    for i in range(ticks):
        obs = plant.observe()
        smart = plant.observe_smart()
        if smart:
            obs = dataclasses.replace(obs, inputs={"smart": smart})
        cmd, state = step(obs, cfg, state)
        result = TickResult(index=i, obs=obs, mpc_cmd=cmd, cmd=cmd, state=state, applied=True)
        plant.apply(cmd.pwm)
        plant.advance()
    assert result is not None  # ticks > 0, checked by the caller
    return result


def bench_writes(cfg, path: Path, *, warm_ticks: int, seed: int, preset: str, repeats: int):
    result = warm_snapshot(cfg, ticks=warm_ticks, seed=seed, preset=preset)
    memory = result.state.solver_memory
    diagnostics = getattr(result.mpc_cmd, "diagnostics", None)
    bays = diagnostics.get("bays") if isinstance(diagnostics, dict) else None
    bays = bays if isinstance(bays, dict) else None
    ts = memory.get("last_ts")
    ts = float(ts) if isinstance(ts, int | float) else None
    wall = time.time()
    # ident_settle=None: this warm-up loop never runs an identification experiment, so
    # the document's "ident_settle" section is unconditionally empty here too, alongside
    # fan_curves/calibration/bays (module docstring, PROJECT.md item 48's floor-size caveat).
    doc = build_document(cfg, memory, ts=ts, wall=wall, bays=bays, ident_settle=None)
    data = json.dumps(doc, allow_nan=False, separators=(",", ":")).encode()

    times_ms: list[float] = []
    for _ in range(repeats):
        t0 = time.perf_counter()
        write_atomic(path, data)
        times_ms.append((time.perf_counter() - t0) * 1e3)
    on_disk = path.stat().st_size

    p99 = percentile(times_ms, 99)
    return {
        "warm_ticks": warm_ticks,
        "sim_preset": preset,
        "size_bytes": len(data),
        "size_bytes_on_disk": on_disk,
        "sections": sorted(k for k, v in doc.items() if k not in ("schema", "v", "saved_wall")),
        "repeats": repeats,
        "min_ms": min(times_ms),
        "mean_ms": statistics.fmean(times_ms),
        "median_ms": statistics.median(times_ms),
        "p99_ms": p99,
        "max_ms": max(times_ms),
        "spread_ms": max(times_ms) - min(times_ms),
        "dt_s": cfg.dt,
        "model_store_interval_s": cfg.model_store_interval_s,
        "p99_fraction_of_dt": p99 / (cfg.dt * 1e3),
        "p99_fraction_of_model_store_interval": p99 / (cfg.model_store_interval_s * 1e3),
        "times_ms": times_ms,
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument(
        "--config",
        default=None,
        help="zoned YAML with mpc.topology (default: config.example-das.yaml)",
    )
    parser.add_argument(
        "--path",
        default=None,
        help="scratch file to write (default: a fresh file under the system temp directory); "
        "refused unless it resolves under a scratch directory (module docstring, Safety)",
    )
    parser.add_argument(
        "--warm-ticks", type=int, default=600, help="closed-loop ticks before timing (default: 600)"
    )
    parser.add_argument("--sim-preset", default="rich", choices=("basic", "rich"))
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--repeats", type=int, default=20, help="writes timed (default: 20)")
    parser.add_argument(
        "--keep", action="store_true", help="leave the scratch file behind instead of deleting it"
    )
    args = parser.parse_args(argv)

    if args.warm_ticks <= 0:
        parser.error("--warm-ticks must be > 0")
    if args.repeats <= 0:
        parser.error("--repeats must be > 0")

    config_path = args.config or str(REPO_ROOT / "config.example-das.yaml")
    cfg = load_config(config_path).mpc
    if not cfg.regulates_drive_limits:
        parser.error("needs a zoned config without setpoints (mpc.topology)")

    if args.path is not None:
        path = ensure_scratch_path(Path(args.path))
    else:
        fd, name = tempfile.mkstemp(prefix="aqua-bridge-bench-model-store-", suffix=".json")
        os.close(fd)
        path = ensure_scratch_path(Path(name))

    try:
        result = bench_writes(
            cfg,
            path,
            warm_ticks=args.warm_ticks,
            seed=args.seed,
            preset=args.sim_preset,
            repeats=args.repeats,
        )
    finally:
        if not args.keep:
            path.unlink(missing_ok=True)
            tmp_glob = list(path.parent.glob(f".{path.name}.*.tmp"))
            for leftover in tmp_glob:  # write_atomic cleans its own on failure; belt and braces
                leftover.unlink(missing_ok=True)

    report = {
        "python": platform.python_version(),
        "machine": platform.machine(),
        "config_path": str(config_path),
        "path": str(path),
        "kept": args.keep,
        "result": result,
    }
    json.dump(report, sys.stdout, indent=2, sort_keys=True)
    sys.stdout.write("\n")
    return 0


if __name__ == "__main__":
    sys.exit(main())
