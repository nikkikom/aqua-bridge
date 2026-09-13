#!/usr/bin/env python3
"""Replay a recording through the thermal model and report its prediction errors
(plan sections 5, 10, 11 milestone 8).

::

    python tools/replay.py --config config.example-das.yaml rec.jsonl
    python tools/replay.py --config config.example-das.yaml --model model.json --frozen rec.jsonl

Every DAS record is fed to :func:`aqua_bridge.control.thermal.update` in order,
exactly as ``mpc.step`` would live. Two modes:

* default (no ``--model``, or ``--model`` without ``--frozen``): starts from the
  model's own prior (or ``--model``'s memory) and keeps learning (``learn=True``)
  as it goes -- "how well would the model have tracked this recording as it was
  made", the same online behaviour ``model_shadow`` gives live.
* ``--model model.json --frozen``: starts from ``model.json``'s fitted memory and
  never updates it (``learn=False``) -- a genuine out-of-sample prediction check of
  a model ``tools/fit_model.py`` already produced, on a *different* recording (or
  the same one, to sanity-check the fit).

Reports, per zone: status, windows closed, excited windows, and ``pred_err_c`` (the
exponentially-weighted a-priori window residual :func:`aqua_bridge.control.thermal.
summary` already tracks); overall ``pred_err_c`` is the worst zone's, matching
``diagnostics["thermal"]["pred_err_c"]`` on a live daemon.
"""

from __future__ import annotations

import argparse
import json
import logging
import sys
from pathlib import Path
from typing import Any

from aqua_bridge.config import AppConfig, ConfigError, load_config
from aqua_bridge.control import thermal
from aqua_bridge.model import MpcConfig
from aqua_bridge.recorder import iter_records, thermal_inputs

__all__ = ["build_parser", "main", "replay"]

_LOG = logging.getLogger("aqua_bridge.replay")


def replay(
    cfg: MpcConfig,
    records: list[dict[str, Any]],
    *,
    memory: Any = None,
    learn: bool = True,
) -> dict[str, Any]:
    """Feed ``records`` (chronological, DAS records only) through
    :func:`aqua_bridge.control.thermal.update` from ``memory`` (default: the prior);
    returns the final ``summary()`` plus the per-zone ``pred_err_c`` time series."""
    st = thermal.structure(cfg)
    mem = memory if memory is not None else thermal.fresh_memory(cfg)
    series: dict[str, list[float | None]] = {z: [] for z in st.zones}
    result = None
    for rec in records:
        result = thermal.update(mem, cfg, learn=learn, **thermal_inputs(cfg, rec))
        mem = result.memory
        for z, info in result.summary["zones"].items():
            series[z].append(info["pred_err_c"])
    summary = result.summary if result is not None else thermal.summary(mem, cfg, st=st)
    return {
        "n_records": len(records),
        "learn": learn,
        "summary": summary,
        "pred_err_series": series,
    }


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    p.add_argument("recordings", nargs="+", metavar="RECORDING.jsonl", help="oldest first")
    p.add_argument("--config", required=True)
    p.add_argument("--model", default=None, metavar="PATH", help="a model.json from fit_model.py")
    p.add_argument(
        "--frozen",
        action="store_true",
        help="with --model: never learn further, a pure out-of-sample prediction check",
    )
    p.add_argument("--out", default=None, metavar="PATH", help="write the full report as JSON")
    p.add_argument("--log-level", default="INFO")
    return p


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    logging.basicConfig(level=getattr(logging, str(args.log_level).upper(), logging.INFO))
    if args.frozen and args.model is None:
        parser.error("--frozen needs --model")

    try:
        app: AppConfig = load_config(args.config)
    except ConfigError as exc:
        print(f"config: {exc}", file=sys.stderr)
        return 2
    cfg = app.mpc
    if not cfg.is_das:
        print("replay needs a zoned (DAS) config", file=sys.stderr)
        return 2

    memory = None
    if args.model is not None:
        try:
            model_doc = json.loads(Path(args.model).read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError) as exc:
            print(f"--model {args.model}: {exc}", file=sys.stderr)
            return 2
        memory = model_doc.get("memory") if isinstance(model_doc, dict) else None
        if not isinstance(memory, dict):
            print(f"--model {args.model}: no usable 'memory' section", file=sys.stderr)
            return 2

    records: list[dict[str, Any]] = []
    for path in args.recordings:
        records.extend(iter_records(path))
    records = [r for r in records if r.get("das")]
    records.sort(key=lambda r: r.get("ts", 0.0))
    if not records:
        print("no usable DAS records in the given recording(s)", file=sys.stderr)
        return 2

    report = replay(cfg, records, memory=memory, learn=not args.frozen)

    if args.out is not None:
        out_path = Path(args.out)
        if out_path.parent != Path(""):
            out_path.parent.mkdir(parents=True, exist_ok=True)
        out_path.write_text(
            json.dumps(report, indent=2, sort_keys=True, allow_nan=False), encoding="utf-8"
        )

    summary = report["summary"]
    mode = "frozen (out-of-sample)" if args.frozen else "learning"
    print(f"records: {report['n_records']}  mode: {mode}")
    print(f"status: {summary['status']}  pred_err_c: {summary['pred_err_c']}")
    for z, info in summary["zones"].items():
        w, ew, pe = info["windows"], info["excited_windows"], info["pred_err_c"]
        print(f"  zone {z}: {info['status']}  pred_err_c={pe} windows={w} excited={ew}")
    if args.out is not None:
        print(f"wrote {args.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
