#!/usr/bin/env python3
"""Fit each fan model's PWM -> RPM curve from a recording (plan sections 4, 10, 11
milestone 8).

::

    python tools/fit_fans.py --config config.example-das.yaml \\
        --out fan_curves.json rec1.jsonl [rec2.jsonl ...]

Model: ``rpm(u) = rpm_max * phi(u; deadband, exponent)`` with
:func:`aqua_bridge.control.thermal.phi` -- the same curve shape
``thermal.PARAMETERS`` already documents as "offline grid" for ``u0``/``n``
(``fan_models.<m>.deadband``/``.exponent``). Fitted per **fan model**, not per
channel, from every ``(pwm, rpm)`` pair the recording carries for a channel that
model is on *and* has a tachometer; a model with no tach anywhere in the recording
keeps its configured curve (or the config's own default when unconfigured), so a
tach-less channel of that model is automatically covered by the same curve --
"copied to tach-less outputs of the same model" (plan section 4) falls out of
fitting one curve per model rather than per channel.

Fit: a small grid over ``deadband`` x ``exponent`` (the table's own bounds); at each
grid point ``rpm_max`` has a closed form (``phi`` fixed, plain weighted least
squares in the one remaining linear parameter), so the whole search is a few
hundred closed-form evaluations, no iterative solver, no scipy. The recorded
``pwm``/``rpm`` fields are the source's readback pair (the output duty and speed the
aquaero or Quadro reports), not the controller's command -- both read by the same
source at the same tick, unlike ``prev``/``cmd``, which are controller-side.

The daemon can fit the same curve **online** (``mpc.fan_curve_online``,
:mod:`aqua_bridge.control.fancurve`), over the same grid, and then reads it from the
model store instead of ``fan_models``. This tool stays the way to fit from a recording
and to see the residual before trusting a curve.

Each model's output also carries ``channels_with_rail_reported`` (PROJECT.md section 8
item 127): the channels of that model for which some record's ``fans.<ch>.rail_reported``
was true, i.e. the rail was this output's own measurement at least once, read with
:func:`aqua_bridge.recorder.rail_known` so a recording made before that flag existed (no
key at all) counts as not measured rather than as measured. It plays no part in the curve
fit itself (only ``pwm``/``rpm`` do); it is here so a reader of the written-out JSON can
tell which channels' ``voltage_v`` in the source recordings meant something.
"""

from __future__ import annotations

import argparse
import json
import logging
import math
import sys
from pathlib import Path
from typing import Any

from aqua_bridge.config import AppConfig, ConfigError, load_config
from aqua_bridge.control import fancurve
from aqua_bridge.control.thermal import phi
from aqua_bridge.model import MpcConfig
from aqua_bridge.recorder import iter_records, rail_known

__all__ = ["build_parser", "fit_fan_curves", "fit_one_model", "main"]

_LOG = logging.getLogger("aqua_bridge.fit_fans")

#: Grid resolution over the table's bounds (thermal.PARAMETERS: deadband [0, 0.5),
#: exponent [0.5, 1.5]). One definition, shared with the online fit
#: (:mod:`aqua_bridge.control.fancurve`), so both searches have the same shape.
DEADBAND_GRID = fancurve.DEADBAND_GRID  # 0.000 .. 0.450
EXPONENT_GRID = fancurve.EXPONENT_GRID  # 0.50 .. 1.50
MIN_SAMPLES = 10


def _finite(value: object) -> bool:
    return isinstance(value, int | float) and not isinstance(value, bool) and math.isfinite(value)


def fit_one_model(pairs: list[tuple[float, float]]) -> dict[str, Any] | None:
    """Grid search (module docstring) over ``pairs`` of ``(pwm, rpm)``; ``None`` when
    there are too few points to fit anything meaningful."""
    if len(pairs) < MIN_SAMPLES:
        return None
    us = [p[0] for p in pairs]
    rpms = [p[1] for p in pairs]
    best: dict[str, Any] | None = None
    for deadband in DEADBAND_GRID:
        for exponent in EXPONENT_GRID:
            phis = [phi(u, deadband, exponent) for u in us]
            denom = sum(p * p for p in phis)
            if denom <= 1e-9:
                continue
            rpm_max = sum(p * r for p, r in zip(phis, rpms, strict=True)) / denom
            if not (rpm_max > 0.0):
                continue
            sse = sum((rpm_max * p - r) ** 2 for p, r in zip(phis, rpms, strict=True))
            if best is None or sse < best["_sse"]:
                best = {"deadband": deadband, "exponent": exponent, "rpm_max": rpm_max, "_sse": sse}
    if best is None:
        return None
    n = len(pairs)
    rmse = (best.pop("_sse") / n) ** 0.5
    best["rmse_rpm"] = rmse
    best["rmse_frac"] = rmse / best["rpm_max"] if best["rpm_max"] > 0 else None
    best["n_samples"] = n
    best["source"] = "fitted"
    return best


def fit_fan_curves(cfg: MpcConfig, records: list[dict[str, Any]]) -> dict[str, Any]:
    """Every ``fan_models`` entry: the fitted curve, or the configured/default one
    with ``source: "prior"`` when the recording has no tach data for that model."""
    channels_by_model: dict[str, list[str]] = {}
    for ch, spec in cfg.fans.items():
        channels_by_model.setdefault(spec.model, []).append(ch)

    pairs_by_model: dict[str, list[tuple[float, float]]] = {m: [] for m in channels_by_model}
    for rec in records:
        pwm = rec.get("pwm") or {}
        rpm = rec.get("rpm") or {}
        for model, channels in channels_by_model.items():
            for ch in channels:
                u, r = pwm.get(ch), rpm.get(ch)
                if _finite(u) and _finite(r):
                    pairs_by_model[model].append((float(u), float(r)))

    out: dict[str, Any] = {}
    for model, channels in channels_by_model.items():
        fitted = fit_one_model(pairs_by_model[model])
        if fitted is None:
            prior = cfg.fan_models[model]
            fitted = {
                "deadband": prior.deadband,
                "exponent": prior.exponent,
                "rpm_max": prior.rpm_max,
                "rmse_rpm": None,
                "rmse_frac": None,
                "n_samples": len(pairs_by_model[model]),
                "source": "prior",
            }
        fitted["channels"] = sorted(channels)
        fitted["channels_with_tach"] = sorted(
            {ch for ch in channels for rec in records if _finite((rec.get("rpm") or {}).get(ch))}
        )
        # A channel whose rail this recording ever actually measured (item 127): an
        # aquabus output's 0.00 V is the aquaero's own rail, not a reading, so a
        # channel with no entry here is not "the rail was always 0 V" -- it is "no
        # record here ever measured it", exactly like a channel absent from
        # channels_with_tach. rail_known reads a fan entry with no rail_reported key
        # at all (a recording made before item 127) as not measured, never as measured.
        fitted["channels_with_rail_reported"] = sorted(
            {
                ch
                for ch in channels
                for rec in records
                if rail_known((rec.get("fans") or {}).get(ch) or {})
            }
        )
        out[model] = fitted
    return out


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    p.add_argument("recordings", nargs="+", metavar="RECORDING.jsonl")
    p.add_argument("--config", required=True)
    p.add_argument("--out", default="fan_curves.json", metavar="PATH")
    p.add_argument("--log-level", default="INFO")
    return p


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    logging.basicConfig(level=getattr(logging, str(args.log_level).upper(), logging.INFO))

    try:
        app: AppConfig = load_config(args.config)
    except ConfigError as exc:
        print(f"config: {exc}", file=sys.stderr)
        return 2
    cfg = app.mpc
    if not cfg.fan_models:
        print(
            "config has no fan_models (legacy config, or DAS config without fans)", file=sys.stderr
        )
        return 2

    records: list[dict[str, Any]] = []
    for path in args.recordings:
        records.extend(iter_records(path))
    if not records:
        print("no usable records in the given recording(s)", file=sys.stderr)
        return 2

    curves = fit_fan_curves(cfg, records)
    out_path = Path(args.out)
    if out_path.parent != Path(""):
        out_path.parent.mkdir(parents=True, exist_ok=True)
    payload = {
        "v": 1,
        "kind": "aqua_bridge.fan_curves",
        "n_records": len(records),
        "models": curves,
    }
    out_path.write_text(
        json.dumps(payload, indent=2, sort_keys=True, allow_nan=False), encoding="utf-8"
    )

    for model, info in sorted(curves.items()):
        rmse = "n/a" if info["rmse_frac"] is None else f"{info['rmse_frac'] * 100:.1f}%"
        print(
            f"{model}: {info['source']:>5}  rpm_max={info['rpm_max']:.0f} "
            f"deadband={info['deadband']:.3f} exponent={info['exponent']:.2f} "
            f"rmse={rmse} n={info['n_samples']} channels={info['channels']}"
        )
    print(f"wrote {out_path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
