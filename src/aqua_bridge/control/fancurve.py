"""Online PWM -> RPM curve fit per fan model (plan section 4, section 8 item 14).

Pure: :func:`update` is a function of its arguments, reads no clock, does no I/O and
draws no random numbers; its state is plain JSON in ``solver_memory["fan_fit"]``.
``mpc.step`` calls it with ``mpc.fan_curve_online: true``, after the tick's command is
final, and puts what it accepts into ``solver_memory["fan_curves"]`` -- the section the
model store already saves and loads (:mod:`aqua_bridge.modelstore`,
:func:`aqua_bridge.control.persist.apply_seed`), in exactly the shape it already
validates: ``{fan model: {"rpm_max", "deadband", "exponent"}}``.

The curve is the one the rest of the controller already uses,
``rpm(u) = rpm_max * phi(u; deadband, exponent)`` with
:func:`aqua_bridge.control.thermal.phi` -- the ``u0.<m>`` / ``n.<m>`` rows of
``thermal.PARAMETERS``, which until now came from the config alone (``fan_models``,
fitted offline by ``tools/fit_fans.py`` and copied in by hand). With the switch on, the
thermal model's identification and the DAS MPC's prediction read the fitted curve
instead (:func:`aqua_bridge.control.thermal.model_params`), per **fan model**, so a
tach-less output is covered by the curve fitted from the outputs of the same model that
do report a speed. Only ``deadband`` and ``exponent`` reach the model; the fitted
``rpm_max`` is reported (``GET /api/state``) but never normalises a tachometer reading
(``model_use_rpm`` keeps the commissioned ``fan_models.<m>.rpm_max``, the only fixed
reference against which an absolute loss of speed shows up --
:func:`aqua_bridge.control.thermal._channel_phi`).

What is sampled
---------------
Only settled points: a channel contributes ``(u, rpm)`` once its commanded duty has
stayed within :data:`SETTLE_TOL` of the same value for ``fan_curve_settle_s`` (a fan
takes seconds to reach its new speed, and a ramping sample would read as a point off the
curve). Samples go into :data:`BINS` PWM bins per fan model, each holding
``(n, sum u, sum rpm, sum rpm^2)``; a bin at :data:`BIN_CAPACITY` becomes an exponential
mean of that length, so the accumulator is bounded and follows a fan that ages or is
replaced instead of being pinned by a year of old samples.

The fit
-------
Every ``fan_curve_refit_s`` per model: the same small grid over ``deadband`` x
``exponent`` ``tools/fit_fans.py`` uses (:data:`DEADBAND_GRID`, :data:`EXPONENT_GRID`,
the bounds of ``thermal.PARAMETERS``), with ``rpm_max`` in closed form at each grid point
(weighted least squares in the one remaining linear parameter). No iterative solver, no
scipy; a few hundred vector operations over at most :data:`BINS` bins, once per interval.
The residual counts the scatter inside each bin, not only the bin means, so a noisy
tachometer cannot look like a perfect fit.

A fit is **accepted** only when the data can carry it: at least :data:`MIN_BINS` bins
with :data:`MIN_BIN_SAMPLES` samples each, spanning at least :data:`MIN_SPAN` of PWM, and
a relative RMSE at most ``fan_curve_max_rmse_frac``. Otherwise the previous accepted
curve (or, with none, the configured one) stays: this never leaves the controller without
a curve, and a stalled or lying tachometer widens the residual rather than moving the
model. The accepted curve is JSON in ``solver_memory["fan_curves"]``, so it survives a
restart through the model store; the accumulator does not (it refills in an hour of
ordinary regulation, and a store file older than ``model_store_max_age_days`` should not
seed a fan's speed anyway).

Memory (plain JSON)::

    {"v": 1, "fp": <channel -> fan model fingerprint>,
     "hold": {channel: [u, since_ts]},
     "models": {model: {"bins": [[n, su, sr, sr2], ... BINS],
                        "fit": {"rpm_max", "deadband", "exponent", "rmse_frac", "n",
                                "bins", "span"} | null,
                        "rejected": str | null, "last_ts": ts | null}}}

A memory that does not match the config's fans, or is malformed in any way, starts over
(never an exception).
"""

from __future__ import annotations

import json
import math
from collections.abc import Mapping
from dataclasses import dataclass
from typing import Any

import numpy as np

from aqua_bridge.model import MpcConfig

__all__ = [
    "BINS",
    "BIN_CAPACITY",
    "DEADBAND_GRID",
    "EXPONENT_GRID",
    "MIN_BINS",
    "MIN_BIN_SAMPLES",
    "MIN_SPAN",
    "SETTLE_TOL",
    "VERSION",
    "FanCurveUpdate",
    "curve_pair",
    "fresh_memory",
    "summary",
    "update",
]

VERSION = 1

#: Grid over the bounds of ``thermal.PARAMETERS`` (``u0`` in [0, 0.5), ``n`` in
#: [0.5, 1.5]); ``tools/fit_fans.py`` searches the same one, so the online and the
#: offline fit have one definition between them.
DEADBAND_GRID: tuple[float, ...] = tuple(round(0.025 * i, 4) for i in range(19))
EXPONENT_GRID: tuple[float, ...] = tuple(round(0.5 + 0.05 * i, 4) for i in range(21))

#: PWM bins of the accumulator over [0, 1] (internal resolution, not an operator setting).
BINS = 32
#: Samples a bin holds before it becomes an exponential mean of that length.
BIN_CAPACITY = 240.0
#: A bin counts toward a fit from this many samples.
MIN_BIN_SAMPLES = 8.0
#: Bins, and the PWM span over them, a fit needs.
MIN_BINS = 4
MIN_SPAN = 0.25
#: A commanded duty within this of the held one counts as unchanged (settling).
SETTLE_TOL = 1e-6

_EPS = 1e-12


def _finite(value: object) -> bool:
    if type(value) is float:
        return math.isfinite(value)
    return isinstance(value, int | float) and not isinstance(value, bool) and math.isfinite(value)


@dataclass(frozen=True)
class FanCurveUpdate:
    """What :func:`update` returns: the next memory and the curves it accepts."""

    memory: dict[str, Any]
    curves: dict[str, dict[str, float]]


def curve_pair(
    curve: object, deadband: float, exponent: float, rpm_max: float
) -> tuple[float, float, float]:
    """``(deadband, exponent, rpm_max)`` of a stored curve, or the configured triple.

    The single place that decides whether a ``fan_curves`` entry is usable: a mapping of
    three finite numbers inside the bounds ``fan_models`` validation and
    ``thermal.PARAMETERS`` give them. Anything else falls back to the config, so a
    malformed entry can never put a NaN or a negative dead band into the model.
    """
    if not isinstance(curve, Mapping):
        return deadband, exponent, rpm_max
    d, n, r = (curve.get(k) for k in ("deadband", "exponent", "rpm_max"))
    if not (_finite(d) and _finite(n) and _finite(r)):
        return deadband, exponent, rpm_max
    d, n, r = float(d), float(n), float(r)  # type: ignore[arg-type]
    if not (0.0 <= d < 0.5 and 0.5 <= n <= 1.5 and r > 0.0):
        return deadband, exponent, rpm_max
    return d, n, r


# ---------------------------------------------------------------------------
# memory
# ---------------------------------------------------------------------------


def _fingerprint(cfg: MpcConfig) -> str:
    return json.dumps(
        [BINS, {ch: cfg.fans[ch].model for ch in cfg.channels}],
        sort_keys=True,
        separators=(",", ":"),
    )


def _models(cfg: MpcConfig) -> list[str]:
    return list(dict.fromkeys(cfg.fans[ch].model for ch in cfg.channels))


def fresh_memory(cfg: MpcConfig) -> dict[str, Any]:
    """The empty accumulator of a config's fan models."""
    return {
        "v": VERSION,
        "fp": _fingerprint(cfg),
        "hold": {},
        "models": {
            m: {
                "bins": [[0.0, 0.0, 0.0, 0.0] for _ in range(BINS)],
                "fit": None,
                "rejected": None,
                "last_ts": None,
            }
            for m in _models(cfg)
        },
    }


def _load(memory: object, cfg: MpcConfig) -> dict[str, Any]:
    fresh = fresh_memory(cfg)
    if not isinstance(memory, Mapping) or memory.get("v") != VERSION:
        return fresh
    if memory.get("fp") != fresh["fp"]:
        return fresh
    try:
        models_in = memory["models"]
        if not isinstance(models_in, Mapping):
            raise TypeError("models")
        out = dict(fresh)
        out["hold"] = {
            str(ch): [float(v[0]), float(v[1])]
            for ch, v in dict(memory.get("hold") or {}).items()
            if ch in cfg.channels
            and isinstance(v, list | tuple)
            and len(v) == 2
            and _finite(v[0])
            and _finite(v[1])
        }
        models: dict[str, Any] = {}
        for m in fresh["models"]:
            raw = models_in[m]
            bins = np.asarray(raw["bins"], dtype=float)
            if bins.shape != (BINS, 4) or not np.isfinite(bins).all() or (bins < 0).any():
                raise ValueError("bins")
            fit = raw.get("fit")
            if fit is not None and not isinstance(fit, Mapping):
                raise TypeError("fit")
            last = raw.get("last_ts")
            if last is not None and not _finite(last):
                raise ValueError("last_ts")
            rejected = raw.get("rejected")
            models[m] = {
                "bins": bins.tolist(),
                "fit": None if fit is None else dict(fit),
                "rejected": rejected if isinstance(rejected, str) else None,
                "last_ts": None if last is None else float(last),
            }
        out["models"] = models
        return out
    except (TypeError, ValueError, KeyError, AttributeError, IndexError):
        return fresh


# ---------------------------------------------------------------------------
# the fit
# ---------------------------------------------------------------------------


def _fit_bins(bins: np.ndarray, max_rmse_frac: float) -> tuple[dict[str, Any] | None, str | None]:
    """Grid fit over the usable bins (module docstring). ``(fit, reason it was rejected)``."""
    usable = bins[bins[:, 0] >= MIN_BIN_SAMPLES]
    if len(usable) < MIN_BINS:
        return None, f"{len(usable)} usable bins, need {MIN_BINS}"
    n = usable[:, 0]
    u = usable[:, 1] / n
    sr = usable[:, 2]
    sr2 = float(usable[:, 3].sum())
    span = float(u.max() - u.min())
    if span < MIN_SPAN:
        return None, f"PWM span {span:.2f}, need {MIN_SPAN}"
    total = float(n.sum())
    best: tuple[float, float, float, float] | None = None  # (sse, rpm_max, deadband, exponent)
    exps = np.array(EXPONENT_GRID)
    for deadband in DEADBAND_GRID:
        frac = np.clip((u - deadband) / (1.0 - deadband), 0.0, 1.0)
        phis = np.where(frac > 0.0, frac[None, :] ** exps[:, None], 0.0)
        denom = (phis * phis * n).sum(axis=1)
        cross = (phis * sr).sum(axis=1)
        rpm_max = np.divide(cross, denom, out=np.zeros_like(denom), where=denom > _EPS)
        sse = sr2 - 2.0 * rpm_max * cross + rpm_max * rpm_max * denom
        sse = np.where((denom > _EPS) & (rpm_max > 0.0), sse, np.inf)
        i = int(np.argmin(sse))
        if math.isfinite(float(sse[i])) and (best is None or float(sse[i]) < best[0]):
            best = (float(sse[i]), float(rpm_max[i]), float(deadband), float(exps[i]))
    if best is None:
        return None, "no grid point fits"
    sse, rpm_max, deadband, exponent = best
    rmse_frac = math.sqrt(max(sse, 0.0) / total) / rpm_max
    fit = {
        "rpm_max": rpm_max,
        "deadband": deadband,
        "exponent": exponent,
        "rmse_frac": rmse_frac,
        "n": total,
        "bins": len(usable),
        "span": span,
    }
    if rmse_frac > max_rmse_frac:
        return None, f"relative RMSE {rmse_frac:.3f} above {max_rmse_frac:.3f}"
    return fit, None


# ---------------------------------------------------------------------------
# the per-tick update
# ---------------------------------------------------------------------------


def update(
    memory: object,
    cfg: MpcConfig,
    *,
    u: Mapping[str, float],
    rpm: Mapping[str, Any] | None,
    ts: float,
) -> FanCurveUpdate:
    """One tick of the online fit (module docstring).

    ``u`` is the command the fans have been on since the previous tick (``prev``) and
    ``rpm`` is ``obs.rpm``. Returns the next memory and the curves accepted so far
    (every model whose last fit passed; a model without one is absent, and its
    configured curve stays in force). Never raises on data: a malformed memory starts
    over and a non-finite reading is skipped.
    """
    mem = _load(memory, cfg)
    ts = float(ts)
    hold: dict[str, list[float]] = mem["hold"]
    for ch in cfg.channels:
        value = u.get(ch)
        if not _finite(value):
            hold.pop(ch, None)
            continue
        duty = min(1.0, max(0.0, float(value)))  # type: ignore[arg-type]
        held = hold.get(ch)
        if held is None or abs(held[0] - duty) > SETTLE_TOL or ts < held[1]:
            hold[ch] = [duty, ts]
            continue
        if ts - held[1] < cfg.fan_curve_settle_s:
            continue
        reading = None if rpm is None else rpm.get(ch)
        if not _finite(reading) or float(reading) < 0.0:  # type: ignore[arg-type]
            continue
        speed = float(reading)  # type: ignore[arg-type]
        bins = mem["models"][cfg.fans[ch].model]["bins"]
        row = bins[min(BINS - 1, int(duty * BINS))]
        if row[0] >= BIN_CAPACITY:  # an exponential mean of BIN_CAPACITY samples
            decay = (BIN_CAPACITY - 1.0) / BIN_CAPACITY
            row[0], row[1], row[2], row[3] = (
                BIN_CAPACITY - 1.0,
                row[1] * decay,
                row[2] * decay,
                row[3] * decay,
            )
        row[0] += 1.0
        row[1] += duty
        row[2] += speed
        row[3] += speed * speed

    curves: dict[str, dict[str, float]] = {}
    for model, block in mem["models"].items():
        last = block["last_ts"]
        if last is None or ts < last:
            block["last_ts"] = ts
        elif ts - last >= cfg.fan_curve_refit_s:
            block["last_ts"] = ts
            fit, reason = _fit_bins(
                np.asarray(block["bins"], dtype=float), cfg.fan_curve_max_rmse_frac
            )
            if fit is not None:
                block["fit"] = fit
            block["rejected"] = reason
        fit = block["fit"]
        if isinstance(fit, Mapping):
            curves[model] = {
                "rpm_max": float(fit["rpm_max"]),
                "deadband": float(fit["deadband"]),
                "exponent": float(fit["exponent"]),
            }
    return FanCurveUpdate(memory=mem, curves=curves)


def summary(
    memory: Mapping[str, Any] | None, cfg: MpcConfig, curves: Mapping[str, Any] | None
) -> dict[str, Any]:
    """``diagnostics["fan_curves"]``: the curve in force per fan model and where it came
    from (``fit`` from this run, ``store`` from ``model.json``, else ``config``), with the
    state of the online fit."""
    out: dict[str, Any] = {"online": cfg.fan_curve_online, "models": {}}
    models = (memory or {}).get("models") if isinstance(memory, Mapping) else None
    for model in _models(cfg):
        spec = cfg.fan_models[model]
        entry = models.get(model) if isinstance(models, Mapping) else None
        fit = entry.get("fit") if isinstance(entry, Mapping) else None
        stored = (curves or {}).get(model)
        deadband, exponent, rpm_max = curve_pair(stored, spec.deadband, spec.exponent, spec.rpm_max)
        source = "config"
        if stored is not None:
            source = "fit" if isinstance(fit, Mapping) else "store"
        row: dict[str, Any] = {
            "source": source,
            "rpm_max": rpm_max,
            "deadband": deadband,
            "exponent": exponent,
        }
        if isinstance(entry, Mapping):
            bins = np.asarray(entry["bins"], dtype=float)
            row["samples"] = float(bins[:, 0].sum())
            row["usable_bins"] = int((bins[:, 0] >= MIN_BIN_SAMPLES).sum())
            row["rejected"] = entry.get("rejected")
            if isinstance(fit, Mapping):
                row["rmse_frac"] = fit.get("rmse_frac")
        out["models"][model] = row
    return out
