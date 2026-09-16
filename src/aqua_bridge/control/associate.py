"""Serial -> bay association by correlation (plan section 1, "SMART path").

The DAS has no SES backplane, so the PC-side SMART agent reports drive
temperatures by serial and nobody can say which bay a serial sits in. This
module finds out from the data. Pure functions, no clocks, no I/O; the
estimator (:mod:`aqua_bridge.control.estimator`) keeps every series in its
JSON memory and calls these functions.

Rules
-----
1. A declared serial (``topology.bays.<bay>.serial``, or the same through
   ``POST /api/bay {bay, serial}``) wins: that serial belongs to that bay and
   neither takes part in the correlation.
2. Otherwise, over a sliding window of ``estimator.associate_window_s``
   seconds, every unassigned fresh serial's SMART series is correlated with
   the ``T_s - T_a`` series of every candidate bay (occupied or unknown, no
   association yet). SMART samples are sparse (30-60 s), so each sample is
   paired with the bay series at the newest recorded time not after it; the
   SMART value is taken relative to the zone air estimate of the bay's zone at
   that time (the residual after the zone-air term: fan changes move every
   drive of a zone together and would otherwise correlate everything with
   everything). Both series are detrended (least-squares line over time)
   before the Pearson correlation. A pair needs :data:`MIN_SAMPLES` paired
   samples and a SMART history that spans at least :data:`MIN_SPAN_FRACTION`
   of the window; fewer, a shorter history, or a series without variance, has
   no score. (On the rich truth preset a 20-sample, quarter-hour window let a
   quantised, lagging SMART series of an idle drive correlate at 0.9 with a
   bay in another zone; the full window removes that.)
3. Greedy assignment by descending score: a pair is accepted only when its
   score is at least ``associate_min_corr`` **and** beats the runner-up of the
   serial (its score with any other bay) and the runner-up of the bay (any
   other serial's score with it) by ``associate_margin``. Runner-ups are taken
   over every scored pair, assigned or not, and never below 0, so two bays
   with the same heat pattern are refused rather than guessed. An accepted pair
   becomes an association only after it is accepted on
   :data:`CONFIRM_EVALUATIONS` consecutive evaluations (:func:`confirm`).
4. Dropping (the estimator applies it): an association ends when the bay's
   occupancy changes into or out of ``empty`` (a hot swap), when the bay's
   proximal sensor shows a jump (the conservative fast-swap rule of the
   estimator), when the serial stays silent for ``smart_max_age_s``, or when it
   stops correlating (rule 5).
5. Re-checking: an association is a claim about correlation, so it has to keep
   holding. The bay series and the serial's SMART history go on being recorded
   after the pair is accepted, and every evaluation re-scores the pair against
   its own bay. ``associate_drop_checks`` consecutive scores below
   ``associate_drop_corr`` end it; a pair too short of history to score is left
   alone. Keeping a pair asks less than choosing one, because a correct pair
   dips through a quiet window while a wrong one sits near zero. A dropped
   pair's history starts over, so it must win rule 3 again over a full window
   before it may calibrate that bay; so does an accepted one, so that a pair's
   first re-check scores a fresh window instead of the one that accepted it.
   A wrong pair from a coincidence over one window does not survive the next
   ones; the absolute SMART band (``smart_reject_c``) cannot tell the two apart,
   because an uncalibrated bay's estimate carries the prior map's own offset.

Series layout (plain JSON, kept by the estimator)::

    {"t": [ts, ...],                   # recording times, one every SAMPLE_S
     "y": {bay: [T_s - T_a | None]},   # per bay
     "a": {zone: [T_a | None]}}        # per zone

SMART history per serial: ``[[sample_ts, temp_c], ...]`` in the same window.
"""

from __future__ import annotations

import math
from collections.abc import Iterable, Mapping, Sequence
from typing import Any

import numpy as np

__all__ = [
    "CANDIDATES_SHOWN",
    "CONFIRM_EVALUATIONS",
    "EVERY_S",
    "MIN_SAMPLES",
    "MIN_SPAN_FRACTION",
    "SAMPLE_S",
    "assign",
    "confirm",
    "correlation",
    "record_series",
    "record_smart",
    "score_matrix",
    "top_candidates",
]

#: Recording period of the bay and zone series, seconds (SMART cadence is 30-60 s).
SAMPLE_S = 30.0
#: How often the estimator re-scores the unassigned serials, seconds.
EVERY_S = 60.0
#: Paired samples a (serial, bay) score needs.
MIN_SAMPLES = 20
#: A serial's SMART history must span this fraction of ``associate_window_s``.
MIN_SPAN_FRACTION = 0.9
#: Consecutive evaluations that must accept the same pair before it is associated.
CONFIRM_EVALUATIONS = 3
#: Candidates per bay shown by ``GET /api/bays``.
CANDIDATES_SHOWN = 3


def _finite(value: object) -> bool:
    if type(value) is float:  # the common case, checked first (hot path)
        return math.isfinite(value)
    return (
        isinstance(value, int | float)
        and not isinstance(value, bool)
        and math.isfinite(float(value))
    )


def record_series(
    series: Mapping[str, Any] | None,
    ts: float,
    bay_y: Mapping[str, float | None],
    zone_air: Mapping[str, float | None],
    window_s: float,
) -> dict[str, Any]:
    """Append one recording when :data:`SAMPLE_S` has passed; trim to the window.

    The input is never mutated: between recordings the same (consistent)
    ``series`` is handed back, a recording builds new lists. A malformed or
    inconsistent ``series`` (lengths differ, other names, time not increasing)
    starts over.
    """
    ok = isinstance(series, dict)
    t_old = series.get("t") if ok else None  # type: ignore[union-attr]
    y_old = series.get("y") if ok else None  # type: ignore[union-attr]
    a_old = series.get("a") if ok else None  # type: ignore[union-attr]
    consistent = (
        isinstance(t_old, list)
        and isinstance(y_old, dict)
        and isinstance(a_old, dict)
        and set(y_old) == set(bay_y)
        and set(a_old) == set(zone_air)
        and all(
            isinstance(v, list) and len(v) == len(t_old) for v in (*y_old.values(), *a_old.values())
        )
        and (not t_old or (_finite(t_old[-1]) and ts >= float(t_old[-1])))
    )
    if not consistent:
        t_old, y_old, a_old = [], {b: [] for b in bay_y}, {z: [] for z in zone_air}
    assert isinstance(t_old, list) and isinstance(y_old, dict) and isinstance(a_old, dict)
    if t_old and ts - float(t_old[-1]) < SAMPLE_S - 1e-9:
        return series  # type: ignore[return-value]
    first = next(
        (i for i, t in enumerate(t_old) if _finite(t) and ts - float(t) <= window_s), len(t_old)
    )
    return {
        "t": [*t_old[first:], float(ts)],
        "y": {b: [*y_old[b][first:], float(v) if _finite(v) else None] for b, v in bay_y.items()},
        "a": {
            z: [*a_old[z][first:], float(v) if _finite(v) else None] for z, v in zone_air.items()
        },
    }


def record_smart(
    hist: Sequence[Sequence[float]] | None,
    sample_ts: float,
    temp_c: float,
    now: float,
    window_s: float,
) -> list[list[float]]:
    """``hist`` plus one SMART sample, trimmed to the window (a new list)."""
    out = [
        [float(p[0]), float(p[1])]
        for p in (hist or ())
        if isinstance(p, Sequence)
        and len(p) == 2
        and _finite(p[0])
        and _finite(p[1])
        and now - float(p[0]) <= window_s
        and float(p[0]) < sample_ts
    ]
    out.append([float(sample_ts), float(temp_c)])
    return out


def _float_array(values: Iterable[Any]) -> np.ndarray:
    """1-D float array with NaN for anything that is not a finite number."""
    return np.array([float(v) if _finite(v) else np.nan for v in values], dtype=float)


def _detrend_rows(values: np.ndarray, t: np.ndarray) -> np.ndarray:
    """Remove the least-squares line over ``t`` from every row of ``values``."""
    tc = t - t.mean()
    denom = float(tc @ tc)
    centred = values - values.mean(axis=1, keepdims=True)
    if denom <= 0:
        return centred
    slope = (centred @ tc) / denom
    return centred - slope[:, None] * tc[None, :]


def correlation(x: Sequence[float], y: Sequence[float], t: Sequence[float]) -> float | None:
    """Pearson correlation of detrended ``x`` and ``y`` (both over ``t``).

    ``None`` when there are fewer than :data:`MIN_SAMPLES` finite triples or a
    detrended series has no variance.
    """
    arr = np.array([x, y, t], dtype=float)
    mask = np.all(np.isfinite(arr), axis=0)
    arr = arr[:, mask]
    if arr.shape[1] < MIN_SAMPLES:
        return None
    res = _detrend_rows(arr[:2], arr[2])
    nx, ny = float(np.linalg.norm(res[0])), float(np.linalg.norm(res[1]))
    if nx <= 1e-9 or ny <= 1e-9:
        return None
    return float(np.clip(res[0] @ res[1] / (nx * ny), -1.0, 1.0))


def score_matrix(
    series: Mapping[str, Any],
    smart_hist: Mapping[str, Sequence[Sequence[float]]],
    serials: Iterable[str],
    bays: Iterable[str],
    bay_zone: Mapping[str, str],
    min_span_s: float = 0.0,
) -> dict[str, dict[str, float]]:
    """``{serial: {bay: score}}`` for every scorable pair (rule 2 of the module docstring).

    A serial whose SMART history spans less than ``min_span_s`` seconds is not scored.
    """
    out: dict[str, dict[str, float]] = {}
    try:
        t_rec = _float_array(series.get("t", []))
        y_all, a_all = series.get("y", {}), series.get("a", {})
        bays = [b for b in bays if b in y_all and bay_zone.get(b) in a_all]
        if t_rec.size == 0 or not bays or np.any(np.diff(t_rec) <= 0):
            return out
        y_rows = np.array([_float_array(y_all[b]) for b in bays])
        a_rows = np.array([_float_array(a_all[bay_zone[b]]) for b in bays])
    except (TypeError, ValueError, AttributeError):  # malformed series: no scores
        return out
    if y_rows.shape != (len(bays), t_rec.size) or a_rows.shape != y_rows.shape:
        return out
    for serial in serials:
        hist = smart_hist.get(serial) or ()
        if len(hist) < MIN_SAMPLES:
            continue
        try:
            pts = np.array(hist, dtype=float)
        except (TypeError, ValueError):
            continue
        if pts.ndim != 2 or pts.shape[1] != 2 or not np.all(np.isfinite(pts)):
            continue
        if float(pts[-1, 0] - pts[0, 0]) < min_span_s:
            continue
        idx = np.searchsorted(t_rec, pts[:, 0], side="right") - 1
        ok = (idx >= 0) & (pts[:, 0] - t_rec[np.clip(idx, 0, None)] <= 2.0 * SAMPLE_S)
        if int(ok.sum()) < MIN_SAMPLES:
            continue
        idx, pts = idx[ok], pts[ok]
        ys = y_rows[:, idx]
        xs = pts[None, :, 1] - a_rows[:, idx]
        t = pts[:, 0]
        scores: dict[str, float] = {}
        full = np.all(np.isfinite(ys), axis=1) & np.all(np.isfinite(xs), axis=1)
        if full.any():
            rx = _detrend_rows(xs[full], t)
            ry = _detrend_rows(ys[full], t)
            nx = np.linalg.norm(rx, axis=1)
            ny = np.linalg.norm(ry, axis=1)
            dot = np.sum(rx * ry, axis=1)
            for k, bay_i in enumerate(np.flatnonzero(full)):
                if nx[k] > 1e-9 and ny[k] > 1e-9:
                    scores[bays[bay_i]] = float(np.clip(dot[k] / (nx[k] * ny[k]), -1.0, 1.0))
        for bay_i in np.flatnonzero(~full):
            value = correlation(xs[bay_i], ys[bay_i], t)
            if value is not None:
                scores[bays[bay_i]] = value
        if scores:
            out[serial] = scores
    return out


def assign(
    scores: Mapping[str, Mapping[str, float]], min_corr: float, margin: float
) -> dict[str, str]:
    """Greedy ``{bay: serial}`` from a score matrix (rule 3 of the module docstring).

    Deterministic: ties are broken by serial, then bay name.
    """
    pairs = sorted(
        (
            (float(score), serial, bay)
            for serial, row in scores.items()
            for bay, score in row.items()
        ),
        key=lambda p: (-p[0], p[1], p[2]),
    )
    taken_serials: set[str] = set()
    taken_bays: set[str] = set()
    out: dict[str, str] = {}
    for score, serial, bay in pairs:
        if score < min_corr:
            break
        if serial in taken_serials or bay in taken_bays:
            continue
        runner_serial = max(
            [0.0, *(v for b, v in scores[serial].items() if b != bay)],
        )
        runner_bay = max(
            [0.0, *(row[bay] for s, row in scores.items() if s != serial and bay in row)],
        )
        if score - max(runner_serial, runner_bay) >= margin:
            out[bay] = serial
            taken_serials.add(serial)
            taken_bays.add(bay)
    return dict(sorted(out.items()))


def confirm(
    pending: Mapping[str, Sequence[Any]] | None, accepted: Mapping[str, str]
) -> tuple[dict[str, list[Any]], dict[str, str]]:
    """Count consecutive acceptances: ``(pending, confirmed)``.

    ``pending`` maps bay -> ``[serial, count]`` from the previous evaluation and
    ``accepted`` is this evaluation's :func:`assign` result. A pair accepted
    again increments its count, a new pair starts at 1, a bay not accepted
    now is forgotten. ``confirmed`` holds the pairs whose count reached
    :data:`CONFIRM_EVALUATIONS` (they leave ``pending``).
    """
    before = pending if isinstance(pending, Mapping) else {}
    out: dict[str, list[Any]] = {}
    confirmed: dict[str, str] = {}
    for bay, serial in accepted.items():
        prev = before.get(bay)
        count = 1
        if (
            isinstance(prev, Sequence)
            and len(prev) == 2
            and prev[0] == serial
            and isinstance(prev[1], int)
            and not isinstance(prev[1], bool)
        ):
            count = max(0, prev[1]) + 1
        if count >= CONFIRM_EVALUATIONS:
            confirmed[bay] = serial
        else:
            out[bay] = [serial, count]
    return out, confirmed


def top_candidates(
    scores: Mapping[str, Mapping[str, float]], bay: str, n: int = CANDIDATES_SHOWN
) -> list[list[Any]]:
    """The ``n`` best ``[serial, score]`` for ``bay``, best first."""
    rows = [(float(row[bay]), serial) for serial, row in scores.items() if bay in row]
    rows.sort(key=lambda p: (-p[0], p[1]))
    return [[serial, round(score, 4)] for score, serial in rows[:n]]
