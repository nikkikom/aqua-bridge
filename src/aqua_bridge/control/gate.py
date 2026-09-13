"""Sensor gate: decides whether one raw observation may reach the solver.

PROJECT.md section 3, "Trusted tick (sensor gate), whole observation". The
gate is a pure function of the observation, the config and a few pieces
of :class:`~aqua_bridge.model.MpcState`; it never raises on malformed
temperatures (missing / extra / ``None`` / NaN keys make the tick
untrusted) and it never touches PWM. ``mpc.step`` calls it, then pushes
the raw sample and the emitted command into the window with
:func:`push_window` (gate rule 5).

Rules implemented (numbers as in the spec):

1. **Pre-filter** only when ``cfg.median3``: the value checked is the
   median of the last three *raw* samples (the two most recent window
   entries plus the current one). With fewer than three finite samples the
   current raw value is used unfiltered. The "previous raw" slew reference
   is then the previous tick's *filtered* value (recomputed from the
   window), so a Jump surfaces one tick later and confirms one tick later,
   exactly as the spec describes; ``last_raw_temps`` itself always stores
   raw values (rule 5).
2. **Range and slew**, per temperature in ``cfg.temps``: the value must be
   finite, inside ``[cfg.temp_min_c, cfg.temp_max_c]`` and within
   ``dT_limit`` (default ``cfg.dT_max_tick``) of *at least one* of
   ``last_good_obs.temps[name]`` and the previous (filtered) raw value.
   When neither reference exists (cold state) the slew check passes: with
   no history nothing can be compared, and a controller that could never
   start would be the less safe choice.
3. **Stuck**: over the last ``cfg.stuck_ticks`` window samples plus the
   current one, every (pre-filter) value lies within ``cfg.stuck_eps_c`` of
   the oldest window sample, and in that same window either the *net*
   commanded PWM displacement on some channel exceeds ``cfg.stuck_pwm_net``
   or another temperature in ``cfg.temps`` moved (net) by more than
   ``cfg.stuck_sibling_dT_c`` along a physically plausible path (finite, in
   range, no single step above ``dT_max_tick`` -- a Spike or Jump on the
   sibling is a sensor event, not plant motion, and must not brand a calm
   sensor as Stuck). The net PWM displacement is measured from the oldest
   window sample to the sample ``stuck_ticks // 4`` ticks *before* the
   newest (:func:`stuck_pwm_lag`), not to the newest itself: the spec's
   own sizing rule says the window exists so that "the coolant must have
   had time to answer a PWM move", and a move commanded two ticks ago has
   had no such time. Taken literally (``|pwm[t] - pwm[t - stuck_ticks]|``)
   the rule flags a perfectly calm, healthy sensor every time the command
   moves faster than ``stuck_pwm_net`` within a couple of ticks from a
   still equilibrium -- a setpoint step or the return from a fault -- and
   nominal operation would flicker into fallback. A sustained ramp across
   the window is still caught (three quarters of it are counted). The
   check needs a full window; a ``None`` inside the run breaks it (the
   dropout tick was untrusted anyway). Once fired the flag is **latched** on the
   band's reference (the oldest window sample at that moment) and stays
   set while the value remains within ``stuck_eps_c`` of it -- the spec
   says "the flag clears when the value leaves the band", not when the
   evidence scrolls out of the window. Without the latch the fallback hold
   freezes the command, one window later the net PWM move is gone, the
   frozen reading confirms as "at setpoint" and the controller limit-cycles
   between auto and fallback on a dead sensor. The latch is the caller's to
   keep (``mpc.step`` stores it in ``solver_memory["stuck_latch"]``): pass
   it in as ``stuck_latch`` and read the next one from
   ``GateResult.stuck_latch``. A ``None`` value keeps the latch (nothing
   proved the sensor alive).
4. **Whole tick**: trusted iff every temperature in ``cfg.temps`` is
   trusted and ``obs.temps`` has no key outside ``cfg.temps``.
5. :func:`push_window` / :func:`sanitize_temps` store raw values (``None``
   for missing / NaN / inf, never NaN) restricted to ``cfg.temps`` and trim
   the window to ``cfg.stuck_ticks`` entries.
"""

from __future__ import annotations

import math
from collections.abc import Iterable, Mapping, Sequence
from dataclasses import dataclass, field
from typing import Any

from aqua_bridge.model import MpcConfig, PlantObservation, WindowSample

__all__ = [
    "REASON_MISSING",
    "REASON_NON_FINITE",
    "REASON_NULL",
    "REASON_RANGE",
    "REASON_SLEW",
    "REASON_STUCK",
    "GateResult",
    "evaluate_gate",
    "filtered_value",
    "median3_of",
    "push_window",
    "sanitize_temps",
    "stuck_pwm_lag",
]

# Per-temperature untrust reasons (strings so they serialise into diagnostics).
REASON_MISSING = "missing"  # key absent from obs.temps
REASON_NULL = "null"  # value is None
REASON_NON_FINITE = "non_finite"  # NaN or +-inf
REASON_RANGE = "range"  # outside [temp_min_c, temp_max_c]
REASON_SLEW = "slew"  # too far from both references
REASON_STUCK = "stuck"  # frozen while PWM or a sibling moved


@dataclass(frozen=True)
class GateResult:
    """Outcome of :func:`evaluate_gate` for one observation.

    * ``trusted``      -- whole-tick verdict (rule 4)
    * ``per_temp``     -- verdict per temperature in ``cfg.temps``
    * ``filtered``     -- the value each check ran on (median3 or raw); ``None`` if unusable
    * ``raw``          -- sanitised raw values for ``cfg.temps`` (what goes into the window)
    * ``reasons``      -- per temperature, the failed rules in check order (empty when trusted)
    * ``unknown_keys`` -- keys of ``obs.temps`` that are not in ``cfg.temps``
    * ``stuck``        -- per temperature, whether the Stuck rule is active (fresh or latched)
    * ``stuck_latch``  -- per latched temperature, the band reference; feed back into the
      next call (``mpc.step`` keeps it in ``solver_memory["stuck_latch"]``)
    """

    trusted: bool
    per_temp: dict[str, bool]
    filtered: dict[str, float | None]
    raw: dict[str, float | None]
    reasons: dict[str, tuple[str, ...]]
    unknown_keys: tuple[str, ...]
    stuck: dict[str, bool]
    stuck_latch: dict[str, float] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        """JSON-serialisable form for ``MpcCommand.diagnostics``; never contains NaN."""
        return {
            "trusted": self.trusted,
            "per_temp": dict(self.per_temp),
            "filtered": dict(self.filtered),
            "raw": dict(self.raw),
            "reasons": {k: list(v) for k, v in self.reasons.items() if v},
            "unknown_keys": list(self.unknown_keys),
            "stuck": dict(self.stuck),
            "stuck_latch": dict(self.stuck_latch),
        }


def _finite_or_none(value: object) -> float | None:
    if value is None or isinstance(value, bool):
        return None
    try:
        f = float(value)  # type: ignore[arg-type]
    except (TypeError, ValueError):
        return None
    return f if math.isfinite(f) else None


def sanitize_temps(
    temps: Mapping[str, float | None] | None, names: Iterable[str]
) -> dict[str, float | None]:
    """Raw temperatures restricted to ``names``; ``None`` for missing / NaN / inf.

    This is what the gate stores in ``WindowSample.raw_temps`` and
    ``MpcState.last_raw_temps`` (NaN is never stored).
    """
    out: dict[str, float | None] = {}
    for name in names:
        out[name] = None if temps is None else _finite_or_none(temps.get(name))
    return out


def median3_of(values: Sequence[float | None]) -> float | None:
    """Median of the last three entries; the last entry itself when fewer than three are finite.

    ``values`` is oldest first. A ``None`` (dropout) among the three
    disables the filter for this tick instead of hiding the dropout.
    """
    if not values:
        return None
    last3 = list(values[-3:])
    if len(last3) < 3 or any(v is None for v in last3):
        return last3[-1]
    return sorted(last3)[1]  # type: ignore[type-var]


def filtered_value(
    history: Sequence[float | None], current: float | None, median3: bool
) -> float | None:
    """The value the checks run on: ``current`` raw, or median3 over ``history + [current]``."""
    if not median3:
        return current
    return median3_of([*history, current])


def _window_series(window: Sequence[WindowSample], name: str) -> list[float | None]:
    return [_finite_or_none(w.raw_temps.get(name)) for w in window]


def _filtered_series(raw: Sequence[float | None], median3: bool) -> list[float | None]:
    """Filtered value at every index of ``raw`` (each uses only its own past)."""
    if not median3:
        return list(raw)
    return [median3_of(raw[: i + 1]) for i in range(len(raw))]


def push_window(
    window: Sequence[WindowSample],
    raw_temps: Mapping[str, float | None],
    cmd_pwm: Mapping[str, float],
    stuck_ticks: int,
) -> tuple[WindowSample, ...]:
    """Append this tick's sanitised raw temps and command, keep the newest ``stuck_ticks``."""
    sample = WindowSample(raw_temps=dict(raw_temps), cmd_pwm=dict(cmd_pwm))
    out = (*window, sample)
    if len(out) > stuck_ticks:
        out = out[-stuck_ticks:]
    return out


def stuck_pwm_lag(stuck_ticks: int) -> int:
    """Ticks a PWM move must be old before it counts as Stuck evidence (rule 3).

    A quarter of the window (the window is sized at several plant time
    constants, so a quarter of it is still a plant-scale delay), at least
    one command interval left to compare over.
    """
    return max(0, min(stuck_ticks // 4, stuck_ticks - 2))


def _stuck(
    name: str,
    cfg: MpcConfig,
    window: Sequence[WindowSample],
    filtered_now: Mapping[str, float | None],
) -> float | None:
    """Rule 3 (fresh window check) for one temperature.

    Returns the band reference (the oldest window sample) when the rule
    fires, ``None`` otherwise. Needs a full window; ``None`` anywhere in the
    run breaks it.
    """
    n = cfg.stuck_ticks
    if len(window) < n:
        return None
    recent = window[-n:]
    current = filtered_now.get(name)
    if current is None:
        return None
    series = _filtered_series(_window_series(recent, name), cfg.median3)
    first = series[0]
    if first is None:
        return None
    for v in [*series[1:], current]:
        if v is None or abs(v - first) > cfg.stuck_eps_c:
            return None

    # Frozen. Did anything that should have moved it actually move? (rule 3:
    # the PWM move must be old enough for the plant to have answered it)
    oldest_pwm = recent[0].cmd_pwm
    newest_pwm = recent[-1 - stuck_pwm_lag(n)].cmd_pwm
    for ch in cfg.channels:
        a = _finite_or_none(oldest_pwm.get(ch))
        b = _finite_or_none(newest_pwm.get(ch))
        if a is not None and b is not None and abs(b - a) > cfg.stuck_pwm_net:
            return first
    for other in cfg.temps:
        if other == name:
            continue
        series_other = _filtered_series(_window_series(recent, other), cfg.median3)
        if _plausible_net_move(cfg, [*series_other, filtered_now.get(other)]):
            return first
    return None


def _plausible_net_move(cfg: MpcConfig, series: Sequence[float | None]) -> bool:
    """A sibling counts as "moved" only when its trajectory could be the plant's.

    Every sample finite and inside the valid range, no single step larger
    than ``dT_max_tick`` (a plant cannot do that; a sensor Spike or Jump
    can), and a net displacement above ``stuck_sibling_dT_c``. Without this
    a Spike or Jump on one sensor would brand a calm sibling as Stuck for a
    whole window and trap the Jump in fallback.
    """
    if len(series) < 2 or any(v is None for v in series):
        return False
    vals = [float(v) for v in series]  # type: ignore[arg-type]
    if any(not cfg.temp_min_c <= v <= cfg.temp_max_c for v in vals):
        return False
    if any(abs(b - a) > cfg.dT_max_tick for a, b in zip(vals, vals[1:], strict=False)):
        return False
    return abs(vals[-1] - vals[0]) > cfg.stuck_sibling_dT_c


def evaluate_gate(
    obs: PlantObservation,
    cfg: MpcConfig,
    *,
    last_good_obs: PlantObservation | None,
    last_raw_temps: Mapping[str, float | None] | None,
    window: Sequence[WindowSample],
    dT_limit: float | None = None,  # noqa: N803 - matches the spec's dT naming
    stuck_latch: Mapping[str, float] | None = None,
) -> GateResult:
    """Run rules 1-4 on ``obs``. Never raises for malformed temperatures.

    ``dT_limit`` overrides ``cfg.dT_max_tick`` (``mpc.step`` scales it when
    the observation arrives late). ``stuck_latch`` is the previous call's
    ``GateResult.stuck_latch`` (rule 3 latch); ``None`` / empty means no
    temperature is currently latched. Missing history (``None`` / empty) is
    allowed and simply disables the slew and stuck checks that need it.
    """
    limit = cfg.dT_max_tick if dT_limit is None else float(dT_limit)
    latch_in: dict[str, float] = {}
    if stuck_latch is not None:
        for name in cfg.temps:
            ref = _finite_or_none(stuck_latch.get(name))
            if ref is not None:
                latch_in[name] = ref
    raw = sanitize_temps(obs.temps, cfg.temps)
    unknown = tuple(sorted(k for k in obs.temps if k not in cfg.temps))

    filtered: dict[str, float | None] = {}
    prev_ref: dict[str, float | None] = {}
    for name in cfg.temps:
        history = _window_series(window, name)
        filtered[name] = filtered_value(history, raw[name], cfg.median3)
        if history:
            prev_ref[name] = _filtered_series(history, cfg.median3)[-1]
        elif last_raw_temps is not None:
            prev_ref[name] = _finite_or_none(last_raw_temps.get(name))
        else:
            prev_ref[name] = None

    per_temp: dict[str, bool] = {}
    reasons: dict[str, tuple[str, ...]] = {}
    stuck: dict[str, bool] = {}
    latch_out: dict[str, float] = {}
    for name in cfg.temps:
        why: list[str] = []
        value = filtered[name]
        if name not in obs.temps:
            why.append(REASON_MISSING)
        elif obs.temps[name] is None:
            why.append(REASON_NULL)
        elif value is None:
            why.append(REASON_NON_FINITE)

        if value is not None:
            if not cfg.temp_min_c <= value <= cfg.temp_max_c:
                why.append(REASON_RANGE)
            refs: list[float] = []
            if last_good_obs is not None:
                g = _finite_or_none(last_good_obs.temps.get(name))
                if g is not None:
                    refs.append(g)
            if prev_ref[name] is not None:
                refs.append(prev_ref[name])  # type: ignore[arg-type]
            if refs and not any(abs(value - r) <= limit for r in refs):
                why.append(REASON_SLEW)
            latched = latch_in.get(name)
            if latched is not None and abs(value - latched) <= cfg.stuck_eps_c:
                stuck_ref: float | None = latched  # still inside the band: flag stays
            else:
                stuck_ref = _stuck(name, cfg, window, filtered)  # left the band (or never in)
            stuck[name] = stuck_ref is not None
            if stuck_ref is not None:
                latch_out[name] = stuck_ref
                why.append(REASON_STUCK)
        else:
            stuck[name] = False
            if name in latch_in:  # no usable value: nothing proved the sensor alive
                latch_out[name] = latch_in[name]

        per_temp[name] = not why
        reasons[name] = tuple(why)

    trusted = not unknown and all(per_temp.values())
    return GateResult(
        trusted=trusted,
        per_temp=per_temp,
        filtered=filtered,
        raw=raw,
        reasons=reasons,
        unknown_keys=unknown,
        stuck=stuck,
        stuck_latch=latch_out,
    )
