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
   trusted and ``obs.temps`` has no key outside ``cfg.temps``. With a
   ``topology`` this whole-tick verdict is only informative: ``mpc.step``
   decides trust per zone from ``per_temp`` (``aqua_bridge.control.zones``).
5. :func:`push_window` / :func:`sanitize_temps` store raw values (``None``
   for missing / NaN / inf, never NaN) restricted to ``cfg.temps`` and trim
   the window to ``cfg.window_ticks`` entries (``cfg.stuck_ticks`` in legacy
   mode).

Per-sensor Stuck sizing (zoned DAS, plan section 0.2)
-----------------------------------------------------
A DS18B20 next to a drive with a time constant of about 12 minutes sits on
one 1/16 degC code for minutes while the fans of its zone move; the global
rule above would brand it Stuck and fault its zone. With ``topology``
every number of rule 3 comes from :meth:`MpcConfig.stuck_params`:

* the window (``sensors.<name>.stuck_s``, per role 1800 s proximal, 180 s
  zone air, 600 s inlet / exhaust) and the band (``stuck_eps_c``, default
  ``1.5 * quant_c``) are the sensor's own;
* the PWM evidence is the net move of the **zone's relative airflow**
  ``sum w * phi(u)`` over the channels of the sensor's zone (``phi`` the
  estimator's fan curve, ``w`` the fan count split evenly over the zones
  listing the channel, normalised) above ``stuck_airflow_net``, not one
  channel's PWM on its own: the solver moves a zone's channels apart (one up,
  a shared one down), and a shared single-fan output moves little of one
  zone's air. The move compares block means (:func:`_airflow_move`), so a fan
  dip of a tick or two at the window's start is not a move of the window. A
  sensor without a zone, such as an inlet in front of the intake, has none:
  fans do not move the inlet temperature;
* for a ``drive_proximal`` sensor that evidence does not count when every
  zone-air sensor of the zone with a plausible path over the window moved
  *against* it by more than ``stuck_air_oppose_c`` (warmer air after more
  airflow, cooler after less): the proximal reading mixes the zone air with
  the drive-to-air difference, which airflow moves the other way, and the
  two cancel on a healthy sensor;
* the sibling evidence counts only the other sensors of the same zone and
  the same role, and for a ``drive_proximal`` sensor only those of its own
  bay: another bay's reading follows that bay's drive heat, which does not
  reach this sensor (an idle drive's reading sits on one code for half an hour
  while a neighbour's activity burst moves that neighbour's reading by degrees).

Long windows are **decimated**: a sensor whose ``StuckParams.decimate`` is
``k > 1`` is checked on a window that keeps one sample every ``k`` ticks
(``StuckParams.samples`` of them). :func:`advance_slow_windows` feeds those
windows from the newest sample of ``MpcState.window`` -- the previous tick,
whose command the loop has already replaced by what was applied -- storing
the value the checks ran on (median3 when enabled) and the command. The
decimated check interpolates nothing: it compares the stored samples and
the current value against the band, measures the airflow move on the stored
commands (:func:`_airflow_move`), and bounds a sibling's or zone-air sensor's
single step by ``k * dT_max_tick``. A reading's excursion shorter than ``k``
ticks can therefore go unseen by the band check, which can only flag Stuck
*more* readily (its zone faults, cooling rises), never less.
The decimated windows live in ``solver_memory["stuck_slow"]`` (factor as a
string -> list of ``{"t": temps, "p": pwm}``) with the dense sample counter
``solver_memory["stuck_seq"]``; legacy mode has neither key.
"""

from __future__ import annotations

import math
from collections.abc import Callable, Iterable, Iterator, Mapping, Sequence
from dataclasses import dataclass, field
from typing import Any

from aqua_bridge.model import MpcConfig, PlantObservation, WindowSample

#: A decimated Stuck window sample: ``{"t": {temp: value | None}, "p": {channel: pwm}}``.
SlowSample = Mapping[str, Any]

__all__ = [
    "REASON_MISSING",
    "REASON_NON_FINITE",
    "REASON_NULL",
    "REASON_RANGE",
    "REASON_SLEW",
    "REASON_STUCK",
    "GateResult",
    "SlowSample",
    "advance_slow_windows",
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
    if type(value) is float:  # the common case, checked first (hot path, item 73)
        return value if math.isfinite(value) else None
    if value is None or isinstance(value, bool):
        return None
    try:
        f = float(value)  # type: ignore[arg-type]
    except (TypeError, ValueError, OverflowError):  # OverflowError: an int no float holds
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


def _filtered_stream(raw: Iterable[float | None], median3: bool) -> Iterator[float | None]:
    """The filtered value at every index of ``raw``, streamed (item 73).

    Yields ``median3_of(raw[: i + 1])`` for each ``i`` -- the value a check at
    that index runs on, each using only its own past -- one at a time, so a
    caller can stop as soon as it has seen enough. ``median3_of`` reads only the
    last three entries of the prefix and its length, so carrying the last three
    raw values is enough; ``median3`` false yields ``raw`` unchanged.
    """
    if not median3:
        yield from raw
        return
    tail: list[float | None] = []
    for value in raw:
        tail.append(value)
        if len(tail) > 3:
            del tail[0]
        yield median3_of(tail)


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


def _is_mapping(value: object) -> bool:
    """``isinstance(value, Mapping)`` with the stored ``dict`` answered first.

    ``Mapping`` is an ABC, so its ``isinstance`` walks the registry; this runs over
    every sample of every decimated window on every tick (item 73).
    """
    return type(value) is dict or isinstance(value, Mapping)


def _slow_sample_ok(sample: object) -> bool:
    """Structure of a stored decimated sample (values are read through ``_finite_or_none``)."""
    return _is_mapping(sample) and _is_mapping(sample.get("t")) and _is_mapping(sample.get("p"))  # type: ignore[union-attr]


def advance_slow_windows(
    slow: Mapping[str, Any] | None,
    window: Sequence[WindowSample],
    seq: int,
    cfg: MpcConfig,
) -> dict[str, list[dict[str, Any]]]:
    """Decimated Stuck windows for this tick (module docstring, *Per-sensor Stuck sizing*).

    ``window`` is the dense window as it arrives in ``step`` (newest entry =
    the previous tick, sample number ``seq - 1`` since the last history
    reset). Every decimation factor ``k`` of ``cfg.slow_window_samples``
    takes that sample when ``(seq - 1) % k == 0`` and keeps its newest
    ``samples`` entries. Pure: returns new lists, never mutates ``slow``.
    """
    sizes = cfg.slow_window_samples
    if not sizes:
        return {}
    out: dict[str, list[dict[str, Any]]] = {}
    newest: dict[str, Any] | None = None
    for k, keep in sizes.items():
        key = str(k)
        old = slow.get(key) if isinstance(slow, Mapping) else None
        samples = list(old) if isinstance(old, list) else []
        if not all(_slow_sample_ok(sample) for sample in samples):
            samples = []  # corrupt memory: start the window over, as after a gap
        if window and seq >= 1 and (seq - 1) % k == 0:
            if newest is None:
                last = window[-1]
                temps: dict[str, float | None] = {}
                for name in cfg.temps:
                    series = _window_series(window[-3:], name)
                    temps[name] = filtered_value(series[:-1], series[-1], cfg.median3)
                newest = {"t": temps, "p": dict(last.cmd_pwm)}
            samples.append(newest)
        out[key] = samples[-keep:]
    return out


def _band_reference(
    series: Iterator[float | None], current: float | None, eps_c: float
) -> float | None:
    """The oldest sample when it and every later one, ``current`` included, are within ``eps_c``.

    ``None`` as soon as a sample is unusable or outside the band, so a moving
    sensor costs a couple of samples instead of a whole window (item 73).
    """
    first = next(series, None)
    if first is None:
        return None
    for v in series:
        if v is None or abs(v - first) > eps_c:
            return None
    if current is None or abs(current - first) > eps_c:
        return None
    return first


def _stuck(
    name: str,
    cfg: MpcConfig,
    window: Sequence[WindowSample],
    filtered_now: Mapping[str, float | None],
    slow_windows: Mapping[str, Sequence[SlowSample]] | None = None,
) -> float | None:
    """Rule 3 (fresh window check) for one temperature.

    Returns the band reference (the oldest window sample) when the rule
    fires, ``None`` otherwise. Needs a full window; ``None`` anywhere in the
    run breaks it. The window, band and evidence come from
    ``cfg.stuck_params(name)`` (legacy mode: the global rule).
    """
    params = cfg.stuck_params(name)
    stream_of: Callable[[str], Iterator[float | None]]
    if params.decimate == 1:
        n = params.ticks
        if len(window) < n:
            return None
        recent = window[-n:]
        current = filtered_now.get(name)
        if current is None:
            return None

        def stream_of(other: str) -> Iterator[float | None]:
            return _filtered_stream(
                (_finite_or_none(w.raw_temps.get(other)) for w in recent), cfg.median3
            )

        commands: list[Mapping[str, Any]] = [w.cmd_pwm for w in recent]
        step_limit = cfg.dT_max_tick
    else:
        stored = None if slow_windows is None else slow_windows.get(str(params.decimate))
        m = params.samples
        if not stored or len(stored) < m:
            return None
        slow = stored[-m:]
        current = filtered_now.get(name)
        if current is None:
            return None

        def stream_of(other: str) -> Iterator[float | None]:
            return (_finite_or_none(sample["t"].get(other)) for sample in slow)

        commands = [sample["p"] for sample in slow]
        step_limit = cfg.dT_max_tick * params.decimate

    def series_of(other: str) -> list[float | None]:
        return list(stream_of(other))

    # Band check first and lazily: a sensor that is moving leaves the band within a few
    # samples, and then nothing else of its window has to be sanitised at all (item 73).
    first = _band_reference(stream_of(name), current, params.eps_c)
    if first is None:
        return None

    # Frozen. Did anything that should have moved it actually move?
    pwm_moves: list[tuple[float | None, float]]
    if params.airflow:  # zoned: the net move of the zone's relative airflow
        pwm_moves = [(_airflow_move(params.airflow, commands), cfg.stuck_airflow_net)]
    else:  # legacy / no zone: the net PWM move of each channel on its own
        # the PWM move must be old enough for the plant to have answered it
        oldest_pwm = commands[0]
        newest_pwm = commands[-1 - stuck_pwm_lag(len(commands))]
        pwm_moves = [
            (_pwm_move(oldest_pwm.get(ch), newest_pwm.get(ch)), cfg.stuck_pwm_net)
            for ch in params.channels
        ]
    air_moves: list[float] | None = None
    for move, threshold in pwm_moves:
        if move is None or abs(move) <= threshold:
            continue
        if params.air and air_moves is None:
            air_moves = []
            for air in params.air:
                air_move = _plausible_move(
                    cfg, [*series_of(air), filtered_now.get(air)], step_limit
                )
                if air_move is not None:
                    air_moves.append(air_move)
        if not _air_opposes(cfg, air_moves, move):
            return first
    for other in params.siblings:
        if _plausible_net_move(cfg, [*series_of(other), filtered_now.get(other)], step_limit):
            return first
    return None


def _pwm_move(old: object, new: object) -> float | None:
    """Net PWM move of one channel between two commands; ``None`` when either is unusable."""
    a = _finite_or_none(old)
    b = _finite_or_none(new)
    return None if a is None or b is None else b - a


def _airflow_move(
    airflow: Sequence[tuple[str, float, float, float]],
    commands: Sequence[Mapping[str, Any]],
) -> float | None:
    """Net move of a zone's relative airflow over a Stuck window (``StuckParams.airflow``).

    ``commands`` are the window's commands, oldest first (``m`` of them). With
    ``L = stuck_pwm_lag(m)`` and ``B = max(1, L)`` the move is the mean airflow
    of the ``B`` commands that end at the lagged one (``L`` before the newest)
    minus the mean of the first ``B``: a move must be ``L`` samples old to count,
    as in the legacy rule, and a short excursion of the command at either end
    (the DAS MPC dips a fan for a tick or two) counts only by the share of the
    block it fills. A drive with a time constant of minutes does not answer such
    a dip, so a single oldest sample caught in it read as a move of the whole
    window. The airflow of one command is ``sum w * phi(u)`` over the zone's
    channels with ``phi(u) = clip((u - deadband) / (1 - deadband), 0, 1) **
    exponent``, the estimator's fan curve; a channel without a usable command
    adds nothing, and a block without any usable command gives ``None``.
    """
    m = len(commands)
    lag = stuck_pwm_lag(m)
    block = max(1, lag)
    first = _mean_airflow(airflow, commands[:block])
    last = _mean_airflow(airflow, commands[m - lag - block : m - lag])
    return None if first is None or last is None else last - first


def _mean_airflow(
    airflow: Sequence[tuple[str, float, float, float]], commands: Sequence[Mapping[str, Any]]
) -> float | None:
    """Mean relative airflow of ``commands`` (see :func:`_airflow_move`)."""
    values: list[float] = []
    for command in commands:
        total = 0.0
        seen = False
        for ch, weight, deadband, exponent in airflow:
            u = _finite_or_none(command.get(ch))
            if u is not None:
                seen = True
                total += weight * _phi(u, deadband, exponent)
        if seen:
            values.append(total)
    return sum(values) / len(values) if values else None


def _phi(u: float, deadband: float, exponent: float) -> float:
    """Relative airflow of a fan at PWM ``u`` (``aqua_bridge.control.thermal.phi``)."""
    frac = min(1.0, max(0.0, (u - deadband) / (1.0 - deadband)))
    return frac**exponent if frac > 0 else 0.0


def _air_opposes(cfg: MpcConfig, air_moves: Sequence[float] | None, pwm_move: float) -> bool:
    """Whether the zone air moved against an airflow move (``StuckParams.air``, proximal only).

    More airflow cools the zone air and shrinks the drive-to-air difference,
    so a proximal reading falls; warmer air (a rising inlet) raises it by as
    much. When every zone-air sensor with a plausible path over the window
    (``air_moves``) moved against the airflow move -- warmer after more
    airflow, cooler after less -- by more than ``stuck_air_oppose_c``, the two
    effects can cancel on a healthy reading and the airflow move is no evidence
    that it should have moved. No plausible zone-air path (a dropout, a Spike)
    keeps the evidence, and so does a zone-air sensor that did not move (a
    frozen one included): flagging more readily only raises cooling.
    """
    if not air_moves:
        return False
    sign = 1.0 if pwm_move > 0 else -1.0
    return all(sign * move > cfg.stuck_air_oppose_c for move in air_moves)


def _plausible_move(
    cfg: MpcConfig, series: Sequence[float | None], step_limit: float | None = None
) -> float | None:
    """Net displacement of ``series`` when its trajectory could be the plant's, else ``None``.

    Every sample finite and inside the valid range, no single step larger
    than ``dT_max_tick`` (a plant cannot do that; a sensor Spike or Jump
    can).
    """
    if len(series) < 2 or any(v is None for v in series):
        return None
    vals = [float(v) for v in series]  # type: ignore[arg-type]
    if any(not cfg.temp_min_c <= v <= cfg.temp_max_c for v in vals):
        return None
    limit = cfg.dT_max_tick if step_limit is None else step_limit
    if any(abs(b - a) > limit for a, b in zip(vals, vals[1:], strict=False)):
        return None
    return vals[-1] - vals[0]


def _plausible_net_move(
    cfg: MpcConfig, series: Sequence[float | None], step_limit: float | None = None
) -> bool:
    """A sibling counts as "moved" only when its trajectory could be the plant's.

    Every sample finite and inside the valid range, no single step larger
    than ``dT_max_tick`` (a plant cannot do that; a sensor Spike or Jump
    can), and a net displacement above ``stuck_sibling_dT_c``. Without this
    a Spike or Jump on one sensor would brand a calm sibling as Stuck for a
    whole window and trap the Jump in fallback.
    """
    move = _plausible_move(cfg, series, step_limit)
    return move is not None and abs(move) > cfg.stuck_sibling_dT_c


def evaluate_gate(
    obs: PlantObservation,
    cfg: MpcConfig,
    *,
    last_good_obs: PlantObservation | None,
    last_raw_temps: Mapping[str, float | None] | None,
    window: Sequence[WindowSample],
    dT_limit: float | None = None,  # noqa: N803 - matches the spec's dT naming
    stuck_latch: Mapping[str, float] | None = None,
    slow_windows: Mapping[str, Sequence[SlowSample]] | None = None,
) -> GateResult:
    """Run rules 1-4 on ``obs``. Never raises for malformed temperatures.

    ``dT_limit`` overrides ``cfg.dT_max_tick`` (``mpc.step`` scales it when
    the observation arrives late). ``stuck_latch`` is the previous call's
    ``GateResult.stuck_latch`` (rule 3 latch); ``None`` / empty means no
    temperature is currently latched. ``slow_windows`` are the decimated
    Stuck windows of :func:`advance_slow_windows` (only a config with a
    ``topology`` has decimated sensors). Missing history (``None`` / empty)
    is allowed and simply disables the slew and stuck checks that need it.
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
    # Both the median3 filter and the previous filtered value read at most the newest
    # three window samples, so the whole window never has to be sanitised here (item 73).
    tail = window[-3:] if cfg.median3 else window[-1:]
    for name in cfg.temps:
        history = _window_series(tail, name)
        filtered[name] = filtered_value(history, raw[name], cfg.median3)
        if history:
            prev_ref[name] = median3_of(history) if cfg.median3 else history[-1]
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
            if latched is not None and abs(value - latched) <= cfg.stuck_params(name).eps_c:
                stuck_ref: float | None = latched  # still inside the band: flag stays
            else:  # left the band (or never in)
                stuck_ref = _stuck(name, cfg, window, filtered, slow_windows)
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
