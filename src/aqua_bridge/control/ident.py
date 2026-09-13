"""Active identification experiments on fan groups (DAS plan section 5, "Active experiments").

Pure state machine: every function here is a function of its arguments, keeps
its state in plain JSON dicts, reads no clock (time is the observation clock
``obs.ts`` handed in through :class:`TickFacts`), draws no randomness (the hold
times come from a seeded LFSR) and does no I/O. The
:class:`~aqua_bridge.control.supervisor.Supervisor` drives it: it starts an
experiment on an explicit intent (``POST /api/ident``, MQTT ``cmd/ident``),
advances it once per loop tick from what that tick did, and expresses the
experiment's PWM levels as ordinary overrides. ``compose`` therefore rate
limits and clamps them, fallback beats them exactly as it beats a human
override, and ``mpc.step`` and the loop do not know an experiment exists. The
thermal model learns from experiment ticks like from any trusted tick; the
experiment only supplies excitation.

Unit of excitation and sequence
-------------------------------
The unit is a **fan group** (``fans.<channel>.group``; a channel without a
group is a group of its own, named like the channel). ``start`` on a group runs
the group level first (every channel of the group together) and then, for a
group of several channels, each channel alone, so the thermal model can split
the group's gain between them; ``start`` on a channel runs that channel alone.
``ident_max_duration_s`` is split evenly over the phases. While one channel of a
group runs alone the group's other channels are held at their frozen base (a
solver-driven sibling would move against the experiment and make the two
regressors collinear again; holding it at its level at start never gives less
cooling than the solver asked for then).

Each channel's base ``u_base`` is the solver's command for it on the last tick
before the start, frozen for the whole experiment. The two levels are
``ident_levels``:

* ``above`` (default): ``u_base`` and ``u_base + ident_amplitude``. The experiment
  never commands less than the solver's level at start (the safe direction);
* ``symmetric`` (owner opt-in): ``u_base - A`` and ``u_base + A``.

A start is refused when a level would leave ``[pwm_min, pwm_max]`` (``band``),
so no level is ever clipped. The sequence is a two-level random telegraph
signal: every phase starts on the high level, the level alternates after each
hold, and each hold is drawn from ``ident_hold_s`` by a 16-bit Galois LFSR
(taps ``0xB400``) seeded from ``ident_seed``; the generator runs on across the
phases, and the last hold of a phase is cut at the phase end. The schedule is
computed once at the start, in plain JSON, and is the same for the same config,
target and base commands.

Time: the start is armed between ticks; offset 0 is the tick after the last
recorded one (``last ts + dt``). After every tick the machine checks that tick
and computes the overrides for the next one at offset ``ts + dt - start``. The
experiment completes when that offset reaches the duration.

Preconditions (:func:`check_start`, each failure has a named reason)
--------------------------------------------------------------------
``ident_enabled`` and a DAS config are checked by the supervisor, which also
refuses a second start while one runs. Here:

* ``control_mode`` -- the control mode is ``auto`` and no human override is set;
* ``no_tick`` / ``mode:<mode>`` -- the last command exists and its mode is ``auto``
  (not ``saturated``, ``degraded`` or ``fallback``);
* ``saturated:<ch>``, ``no_command:<ch>``, ``band:<ch>`` -- per channel of the target;
  ``fan_stall:<ch>`` -- on any channel;
* ``settle:<zone>`` -- every zone the target serves (``ZoneLayout.served``: the zones
  listing one of its channels plus their declared ``coupled_to``) has been trusted
  and fault-free for ``ident_settle_s`` (tracked by :func:`track` from every tick; a
  restart starts the count over);
* ``bay_unknown:<bay>`` / ``bay_transition:<bay>`` -- no bay of those zones is
  ``unknown``, has an occupancy change pending (``pending_empty_s`` or
  ``pending_occupied_ticks`` of the estimator) or changed occupancy less than
  ``estimator.bay_settle_s`` ago;
* ``calibrating:<bay>`` -- no bay of those zones has a SMART calibration still in its
  first 20 samples (:data:`~aqua_bridge.control.estimator.CAL_MIN_SAMPLES`);
* ``start_band:<bay>`` -- every occupied or unknown bay of those zones has
  ``T_hat + k sigma <= soft + ident_start_band_c``, plus the running envelope below.

Envelope (checked on every tick while running, and at start)
-------------------------------------------------------------
For every occupied or unknown bay of the served zones, on the estimates of the
tick (``diagnostics["estimates"]``: ``t_c``, ``margin_c = k sigma``, ``soft_c``,
``hard_c``, ``limit_c``), with ``upper = T_hat + k sigma``:

* ``envelope:<bay>``   -- ``upper > soft + ident_max_over_c`` (3.0 degC, owner-accepted);
* ``hard:<bay>``       -- ``upper > hard`` (always);
* ``abort_temp:<bay>`` -- ``upper >= limit - 1`` (:data:`ABORT_BELOW_LIMIT_C`): the base
  plan's absolute ``ident_abort_temp_c`` becomes per drive class (the bay's limit in
  force), so there is no ``ident_abort_temp_c`` key;
* ``no_estimate:<bay>`` -- the bay has no finite estimate.

The formulas are the plan's as written: ``soft`` and ``hard`` already subtract
``k sigma``, so ``upper <= hard`` means ``T_hat <= limit - 2 k sigma`` (conservative,
the same double count as PI-DAS). Under PI-DAS, which regulates ``upper`` to
``soft``, a settled enclosure starts and runs inside this envelope. The DAS MPC
counts ``k sigma`` once and rides ``T_hat = soft``, i.e. ``upper = soft + k sigma``;
with the uncalibrated sigma (``k sigma`` about 3 degC) such a state is refused at
start and aborts at once. That is acceptable: the DAS MPC only acts on a converged
model, which is what experiments are run to obtain, so experiments run under its
PI-DAS fallback in practice.

Abort list (:func:`advance`; abort = release the override)
----------------------------------------------------------
``fallback`` (a fallback tick or a tick without a solver command, e.g. a
controller error), ``degraded`` (any zone in fault anywhere), ``apply_failed``
(:data:`APPLY_FAILURES_ABORT` consecutive failed writes), ``clock`` (no finite ``ts``
or ``ts`` running backwards), ``duration`` (``ts`` beyond the duration plus one tick:
a clock jump), the envelope reasons above, ``zone_untrusted:<zone>`` and
``bay_unknown:<bay>`` in a served zone, ``fan_stall:<ch>`` on an experiment channel,
a stop intent (``stop``) and any human intent (``human_intent:<kind>``, by the
supervisor). A daemon restart never resumes: the experiment lives only in the
supervisor's memory, as do the settle timers.

Release: the supervisor hands the experiment's channels to the loop as
``TickPlan.released``, which drops their integrator entries so the solver
re-initialises bumplessly (its first output equals the level on the fan), and the
fan moves at most ``d_pwm_max`` per tick from there; there is no separate return
ramp.
"""

from __future__ import annotations

import math
from collections.abc import Mapping
from dataclasses import dataclass, field
from typing import Any

from aqua_bridge.control.estimator import CAL_MIN_SAMPLES, EMPTY, UNKNOWN
from aqua_bridge.model import MpcCommand, MpcConfig

__all__ = [
    "ABORT_BELOW_LIMIT_C",
    "ACTIONS",
    "APPLY_FAILURES_ABORT",
    "LEVEL_HIGH",
    "LEVEL_LOW",
    "LFSR_TAPS",
    "RESULT_ABORTED",
    "RESULT_COMPLETED",
    "TARGET_KINDS",
    "Advance",
    "TickFacts",
    "advance",
    "check_start",
    "envelope_violations",
    "facts_from_tick",
    "group_channels",
    "groups",
    "hold_sequence",
    "levels_at",
    "new_tracker",
    "served_zones",
    "start",
    "status",
    "target_channels",
    "track",
]

#: The absolute abort sits this far below each drive's limit in force, degC.
ABORT_BELOW_LIMIT_C = 1.0
#: This many consecutive failed writes abort a running experiment.
APPLY_FAILURES_ABORT = 2
#: Galois LFSR taps (16 bit, maximal length) of the hold-time sequence.
LFSR_TAPS = 0xB400
LEVEL_LOW = 0
LEVEL_HIGH = 1
#: ``POST /api/ident`` actions and target kinds.
ACTIONS: tuple[str, ...] = ("start", "stop")
TARGET_KINDS: tuple[str, ...] = ("group", "channel")
RESULT_COMPLETED = "completed"
RESULT_ABORTED = "aborted"

_EPS = 1e-9


# ---------------------------------------------------------------------------
# Groups, targets and served zones
# ---------------------------------------------------------------------------


def _group_of(cfg: MpcConfig, channel: str) -> str:
    spec = cfg.fans.get(channel)
    return channel if spec is None or spec.group is None else spec.group


def groups(cfg: MpcConfig) -> dict[str, tuple[str, ...]]:
    """Fan group -> its channels in ``cfg.channels`` order (a channel without a group is
    its own group)."""
    out: dict[str, list[str]] = {}
    for ch in cfg.channels:
        out.setdefault(_group_of(cfg, ch), []).append(ch)
    return {g: tuple(chs) for g, chs in out.items()}


def group_channels(cfg: MpcConfig, group: str) -> tuple[str, ...]:
    """Channels of ``group``; ``KeyError`` for an unknown group."""
    return groups(cfg)[group]


def target_channels(cfg: MpcConfig, kind: str, name: str) -> tuple[str, ...]:
    """Every channel an experiment on ``kind`` ``name`` commands; ``KeyError`` if unknown.

    A channel target commands only that channel; a group target every channel of
    the group (the group phase, then each channel with its siblings at base)."""
    if kind == "channel":
        if name not in cfg.channels:
            raise KeyError(name)
        return (name,)
    if kind == "group":
        return group_channels(cfg, name)
    raise KeyError(kind)


def _phases(cfg: MpcConfig, kind: str, name: str) -> list[tuple[str, ...]]:
    channels = target_channels(cfg, kind, name)
    if kind == "channel" or len(channels) == 1:
        return [channels]
    return [channels, *((ch,) for ch in channels)]


def served_zones(cfg: MpcConfig, channels: tuple[str, ...] | list[str]) -> tuple[str, ...]:
    """Zones whose drives an experiment on ``channels`` may warm (declared, with coupling)."""
    layout = cfg.zone_layout
    members: set[str] = set()
    for ch in channels:
        members.update(layout.served.get(ch, layout.channel_zones.get(ch, ())))
    return tuple(z for z in layout.zones if z in members)


# ---------------------------------------------------------------------------
# What one tick did
# ---------------------------------------------------------------------------


def _finite(value: object) -> bool:
    return (
        isinstance(value, int | float)
        and not isinstance(value, bool)
        and math.isfinite(float(value))
    )


def _mapping(value: object) -> dict[str, Any]:
    return dict(value) if isinstance(value, Mapping) else {}


@dataclass(frozen=True)
class TickFacts:
    """The facts of one loop tick the experiment machine reads.

    ``mode`` is the solver command's mode (``None``: no solver command, e.g. a
    controller error); ``pwm`` the solver's command per channel; ``zones``,
    ``estimates``, ``bays``, ``saturated`` and ``fan_stall`` the matching entries of
    its diagnostics (empty when absent or malformed, which every check reads as
    unsafe).
    """

    ts: float | None
    mode: str | None
    applied: bool
    pwm: dict[str, float] = field(default_factory=dict)
    zones: dict[str, Any] = field(default_factory=dict)
    estimates: dict[str, Any] = field(default_factory=dict)
    bays: dict[str, Any] = field(default_factory=dict)
    saturated: dict[str, Any] = field(default_factory=dict)
    fan_stall: dict[str, Any] = field(default_factory=dict)


def facts_from_tick(mpc_cmd: MpcCommand | None, *, ts: float | None, applied: bool) -> TickFacts:
    """:class:`TickFacts` from the solver command of a tick (before ``compose``)."""
    ts_out = float(ts) if _finite(ts) else None
    if mpc_cmd is None:
        return TickFacts(ts=ts_out, mode=None, applied=bool(applied))
    diag = mpc_cmd.diagnostics if isinstance(mpc_cmd.diagnostics, Mapping) else {}
    return TickFacts(
        ts=ts_out,
        mode=mpc_cmd.mode.value,
        applied=bool(applied),
        pwm={ch: float(v) for ch, v in mpc_cmd.pwm.items()},
        zones=_mapping(diag.get("zones")),
        estimates=_mapping(diag.get("estimates")),
        bays=_mapping(diag.get("bays")),
        saturated=_mapping(diag.get("saturated")),
        fan_stall=_mapping(diag.get("fan_stall")),
    )


# ---------------------------------------------------------------------------
# Settle tracking (every tick, running or not)
# ---------------------------------------------------------------------------


def new_tracker() -> dict[str, Any]:
    """Empty settle tracker: ``{"ts": last ts, "ok_since": {zone: ts}}``."""
    return {"ts": None, "ok_since": {}}


def track(tracker: Mapping[str, Any], cfg: MpcConfig, facts: TickFacts) -> dict[str, Any]:
    """The tracker after ``facts``: per zone, the ``ts`` since which it has been trusted
    and fault-free without a break. A tick without a finite ``ts``, a clock running
    backwards or a tick without zone diagnostics starts every count over."""
    last = tracker.get("ts") if isinstance(tracker, Mapping) else None
    old = _mapping(tracker.get("ok_since")) if isinstance(tracker, Mapping) else {}
    ts = facts.ts
    if ts is None or (_finite(last) and ts < float(last)) or not facts.zones:
        return {"ts": ts, "ok_since": {}}
    ok_since: dict[str, float] = {}
    for zone in cfg.zone_layout.zones:
        info = facts.zones.get(zone)
        if not isinstance(info, Mapping) or info.get("trusted") is not True:
            continue
        if info.get("fault") is not False:
            continue
        since = old.get(zone)
        ok_since[zone] = float(since) if _finite(since) else ts
    return {"ts": ts, "ok_since": ok_since}


# ---------------------------------------------------------------------------
# Envelope and preconditions
# ---------------------------------------------------------------------------


def _constrained_bays(cfg: MpcConfig, facts: TickFacts, zones: tuple[str, ...]) -> list[str]:
    """Bays of ``zones`` that carry a constraint (not ``empty`` by the estimator; a bay
    the estimator does not report counts, conservatively)."""
    topo = cfg.topology
    if topo is None:
        return []
    wanted = set(zones)
    out = []
    for bay, spec in topo.bays.items():
        if spec.zone not in wanted:
            continue
        info = facts.bays.get(bay)
        occupancy = info.get("occupancy") if isinstance(info, Mapping) else None
        if occupancy == EMPTY:
            continue
        out.append(bay)
    return out


def envelope_violations(
    cfg: MpcConfig, facts: TickFacts, zones: tuple[str, ...], over_c: float
) -> list[str]:
    """Envelope reasons (module docstring) for the drives of ``zones``, with ``over_c``
    allowed above each soft target."""
    reasons: list[str] = []
    for bay in _constrained_bays(cfg, facts, zones):
        est = facts.estimates.get(bay)
        keys = ("t_c", "margin_c", "soft_c", "hard_c", "limit_c")
        if not isinstance(est, Mapping) or not all(_finite(est.get(k)) for k in keys):
            reasons.append(f"no_estimate:{bay}")
            continue
        upper = float(est["t_c"]) + float(est["margin_c"])
        if upper > float(est["soft_c"]) + over_c + _EPS:
            reasons.append(f"envelope:{bay}")
        if upper > float(est["hard_c"]) + _EPS:
            reasons.append(f"hard:{bay}")
        if upper >= float(est["limit_c"]) - ABORT_BELOW_LIMIT_C - _EPS:
            reasons.append(f"abort_temp:{bay}")
    return reasons


def _levels(cfg: MpcConfig, base: float) -> tuple[float, float]:
    a = cfg.ident_amplitude
    if cfg.ident_levels == "symmetric":
        return base - a, base + a
    return base, base + a


def check_start(
    cfg: MpcConfig,
    tracker: Mapping[str, Any],
    facts: TickFacts | None,
    kind: str,
    name: str,
    *,
    human_control: bool,
) -> list[str]:
    """Every reason an experiment on ``kind`` ``name`` may not start now (empty: it may).

    ``human_control`` is true when the control mode is not ``auto`` or a human
    override is set. ``KeyError`` for an unknown target."""
    channels = target_channels(cfg, kind, name)
    reasons: list[str] = []
    if human_control:
        reasons.append("control_mode")
    if facts is None or facts.mode is None:
        reasons.append("no_tick")
        return reasons
    if facts.mode != "auto":
        reasons.append(f"mode:{facts.mode}")
    for ch in channels:
        base = facts.pwm.get(ch)
        if not _finite(base):
            reasons.append(f"no_command:{ch}")
            continue
        if facts.saturated.get(ch) is not False:
            reasons.append(f"saturated:{ch}")
        lo, hi = _levels(cfg, float(base))  # type: ignore[arg-type]
        if lo < cfg.pwm_min - _EPS or hi > cfg.pwm_max + _EPS:
            reasons.append(f"band:{ch}")
    for ch in cfg.channels:
        if facts.fan_stall.get(ch):
            reasons.append(f"fan_stall:{ch}")
    zones = served_zones(cfg, channels)
    ok_since = _mapping(tracker.get("ok_since")) if isinstance(tracker, Mapping) else {}
    for zone in zones:
        since = ok_since.get(zone)
        if (
            facts.ts is None
            or not _finite(since)
            or facts.ts - float(since) < cfg.ident_settle_s - _EPS  # type: ignore[arg-type]
        ):
            reasons.append(f"settle:{zone}")
    reasons.extend(_bay_reasons(cfg, facts, zones))
    for reason in envelope_violations(cfg, facts, zones, cfg.ident_start_band_c):
        reasons.append(
            reason.replace("envelope:", "start_band:", 1)
            if reason.startswith("envelope:")
            else reason
        )
    return reasons


def _bay_reasons(cfg: MpcConfig, facts: TickFacts, zones: tuple[str, ...]) -> list[str]:
    topo = cfg.topology
    if topo is None:
        return []
    settle = 0.0 if cfg.estimator is None else cfg.estimator.bay_settle_s
    wanted = set(zones)
    reasons: list[str] = []
    for bay, spec in topo.bays.items():
        if spec.zone not in wanted:
            continue
        info = facts.bays.get(bay)
        if not isinstance(info, Mapping):
            reasons.append(f"bay_unknown:{bay}")
            continue
        if info.get("occupancy") == UNKNOWN or info.get("occupancy") is None:
            reasons.append(f"bay_unknown:{bay}")
        since = info.get("since_ts")
        pending = info.get("pending_empty_s", 0.0), info.get("pending_occupied_ticks", 0)
        if any(not _finite(v) or float(v) != 0.0 for v in pending) or (
            since is not None
            and (not _finite(since) or facts.ts is None or facts.ts - float(since) < settle)
        ):
            reasons.append(f"bay_transition:{bay}")
        cal = info.get("calibration")
        if isinstance(cal, Mapping):
            samples = cal.get("samples")
            if not _finite(samples) or float(samples) < CAL_MIN_SAMPLES:  # type: ignore[arg-type]
                reasons.append(f"calibrating:{bay}")
    return reasons


# ---------------------------------------------------------------------------
# Sequence
# ---------------------------------------------------------------------------


def _lfsr_next(state: int) -> int:
    lsb = state & 1
    state >>= 1
    if lsb:
        state ^= LFSR_TAPS
    return state


def hold_sequence(cfg: MpcConfig, total_s: float) -> list[float]:
    """Hold times (s) drawn from ``ident_hold_s`` until they cover ``total_s``."""
    state = cfg.ident_seed % 0xFFFF + 1
    holds = list(cfg.ident_hold_s)
    out: list[float] = []
    covered = 0.0
    while covered < total_s:
        for _ in range(16):
            state = _lfsr_next(state)
        hold = holds[state % len(holds)]
        out.append(hold)
        covered += hold
    return out


def _schedule(cfg: MpcConfig, phases: list[tuple[str, ...]]) -> list[dict[str, Any]]:
    duration = cfg.ident_max_duration_s
    per_phase = duration / len(phases)
    holds = iter(hold_sequence(cfg, duration + len(phases) * max(cfg.ident_hold_s)))
    out: list[dict[str, Any]] = []
    for i, channels in enumerate(phases):
        start_s = i * per_phase
        end_s = duration if i == len(phases) - 1 else (i + 1) * per_phase
        segments: list[list[float]] = []
        t, level = start_s, LEVEL_HIGH
        while t < end_s - _EPS:
            segments.append([t, level])
            t += next(holds)
            level = LEVEL_LOW if level == LEVEL_HIGH else LEVEL_HIGH
        out.append(
            {"channels": list(channels), "start_s": start_s, "end_s": end_s, "segments": segments}
        )
    return out


def start(cfg: MpcConfig, facts: TickFacts, kind: str, name: str) -> dict[str, Any]:
    """A new experiment (plain JSON) armed for the tick after ``facts``.

    Call only when :func:`check_start` returned no reason."""
    channels = target_channels(cfg, kind, name)
    phases = _phases(cfg, kind, name)
    base = {ch: float(facts.pwm[ch]) for ch in channels}
    assert facts.ts is not None
    exp: dict[str, Any] = {
        "target": {"kind": kind, "name": name},
        "start_ts": facts.ts + cfg.dt,
        "last_ts": facts.ts,
        "duration_s": cfg.ident_max_duration_s,
        "channels": list(channels),
        "served_zones": list(served_zones(cfg, channels)),
        "base": base,
        "levels": {ch: list(_levels(cfg, base[ch])) for ch in channels},
        "phases": _schedule(cfg, phases),
        "apply_failures": 0,
    }
    exp.update(levels_at(exp, 0.0))
    return exp


def levels_at(exp: Mapping[str, Any], offset_s: float) -> dict[str, Any]:
    """``{"phase", "level", "overrides"}`` at ``offset_s`` into the experiment.

    Every channel of the target gets an override: the phase's channels their level,
    the other channels of the group their base."""
    phases = exp["phases"]
    index = len(phases) - 1
    for i, phase in enumerate(phases):
        if offset_s < phase["end_s"] - _EPS:
            index = i
            break
    phase = phases[index]
    level = LEVEL_HIGH
    for seg_t, seg_level in phase["segments"]:
        if seg_t <= offset_s + _EPS:
            level = int(seg_level)
    active = set(phase["channels"])
    overrides = {
        ch: float(exp["levels"][ch][level]) if ch in active else float(exp["base"][ch])
        for ch in exp["channels"]
    }
    return {"phase": index, "level": level, "overrides": overrides}


# ---------------------------------------------------------------------------
# Running
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class Advance:
    """Outcome of :func:`advance`: the experiment to keep (``None`` when it ended),
    ``result`` (``None`` while running, else :data:`RESULT_COMPLETED` or
    :data:`RESULT_ABORTED`) and the abort ``reason``."""

    experiment: dict[str, Any] | None
    result: str | None = None
    reason: str | None = None


def advance(exp: Mapping[str, Any], cfg: MpcConfig, facts: TickFacts) -> Advance:
    """Check the tick that just ran with ``exp``'s overrides and arm the next tick."""
    if facts.mode is None or facts.mode == "fallback":
        return Advance(None, RESULT_ABORTED, "fallback")
    if facts.mode == "degraded":
        return Advance(None, RESULT_ABORTED, "degraded")
    failures = 0 if facts.applied else int(exp.get("apply_failures", 0)) + 1
    if failures >= APPLY_FAILURES_ABORT:
        return Advance(None, RESULT_ABORTED, "apply_failed")
    ts = facts.ts
    if ts is None or ts < float(exp["last_ts"]):
        return Advance(None, RESULT_ABORTED, "clock")
    duration = float(exp["duration_s"])
    if ts - float(exp["start_ts"]) > duration + cfg.dt + _EPS:
        return Advance(None, RESULT_ABORTED, "duration")
    zones = tuple(exp["served_zones"])
    reasons: list[str] = []
    for zone in zones:
        info = facts.zones.get(zone)
        if not isinstance(info, Mapping) or info.get("trusted") is not True:
            reasons.append(f"zone_untrusted:{zone}")
    for bay in _constrained_bays(cfg, facts, zones):
        info = facts.bays.get(bay)
        if not isinstance(info, Mapping) or info.get("occupancy") in (UNKNOWN, None):
            reasons.append(f"bay_unknown:{bay}")
    reasons.extend(envelope_violations(cfg, facts, zones, cfg.ident_max_over_c))
    for ch in exp["channels"]:
        if facts.fan_stall.get(ch):
            reasons.append(f"fan_stall:{ch}")
    if reasons:
        return Advance(None, RESULT_ABORTED, reasons[0])
    offset = ts + cfg.dt - float(exp["start_ts"])
    if offset >= duration - _EPS:
        return Advance(None, RESULT_COMPLETED, None)
    out = dict(exp)
    out["last_ts"] = ts
    out["apply_failures"] = failures
    out.update(levels_at(exp, max(offset, 0.0)))
    return Advance(out)


def status(
    exp: Mapping[str, Any] | None, last: Mapping[str, Any] | None, cfg: MpcConfig
) -> dict[str, Any]:
    """``ControlSnapshot.extra["experiment"]`` (plain JSON).

    ``running``, ``target`` (``{kind, name}``), ``group`` (the target when it is a
    group), ``channel`` (the single channel of the current phase, else ``None``),
    ``channels`` of the current phase, ``phase`` / ``phases``, ``level``
    (``high`` | ``low``), ``overrides``, ``base``, ``elapsed_s``, ``remaining_s``,
    and from the last experiment that ended: ``last_result`` (``completed`` |
    ``aborted``), ``last_abort_reason`` and ``last_target``.
    """
    last = last or {}
    out: dict[str, Any] = {
        "enabled": cfg.ident_enabled,
        "running": exp is not None,
        "target": None,
        "group": None,
        "channel": None,
        "channels": [],
        "phase": None,
        "phases": 0,
        "level": None,
        "overrides": {},
        "base": {},
        "elapsed_s": None,
        "remaining_s": None,
        "last_result": last.get("result"),
        "last_abort_reason": last.get("reason"),
        "last_target": last.get("target"),
    }
    if exp is None:
        return out
    phase = exp["phases"][exp["phase"]]
    elapsed = max(0.0, float(exp["last_ts"]) + cfg.dt - float(exp["start_ts"]))
    out.update(
        {
            "target": dict(exp["target"]),
            "group": exp["target"]["name"] if exp["target"]["kind"] == "group" else None,
            "channel": phase["channels"][0] if len(phase["channels"]) == 1 else None,
            "channels": list(phase["channels"]),
            "phase": int(exp["phase"]),
            "phases": len(exp["phases"]),
            "level": "high" if exp["level"] == LEVEL_HIGH else "low",
            "overrides": dict(exp["overrides"]),
            "base": dict(exp["base"]),
            "elapsed_s": elapsed,
            "remaining_s": max(0.0, float(exp["duration_s"]) - elapsed),
        }
    )
    return out
