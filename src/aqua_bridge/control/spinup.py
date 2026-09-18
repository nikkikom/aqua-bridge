"""The spin-up kick: a fan that is commanded but does not turn (PROJECT.md section 8 item 75).

A fan's *starting* duty is higher than its running duty -- the owner's aquaero test
fan stops at 13 % and starts again only at 25 % -- so a channel can sit at a healthy
duty, on a healthy rail, with a healthy-looking output, while the rotor stands still.
Nothing in the status report says so: the duty read back is the duty written and the
rail reads 12 V. Only the tachometer knows, and the enclosure is silently uncooled on
that channel until someone notices. The owner met exactly that after a restart
(2026-09-17).

What this module does
---------------------
It is a *supervised sequence*, not a boost:

1. **Detect.** A channel whose commanded duty has stayed at or above its stall duty
   and whose tachometer has read at or below ``min_rpm`` for ``confirm_s`` of live,
   uninterrupted readings.
2. **Kick.** Raise that channel to its own ``kick_duty`` for ``kick_s``, then let go.
   The kick is a *floor*, never a level: it is composed as
   ``max(what the solver or a human asked for, kick_duty)``, so a solver that already
   wants more is untouched and the kick can never reduce cooling anywhere.
3. **Verify.** Watch the tachometer for ``verify_s``. Motion ends the sequence. No
   motion starts the next attempt, and after ``max_attempts`` the channel is a
   **failed fan**: a health problem with a clear message, and -- with
   ``failed_channel_floor`` -- a floor under every other channel of the zones it
   served, at the duty they carried when the failure was declared (never above
   ``failed_channel_floor_max``, and never on a sibling whose own fan is already
   declared dead), so the zone cannot lose cooling because one of its fans died. A
   failed channel gets one more sequence every ``retry_s``, so a fan replaced while the
   daemon runs is picked up without a restart; that retry runs *under* the floor, which
   is recorded once at the first failure and never re-recorded, so repeated retries on
   a hot enclosure cannot ratchet the siblings upward. The declaration -- the alarm and
   the floor -- is retracted only by ``clear_s`` of live readings above ``min_rpm``:
   only the tachometer lifts it, and not on one sample, because a dead rotor windmilled
   by its siblings' airflow reads a handful of rpm.
4. **At start too.** Every commanded channel is under the rule from the first tick;
   the same confirmation window covers the seconds a healthy fan needs to spin up.

Why the kick duty is per channel
---------------------------------
Measured on the owner's hardware, 2026-09-18: the same fan model reads **174 rpm on an
aquaero output and 255 rpm on an output behind the aquabus device at the same nominal
20 %** (244 against 311 at 25 %). The duty-to-rpm mapping belongs to the *output*, not
to the fan model, and the difference is largest exactly at the low duties where
starting happens. One global kick would be wasteful noise on the stronger output and
too weak on the other -- and the one that failed to start is the weaker one, the
aquaero's own. So ``spin_up.channels.<ch>.kick_duty`` is per output, with
``spin_up.kick_duty`` as the documented fallback for a channel that has none.

What is never kicked, and never alarmed
----------------------------------------
Three things look alike in a status report and must not: an output with no fan on it,
an output whose fan has no tachometer, and a fan that should be turning and is not.
The daemon tells them apart from the configuration, never from a guess:

* ``spin_up.channels.<ch>.fan: false`` -- nothing hangs on this output. No rule, no
  kick, no alarm; the verdict says so.
* ``spin_up.channels.<ch>.tachometer: false`` -- a fan with no tach wire (the second
  fan of a splitter drives no tachometer, and a config may leave an output's ``rpm:``
  unbound). Nothing measures whether it turns, so nothing may claim it does: no rule,
  no kick, no alarm, and the verdict says so. Watching its *current* instead is
  PROJECT.md section 8 (the proposed item of this work), not this rule.
* a channel with no tachometer reading in the observation at all (no ``rpm:`` in the
  ``aquacomputer:`` binding) -- the same answer, with the binding named as the reason.
* a channel with no fitted curve (``mpc.fans`` / ``mpc.fan_models``, which need
  ``mpc.topology``) -- without a curve nothing here can say what "implausibly low"
  means for that output, so the rule is off and says so. A legacy config therefore
  gets no kicks at all and its command path is bit-identical.

Where it sits
-------------
The tracker is the supervisor's (:class:`~aqua_bridge.control.supervisor.Supervisor`):
:meth:`Supervisor.record_tick` advances it on the tick's facts and
:meth:`Supervisor.plan_tick` puts the floors in force into
:class:`~aqua_bridge.control.supervisor.TickPlan`, where ``compose`` applies them
through the same rate limit (``d_pwm_max``) and clamp (``[pwm_min, pwm_max]``) as any
other command. ``mpc.step`` is not touched at all: it stays pure and deterministic and
the legacy path is unchanged. Every function here is pure and clock-free -- the clock
is the observation's ``ts``, like everywhere else in the controller.

A tick with no live reading (the read failed, the write failed, the channel's aquabus
slot went away) is a **gap**: every window of that channel starts again, because wall
time passing while nothing was measured is not evidence of anything. That is the same
rule the fan-health monitor follows (:mod:`aqua_bridge.health`). A gap is equally not
evidence that a dead fan came back: a declared failure, and the floor under its zone,
survives one.

Every threshold and timing is a ``spin_up:`` key with one default declared once in
:class:`SpinUpConfig`, validated there and in :func:`validate_spin_up` (which needs the
controller config), shown in both example configs and described in PROJECT.md
section 3.
"""

from __future__ import annotations

import dataclasses
import math
from collections.abc import Collection, Mapping
from dataclasses import dataclass
from typing import Any

from aqua_bridge.health import expected_rpm
from aqua_bridge.model import ConfigError, MpcConfig

__all__ = [
    "SPINUP_CHANNEL_KEYS",
    "SPINUP_KEYS",
    "STATES",
    "Floors",
    "SpinUpChannel",
    "SpinUpConfig",
    "TickFacts",
    "advance",
    "channel_kick_duty",
    "channel_stall_duty",
    "covers_any",
    "floors",
    "new_tracker",
    "status",
    "validate_spin_up",
]

#: The rule does not run for this channel; ``reason`` says why (coverage, not a verdict).
OFF = "off"
#: Judged region not entered this tick: the duty is below the stall duty, or the tick
#: carried no live reading for this channel.
IDLE = "idle"
#: Judged and turning.
TURNING = "turning"
#: Judged, not turning, accumulating ``confirm_s``.
CONFIRMING = "confirming"
#: A kick is in force on this channel.
KICKING = "kicking"
#: The kick is over; watching the tachometer for ``verify_s``.
VERIFYING = "verifying"
#: ``max_attempts`` kicks went out and the fan never turned.
FAILED = "failed"

#: Every state a channel's verdict can carry, in the order they occur.
STATES: tuple[str, ...] = (OFF, IDLE, TURNING, CONFIRMING, KICKING, VERIFYING, FAILED)

_DISABLED = "spin_up.enabled is false, so no spin-up rule runs for any channel"
_NO_FAN = "spin_up.channels.<ch>.fan is false: nothing hangs on this output"
_NO_TACH = (
    "spin_up.channels.<ch>.tachometer is false: this fan drives no tachometer, so nothing "
    "measures whether it turns and no kick may be aimed at it"
)
_NO_BINDING = (
    "this channel reports no speed at all -- no tachometer is bound to it (an "
    "`aquacomputer:` fans entry without `rpm:`), so nothing measures whether it turns"
)
_NO_CURVE = (
    "no fan model with an `rpm_max` is configured for this channel in `mpc.fans` / "
    "`mpc.fan_models` (which need `mpc.topology`), so nothing here can say what speed "
    "this output should reach and the spin-up rule is off for it"
)


def _number(name: str, value: Any, *, minimum: float | None, maximum: float | None) -> float:
    if isinstance(value, bool) or not isinstance(value, int | float):
        raise ConfigError(f"{name} must be a number, got {value!r}")
    out = float(value)
    if not math.isfinite(out):
        raise ConfigError(f"{name} must be finite, got {value!r}")
    if minimum is not None and out < minimum:
        raise ConfigError(f"{name} must be >= {minimum:g}, got {out:g}")
    if maximum is not None and out > maximum:
        raise ConfigError(f"{name} must be <= {maximum:g}, got {out:g}")
    return out


def _flag(name: str, value: Any) -> bool:
    if not isinstance(value, bool):
        raise ConfigError(f"{name} must be true or false, got {value!r}")
    return value


@dataclass(frozen=True, kw_only=True)
class SpinUpChannel:
    """``spin_up.channels.<channel>``: what this one output is, and what it needs.

    ``kick_duty`` and ``stall_duty`` default to the section's own values
    (``None`` here means "the section's"); ``fan`` and ``tachometer`` declare what is
    physically on the output, which no status report can tell the daemon.
    """

    fan: bool = True
    tachometer: bool = True
    stall_duty: float | None = None
    kick_duty: float | None = None

    @classmethod
    def coerce(cls, path: str, data: Any) -> SpinUpChannel:
        if isinstance(data, SpinUpChannel):
            return data
        if not isinstance(data, Mapping):
            raise ConfigError(
                f"{path} must be a mapping like {{kick_duty: 0.5}}, got {type(data).__name__}"
            )
        unknown = sorted(str(k) for k in data if k not in SPINUP_CHANNEL_KEYS)
        if unknown:
            raise ConfigError(
                f"{path}: unknown key(s) {unknown}; allowed: {list(SPINUP_CHANNEL_KEYS)}"
            )
        stall = data.get("stall_duty")
        kick = data.get("kick_duty")
        return cls(
            fan=_flag(f"{path}.fan", data.get("fan", True)),
            tachometer=_flag(f"{path}.tachometer", data.get("tachometer", True)),
            stall_duty=(
                None
                if stall is None
                else _number(f"{path}.stall_duty", stall, minimum=0.0, maximum=1.0)
            ),
            kick_duty=(
                None
                if kick is None
                else _number(f"{path}.kick_duty", kick, minimum=0.0, maximum=1.0)
            ),
        )

    def to_dict(self) -> dict[str, Any]:
        return {
            "fan": self.fan,
            "tachometer": self.tachometer,
            "stall_duty": self.stall_duty,
            "kick_duty": self.kick_duty,
        }


SPINUP_CHANNEL_KEYS: tuple[str, ...] = tuple(f.name for f in dataclasses.fields(SpinUpChannel))


@dataclass(frozen=True, kw_only=True)
class SpinUpConfig:
    """The ``spin_up:`` section: every threshold and timing of the kick, with its default.

    The defaults are the owner's enclosure read conservatively: a fan that has stood
    still for half a minute under a duty it should be turning at is not a slow fan, and
    a kick to half duty for half a minute is far above the 25 % start duty measured on
    the owner's test fan while costing a few seconds of audible fan on a rare event.
    Narrow them once ``tools/fit_fans.py`` has measured the start duty of each output
    (PROJECT.md section 8 item 75).
    """

    #: Run the rule at all. False: nothing is detected, nothing is kicked, and every
    #: channel's verdict says the rule is off.
    enabled: bool = True
    #: A channel commanded below this output duty may legitimately stand still, so it is
    #: not judged (0..1). The default sits above the 13 % at which the owner's test fan
    #: stops. Per channel: ``channels.<ch>.stall_duty``.
    stall_duty: float = 0.15
    #: The duty a kick raises a channel to (0..1), for a channel with no
    #: ``channels.<ch>.kick_duty`` of its own. Twice the 25 % start duty measured on the
    #: owner's aquaero output: the margin an uncharacterised output gets. Clamped into
    #: ``[mpc.pwm_min, mpc.pwm_max]`` when it is applied.
    kick_duty: float = 0.5
    #: How long one kick holds the channel at its kick duty, seconds (> 0). It must
    #: cover the ``mpc.d_pwm_max`` ramp up to that duty as well as the rotor's own start
    #: time, which :func:`validate_spin_up` checks against ``mpc.dt``.
    kick_s: float = 30.0
    #: A channel must read at or below ``min_rpm`` under a duty at or above its stall
    #: duty for this many seconds of live, uninterrupted readings before it is kicked
    #: (> 0). It is also what covers the seconds a healthy fan needs to spin up.
    confirm_s: float = 30.0
    #: After a kick, how long the tachometer is watched before the next attempt,
    #: seconds (> 0).
    verify_s: float = 30.0
    #: Kicks one channel gets before it is declared a failed fan (>= 1).
    max_attempts: int = 3
    #: A speed at or below this is "not turning", rpm (>= 0). Not zero: a tachometer
    #: reading a handful of rpm is a rotor that is not moving air either.
    min_rpm: float = 60.0
    #: A failed channel gets one more sequence after this long, seconds (> 0), so a fan
    #: replaced while the daemon runs is picked up without a restart -- and a truly dead
    #: one costs at most ``max_attempts`` kicks per this interval instead of for ever.
    retry_s: float = 900.0
    #: A declared failure is retracted only once the tachometer has read above
    #: ``min_rpm`` for this long of live, uninterrupted readings (> 0). Declaring one
    #: costs ``confirm_s`` and every attempt, so retracting it -- the alarm and the floor
    #: under the zone's other fans -- may not cost one reading: a dead rotor windmilled
    #: by its siblings' airflow reads a handful of rpm.
    clear_s: float = 30.0
    #: While a fan is declared failed, hold every other channel of the zones it served
    #: at the duty it carried when the failure was declared, so the zone cannot lose
    #: cooling because one of its fans died. The rise above that floor comes from the
    #: measured temperatures, through the solver, like any other heat.
    failed_channel_floor: bool = True
    #: No such hold is ever recorded above this duty (0..1). A failure declared in the
    #: middle of a hot spell would otherwise pin the enclosure at that duty for as long
    #: as the fan stays dead; above the cap the temperatures hold the siblings up on
    #: their own, through the solver, exactly as they did before the failure.
    failed_channel_floor_max: float = 0.6
    #: One log line per channel and transition at most this often, seconds (> 0).
    log_interval_s: float = 300.0
    #: Per output: what is on it and what it needs (:class:`SpinUpChannel`).
    channels: dict[str, SpinUpChannel] = dataclasses.field(default_factory=dict)

    def __post_init__(self) -> None:
        _flag("spin_up.enabled", self.enabled)
        _flag("spin_up.failed_channel_floor", self.failed_channel_floor)
        stall = _number("spin_up.stall_duty", self.stall_duty, minimum=0.0, maximum=1.0)
        kick = _number("spin_up.kick_duty", self.kick_duty, minimum=0.0, maximum=1.0)
        if kick <= stall:
            raise ConfigError(
                f"spin_up.kick_duty ({kick:g}) must be above spin_up.stall_duty ({stall:g}): "
                "a kick at or below the duty a fan already fails to start at is not a kick"
            )
        _number("spin_up.min_rpm", self.min_rpm, minimum=0.0, maximum=None)
        _number(
            "spin_up.failed_channel_floor_max",
            self.failed_channel_floor_max,
            minimum=0.0,
            maximum=1.0,
        )
        for name in ("kick_s", "confirm_s", "verify_s", "retry_s", "clear_s", "log_interval_s"):
            _number(f"spin_up.{name}", getattr(self, name), minimum=1e-9, maximum=None)
        if isinstance(self.max_attempts, bool) or not isinstance(self.max_attempts, int):
            raise ConfigError(f"spin_up.max_attempts must be an integer, got {self.max_attempts!r}")
        if self.max_attempts < 1:
            raise ConfigError(f"spin_up.max_attempts must be >= 1, got {self.max_attempts}")
        if not isinstance(self.channels, Mapping):
            raise ConfigError(
                f"spin_up.channels must be a mapping, got {type(self.channels).__name__}"
            )
        coerced = {
            str(name): SpinUpChannel.coerce(f"spin_up.channels.{name}", entry)
            for name, entry in self.channels.items()
        }
        object.__setattr__(self, "channels", coerced)
        for name, entry in coerced.items():
            where = f"spin_up.channels.{name}"
            ch_stall = stall if entry.stall_duty is None else entry.stall_duty
            ch_kick = kick if entry.kick_duty is None else entry.kick_duty
            if ch_kick <= ch_stall:
                raise ConfigError(
                    f"{where}.kick_duty ({ch_kick:g}) must be above its stall duty "
                    f"({ch_stall:g}): a kick at or below the duty this fan already fails to "
                    "start at is not a kick"
                )

    @classmethod
    def from_section(cls, section: Mapping[str, Any] | None) -> SpinUpConfig:
        """Build from the raw ``spin_up:`` section; :class:`ConfigError` for an unknown
        key or a bad value, so a misspelt key cannot silently keep its default (the rule
        every other section follows)."""
        raw = dict(section or {})
        unknown = sorted(str(k) for k in raw if k not in SPINUP_KEYS)
        if unknown:
            raise ConfigError(f"spin_up: unknown key(s) {unknown}; allowed: {list(SPINUP_KEYS)}")
        return cls(**raw)


SPINUP_KEYS: tuple[str, ...] = tuple(f.name for f in dataclasses.fields(SpinUpConfig))


def channel_stall_duty(settings: SpinUpConfig, channel: str) -> float:
    """The duty at or above which ``channel`` is judged (its own, else the section's)."""
    entry = settings.channels.get(channel)
    if entry is None or entry.stall_duty is None:
        return float(settings.stall_duty)
    return float(entry.stall_duty)


def channel_kick_duty(cfg: MpcConfig, settings: SpinUpConfig, channel: str) -> float:
    """The duty a kick raises ``channel`` to, clamped into ``[pwm_min, pwm_max]``.

    Per output, because the duty-to-rpm mapping is the output's, not the fan model's
    (module docstring); ``spin_up.kick_duty`` is the documented fallback."""
    entry = settings.channels.get(channel)
    want = settings.kick_duty if entry is None or entry.kick_duty is None else entry.kick_duty
    return min(cfg.pwm_max, max(cfg.pwm_min, float(want)))


def validate_spin_up(cfg: MpcConfig, settings: SpinUpConfig) -> None:
    """The checks that need the controller config; :class:`ConfigError` naming the key.

    Run at startup before anything is opened, so a kick the daemon could never deliver
    is a configuration error (exit 2) rather than a rule that silently never fires.
    """
    unknown = sorted(set(settings.channels) - set(cfg.channels))
    if unknown:
        raise ConfigError(
            f"spin_up.channels names {unknown}, which are not in mpc.channels {list(cfg.channels)}"
        )
    if not settings.enabled:
        return
    # The worst case a kick has to deliver: the highest kick duty of any channel,
    # reached from pwm_min one d_pwm_max step per tick, plus the one tick that actually
    # holds the fan there. A kick_s shorter than that is not a rule that silently never
    # fires -- the channel is kicked to a duty it never reaches, and after max_attempts
    # the fan is declared failed, alarmed and its siblings floored for a kick it never
    # got. So it is a configuration error, refused before anything is opened.
    top = max(
        [channel_kick_duty(cfg, settings, ch) for ch in cfg.channels]
        or [min(cfg.pwm_max, max(cfg.pwm_min, float(settings.kick_duty)))]
    )
    ramp_s = math.ceil(max(0.0, top - cfg.pwm_min) / cfg.d_pwm_max) * cfg.dt
    if settings.kick_s < cfg.dt:
        raise ConfigError(
            f"spin_up.kick_s ({settings.kick_s:g} s) is shorter than one tick "
            f"(mpc.dt = {cfg.dt:g} s), so no kick would ever reach the fans; it also has to "
            f"cover the mpc.d_pwm_max ramp up to the kick duty ({ramp_s:g} s here)"
        )
    if settings.kick_s < ramp_s + cfg.dt:
        raise ConfigError(
            f"spin_up.kick_s ({settings.kick_s:g} s) cannot deliver a kick to {top:g}: the "
            f"mpc.d_pwm_max ({cfg.d_pwm_max:g} per tick) ramp from mpc.pwm_min "
            f"({cfg.pwm_min:g}) up to it takes {ramp_s:g} s, so the kick must last at least "
            f"{ramp_s + cfg.dt:g} s (that ramp plus one tick at the duty). As it stands the "
            "fan would be declared failed for a kick it never got"
        )


def covers_any(cfg: MpcConfig, settings: SpinUpConfig) -> bool:
    """True when the rule can produce a verdict other than "off" on some channel.

    A legacy config (no fitted curve on any output), a section that is switched off, or
    one that declares every output fan-less or tach-less can never kick anything and can
    never say anything but "off" -- so ``aqua_bridge.__main__.build_health_monitor``
    does not build an observer for this rule alone in that case, and an operator who
    turned the health sections off keeps them off.
    """
    return settings.enabled and any(
        _coverage(cfg, settings, channel) is None for channel in cfg.channels
    )


# ---------------------------------------------------------------------------
# The tracker
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class TickFacts:
    """What one tick tells the rule (all of it from the loop's own report).

    ``ts`` is the observation clock, ``duty`` what the composed command put on each
    channel, ``rpm`` the observation's tachometers. ``live`` is false for a tick whose
    read or whose write failed: nothing was measured, so every window starts again.
    """

    ts: float | None
    duty: Mapping[str, Any]
    rpm: Mapping[str, Any]
    live: bool


def new_tracker() -> dict[str, Any]:
    """A tracker with nothing seen yet."""
    return {"channels": {}, "failed": {}}


def _finite(value: Any) -> bool:
    return isinstance(value, int | float) and not isinstance(value, bool) and math.isfinite(value)


def _coverage(cfg: MpcConfig, settings: SpinUpConfig, channel: str) -> str | None:
    """Why the rule is off for ``channel`` by configuration alone, or ``None``.

    Only reasons that do not move from tick to tick: what is on the output and what the
    controller config says about it. "No tachometer is bound to this channel" is the one
    coverage answer that is read off the observation instead, and it can only be read off
    a tick that carried one, so :func:`advance` decides it -- a tick that measured
    nothing says nothing about a binding."""
    if not settings.enabled:
        return _DISABLED
    entry = settings.channels.get(channel)
    if entry is not None and not entry.fan:
        return _NO_FAN
    if entry is not None and not entry.tachometer:
        return _NO_TACH
    if expected_rpm(cfg, channel, 1.0) is None:
        return _NO_CURVE
    return None


def _record(state: str, **kw: Any) -> dict[str, Any]:
    out: dict[str, Any] = {
        "state": state,
        "since": None,
        "attempts": 0,
        "kick_until": None,
        "reason": None,
        "rpm": None,
        "duty": None,
        #: The fan is declared failed: the alarm and the sibling floor stand. It
        #: survives a gap and the whole of a ``retry_s`` retry, and only ``clear_s`` of
        #: live readings above ``min_rpm`` retracts it.
        "declared": False,
        #: When that declaration was first made (never re-recorded while it stands).
        "failed_since": None,
        #: When the wait for the next retry started (the declaration, or the last retry).
        "retry_since": None,
        #: Since when a declared channel's tachometer has been reading above ``min_rpm``.
        "turning_since": None,
    }
    out.update(kw)
    return out


def _siblings(cfg: MpcConfig, channel: str) -> tuple[str, ...]:
    """Every other channel that moves air through a zone this one served.

    ``ZoneLayout.served`` is the declared topology, never identified numbers; in legacy
    mode there is one implicit zone holding every channel, which is the conservative
    reading of "the same air path"."""
    layout = cfg.zone_layout
    zones = layout.served.get(channel) or layout.channel_zones.get(channel) or ()
    out: list[str] = []
    for zone in zones:
        for ch in layout.zone_channels.get(zone, ()):
            if ch != channel and ch not in out:
                out.append(ch)
    return tuple(out)


def advance(
    tracker: Mapping[str, Any],
    cfg: MpcConfig,
    settings: SpinUpConfig,
    facts: TickFacts,
) -> dict[str, Any]:
    """The tracker after one tick. Pure: a function of its arguments only.

    The state machine is the module docstring's sequence, per channel::

        idle -> confirming --confirm_s--> kicking --kick_s--> verifying --verify_s-->
                the next kick, or failed once max_attempts kicks have gone out

    A tachometer above ``min_rpm`` ends a sequence that has declared nothing. A
    *declared* failure is retracted only by ``clear_s`` of live readings above
    ``min_rpm``: declaring one costs a confirmation window and every attempt, so
    retracting it -- the alarm and the floor under the zone's remaining fans -- may not
    cost one sample, because a dead rotor windmilled by its siblings' airflow reads a
    handful of rpm. A gap (no live reading) drops every timer and ends any kick in
    force, but keeps the attempt count and the declaration: a tick that measured nothing
    is not evidence that a fan came back. A duty that falls below the channel's stall
    duty ends an undeclared sequence instead -- the fan is no longer expected to turn --
    and the next judged tick starts a fresh one.
    """
    old = tracker.get("channels") or {}
    old_failed = tracker.get("failed") or {}
    ts = facts.ts if _finite(facts.ts) else None
    channels: dict[str, Any] = {}
    failed: dict[str, Any] = {}
    # Channels whose own fan the daemon has already declared dead. A sibling's failure
    # does not floor them: a rotor that does not turn moves no air at any duty, so
    # pinning it high is noise and nothing else.
    dead = {ch for ch, record in old.items() if record.get("declared")}
    for channel in cfg.channels:
        prev = old.get(channel)
        # The floor a failed fan left under its siblings, carried for as long as the
        # declaration stands -- through a gap, and through the whole of a ``retry_s``
        # retry, which runs *under* it. It is recorded once, at the first failure, and
        # only the tachometer lifts it (``clear_s`` of it).
        held = old_failed.get(channel)
        reason = _coverage(cfg, settings, channel)
        if reason is not None:
            # A config that says there is no fan here, or no tachometer, also withdraws
            # the claim that one died: the floor goes with it.
            channels[channel] = _record(OFF, reason=reason)
            continue
        measured = facts.live and ts is not None
        unbound = channel not in facts.rpm
        if (measured and unbound) or (
            not measured and prev is not None and prev["reason"] == _NO_BINDING
        ):
            # No tachometer is bound to this output at all: nothing measures whether it
            # turns, so nothing may claim it does. A tick that measured nothing cannot
            # say this, so it keeps the answer the last measuring tick gave.
            channels[channel] = _record(OFF, reason=_NO_BINDING)
            continue
        duty = facts.duty.get(channel)
        rpm = facts.rpm.get(channel)
        live = measured and not unbound and _finite(duty) and _finite(rpm)
        attempts = int(prev["attempts"]) if prev else 0
        declared = bool(prev and prev["declared"])
        if declared and held is not None:
            failed[channel] = held

        if not live:
            # Nothing was measured, so nothing here was established by live data --
            # including that a declared fan came back.
            if declared:
                assert prev is not None
                channels[channel] = _keep_failed(prev)
            else:
                channels[channel] = _record(
                    IDLE,
                    attempts=attempts,
                    reason="no live reading on this tick, so every window starts again",
                )
            continue

        assert ts is not None
        duty, rpm = float(duty), float(rpm)  # type: ignore[arg-type]
        common: dict[str, Any] = {"rpm": rpm, "duty": duty}
        if rpm > settings.min_rpm:
            if not declared:  # it turns: the sequence is over, whatever it was
                channels[channel] = _record(TURNING, since=ts, **common)
                continue
            assert prev is not None
            started = float(prev["turning_since"]) if _finite(prev["turning_since"]) else ts
            if ts - started >= settings.clear_s:
                channels[channel] = _record(TURNING, since=ts, **common)
                failed.pop(channel, None)  # the tachometer, and only it, lifts the floor
            else:
                channels[channel] = _keep_failed(
                    prev,
                    turning_since=started,
                    reason=_recovering(channel, settings, rpm),
                    **common,
                )
            continue

        stall = channel_stall_duty(settings, channel)
        target = expected_rpm(cfg, channel, duty)
        # Judged only where the curve says the fan should clearly be turning: inside and
        # just above a dead band a standing rotor is normal and a kick would be noise.
        judged = duty >= stall and (target is None or target > settings.min_rpm)

        if declared:
            assert prev is not None
            if prev["state"] not in (CONFIRMING, KICKING, VERIFYING):
                # Waiting out ``retry_s``: a fan may have been replaced while the daemon
                # ran, so it gets one more sequence -- under the floor its failure left.
                since_retry = prev["retry_since"]
                due = _finite(since_retry) and ts - float(since_retry) >= settings.retry_s
                if judged and due:
                    channels[channel] = _record(
                        CONFIRMING,
                        since=ts,
                        declared=True,
                        failed_since=prev["failed_since"],
                        retry_since=ts,
                        **common,
                    )
                else:
                    channels[channel] = _keep_failed(prev, **common)
                continue
            if not judged:  # a retry cannot run under this duty: back to waiting
                channels[channel] = _keep_failed(prev, **common)
                continue

        # What a declared failure carries through the sequence of its own retry: the
        # declaration itself, the moment it was first made, and the retry clock.
        carry: dict[str, Any] = {
            "declared": declared,
            "failed_since": prev["failed_since"] if declared else None,
            "retry_since": prev["retry_since"] if declared else None,
        }

        if not judged:
            channels[channel] = _record(IDLE, **common)
            continue

        state = prev["state"] if prev else IDLE
        since = float(prev["since"]) if prev and _finite(prev["since"]) else ts

        if state == KICKING:
            if _finite(prev["kick_until"]) and ts < float(prev["kick_until"]):  # type: ignore[index]
                channels[channel] = _record(
                    KICKING,
                    since=since,
                    attempts=attempts,
                    kick_until=prev["kick_until"],  # type: ignore[index]
                    reason=prev["reason"],  # type: ignore[index]
                    **carry,
                    **common,
                )
            else:
                channels[channel] = _record(
                    VERIFYING, since=ts, attempts=attempts, **carry, **common
                )
            continue

        if state == VERIFYING:
            if ts - since < settings.verify_s:
                channels[channel] = _record(
                    VERIFYING, since=since, attempts=attempts, **carry, **common
                )
            elif attempts >= settings.max_attempts:
                channels[channel] = _fail(channel, ts, attempts, carry=carry, **common)
                failed[channel] = (
                    held if held is not None else _hold(cfg, settings, channel, facts, dead=dead)
                )
                dead.add(channel)
            else:
                channels[channel] = _kick(
                    cfg, settings, channel, ts, attempts, carry=carry, **common
                )
            continue

        # idle / turning / confirming: the confirmation window
        if state != CONFIRMING:
            since = ts
        if ts - since < settings.confirm_s:
            channels[channel] = _record(
                CONFIRMING, since=since, attempts=attempts, **carry, **common
            )
        elif attempts >= settings.max_attempts:
            channels[channel] = _fail(channel, ts, attempts, carry=carry, **common)
            failed[channel] = (
                held if held is not None else _hold(cfg, settings, channel, facts, dead=dead)
            )
            dead.add(channel)
        else:
            channels[channel] = _kick(cfg, settings, channel, ts, attempts, carry=carry, **common)
    return {"channels": channels, "failed": failed}


def _keep_failed(
    prev: Mapping[str, Any],
    *,
    turning_since: float | None = None,
    reason: str | None = None,
    **common: Any,
) -> dict[str, Any]:
    """A channel whose fan is declared failed stays declared, with its message.

    The state falls back to ``failed`` whatever the sequence was doing -- a gap or a
    duty below the stall duty ends a retry -- because the declaration, the alarm and the
    floor under the zone's other fans stand until ``clear_s`` of the tachometer retracts
    them."""
    return _record(
        FAILED,
        since=prev["since"],
        attempts=prev["attempts"],
        declared=True,
        failed_since=prev["failed_since"],
        retry_since=prev["retry_since"],
        turning_since=turning_since,
        reason=prev["reason"] if reason is None else reason,
        **common,
    )


def _recovering(channel: str, settings: SpinUpConfig, rpm: float) -> str:
    """The message of a declared fan whose tachometer has started reading again."""
    return (
        f"{channel}: the tachometer reads {rpm:.0f} rpm again, but the fan stays declared "
        f"failed -- and its siblings floored -- until it has kept turning for "
        f"spin_up.clear_s ({settings.clear_s:g} s). Declaring a fan dead costs a "
        "confirmation window and every attempt, so retracting it may not cost one reading: "
        "a dead rotor windmilled by the air of its siblings reads a handful of rpm"
    )


def _since(prev: Mapping[str, Any] | None, state: str, ts: float) -> float:
    if prev is not None and prev["state"] == state and _finite(prev["since"]):
        return float(prev["since"])
    return ts


def _kick(
    cfg: MpcConfig,
    settings: SpinUpConfig,
    channel: str,
    ts: float,
    attempts: int,
    *,
    carry: Mapping[str, Any] | None = None,
    **common: Any,
) -> dict[str, Any]:
    duty = channel_kick_duty(cfg, settings, channel)
    return _record(
        KICKING,
        since=ts,
        attempts=attempts + 1,
        kick_until=ts + settings.kick_s,
        **(dict(carry) if carry else {}),
        reason=(
            f"{channel}: commanded above its stall duty with the tachometer at or below "
            f"{settings.min_rpm:g} rpm; kicking to {duty * 100:.0f} % for "
            f"{settings.kick_s:g} s (attempt {attempts + 1} of {settings.max_attempts})"
        ),
        **common,
    )


def _fail(
    channel: str,
    ts: float,
    attempts: int,
    *,
    carry: Mapping[str, Any] | None = None,
    **common: Any,
) -> dict[str, Any]:
    """The declaration. A retry that fails again keeps the *first* failure's moment, so
    the floor it recorded is never re-recorded and the message keeps its age."""
    duty = common.get("duty")
    first = carry.get("failed_since") if carry else None
    return _record(
        FAILED,
        since=ts,
        attempts=attempts,
        declared=True,
        failed_since=float(first) if _finite(first) else ts,
        retry_since=ts,
        reason=(
            f"{channel}: the fan does not turn. {attempts} spin-up kick(s) went out and the "
            f"tachometer still reads {common.get('rpm', 0.0):.0f} rpm at "
            f"{0.0 if duty is None else duty * 100:.0f} % duty. Treat this output as having "
            "lost its airflow: check the fan, its splitter and its cable"
        ),
        **common,
    )


def _hold(
    cfg: MpcConfig,
    settings: SpinUpConfig,
    channel: str,
    facts: TickFacts,
    *,
    dead: Collection[str] = (),
) -> dict[str, float]:
    """The floor a failed channel leaves under its siblings: what they carried now.

    Empty with ``failed_channel_floor: false``. The floor never rises by itself and is
    gone once the failed fan has turned again for ``clear_s``; the *rise* above it comes
    from the measured temperatures through the solver, which is the only thing that
    knows how much more air the zone needs.

    Two bounds keep a hold from outliving its reason. A sibling whose own fan is already
    declared dead (``dead``) is not floored at all -- a rotor that does not turn moves no
    air at any duty, so pinning it high is noise and nothing else -- and no level is
    recorded above ``failed_channel_floor_max``, because a failure declared in the middle
    of a hot spell would otherwise pin the enclosure at that duty for as long as the fan
    stays dead. Above the cap the temperatures hold the siblings up on their own, through
    the solver, exactly as they did before the failure.
    """
    if not settings.failed_channel_floor:
        return {}
    out: dict[str, float] = {}
    cap = float(settings.failed_channel_floor_max)
    for sib in _siblings(cfg, channel):
        if sib in dead:
            continue
        value = facts.duty.get(sib)
        if _finite(value):
            out[sib] = min(float(value), cap)  # type: ignore[arg-type]
    return out


@dataclass(frozen=True)
class Floors:
    """The floors in force this tick: ``kick`` per kicking channel, ``hold`` per sibling
    of a failed one. Both are minima ``compose`` raises a channel to; neither ever
    lowers anything."""

    kick: dict[str, float]
    hold: dict[str, float]

    def combined(self) -> dict[str, float]:
        out = dict(self.hold)
        for ch, value in self.kick.items():
            out[ch] = max(out.get(ch, value), value)
        return out

    def __bool__(self) -> bool:
        return bool(self.kick or self.hold)

    def to_dict(self) -> dict[str, Any]:
        return {"kick": dict(self.kick), "hold": dict(self.hold)}


def floors(tracker: Mapping[str, Any], cfg: MpcConfig, settings: SpinUpConfig) -> Floors:
    """What the supervisor must not let any channel fall below this tick.

    A channel whose own fan is declared failed carries no *hold*, whoever recorded it:
    a rotor that does not turn moves no air at any duty, so holding it high is noise and
    nothing else. Its own retry's *kick* still raises it -- that is what tests whether
    the fan came back."""
    kick: dict[str, float] = {}
    dead: set[str] = set()
    for channel, record in (tracker.get("channels") or {}).items():
        if record["state"] == KICKING and channel in cfg.channels:
            kick[channel] = channel_kick_duty(cfg, settings, channel)
        if record["declared"]:
            dead.add(channel)
    hold: dict[str, float] = {}
    for levels in (tracker.get("failed") or {}).values():
        for ch, value in levels.items():
            if ch in cfg.channels and ch not in dead:
                hold[ch] = max(hold.get(ch, value), float(value))
    return Floors(kick=kick, hold=hold)


def status(tracker: Mapping[str, Any]) -> dict[str, Any]:
    """The published per-channel verdict: state, attempts, the last reading and, for a
    channel the rule does not run on, why.

    ``monitored`` is the same promise the fan-health verdict makes (PROJECT.md section 8
    item 117): true only where the rule actually ran on this channel, so a channel
    nothing watches can never read as a fan found healthy.

    ``failed`` is the *declaration*, not the state string: it stands through the ticks
    of a ``retry_s`` retry and through a gap, and goes away only when ``clear_s`` of the
    tachometer has retracted it -- so the operator's alarm is not withdrawn by one
    reading, or by the daemon merely trying again.
    """
    out: dict[str, Any] = {}
    for channel, record in (tracker.get("channels") or {}).items():
        out[channel] = {
            "state": record["state"],
            "monitored": record["state"] != OFF,
            # A flag, not a state string to compare: :mod:`aqua_bridge.health` publishes
            # this verdict and must not import this module (it is the other way round).
            "failed": bool(record["declared"]),
            "attempts": record["attempts"],
            "rpm": record["rpm"],
            "duty": record["duty"],
            "reason": record["reason"],
        }
    return out
