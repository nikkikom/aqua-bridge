"""aquaero / Quadro source and sink over hidraw (PROJECT.md section 3, Track B).

:class:`AquacomputerAdapter` is one controller: ``read() -> PlantObservation``,
``apply(MpcCommand)``, ``release()`` and the commissioning-only ``save()``,
the contract ``control/loop.py`` and
:class:`~aqua_bridge.hw.sources.CompositeSource` use. The Linux hwmon driver
is not involved: through hwmon every ``pwmN`` read cost 210 ms and every write
420 ms, about 5 s per tick with 8 outputs (PROJECT.md section 2). Here a read
drains the unsolicited status report (2 ms on the Pi) and a changed command is
one control report SET per controller (9-13 ms).

Reading
    Every ``read()`` drains the input reports and uses the newest status
    report: temperatures in degC (``None`` where nothing is connected), rpm,
    and ``obs.pwm`` from the status report's *output duty* -- what the device
    actually drives, not the cached command. A report's age is known to one
    read: it counts as received when the queue is drained. The kernel queue
    holds 63 reports and drops new ones while full, so a drain that returns a
    full queue proves nothing about the present: those reports are discarded
    (no status time, no duty evidence) and the queue is read once more. No
    status report for longer than ``status_max_age_s`` raises
    :class:`~aqua_bridge.hw.hidraw.DeviceUnavailable` (the loop's
    blank-observation fallback ramps the fans up). After an open, the first
    ``read()`` waits up to ``status_max_age_s`` for a report; later reads never
    block. A vanished node closes the device; the next call finds it again,
    possibly as a new ``hidrawN``. With ``serial:`` configured, a first status
    report carrying another serial closes the device and raises. A commanded
    aquabus output (aquaero 5-8) or a bound aquabus tachometer whose fan block
    reads rpm ``0xFFFF`` has no device behind it (nothing on the aquaero's
    aquabus): that channel's ``rpm`` and ``pwm`` are ``None`` in the
    observation, it is listed in ``absent_channels``, and one error is logged
    when that list changes (one info line when it empties) -- once per state
    change, not once per tick. Everything else the controller reports comes
    through unchanged, its temperatures included, and the write path is
    untouched: ``apply()`` writes an absent output with the others and does not
    raise for it. **Only a controller with no usable status report is
    unavailable** (a vanished node, the wrong serial, or nothing within
    ``status_max_age_s``): faulting the whole device for an empty slot would
    blind its healthy half -- the aquaero's own thermistors and outputs -- and
    put the loop in fallback for as long as the slot stays empty, which is what
    PROJECT.md section 8 item 90 is about. A fan the daemon cannot command is
    still safe through the plant: the zones it served lose cooling, their
    temperatures rise and the remaining fans ramp. An adapter whose *every*
    commanded output is absent says so in that one error line and is still not
    unavailable, since its temperatures are exactly what the solver needs to
    ramp the fans that are left. The check uses the newest status report alone,
    with no confirmation over time, and never covers the aquaero's own outputs
    1-4; a single transient ``0xFFFF`` costs that channel one tick of ``None``.

The device on aquabus
    Every one of those blocks reading ``0xFFFF`` at once means something the
    per-channel rule above does not say: **no device answers on the aquabus at
    all** (:func:`~aqua_bridge.hw.aquacomputer.aquabus_present`). That is what
    happened on 2026-09-15, when the Quadro left the bus with the controller
    running and nothing noticed for an hour (PROJECT.md section 8 item 92). Its
    outputs went absent -- item 90's business -- but its *temperature* slots
    ``bus1..8`` did something worse: they kept serving the last value they read,
    24.12 degC an hour later, with nothing in the report marking them old.

    So a logical name bound to one of those slots reads ``None`` in the
    observation for as long as no device answers, from the very first report that
    shows the bus empty. Unknown is a value; a frozen number that looks like a
    measurement is not, and this is the one reading the daemon has no other way to
    catch (the gate's Stuck rule needs some other channel to move first). The
    observation's other temperatures, the aquaero's own outputs and the whole
    write path are untouched, exactly as with item 90, and nothing here lowers a
    duty: a missing temperature makes its zone untrusted, which holds and then
    raises. A lost bus device is less evidence, not less heat.

    The *diagnosis* is slower than the safety on purpose. ``bus_absent_s`` (10 s
    by default) is how long every aquabus block must read ``0xFFFF``, over
    received reports, before the controller reports the loss: one error line
    naming what went missing, an entry in ``device_health``'s ``problems``, and
    ``bus_device`` carrying the state -- including whether a device ever answered
    since this adapter started reading, which is what separates "the Quadro left"
    from "nothing has ever been on this bus". What the window keeps out is a
    device re-enumerating: one transient ``0xFFFF`` (item 90) is a blip, not a
    departure. It is *not* bounded by the aquabus refresh interval of item 115:
    that interval moves the electrical fields only, while presence is read from
    the speed field, which every report carries, so a poll the controller skipped
    cannot look like a departure however short this window is.

    What this cannot see is a bus device with **no fan outputs** at all: presence
    is judged from the fan blocks, so such a device reads as an empty bus and its
    temperature slots would stay missing for ever. That is why binding a ``busN``
    needs an aquabus output of the same device bound in the same entry, which the
    config model enforces (:class:`DeviceBinding`) rather than leaving to a line
    in the documentation.

Writing
    ``apply()`` opens the node without waiting for a status report, so the
    fallback ramp and the stop write reach a device whose status reports have
    stopped. The control report is read (GET) once after opening and kept as a
    cache; a normal tick does no control read. A write is **one** control report
    SET carrying every channel with a pending change. It takes effect at once and
    is not saved: the controller's memory keeps the configuration last saved
    with the save report, which only :meth:`AquacomputerAdapter.save` sends
    (PROJECT.md section 8 items 84, 86). A channel is written when its duty
    rises (cooling never waits) or when it does not follow its duty at all; a
    fall is written only when it is at least ``write_deadband`` below the
    written duty and ``write_min_interval_s`` has passed since this device's
    last write (both 0 by default: every change is written; the keys only limit
    USB traffic). A command in FALLBACK or DEGRADED mode never takes a channel
    below the duty the device holds, in a forced rewrite too: a deferred fall
    that the loop counted as applied must not mature, or ride along, during a
    fault (the stop write is a FALLBACK command, so it never lowers a fan
    either). Before every GET or SET the adapter waits ``ctrl_gap_ms`` after
    the previous control operation, failed ones included. A failed operation
    invalidates the cache and is retried from a fresh GET up to
    ``ctrl_retries`` times, and a failed SET makes the next write send every
    configured channel (it may or may not have reached the device); no control
    operation starts once ``ctrl_budget_s`` of this call is spent. Either way
    ``DeviceUnavailable`` is raised.

The software-sensor heartbeat (hardware watchdog)
    With ``heartbeat_sensor`` set (0, the default, is off), every ``apply()``
    writes ``heartbeat_value_c`` into that aquaero software sensor with the HID
    output report ``0x07`` -- once per tick, **after** the duty work and only
    when it succeeded, so once per ``apply()`` however often the control write
    is retried. Only the configured sensor is written; the other seven slots
    carry ``0x7FFF`` ("no data"), so the device keeps its own values for them.
    The point is the controller's own watchdog: with that sensor enabled on the
    device, given a timeout and a high fallback temperature, and an alarm on it
    that selects a safe profile, a daemon (or Pi) that stops writing lets the
    sensor fall back and the controller take over -- verified on the owner's
    aquaero, which goes to 100 % on every output about 30 s after the last
    heartbeat and back on the next one (PROJECT.md section 8 items 33, 84).

    The heartbeat must therefore never outlive the writing it stands for.
    The failure the controller's watchdog uniquely covers is a daemon that is
    *alive but cannot write* -- every control operation failing on a live node
    (``DeviceUnavailable`` every tick, the loop logging the apply and carrying
    on) -- so the heartbeat goes out only behind a ``_apply_once`` that
    returned: a tick that could not command a duty sends none, the sensor runs
    down the device's timeout and the alarm takes over. ``_apply_once``
    returning with nothing to send (no duty changed) still counts, which is why
    the cadence is one per tick and not one per SET. What the heartbeat does
    *not* cover is a write that reaches the kernel but not the device: the
    output report is handed to hidraw, not acknowledged by the controller
    (:meth:`~aqua_bridge.hw.hidraw.HidrawTransport.write_report`), so
    ``heartbeat_ok`` means "accepted for sending", and only a soft sensor read
    back from the status report proves delivery (PROJECT.md section 8 item 93).

    The heartbeat is one device operation like any other: it waits
    ``ctrl_gap_ms`` and is inside ``ctrl_budget_s``, so the worst case per tick
    stays what ``worst_case_tick_s`` states. It never fails the tick -- a
    failed write is logged once per state change (error when it starts failing,
    info when it goes out again), and the cost of it not arriving is exactly
    the fallback the device is configured for, which is the safe direction.
    ``heartbeat_on`` and ``heartbeat_ok`` expose it.

The active profile
    Byte ``0x06`` of the aquaero's control report is the profile it runs
    (``active_profile``, 1-based). A profile switch -- the alarm above, or a
    press on the controller's own panel -- reloads the **saved** profile, so
    every duty the daemon wrote live is gone. Every fresh control report is
    therefore checked: a changed byte logs one line naming both profiles and
    makes the next write send every configured channel. No control read is
    added for it; the report is the one the periodic ``ctrl_refresh_s`` read
    (60 s by default) or an invalidation already fetches. So a switch is
    noticed within ``duty_mismatch_s`` plus a tick whenever the reloaded
    profile drives the outputs differently than the daemon last wrote (the duty
    verification sees the changed output duty and re-reads the report), and
    otherwise at the latest after ``ctrl_refresh_s``.

    Both edges matter, and byte ``0x06`` is what bounds neither of them. On the
    *alarm-set* edge the reloaded profile is the safe one (on the owner's
    controller every output at 100 %), so a late rewrite only postpones the
    daemon taking the fans back down. On the *alarm-clear* edge -- the one every
    daemon restart after an alarm goes through -- the reloaded profile is the
    quiet one (20 % there), which **reduces cooling** below what the daemon
    thinks it commands. What bounds that edge is the duty verification, not the
    profile check: the reloaded duty differs from the written one, so within
    ``duty_mismatch_s`` plus a tick the cache is invalidated, the report re-read,
    the profile change seen and every channel written again. The
    ``ctrl_refresh_s`` worst case is reached only when the reloaded profile
    happens to drive the outputs exactly as the daemon last wrote them, i.e.
    when there is nothing to lose by waiting.
Diagnostics
    Speed, duty, voltage, current and power arrive in every status report.
    :meth:`AquacomputerAdapter.fan_readings` hands them out per commanded channel
    and ``read()`` puts them in ``PlantObservation.inputs["fans"]``, so they are
    recorded and published without entering ``temps``/``rpm``/``pwm`` and so
    without changing what ``mpc.step`` sees (PROJECT.md section 8 item 79); the
    drift rules over them live in :mod:`aqua_bridge.health`.
    :meth:`AquacomputerAdapter.device_health` is the controller's own state --
    ``stuck_channels``, ``absent_channels``, the commanded outputs not in PWM
    mode, unconfigured blocks, the flow sensors and the ``aquabus`` state above
    -- on a path that survives a failed ``read()``, since that is exactly when it
    matters (item 83). The ``u16`` at ``+0x0A`` of a fan block is *not* there: it
    is unidentified (item 114), so it is decoded raw for
    ``tools/aquabus_watch.py`` and published nowhere.

Keeping the cache honest
    Because the fans' own drift is visible in every status report, it needs no
    control read. What a one-time read misses is a configuration changed behind
    the daemon's back
    (front panel, aquasuite, liquidctl, a controller reset or power cycle, which
    brings back the saved configuration):

    a) per channel, a status duty further than ``duty_mismatch_tolerance``
       from the duty written to the device, on reports received after that
       channel's last change, for longer than ``duty_mismatch_s`` invalidates
       the cache and makes the next ``apply()`` GET and rewrite every channel.
       A channel that mismatches again after that rewrite is logged once as an
       error (with :attr:`AquacomputerAdapter.stuck_hint`'s explanation when it
       has one) and listed in ``stuck_channels``; it is not rewritten for the
       mismatch again until the device reports its duty (its normal writes on
       a changed command continue);
       the aquaero's output mode is only reported: every commanded output of
       its own (1-4) not in PWM mode gets one warning per open and an aquabus
       output (5-8, mode word not interpreted) none; the mode is never written.
       A commanded channel whose controller block has no control source
       (``0xFFFF``) is left out of the write instead of being written blind:
       ``apply()`` writes every other channel of that controller and sends the
       heartbeat as usual, the channel is named in ``unconfigured_channels``
       and in ``device_health``'s ``problems`` for as long as it reads that way,
       and it is logged once per open (item 89). ``read()`` is unaffected --
       it never adopts a control report. Both lists
       are recomputed from
       every control report the adapter adopts (so a mode changed in aquasuite
       mid-run reaches ``device_health`` at the next refresh), only the warnings
       are kept to one;
    b) a change of the Quadro's power-cycle count invalidates the same way;
    c) every ``ctrl_refresh_s`` (0 disables) ``apply()`` GETs the report again
       and rewrites any channel that no longer holds its duty.

Releasing and saving
    ``release()`` restores, for every channel this adapter has written, the
    fields captured by the first GET after the daemon started (aquaero:
    preset, source, minimum and maximum power; Quadro: duty) with one live SET,
    not saved. It is not called at exit (PROJECT.md section 2); a power cycle
    of the controller restores the saved configuration anyway. ``save()`` sends
    the save report once: the controller stores what it currently holds. The
    daemon never calls it; it is the commissioning step for the saved safe
    configuration (PROJECT.md section 8 item 84). On the aquaero it is verified
    to persist; on the Quadro it is not.

This module must not import :mod:`aqua_bridge.control`.
"""

from __future__ import annotations

import dataclasses
import logging
import math
import re
import time
from collections.abc import Callable, Mapping, Sequence
from dataclasses import dataclass, field
from types import MappingProxyType
from typing import Any, TypeVar

from aqua_bridge.hw.aquacomputer import (
    DUTY_MAX,
    KINDS,
    SOFT_SENSOR_PREFIX,
    SOURCE_UNCONFIGURED,
    TEMP_MAX_C,
    TEMP_MIN_C,
    ChannelSnapshot,
    DeviceKind,
    ReportError,
    SoftSensorSettings,
    StatusReport,
    active_profile,
    aquabus_present,
    capture_channel,
    channel_holds,
    channel_state,
    check_control_report,
    decode_status,
    finalize_control_report,
    is_status_report,
    patch_duties,
    restore_channel,
    software_sensor_report,
    software_sensor_settings,
)
from aqua_bridge.hw.hidraw import (
    HIDRAW_QUEUE_FULL,
    USB_CTRL_TIMEOUT_S,
    DeviceUnavailable,
    FeatureReportError,
    HidTransport,
    open_device,
)
from aqua_bridge.model import ConfigError, Mode, MpcCommand, PlantObservation

__all__ = [
    "CTRL_RETRIES_MAX",
    "ENTRY_KEYS",
    "KIND_TIMING_DEFAULTS",
    "TIMING_KEYS",
    "AquacomputerAdapter",
    "AquacomputerTiming",
    "DeviceBinding",
    "DeviceUnavailable",
    "Opener",
    "build_adapter_from_config",
    "check_watchdog",
    "parse_device_section",
]

_LOG = logging.getLogger("aqua_bridge.hw.aquacomputer")

#: Command modes in which no channel is written below the duty the device holds.
_NEVER_LOWER_MODES = frozenset({Mode.FALLBACK, Mode.DEGRADED})

#: ``(kind, serial) -> open transport``; the default discovers and opens hidraw.
Opener = Callable[[DeviceKind, str | None], HidTransport]

_T = TypeVar("_T")


# ---------------------------------------------------------------------------
# Configuration model
# ---------------------------------------------------------------------------

#: Validation bound of ``ctrl_retries``: with ``ctrl_budget_s`` limiting the
#: time, more retries only add log lines.
CTRL_RETRIES_MAX = 5

#: Why no ``softN`` may be bound as a temperature input (PROJECT.md section 8 item
#: 113). One sentence, used by both the config error and the binding's own check, so
#: the operator reads the same reason wherever the refusal comes from.
_SOFT_SENSOR_REFUSAL = (
    "{name!r} is a software sensor, which cannot be bound as a temperature "
    "(PROJECT.md section 8 item 113): a softN slot holds whatever some host last wrote "
    "into it, and its configured fallback for ever after that host stops -- a steady "
    "number that reads exactly like a measurement, never goes stale and never shows "
    "'no data'. The daemon writes at most one of them itself (heartbeat_sensor), and "
    "that one carries heartbeat_value_c, its own constant. Bind a physical sensor "
    "(tempN), or an aquabus slot (busN) of a device that cannot leave the bus"
)

#: The timing defaults that depend on the device kind (owner decision
#: 2026-09-15, measured on the Pi: aquaero writes back to back fail with EPIPE
#: at 0 and 25 ms, none at 50-150 ms; the Quadro needs no gap).
KIND_TIMING_DEFAULTS: Mapping[str, Mapping[str, Any]] = MappingProxyType(
    {
        "aquaero": MappingProxyType({"ctrl_gap_ms": 100.0}),
        "quadro": MappingProxyType({"ctrl_gap_ms": 0.0}),
    }
)


@dataclass(frozen=True, kw_only=True)
class AquacomputerTiming:
    """The operator-tunable keys of one device: the control-report timing and the
    software-sensor heartbeat. Every default is declared here once, ``ctrl_gap_ms``
    per kind in :data:`KIND_TIMING_DEFAULTS`; build one with :meth:`for_kind`
    (documented in both example configs and PROJECT.md section 3). The keys whose
    bounds depend on the device kind are checked by :meth:`check_kind`."""

    #: Wait after a control operation before the next GET or SET, ms (>= 0); per kind.
    ctrl_gap_ms: float
    #: A status report older than this is no observation, seconds (> 0).
    status_max_age_s: float = 3.0
    #: Retries of a failed control operation, each from a fresh GET (int 0..5).
    ctrl_retries: int = 1
    #: No control operation starts once this much of one apply() is spent, seconds (> 0).
    ctrl_budget_s: float = 5.0
    #: Read the control report again this often, seconds (>= 0; 0 disables).
    ctrl_refresh_s: float = 60.0
    #: Status duty further than this from the written duty is a mismatch, centi-percent.
    duty_mismatch_tolerance: int = 100
    #: A mismatch lasting longer than this rewrites the channels, seconds (> 0).
    duty_mismatch_s: float = 5.0
    #: A falling duty is written at most this often per device, seconds (>= 0; 0 writes
    #: every fall). Writes are not saved (item 86), so this only limits USB traffic.
    write_min_interval_s: float = 0.0
    #: A falling duty is written only this far below the written one, centi-percent
    #: (0 writes every fall).
    write_deadband: int = 0
    #: Every aquabus fan block must read "no device" for this long, over live
    #: status reports, before the controller reports the device on its aquabus as
    #: lost, seconds (> 0). It bounds only the *diagnosis*: the aquabus
    #: temperatures of an absent device read as missing from the first such
    #: report, without waiting (module docstring, Reading). What it buys is the
    #: blip: a device re-enumerating shows one or two reports of ``0xFFFF``
    #: (item 90), and a window of a few report periods keeps those out of the
    #: health payload. Nothing about the aquabus bounds it from below: the
    #: electrical fields of a bus device's block are sampled inside the PWM cycle
    #: and alternate with the duty, but the speed field presence is read from is
    #: in every report and takes no part in that (item 115).
    bus_absent_s: float = 10.0
    #: Software sensor (``softN``) that gets the heartbeat every ``apply()``;
    #: 0 is off. Only the aquaero has a known software-sensor report.
    heartbeat_sensor: int = 0
    #: The temperature the heartbeat writes, degC. It must stay below whatever
    #: alarm the controller has on that sensor (the device's own configuration).
    heartbeat_value_c: float = 20.0

    @property
    def heartbeat_on(self) -> bool:
        return self.heartbeat_sensor > 0

    def __post_init__(self) -> None:
        _number("ctrl_gap_ms", self.ctrl_gap_ms, positive=False)
        _number("status_max_age_s", self.status_max_age_s, positive=True)
        _integer("ctrl_retries", self.ctrl_retries, 0, CTRL_RETRIES_MAX)
        _number("ctrl_budget_s", self.ctrl_budget_s, positive=True)
        _number("ctrl_refresh_s", self.ctrl_refresh_s, positive=False)
        _integer("duty_mismatch_tolerance", self.duty_mismatch_tolerance, 0, DUTY_MAX)
        _number("duty_mismatch_s", self.duty_mismatch_s, positive=True)
        _number("write_min_interval_s", self.write_min_interval_s, positive=False)
        _integer("write_deadband", self.write_deadband, 0, DUTY_MAX)
        _number("bus_absent_s", self.bus_absent_s, positive=True)
        _integer("heartbeat_sensor", self.heartbeat_sensor, 0, None)
        _in_range("heartbeat_value_c", self.heartbeat_value_c, TEMP_MIN_C, TEMP_MAX_C)

    def check_kind(self, kind: DeviceKind) -> None:
        """Validates what only the device kind can bound: which software sensors
        exist, and whether the kind has a software-sensor report at all.

        ``bus_absent_s`` is deliberately *not* bounded here by the kind's aquabus
        refresh interval: that interval moves the electrical fields of an aquabus
        block, while presence is read from the speed field, which every report
        carries (PROJECT.md section 8 items 115 and 92), so no skipped poll can be
        read as a departure at any window. A very short window only risks
        reporting a re-enumeration blip, which costs a log line, never cooling.
        """
        if not self.heartbeat_on:
            return
        count = kind.soft_sensor_count if kind.soft_sensor_report_id is not None else None
        if count is None:
            raise ConfigError(
                f"heartbeat_sensor is not supported on the {kind.name}: its software-sensor "
                "report is not known (only the aquaero's is); use 0 to switch the heartbeat off"
            )
        if not 1 <= self.heartbeat_sensor <= count:
            raise ConfigError(
                f"heartbeat_sensor must be 0 (off) or one of the {kind.name}'s software "
                f"sensors 1..{count}, got {self.heartbeat_sensor}"
            )

    @classmethod
    def for_kind(cls, kind: DeviceKind | str, **overrides: Any) -> AquacomputerTiming:
        """The defaults for ``kind`` with ``overrides`` applied."""
        name = kind if isinstance(kind, str) else kind.name
        timing = cls(**{**KIND_TIMING_DEFAULTS[name], **overrides})
        timing.check_kind(KINDS[name])
        return timing

    @classmethod
    def from_section(
        cls, section: Mapping[str, Any], label: str, kind: DeviceKind
    ) -> AquacomputerTiming:
        values = {f.name: section[f.name] for f in dataclasses.fields(cls) if f.name in section}
        try:
            return cls.for_kind(kind, **values)
        except ConfigError as exc:
            raise ConfigError(f"{label}.{exc}") from exc

    def worst_case_tick_s(self) -> float:
        """The longest one ``read()`` plus one ``apply()`` of this device can block.

        ``read()`` waits up to ``status_max_age_s`` for the first report after an
        open. ``apply()`` starts no control operation after ``ctrl_budget_s``,
        but one started just before can still block for a usbhid control
        transfer timeout, and the retry after it sleeps ``ctrl_gap_ms`` before it
        finds the budget spent.
        """
        return (
            self.status_max_age_s
            + self.ctrl_budget_s
            + USB_CTRL_TIMEOUT_S
            + self.ctrl_gap_ms / 1000.0
        )


TIMING_KEYS: tuple[str, ...] = tuple(f.name for f in dataclasses.fields(AquacomputerTiming))


def _number(name: str, value: Any, *, positive: bool) -> None:
    ok = isinstance(value, int | float) and not isinstance(value, bool) and math.isfinite(value)
    if ok and (value > 0 if positive else value >= 0):
        return
    bound = "> 0" if positive else ">= 0"
    raise ConfigError(f"{name} must be a finite number {bound}, got {value!r}")


def _in_range(name: str, value: Any, minimum: float, maximum: float) -> None:
    ok = isinstance(value, int | float) and not isinstance(value, bool) and math.isfinite(value)
    if ok and minimum <= value <= maximum:
        return
    raise ConfigError(f"{name} must be a finite number in {minimum:g}..{maximum:g}, got {value!r}")


def _integer(name: str, value: Any, minimum: int, maximum: int | None) -> None:
    ok = isinstance(value, int) and not isinstance(value, bool) and value >= minimum
    if ok and (maximum is None or value <= maximum):
        return
    bound = f">= {minimum}" if maximum is None else f"in {minimum}..{maximum}"
    raise ConfigError(f"{name} must be an integer {bound}, got {value!r}")


def check_watchdog(
    timings: Sequence[tuple[str, AquacomputerTiming]],
    watchdog_s: float | None,
    *,
    dt: float | None = None,
    step_bound_s: float | None = None,
) -> None:
    """Raises :class:`ConfigError` when the longest interval between two watchdog
    pings can reach the systemd watchdog: a daemon that silent is killed without
    its ``fallback_pwm`` stop write. ``watchdog_s`` ``None`` (no watchdog) skips it.

    ``WATCHDOG=1`` goes out at the end of a tick and the loop then sleeps until the
    next tick, so after a normal tick the next ping can come ``dt`` plus one
    worst-case tick later. That tick is the controller I/O
    (:meth:`AquacomputerTiming.worst_case_tick_s`, summed over the devices) plus
    ``step()``. ``step_bound_s`` is the step time the config states,
    ``mpc.budget_alarm_ms`` (both modes have it): a step past it is logged as an
    error, not interrupted, so this is the configured bound, not a guarantee.
    Publishers and the recorder are not counted; keep a margin for them.
    """
    if watchdog_s is None:
        return
    if dt is None or step_bound_s is None:
        raise ValueError("check_watchdog needs dt and step_bound_s with a watchdog")
    parts = [(label, timing.worst_case_tick_s()) for label, timing in timings]
    total = dt + step_bound_s + sum(worst for _, worst in parts)
    if total >= watchdog_s:
        detail = ", ".join(f"{label} {worst:g} s" for label, worst in parts)
        raise ConfigError(
            f"two watchdog pings can be {total:g} s apart (mpc.dt {dt:g} s + step bound "
            f"mpc.budget_alarm_ms {step_bound_s:g} s + controllers {detail}: status_max_age_s "
            f"+ ctrl_budget_s + one {USB_CTRL_TIMEOUT_S:g} s control transfer + ctrl_gap_ms "
            f"each), not below the systemd watchdog of {watchdog_s:g} s; lower those keys or "
            f"raise WatchdogSec"
        )


def aquabus_binding_problem(
    kind: DeviceKind,
    pwm_map: Mapping[str, int],
    fan_map: Mapping[str, int],
    temp_map: Mapping[str, str],
) -> str | None:
    """Why this binding may not read the kind's aquabus temperature slots, or ``None``.

    A ``busN`` slot keeps the last value it read when the device on aquabus leaves
    (PROJECT.md section 8 item 92), so it is a reading only while a device answers
    there -- and that is judged from the aquabus *fan* blocks
    (:func:`~aqua_bridge.hw.aquacomputer.aquabus_present`). A bus device with no fan
    outputs is therefore indistinguishable from an empty bus, and its slots would read
    as missing for ever: a zone that never gets a temperature on a healthy system. The
    config model refuses that shape instead of warning about it -- binding one of the
    device's aquabus outputs in the same entry is how a config says the device on the
    bus is one whose presence can be seen.
    """
    bus_inputs = frozenset(kind.aquabus_temp_names)
    bound = sorted(name for name, value in temp_map.items() if value in bus_inputs)
    if not bound:
        return None
    aquabus = frozenset(kind.aquabus_outputs)
    if any(number in aquabus for mapping in (pwm_map, fan_map) for number in mapping.values()):
        return None
    outputs = ", ".join(f"pwm{n}" for n in sorted(aquabus))
    return (
        f"{bound} are bound to {kind.name} aquabus temperature slots while no aquabus output "
        f"({outputs}) is bound in the same entry. Such a slot keeps the last value it read when "
        f"the device leaves the bus, so it is a reading only while a device answers on aquabus "
        f"-- which is judged from the aquabus fan blocks, so a bus device with no fan outputs "
        f"cannot be told from an empty bus and those names would read as missing for ever. Bind "
        f"one of the device's aquabus outputs here (the supported topology commands the Quadro "
        f"through {outputs}), or drop the aquabus temperature binding (PROJECT.md section 8 "
        f"item 92)"
    )


@dataclass(frozen=True)
class DeviceBinding:
    """Which device, and where the logical names live on it.

    ``pwm_map`` is channel -> output number (``pwmN``), ``fan_map`` channel ->
    tachometer number (``fanN``; keys must be ``pwm_map`` channels), ``temp_map``
    logical temperature -> the kind's temperature input name (``temp1``,
    ``bus2``, ``virt1``). Numbers are 1-based as in the config.
    ``timing`` defaults to :meth:`AquacomputerTiming.for_kind`.

    A ``softN`` name is **refused** here, on every kind (PROJECT.md section 8 item
    113): a software sensor holds whatever a host wrote and its configured fallback
    once that host stops, which no status report tells apart from a reading. The
    check sits in this constructor, not only in the config parser, so no path
    reaches the estimator with one.
    """

    kind: DeviceKind
    pwm_map: Mapping[str, int]
    fan_map: Mapping[str, int] = field(default_factory=dict)
    temp_map: Mapping[str, str] = field(default_factory=dict)
    serial: str | None = None
    timing: AquacomputerTiming = None  # type: ignore[assignment]  # set in __post_init__

    def __post_init__(self) -> None:
        if self.timing is None:
            object.__setattr__(self, "timing", AquacomputerTiming.for_kind(self.kind))
        self.timing.check_kind(self.kind)
        kind = self.kind
        for what, mapping, count in (
            ("pwm", self.pwm_map, kind.pwm_count),
            ("fan", self.fan_map, kind.fan_count),
        ):
            for name, number in mapping.items():
                if isinstance(number, bool) or not isinstance(number, int):
                    raise ValueError(f"{what} number for {name!r} must be an int, got {number!r}")
                if not 1 <= number <= count:
                    raise ValueError(
                        f"{name!r}: {kind.name} has {what}1..{what}{count}, not {what}{number}"
                    )
        soft = set(kind.soft_sensor_names)
        for name, value in self.temp_map.items():
            if value not in kind.temp_names:
                raise ValueError(
                    f"{name!r}: {kind.name} has temperature inputs {_bindable_temps(kind)}, "
                    f"not {value!r}"
                )
            if value in soft:
                raise ValueError(f"{name!r}: {_SOFT_SENSOR_REFUSAL.format(name=value)}")
        for what, mapping in (
            ("pwm", self.pwm_map),
            ("fan", self.fan_map),
            ("temp", self.temp_map),
        ):
            if len(set(mapping.values())) != len(mapping):
                raise ValueError(f"two names share one {kind.name} {what} input: {dict(mapping)}")
        stray = sorted(set(self.fan_map) - set(self.pwm_map))
        if stray:
            raise ValueError(f"fan_map keys {stray} are not pwm_map channels")
        problem = aquabus_binding_problem(kind, self.pwm_map, self.fan_map, self.temp_map)
        if problem is not None:
            raise ValueError(problem)

    @property
    def label(self) -> str:
        return self.kind.name if self.serial is None else f"{self.kind.name} {self.serial}"


# ---------------------------------------------------------------------------
# Adapter
# ---------------------------------------------------------------------------


class _BudgetSpent(Exception):
    """``ctrl_budget_s`` ran out before ``operation`` could start."""

    def __init__(self, operation: str) -> None:
        super().__init__(f"ctrl_budget_s spent before the {operation}")
        self.operation = operation


def _default_opener(kind: DeviceKind, serial: str | None) -> HidTransport:
    return open_device(kind, serial)


class AquacomputerAdapter:
    """One aquaero or Quadro over hidraw (module docstring).

    ``clock`` (monotonic seconds) stamps observations and drives every timer;
    ``sleep`` waits out ``ctrl_gap_ms``; ``opener`` finds and opens the device.
    Nothing is opened at construction: the first ``read()`` / ``apply()`` does.
    """

    def __init__(
        self,
        binding: DeviceBinding,
        *,
        clock: Callable[[], float] = time.monotonic,
        sleep: Callable[[float], None] = time.sleep,
        opener: Opener | None = None,
    ) -> None:
        self.binding = binding
        self.kind = binding.kind
        self.timing = binding.timing
        self._clock = clock
        self._sleep = sleep
        self._opener: Opener = opener if opener is not None else _default_opener
        self._index = {ch: number - 1 for ch, number in binding.pwm_map.items()}
        self._names = {k: ch for ch, k in self._index.items()}
        #: Called when a channel is found stuck; a non-empty string it returns is
        #: added to the error (CompositeSource sets it on a Quadro next to an aquaero).
        self.stuck_hint: Callable[[], str | None] | None = None

        self._transport: HidTransport | None = None
        self._opened_t: float | None = None
        self._status: StatusReport | None = None
        #: When the newest status report was drained; ``None`` until one arrives after an open.
        self._status_t: float | None = None
        self._power_cycles: int | None = None

        #: Cached control report; ``None`` means invalid (GET before the next write).
        self._ctrl: bytes | None = None
        self._ctrl_t: float | None = None
        #: The software-sensor settings of the last control report that was decoded,
        #: kept across an invalidation and a close: they are the operator's
        #: configuration of the controller, which no write of this daemon changes, and
        #: dropping them with the cached report would make the published slots -- and
        #: the disabled-heartbeat problem that reads them -- flap (item 113).
        #: ``None`` until one has been decoded, which is *not* the same as the empty
        #: tuple of a kind whose settings are unknown (the Quadro).
        self._soft_settings: tuple[SoftSensorSettings, ...] | None = None
        #: Output number -> the ``not_measured`` mapping published for it, built once
        #: (:meth:`_not_measured`): constant prose that would otherwise be reformatted
        #: for every output on every tick and is read-only downstream.
        self._not_measured_cache: dict[int, dict[str, str]] = {}
        #: The next write sends every configured channel, held or not.
        self._rewrite = False
        #: End of the last control operation, failed ones included (ctrl_gap_ms).
        self._last_op_t: float | None = None
        #: End of the last successful SET (write_min_interval_s).
        self._last_write_t: float | None = None
        #: Channel index -> the duty the device holds for it, as written or read back.
        self._written: dict[int, int] = {}
        #: Channel index -> when that duty was last written or changed.
        self._changed_t: dict[int, float] = {}
        self._mismatch_since: dict[int, float] = {}
        #: Rewritten after a mismatch; the device has not reported the duty since.
        self._rewritten: set[int] = set()
        #: Mismatching again after that rewrite: logged once, not rewritten again.
        self._stuck: set[int] = set()
        #: Fields captured by the first GET, restored by release().
        self._originals: dict[int, ChannelSnapshot] | None = None
        self._ever_written: set[int] = set()
        #: Commanded own outputs in DC (or an unknown) mode, and commanded
        #: unconfigured blocks, as the newest adopted control report showed them
        #: (item 83): recomputed on every adoption, so a mode changed behind the
        #: daemon's back shows up at the next periodic control read.
        self._not_pwm_channels: tuple[str, ...] = ()
        self._unconfigured_channels: tuple[str, ...] = ()
        #: The same blocks by channel index: apply() leaves them out of its write.
        self._unconfigured_ks: frozenset[int] = frozenset()
        #: Blocks already logged, once per open (the lists above are state, these two
        #: only gate the log): outputs not in PWM mode, and the channels left out of
        #: a write for having no control source.
        self._not_pwm_reported: set[int] = set()
        self._unconfigured_reported: set[int] = set()
        #: Output numbers that belong to a device on the aquaero's aquabus.
        self._aquabus = frozenset(binding.kind.aquabus_outputs)
        #: The absent inputs read() found last ("pwm5 (qd1)", ...), logged when it changes.
        self._absent_inputs: tuple[str, ...] = ()
        self._absent_channels: tuple[str, ...] = ()
        #: Logical names bound to one of the kind's aquabus temperature slots
        #: (``bus1..8``): they read as missing while no device answers on aquabus
        #: (item 92), because such a slot freezes instead of going missing.
        bus_inputs = frozenset(binding.kind.aquabus_temp_names)
        self._bus_temps = frozenset(
            name for name, input_name in binding.temp_map.items() if input_name in bus_inputs
        )
        #: Anything at all is bound behind the bus: an aquabus output, an aquabus
        #: tachometer or one of those temperatures. With nothing bound, a bus with
        #: no device on it is this controller's normal state and no problem of the
        #: daemon's (the state is still published).
        self._bus_bound = bool(self._bus_temps) or any(
            number in self._aquabus
            for mapping in (binding.pwm_map, binding.fan_map)
            for number in mapping.values()
        )
        #: Whether a device answered on aquabus in the newest status report
        #: (``None``: not judged yet, or a kind without aquabus outputs).
        self._bus_present: bool | None = None
        #: A device answered on aquabus at least once since this adapter started
        #: reading: what tells a device that *left* the bus from one that was never
        #: there (item 92).
        self._bus_seen = False
        #: When the current run of "no device on aquabus" reports began, by the
        #: receipt time of the report; ``None`` while a device answers.
        self._bus_absent_since: float | None = None
        #: The loss has been reported (one error line, and ``problems`` says so).
        self._bus_lost = False
        #: The profile the last control report said the controller runs (1-based).
        self._profile: int | None = None
        #: Whether the last heartbeat write succeeded; ``None`` before the first one.
        self._heartbeat_ok: bool | None = None
        #: Why the last opened node was rejected for its serial; apply() refuses to
        #: write until a status report carries the configured serial.
        self._serial_rejected: str | None = None

    # -- diagnostics -------------------------------------------------------

    @property
    def last_status(self) -> StatusReport | None:
        """The newest decoded status report (kept after the device goes away)."""
        return self._status

    @property
    def control_report(self) -> bytes | None:
        """The cached control report, ``None`` while invalid."""
        return self._ctrl

    @property
    def is_open(self) -> bool:
        return self._transport is not None

    @property
    def stuck_channels(self) -> tuple[str, ...]:
        """Channels whose output kept disagreeing with the written duty after a
        rewrite (module docstring, rule a); empty once the device follows."""
        return tuple(sorted(self._names[k] for k in self._stuck))

    @property
    def absent_channels(self) -> tuple[str, ...]:
        """Channels whose aquabus output or bound aquabus tachometer had no device
        behind it (rpm ``0xFFFF``) in the status report the last ``read()`` used.
        Their ``rpm`` / ``pwm`` are ``None`` in the observation; every other channel
        and every temperature still comes through (PROJECT.md section 8 item 90)."""
        return self._absent_channels

    @property
    def bus_device(self) -> dict[str, Any]:
        """What the newest status report says about a device on this controller's
        aquabus (PROJECT.md section 8 item 92), for ``device_health``.

        ``state`` is the one field a line of a page or a binary sensor can show:
        ``"unknown"`` (not judged yet, or a kind with no aquabus outputs),
        ``"present"``, ``"empty"`` (reading empty, not for long enough to report),
        ``"lost"`` (a device answered and then stopped) or ``"never_seen"`` (a bus
        that has read empty since this adapter started reading -- the normal state
        of an aquaero with nothing on its bus, and not a fault).
        ``present`` is what :func:`~aqua_bridge.hw.aquacomputer.aquabus_present`
        judged (``None`` before the first read, and on a kind with no aquabus
        outputs); ``seen`` whether one ever answered since this adapter started
        reading, which is what separates a device that *left* the bus from one that
        was never there; ``absent_s`` how long the bus has read empty, measured from
        the receipt of the first report that showed it so (``None`` until a report
        arrives; the span survives a reopen of the node, because the bus state does);
        ``lost`` whether a device that *had* answered has now been absent for
        ``bus_absent_s`` and is reported -- it stays False on a bus nothing was ever
        on, which is why a consumer with room for one field should show ``state``;
        and ``temps_missing`` the logical names whose aquabus slot is reported as
        missing this tick instead of as the frozen value the controller keeps
        there.

        There is no ``refresh_reports`` any more. It carried a mean aquabus
        refresh interval measured from how many reports held a non-zero current,
        and that count is now known to follow the output's duty rather than any
        bus poll (item 115, PROJECT.md section 2), so the number was withdrawn
        rather than re-rounded. Nothing keyed on it; presence is judged from the
        speed field, which every report carries.
        """
        absent_s: float | None = None
        if self._bus_absent_since is not None and self._status_t is not None:
            absent_s = max(0.0, self._status_t - self._bus_absent_since)
        if self._bus_present is None:
            state = "unknown"
        elif self._bus_present:
            state = "present"
        elif not self._bus_lost:
            state = "empty"
        else:
            state = "lost" if self._bus_seen else "never_seen"
        return {
            "state": state,
            "present": self._bus_present,
            "seen": self._bus_seen,
            "absent_s": absent_s,
            "lost": state == "lost",
            "temps_missing": sorted(self._bus_temps) if self._bus_present is False else [],
        }

    @property
    def active_profile(self) -> int | None:
        """The profile the controller ran when its control report was last read
        (1-based; ``None`` on a kind without a profile byte, or before the first
        read). It is only as fresh as the cache: ``ctrl_refresh_s`` and the duty
        verification decide how soon a switch is noticed (module docstring)."""
        return self._profile

    @property
    def heartbeat_on(self) -> bool:
        """The software-sensor heartbeat is configured (``heartbeat_sensor``)."""
        return self.timing.heartbeat_on

    @property
    def heartbeat_ok(self) -> bool | None:
        """Whether the last heartbeat the adapter tried to send was accepted for
        sending; ``None`` while the heartbeat is off and before the first attempt.
        A tick whose duty write failed attempts none and leaves this as it was --
        what matters then is the controller's own timeout. ``True`` is not proof
        the controller saw the report either: an output report is queued, not
        acknowledged (:meth:`~aqua_bridge.hw.hidraw.HidrawTransport.write_report`);
        reading the sensor back from a status report is what would prove delivery
        (PROJECT.md section 8 item 93)."""
        return self._heartbeat_ok

    def not_pwm_channels(self) -> tuple[str, ...]:
        """Commanded outputs of the aquaero's own that the newest adopted control
        report showed in DC voltage (or an unknown) mode instead of PWM; empty until a
        control report has been read, and on the Quadro, whose mode field is
        unknown."""
        return self._not_pwm_channels

    @property
    def unconfigured_channels(self) -> tuple[str, ...]:
        """Commanded outputs whose aquaero controller block has no control source
        (``0xFFFF``), as the newest adopted control report showed them.

        The daemon leaves exactly those channels out of its writes (module docstring,
        item 89) and writes every other channel of the controller as usual, so this
        is not a failure of ``read()`` or of ``apply()``: it is the list of outputs
        this daemon is *not* commanding, and the reason belongs in front of the owner
        (``device_health``'s ``problems``). Empty until a control report has been
        adopted -- an open clears it and the first report of that open recomputes
        it -- and on the Quadro, whose blocks carry no control source."""
        return self._unconfigured_channels

    def fan_readings(self, status: StatusReport | None = None) -> dict[str, dict[str, Any]]:
        """Per commanded channel, its output's electrical readings from ``status``
        (default: the newest status report), for ``PlantObservation.inputs['fans']``
        and the fan-health rules (:mod:`aqua_bridge.health`, PROJECT.md section 8
        item 79).

        ``duty`` is the output duty the device drives, in 0..1 like ``obs.pwm``;
        ``rpm`` the speed of the channel's *bound* tachometer (``fans.<ch>.rpm``,
        reported in ``tach``), which a config may deliberately put on another block
        than the output -- a splitter, or a fan whose tach lead is on a different
        header -- and which is therefore the only speed the fitted curve of
        ``mpc.fan_models`` describes; without a binding the output's own block is
        used. ``voltage_v`` is the 12 V rail as the *output's* block reports it and
        ``current_ma`` / ``power_w`` that block's electrical draw -- each ``None``, not
        a number, where the device does not measure it for that output, because the
        figure there is a placeholder and not a measurement. Two flags say which:
        ``power_reported`` (an aquaero reports 0 mA and 0 W for its own outputs in PWM
        mode, and the current its aquabus blocks 5-8 carry is the bus device's own
        sample taken inside the PWM cycle, which at a low duty reads 0 mA in most
        reports with the fan turning) and ``rail_reported`` (the same aquabus blocks
        hold the *aquaero's own* rail in every report that sample missed the on
        phase, indistinguishable from the bus device's in a single report, so an
        aquabus output's rail is published as unknown rather than as the aquaero's
        -- PROJECT.md section 2, section 8 items 89, 115). An aquabus slot with
        no device behind it is left out entirely: its whole block is meaningless; a
        bound tachometer on such a slot leaves ``rpm`` ``None`` rather than publishing
        its ``0xFFFF``.
        """
        if status is None:
            status = self._status
        if status is None:
            return {}
        out: dict[str, dict[str, Any]] = {}
        for ch, number in sorted(self.binding.pwm_map.items(), key=lambda item: item[1]):
            if self._empty_slot(status, number):
                continue
            fan = status.fans[number - 1]
            tach = self.binding.fan_map.get(ch, number)
            reported = self.kind.reports_power(number)
            rail = self.kind.reports_rail(number)
            out[ch] = {
                "device": self.kind.name,
                "output": f"pwm{number}",
                "tach": f"fan{tach}",
                "rpm": None if self._empty_slot(status, tach) else float(status.fans[tach - 1].rpm),
                "duty": fan.duty / DUTY_MAX,
                "voltage_v": fan.voltage_v if rail else None,
                "current_ma": float(fan.current_ma) if reported else None,
                "power_w": fan.power_w if reported else None,
                "power_reported": reported,
                "rail_reported": rail,
                "not_measured": self._not_measured(number, reported=reported, rail=rail),
                "aquabus": number in self._aquabus,
            }
        return out

    def _not_measured(self, number: int, *, reported: bool, rail: bool) -> dict[str, str]:
        """``{health rule: why this output's reading cannot feed it}`` (item 117).

        The hardware module knows *why* a field is not that output's own measurement,
        so the reason is written here and carried to the health rules rather than
        guessed there. A rule named in this mapping is one the fan-health check does
        not run for this channel, which is what the published verdict has to say
        instead of a silent pass (PROJECT.md section 8 items 79, 117).

        The text depends on nothing that moves -- the output number and what the kind
        reports for it -- so each output's mapping is built once per adapter and the
        same object is handed out every tick. Nothing may mutate it: it goes straight
        into ``PlantObservation.inputs['fans']``, and from there into the published
        verdict, on a path that only reads.
        """
        cached = self._not_measured_cache.get(number)
        if cached is not None:
            return cached
        out: dict[str, str] = {}
        if not rail:
            out["rail"] = (
                f"pwm{number} is an output of a device on the {self.kind.name}'s aquabus: "
                "that block's voltage alternates between the bus device's rail and the "
                f"{self.kind.name}'s own with the sampling of the block's electrical group, "
                "with nothing in a report to tell them apart, so no rail is published and a "
                "rail sagging behind the bus device is NOT detected "
                "(PROJECT.md section 8 item 117)"
            )
        if not reported:
            out["power"] = (
                f"pwm{number} reports no current or power this daemon may judge: "
                f"{self.kind.no_power_reason(number)} (PROJECT.md section 8 items 79, 115)"
            )
        self._not_measured_cache[number] = out
        return out

    def software_sensors(self) -> list[dict[str, Any]] | None:
        """Every software temperature slot of this controller, as the newest control
        report configures it and the newest status report reads it (item 113).

        A ``softN`` slot is not a measurement: it holds the value some host last wrote
        and, once that host has been quiet for ``timeout_s``, the configured
        ``fallback_c`` for ever -- a steady number that never goes stale and never
        reads "no data", so nothing in the status report separates it from a live
        sensor. No config can bind one (:class:`DeviceBinding`), so none of this
        reaches the estimator, the recorder or a health rule; it is published so that
        a human reading ``/api/state`` sees the slot for what it is.

        Per slot: ``name``, ``enabled``, ``fallback_c``, ``timeout_s``,
        ``written_by_daemon`` (this daemon's own ``heartbeat_sensor``, the only slot
        anything here feeds), ``reading_c`` (what the status report shows, ``None``
        for a disabled slot's ``0x7FFF``) and ``reads_fallback`` -- the reading is
        exactly the configured fallback, which on an unfed slot is what it will read
        for ever.

        ``None`` while no control report has been decoded yet, which is *not* the
        empty list of a kind whose settings are not known (the Quadro): "not known
        yet" and "this device has none" must not read alike. The settings survive an
        invalidation of the cached control report (a duty mismatch invalidates one
        every time it happens) and a close, because they are the operator's
        configuration of the controller and no write of this daemon changes them --
        without that, these slots and the disabled-heartbeat problem below would
        vanish and come back with every refetch, and the Home Assistant problem
        sensor with them.
        """
        settings_all, status = self._soft_sensor_settings(), self._status
        if settings_all is None:
            return None
        fed = self.timing.heartbeat_sensor if self.timing.heartbeat_on else 0
        out: list[dict[str, Any]] = []
        for settings in settings_all:
            reading = None if status is None else status.temps.get(settings.name)
            out.append(
                {
                    "name": settings.name,
                    "enabled": settings.enabled,
                    "fallback_c": settings.fallback_c,
                    "timeout_s": settings.timeout_s,
                    "written_by_daemon": settings.number == fed,
                    "reading_c": reading,
                    "reads_fallback": settings.reads_fallback(reading),
                }
            )
        return out

    def device_health(self) -> dict[str, Any]:
        """What this controller looks like right now, for ``/api/state``, ``/api/health``,
        the MQTT state blob and the page (PROJECT.md section 8 item 83).

        Never raises and never does I/O: it reports the newest status report and what
        the last control report showed, so it still answers while the device is gone.
        ``problems`` is the human-readable list the Home Assistant problem sensor and
        the page show; it is empty exactly when nothing is wrong. ``active_profile``
        is read defensively -- the aquaero's active profile is another change's
        (PROJECT.md section 8 item 84) and is simply absent until that lands.

        ``heartbeat`` (``on``, ``ok``, ``sensor``, ``value_c``) and
        ``software_sensors`` (:meth:`software_sensors`) complete item 83's list: the
        heartbeat was on the adapter for diagnostics only until now, and the software
        sensors are what item 113 asks a human to be able to see -- which ``softN``
        slots this controller has enabled, what each one falls back to, which one this
        daemon feeds, and whether a slot is reading its fallback right now.
        ``software_sensors`` is ``null`` until a control report has been decoded and
        ``[]`` on a kind with no software sensors this daemon knows how to read.
        """
        status = self._status
        label = self.binding.label
        age: float | None = None
        if self._status_t is not None:
            age = max(0.0, self._clock() - self._status_t)
        stuck = list(self.stuck_channels)
        health: dict[str, Any] = {
            "label": label,
            "device": self.kind.name,
            "serial": self.binding.serial if status is None else status.serial,
            "firmware": None if status is None else status.firmware,
            "power_cycles": None if status is None else status.power_cycles,
            "open": self.is_open,
            "status_age_s": age,
            "stuck_channels": stuck,
            "absent_channels": list(self._absent_channels),
            "not_pwm_channels": list(self._not_pwm_channels),
            "unconfigured_channels": list(self._unconfigured_channels),
            "flows": (
                {} if status is None else {f"flow{j}": v for j, v in enumerate(status.flows, 1)}
            ),
            # The software-sensor heartbeat and the controller's own watchdog
            # configuration (PROJECT.md section 8 items 83, 84, 113).
            "heartbeat": {
                "on": self.heartbeat_on,
                "ok": self.heartbeat_ok,
                "sensor": self.timing.heartbeat_sensor if self.heartbeat_on else None,
                "value_c": self.timing.heartbeat_value_c if self.heartbeat_on else None,
            },
            "software_sensors": self._software_sensors_safely(),
            "aquabus": self.bus_device,
        }
        profile = getattr(self, "active_profile", None)  # another change adds it (item 84)
        if profile is not None:
            health["active_profile"] = profile
        problems: list[str] = []
        if stuck:
            problems.append(f"{label}: {', '.join(stuck)} do not follow the written duty")
        if self._bus_lost and self._bus_bound:
            problems.append(self._bus_problem())
        if self._absent_channels:
            problems.append(
                f"{label}: no device on aquabus behind {', '.join(self._absent_channels)}"
            )
        if self._not_pwm_channels:
            problems.append(f"{label}: {', '.join(self._not_pwm_channels)} are not in PWM mode")
        if self._unconfigured_channels:
            problems.append(
                f"{label}: the controller block of {', '.join(self._unconfigured_channels)} "
                "has no control source; those outputs are not commanded (the others are)"
            )
        if self._serial_rejected is not None:
            problems.append(self._serial_rejected)
        problems.extend(self._heartbeat_problems(health["software_sensors"]))
        health["problems"] = problems
        return health

    def _soft_sensor_settings(self) -> tuple[SoftSensorSettings, ...] | None:
        """The controller's software-sensor configuration, decoded from the cached
        control report and **kept** once decoded (``self._soft_settings``).

        The cache is not an optimisation: ``self._ctrl`` is dropped on every duty
        mismatch, every close and every refresh, and what it holds here -- which slots
        the operator enabled, their fallbacks and timeouts -- does not change with it.
        Publishing it only while a report happens to be cached would make the slots and
        the disabled-heartbeat problem appear and disappear on a rhythm that has
        nothing to do with the controller (item 113).
        """
        ctrl = self._ctrl
        if ctrl is not None:
            self._soft_settings = software_sensor_settings(self.kind, ctrl)
        return self._soft_settings

    def _software_sensors_safely(self) -> list[dict[str, Any]] | None:
        """:meth:`software_sensors`, but never raising: ``device_health`` is a
        diagnostics path and a malformed cached report must not cost it. A failure
        publishes ``None`` (not known), never ``[]`` (this device has none)."""
        try:
            return self.software_sensors()
        except Exception:  # pragma: no cover - the cached report is validated already
            _LOG.exception("%s: reading the software-sensor settings failed", self.binding.label)
            return None

    def _heartbeat_problems(self, sensors: Sequence[Mapping[str, Any]] | None) -> list[str]:
        """The two states of the software-sensor watchdog worth a human's attention.

        A heartbeat whose last write failed: the controller's own timeout is running
        and nothing here has reset it. And a ``heartbeat_sensor`` the controller has
        **disabled**, which the daemon cannot see any other way -- it writes the slot
        every tick, the write succeeds, and the sensor the alarm watches stays
        ``0x7FFF``, so the watchdog that item 84 rests on cannot fire at all. An
        enabled slot nothing feeds is *not* a problem here: it is a fact about the
        controller's configuration, it is published in ``software_sensors``, and
        making it a problem would leave a board with eight enabled sensors
        permanently not-ok.

        ``sensors`` is ``None`` only while no control report has ever been decoded:
        the settings are kept across an invalidation (:meth:`_soft_sensor_settings`),
        so the disabled-heartbeat verdict holds instead of flapping with the cache.
        """
        if not self.heartbeat_on:
            return []
        label, number = self.binding.label, self.timing.heartbeat_sensor
        out: list[str] = []
        if self._heartbeat_ok is False:
            out.append(
                f"{label}: the last software-sensor heartbeat to soft{number} was not sent; "
                "the controller's own timeout is running"
            )
        slot = next((s for s in sensors or () if s.get("name") == f"soft{number}"), None)
        if slot is not None and not slot.get("enabled"):
            out.append(
                f"{label}: heartbeat_sensor is soft{number}, which is disabled on the "
                "controller -- the slot reads 'no data' whatever this daemon writes, so the "
                "software-sensor watchdog cannot fire (PROJECT.md section 8 item 84)"
            )
        return out

    def _bus_problem(self) -> str:
        label = self.binding.label
        temps = ", ".join(sorted(self._bus_temps))
        frozen = (
            f"; the temperatures it fed are reported as missing rather than as the frozen "
            f"value of their slots: {temps}"
            if temps
            else ""
        )
        if self._bus_seen:
            return (
                f"{label}: the device on its aquabus stopped answering{frozen} "
                "(PROJECT.md section 8 item 92)"
            )
        return (
            f"{label}: no device answers on its aquabus, and none has since this daemon "
            f"started reading it{frozen} (PROJECT.md section 8 item 92)"
        )

    def close(self) -> None:
        """Closes the device node (the next call opens it again)."""
        if self._transport is not None:
            transport, self._transport = self._transport, None
            transport.close()
        self._opened_t = None
        self._status_t = None
        self._ctrl = None

    # -- open / status -----------------------------------------------------

    def _io(self, call: Callable[[], _T]) -> _T:
        """A transport call; a vanished node closes the device."""
        try:
            return call()
        except DeviceUnavailable:
            self.close()
            raise

    def _open(self) -> HidTransport:
        """Opens the node if needed, without waiting for a status report."""
        if self._transport is not None:
            return self._transport
        transport = self._opener(self.kind, self.binding.serial)
        self._transport = transport
        self._opened_t = self._clock()
        self._status_t = None
        self._power_cycles = None
        self._ctrl = None
        self._mismatch_since.clear()
        self._not_pwm_reported.clear()
        self._unconfigured_reported.clear()
        # Both lists are per-open state, like the cached control report they come
        # from: the first report this open adopts recomputes them (_scan_modes).
        self._not_pwm_channels = ()
        self._unconfigured_channels = ()
        self._unconfigured_ks = frozenset()
        # The aquabus state is deliberately *not* reset here (item 92): the bus does
        # not change because this daemon reopened the node, and _bus_lost survives an
        # open already. Clearing only the start of the empty run would leave the pair
        # inconsistent -- "lost" with an absent_s counting up from zero -- exactly
        # when someone reads it. So absent_s keeps measuring from the first report
        # that showed the bus empty, across the reopen.
        _LOG.info("%s: opened %s", self.binding.label, transport.info.node)
        return transport

    def _newest_status(self, reports: Sequence[bytes]) -> StatusReport | None:
        newest = None
        for report in reports:
            if is_status_report(self.kind, report):
                newest = report
        return None if newest is None else decode_status(self.kind, newest)

    def _drain(self, transport: HidTransport) -> bool:
        """Takes queued reports; True when a status report known to be recent arrived."""
        reports = self._io(transport.read_reports)
        if len(reports) >= HIDRAW_QUEUE_FULL:
            _LOG.warning(
                "%s: %d input reports were queued: the kernel queue was full and dropped newer "
                "reports, so these are discarded as stale",
                self.binding.label,
                len(reports),
            )
            reports = self._io(transport.read_reports)
            if len(reports) >= HIDRAW_QUEUE_FULL:
                return False
        status = self._newest_status(reports)
        if status is None:
            return False
        first = self._status_t is None
        if first:
            self._check_serial(transport, status)
        self._check_power_cycles(status, first=first)
        self._status, self._status_t = status, self._clock()
        return True

    def _check_serial(self, transport: HidTransport, status: StatusReport) -> None:
        configured = self.binding.serial
        if configured is None or status.serial == configured:
            self._serial_rejected = None
            return
        self._serial_rejected = (
            f"{self.binding.label}: {transport.info.node} reports serial {status.serial} in "
            f"its status report, not the configured serial {configured}"
        )
        self.close()
        raise DeviceUnavailable(self._serial_rejected)

    def _check_power_cycles(self, status: StatusReport, *, first: bool) -> None:
        count = status.power_cycles
        if count is None:
            return
        if not first and self._power_cycles is not None and count != self._power_cycles:
            _LOG.warning(
                "%s: power-cycle count changed from %d to %d; reading the control report "
                "again and rewriting every channel",
                self.binding.label,
                self._power_cycles,
                count,
            )
            self._invalidate(rewrite=True)
        self._power_cycles = count

    def _empty_slot(self, status: StatusReport, number: int) -> bool:
        """Output / tachometer ``number`` (1-based) is an aquabus slot with no device
        behind it (rpm ``0xFFFF``). The aquaero's own outputs are never empty slots."""
        return number in self._aquabus and not status.fans[number - 1].present

    def _check_absent(self, status: StatusReport) -> tuple[frozenset[str], frozenset[str]]:
        """Which channels have no device behind their output / their tachometer,
        with one log line per state change (module docstring, Reading).

        Returns ``(channels whose pwm is absent, channels whose rpm is absent)``;
        those values are ``None`` in the observation. Nothing raises here: an empty
        aquabus slot faults its own channel, never the whole controller.
        """
        b = self.binding
        absent_pwm = frozenset(ch for ch, n in b.pwm_map.items() if self._empty_slot(status, n))
        absent_rpm = frozenset(ch for ch, n in b.fan_map.items() if self._empty_slot(status, n))
        inputs = tuple(
            f"{role}{n} ({ch})"
            for role, mapping in (("pwm", b.pwm_map), ("fan", b.fan_map))
            for ch, n in sorted(mapping.items(), key=lambda item: item[1])
            if self._empty_slot(status, n)
        )
        if inputs != self._absent_inputs:
            self._log_absent_change(inputs, absent_pwm)
            self._absent_inputs = inputs
            self._absent_channels = tuple(sorted(absent_pwm | absent_rpm))
        return absent_pwm, absent_rpm

    def _log_absent_change(self, inputs: tuple[str, ...], absent_pwm: frozenset[str]) -> None:
        label = self.binding.label
        if not inputs:
            _LOG.info("%s: a device is behind %s again", label, ", ".join(self._absent_inputs))
            return
        commanded = set(self.binding.pwm_map)
        all_gone = bool(commanded) and absent_pwm >= commanded
        blind = "; this controller commands no reachable output now" if all_gone else ""
        _LOG.error(
            "%s: no device behind %s: the status report's fan block reads rpm 0xFFFF (nothing "
            "on the aquaero's aquabus). Those channels report no rpm and no duty, every other "
            "channel and every temperature of this controller is unaffected, and writes still "
            "go out%s (PROJECT.md section 8 item 90)",
            label,
            ", ".join(inputs),
            blind,
        )

    def _check_bus(self, status: StatusReport, received: float) -> bool:
        """Whether no device answers on this controller's aquabus, from the newest
        status report (module docstring, Reading; PROJECT.md section 8 item 92).

        Returns True as soon as one report shows every aquabus fan block without a
        device -- that is what makes this controller's aquabus temperatures read as
        missing, and holding a frozen temperature back one report too early costs
        nothing while passing one on costs the solver its evidence. The *report* --
        the error line and ``device_health``'s ``problems`` -- waits until the bus
        has read that way for ``bus_absent_s``, which is what tells a device that
        left the bus from one re-enumerating for a report or two (item 90). A poll
        the controller skipped (item 115) never enters into it: that moves the
        block's electrical fields, not the speed field read here.

        The run is measured between report receipts -- from the first report that
        showed the bus empty to the newest one -- and survives a reopen of the node,
        because the bus does not change when this daemon reopens a device. Only a
        report showing a device ends it.
        """
        present = aquabus_present(self.kind, status)
        self._bus_present = present
        if present is None:  # a kind with no aquabus outputs cannot say
            return False
        if present:
            if self._bus_lost and self._bus_bound:
                _LOG.info(
                    "%s: a device answers on aquabus again%s",
                    self.binding.label,
                    "; its temperature slots are readings once more"
                    if self._bus_temps
                    else " (nothing of this controller reads its aquabus temperature slots)",
                )
            self._bus_seen = True
            self._bus_absent_since = None
            self._bus_lost = False
            return False
        if self._bus_absent_since is None:
            self._bus_absent_since = received
        held = received - self._bus_absent_since
        if not self._bus_lost and held >= self.timing.bus_absent_s:
            self._bus_lost = True
            if self._bus_bound:
                self._log_bus_lost(held)
        return True

    def _log_bus_lost(self, held: float) -> None:
        label = self.binding.label
        temps = ", ".join(sorted(self._bus_temps))
        frozen = (
            f"Its aquabus temperature slots keep the last value they read, which nothing in "
            f"the report marks as old, so they are reported as missing from here on: {temps}"
            if temps
            else "Nothing of this controller is bound to its aquabus temperature slots, so "
            "only its outputs are lost"
        )
        what = (
            "the device on its aquabus stopped answering"
            if self._bus_seen
            else "no device has answered on its aquabus, and none has since this daemon "
            "started reading it"
        )
        _LOG.error(
            "%s: %s -- every aquabus fan block has read rpm 0xFFFF for %.1f s (longer than "
            "bus_absent_s = %g, so it is not a device re-enumerating for a report or two). "
            "%s. "
            "Nothing is commanded lower for it: a lost bus device is less evidence, not "
            "less heat, so what it fed reads as missing and the zones it served hold or "
            "raise (PROJECT.md section 8 item 92)",
            label,
            what,
            held,
            self.timing.bus_absent_s,
            frozen,
        )

    # -- read --------------------------------------------------------------

    def read(self) -> PlantObservation:
        """One observation from the newest status report (module docstring)."""
        transport = self._open()
        max_age = self.timing.status_max_age_s
        fresh = self._drain(transport)
        if self._status_t is None:
            assert self._opened_t is not None
            deadline = self._opened_t + max_age
            while not fresh:
                remaining = deadline - self._clock()
                if remaining <= 0:
                    raise DeviceUnavailable(
                        f"{self.binding.label}: {transport.info.node} sent no status report "
                        f"within status_max_age_s = {max_age:g} s of opening"
                    )
                self._io(lambda wait=remaining: transport.wait_readable(wait))
                fresh = self._drain(transport)
        status, received = self._status, self._status_t
        assert status is not None and received is not None
        now = self._clock()
        if now - received > max_age:
            raise DeviceUnavailable(
                f"{self.binding.label}: no status report from {transport.info.node} for "
                f"{now - received:.1f} s (status_max_age_s = {max_age:g})"
            )
        if fresh:
            self._verify_duties(status, received)
        absent_pwm, absent_rpm = self._check_absent(status)
        bus_gone = self._check_bus(status, received)
        b = self.binding
        readings = self.fan_readings(status)
        return PlantObservation(
            temps={
                name: None if (bus_gone and name in self._bus_temps) else status.temp(input_name)
                for name, input_name in b.temp_map.items()
            },
            rpm={
                ch: None if ch in absent_rpm else float(status.rpm(n))
                for ch, n in b.fan_map.items()
            },
            pwm={
                ch: None if ch in absent_pwm else status.duty(n) / DUTY_MAX
                for ch, n in b.pwm_map.items()
            },
            ts=now,
            inputs={"fans": readings} if readings else {},
        )

    def _invalidate(self, *, rewrite: bool) -> None:
        self._ctrl = None
        if rewrite:
            self._rewrite = True

    def _stuck_hint(self) -> str:
        if self.stuck_hint is None:
            return ""
        hint = self.stuck_hint()
        return f"; {hint}" if hint else ""

    def _verify_duties(self, status: StatusReport, received: float) -> None:
        """Rule a of the module docstring, per channel, on one newly received report."""
        if self._ctrl is None:
            return
        tolerance = self.timing.duty_mismatch_tolerance
        label = self.binding.label
        for k, duty in sorted(self._written.items()):
            if self._empty_slot(status, k + 1):
                continue  # no device behind it: no duty evidence (read() reports the
                # channel as None and lists it in absent_channels, item 90)
            reported = status.fans[k].duty
            name = self._names.get(k, "?")
            if abs(reported - duty) <= tolerance:
                self._mismatch_since.pop(k, None)
                self._rewritten.discard(k)
                if k in self._stuck:
                    self._stuck.discard(k)
                    _LOG.info("%s: pwm%d (%s) follows its duty again", label, k + 1, name)
                continue
            if k in self._stuck or received <= self._changed_t.get(k, -math.inf):
                continue
            start = self._mismatch_since.setdefault(k, received)
            if received - start <= self.timing.duty_mismatch_s:
                continue
            del self._mismatch_since[k]
            if k in self._rewritten:
                self._stuck.add(k)
                _LOG.error(
                    "%s: pwm%d (%s) still reports %.2f %% after it was rewritten to %.2f %%; "
                    "not rewriting it again until the device follows (PROJECT.md section 8 "
                    "item 81)%s",
                    label,
                    k + 1,
                    name,
                    reported / 100.0,
                    duty / 100.0,
                    self._stuck_hint(),
                )
                continue
            self._rewritten.add(k)
            _LOG.warning(
                "%s: pwm%d (%s) holds %.2f %% but the device has reported %.2f %% for %.1f s; "
                "reading the control report again and rewriting every channel",
                label,
                k + 1,
                name,
                duty / 100.0,
                reported / 100.0,
                received - start,
            )
            self._invalidate(rewrite=True)

    # -- control report ----------------------------------------------------

    def _wait_gap(self) -> None:
        if self._last_op_t is None:
            return
        remaining = self.timing.ctrl_gap_ms / 1000.0 - (self._clock() - self._last_op_t)
        if remaining > 0:
            self._sleep(remaining)

    def _control(self, deadline: float, operation: str, call: Callable[[], _T]) -> _T:
        """One control operation: gap, budget, and its end time recorded even on failure."""
        self._wait_gap()
        if self._clock() >= deadline:
            raise _BudgetSpent(operation)
        try:
            return call()
        finally:
            self._last_op_t = self._clock()

    def _fetch(self, transport: HidTransport, deadline: float) -> bytes:
        self._ctrl = None
        data = self._control(
            deadline,
            "control report GET",
            lambda: transport.get_feature(self.kind.ctrl_report_id, self.kind.ctrl_size),
        )
        check_control_report(self.kind, data)
        data = bytes(data)
        if self._originals is None:
            self._originals = {k: capture_channel(self.kind, data, k) for k in self._names}
        return data

    def _check_profile(self, data: bytes) -> None:
        """Byte ``0x06`` of a fresh aquaero control report is the profile it runs.

        A profile switch reloads the *saved* profile, so every duty written live is
        gone; the next write therefore sends every configured channel. One line per
        change (PROJECT.md section 8 item 84).
        """
        profile = active_profile(self.kind, data)
        if profile is None:
            return
        previous, self._profile = self._profile, profile
        if previous is None or previous == profile:
            return
        _LOG.warning(
            "%s: the active profile changed from profile %d to profile %d; the switch "
            "reloads the saved profile, so every duty written live is gone: writing every "
            "configured channel again (PROJECT.md section 8 item 84)",
            self.binding.label,
            previous,
            profile,
        )
        self._rewrite = True

    def _adopt(self, data: bytes) -> list[int]:
        """Takes a fresh control report as the cache. Returns the channels whose held
        duty is no longer the one this adapter knew (external changes).

        Adopting never refuses anything: a commanded block with no control source is
        only recorded here (:meth:`_scan_modes`), and :meth:`_apply_once` leaves that
        one channel out of the write. Refusing here would take every other channel of
        the same controller -- and the software-sensor heartbeat -- down with it
        (PROJECT.md section 8 item 89)."""
        self._check_profile(data)
        now = self._clock()
        drifted: list[int] = []
        for k in sorted(self._names):
            state = channel_state(self.kind, data, k)
            known = self._written.get(k)
            held = state.duty if state.on_duty else None
            if held == known:
                continue
            if known is not None:
                drifted.append(k)
            self._mismatch_since.pop(k, None)
            self._rewritten.discard(k)
            self._stuck.discard(k)
            if held is None:
                self._written.pop(k, None)
            else:
                self._written[k] = held
                self._changed_t[k] = now
        self._ctrl, self._ctrl_t = data, now
        self._scan_modes(data)
        return drifted

    def _write(self, transport: HidTransport, report: bytes, deadline: float) -> float:
        """One control report SET, not saved; returns when it ended."""
        # Until the SET went out, the next write sends every channel: a failed SET may
        # or may not have reached the device.
        self._rewrite = True
        self._control(deadline, "control report SET", lambda: transport.set_feature(report))
        assert self._last_op_t is not None
        self._last_write_t = self._last_op_t
        self._ctrl = report
        self._rewrite = False
        return self._last_write_t

    def _with_retries(
        self,
        what: str,
        operation: Callable[[HidTransport, float], _T],
        *,
        retry: bool = True,
    ) -> _T:
        if self._serial_rejected is not None:
            raise DeviceUnavailable(
                f"{self._serial_rejected}; nothing is written until a status report carries "
                "the configured serial"
            )
        transport = self._open()
        budget = self.timing.ctrl_budget_s
        deadline = self._clock() + budget
        attempts = self.timing.ctrl_retries + 1 if retry else 1
        failure: Exception | None = None
        for attempt in range(1, attempts + 1):
            try:
                return operation(transport, deadline)
            except _BudgetSpent as spent:
                after = f"; last failure: {failure}" if failure is not None else ""
                raise DeviceUnavailable(
                    f"{self.binding.label}: {what}: ctrl_budget_s = {budget:g} s spent before "
                    f"the {spent.operation}{after}"
                ) from failure
            except (FeatureReportError, ReportError) as exc:
                failure = exc
                self._invalidate(rewrite=False)
                _LOG.warning(
                    "%s: %s failed (attempt %d of %d): %s",
                    self.binding.label,
                    what,
                    attempt,
                    attempts,
                    exc,
                )
                if attempt == attempts:
                    raise DeviceUnavailable(
                        f"{self.binding.label}: {what} failed {attempts} time(s): {exc}"
                    ) from exc
            except DeviceUnavailable:
                self.close()
                raise
        raise AssertionError("unreachable")  # pragma: no cover

    # -- software-sensor heartbeat ------------------------------------------

    def _send_heartbeat(self, transport: HidTransport, deadline: float) -> None:
        """One heartbeat into the configured software sensor (module docstring).

        Called only after the tick's duty work went out: the heartbeat holds the
        controller's own watchdog shut, so a daemon that is alive but cannot write
        must not send it (module docstring).

        Never raises: a heartbeat that does not go out costs the controller's own
        timeout, and the profile its alarm then selects is the safe one, so it must
        not turn a tick into a failed write. It is sequenced like every other device
        operation (``ctrl_gap_ms`` before it, ``ctrl_budget_s`` over it), which keeps
        the worst case per tick exactly the one ``worst_case_tick_s`` states.
        """
        number = self.timing.heartbeat_sensor
        try:
            report = software_sensor_report(self.kind, {number: self.timing.heartbeat_value_c})
            self._control(
                deadline,
                f"software-sensor heartbeat to soft{number}",
                lambda: transport.write_report(report),
            )
        except Exception as exc:  # a failed heartbeat never fails the tick
            self._note_heartbeat(False, exc)
        else:
            self._note_heartbeat(True, None)

    def _note_heartbeat(self, ok: bool, error: BaseException | None) -> None:
        """Logs a heartbeat state change, once (not once per tick)."""
        if ok == self._heartbeat_ok:
            return
        first, self._heartbeat_ok = self._heartbeat_ok is None, ok
        number = self.timing.heartbeat_sensor
        if ok:
            _LOG.info(
                "%s: writing the software-sensor heartbeat of %.2f degC to soft%d %s",
                self.binding.label,
                self.timing.heartbeat_value_c,
                number,
                "every write" if first else "again",
            )
            return
        _LOG.error(
            "%s: the software-sensor heartbeat to soft%d failed: %s; the controller falls "
            "back to its own configured temperature for that sensor after its timeout, and "
            "whatever alarm it has on it takes over (PROJECT.md section 8 item 84)",
            self.binding.label,
            number,
            error,
        )

    def _refresh_due(self) -> bool:
        period = self.timing.ctrl_refresh_s
        return (
            period > 0
            and self._ctrl is not None
            and self._ctrl_t is not None
            and self._clock() - self._ctrl_t >= period
        )

    # -- apply -------------------------------------------------------------

    def _duties(self, cmd: MpcCommand) -> dict[int, int]:
        duties: dict[int, int] = {}
        for channel, k in self._index.items():
            if channel not in cmd.pwm:
                raise ValueError(f"apply: command has no pwm value for channel {channel!r}")
            value = cmd.pwm[channel]
            if not isinstance(value, int | float) or isinstance(value, bool):
                raise ValueError(f"apply: pwm[{channel!r}] must be a number, got {value!r}")
            if not math.isfinite(value):
                raise ValueError(f"apply: pwm[{channel!r}] is not finite: {value!r}")
            if not 0.0 <= value <= 1.0:
                raise ValueError(f"apply: pwm[{channel!r}] out of range [0, 1]: {value!r}")
            duties[k] = round(value * DUTY_MAX)
        return duties

    def apply(self, cmd: MpcCommand) -> None:
        """Commands ``cmd.pwm`` for every configured channel (module docstring).

        Every configured channel needs a finite value in ``[0, 1]`` or
        :class:`ValueError` is raised before anything is sent (no silent
        clamping). Channels of other devices in ``cmd.pwm`` are ignored. A
        deferred fall (write limiting) is not an error: ``obs.pwm`` keeps
        reporting what the output actually drives.

        With ``cmd.mode`` FALLBACK or DEGRADED no channel is written below the
        duty the device holds (module docstring, Writing): the loop may have
        recorded a deferred fall as applied, and a fault must never slow a fan.

        An aquabus output with no device behind it is written with the others and
        is no error here: ``read()`` reports it as ``None`` and lists it in
        ``absent_channels`` (module docstring, Reading).

        A channel whose controller block has no control source is the one thing
        this call does not write: it is dropped from the duties, the rest of the
        controller is written in the same SET, and the channel is reported through
        ``unconfigured_channels`` and ``device_health`` (item 89). No exception --
        one such block must not take a whole controller to the aquaero's watchdog
        fallback.

        With ``heartbeat_sensor`` configured, the software-sensor heartbeat goes
        out after the duty work, once per call and whatever the duties do -- and
        *not at all* when this call cannot command the duties (module docstring).
        """
        duties = self._duties(cmd)
        never_lower = getattr(cmd, "mode", None) in _NEVER_LOWER_MODES

        def once(transport: HidTransport, deadline: float) -> None:
            # The duties first: the heartbeat tells the device the daemon is still
            # commanding it, so it must not go out on a tick that failed to. A
            # retried attempt reaches this line only once, when it succeeded.
            self._apply_once(transport, duties, deadline, never_lower)
            if self.timing.heartbeat_on:
                self._send_heartbeat(transport, deadline)

        self._with_retries("control report write", once)

    def _scan_modes(self, ctrl: bytes) -> None:
        """Recomputes ``not_pwm_channels`` / ``unconfigured_channels`` from ``ctrl`` and
        warns once per open about each output not in PWM mode.

        Called for every control report this adapter adopts, not only the first of an
        open: the owner can switch an output to DC voltage in aquasuite at any time,
        and :meth:`device_health` promises what the controller looks like *now*
        (PROJECT.md section 8 item 83). The mode itself is only reported, never
        changed (items 81, 85). A commanded block with no control source is collected
        here, by channel index as well as by name; :meth:`_apply_once` leaves those
        channels out of the write (item 89)."""
        not_pwm: list[str] = []
        unconfigured: list[str] = []
        unconfigured_ks: list[int] = []
        for k in sorted(self._names):
            state = channel_state(self.kind, ctrl, k)
            mode = state.mode
            if mode is None:
                continue
            if state.unconfigured:
                unconfigured.append(self._names[k])
                unconfigured_ks.append(k)
                continue
            if state.aquabus or mode.is_pwm:
                self._not_pwm_reported.discard(k)  # back in PWM: say so again if it leaves
                continue  # an aquabus output's mode word is not interpreted
            not_pwm.append(self._names[k])
            if k in self._not_pwm_reported:
                continue
            self._not_pwm_reported.add(k)
            _LOG.warning(
                "%s: pwm%d (%s) is in %s mode (mode word 0x%04X), not PWM; the daemon does not "
                "change the mode, set it with the controller's own software",
                self.binding.label,
                k + 1,
                self._names[k],
                "DC voltage" if mode.name == "dc" else "an unknown",
                mode.raw,
            )
        self._not_pwm_channels = tuple(sorted(not_pwm))
        self._unconfigured_channels = tuple(sorted(unconfigured))
        self._unconfigured_ks = frozenset(unconfigured_ks)

    def _falls_may_be_written(self) -> bool:
        last = self._last_write_t
        return last is None or self._clock() - last >= self.timing.write_min_interval_s

    def _plan(self, ctrl: bytes, duties: Mapping[int, int]) -> dict[int, int]:
        """The channels one write sends now (empty: no write). Module docstring, Writing."""
        if self._rewrite:
            return dict(duties)
        pending = {
            k: duty for k, duty in duties.items() if not channel_holds(self.kind, ctrl, k, duty)
        }
        for k, duty in pending.items():
            state = channel_state(self.kind, ctrl, k)
            if not state.on_duty or duty > state.duty:
                return pending  # a rise, or a channel not following: never waits
            if state.duty - duty >= self.timing.write_deadband and self._falls_may_be_written():
                return pending
        return {}

    def _not_below_held(self, ctrl: bytes, duties: Mapping[int, int]) -> dict[int, int]:
        """``max(command, held)`` per channel. Held is the duty the control report
        holds; for a channel that does not follow its duty (an aquaero channel on a
        firmware controller) it is the newest status report's output duty since this
        open, or 100 % when there is none."""
        status = self._status if self._status_t is not None else None
        out: dict[int, int] = {}
        for k, duty in duties.items():
            state = channel_state(self.kind, ctrl, k)
            if state.on_duty:
                held = state.duty
            elif status is not None:
                held = status.fans[k].duty
            else:
                held = DUTY_MAX
            out[k] = max(duty, held)
        return out

    def _without_unconfigured(self, duties: Mapping[int, int]) -> Mapping[int, int]:
        """``duties`` without the channels whose controller block has no control source.

        Nothing on the device drives such an output, and no capture shows that writing
        the block the way a configured one is written would change that, so the daemon
        will not write it blind (PROJECT.md section 8 item 89). The refusal is per
        channel: every other channel of the controller goes out in the same write, and
        the software-sensor heartbeat still follows it -- a controller with one
        unconfigured block must not take the whole device to the watchdog fallback.
        The channel stays in ``unconfigured_channels`` and in ``device_health``'s
        ``problems`` for as long as the controller reads that way, and is logged once
        per open."""
        if not self._unconfigured_ks:
            return duties
        refused = sorted(k for k in duties if k in self._unconfigured_ks)
        if not refused:
            return duties
        fresh = [k for k in refused if k not in self._unconfigured_reported]
        if fresh:
            self._unconfigured_reported.update(fresh)
            _LOG.error(
                "%s: %s: the controller block has no control source (0x%04X), so nothing on "
                "the device drives that output and writing the block the way a configured one "
                "is written has never been seen to change that; the daemon leaves the channel "
                "out of its writes (the other channels are written as usual). Give the output "
                "any control source in the controller's own software (the daemon replaces it "
                "with its own preset) or drop the channel from the config "
                "(PROJECT.md section 8 item 89)",
                self.binding.label,
                ", ".join(f"pwm{k + 1} ({self._names.get(k, '?')})" for k in fresh),
                SOURCE_UNCONFIGURED,
            )
        return {k: duty for k, duty in duties.items() if k not in self._unconfigured_ks}

    def _apply_once(
        self,
        transport: HidTransport,
        duties: Mapping[int, int],
        deadline: float,
        never_lower: bool = False,
    ) -> None:
        if self._refresh_due():
            drifted = self._adopt(self._fetch(transport, deadline))
            if drifted:
                _LOG.warning(
                    "%s: periodic control report read: %s changed outside the daemon; "
                    "writing the commanded duty again",
                    self.binding.label,
                    ", ".join(f"pwm{k + 1} ({self._names.get(k, '?')})" for k in drifted),
                )
        if self._ctrl is None:
            self._adopt(self._fetch(transport, deadline))
        ctrl = self._ctrl
        assert ctrl is not None
        duties = self._without_unconfigured(duties)
        if never_lower:
            duties = self._not_below_held(ctrl, duties)
        send = self._plan(ctrl, duties)
        if not send:
            return
        forced = self._rewrite
        buf = bytearray(ctrl)
        patch_duties(self.kind, buf, send)
        finalize_control_report(self.kind, buf)
        written_at = self._write(transport, bytes(buf), deadline)
        for k, duty in send.items():
            if forced or self._written.get(k) != duty:
                self._changed_t[k] = written_at
                self._mismatch_since.pop(k, None)
            self._written[k] = duty
        self._ever_written.update(send)

    # -- release / save ------------------------------------------------------

    def release(self) -> None:
        """Restores the captured control fields of every channel this adapter
        wrote, with one live SET (module docstring). No-op when nothing was written."""
        if not self._ever_written or not self._originals:
            return
        restore = {
            k: self._originals[k] for k in sorted(self._ever_written) if k in self._originals
        }
        self._with_retries(
            "control report restore",
            lambda transport, deadline: self._release_once(transport, restore, deadline),
        )

    def _release_once(
        self, transport: HidTransport, restore: dict[int, ChannelSnapshot], deadline: float
    ) -> None:
        buf = bytearray(self._fetch(transport, deadline))
        for snapshot in restore.values():
            restore_channel(buf, snapshot)
        finalize_control_report(self.kind, buf)
        self._write(transport, bytes(buf), deadline)
        self._ever_written.clear()
        self._written.clear()
        self._changed_t.clear()
        self._mismatch_since.clear()
        self._rewritten.clear()
        self._stuck.clear()
        self._adopt(bytes(buf))
        _LOG.info(
            "%s: restored the captured control settings of %s (not saved)",
            self.binding.label,
            ", ".join(f"pwm{k + 1}" for k in restore),
        )

    def control_snapshot(self) -> bytes:
        """Fetches the control report fresh and caches it (``control_report``),
        writing nothing: the same GET ``apply()`` runs when its cache is stale,
        called on its own for commissioning (PROJECT.md section 8 item 88, the
        tool that shows what :meth:`save` is about to store before asking to run
        it). Raises :class:`~aqua_bridge.hw.hidraw.DeviceUnavailable` on failure,
        not retried, like :meth:`save`."""

        def once(transport: HidTransport, deadline: float) -> bytes:
            self._adopt(self._fetch(transport, deadline))
            assert self._ctrl is not None
            return self._ctrl

        return self._with_retries("control report GET", once, retry=False)

    def save(self) -> None:
        """Sends the save report once: the controller stores the configuration it
        holds now in its memory, and a power cycle brings that back.

        Commissioning only (the saved safe configuration, PROJECT.md section 8 item
        84): the daemon never calls it, since every live write the adapter makes
        would otherwise become the configuration a controller restarts with. It
        waits ``ctrl_gap_ms`` and honours ``ctrl_budget_s`` like any control
        operation but is not retried; a failure raises :class:`DeviceUnavailable`.
        Verified on the aquaero; on the Quadro the report is only known from the
        Farbwerk 360.
        """
        self._with_retries(
            "save report",
            lambda transport, deadline: self._control(
                deadline, "save report", lambda: transport.set_feature(self.kind.save_report)
            ),
            retry=False,
        )
        _LOG.warning(
            "%s: sent the save report: the configuration the controller holds now is stored "
            "in its memory%s",
            self.binding.label,
            "" if self.kind.save_verified else " (not verified on this kind)",
        )


# ---------------------------------------------------------------------------
# Config
# ---------------------------------------------------------------------------

#: Keys of one device entry (``aquacomputer:`` list entry or ``xt6:``).
ENTRY_KEYS: tuple[str, ...] = ("device", "serial", "fans", "temp_map", *TIMING_KEYS)
_FAN_ENTRY_KEYS = ("pwm", "rpm")
_FANS_EXAMPLE = "radiator: {pwm: pwm1, rpm: fan1}"
_SERIAL_HINT = "'serial:' when several of one kind are attached"
#: Keys of the hwmon era and what replaced them.
_REMOVED_KEYS = {
    "hwmon_name": (
        "is no longer supported: the daemon talks to the controllers over hidraw, not the "
        "hwmon driver; use 'device: aquaero' (or quadro), plus " + _SERIAL_HINT
    ),
    "root": (
        "is no longer supported: devices are discovered under /sys/class/hidraw by USB id; "
        "remove it and use " + _SERIAL_HINT
    ),
    "name": "was renamed to 'device' (aquaero or quadro)",
}
_LEGACY_MAP_KEYS = {"map": "fans.<channel>.pwm", "fan_map": "fans.<channel>.rpm"}
_NUMBERED = {role: re.compile(rf"{role}([1-9][0-9]*)") for role in ("pwm", "fan")}
#: Temperature names of the hwmon driver's numbering that mean another input now.
_HWMON_ERA_TEMPS: Mapping[str, Mapping[str, str]] = MappingProxyType(
    {
        "aquaero": MappingProxyType(
            {
                **{f"temp{8 + n}": f"soft{n}" for n in range(1, 9)},
                **{f"temp{16 + n}": f"virt{n}" for n in range(1, 5)},
            }
        ),
        "quadro": MappingProxyType({f"temp{4 + n}": f"soft{n}" for n in range(1, 17)}),
    }
)
#: ``fanN`` numbers the hwmon driver gave the flow sensors that are out of a kind's
#: tachometer range now -> ``flowN``. The aquaero's hwmon ``fan5``/``fan6`` (flow 1-2)
#: are its aquabus tachometers now, accepted like any other ``fanN``.
_HWMON_ERA_FLOWS: Mapping[str, Mapping[int, int]] = MappingProxyType(
    {"aquaero": MappingProxyType({}), "quadro": MappingProxyType({5: 1})}
)


def _numbered(role: str, value: Any) -> int | None:
    match = _NUMBERED[role].fullmatch(value) if isinstance(value, str) else None
    return None if match is None else int(match.group(1))


def _output_number(where: str, value: Any, kind: DeviceKind) -> int:
    number = _numbered("pwm", value)
    if number is None or not 1 <= number <= kind.pwm_count:
        raise ConfigError(
            f"{where} must be one of the {kind.name}'s outputs pwm1..pwm{kind.pwm_count}, "
            f"got {value!r}{_flow_hint(value)}"
        )
    return number


def _flow_hint(value: Any) -> str:
    """The hint for a flow sensor name where an output, tachometer or temperature is
    expected. Flow stays out of :class:`~aqua_bridge.model.PlantObservation` (owner
    decision 2026-09-16, PROJECT.md section 8.1): it is decoded, shown by
    ``tools/aquacomputer_probe.py`` and published with the device health, but no
    config key binds it."""
    if isinstance(value, str) and re.fullmatch(r"flow[1-9][0-9]*", value):
        return (
            f"; {value!r} is a flow sensor, and flow sensors cannot be bound in the config "
            "(PROJECT.md section 8 item 91): the daemon decodes them and publishes them with "
            "the device health, but nothing in a device entry takes a 'flowN' name. The "
            "hwmon driver's aquaero 'fan5'/'fan6' were flow sensors; here fan5..fan8 are the "
            "aquaero's aquabus tachometers (a Quadro's outputs 1-4)"
        )
    return ""


def _tachometer_number(where: str, value: Any, kind: DeviceKind) -> int:
    number = _numbered("fan", value)
    if number is not None and 1 <= number <= kind.fan_count:
        return number
    flow = None if number is None else _HWMON_ERA_FLOWS[kind.name].get(number)
    if flow is not None:
        raise ConfigError(
            f"{where}: {value!r} was the hwmon driver's name of the {kind.name}'s flow sensor "
            f"flow{flow}, which is not a tachometer ({kind.name}: fan1..fan{kind.fan_count}); "
            "flow sensors cannot be bound in the config (PROJECT.md section 8 item 91). On an "
            "aquaero the driver's flow 'fan5'/'fan6' are aquabus tachometers now (a Quadro's "
            "outputs 1-2 on its aquabus), so the same name means another input there"
        )
    raise ConfigError(
        f"{where} must be one of the {kind.name}'s tachometers fan1..fan{kind.fan_count}, "
        f"got {value!r}{_flow_hint(value)}"
    )


def _bindable_temps(kind: DeviceKind) -> str:
    """The kind's temperature inputs a config may bind, for the messages that offer a
    choice: everything :meth:`DeviceKind.describe_temps` lists *except* the software
    sensors, which no config may bind (item 113). Offering ``softN`` as a valid answer
    to "which input did you mean" would send an operator straight into the refusal.
    """
    return ", ".join(
        f"{g.prefix}1..{g.prefix}{g.count}"
        for g in kind.temp_groups
        if g.prefix != SOFT_SENSOR_PREFIX
    )


def _temperature_input(where: str, value: Any, kind: DeviceKind) -> str:
    soft = kind.soft_sensor_names
    renamed = _HWMON_ERA_TEMPS[kind.name].get(value) if isinstance(value, str) else None
    if isinstance(value, str) and value in soft:
        raise ConfigError(f"{where}: {_SOFT_SENSOR_REFUSAL.format(name=value)}")
    if renamed is not None and renamed in soft:
        # The hwmon-era name would have been renamed onto a slot nothing may bind, so
        # the message says what the name is *and* why the new one is refused too.
        raise ConfigError(
            f"{where}: in the hwmon driver's numbering {value!r} is {renamed!r}, and "
            f"{_SOFT_SENSOR_REFUSAL.format(name=renamed)}"
        )
    if isinstance(value, str) and value in kind.temp_names:
        return value
    hint = _flow_hint(value)
    if renamed is not None:
        group = next(g for g in kind.temp_groups if renamed.startswith(g.prefix))
        hint = (
            f"; in the hwmon driver's numbering {value!r} is {renamed!r} (the {kind.name}'s "
            f"{group.description}): use {renamed!r}"
        )
    raise ConfigError(
        f"{where} must be one of the {kind.name}'s temperature inputs {_bindable_temps(kind)}, "
        f"got {value!r}{hint}"
    )


def _claim_input(seen: dict[Any, str], name: Any, owner: str, where: str) -> None:
    if name in seen:
        raise ConfigError(f"{where}: {seen[name]} and {owner} name the same input '{name}'")
    seen[name] = owner


def _parse_fans(
    section: Mapping[str, Any], label: str, kind: DeviceKind
) -> tuple[dict[str, int], dict[str, int]]:
    fans = section.get("fans")
    if fans is None:
        raise ConfigError(
            f"{label}.fans is required: one entry per fan channel, e.g. '{_FANS_EXAMPLE}'"
        )
    if not isinstance(fans, Mapping):
        raise ConfigError(f"{label}.fans must be a mapping, got {type(fans).__name__}")
    pwm_map: dict[str, int] = {}
    fan_map: dict[str, int] = {}
    pwm_seen: dict[Any, str] = {}
    fan_seen: dict[Any, str] = {}
    for channel, entry in fans.items():
        if not isinstance(channel, str) or not channel:
            raise ConfigError(f"{label}.fans keys must be non-empty channel names, got {channel!r}")
        where = f"{label}.fans.{channel}"
        if not isinstance(entry, Mapping):
            raise ConfigError(
                f"{where} must be a mapping like {{pwm: pwm1, rpm: fan1}}, "
                f"got {type(entry).__name__} {entry!r}"
            )
        unknown = sorted(str(k) for k in entry if k not in _FAN_ENTRY_KEYS)
        if unknown:
            raise ConfigError(
                f"{where}: unknown key(s) {unknown}; allowed: {list(_FAN_ENTRY_KEYS)}"
            )
        if "pwm" not in entry:
            raise ConfigError(f"{where}.pwm is required (the output, e.g. 'pwm1')")
        pwm_map[channel] = _output_number(f"{where}.pwm", entry["pwm"], kind)
        _claim_input(pwm_seen, f"pwm{pwm_map[channel]}", channel, f"{label}.fans")
        if entry.get("rpm") is not None:
            fan_map[channel] = _tachometer_number(f"{where}.rpm", entry["rpm"], kind)
            _claim_input(fan_seen, f"fan{fan_map[channel]}", channel, f"{label}.fans")
    return pwm_map, fan_map


def _check_keys(label: str, mapping: Mapping[str, Any], expected: Sequence[str], what: str) -> None:
    missing = sorted(set(expected) - set(mapping))
    extra = sorted(set(mapping) - set(expected))
    if missing or extra:
        raise ConfigError(
            f"{label} keys must equal {what} {sorted(expected)}: missing {missing}, extra {extra}"
        )


def parse_device_section(
    section: Mapping[str, Any],
    *,
    label: str,
    channels: Sequence[str] | None = None,
    temps: Sequence[str] | None = None,
    ignored_keys: Sequence[str] = (),
) -> DeviceBinding:
    """One device entry -> :class:`DeviceBinding`; :class:`ConfigError` naming ``label``.

    Entry shape (``xt6:`` or one ``aquacomputer:`` list entry)::

        device: aquaero               # required: aquaero | quadro
        serial: "12345-54321"         # optional; required when several of one kind are attached
        fans:                         # one entry per fan channel
          radiator: {pwm: pwm1, rpm: fan1}
          intake:   {pwm: pwm2}       # rpm optional
          quadro1:  {pwm: pwm5, rpm: fan5}  # aquaero 5-8: a Quadro on its aquabus
        temp_map: {coolant: temp1, quadro_t2: bus2}
        ctrl_gap_ms: 100              # optional timing keys, see AquacomputerTiming

    Names are checked against the device kind: outputs ``pwmN`` and
    tachometers ``fanN`` (aquaero 1-8, Quadro 1-4); temperatures ``tempN``
    (physical sensors: aquaero 1-8, Quadro 1-4), ``busN`` (aquaero aquabus
    slots 1-8), ``softN`` (software sensors: aquaero 1-8, Quadro 1-16) and
    ``virtN`` (aquaero virtual sensors 1-4). Any tachometer may be bound to any
    output. A temperature or flow name of the hwmon driver's numbering that
    means another input now (aquaero ``temp9..20``, Quadro ``temp5..20``, the
    Quadro's flow sensor ``fan5``) is rejected with the new name; the aquaero's
    hwmon ``fan5``/``fan6`` (flow) are its aquabus tachometers now. Flow
    sensors (``flowN``) cannot be bound. ``hwmon_name``, ``root`` and ``name`` (the
    hwmon era) and ``map`` / ``fan_map`` are rejected with a hint; any other
    unknown key is rejected too, so a misspelt timing key cannot silently fall
    back to its default. ``ignored_keys`` are accepted and ignored
    (``xt6.prefer``).

    ``channels`` / ``temps`` (the ``mpc`` tuples, as plain sequences) make
    the ``fans`` keys equal ``channels`` and the ``temp_map`` keys equal
    ``temps``: a channel missing from ``fans`` would silently never be
    written, and a temperature missing from or extra in ``temp_map`` would
    keep the gate in permanent fallback with no visible error.
    """
    if not isinstance(section, Mapping):
        raise ConfigError(f"{label} must be a mapping, got {type(section).__name__}")
    for key, hint in _REMOVED_KEYS.items():
        if key in section:
            raise ConfigError(f"{label}.{key} {hint}")
    for key, replacement in _LEGACY_MAP_KEYS.items():
        if key in section:
            raise ConfigError(
                f"{label}.{key} is no longer supported; use {label}.{replacement}, one entry "
                f"per fan: 'fans: {{{_FANS_EXAMPLE}}}'"
            )
    unknown = sorted(str(k) for k in section if k not in ENTRY_KEYS and k not in ignored_keys)
    if unknown:
        raise ConfigError(f"{label}: unknown key(s) {unknown}; allowed: {list(ENTRY_KEYS)}")
    device = section.get("device")
    if device is None:
        raise ConfigError(f"{label}.device is required: one of {sorted(KINDS)}")
    if not isinstance(device, str) or device not in KINDS:
        raise ConfigError(f"{label}.device must be one of {sorted(KINDS)}, got {device!r}")
    kind = KINDS[device]
    serial = section.get("serial")
    if serial is not None and (not isinstance(serial, str) or not serial.strip()):
        raise ConfigError(
            f'{label}.serial must be a non-empty string such as "12345-54321", got {serial!r}'
        )
    pwm_map, fan_map = _parse_fans(section, label, kind)
    temp_section = section.get("temp_map")
    if temp_section is not None and not isinstance(temp_section, Mapping):
        raise ConfigError(f"{label}.temp_map must be a mapping, got {type(temp_section).__name__}")
    temp_map: dict[str, str] = {}
    temp_seen: dict[Any, str] = {}
    for name, value in dict(temp_section or {}).items():
        temp_map[name] = _temperature_input(f"{label}.temp_map.{name}", value, kind)
        _claim_input(temp_seen, temp_map[name], name, f"{label}.temp_map")
    problem = aquabus_binding_problem(kind, pwm_map, fan_map, temp_map)
    if problem is not None:
        raise ConfigError(f"{label}.temp_map: {problem}")
    if channels is not None:
        _check_keys(f"{label}.fans", pwm_map, channels, "mpc.channels")
    if temps is not None:
        _check_keys(f"{label}.temp_map", temp_map, temps, "mpc.temps")
    timing = AquacomputerTiming.from_section(section, label, kind)
    return DeviceBinding(
        kind=kind,
        pwm_map=pwm_map,
        fan_map=fan_map,
        temp_map=temp_map,
        serial=None if serial is None else serial.strip(),
        timing=timing,
    )


def build_adapter_from_config(
    section: Mapping[str, Any],
    *,
    label: str = "xt6",
    channels: Sequence[str] | None = None,
    temps: Sequence[str] | None = None,
    clock: Callable[[], float] = time.monotonic,
    sleep: Callable[[float], None] = time.sleep,
    opener: Opener | None = None,
    watchdog_s: float | None = None,
    dt: float | None = None,
    step_bound_s: float | None = None,
) -> AquacomputerAdapter:
    """``--source xt6``: the single-device ``xt6:`` section (``prefer`` ignored).
    Opens nothing; the first ``read()`` / ``apply()`` does. ``watchdog_s`` (the
    systemd watchdog, ``None`` without one) is checked by :func:`check_watchdog`
    with ``dt`` and ``step_bound_s``."""
    binding = parse_device_section(
        section, label=label, channels=channels, temps=temps, ignored_keys=("prefer",)
    )
    check_watchdog([(label, binding.timing)], watchdog_s, dt=dt, step_bound_s=step_bound_s)
    return AquacomputerAdapter(binding, clock=clock, sleep=sleep, opener=opener)
