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
    aquabus): ``read()`` raises naming it (the loop runs its fallback), lists
    it in ``absent_channels`` and logs one error when that list changes. The
    check uses the newest status report alone; the aquaero's own outputs 1-4
    are not checked. ``apply()`` writes such an output with the others and
    does not raise for it: the loop rate limits its fallback ramp against the
    last command whose ``apply()`` succeeded, so a write that went out and
    then raised would hold every fan below ``fallback_pwm`` for as long as the
    slot stays empty (PROJECT.md section 8 item 90).

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

Keeping the cache honest
    Speed, duty, voltage, current and power arrive in every status report, so
    drift of the fans themselves is visible without a control read. What a
    one-time read misses is a configuration changed behind the daemon's back
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
       its own (1-4) not in PWM mode gets one warning per open, an aquabus
       output (5-8, mode word not interpreted) none, an unconfigured block
       (source ``0xFFFF``, mode 0) one warning per adapter that writing it is
       unverified; the mode is never written;
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
    ChannelSnapshot,
    DeviceKind,
    ReportError,
    StatusReport,
    capture_channel,
    channel_holds,
    channel_state,
    check_control_report,
    decode_status,
    finalize_control_report,
    is_status_report,
    patch_duties,
    restore_channel,
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
    """Operator-tunable timing of one device. Every default is declared here once,
    ``ctrl_gap_ms`` per kind in :data:`KIND_TIMING_DEFAULTS`; build one with
    :meth:`for_kind` (documented in both example configs and PROJECT.md section 3)."""

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

    @classmethod
    def for_kind(cls, kind: DeviceKind | str, **overrides: Any) -> AquacomputerTiming:
        """The defaults for ``kind`` with ``overrides`` applied."""
        name = kind if isinstance(kind, str) else kind.name
        return cls(**{**KIND_TIMING_DEFAULTS[name], **overrides})

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


@dataclass(frozen=True)
class DeviceBinding:
    """Which device, and where the logical names live on it.

    ``pwm_map`` is channel -> output number (``pwmN``), ``fan_map`` channel ->
    tachometer number (``fanN``; keys must be ``pwm_map`` channels), ``temp_map``
    logical temperature -> the kind's temperature input name (``temp1``,
    ``bus2``, ``soft1``, ``virt1``). Numbers are 1-based as in the config.
    ``timing`` defaults to :meth:`AquacomputerTiming.for_kind`.
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
        for name, value in self.temp_map.items():
            if value not in kind.temp_names:
                raise ValueError(
                    f"{name!r}: {kind.name} has temperature inputs {kind.describe_temps()}, "
                    f"not {value!r}"
                )
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

    @property
    def label(self) -> str:
        return self.kind.name if self.serial is None else f"{self.kind.name} {self.serial}"


# ---------------------------------------------------------------------------
# Adapter
# ---------------------------------------------------------------------------


class _BudgetSpent(Exception):
    """``ctrl_budget_s`` ran out before ``operation`` could start."""

    def __init__(self, operation: str) -> None:
        super().__init__(operation)
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
        #: The output modes were checked on this open's first control report.
        self._modes_checked = False
        #: Unconfigured aquaero blocks already reported (once per adapter).
        self._unconfigured_reported: set[int] = set()
        #: Output numbers that belong to a device on the aquaero's aquabus.
        self._aquabus = frozenset(binding.kind.aquabus_outputs)
        #: The absent inputs read() found last ("pwm5 (qd1)", ...), logged when it changes.
        self._absent_inputs: tuple[str, ...] = ()
        self._absent_channels: tuple[str, ...] = ()
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
        behind it (rpm ``0xFFFF``) in the status report the last ``read()`` used."""
        return self._absent_channels

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
        self._modes_checked = False
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

    def _check_absent(self, status: StatusReport) -> None:
        """Updates ``absent_channels`` from ``status`` (one log line when it changes) and
        raises :class:`DeviceUnavailable` while it is not empty."""
        b = self.binding
        found = [
            (f"{role}{n} ({ch})", ch)
            for role, mapping in (("pwm", b.pwm_map), ("fan", b.fan_map))
            for ch, n in sorted(mapping.items(), key=lambda item: item[1])
            if self._empty_slot(status, n)
        ]
        inputs = tuple(name for name, _ in found)
        message = (
            f"{b.label}: no device behind {', '.join(inputs)}: the status report's fan block "
            "reads rpm 0xFFFF (nothing on the aquaero's aquabus)"
        )
        if inputs != self._absent_inputs:
            if inputs:
                _LOG.error(
                    "%s; reads fail until a device is behind it, writes still go out "
                    "(PROJECT.md section 8 item 90)",
                    message,
                )
            else:
                _LOG.info(
                    "%s: a device is behind %s again",
                    b.label,
                    ", ".join(self._absent_inputs),
                )
            self._absent_inputs = inputs
            self._absent_channels = tuple(sorted({ch for _, ch in found}))
        if inputs:
            raise DeviceUnavailable(message)

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
        self._check_absent(status)
        b = self.binding
        return PlantObservation(
            temps={name: status.temp(input_name) for name, input_name in b.temp_map.items()},
            rpm={ch: float(status.rpm(n)) for ch, n in b.fan_map.items()},
            pwm={ch: status.duty(n) / DUTY_MAX for ch, n in b.pwm_map.items()},
            ts=now,
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
                continue  # no device behind it: no duty evidence, read() raises instead
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

    def _adopt(self, data: bytes) -> list[int]:
        """Takes a fresh control report as the cache. Returns the channels whose held
        duty is no longer the one this adapter knew (external changes)."""
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
        is no error here: ``read()`` raises for it, which runs the loop's
        fallback, and ``absent_channels`` lists it (module docstring, Reading).
        """
        duties = self._duties(cmd)
        never_lower = getattr(cmd, "mode", None) in _NEVER_LOWER_MODES
        self._with_retries(
            "control report write",
            lambda transport, deadline: self._apply_once(transport, duties, deadline, never_lower),
        )

    def _warn_about_modes(self, ctrl: bytes) -> None:
        """One warning per open for every commanded output of the aquaero's own not in
        PWM mode, and one per adapter for every commanded unconfigured block. The mode
        is only reported, never changed (PROJECT.md section 8 items 81, 85)."""
        self._modes_checked = True
        for k in sorted(self._names):
            state = channel_state(self.kind, ctrl, k)
            mode = state.mode
            if mode is None:
                continue
            if state.unconfigured:
                if k not in self._unconfigured_reported:
                    self._unconfigured_reported.add(k)
                    _LOG.warning(
                        "%s: pwm%d (%s): controller block %d is unconfigured (source 0x%04X, "
                        "mode word 0x%04X); it is written like the others, which is not "
                        "verified on hardware (PROJECT.md section 8 item 85)",
                        self.binding.label,
                        k + 1,
                        self._names[k],
                        k + 1,
                        state.source,
                        mode.raw,
                    )
                continue
            if state.aquabus or mode.is_pwm:
                continue  # an aquabus output's mode word is not interpreted
            _LOG.warning(
                "%s: pwm%d (%s) is in %s mode (mode word 0x%04X), not PWM; the daemon does not "
                "change the mode, set it with the controller's own software",
                self.binding.label,
                k + 1,
                self._names[k],
                "DC voltage" if mode.name == "dc" else "an unknown",
                mode.raw,
            )

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
        if not self._modes_checked and ctrl is not None:
            self._warn_about_modes(ctrl)
        assert ctrl is not None
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
            f"got {value!r}"
        )
    return number


def _flow_hint(value: Any) -> str:
    """The hint for a flow sensor name where a tachometer or temperature is expected."""
    if isinstance(value, str) and re.fullmatch(r"flow[1-9][0-9]*", value):
        return (
            f"; {value!r} is a flow sensor, and flow sensors cannot be bound in the config "
            "(PROJECT.md section 8 item 91)"
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
            "flow sensors cannot be bound in the config (PROJECT.md section 8 item 91)"
        )
    raise ConfigError(
        f"{where} must be one of the {kind.name}'s tachometers fan1..fan{kind.fan_count}, "
        f"got {value!r}{_flow_hint(value)}"
    )


def _temperature_input(where: str, value: Any, kind: DeviceKind) -> str:
    if isinstance(value, str) and value in kind.temp_names:
        return value
    hint = _flow_hint(value)
    renamed = _HWMON_ERA_TEMPS[kind.name].get(value) if isinstance(value, str) else None
    if renamed is not None:
        group = next(g for g in kind.temp_groups if renamed.startswith(g.prefix))
        hint = (
            f"; in the hwmon driver's numbering {value!r} is {renamed!r} (the {kind.name}'s "
            f"{group.description}): use {renamed!r}"
        )
    raise ConfigError(
        f"{where} must be one of the {kind.name}'s temperature inputs {kind.describe_temps()}, "
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
