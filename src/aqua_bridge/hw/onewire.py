"""DS18B20 temperatures over the Linux ``w1_therm`` sysfs ABI.

PROJECT.md section 3 (Track B), section 8 items 38 and 39 (the measurements
below) and section 12 risk 5 ("bulk 1-Wire on a busy single core").

Sysfs layout, confirmed on the board (kernel 6.18.50, two externally powered
DS18B20; kept name-based and rooted at ``onewire.root`` so a different kernel
version is a config change, not a code change)::

    <root>/w1_bus_master<N>/therm_bulk_read        write "trigger", read "0"|"-1"|"1"
    <root>/w1_bus_master<N>/<rom_id>/temperature   read-only, millidegrees C
    <root>/w1_bus_master<N>/<rom_id>/resolution    read/write, 9..12 (bits)
    <root>/w1_bus_master<N>/<rom_id>/conv_time     read/write, ms per conversion

``<N>`` does **not** follow the order of the ``dtoverlay=w1-gpio`` lines in
``config.txt`` (on the board the GPIO 4 bus enumerates as
``w1_bus_master2``), and ``therm_bulk_read`` exists on a master only once its
first ``w1_therm`` slave has attached. Nothing here may assume either:
``onewire.buses:`` in config is documentation only, this module discovers
every ``w1_bus_master*`` directory under ``root`` and, on every cycle, which
of the *declared* ROM ids currently live under each one. A declared ROM id
that is not found under any bus master is a warning at :meth:`W1Source.start`
(a sensor may legitimately be unplugged with its drive, plan section 1
"Failure and redundancy"), never fatal: its readings are simply ``None``
until it (re)appears. Undeclared slaves are ignored, which is also what
keeps the family-``00`` phantoms an unterminated bus manufactures on every
kernel search out of the readings (section 9 "Overlays and modules").

**Reading strategy (section 8 item 38).** Two paths exist and the driver is
asked, never assumed, which one it gives for a given bus:

* *bulk* -- write ``trigger\n`` to the master's ``therm_bulk_read``, then
  read every slave's ``temperature``, which returns the scratchpad of that
  one conversion instead of starting its own. The kernel's implementation is
  **synchronous**: the ``write()`` resets the bus, sends Skip ROM + Convert
  T and sleeps the conversion out inside the syscall, so by the time it
  returns the status is already ``1``. One conversion per cycle regardless
  of sensor count, plus ~19 ms of scratchpad read per sensor. Measured, two
  sensors at 12 bit: 780 ms in the write, 38 ms of reads, 818 ms total
  against 1600 ms serial.
* *serial* -- read every slave's ``temperature`` with no trigger at all.
  Each read starts, and blocks in the kernel for, its own conversion:
  ``conv_time`` plus about 40 ms of bit-banging and sysfs overhead per
  sensor (measured; see :data:`_DEFAULT_RESOLUTION_BITS`).

The trigger must be **exactly eight bytes**. ``therm_bulk_read_store()``
guards on ``size == sizeof(BULK_TRIGGER_CMD)``, and that ``sizeof`` counts
the string literal's terminating NUL, so ``echo trigger >`` passes (seven
letters plus a newline) and a bare seven-byte ``trigger`` does not. A
rejected trigger is **silent**: the store function returns ``size``
regardless, so the write looks successful and only the kernel log says
``unable to trigger a bulk read on the bus. err=-22``. This module wrote
seven bytes until section 8 item 38, which is why the bulk path looked
worthless.

Per the ``w1_therm`` ABI ``therm_bulk_read`` reads ``0`` when no bulk
conversion is pending, ``-1`` while at least one sensor is still converting
and ``1`` when the conversion is complete and at least one value has not
been read yet. So a trigger that registered is observable, and once:
:meth:`W1Source.run_bus_cycle` reads the attribute, writes the trigger,
reads it again, and expects ``1``. ``-1`` (an implementation that returns
before the conversion, which the synchronous one never does) is polled out
at ``poll_interval_s``, bounded by ``bulk_timeout_s``. Anything else --
the file missing, a value outside the ABI, a write that fails (``EACCES``
when the udev rule of ``deploy/99-w1-therm.rules`` has not applied), a
trigger that leaves the attribute at ``0``, or a conversion unfinished
inside the timeout -- drops that bus to serial reads, logged with the
reason, and **the same cycle finishes with serial reads so no sample is
lost**. Nothing here waits on a signal that may never come, and the one
bounded wait's timeout is a config key.

That fallback also covers two upstream defects the kernel reports only to
its own log (section 8 item 38):

* ``bulk_read_device_counter`` is a file-scope global, so
  ``therm_bulk_read`` is created on **one master system-wide** -- whichever
  owns the first bulk-capable slave to attach anywhere. A second 1-Wire bus
  has no bulk control at all and is always read serially. This is why
  ``resolution_bits`` defaults to a value that fits the *serial* path.
* One slave with ``family_data == NULL`` on a master makes every trigger a
  no-op returning ``-ENODEV``. The family-``00`` phantoms an unterminated
  bit-banged bus manufactures on every kernel search are exactly such
  slaves, and they come and go, so a bus dropped to serial reads is
  re-probed every ``bulk_retry_s`` rather than written off for good. A
  failed probe costs one write that returns immediately and two reads.

One reader thread per bus master runs :meth:`W1Source.run_bus_cycle` in a
loop. A read that raises ``OSError`` (the kernel reports EIO for a failed
CRC) or does not parse as an integer becomes ``None`` for that sensor this
cycle, counted in :meth:`crc_error_counts` for
``tools/w1_commission.py --check``.

:meth:`W1Source.read` (called from the control loop thread) never touches
the filesystem and never blocks: it returns the latest published sample for
every declared sensor, or ``None`` when the sensor has never reported, its
last read failed, or the sample is older than ``max_age_s``.

This module must not import :mod:`aqua_bridge.control` (or anything MPC) --
see the static AST check in ``tests/test_hw_imports.py``. It imports only
:mod:`aqua_bridge.model` (for :class:`~aqua_bridge.model.ConfigError`), per
the plan's Track B constraint.
"""

from __future__ import annotations

import logging
import math
import threading
import time
from collections.abc import Callable, Mapping
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from aqua_bridge.model import ConfigError

__all__ = ["W1Source", "build_onewire_from_config"]

_LOG = logging.getLogger("aqua_bridge.hw.onewire")

DEFAULT_ROOT = "/sys/bus/w1/devices"
_BUS_GLOB = "w1_bus_master*"
_TRIGGER_FILE = "therm_bulk_read"
_RESOLUTION_FILE = "resolution"
_CONV_TIME_FILE = "conv_time"
_TEMPERATURE_FILE = "temperature"
#: ``therm_bulk_read`` read values, per the kernel's ``w1_therm`` ABI.
_BULK_IDLE = "0"  # no bulk conversion pending
_BULK_RUNNING = "-1"  # at least one sensor still converting
_BULK_DONE = "1"  # complete, at least one value not read yet
_BULK_STATES = (_BULK_IDLE, _BULK_RUNNING, _BULK_DONE)
#: Exactly eight bytes, and the newline is load-bearing: the kernel's store
#: function accepts only ``size == sizeof("trigger")``, NUL included (module
#: docstring). Seven bytes are rejected silently.
_BULK_TRIGGER = b"trigger\n"

_VALID_RESOLUTIONS = (9, 10, 11, 12)
#: Measured on the board, two sensors, per sensor:
#:
#: ============  =========  =============  ===================
#: resolution    conv_time  serial / sens  bulk scratchpad read
#: ============  =========  =============  ===================
#: 12 bit        750 ms     800 ms         19 ms
#: 11 bit        375 ms     416 ms         --
#: 10 bit        190 ms     228 ms         17 ms
#: 9 bit          95 ms     132 ms         --
#: ============  =========  =============  ===================
#:
#: A serial cycle is ``n * (conv_time + ~40 ms)``; a bulk cycle is
#: ``conv_time + n * ~19 ms``, one conversion for the whole bus.
#:
#: The budget is ``max_age_s / 2`` per bus (default ``0.75 * dt`` = 3.75 s at
#: ``dt = 5 s``): a cycle that fits it refreshes every sensor twice inside
#: ``max_age_s``, so one lost cycle never ages a sensor out of ``read()``.
#: For the planned 24 sensors on two buses -- 12 per bus, read in parallel by
#: their own threads -- and given that only one master system-wide ever gets a
#: ``therm_bulk_read`` (module docstring), the default has to fit the bus that
#: cannot have it: serially, 12 sensors cost 9.6 s at 12 bit and 5.0 s at 11,
#: both over the budget and both over a whole tick, against 2.74 s at 10 bit
#: (0.55 dt, 27 % of margin) and 1.58 s at 9. So 10 bit, the finest resolution
#: that fits the serial bus; the bulk-capable bus then costs 0.39 s and would
#: fit 12 bit (0.98 s) on its own.
#:
#: 9 bit is not worth its 0.5 C step: one LSB would equal the estimator's
#: ``jump_min_c`` and double the standing offset item 40 is sensitive to. At 10
#: bit a serial bus takes 16 sensors inside the budget at ``dt = 5 s`` (6 at
#: ``dt = 2 s``); the quantisation the estimator carries is
#: ``quant_c ** 2 / 12``, sigma 0.072 C, still seven times under the
#: DS18B20's own +/-0.5 C accuracy (PROJECT.md section 8 item 39).
_DEFAULT_RESOLUTION_BITS = 10
_DEFAULT_POLL_INTERVAL_S = 0.02
# The kernel's bulk read sleeps the conversion out inside write(), so the status
# is 1 by the time the write returns and this timeout is never reached there. It
# bounds the one case the ABI leaves open -- an implementation that reports -1
# and finishes later -- with room for the 750 ms of a 12-bit conversion and a
# slow bus, while still giving up well inside one tick at the recommended
# dt = 5 s.
_DEFAULT_BULK_TIMEOUT_S = 2.0
# How long a bus that failed the bulk probe reads serially before it is probed
# again. A family-00 phantom anywhere on a master makes every trigger a no-op
# (module docstring) and phantoms come and go, so the drop must not be
# permanent; a failed probe costs one write that returns at once and two reads,
# so retrying is cheap, and 5 minutes is far longer than a search interval.
_DEFAULT_BULK_RETRY_S = 300.0
#: ``onewire.bulk_read``: ``auto`` probes each bus as described in the module
#: docstring and uses bulk only where the driver answers; ``off`` never writes
#: ``therm_bulk_read`` at all.
_BULK_READ_MODES = ("auto", "off")
_DEFAULT_BULK_READ = "auto"


@dataclass
class _Sample:
    value: float | None
    ts: float


class W1Source:
    """DS18B20 temperatures over one or more ``w1_therm`` bus masters.

    Parameters
    ----------
    sensors:
        Logical name -> 1-Wire ROM id (e.g. ``{"prox_b01": "28-0316a27a0aff"}``).
        A ROM id may be shared by more than one logical name (redundant
        sensors, plan section 1); every listed name then gets the same
        reading.
    resolution_bits:
        Written once to each slave's ``resolution`` file the first time it
        is seen (never rewritten after that -- not every cycle: a scratchpad
        write is not free and the value does not change on its own). The
        driver's own ``conv_time`` is read back afterwards and reported by
        :meth:`conv_time_ms`.
    max_age_s:
        :meth:`read` reports ``None`` for a sensor whose latest sample is
        older than this.
    root:
        Root of the w1 sysfs tree. Overridable so tests point it at a fake
        tree under ``tmp_path``.
    clock:
        Zero-argument callable returning monotonic seconds, used both to
        stamp published samples and to judge staleness in :meth:`read` --
        injected so tests control time without sleeping.
    bulk_read:
        ``auto`` (probe each bus, module docstring) or ``off`` (never touch
        ``therm_bulk_read``).
    bulk_timeout_s:
        Bound on the only wait in a cycle: how long a bus that reports a
        bulk conversion still running is given to finish it before it drops
        to serial reads.
    bulk_retry_s:
        How long a bus that failed the bulk probe reads serially before it
        is probed again.
    """

    def __init__(
        self,
        sensors: Mapping[str, str],
        *,
        resolution_bits: int = _DEFAULT_RESOLUTION_BITS,
        max_age_s: float,
        root: str | Path = DEFAULT_ROOT,
        clock: Callable[[], float] = time.monotonic,
        poll_interval_s: float = _DEFAULT_POLL_INTERVAL_S,
        bulk_timeout_s: float = _DEFAULT_BULK_TIMEOUT_S,
        bulk_retry_s: float = _DEFAULT_BULK_RETRY_S,
        bulk_read: str = _DEFAULT_BULK_READ,
    ) -> None:
        if not sensors:
            raise ConfigError("onewire.sensors must not be empty")
        if resolution_bits not in _VALID_RESOLUTIONS:
            raise ConfigError(
                f"onewire.resolution_bits must be one of {_VALID_RESOLUTIONS}, "
                f"got {resolution_bits}"
            )
        if not max_age_s > 0:
            raise ConfigError(f"onewire.max_age_s must be > 0, got {max_age_s}")
        if not poll_interval_s > 0:
            raise ConfigError(f"onewire poll_interval_s must be > 0, got {poll_interval_s}")
        if not bulk_timeout_s > 0:
            raise ConfigError(f"onewire bulk_timeout_s must be > 0, got {bulk_timeout_s}")
        if not bulk_retry_s > 0:
            raise ConfigError(f"onewire bulk_retry_s must be > 0, got {bulk_retry_s}")
        if bulk_read not in _BULK_READ_MODES:
            raise ConfigError(
                f"onewire.bulk_read must be one of {_BULK_READ_MODES}, got {bulk_read!r}"
            )

        self.sensors: dict[str, str] = dict(sensors)
        self._rom_to_names: dict[str, list[str]] = {}
        for name, rom in self.sensors.items():
            self._rom_to_names.setdefault(rom, []).append(name)
        self._resolution_bits = int(resolution_bits)
        self._max_age_s = float(max_age_s)
        self.root = Path(root)
        self._clock = clock
        self._poll_interval_s = float(poll_interval_s)
        self._bulk_timeout_s = float(bulk_timeout_s)
        self._bulk_retry_s = float(bulk_retry_s)
        self._bulk_read = bulk_read

        self._lock = threading.Lock()
        self._latest: dict[str, _Sample] = {}
        self._configured_resolution: set[str] = set()
        self._conv_time_ms: dict[str, int] = {}
        self._crc_errors: dict[str, int] = dict.fromkeys(self._rom_to_names, 0)
        self._cycle_counts: dict[str, int] = {}
        # Bus master name -> whether the bulk path is in use on it (absent: not
        # probed yet), and when a bus that failed may be probed again.
        self._bulk_ok: dict[str, bool] = {}
        self._bulk_retry_at: dict[str, float] = {}
        self._threads: list[threading.Thread] = []
        self._stop = threading.Event()

    # -- discovery ------------------------------------------------------------

    def discover_buses(self) -> list[Path]:
        """Every ``w1_bus_master*`` directory currently under ``root``, sorted by name."""
        if not self.root.is_dir():
            return []
        return sorted(p for p in self.root.glob(_BUS_GLOB) if p.is_dir())

    def missing_roms(self) -> list[str]:
        """Declared ROM ids not currently found under any bus master."""
        found: set[str] = set()
        for bus_dir in self.discover_buses():
            for rom in self._rom_to_names:
                if (bus_dir / rom).is_dir():
                    found.add(rom)
        return sorted(set(self._rom_to_names) - found)

    # -- one read cycle (synchronous; also used directly by tests and tools) --

    def run_bus_cycle(self, bus_dir: Path) -> dict[str, float | None]:
        """Runs one read cycle on ``bus_dir`` (bulk where the driver answers, else serial).

        Returns ``{name: value_or_None}`` for every declared sensor name
        currently found under ``bus_dir`` (empty if none of the declared ROM
        ids live there). Never raises: every failure mode narrows to
        ``None`` for the sensor(s) it affects, per the module docstring.
        """
        present = {rom: bus_dir / rom for rom in self._rom_to_names if (bus_dir / rom).is_dir()}
        if not present:
            return {}
        self._ensure_resolution(present)
        if self._bulk_read != "off" and self._bulk_probe_due(bus_dir.name):
            self._bulk_convert(bus_dir)
        result: dict[str, float | None] = {}
        for rom, slave_dir in present.items():
            value = self._read_temperature(slave_dir, rom)
            for name in self._rom_to_names[rom]:
                result[name] = value
        return result

    def bulk_read_modes(self) -> dict[str, str]:
        """Bus master name -> ``"bulk"``, ``"serial"`` or ``"unprobed"``.

        Diagnostics for ``tools/w1_commission.py --check``: which path each bus
        ended up on, so the measured cycle time can be read against it.
        """
        names = set(self._cycle_counts) | set(self._bulk_ok)
        if self._bulk_read == "off":
            return dict.fromkeys(names, "serial")
        out: dict[str, str] = dict.fromkeys(names, "unprobed")
        for name, ok in self._bulk_ok.items():
            out[name] = "bulk" if ok else "serial"
        return out

    def conv_time_ms(self) -> dict[str, int]:
        """ROM id -> the driver's own ``conv_time`` after ``resolution`` was written."""
        return dict(self._conv_time_ms)

    def _ensure_resolution(self, present: Mapping[str, Path]) -> None:
        for rom, slave_dir in present.items():
            if rom in self._configured_resolution:
                continue
            self._configured_resolution.add(rom)
            try:
                (slave_dir / _RESOLUTION_FILE).write_text(str(self._resolution_bits))
            except OSError as exc:
                # Most likely the udev rule of deploy/99-w1-therm.rules has not
                # applied: resolution is root-owned and the service user is not
                # root. The sensor still reads, at whatever resolution it has,
                # so this is a warning and not a reason to drop the bus.
                _LOG.warning("onewire: cannot set resolution for %s: %s", rom, exc)
            try:
                readback = (slave_dir / _RESOLUTION_FILE).read_text().strip()
                conv_time = int((slave_dir / _CONV_TIME_FILE).read_text().strip())
            except (OSError, ValueError) as exc:
                _LOG.debug("onewire: cannot read back resolution/conv_time for %s: %s", rom, exc)
                continue
            self._conv_time_ms[rom] = conv_time
            if readback != str(self._resolution_bits):
                _LOG.warning(
                    "onewire: %s reports resolution %s bits, not the configured %d",
                    rom,
                    readback,
                    self._resolution_bits,
                )
            _LOG.debug("onewire: %s at %s bits, conv_time %d ms", rom, readback, conv_time)

    def _bulk_probe_due(self, bus_name: str) -> bool:
        """Whether this cycle may trigger a bulk read on ``bus_name``."""
        if self._bulk_ok.get(bus_name, True):
            return True
        return self._clock() >= self._bulk_retry_at.get(bus_name, 0.0)

    def _demote_bulk(self, bus_dir: Path, reason: str) -> None:
        first = self._bulk_ok.get(bus_dir.name) is not False
        self._bulk_ok[bus_dir.name] = False
        self._bulk_retry_at[bus_dir.name] = self._clock() + self._bulk_retry_s
        # Once loudly, then quietly: a bus whose master has no therm_bulk_read at
        # all (only one master system-wide gets one) would otherwise warn every
        # bulk_retry_s for the life of the daemon.
        log = _LOG.warning if first else _LOG.debug
        log(
            "onewire: %s reads serially for the next %.0f s (%s); PROJECT.md item 38",
            bus_dir.name,
            self._bulk_retry_s,
            reason,
        )

    def _read_bulk_state(self, trigger_path: Path) -> str | None:
        try:
            return trigger_path.read_text().strip()
        except OSError as exc:
            _LOG.debug("onewire: cannot read %s: %s", trigger_path, exc)
            return None

    def _bulk_convert(self, bus_dir: Path) -> None:
        """Triggers one bulk conversion on ``bus_dir`` and checks it completed.

        The kernel's implementation converts inside the ``write()``, so the
        status is read **once** afterwards and is expected to be ``1``; ``-1``
        is polled out under ``bulk_timeout_s`` for an implementation that
        returns early. Anything else drops the bus to serial reads (retried
        after ``bulk_retry_s``), and the caller's per-slave reads -- which then
        each run their own conversion -- still produce this cycle's readings.
        Never raises.
        """
        trigger_path = bus_dir / _TRIGGER_FILE
        before = self._read_bulk_state(trigger_path)
        if before is None:
            self._demote_bulk(bus_dir, f"{_TRIGGER_FILE} is absent or unreadable")
            return
        if before not in _BULK_STATES:
            self._demote_bulk(bus_dir, f"{_TRIGGER_FILE} reads {before!r}, not a w1_therm state")
            return
        try:
            trigger_path.write_bytes(_BULK_TRIGGER)
        except OSError as exc:
            self._demote_bulk(bus_dir, f"trigger write failed: {exc}")
            return
        state = self._read_bulk_state(trigger_path)
        if state == _BULK_IDLE:
            # The ABI's "no bulk conversion pending" after a write the kernel
            # accepted: it refused the trigger and said so only in its own log
            # (the store function returns size either way). The short payload
            # that used to cause this is fixed; what is left is a master with a
            # family_data-less slave on it (-ENODEV), i.e. a phantom.
            self._demote_bulk(
                bus_dir,
                f"{_TRIGGER_FILE} still {_BULK_IDLE!r} after the trigger: the kernel refused it, "
                "look for 'unable to trigger a bulk read' and a phantom slave in dmesg",
            )
            return
        deadline = self._clock() + self._bulk_timeout_s
        while state == _BULK_RUNNING:
            if self._stop.is_set():
                # Shutting down: leave the bus as it is (the driver finishes the
                # conversion on its own) rather than spinning out the deadline.
                return
            if self._clock() >= deadline:
                self._demote_bulk(
                    bus_dir, f"conversion unfinished after bulk_timeout_s={self._bulk_timeout_s} s"
                )
                return
            self._stop.wait(self._poll_interval_s)
            state = self._read_bulk_state(trigger_path)
        if state != _BULK_DONE:
            self._demote_bulk(bus_dir, f"{_TRIGGER_FILE} reads {state!r} mid-conversion")
            return
        if self._bulk_ok.get(bus_dir.name) is False:
            _LOG.info("onewire: %s honours the bulk trigger again", bus_dir.name)
        self._bulk_ok[bus_dir.name] = True

    def _read_temperature(self, slave_dir: Path, rom: str) -> float | None:
        try:
            raw = (slave_dir / _TEMPERATURE_FILE).read_text().strip()
            milli = int(raw)
        except (OSError, ValueError) as exc:
            self._crc_errors[rom] = self._crc_errors.get(rom, 0) + 1
            _LOG.debug("onewire: read failed for %s: %s", rom, exc)
            return None
        value = milli / 1000.0
        return value if math.isfinite(value) else None

    def _publish(self, result: Mapping[str, float | None]) -> None:
        ts = self._clock()
        with self._lock:
            for name, value in result.items():
                self._latest[name] = _Sample(value, ts)

    # -- reader threads ---------------------------------------------------------

    def start(self) -> None:
        """Spawns one daemon reader thread per discovered bus master.

        A declared ROM id missing from every bus at this point is logged as
        a warning (plan section 1: "a sensor absent from the bus at
        startup"), never raised.
        """
        if self._threads:
            return
        buses = self.discover_buses()
        if not buses:
            _LOG.warning("onewire: no w1 bus master found under %s", self.root)
        for missing in self.missing_roms():
            names = ", ".join(self._rom_to_names[missing])
            _LOG.warning("onewire: ROM %s (%s) not found under %s", missing, names, self.root)
        self._stop.clear()
        for bus_dir in buses:
            self._cycle_counts.setdefault(bus_dir.name, 0)
            thread = threading.Thread(
                target=self._run_bus_forever,
                args=(bus_dir,),
                name=f"w1-{bus_dir.name}",
                daemon=True,
            )
            self._threads.append(thread)
            thread.start()

    def _run_bus_forever(self, bus_dir: Path) -> None:
        while not self._stop.is_set():
            result = self.run_bus_cycle(bus_dir)
            if result:
                self._publish(result)
                self._cycle_counts[bus_dir.name] += 1
            # The conversions pace this loop on their own (12 sensors at the
            # default 10 bit: 2.74 s read serially, 0.39 s in bulk); this floor
            # only guards the pathological/test case of a bus with no declared
            # sensor on it, or a fake that answers instantly, so the thread
            # never busy-spins a core.
            self._stop.wait(self._poll_interval_s)

    def stop(self) -> None:
        """Stops every reader thread. Idempotent; safe to call if never started."""
        self._stop.set()
        for thread in self._threads:
            thread.join(timeout=5.0)
        self._threads.clear()

    # -- read (non-blocking, loop thread) ----------------------------------------

    def read(self) -> dict[str, float | None]:
        """Latest sample for every declared sensor name; never blocks or touches disk."""
        now = self._clock()
        with self._lock:
            out: dict[str, float | None] = {}
            for name in self.sensors:
                sample = self._latest.get(name)
                if sample is None or sample.value is None or now - sample.ts > self._max_age_s:
                    out[name] = None
                else:
                    out[name] = sample.value
            return out

    # -- diagnostics (tools/w1_commission.py) ------------------------------------

    def crc_error_counts(self) -> dict[str, int]:
        """Failed reads per ROM id since construction (``tools/w1_commission.py --check``)."""
        return dict(self._crc_errors)

    def cycle_counts(self) -> dict[str, int]:
        """Completed read cycles per bus master name since :meth:`start`."""
        return dict(self._cycle_counts)


def build_onewire_from_config(
    section: Mapping[str, Any],
    *,
    default_max_age_s: float,
    clock: Callable[[], float] = time.monotonic,
) -> W1Source | None:
    """Builds a :class:`W1Source` from the ``onewire:`` config section.

    Returns ``None`` when the section declares no sensors -- a config
    without any DS18B20 is legal (not every DAS build uses 1-Wire); the
    caller (``hw/sources.py``) then omits 1-Wire from the composite. The
    legacy placeholder ``onewire: {enabled: false, sensors: []}`` in
    ``config.example.yaml`` falls into this case (a bare list, not a
    mapping of ``{name: rom_id}``, is also treated as "no sensors" rather
    than a config error, since nothing ever read that placeholder and it
    predates this module).
    """
    if not isinstance(section, Mapping):
        raise ConfigError(f"onewire section must be a mapping, got {type(section).__name__}")
    if "enabled" in section and not isinstance(section["enabled"], bool):
        # Item 57: the key is a legacy placeholder (see the docstring above) that
        # nothing here reads to decide anything, but a string such as "true" is
        # still a config mistake worth naming rather than passing through silently.
        raise ConfigError(
            f"onewire.enabled must be true or false, got {type(section['enabled']).__name__}"
        )
    sensors = section.get("sensors")
    if not sensors:
        return None
    if not isinstance(sensors, Mapping):
        raise ConfigError(
            "onewire.sensors must be a mapping of {name: rom_id} once non-empty, got "
            f"{type(sensors).__name__}"
        )
    clean: dict[str, str] = {}
    for name, rom in sensors.items():
        if not isinstance(name, str) or not name:
            raise ConfigError(f"onewire.sensors key must be a non-empty string, got {name!r}")
        if not isinstance(rom, str) or not rom:
            raise ConfigError(
                f"onewire.sensors[{name!r}] must be a non-empty ROM id string, got {rom!r}"
            )
        clean[name] = rom

    resolution_bits = section.get("resolution_bits", _DEFAULT_RESOLUTION_BITS)
    if not isinstance(resolution_bits, int) or isinstance(resolution_bits, bool):
        raise ConfigError(
            f"onewire.resolution_bits must be an int, got {type(resolution_bits).__name__}"
        )
    max_age_s = section.get("max_age_s", default_max_age_s)
    if not isinstance(max_age_s, int | float) or isinstance(max_age_s, bool):
        raise ConfigError(f"onewire.max_age_s must be a number, got {type(max_age_s).__name__}")
    bulk_read = section.get("bulk_read", _DEFAULT_BULK_READ)
    if not isinstance(bulk_read, str):
        raise ConfigError(f"onewire.bulk_read must be a string, got {type(bulk_read).__name__}")
    bulk_timeout_s = section.get("bulk_timeout_s", _DEFAULT_BULK_TIMEOUT_S)
    if not isinstance(bulk_timeout_s, int | float) or isinstance(bulk_timeout_s, bool):
        raise ConfigError(
            f"onewire.bulk_timeout_s must be a number, got {type(bulk_timeout_s).__name__}"
        )
    bulk_retry_s = section.get("bulk_retry_s", _DEFAULT_BULK_RETRY_S)
    if not isinstance(bulk_retry_s, int | float) or isinstance(bulk_retry_s, bool):
        raise ConfigError(
            f"onewire.bulk_retry_s must be a number, got {type(bulk_retry_s).__name__}"
        )
    poll_interval_s = section.get("poll_interval_s", _DEFAULT_POLL_INTERVAL_S)
    if not isinstance(poll_interval_s, int | float) or isinstance(poll_interval_s, bool):
        raise ConfigError(
            f"onewire.poll_interval_s must be a number, got {type(poll_interval_s).__name__}"
        )
    root = section.get("root", DEFAULT_ROOT)

    try:
        return W1Source(
            clean,
            resolution_bits=resolution_bits,
            max_age_s=float(max_age_s),
            root=root,
            clock=clock,
            poll_interval_s=float(poll_interval_s),
            bulk_timeout_s=float(bulk_timeout_s),
            bulk_retry_s=float(bulk_retry_s),
            bulk_read=bulk_read,
        )
    except ConfigError as exc:
        raise ConfigError(f"onewire: {exc}") from exc
