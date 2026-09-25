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

**Reading strategy (section 8 item 38).** Three tiers exist, tried in
this order per bus, and each one's availability is a *decision from what the
kernel just did*, never a guess from config:

1. *netlink* -- one bus-wide conversion and one scratchpad read per sensor over
   the kernel's netlink connector (:mod:`aqua_bridge.hw.w1_netlink`,
   :mod:`aqua_bridge.hw.w1_therm_netlink`). Same shape as *bulk* below and the
   same cost, but it addresses a master by **id**, so it works on the bus the
   kernel never gave a ``therm_bulk_read`` to, and on a master carrying
   family-``00`` phantoms that make the sysfs trigger a silent no-op. A bus
   takes this tier when the connector answers ``W1_LIST_MASTERS`` with this
   master's id in it, every present sensor's 8-byte identifier is readable, and
   every present sensor has a known ``conv_time``; it keeps it as long as the
   conversion command and at least one scratchpad succeed. Anything else drops
   it one tier for ``netlink_retry_s``, **and the same cycle still finishes on
   the tier below, so no sample is lost**. Measured on the board, 7 sensors at
   12 bit (the owner swaps sensors in and out, so the count belongs with the
   numbers): 6 ms for the conversion command, 16 ms per scratchpad, 876 ms per
   cycle against 904 ms for sysfs bulk and 5664 ms read one at a time; at 10
   bit 316, 324 and 1604 ms.
2. *bulk* -- the kernel's own ``therm_bulk_read``, described below. It exists
   on one master system-wide and one phantom disables it.
3. *serial* -- one sensor at a time, also described below. Always available.

``onewire.read_tier`` forces one tier for debugging (``auto`` is the default
and the only production value); ``onewire.bulk_read: off`` still means "never
write ``therm_bulk_read``", which under ``auto`` leaves netlink then serial.
:meth:`W1Source.read_tiers` reports which tier produced each bus's last
numbers, and ``tools/w1_commission.py --check`` prints it next to the measured
cycle time, so a number is never read against the wrong path.

The sysfs tiers, unchanged (section 8 item 38). The driver is asked, never
assumed, which one it gives for a given bus:

* *bulk* -- write ``trigger\n`` to the master's ``therm_bulk_read``, then
  read every slave's ``temperature``, which returns the scratchpad of that
  one conversion instead of starting its own. The kernel's implementation is
  **synchronous**: the ``write()`` resets the bus, sends Skip ROM + Convert
  T and sleeps the conversion out inside the syscall, so by the time it
  returns the status is already ``1``. One conversion per cycle regardless
  of sensor count, plus ~19 ms of scratchpad read per sensor. Measured, two
  sensors at 12 bit: 773 ms in the write, 58 ms of reads, 831 ms total
  against 2400 ms serial; the whole cycle through this module is 308 ms at
  10 bit against 682 ms serial.
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
  has no bulk control at all. That used to mean it was always read serially,
  and the ``resolution_bits`` default had to fit the *serial* path; the
  netlink tier above addresses a master by id and so gives that bus a
  bus-wide conversion too, which is what let the default go back to 12 bit
  (see :data:`_DEFAULT_RESOLUTION_BITS`).
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

from aqua_bridge.hw.w1_netlink import W1Netlink, W1NetlinkError
from aqua_bridge.hw.w1_therm_netlink import (
    CONVERSION_TIME_S,
    ScratchpadError,
    W1Therm,
    reg_num_from_rom_name,
    rom_name_from_reg_num,
)
from aqua_bridge.model import ConfigError

__all__ = ["W1Source", "build_onewire_from_config"]

_LOG = logging.getLogger("aqua_bridge.hw.onewire")

DEFAULT_ROOT = "/sys/bus/w1/devices"
_BUS_PREFIX = "w1_bus_master"
_BUS_GLOB = f"{_BUS_PREFIX}*"
_TRIGGER_FILE = "therm_bulk_read"
_RESOLUTION_FILE = "resolution"
_CONV_TIME_FILE = "conv_time"
_TEMPERATURE_FILE = "temperature"
#: A slave's raw ``struct w1_reg_num``: the 8 bytes a netlink slave command
#: addresses it by (``drivers/w1/w1.c``, ``id_show``).
_ID_FILE = "id"
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
#: Measured on the board, per sensor (three sensors):
#:
#: ============  =========  =============  ====================
#: resolution    conv_time  serial / sens  bulk scratchpad read
#: ============  =========  =============  ====================
#: 12 bit        750 ms     800 ms         19.5 ms
#: 11 bit        375 ms     415 ms         19.5 ms
#: 10 bit        190 ms     227 ms         16.9 ms
#: 9 bit          95 ms     131 ms         18.8 ms
#: ============  =========  =============  ====================
#:
#: A serial cycle is ``n * (conv_time + ~40 ms)``; a bus-wide cycle (netlink or
#: the kernel's bulk read) is one conversion for the whole bus plus ~17-20 ms
#: per sensor. Re-measured through this module, eight sensors on one bus, cycle
#: time fitted against the sensor count over n = 2, 4, 6, 8:
#:
#: ============  =====================  =====================
#: tier          12 bit                 10 bit
#: ============  =====================  =====================
#: netlink       764 + 16.6 n ms        203 + 16.3 n ms
#: sysfs bulk    759 + 20.5 n ms        211 + 15.9 n ms
#: serial        800 n ms               228 n ms
#: ============  =====================  =====================
#:
#: The budget is ``max_age_s / 2`` per bus (default ``0.75 * dt`` = 3.75 s at
#: ``dt = 5 s``): a cycle that fits it refreshes every sensor twice inside
#: ``max_age_s``, so one lost cycle never ages a sensor out of ``read()``. For
#: the planned 24 sensors on two buses -- 12 per bus, read in parallel by their
#: own threads -- 12 bit costs **0.96 s** over netlink and **1.00 s** over the
#: kernel's bulk read: about a quarter of the budget, 0.2 of a tick. Both tiers
#: are bus-wide, and netlink addresses a master by id, so it serves the second
#: bus too -- the one the kernel never gives a ``therm_bulk_read`` to (module
#: docstring). That is what the default rests on, and it is why the default is
#: the finest step the sensor has.
#:
#: **It does not fit the serial tier**, and that is deliberate, not overlooked:
#: 12 sensors read one at a time cost 9.6 s at 12 bit, 2.6x the budget and
#: longer than ``max_age_s`` itself, so a bus demoted that far publishes each
#: sensor less often than ``read()`` will accept it and the bus reads as
#: missing on roughly a fifth of the ticks (missing raises cooling and never
#: lowers it -- PROJECT.md section 2 -- and ``read()`` still never blocks, so
#: the tick is never held up). Serial is the floor under two bus-wide tiers,
#: reached only when netlink fails *and* the kernel's bulk read is absent or
#: refused, and then only for ``netlink_retry_s``. An installation whose kernel
#: has no ``w1`` connector at all has no netlink tier on any bus and should set
#: ``onewire.resolution_bits`` to 10 (2.7 s serial, 0.73 of the budget) or 9
#: (1.6 s); ``tools/w1_commission.py --check`` prints the tier each bus got
#: beside its measured cycle time, which is how that is noticed rather than
#: guessed (PROJECT.md section 8 items 38 and 39).
#:
#: The step itself is why 12 bit is worth its conversion: 0.0625 C, which the
#: estimator carries as ``quant_c ** 2 / 12``, sigma 0.018 C, and the Stuck
#: rule as ``1.5 * quant_c`` = 0.094 C, comfortably above one LSB so an idle
#: sensor's own dither clears the band. Coarser steps stay one config key away:
#: 0.125 C at 11 bit, 0.25 at 10, 0.5 at 9 -- and at 9 bit one LSB would equal
#: the estimator's ``jump_min_c`` and put the standing error at 0.25 C, 2.5x
#: the offset item 40 is sensitive to. Whichever is chosen,
#: ``sensors.<name>.quant_c`` has to be moved with it: nothing cross-checks the
#: two sections.
_DEFAULT_RESOLUTION_BITS = 12
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
#: ``onewire.read_tier``: the tier ladder's top. ``auto`` is the full ladder
#: (netlink, then the kernel's bulk read, then one sensor at a time); the other
#: three name one tier and forbid the rest, which is a debugging aid -- a bus
#: that cannot run the tier it was pinned to publishes nothing that cycle, and
#: a sensor with no sample reads as missing, which the control loop turns into
#: more cooling and never less (PROJECT.md section 3).
_READ_TIERS = ("auto", "netlink", "sysfs_bulk", "serial")
_DEFAULT_READ_TIER = "auto"
#: Tier names as :meth:`W1Source.read_tiers` reports them.
_TIER_NETLINK = "netlink"
_TIER_BULK = "bulk"
_TIER_SERIAL = "serial"
_TIER_UNPROBED = "unprobed"
_TIER_NONE = "none"
# Bound on one netlink request. A request of ours costs the bus 6 ms (the
# conversion command) to 16 ms (a scratchpad), but the kernel serialises every
# path on the master's bus_mutex, so a sysfs reader that got there first can
# hold it for a whole conversion -- 750 ms at 12 bit. One second covers that
# plus scheduling on a loaded single core and still gives up well inside one
# tick at the recommended dt = 5 s. A cycle makes one request plus one per
# sensor, and the first request that goes unanswered ends the cycle
# (hw/w1_therm_netlink.py, read_bus), so the worst case is about two of these
# and not one per sensor.
_DEFAULT_NETLINK_TIMEOUT_S = 1.0
# How long a bus that failed the netlink tier stays on the tier below before it
# is tried again. Same argument as bulk_retry_s: what takes the tier away
# (a sensor pulled out mid-cycle, a kernel busy elsewhere) can go away again,
# and a failed probe costs one command that returns in milliseconds.
_DEFAULT_NETLINK_RETRY_S = 300.0


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
        9..12; 12 by default (:data:`_DEFAULT_RESOLUTION_BITS` for the
        arithmetic and what a coarser one buys). Written once to each slave's
        ``resolution`` file the first time it is seen (never rewritten after
        that -- not every cycle: a scratchpad write is not free and the value
        does not change on its own), so it is also the resolution a bus keeps
        while it is demoted down the tier ladder. The driver's own
        ``conv_time`` is read back afterwards and reported by
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
    read_tier:
        ``auto`` (the full ladder, module docstring) or one tier name, which
        forbids the others. Debugging only.
    netlink_timeout_s:
        Bound on one netlink request. Every wait on that socket is bounded by
        it and a request that goes unanswered ends the cycle.
    netlink_retry_s:
        How long a bus that failed the netlink tier stays on the tier below
        before it is tried again.
    netlink_factory:
        Called with no arguments to build one
        :class:`~aqua_bridge.hw.w1_netlink.W1Netlink` per bus master. Injected
        so tests drive the whole ladder without opening a socket.
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
        read_tier: str = _DEFAULT_READ_TIER,
        netlink_timeout_s: float = _DEFAULT_NETLINK_TIMEOUT_S,
        netlink_retry_s: float = _DEFAULT_NETLINK_RETRY_S,
        netlink_factory: Callable[[], W1Netlink] | None = None,
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
        if read_tier not in _READ_TIERS:
            raise ConfigError(f"onewire.read_tier must be one of {_READ_TIERS}, got {read_tier!r}")
        if not netlink_timeout_s > 0:
            raise ConfigError(f"onewire netlink_timeout_s must be > 0, got {netlink_timeout_s}")
        if not netlink_retry_s > 0:
            raise ConfigError(f"onewire netlink_retry_s must be > 0, got {netlink_retry_s}")

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
        self._read_tier = read_tier
        self._netlink_timeout_s = float(netlink_timeout_s)
        self._netlink_retry_s = float(netlink_retry_s)
        self._netlink_factory = netlink_factory

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
        # The same, for the netlink tier, plus the socket each bus reads over
        # (one per reader thread: a netlink socket carries one exchange at a
        # time), the identifiers slaves are addressed by, the resolution each
        # sensor last reported about itself, and the tier each bus's last
        # numbers came from.
        self._netlink_ok: dict[str, bool] = {}
        self._netlink_retry_at: dict[str, float] = {}
        self._transports: dict[str, W1Netlink] = {}
        self._slave_ids: dict[str, bytes] = {}
        self._observed_bits: dict[str, int] = {}
        self._tier: dict[str, str] = {}
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
        """Runs one read cycle on ``bus_dir``, on the best tier that bus answers on.

        Returns ``{name: value_or_None}`` for every declared sensor name
        currently found under ``bus_dir`` (empty if none of the declared ROM
        ids live there, if the bus is pinned to a tier it cannot run, or if
        :meth:`stop` arrived mid-conversion). Never raises: every failure mode
        narrows to ``None`` for the sensor(s) it affects or to a tier below,
        per the module docstring.
        """
        present = {rom: bus_dir / rom for rom in self._rom_to_names if (bus_dir / rom).is_dir()}
        if not present:
            self._tier[bus_dir.name] = _TIER_NONE
            return {}
        self._ensure_resolution(present)
        if self._netlink_due(bus_dir.name):
            netlink_result = self._netlink_cycle(bus_dir, present)
            if netlink_result is not None:
                return netlink_result
        if self._read_tier == _TIER_NETLINK:
            # Pinned to netlink and this bus cannot run it: publish nothing
            # rather than quietly reading it another way. Missing samples read
            # as missing, which raises cooling and never lowers it.
            self._tier[bus_dir.name] = _TIER_NONE
            return {}
        if (
            self._bulk_read != "off"
            and self._read_tier != _TIER_SERIAL
            and self._bulk_probe_due(bus_dir.name)
        ):
            self._bulk_convert(bus_dir)
        result: dict[str, float | None] = {}
        for rom, slave_dir in present.items():
            value = self._read_temperature(slave_dir, rom)
            for name in self._rom_to_names[rom]:
                result[name] = value
        self._tier[bus_dir.name] = self._sysfs_tier(bus_dir.name)
        return result

    # -- the netlink tier -------------------------------------------------------

    def _netlink_due(self, bus_name: str) -> bool:
        """Whether this cycle may try the netlink tier on ``bus_name``."""
        if self._read_tier not in ("auto", _TIER_NETLINK):
            return False
        if self._netlink_ok.get(bus_name, True):
            return True
        return self._clock() >= self._netlink_retry_at.get(bus_name, 0.0)

    def _sysfs_tier(self, bus_name: str) -> str:
        if self._bulk_read == "off" or self._read_tier == _TIER_SERIAL:
            return _TIER_SERIAL
        return _TIER_BULK if self._bulk_ok.get(bus_name) else _TIER_SERIAL

    def _netlink_cycle(
        self, bus_dir: Path, present: Mapping[str, Path]
    ) -> dict[str, float | None] | None:
        """One netlink cycle on ``bus_dir``, or ``None`` if this bus cannot run it.

        ``None`` means "fall through to the tier below, this same cycle" and
        drops the bus for ``netlink_retry_s``; an empty dict means the cycle was
        abandoned (:meth:`stop` during the conversion) and nothing is published.
        Never raises.
        """
        bus_name = bus_dir.name
        interrupted: list[bool] = []
        try:
            # What this host can know on its own first, so a bus that cannot run
            # the tier costs no traffic at all: the identifiers, then the wait.
            targets = {rom: self._slave_id_of(slave_dir, rom) for rom, slave_dir in present.items()}
            conversion_s = self._conversion_s(present)
            transport = self._transport_for(bus_name)
            master_id = self._master_id_of(bus_dir, transport)
            therm = W1Therm(
                transport,
                sleeper=lambda seconds: interrupted.append(bool(self._stop.wait(seconds))),
            )
            read = therm.read_bus(
                master_id, targets, conversion_s=conversion_s, timeout_s=self._netlink_timeout_s
            )
        except (W1NetlinkError, ScratchpadError, OSError, ValueError) as exc:
            # A fresh socket next time: one that erred may have a reply to this
            # cycle still queued behind it, and a new port id has no backlog.
            self._close_transport(bus_name)
            self._demote_netlink(bus_dir, str(exc))
            return None
        if any(interrupted):
            # stop() landed inside the conversion, so the scratchpads that
            # followed it are one conversion stale and nobody will use them.
            return {}
        if read.all_failed:
            reasons = "; ".join(sorted(set(read.failures.values())))
            self._demote_netlink(bus_dir, f"every sensor on the bus failed ({reasons})")
            return None
        if self._netlink_ok.get(bus_name) is not True:
            _LOG.info(
                "onewire: %s reads over netlink (master id %d, %d sensor(s), %.3f s conversion)",
                bus_name,
                master_id,
                len(targets),
                conversion_s,
            )
        self._netlink_ok[bus_name] = True
        self._tier[bus_name] = _TIER_NETLINK
        result: dict[str, float | None] = {}
        for rom, reading in read.readings.items():
            if reading is None:
                self._crc_errors[rom] = self._crc_errors.get(rom, 0) + 1
                _LOG.debug("onewire: %s over netlink: %s", rom, read.failures.get(rom))
                value: float | None = None
            else:
                self._observed_bits[rom] = reading.resolution_bits
                value = reading.temperature_c
                if not math.isfinite(value):
                    value = None
            for name in self._rom_to_names[rom]:
                result[name] = value
        return result

    def _transport_for(self, bus_name: str) -> W1Netlink:
        """The open netlink socket this bus reads over, opening it if needed."""
        transport = self._transports.get(bus_name)
        if transport is None:
            transport = (
                self._netlink_factory()
                if self._netlink_factory is not None
                else W1Netlink(timeout_s=self._netlink_timeout_s)
            )
            self._transports[bus_name] = transport
        if not transport.is_open:
            transport.open()
        return transport

    def _close_transport(self, bus_name: str) -> None:
        transport = self._transports.pop(bus_name, None)
        if transport is not None:
            transport.close()

    def _master_id_of(self, bus_dir: Path, transport: W1Netlink) -> int:
        """The kernel's master id for ``bus_dir``, confirmed against the kernel's own list.

        A master's sysfs name is ``w1_bus_master%u`` of the very id a netlink
        master command carries (``drivers/w1/w1_int.c``), so the number in the
        name is the id -- but that is read out of a directory name, so it is
        checked against ``W1_LIST_MASTERS`` rather than trusted. The check is
        also the tier's liveness probe: a kernel with no w1 connector never
        answers it, and one round trip costs 0.13 ms (measured on the board)
        against a cycle of hundreds.
        """
        suffix = bus_dir.name.removeprefix(_BUS_PREFIX)
        if not suffix.isdigit():
            raise ValueError(f"{bus_dir.name} does not end in a bus master id")
        master_id = int(suffix)
        masters = transport.list_masters(timeout_s=self._netlink_timeout_s)
        if master_id not in masters:
            raise ValueError(f"the w1 connector lists masters {masters}, without {master_id}")
        return master_id

    def _slave_id_of(self, slave_dir: Path, rom: str) -> bytes:
        """The 8-byte ``struct w1_reg_num`` a netlink slave command addresses ``rom`` by.

        The slave's own ``id`` attribute is the authority; it is checked against
        the directory name, and a name that cannot be read falls back to
        rebuilding the identifier from that name and its CRC-8
        (:func:`~aqua_bridge.hw.w1_therm_netlink.reg_num_from_rom_name`).
        Cached: a ROM id is unique and does not change under its own name.
        """
        cached = self._slave_ids.get(rom)
        if cached is not None:
            return cached
        raw = b""
        try:
            raw = (slave_dir / _ID_FILE).read_bytes()
        except OSError as exc:
            _LOG.debug("onewire: cannot read %s/%s: %s", rom, _ID_FILE, exc)
        if len(raw) == 8 and rom_name_from_reg_num(raw) == rom:
            self._slave_ids[rom] = raw
            return raw
        derived = reg_num_from_rom_name(rom)
        self._slave_ids[rom] = derived
        return derived

    def _conversion_s(self, present: Mapping[str, Path]) -> float:
        """How long to wait for the bus-wide conversion, from what the sensors report.

        The driver's own ``conv_time`` for each present sensor, and -- for a
        sensor that has already been read this way -- the conversion time of
        the resolution it reported about *itself*, whichever is longer: a
        ``resolution`` write that silently failed would otherwise have us read
        the conversion before last. A sensor with no ``conv_time`` yet has not
        answered the driver at all, which is not a bus to run this tier on.
        """
        waits: list[float] = []
        for rom in present:
            conv_time_ms = self._conv_time_ms.get(rom)
            if conv_time_ms is None:
                raise ValueError(f"{rom} reports no {_CONV_TIME_FILE}")
            observed = self._observed_bits.get(rom)
            waits.append(max(conv_time_ms / 1000.0, CONVERSION_TIME_S.get(observed or 0, 0.0)))
        return max(waits)

    def _demote_netlink(self, bus_dir: Path, reason: str) -> None:
        first = self._netlink_ok.get(bus_dir.name) is not False
        self._netlink_ok[bus_dir.name] = False
        self._netlink_retry_at[bus_dir.name] = self._clock() + self._netlink_retry_s
        # Once loudly, then quietly: a kernel without the w1 connector at all
        # would otherwise warn every netlink_retry_s for the life of the daemon.
        log = _LOG.warning if first else _LOG.debug
        log(
            "onewire: %s does not read over netlink for the next %.0f s (%s); PROJECT.md item 38",
            bus_dir.name,
            self._netlink_retry_s,
            reason,
        )

    def bulk_read_modes(self) -> dict[str, str]:
        """Bus master name -> ``"bulk"``, ``"serial"`` or ``"unprobed"``.

        Whether the *sysfs* bulk probe succeeded on a bus, which is not the
        same question as which tier it read on: a bus reading over netlink
        never writes ``therm_bulk_read`` at all and so stays ``unprobed``
        here. :meth:`read_tiers` is the one that says what a bus used.
        """
        names = set(self._cycle_counts) | set(self._bulk_ok)
        if self._bulk_read == "off":
            return dict.fromkeys(names, "serial")
        out: dict[str, str] = dict.fromkeys(names, "unprobed")
        for name, ok in self._bulk_ok.items():
            out[name] = "bulk" if ok else "serial"
        return out

    def read_tiers(self) -> dict[str, str]:
        """Bus master name -> the tier that produced its last numbers.

        ``netlink``, ``bulk`` or ``serial``; ``none`` for a bus with no
        declared sensor on it (or one pinned to a tier it cannot run) and
        ``unprobed`` for one that has not run a cycle yet. Diagnostics for
        ``tools/w1_commission.py --check``: a cycle time means nothing without
        the tier it was measured on.
        """
        names = (
            set(self._cycle_counts) | set(self._tier) | set(self._bulk_ok) | set(self._netlink_ok)
        )
        out = dict.fromkeys(names, _TIER_UNPROBED)
        out.update(self._tier)
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
            # default 12 bit: 0.96 s over netlink, 9.6 s read serially); this floor
            # only guards the pathological/test case of a bus with no declared
            # sensor on it, or a fake that answers instantly, so the thread
            # never busy-spins a core.
            self._stop.wait(self._poll_interval_s)

    def stop(self) -> None:
        """Stops every reader thread and closes every netlink socket.

        Idempotent; safe to call if never started.
        """
        self._stop.set()
        for thread in self._threads:
            thread.join(timeout=5.0)
        self._threads.clear()
        for bus_name in list(self._transports):
            self._close_transport(bus_name)

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
    read_tier = section.get("read_tier", _DEFAULT_READ_TIER)
    if not isinstance(read_tier, str):
        raise ConfigError(f"onewire.read_tier must be a string, got {type(read_tier).__name__}")
    netlink_timeout_s = section.get("netlink_timeout_s", _DEFAULT_NETLINK_TIMEOUT_S)
    if not isinstance(netlink_timeout_s, int | float) or isinstance(netlink_timeout_s, bool):
        raise ConfigError(
            f"onewire.netlink_timeout_s must be a number, got {type(netlink_timeout_s).__name__}"
        )
    netlink_retry_s = section.get("netlink_retry_s", _DEFAULT_NETLINK_RETRY_S)
    if not isinstance(netlink_retry_s, int | float) or isinstance(netlink_retry_s, bool):
        raise ConfigError(
            f"onewire.netlink_retry_s must be a number, got {type(netlink_retry_s).__name__}"
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
            read_tier=read_tier,
            netlink_timeout_s=float(netlink_timeout_s),
            netlink_retry_s=float(netlink_retry_s),
        )
    except ConfigError as exc:
        raise ConfigError(f"onewire: {exc}") from exc
