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
   master's id in it and every present sensor's 8-byte identifier is readable;
   it keeps it as long as the conversion command and at least one scratchpad
   succeed. What takes it away is described under "Giving up a tier" below,
   **and the same cycle still finishes on the tier below, so no sample is
   lost**. Measured on the board, 7 sensors at
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

**Giving up a tier: what counts as evidence (section 8 item 39).** A tier is
worth keeping, so it is not surrendered on one bad cycle. Two things decide
it, and both are about *granularity*:

* **Whose failure is it?** One sensor's is that sensor's. A scratchpad that
  does not come back, or an attribute that cannot be read, is ``None`` (or a
  documented fallback) for that sensor and nothing at all for the fifteen
  beside it. Only a failure of the *bus* -- no connector on this kernel, a
  master the connector does not list, a conversion command that failed, a
  socket error, or **every** sensor on the bus failing at once -- is evidence
  about the bus, and only that may move the whole bus down a tier. In
  particular a sensor with no readable ``conv_time`` does **not** cost the bus
  its tier: the bus-wide wait falls back to the conversion time of the
  configured ``resolution_bits``, which is an upper bound for anything at or
  below that resolution, and the read-back is retried on later cycles instead
  of the absence being cached for the life of the process. (It was the other
  way round until section 8 item 39, and it cost the owner's 14-sensor bus its
  top tier on a sensor that reports ``conv_time=750`` when asked directly.)
* **How many times?** ``tier_failures_before_demote`` (default 3) consecutive
  cycles must fail on a tier before the bus drops off it for its retry window;
  a success resets the count. A failing probe costs one command and the cycle
  still finishes below, so trying twice more is cheap -- while a demotion
  costs the *fastest* tier for five minutes, and on a bus whose floor is
  serial reads that is measured in samples the control loop never sees
  (section 8 item 39, "What a demoted bus costs"). The exception is evidence
  that cannot change by being asked again -- this kernel has no ``w1``
  connector, this master is not in its list, this bus master has no
  ``therm_bulk_read`` file -- which demotes at once and is retried on the
  usual window like anything else.

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
loop. A read that fails becomes ``None`` for that sensor this cycle and is
counted, by *why* it failed, in :meth:`W1Source.read_stats` for
``tools/w1_commission.py --check``: ``errors`` is an ``OSError`` from the
read (the kernel reports EIO for a failed CRC) or a netlink scratchpad that
did not come back, ``empty`` is a ``temperature`` attribute that answered
with **nothing at all**, and ``rejected`` is an answer that is not a number
this module will publish. ``attempts`` is counted with them, so a rate has
the denominator that belongs to it: a sensor no cycle ever reached (it is
not under any bus master) has none, which is a different report from a
sensor that was read and answered every time.

**One cycle at a time per bus master (section 8 item 39).** A bus-wide cycle
*owns* its bus master from the conversion it starts to the last scratchpad it
reads, and the kernel keeps that ownership in one flag per slave: a bulk
trigger sets "converting" on every slave on the master, the conversion sets
"a value nobody has read yet", and the **first** read of that slave's
``temperature`` consumes it. So two cycles overlapping on one bus master do
not merely compete for the wire, they eat each other's readings, and both
halves of that were measured on the board (14 sensors, one bit-banged bus,
12 bit):

* whichever cycle reads a slave first gets its scratchpad in 18 ms; the other
  finds the flag consumed and its read starts **its own** conversion, 800 ms
  (measured: 18 ms for the first read after a trigger, 800 ms for the second
  and third). Enough of those and a cycle that should cost 1.0 s costs 6.4 s.
* a read that lands while the *other* cycle's Convert T is in flight gets an
  **empty** string back, not an error and not a number -- and since that
  conversion holds the bus for 765 ms, so does every read left in this cycle:
  the whole tail of the cycle is lost at once (measured: two sensors read,
  twelve empty, from one trigger by a second reader).

That is what :meth:`run_bus_cycle` is serialised against: it takes a lock per
bus master name for the whole cycle, so a second caller **waits** instead of
reading into the middle of a cycle, and says so once per bus in the log --
two readers on one bus is a programming mistake, not a bus fault, and the
symptom (a third of the readings gone, six times the cycle time) looks
exactly like bad wiring. The lock covers this process only, so it cannot
reach a second *process* reading the same bus -- ``tools/w1_commission.py``
against a running daemon -- which is why ``empty`` is still counted and named
apart: a cross-process reader that goes ahead anyway (``--check --force``, or
a daemon that could not take :class:`ReaderLock` below) is caught here too,
just later and more expensively than being refused up front.
``read()`` never takes this lock, so nothing here can hold up a tick.

**A cross-process lock is a different animal (section 8 item 39,
"proposals").** :meth:`W1Source.start` takes an OS-level advisory lock
(:class:`ReaderLock`, ``flock(2)`` on ``onewire.lock_path``) for as long as
its reader threads run, and :meth:`W1Source.stop` releases it.
``tools/w1_commission.py --check`` tries to take the same lock before it
drives a single cycle; if it cannot, something else is already reading these
buses and its measurement would be the corruption above, reported as if it
were the hardware's fault. This is preferred over asking systemd whether
``aqua-bridge.service`` is active: a unit lookup only answers for one name,
needs systemd reachable, and says nothing about a foreground run, a renamed
unit or a container sharing this root, where an ``flock`` on a path every
reader opens does not care what called it or how. The daemon never refuses
to *start* over this lock (a diagnostic tool's lock must never cost a zone
its cooling, plan section 1 priority 1); it only logs if it could not be
taken. Three outcomes, never guessed past: the lock is taken (nothing else
was reading), it is held (something is), or it cannot be told at all -- the
lock directory does not exist and could not be created, a permission error,
or a filesystem with no ``flock`` support -- which ``--check`` treats the
same as "held" unless ``--force`` says otherwise.

:meth:`W1Source.read` (called from the control loop thread) never touches
the filesystem and never blocks: it returns the latest published sample for
every declared sensor, or ``None`` when the sensor has never reported, its
last read failed, or the sample is older than ``max_age_s``.

**Shutdown.** :meth:`W1Source.stop` sets a stop event and joins each reader
thread with a 5 s bound. A netlink cycle already notices the event during
its one conversion wait, the ``sleeper`` it hands
:class:`~aqua_bridge.hw.w1_therm_netlink.W1Therm`; the sysfs tiers check it
before starting a new bus-wide bulk conversion and again between every
per-sensor serial read, so :meth:`run_bus_cycle` abandons whatever sensors
are left rather than read through them. None of these checks can interrupt
a read or a bulk trigger already in flight -- each is one blocking sysfs
call the kernel finishes in its own time -- so the worst a stop has to ride
out is whichever one was already running when it landed: one sensor's
conversion (about 800 ms at the 12-bit default) or the bulk trigger's, never
the rest of the bus behind it. An abandoned cycle publishes nothing --
same contract a netlink cycle interrupted by :meth:`stop` already has: the
sensors it did not reach keep the sample and timestamp they already had
rather than being silently left out of an otherwise-published partial
result, and what it did read this cycle is discarded too rather than kept
half of the bus. The 5 s join is now a backstop against a misconfigured
timeout, not the common case.

**A cycle over budget.** The budget is ``max_age_s / 2`` per bus (module
docstring below): a cycle that fits it refreshes every sensor twice inside
``max_age_s``, so one slow cycle never ages a sensor out of :meth:`read`.
Past it, every sensor on that bus starts reading as missing on some ticks
with no other symptom (PROJECT.md section 8 item 39) -- a cycle that runs to
completion (not one :meth:`stop` cut short) and takes longer than the budget
logs a warning naming the measured cycle, the budget, the sensor count and
the tier it ran on, at most once per ``onewire.slow_cycle_log_interval_s``
(default 60 s) so a permanently slow bus does not flood the log.

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
from dataclasses import dataclass, replace
from enum import StrEnum
from pathlib import Path
from typing import Any

from aqua_bridge.hw.w1_netlink import W1Netlink, W1NetlinkError, W1NetlinkUnavailable
from aqua_bridge.hw.w1_therm_netlink import (
    CONVERSION_TIME_S,
    ScratchpadError,
    W1Therm,
    reg_num_from_rom_name,
    rom_name_from_reg_num,
)
from aqua_bridge.model import ConfigError

try:  # Linux and macOS have fcntl; flock(2) is what ReaderLock needs from it.
    import fcntl
except ImportError:  # pragma: no cover - Windows
    fcntl = None  # type: ignore[assignment]

__all__ = [
    "ReadStats",
    "ReaderLock",
    "ReaderLockOutcome",
    "W1Source",
    "build_onewire_from_config",
]

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
#: How many consecutive cycles must fail on a tier before the bus is dropped off
#: it for that tier's retry window (module docstring, "Giving up a tier"). One
#: failed cycle is not evidence about a tier: it can be a kernel busy elsewhere,
#: a sensor pulled out mid-cycle, or a second reader of the same bus in another
#: process. Retrying is cheap -- a failed probe is one command and the same cycle
#: still finishes on the tier below -- and a demotion is expensive: five minutes
#: (``netlink_retry_s`` / ``bulk_retry_s``) on a slower tier, which on a bus whose
#: floor is serial reads means sensors reading as missing on some ticks (section 8
#: item 39). Three consecutive failures is about 15 s of evidence at ``dt = 5 s``,
#: and evidence that cannot change by asking again (no connector, no such master,
#: no ``therm_bulk_read``) demotes at once regardless.
_DEFAULT_TIER_FAILURES_BEFORE_DEMOTE = 3
# How often the slow-cycle warning (module docstring, "A cycle over budget")
# may repeat for one bus once it is past its budget. Same default and the
# same argument as control/loop.py's mpc.budget_log_interval_s: frequent
# enough that an operator watching the log sees it soon, rare enough that a
# bus that stays over budget for the life of the daemon does not flood it.
_DEFAULT_SLOW_CYCLE_LOG_INTERVAL_S = 60.0
#: ``onewire.lock_path``: where :class:`ReaderLock` takes its ``flock(2)``.
#: ``/run`` is a tmpfs cleared every boot, matching a lock that only ever means
#: "someone is reading right now" -- it must never survive to mean anything
#: once the reader that took it is gone. A per-service ``RuntimeDirectory=``
#: (``deploy/aqua-bridge.service``) gives the daemon's user write access to
#: this directory without running it as root; ``tools/w1_commission.py`` is
#: expected to run as the same user, or with equivalent access to this path.
_DEFAULT_LOCK_PATH = "/run/aqua-bridge/onewire.lock"


class ReaderLockOutcome(StrEnum):
    """What :meth:`ReaderLock.try_acquire` found.

    A name lookup (is a given systemd unit active?) can only ever answer for
    the one name it asks about, needs systemd reachable to ask at all, and
    says nothing about a foreground run, a differently named unit or a
    container sharing this root. ``flock(2)`` on a path every reader opens
    does not care what called it or how, which is why :class:`ReaderLock`
    uses that instead (section 8 item 39, "proposals").
    """

    #: We hold the lock now; nothing else did a moment ago. Caller must
    #: :meth:`ReaderLock.release` it when done.
    ACQUIRED = "acquired"
    #: Something else holds it -- reading right now, by definition, since
    #: nothing takes this lock except to drive read cycles.
    HELD = "held"
    #: Could not tell, and never guessed past that: the lock directory does
    #: not exist and could not be created, a permission error, or a
    #: filesystem with no ``flock`` support (or no :mod:`fcntl` at all).
    UNKNOWN = "unknown"


class ReaderLock:
    """One ``flock(2)`` advisory lock, held by whoever is driving read cycles.

    :meth:`W1Source.start` takes it for as long as its reader threads run and
    :meth:`W1Source.stop` releases it; ``tools/w1_commission.py --check``
    tries to take the same one before it drives a single cycle of its own.
    Two cycles overlapping on one bus master consume each other's kernel
    "value ready" marks (module docstring, "One cycle at a time per bus
    master") -- this lock is what lets a second driver find that out *before*
    it starts, rather than from the corrupted numbers afterward.

    Non-blocking throughout: a caller that cannot have the lock right now
    is told so and left to decide, never made to wait for it (the daemon
    must never block starting on a diagnostic tool, and a diagnostic tool
    that blocked here could wait out the daemon's entire run).
    """

    def __init__(self, path: str | Path) -> None:
        self._path = Path(path)
        self._file: Any = None

    @property
    def path(self) -> Path:
        return self._path

    def try_acquire(self) -> ReaderLockOutcome:
        """Non-blocking; see :class:`ReaderLockOutcome` for the three results.

        Idempotent while already held by this instance (returns ``ACQUIRED``
        again without reopening anything).
        """
        if self._file is not None:
            return ReaderLockOutcome.ACQUIRED
        if fcntl is None:  # pragma: no cover - Windows
            return ReaderLockOutcome.UNKNOWN
        try:
            self._path.parent.mkdir(parents=True, exist_ok=True)
            fh = self._path.open("a+")
        except OSError as exc:
            _LOG.debug("onewire: could not open reader lock %s: %s", self._path, exc)
            return ReaderLockOutcome.UNKNOWN
        try:
            fcntl.flock(fh.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            fh.close()
            return ReaderLockOutcome.HELD
        except OSError as exc:
            fh.close()
            _LOG.debug("onewire: could not lock %s: %s", self._path, exc)
            return ReaderLockOutcome.UNKNOWN
        self._file = fh
        return ReaderLockOutcome.ACQUIRED

    def release(self) -> None:
        """Idempotent; safe to call whether or not the lock was ever taken."""
        if self._file is None:
            return
        try:
            if fcntl is not None:  # pragma: no branch - only None on Windows
                fcntl.flock(self._file.fileno(), fcntl.LOCK_UN)
        finally:
            self._file.close()
            self._file = None


@dataclass
class _Sample:
    value: float | None
    ts: float


@dataclass
class ReadStats:
    """One sensor's read outcomes since construction (:meth:`W1Source.read_stats`).

    Counted apart because they are different faults with different owners, and
    a single "failed reads" number hides which one is happening (section 8
    item 39):

    attempts:
        Reads this module actually issued for this ROM id. Zero means no cycle
        ever reached the sensor -- it is declared in config but not under any
        bus master -- which is a *binding* report, not a reliability one, and
        must never be shown as a 0 % failure rate.
    errors:
        The read raised ``OSError`` (``w1_therm`` reports EIO when the
        scratchpad fails its CRC in the kernel), or, on the netlink tier, the
        scratchpad did not come back or failed the CRC check this module does
        itself. The sensor or its wiring.
    empty:
        The ``temperature`` attribute answered with nothing at all. The kernel
        does that while a bulk conversion it started is still in flight, so
        this is the signature of a *second reader* on the same bus master --
        another process, since one process serialises its own cycles (module
        docstring, "One cycle at a time per bus master").
    rejected:
        The read answered, and the answer is not a temperature this module will
        publish (it does not parse as an integer, or does not survive being
        turned into a finite number of degrees).
    """

    attempts: int = 0
    errors: int = 0
    empty: int = 0
    rejected: int = 0

    @property
    def failed(self) -> int:
        """Reads that produced no value, whatever the reason."""
        return self.errors + self.empty + self.rejected


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
        ``conv_time`` is read back afterwards -- and asked again on later
        cycles while it has not answered, since that read costs no bus
        traffic -- and reported by :meth:`conv_time_ms`.
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
    tier_failures_before_demote:
        How many consecutive cycles must fail on a tier before the bus is
        dropped off it (module docstring, "Giving up a tier";
        :data:`_DEFAULT_TIER_FAILURES_BEFORE_DEMOTE` for the argument). 1
        restores the old "one bad cycle is enough". Evidence that cannot
        change by asking again demotes on the first cycle whatever this says.
    slow_cycle_log_interval_s:
        At most one "cycle over budget" warning per bus in this many seconds
        (module docstring); the first exceedance for a bus always logs.
    netlink_factory:
        Called with no arguments to build one
        :class:`~aqua_bridge.hw.w1_netlink.W1Netlink` per bus master. Injected
        so tests drive the whole ladder without opening a socket.
    lock_path:
        Where :meth:`start`/:meth:`stop` take and release the cross-process
        :class:`ReaderLock` (:data:`_DEFAULT_LOCK_PATH`). Also what
        ``tools/w1_commission.py --check`` probes before measuring, so it
        must name a path every reader of this ``root`` can reach -- a
        second, unrelated ``root`` (a config for a different board sharing
        this file by mistake) would make ``--check`` refuse against a
        daemon reading nothing it cares about, which is why this is a
        config key and not folded into ``root`` itself.
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
        tier_failures_before_demote: int = _DEFAULT_TIER_FAILURES_BEFORE_DEMOTE,
        slow_cycle_log_interval_s: float = _DEFAULT_SLOW_CYCLE_LOG_INTERVAL_S,
        netlink_factory: Callable[[], W1Netlink] | None = None,
        lock_path: str | Path = _DEFAULT_LOCK_PATH,
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
        if not tier_failures_before_demote >= 1:
            raise ConfigError(
                "onewire tier_failures_before_demote must be >= 1, got "
                f"{tier_failures_before_demote}"
            )
        if not slow_cycle_log_interval_s > 0:
            raise ConfigError(
                f"onewire slow_cycle_log_interval_s must be > 0, got {slow_cycle_log_interval_s}"
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
        self._read_tier = read_tier
        self._netlink_timeout_s = float(netlink_timeout_s)
        self._netlink_retry_s = float(netlink_retry_s)
        self._tier_failures_before_demote = int(tier_failures_before_demote)
        self._slow_cycle_log_interval_s = float(slow_cycle_log_interval_s)
        self._netlink_factory = netlink_factory
        self.lock_path = Path(lock_path)
        self._reader_lock = ReaderLock(self.lock_path)

        self._lock = threading.Lock()
        self._latest: dict[str, _Sample] = {}
        self._configured_resolution: set[str] = set()
        self._conv_time_ms: dict[str, int] = {}
        # ROM ids whose conv_time could not be read yet and so run on the
        # configured resolution's conversion time (logged once each, not per
        # cycle: the read-back is retried every cycle until it answers).
        self._conv_time_fallback_logged: set[str] = set()
        self._read_stats: dict[str, ReadStats] = {rom: ReadStats() for rom in self._rom_to_names}
        self._cycle_counts: dict[str, int] = {}
        # One cycle at a time per bus master (module docstring, "One cycle at a
        # time per bus master"): a bus-wide cycle owns its master from the
        # conversion to the last scratchpad, so a second caller waits here
        # rather than reading into the middle of one. Created on demand under
        # _cycle_locks_guard because buses are discovered, not declared, and
        # logged once per bus when it is actually contended -- two readers on
        # one bus is a programming mistake worth a line, not a bus fault.
        self._cycle_locks_guard = threading.Lock()
        self._cycle_locks: dict[str, threading.Lock] = {}
        self._contention_logged: set[str] = set()
        # Bus master name -> when a slow-cycle warning was last logged for it,
        # and how many exceedances have happened since (module docstring, "A
        # cycle over budget"; same bookkeeping shape as control/loop.py's step
        # budget alarm).
        self._slow_cycle_logged_at: dict[str, float] = {}
        self._slow_cycle_since_log: dict[str, int] = {}
        # Bus master name -> whether the bulk path is in use on it (absent: not
        # probed yet), when a bus that failed may be probed again, and how many
        # cycles in a row have failed on it without a demotion yet (module
        # docstring, "Giving up a tier").
        self._bulk_ok: dict[str, bool] = {}
        self._bulk_retry_at: dict[str, float] = {}
        self._bulk_failures: dict[str, int] = {}
        # The same, for the netlink tier, plus the socket each bus reads over
        # (one per reader thread: a netlink socket carries one exchange at a
        # time), the identifiers slaves are addressed by, the resolution each
        # sensor last reported about itself, and the tier each bus's last
        # numbers came from.
        self._netlink_ok: dict[str, bool] = {}
        self._netlink_retry_at: dict[str, float] = {}
        self._netlink_failures: dict[str, int] = {}
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
        :meth:`stop` arrived mid-cycle, on any tier -- module docstring,
        "Shutdown"). Never raises: every failure mode narrows to ``None`` for
        the sensor(s) it affects or to a tier below, per the module
        docstring. A cycle that runs to completion and took longer than the
        bus's budget logs a rate-limited warning (module docstring, "A cycle
        over budget").

        **One cycle at a time per bus master.** A cycle owns its bus master
        from the conversion it starts to the last scratchpad it reads (module
        docstring, "One cycle at a time per bus master"), so a call that
        arrives while another cycle is running on the same bus *waits* for it
        instead of reading into the middle of it. Two cycles that do overlap
        consume each other's readings and multiply the cycle time, which is
        what section 8 item 39 measured, so the wait is the point -- and it is
        bounded by one cycle on that bus. :meth:`read` takes no part in this
        and never blocks.
        """
        lock = self._cycle_lock(bus_dir.name)
        if not lock.acquire(blocking=False):
            self._log_contention(bus_dir.name)
            lock.acquire()
        try:
            return self._run_one_cycle(bus_dir)
        finally:
            lock.release()

    def _cycle_lock(self, bus_name: str) -> threading.Lock:
        """The one lock that serialises cycles on ``bus_name``, created on demand."""
        with self._cycle_locks_guard:
            lock = self._cycle_locks.get(bus_name)
            if lock is None:
                lock = threading.Lock()
                self._cycle_locks[bus_name] = lock
            return lock

    def _log_contention(self, bus_name: str) -> None:
        """Says once per bus that two cycles overlapped on it."""
        if bus_name in self._contention_logged:
            return
        self._contention_logged.add(bus_name)
        _LOG.warning(
            "onewire: two read cycles overlapped on %s; they are serialised, but one reader "
            "per bus is the contract -- a second reader in another process (a commissioning "
            "tool against a running daemon) loses about a third of the readings and costs six "
            "times the cycle time; PROJECT.md item 39",
            bus_name,
        )

    def _run_one_cycle(self, bus_dir: Path) -> dict[str, float | None]:
        """One cycle on ``bus_dir``, with this bus master's cycle lock held."""
        present = {rom: bus_dir / rom for rom in self._rom_to_names if (bus_dir / rom).is_dir()}
        if not present:
            self._tier[bus_dir.name] = _TIER_NONE
            return {}
        self._ensure_resolution(present)
        start = self._clock()
        if self._netlink_due(bus_dir.name):
            netlink_result = self._netlink_cycle(bus_dir, present)
            if netlink_result is not None:
                if netlink_result:
                    # Non-empty: a cycle that ran to completion, not one stop()
                    # cut short (which publishes {}, same as every path below).
                    self._check_slow_cycle(bus_dir.name, start, len(present), _TIER_NETLINK)
                return netlink_result
        if self._read_tier == _TIER_NETLINK:
            # Pinned to netlink and this bus cannot run it: publish nothing
            # rather than quietly reading it another way. Missing samples read
            # as missing, which raises cooling and never lowers it.
            self._tier[bus_dir.name] = _TIER_NONE
            return {}
        if self._stop.is_set():
            # Nothing on this tier has touched the bus yet: the bulk trigger
            # below is one blocking write the kernel sleeps a whole conversion
            # out inside (module docstring), so the only place to act on a
            # stop request is before writing it, not during. Same "abandoned
            # cycle publishes nothing" contract as the netlink tier above.
            return {}
        triggered = False
        if (
            self._bulk_read != "off"
            and self._read_tier != _TIER_SERIAL
            and self._bulk_probe_due(bus_dir.name)
        ):
            triggered = self._bulk_convert(bus_dir)
        result: dict[str, float | None] = {}
        for rom, slave_dir in present.items():
            if self._stop.is_set():
                # Abandon the rest: each read is its own blocking conversion
                # (module docstring), so the check runs between sensors, not
                # inside one. What was already read this cycle is discarded
                # too, for the same reason the netlink tier discards a
                # conversion stop() cut short -- consistent beats partial, and
                # the sensors not reached keep their previous sample and its
                # real timestamp rather than being silently missing from an
                # otherwise-published result.
                return {}
            value = self._read_temperature(slave_dir, rom)
            for name in self._rom_to_names[rom]:
                result[name] = value
        tier = self._sysfs_tier(triggered=triggered)
        self._tier[bus_dir.name] = tier
        self._check_slow_cycle(bus_dir.name, start, len(present), tier)
        return result

    def _check_slow_cycle(self, bus_name: str, start: float, sensor_count: int, tier: str) -> None:
        """Warns, rate limited, when this cycle ran longer than ``bus_name``'s budget.

        The budget is ``max_age_s / 2`` (module docstring, "A cycle over
        budget"): a cycle that fits it refreshes every sensor twice inside
        ``max_age_s``, so one slow cycle never ages a sensor out of
        :meth:`read`. Past it, every sensor on this bus starts reading as
        missing on some ticks with no other symptom (PROJECT.md section 8
        item 39) -- this is the one place that says so, at most once per
        ``onewire.slow_cycle_log_interval_s`` per bus, naming every number a
        person needs to tell a permanently too-slow bus from one tick that
        ran long.
        """
        now = self._clock()
        elapsed = now - start
        budget = self._max_age_s / 2.0
        if elapsed <= budget:
            return
        since = self._slow_cycle_since_log.get(bus_name, 0) + 1
        logged_at = self._slow_cycle_logged_at.get(bus_name)
        if logged_at is not None and now - logged_at < self._slow_cycle_log_interval_s:
            self._slow_cycle_since_log[bus_name] = since
            return
        _LOG.warning(
            "onewire: %s cycle took %.2f s, over its budget of %.2f s (max_age_s / 2) with "
            "%d sensor(s) on the %s tier (%d exceedance(s) since the last line); "
            "PROJECT.md item 39",
            bus_name,
            elapsed,
            budget,
            sensor_count,
            tier,
            since,
        )
        self._slow_cycle_logged_at[bus_name] = now
        self._slow_cycle_since_log[bus_name] = 0

    # -- the netlink tier -------------------------------------------------------

    def _netlink_due(self, bus_name: str) -> bool:
        """Whether this cycle may try the netlink tier on ``bus_name``."""
        if self._read_tier not in ("auto", _TIER_NETLINK):
            return False
        if self._netlink_ok.get(bus_name, True):
            return True
        return self._clock() >= self._netlink_retry_at.get(bus_name, 0.0)

    def _sysfs_tier(self, *, triggered: bool) -> str:
        """Which sysfs tier this cycle actually ran on.

        The bus-wide conversion of *this* cycle, not whether the tier is still
        allowed: a trigger the kernel refused leaves every following read
        converting on its own, which is a serial cycle whatever
        ``tier_failures_before_demote`` has decided about the tier. A cycle time
        reported against the wrong tier is worse than no number
        (``tools/w1_commission.py --check``).
        """
        return _TIER_BULK if triggered else _TIER_SERIAL

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
        except (W1NetlinkUnavailable, ValueError) as exc:
            # The tier is not there to be had on this bus: this kernel has no w1
            # connector (W1NetlinkUnavailable), or the connector does not list
            # this master at all (ValueError, the only source of one here now
            # that a missing conv_time no longer raises). Asking again in the
            # same minute cannot change either, so this does not wait for a
            # count; the usual retry window still re-probes it.
            self._close_transport(bus_name)
            self._demote_netlink(bus_dir, str(exc), structural=True)
            return None
        except (W1NetlinkError, ScratchpadError, OSError) as exc:
            # A timeout, a status, a malformed reply, a socket that erred: all
            # things a busy kernel or a sensor pulled out mid-cycle can do once.
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
        self._netlink_failures[bus_name] = 0
        self._tier[bus_name] = _TIER_NETLINK
        result: dict[str, float | None] = {}
        for rom, reading in read.readings.items():
            stats = self._stats_of(rom)
            stats.attempts += 1
            if reading is None:
                # A scratchpad that did not come back or failed its CRC: this
                # sensor's fault, counted as an error like an EIO from sysfs.
                # There is no "empty" on this tier -- a netlink read either
                # carries nine bytes or raises.
                stats.errors += 1
                _LOG.debug("onewire: %s over netlink: %s", rom, read.failures.get(rom))
                value: float | None = None
            else:
                self._observed_bits[rom] = reading.resolution_bits
                value = reading.temperature_c
                if not math.isfinite(value):
                    stats.rejected += 1
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
        the conversion before last.

        A sensor whose ``conv_time`` has not been read yet contributes the
        conversion time of the configured ``resolution_bits`` instead, which is
        an upper bound for any sensor at or below that resolution -- so the
        wait is still long enough for it, and one unreadable attribute on one
        sensor costs the bus nothing. That attribute is retried every cycle
        (:meth:`_ensure_resolution`); until it answers, this logs once per
        sensor. Until section 8 item 39 this raised instead, and the bus lost
        its fastest tier for five minutes over a sensor that answers
        ``conv_time`` perfectly well when asked again.
        """
        fallback_s = CONVERSION_TIME_S.get(self._resolution_bits, max(CONVERSION_TIME_S.values()))
        waits: list[float] = []
        for rom in present:
            conv_time_ms = self._conv_time_ms.get(rom)
            if conv_time_ms is None:
                if rom not in self._conv_time_fallback_logged:
                    self._conv_time_fallback_logged.add(rom)
                    _LOG.info(
                        "onewire: %s has not reported %s yet; the bus-wide conversion waits "
                        "%.3f s for it, the configured %d-bit time, until it does",
                        rom,
                        _CONV_TIME_FILE,
                        fallback_s,
                        self._resolution_bits,
                    )
                sensor_s = fallback_s
            else:
                sensor_s = conv_time_ms / 1000.0
            observed = self._observed_bits.get(rom)
            waits.append(max(sensor_s, CONVERSION_TIME_S.get(observed or 0, 0.0)))
        return max(waits)

    def _tier_failure_is_enough(
        self, bus_name: str, counts: dict[str, int], tier: str, reason: str, *, structural: bool
    ) -> bool:
        """Counts this cycle's failure on a tier and says whether to give the tier up.

        Module docstring, "Giving up a tier": ``tier_failures_before_demote``
        consecutive failing cycles are the evidence, unless the failure is one
        that cannot change by asking again. A cycle that fails without
        demoting still finishes on the tier below, so nothing is lost while the
        evidence is collected -- what is spent is one more failed probe.
        """
        failures = counts.get(bus_name, 0) + 1
        counts[bus_name] = failures
        if structural or failures >= self._tier_failures_before_demote:
            return True
        _LOG.info(
            "onewire: %s failed the %s tier (%s); %d of %d consecutive failures, keeping the "
            "tier and finishing this cycle on the one below; PROJECT.md item 39",
            bus_name,
            tier,
            reason,
            failures,
            self._tier_failures_before_demote,
        )
        return False

    def _demote_netlink(self, bus_dir: Path, reason: str, *, structural: bool = False) -> None:
        if not self._tier_failure_is_enough(
            bus_dir.name, self._netlink_failures, _TIER_NETLINK, reason, structural=structural
        ):
            return
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
        """ROM id -> the driver's own ``conv_time``, for the ids that have reported one.

        A ROM id missing here has not answered that attribute *yet*: it is asked
        again every cycle, and meanwhile the bus-wide conversion waits the
        configured resolution's time for it (:meth:`_conversion_s`).
        """
        return dict(self._conv_time_ms)

    def _ensure_resolution(self, present: Mapping[str, Path]) -> None:
        """Writes the configured resolution once per sensor and learns its ``conv_time``.

        Two facts, kept apart on purpose (module docstring, "Giving up a
        tier"). The ``resolution`` write and its read-back happen **once** per
        sensor: both are scratchpad traffic, and the value does not change on
        its own. Whether this module knows the sensor's ``conv_time`` is a
        different question, and its answer can be "not yet": that attribute is
        a value the driver already holds, so reading it costs no bus traffic,
        and it is asked again on every cycle until it answers rather than its
        absence being cached for the life of the process. A transient failure
        to read one attribute of one sensor used to cost the whole bus its
        fastest tier for five minutes (section 8 item 39).
        """
        for rom, slave_dir in present.items():
            if rom not in self._configured_resolution:
                self._configured_resolution.add(rom)
                try:
                    (slave_dir / _RESOLUTION_FILE).write_text(str(self._resolution_bits))
                except OSError as exc:
                    # Most likely the udev rule of deploy/99-w1-therm.rules has
                    # not applied: resolution is root-owned and the service user
                    # is not root. The sensor still reads, at whatever resolution
                    # it has, so this is a warning and not a reason to drop the
                    # bus.
                    _LOG.warning("onewire: cannot set resolution for %s: %s", rom, exc)
                try:
                    readback = (slave_dir / _RESOLUTION_FILE).read_text().strip()
                except OSError as exc:
                    _LOG.debug("onewire: cannot read back resolution for %s: %s", rom, exc)
                else:
                    if readback != str(self._resolution_bits):
                        _LOG.warning(
                            "onewire: %s reports resolution %s bits, not the configured %d",
                            rom,
                            readback,
                            self._resolution_bits,
                        )
                    _LOG.debug("onewire: %s at %s bits", rom, readback)
            if rom in self._conv_time_ms:
                continue
            try:
                conv_time = int((slave_dir / _CONV_TIME_FILE).read_text().strip())
            except (OSError, ValueError) as exc:
                _LOG.debug("onewire: cannot read %s for %s: %s", _CONV_TIME_FILE, rom, exc)
                continue
            self._conv_time_ms[rom] = conv_time
            self._conv_time_fallback_logged.discard(rom)
            _LOG.debug("onewire: %s reports conv_time %d ms", rom, conv_time)

    def _bulk_probe_due(self, bus_name: str) -> bool:
        """Whether this cycle may trigger a bulk read on ``bus_name``."""
        if self._bulk_ok.get(bus_name, True):
            return True
        return self._clock() >= self._bulk_retry_at.get(bus_name, 0.0)

    def _demote_bulk(self, bus_dir: Path, reason: str, *, structural: bool = False) -> None:
        if not self._tier_failure_is_enough(
            bus_dir.name, self._bulk_failures, _TIER_BULK, reason, structural=structural
        ):
            return
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

    def _bulk_convert(self, bus_dir: Path) -> bool:
        """Triggers one bulk conversion on ``bus_dir``; True if it registered.

        The kernel's implementation converts inside the ``write()``, so the
        status is read **once** afterwards and is expected to be ``1``; ``-1``
        is polled out under ``bulk_timeout_s`` for an implementation that
        returns early. Anything else counts a failure against the tier (module
        docstring, "Giving up a tier": ``tier_failures_before_demote``
        consecutive ones drop the bus to serial reads for ``bulk_retry_s``, and
        evidence that cannot change -- no such attribute -- drops it at once),
        and either way the caller's per-slave reads still produce this cycle's
        readings, each running its own conversion. False also means "this cycle
        was a serial one", whatever the tier's standing. Never raises.
        """
        trigger_path = bus_dir / _TRIGGER_FILE
        before = self._read_bulk_state(trigger_path)
        if before is None:
            # No such file is structural -- only one master system-wide gets one
            # (module docstring) and that does not change while we watch. One
            # that exists but would not read is a failure like any other.
            absent = not trigger_path.exists()
            self._demote_bulk(
                bus_dir,
                f"{_TRIGGER_FILE} is {'absent' if absent else 'unreadable'}",
                structural=absent,
            )
            return False
        if before not in _BULK_STATES:
            self._demote_bulk(bus_dir, f"{_TRIGGER_FILE} reads {before!r}, not a w1_therm state")
            return False
        try:
            trigger_path.write_bytes(_BULK_TRIGGER)
        except OSError as exc:
            self._demote_bulk(bus_dir, f"trigger write failed: {exc}")
            return False
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
            return False
        deadline = self._clock() + self._bulk_timeout_s
        while state == _BULK_RUNNING:
            if self._stop.is_set():
                # Shutting down: leave the bus as it is (the driver finishes the
                # conversion on its own) rather than spinning out the deadline.
                return False
            if self._clock() >= deadline:
                self._demote_bulk(
                    bus_dir, f"conversion unfinished after bulk_timeout_s={self._bulk_timeout_s} s"
                )
                return False
            self._stop.wait(self._poll_interval_s)
            state = self._read_bulk_state(trigger_path)
        if state != _BULK_DONE:
            self._demote_bulk(bus_dir, f"{_TRIGGER_FILE} reads {state!r} mid-conversion")
            return False
        if self._bulk_ok.get(bus_dir.name) is False:
            _LOG.info("onewire: %s honours the bulk trigger again", bus_dir.name)
        self._bulk_ok[bus_dir.name] = True
        self._bulk_failures[bus_dir.name] = 0
        return True

    def _stats_of(self, rom: str) -> ReadStats:
        """The mutable counters for ``rom`` (:class:`ReadStats`), created on demand."""
        stats = self._read_stats.get(rom)
        if stats is None:
            stats = ReadStats()
            self._read_stats[rom] = stats
        return stats

    def _read_temperature(self, slave_dir: Path, rom: str) -> float | None:
        """One sysfs ``temperature`` read, counted by outcome (:class:`ReadStats`)."""
        stats = self._stats_of(rom)
        stats.attempts += 1
        try:
            raw = (slave_dir / _TEMPERATURE_FILE).read_text().strip()
        except OSError as exc:
            stats.errors += 1
            _LOG.debug("onewire: read failed for %s: %s", rom, exc)
            return None
        if not raw:
            # Nothing at all, which is not the same as a failure: the kernel
            # answers an empty read while a bulk conversion it started is in
            # flight, so this says another reader is converting on this bus
            # (module docstring, "One cycle at a time per bus master"). One
            # process cannot do this to itself any more; another can.
            stats.empty += 1
            _LOG.debug(
                "onewire: %s answered an empty %s -- a bulk conversion is in flight on this "
                "bus, started by someone else",
                rom,
                _TEMPERATURE_FILE,
            )
            return None
        try:
            value = int(raw) / 1000.0
        except (ValueError, OverflowError) as exc:
            stats.rejected += 1
            _LOG.debug("onewire: %s answered %r, not a temperature: %s", rom, raw, exc)
            return None
        if not math.isfinite(value):
            stats.rejected += 1
            return None
        return value

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

        Also takes :attr:`lock_path`'s cross-process :class:`ReaderLock` for
        as long as the reader threads run (:meth:`stop` releases it), so a
        commissioning run elsewhere can tell this daemon is reading before it
        starts a cycle of its own (module docstring, "A cross-process lock is
        a different animal"). Never blocks and never refuses to start over
        it -- a diagnostic tool's lock must not cost a zone its cooling
        (plan section 1 priority 1) -- only logs if it could not be taken.
        """
        if self._threads:
            return
        buses = self.discover_buses()
        if not buses:
            _LOG.warning("onewire: no w1 bus master found under %s", self.root)
        for missing in self.missing_roms():
            names = ", ".join(self._rom_to_names[missing])
            _LOG.warning("onewire: ROM %s (%s) not found under %s", missing, names, self.root)
        outcome = self._reader_lock.try_acquire()
        if outcome is not ReaderLockOutcome.ACQUIRED:
            _LOG.warning(
                "onewire: could not take the reader lock at %s (%s); "
                "tools/w1_commission.py --check will not be able to tell this daemon is "
                "reading from it and may measure the two-cycle corruption of PROJECT.md "
                "item 39 without warning",
                self.lock_path,
                outcome.value,
            )
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
        """Stops every reader thread, closes every netlink socket, releases the lock.

        Idempotent; safe to call if never started. Each reader thread notices
        the stop request within one bus-wide conversion or one sensor's
        (module docstring, "Shutdown"), so the join below returns promptly;
        its 5 s bound is a backstop against a misconfigured timeout, not the
        common case.
        """
        self._stop.set()
        for thread in self._threads:
            thread.join(timeout=5.0)
        self._threads.clear()
        for bus_name in list(self._transports):
            self._close_transport(bus_name)
        self._reader_lock.release()

    # -- cross-process reader lock (tools/w1_commission.py) ----------------------

    def acquire_reader_lock(self) -> ReaderLockOutcome:
        """Tries to take :attr:`lock_path`'s :class:`ReaderLock`; see
        :class:`ReaderLockOutcome`. Used directly by ``tools/w1_commission.py
        --check`` (never by :meth:`read` or a cycle) before it drives a
        single cycle of its own; :meth:`start` uses the same instance for the
        daemon's own run, so only one of the two ever actually opens it on a
        given :class:`W1Source`.
        """
        return self._reader_lock.try_acquire()

    def release_reader_lock(self) -> None:
        """Releases the lock :meth:`acquire_reader_lock` took; idempotent."""
        self._reader_lock.release()

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

    def read_stats(self) -> dict[str, ReadStats]:
        """ROM id -> its read outcomes since construction (:class:`ReadStats`).

        Every declared ROM id has an entry, whether or not any cycle ever
        reached it: ``attempts`` is what a failure rate has to be measured
        against, and a sensor with none was never on a bus to be read
        (``tools/w1_commission.py --check`` prints the difference). Snapshot
        copies, so a caller cannot move our counters; taken without the
        publish lock, so a concurrent cycle may be counted mid-sensor -- these
        are diagnostics, not the reading path.
        """
        return {rom: replace(stats) for rom, stats in self._read_stats.items()}

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
    tier_failures_before_demote = section.get(
        "tier_failures_before_demote", _DEFAULT_TIER_FAILURES_BEFORE_DEMOTE
    )
    if not isinstance(tier_failures_before_demote, int) or isinstance(
        tier_failures_before_demote, bool
    ):
        raise ConfigError(
            "onewire.tier_failures_before_demote must be an int, got "
            f"{type(tier_failures_before_demote).__name__}"
        )
    slow_cycle_log_interval_s = section.get(
        "slow_cycle_log_interval_s", _DEFAULT_SLOW_CYCLE_LOG_INTERVAL_S
    )
    if not isinstance(slow_cycle_log_interval_s, int | float) or isinstance(
        slow_cycle_log_interval_s, bool
    ):
        raise ConfigError(
            "onewire.slow_cycle_log_interval_s must be a number, got "
            f"{type(slow_cycle_log_interval_s).__name__}"
        )
    root = section.get("root", DEFAULT_ROOT)
    lock_path = section.get("lock_path", _DEFAULT_LOCK_PATH)
    if not isinstance(lock_path, str) or not lock_path:
        raise ConfigError(f"onewire.lock_path must be a non-empty string, got {lock_path!r}")

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
            tier_failures_before_demote=int(tier_failures_before_demote),
            slow_cycle_log_interval_s=float(slow_cycle_log_interval_s),
            lock_path=lock_path,
        )
    except ConfigError as exc:
        raise ConfigError(f"onewire: {exc}") from exc
