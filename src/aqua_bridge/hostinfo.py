"""Host machine metrics for the HTTP "Host" page and the MQTT host sensors.

Every reader takes an injectable path/root so tests can point at a fake
sysfs/procfs tree instead of the real one; a missing or malformed file
yields ``None`` for that one metric, never an exception. Callers combine
the metrics into a single JSON-serialisable dict with :func:`collect_hostinfo`.

:class:`CachedHostInfo` wraps a reader (:func:`collect_hostinfo` by default)
with a refresh interval, so a caller polled more often than ``host.interval_s``
(the HTTP ``/api/state`` route, the MQTT per-tick publisher) does not re-read
``/proc`` and ``/sys`` on every call.

Besides the metrics, :func:`read_throttled` reports the Raspberry Pi's throttling
state -- under-voltage, a capped ARM frequency, hard throttling and the soft
temperature limit, each both *now* and *since boot*. It rides ``collect_hostinfo``'s
``throttled`` key and feeds the host-health rules (:mod:`aqua_bridge.health`,
PROJECT.md section 8 item 103). Like the board's own temperature it is a health
signal only: nothing here ever reaches the solver.

Three sources, best first, because no single one is present everywhere:

1. the firmware driver's sysfs attribute :data:`THROTTLED_SYSFS` -- the whole word
   for a file read, but **absent** on a Raspberry Pi Zero 2 W running kernel 6.18,
   where the ``soc:firmware`` platform device carries no such attribute;
2. ``vcgencmd get_throttled`` -- the whole word, and on that kernel the only source
   of it, but a process: 3.3 ms median / 3.8 ms p95 on that board. Rate limited by
   :class:`ThrottledReader` so a caller polled every tick forks once a minute;
3. the ``rpi_volt`` hwmon device's ``in0_lcrit_alarm`` (:func:`read_rpi_volt_hwmon`)
   -- the under-voltage condition *only*, 0.17 ms, found by the device's ``name``
   because hwmon numbering is not stable across boots.

A source that reads less than the whole word says so: the conditions it did not
read are ``None``, ``partial`` is true and ``unknown`` names them. An unknown bit is
never reported as a false one. A board with none of the three sources gets ``None``
and warns about nothing.

Besides the throttling state, :func:`read_mount_ro` reports whether the filesystem
under a given path is mounted read-only -- the kernel's own report, not a write
probe: on an SD card the first sign of a dying card is usually the kernel
remounting the root filesystem read-only after an I/O error (``errors=remount-ro``),
and that state already shows up in ``/proc/mounts`` without this daemon writing
anything to find out. It feeds :mod:`aqua_bridge.health`'s host-health rules
(PROJECT.md section 8, "the disk nobody watches") next to ``disk_used_pct`` and
``disk_free_gb``, which ``collect_hostinfo`` already read. Like the throttling word,
a reading that cannot be determined is ``None``, never guessed either way.
"""

from __future__ import annotations

import logging
import os
import re
import shutil
import subprocess
import time
from collections.abc import Callable, Mapping, Sequence
from pathlib import Path
from typing import Any

__all__ = [
    "HWMON_ROOT",
    "MOUNTS_PATH",
    "RPI_VOLT_HWMON_NAME",
    "THROTTLED_BITS",
    "THROTTLED_SINCE_BOOT_SHIFT",
    "THROTTLED_SYSFS",
    "UNDER_VOLTAGE_ALARM",
    "CachedHostInfo",
    "ThrottledReader",
    "collect_hostinfo",
    "decode_throttled",
    "read_cpu_temp_c",
    "read_disk",
    "read_loadavg",
    "read_memory",
    "read_mount_ro",
    "read_rpi_volt_hwmon",
    "read_throttled",
    "read_throttled_sysfs",
    "read_uptime_s",
    "read_wifi_rssi",
    "run_vcgencmd",
]

_LOG = logging.getLogger(__name__)


def _read_text(path: Path) -> str | None:
    try:
        return path.read_text(encoding="utf-8")
    except OSError:
        return None


def read_cpu_temp_c(thermal_root: Path = Path("/sys/class/thermal")) -> float | None:
    """CPU temperature in degrees C from the first CPU-ish thermal zone.

    Prefers a zone whose ``type`` mentions "cpu" (case-insensitively);
    falls back to ``thermal_zone0``. Values are millidegrees C on Linux.
    """
    if not thermal_root.is_dir():
        return None
    zones = sorted(p for p in thermal_root.glob("thermal_zone*") if p.is_dir())
    if not zones:
        return None
    best = None
    for zone in zones:
        type_text = _read_text(zone / "type")
        if type_text is not None and "cpu" in type_text.strip().lower():
            best = zone
            break
    if best is None:
        best = zones[0]
    raw = _read_text(best / "temp")
    if raw is None:
        return None
    try:
        millideg = float(raw.strip())
    except ValueError:
        return None
    return millideg / 1000.0


def read_loadavg(path: Path = Path("/proc/loadavg")) -> tuple[float, float, float] | None:
    """(load1, load5, load15) from ``/proc/loadavg``."""
    raw = _read_text(path)
    if raw is None:
        return None
    parts = raw.split()
    if len(parts) < 3:
        return None
    try:
        return float(parts[0]), float(parts[1]), float(parts[2])
    except ValueError:
        return None


def read_memory(path: Path = Path("/proc/meminfo")) -> dict[str, float] | None:
    """``{"total_kb", "available_kb", "used_pct"}`` from ``/proc/meminfo``."""
    raw = _read_text(path)
    if raw is None:
        return None
    values: dict[str, float] = {}
    for line in raw.splitlines():
        m = re.match(r"^(\w+):\s*(\d+)\s*kB", line)
        if m:
            values[m.group(1)] = float(m.group(2))
    total = values.get("MemTotal")
    available = values.get("MemAvailable")
    if total is None or available is None or total <= 0:
        return None
    used_pct = 100.0 * (total - available) / total
    return {"total_kb": total, "available_kb": available, "used_pct": used_pct}


def read_uptime_s(path: Path = Path("/proc/uptime")) -> float | None:
    """Seconds since boot from ``/proc/uptime``."""
    raw = _read_text(path)
    if raw is None:
        return None
    parts = raw.split()
    if not parts:
        return None
    try:
        return float(parts[0])
    except ValueError:
        return None


def read_disk(path: str | os.PathLike[str] = "/") -> dict[str, float] | None:
    """``{"total_gb", "free_gb", "used_pct"}`` via ``os.statvfs(path)``."""
    try:
        st = os.statvfs(path)
    except OSError:
        return None
    total = st.f_frsize * st.f_blocks
    free = st.f_frsize * st.f_bavail
    if total <= 0:
        return None
    used_pct = 100.0 * (total - free) / total
    return {
        "total_gb": total / (1024**3),
        "free_gb": free / (1024**3),
        "used_pct": used_pct,
    }


#: Where the kernel lists every mount, its point and its current options --
#: including ``ro``, which a filesystem gains on its own when the kernel remounts it
#: read-only after an I/O error (``errors=remount-ro``, the usual ``ext4`` default).
#: A plain file, refreshed by the kernel on every mount change; reading it costs
#: nothing and writes nothing to the card :func:`read_mount_ro` is asking about.
MOUNTS_PATH = Path("/proc/mounts")


def read_mount_ro(
    path: str | os.PathLike[str] = "/", mounts_path: Path = MOUNTS_PATH
) -> bool | None:
    """Whether the filesystem carrying ``path`` is mounted read-only, or ``None``
    when that cannot be determined.

    Reads it from the kernel's own ``/proc/mounts`` rather than probing with a
    write: the failure this exists to catch -- an SD card the kernel has already
    remounted read-only after an I/O error -- is already a fact in that table the
    moment it happens, and a write probe would add both a write and a fsync, on
    every check, to the very card a dying-card rule is trying to protect. The kernel
    updates ``/proc/mounts`` synchronously on every mount and remount, so there is no
    staleness a poll interval would need to cover (contrast
    :class:`ThrottledReader`, whose ``vcgencmd`` source is rate limited because it
    forks a process; this one never does).

    ``path`` is matched to the mount whose mount point is the longest prefix of it
    (the same rule the kernel itself uses to answer a lookup), so a path on a
    sub-mount is judged by its own entry and not the root filesystem's; on two
    entries for the same mount point (a remount, which appends a new line rather
    than rewriting the old one) the later entry wins, since that is the one in
    effect. Returns ``None`` -- never guessed as read-only or as read-write --
    when ``mounts_path`` does not read, is empty, or names no mount point that is a
    prefix of ``path``; never raises.
    """
    text = _read_text(mounts_path)
    if text is None:
        return None
    try:
        resolved = os.path.realpath(path)
    except OSError:
        return None
    best_point: str | None = None
    best_ro: bool | None = None
    for line in text.splitlines():
        fields = line.split()
        if len(fields) < 4:
            continue
        mount_point, options = fields[1], fields[3]
        if mount_point == "/":
            matches = True
        else:
            matches = resolved == mount_point or resolved.startswith(mount_point + "/")
        if not matches:
            continue
        if best_point is None or len(mount_point) >= len(best_point):
            best_point = mount_point
            best_ro = "ro" in options.split(",")
    return best_ro


def read_wifi_rssi(
    path: Path = Path("/proc/net/wireless"), iface: str | None = None
) -> float | None:
    """Wi-Fi signal level in dBm from ``/proc/net/wireless``.

    Picks ``iface`` when given, otherwise the first interface listed.
    Format (two header lines, then one line per interface)::

        Inter-|sta-|   Quality        |   Discarded packets  ...
         face |tus | link level noise |  nwid  crypt ...
         wlan0: 0000   50.  -60.  -256        0 ...
    """
    raw = _read_text(path)
    if raw is None:
        return None
    for line in raw.splitlines()[2:]:
        line = line.strip()
        if not line:
            continue
        name, _, rest = line.partition(":")
        name = name.strip()
        if iface is not None and name != iface:
            continue
        fields = rest.split()
        if len(fields) < 3:
            continue
        try:
            return float(fields[2].rstrip("."))
        except ValueError:
            continue
    return None


# ---------------------------------------------------------------------------
# Throttling (PROJECT.md section 8 item 103)
# ---------------------------------------------------------------------------

#: The Raspberry Pi firmware's ``get_throttled`` word, low half: ``(bit, name)``
#: of the conditions in force *now*.
THROTTLED_BITS: tuple[tuple[int, str], ...] = (
    (0, "under_voltage"),
    (1, "freq_capped"),
    (2, "throttled"),
    (3, "soft_temp_limit"),
)

#: The same four conditions repeat this many bits up as "has occurred since boot".
THROTTLED_SINCE_BOOT_SHIFT = 16

#: Where the firmware driver exposes the *whole* word without ``vcgencmd``: an
#: attribute of the ``raspberrypi-firmware`` platform device, one ``0x``-prefixed hex
#: number. Preferred where it exists, but it is not the usual case: on a Raspberry Pi
#: Zero 2 W running kernel 6.18 the ``soc:firmware`` platform device is there and
#: carries no ``get_throttled`` attribute at all (nothing under ``/sys`` is named for
#: throttling on that kernel), which is why the chain below has two more sources.
THROTTLED_SYSFS = Path("/sys/devices/platform/soc/soc:firmware/get_throttled")

#: Where the kernel exposes the under-voltage condition -- and only that one -- as a
#: plain file: the ``rpi_volt`` hwmon device's ``in0_lcrit_alarm``, ``0`` or ``1``.
#: Found by the device's ``name``, never by its index: hwmon numbering is not stable
#: across boots (on the owner's board ``hwmon0`` is ``cpu_thermal`` and ``hwmon1`` is
#: ``rpi_volt`` today, and nothing promises that tomorrow).
HWMON_ROOT = Path("/sys/class/hwmon")

#: The ``name`` of the hwmon device that carries :data:`UNDER_VOLTAGE_ALARM`.
RPI_VOLT_HWMON_NAME = "rpi_volt"

#: The under-voltage alarm attribute of that device.
UNDER_VOLTAGE_ALARM = "in0_lcrit_alarm"


def _summary(values: Sequence[bool | None]) -> bool | None:
    """``True`` if any condition holds, ``False`` if none does *and* all were read,
    ``None`` while a condition nobody read could be the one in force.

    An unknown bit never reads as false: a summary over a partial reading that saw
    nothing is ``None``, not "fine".
    """
    if any(value is True for value in values):
        return True
    return None if any(value is None for value in values) else False


def _throttled_reading(
    now_bits: Mapping[str, bool | None],
    since_bits: Mapping[str, bool | None],
    *,
    word: int | None = None,
    source: str | None = None,
    age_s: float | None = None,
) -> dict[str, Any]:
    """One reading of the board's throttling state, however much of it was read.

    Every condition :data:`THROTTLED_BITS` names gets a ``<name>_now`` and a
    ``<name>_since_boot`` key, each ``True``, ``False`` or -- for a source that
    could not see that bit -- ``None``. ``now`` / ``since_boot`` summarise them
    (:func:`_summary`), ``unknown`` lists the conditions whose *now* bit this source
    did not read and ``partial`` is true whenever any bit is unknown, so a consumer
    never has to infer "not read" from a false. ``raw``/``hex`` carry the word where
    one was read and are ``None`` otherwise -- a partial reading invents no word.
    ``source`` names where it came from and ``age_s`` how long ago it was read
    (``0.0`` fresh, ``None`` where the question does not apply).
    """
    out: dict[str, Any] = {
        "raw": None if word is None else int(word),
        "hex": None if word is None else f"0x{int(word):x}",
        "source": source,
        "age_s": None if age_s is None else float(age_s),
    }
    for _, name in THROTTLED_BITS:
        out[f"{name}_now"] = now_bits.get(name)
        out[f"{name}_since_boot"] = since_bits.get(name)
    names = [name for _, name in THROTTLED_BITS]
    out["now"] = _summary([out[f"{name}_now"] for name in names])
    out["since_boot"] = _summary([out[f"{name}_since_boot"] for name in names])
    out["unknown"] = [name for name in names if out[f"{name}_now"] is None]
    out["partial"] = any(
        out[f"{name}_{half}"] is None for name in names for half in ("now", "since_boot")
    )
    return out


def decode_throttled(
    word: int, *, source: str | None = None, age_s: float | None = None
) -> dict[str, Any]:
    """Decode a whole ``get_throttled`` word into named booleans.

    ``<name>_now`` is the condition in force at the read, ``<name>_since_boot``
    the latched "has occurred" half :data:`THROTTLED_SINCE_BOOT_SHIFT` bits up;
    ``now`` / ``since_boot`` are the two summaries. ``raw`` keeps the word and
    ``hex`` its usual spelling, so a bit this table does not name is still visible
    in the payload. Nothing here is unknown -- the word carries every condition --
    so ``partial`` is false and ``unknown`` empty; see :func:`_throttled_reading`
    for the sources that read less than the whole word.
    """
    now = {name: bool(word & (1 << bit)) for bit, name in THROTTLED_BITS}
    since = {
        name: bool(word & (1 << (bit + THROTTLED_SINCE_BOOT_SHIFT))) for bit, name in THROTTLED_BITS
    }
    return _throttled_reading(now, since, word=int(word), source=source, age_s=age_s)


def _parse_throttled_word(text: str) -> int | None:
    """``0x50005`` / ``throttled=0x0`` / ``0`` -> the word, anything else ``None``."""
    tail = text.strip().rpartition("=")[2].strip()
    if not tail:
        return None
    try:
        word = int(tail, 0)
    except ValueError:
        return None
    return word if word >= 0 else None


def run_vcgencmd(timeout_s: float = 2.0) -> str | None:
    """``vcgencmd get_throttled``'s output, or ``None`` when there is no such binary.

    The only source of the *whole* word on a kernel that exposes no sysfs attribute,
    and the only one that costs a process: measured at 3.3 ms median / 3.8 ms p95 on
    a Raspberry Pi Zero 2 W (kernel 6.18, 4 cores at 1.0 GHz). :class:`ThrottledReader`
    is what keeps that off the tick more often than ``vcgencmd_interval_s``.

    Never raises: no ``vcgencmd`` on ``PATH`` (every machine that is not a Raspberry
    Pi, the test machines included) short-circuits before any process is started, and
    a failure, a timeout or a non-zero exit is ``None``. ``timeout_s`` is the guard
    against the one way this can hurt a caller that is on a thread with work to do:
    the VideoCore mailbox not answering, which would otherwise block indefinitely.
    """
    binary = shutil.which("vcgencmd")
    if binary is None:
        return None
    try:
        proc = subprocess.run(  # noqa: S603 - fixed argv, resolved binary, no shell
            [binary, "get_throttled"],
            capture_output=True,
            text=True,
            timeout=timeout_s,
            check=False,
        )
    except (OSError, subprocess.SubprocessError):
        return None
    return proc.stdout if proc.returncode == 0 else None


def read_throttled_sysfs(path: Path = THROTTLED_SYSFS) -> dict[str, Any] | None:
    """The whole word from the firmware driver's sysfs attribute, or ``None``.

    The cheapest complete source and so the first one tried -- a plain file read on
    the caller's own thread. Absent on kernel 6.18 (see :data:`THROTTLED_SYSFS`), so
    "``None`` here" is the normal case on the owner's board, not a fault.
    """
    text = _read_text(path)
    if text is None:
        return None
    word = _parse_throttled_word(text)
    return None if word is None else decode_throttled(word, source="sysfs")


def read_rpi_volt_hwmon(root: Path = HWMON_ROOT) -> dict[str, Any] | None:
    """The under-voltage condition alone, from the ``rpi_volt`` hwmon device.

    One bit, not the word: ``in0_lcrit_alarm`` says whether the board is under-volted
    *now* and nothing about the other three conditions or about anything since boot.
    The reading says so -- those bits are ``None``, ``partial`` is true and ``unknown``
    names them -- rather than reporting a zero nobody read. 0.17 ms median / 0.23 ms
    p95 on a Raspberry Pi Zero 2 W (kernel 6.18), against 3.3 ms for ``vcgencmd``.

    The device is found by its ``name`` across ``/sys/class/hwmon/hwmon*``: the index
    is not stable across boots, and on this board ``hwmon0`` is the ``cpu_thermal``
    zone the temperature already comes from. Returns ``None`` where no such device
    exists (every machine that is not a Raspberry Pi) or its attribute does not read.
    """
    if not root.is_dir():
        return None
    for entry in sorted(root.glob("hwmon*")):
        name = _read_text(entry / "name")
        if name is None or name.strip() != RPI_VOLT_HWMON_NAME:
            continue
        raw = _read_text(entry / UNDER_VOLTAGE_ALARM)
        if raw is None:
            continue
        try:
            alarm = int(raw.strip(), 10)
        except ValueError:
            continue
        return _throttled_reading({"under_voltage": bool(alarm)}, {}, source="hwmon")
    return None


def _vcgencmd_word(runner: Callable[[], str | None]) -> int | None:
    """``runner()``'s output parsed into a word, or ``None``; never raises."""
    try:
        text = runner()
    except Exception:  # the runner is the caller's; a diagnostic must not raise
        _LOG.exception("vcgencmd get_throttled failed")
        return None
    return None if text is None else _parse_throttled_word(text)


def _throttled_chain(
    sysfs_path: Path,
    word_source: Callable[[], tuple[int | None, float | None]],
    hwmon_root: Path,
) -> dict[str, Any] | None:
    """The source chain, best first: the sysfs attribute's whole word, then whatever
    word ``word_source`` has (``vcgencmd``, possibly the last one it read), then the
    hwmon under-voltage bit on its own, then ``None`` -- a board with no source at
    all degrades to "unknown" and warns about nothing.
    """
    reading = read_throttled_sysfs(sysfs_path)
    if reading is not None:
        return reading
    word, age_s = word_source()
    if word is not None:
        return decode_throttled(word, source="vcgencmd", age_s=age_s)
    return read_rpi_volt_hwmon(hwmon_root)


def read_throttled(
    path: Path = THROTTLED_SYSFS,
    *,
    vcgencmd: Callable[[], str | None] | None = None,
    hwmon_root: Path = HWMON_ROOT,
) -> dict[str, Any] | None:
    """The board's throttling state, decoded, or ``None`` when no source reads.

    The one-shot form of the chain (:func:`_throttled_chain`), with no cache: the
    ``vcgencmd`` runner, when one is given at all, is called on every read where the
    sysfs attribute is missing. The daemon uses :class:`ThrottledReader` instead,
    which is the same chain with that one process rate limited. ``vcgencmd`` defaults
    to ``None`` -- a bare ``read_throttled()`` reads files and starts nothing.

    Like every other reader here this never raises: a machine that is not a Raspberry
    Pi simply has no source and gets ``None``.
    """

    def once() -> tuple[int | None, float | None]:
        if vcgencmd is None:
            return None, None
        return _vcgencmd_word(vcgencmd), 0.0

    return _throttled_chain(path, once, hwmon_root)


class ThrottledReader:
    """The source chain with the ``vcgencmd`` process rate limited: callable, returns
    what :func:`read_throttled` returns.

    Order: the sysfs attribute (whole word, a file read), then ``vcgencmd`` (whole
    word, a process) at most once per ``poll_interval_s`` with the last word it read
    served in between, then the ``rpi_volt`` hwmon alarm (the under-voltage condition
    only), then ``None``.

    The cadence is what makes ``vcgencmd`` affordable on the control-loop thread. One
    fork costs 3.3 ms median / 3.8 ms p95 on a Raspberry Pi Zero 2 W (kernel 6.18);
    at one poll per minute against ``dt = 5 s`` that is 3.3 ms in 60 s of wall clock,
    0.005 % of the loop's time and 0.5 % of one tick's 600 ms solver budget on the
    tick it actually runs. A word served from the cache carries its ``age_s``, so a
    consumer can see how old the bits it is reading are.

    A poll that fails (no binary, a timeout, a non-zero exit, a word that does not
    parse) drops the cached word rather than serving it on: a failed poll is not a
    reading. The next attempt still waits out the cadence, so a board where the
    binary hangs costs one timed-out call per ``poll_interval_s`` and not one per
    tick -- and the chain falls through to the hwmon bit in the meantime.

    One instance per caller; it is not thread-safe, and each caller (the tick's
    reader, the HTTP app's, the MQTT service's) builds its own in
    :func:`aqua_bridge.health.host_metrics_reader`.
    """

    def __init__(
        self,
        *,
        sysfs_path: Path = THROTTLED_SYSFS,
        hwmon_root: Path = HWMON_ROOT,
        vcgencmd: Callable[[], str | None] | None = None,
        poll_interval_s: float = 60.0,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        self._sysfs_path = sysfs_path
        self._hwmon_root = hwmon_root
        self._vcgencmd = vcgencmd
        self._poll_interval_s = max(0.0, float(poll_interval_s))
        self._clock = clock
        self._word: int | None = None
        self._word_at = 0.0
        self._polled_at = float("-inf")

    def __call__(self) -> dict[str, Any] | None:
        return _throttled_chain(self._sysfs_path, self._poll, self._hwmon_root)

    def _poll(self) -> tuple[int | None, float | None]:
        """The last ``vcgencmd`` word and its age, polling again when it is due."""
        if self._vcgencmd is None:
            return None, None
        now = self._clock()
        if now - self._polled_at < self._poll_interval_s:
            age_s = None if self._word is None else max(0.0, now - self._word_at)
            return self._word, age_s
        self._polled_at = now
        word = _vcgencmd_word(self._vcgencmd)
        if word is None:
            self._word = None
            return None, None
        self._word = word
        self._word_at = now
        return word, 0.0


def collect_hostinfo(
    *,
    thermal_root: Path = Path("/sys/class/thermal"),
    loadavg_path: Path = Path("/proc/loadavg"),
    meminfo_path: Path = Path("/proc/meminfo"),
    uptime_path: Path = Path("/proc/uptime"),
    disk_path: str | os.PathLike[str] = "/",
    mounts_path: Path = MOUNTS_PATH,
    wireless_path: Path = Path("/proc/net/wireless"),
    wifi_iface: str | None = None,
    throttled_path: Path = THROTTLED_SYSFS,
    hwmon_root: Path = HWMON_ROOT,
    throttled: Callable[[], dict[str, Any] | None] | None = None,
) -> dict[str, Any]:
    """Collect every host metric into one JSON-serialisable dict.

    Missing/unreadable sources leave their key ``None``; this never raises.
    ``throttled`` is the only nested value: the board's throttling state as
    :func:`read_throttled` returns it, ``None`` where no source reads. Pass a
    ``throttled`` callable -- a :class:`ThrottledReader`, which is what
    :func:`aqua_bridge.health.host_metrics_reader` builds -- to use a source chain
    that may start a ``vcgencmd`` process; left ``None`` the file sources are read
    and no process is ever started.

    ``read_only`` (:func:`read_mount_ro`) answers for the same ``disk_path`` that
    ``disk_used_pct``/``disk_free_gb`` describe -- the filesystem this daemon's
    recorder, model store and (on the Pi) journal all write to -- so the three
    numbers a health rule about that card needs come from one consistent mount.
    """
    load = read_loadavg(loadavg_path)
    mem = read_memory(meminfo_path)
    disk = read_disk(disk_path)
    return {
        "cpu_temp_c": read_cpu_temp_c(thermal_root),
        "load1": load[0] if load else None,
        "load5": load[1] if load else None,
        "load15": load[2] if load else None,
        "mem_used_pct": mem["used_pct"] if mem else None,
        "mem_total_kb": mem["total_kb"] if mem else None,
        "disk_used_pct": disk["used_pct"] if disk else None,
        "disk_free_gb": disk["free_gb"] if disk else None,
        "wifi_rssi_dbm": read_wifi_rssi(wireless_path, wifi_iface),
        "uptime_s": read_uptime_s(uptime_path),
        "throttled": _throttled(throttled, throttled_path, hwmon_root),
        "read_only": read_mount_ro(disk_path, mounts_path),
    }


def _throttled(
    reader: Callable[[], dict[str, Any] | None] | None,
    throttled_path: Path,
    hwmon_root: Path,
) -> dict[str, Any] | None:
    """``reader()`` where one is given, else the file-only chain; never raises."""
    if reader is None:
        return read_throttled(throttled_path, hwmon_root=hwmon_root)
    try:
        return reader()
    except Exception:  # the reader is the caller's; collect_hostinfo must not raise
        _LOG.exception("the throttled reader failed")
        return None


class CachedHostInfo:
    """``reader()`` (:func:`collect_hostinfo` by default), refreshed at most every
    ``interval_s`` seconds.

    ``get()`` never raises: a reader exception is logged and yields ``{}`` for
    that refresh, leaving the previous cached value in place from the next
    call on (so one failed refresh does not blank an otherwise working page).
    Not thread-safe by itself; each caller (the HTTP app, the MQTT service)
    owns one instance.
    """

    def __init__(
        self,
        *,
        interval_s: float = 5.0,
        reader: Callable[[], Mapping[str, Any]] = collect_hostinfo,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        self._interval_s = max(0.0, float(interval_s))
        self._reader = reader
        self._clock = clock
        self._cached: dict[str, Any] = {}
        self._at = float("-inf")

    def get(self) -> dict[str, Any]:
        now = self._clock()
        if now - self._at >= self._interval_s:
            self._at = now
            try:
                self._cached = dict(self._reader())
            except Exception:  # readers promise not to raise; belt and braces
                _LOG.exception("hostinfo reader failed")
                self._cached = {}
        return self._cached
