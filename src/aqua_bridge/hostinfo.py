"""Host machine metrics for the HTTP "Host" page and the MQTT host sensors.

Every reader takes an injectable path/root so tests can point at a fake
sysfs/procfs tree instead of the real one; a missing or malformed file
yields ``None`` for that one metric, never an exception. Callers combine
the metrics into a single JSON-serialisable dict with :func:`collect_hostinfo`.

:class:`CachedHostInfo` wraps a reader (:func:`collect_hostinfo` by default)
with a refresh interval, so a caller polled more often than ``host.interval_s``
(the HTTP ``/api/state`` route, the MQTT per-tick publisher) does not re-read
``/proc`` and ``/sys`` on every call.
"""

from __future__ import annotations

import logging
import os
import re
import time
from collections.abc import Callable, Mapping
from pathlib import Path
from typing import Any

__all__ = [
    "CachedHostInfo",
    "collect_hostinfo",
    "read_cpu_temp_c",
    "read_disk",
    "read_loadavg",
    "read_memory",
    "read_uptime_s",
    "read_wifi_rssi",
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


def collect_hostinfo(
    *,
    thermal_root: Path = Path("/sys/class/thermal"),
    loadavg_path: Path = Path("/proc/loadavg"),
    meminfo_path: Path = Path("/proc/meminfo"),
    uptime_path: Path = Path("/proc/uptime"),
    disk_path: str | os.PathLike[str] = "/",
    wireless_path: Path = Path("/proc/net/wireless"),
    wifi_iface: str | None = None,
) -> dict[str, Any]:
    """Collect every host metric into one JSON-serialisable dict.

    Missing/unreadable sources leave their key ``None``; this never raises.
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
    }


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
