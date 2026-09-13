"""DS18B20 temperatures over the Linux ``w1_therm`` bulk-read sysfs ABI.

PROJECT.md section 3 (Track B) / the DAS plan section 1 "Read timing vs dt"
and section 12 risk 5 ("bulk 1-Wire on a busy single core").

Assumed sysfs layout (kernel docs, ``w1_therm``; **unverified** against real
hardware -- see the plan's "Kernel docs" note; kept name-based so a
different kernel version is a config change, not a code change) under
``onewire.root`` (default ``/sys/bus/w1/devices``)::

    <root>/w1_bus_master<N>/therm_bulk_read     write "trigger", read "0"|"1"|"-1"
    <root>/w1_bus_master<N>/<rom_id>/temperature   read-only, millidegrees C
    <root>/w1_bus_master<N>/<rom_id>/resolution    read/write, 9..12 (bits)

``onewire.buses:`` in config is documentation only (the overlays that create
these bus masters live in ``config.txt``, not here): this module discovers
every ``w1_bus_master*`` directory under ``root`` and, on every cycle, which
of the *declared* ROM ids currently live under each one -- it never needs to
know which GPIO a bus uses. A declared ROM id that is not found under any
bus master is a warning at :meth:`W1Source.start` (a sensor may legitimately
be unplugged with its drive, plan section 1 "Failure and redundancy"), never
fatal: its readings are simply ``None`` until it (re)appears.

One reader thread per bus master runs :meth:`W1Source.run_bus_cycle` in a
loop: write ``trigger``, poll the same file until it reads ``"1"`` (bulk
conversion done; a driver that never signals done times out at
``bulk_timeout_s`` and the affected sensors report ``None`` this cycle, not
an exception -- a stalled bus must not block the daemon), then read every
present slave's ``temperature`` file. A read that raises ``OSError`` (the
kernel reports EIO for a failed CRC) or does not parse as an integer becomes
``None`` for that sensor this cycle, counted in :meth:`crc_error_counts` for
``tools/w1_commission.py --check``.

:meth:`W1Source.read` (called from the control loop thread) never touches
the filesystem and never blocks: it returns the latest published sample for
every declared sensor, or ``None`` when the sensor has never reported, its
last read failed, or the sample is older than ``max_age_s``.

This module must not import :mod:`aqua_bridge.control` (or anything MPC) --
see the static AST check in ``tests/test_hw_map.py``. It imports only
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
_TEMPERATURE_FILE = "temperature"
_VALID_RESOLUTIONS = (9, 10, 11, 12)
_DEFAULT_RESOLUTION_BITS = 12
_DEFAULT_POLL_INTERVAL_S = 0.02
# 750 ms is the datasheet's 12-bit conversion time (the plan's "Assumed" note);
# this is a generous multiple so a slightly slow real bus is not mistaken for
# a stalled one, while a genuinely wedged driver still gives up well inside
# one tick at the recommended dt = 5 s.
_DEFAULT_BULK_TIMEOUT_S = 2.0


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
        write is not free and the value does not change on its own).
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

        self._lock = threading.Lock()
        self._latest: dict[str, _Sample] = {}
        self._configured_resolution: set[str] = set()
        self._crc_errors: dict[str, int] = dict.fromkeys(self._rom_to_names, 0)
        self._cycle_counts: dict[str, int] = {}
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

    # -- one bulk-read cycle (synchronous; also used directly by tests and tools) --

    def run_bus_cycle(self, bus_dir: Path) -> dict[str, float | None]:
        """Runs one trigger/poll/read cycle on ``bus_dir``.

        Returns ``{name: value_or_None}`` for every declared sensor name
        currently found under ``bus_dir`` (empty if none of the declared ROM
        ids live there). Never raises: every failure mode narrows to
        ``None`` for the sensor(s) it affects, per the module docstring.
        """
        present = {rom: bus_dir / rom for rom in self._rom_to_names if (bus_dir / rom).is_dir()}
        if not present:
            return {}
        self._ensure_resolution(present)
        names_present = [name for rom in present for name in self._rom_to_names[rom]]
        if not self._bulk_trigger_and_poll(bus_dir / _TRIGGER_FILE):
            return dict.fromkeys(names_present, None)
        result: dict[str, float | None] = {}
        for rom, slave_dir in present.items():
            value = self._read_temperature(slave_dir, rom)
            for name in self._rom_to_names[rom]:
                result[name] = value
        return result

    def _ensure_resolution(self, present: Mapping[str, Path]) -> None:
        for rom, slave_dir in present.items():
            if rom in self._configured_resolution:
                continue
            try:
                (slave_dir / _RESOLUTION_FILE).write_text(str(self._resolution_bits))
            except OSError as exc:
                _LOG.warning("onewire: cannot set resolution for %s: %s", rom, exc)
            self._configured_resolution.add(rom)

    def _bulk_trigger_and_poll(self, trigger_path: Path) -> bool:
        try:
            trigger_path.write_text("trigger")
        except OSError as exc:
            _LOG.warning("onewire: bulk trigger failed at %s: %s", trigger_path, exc)
            return False
        deadline = self._clock() + self._bulk_timeout_s
        while True:
            try:
                status = trigger_path.read_text().strip()
            except OSError as exc:
                _LOG.warning("onewire: bulk poll failed at %s: %s", trigger_path, exc)
                return False
            if status == "1":
                return True
            if self._clock() >= deadline:
                _LOG.warning("onewire: bulk read timed out at %s", trigger_path)
                return False
            self._stop.wait(self._poll_interval_s)

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
            # A real bulk conversion paces this loop on its own (~750 ms at 12
            # bit); this floor only guards the pathological/test case of a
            # driver (or fake) that reports "done" instantly every time, so
            # the thread never busy-spins a core.
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
        """Completed bulk-read cycles per bus master name since :meth:`start`."""
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
    root = section.get("root", DEFAULT_ROOT)

    try:
        return W1Source(
            clean,
            resolution_bits=resolution_bits,
            max_age_s=float(max_age_s),
            root=root,
            clock=clock,
        )
    except ConfigError as exc:
        raise ConfigError(f"onewire: {exc}") from exc
