"""aquaero 6 XT hardware adapter over the Linux hwmon sysfs ABI.

PROJECT.md section 3 (Track B) / section 4.7 / section 2 (Risk, USB spike).

ABI of the ``aquacomputer_d5next`` driver (not built into Raspberry Pi OS
kernels; ``deploy/install-aquacomputer-dkms.sh`` installs it). Names and
units are confirmed on USB-attached devices, which have no ``pwmK_enable``
(PROJECT.md section 2, "USB spike results"):

* ``tempK_input``  -- millidegrees Celsius, read-only
* ``fanK_input``   -- RPM, read-only
* ``pwmK``         -- 0..255, read/write
* ``pwmK_enable``  -- optional; ``1`` means "manual" (accepts our writes)

Files may be missing, unreadable, or contain garbage at any moment (a
loose USB connection glitches single reads without the device
disappearing); the whole device directory may vanish (USB dropout) or
reappear renumbered after a re-plug. :class:`HwmonMap` re-resolves the
device directory on every call so a re-plug is transparent here.

This module must not import :mod:`aqua_bridge.control` (or anything
MPC) -- see the static AST check in ``tests/test_hw_map.py`` /
``tests/test_hw_xt6.py``.
"""

from __future__ import annotations

import math
import re
from collections.abc import Callable, Mapping, Sequence
from pathlib import Path
from typing import Any

from aqua_bridge.hw.map import DeviceUnavailable, HwmonMap, ResolvedPaths
from aqua_bridge.model import ConfigError, MpcCommand, PlantObservation

__all__ = ["DeviceUnavailable", "Xt6Adapter", "build_map_from_config"]

_PWM_MAX_RAW = 255


class Xt6Adapter:
    """Reads/writes one aquaero (+ aquabus Quadro) hwmon device.

    Parameters
    ----------
    hwmon_map:
        A :class:`~aqua_bridge.hw.map.HwmonMap` describing where logical
        names live in sysfs.
    clock:
        Zero-argument callable returning monotonic seconds, injected so
        tests control ``PlantObservation.ts`` (section 3: "no
        ``time.time()`` inside the solver" -- the adapter follows the
        same discipline so a recorded observation's ``ts`` is
        reproducible in tests).
    """

    def __init__(self, hwmon_map: HwmonMap, clock: Callable[[], float]) -> None:
        self._map = hwmon_map
        self._clock = clock
        # Original pwmK_enable contents, captured the first time we see
        # each channel -- restored by release() (spike question 3:
        # hand control back to firmware curves).
        self._enable_original: dict[str, str] = {}
        # True once apply() has run at least once (release() is a no-op before).
        self._enable_initialized = False

    # -- read -----------------------------------------------------------

    def read(self) -> PlantObservation:
        """Read one sample. Raises :class:`DeviceUnavailable` if the whole
        device is gone; a single bad/missing file becomes ``None`` for
        that channel, never an exception.
        """
        resolved = self._map.resolve(check_files=False)
        device_dir = resolved.device_dir

        temps = {
            name: self._read_scaled(path, device_dir, 1000.0)
            for name, path in resolved.temp.items()
        }
        pwm = {
            ch: self._read_scaled(path, device_dir, float(_PWM_MAX_RAW))
            for ch, path in resolved.pwm.items()
        }
        rpm = {name: self._read_raw(path, device_dir) for name, path in resolved.fan.items()}
        return PlantObservation(temps=temps, rpm=rpm, pwm=pwm, ts=self._clock())

    def _read_raw(self, path: Path, device_dir: Path) -> float | None:
        try:
            text = path.read_text()
        except OSError as exc:
            self._raise_if_device_gone(device_dir, exc)
            return None
        try:
            value = float(text.strip())
        except ValueError:
            return None
        if not math.isfinite(value):
            return None
        return value

    def _read_scaled(self, path: Path, device_dir: Path, divisor: float) -> float | None:
        raw = self._read_raw(path, device_dir)
        if raw is None:
            return None
        return raw / divisor

    @staticmethod
    def _raise_if_device_gone(device_dir: Path, cause: OSError) -> None:
        """A single unreadable file is reported as ``None``; a vanished
        device directory means the USB link itself dropped."""
        if not device_dir.is_dir():
            raise DeviceUnavailable(f"hwmon device directory gone: {device_dir}") from cause

    # -- write ------------------------------------------------------------

    def apply(self, cmd: MpcCommand) -> None:
        """Write ``cmd.pwm`` for every configured channel.

        Only channels present in the map are touched. Every configured
        channel must have a finite value in ``[0, 1]`` in ``cmd.pwm`` --
        clamping is not allowed to hide a bad command from a caller, so
        an out-of-range or non-finite value raises :class:`ValueError`
        instead of being silently coerced.
        """
        resolved = self._map.resolve(check_files=False)
        device_dir = resolved.device_dir

        values: dict[str, int] = {}
        for channel in resolved.pwm:
            if channel not in cmd.pwm:
                raise ValueError(f"apply: command has no pwm value for channel {channel!r}")
            value = cmd.pwm[channel]
            if not isinstance(value, int | float) or isinstance(value, bool):
                raise ValueError(f"apply: pwm[{channel!r}] must be a number, got {value!r}")
            if not math.isfinite(value):
                raise ValueError(f"apply: pwm[{channel!r}] is not finite: {value!r}")
            if not 0.0 <= value <= 1.0:
                raise ValueError(f"apply: pwm[{channel!r}] out of range [0, 1]: {value!r}")
            values[channel] = round(value * _PWM_MAX_RAW)

        self._ensure_manual_mode(resolved, device_dir)

        for channel, raw in values.items():
            path = resolved.pwm[channel]
            self._write_text(path, device_dir, str(raw))

    def _ensure_manual_mode(self, resolved: ResolvedPaths, device_dir: Path) -> None:
        """Make sure every configured channel's ``pwmK_enable`` reads manual
        (``1``) before this write, remembering the value first seen for
        :meth:`release`.

        Re-checked on **every** ``apply`` (one small read per channel), not
        only before the first write: a USB dropout followed by a re-plug
        resets ``pwmK_enable`` to the firmware value while the daemon keeps
        running, and writing ``pwmK`` into a channel the firmware controls
        would look like success while the fans follow the firmware curve.
        Only a value other than ``1`` is written back.
        """
        for channel in resolved.pwm:
            enable_path = resolved.pwm_enable(channel)
            try:
                current = enable_path.read_text().strip()
            except OSError:
                continue  # no _enable file for this channel; nothing to restore
            if channel not in self._enable_original:
                self._enable_original[channel] = current
            if current != "1":
                self._write_text(enable_path, device_dir, "1")
        self._enable_initialized = True

    def _write_text(self, path: Path, device_dir: Path, text: str) -> None:
        try:
            path.write_text(text)
        except OSError as exc:
            self._raise_if_device_gone(device_dir, exc)
            raise DeviceUnavailable(f"failed to write {path}: {exc}") from exc

    def release(self) -> None:
        """Restore every ``pwmK_enable`` this adapter changed to what it
        was before the first write, handing control back to firmware
        curves (spike question 3). No-op if ``apply`` was never called.
        """
        if not self._enable_initialized:
            return
        try:
            resolved = self._map.resolve(check_files=False)
        except DeviceUnavailable:
            self._enable_original.clear()
            self._enable_initialized = False
            return
        for channel, original in self._enable_original.items():
            if channel not in resolved.pwm:
                continue
            enable_path = resolved.pwm_enable(channel)
            try:
                enable_path.write_text(original)
            except OSError:
                continue
        self._enable_original.clear()
        self._enable_initialized = False


def _check_keys(label: str, mapping: Mapping[str, Any], expected: Sequence[str], what: str) -> None:
    missing = sorted(set(expected) - set(mapping))
    extra = sorted(set(mapping) - set(expected))
    if missing or extra:
        raise ConfigError(
            f"xt6.{label} keys must equal {what} {sorted(expected)}: "
            f"missing {missing}, extra {extra}"
        )


#: Keys allowed inside one ``xt6.fans`` entry.
_FAN_ENTRY_KEYS = ("pwm", "rpm")
#: hwmon attribute names accepted per role (bare names, no ``_input`` suffix).
_ATTR_PATTERNS = {
    "pwm": re.compile(r"pwm[1-9][0-9]*"),
    "rpm": re.compile(r"fan[1-9][0-9]*"),
    "temp": re.compile(r"temp[1-9][0-9]*"),
}
#: Pre-``fans`` keys and what replaced them.
_LEGACY_KEYS = {
    "map": "xt6.fans.<channel>.pwm",
    "fan_map": "xt6.fans.<channel>.rpm",
}
_FANS_EXAMPLE = "radiator: {pwm: pwm1, rpm: fan1}"


def _check_attr(where: str, role: str, value: Any) -> str:
    if not isinstance(value, str) or not _ATTR_PATTERNS[role].fullmatch(value):
        expected = {"pwm": "pwmN", "rpm": "fanN", "temp": "tempN"}[role]
        raise ConfigError(
            f"{where} must be a hwmon attribute name like {expected!r}, got {value!r}"
        )
    return value


def _parse_fans(xt6_section: Mapping[str, Any]) -> tuple[dict[str, str], dict[str, str]]:
    """``xt6.fans`` -> ``(pwm_map, fan_map)``, both keyed by the channel name.

    One entry per fan names the channel once and carries both attributes,
    so a PWM output and its tachometer can never end up under two different
    (for example misspelt) names::

        fans:
          radiator: {pwm: pwm1, rpm: fan1}
          intake:   {pwm: pwm2}            # rpm is optional
    """
    for legacy, replacement in _LEGACY_KEYS.items():
        if legacy in xt6_section:
            raise ConfigError(
                f"xt6.{legacy} is no longer supported; use {replacement}, one entry per fan: "
                f"'fans: {{{_FANS_EXAMPLE}}}'"
            )
    fans = xt6_section.get("fans")
    if fans is None:
        raise ConfigError(
            f"xt6.fans is required: one entry per fan channel, e.g. '{_FANS_EXAMPLE}'"
        )
    if not isinstance(fans, Mapping):
        raise ConfigError(f"xt6.fans must be a mapping, got {type(fans).__name__}")
    pwm_map: dict[str, str] = {}
    fan_map: dict[str, str] = {}
    for channel, entry in fans.items():
        if not isinstance(channel, str) or not channel:
            raise ConfigError(f"xt6.fans keys must be non-empty channel names, got {channel!r}")
        where = f"xt6.fans.{channel}"
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
            raise ConfigError(f"{where}.pwm is required (the hwmon PWM attribute, e.g. 'pwm1')")
        pwm_map[channel] = _check_attr(f"{where}.pwm", "pwm", entry["pwm"])
        if entry.get("rpm") is not None:
            fan_map[channel] = _check_attr(f"{where}.rpm", "rpm", entry["rpm"])
    return pwm_map, fan_map


def build_map_from_config(
    xt6_section: Mapping[str, Any],
    *,
    channels: Sequence[str] | None = None,
    temps: Sequence[str] | None = None,
) -> HwmonMap:
    """Convenience constructor from the ``xt6:`` config section.

    Not part of the hot read/apply path; kept here so the glue/loop code
    does not need to know the exact ``HwmonMap`` field names.

    Section shape::

        xt6:
          hwmon_name: aquaero
          fans:                       # one entry per fan channel
            radiator: {pwm: pwm1, rpm: fan1}
            intake:   {pwm: pwm2, rpm: fan2}
          temp_map:                   # logical temperature -> tempN
            coolant: temp1
          root: /sys/class/hwmon      # optional

    Each fan names its channel once, with the PWM attribute and the
    optional tachometer attribute side by side (see :func:`_parse_fans`).
    The old ``map`` / ``fan_map`` pair is rejected with a hint: a typo in
    one of two repeated names used to split one fan into a PWM-only channel
    and an RPM reading nobody looks at. Unknown keys inside an entry
    (``rmp:``) and attribute names of the wrong kind (``rpm: pwm1``) are
    config errors too.

    ``channels`` / ``temps`` are the ``mpc`` section's tuples (passed as plain
    sequences: this module must not know ``MpcConfig``). When given, the
    ``fans`` keys must equal ``channels`` and the ``temp_map`` keys must equal
    ``temps``, or :class:`~aqua_bridge.model.ConfigError` is raised at
    startup. Without the check a channel missing from ``fans`` is silently
    never written (the fan stays on whatever the firmware last had while
    the controller believes it commands it), and a temperature missing from
    or extra in ``temp_map`` makes every observation fail the gate, so the
    daemon would sit in permanent fallback with no visible config error
    (section 4.2 "no silent drop of a channel", section 3 gate rule 2).
    """
    if not isinstance(xt6_section, Mapping):
        raise ConfigError(f"xt6 section must be a mapping, got {type(xt6_section).__name__}")
    if not xt6_section.get("hwmon_name"):
        raise ConfigError("xt6.hwmon_name is required (the hwmon 'name' file content)")
    pwm_map, fan_map = _parse_fans(xt6_section)
    temp_section = xt6_section.get("temp_map")
    if temp_section is not None and not isinstance(temp_section, Mapping):
        raise ConfigError(f"xt6.temp_map must be a mapping, got {type(temp_section).__name__}")
    temp_map = {
        name: _check_attr(f"xt6.temp_map.{name}", "temp", attr)
        for name, attr in dict(temp_section or {}).items()
    }
    if channels is not None:
        _check_keys("fans", pwm_map, channels, "mpc.channels")
    if temps is not None:
        _check_keys("temp_map", temp_map, temps, "mpc.temps")
    kwargs: dict[str, Any] = {
        "hwmon_name": xt6_section["hwmon_name"],
        "pwm_map": pwm_map,
        "temp_map": temp_map,
        "fan_map": fan_map,
    }
    if "root" in xt6_section:
        kwargs["root"] = xt6_section["root"]
    try:
        return HwmonMap(**kwargs)
    except (TypeError, ValueError) as exc:
        raise ConfigError(f"xt6 section: {exc}") from exc
