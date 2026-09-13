"""Logical name -> hwmon sysfs file resolution (PROJECT.md section 3, Track B).

The aquaero (and Quadro, over aquabus) show up under Linux as one hwmon
device (assumed driver: ``aquacomputer_d5next``; to be confirmed by the
USB spike in section 2 -- keep everything here name-based and configurable
so a different driver name or file layout is a config change, not a code
change).

hwmon device numbers are **not stable**: a re-plug, a driver reload, or
another hwmon-exposing device probing first can change ``hwmonN``. This
module never hard-codes a number -- it always finds the device by reading
each ``hwmon*/name`` file and re-resolves this on every ``resolve()`` call
so a re-plug after a USB dropout picks up the new number transparently.

This module must not import :mod:`aqua_bridge.control` (or anything MPC).
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path

__all__ = ["DeviceUnavailable", "HwmonMap", "ResolvedPaths"]


class DeviceUnavailable(RuntimeError):
    """The hwmon device (by ``name``) cannot currently be found."""


@dataclass(frozen=True)
class ResolvedPaths:
    """File paths resolved for one moment in time.

    Kept as its own frozen object (rather than resolving pwm/temp/fan
    paths ad hoc) so a caller resolves once per operation and gets a
    consistent snapshot even if the device is renumbered between calls.
    """

    device_dir: Path
    pwm: dict[str, Path]
    temp: dict[str, Path]
    fan: dict[str, Path]

    def pwm_enable(self, channel: str) -> Path:
        """Path of the ``pwmN_enable`` sibling file for a mapped pwm channel."""
        pwm_path = self.pwm[channel]
        return pwm_path.with_name(pwm_path.name + "_enable")


@dataclass(frozen=True)
class HwmonMap:
    """Resolves logical channel/temperature names to hwmon sysfs files.

    Parameters
    ----------
    root:
        Root of the hwmon tree, default ``/sys/class/hwmon``. Overridable
        so tests point it at a fake tree in ``tmp_path``.
    hwmon_name:
        Expected content of the device's ``name`` file (``xt6.hwmon_name``
        in config, default ``"aquaero"``).
    pwm_map:
        Logical fan channel -> hwmon pwm attribute name (e.g.
        ``{"radiator": "pwm1"}``). The attribute name is the bare
        ``pwmN`` -- the ``_enable`` sibling is derived from it.
    temp_map:
        Logical temperature -> hwmon temp attribute name (e.g.
        ``{"coolant": "temp1"}``).
    fan_map:
        Optional fan channel -> hwmon tachometer attribute name (e.g.
        ``{"radiator": "fan1"}``). Keys must be channels of ``pwm_map``:
        RPM readings are consumed per channel (stall detection, Home
        Assistant sensors), so a key outside ``pwm_map`` would be a reading
        nobody looks at. Defaults to empty (no tachometer readback). The
        config builds both maps from one ``xt6.fans`` entry per fan.

    Resolution happens on demand (:meth:`resolve`), never once at
    construction, so a device that reappears after a dropout (possibly
    renumbered) is picked up without rebuilding this object.
    """

    hwmon_name: str
    pwm_map: dict[str, str]
    temp_map: dict[str, str]
    fan_map: dict[str, str] = field(default_factory=dict)
    root: str | Path = "/sys/class/hwmon"

    def __post_init__(self) -> None:
        if not self.hwmon_name:
            raise ValueError("hwmon_name must be a non-empty string")
        object.__setattr__(self, "root", Path(self.root))
        for label, mapping in (
            ("pwm_map", self.pwm_map),
            ("temp_map", self.temp_map),
            ("fan_map", self.fan_map),
        ):
            if not isinstance(mapping, dict):
                raise TypeError(f"{label} must be a dict, got {type(mapping).__name__}")
            _check_no_duplicate_targets(label, mapping)
        orphan_rpm = sorted(set(self.fan_map) - set(self.pwm_map))
        if orphan_rpm:
            raise ValueError(
                f"fan_map keys {orphan_rpm} are not pwm_map channels {sorted(self.pwm_map)}"
            )

    def find_device_dir(self) -> Path:
        """Locate the ``hwmonN`` directory whose ``name`` file matches.

        Scans by name every call -- never caches a ``hwmonN`` number --
        so re-plug/renumbering is transparent to callers that call this
        (directly or via :meth:`resolve`) each time they need the device.
        """
        root = Path(self.root)
        if not root.is_dir():
            raise DeviceUnavailable(f"hwmon root not found: {root}")
        candidates = sorted(p for p in root.iterdir() if p.is_dir() or p.is_symlink())
        for dev in candidates:
            name_file = dev / "name"
            try:
                name = name_file.read_text().strip()
            except OSError:
                continue
            if name == self.hwmon_name:
                return dev
        raise DeviceUnavailable(f"no hwmon device named {self.hwmon_name!r} found under {root}")

    def resolve(self, *, check_files: bool = True) -> ResolvedPaths:
        """Resolve every mapped logical name to a concrete file path.

        Raises :class:`DeviceUnavailable` if the device itself cannot be
        found. With ``check_files`` true (the default -- suited to a
        one-off startup sanity check), also raises :class:`ValueError`
        if a mapped attribute file does not exist under the (found)
        device directory: a clear error up front rather than a confusing
        failure deep in a read/write.

        The read/apply hot path (:meth:`Xt6Adapter.read` /
        :meth:`Xt6Adapter.apply`) resolves with ``check_files=False``:
        a *single* attribute going missing or unreadable is a normal
        transient glitch there (becomes ``None`` on read), not a reason
        to fail the whole sample -- only a vanished *device directory*
        is escalated to :class:`DeviceUnavailable`.
        """
        device_dir = self.find_device_dir()

        # config.example.yaml gives bare attribute names ("pwm1", "temp1",
        # "fan1"); pwmN is both the read and the write file, while
        # tempN/fanN readings live in the "*_input" sibling.
        pwm = {
            ch: self._resolve_one(device_dir, "pwm_map", ch, attr, check_files)
            for ch, attr in self.pwm_map.items()
        }
        temp = {
            name: self._resolve_one(device_dir, "temp_map", name, attr + "_input", check_files)
            for name, attr in self.temp_map.items()
        }
        fan = {
            name: self._resolve_one(device_dir, "fan_map", name, attr + "_input", check_files)
            for name, attr in self.fan_map.items()
        }
        return ResolvedPaths(device_dir=device_dir, pwm=pwm, temp=temp, fan=fan)

    @staticmethod
    def _resolve_one(
        device_dir: Path, label: str, logical: str, filename: str, check_files: bool
    ) -> Path:
        path = device_dir / filename
        if check_files and not path.exists():
            raise ValueError(
                f"{label}[{logical!r}] -> {filename!r} does not exist under {device_dir}"
            )
        return path


def _check_no_duplicate_targets(label: str, mapping: dict[str, str]) -> None:
    seen: dict[str, str] = {}
    for logical, attr in mapping.items():
        if attr in seen:
            raise ValueError(
                f"{label} maps both {seen[attr]!r} and {logical!r} to the same "
                f"hwmon attribute {attr!r}"
            )
        seen[attr] = logical
