"""YAML configuration loading.

``load_config(path)`` returns an :class:`AppConfig`. Only the ``mpc``
section is typed and validated (:class:`aqua_bridge.model.MpcConfig`);
the other sections (``mqtt``, ``host``, ``xt6``, ``http``, ``digole``,
``onewire``) are handed to their owners as plain dicts. Unknown keys
*inside* ``mpc`` are an error; unknown top-level sections are kept in
``AppConfig.extra`` so a typo there is visible without being fatal.

``hwmon`` is the one section shaped as a *list* rather than a mapping (one
entry per hwmon device, plan section 12 Q1: the Quadro possibly needing its
own USB port and hwmon device alongside the aquaero) and so is parsed
separately from the ``mapping`` sections below; it defaults to an empty
tuple and its entries are handed to ``hw/sources.py`` unvalidated (that is
where the DAS plan's binding check against ``mpc.temps``/``mpc.channels``
lives, section 1).
"""

from __future__ import annotations

import os
from collections.abc import Mapping
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import yaml

from aqua_bridge.model import ConfigError, MpcConfig

__all__ = ["AppConfig", "ConfigError", "KNOWN_SECTIONS", "load_config"]

# Top-level sections with a dedicated attribute on AppConfig (besides "mpc").
KNOWN_SECTIONS: tuple[str, ...] = ("mqtt", "host", "xt6", "http", "digole", "onewire")


def _section(name: str, value: object) -> dict[str, Any]:
    """A YAML section must be a mapping (or absent/null -> empty dict)."""
    if value is None:
        return {}
    if not isinstance(value, Mapping):
        raise ConfigError(f"config section {name!r} must be a mapping, got {type(value).__name__}")
    for key in value:
        if not isinstance(key, str):
            raise ConfigError(f"config section {name!r} has a non-string key: {key!r}")
    return dict(value)


def _hwmon_section(value: object) -> tuple[dict[str, Any], ...]:
    """``hwmon:`` must be a list of mappings (or absent/null -> empty tuple)."""
    if value is None:
        return ()
    if not isinstance(value, list):
        raise ConfigError(f"config section 'hwmon' must be a list, got {type(value).__name__}")
    devices: list[dict[str, Any]] = []
    for i, entry in enumerate(value):
        if not isinstance(entry, Mapping):
            raise ConfigError(
                f"config section 'hwmon'[{i}] must be a mapping, got {type(entry).__name__}"
            )
        for key in entry:
            if not isinstance(key, str):
                raise ConfigError(f"config section 'hwmon'[{i}] has a non-string key: {key!r}")
        devices.append(dict(entry))
    return tuple(devices)


@dataclass(frozen=True)
class AppConfig:
    """The whole ``config.yaml``: a validated ``mpc`` plus raw sections."""

    mpc: MpcConfig
    mqtt: dict[str, Any] = field(default_factory=dict)
    host: dict[str, Any] = field(default_factory=dict)
    xt6: dict[str, Any] = field(default_factory=dict)
    http: dict[str, Any] = field(default_factory=dict)
    digole: dict[str, Any] = field(default_factory=dict)
    onewire: dict[str, Any] = field(default_factory=dict)
    hwmon: tuple[dict[str, Any], ...] = field(default_factory=tuple)
    extra: dict[str, Any] = field(default_factory=dict)
    source: str | None = None

    def section(self, name: str) -> dict[str, Any]:
        """Raw dict for any *mapping* section by name (``{}`` when absent).

        ``hwmon`` is list-shaped, not a mapping -- read ``AppConfig.hwmon``
        directly for it.
        """
        if name in KNOWN_SECTIONS:
            return getattr(self, name)
        return self.extra.get(name, {})

    @classmethod
    def from_mapping(cls, data: Mapping[str, Any], *, source: str | None = None) -> AppConfig:
        """Build from an already-parsed mapping (what ``yaml.safe_load`` returns)."""
        if not isinstance(data, Mapping):
            raise ConfigError(f"config root must be a mapping, got {type(data).__name__}")
        if "mpc" not in data or data["mpc"] is None:
            raise ConfigError("config is missing the required 'mpc' section")
        mpc = MpcConfig.from_mapping(_section("mpc", data["mpc"]))
        sections = {name: _section(name, data.get(name)) for name in KNOWN_SECTIONS}
        hwmon = _hwmon_section(data.get("hwmon"))
        extra = {
            str(name): value
            for name, value in data.items()
            if name != "mpc" and name != "hwmon" and name not in KNOWN_SECTIONS
        }
        return cls(mpc=mpc, hwmon=hwmon, extra=extra, source=source, **sections)


def load_config(path: str | os.PathLike[str]) -> AppConfig:
    """Read and validate a YAML config file. Raises :class:`ConfigError`."""
    path = Path(path)
    try:
        text = path.read_text(encoding="utf-8")
    except OSError as exc:
        raise ConfigError(f"cannot read config {path}: {exc}") from exc
    try:
        data = yaml.safe_load(text)
    except yaml.YAMLError as exc:
        raise ConfigError(f"invalid YAML in {path}: {exc}") from exc
    if data is None:
        raise ConfigError(f"config {path} is empty")
    return AppConfig.from_mapping(data, source=str(path))
