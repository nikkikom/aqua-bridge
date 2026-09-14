"""YAML configuration loading.

``load_config(path)`` returns an :class:`AppConfig`. Only the ``mpc``
section is typed and validated (:class:`aqua_bridge.model.MpcConfig`);
the other sections (``mqtt``, ``host``, ``xt6``, ``http``, ``digole``,
``onewire``) are handed to their owners as plain dicts. Unknown keys
*inside* ``mpc`` are an error; unknown top-level sections are kept in
``AppConfig.extra`` so a typo there is visible without being fatal.

Each owner validates its own scalars against a *bad type*, not only a
missing key (item 57: ``enabled: "true"`` is a string, not the ``bool``
every reader compares against, and must not silently behave like
``false``): ``http:`` in :class:`~aqua_bridge.publishers.httpauth.HttpSettings`,
``mqtt:`` in :func:`~aqua_bridge.publishers.mqtt_ha.validate_mqtt_section`, and
``onewire:`` inline in :func:`~aqua_bridge.hw.onewire.build_onewire_from_config` --
each raises naming the key, at the point that section is actually used (config
load for ``onewire`` since a DAS temperature source cannot start without it;
service start for ``http``/``mqtt`` so a typo in one optional publisher's
section never stops the daemon from controlling the fans). ``digole:`` has no
owner yet (section 5, "after Command is stable"), so ``_warn_bad_digole_enabled``
below only logs a warning at config load -- nothing reads the section, so there
is nowhere yet to raise.

``aquacomputer`` is the one section shaped as a *list* rather than a mapping
(one entry per Aqua Computer controller over hidraw, plan section 12 Q1: the
Quadro on its own USB port alongside the aquaero) and so is parsed separately
from the ``mapping`` sections below; it defaults to an empty tuple and its
entries are handed to ``hw/sources.py``, which validates each entry
(:func:`~aqua_bridge.hw.aquacomputer_adapter.parse_device_section`) and runs
the DAS plan's binding check against ``mpc.temps``/``mpc.channels`` (section
1) when ``--source composite`` builds the hardware. A config that still has
the former ``hwmon:`` section is rejected here with a pointer to the rename.
"""

from __future__ import annotations

import logging
import os
from collections.abc import Mapping
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import yaml

from aqua_bridge.model import ConfigError, MpcConfig

__all__ = ["AppConfig", "ConfigError", "KNOWN_SECTIONS", "load_config"]

_LOG = logging.getLogger(__name__)

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


#: The list-shaped section of controller entries.
DEVICE_LIST_SECTION = "aquacomputer"


def _device_list_section(value: object) -> tuple[dict[str, Any], ...]:
    """``aquacomputer:`` must be a list of mappings (or absent/null -> empty tuple)."""
    name = DEVICE_LIST_SECTION
    if value is None:
        return ()
    if not isinstance(value, list):
        raise ConfigError(f"config section {name!r} must be a list, got {type(value).__name__}")
    devices: list[dict[str, Any]] = []
    for i, entry in enumerate(value):
        if not isinstance(entry, Mapping):
            raise ConfigError(
                f"config section {name!r}[{i}] must be a mapping, got {type(entry).__name__}"
            )
        for key in entry:
            if not isinstance(key, str):
                raise ConfigError(f"config section {name!r}[{i}] has a non-string key: {key!r}")
        devices.append(dict(entry))
    return tuple(devices)


def _warn_bad_digole_enabled(section: Mapping[str, Any]) -> None:
    """``digole:`` has no owner module yet (PROJECT.md section 5, "after Command is
    stable"): unlike ``http:``/``mqtt:``/``onewire:`` there is nowhere to raise a
    service-start :class:`ConfigError` for it (item 57). Log now, at config load, so
    a value such as ``enabled: "true"`` (a string, not a bool) does not sit unnoticed
    until that code exists -- this never raises: a cosmetic section must not stop the
    daemon from controlling the fans.
    """
    if "enabled" in section and not isinstance(section["enabled"], bool):
        _LOG.warning(
            "config: digole.enabled must be true or false, got %r (no effect yet)",
            section["enabled"],
        )


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
    aquacomputer: tuple[dict[str, Any], ...] = field(default_factory=tuple)
    extra: dict[str, Any] = field(default_factory=dict)
    source: str | None = None

    def section(self, name: str) -> dict[str, Any]:
        """Raw dict for any *mapping* section by name (``{}`` when absent).

        ``aquacomputer`` is list-shaped, not a mapping -- read
        ``AppConfig.aquacomputer`` directly for it.
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
        if "hwmon" in data:
            raise ConfigError(
                "config section 'hwmon' was renamed to 'aquacomputer': the daemon talks to the "
                "controllers over hidraw, not the hwmon driver. Keep the list and each entry's "
                "fans and temp_map, and replace each entry's 'name:' with 'device:' (aquaero "
                "or quadro), adding 'serial:' when several of one kind are attached"
            )
        mpc = MpcConfig.from_mapping(_section("mpc", data["mpc"]))
        sections = {name: _section(name, data.get(name)) for name in KNOWN_SECTIONS}
        _warn_bad_digole_enabled(sections["digole"])
        devices = _device_list_section(data.get(DEVICE_LIST_SECTION))
        extra = {
            str(name): value
            for name, value in data.items()
            if name != "mpc" and name != DEVICE_LIST_SECTION and name not in KNOWN_SECTIONS
        }
        return cls(mpc=mpc, aquacomputer=devices, extra=extra, source=source, **sections)


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
