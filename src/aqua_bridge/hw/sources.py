"""Composite hardware source/sink: several hwmon devices plus one 1-Wire bus.

PROJECT.md section 3 (Track B) / the DAS plan section 1 ("CompositeSource
merges Xt6Adapter.read(), W1Source.read() ... into one PlantObservation")
and section 12 risk 1 (the Quadro possibly on its own USB, hence a *list*
of hwmon devices rather than one).

Config shape (all optional; ``build_composite_from_config`` needs at least
one hwmon device -- fans can only be commanded through hwmon, 1-Wire is
read-only)::

    hwmon:                        # list of devices; new, generalises xt6:
      - name: aquaero
        fans: {radiator: {pwm: pwm1, rpm: fan1}}
        temp_map: {air_z0: temp1}
      - name: quadro               # e.g. on its own USB port, section 12 Q1
        fans: {exhaust: {pwm: pwm1}}
        temp_map: {air_z1: temp2}
    xt6:                           # still accepted as exactly one device
      hwmon_name: aquaero          # (legacy key name for what "hwmon:" calls "name")
      fans: {...}
      temp_map: {...}
    onewire:
      sensors: {prox_b01: 28-0316a27a0aff}
      resolution_bits: 12
      max_age_s: 7.5                # default: 1.5 * mpc.dt

Every ``mpc.temps`` name must be bound by exactly one hwmon ``temp_map`` or
``onewire.sensors`` entry across every device; every ``mpc.channels`` name
by exactly one hwmon ``fans`` entry. A name bound twice, or a declared name
left unbound, is a :class:`~aqua_bridge.model.ConfigError` (startup, exit
2) -- the same "no silently unwritten channel / permanently untrusted
temperature" guarantee ``xt6.build_map_from_config`` already gives a single
device, now checked across the whole fleet. A ROM id missing from the 1-Wire
bus at this point is only a warning (see :meth:`~aqua_bridge.hw.onewire.
W1Source.start`) -- a sensor may be legitimately unplugged with its drive.

This module must not import :mod:`aqua_bridge.control` (or anything MPC) --
see the static AST check in ``tests/test_hw_map.py``.
"""

from __future__ import annotations

import logging
import time
from collections.abc import Callable, Iterable, Mapping, Sequence
from typing import Any

from aqua_bridge.hw.onewire import W1Source, build_onewire_from_config
from aqua_bridge.hw.xt6 import Xt6Adapter, build_map_from_config
from aqua_bridge.model import ConfigError, MpcCommand, PlantObservation

__all__ = ["CompositeSink", "CompositeSource", "build_composite_from_config"]

_LOG = logging.getLogger("aqua_bridge.hw.sources")

#: default onewire.max_age_s = this * mpc.dt (plan section 1).
_DEFAULT_MAX_AGE_DT_FACTOR = 1.5


class CompositeSource:
    """Merges several :class:`Xt6Adapter` hwmon devices and one optional
    :class:`~aqua_bridge.hw.onewire.W1Source` into one :class:`PlantObservation`;
    applies a command by writing each channel to whichever device's map
    claims it.

    ``SmartInbox`` (MQTT-fed SMART readings) is a later milestone and is not
    wired in here -- ``PlantObservation.inputs`` does not exist yet either
    (plan section 7, "contract changes", is milestone 4 ``estimator``).
    """

    def __init__(
        self,
        hwmon: Sequence[Xt6Adapter],
        onewire: W1Source | None = None,
        *,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        if not hwmon:
            raise ValueError("CompositeSource needs at least one hwmon device")
        self.hwmon: list[Xt6Adapter] = list(hwmon)
        self.onewire = onewire
        self._clock = clock

    def read(self) -> PlantObservation:
        """Reads every hwmon device, then the latest 1-Wire samples.

        A hwmon device that has vanished (``DeviceUnavailable``) propagates
        exactly as it does for a single-device ``xt6`` source today: the
        whole observation is lost this tick and the loop's blank-observation
        fallback runs (safe direction -- fans ramp toward ``fallback_pwm``,
        never down). A 1-Wire sensor's own loss is finer-grained and never
        escalates here: :meth:`W1Source.read` already reports ``None`` for
        the sensor(s) it affects, which the gate handles per sensor.
        """
        temps: dict[str, float | None] = {}
        rpm: dict[str, float | None] = {}
        pwm: dict[str, float | None] = {}
        for adapter in self.hwmon:
            obs = adapter.read()
            temps.update(obs.temps)
            rpm.update(obs.rpm)
            pwm.update(obs.pwm)
        if self.onewire is not None:
            temps.update(self.onewire.read())
        return PlantObservation(temps=temps, rpm=rpm, pwm=pwm, ts=self._clock())

    def apply(self, cmd: MpcCommand) -> None:
        """Writes ``cmd`` to every hwmon device in turn.

        Each :class:`Xt6Adapter` only ever writes the channels its own
        ``fans`` map claims (extra keys in ``cmd.pwm`` are ignored by
        design, see ``Xt6Adapter.apply``), so passing the whole command to
        every device is exactly "write each channel to its device". If one
        device raises, the ones already written this tick keep their new
        PWM while the remaining ones do not -- the same shape of partial
        write ``control/loop.py`` already documents for a single multi-
        channel device (one ``pwmN`` write among several failing), now
        possibly spanning devices instead of channels within one; the loop
        treats the whole tick as not applied and reads real state back next
        tick either way.
        """
        for adapter in self.hwmon:
            adapter.apply(cmd)


#: Alias: the sink half of a CompositeSource is the same object (its own
#: apply()); kept as a name so callers can express "the composite sink" in
#: type hints without importing CompositeSource for that alone.
CompositeSink = CompositeSource


def _label_device(index: int, spec: Mapping[str, Any]) -> str:
    name = spec.get("name") or spec.get("hwmon_name")
    return f"hwmon[{index}] ({name})" if name else f"hwmon[{index}]"


def _build_device_map(spec: Mapping[str, Any], label: str):
    if not isinstance(spec, Mapping):
        raise ConfigError(f"{label} must be a mapping, got {type(spec).__name__}")
    section = dict(spec)
    name = section.pop("name", None)
    if name is not None and "hwmon_name" not in section:
        section["hwmon_name"] = name
    try:
        return build_map_from_config(section)
    except ConfigError as exc:
        raise ConfigError(f"{label}: {exc}") from exc


def _claim(owner: dict[str, str], names: Iterable[str], holder: str, what: str) -> None:
    for name in names:
        if name in owner:
            raise ConfigError(
                f"{what} name {name!r} is bound twice: by {owner[name]} and by {holder}"
            )
        owner[name] = holder


def _check_bound_exactly(
    owner: Mapping[str, str], declared: Sequence[str], what: str, source_label: str
) -> None:
    missing = [name for name in declared if name not in owner]
    if missing:
        raise ConfigError(f"{what} {missing} are not bound by {source_label}")
    extra = sorted(name for name in owner if name not in declared)
    if extra:
        raise ConfigError(
            f"name(s) {extra} are bound by {source_label} but not in {what}: {sorted(declared)}"
        )


def build_composite_from_config(
    *,
    hwmon_section: Sequence[Mapping[str, Any]] | None,
    xt6_section: Mapping[str, Any] | None,
    onewire_section: Mapping[str, Any] | None,
    channels: Sequence[str],
    temps: Sequence[str],
    dt: float,
    clock: Callable[[], float] = time.monotonic,
) -> tuple[CompositeSource, Callable[[], None] | None]:
    """Builds the composite source/sink from raw config sections.

    ``hwmon_section`` is ``AppConfig.hwmon`` (a list, possibly empty);
    ``xt6_section`` is ``AppConfig.section("xt6")`` (possibly empty) and is
    accepted as exactly one more device when non-empty, so an existing
    single-aquaero config needs no rewrite to gain a 1-Wire bus. At least
    one device (from either) is required.

    Returns ``(composite, release)`` where ``release`` stops the 1-Wire
    reader threads (``None`` when there is no 1-Wire source) -- the caller
    (``__main__.build_io``) runs it once at shutdown, the same slot the
    ``xt6``-only source leaves ``None`` today.
    """
    device_specs: list[Mapping[str, Any]] = list(hwmon_section or [])
    if xt6_section:
        device_specs.append(xt6_section)
    if not device_specs:
        raise ConfigError(
            "no hwmon device configured: add 'hwmon:' (a list of devices) or 'xt6:' "
            "(a single device)"
        )

    temp_owner: dict[str, str] = {}
    pwm_owner: dict[str, str] = {}
    devices: list[Xt6Adapter] = []
    for index, spec in enumerate(device_specs):
        if not isinstance(spec, Mapping):
            raise ConfigError(f"hwmon[{index}] must be a mapping, got {type(spec).__name__}")
        label = _label_device(index, spec)
        hwmon_map = _build_device_map(spec, label)
        _claim(temp_owner, hwmon_map.temp_map, label, "mpc.temps")
        _claim(pwm_owner, hwmon_map.pwm_map, label, "mpc.channels")
        devices.append(Xt6Adapter(hwmon_map, clock=clock))

    onewire_source = build_onewire_from_config(
        onewire_section or {}, default_max_age_s=_DEFAULT_MAX_AGE_DT_FACTOR * dt, clock=clock
    )
    if onewire_source is not None:
        _claim(temp_owner, onewire_source.sensors, "onewire.sensors", "mpc.temps")

    _check_bound_exactly(temp_owner, temps, "mpc.temps", "a hwmon temp_map or onewire.sensors")
    _check_bound_exactly(pwm_owner, channels, "mpc.channels", "a hwmon fans map")

    composite = CompositeSource(devices, onewire_source, clock=clock)
    release: Callable[[], None] | None = None
    if onewire_source is not None:
        onewire_source.start()
        release = onewire_source.stop
    return composite, release
