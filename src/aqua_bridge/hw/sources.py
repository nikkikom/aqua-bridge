"""Composite hardware source/sink: several Aqua Computer controllers plus one 1-Wire bus.

PROJECT.md section 3 (Track B) / the DAS plan section 1 ("CompositeSource
merges the controllers' read(), W1Source.read() ... into one PlantObservation")
and section 12 risk 1 (the Quadro on its own USB port, hence a *list* of
devices rather than one).

Config shape (``build_composite_from_config`` needs at least one controller --
fans can only be commanded through the aquaero or the Quadro, 1-Wire is
read-only)::

    aquacomputer:                 # list of devices, each over hidraw
      - device: aquaero
        fans: {radiator: {pwm: pwm1, rpm: fan1}}
        temp_map: {air_z0: temp1}
      - device: quadro             # on its own USB port
        serial: "12345-67890"      # only needed with several of one kind
        fans: {exhaust: {pwm: pwm1}}
        temp_map: {air_z1: temp2}
    xt6:                           # still accepted as exactly one more device
      device: aquaero
      fans: {...}
      temp_map: {...}
    onewire:
      sensors: {prox_b01: 28-0316a27a0aff}
      resolution_bits: 12
      max_age_s: 7.5                # default: 1.5 * mpc.dt

Every ``mpc.temps`` name must be bound by exactly one controller ``temp_map``
or ``onewire.sensors`` entry across every device; every ``mpc.channels`` name
by exactly one controller ``fans`` entry. A name bound twice, or a declared
name left unbound, is a :class:`~aqua_bridge.model.ConfigError` (startup, exit
2) -- the same "no silently unwritten channel / permanently untrusted
temperature" guarantee a single ``xt6:`` device gets, checked across the whole
fleet. Two entries that could open the same controller (one kind without
distinct serials) are a ``ConfigError`` too: they would fight over one device.
A ROM id missing from the 1-Wire bus at this point is only a warning (see
:meth:`~aqua_bridge.hw.onewire.W1Source.start`) -- a sensor may be legitimately
unplugged with its drive.

This module must not import :mod:`aqua_bridge.control` (or anything MPC) --
see the static AST check in ``tests/test_hw_imports.py``.
"""

from __future__ import annotations

import time
from collections.abc import Callable, Iterable, Mapping, Sequence
from typing import Any, Protocol, runtime_checkable

from aqua_bridge.hw.aquacomputer_adapter import (
    AquacomputerAdapter,
    DeviceBinding,
    Opener,
    parse_device_section,
)
from aqua_bridge.hw.onewire import W1Source, build_onewire_from_config
from aqua_bridge.model import ConfigError, MpcCommand, PlantObservation

__all__ = ["CompositeSink", "CompositeSource", "SmartSource", "build_composite_from_config"]

#: default onewire.max_age_s = this * mpc.dt (plan section 1).
_DEFAULT_MAX_AGE_DT_FACTOR = 1.5


@runtime_checkable
class SmartSource(Protocol):
    """What :class:`CompositeSource` needs from a SMART inbox (duck-typed so
    this module never imports :mod:`aqua_bridge.publishers`, matching the
    layering every other ``hw/`` module already keeps -- see the "must not
    import" note above)."""

    def snapshot(self) -> Mapping[str, Mapping[str, Any]]: ...


class CompositeSource:
    """Merges several :class:`~aqua_bridge.hw.aquacomputer_adapter.AquacomputerAdapter`
    controllers, one optional :class:`~aqua_bridge.hw.onewire.W1Source` and one
    optional SMART inbox (:class:`~aqua_bridge.publishers.inputs.SmartInbox`,
    duck-typed as :class:`SmartSource`) into one :class:`PlantObservation`;
    applies a command by writing each channel to whichever device claims it.
    """

    def __init__(
        self,
        devices: Sequence[AquacomputerAdapter],
        onewire: W1Source | None = None,
        *,
        smart: SmartSource | None = None,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        if not devices:
            raise ValueError("CompositeSource needs at least one Aqua Computer device")
        self.devices: list[AquacomputerAdapter] = list(devices)
        self.onewire = onewire
        self.smart = smart
        self._clock = clock

    def read(self) -> PlantObservation:
        """Reads every controller, then the latest 1-Wire samples, then (if
        configured) the latest SMART snapshot into ``inputs["smart"]``.

        A controller that is unavailable (``DeviceUnavailable``: gone, or no
        status report within its ``status_max_age_s``) propagates exactly as it
        does for a single-device ``xt6`` source: the whole observation is lost
        this tick and the loop's blank-observation fallback runs (safe
        direction -- fans ramp toward ``fallback_pwm``, never down). A 1-Wire
        sensor's own loss is finer-grained and never escalates here:
        :meth:`W1Source.read` already reports ``None`` for the sensor(s) it
        affects, which the gate handles per sensor. SMART is never gated at
        all (plan section 1): a stale or empty inbox just means
        ``inputs["smart"]`` is empty or missing that serial this tick.
        """
        temps: dict[str, float | None] = {}
        rpm: dict[str, float | None] = {}
        pwm: dict[str, float | None] = {}
        for device in self.devices:
            obs = device.read()
            temps.update(obs.temps)
            rpm.update(obs.rpm)
            pwm.update(obs.pwm)
        if self.onewire is not None:
            temps.update(self.onewire.read())
        inputs: dict[str, Any] = {}
        if self.smart is not None:
            inputs["smart"] = dict(self.smart.snapshot())
        return PlantObservation(temps=temps, rpm=rpm, pwm=pwm, ts=self._clock(), inputs=inputs)

    def apply(self, cmd: MpcCommand) -> None:
        """Writes ``cmd`` to every controller in turn.

        Each adapter only commands the channels its own ``fans`` map claims
        (extra keys in ``cmd.pwm`` are ignored by design) and sends at most one
        control report for all of them, so passing the whole command to every
        device is exactly "write each channel to its device". If one device
        raises, the ones already written this tick keep their new PWM while the
        remaining ones do not -- the partial write ``control/loop.py`` already
        documents; the loop treats the whole tick as not applied and reads the
        real output duty back next tick either way.
        """
        for device in self.devices:
            device.apply(cmd)


#: Alias: the sink half of a CompositeSource is the same object (its own
#: apply()); kept as a name so callers can express "the composite sink" in
#: type hints without importing CompositeSource for that alone.
CompositeSink = CompositeSource


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


def _check_distinct_devices(bindings: Sequence[tuple[str, DeviceBinding]]) -> None:
    """Two entries of one kind must name distinct serials, or both would open
    (and command) the same controller."""
    for i, (label_a, a) in enumerate(bindings):
        for label_b, b in bindings[i + 1 :]:
            if a.kind is b.kind and (a.serial is None or b.serial is None or a.serial == b.serial):
                raise ConfigError(
                    f"{label_a} and {label_b} can both open the same {a.kind.name}: "
                    "give each a distinct 'serial:'"
                )


def build_composite_from_config(
    *,
    aquacomputer_section: Sequence[Mapping[str, Any]] | None,
    xt6_section: Mapping[str, Any] | None,
    onewire_section: Mapping[str, Any] | None,
    channels: Sequence[str],
    temps: Sequence[str],
    dt: float,
    smart: SmartSource | None = None,
    clock: Callable[[], float] = time.monotonic,
    sleep: Callable[[float], None] = time.sleep,
    opener: Opener | None = None,
) -> tuple[CompositeSource, Callable[[], None] | None]:
    """Builds the composite source/sink from raw config sections.

    ``aquacomputer_section`` is ``AppConfig.aquacomputer`` (a list, possibly
    empty); ``xt6_section`` is ``AppConfig.section("xt6")`` (possibly empty) and
    is accepted as exactly one more device when non-empty, so a single-aquaero
    config needs no rewrite to gain a 1-Wire bus. At least one device (from
    either) is required. Nothing is opened here: each adapter opens its device
    on its first ``read()`` / ``apply()``. ``sleep`` and ``opener`` reach every
    adapter (tests inject fakes).

    ``smart`` is an already-built :class:`SmartSource` (typically a
    :class:`~aqua_bridge.publishers.inputs.SmartInbox` shared with the MQTT/
    HTTP publishers, whose lifetime they own) or ``None`` when SMART is not
    wired up; it is not built here since it has no ``config.yaml`` section
    of its own yet and its wiring spans the MQTT client and the HTTP app,
    not just hardware (``__main__.build_io`` passes it through).

    Returns ``(composite, release)`` where ``release`` stops the 1-Wire
    reader threads (``None`` when there is no 1-Wire source) -- the caller
    (``__main__.build_io``) runs it once at shutdown, the same slot the
    ``xt6``-only source leaves ``None``.
    """
    specs: list[tuple[str, Any]] = [
        (f"aquacomputer[{index}]", spec) for index, spec in enumerate(aquacomputer_section or [])
    ]
    if xt6_section:
        specs.append(("xt6", xt6_section))
    if not specs:
        raise ConfigError(
            "no Aqua Computer device configured: add 'aquacomputer:' (a list of devices) "
            "or 'xt6:' (a single device)"
        )

    temp_owner: dict[str, str] = {}
    pwm_owner: dict[str, str] = {}
    bindings: list[tuple[str, DeviceBinding]] = []
    for label, spec in specs:
        binding = parse_device_section(
            spec, label=label, ignored_keys=("prefer",) if label == "xt6" else ()
        )
        holder = f"{label} ({binding.label})"
        _claim(temp_owner, binding.temp_map, holder, "mpc.temps")
        _claim(pwm_owner, binding.pwm_map, holder, "mpc.channels")
        bindings.append((holder, binding))
    _check_distinct_devices(bindings)

    onewire_source = build_onewire_from_config(
        onewire_section or {}, default_max_age_s=_DEFAULT_MAX_AGE_DT_FACTOR * dt, clock=clock
    )
    if onewire_source is not None:
        _claim(temp_owner, onewire_source.sensors, "onewire.sensors", "mpc.temps")

    _check_bound_exactly(temp_owner, temps, "mpc.temps", "a controller temp_map or onewire.sensors")
    _check_bound_exactly(pwm_owner, channels, "mpc.channels", "a controller fans map")

    devices = [
        AquacomputerAdapter(binding, clock=clock, sleep=sleep, opener=opener)
        for _, binding in bindings
    ]
    composite = CompositeSource(devices, onewire_source, smart=smart, clock=clock)
    release: Callable[[], None] | None = None
    if onewire_source is not None:
        onewire_source.start()
        release = onewire_source.stop
    return composite, release
