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
      - device: quadro             # on its own USB port (on the aquaero's aquabus its
        serial: "12345-54321"      # outputs are the aquaero's pwm5..pwm8 instead)
        fans: {exhaust: {pwm: pwm1}}
        temp_map: {air_z1: temp2}
    xt6:                           # still accepted as exactly one more device
      device: aquaero
      fans: {...}
      temp_map: {...}
    onewire:
      sensors: {prox_b01: 28-0316a27a0aff}
      resolution_bits: 10           # default; 9..12 (PROJECT.md section 8 item 39)
      max_age_s: 7.5                # default: 1.5 * mpc.dt

Every ``mpc.temps`` name must be bound by exactly one controller ``temp_map``
or ``onewire.sensors`` entry across every device; every ``mpc.channels`` name
by exactly one controller ``fans`` entry. A name bound twice, or a declared
name left unbound, is a :class:`~aqua_bridge.model.ConfigError` (startup, exit
2) -- the same "no silently unwritten channel / permanently untrusted
temperature" guarantee a single ``xt6:`` device gets, checked across the whole
fleet. Two entries that could open the same controller (one kind without
distinct serials) are a ``ConfigError`` too: they would fight over one device.
So is a config that commands aquaero outputs 5-8 (a Quadro on the aquaero's
aquabus) and also, through a quadro entry without ``serial:``, the outputs of
whichever Quadro is attached over USB: a Quadro on aquabus ignores writes over
its USB, so that entry may command the Quadro on aquabus and do nothing
(PROJECT.md section 8 item 85). Which Quadro sits on aquabus cannot be read
before the devices are opened, so this is refused whichever way the second
entry names its Quadro: with a ``serial:`` it opens a second, physically
distinct Quadro on its own USB port, which is a second, independent
controller -- not the supported topology (PROJECT.md section 2 "Supported
topology", section 8 item 106): exactly one controlling controller, with any
slave devices hanging off it over aquabus. At runtime, a Quadro whose outputs
do not follow while an aquaero next to it reports a device on its aquabus
gets that explanation in its stuck-channel error (this can still happen with
a single commanding Quadro entry, opened before the aquaero's aquabus use was
added to its config or the Quadro moved to the wrong port).
A ROM id missing from the 1-Wire bus at this point is only a warning (see
:meth:`~aqua_bridge.hw.onewire.W1Source.start`) -- a sensor may be legitimately
unplugged with its drive.

This module must not import :mod:`aqua_bridge.control` (or anything MPC) --
see the static AST check in ``tests/test_hw_imports.py``.
"""

from __future__ import annotations

import functools
import logging
import time
from collections.abc import Callable, Iterable, Mapping, Sequence
from typing import Any, Protocol, runtime_checkable

from aqua_bridge.hw.aquacomputer import QUADRO
from aqua_bridge.hw.aquacomputer_adapter import (
    AquacomputerAdapter,
    DeviceBinding,
    Opener,
    check_watchdog,
    parse_device_section,
)
from aqua_bridge.hw.hidraw import DeviceUnavailable
from aqua_bridge.hw.onewire import W1Source, build_onewire_from_config
from aqua_bridge.model import ConfigError, MpcCommand, PlantObservation

__all__ = ["CompositeSink", "CompositeSource", "SmartSource", "build_composite_from_config"]

#: default onewire.max_age_s = this * mpc.dt (plan section 1).
_DEFAULT_MAX_AGE_DT_FACTOR = 1.5

_LOG = logging.getLogger("aqua_bridge.hw.sources")


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

    :meth:`device_health` merges the controllers' own diagnostics on a path that
    does not go through the observation, so it answers after a failed ``read()``
    too (PROJECT.md section 8 item 83).
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
        aquaeros = tuple(device for device in self.devices if device.kind.aquabus_outputs)
        if aquaeros:
            for device in self.devices:
                if device.kind is QUADRO:
                    device.stuck_hint = functools.partial(_quadro_on_aquabus_hint, aquaeros)

    def _each_device(self, what: str, call: Callable[[AquacomputerAdapter], Any]) -> list[Any]:
        """Runs ``call`` on every device, even after one of them raised.

        A device that fails must not keep the others from being read (each
        read drains that controller's hidraw queue) or written (the loop's
        fallback ramp and the SIGTERM ``fallback_pwm`` write must reach every
        healthy controller). Failures are collected and raised afterwards as
        one :class:`~aqua_bridge.hw.hidraw.DeviceUnavailable` naming every
        failed device, chained to the first exception; when every failure is
        a ``ValueError`` (a bad command) a ``ValueError`` is raised instead.
        """
        results: list[Any] = []
        failures: list[tuple[AquacomputerAdapter, Exception]] = []
        for device in self.devices:
            try:
                results.append(call(device))
            except Exception as exc:  # collected, raised after the loop
                failures.append((device, exc))
        if not failures:
            return results
        detail = "; ".join(
            f"{device.binding.label}: {type(exc).__name__}: {exc}" for device, exc in failures
        )
        message = f"{what} failed on {len(failures)} of {len(self.devices)} device(s): {detail}"
        first = failures[0][1]
        if all(isinstance(exc, ValueError) for _, exc in failures):
            raise ValueError(message) from first
        raise DeviceUnavailable(message) from first

    def read(self) -> PlantObservation:
        """Reads every controller, then the latest 1-Wire samples, then (if
        configured) the latest SMART snapshot into ``inputs["smart"]``.

        Every controller is read even when an earlier one fails, so every
        hidraw queue is drained each tick. If any controller is unavailable
        (``DeviceUnavailable``: gone, or no status report within its
        ``status_max_age_s``) the whole observation is lost this tick, as for a
        single-device ``xt6`` source, and the loop's blank-observation fallback
        runs (safe direction -- fans ramp toward ``fallback_pwm``, never down);
        the raised ``DeviceUnavailable`` names every failed device. A 1-Wire
        sensor's own loss is finer-grained and never escalates here:
        :meth:`W1Source.read` already reports ``None`` for the sensor(s) it
        affects, which the gate handles per sensor. SMART is never gated at
        all (plan section 1): a stale or empty inbox just means
        ``inputs["smart"]`` is empty or missing that serial this tick.
        """
        observations = self._each_device("read", lambda device: device.read())
        temps: dict[str, float | None] = {}
        rpm: dict[str, float | None] = {}
        pwm: dict[str, float | None] = {}
        fans: dict[str, Any] = {}
        for obs in observations:
            temps.update(obs.temps)
            rpm.update(obs.rpm)
            pwm.update(obs.pwm)
            fans.update(obs.inputs.get("fans") or {})
        if self.onewire is not None:
            temps.update(self.onewire.read())
        inputs: dict[str, Any] = {}
        if fans:
            inputs["fans"] = fans
        if self.smart is not None:
            inputs["smart"] = dict(self.smart.snapshot())
        return PlantObservation(temps=temps, rpm=rpm, pwm=pwm, ts=self._clock(), inputs=inputs)

    def device_health(self) -> dict[str, Any]:
        """Every controller's :meth:`~aqua_bridge.hw.aquacomputer_adapter.
        AquacomputerAdapter.device_health`, merged (PROJECT.md section 8 item 83).

        ``devices`` is one entry per controller in config order, ``problems`` every
        controller's problems concatenated and ``ok`` whether that list is empty.
        Never raises and never does I/O: a controller that is gone still reports what
        its last status report said, which is what an operator needs in exactly that
        case.
        """
        devices: list[dict[str, Any]] = []
        problems: list[str] = []
        for device in self.devices:
            try:
                health = device.device_health()
            except Exception:  # a diagnostics path must never break a tick
                _LOG.exception("%s: device_health failed", device.binding.label)
                continue
            devices.append(health)
            problems.extend(str(p) for p in health.get("problems") or ())
        return {"devices": devices, "problems": problems, "ok": not problems}

    def apply(self, cmd: MpcCommand) -> None:
        """Writes ``cmd`` to every controller.

        Each adapter only commands the channels its own ``fans`` map claims
        (extra keys in ``cmd.pwm`` are ignored by design) and sends at most one
        control report for all of them, so passing the whole command to every
        device is exactly "write each channel to its device". Every device is
        written even when an earlier one fails, so a fallback command (and the
        stop write) reaches every healthy controller; the failures are raised
        afterwards. The loop then treats the whole tick as not applied -- the
        conservative reading of a partial write ``control/loop.py`` documents
        -- and reads the real output duty back next tick either way.
        """
        self._each_device("apply", lambda device: device.apply(cmd))


def _quadro_on_aquabus_hint(aquaeros: Sequence[AquacomputerAdapter]) -> str | None:
    """Why a Quadro's outputs may not follow: an aquaero in the same composite reports a
    device on its aquabus (a fan block 5-8 whose rpm is not 0xFFFF)."""
    for aquaero in aquaeros:
        status = aquaero.last_status
        if status is None:
            continue
        present = [n for n in aquaero.kind.aquabus_outputs if status.fans[n - 1].present]
        if present:
            outputs = ", ".join(f"pwm{n}" for n in present)
            return (
                f"{aquaero.binding.label} reports a device on its aquabus ({outputs}): the "
                "Quadro is probably on the aquaero's aquabus, where it ignores writes over "
                "its own USB; command its outputs through the aquaero (pwm5..pwm8) instead"
            )
    return None


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


def _check_quadro_commanded_once(bindings: Sequence[tuple[str, DeviceBinding]]) -> None:
    """Aquaero outputs 5-8 command a Quadro on the aquaero's aquabus, which ignores
    writes over its own USB. A commanding Quadro entry alongside those outputs is
    refused either way (PROJECT.md section 2 "Supported topology", section 8 item 106):
    one without ``serial:`` opens whichever Quadro is attached, possibly the very one
    already reached through the aquaero, and might silently do nothing; one with a
    ``serial:`` is a second, physically distinct Quadro on its own USB port -- a second,
    independent controller, which is not the supported topology (exactly one
    controlling controller, with any slave devices hanging off it over aquabus)."""
    aquabus = [
        (holder, sorted(n for n in binding.pwm_map.values() if n in binding.kind.aquabus_outputs))
        for holder, binding in bindings
    ]
    aquabus = [(holder, outputs) for holder, outputs in aquabus if outputs]
    quadros = [
        (holder, binding)
        for holder, binding in bindings
        if binding.kind is QUADRO and binding.pwm_map
    ]
    if not aquabus or not quadros:
        return
    holder, outputs = aquabus[0]
    names = ", ".join(f"pwm{n}" for n in outputs)
    unidentified = [quadro for quadro, binding in quadros if binding.serial is None]
    if unidentified:
        raise ConfigError(
            f"{holder} commands aquabus outputs {names} (a Quadro on the aquaero's aquabus) "
            f"and {unidentified[0]} commands the outputs of whichever Quadro is attached over "
            "USB, which can be that one; a Quadro on aquabus ignores writes over its USB. "
            "Command the Quadro either through the aquaero (pwm5..pwm8, and 'fans: {}' in the "
            "quadro entry) or over its own USB (no aquaero pwm5..pwm8, and a second Quadro on "
            "its own USB port needs a distinct 'serial:' too -- naming one here does not make "
            "this config valid, since two commanding entries are refused either way)"
        )
    identified = quadros[0][0]
    raise ConfigError(
        f"{holder} commands aquabus outputs {names} (a Quadro on the aquaero's aquabus) and "
        f"{identified} commands a second, physically distinct Quadro's outputs over its own "
        "USB: two independent controllers are not a supported topology (PROJECT.md section 2 "
        "'Supported topology', section 8 item 106). The supported shape is exactly one "
        "controlling controller, with any slave devices (a Quadro on its aquaero's aquabus, "
        "for instance) hanging off it -- command the Quadro through the aquaero (pwm5..pwm8, "
        "and 'fans: {}' in the quadro entry) instead of over its own USB, or remove the "
        "aquaero's pwm5..pwm8 mapping if this Quadro is really meant to stay independent"
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
    watchdog_s: float | None = None,
    step_bound_s: float | None = None,
) -> tuple[CompositeSource, Callable[[], None] | None]:
    """Builds the composite source/sink from raw config sections.

    ``aquacomputer_section`` is ``AppConfig.aquacomputer`` (a list, possibly
    empty); ``xt6_section`` is ``AppConfig.section("xt6")`` (possibly empty) and
    is accepted as exactly one more device when non-empty, so a single-aquaero
    config needs no rewrite to gain a 1-Wire bus. At least one device (from
    either) is required. Nothing is opened here: each adapter opens its device
    on its first ``read()`` / ``apply()``. ``sleep`` and ``opener`` reach every
    adapter (tests inject fakes). ``watchdog_s`` is the systemd watchdog period
    (``None`` without one): ``dt`` + ``step_bound_s`` (``mpc.budget_alarm_ms``) +
    the devices' summed worst-case blocking per tick must stay below it
    (:func:`~aqua_bridge.hw.aquacomputer_adapter.check_watchdog`), or the build
    fails with a ``ConfigError``.

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
    _check_quadro_commanded_once(bindings)
    check_watchdog(
        [(holder, binding.timing) for holder, binding in bindings],
        watchdog_s,
        dt=dt,
        step_bound_s=step_bound_s,
    )

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
