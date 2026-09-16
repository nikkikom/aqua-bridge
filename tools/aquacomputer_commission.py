#!/usr/bin/env python3
"""Commissioning tool for the saved configuration (PROJECT.md section 8 item 88).

Run on the Pi, once per controller, with the ``aqua-bridge`` service **stopped**::

    tools/aquacomputer_commission.py --config /etc/aqua-bridge/config.yaml --device aquaero
    tools/aquacomputer_commission.py --config /etc/aqua-bridge/config.yaml --device quadro --save

Opens the one configured controller named by ``--device`` (``--serial`` picks
between several entries of the same kind), reads its control report and prints
every output's duty, source and limits and the active profile -- exactly what
:meth:`~aqua_bridge.hw.aquacomputer_adapter.AquacomputerAdapter.save` is about
to store, since ``save()`` persists whatever the controller holds *right now*
and changes nothing about it first (PROJECT.md section 8 items 84, 86).
Without ``--save`` this is the whole run: nothing is written. With ``--save``
the same report is shown, then a typed confirmation (``yes``) is required
before the save report actually goes out -- ``--save`` alone is not enough,
so a scripted run never saves by accident. The device stays open across the
prompt and the control report is read once more right before the save: the
controller can change while the prompt waits (the aquaero's own alarm selects
another profile when the heartbeat times out, and a profile switch reloads
that profile's saved settings; aquasuite on the PC; the front panel), and
``save()`` would store whatever it holds *then*. A report that no longer
matches the one shown aborts the run without saving.

On the Quadro the save report is only known from the identical report on the
Farbwerk 360: it is *not* verified to persist a configuration over a power
cycle the way the aquaero's is (items 86, 96), and this tool says so before
asking to send it.

Refuses to run at all unless systemd reports ``--unit`` (default
``aqua-bridge.service``) ``inactive`` or ``failed``: every other answer,
``deactivating`` and an undeterminable state included, counts as running.
The running daemon owns the controller's control report, and a
commissioning run racing it for the bus -- or saving a report the daemon is
about to change again -- is exactly the failure the daemon's own watchdog
guards against (PROJECT.md section 8 item 84). Stop the service first
(``systemctl stop <unit>``).

This tool commissions whatever configuration the controller already runs; it
never changes a duty, a source or a limit itself. Commission only a
configuration you have confirmed is safe -- the controller's own watchdog
fallback, ``fallback_pwm`` on the daemon's own, PROJECT.md section 2 -- since
a saved bad configuration is what a power cycle brings back.

Exit codes: 0 shown (and saved, if asked and confirmed); 2 a config or
``--device``/``--serial`` selection error; 3 refused because the service
might be running; 4 a hardware/read error; 5 ``--save`` was given but the
confirmation prompt was declined; 6 the control report changed between the
report and the confirmation, so nothing was saved.
"""

from __future__ import annotations

import argparse
import logging
import subprocess
import sys
import time
from collections.abc import Callable, Mapping, Sequence
from typing import Any, TextIO

from aqua_bridge.config import AppConfig, ConfigError, load_config
from aqua_bridge.hw.aquacomputer import (
    KINDS,
    active_profile,
    channel_state,
    format_channel_state,
)
from aqua_bridge.hw.aquacomputer_adapter import (
    AquacomputerAdapter,
    DeviceBinding,
    DeviceUnavailable,
    Opener,
    parse_device_section,
)

__all__ = [
    "DEFAULT_SYSTEMCTL",
    "DEFAULT_UNIT",
    "build_parser",
    "commission",
    "main",
    "select_binding",
    "unit_is_running",
]

_LOG = logging.getLogger("aqua_bridge.tools.aquacomputer_commission")

DEFAULT_UNIT = "aqua-bridge.service"
DEFAULT_SYSTEMCTL = "systemctl"
#: The only systemd states in which the unit certainly holds no controller;
#: every other answer (``active``, ``activating``, ``reloading``,
#: ``deactivating``, a blank line, anything unrecognised) means it might.
_STOPPED_STATES = frozenset({"inactive", "failed"})
_STATE_TIMEOUT_S = 5.0
_CONFIRM = "yes"


def unit_is_running(
    unit: str,
    *,
    systemctl: str = DEFAULT_SYSTEMCTL,
    timeout_s: float = _STATE_TIMEOUT_S,
    runner: Callable[..., subprocess.CompletedProcess] = subprocess.run,
) -> bool:
    """True unless systemd reports ``unit`` in one of the two states that mean
    it holds nothing: ``inactive`` or ``failed``.

    Everything else is "might be running" -- ``active``, ``activating`` and
    ``reloading``, but also ``deactivating``, a stop that has returned while
    the daemon's own shutdown writes (the ``fallback_pwm`` write, then the
    ``release()`` control report SET) are still in flight -- and so is every
    answer this function cannot make sense of: a missing ``systemctl``, a
    timeout, a blank line, an unrecognised state. Refusing to commission is
    always safe, guessing that the daemon is stopped is not.

    The exit status is deliberately not consulted: ``systemctl is-active``
    exits non-zero for exactly the stopped states, so the printed state alone
    decides. A unit that does not exist answers "inactive", the same as a
    normal stopped unit, and is not special-cased either.
    """
    try:
        proc = runner(
            [systemctl, "is-active", unit], capture_output=True, text=True, timeout=timeout_s
        )
    except (OSError, subprocess.TimeoutExpired) as exc:
        _LOG.warning("%s is-active %s failed to run: %s", systemctl, unit, exc)
        return True
    state = (proc.stdout or "").strip()
    if state in _STOPPED_STATES:
        return False
    _LOG.debug("%s is-active %s answered %r: treating it as running", systemctl, unit, state)
    return True


def _device_specs(app: AppConfig) -> list[tuple[str, Mapping[str, Any]]]:
    specs: list[tuple[str, Mapping[str, Any]]] = [
        (f"aquacomputer[{i}]", spec) for i, spec in enumerate(app.aquacomputer)
    ]
    if app.xt6:
        specs.append(("xt6", app.xt6))
    return specs


def select_binding(app: AppConfig, *, device: str, serial: str | None = None) -> DeviceBinding:
    """The one configured entry naming ``device`` (``aquaero``/``quadro``),
    narrowed by ``serial`` when given. Raises :class:`ConfigError` when there
    is none or more than one match -- commissioning the wrong controller is
    the one mistake this tool must never make silently."""
    matches = [
        (label, spec)
        for label, spec in _device_specs(app)
        if isinstance(spec, Mapping)
        and spec.get("device") == device
        and (serial is None or spec.get("serial") == serial)
    ]
    if not matches:
        suffix = f" with serial {serial!r}" if serial is not None else ""
        raise ConfigError(f"no configured {device} entry{suffix} in 'aquacomputer:' or 'xt6:'")
    if len(matches) > 1:
        labels = ", ".join(label for label, _ in matches)
        raise ConfigError(
            f"{len(matches)} configured {device} entries match ({labels}); narrow with --serial"
        )
    label, spec = matches[0]
    ignored = ("prefer",) if label == "xt6" else ()
    return parse_device_section(spec, label=label, ignored_keys=ignored)


def _print_report(binding: DeviceBinding, ctrl: bytes, out: TextIO) -> None:
    kind = binding.kind
    names = {number - 1: name for name, number in binding.pwm_map.items()}
    print(f"{binding.label}: control report held on the controller now", file=out)
    profile = active_profile(kind, ctrl)
    if profile is not None:
        print(f"  active profile: {profile}", file=out)
    print("  outputs:", file=out)
    for k in range(kind.pwm_count):
        name = f" ({names[k]})" if k in names else " (not commanded by this config entry)"
        state = channel_state(kind, ctrl, k)
        print(f"    {format_channel_state(state, k, name=name)}", file=out)
    if not kind.save_verified:
        print(
            f"  note: the {kind.name}'s save report is only known from the identical report on "
            "the Farbwerk 360; it is not verified to persist this over a power cycle "
            "(PROJECT.md section 8 items 86 and 96).",
            file=out,
        )


def commission(
    *,
    config_path: str,
    device: str,
    serial: str | None = None,
    save: bool,
    unit: str = DEFAULT_UNIT,
    systemctl: str = DEFAULT_SYSTEMCTL,
    opener: Opener | None = None,
    clock: Callable[[], float] = time.monotonic,
    sleep: Callable[[float], None] = time.sleep,
    runner: Callable[..., subprocess.CompletedProcess] = subprocess.run,
    out: TextIO | None = None,
    in_: TextIO | None = None,
) -> int:
    """Runs one commissioning pass (module docstring); returns the exit code."""
    out = sys.stdout if out is None else out
    if unit_is_running(unit, systemctl=systemctl, runner=runner):
        print(
            f"refusing: systemd reports {unit!r} might be running; stop it first "
            f"(systemctl stop {unit}) -- the daemon owns the controller while it runs",
            file=out,
        )
        return 3
    try:
        app = load_config(config_path)
        binding = select_binding(app, device=device, serial=serial)
    except ConfigError as exc:
        print(f"config error: {exc}", file=out)
        return 2

    adapter = AquacomputerAdapter(binding, clock=clock, sleep=sleep, opener=opener)
    try:
        return _show_and_save(adapter, binding, save=save, out=out, in_=in_)
    finally:
        # one open for the whole run: the report, the confirmation and the save
        adapter.close()


def _show_and_save(
    adapter: AquacomputerAdapter,
    binding: DeviceBinding,
    *,
    save: bool,
    out: TextIO,
    in_: TextIO | None,
) -> int:
    """The run against an open adapter: report, confirmation, re-read, save."""
    try:
        adapter.read()  # opens the node and checks binding.serial against the status report
        ctrl = adapter.control_snapshot()
    except DeviceUnavailable as exc:
        print(f"device error: {exc}", file=out)
        return 4

    _print_report(binding, ctrl, out)
    if not save:
        print("\n(dry run: pass --save to store this in the controller's memory)", file=out)
        return 0

    print(
        f"\nThis sends the save report to {binding.label}: the configuration shown above is "
        "stored in the controller's memory now and comes back after a power cycle instead of "
        "whatever it held before.",
        file=out,
    )
    print(f"Type {_CONFIRM!r} to save, anything else aborts: ", end="", file=out)
    out.flush()
    reader = sys.stdin if in_ is None else in_
    answer = reader.readline().strip()
    if answer != _CONFIRM:
        print("aborted: nothing was saved", file=out)
        return 5

    # The controller can change while the prompt waits -- a profile the aquaero's
    # own alarm selects when its heartbeat times out, aquasuite on the PC, a hand
    # on the front panel -- and save() stores whatever it holds at that moment.
    # Read it once more and save only what was shown (PROJECT.md section 8 item 88).
    try:
        again = adapter.control_snapshot()
    except DeviceUnavailable as exc:
        print(f"device error: {exc}", file=out)
        return 4
    if again != ctrl:
        print(
            "refusing: the control report changed between the report above and the "
            "confirmation, so saving now would store something you have not seen "
            "(an alarm selecting another profile, aquasuite, the front panel). "
            "Nothing was saved; run the tool again.",
            file=out,
        )
        _print_report(binding, again, out)
        return 6

    try:
        adapter.save()
    except DeviceUnavailable as exc:
        print(f"device error: {exc}", file=out)
        return 4
    print("saved.", file=out)
    return 0


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        prog="aquacomputer_commission", description=__doc__.split("\n\n")[0]
    )
    p.add_argument("--config", required=True, help="config.yaml path")
    p.add_argument(
        "--device", required=True, choices=sorted(KINDS), help="the configured entry to open"
    )
    p.add_argument("--serial", help="narrow to this serial when more than one entry matches")
    p.add_argument(
        "--save",
        action="store_true",
        help=(
            "after a typed confirmation, and only while the control report still reads as "
            "shown, send the save report (default: show only, write nothing)"
        ),
    )
    p.add_argument(
        "--unit",
        default=DEFAULT_UNIT,
        help=f"refuse while this systemd unit might be running (default {DEFAULT_UNIT})",
    )
    p.add_argument("--systemctl", default=DEFAULT_SYSTEMCTL, help="systemctl binary path/name")
    return p


def main(argv: Sequence[str] | None = None, *, opener: Opener | None = None) -> int:
    args = build_parser().parse_args(argv)
    return commission(
        config_path=args.config,
        device=args.device,
        serial=args.serial,
        save=args.save,
        unit=args.unit,
        systemctl=args.systemctl,
        opener=opener,
    )


if __name__ == "__main__":
    sys.exit(main())
