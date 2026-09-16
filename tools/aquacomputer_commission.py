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
so a scripted run never saves by accident. On the Quadro the save report is
only known from the identical report on the Farbwerk 360: it is *not*
verified to persist a configuration over a power cycle the way the aquaero's
is (item 86), and this tool says so before asking to send it.

Refuses to run at all while systemd reports ``--unit`` (default
``aqua-bridge.service``) active, activating or reloading, or when it cannot
tell: the running daemon owns the controller's control report, and a
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
confirmation prompt was declined.
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
from aqua_bridge.hw.aquacomputer import KINDS, active_profile, channel_state
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
#: systemd states that mean the unit's process might be running.
_RUNNING_STATES = frozenset({"active", "activating", "reloading"})
_STATE_TIMEOUT_S = 5.0
_CONFIRM = "yes"


def unit_is_running(
    unit: str,
    *,
    systemctl: str = DEFAULT_SYSTEMCTL,
    timeout_s: float = _STATE_TIMEOUT_S,
    runner: Callable[..., subprocess.CompletedProcess] = subprocess.run,
) -> bool:
    """True when systemd reports ``unit`` active, activating or reloading, or
    when its state cannot be determined at all (a missing ``systemctl``, a
    timeout, an answer this function does not recognise): refusing to
    commission is always safe, guessing that the daemon is stopped is not.
    ``systemctl is-active`` on a unit that does not exist answers "inactive",
    the same as a normal stopped unit, so it is not special-cased here.
    """
    try:
        proc = runner(
            [systemctl, "is-active", unit], capture_output=True, text=True, timeout=timeout_s
        )
    except (OSError, subprocess.TimeoutExpired) as exc:
        _LOG.warning("%s is-active %s failed to run: %s", systemctl, unit, exc)
        return True
    state = (proc.stdout or "").strip()
    return state in _RUNNING_STATES or state == ""


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


def _percent(centi: int) -> str:
    return f"{centi / 100:.2f} %"


def _print_report(binding: DeviceBinding, ctrl: bytes, out: TextIO) -> None:
    kind = binding.kind
    names = {number - 1: name for name, number in binding.pwm_map.items()}
    print(f"{binding.label}: control report held on the controller now", file=out)
    profile = active_profile(kind, ctrl)
    if profile is not None:
        print(f"  active profile: {profile}", file=out)
    print("  outputs:", file=out)
    for k in range(kind.pwm_count):
        state = channel_state(kind, ctrl, k)
        name = f" ({names[k]})" if k in names else " (not commanded by this config entry)"
        line = f"    pwm{k + 1}{name}  duty {_percent(state.duty)}"
        if state.source is not None:
            assert state.min_power is not None and state.max_power is not None
            follows = "follows its preset" if state.on_duty else "does not follow its preset"
            line += (
                f"  source 0x{state.source:02X}  min {_percent(state.min_power)}"
                f"  max {_percent(state.max_power)}  ({follows})"
            )
        if state.unconfigured:
            line += "  (unconfigured)"
        elif state.aquabus and state.mode is not None:
            line += f"  mode 0x{state.mode.raw:04X} (aquabus, not interpreted)"
        elif state.mode is not None:
            line += f"  mode {state.mode.name} (0x{state.mode.raw:04X})"
        print(line, file=out)
    if not kind.save_verified:
        print(
            f"  note: the {kind.name}'s save report is only known from the identical report on "
            "the Farbwerk 360; it is not verified to persist this over a power cycle "
            "(PROJECT.md section 8 item 88).",
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
        adapter.read()  # opens the node and checks binding.serial against the status report
        ctrl = adapter.control_snapshot()
    except DeviceUnavailable as exc:
        print(f"device error: {exc}", file=out)
        return 4
    finally:
        adapter.close()

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

    try:
        adapter.save()
    except DeviceUnavailable as exc:
        print(f"device error: {exc}", file=out)
        return 4
    finally:
        adapter.close()
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
        help="after a typed confirmation, send the save report (default: show only, write nothing)",
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
