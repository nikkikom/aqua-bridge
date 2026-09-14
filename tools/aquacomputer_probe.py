#!/usr/bin/env python3
"""Read-only bring-up probe for the aquaero and the Quadro over hidraw (PROJECT.md section 10).

Run on the Pi (the service user, or any user in ``plugdev``)::

    tools/aquacomputer_probe.py
    tools/aquacomputer_probe.py --device quadro --serial 12345-54321

Lists every discovered status/control hidraw node (kind, serial, USB
interface, device node), then for each device prints its newest status
report -- temperatures, each output's rpm, output duty, voltage, current and
power, flow, and the Quadro's power-cycle count -- and each output's duty in
the control report, with the aquaero's control source and power limits (the
daemon's duty is in effect only while the channel follows its own preset with
limits 0 / 100 %), and the aquaero output mode (PWM or DC voltage). Use it to
pick ``tempN`` / ``fanN`` / ``pwmN`` for the config and a ``serial:`` when
several of one kind are attached.

It never writes: it reads input reports and fetches the control report
(``HIDIOCGFEATURE``), nothing else. The fetch is a control operation like the
daemon's own, so prefer running it with the daemon stopped (a fetch within
``ctrl_gap_ms`` of the daemon's write may fail; that is reported, not retried).
"""

from __future__ import annotations

import argparse
import sys
import time
from collections.abc import Callable, Sequence
from pathlib import Path
from typing import TextIO

from aqua_bridge.hw.aquacomputer import (
    KINDS,
    DeviceKind,
    ReportError,
    StatusReport,
    channel_state,
    check_control_report,
    decode_status,
    is_status_report,
)
from aqua_bridge.hw.aquacomputer_adapter import AquacomputerTiming
from aqua_bridge.hw.hidraw import (
    DEFAULT_DEV_DIR,
    DEFAULT_SYSFS_ROOT,
    DeviceUnavailable,
    FeatureReportError,
    HidrawInfo,
    HidrawTransport,
    HidTransport,
    list_hidraw_devices,
    matches_kind,
)

__all__ = ["build_parser", "main", "probe"]

Opener = Callable[[HidrawInfo], HidTransport]


def _percent(centi: int) -> str:
    return f"{centi / 100:.2f} %"


def _wait_status(
    transport: HidTransport, kind: DeviceKind, timeout_s: float, clock: Callable[[], float]
) -> StatusReport | None:
    deadline = clock() + timeout_s
    while True:
        newest = None
        for report in transport.read_reports():
            if is_status_report(kind, report):
                newest = report
        if newest is not None:
            return decode_status(kind, newest)
        remaining = deadline - clock()
        if remaining <= 0:
            return None
        transport.wait_readable(remaining)


def _print_status(kind: DeviceKind, status: StatusReport, out: TextIO) -> None:
    print(f"  firmware {status.firmware}, serial in the status report {status.serial}", file=out)
    if status.power_cycles is not None:
        print(f"  power cycles: {status.power_cycles}", file=out)
    connected = [(n, t) for n, t in enumerate(status.temps, start=1) if t is not None]
    print("  temperatures (degC):", file=out)
    for n, value in connected:
        print(f"    temp{n:<3} {value:7.2f}", file=out)
    missing = [f"temp{n}" for n, t in enumerate(status.temps, start=1) if t is None]
    if missing:
        print(f"    not connected: {' '.join(missing)}", file=out)
    print("  outputs (status report):", file=out)
    for k, fan in enumerate(status.fans):
        print(
            f"    pwm{k + 1}/fan{k + 1}  {fan.rpm:5d} rpm  duty {_percent(fan.duty):>8}  "
            f"{fan.voltage_v:5.2f} V  {fan.current_ma:5d} mA  {fan.power_w:6.2f} W",
            file=out,
        )
    for j, flow in enumerate(status.flows):
        print(f"    fan{len(status.fans) + j + 1} (flow)  {flow}", file=out)


def _print_control(kind: DeviceKind, data: bytes, out: TextIO) -> None:
    print("  outputs (control report):", file=out)
    for k in range(kind.pwm_count):
        state = channel_state(kind, data, k)
        line = f"    pwm{k + 1}  duty {_percent(state.duty):>8}"
        if state.source is not None:
            assert state.min_power is not None and state.max_power is not None
            follows = "follows its preset" if state.on_duty else "does not follow its preset"
            line += (
                f"  source 0x{state.source:02X}  min {_percent(state.min_power)}"
                f"  max {_percent(state.max_power)}  ({follows})"
            )
        if state.mode is not None:
            line += f"  mode {state.mode.name} (0x{state.mode.raw:04X})"
        print(line, file=out)


def _probe_device(
    kind: DeviceKind,
    info: HidrawInfo,
    *,
    opener: Opener,
    timeout_s: float,
    clock: Callable[[], float],
    out: TextIO,
) -> bool:
    print(f"{kind.name} {info.serial or '(no serial)'} at {info.node}", file=out)
    try:
        transport = opener(info)
    except DeviceUnavailable as exc:
        print(f"  cannot open: {exc}", file=out)
        return False
    try:
        status = _wait_status(transport, kind, timeout_s, clock)
        if status is None:
            print(f"  no status report within {timeout_s:g} s", file=out)
            return False
        _print_status(kind, status, out)
        try:
            data = transport.get_feature(kind.ctrl_report_id, kind.ctrl_size)
            check_control_report(kind, data)
        except (FeatureReportError, ReportError) as exc:
            print(f"  control report: {exc}", file=out)
            return False
        _print_control(kind, data, out)
        return True
    except DeviceUnavailable as exc:
        print(f"  device went away: {exc}", file=out)
        return False
    finally:
        transport.close()


def probe(
    *,
    sysfs_root: Path = DEFAULT_SYSFS_ROOT,
    dev_dir: Path = DEFAULT_DEV_DIR,
    kinds: Sequence[DeviceKind] = tuple(KINDS.values()),
    serial: str | None = None,
    timeout_s: float | None = None,
    opener: Opener | None = None,
    clock: Callable[[], float] = time.monotonic,
    out: TextIO | None = None,
) -> int:
    """Lists and reads every matching device. Exit code: 0 all probed, 1 none
    found, 3 at least one could not be read."""
    out = sys.stdout if out is None else out

    def wait(kind: DeviceKind) -> float:
        if timeout_s is not None:
            return timeout_s
        return AquacomputerTiming.for_kind(kind).status_max_age_s

    open_node: Opener = opener if opener is not None else HidrawTransport.open
    devices = list_hidraw_devices(sysfs_root, dev_dir)
    found = [
        (kind, info)
        for kind in kinds
        for info in devices
        if matches_kind(info, kind) and (serial is None or info.serial == serial)
    ]
    if not found:
        wanted = ", ".join(kind.name for kind in kinds)
        suffix = f" with serial {serial!r}" if serial is not None else ""
        print(f"no {wanted}{suffix} found under {sysfs_root}", file=out)
        return 1
    print("discovered:", file=out)
    for kind, info in found:
        serial_text = info.serial or "(no serial)"
        print(
            f"  {kind.name:<8} serial {serial_text:<14} interface {info.interface}  {info.node}",
            file=out,
        )
    ok = True
    for kind, info in found:
        print(file=out)
        ok = (
            _probe_device(kind, info, opener=open_node, timeout_s=wait(kind), clock=clock, out=out)
            and ok
        )
    return 0 if ok else 3


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(prog="aquacomputer_probe", description=__doc__.split("\n\n")[0])
    p.add_argument("--device", choices=sorted(KINDS), help="only this kind (default: both)")
    p.add_argument("--serial", help="only the device with this serial")
    p.add_argument(
        "--timeout",
        type=float,
        default=None,
        metavar="S",
        help=(
            "seconds to wait for a status report (default: the status_max_age_s default, "
            f"{AquacomputerTiming.for_kind('aquaero').status_max_age_s:g})"
        ),
    )
    p.add_argument(
        "--sysfs-root", default=str(DEFAULT_SYSFS_ROOT), help="hidraw class directory in sysfs"
    )
    p.add_argument("--dev-dir", default=str(DEFAULT_DEV_DIR), help="directory of the device nodes")
    return p


def main(argv: Sequence[str] | None = None, *, opener: Opener | None = None) -> int:
    args = build_parser().parse_args(argv)
    if args.timeout is not None and not args.timeout > 0:
        print("--timeout must be > 0", file=sys.stderr)
        return 2
    kinds = (KINDS[args.device],) if args.device else tuple(KINDS.values())
    return probe(
        sysfs_root=Path(args.sysfs_root),
        dev_dir=Path(args.dev_dir),
        kinds=kinds,
        serial=args.serial,
        timeout_s=args.timeout,
        opener=opener,
    )


if __name__ == "__main__":
    sys.exit(main())
