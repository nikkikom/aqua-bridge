#!/usr/bin/env python3
"""Read-only watch of an aquaero's aquabus: the refresh interval, presence, and the
unidentified fan-block field (PROJECT.md section 8 items 115, 114, 92).

Run on the Pi (the service user, or any user in ``plugdev``)::

    tools/aquabus_watch.py --reports 120
    tools/aquabus_watch.py --seconds 600 --raw

It reads status reports and **nothing else**: no feature report is fetched, no
control report is written, no duty is touched. It is the measurement behind three
questions the captures left open.

*How often does the aquaero refresh an aquabus fan block?* Only speed and output
duty are in every report; the electrical fields of blocks 5-8 carry the bus
device's own measurements once in about four reports and substitutes -- the
aquaero's own rail, 0 mA, 0 W -- in the rest
(:data:`~aqua_bridge.hw.aquacomputer.AQUABUS_REFRESH_REPORTS`). This prints, per
block, how many reports carried a measurement, the interval between them in
reports and in seconds, its spread, and whether the four blocks refresh together.

*Is a bus device there the whole time?* An aquabus block with no device behind it
reads speed ``0xFFFF``, and that is how the daemon judges the bus
(``aquabus_present``, item 92). The run prints the longest stretch of reports
with no device, so a healthy bus can be shown to stay quiet under the daemon's
``bus_absent_s``.

*What is the ``u16`` at ``+0x0A`` of a fan block?* Unidentified (item 114). It is
printed raw next to that block's duty, current and power, with the ratio the
captures show (field x duty vs the current field) -- the way to collect the
evidence at a duty the captures do not cover. Nothing in the daemon reads it.

Classifying a report as one that refreshed a block is a judgement from what the
fields hold: a block that reads 0.00 V (an output with no fan, which reads the
rail in the reports that refreshed nothing) or any non-zero current, power or
``+0x0A`` carried a measurement. A *stopped* fan on a populated output measures 0
mA legitimately, so a block like that cannot be told apart, and ``--raw`` prints
every report's fields for the owner to look at instead.
"""

from __future__ import annotations

import argparse
import statistics
import sys
import time
from collections.abc import Callable, Sequence
from dataclasses import dataclass
from pathlib import Path
from typing import TextIO

from aqua_bridge.hw.aquacomputer import (
    AQUAERO,
    DeviceKind,
    FanStatus,
    StatusReport,
    aquabus_present,
    decode_status,
    format_percent,
    is_status_report,
)
from aqua_bridge.hw.hidraw import (
    DEFAULT_DEV_DIR,
    DEFAULT_SYSFS_ROOT,
    DeviceUnavailable,
    HidrawInfo,
    HidrawTransport,
    HidTransport,
    list_hidraw_devices,
    matches_kind,
)

__all__ = ["Sample", "build_parser", "main", "summarise", "watch"]

Opener = Callable[[HidrawInfo], HidTransport]


@dataclass(frozen=True)
class Sample:
    """One status report and when it was drained."""

    t: float
    status: StatusReport


def _measured(fan: FanStatus) -> bool:
    """This block's electrical fields carry the bus device's own measurement.

    Module docstring: a populated block that refreshed nothing reads the aquaero's
    own rail with 0 mA / 0 W, and an empty output reads 0.00 V in a report that did
    refresh. A block with no device behind it at all (speed ``0xFFFF``) measures
    nothing either way.
    """
    if not fan.present:
        return False
    return (
        fan.voltage_cv == 0 or fan.current_ma > 0 or fan.power_cw > 0 or bool(fan.unidentified_raw)
    )


def _runs(flags: Sequence[bool]) -> list[int]:
    """Lengths of the runs of True in ``flags``."""
    out: list[int] = []
    run = 0
    for flag in flags:
        if flag:
            run += 1
        elif run:
            out.append(run)
            run = 0
    if run:
        out.append(run)
    return out


def _spread(values: Sequence[float], unit: str) -> str:
    if not values:
        return "no interval seen"
    mean = statistics.fmean(values)
    line = f"every {mean:.2f} {unit} (min {min(values):g}, max {max(values):g}"
    if len(values) > 1:
        line += f", sd {statistics.stdev(values):.2f}"
    return line + ")"


def _presence(kind: DeviceKind, samples: Sequence[Sample], out: TextIO) -> None:
    present = [aquabus_present(kind, s.status) is True for s in samples]
    answered = sum(present)
    print(f"  a device answers on aquabus in {answered} of {len(samples)} reports", file=out)
    empty = _runs([not flag for flag in present])
    if not empty:
        print("    no report showed the bus empty: the bus-absent rule stays quiet", file=out)
        return
    longest = max(empty)
    seconds = _seconds_of_longest_empty(samples, present)
    print(
        f"    longest stretch with no device: {longest} reports"
        + (f", {seconds:.1f} s" if seconds is not None else "")
        + " (the daemon reports the bus device lost after bus_absent_s of them)",
        file=out,
    )


def _seconds_of_longest_empty(samples: Sequence[Sample], present: Sequence[bool]) -> float | None:
    best: float | None = None
    start: int | None = None
    for i, flag in enumerate(present):
        if not flag and start is None:
            start = i
        elif flag and start is not None:
            span = samples[i - 1].t - samples[start].t
            best = span if best is None else max(best, span)
            start = None
    if start is not None:
        span = samples[-1].t - samples[start].t
        best = span if best is None else max(best, span)
    return best


def _refresh(kind: DeviceKind, samples: Sequence[Sample], out: TextIO) -> None:
    print("  aquabus block refreshes (the bus device's own voltage and current):", file=out)
    measured: dict[int, list[bool]] = {}
    for number in kind.aquabus_outputs:
        flags = [_measured(s.status.fans[number - 1]) for s in samples]
        measured[number] = flags
        indices = [i for i, flag in enumerate(flags) if flag]
        gaps = [float(b - a) for a, b in zip(indices, indices[1:], strict=False)]
        seconds = [samples[b].t - samples[a].t for a, b in zip(indices, indices[1:], strict=False)]
        line = f"    pwm{number}  {len(indices)} of {len(samples)} reports"
        if gaps:
            line += f"  {_spread(gaps, 'reports')}  {_spread(seconds, 's')}"
        print(line, file=out)
    together = sum(1 for i in range(len(samples)) if all(f[i] for f in measured.values()))
    any_block = sum(1 for i in range(len(samples)) if any(f[i] for f in measured.values()))
    if any_block:
        verdict = (
            "the refresh is atomic"
            if together == any_block
            else "so the blocks do not all refresh together"
        )
        print(
            f"    {together} of {any_block} refreshing reports refreshed every block: {verdict}",
            file=out,
        )


def _unidentified(kind: DeviceKind, samples: Sequence[Sample], out: TextIO) -> None:
    """Item 114's evidence: the field next to the duty and current of the same block."""
    print("  the unidentified u16 at +0x0A, in the reports that carried a measurement:", file=out)
    seen = False
    for number in kind.aquabus_outputs:
        rows: dict[tuple[int, int, int, int], int] = {}
        for sample in samples:
            fan = sample.status.fans[number - 1]
            if not _measured(fan) or fan.unidentified_raw is None:
                continue
            key = (fan.duty, fan.current_ma, fan.power_cw, fan.unidentified_raw)
            rows[key] = rows.get(key, 0) + 1
        for (duty, current, power, raw), count in sorted(rows.items()):
            seen = True
            scaled = raw * duty / 10000.0
            print(
                f"    pwm{number}  duty {format_percent(duty):>8}  {current:4d} mA  "
                f"{power / 100:5.2f} W  +0x0A {raw:5d}  (+0x0A x duty = {scaled:5.2f} mA)  "
                f"x{count}",
                file=out,
            )
    if not seen:
        print("    no aquabus block carried a measurement in this run", file=out)


def _raw(kind: DeviceKind, samples: Sequence[Sample], out: TextIO) -> None:
    print("  every report, aquabus blocks (rpm, duty, V, mA, cW, +0x0A):", file=out)
    first = samples[0].t
    for i, sample in enumerate(samples):
        parts = []
        for number in kind.aquabus_outputs:
            fan = sample.status.fans[number - 1]
            if not fan.present:
                parts.append(f"pwm{number} -")
                continue
            parts.append(
                f"pwm{number} {fan.rpm:5d} {fan.duty:5d} {fan.voltage_v:5.2f} "
                f"{fan.current_ma:4d} {fan.power_cw:4d} {fan.unidentified_raw:5d}"
                f"{'*' if _measured(fan) else ' '}"
            )
        print(f"    {i:4d} {sample.t - first:7.2f}s  " + "  ".join(parts), file=out)


def summarise(kind: DeviceKind, samples: Sequence[Sample], out: TextIO, *, raw: bool) -> None:
    """Prints what the run measured; ``*`` marks a block that carried a measurement."""
    if not samples:
        print("  no status report arrived", file=out)
        return
    span = samples[-1].t - samples[0].t
    periods = [b.t - a.t for a, b in zip(samples, samples[1:], strict=False)]
    line = f"  {len(samples)} status reports in {span:.1f} s"
    if periods:
        line += f"  ({_spread(periods, 's')})"
    print(line, file=out)
    if any(period == 0.0 for period in periods):
        print("    (some reports were drained together and share a timestamp)", file=out)
    if not kind.aquabus_outputs:
        print(f"  the {kind.name} has no aquabus outputs", file=out)
        return
    _presence(kind, samples, out)
    _refresh(kind, samples, out)
    _unidentified(kind, samples, out)
    values = {
        name: [s.status.temp(name) for s in samples if s.status.temp(name) is not None]
        for name in kind.aquabus_temp_names
    }
    live = {name: v for name, v in values.items() if v}
    if live:
        shown = ", ".join(f"{name} {min(v):.2f}..{max(v):.2f} degC" for name, v in live.items())
        print(f"  aquabus temperature slots with a value: {shown}", file=out)
    else:
        print("  no aquabus temperature slot carried a value", file=out)
    if raw:
        _raw(kind, samples, out)


def watch(
    *,
    kind: DeviceKind = AQUAERO,
    reports: int | None = None,
    seconds: float | None = None,
    serial: str | None = None,
    sysfs_root: Path = DEFAULT_SYSFS_ROOT,
    dev_dir: Path = DEFAULT_DEV_DIR,
    opener: Opener | None = None,
    clock: Callable[[], float] = time.monotonic,
    out: TextIO | None = None,
    raw: bool = False,
) -> int:
    """Watches one controller's status reports and prints the summary.

    Exit code: 0 measured, 1 no matching device, 3 the device could not be read.
    """
    out = sys.stdout if out is None else out
    open_node: Opener = opener if opener is not None else HidrawTransport.open
    devices = [
        info
        for info in list_hidraw_devices(sysfs_root, dev_dir)
        if matches_kind(info, kind) and (serial is None or info.serial == serial)
    ]
    if not devices:
        suffix = f" with serial {serial!r}" if serial is not None else ""
        print(f"no {kind.name}{suffix} found under {sysfs_root}", file=out)
        return 1
    info = devices[0]
    print(f"{kind.name} {info.serial or '(no serial)'} at {info.node}", file=out)
    try:
        transport = open_node(info)
    except DeviceUnavailable as exc:
        print(f"  cannot open: {exc}", file=out)
        return 3
    samples: list[Sample] = []
    deadline = None if seconds is None else clock() + seconds
    try:
        while True:
            if reports is not None and len(samples) >= reports:
                break
            remaining = None if deadline is None else deadline - clock()
            if remaining is not None and remaining <= 0:
                break
            transport.wait_readable(1.0 if remaining is None else min(1.0, remaining))
            now = clock()
            for report in transport.read_reports():
                if is_status_report(kind, report):
                    samples.append(Sample(now, decode_status(kind, report)))
    except DeviceUnavailable as exc:
        print(f"  device went away after {len(samples)} reports: {exc}", file=out)
        summarise(kind, samples, out, raw=raw)
        return 3
    finally:
        transport.close()
    summarise(kind, samples, out, raw=raw)
    return 0


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(prog="aquabus_watch", description=__doc__.split("\n\n")[0])
    p.add_argument("--reports", type=int, default=None, help="stop after this many status reports")
    p.add_argument("--seconds", type=float, default=None, help="stop after this many seconds")
    p.add_argument("--serial", help="only the device with this serial")
    p.add_argument("--raw", action="store_true", help="print every report's aquabus blocks")
    p.add_argument(
        "--sysfs-root", default=str(DEFAULT_SYSFS_ROOT), help="hidraw class directory in sysfs"
    )
    p.add_argument("--dev-dir", default=str(DEFAULT_DEV_DIR), help="directory of the device nodes")
    return p


def main(argv: Sequence[str] | None = None, *, opener: Opener | None = None) -> int:
    args = build_parser().parse_args(argv)
    if args.reports is None and args.seconds is None:
        args.reports = 120
    for name in ("reports", "seconds"):
        value = getattr(args, name)
        if value is not None and not value > 0:
            print(f"--{name} must be > 0", file=sys.stderr)
            return 2
    return watch(
        reports=args.reports,
        seconds=args.seconds,
        serial=args.serial,
        sysfs_root=Path(args.sysfs_root),
        dev_dir=Path(args.dev_dir),
        opener=opener,
        raw=args.raw,
    )


if __name__ == "__main__":
    sys.exit(main())
