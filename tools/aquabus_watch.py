#!/usr/bin/env python3
"""Read-only watch of an aquaero's aquabus: how the electrical fields are sampled,
presence, and the unidentified fan-block field (PROJECT.md section 8 items 115, 114, 92).

Run on the Pi (the service user, or any user in ``plugdev``)::

    tools/aquabus_watch.py --reports 120
    tools/aquabus_watch.py --seconds 600 --raw

It reads status reports and **nothing else**: no feature report is fetched, no
control report is written, no duty is touched. It is the measurement behind three
questions the captures left open.

*How are an aquabus block's electrical fields sampled?* Only speed and output duty
are in every report. The voltage, current, power and the ``+0x0A`` word of blocks
5-8 come from one instantaneous sample inside the output's PWM cycle, so the share
of reports carrying a non-zero current follows the **duty**: 4 of 14 at 25 %, 16 of
16 at 60 %, 18 of 20 at 100 %, 11 of 45 at 20 % (PROJECT.md section 2). This prints,
per block, how many reports carried one against that block's duty, and whether the
blocks alternate together -- they did in all 45 reports of 2026-09-18, which is one
sampling instant for the whole device and not a per-block refresh. An earlier
version of this tool read the same count as a fixed bus-poll interval of about four
reports; that reading is withdrawn, and a run at two duties is what shows why.

*Is a bus device there the whole time?* An aquabus block with no device behind it
reads speed ``0xFFFF``, and that is how the daemon judges the bus
(``aquabus_present``, item 92). The run prints the longest stretch of reports
with no device, so a healthy bus can be shown to stay quiet under the daemon's
``bus_absent_s``.

*What is the ``u16`` at ``+0x0A`` of a fan block?* Unidentified (item 114). It is
printed raw next to that block's duty, current and power, which are three
coordinates of the same sample -- no scaling of the current fits it (one report
read 2 mA with the field at 0 next to nine that read 6 mA with 26). Nothing in the
daemon reads the field.

Classifying a report as one whose sample fell in the on phase is a judgement from
what the fields hold: a block that reads 0.00 V (an output with no fan, which reads
the rail when the sample misses) or any non-zero current, power or ``+0x0A``
carried one. A *stopped* fan on a populated output measures 0 mA legitimately, so a
block like that cannot be told apart, and ``--raw`` prints every report's fields
for the owner to look at instead.
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


def _sampled(fan: FanStatus) -> bool:
    """This block's electrical fields carry a sample taken in the on phase of the
    output's PWM cycle.

    Module docstring: a populated block whose sample missed reads the aquaero's own
    rail with 0 mA / 0 W, and an empty output reads 0.00 V in a report whose sample
    landed. A block with no device behind it at all (speed ``0xFFFF``) carries
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


def _sampling(kind: DeviceKind, samples: Sequence[Sample], out: TextIO) -> None:
    """How often a block's electrical group carried a sample, against its duty.

    The share is the measurement that matters: it tracks the duty, so a run at one
    duty says nothing on its own and two runs at different duties say everything
    (module docstring). The spread of the gaps is printed as well, because it is what
    an interval would have to be read from -- and is not one.
    """
    print(
        "  aquabus blocks carrying a sample (the bus device's own voltage and current):", file=out
    )
    sampled: dict[int, list[bool]] = {}
    for number in kind.aquabus_outputs:
        flags = [_sampled(s.status.fans[number - 1]) for s in samples]
        sampled[number] = flags
        indices = [i for i, flag in enumerate(flags) if flag]
        gaps = [float(b - a) for a, b in zip(indices, indices[1:], strict=False)]
        seconds = [samples[b].t - samples[a].t for a, b in zip(indices, indices[1:], strict=False)]
        duties = {s.status.fans[number - 1].duty for s in samples}
        duty = (
            format_percent(next(iter(duties)))
            if len(duties) == 1
            else f"{format_percent(min(duties))}..{format_percent(max(duties))}"
        )
        share = len(indices) / len(samples)
        line = (
            f"    pwm{number}  duty {duty:>8}  {len(indices)} of {len(samples)} "
            f"reports ({share:.0%})"
        )
        if gaps:
            line += f"  {_spread(gaps, 'reports')}  {_spread(seconds, 's')}"
        print(line, file=out)
    together = sum(1 for i in range(len(samples)) if all(f[i] for f in sampled.values()))
    any_block = sum(1 for i in range(len(samples)) if any(f[i] for f in sampled.values()))
    if any_block:
        verdict = (
            "one sampling instant for the whole device"
            if together == any_block
            else "so the blocks are not sampled together"
        )
        print(
            f"    {together} of {any_block} sampling reports carried every block: {verdict}",
            file=out,
        )


def _unidentified(kind: DeviceKind, samples: Sequence[Sample], out: TextIO) -> None:
    """Item 114's evidence: the field next to the duty, current and power of the
    same sample.

    They are printed as the distinct combinations they took and how often, because
    that is the shape of the evidence: all four come from one instant inside the PWM
    cycle, so a pair of them is not a relation between two quantities and nothing
    here fits one for the reader (module docstring).
    """
    print("  the unidentified u16 at +0x0A, in the reports that carried a sample:", file=out)
    seen = False
    for number in kind.aquabus_outputs:
        rows: dict[tuple[int, int, int, int], int] = {}
        for sample in samples:
            fan = sample.status.fans[number - 1]
            if not _sampled(fan) or fan.unidentified_raw is None:
                continue
            key = (fan.duty, fan.current_ma, fan.power_cw, fan.unidentified_raw)
            rows[key] = rows.get(key, 0) + 1
        for (duty, current, power, raw), count in sorted(rows.items()):
            seen = True
            print(
                f"    pwm{number}  duty {format_percent(duty):>8}  {current:4d} mA  "
                f"{power / 100:5.2f} W  +0x0A {raw:5d}  x{count}",
                file=out,
            )
    if not seen:
        print("    no aquabus block carried a sample in this run", file=out)


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
                f"{'*' if _sampled(fan) else ' '}"
            )
        print(f"    {i:4d} {sample.t - first:7.2f}s  " + "  ".join(parts), file=out)


def summarise(kind: DeviceKind, samples: Sequence[Sample], out: TextIO, *, raw: bool) -> None:
    """Prints what the run measured; ``*`` marks a block that carried a sample."""
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
    _sampling(kind, samples, out)
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
