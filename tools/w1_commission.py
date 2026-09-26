#!/usr/bin/env python3
"""Sensor commissioning for the 1-Wire buses (the DAS plan, section 10).

Run on the Pi, once, before the daemon starts::

    tools/w1_commission.py --list
    tools/w1_commission.py --identify
    tools/w1_commission.py --check --config /etc/aqua-bridge/config.yaml

``--list`` prints every DS18B20 (family ``28``) ROM id found on every bus
master with its current reading, for copying into ``onewire.sensors``, and
names any device of another family separately: an unterminated bus
manufactures a fresh family-``00`` phantom on every kernel search (PROJECT.md
section 9 "Overlays and modules"), so a list of those is a wiring report, not
a sensor list. ``--identify`` samples every
discovered sensor repeatedly while you warm one with a finger and reports
the sensors ranked by how fast their reading rose, so a ROM id can be bound
to a name (``prox_b01``, ``air_z0``, ...) with confidence. ``--check
--config ...`` builds the exact composite hardware source the daemon would
build (:func:`aqua_bridge.hw.sources.build_composite_from_config`), so every
startup binding error (a name not bound exactly once across
controllers/onewire) is caught here first, then runs a few dozen read cycles
per bus and prints, per bus, which read path it ended up on (netlink, the
kernel's bulk read, or one sensor at a time -- ``hw/onewire.py``, "Reading
strategy"), the conversion time the
driver reports for the configured resolution, the measured cycle time and,
per sensor, how its reads turned out against the attempts it actually got
(plan section 12 risk 5: "measure with w1_commission.py --check").

``--check`` is the **only reader on the bus while it measures**: it builds the
daemon's composite source with the reader threads left unstarted and drives
the cycles itself, because two cycles overlapping on one bus master consume
each other's readings -- 6.4 s per cycle instead of 1.0 and a third of the
reads lost, which is how this was found (PROJECT.md section 8 item 39,
``hw/onewire.py`` "One cycle at a time per bus master"). A *running daemon* is
a second reader in a different process, which the in-process lock above
cannot reach, so before driving a single cycle ``--check`` tries to take the
same cross-process ``onewire.lock_path`` lock the daemon holds while its
reader threads run (:class:`~aqua_bridge.hw.onewire.ReaderLock`) and refuses
to measure if it cannot: reporting a cycle time or a failure rate while
something else is converting on the bus would print exactly the corruption
above, credibly, as if it were the hardware's fault. Three outcomes: the lock
is free (nothing else is reading -- measures normally), it is held (refuses,
and says why), or it cannot be told either way (refuses the same as held,
since a wrong guess here is a wrong measurement, not a wrong warning).
``--force`` measures anyway in the last two cases; see its ``--help`` text
for what that costs.

This tool imports :mod:`aqua_bridge.config` / :mod:`aqua_bridge.hw` only,
never :mod:`aqua_bridge.control`: commissioning runs stand-alone, before an
``MpcConfig`` is meaningful to the sensors themselves.
"""

from __future__ import annotations

import argparse
import re
import sys
import time
from collections.abc import Callable, Mapping
from pathlib import Path

from aqua_bridge.hw.onewire import DEFAULT_ROOT, ReaderLockOutcome, ReadStats, W1Source

__all__ = [
    "build_parser",
    "cmd_check",
    "cmd_identify",
    "cmd_list",
    "discover_all",
    "discover_other_families",
    "main",
    "rank_by_warming_rate",
]

#: Default ``--check`` detector: try to take the same cross-process lock the
#: daemon holds while its reader threads run (``hw/onewire.py``,
#: :class:`~aqua_bridge.hw.onewire.ReaderLock`). Injected so tests exercise
#: the refusal / override / "cannot tell" branches with a fake outcome and
#: never touch a real lock file or systemd (see ``tests/test_w1_commission.py``).
DetectRunningDaemon = Callable[[W1Source], ReaderLockOutcome]
_default_detect: DetectRunningDaemon = W1Source.acquire_reader_lock

_ROM_PATTERN = re.compile(r"^[0-9a-f]{2}-[0-9a-f]{12}$")
#: DS18B20 family. Every other ROM-shaped directory on a bus is reported apart:
#: on an unwired bus the kernel's periodic search invents family-``00`` devices
#: (PROJECT.md section 8 item 38), and listing those as sensors to bind would be
#: an invitation to bind a phantom.
_DS18B20_PATTERN = re.compile(r"^28-[0-9a-f]{12}$")


def _discover(root: Path, *, ds18b20: bool) -> dict[str, list[str]]:
    out: dict[str, list[str]] = {}
    if not root.is_dir():
        return out
    for bus_dir in sorted(root.glob("w1_bus_master*")):
        if not bus_dir.is_dir():
            continue
        roms = sorted(
            p.name
            for p in bus_dir.iterdir()
            if p.is_dir()
            and _ROM_PATTERN.match(p.name)
            and bool(_DS18B20_PATTERN.match(p.name)) is ds18b20
        )
        out[bus_dir.name] = roms
    return out


def discover_all(root: Path) -> dict[str, list[str]]:
    """Bus master name -> sorted DS18B20 ROM ids currently found under it.

    Discovery, not binding: every family-``28`` subdirectory is listed, not
    only ones a config declares (there is no config yet at this point).
    Devices of other families are :func:`discover_other_families`' business.
    """
    return _discover(root, ds18b20=True)


def discover_other_families(root: Path) -> dict[str, list[str]]:
    """Bus master name -> sorted ROM ids that are not DS18B20, buses without any omitted.

    Anything here is either a 1-Wire device this project has no use for or,
    family ``00``, the phantom an unterminated bus produces on every search.
    """
    return {bus: roms for bus, roms in _discover(root, ds18b20=False).items() if roms}


def _flatten_roms(roms_by_bus: dict[str, list[str]]) -> dict[str, str]:
    return {rom: rom for roms in roms_by_bus.values() for rom in roms}


def _probe_source(root: Path, sensors: dict[str, str]) -> W1Source:
    """A throwaway :class:`W1Source` whose sensor *names* are the ROM ids
    themselves, so :meth:`W1Source.run_bus_cycle` can be driven directly
    for discovery/identify/check without an ``onewire.sensors`` binding."""
    return W1Source(sensors, max_age_s=3600.0, root=root)


def cmd_list(root: Path) -> int:
    roms_by_bus = discover_all(root)
    others = discover_other_families(root)
    if not roms_by_bus:
        print(f"no w1 bus master found under {root}")
        return 1
    sensors = _flatten_roms(roms_by_bus)
    if not sensors:
        _print_other_families(others)
        print("no DS18B20 (family 28) found on any bus")
        return 1
    src = _probe_source(root, sensors)
    for bus_name, roms in roms_by_bus.items():
        readings = src.run_bus_cycle(root / bus_name)
        print(f"{bus_name}:")
        for rom in roms:
            value = readings.get(rom)
            shown = f"{value:.3f} C" if value is not None else "(no reading)"
            print(f"  {rom}  {shown}")
    _print_other_families(others)
    return 0


def _print_other_families(others: dict[str, list[str]]) -> None:
    for bus_name, roms in others.items():
        print(f"{bus_name}: {len(roms)} device(s) of another family, not DS18B20: {roms}")
        if any(rom.startswith("00-") for rom in roms):
            print(
                "  family 00 is what the kernel's periodic search reads off an "
                "unterminated bus; wire and terminate it (4.7 kOhm), or drop its "
                "w1-gpio overlay (PROJECT.md section 9)"
            )


def rank_by_warming_rate(series: dict[str, list[float | None]]) -> list[tuple[str, float]]:
    """``[(rom, total_rise_c)]`` sorted descending -- the fastest-rising
    sensor first. A pure function of the sampled series so the ranking
    logic is unit-testable without a real 1-Wire bus; missing samples
    (``None``, a failed read mid-series) are dropped before comparing the
    first and last finite reading.
    """
    ranked: list[tuple[str, float]] = []
    for rom, values in series.items():
        finite = [v for v in values if v is not None]
        if len(finite) < 2:
            continue
        ranked.append((rom, finite[-1] - finite[0]))
    ranked.sort(key=lambda pair: pair[1], reverse=True)
    return ranked


def cmd_identify(root: Path, *, samples: int = 10, interval_s: float = 2.0) -> int:
    roms_by_bus = discover_all(root)
    sensors = _flatten_roms(roms_by_bus)
    if not sensors:
        print("no ROM ids found on any bus")
        return 1
    src = _probe_source(root, sensors)
    print(f"warm one sensor with a finger; sampling {samples} times every {interval_s:.0f}s ...")
    series: dict[str, list[float | None]] = {rom: [] for rom in sensors}
    for i in range(samples):
        for bus_name, roms in roms_by_bus.items():
            readings = src.run_bus_cycle(root / bus_name)
            for rom in roms:
                series[rom].append(readings.get(rom))
        if i < samples - 1:
            time.sleep(interval_s)
    ranked = rank_by_warming_rate(series)
    if not ranked:
        print("not enough readings to rank any sensor")
        return 1
    print("ranked by rise (fastest first):")
    for rom, delta in ranked:
        print(f"  {rom}  {delta:+.3f} C")
    print(f"likely the one you warmed: {ranked[0][0]}")
    return 0


#: ``cmd_check``'s exit code when it refuses to measure: the daemon (or
#: another reader) was detected, or could not be ruled out, and ``--force``
#: was not given. Distinct from 2 (a config error) so a script can tell "the
#: config is fine, something else is on the bus" apart from "the config is
#: wrong".
EXIT_DAEMON_DETECTED = 3


def cmd_check(
    config_path: str,
    *,
    cycles: int = 20,
    force: bool = False,
    detect: DetectRunningDaemon = _default_detect,
) -> int:
    from aqua_bridge.config import ConfigError, load_config
    from aqua_bridge.hw.sources import build_composite_from_config

    try:
        app = load_config(config_path)
    except ConfigError as exc:
        print(f"config error: {exc}", file=sys.stderr)
        return 2

    try:
        composite, release = build_composite_from_config(
            aquacomputer_section=app.aquacomputer,
            xt6_section=app.section("xt6"),
            onewire_section=app.section("onewire"),
            channels=app.mpc.channels,
            temps=app.mpc.temps,
            dt=app.mpc.dt,
            # The daemon's builder starts one reader thread per bus; this
            # command then drives cycles itself, and two cycles overlapping on
            # one bus master consume each other's readings -- 6.4 s per cycle
            # instead of 1.0 and a third of the reads lost, measured on the
            # board (PROJECT.md section 8 item 39). So: build everything the
            # daemon builds, including every binding check, and be the *only*
            # reader while measuring.
            start_readers=False,
        )
    except ConfigError as exc:
        print(f"config error: {exc}", file=sys.stderr)
        return 2

    print("binding check: every mpc.temps / mpc.channels name is bound exactly once. OK")
    onewire = composite.onewire
    if onewire is None:
        print("no onewire.sensors configured; nothing more to check")
        if release is not None:
            release()
        return 0

    try:
        outcome = detect(onewire)
        if outcome is not ReaderLockOutcome.ACQUIRED:
            _print_daemon_detected(outcome, onewire.lock_path)
            if not force:
                return EXIT_DAEMON_DETECTED
            print(
                "--force: measuring anyway. The numbers below will be exactly this wrong if "
                "something really is reading these buses -- inflated cycle time, reads lost to "
                "a stolen conversion, not a hardware fault."
            )

        missing = onewire.missing_roms()
        if missing:
            print(f"WARNING: ROM id(s) declared in config but not found on any bus: {missing}")

        buses = onewire.discover_buses()
        if not buses:
            print(f"WARNING: no w1 bus master found under {onewire.root}")
        for bus_dir in buses:
            t0 = time.monotonic()
            for _ in range(cycles):
                onewire.run_bus_cycle(bus_dir)
            elapsed = time.monotonic() - t0
            # Which tier produced that number. A cycle time read against the wrong
            # tier is worse than no number: netlink and the kernel's bulk read each
            # cost one conversion for the whole bus, one sensor at a time costs one
            # conversion each (PROJECT.md section 8 item 38).
            tier = onewire.read_tiers().get(bus_dir.name, "unprobed")
            print(
                f"{bus_dir.name}: {elapsed / cycles * 1000:.0f} ms/cycle over {cycles} cycles "
                f"({tier} reads)"
            )
        conv_times = sorted(set(onewire.conv_time_ms().values()))
        if conv_times:
            print(
                f"conversion time the driver reports at the configured resolution: {conv_times} ms"
            )
        for bus_name, roms in discover_other_families(onewire.root).items():
            print(f"WARNING: {bus_name} carries {len(roms)} non-DS18B20 device(s): {roms}")

        _print_read_stats(onewire.read_stats(), declared_but_absent=set(missing))
        return 0
    finally:
        onewire.release_reader_lock()
        if release is not None:
            release()


def _print_daemon_detected(outcome: ReaderLockOutcome, lock_path: Path) -> None:
    """Explains *why* the measurement would be wrong, not merely that
    something is running: two cycles overlapping on one bus master consume
    each other's kernel "value ready" marks, so a cycle pays a fresh
    conversion per sensor the other cycle claimed first, and a read that
    lands inside the other cycle's conversion comes back empty -- measured at
    6377 ms/cycle and a third of the reads lost against 1044 ms/cycle and
    none, on the same board, same sensors (PROJECT.md item 39). Printed to
    stderr like the other refusals in this tool.
    """
    corruption = (
        'Two cycles overlapping on one bus master consume each other\'s kernel "value ready" '
        "marks: a cycle pays a fresh conversion for every sensor the other cycle claimed "
        "first, and a read landing inside that conversion comes back empty. Measured "
        "this way: 6377 ms/cycle and up to a third of the reads lost, against 1044 ms/cycle "
        "and none once there was only one reader (PROJECT.md item 39). This is not a hardware "
        "fault, and the numbers below would report it as one."
    )
    if outcome is ReaderLockOutcome.HELD:
        print(
            f"refusing to measure: {lock_path} is held by another reader of these buses -- "
            f"most likely aqua-bridge.service. {corruption}\n"
            "Stop the daemon and run this again, or pass --force if you accept the numbers "
            "will be wrong.",
            file=sys.stderr,
        )
    else:
        print(
            f"refusing to measure: cannot tell whether another reader has {lock_path} -- the "
            "lock directory does not exist and could not be created, a permission error, or a "
            f"filesystem without flock(2) support (see the log for which). {corruption}\n"
            "Confirm nothing else is reading these buses, then pass --force.",
            file=sys.stderr,
        )


def _print_read_stats(stats: Mapping[str, ReadStats], *, declared_but_absent: set[str]) -> None:
    """Per-sensor read outcomes, each against the attempts it actually got.

    Three different things used to print as one "failed reads" number over a
    denominator that belonged to neither of them -- ``cycles * len(buses)``,
    which is the attempts of no sensor at all on a board with two buses, and
    which made a ROM id that is not on any bus read as ``0/40 failed (0.0%)``,
    the most reassuring line in the report (PROJECT.md section 8 item 39). What
    a sensor's reads did is:

    * *not attempted* -- no cycle ever reached it. Either it is declared in
      config and not under any bus master (a binding or wiring report, and
      ``missing_roms()`` already said so above), or every cycle on its bus was
      cut short before it. Never a rate.
    * *errors* -- the kernel refused the read (EIO: the scratchpad failed its
      CRC), or a netlink scratchpad did not come back. This is the number item
      39's "< 1 %" is about.
    * *empty* -- the read answered with nothing, which the kernel does while a
      bulk conversion it started is in flight: someone else is reading this bus.
    * *rejected* -- it answered something that is not a temperature.
    """
    for rom, stat in sorted(stats.items()):
        if stat.attempts == 0:
            why = (
                "not under any bus master" if rom in declared_but_absent else "no cycle reached it"
            )
            print(f"{rom}: no read attempted ({why})")
            continue
        rate = stat.failed / stat.attempts
        detail = ", ".join(
            f"{count} {label}"
            for label, count in (
                ("errno/CRC", stat.errors),
                ("empty", stat.empty),
                ("rejected", stat.rejected),
            )
            if count
        )
        flag = "  WARNING: > 1%" if rate > 0.01 else ""
        print(
            f"{rom}: {stat.failed}/{stat.attempts} reads failed ({rate:.1%})"
            f"{' [' + detail + ']' if detail else ''}{flag}"
        )
    empty = sum(stat.empty for stat in stats.values())
    if empty:
        print(
            f"WARNING: {empty} read(s) answered nothing at all. The kernel does that while a "
            "bulk conversion it started is still in flight, so a second reader was on the bus "
            "-- stop the daemon (aqua-bridge.service) and run this again (PROJECT.md item 39)"
        )


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(prog="w1_commission", description=__doc__.split("\n\n")[0])
    group = p.add_mutually_exclusive_group(required=True)
    group.add_argument(
        "--list", action="store_true", help="list every ROM id per bus with its current reading"
    )
    group.add_argument(
        "--identify", action="store_true", help="find which ROM id is the sensor you are warming"
    )
    group.add_argument(
        "--check",
        action="store_true",
        help=(
            "verify bindings, cycle time and CRC error rate; refuses to measure if another "
            "reader of these buses is detected, or cannot be ruled out (see --force)"
        ),
    )
    p.add_argument("--config", help="config.yaml path (required with --check)")
    p.add_argument("--root", default=DEFAULT_ROOT, help=f"w1 sysfs root (default {DEFAULT_ROOT})")
    p.add_argument("--samples", type=int, default=10, help="--identify: sample count (default 10)")
    p.add_argument(
        "--interval", type=float, default=2.0, help="--identify: seconds between samples"
    )
    p.add_argument(
        "--cycles", type=int, default=20, help="--check: read cycles per bus (default 20)"
    )
    p.add_argument(
        "--force",
        action="store_true",
        help=(
            "--check: measure even though another reader of these buses was detected, or "
            "could not be ruled out. The numbers will be wrong in exactly the way PROJECT.md "
            "item 39 describes if the daemon (or anything else) really is reading these buses "
            "-- inflated cycle time, reads lost to a stolen conversion, reported as if it were "
            "a hardware fault. Use only once you understand why the detection said what it did."
        ),
    )
    return p


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    root = Path(args.root)
    if args.list:
        return cmd_list(root)
    if args.identify:
        return cmd_identify(root, samples=args.samples, interval_s=args.interval)
    if not args.config:
        print("--check requires --config", file=sys.stderr)
        return 2
    return cmd_check(args.config, cycles=args.cycles, force=args.force)


if __name__ == "__main__":
    sys.exit(main())
