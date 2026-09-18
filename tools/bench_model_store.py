#!/usr/bin/env python3
"""Benchmark ``model.json`` writes (``aqua_bridge.modelstore``) at the size the
store actually reaches, against a **scratch path only** (PROJECT.md section 8
item 48).

Builds one realistic snapshot -- a DAS closed loop of ``--warm-ticks`` steps
against ``config.example-das.yaml`` (or ``--config``), so the estimator's
calibration, the thermal model and (with ``--sim-preset rich``) more of what a
running daemon actually accumulates are in ``solver_memory`` before anything
is timed, not an empty cold-start document -- then serialises it exactly as
:meth:`aqua_bridge.modelstore.ModelPersister.save` does
(:func:`aqua_bridge.modelstore.build_document`, then
``json.dumps(doc, allow_nan=False, separators=(",", ":"))``) and times
:func:`aqua_bridge.modelstore.write_atomic` -- open, write, ``fsync``,
``os.replace``, ``fsync`` of the directory, the same call the daemon makes --
against ``--path`` for ``--repeats`` repeats.

**The warm-up is forced to actually reach that state (PROJECT.md section 8 item
133).** Three things the closed loop alone does not produce against the shipped
example, however many ``--warm-ticks``, and each was checked against
``config.example-das.yaml`` before this was written, not assumed:

* ``calibration`` needs SMART samples, and none of the shipped examples declare a
  bay's serial (PROJECT.md section 3, "no SES backplane" -- the estimator finds the
  mapping by correlation instead). :func:`aqua_bridge.sim.das.DriveSpec` reads
  ``serial`` straight from the topology dict, so an occupied bay with none never
  schedules a SMART sample at all (:meth:`~aqua_bridge.sim.das.DasPlant._schedule_smart`):
  zero SMART traffic however long the loop runs. This tool's warm-up gives the *truth*
  plant (never ``cfg.topology``, which stays exactly as loaded) a synthetic serial per
  occupied bay with none (``tools/bench_step.py``'s ``das_plant(..., inject_smart_serials=True)``),
  so the estimator has something to correlate and calibrate against, the same way a
  real enclosure's unlabelled drives do.
* ``fan_curves`` needs ``mpc.fan_curve_online: true``, which the shipped examples leave
  off (a config decision, not a bug); the warm-up runs against
  ``dataclasses.replace(cfg, fan_curve_online=True)`` instead of the loaded config, and
  the written document is built and timed against that same forced config, reported as
  ``result.fan_curve_online_forced``.
* Forcing the flag alone is not enough: measured directly (2026-09-18), a closed loop
  that is *always actively regulating* practically never holds a duty within
  :data:`aqua_bridge.control.fancurve.SETTLE_TOL` (1e-6) for
  ``mpc.fan_curve_settle_s`` straight -- sensor noise alone moves the MPC/PI output by
  more than that most ticks. Six simulated hours of the ordinary closed loop left the
  fit's bin accumulator with exactly one usable bin (the ``pwm_max`` clamp, the one
  duty that ever holds bit-for-bit), never the four bins across a 0.25 span
  :data:`aqua_bridge.control.fancurve.MIN_BINS` / ``MIN_SPAN`` need -- not a "modest
  number of ticks" problem, a structural one. So the warm-up appends a short, separate
  dwell scan after the closed loop (:func:`_fan_curve_dwell_scan`): it holds every
  channel at a few fixed duties spanning ``[mpc.pwm_min, mpc.pwm_max]`` long enough to
  settle, feeding the *same* :func:`aqua_bridge.control.fancurve.update` the daemon's
  own ``step()`` calls -- a commissioning-style sweep through the real online-fit code,
  not a fabricated curve.

Reports the size in bytes, the min / mean / median / p99 / max write time in
milliseconds, the spread (max - min) over the repeats, and that p99 as a
fraction of ``dt`` (the tick period) and of ``mpc.model_store_interval_s``
(how often the daemon actually writes; a write happens on at most one tick out
of every ``model_store_interval_s / dt`` of them, and the write itself never
delays the fan command -- ``control/loop.py`` runs ``on_tick`` observers,
model.json's persister included, only after ``apply()`` has already sent that
tick's PWM and after the step-budget gate has already been measured and
recorded on ``step()`` alone -- but a slow write still runs inside ``tick()``
and so delays the *next* tick's read, which is what the fraction of
``dt`` is checking). Also reports whether ``fan_curve_online`` was forced on for this
run (``fan_curve_online_forced``, true whenever ``--config`` itself leaves it off),
how many bays reached an accepted calibration (``calibrated_bays``) and which fan
models reached an accepted curve (``fan_curve_fit_accepted``) -- so a reader can tell
a genuinely empty section (nothing accepted in ``--warm-ticks``) from one this warm-up
never tries to populate at all.

**Safety**: refuses to write anywhere but under a scratch directory
(``tempfile.gettempdir()``, ``/tmp`` or ``/var/tmp``) -- never the daemon's
own ``$STATE_DIRECTORY/model.json`` or a path under ``/opt/aqua-bridge`` /
``/etc/aqua-bridge``. The default ``--path`` is already such a scratch file
and is removed again at the end of the run unless ``--keep`` is given.

**Medium (PROJECT.md section 8 item 148).** A path under a scratch directory
is not necessarily disk: on the owner's board ``/tmp`` is ``tmpfs``, so
``tools/bench_model_store.py --path /tmp/...`` was writing to RAM and
reporting RAM write-and-``fsync`` latency as if it were the SD card's --
item 48's first board number, silently. So before timing anything,
:func:`detect_medium` resolves ``--path`` against ``/proc/mounts`` (the same
source ``findmnt`` reads) and the mount, filesystem type and backing device
are printed to stderr and recorded in the report's ``medium`` key,
regardless of what that medium turns out to be. When it is RAM-backed
(``tmpfs`` or ``ramfs``) the tool refuses to run, the same way
``ensure_scratch_path`` already refuses a non-scratch path, unless
``--allow-ram-backed`` says the RAM numbers are wanted on purpose -- for a
labelled comparison against a disk-backed run, which is exactly how this
tool was used to settle item 148. Either way the report's
``medium.ram_backed`` says which kind of run produced it, so a reader is
never left to guess, and a future run cannot repeat item 48's original
mistake without saying so loudly first.

Usage::

    python tools/bench_model_store.py
    python tools/bench_model_store.py --config config.example-das.yaml \\
        --warm-ticks 8640 --sim-preset rich --repeats 30 --path /var/tmp/model-bench.json
"""

from __future__ import annotations

import argparse
import dataclasses
import importlib.util
import json
import os
import platform
import statistics
import sys
import tempfile
import time
from pathlib import Path

from aqua_bridge.config import load_config
from aqua_bridge.control import fancurve
from aqua_bridge.control.loop import TickResult
from aqua_bridge.control.mpc import step
from aqua_bridge.model import MpcConfig, MpcState
from aqua_bridge.modelstore import build_document, write_atomic

#: Duty levels the fan-curve dwell scan holds (module docstring): evenly spread over
#: the config's own ``[pwm_min, pwm_max]``, six points so at least
#: :data:`aqua_bridge.control.fancurve.MIN_BINS` fall in distinct accumulator bins
#: with room to spare.
_FAN_CURVE_SCAN_LEVELS = 6

REPO_ROOT = Path(__file__).resolve().parent.parent

_SCRATCH_ROOTS = tuple(
    dict.fromkeys(  # de-duplicated, in order: gettempdir() usually already is /tmp
        p.resolve() for p in (Path(tempfile.gettempdir()), Path("/tmp"), Path("/var/tmp"))
    )
)

#: Filesystem types :func:`detect_medium` treats as RAM, not disk (module docstring,
#: *Medium*): a write timed against one of these measures memory bandwidth and an
#: `fsync` that has nothing to flush, not the storage device PROJECT.md item 48 asks
#: about. ``ramfs`` has no size limit and no writeback at all, so it belongs here even
#: though nothing in this repo mounts one.
_RAM_BACKED_FSTYPES = frozenset({"tmpfs", "ramfs"})

#: /proc/mounts escapes space, tab, newline and backslash as octal in both the device
#: and mountpoint fields (see ``proc(5)``); undone here so a mountpoint containing one
#: of those -- rare, but legal -- still compares equal to the real path.
_MOUNT_ESCAPES = {"040": " ", "011": "\t", "012": "\n", "134": "\\"}


def _unescape_mount_field(field: str) -> str:
    out: list[str] = []
    i = 0
    while i < len(field):
        ch = field[i]
        code = field[i + 1 : i + 4]
        if ch == "\\" and code in _MOUNT_ESCAPES:
            out.append(_MOUNT_ESCAPES[code])
            i += 4
        else:
            out.append(ch)
            i += 1
    return "".join(out)


@dataclasses.dataclass(frozen=True)
class Medium:
    """What actually backs a scratch path (module docstring, *Medium*): the mount
    ``detect_medium`` matched, its filesystem type and its device/source field exactly
    as ``/proc/mounts`` names it (``tmpfs`` for a RAM-backed mount, a block device such
    as ``/dev/mmcblk0p2`` for a real one)."""

    mountpoint: str
    fstype: str
    device: str

    @property
    def ram_backed(self) -> bool:
        return self.fstype in _RAM_BACKED_FSTYPES

    def asdict(self) -> dict[str, object]:
        return {
            "mountpoint": self.mountpoint,
            "fstype": self.fstype,
            "device": self.device,
            "ram_backed": self.ram_backed,
        }


_UNKNOWN_MEDIUM = Medium(mountpoint="unknown", fstype="unknown", device="unknown")


def detect_medium(path: Path, *, mounts_path: Path = Path("/proc/mounts")) -> Medium:
    """The mount that actually backs ``path`` (module docstring, *Medium*): the
    longest-matching-prefix entry in ``mounts_path`` (default ``/proc/mounts``, the
    same source ``findmnt`` reads), so a scratch path under a disk-backed directory and
    one under a RAM-backed one are told apart before anything is timed. ``path`` need
    not exist yet -- the check walks up to the first existing ancestor, same as a
    relative ``--path`` under a directory that is about to be created. Never raises: a
    platform with no ``/proc/mounts`` (anything but Linux) or an unreadable one gets
    :data:`_UNKNOWN_MEDIUM`, which is not RAM-backed by construction -- silence about
    the medium is a reason to look closer, not a signal a tool should ever misuse to
    also mean tmpfs."""
    anchor = path
    while not anchor.exists() and anchor.parent != anchor:
        anchor = anchor.parent
    anchor = anchor.resolve()

    try:
        lines = mounts_path.read_text().splitlines()
    except OSError:
        return _UNKNOWN_MEDIUM

    best: Medium | None = None
    best_len = -1
    for line in lines:
        fields = line.split()
        if len(fields) < 3:
            continue
        device, raw_mountpoint, fstype = fields[0], fields[1], fields[2]
        mountpoint = _unescape_mount_field(raw_mountpoint)
        mp = Path(mountpoint)
        under_mount = anchor == mp or anchor.is_relative_to(mp)
        if under_mount and len(mountpoint) > best_len:
            best = Medium(mountpoint=mountpoint, fstype=fstype, device=device)
            best_len = len(mountpoint)
    return best if best is not None else _UNKNOWN_MEDIUM


def report_and_check_medium(path: Path, *, allow_ram_backed: bool) -> Medium:
    """:func:`detect_medium` for ``path``, always printed to stderr before anything is
    timed (module docstring, *Medium*): what backs a scratch path is not obvious from
    the path alone, and the whole point of item 148 is that it must never again be left
    to guessing. When the medium is RAM-backed, refuses -- same shape as
    :func:`ensure_scratch_path`'s refusal -- unless ``allow_ram_backed`` says the RAM
    numbers are wanted on purpose, in which case it prints a loud warning instead and
    still proceeds."""
    medium = detect_medium(path)
    print(
        f"medium: {path} -> {medium.fstype} on {medium.device}, mounted at {medium.mountpoint}",
        file=sys.stderr,
    )
    if not medium.ram_backed:
        return medium
    if not allow_ram_backed:
        raise SystemExit(
            f"refusing to benchmark {path}: {medium.mountpoint} ({medium.fstype} on "
            f"{medium.device}) is RAM-backed. This is exactly what produced PROJECT.md "
            "section 8 item 48's first board number -- RAM write-and-fsync latency reported "
            "as if it were the SD card's (section 8 item 148). Point --path at a disk-backed "
            "scratch directory (for example /var/tmp on the board), or pass --allow-ram-backed "
            "to measure tmpfs on purpose, for a labelled comparison against a disk-backed run."
        )
    banner = "!" * 70
    print(banner, file=sys.stderr)
    print(
        f"WARNING: {medium.mountpoint} ({medium.fstype} on {medium.device}) is RAM-backed -- "
        "these write/fsync numbers are RAM latency, not disk latency. --allow-ram-backed was "
        "given, so this run proceeds; the report's medium.ram_backed is true so it cannot be "
        "mistaken for a disk-backed run afterwards.",
        file=sys.stderr,
    )
    print(banner, file=sys.stderr)
    return medium


def percentile(samples: list[float], q: float) -> float:
    """Nearest-rank percentile (``q`` in [0, 100]); same method as ``tools/bench_step.py``."""
    ordered = sorted(samples)
    k = max(0, min(len(ordered) - 1, int(round(q / 100.0 * (len(ordered) - 1)))))
    return ordered[k]


def _load_bench_step():
    """``tools/bench_step.py`` as a module (``tools`` is not a package): reused for its
    ``das_plant`` builder so the warm-up closed loop here matches item 95's bench exactly,
    rather than a second, silently-drifting copy of the same plant construction."""
    spec = importlib.util.spec_from_file_location(
        "bench_step", REPO_ROOT / "tools" / "bench_step.py"
    )
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def ensure_scratch_path(path: Path) -> Path:
    """Refuse anything but a path under a scratch directory (module docstring, *Safety*)."""
    resolved = path.resolve()
    if not any(resolved.is_relative_to(root) for root in _SCRATCH_ROOTS):
        roots = " or ".join(str(r) for r in _SCRATCH_ROOTS)
        raise SystemExit(
            f"refusing --path {resolved}: not under {roots}. This tool never writes over "
            "the daemon's own model store (PROJECT.md section 8 item 48) -- pass a path "
            "under a scratch directory."
        )
    return resolved


def _fan_curve_dwell_scan(plant, cfg: MpcConfig, state: MpcState) -> MpcState:
    """Appends a fixed-duty dwell scan to ``state.solver_memory["fan_fit"]`` /
    ``["fan_curves"]`` after the closed loop (module docstring, *The warm-up is forced*):
    holds every channel of ``plant`` at :data:`_FAN_CURVE_SCAN_LEVELS` duties spanning
    ``[cfg.pwm_min, cfg.pwm_max]``, each long enough to clear ``cfg.fan_curve_settle_s``
    with a few samples to spare, feeding the real :func:`aqua_bridge.control.fancurve.update`
    the daemon's own ``step()`` calls. Pure in the sense that matters here: it only ever
    grows ``fan_fit`` / ``fan_curves``, and every reading comes from ``plant`` like any
    other tick -- nothing here is a made-up ``(pwm, rpm)`` pair."""
    span = cfg.pwm_max - cfg.pwm_min
    levels = [
        cfg.pwm_min + span * i / (_FAN_CURVE_SCAN_LEVELS - 1) for i in range(_FAN_CURVE_SCAN_LEVELS)
    ]
    # fancurve.update() only *attempts* a refit once every cfg.fan_curve_refit_s of
    # controller time since its last attempt (accepted or not) -- the bins fill on every
    # settled tick, but nothing reads them back into a fit before that clock elapses, and
    # the closed loop above already primed it to an unknown phase. So each level gets
    # enough ticks that the scan's total span safely clears a whole refit interval
    # (2x, so a scan that starts right after a refit attempt still crosses one) on top of
    # the settle time each level needs, not merely the shorter of the two.
    settle_ticks = int(cfg.fan_curve_settle_s / cfg.dt) + 10
    refit_ticks = int(2.0 * cfg.fan_curve_refit_s / cfg.dt / _FAN_CURVE_SCAN_LEVELS) + 1
    dwell_ticks = max(settle_ticks, refit_ticks)
    memory = dict(state.solver_memory)
    for level in levels:
        u = dict.fromkeys(cfg.channels, level)
        for _ in range(dwell_ticks):
            plant.apply(u)
            plant.advance()
            obs = plant.observe()
            update = fancurve.update(memory.get("fan_fit"), cfg, u=u, rpm=obs.rpm, ts=obs.ts)
            memory["fan_fit"] = update.memory
            if update.curves:
                memory["fan_curves"] = update.curves
    return dataclasses.replace(state, solver_memory=memory)


def warm_snapshot(cfg: MpcConfig, *, ticks: int, seed: int, preset: str) -> TickResult:
    """A ``TickResult`` after ``ticks`` DAS closed-loop steps plus the fan-curve dwell
    scan (module docstring): ``solver_memory`` at roughly the size a running daemon's
    store reaches, not the empty document a cold start would write. ``cfg`` should
    already carry ``fan_curve_online: true`` (:func:`bench_writes` forces it); this
    function does not force it itself, so a caller that wants the old, unforced
    behaviour still gets it by passing the config unchanged."""
    bench_step = _load_bench_step()
    plant = bench_step.das_plant(cfg, ticks, seed, preset=preset, inject_smart_serials=True)
    state = MpcState.cold()
    result: TickResult | None = None
    for i in range(ticks):
        obs = plant.observe()
        smart = plant.observe_smart()
        if smart:
            obs = dataclasses.replace(obs, inputs={"smart": smart})
        cmd, state = step(obs, cfg, state)
        result = TickResult(index=i, obs=obs, mpc_cmd=cmd, cmd=cmd, state=state, applied=True)
        plant.apply(cmd.pwm)
        plant.advance()
    assert result is not None  # ticks > 0, checked by the caller
    state = _fan_curve_dwell_scan(plant, cfg, state)
    return dataclasses.replace(result, state=state)


def bench_writes(
    cfg: MpcConfig, path: Path, *, warm_ticks: int, seed: int, preset: str, repeats: int
):
    # Forced on for the warm-up and the document alike (module docstring, *The warm-up
    # is forced*): a config that ships fan_curve_online: false can still carry an
    # accepted curve in its store from when it was true, so a document built against
    # this forced config is not a fabrication, only a choice of which real config's
    # store this size estimate is of. Reported below so a reader is never left
    # guessing which config actually produced the numbers.
    warm_cfg = dataclasses.replace(cfg, fan_curve_online=True)
    result = warm_snapshot(warm_cfg, ticks=warm_ticks, seed=seed, preset=preset)
    memory = result.state.solver_memory
    diagnostics = getattr(result.mpc_cmd, "diagnostics", None)
    bays = diagnostics.get("bays") if isinstance(diagnostics, dict) else None
    bays = bays if isinstance(bays, dict) else None
    ts = memory.get("last_ts")
    ts = float(ts) if isinstance(ts, int | float) else None
    wall = time.time()
    # ident_settle=None: this warm-up loop never runs an identification experiment, so
    # the document's "ident_settle" section is empty; calibration/fan_curves/bays are
    # not (module docstring, PROJECT.md item 133).
    doc = build_document(warm_cfg, memory, ts=ts, wall=wall, bays=bays, ident_settle=None)
    data = json.dumps(doc, allow_nan=False, separators=(",", ":")).encode()

    times_ms: list[float] = []
    for _ in range(repeats):
        t0 = time.perf_counter()
        write_atomic(path, data)
        times_ms.append((time.perf_counter() - t0) * 1e3)
    on_disk = path.stat().st_size

    p99 = percentile(times_ms, 99)
    return {
        "warm_ticks": warm_ticks,
        "sim_preset": preset,
        "fan_curve_online_forced": warm_cfg.fan_curve_online and not cfg.fan_curve_online,
        "calibrated_bays": sum(1 for v in doc["calibration"].values() if v),
        "fan_curve_fit_accepted": sorted(doc["fan_curves"]),
        "size_bytes": len(data),
        "size_bytes_on_disk": on_disk,
        "sections": sorted(k for k, v in doc.items() if k not in ("schema", "v", "saved_wall")),
        "repeats": repeats,
        "min_ms": min(times_ms),
        "mean_ms": statistics.fmean(times_ms),
        "median_ms": statistics.median(times_ms),
        "p99_ms": p99,
        "max_ms": max(times_ms),
        "spread_ms": max(times_ms) - min(times_ms),
        "dt_s": cfg.dt,
        "model_store_interval_s": cfg.model_store_interval_s,
        "p99_fraction_of_dt": p99 / (cfg.dt * 1e3),
        "p99_fraction_of_model_store_interval": p99 / (cfg.model_store_interval_s * 1e3),
        "times_ms": times_ms,
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument(
        "--config",
        default=None,
        help="zoned YAML with mpc.topology (default: config.example-das.yaml)",
    )
    parser.add_argument(
        "--path",
        default=None,
        help="scratch file to write (default: a fresh file under the system temp directory); "
        "refused unless it resolves under a scratch directory (module docstring, Safety)",
    )
    parser.add_argument(
        "--warm-ticks",
        type=int,
        default=4320,
        help="closed-loop ticks before timing (default: 4320, 6 h at dt=5 s -- section 8 item "
        "133's own measurement: enough for most bays' SMART calibration to accept and, with the "
        "dwell scan appended after it, an accepted fan-curve fit)",
    )
    parser.add_argument("--sim-preset", default="rich", choices=("basic", "rich"))
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--repeats", type=int, default=20, help="writes timed (default: 20)")
    parser.add_argument(
        "--keep", action="store_true", help="leave the scratch file behind instead of deleting it"
    )
    parser.add_argument(
        "--allow-ram-backed",
        action="store_true",
        help="proceed even when --path resolves to a RAM-backed filesystem (tmpfs/ramfs) "
        "instead of refusing (module docstring, Medium); use only to deliberately record RAM "
        "numbers for a labelled comparison (PROJECT.md section 8 item 148) -- the report's "
        "medium.ram_backed says which kind of run this was either way",
    )
    args = parser.parse_args(argv)

    if args.warm_ticks <= 0:
        parser.error("--warm-ticks must be > 0")
    if args.repeats <= 0:
        parser.error("--repeats must be > 0")

    config_path = args.config or str(REPO_ROOT / "config.example-das.yaml")
    cfg = load_config(config_path).mpc
    if not cfg.regulates_drive_limits:
        parser.error("needs a zoned config without setpoints (mpc.topology)")

    if args.path is not None:
        path = ensure_scratch_path(Path(args.path))
        created_default = False
    else:
        fd, name = tempfile.mkstemp(prefix="aqua-bridge-bench-model-store-", suffix=".json")
        os.close(fd)
        path = ensure_scratch_path(Path(name))
        created_default = True

    try:
        medium = report_and_check_medium(path, allow_ram_backed=args.allow_ram_backed)
    except SystemExit:
        if created_default:
            path.unlink(missing_ok=True)
        raise

    try:
        result = bench_writes(
            cfg,
            path,
            warm_ticks=args.warm_ticks,
            seed=args.seed,
            preset=args.sim_preset,
            repeats=args.repeats,
        )
    finally:
        if not args.keep:
            path.unlink(missing_ok=True)
            tmp_glob = list(path.parent.glob(f".{path.name}.*.tmp"))
            for leftover in tmp_glob:  # write_atomic cleans its own on failure; belt and braces
                leftover.unlink(missing_ok=True)

    report = {
        "python": platform.python_version(),
        "machine": platform.machine(),
        "config_path": str(config_path),
        "path": str(path),
        "medium": medium.asdict(),
        "kept": args.keep,
        "result": result,
    }
    json.dump(report, sys.stdout, indent=2, sort_keys=True)
    sys.stdout.write("\n")
    return 0


if __name__ == "__main__":
    sys.exit(main())
