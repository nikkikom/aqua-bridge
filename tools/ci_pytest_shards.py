#!/usr/bin/env python3
"""Run pytest on disjoint, deterministic shards of ``tests/test_*.py`` concurrently.

Used by the CI ``test`` job (PROJECT.md section 12) to fit the PR run inside its
time budget without a pytest plugin (no ``pytest-xdist``): each shard is a plain
``pytest <forwarded args> <files>`` subprocess, on its own slice of test files, run
in parallel with the others. Exits non-zero if any shard fails; prints each
shard's file list up front and its wall time at the end, so a failure is
reproducible with ``pytest <same files> <same forwarded args>``.

Sharding is a deterministic greedy longest-processing-time-first bin pack over
``tests/test_*.py``, sorted by name and weighted by each file's line count plus a
flat bonus per ``@given`` (Hypothesis runs many examples through a closed-loop sim
per property, so a short property-test file can cost far more than its line count
suggests; a plain line count badly under-weighted ``test_mpc_fuzzy.py`` in
practice). Both are cheap stand-ins for actual running time -- no timing history is
kept or needed, so the split never goes stale as tests are added. Same files, same
weights -> the same assignment every run, on every machine.

Every shard subprocess inherits the parent's environment unchanged (in particular
``HYPOTHESIS_PROFILE``), and each gets its own OS-assigned temp directories and
ports through the suites' existing fixtures, so shards need no isolation beyond
running as separate processes.

Usage::

    python tools/ci_pytest_shards.py --shards 4 -- -m "not hardware and not nightly" --durations=15
"""

from __future__ import annotations

import argparse
import os
import re
import subprocess
import sys
import threading
import time
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
TESTS_DIR = REPO_ROOT / "tests"

# Flat weight bonus per "@given" in a file, added to its line count. Calibrated
# against one measured run (tests/test_mpc_fuzzy.py: 318 lines, 13 @given, ~60s of
# a 78s shard that otherwise held only small, sub-15s files) rather than derived
# analytically; revisit if a shard is consistently the tail.
GIVEN_WEIGHT = 400
GIVEN_RE = re.compile(r"^\s*@given\b", re.MULTILINE)


def discover_test_files() -> list[Path]:
    """Every ``tests/test_*.py`` file, sorted for a deterministic weight order."""
    files = sorted(TESTS_DIR.glob("test_*.py"))
    if not files:
        raise SystemExit(f"no test_*.py files found under {TESTS_DIR}")
    return files


def shard_files(files: list[Path], shards: int) -> list[list[Path]]:
    """Greedy longest-processing-time-first bin pack, weighted by line count plus
    a per-``@given`` bonus (see :data:`GIVEN_WEIGHT`).

    Deterministic: ties in weight and in running bucket total break on file name,
    so the same file set always yields the same assignment. Never more buckets
    than files (an oversized ``--shards`` just leaves some idle).
    """

    def weight(f: Path) -> int:
        text = f.read_text(encoding="utf-8")
        return text.count("\n") + GIVEN_WEIGHT * len(GIVEN_RE.findall(text))

    weighted = sorted(
        ((weight(f), f.name, f) for f in files),
        key=lambda row: (-row[0], row[1]),
    )
    n = max(1, min(shards, len(files)))
    buckets: list[list[Path]] = [[] for _ in range(n)]
    totals = [0] * n
    for w, _name, f in weighted:
        i = min(range(n), key=lambda i: (totals[i], i))
        buckets[i].append(f)
        totals[i] += w
    return buckets


def run_shard(index: int, files: list[Path], pytest_args: list[str]) -> tuple[int, float]:
    cmd = [sys.executable, "-m", "pytest", *pytest_args, *(str(f) for f in files)]
    prefix = f"[shard {index}]"
    print(f"{prefix} {len(files)} files: {' '.join(f.name for f in files)}", flush=True)
    start = time.monotonic()
    proc = subprocess.Popen(
        cmd,
        cwd=REPO_ROOT,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        bufsize=1,
    )
    assert proc.stdout is not None
    for line in proc.stdout:
        print(f"{prefix} {line}", end="", flush=True)
    rc = proc.wait()
    return rc, time.monotonic() - start


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--shards", type=int, default=None, help="shard count (default: CPU count on the runner)"
    )
    parser.add_argument(
        "pytest_args",
        nargs=argparse.REMAINDER,
        help="forwarded to every pytest shard; put after a literal --",
    )
    args = parser.parse_args(argv)
    pytest_args = args.pytest_args
    if pytest_args and pytest_args[0] == "--":
        pytest_args = pytest_args[1:]

    shards = max(1, args.shards or os.cpu_count() or 4)

    files = discover_test_files()
    buckets = shard_files(files, shards)

    results: list[tuple[int, float] | None] = [None] * len(buckets)

    def worker(i: int, shard: list[Path]) -> None:
        results[i] = run_shard(i, shard, pytest_args)

    threads = [threading.Thread(target=worker, args=(i, b)) for i, b in enumerate(buckets)]
    for t in threads:
        t.start()
    for t in threads:
        t.join()

    print("\n--- shard durations ---", flush=True)
    failed: list[int] = []
    for i, result in enumerate(results):
        assert result is not None
        rc, dt = result
        status = "ok" if rc == 0 else f"FAILED (rc={rc})"
        print(f"shard {i}: {len(buckets[i])} files, {dt:.1f}s, {status}")
        if rc != 0:
            failed.append(i)

    if failed:
        print(f"\n{len(failed)} shard(s) failed: {failed}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
