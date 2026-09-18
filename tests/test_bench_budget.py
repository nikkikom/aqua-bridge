"""Step budget of the DAS MPC (plan sections 4 and 9, ``tests/test_bench_budget.py``).

Two gates on ``mpc.step`` in a closed loop, timed like ``tools/bench_step.py``:

* **relative, every CI run**: the p99 step time of the DAS MPC on
  ``config.example-das.yaml`` (15 bays, 25 sensors, 8 channels, the estimator every
  tick, the MPC every ``mpc_every_ticks``) against the DAS truth plant, divided by
  the p99 of the legacy MPC on ``config.example.yaml`` against the RC plant,
  measured in the same process. A ratio is what a CI runner can check; it tracks the
  Zero W's absolute numbers only as far as the plan's 100x factor holds for both.

  Method (item 6, robust on a slow shared runner): :data:`REPEATS` repeats, each
  timing both sides back to back with the garbage collector paused; the two sides
  alternate which one goes first every repeat (``legacy, das`` on even repeats,
  ``das, legacy`` on odd ones) so a runner hiccup during one repeat does not
  systematically favour either side. The first :data:`WARMUP_REPEATS` repeats
  (import / cache / branch-predictor warm-up) are discarded, and the gate compares
  the :data:`RELATIVE_PERCENTILE` percentile of the remaining per-repeat ratios
  (das p99 / legacy p99) against the named constant :data:`RELATIVE_FACTOR` -- a
  high percentile rather than the single worst repeat, so one outlier repeat does
  not flake the gate while a real regression across most repeats still trips it.
  Timed with **CPU time** (:func:`time.process_time`), not wall clock (§8 item 126):
  the two-sided alternation above only cancels a bias that favours one side every
  repeat, not a runner that is merely busy elsewhere while one of the two ``step()``
  calls happens to run, which wall clock cannot tell from the solver actually taking
  longer. CPU time is unaffected by a process being descheduled, so it stays a
  property of the two solvers under sustained contention as well as on an idle
  machine (see the test's own docstring for the measurement).

* **absolute, only on the Pi** (marker ``pi``): the DAS MPC's p99 step time is at
  most ``mpc.budget_ms`` of ``config.example-das.yaml`` (the per-tick gate at
  ``dt = 5 s``; §8.1 owner decision 2026-09-14, was 500 ms; re-derived to 250 ms on
  the Zero 2 W, §8 item 73, 2026-09-17). Run it there with
  ``pytest tests/test_bench_budget.py -m pi``.

  "On the Pi" is ``platform.system() == "Linux"`` and ``platform.machine()`` one of
  :data:`PI_MACHINES` -- every board this project has actually measured the gate on
  (the Zero W's ``armv6l``, the Zero 2 W's ``aarch64``; §8 item 108). A board change
  that gets a new ``platform.machine()`` string needs a one-line addition here, same
  as the last one did; §8 item 108 chose this over a provisioning-set environment
  variable because an architecture check needs nothing from the owner to work --
  ``pytest -m pi`` on the actual board just runs -- while an env var the Pi's own run
  sets is one more thing to remember to export by hand each session, i.e. exactly the
  kind of silent, easy-to-forget precondition item 108 is about. Either mechanism can
  go stale when the board changes; the difference is what happens next. A stale env
  var stays unset and skips silently, same as today's bug. A stale architecture list
  also skips -- but :data:`PI_SKIP_REASON` names the actual ``platform.system()`` /
  ``platform.machine()`` seen and what :data:`PI_MACHINES` expects, so the terminal
  output of the very next ``pytest -m pi`` run says exactly why, and a silent pass is
  not possible.

The plan's second relative line (the gate with 30 sensors within 3x the solver-free step)
is not a separate test here: the gate is part of both steps measured above, a gate alone
is always cheaper than a step that contains it, and ``tests/test_gate.py`` checks that the
decimated Stuck windows keep their storage bounded. ``tools/bench_step.py --sim-plant das``
prints the same numbers for both DAS solvers.
"""

from __future__ import annotations

import dataclasses
import gc
import importlib.util
import json
import platform
import re
import statistics
import time
from collections.abc import Callable

import pytest

from aqua_bridge.config import load_config
from aqua_bridge.control.mpc import step
from aqua_bridge.model import ConfigError, MpcConfig, MpcState, SolverKind
from aqua_bridge.sim.das import SENSOR_TYPES, build_das_plant, topology_from_config
from aqua_bridge.sim.plant import Plant, PlantParams
from conftest import EXAMPLE_CONFIG, EXAMPLE_DAS_CONFIG, REPO_ROOT

#: Plan section 9: DAS step p99 <= 12x the legacy MPC step p99.
#: Test parameter, not an operator setting (PROJECT.md "No hardcoded tunables"
#: applies to config the controller reads, not to a CI gate's own threshold).
RELATIVE_FACTOR = 12.0
#: Repeats of the relative comparison, and how many leading ones to discard as warm-up.
REPEATS = 5
WARMUP_REPEATS = 1
#: Percentile of the (REPEATS - WARMUP_REPEATS) per-repeat ratios the gate checks.
RELATIVE_PERCENTILE = 75.0
TICKS = 240
WARMUP = 20
#: Ticks per closed loop in the Zero W fallback tests (shorter: they run several loops).
FALLBACK_TICKS = 140
#: PROJECT.md section 8 item 73, the owner's fallback of 2026-09-14 for the Zero W: what
#: goes into ``mpc:`` when the DAS MPC misses the budget there. Not a default -- the
#: shipped configs keep 250 / 350 ms (re-derived on the Zero 2 W, item 73, 2026-09-17)
#: and ``mpc_every_ticks: 2``, and
#: :func:`test_the_documented_zero_w_fallback_loads_from_the_example_config` reads these
#: same numbers back out of ``config.example-das.yaml`` and applies them to the file, so
#: the literals here and the shipped documentation cannot drift apart.
ZERO_W_FALLBACK = {"budget_ms": 1000.0, "budget_alarm_ms": 1250.0, "mpc_every_ticks": 3}
#: How ``config.example-das.yaml`` marks the value to write for a fallback key.
FALLBACK_COMMENT = re.compile(r"item 73 fallback: ([0-9]+(?:\.[0-9]+)?)")

#: §8 item 108: ``platform.machine()`` of every board this project has actually run
#: the absolute ``mpc.budget_ms`` gate on -- not an exhaustive list of Raspberry Pi
#: architectures, just the ones measured (PROJECT.md §8 items 73, 95): ``armv6l`` is
#: the Zero W (32-bit, the original board the budget was set for), ``aarch64`` is the
#: Zero 2 W (64-bit trixie, the owner's current board, items 50/51). Add the new
#: string here the next time the board changes; forgetting it is caught, not silent
#: -- see :data:`PI_SKIP_REASON`.
PI_MACHINES = frozenset({"armv6l", "aarch64"})
_PI_SYSTEM = platform.system()
_PI_MACHINE = platform.machine()
#: Whether this process is running on a board named in :data:`PI_MACHINES`.
ON_PI = _PI_SYSTEM == "Linux" and _PI_MACHINE in PI_MACHINES
#: Names the exact check that failed and what it saw, so ``pytest -m pi`` on a board
#: this file does not recognise reports *why* the absolute gate did not run instead of
#: reporting nothing (§8 item 108 -- a hardcoded ``armv6l`` check skipped silently on
#: the owner's own Zero 2 W).
PI_SKIP_REASON = (
    f"absolute budget: Raspberry Pi only -- platform.system() == {_PI_SYSTEM!r} "
    f"(want 'Linux') and platform.machine() == {_PI_MACHINE!r} "
    f"(want one of {sorted(PI_MACHINES)}); add this board to PI_MACHINES if it is one"
)
#: A ``key: value`` line of a YAML mapping (``value`` without its trailing comment).
YAML_KEY = re.compile(r"^(?P<head>\s*(?P<key>[A-Za-z_][A-Za-z0-9_]*):\s*)(?P<value>[^#\s]\S*)")


def documented_fallback(text: str) -> dict[str, tuple[int, str]]:
    """``{key: (line index of its live value, the fallback value documented for it)}``.

    Reads ``config.example-das.yaml`` the way an operator does: each of the three keys
    carries its item 73 fallback in its own comment block -- the tail of its ``key:``
    line and the comment-only lines that follow it, up to the next key.
    """
    found: dict[str, tuple[int, str]] = {}
    key: str | None = None
    for i, line in enumerate(text.splitlines()):
        match = YAML_KEY.match(line)
        if match:
            key = match["key"] if match["key"] in ZERO_W_FALLBACK else None
            if key is not None:
                assert key not in found, f"{key} appears twice in the example config"
                found[key] = (i, "")
            comment = line.partition("#")[2]
        elif line.lstrip().startswith("#"):
            comment = line.lstrip()[1:]
        else:
            key = None
            continue
        hit = FALLBACK_COMMENT.search(comment)
        if key is not None and hit is not None:
            index, seen = found[key]
            assert not seen, f"{key} documents two fallback values"
            found[key] = (index, hit[1])
    return found


def p99(samples: list[float]) -> float:
    ordered = sorted(samples)
    return ordered[min(len(ordered) - 1, round(0.99 * (len(ordered) - 1)))]


def legacy_mpc_times(
    ticks: int = TICKS, *, clock: Callable[[], float] = time.perf_counter
) -> list[float]:
    cfg = dataclasses.replace(load_config(EXAMPLE_CONFIG).mpc, solver=SolverKind.MPC)
    plant = Plant(
        PlantParams(dt=cfg.dt, heat_w=100.0, noise_sigma_c=0.2, delay_ticks=1),
        initial_pwm=0.5,
        t_coolant=40.0,
        t_air=30.0,
        seed=7,
    )
    return _timed(cfg, plant, ticks, clock=clock)


def das_mpc_config() -> MpcConfig:
    return dataclasses.replace(
        load_config(EXAMPLE_DAS_CONFIG).mpc, solver=SolverKind.MPC, model_accept_prior=True
    )


def das_mpc_times(
    cfg: MpcConfig,
    ticks: int = TICKS,
    *,
    solved: list[bool] | None = None,
    work: list[tuple[int, int]] | None = None,
    clock: Callable[[], float] = time.perf_counter,
) -> list[float]:
    topology = topology_from_config(cfg)
    for entry in topology["sensors"].values():
        entry["noise_sigma_c"] = SENSOR_TYPES[entry["type"]].noise_sigma_c
    plant = build_das_plant(
        topology,
        preset="basic",
        dt=cfg.dt,
        initial_pwm=0.5,
        seed=7,
        heat_schedule={"b02": [(0.0, 1.0)], "b10": [(300.0, 1.0)], "b13": [(0.0, 0.5)]},
    )
    return _timed(cfg, plant, ticks, solved=solved, work=work, clock=clock)


def _timed(
    cfg: MpcConfig,
    plant,
    ticks: int,
    *,
    solved: list[bool] | None = None,
    work: list[tuple[int, int]] | None = None,
    clock: Callable[[], float] = time.perf_counter,
) -> list[float]:
    """Step times of ``ticks`` closed-loop ticks; ``solved`` collects, per measured tick,
    whether the solver ran its solve rather than replaying a stored plan, and ``work`` the
    solve work it reports -- ``(outer iterations, box-QP iterations)``, ``(0, 0)`` on a
    replay tick, which is the structural difference behind the timing one.

    ``clock`` defaults to :func:`time.perf_counter` (wall clock), right for the absolute
    Pi gate below, which is a real-time deadline: the actual wall-clock cost of one tick
    is exactly what must fit inside ``dt`` on the board. The relative gate
    (:func:`test_das_mpc_step_p99_within_the_relative_budget`) passes
    :func:`time.process_time` instead -- see that test and PROJECT.md section 8 item 126."""
    state = MpcState.cold()
    times: list[float] = []
    enabled = gc.isenabled()
    gc.disable()
    try:
        for i in range(ticks):
            obs = plant.observe()
            t0 = clock()
            cmd, state = step(obs, cfg, state)
            elapsed = (clock() - t0) * 1e3
            if i >= WARMUP:
                times.append(elapsed)
                diag = cmd.diagnostics.get("solver_diag", {})
                if solved is not None:
                    solved.append(bool(diag.get("solved")))
                if work is not None:
                    work.append((int(diag.get("outer", 0)), int(diag.get("iterations_total", 0))))
            plant.apply(cmd.pwm)
            plant.advance()
    finally:
        if enabled:
            gc.enable()
    return times


def percentile(samples: list[float], q: float) -> float:
    """Nearest-rank percentile (``q`` in [0, 100]); used on the per-repeat ratios."""
    ordered = sorted(samples)
    k = max(0, min(len(ordered) - 1, round(q / 100.0 * (len(ordered) - 1))))
    return ordered[k]


def test_das_mpc_step_p99_within_the_relative_budget():
    """Method: module docstring "relative, every CI run".

    Timed with :func:`time.process_time` (CPU time), not :func:`time.perf_counter` (wall
    clock) -- section 8 item 126. The ratio this gate checks is meant to be the DAS
    solver's cost relative to the legacy one, a property of the two solvers, not of
    whatever else the machine was doing while this process waited for the scheduler.
    Wall clock conflates the two: on a runner shared with other work (this project's own
    contention, an 8-way ``ci_pytest_shards.py`` run, or several branches' suites at
    once, as this repository's CI is run) a preemption that lands inside one timed
    ``step()`` and not the other inflates that repeat's ratio without either solver
    doing more work, and it inflates the DAS side more often simply because its ``step()``
    takes longer wall clock to begin with (more calls it can be preempted inside, and
    every ``ci_pytest_shards.py`` shard is itself CPU-bound, so the two sides are never
    actually idle-waiting at the same rate). CPU time is immune to that: a process
    descheduled while another one runs accrues no CPU time for the gap, wall clock or
    not, so the ratio measures instruction cost, not scheduler luck. Measured on this
    change (module-level ``python`` alone, on a host already running three other
    branches' test suites, no ``taskset`` or quiescing): ``perf_counter`` ratios of
    3.1x-39.3x across five independent single-repeat samples -- both under and over
    :data:`RELATIVE_FACTOR`, on the same code -- against ``process_time`` ratios of
    9.6x-11.7x, in the same range item 126's own prior measurements report (10.4x-10.6x,
    10.57x-10.62x). The absolute Pi gate below keeps wall clock: there, real elapsed time
    against ``dt`` is exactly the question, and the Pi runs this suite alone.

    ``process_time`` sums CPU time across every thread of the process, so it would stop
    being machine-independent if the linear algebra here (``np.linalg.solve``,
    ``eigvalsh``, ``pinv``, ``lstsq``) ever ran multi-threaded BLAS -- CPU time would then
    scale with however many worker threads OpenBLAS spun up, which can itself vary with
    the machine. Checked directly, not assumed: forcing ``OMP_NUM_THREADS`` /
    ``OPENBLAS_NUM_THREADS`` to 1, 16 and unset (this machine has 32 cores) all gave the
    same ~8x-10x ratio on this branch's code -- the matrices this solver ever forms
    (``mpc.channels`` up to a handful, at most a few dozen decision variables) sit well
    under OpenBLAS's own threshold for switching a solve to multiple threads, so thread
    count is not, in practice, a second source of scheduler-dependent noise here.
    """
    cfg = das_mpc_config()
    ratios: list[float] = []
    runs: list[tuple[float, float]] = []  # (das_p99, legacy_p99), every repeat, for the message
    for i in range(REPEATS):
        if i % 2 == 0:
            legacy_ms = p99(legacy_mpc_times(clock=time.process_time))
            das_ms = p99(das_mpc_times(cfg, clock=time.process_time))
        else:
            das_ms = p99(das_mpc_times(cfg, clock=time.process_time))
            legacy_ms = p99(legacy_mpc_times(clock=time.process_time))
        runs.append((das_ms, legacy_ms))
        if i >= WARMUP_REPEATS:
            ratios.append(das_ms / legacy_ms)
    ratio = percentile(ratios, RELATIVE_PERCENTILE)
    assert ratio <= RELATIVE_FACTOR, (
        f"DAS MPC step p99 is {RELATIVE_PERCENTILE:.0f}th percentile {ratio:.1f}x the legacy "
        f"MPC step p99 (budget {RELATIVE_FACTOR}x); per-repeat (das_ms, legacy_ms): {runs}"
    )


@pytest.mark.pi
@pytest.mark.skipif(not ON_PI, reason=PI_SKIP_REASON)
def test_das_mpc_step_p99_within_budget_ms_on_the_pi():
    cfg = das_mpc_config()
    times = das_mpc_times(cfg, ticks=200)
    assert p99(times) <= cfg.budget_ms, f"DAS MPC step p99 {p99(times):.0f} ms > {cfg.budget_ms} ms"


# --- the documented Zero W fallback (PROJECT.md section 8 item 73) -------------------


def test_the_zero_w_fallback_is_a_config_the_model_accepts():
    """``budget_ms: 1000`` alone is rejected: it must stay below ``budget_alarm_ms``."""
    cfg = das_mpc_config()
    raised = dataclasses.replace(cfg, **ZERO_W_FALLBACK)
    assert raised.budget_ms == 1000.0 and raised.budget_alarm_ms == 1250.0
    assert raised.mpc_every_ticks == 3
    with pytest.raises(ConfigError) as exc:  # the stock alarm is 350 ms
        dataclasses.replace(cfg, budget_ms=1000.0)
    assert "mpc.budget_ms" in str(exc.value) and "mpc.budget_alarm_ms" in str(exc.value)


def test_the_documented_zero_w_fallback_loads_from_the_example_config(tmp_path):
    """The shipped file, edited the way item 73 tells the owner to edit it.

    Not the values of :data:`ZERO_W_FALLBACK` handed to ``dataclasses.replace``: this
    reads the numbers out of ``config.example-das.yaml``'s own comments, writes each one
    over the live value of its key, and loads the result -- so a fallback the file
    documents in a form that does not take effect (a second copy of a key, which YAML
    drops silently) fails here instead of on the Pi.
    """
    text = EXAMPLE_DAS_CONFIG.read_text(encoding="utf-8")
    documented = documented_fallback(text)
    assert set(documented) == set(ZERO_W_FALLBACK), "every fallback key documents its value"
    assert {key: float(value) for key, (_, value) in documented.items()} == {
        key: float(value) for key, value in ZERO_W_FALLBACK.items()
    }, "the example config and item 73 name the same numbers"

    lines = text.splitlines(keepends=True)
    for key, (index, value) in documented.items():
        match = YAML_KEY.match(lines[index])
        assert match is not None and match["key"] == key
        lines[index] = match["head"] + value + lines[index][match.end("value") :]
    edited = tmp_path / "config.yaml"
    edited.write_text("".join(lines), encoding="utf-8")

    stock = load_config(EXAMPLE_DAS_CONFIG).mpc  # the fallback is documentation, not a default
    assert (stock.budget_ms, stock.budget_alarm_ms, stock.mpc_every_ticks) == (250.0, 350.0, 2)
    mpc = load_config(edited).mpc
    assert (mpc.budget_ms, mpc.budget_alarm_ms, mpc.mpc_every_ticks) == (1000.0, 1250.0, 3)


@pytest.mark.slow
def test_mpc_every_ticks_replays_the_plan_between_solves_on_the_das_plant():
    """Item 73's second lever: the DAS MPC solves every n-th tick, replays in between.

    Checked on the truth plant rather than on a synthetic request, because the solver
    also re-solves at once when the fixed channels, trusted zones, constrained bays or
    active model change -- the share of solve ticks is what an operator gets, not the
    nominal ``1 / n``.
    """
    base = das_mpc_config()
    shares: dict[int, float] = {}
    for every in (1, 3):
        solved: list[bool] = []
        das_mpc_times(
            dataclasses.replace(base, mpc_every_ticks=every), FALLBACK_TICKS, solved=solved
        )
        shares[every] = sum(solved) / len(solved)
    assert shares[1] == 1.0
    assert shares[3] == pytest.approx(1 / 3, abs=0.05)


@pytest.mark.slow
def test_solving_less_often_lowers_the_mean_and_leaves_the_p99_a_solve_tick():
    """The note in item 73: a solve tick is the expensive one, and with
    ``mpc_every_ticks: 3`` solve ticks are still a third of all ticks -- far above the
    1 % where the p99 over all ticks would stop being one. So the lever buys mean time
    (and CPU), not p99, and the Pi gate on ``mpc.budget_ms`` still has to be met by the
    solve tick itself.

    What separates a solve tick from a replay tick is asserted on the work the solver
    reports, not on the clock: a solve tick runs SQP outer iterations and box QPs, a
    replay tick runs none. The two remaining comparisons are on wall time because the
    claim is about time; both are relative and wide (a solve tick is roughly 2-3x a
    replay tick here), never an absolute millisecond threshold a loaded runner could
    trip.
    """
    base = das_mpc_config()
    solved: list[bool] = []
    work: list[tuple[int, int]] = []
    every3 = das_mpc_times(
        dataclasses.replace(base, mpc_every_ticks=3), FALLBACK_TICKS, solved=solved, work=work
    )
    every1 = das_mpc_times(dataclasses.replace(base, mpc_every_ticks=1), FALLBACK_TICKS)
    solve_ms = [t for t, s in zip(every3, solved, strict=True) if s]
    solve_work = [w for w, s in zip(work, solved, strict=True) if s]
    replay_work = [w for w, s in zip(work, solved, strict=True) if not s]
    assert solve_work and replay_work
    assert all(outer >= 1 and qp_iters >= 1 for outer, qp_iters in solve_work)
    assert all(counts == (0, 0) for counts in replay_work)  # the replay tick solves nothing
    assert statistics.median(every3) < statistics.median(every1)
    assert len(solve_ms) / len(every3) > 0.01  # the p99 over all ticks is a solve tick
    assert p99(every3) >= statistics.median(solve_ms)


def test_bench_tool_runs_both_das_solvers_on_the_das_plant(capsys):
    cfg = das_mpc_config()
    spec = importlib.util.spec_from_file_location(
        "bench_step", REPO_ROOT / "tools" / "bench_step.py"
    )
    assert spec is not None and spec.loader is not None
    tool = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(tool)
    assert tool.main(["--sim-plant", "das", "--ticks", "12"]) == 0
    report = json.loads(capsys.readouterr().out)
    assert report["sim_plant"] == "das" and set(report["results"]) == {"pi", "mpc"}
    mpc = report["results"]["mpc"]
    assert mpc["model_active_fraction"] == 1.0 and mpc["solve_ticks"] >= 6
    assert mpc["budget_ms"] == cfg.budget_ms and report["results"]["pi"]["solve_ticks"] == 0
    assert mpc["budget_alarm_ms"] == cfg.budget_alarm_ms


def test_bench_tool_reports_the_sim_preset_it_actually_ran(capsys):
    """Item 28: ``plant.preset`` names the preset ``--sim-preset`` selected, not a
    literal ``"basic"`` regardless of it."""
    spec = importlib.util.spec_from_file_location(
        "bench_step", REPO_ROOT / "tools" / "bench_step.py"
    )
    assert spec is not None and spec.loader is not None
    tool = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(tool)

    assert tool.main(["--sim-plant", "das", "--ticks", "6"]) == 0
    assert json.loads(capsys.readouterr().out)["plant"]["preset"] == "basic"

    assert tool.main(["--sim-plant", "das", "--ticks", "6", "--sim-preset", "rich"]) == 0
    assert json.loads(capsys.readouterr().out)["plant"]["preset"] == "rich"

    with pytest.raises(SystemExit):
        tool.main(["--sim-plant", "das", "--ticks", "6", "--sim-preset", "bogus"])
    assert "--sim-preset must be one of" in capsys.readouterr().err

    # basic: --sim-preset is unused and never validated
    assert tool.main(["--sim-plant", "basic", "--ticks", "6", "--sim-preset", "bogus"]) == 0
    assert "preset" not in json.loads(capsys.readouterr().out)["plant"]


def _load_bench_tool():
    spec = importlib.util.spec_from_file_location(
        "bench_step", REPO_ROOT / "tools" / "bench_step.py"
    )
    assert spec is not None and spec.loader is not None
    tool = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(tool)
    return tool


def test_bench_tool_profile_phases_add_up_to_the_solve_tick(capsys):
    """Item 95: ``--profile-phases`` breaks a DAS MPC solve tick into the SQP and
    its box QPs, the estimator's Kalman update, the sensor gate and bookkeeping
    (PROJECT.md section 4 "Budget at `dt = 5 s`", coarsened to what the item asks
    the bench tool to add), and the four add up to that tick's own step time."""
    tool = _load_bench_tool()
    assert tool.main(["--sim-plant", "das", "--ticks", "40", "--profile-phases"]) == 0
    report = json.loads(capsys.readouterr().out)
    mpc = report["results"]["mpc"]
    phases = mpc["phases"]
    assert set(phases) == {"gate_ms", "estimator_kalman_ms", "sqp_box_qp_ms", "bookkeeping_ms"}
    for stats in phases.values():
        assert stats["n"] == mpc["solve_ticks"] > 0
        assert stats["mean_ms"] >= 0.0
        assert stats["p99_ms"] >= 0.0
        assert stats["max_ms"] >= stats["mean_ms"] - 1e-9

    # every named phase actually ran (a phase stuck at 0 would mean the wrapper
    # never reached the function it claims to time, not that the phase is free)
    assert phases["sqp_box_qp_ms"]["mean_ms"] > 0.0
    assert phases["estimator_kalman_ms"]["mean_ms"] > 0.0

    # bookkeeping_ms is defined per tick as elapsed - (the three named phases),
    # so their means sum to (at most, floating point) the mean elapsed time over
    # solve ticks; that mean is itself at most the run's own max_ms
    named = ("gate_ms", "estimator_kalman_ms", "sqp_box_qp_ms")
    named_mean_sum = sum(phases[p]["mean_ms"] for p in named)
    assert named_mean_sum <= mpc["max_ms"]
    assert named_mean_sum + phases["bookkeeping_ms"]["mean_ms"] <= mpc["max_ms"] + 1e-6

    # pi never solves an SQP: no phases reported for it
    assert "phases" not in report["results"]["pi"]

    # step() is untouched: the wrapper is undone, so a plain run right after
    # gives the same shape of report as always
    assert tool.main(["--sim-plant", "das", "--ticks", "6"]) == 0
    plain = json.loads(capsys.readouterr().out)
    assert "phases" not in plain["results"]["mpc"]


def test_bench_tool_profile_phases_needs_das(capsys):
    tool = _load_bench_tool()
    with pytest.raises(SystemExit):
        tool.main(["--profile-phases", "--ticks", "6"])
    assert "--profile-phases needs --sim-plant das" in capsys.readouterr().err
