"""Identification experiments against the DAS truth simulator (section 8 items 53, 54, 72).

The pure machine is covered by ``tests/test_ident_experiment.py``; this asks the
physical question the plan asks of it: **does an experiment ever raise a drive above
its limit?** The enclosure runs on ``config.example-das.yaml`` and the PI-like DAS
solver (the form experiments run under in practice, section 5), settles, then a start
on the first output (``config.example-das.yaml`` declares no fan groups, so every
channel is a group of its own) steps it between ``u_base`` and
``u_base + ident_amplitude`` for the whole ``ident_max_duration_s``.

Reported per run: whether the start was accepted, how the experiment ended, the worst
true margin to a limit with the experiment and on the same seed without it, and the
extra PWM the excitation put on the fans. ``ident_levels: above`` never commands less
than the solver's level at start, so the experiment can only cool harder than the
reference; the margin is the number that has to stay positive.

A settled enclosure never comes near the envelope, so those runs alone would not
exercise the abort rules at all (they keep 8-11 degC of true margin from end to end).
The second half of this file therefore *makes* the envelope bind: the same enclosure in
a room warming at ``INLET_DRIFT_C_PER_H``, on ``ident_levels: above`` and on
``symmetric`` (the only mode that can command less cooling than the solver asked for).
Those runs assert what section 5 claims: the **soft envelope** is the rule that fires --
it binds before the absolute abort under the shipped ``config.example-das.yaml`` -- the
drive's ``T_hat + k sigma`` is still below ``limit - ident_abort_below_limit_c`` when it
does, and no drive goes over its limit, from the abort on included.
"""

from __future__ import annotations

import dataclasses
from typing import Any

import pytest

from aqua_bridge.config import load_config
from aqua_bridge.control.intents import Ident, IntentConflict
from aqua_bridge.control.mpc import step
from aqua_bridge.control.supervisor import Supervisor
from aqua_bridge.model import Mode, MpcConfig, MpcState, SolverKind
from aqua_bridge.sim.das import (
    SENSOR_TYPES,
    DasRun,
    build_das_plant,
    run_das_closed_loop,
    topology_from_config,
)
from conftest import EXAMPLE_DAS_CONFIG

#: Warm-up before the start: past ``ident_settle_s`` and the bays' ``bay_settle_s``.
WARMUP_S = 900.0
DURATION_S = 900.0
#: Per-bay activity: warm enough that the solver regulates, light enough that it is
#: not saturated (an experiment on a saturated channel is refused, and rightly).
ACTIVITY = 0.35
#: The room the enclosure settles in, degC.
INLET_BASE_C = 25.0
#: The envelope runs warm the room at this rate instead (degC per hour: a closed rack
#: door, a stopped air conditioner). A *ramp*, not a step: the estimator follows it, so
#: the run reaches the envelope with an honest estimate and the uncalibrated sigma floor
#: (a 10 degC step instead shocks the filter, sigma blows up to 9 degC and the abort
#: says more about the shock than about the rule). 20 degC/h brings the hottest bay onto
#: the envelope about 12 minutes into the experiment, with the fans still off their
#: stops.
INLET_DRIFT_C_PER_H = 20.0


def sim_cfg(**changes: Any) -> MpcConfig:
    """The DAS example with experiments enabled and the settle rules shortened."""
    base = load_config(EXAMPLE_DAS_CONFIG).mpc
    estimator = dataclasses.replace(base.estimator, bay_settle_s=60.0)
    return dataclasses.replace(
        base,
        solver=SolverKind.PI,
        estimator=estimator,
        ident_enabled=True,
        ident_settle_s=max(base.confirm_s, 300.0),
        ident_max_duration_s=DURATION_S,
        **changes,
    )


def plant_for(cfg: MpcConfig, *, preset: str, seed: int, inlet_drift_c_per_h: float = 0.0):
    """The truth plant. ``inlet_drift_c_per_h`` warms the room steadily, the way the
    envelope runs below drive the drives up."""
    topology = topology_from_config(cfg)
    for entry in topology["sensors"].values():
        entry["noise_sigma_c"] = SENSOR_TYPES[entry["type"]].noise_sigma_c
    topology["inlet"] = {"base_c": INLET_BASE_C}
    if inlet_drift_c_per_h:  # left out otherwise: the rich preset draws its own drift
        topology["inlet"]["drift_c_per_h"] = float(inlet_drift_c_per_h)
    schedule = {bay: [(0.0, ACTIVITY)] for bay in topology["bays"]}
    return build_das_plant(
        topology, preset=preset, dt=cfg.dt, seed=seed, initial_pwm=0.5, heat_schedule=schedule
    )


@dataclasses.dataclass
class ExperimentRun:
    run: DasRun
    reasons: list[str]
    status: dict[str, Any]
    #: ``ts`` of the tick the experiment started on, of the tick it ended on, and the
    #: estimates of that last tick (what the abort was decided from).
    start_ts: float | None = None
    end_ts: float | None = None
    end_estimates: dict[str, Any] = dataclasses.field(default_factory=dict)
    #: Commanded PWM range on the experiment channel while it ran.
    target_pwm: tuple[float, float] | None = None

    @property
    def worst_margin_c(self) -> float:
        return self.run.worst_margin_c()

    @property
    def abort_reason(self) -> str:
        return str(self.status.get("last_abort_reason") or "")

    def violations_after(self, ts0: float) -> int:
        """True limit violations from ``ts0`` on (``margin_c < 0`` on any bay)."""
        ts = self.run.series["ts"]
        return sum(
            1
            for seq in self.run.series["margin_c"].values()
            for t, v in zip(ts, seq, strict=True)
            if t >= ts0 and v is not None and v < 0.0
        )

    def worst_margin_after_c(self, ts0: float) -> float:
        """The worst true margin from ``ts0`` on: the window the experiment runs in."""
        ts = self.run.series["ts"]
        return min(
            v
            for seq in self.run.series["margin_c"].values()
            for t, v in zip(ts, seq, strict=True)
            if t >= ts0 and v is not None
        )

    @property
    def max_pwm(self) -> float:
        return max(v for seq in self.run.series["pwm_cmd"].values() for v in seq if v is not None)


def drive(
    cfg: MpcConfig, *, preset: str, seed: int, start: bool, inlet_drift_c_per_h: float = 0.0
) -> ExperimentRun:
    """Run the supervisor + solver against the truth plant, starting an experiment on
    the first output after ``WARMUP_S`` when ``start``."""
    plant = plant_for(cfg, preset=preset, seed=seed, inlet_drift_c_per_h=inlet_drift_c_per_h)
    sup = Supervisor(cfg)
    target = cfg.channels[0]  # the DAS example declares no fan groups: one channel
    prev: dict[str, float] = dict.fromkeys(cfg.channels, 0.5)
    reasons: list[str] = []
    state: dict[str, Any] = {"status": {}, "ran": False}
    span: dict[str, Any] = {"start_ts": None, "end_ts": None, "estimates": {}, "pwm": []}

    def controller(obs, _cfg, st: MpcState):
        nonlocal prev
        if start and obs.ts >= WARMUP_S and sup.experiment is None and not state["ran"]:
            try:  # retried every tick: the enclosure may still be settling
                sup.submit(Ident("start", channel=target))
                state["ran"] = True
                span["start_ts"] = obs.ts
                reasons.clear()
            except IntentConflict as exc:
                reasons[:] = [str(exc)]
        plan = sup.plan_tick()
        cmd, st2 = step(obs, plan.cfg, st)
        out = sup.compose(cmd, plan, prev)
        running = sup.experiment is not None
        sup.record_tick(
            obs=obs, mpc_cmd=cmd, cmd=out, state=st2, applied=True, usb_present=True, ts=obs.ts
        )
        if running:
            # The tick the experiment ended on (completed or aborted) is the last one
            # it was running on, and its estimates are what the decision was made from.
            span["pwm"].append(float(out.pwm[target]))
            span["end_ts"] = obs.ts
            if sup.experiment is None:
                span["estimates"] = dict(cmd.diagnostics.get("estimates", {}))
        prev = dict(out.pwm)
        if sup.experiment is not None or state["status"].get("running"):
            state["status"] = sup.snapshot().extra["experiment"]
        return out, st2

    ticks = int((WARMUP_S + 2 * DURATION_S) / cfg.dt)
    run = run_das_closed_loop(plant, cfg, controller, ticks)
    return ExperimentRun(
        run,
        reasons,
        state["status"],
        start_ts=span["start_ts"],
        end_ts=span["end_ts"],
        end_estimates=span["estimates"],
        target_pwm=(min(span["pwm"]), max(span["pwm"])) if span["pwm"] else None,
    )


def describe(tag: str, r: ExperimentRun) -> str:
    return (
        f"{tag}: worst true margin {r.worst_margin_c:.2f} degC overall, "
        f"{r.worst_margin_after_c(WARMUP_S):.2f} degC from the start on, violations "
        f"{r.run.violations()}, max PWM {r.max_pwm:.3f}, "
        f"result {r.status.get('last_result')} {r.status.get('last_abort_reason') or ''}"
    )


def _assert_ran(r: ExperimentRun) -> None:
    assert not r.reasons, f"the start was refused: {r.reasons}"
    assert r.status.get("last_result") == "completed", describe("experiment", r)
    modes = {rec.cmd.mode for rec in r.run.records}
    assert Mode.FALLBACK not in modes and Mode.DEGRADED not in modes, modes


def test_an_experiment_never_takes_a_drive_over_its_limit():
    cfg = sim_cfg()
    ran = drive(cfg, preset="basic", seed=1, start=True)
    quiet = drive(cfg, preset="basic", seed=1, start=False)
    print(describe("basic with experiment   ", ran))
    print(describe("basic without experiment", quiet))
    _assert_ran(ran)
    assert ran.run.violations() == 0, describe("with experiment", ran)
    assert quiet.run.violations() == 0, describe("without experiment", quiet)
    # ident_levels: above only ever adds cooling, so the run with the experiment is
    # never the hotter of the two.
    assert ran.worst_margin_after_c(WARMUP_S) >= quiet.worst_margin_after_c(WARMUP_S) - 0.5, (
        describe("with experiment", ran),
        describe("without experiment", quiet),
    )
    assert ran.max_pwm > quiet.max_pwm, "the excitation never reached the fans"


@pytest.mark.nightly
@pytest.mark.parametrize("seed", range(1, 6))
@pytest.mark.parametrize("preset", ["basic", "rich"])
def test_experiment_sweep_never_takes_a_drive_over_its_limit(preset, seed):
    cfg = sim_cfg()
    ran = drive(cfg, preset=preset, seed=seed, start=True)
    quiet = drive(cfg, preset=preset, seed=seed, start=False)
    print(describe(f"{preset} seed {seed} with experiment   ", ran))
    print(describe(f"{preset} seed {seed} without experiment", quiet))
    assert ran.run.violations() == 0, describe("with experiment", ran)
    assert quiet.run.violations() == 0, describe("without experiment", quiet)
    if ran.reasons or ran.status.get("last_result") != "completed":
        # a drawn enclosure the preconditions rightly refuse, or an abort: the run is
        # still evidence that nothing went over a limit
        pytest.skip(f"no full run on this seed ({ran.reasons or ran.status})")
    assert ran.worst_margin_after_c(WARMUP_S) >= quiet.worst_margin_after_c(WARMUP_S) - 0.5, (
        describe("with experiment", ran),
        describe("without experiment", quiet),
    )


# ---------------------------------------------------------------------------
# The abort rules, with the envelope actually made to bind
# ---------------------------------------------------------------------------


def _assert_envelope_abort(r: ExperimentRun, cfg: MpcConfig) -> None:
    """The run ended on the soft envelope, before the absolute abort, and no drive went
    over its limit -- from the abort on included (the check section 5 and section 8
    item 53 cite)."""
    assert not r.reasons, f"the start was refused: {r.reasons}"
    assert r.status.get("last_result") == "aborted", describe("envelope run", r)
    reason = r.abort_reason
    assert reason.startswith("envelope:"), f"expected a soft-envelope abort, got {reason!r}"
    bay = reason.split(":", 1)[1]

    est = r.end_estimates.get(bay)
    assert est is not None, f"no estimate for {bay} on the abort tick"
    t_hat, margin = float(est["t_c"]), float(est["margin_c"])
    over = t_hat - (float(est["soft_c"]) + cfg.ident_max_over_c)
    absolute = float(est["limit_c"]) - cfg.ident_abort_below_limit_c
    print(
        f"abort on {bay} at ts {r.end_ts}: T_hat {t_hat:.2f} degC is {over:+.2f} past "
        f"soft + ident_max_over_c, T_hat + k sigma {t_hat + margin:.2f} against the "
        f"absolute abort at {absolute:.2f} degC (limit {float(est['limit_c']):.1f})"
    )
    # The rule fired on the tick the estimate crossed it, not somewhere well past it:
    # a looser envelope would show up here as a larger overshoot (or as no abort).
    assert 0.0 < over < 1.0, f"{bay} did not abort on the soft envelope: {est}"
    # Section 5: under the shipped config the soft envelope binds first, so the
    # absolute backstop is still untouched when the run is stopped.
    assert t_hat + margin < absolute, (
        f"{bay} reached the absolute abort ({t_hat + margin:.2f} >= {absolute:.2f}); "
        "the soft envelope is supposed to bind first under config.example-das.yaml"
    )

    assert r.run.violations() == 0, describe("envelope run", r)
    assert r.end_ts is not None
    assert r.violations_after(r.end_ts) == 0, describe("envelope run", r)


@pytest.mark.parametrize("levels", ["above", "symmetric"])
def test_an_experiment_driven_onto_the_envelope_aborts_before_any_limit(levels):
    """A rising room pushes a running experiment onto the soft envelope: it aborts there
    and no drive ever passes its limit, on both ``ident_levels``. ``symmetric`` is the
    only mode that can command less cooling than the solver asked for, so it is the one
    that could in principle make a drive hotter."""
    cfg = sim_cfg(ident_levels=levels)
    r = drive(cfg, preset="basic", seed=1, start=True, inlet_drift_c_per_h=INLET_DRIFT_C_PER_H)
    print(describe(f"basic seed 1, inlet +{INLET_DRIFT_C_PER_H:g} degC/h, {levels}", r))
    _assert_envelope_abort(r, cfg)
    assert r.target_pwm is not None
    lo, hi = r.target_pwm
    if levels == "symmetric":
        # the low level really was commanded before the abort, so the run covers the
        # one mode in which an experiment can reduce cooling
        assert lo < hi - cfg.ident_amplitude + 1e-3, f"the low level never ran: {r.target_pwm}"


@pytest.mark.nightly
@pytest.mark.parametrize("seed", range(1, 6))
@pytest.mark.parametrize("levels", ["above", "symmetric"])
def test_envelope_abort_sweep(levels, seed):
    cfg = sim_cfg(ident_levels=levels)
    r = drive(cfg, preset="basic", seed=seed, start=True, inlet_drift_c_per_h=INLET_DRIFT_C_PER_H)
    print(describe(f"basic seed {seed}, inlet +{INLET_DRIFT_C_PER_H:g} degC/h, {levels}", r))
    _assert_envelope_abort(r, cfg)
