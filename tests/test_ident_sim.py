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


def plant_for(cfg: MpcConfig, *, preset: str, seed: int):
    topology = topology_from_config(cfg)
    for entry in topology["sensors"].values():
        entry["noise_sigma_c"] = SENSOR_TYPES[entry["type"]].noise_sigma_c
    topology["inlet"] = {"base_c": 25.0}
    schedule = {bay: [(0.0, ACTIVITY)] for bay in topology["bays"]}
    return build_das_plant(
        topology, preset=preset, dt=cfg.dt, seed=seed, initial_pwm=0.5, heat_schedule=schedule
    )


@dataclasses.dataclass
class ExperimentRun:
    run: DasRun
    reasons: list[str]
    status: dict[str, Any]

    @property
    def worst_margin_c(self) -> float:
        return self.run.worst_margin_c()

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


def drive(cfg: MpcConfig, *, preset: str, seed: int, start: bool) -> ExperimentRun:
    """Run the supervisor + solver against the truth plant, starting an experiment on
    the first output after ``WARMUP_S`` when ``start``."""
    plant = plant_for(cfg, preset=preset, seed=seed)
    sup = Supervisor(cfg)
    target = cfg.channels[0]  # the DAS example declares no fan groups: one channel
    prev: dict[str, float] = dict.fromkeys(cfg.channels, 0.5)
    reasons: list[str] = []
    state: dict[str, Any] = {"status": {}, "ran": False}

    def controller(obs, _cfg, st: MpcState):
        nonlocal prev
        if start and obs.ts >= WARMUP_S and sup.experiment is None and not state["ran"]:
            try:  # retried every tick: the enclosure may still be settling
                sup.submit(Ident("start", channel=target))
                state["ran"] = True
                reasons.clear()
            except IntentConflict as exc:
                reasons[:] = [str(exc)]
        plan = sup.plan_tick()
        cmd, st2 = step(obs, plan.cfg, st)
        out = sup.compose(cmd, plan, prev)
        sup.record_tick(
            obs=obs, mpc_cmd=cmd, cmd=out, state=st2, applied=True, usb_present=True, ts=obs.ts
        )
        prev = dict(out.pwm)
        if sup.experiment is not None or state["status"].get("running"):
            state["status"] = sup.snapshot().extra["experiment"]
        return out, st2

    ticks = int((WARMUP_S + 2 * DURATION_S) / cfg.dt)
    run = run_das_closed_loop(plant, cfg, controller, ticks)
    return ExperimentRun(run, reasons, state["status"])


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
