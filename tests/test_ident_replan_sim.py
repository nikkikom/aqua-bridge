"""Re-planned experiment levels against frozen ones on the truth simulator (item 52).

Nightly, because it runs the real ``Loop`` + ``Supervisor`` + ``control/ident`` closed
loop over the DAS truth simulator (``sim/das.py``, ``rich`` preset) twice per case, once
with ``ident_replan: true`` and once with ``false``, on the same plant and the same seed,
starting one channel experiment after another as soon as the preconditions allow.

It is the A/B the numbers in PROJECT.md section 8.4 item 52 come from, and it is run for
**both** solvers: the ``pi`` solver has no dependence on the last command at all, while
the DAS MPC's objective carries ``weight_dpwm ||u_0 - prev||^2``, which is what makes the
demand echo the experiment's own level (``control/ident.py``, "Levels re-planned"). The
three things it pins are the point of the item and of that echo:

* re-planning never holds a channel below the solver's own command, and the frozen plan
  does (the behaviour item 52 replaces);
* the levels do not run away: no anchor ends more than ``ident_amplitude`` above the
  highest demand the frozen arm ever saw on that channel -- a ratchet would show here
  first, and on the MPC arm first of all;
* the fit is no worse than the frozen arm's beyond a loose margin, per seed.

The bounds are loose on purpose: this is a regression net for the re-planning, not a
convergence test. Convergence evidence is ``tests/test_thermal_ident.py`` (open-loop
excitation); with one experiment at a time and the solver moving a zone's other channels,
neither arm converges in this scenario.
"""

from __future__ import annotations

import dataclasses
from typing import Any

import numpy as np
import pytest

from aqua_bridge.control.intents import Ident, IntentConflict, IntentInvalid
from aqua_bridge.control.loop import Loop
from aqua_bridge.control.supervisor import Supervisor
from aqua_bridge.model import MpcConfig, PlantObservation, SolverKind
from aqua_bridge.sim.das import build_das_plant
from test_thermal_ident import e_errors, k_errors, truth_topology

HOURS = 4.0
TOL = 1e-9
#: How much worse than the frozen arm the re-planned fit may come out, per seed.
FIT_SLACK = 0.5


class _Rig:
    """Loop source and sink over a ``DasPlant``."""

    def __init__(self, cfg: MpcConfig, plant: Any) -> None:
        self.cfg = cfg
        self.plant = plant
        self.sup = Supervisor(cfg)
        self.loop = Loop(self, self, cfg, self.sup)

    def read(self) -> PlantObservation:
        raw = self.plant.observe()
        obs = PlantObservation(
            temps={name: raw.temps.get(name) for name in self.cfg.temps},
            rpm=raw.rpm,
            pwm=raw.pwm,
            ts=raw.ts,
        )
        smart = self.plant.observe_smart()
        return dataclasses.replace(obs, inputs={**obs.inputs, "smart": smart}) if smart else obs

    def apply(self, cmd: Any) -> None:
        self.plant.apply(cmd.pwm)


def _config(example: MpcConfig, *, replan: bool, solver: str) -> MpcConfig:
    """The DAS example with experiments on. ``k_sigma: 1`` is what lets one start at all
    (the envelope counts it twice, section 8 item 53); the DAS MPC needs the prior model
    to act before any experiment has taught it one."""
    return dataclasses.replace(
        example,
        solver=SolverKind.MPC if solver == "mpc" else SolverKind.PI,
        model_accept_prior=solver == "mpc",
        model_shadow=True,
        ident_enabled=True,
        ident_replan=replan,
        estimator=dataclasses.replace(example.estimator, k_sigma=1.0),
    )


def _run(cfg: MpcConfig, seed: int, hours: float = HOURS) -> dict[str, Any]:
    """One closed-loop run; experiments started one channel at a time, round robin."""
    plant = build_das_plant(
        truth_topology(cfg, seed), preset="rich", seed=seed, dt=cfg.dt, initial_pwm=0.6
    )
    rig = _Rig(cfg, plant)
    order, nxt, starts, held, aborts = list(cfg.channels), 0, 0, 0, 0
    was_running = False
    anchor_max: dict[str, float] = {}
    demand_max: dict[str, float] = {}
    summary: dict[str, Any] = {}
    for _ in range(int(hours * 3600.0 / cfg.dt)):
        r = rig.loop.tick()
        plant.advance()
        if r.mpc_cmd is None:
            continue
        summary = r.mpc_cmd.diagnostics.get("thermal") or summary
        for ch, want in (r.mpc_cmd.diagnostics.get("target_pwm") or {}).items():
            demand_max[ch] = max(demand_max.get(ch, 0.0), float(want))
        status = rig.sup.snapshot().extra["experiment"]
        if not status["running"]:
            aborts += status["last_abort_reason"] is not None and was_running
            was_running = False
            try:
                rig.sup.submit(Ident("start", channel=order[nxt % len(order)]))
                starts += 1
            except (IntentConflict, IntentInvalid):
                pass
            nxt += 1
            continue
        was_running = True
        for ch in status["channels"]:
            held += r.mpc_cmd.pwm[ch] - r.cmd.pwm[ch] > TOL
            anchor_max[ch] = max(anchor_max.get(ch, 0.0), float(status["plan_base"][ch]))
    out: dict[str, Any] = {
        "starts": starts,
        "aborts": aborts,
        "held_ticks": held,
        "anchor_max": anchor_max,
        "demand_max": demand_max,
        "status": summary.get("status"),
    }
    if summary.get("zones"):
        e = e_errors(cfg, summary, plant)
        k = k_errors(summary, plant)
        out["e_rms"] = float(np.sqrt(np.mean([v * v for v in e.values()])))
        out["k_rms"] = float(np.sqrt(np.mean([v * v for v in k.values()])))
    return out


@pytest.mark.nightly
@pytest.mark.parametrize("solver", ["pi", "mpc"])
@pytest.mark.parametrize("seed", [1, 2, 3])
def test_re_planned_levels_follow_the_solver_without_running_away(das_example_cfg, solver, seed):
    frozen = _run(_config(das_example_cfg, replan=False, solver=solver), seed)
    live = _run(_config(das_example_cfg, replan=True, solver=solver), seed)
    assert live["starts"] > 0 and frozen["starts"] > 0

    # 1. the rule: never below the solver's own command -- and the frozen plan does hold
    assert live["held_ticks"] == 0
    assert frozen["held_ticks"] > 0

    # 2. no ratchet: the echo of the experiment's own level does not accumulate into the
    #    anchor, which would show as an anchor far above any demand the frozen arm saw
    amplitude = das_example_cfg.ident_amplitude
    for ch, anchor in live["anchor_max"].items():
        ceiling = max(frozen["demand_max"].get(ch, 0.0), frozen["anchor_max"].get(ch, 0.0))
        assert anchor <= ceiling + amplitude + TOL, (ch, anchor, ceiling)

    # 3. the fit is no worse than the frozen arm's beyond a loose margin
    if "e_rms" in frozen and "e_rms" in live:
        assert live["e_rms"] <= frozen["e_rms"] + FIT_SLACK, (live["e_rms"], frozen["e_rms"])
        assert live["k_rms"] <= frozen["k_rms"] + FIT_SLACK, (live["k_rms"], frozen["k_rms"])
