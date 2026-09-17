"""A closed-loop scenario in which the thermal fit converges (plan section 8 item 102).

Nightly, because each case runs the real ``Loop`` + ``Supervisor`` + ``control/ident``
over the DAS truth simulator (``sim/das.py``, ``rich`` preset) for 16 simulated hours,
twice: once with ``ident_parallel: true`` and once with ``false``, on the same plant and
the same seed.

Until this test there was no closed-loop convergence evidence at all: every
``converged`` number came from ``tests/test_thermal_ident.py``, which drives the outputs
**open loop** (fixed 0.35 / 0.8 levels, an independent sequence on every channel at once)
and never runs ``control/ident.py``, while ``tests/test_ident_replan_sim.py`` runs
``ident.py`` in a scenario whose ``pe_min`` never leaves the floor. The two machines were
tested for different things.

Why one group at a time cannot do it. The thermal model's air block per zone regresses
one airflow regressor per fan group of that zone, and ``converged`` asks the smallest
eigenvalue of their information matrix to pass ``thermal.PE_MIN``
(:mod:`aqua_bridge.control.thermal`, *PE monitor*): every group of the zone has to move
independently of the others inside the monitor's ~30-window memory. Telegraphing one
group while the solver carries the rest fills one direction of that matrix, not ``G`` of
them -- measured on the example config, the channels of a three-group zone correlate 0.91
to 0.98 and the smallest eigenvalue sits at 0.002 to 0.05 against diagonals of 0.06 to
0.17. ``ident_parallel`` excites every channel of the target's own zones at once, each on
its own code, which is the same excitation the open-loop test uses.

The scenario, and why each knob is where it is:

* ``ident_parallel: true`` -- the point of the test;
* ``ident_amplitude: 0.25`` -- ``pe_min`` is a *relative* measure, so an ``above``
  telegraph at base ``u`` needs ``A >= 0.576 (u - deadband)`` to clear ``PE_MIN`` at all;
  the example's channels park between 0.21 and 0.94 PWM, which asks 0.06 to 0.49;
* ``ident_hold_s: [240, 360, 480]`` -- longer than ``model_window_s`` (120 s), so a
  regression window sits inside one hold instead of averaging two;
* ``ident_max_duration_s: 3600`` -- ``PE_WINDOWS * model_window_s``, the monitor's own
  memory: a shorter experiment can never dominate it;
* starts on ``xt1, xt3, xt2, xt4`` -- one per zone, and consecutive starts share as few
  channels as the chain allows (a channel released at its high level is re-initialised
  there by the bumpless release, and a start that takes it as its base steps up).
"""

from __future__ import annotations

import dataclasses
from typing import Any

import pytest

from aqua_bridge.control import thermal
from aqua_bridge.control.intents import Ident, IntentConflict, IntentInvalid
from aqua_bridge.control.loop import Loop
from aqua_bridge.control.supervisor import Supervisor
from aqua_bridge.model import MpcConfig, PlantObservation, SolverKind
from aqua_bridge.sim.das import build_das_plant
from test_thermal_ident import truth_topology

HOURS = 16.0
#: One start per zone; consecutive starts share as few channels as the chain allows.
ORDER = ("xt1", "xt3", "xt2", "xt4")
TOL = 1e-9


class _Rig:
    """Loop source and sink over a ``DasPlant`` (as ``test_ident_replan_sim``)."""

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


def _config(example: MpcConfig, *, parallel: bool) -> MpcConfig:
    """The DAS example with the scenario of the module docstring. ``k_sigma: 1`` is what
    lets a start happen at all on a settled enclosure (the envelope counts it twice,
    section 8 item 53)."""
    return dataclasses.replace(
        example,
        solver=SolverKind.PI,
        model_shadow=True,
        ident_enabled=True,
        ident_parallel=parallel,
        ident_amplitude=0.25,
        ident_hold_s=(240.0, 360.0, 480.0),
        ident_max_duration_s=3600.0,
        estimator=dataclasses.replace(example.estimator, k_sigma=1.0),
    )


def _run(cfg: MpcConfig, seed: int) -> dict[str, Any]:
    """One closed-loop run; a start on each zone's own aquaero channel, round robin."""
    plant = build_das_plant(
        truth_topology(cfg, seed), preset="rich", seed=seed, dt=cfg.dt, initial_pwm=0.6
    )
    rig = _Rig(cfg, plant)
    nxt = starts = 0
    worst_margin = float("inf")
    violations = 0
    pwm_sum = 0.0
    ticks = 0
    pe_max: dict[str, float] = {}
    summary: dict[str, Any] = {}
    for _ in range(int(HOURS * 3600.0 / cfg.dt)):
        r = rig.loop.tick()
        plant.advance()
        if r.mpc_cmd is None:
            continue
        ticks += 1
        summary = r.mpc_cmd.diagnostics.get("thermal") or summary
        for z, zone in (summary.get("zones") or {}).items():
            pe_max[z] = max(pe_max.get(z, 0.0), float(zone["pe_min"]))
        margins = [v for v in plant.margins().values() if v is not None]
        if margins:
            worst_margin = min(worst_margin, min(margins))
            violations += sum(1 for v in margins if v < 0.0)
        pwm_sum += sum(r.cmd.pwm.values()) / len(r.cmd.pwm)
        if rig.sup.snapshot().extra["experiment"]["running"]:
            continue
        try:
            rig.sup.submit(Ident("start", channel=ORDER[nxt % len(ORDER)]))
            starts += 1
        except (IntentConflict, IntentInvalid):
            pass
        nxt += 1
    zones = summary.get("zones") or {}
    return {
        "starts": starts,
        "converged": sorted(z for z, v in zones.items() if v["status"] in ("converged", "frozen")),
        "pe_max": pe_max,
        "zones": zones,
        "worst_margin_c": worst_margin,
        "violations": violations,
        "mean_pwm": pwm_sum / max(ticks, 1),
    }


@pytest.mark.nightly
@pytest.mark.parametrize("seed", [2, 3, 4])
def test_a_zone_wide_experiment_converges_a_zone_and_one_group_at_a_time_does_not(
    das_example_cfg, seed
):
    """The item's question, answered on the daemon's own path.

    Observed over 16 h (seeds 2 / 3 / 4): ``ident_parallel: true`` converges z0 and z3 /
    z1 / z3 with ``pe_min`` peaking at 0.12-0.34, and ``false`` converges nothing on any
    of the three, its air ``pe_min`` peaking at 0.05-0.25 and holding above
    ``PE_MIN`` in at most a tenth of the windows of the zones that matter. Neither arm
    ever violates a drive limit, and the zone-wide arm keeps the *larger* true margin --
    ``ident_levels: above`` only ever adds cooling.
    """
    parallel = _run(_config(das_example_cfg, parallel=True), seed)
    sequential = _run(_config(das_example_cfg, parallel=False), seed)
    assert parallel["starts"] > 0 and sequential["starts"] > 0

    # 1. the answer: a zone reaches converged, and one group at a time does not
    assert parallel["converged"], {z: v["status"] for z, v in parallel["zones"].items()}
    assert not sequential["converged"], sequential["converged"]

    # 2. it is the PE monitor that moved, and it is above the threshold where it counts
    for z in parallel["converged"]:
        assert parallel["pe_max"][z] > thermal.PE_MIN
        assert parallel["zones"][z]["excited_windows"] >= thermal.MIN_WINDOWS
        assert parallel["zones"][z]["pred_err_c"] < das_example_cfg.model_max_pred_err_c

    # 3. the safety price: none. `above` never commands less cooling than the solver,
    #    so the true margin cannot be the worse of the two, and no limit is crossed.
    assert parallel["violations"] == 0 and sequential["violations"] == 0
    assert parallel["worst_margin_c"] >= sequential["worst_margin_c"] - TOL

    # 4. the noise price is real and bounded: more fan than one group at a time, and
    #    well inside the band (observed +0.03 to +0.06 mean PWM over the whole run)
    assert parallel["mean_pwm"] >= sequential["mean_pwm"] - TOL
    assert parallel["mean_pwm"] <= sequential["mean_pwm"] + 0.15
