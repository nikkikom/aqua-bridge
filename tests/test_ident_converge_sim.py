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

**The two arms.** Both run 16 starts of the same length at the same amplitude on the same
plant. The zone-wide arm starts on one channel per zone (``ORDER``) and ``ident_parallel``
widens each start to that zone's channels, each on its own code. The control arm is a
round robin over **all eight** channels (``SEQUENTIAL_ORDER``) -- the schedule a daemon
cycling every fan group would actually run -- one channel per start. Restricting the
control arm to the four aquaero channels instead would compare channel *coverage* rather
than simultaneity, and would flatter the zone-wide arm: it converges nothing on any seed.

**What one channel at a time does and does not do.** The thermal model's air block per
zone regresses one airflow regressor per fan group of that zone, and ``converged`` asks
the smallest eigenvalue of their information matrix to pass ``thermal.PE_MIN``
(:mod:`aqua_bridge.control.thermal`, *PE monitor*): every group of the zone has to move
independently of the others inside the monitor's ~30-window memory. Telegraphing one
group while the solver carries the rest excites one direction at a time -- measured on
the example config, the channels of a three-group zone correlate 0.91 to 0.98 and the
smallest eigenvalue sits at 0.002 to 0.05 against diagonals of 0.06 to 0.17. It is *not*
true that it never clears the bound: the round robin peaks at 0.13 to 0.21 on some zone
of every seed (asserted below), and on seed 3 that was enough for z1 to latch
``converged`` from a transient, its live ``pe_min`` back at 0.0076 by the end of the run.
What the zone-wide excitation buys is that the bound is *held* -- 0.08 to 0.17 still
standing at the end -- and a zone more per seed on two seeds of three.

Observed over 16 h, zone-wide against the round robin (converged zones; the zone-wide
arm's ``pe_min`` peak and its value at the end of the run; mean PWM of each arm):

* seed 2 -- z0 and z3 against nothing; 0.117 and 0.336 peak, 0.083 and 0.125 at the
  end; 443 and 472 excited windows; mean PWM 0.406 against 0.357;
* seed 3 -- z1 against z1; 0.253 peak, 0.100 at the end; 478 windows; 0.276 / 0.239;
* seed 4 -- z3 against nothing; 0.359 peak, 0.172 at the end; 472 windows;
  0.314 / 0.279.

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
#: Zone-wide arm: one start per zone; consecutive starts share as few channels as the
#: chain allows. ``ident_parallel`` widens each to that zone's own channels.
ORDER = ("xt1", "xt3", "xt2", "xt4")
#: Control arm: one channel per start, round robin over every channel of the plant.
SEQUENTIAL_ORDER = ("xt1", "qd1", "xt3", "qd3", "xt2", "qd2", "xt4", "qd4")
TOL = 1e-9
#: What each arm reached per seed (module docstring). ``dead`` are the zones that close
#: **no** regression window in 16 h: on seed 3 bay b03's redundant proximal pair is read
#: as ``swapped`` on nearly every tick and ``model_reset_on_swap`` resets z0's air
#: accumulator with it, so that zone can never converge for reasons no excitation
#: reaches. It is pinned here so a zone going dead is visible instead of silent.
MEASURED: dict[int, dict[str, tuple[str, ...]]] = {
    2: {"parallel": ("z0", "z3"), "sequential": (), "dead": ()},
    3: {"parallel": ("z1",), "sequential": ("z1",), "dead": ("z0",)},
    4: {"parallel": ("z3",), "sequential": (), "dead": ()},
}
#: Floors on the converged zones, from the runs above (peak 0.12-0.36, end 0.08-0.17,
#: 443-478 windows) rather than from the ``converged`` rule's own 0.05 / 30, which
#: ``status == "converged"`` already implies.
PE_PEAK_FLOOR = 0.10
WINDOWS_FLOOR = 300
#: Mean PWM the zone-wide arm may add over the round robin (observed +0.036 to +0.050).
MEAN_PWM_BUDGET = 0.10


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


def _run(cfg: MpcConfig, seed: int, order: tuple[str, ...]) -> dict[str, Any]:
    """One closed-loop run, starting an experiment on ``order`` round robin."""
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
            rig.sup.submit(Ident("start", channel=order[nxt % len(order)]))
            starts += 1
        except (IntentConflict, IntentInvalid):
            pass
        nxt += 1
    zones = summary.get("zones") or {}
    return {
        "starts": starts,
        "converged": sorted(z for z, v in zones.items() if v["status"] in ("converged", "frozen")),
        "dead": sorted(z for z, v in zones.items() if not v["excited_windows"]),
        "pe_max": pe_max,
        "zones": zones,
        "worst_margin_c": worst_margin,
        "violations": violations,
        "mean_pwm": pwm_sum / max(ticks, 1),
    }


@pytest.mark.nightly
@pytest.mark.parametrize("seed", sorted(MEASURED))
def test_a_zone_wide_experiment_converges_a_zone_that_one_channel_at_a_time_does_not(
    das_example_cfg, seed
):
    """The item's question, answered on the daemon's own path, against the honest control
    arm: the same 16 starts, one channel each, over every channel of the plant.

    The numbers the runs reached are in :data:`MEASURED` and the module docstring, and
    are asserted here rather than the thresholds of the ``converged`` rule, which
    ``status == "converged"`` implies on its own. Neither arm ever violates a drive
    limit, and the zone-wide arm keeps the *larger* true margin -- ``ident_levels:
    above`` only ever adds cooling.
    """
    want = MEASURED[seed]
    parallel = _run(_config(das_example_cfg, parallel=True), seed, ORDER)
    sequential = _run(_config(das_example_cfg, parallel=False), seed, SEQUENTIAL_ORDER)
    assert parallel["starts"] == sequential["starts"] > 0  # the same experiment time

    # 1. the answer, per seed and per zone: the zone-wide arm converges the zones it
    #    converged, one channel at a time converges no more than it did, and never more
    #    than the zone-wide arm.
    assert set(parallel["converged"]) >= set(want["parallel"]), {
        z: v["status"] for z, v in parallel["zones"].items()
    }
    assert set(sequential["converged"]) <= set(want["sequential"]), sequential["converged"]
    assert set(parallel["converged"]) >= set(sequential["converged"])

    # 2. it is the PE monitor that moved, and it is held, not touched once
    for z in parallel["converged"]:
        assert parallel["pe_max"][z] >= PE_PEAK_FLOOR
        assert parallel["zones"][z]["pe_min"] >= thermal.PE_MIN  # still standing at the end
        assert parallel["zones"][z]["excited_windows"] >= WINDOWS_FLOOR
        assert parallel["zones"][z]["pred_err_c"] < das_example_cfg.model_max_pred_err_c
    # one channel at a time does clear the bound in bursts -- what it cannot do is hold
    # it, so a zone it converges ends below what the zone-wide arm holds there.
    assert max(sequential["pe_max"].values()) > thermal.PE_MIN
    for z in sequential["converged"]:
        assert sequential["zones"][z]["pe_min"] < parallel["zones"][z]["pe_min"]

    # 3. a zone that closes no regression window at all is a defect, not a result: it is
    #    named per seed, so a new one fails here instead of quietly shrinking the test.
    assert parallel["dead"] == sequential["dead"] == list(want["dead"])

    # 4. the safety price: none. `above` never commands less cooling than the solver,
    #    so the true margin cannot be the worse of the two, and no limit is crossed.
    assert parallel["violations"] == 0 and sequential["violations"] == 0
    assert parallel["worst_margin_c"] >= sequential["worst_margin_c"] - TOL

    # 5. the noise price is real and bounded: more fan than one channel at a time
    #    (observed +0.036 to +0.050 mean PWM over the whole run)
    assert parallel["mean_pwm"] >= sequential["mean_pwm"] - TOL
    assert parallel["mean_pwm"] <= sequential["mean_pwm"] + MEAN_PWM_BUDGET
