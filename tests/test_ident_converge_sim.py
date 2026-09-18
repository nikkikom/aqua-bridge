"""Closed-loop identification on the daemon's own path (section 8 items 102, 109, 111, 112).

Nightly, because each case runs the real ``Loop`` + ``Supervisor`` + ``control/ident``
over the DAS truth simulator (``sim/das.py``, ``rich`` preset) for 16 simulated hours,
twice: once with ``ident_parallel: true`` and once with ``false``, on the same plant and
the same seed. Three tests:

* item 102 -- the zone-wide schedule against a one-channel-at-a-time round robin;
* item 112 -- what a finished experiment leaves on the fans, now that the release hands
  the solver its own level instead of the level on the fan;
* item 111 -- what the bays whose ``rel_se(k)`` stays above the bound actually lack,
  measured by running the same schedule for 36 h instead of 16 h.

**Nightly cost (section 8 item 122).** Measured at 9 cases (3 seeds x 3 tests) in
~12.5-13.7 min, isolated or under an 8-way ``ci_pytest_shards.py`` run alike (item 126
covers what contention does to a *timing* gate; this suite has none). The 36 h case is
the one item 122 names as worth trimming: at 16 h all three seeds' evidence is already
in :data:`MEASURED`, and the 36 h re-run mainly re-confirms convergence rather than
finding something new -- seeds 2 and 4 converge every zone with **no** blocked bay,
seed 3 alone still has one (b15, module docstring above). So the 36 h case now runs
:data:`NIGHTLY_LONG_SEEDS` (seed 3 only), not every seed in :data:`MEASURED_LONG`:
seed 3 is the one case that both confirms the 16 h -> 36 h improvement *and* exercises
the residual-blocker assertions (``blocked_bays``), which no other seed does. **What
this drops**: nightly no longer re-verifies that seeds 2 and 4 also reach full,
unblocked convergence at 36 h -- that is now a one-time, dated measurement
(:data:`MEASURED_LONG`, kept for both seeds as the historical record and the module
docstring's own numbers) rather than a nightly-checked fact. A regression that broke
convergence on seed 2 or 4 specifically at 36 h (and not at 16 h, and not on seed 3)
would go unnoticed until someone re-runs those seeds by hand
(``pytest tests/test_ident_converge_sim.py -m nightly -k "second_gate and (2 or 4)"``).
That risk was judged smaller than the wall clock two more 36 h runs cost every night,
given seed 3 already exercises the same mechanism (the air block is never the
blocker, ``se(k)`` falls with excited windows) and the two dropped seeds add no new
*kind* of evidence, only more of the same kind.

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
group while the solver carries the rest excites one direction at a time. It is *not*
true that it never clears the bound: the round robin peaks at 0.19 to 0.48 on some zone
of every seed (asserted below), and on seed 4 that is enough for z0 to latch
``converged`` *and hold* the bound there (``pe_min`` 0.209 at the end) -- z0 is the one
zone of the example with two fan groups rather than three, so it has one direction fewer
to separate. What the zone-wide excitation buys is a zone more per seed on all three
seeds, and the bound held on every zone it converges.

Observed over 16 h, zone-wide against the round robin (converged zones; the zone-wide
arm's ``pe_min`` peak and its value at the end of the run; mean PWM of each arm). These
numbers moved with section 8 item 112: with the fans no longer left elevated after each
experiment the channels park lower, where a fixed 0.25 step is a *larger* relative
airflow swing, so ``pe_min`` roughly doubles on the zones that converge. They moved again
with section 8 item 109: on the fused layout a redundant pair's bay statistic had a
standing bias that read as `swapped` on almost every tick for bay b03 (2788 and 2391 of
2880 ticks on seeds 3 and 4 pre-fix), and ``model_reset_on_swap`` threw away not just the
bay's block but its **zone's** air accumulator with it, so z0 closed 0 windows on seed 3
and stayed short on seed 4. With the statistic corrected the bay is never reported
swapped on either seed and z0 converges on both.

* seed 2 -- z0 and z3 against nothing; 0.282 and 0.238 peak, 0.216 and 0.205 at the
  end; 467 and 471 excited windows; mean PWM 0.3587 against 0.3660; unaffected by
  item 109 (b03's pair never tripped the rule on this seed, before or after);
* seed 3 -- z0 and z2 against nothing; 0.320 and 0.344 peak, 0.280 and 0.090 at the
  end; 466 and 471 excited windows; mean PWM 0.2636 / 0.2387; z0 was dead before
  item 109 (pinned exactly, so a zone going dead again is visible instead of silent)
  and now converges;
* seed 4 -- z0 and z3 against z0; 0.292 and 0.316 peak, 0.096 and 0.201 at the end;
  474 and 468 excited windows; mean PWM 0.2969 / 0.2794; the round robin now also
  converges (and holds) z0, which item 109's fix stopped resetting.

The scenario, and why each knob is where it is:

* ``ident_parallel: true`` -- the point of the test;
* ``ident_amplitude: 0.25`` -- ``pe_min`` is a *relative* measure, so an ``above``
  telegraph at base ``u`` needs ``A >= 0.576 (u - deadband)`` to clear ``PE_MIN`` at all;
  the example's channels park between 0.20 and 0.86 PWM, which asks 0.06 to 0.44;
* ``ident_hold_s: [240, 360, 480]`` -- longer than ``model_window_s`` (120 s), so a
  regression window sits inside one hold instead of averaging two;
* ``ident_max_duration_s: 3600`` -- ``PE_WINDOWS * model_window_s``, the monitor's own
  memory: a shorter experiment can never dominate it;
* starts on ``xt1, xt3, xt2, xt4`` -- one per zone, and consecutive starts share as few
  channels as the chain allows.
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
#: **no** regression window in 16 h -- none do since item 109's fix, and it is pinned
#: here so a zone going dead is visible instead of silent. ``held`` are the zones the
#: *round robin* converged whose ``pe_min`` is still above ``PE_MIN`` at the end of the
#: run rather than latched from a burst.
MEASURED: dict[int, dict[str, tuple[str, ...]]] = {
    2: {"parallel": ("z0", "z3"), "sequential": (), "dead": (), "held": ()},
    3: {"parallel": ("z0", "z2"), "sequential": (), "dead": (), "held": ()},
    4: {"parallel": ("z0", "z3"), "sequential": ("z0",), "dead": (), "held": ("z0",)},
}
#: What 36 h of the *same* schedule reaches, and what still blocks each zone that does
#: not converge even then (section 8 item 111). With item 109's fix, b03 and b10 are no
#: longer reported ``swapped`` on almost every tick and close windows like any other
#: bay; the one remaining blocker is b15 on seed 3, whose fitted ``k`` is under half the
#: prior, so even its ``se`` of 0.109 is half of it.
MEASURED_LONG: dict[int, dict[str, Any]] = {
    2: {"converged": ("z0", "z1", "z2", "z3"), "blocked_bays": {}},
    3: {"converged": ("z0", "z1", "z2"), "blocked_bays": {"z3": ("b15",)}},
    4: {"converged": ("z0", "z1", "z2", "z3"), "blocked_bays": {}},
}
#: Which of :data:`MEASURED_LONG`'s seeds the nightly job actually re-runs at 36 h
#: (section 8 item 122): seed 3 alone, the one with a real ``blocked_bays`` entry to
#: exercise. Seeds 2 and 4's rows stay in :data:`MEASURED_LONG` as the dated
#: measurement the module docstring's "Nightly cost" section describes, not as a
#: nightly-checked fact -- see that section for what dropping them costs.
NIGHTLY_LONG_SEEDS = (3,)
LONG_HOURS = 36.0
#: Floors on the converged zones, from the runs above (peak 0.24-0.34, end 0.09-0.28,
#: 466-474 windows) rather than from the ``converged`` rule's own 0.05 / 30, which
#: ``status == "converged"`` already implies.
PE_PEAK_FLOOR = 0.10
WINDOWS_FLOOR = 300
#: What the **old** release (the experiment's channels put into ``TickPlan.released``,
#: so the solver re-initialised bumplessly at the PWM on the fan) spent on the same
#: plant and seed, measured by reinstating it beside the new one: the enclosure mean and
#: qd1's own mean, the channel the ratchet hit hardest. Both are asserted, because the
#: enclosure mean alone leaves margins of 0.012 and 0.017 on seeds 3 and 4 while qd1's
#: separates the two releases by 0.06 to 0.30 (section 8 item 112).
OLD_RELEASE: dict[int, dict[str, float]] = {
    2: {"mean_pwm": 0.4064, "qd1": 0.790},
    3: {"mean_pwm": 0.2758, "qd1": 0.362},
    4: {"mean_pwm": 0.3142, "qd1": 0.374},
}
#: How far the two arms' mean PWM may sit apart. Since section 8 item 112 the zone-wide
#: arm is no longer always the louder of the two: a released experiment hands the solver
#: its own level instead of the level on the fan, so nothing is left elevated between
#: starts. Observed 0.3587 / 0.2636 / 0.2969 against 0.3660 / 0.2387 / 0.2794.
MEAN_PWM_BUDGET = 0.10
#: Worst true margin either arm may come down to, degC. Observed 4.03-4.51 zone-wide and
#: 4.12-4.30 for the round robin, against 4.07-4.30 with no experiment at all (item 102).
MARGIN_FLOOR_C = 3.5


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


def _run(cfg: MpcConfig, seed: int, order: tuple[str, ...], hours: float = HOURS) -> dict[str, Any]:
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
    ch_min: dict[str, float] = {}
    ch_mean: dict[str, float] = {}
    # section 8 item 120: the deepest any channel ever sat under the solver's own command
    # on one tick, the highest it ever sat above it, the cooling given up per tick summed
    # over the run, and the starts the ``band:`` precondition refused. None of the three
    # excursion figures is a bound on the *experiment* alone: the command a channel is
    # measured against is this tick's solver demand, which the plan does not follow within
    # a tick, and every other floor the supervisor applies is in there too. What the
    # experiment itself may plan is bounded by ``ident_amplitude`` in both directions and
    # pinned in ``tests/test_ident_experiment.py``.
    max_dip = 0.0
    max_rise = 0.0
    dip_sum = 0.0
    band_refusals = 0
    summary: dict[str, Any] = {}
    for _ in range(int(hours * 3600.0 / cfg.dt)):
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
        tick_dip = 0.0
        for ch, value in r.cmd.pwm.items():
            ch_min[ch] = min(ch_min.get(ch, 1.0), float(value))
            ch_mean[ch] = ch_mean.get(ch, 0.0) + float(value)
            want = r.mpc_cmd.pwm.get(ch)
            if want is not None:
                max_dip = max(max_dip, float(want) - float(value))
                max_rise = max(max_rise, float(value) - float(want))
                tick_dip += max(0.0, float(want) - float(value))
        dip_sum += tick_dip / len(r.cmd.pwm)
        if rig.sup.snapshot().extra["experiment"]["running"]:
            continue
        try:
            rig.sup.submit(Ident("start", channel=order[nxt % len(order)]))
            starts += 1
        except (IntentConflict, IntentInvalid) as exc:
            # ``band:<ch>`` only -- ``start_band:<bay>`` is a different precondition that
            # happens to end in the same five characters
            reasons = str(exc).rsplit(": ", 1)[-1].split(", ")
            band_refusals += any(r.startswith("band:") for r in reasons)
        nxt += 1
    zones = summary.get("zones") or {}
    return {
        "starts": starts,
        "converged": sorted(z for z, v in zones.items() if v["status"] in ("converged", "frozen")),
        "dead": sorted(z for z, v in zones.items() if not v["excited_windows"]),
        "pe_max": pe_max,
        "zones": zones,
        "bays": summary.get("bays") or {},
        "worst_margin_c": worst_margin,
        "violations": violations,
        "mean_pwm": pwm_sum / max(ticks, 1),
        "max_dip": max_dip,
        "max_rise": max_rise,
        "dip_per_tick": dip_sum / max(ticks, 1),
        "band_refusals": band_refusals,
        "ch_min": ch_min,
        "ch_mean": {ch: v / max(ticks, 1) for ch, v in ch_mean.items()},
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
    limit and both keep several degrees of true margin.
    """
    want = MEASURED[seed]
    parallel = _run(_config(das_example_cfg, parallel=True), seed, ORDER)
    sequential = _run(_config(das_example_cfg, parallel=False), seed, SEQUENTIAL_ORDER)
    assert parallel["starts"] == sequential["starts"] > 0  # the same experiment time

    # 1. the answer, per seed and per zone: the zone-wide arm converges the zones it
    #    converged, one channel at a time converges no more than it did, and never more
    #    than the zone-wide arm. With item 109's fix every seed's zone-wide arm converges
    #    something, but the empty branch stays: a seed converging *nothing* would assert
    #    nothing at all about the behaviour the test's name claims (``>= set()`` is a
    #    tautology) if it were only ever checked the other way.
    if want["parallel"]:
        assert set(parallel["converged"]) >= set(want["parallel"]), {
            z: v["status"] for z, v in parallel["zones"].items()
        }
    else:
        assert parallel["converged"] == [], {z: v["status"] for z, v in parallel["zones"].items()}
    assert set(sequential["converged"]) <= set(want["sequential"]), sequential["converged"]
    assert set(parallel["converged"]) >= set(sequential["converged"])

    # 2. it is the PE monitor that moved, and it is held, not touched once
    for z in parallel["converged"]:
        assert parallel["pe_max"][z] >= PE_PEAK_FLOOR
        assert parallel["zones"][z]["pe_min"] >= thermal.PE_MIN  # still standing at the end
        assert parallel["zones"][z]["excited_windows"] >= WINDOWS_FLOOR
        assert parallel["zones"][z]["pred_err_c"] < das_example_cfg.model_max_pred_err_c
    # one channel at a time does clear the bound, in bursts on a three-group zone and
    # for good on the two-group z0 of seed 4: which of its converged zones still stand
    # above the bound at the end of the run is per-seed data, not a rule.
    assert max(sequential["pe_max"].values()) > thermal.PE_MIN
    # a held zone is a converged zone by construction, so this pins the record itself:
    # without it the loop below goes quiet the day a recorded zone stops converging.
    assert set(sequential["converged"]) >= set(want["held"]), sequential["converged"]
    for z in sequential["converged"]:
        held = float(sequential["zones"][z]["pe_min"]) >= thermal.PE_MIN
        assert held == (z in want["held"]), (z, sequential["zones"][z]["pe_min"])

    # 3. a zone that closes no regression window at all is a defect, not a result: it is
    #    named per seed, so a new one fails here instead of quietly shrinking the test.
    assert parallel["dead"] == sequential["dead"] == list(want["dead"])

    # 4. the safety price: no drive ever crosses a limit in either arm, and both keep
    #    several degrees of true margin. The cross-arm *ordering* of the margin is no
    #    longer asserted: `above` never commands less cooling than the solver **on the
    #    tick**, which is a per-tick statement, and since section 8 item 112 the arms no
    #    longer differ mainly by what a finished experiment left on the fans -- on seed 4
    #    the zone-wide arm ends 0.27 degC below the round robin (4.026 against 4.296).
    assert parallel["violations"] == 0 and sequential["violations"] == 0
    assert parallel["worst_margin_c"] >= MARGIN_FLOOR_C
    assert sequential["worst_margin_c"] >= MARGIN_FLOOR_C

    # 5. the noise price is real and bounded, and it now goes both ways: with the release
    #    of item 112 the zone-wide arm leaves nothing elevated between starts, so on
    #    seed 2 it is the *quieter* of the two (0.3587 against 0.3660).
    assert abs(parallel["mean_pwm"] - sequential["mean_pwm"]) <= MEAN_PWM_BUDGET


@pytest.mark.nightly
@pytest.mark.parametrize("seed", sorted(MEASURED))
def test_a_released_experiment_hands_the_solver_its_own_level_not_the_fans(das_example_cfg, seed):
    """Section 8 item 112, on the closed loop: what a finished experiment leaves behind.

    Until this item the supervisor released the experiment's channels, which dropped
    their integrator entries and re-initialised the solver bumplessly at *the PWM on the
    fan*. A channel released on its high level was handed that level as the solver's own
    starting point, and the next start took the inflated command as its base. The
    symptom in this very scenario, measured by reinstating the old release beside the
    new one (16 h, ``rich``, zone-wide arm):

    ==== ================= ================ ==================================
    seed mean PWM          qd1 mean PWM     qd1 lowest command over the run
    ==== ================= ================ ==================================
    2    0.4064 -> 0.3587  0.790 -> 0.492   0.326 -> 0.200 (``pwm_min``)
    3    0.2758 -> 0.2636  0.362 -> 0.299   0.200 -> 0.200
    4    0.3142 -> 0.2969  0.374 -> 0.304   0.200 -> 0.200
    ==== ================= ================ ==================================

    Worst true margin 4.586 / 4.893 / 4.528 degC before against 4.133 / 4.510 / 4.026
    after, zero limit violations either way: the enclosure is quieter *and* warmer by
    about the same amount, and it lands back on the 4.07 to 4.30 degC the same seeds keep
    with no experiment at all (item 102). That is the point -- the fans were elevated
    after the experiments, not during them.

    Asserted here without the old code, against :data:`OLD_RELEASE`: no channel is left
    with a floor above ``pwm_min``, which is exactly the ratchet (qd1's 0.326 above), and
    both the enclosure mean and qd1's own mean stay under what the old release spent. The
    floor clause only discriminates on seed 2 -- on seeds 3 and 4 qd1 reached ``pwm_min``
    under the old release too -- and the enclosure mean leaves margins of 0.012 and 0.017
    there, so qd1's mean is what carries those two seeds.
    """
    run = _run(_config(das_example_cfg, parallel=True), seed, ORDER)
    old = OLD_RELEASE[seed]
    assert run["starts"] > 0 and run["violations"] == 0
    # every channel comes all the way back down between experiments: no ratcheted floor
    assert run["ch_min"] == pytest.approx(
        dict.fromkeys(run["ch_min"], das_example_cfg.pwm_min), abs=TOL
    )
    assert run["mean_pwm"] <= old["mean_pwm"]
    assert run["ch_mean"]["qd1"] <= old["qd1"]
    assert run["worst_margin_c"] >= MARGIN_FLOOR_C


@pytest.mark.nightly
@pytest.mark.parametrize("seed", NIGHTLY_LONG_SEEDS)
def test_the_bays_second_gate_closes_on_observations_not_on_more_excitation(das_example_cfg, seed):
    """Section 8 item 111: what the bays whose ``rel_se(k)`` stays high actually lack.

    Measured over the three seeds at 16 h: the air block is **never** the blocker for a
    zone that closes windows (``rel_se(E)`` 0.011 to 0.052 against the 0.25 bound), and
    ``blocked`` names ``rel_se:k.<bay>`` for 3 to 5 bays of 15. Those bays are not
    confounded -- the correlation between a bay's ``g0`` and ``k`` is -0.05 to +0.01 on
    every bay and seed -- and their own ``Qn_z`` regressor is excited (bay ``pe_min``
    0.06 to 0.26 against the air block's own 0.08 to 0.21). What they lack is
    **observations**: ``se(k)`` lands in 0.04 to 0.22 whatever the bay, while the true
    ``k`` spans 0.22 to 0.75, so ``rel_se = se / |k|`` fails on the bays with the least
    airflow sensitivity to measure, on a fit no worse than their neighbours'.

    ``se(k)`` falls as ``1 / sqrt(excited windows)``, so the answer is time, and this
    test measures how much: the same schedule at 36 h instead of 16 h closes about 900 to
    1075 excited windows per bay, ``se(k)`` lands at 0.03-0.12, and the converged zones go
    from 6 of 12 at 16 h to 11 of 12 at 36 h (section 8 item 109's fix on top of this:
    before it, b03 and b10 -- the two bays with a redundant second proximal sensor --
    were reported ``swapped`` on nearly every tick, closing 0 to 22 windows in 36 h and
    holding z0 on seeds 3 and 4 back with them; fixed, both bays close as many windows as
    any other and z0 converges on both seeds). Amplitude cannot substitute: reaching the
    same gain that way needs about five times the ``Qn_z`` variance, a relative zone-flow
    swing of 0.55 to 1.1 against the 0.24 to 0.51 measured, which no ``ident_amplitude``
    in ``(0, 0.3]`` produces.

    The one zone that still does not converge at 36 h is z3 on seed 3, blocked on b15
    alone: its fitted ``k`` is 0.215, less than half the prior, so even an ``se`` of
    0.109 is half of it -- more observations narrow ``se`` but cannot inflate ``k``.
    """
    want = MEASURED_LONG[seed]
    long_run = _run(_config(das_example_cfg, parallel=True), seed, ORDER, hours=LONG_HOURS)
    assert long_run["violations"] == 0

    # 1. waiting works: the zones the longer run reaches, and more than 16 h reached
    assert set(long_run["converged"]) >= set(want["converged"]), {
        z: v["status"] for z, v in long_run["zones"].items()
    }
    assert set(long_run["converged"]) >= set(MEASURED[seed]["parallel"])

    # 2. the air block is not what holds a zone back: no zone that closes windows is
    #    blocked by an ``E``'s relative standard error
    for z, zone in long_run["zones"].items():
        if not zone["excited_windows"]:
            continue  # defensive: no zone is dead any more since item 109's fix
        assert not [r for r in zone["blocked"] if r.startswith("rel_se:E.")], (z, zone["blocked"])

    # 3. what does hold the rest back, named bay by bay -- and it is a bay's ``k``
    for z, bays in want["blocked_bays"].items():
        blocked = long_run["zones"][z]["blocked"]
        assert blocked, z
        assert {r.split(".", 1)[1] for r in blocked if r.startswith("rel_se:k.")} == set(bays), (
            z,
            blocked,
        )

    # 4. the mechanism: the standard error is what fell, and it fell with the windows
    for b, bay in long_run["bays"].items():
        if bay["excited_windows"] < WINDOWS_FLOOR:
            continue  # defensive: every bay clears this floor since item 109's fix
        assert bay["se"][f"k.{b}"] <= 0.12
        assert bay["rel_se"][f"k.{b}"] == pytest.approx(
            bay["se"][f"k.{b}"] / abs(bay["theta"][f"k.{b}"]), rel=1e-6
        )


#: Section 8 item 119, measured on this very scenario: zones converged per seed at
#: ``model_converged_bays_frac`` 1.0 (today's all-or-nothing rule), 0.75 and 0.5, and the
#: bays each converged zone was converged *without*. 6 of 12 zone-seeds at 16 h become 9
#: at 0.75 and 10 at 0.5; at 36 h 11 of 12 become 12 at 0.5 (z3 on seed 3, whose b15 has
#: a genuinely small ``k``, is the one the whole-zone rule never reaches).
PARTIAL_16: dict[int, dict[str, Any]] = {
    2: {
        0.75: {"converged": ("z0", "z2", "z3"), "without": {"z2": ("b10",)}},
        0.5: {"converged": ("z0", "z2", "z3"), "without": {"z2": ("b10",)}},
    },
    3: {
        0.75: {"converged": ("z0", "z1", "z2"), "without": {"z1": ("b05",)}},
        0.5: {"converged": ("z0", "z1", "z2"), "without": {"z1": ("b05",)}},
    },
    4: {
        0.75: {"converged": ("z0", "z1", "z3"), "without": {"z1": ("b05",)}},
        0.5: {
            "converged": ("z0", "z1", "z2", "z3"),
            "without": {"z1": ("b05",), "z2": ("b11", "b12")},
        },
    },
}
#: The same at 36 h and ``model_converged_bays_frac: 0.5``: every zone of every seed.
PARTIAL_LONG: dict[int, dict[str, Any]] = {
    2: {"converged": ("z0", "z1", "z2", "z3"), "without": {}},
    3: {"converged": ("z0", "z1", "z2", "z3"), "without": {"z3": ("b15",)}},
    4: {"converged": ("z0", "z1", "z2", "z3"), "without": {}},
}


@pytest.mark.nightly
@pytest.mark.parametrize("seed", sorted(PARTIAL_16))
def test_a_zone_may_converge_on_the_bays_that_have_informed_themselves(das_example_cfg, seed):
    """Section 8 item 119, on the same closed loop as the rest of this module.

    All-or-nothing is what item 111 left: at 16 h the air block is never the blocker and
    3 to 5 bays of 15 hold four zones back on their own ``rel_se(k)``, so one stubborn bay
    keeps a whole zone in ``learning`` for ever. ``model_converged_bays_frac`` lets the
    zone converge on its air block plus the bays that did inform themselves --
    :data:`PARTIAL_16` and :data:`PARTIAL_LONG` are what that reaches, 6 of 12 zone-seeds
    at 16 h becoming 9 at 0.75 and 10 at 0.5, and 11 of 12 at 36 h becoming 12 at 0.5.

    The safety half is asserted with it, and it is the point of the item: a zone converged
    this way is **not** trusted as a fully informed one. It publishes ``partial``, it names
    the bays in ``uninformed`` and in ``blocked``, and the ``k`` the DAS MPC plans those
    bays with is their own lower confidence bound, never the fitted value -- an
    under-estimated ``k`` makes the solver believe airflow helps that bay less than it
    does, so it runs the fans higher.
    """
    want = PARTIAL_16[seed]
    strict = set(MEASURED[seed]["parallel"])
    for frac in (0.75, 0.5):
        cfg = dataclasses.replace(
            _config(das_example_cfg, parallel=True), model_converged_bays_frac=frac
        )
        run = _run(cfg, seed, ORDER)
        assert run["violations"] == 0
        assert run["worst_margin_c"] >= MARGIN_FLOOR_C
        got = set(run["converged"])
        assert got >= set(want[frac]["converged"]), (frac, run["converged"])
        # a lower fraction can only ever converge more zones, never fewer
        assert got >= strict, (frac, run["converged"], sorted(strict))
        for z, zone in run["zones"].items():
            uninformed = tuple(zone["uninformed"])
            if z in got:
                assert uninformed == want[frac]["without"].get(z, ()), (frac, z, uninformed)
                assert zone["partial"] is bool(uninformed)
                # the reader is told, in the rule's own words, which bays it converged
                # without -- ``blocked`` is empty exactly for a *whole* converged zone
                assert bool(zone["blocked"]) is bool(uninformed), (z, zone["blocked"])
                for b in uninformed:
                    assert any(r == f"rel_se:k.{b}" for r in zone["blocked"]), zone["blocked"]
                    bay = run["bays"][b]
                    assert bay["k_used"] < bay["theta"][f"k.{b}"], (b, bay["k_used"])
                    assert bay["k_used"] == pytest.approx(
                        max(
                            thermal.PARAMETERS["k"].lo,
                            bay["theta"][f"k.{b}"]
                            - cfg.model_partial_k_sigmas * bay["se"][f"k.{b}"],
                        )
                    )
            else:
                assert zone["partial"] is False
        # and a bay of a zone that converged *whole* is planned at its fitted value
        for b, bay in run["bays"].items():
            zone = run["zones"][bay["zone"]]
            if b not in zone["uninformed"]:
                assert bay["k_used"] == pytest.approx(bay["theta"][f"k.{b}"])


@pytest.mark.nightly
@pytest.mark.parametrize("seed", sorted(PARTIAL_LONG))
def test_partial_convergence_closes_the_last_zone_the_long_run_leaves(das_example_cfg, seed):
    """The 36 h picture of item 119. Item 111 measured the same schedule at 36 h reaching
    11 of 12 zones, the one hold-out being z3 on seed 3, blocked on b15 alone -- a bay
    whose fitted ``k`` is under half the prior, so more observations narrow ``se`` but
    cannot inflate ``k``. At ``model_converged_bays_frac: 0.5`` that zone converges on its
    other two bays and says it did, which is the whole of the policy question."""
    want = PARTIAL_LONG[seed]
    cfg = dataclasses.replace(
        _config(das_example_cfg, parallel=True), model_converged_bays_frac=0.5
    )
    run = _run(cfg, seed, ORDER, hours=LONG_HOURS)
    assert run["violations"] == 0 and run["worst_margin_c"] >= MARGIN_FLOOR_C
    assert set(run["converged"]) >= set(want["converged"]), {
        z: v["status"] for z, v in run["zones"].items()
    }
    assert set(run["converged"]) >= set(MEASURED_LONG[seed]["converged"])
    for z, bays in want["without"].items():
        assert tuple(run["zones"][z]["uninformed"]) == bays
        assert run["zones"][z]["partial"] is True
    whole = [z for z in run["converged"] if z not in want["without"]]
    assert all(run["zones"][z]["partial"] is False for z in whole)


#: Section 8 item 120, measured over 12 h of this scenario at the shipped
#: ``ident_amplitude: 0.15`` under ``ident_levels: symmetric``: starts taken and the mean
#: PWM given up per tick, ``fixed`` against ``headroom``. Since item 112 the solver parks
#: the channels near ``pwm_min`` between experiments, which is exactly where a symmetric
#: pair does not fit, so the ``band:`` precondition refuses most of the programme; sizing
#: the telegraph to the room the channel really has runs most of them instead, and gives
#: up *less* per experiment on the two seeds where the schedule was starved:
#:
#: * seed 2 -- 11 starts / 0.00724 per tick against 12 / 0.00847 (per start 0.000658 ->
#:   0.000706, +7 %);
#: * seed 3 -- 4 / 0.00151 against 10 / 0.00289 (0.000378 -> 0.000289, -24 %);
#: * seed 4 -- 7 / 0.00662 against 12 / 0.00682 (0.000946 -> 0.000568, -40 %).
#:
#: The deepest dip any single tick took also falls where the sizing bites: 0.1500 on every
#: ``fixed`` arm against 0.1500 / 0.1331 / 0.1496. ``band:`` refusals fall from 848 / 5513
#: / 3604 to 171 / 1771 / 447 -- they do not reach zero, because under ``headroom`` the
#: refusal is no longer "a level leaves the band" but "the cut left no usable swing", and
#: the channels the solver parks *high* still have none (review finding).
SYM_HOURS = 12.0
SYM_STARTS: dict[int, dict[str, int]] = {
    2: {"fixed": 11, "headroom": 12},
    3: {"fixed": 4, "headroom": 10},
    4: {"fixed": 7, "headroom": 12},
}


@pytest.mark.nightly
@pytest.mark.parametrize("seed", sorted(SYM_STARTS))
def test_sizing_the_telegraph_to_the_headroom_runs_the_starts_symmetric_refuses(
    das_example_cfg, seed
):
    """Section 8 item 120, on the closed loop. Under ``symmetric`` the amplitude is
    cooling **given up**, and the ``band:`` precondition refuses any start whose low level
    would fall through ``pwm_min``. Since item 112 that is most of them: 11, 4 and 7 of
    the 12 starts offered over 12 h on the three seeds.

    ``ident_amplitude_mode: headroom`` sizes each channel's step to the smallest that
    reaches the PE bound and cuts the pair into the band -- a symmetric pair on the rail
    keeps the dip it had room for and gives up less cooling. Asserted here: every start
    runs, none is refused for the band, and no channel is ever further from the solver's
    own command than ``ident_amplitude`` **in either direction** -- the owner's accepted
    amplitude is a cap the sizing only ever undercuts, above the command as well as below
    it (review finding: only the dip was pinned).
    """
    want = SYM_STARTS[seed]
    base = dataclasses.replace(
        _config(das_example_cfg, parallel=True),
        ident_levels="symmetric",
        ident_amplitude=das_example_cfg.ident_amplitude,  # the shipped 0.15
    )
    fixed = _run(base, seed, ORDER, hours=SYM_HOURS)
    head = _run(
        dataclasses.replace(base, ident_amplitude_mode="headroom"), seed, ORDER, hours=SYM_HOURS
    )
    assert fixed["violations"] == 0 and head["violations"] == 0
    assert head["worst_margin_c"] >= MARGIN_FLOOR_C
    assert fixed["starts"] <= want["fixed"] and head["starts"] >= want["headroom"]
    assert head["starts"] > fixed["starts"], (head["starts"], fixed["starts"])
    # the refusal does not go away, it changes question: under ``headroom`` the levels are
    # cut into the band, so ``band:`` refuses the channels whose cut left no usable swing
    # -- the ones the solver parks high -- instead of every channel parked near a rail
    assert fixed["band_refusals"] > 0, fixed["band_refusals"]
    assert head["band_refusals"] < 0.5 * fixed["band_refusals"], (
        head["band_refusals"],
        fixed["band_refusals"],
    )
    # the dip is the accepted worst case and stays one: no tick gives up more than the cap
    for run in (fixed, head):
        assert run["max_dip"] <= base.ident_amplitude + TOL, run["max_dip"]
    assert head["max_dip"] <= fixed["max_dip"] + TOL
    # and the sizing never puts a channel further *above* the live command than the fixed
    # mode does either -- what the experiment itself may plan is capped at
    # ``ident_amplitude`` both ways and pinned in ``tests/test_ident_experiment.py``
    assert head["max_rise"] <= fixed["max_rise"] + TOL, (head["max_rise"], fixed["max_rise"])
