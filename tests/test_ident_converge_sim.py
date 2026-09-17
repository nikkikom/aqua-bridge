"""Closed-loop identification on the daemon's own path (section 8 items 102, 111, 112).

Nightly, because each case runs the real ``Loop`` + ``Supervisor`` + ``control/ident``
over the DAS truth simulator (``sim/das.py``, ``rich`` preset) for 16 simulated hours,
twice: once with ``ident_parallel: true`` and once with ``false``, on the same plant and
the same seed. Three tests:

* item 102 -- the zone-wide schedule against a one-channel-at-a-time round robin;
* item 112 -- what a finished experiment leaves on the fans, now that the release hands
  the solver its own level instead of the level on the fan;
* item 111 -- what the bays whose ``rel_se(k)`` stays above the bound actually lack,
  measured by running the same schedule for 36 h instead of 16 h.

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
group while the solver carries the rest excites one direction at a time -- the round
robin's own ``pe_diag`` shows it, one group of a zone at 0.004 to 0.04 while another sits
at 0.18 to 0.43. It is *not* true that it never clears the bound: the round robin peaks
at 0.15 to 0.22 on some zone of every seed (asserted below). What the zone-wide
excitation buys is that the bound is *held*.

Observed over 16 h, zone-wide against the round robin (converged zones; the zone-wide
arm's ``pe_min`` peak and its value at the end of the run; mean PWM of each arm). These
numbers moved with section 8 item 112: with the fans no longer left elevated after each
experiment the channels park lower, where a fixed 0.25 step is a *larger* relative
airflow swing, so ``pe_min`` roughly doubles on the zones that converge -- and on seed 3
z1 loses the race on one bay's ``rel_se(k)`` instead of winning it.

* seed 2 -- z0 and z3 against nothing; 0.282 and 0.238 peak, 0.216 and 0.205 at the
  end; 467 and 471 excited windows; mean PWM 0.3587 against 0.3660;
* seed 3 -- nothing against nothing (z0 closes no window at all, item 109), pinned
  exactly rather than as a lower bound, so that case stays a no-regression test instead
  of a tautology; mean PWM 0.2636 / 0.2387;
* seed 4 -- z3 against nothing; 0.316 peak, 0.201 at the end; 468 windows;
  0.2969 / 0.2794.

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
#: **no** regression window in 16 h: on seed 3 bay b03's redundant proximal pair is read
#: as ``swapped`` on nearly every tick and ``model_reset_on_swap`` resets z0's air
#: accumulator with it, so that zone can never converge for reasons no excitation
#: reaches. It is pinned here so a zone going dead is visible instead of silent.
MEASURED: dict[int, dict[str, tuple[str, ...]]] = {
    2: {"parallel": ("z0", "z3"), "sequential": (), "dead": ()},
    3: {"parallel": (), "sequential": (), "dead": ("z0",)},
    4: {"parallel": ("z3",), "sequential": (), "dead": ()},
}
#: What 36 h of the *same* schedule reaches, and what still blocks each zone that does
#: not converge even then (section 8 item 111). Every remaining blocker but one is a bay
#: the estimator reports ``swapped`` on nearly every tick (b03, b10 -- section 8 item
#: 109), which closes no regression window and which no excitation can reach.
MEASURED_LONG: dict[int, dict[str, Any]] = {
    2: {"converged": ("z0", "z1", "z3"), "blocked_bays": {"z2": ("b10",)}},
    3: {"converged": ("z1",), "blocked_bays": {"z0": ("b03",), "z2": ("b10",), "z3": ("b15",)}},
    4: {"converged": ("z1", "z2", "z3"), "blocked_bays": {"z0": ("b03",)}},
}
LONG_HOURS = 36.0
#: Floors on the converged zones, from the runs above (peak 0.24-0.32, end 0.20-0.22,
#: 467-471 windows) rather than from the ``converged`` rule's own 0.05 / 30, which
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
        for ch, value in r.cmd.pwm.items():
            ch_min[ch] = min(ch_min.get(ch, 1.0), float(value))
            ch_mean[ch] = ch_mean.get(ch, 0.0) + float(value)
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
        "bays": summary.get("bays") or {},
        "worst_margin_c": worst_margin,
        "violations": violations,
        "mean_pwm": pwm_sum / max(ticks, 1),
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
    #    than the zone-wide arm. A seed whose zone-wide arm converged *nothing* (seed 3
    #    since item 112) is pinned exactly, both ways: ``>= set()`` is a tautology, so
    #    without this the case would assert nothing at all about the behaviour its name
    #    claims and only seeds 2 and 4 would separate the arms.
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
    # one channel at a time does clear the bound in bursts -- what it cannot do is hold
    # it, so a zone it converges ends below what the zone-wide arm holds there.
    assert max(sequential["pe_max"].values()) > thermal.PE_MIN
    for z in sequential["converged"]:
        assert sequential["zones"][z]["pe_min"] < parallel["zones"][z]["pe_min"]

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
@pytest.mark.parametrize("seed", sorted(MEASURED_LONG))
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
    test measures how much: the same schedule at 36 h instead of 16 h closes 900 to 1000
    excited windows per bay instead of 390 to 450, ``se(k)`` falls from 0.10-0.16 to
    0.07-0.11, and the converged zones go from 3 of 12 to 7 of 12. Amplitude cannot
    substitute: reaching the same gain that way needs about five times the ``Qn_z``
    variance, a relative zone-flow swing of 0.55 to 1.1 against the 0.24 to 0.51
    measured, which no ``ident_amplitude`` in ``(0, 0.3]`` produces.

    Of the five zones that still do not converge at 36 h, four are blocked by b03 or b10
    -- the two bays with a redundant second proximal sensor, reported ``swapped`` on
    nearly every tick, which close 0 to 22 windows in 36 h (section 8 item 109). The
    fifth is b15 on seed 3, whose fitted ``k`` is 0.215: less than half the prior, so
    even an ``se`` of 0.109 is 50 % of it.
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
            continue  # item 109's dead zone, named by MEASURED
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
            continue  # item 109's swap-reset bays close almost none
        assert bay["se"][f"k.{b}"] <= 0.12
        assert bay["rel_se"][f"k.{b}"] == pytest.approx(
            bay["se"][f"k.{b}"] / abs(bay["theta"][f"k.{b}"]), rel=1e-6
        )
