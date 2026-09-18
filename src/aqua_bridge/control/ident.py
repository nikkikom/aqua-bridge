"""Active identification experiments on fan groups (DAS plan section 5, "Active experiments").

Pure state machine: every function here is a function of its arguments, keeps
its state in plain JSON dicts, reads no clock (time is the observation clock
``obs.ts`` handed in through :class:`TickFacts`), draws no randomness (the hold
times come from a seeded LFSR) and does no I/O. The
:class:`~aqua_bridge.control.supervisor.Supervisor` drives it: it starts an
experiment on an explicit intent (``POST /api/ident``, MQTT ``cmd/ident``),
advances it once per loop tick from what that tick did, and expresses the
experiment's PWM levels as ordinary overrides. ``compose`` therefore rate
limits and clamps them, fallback beats them exactly as it beats a human
override, and ``mpc.step`` and the loop do not know an experiment exists. The
thermal model learns from experiment ticks like from any trusted tick; the
experiment only supplies excitation.

Unit of excitation and sequence
-------------------------------
The unit is a **fan group** (``fans.<channel>.group``; a channel without a
group is a group of its own, named like the channel). ``start`` on a group runs
the group level first (every channel of the group together) and then, for a
group of several channels, each channel alone, so the thermal model can split
the group's gain between them; ``start`` on a channel runs that channel alone
(``ident_parallel`` widens both to the target's whole zones, below).
``ident_max_duration_s`` is split evenly over the phases. While one channel of a
group runs alone the group's other channels are held at the group's base (a
solver-driven sibling would move against the experiment and make the two
regressors collinear again; that base follows the solver's demand like every
other level below, and a held sibling is floored by the solver's own command on
every tick like every other experiment channel, so holding one never gives less
cooling than the solver asks for).

Excite a whole zone at once (``ident_parallel``, default false)
----------------------------------------------------------------
One group at a time is the wrong unit for the *thermal model*. Its air block per
zone regresses one airflow regressor per fan group of that zone
(:mod:`aqua_bridge.control.thermal`, *PE monitor*), and its convergence rule asks
the smallest eigenvalue of their information matrix to pass ``PE_MIN``: **every**
group of the zone has to move, independently of the others, inside the monitor's
~30-window memory. A schedule that telegraphs one group while the solver carries
the rest excites one direction of that matrix at a time, and the others only as
far as the solver happens to move them: measured over 16 h on the DAS example, a
round robin over **every** channel does push a zone's ``pe_min`` past ``PE_MIN``
now and then -- peaks of 0.15 to 0.22 against the 0.05 bound -- but it does not
hold it there, and over three seeds it latches ``converged`` on no zone of twelve.
A coded zone-wide phase peaks at 0.24 to 0.32 on the zones it converges and still
sits at 0.20 to 0.22 at the end of the run (section 8 item 102 has the eigenvalues
and the per-seed numbers, item 112 what moved them).

``ident_parallel: true`` changes the channel set and the schedule, nothing else:

* the target's channel set grows to every channel of the zones that **list** the
  target's channels (``ZoneLayout.zone_channels``; the coupled zones are not added
  -- they are served, and still checked and aborted on, but exciting them is not
  what makes the target's zone identifiable);
* the experiment is one phase over all of them, and each channel gets its **own**
  telegraph -- its own start level and its own hold draws, from an LFSR seeded per
  channel from ``ident_seed`` -- instead of the whole phase sharing one level.

Everything else is untouched: the levels are the same two levels around the same
anchor, ``above`` still never commands less cooling than the solver, the same
preconditions are checked (over the larger channel set, so a start is refused when
any of those channels is saturated or would leave the band), the same envelope and
the same abort list run on the same served zones, and ``compose`` still floors,
rate-limits and clamps every override. With ``ident_parallel: false`` the schedule,
the channel set and the experiment dict are what they were.

What it costs: under ``above`` a channel spends about half an experiment a step
above the anchor, so ``G`` channels are raised at once instead of one. *Per zone
identified* that is arithmetic-neutral -- one zone-wide experiment does the work
of the ``G`` sequential ones its zone needs, in a ``G``-times shorter window --
but that is arithmetic, not what was measured: in the A/B of
``tests/test_ident_converge_sim.py`` both arms run the same 16 h of experiment,
and the zone-wide arm costs **-0.007 to +0.025 mean PWM** over the whole run -- since
section 8 item 112 it is the quieter of the two on one seed of three. ``above``
never commands less cooling than the solver *on the tick*, which is a per-tick
statement and not a claim about two 16 h trajectories: the zone-wide arm ends with
the larger worst true margin on two seeds of three and 0.27 degC below the round
robin on the third, both above 4 degC with no limit crossed.

What it costs under ``symmetric``: more than it used to, and this is the one
accepted worst case the key widens. ``compose`` floors an experiment channel at
its own solver command minus ``ident_amplitude``; before, at most one group of a
zone was at its low level at a time (a group's siblings are held at base, above),
so that is all the cooling a tick could give up. A coded phase has every channel
of the zone drawing its own level, so all of them can be low on the same tick --
on the scenario's knobs that is 10 % of the ticks of a three-channel phase and
20 % of a two-channel one, with the whole zone a step under the solver's command.
The envelope, the abort list and the per-tick floor are unchanged, and the
default ``above`` gives up no cooling at all; ``symmetric`` with
``ident_parallel`` is the combination to weigh against the drives' own margin.

Each channel's base ``u_base`` starts as the solver's command for it on the last
tick before the start. The two levels are ``ident_levels``:

* ``above`` (default): ``u_base`` and ``u_base + ident_amplitude``. The experiment
  never commands less than the solver does (the safe direction);
* ``symmetric`` (owner opt-in): ``u_base - A`` and ``u_base + A``, so the low level
  is a deliberate dip of at most ``ident_amplitude`` below the solver.

How far a channel can move the PE monitor from where it sits (section 8 item 110)
----------------------------------------------------------------------------------
``pe_min`` is a **relative** measure: the thermal model's monitor normalises each fan
regressor by its own running mean, so the ``converged`` rule wants a relative airflow
variation of ``sqrt(PE_MIN)`` = 0.224 in the least excited direction. A 50/50 telegraph
at base ``u`` with dead band ``d`` gives ``A / (2 (u - d) + A)`` under ``above``, so it
needs ``A >= 0.576 (u - d)``: a channel parked past about 0.62 PWM cannot clear the bound
at any amplitude the ``(0, 0.3]`` cap allows, and one parked at ``pwm_min`` clears it with
0.06 while the schedule spends whatever ``ident_amplitude`` says.

That normalisation stays as it is. It is what makes the monitor read as *information*
rather than as PWM: a fan already near ``pwm_max`` really does move proportionally less
extra air per unit of PWM, and dividing by something else would relabel an uninformative
channel, not inform it. What changes is that the arithmetic is no longer invisible.
:func:`excitation` publishes it per channel -- ``rel_swing``, the ``pe_reach`` it implies,
whether that clears :data:`~aqua_bridge.control.thermal.PE_MIN` at the configured
``ident_amplitude`` (``excitable``) and whether any allowed amplitude would
(``excitable_at_cap``) -- in ``snapshot().extra["experiment"]`` and in the start log line,
so a channel that cannot be excited is **visible** instead of leaving its zone silently
pending, and the owner is told which of the two remedies applies: a larger amplitude, or
waiting for the solver to park the channel lower.

``pe_reach`` is an **upper bound**, not a prediction. The monitor accumulates one
sin^2-weighted mean of the airflow regressor per ``model_window_s`` block, so it sees the
telegraph's own two levels only while a regression window fits inside one hold; a window
that spans a switch averages them and reads less. :func:`holds_cover_window` says whether
``min(ident_hold_s) >= model_window_s`` holds, and the status publishes it beside the
reach, so a schedule that cannot deliver what it promises says so. ``excitable`` is
therefore a necessary condition: ``false`` is a proof that the channel cannot inform its
zone from this base, ``true`` is permission for the schedule to try. The measured half is
the thermal model's own ``pe_diag``, the diagonal beside ``pe_min``, which names the group
that is short after the fact, and its ``blocked`` list, which names the gate.

Sizing the telegraph to that headroom (``ident_amplitude_mode``, section 8 item 120)
----------------------------------------------------------------------------------------
``fixed`` (the default) spends ``ident_amplitude`` on every channel whatever room it has,
which is what shipped before. ``headroom`` spends the **smallest** amplitude in
``(0, ident_amplitude]`` whose own ``rel_swing`` reaches ``ident_pe_aim * sqrt(PE_MIN)``
from where that channel is parked -- falling back to ``ident_amplitude`` when even that
falls short, so no channel is ever excited less than ``fixed`` excites it -- and places
the two levels **inside** ``[pwm_min, pwm_max]`` with the room the channel really has on
each side.

Why it is worth a key under ``symmetric`` and not under ``above``. Under ``above`` the
amplitude is cooling *added* and the solver takes it straight back off its own demand, so
sizing down buys 0.004 mean PWM (measured over 16 h on three seeds, item 110) -- nothing.
Under ``symmetric`` the amplitude is cooling *given up*: ``compose`` floors an experiment
channel at its own solver command minus the planned dip, so a smaller step comes directly
off the one accepted worst case in this module. The placement matters there too, and in
both directions: a channel parked within ``ident_amplitude`` of ``pwm_min`` -- where item
112 leaves most of them between experiments -- has its symmetric pair refused outright by
the ``band:`` precondition today, while a pair cut at ``pwm_min`` runs the experiment at
the dip the channel had room for. Neither level is ever further from the base than
``ident_amplitude``, in either direction: that cap is the owner's ceiling on what one
experiment may move the fans by, and sliding a pair off a rail to keep its swing would
spend up to twice it *above* the solver's command, on a channel the solver had parked low
for quiet. So a level the band will not take is cut at the rail and ``excitable`` says the
swing is short, and a swing too short to reach the PE monitor's own floor is refused at
the start as ``band:`` -- the refusal the placement replaced, asked of the swing that is
left rather than of the levels.

``ident_require_excitable`` (default false) turns it into a refusal: ``check_start`` then
rejects a start whose channel cannot clear the bound (``not_excitable:<ch>``), the way
``band:`` and ``saturated:`` reject. It is off by default deliberately -- such a run still
informs the ``E`` split and the bays' ``g0`` / ``k``, and a zone-wide start is widened to
channels the solver parks high, so refusing gives up the zones that do converge.

A start is refused when a level would leave ``[pwm_min, pwm_max]`` (``band``), so no level
is ever clipped at the start; under ``ident_amplitude_mode: headroom`` the levels are cut
into the band by construction and ``band`` refuses the starts whose remaining swing cannot
reach the PE monitor's floor instead. The sequence is a two-level random
telegraph signal: every phase starts on the high level, the level alternates
after each hold, and each hold is drawn from ``ident_hold_s`` by a 16-bit Galois
LFSR (taps ``0xB400``) seeded from ``ident_seed``; the generator runs on across the
phases, and the last hold of a phase is cut at the phase end. The schedule (which
channels, when the level switches) is computed once at the start, in plain JSON,
and is the same for the same config, target and base commands; only the levels
themselves move, below.

Levels re-planned from the live demand (``ident_replan``, default true)
-----------------------------------------------------------------------
**A rise in the solver's demand wins over the experiment's plan, always: the
experiment never holds a channel below what the solver asks for.** With a base
frozen at the start (``ident_replan: false``, the Zero W behaviour) a solver that
later wants more cooling on a channel under test is held back until the envelope
aborts the experiment; re-planning follows it instead, which is what the Zero
2 W's compute allows on every tick (owner, 2026-09-14; the measured cost is in
PROJECT.md section 8.4).

Three pieces, all three on only with ``ident_replan``:

* **The anchor follows the demand up, de-biased.** :func:`advance` reads the
  solver's want for the tick that just ran (``diagnostics["target_pwm"]``: before
  the rate limit and the clamp) and raises each channel's base to it, then
  re-derives that channel's two levels around the new base and clamps them into
  ``[pwm_min, pwm_max]``.

  That want is **not** free of the experiment's own influence, and taking it raw
  would ratchet. The DAS MPC's objective carries ``weight_dpwm ||u_0 - prev||^2``
  and ``prev`` is what was last put on the fan, which during an experiment is the
  experiment's own level: the high level raises the want, the want raises the
  anchor, the anchor raises the high level (measured on the DAS example, all else
  equal: ``prev`` 0.40 gives a want of 0.398, ``prev`` 1.00 gives 0.962 -- a slope
  near 1). So the echo of the experiment's own level comes off the want first,

      ``wanted = demand - max(0, min(prev, own) - plan_base)``

  with ``prev`` from ``diagnostics["prev_pwm"]`` (the very PWM that penalty pulls
  toward) and ``own`` the override the experiment itself had in force on that tick.
  Bounding the echo by ``own`` is what keeps the two reasons for a fan above the
  anchor apart: the experiment put it there (an echo, it comes off) or the solver's
  own command floored it there (demand, it stays -- otherwise the anchor would stick
  wherever the floor found it and the levels would stop tracking the solver at all).
  On a tick the experiment ran at its anchor -- every low tick under ``above``, every
  held sibling, every tick under ``symmetric`` -- nothing is subtracted and the
  anchor follows the want whole; on a high tick the level's own excess comes off,
  which under-follows by however much of the pull the solver did not exert (the
  slope is below 1), and that is the safe direction: the composed command is floored
  by the solver's own on every tick regardless. The PI solver has no such term at
  all (its integrator is its own state), so in practice nothing is taken off it --
  its ``prev`` is at the anchor whenever the echo would matter.
* **The anchor rises at once, falls slowly.** A rise is taken on the tick it
  appears. A fall is taken only at a level switch, at most ``d_pwm_max`` at a time,
  and never below ``base`` (the frozen base of the start): a hold keeps the level
  it started with, so its excitation is preserved, while a transient peak in the
  demand is released within a hold or two instead of pinning the channel -- and
  every sibling held at its anchor -- at that peak for the rest of
  ``ident_max_duration_s``. A channel whose demand or ``prev`` this tick is missing
  or not finite keeps its anchor.
* **The composed command is floored by the solver's.** The anchor can only act on
  the next tick, so :meth:`~aqua_bridge.control.supervisor.Supervisor.compose`
  floors an experiment channel's override with the solver's own command for that
  tick (``max(override, mpc_cmd.pwm[ch])``, before the usual rate limit and clamp).
  That closes the one-tick gap exactly: on no tick does an experiment channel get
  less than the controller would have put on it, ``symmetric`` excepted -- there
  the floor is the owner-accepted dip, ``mpc_cmd.pwm[ch] - ident_amplitude``. The
  floor follows the **running** experiment's own plan (:func:`planned_dip`), not
  the live config, so a config rebuilt mid-experiment cannot take it away.

What it costs the identification: the levels move with the demand, so the step
sizes change. The telegraph's own step stays ``ident_amplitude`` except where a
level clamps at ``pwm_max`` (then the high level is squeezed and, under ``above``
with the demand at ``pwm_max``, the excitation stops until the demand falls back);
a rise of ``d`` inside a hold makes the next switch a step of ``A - d`` (down) or
``A + d`` (up) instead of ``A``, and a fall is confined to the switches, at most
``d_pwm_max`` each. The base drift is slow next to the 60-180 s holds, so the
regressors keep the high-frequency content the fit lives on; the sim numbers are
in PROJECT.md section 8.4.

What the re-plan does **not** touch is the abort decision of a tick:
:func:`advance` decides that tick's aborts from that tick's estimates, with the
same thresholds in the same order, *before* it re-plans the next tick's levels, so
the re-plan cannot weaken the decision itself, and a re-planned level is never
below the frozen plan's level for the same tick, so no tick of an experiment runs
the enclosure warmer than the frozen plan would have. Across ticks it does move
the aborts, and that is the point of the item: a channel followed up cools its
bay, so an excursion the frozen plan would have ended on the envelope can run to
completion instead. Fewer envelope aborts are the intended outcome, not a
weakened rule.

Time: the start is armed between ticks; offset 0 is the tick after the last
recorded one (``last ts + dt``), or one tick later when the loop has already taken
the plan for that tick (``start``'s ``skip_ticks``, set by the supervisor; section 8
item 20). After every tick the machine checks that tick and computes the overrides
for the next one at offset ``ts + dt - start``. The experiment completes when that
offset reaches the duration.

Preconditions (:func:`check_start`, each failure has a named reason)
--------------------------------------------------------------------
``ident_enabled`` and a DAS config are checked by the supervisor, which also
refuses a second start while one runs. Here:

* ``control_mode`` -- the control mode is ``auto`` and no human override is set;
* ``no_tick`` / ``mode:<mode>`` -- the last command exists and its mode is ``auto``
  (not ``saturated``, ``degraded`` or ``fallback``);
* ``saturated:<ch>``, ``no_command:<ch>``, ``band:<ch>`` -- per channel of the target;
  ``not_excitable:<ch>`` -- only with ``ident_require_excitable``: the channel's
  telegraph cannot reach the thermal model's PE bound from where the solver parked it
  (:func:`excitation`), so the run could not inform that zone's air block;
  ``fan_stall:<ch>`` -- on any channel;
* ``settle:<zone>`` -- every zone the target serves (``ZoneLayout.served``: the zones
  listing one of its channels plus their declared ``coupled_to``) has been trusted
  and fault-free for ``ident_settle_s`` (tracked by :func:`track` from every tick);
* ``sensor_lost:<zone>`` -- no zone the target serves has a zone-air or bay sensor
  group without a trusted, confirmed member (:func:`lost_sensor_zones`, from
  ``diagnostics["sigma_floor"]``). Under ``zones.trust_rule: sigma`` a zone with a
  lost sensor stays trusted while the estimator's sigma is still small, so without
  this the enclosure would be excited while blind on one node;
* ``bay_unknown:<bay>`` / ``bay_transition:<bay>`` -- no bay of those zones is
  ``unknown``, has an occupancy change pending (``pending_empty_s``,
  ``pending_occupied_ticks`` or the occupancy debounce ``pending_unknown_s`` of the
  estimator) or changed occupancy less than ``estimator.bay_settle_s`` ago;
* ``calibrating:<bay>`` -- no bay of those zones has a SMART calibration still in its
  first 20 samples (:data:`~aqua_bridge.control.estimator.CAL_MIN_SAMPLES`);
* ``start_band:<bay>`` -- every occupied or unknown bay of those zones has
  ``T_hat <= soft + ident_start_band_c``, plus the running envelope below.

Settle timers across a restart
-------------------------------
The tracker is per-zone seconds, not wall times: :func:`settle_snapshot` hands the
model store ``{zone: seconds settled so far}``,
:func:`aqua_bridge.control.persist.apply_seed` subtracts the daemon's outage from
them (and drops the lot when the outage is longer than
``ident_settle_resume_max_gap_s``, or when the file is stale or the wall clock is
behind it), and :func:`resume_tracker` installs what is left as a *credit*.
:func:`track` spends a zone's credit on the first tick that zone is trusted and
fault-free again, minus the time it took to get there, so no second the daemon did
not observe is ever counted as settled. A zone that never comes back never spends
its credit.

Envelope (checked on every tick while running, and at start)
-------------------------------------------------------------
For every occupied or unknown bay of the served zones, on the estimates of the
tick (``diagnostics["estimates"]``: ``t_c``, ``margin_c = k sigma``, ``soft_c``,
``hard_c``, ``limit_c``), with ``upper = T_hat + k sigma``:

* ``envelope:<bay>``   -- ``T_hat > soft + ident_max_over_c`` (3.0 degC, owner-accepted);
* ``hard:<bay>``       -- ``T_hat > hard`` (always);
* ``abort_temp:<bay>`` -- ``upper >= limit - ident_abort_below_limit_c``: the base plan's
  absolute ``ident_abort_temp_c`` becomes per drive class (the bay's limit in force),
  so there is no ``ident_abort_temp_c`` key;
* ``no_estimate:<bay>`` -- the bay has no finite estimate.

``k sigma`` is counted **once** (section 8 item 53). ``soft`` and ``hard`` already
subtract it, so the soft and hard rules read ``T_hat``, and only the absolute rule,
which is measured against the raw limit, reads ``upper``. ``T_hat <= hard`` therefore
means ``T_hat + k sigma <= limit``, the 2-sigma statement the plan intends.

Which rule binds, and what item 53 moved. Put every rule on ``upper``, so they compare:

* ``envelope`` fires at ``upper > limit - comfort_c + ident_max_over_c``;
* ``abort_temp`` at ``upper >= limit - ident_abort_below_limit_c``;
* ``hard`` at ``upper > limit``.

``abort_temp`` is therefore always stricter than ``hard``, but the **soft envelope is
the rule that binds first** whenever ``ident_max_over_c < comfort_c -
ident_abort_below_limit_c``, which is the case for every class of
``config.example-das.yaml`` (``ident_max_over_c`` 3.0, ``ident_abort_below_limit_c``
1.0): the envelope fires at ``upper`` 48.0 / 58.0 / 63.0 degC for hdd / ssd_sata /
nvme, the absolute abort only at 49.0 / 64.0 / 69.0. The absolute abort binds instead
when ``ident_max_over_c >= comfort_c - ident_abort_below_limit_c`` -- for instance the
``quiet`` preset on hdd, which narrows ``comfort_c`` to 3.0.

So item 53 did move the operational abort point, by exactly ``k sigma``, on the rule
that usually binds: the envelope. That is the whole point of the item -- the ``k sigma``
it dropped was being counted a second time inside a reference that had already
subtracted it. What did **not** move is the backstop: ``abort_temp`` is bit-identical to
before item 53, so an experiment still cannot take a drive's ``upper`` to within
``ident_abort_below_limit_c`` of its limit whatever the envelope allows, and
``ident_levels: above`` never commands less cooling than the solver asked for at the
start in the first place. What the item bought is that a settled enclosure is no longer
refused at start and no longer sits on the abort edge: PI-like DAS regulates ``T_hat``
to ``soft`` (section 8.5 item 1) and the DAS MPC rides ``T_hat = soft`` too, so both now
start with the whole ``ident_start_band_c`` / ``ident_max_over_c`` band in hand.

Abort list (:func:`advance`; abort = release the override)
----------------------------------------------------------
``fallback`` (a fallback tick or a tick without a solver command, e.g. a
controller error), ``degraded`` (any zone in fault anywhere), ``apply_failed``
(:data:`APPLY_FAILURES_ABORT` consecutive failed writes), ``clock`` (no finite ``ts``
or ``ts`` running backwards), ``duration`` (``ts`` beyond the duration plus one tick:
a clock jump), the envelope reasons above, ``zone_untrusted:<zone>`` and
``bay_unknown:<bay>`` in a served zone, ``fan_stall:<ch>`` on an experiment channel,
``sensor_lost:<zone>`` in a served zone, a stop intent (``stop``) and any human
intent (``human_intent:<kind>``, by the supervisor). A daemon restart never resumes a
running experiment: it lives only in the supervisor's memory. The settle timers do
survive one, through the model store (above).

Release: the solver keeps its own integrator (section 8 item 112)
-----------------------------------------------------------------
An experiment is an override *after* ``mpc.step``, so the solver ran normally on every
one of its ticks and the integrator at the release is its own state, advanced throughout
and never re-seeded from the fan. The supervisor therefore **does not** put the
experiment's channels into ``TickPlan.released``. It used to, and
that dropped their integrator entries, which made ``step`` re-initialise the solver
bumplessly -- its first output equal to *the PWM on the fan*, i.e. the experiment's own
level. A channel released on its high level was handed that level as the solver's
starting point and stayed there until the integral wound it back down, and a later start
that took the inflated command as its base stepped up by another ``ident_amplitude``
(measured on the zone-wide schedule: qd1 ran 0.44-0.99 PWM where the sequential schedule
kept it at 0.2-0.69).

It returns control *near* the solver's own demand rather than exactly at it. On the
PI-DAS branch the integrator is exact -- the solver advances it from its stored entry and
never reads the PWM on the fan while that entry exists. Under the DAS MPC the first block
is solved with ``weight_dpwm`` against the PWM in force, which during the experiment is
the experiment's own level, so the stored core keeps a small pull toward it (in the safe
direction: more cooling than the counterfactual) and ``_replay_core`` hands the same plan
back for up to ``mpc_every_ticks`` ticks after the release. A larger ``weight_dpwm`` or
``mpc_every_ticks`` makes that residue larger; measured at the shipped values it does not
reintroduce the ratchet (section 8 item 112 has the numbers).

Nothing steps: ``step`` still rate-limits the command against the PWM on the fan, so the
return to the solver's own demand takes ``d_pwm_max`` per tick like every other move, and
under ``ident_levels: symmetric`` -- where the old release could hand the solver the
*low* level, below what it wanted -- the correction is now upward. A human override of
the same channel is released as before: there the fan was parked by somebody outside the
loop and bumpless transfer is the right transfer.
"""

from __future__ import annotations

import math
from collections.abc import Mapping
from dataclasses import dataclass, field
from typing import Any

from aqua_bridge.control import thermal
from aqua_bridge.control.estimator import CAL_MIN_SAMPLES, EMPTY, UNKNOWN
from aqua_bridge.model import IDENT_AMPLITUDE_MAX, MpcCommand, MpcConfig

__all__ = [
    "ACTIONS",
    "APPLY_FAILURES_ABORT",
    "CODE_STRIDE",
    "LEVEL_HIGH",
    "LEVEL_LOW",
    "LFSR_TAPS",
    "RESULT_ABORTED",
    "RESULT_COMPLETED",
    "TARGET_KINDS",
    "Advance",
    "TickFacts",
    "advance",
    "check_start",
    "dip_below_solver",
    "envelope_violations",
    "excitation",
    "facts_from_tick",
    "group_channels",
    "groups",
    "hold_sequence",
    "holds_cover_window",
    "levels_at",
    "lost_sensor_zones",
    "new_tracker",
    "no_headroom",
    "planned_dip",
    "rel_swing",
    "resume_tracker",
    "served_zones",
    "settle_snapshot",
    "start",
    "status",
    "target_channels",
    "track",
    "unexcitable",
    "zone_channels",
]

#: This many consecutive failed writes abort a running experiment.
APPLY_FAILURES_ABORT = 2
#: Galois LFSR taps (16 bit, maximal length) of the hold-time sequence.
LFSR_TAPS = 0xB400
#: Seed stride between the per-channel code streams of an ``ident_parallel`` phase.
CODE_STRIDE = 7919
LEVEL_LOW = 0
LEVEL_HIGH = 1
#: ``POST /api/ident`` actions and target kinds.
ACTIONS: tuple[str, ...] = ("start", "stop")
TARGET_KINDS: tuple[str, ...] = ("group", "channel")
RESULT_COMPLETED = "completed"
RESULT_ABORTED = "aborted"

_EPS = 1e-9
#: Bisection steps of ``ident_amplitude_mode: headroom`` (:func:`_levels`). Fixed, so the
#: sizing is a pure function of the config, the channel and the base; 40 halvings of a
#: ``(0, 0.3]`` interval land far inside the PWM quantisation of any controller here.
_SIZE_ITERS = 40


# ---------------------------------------------------------------------------
# Groups, targets and served zones
# ---------------------------------------------------------------------------


def _group_of(cfg: MpcConfig, channel: str) -> str:
    spec = cfg.fans.get(channel)
    return channel if spec is None or spec.group is None else spec.group


def groups(cfg: MpcConfig) -> dict[str, tuple[str, ...]]:
    """Fan group -> its channels in ``cfg.channels`` order (a channel without a group is
    its own group)."""
    out: dict[str, list[str]] = {}
    for ch in cfg.channels:
        out.setdefault(_group_of(cfg, ch), []).append(ch)
    return {g: tuple(chs) for g, chs in out.items()}


def group_channels(cfg: MpcConfig, group: str) -> tuple[str, ...]:
    """Channels of ``group``; ``KeyError`` for an unknown group."""
    return groups(cfg)[group]


def zone_channels(cfg: MpcConfig, channels: tuple[str, ...] | list[str]) -> tuple[str, ...]:
    """Every channel of the zones that *list* one of ``channels``, in config order.

    The unit ``ident_parallel`` excites: the zone's own outputs, not those of the zones
    it is only coupled to. A legacy config (no ``topology``) has no zones, so the input
    comes back unchanged."""
    layout = cfg.zone_layout
    zones = {z for ch in channels for z in layout.channel_zones.get(ch, ())}
    members = {ch for z in zones for ch in layout.zone_channels.get(z, ())}
    members.update(channels)
    return tuple(ch for ch in cfg.channels if ch in members)


def target_channels(cfg: MpcConfig, kind: str, name: str) -> tuple[str, ...]:
    """Every channel an experiment on ``kind`` ``name`` commands; ``KeyError`` if unknown.

    A channel target commands only that channel; a group target every channel of
    the group (the group phase, then each channel with its siblings at base). With
    ``ident_parallel`` both widen to every channel of the target's own zones
    (:func:`zone_channels`), which is the unit the thermal model's PE monitor needs."""
    if kind == "channel":
        if name not in cfg.channels:
            raise KeyError(name)
        base: tuple[str, ...] = (name,)
    elif kind == "group":
        base = group_channels(cfg, name)
    else:
        raise KeyError(kind)
    return zone_channels(cfg, base) if cfg.ident_parallel else base


def _phases(cfg: MpcConfig, kind: str, name: str) -> list[tuple[str, ...]]:
    channels = target_channels(cfg, kind, name)
    if cfg.ident_parallel or kind == "channel" or len(channels) == 1:
        # parallel: one phase over the whole zone, each channel on its own code
        return [channels]
    return [channels, *((ch,) for ch in channels)]


def served_zones(cfg: MpcConfig, channels: tuple[str, ...] | list[str]) -> tuple[str, ...]:
    """Zones whose drives an experiment on ``channels`` may warm (declared, with coupling)."""
    layout = cfg.zone_layout
    members: set[str] = set()
    for ch in channels:
        members.update(layout.served.get(ch, layout.channel_zones.get(ch, ())))
    return tuple(z for z in layout.zones if z in members)


# ---------------------------------------------------------------------------
# What one tick did
# ---------------------------------------------------------------------------


def _finite(value: object) -> bool:
    return (
        isinstance(value, int | float)
        and not isinstance(value, bool)
        and math.isfinite(float(value))
    )


def _mapping(value: object) -> dict[str, Any]:
    return dict(value) if isinstance(value, Mapping) else {}


def _floats(value: object) -> dict[str, float]:
    """The finite floats of a diagnostics mapping (empty when absent or malformed)."""
    if not isinstance(value, Mapping):
        return {}
    return {ch: float(v) for ch, v in value.items() if _finite(v)}


@dataclass(frozen=True)
class TickFacts:
    """The facts of one loop tick the experiment machine reads.

    ``mode`` is the solver command's mode (``None``: no solver command, e.g. a
    controller error); ``pwm`` the solver's command per channel; ``demand`` its want
    before the rate limit and the clamp (``diagnostics["target_pwm"]``, what the
    re-planned levels follow) and ``prev`` the PWM that want was penalised against
    (``diagnostics["prev_pwm"]``, what the fans were last set to), which is how the
    experiment's own influence comes back out of the want (module docstring). Both
    are empty when absent or malformed, and on ticks the supervisor knows no
    experiment can read them, which reads as "no demand known this tick". ``zones``,
    ``estimates``, ``bays``, ``saturated``, ``fan_stall``, ``sigma_floor`` and
    ``store`` are the matching entries of the diagnostics, empty when absent or
    malformed.

    An empty ``zones``, ``estimates``, ``bays``, ``saturated`` or ``fan_stall`` reads as
    **unsafe**: no zone is settled, no bay has an estimate, every channel counts as
    saturated. The remaining fields are not that shape, deliberately:

    * ``sigma_floor`` is written only under ``zones.trust_rule: sigma``, so an empty one
      means *no lost sensor group*, not *unknown*: :func:`lost_sensor_zones` returns no
      reason. Under ``strict`` a lost group faults its zone, which ``settle:`` and
      ``degraded`` catch instead, so the only way to lose the check is a ``sigma`` tick
      whose diagnostics are malformed -- ``mpc.step`` always writes the key there, and a
      missing solver command is already caught by ``mode is None``;
    * ``store`` is the model store's summary; an empty one restores no settle credit,
      which only makes a start wait longer.
    """

    ts: float | None
    mode: str | None
    applied: bool
    pwm: dict[str, float] = field(default_factory=dict)
    demand: dict[str, float] = field(default_factory=dict)
    prev: dict[str, float] = field(default_factory=dict)
    zones: dict[str, Any] = field(default_factory=dict)
    estimates: dict[str, Any] = field(default_factory=dict)
    bays: dict[str, Any] = field(default_factory=dict)
    saturated: dict[str, Any] = field(default_factory=dict)
    fan_stall: dict[str, Any] = field(default_factory=dict)
    sigma_floor: dict[str, Any] = field(default_factory=dict)
    store: dict[str, Any] = field(default_factory=dict)


def facts_from_tick(
    mpc_cmd: MpcCommand | None, *, ts: float | None, applied: bool, with_demand: bool = False
) -> TickFacts:
    """:class:`TickFacts` from the solver command of a tick (before ``compose``).

    ``with_demand`` copies ``demand`` and ``prev`` out of the diagnostics; only a
    running experiment that re-plans reads those two, so the supervisor asks for them
    on the ticks of one and no other tick -- ``ident_replan: false`` included -- pays
    for the copy."""
    ts_out = float(ts) if _finite(ts) else None
    if mpc_cmd is None:
        return TickFacts(ts=ts_out, mode=None, applied=bool(applied))
    diag = mpc_cmd.diagnostics if isinstance(mpc_cmd.diagnostics, Mapping) else {}
    target = diag.get("target_pwm") if with_demand else None
    prev = diag.get("prev_pwm") if with_demand else None
    return TickFacts(
        ts=ts_out,
        mode=mpc_cmd.mode.value,
        applied=bool(applied),
        pwm={ch: float(v) for ch, v in mpc_cmd.pwm.items()},
        demand=_floats(target),
        prev=_floats(prev),
        zones=_mapping(diag.get("zones")),
        estimates=_mapping(diag.get("estimates")),
        bays=_mapping(diag.get("bays")),
        saturated=_mapping(diag.get("saturated")),
        fan_stall=_mapping(diag.get("fan_stall")),
        sigma_floor=_mapping(diag.get("sigma_floor")),
        store=_mapping(diag.get("store")),
    )


# ---------------------------------------------------------------------------
# Settle tracking (every tick, running or not)
# ---------------------------------------------------------------------------


def new_tracker() -> dict[str, Any]:
    """Empty settle tracker: ``{"ts": last ts, "ok_since": {zone: ts}}`` (plus the
    optional ``resume`` credit of :func:`resume_tracker`)."""
    return {"ts": None, "ok_since": {}}


def resume_tracker(tracker: Mapping[str, Any], facts: TickFacts) -> dict[str, Any]:
    """``tracker`` with the settle credit the model store carried across a restart.

    ``facts.store["ident_settle"]`` is what
    :func:`aqua_bridge.control.persist.apply_seed` put there: ``{"ts": the tick the
    seed was applied on, "credit_s": {zone: seconds}}``, the settled time each zone
    had before the shutdown minus the outage. A malformed or absent section leaves the
    tracker untouched. The credit is spent by :func:`track` on the first tick the zone
    is trusted and fault-free again, and it keeps decaying until then, so nothing the
    daemon did not observe is ever counted as settled."""
    base = dict(tracker) if isinstance(tracker, Mapping) else new_tracker()
    raw = facts.store.get("ident_settle")
    if not isinstance(raw, Mapping):
        return base
    ts, credits = raw.get("ts"), raw.get("credit_s")
    if not _finite(ts) or not isinstance(credits, Mapping):
        return base
    credit_s = {
        str(zone): float(value)
        for zone, value in credits.items()
        if _finite(value) and float(value) > 0.0
    }
    if credit_s:
        base["resume"] = {"ts": float(ts), "credit_s": credit_s}  # type: ignore[arg-type]
    return base


def track(tracker: Mapping[str, Any], cfg: MpcConfig, facts: TickFacts) -> dict[str, Any]:
    """The tracker after ``facts``: per zone, the ``ts`` since which it has been trusted
    and fault-free without a break. A tick without a finite ``ts``, a clock running
    backwards or a tick without zone diagnostics starts every count over (a stored
    credit survives everything but a clock running backwards: it is timed against its
    own ``ts``)."""
    last = tracker.get("ts") if isinstance(tracker, Mapping) else None
    old = _mapping(tracker.get("ok_since")) if isinstance(tracker, Mapping) else {}
    resume = _mapping(tracker.get("resume")) if isinstance(tracker, Mapping) else {}
    ts = facts.ts
    backwards = ts is not None and _finite(last) and ts < float(last)
    if ts is None or backwards or not facts.zones:
        out: dict[str, Any] = {"ts": ts, "ok_since": {}}
        if resume and not backwards:
            out["resume"] = resume
        return out
    credit_s = _mapping(resume.get("credit_s"))
    resume_ts = resume.get("ts")
    left = dict(credit_s)
    ok_since: dict[str, float] = {}
    for zone in cfg.zone_layout.zones:
        info = facts.zones.get(zone)
        if not isinstance(info, Mapping) or info.get("trusted") is not True:
            continue
        if info.get("fault") is not False:
            continue
        since = old.get(zone)
        if _finite(since):
            ok_since[zone] = float(since)
            continue
        credit = 0.0
        if zone in left and _finite(resume_ts):
            credit = max(0.0, float(left[zone]) - (ts - float(resume_ts)))  # type: ignore[arg-type]
        left.pop(zone, None)  # spent, whatever was left of it
        ok_since[zone] = ts - credit
    out = {"ts": ts, "ok_since": ok_since}
    if left and _finite(resume_ts):
        out["resume"] = {"ts": float(resume_ts), "credit_s": left}  # type: ignore[arg-type]
    return out


def settle_snapshot(tracker: Mapping[str, Any]) -> dict[str, float]:
    """``{zone: seconds settled so far}`` for the model store (:mod:`aqua_bridge.modelstore`).

    Seconds, not absolute times, so the file needs no clock conversion: the outage is
    subtracted from them when the file is loaded again."""
    if not isinstance(tracker, Mapping):
        return {}
    ts = tracker.get("ts")
    if not _finite(ts):
        return {}
    out: dict[str, float] = {}
    for zone, since in _mapping(tracker.get("ok_since")).items():
        if _finite(since) and float(ts) >= float(since):  # type: ignore[arg-type]
            out[str(zone)] = float(ts) - float(since)  # type: ignore[arg-type]
    return out


# ---------------------------------------------------------------------------
# Envelope and preconditions
# ---------------------------------------------------------------------------


def _constrained_bays(cfg: MpcConfig, facts: TickFacts, zones: tuple[str, ...]) -> list[str]:
    """Bays of ``zones`` that carry a constraint (not ``empty`` by the estimator; a bay
    the estimator does not report counts, conservatively)."""
    topo = cfg.topology
    if topo is None:
        return []
    wanted = set(zones)
    out = []
    for bay, spec in topo.bays.items():
        if spec.zone not in wanted:
            continue
        info = facts.bays.get(bay)
        occupancy = info.get("occupancy") if isinstance(info, Mapping) else None
        if occupancy == EMPTY:
            continue
        out.append(bay)
    return out


def envelope_violations(
    cfg: MpcConfig, facts: TickFacts, zones: tuple[str, ...], over_c: float
) -> list[str]:
    """Envelope reasons (module docstring) for the drives of ``zones``, with ``over_c``
    allowed above each soft target."""
    reasons: list[str] = []
    for bay in _constrained_bays(cfg, facts, zones):
        est = facts.estimates.get(bay)
        keys = ("t_c", "margin_c", "soft_c", "hard_c", "limit_c")
        if not isinstance(est, Mapping) or not all(_finite(est.get(k)) for k in keys):
            reasons.append(f"no_estimate:{bay}")
            continue
        t_hat = float(est["t_c"])
        upper = t_hat + float(est["margin_c"])
        if t_hat > float(est["soft_c"]) + over_c + _EPS:
            reasons.append(f"envelope:{bay}")
        if t_hat > float(est["hard_c"]) + _EPS:
            reasons.append(f"hard:{bay}")
        if upper >= float(est["limit_c"]) - cfg.ident_abort_below_limit_c - _EPS:
            reasons.append(f"abort_temp:{bay}")
    return reasons


def lost_sensor_zones(facts: TickFacts, zones: tuple[str, ...]) -> list[str]:
    """``sensor_lost:<zone>`` reasons: zones of ``zones`` with a zone-air or bay sensor
    group that has no trusted, confirmed member this tick.

    Under ``zones.trust_rule: sigma`` such a zone stays trusted while the estimator's
    sigma is still small (the soft sigma floor holds its fans meanwhile), so nothing
    else in the precondition list sees the loss; under ``strict`` the zone is in fault
    and ``settle`` / ``degraded`` catch it first. Section 8 item 72."""
    reasons = []
    for zone in zones:
        info = facts.sigma_floor.get(zone)
        lost = info.get("lost") if isinstance(info, Mapping) else None
        if isinstance(lost, list | tuple) and lost:
            reasons.append(f"sensor_lost:{zone}")
    return reasons


def _curve(cfg: MpcConfig, ch: str) -> tuple[float, float]:
    """``(deadband, exponent)`` of ``ch``'s fan model, from the commissioned
    ``fan_models`` entry (:data:`aqua_bridge.control.fancurve.READERS`)."""
    spec = cfg.fan_models[cfg.fans[ch].model]
    return float(spec.deadband), float(spec.exponent)


def rel_swing(cfg: MpcConfig, ch: str, lo: float, hi: float) -> float:
    """Relative airflow variation of a 50/50 telegraph between PWM ``lo`` and ``hi``.

    The two levels are clamped into ``[pwm_min, pwm_max]`` first (a level the rail eats
    moves no air), turned into airflow with :func:`aqua_bridge.control.thermal.phi` on
    the channel's commissioned curve, and divided by their mean with the PE monitor's
    own floor under it.

    This is the swing of the telegraph *itself*, at its two levels. The PE monitor does
    not see those levels: it accumulates one sin^2-weighted mean of the airflow
    regressor per ``model_window_s`` block, so its own swing equals this one only while
    a regression window sits inside a single hold. A window that spans a switch averages
    the two levels, which can only move it toward their mean -- so this is an **upper
    bound** on what the monitor reads, tight exactly when every hold of ``ident_hold_s``
    is at least ``model_window_s`` long (:func:`holds_cover_window`)."""
    deadband, exponent = _curve(cfg, ch)
    p_lo = thermal.phi(_clamp(lo, cfg.pwm_min, cfg.pwm_max), deadband, exponent)
    p_hi = thermal.phi(_clamp(hi, cfg.pwm_min, cfg.pwm_max), deadband, exponent)
    mean = 0.5 * (p_lo + p_hi)
    return 0.5 * abs(p_hi - p_lo) / max(mean, thermal.PE_SCALE_FLOOR)


def _levels_at(cfg: MpcConfig, base: float, amplitude: float) -> tuple[float, float]:
    """The two levels ``amplitude`` around ``base``, unclamped."""
    if cfg.ident_levels == "symmetric":
        return base - amplitude, base + amplitude
    return base, base + amplitude


def _placed(cfg: MpcConfig, base: float, amplitude: float) -> tuple[float, float]:
    """The two levels at ``amplitude``, placed inside ``[pwm_min, pwm_max]`` with the room
    the channel really has on each side (``ident_amplitude_mode: headroom``, section 8
    item 120).

    A level the band will not take is **cut** at the rail, never slid across the base.
    ``ident_amplitude`` is the owner's ceiling on what one experiment may move the fans by
    -- added PWM under ``above``, given up under ``symmetric``
    (:data:`aqua_bridge.model.IDENT_AMPLITUDE_MAX`) -- and it bounds *both* directions, so
    neither level is ever further from the base than the amplitude asked for. Sliding a
    symmetric pair up off ``pwm_min`` would keep its full ``2 * amplitude`` swing, but it
    would buy that swing by putting the high level up to ``2 * ident_amplitude`` above the
    solver's own command: the loudest the enclosure gets, spent on the channel the solver
    had parked at the rail precisely because it wanted quiet there. The cut costs swing
    instead, ``excitable`` reports the short swing, and the sizing in :func:`_levels` then
    spends what room there is rather than a cap the owner never set.

    Under ``above`` the low level *is* the base, which the solver already commands, so
    only the high level can be cut, at ``pwm_max``."""
    lo, hi = _levels_at(cfg, base, amplitude)
    return _clamp(lo, cfg.pwm_min, cfg.pwm_max), _clamp(hi, cfg.pwm_min, cfg.pwm_max)


def _levels(cfg: MpcConfig, ch: str, base: float) -> tuple[float, float]:
    """The two levels this channel's telegraph runs between, from ``base`` (item 120).

    ``ident_amplitude_mode: fixed`` (the default) is :func:`_levels_at` at
    ``ident_amplitude``, unclamped and unchanged: the start refuses a base whose levels
    would leave the band, and a re-planned base clamps them (:func:`_replan`).

    ``headroom`` asks for the *smallest* amplitude in ``(0, ident_amplitude]`` whose
    placed levels reach ``ident_pe_aim * sqrt(PE_MIN)`` of relative airflow swing
    (:func:`rel_swing`), and falls back to ``ident_amplitude`` when even that falls short
    -- so a channel is never excited *less* than the fixed mode excites it, while one with
    room to spare spends less. Because the placement is inside the band by construction,
    the ``band:`` precondition no longer refuses a channel parked within
    ``ident_amplitude`` of a rail; it refuses one whose *swing*, after the cut, cannot
    reach the PE monitor's own floor (:func:`check_start`), which is the thing the band
    refusal was standing in for. A start that could inform the model of nothing is still
    refused; one that has room to say something now runs.

    The search is a fixed-length bisection on a swing that grows with the amplitude, so it
    is a pure function of the config, the channel and the base -- the schedule stays
    reproducible for the same three."""
    return _sized(cfg, ch, base)[:2]


def _sized(cfg: MpcConfig, ch: str, base: float) -> tuple[float, float, float]:
    """:func:`_levels` plus the amplitude it settled on, for the callers that have to ask
    what the band took away from it (:func:`check_start`)."""
    if cfg.ident_amplitude_mode != "headroom":
        return (*_levels_at(cfg, base, cfg.ident_amplitude), cfg.ident_amplitude)
    aim = (cfg.ident_pe_aim**2) * thermal.PE_MIN
    full = _placed(cfg, base, cfg.ident_amplitude)
    if rel_swing(cfg, ch, *full) ** 2 <= aim:
        return (*full, cfg.ident_amplitude)
    lo_a, hi_a = 0.0, cfg.ident_amplitude
    for _ in range(_SIZE_ITERS):
        mid = 0.5 * (lo_a + hi_a)
        if rel_swing(cfg, ch, *_placed(cfg, base, mid)) ** 2 > aim:
            hi_a = mid
        else:
            lo_a = mid
    return (*_placed(cfg, base, hi_a), hi_a)


def no_headroom(cfg: MpcConfig, ch: str, base: float) -> bool:
    """Whether ``ch`` has no room to run an experiment from ``base`` -- the question the
    ``band:`` precondition asks (section 8 item 120).

    Under ``fixed`` that is a level outside ``[pwm_min, pwm_max]``: the amplitude is
    whatever the owner set, so a channel that cannot take it whole is refused.

    Under ``headroom`` the levels are cut into the band by construction
    (:func:`_placed`), so nothing is left outside it to refuse and the same question is
    asked of what is left: the placement **cut** the pair, *and* what survived the cut
    cannot reach the aim. Both halves matter. A pair the band never touched is the pair
    ``fixed`` would have run, and refusing it would make ``headroom`` the stricter mode,
    which it is not; a pair that was cut but still reaches ``ident_pe_aim * sqrt(PE_MIN)``
    is exactly the start item 120 exists to let through -- a channel parked near a rail
    running a telegraph sized to the room it has. What is refused is the third case: a
    channel so near a rail that the band ate its step and the stub that is left cannot
    inform the model of anything, which would hold the schedule and every sibling channel
    for ``ident_max_duration_s`` to learn nothing."""
    lo, hi, amplitude = _sized(cfg, ch, base)
    if cfg.ident_amplitude_mode != "headroom":
        return lo < cfg.pwm_min - _EPS or hi > cfg.pwm_max + _EPS
    asked_lo, asked_hi = _levels_at(cfg, base, amplitude)
    cut = asked_lo < cfg.pwm_min - _EPS or asked_hi > cfg.pwm_max + _EPS
    aim = (cfg.ident_pe_aim**2) * thermal.PE_MIN
    return cut and rel_swing(cfg, ch, lo, hi) ** 2 <= aim


def _anchor(cfg: MpcConfig, lo: float, hi: float) -> float:
    """The base a pair of levels was drawn around: the low level under ``above``, their
    midpoint under ``symmetric``."""
    return 0.5 * (lo + hi) if cfg.ident_levels == "symmetric" else lo


def holds_cover_window(cfg: MpcConfig) -> bool:
    """Whether every hold of ``ident_hold_s`` lasts at least ``model_window_s``.

    True: a regression window can sit inside one hold, so the monitor sees the
    telegraph's own levels and :func:`rel_swing` is what it reads. False: some hold is
    shorter than a window, so windows that span a switch average the two levels and the
    monitor reads *less* than :func:`rel_swing` says -- ``pe_reach`` stays an upper
    bound and ``excitable`` a necessary condition, not a sufficient one."""
    return bool(cfg.ident_hold_s) and min(cfg.ident_hold_s) >= cfg.model_window_s


def _at_amplitude(cfg: MpcConfig, base: float, amplitude: float) -> tuple[float, float]:
    """The two levels an experiment planned *now* would use at ``amplitude``, in whichever
    way ``ident_amplitude_mode`` places them."""
    if cfg.ident_amplitude_mode == "headroom":
        return _placed(cfg, base, amplitude)
    return _levels_at(cfg, base, amplitude)


def excitation(
    cfg: MpcConfig, levels: Mapping[str, Any], bases: Mapping[str, Any] | None = None
) -> dict[str, dict[str, Any]]:
    """Per channel of ``levels`` (``{channel: [low, high]}``), how much airflow variation
    that telegraph can put into the thermal model's PE monitor.

    ``rel_swing`` is :func:`rel_swing` of the two levels and ``pe_reach`` its square: the
    most a group of this one channel could report on ``pe_diag`` from the telegraph
    alone, reached when a regression window sits inside one hold and short of it
    otherwise (:func:`holds_cover_window`). ``pe_bound`` is
    :data:`aqua_bridge.control.thermal.PE_MIN`, the bound the ``converged`` rule asks the
    zone's smallest eigenvalue to pass -- named apart from the thermal summary's own
    ``pe_min``, which is the *measured* eigenvalue and not this constant.

    ``excitable`` is whether ``pe_reach`` clears that bound **at the configured
    ``ident_amplitude``**, from where the channel sits. Because ``pe_reach`` is an upper
    bound it is a necessary condition, not a sufficient one: ``false`` means the channel
    provably cannot carry its zone over the bound from this base, ``true`` that it can
    only if the schedule delivers the swing. ``pe_reach_at_cap`` / ``excitable_at_cap``
    are the same numbers at :data:`~aqua_bridge.model.IDENT_AMPLITUDE_MAX`, the largest
    amplitude the config allows, so a channel that is ``excitable_at_cap`` but not
    ``excitable`` wants a larger ``ident_amplitude`` while one that is neither is out of
    reach at any allowed amplitude and waits for the solver to park it lower (section 8
    item 110)."""
    out: dict[str, dict[str, Any]] = {}
    for ch, pair in levels.items():
        if not (isinstance(pair, list | tuple) and len(pair) == 2):
            continue
        lo, hi = float(pair[0]), float(pair[1])
        swing = rel_swing(cfg, ch, lo, hi)
        # The base the pair was drawn around. ``ident_amplitude_mode: headroom`` may place
        # the two levels asymmetrically about it, so the caller's own base wins where it
        # has one and the midpoint/low-level rule is the fallback (section 8 item 120).
        base = bases.get(ch) if isinstance(bases, Mapping) else None
        anchor = float(base) if _finite(base) else _anchor(cfg, lo, hi)  # type: ignore[arg-type]
        cap_lo, cap_hi = _at_amplitude(cfg, anchor, IDENT_AMPLITUDE_MAX)
        cap = rel_swing(cfg, ch, cap_lo, cap_hi)
        out[ch] = {
            "rel_swing": swing,
            "pe_reach": swing * swing,
            "pe_reach_at_cap": cap * cap,
            "pe_bound": thermal.PE_MIN,
            "excitable": swing * swing > thermal.PE_MIN,
            "excitable_at_cap": cap * cap > thermal.PE_MIN,
        }
    return out


def unexcitable(
    cfg: MpcConfig, levels: Mapping[str, Any], bases: Mapping[str, Any] | None = None
) -> list[str]:
    """The channels of ``levels`` whose telegraph cannot reach ``PE_MIN`` at the
    configured ``ident_amplitude`` (in order). See :func:`excitation`: a channel is
    listed here whatever ``excitable_at_cap`` says, so the list answers "which channel
    cannot inform the model on this run", not "which one never could"."""
    return [ch for ch, e in excitation(cfg, levels, bases).items() if not e["excitable"]]


def _clamp(value: float, lo: float, hi: float) -> float:
    return lo if value < lo else (hi if value > hi else value)


def dip_below_solver(cfg: MpcConfig) -> float:
    """How far below the solver's own command an experiment planned *now* would sit, PWM.

    ``0`` under ``above`` (the experiment never commands less cooling than the
    solver), ``ident_amplitude`` under ``symmetric`` (the owner-accepted dip). A
    *running* experiment is floored by its own plan instead (:func:`planned_dip`).

    It stays the worst case under ``ident_amplitude_mode: headroom`` (section 8 item 120),
    where the per-channel dip is smaller: the sizing only ever shrinks the amplitude and
    the placement only ever slides the pair *up*, so no channel is ever further under the
    solver's command than ``ident_amplitude``. A bound that cannot be exceeded is the
    right fallback for a floor."""
    return cfg.ident_amplitude if cfg.ident_levels == "symmetric" else 0.0


def planned_dip(exp: Mapping[str, Any] | None, ch: str, cfg: MpcConfig) -> float:
    """How far below the solver's command the running experiment's own plan puts ``ch``.

    ``plan_base - low level`` of that channel as the experiment itself planned them:
    ``0`` under ``above``, ``ident_amplitude`` under ``symmetric``, and less wherever
    the low level clamped at ``pwm_min`` -- read off the plan, so the floor
    :meth:`~aqua_bridge.control.supervisor.Supervisor.compose` applies follows the
    experiment that is running and not a config rebuilt under it. Falls back to
    :func:`dip_below_solver` when the plan does not carry the two (and never below
    zero, so the floor is never weaker than the solver's own command)."""
    if isinstance(exp, Mapping):
        bases, pairs = exp.get("plan_base"), exp.get("levels")
        anchor = bases.get(ch) if isinstance(bases, Mapping) else None
        levels = pairs.get(ch) if isinstance(pairs, Mapping) else None
        low = levels[0] if isinstance(levels, list | tuple) and levels else None
        if _finite(anchor) and _finite(low):
            return max(0.0, float(anchor) - float(low))  # type: ignore[arg-type]
    return max(0.0, dip_below_solver(cfg))


def check_start(
    cfg: MpcConfig,
    tracker: Mapping[str, Any],
    facts: TickFacts | None,
    kind: str,
    name: str,
    *,
    human_control: bool,
) -> list[str]:
    """Every reason an experiment on ``kind`` ``name`` may not start now (empty: it may).

    ``human_control`` is true when the control mode is not ``auto`` or a human
    override is set. ``KeyError`` for an unknown target."""
    channels = target_channels(cfg, kind, name)
    reasons: list[str] = []
    if human_control:
        reasons.append("control_mode")
    if facts is None or facts.mode is None:
        reasons.append("no_tick")
        return reasons
    if facts.mode != "auto":
        reasons.append(f"mode:{facts.mode}")
    for ch in channels:
        base = facts.pwm.get(ch)
        if not _finite(base):
            reasons.append(f"no_command:{ch}")
            continue
        if facts.saturated.get(ch) is not False:
            reasons.append(f"saturated:{ch}")
        lo, hi = _levels(cfg, ch, float(base))  # type: ignore[arg-type]
        # "this channel has no room to run the experiment", in whichever form the sizing
        # mode makes that question (:func:`no_headroom`, section 8 item 120). Asked
        # whatever ``ident_require_excitable`` says, because the refusal it replaces was
        # not optional either.
        if no_headroom(cfg, ch, float(base)):  # type: ignore[arg-type]
            reasons.append(f"band:{ch}")
        elif (
            cfg.ident_require_excitable
            and not excitation(cfg, {ch: (lo, hi)}, {ch: base})[ch]["excitable"]
        ):
            reasons.append(f"not_excitable:{ch}")
    for ch in cfg.channels:
        if facts.fan_stall.get(ch):
            reasons.append(f"fan_stall:{ch}")
    zones = served_zones(cfg, channels)
    ok_since = _mapping(tracker.get("ok_since")) if isinstance(tracker, Mapping) else {}
    for zone in zones:
        since = ok_since.get(zone)
        if (
            facts.ts is None
            or not _finite(since)
            or facts.ts - float(since) < cfg.ident_settle_s - _EPS  # type: ignore[arg-type]
        ):
            reasons.append(f"settle:{zone}")
    reasons.extend(lost_sensor_zones(facts, zones))
    reasons.extend(_bay_reasons(cfg, facts, zones))
    for reason in envelope_violations(cfg, facts, zones, cfg.ident_start_band_c):
        reasons.append(
            reason.replace("envelope:", "start_band:", 1)
            if reason.startswith("envelope:")
            else reason
        )
    return reasons


def _bay_reasons(cfg: MpcConfig, facts: TickFacts, zones: tuple[str, ...]) -> list[str]:
    topo = cfg.topology
    if topo is None:
        return []
    settle = 0.0 if cfg.estimator is None else cfg.estimator.bay_settle_s
    wanted = set(zones)
    reasons: list[str] = []
    for bay, spec in topo.bays.items():
        if spec.zone not in wanted:
            continue
        info = facts.bays.get(bay)
        if not isinstance(info, Mapping):
            reasons.append(f"bay_unknown:{bay}")
            continue
        if info.get("occupancy") == UNKNOWN or info.get("occupancy") is None:
            reasons.append(f"bay_unknown:{bay}")
        since = info.get("since_ts")
        pending = (
            info.get("pending_empty_s", 0.0),
            info.get("pending_occupied_ticks", 0),
            info.get("pending_unknown_s", 0.0),  # a blind bay: the debounce is counting
        )
        if any(not _finite(v) or float(v) != 0.0 for v in pending) or (
            since is not None
            and (not _finite(since) or facts.ts is None or facts.ts - float(since) < settle)
        ):
            reasons.append(f"bay_transition:{bay}")
        cal = info.get("calibration")
        if isinstance(cal, Mapping):
            samples = cal.get("samples")
            if not _finite(samples) or float(samples) < CAL_MIN_SAMPLES:  # type: ignore[arg-type]
                reasons.append(f"calibrating:{bay}")
    return reasons


# ---------------------------------------------------------------------------
# Sequence
# ---------------------------------------------------------------------------


def _lfsr_next(state: int) -> int:
    lsb = state & 1
    state >>= 1
    if lsb:
        state ^= LFSR_TAPS
    return state


def hold_sequence(cfg: MpcConfig, total_s: float) -> list[float]:
    """Hold times (s) drawn from ``ident_hold_s`` until they cover ``total_s``."""
    state = cfg.ident_seed % 0xFFFF + 1
    holds = list(cfg.ident_hold_s)
    out: list[float] = []
    covered = 0.0
    while covered < total_s:
        for _ in range(16):
            state = _lfsr_next(state)
        hold = holds[state % len(holds)]
        out.append(hold)
        covered += hold
    return out


def _code(cfg: MpcConfig, index: int, start_s: float, end_s: float) -> list[list[float]]:
    """One channel's own telegraph over ``[start_s, end_s)`` (``ident_parallel``).

    Its own LFSR stream, seeded from ``ident_seed`` and the channel's position in the
    phase, draws both the start level and every hold, so the codes of a zone's channels
    are independent of one another -- which is the whole point: the PE monitor reads the
    smallest eigenvalue over the zone's groups, and channels that switch together leave
    it at zero."""
    state = (cfg.ident_seed + CODE_STRIDE * (index + 1)) % 0xFFFF + 1
    holds = list(cfg.ident_hold_s)
    segments: list[list[float]] = []
    for _ in range(16):
        state = _lfsr_next(state)
    level = LEVEL_HIGH if state & 1 else LEVEL_LOW
    t = start_s
    while t < end_s - _EPS:
        segments.append([t, level])
        for _ in range(16):
            state = _lfsr_next(state)
        t += holds[state % len(holds)]
        level = LEVEL_LOW if level == LEVEL_HIGH else LEVEL_HIGH
    return segments


def _schedule(cfg: MpcConfig, phases: list[tuple[str, ...]]) -> list[dict[str, Any]]:
    duration = cfg.ident_max_duration_s
    per_phase = duration / len(phases)
    # the shared stream: a coded phase draws from its own per-channel streams instead
    holds = (
        iter(())
        if cfg.ident_parallel
        else iter(hold_sequence(cfg, duration + len(phases) * max(cfg.ident_hold_s)))
    )
    out: list[dict[str, Any]] = []
    for i, channels in enumerate(phases):
        start_s = i * per_phase
        end_s = duration if i == len(phases) - 1 else (i + 1) * per_phase
        phase: dict[str, Any] = {"channels": list(channels), "start_s": start_s, "end_s": end_s}
        if cfg.ident_parallel:
            phase["codes"] = {ch: _code(cfg, j, start_s, end_s) for j, ch in enumerate(channels)}
        else:
            segments: list[list[float]] = []
            t, level = start_s, LEVEL_HIGH
            while t < end_s - _EPS:
                segments.append([t, level])
                t += next(holds)
                level = LEVEL_LOW if level == LEVEL_HIGH else LEVEL_HIGH
            phase["segments"] = segments
        out.append(phase)
    return out


def start(
    cfg: MpcConfig, facts: TickFacts, kind: str, name: str, *, skip_ticks: int = 0
) -> dict[str, Any]:
    """A new experiment (plain JSON) armed for the tick after ``facts``.

    ``skip_ticks`` moves offset 0 that many ticks further out, for the ticks whose plan
    the loop has already taken and which therefore cannot carry the overrides: the
    supervisor passes 1 for a start that arrives between ``plan_tick`` and
    ``record_tick``, so the recorded levels are the levels that ran (section 8 item 20).

    Call only when :func:`check_start` returned no reason."""
    channels = target_channels(cfg, kind, name)
    phases = _phases(cfg, kind, name)
    base = {ch: float(facts.pwm[ch]) for ch in channels}
    assert facts.ts is not None
    exp: dict[str, Any] = {
        "target": {"kind": kind, "name": name},
        "start_ts": facts.ts + cfg.dt * (1 + max(int(skip_ticks), 0)),
        "last_ts": facts.ts,
        "duration_s": cfg.ident_max_duration_s,
        "channels": list(channels),
        "served_zones": list(served_zones(cfg, channels)),
        "base": base,
        "plan_base": dict(base),
        "replan": bool(cfg.ident_replan),
        "levels": {ch: list(_levels(cfg, ch, base[ch])) for ch in channels},
        "phases": _schedule(cfg, phases),
        "apply_failures": 0,
        # the overrides of the tick whose applied PWM the next tick reports as its
        # ``prev``: empty here, since no experiment ran on the tick before the start
        "prev_overrides": {},
    }
    exp.update(levels_at(exp, 0.0))
    return exp


def _replan(
    exp: Mapping[str, Any], cfg: MpcConfig, facts: TickFacts, *, switching: bool
) -> dict[str, Any]:
    """``{"plan_base", "levels"}`` after following the tick's demand (module docstring).

    Per channel: the tick's want minus the echo of the experiment's own level in it
    (``min(prev, the override the experiment had on that tick) - plan_base``, never
    negative), clamped into ``[pwm_min, pwm_max]``. Bounding the echo by the experiment's
    own override is what keeps the two cases apart: a fan above the anchor because the
    experiment put it there is an echo and comes off, a fan above the anchor because the
    solver's own command floored it there is demand and stays. The anchor rises to the
    result at once; it falls to it only when ``switching`` (this tick ends a hold or a
    phase), by at most ``d_pwm_max``, and never below the frozen base of the start. A
    channel whose demand or ``prev`` this tick is missing or not finite keeps its
    anchor."""
    plan_base = {ch: float(v) for ch, v in exp["plan_base"].items()}
    base = exp["base"]
    ran = _mapping(exp.get("prev_overrides"))
    for ch in exp["channels"]:
        want, prev = facts.demand.get(ch), facts.prev.get(ch)
        if not _finite(want) or not _finite(prev):
            continue
        own = ran.get(ch)
        echo = 0.0 if not _finite(own) else max(0.0, min(float(prev), float(own)) - plan_base[ch])  # type: ignore[arg-type]
        wanted = _clamp(float(want) - echo, cfg.pwm_min, cfg.pwm_max)  # type: ignore[arg-type]
        if wanted > plan_base[ch]:
            plan_base[ch] = wanted
        elif switching:
            floor = max(wanted, float(base[ch]), plan_base[ch] - cfg.d_pwm_max)
            plan_base[ch] = min(plan_base[ch], floor)
    levels = {
        ch: [_clamp(v, cfg.pwm_min, cfg.pwm_max) for v in _levels(cfg, ch, plan_base[ch])]
        for ch in exp["channels"]
    }
    return {"plan_base": plan_base, "levels": levels}


def _level_at(segments: list[Any], offset_s: float) -> int:
    level = LEVEL_HIGH
    for seg_t, seg_level in segments:
        if seg_t <= offset_s + _EPS:
            level = int(seg_level)
    return level


def _phase_levels(phase: Mapping[str, Any], offset_s: float) -> dict[str, int]:
    """Level per channel of ``phase`` at ``offset_s``: one shared level, or, for a
    ``ident_parallel`` phase, each channel's own code."""
    codes = phase.get("codes")
    if codes:
        return {ch: _level_at(codes[ch], offset_s) for ch in phase["channels"]}
    return dict.fromkeys(phase["channels"], _level_at(phase["segments"], offset_s))


def _position(exp: Mapping[str, Any], offset_s: float) -> tuple[int, int]:
    """``(phase index, level)`` of the schedule at ``offset_s`` -- the schedule only,
    so it says nothing about the levels themselves (they are re-planned around it).
    For a coded phase the level is that of its first channel, the phase's reference;
    :func:`_phase_levels` has the rest."""
    phases = exp["phases"]
    index = len(phases) - 1
    for i, phase in enumerate(phases):
        if offset_s < phase["end_s"] - _EPS:
            index = i
            break
    phase = phases[index]
    codes = phase.get("codes")
    segments = codes[phase["channels"][0]] if codes else phase["segments"]
    return index, _level_at(segments, offset_s)


def levels_at(exp: Mapping[str, Any], offset_s: float) -> dict[str, Any]:
    """``{"phase", "level", "overrides"}`` at ``offset_s`` into the experiment, plus
    ``"code"`` (the level per channel) on a ``ident_parallel`` phase.

    Every channel of the target gets an override: the phase's channels their level,
    the other channels of the group their base (``plan_base``: the frozen start base,
    or the re-planned anchor with ``ident_replan``)."""
    index, level = _position(exp, offset_s)
    phase = exp["phases"][index]
    per_channel = _phase_levels(phase, offset_s)
    held = exp.get("plan_base") or exp["base"]
    overrides = {
        ch: float(exp["levels"][ch][per_channel[ch]]) if ch in per_channel else float(held[ch])
        for ch in exp["channels"]
    }
    out: dict[str, Any] = {"phase": index, "level": level, "overrides": overrides}
    if phase.get("codes"):
        out["code"] = [per_channel[ch] for ch in phase["channels"]]
    return out


# ---------------------------------------------------------------------------
# Running
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class Advance:
    """Outcome of :func:`advance`: the experiment to keep (``None`` when it ended),
    ``result`` (``None`` while running, else :data:`RESULT_COMPLETED` or
    :data:`RESULT_ABORTED`) and the abort ``reason``."""

    experiment: dict[str, Any] | None
    result: str | None = None
    reason: str | None = None


def advance(exp: Mapping[str, Any], cfg: MpcConfig, facts: TickFacts) -> Advance:
    """Check the tick that just ran with ``exp``'s overrides and arm the next tick.

    The aborts of this tick are decided first, on this tick's estimates; only a
    surviving experiment re-plans its levels from this tick's demand."""
    if facts.mode is None or facts.mode == "fallback":
        return Advance(None, RESULT_ABORTED, "fallback")
    if facts.mode == "degraded":
        return Advance(None, RESULT_ABORTED, "degraded")
    failures = 0 if facts.applied else int(exp.get("apply_failures", 0)) + 1
    if failures >= APPLY_FAILURES_ABORT:
        return Advance(None, RESULT_ABORTED, "apply_failed")
    ts = facts.ts
    if ts is None or ts < float(exp["last_ts"]):
        return Advance(None, RESULT_ABORTED, "clock")
    duration = float(exp["duration_s"])
    if ts - float(exp["start_ts"]) > duration + cfg.dt + _EPS:
        return Advance(None, RESULT_ABORTED, "duration")
    zones = tuple(exp["served_zones"])
    reasons: list[str] = []
    for zone in zones:
        info = facts.zones.get(zone)
        if not isinstance(info, Mapping) or info.get("trusted") is not True:
            reasons.append(f"zone_untrusted:{zone}")
    for bay in _constrained_bays(cfg, facts, zones):
        info = facts.bays.get(bay)
        if not isinstance(info, Mapping) or info.get("occupancy") in (UNKNOWN, None):
            reasons.append(f"bay_unknown:{bay}")
    reasons.extend(lost_sensor_zones(facts, zones))
    reasons.extend(envelope_violations(cfg, facts, zones, cfg.ident_max_over_c))
    for ch in exp["channels"]:
        if facts.fan_stall.get(ch):
            reasons.append(f"fan_stall:{ch}")
    if reasons:
        return Advance(None, RESULT_ABORTED, reasons[0])
    offset = ts + cfg.dt - float(exp["start_ts"])
    if offset >= duration - _EPS:
        return Advance(None, RESULT_COMPLETED, None)
    out = dict(exp)
    out["last_ts"] = ts
    out["apply_failures"] = failures
    at = max(offset, 0.0)
    if exp.get("replan"):
        out["prev_overrides"] = dict(exp["overrides"])  # the level this tick ran at
        index, level = _position(out, at)
        switching = (index, level) != (int(exp["phase"]), int(exp["level"]))
        phase = out["phases"][index]
        if not switching and phase.get("codes"):
            # a coded phase switches when *any* of its channels does, not only the
            # reference channel ``_position`` reads
            now = [_phase_levels(phase, at)[ch] for ch in phase["channels"]]
            switching = now != list(exp.get("code") or ())
        out.update(_replan(exp, cfg, facts, switching=switching))
    out.update(levels_at(out, at))
    return Advance(out)


def status(
    exp: Mapping[str, Any] | None, last: Mapping[str, Any] | None, cfg: MpcConfig
) -> dict[str, Any]:
    """``ControlSnapshot.extra["experiment"]`` (plain JSON).

    ``running``, ``target`` (``{kind, name}``), ``group`` (the target when it is a
    group), ``channel`` (the single channel of the current phase, else ``None``),
    ``channels`` of the current phase, ``phase`` / ``phases``, ``level``
    (``high`` | ``low``), ``overrides``, ``base`` (the frozen base at the start),
    ``plan_base`` (the anchor the levels are drawn around now) and ``levels``
    (``{channel: [low, high]}``) with ``replan`` (whether this experiment follows the
    live demand), ``excitation`` / ``unexcitable`` (:func:`excitation`: how far each
    channel's telegraph can move the PE monitor from where it sits, and the channels
    that cannot clear its bound at the configured ``ident_amplitude`` -- section 8 item
    110), ``holds_cover_window`` (:func:`holds_cover_window`: whether the schedule can
    deliver that reach at all, or whether a regression window averages two levels and
    the monitor reads less), ``elapsed_s``,
    ``remaining_s``, and from the last experiment that ended: ``last_result``
    (``completed`` | ``aborted``), ``last_abort_reason`` and ``last_target``.
    """
    last = last or {}
    out: dict[str, Any] = {
        "enabled": cfg.ident_enabled,
        "replan": cfg.ident_replan,
        "running": exp is not None,
        "target": None,
        "group": None,
        "channel": None,
        "channels": [],
        "phase": None,
        "phases": 0,
        "level": None,
        "overrides": {},
        "base": {},
        "plan_base": {},
        "levels": {},
        "excitation": {},
        "unexcitable": [],
        "holds_cover_window": holds_cover_window(cfg),
        "elapsed_s": None,
        "remaining_s": None,
        "last_result": last.get("result"),
        "last_abort_reason": last.get("reason"),
        "last_target": last.get("target"),
    }
    if exp is None:
        return out
    phase = exp["phases"][exp["phase"]]
    elapsed = max(0.0, float(exp["last_ts"]) + cfg.dt - float(exp["start_ts"]))
    out.update(
        {
            "target": dict(exp["target"]),
            "group": exp["target"]["name"] if exp["target"]["kind"] == "group" else None,
            "channel": phase["channels"][0] if len(phase["channels"]) == 1 else None,
            "channels": list(phase["channels"]),
            "phase": int(exp["phase"]),
            "phases": len(exp["phases"]),
            "level": "high" if exp["level"] == LEVEL_HIGH else "low",
            "overrides": dict(exp["overrides"]),
            "base": dict(exp["base"]),
            "plan_base": dict(exp.get("plan_base") or exp["base"]),
            "levels": {ch: list(v) for ch, v in exp["levels"].items()},
            "excitation": excitation(cfg, exp["levels"], exp.get("plan_base")),
            "unexcitable": unexcitable(cfg, exp["levels"], exp.get("plan_base")),
            "replan": bool(exp.get("replan")),
            "elapsed_s": elapsed,
            "remaining_s": max(0.0, float(exp["duration_s"]) - elapsed),
        }
    )
    return out
