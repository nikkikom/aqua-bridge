"""Latent drive temperatures: per-zone Kalman filter, occupancy, SMART calibration (plan section 2).

Pure: :func:`update` is a function of its arguments, keeps everything in a
plain-JSON memory (``solver_memory["estimator"]``), reads no clock and does
no I/O. ``mpc.step`` calls it every tick, fault ticks included (plan section
6, step 5a), and hands the estimates block it returns to the solver
(:mod:`aqua_bridge.control.estimates` describes the block).

Model (per zone ``z``; SI units: W, J/K, W/K, degC, s)
-----------------------------------------------------
States ``x = [T_a, d_a, T_d(1..n), T_s(1..n), q(1..n), c(1..r)]`` over **every**
bay of the zone (topology order), so the layout never changes with occupancy::

    C_a dT_a/dt = sum_j g_j (T_d,j - T_a) - (Q_z + leak)(T_a - T_in) + sum_z' kappa (T_a,z' - T_a)
                  + C_a d_a
    C_d dT_d/dt = C_d q_j - g_j (T_d,j - T_a)            g_j = g0 + k * Qn_z
    tau_s dT_s/dt = s_j T_d,j + (1 - s_j) T_a + b_j - T_s,j
    dd_a/dt = 0, dq_j/dt = 0                             (integrating disturbances)
    dc_i/dt = 0                                          (placement offsets, random walk)

``s_j = 1 - beta_j`` and ``b_j`` are the proximal sensor map: the prior
``s = 0.7``, ``b = -2.1`` (:data:`~aqua_bridge.control.estimates.PRIOR_BETA`,
``PRIOR_OFFSET_C``) or the bay's accepted SMART calibration. An ``empty`` bay
has no drive: its drive and heat rows are frozen (no dynamics, no process
noise) and its sensor node follows the air (``tau_s dT_s/dt = T_a - T_s``); an
``unknown`` bay is modelled like an occupied one (conservative).

**Placement offsets** (``c``, plan section 8 item 67). A bay's sensor node is
anchored on its first proximal member that is not ``redundant``; every further
member reads that node **plus its own offset** ``c_i``, so its measurement row is
``H = e_{T_s} + e_{c_i}``. Two sensors on one bay sit at different placements: they
see different fractions of the drive (``beta``) and carry different offsets, so
they disagree by several degC at load, and one sensor node for both leaves that
disagreement in the innovations, where the fast-swap rule below reads it as a swap
on every tick (a bay's sigma stuck at 4-7 degC on the ``rich`` simulator, which
faulted healthy zones under ``zones.trust_rule: sigma``). The offsets are ``r``
random-walk states per zone (``estimator.q_offset`` per tick) with the prior
``N(0, estimator.proximal_offset_c ** 2)``; that prior is wide, so the first
reading of a further member identifies its offset instead of moving the drive.
With ``proximal_offset_c: 0`` there are no offset states and every member reads
the node directly, as before this existed. A bay with one proximal sensor has no
offset either, so its arithmetic is unchanged.

**A node per proximal sensor** (plan section 8 item 101,
``estimator.proximal_slope_spread``, default 0 = the layout above). An offset is a
*constant*, and two placements do not differ by one: they see different fractions
``1 - beta`` of the drive, so their disagreement is ``ds * (T_d - T_a) + db`` and moves
with the drive-to-air rise. Above 0, every proximal member of a bay is its own state
``T_s,i`` with its own lag ``sensors.<name>.tau_s`` and its own map ``s + ds_i``,
``b + db_i`` (clipped to :data:`CAL_SLOPE_BOUNDS` and :data:`CAL_OFFSET_BOUNDS`), and it
measures that node directly -- a plain scalar update, no offset row. The ``c`` block
becomes those extra nodes, so **the state is exactly the same size** and the measurement
is cheaper than the rank-two one it replaces.

``(ds_i, db_i)`` is a two-parameter RLS without forgetting (:func:`_dmap_absorb`), prior
``(0, 0)`` at ``diag(proximal_slope_spread ** 2, proximal_offset_c ** 2)``, one row
``z_i - z_anchor = ds * rise + db`` per tick on which both members report, the bay is not
``empty`` and the fast-swap rule did not fire, with variance
``R_i + R_anchor + CAL_ROW_VAR``. No SMART is needed for it: both members see the same
drive and the same air, so their *difference* is identifiable where the absolute map is
not. The member's measurement carries ``h P h^T`` of that map in its ``R``, so an
unconverged placement still cannot look like a swap, exactly as the offset's own variance
does above. The bay's own node stays the anchor's: ``t_sensor``, the occupancy ``dT``,
the association series and the SMART calibration all read it, so ``mem["cal"]`` and the
model store are untouched. A config with one proximal sensor per bay is bit-identical
either way (there is no further member to give a node to).

**Which layout learns what, and the consequence** (plan section 8 item 125). Two
placements on one bay disagree by ``ds * (T_d - T_a) + db``. The per-sensor layout fits
both halves; the fused layout has **one** state for the pair, and that state is the whole
disagreement at the current rise, so it absorbs ``ds * rise`` as drift. The fused layout
therefore cannot learn the *shape* of a placement difference, and nothing here is going to
make it: doing so **is** the per-sensor layout, at the same state size and a cheaper
measurement. What both layouts can carry is the number itself, so every bay with a
redundant pair publishes a ``placement`` verdict per further member
(:func:`_placement_view`) -- the learned gap, the box the config's own priors allow it at
this tick's rise (``estimator.proximal_gap_sigmas``), whether it has walked out of that
box, and **the layout with what that layout separates**. On the per-sensor layout ``over``
is a placement the config does not allow: a sensor coming loose, fouling or ageing. On the
fused layout it may equally be a load the layout has no slope for, and the verdict says so
rather than letting a reader take the two for the same evidence.

Priors (plan section 3 table): ``C_a = 200``, ``leak = 1``, ``kappa = 3`` for
declared ``coupled_to`` pairs (the neighbour's previous air estimate is a
known input), ``g0 = 0.3``, ``k = 0.5``, ``C_d = tau_d_s * (g0 + k)`` (the
drive class's time constant at full airflow). Airflow: every channel's
``phi = clip((u - deadband) / (1 - deadband), 0, 1) ** exponent`` of its fan
model at ``u = prev`` (the command on the fans over the last interval) --
from the curve in force, which is the online fit's when ``update`` is given
``curves`` and it has a usable entry for that fan model, else the configured
``fan_models`` one (item 107: the estimator and the thermal model must plan on
the same air, or the MPC's prediction-error guard reads the difference as a bad
model and falls back) -- and
``E = 33 W/K * count`` split evenly over the zones that list the channel;
``Q_z = sum E phi``, ``Qn_z = Q_z / sum E``. ``T_in`` is the mean of the zone's
trusted inlet sensors (``inlet`` or every inlet sensor for ``mix``), else the
last one seen, else the zone's own air estimate.

Transition and filter
---------------------
The plan writes a second-order Euler transition. At ``dt = 5 s`` the zone air
node's time constant is about ``C_a / (Q + leak) ~ 2 s`` and Euler of any low
order is unstable there, so the transition is the exact discretisation of the
affine system instead: the matrix exponential of ``[[A h, c h], [0, 0]]`` by
scaling and squaring with a degree-10 Taylor polynomial (numpy only, no
scipy). ``h`` is the time since the previous tick (0 on a clock that does not
advance, at most :data:`MAX_PREDICT_S`). Process noise per tick
``diag(q_t_air, q_d_air, q_t_drive..., q_t_sensor..., q_heat..., q_offset...)
* h / dt``.

Measurements are sequential scalar updates in Joseph form
``P = (I - K H) P (I - K H)^T + K R K^T`` (evaluated through the rank-one
structure of ``H``, then symmetrised): every trusted ``zone_air`` sensor
on ``T_a`` and every trusted ``drive_proximal`` sensor of a bay on that bay's
``T_s`` -- on ``T_s + c_i`` for a member that is not the anchor -- with
``R = sensor_noise_c ** 2 + quant_c ** 2 / 12`` (an empty bay's
sensor measures air: weight 0.5, ``R / 0.5``). Untrusted sensors are skipped
(predict only). SMART enters as a measurement of ``T_d`` with ``R = 1`` only
for a calibrated bay.

Fast-swap rule (an addition to the plan, conservative): a proximal innovation
``nu`` with ``|nu| > jump_min_c`` and ``|nu| > jump_sigmas * sqrt(S)`` on an
occupied or unknown bay adds ``(nu / s) ** 2`` to the drive variance and
``nu ** 2`` to the sensor node's variance before the update, and drops the
bay's correlation association. ``S`` is that measurement's own innovation
variance, so for a member with an offset it carries ``c_i``'s variance too: an
uncertain placement offset cannot look like a swap, while a swap -- which moves
the drive and therefore every member together -- still does. The offset keeps its
variance through the inflation: the drive moved, the placement did not.

The innovations of a bay's proximal members are all taken against the node
**before** any of them updates it, and the members have to agree: the rule
fires only when at least one member passes the test and no other member of
the bay has an innovation past ``jump_min_c`` of the opposite sign. One bay
holds one drive, so a swap moves every proximal sensor of the bay the same
way, while two redundant sensors at different placements disagree in opposite
directions for as long as they sit there. Without the agreement test that
standing disagreement re-armed the rule on every tick and latched the bay's
SMART out of the calibration for good (section 8 item 17): on the ``rich``
sim preset the two bays with a redundant pair never calibrated on 7 of 8
seeds and their estimates were 4-8 degC off. A bay with a single proximal
sensor, the only shape the goldens and the DAS example's other bays have, is
unaffected.

A drive pulled and another one pushed in within
``empty_confirm_s`` never passes through ``empty``; this rule makes sigma (hence
the margin) grow at once and lets the filter follow the new drive instead of
explaining the step slowly through ``q``. Inflating the sensor node as well lets
it take the step itself: with the drive alone inflated, the lagging sensor
model turned a pulled drive's falling reading into a drive estimate 7 degC
below the air and a hot insert into one 15 degC above the truth for a tick or
two (seen on the truth simulator). On an ``empty`` bay the same test adds
``nu ** 2`` to the sensor node's variance, so an inserted drive's warming sensor
shows in ``T_s - T_a`` at once (the rising-edge count starts sooner).

Initialisation (per zone, on the first tick with a trusted zone-air sensor):
``T_a`` the mean of the trusted air sensors, ``T_s`` the bay's anchor member's
reading (the hottest trusted member when the anchor is missing, else ``T_a``),
``T_d`` the inverted sensor map at steady
state, ``q`` and ``d_a`` the values that make the model stationary at that
point, so constant readings at a constant command keep the estimate where it
starts. ``P0 = diag(p0_t_air, p0_d_air, p0_t_drive, p0_t_sensor, p0_heat)`` per
state kind (the drive variance only covers the transient: the map offset is
``sigma_cal``), and ``proximal_offset_c ** 2`` per placement offset, which
starts at 0.

**Per bay** (plan section 8 item 69): a bay with no trusted proximal member when
its zone starts is *not* initialised -- its node holds the prior above until its
first trusted reading arrives, which then seeds ``T_s``, ``T_d``, ``q`` and the
bay's offsets exactly as the zone start would have. Without that, the first
reading of a sensor that was missing on the zone's first tick met a node sitting
at the zone air and tripped the fast-swap rule. A bay is seeded once: a sensor
that returns after a *later* loss is an innovation like any other, and the
fast-swap rule is right to widen the bay, since the drive may have been changed
while nothing was watching. A bay seeded from a member that is *not* its anchor
(the anchor was missing too) holds a node placed where that member sits, so the
anchor's own first reading is a step the fast-swap rule fires on -- the
conservative direction, and the trust rule's settling exemption covers it while
the bay stays observed.

Every number of this module that an operator could tune is a key of the config's
``estimator`` section with one documented default there (plan section 8 item 71):
the process noise ``q_*``, the initial covariance ``p0_*``, the fast-swap
``jump_min_c`` and ``jump_sigmas``, the occupancy thresholds and
``occupancy_hold_s``,
``reset_drive_var``, ``sigma_uncalibrated_c`` and the SMART and association keys.
The priors of the physical model (``C_a``, ``leak``, ``kappa``, ``E``, ``g0``, ``k``)
stay module constants: they are the thermal model's, not the operator's.

Output
------
Per constrained bay (occupied or unknown): ``t = T_d``,
``sigma = sqrt(P_dd + sigma_cal^2)``, ``margin = k_sigma * sigma`` and the soft /
hard targets of its class; per zone ``T_a``, ``sigma_air = sqrt(P_aa)``, ``d_a``
and ``drift = max_j |q_j - g_j (T_d,j - T_a) / C_d|`` (degC/min, a validity
metric for the DAS MPC milestone). ``sigma_cal`` is ``sigma_uncalibrated_c``
(1.5 degC by default) uncalibrated and ``max(0.5, EW-RMS residual)`` calibrated;
the filter cannot shrink it.

Per bay the block also carries ``observed`` (a trusted proximal member this tick),
``seeded`` (the bay has had a reading of its own; see *Per bay* above),
``offsets_c`` (the disagreement the filter carries per further proximal member) and
``proximal_map`` (that member's learned ``ds, db``); per zone ``air_blind_s``, the
wall-clock time since a trusted ``zone_air`` reading was last fused (a tick gap counts in
full, at most :data:`MAX_PREDICT_S`).

**One owner, two exemptions** (plan section 8 item 100). "This bay is not itself" used to
be decided twice: here, and again in ``control/solver_das.py`` out of the published sigma.
The estimator is the one that widened the bay and the only one that knows why, so it marks
every reason (:data:`SETTLE_JUMP`, :data:`SETTLE_OCCUPANCY`, :data:`SETTLE_UNCERTAIN`,
:data:`SETTLE_CALIBRATION`) and publishes **two** verdicts per bay, each with its reason
and the seconds it has left:

* ``settling`` / ``settling_reason`` / ``settling_until_s`` / ``settling_spent_s`` /
  ``settling_budget_s`` -- the ``zones.trust_rule: sigma`` exemption. Reasons ``jump`` and
  ``occupancy`` only: a *deliberate widening*, granted while the bay is ``observed``, for
  ``bay_settle_s`` per window and ``bay_settle_max_s`` in total, so readings that keep
  tripping the fast-swap rule cannot stay exempt for ever.
* ``model_exempt`` / ``model_exempt_reason`` / ``model_exempt_until_s`` -- the DAS MPC
  validity gate's. Reasons ``occupancy`` (any change), ``uncertain``
  (``sigma ** 2 - sigma_cal ** 2`` over ``bay_uncertain_var_c2``) and ``calibration``
  (``sigma_cal`` moved by more than ``bay_cal_step_c``), for ``bay_settle_s``, with no
  budget and no ``observed``.

The two lists differ on purpose. The trust rule asks *did the filter widen this bay
deliberately* -- an event. The model gate asks *is this bay tight enough to score a model
against* -- a level, which a bay nobody is reading fails as surely as one just swapped,
and which the trust rule must **not** excuse, because that is the observability loss it
exists to catch. A jump reaches the model gate through ``uncertain``: the jump is what put
the variance there -- except on a bay whose swap verdict the settling budget has just
refused, where the fit that gate is about to score is the one the estimator wanted thrown
away, and the exemption is withheld so it is scored rather than excused (item 124,
:func:`_mark_model_reasons`). ``zones.trust_rule: sigma`` reads ``observed``, ``settling`` and
``air_blind_s``; ``seeded``, ``offsets_c`` and ``proximal_map`` are diagnostics.

Occupancy (``topology.bays.<b>.occupied``; runtime ``POST /api/bay``)
-------------------------------------------------------------------
``true`` is always ``occupied``, ``false`` always ``empty``; ``auto`` runs the
machine below on ``dT = T_s - T_a`` and ``heat = q C_d + g b / s`` (the heat the
filter attributes to the drive beyond what the sensor offset alone explains:
an empty bay has no case-to-drive offset, so with the prior ``b`` its plain
``q C_d`` would never fall below 1 W):

* ``unknown`` at start (and whenever the zone has no estimate yet), and after
  ``occupancy_hold_s`` seconds without any trusted proximal member of the bay
  (loss of observability; a redundant member standing in keeps the state). A
  shorter dropout -- one tick of CRC failures, a clock glitch -- is not evidence
  of anything: the bay keeps its state and its pending counts (item 19). The
  count runs on every blind tick and restarts as soon as a member reports again,
  so ``occupancy_hold_s: 0`` is exactly the undebounced rule;
* ``unknown -> occupied`` at once when ``dT > occupied_dT_c`` or a SMART sample
  of the bay's associated serial arrives;
* ``occupied | unknown -> empty`` after ``empty_confirm_s`` seconds with
  ``dT < empty_dT_c`` **and** ``heat < 1 W`` **and** no fresh SMART from an
  associated serial (the plan's "SMART presence" evidence), every one of them
  on a tick with a trusted zone-air sensor of the bay's zone. Without one,
  ``T_a`` is a prediction that the proximal reading itself pulls along (a
  rising inlet dragged it onto a warm idle drive's sensor in review), so such a
  tick restarts the count (conservative: evidence for a drive still counts);
* ``empty -> occupied`` when ``dT > occupied_dT_c`` on 3 consecutive ticks (or a
  SMART sample of a declared serial arrives), with ``T_d`` reset to ``T_s`` and
  variance ``reset_drive_var`` (25 degC^2) and ``q`` reset to 0: an inserted drive raises the fans
  through its margin within a few ticks. Entering ``unknown`` from ``empty``
  resets the same way.

``unknown`` and ``occupied`` bays both carry constraints. The estimator never faults
a zone over a removed drive; the zone trust groups stay those of the declared config.

A bay whose occupancy crosses the ``empty`` boundary in either direction, or whose
**mean innovation** over its trusted proximal members passes the fast-swap rule's own
thresholds, reports ``swapped: true`` for that one tick: the drive in it may be a
different one from now on. The mean is the point. The per-sensor test above runs
sequentially, so with two members one of them jumps whenever the two disagree --
which is placement, not a swap, and happens on almost every tick of a bay with a
redundant sensor under fan excitation (5689 of 5760 ticks measured on the truth
simulator, the same pathology plan section 3 records for the ``rich`` preset). A swap
moves every member of the bay together and so survives the mean, while a disagreement
between placements cancels in it.

It cancels only if each member is predicted **where the filter says that member sits**
(plan section 8 item 109). Two placements differ by ``ds (T_d - T_a) + db``, so the
mean reading of a pair is *not* the bay's node: it is the node plus the mean of the
members' placement offsets, and the innovation variance of that mean carries the
offsets' variance with it. Taking the mean reading against the bare node instead left
half the pair's disagreement inside the statistic and none of its uncertainty, so a
pair more than ``2 * jump_min_c`` apart -- a gap that *grows with the load*, since it
is proportional to the drive-to-air rise -- was a swap on every tick. Both layouts
now form the same quantity out of the same per-member innovations the rule already
computes: the fused layout's node-plus-offset, the per-sensor layout's own nodes and
maps. A bay with one proximal sensor is one member, so its statistic is unchanged.

**One event, one rate limit** (plan section 8 item 124). The two halves of the rule used
to disagree about both. A per-sensor jump widened the bay's drive variance, opened a
settling window and held the tick's SMART back, all bounded by ``bay_settle_s`` per window
and ``bay_settle_max_s`` in total; the bay-level step reset the whole thermal block with
no budget at all, and on a bay where the mean crossed the thresholds while no single
member did, it reset that block without widening anything. Now:

* a bay-level step **implies** the per-sensor consequences. Where no member of the bay
  was itself a jump, the drive variance is widened by the mean innovation the statistic
  itself used, the settling window opens and the tick's SMART and correlation pair are
  treated exactly as a jump's, so a block the model just threw away can never be scored
  against a drive estimate the filter still calls confident;
* the **model reset** spends the *same* budget as the trust exemption. ``swap_reset`` is
  the ``swapped`` verdict rate-limited to one event per ``bay_settle_s`` (a swap is one
  event, not one per tick it is still visible on) while the bay has exemption left
  (``settling_spent_s < bay_settle_max_s``), and it is what ``mpc.step`` hands to
  :func:`aqua_bridge.control.thermal.update`. ``bay_settle_max_s: 0`` therefore grants
  neither: with no exemption to spend there is no reset either. The rate limit is on the
  *statistical* half only -- an occupancy crossing has the debounce of item 19 for its
  rate limit and is always an event, since refusing one would model an arriving drive
  with the coefficients of the drive that left.

Exhausting the budget falls back to **keeping the fit**, not to resetting it, and says so
(item 123). A bay that trips the rule oftener than the budget allows is not a drive being
swapped three times an hour; resetting it every time is what left a zone at the prior for
ever (item 109's pathology), while a stale fit is judged by the DAS MPC's own
prediction-error gate and lands the solver in its PI-like fallback. For that to be the
answer and not a hope, a bay whose verdict the budget refused has its model-gate
``uncertain`` exemption **taken away** while the refused verdict stands: the bay-level
widening that the verdict itself applies is what would otherwise keep refreshing that
exemption, so the bay would be neither reset nor scored, and the fallback would rest on a
gate that never looked. With the mark withheld the stale fit really is scored
(:func:`_mark_model_reasons`). The record is published beside the verdict: ``swapped``
(this tick's verdict), ``swap_reset`` (the event the reset follows), ``swap_count``,
``swap_last_s``, ``swap_reason`` and ``swap_held`` (events the budget refused, counted at
the cadence an accepted event would have had -- one per ``bay_settle_s``, not one per
tick).

The per-bay output shows a change in progress: ``pending_empty_s`` (seconds of
evidence toward ``empty`` counted so far), ``pending_occupied_ticks`` (ticks of
evidence toward ``occupied`` on an empty bay) and ``pending_unknown_s`` (seconds
of the debounce toward ``unknown`` a blind bay has counted); all three are 0 when
nothing is pending.
The identification experiments (:mod:`aqua_bridge.control.ident`) refuse to start
while either is non-zero in a zone they serve.

Drive class
-----------
``bays.<b>.class`` or ``topology.default_class``, overridden by the SMART model
of the bay's associated serial when a ``drive_classes.<c>.models`` regex
matches it (``re.search``, classes in config order; the first matching class
decides). A serial associated by correlation is a statistical guess, so its
model may only make the class stricter (a limit no higher, then a soft target
no higher); a declared serial may also relax it (deviation from the plan,
conservative: a wrong pair must not raise an HDD bay's limit to an SSD's). The limit is
``min(class limit_c, bays.<b>.limit_c)``.

SMART calibration (per bay and serial)
--------------------------------------
Every new, fresh (``age_s <= smart_max_age_s``) SMART sample of a bay's
associated serial (not on a tick whose proximal reading jumped) whose value lies
within ``smart_reject_c`` of ``T_d`` gives one
row ``T_s - T_a = s (T_smart - T_a) + b`` (posterior estimates) for a
two-parameter RLS without forgetting: prior ``[0.7, -2.1]``, prior covariance
``diag(0.05, 4)`` (ridge to the prior), row variance :data:`CAL_ROW_VAR`, slope
clamped to ``[0.3, 0.98]``. A sample outside ``smart_reject_c`` is dropped and
counted. The calibration is **accepted** with at least 20 fresh samples and a
slope variance below 0.01; from then on the filter uses its ``s, b`` (``T_d``
is re-mapped through the new map at that moment) and ``sigma_cal`` is
``max(0.5, EW-RMS of the drive-equivalent a-priori residuals)``. Entries are
keyed by serial inside the bay: a different serial starts from the prior, the
same serial re-inserted finds its calibration again. **Expiry**: without an
accepted sample for ``calibration_max_age_days`` (or when ``ts`` runs backwards)
the fresh-sample count restarts at 0, so ``sigma_cal`` returns to
``sigma_uncalibrated_c`` while
``s, b`` stay as the starting point; 20 fresh samples confirm it again.

**Restored from the model store** (:func:`restore_calibration`): entries come back per
bay and serial with their ``s, b``, covariance and counts, their time re-based on the
new clock from the stored age of their last accepted sample (an entry whose age is
unknown, negative or past its stored expiry restarts its fresh-sample count, as on
expiry). Entries from a ``stale`` file also carry ``"inflate": 2.0`` and
``"confirm": 20``: a calibrated ``sigma_cal`` is multiplied by ``inflate`` until
``confirm`` SMART samples of that serial have updated the entry (without SMART it
stays inflated), the owner's stale rule.

A new sample is one whose ``ts - age_s`` lies more than :data:`NEW_SAMPLE_EPS_S`
after the previous one of that serial. Serial -> bay association:
:mod:`aqua_bridge.control.associate`.

**Correlation pairs are re-checked** (item 18). A declared serial is the owner's
statement; a correlated one is a guess, and a wrong guess feeds another drive's
SMART into this bay's calibration. The absolute band cannot tell the two apart
(an uncalibrated bay's estimate carries the prior map's own offset, so a *correct*
pair is several degC away too), so the pair is re-tested with the statistic that
accepted it: its SMART history and the bay series keep being recorded, every
evaluation re-scores the pair against its own bay, and ``associate_drop_checks``
consecutive scores below ``associate_drop_corr`` end the
association (``assoc_check_fails`` in the per-bay output counts them). A dropped
pair's history starts over: it has to win the full acceptance rule again, over a
fresh window, before it may calibrate that bay. So does an *accepted* one, so the
window that made the pair cannot sit as the pair's own first re-check. Until a
correlation pair has passed one re-check its calibration is **not used**, however
many samples it has (``calibrated`` stays false, ``sigma_cal`` stays at the
uncalibrated floor and ``calibration.accepted_once`` reads false, so the thermal
identification does not convert a row with that map either):
the first window of a guessed pair is exactly the window the re-check cannot
judge yet, and a wrong map there would bias the drive estimate that sets the fan
speed. A declared serial is used as before.

Manual calibration (PROJECT.md section 8 item 23)
-------------------------------------------------
Where no SMART agent can reach the drives, an operator measures one with a handheld
thermometer and posts it (``POST /api/calibrate {bay, drive_temp_c}``); it arrives
here as ``obs.inputs["calibration"]``, ``{bay: {"temp_c", "ts"}}`` on the
observation clock. It is **keyed by bay, not by serial** -- the operator names the
bay, so there is nothing to associate -- and is otherwise the same sample as a
SMART one: fresh within ``smart_max_age_s``, skipped on a tick whose proximal
reading jumped or for an empty bay, dropped when further than ``smart_reject_c``
from ``T_d``, one RLS row otherwise, and a measurement of ``T_d`` at
``R = SMART_R`` once the entry is accepted -- accepted by the same rule as SMART,
:data:`CAL_MIN_SAMPLES` fresh samples with slope variance below
:data:`CAL_SLOPE_VAR_MAX`, so a single reading changes no estimate. It is counted
in its own ``manual_used`` / ``manual_rejected`` totals, never in the SMART pair,
so the SMART counters stay a diagnostic of the SMART path alone. It is presence
evidence for the occupancy machine exactly like a SMART sample. Its entry lives in
``mem["manual"]`` (never in ``mem["cal"]``, which the model store owns per serial),
so a swapped drive's SMART calibration and the bay's manual one can never be
confused. A bay's associated serial's calibration wins once it is an entry the
filter would use (accepted once, or accepted now) -- a SMART entry still collecting
its first samples leaves the manual map in force instead of dropping the bay back
to the prior; the manual one is what a bay without SMART gets.

**Manual calibrations survive a restart** (section 8 item 104), in their own per-bay
``manual_calibration`` section of the model store, next to the per-serial
``calibration`` one. What comes back, and what does not
(:func:`restore_manual_calibration`) -- a stale calibration silently trusted is worse
than none, so every doubtful case is a *drop*, which puts the bay back on the prior map
at the uncalibrated sigma (more margin, more cooling, never less):

* **old** -- an entry whose last accepted reading is further back than
  ``estimator.manual_calibration_max_age_days`` (wall clock, the outage included), or
  whose age is unknown or negative (a wall clock behind the file), is dropped outright,
  not restored with its fresh count reset the way a SMART entry is. Nothing but the
  operator can refresh a manual map, so an un-freshened one would sit there for ever
  showing ``cal manual`` with no evidence behind it. The window is its own config key
  and shorter than ``calibration_max_age_days`` on purpose: the daemon was not
  watching, and a manual map is keyed by *bay*, so a drive swapped during the outage
  would silently inherit the previous drive's map.
* **swapped** -- the entry records the bay's declaration as it stood when the file was
  written (``occupied``, ``class``, ``serial``). Any of the three different in the
  running config drops it: the owner has said this bay holds a different drive. While
  the daemon runs, the evidenced half of the estimator's own swap rule (item 12 -- the
  bay's occupancy crossing ``empty`` in either direction) drops the bay's manual entry
  on the tick it fires, restored or not. The other half -- a bare proximal step with no
  occupancy crossing -- only *inflates* it (``"inflate": 2.0`` / ``"confirm": 20``): one
  tick's innovation is raised by a spin-up or a fan step as readily as by a swap, and
  the same event leaves a SMART entry alone, so twenty hand readings are not thrown away
  on evidence that thin.
* **reconfigured** -- the store's fingerprint already refuses a file written for
  another structure; the per-entry declaration above covers the policy the fingerprint
  deliberately leaves out.
* **always provisional** -- a restored entry always comes back with ``"inflate": 2.0``
  and ``"confirm": 20``, from a ``fresh`` file as much as from a ``stale`` one. A SMART
  calibration may be trusted at face value out of a fresh file because it is keyed by
  serial and the SMART feed re-associates it; a manual one is keyed by bay and has no
  feed, so its ``sigma_cal`` stays doubled until :data:`CAL_MIN_SAMPLES` fresh hand
  readings of that bay confirm it (indefinitely without them).

Memory (plain JSON)::

    {"v": 1, "fp": <structure fingerprint>, "ts": last ts,
     "zones": {zone: {"x": [...], "P": [[...]], "t_in": float | None, "blind": s}},
     "bays": {bay: {"occ", "low", "rise", "blind", "rej" (failed re-checks),
                    "ver" (a re-check passed), "since", "assoc", "map", "init",
                    "disturb", "why", "spent", "clean",
                    "swaps", "swap_ts", "swap_why", "swap_held", "hold_ts" (the swap
                     record, items 123, 124),
                    "unc", "calstep", "calseen" (the model gate's marks, item 100),
                    "dmap": {sensor: {"th", "P", "n"}} (item 101)}},
     "cal": {bay: {serial: {"th", "P", "n", "fresh", "rms2", "ts", "used"
                            [, "inflate", "confirm"]}}},
     "smart": {serial: {"ts", "t", "model", "hist"}},
     "manual": {bay: {"ts": last sample ts, "cal": calibration entry | None}},
     "series": associate series | None, "scores": {serial: {bay: score}},
     "pending": {bay: [serial, evaluations]}, "next_assoc": ts,
     "count": {"smart_used", "smart_rejected", "manual_used", "manual_rejected"}}

A memory that does not match the config's structure, or is malformed in any
way, starts over (never an exception).
"""

from __future__ import annotations

import json
import math
import re
from collections.abc import Mapping
from dataclasses import dataclass
from typing import Any

import numpy as np

from aqua_bridge.control import associate, fancurve
from aqua_bridge.control.estimates import (
    PRIOR_BETA,
    PRIOR_OFFSET_C,
    SOURCE_ESTIMATOR,
    estimate_entry,
)
from aqua_bridge.model import MpcConfig

__all__ = [
    "C_AIR_J_PER_K",
    "CAL_MIN_SAMPLES",
    "CAL_PRIOR",
    "CAL_PRIOR_VAR",
    "CAL_ROW_VAR",
    "CAL_SLOPE_BOUNDS",
    "CAL_SLOPE_VAR_MAX",
    "E_W_PER_K_PER_FAN",
    "EMPTY",
    "G0_W_PER_K",
    "K_W_PER_K",
    "KAPPA_W_PER_K",
    "LEAK_W_PER_K",
    "MAX_PREDICT_S",
    "NEW_SAMPLE_EPS_S",
    "OCCUPIED",
    "CAL_OFFSET_BOUNDS",
    "SIGMA_CAL_FLOOR_C",
    "UNKNOWN",
    "EstimatorUpdate",
    "calibration_update",
    "drive_capacity",
    "restore_calibration",
    "restore_manual_calibration",
    "update",
]

VERSION = 1

OCCUPIED = "occupied"
EMPTY = "empty"
UNKNOWN = "unknown"

#: Plan section 3 priors.
C_AIR_J_PER_K = 200.0
LEAK_W_PER_K = 1.0
KAPPA_W_PER_K = 3.0
E_W_PER_K_PER_FAN = 33.0
G0_W_PER_K = 0.3
K_W_PER_K = 0.5
TAU_SENSOR_S = 15.0

#: Occupancy: heat below which a bay may be empty, rising-edge ticks.
EMPTY_HEAT_W = 1.0
RISE_TICKS = 3
#: Weight of an empty bay's proximal sensor as an air reading.
EMPTY_WEIGHT = 0.5

#: SMART: measurement variance, new-sample tolerance.
SMART_R = 1.0
NEW_SAMPLE_EPS_S = 1.0

#: Calibration RLS (plan section 2).
CAL_PRIOR = (1.0 - PRIOR_BETA, PRIOR_OFFSET_C)
CAL_PRIOR_VAR = (0.05, 4.0)
CAL_ROW_VAR = 0.25
CAL_SLOPE_BOUNDS = (0.3, 0.98)
#: Offset of an accepted map, degC (the thermal parameter table's bounds on ``b``).
CAL_OFFSET_BOUNDS = (-10.0, 10.0)
#: sigma_cal factor of a calibration restored from a stale model file.
STALE_SIGMA_CAL_FACTOR = 2.0
CAL_MIN_SAMPLES = 20
CAL_SLOPE_VAR_MAX = 0.01
CAL_RMS_ALPHA = 0.1
SIGMA_CAL_FLOOR_C = 0.5
#: Calibrations kept per bay (other serials), oldest dropped first.
CAL_SERIALS_PER_BAY = 4
#: Identity of a manual calibration's sensor map (item 23), where a SMART one uses the
#: drive serial. The parentheses keep it out of the space of real drive serials.
MANUAL_MAP = "(manual)"

#: Why a bay is not itself this tick (item 100). ``JUMP`` and ``OCCUPANCY`` are the
#: estimator's own deliberate widenings and are what the ``sigma`` trust rule may
#: excuse; ``UNCERTAIN`` and ``CALIBRATION`` are the two further reasons the DAS MPC's
#: validity gate must not read as a model error.
SETTLE_JUMP = "jump"
SETTLE_OCCUPANCY = "occupancy"
SETTLE_UNCERTAIN = "uncertain"
SETTLE_CALIBRATION = "calibration"
#: The trust rule's reasons, then the model gate's, each in the order a tie is broken.
TRUST_REASONS: tuple[str, ...] = (SETTLE_JUMP, SETTLE_OCCUPANCY)
MODEL_REASONS: tuple[str, ...] = (SETTLE_OCCUPANCY, SETTLE_UNCERTAIN, SETTLE_CALIBRATION)
_SETTLE_REASONS = frozenset(TRUST_REASONS) | frozenset(MODEL_REASONS)

#: Longest interval one prediction covers, seconds.
MAX_PREDICT_S = 3600.0

_TAYLOR_ORDER = 10
_DAY_S = 86400.0


@dataclass(frozen=True)
class EstimatorUpdate:
    """What :func:`update` returns.

    * ``estimates`` -- the estimates block (constrained bays of initialised zones)
    * ``memory``    -- the next ``solver_memory["estimator"]``
    * ``zones``     -- per zone: air estimate, its sigma, disturbance, drift, inlet,
      airflow and the curve that produced it (``airflow_curve``: ``fit`` | ``config``
      | ``mixed``, when only some of the zone's fan models have a fitted curve)
    * ``bays``      -- per bay: occupancy, class, association, calibration, candidates
    * ``summary``   -- SMART counters and unassigned serials
    """

    estimates: dict[str, dict[str, Any]]
    memory: dict[str, Any]
    zones: dict[str, dict[str, Any]]
    bays: dict[str, dict[str, Any]]
    summary: dict[str, Any]


# ---------------------------------------------------------------------------
# structure derived from the config
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class _Bay:
    name: str
    zone: str
    index: int
    sensors: tuple[str, ...]
    #: The member the bay's sensor node is anchored on (the first that is not
    #: ``redundant``): the bay's sensor map is that node's map, and in the fused layout
    #: its lag is the node's and it is the one member without a placement offset.
    primary: str
    tau_s: float
    #: Every proximal member beyond the first -> its index in the zone's offset block
    #: (the first member anchors the node, so it has no offset; empty in the per-sensor
    #: layout and when ``estimator.proximal_offset_c`` is 0).
    offsets: dict[str, int]
    #: Every proximal member -> the index of the sensor node it reads, inside the zone's
    #: sensor block. Fused layout: every member reads the bay's one node (``index``).
    #: Per-sensor layout (``estimator.proximal_slope_spread > 0``): one node each.
    nodes: dict[str, int]
    #: The bay's own node: the anchor's. ``T_s`` of the estimates block and every
    #: per-bay rule that needs one temperature (occupancy, association, calibration).
    node: int


@dataclass(frozen=True)
class _Zone:
    name: str
    bays: tuple[str, ...]
    air: tuple[str, ...]
    inlet: tuple[str, ...]
    e_w_per_k: dict[str, float]
    coupled: tuple[str, ...]
    #: Placement-offset states of the zone (``_Bay.offsets`` over its bays).
    n_offsets: int
    #: Sensor-node states of the zone: one per bay (fused) or one per proximal member.
    n_nodes: int
    #: Per sensor node: the index of its bay in ``bays``, its lag, and the member it
    #: belongs to (``None`` for a fused node, which every member of its bay reads).
    node_bay: tuple[int, ...]
    node_tau: tuple[float, ...]
    node_sensor: tuple[str | None, ...]

    @property
    def dim(self) -> int:
        """States: ``T_a, d_a, T_d(n), T_s(n_nodes), q(n), c(n_offsets)``."""
        return 2 + 2 * len(self.bays) + self.n_nodes + self.n_offsets


@dataclass(frozen=True)
class _Structure:
    zones: dict[str, _Zone]
    bays: dict[str, _Bay]
    fingerprint: str


#: Config-derived structure, keyed by config identity (``MpcConfig`` is frozen and the
#: loop keeps one effective config between intents), exactly as
#: :data:`aqua_bridge.control.thermal._DERIVED_CACHE`. A pure memo: same result as
#: recomputing, only cheaper per tick (item 73).
_STRUCTURE_CACHE: dict[int, tuple[MpcConfig, _Structure]] = {}
_STRUCTURE_CACHE_MAX = 16


def _structure(cfg: MpcConfig) -> _Structure:
    hit = _STRUCTURE_CACHE.get(id(cfg))
    if hit is not None and hit[0] is cfg:
        return hit[1]
    st = _build_structure(cfg)
    while len(_STRUCTURE_CACHE) >= _STRUCTURE_CACHE_MAX:
        del _STRUCTURE_CACHE[next(iter(_STRUCTURE_CACHE))]
    _STRUCTURE_CACHE[id(cfg)] = (cfg, st)
    return st


def _build_structure(cfg: MpcConfig) -> _Structure:
    topo = cfg.topology
    assert topo is not None
    spec_est = cfg.estimator
    per_sensor = spec_est is not None and spec_est.proximal_slope_spread > 0
    with_offsets = not per_sensor and spec_est is not None and spec_est.proximal_offset_c > 0
    sensors = cfg.sensors
    zones: dict[str, _Zone] = {}
    bays: dict[str, _Bay] = {}
    inlets = tuple(t for t in cfg.temps if sensors[t].role == "inlet")
    for z, spec in topo.zones.items():
        zone_bays = tuple(b for b, bay in topo.bays.items() if bay.zone == z)
        n_offsets = 0
        node_bay: list[int] = []
        node_tau: list[float] = []
        node_sensor: list[str | None] = []
        for i, b in enumerate(zone_bays):
            members = tuple(
                t for t in cfg.temps if sensors[t].role == "drive_proximal" and sensors[t].bay == b
            )
            primary = next((t for t in members if not sensors[t].redundant), members[0])
            tau = _tau(sensors[primary].tau_s)
            offsets: dict[str, int] = {}
            if with_offsets:
                for name in members:
                    if name != primary:
                        offsets[name] = n_offsets
                        n_offsets += 1
            if per_sensor:
                # One node per proximal member (item 101): its own lag, its own map.
                nodes = {}
                anchor = len(node_bay)
                for name in members:
                    if name == primary:
                        anchor = len(node_bay)
                    nodes[name] = len(node_bay)
                    node_bay.append(i)
                    node_tau.append(_tau(sensors[name].tau_s))
                    node_sensor.append(name)
            else:  # one node per bay, anchored on ``primary`` (item 67)
                anchor = len(node_bay)
                nodes = dict.fromkeys(members, anchor)
                node_bay.append(i)
                node_tau.append(tau)
                node_sensor.append(None)
            bays[b] = _Bay(b, z, i, members, primary, tau, offsets, nodes, anchor)
        e: dict[str, float] = {}
        for ch in spec.channels:
            listed = sum(1 for other in topo.zones.values() if ch in other.channels)
            e[ch] = E_W_PER_K_PER_FAN * cfg.fans[ch].count / listed
        zones[z] = _Zone(
            name=z,
            bays=zone_bays,
            air=tuple(
                t for t in cfg.temps if sensors[t].role == "zone_air" and sensors[t].zone == z
            ),
            inlet=inlets if spec.inlet == "mix" else (spec.inlet,),
            e_w_per_k=e,
            coupled=tuple(spec.coupled_to),
            n_offsets=n_offsets,
            n_nodes=len(node_bay),
            node_bay=tuple(node_bay),
            node_tau=tuple(node_tau),
            node_sensor=tuple(node_sensor),
        )
    fingerprint = json.dumps(
        [
            [z, list(zs.bays), list(zs.air), zs.n_offsets, list(zs.node_sensor)]
            for z, zs in zones.items()
        ]
        + [[b, list(bs.sensors)] for b, bs in bays.items()],
        separators=(",", ":"),
    )
    return _Structure(zones=zones, bays=bays, fingerprint=fingerprint)


def _tau(value: float | None) -> float:
    """A sensor's lag, seconds: its ``tau_s`` or the default."""
    return float(value) if value else TAU_SENSOR_S


def drive_capacity(cfg: MpcConfig, drive_class: str) -> float:
    """``C_d`` prior of a class, J/K: ``tau_d_s * (g0 + k)`` (module docstring)."""
    return cfg.drive_classes[drive_class].tau_d_s * (G0_W_PER_K + K_W_PER_K)


def _airflow(
    cfg: MpcConfig,
    zone: _Zone,
    u: Mapping[str, float],
    curves: Mapping[str, Any] | None = None,
) -> tuple[float, float, str]:
    """``(Q_z, Qn_z, the curve behind them)`` at command ``u``.

    ``curves`` is ``solver_memory["fan_curves"]`` (the online fit,
    :mod:`aqua_bridge.control.fancurve`): a usable entry replaces its fan model's
    ``deadband`` / ``exponent``, anything else keeps the configured pair. The estimator
    follows the fit for the same reason the thermal model does -- an estimator planning on
    one airflow while the model plans on another is a disagreement the MPC's
    prediction-error guard answers with a fallback, which is safe but louder than it needs
    to be (PROJECT.md section 8 item 107).

    The third value is ``fit``, ``config`` or -- for a zone whose channels are of several
    fan models and only some of them have a usable curve -- ``mixed``, so nobody reads
    "the estimator is on the fit" off a zone most of whose airflow is still the
    configured curve (item 107: which curve produced which number).
    """
    q = 0.0
    total = 0.0
    fitted = 0
    channels = 0
    for ch, e in zone.e_w_per_k.items():
        name = cfg.fans[ch].model
        model = cfg.fan_models[name]
        deadband, exponent = float(model.deadband), float(model.exponent)
        channels += 1
        if curves is not None:
            deadband, exponent, _ = fancurve.curve_pair(
                curves.get(name), deadband, exponent, model.rpm_max
            )
            fitted += 1 if fancurve.usable(curves.get(name)) else 0
        value = u.get(ch)
        pwm = float(value) if _finite(value) else 0.0
        frac = min(1.0, max(0.0, (pwm - deadband) / (1.0 - deadband)))
        q += e * frac**exponent
        total += e
    if fitted == 0:
        source = "config"
    elif fitted == channels:
        source = "fit"
    else:
        source = "mixed"
    return q, (q / total if total > 0 else 0.0), source


# ---------------------------------------------------------------------------
# small numerics
# ---------------------------------------------------------------------------


def _finite(value: object) -> bool:
    if type(value) is float:  # the common case, checked first (hot path)
        return math.isfinite(value)
    return (
        isinstance(value, int | float)
        and not isinstance(value, bool)
        and math.isfinite(float(value))
    )


def _expm(m: np.ndarray) -> np.ndarray:
    """Matrix exponential by scaling and squaring with a Taylor polynomial."""
    norm = float(np.abs(m).sum(axis=1).max()) if m.size else 0.0
    squarings = 0 if norm <= 0.5 else int(math.ceil(math.log2(norm / 0.5)))
    x = m / (2.0**squarings)
    eye = np.eye(m.shape[0])
    e = eye.copy()
    for k in range(_TAYLOR_ORDER, 0, -1):
        e = eye + (x @ e) / k
    for _ in range(squarings):
        e = e @ e
    return e


def _discretise(a: np.ndarray, c: np.ndarray, h: float) -> tuple[np.ndarray, np.ndarray]:
    """``(Phi, gamma)`` of ``x(t + h) = Phi x(t) + gamma`` for ``dx/dt = A x + c``."""
    n = a.shape[0]
    m = np.zeros((n + 1, n + 1))
    m[:n, :n] = a * h
    col = c * h
    norm_a = max(float(np.abs(m[:n, :n]).sum(axis=1).max()), 1e-12)
    scale = max(1.0, float(np.abs(col).max()) / norm_a)
    m[:n, n] = col / scale
    e = _expm(m)
    return e[:n, :n], e[:n, n] * scale


def _scalar_update(x: np.ndarray, p: np.ndarray, i: int, z: float, r: float) -> None:
    """Joseph-form update of ``x``, ``p`` (in place) with one measurement of state ``i``."""
    s = p[i, i] + r
    if not s > 0:
        return
    k = p[:, i] / s
    x += k * (z - x[i])
    # ``a[:, None] * b`` is the elementwise product ``numpy.outer`` computes, without its
    # wrapper (this runs ~20 times a tick; item 73).
    a1 = p - k[:, None] * p[i, :]  # (I - K H) P
    joseph = a1 - a1[:, i][:, None] * k + r * (k[:, None] * k)  # ... (I - K H)^T + K R K^T
    np.add(joseph, joseph.T, out=p)
    p *= 0.5


def _reset_state(x: np.ndarray, p: np.ndarray, i: int, value: float, var: float) -> None:
    """Re-seed state ``i`` at ``value`` with variance ``var`` and no correlations."""
    x[i] = value
    p[i, :] = 0.0
    p[:, i] = 0.0
    p[i, i] = var


def _pair_update(x: np.ndarray, p: np.ndarray, i: int, j: int, z: float, r: float) -> None:
    """Joseph-form update of ``x``, ``p`` (in place) with one measurement of ``x[i] + x[j]``.

    ``H = e_i + e_j``: a proximal sensor reads its bay's sensor node plus its own placement
    offset. With ``j`` absent this is :func:`_scalar_update`.
    """
    hp = p[i, :] + p[j, :]  # H P
    s = hp[i] + hp[j] + r
    if not s > 0:
        return
    k = hp / s
    x += k * (z - x[i] - x[j])
    a1 = p - k[:, None] * hp  # (I - K H) P
    joseph = a1 - (a1[:, i] + a1[:, j])[:, None] * k + r * (k[:, None] * k)
    np.add(joseph, joseph.T, out=p)
    p *= 0.5


def _sensor_var(cfg: MpcConfig, name: str) -> float:
    quant = cfg.sensors[name].quant_c
    assert cfg.estimator is not None
    return cfg.estimator.sensor_noise_c**2 + quant * quant / 12.0


# ---------------------------------------------------------------------------
# memory
# ---------------------------------------------------------------------------


def _fresh_memory(st: _Structure) -> dict[str, Any]:
    return {
        "v": VERSION,
        "fp": st.fingerprint,
        "ts": None,
        "zones": {},
        "bays": {b: _fresh_bay() for b in st.bays},
        "cal": {},
        "smart": {},
        "manual": {},
        "series": None,
        "scores": {},
        "pending": {},
        "next_assoc": None,
        "count": {"smart_used": 0, "smart_rejected": 0, "manual_used": 0, "manual_rejected": 0},
    }


def _fresh_bay() -> dict[str, Any]:
    return {
        "occ": UNKNOWN,
        "low": 0.0,
        "rise": 0,
        "blind": 0.0,
        "rej": 0,
        "ver": False,
        "since": None,
        "assoc": None,
        "map": None,
        "init": False,
        "disturb": None,
        "why": None,
        "spent": 0.0,
        "clean": 0.0,
        # items 123, 124: the swap record -- declared events, when the last one was, why,
        # and how many verdicts the rate limit refused
        "swaps": 0,
        "swap_ts": None,
        "swap_why": None,
        "swap_held": 0,
        # item 124: when the last *refused* event was counted, so a refusal is counted at
        # the cadence an accepted event would have had and ``swap_held`` is a count of
        # events, not of ticks
        "hold_ts": None,
        # item 100: the model gate's own marks, beside the trust rule's ``disturb``
        "unc": None,
        "calstep": None,
        "calseen": None,
        # item 101: the learned map of every proximal member beyond the anchor
        "dmap": {},
    }


def _load(memory: object, st: _Structure) -> dict[str, Any]:
    """A validated deep copy of ``memory``, or a fresh memory (module docstring)."""
    try:
        return _parse(memory, st)
    except (TypeError, ValueError, KeyError, AttributeError, IndexError):
        return _fresh_memory(st)


def _num(value: object) -> float:
    if not _finite(value):
        raise ValueError("not a finite number")
    return float(value)  # type: ignore[arg-type]


def _opt_num(value: object) -> float | None:
    return None if value is None else _num(value)


def _parse(memory: object, st: _Structure) -> dict[str, Any]:
    if not isinstance(memory, Mapping) or memory.get("v") != VERSION:
        raise ValueError("version")
    if memory.get("fp") != st.fingerprint:
        raise ValueError("structure changed")
    out = _fresh_memory(st)
    out["ts"] = _opt_num(memory.get("ts"))
    for z, raw in dict(memory.get("zones") or {}).items():
        zone = st.zones[z]
        n = zone.dim
        x = np.array(raw["x"], dtype=float)
        p = np.array(raw["P"], dtype=float)
        if x.shape != (n,) or p.shape != (n, n):
            raise ValueError("shape")
        if not (np.all(np.isfinite(x)) and np.all(np.isfinite(p))):
            raise ValueError("non-finite filter state")
        blind = _num(raw.get("blind", 0.0))
        if blind < 0:
            raise ValueError("blind")
        out["zones"][z] = {
            "x": x,
            "P": 0.5 * (p + p.T),
            "t_in": _opt_num(raw.get("t_in")),
            "blind": blind,
        }
    for b, raw in dict(memory.get("bays") or {}).items():
        if b not in st.bays:
            raise KeyError(b)
        occ = raw["occ"]
        if occ not in (OCCUPIED, EMPTY, UNKNOWN):
            raise ValueError("occupancy")
        assoc = raw.get("assoc")
        if assoc is not None and not isinstance(assoc, str):
            raise TypeError("assoc")
        mapping = raw.get("map")
        if mapping is not None and not isinstance(mapping, str):
            raise TypeError("map")
        rise = raw.get("rise", 0)
        if isinstance(rise, bool) or not isinstance(rise, int) or rise < 0:
            raise ValueError("rise")
        spent, clean = _num(raw.get("spent", 0.0)), _num(raw.get("clean", 0.0))
        if spent < 0 or clean < 0:
            raise ValueError("settling budget")
        blind = _num(raw.get("blind", 0.0))
        if blind < 0:
            raise ValueError("blind")
        rej = raw.get("rej", 0)
        if isinstance(rej, bool) or not isinstance(rej, int) or rej < 0:
            raise ValueError("rej")
        swaps, held = raw.get("swaps", 0), raw.get("swap_held", 0)
        for count in (swaps, held):
            if isinstance(count, bool) or not isinstance(count, int) or count < 0:
                raise ValueError("swap count")
        out["bays"][b] = {
            "occ": occ,
            "low": _num(raw.get("low", 0.0)),
            "rise": rise,
            "blind": blind,
            "rej": rej,
            "ver": bool(raw.get("ver", False)),
            "since": _opt_num(raw.get("since")),
            "assoc": assoc,
            "map": mapping,
            # A memory written before the per-bay initialisation existed has every bay
            # of an initialised zone seeded, which is what ``True`` says.
            "init": bool(raw.get("init", True)),
            "disturb": _opt_num(raw.get("disturb")),
            "why": _settle_reason(raw.get("why")),
            "spent": spent,
            "clean": clean,
            "swaps": swaps,
            "swap_ts": _opt_num(raw.get("swap_ts")),
            "swap_why": _settle_reason(raw.get("swap_why")),
            "swap_held": held,
            "hold_ts": _opt_num(raw.get("hold_ts")),
            "unc": _opt_num(raw.get("unc")),
            "calstep": _opt_num(raw.get("calstep")),
            "calseen": _opt_num(raw.get("calseen")),
            "dmap": {
                str(name): _parse_dmap_entry(entry)
                for name, entry in dict(raw.get("dmap") or {}).items()
                if name in st.bays[b].sensors
            },
        }
    for b, per_serial in dict(memory.get("cal") or {}).items():
        if b not in st.bays:
            raise KeyError(b)
        entries = {}
        for serial, raw in dict(per_serial).items():
            if not isinstance(serial, str):
                raise TypeError("serial")
            entries[serial] = _parse_calibration_entry(raw)
        out["cal"][b] = entries
    for serial, raw in dict(memory.get("smart") or {}).items():
        if not isinstance(serial, str):
            raise TypeError("serial")
        model = raw.get("model")
        if model is not None and not isinstance(model, str):
            raise TypeError("model")
        hist = raw.get("hist") or []
        if not isinstance(hist, list):
            raise TypeError("hist")
        # Shared, never mutated: the association helpers build new lists and check
        # every value they read (a malformed history only loses its scores).
        out["smart"][serial] = {
            "ts": _num(raw["ts"]),
            "t": _num(raw["t"]),
            "model": model,
            "hist": hist,
        }
    for b, raw in dict(memory.get("manual") or {}).items():
        if b not in st.bays:
            raise KeyError(b)
        if not isinstance(raw, Mapping):
            raise TypeError("manual")
        cal = raw.get("cal")
        out["manual"][b] = {
            "ts": _num(raw["ts"]),
            "cal": None if cal is None else _parse_calibration_entry(cal),
        }
    series = memory.get("series")
    if series is not None and not isinstance(series, Mapping):
        raise TypeError("series")
    out["series"] = series  # shared, never mutated (record_series builds a new one)
    scores = memory.get("scores") or {}
    out["scores"] = {
        str(s): {str(b): _num(v) for b, v in dict(row).items()} for s, row in dict(scores).items()
    }
    pending = memory.get("pending") or {}
    out["pending"] = {
        str(b): [str(v[0]), int(v[1])]
        for b, v in dict(pending).items()
        if b in st.bays and isinstance(v, list) and len(v) == 2 and isinstance(v[1], int)
    }
    out["next_assoc"] = _opt_num(memory.get("next_assoc"))
    count = memory.get("count") or {}
    for key in ("smart_used", "smart_rejected", "manual_used", "manual_rejected"):
        value = count.get(key, 0)
        if isinstance(value, bool) or not isinstance(value, int) or value < 0:
            raise ValueError("counter")
        out["count"][key] = value
    return out


def _settle_reason(value: object) -> str | None:
    if value is None:
        return None
    if value not in _SETTLE_REASONS:
        raise ValueError("settling reason")
    return str(value)


def _fresh_dmap(spec: Any) -> dict[str, Any]:
    """A proximal member's map difference before any evidence: ``(0, 0)`` at the config's
    prior spread (item 101). The prior *is* the statement that two members of one bay
    read alike; the spread says how far apart their placements may put them."""
    return {
        "th": [0.0, 0.0],
        "P": [[spec.proximal_slope_spread**2, 0.0], [0.0, spec.proximal_offset_c**2]],
        "n": 0,
    }


def _parse_dmap_entry(raw: object) -> dict[str, Any]:
    """One stored ``(delta slope, delta offset)`` entry; raises on any malformed field."""
    if not isinstance(raw, Mapping):
        raise TypeError("dmap entry")
    th = [_num(v) for v in raw["th"]]
    p = [[_num(v) for v in row] for row in raw["P"]]
    n_rows = raw["n"]
    if len(th) != 2 or len(p) != 2 or any(len(row) != 2 for row in p):
        raise ValueError("dmap shape")
    if isinstance(n_rows, bool) or not isinstance(n_rows, int) or n_rows < 0:
        raise ValueError("dmap count")
    return {"th": th, "P": p, "n": n_rows}


def _dmap_row_var(entry: Mapping[str, Any], rise: float) -> float:
    """``h P h^T`` of the member's map at this tick's drive-to-air rise: how uncertain the
    filter is about where this sensor sits, in degC^2 on its reading."""
    p = entry["P"]
    return rise * (rise * p[0][0] + p[0][1]) + rise * p[1][0] + p[1][1]


def _dmap_bound(
    entry: dict[str, Any],
    rise: float,
    ds: tuple[float, float],
    db: tuple[float, float],
) -> None:
    """Hold ``(ds, db)`` inside the box the config and the bay's own map allow, sliding it
    along the one direction its rows cannot see (item 101).

    A row ``z = ds * rise + db`` identifies only the *combination*: ``(ds, db)`` may walk
    along ``(1, -rise)`` for ever and still fit every reading it has ever taken, and under
    regulation the rise barely moves, so that is what it does. Left alone the pair leaves
    the box and the map has to be truncated one coordinate at a time, which throws the
    combination away with it -- the member then predicts a temperature no reading ever
    supported. Sliding it back along the same null direction keeps the member predicting
    exactly what the evidence says it reads and changes only the *split* between slope and
    offset; the componentwise clip afterwards is the fallback for a line that misses the
    box entirely (no rise, or a box too narrow for the combination).
    """
    th = entry["th"]
    lo, hi = ds[0] - th[0], ds[1] - th[0]
    if rise > 0.0:
        lo, hi = max(lo, (th[1] - db[1]) / rise), min(hi, (th[1] - db[0]) / rise)
    elif rise < 0.0:
        lo, hi = max(lo, (th[1] - db[0]) / rise), min(hi, (th[1] - db[1]) / rise)
    elif not db[0] <= th[1] <= db[1]:
        lo, hi = 1.0, -1.0  # no rise: nothing to slide along
    if lo <= hi:
        t = min(max(0.0, lo), hi)
        th[0] += t
        th[1] -= t * rise
    th[0] = min(max(th[0], ds[0]), ds[1])
    th[1] = min(max(th[1], db[0]), db[1])


def _dmap_absorb(
    entry: dict[str, Any],
    rise: float,
    z: float,
    r: float,
    *,
    q: float,
    ds: tuple[float, float],
    db: tuple[float, float],
) -> None:
    """One RLS row ``z = ds * rise + db`` (item 101), Joseph not needed (the two parameters
    are the whole state and ``P`` stays symmetric by construction).

    ``q`` is the *offset* difference's random walk for this row, ``estimator.q_offset``
    scaled to the tick, and it is the one thing that keeps this from being strictly worse
    than the fused layout it replaces: there the disagreement was a random-walk state, and
    fouling, a loosening sensor and a thermistor ageing all move it. Without ``q`` the
    RLS has no forgetting, ``P`` collapses and a drifting placement can never be followed
    again -- the residual would have nowhere to go but the shared drive estimate, which is
    a failure that *reduces* cooling (plan section 2). The slope difference gets none: a
    placement's geometry does not drift, which is also why its own bound below is the
    config's prior spread. ``ds`` / ``db`` are the box :func:`_dmap_bound` holds the pair
    in, both sides of each.
    """
    th, p = entry["th"], entry["P"]
    p[1][1] += q
    ph = (rise * p[0][0] + p[0][1], rise * p[1][0] + p[1][1])
    s = rise * ph[0] + ph[1] + r
    if not s > 0:
        return
    k = (ph[0] / s, ph[1] / s)
    nu = z - (th[0] * rise + th[1])
    th[0] += k[0] * nu
    th[1] += k[1] * nu
    p00 = p[0][0] - k[0] * ph[0]
    p11 = p[1][1] - k[1] * ph[1]
    off = 0.5 * ((p[0][1] - k[0] * ph[1]) + (p[1][0] - k[1] * ph[0]))
    entry["P"] = [[p00, off], [off, p11]]
    entry["n"] += 1
    _dmap_bound(entry, rise, ds, db)


def _parse_calibration_entry(raw: object) -> dict[str, Any]:
    """One stored calibration entry (SMART or manual); raises on any malformed field."""
    if not isinstance(raw, Mapping):
        raise TypeError("calibration entry")
    th = [_num(v) for v in raw["th"]]
    p = [[_num(v) for v in row] for row in raw["P"]]
    n_all, fresh = raw["n"], raw["fresh"]
    if len(th) != 2 or len(p) != 2 or any(len(row) != 2 for row in p):
        raise ValueError("calibration shape")
    for count in (n_all, fresh):
        if isinstance(count, bool) or not isinstance(count, int) or count < 0:
            raise ValueError("count")
    entry: dict[str, Any] = {
        "th": th,
        "P": p,
        "n": n_all,
        "fresh": fresh,
        "rms2": _num(raw["rms2"]),
        "ts": _opt_num(raw.get("ts")),
        "used": bool(raw.get("used", False)),
    }
    if "inflate" in raw:
        inflate, confirm = _num(raw["inflate"]), raw.get("confirm")
        if inflate < 1.0 or isinstance(confirm, bool) or not isinstance(confirm, int):
            raise ValueError("inflate")
        entry["inflate"] = inflate
        entry["confirm"] = confirm
    return entry


def _parse_manual(
    raw: object, bays: Mapping[str, Any], ts: float, max_age_s: float
) -> dict[str, tuple[float, float]]:
    """Fresh manual calibrations ``{bay: (sample_ts, temp_c)}`` (module docstring).

    ``obs.inputs["calibration"]`` is ``{bay: {"temp_c", "ts"}}`` on the observation
    clock. An unknown bay, a malformed entry, a sample time in the future or older
    than ``max_age_s`` is ignored -- like SMART, a manual reading can only ever add
    information and must never raise into ``step``.
    """
    out: dict[str, tuple[float, float]] = {}
    if not isinstance(raw, Mapping):
        return out
    for bay, entry in raw.items():
        if bay not in bays or not isinstance(entry, Mapping):
            continue
        temp, sample_ts = entry.get("temp_c"), entry.get("ts")
        if not _finite(temp) or not _finite(sample_ts):
            continue
        age = float(ts) - float(sample_ts)  # type: ignore[arg-type]
        if age < 0 or age > max_age_s:
            continue
        out[str(bay)] = (float(sample_ts), float(temp))  # type: ignore[arg-type]
    return dict(sorted(out.items()))


def _parse_smart(raw: object, max_age_s: float) -> dict[str, tuple[float, float, str | None]]:
    """Fresh SMART samples ``{serial: (temp_c, age_s, model)}``; malformed entries are ignored."""
    out: dict[str, tuple[float, float, str | None]] = {}
    if not isinstance(raw, Mapping):
        return out
    for serial, entry in raw.items():
        if not isinstance(serial, str) or not serial or not isinstance(entry, Mapping):
            continue
        temp, age, model = entry.get("temp_c"), entry.get("age_s"), entry.get("model")
        if not _finite(temp) or not _finite(age) or float(age) < 0 or float(age) > max_age_s:  # type: ignore[arg-type]
            continue
        out[serial] = (float(temp), float(age), model if isinstance(model, str) else None)  # type: ignore[arg-type]
    return dict(sorted(out.items()))


# ---------------------------------------------------------------------------
# calibration
# ---------------------------------------------------------------------------


def _fresh_calibration(sigma_uncalibrated_c: float) -> dict[str, Any]:
    """A bay-and-serial calibration entry at the prior; ``rms2`` starts at the
    uncalibrated floor (``estimator.sigma_uncalibrated_c``) squared."""
    return {
        "th": list(CAL_PRIOR),
        "P": [[CAL_PRIOR_VAR[0], 0.0], [0.0, CAL_PRIOR_VAR[1]]],
        "n": 0,
        "fresh": 0,
        "rms2": float(sigma_uncalibrated_c) ** 2,
        "ts": None,
        "used": False,
    }


def calibration_update(
    entry: Mapping[str, Any], x: float, y: float, ts: float
) -> tuple[dict[str, Any], float]:
    """One RLS row ``y = s x + b`` (module docstring); returns the new entry and the
    drive-equivalent a-priori residual. Pure."""
    th = np.array(entry["th"], dtype=float)
    p = np.array(entry["P"], dtype=float)
    phi = np.array([x, 1.0])
    residual = y - float(phi @ th)
    s = CAL_ROW_VAR + float(phi @ p @ phi)
    k = (p @ phi) / s
    th = th + k * residual
    ikh = np.eye(2) - np.outer(k, phi)
    p = ikh @ p @ ikh.T + CAL_ROW_VAR * np.outer(k, k)
    p = 0.5 * (p + p.T)
    th[0] = min(CAL_SLOPE_BOUNDS[1], max(CAL_SLOPE_BOUNDS[0], float(th[0])))
    slope_before = min(CAL_SLOPE_BOUNDS[1], max(CAL_SLOPE_BOUNDS[0], float(entry["th"][0])))
    drive_residual = residual / slope_before
    rms2 = (1.0 - CAL_RMS_ALPHA) * float(entry["rms2"]) + CAL_RMS_ALPHA * drive_residual**2
    out = {
        "th": [float(th[0]), float(th[1])],
        "P": p.tolist(),
        "n": int(entry["n"]) + 1,
        "fresh": int(entry["fresh"]) + 1,
        "rms2": rms2,
        "ts": float(ts),
        "used": bool(entry.get("used", False)),
    }
    if "inflate" in entry:  # a stale restored calibration: confirmed by this many samples
        confirm = int(entry.get("confirm", 0)) - 1
        if confirm > 0:
            out["inflate"] = float(entry["inflate"])
            out["confirm"] = confirm
    return out, drive_residual


def _absorb_sample(
    mem: dict[str, Any],
    spec: Any,
    arrays: tuple[np.ndarray, np.ndarray],
    idx: tuple[int, int],
    entry: Mapping[str, Any],
    sample: tuple[float, float],
    ts: float,
    max_age_cal: float,
    kind: str = "smart",
    *,
    gate: bool,
) -> dict[str, Any] | None:
    """One drive-temperature sample into a bay's calibration RLS (module docstring).

    The single rule both a SMART sample and a manual reading (item 23) follow:
    ``None`` (counted as rejected) when the value is further than
    ``smart_reject_c`` from ``T_d``; otherwise the updated entry. ``kind``
    (``"smart"`` or ``"manual"``) picks the counter pair the sample is told
    in, so the SMART totals stay a diagnostic of the SMART path alone.
    ``gate`` withholds ``used`` and the filter's drive-temperature measurement
    even once the entry is accepted (SMART's correlation-pair re-check, item
    18): a wrong guess would otherwise feed another drive's reading straight
    into this bay's estimate. A manual reading has no association to
    re-check, so it always gates true.
    """
    x, p = arrays
    i_d, i_s = idx
    sample_ts, temp = sample
    if abs(temp - x[i_d]) > spec.smart_reject_c:
        mem["count"][f"{kind}_rejected"] += 1
        return None
    mem["count"][f"{kind}_used"] += 1
    out, _ = calibration_update(entry, temp - x[0], x[i_s] - x[0], sample_ts)
    if gate and _calibrated(out, float(ts), max_age_cal):
        out["used"] = True
        _scalar_update(x, p, i_d, temp, SMART_R)
    return out


def _calibrated(entry: Mapping[str, Any] | None, ts: float, max_age_s: float) -> bool:
    if entry is None or entry["ts"] is None:
        return False
    age = ts - float(entry["ts"])
    return (
        int(entry["fresh"]) >= CAL_MIN_SAMPLES
        and float(entry["P"][0][0]) < CAL_SLOPE_VAR_MAX
        and 0.0 <= age <= max_age_s
    )


# ---------------------------------------------------------------------------
# the update
# ---------------------------------------------------------------------------


def _strictness(cfg: MpcConfig, drive_class: str) -> tuple[float, float]:
    """Sort key of a class: lower is stricter (limit, then soft target without margin)."""
    dc = cfg.drive_classes[drive_class]
    return dc.limit_c, dc.limit_c - dc.comfort_c


def _drive_class(
    cfg: MpcConfig, bay: str, model: str | None, *, may_relax: bool = True
) -> tuple[str, str]:
    """``(class, source)``: ``smart_model`` | ``declared`` | ``default``.

    With ``may_relax`` false (a serial associated by correlation, module docstring)
    a SMART model match is used only when its class is at least as strict as the
    bay's declared (or default) class.
    """
    assert cfg.topology is not None
    base = cfg.bay_class(bay)
    if model:
        for name, dc in cfg.drive_classes.items():
            if any(re.search(pattern, model) for pattern in dc.models):
                if may_relax or _strictness(cfg, name) <= _strictness(cfg, base):
                    return name, "smart_model"
                break
    if cfg.topology.bays[bay].drive_class is not None:
        return base, "declared"
    return base, "default"


def _slots(zone: _Zone, bay: _Bay) -> tuple[int, int, int]:
    """``(i_d, i_s, i_q)`` of one bay inside its zone's state vector: its drive, the
    sensor node the bay is read through (the anchor's) and its heat."""
    n = len(zone.bays)
    return 2 + bay.index, 2 + n + bay.node, 2 + n + zone.n_nodes + bay.index


def _mean(values: list[float]) -> float | None:
    return sum(values) / len(values) if values else None


def _settling(bm: Mapping[str, Any], ts: float, window: float) -> bool:
    """The bay is inside the settling window its last deliberate widening opened."""
    mark = bm["disturb"]
    return mark is not None and 0.0 <= float(ts) - float(mark) < window


def _mark_disturbed(bm: dict[str, Any], ts: float, budget: float, why: str) -> None:
    """Open (or extend) a settling window on a deliberate widening (a fast-swap jump, an
    occupancy change into or out of ``empty``), while the bay has exemption left.

    ``why`` is the reason the diagnostics report, so a reader of ``/api/state`` can tell a
    swap the filter followed from a bay that has just changed occupancy (item 100).

    The bound is wall-clock, not sigma recovery: :func:`_account_settle` charges every tick
    the exemption actually suppressed a sigma check against ``estimator.bay_settle_max_s``,
    and only a run of that length with neither a window nor a sigma over ``sigma_fault_c``
    earns the budget back. A swap spends one window or two; a sensor that keeps jumping --
    at any cadence, since its sigma falls back within a tick or two of each jump -- spends
    the budget and then faults its zone every tick it is over, as it did before item 69.
    """
    if bm["spent"] < budget:
        bm["disturb"] = float(ts)
        bm["why"] = why


def _crossed(nu: float, var: float, jump_min_c: float, jump_var: float) -> bool:
    """The fast-swap threshold: an innovation ``nu`` of predictive variance ``var`` that is
    both bigger than ``jump_min_c`` in degrees and further than ``jump_sigmas`` standard
    deviations out.

    **One function for both halves of the rule** (plan section 8 item 124). The per-sensor
    half asks it of one member's own innovation and that member's own variance; the
    bay-level half asks it of the bay's mean innovation and the variance of that mean. On
    a bay with one proximal member the mean of one number is that number and the variance
    of that mean is its own variance, so the two calls carry identical arguments and the
    two halves *cannot* disagree -- which is the property item 124 exists to keep and the
    one a test can pin without judgement (tests/test_estimator.py). Changing the threshold
    changes it in both places; changing one caller's arithmetic away from the other's is
    then visible as two calls with different arguments on a bay that has only one."""
    return abs(nu) > jump_min_c and nu * nu > jump_var * var


#: What :func:`_swap_event` made of this tick's swap verdict.
SWAP_EVENT = "event"  #: a new event: the thermal block is reset
SWAP_REFRACTORY = "refractory"  #: the event just before it, still visible
SWAP_HELD = "held"  #: the budget is gone: the fit is kept, and the bay is scored for it


def _swap_event(bm: dict[str, Any], ts: float, spec: Any, why: str, *, limited: bool) -> str:
    """What this tick's swap verdict is -- a new **event**, the last one still standing,
    or one the budget refused -- and the record it leaves (items 123, 124).

    The *statistical* half of the rule -- the bay-level step, ``why`` :data:`SETTLE_JUMP`
    -- is rate-limited, and on both halves of what a rate limit is it now agrees with the
    per-sensor fast-swap rule that reads the same innovations at the same thresholds:

    * *cadence* -- one event per ``bay_settle_s``, the very window one deliberate widening
      opens. A swap moves the readings for as long as it takes the filter to follow them,
      so the verdict stands for several ticks; that is one event, and one model reset;
    * *budget* -- the same ``bay_settle_max_s`` the trust exemption spends. When it is
      gone the event is refused, and ``swap_held`` counts the refusal **at the same
      cadence an accepted event would have had**: one per ``bay_settle_s``, not one per
      tick the verdict happens to stand. A held event is an event that did not happen, so
      it is counted the way events are -- otherwise the number would be a tick count in
      disguise, its rate would depend on ``dt``, and one stuck sensor would read as
      hundreds of refusals an hour. (A verdict inside the refractory of an *accepted*
      event is that event still visible, not a second one held back, and is counted
      nowhere.) The thermal fit is then kept (possibly stale) rather than thrown away
      again, and the caller takes the bay's model-gate ``uncertain`` exemption away for as
      long as the refused verdict stands (:func:`_mark_model_reasons`), so the stale fit
      is *scored* by the DAS MPC's validity gate instead of excused by it, which is the
      loud half. ``bay_settle_max_s: 0`` grants neither exemption nor reset, exactly as it
      grants no settling window.

    An **occupancy crossing** (``limited=False``) is always an event. It is not a
    statistic but a declared or debounced fact -- ``empty_confirm_s`` of evidence, three
    consecutive ticks the other way, or the owner's own ``occupied`` flag -- and that
    debounce (item 19) is its rate limit. Refusing one would leave an arriving drive
    modelled with the coefficients of the drive that just left, which is the whole of
    item 12: two crossings in opposite directions inside one settling window are two
    events, not one.

    A clock that steps back is one event's worth of refractory at most: a mark in the
    future is treated as expired, like every other window here.
    """
    if limited:
        last = bm["swap_ts"]
        if last is not None and 0.0 <= float(ts) - float(last) < spec.bay_settle_s:
            return SWAP_REFRACTORY
        if bm["spent"] >= spec.bay_settle_max_s:
            held = bm["hold_ts"]
            if held is None or not 0.0 <= float(ts) - float(held) < spec.bay_settle_s:
                bm["swap_held"] += 1
                bm["hold_ts"] = float(ts)
            return SWAP_HELD
    bm["swaps"] += 1
    bm["swap_ts"], bm["swap_why"] = float(ts), why
    return SWAP_EVENT


def _account_settle(
    bm: dict[str, Any], *, settling: bool, checked: bool, over: bool, h: float, budget: float
) -> None:
    """Charge this tick's settling exemption and let a clean run earn the budget back.

    ``checked`` says the exemption really stood in for a sigma check this tick (the bay is
    observed, not ``empty``, and its zone has an estimate): only those ticks are charged,
    so the empty half of a hot swap -- which the trust rule skips anyway -- costs nothing.
    A settling tick that is not charged, and a bay whose sigma is over its threshold, still
    break the clean run, so the budget cannot recover while either goes on.
    """
    if settling:
        if checked:
            bm["spent"] = min(bm["spent"] + h, budget)
        bm["clean"] = 0.0
        return
    if over:
        bm["clean"] = 0.0
        return
    bm["clean"] += h
    if bm["clean"] >= budget:
        bm["spent"] = 0.0
        bm["clean"] = budget


def _mark_model_reasons(
    bm: dict[str, Any],
    ts: float,
    spec: Any,
    sigma: float | None,
    sigma_cal: float | None,
    *,
    scored: bool = False,
) -> None:
    """The two reasons beyond the deliberate widenings that keep a bay out of the DAS MPC's
    model checks (item 100), marked here because the estimator is the one that knows them:

    * ``uncertain`` -- the drive variance ``sigma ** 2 - sigma_cal ** 2`` is over
      ``bay_uncertain_var_c2``. A *level*: it covers the ticks after a fast-swap jump while
      the variance decays, and equally a bay nobody is watching, whose prediction error is
      not evidence about a model. The trust rule must **not** excuse that second case,
      which is why this reason is the model gate's alone;
    * ``calibration`` -- the ``sigma_cal`` floor moved by more than ``bay_cal_step_c``, so
      an accepted or expired map has just re-mapped the drive estimate.

    A mark outside its window is forgotten rather than kept, so a clock that runs backwards
    cannot bring an old one back. ``sigma`` is ``None`` for a bay with no estimate this
    tick: nothing is marked and the last floor is forgotten, so the bay's next estimate
    starts the comparison afresh instead of stepping against a stale one.

    ``scored`` is the one case where ``uncertain`` is **withheld** (item 124): the bay's
    swap verdict stands and the settling budget refused to act on it, so the thermal fit
    it is about to be judged against is the one the estimator wanted thrown away. Item
    124's fallback is to keep that fit rather than reset it for ever (item 109's
    pathology), and the whole of what makes that fallback safe is that the stale fit is
    then *scored* instead of excused -- which cannot happen while the widening the verdict
    itself applied keeps refreshing the exemption that excuses it. So the mark is neither
    refreshed nor kept, and the DAS MPC's validity gate reads that bay's prediction error
    like any other, lands in its PI-like fallback if the fit really has gone stale, and
    says so. The trust rule's own ``settling`` exemption is unaffected: it ran out of
    budget by itself, which is what put the bay here.
    """
    window = spec.bay_settle_s
    for key in ("unc", "calstep"):
        mark = bm[key]
        if mark is not None and not 0.0 <= ts - float(mark) < window:
            bm[key] = None
    if scored:
        bm["unc"] = None
    if sigma is None or sigma_cal is None:
        bm["calseen"] = None
        return
    if not scored and sigma * sigma - sigma_cal * sigma_cal > spec.bay_uncertain_var_c2:
        bm["unc"] = ts
    last = bm["calseen"]
    if last is not None and abs(sigma_cal - float(last)) > spec.bay_cal_step_c:
        bm["calstep"] = ts
    bm["calseen"] = sigma_cal


def _settle_view(bm: Mapping[str, Any], ts: float, spec: Any, settling: bool) -> dict[str, Any]:
    """What ``/api/state`` says about a bay's exemptions: which one, why and for how long.

    Two consumers, one owner (item 100). ``settling`` is the ``sigma`` trust rule's
    exemption -- a deliberate widening, ``jump`` or ``occupancy``, bounded by
    ``bay_settle_max_s`` and granted only while the bay is ``observed``
    (:mod:`aqua_bridge.control.zones` applies that part). ``model_exempt`` is the DAS MPC
    validity gate's -- every ``occupancy`` change plus the two reasons above, with no
    budget and no observation, because a bay nobody can bound is not evidence about a
    model either way.
    """
    window = spec.bay_settle_s
    live: dict[str, float] = {}
    since = bm["since"]
    if since is not None and 0.0 <= ts - float(since) < window:
        live[SETTLE_OCCUPANCY] = float(since)
    for key, reason in (("unc", SETTLE_UNCERTAIN), ("calstep", SETTLE_CALIBRATION)):
        if bm[key] is not None:
            live[reason] = float(bm[key])
    model_reason = max(live, key=lambda r: (live[r], -MODEL_REASONS.index(r))) if live else None
    return {
        "settling": settling,
        "settling_reason": bm["why"] if settling else None,
        "settling_until_s": (max(0.0, window - (ts - float(bm["disturb"]))) if settling else None),
        "settling_spent_s": float(bm["spent"]),
        "settling_budget_s": float(spec.bay_settle_max_s),
        "model_exempt": model_reason is not None,
        "model_exempt_reason": model_reason,
        "model_exempt_until_s": (
            None if model_reason is None else max(0.0, window - (ts - live[model_reason]))
        ),
    }


def _offsets_view(
    zone: _Zone,
    bay: _Bay,
    arrays: tuple[np.ndarray, np.ndarray] | None,
    per_sensor: bool,
) -> dict[str, float]:
    """The disagreement the filter carries for every proximal member beyond the anchor,
    degC: the offset state in the fused layout, and the distance between the member's own
    node and the anchor's in the per-sensor one (item 101)."""
    if arrays is None:
        return {}
    x = arrays[0]
    n = len(zone.bays)
    if not per_sensor:
        i_off = 2 + n + zone.n_nodes + n
        return {name: float(x[i_off + k]) for name, k in bay.offsets.items()}
    anchor = float(x[2 + n + bay.node])
    return {
        name: float(x[2 + n + k]) - anchor for name, k in bay.nodes.items() if name != bay.primary
    }


def _proximal_map_view(
    bay: _Bay,
    dmap: Mapping[str, Mapping[str, Any]],
    member_map: Any,
    bay_map: tuple[float, float],
) -> dict[str, dict[str, Any]]:
    """What ``/api/state`` says about the map each further proximal member was learned to
    sit on (item 101).

    ``slope_delta`` / ``offset_delta_c`` are the learned difference from the bay's own map;
    ``slope`` / ``offset_c`` are the map the filter actually predicts that member with,
    which is the difference added to the bay's map and clipped to :data:`CAL_SLOPE_BOUNDS`
    and :data:`CAL_OFFSET_BOUNDS`. ``clipped`` says whether that clip bit -- without it a
    reader works out why a bay under-reads from numbers the filter is not using.
    """
    s_bay, b_bay = bay_map
    out: dict[str, dict[str, Any]] = {}
    for name, entry in dmap.items():
        raw_s, raw_b = s_bay + entry["th"][0], b_bay + entry["th"][1]
        s_eff, b_eff = member_map(bay, name, s_bay, b_bay)
        out[name] = {
            "slope_delta": entry["th"][0],
            "offset_delta_c": entry["th"][1],
            "slope": s_eff,
            "offset_c": b_eff,
            "clipped": not (
                CAL_SLOPE_BOUNDS[0] <= raw_s <= CAL_SLOPE_BOUNDS[1]
                and CAL_OFFSET_BOUNDS[0] <= raw_b <= CAL_OFFSET_BOUNDS[1]
            ),
            "samples": entry["n"],
        }
    return out


def _bay_rise(zone: _Zone, bay: _Bay, arrays: tuple[np.ndarray, np.ndarray] | None) -> float:
    """This tick's drive-to-air rise of the bay, degC, or 0 with no filter state: the
    scale on which two placements on one bay disagree (item 125)."""
    if arrays is None:
        return 0.0
    x = arrays[0]
    return float(x[2 + bay.index] - x[0])


#: What each proximal layout can identify about a redundant pair's placement difference
#: (plan section 8 item 125). Two sensors on one bay disagree by ``ds * rise + db``.
LAYOUT_FUSED = "fused"
LAYOUT_PER_SENSOR = "per_sensor"
#: Which halves of ``ds * rise + db`` the layout separates, published beside the verdict.
_LAYOUT_LEARNS: dict[str, str] = {
    LAYOUT_FUSED: "offset",
    LAYOUT_PER_SENSOR: "offset+slope",
}


def _placement_view(
    bay: _Bay,
    bm: Mapping[str, Any],
    offsets: Mapping[str, float],
    spec: Any,
    per_sensor: bool,
    rise: float,
) -> dict[str, dict[str, Any]]:
    """What ``/api/state`` says about whether a redundant pair still sits where it did
    (plan section 8 item 125), per further proximal member of the bay.

    A pair of sensors on one bay disagrees by ``ds * (T_d - T_a) + db`` -- a slope
    difference in the fraction of the drive each sees, and a constant offset. Both layouts
    learn *a* number for that disagreement; only one of them learns its **shape**, and
    this view says which, so nothing downstream reads the two as the same evidence:

    * ``per_sensor`` (``estimator.proximal_slope_spread > 0``, item 101) carries ``(ds,
      db)`` as a two-parameter fit, so the load-dependent half and the constant half are
      separated. A gap outside the prior box is then a placement the config does not
      allow: a sensor coming loose, fouling or ageing. ``learns: offset+slope``;
    * ``fused`` (item 67, the default) carries one random-walk offset state per member.
      That state is the *whole* disagreement at the current rise, so it absorbs ``ds *
      rise`` as drift and the layout cannot tell a moved sensor from a load it has no
      slope for. ``learns: offset`` -- and the honest consequence is that its ``over`` is
      a diagnostic, not a fault: the fused layout **cannot** learn the shape, and making
      it do so is the per-sensor layout, which is the same state size and a cheaper
      measurement. Nothing here pretends otherwise.

    ``gap_c`` is the learned disagreement at this tick's rise, ``bound_c`` the prior box
    at the same rise (``proximal_gap_sigmas`` standard deviations of ``ds * rise + db``
    under independent priors, so it widens with the load as the disagreement does), and
    ``over`` whether the gap has walked out of it. ``evidence`` is how many rows the
    per-sensor map has taken; on the fused layout it is ``None``, since an offset state
    has no row count of its own."""
    out: dict[str, dict[str, Any]] = {}
    layout = LAYOUT_PER_SENSOR if per_sensor else LAYOUT_FUSED
    spread = spec.proximal_slope_spread if per_sensor else 0.0
    bound = spec.proximal_gap_sigmas * math.sqrt((spread * rise) ** 2 + spec.proximal_offset_c**2)
    for name in bay.sensors:
        if name == bay.primary or name not in offsets:
            continue
        gap = float(offsets[name])
        entry = bm["dmap"].get(name) if per_sensor else None
        out[name] = {
            "layout": layout,
            "learns": _LAYOUT_LEARNS[layout],
            "gap_c": gap,
            "bound_c": bound,
            "over": abs(gap) > bound,
            "evidence": None if entry is None else int(entry["n"]),
        }
    return out


def _seed_sensor(bay: _Bay, temps: Mapping[str, float]) -> tuple[str, float]:
    """The member a bay's sensor node starts from and its reading: the anchor if that one
    is trusted, else the hottest trusted member (conservative). The offsets of the others
    are seeded at 0, so a pair at different placements starts consistent instead of handing
    the anchor a step the fast-swap rule would read as a swap; in the per-sensor layout
    (item 101) the other nodes start at their own reading, or at their own map when they
    are not reporting."""
    if bay.primary in temps:
        return bay.primary, float(temps[bay.primary])
    present = [name for name in bay.sensors if name in temps]
    hottest = max(present, key=lambda name: (float(temps[name]), name))
    return hottest, float(temps[hottest])


def _forget_pair(mem: dict[str, Any], bay: str, serial: str | None) -> None:
    """End a correlation association and forget the evidence that made it.

    The serial's SMART history starts over, so the pair has to win the acceptance
    rule again over a full window before it may calibrate that bay -- whether it
    ended on a hot swap, a jump, silence or a failed re-check (item 18).
    """
    bm = mem["bays"][bay]
    bm["assoc"], bm["rej"], bm["ver"] = None, 0, False
    mem["pending"].pop(bay, None)
    known = mem["smart"].get(serial) if serial is not None else None
    if known is not None:
        known["hist"] = []


def update(
    memory: object,
    cfg: MpcConfig,
    *,
    temps: Mapping[str, float],
    u: Mapping[str, float],
    ts: float,
    smart: object = None,
    calibration: object = None,
    curves: Mapping[str, Any] | None = None,
) -> EstimatorUpdate:
    """One estimator tick (module docstring).

    ``temps`` holds the gate-trusted values of this tick only (a missing name is
    untrusted), ``u`` the command on the fans since the previous tick (``prev``),
    ``ts`` the observation time, ``smart`` the raw ``PlantObservation.inputs
    ["smart"]`` (``None`` or malformed: no SMART) and ``calibration`` the raw
    ``PlantObservation.inputs["calibration"]`` -- manual drive readings per bay
    (``None`` or malformed: none, item 23). ``curves`` is
    ``solver_memory["fan_curves"]`` (the online fan-curve fit,
    :mod:`aqua_bridge.control.fancurve`): with it the airflow ``Q_z`` / ``Qn_z``
    follows the fitted ``deadband`` / ``exponent`` per fan model, the way the thermal
    model does, and ``None`` or an unusable entry keeps the configured curve
    (:func:`_airflow`). Raises only on a numerical failure of the filter (non-finite
    state), which ``step`` turns into an estimator fault.
    """
    topo = cfg.topology
    spec = cfg.estimator
    if topo is None or spec is None:
        raise ValueError("the estimator needs a zoned config (mpc.topology)")
    st = _structure(cfg)
    mem = _load(memory, st)
    last_ts = mem["ts"]
    elapsed = 0.0 if last_ts is None else float(ts) - last_ts
    h_pred = min(max(elapsed, 0.0), MAX_PREDICT_S)
    h_occ = min(max(elapsed, 0.0), 3.0 * cfg.dt)
    max_age_cal = spec.calibration_max_age_days * _DAY_S
    fresh_smart = _parse_smart(smart, spec.smart_max_age_s)
    fresh_manual = _parse_manual(calibration, topo.bays, float(ts), spec.smart_max_age_s)

    # -- SMART samples: which are new ----------------------------------------
    new_samples: dict[str, tuple[float, float]] = {}
    for serial, (temp, age, model) in fresh_smart.items():
        sample_ts = float(ts) - age
        known = mem["smart"].get(serial)
        if known is None or sample_ts > known["ts"] + NEW_SAMPLE_EPS_S:
            new_samples[serial] = (sample_ts, temp)
        entry = dict(known) if known is not None else {"hist": []}
        if serial in new_samples:
            entry.update(ts=sample_ts, t=temp)
        entry["model"] = model if model is not None else entry.get("model")
        mem["smart"][serial] = entry

    # -- manual calibrations: which are new (item 23) --------------------------
    new_manual: dict[str, tuple[float, float]] = {}
    for b, (sample_ts, temp) in fresh_manual.items():
        known = mem["manual"].get(b)
        if known is None or sample_ts > known["ts"] + NEW_SAMPLE_EPS_S:
            new_manual[b] = (sample_ts, temp)
            mem["manual"][b] = {"ts": sample_ts, "cal": None if known is None else known["cal"]}

    # -- association before the filter: declared wins, silent serials drop --------
    declared = {b: bay.serial for b, bay in topo.bays.items() if bay.serial}
    declared_serials = set(declared.values())
    assoc: dict[str, tuple[str, str]] = {}
    for b in topo.bays:
        bm = mem["bays"][b]
        if b in declared:
            assoc[b] = (declared[b], "declared")
            # The declaration replaces any correlation pair, verification included: an
            # undeclared bay must earn ``ver`` again from its next pair (item 18).
            bm["assoc"], bm["rej"], bm["ver"] = None, 0, False
            continue
        serial = bm["assoc"]
        if serial is not None and (serial not in fresh_smart or serial in declared_serials):
            _forget_pair(mem, b, serial)
            serial = None
        if serial is not None:
            assoc[b] = (serial, "correlation")

    # -- calibration expiry (SMART and manual alike) ----------------------------
    manual_entries = [known["cal"] for known in mem["manual"].values() if known["cal"]]
    for entry in [e for entries in mem["cal"].values() for e in entries.values()] + manual_entries:
        if entry["ts"] is not None and entry["fresh"] > 0:
            age = float(ts) - entry["ts"]
            if age > max_age_cal or age < 0:
                entry["fresh"] = 0

    classes: dict[str, tuple[str, str]] = {}
    for b in topo.bays:
        model = None
        if b in assoc:
            known = mem["smart"].get(assoc[b][0])
            model = None if known is None else known.get("model")
        classes[b] = _drive_class(cfg, b, model, may_relax=b in assoc and assoc[b][1] == "declared")

    def cal_in_force(b: str) -> tuple[dict[str, Any] | None, str | None]:
        """The calibration bay ``b`` runs on, and where it came from.

        The bay's associated serial wins, but only once its entry is one the filter
        would actually use (accepted once, or accepted now): a SMART entry in its
        first samples must not displace a manual calibration already in force, or
        the bay would drop back to the untrusted prior for the twenty samples the
        new entry needs (item 23). One helper for all of it, so the map the filter
        uses and the ``calibration_source`` / ``calibrated`` / ``sigma_cal_c`` the
        views report can never disagree.
        """
        known = mem["manual"].get(b)
        manual = None if known is None else known["cal"]
        if b in assoc:
            entry = mem["cal"].get(b, {}).get(assoc[b][0])
            if entry is not None and (
                manual is None or entry["used"] or _calibrated(entry, float(ts), max_age_cal)
            ):
                return entry, "smart"
        return (manual, "manual") if manual is not None else (None, None)

    def verified(b: str) -> bool:
        """Whether the bay's association may calibrate it: declared, or re-checked once."""
        return b in assoc and (assoc[b][1] == "declared" or bool(mem["bays"][b]["ver"]))

    def sensor_map(b: str) -> tuple[float, float, str | None]:
        """``(s, b, source)`` the filter uses for bay ``b``.

        ``source`` identifies the map, so a switch (a calibration accepted or lost, a
        different drive) re-maps the drive state: the serial for a SMART calibration
        (re-verified here, item 18 -- an unverified correlation guess is never used,
        however calibrated its entry), :data:`MANUAL_MAP` for a manual one (item 23,
        no association to re-check), ``None`` for the prior.
        """
        entry, kind = cal_in_force(b)
        if entry is None or not entry["used"]:
            return CAL_PRIOR[0], CAL_PRIOR[1], None
        if kind == "smart":
            if not verified(b):
                return CAL_PRIOR[0], CAL_PRIOR[1], None
            return entry["th"][0], entry["th"][1], assoc[b][0]
        return entry["th"][0], entry["th"][1], MANUAL_MAP

    per_sensor = spec.proximal_slope_spread > 0

    def dmap_entry(bay: _Bay, name: str) -> dict[str, Any]:
        """The learned map difference of one proximal member beyond the anchor, created at
        its prior on first use (item 101)."""
        entries = mem["bays"][bay.name]["dmap"]
        entry = entries.get(name)
        if entry is None:
            entry = entries[name] = _fresh_dmap(spec)
        return entry

    def dmap_box(s_bay: float, b_bay: float) -> tuple[tuple[float, float], tuple[float, float]]:
        """The box a member's learned ``(ds, db)`` is held in, given the bay's own map.

        The slope difference is bounded by ``estimator.proximal_slope_spread`` -- the
        operator's statement of how far apart two placements of one bay may sit -- and by
        what keeps the member's map inside :data:`CAL_SLOPE_BOUNDS`; the offset difference
        by what keeps it inside :data:`CAL_OFFSET_BOUNDS`. So the clip in
        :func:`member_map` is a backstop for a stored entry or a bay map that moved under
        it, not the normal end state (item 101).
        """
        spread = spec.proximal_slope_spread
        return (
            (
                max(-spread, CAL_SLOPE_BOUNDS[0] - s_bay),
                min(spread, CAL_SLOPE_BOUNDS[1] - s_bay),
            ),
            (CAL_OFFSET_BOUNDS[0] - b_bay, CAL_OFFSET_BOUNDS[1] - b_bay),
        )

    def member_map(bay: _Bay, name: str, s_bay: float, b_bay: float) -> tuple[float, float]:
        """``(s, b)`` of one proximal member: the bay's map for the anchor, and for every
        further member the bay's map plus that member's learned difference, clipped to the
        same bounds an accepted calibration is (item 101). In the fused layout every member
        reads the bay's own map and the difference is an additive state instead."""
        if not per_sensor or name == bay.primary:
            return s_bay, b_bay
        th = dmap_entry(bay, name)["th"]
        return (
            min(max(s_bay + th[0], CAL_SLOPE_BOUNDS[0]), CAL_SLOPE_BOUNDS[1]),
            min(max(b_bay + th[1], CAL_OFFSET_BOUNDS[0]), CAL_OFFSET_BOUNDS[1]),
        )

    def seed_nodes(
        x: np.ndarray,
        i_s: int,
        bay: _Bay,
        *,
        t_ref: float,
        air: float,
        maps: tuple[float, float, float] | None,
    ) -> None:
        """Seed a bay's sensor node(s) (item 69, per member since item 101).

        The fused layout has one node and it takes ``t_ref``. With a node per member every
        reporting member starts at its own reading and every silent one at the map it is
        believed to sit on: ``maps`` is ``(T_d, s, b)`` of the bay, or ``None`` for an
        empty bay, whose nodes follow the air.
        """
        if not per_sensor:
            x[i_s + bay.node] = t_ref
            return
        for name, k in bay.nodes.items():
            if name in temps:
                x[i_s + k] = float(temps[name])
            elif maps is None:
                x[i_s + k] = air
            else:
                t_d, s_map, b_map = maps
                s_i, b_i = member_map(bay, name, s_map, b_map)
                x[i_s + k] = s_i * t_d + (1.0 - s_i) * air + b_i

    prev_air = {z: float(zm["x"][0]) for z, zm in mem["zones"].items()}
    trusted_air = {z: [temps[t] for t in zone.air if t in temps] for z, zone in st.zones.items()}
    smart_arrived: dict[str, bool] = {}
    jumped: dict[str, bool] = {}
    # Bays whose *mean innovation* over their trusted proximal members stepped away from
    # zero (items 12, 109) -- each member against its own prediction, its node and, on the
    # fused layout, its placement offset: the drive in them may be a different one now.
    stepped: dict[str, bool] = {}
    arrays: dict[str, tuple[np.ndarray, np.ndarray]] = {}
    # zone -> (Q, Qn, t_in, the airflow followed a fitted fan curve)
    geometry: dict[str, tuple[float, float, float, str]] = {}

    q_scale = h_pred / cfg.dt if cfg.dt > 0 else 0.0
    offset_var = spec.proximal_offset_c**2
    jump_var = spec.jump_sigmas**2
    for z, zone in st.zones.items():
        n = len(zone.bays)
        i_d, i_s = 2, 2 + n
        i_q = i_s + zone.n_nodes
        i_off = i_q + n
        dim = zone.dim
        inlet_vals = [temps[t] for t in zone.inlet if t in temps]
        zm = mem["zones"].get(z)
        q_flow, qn, fan_curve = _airflow(cfg, zone, u, curves)

        if zm is None:
            air0 = _mean(trusted_air[z])
            if air0 is None:
                continue
            t_in = _mean(inlet_vals)
            t_in = air0 if t_in is None else t_in
            x = np.zeros(dim)
            x[0] = air0
            p_diag = np.zeros(dim)
            p_diag[0], p_diag[1] = spec.p0_t_air, spec.p0_d_air
            p_diag[i_off:] = offset_var
            p_diag[i_s:i_q] = spec.p0_t_sensor
            f_air = 0.0
            for j, b in enumerate(zone.bays):
                bay = st.bays[b]
                present = [t for t in bay.sensors if t in temps]
                ref, t_ref = _seed_sensor(bay, temps) if present else (bay.primary, air0)
                mem["bays"][b]["init"] = bool(present)
                p_diag[i_d + j], p_diag[i_q + j] = spec.p0_t_drive, spec.p0_heat
                occ = _declared_state(topo.bays[b].occupied, mem["bays"][b]["occ"])
                s_map, b_map, source = sensor_map(b)
                if occ == EMPTY:
                    x[i_d + j] = air0
                    seed_nodes(x, i_s, bay, t_ref=t_ref, air=air0, maps=None)
                    continue
                mem["bays"][b]["map"] = source
                s_ref, b_ref = member_map(bay, ref, s_map, b_map)
                t_d = air0 + (t_ref - air0 - b_ref) / s_ref
                seed_nodes(x, i_s, bay, t_ref=t_ref, air=air0, maps=(t_d, s_map, b_map))
                g = G0_W_PER_K + K_W_PER_K * qn
                cd = drive_capacity(cfg, classes[b][0])
                x[i_d + j] = t_d
                x[i_q + j] = g * (t_d - air0) / cd
                f_air += g * (t_d - air0)
            f_air -= (q_flow + LEAK_W_PER_K) * (air0 - t_in)
            for other in zone.coupled:
                if other in prev_air:
                    f_air += KAPPA_W_PER_K * (prev_air[other] - air0)
            x[1] = -f_air / C_AIR_J_PER_K
            p = np.diag(p_diag)
            arrays[z] = (x, p)
            geometry[z] = (q_flow, qn, t_in, fan_curve)
            mem["zones"][z] = {"x": x, "P": p, "t_in": t_in, "blind": 0.0}
            continue

        x = zm["x"]
        p = zm["P"]
        t_in = _mean(inlet_vals)
        if t_in is None:
            t_in = zm["t_in"] if zm["t_in"] is not None else float(x[0])

        # re-map the drive state when the sensor map switches (calibration accepted / lost)
        for j, b in enumerate(zone.bays):
            bm = mem["bays"][b]
            s_map, b_map, source = sensor_map(b)
            if source != bm["map"]:
                if bm["occ"] != EMPTY:
                    node = i_s + st.bays[b].node
                    x[i_d + j] = x[0] + (x[node] - x[0] - b_map) / s_map
                bm["map"] = source

        if h_pred > 0:
            a = np.zeros((dim, dim))
            c = np.zeros(dim)
            a[0, 0] -= (q_flow + LEAK_W_PER_K) / C_AIR_J_PER_K
            c[0] += (q_flow + LEAK_W_PER_K) * t_in / C_AIR_J_PER_K
            for other in zone.coupled:
                if other in prev_air:
                    a[0, 0] -= KAPPA_W_PER_K / C_AIR_J_PER_K
                    c[0] += KAPPA_W_PER_K * prev_air[other] / C_AIR_J_PER_K
            a[0, 1] = 1.0
            qd = np.zeros(dim)
            qd[0], qd[1] = spec.q_t_air, spec.q_d_air
            qd[i_off:] = spec.q_offset  # the placement offsets are random walks
            qd[i_s:i_q] = spec.q_t_sensor
            g = G0_W_PER_K + K_W_PER_K * qn
            for j, b in enumerate(zone.bays):
                bay = st.bays[b]
                empty = mem["bays"][b]["occ"] == EMPTY
                s_map, b_map, _ = sensor_map(b)
                if not empty:
                    cd = drive_capacity(cfg, classes[b][0])
                    a[0, 0] -= g / C_AIR_J_PER_K
                    a[0, i_d + j] += g / C_AIR_J_PER_K
                    a[i_d + j, i_d + j] -= g / cd
                    a[i_d + j, 0] += g / cd
                    a[i_d + j, i_q + j] = 1.0
                    qd[i_d + j], qd[i_q + j] = spec.q_t_drive, spec.q_heat
                # every sensor node of the bay: its own lag, and its own map when the
                # layout carries one node per member (item 101)
                for k in sorted(set(bay.nodes.values())) if per_sensor else (bay.node,):
                    row = i_s + k
                    tau = zone.node_tau[k]
                    if empty:  # no drive: the node follows the air
                        a[row, 0] += 1.0 / tau
                        a[row, row] -= 1.0 / tau
                        continue
                    name = zone.node_sensor[k]
                    if name is None:
                        s_i, b_i = s_map, b_map
                    else:
                        s_i, b_i = member_map(bay, name, s_map, b_map)
                    a[row, i_d + j] += s_i / tau
                    a[row, 0] += (1.0 - s_i) / tau
                    a[row, row] -= 1.0 / tau
                    c[row] += b_i / tau
            phi, gamma = _discretise(a, c, h_pred)
            x = phi @ x + gamma
            p = phi @ p @ phi.T + np.diag(qd * q_scale)
            p = 0.5 * (p + p.T)

        for name in zone.air:
            if name in temps:
                _scalar_update(x, p, 0, float(temps[name]), _sensor_var(cfg, name))
        for j, b in enumerate(zone.bays):
            bay = st.bays[b]
            bm = mem["bays"][b]
            occ = bm["occ"]
            s_map, b_map, _ = sensor_map(b)
            present = [name for name in bay.sensors if name in temps]
            if present and not bm["init"]:
                # No trusted member when the zone started: seed the bay from this first
                # reading instead of leaving the fast-swap rule to see the gap (item 69).
                ref, t_s = _seed_sensor(bay, temps)
                air_now = float(x[0])
                for k_off in bay.offsets.values():
                    _reset_state(x, p, i_off + k_off, 0.0, offset_var)
                if occ == EMPTY:
                    seed_nodes(x, i_s, bay, t_ref=t_s, air=air_now, maps=None)
                    _reset_state(x, p, i_d + j, air_now, spec.p0_t_drive)
                else:
                    s_seed, b_seed = member_map(bay, ref, s_map, b_map)
                    t_d = air_now + (t_s - air_now - b_seed) / s_seed
                    seed_nodes(x, i_s, bay, t_ref=t_s, air=air_now, maps=(t_d, s_map, b_map))
                    g_seed = G0_W_PER_K + K_W_PER_K * qn
                    cd_seed = drive_capacity(cfg, classes[b][0])
                    _reset_state(x, p, i_d + j, t_d, spec.p0_t_drive)
                    _reset_state(x, p, i_q + j, g_seed * (t_d - air_now) / cd_seed, spec.p0_heat)
                for k in sorted(set(bay.nodes.values())):
                    _reset_state(x, p, i_s + k, float(x[i_s + k]), spec.p0_t_sensor)
                bm["init"] = True
            # The fast-swap test runs on the nodes before any member of the bay has
            # updated one, and the members have to agree (module docstring): one bay,
            # one drive, so a swap moves every proximal sensor of the bay the same way.
            rise = float(x[i_d + j] - x[0]) if per_sensor else 0.0
            step_nu = 0.0  # the bay-level statistic, when it crosses (item 124)
            members: list[tuple[str, float, float, float, bool, int | None, int]] = []
            for name in present:
                value = float(temps[name])
                r = _sensor_var(cfg, name)
                if occ == EMPTY:
                    r /= EMPTY_WEIGHT
                idx = i_s + bay.nodes[name]
                k_off = bay.offsets.get(name)
                if per_sensor and name != bay.primary:
                    # the member reads its own node; what is uncertain is *where it sits*,
                    # which is its map's own predictive variance at this tick's rise
                    r = r + _dmap_row_var(dmap_entry(bay, name), rise)
                if k_off is None:
                    nu = value - x[idx]
                    s_innov = p[idx, idx] + r
                else:
                    off = i_off + k_off
                    nu = value - x[idx] - x[off]
                    s_innov = p[idx, idx] + 2.0 * p[idx, off] + p[off, off] + r
                big = _crossed(nu, s_innov, spec.jump_min_c, jump_var)
                members.append((name, value, r, nu, big, k_off, idx))
            if members and occ != EMPTY:
                # The bay-level test (item 12), on the very innovations above and before
                # any of this tick's updates: the *mean innovation* of the bay's trusted
                # members against the variance of that mean. Per-sensor jumps are
                # sequential, so with two sensors one of them jumps whenever the two
                # disagree, which is placement, not a swap; a swap moves every member of
                # the bay together, so it survives the mean while a disagreement cancels.
                # Each member is predicted where the filter says *it* sits -- its own node
                # and, on the fused layout, its own placement offset -- and the variance
                # of the mean is that same prediction's, so a placement the filter is
                # still unsure of cannot look like a swap either (item 109). ``rows`` are
                # the states each member's prediction is built from, so the double sum is
                # the covariance of the mean prediction.
                rows = [(m[6],) if m[5] is None else (m[6], i_off + m[5]) for m in members]
                nu_bay = sum(m[3] for m in members) / len(members)
                var_bay = sum(m[2] for m in members)
                for row_a in rows:
                    for row_c in rows:
                        var_bay += sum(float(p[a, c]) for a in row_a for c in row_c)
                s_bay = var_bay / len(members) ** 2
                if _crossed(nu_bay, s_bay, spec.jump_min_c, jump_var):
                    stepped[b] = True
                    step_nu = nu_bay
            jump = any(m[4] for m in members) and not (
                any(m[3] > spec.jump_min_c for m in members)
                and any(m[3] < -spec.jump_min_c for m in members)
            )
            # item 124: a bay-level step is an event for the per-sensor rule too. Both
            # halves read the same per-member innovations at the same thresholds (item
            # 109), and on a one-member bay they are the same number, so they can only
            # part on a bay with a redundant pair -- where the mean can cross while no
            # single member does. The drive itself moved, so the placement row this tick
            # would carry that move instead of the two sensors' geometry.
            if (
                per_sensor
                and not (jump or stepped.get(b))
                and occ != EMPTY
                and bay.primary in temps
            ):
                # Only the two readings enter the row: both members see the same drive and
                # the same air, so their difference identifies the placement without any
                # SMART (item 101). Not on a jump tick, where the drive itself moved.
                anchor_v = float(temps[bay.primary])
                anchor_r = _sensor_var(cfg, bay.primary)
                ds_box, db_box = dmap_box(s_map, b_map)
                for name, value, _r, _nu, _big, _k, _idx in members:
                    if name == bay.primary:
                        continue
                    _dmap_absorb(
                        dmap_entry(bay, name),
                        rise,
                        value - anchor_v,
                        anchor_r + _sensor_var(cfg, name) + CAL_ROW_VAR,
                        q=spec.q_offset * q_scale,
                        ds=ds_box,
                        db=db_box,
                    )
            for name, value, r, nu, big, k_off, idx in members:
                if jump and big:
                    # something moved: follow the sensor (the placement did not move, so
                    # its offset state -- or its learned map -- keeps its own variance)
                    if occ != EMPTY:
                        s_i = member_map(bay, name, s_map, b_map)[0] if per_sensor else s_map
                        p[i_d + j, i_d + j] += (nu / s_i) ** 2
                        jumped[b] = True
                    p[idx, idx] += nu * nu
                if k_off is None:
                    _scalar_update(x, p, idx, value, r)
                else:
                    _pair_update(x, p, idx, i_off + k_off, value, r)
            if stepped.get(b) and not jumped.get(b):
                # The bay's mean crossed the thresholds while no single member did (item
                # 124). The thermal block is about to be thrown away, so the drive
                # estimate it would be scored against must not stay confident: widen it
                # by the very statistic that said so, through the bay's own map, and give
                # the bay the rest of a jump's consequences (the settling window, this
                # tick's SMART, the correlation pair) below.
                p[i_d + j, i_d + j] += (step_nu / s_map) ** 2
                jumped[b] = True
        arrays[z] = (x, p)
        geometry[z] = (q_flow, qn, t_in, fan_curve)
        mem["zones"][z]["t_in"] = t_in
        # The wall clock, not the occupancy horizon: a tick gap longer than 3 dt must not
        # under-count the time the air node has run unmeasured (it delays the fault).
        mem["zones"][z]["blind"] = 0.0 if trusted_air[z] else zm.get("blind", 0.0) + h_pred

    def _bay_slot(b: str) -> tuple[tuple[np.ndarray, np.ndarray], tuple[int, int]] | None:
        """``((x, P), (i_d, i_s))`` of bay ``b``, or ``None`` when it takes no sample."""
        bay = st.bays[b]
        if bay.zone not in arrays or mem["bays"][b]["occ"] == EMPTY:
            return None
        return arrays[bay.zone], _slots(st.zones[bay.zone], bay)[:2]

    # -- SMART: reject, calibrate, measure ------------------------------------------
    for b, (serial, _source) in assoc.items():
        if serial not in new_samples or jumped.get(b):
            continue  # a jump this tick: the sample may describe the drive that just left
        smart_arrived[b] = True
        slot = _bay_slot(b)
        if slot is None:
            continue
        per_bay = mem["cal"].setdefault(b, {})
        entry = _absorb_sample(
            mem,
            spec,
            slot[0],
            slot[1],
            per_bay.get(serial) or _fresh_calibration(spec.sigma_uncalibrated_c),
            new_samples[serial],
            float(ts),
            max_age_cal,
            gate=verified(b),
        )
        if entry is None:
            continue
        per_bay[serial] = entry
        if len(per_bay) > CAL_SERIALS_PER_BAY:
            oldest = min(
                (s for s in per_bay if s != serial), key=lambda s: (per_bay[s]["ts"] or -1e300, s)
            )
            del per_bay[oldest]

    # -- manual calibration: the same rule, keyed by bay (item 23) -------------------
    for b, sample in new_manual.items():
        if jumped.get(b):
            continue
        smart_arrived[b] = True  # a measured drive is presence evidence, like SMART
        slot = _bay_slot(b)
        if slot is None:
            continue
        known = mem["manual"][b]
        entry = _absorb_sample(
            mem,
            spec,
            slot[0],
            slot[1],
            known["cal"] or _fresh_calibration(spec.sigma_uncalibrated_c),
            sample,
            float(ts),
            max_age_cal,
            kind="manual",
            gate=True,
        )
        if entry is not None:
            known["cal"] = entry

    # -- occupancy -----------------------------------------------------------------------
    evidence: dict[str, tuple[float, float]] = {}
    observed: dict[str, bool] = {}
    # Bays whose drive may be a different one from this tick on (item 12): the thermal
    # model's coefficients for them describe a drive that is no longer there.
    swapped: dict[str, bool] = dict(stepped)
    # the half of ``swapped`` that is an occupancy crossing, so the record below names the
    # stronger of the two reasons when a tick carries both (items 123, 124)
    crossed: set[str] = set()
    for b, bay in st.bays.items():
        bm = mem["bays"][b]
        if jumped.get(b):  # the fast-swap rule widened this bay on purpose
            _mark_disturbed(bm, float(ts), spec.bay_settle_max_s, SETTLE_JUMP)
        before = bm["occ"]
        declared_occ = topo.bays[b].occupied
        zone_ready = bay.zone in arrays
        sensor_ok = any(t in temps for t in bay.sensors)
        observed[b] = sensor_ok
        air_ok = bool(trusted_air.get(bay.zone))
        after = before
        if declared_occ is True:
            after = OCCUPIED
            bm["blind"] = 0.0
        elif declared_occ is False:
            after = EMPTY
            bm["blind"] = 0.0
        elif not zone_ready:  # no filter state at all: nothing is known about the bay
            after = UNKNOWN
            bm["low"], bm["rise"], bm["blind"] = 0.0, 0, 0.0
        elif not sensor_ok and not smart_arrived.get(b):
            # Item 19: a dropout is not evidence. The bay keeps its state (and its
            # pending counts, which this tick neither confirms nor contradicts) until
            # the blindness has lasted occupancy_hold_s. ``blind`` is a *pending*
            # transition, so a bay that is already unknown counts nothing: there is no
            # transition left to hold and pending_unknown_s stays 0 however long the
            # blindness lasts.
            if before == UNKNOWN:
                bm["blind"] = 0.0
            else:
                bm["blind"] += h_occ
                if bm["blind"] >= spec.occupancy_hold_s:
                    after = UNKNOWN
                    bm["low"], bm["rise"] = 0.0, 0
        else:
            bm["blind"] = 0.0
            x, _ = arrays[bay.zone]
            i_d, i_s, i_q = _slots(st.zones[bay.zone], bay)
            s_map, b_map, _ = sensor_map(b)
            g = G0_W_PER_K + K_W_PER_K * geometry[bay.zone][1]
            d_t = float(x[i_s] - x[0])
            heat = float(x[i_q]) * drive_capacity(cfg, classes[b][0]) + g * b_map / s_map
            evidence[b] = (d_t, heat)
            if before == EMPTY:
                bm["low"] = 0.0
                if smart_arrived.get(b) or (sensor_ok and d_t > spec.occupied_dT_c):
                    bm["rise"] += 1
                    if smart_arrived.get(b) or bm["rise"] >= RISE_TICKS:
                        after = OCCUPIED
                else:
                    bm["rise"] = 0
            else:
                bm["rise"] = 0
                if smart_arrived.get(b) or d_t > spec.occupied_dT_c:
                    after = OCCUPIED
                    bm["low"] = 0.0
                elif (
                    air_ok
                    and d_t < spec.empty_dT_c
                    and heat < EMPTY_HEAT_W
                    and not (b in assoc and assoc[b][0] in fresh_smart)
                ):
                    bm["low"] += h_occ
                    if bm["low"] >= spec.empty_confirm_s:
                        after = EMPTY
                else:
                    bm["low"] = 0.0
        if after != before:
            bm["since"] = float(ts)
            if bay.zone in arrays:
                x, p = arrays[bay.zone]
                i_d, i_s, i_q = _slots(st.zones[bay.zone], bay)
                if before == EMPTY:  # a drive (possibly) arrived: start from the sensor, wide
                    x[i_d] = x[i_s]
                    p[i_d, :], p[:, i_d] = 0.0, 0.0
                    p[i_d, i_d] = spec.reset_drive_var
                if before == EMPTY or after == EMPTY:
                    x[i_q] = 0.0
                    p[i_q, :], p[:, i_q] = 0.0, 0.0
                    p[i_q, i_q] = spec.p0_heat
                if before == EMPTY or after == EMPTY:  # a deliberate widening, as a jump
                    _mark_disturbed(bm, float(ts), spec.bay_settle_max_s, SETTLE_OCCUPANCY)
            if (before == EMPTY) != (after == EMPTY):
                swapped[b] = True
                crossed.add(b)
                _forget_pair(mem, b, bm["assoc"])
                if b in assoc and assoc[b][1] == "correlation":
                    _forget_pair(mem, b, assoc[b][0])
                    del assoc[b]
                # The bay's manual calibration described the drive that was there (items
                # 12, 104). It is keyed by bay, not by serial, so unlike a SMART entry it
                # would follow the slot rather than the drive and nothing later would
                # notice. Only this half of the swap rule drops it -- an occupancy
                # crossing of ``empty``, where a drive demonstrably left or arrived --
                # and the bay returns to the prior map at the uncalibrated sigma: more
                # margin, never less cooling.
                mem["manual"].pop(b, None)
            bm["low"], bm["rise"], bm["blind"] = 0.0, 0, 0.0
            bm["occ"] = after
        if jumped.get(b) and b in assoc and assoc[b][1] == "correlation":
            _forget_pair(mem, b, assoc[b][0])
            del assoc[b]

    # The other half of the swap rule -- a bare proximal step, with no occupancy crossing
    # (items 12, 104) -- *inflates* the bay's manual calibration instead of dropping it.
    # ``stepped`` is one tick's mean innovation over the bay's trusted proximal members,
    # each against its own prediction -- its node and, on the fused layout, its placement
    # offset -- and a spin-up, an I/O burst or a fan step raises it as readily as a swap
    # does; deleting on that would throw away twenty readings the owner took by hand on
    # evidence that thin, and it would be asymmetric anyway, since the same event leaves
    # a SMART entry alone.
    # Inflation is the conservative direction: ``sigma_cal`` doubles until ``confirm``
    # further accepted hand readings of that bay clear it, indefinitely without them.
    for b, was_stepped in stepped.items():
        known = mem["manual"].get(b)
        if was_stepped and known is not None and known["cal"] is not None:
            known["cal"]["inflate"] = STALE_SIGMA_CAL_FACTOR
            known["cal"]["confirm"] = CAL_MIN_SAMPLES

    # -- the swap *event*, rate-limited, and the record it leaves (items 123, 124) -------
    # ``swapped`` above is this tick's verdict and keeps every consequence it had.
    # ``resets`` is the subset that is a new event: one per ``bay_settle_s`` while the
    # bay has ``bay_settle_max_s`` of exemption left, the same budget the trust rule
    # spends. Only these reach ``thermal.update(reset_bays=...)``.
    resets: dict[str, bool] = {}
    # the bays whose verdict the *budget* refused this tick. Their fit is kept, so the
    # model gate must not go on excusing them for the very widening the verdict applied
    # (item 124, :func:`_mark_model_reasons`).
    held_now: set[str] = set()
    for b, was_swapped in swapped.items():
        if not was_swapped:
            continue
        occ_event = b in crossed
        verdict = _swap_event(
            mem["bays"][b],
            float(ts),
            spec,
            SETTLE_OCCUPANCY if occ_event else SETTLE_JUMP,
            limited=not occ_event,
        )
        resets[b] = verdict == SWAP_EVENT
        if verdict == SWAP_HELD:
            held_now.add(b)

    # -- store the filter ----------------------------------------------------------------
    for z, (x, p) in arrays.items():
        if not (np.all(np.isfinite(x)) and np.all(np.isfinite(p))):
            raise FloatingPointError(f"estimator: non-finite state in zone {z!r}")
        if np.any(np.diag(p) < -1e-9):
            raise FloatingPointError(f"estimator: negative variance in zone {z!r}")
        mem["zones"][z]["x"] = x.tolist()
        mem["zones"][z]["P"] = p.tolist()

    # -- association: series, SMART histories, scoring -----------------------------------
    taken = {serial for serial, _ in assoc.values()}
    unassigned = [s for s in fresh_smart if s not in taken and s not in declared_serials]
    # Correlation pairs are re-checked against the same statistic that accepted them
    # (item 18), so their histories keep being recorded instead of being cleared.
    held = {b: serial for b, (serial, source) in assoc.items() if source == "correlation"}
    tracked = set(unassigned) | set(held.values())
    window = spec.associate_window_s
    min_span = associate.MIN_SPAN_FRACTION * window
    for serial, entry in list(mem["smart"].items()):
        if serial in tracked and serial in new_samples:
            sample_ts, temp = new_samples[serial]
            entry["hist"] = associate.record_smart(entry["hist"], sample_ts, temp, ts, window)
        elif serial not in tracked:
            entry["hist"] = []
        if serial not in fresh_smart and float(ts) - entry["ts"] > window:
            del mem["smart"][serial]
    candidates = [
        b
        for b, bay in st.bays.items()
        if b not in assoc and mem["bays"][b]["occ"] != EMPTY and b not in declared
    ]
    scoring = bool(unassigned and candidates)
    if scoring or held:
        bay_y: dict[str, float | None] = {}
        for b, bay in st.bays.items():
            if bay.zone in arrays:
                x, _ = arrays[bay.zone]
                n = len(st.zones[bay.zone].bays)
                bay_y[b] = float(x[2 + n + bay.node] - x[0])
            else:
                bay_y[b] = None
        zone_air = {z: (float(arrays[z][0][0]) if z in arrays else None) for z in st.zones}
        mem["series"] = associate.record_series(mem["series"], float(ts), bay_y, zone_air, window)
        due = (
            mem["next_assoc"] is None
            or float(ts) >= mem["next_assoc"]
            or float(ts) < (mem["next_assoc"] - associate.EVERY_S)
        )
        if due:
            mem["next_assoc"] = float(ts) + associate.EVERY_S
            scores: dict[str, dict[str, float]] = {}
            if scoring:
                scores = associate.score_matrix(
                    mem["series"],
                    {s: mem["smart"][s]["hist"] for s in unassigned},
                    unassigned,
                    candidates,
                    {b: st.bays[b].zone for b in candidates},
                    min_span_s=min_span,
                )
                mem["pending"], confirmed = associate.confirm(
                    mem["pending"],
                    associate.assign(scores, spec.associate_min_corr, spec.associate_margin),
                )
                for b, serial in confirmed.items():
                    # A pair starts unverified and with no evidence behind it (item 18):
                    # the window that accepted it must not count again as its own first
                    # re-check, so the serial's SMART history starts over and the pair
                    # has to rebuild a full window before its map may calibrate the bay.
                    bm = mem["bays"][b]
                    bm["assoc"], bm["rej"], bm["ver"] = serial, 0, False
                    mem["smart"][serial]["hist"] = []
            else:
                mem["pending"] = {}
            mem["scores"] = scores
            _recheck_associations(mem, st, held, spec, min_span, assoc)
    else:
        mem["series"] = None
        mem["scores"] = {}
        mem["pending"] = {}
        mem["next_assoc"] = None

    mem["ts"] = float(ts)

    # -- outputs ----------------------------------------------------------------------------
    estimates_out: dict[str, dict[str, Any]] = {}
    bays_out: dict[str, dict[str, Any]] = {}
    zones_out: dict[str, dict[str, Any]] = {}
    for z, zone in st.zones.items():
        if z not in arrays:
            zones_out[z] = {"initialised": False}
            continue
        x, p = arrays[z]
        n = len(zone.bays)
        drift = 0.0
        g = G0_W_PER_K + K_W_PER_K * geometry[z][1]
        for j, b in enumerate(zone.bays):
            if mem["bays"][b]["occ"] == EMPTY:
                continue
            cd = drive_capacity(cfg, classes[b][0])
            f = float(x[2 + n + zone.n_nodes + j]) - g * float(x[2 + j] - x[0]) / cd
            drift = max(drift, abs(f) * 60.0)
        zones_out[z] = {
            "initialised": True,
            "t_air_c": float(x[0]),
            "sigma_air_c": math.sqrt(max(float(p[0, 0]), 0.0)),
            "air_blind_s": float(mem["zones"][z]["blind"]),
            "d_air_c_per_s": float(x[1]),
            "t_in_c": geometry[z][2],
            "airflow_w_per_k": geometry[z][0],
            # which curve produced that airflow (item 107): the online fit, fan_models,
            # or mixed when only some of the zone's fan models have a fitted curve
            "airflow_curve": geometry[z][3],
            "drift_c_per_min": drift,
        }
    for b, bay in st.bays.items():
        bm = mem["bays"][b]
        cls, cls_source = classes[b]
        serial, assoc_source = assoc.get(b, (None, None))
        entry, cal_kind = cal_in_force(b)
        # A SMART map needs its association re-verified (item 18); a manual one
        # (item 23) has no association to re-check.
        map_verified = verified(b) if cal_kind == "smart" else True
        calibrated = _calibrated(entry, float(ts), max_age_cal) and map_verified
        sigma_cal = (
            max(SIGMA_CAL_FLOOR_C, math.sqrt(float(entry["rms2"])))  # type: ignore[index]
            * float(entry.get("inflate", 1.0))  # type: ignore[union-attr]
            if calibrated
            else spec.sigma_uncalibrated_c
        )
        offsets_c = _offsets_view(st.zones[bay.zone], bay, arrays.get(bay.zone), per_sensor)
        bay_rise = _bay_rise(st.zones[bay.zone], bay, arrays.get(bay.zone))
        info: dict[str, Any] = {
            "zone": bay.zone,
            "occupancy": bm["occ"],
            "declared": topo.bays[b].occupied,
            "since_ts": bm["since"],
            "pending_empty_s": float(bm["low"]),
            "pending_occupied_ticks": int(bm["rise"]),
            "pending_unknown_s": float(bm["blind"]),
            "assoc_check_fails": int(bm["rej"]),
            "swapped": bool(swapped.get(b, False)),
            # items 123, 124: the verdict's rate-limited *event* -- what ``mpc.step``
            # hands to ``thermal.update(reset_bays=...)`` -- and the record it leaves, so
            # a bay whose model was thrown away says so afterwards instead of only on
            # the tick it happened.
            "swap_reset": bool(resets.get(b, False)),
            "swap_count": int(bm["swaps"]),
            "swap_last_s": None if bm["swap_ts"] is None else float(ts) - float(bm["swap_ts"]),
            "swap_reason": bm["swap_why"],
            "swap_held": int(bm["swap_held"]),
            "observed": observed.get(b, False),
            "seeded": bool(bm["init"]),
            "offsets_c": offsets_c,
            "proximal_map": _proximal_map_view(bay, bm["dmap"], member_map, sensor_map(b)[:2]),
            # item 125: whether the pair still sits where it did, and -- said plainly --
            # which of the two halves of ``ds * rise + db`` this layout can separate
            "placement": _placement_view(bay, bm, offsets_c, spec, per_sensor, bay_rise),
            "class": cls,
            "class_source": cls_source,
            "serial": serial,
            "association": assoc_source,
            "calibrated": calibrated,
            "sigma_cal_c": sigma_cal,
            "calibration_source": cal_kind,
            "calibration": None
            if entry is None
            else {
                "slope": entry["th"][0],
                "offset_c": entry["th"][1],
                "slope_var": entry["P"][0][0],
                "samples": entry["n"],
                "fresh_samples": entry["fresh"],
                "rms_c": math.sqrt(float(entry["rms2"])),
                "age_s": None if entry["ts"] is None else float(ts) - entry["ts"],
                # The thermal identification anchors on this flag (``mpc._thermal_shadow``),
                # so it says what the filter itself does: a pair the estimator refuses to
                # use is not a map the RLS may convert a row with either (item 18).
                "accepted_once": bool(entry["used"]) and map_verified,
            },
            "candidates": associate.top_candidates(mem["scores"], b) if b not in assoc else [],
        }
        if b in evidence:
            info["delta_t_c"], info["heat_w"] = evidence[b]
        settling = _settling(bm, float(ts), spec.bay_settle_s)
        budget = spec.bay_settle_max_s
        estimated = bay.zone in arrays and bm["occ"] != EMPTY
        sigma: float | None = None
        if estimated:
            x, p = arrays[bay.zone]
            i_d, i_s, i_q = _slots(st.zones[bay.zone], bay)
            sigma = math.sqrt(max(float(p[i_d, i_d]), 0.0) + sigma_cal * sigma_cal)
        # item 100: the estimator, not the solver, decides why a bay is not itself
        _mark_model_reasons(
            bm, float(ts), spec, sigma, sigma_cal if estimated else None, scored=b in held_now
        )
        info.update(_settle_view(bm, float(ts), spec, settling))
        bays_out[b] = info
        if not estimated:
            # No sigma check to suspend: the trust rule skips such a bay outright.
            _account_settle(
                bm, settling=settling, checked=False, over=False, h=h_pred, budget=budget
            )
            continue
        assert sigma is not None
        _account_settle(
            bm,
            settling=settling,
            checked=bool(info["observed"]),
            over=sigma > spec.sigma_fault_c,
            h=h_pred,
            budget=budget,
        )
        limit = cfg.drive_classes[cls].limit_c
        own = topo.bays[b].limit_c
        estimates_out[b] = estimate_entry(
            zone=bay.zone,
            drive_class=cls,
            occupancy=bm["occ"],
            t=float(x[i_d]),
            sigma=sigma,
            k_sigma=spec.k_sigma,
            limit=limit if own is None else min(limit, own),
            comfort=cfg.drive_classes[cls].comfort_c,
            source=SOURCE_ESTIMATOR,
            calibrated=calibrated,
            q_w=float(x[i_q]) * drive_capacity(cfg, cls),
            t_sensor=float(x[i_s]),
            sigma_cal=sigma_cal,
        )
    summary = {
        "smart_used": mem["count"]["smart_used"],
        "smart_rejected": mem["count"]["smart_rejected"],
        "manual_used": mem["count"]["manual_used"],
        "manual_rejected": mem["count"]["manual_rejected"],
        "smart_fresh": list(fresh_smart),
        "manual_fresh": list(fresh_manual),
        "unassigned": unassigned,
    }
    return EstimatorUpdate(
        estimates=estimates_out, memory=mem, zones=zones_out, bays=bays_out, summary=summary
    )


# ---------------------------------------------------------------------------
# model store
# ---------------------------------------------------------------------------


def _restored_entry(raw: object, ts: float, stale: bool) -> dict[str, Any]:
    """One stored calibration entry in the new clock (module docstring); ``ValueError``
    names the first problem."""
    if not isinstance(raw, Mapping):
        raise ValueError("not a mapping")
    th = raw.get("th")
    if not isinstance(th, list) or len(th) != 2 or not all(_finite(v) for v in th):
        raise ValueError("th must be two finite numbers")
    slope, offset = float(th[0]), float(th[1])
    if not CAL_SLOPE_BOUNDS[0] <= slope <= CAL_SLOPE_BOUNDS[1]:
        raise ValueError(f"slope {slope} outside {list(CAL_SLOPE_BOUNDS)}")
    if not CAL_OFFSET_BOUNDS[0] <= offset <= CAL_OFFSET_BOUNDS[1]:
        raise ValueError(f"offset {offset} outside {list(CAL_OFFSET_BOUNDS)}")
    p = raw.get("P")
    if (
        not isinstance(p, list)
        or len(p) != 2
        or not all(isinstance(row, list) and len(row) == 2 for row in p)
        or not all(_finite(v) for row in p for v in row)
    ):
        raise ValueError("P must be a finite 2x2 matrix")
    cov = [[float(v) for v in row] for row in p]
    if (
        cov[0][0] < 0
        or cov[1][1] < 0
        or abs(cov[0][1] - cov[1][0]) > 1e-6 * max(1.0, abs(cov[0][0]), abs(cov[1][1]))
    ):
        raise ValueError("P is not a covariance")
    n_all, fresh = raw.get("n"), raw.get("fresh")
    for count in (n_all, fresh):
        if isinstance(count, bool) or not isinstance(count, int) or count < 0:
            raise ValueError("n and fresh must be integers >= 0")
    assert isinstance(n_all, int) and isinstance(fresh, int)
    if fresh > n_all:
        raise ValueError("fresh exceeds n")
    rms2 = raw.get("rms2")
    if not _finite(rms2) or float(rms2) < 0:  # type: ignore[arg-type]
        raise ValueError("rms2 must be a finite number >= 0")
    used = raw.get("used", False)
    if not isinstance(used, bool):
        raise ValueError("used must be a bool")
    age = raw.get("age_s")
    entry: dict[str, Any] = {
        "th": [slope, offset],
        "P": cov,
        "n": n_all,
        "fresh": fresh,
        "rms2": float(rms2),  # type: ignore[arg-type]
        "ts": None,
        "used": used,
    }
    if _finite(age) and float(age) >= 0 and raw.get("expired") is not True:  # type: ignore[arg-type]
        entry["ts"] = float(ts) - float(age)  # type: ignore[arg-type]
    else:  # unknown, negative or expired age: restart the fresh count (as on expiry)
        entry["fresh"] = 0
        if _finite(age) and float(age) >= 0:  # type: ignore[arg-type]
            entry["ts"] = float(ts) - float(age)  # type: ignore[arg-type]
    if stale:
        entry["inflate"] = STALE_SIGMA_CAL_FACTOR
        entry["confirm"] = CAL_MIN_SAMPLES
    elif "inflate" in raw:
        inflate, confirm = raw.get("inflate"), raw.get("confirm")
        if (
            not _finite(inflate)
            or float(inflate) < 1.0  # type: ignore[arg-type]
            or isinstance(confirm, bool)
            or not isinstance(confirm, int)
        ):
            raise ValueError("inflate must be a number >= 1 with an integer confirm")
        if confirm > 0:
            entry["inflate"] = float(inflate)  # type: ignore[arg-type]
            entry["confirm"] = confirm
    return entry


def restore_calibration(
    memory: object, calibration: object, cfg: MpcConfig, *, ts: float, stale: bool
) -> tuple[dict[str, Any], list[str]]:
    """``(estimator memory, warnings)`` with the model store's calibrations installed.

    ``calibration`` is ``{bay: {serial: {"th", "P", "n", "fresh", "rms2", "used",
    "age_s", "expired"?, "inflate"?, "confirm"?}}}`` (``age_s``: seconds since the entry's
    last accepted SMART sample at load time, ``None`` when unknown) and ``ts`` the clock of
    the tick it is applied on. A bay that is not in the topology or an entry that is
    malformed, not finite or outside the bounds (slope in :data:`CAL_SLOPE_BOUNDS`,
    offset in :data:`CAL_OFFSET_BOUNDS`) is dropped with a warning; at most
    ``CAL_SERIALS_PER_BAY`` entries per bay are kept (the most recent). Entries already in
    ``memory`` (same bay and serial) are kept as they are. Pure; never raises for a bad
    ``calibration`` (only for a legacy ``cfg``).
    """
    if cfg.topology is None or cfg.estimator is None:
        raise ValueError("the estimator needs a zoned config (mpc.topology)")
    st = _structure(cfg)
    mem = _load(memory, st)
    warnings: list[str] = []
    if not isinstance(calibration, Mapping):
        return mem, ["calibration: not a mapping, section dropped"]
    for bay, per_serial in calibration.items():
        if bay not in st.bays:
            warnings.append(f"calibration: bay {bay!r} is not in the topology, dropped")
            continue
        if not isinstance(per_serial, Mapping):
            warnings.append(f"calibration: bay {bay!r} is not a mapping, dropped")
            continue
        entries: dict[str, dict[str, Any]] = {}
        for serial, raw in per_serial.items():
            if not isinstance(serial, str) or not serial:
                warnings.append(f"calibration: bay {bay!r} has an invalid serial, dropped")
                continue
            try:
                entries[serial] = _restored_entry(raw, ts, stale)
            except ValueError as exc:
                warnings.append(f"calibration: {bay}/{serial}: {exc}; dropped")
        newest = sorted(entries, key=lambda s: (-(entries[s]["ts"] or -1e300), s))
        per_bay = mem["cal"].setdefault(bay, {})
        for serial in newest[:CAL_SERIALS_PER_BAY]:
            per_bay.setdefault(serial, entries[serial])
        if not per_bay:
            del mem["cal"][bay]
    return mem, warnings


def _declaration_change(cfg: MpcConfig, bay: str, declared: object) -> str | None:
    """Why a stored manual calibration no longer belongs to ``bay``, ``None`` when it
    still does (module docstring, *swapped*; :meth:`MpcConfig.bay_declaration`)."""
    now = cfg.bay_declaration(bay)
    if now["occupied"] is False:
        return "the config now declares the bay empty"
    if not isinstance(declared, Mapping):
        return "it carries no record of the bay's declaration when it was saved"
    for key in ("occupied", "class", "serial"):
        was = declared.get(key)
        if was != now[key]:
            return f"the bay's declared {key} was {was!r} and is now {now[key]!r}"
    return None


def restore_manual_calibration(
    memory: object, manual: object, cfg: MpcConfig, *, ts: float
) -> tuple[dict[str, Any], list[str]]:
    """``(estimator memory, warnings)`` with the model store's manual calibrations
    installed (module docstring, *Manual calibration*; PROJECT.md section 8 item 104).

    ``manual`` is ``{bay: {"th", "P", "n", "fresh", "rms2", "used", "age_s"[,
    "age_reason"], "declared": {"occupied", "class", "serial"}}}`` -- ``age_s`` the
    seconds since the entry's last accepted hand reading at load time, ``None`` when
    unknown, with the optional ``age_reason`` naming *why* it is unknown (no saved sample
    time, against a wall clock behind the file) so the warning does not point at one
    wrong cause -- and ``ts`` the clock of the tick it is applied on. Every doubtful entry
    is **dropped** with a named warning: an unknown bay, an entry that is malformed or out
    of bounds, an age that is unknown, negative or past
    ``estimator.manual_calibration_max_age_days``, or a bay whose declaration has changed
    since the file was written. What survives is
    re-based on ``ts`` and always inflated (``inflate`` / ``confirm``), a fresh file
    included. A bay already in ``memory`` is left alone. Pure; never raises for a bad
    ``manual`` (only for a legacy ``cfg``).
    """
    if cfg.topology is None or cfg.estimator is None:
        raise ValueError("the estimator needs a zoned config (mpc.topology)")
    st = _structure(cfg)
    mem = _load(memory, st)
    warnings: list[str] = []
    if not isinstance(manual, Mapping):
        return mem, ["manual_calibration: not a mapping, section dropped"]
    window = cfg.estimator.manual_calibration_max_age_days * _DAY_S
    for bay, raw in manual.items():
        where = f"manual_calibration: bay {bay!r}"
        if bay not in st.bays:
            warnings.append(f"{where} is not in the topology, dropped")
            continue
        if not isinstance(raw, Mapping):
            warnings.append(f"{where} is not a mapping, dropped")
            continue
        if bay in mem["manual"]:
            continue  # this run already has one: the live entry wins
        change = _declaration_change(cfg, str(bay), raw.get("declared"))
        if change is not None:
            warnings.append(f"{where}: {change}; dropped (item 104)")
            continue
        age = raw.get("age_s")
        if not _finite(age) or float(age) < 0.0:  # type: ignore[arg-type]
            reason = raw.get("age_reason")
            why = (
                reason
                if isinstance(reason, str) and reason
                else "it carries no saved sample time, or the wall clock is behind the file"
            )
            warnings.append(
                f"{where}: the age of its last hand reading is unknown ({why}); "
                "dropped rather than trusted"
            )
            continue
        if float(age) > window:  # type: ignore[arg-type]
            warnings.append(
                f"{where}: its last hand reading is {float(age) / _DAY_S:.1f} days old, past "  # type: ignore[arg-type]
                f"estimator.manual_calibration_max_age_days "
                f"({cfg.estimator.manual_calibration_max_age_days:g}); dropped"
            )
            continue
        try:
            # ``stale=True`` unconditionally: a bay-keyed map has no feed that could
            # re-associate it, so it comes back provisional however fresh the file is.
            entry = _restored_entry(raw, ts, True)
        except ValueError as exc:
            warnings.append(f"{where}: {exc}; dropped")
            continue
        if entry["ts"] is None:  # no usable sample time: nothing to age the entry by
            warnings.append(f"{where}: it has no usable sample time; dropped")
            continue
        mem["manual"][str(bay)] = {"ts": float(entry["ts"]), "cal": entry}
    return mem, warnings


def _recheck_associations(
    mem: dict[str, Any],
    st: _Structure,
    held: Mapping[str, str],
    spec: Any,
    min_span_s: float,
    assoc: dict[str, tuple[str, str]] | None = None,
) -> None:
    """Re-score every correlation pair against its own bay and drop the ones that stopped
    correlating (module docstring, *Association*; item 18). In place, pure.

    A pair whose history is too short for a score yet (a fresh association, a serial that
    was briefly silent) is left alone. ``associate_drop_checks`` consecutive scores below
    ``associate_drop_corr`` end the association through :func:`_forget_pair`. Keeping a
    pair deliberately asks less than choosing one (``associate_min_corr``): on the truth
    simulator a correct pair scores 0.71-0.97 over the window and dips to 0.37 through a
    quiet one, while a wrong pair sits at a median of -0.27 to +0.24.
    """
    if not held:
        return
    floor = spec.associate_drop_corr
    scores = associate.score_matrix(
        mem["series"],
        {
            serial: mem["smart"][serial]["hist"]
            for serial in held.values()
            if serial in mem["smart"]
        },
        sorted(set(held.values())),
        sorted(held),
        {b: st.bays[b].zone for b in held},
        min_span_s=min_span_s,
    )
    for b, serial in held.items():
        score = scores.get(serial, {}).get(b)
        if score is None:
            continue
        bm = mem["bays"][b]
        if score >= floor:
            bm["rej"], bm["ver"] = 0, True
            continue
        bm["rej"] += 1
        if bm["rej"] >= spec.associate_drop_checks:
            _forget_pair(mem, b, serial)
            if assoc is not None:
                assoc.pop(b, None)  # this tick's output already shows the bay unpaired


def _declared_state(declared: bool | str, current: str) -> str:
    if declared is True:
        return OCCUPIED
    if declared is False:
        return EMPTY
    return current
