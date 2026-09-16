"""Zoned DAS thermal model: structure, linearisation and online identification (plan sections 3, 5).

Pure: every function here is a function of its arguments. :func:`update` keeps
its state in plain JSON (``solver_memory["thermal"]``), reads no clock, does no
I/O and draws no random numbers. ``mpc.step`` calls it only with
``model_shadow: true`` and after the command of the tick is final, so the
model *learns and predicts without acting*: nothing here feeds a PWM value
directly. The DAS MPC (:mod:`aqua_bridge.control.solver_das`) reads the identified
parameters of the previous tick's memory through :func:`current_model` and
linearises with :func:`model_params`, :func:`derivatives`, :func:`jacobians` and
:func:`discretise`.

Model structure (SI units: W, J/K, W/K, degC, s)
------------------------------------------------
Per zone ``z`` one air node ``T_a,z``; per bay ``j`` a latent drive node
``T_d,j`` and its proximal sensor node ``T_s,j``. State order of the network:
``x = [T_a (zones), T_d (bays), T_s (bays)]`` in topology order::

    C_a dT_a,z/dt = sum_j g_j (T_d,j - T_a,z) - (Q_z + leak_z)(T_a,z - T_in,z)
                    + sum_z' kappa_zz' (T_a,z' - T_a,z) + p_air,z + C_a d_air,z
    C_d dT_d,j/dt = C_d q_j - g_j (T_d,j - T_a,z)
    tau_s dT_s,j/dt = s_j T_d,j + (1 - s_j) T_a,z + b_j - T_s,j          s_j = 1 - beta_j

    g_j = g0_j + k_j Qn_z,   Q_z = sum_G E_zG phi_zG(u),   Qn_z = Q_z / sum_G E_zG

An ``empty`` bay has no drive: its drive row is frozen (no dynamics) and its
sensor follows the air (``tau_s dT_s/dt = T_a - T_s``); an ``unknown`` bay is
modelled like an occupied one (conservative). ``q`` (K/s per bay) and
``d_air`` (K/s per zone) are the estimator's integrating disturbances and enter
as affine inputs.

Airflow. Channel ``i`` with fan model ``m`` moves air in proportion to
``phi_i(u) = clip((u - u0_m) / (1 - u0_m), 0, 1) ** n_m`` (``deadband``,
``exponent``), or ``clip(rpm / rpm_max, 0, 1) ** n_m`` for identification when
``model_use_rpm`` is on and the channel reports a finite rpm. ``rpm_max`` there is
always ``fan_models.<m>.rpm_max``, the commissioned reference, never a fitted one:
that fixed divisor is what lets the tach branch see a fan that has lost speed
(``fan_curve_online`` gives the rpm branch its ``exponent`` and nothing else).
``E`` is sparse by
declaration: channel ``i`` reaches zone ``z`` with the prior weight
``w_zi = 33 W/K * count_i / (zones listing i)`` when ``z`` lists it, ``0.1 w``
when ``z`` is declared ``coupled_to`` a zone that lists it (the weak cross-zone
prior), else not at all. Channels of one ``fans.<ch>.group`` (default: the
channel itself) share one coefficient per zone, ``E_zG`` (prior
``sum_{i in G} w_zi``), with ``phi_zG = sum_i w_zi phi_i / sum_i w_zi``: one
multiplier on the group's prior effectiveness.

**Split** (``model_split_channels``, plan section 8 item 13). Regulation moves a
group's channels together, so only ``E_zG`` is identifiable from it; the
single-channel phases of an experiment (``control/ident.py``: the whole group,
then each channel alone) move them apart. With the switch on, each group of more
than one channel in a zone carries one extra coefficient per channel beyond the
first, ``Es.<z>.<G>.<ch>``, in the same air-block regression::

    Q_zG = E_zG phi_zG + sum_{ch != ref} Es_zGch (phi_ch - phi_zG)

The extra regressors are **exactly zero** while the group's channels hold the same
duty, so under regulation the ridge keeps them at their prior of 0 and the model is
the shared-``E`` model, term for term. A single-channel phase makes them the only
regressors that move, and what they learn is a redistribution: the implied per-channel
effectiveness is ``E_ch = E_zG w_ch / W + Es_ch - (w_ch / W) sum Es``
(:func:`channel_effectiveness`), whose sum over the group is ``E_zG`` whatever the
split. So the split never changes what the group as a whole does, only how its
channels share it -- and the convergence rules below still read the group
coefficients alone (``Es`` is not a gain for ``rel_se``, and the excitation and PE
monitors still see ``phi_zG``), so turning the switch on cannot stop a model
converging. ``GET /api/model`` reports ``e_per_channel`` per zone.

After every window -- and when a stored memory is read -- the split is projected by
:func:`split_scale`: scaled back by the largest factor in ``[0, 1]`` that leaves every
``E_ch >= 0``. ``Q_zG = sum_ch E_ch phi_ch`` exactly, so a feasible split can never
reverse a zone's modelled airflow, and a split at its prior (or any other feasible one)
is left untouched.

Parameter table (:data:`PARAMETERS`; keys as in :func:`parameter_keys`)
------------------------------------------------------------------------

::

    key          unit  bounds          prior                           identified from
    E.z.G        W/K   [0, 200]        33/fan x count / zones listing  air-node RLS, fan excitation
                                       (0.1x for a coupled zone)       (weak cross-zone E: ridge)
    Es.z.G.ch    W/K   [-200, 200]     0 (model_split_channels)        air-node RLS, single-channel
                                                                       excitation only (ridge to 0)
    leak.z       W/K   [0, 20]         1                               air-node RLS, ridge (weak)
    kappa.z.z2   W/K   [0, 50]         3 (declared pairs only)         air-node RLS, ridge (weak)
    p_air.z      W     [-100, 100]     0                               air-node RLS constant
    g0.j         W/K   [0.05, 5]       0.3                             proximal RLS, ridge (weak)
    k.j          W/K   [0.05, 5]       0.5                             proximal RLS, fan excitation
    q_s.j        K/s   [-0.2, 0.2]     0                               proximal RLS constant
    c_air.z      J/K   [50, 2000]      200                             prior only (not identifiable)
    c_drive.j    J/K   [50, 2000]      tau_d_s * (g0 + k)              prior only (SMART, later)
    beta.j       -     [0.02, 0.7]     0.3                             SMART calibration (estimator)
    b.j          degC  [-10, 10]       -2.1                            SMART calibration (estimator)
    tau_s.j      s     [3, 120]        sensors.<name>.tau_s, else type  prior only (config)
    u0.m         -     [0, 0.5)        fan_models.<m>.deadband         config, tools/fit_fans.py,
    n.m          -     [0.5, 1.5]      fan_models.<m>.exponent         or the online fan-curve fit
                                                                       (control/fancurve.py,
                                                                       fan_curve_online)

Linearisation and discretisation
--------------------------------
:func:`jacobians` returns the continuous ``A = df/dx``, ``B = df/du`` and the
affine term ``c = f(x0, u0) - A x0 - B u0`` at an operating point (analytic;
``dphi/du`` is the one-sided derivative, 0 outside the dead band and above
full speed). :func:`discretise` computes the exact zero-order-hold
discretisation ``x+ = Ad x + Bd u + cd`` by eigendecomposition
(``Ad = V exp(L h) V^-1``, ``Bd = V diag((exp(l h) - 1) / l) V^-1 B``, ``cd``
likewise). The RC network is similar to a symmetric matrix, so its eigenvalues
are real and negative; when an eigenvalue has an imaginary part above
:data:`EIG_IMAG_TOL` or ``cond(V) > EIG_COND_MAX`` the fallback is forward
Euler with ``N = max(4, ceil(h * ||A||_inf / 0.5))`` substeps (the plan's four
substeps are unstable for the fast air nodes at the MPC's 30 s prediction
step, so the count grows with the stiffness; conservative deviation).

Identification (plan section 5)
-------------------------------
Two windowed integral regressions per zone. A window accumulates over contiguous
ticks on which the zone is trusted and not in fault and the set of gate-trusted
sensors its regression reads (all members of each averaged group that are trusted)
stays the same, with at least one per group (any other tick restarts it: redundant
sensors disagree by their placement offsets, and the lag correction below would turn
the step of a mean over a different set into a spike); the command
``u = prev`` is held over each interval and the temperatures follow the trapezoid
rule. It closes after ``model_window_s``.

**Window weight (modulating function).** Both sides of each equation are
integrated against ``w(t) = sin^2(pi t / T_w)``, which vanishes with its
derivative at both ends, so ``int w dX/dt dt = -int dw/dt X dt`` needs no endpoint
values and no derivative of the data. This replaces the plan's plain integral
(``T[k+M] - T[k] = sum ...``): with plain windows a fan step that lands on a
window edge (every experiment whose holds are multiples of the window does) puts
the lagging sensors' transient into the endpoint values, which biased the fan
gains 10-15 % low on the truth simulator. The effective length of the weighted
window is about half of ``T_w``, hence the default 120 s (the plan's 60 s
rectangle).
A closing window is used only when its samples resolve the weight: the trapezoid
sums over the actual sample times must give ``int w = T_w / 2`` within
:data:`WINDOW_MASS_TOL` and ``int dw = int d2w = 0`` within :data:`WINDOW_SUM_TOL`
of their scales (exact for uniform sampling). A window of a few ticks spanned by a
missed tick fails that and is dropped: its row would otherwise carry the absolute
temperature through the ``dw`` / ``d2w`` sums, or divide by a weight of ~0.

**Sensor lag.** Every sensor is modelled as a first-order lag of its node,
``X = X_sensor + tau dX_sensor/dt`` (``tau`` from ``sensors.<name>.tau_s``, else
:data:`LAG_THERMISTOR_S` for a thermistor-like ``quant_c <= 0.02`` and
:data:`LAG_DS18B20_S` otherwise). Under the window integral that is exact and
needs only increments: ``int w tau dX/dt dt ~ tau sum w_mid (X_k - X_k-1)`` and, by
parts, ``int dw/dt tau dX/dt dt = -tau int d2w/dt2 X dt``. The inlet is not
lag-corrected (it moves slowly; the correction only added noise). Without the
correction the 15 s DS18B20 lag biased ``k`` about -12 % and the 5 s zone-air
thermistor lag biased ``E`` about -15 % (measured).

* **Proximal sensor, per bay** (occupied or unknown), the rc form on the measured
  node through the bay's sensor map ``(s, b)``. With the unlagged sensor target
  ``S = s T_d + (1 - s) T_a + b`` in the drive equation::

      -int dw S + (1 - s) int dw T_a = q_s int w - (g0 / C_d) int w (S - T_a - b)
                                               - (k / C_d) int w Qn (S - T_a - b)

  ``g0`` and ``k`` come out in W/K on the scale of the class prior ``C_d``
  (the plan's ``[q'_s, g0'_j, k'_j]``).

* **Zone air**, anchored on the drive heat. The plan writes the air node in the
  dynamic form ``dT_a = int [...] . theta`` with free per-bay drive terms. At
  ``dt = 5 s`` the zone air time constant is 3-8 s: the sums cannot resolve the
  transient that carries ``1 / C_a``, the equation is effectively algebraic and only
  ratios are identifiable. On the truth simulator that form returned every air
  coefficient at about 0.35x its value and per-bay drive terms with the wrong sign
  (deviation, measured). The air block therefore regresses the weighted balance
  with the drive heat ``H_z = sum_j g_j (S_j - T_a - b_j) / s_j`` (the
  controller's own proxy, from the per-bay coefficients above) as the anchor that
  fixes the scale, normalised by ``int w``::

      int w H_z + C_a int dw T_a = int w [ sum_G E_zG phi_zG (T_a - T_in)
                                           + leak (T_a - T_in)
                                           - sum_z2 kappa_zz2 (T_a,z2 - T_a) - p_air ]

  ``C_a`` (prior) only carries the net air change. ``E`` is on the scale of ``C_d``
  and the sensor map, the scale the estimator and the MPC use for the drive heat,
  so predicted temperatures do not depend on it; its absolute W/K value is as good
  as those priors (SMART calibration, class time constants). Two limits, both
  measured: the air rise ``T_a - T_in`` of a DAS zone is only 0.3-1 degC, so a
  relative offset between the zone-air and inlet sensors of 0.1 degC already moves
  ``E`` by 15-35 % (the per-bay ``k`` is unaffected); and ``p_air`` competes with
  the dominant group's ``E`` (``phi (T_a - T_in)`` is nearly constant when one
  group dominates), so it is ridged toward 0 and misc heat in the zone biases
  ``E`` by roughly its share of the zone's heat.

Constrained RLS per block, in coordinates scaled by the prior magnitude
(``psi = theta / scale``) and rows normalised by the running RMS of the a-priori
residual:

* **conditional update**: a window is *excited* when the weighted mean of a
  fan-direction regressor (``phi_zG`` of an in-zone group for the air block,
  ``Qn_z`` for a proximal block) moves by more than :data:`EXCITATION_MIN`
  relative to its running mean. Otherwise only the constant (``p_air`` /
  ``q_s``) is updated (a Schmidt "consider" update: full row, gain on the constant
  alone), with no forgetting on the rest, so the covariance never winds up at
  equilibrium;
* **forgetting** ``P /= model_lambda`` per excited window, and a random walk on
  the constants every window (heat changes);
* **ridge** toward the prior for the weak directions (``leak``, ``kappa``,
  weak cross-zone ``E``, ``g0``, and ``p_air`` toward 0): one pseudo-measurement
  per excited window with variance :data:`RIDGE_VAR` in scaled units;
* **covariance trace bound** ``trace(P) <= model_p_trace_max``;
* **trust region**: ``|delta psi_i| <= 5 %`` of ``max(|psi_i|, 1)`` per window on
  every coefficient except the constants;
* **projection** onto the bounds of the table;
* residuals beyond :data:`HUBER_SIGMAS` sigma are clipped (a hot swap or a glitch
  must not throw the fit); every update is in Joseph form, so ``P`` stays
  positive semi-definite.

**PE monitor** ``pe_min``: smallest eigenvalue of the information matrix of the
fan-direction regressors over the last ~30 windows (exponentially weighted),
centred and normalised by their means, so it reads as the squared relative
variation in the least excited direction (``[0, 1]``). With several groups moving
together (a regulator drives them from the same drive) it is ~0.

**Prediction error** ``pred_err_c``: the one-window-ahead prediction error of the
regressions, i.e. the a-priori residual before the window updates the
coefficients, in degC (proximal: kelvin of the weighted window, where a change of
the sensor target in mid-window counts 1; air: W divided by the zone's total
conductance ``Q + leak + sum kappa + sum g``), as an exponentially weighted RMS per
zone; the model's value is the worst zone. It is an equation-error prediction
from measured inputs, not a free run of the model (the MPC milestone adds its own
rolling prediction check on the linearised model).

Status machine per zone (the model's status is the least advanced zone, with
``error`` and ``suspect`` taking precedence):

* ``prior`` -> ``learning`` on the first excited window;
* ``learning`` -> ``converged`` when the air block and every constrained proximal
  block have :data:`MIN_WINDOWS` excited windows, ``pe_min > PE_MIN``, every
  in-zone ``E`` and every ``k`` a relative standard error below
  ``model_converged_rel_se``, and ``pred_err_c < model_max_pred_err_c``;
* ``converged`` -> ``suspect`` when the window error exceeds 3x its level at
  convergence (floored at :data:`PRED_ERR_FLOOR_C`) for :data:`SUSPECT_WINDOWS`
  consecutive windows;
* ``suspect`` -> ``learning`` when excitation returns;
* ``error``: ``step`` resets the memory to the prior with this status after any
  exception; the next excited window moves it to ``learning``;
* ``frozen``: a zone whose coefficients are held. Two ways in. (a) The model store: a
  zone that was ``converged`` or ``frozen`` when a file younger than
  ``model_store_max_age_days`` was saved loads ``frozen`` (:func:`restore`).
  (b) ``model_freeze: true``: a zone that reaches the ``converged`` rule is entered as
  ``frozen`` instead, so a model the owner considers converged stops adapting online
  (plan section 8 item 16). Either way the zone is accepted by the DAS MPC's validity
  gate like ``converged`` and its windows still close, score the prediction error and
  advance the PE monitor, but never move ``theta``/``P`` (``learn=False`` for its
  blocks). It becomes ``suspect`` by the same rule as ``converged`` and then learns
  again from ``learning``: the switch freezes a *good* model, it never holds a model
  the data has contradicted. A stored zone that had not converged keeps its status
  (conservative: a fresh file does not make a model that never converged act).

Hot swap (plan section 8 item 12): ``update`` takes ``reset_bays``, the bays the
estimator reported as ``swapped`` this tick (the occupancy crossed the ``empty``
boundary, or the fast-swap rule tripped). With ``model_reset_on_swap`` (default
``true``) each of them starts over from its prior -- ``g0``, ``k``, ``q_s``, the
covariance, the counters, the PE monitor and the window in progress -- because those
coefficients describe the drive that left, and the MPC's ``bay_settle_s`` exclusion
only covers the transient. The zone air block's window in progress goes too: it
anchors on the bay's heat through those very coefficients. The zone's *status* is
left alone; demoting it would park the DAS MPC in its PI-like fallback until the
next identification experiment, while the bay simply relearns like a new one. A
``frozen`` zone is skipped: it never moves its coefficients, so a reset there would
strand the bay at the prior.

Stale hold (model store, the owner's stale rule): a model restored from a file older
than ``model_store_max_age_days`` (or one saved while such a hold was still pending)
carries ``"hold": {"since": ts | None}``. Its zones restart at ``learning`` (a zone that
had not left ``prior`` stays there) with no prediction error, and the model's status is
``stale`` whatever its zones say, so neither the validity gate nor
``model_accept_prior`` lets it act. After every tick, ``since`` is set when every zone
is ``converged`` (or ``frozen``: with ``model_freeze`` a re-confirmed zone enters
``frozen``) and every zone's prediction error is below ``model_max_pred_err_c``,
and cleared when either fails; the hold is dropped (the model's status is its zones'
again) once that has lasted ``model_reconfirm_s``. The DAS MPC then still needs its own
checks, including its rolling one-step prediction error, for its dwell before it acts.

Memory (plain JSON)::

    {"v": 1, "fp": <structure fingerprint>, "ts": last ts, "error": str | None,
     "hold": {"since": ts | None} (only while a stale model is re-confirmed),
     "zones": {zone: {"status", "conv", "bad", "err2", "prev": sample, "air": block}},
     "bays": {bay: block}}
    block = {"theta": [...], "P": [[...]], "s2": float, "n": excited windows,
             "w": windows, "m": [...], "S": [[...]], "fm": [...] | None, "pe": float,
             "rel": [...] | None, "acc": {"x", "y", "h", "fan", "t0", "c"} | None}

A memory that does not match the config's structure, or is malformed in any way,
starts over (never an exception).
"""

from __future__ import annotations

import json
import math
from collections.abc import Collection, Mapping
from dataclasses import dataclass, field
from functools import cached_property
from typing import Any

import numpy as np

from aqua_bridge.control import fancurve
from aqua_bridge.control.estimates import PRIOR_BETA, PRIOR_OFFSET_C
from aqua_bridge.control.estimator import (
    C_AIR_J_PER_K,
    E_W_PER_K_PER_FAN,
    EMPTY,
    G0_W_PER_K,
    K_W_PER_K,
    KAPPA_W_PER_K,
    LEAK_W_PER_K,
    TAU_SENSOR_S,
    drive_capacity,
)
from aqua_bridge.model import MpcConfig

__all__ = [
    "EIG_COND_MAX",
    "EIG_IMAG_TOL",
    "EXCITATION_MIN",
    "HUBER_SIGMAS",
    "LAG_DS18B20_S",
    "MODEL_STATUSES",
    "LAG_THERMISTOR_S",
    "MIN_WINDOWS",
    "PARAMETERS",
    "PE_MIN",
    "PRED_ERR_FLOOR_C",
    "RIDGE_VAR",
    "STATUSES",
    "SUSPECT_WINDOWS",
    "TRUST_REGION",
    "WEAK_E_FACTOR",
    "Discretisation",
    "Linearisation",
    "ParamSpec",
    "Structure",
    "ThermalParams",
    "ThermalUpdate",
    "cached_structure",
    "channel_effectiveness",
    "current_model",
    "derivatives",
    "discretise",
    "fresh_memory",
    "jacobians",
    "model_status",
    "model_params",
    "overall_status",
    "parameter_keys",
    "phi",
    "prior_theta",
    "project",
    "restore",
    "sensor_lag_s",
    "split_scale",
    "state_jacobian",
    "structure",
    "summary",
    "theta_from_memory",
    "update",
]

VERSION = 1

#: Status machine states (module docstring).
#: Zone statuses (module docstring, *Status machine*).
STATUSES: tuple[str, ...] = ("prior", "learning", "converged", "suspect", "error", "frozen")
#: Statuses of the whole model: a zone status, or ``stale`` while a stale hold is pending.
MODEL_STATUSES: tuple[str, ...] = (*STATUSES, "stale")

#: Weak cross-zone effectiveness prior, as a fraction of the in-zone prior.
WEAK_E_FACTOR = 0.1

#: Discretisation: eigendecomposition acceptance, Euler stability target.
EIG_IMAG_TOL = 1e-9
EIG_COND_MAX = 1e8
EULER_MIN_SUBSTEPS = 4
EULER_STEP_NORM = 0.5

#: Sensor lag defaults when ``sensors.<name>.tau_s`` is absent (module docstring).
THERMISTOR_MAX_QUANT_C = 0.02
LAG_THERMISTOR_S = 5.0
LAG_DS18B20_S = TAU_SENSOR_S

#: Identification constants (module docstring).
EXCITATION_MIN = 0.05
RIDGE_VAR = 1.0
TRUST_REGION = 0.05
HUBER_SIGMAS = 5.0
PE_WINDOWS = 30
PE_MIN = 0.05
MIN_WINDOWS = 30
PRED_ERR_FLOOR_C = 0.1
SUSPECT_FACTOR = 3.0
SUSPECT_WINDOWS = 10
#: EW factors: residual variance, fan-direction running mean, prediction error.
RESIDUAL_ALPHA = 0.05
FAN_MEAN_ALPHA = 0.1
PRED_ERR_ALPHA = 0.1
#: Longest interval between two samples of a window, in ticks.
GAP_TICKS = 3.0
#: A closing window's sampled ``int w`` within this fraction of ``T / 2``, and its
#: sampled ``int dw`` / ``int d2w`` within this fraction of their scales, else dropped.
WINDOW_MASS_TOL = 0.05
WINDOW_SUM_TOL = 0.01
_EPS = 1e-12


# ---------------------------------------------------------------------------
# parameter table
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class ParamSpec:
    """One row of the parameter table (module docstring)."""

    unit: str
    lo: float
    hi: float
    prior: str
    identified_from: str
    #: scale of the scaled RLS coordinate when the prior is not a usable magnitude
    scale: float = 1.0
    #: initial variance in scaled units
    var0: float = 0.25

    def to_dict(self) -> dict[str, Any]:
        return {
            "unit": self.unit,
            "lo": self.lo,
            "hi": self.hi,
            "prior": self.prior,
            "identified_from": self.identified_from,
        }


#: Parameter kinds, keyed by the prefix of the parameter key (``E.z0.xt1`` -> ``E``).
PARAMETERS: dict[str, ParamSpec] = {
    "E": ParamSpec(
        "W/K",
        0.0,
        200.0,
        "33 W/K per fan x count / zones listing the channel; 0.1x for a coupled zone",
        "air-node RLS with fan excitation (weak cross-zone entries: ridge)",
    ),
    "Es": ParamSpec(
        "W/K",
        -200.0,
        200.0,
        "0 (the group's prior split, model_split_channels)",
        "air-node RLS, single-channel excitation only (ridge toward 0)",
        scale=E_W_PER_K_PER_FAN,
    ),
    "leak": ParamSpec("W/K", 0.0, 20.0, "1", "air-node RLS, ridge (weak)", var0=1.0),
    "kappa": ParamSpec("W/K", 0.0, 50.0, "3 for declared pairs", "air-node RLS, ridge", var0=1.0),
    "p_air": ParamSpec(
        "W", -100.0, 100.0, "0", "air-node RLS constant, ridge toward 0", scale=10.0, var0=1.0
    ),
    "g0": ParamSpec("W/K", 0.05, 5.0, "0.3", "proximal-sensor RLS, ridge (weak)"),
    "k": ParamSpec("W/K", 0.05, 5.0, "0.5", "proximal-sensor RLS with fan excitation"),
    "q_s": ParamSpec("K/s", -0.2, 0.2, "0", "proximal-sensor RLS constant", scale=0.01, var0=1.0),
    "c_air": ParamSpec("J/K", 50.0, 2000.0, "200", "prior only (not identifiable at this dt)"),
    "c_drive": ParamSpec("J/K", 50.0, 2000.0, "tau_d_s * (g0 + k)", "prior only (SMART, later)"),
    "beta": ParamSpec("-", 0.02, 0.7, "0.3", "SMART calibration (estimator)"),
    "b": ParamSpec("degC", -10.0, 10.0, "-2.1", "SMART calibration (estimator)"),
    "tau_s": ParamSpec(
        "s", 3.0, 120.0, "sensors.<name>.tau_s, else 5 (thermistor) / 15 (DS18B20)", "config"
    ),
    "u0": ParamSpec(
        "-", 0.0, 0.5, "fan_models.<m>.deadband", "config, tools/fit_fans.py, or fan_curve_online"
    ),
    "n": ParamSpec(
        "-", 0.5, 1.5, "fan_models.<m>.exponent", "config, tools/fit_fans.py, or fan_curve_online"
    ),
}

#: Random walk per window on the constants, scaled units.
_CONST_WALK = {"p_air": 0.02, "q_s": 0.1}
#: Kinds pulled toward the prior by the ridge (``Es`` toward 0: without single-channel
#: excitation a group keeps its prior split, which is the shared-``E`` model exactly).
_RIDGED = frozenset({"Es", "leak", "kappa", "g0", "p_air"})
#: Kinds that are constants (no trust region, random walk).
_CONSTANTS = frozenset({"p_air", "q_s"})
#: Row noise: initial RMS and floor of the a-priori residual (air W, proximal K).
_AIR_SIGMA0_W, _AIR_SIGMA_FLOOR_W = 1.0, 0.1
_PROX_SIGMA0_C, _PROX_SIGMA_FLOOR_C = 0.05, 0.005


def _kind(key: str) -> str:
    return key.split(".", 1)[0]


# ---------------------------------------------------------------------------
# structure derived from the config
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class Group:
    """One effectiveness coefficient ``E.<zone>.<group>``, with the per-channel split
    coefficients ``Es.<zone>.<group>.<channel>`` of ``model_split_channels``."""

    key: str
    zone: str
    name: str
    weak: bool
    weights: dict[str, float]  # channel -> prior W/K at phi = 1
    prior: float
    #: ``(key, channel)`` per split coefficient: every member but the reference channel
    #: (the first). Empty for a single-channel group and without the switch.
    splits: tuple[tuple[str, str], ...] = ()


@dataclass(frozen=True)
class ZoneStruct:
    name: str
    index: int
    bays: tuple[str, ...]
    air: tuple[str, ...]
    inlet: tuple[str, ...]
    coupled: tuple[str, ...]
    groups: tuple[Group, ...]
    air_keys: tuple[str, ...]  # [E..., leak, kappa..., p_air]
    tau_air: float  # mean lag of the zone-air sensors, s
    tau_inlet: float  # mean lag of the zone's inlet sensors, s


@dataclass(frozen=True)
class BayStruct:
    name: str
    zone: str
    index: int
    sensors: tuple[str, ...]
    tau_s: float
    keys: tuple[str, str, str]  # (q_s, g0, k)


@dataclass(frozen=True)
class Structure:
    """What the config fixes about the model: zones, bays, groups, keys, state layout."""

    channels: tuple[str, ...]
    zones: dict[str, ZoneStruct]
    bays: dict[str, BayStruct]
    fingerprint: str
    #: The same structure with ``model_split_channels`` the other way round: its air keys
    #: per zone and its fingerprint, so :func:`restore` can convert a store file written
    #: before the switch was turned on (or off) instead of dropping the model.
    alt_air_keys: dict[str, tuple[str, ...]] = field(default_factory=dict)
    alt_fingerprint: str = ""

    # cached: the index helpers below run in the per-tick loops of the model
    @cached_property
    def n_zones(self) -> int:
        return len(self.zones)

    @cached_property
    def n_bays(self) -> int:
        return len(self.bays)

    @cached_property
    def n_states(self) -> int:
        return self.n_zones + 2 * self.n_bays

    def i_air(self, zone: str) -> int:
        return self.zones[zone].index

    def i_drive(self, bay: str) -> int:
        return self.n_zones + self.bays[bay].index

    def i_sensor(self, bay: str) -> int:
        return self.n_zones + self.n_bays + self.bays[bay].index


def sensor_lag_s(cfg: MpcConfig, name: str) -> float:
    """First-order lag of a sensor: ``sensors.<name>.tau_s``, else by type from ``quant_c``
    (a thermistor, ``quant_c <= 0.02``: :data:`LAG_THERMISTOR_S`; else a DS18B20:
    :data:`LAG_DS18B20_S`)."""
    spec = cfg.sensors[name]
    if spec.tau_s:
        return float(spec.tau_s)
    return LAG_THERMISTOR_S if spec.quant_c <= THERMISTOR_MAX_QUANT_C else LAG_DS18B20_S


def structure(cfg: MpcConfig) -> Structure:
    """The model structure of a zoned config (module docstring). Raises ``ValueError``
    for a legacy config."""
    topo = cfg.topology
    if topo is None:
        raise ValueError("the thermal model needs a zoned config (mpc.topology)")
    sensors = cfg.sensors
    channels = tuple(cfg.channels)
    listed = {ch: [z for z, spec in topo.zones.items() if ch in spec.channels] for ch in channels}
    group_of = {ch: (cfg.fans[ch].group or ch) for ch in channels}
    group_names = list(dict.fromkeys(group_of[ch] for ch in channels))
    inlets = tuple(t for t in cfg.temps if sensors[t].role == "inlet")
    zones: dict[str, ZoneStruct] = {}
    bays: dict[str, BayStruct] = {}
    bay_index = 0
    for zi, (z, spec) in enumerate(topo.zones.items()):
        groups: list[Group] = []
        for g in group_names:
            weights: dict[str, float] = {}
            strong = False
            for ch in channels:
                if group_of[ch] != g:
                    continue
                base = E_W_PER_K_PER_FAN * cfg.fans[ch].count / len(listed[ch])
                if z in listed[ch]:
                    weights[ch] = base
                    strong = True
                elif any(z in topo.zones[other].coupled_to for other in listed[ch]):
                    weights[ch] = WEAK_E_FACTOR * base
            if weights:
                members = [ch for ch in channels if ch in weights]
                splits = (
                    tuple((f"Es.{z}.{g}.{ch}", ch) for ch in members[1:])
                    if cfg.model_split_channels and len(members) > 1
                    else ()
                )
                groups.append(
                    Group(
                        key=f"E.{z}.{g}",
                        zone=z,
                        name=g,
                        weak=not strong,
                        weights=weights,
                        prior=sum(weights.values()),
                        splits=splits,
                    )
                )
        zone_bays = tuple(b for b, bay in topo.bays.items() if bay.zone == z)
        for b in zone_bays:
            members = tuple(
                t for t in cfg.temps if sensors[t].role == "drive_proximal" and sensors[t].bay == b
            )
            primary = next((t for t in members if not sensors[t].redundant), members[0])
            bays[b] = BayStruct(
                name=b,
                zone=z,
                index=bay_index,
                sensors=members,
                tau_s=sensor_lag_s(cfg, primary),
                keys=(f"q_s.{b}", f"g0.{b}", f"k.{b}"),
            )
            bay_index += 1
        air_keys = (
            *(gr.key for gr in groups),
            *(key for gr in groups for key, _ in gr.splits),
            f"leak.{z}",
            *(f"kappa.{z}.{o}" for o in spec.coupled_to),
            f"p_air.{z}",
        )
        air = tuple(t for t in cfg.temps if sensors[t].role == "zone_air" and sensors[t].zone == z)
        inlet = inlets if spec.inlet == "mix" else (spec.inlet,)
        zones[z] = ZoneStruct(
            name=z,
            index=zi,
            bays=zone_bays,
            air=air,
            inlet=inlet,
            coupled=tuple(spec.coupled_to),
            groups=tuple(groups),
            air_keys=air_keys,
            tau_air=sum(sensor_lag_s(cfg, t) for t in air) / len(air),
            tau_inlet=sum(sensor_lag_s(cfg, t) for t in inlet) / len(inlet),
        )

    def fp(air_keys: Mapping[str, tuple[str, ...]]) -> str:
        return json.dumps(
            [list(channels)]
            + [[z, list(zs.bays), list(zs.air), list(air_keys[z])] for z, zs in zones.items()]
            + [[b, list(bs.sensors)] for b, bs in bays.items()],
            separators=(",", ":"),
        )

    # the same structure with model_split_channels the other way round, so restore() can
    # convert a file written before the switch was flipped instead of dropping the model
    alt_air_keys = {
        z: _alt_air_keys(zs, split=not cfg.model_split_channels) for z, zs in zones.items()
    }
    return Structure(
        channels=channels,
        zones=zones,
        bays=bays,
        fingerprint=fp({z: zs.air_keys for z, zs in zones.items()}),
        alt_air_keys=alt_air_keys,
        alt_fingerprint=fp(alt_air_keys),
    )


def _alt_air_keys(zone: ZoneStruct, *, split: bool) -> tuple[str, ...]:
    """A zone's air keys with the per-channel split coefficients present or absent."""
    without = tuple(k for k in zone.air_keys if _kind(k) != "Es")
    if not split:
        return without
    groups = tuple(gr.key for gr in zone.groups)
    extra = tuple(f"Es.{zone.name}.{gr.name}.{ch}" for gr in zone.groups for ch in _members(gr)[1:])
    return (*groups, *extra, *without[len(groups) :])


def _members(gr: Group) -> tuple[str, ...]:
    """The group's channels in this zone, in the structure's channel order."""
    return tuple(gr.weights)


def parameter_keys(st: Structure) -> tuple[str, ...]:
    """Every identified parameter key, zone air blocks first, then bays."""
    keys: list[str] = []
    for zone in st.zones.values():
        keys.extend(zone.air_keys)
    for bay in st.bays.values():
        keys.extend(bay.keys)
    return tuple(keys)


def prior_theta(cfg: MpcConfig, st: Structure | None = None) -> dict[str, float]:
    """Prior value of every identified parameter (the table in the module docstring)."""
    st = structure(cfg) if st is None else st
    out: dict[str, float] = {}
    for zone in st.zones.values():
        for gr in zone.groups:
            out[gr.key] = gr.prior
            for key, _ in gr.splits:
                out[key] = 0.0
        out[f"leak.{zone.name}"] = LEAK_W_PER_K
        for other in zone.coupled:
            out[f"kappa.{zone.name}.{other}"] = KAPPA_W_PER_K
        out[f"p_air.{zone.name}"] = 0.0
    for bay in st.bays.values():
        q_key, g0_key, k_key = bay.keys
        out[q_key] = 0.0
        out[g0_key] = G0_W_PER_K
        out[k_key] = K_W_PER_K
    return out


def theta_from_memory(
    cfg: MpcConfig, memory: Mapping[str, Any], *, st: Structure | None = None
) -> dict[str, float]:
    """Every identified parameter's current value: the prior, overridden by whatever
    ``memory`` (a live ``solver_memory["thermal"]`` or a loaded ``model.json``) holds
    for each zone air block and bay block -- the same merge :func:`update` does
    internally before it linearises. Raises the same way :func:`structure` does on a
    legacy config; a malformed ``memory`` (wrong shape, missing keys) raises
    ``KeyError``/``ValueError``/``TypeError``, same as any other malformed input here."""
    st = structure(cfg) if st is None else st
    theta = prior_theta(cfg, st)
    for z, zone in st.zones.items():
        theta.update(zip(zone.air_keys, memory["zones"][z]["air"]["theta"], strict=True))
    for b, bay in st.bays.items():
        theta.update(zip(bay.keys, memory["bays"][b]["theta"], strict=True))
    return theta


def project(theta: Mapping[str, float]) -> dict[str, float]:
    """``theta`` clamped into the bounds of :data:`PARAMETERS`."""
    out: dict[str, float] = {}
    for key, value in theta.items():
        spec = PARAMETERS[_kind(key)]
        out[key] = min(spec.hi, max(spec.lo, float(value)))
    return out


# ---------------------------------------------------------------------------
# model parameters and airflow
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class ThermalParams:
    """Everything the continuous model needs besides the operating point.

    ``theta`` holds the identified keys (:func:`prior_theta` layout); the rest are
    priors or come from the estimator (sensor map, occupancy, class).
    """

    theta: dict[str, float]
    c_air: dict[str, float]
    c_drive: dict[str, float]
    s: dict[str, float]
    b: dict[str, float]
    tau_s: dict[str, float]
    occupied: dict[str, bool]
    fan: dict[str, tuple[float, float]]  # channel -> (deadband, exponent)


def model_params(
    cfg: MpcConfig,
    theta: Mapping[str, float] | None = None,
    *,
    st: Structure | None = None,
    occupancy: Mapping[str, str] | None = None,
    maps: Mapping[str, tuple[float, float]] | None = None,
    classes: Mapping[str, str] | None = None,
    curves: Mapping[str, Any] | None = None,
) -> ThermalParams:
    """:class:`ThermalParams` from identified ``theta`` (default: the prior), the
    estimator's occupancy (``empty`` freezes the drive; default: declared), sensor
    maps ``(s, b)`` (default, or not a finite slope in ``(0, 1]`` with a finite offset:
    the prior map) and drive classes (default: declared).

    ``curves`` is ``solver_memory["fan_curves"]`` (the model store's section, produced
    online by :mod:`aqua_bridge.control.fancurve` with ``fan_curve_online``): a usable
    entry replaces its fan model's ``deadband`` / ``exponent`` for the ``u0.<m>`` /
    ``n.<m>`` rows of the table. ``None``, a missing entry or a malformed one keeps the
    configured curve (:func:`aqua_bridge.control.fancurve.curve_pair`).
    """
    d = _derived(cfg)
    st = d.st if st is None else st
    base = dict(d.prior) if st is d.st else prior_theta(cfg, st)
    if theta is not None:
        base.update({k: float(v) for k, v in theta.items() if k in base})
    topo = cfg.topology
    assert topo is not None
    occupied: dict[str, bool] = {}
    c_drive: dict[str, float] = {}
    s: dict[str, float] = {}
    b: dict[str, float] = {}
    for bay in st.bays:
        occ = None if occupancy is None else occupancy.get(bay)
        occupied[bay] = (occ != EMPTY) if occ is not None else topo.bays[bay].constrained
        cls = (classes or {}).get(bay) or cfg.bay_class(bay)
        c_drive[bay] = drive_capacity(cfg, cls)
        s[bay], b[bay] = _sensor_map((maps or {}).get(bay))
    fan = {}
    for ch in st.channels:
        name = cfg.fans[ch].model
        model = cfg.fan_models[name]
        if curves is None:
            fan[ch] = (float(model.deadband), float(model.exponent))
        else:
            deadband, exponent, _ = fancurve.curve_pair(
                curves.get(name), model.deadband, model.exponent, model.rpm_max
            )
            fan[ch] = (deadband, exponent)
    return ThermalParams(
        theta=base,
        c_air=dict.fromkeys(st.zones, C_AIR_J_PER_K),
        c_drive=c_drive,
        s=s,
        b=b,
        tau_s={bay: bs.tau_s for bay, bs in st.bays.items()},
        occupied=occupied,
        fan=fan,
    )


def _sensor_map(raw: object) -> tuple[float, float]:
    """A bay's sensor map ``(s, b)``: a finite slope in ``(0, 1]`` and a finite offset, else
    the prior map (a non-finite or zero slope would put NaN into the model and the state)."""
    prior = (1.0 - PRIOR_BETA, PRIOR_OFFSET_C)
    if not isinstance(raw, tuple | list) or len(raw) != 2:
        return prior
    s_map, b_map = raw
    if not (_finite(s_map) and _finite(b_map) and 0.0 < float(s_map) <= 1.0):
        return prior
    return float(s_map), float(b_map)


def phi(u: float, deadband: float, exponent: float) -> float:
    """Relative airflow of a channel at PWM ``u`` (module docstring)."""
    frac = min(1.0, max(0.0, (float(u) - deadband) / (1.0 - deadband)))
    return frac**exponent if frac > 0 else 0.0


def dphi(u: float, deadband: float, exponent: float) -> float:
    """One-sided derivative of :func:`phi` (0 outside the open linear range)."""
    frac = (float(u) - deadband) / (1.0 - deadband)
    if not 0.0 < frac < 1.0:
        return 0.0
    return exponent * max(frac, 1e-6) ** (exponent - 1.0) / (1.0 - deadband)


def split_scale(gr: Group, theta: Mapping[str, float]) -> float:
    """The largest factor in ``[0, 1]`` on the group's split coefficients that leaves
    every per-channel effectiveness (:func:`channel_effectiveness`) at or above zero.

    A fan never cools less than nothing, and ``Q_zG = sum_ch E_ch phi_ch`` exactly
    (module docstring, *Split*), so a split with every ``E_ch >= 0`` can never reverse
    the zone's modelled airflow. Scaling the whole split toward the prior split is the
    projection that keeps its direction and its total: at 1 nothing moves, at 0 the
    group is back at the shared-``E`` model. 1 whenever the split is already feasible,
    so the ordinary case costs one pass and changes nothing.
    """
    total = gr.prior
    if not gr.splits or total <= 0:
        return 1.0
    e = float(theta[gr.key])
    values = {ch: float(theta[key]) for key, ch in gr.splits}
    net = sum(values.values())
    alpha = 1.0
    for ch, w in gr.weights.items():
        share = w / total
        base = e * share  # >= 0: E is bounded below by 0 and the weights are positive
        delta = values.get(ch, 0.0) - share * net
        if delta < 0.0 and base + delta < 0.0:
            alpha = min(alpha, base / -delta)
    return max(0.0, alpha)


def _project_split(theta: list[float], zone: ZoneStruct, keys: tuple[str, ...]) -> None:
    """Project one zone's air-block ``theta`` (in ``keys`` order, mutated in place) so no
    group's split makes a channel's effectiveness negative (:func:`split_scale`)."""
    if not any(gr.splits for gr in zone.groups):
        return
    index = {key: i for i, key in enumerate(keys)}
    values = {key: float(theta[i]) for key, i in index.items()}
    for gr in zone.groups:
        alpha = split_scale(gr, values)
        if alpha < 1.0:
            for key, _ in gr.splits:
                theta[index[key]] = values[key] * alpha


def _group_phi(gr: Group, phis: Mapping[str, float]) -> float:
    return sum(w * phis[ch] for ch, w in gr.weights.items()) / gr.prior if gr.prior > 0 else 0.0


def channel_effectiveness(gr: Group, theta: Mapping[str, float]) -> dict[str, float]:
    """The group's effectiveness split over its channels, W/K (module docstring, *Split*).

    ``E_ch = E_G w_ch / W + dE_ch - (w_ch / W) sum dE``, with ``dE`` of the reference
    channel fixed at 0 -- so the split redistributes the group's coefficient and never
    changes its total, and every ``dE`` at its prior of 0 gives exactly the shared-``E``
    model's ``E_G w_ch / W``.
    """
    total = gr.prior
    e = float(theta[gr.key])
    if total <= 0:
        return dict.fromkeys(gr.weights, 0.0)
    shares = {ch: w / total for ch, w in gr.weights.items()}
    out = {ch: e * share for ch, share in shares.items()}
    if not gr.splits:
        return out
    net = 0.0
    for key, ch in gr.splits:
        value = float(theta[key])
        out[ch] += value
        net += value
    for ch, share in shares.items():
        out[ch] -= share * net
    return out


def _vec_u(st: Structure, u: Mapping[str, float] | np.ndarray | Any) -> np.ndarray:
    if isinstance(u, Mapping):
        return np.array([float(u[ch]) for ch in st.channels])
    arr = np.asarray(u, dtype=float)
    if arr.shape != (len(st.channels),):
        raise ValueError(f"u must have {len(st.channels)} entries, got shape {arr.shape}")
    return arr


def _airflow(
    st: Structure, p: ThermalParams, u: np.ndarray
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """``(Q, Qn, dQ/du, E_total)`` per zone at the command vector ``u``."""
    phis = {ch: phi(u[i], *p.fan[ch]) for i, ch in enumerate(st.channels)}
    dphis = {ch: dphi(u[i], *p.fan[ch]) for i, ch in enumerate(st.channels)}
    n_z, n_u = st.n_zones, len(st.channels)
    q = np.zeros(n_z)
    dq = np.zeros((n_z, n_u))
    total = np.zeros(n_z)
    col = {ch: i for i, ch in enumerate(st.channels)}
    for zone in st.zones.values():
        zi = zone.index
        split = False
        for gr in zone.groups:
            e = p.theta[gr.key]
            total[zi] += e
            if gr.prior <= 0:
                continue
            q[zi] += e * _group_phi(gr, phis)
            for ch, w in gr.weights.items():
                dq[zi, col[ch]] += e * w * dphis[ch] / gr.prior
            if not gr.splits:
                continue
            # the split terms vanish exactly when the group's channels move together,
            # so a group at its prior split is bit-identical to the shared-E model
            split = True
            group_phi = _group_phi(gr, phis)
            for key, ch in gr.splits:
                value = p.theta[key]
                q[zi] += value * (phis[ch] - group_phi)
                dq[zi, col[ch]] += value * dphis[ch]
                for other, w in gr.weights.items():
                    dq[zi, col[other]] -= value * w * dphis[other] / gr.prior
        # Q_zG = sum_ch E_ch phi_ch exactly, and the split is projected so no E_ch is
        # negative (:func:`split_scale`), so this guard is unreachable by construction;
        # it is kept for a theta handed in directly. The Jacobian stays as computed: a
        # zero row would tell the optimiser that no fan in the zone moves any air.
        if split and q[zi] < 0.0:
            q[zi] = 0.0
    qn = np.divide(q, total, out=np.zeros(n_z), where=total > _EPS)
    dqn = np.divide(dq, total[:, None], out=np.zeros_like(dq), where=total[:, None] > _EPS)
    return q, qn, dq, dqn


# ---------------------------------------------------------------------------
# continuous model, Jacobians, discretisation
# ---------------------------------------------------------------------------


def _inputs(st: Structure, t_in: Any, q: Any, d_air: Any) -> tuple[np.ndarray, ...]:
    def vec(value: Any, names: Collection[str], default: float) -> np.ndarray:
        if value is None:
            return np.full(len(names), default)
        if isinstance(value, Mapping):
            return np.array([float(value.get(n, default)) for n in names])
        arr = np.asarray(value, dtype=float)
        if arr.shape != (len(names),):
            raise ValueError(f"expected {len(names)} values, got shape {arr.shape}")
        return arr

    return vec(t_in, st.zones, 0.0), vec(q, st.bays, 0.0), vec(d_air, st.zones, 0.0)


def derivatives(
    st: Structure,
    p: ThermalParams,
    x: np.ndarray,
    u: Mapping[str, float] | np.ndarray,
    *,
    t_in: Any,
    q: Any = None,
    d_air: Any = None,
) -> np.ndarray:
    """``dx/dt`` of the zoned network (module docstring) at state ``x`` and command ``u``.

    ``t_in`` per zone (degC), ``q`` per bay (K/s), ``d_air`` per zone (K/s): mappings
    by name or arrays in structure order.
    """
    x = np.asarray(x, dtype=float)
    uv = _vec_u(st, u)
    tin, qv, dv = _inputs(st, t_in, q, d_air)
    flow, qn, _, _ = _airflow(st, p, uv)
    th = p.theta
    f = np.zeros(st.n_states)
    for zone in st.zones.values():
        z, zi = zone.name, zone.index
        ta = x[zi]
        acc = -(flow[zi] + th[f"leak.{z}"]) * (ta - tin[zi]) + th[f"p_air.{z}"]
        for other in zone.coupled:
            acc += th[f"kappa.{z}.{other}"] * (x[st.i_air(other)] - ta)
        for bay in zone.bays:
            if p.occupied[bay]:
                g = th[f"g0.{bay}"] + th[f"k.{bay}"] * qn[zi]
                acc += g * (x[st.i_drive(bay)] - ta)
        f[zi] = acc / p.c_air[z] + dv[zi]
    for bay, bs in st.bays.items():
        zi = st.zones[bs.zone].index
        ta, td, ts = x[zi], x[st.i_drive(bay)], x[st.i_sensor(bay)]
        tau = p.tau_s[bay]
        if p.occupied[bay]:
            g = th[f"g0.{bay}"] + th[f"k.{bay}"] * qn[zi]
            f[st.i_drive(bay)] = qv[bs.index] - g * (td - ta) / p.c_drive[bay]
            f[st.i_sensor(bay)] = (p.s[bay] * td + (1.0 - p.s[bay]) * ta + p.b[bay] - ts) / tau
        else:
            f[st.i_sensor(bay)] = (ta - ts) / tau
    return f


@dataclass(frozen=True)
class Linearisation:
    """``dx/dt ~ A x + B u + c`` around ``(x0, u0)``; ``f`` is ``dx/dt`` at the point."""

    a: np.ndarray
    b: np.ndarray
    c: np.ndarray
    f: np.ndarray


def state_jacobian(
    st: Structure, p: ThermalParams, u: Mapping[str, float] | np.ndarray
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """``(A, dQ/du, dQn/du)`` at command ``u``, from one airflow evaluation.

    ``A = df/dx`` does not depend on the state (every entry is built from the
    parameters and the airflow alone), and ``dQ/du`` / ``dQn/du`` are the pieces
    of :func:`jacobians`' input matrix that do not depend on the state either, so
    the DAS MPC's linearisation
    (:func:`aqua_bridge.control.solver_das._dynamics`) gets everything it needs
    from this one call instead of a :func:`jacobians` whose input matrix,
    derivatives and affine term it discards (item 73). Same ``A`` as
    :func:`jacobians` builds, entry for entry.
    """
    uv = _vec_u(st, u)
    flow, qn, dq, dqn = _airflow(st, p, uv)
    return _jacobian_a(st, p, flow, qn), dq, dqn


def _jacobian_a(st: Structure, p: ThermalParams, flow: np.ndarray, qn: np.ndarray) -> np.ndarray:
    """``df/dx`` of the zoned network at an airflow (state-independent)."""
    th = p.theta
    a = np.zeros((st.n_states, st.n_states))
    for zone in st.zones.values():
        z, zi = zone.name, zone.index
        ca = p.c_air[z]
        a[zi, zi] -= (flow[zi] + th[f"leak.{z}"]) / ca
        for other in zone.coupled:
            kap = th[f"kappa.{z}.{other}"]
            a[zi, zi] -= kap / ca
            a[zi, st.i_air(other)] += kap / ca
        for bay in zone.bays:
            if not p.occupied[bay]:
                continue
            g = th[f"g0.{bay}"] + th[f"k.{bay}"] * qn[zi]
            a[zi, zi] -= g / ca
            a[zi, st.i_drive(bay)] += g / ca
    for bay, bs in st.bays.items():
        zi = st.zones[bs.zone].index
        di, si = st.i_drive(bay), st.i_sensor(bay)
        tau = p.tau_s[bay]
        a[si, si] = -1.0 / tau
        if p.occupied[bay]:
            g = th[f"g0.{bay}"] + th[f"k.{bay}"] * qn[zi]
            cd = p.c_drive[bay]
            a[di, di] = -g / cd
            a[di, zi] = g / cd
            a[si, di] = p.s[bay] / tau
            a[si, zi] = (1.0 - p.s[bay]) / tau
        else:
            a[si, zi] = 1.0 / tau
    return a


def jacobians(
    st: Structure,
    p: ThermalParams,
    x: np.ndarray,
    u: Mapping[str, float] | np.ndarray,
    *,
    t_in: Any,
    q: Any = None,
    d_air: Any = None,
) -> Linearisation:
    """Analytic continuous Jacobians and affine term (module docstring)."""
    x = np.asarray(x, dtype=float)
    uv = _vec_u(st, u)
    tin, qv, dv = _inputs(st, t_in, q, d_air)
    flow, qn, dq, dqn = _airflow(st, p, uv)
    th = p.theta
    a = _jacobian_a(st, p, flow, qn)
    b = np.zeros((st.n_states, len(st.channels)))
    for zone in st.zones.values():
        z, zi = zone.name, zone.index
        ca = p.c_air[z]
        ta = x[zi]
        b[zi] -= dq[zi] * (ta - tin[zi]) / ca
        for bay in zone.bays:
            if not p.occupied[bay]:
                continue
            b[zi] += th[f"k.{bay}"] * dqn[zi] * (x[st.i_drive(bay)] - ta) / ca
    for bay, bs in st.bays.items():
        if p.occupied[bay]:
            zi = st.zones[bs.zone].index
            di = st.i_drive(bay)
            b[di] = -th[f"k.{bay}"] * dqn[zi] * (x[di] - x[zi]) / p.c_drive[bay]
    f = derivatives(st, p, x, uv, t_in=tin, q=qv, d_air=dv)
    c = f - a @ x - b @ uv
    return Linearisation(a=a, b=b, c=c, f=f)


@dataclass(frozen=True)
class Discretisation:
    """``x+ = ad x + bd u + cd`` over one step ``h``; ``method`` ``eig`` | ``euler``."""

    ad: np.ndarray
    bd: np.ndarray
    cd: np.ndarray
    method: str
    substeps: int = 1


def discretise(a: np.ndarray, b: np.ndarray, c: np.ndarray, h: float) -> Discretisation:
    """Exact ZOH discretisation by eigendecomposition, Euler fallback (module docstring)."""
    a = np.asarray(a, dtype=float)
    b = np.asarray(b, dtype=float)
    c = np.asarray(c, dtype=float)
    n = a.shape[0]
    if not h > 0:
        raise ValueError(f"h must be > 0, got {h}")
    if n == 0:
        return Discretisation(np.zeros((0, 0)), b.copy(), c.copy(), "eig")
    try:
        w, v = np.linalg.eig(a)
        imag_ok = bool(np.all(np.abs(w.imag) <= EIG_IMAG_TOL * np.maximum(1.0, np.abs(w.real))))
        cond_ok = bool(np.all(np.isfinite(v))) and float(np.linalg.cond(v)) <= EIG_COND_MAX
    except np.linalg.LinAlgError:
        imag_ok = cond_ok = False
    if imag_ok and cond_ok:
        try:
            # repeated eigenvalues (identical bays) can come back as a conjugate pair
            # with ~0 imaginary parts whose eigenvectors' real parts are singular
            vi = np.linalg.inv(v.real)
        except np.linalg.LinAlgError:
            imag_ok = False
    if imag_ok and cond_ok:
        lam = w.real
        vr = v.real
        ex = np.exp(lam * h)
        small = np.abs(lam * h) < 1e-8
        safe = np.where(small, 1.0, lam)
        ig = np.where(small, h * (1.0 + 0.5 * lam * h), (ex - 1.0) / safe)
        ad = (vr * ex) @ vi
        m_int = (vr * ig) @ vi
        out = Discretisation(ad=ad, bd=m_int @ b, cd=m_int @ c, method="eig")
        if all(np.all(np.isfinite(arr)) for arr in (out.ad, out.bd, out.cd)):
            return out
    norm = float(np.abs(a).sum(axis=1).max())
    substeps = max(EULER_MIN_SUBSTEPS, math.ceil(h * norm / EULER_STEP_NORM))
    hs = h / substeps
    step = np.eye(n) + hs * a
    ad = np.eye(n)
    bd = np.zeros_like(b)
    cd = np.zeros_like(c)
    for _ in range(substeps):
        ad = step @ ad
        bd = step @ bd + hs * b
        cd = step @ cd + hs * c
    return Discretisation(ad=ad, bd=bd, cd=cd, method="euler", substeps=substeps)


# ---------------------------------------------------------------------------
# identification: blocks
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class _BlockSpec:
    keys: tuple[str, ...]
    scale: np.ndarray
    lo: np.ndarray
    hi: np.ndarray
    prior: np.ndarray
    var0: np.ndarray
    ridge: np.ndarray  # bool
    const: np.ndarray  # bool
    gain: tuple[int, ...]  # indices whose rel. SE decides convergence
    sigma0: float
    sigma_floor: float


def _block_spec(
    keys: tuple[str, ...], prior: Mapping[str, float], weak: Collection[str]
) -> _BlockSpec:
    n = len(keys)
    scale = np.ones(n)
    lo = np.zeros(n)
    hi = np.zeros(n)
    pr = np.zeros(n)
    var0 = np.zeros(n)
    ridge = np.zeros(n, dtype=bool)
    const = np.zeros(n, dtype=bool)
    gain: list[int] = []
    for i, key in enumerate(keys):
        kind = _kind(key)
        spec = PARAMETERS[kind]
        pr[i] = prior[key]
        scale[i] = abs(pr[i]) if abs(pr[i]) > _EPS else spec.scale
        lo[i], hi[i] = spec.lo, spec.hi
        var0[i] = spec.var0
        const[i] = kind in _CONSTANTS
        ridge[i] = kind in _RIDGED or key in weak
        if key in weak:
            var0[i] = 1.0
        if (kind == "E" and key not in weak) or kind == "k":
            gain.append(i)
    is_air = _kind(keys[-1]) == "p_air"
    return _BlockSpec(
        keys=keys,
        scale=scale,
        lo=lo,
        hi=hi,
        prior=pr,
        var0=var0,
        ridge=ridge,
        const=const,
        gain=tuple(gain),
        sigma0=_AIR_SIGMA0_W if is_air else _PROX_SIGMA0_C,
        sigma_floor=_AIR_SIGMA_FLOOR_W if is_air else _PROX_SIGMA_FLOOR_C,
    )


def _fresh_block(spec: _BlockSpec, n_fan: int) -> dict[str, Any]:
    return {
        "theta": spec.prior.tolist(),
        "P": np.diag(spec.var0).tolist(),
        "s2": spec.sigma0**2,
        "n": 0,
        "w": 0,
        "m": [0.0] * n_fan,
        "S": np.zeros((n_fan, n_fan)).tolist(),
        "fm": None,
        "pe": 0.0,
        "rel": None,
        "acc": None,
    }


def _rls_window(
    block: dict[str, Any],
    spec: _BlockSpec,
    x: np.ndarray,
    y: float,
    fan: np.ndarray,
    cfg: MpcConfig,
    *,
    learn: bool = True,
) -> tuple[float, bool]:
    """One window of the constrained RLS (module docstring). Returns the a-priori
    residual (row units) and whether the window was excited. Mutates ``block``.

    ``learn=False`` (:func:`update`'s hold-out mode) still advances every piece of
    per-window bookkeeping (the residual, the excitation and PE-monitor state, the
    window counters) but leaves ``theta``/``P`` exactly as given, so the returned
    residual is a genuine a-priori prediction from parameters the window played no
    part in fitting -- a windowed replay of the same recording with ``learn=True``
    fits and predicts on the same data, which is not a hold-out check.
    """
    theta = np.array(block["theta"], dtype=float)
    p = np.array(block["P"], dtype=float)
    psi = theta / spec.scale
    sigma = math.sqrt(max(float(block["s2"]), spec.sigma_floor**2))
    xs = x * spec.scale / sigma
    residual = float(y - x @ theta)
    r_clip = max(-HUBER_SIGMAS * sigma, min(HUBER_SIGMAS * sigma, residual)) / sigma

    fm = block["fm"]
    if fm is None:
        excited = False
        fm_arr = fan.copy()
    else:
        fm_arr = np.array(fm, dtype=float)
        rel = np.abs(fan - fm_arr) / np.maximum(np.abs(fm_arr), 0.05)
        excited = bool(rel.size and float(rel.max()) > EXCITATION_MIN)
        fm_arr = (1.0 - FAN_MEAN_ALPHA) * fm_arr + FAN_MEAN_ALPHA * fan

    if learn:
        for i in np.flatnonzero(spec.const):
            p[i, i] += _CONST_WALK[_kind(spec.keys[i])] ** 2

        before = psi.copy()
        eye = np.eye(len(psi))
        if excited:
            p = p / cfg.model_lambda
            psi, p = _joseph(psi, p, xs, r_clip, 1.0, eye)
            for i in np.flatnonzero(spec.ridge):
                psi, p = _joseph(
                    psi, p, eye[i], spec.prior[i] / spec.scale[i] - psi[i], RIDGE_VAR, eye
                )
        else:
            # Constant only (Schmidt "consider" update): the full row, a gain on the
            # constant alone, Joseph form so P stays positive semi-definite.
            mask = spec.const.astype(float)
            psi, p = _joseph(psi, p, xs, r_clip, 1.0, eye, mask=mask)
        p = 0.5 * (p + p.T)
        trace = float(np.trace(p))
        if trace > cfg.model_p_trace_max:
            p *= cfg.model_p_trace_max / trace
        step = psi - before
        limit = TRUST_REGION * np.maximum(np.abs(before), 1.0)
        step = np.where(spec.const, step, np.clip(step, -limit, limit))
        theta = np.clip((before + step) * spec.scale, spec.lo, spec.hi)
        if not (np.all(np.isfinite(theta)) and np.all(np.isfinite(p))):
            raise FloatingPointError(f"thermal: non-finite RLS state in {spec.keys[-1]!r}")
        block["theta"] = theta.tolist()
        block["P"] = p.tolist()

    block["s2"] = (1.0 - RESIDUAL_ALPHA) * float(block["s2"]) + RESIDUAL_ALPHA * min(
        residual**2, (HUBER_SIGMAS * sigma) ** 2
    )
    block["w"] = int(block["w"]) + 1
    if excited:
        block["n"] = int(block["n"]) + 1
    block["fm"] = fm_arr.tolist()
    # PE monitor over the fan-direction regressors (module docstring)
    beta = 1.0 / PE_WINDOWS
    m_arr = (1.0 - beta) * np.array(block["m"], dtype=float) + beta * fan
    s_arr = (1.0 - beta) * np.array(block["S"], dtype=float) + beta * np.outer(fan, fan)
    block["m"] = m_arr.tolist()
    block["S"] = s_arr.tolist()
    block["pe"] = _pe_min(m_arr, s_arr, int(block["w"]))
    block["rel"] = _rel_se(block, spec)
    return residual, excited


def _joseph(
    psi: np.ndarray,
    p: np.ndarray,
    row: np.ndarray,
    residual: float,
    var: float,
    eye: np.ndarray,
    *,
    mask: np.ndarray | None = None,
) -> tuple[np.ndarray, np.ndarray]:
    """Scalar measurement ``residual = row . (psi_true - psi)`` with variance ``var``.

    Joseph form, valid for any gain; ``mask`` zeroes the gain outside the coordinates
    that may move (the others are only *considered*).
    """
    pr = p @ row
    s = float(row @ pr) + var
    if not s > 0:
        return psi, p
    k = pr / s
    if mask is not None:
        k = k * mask
    ikh = eye - np.outer(k, row)
    return psi + k * residual, ikh @ p @ ikh.T + var * np.outer(k, k)


def _pe_min(mean: np.ndarray, second: np.ndarray, windows: int) -> float:
    if mean.size == 0 or windows < 2:
        return 0.0
    # undo the start-up bias of the exponential weights
    bias = 1.0 - (1.0 - 1.0 / PE_WINDOWS) ** windows
    m = mean / bias
    s = second / bias
    cov = s - np.outer(m, m)
    scale = np.maximum(np.abs(m), 0.05)
    norm = cov / np.outer(scale, scale)
    vals = np.linalg.eigvalsh(0.5 * (norm + norm.T))
    return float(min(1.0, max(0.0, vals[0])))


def _rel_se(block: Mapping[str, Any], spec: _BlockSpec) -> list[float | None]:
    """Relative standard error per coefficient (rows are normalised by the residual
    RMS, so ``P`` is the covariance in scaled units); ``None`` for a zero value."""
    theta = np.array(block["theta"], dtype=float)
    p = np.array(block["P"], dtype=float)
    out: list[float | None] = []
    for i in range(len(spec.keys)):
        se = spec.scale[i] * math.sqrt(max(float(p[i, i]), 0.0))
        value = abs(float(theta[i]))
        out.append(None if value < 1e-9 else se / value)
    return out


# ---------------------------------------------------------------------------
# memory
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class _Derived:
    """Everything :func:`update` derives from the config alone."""

    st: Structure
    prior: dict[str, float]
    zone_specs: dict[str, _BlockSpec]
    bay_specs: dict[str, _BlockSpec]


#: Config-derived structure, keyed by config identity (``MpcConfig`` is frozen and the
#: loop keeps one effective config between intents). A pure memo: same result as
#: recomputing, only cheaper per tick.
_DERIVED_CACHE: dict[int, tuple[MpcConfig, _Derived]] = {}
_DERIVED_CACHE_MAX = 16


def _derived(cfg: MpcConfig) -> _Derived:
    hit = _DERIVED_CACHE.get(id(cfg))
    if hit is not None and hit[0] is cfg:
        return hit[1]
    st = structure(cfg)
    prior = prior_theta(cfg, st)
    derived = _Derived(
        st=st,
        prior=prior,
        zone_specs={
            z: _block_spec(zone.air_keys, prior, {gr.key for gr in zone.groups if gr.weak})
            for z, zone in st.zones.items()
        },
        bay_specs={b: _block_spec(bay.keys, prior, ()) for b, bay in st.bays.items()},
    )
    while len(_DERIVED_CACHE) >= _DERIVED_CACHE_MAX:
        del _DERIVED_CACHE[next(iter(_DERIVED_CACHE))]
    _DERIVED_CACHE[id(cfg)] = (cfg, derived)
    return derived


def _n_fan_air(zone: ZoneStruct) -> int:
    return sum(1 for gr in zone.groups if not gr.weak)


def fresh_memory(
    cfg: MpcConfig, *, status: str = "prior", error: str | None = None
) -> dict[str, Any]:
    """The prior memory of a zoned config (every block at its prior, no windows)."""
    d = _derived(cfg)
    st, zone_specs, bay_specs = d.st, d.zone_specs, d.bay_specs
    return {
        "v": VERSION,
        "fp": st.fingerprint,
        "ts": None,
        "error": error,
        "zones": {
            z: {
                "status": status,
                "conv": None,
                "bad": 0,
                "err2": None,
                "prev": None,
                "air": _fresh_block(zone_specs[z], _n_fan_air(zone)),
            }
            for z, zone in st.zones.items()
        },
        "bays": {b: _fresh_block(bay_specs[b], 1) for b in st.bays},
    }


def _num(value: object) -> float:
    if isinstance(value, bool) or not isinstance(value, int | float) or not math.isfinite(value):
        raise ValueError("not a finite number")
    return float(value)


def _opt_num(value: object) -> float | None:
    return None if value is None else _num(value)


def _checked(raw: object, shape: tuple[int, ...]) -> list[Any]:
    """A finite float list (nested for 2-D) of ``shape``; the input list itself when it
    already is one (never mutated in place, so sharing it with the old state is safe)."""
    if not isinstance(raw, list):
        raise TypeError("list")
    arr = np.asarray(raw, dtype=float)
    if arr.shape != shape or not np.isfinite(arr).all():
        raise ValueError("shape or non-finite")
    return raw


def _parse_block(raw: object, spec: _BlockSpec, n_fan: int) -> dict[str, Any]:
    if not isinstance(raw, Mapping):
        raise TypeError("block")
    n = len(spec.keys)
    theta = [_num(v) for v in _checked(raw["theta"], (n,))]
    for i, value in enumerate(theta):
        theta[i] = min(float(spec.hi[i]), max(float(spec.lo[i]), value))
    fm = raw.get("fm")
    counts = []
    for key in ("n", "w"):
        value = raw[key]
        if isinstance(value, bool) or not isinstance(value, int) or value < 0:
            raise ValueError("count")
        counts.append(value)
    rel = raw.get("rel")
    if rel is not None:
        if not isinstance(rel, list) or len(rel) != n:
            raise ValueError("rel")
        rel = [_opt_num(v) for v in rel]
    acc = raw.get("acc")
    return {
        "theta": theta,
        "P": _checked(raw["P"], (n, n)),
        "s2": _num(raw["s2"]),
        "n": counts[0],
        "w": counts[1],
        "m": _checked(raw["m"], (n_fan,)),
        "S": _checked(raw["S"], (n_fan, n_fan)),
        "fm": None if fm is None else _checked(fm, (n_fan,)),
        "pe": _num(raw.get("pe", 0.0)),
        "rel": rel,
        "acc": None if acc is None else _parse_acc(acc, n, n_fan),
    }


def _parse_acc(raw: object, n: int, n_fan: int) -> dict[str, Any]:
    if not isinstance(raw, Mapping):
        raise TypeError("accumulator")
    x = [_num(v) for v in _checked(raw["x"], (n,))]
    fan = [_num(v) for v in _checked(raw["fan"], (n_fan,))]
    sums = [_num(v) for v in _checked(raw["c"], (2,))]
    return {
        "x": x,
        "y": _num(raw["y"]),
        "h": _num(raw["h"]),
        "fan": fan,
        "t0": _num(raw["t0"]),
        "c": sums,
    }


def _parse_sample(raw: object) -> dict[str, Any] | None:
    if raw is None:
        return None
    if not isinstance(raw, Mapping):
        raise TypeError("sample")
    return {
        "ts": _num(raw["ts"]),
        "ok": bool(raw["ok"]),
        "ta": _opt_num(raw.get("ta")),
        "tin": _opt_num(raw.get("tin")),
        "nb": {str(k): _opt_num(v) for k, v in dict(raw.get("nb") or {}).items()},
        "prox": {str(k): _opt_num(v) for k, v in dict(raw.get("prox") or {}).items()},
        "occ": [str(b) for b in raw.get("occ") or []],
        "src": [str(t) for t in raw.get("src") or []],
    }


def _load(memory: object, cfg: MpcConfig, st: Structure) -> dict[str, Any]:
    try:
        return _parse(memory, cfg, st)
    except (TypeError, ValueError, KeyError, AttributeError, IndexError):
        return fresh_memory(cfg)


def _parse(memory: object, cfg: MpcConfig, st: Structure) -> dict[str, Any]:
    if not isinstance(memory, Mapping) or memory.get("v") != VERSION:
        raise ValueError("version")
    if memory.get("fp") != st.fingerprint:
        raise ValueError("structure changed")
    d = _derived(cfg)
    zone_specs, bay_specs = d.zone_specs, d.bay_specs
    error = memory.get("error")
    if error is not None and not isinstance(error, str):
        raise TypeError("error")
    out: dict[str, Any] = {
        "v": VERSION,
        "fp": st.fingerprint,
        "ts": _opt_num(memory.get("ts")),
        "error": error,
        "zones": {},
        "bays": {},
    }
    hold = memory.get("hold")
    if hold is not None:
        if not isinstance(hold, Mapping):
            raise TypeError("hold")
        out["hold"] = {"since": _opt_num(hold.get("since"))}
    zones = memory["zones"]
    for z, zone in st.zones.items():
        raw = zones[z]
        status = raw["status"]
        if status not in STATUSES:
            raise ValueError("status")
        bad = raw.get("bad", 0)
        if isinstance(bad, bool) or not isinstance(bad, int) or bad < 0:
            raise ValueError("bad")
        out["zones"][z] = {
            "status": status,
            "conv": _opt_num(raw.get("conv")),
            "bad": bad,
            "err2": _opt_num(raw.get("err2")),
            "prev": _parse_sample(raw.get("prev")),
            "air": _parse_block(raw["air"], zone_specs[z], _n_fan_air(zone)),
        }
        # a stored split that would reverse a channel's airflow is projected, as a
        # coefficient outside its bounds is clamped (:func:`split_scale`)
        _project_split(out["zones"][z]["air"]["theta"], zone, zone_specs[z].keys)
    for b in st.bays:
        out["bays"][b] = _parse_block(memory["bays"][b], bay_specs[b], 1)
    return out


# ---------------------------------------------------------------------------
# identification: the per-tick update
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class ThermalUpdate:
    """What :func:`update` returns: the next memory and the diagnostics summary."""

    memory: dict[str, Any]
    summary: dict[str, Any]


def _finite(value: object) -> bool:
    if type(value) is float:
        return math.isfinite(value)
    return isinstance(value, int | float) and not isinstance(value, bool) and math.isfinite(value)


def _mean(values: list[float]) -> float | None:
    return sum(values) / len(values) if values else None


def _channel_phi(
    cfg: MpcConfig,
    st: Structure,
    u: Mapping[str, float],
    rpm: Mapping[str, Any] | None,
    curves: Mapping[str, Any] | None = None,
) -> dict[str, float]:
    out: dict[str, float] = {}
    for ch in st.channels:
        name = cfg.fans[ch].model
        model = cfg.fan_models[name]
        deadband, exponent = float(model.deadband), float(model.exponent)
        if curves is not None:
            # the fitted curve supplies the shape only: normalising the tachometer by a
            # rpm_max fitted to those same readings would hide an absolute loss of speed
            deadband, exponent, _ = fancurve.curve_pair(
                curves.get(name), deadband, exponent, model.rpm_max
            )
        reading = None if rpm is None else rpm.get(ch)
        if cfg.model_use_rpm and _finite(reading):
            frac = min(1.0, max(0.0, float(reading) / model.rpm_max))  # type: ignore[arg-type]
            out[ch] = frac**exponent if frac > 0 else 0.0
        else:
            value = u.get(ch)
            out[ch] = phi(float(value), deadband, exponent) if _finite(value) else 0.0  # type: ignore[arg-type]
    return out


def _zone_sensors(st: Structure, zone: ZoneStruct) -> tuple[str, ...]:
    """Every sensor a zone's regressions can read, in a fixed order."""
    names = [*zone.air, *zone.inlet]
    for other in zone.coupled:
        names.extend(st.zones[other].air)
    for b in zone.bays:
        names.extend(st.bays[b].sensors)
    return tuple(dict.fromkeys(names))


def _air_block_sensors(st: Structure, zone: ZoneStruct, occ: Collection[str]) -> tuple[str, ...]:
    names = [*zone.air, *zone.inlet]
    for other in zone.coupled:
        names.extend(st.zones[other].air)
    for b in occ:
        names.extend(st.bays[b].sensors)
    return tuple(names)


def _sources_changed(
    prev: Mapping[str, Any], sample: Mapping[str, Any], names: Collection[str]
) -> bool:
    """Whether the trusted members among ``names`` differ between two samples: redundant
    sensors disagree by their placement offsets, so a mean over a different set steps."""
    before, now = set(prev["src"]), set(sample["src"])
    return any((n in before) != (n in now) for n in names)


def _fresh_acc(n: int, n_fan: int, t0: float) -> dict[str, Any]:
    return {"x": [0.0] * n, "y": 0.0, "h": 0.0, "fan": [0.0] * n_fan, "t0": t0, "c": [0.0, 0.0]}


def _window_sampled(acc: Mapping[str, Any], mass: float, window_s: float) -> bool:
    """Whether a closing window's samples resolve its weight (module docstring).

    Analytically ``int w = T / 2`` and ``int dw = int d2w = 0`` over the window. The
    trapezoid sums of the actual sample times reproduce that exactly for uniform
    sampling at any interval dividing ``T``; an interval that spans most of a short
    window (allowed: up to ``GAP_TICKS`` ticks against a window of 2 ``dt``) does not,
    and its row then carries the absolute temperature through the ``dw`` / ``d2w``
    sums, or divides by a weight of ~0. Such a window is dropped, never learned.
    """
    k = math.pi / window_s
    half_t = 0.5 * window_s
    c_dw, c_ddw = acc["c"]
    return (
        abs(mass - half_t) <= WINDOW_MASS_TOL * half_t
        and abs(c_dw) <= WINDOW_SUM_TOL * k * half_t
        and abs(c_ddw) <= WINDOW_SUM_TOL * 2.0 * k * k * half_t
    )


def _weights(t: float, window_s: float) -> tuple[float, float, float]:
    """``(w, dw/dt, d2w/dt2)`` of the window ``w = sin^2(pi t / T)`` (0 outside ``[0, T]``)."""
    if not 0.0 <= t <= window_s:
        return 0.0, 0.0, 0.0
    a = math.pi * t / window_s
    k = math.pi / window_s
    return math.sin(a) ** 2, k * math.sin(2.0 * a), 2.0 * k * k * math.cos(2.0 * a)


def update(
    memory: object,
    cfg: MpcConfig,
    *,
    temps: Mapping[str, float],
    u: Mapping[str, float],
    ts: float,
    zones_ok: Collection[str],
    occupancy: Mapping[str, str] | None = None,
    maps: Mapping[str, tuple[float, float]] | None = None,
    classes: Mapping[str, str] | None = None,
    rpm: Mapping[str, Any] | None = None,
    reset_bays: Collection[str] = (),
    curves: Mapping[str, Any] | None = None,
    learn: bool = True,
) -> ThermalUpdate:
    """One identification tick (module docstring).

    ``temps`` holds this tick's gate-trusted values only, ``u`` the command on the
    fans since the previous tick (``prev``), ``zones_ok`` the zones that are trusted
    and not in fault on this tick; ``occupancy``, ``maps`` (``(s, b)`` per bay) and
    ``classes`` come from the estimator (defaults: declared occupancy, prior map,
    declared class); ``rpm`` is ``obs.rpm`` (read with ``model_use_rpm``). Raises
    only on a numerical failure (non-finite RLS state), which ``step`` turns into
    ``status: error``.

    ``reset_bays`` are the bays the estimator reported a hot swap on this tick
    (``swapped``); with ``model_reset_on_swap`` their blocks start over from the prior
    before anything else (module docstring, *Hot swap*).

    ``learn=False`` (used offline by ``tools/replay.py`` and ``tools/fit_model.py``'s
    hold-out pass, never by ``mpc.step``) still tracks windows, excitation and the
    PE monitor, and still returns a genuine a-priori residual per closing window, but
    never moves ``theta``/``P``: a frozen-parameter prediction check on data the
    parameters were not fitted from.
    """
    d = _derived(cfg)
    st, zone_specs, bay_specs = d.st, d.zone_specs, d.bay_specs
    mem = _load(memory, cfg, st)
    _reset_swapped_bays(mem, cfg, st, bay_specs, reset_bays)
    maps = dict(maps or {})
    ts = float(ts)
    last_ts = mem["ts"]
    h = None if last_ts is None else ts - last_ts
    interval_ok = h is not None and 0.0 < h <= GAP_TICKS * cfg.dt
    window_s = cfg.model_window_s

    # current snapshot of the coefficients (for H_z and Qn)
    theta = theta_from_memory(cfg, mem, st=st)
    params = model_params(
        cfg, theta, st=st, occupancy=occupancy, maps=maps, classes=classes, curves=curves
    )
    phis = _channel_phi(cfg, st, u, rpm, curves)
    closed: dict[str, list[tuple[float, bool]]] = {}  # zone -> (residual degC, excited)

    for z, zone in st.zones.items():
        zm = mem["zones"][z]
        ok = z in zones_ok
        sample = {
            "ts": ts,
            "ok": ok,
            "ta": _mean([temps[t] for t in zone.air if t in temps]),
            "tin": _mean([temps[t] for t in zone.inlet if t in temps]),
            "nb": {
                o: _mean([temps[t] for t in st.zones[o].air if t in temps]) for o in zone.coupled
            },
            "prox": {
                b: _mean([temps[t] for t in st.bays[b].sensors if t in temps]) for b in zone.bays
            },
            "occ": [b for b in zone.bays if params.occupied[b]],
            # the sensors behind those means: a change restarts the windows that read them
            "src": [t for t in _zone_sensors(st, zone) if t in temps],
        }
        prev = zm["prev"]
        zm["prev"] = sample
        zone_learn = learn and zm["status"] != "frozen"
        if not (interval_ok and ok and prev is not None and prev["ok"]):
            zm["air"]["acc"] = None
            for b in zone.bays:
                mem["bays"][b]["acc"] = None
            continue
        assert h is not None and prev is not None
        group_phi = [_group_phi(gr, phis) for gr in zone.groups]
        e_total = sum(theta[gr.key] for gr in zone.groups)
        q_flow = sum(theta[gr.key] * v for gr, v in zip(zone.groups, group_phi, strict=True))
        for i, gr in enumerate(zone.groups):  # the split terms (0 under common motion)
            q_flow += sum(theta[key] * (phis[ch] - group_phi[i]) for key, ch in gr.splits)
        q_flow = max(q_flow, 0.0)
        qn = q_flow / e_total if e_total > _EPS else 0.0
        ta0, ta1 = prev["ta"], sample["ta"]
        tau_a = zone.tau_air

        # -- proximal blocks ------------------------------------------------------------
        for b in zone.bays:
            block = mem["bays"][b]
            p0, p1 = prev["prox"].get(b), sample["prox"][b]
            if (
                not params.occupied[b]
                or b not in prev["occ"]
                or None in (p0, p1, ta0, ta1)
                or _sources_changed(prev, sample, (*st.bays[b].sensors, *zone.air))
            ):
                block["acc"] = None
                continue
            assert p0 is not None and p1 is not None and ta0 is not None and ta1 is not None
            acc = block["acc"]
            if acc is None:
                acc = block["acc"] = _fresh_acc(3, 1, prev["ts"])
            w0, dw0, ddw0 = _weights(prev["ts"] - acc["t0"], window_s)
            w1, dw1, ddw1 = _weights(ts - acc["t0"], window_s)
            s_map, b_map, tau = params.s[b], params.b[b], params.tau_s[b]
            cd = params.c_drive[b]
            half = 0.5 * h
            weight = half * (w0 + w1)
            # int w (S - T_a - b) dt with the unlagged S = T_s + tau dT_s/dt, T_a likewise
            drive = half * (w0 * (p0 - ta0 - b_map) + w1 * (p1 - ta1 - b_map))
            drive += 0.5 * (w0 + w1) * (tau * (p1 - p0) - tau_a * (ta1 - ta0))
            acc["x"][0] += weight
            acc["x"][1] -= drive / cd
            acc["x"][2] -= qn * drive / cd
            # -int dw S dt + (1 - s) int dw T_a dt; by parts int dw tau dT/dt = -tau int ddw T
            acc["y"] += -half * (dw0 * p0 + dw1 * p1) + tau * half * (ddw0 * p0 + ddw1 * p1)
            acc["y"] += (1.0 - s_map) * (
                half * (dw0 * ta0 + dw1 * ta1) - tau_a * half * (ddw0 * ta0 + ddw1 * ta1)
            )
            acc["fan"][0] += weight * qn
            acc["h"] += h
            acc["c"][0] += half * (dw0 + dw1)
            acc["c"][1] += half * (ddw0 + ddw1)
            if ts - acc["t0"] + 1e-9 < window_s:
                continue
            if not _window_sampled(acc, acc["x"][0], window_s):
                block["acc"] = None
                continue
            # rows stay in kelvin (a change of the sensor target mid-window weighs 1)
            fan = np.array(acc["fan"]) / acc["x"][0]
            x = np.array(acc["x"])
            residual, excited = _rls_window(
                block, bay_specs[b], x, acc["y"], fan, cfg, learn=zone_learn
            )
            closed.setdefault(z, []).append((residual, excited))
            block["acc"] = None

        # -- zone air block, anchored on the drive heat --------------------------------------
        block = zm["air"]
        occ = sample["occ"]
        nb0 = [prev["nb"].get(o) for o in zone.coupled]
        nb1 = [sample["nb"][o] for o in zone.coupled]
        if (
            occ != prev["occ"]
            or None in (ta0, ta1, prev["tin"], sample["tin"])
            or None in nb0
            or None in nb1
            or any(prev["prox"].get(b) is None or sample["prox"][b] is None for b in occ)
            or _sources_changed(prev, sample, _air_block_sensors(st, zone, occ))
        ):
            block["acc"] = None
            continue
        assert ta0 is not None and ta1 is not None
        acc = block["acc"]
        strong = [i for i, gr in enumerate(zone.groups) if not gr.weak]
        if acc is None:
            acc = block["acc"] = _fresh_acc(len(zone.air_keys), len(strong), prev["ts"])
        w0, dw0, ddw0 = _weights(prev["ts"] - acc["t0"], window_s)
        w1, dw1, ddw1 = _weights(ts - acc["t0"], window_s)
        half = 0.5 * h
        weight = half * (w0 + w1)
        wm = 0.5 * (w0 + w1)
        lag_a = tau_a * (ta1 - ta0)  # every sensor unlagged as T + tau dT/dt (module docstring)
        din = half * (w0 * (ta0 - prev["tin"]) + w1 * (ta1 - sample["tin"]))
        din += wm * lag_a
        heat = 0.0
        for b in occ:
            g = theta[f"g0.{b}"] + theta[f"k.{b}"] * qn
            p0, p1 = prev["prox"][b], sample["prox"][b]
            drive = half * (w0 * (p0 - ta0 - params.b[b]) + w1 * (p1 - ta1 - params.b[b]))
            drive += wm * (params.tau_s[b] * (p1 - p0) - lag_a)
            heat += g * drive / params.s[b]
        for i, value in enumerate(group_phi):
            acc["x"][i] += value * din
        i_leak = len(zone.groups)
        for i, gr in enumerate(zone.groups):
            for _, ch in gr.splits:  # regressor (phi_ch - phi_zG): 0 under common motion
                acc["x"][i_leak] += (phis[ch] - group_phi[i]) * din
                i_leak += 1
        acc["x"][i_leak] += din
        for j, other in enumerate(zone.coupled):
            nb = half * (w0 * (nb0[j] - ta0) + w1 * (nb1[j] - ta1))
            nb += wm * (st.zones[other].tau_air * (nb1[j] - nb0[j]) - lag_a)
            acc["x"][i_leak + 1 + j] -= nb
        acc["x"][-1] -= weight
        # int w H dt + C_a int dw T_a dt (T_a unlagged: -tau_a int ddw T_a)
        acc["y"] += heat + params.c_air[z] * (
            half * (dw0 * ta0 + dw1 * ta1) - tau_a * half * (ddw0 * ta0 + ddw1 * ta1)
        )
        for j, i in enumerate(strong):
            acc["fan"][j] += weight * group_phi[i]
        acc["h"] += h
        acc["c"][0] += half * (dw0 + dw1)
        acc["c"][1] += half * (ddw0 + ddw1)
        if ts - acc["t0"] + 1e-9 < window_s:
            continue
        norm = -acc["x"][-1]
        if not _window_sampled(acc, norm, window_s):
            block["acc"] = None
            continue
        x = np.array(acc["x"]) / norm
        fan = np.array(acc["fan"]) / norm
        residual, excited = _rls_window(
            block, zone_specs[z], x, acc["y"] / norm, fan, cfg, learn=zone_learn
        )
        # a window may have moved a split past the point where a channel would cool
        # less than nothing; the projection scales it back, total untouched
        _project_split(block["theta"], zone, zone_specs[z].keys)
        conductance = q_flow + theta[f"leak.{z}"]
        conductance += sum(theta[f"kappa.{z}.{o}"] for o in zone.coupled)
        conductance += sum(theta[f"g0.{b}"] + theta[f"k.{b}"] * qn for b in occ)
        closed.setdefault(z, []).append((residual / max(conductance, 1.0), excited))
        block["acc"] = None

    # -- status machine -------------------------------------------------------------
    for z, results in closed.items():
        zm = mem["zones"][z]
        err2 = sum(r * r for r, _ in results) / len(results)
        zm["err2"] = (
            err2
            if zm["err2"] is None
            else (1.0 - PRED_ERR_ALPHA) * zm["err2"] + PRED_ERR_ALPHA * err2
        )
        excited = any(e for _, e in results)
        _advance_status(
            zm, mem, cfg, st, z, math.sqrt(err2), excited, params, zone_specs, bay_specs
        )

    _advance_hold(mem, cfg, ts)
    mem["ts"] = ts
    return ThermalUpdate(memory=mem, summary=summary(mem, cfg, st=st, occupancy=occupancy))


def _reset_swapped_bays(
    mem: dict[str, Any],
    cfg: MpcConfig,
    st: Structure,
    bay_specs: Mapping[str, _BlockSpec],
    reset_bays: Collection[str],
) -> None:
    """Start a hot-swapped bay's block over from the prior (plan section 8 item 12).

    ``g0``, ``k`` and ``q_s`` describe the drive that was in the bay; a different one
    has its own. The window in progress goes with them, and so does the zone air block's
    window, which anchors on this bay's heat through those very coefficients. The zone's
    status is left alone: the bay relearns like a new one, and demoting the zone would
    park the DAS MPC in its fallback until the next identification experiment. The
    consequence is deliberate and worth knowing: the zone keeps its ``converged``
    status, its ``conv`` level and its exponentially-weighted ``err2`` from before the
    swap, so ``solver_das.check_model`` accepts the zone while this bay's block is back
    at the generic prior, until the prediction error has climbed over the next few
    windows. Cooling is not reduced by it -- the estimator inflates the swapped bay's
    variance in the same tick, so its margin dominates the plan.

    A ``frozen`` zone never moves its coefficients (a model loaded converged from a fresh
    store file), so resetting one of its bays would strand it at the prior: it is skipped.
    """
    if not reset_bays or not cfg.model_reset_on_swap:
        return
    for b in reset_bays:
        if b not in st.bays:
            continue
        z = st.bays[b].zone
        if mem["zones"][z]["status"] == "frozen":
            continue
        mem["bays"][b] = _fresh_block(bay_specs[b], 1)
        mem["zones"][z]["air"]["acc"] = None


def _zone_blocks_converged(
    mem: Mapping[str, Any],
    cfg: MpcConfig,
    st: Structure,
    z: str,
    params: ThermalParams,
    zone_specs: Mapping[str, _BlockSpec],
    bay_specs: Mapping[str, _BlockSpec],
) -> bool:
    checks = [(mem["zones"][z]["air"], zone_specs[z])]
    checks += [(mem["bays"][b], bay_specs[b]) for b in st.zones[z].bays if params.occupied[b]]
    for block, spec in checks:
        rel = block["rel"]
        if int(block["n"]) < MIN_WINDOWS or float(block["pe"]) <= PE_MIN or rel is None:
            return False
        for i in spec.gain:
            if rel[i] is None or rel[i] >= cfg.model_converged_rel_se:  # type: ignore[operator]
                return False
    return True


def _advance_status(
    zm: dict[str, Any],
    mem: Mapping[str, Any],
    cfg: MpcConfig,
    st: Structure,
    z: str,
    window_err: float,
    excited: bool,
    params: ThermalParams,
    zone_specs: Mapping[str, _BlockSpec],
    bay_specs: Mapping[str, _BlockSpec],
) -> None:
    """One step of the zone's status machine after its windows closed (module docstring)."""
    status = zm["status"]
    pred_err = math.sqrt(zm["err2"]) if zm["err2"] is not None else math.inf
    if status in ("prior", "error", "suspect") and excited:
        status = "learning"
        zm["bad"] = 0
    if (
        status == "learning"
        and pred_err < cfg.model_max_pred_err_c
        and _zone_blocks_converged(mem, cfg, st, z, params, zone_specs, bay_specs)
    ):
        # model_freeze: enter the converged model as frozen, so nothing adapts it further
        status = "frozen" if cfg.model_freeze else "converged"
        zm["conv"] = max(pred_err, PRED_ERR_FLOOR_C)
        zm["bad"] = 0
    elif status in ("converged", "frozen"):
        level = max(float(zm["conv"] or PRED_ERR_FLOOR_C), PRED_ERR_FLOOR_C)
        zm["bad"] = int(zm["bad"]) + 1 if window_err > SUSPECT_FACTOR * level else 0
        if zm["bad"] >= SUSPECT_WINDOWS:
            status = "suspect"
    zm["status"] = status


def _advance_hold(mem: dict[str, Any], cfg: MpcConfig, ts: float) -> None:
    """The stale hold after a tick (module docstring, *Stale hold*)."""
    hold = mem.get("hold")
    if hold is None:
        return
    zones = list(mem["zones"].values())
    errs = [zm["err2"] for zm in zones]
    ok = (
        # frozen counts: with model_freeze every re-confirmed zone enters frozen
        overall_status([zm["status"] for zm in zones]) in ("converged", "frozen")
        and all(e is not None for e in errs)
        and math.sqrt(max(float(e) for e in errs)) < cfg.model_max_pred_err_c
    )
    if not ok:
        hold["since"] = None
        return
    since = hold["since"]
    if since is None or since > ts:  # a clock stepped back restarts the count from now
        since = ts
    if ts - since >= cfg.model_reconfirm_s:
        del mem["hold"]
        return
    hold["since"] = since


# ---------------------------------------------------------------------------
# model store: restoring a saved memory (aqua_bridge.control.persist)
# ---------------------------------------------------------------------------

#: Stored zone status -> restored status, for a fresh file and for a stale one.
_RESTORE_FRESH = {
    "converged": "frozen",
    "frozen": "frozen",
    "learning": "learning",
    "prior": "prior",
    "suspect": "suspect",
    "error": "prior",
}
_RESTORE_STALE = {
    "converged": "learning",
    "frozen": "learning",
    "learning": "learning",
    "prior": "prior",
    "suspect": "suspect",
    "error": "prior",
}
_SYMMETRY_TOL = 1e-6


def _check_block_bounds(raw: object, spec: _BlockSpec, where: str) -> None:
    if not isinstance(raw, Mapping):
        raise ValueError(f"{where}: not a mapping")
    theta = raw.get("theta")
    n = len(spec.keys)
    if not isinstance(theta, list) or len(theta) != n:
        raise ValueError(f"{where}: theta has the wrong shape")
    for i, value in enumerate(theta):
        if not _finite(value):
            raise ValueError(f"{where}: {spec.keys[i]} is not a finite number")
        if not float(spec.lo[i]) <= float(value) <= float(spec.hi[i]):
            raise ValueError(
                f"{where}: {spec.keys[i]} = {value} outside [{spec.lo[i]:g}, {spec.hi[i]:g}]"
            )
    try:
        p = np.asarray(raw.get("P"), dtype=float)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{where}: P is not a matrix") from exc
    if p.shape != (n, n) or not np.all(np.isfinite(p)):
        raise ValueError(f"{where}: P is not a finite {n}x{n} matrix")
    scale = max(1.0, float(np.max(np.abs(p))))
    if float(np.max(np.abs(p - p.T))) > _SYMMETRY_TOL * scale or np.any(np.diag(p) < 0):
        raise ValueError(f"{where}: P is not a covariance (asymmetric or negative variance)")
    s2 = raw.get("s2")
    if not _finite(s2) or float(s2) <= 0:  # type: ignore[arg-type]
        raise ValueError(f"{where}: s2 must be a finite number > 0")


def _convert_air_block(
    raw: object, old_keys: tuple[str, ...], spec: _BlockSpec, where: str
) -> dict[str, Any]:
    """One zone's air block re-indexed from ``old_keys`` to ``spec.keys``.

    Only the per-channel split coefficients differ between the two (``model_split_channels``
    on or off), so every shared key keeps its value, its variance and its covariances with
    the other shared keys, and a key that is new starts at its prior with its initial
    variance and no covariance. The open window is dropped (its accumulator has the old
    width) and the relative standard errors come back on the next closing window.

    Raises ``ValueError`` -- never anything else -- for a malformed block, so a corrupt
    coefficient costs the thermal section alone and not the whole seed
    (:func:`aqua_bridge.control.persist.apply_seed`); the bounds themselves are checked
    afterwards, on the converted block.
    """
    if not isinstance(raw, Mapping):
        raise ValueError(f"{where}: not a mapping")
    theta_in = raw.get("theta")
    p_in = raw.get("P")
    n_old = len(old_keys)
    if not isinstance(theta_in, list) or len(theta_in) != n_old:
        raise ValueError(f"{where}: theta has the wrong shape")
    try:
        p_old = np.asarray(p_in, dtype=float)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{where}: P is not a matrix") from exc
    if p_old.shape != (n_old, n_old):
        raise ValueError(f"{where}: P is not a finite {n_old}x{n_old} matrix")
    where_old = {key: i for i, key in enumerate(old_keys)}
    theta = list(spec.prior)
    p = np.diag(spec.var0).astype(float)
    shared = [(i, where_old[key]) for i, key in enumerate(spec.keys) if key in where_old]
    for i, j in shared:
        # checked here, not only in _check_block_bounds afterwards: a null or a string in
        # the file must raise ValueError like every other malformed field, never TypeError
        if not _finite(theta_in[j]):
            raise ValueError(f"{where}: {spec.keys[i]} is not a finite number")
        theta[i] = float(theta_in[j])
    rows = np.array([i for i, _ in shared], dtype=int)
    cols = np.array([j for _, j in shared], dtype=int)
    if rows.size:
        p[np.ix_(rows, rows)] = p_old[np.ix_(cols, cols)]
    out = dict(raw)
    out["theta"] = theta
    out["P"] = p.tolist()
    out["rel"] = None
    out["acc"] = None
    return out


def restore(stored: object, cfg: MpcConfig, *, stale: bool) -> dict[str, Any]:
    """A thermal memory saved by the model store, ready for :func:`update`.

    Strict, unlike :func:`update`'s own loading (which starts over on anything it cannot
    read and clamps coefficients into their bounds): raises ``ValueError`` naming the
    first problem -- schema version, a structure that differs from ``cfg``'s, a
    coefficient that is not finite or lies outside its bounds (:data:`PARAMETERS`), a
    covariance that is not finite, symmetric and non-negative on its diagonal, or any
    malformed field. The result has no open window, no previous sample, no error and no
    ``ts`` (the next tick starts the windows in the new clock); zone statuses are mapped
    for a fresh or a ``stale`` file (module docstring, ``frozen`` and *Stale hold*), and a
    stale file -- or a stored memory with a pending hold -- gets a new hold.
    """
    d = _derived(cfg)
    st = d.st
    if not isinstance(stored, Mapping) or stored.get("v") != VERSION:
        raise ValueError("thermal: unknown schema version")
    converted = False
    if stored.get("fp") != st.fingerprint:
        if stored.get("fp") != st.alt_fingerprint:
            raise ValueError("thermal: the model structure differs from the config")
        converted = True  # written with model_split_channels the other way round
    zones = stored.get("zones")
    bays = stored.get("bays")
    if not isinstance(zones, Mapping) or not isinstance(bays, Mapping):
        raise ValueError("thermal: zones or bays missing")
    if converted:
        rebuilt: dict[str, Any] = {}
        for z in st.zones:
            raw = zones.get(z)
            if not isinstance(raw, Mapping):
                raise ValueError(f"thermal: zone {z!r} missing")
            rebuilt[z] = {
                **raw,
                "air": _convert_air_block(
                    raw.get("air"), st.alt_air_keys[z], d.zone_specs[z], f"thermal: zone {z!r}"
                ),
            }
        stored = {**stored, "fp": st.fingerprint, "zones": rebuilt}
        zones = rebuilt
    for z in st.zones:
        raw = zones.get(z)
        if not isinstance(raw, Mapping):
            raise ValueError(f"thermal: zone {z!r} missing")
        _check_block_bounds(raw.get("air"), d.zone_specs[z], f"thermal: zone {z!r}")
    for b in st.bays:
        _check_block_bounds(bays.get(b), d.bay_specs[b], f"thermal: bay {b!r}")
    try:
        mem = _parse(stored, cfg, st)
    except (TypeError, ValueError, KeyError, AttributeError, IndexError) as exc:
        raise ValueError(f"thermal: malformed memory ({type(exc).__name__}: {exc})") from exc
    stale = stale or "hold" in mem
    mapping = _RESTORE_STALE if stale else _RESTORE_FRESH
    mem["ts"] = None
    mem["error"] = None
    for zm in mem["zones"].values():
        zm["status"] = mapping[zm["status"]]
        zm["prev"] = None
        zm["bad"] = 0
        zm["air"]["acc"] = None
        if stale:
            zm["err2"] = None
    for block in mem["bays"].values():
        block["acc"] = None
    mem.pop("hold", None)
    if stale:
        mem["hold"] = {"since": None}
    return mem


# ---------------------------------------------------------------------------
# summary (diagnostics, GET /api/model) and the model the DAS MPC reads
# ---------------------------------------------------------------------------


def cached_structure(cfg: MpcConfig) -> Structure:
    """:func:`structure` of ``cfg``, memoised like :func:`update`'s (same result, cheaper)."""
    return _derived(cfg).st


def current_model(memory: object, cfg: MpcConfig) -> tuple[str, dict[str, float]]:
    """``(status, theta)`` of a thermal memory, for the DAS MPC's validity gate.

    ``status`` is the model's status (:func:`model_status`) and ``theta``
    every identified parameter (:func:`prior_theta` layout) as the blocks hold it.
    ``None`` (no ``model_shadow``) reads ``("off", prior)``; a memory that does not
    match the config's structure, or is malformed, reads as the fresh prior
    (``"prior"``), exactly as :func:`update` would start over from it.
    """
    d = _derived(cfg)
    if memory is None:
        return "off", dict(d.prior)
    mem = _load(memory, cfg, d.st)
    theta = theta_from_memory(cfg, mem, st=d.st)
    return model_status(mem), {k: float(v) for k, v in theta.items()}


def overall_status(zone_statuses: Collection[str]) -> str:
    """The model's status from its zones' (module docstring): ``error``, then ``suspect``,
    then ``frozen`` when every zone is ``converged`` or ``frozen`` and one is ``frozen``,
    ``converged`` when every zone is, ``learning`` when some zone got that far, else
    ``prior``."""
    statuses = set(zone_statuses)
    if not statuses:
        return "prior"
    if "error" in statuses:
        return "error"
    if "suspect" in statuses:
        return "suspect"
    if statuses <= {"converged", "frozen"}:
        return "frozen" if "frozen" in statuses else "converged"
    if statuses & {"learning", "converged", "frozen"}:
        return "learning"
    return "prior"


def model_status(memory: Mapping[str, Any]) -> str:
    """The status of a (parsed) thermal memory: ``stale`` while a stale hold is pending,
    else :func:`overall_status` of its zones."""
    if memory.get("hold") is not None:
        return "stale"
    return overall_status([zm["status"] for zm in memory["zones"].values()])


def summary(
    memory: Mapping[str, Any],
    cfg: MpcConfig,
    *,
    st: Structure | None = None,
    occupancy: Mapping[str, str] | None = None,
) -> dict[str, Any]:
    """``diagnostics["thermal"]``: status, prediction error and the coefficients (plain JSON)."""
    st = _derived(cfg).st if st is None else st
    zones_out: dict[str, Any] = {}
    worst: float | None = None
    for z, zone in st.zones.items():
        zm = memory["zones"][z]
        block = zm["air"]
        rel = block.get("rel") or [None] * len(zone.air_keys)
        pred = None if zm["err2"] is None else math.sqrt(float(zm["err2"]))
        if pred is not None:
            worst = pred if worst is None else max(worst, pred)
        theta = dict(zip(zone.air_keys, block["theta"], strict=True))
        zones_out[z] = {
            "status": zm["status"],
            "pred_err_c": pred,
            "windows": block["w"],
            "excited_windows": block["n"],
            "pe_min": block["pe"],
            "theta": theta,
            "rel_se": dict(zip(zone.air_keys, rel, strict=True)),
        }
        if any(gr.splits for gr in zone.groups):
            # what the split says each channel contributes, W/K (module docstring, *Split*)
            per_channel: dict[str, float] = {}
            for gr in zone.groups:
                per_channel.update(channel_effectiveness(gr, theta))
            zones_out[z]["e_per_channel"] = per_channel
    bays_out: dict[str, Any] = {}
    for b, bay in st.bays.items():
        block = memory["bays"][b]
        rel = block.get("rel") or [None] * len(bay.keys)
        bays_out[b] = {
            "zone": bay.zone,
            "windows": block["w"],
            "excited_windows": block["n"],
            "pe_min": block["pe"],
            "theta": dict(zip(bay.keys, block["theta"], strict=True)),
            "rel_se": dict(zip(bay.keys, rel, strict=True)),
        }
        if occupancy is not None and b in occupancy:
            bays_out[b]["occupancy"] = occupancy[b]
    out: dict[str, Any] = {
        "status": model_status(memory),
        "error": memory.get("error"),
        "pred_err_c": worst,
        "max_pred_err_c": cfg.model_max_pred_err_c,
        "window_s": cfg.model_window_s,
        "freeze": cfg.model_freeze,
        "zones": zones_out,
        "bays": bays_out,
    }
    hold = memory.get("hold")
    if hold is not None:
        out["hold"] = {"since_ts": hold.get("since"), "reconfirm_s": cfg.model_reconfirm_s}
    return out
