"""DAS MPC: least modelled fan noise with every drive under its targets (plan sections 4, 6).

The ``mpc`` solver of a zoned config that regulates drive limits (``topology`` and
no ``setpoints``, :attr:`~aqua_bridge.model.MpcConfig.regulates_drive_limits`);
``mpc.step`` picks it instead of the legacy :class:`~aqua_bridge.control.solver_mpc.
MpcSolver`, which stays unchanged for legacy and zoned setpoint configs. Pure like
every solver: no clock (``SolverRequest.ts`` is ``obs.ts``), no I/O, no randomness,
all state in plain JSON (``solver_memory["mpc"]``). ``step`` still owns the gate, the
zone faults, the fallback policy, the rate limit and the clamp.

Per-tick problem
----------------
Decision ``U``: ``n_b`` move blocks (:meth:`~aqua_bridge.model.MpcConfig.blocks`,
lengths ``len_b`` summing to ``horizon``) times the channels; block ``b`` holds its
command for ``len_b`` prediction steps of ``h = mpc_pred_dt_s``. Outputs: the
predicted drive temperature ``T_d`` of every bay with an estimate in a trusted zone
(``SolverRequest.estimates``) after every step, plus the **terminal equilibrium**
rows ``T_d,ss(u_last) = -C A^-1 (B u_last + c)`` (a 10-minute horizon is one drive
time constant: without them a slow drive would look safe until it is too late)::

    min  sum_b len_b sum_i weight_noise [g_i (u_ib - u_i) + 1/2 h_i (u_ib - u_i)^2]
       + weight_dpwm sum_b ||u_b - u_(b-1)||^2                    u_(-1) = prev
       + rho_soft sum_rows (y_r - soft_r)_+^2 + rho_hard sum_rows (y_r - hard_r)_+^2
    s.t. pwm_min <= U <= pwm_max

with the quadratic noise surrogate of :func:`aqua_bridge.control.noise.surrogate` at
``u = prev`` (normalised to 1 at full speed), ``soft = limit - comfort - k sigma`` and
``hard = limit - k sigma`` from the estimates block. ``k sigma`` counts once (the
PI-like DAS form counts it twice, see ``solver_pi``). The soft constraints are slack
penalties: quadratic penalties leave a small steady violation of the soft target that
the comfort band absorbs (plan section 12, risk 9). A tiny ridge
(:data:`RIDGE` per variable) keeps the Hessian positive definite when a fan has
``noise_weight: 0`` and ``weight_dpwm`` is 0.

Channels in ``SolverRequest.fixed_channels`` (zones in fault and, with declared
coupling, their neighbours) are known inputs at their fallback command: their
variables are removed from the problem and enter the prediction as constants, the
faulted zones' rows are absent (their bays are not in ``estimates``), and the healthy
zones' rows still see the fixed channels' airflow and the faulted zones' air through
the model, so a neighbour's fault can only add cooling to a healthy zone.

Prediction model
----------------
The zoned network of :mod:`aqua_bridge.control.thermal` with the identified
parameters (:func:`~aqua_bridge.control.thermal.current_model`), the estimator's
occupancy and classes, linearised per tick at ``(x_hat, prev)``: ``x_hat`` is the
estimator's air per zone and drive per occupied or unknown bay, and its integrating
disturbances (``d_air`` per zone, ``q = q_w / C_d`` per bay) and inlet temperatures
enter the affine term. The state is reduced to the air and drive nodes: the proximal
sensor nodes feed nothing back into them and the outputs are drives, so dropping them
is exact, and an empty bay's frozen drive (eigenvalue 0) is not a state at all (19
states instead of 34 for the example layout; the plan counts 34). ``A`` is taken at the
**quantised command** (:data:`QUANT_U`, clamped into the fan curve's open range so a
command at full speed or in the dead band keeps the one-sided slope of the curve), ``B``
at that command and the exact state (``B`` is affine in the temperatures), and the
affine term ``c = f(x_hat, prev) - A x_hat - B prev`` exactly, so the model is exact at
the operating point. ``A``, its discretisation (exact zero-order hold by one matrix
exponential, :func:`zoh`), its inverse for the terminal rows and the command-dependent
pieces of ``B`` depend only on the parameters and the quantised command and are
memoised on them (plan section 4, reduction 1, which also quantises the temperatures;
the affine structure of ``B`` makes that unnecessary): a memo, not state; cached or
recomputed the result is bit-identical. The condensation keeps the output rows only.

Disturbances. The estimator's heat per drive ``q`` is a random walk that follows sensor
noise tick by tick (about 0.4 W of tick-to-tick noise per HDD bay on the truth simulator
with DS18B20 noise), and the terminal rows turn it into ``q / g`` degrees C of steady
state (0.6 degC): fed raw, the plan chattered by 0.1 PWM per tick. The prediction
therefore uses ``q`` and ``d_air`` low-passed in the solver's memory with the time
constant :data:`DIST_TAU_S`, restarted from the estimate when a bay's occupancy changes;
the drive and air states are used as estimated, so a real heat burst still shows in the
rows as the drive warms (drive time constants are minutes). Measured on the truth
simulator: 60 s left 0.05 PWM of chatter; a filter that
rises faster than it falls (30 s / 120 s) biased the steady state 0.3 degC high, which
over-cooled the hot bays and cost noise; 120 s both ways did neither.

Solver: active-piece SQP on ``solve_box_qp``
--------------------------------------------
With the currently violated rows ``V``: ``H = H0 + 2 sum_(r in V) rho_r s_r s_r^T``,
``f = f0 + 2 sum_(r in V) rho_r (y0_r - t_r) s_r``; solve the box QP warm-started
(:func:`~aqua_bridge.control.solver_mpc.solve_box_qp`, the legacy MPC's exact primal
active-set method; when it needs more than :data:`QP_FAST_ITER` iterations from the
current point it restarts from a few projected Newton iterations (:func:`projected_newton`)),
on the piece normalised by its Gershgorin bound and with a Newton-step
tolerance of :data:`QP_STEP_TOL` PWM: with penalty rows the Hessian's condition number
reaches ~1e7, the rounding of a zero Newton step then exceeds the legacy 1e-12 and the
method spent hundreds of iterations on rounding), then backtrack on the true penalised
objective (convex, C1) until it does not increase, recompute ``V`` and stop when ``V``
is unchanged after a full step (then the point is the global minimiser: the piece's KKT
conditions are the true problem's), the decrease is below :data:`REL_DECREASE_TOL`
relative, an accepted step moves no variable by more than :data:`SQP_STEP_TOL` (a
ten-thousandth of the PWM range, below the actuators' resolution), no step descends,
or after ``solver_outer_max`` iterations -- monotone and finite (:func:`solve_penalty_qp`).
A box QP that hits ``solver_max_iter`` is ``converged=False``, which ``step`` turns into a
solver fault exactly as for the legacy MPC; ``iterations`` reports the largest count of
a single box QP (checked against ``solver_max_iter``), the total is in the diagnostics.

Honest saturation: the demand of a channel is its first block plus the pressure on an
active bound, ``u - g / H_ii`` with the true gradient ``g`` and the final piece's
curvature, exactly as the legacy MPC does; a demand above ``pwm_max`` makes ``step``
report ``saturated``.

Forbidden PWM bands (``fans.<ch>.forbidden_pwm``) are non-convex and handled after the
solve (:func:`snap_bands`): a demand inside a band snaps to its upper edge (more
cooling); a channel held at a band leaves it upward when the demand exceeds the upper
edge and downward only when the demand drops below ``lo - noise.band_hysteresis``.
Deterministic, and ``step`` rate-limits the result as usual.

``mpc_every_ticks``: the SQP and the validity gate run on every n-th call (and at once
when the fixed channels, the trusted zones, the constrained bays or the active model
change, or the plan ran out); in between the stored plan is replayed (the block of the
elapsed time, the first block with its pressure). The prediction-error bookkeeping and
the PI-like DAS fallback run every tick.

Validity gate and model fallback
--------------------------------
On every solve tick the model must pass (:func:`check_model`):

* the thermal status is ``converged`` or ``frozen`` (``frozen`` comes with the model
  store, a later milestone); with ``model_accept_prior: true`` also ``prior`` /
  ``learning`` and ``off`` (no ``model_shadow``: the prior parameters) -- an owner
  opt-in this milestone adds, because without experiments and the store no model
  can reach ``converged`` yet (identification from regulation alone never does);
* every parameter finite and inside its bounds (``thermal.PARAMETERS``);
* every eigenvalue of ``Ad`` is real and in ``(0, 1)``: every eigenvalue of the
  continuous ``A`` real (:data:`EIG_IMAG_REL`) and negative, and ``A`` invertible;
* every constrained bay has at least one channel whose steady-state gain on its drive
  is non-zero (:data:`GAIN_MIN`);
* the rolling one-step prediction error of the drive rows is at most
  ``model_max_pred_err_c``: on a solve tick the model predicts the drives one
  prediction step ``h`` ahead with the command held at ``prev`` (the drive rows move
  over minutes, the command difference within one step is second order), and the
  first tick at or after that time compares it with the estimator's drives
  (worst bay), as an exponentially weighted RMS (:data:`PRED_ERR_ALPHA`);
* the equilibrium drift ``max_j |dT_d,j/dt|`` of the model at ``(x_hat, prev)`` over the
  constrained bays is at most ``model_max_drift_c_per_min``.

Bays within ``estimator.bay_settle_s`` of an occupancy change are left out of the last
two checks (a hot swap's transient is real, not a model error), and so are bays within
``bay_settle_s`` of a tick on which the estimator's own drive variance exceeded
:data:`SETTLE_DRIVE_VAR_C2` or its calibration floor ``sigma_cal`` changed (additions: a
drive pulled and another pushed in within ``empty_confirm_s`` never passes through
``empty``, the estimator's fast-swap rule follows it as a jump with a wide variance, and
an accepted or expired SMART calibration re-maps the drive estimate by up to the prior's
3 degC; on the truth simulator both tripped the prediction-error check and held the
fallback for 20 minutes). A failure switches to
the **PI-like DAS solver on the same estimates** (``solver_pi``, margin-deficit form):
``mode`` stays ``auto``, ``diagnostics["solver_diag"]["model"]`` says ``active: pi_das``
and why. The MPC comes back only after the checks have passed with the numeric limits
scaled by :data:`MODEL_HYSTERESIS` continuously for :data:`MODEL_DWELL_S` (so at least
that long after the fallback began). A cold solver starts on whichever model passes.
The PI-like DAS fallback regulates every drive at its soft target with the same margins,
so a model fallback changes loudness, not safety. (The plan's "no unknown bay whose
sigma exceeds sigma_fault_c" is a zone fault of the ``sigma`` trust rule, not a model
check, and is not evaluated here.)

Bumpless transfer
-----------------
Whenever a channel starts being driven by this solver (``initialise``: it has no
integrator entry, i.e. a cold start, a zone returning from fault, a manual override
released) and on every switch of the active model (all its channels), the solver
records a per-channel offset ``bias = prev - demand`` so the first output equals
``prev`` exactly; the offset then decays exponentially with :data:`BUMPLESS_TAU_UP_S`
when the demand is above ``prev`` (more cooling wanted: follow quickly) and
:data:`BUMPLESS_TAU_DOWN_S` when it is below (less cooling: follow slowly). The plan
describes a search for a disturbance shift on the terminal rows, like the legacy MPC's
``_bumpless_d``; with a noise objective whose optimum rides the constraints, the first
move is non-smooth in any such shift and a channel whose drives sit below their soft
targets has no row to shift, so an exact shift need not exist. The output offset is
exact, finite and decays deterministically (deviation, documented). The PI-like
fallback is bumpless by its own initialisation, and the offset carries only a band
snap of ``prev``. ``initialise`` computes the tick and stores it (``fresh``); the
following ``solve`` with that memory replays it, so both return exactly ``prev`` on
the new channels.

``MpcState.integrator`` holds the first block's command per driven channel (clamped
into the box) while the MPC is active, and the PI integrator while the fallback is.

Memory (``solver_memory["mpc"]``, plain JSON)::

    {"v": 1, "ts": last ts, "active": "mpc" | "pi_das" | None, "since": ts,
     "ok_since": ts | None, "reason": str | None, "tick": calls since the last solve,
     "key": structure key, "plan": [U] | None, "plan_ts": ts, "core": {ch: demand},
     "bias": {ch: offset}, "band": {ch: band index}, "pred": {"ts", "t": {bay: degC}},
     "err2": mean-square prediction error | None, "checks": {...}, "status": str,
     "fresh": {"pwm", "integrator", "iterations", "diagnostics"} | None}

A malformed memory starts over (cold: the next output is bumpless from ``prev``).
"""

from __future__ import annotations

import dataclasses
import json
import math
from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from typing import Any

import numpy as np

from aqua_bridge.control import noise as noise_model
from aqua_bridge.control import thermal
from aqua_bridge.control.solver_mpc import solve_box_qp
from aqua_bridge.control.solver_pi import PiSolver, SolverRequest, SolverResult
from aqua_bridge.model import MpcConfig

__all__ = [
    "ACCEPTED_STATUSES",
    "BUMPLESS_TAU_DOWN_S",
    "BUMPLESS_TAU_UP_S",
    "DIST_TAU_S",
    "EIG_IMAG_REL",
    "GAIN_MIN",
    "MODEL_DWELL_S",
    "MODEL_HYSTERESIS",
    "PRED_ERR_ALPHA",
    "PN_ITER",
    "QP_FAST_ITER",
    "PRIOR_STATUSES",
    "QP_STEP_TOL",
    "QUANT_U",
    "REL_DECREASE_TOL",
    "RIDGE",
    "SETTLE_CAL_STEP_C",
    "SQP_STEP_TOL",
    "SETTLE_DRIVE_VAR_C2",
    "DasMpcSolver",
    "ModelCheck",
    "PenaltyQp",
    "Prediction",
    "SqpResult",
    "build_prediction",
    "check_model",
    "gradient",
    "objective",
    "projected_newton",
    "snap_bands",
    "solve_penalty_qp",
    "zoh",
]

VERSION = 1

#: Thermal statuses the validity gate accepts; with ``model_accept_prior`` also these.
ACCEPTED_STATUSES: tuple[str, ...] = ("converged", "frozen")
PRIOR_STATUSES: tuple[str, ...] = ("prior", "learning", "off")
#: Model fallback: numeric limits scaled by this to come back, for this long.
MODEL_HYSTERESIS = 0.5
MODEL_DWELL_S = 300.0
#: EW factor of the prediction error's mean square.
PRED_ERR_ALPHA = 0.2
#: Quantisation of the command at which ``A`` is linearised (PWM).
QUANT_U = 1.0 / 64.0
#: Steady-state gain below this (degC per unit PWM) is no gain.
GAIN_MIN = 1e-6
#: An eigenvalue of ``A`` is real when its imaginary part is below this, relative.
EIG_IMAG_REL = 1e-6
_TAYLOR_ORDER = 12
#: Diagonal ridge on the Hessian, per variable.
RIDGE = 1e-9
#: Box QP: a Newton step below this (PWM) is zero; the rounding of ``H^-1 g`` on the
#: penalised Hessians (condition numbers up to ~1e7) is ~1e-10.
QP_STEP_TOL = 1e-9
#: Projected Newton warm-start iterations per box QP (:func:`projected_newton`), used when
#: the box QP from the current point does not finish within ``QP_FAST_ITER`` iterations.
PN_ITER = 12
QP_FAST_ITER = 8
#: SQP stops: relative decrease, largest move of an accepted step (PWM), halvings.
REL_DECREASE_TOL = 1e-8
SQP_STEP_TOL = 1e-4
_HALVINGS = 30
#: A bay whose estimator drive variance (``sigma^2 - sigma_cal^2``) exceeds this, degC^2, is
#: settling (module docstring, validity gate).
SETTLE_DRIVE_VAR_C2 = 1.0
#: A change of a bay's ``sigma_cal`` above this, degC, is a calibration event (settling).
SETTLE_CAL_STEP_C = 0.05
#: Low-pass of the estimator's disturbances for the prediction, seconds (module docstring).
DIST_TAU_S = 120.0
#: Bumpless offset decay, seconds (module docstring).
BUMPLESS_TAU_UP_S = 15.0
BUMPLESS_TAU_DOWN_S = 60.0
#: Pressure on a bound below this is not reported as demand.
_PRESSURE_TOL = 1e-9
#: Offsets below this are dropped.
_BIAS_EPS = 1e-9
#: A pending prediction older than this many ticks past its time is dropped.
_PRED_STALE_TICKS = 3.0
_LIN_CACHE_SIZE = 8

MPC = "mpc"
PI_DAS = "pi_das"


def _finite(value: object) -> bool:
    if type(value) is float:  # the common case first (hot path)
        return math.isfinite(value)
    return (
        isinstance(value, int | float)
        and not isinstance(value, bool)
        and math.isfinite(float(value))
    )


# ---------------------------------------------------------------------------
# the penalised box QP and its active-piece SQP
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class PenaltyQp:
    """``min 1/2 x^T h0 x + f0^T x + rho_soft sum (y - soft)_+^2 + rho_hard sum (y - hard)_+^2``
    with ``y = y0 + s x`` and ``lo <= x <= hi`` (``h0`` positive definite)."""

    h0: np.ndarray
    f0: np.ndarray
    s: np.ndarray
    y0: np.ndarray
    soft: np.ndarray
    hard: np.ndarray
    rho_soft: float
    rho_hard: float
    lo: np.ndarray
    hi: np.ndarray


def objective(qp: PenaltyQp, x: np.ndarray) -> float:
    """The true penalised objective at ``x``."""
    y = qp.y0 + qp.s @ x
    vs = np.maximum(y - qp.soft, 0.0)
    vh = np.maximum(y - qp.hard, 0.0)
    return float(
        0.5 * x @ (qp.h0 @ x) + qp.f0 @ x + qp.rho_soft * (vs @ vs) + qp.rho_hard * (vh @ vh)
    )


def gradient(qp: PenaltyQp, x: np.ndarray) -> np.ndarray:
    """Gradient of :func:`objective` (continuous: the penalties are C1)."""
    y = qp.y0 + qp.s @ x
    w = 2.0 * qp.rho_soft * np.maximum(y - qp.soft, 0.0) + 2.0 * qp.rho_hard * np.maximum(
        y - qp.hard, 0.0
    )
    return qp.h0 @ x + qp.f0 + qp.s.T @ w


def _activity(qp: PenaltyQp, x: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    y = qp.y0 + qp.s @ x
    return y > qp.soft, y > qp.hard


def _piece(qp: PenaltyQp, act_s: np.ndarray, act_h: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Hessian and linear term of the quadratic that equals the objective on the piece."""
    w = 2.0 * qp.rho_soft * act_s + 2.0 * qp.rho_hard * act_h
    lin = 2.0 * qp.rho_soft * act_s * (qp.y0 - qp.soft) + 2.0 * qp.rho_hard * act_h * (
        qp.y0 - qp.hard
    )
    idx = np.flatnonzero(w)
    if idx.size == 0:
        return qp.h0, qp.f0
    rows = qp.s[idx]
    h = qp.h0 + (rows.T * w[idx]) @ rows
    return 0.5 * (h + h.T), qp.f0 + rows.T @ lin[idx]


@dataclass(frozen=True)
class SqpResult:
    """What :func:`solve_penalty_qp` returns.

    ``g`` is the true gradient at ``x``, ``h_diag`` the diagonal of the last piece's
    Hessian, ``side`` -1 / +1 / 0 per variable at its lower / upper bound / free,
    ``history`` the objective after every accepted outer iteration (first: at the
    start point), ``iterations`` the largest box-QP iteration count (``warm_iterations`` the
    projected-Newton warm-start iterations, in total) and ``stop`` one of
    ``unchanged`` | ``small_decrease`` | ``small_step`` | ``no_descent`` | ``outer_max`` |
    ``qp_cap``.
    """

    x: np.ndarray
    g: np.ndarray
    h_diag: np.ndarray
    side: np.ndarray
    objective: float
    outer: int
    iterations: int
    total_iterations: int
    converged: bool
    stop: str
    history: tuple[float, ...]
    warm_iterations: int = 0


def projected_newton(
    h: np.ndarray, f: np.ndarray, lo: np.ndarray, hi: np.ndarray, x0: np.ndarray
) -> tuple[np.ndarray, int]:
    """Warm start for :func:`solve_box_qp` on ``min 1/2 x^T h x + f^T x``, ``lo <= x <= hi``:
    at most :data:`PN_ITER` projected Newton iterations (Bertsekas): the bounds whose
    gradient points outward are held, a Newton step on the rest, projected backtracking
    with an Armijo test. Returns ``(x, iterations)``; the cost never increases.

    ``solve_box_qp`` adds one bound per blocked step, so from a poor start on 48 variables
    it needed 40-70 iterations (more than the default ``solver_max_iter`` of 50) on the
    penalised pieces of a hot enclosure; from this warm start it finishes in 1-7.
    """
    x = np.clip(np.asarray(x0, dtype=float), lo, hi)
    cost = 0.5 * float(x @ (h @ x)) + float(f @ x)
    for k in range(PN_ITER):
        g = h @ x + f
        held = ((x <= lo) & (g > 0.0)) | ((x >= hi) & (g < 0.0))
        free = np.flatnonzero(~held)
        if free.size == 0:
            return x, k
        p = np.zeros_like(x)
        p[free] = np.linalg.solve(h[np.ix_(free, free)], -g[free])
        alpha = 1.0
        accepted = False
        x_new, cost_new = x, cost
        for _ in range(_HALVINGS):
            x_new = np.clip(x + alpha * p, lo, hi)
            cost_new = 0.5 * float(x_new @ (h @ x_new)) + float(f @ x_new)
            if cost_new <= cost + 1e-4 * float(g @ (x_new - x)):
                accepted = True
                break
            alpha *= 0.5
        if not accepted or float(np.max(np.abs(x_new - x))) <= QP_STEP_TOL:
            return x, k + 1
        x, cost = x_new, cost_new
    return x, PN_ITER


def solve_penalty_qp(
    qp: PenaltyQp, x0: np.ndarray, *, max_iter: int, outer_max: int, step_tol: float = 0.0
) -> SqpResult:
    """Active-piece SQP (module docstring): monotone in :func:`objective`, finite.

    ``step_tol`` > 0 also stops after an accepted step whose largest move is below it
    (``stop="small_step"``; the DAS MPC passes :data:`SQP_STEP_TOL`)."""
    x = np.clip(np.asarray(x0, dtype=float), qp.lo, qp.hi)
    fval = objective(qp, x)
    act_s, act_h = _activity(qp, x)
    history = [fval]
    it_max = 0
    it_total = 0
    pn_total = 0
    outer = 0
    stop = "outer_max"
    converged = True
    for outer in range(1, outer_max + 1):  # noqa: B007 - read after the loop
        h, f = _piece(qp, act_s, act_h)
        # Normalised by its Gershgorin bound (>= lambda_max), so the multiplier tolerance
        # is relative; the Newton step tolerance is in PWM (module docstring).
        lip = max(float(np.max(np.sum(np.abs(h), axis=1))), 1e-12)
        hn, fn = h / lip, f / lip
        # from the current point first (a warm plan is usually a few bound changes away);
        # only a start that needs more gets the projected Newton warm start
        res = solve_box_qp(
            hn, fn, qp.lo, qp.hi, x, 1.0, min(max_iter, QP_FAST_ITER), step_tol=QP_STEP_TOL
        )
        it_total += res.iterations
        if not res.converged and max_iter > QP_FAST_ITER:
            warm, pn_iter = projected_newton(hn, fn, qp.lo, qp.hi, x)
            pn_total += pn_iter
            res = solve_box_qp(hn, fn, qp.lo, qp.hi, warm, 1.0, max_iter, step_tol=QP_STEP_TOL)
            it_total += res.iterations
        it_max = max(it_max, res.iterations)
        if not res.converged:
            converged = False
            stop = "qp_cap"
            break
        d = res.x - x
        alpha = 1.0
        accepted = False
        x_new = x
        f_new = fval
        for _ in range(_HALVINGS):
            x_new = np.clip(x + alpha * d, qp.lo, qp.hi)
            f_new = objective(qp, x_new)
            if f_new <= fval:
                accepted = True
                break
            alpha *= 0.5
        if not accepted:
            stop = "no_descent"
            break
        new_s, new_h = _activity(qp, x_new)
        moved = float(np.max(np.abs(x_new - x))) if x.size else 0.0
        same = bool(np.array_equal(new_s, act_s) and np.array_equal(new_h, act_h))
        decrease = fval - f_new
        x, fval, act_s, act_h = x_new, f_new, new_s, new_h
        history.append(fval)
        if same and alpha == 1.0:
            stop = "unchanged"
            break
        if decrease <= REL_DECREASE_TOL * max(1.0, abs(fval)):
            stop = "small_decrease"
            break
        if step_tol > 0.0 and moved <= step_tol:
            stop = "small_step"
            break
    w = 2.0 * qp.rho_soft * act_s + 2.0 * qp.rho_hard * act_h
    h_diag = np.diag(qp.h0) + (qp.s * qp.s).T @ w
    side = np.zeros(x.shape[0], dtype=np.int8)
    side[x <= qp.lo] = -1
    side[x >= qp.hi] = 1
    return SqpResult(
        x=x,
        g=gradient(qp, x),
        h_diag=h_diag,
        side=side,
        objective=fval,
        outer=outer,
        iterations=it_max,
        total_iterations=it_total,
        converged=converged,
        stop=stop,
        history=tuple(history),
        warm_iterations=pn_total,
    )


# ---------------------------------------------------------------------------
# forbidden bands
# ---------------------------------------------------------------------------


def snap_bands(
    u: float,
    bands: Sequence[tuple[float, float]],
    held: int | None,
    hysteresis: float,
) -> tuple[float, int | None]:
    """A demand outside the forbidden bands (module docstring) and the band it is held at.

    ``held`` is the band the channel was snapped to on the previous tick (``None``: none).
    Inside a band (``lo <= u <= hi``) the output is the upper edge; a held channel
    stays at the edge while ``lo - hysteresis <= u <= hi``. Snapping repeats while the
    edge lies inside another band, so overlapping bands never leave the output in one.
    """
    u = float(u)
    out: float | None = None
    band: int | None = None
    if held is not None and 0 <= held < len(bands):
        lo, hi = bands[held]
        if lo - hysteresis <= u <= hi:
            out, band = float(hi), held
    if out is None:
        for i, (lo, hi) in enumerate(bands):
            if lo <= u <= hi:
                out, band = float(hi), i
                break
    if out is None:
        return u, None
    for _ in range(len(bands)):
        moved = False
        for i, (lo, hi) in enumerate(bands):
            if lo <= out < hi:
                out, band, moved = float(hi), i, True
        if not moved:
            break
    return out, band


# ---------------------------------------------------------------------------
# prediction model
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class Prediction:
    """The linearised, discretised, condensed model at one operating point.

    State ``x = [T_a (zones), T_d (drives)]``; ``drives`` are the bays with a drive
    state (occupied or unknown) in topology order. ``steps[k]`` is the sensitivity of
    the drives after step ``k + 1`` to the block variables (``n_b * m`` columns, block
    ``b`` channel ``i`` at ``b * m + i``); ``g_ss`` the steady-state gain of the drives
    on the channels and ``w_ss = C A^-1`` (drives x states) for the terminal rows.
    """

    zones: tuple[str, ...]
    drives: tuple[str, ...]
    channels: tuple[str, ...]
    blocks: tuple[int, ...]
    h: float
    a: np.ndarray
    b: np.ndarray
    ad: np.ndarray
    bd: np.ndarray
    m_int: np.ndarray
    eig_ok: bool
    eig_max: float
    steps: np.ndarray
    w_ss: np.ndarray | None
    g_ss: np.ndarray | None
    u_lin: tuple[float, ...] = ()

    @property
    def n(self) -> int:
        return len(self.zones) + len(self.drives)

    @property
    def n_vars(self) -> int:
        return len(self.blocks) * len(self.channels)


_GRAM_CACHE: dict[tuple[int, int], np.ndarray] = {}


def _move_gram(n_b: int, m: int) -> np.ndarray:
    """``D^T D`` of the block-difference operator ``(D U)_b = u_b - u_(b-1)`` (``u_(-1)`` = 0)."""
    gram = _GRAM_CACHE.get((n_b, m))
    if gram is None:
        n = n_b * m
        d = np.eye(n)
        if n_b > 1:
            d[m:, :-m] -= np.eye(n - m)
        gram = d.T @ d
        gram.setflags(write=False)
        _GRAM_CACHE[(n_b, m)] = gram
    return gram


@dataclass(frozen=True)
class _Dynamics:
    """What depends on the parameters and the quantised command only (the memo's value):
    the reduced ``A``, its discretisation and inverse, and the pieces of ``B`` that the
    state multiplies (``B`` is affine in the temperatures)."""

    a: np.ndarray
    ad: np.ndarray
    m_int: np.ndarray
    w_ss: np.ndarray | None
    eig_ok: bool
    eig_max: float
    dq: np.ndarray  # zones x channels: dQ_z / du
    dqn: np.ndarray  # zones x channels: dQn_z / du
    k: np.ndarray  # drives: k of the bay
    zone_of: np.ndarray  # drives: zone index
    incidence: np.ndarray  # zones x drives
    c_air: np.ndarray
    c_drive: np.ndarray


_DYN_CACHE: dict[tuple[Any, ...], _Dynamics] = {}


def _dynamics(
    st: thermal.Structure,
    params: thermal.ThermalParams,
    drives: tuple[str, ...],
    uq: np.ndarray,
    h: float,
) -> _Dynamics:
    zones = tuple(st.zones)
    x_any = np.zeros(st.n_states)  # A does not depend on the state
    lin = thermal.jacobians(st, params, x_any, uq, t_in=np.zeros(len(zones)))
    idx = [st.i_air(z) for z in zones] + [st.i_drive(b) for b in drives]
    a = lin.a[np.ix_(idx, idx)]
    n = a.shape[0]
    lam = np.linalg.eigvals(a) if n else np.zeros(0)
    eig_real = bool(np.all(np.abs(lam.imag) <= EIG_IMAG_REL * np.maximum(1.0, np.abs(lam))))
    eig_max = float(np.max(lam.real)) if n else -math.inf
    w_ss: np.ndarray | None
    inv: np.ndarray | None
    try:
        inv = np.linalg.inv(a)
        if not np.all(np.isfinite(inv)):
            inv = None
    except np.linalg.LinAlgError:
        inv = None
    w_ss = None if inv is None else inv[len(zones) :]
    ad, m_int = zoh(a, h, inv)
    dq, dqn = thermal.airflow_gradients(st, params, uq)
    zone_of = np.array([st.zones[st.bays[b].zone].index for b in drives], dtype=int)
    incidence = np.zeros((len(zones), len(drives)))
    incidence[zone_of, np.arange(len(drives))] = 1.0
    return _Dynamics(
        a=a,
        ad=ad,
        m_int=m_int,
        w_ss=w_ss,
        eig_ok=eig_real and eig_max < 0.0,
        eig_max=eig_max,
        dq=dq,
        dqn=dqn,
        k=np.array([params.theta[f"k.{b}"] for b in drives]),
        zone_of=zone_of,
        incidence=incidence,
        c_air=np.array([params.c_air[z] for z in zones]),
        c_drive=np.array([params.c_drive[b] for b in drives]),
    )


def _expm(m: np.ndarray) -> np.ndarray:
    """Matrix exponential by scaling and squaring a Taylor polynomial (numpy only)."""
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


def zoh(a: np.ndarray, h: float, inv: np.ndarray | None = None) -> tuple[np.ndarray, np.ndarray]:
    """``(exp(A h), int_0^h exp(A s) ds)``: the exact zero-order-hold discretisation of
    ``dx/dt = A x + v`` is ``x+ = Ad x + M v``.

    With ``inv = A^-1`` (the caller's, for the terminal rows) ``M = A^-1 (Ad - I)`` from one
    exponential of ``A h``; otherwise one exponential of ``[[A, I], [0, 0]] h``, which needs
    no inverse. ``thermal.discretise`` diagonalises instead; a layout with identical bays
    has repeated eigenvalues, for which ``numpy.linalg.eig`` may return eigenvectors whose
    real parts are singular, so the MPC uses the exponential, which needs no eigenvectors.
    """
    n = a.shape[0]
    if inv is not None:
        ad = _expm(a * h)
        m_int = inv @ (ad - np.eye(n))
        if np.all(np.isfinite(m_int)):
            return ad, m_int
    big = np.zeros((2 * n, 2 * n))
    big[:n, :n] = a * h
    big[:n, n:] = np.eye(n) * h
    e = _expm(big)
    return e[:n, :n], e[:n, n:]


def _u_lin(u: float, deadband: float, last: float | None = None) -> float:
    """Quantised command clamped into the open range of the fan curve; the previous
    linearisation command ``last`` is kept while ``u`` stays within one step of it."""
    q = round(float(u) / QUANT_U) * QUANT_U
    if last is not None and abs(float(u) - last) <= QUANT_U:
        q = last
    return min(1.0 - QUANT_U, max(deadband + QUANT_U, q))


def build_prediction(
    cfg: MpcConfig,
    st: thermal.Structure,
    params: thermal.ThermalParams,
    *,
    x_air: Mapping[str, float],
    x_drive: Mapping[str, float],
    u: Mapping[str, float],
    t_in: Mapping[str, float],
    u_lin: Mapping[str, float] | None = None,
) -> tuple[Prediction, bool]:
    """:class:`Prediction` at ``(x, u, t_in)``; ``(prediction, memo hit)``.

    ``A`` and the command-dependent pieces of ``B`` are taken at the quantised command
    and memoised on it (with the parameters; ``u_lin`` is the previous tick's linearisation
    command, kept per channel while the command stays within one quantisation step of it,
    so a fan dithering across a step boundary does not rebuild ``A``; the command used is
    ``Prediction.u_lin``); ``B`` itself is affine in the temperatures
    and evaluated at the exact state (the module docstring's quantisation of the
    temperatures is therefore not needed). Raises ``numpy.linalg.LinAlgError`` /
    ``ValueError`` on a numerical failure.
    """
    zones = tuple(st.zones)
    drives = tuple(b for b in st.bays if params.occupied[b])
    channels = tuple(st.channels)
    blocks = cfg.blocks()
    h = float(cfg.mpc_pred_dt_s)
    last = {} if u_lin is None else u_lin
    uq = tuple(_u_lin(u[ch], params.fan[ch][0], last.get(ch)) for ch in channels)
    key = (
        st.fingerprint,
        tuple(params.theta.items()),
        tuple(params.c_air.values()),
        tuple(params.c_drive.values()),
        tuple(params.fan.values()),
        drives,
        uq,
        h,
    )
    dyn = _DYN_CACHE.get(key)
    hit = dyn is not None
    if dyn is None:
        dyn = _dynamics(st, params, drives, np.array(uq), h)
        while len(_DYN_CACHE) >= _LIN_CACHE_SIZE:
            del _DYN_CACHE[next(iter(_DYN_CACHE))]
        _DYN_CACHE[key] = dyn
    xa = np.array([float(x_air[z]) for z in zones])
    xd = np.array([float(x_drive[b]) for b in drives])
    tin = np.array([float(t_in[z]) for z in zones])
    # thermal.jacobians: air rows -dQ (T_a - T_in) / C_a + sum_j k_j dQn (T_d,j - T_a) / C_a,
    # drive rows -k dQn (T_d - T_a) / C_d
    drive_gain = (dyn.k * (xd - xa[dyn.zone_of]))[:, None] * dyn.dqn[dyn.zone_of]
    b_air = (-dyn.dq * (xa - tin)[:, None] + dyn.incidence @ drive_gain) / dyn.c_air[:, None]
    b_drive = -drive_gain / dyn.c_drive[:, None]
    bm = np.vstack([b_air, b_drive])
    if not np.all(np.isfinite(bm)):
        raise FloatingPointError("non-finite input matrix")
    n, m = len(zones) + len(drives), len(channels)
    bd = dyn.m_int @ bm
    g_ss = None if dyn.w_ss is None else -(dyn.w_ss @ bm)
    n_b = len(blocks)
    nv = n_b * m
    steps = np.zeros((cfg.horizon, len(drives), nv))
    sens = np.zeros((n, nv))
    k = 0
    for bi, length in enumerate(blocks):
        cols = slice(bi * m, (bi + 1) * m)
        for _ in range(length):
            sens = dyn.ad @ sens
            sens[:, cols] += bd
            steps[k] = sens[len(zones) :]
            k += 1
    pred = Prediction(
        zones=zones,
        drives=drives,
        channels=channels,
        blocks=blocks,
        h=h,
        a=dyn.a,
        b=bm,
        ad=dyn.ad,
        bd=bd,
        m_int=dyn.m_int,
        eig_ok=dyn.eig_ok,
        eig_max=dyn.eig_max,
        steps=steps,
        w_ss=dyn.w_ss,
        g_ss=g_ss,
        u_lin=uq,
    )
    return pred, hit


# ---------------------------------------------------------------------------
# validity gate
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class ModelCheck:
    """Validity gate verdict: ``ok``, the first failing check (``reason``) and every check."""

    ok: bool
    reason: str | None
    checks: dict[str, Any]


def check_model(
    cfg: MpcConfig,
    *,
    status: str,
    theta: Mapping[str, float],
    pred: Prediction | None,
    rows: Sequence[str],
    pred_err_c: float | None,
    drift_c_per_min: float | None,
    relax: float = 1.0,
    error: str | None = None,
) -> ModelCheck:
    """The validity gate (module docstring). ``relax`` scales the numeric limits
    (:data:`MODEL_HYSTERESIS` while the fallback is active); ``error`` is a failure to
    build the model at all."""
    accepted = ACCEPTED_STATUSES + (PRIOR_STATUSES if cfg.model_accept_prior else ())
    checks: dict[str, Any] = {"status": status}
    reasons: list[str] = []
    if error is not None:
        reasons.append(f"model:{error}")
    if status not in accepted:
        reasons.append(f"status:{status}")
    in_bounds = True
    for key, value in theta.items():
        spec = thermal.PARAMETERS.get(key.split(".", 1)[0])
        if spec is None or not _finite(value) or not spec.lo <= float(value) <= spec.hi:
            in_bounds = False
            break
    checks["theta_in_bounds"] = in_bounds
    if not in_bounds:
        reasons.append("theta_out_of_bounds")
    if pred is not None:
        checks["eig_max"] = pred.eig_max if math.isfinite(pred.eig_max) else None
        if not pred.eig_ok or pred.w_ss is None or pred.g_ss is None:
            reasons.append("eigenvalues")
        else:
            no_gain = [
                b
                for b in rows
                if float(np.max(np.abs(pred.g_ss[pred.drives.index(b)]))) <= GAIN_MIN  # type: ignore[index]
            ]
            checks["no_gain"] = no_gain
            if no_gain:
                reasons.append(f"no_gain:{no_gain[0]}")
    elif error is None:
        reasons.append("model:unavailable")
    max_err = cfg.model_max_pred_err_c * relax
    checks["pred_err_c"] = pred_err_c
    checks["max_pred_err_c"] = max_err
    if pred_err_c is not None and pred_err_c > max_err:
        reasons.append(f"pred_err:{pred_err_c:.3g}")
    max_drift = cfg.model_max_drift_c_per_min * relax
    checks["drift_c_per_min"] = drift_c_per_min
    checks["max_drift_c_per_min"] = max_drift
    if drift_c_per_min is not None and drift_c_per_min > max_drift:
        reasons.append(f"drift:{drift_c_per_min:.3g}")
    return ModelCheck(ok=not reasons, reason=reasons[0] if reasons else None, checks=checks)


# ---------------------------------------------------------------------------
# memory
# ---------------------------------------------------------------------------


def _fresh_memory() -> dict[str, Any]:
    return {
        "v": VERSION,
        "ts": None,
        "active": None,
        "since": None,
        "ok_since": None,
        "reason": None,
        "tick": 0,
        "key": None,
        "plan": None,
        "plan_ts": None,
        "core": None,
        "bias": {},
        "band": {},
        "pred": None,
        "err2": None,
        "checks": {},
        "status": None,
        "fresh": None,
        "dist": {"q": {}, "d": {}, "since": {}},
        "settle": {},
        "cal": {},
        "lin_u": {},
    }


def _opt_num(value: object) -> float | None:
    if value is None:
        return None
    if not _finite(value):
        raise ValueError("number")
    return float(value)  # type: ignore[arg-type]


def _num_map(value: object, names: Sequence[str] | None = None) -> dict[str, float]:
    if not isinstance(value, Mapping):
        raise TypeError("map")
    out: dict[str, float] = {}
    for k, v in value.items():
        if not isinstance(k, str) or not _finite(v):
            raise ValueError("map entry")
        if names is None or k in names:
            out[k] = float(v)
    return out


def _parse_memory(raw: object, cfg: MpcConfig) -> dict[str, Any]:
    """The memory, or a fresh one when anything is malformed (module docstring)."""
    try:
        return _parse(raw, cfg)
    except (TypeError, ValueError, KeyError, AttributeError, IndexError):
        return _fresh_memory()


def _parse(raw: object, cfg: MpcConfig) -> dict[str, Any]:
    if not isinstance(raw, Mapping) or raw.get("v") != VERSION:
        raise ValueError("version")
    mem = _fresh_memory()
    mem["ts"] = _opt_num(raw.get("ts"))
    active = raw.get("active")
    if active not in (None, MPC, PI_DAS):
        raise ValueError("active")
    mem["active"] = active
    mem["since"] = _opt_num(raw.get("since"))
    mem["ok_since"] = _opt_num(raw.get("ok_since"))
    reason = raw.get("reason")
    mem["reason"] = reason if isinstance(reason, str) else None
    tick = raw.get("tick", 0)
    if isinstance(tick, bool) or not isinstance(tick, int) or tick < 0:
        raise ValueError("tick")
    mem["tick"] = tick
    key = raw.get("key")
    mem["key"] = key if isinstance(key, str) else None
    channels = cfg.channels
    plan = raw.get("plan")
    if plan is not None:
        n = len(cfg.blocks()) * len(channels)
        if not isinstance(plan, list) or len(plan) != n or not all(_finite(v) for v in plan):
            raise ValueError("plan")
        mem["plan"] = [float(v) for v in plan]
    mem["plan_ts"] = _opt_num(raw.get("plan_ts"))
    core = raw.get("core")
    mem["core"] = None if core is None else _num_map(core, channels)
    mem["bias"] = _num_map(raw.get("bias", {}), channels)
    band = raw.get("band", {})
    if not isinstance(band, Mapping):
        raise TypeError("band")
    mem["band"] = {
        k: v
        for k, v in band.items()
        if k in channels and isinstance(v, int) and not isinstance(v, bool) and v >= 0
    }
    pred = raw.get("pred")
    if pred is not None:
        mem["pred"] = {"ts": float(_opt_num(pred["ts"])), "t": _num_map(pred["t"])}  # type: ignore[arg-type]
    err2 = _opt_num(raw.get("err2"))
    mem["err2"] = None if err2 is None or err2 < 0 else err2
    checks = raw.get("checks", {})
    mem["checks"] = dict(checks) if isinstance(checks, Mapping) else {}
    status = raw.get("status")
    mem["status"] = status if isinstance(status, str) else None
    mem["settle"] = _num_map(raw.get("settle", {}))
    mem["cal"] = _num_map(raw.get("cal", {}))
    mem["lin_u"] = _num_map(raw.get("lin_u", {}), channels)
    dist = raw.get("dist")
    if dist is not None:
        since = dist.get("since", {})
        if not isinstance(since, Mapping):
            raise TypeError("dist.since")
        mem["dist"] = {
            "q": _num_map(dist["q"]),
            "d": _num_map(dist["d"]),
            "since": {k: _opt_num(v) for k, v in since.items() if isinstance(k, str)},
        }
    fresh = raw.get("fresh")
    if fresh is not None:
        pwm = _num_map(fresh["pwm"])
        integ = _num_map(fresh["integrator"])
        iterations = fresh["iterations"]
        if (
            set(pwm) != set(channels)
            or isinstance(iterations, bool)
            or not isinstance(iterations, int)
        ):
            raise ValueError("fresh")
        diag = fresh.get("diagnostics", {})
        mem["fresh"] = {
            "pwm": pwm,
            "integrator": integ,
            "iterations": iterations,
            "diagnostics": dict(diag) if isinstance(diag, Mapping) else {},
        }
    return mem


# ---------------------------------------------------------------------------
# the solver
# ---------------------------------------------------------------------------


@dataclass
class _Model:
    """The per-tick model inputs and, when it could be built, the prediction."""

    st: thermal.Structure
    status: str
    theta: dict[str, float]
    rows: list[str]
    x_air: dict[str, float]
    x_drive: dict[str, float]
    pred: Prediction | None = None
    hit: bool | None = None
    c: np.ndarray | None = None
    x0: np.ndarray | None = None
    drift: float | None = None
    error: str | None = None


class DasMpcSolver:
    """The DAS MPC on the ``Solver`` protocol (module docstring)."""

    name = MPC

    def __init__(self) -> None:
        self._pi = PiSolver()

    # -- protocol --------------------------------------------------------------

    def initialise(
        self, cfg: MpcConfig, req: SolverRequest
    ) -> tuple[dict[str, float], dict[str, Any]]:
        result, memory = self._tick(cfg, req)
        if not result.converged:
            raise RuntimeError(
                f"DAS MPC initialisation: a box QP hit solver_max_iter ({cfg.solver_max_iter})"
            )
        memory["fresh"] = {
            "pwm": dict(result.pwm),
            "integrator": dict(result.integrator),
            "iterations": result.iterations,
            "diagnostics": dict(result.diagnostics),
        }
        return dict(result.integrator), memory

    def solve(self, cfg: MpcConfig, req: SolverRequest) -> SolverResult:
        mem = _parse_memory(req.memory, cfg)
        fresh = mem["fresh"]
        if fresh is not None:
            mem["fresh"] = None
            return SolverResult(
                pwm=dict(fresh["pwm"]),
                integrator=dict(fresh["integrator"]),
                memory=mem,
                converged=True,
                iterations=fresh["iterations"],
                diagnostics={**fresh["diagnostics"], "replayed": True},
            )
        result, memory = self._tick(cfg, req, mem)
        return dataclasses.replace(result, memory=memory)

    # -- one tick -----------------------------------------------------------------

    def _tick(
        self, cfg: MpcConfig, req: SolverRequest, parsed: dict[str, Any] | None = None
    ) -> tuple[SolverResult, dict[str, Any]]:
        if not cfg.regulates_drive_limits:
            raise ValueError("the DAS MPC needs a zoned config without setpoints")
        mem = _parse_memory(req.memory, cfg) if parsed is None else parsed
        mem["fresh"] = None
        prev = {ch: float(req.prev_pwm[ch]) for ch in cfg.channels}
        ts = self._clock(cfg, req, mem)
        fixed = {ch: float(v) for ch, v in req.fixed_channels.items()}
        free = [ch for ch in cfg.channels if ch not in fixed]
        self._decay_bias(mem, ts, free)
        settling = self._settling_bays(cfg, req, mem, ts)
        self._score_prediction(cfg, req, mem, ts, settling)
        self._filter_disturbances(req, mem, ts)

        rows = self._rows(cfg, req)
        key = json.dumps(
            [sorted(fixed), sorted(z for z, ok in req.zone_trust.items() if ok), rows],
            separators=(",", ":"),
        )
        solve_now = (
            mem["active"] is None
            or mem["key"] != key
            or mem["tick"] + 1 >= cfg.mpc_every_ticks
            or (mem["active"] == MPC and (mem["core"] is None or mem["plan"] is None))
            or (mem["active"] == MPC and self._block_index(cfg, mem, ts) is None)
        )
        switched = False
        model: _Model | None = None
        if solve_now:
            model = self._model(cfg, req, rows, mem["dist"], settling, mem["lin_u"])
            if model.pred is not None:
                mem["lin_u"] = dict(zip(model.pred.channels, model.pred.u_lin, strict=True))
            self._store_prediction(mem, model, prev, ts)
            switched = self._decide(cfg, mem, model, ts)
            mem["tick"] = 0
            mem["key"] = key
        else:
            mem["tick"] += 1

        diag: dict[str, Any] = {"form": "das_mpc", "solved": False}
        if mem["active"] == MPC:
            if solve_now:
                assert model is not None
                sqp_diag = self._solve_mpc(cfg, req, mem, model, prev, fixed, free, ts)
                if sqp_diag.get("converged") is False:
                    return self._not_converged(cfg, req, mem, prev, sqp_diag)
                diag.update(sqp_diag)
            core = self._replay_core(cfg, mem, ts, free)
            iterations = int(diag.get("iterations", 1))
            integrator = {ch: min(cfg.pwm_max, max(cfg.pwm_min, core[ch])) for ch in free}
        else:
            pi_req = dataclasses.replace(
                req, integrator={} if switched else dict(req.integrator), memory={}
            )
            pi_res = self._pi.solve(cfg, pi_req)
            core = {ch: float(pi_res.pwm[ch]) for ch in free}
            iterations = 1
            integrator = {ch: float(v) for ch, v in pi_res.integrator.items() if ch in free}
            diag["pi"] = pi_res.diagnostics
            mem["core"] = None
            mem["plan"] = None

        # bands, then the bumpless offset of new channels / a model switch
        hysteresis = cfg.noise.band_hysteresis if cfg.noise is not None else 0.0
        snapped: dict[str, float] = {}
        bands_out: dict[str, int] = {}
        for ch in free:
            held = mem["band"].get(ch)
            out, band = snap_bands(core[ch], cfg.fans[ch].forbidden_pwm, held, hysteresis)
            snapped[ch] = out
            if band is not None:
                bands_out[ch] = band
        mem["band"] = bands_out
        new = [ch for ch in free if switched or ch not in req.integrator]
        bias = {ch: v for ch, v in mem["bias"].items() if ch in free}
        for ch in new:
            bias[ch] = prev[ch] - snapped[ch]
        mem["bias"] = {ch: v for ch, v in bias.items() if abs(v) > _BIAS_EPS or ch in new}
        pwm = {ch: fixed[ch] for ch in fixed}
        for ch in free:
            pwm[ch] = snapped[ch] + mem["bias"].get(ch, 0.0)
        for ch in new:  # exactly prev, not prev up to the rounding of (prev - x) + x
            pwm[ch] = prev[ch]

        diag["model"] = {
            "active": PI_DAS if mem["active"] == PI_DAS else MPC,
            "reason": mem["reason"],
            "since_ts": mem["since"],
            "status": mem["status"],
            "checks": mem["checks"],
        }
        diag["demand"] = dict(core)
        diag["bias"] = dict(mem["bias"])
        diag["band"] = {ch: list(cfg.fans[ch].forbidden_pwm[i]) for ch, i in bands_out.items()}
        diag["pred_err_c"] = None if mem["err2"] is None else math.sqrt(mem["err2"])
        diag["tick"] = mem["tick"]
        mem["ts"] = ts
        result = SolverResult(
            pwm=pwm,
            integrator=integrator,
            memory={},
            converged=True,
            iterations=iterations,
            diagnostics=diag,
        )
        return result, mem

    # -- pieces ---------------------------------------------------------------------

    @staticmethod
    def _clock(cfg: MpcConfig, req: SolverRequest, mem: Mapping[str, Any]) -> float:
        if _finite(req.ts):
            return float(req.ts)  # type: ignore[arg-type]
        last = mem["ts"]
        return 0.0 if last is None else last + cfg.dt

    @staticmethod
    def _decay_bias(mem: dict[str, Any], ts: float, free: list[str]) -> None:
        last = mem["ts"]
        elapsed = 0.0 if last is None else min(max(ts - last, 0.0), 3600.0)
        out: dict[str, float] = {}
        for ch, v in mem["bias"].items():
            if ch not in free:
                continue
            # bias > 0: the demand is below prev (less cooling) -> slow decay
            tau = BUMPLESS_TAU_DOWN_S if v > 0 else BUMPLESS_TAU_UP_S
            decayed = v * math.exp(-elapsed / tau)
            if abs(decayed) > _BIAS_EPS:
                out[ch] = decayed
        mem["bias"] = out

    @staticmethod
    def _rows(cfg: MpcConfig, req: SolverRequest) -> list[str]:
        """Bays with a row: an estimate, a trusted zone, not empty (topology order)."""
        assert cfg.topology is not None
        out = []
        for bay, spec in cfg.topology.bays.items():
            est = req.estimates.get(bay)
            if est is None or not req.zone_trust.get(spec.zone):
                continue
            if req.occupancy.get(bay) == "empty":
                continue
            out.append(bay)
        return out

    @staticmethod
    def _score_prediction(
        cfg: MpcConfig, req: SolverRequest, mem: dict[str, Any], ts: float, settling: set[str]
    ) -> None:
        """Compare a pending one-step prediction with the estimator's drives when due."""
        pending = mem["pred"]
        if pending is None:
            return
        due = pending["ts"]
        if ts < due - 0.5 * cfg.dt:
            return
        mem["pred"] = None
        if ts > due + _PRED_STALE_TICKS * cfg.dt:
            return
        bays = req.plant.get("bays", {}) if isinstance(req.plant, Mapping) else {}
        errs = []
        for bay, predicted in pending["t"].items():
            info = bays.get(bay)
            if not isinstance(info, Mapping) or not _finite(info.get("t")):
                continue
            if bay in settling:
                continue
            errs.append(abs(float(info["t"]) - predicted))
        if not errs:
            return
        e2 = max(errs) ** 2
        mem["err2"] = (
            e2
            if mem["err2"] is None
            else (1 - PRED_ERR_ALPHA) * mem["err2"] + (PRED_ERR_ALPHA * e2)
        )

    @staticmethod
    def _filter_disturbances(req: SolverRequest, mem: dict[str, Any], ts: float) -> None:
        """Low-pass the estimator's integrating disturbances for the prediction (module
        docstring, *Disturbances*) with :data:`DIST_TAU_S`; a bay whose occupancy changed
        restarts from the estimate."""
        plant = req.plant if isinstance(req.plant, Mapping) else {}
        last = mem["ts"]
        elapsed = 0.0 if last is None else min(max(ts - last, 0.0), 3600.0)
        old = mem["dist"]
        new: dict[str, Any] = {"q": {}, "d": {}, "since": {}}

        def smooth(prev: float | None, value: float) -> float:
            if prev is None:
                return value
            return prev + (value - prev) * (1.0 - math.exp(-elapsed / DIST_TAU_S))

        for zone, info in (plant.get("zones") or {}).items():
            if isinstance(info, Mapping) and _finite(info.get("d_air")):
                new["d"][zone] = smooth(old["d"].get(zone), float(info["d_air"]))
        for bay, info in (plant.get("bays") or {}).items():
            if not isinstance(info, Mapping) or not _finite(info.get("q_w")):
                continue
            since = float(info["since_ts"]) if _finite(info.get("since_ts")) else None
            prev = old["q"].get(bay) if old["since"].get(bay) == since else None
            new["q"][bay] = smooth(prev, float(info["q_w"]))
            new["since"][bay] = since
        mem["dist"] = new

    @staticmethod
    def _store_prediction(
        mem: dict[str, Any], model: _Model, prev: Mapping[str, float], ts: float
    ) -> None:
        """On a solve tick without a pending prediction: the drives one prediction step
        ahead with the command held at ``prev`` (module docstring, validity gate)."""
        pred = model.pred
        if mem["pred"] is not None or pred is None or model.c is None or model.x0 is None:
            return
        uv = np.array([min(1.0, max(0.0, float(prev[ch]))) for ch in pred.channels])
        x1 = pred.ad @ model.x0 + pred.bd @ uv + pred.m_int @ model.c
        n_z = len(pred.zones)
        if not np.all(np.isfinite(x1)):
            return
        mem["pred"] = {
            "ts": ts + pred.h,
            "t": {b: float(x1[n_z + i]) for i, b in enumerate(pred.drives)},
        }

    @staticmethod
    def _settling_bays(
        cfg: MpcConfig, req: SolverRequest, mem: dict[str, Any], ts: float
    ) -> set[str]:
        """Bays left out of the prediction-error and drift checks (module docstring): within
        ``estimator.bay_settle_s`` of an occupancy change, or of the last tick on which the
        estimator's own drive uncertainty exceeded :data:`SETTLE_DRIVE_VAR_C2` (a swap it
        followed as a jump without passing through ``empty``) or its calibration floor
        ``sigma_cal`` changed (a SMART calibration accepted or expired re-maps the drive
        estimate). Updates ``mem["settle"]`` and ``mem["cal"]``."""
        assert cfg.estimator is not None
        window = cfg.estimator.bay_settle_s
        bays = req.plant.get("bays", {}) if isinstance(req.plant, Mapping) else {}
        marks = {b: t for b, t in mem["settle"].items() if b in bays and 0.0 <= ts - t < window}
        cal_seen: dict[str, float] = {}
        out: set[str] = set()
        for bay, info in bays.items():
            if not isinstance(info, Mapping):
                continue
            sigma, cal = info.get("sigma"), info.get("sigma_cal")
            if _finite(sigma) and _finite(cal):
                cal_f = float(cal)
                if float(sigma) ** 2 - cal_f**2 > SETTLE_DRIVE_VAR_C2:
                    marks[bay] = ts
                last_cal = mem["cal"].get(bay)
                if last_cal is not None and abs(cal_f - last_cal) > SETTLE_CAL_STEP_C:
                    marks[bay] = ts
                cal_seen[bay] = cal_f
            since = info.get("since_ts")
            if (_finite(since) and 0.0 <= ts - float(since) < window) or bay in marks:
                out.add(bay)
        mem["settle"] = marks
        mem["cal"] = cal_seen
        return out

    def _model(
        self,
        cfg: MpcConfig,
        req: SolverRequest,
        rows: list[str],
        dist: Mapping[str, Any],
        settling: set[str],
        lin_u: Mapping[str, float] | None = None,
    ) -> _Model:
        """Model inputs from the request and, when possible, the prediction."""
        st = thermal.cached_structure(cfg)
        try:
            status, theta = thermal.current_model(req.thermal, cfg)
        except Exception as exc:  # a thermal memory that cannot be read is no model
            status, theta = "error", thermal.prior_theta(cfg, st)
            model = _Model(st, status, theta, rows, {}, {}, error=f"{type(exc).__name__}")
            return model
        model = _Model(st, status, theta, rows, {}, {})
        try:
            self._build(cfg, req, model, dist, settling, lin_u)
        except Exception as exc:  # numerical failure of the model: a model fallback
            model.pred = None
            model.error = f"{type(exc).__name__}: {exc}"[:120]
        return model

    @staticmethod
    def _build(
        cfg: MpcConfig,
        req: SolverRequest,
        model: _Model,
        dist: Mapping[str, Any],
        settling: set[str],
        lin_u: Mapping[str, float] | None = None,
    ) -> None:
        st = model.st
        plant = req.plant if isinstance(req.plant, Mapping) else {}
        zones_in = plant.get("zones", {})
        bays_in = plant.get("bays", {})
        known_air = {
            z: float(v["t_air"])
            for z, v in zones_in.items()
            if isinstance(v, Mapping) and _finite(v.get("t_air")) and z in st.zones
        }
        if not known_air:
            raise ValueError("no estimator state")
        hottest = max(known_air.values())
        x_air: dict[str, float] = {}
        t_in: dict[str, float] = {}
        d_air: dict[str, float] = {}
        for z in st.zones:
            info = zones_in.get(z) if isinstance(zones_in.get(z), Mapping) else {}
            # a zone the estimator has not initialised: the hottest known air (conservative)
            x_air[z] = known_air.get(z, hottest)
            t_in[z] = float(info["t_in"]) if _finite(info.get("t_in")) else x_air[z]
            d_air[z] = float(dist["d"].get(z, 0.0))
        occupancy: dict[str, str] = {}
        classes: dict[str, str] = {}
        for b in st.bays:
            info = bays_in.get(b) if isinstance(bays_in.get(b), Mapping) else {}
            occ = info.get("occupancy") or req.occupancy.get(b)
            if isinstance(occ, str):
                occupancy[b] = occ
            if isinstance(info.get("class"), str) and info["class"] in cfg.drive_classes:
                classes[b] = info["class"]
        params = thermal.model_params(cfg, model.theta, st=st, occupancy=occupancy, classes=classes)
        x_drive: dict[str, float] = {}
        q: dict[str, float] = {}
        for b, bs in st.bays.items():
            if not params.occupied[b]:
                continue
            info = bays_in.get(b) if isinstance(bays_in.get(b), Mapping) else {}
            x_drive[b] = float(info["t"]) if _finite(info.get("t")) else x_air[bs.zone]
            q[b] = float(dist["q"].get(b, 0.0)) / params.c_drive[b]
        missing = [b for b in model.rows if b not in x_drive]
        if missing:
            raise ValueError(f"no drive state for bay {missing[0]!r}")
        u_now = {ch: min(1.0, max(0.0, float(req.prev_pwm[ch]))) for ch in cfg.channels}
        pred, hit = build_prediction(
            cfg, st, params, x_air=x_air, x_drive=x_drive, u=u_now, t_in=t_in, u_lin=lin_u
        )
        x_full = np.zeros(st.n_states)
        for z in st.zones:
            x_full[st.i_air(z)] = x_air[z]
        for b, bs in st.bays.items():
            x_full[st.i_drive(b)] = x_drive.get(b, x_air[bs.zone])
            x_full[st.i_sensor(b)] = x_air[bs.zone]
        f = thermal.derivatives(st, params, x_full, u_now, t_in=t_in, q=q, d_air=d_air)
        idx = [st.i_air(z) for z in pred.zones] + [st.i_drive(b) for b in pred.drives]
        x0 = x_full[idx]
        uv = np.array([u_now[ch] for ch in pred.channels])
        c = f[idx] - pred.a @ x0 - pred.b @ uv
        if not (np.all(np.isfinite(c)) and np.all(np.isfinite(x0))):
            raise FloatingPointError("non-finite operating point")
        model.pred, model.hit, model.c, model.x0 = pred, hit, c, x0
        model.x_air, model.x_drive = x_air, x_drive
        drifts = [abs(float(f[st.i_drive(b)])) * 60.0 for b in model.rows if b not in settling]
        model.drift = max(drifts) if drifts else None

    def _decide(self, cfg: MpcConfig, mem: dict[str, Any], model: _Model, ts: float) -> bool:
        """Run the validity gate and switch the active model; ``True`` on a switch."""
        in_fallback = mem["active"] == PI_DAS
        err = None if mem["err2"] is None else math.sqrt(mem["err2"])
        verdict = check_model(
            cfg,
            status=model.status,
            theta=model.theta,
            pred=model.pred,
            rows=model.rows,
            pred_err_c=err,
            drift_c_per_min=model.drift,
            relax=MODEL_HYSTERESIS if in_fallback else 1.0,
            error=model.error,
        )
        checks = dict(verdict.checks)
        checks["cache_hit"] = model.hit
        mem["checks"] = checks
        mem["status"] = model.status
        before = mem["active"]
        if before is None:
            mem["active"] = MPC if verdict.ok else PI_DAS
            mem["since"] = ts
            mem["ok_since"] = ts if verdict.ok else None
            mem["reason"] = verdict.reason
            return True
        if before == MPC:
            if verdict.ok:
                mem["reason"] = None
                return False
            mem.update(active=PI_DAS, since=ts, ok_since=None, reason=verdict.reason)
            return True
        # fallback active: come back after passing the relaxed checks for the dwell
        if not verdict.ok:
            mem["ok_since"] = None
            mem["reason"] = verdict.reason
            return False
        if mem["ok_since"] is None:
            mem["ok_since"] = ts
        since = mem["since"] if mem["since"] is not None else ts
        if ts - mem["ok_since"] >= MODEL_DWELL_S and ts - since >= MODEL_DWELL_S:
            mem.update(active=MPC, since=ts, reason=None)
            return True
        mem["reason"] = "dwell"
        return False

    def _solve_mpc(
        self,
        cfg: MpcConfig,
        req: SolverRequest,
        mem: dict[str, Any],
        model: _Model,
        prev: Mapping[str, float],
        fixed: Mapping[str, float],
        free: list[str],
        ts: float,
    ) -> dict[str, Any]:
        pred = model.pred
        assert pred is not None and model.c is not None and model.x0 is not None
        assert pred.w_ss is not None and pred.g_ss is not None
        channels = pred.channels
        m = len(channels)
        blocks = pred.blocks
        n_b = len(blocks)
        nv = n_b * m
        n_z = len(pred.zones)
        rows = model.rows
        ridx = [pred.drives.index(b) for b in rows]
        n_r = len(rows)
        horizon = cfg.horizon

        uv = np.array([min(1.0, max(0.0, prev[ch])) for ch in channels])
        cd = pred.m_int @ model.c

        # free response and the row matrices
        y0_steps = np.zeros((horizon, n_r))
        x = model.x0.copy()
        for k in range(horizon):
            x = pred.ad @ x + cd
            y0_steps[k] = x[n_z:][ridx]
        s_rows = pred.steps[:, ridx, :].reshape(horizon * n_r, nv)
        term = np.zeros((n_r, nv))
        term[:, (n_b - 1) * m :] = pred.g_ss[ridx]
        s_all = np.vstack([s_rows, term])
        y0 = np.concatenate([y0_steps.reshape(-1), -(pred.w_ss[ridx] @ model.c)])
        soft1 = np.array([float(req.estimates[b]["soft"]) for b in rows])
        hard1 = np.array([float(req.estimates[b]["hard"]) for b in rows])
        soft = np.tile(soft1, horizon + 1)
        hard = np.tile(hard1, horizon + 1)

        # noise surrogate and moves over every variable
        sur = noise_model.surrogate(cfg, prev)
        assert cfg.noise is not None
        coef = cfg.noise.weight_noise * np.repeat(np.array(blocks, dtype=float), m)
        g_n = np.tile(np.array([sur.g[ch] for ch in channels]), n_b)
        h_n = np.tile(np.array([sur.h[ch] for ch in channels]), n_b)
        u_n = np.tile(np.array([sur.u_now[ch] for ch in channels]), n_b)
        h0 = np.diag(coef * h_n + RIDGE)
        f0 = coef * (g_n - h_n * u_n)
        wd = cfg.weight_dpwm
        if wd > 0:  # weight_dpwm sum_b ||u_b - u_(b-1)||^2 with u_(-1) = prev
            h0 += 2.0 * wd * _move_gram(n_b, m)
            f0[:m] -= 2.0 * wd * np.array([prev[ch] for ch in channels])

        # remove the fixed channels
        col_free = np.array(
            [bi * m + i for bi in range(n_b) for i, ch in enumerate(channels) if ch not in fixed],
            dtype=int,
        )
        col_fixed = np.array(
            [bi * m + i for bi in range(n_b) for i, ch in enumerate(channels) if ch in fixed],
            dtype=int,
        )
        x_fixed = np.array(
            [min(cfg.pwm_max, max(cfg.pwm_min, fixed[channels[j % m]])) for j in col_fixed]
        )
        if col_fixed.size:
            f_red = f0[col_free] + h0[np.ix_(col_free, col_fixed)] @ x_fixed
            y0 = y0 + s_all[:, col_fixed] @ x_fixed
        else:
            f_red = f0[col_free]
        qp = PenaltyQp(
            h0=h0[np.ix_(col_free, col_free)],
            f0=f_red,
            s=s_all[:, col_free],
            y0=y0,
            soft=soft,
            hard=hard,
            rho_soft=cfg.rho_soft,
            rho_hard=cfg.rho_hard,
            lo=np.full(col_free.size, cfg.pwm_min),
            hi=np.full(col_free.size, cfg.pwm_max),
        )
        plan = mem["plan"]
        if plan is not None and len(plan) == nv:
            warm = np.array(plan)[col_free]
        else:
            warm = np.tile(np.clip(uv, cfg.pwm_min, cfg.pwm_max), n_b)[col_free]
        res = solve_penalty_qp(
            qp,
            warm,
            max_iter=cfg.solver_max_iter,
            outer_max=cfg.solver_outer_max,
            step_tol=SQP_STEP_TOL,
        )
        if not res.converged:
            return {"converged": False, "iterations": res.iterations, "stop": res.stop}

        full = np.zeros(nv)
        full[col_free] = res.x
        if col_fixed.size:
            full[col_fixed] = x_fixed
        core: dict[str, float] = {}
        pressure: dict[str, float] = {}
        pos = {int(c): k for k, c in enumerate(col_free)}
        for i, ch in enumerate(channels):
            if ch in fixed:
                continue
            k = pos[i]  # block 0 variable of the channel
            u0 = float(res.x[k])
            g = float(res.g[k])
            push = 0.0
            if (res.side[k] > 0 and g < -_PRESSURE_TOL) or (res.side[k] < 0 and g > _PRESSURE_TOL):
                push = -g / float(res.h_diag[k])
            core[ch] = u0 + push
            pressure[ch] = push
        mem["plan"] = [float(v) for v in full]
        mem["plan_ts"] = ts
        mem["core"] = core

        y = qp.y0 + qp.s @ res.x
        worst_bay: str | None = None
        worst_margin: float | None = None
        if n_r:
            margins = (hard - y).reshape(horizon + 1, n_r)
            j = int(np.argmin(margins.min(axis=0)))
            worst_bay, worst_margin = rows[j], float(margins[:, j].min())
        return {
            "solved": True,
            "converged": True,
            "iterations": res.iterations,
            "iterations_total": res.total_iterations,
            "warm_iterations": res.warm_iterations,
            "outer": res.outer,
            "stop": res.stop,
            "objective": res.objective,
            "rows": int(y.size),
            "rows_over_soft": int(np.count_nonzero(y > soft)),
            "rows_over_hard": int(np.count_nonzero(y > hard)),
            "pressure": pressure,
            "worst_bay": worst_bay,
            "predicted_hard_margin_c": worst_margin,
            "drift_c_per_min": model.drift,
            "cache_hit": model.hit,
        }

    @staticmethod
    def _block_index(cfg: MpcConfig, mem: Mapping[str, Any], ts: float) -> int | None:
        """Block of the stored plan that holds at ``ts`` (``None``: none left)."""
        plan_ts = mem["plan_ts"]
        if plan_ts is None:
            return None
        step = int(max(ts - plan_ts, 0.0) // cfg.mpc_pred_dt_s)
        total = 0
        for bi, length in enumerate(cfg.blocks()):
            total += length
            if step < total:
                return bi
        return None

    def _replay_core(
        self, cfg: MpcConfig, mem: dict[str, Any], ts: float, free: list[str]
    ) -> dict[str, float]:
        bi = self._block_index(cfg, mem, ts)
        core = mem["core"]
        assert core is not None
        if bi in (None, 0):
            return {ch: float(core[ch]) for ch in free}
        m = len(cfg.channels)
        plan = mem["plan"]
        return {ch: float(plan[bi * m + cfg.channels.index(ch)]) for ch in free}

    @staticmethod
    def _not_converged(
        cfg: MpcConfig,
        req: SolverRequest,
        mem: dict[str, Any],
        prev: Mapping[str, float],
        diag: Mapping[str, Any],
    ) -> tuple[SolverResult, dict[str, Any]]:
        """A box QP hit ``solver_max_iter``: ``converged=False`` (a solver fault in ``step``)."""
        mem["plan"] = None
        mem["core"] = None
        pwm = {ch: float(prev[ch]) for ch in cfg.channels}
        pwm.update({ch: float(v) for ch, v in req.fixed_channels.items()})
        return (
            SolverResult(
                pwm=pwm,
                integrator={},
                memory={},
                converged=False,
                iterations=int(diag.get("iterations", cfg.solver_max_iter)),
                diagnostics=dict(diag),
            ),
            mem,
        )
