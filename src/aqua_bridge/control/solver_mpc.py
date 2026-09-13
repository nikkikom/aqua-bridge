"""Small linear MPC behind the :class:`~aqua_bridge.control.solver_pi.Solver` protocol.

PROJECT.md section 3 (Track A: "2-4 temperatures, 4-8 PWM, dt 1-2 s, horizon
10-20 s, numpy, no CasADi/IPOPT") and section 8 ("replace PI with a small
linear MPC (numpy); keep the same tests"). ``mpc.step`` still owns the gate,
the fault timer, the fallback policy, the rate limit and the clamp; this
module only turns trusted temperatures into a per-channel *demand*.

Plant model (per controlled temperature ``j``, i.e. every ``cfg.temps`` entry
with a setpoint, and per fan channel ``i``)::

    T_j[k+1] = a * T_j[k] + sum_i B_ji * u_i[k] + d_j
    a        = exp(-dt / mpc_tau_s)
    B_ji     = -(1 - a) * mpc_gain_c_per_pwm   if j in cfg.temps_for_channel(i), else 0
    d_j      = offset-free disturbance estimate, degrees C per tick

So a channel at +1.0 PWM lowers each temperature it controls by
``mpc_gain_c_per_pwm`` degrees C at steady state, with time constant
``mpc_tau_s``. Model mismatch is expected (section 4.6) and is absorbed by
``d``.

Cost over the horizon ``N = cfg.horizon`` (``u_{-1} = prev``, the PWM the
command will be rate-limited against)::

    sum_{k=1..N} sum_j weights[j] * (T_j[k] - setpoint_j)^2
      + weight_pwm  * sum_{k=0..N-1} |u_k|^2
      + weight_dpwm * sum_{k=0..N-1} |u_k - u_{k-1}|^2

subject to ``pwm_min <= u_k <= pwm_max``. The QP is condensed (decision
vector ``U = [u_0 .. u_{N-1}]``, ``N * m`` entries) and solved with a primal
active-set method on the box: one projected-gradient step identifies the
active face, then each iteration solves the equality-constrained problem
on the free variables (a small dense ``numpy.linalg.solve``), takes the
longest feasible step and adds the blocking bound -- or, when the whole
step clipped into the box is cheaper, that projected step with every
bound it touches -- or releases the bound with the most negative
multiplier. It terminates finitely and exactly (no
tolerance-dependent output), is warm-started from the previous plan
shifted by one tick, and stops at ``cfg.solver_max_iter`` iterations with
``converged=False`` -- ``step`` turns that into a ``solver`` fault
(section 4.3). The rate limit ``d_pwm_max`` is *not* a QP constraint: the
move penalty keeps planned moves small and ``step`` enforces the hard
limit on the emitted command, while the estimator always uses the PWM
that was actually applied.

Honest saturation (section 4.2): the QP keeps the plan inside the box, so
the *reported* demand adds the pressure on an active bound of ``u_0`` --
``u_0 - g_0 / H_00`` (the coordinate-wise unconstrained minimiser) whenever
the gradient still pushes outward. A demand above ``pwm_max`` is how
``step`` learns the plant needs more than the fans can give.

Offset-free tracking: ``d`` is a per-temperature disturbance state updated
once per solve from the one-step prediction residual::

    d <- d + mpc_estimator_gain * (T_now - (a * T_prev + B * u_applied + d))

``u_applied`` is ``req.prev_pwm`` (the previous *emitted* command) and
``T_prev`` the temperatures of the previous solve. At steady state the
residual is zero only when the model predicts the observed temperature, so
the plan's equilibrium input is exactly what holds the setpoint: no
persistent error. ``d`` converges to a physical quantity and cannot wind
up; it is not updated while ``step`` keeps the solver out (fallback).

Bumpless transfer (section 3): ``initialise`` chooses ``d`` such that the
constrained QP's first move equals ``prev`` (clamped into the box).
``u_0(d)`` is piecewise affine and non-decreasing in ``d``, so the
solution set is a point when the first move is interior and a half-line
when it sits on a bound (every ``d`` past the knee pins ``u_0`` there).
Which point of that half-line is taken matters: ``d`` is the model's
belief about the heat load, and the estimator unwinds any fiction in it
over the following ticks, moving the PWM while it does. The reference is
the *physical* prior ``d_eq = (1 - a) T_0 - B prev`` -- the disturbance
under which the loop is at rest where it is with the PWM that is on the
fans. ``initialise`` keeps ``d_eq`` itself when it already satisfies the
condition (a cold loop whose PWM was clamped up to ``pwm_min``, a hot loop
at ``pwm_max``: the demand then carries honest pressure into the rail and
``step`` clamps it, so nothing moves on the fans); otherwise a few
Gauss-Newton steps on ``u_0(d) - prev`` from the closed-form unconstrained
solution find a solution (once the right piece is found one step lands to
rounding precision), and if that solution pins ``u_0`` with pressure it is
pulled back along the segment toward ``d_eq`` to the knee, the nearest
``d`` that still satisfies the condition. Before this rule the search
accepted whatever point of the half-line it hit first; a loop 5 degC
*below* setpoint whose PWM was clamped up to ``pwm_min`` came out with a
``d`` worth a 90 degC equilibrium and the next ticks *raised* cooling while
the estimator unwound it (the wrong direction per section 4.2). With
``weight_pwm = 0`` and every channel of a temperature sharing the same gain
the system is exactly solvable for any number of channels; otherwise it is
solved in the least-squares sense and the (small) residual is
rate-limited by ``step``. The estimator skips the first solve after
``initialise`` (``memory["fresh"]``), so that solve reproduces the
initialisation bit for bit.

``MpcState.integrator`` holds ``u_ss``, the equilibrium PWM per channel the
model needs to hold every setpoint given ``d`` (``B^+ ((1 - a) sp - d)``,
clamped into ``[pwm_min, pwm_max]``) -- the MPC counterpart of the PI
integral term. The solver's own slot in ``MpcState.solver_memory["mpc"]``
carries ``d``, the previous plan (warm start), the previous temperatures
and the ``fresh`` flag; everything is finite JSON.
"""

from __future__ import annotations

import math
from collections.abc import Callable, Mapping
from dataclasses import dataclass
from typing import Any

import numpy as np

from aqua_bridge.control.solver_pi import SolverRequest, SolverResult
from aqua_bridge.model import MpcConfig

__all__ = [
    "BUMPLESS_TOL",
    "BoxQpResult",
    "MpcProblem",
    "MpcSolver",
    "build_problem",
    "solve_box_qp",
]

#: |u_0 - prev| accepted by ``initialise`` as bumpless (PWM units).
BUMPLESS_TOL = 1e-10
#: Gauss-Newton iterations / step halvings in the bumpless search.
_BUMPLESS_ITER = 30
_BUMPLESS_HALVINGS = 8
#: Bisection steps pulling a pinned bumpless ``d`` back toward ``d_eq``, and the
#: resolution in ``d`` (degrees C per tick) at which the knee counts as found.
_BUMPLESS_BISECT = 60
_KNEE_TOL = 1e-12
#: Finite-difference step on ``d`` (degrees C per tick) for the search Jacobian.
_FD_STEP = 1e-6
#: Active-set tolerances: a Newton step this small is zero; a multiplier this
#: negative means the bound is wrongly active.
_STEP_TOL = 1e-12
_LAMBDA_TOL = 1e-9
#: Gradient pressure on an active bound below this is not reported as demand.
_PRESSURE_TOL = 1e-9
_CACHE_SIZE = 16


# ---------------------------------------------------------------------------
# problem matrices (per config, cached)
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class MpcProblem:
    """Condensed QP data for one configuration; built once, reused every tick."""

    temps: tuple[str, ...]
    channels: tuple[str, ...]
    horizon: int
    a: float
    B: np.ndarray  # p x m
    Phi: np.ndarray  # Np x p : free response of T_0
    S: np.ndarray  # Np x Nm : response to U
    Gamma: np.ndarray  # Np x p : response to a constant d
    SW: np.ndarray  # Nm x Np : S^T diag(Wbar)
    H: np.ndarray  # Nm x Nm : Hessian
    Fd: np.ndarray  # Nm x p : df/dd = 2 S^T Wbar Gamma
    DtE: np.ndarray  # Nm x m : D^T E (move penalty against prev)
    J_unc: np.ndarray  # m x p : d u_0_unconstrained / d d
    L: float  # largest eigenvalue of H
    lo: np.ndarray
    hi: np.ndarray
    Bpinv: np.ndarray  # m x p
    sp: np.ndarray  # p
    spbar: np.ndarray  # Np
    weight_dpwm: float

    @property
    def p(self) -> int:
        return len(self.temps)

    @property
    def m(self) -> int:
        return len(self.channels)

    def linear_term(self, T0: np.ndarray, d: np.ndarray, prev: np.ndarray) -> np.ndarray:
        """``f`` of ``0.5 U^T H U + f^T U`` for the current temperatures, disturbance, prev."""
        c = self.Phi @ T0 + self.Gamma @ d - self.spbar
        return 2.0 * (self.SW @ c) - 2.0 * self.weight_dpwm * (self.DtE @ prev)

    def d_equilibrium(self, T0: np.ndarray, u: np.ndarray) -> np.ndarray:
        """Disturbance under which ``T0`` is the steady state for input ``u``."""
        return (1.0 - self.a) * T0 - self.B @ u

    def u_ss(self, d: np.ndarray, lo: float, hi: float) -> np.ndarray:
        """Equilibrium PWM holding every setpoint under ``d`` (min-norm), clamped."""
        return np.clip(self.Bpinv @ ((1.0 - self.a) * self.sp - d), lo, hi)


_CACHE: dict[tuple, MpcProblem] = {}


def _cache_key(cfg: MpcConfig) -> tuple:
    return (
        cfg.dt,
        cfg.horizon,
        cfg.temps,
        tuple(sorted(cfg.setpoints.items())),
        cfg.channels,
        tuple(sorted((ch, tuple(cfg.temps_for_channel(ch))) for ch in cfg.channels)),
        tuple(sorted(cfg.weights.items())),
        cfg.weight_pwm,
        cfg.weight_dpwm,
        cfg.pwm_min,
        cfg.pwm_max,
        cfg.mpc_tau_s,
        cfg.mpc_gain_c_per_pwm,
    )


def build_problem(cfg: MpcConfig) -> MpcProblem:
    """Prediction and cost matrices for ``cfg`` (cached on the relevant fields)."""
    key = _cache_key(cfg)
    prob = _CACHE.get(key)
    if prob is not None:
        return prob

    temps = tuple(t for t in cfg.temps if t in cfg.setpoints)
    channels = tuple(cfg.channels)
    p, m, N = len(temps), len(channels), cfg.horizon
    a = math.exp(-cfg.dt / cfg.mpc_tau_s)

    K = np.zeros((p, m))
    for i, ch in enumerate(channels):
        for name in cfg.temps_for_channel(ch):
            K[temps.index(name), i] = cfg.mpc_gain_c_per_pwm
    B = -(1.0 - a) * K

    Phi = np.zeros((N * p, p))
    S = np.zeros((N * p, N * m))
    Gamma = np.zeros((N * p, p))
    Ip = np.eye(p)
    powers = [a**k for k in range(N + 1)]
    gsum = 0.0  # sum_{l<k} a^l
    for k in range(1, N + 1):
        rows = slice((k - 1) * p, k * p)
        gsum += powers[k - 1]
        Phi[rows] = powers[k] * Ip
        Gamma[rows] = gsum * Ip
        for j in range(k):
            S[rows, j * m : (j + 1) * m] = powers[k - 1 - j] * B

    w = np.array([cfg.weight_for(t) for t in temps], dtype=float)
    Wbar = np.tile(w, N)
    D = np.eye(N * m)
    for k in range(1, N):
        D[k * m : (k + 1) * m, (k - 1) * m : k * m] = -np.eye(m)
    E = np.zeros((N * m, m))
    E[:m] = np.eye(m)

    SW = S.T * Wbar[None, :]
    H = 2.0 * (SW @ S + cfg.weight_pwm * np.eye(N * m) + cfg.weight_dpwm * (D.T @ D))
    H = 0.5 * (H + H.T)
    Fd = 2.0 * (SW @ Gamma)
    DtE = D.T @ E
    J_unc = -np.linalg.solve(H, Fd)[:m]
    L = float(np.linalg.eigvalsh(H)[-1])
    sp = np.array([cfg.setpoints[t] for t in temps], dtype=float)

    prob = MpcProblem(
        temps=temps,
        channels=channels,
        horizon=N,
        a=a,
        B=B,
        Phi=Phi,
        S=S,
        Gamma=Gamma,
        SW=SW,
        H=H,
        Fd=Fd,
        DtE=DtE,
        J_unc=J_unc,
        L=L,
        lo=np.full(N * m, cfg.pwm_min),
        hi=np.full(N * m, cfg.pwm_max),
        Bpinv=np.linalg.pinv(B),
        sp=sp,
        spbar=np.tile(sp, N),
        weight_dpwm=cfg.weight_dpwm,
    )
    if len(_CACHE) >= _CACHE_SIZE:
        _CACHE.pop(next(iter(_CACHE)))
    _CACHE[key] = prob
    return prob


# ---------------------------------------------------------------------------
# box-constrained QP: primal active set
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class BoxQpResult:
    x: np.ndarray
    g: np.ndarray  # gradient H x + f at x
    side: np.ndarray  # int8 per variable: -1 at lower bound, +1 at upper, 0 free
    iterations: int
    converged: bool


def solve_box_qp(
    H: np.ndarray,
    f: np.ndarray,
    lo: np.ndarray,
    hi: np.ndarray,
    x0: np.ndarray,
    L: float,
    max_iter: int,
) -> BoxQpResult:
    """``min 0.5 x^T H x + f^T x`` s.t. ``lo <= x <= hi`` (``H`` positive definite).

    Primal active-set method (module docstring). Exact and finite; the
    iteration count is the number of active-set changes, ``converged`` is
    False when ``max_iter`` was reached first.
    """
    n = f.shape[0]
    x = np.clip(np.asarray(x0, dtype=float), lo, hi)
    g = H @ x + f
    # Face identification: one projected gradient step adds every bound at once.
    x = np.clip(x - g / L, lo, hi)
    g = H @ x + f
    side = np.zeros(n, dtype=np.int8)
    side[x <= lo] = -1
    side[x >= hi] = 1
    x[side < 0] = lo[side < 0]
    x[side > 0] = hi[side > 0]

    iterations = 0
    released = np.zeros(n, dtype=bool)  # bounds dropped by the previous release step
    single_release = False
    while True:
        iterations += 1
        if iterations > max_iter:
            return BoxQpResult(x=x, g=g, side=side, iterations=max_iter, converged=False)
        free = np.flatnonzero(side == 0)
        p = np.zeros(n)
        if free.size:
            p[free] = np.linalg.solve(H[np.ix_(free, free)], -g[free])
        if free.size == 0 or float(np.max(np.abs(p))) <= _STEP_TOL:
            fixed = np.flatnonzero(side != 0)
            if fixed.size == 0:
                return BoxQpResult(x=x, g=g, side=side, iterations=iterations, converged=True)
            # Multiplier of an active bound: the gradient must point outward.
            lam = np.where(side[fixed] < 0, g[fixed], -g[fixed])
            j = int(np.argmin(lam))
            if lam[j] >= -_LAMBDA_TOL:
                return BoxQpResult(x=x, g=g, side=side, iterations=iterations, converged=True)
            # Release every wrongly active bound at once (few iterations when many rail);
            # fall back to the textbook single release -- guaranteed to move inward --
            # as soon as a multi-release is caught pushing a freed variable back out.
            released[:] = False
            if single_release:
                side[fixed[j]] = 0
                released[fixed[j]] = True
            else:
                drop = fixed[lam < -_LAMBDA_TOL]
                side[drop] = 0
                released[drop] = True
            continue
        ratio = np.full(n, np.inf)
        pos = p > 0
        neg = p < 0
        ratio[pos] = (hi[pos] - x[pos]) / p[pos]
        ratio[neg] = (lo[neg] - x[neg]) / p[neg]
        j = int(np.argmin(ratio))
        alpha = float(ratio[j])
        if alpha <= 0.0 and released[j]:
            single_release = True  # a just-released variable wants straight back out
        released[:] = False
        if alpha < 1.0:
            # Blocked: the classic step goes to the first bound and adds it.
            x_a = np.clip(x + max(alpha, 0.0) * p, lo, hi)
            x_a[j] = hi[j] if p[j] > 0 else lo[j]
            side_a = side.copy()
            side_a[j] = 1 if p[j] > 0 else -1
            # Projected Newton step: the whole step clipped into the box adds every bound
            # it touches at once (keeps the iteration count small when many variables
            # rail); accepted only when it lowers the cost, so descent stays monotone.
            x_b = np.clip(x + p, lo, hi)
            g_a = H @ x_a + f
            g_b = H @ x_b + f
            cost_a = 0.5 * float(x_a @ (g_a + f))
            cost_b = 0.5 * float(x_b @ (g_b + f))
            if cost_b < cost_a - 1e-15 * (1.0 + abs(cost_a)):
                x, g = x_b, g_b
                side = side.copy()
                side[(side == 0) & (x <= lo)] = -1
                side[(side == 0) & (x >= hi)] = 1
                x[side < 0] = lo[side < 0]
                x[side > 0] = hi[side > 0]
            else:
                x, g, side = x_a, g_a, side_a
        else:
            x = np.clip(x + p, lo, hi)
            g = H @ x + f


# ---------------------------------------------------------------------------
# helpers
# ---------------------------------------------------------------------------


def _finite_vec(values: Mapping[str, Any], names: tuple[str, ...], what: str) -> np.ndarray:
    out = np.empty(len(names))
    for i, name in enumerate(names):
        v = values.get(name)
        if not isinstance(v, int | float) or isinstance(v, bool) or not math.isfinite(float(v)):
            raise ValueError(f"{what}[{name!r}] is not a finite number: {v!r}")
        out[i] = float(v)
    return out


def _finite_list(value: Any, length: int) -> np.ndarray | None:
    """A JSON list of ``length`` finite numbers as an array, else ``None``."""
    if not isinstance(value, list | tuple) or len(value) != length:
        return None
    try:
        arr = np.array([float(v) for v in value], dtype=float)
    except (TypeError, ValueError):
        return None
    if not np.all(np.isfinite(arr)):
        return None
    return arr


def _finite_map(value: Any, names: tuple[str, ...]) -> np.ndarray | None:
    if not isinstance(value, Mapping):
        return None
    try:
        return _finite_vec(value, names, "memory")
    except ValueError:
        return None


def _to_map(names: tuple[str, ...], vec: np.ndarray) -> dict[str, float]:
    return {name: float(v) for name, v in zip(names, vec, strict=True)}


# ---------------------------------------------------------------------------
# the solver
# ---------------------------------------------------------------------------


class MpcSolver:
    """Linear offset-free MPC on the ``Solver`` protocol (module docstring)."""

    name = "mpc"

    # -- QP wrapper -----------------------------------------------------------

    @staticmethod
    def _qp(
        prob: MpcProblem,
        cfg: MpcConfig,
        T0: np.ndarray,
        d: np.ndarray,
        prev: np.ndarray,
        warm: np.ndarray,
    ) -> BoxQpResult:
        f = prob.linear_term(T0, d, prev)
        return solve_box_qp(prob.H, f, prob.lo, prob.hi, warm, prob.L, cfg.solver_max_iter)

    @staticmethod
    def _demand(prob: MpcProblem, res: BoxQpResult) -> np.ndarray:
        """First move plus the pressure on an active bound (honest saturation)."""
        m = prob.m
        u0 = res.x[:m].copy()
        for i in range(m):
            g = res.g[i]
            if (res.side[i] > 0 and g < -_PRESSURE_TOL) or (res.side[i] < 0 and g > _PRESSURE_TOL):
                u0[i] = res.x[i] - g / prob.H[i, i]
        return u0

    # -- bumpless initialisation ---------------------------------------------

    def _bumpless_d(
        self,
        prob: MpcProblem,
        cfg: MpcConfig,
        T0: np.ndarray,
        prev: np.ndarray,
        target: np.ndarray,
        warm: np.ndarray,
    ) -> np.ndarray:
        """``d`` such that the constrained first move equals ``target`` (see module docstring)."""

        def first_move(d: np.ndarray) -> np.ndarray:
            res = self._qp(prob, cfg, T0, d, prev, warm)
            if not res.converged:
                raise RuntimeError("bumpless initialisation: QP hit the iteration cap")
            return res.x[: prob.m]

        def pressure(d: np.ndarray) -> float:
            res = self._qp(prob, cfg, T0, d, prev, warm)
            return float(np.max(np.abs(self._demand(prob, res) - res.x[: prob.m])))

        # Physical prior: the loop is at rest at T0 with ``prev`` on the fans. When the
        # first move already equals the target here (it sits on a bound and the plant
        # pushes into that bound) there is no fictitious heat load to unwind later.
        d_eq = prob.d_equilibrium(T0, prev)
        if float(np.max(np.abs(first_move(d_eq) - target))) <= BUMPLESS_TOL:
            return d_eq
        # Closed-form unconstrained start u0_unc(d) = c + J_unc d, then Gauss-Newton on
        # the constrained first move; from ``d_eq`` as the fallback start.
        f0 = prob.linear_term(T0, np.zeros(prob.p), prev)
        c = -np.linalg.solve(prob.H, f0)[: prob.m]
        d, err = self._refine(
            first_move, prob, np.linalg.lstsq(prob.J_unc, target - c, rcond=None)[0], target
        )
        if err > BUMPLESS_TOL:
            d2, err2 = self._refine(first_move, prob, d_eq, target)
            if err2 < err:
                d, err = d2, err2
        if err > BUMPLESS_TOL or pressure(d) <= _PRESSURE_TOL:
            return d
        # ``u_0`` is pinned with pressure: every ``d`` further out satisfies the condition
        # too, so walk back toward ``d_eq`` to the knee (bisection on the segment; the
        # far end satisfies, ``d_eq`` does not).
        lo, hi = 0.0, 1.0
        best = d
        span = float(np.max(np.abs(d - d_eq)))
        for _ in range(_BUMPLESS_BISECT):
            if (hi - lo) * span <= _KNEE_TOL:
                break
            mid = 0.5 * (lo + hi)
            dm = d_eq + mid * (d - d_eq)
            if float(np.max(np.abs(first_move(dm) - target))) <= BUMPLESS_TOL:
                hi, best = mid, dm
            else:
                lo = mid
        return best

    @staticmethod
    def _refine(
        first_move: Callable[[np.ndarray], np.ndarray],
        prob: MpcProblem,
        d: np.ndarray,
        target: np.ndarray,
    ) -> tuple[np.ndarray, float]:
        """Gauss-Newton on ``first_move(d) - target`` from ``d``; -> ``(d, max abs error)``."""
        u = first_move(d)
        g = u - target
        err = float(np.max(np.abs(g)))
        for _ in range(_BUMPLESS_ITER):
            if err <= BUMPLESS_TOL:
                break
            # Finite-difference Jacobian of the constrained first move; a flat
            # column (u_0 pinned) falls back to the unconstrained slope.
            J = np.empty((prob.m, prob.p))
            for j in range(prob.p):
                dj = d.copy()
                dj[j] += _FD_STEP
                J[:, j] = (first_move(dj) - u) / _FD_STEP
                if np.linalg.norm(J[:, j]) < 1e-9 * max(1.0, np.linalg.norm(prob.J_unc[:, j])):
                    J[:, j] = prob.J_unc[:, j]
            step = np.linalg.lstsq(J, -g, rcond=None)[0]
            if not np.all(np.isfinite(step)):
                break
            improved = False
            scale = 1.0
            for _ in range(_BUMPLESS_HALVINGS):
                dn = d + scale * step
                un = first_move(dn)
                gn = un - target
                errn = float(np.max(np.abs(gn)))
                if errn < err:
                    d, u, g, err = dn, un, gn, errn
                    improved = True
                    break
                scale *= 0.5
            if not improved:
                break
        return d, err

    def initialise(
        self, cfg: MpcConfig, req: SolverRequest
    ) -> tuple[dict[str, float], dict[str, Any]]:
        prob = build_problem(cfg)
        T0 = _finite_vec(req.temps, prob.temps, "temps")
        prev = _finite_vec(req.prev_pwm, prob.channels, "prev_pwm")
        target = np.clip(prev, cfg.pwm_min, cfg.pwm_max)
        warm = np.tile(target, prob.horizon)
        d = self._bumpless_d(prob, cfg, T0, prev, target, warm)
        integrator = _to_map(prob.channels, prob.u_ss(d, cfg.pwm_min, cfg.pwm_max))
        memory = {
            "d": _to_map(prob.temps, d),
            "warm": [float(v) for v in warm],
            "fresh": True,
            "last_temps": None,
        }
        return integrator, memory

    # -- one tick -------------------------------------------------------------

    def solve(self, cfg: MpcConfig, req: SolverRequest) -> SolverResult:
        prob = build_problem(cfg)
        T0 = _finite_vec(req.temps, prob.temps, "temps")
        prev = _finite_vec(req.prev_pwm, prob.channels, "prev_pwm")
        mem = req.memory if isinstance(req.memory, Mapping) else {}
        n = prob.horizon * prob.m

        d = _finite_map(mem.get("d"), prob.temps)
        if d is None:  # no usable memory: assume the loop sits at equilibrium
            d = prob.d_equilibrium(T0, prev)
        fresh = mem.get("fresh") is True
        residual: np.ndarray | None = None
        if fresh:
            warm = _finite_list(mem.get("warm"), n)
            if warm is None:
                warm = np.tile(np.clip(prev, cfg.pwm_min, cfg.pwm_max), prob.horizon)
        else:
            plan = _finite_list(mem.get("plan"), n)
            if plan is None:
                warm = np.tile(np.clip(prev, cfg.pwm_min, cfg.pwm_max), prob.horizon)
            else:  # shift by one tick, repeat the last move
                warm = np.concatenate([plan[prob.m :], plan[-prob.m :]])
            last = _finite_map(mem.get("last_temps"), prob.temps)
            if last is not None:
                predicted = prob.a * last + prob.B @ prev + d
                residual = T0 - predicted
                d = d + cfg.mpc_estimator_gain * residual

        res = self._qp(prob, cfg, T0, d, prev, warm)
        u_ss = prob.u_ss(d, cfg.pwm_min, cfg.pwm_max)
        if not res.converged:
            return SolverResult(
                pwm=_to_map(prob.channels, res.x[: prob.m]),
                integrator=_to_map(prob.channels, u_ss),
                memory=dict(mem),
                converged=False,
                iterations=res.iterations,
                diagnostics={"iterations": res.iterations, "converged": False},
            )
        demand = self._demand(prob, res)
        predicted_end = (
            prob.Phi[-prob.p :] @ T0 + prob.S[-prob.p :] @ res.x + prob.Gamma[-prob.p :] @ d
        )
        memory = {
            "d": _to_map(prob.temps, d),
            "plan": [float(v) for v in res.x],
            "fresh": False,
            "last_temps": _to_map(prob.temps, T0),
        }
        diagnostics = {
            "error": _to_map(prob.temps, T0 - prob.sp),
            "disturbance": _to_map(prob.temps, d),
            "residual": None if residual is None else _to_map(prob.temps, residual),
            "u_ss": _to_map(prob.channels, u_ss),
            "pressure": _to_map(prob.channels, demand - res.x[: prob.m]),
            "predicted_end": _to_map(prob.temps, predicted_end),
            "iterations": res.iterations,
            "active_bounds": int(np.count_nonzero(res.side)),
            "fresh": fresh,
        }
        return SolverResult(
            pwm=_to_map(prob.channels, demand),
            integrator=_to_map(prob.channels, u_ss),
            memory=memory,
            converged=True,
            iterations=res.iterations,
            diagnostics=diagnostics,
        )
