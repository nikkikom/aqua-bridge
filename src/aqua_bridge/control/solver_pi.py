"""Solver protocol and the first-cut PI solver.

``mpc.step`` owns the gate, the fault timer, the fallback policy, the rate
limit and the clamp. A :class:`Solver` only turns trusted temperatures
into an *unconstrained demand* per channel plus its own next memory. That
keeps the solver swappable: the MPC (``solver_mpc.py``) implements the same
protocol and ``step`` does not change.

Contract every solver must honour (checked by ``step``, violations become
a ``solver`` fault, never an exception out of ``step``):

* ``solve`` returns a finite demand for exactly ``cfg.channels``;
* ``integrator`` / ``memory`` in the result are finite and JSON-serialisable;
* ``initialise`` returns an integrator/memory such that an immediately
  following ``solve`` with the same request yields ``prev_pwm`` (bumpless
  transfer, section 3) up to float rounding;
* ``converged=False`` means the iteration cap was hit (section 4.3);
* channels in ``SolverRequest.fixed_channels`` are under ``step``'s fallback
  policy (their zone, or a zone coupled to it, is in fault): the solver
  must still return a demand for them (``step`` ignores it), should treat
  them as known inputs at the given command, must not need the
  temperatures of faulted zones (they are absent from ``temps``) and
  ``initialise`` owes bumplessness only on the other channels. A solver
  that cannot honour this raises, which ``step`` turns into a fault of
  every zone it was asked to drive. Legacy mode never fixes a channel.

PI policy (per fan channel ``ch``)::

    e   = max(T[name] - setpoint[name] for name in cfg.temps_for_channel(ch))
    u   = pi_kp * e + I[ch]                       # demand, unclamped
    I'  = clamp(I + pi_ki * e * dt, pwm_min, pwm_max)

The ``max`` over a channel's temperatures is the conservative choice for a
cooling loop: the hottest deviation drives the fan. A channel in
``fixed_channels`` gets no error, no integrator entry (so it starts
bumplessly when released) and its fixed command as demand. Anti-windup is the
clamp of ``I`` into ``[pwm_min, pwm_max]`` ("the integral alone stays in
the actuator range"), deliberately *not* conditional integration: holding
``I`` as soon as ``u`` crosses a rail parks the integrator exactly where
``u == pwm_max``, so at an unreachable setpoint the demand chatters around
the rail (auto with ``pwm = 0.9998`` on some ticks) and section 4.2
"saturation is honest -- mode=saturated, PWM pinned at max" does not hold.
With the clamp, ``I`` settles at ``pwm_max`` and ``u = pwm_max + pi_kp * e``
stays strictly above the rail while ``e > 0``; the price is a
de-saturation lag of at most ``pi_kp * e / (pi_ki * |e|)`` seconds, which
errs toward more cooling. Bumpless re-initialisation sets
``I = prev_pwm - pi_kp * e`` so the first output equals ``prev_pwm``.
"""

from __future__ import annotations

from collections.abc import Container, Mapping
from dataclasses import dataclass, field
from typing import Any, Protocol, runtime_checkable

from aqua_bridge.model import MpcConfig

__all__ = [
    "PiSolver",
    "Solver",
    "SolverRequest",
    "SolverResult",
    "channel_errors",
]


@dataclass(frozen=True)
class SolverRequest:
    """Everything a solver may look at for one tick.

    * ``temps``          -- trusted (gate-filtered) temperatures, keys ``cfg.temps``
      (with zones: only the sensors of zones that are not in fault and trusted
      sensors without a zone)
    * ``prev_pwm``       -- the PWM the command will be rate-limited against
    * ``integrator``     -- ``MpcState.integrator`` (may be empty or lack channels)
    * ``memory``         -- the solver's own slot of ``MpcState.solver_memory``
    * ``fixed_channels`` -- channels under fallback policy -> the command ``step`` will
      target on them this tick (empty in legacy mode)
    * ``zone_trust``     -- zone -> whether the solver may rely on its sensors this tick
      (empty in legacy mode)
    """

    temps: dict[str, float]
    prev_pwm: dict[str, float]
    integrator: dict[str, float] = field(default_factory=dict)
    memory: dict[str, Any] = field(default_factory=dict)
    fixed_channels: dict[str, float] = field(default_factory=dict)
    zone_trust: dict[str, bool] = field(default_factory=dict)


@dataclass(frozen=True)
class SolverResult:
    """What a solver hands back to ``step``.

    ``pwm`` is the *unconstrained* demand: ``step`` rate-limits and clamps
    it. A demand above ``pwm_max`` is how the solver says "the plant needs
    more than the fans can give" (``mode=saturated``).
    """

    pwm: dict[str, float]
    integrator: dict[str, float]
    memory: dict[str, Any] = field(default_factory=dict)
    converged: bool = True
    iterations: int = 1
    diagnostics: dict[str, Any] = field(default_factory=dict)


@runtime_checkable
class Solver(Protocol):
    """Pure controller core; see the module docstring for the contract."""

    name: str

    def initialise(
        self, cfg: MpcConfig, req: SolverRequest
    ) -> tuple[dict[str, float], dict[str, Any]]:
        """Bumpless (re)initialisation: memory such that ``solve`` returns ``req.prev_pwm``."""
        ...

    def solve(self, cfg: MpcConfig, req: SolverRequest) -> SolverResult:
        """One tick. Must not mutate ``req``; may raise (``step`` turns it into a fault)."""
        ...


def channel_errors(
    cfg: MpcConfig, temps: Mapping[str, float], skip: Container[str] = ()
) -> dict[str, float]:
    """Tracking error per channel: ``max(T - setpoint)`` over the channel's temperatures.

    Raises ``KeyError`` when a controlled temperature is absent from
    ``temps``; ``step`` only calls solvers with a complete trusted set for
    every channel outside ``skip`` (the fixed channels). Raises
    ``ValueError`` for a channel that controls no temperature.
    """
    out: dict[str, float] = {}
    for ch in cfg.channels:
        if ch in skip:
            continue
        names = cfg.temps_for_channel(ch)
        out[ch] = max(float(temps[name]) - cfg.setpoints[name] for name in names)
    return out


def _clamp(value: float, lo: float, hi: float) -> float:
    return lo if value < lo else hi if value > hi else value


class PiSolver:
    """Per-channel PI on the controlled temperature error (module docstring)."""

    name = "pi"

    def initialise(
        self, cfg: MpcConfig, req: SolverRequest
    ) -> tuple[dict[str, float], dict[str, Any]]:
        fixed = req.fixed_channels
        errors = channel_errors(cfg, req.temps, fixed)
        integrator = {
            ch: float(req.prev_pwm[ch]) - cfg.pi_kp * errors[ch]
            for ch in cfg.channels
            if ch not in fixed
        }
        return integrator, {}

    def solve(self, cfg: MpcConfig, req: SolverRequest) -> SolverResult:
        fixed = req.fixed_channels
        errors = channel_errors(cfg, req.temps, fixed)
        pwm: dict[str, float] = {}
        integrator: dict[str, float] = {}
        p_term: dict[str, float] = {}
        i_term: dict[str, float] = {}
        sat_hi: dict[str, bool] = {}
        sat_lo: dict[str, bool] = {}
        for ch in cfg.channels:
            if ch in fixed:
                pwm[ch] = float(fixed[ch])
                continue
            e = errors[ch]
            i_now = req.integrator.get(ch)
            if i_now is None:  # channel without memory: start bumpless
                i_now = float(req.prev_pwm[ch]) - cfg.pi_kp * e
            p = cfg.pi_kp * e
            u = p + i_now
            hi = u > cfg.pwm_max
            lo = u < cfg.pwm_min
            # Anti-windup by clamping only (module docstring): the integral keeps
            # advancing at the rail so the demand stays past it, but never
            # leaves the actuator range.
            integrator[ch] = _clamp(i_now + cfg.pi_ki * e * cfg.dt, cfg.pwm_min, cfg.pwm_max)
            pwm[ch] = u
            p_term[ch] = p
            i_term[ch] = i_now
            sat_hi[ch] = hi
            sat_lo[ch] = lo
        return SolverResult(
            pwm=pwm,
            integrator=integrator,
            memory={},
            converged=True,
            iterations=1,
            diagnostics={
                "error": errors,
                "p_term": p_term,
                "i_term": i_term,
                "saturated_high": sat_hi,
                "saturated_low": sat_lo,
            },
        )
