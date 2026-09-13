"""Solver protocol and the first-cut PI solver.

``mpc.step`` owns the gate, the fault timer, the fallback policy, the rate
limit and the clamp. A :class:`Solver` only turns trusted temperatures
into an *unconstrained demand* per channel plus its own next memory. That
keeps the solver swappable: the MPC (``solver_mpc.py``) and the DAS MPC
(``solver_das.py``) implement the same protocol and ``step`` does not change.

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
cooling loop: the hottest deviation drives the fan (anti-windup and bumpless
start below).

PI-like DAS form (plan section 4, "PI-like DAS fallback")
---------------------------------------------------------
With ``topology`` and no ``setpoints`` (:attr:`MpcConfig.regulates_drive_limits`)
the error is the *margin deficit* of the worst drive the channel cools, read
from ``SolverRequest.estimates`` (:mod:`aqua_bridge.control.estimates`)::

    e = max(est[b].t + est[b].margin - est[b].soft
            for b in the constrained bays of cfg.zone_layout.served[ch]
            whose zone is trusted this tick)

with ``margin = k * sigma`` and ``soft = limit - comfort - k * sigma``; the rest
(integrator, anti-windup, bumpless start) is exactly the PI above, and the
solver keeps ``name = "pi"``. A bay is constrained unless it is declared
``occupied: false`` or the estimator reports it ``empty`` this tick
(``SolverRequest.occupancy``). The served zones are the zones that list the
channel plus their declared ``coupled_to``: the drives next door feel the
channel's air too, so the channel works for them as well; drives of unknown
occupancy count. The formula is the plan's as written, and it counts
``k * sigma`` twice (once inside ``soft``, once added to ``t``), so the
drive settles ``2 k sigma`` below ``limit - comfort``. That errs toward more
cooling; the DAS MPC (``solver_das.py``) counts ``k * sigma`` once (its soft rows
are ``T_d <= soft``), so its model fallback to this form is louder, never hotter.

Two edge cases, both documented choices:

* a constrained bay in a trusted served zone without an estimate is a
  contract violation (``KeyError``, which ``step`` turns into a fault: never
  less cooling);
* a channel whose served zones hold no constrained bay with a trusted zone
  (every bay ``occupied: false``, or the only drives sit in a coupled zone in
  fault under ``fault_coupling: none``) has nothing to regulate: ``e = 0``,
  so the integrator holds the command where it is (neither raised nor lowered)
  and ``diagnostics["unconstrained"]`` lists the channel.

In legacy mode, and on zoned configs that still declare ``setpoints``, the
error is today's ``channel_errors`` bit for bit.

Both forms: a channel in ``fixed_channels`` gets no error, no integrator entry (so it starts
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
    "channel_margin_errors",
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
    * ``estimates``      -- bay -> estimate entry (``aqua_bridge.control.estimates``) for
      the constrained bays of trusted zones (empty in legacy mode)
    * ``occupancy``      -- bay -> ``occupied`` | ``unknown`` | ``empty`` as the estimator
      sees it this tick; a bay missing here falls back to its declaration
      (``occupied: false`` is empty, anything else constrained)
    * ``ts``             -- ``obs.ts`` of the tick (with zones; ``None`` in legacy mode)
    * ``thermal``        -- the thermal model's memory as the previous tick left it
      (``solver_memory["thermal"]``, read-only; ``None`` without ``model_shadow`` or in
      legacy mode), for the DAS MPC's model (``aqua_bridge.control.solver_das``)
    * ``plant``          -- the estimator's state for the DAS MPC's prediction, every
      zone including those in fault: ``{"zones": {zone: {"t_air", "d_air", "t_in"}},
      "bays": {bay: {"occupancy", "class", "since_ts", "t"?, "q_w"?, "sigma"?, "sigma_cal"?}}}``
      (the last four only for a bay with an estimate); empty in legacy mode
    """

    temps: dict[str, float]
    prev_pwm: dict[str, float]
    integrator: dict[str, float] = field(default_factory=dict)
    memory: dict[str, Any] = field(default_factory=dict)
    fixed_channels: dict[str, float] = field(default_factory=dict)
    zone_trust: dict[str, bool] = field(default_factory=dict)
    estimates: dict[str, dict[str, Any]] = field(default_factory=dict)
    occupancy: dict[str, str] = field(default_factory=dict)
    ts: float | None = None
    thermal: Mapping[str, Any] | None = None
    plant: dict[str, Any] = field(default_factory=dict)


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


def channel_margin_errors(
    cfg: MpcConfig,
    estimates: Mapping[str, Mapping[str, Any]],
    zone_trust: Mapping[str, bool],
    skip: Container[str] = (),
    occupancy: Mapping[str, str] | None = None,
) -> tuple[dict[str, float], dict[str, str | None]]:
    """Margin deficit per channel and the bay that sets it (module docstring, DAS form).

    Returns ``(errors, worst_bay)``; ``worst_bay[ch]`` is ``None`` for an
    unconstrained channel (error 0). A bay is skipped when it is declared
    ``occupied: false`` or ``occupancy`` reports it ``empty``. Raises ``KeyError``
    when any other bay of a trusted served zone has no estimate.
    """
    topo = cfg.topology
    if topo is None:
        raise ValueError("the margin-deficit form needs mpc.topology")
    layout = cfg.zone_layout
    errors: dict[str, float] = {}
    worst: dict[str, str | None] = {}
    for ch in cfg.channels:
        if ch in skip:
            continue
        served = layout.served[ch]
        best: tuple[float, str] | None = None
        for bay, spec in topo.bays.items():
            if not spec.constrained or spec.zone not in served or not zone_trust.get(spec.zone):
                continue
            if occupancy is not None and occupancy.get(bay) == "empty":
                continue
            est = estimates[bay]
            e = float(est["t"]) + float(est["margin"]) - float(est["soft"])
            if best is None or e > best[0]:
                best = (e, bay)
        if best is None:
            errors[ch], worst[ch] = 0.0, None
        else:
            errors[ch], worst[ch] = best
    return errors, worst


def _clamp(value: float, lo: float, hi: float) -> float:
    return lo if value < lo else hi if value > hi else value


class PiSolver:
    """Per-channel PI on the controlled temperature error (module docstring)."""

    name = "pi"

    @staticmethod
    def _errors(
        cfg: MpcConfig, req: SolverRequest
    ) -> tuple[dict[str, float], dict[str, str | None] | None]:
        if cfg.regulates_drive_limits:
            return channel_margin_errors(
                cfg, req.estimates, req.zone_trust, req.fixed_channels, req.occupancy
            )
        return channel_errors(cfg, req.temps, req.fixed_channels), None

    def initialise(
        self, cfg: MpcConfig, req: SolverRequest
    ) -> tuple[dict[str, float], dict[str, Any]]:
        fixed = req.fixed_channels
        errors, _ = self._errors(cfg, req)
        integrator = {
            ch: float(req.prev_pwm[ch]) - cfg.pi_kp * errors[ch]
            for ch in cfg.channels
            if ch not in fixed
        }
        return integrator, {}

    def solve(self, cfg: MpcConfig, req: SolverRequest) -> SolverResult:
        fixed = req.fixed_channels
        errors, worst_bay = self._errors(cfg, req)
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
        diagnostics: dict[str, Any] = {
            "error": errors,
            "p_term": p_term,
            "i_term": i_term,
            "saturated_high": sat_hi,
            "saturated_low": sat_lo,
        }
        if worst_bay is not None:  # DAS form only: legacy diagnostics keep their keys
            diagnostics["form"] = "margin_deficit"
            diagnostics["worst_bay"] = worst_bay
            diagnostics["unconstrained"] = [ch for ch, bay in worst_bay.items() if bay is None]
        return SolverResult(
            pwm=pwm,
            integrator=integrator,
            memory={},
            converged=True,
            iterations=1,
            diagnostics=diagnostics,
        )
