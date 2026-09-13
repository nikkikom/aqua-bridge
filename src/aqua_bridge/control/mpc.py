"""``step(obs, cfg, state) -> (cmd, state)``: the pure controller tick.

PROJECT.md section 3 (contract, safe PWM on untrusted observation, one
fault timer, confirm ticks, bumpless transfer) and section 4.1 (invariants).
No clocks, no randomness, no files: ``obs.ts`` and ``cfg.dt`` are the clock,
``state`` carries everything between ticks, and the same ``(obs, cfg,
state)`` always yields the same ``(cmd, state)``.

Order inside :func:`step`
-------------------------
1. ``prev`` -- what the command is rate-limited against (section 4.1):
   ``state.last_cmd.pwm``; else ``obs.pwm`` when every channel is finite and
   inside ``[pwm_min - d_pwm_max, pwm_max + d_pwm_max]`` (the only range
   from which one rate-limited step can reach the actuator range, so the
   bound and the rate-limit invariants stay jointly satisfiable); else
   ``cfg.fallback_pwm``. Never ``pwm_min``.
2. Time check against ``solver_memory["last_ts"]`` (see *Time policy*).
3. Sensor gate (:mod:`aqua_bridge.control.gate`), fed the Stuck latch from
   ``solver_memory["stuck_latch"]``; the gate's next latch is stored back.
   A "gap" tick drops the window but keeps the latch: only a value that
   leaves the ``stuck_eps_c`` band proves the sensor alive.
4. Fault bookkeeping: an untrusted tick resets ``trusted_streak`` and opens
   the single fault timer (``fault_since_ts`` is kept, never restarted,
   while a fault is active -- Flicker must not reset the hold). A trusted
   tick while in fault only counts toward ``confirm_ticks``.
5. Solver -- only on a trusted tick that is either fault-free or the
   ``confirm_ticks``-th consecutive trusted one. On that returning tick
   (and on a cold integrator) the solver is re-initialised so that its
   first output equals ``prev`` before the rate limit (bumpless). Any
   exception, non-finite / wrong-shaped output, ``converged=False`` or
   ``iterations > solver_max_iter`` is a ``solver`` fault handled exactly
   like a gate fault: same timer, same hold / ramp-high policy, and the
   fault is *not* cleared (the old ``fault_since_ts`` survives a failed
   retry so a persistently broken solver still ramps high).
6. Fallback policy while a fault is active: hold ``prev`` until the fault
   has lasted longer than ``fallback_hold_s``, then target
   ``max(prev, cfg.fallback_pwm)`` per channel. Section 3 item 2 says
   "ramp toward fallback_pwm"; read literally that lowers a channel that is
   already above ``fallback_pwm`` (a hot, saturated plant at ``pwm_max``)
   while the controller is blind, which section 4.1 forbids ("never a step
   toward pwm_min because of the fault"). The conservative reading is
   implemented: a fault never reduces cooling, ``fallback_pwm`` is a floor
   the fans are raised to, not a level they are pulled down to. Elapsed
   time is ``max(obs.ts - fault_since_ts, (fault_ticks - 1) * dt)`` so a
   broken clock cannot stall the ramp.
7. Rate limit vs ``prev`` (``|delta| <= d_pwm_max``), then clamp into
   ``[pwm_min, pwm_max]``.
8. Mode: ``fallback`` while a fault is active; ``saturated`` when a
   channel's demand exceeds ``pwm_max`` *and* the emitted PWM sits at
   ``pwm_max`` (honest: pinned at the rail); ``auto`` otherwise.

Time policy (documented conservative choices; section 4.3 "ts")
---------------------------------------------------------------
* First tick (no ``last_ts``): no time check.
* ``ts <= last_ts`` (duplicate, stalled or backwards clock): the sample is
  not fresh -> untrusted tick (``time.status = "not_advancing"``),
  ``sensor_gate`` fault. ``last_ts`` still follows ``obs.ts`` so a clock
  reset costs ``confirm_ticks`` ticks, while a permanently stuck clock
  keeps the controller in fallback (ramping high by tick count).
* ``ts - last_ts > GAP_TICKS_MAX * dt`` (missed ticks): the history is stale
  -> untrusted tick (``"gap"``), ``window`` / ``last_raw_temps`` are
  dropped so the next tick compares against this fresh sample;
  ``last_good_obs`` is kept.
* Otherwise the gate slew limit is scaled to the real elapsed time,
  ``dT_max_c_per_s * max(gap, dt)``, never below ``dT_max_tick``.

Fan stall (section 4.3 "RPM = 0 ... while commanded PWM is high")
---------------------------------------------------------------
``solver_memory["stall_ticks"][ch]`` counts consecutive ticks with a finite
``obs.rpm[ch] <= 0`` while the previous command for ``ch`` was at or above
the midpoint of ``[pwm_min, pwm_max]``; ``rpm > 0`` resets it, a missing
reading leaves it. ``diagnostics["fan_stall"][ch]`` is set after
``STALL_TICKS``. Policy: flag only. The PI integrator is capped at
``pwm_max`` so it cannot wind up forever, the demand on a stalled channel
saturates honestly, and switching to fallback would not add cooling (a
stalled fan does not move air at ``fallback_pwm`` either) while it would
stop regulating the healthy channels.

``solver_memory`` layout (all JSON, all finite): ``last_ts`` (float),
``fault_ticks`` (int, consecutive ticks in fault), ``stall_ticks`` (dict
channel -> int), ``stuck_latch`` (dict temperature -> band reference in
degrees C, only latched temperatures; gate rule 3) and one sub-dict per
solver under ``solver.name``.
"""

from __future__ import annotations

import dataclasses
import json
import math
from collections.abc import Mapping
from typing import Any

from aqua_bridge.control.gate import GateResult, evaluate_gate, push_window
from aqua_bridge.control.solver_mpc import MpcSolver
from aqua_bridge.control.solver_pi import PiSolver, Solver, SolverRequest, SolverResult
from aqua_bridge.model import (
    FaultReason,
    Mode,
    MpcCommand,
    MpcConfig,
    MpcState,
    PlantObservation,
    SolverKind,
)

__all__ = [
    "GAP_TICKS_MAX",
    "SOLVERS",
    "STALL_TICKS",
    "SolverFault",
    "obs_pwm_usable",
    "resolve_prev",
    "step",
]

#: An observation later than this many ``dt`` after the previous one is a "gap" tick.
GAP_TICKS_MAX = 3.0
#: Consecutive zero-RPM ticks (with a high command) before ``fan_stall`` is flagged.
STALL_TICKS = 10

#: Solver registry by ``cfg.solver`` (section 3: "first step may be PI; swap in MPC later").
SOLVERS: dict[SolverKind, Solver] = {SolverKind.PI: PiSolver(), SolverKind.MPC: MpcSolver()}

_EPS = 1e-12


class SolverFault(RuntimeError):
    """A solver result that ``step`` refuses (shape, finiteness, iteration cap)."""


# ---------------------------------------------------------------------------
# prev resolution
# ---------------------------------------------------------------------------


def obs_pwm_usable(obs: PlantObservation, cfg: MpcConfig) -> bool:
    """``obs.pwm`` may serve as ``prev`` on a cold state.

    Every channel present, finite and inside
    ``[max(0, pwm_min - d_pwm_max), min(1, pwm_max + d_pwm_max)]``: from
    anywhere in that band one rate-limited step reaches ``[pwm_min,
    pwm_max]``, so both section 4.1 invariants can hold at once.
    """
    lo = max(0.0, cfg.pwm_min - cfg.d_pwm_max)
    hi = min(1.0, cfg.pwm_max + cfg.d_pwm_max)
    for ch in cfg.channels:
        v = obs.pwm.get(ch)
        if v is None or not math.isfinite(v) or not lo <= v <= hi:
            return False
    return True


def resolve_prev(
    state: MpcState, obs: PlantObservation, cfg: MpcConfig
) -> tuple[dict[str, float], str]:
    """``prev`` per section 4.1 and its source: ``last_cmd`` | ``obs_pwm`` | ``fallback_pwm``."""
    last = state.last_cmd
    if last is not None and all(
        ch in last.pwm and math.isfinite(last.pwm[ch]) for ch in cfg.channels
    ):
        return {ch: float(last.pwm[ch]) for ch in cfg.channels}, "last_cmd"
    if obs_pwm_usable(obs, cfg):
        return {ch: float(obs.pwm[ch]) for ch in cfg.channels}, "obs_pwm"  # type: ignore[arg-type]
    return dict(cfg.fallback_pwm), "fallback_pwm"


# ---------------------------------------------------------------------------
# helpers
# ---------------------------------------------------------------------------


def _finite(value: object) -> bool:
    return (
        isinstance(value, int | float)
        and not isinstance(value, bool)
        and math.isfinite(float(value))
    )


def _time_check(ts: float, last_ts: float | None, cfg: MpcConfig) -> tuple[str, float, bool]:
    """-> (status, slew limit for this tick, drop history?)."""
    if last_ts is None:
        return "first", cfg.dT_max_tick, False
    gap = ts - last_ts
    if gap <= 0:
        return "not_advancing", cfg.dT_max_tick, False
    if gap > GAP_TICKS_MAX * cfg.dt:
        return "gap", cfg.dT_max_tick, True
    return "ok", cfg.dT_max_c_per_s * max(gap, cfg.dt), False


def _json_finite(value: Any) -> bool:
    try:
        json.dumps(value, allow_nan=False)
    except (TypeError, ValueError):
        return False
    return True


def _check_result(result: object, cfg: MpcConfig) -> SolverResult:
    if not isinstance(result, SolverResult):
        raise SolverFault(f"solver returned {type(result).__name__}, not SolverResult")
    if set(result.pwm) != set(cfg.channels):
        raise SolverFault(
            f"solver pwm keys {sorted(result.pwm)} != channels {sorted(cfg.channels)}"
        )
    for ch, v in result.pwm.items():
        if not _finite(v):
            raise SolverFault(f"solver pwm[{ch!r}] is not finite: {v!r}")
    for ch, v in result.integrator.items():
        if not isinstance(ch, str) or not _finite(v):
            raise SolverFault(f"solver integrator[{ch!r}] is not finite: {v!r}")
    if not isinstance(result.memory, Mapping) or not _json_finite(result.memory):
        raise SolverFault("solver memory is not finite JSON")
    if not result.converged:
        raise SolverFault("solver did not converge (iteration cap)")
    if result.iterations > cfg.solver_max_iter:
        raise SolverFault(f"solver used {result.iterations} > solver_max_iter iterations")
    return result


def _stall_update(
    counts: Mapping[str, Any], obs: PlantObservation, state: MpcState, cfg: MpcConfig
) -> dict[str, int]:
    """Advance the per-channel zero-RPM counters (module docstring, *Fan stall*)."""
    high = cfg.pwm_min + 0.5 * (cfg.pwm_max - cfg.pwm_min)
    out: dict[str, int] = {}
    for ch in cfg.channels:
        n = counts.get(ch, 0)
        n = int(n) if isinstance(n, int | float) and math.isfinite(float(n)) else 0
        rpm = obs.rpm.get(ch)
        if rpm is None or not math.isfinite(rpm):
            out[ch] = n  # unknown reading: neither evidence for nor against a stall
            continue
        if rpm > 0:
            out[ch] = 0
            continue
        commanded = None if state.last_cmd is None else state.last_cmd.pwm.get(ch)
        out[ch] = n + 1 if commanded is not None and commanded >= high - _EPS else n
    return out


def _clamp(value: float, lo: float, hi: float) -> float:
    return lo if value < lo else hi if value > hi else value


# ---------------------------------------------------------------------------
# step
# ---------------------------------------------------------------------------


def step(
    obs: PlantObservation,
    cfg: MpcConfig,
    state: MpcState,
    *,
    solver: Solver | None = None,
) -> tuple[MpcCommand, MpcState]:
    """One controller tick; see the module docstring for the exact order.

    ``solver`` overrides the registry lookup by ``cfg.solver`` (tests inject
    failing solvers). Raises ``TypeError`` only for arguments of the wrong
    *type*; every runtime problem -- malformed temperatures, bad time, a
    solver that throws or returns garbage -- becomes ``mode=fallback``.
    """
    if not isinstance(obs, PlantObservation):
        raise TypeError(f"obs must be a PlantObservation, got {type(obs).__name__}")
    if not isinstance(cfg, MpcConfig):
        raise TypeError(f"cfg must be an MpcConfig, got {type(cfg).__name__}")
    if not isinstance(state, MpcState):
        raise TypeError(f"state must be an MpcState, got {type(state).__name__}")

    mem: dict[str, Any] = dict(state.solver_memory)

    # 1. prev
    prev, prev_source = resolve_prev(state, obs, cfg)

    # 2. time
    last_ts = mem.get("last_ts")
    last_ts = float(last_ts) if _finite(last_ts) else None
    time_status, slew_limit, drop_history = _time_check(obs.ts, last_ts, cfg)
    window = () if drop_history else state.window
    last_raw = None if drop_history else state.last_raw_temps

    # 3. gate
    latch_raw = mem.get("stuck_latch")
    stuck_latch = (
        {k: float(v) for k, v in latch_raw.items() if isinstance(k, str) and _finite(v)}
        if isinstance(latch_raw, Mapping)
        else {}
    )
    gate: GateResult = evaluate_gate(
        obs,
        cfg,
        last_good_obs=state.last_good_obs,
        last_raw_temps=last_raw,
        window=window,
        dT_limit=slew_limit,
        stuck_latch=stuck_latch,
    )
    trusted = gate.trusted and time_status in ("ok", "first")

    # 4. fault bookkeeping
    fault_since = state.fault_since_ts
    fault_reason = state.fault_reason
    if trusted:
        streak = state.trusted_streak + 1
    else:
        streak = 0
        if fault_since is None:
            fault_since = obs.ts
        fault_reason = FaultReason.SENSOR_GATE

    # 5. solver
    integrator: dict[str, float] = dict(state.integrator)
    solver_obj = SOLVERS.get(cfg.solver) if solver is None else solver
    solver_name = getattr(solver_obj, "name", str(cfg.solver.value)) if solver_obj else "none"
    solver_mem: dict[str, Any] = dict(mem.get(solver_name) or {})
    target: dict[str, float] | None = None
    policy = "hold"
    solver_error: str | None = None
    solver_diag: dict[str, Any] = {}
    returning = False
    ran_solver = False

    run_solver = trusted and (fault_since is None or streak >= cfg.confirm_ticks)
    if run_solver:
        ran_solver = True
        was_in_fault = fault_since is not None
        temps = {name: float(gate.filtered[name]) for name in cfg.temps}  # type: ignore[arg-type]
        req = SolverRequest(
            temps=temps, prev_pwm=dict(prev), integrator=integrator, memory=solver_mem
        )
        try:
            if solver_obj is None:
                raise SolverFault(f"no solver registered for {cfg.solver.value!r}")
            if was_in_fault or set(integrator) != set(cfg.channels):
                # Bumpless transfer: first auto output == prev before the rate limit.
                returning = True
                init_i, init_m = solver_obj.initialise(cfg, req)
                req = dataclasses.replace(req, integrator=dict(init_i), memory=dict(init_m))
            result = _check_result(solver_obj.solve(cfg, req), cfg)
        except Exception as exc:  # every solver failure is a fault, never a raise out of step
            solver_error = f"{type(exc).__name__}: {exc}"[:200]
        if solver_error is None:
            target = {ch: float(result.pwm[ch]) for ch in cfg.channels}
            integrator = {ch: float(v) for ch, v in result.integrator.items()}
            solver_mem = dict(result.memory)
            solver_diag = dict(result.diagnostics) if _json_finite(result.diagnostics) else {}
            fault_since = None
            fault_reason = None
            policy = "solver"
        else:
            returning = False
            if fault_since is None:
                fault_since = obs.ts
            fault_reason = FaultReason.SOLVER
            streak = 0

    # 6. fallback policy
    fault_ticks = mem.get("fault_ticks", 0)
    fault_ticks = int(fault_ticks) if _finite(fault_ticks) else 0
    elapsed = 0.0
    if fault_since is not None:
        fault_ticks += 1
        elapsed = max(obs.ts - fault_since, (fault_ticks - 1) * cfg.dt, 0.0)
        if elapsed > cfg.fallback_hold_s:
            # Never below what is already on the fans (module docstring, step 6).
            target = {ch: max(prev[ch], cfg.fallback_pwm[ch]) for ch in cfg.channels}
            policy = "ramp_high"
        else:
            target = dict(prev)
            policy = "hold"
    else:
        fault_ticks = 0
    if target is None:  # unreachable: every branch above sets it; hold is the safe default
        target = dict(prev)

    # 7. rate limit vs prev, then clamp
    pwm: dict[str, float] = {}
    rate_limited: dict[str, bool] = {}
    for ch in cfg.channels:
        want = target[ch]
        limited = _clamp(want, prev[ch] - cfg.d_pwm_max, prev[ch] + cfg.d_pwm_max)
        rate_limited[ch] = limited != want
        pwm[ch] = _clamp(limited, cfg.pwm_min, cfg.pwm_max)

    # 8. mode
    saturated = {
        ch: target[ch] > cfg.pwm_max + _EPS and pwm[ch] >= cfg.pwm_max - _EPS for ch in cfg.channels
    }
    if fault_since is not None:
        mode = Mode.FALLBACK
    elif any(saturated.values()):
        mode = Mode.SATURATED
    else:
        mode = Mode.AUTO

    # fan stall bookkeeping
    stall_counts = mem.get("stall_ticks")
    stall_ticks = _stall_update(
        stall_counts if isinstance(stall_counts, Mapping) else {}, obs, state, cfg
    )
    fan_stall = {ch: n >= STALL_TICKS for ch, n in stall_ticks.items()}

    diagnostics: dict[str, Any] = {
        "trusted": trusted,
        "gate": gate.to_dict(),
        "time": {
            "status": time_status,
            "dt_obs": None if last_ts is None else obs.ts - last_ts,
            "dT_limit": slew_limit,
        },
        "prev_source": prev_source,
        "prev_pwm": dict(prev),
        "obs_pwm_usable": obs_pwm_usable(obs, cfg),
        "target_pwm": dict(target),
        "rate_limited": rate_limited,
        "saturated": saturated,
        "policy": policy,
        "fault_reason": None if fault_reason is None else fault_reason.value,
        "fault_since_ts": fault_since,
        "fault_elapsed_s": elapsed,
        "fault_ticks": fault_ticks,
        "trusted_streak": streak,
        "confirm_ticks": cfg.confirm_ticks,
        "returning_to_auto": returning and fault_since is None and state.fault_since_ts is not None,
        "solver": solver_name,
        "solver_ran": ran_solver,
        "solver_error": solver_error,
        "solver_diag": solver_diag,
        "fan_stall": fan_stall,
        "median3": cfg.median3,
    }
    cmd = MpcCommand(pwm=pwm, mode=mode, diagnostics=diagnostics)

    # 9. next state (gate rule 5: raw values and cmd.pwm always go into the window)
    mem["last_ts"] = obs.ts
    mem["fault_ticks"] = fault_ticks
    mem["stall_ticks"] = stall_ticks
    mem["stuck_latch"] = dict(gate.stuck_latch)
    mem[solver_name] = solver_mem
    good_now = trusted and fault_since is None
    if good_now:
        # last_good holds what the gate trusted: the filtered temperatures (with
        # median3 the raw sample may hide a Jump the median has not shown yet),
        # and rpm/pwm with NaN/inf replaced by None so the state stays finite.
        good_obs = PlantObservation(
            temps=dict(gate.filtered),
            rpm={k: (v if _finite(v) else None) for k, v in obs.rpm.items()},
            pwm={k: (v if _finite(v) else None) for k, v in obs.pwm.items()},
            ts=obs.ts,
        )
    else:
        good_obs = state.last_good_obs
    new_state = MpcState(
        last_cmd=cmd,
        last_good_obs=good_obs,
        last_raw_temps=dict(gate.raw),
        window=push_window(window, gate.raw, pwm, cfg.stuck_ticks),
        fault_since_ts=fault_since,
        fault_reason=fault_reason,
        trusted_streak=streak,
        integrator=integrator,
        solver_memory=mem,
    )
    return cmd, new_state
