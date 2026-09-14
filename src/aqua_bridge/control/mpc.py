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
   leaves the ``stuck_eps_c`` band proves the sensor alive. (With zones the
   decimated Stuck windows are advanced first and dropped on a gap, too.)
3b. Sensor confirmation (zones only): a sensor whose value the gate rejected
   (``range``, ``slew``, ``stuck``) first confirms over ``confirm_ticks``
   consecutive trusted ticks (:func:`aqua_bridge.control.zones.advance_confirmation`,
   counts in ``solver_memory["sensor_confirm"]``, ``diagnostics["sensor_confirm"]``):
   a confirming sensor is not fused by the estimator, not in the solver's
   ``temps``, not a last good value, and counts toward its group's trust (step
   3d) only while its zone is already in fault, so a redundant member that jumps
   stays out while its group stays trusted through the others.
3b'. With zones, a model loaded from the store (``solver_memory["store_seed"]``, put
   into the initial state by :mod:`aqua_bridge.modelstore`) is applied once, on the
   first zoned tick, by :func:`aqua_bridge.control.persist.apply_seed` with this tick's
   ``ts``: the thermal memory (``frozen`` from a fresh file, a stale hold from an old
   one), the SMART calibrations and the fan curves go into ``solver_memory``; what was
   loaded stays in ``solver_memory["store"]`` and is ``diagnostics["store"]`` on every
   zoned tick. Never a raise, never a fault: whatever fails its checks starts at the
   prior with a warning.
3c. With zones, the estimator (:func:`aqua_bridge.control.estimator.update`,
   plan section 6 step 5a) runs every tick, fault ticks included, on the
   gate-trusted, confirmed temperatures of this tick (none on a tick whose time
   status is not ``first`` / ``ok``), the command ``prev`` and
   ``obs.inputs["smart"]``; its memory is ``solver_memory["estimator"]`` and
   its estimates block feeds the solver and the diagnostics. Its inputs depend
   on no zone verdict, so it runs before zone trust. Any exception from it is an
   *estimator fault*: its memory is dropped (the next tick starts over), the
   diagnostics fall back to the prior map
   (:func:`aqua_bridge.control.estimates.prior_estimates`) and, when the
   solver regulates on the estimates (``cfg.regulates_drive_limits``), every
   zone with a constrained bay (``occupied: true`` / ``auto``) is untrusted
   this tick with reason ``estimator`` and fault reason ``solver`` (without
   estimates nothing in it can be constrained). A zoned config that still
   regulates on setpoints does not read the estimates, so there the fault is
   only reported.
3d. Zone trust (:func:`aqua_bridge.control.zones.evaluate`). Legacy mode is
   one implicit zone whose verdict is exactly the whole-tick gate verdict.
   ``zones.trust_rule: strict`` checks the required sensor groups;
   ``sigma`` reads this tick's estimator update (step 3c), so the verdict uses
   the posterior sigma of the same tick the solver acts on. On a tick with an
   estimator fault ``sigma`` applies ``strict``; ``diagnostics["trust_rule"]`` is
   the rule that ran.
4. Fault bookkeeping, per zone: an untrusted tick resets the zone's streak
   and opens its fault timer (``since`` is kept, never restarted, while
   the zone's fault is active -- Flicker must not reset the hold). A
   trusted tick while in fault only counts toward ``confirm_ticks``. A
   zone is *eligible* for the solver when it is trusted and either
   fault-free or on its ``confirm_ticks``-th consecutive trusted tick with a
   confirmed trusted member in every required group its trust rule checks
   (``sigma``: the setpoint groups only).
4b. Sigma floor (``zones.trust_rule: sigma`` only): an eligible zone whose
   required sensor groups (the ``strict`` groups) are not all held by a
   gate-trusted, confirmed member runs on the estimator's growing sigma; while
   it does, the solver's demand on every channel of its reach is raised to at
   least ``prev`` (``diagnostics["sigma_floor_channels"]``). The estimator
   cannot see the heat a lost sensor would have shown, and ``k * sigma`` grows
   slowly, so without the floor a solver that had followed the measured warming
   (the DAS MPC) lowers the fans the moment the sensor goes (PROJECT.md
   section 3 per-zone trust). The floor is released with the sensor.
5. Solver -- only when some zone is eligible. The channels of the zones
   that stay in fault (and, with ``fault_coupling: declared``, of the zones
   coupled to them) are under fallback policy: they reach the solver as
   ``SolverRequest.fixed_channels`` and only the sensors of eligible zones
   (plus trusted sensors without a zone) reach it as ``temps``, and the
   estimates of the bays of eligible zones as ``estimates``. On a
   returning tick (legacy: the zone was in fault or the integrator is
   incomplete; zones: a channel the solver drives lacks an integrator
   entry, which is what a channel released from fallback policy or from
   a manual override looks like) the solver is re-initialised so that its
   first output equals ``prev`` before the rate limit (bumpless). Any
   exception, non-finite / wrong-shaped output, ``converged=False`` or
   ``iterations > solver_max_iter`` is a ``solver`` fault of every
   eligible zone, handled exactly like a gate fault: same timer, same
   hold / ramp-high policy, and the fault is *not* cleared (the old
   ``since`` survives a failed retry so a persistently broken solver still
   ramps high). A solver fault also restarts the confirmation of every
   zone, so the zones confirm together afterwards (a solver that refuses
   to run beside a faulted zone would otherwise meet zones confirming out
   of phase forever). When every channel stays under fallback policy the
   solver is not called and the eligible zones' own faults clear.
6. Fallback policy per channel while a zone that reaches it is in fault:
   hold ``prev`` until that zone's fault has lasted longer than
   ``fallback_hold_s`` (the oldest such zone when several reach it), then
   target ``max(prev, cfg.fallback_pwm)``. Section 3 item 2 says
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
8. Mode: ``fallback`` while every zone is in fault (legacy: while a fault
   is active); ``degraded`` while some but not all zones are; ``saturated``
   when a channel's demand exceeds ``pwm_max`` *and* the emitted PWM sits
   at ``pwm_max`` (honest: pinned at the rail); ``auto`` otherwise.
8b. With zones and ``model_shadow: true``, the thermal model's online
   identification (:func:`aqua_bridge.control.thermal.update`) runs after the
   command is final, so it learns and predicts without acting: it reads this
   tick's gate-trusted temperatures, ``prev``, ``obs.rpm``, the zones that are
   trusted and not in fault (only their windows accumulate) and the estimator's
   occupancy, class and accepted sensor map per bay; its memory is
   ``solver_memory["thermal"]`` and ``diagnostics["thermal"]`` its summary
   (status, prediction error, coefficients). Any exception resets the memory to
   the prior with ``status: error`` (never a raise, never a fault; the DAS MPC,
   which reads this memory on the next tick, falls back to its PI-like DAS
   form on a model in error; a pending stale hold survives that reset, so a stale
   model still has to re-confirm). Without ``model_shadow`` nothing runs and
   neither key exists.

Zones and the global fields
---------------------------
Legacy mode keeps ``fault_since_ts`` / ``fault_reason`` / ``trusted_streak``
and ``solver_memory["fault_ticks"]`` exactly as before and leaves
``MpcState.zone_faults`` empty. With zones, ``zone_faults`` holds one
:class:`~aqua_bridge.model.ZoneFault` per zone and the global fields are
aggregates: the earliest zone fault and its reason, the smallest streak,
the largest fault tick count. ``MpcState.in_fault`` is therefore true in
``degraded`` as well as ``fallback``. Diagnostics add ``zones`` (per zone:
trusted, reasons, fault timer, ``in_closure``, channels, policy),
``zones_in_fault``, ``fallback_channels``, ``policy_by_channel``,
``trust_rule``, ``estimates`` (per constrained bay with an estimate, any
zone: ``t_c``, ``sigma_c``, ``margin_c``, ``soft_c``, ``hard_c``, ``limit_c``,
``limit_margin_c`` (``hard_c - t_c``), ``occupancy``, ``class``, ``zone``,
``zone_trusted``, ``calibrated``,
``source`` and, from the estimator, ``q_w``), ``bays`` (every bay: occupancy,
class, serial association, calibration, association candidates; see
:class:`~aqua_bridge.control.estimator.EstimatorUpdate`) and ``estimator``
(``status`` ``ok`` | ``error``, ``error``, per zone air estimate, SMART
counters) and ``noise`` (:func:`aqua_bridge.control.noise.noise_diagnostics`:
``db_index`` from the fans' speed, ``db_index_cmd`` at the new command, per channel
rpm and its source); ``policy`` becomes ``mixed`` when channels differ. Without
``setpoints`` (:attr:`~aqua_bridge.model.MpcConfig.regulates_drive_limits`) the solver
comes from :data:`DAS_SOLVERS`: ``pi`` regulates the margin deficit of the drives
(PI-like DAS form, :mod:`aqua_bridge.control.solver_pi`) and ``mpc`` is the DAS MPC
(:mod:`aqua_bridge.control.solver_das`), which plans beside zones in fault with
their channels as known inputs. Its request also carries ``ts``, the estimator's
zones and bays (``plant``) and, with ``model_shadow``, the thermal memory of the
previous tick (``thermal``). A zoned config that still declares setpoints keeps
the per-zone setpoint regulation: ``pi`` skips the fixed channels and runs per zone;
the legacy ``mpc`` has one coupled problem over every channel and raises on fixed
channels, so with zones it turns any zone fault into a fault of every zone it drives
(whole-enclosure fallback, never less cooling).

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
degrees C, only latched temperatures; gate rule 3), with decimated Stuck
windows ``stuck_slow`` and ``stuck_seq`` (gate module docstring), with zones
``estimator`` (step 3c), ``sensor_confirm`` (dict temperature -> consecutive
trusted ticks, confirming sensors only; step 3b) and, with ``model_shadow``,
``thermal`` (step 8b), with a model store ``store`` and ``fan_curves`` (step
3b'), and one sub-dict per solver under ``solver.name``.
"""

from __future__ import annotations

import dataclasses
import json
import math
from collections.abc import Mapping
from dataclasses import dataclass
from typing import Any

from aqua_bridge.control import estimates, estimator, noise, persist, thermal, zones
from aqua_bridge.control.gate import (
    GateResult,
    advance_slow_windows,
    evaluate_gate,
    push_window,
)
from aqua_bridge.control.solver_das import DasMpcSolver
from aqua_bridge.control.solver_mpc import MpcSolver
from aqua_bridge.control.solver_pi import PiSolver, Solver, SolverRequest, SolverResult
from aqua_bridge.model import (
    STORE_KEY,
    STORE_SEED_KEY,
    FaultReason,
    Mode,
    MpcCommand,
    MpcConfig,
    MpcState,
    PlantObservation,
    SolverKind,
    ZoneFault,
)

__all__ = [
    "DAS_SOLVERS",
    "GAP_TICKS_MAX",
    "SOLVERS",
    "STALL_TICKS",
    "SolverFault",
    "obs_pwm_usable",
    "resolve_prev",
    "solver_for",
    "step",
]

#: An observation later than this many ``dt`` after the previous one is a "gap" tick.
GAP_TICKS_MAX = 3.0
#: Consecutive zero-RPM ticks (with a high command) before ``fan_stall`` is flagged.
STALL_TICKS = 10

#: Solver registry by ``cfg.solver`` (section 3: "first step may be PI; swap in MPC later").
SOLVERS: dict[SolverKind, Solver] = {SolverKind.PI: PiSolver(), SolverKind.MPC: MpcSolver()}
#: Registry for a zoned config that regulates drive limits (``cfg.regulates_drive_limits``):
#: ``pi`` is the same PI solver in its margin-deficit form, ``mpc`` the DAS MPC.
DAS_SOLVERS: dict[SolverKind, Solver] = {
    SolverKind.PI: SOLVERS[SolverKind.PI],
    SolverKind.MPC: DasMpcSolver(),
}


def solver_for(cfg: MpcConfig) -> Solver | None:
    """The registered solver of ``cfg.solver``: :data:`DAS_SOLVERS` when the config
    regulates drive limits, else :data:`SOLVERS` (legacy and zoned setpoint configs)."""
    registry = DAS_SOLVERS if cfg.regulates_drive_limits else SOLVERS
    return registry.get(cfg.solver)


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


@dataclass
class _ZoneBook:
    """Mutable per-zone fault bookkeeping inside one ``step`` call."""

    since: float | None
    reason: FaultReason | None
    streak: int
    ticks: int


def _legacy_fault_ticks(mem: Mapping[str, Any]) -> int:
    fault_ticks = mem.get("fault_ticks", 0)
    return int(fault_ticks) if _finite(fault_ticks) else 0


def _read_books(state: MpcState, mem: Mapping[str, Any], cfg: MpcConfig) -> dict[str, _ZoneBook]:
    """Zone bookkeeping as it stands before this tick.

    Legacy mode reads the single implicit zone from the global fields exactly
    as before. With zones, a zone missing from ``state.zone_faults`` while the
    global fault is active (a state from another config) starts in fault with
    a zero streak: it has to confirm like any other faulted zone.
    """
    layout = cfg.zone_layout
    if layout.implicit:
        return {
            layout.zones[0]: _ZoneBook(
                since=state.fault_since_ts,
                reason=state.fault_reason,
                streak=state.trusted_streak,
                ticks=_legacy_fault_ticks(mem),
            )
        }
    books: dict[str, _ZoneBook] = {}
    for zone in layout.zones:
        zf = state.zone_faults.get(zone)
        if zf is not None:
            books[zone] = _ZoneBook(zf.since_ts, zf.reason, zf.streak, zf.ticks)
        elif state.fault_since_ts is not None:
            books[zone] = _ZoneBook(
                since=state.fault_since_ts,
                reason=state.fault_reason or FaultReason.SENSOR_GATE,
                streak=0,
                ticks=_legacy_fault_ticks(mem),
            )
        else:
            books[zone] = _ZoneBook(None, None, state.trusted_streak, 0)
    return books


def _fault_elapsed(book: _ZoneBook, ts: float, cfg: MpcConfig, ticks: int) -> float:
    """Fault age: wall time, but never less than the tick count says (a broken clock)."""
    assert book.since is not None
    return max(ts - book.since, (ticks - 1) * cfg.dt, 0.0)


def _overall_policy(policy_by_channel: Mapping[str, str]) -> str:
    kinds = set(policy_by_channel.values())
    return kinds.pop() if len(kinds) == 1 else "mixed"


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
    solver that throws or returns garbage -- becomes ``mode=fallback``
    (or, with zones, fallback policy on the zones concerned).
    """
    if not isinstance(obs, PlantObservation):
        raise TypeError(f"obs must be a PlantObservation, got {type(obs).__name__}")
    if not isinstance(cfg, MpcConfig):
        raise TypeError(f"cfg must be an MpcConfig, got {type(cfg).__name__}")
    if not isinstance(state, MpcState):
        raise TypeError(f"state must be an MpcState, got {type(state).__name__}")

    mem: dict[str, Any] = dict(state.solver_memory)
    layout = cfg.zone_layout
    das = not layout.implicit
    zone_names = layout.zones

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
    slow_windows: dict[str, list[dict[str, Any]]] = {}
    stuck_seq = 0
    if cfg.slow_window_samples:
        seq_raw = mem.get("stuck_seq")
        valid_seq = isinstance(seq_raw, int) and not isinstance(seq_raw, bool) and seq_raw >= 0
        if not drop_history and valid_seq:
            stuck_seq = int(seq_raw)  # type: ignore[arg-type]
            slow_in = mem.get("stuck_slow")
        else:
            slow_in = None
        slow_windows = advance_slow_windows(slow_in, window, stuck_seq, cfg)
    gate: GateResult = evaluate_gate(
        obs,
        cfg,
        last_good_obs=state.last_good_obs,
        last_raw_temps=last_raw,
        window=window,
        dT_limit=slew_limit,
        stuck_latch=stuck_latch,
        slow_windows=slow_windows or None,
    )
    trusted = gate.trusted and time_status in ("ok", "first")

    # 3b. with zones a sensor whose value the gate rejected confirms first (module docstring)
    books = _read_books(state, mem, cfg)
    confirming = zones.advance_confirmation(mem.get("sensor_confirm"), gate, time_status, cfg)
    trusted_temps: dict[str, float] = {}
    if das:
        trusted_temps = {
            name: float(gate.filtered[name])  # type: ignore[arg-type]
            for name in cfg.temps
            if gate.per_temp[name] and time_status in ("ok", "first") and name not in confirming
        }

    # 3b'. a model loaded from the store, applied once (zones only; module docstring)
    if das and STORE_SEED_KEY in mem:
        persist.apply_seed(mem, cfg, mem.pop(STORE_SEED_KEY), obs.ts)

    # 3c. estimator (zones only; module docstring)
    est_block: dict[str, dict[str, Any]] = {}
    est_update: estimator.EstimatorUpdate | None = None
    estimator_error: str | None = None
    estimator_faulted: set[str] = set()
    if das:
        try:
            est_update = estimator.update(
                mem.get("estimator"),
                cfg,
                temps=trusted_temps,
                u=prev,
                ts=obs.ts,
                smart=obs.inputs.get("smart"),
            )
            est_block = est_update.estimates
        except Exception as exc:  # an estimator failure is a fault, never a raise out of step
            estimator_error = f"{type(exc).__name__}: {exc}"[:200]
            est_block = estimates.prior_estimates(cfg, trusted_temps)
            if cfg.regulates_drive_limits and cfg.topology is not None:
                constrained = {b.zone for b in cfg.topology.bays.values() if b.constrained}
                estimator_faulted = {z for z in zone_names if z in constrained}

    # 3d. zone trust (legacy: the implicit zone's verdict is exactly ``trusted``), after the
    # estimator: the sigma rule reads this tick's posterior sigma (module docstring)
    trust_rule = zones.effective_trust_rule(cfg, est_update)
    verdicts = zones.evaluate(
        gate,
        time_status,
        cfg,
        est_update,
        confirming=confirming,
        in_fault=[z for z in zone_names if books[z].since is not None],
    )
    if das:
        for zone in estimator_faulted:
            verdicts[zone] = zones.ZoneTrust(
                trusted=False,
                reasons=(*verdicts[zone].reasons, f"estimator:{estimator_error}"),
            )

    # 4. fault bookkeeping per zone
    for zone in zone_names:
        book = books[zone]
        if verdicts[zone].trusted:
            book.streak += 1
        else:
            book.streak = 0
            if book.since is None:
                book.since = obs.ts
            book.reason = (
                FaultReason.SOLVER if zone in estimator_faulted else FaultReason.SENSOR_GATE
            )
    eligible = [
        z
        for z in zone_names
        if verdicts[z].trusted
        and (
            books[z].since is None
            or (
                books[z].streak >= cfg.confirm_ticks
                and zones.groups_confirmed(z, gate, confirming, cfg, trust_rule)
            )
        )
    ]
    # 4b. sigma floor (``trust_rule: sigma``; module docstring): an eligible zone with a
    # required sensor group that has no confirmed trusted member keeps the channels of its
    # reach at or above ``prev`` while the solver drives them
    sigma_floor: set[str] = set()
    if das and trust_rule == "sigma":
        degraded = [
            z for z in eligible if not zones.groups_confirmed(z, gate, confirming, cfg, "strict")
        ]
        sigma_floor = set(zones.fallback_channels(degraded, cfg))
    # Zones that stay in fault whatever the solver does, and their channels.
    remaining = [z for z in zone_names if z not in eligible]
    fixed_pre = set(zones.fallback_channels(remaining, cfg))
    active = [ch for ch in cfg.channels if ch not in fixed_pre]

    # 5. solver
    integrator: dict[str, float] = dict(state.integrator)
    if das:  # a channel under fallback policy re-initialises bumplessly when released
        integrator = {ch: v for ch, v in integrator.items() if ch not in fixed_pre}
    solver_obj = solver_for(cfg) if solver is None else solver
    solver_name = getattr(solver_obj, "name", str(cfg.solver.value)) if solver_obj else "none"
    solver_mem: dict[str, Any] = dict(mem.get(solver_name) or {})
    result_pwm: dict[str, float] | None = None
    solver_error: str | None = None
    solver_diag: dict[str, Any] = {}
    returning = False
    ran_solver = False

    if eligible and not active:
        # Every channel stays under fallback policy (coupled to a zone still in
        # fault): nothing for the solver to drive; the eligible zones' own
        # sensor faults are confirmed over.
        for zone in eligible:
            books[zone].since = None
            books[zone].reason = None
    elif eligible:
        ran_solver = True
        was_in_fault = any(books[z].since is not None for z in eligible)
        if das:
            usable_zones = set(eligible)
            temps = {
                name: float(gate.filtered[name])  # type: ignore[arg-type]
                for name in cfg.temps
                if gate.per_temp[name]
                and name not in confirming
                and (layout.sensor_zone[name] is None or layout.sensor_zone[name] in usable_zones)
            }
            elapsed_pre = {
                z: _fault_elapsed(books[z], obs.ts, cfg, books[z].ticks + 1) for z in remaining
            }
            ch_elapsed_pre = zones.channel_fallback_elapsed(elapsed_pre, cfg)
            req = SolverRequest(
                temps=temps,
                prev_pwm=dict(prev),
                integrator=integrator,
                memory=solver_mem,
                fixed_channels={
                    ch: zones.fallback_target(
                        prev[ch], cfg.fallback_pwm[ch], ch_elapsed_pre[ch], cfg
                    )[0]
                    for ch in cfg.channels
                    if ch in fixed_pre
                },
                zone_trust={z: z in usable_zones for z in zone_names},
                estimates={b: e for b, e in est_block.items() if e["zone"] in usable_zones},
                occupancy=(
                    {}
                    if est_update is None
                    else {b: info["occupancy"] for b, info in est_update.bays.items()}
                ),
                ts=obs.ts,
                thermal=mem.get("thermal") if cfg.model_shadow else None,
                plant=_plant_view(est_block, est_update),
            )
        else:
            temps = {name: float(gate.filtered[name]) for name in cfg.temps}  # type: ignore[arg-type]
            req = SolverRequest(
                temps=temps, prev_pwm=dict(prev), integrator=integrator, memory=solver_mem
            )
        try:
            if solver_obj is None:
                raise SolverFault(f"no solver registered for {cfg.solver.value!r}")
            if das:
                need_init = any(ch not in integrator for ch in active)
            else:
                need_init = was_in_fault or set(integrator) != set(cfg.channels)
            if need_init:
                # Bumpless transfer: first auto output == prev before the rate limit
                # (with zones: on the channels that lack an integrator entry).
                returning = True
                init_i, init_m = solver_obj.initialise(cfg, req)
                if das:
                    merged = dict(integrator)
                    merged.update({ch: init_i[ch] for ch in active if ch not in integrator})
                    req = dataclasses.replace(req, integrator=merged, memory=dict(init_m))
                else:
                    req = dataclasses.replace(req, integrator=dict(init_i), memory=dict(init_m))
            result = _check_result(solver_obj.solve(cfg, req), cfg)
        except Exception as exc:  # every solver failure is a fault, never a raise out of step
            solver_error = f"{type(exc).__name__}: {exc}"[:200]
        if solver_error is None:
            result_pwm = {ch: float(result.pwm[ch]) for ch in cfg.channels}
            integrator = {ch: float(v) for ch, v in result.integrator.items()}
            solver_mem = dict(result.memory)
            solver_diag = dict(result.diagnostics) if _json_finite(result.diagnostics) else {}
            for zone in eligible:
                books[zone].since = None
                books[zone].reason = None
        else:
            # A solver fault faults every zone it was asked to drive: same timer,
            # same hold / ramp-high policy, the old since survives a failed retry.
            # Every zone restarts its confirmation, so zones re-confirm together
            # (otherwise a solver that cannot run beside a faulted zone, like the
            # legacy mpc, meets the zones confirming out of phase forever).
            returning = False
            for zone in eligible:
                book = books[zone]
                if book.since is None:
                    book.since = obs.ts
                book.reason = FaultReason.SOLVER
            for zone in zone_names:
                books[zone].streak = 0

    # 6. fallback policy per channel from the zone timers
    faulted = [z for z in zone_names if books[z].since is not None]
    elapsed_by_zone: dict[str, float] = {}
    for zone in zone_names:
        book = books[zone]
        if book.since is not None:
            book.ticks += 1
            elapsed_by_zone[zone] = _fault_elapsed(book, obs.ts, cfg, book.ticks)
        else:
            book.ticks = 0
    ch_elapsed = zones.channel_fallback_elapsed(elapsed_by_zone, cfg)
    target: dict[str, float] = {}
    policy_by_channel: dict[str, str] = {}
    for ch in cfg.channels:
        if ch in ch_elapsed:
            # Never below what is already on the fans (module docstring, step 6).
            target[ch], policy_by_channel[ch] = zones.fallback_target(
                prev[ch], cfg.fallback_pwm[ch], ch_elapsed[ch], cfg
            )
        elif result_pwm is not None:
            want = result_pwm[ch]
            if ch in sigma_floor:  # never below what is already on the fans (step 4b)
                want = max(want, prev[ch])
            target[ch], policy_by_channel[ch] = want, "solver"
        else:  # unreachable: a channel outside fallback policy had a solver result; hold
            target[ch], policy_by_channel[ch] = prev[ch], "hold"
    policy = _overall_policy(policy_by_channel)
    elapsed = max(elapsed_by_zone.values(), default=0.0)

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
    if faulted and len(faulted) == len(zone_names):
        mode = Mode.FALLBACK
    elif faulted:
        mode = Mode.DEGRADED
    elif any(saturated.values()):
        mode = Mode.SATURATED
    else:
        mode = Mode.AUTO

    # global aggregates (legacy: the implicit zone itself)
    if das:
        first_fault = min(
            (
                (books[z].since, i, z)
                for i, z in enumerate(zone_names)
                if books[z].since is not None
            ),
            default=None,
        )
        fault_since = None if first_fault is None else first_fault[0]
        fault_reason = None if first_fault is None else books[first_fault[2]].reason
        streak = min(books[z].streak for z in zone_names)
        fault_ticks = max(books[z].ticks for z in zone_names)
    else:
        only = books[zone_names[0]]
        fault_since, fault_reason, streak, fault_ticks = (
            only.since,
            only.reason,
            only.streak,
            only.ticks,
        )

    # 8b. thermal model identification in shadow (zones only; module docstring)
    thermal_summary: dict[str, Any] | None = None
    if das and cfg.model_shadow:
        thermal_summary = _thermal_shadow(
            mem, obs, cfg, trusted_temps, prev, est_update, verdicts, faulted
        )
    else:
        mem.pop("thermal", None)

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
    if das:
        in_closure = set(zones.closure(faulted, cfg))
        zone_diag: dict[str, Any] = {}
        for zone in zone_names:
            book = books[zone]
            if book.since is not None:
                zone_policy = "ramp_high" if elapsed_by_zone[zone] > cfg.fallback_hold_s else "hold"
            elif zone in in_closure:
                zone_policy = "coupled"
            else:
                zone_policy = "solver"
            zone_diag[zone] = {
                "trusted": verdicts[zone].trusted,
                "reasons": list(verdicts[zone].reasons),
                "fault": book.since is not None,
                "fault_reason": None if book.reason is None else book.reason.value,
                "fault_since_ts": book.since,
                "fault_elapsed_s": elapsed_by_zone.get(zone, 0.0),
                "fault_ticks": book.ticks,
                "trusted_streak": book.streak,
                "in_closure": zone in in_closure,
                "channels": list(layout.zone_channels[zone]),
                "policy": zone_policy,
            }
        diagnostics["zones"] = zone_diag
        diagnostics["zones_in_fault"] = list(faulted)
        diagnostics["fallback_channels"] = [ch for ch in cfg.channels if ch in ch_elapsed]
        diagnostics["policy_by_channel"] = policy_by_channel
        diagnostics["sigma_floor_channels"] = [
            ch for ch in cfg.channels if ch in sigma_floor and policy_by_channel[ch] == "solver"
        ]
        diagnostics["trust_rule"] = trust_rule
        diagnostics["sensor_confirm"] = dict(confirming)
        diagnostics["estimates"] = _estimate_diagnostics(est_block, verdicts, faulted)
        diagnostics["bays"] = {} if est_update is None else est_update.bays
        diagnostics["estimator"] = {
            "status": "ok" if estimator_error is None else "error",
            "error": estimator_error,
            "zones": {} if est_update is None else est_update.zones,
            **({} if est_update is None else est_update.summary),
        }
        if thermal_summary is not None:
            diagnostics["thermal"] = thermal_summary
        if isinstance(mem.get(STORE_KEY), Mapping):
            diagnostics["store"] = mem[STORE_KEY]
        diagnostics["noise"] = noise.noise_diagnostics(cfg, prev=prev, pwm=pwm, rpm=obs.rpm)
    cmd = MpcCommand(pwm=pwm, mode=mode, diagnostics=diagnostics)

    # 9. next state (gate rule 5: raw values and cmd.pwm always go into the window)
    mem["last_ts"] = obs.ts
    mem["fault_ticks"] = fault_ticks
    mem["stall_ticks"] = stall_ticks
    mem["stuck_latch"] = dict(gate.stuck_latch)
    if das:
        mem["sensor_confirm"] = dict(confirming)
    else:
        mem.pop("sensor_confirm", None)
    if cfg.slow_window_samples:
        mem["stuck_slow"] = slow_windows
        mem["stuck_seq"] = stuck_seq + 1
    else:
        mem.pop("stuck_slow", None)
        mem.pop("stuck_seq", None)
    if est_update is not None:
        mem["estimator"] = est_update.memory
    else:  # legacy mode, or an estimator fault: the next tick starts over
        mem.pop("estimator", None)
    mem[solver_name] = solver_mem
    if das:
        integrator = {ch: v for ch, v in integrator.items() if ch not in ch_elapsed}
    good_obs = _next_good_obs(
        obs, cfg, state, gate, trusted, fault_since, faulted, time_status, confirming
    )
    new_state = MpcState(
        last_cmd=cmd,
        last_good_obs=good_obs,
        last_raw_temps=dict(gate.raw),
        window=push_window(window, gate.raw, pwm, cfg.window_ticks),
        fault_since_ts=fault_since,
        fault_reason=fault_reason,
        trusted_streak=streak,
        integrator=integrator,
        solver_memory=mem,
        zone_faults=(
            {
                z: ZoneFault(
                    since_ts=books[z].since,
                    reason=books[z].reason,
                    streak=books[z].streak,
                    ticks=books[z].ticks,
                )
                for z in zone_names
            }
            if das
            else {}
        ),
    )
    return cmd, new_state


def _thermal_shadow(
    mem: dict[str, Any],
    obs: PlantObservation,
    cfg: MpcConfig,
    trusted_temps: Mapping[str, float],
    prev: Mapping[str, float],
    est_update: estimator.EstimatorUpdate | None,
    verdicts: Mapping[str, zones.ZoneTrust],
    faulted: list[str],
) -> dict[str, Any]:
    """Step 8b: advance ``mem["thermal"]`` in place and return its summary (module docstring)."""
    occupancy: dict[str, str] | None = None
    try:
        classes: dict[str, str] | None = None
        maps: dict[str, tuple[float, float]] = {}
        if est_update is not None:
            occupancy = {b: str(info["occupancy"]) for b, info in est_update.bays.items()}
            classes = {b: str(info["class"]) for b, info in est_update.bays.items()}
            for b, info in est_update.bays.items():
                cal = info.get("calibration")
                if info.get("serial") is not None and cal is not None and cal.get("accepted_once"):
                    maps[b] = (float(cal["slope"]), float(cal["offset_c"]))
        faulted_set = set(faulted)
        zones_ok = {z for z, v in verdicts.items() if v.trusted and z not in faulted_set}
        result = thermal.update(
            mem.get("thermal"),
            cfg,
            temps=trusted_temps,
            u=prev,
            ts=obs.ts,
            zones_ok=zones_ok,
            occupancy=occupancy,
            maps=maps,
            classes=classes,
            rpm=obs.rpm,
        )
    except Exception as exc:  # identification never raises out of step and never faults
        error = f"{type(exc).__name__}: {exc}"[:200]
        memory = thermal.fresh_memory(cfg, status="error", error=error)
        old = mem.get("thermal")
        if isinstance(old, Mapping) and old.get("hold") is not None:
            memory["hold"] = {"since": None}  # a stale model still re-confirms after a reset
        mem["thermal"] = memory
        return thermal.summary(memory, cfg, occupancy=occupancy)
    mem["thermal"] = result.memory
    return result.summary


def _plant_view(
    block: Mapping[str, Mapping[str, Any]], update: estimator.EstimatorUpdate | None
) -> dict[str, Any]:
    """``SolverRequest.plant``: the estimator's zones and bays for the DAS MPC's model
    (every zone, faulted ones included; ``solver_pi.SolverRequest``)."""
    if update is None:
        return {}
    zones_out: dict[str, Any] = {}
    for zone, info in update.zones.items():
        if not info.get("initialised"):
            continue
        zones_out[zone] = {
            "t_air": info["t_air_c"],
            "d_air": info["d_air_c_per_s"],
            "t_in": info["t_in_c"],
        }
    bays_out: dict[str, Any] = {}
    for bay, info in update.bays.items():
        entry: dict[str, Any] = {
            "occupancy": info["occupancy"],
            "class": info["class"],
            "since_ts": info["since_ts"],
        }
        est = block.get(bay)
        if est is not None:
            entry["t"] = est["t"]
            entry["q_w"] = est.get("q_w", 0.0)
            entry["sigma"] = est["sigma"]
            entry["sigma_cal"] = est.get("sigma_cal", est["sigma"])
        bays_out[bay] = entry
    return {"zones": zones_out, "bays": bays_out}


def _estimate_diagnostics(
    block: Mapping[str, Mapping[str, Any]],
    verdicts: Mapping[str, zones.ZoneTrust],
    faulted: list[str],
) -> dict[str, dict[str, Any]]:
    """``diagnostics["estimates"]``: the estimates block in display units (module docstring)."""
    out: dict[str, dict[str, Any]] = {}
    for bay, est in block.items():
        zone = est["zone"]
        out[bay] = {
            "zone": zone,
            "class": est["class"],
            "occupancy": est["occupancy"],
            "t_c": est["t"],
            "sigma_c": est["sigma"],
            "margin_c": est["margin"],
            "soft_c": est["soft"],
            "hard_c": est["hard"],
            "limit_c": est["limit"],
            "limit_margin_c": est["hard"] - est["t"],
            "zone_trusted": verdicts[zone].trusted and zone not in faulted,
            "calibrated": est["calibrated"],
            "source": est["source"],
        }
        if "q_w" in est:
            out[bay]["q_w"] = est["q_w"]
    return out


def _sanitized(values: Mapping[str, float | None]) -> dict[str, float | None]:
    return {k: (v if _finite(v) else None) for k, v in values.items()}


def _next_good_obs(
    obs: PlantObservation,
    cfg: MpcConfig,
    state: MpcState,
    gate: GateResult,
    trusted: bool,
    fault_since: float | None,
    faulted: list[str],
    time_status: str,
    confirming: Mapping[str, int],
) -> PlantObservation | None:
    """``last_good_obs`` for the next state.

    Legacy mode: replaced as a whole by what the gate trusted -- the filtered
    temperatures (with median3 the raw sample may hide a Jump the median has
    not shown yet), and rpm/pwm with NaN/inf replaced by None so the state
    stays finite -- on a trusted tick that ends fault-free.

    With zones: per temperature. A gate-trusted value on a time-valid tick
    replaces the old one when its zone is not in fault (a sensor without a
    zone: when some zone is not in fault) and the sensor is not confirming; every
    other temperature keeps its last good value. Nothing usable keeps the old
    observation as a whole.
    """
    layout = cfg.zone_layout
    if layout.implicit:
        if trusted and fault_since is None:
            return PlantObservation(
                temps=dict(gate.filtered),
                rpm=_sanitized(obs.rpm),
                pwm=_sanitized(obs.pwm),
                ts=obs.ts,
            )
        return state.last_good_obs
    if time_status not in ("ok", "first"):
        return state.last_good_obs
    faulted_set = set(faulted)
    some_zone_ok = len(faulted_set) < len(layout.zones)
    usable = {
        name
        for name in cfg.temps
        if gate.per_temp[name]
        and name not in confirming
        and (
            some_zone_ok
            if layout.sensor_zone[name] is None
            else layout.sensor_zone[name] not in faulted_set
        )
    }
    if not usable:
        return state.last_good_obs
    old = {} if state.last_good_obs is None else state.last_good_obs.temps
    temps: dict[str, float | None] = {}
    for name in cfg.temps:
        if name in usable:
            temps[name] = gate.filtered[name]
        else:
            v = old.get(name)
            temps[name] = v if _finite(v) else None
    return PlantObservation(
        temps=temps, rpm=_sanitized(obs.rpm), pwm=_sanitized(obs.pwm), ts=obs.ts
    )
