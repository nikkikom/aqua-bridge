"""Per-zone trust, fault closure and per-channel fallback policy (plan section 0.1).

Pure functions of the gate result, the time status, the config and the zone
fault bookkeeping; no clocks, no I/O. ``mpc.step`` calls them in its steps
3b (zone trust), 4 (per-zone fault bookkeeping) and 6 (fallback policy per
channel). In legacy mode (no ``topology``) the config is one implicit zone
(:data:`aqua_bridge.model.IMPLICIT_ZONE`) that holds every channel and one
required group per temperature, so every rule below reduces exactly to the
whole-tick behaviour of section 3.

Zone trust (:func:`evaluate`)
-----------------------------
The gate stays per sensor. Zone ``z`` is **trusted** this tick iff the time
status is ``first`` / ``ok``, ``obs.temps`` has no key outside
``cfg.temps``, and the zone's trust rule holds:

* ``strict`` -- every required group of the zone has at least one
  gate-trusted member (:attr:`ZoneLayout.required_groups`): the zone-air
  sensors of the zone; the ``drive_proximal`` sensors of every bay of the
  zone that is occupied or unknown (``occupied: true`` / ``auto``); and,
  as a conservative addition, every sensor that carries a setpoint on its
  own. The legacy ``pi`` / ``mpc`` solvers regulate on setpoint
  temperatures and have no estimator to let a group member stand in for a
  lost one, so losing a regulated sensor faults its zone instead of
  silently dropping the hottest reading from the channel's error.
  Redundant members of a group (a second air sensor, a second proximal
  sensor on a bay) therefore only matter as "one of them is enough".
  Sensors outside every group (inlet, exhaust, the proximal sensor of an
  ``occupied: false`` bay) are still gated but never fault a zone.
* **Sensor confirmation** (zones only, :func:`advance_confirmation`). A
  sensor whose value the gate rejects (``range``, ``slew``, ``stuck``: a
  Jump, a Spike, a frozen reading leaving its band) is *confirming* until it
  has been gate-trusted on ``confirm_ticks`` consecutive time-valid ticks,
  exactly the count a zone needs to leave a fault. A dropout (``missing``,
  ``null``, ``non_finite``) starts nothing: the value that returns is gated
  against the last good one -- unless there is no last good one to gate it
  against (a sensor missing since boot, ``gate.no_reference``, item 61):
  such a first reading has no evidence behind it either, so it confirms too,
  on the tick it first appears. A confirming sensor is not fused by the
  estimator, not handed to the solver and not a last good value. For zone
  trust it counts as a trusted member only while its zone is already in
  fault (the zone's own confirmation runs beside it, so a sole member costs
  ``confirm_ticks`` once, not twice); a fault-free zone whose group has no
  confirmed trusted member faults. A zone in fault is eligible for the
  solver only when every required group also has a confirmed trusted member
  (:func:`groups_confirmed`). A redundant member that jumps therefore stays
  out of the estimator until it confirms while its group stays trusted
  through the other members, and the zone does not fault. The counts live in
  ``solver_memory["sensor_confirm"]`` (sensor -> consecutive trusted ticks,
  confirming sensors only); legacy mode has no such key.
* ``sigma`` -- the estimator's uncertainty instead of the drive and air
  groups (:func:`sigma_reasons`): for every bay of the zone declared
  ``occupied: true`` / ``auto`` that the estimator does not report
  ``empty`` this tick, the bay has an estimate whose ``sigma`` is at most
  ``estimator.sigma_fault_c``, and the zone's air estimate is initialised
  with ``sigma_air_c`` at most ``estimator.sigma_air_fault_c``. A lost
  sensor is then not a fault by itself: the estimator predicts the
  unobserved node, its variance grows every tick, the margin ``k * sigma``
  widens and the solver raises the fans; the zone faults only once the
  estimator can no longer bound a drive or the air (observability loss). The
  setpoint groups stay required with the confirmation rule above (a zoned
  setpoint config regulates on those sensors directly, and nothing stands in
  for them). The zone-air and bay groups, and their confirmation
  (:func:`groups_confirmed`), are not checked: a confirming sensor is not
  fused, so the sigma of its bay or zone already carries its absence.
  *Tick ordering*: ``step`` runs the estimator before zone trust, on this
  tick's gate-trusted, confirmed temperatures, which depend on no zone
  verdict, so the rule reads this tick's posterior sigma. Without an
  estimator update (an estimator fault) :func:`effective_trust_rule` is
  ``strict`` for that tick, and the diagnostics say so.

Fault closure (:func:`closure`, :func:`fallback_channels`)
----------------------------------------------------------
``F`` is the set of zones in fault. ``F* = F`` plus every zone listed in
``coupled_to`` of a zone in ``F`` (the *declared* topology, never
identified numbers; with ``zones.fault_coupling: none`` the closure is
``F`` itself). A channel is **under fallback policy** iff it moves air
through a zone of ``F*`` (``ZoneLayout.reach``). Those channels get the
section 3 policy by zone timer: hold ``prev`` while the fault is younger
than ``fallback_hold_s``, then ``max(prev, fallback_pwm)``. A channel
reached by several faulted zones follows the oldest of their timers (the
first to ramp high). Consequences, per zone and across coupling:

* no channel that moves air through a faulted zone or a zone coupled to
  it is ever commanded below ``prev`` while the fault lasts;
* every other channel follows the solver, whose request carries only
  sensors of zones that are not in fault, and the channels under fallback
  policy as fixed inputs (``SolverRequest.fixed_channels``);
* with ``coupled_to`` empty everywhere the rule is strictly per zone.

Mode (``mpc.step``): ``fallback`` iff every zone is in fault, ``degraded``
iff some but not all are, else ``auto`` / ``saturated``.

Soft sigma floor (:func:`advance_sigma_floor`, ``zones.trust_rule: sigma``)
---------------------------------------------------------------------------
A zone with a zone-air or bay group without a gate-trusted, confirmed member
(:func:`lost_groups`) stays on the solver under ``sigma``, but the estimator's blind
estimate lags the heat the lost sensor would have shown while its sigma is still small.
Per zone an *episode* opens on the tick a group is lost: the floor is ``prev`` on every
channel of the zone's reach (the command of the tick before the loss; when a further
group is lost during an episode, the higher of ``prev`` and the floor still in force),
and the sigma of each lost group is recorded (:func:`group_sigma`: the bay's drive sigma,
the zone's air sigma). The floor **holds** until ``zones.sigma_floor_hold_s`` has passed,
counted in ticks of ``dt`` so a late tick never shortens it, or the sigma of every lost
group has grown by ``zones.sigma_floor_growth_c``, whichever comes first: by then
``k_sigma * growth`` of extra margin carries the uncertainty. It is then **released**:
lowered by ``zones.sigma_floor_release_per_min * dt / 60`` per tick until it reaches
``pwm_min`` and is gone (``released``), so a floor ends at most ``sigma_floor_hold_s +
60 * (pwm_max - pwm_min) / sigma_floor_release_per_min`` seconds after its last opening.
A group that returns while others stay lost drops out of the growth check; the episode
closes when every group is held again. Episodes advance while their zone is in fault,
but the floor applies to eligible zones only (a zone in fault holds and ramps high); on a
tick without an estimator update the episodes are kept and not advanced. On a channel
several zones reach, the floor is the highest of theirs. The floor is a level, not
``prev`` on every tick: the solver may move above it and come back, so its swings do not
ratchet the fans up.
"""

from __future__ import annotations

import math
from collections.abc import Iterable, Mapping
from dataclasses import dataclass
from typing import TYPE_CHECKING, Any

from aqua_bridge.control.gate import REASON_RANGE, REASON_SLEW, REASON_STUCK, GateResult
from aqua_bridge.model import SETPOINT_GROUP_PREFIX, MpcConfig

if TYPE_CHECKING:
    from aqua_bridge.control.estimator import EstimatorUpdate

__all__ = [
    "CONFIRM_REASONS",
    "FLOOR_HOLD",
    "FLOOR_RELEASE",
    "FLOOR_RELEASED",
    "SigmaFloor",
    "ZoneTrust",
    "advance_confirmation",
    "advance_sigma_floor",
    "channel_fallback_elapsed",
    "closure",
    "effective_trust_rule",
    "evaluate",
    "fallback_channels",
    "fallback_target",
    "group_sigma",
    "groups_confirmed",
    "lost_groups",
    "sigma_reasons",
]

#: Time statuses of ``mpc.step`` under which a sample may be trusted.
_TIME_OK = ("first", "ok")

#: Gate reasons that reject a present value and start a sensor's confirmation.
CONFIRM_REASONS = frozenset({REASON_RANGE, REASON_SLEW, REASON_STUCK})


@dataclass(frozen=True)
class ZoneTrust:
    """Verdict for one zone this tick: trusted or not, and why not (JSON-friendly strings)."""

    trusted: bool
    reasons: tuple[str, ...] = ()

    def to_dict(self) -> dict[str, Any]:
        return {"trusted": self.trusted, "reasons": list(self.reasons)}


def effective_trust_rule(cfg: MpcConfig, estimator: EstimatorUpdate | None = None) -> str:
    """The trust rule that actually runs: ``sigma`` falls back to ``strict`` without an
    estimator update of this tick (an estimator fault) and in legacy mode."""
    rule = "strict" if cfg.zones is None else cfg.zones.trust_rule
    if rule == "sigma" and (estimator is None or cfg.zone_layout.implicit):
        return "strict"
    return rule


def _rule_groups(zone: str, cfg: MpcConfig, rule: str) -> tuple[tuple[str, tuple[str, ...]], ...]:
    """The required groups ``rule`` checks: every group (``strict``) or only the setpoint
    groups (``sigma``, whose drive and air checks come from the estimator)."""
    groups = cfg.zone_layout.required_groups[zone]
    if rule == "sigma":
        return tuple(g for g in groups if g[0].startswith(SETPOINT_GROUP_PREFIX))
    return groups


def _sigma_ok(value: object, threshold: float) -> bool:
    """``value`` is a number at most ``threshold`` (NaN, None and non-numbers are not)."""
    if isinstance(value, bool) or not isinstance(value, int | float):
        return False
    return value <= threshold


def _sigma_text(value: object) -> str:
    if isinstance(value, bool) or not isinstance(value, int | float):
        return "none"
    return f"{float(value):.3f}"


def sigma_reasons(zone: str, cfg: MpcConfig, estimator: EstimatorUpdate) -> list[str]:
    """Why the ``sigma`` rule does not trust ``zone`` this tick (empty: it does).

    Reads this tick's estimator update: ``zones[zone]["sigma_air_c"]`` of an initialised
    zone, and ``estimates[bay]["sigma"]`` for every bay of the zone declared ``occupied:
    true`` / ``auto`` that ``bays[bay]["occupancy"]`` does not report ``empty``. A zone
    that is not initialised, a missing estimate or a sigma that is not a number at most
    its threshold is a reason (module docstring).
    """
    topo = cfg.topology
    spec = cfg.estimator
    if topo is None or spec is None:
        return ["sigma:no_estimator"]
    reasons: list[str] = []
    air = estimator.zones.get(zone)
    sigma_air = (
        air.get("sigma_air_c") if isinstance(air, Mapping) and air.get("initialised") else None
    )
    if not _sigma_ok(sigma_air, spec.sigma_air_fault_c):
        reasons.append(f"sigma:zone_air={_sigma_text(sigma_air)}>{spec.sigma_air_fault_c:g}")
    for bay, bay_spec in topo.bays.items():
        if bay_spec.zone != zone or not bay_spec.constrained:
            continue
        info = estimator.bays.get(bay)
        if isinstance(info, Mapping) and info.get("occupancy") == "empty":
            continue
        est = estimator.estimates.get(bay)
        sigma = est.get("sigma") if isinstance(est, Mapping) else None
        if not _sigma_ok(sigma, spec.sigma_fault_c):
            reasons.append(f"sigma:bay:{bay}={_sigma_text(sigma)}>{spec.sigma_fault_c:g}")
    return reasons


def advance_confirmation(
    counts: object, gate: GateResult, time_status: str, cfg: MpcConfig
) -> dict[str, int]:
    """Confirming sensors after this tick: sensor -> consecutive trusted ticks (module docstring).

    ``counts`` is the previous tick's ``solver_memory["sensor_confirm"]``; a malformed
    count reads as zero and one at or above ``confirm_ticks`` as ``confirm_ticks - 1``
    (neither can come from this function). A gate rejection
    of a present value (:data:`CONFIRM_REASONS`) sets the count to 0; a gate-trusted
    value on a time-valid tick adds one, and the sensor leaves the map on reaching
    ``cfg.confirm_ticks``; any other tick of a confirming sensor (a dropout, a time
    fault) restarts its count. A name gate-trusted only because it had no reference to
    check against (``gate.no_reference``, item 61 -- a sensor missing since boot, while
    the rest of the system is past its own cold start) starts confirming too, on this
    same tick, exactly like a fresh rejection: it has no more evidence behind it than a
    Jump does. Legacy mode: always empty.
    """
    if cfg.zone_layout.implicit:
        return {}
    old = counts if isinstance(counts, Mapping) else {}
    time_ok = time_status in _TIME_OK
    out: dict[str, int] = {}
    for name in cfg.temps:
        if CONFIRM_REASONS.intersection(gate.reasons.get(name, ())):
            out[name] = 0
            continue
        if name not in old:
            if time_ok and gate.per_temp.get(name, False) and name in gate.no_reference:
                out[name] = 0  # a never-referenced first reading confirms like any return
            continue
        raw = old[name]
        valid = isinstance(raw, int) and not isinstance(raw, bool) and raw >= 0
        n = min(raw, cfg.confirm_ticks - 1) if valid else 0  # a confirmed sensor is never stored
        if gate.per_temp.get(name, False) and time_ok:
            n += 1
            if n >= cfg.confirm_ticks:
                continue
        else:
            n = 0
        out[name] = n
    return out


def _member_trusted(
    name: str, gate: GateResult, confirming: Mapping[str, int], counts_confirming: bool
) -> bool:
    """A group member counts: gate-trusted, and confirmed unless confirming members count."""
    if not gate.per_temp.get(name, False):
        return False
    return counts_confirming or name not in confirming


def groups_confirmed(
    zone: str,
    gate: GateResult,
    confirming: Mapping[str, int],
    cfg: MpcConfig,
    rule: str = "strict",
) -> bool:
    """Whether every required group of ``zone`` that ``rule`` checks (every group with
    ``strict``, the setpoint groups with ``sigma``) has a gate-trusted member that is not
    confirming."""
    return all(
        any(_member_trusted(name, gate, confirming, False) for name in members)
        for _label, members in _rule_groups(zone, cfg, rule)
    )


def evaluate(
    gate: GateResult,
    time_status: str,
    cfg: MpcConfig,
    estimator: EstimatorUpdate | None = None,
    *,
    confirming: Mapping[str, int] | None = None,
    in_fault: Iterable[str] = (),
) -> dict[str, ZoneTrust]:
    """Per-zone trust for this tick, keyed and ordered like ``cfg.zone_layout.zones``.

    ``estimator`` is this tick's estimator update, which the ``sigma`` rule reads;
    without it ``sigma`` applies ``strict`` (:func:`effective_trust_rule`).
    ``confirming`` is :func:`advance_confirmation` for this tick and ``in_fault`` the
    zones in fault before it: a confirming member counts only for a zone in fault.
    Legacy mode: the implicit zone is trusted iff the gate's whole-tick verdict is and
    the time status is ``first`` / ``ok``.
    """
    layout = cfg.zone_layout
    rule = effective_trust_rule(cfg, estimator)
    pending: Mapping[str, int] = {} if confirming is None else confirming
    faulted = set(in_fault)
    common: list[str] = []
    if time_status not in _TIME_OK:
        common.append(f"time:{time_status}")
    if gate.unknown_keys:
        common.append("unknown_keys:" + ",".join(gate.unknown_keys))
    out: dict[str, ZoneTrust] = {}
    for zone in layout.zones:
        reasons = list(common)
        counts_confirming = zone in faulted
        for label, members in _rule_groups(zone, cfg, rule):
            if any(_member_trusted(name, gate, pending, counts_confirming) for name in members):
                continue
            detail = ",".join(f"{name}={_member_detail(name, gate, pending)}" for name in members)
            reasons.append(f"{label}:{detail}")
        if rule == "sigma" and estimator is not None:
            reasons.extend(sigma_reasons(zone, cfg, estimator))
        out[zone] = ZoneTrust(trusted=not reasons, reasons=tuple(reasons))
    return out


def _member_detail(name: str, gate: GateResult, confirming: Mapping[str, int]) -> str:
    """Why a member does not count, for the zone's reasons."""
    if gate.per_temp.get(name, False) and name in confirming:
        return "confirming"
    return "/".join(gate.reasons.get(name, ())) or "untrusted"


def closure(faulted: Iterable[str], cfg: MpcConfig) -> tuple[str, ...]:
    """``F*``: the faulted zones plus their declared ``coupled_to`` (in zone order).

    With ``zones.fault_coupling: none`` the closure is the faulted set itself.
    """
    layout = cfg.zone_layout
    faulted_set = set(faulted)
    declared = cfg.zones is None or cfg.zones.fault_coupling == "declared"
    members = set(faulted_set)
    if declared:
        for zone in faulted_set:
            members.update(layout.coupled.get(zone, ()))
    return tuple(z for z in layout.zones if z in members)


def fallback_channels(faulted: Iterable[str], cfg: MpcConfig) -> tuple[str, ...]:
    """Channels under fallback policy while ``faulted`` zones are in fault (channel order)."""
    layout = cfg.zone_layout
    touched: set[str] = set()
    for zone in faulted:
        touched.update(layout.reach.get(zone, ()))
    return tuple(ch for ch in cfg.channels if ch in touched)


def channel_fallback_elapsed(
    elapsed_by_zone: Mapping[str, float], cfg: MpcConfig
) -> dict[str, float]:
    """Per channel under fallback policy, the largest fault age among the zones reaching it.

    ``elapsed_by_zone`` holds the faulted zones only; channels no faulted zone
    reaches are absent from the result.
    """
    layout = cfg.zone_layout
    out: dict[str, float] = {}
    for zone, elapsed in elapsed_by_zone.items():
        for ch in layout.reach.get(zone, ()):
            if ch not in out or elapsed > out[ch]:
                out[ch] = elapsed
    return {ch: out[ch] for ch in cfg.channels if ch in out}


def fallback_target(
    prev: float, fallback_pwm: float, elapsed_s: float, cfg: MpcConfig
) -> tuple[float, str]:
    """Section 3 policy for one channel: hold ``prev``, then ``max(prev, fallback_pwm)``.

    Returns ``(target, policy)`` with policy ``hold`` or ``ramp_high``. The
    target is never below ``prev``: a fault never reduces cooling.
    """
    if elapsed_s > cfg.fallback_hold_s:
        return max(prev, fallback_pwm), "ramp_high"
    return prev, "hold"


#: Phases of a soft sigma floor episode (module docstring, *Soft sigma floor*).
FLOOR_HOLD = "hold"
FLOOR_RELEASE = "release"
FLOOR_RELEASED = "released"
_FLOOR_PHASES = (FLOOR_HOLD, FLOOR_RELEASE, FLOOR_RELEASED)


@dataclass(frozen=True)
class SigmaFloor:
    """What :func:`advance_sigma_floor` returns.

    * ``floor``  -- per channel, the soft floor under the solver's demand this tick
      (channels without one are absent)
    * ``memory`` -- the next ``solver_memory["sigma_floor"]`` (JSON-serialisable)
    * ``zones``  -- per zone with an open episode, its state for the diagnostics
    """

    floor: dict[str, float]
    memory: dict[str, dict[str, Any]]
    zones: dict[str, dict[str, Any]]


def lost_groups(
    zone: str, gate: GateResult, confirming: Mapping[str, int], cfg: MpcConfig
) -> tuple[str, ...]:
    """Labels of the zone-air and bay groups of ``zone`` (the ``strict`` groups that the
    ``sigma`` rule replaces by the estimator) without a gate-trusted, confirmed member."""
    return tuple(
        label
        for label, members in cfg.zone_layout.required_groups[zone]
        if not label.startswith(SETPOINT_GROUP_PREFIX)
        and not any(_member_trusted(name, gate, confirming, False) for name in members)
    )


def _finite(value: object) -> float | None:
    if isinstance(value, bool) or not isinstance(value, int | float) or not math.isfinite(value):
        return None
    return float(value)


def group_sigma(label: str, zone: str, estimator: EstimatorUpdate) -> float | None:
    """This tick's sigma of the node a group observes: the bay's drive sigma for
    ``bay:<b>``, the zone's air sigma for ``zone_air``; ``None`` when there is none."""
    if label.startswith("bay:"):
        est = estimator.estimates.get(label[len("bay:") :])
        return _finite(est.get("sigma")) if isinstance(est, Mapping) else None
    air = estimator.zones.get(zone)
    if isinstance(air, Mapping) and air.get("initialised"):
        return _finite(air.get("sigma_air_c"))
    return None


def _stored_episode(raw: object, zone: str, cfg: MpcConfig) -> dict[str, Any] | None:
    """A stored episode, or ``None`` when it is malformed (a malformed one starts over,
    which only holds the floor longer)."""
    if not isinstance(raw, Mapping):
        return None
    lost, ticks, phase = raw.get("lost"), raw.get("ticks"), raw.get("phase")
    sigma0, level = raw.get("sigma0"), raw.get("level")
    if (
        not isinstance(lost, list)
        or not all(isinstance(x, str) for x in lost)
        or isinstance(ticks, bool)
        or not isinstance(ticks, int)
        or ticks < 0
        or phase not in _FLOOR_PHASES
        or not isinstance(sigma0, Mapping)
        or not isinstance(level, Mapping)
    ):
        return None
    reach = cfg.zone_layout.reach[zone]
    levels = {ch: _finite(level.get(ch)) for ch in reach if ch in level}
    if any(v is None for v in levels.values()) or (
        phase != FLOOR_RELEASED and len(levels) != len(reach)
    ):
        return None
    return {
        "lost": list(lost),
        "ticks": ticks,
        "phase": phase,
        "sigma0": {label: _finite(sigma0.get(label)) for label in lost},
        "level": levels,
    }


def advance_sigma_floor(
    memory: object,
    cfg: MpcConfig,
    *,
    lost: Mapping[str, tuple[str, ...]],
    estimator: EstimatorUpdate | None,
    prev: Mapping[str, float],
    eligible: Iterable[str],
) -> SigmaFloor:
    """The soft sigma floor after this tick (module docstring, *Soft sigma floor*).

    ``memory`` is the previous ``solver_memory["sigma_floor"]``, ``lost`` the
    :func:`lost_groups` of every zone this tick, ``estimator`` this tick's update
    (``None`` on an estimator fault: the open episodes are kept and not advanced),
    ``prev`` the command the tick is rate-limited against and ``eligible`` the zones
    the solver drives this tick (the only ones whose floor applies).
    """
    policy = cfg.zones
    assert policy is not None
    old = memory if isinstance(memory, Mapping) else {}
    step_down = policy.sigma_floor_release_per_min * cfg.dt / 60.0
    out: dict[str, dict[str, Any]] = {}
    for zone in cfg.zone_layout.zones:
        now_lost = list(lost.get(zone, ()))
        if not now_lost:
            continue  # every group is held again: the episode closes
        episode = _stored_episode(old.get(zone), zone, cfg)
        if estimator is None:
            if episode is not None:
                out[zone] = episode
            continue
        sigma_now = {label: group_sigma(label, zone, estimator) for label in now_lost}
        if episode is None or not set(now_lost) <= set(episode["lost"]):
            carried = {} if episode is None else episode["level"]
            episode = {
                "lost": now_lost,
                "ticks": 0,
                "phase": FLOOR_HOLD,
                "sigma0": sigma_now,
                "level": {
                    ch: max(float(prev[ch]), carried.get(ch, -math.inf))
                    for ch in cfg.zone_layout.reach[zone]
                },
            }
        else:
            episode["lost"] = now_lost
            episode["ticks"] += 1
            episode["sigma0"] = {label: episode["sigma0"].get(label) for label in now_lost}
            if episode["phase"] == FLOOR_HOLD:
                grown = all(
                    sigma_now[label] is not None
                    and episode["sigma0"][label] is not None
                    and sigma_now[label] - episode["sigma0"][label] >= policy.sigma_floor_growth_c
                    for label in now_lost
                )
                if episode["ticks"] * cfg.dt >= policy.sigma_floor_hold_s or grown:
                    episode["phase"] = FLOOR_RELEASE
            if episode["phase"] == FLOOR_RELEASE:
                episode["level"] = {ch: v - step_down for ch, v in episode["level"].items()}
                if all(v <= cfg.pwm_min for v in episode["level"].values()):
                    episode["phase"] = FLOOR_RELEASED
                    episode["level"] = {}
        out[zone] = episode
    floor: dict[str, float] = {}
    for zone in eligible:
        for ch, value in out.get(zone, {}).get("level", {}).items():
            floor[ch] = max(floor.get(ch, -math.inf), value)
    diag: dict[str, dict[str, Any]] = {}
    for zone, ep in out.items():
        growth: dict[str, float | None] = {}
        for label in ep["lost"]:
            now = None if estimator is None else group_sigma(label, zone, estimator)
            start = ep["sigma0"].get(label)
            growth[label] = None if now is None or start is None else now - start
        diag[zone] = {
            "lost": list(ep["lost"]),
            "phase": ep["phase"],
            "held_s": ep["ticks"] * cfg.dt,
            "sigma_growth_c": growth,
            "floor": dict(ep["level"]),
        }
    return SigmaFloor(
        floor={ch: floor[ch] for ch in cfg.channels if ch in floor}, memory=out, zones=diag
    )
