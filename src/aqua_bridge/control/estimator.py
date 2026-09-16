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

Priors (plan section 3 table): ``C_a = 200``, ``leak = 1``, ``kappa = 3`` for
declared ``coupled_to`` pairs (the neighbour's previous air estimate is a
known input), ``g0 = 0.3``, ``k = 0.5``, ``C_d = tau_d_s * (g0 + k)`` (the
drive class's time constant at full airflow). Airflow: every channel's
``phi = clip((u - deadband) / (1 - deadband), 0, 1) ** exponent`` of its fan
model at ``u = prev`` (the command on the fans over the last interval), and
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
``nu`` with ``|nu| > JUMP_MIN_C`` and ``|nu| > JUMP_SIGMAS * sqrt(S)`` on an
occupied or unknown bay adds ``(nu / s) ** 2`` to the drive variance and
``nu ** 2`` to the sensor node's variance before the update, and drops the
bay's correlation association. ``S`` is that measurement's own innovation
variance, so for a member with an offset it carries ``c_i``'s variance too: an
uncertain placement offset cannot look like a swap, while a swap -- which moves
the drive and therefore every member together -- still does. The offset keeps its
variance through the inflation: the drive moved, the placement did not.

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
starts. ``P0 = diag(0.25, 0.05^2, 0.1, 0.1, 0.005^2)`` per state kind (the drive
variance only covers the transient: the map offset is ``sigma_cal``), and
``proximal_offset_c ** 2`` per placement offset, which starts at 0.

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

Output
------
Per constrained bay (occupied or unknown): ``t = T_d``,
``sigma = sqrt(P_dd + sigma_cal^2)``, ``margin = k_sigma * sigma`` and the soft /
hard targets of its class; per zone ``T_a``, ``sigma_air = sqrt(P_aa)``, ``d_a``
and ``drift = max_j |q_j - g_j (T_d,j - T_a) / C_d|`` (degC/min, a validity
metric for the DAS MPC milestone). ``sigma_cal`` is 1.5 degC uncalibrated and
``max(0.5, EW-RMS residual)`` calibrated; the filter cannot shrink it.

Per bay the block also carries ``observed`` (a trusted proximal member this tick),
``seeded`` (the bay has had a reading of its own; see *Per bay* above),
``settling`` (within ``bay_settle_s`` of a fast-swap jump or of an occupancy change
into or out of ``empty``, both of which widen the bay's variance on purpose; a window
opens only on a bay whose sigma had come back within ``sigma_fault_c``, so readings that
keep tripping the fast-swap rule cannot renew it for ever) and
``offsets_c`` (the placement offset the filter carries per further member); per
zone ``air_blind_s``, the time since a trusted ``zone_air`` reading was last fused.
``zones.trust_rule: sigma`` reads all four (``aqua_bridge.control.zones``).

Occupancy (``topology.bays.<b>.occupied``; runtime ``POST /api/bay``)
-------------------------------------------------------------------
``true`` is always ``occupied``, ``false`` always ``empty``; ``auto`` runs the
machine below on ``dT = T_s - T_a`` and ``heat = q C_d + g b / s`` (the heat the
filter attributes to the drive beyond what the sensor offset alone explains:
an empty bay has no case-to-drive offset, so with the prior ``b`` its plain
``q C_d`` would never fall below 1 W):

* ``unknown`` at start (and whenever the zone has no estimate yet), and after a
  tick without any trusted proximal member of the bay (loss of observability;
  a redundant member standing in keeps the state);
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
  variance 25 degC^2 and ``q`` reset to 0: an inserted drive raises the fans
  through its margin within a few ticks. Entering ``unknown`` from ``empty``
  resets the same way.

``unknown`` and ``occupied`` bays both carry constraints. The estimator never faults
a zone over a removed drive; the zone trust groups stay those of the declared config.

The per-bay output shows a change in progress: ``pending_empty_s`` (seconds of
evidence toward ``empty`` counted so far) and ``pending_occupied_ticks`` (ticks of
evidence toward ``occupied`` on an empty bay); both are 0 when nothing is pending.
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
the fresh-sample count restarts at 0, so ``sigma_cal`` returns to 1.5 degC while
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

Memory (plain JSON)::

    {"v": 1, "fp": <structure fingerprint>, "ts": last ts,
     "zones": {zone: {"x": [...], "P": [[...]], "t_in": float | None, "blind": s}},
     "bays": {bay: {"occ", "low", "rise", "since", "assoc", "map", "init",
                    "disturb", "over"}},
     "cal": {bay: {serial: {"th", "P", "n", "fresh", "rms2", "ts", "used"
                            [, "inflate", "confirm"]}}},
     "smart": {serial: {"ts", "t", "model", "hist"}},
     "series": associate series | None, "scores": {serial: {bay: score}},
     "pending": {bay: [serial, evaluations]}, "next_assoc": ts,
     "count": {"smart_used", "smart_rejected"}}

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

from aqua_bridge.control import associate
from aqua_bridge.control.estimates import (
    PRIOR_BETA,
    PRIOR_OFFSET_C,
    SIGMA_UNCALIBRATED_C,
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
    "JUMP_MIN_C",
    "JUMP_SIGMAS",
    "K_W_PER_K",
    "KAPPA_W_PER_K",
    "LEAK_W_PER_K",
    "MAX_PREDICT_S",
    "NEW_SAMPLE_EPS_S",
    "OCCUPIED",
    "CAL_OFFSET_BOUNDS",
    "RESET_DRIVE_VAR",
    "SIGMA_CAL_FLOOR_C",
    "UNKNOWN",
    "EstimatorUpdate",
    "calibration_update",
    "drive_capacity",
    "restore_calibration",
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

#: Initial covariance per state kind: T_a, d_a, T_d, T_s, q.
P0_T_AIR = 0.25
P0_D_AIR = 0.05**2
P0_T_DRIVE = 0.1
P0_T_SENSOR = 0.1
P0_HEAT = 0.005**2

#: Occupancy: heat below which a bay may be empty, rising-edge ticks, reset variance.
EMPTY_HEAT_W = 1.0
RISE_TICKS = 3
RESET_DRIVE_VAR = 25.0
#: Weight of an empty bay's proximal sensor as an air reading.
EMPTY_WEIGHT = 0.5

#: Fast-swap rule (module docstring).
JUMP_MIN_C = 0.5
JUMP_SIGMAS = 6.0

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

#: Longest interval one prediction covers, seconds.
MAX_PREDICT_S = 3600.0

_TAYLOR_ORDER = 10
_DAY_S = 86400.0


@dataclass(frozen=True)
class EstimatorUpdate:
    """What :func:`update` returns.

    * ``estimates`` -- the estimates block (constrained bays of initialised zones)
    * ``memory``    -- the next ``solver_memory["estimator"]``
    * ``zones``     -- per zone: air estimate, its sigma, disturbance, drift, inlet
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
    #: ``redundant``): its lag and the bay's sensor map are the node's, and it is the
    #: one member without a placement offset.
    primary: str
    tau_s: float
    #: Every proximal member beyond the first -> its index in the zone's offset block
    #: (the first member anchors the node, so it has no offset; empty when
    #: ``estimator.proximal_offset_c`` is 0).
    offsets: dict[str, int]


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
    with_offsets = spec_est is not None and spec_est.proximal_offset_c > 0
    sensors = cfg.sensors
    zones: dict[str, _Zone] = {}
    bays: dict[str, _Bay] = {}
    inlets = tuple(t for t in cfg.temps if sensors[t].role == "inlet")
    for z, spec in topo.zones.items():
        zone_bays = tuple(b for b, bay in topo.bays.items() if bay.zone == z)
        n_offsets = 0
        for i, b in enumerate(zone_bays):
            members = tuple(
                t for t in cfg.temps if sensors[t].role == "drive_proximal" and sensors[t].bay == b
            )
            primary = next((t for t in members if not sensors[t].redundant), members[0])
            tau = sensors[primary].tau_s
            offsets: dict[str, int] = {}
            if with_offsets:
                for name in members:
                    if name != primary:
                        offsets[name] = n_offsets
                        n_offsets += 1
            bays[b] = _Bay(b, z, i, members, primary, float(tau) if tau else TAU_SENSOR_S, offsets)
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
        )
    fingerprint = json.dumps(
        [[z, list(zs.bays), list(zs.air), zs.n_offsets] for z, zs in zones.items()]
        + [[b, list(bs.sensors)] for b, bs in bays.items()],
        separators=(",", ":"),
    )
    return _Structure(zones=zones, bays=bays, fingerprint=fingerprint)


def drive_capacity(cfg: MpcConfig, drive_class: str) -> float:
    """``C_d`` prior of a class, J/K: ``tau_d_s * (g0 + k)`` (module docstring)."""
    return cfg.drive_classes[drive_class].tau_d_s * (G0_W_PER_K + K_W_PER_K)


def _airflow(cfg: MpcConfig, zone: _Zone, u: Mapping[str, float]) -> tuple[float, float]:
    """``(Q_z, Qn_z)`` at command ``u``."""
    q = 0.0
    total = 0.0
    for ch, e in zone.e_w_per_k.items():
        model = cfg.fan_models[cfg.fans[ch].model]
        value = u.get(ch)
        pwm = float(value) if _finite(value) else 0.0
        frac = min(1.0, max(0.0, (pwm - model.deadband) / (1.0 - model.deadband)))
        q += e * frac**model.exponent
        total += e
    return q, (q / total if total > 0 else 0.0)


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
        "series": None,
        "scores": {},
        "pending": {},
        "next_assoc": None,
        "count": {"smart_used": 0, "smart_rejected": 0},
    }


def _fresh_bay() -> dict[str, Any]:
    return {
        "occ": UNKNOWN,
        "low": 0.0,
        "rise": 0,
        "since": None,
        "assoc": None,
        "map": None,
        "init": False,
        "disturb": None,
        "over": False,
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
        n = 2 + 3 * len(zone.bays) + zone.n_offsets
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
        out["bays"][b] = {
            "occ": occ,
            "low": _num(raw.get("low", 0.0)),
            "rise": rise,
            "since": _opt_num(raw.get("since")),
            "assoc": assoc,
            "map": mapping,
            # A memory written before the per-bay initialisation existed has every bay
            # of an initialised zone seeded, which is what ``True`` says.
            "init": bool(raw.get("init", True)),
            "disturb": _opt_num(raw.get("disturb")),
            "over": bool(raw.get("over", False)),
        }
    for b, per_serial in dict(memory.get("cal") or {}).items():
        if b not in st.bays:
            raise KeyError(b)
        entries = {}
        for serial, raw in dict(per_serial).items():
            if not isinstance(serial, str):
                raise TypeError("serial")
            th = [_num(v) for v in raw["th"]]
            p = [[_num(v) for v in row] for row in raw["P"]]
            n_all, fresh = raw["n"], raw["fresh"]
            if len(th) != 2 or len(p) != 2 or any(len(row) != 2 for row in p):
                raise ValueError("calibration shape")
            for count in (n_all, fresh):
                if isinstance(count, bool) or not isinstance(count, int) or count < 0:
                    raise ValueError("count")
            entries[serial] = {
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
                entries[serial]["inflate"] = inflate
                entries[serial]["confirm"] = confirm
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
    for key in ("smart_used", "smart_rejected"):
        value = count.get(key, 0)
        if isinstance(value, bool) or not isinstance(value, int) or value < 0:
            raise ValueError("counter")
        out["count"][key] = value
    return out


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


def _fresh_calibration() -> dict[str, Any]:
    return {
        "th": list(CAL_PRIOR),
        "P": [[CAL_PRIOR_VAR[0], 0.0], [0.0, CAL_PRIOR_VAR[1]]],
        "n": 0,
        "fresh": 0,
        "rms2": SIGMA_UNCALIBRATED_C**2,
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


def _mean(values: list[float]) -> float | None:
    return sum(values) / len(values) if values else None


def _settling(bm: Mapping[str, Any], ts: float, window: float) -> bool:
    """The bay is inside the settling window its last deliberate widening opened."""
    mark = bm["disturb"]
    return mark is not None and 0.0 <= float(ts) - float(mark) < window


def _mark_disturbed(bm: dict[str, Any], ts: float) -> None:
    """Open a settling window on a deliberate widening (a fast-swap jump, an occupancy
    change into or out of ``empty``).

    Only a bay whose sigma was back within ``sigma_fault_c`` at the end of the previous
    tick opens one, so an episode gets **one** window: readings that keep tripping the
    fast-swap rule cannot renew it, and the bay is exempt from the sigma trust rule for at
    most ``bay_settle_s`` however long they go on.
    """
    if not bm["over"]:
        bm["disturb"] = float(ts)


def _seed_sensor(bay: _Bay, temps: Mapping[str, float]) -> float:
    """The value a bay's sensor node starts at: its anchor member's reading if that one is
    trusted, else the hottest trusted member (conservative). The offsets of the others are
    seeded at 0, so a pair at different placements starts consistent instead of handing the
    anchor a step the fast-swap rule would read as a swap."""
    if bay.primary in temps:
        return float(temps[bay.primary])
    return max(float(temps[name]) for name in bay.sensors if name in temps)


def update(
    memory: object,
    cfg: MpcConfig,
    *,
    temps: Mapping[str, float],
    u: Mapping[str, float],
    ts: float,
    smart: object = None,
) -> EstimatorUpdate:
    """One estimator tick (module docstring).

    ``temps`` holds the gate-trusted values of this tick only (a missing name is
    untrusted), ``u`` the command on the fans since the previous tick (``prev``),
    ``ts`` the observation time and ``smart`` the raw ``PlantObservation.inputs
    ["smart"]`` (``None`` or malformed: no SMART). Raises only on a numerical
    failure of the filter (non-finite state), which ``step`` turns into an
    estimator fault.
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

    # -- association before the filter: declared wins, silent serials drop --------
    declared = {b: bay.serial for b, bay in topo.bays.items() if bay.serial}
    declared_serials = set(declared.values())
    assoc: dict[str, tuple[str, str]] = {}
    for b in topo.bays:
        bm = mem["bays"][b]
        if b in declared:
            assoc[b] = (declared[b], "declared")
            bm["assoc"] = None
            continue
        serial = bm["assoc"]
        if serial is not None and (serial not in fresh_smart or serial in declared_serials):
            bm["assoc"] = None
            serial = None
        if serial is not None:
            assoc[b] = (serial, "correlation")

    # -- calibration expiry -----------------------------------------------------
    for entries in mem["cal"].values():
        for entry in entries.values():
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

    def cal_entry(b: str) -> dict[str, Any] | None:
        if b not in assoc:
            return None
        return mem["cal"].get(b, {}).get(assoc[b][0])

    def sensor_map(b: str) -> tuple[float, float, str | None]:
        """``(s, b, source)`` the filter uses for bay ``b``."""
        entry = cal_entry(b)
        if entry is not None and entry["used"]:
            return entry["th"][0], entry["th"][1], assoc[b][0]
        return CAL_PRIOR[0], CAL_PRIOR[1], None

    prev_air = {z: float(zm["x"][0]) for z, zm in mem["zones"].items()}
    trusted_air = {z: [temps[t] for t in zone.air if t in temps] for z, zone in st.zones.items()}
    smart_arrived: dict[str, bool] = {}
    jumped: dict[str, bool] = {}
    arrays: dict[str, tuple[np.ndarray, np.ndarray]] = {}
    geometry: dict[str, tuple[float, float, float]] = {}  # zone -> (Q, Qn, t_in)

    q_scale = h_pred / cfg.dt if cfg.dt > 0 else 0.0
    offset_var = spec.proximal_offset_c**2
    for z, zone in st.zones.items():
        n = len(zone.bays)
        i_d, i_s, i_q, i_off = 2, 2 + n, 2 + 2 * n, 2 + 3 * n
        dim = 2 + 3 * n + zone.n_offsets
        inlet_vals = [temps[t] for t in zone.inlet if t in temps]
        zm = mem["zones"].get(z)
        q_flow, qn = _airflow(cfg, zone, u)

        if zm is None:
            air0 = _mean(trusted_air[z])
            if air0 is None:
                continue
            t_in = _mean(inlet_vals)
            t_in = air0 if t_in is None else t_in
            x = np.zeros(dim)
            x[0] = air0
            p_diag = np.zeros(dim)
            p_diag[0], p_diag[1] = P0_T_AIR, P0_D_AIR
            p_diag[i_off:] = offset_var
            f_air = 0.0
            for j, b in enumerate(zone.bays):
                members = [temps[t] for t in st.bays[b].sensors if t in temps]
                t_s = _seed_sensor(st.bays[b], temps) if members else air0
                x[i_s + j] = t_s
                mem["bays"][b]["init"] = bool(members)
                p_diag[i_d + j], p_diag[i_s + j], p_diag[i_q + j] = P0_T_DRIVE, P0_T_SENSOR, P0_HEAT
                occ = _declared_state(topo.bays[b].occupied, mem["bays"][b]["occ"])
                if occ == EMPTY:
                    x[i_d + j] = air0
                    continue
                s_map, b_map, source = sensor_map(b)
                mem["bays"][b]["map"] = source
                t_d = air0 + (t_s - air0 - b_map) / s_map
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
            geometry[z] = (q_flow, qn, t_in)
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
                    x[i_d + j] = x[0] + (x[i_s + j] - x[0] - b_map) / s_map
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
            g = G0_W_PER_K + K_W_PER_K * qn
            for j, b in enumerate(zone.bays):
                tau = st.bays[b].tau_s
                qd[i_s + j] = spec.q_t_sensor
                if mem["bays"][b]["occ"] == EMPTY:
                    a[i_s + j, 0] += 1.0 / tau
                    a[i_s + j, i_s + j] -= 1.0 / tau
                    continue
                s_map, b_map, _ = sensor_map(b)
                cd = drive_capacity(cfg, classes[b][0])
                a[0, 0] -= g / C_AIR_J_PER_K
                a[0, i_d + j] += g / C_AIR_J_PER_K
                a[i_d + j, i_d + j] -= g / cd
                a[i_d + j, 0] += g / cd
                a[i_d + j, i_q + j] = 1.0
                a[i_s + j, i_d + j] += s_map / tau
                a[i_s + j, 0] += (1.0 - s_map) / tau
                a[i_s + j, i_s + j] -= 1.0 / tau
                c[i_s + j] += b_map / tau
                qd[i_d + j], qd[i_q + j] = spec.q_t_drive, spec.q_heat
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
            s_map = sensor_map(b)[0]
            present = [name for name in bay.sensors if name in temps]
            if present and not bm["init"]:
                # No trusted member when the zone started: seed the bay from this first
                # reading instead of leaving the fast-swap rule to see the gap (item 69).
                t_s = _seed_sensor(bay, temps)
                _reset_state(x, p, i_s + j, t_s, P0_T_SENSOR)
                for k_off in bay.offsets.values():
                    _reset_state(x, p, i_off + k_off, 0.0, offset_var)
                air_now = float(x[0])
                if occ == EMPTY:
                    _reset_state(x, p, i_d + j, air_now, P0_T_DRIVE)
                else:
                    s_seed, b_seed, _ = sensor_map(b)
                    t_d = air_now + (t_s - air_now - b_seed) / s_seed
                    g_seed = G0_W_PER_K + K_W_PER_K * qn
                    cd_seed = drive_capacity(cfg, classes[b][0])
                    _reset_state(x, p, i_d + j, t_d, P0_T_DRIVE)
                    _reset_state(x, p, i_q + j, g_seed * (t_d - air_now) / cd_seed, P0_HEAT)
                bm["init"] = True
            for name in present:
                value = float(temps[name])
                r = _sensor_var(cfg, name)
                if occ == EMPTY:
                    r /= EMPTY_WEIGHT
                k_off = bay.offsets.get(name)
                idx = i_s + j
                if k_off is None:
                    nu = value - x[idx]
                    s_innov = p[idx, idx] + r
                else:
                    off = i_off + k_off
                    nu = value - x[idx] - x[off]
                    s_innov = p[idx, idx] + 2.0 * p[idx, off] + p[off, off] + r
                if abs(nu) > JUMP_MIN_C and nu * nu > JUMP_SIGMAS**2 * s_innov:
                    # something moved: follow the sensor (the placement offset did not
                    # move, so it keeps its own variance)
                    if occ != EMPTY:
                        p[i_d + j, i_d + j] += (nu / s_map) ** 2
                        jumped[b] = True
                    p[idx, idx] += nu * nu
                if k_off is None:
                    _scalar_update(x, p, idx, value, r)
                else:
                    _pair_update(x, p, idx, i_off + k_off, value, r)
        arrays[z] = (x, p)
        geometry[z] = (q_flow, qn, t_in)
        mem["zones"][z]["t_in"] = t_in
        mem["zones"][z]["blind"] = 0.0 if trusted_air[z] else zm.get("blind", 0.0) + h_occ

    # -- SMART: reject, calibrate, measure ------------------------------------------
    for b, (serial, _source) in assoc.items():
        if serial not in new_samples or jumped.get(b):
            continue  # a jump this tick: the sample may describe the drive that just left
        smart_arrived[b] = True
        bay = st.bays[b]
        if bay.zone not in arrays or mem["bays"][b]["occ"] == EMPTY:
            continue
        x, p = arrays[bay.zone]
        n = len(st.zones[bay.zone].bays)
        i_d, i_s = 2 + bay.index, 2 + n + bay.index
        sample_ts, temp = new_samples[serial]
        if abs(temp - x[i_d]) > spec.smart_reject_c:
            mem["count"]["smart_rejected"] += 1
            continue
        mem["count"]["smart_used"] += 1
        per_bay = mem["cal"].setdefault(b, {})
        entry = per_bay.get(serial) or _fresh_calibration()
        entry, _ = calibration_update(entry, temp - x[0], x[i_s] - x[0], sample_ts)
        if _calibrated(entry, float(ts), max_age_cal):
            entry["used"] = True
            _scalar_update(x, p, i_d, temp, SMART_R)
        per_bay[serial] = entry
        if len(per_bay) > CAL_SERIALS_PER_BAY:
            oldest = min(
                (s for s in per_bay if s != serial), key=lambda s: (per_bay[s]["ts"] or -1e300, s)
            )
            del per_bay[oldest]

    # -- occupancy -----------------------------------------------------------------------
    evidence: dict[str, tuple[float, float]] = {}
    observed: dict[str, bool] = {}
    for b, bay in st.bays.items():
        bm = mem["bays"][b]
        if jumped.get(b):  # the fast-swap rule widened this bay on purpose
            _mark_disturbed(bm, float(ts))
        before = bm["occ"]
        declared_occ = topo.bays[b].occupied
        zone_ready = bay.zone in arrays
        sensor_ok = any(t in temps for t in bay.sensors)
        observed[b] = sensor_ok
        air_ok = bool(trusted_air.get(bay.zone))
        after = before
        if declared_occ is True:
            after = OCCUPIED
        elif declared_occ is False:
            after = EMPTY
        elif not zone_ready or (not sensor_ok and not smart_arrived.get(b)):
            after = UNKNOWN
            bm["low"], bm["rise"] = 0.0, 0
        else:
            x, _ = arrays[bay.zone]
            n = len(st.zones[bay.zone].bays)
            i_d, i_s, i_q = 2 + bay.index, 2 + n + bay.index, 2 + 2 * n + bay.index
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
                n = len(st.zones[bay.zone].bays)
                i_d, i_s, i_q = 2 + bay.index, 2 + n + bay.index, 2 + 2 * n + bay.index
                if before == EMPTY:  # a drive (possibly) arrived: start from the sensor, wide
                    x[i_d] = x[i_s]
                    p[i_d, :], p[:, i_d] = 0.0, 0.0
                    p[i_d, i_d] = RESET_DRIVE_VAR
                if before == EMPTY or after == EMPTY:
                    x[i_q] = 0.0
                    p[i_q, :], p[:, i_q] = 0.0, 0.0
                    p[i_q, i_q] = P0_HEAT
                if before == EMPTY or after == EMPTY:  # a deliberate widening, as a jump
                    _mark_disturbed(bm, float(ts))
            if (before == EMPTY) != (after == EMPTY):
                bm["assoc"] = None
                if b in assoc and assoc[b][1] == "correlation":
                    del assoc[b]
            bm["low"], bm["rise"] = 0.0, 0
            bm["occ"] = after
        if jumped.get(b) and b in assoc and assoc[b][1] == "correlation":
            bm["assoc"] = None
            del assoc[b]

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
    window = spec.associate_window_s
    for serial, entry in list(mem["smart"].items()):
        if serial in unassigned and serial in new_samples:
            sample_ts, temp = new_samples[serial]
            entry["hist"] = associate.record_smart(entry["hist"], sample_ts, temp, ts, window)
        elif serial not in unassigned:
            entry["hist"] = []
        if serial not in fresh_smart and float(ts) - entry["ts"] > window:
            del mem["smart"][serial]
    candidates = [
        b
        for b, bay in st.bays.items()
        if b not in assoc and mem["bays"][b]["occ"] != EMPTY and b not in declared
    ]
    if unassigned and candidates:
        bay_y: dict[str, float | None] = {}
        for b, bay in st.bays.items():
            if bay.zone in arrays:
                x, _ = arrays[bay.zone]
                n = len(st.zones[bay.zone].bays)
                bay_y[b] = float(x[2 + n + bay.index] - x[0])
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
            scores = associate.score_matrix(
                mem["series"],
                {s: mem["smart"][s]["hist"] for s in unassigned},
                unassigned,
                candidates,
                {b: st.bays[b].zone for b in candidates},
                min_span_s=associate.MIN_SPAN_FRACTION * window,
            )
            mem["scores"] = scores
            mem["pending"], confirmed = associate.confirm(
                mem["pending"],
                associate.assign(scores, spec.associate_min_corr, spec.associate_margin),
            )
            for b, serial in confirmed.items():
                mem["bays"][b]["assoc"] = serial
                mem["smart"][serial]["hist"] = []
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
            f = float(x[2 + 2 * n + j]) - g * float(x[2 + j] - x[0]) / cd
            drift = max(drift, abs(f) * 60.0)
        zones_out[z] = {
            "initialised": True,
            "t_air_c": float(x[0]),
            "sigma_air_c": math.sqrt(max(float(p[0, 0]), 0.0)),
            "air_blind_s": float(mem["zones"][z]["blind"]),
            "d_air_c_per_s": float(x[1]),
            "t_in_c": geometry[z][2],
            "airflow_w_per_k": geometry[z][0],
            "drift_c_per_min": drift,
        }
    for b, bay in st.bays.items():
        bm = mem["bays"][b]
        cls, cls_source = classes[b]
        serial, assoc_source = assoc.get(b, (None, None))
        entry = cal_entry(b)
        calibrated = _calibrated(entry, float(ts), max_age_cal)
        sigma_cal = (
            max(SIGMA_CAL_FLOOR_C, math.sqrt(float(entry["rms2"])))  # type: ignore[index]
            * float(entry.get("inflate", 1.0))  # type: ignore[union-attr]
            if calibrated
            else SIGMA_UNCALIBRATED_C
        )
        info: dict[str, Any] = {
            "zone": bay.zone,
            "occupancy": bm["occ"],
            "declared": topo.bays[b].occupied,
            "since_ts": bm["since"],
            "pending_empty_s": float(bm["low"]),
            "pending_occupied_ticks": int(bm["rise"]),
            "observed": observed.get(b, False),
            "seeded": bool(bm["init"]),
            "settling": _settling(bm, float(ts), spec.bay_settle_s),
            "offsets_c": {
                name: float(arrays[bay.zone][0][2 + 3 * len(st.zones[bay.zone].bays) + k])
                for name, k in bay.offsets.items()
            }
            if bay.zone in arrays
            else {},
            "class": cls,
            "class_source": cls_source,
            "serial": serial,
            "association": assoc_source,
            "calibrated": calibrated,
            "sigma_cal_c": sigma_cal,
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
                "accepted_once": entry["used"],
            },
            "candidates": associate.top_candidates(mem["scores"], b) if b not in assoc else [],
        }
        if b in evidence:
            info["delta_t_c"], info["heat_w"] = evidence[b]
        bays_out[b] = info
        if bay.zone not in arrays or bm["occ"] == EMPTY:
            bm["over"] = False
            continue
        x, p = arrays[bay.zone]
        n = len(st.zones[bay.zone].bays)
        i_d, i_s, i_q = 2 + bay.index, 2 + n + bay.index, 2 + 2 * n + bay.index
        sigma = math.sqrt(max(float(p[i_d, i_d]), 0.0) + sigma_cal * sigma_cal)
        # A settling window opens only on a bay whose sigma had come back within its
        # threshold (:func:`_mark_disturbed`).
        bm["over"] = sigma > spec.sigma_fault_c
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
        "smart_fresh": list(fresh_smart),
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


def _declared_state(declared: bool | str, current: str) -> str:
    if declared is True:
        return OCCUPIED
    if declared is False:
        return EMPTY
    return current
