"""Per-bay drive temperature estimates for the DAS solvers (plan sections 2, 4 and 6).

The DAS solvers never see a drive temperature: drives are latent and the
sensors sit next to them, never on them. They read an *estimates block*
instead, one entry per constrained bay (occupied or of unknown occupancy),
built once per tick by ``mpc.step`` and handed over as
``SolverRequest.estimates``. The provider is the Kalman filter of
:mod:`aqua_bridge.control.estimator`; the prior map below is what ``step``
falls back to for display when the estimator fails (the zones it serves are
then in fault). Both build their entries with :func:`estimate_entry`.

Estimates block (plain JSON, one entry per bay)::

    {"zone": "z0", "class": "hdd", "occupancy": "occupied" | "unknown",
     "t": 44.1,        # estimated drive temperature, degC (drive-reported equivalent)
     "sigma": 1.5,     # its standard deviation, degC
     "k_sigma": 2.0,   # k in margin = k * sigma
     "margin": 3.0,    # k * sigma, degC
     "limit": 50.0, "comfort": 5.0,
     "soft": 42.0,     # limit - comfort - k * sigma
     "hard": 47.0,     # limit - k * sigma
     "source": "estimator" | "prior_map", "calibrated": false}

The estimator adds ``q_w`` (the heat it attributes to the drive, W),
``t_sensor`` (the filtered proximal sensor node, degC) and ``sigma_cal``
(the calibration floor inside ``sigma``, degC).

A bay that is ``empty`` (declared ``occupied: false``, or found empty by the
estimator's occupancy machine) carries no constraint and has no entry.
``unknown`` is constrained exactly like ``occupied`` (conservative: an
unknown bay might hold a drive).

Prior-map provider (:func:`prior_estimates`)
--------------------------------------------
No filter, no memory. The proximal sensor model of plan section 2 at steady
state, ``T_s = (1 - beta) T_d + beta T_a + b``, is inverted with the prior
``beta = 0.3`` and ``b = -(1 - beta) * 3 degC`` (a typical case-to-internal
offset):

    T_d = (T_s - beta * T_a - b) / (1 - beta)

``T_s`` is the hottest trusted proximal reading of the bay and ``T_a`` the
coolest trusted zone-air reading of its zone. Both choices are the
conservative ones: ``T_d`` rises with ``T_s`` and falls with ``T_a``. The
sensor lag and the drive's own time constant are ignored (a prior map on a
lagging sensor is exactly what the estimator improves on), and
``sigma`` is the uncalibrated calibration floor, 1.5 degC, for every bay:
without SMART the absolute offset between sensor and drive is a prior. A
constrained bay without a trusted proximal reading or zone-air reading gets
no entry; under the ``strict`` trust rule that only happens in a zone that
is in fault, whose sensors ``step`` withholds from the solver anyway.

Targets (plan section 4, "Drive classes"): ``soft = limit - comfort - k * sigma``
and ``hard = limit - k * sigma`` with the bay's limit from
:meth:`~aqua_bridge.model.MpcConfig.bay_limit` and ``k = 2``.
"""

from __future__ import annotations

from collections.abc import Collection, Mapping
from typing import Any

from aqua_bridge.model import MpcConfig

__all__ = [
    "K_SIGMA",
    "PRIOR_BETA",
    "PRIOR_OFFSET_C",
    "SIGMA_UNCALIBRATED_C",
    "SOURCE_ESTIMATOR",
    "SOURCE_PRIOR_MAP",
    "drive_targets",
    "estimate_entry",
    "occupancy",
    "prior_drive_temp",
    "prior_estimates",
]

#: Prior air fraction of a proximal sensor's reading (plan section 3 table).
PRIOR_BETA = 0.3
#: Prior offset ``b`` of the proximal sensor model, degC: ``-(1 - beta) * 3``.
PRIOR_OFFSET_C = -(1.0 - PRIOR_BETA) * 3.0
#: Calibration floor of an uncalibrated bay, degC (plan section 2).
SIGMA_UNCALIBRATED_C = 1.5
#: ``k`` in ``margin = k * sigma`` (plan section 2, ``estimator.k_sigma`` default).
K_SIGMA = 2.0
#: ``source`` of an entry built by :func:`prior_estimates`.
SOURCE_PRIOR_MAP = "prior_map"
#: ``source`` of an entry built by :mod:`aqua_bridge.control.estimator`.
SOURCE_ESTIMATOR = "estimator"


def drive_targets(
    limit_c: float, comfort_c: float, sigma_c: float, k_sigma: float
) -> tuple[float, float]:
    """``(soft, hard)``: ``limit - comfort - k * sigma`` and ``limit - k * sigma``."""
    margin = k_sigma * sigma_c
    return limit_c - comfort_c - margin, limit_c - margin


def estimate_entry(
    *,
    zone: str,
    drive_class: str,
    occupancy: str,
    t: float,
    sigma: float,
    k_sigma: float,
    limit: float,
    comfort: float,
    source: str,
    calibrated: bool,
    **extra: Any,
) -> dict[str, Any]:
    """One entry of the estimates block (module docstring); ``extra`` keys are appended."""
    soft, hard = drive_targets(limit, comfort, sigma, k_sigma)
    out: dict[str, Any] = {
        "zone": zone,
        "class": drive_class,
        "occupancy": occupancy,
        "t": t,
        "sigma": sigma,
        "k_sigma": k_sigma,
        "margin": k_sigma * sigma,
        "limit": limit,
        "comfort": comfort,
        "soft": soft,
        "hard": hard,
        "source": source,
        "calibrated": calibrated,
    }
    out.update(extra)
    return out


def prior_drive_temp(t_proximal: float, t_air: float) -> float:
    """Drive temperature from the prior proximal map (module docstring)."""
    return (t_proximal - PRIOR_BETA * t_air - PRIOR_OFFSET_C) / (1.0 - PRIOR_BETA)


def occupancy(cfg: MpcConfig, bay: str) -> str:
    """``occupied`` | ``unknown`` | ``empty`` from ``topology.bays.<bay>.occupied``."""
    assert cfg.topology is not None
    declared = cfg.topology.bays[bay].occupied
    if declared is True:
        return "occupied"
    if declared is False:
        return "empty"
    return "unknown"


def prior_estimates(
    cfg: MpcConfig,
    temps: Mapping[str, float],
    zones: Collection[str] | None = None,
) -> dict[str, dict[str, Any]]:
    """Estimates block from trusted temperatures (module docstring).

    ``temps`` holds gate-trusted values only (a missing name is untrusted);
    ``zones`` restricts the block to the bays of those zones (``None``: every
    zone). Legacy configs have no bays and get an empty block. Pure and
    deterministic; entries are in ``topology.bays`` order.
    """
    topo = cfg.topology
    if topo is None:
        return {}
    air: dict[str, list[float]] = {}
    proximal: dict[str, list[float]] = {}
    for name, value in temps.items():
        spec = cfg.sensors.get(name)
        if spec is None:
            continue
        if spec.role == "zone_air" and spec.zone is not None:
            air.setdefault(spec.zone, []).append(float(value))
        elif spec.role == "drive_proximal" and spec.bay is not None:
            proximal.setdefault(spec.bay, []).append(float(value))
    out: dict[str, dict[str, Any]] = {}
    for bay, spec in topo.bays.items():
        if not spec.constrained or (zones is not None and spec.zone not in zones):
            continue
        if bay not in proximal or spec.zone not in air:
            continue
        out[bay] = estimate_entry(
            zone=spec.zone,
            drive_class=cfg.bay_class(bay),
            occupancy=occupancy(cfg, bay),
            t=prior_drive_temp(max(proximal[bay]), min(air[spec.zone])),
            sigma=SIGMA_UNCALIBRATED_C,
            k_sigma=K_SIGMA,
            limit=cfg.bay_limit(bay),
            comfort=cfg.bay_comfort(bay),
            source=SOURCE_PRIOR_MAP,
            calibrated=False,
        )
    return out
