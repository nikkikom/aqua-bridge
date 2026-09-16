"""A small zoned DAS config for the zone, gate and fallback suites.

Three zones, four channels, four bays::

    zone za  channels fa1, fa2   bays a1 (hdd), a2 (ssd_sata)   coupled_to zb
    zone zb  channel  fb1        bay  b1                        coupled_to za
    zone zc  channel  fc1        bay  c1 (occupied: false)      no coupling

Sensors: ``inlet`` (no zone), ``air_a`` + redundant ``air_a2``, ``air_b``,
``air_c``, ``prox_a1`` + redundant ``prox_a1b``, ``prox_a2``, ``prox_b1``,
``prox_c1`` (empty bay) and ``exhaust`` (zone zb). Setpoints sit on the
three zone-air primaries so the legacy PI solver regulates per zone. Tick
quantities are small (``dt = 1``, ``confirm_ticks = 2``, hold 4 s) like the
``fast_cfg`` fixture; the per-sensor Stuck windows keep their role defaults
unless a test overrides them.
"""

from __future__ import annotations

import copy
from collections.abc import Mapping
from typing import Any

from aqua_bridge.model import MpcConfig, PlantObservation

SP = 35.0
PROX_C = 40.0
INLET_C = 25.0

CHANNELS = ("fa1", "fa2", "fb1", "fc1")
ZONE_CHANNELS = {"za": ("fa1", "fa2"), "zb": ("fb1",), "zc": ("fc1",)}


def das_mapping() -> dict[str, Any]:
    """The ``mpc:`` mapping (fresh deep copy on every call)."""
    return copy.deepcopy(
        {
            "solver": "pi",
            "dt": 1.0,
            "horizon": 8,
            "temps": [
                "inlet",
                "air_a",
                "air_a2",
                "air_b",
                "air_c",
                "prox_a1",
                "prox_a1b",
                "prox_a2",
                "prox_b1",
                "prox_c1",
                "exhaust",
            ],
            "setpoints": {"air_a": SP, "air_b": SP, "air_c": SP},
            "pwm_min": 0.15,
            "pwm_max": 1.0,
            "d_pwm_max": 0.1,
            "fallback_pwm": dict.fromkeys(CHANNELS, 0.8),
            "fallback_hold_s": 4.0,
            "confirm_s": 2.0,
            "dT_max_c_per_s": 2.0,
            "stuck_s": 4.0,
            "stuck_eps_c": 0.02,
            "stuck_pwm_net": 0.15,
            "stuck_sibling_dT_c": 1.0,
            "channels": list(CHANNELS),
            "pi_kp": 0.05,
            "pi_ki": 0.002,
            "topology": {
                "zones": {
                    "za": {"channels": ["fa1", "fa2"], "coupled_to": ["zb"], "inlet": "inlet"},
                    "zb": {"channels": ["fb1"], "coupled_to": ["za"]},
                    "zc": {"channels": ["fc1"]},
                },
                "bays": {
                    "a1": {"zone": "za", "class": "hdd"},
                    "a2": {"zone": "za", "class": "ssd_sata", "occupied": True},
                    "b1": {"zone": "zb"},
                    "c1": {"zone": "zc", "occupied": False},
                },
            },
            "sensors": {
                "inlet": {"role": "inlet"},
                "air_a": {"role": "zone_air", "zone": "za", "quant_c": 0.01},
                "air_a2": {"role": "zone_air", "zone": "za", "quant_c": 0.01, "redundant": True},
                "air_b": {"role": "zone_air", "zone": "zb", "quant_c": 0.01},
                "air_c": {"role": "zone_air", "zone": "zc", "quant_c": 0.01},
                "prox_a1": {"role": "drive_proximal", "zone": "za", "bay": "a1", "tau_s": 15},
                "prox_a1b": {
                    "role": "drive_proximal",
                    "zone": "za",
                    "bay": "a1",
                    "redundant": True,
                },
                "prox_a2": {"role": "drive_proximal", "zone": "za", "bay": "a2"},
                "prox_b1": {"role": "drive_proximal", "zone": "zb", "bay": "b1"},
                "prox_c1": {"role": "drive_proximal", "zone": "zc", "bay": "c1"},
                "exhaust": {"role": "exhaust", "zone": "zb"},
            },
            "fans": {
                "fa1": {"model": "p12", "count": 2, "group": "front"},
                "fa2": {"model": "p12", "group": "front"},
                "fb1": {"model": "p14"},
                "fc1": {"model": "p14", "noise_weight": 0.5, "forbidden_pwm": [[0.4, 0.5]]},
            },
            "fan_models": {
                "p12": {"rpm_max": 1800, "noise_db_at_max": 30.0},
                "p14": {"rpm_max": 1500, "deadband": 0.2, "exponent": 1.1},
            },
        }
    )


def das_cfg(**changes: Any) -> MpcConfig:
    """``MpcConfig`` from :func:`das_mapping` with top-level keys replaced by ``changes``."""
    data = das_mapping()
    data.update(changes)
    return MpcConfig.from_mapping(data)


def default_temps(cfg: MpcConfig) -> dict[str, float | None]:
    """Setpoint on setpoint sensors, 35 degC air, 40 degC proximal, 25 degC inlet, 38 exhaust."""
    out: dict[str, float | None] = {}
    for name in cfg.temps:
        role = cfg.sensors[name].role
        if name in cfg.setpoints:
            out[name] = cfg.setpoints[name]
        elif role == "zone_air":
            out[name] = SP
        elif role == "drive_proximal":
            out[name] = PROX_C
        elif role == "inlet":
            out[name] = INLET_C
        else:
            out[name] = 38.0
    return out


def das_obs(
    cfg: MpcConfig,
    ts: float,
    *,
    temps: Mapping[str, float | None] | None = None,
    pwm: float | Mapping[str, float | None] = 0.5,
    rpm: Mapping[str, float | None] | None = None,
    drop: tuple[str, ...] = (),
    **overrides: float | None,
) -> PlantObservation:
    """A well-formed observation: :func:`default_temps` patched by ``overrides``,
    keys in ``drop`` removed, PWM ``pwm`` on every channel, 1000 rpm unless ``rpm``
    says otherwise."""
    t = default_temps(cfg) if temps is None else dict(temps)
    t.update(overrides)
    for name in drop:
        t.pop(name, None)
    p = dict.fromkeys(cfg.channels, float(pwm)) if isinstance(pwm, int | float) else dict(pwm)
    r = dict.fromkeys(cfg.channels, 1000.0) if rpm is None else dict(rpm)
    return PlantObservation(temps=t, rpm=r, pwm=p, ts=ts)
