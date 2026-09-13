"""DAS truth plant: zoned air, latent drives, proximal sensors (plan section 3, "Simulator").

This is the *truth* that later milestones (estimator, zoned fallback, PI-DAS,
MPC-DAS, identification) are tested against. It is deliberately richer than
any controller model: the controller never sees these parameters, only the
observations. It shares nothing with :class:`aqua_bridge.model.MpcConfig`
beyond the observation types, except :func:`topology_from_config`, which maps a zoned config onto
the plain topology dict accepted by :func:`build_das_params`.

Physics (SI: W, J/K, W/K, degC, s)
----------------------------------
Per zone ``z`` one air node, per bay ``j`` one drive node (when a drive is
inserted), per sensor one lag node, per bay one SMART lag node::

    C_a dT_a/dt = sum_j h_j (T_d,j - T_a) + P_misc,z
                  - (Q_z + leak_z)(T_a - T_in,z) + sum_z' kappa_zz' (T_a,z' - T_a)
    C_d dT_d/dt = P_j(t) - h_j (T_d,j - T_a)          h_j = g0_j + k_j * Qn_z
    tau_s dT_s/dt = target_s - T_s
    tau_m dT_m/dt = T_d,j - T_m                        (SMART's internal lag)

    target_s = (1 - beta) T_d,j + beta T_a,z + offset   drive_proximal, bay occupied
             = T_a,z + offset                          drive_proximal, bay empty
             = T_a,z + offset                          zone_air, exhaust
             = T_in,z + offset                         inlet (ambient if no zone)

Airflow: each output (PWM channel) drives ``count`` fans (1 = direct, 2 = a
splitter). Fan ``k`` of output ``i`` turns at
``rpm_k = rpm_max,k * clip((u - u0_i) / (1 - u0_i), 0, 1)`` (dead band ``u0``),
lagged by ``tau_s`` and zero when stalled, and moves
``e_i * fouling_i(t) * (rpm_k / rpm_max,i)^exponent_i`` W/K of air. The
output's airflow splits over zones by ``share``. ``Qn_z`` is ``Q_z``
normalised by the zone's clean full-speed airflow. ``kappa`` is symmetric
(inter-zone leakage conserves energy). Only the first fan of an output
drives the tachometer (splitter sense wire), and an output may have none
(its key is then absent from ``obs.rpm``, exactly as the hwmon adapter does
without ``rpm:``).

Integration: within one ``dt`` the system is linear (the fan state is
updated once per tick), so every substep is **backward Euler**
``(I - hA) x1 = x0 + h b(t1)``. That is unconditionally stable for the
stiff air nodes and conserves energy exactly in the discrete sense:
``sum C dT = h (sum P - sum (Q + leak)(T_a - T_in))`` with every flow at
the new state, which :attr:`DasPlant.energy` accounts (tested).
Substeps: at least ``substeps``, and ``h <= min(max_substep_s, tau_air_min)``.

Sensing: sensor node plus a per-tick Gaussian noise draw, then
quantisation per sensor type (thermistor 0.01, DS18B20 0.0625 degC);
optional dropout (``None``). SMART: per inserted drive with a serial, a
sample ``round(T_m + offset)`` to 1 degC every 30-60 s (seeded cadence).

Schedules: all times are **plant seconds** (``ts``), not tick indices.
``heat_schedule[bay]`` is ``[(t_s, activity)]`` (activity 0 = idle, 1 =
full load, piecewise constant, evaluated at every substep). ``bay_schedule``
events (``remove`` / ``insert`` with class, serial, model, temp) take
effect at the first tick boundary at or after ``t_s``. ``inlet`` has a
base temperature, a linear drift, a sinusoidal swing and step changes.
Fan fouling reduces ``e`` linearly per day down to a floor.

Determinism: everything random is drawn from generators seeded by
``(seed, purpose[, index])``, so a run is bit-reproducible per seed, and
:meth:`DasPlant.observe` is idempotent (noise is drawn once per tick in
:meth:`DasPlant.advance`). The ``rich`` preset draws the unknown physical
parameters (placement offsets, ``beta``, drive and fan spread, dead band,
exponent, fouling, inlet drift, SMART offset and lag, activity bursts,
undeclared inter-zone leakage, resonance bands, a tach-less output (the
output whose name sorts last),
actuator delay) from the seed; explicit topology values always win.
``basic`` uses nominal values, no sensor noise, no drift.

The plant never raises during a run for physical reasons, never touches
wall clocks or I/O, and knows nothing about the controller.
"""

from __future__ import annotations

import math
from collections.abc import Callable, Iterable, Mapping, Sequence
from dataclasses import dataclass, field
from typing import Any

import numpy as np

from aqua_bridge.model import MpcCommand, MpcConfig, MpcState, PlantObservation
from aqua_bridge.sim.plant import TickRecord

__all__ = [
    "DRIVE_CLASSES",
    "PRESETS",
    "ROLES",
    "SENSOR_TYPES",
    "SMART_QUANT_C",
    "BayEvent",
    "BayParams",
    "DasPlant",
    "DasPlantParams",
    "DasRun",
    "DriveClassParams",
    "DriveSpec",
    "FanParams",
    "InletParams",
    "SensorParams",
    "SensorTypeParams",
    "ZoneParams",
    "build_das_params",
    "build_das_plant",
    "default_topology",
    "topology_from_config",
    "quantise",
    "run_das_closed_loop",
]

ROLES: tuple[str, ...] = ("inlet", "zone_air", "drive_proximal", "exhaust")
PRESETS: tuple[str, ...] = ("basic", "rich")
SMART_QUANT_C = 1.0
_TAU_MIN = 1e-3


# ---------------------------------------------------------------------------
# Parameter dataclasses
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class DriveClassParams:
    """Nominal truth per drive class. ``limit_c`` is used for margins in :class:`DasRun`."""

    limit_c: float
    heat_idle_w: float
    heat_active_w: float
    c_j_per_k: float
    g0_w_per_k: float
    k_w_per_k: float


#: Nominal classes. HDD tau = 500 / 0.8 ~ 10 min at full airflow, SSD ~ 3 min.
DRIVE_CLASSES: dict[str, DriveClassParams] = {
    "hdd": DriveClassParams(50.0, 5.0, 8.0, 500.0, 0.3, 0.5),
    "ssd_sata": DriveClassParams(65.0, 1.0, 4.0, 150.0, 0.3, 0.5),
    "nvme": DriveClassParams(70.0, 3.0, 8.0, 120.0, 0.3, 0.6),
}


@dataclass(frozen=True)
class SensorTypeParams:
    quant_c: float
    noise_sigma_c: float
    tau_s: float


#: Quantisation, white noise (``rich`` only) and default lag per sensor type.
SENSOR_TYPES: dict[str, SensorTypeParams] = {
    "thermistor": SensorTypeParams(0.01, 0.02, 5.0),
    "ds18b20": SensorTypeParams(0.0625, 0.03, 15.0),
}


@dataclass(frozen=True)
class DriveSpec:
    """One physical drive (it moves with hot swap, so it is not a bay property)."""

    drive_class: str
    serial: str | None = None
    model: str = ""
    c_j_per_k: float = 500.0
    g0_w_per_k: float = 0.3
    k_w_per_k: float = 0.5
    heat_idle_w: float = 5.0
    heat_active_w: float = 8.0
    limit_c: float = 50.0
    smart: bool = True
    smart_offset_c: float = 0.0
    smart_tau_s: float = 0.0

    def __post_init__(self) -> None:
        if self.drive_class not in DRIVE_CLASSES:
            raise ValueError(
                f"unknown drive class {self.drive_class!r}; known: {sorted(DRIVE_CLASSES)}"
            )
        for name in ("c_j_per_k", "g0_w_per_k"):
            if not getattr(self, name) > 0:
                raise ValueError(f"drive {name} must be > 0")
        if self.k_w_per_k < 0 or self.heat_idle_w < 0 or self.heat_active_w < 0:
            raise ValueError("drive k and heat must be >= 0")
        if self.smart_tau_s < 0:
            raise ValueError("smart_tau_s must be >= 0")

    @classmethod
    def nominal(cls, drive_class: str, **overrides: Any) -> DriveSpec:
        if drive_class not in DRIVE_CLASSES:
            raise ValueError(f"unknown drive class {drive_class!r}; known: {sorted(DRIVE_CLASSES)}")
        c = DRIVE_CLASSES[drive_class]
        base = {
            "c_j_per_k": c.c_j_per_k,
            "g0_w_per_k": c.g0_w_per_k,
            "k_w_per_k": c.k_w_per_k,
            "heat_idle_w": c.heat_idle_w,
            "heat_active_w": c.heat_active_w,
            "limit_c": c.limit_c,
        }
        base.update(overrides)
        return cls(drive_class=drive_class, **base)

    def heat_w(self, activity: float) -> float:
        a = min(1.0, max(0.0, float(activity)))
        return self.heat_idle_w + a * (self.heat_active_w - self.heat_idle_w)


@dataclass(frozen=True)
class ZoneParams:
    name: str
    c_air_j_per_k: float = 200.0
    leak_w_per_k: float = 1.0
    inlet_offset_c: float = 0.0
    heat_w: float = 0.0
    kappa: dict[str, float] = field(default_factory=dict)

    def __post_init__(self) -> None:
        if not self.c_air_j_per_k > 0:
            raise ValueError(f"zone {self.name}: c_air_j_per_k must be > 0")
        if self.leak_w_per_k < 0 or self.heat_w < 0:
            raise ValueError(f"zone {self.name}: leak and heat must be >= 0")
        if any(v < 0 for v in self.kappa.values()):
            raise ValueError(f"zone {self.name}: kappa must be >= 0")


@dataclass(frozen=True)
class BayParams:
    name: str
    zone: str
    drive: DriveSpec | None = None
    activity: float = 0.0


@dataclass(frozen=True)
class FanParams:
    """One PWM output: ``count`` fans (a splitter when 2), one tach wire at most."""

    name: str
    shares: dict[str, float]
    count: int = 1
    tach: bool = True
    rpm_max: float = 1500.0
    rpm_spread: tuple[float, ...] = ()
    deadband: float = 0.1
    exponent: float = 1.0
    e_w_per_k: float = 33.0
    fouling_per_day: float = 0.0
    fouling_floor: float = 0.5
    tau_s: float = 0.0
    noise_db_at_max: float = 30.0
    resonance: tuple[tuple[float, float, float], ...] = ()

    def __post_init__(self) -> None:
        if self.count < 1:
            raise ValueError(f"fan {self.name}: count must be >= 1")
        if not self.shares or any(v <= 0 for v in self.shares.values()):
            raise ValueError(f"fan {self.name}: needs at least one zone with share > 0")
        if not 0 <= self.deadband < 1:
            raise ValueError(f"fan {self.name}: deadband must be in [0, 1)")
        if not self.exponent > 0 or not self.rpm_max > 0 or self.e_w_per_k < 0:
            raise ValueError(f"fan {self.name}: exponent and rpm_max > 0, e >= 0")
        if self.fouling_per_day < 0 or not 0 < self.fouling_floor <= 1 or self.tau_s < 0:
            raise ValueError(f"fan {self.name}: bad fouling or tau")
        spread = tuple(float(s) for s in self.rpm_spread) or (1.0,) * self.count
        if len(spread) != self.count or any(s <= 0 for s in spread):
            raise ValueError(f"fan {self.name}: rpm_spread needs {self.count} positive values")
        object.__setattr__(self, "rpm_spread", spread)
        object.__setattr__(self, "resonance", tuple(tuple(map(float, r)) for r in self.resonance))

    def rpm_frac(self, pwm: float) -> float:
        return min(1.0, max(0.0, (pwm - self.deadband) / (1.0 - self.deadband)))


@dataclass(frozen=True)
class SensorParams:
    name: str
    role: str
    type: str
    zone: str | None = None
    bay: str | None = None
    quant_c: float = 0.0625
    noise_sigma_c: float = 0.0
    tau_s: float = 15.0
    offset_c: float = 0.0
    beta: float = 0.3
    dropout_prob: float = 0.0

    def __post_init__(self) -> None:
        if self.role not in ROLES:
            raise ValueError(f"sensor {self.name}: role must be one of {ROLES}")
        if self.quant_c < 0 or self.noise_sigma_c < 0 or self.tau_s < 0:
            raise ValueError(f"sensor {self.name}: quant, noise and tau must be >= 0")
        if not 0 <= self.beta <= 1 or not 0 <= self.dropout_prob <= 1:
            raise ValueError(f"sensor {self.name}: beta and dropout_prob must be in [0, 1]")


@dataclass(frozen=True)
class InletParams:
    base_c: float = 25.0
    drift_c_per_h: float = 0.0
    swing_c: float = 0.0
    swing_period_s: float = 86400.0
    schedule: tuple[tuple[float, float], ...] = ()

    def ambient(self, t: float) -> float:
        base = self.base_c
        for t_s, value in self.schedule:
            if t >= t_s:
                base = value
        out = base + self.drift_c_per_h * t / 3600.0
        if self.swing_c and self.swing_period_s > 0:
            out += self.swing_c * math.sin(2.0 * math.pi * t / self.swing_period_s)
        return out


@dataclass(frozen=True)
class BayEvent:
    t_s: float
    bay: str
    action: str
    drive: DriveSpec | None = None
    temp_c: float | None = None

    def __post_init__(self) -> None:
        if self.action not in ("remove", "insert"):
            raise ValueError(f"bay event action must be remove or insert, got {self.action!r}")
        if self.action == "insert" and self.drive is None:
            raise ValueError("an insert event needs a drive")


@dataclass(frozen=True)
class DasPlantParams:
    dt: float
    zones: tuple[ZoneParams, ...]
    bays: tuple[BayParams, ...]
    fans: tuple[FanParams, ...]
    sensors: tuple[SensorParams, ...]
    inlet: InletParams = field(default_factory=InletParams)
    heat_schedule: dict[str, tuple[tuple[float, float], ...]] = field(default_factory=dict)
    bay_schedule: tuple[BayEvent, ...] = ()
    burst_prob: float = 0.0
    burst_window_s: float = 300.0
    delay_ticks: int = 0
    substeps: int = 2
    max_substep_s: float = 1.0
    smart_cadence_s: tuple[float, float] = (30.0, 60.0)
    noise_exponent: float = 5.0
    seed: int = 0

    def __post_init__(self) -> None:
        if not self.dt > 0 or self.substeps < 1 or self.delay_ticks < 0:
            raise ValueError("dt > 0, substeps >= 1, delay_ticks >= 0")
        if not self.max_substep_s > 0:
            raise ValueError("max_substep_s must be > 0")
        lo, hi = self.smart_cadence_s
        if not 0 < lo <= hi:
            raise ValueError("smart_cadence_s must be 0 < lo <= hi")
        zones = {z.name for z in self.zones}
        if not zones or len(zones) != len(self.zones):
            raise ValueError("zones must be non-empty with unique names")
        for z in self.zones:
            for other in z.kappa:
                if other not in zones or other == z.name:
                    raise ValueError(f"zone {z.name}: kappa to unknown zone {other!r}")
        bays = {b.name: b for b in self.bays}
        if len(bays) != len(self.bays):
            raise ValueError("bay names must be unique")
        for b in self.bays:
            if b.zone not in zones:
                raise ValueError(f"bay {b.name}: unknown zone {b.zone!r}")
        names = [f.name for f in self.fans]
        if not names or len(set(names)) != len(names):
            raise ValueError("fans must be non-empty with unique names")
        for f in self.fans:
            for z in f.shares:
                if z not in zones:
                    raise ValueError(f"fan {f.name}: unknown zone {z!r}")
        snames = [s.name for s in self.sensors]
        if len(set(snames)) != len(snames):
            raise ValueError("sensor names must be unique")
        for s in self.sensors:
            if s.role in ("zone_air", "drive_proximal", "exhaust") and s.zone is None:
                raise ValueError(f"sensor {s.name}: role {s.role} needs a zone")
            if s.zone is not None and s.zone not in zones:
                raise ValueError(f"sensor {s.name}: unknown zone {s.zone!r}")
            if s.role == "drive_proximal":
                if s.bay not in bays:
                    raise ValueError(f"sensor {s.name}: drive_proximal needs a known bay")
                if bays[s.bay].zone != s.zone:
                    raise ValueError(f"sensor {s.name}: bay {s.bay} is not in zone {s.zone}")
            elif s.bay is not None:
                raise ValueError(f"sensor {s.name}: only drive_proximal sensors have a bay")
        for bay in self.heat_schedule:
            if bay not in bays:
                raise ValueError(f"heat_schedule: unknown bay {bay!r}")
        for ev in self.bay_schedule:
            if ev.bay not in bays:
                raise ValueError(f"bay_schedule: unknown bay {ev.bay!r}")

    @property
    def zone_names(self) -> tuple[str, ...]:
        return tuple(z.name for z in self.zones)

    @property
    def bay_names(self) -> tuple[str, ...]:
        return tuple(b.name for b in self.bays)

    @property
    def channels(self) -> tuple[str, ...]:
        return tuple(f.name for f in self.fans)

    @property
    def sensor_names(self) -> tuple[str, ...]:
        return tuple(s.name for s in self.sensors)


# ---------------------------------------------------------------------------
# Topology dict -> parameters
# ---------------------------------------------------------------------------

_TOP_KEYS = {"zones", "bays", "fans", "sensors", "inlet", "heat_schedule", "bay_schedule"}
_ZONE_KEYS = {"c_air_j_per_k", "leak_w_per_k", "inlet_offset_c", "heat_w", "coupled_to", "channels"}
_DRIVE_KEYS = {
    "c_j_per_k",
    "g0_w_per_k",
    "k_w_per_k",
    "heat_idle_w",
    "heat_active_w",
    "limit_c",
    "smart",
    "smart_offset_c",
    "smart_tau_s",
}
_BAY_KEYS = {"zone", "class", "occupied", "serial", "model", "activity"} | _DRIVE_KEYS
_FAN_KEYS = {
    "zone",
    "zones",
    "count",
    "tach",
    "rpm_max",
    "rpm_spread",
    "deadband",
    "exponent",
    "e_w_per_k",
    "fouling_per_day",
    "fouling_floor",
    "tau_s",
    "noise_db_at_max",
    "resonance",
    "model",
    "group",
}
_SENSOR_KEYS = {
    "role",
    "type",
    "zone",
    "bay",
    "quant_c",
    "noise_sigma_c",
    "tau_s",
    "offset_c",
    "beta",
    "dropout_prob",
    "redundant",
}
_INLET_KEYS = {"base_c", "drift_c_per_h", "swing_c", "swing_period_s", "schedule"}
_EVENT_KEYS = {"t_s", "bay", "action", "class", "serial", "model", "temp_c"} | _DRIVE_KEYS


def _mapping(where: str, value: object, allowed: set[str] | None = None) -> dict[str, Any]:
    if value is None:
        return {}
    if not isinstance(value, Mapping):
        raise ValueError(f"{where} must be a mapping, got {type(value).__name__}")
    out = {str(k): v for k, v in value.items()}
    if allowed is not None:
        unknown = sorted(set(out) - allowed)
        if unknown:
            raise ValueError(f"{where}: unknown keys {unknown}")
    return out


def default_topology() -> dict[str, Any]:
    """A 4-zone, 15-bay, 8-output DAS used by tests and ``--sim-plant das`` examples.

    Outputs ``xt1..xt4`` (aquaero) carry 1-2 fans, ``qd1..qd4`` (Quadro) one
    each; ``qd4`` has no tach. Eight thermistors, the rest DS18B20: two inlets
    (one of each type), four zone-air and two exhaust thermistors, a DS18B20
    on every bay, and two redundant proximal sensors (one a thermistor). The
    bay classes and the fast sensor positions are a simulation fixture, not
    an installation assumption.
    """
    zone_bays = {"z0": 4, "z1": 4, "z2": 4, "z3": 3}
    zones: dict[str, Any] = {}
    bays: dict[str, Any] = {}
    sensors: dict[str, Any] = {
        "inlet_a": {"role": "inlet", "type": "thermistor"},
        "inlet_b": {"role": "inlet", "type": "ds18b20"},
    }
    n = 0
    order = list(zone_bays)
    for zi, (z, nb) in enumerate(zone_bays.items()):
        coupled = [order[k] for k in (zi - 1, zi + 1) if 0 <= k < len(order)]
        zones[z] = {"coupled_to": coupled}
        sensors[f"air_{z}"] = {"role": "zone_air", "zone": z, "type": "thermistor"}
        for _ in range(nb):
            n += 1
            b = f"b{n:02d}"
            bays[b] = {
                "zone": z,
                "class": "ssd_sata" if n in (4, 8) else "hdd",
                "serial": f"SN{n:04d}",
            }
            sensors[f"prox_{b}"] = {
                "role": "drive_proximal",
                "zone": z,
                "bay": b,
                "type": "ds18b20",
            }
    sensors["prox_b03b"] = {
        "role": "drive_proximal",
        "zone": "z0",
        "bay": "b03",
        "type": "thermistor",
    }
    sensors["prox_b10b"] = {"role": "drive_proximal", "zone": "z2", "bay": "b10", "type": "ds18b20"}
    sensors["exhaust_z0"] = {"role": "exhaust", "zone": "z0", "type": "thermistor"}
    sensors["exhaust_z2"] = {"role": "exhaust", "zone": "z2", "type": "thermistor"}
    fans = {
        "xt1": {"zone": "z0", "count": 2},
        "xt2": {"zone": "z1", "count": 2},
        "xt3": {"zone": "z2", "count": 2},
        "xt4": {"zone": "z3", "count": 1},
        "qd1": {"zones": {"z0": 0.7, "z1": 0.3}},
        "qd2": {"zones": {"z1": 0.5, "z2": 0.5}},
        "qd3": {"zones": {"z2": 0.3, "z3": 0.7}},
        "qd4": {"zone": "z3", "tach": False},
    }
    return {"zones": zones, "bays": bays, "fans": fans, "sensors": sensors}


#: A sensor with ``quant_c`` at or below this is simulated as a thermistor, else a DS18B20.
THERMISTOR_MAX_QUANT_C = 0.02


def topology_from_config(cfg: MpcConfig) -> dict[str, Any]:
    """The plain topology dict for :func:`build_das_params` from a zoned ``MpcConfig``.

    Truth parameters stay the simulator's (nominal, or drawn by the ``rich``
    preset); the config only supplies the structure: zones and their declared
    couplings, bays with their class (``default_class`` when undeclared; an
    ``occupied: auto`` bay holds a drive, ``false`` is empty), one output per
    channel spread evenly over the zones that list it with ``count`` fans and
    its fan model's ``rpm_max`` / ``deadband`` / ``exponent`` /
    ``noise_db_at_max``, and one sensor per temperature with its role, zone and
    bay (a ``quant_c`` of at most 0.02 degC is a thermistor, anything coarser a
    DS18B20). Raises ``ValueError`` for a config without ``topology``.
    """
    topo = cfg.topology
    if topo is None:
        raise ValueError("topology_from_config needs a zoned config (mpc.topology)")
    zones = {z: {"coupled_to": list(spec.coupled_to)} for z, spec in topo.zones.items()}
    bays: dict[str, Any] = {}
    for b, bay in topo.bays.items():
        entry: dict[str, Any] = {"zone": bay.zone, "occupied": bay.occupied is not False}
        drive_class = cfg.bay_class(b)
        entry["class"] = drive_class if drive_class in DRIVE_CLASSES else "hdd"
        if bay.serial is not None:
            entry["serial"] = bay.serial
        bays[b] = entry
    fans: dict[str, Any] = {}
    for ch in cfg.channels:
        listed = [z for z, spec in topo.zones.items() if ch in spec.channels]
        entry = {"zones": {z: 1.0 / len(listed) for z in listed}}
        spec = cfg.fans.get(ch)
        if spec is not None:
            entry["count"] = spec.count
            model = cfg.fan_models.get(spec.model)
            if model is not None:
                entry.update(
                    rpm_max=model.rpm_max,
                    deadband=model.deadband,
                    exponent=model.exponent,
                    noise_db_at_max=model.noise_db_at_max,
                )
        fans[ch] = entry
    sensors: dict[str, Any] = {}
    for name in cfg.temps:
        sp = cfg.sensors[name]
        entry = {
            "role": sp.role,
            "type": "thermistor" if sp.quant_c <= THERMISTOR_MAX_QUANT_C else "ds18b20",
        }
        if sp.zone is not None:
            entry["zone"] = sp.zone
        if sp.bay is not None:
            entry["bay"] = sp.bay
        sensors[name] = entry
    return {"zones": zones, "bays": bays, "fans": fans, "sensors": sensors}


def _num(where: str, value: object) -> float:
    if isinstance(value, bool) or not isinstance(value, int | float):
        raise ValueError(f"{where} must be a number, got {value!r}")
    out = float(value)
    if not math.isfinite(out):
        raise ValueError(f"{where} must be finite")
    return out


def _drive_from(where: str, entry: Mapping[str, Any], rich: _Rich | None, key: str) -> DriveSpec:
    klass = str(entry.get("class", "hdd"))
    if klass not in DRIVE_CLASSES:
        raise ValueError(f"{where}: unknown class {klass!r}; known: {sorted(DRIVE_CLASSES)}")
    nominal = DRIVE_CLASSES[klass]
    kw: dict[str, Any] = {
        "c_j_per_k": nominal.c_j_per_k,
        "g0_w_per_k": nominal.g0_w_per_k,
        "k_w_per_k": nominal.k_w_per_k,
        "heat_idle_w": nominal.heat_idle_w,
        "heat_active_w": nominal.heat_active_w,
        "limit_c": nominal.limit_c,
    }
    if rich is not None:
        r = rich.gen("drive", key)
        for name in ("c_j_per_k", "g0_w_per_k", "k_w_per_k"):
            kw[name] *= float(r.uniform(0.7, 1.3))
        for name in ("heat_idle_w", "heat_active_w"):
            kw[name] *= float(r.uniform(0.8, 1.2))
        kw["heat_active_w"] = max(kw["heat_active_w"], kw["heat_idle_w"])
        kw["smart_offset_c"] = float(r.uniform(-2.0, 2.0))
        kw["smart_tau_s"] = float(r.uniform(60.0, 240.0))
    for name in _DRIVE_KEYS:
        if name in entry:
            kw[name] = (
                bool(entry[name]) if name == "smart" else _num(f"{where}.{name}", entry[name])
            )
    serial = entry.get("serial")
    return DriveSpec(
        drive_class=klass,
        serial=None if serial is None else str(serial),
        model=str(entry.get("model", "")),
        **kw,
    )


class _Rich:
    """Seeded draws keyed by purpose and name, independent of dict order."""

    def __init__(self, seed: int) -> None:
        self.seed = int(seed)

    def gen(self, purpose: str, name: str) -> np.random.Generator:
        key = [self.seed, *(ord(c) for c in f"{purpose}:{name}")]
        return np.random.default_rng(key)


def _pairs(where: str, value: object) -> tuple[tuple[float, float], ...]:
    if value is None:
        return ()
    if isinstance(value, str | bytes) or not isinstance(value, Iterable):
        raise ValueError(f"{where} must be a list of [t_s, value] pairs")
    out = []
    for item in value:
        if isinstance(item, str | bytes) or not isinstance(item, Sequence) or len(item) != 2:
            raise ValueError(f"{where} entries must be [t_s, value], got {item!r}")
        out.append((_num(where, item[0]), _num(where, item[1])))
    return tuple(sorted(out, key=lambda p: p[0]))


def build_das_params(
    topology: Mapping[str, Any],
    *,
    preset: str = "basic",
    seed: int = 0,
    dt: float = 2.0,
    heat_schedule: Mapping[str, Any] | None = None,
    bay_schedule: Iterable[Mapping[str, Any]] | None = None,
    **overrides: Any,
) -> DasPlantParams:
    """Parameters from a plain topology dict ``{zones, bays, fans, sensors}``.

    Optional topology keys ``inlet``, ``heat_schedule``, ``bay_schedule``
    (the keyword arguments of the same name take precedence). Any physical
    key in an entry overrides the preset. ``overrides`` go to
    :class:`DasPlantParams` (for example ``delay_ticks``, ``burst_prob``).
    Controller-side keys that the truth does not use (``zones.<z>.channels``,
    ``fans.<ch>.model`` / ``group``, ``sensors.<s>.redundant``) are accepted
    and ignored, so a later milestone can pass its zoned sections through.
    Raises ``ValueError`` on unknown keys and inconsistent topology.
    """
    if preset not in PRESETS:
        raise ValueError(f"preset must be one of {PRESETS}, got {preset!r}")
    top = _mapping("topology", topology, _TOP_KEYS)
    rich = _Rich(seed) if preset == "rich" else None
    zones_in = _mapping("zones", top.get("zones"))
    if not zones_in:
        raise ValueError("topology.zones must not be empty")

    # -- zones and symmetric kappa -----------------------------------------
    kappa: dict[tuple[str, str], float] = {}
    zone_kw: dict[str, dict[str, Any]] = {}
    for z, raw in zones_in.items():
        entry = _mapping(f"zones.{z}", raw, _ZONE_KEYS)
        kw: dict[str, Any] = {"c_air_j_per_k": 200.0, "leak_w_per_k": 1.0, "heat_w": 0.0}
        if rich is not None:
            r = rich.gen("zone", z)
            kw["c_air_j_per_k"] = float(r.uniform(150.0, 300.0))
            kw["leak_w_per_k"] = float(r.uniform(0.5, 2.0))
            kw["heat_w"] = float(r.uniform(0.0, 3.0))
            kw["inlet_offset_c"] = float(r.uniform(0.0, 0.5))
        for name in ("c_air_j_per_k", "leak_w_per_k", "inlet_offset_c", "heat_w"):
            if name in entry:
                kw[name] = _num(f"zones.{z}.{name}", entry[name])
        zone_kw[z] = kw
        coupled = entry.get("coupled_to") or {}
        if isinstance(coupled, Mapping):
            items = [(str(k), _num(f"zones.{z}.coupled_to", v)) for k, v in coupled.items()]
        else:
            items = [(str(k), None) for k in coupled]
        for other, value in items:
            if other not in zones_in or other == z:
                raise ValueError(f"zones.{z}.coupled_to: unknown zone {other!r}")
            pair = tuple(sorted((z, other)))
            if value is None:
                value = float(rich.gen("kappa", "-".join(pair)).uniform(2.0, 5.0)) if rich else 3.0
            kappa[pair] = max(kappa.get(pair, 0.0), value)  # type: ignore[index]
    if rich is not None:  # undeclared leakage between every other pair
        names = list(zones_in)
        for i, a in enumerate(names):
            for b in names[i + 1 :]:
                pair = (a, b) if a < b else (b, a)
                if pair not in kappa:
                    kappa[pair] = float(rich.gen("kappa", "-".join(pair)).uniform(0.0, 1.0))
    zones = []
    for z, kw in zone_kw.items():
        kz = {}
        for (a, b), v in kappa.items():
            if z in (a, b) and v > 0:
                kz[b if a == z else a] = v
        zones.append(ZoneParams(name=z, kappa=kz, **kw))

    # -- bays ----------------------------------------------------------------
    bays = []
    for b, raw in _mapping("bays", top.get("bays")).items():
        entry = _mapping(f"bays.{b}", raw, _BAY_KEYS)
        if "zone" not in entry:
            raise ValueError(f"bays.{b}: zone is required")
        occupied = entry.get("occupied", True)
        if occupied not in (True, False):
            raise ValueError(f"bays.{b}.occupied must be true or false in the truth plant")
        drive = _drive_from(f"bays.{b}", entry, rich, b) if occupied else None
        activity = _num(f"bays.{b}.activity", entry.get("activity", 0.0))
        bays.append(BayParams(name=b, zone=str(entry["zone"]), drive=drive, activity=activity))

    # -- fans ----------------------------------------------------------------
    fans = []
    fan_items = list(_mapping("fans", top.get("fans")).items())
    tachless = max(ch for ch, _ in fan_items) if fan_items else None
    for ch, raw in fan_items:
        entry = _mapping(f"fans.{ch}", raw, _FAN_KEYS)
        if "zones" in entry:
            zs = entry["zones"]
            if isinstance(zs, Mapping):
                shares = {str(k): _num(f"fans.{ch}.zones", v) for k, v in zs.items()}
            else:
                zl = [str(k) for k in zs]
                shares = {k: 1.0 / len(zl) for k in zl} if zl else {}
        elif "zone" in entry:
            shares = {str(entry["zone"]): 1.0}
        else:
            raise ValueError(f"fans.{ch}: zone or zones is required")
        count = entry.get("count", 1)
        if isinstance(count, bool) or not isinstance(count, int):
            raise ValueError(f"fans.{ch}.count must be an integer")
        kw = {"count": count, "shares": shares}
        resonance_frac = None
        if rich is not None:
            r = rich.gen("fan", ch)
            kw["rpm_spread"] = tuple(float(x) for x in r.uniform(0.95, 1.05, size=count))
            kw["deadband"] = float(r.uniform(0.05, 0.2))
            kw["exponent"] = float(r.uniform(0.8, 1.2))
            kw["e_w_per_k"] = 33.0 * float(r.uniform(0.8, 1.2))
            kw["fouling_per_day"] = float(r.uniform(0.005, 0.02))
            kw["tau_s"] = 3.0
            resonance_frac = float(r.uniform(0.4, 0.7))
            if ch == tachless:
                kw["tach"] = False
        for name in (
            "rpm_max",
            "deadband",
            "exponent",
            "e_w_per_k",
            "fouling_per_day",
            "fouling_floor",
            "tau_s",
            "noise_db_at_max",
        ):
            if name in entry:
                kw[name] = _num(f"fans.{ch}.{name}", entry[name])
        if "tach" in entry:
            kw["tach"] = bool(entry["tach"])
        if "rpm_spread" in entry:
            kw["rpm_spread"] = tuple(_num(f"fans.{ch}.rpm_spread", v) for v in entry["rpm_spread"])
        if "resonance" in entry:
            kw["resonance"] = tuple(tuple(r) for r in entry["resonance"])
        elif resonance_frac is not None:
            lo = resonance_frac * kw.get("rpm_max", 1500.0)
            kw["resonance"] = ((lo, lo + 100.0, 3.0),)
        fans.append(FanParams(name=ch, **kw))

    # -- sensors -------------------------------------------------------------
    sensors = []
    for s, raw in _mapping("sensors", top.get("sensors")).items():
        entry = _mapping(f"sensors.{s}", raw, _SENSOR_KEYS)
        stype = str(entry.get("type", "ds18b20"))
        if stype not in SENSOR_TYPES:
            raise ValueError(f"sensors.{s}.type must be one of {sorted(SENSOR_TYPES)}")
        tp = SENSOR_TYPES[stype]
        role = entry.get("role")
        kw = {"quant_c": tp.quant_c, "tau_s": tp.tau_s, "noise_sigma_c": 0.0, "beta": 0.3}
        if role == "zone_air":
            kw["tau_s"] = min(tp.tau_s, 5.0)
        if rich is not None:
            r = rich.gen("sensor", s)
            kw["noise_sigma_c"] = tp.noise_sigma_c
            kw["offset_c"] = float(
                r.uniform(-0.3, 0.3) if stype == "thermistor" else r.uniform(-0.25, 0.25)
            )
            if role == "zone_air":
                kw["offset_c"] += float(r.uniform(0.0, 0.5))
            kw["beta"] = float(r.uniform(0.15, 0.5))
            kw["tau_s"] *= float(r.uniform(0.7, 1.5))
        for name in ("quant_c", "noise_sigma_c", "tau_s", "offset_c", "beta", "dropout_prob"):
            if name in entry:
                kw[name] = _num(f"sensors.{s}.{name}", entry[name])
        sensors.append(
            SensorParams(
                name=s,
                role=str(role),
                type=stype,
                zone=None if entry.get("zone") is None else str(entry["zone"]),
                bay=None if entry.get("bay") is None else str(entry["bay"]),
                **kw,
            )
        )

    # -- inlet and schedules -------------------------------------------------
    inlet_in = _mapping("inlet", top.get("inlet"), _INLET_KEYS)
    ikw: dict[str, Any] = {}
    if rich is not None:
        r = rich.gen("inlet", "")
        ikw = {
            "drift_c_per_h": float(r.uniform(-0.5, 0.5)),
            "swing_c": float(r.uniform(0.5, 1.5)),
            "swing_period_s": float(r.uniform(2.0, 6.0)) * 3600.0,
        }
    for name in ("base_c", "drift_c_per_h", "swing_c", "swing_period_s"):
        if name in inlet_in:
            ikw[name] = _num(f"inlet.{name}", inlet_in[name])
    ikw["schedule"] = _pairs("inlet.schedule", inlet_in.get("schedule"))
    hs_in = _mapping(
        "heat_schedule", heat_schedule if heat_schedule is not None else top.get("heat_schedule")
    )
    hs = {str(b): _pairs(f"heat_schedule.{b}", v) for b, v in hs_in.items()}
    ev_in = bay_schedule if bay_schedule is not None else top.get("bay_schedule") or ()
    events = []
    for i, raw in enumerate(ev_in):
        entry = _mapping(f"bay_schedule[{i}]", raw, _EVENT_KEYS)
        for req in ("t_s", "bay", "action"):
            if req not in entry:
                raise ValueError(f"bay_schedule[{i}]: {req} is required")
        action = str(entry["action"])
        drive = None
        if action == "insert":
            key = f"{entry['bay']}@{entry['t_s']}"
            drive = _drive_from(f"bay_schedule[{i}]", entry, rich, key)
        temp = entry.get("temp_c")
        events.append(
            BayEvent(
                t_s=_num(f"bay_schedule[{i}].t_s", entry["t_s"]),
                bay=str(entry["bay"]),
                action=action,
                drive=drive,
                temp_c=None if temp is None else _num(f"bay_schedule[{i}].temp_c", temp),
            )
        )
    events.sort(key=lambda e: e.t_s)
    kwargs: dict[str, Any] = {"dt": float(dt), "seed": int(seed)}
    if rich is not None:
        kwargs.update(delay_ticks=1, burst_prob=0.2)
    kwargs.update(overrides)
    return DasPlantParams(
        zones=tuple(zones),
        bays=tuple(bays),
        fans=tuple(fans),
        sensors=tuple(sensors),
        inlet=InletParams(**ikw),
        heat_schedule=hs,
        bay_schedule=tuple(events),
        **kwargs,
    )


def build_das_plant(
    topology: Mapping[str, Any] | None = None,
    *,
    preset: str = "basic",
    seed: int = 0,
    dt: float = 2.0,
    initial_pwm: Mapping[str, float] | float = 0.5,
    ts0: float = 0.0,
    **kwargs: Any,
) -> DasPlant:
    """:func:`build_das_params` then :class:`DasPlant`; ``None``: :func:`default_topology`."""
    params = build_das_params(
        default_topology() if topology is None else topology,
        preset=preset,
        seed=seed,
        dt=dt,
        **kwargs,
    )
    return DasPlant(params, initial_pwm=initial_pwm, ts0=ts0)


def quantise(value: float, quant: float) -> float:
    """Round to the sensor's LSB (``quant <= 0``: unchanged)."""
    if quant <= 0:
        return float(value)
    return round(math.floor(value / quant + 0.5) * quant, 6)


# ---------------------------------------------------------------------------
# The plant
# ---------------------------------------------------------------------------


class DasPlant:
    """Mutable truth simulation; one instance per run. See the module docstring."""

    def __init__(
        self,
        params: DasPlantParams,
        *,
        initial_pwm: Mapping[str, float] | float = 0.5,
        ts0: float = 0.0,
    ) -> None:
        p = self.params = params
        self.ts = float(ts0)
        self.tick = 0
        self._zi = {z: i for i, z in enumerate(p.zone_names)}
        self._bi = {b: i for i, b in enumerate(p.bay_names)}
        nz, nb, ns = len(p.zones), len(p.bays), len(p.sensors)
        self._nz, self._nb, self._ns = nz, nb, ns
        self._n = nz + 2 * nb + ns  # [T_a | T_d | T_m (SMART) | T_s]
        self.drives: list[DriveSpec | None] = [b.drive for b in p.bays]
        self.stalled: set[tuple[str, int]] = set()
        self._events = list(p.bay_schedule)
        self._burst_cache: dict[tuple[int, int], float] = {}
        self._noise_rng = np.random.default_rng([p.seed, 0])
        self._smart_rng = [np.random.default_rng([p.seed, 1, i]) for i in range(nb)]
        self._next_smart = [math.inf] * nb
        self.smart_samples: dict[str, dict[str, Any]] = {}
        self._qn_max = np.zeros(nz)
        for f in p.fans:
            for z, share in f.shares.items():
                self._qn_max[self._zi[z]] += share * f.e_w_per_k * f.count
        if isinstance(initial_pwm, Mapping):
            start = {ch: float(initial_pwm.get(ch, 0.0)) for ch in p.channels}
        else:
            start = dict.fromkeys(p.channels, float(initial_pwm))
        start = {ch: min(1.0, max(0.0, v)) for ch, v in start.items()}
        self._history: list[dict[str, float]] = [start]
        self._fan_frac = {f.name: np.full(f.count, f.rpm_frac(start[f.name])) for f in p.fans}
        self._noise = np.zeros(ns)
        self._dropout = np.zeros(ns, dtype=bool)
        self.energy = {"in_j": 0.0, "out_j": 0.0, "swap_j": 0.0}
        self.x = self._steady(self._airflow(), self.ts, fill=np.zeros(self._n))
        for i, d in enumerate(self.drives):
            if d is not None:
                self._schedule_smart(i, first=True)
        self.energy["stored0_j"] = self.stored_energy()
        self._draw_noise()

    # -- names and truth accessors ------------------------------------------

    @property
    def channels(self) -> tuple[str, ...]:
        return self.params.channels

    def _slot(self, kind: str, idx: int) -> int:
        nz, nb = self._nz, self._nb
        return {"a": 0, "d": nz, "m": nz + nb, "s": nz + 2 * nb}[kind] + idx

    def t_air(self) -> dict[str, float]:
        return {z: float(self.x[i]) for z, i in self._zi.items()}

    def t_drive(self) -> dict[str, float | None]:
        return {
            b: None if self.drives[i] is None else float(self.x[self._slot("d", i)])
            for b, i in self._bi.items()
        }

    def t_inlet(self, t: float | None = None) -> dict[str, float]:
        amb = self.params.inlet.ambient(self.ts if t is None else t)
        return {z.name: amb + z.inlet_offset_c for z in self.params.zones}

    def ambient(self, t: float | None = None) -> float:
        return self.params.inlet.ambient(self.ts if t is None else t)

    def sensor_truth(self) -> dict[str, float]:
        """Sensor nodes (lagged, with placement offset), before noise and quantisation."""
        base = self._slot("s", 0)
        return {s.name: float(self.x[base + i]) for i, s in enumerate(self.params.sensors)}

    def occupied(self) -> dict[str, bool]:
        return {b: self.drives[i] is not None for b, i in self._bi.items()}

    def activity(self, bay: str, t: float | None = None) -> float:
        t = self.ts if t is None else t
        p = self.params
        bp = p.bays[self._bi[bay]]
        sched = p.heat_schedule.get(bay)
        if sched:
            a = bp.activity
            for t_s, v in sched:
                if t >= t_s:
                    a = v
            return a
        if p.burst_prob > 0:
            key = (self._bi[bay], math.floor(t / p.burst_window_s))
            r = self._burst_cache.get(key)
            if r is None:
                if len(self._burst_cache) > 4096:
                    self._burst_cache.clear()
                r = self._burst_cache[key] = float(
                    np.random.default_rng([p.seed, 2, *key]).random()
                )
            return 1.0 if r < p.burst_prob else bp.activity
        return bp.activity

    def heat_w(self, t: float | None = None) -> dict[str, float]:
        return {
            b: 0.0 if self.drives[i] is None else self.drives[i].heat_w(self.activity(b, t))  # type: ignore[union-attr]
            for b, i in self._bi.items()
        }

    def margins(self) -> dict[str, float | None]:
        """``limit_c - T_d`` per bay (``None`` when empty)."""
        out: dict[str, float | None] = {}
        for b, i in self._bi.items():
            d = self.drives[i]
            out[b] = None if d is None else d.limit_c - float(self.x[self._slot("d", i)])
        return out

    def stored_energy(self) -> float:
        p = self.params
        e = sum(z.c_air_j_per_k * float(self.x[i]) for i, z in enumerate(p.zones))
        for i, d in enumerate(self.drives):
            if d is not None:
                e += d.c_j_per_k * float(self.x[self._slot("d", i)])
        return e

    # -- fans -----------------------------------------------------------------

    def effective_pwm(self) -> dict[str, float]:
        idx = max(0, len(self._history) - 1 - self.params.delay_ticks)
        return dict(self._history[idx])

    def fouling(self, fan: FanParams, t: float | None = None) -> float:
        t = self.ts if t is None else t
        return max(fan.fouling_floor, 1.0 - fan.fouling_per_day * t / 86400.0)

    def fan_rpm(self) -> dict[str, list[float]]:
        """True rpm of every fan of every output."""
        return {
            f.name: [
                f.rpm_max * f.rpm_spread[k] * float(self._fan_frac[f.name][k])
                for k in range(f.count)
            ]
            for f in self.params.fans
        }

    def _airflow(self, t: float | None = None) -> np.ndarray:
        q = np.zeros(self._nz)
        for f in self.params.fans:
            flow = 0.0
            for k in range(f.count):
                rel = f.rpm_spread[k] * float(self._fan_frac[f.name][k])
                flow += rel**f.exponent if rel > 0 else 0.0
            flow *= f.e_w_per_k * self.fouling(f, t)
            for z, share in f.shares.items():
                q[self._zi[z]] += share * flow
        return q

    def airflow(self) -> dict[str, float]:
        """Zone airflow ``Q_z`` in W/K at the current fan state."""
        return {z: float(v) for z, v in zip(self.params.zone_names, self._airflow(), strict=True)}

    def noise_db(self) -> float:
        """Energetic noise index from true rpm (fan law ``N^noise_exponent``, resonance bands)."""
        p = self.params
        power = 0.0
        rpm_all = self.fan_rpm()
        for f in p.fans:
            for rpm in rpm_all[f.name]:
                if rpm <= 0:
                    continue
                level = f.noise_db_at_max + 10.0 * p.noise_exponent * math.log10(rpm / f.rpm_max)
                for lo, hi, extra in f.resonance:
                    if lo <= rpm <= hi:
                        level += extra
                power += 10.0 ** (level / 10.0)
        return 10.0 * math.log10(max(power, 1e-12))

    def stall(self, channel: str, fan: int | None = None) -> None:
        """Stall one fan (``fan`` index) or every fan of an output."""
        f = self.params.fans[self.params.channels.index(channel)]
        for k in range(f.count) if fan is None else (fan,):
            self.stalled.add((channel, k))

    def unstall(self, channel: str, fan: int | None = None) -> None:
        f = self.params.fans[self.params.channels.index(channel)]
        for k in range(f.count) if fan is None else (fan,):
            self.stalled.discard((channel, k))

    def _update_fans(self, pwm: Mapping[str, float], dt: float, lag: bool = True) -> None:
        for f in self.params.fans:
            target = f.rpm_frac(pwm[f.name])
            frac = self._fan_frac[f.name]
            for k in range(f.count):
                tgt = 0.0 if (f.name, k) in self.stalled else target
                if lag and f.tau_s > 0:
                    frac[k] += (tgt - frac[k]) * (1.0 - math.exp(-dt / f.tau_s))
                else:
                    frac[k] = tgt

    # -- linear system ----------------------------------------------------------

    def _system(self, q: np.ndarray, t: float) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """``(A, b, frozen)`` of ``dx/dt = A x + b`` at airflow ``q`` and time ``t``.

        ``frozen`` marks rows with no dynamics (drive and SMART nodes of empty bays).
        """
        p = self.params
        n = self._n
        A = np.zeros((n, n))
        b = np.zeros(n)
        frozen = np.zeros(n, dtype=bool)
        amb = p.inlet.ambient(t)
        qn = np.divide(q, self._qn_max, out=np.zeros_like(q), where=self._qn_max > 0)
        for zi, z in enumerate(p.zones):
            ca = z.c_air_j_per_k
            g_out = q[zi] + z.leak_w_per_k
            A[zi, zi] -= g_out / ca
            b[zi] += (g_out * (amb + z.inlet_offset_c) + z.heat_w) / ca
            for other, kap in z.kappa.items():
                oi = self._zi[other]
                A[zi, zi] -= kap / ca
                A[zi, oi] += kap / ca
        for bi, bay in enumerate(p.bays):
            d = self.drives[bi]
            di, mi = self._slot("d", bi), self._slot("m", bi)
            if d is None:
                frozen[di] = frozen[mi] = True
                continue
            zi = self._zi[bay.zone]
            h = d.g0_w_per_k + d.k_w_per_k * qn[zi]
            A[di, di] -= h / d.c_j_per_k
            A[di, zi] += h / d.c_j_per_k
            b[di] += d.heat_w(self.activity(bay.name, t)) / d.c_j_per_k
            ca = p.zones[zi].c_air_j_per_k
            A[zi, zi] -= h / ca
            A[zi, di] += h / ca
            tau_m = max(d.smart_tau_s, _TAU_MIN)
            A[mi, mi] -= 1.0 / tau_m
            A[mi, di] += 1.0 / tau_m
        for si, s in enumerate(p.sensors):
            row = self._slot("s", si)
            inv = 1.0 / max(s.tau_s, _TAU_MIN)
            A[row, row] -= inv
            b[row] += s.offset_c * inv
            if s.role == "inlet":
                off = 0.0 if s.zone is None else p.zones[self._zi[s.zone]].inlet_offset_c
                b[row] += (amb + off) * inv
                continue
            zi = self._zi[s.zone]  # type: ignore[index]
            if s.role == "drive_proximal" and self.drives[self._bi[s.bay]] is not None:  # type: ignore[index]
                A[row, self._slot("d", self._bi[s.bay])] += (1.0 - s.beta) * inv  # type: ignore[index]
                A[row, zi] += s.beta * inv
            else:
                A[row, zi] += inv
        return A, b, frozen

    def _steady(self, q: np.ndarray, t: float, fill: np.ndarray) -> np.ndarray:
        A, b, frozen = self._system(q, t)
        M = A.copy()
        rhs = -b
        idx = np.flatnonzero(frozen)
        M[idx, :] = 0.0
        M[idx, idx] = 1.0
        rhs[idx] = fill[idx]
        return np.linalg.solve(M, rhs)

    def steady_state(self, pwm: Mapping[str, float] | float) -> dict[str, dict[str, float | None]]:
        """Equilibrium for a constant PWM at the current inputs (activity, ambient, drives).

        Does not change the plant. Returns ``{"t_air", "t_drive", "sensors"}``.
        """
        if not isinstance(pwm, Mapping):
            pwm = dict.fromkeys(self.channels, float(pwm))
        saved = {k: v.copy() for k, v in self._fan_frac.items()}
        try:
            self._update_fans({ch: float(pwm[ch]) for ch in self.channels}, 0.0, lag=False)
            x = self._steady(self._airflow(), self.ts, fill=self.x)
        finally:
            self._fan_frac = saved
        p = self.params
        return {
            "t_air": {z: float(x[i]) for z, i in self._zi.items()},
            "t_drive": {
                b: None if self.drives[i] is None else float(x[self._slot("d", i)])
                for b, i in self._bi.items()
            },
            "sensors": {s.name: float(x[self._slot("s", i)]) for i, s in enumerate(p.sensors)},
        }

    # -- hot swap -----------------------------------------------------------------

    def remove(self, bay: str) -> None:
        i = self._bi[bay]
        d = self.drives[i]
        if d is None:
            return
        self.energy["swap_j"] -= d.c_j_per_k * float(self.x[self._slot("d", i)])
        self.drives[i] = None
        self._next_smart[i] = math.inf

    def insert(self, bay: str, drive: DriveSpec, temp_c: float | None = None) -> None:
        """Insert ``drive`` at ``temp_c`` (default: the zone's inlet temperature)."""
        i = self._bi[bay]
        if self.drives[i] is not None:
            self.remove(bay)
        zone = self.params.bays[i].zone
        t0 = self.t_inlet()[zone] if temp_c is None else float(temp_c)
        self.drives[i] = drive
        self.x[self._slot("d", i)] = t0
        self.x[self._slot("m", i)] = t0
        self.energy["swap_j"] += drive.c_j_per_k * t0
        self._schedule_smart(i, first=True)

    def _apply_events(self) -> None:
        while self._events and self._events[0].t_s <= self.ts + 1e-9:
            ev = self._events.pop(0)
            if ev.action == "remove":
                self.remove(ev.bay)
            else:
                self.insert(ev.bay, ev.drive, ev.temp_c)  # type: ignore[arg-type]

    # -- SMART ----------------------------------------------------------------------

    def _schedule_smart(self, i: int, first: bool = False) -> None:
        d = self.drives[i]
        if d is None or not d.smart or d.serial is None:
            self._next_smart[i] = math.inf
            return
        lo, hi = self.params.smart_cadence_s
        step = float(self._smart_rng[i].uniform(lo, hi))
        base = self.ts if first or not math.isfinite(self._next_smart[i]) else self._next_smart[i]
        self._next_smart[i] = base + (step * float(self._smart_rng[i].random()) if first else step)

    def _sample_smart(self) -> None:
        for i, d in enumerate(self.drives):
            if d is None or d.serial is None:
                continue
            if self._next_smart[i] <= self.ts + 1e-9:
                value = float(self.x[self._slot("m", i)]) + d.smart_offset_c
                self.smart_samples[d.serial] = {
                    "temp_c": quantise(value, SMART_QUANT_C),
                    "ts": self.ts,
                    "model": d.model,
                    "bay": self.params.bays[i].name,
                }
                while self._next_smart[i] <= self.ts + 1e-9:
                    self._schedule_smart(i)

    def observe_smart(self) -> dict[str, dict[str, Any]]:
        """``{serial: {temp_c, age_s, model}}`` as the PC agent + inbox would present it.

        Entries of removed drives stay with a growing ``age_s`` (retained MQTT).
        The truth bay is deliberately not included (there is no SES).
        """
        return {
            serial: {"temp_c": s["temp_c"], "age_s": self.ts - s["ts"], "model": s["model"]}
            for serial, s in sorted(self.smart_samples.items())
        }

    # -- simulation -------------------------------------------------------------------

    def _draw_noise(self) -> None:
        p = self.params
        ns = self._ns
        z = self._noise_rng.standard_normal(ns)
        u = self._noise_rng.random(ns)
        sig = np.array([s.noise_sigma_c for s in p.sensors]) if ns else np.zeros(0)
        drop = np.array([s.dropout_prob for s in p.sensors]) if ns else np.zeros(0)
        self._noise = z * sig
        self._dropout = u < drop

    def apply(self, pwm: Mapping[str, float]) -> None:
        """Queue a command; it acts after ``delay_ticks`` calls to :meth:`advance`."""
        clean = {}
        for ch in self.channels:
            v = pwm.get(ch, self._history[-1][ch])
            v = float(v) if v is not None and math.isfinite(float(v)) else self._history[-1][ch]
            clean[ch] = min(1.0, max(0.0, v))
        self._history.append(clean)
        keep = self.params.delay_ticks + 2
        if len(self._history) > keep:
            del self._history[:-keep]

    def advance(self) -> None:
        """Integrate one ``dt``; events at the start, SMART and noise at the end; ``ts += dt``."""
        p = self.params
        self._apply_events()
        self._update_fans(self.effective_pwm(), p.dt)
        q = self._airflow(self.ts)
        A, _, _ = self._system(q, self.ts)
        # Fastest air node time constant bounds the substep for accuracy.
        diag = -np.diag(A)[: self._nz]
        tau_air = float(1.0 / diag.max()) if diag.size and diag.max() > 0 else p.dt
        h_max = min(p.max_substep_s, tau_air)
        n = max(p.substeps, math.ceil(p.dt / h_max - 1e-9))
        h = p.dt / n
        eye = np.eye(self._n)
        for k in range(1, n + 1):
            t1 = self.ts + k * h
            A, b, _ = self._system(q, t1)
            x1 = np.linalg.solve(eye - h * A, self.x + h * b)
            amb = p.inlet.ambient(t1)
            heat = sum(self.heat_w(t1).values()) + sum(z.heat_w for z in p.zones)
            out = sum(
                (q[i] + z.leak_w_per_k) * (float(x1[i]) - amb - z.inlet_offset_c)
                for i, z in enumerate(p.zones)
            )
            self.energy["in_j"] += h * heat
            self.energy["out_j"] += h * out
            self.x = x1
        self.ts += p.dt
        self.tick += 1
        self._sample_smart()
        self._draw_noise()

    def observe(self) -> PlantObservation:
        """Quantised, noisy readings for every sensor; rpm only for outputs with a tach."""
        p = self.params
        temps: dict[str, float | None] = {}
        base = self._slot("s", 0)
        for i, s in enumerate(p.sensors):
            if self._dropout[i]:
                temps[s.name] = None
            else:
                temps[s.name] = quantise(float(self.x[base + i]) + float(self._noise[i]), s.quant_c)
        rpm_all = self.fan_rpm()
        rpm: dict[str, float | None] = {
            f.name: float(round(rpm_all[f.name][0])) for f in p.fans if f.tach
        }
        return PlantObservation(temps=temps, rpm=rpm, pwm=self.effective_pwm(), ts=self.ts)

    def step(self, pwm: Mapping[str, float]) -> PlantObservation:
        self.apply(pwm)
        self.advance()
        return self.observe()


# ---------------------------------------------------------------------------
# Closed loop
# ---------------------------------------------------------------------------

Controller = Callable[[PlantObservation, MpcConfig, MpcState], tuple[MpcCommand, MpcState]]
ObsHook = Callable[[int, PlantObservation], PlantObservation]
SmartHook = Callable[[int, dict[str, dict[str, Any]]], dict[str, dict[str, Any]]]


@dataclass
class DasRun:
    """Records plus per-tick truth series (plain lists, JSON-serialisable).

    ``series`` keys: ``ts``, ``noise_db``, ``ambient_c`` (lists) and
    ``pwm_cmd``, ``pwm_eff``, ``rpm`` (output's first fan, true), ``airflow``,
    ``t_air``, ``t_inlet``, ``t_drive``, ``margin_c``, ``occupied``,
    ``heat_w``, ``sensor_true``, ``obs`` (dicts of lists; ``None`` where a
    bay is empty or a reading is missing) and ``smart`` (a list of the
    SMART dict the loop saw per tick). Truth is sampled at observation time.
    """

    records: list[TickRecord]
    series: dict[str, Any]

    def array(self, key: str, name: str | None = None) -> np.ndarray:
        data = self.series[key] if name is None else self.series[key][name]
        return np.array([np.nan if v is None else float(v) for v in data])

    def worst_margin_c(self) -> float:
        vals = [v for seq in self.series["margin_c"].values() for v in seq if v is not None]
        return min(vals) if vals else math.inf

    def violations(self) -> int:
        return sum(
            1 for seq in self.series["margin_c"].values() for v in seq if v is not None and v < 0
        )


def run_das_closed_loop(
    plant: DasPlant,
    cfg: MpcConfig,
    controller: Controller,
    ticks: int,
    *,
    state: MpcState | None = None,
    observe_hook: ObsHook | None = None,
    smart_hook: SmartHook | None = None,
    on_tick: Callable[[TickRecord], Any] | None = None,
) -> DasRun:
    """Drive ``controller`` against the DAS truth plant for ``ticks`` ticks.

    Per tick: truth is recorded, ``obs = plant.observe()`` restricted to
    ``cfg.temps`` (a missing name reads ``None``), ``observe_hook(i, obs)``
    may replace it (lie injection), ``smart_hook(i, smart)`` may replace the
    SMART view, the controller steps, the plant is ``apply``-ed and advanced.
    Until ``PlantObservation`` carries ``inputs`` the SMART view is recorded
    in the series only.
    """
    p = plant.params
    st = MpcState.cold() if state is None else state
    records: list[TickRecord] = []
    zones, bays, sens = p.zone_names, p.bay_names, p.sensor_names
    series: dict[str, Any] = {
        "ts": [],
        "noise_db": [],
        "ambient_c": [],
        "pwm_cmd": {ch: [] for ch in cfg.channels},
        "pwm_eff": {ch: [] for ch in p.channels},
        "rpm": {ch: [] for ch in p.channels},
        "airflow": {z: [] for z in zones},
        "t_air": {z: [] for z in zones},
        "t_inlet": {z: [] for z in zones},
        "t_drive": {b: [] for b in bays},
        "margin_c": {b: [] for b in bays},
        "occupied": {b: [] for b in bays},
        "heat_w": {b: [] for b in bays},
        "sensor_true": {s: [] for s in sens},
        "obs": {s: [] for s in cfg.temps},
        "smart": [],
    }

    def push(key: str, values: Mapping[str, Any]) -> None:
        for name, seq in series[key].items():
            seq.append(values.get(name))

    for i in range(ticks):
        series["ts"].append(plant.ts)
        series["noise_db"].append(plant.noise_db())
        series["ambient_c"].append(plant.ambient())
        push("pwm_eff", plant.effective_pwm())
        push("rpm", {ch: v[0] for ch, v in plant.fan_rpm().items()})
        push("airflow", plant.airflow())
        push("t_air", plant.t_air())
        push("t_inlet", plant.t_inlet())
        push("t_drive", plant.t_drive())
        push("margin_c", plant.margins())
        push("occupied", plant.occupied())
        push("heat_w", plant.heat_w())
        push("sensor_true", plant.sensor_truth())
        raw = plant.observe()
        obs = PlantObservation(
            temps={name: raw.temps.get(name) for name in cfg.temps},
            rpm=raw.rpm,
            pwm=raw.pwm,
            ts=raw.ts,
        )
        if observe_hook is not None:
            obs = observe_hook(i, obs)
        smart = plant.observe_smart()
        if smart_hook is not None:
            smart = smart_hook(i, smart)
        series["smart"].append(smart)
        push("obs", obs.temps)
        cmd, st = controller(obs, cfg, st)
        push("pwm_cmd", cmd.pwm)
        rec = TickRecord(obs=obs, cmd=cmd, state=st)
        records.append(rec)
        if on_tick is not None:
            on_tick(rec)
        plant.apply(cmd.pwm)
        plant.advance()
    return DasRun(records=records, series=series)
