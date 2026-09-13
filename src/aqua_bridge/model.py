"""The controller contract (PROJECT.md section 3).

Everything the solver, the hardware adapter, the glue loop and the
publishers exchange is defined here. The hardware side must not import
anything from ``aqua_bridge.control``; the control side must not know USB,
Digole or MQTT. Both sides speak only these types.

Design notes
------------
* All dataclasses are frozen. ``mpc.step`` is a pure function
  ``(obs, cfg, state) -> (cmd, state)``; it returns a *new* ``MpcState``
  instead of mutating the one it was given.
* The spec writes ``MpcState.window`` as a ``deque``. A mutable deque
  inside an immutable state would break "same input -> same output"
  when a caller re-uses a state object, so ``window`` is a tuple of
  :class:`WindowSample`. The gate appends by building a new tuple and
  trimming it to ``cfg.stuck_ticks`` (the deque ``maxlen``).
* Observation dict *values* are raw sensor data: they may be ``None`` or
  ``NaN`` (the gate decides whether to trust them). Observation *structure*
  is validated at construction and rejects malformed input with
  ``TypeError`` / ``ValueError`` (section 4.5, "Malformed Observation
  construction").
* ``MpcConfig`` validates itself in ``__post_init__``: an invalid config
  cannot exist, so ``step`` never sees one. ``validate()`` is public and
  idempotent so tests can call it explicitly.
* ``to_dict`` / ``from_dict`` round-trip through plain JSON types.

DAS layout (zoned enclosure)
----------------------------
``MpcConfig`` optionally carries the zoned DAS description: ``topology``
(zones, bays, default drive class), ``sensors`` (placement role, zone, bay,
quantisation and the per-sensor Stuck parameters), ``drive_classes``,
``fans`` and ``fan_models`` (parsed and validated now, consumed by later
solvers) and ``zones`` (the zone trust policy). Every one of them defaults
to "absent" and a config without ``topology`` is *legacy mode*: one implicit
zone that contains every channel and every temperature, behaving bit for
bit as before. The sections other than ``topology`` are rejected without
it, so a half-declared layout can never silently run in legacy mode.

Rules that go beyond the plan's section 7 table, each the conservative
choice for cooling:

* ``coupled_to`` must be declared symmetrically (``z0: [z1]`` needs
  ``z1: [z0]``); the fault closure uses the declared relation only.
* ``drive_classes`` absent means the built-in ``hdd`` / ``ssd_sata`` /
  ``nvme`` classes; present means exactly the classes given, each with
  ``limit_c``, ``comfort_c`` and ``tau_d_s``. ``topology.default_class``
  absent means the strictest class (lowest ``limit_c``, then lowest
  ``limit_c - comfort_c``, then name).
* ``sensors.<name>.quant_c`` defaults to 0.0625 (DS18B20 at 12 bit, the
  common case): the wider the Stuck band, the more readily a frozen sensor
  is flagged, which faults its zone and raises cooling.
* A group (the zone-air sensors of a zone, the proximal sensors of a bay,
  the inlet sensors) needs at least one member that is not
  ``redundant: true``; ``redundant`` marks the backups of a group.
* With ``topology`` a setpoint may only sit on a zoned sensor
  (``zone_air`` / ``drive_proximal``), ``channel_temps`` must be absent and a
  channel controls the setpoint temperatures of the zones that list it.
  When ``setpoints`` is not empty every channel must control at least one.
  An empty ``setpoints`` is DAS limit regulation
  (:attr:`MpcConfig.regulates_drive_limits`): the ``pi`` solver regulates
  the margin deficit of the drives (``aqua_bridge.control.solver_pi``); the
  legacy ``mpc`` solver cannot compute a demand without a setpoint, raises,
  and every zone it drives stays in fallback (high cooling), visible as
  ``solver_error``, until the DAS MPC milestone.
* ``topology.bays.<bay>.limit_c`` can only tighten the bay's class limit
  (:meth:`MpcConfig.bay_limit` is the minimum of both).
* A ``topology.bays.<bay>.serial`` may be declared on one bay only.
* ``noise`` (exponent, noise weight, band hysteresis) is DAS-only like the
  other sections and defaults to :data:`NOISE_DEFAULTS` with ``topology``.
* ``estimator`` (the per-zone Kalman filter, occupancy, SMART calibration and
  association of ``aqua_bridge.control.estimator``) is DAS-only as well and
  defaults to :data:`ESTIMATOR_DEFAULTS` with ``topology``.
* The ``model_*`` keys configure the zoned thermal model's online identification
  (``aqua_bridge.control.thermal``). They are flat keys like the plan's table.
  ``model_shadow: true`` (learn and predict without acting) and
  ``model_use_rpm: true`` need ``topology``; the numeric keys are validated
  always and inert in legacy mode, and ``model_window_s >= 2 * dt`` is checked
  only with ``model_shadow`` so a default never invalidates a legacy config with
  a long ``dt``.
"""

from __future__ import annotations

import dataclasses
import json
import math
import numbers
import re
from collections.abc import Iterable, Mapping
from dataclasses import dataclass, field
from enum import Enum, StrEnum
from typing import Any

__all__ = [
    "BUILTIN_DRIVE_CLASSES",
    "DAS_SECTIONS",
    "FAULT_COUPLINGS",
    "IMPLICIT_ZONE",
    "ROLE_STUCK_S",
    "SENSOR_ROLES",
    "STUCK_WINDOW_SAMPLES",
    "TRUST_RULES",
    "BaySpec",
    "ConfigError",
    "DriveClass",
    "ESTIMATOR_DEFAULTS",
    "EstimatorSpec",
    "FanModel",
    "FanSpec",
    "FaultReason",
    "NOISE_DEFAULTS",
    "Mode",
    "MpcCommand",
    "MpcConfig",
    "MpcState",
    "NoiseSpec",
    "PlantObservation",
    "SensorSpec",
    "SolverKind",
    "StuckParams",
    "Topology",
    "WindowSample",
    "ZoneFault",
    "ZoneLayout",
    "ZonePolicy",
    "ZoneSpec",
    "is_finite_number",
]


# ---------------------------------------------------------------------------
# Enums and errors
# ---------------------------------------------------------------------------


class Mode(StrEnum):
    """Command mode.

    ``fallback`` while every zone is in fault (legacy mode: the one implicit
    zone, i.e. the whole time a fault cause is active); ``degraded`` while
    some but not all zones are in fault (only possible with a
    ``topology``); otherwise ``saturated`` or ``auto``.
    """

    AUTO = "auto"
    SATURATED = "saturated"
    FALLBACK = "fallback"
    DEGRADED = "degraded"


class FaultReason(StrEnum):
    """Why ``step`` cannot emit a trusted ``auto`` command."""

    SENSOR_GATE = "sensor_gate"
    SOLVER = "solver"


class SolverKind(StrEnum):
    """Which solver ``mpc.step`` dispatches to. PI is the first cut."""

    PI = "pi"
    MPC = "mpc"


class ConfigError(ValueError):
    """Raised for every rejected configuration (section 4.3, "Config / solver")."""


# ---------------------------------------------------------------------------
# Small validation helpers (shared by model, config, intents, tests)
# ---------------------------------------------------------------------------


def is_finite_number(value: object) -> bool:
    """True for a real number (not bool) that is neither NaN nor infinite."""
    if isinstance(value, bool) or not isinstance(value, numbers.Real):
        return False
    return math.isfinite(float(value))


def _is_real(value: object) -> bool:
    return isinstance(value, numbers.Real) and not isinstance(value, bool)


def _channel_values(name: str, value: object) -> dict[str, float | None]:
    """Validate an observation dict: str keys, numeric-or-None values.

    Values are copied into a fresh ``dict`` and normalised to ``float``
    (NaN and inf survive: they are *raw* data, the gate handles them).
    """
    if not isinstance(value, Mapping):
        raise TypeError(f"{name} must be a mapping, got {type(value).__name__}")
    out: dict[str, float | None] = {}
    for key, raw in value.items():
        if not isinstance(key, str):
            raise TypeError(f"{name} key must be str, got {type(key).__name__}: {key!r}")
        if raw is None:
            out[key] = None
        elif _is_real(raw):
            out[key] = float(raw)
        else:
            raise TypeError(
                f"{name}[{key!r}] must be a number or None, got {type(raw).__name__}: {raw!r}"
            )
    return out


def _float_map(name: str, value: object) -> dict[str, float]:
    """Validate a str -> float mapping where every value is finite."""
    if not isinstance(value, Mapping):
        raise TypeError(f"{name} must be a mapping, got {type(value).__name__}")
    out: dict[str, float] = {}
    for key, raw in value.items():
        if not isinstance(key, str):
            raise TypeError(f"{name} key must be str, got {type(key).__name__}: {key!r}")
        if not _is_real(raw):
            raise TypeError(f"{name}[{key!r}] must be a number, got {type(raw).__name__}: {raw!r}")
        out[key] = float(raw)
    return out


def _opt_float_map(name: str, value: object) -> dict[str, float | None] | None:
    return None if value is None else _channel_values(name, value)


# ---------------------------------------------------------------------------
# PlantObservation
# ---------------------------------------------------------------------------


def _json_inputs(value: object) -> dict[str, Any]:
    """Validate ``PlantObservation.inputs``: a str-keyed mapping of finite JSON.

    Returns an independent deep copy (a JSON round trip), so the frozen
    observation cannot be changed through the caller's dict.
    """
    if not isinstance(value, Mapping):
        raise TypeError(f"inputs must be a mapping, got {type(value).__name__}")
    for key in value:
        if not isinstance(key, str):
            raise TypeError(f"inputs key must be str, got {type(key).__name__}: {key!r}")
    try:
        text = json.dumps(dict(value), allow_nan=False)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"inputs must be finite JSON: {exc}") from exc
    return json.loads(text)


@dataclass(frozen=True)
class PlantObservation:
    """One raw sample from the plant.

    ``temps``/``rpm``/``pwm`` use *logical* names (``coolant``, ``radiator``,
    ...). Values may be ``None`` or NaN; structure must be sound.
    ``ts`` is monotonic cycle seconds (not wall clock) and must be finite.

    ``inputs`` carries exogenous, non-gated data by source name (for example
    ``inputs["smart"]`` from the PC's SMART agent in a later milestone). It
    must be a str-keyed mapping of finite JSON; it never enters the sensor
    gate and never faults anything. It defaults to empty and is omitted from
    :meth:`to_dict` while empty, so an observation without inputs serialises
    exactly as before.
    """

    temps: dict[str, float | None]
    rpm: dict[str, float | None]
    pwm: dict[str, float | None]
    ts: float
    inputs: dict[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        object.__setattr__(self, "temps", _channel_values("temps", self.temps))
        object.__setattr__(self, "rpm", _channel_values("rpm", self.rpm))
        object.__setattr__(self, "pwm", _channel_values("pwm", self.pwm))
        if not _is_real(self.ts):
            raise TypeError(f"ts must be a number, got {type(self.ts).__name__}: {self.ts!r}")
        ts = float(self.ts)
        if not math.isfinite(ts):
            raise ValueError(f"ts must be finite, got {ts!r}")
        object.__setattr__(self, "ts", ts)
        object.__setattr__(self, "inputs", _json_inputs(self.inputs))

    def to_dict(self) -> dict[str, Any]:
        out: dict[str, Any] = {
            "temps": dict(self.temps),
            "rpm": dict(self.rpm),
            "pwm": dict(self.pwm),
            "ts": self.ts,
        }
        if self.inputs:
            out["inputs"] = json.loads(json.dumps(self.inputs))
        return out

    @classmethod
    def from_dict(cls, data: Mapping[str, Any]) -> PlantObservation:
        if not isinstance(data, Mapping):
            raise TypeError(
                f"PlantObservation.from_dict expects a mapping, got {type(data).__name__}"
            )
        return cls(
            temps=data.get("temps", {}),
            rpm=data.get("rpm", {}),
            pwm=data.get("pwm", {}),
            ts=data["ts"],
            inputs=data.get("inputs", {}),
        )


# ---------------------------------------------------------------------------
# MpcCommand
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class MpcCommand:
    """What the controller wants the fans to do.

    ``pwm`` is keyed by logical channel, values in ``[0, 1]``. Construction
    checks structure only (str keys, numeric values); value invariants
    (finite, inside ``[pwm_min, pwm_max]``, rate limit) are the job of
    ``mpc.step`` and are asserted by ``tests/invariants.py``.
    ``diagnostics`` must stay JSON-serialisable (str keys, plain values).
    """

    pwm: dict[str, float]
    mode: Mode
    diagnostics: dict[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        object.__setattr__(self, "pwm", _float_map("pwm", self.pwm))
        try:
            mode = Mode(self.mode)
        except ValueError as exc:
            raise ValueError(
                f"mode must be one of {[m.value for m in Mode]}, got {self.mode!r}"
            ) from exc
        object.__setattr__(self, "mode", mode)
        if not isinstance(self.diagnostics, Mapping):
            raise TypeError(f"diagnostics must be a mapping, got {type(self.diagnostics).__name__}")
        for key in self.diagnostics:
            if not isinstance(key, str):
                raise TypeError(f"diagnostics key must be str, got {type(key).__name__}: {key!r}")
        object.__setattr__(self, "diagnostics", dict(self.diagnostics))

    def to_dict(self) -> dict[str, Any]:
        return {
            "pwm": dict(self.pwm),
            "mode": self.mode.value,
            "diagnostics": dict(self.diagnostics),
        }

    @classmethod
    def from_dict(cls, data: Mapping[str, Any]) -> MpcCommand:
        if not isinstance(data, Mapping):
            raise TypeError(f"MpcCommand.from_dict expects a mapping, got {type(data).__name__}")
        return cls(pwm=data["pwm"], mode=data["mode"], diagnostics=data.get("diagnostics", {}))


# ---------------------------------------------------------------------------
# MpcState
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class WindowSample:
    """One entry of ``MpcState.window``: this tick's raw temps and commanded PWM.

    Stored trusted or not (section 3, gate rule 5). ``raw_temps`` values may
    be ``None`` (missing/garbage sample); the gate must not store NaN
    (``assert_state_finite`` rejects it), use ``None`` instead.
    """

    raw_temps: dict[str, float | None]
    cmd_pwm: dict[str, float]

    def __post_init__(self) -> None:
        object.__setattr__(self, "raw_temps", _channel_values("raw_temps", self.raw_temps))
        object.__setattr__(self, "cmd_pwm", _float_map("cmd_pwm", self.cmd_pwm))

    def to_dict(self) -> dict[str, Any]:
        return {"raw_temps": dict(self.raw_temps), "cmd_pwm": dict(self.cmd_pwm)}

    @classmethod
    def from_dict(cls, data: Mapping[str, Any]) -> WindowSample:
        return cls(raw_temps=data["raw_temps"], cmd_pwm=data["cmd_pwm"])


@dataclass(frozen=True)
class ZoneFault:
    """Fault bookkeeping of one zone (``MpcState.zone_faults``).

    * ``since_ts`` -- first ``ts`` at which a fault cause of this zone became active,
      ``None`` while the zone is not in fault
    * ``reason``   -- which cause; set exactly when ``since_ts`` is set
    * ``streak``   -- consecutive ticks on which the zone was trusted
    * ``ticks``    -- consecutive ticks the zone has been in fault (0 when not)
    """

    since_ts: float | None = None
    reason: FaultReason | None = None
    streak: int = 0
    ticks: int = 0

    def __post_init__(self) -> None:
        if self.since_ts is not None:
            if not _is_real(self.since_ts) or not math.isfinite(float(self.since_ts)):
                raise TypeError("ZoneFault.since_ts must be a finite number or None")
            object.__setattr__(self, "since_ts", float(self.since_ts))
        if self.reason is not None:
            object.__setattr__(self, "reason", FaultReason(self.reason))
        if (self.since_ts is None) != (self.reason is None):
            raise ValueError("ZoneFault.since_ts and ZoneFault.reason must be set together")
        for name in ("streak", "ticks"):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, int):
                raise TypeError(f"ZoneFault.{name} must be an int")
            if value < 0:
                raise ValueError(f"ZoneFault.{name} must be >= 0")

    @property
    def in_fault(self) -> bool:
        return self.since_ts is not None

    def to_dict(self) -> dict[str, Any]:
        return {
            "since_ts": self.since_ts,
            "reason": None if self.reason is None else self.reason.value,
            "streak": self.streak,
            "ticks": self.ticks,
        }

    @classmethod
    def from_dict(cls, data: Mapping[str, Any]) -> ZoneFault:
        if not isinstance(data, Mapping):
            raise TypeError(f"ZoneFault.from_dict expects a mapping, got {type(data).__name__}")
        return cls(
            since_ts=data.get("since_ts"),
            reason=data.get("reason"),
            streak=data.get("streak", 0),
            ticks=data.get("ticks", 0),
        )


@dataclass(frozen=True)
class MpcState:
    """Everything ``step`` remembers between ticks. Immutable; ``step`` returns a new one.

    Fields (section 3):

    * ``last_cmd``       -- previous command; ``None`` on a cold state
    * ``last_good_obs``  -- last *confirmed* trusted observation, replaced as a whole
    * ``last_raw_temps`` -- previous raw temps, stored even when untrusted
    * ``window``         -- last ``cfg.stuck_ticks`` samples (newest last); feeds median3 and stuck
    * ``fault_since_ts`` -- first ``ts`` at which *any* fallback cause became active
    * ``fault_reason``   -- which cause (one timer for both, never two)
    * ``trusted_streak`` -- consecutive trusted observations; any untrusted tick resets to 0
    * ``integrator``     -- PI / MPC integral term per channel (bumpless transfer rules apply)
    * ``solver_memory``  -- generic slot for any other solver memory; must stay JSON-serialisable
    * ``zone_faults``    -- per-zone fault bookkeeping (:class:`ZoneFault`), one entry per
      ``topology`` zone; empty in legacy mode. With zones, ``fault_since_ts`` /
      ``fault_reason`` / ``trusted_streak`` are the aggregates: the earliest zone
      fault and its reason, and the smallest zone streak. ``to_dict`` omits the key
      while it is empty, so a legacy state serialises exactly as before.
    """

    last_cmd: MpcCommand | None = None
    last_good_obs: PlantObservation | None = None
    last_raw_temps: dict[str, float | None] | None = None
    window: tuple[WindowSample, ...] = ()
    fault_since_ts: float | None = None
    fault_reason: FaultReason | None = None
    trusted_streak: int = 0
    integrator: dict[str, float] = field(default_factory=dict)
    solver_memory: dict[str, Any] = field(default_factory=dict)
    zone_faults: dict[str, ZoneFault] = field(default_factory=dict)

    def __post_init__(self) -> None:
        if self.last_cmd is not None and not isinstance(self.last_cmd, MpcCommand):
            raise TypeError("last_cmd must be an MpcCommand or None")
        if self.last_good_obs is not None and not isinstance(self.last_good_obs, PlantObservation):
            raise TypeError("last_good_obs must be a PlantObservation or None")
        object.__setattr__(
            self, "last_raw_temps", _opt_float_map("last_raw_temps", self.last_raw_temps)
        )
        window = tuple(self.window)
        for sample in window:
            if not isinstance(sample, WindowSample):
                raise TypeError("window entries must be WindowSample")
        object.__setattr__(self, "window", window)
        if self.fault_since_ts is not None:
            if not _is_real(self.fault_since_ts):
                raise TypeError("fault_since_ts must be a number or None")
            object.__setattr__(self, "fault_since_ts", float(self.fault_since_ts))
        if self.fault_reason is not None:
            object.__setattr__(self, "fault_reason", FaultReason(self.fault_reason))
        if isinstance(self.trusted_streak, bool) or not isinstance(self.trusted_streak, int):
            raise TypeError("trusted_streak must be an int")
        if self.trusted_streak < 0:
            raise ValueError("trusted_streak must be >= 0")
        object.__setattr__(self, "integrator", _float_map("integrator", self.integrator))
        if not isinstance(self.solver_memory, Mapping):
            raise TypeError("solver_memory must be a mapping")
        for key in self.solver_memory:
            if not isinstance(key, str):
                raise TypeError("solver_memory keys must be str")
        object.__setattr__(self, "solver_memory", dict(self.solver_memory))
        if not isinstance(self.zone_faults, Mapping):
            raise TypeError("zone_faults must be a mapping zone -> ZoneFault")
        zone_faults: dict[str, ZoneFault] = {}
        for key, value in self.zone_faults.items():
            if not isinstance(key, str) or not key:
                raise TypeError("zone_faults keys must be non-empty str")
            if not isinstance(value, ZoneFault):
                raise TypeError("zone_faults values must be ZoneFault")
            zone_faults[key] = value
        object.__setattr__(self, "zone_faults", zone_faults)

    @classmethod
    def cold(cls) -> MpcState:
        """State before the first tick: nothing known, no fault active."""
        return cls()

    @property
    def in_fault(self) -> bool:
        return self.fault_since_ts is not None

    def to_dict(self) -> dict[str, Any]:
        out: dict[str, Any] = {
            "last_cmd": None if self.last_cmd is None else self.last_cmd.to_dict(),
            "last_good_obs": None if self.last_good_obs is None else self.last_good_obs.to_dict(),
            "last_raw_temps": None if self.last_raw_temps is None else dict(self.last_raw_temps),
            "window": [w.to_dict() for w in self.window],
            "fault_since_ts": self.fault_since_ts,
            "fault_reason": None if self.fault_reason is None else self.fault_reason.value,
            "trusted_streak": self.trusted_streak,
            "integrator": dict(self.integrator),
            "solver_memory": dict(self.solver_memory),
        }
        if self.zone_faults:
            out["zone_faults"] = {z: f.to_dict() for z, f in self.zone_faults.items()}
        return out

    @classmethod
    def from_dict(cls, data: Mapping[str, Any]) -> MpcState:
        if not isinstance(data, Mapping):
            raise TypeError(f"MpcState.from_dict expects a mapping, got {type(data).__name__}")
        last_cmd = data.get("last_cmd")
        last_good_obs = data.get("last_good_obs")
        return cls(
            last_cmd=None if last_cmd is None else MpcCommand.from_dict(last_cmd),
            last_good_obs=(
                None if last_good_obs is None else PlantObservation.from_dict(last_good_obs)
            ),
            last_raw_temps=data.get("last_raw_temps"),
            window=tuple(WindowSample.from_dict(w) for w in data.get("window", ())),
            fault_since_ts=data.get("fault_since_ts"),
            fault_reason=data.get("fault_reason"),
            trusted_streak=data.get("trusted_streak", 0),
            integrator=data.get("integrator", {}),
            solver_memory=data.get("solver_memory", {}),
            zone_faults={
                str(z): ZoneFault.from_dict(f) for z, f in (data.get("zone_faults") or {}).items()
            },
        )


# ---------------------------------------------------------------------------
# MpcConfig
# ---------------------------------------------------------------------------


def _cfg_num(name: str, value: object) -> float:
    if not _is_real(value):
        raise ConfigError(f"mpc.{name} must be a number, got {type(value).__name__}: {value!r}")
    out = float(value)
    if not math.isfinite(out):
        raise ConfigError(f"mpc.{name} must be finite, got {out!r}")
    return out


def _cfg_int(name: str, value: object) -> int:
    if isinstance(value, bool) or not isinstance(value, numbers.Integral):
        raise ConfigError(f"mpc.{name} must be an integer, got {type(value).__name__}: {value!r}")
    return int(value)


def _cfg_names(name: str, value: object) -> tuple[str, ...]:
    if isinstance(value, str | bytes) or not isinstance(value, Iterable):
        raise ConfigError(f"mpc.{name} must be a list of names, got {type(value).__name__}")
    names = tuple(value)
    for item in names:
        if not isinstance(item, str) or not item:
            raise ConfigError(f"mpc.{name} entries must be non-empty strings, got {item!r}")
    if len(set(names)) != len(names):
        raise ConfigError(f"mpc.{name} has duplicate entries: {names}")
    return names


def _cfg_float_map(name: str, value: object) -> dict[str, float]:
    if not isinstance(value, Mapping):
        raise ConfigError(f"mpc.{name} must be a mapping, got {type(value).__name__}")
    out: dict[str, float] = {}
    for key, raw in value.items():
        if not isinstance(key, str) or not key:
            raise ConfigError(f"mpc.{name} keys must be non-empty strings, got {key!r}")
        out[key] = _cfg_num(f"{name}[{key!r}]", raw)
    return out


# ---------------------------------------------------------------------------
# DAS layout sections (topology, sensors, drive classes, fans, zone policy)
# ---------------------------------------------------------------------------

#: ``MpcConfig`` fields that describe the zoned DAS; all absent = legacy mode.
DAS_SECTIONS: tuple[str, ...] = (
    "topology",
    "sensors",
    "drive_classes",
    "fans",
    "fan_models",
    "zones",
    "noise",
    "estimator",
)

#: Name of the single zone that stands for the whole config in legacy mode.
IMPLICIT_ZONE = "all"

#: Placement roles of ``sensors.<name>.role``.
SENSOR_ROLES: tuple[str, ...] = ("inlet", "zone_air", "drive_proximal", "exhaust")

#: Default Stuck window per role, seconds (plan section 0.2).
ROLE_STUCK_S: dict[str, float] = {
    "drive_proximal": 1800.0,
    "zone_air": 180.0,
    "inlet": 600.0,
    "exhaust": 600.0,
}

#: Default ``quant_c`` (DS18B20 at 12 bit) and the default Stuck band factor on it.
DEFAULT_QUANT_C = 0.0625
STUCK_EPS_QUANT_FACTOR = 1.5

#: Automatic ``stuck_decimate`` keeps about this many stored samples per window.
STUCK_WINDOW_SAMPLES = 60

#: ``zones.trust_rule`` and ``zones.fault_coupling`` values.
TRUST_RULES: tuple[str, ...] = ("strict", "sigma")
FAULT_COUPLINGS: tuple[str, ...] = ("declared", "none")

_MIX = "mix"


def _section_keys(
    path: str, data: object, *, required: Iterable[str] = (), optional: Iterable[str] = ()
) -> dict[str, Any]:
    """A nested config mapping with only known keys and every required key present."""
    if not isinstance(data, Mapping):
        raise ConfigError(f"mpc.{path} must be a mapping, got {type(data).__name__}")
    required = tuple(required)
    known = set(required) | set(optional)
    for key in data:
        if not isinstance(key, str):
            raise ConfigError(f"mpc.{path} keys must be strings, got {key!r}")
    unknown = sorted(set(data) - known)
    if unknown:
        raise ConfigError(f"unknown mpc.{path} keys: {unknown}")
    missing = [key for key in required if key not in data]
    if missing:
        raise ConfigError(f"missing mpc.{path} keys: {missing}")
    return dict(data)


def _named_map(path: str, data: object) -> dict[str, Any]:
    """``name -> entry`` mapping with non-empty string names (``None`` -> empty)."""
    if data is None:
        return {}
    if not isinstance(data, Mapping):
        raise ConfigError(f"mpc.{path} must be a mapping name -> entry, got {type(data).__name__}")
    for key in data:
        if not isinstance(key, str) or not key:
            raise ConfigError(f"mpc.{path} names must be non-empty strings, got {key!r}")
    return dict(data)


def _req_str(path: str, value: object) -> str:
    if not isinstance(value, str) or not value:
        raise ConfigError(f"mpc.{path} must be a non-empty string, got {value!r}")
    return value


def _opt_str(path: str, value: object) -> str | None:
    return None if value is None else _req_str(path, value)


def _cfg_bool(path: str, value: object) -> bool:
    if not isinstance(value, bool):
        raise ConfigError(f"mpc.{path} must be a bool (true/false), got {value!r}")
    return value


def _choice(path: str, value: object, choices: tuple[str, ...]) -> str:
    if not isinstance(value, str) or value not in choices:
        raise ConfigError(f"mpc.{path} must be one of {list(choices)}, got {value!r}")
    return value


def _cfg_list(path: str, value: object) -> tuple[Any, ...]:
    if value is None:
        return ()
    if isinstance(value, str | bytes | Mapping) or not isinstance(value, Iterable):
        raise ConfigError(f"mpc.{path} must be a list, got {type(value).__name__}")
    return tuple(value)


@dataclass(frozen=True)
class ZoneSpec:
    """``topology.zones.<zone>``: the channels that move air through it and its neighbours.

    ``channels`` lists every channel whose fans move air through the zone (a
    channel that touches several zones is listed in each). ``coupled_to``
    names the zones it exchanges air with (declared symmetrically); ``inlet``
    is the sensor of role ``inlet`` that feeds it, or ``mix`` (every inlet
    sensor, the default).
    """

    channels: tuple[str, ...]
    coupled_to: tuple[str, ...] = ()
    inlet: str = _MIX

    @classmethod
    def coerce(cls, path: str, data: object) -> ZoneSpec:
        if isinstance(data, ZoneSpec):
            return data
        raw = _section_keys(path, data, required=("channels",), optional=("coupled_to", "inlet"))
        return cls(
            channels=_cfg_names(f"{path}.channels", raw["channels"]),
            coupled_to=_cfg_names(f"{path}.coupled_to", raw.get("coupled_to") or ()),
            inlet=_req_str(f"{path}.inlet", raw.get("inlet", _MIX)),
        )

    def to_dict(self) -> dict[str, Any]:
        return {
            "channels": list(self.channels),
            "coupled_to": list(self.coupled_to),
            "inlet": self.inlet,
        }


@dataclass(frozen=True)
class BaySpec:
    """``topology.bays.<bay>``: a drive slot. The YAML key ``class`` is ``drive_class`` here.

    ``occupied`` is ``true``, ``false`` or ``"auto"`` (unknown until an estimator
    decides; an unknown bay is treated like an occupied one: conservative).
    ``limit_c`` optionally tightens the class limit for this one bay: the bay's
    limit is ``min(class limit_c, limit_c)`` (:meth:`MpcConfig.bay_limit`), so a
    bay entry can never allow a drive hotter than its class.
    """

    zone: str
    drive_class: str | None = None
    occupied: bool | str = "auto"
    serial: str | None = None
    limit_c: float | None = None

    @classmethod
    def coerce(cls, path: str, data: object) -> BaySpec:
        if isinstance(data, BaySpec):
            return data
        raw = _section_keys(
            path, data, required=("zone",), optional=("class", "occupied", "serial", "limit_c")
        )
        occupied = raw.get("occupied", "auto")
        if not (isinstance(occupied, bool) or occupied == "auto"):
            raise ConfigError(f"mpc.{path}.occupied must be true, false or auto, got {occupied!r}")
        return cls(
            zone=_req_str(f"{path}.zone", raw["zone"]),
            drive_class=_opt_str(f"{path}.class", raw.get("class")),
            occupied=occupied,
            serial=_opt_str(f"{path}.serial", raw.get("serial")),
            limit_c=(
                None if raw.get("limit_c") is None else _cfg_num(f"{path}.limit_c", raw["limit_c"])
            ),
        )

    @property
    def constrained(self) -> bool:
        """Occupied or unknown: the drive carries constraints, its proximal sensors are required."""
        return self.occupied is not False

    def to_dict(self) -> dict[str, Any]:
        return {
            "zone": self.zone,
            "class": self.drive_class,
            "occupied": self.occupied,
            "serial": self.serial,
            "limit_c": self.limit_c,
        }


@dataclass(frozen=True)
class Topology:
    """``topology``: zones, bays and the class of a bay that declares none."""

    zones: dict[str, ZoneSpec]
    bays: dict[str, BaySpec] = field(default_factory=dict)
    default_class: str | None = None

    @classmethod
    def coerce(cls, data: object) -> Topology:
        if isinstance(data, Topology):
            return data
        raw = _section_keys(
            "topology", data, required=("zones",), optional=("bays", "default_class")
        )
        zones = {
            name: ZoneSpec.coerce(f"topology.zones.{name}", entry)
            for name, entry in _named_map("topology.zones", raw["zones"]).items()
        }
        bays = {
            name: BaySpec.coerce(f"topology.bays.{name}", entry)
            for name, entry in _named_map("topology.bays", raw.get("bays")).items()
        }
        return cls(
            zones=zones,
            bays=bays,
            default_class=_opt_str("topology.default_class", raw.get("default_class")),
        )

    def to_dict(self) -> dict[str, Any]:
        return {
            "zones": {name: z.to_dict() for name, z in self.zones.items()},
            "bays": {name: b.to_dict() for name, b in self.bays.items()},
            "default_class": self.default_class,
        }


@dataclass(frozen=True)
class DriveClass:
    """``drive_classes.<class>``: absolute limit, comfort band, thermal time constant.

    ``models`` are regular expressions searched in the SMART model string of a
    bay's associated drive (``aqua_bridge.control.estimator``: a match sets the
    bay's class); they are compiled here so a typo is a config error.
    """

    limit_c: float
    comfort_c: float
    tau_d_s: float
    models: tuple[str, ...] = ()

    @classmethod
    def coerce(cls, path: str, data: object) -> DriveClass:
        if isinstance(data, DriveClass):
            return data
        raw = _section_keys(
            path, data, required=("limit_c", "comfort_c", "tau_d_s"), optional=("models",)
        )
        models = _cfg_list(f"{path}.models", raw.get("models"))
        for pattern in models:
            if not isinstance(pattern, str):
                raise ConfigError(f"mpc.{path}.models entries must be strings, got {pattern!r}")
            try:
                re.compile(pattern)
            except re.error as exc:
                raise ConfigError(f"mpc.{path}.models: invalid regex {pattern!r}: {exc}") from exc
        return cls(
            limit_c=_cfg_num(f"{path}.limit_c", raw["limit_c"]),
            comfort_c=_cfg_num(f"{path}.comfort_c", raw["comfort_c"]),
            tau_d_s=_cfg_num(f"{path}.tau_d_s", raw["tau_d_s"]),
            models=models,
        )

    def to_dict(self) -> dict[str, Any]:
        return {
            "limit_c": self.limit_c,
            "comfort_c": self.comfort_c,
            "tau_d_s": self.tau_d_s,
            "models": list(self.models),
        }


#: Built-in drive classes (owner decision: HDD 50 / SATA SSD 65 / NVMe 70 degC,
#: comfort 5 / 10 / 10 degC), used when ``drive_classes`` is absent.
BUILTIN_DRIVE_CLASSES: dict[str, DriveClass] = {
    "hdd": DriveClass(limit_c=50.0, comfort_c=5.0, tau_d_s=720.0),
    "ssd_sata": DriveClass(limit_c=65.0, comfort_c=10.0, tau_d_s=200.0),
    "nvme": DriveClass(limit_c=70.0, comfort_c=10.0, tau_d_s=120.0),
}


@dataclass(frozen=True)
class SensorSpec:
    """``sensors.<name>``: where a temperature sensor sits and how its Stuck rule is sized.

    ``stuck_s`` defaults per role (:data:`ROLE_STUCK_S`), ``stuck_eps_c`` to
    ``1.5 * quant_c``; both are resolved at construction so ``to_dict`` shows
    the values in force. ``stuck_decimate`` ``None`` means automatic (see
    :meth:`MpcConfig.stuck_params`). ``tau_s`` (default 15 s) is the lag of the
    bay's sensor node in the estimator; a bay's first non-redundant proximal
    sensor sets it.
    """

    role: str
    zone: str | None = None
    bay: str | None = None
    quant_c: float = DEFAULT_QUANT_C
    stuck_s: float | None = None
    stuck_eps_c: float | None = None
    stuck_decimate: int | None = None
    redundant: bool = False
    tau_s: float | None = None

    def __post_init__(self) -> None:
        if self.role in ROLE_STUCK_S and self.stuck_s is None:
            object.__setattr__(self, "stuck_s", ROLE_STUCK_S[self.role])
        if self.stuck_eps_c is None and _is_real(self.quant_c):
            object.__setattr__(self, "stuck_eps_c", STUCK_EPS_QUANT_FACTOR * float(self.quant_c))

    @classmethod
    def coerce(cls, path: str, data: object) -> SensorSpec:
        if isinstance(data, SensorSpec):
            return data
        raw = _section_keys(
            path,
            data,
            required=("role",),
            optional=(
                "zone",
                "bay",
                "quant_c",
                "stuck_s",
                "stuck_eps_c",
                "stuck_decimate",
                "redundant",
                "tau_s",
            ),
        )
        stuck_s = raw.get("stuck_s")
        stuck_eps = raw.get("stuck_eps_c")
        decimate = raw.get("stuck_decimate")
        tau = raw.get("tau_s")
        return cls(
            role=_choice(f"{path}.role", raw["role"], SENSOR_ROLES),
            zone=_opt_str(f"{path}.zone", raw.get("zone")),
            bay=_opt_str(f"{path}.bay", raw.get("bay")),
            quant_c=_cfg_num(f"{path}.quant_c", raw.get("quant_c", DEFAULT_QUANT_C)),
            stuck_s=None if stuck_s is None else _cfg_num(f"{path}.stuck_s", stuck_s),
            stuck_eps_c=None if stuck_eps is None else _cfg_num(f"{path}.stuck_eps_c", stuck_eps),
            stuck_decimate=(
                None if decimate is None else _cfg_int(f"{path}.stuck_decimate", decimate)
            ),
            redundant=_cfg_bool(f"{path}.redundant", raw.get("redundant", False)),
            tau_s=None if tau is None else _cfg_num(f"{path}.tau_s", tau),
        )

    def to_dict(self) -> dict[str, Any]:
        return {
            "role": self.role,
            "zone": self.zone,
            "bay": self.bay,
            "quant_c": self.quant_c,
            "stuck_s": self.stuck_s,
            "stuck_eps_c": self.stuck_eps_c,
            "stuck_decimate": self.stuck_decimate,
            "redundant": self.redundant,
            "tau_s": self.tau_s,
        }


@dataclass(frozen=True)
class FanSpec:
    """``fans.<channel>``: what hangs on one PWM output (1-2 fans on one tach is typical).

    ``group`` names the air path the channel shares with other channels
    (``None``: its own). ``forbidden_pwm`` is parsed and validated only; the
    solver that honours it is a later milestone.
    """

    model: str
    count: int = 1
    group: str | None = None
    noise_weight: float = 1.0
    forbidden_pwm: tuple[tuple[float, float], ...] = ()

    @classmethod
    def coerce(cls, path: str, data: object) -> FanSpec:
        if isinstance(data, FanSpec):
            return data
        raw = _section_keys(
            path,
            data,
            required=("model",),
            optional=("count", "group", "noise_weight", "forbidden_pwm"),
        )
        bands: list[tuple[float, float]] = []
        for band in _cfg_list(f"{path}.forbidden_pwm", raw.get("forbidden_pwm")):
            pair = _cfg_list(f"{path}.forbidden_pwm entry", band)
            if len(pair) != 2:
                raise ConfigError(
                    f"mpc.{path}.forbidden_pwm entries must be [lo, hi], got {band!r}"
                )
            bands.append(
                (
                    _cfg_num(f"{path}.forbidden_pwm lo", pair[0]),
                    _cfg_num(f"{path}.forbidden_pwm hi", pair[1]),
                )
            )
        return cls(
            model=_req_str(f"{path}.model", raw["model"]),
            count=_cfg_int(f"{path}.count", raw.get("count", 1)),
            group=_opt_str(f"{path}.group", raw.get("group")),
            noise_weight=_cfg_num(f"{path}.noise_weight", raw.get("noise_weight", 1.0)),
            forbidden_pwm=tuple(bands),
        )

    def to_dict(self) -> dict[str, Any]:
        return {
            "model": self.model,
            "count": self.count,
            "group": self.group,
            "noise_weight": self.noise_weight,
            "forbidden_pwm": [list(b) for b in self.forbidden_pwm],
        }


@dataclass(frozen=True)
class FanModel:
    """``fan_models.<model>``: RPM curve and noise at full speed of one fan type.

    ``noise_db_at_max`` defaults to 0 dB: the noise figure is an index, and
    equal defaults weigh every model the same.
    """

    rpm_max: float
    deadband: float = 0.1
    exponent: float = 1.0
    noise_db_at_max: float = 0.0

    @classmethod
    def coerce(cls, path: str, data: object) -> FanModel:
        if isinstance(data, FanModel):
            return data
        raw = _section_keys(
            path,
            data,
            required=("rpm_max",),
            optional=("deadband", "exponent", "noise_db_at_max"),
        )
        return cls(
            rpm_max=_cfg_num(f"{path}.rpm_max", raw["rpm_max"]),
            deadband=_cfg_num(f"{path}.deadband", raw.get("deadband", 0.1)),
            exponent=_cfg_num(f"{path}.exponent", raw.get("exponent", 1.0)),
            noise_db_at_max=_cfg_num(f"{path}.noise_db_at_max", raw.get("noise_db_at_max", 0.0)),
        )

    def to_dict(self) -> dict[str, Any]:
        return {
            "rpm_max": self.rpm_max,
            "deadband": self.deadband,
            "exponent": self.exponent,
            "noise_db_at_max": self.noise_db_at_max,
        }


@dataclass(frozen=True)
class ZonePolicy:
    """``zones``: how a zone's trust is decided and how far a zone fault reaches.

    * ``trust_rule``     -- ``strict`` (every required sensor group has a trusted
      member) or ``sigma`` (estimator uncertainty; the rule itself is a later
      milestone and until then ``strict`` applies, see ``aqua_bridge.control.zones``)
    * ``fault_coupling`` -- ``declared`` (a zone fault also puts the channels of
      the zones in its ``coupled_to`` under fallback policy) or ``none``
      (strictly per zone)
    """

    trust_rule: str = "strict"
    fault_coupling: str = "declared"

    @classmethod
    def coerce(cls, data: object) -> ZonePolicy:
        if isinstance(data, ZonePolicy):
            return data
        raw = _section_keys("zones", data, optional=("trust_rule", "fault_coupling"))
        return cls(
            trust_rule=_choice("zones.trust_rule", raw.get("trust_rule", "strict"), TRUST_RULES),
            fault_coupling=_choice(
                "zones.fault_coupling", raw.get("fault_coupling", "declared"), FAULT_COUPLINGS
            ),
        )

    def to_dict(self) -> dict[str, Any]:
        return {"trust_rule": self.trust_rule, "fault_coupling": self.fault_coupling}


#: ``noise`` defaults (plan section 7): fan affinity exponent, weight of the noise
#: term, hysteresis of the forbidden PWM bands.
NOISE_DEFAULTS: dict[str, float] = {"exponent": 5.0, "weight_noise": 1.0, "band_hysteresis": 0.02}


@dataclass(frozen=True)
class NoiseSpec:
    """``noise``: how fan noise is weighed (plan section 4, "Noise model").

    * ``exponent``        -- sound power ~ rpm ** exponent, in ``[3, 7]``
    * ``weight_noise``    -- weight of the noise term in the DAS MPC cost (``>= 0``);
      the ``quiet`` / ``cool`` presets scale it in DAS mode
    * ``band_hysteresis`` -- PWM hysteresis for leaving a forbidden band (``>= 0``)

    Parsed and validated now; the PI-like DAS solver does not read it (it
    regulates margins, not noise) and the DAS MPC milestone consumes it.
    """

    exponent: float = NOISE_DEFAULTS["exponent"]
    weight_noise: float = NOISE_DEFAULTS["weight_noise"]
    band_hysteresis: float = NOISE_DEFAULTS["band_hysteresis"]

    @classmethod
    def coerce(cls, data: object) -> NoiseSpec:
        if isinstance(data, NoiseSpec):
            return data
        raw = _section_keys("noise", data, optional=tuple(NOISE_DEFAULTS))
        return cls(
            **{
                key: _cfg_num(f"noise.{key}", raw.get(key, default))
                for key, default in NOISE_DEFAULTS.items()
            }
        )

    def to_dict(self) -> dict[str, Any]:
        return {
            "exponent": self.exponent,
            "weight_noise": self.weight_noise,
            "band_hysteresis": self.band_hysteresis,
        }


#: ``estimator`` defaults (plan sections 2 and 7): ``k`` of the ``k * sigma`` margin,
#: sigma trust-rule thresholds, per-tick process noise of the Kalman filter states,
#: sensor white noise, SMART staleness and rejection, occupancy thresholds, SMART
#: calibration expiry and serial -> bay association.
ESTIMATOR_DEFAULTS: dict[str, float] = {
    "k_sigma": 2.0,
    "sigma_fault_c": 4.0,
    "sigma_air_fault_c": 2.0,
    "q_t_air": 1e-4,
    "q_d_air": 4e-7,
    "q_t_drive": 1e-5,
    "q_t_sensor": 1e-4,
    "q_heat": 4e-7,
    "sensor_noise_c": 0.03,
    "smart_max_age_s": 300.0,
    "smart_reject_c": 8.0,
    "occupied_dT_c": 2.0,
    "empty_dT_c": 0.7,
    "empty_confirm_s": 300.0,
    "bay_settle_s": 600.0,
    "calibration_max_age_days": 30.0,
    "associate_window_s": 3600.0,
    "associate_min_corr": 0.8,
    "associate_margin": 0.15,
}


@dataclass(frozen=True)
class EstimatorSpec:
    """``estimator``: the drive temperature estimator (``aqua_bridge.control.estimator``).

    * ``k_sigma``                  -- ``k`` in ``margin = k * sigma``, in ``[0, 4]``
    * ``sigma_fault_c`` / ``sigma_air_fault_c`` -- thresholds of the ``sigma`` zone trust
      rule (parsed; the rule itself is a later milestone)
    * ``q_t_air`` / ``q_d_air`` / ``q_t_drive`` / ``q_t_sensor`` / ``q_heat`` -- process
      noise per tick of the filter states ``T_a``, ``d_a``, ``T_d``, ``T_s``, ``q`` (> 0)
    * ``sensor_noise_c``           -- white noise of a temperature sensor, degC (>= 0);
      the measurement variance is ``sensor_noise_c ** 2 + quant_c ** 2 / 12``
    * ``smart_max_age_s``          -- a SMART sample older than this is ignored and an
      association whose serial stays silent this long is dropped (``>= dt``)
    * ``smart_reject_c``           -- a SMART value this far from the estimate is dropped
    * ``occupied_dT_c`` / ``empty_dT_c`` -- occupancy evidence thresholds on
      ``T_s - T_a`` (``occupied_dT_c > empty_dT_c > 0``)
    * ``empty_confirm_s``          -- low evidence must last this long before a bay is
      empty (``>= 2 dt``)
    * ``bay_settle_s``             -- settling time after an occupancy change (``>= 0``)
    * ``calibration_max_age_days`` -- a SMART calibration without an accepted sample for
      this long is no longer trusted (> 0)
    * ``associate_window_s`` / ``associate_min_corr`` / ``associate_margin`` -- serial
      -> bay association by correlation (``>= 600``, ``(0, 1)``, ``(0, 1)``)
    """

    k_sigma: float = ESTIMATOR_DEFAULTS["k_sigma"]
    sigma_fault_c: float = ESTIMATOR_DEFAULTS["sigma_fault_c"]
    sigma_air_fault_c: float = ESTIMATOR_DEFAULTS["sigma_air_fault_c"]
    q_t_air: float = ESTIMATOR_DEFAULTS["q_t_air"]
    q_d_air: float = ESTIMATOR_DEFAULTS["q_d_air"]
    q_t_drive: float = ESTIMATOR_DEFAULTS["q_t_drive"]
    q_t_sensor: float = ESTIMATOR_DEFAULTS["q_t_sensor"]
    q_heat: float = ESTIMATOR_DEFAULTS["q_heat"]
    sensor_noise_c: float = ESTIMATOR_DEFAULTS["sensor_noise_c"]
    smart_max_age_s: float = ESTIMATOR_DEFAULTS["smart_max_age_s"]
    smart_reject_c: float = ESTIMATOR_DEFAULTS["smart_reject_c"]
    occupied_dT_c: float = ESTIMATOR_DEFAULTS["occupied_dT_c"]  # noqa: N815 - YAML key
    empty_dT_c: float = ESTIMATOR_DEFAULTS["empty_dT_c"]  # noqa: N815 - YAML key
    empty_confirm_s: float = ESTIMATOR_DEFAULTS["empty_confirm_s"]
    bay_settle_s: float = ESTIMATOR_DEFAULTS["bay_settle_s"]
    calibration_max_age_days: float = ESTIMATOR_DEFAULTS["calibration_max_age_days"]
    associate_window_s: float = ESTIMATOR_DEFAULTS["associate_window_s"]
    associate_min_corr: float = ESTIMATOR_DEFAULTS["associate_min_corr"]
    associate_margin: float = ESTIMATOR_DEFAULTS["associate_margin"]

    @classmethod
    def coerce(cls, data: object) -> EstimatorSpec:
        if isinstance(data, EstimatorSpec):
            return data
        raw = _section_keys("estimator", data, optional=tuple(ESTIMATOR_DEFAULTS))
        return cls(
            **{
                key: _cfg_num(f"estimator.{key}", raw.get(key, default))
                for key, default in ESTIMATOR_DEFAULTS.items()
            }
        )

    def to_dict(self) -> dict[str, Any]:
        return {key: getattr(self, key) for key in ESTIMATOR_DEFAULTS}

    def validate(self, dt: float) -> None:
        """Plan section 7 rules; raises :class:`ConfigError`."""
        where = "mpc.estimator"
        if not 0.0 <= self.k_sigma <= 4.0:
            raise ConfigError(f"{where}.k_sigma must be in [0, 4], got {self.k_sigma}")
        for key in ("sigma_fault_c", "sigma_air_fault_c", "smart_reject_c"):
            if getattr(self, key) <= 0:
                raise ConfigError(f"{where}.{key} must be > 0, got {getattr(self, key)}")
        for key in ("q_t_air", "q_d_air", "q_t_drive", "q_t_sensor", "q_heat"):
            if getattr(self, key) <= 0:
                raise ConfigError(f"{where}.{key} must be > 0, got {getattr(self, key)}")
        if self.sensor_noise_c < 0:
            raise ConfigError(f"{where}.sensor_noise_c must be >= 0, got {self.sensor_noise_c}")
        if self.smart_max_age_s < dt:
            raise ConfigError(
                f"{where}.smart_max_age_s must be >= dt ({dt}), got {self.smart_max_age_s}"
            )
        if not self.occupied_dT_c > self.empty_dT_c > 0:
            raise ConfigError(
                f"{where}: occupied_dT_c > empty_dT_c > 0 is required, got "
                f"{self.occupied_dT_c} and {self.empty_dT_c}"
            )
        if self.empty_confirm_s < 2 * dt:
            raise ConfigError(
                f"{where}.empty_confirm_s must be >= 2 * dt ({2 * dt}), got {self.empty_confirm_s}"
            )
        if self.bay_settle_s < 0:
            raise ConfigError(f"{where}.bay_settle_s must be >= 0, got {self.bay_settle_s}")
        if self.calibration_max_age_days <= 0:
            raise ConfigError(
                f"{where}.calibration_max_age_days must be > 0, got {self.calibration_max_age_days}"
            )
        if self.associate_window_s < 600:
            raise ConfigError(
                f"{where}.associate_window_s must be >= 600, got {self.associate_window_s}"
            )
        for key in ("associate_min_corr", "associate_margin"):
            if not 0.0 < getattr(self, key) < 1.0:
                raise ConfigError(f"{where}.{key} must be in (0, 1), got {getattr(self, key)}")


@dataclass(frozen=True)
class StuckParams:
    """Stuck rule parameters of one temperature (gate rule 3), derived from the config.

    * ``ticks``    -- window length in ticks
    * ``eps_c``    -- "unchanged" band, degrees C
    * ``decimate`` -- the window keeps one sample every ``decimate`` ticks (1: the
      dense ``MpcState.window``; > 1: a decimated window kept in ``solver_memory``)
    * ``samples``  -- stored samples the check needs (``ceil(ticks / decimate)``)
    * ``channels`` -- channels whose net PWM move counts as evidence
    * ``siblings`` -- temperatures whose plausible net move counts as evidence

    Legacy mode: the global ``stuck_s`` / ``stuck_eps_c``, every channel and every
    other temperature. With ``topology``: the sensor's own values, the channels
    of its zone (none for a sensor without a zone) and the other sensors of the
    same zone and role.
    """

    ticks: int
    eps_c: float
    decimate: int
    samples: int
    channels: tuple[str, ...]
    siblings: tuple[str, ...]


@dataclass(frozen=True)
class ZoneLayout:
    """Structural view of the zones, derived from the declared config (never identified numbers).

    Legacy mode (``implicit``): the single :data:`IMPLICIT_ZONE` holding every
    channel, no coupling, and one required group per temperature, so "every
    group has a trusted member" is exactly the whole-tick gate rule 4.

    * ``required_groups[zone]`` -- ``(label, members)`` pairs; with the ``strict`` rule
      the zone is trusted only when every group has at least one gate-trusted member
    * ``reach[zone]``           -- channels put under fallback policy while the zone is
      in fault: its own channels plus, with ``fault_coupling: declared``, those of
      the zones in its ``coupled_to``
    * ``served[channel]``       -- zones whose air the channel moves or exchanges air
      with: the zones that list it plus their declared ``coupled_to`` (independent of
      ``fault_coupling``); the PI-like DAS solver regulates the drives of these zones
    """

    implicit: bool
    zones: tuple[str, ...]
    zone_channels: dict[str, tuple[str, ...]]
    coupled: dict[str, tuple[str, ...]]
    channel_zones: dict[str, tuple[str, ...]]
    sensor_zone: dict[str, str | None]
    required_groups: dict[str, tuple[tuple[str, tuple[str, ...]], ...]]
    reach: dict[str, tuple[str, ...]]
    served: dict[str, tuple[str, ...]] = field(default_factory=dict)


def _strictest_class(classes: Mapping[str, DriveClass]) -> str:
    """Lowest ``limit_c``, then lowest ``limit_c - comfort_c``, then name."""
    return min(
        classes,
        key=lambda n: (classes[n].limit_c, classes[n].limit_c - classes[n].comfort_c, n),
    )


def _served_zones(
    channel: str,
    zones: tuple[str, ...],
    zone_channels: Mapping[str, tuple[str, ...]],
    coupled: Mapping[str, tuple[str, ...]],
) -> tuple[str, ...]:
    """Zones listing ``channel`` plus their declared ``coupled_to``, in zone order."""
    members: set[str] = set()
    for z in zones:
        if channel in zone_channels[z]:
            members.add(z)
            members.update(coupled[z])
    return tuple(z for z in zones if z in members)


def _plain_entry(value: Any) -> Any:
    if isinstance(value, tuple):
        return list(value)
    if hasattr(value, "to_dict"):
        return value.to_dict()
    return value


@dataclass(frozen=True)
class _Derived:
    """What :meth:`MpcConfig.validate` derives once (not a config field)."""

    layout: ZoneLayout
    stuck: dict[str, StuckParams]
    window_ticks: int
    slow_samples: dict[int, int]


@dataclass(frozen=True)
class MpcConfig:
    """Controller configuration, the ``mpc:`` section of ``config.yaml``.

    Physical units live here (seconds, degrees C per second). Tick
    quantities are *derived* (``dT_max_tick``, ``confirm_ticks``,
    ``stuck_ticks``) so changing ``dt`` never silently changes a slew limit.

    Fields beyond the spec's list, each documented in ``config.example.yaml``:

    * ``solver``         -- ``"pi"`` (default) or ``"mpc"``
    * ``pi_kp``/``pi_ki`` -- global PI gains, PWM per degree C and PWM per (degree C * s)
    * ``channel_temps``  -- fan channel -> temperatures it controls. Every listed
      temperature must have a setpoint. Empty (default) means every channel
      controls every setpoint temperature. Recommended semantics for a
      cooling loop: a channel's error is the *maximum* over its temperatures
      of ``T - setpoint``, so the hottest deviation drives the fan.
    * ``weights``        -- per-temperature tracking weights for the MPC cost
      (missing temperature -> 1.0); ``weight_pwm``/``weight_dpwm`` penalise
      effort and moves.
    * ``temp_min_c``/``temp_max_c`` -- the gate's absolute valid range
    * ``solver_max_iter`` -- iteration cap; hitting it is a solver fault
    * ``mpc_tau_s`` / ``mpc_gain_c_per_pwm`` / ``mpc_estimator_gain`` -- the
      linear MPC's plant model (``control/solver_mpc.py``): first-order time
      constant of every controlled temperature, steady-state degrees C a
      controlled temperature drops when one of its channels goes +1.0 PWM,
      and the per-tick gain of the offset-free disturbance estimator. Ignored
      by the ``pi`` solver.
    * ``model_shadow`` / ``model_window_s`` / ``model_lambda`` / ``model_p_trace_max`` /
      ``model_converged_rel_se`` / ``model_max_pred_err_c`` / ``model_use_rpm`` -- the
      zoned thermal model's online identification (``control/thermal.py``): shadow
      learning on/off, regression window, RLS forgetting per window, covariance trace
      bound, relative standard error and prediction error for ``converged``, and fan
      airflow from the tachometer instead of the PWM curve.
    * ``topology`` / ``sensors`` / ``drive_classes`` / ``fans`` / ``fan_models`` /
      ``zones`` / ``noise`` / ``estimator`` -- the zoned DAS layout (module
      docstring, *DAS layout*); all absent is legacy mode. With ``topology`` the
      gate's Stuck rule is sized
      per sensor (:meth:`stuck_params`), zone trust and fallback run per zone
      (``aqua_bridge.control.zones``) and ``temps_for_channel`` returns the
      setpoint temperatures of the channel's zones.
    """

    dt: float
    horizon: int
    temps: tuple[str, ...]
    setpoints: dict[str, float]
    pwm_min: float
    pwm_max: float
    d_pwm_max: float
    fallback_pwm: dict[str, float]
    fallback_hold_s: float
    confirm_s: float
    dT_max_c_per_s: float  # noqa: N815 - name matches the YAML key and the spec
    stuck_s: float
    stuck_eps_c: float
    stuck_pwm_net: float
    stuck_sibling_dT_c: float  # noqa: N815
    channels: tuple[str, ...]
    median3: bool = False
    weights: dict[str, float] = field(default_factory=dict)
    weight_pwm: float = 0.0
    weight_dpwm: float = 0.05
    solver: SolverKind = SolverKind.PI
    pi_kp: float = 0.05
    pi_ki: float = 0.002
    channel_temps: dict[str, tuple[str, ...]] = field(default_factory=dict)
    temp_min_c: float = -20.0
    temp_max_c: float = 120.0
    solver_max_iter: int = 50
    mpc_tau_s: float = 120.0
    mpc_gain_c_per_pwm: float = 8.0
    mpc_estimator_gain: float = 0.1
    topology: Topology | None = None
    sensors: dict[str, SensorSpec] = field(default_factory=dict)
    drive_classes: dict[str, DriveClass] = field(default_factory=dict)
    fans: dict[str, FanSpec] = field(default_factory=dict)
    fan_models: dict[str, FanModel] = field(default_factory=dict)
    zones: ZonePolicy | None = None
    noise: NoiseSpec | None = None
    estimator: EstimatorSpec | None = None
    model_shadow: bool = False
    model_window_s: float = 120.0
    model_lambda: float = 0.9995
    model_p_trace_max: float = 100.0
    model_converged_rel_se: float = 0.25
    model_max_pred_err_c: float = 1.0
    model_use_rpm: bool = False

    # -- construction -------------------------------------------------------

    def __post_init__(self) -> None:
        self._coerce()
        self.validate()

    def _coerce(self) -> None:
        """Type-check and normalise every field (list -> tuple, int -> float, ...)."""
        s = object.__setattr__
        s(self, "dt", _cfg_num("dt", self.dt))
        s(self, "horizon", _cfg_int("horizon", self.horizon))
        s(self, "temps", _cfg_names("temps", self.temps))
        s(self, "setpoints", _cfg_float_map("setpoints", self.setpoints))
        s(self, "pwm_min", _cfg_num("pwm_min", self.pwm_min))
        s(self, "pwm_max", _cfg_num("pwm_max", self.pwm_max))
        s(self, "d_pwm_max", _cfg_num("d_pwm_max", self.d_pwm_max))
        s(self, "fallback_pwm", _cfg_float_map("fallback_pwm", self.fallback_pwm))
        s(self, "fallback_hold_s", _cfg_num("fallback_hold_s", self.fallback_hold_s))
        s(self, "confirm_s", _cfg_num("confirm_s", self.confirm_s))
        s(self, "dT_max_c_per_s", _cfg_num("dT_max_c_per_s", self.dT_max_c_per_s))
        s(self, "stuck_s", _cfg_num("stuck_s", self.stuck_s))
        s(self, "stuck_eps_c", _cfg_num("stuck_eps_c", self.stuck_eps_c))
        s(self, "stuck_pwm_net", _cfg_num("stuck_pwm_net", self.stuck_pwm_net))
        s(self, "stuck_sibling_dT_c", _cfg_num("stuck_sibling_dT_c", self.stuck_sibling_dT_c))
        s(self, "channels", _cfg_names("channels", self.channels))
        if not isinstance(self.median3, bool):
            raise ConfigError(
                f"mpc.median3 must be a bool (true/false), got {type(self.median3).__name__}: "
                f"{self.median3!r}"
            )
        s(self, "weights", _cfg_float_map("weights", self.weights))
        s(self, "weight_pwm", _cfg_num("weight_pwm", self.weight_pwm))
        s(self, "weight_dpwm", _cfg_num("weight_dpwm", self.weight_dpwm))
        try:
            s(self, "solver", SolverKind(self.solver))
        except ValueError as exc:
            raise ConfigError(
                f"mpc.solver must be one of {[k.value for k in SolverKind]}, got {self.solver!r}"
            ) from exc
        s(self, "pi_kp", _cfg_num("pi_kp", self.pi_kp))
        s(self, "pi_ki", _cfg_num("pi_ki", self.pi_ki))
        if not isinstance(self.channel_temps, Mapping):
            raise ConfigError("mpc.channel_temps must be a mapping channel -> [temps]")
        channel_temps: dict[str, tuple[str, ...]] = {}
        for key, raw in self.channel_temps.items():
            if not isinstance(key, str) or not key:
                raise ConfigError(f"mpc.channel_temps keys must be non-empty strings, got {key!r}")
            if isinstance(raw, str):
                raw = (raw,)
            channel_temps[key] = _cfg_names(f"channel_temps[{key!r}]", raw)
        s(self, "channel_temps", channel_temps)
        s(self, "temp_min_c", _cfg_num("temp_min_c", self.temp_min_c))
        s(self, "temp_max_c", _cfg_num("temp_max_c", self.temp_max_c))
        s(self, "solver_max_iter", _cfg_int("solver_max_iter", self.solver_max_iter))
        s(self, "mpc_tau_s", _cfg_num("mpc_tau_s", self.mpc_tau_s))
        s(self, "mpc_gain_c_per_pwm", _cfg_num("mpc_gain_c_per_pwm", self.mpc_gain_c_per_pwm))
        s(self, "mpc_estimator_gain", _cfg_num("mpc_estimator_gain", self.mpc_estimator_gain))
        for name in ("model_shadow", "model_use_rpm"):
            _cfg_bool(name, getattr(self, name))
        for name in (
            "model_window_s",
            "model_lambda",
            "model_p_trace_max",
            "model_converged_rel_se",
            "model_max_pred_err_c",
        ):
            s(self, name, _cfg_num(name, getattr(self, name)))
        self._coerce_das()

    def _coerce_das(self) -> None:
        """Parse the DAS sections; fill their defaults only when ``topology`` is present."""
        s = object.__setattr__
        topology = None if self.topology is None else Topology.coerce(self.topology)
        s(
            self,
            "sensors",
            {
                name: SensorSpec.coerce(f"sensors.{name}", entry)
                for name, entry in _named_map("sensors", self.sensors).items()
            },
        )
        classes = {
            name: DriveClass.coerce(f"drive_classes.{name}", entry)
            for name, entry in _named_map("drive_classes", self.drive_classes).items()
        }
        if topology is not None and not classes:
            classes = dict(BUILTIN_DRIVE_CLASSES)
        s(self, "drive_classes", classes)
        s(
            self,
            "fans",
            {
                name: FanSpec.coerce(f"fans.{name}", entry)
                for name, entry in _named_map("fans", self.fans).items()
            },
        )
        s(
            self,
            "fan_models",
            {
                name: FanModel.coerce(f"fan_models.{name}", entry)
                for name, entry in _named_map("fan_models", self.fan_models).items()
            },
        )
        zones = None if self.zones is None else ZonePolicy.coerce(self.zones)
        noise = None if self.noise is None else NoiseSpec.coerce(self.noise)
        estimator = None if self.estimator is None else EstimatorSpec.coerce(self.estimator)
        if topology is not None:
            if noise is None:
                noise = NoiseSpec()
            if estimator is None:
                estimator = EstimatorSpec()
            if zones is None:
                zones = ZonePolicy()
            if topology.default_class is None and classes:
                topology = dataclasses.replace(topology, default_class=_strictest_class(classes))
        s(self, "topology", topology)
        s(self, "zones", zones)
        s(self, "noise", noise)
        s(self, "estimator", estimator)

    # -- validation ---------------------------------------------------------

    def validate(self) -> None:
        """Raise :class:`ConfigError` for every rule in section 4.3 "Config / solver"."""
        if self.dt <= 0:
            raise ConfigError(f"mpc.dt must be > 0, got {self.dt}")
        if self.horizon < 1:
            raise ConfigError(f"mpc.horizon must be >= 1, got {self.horizon}")
        if not self.temps:
            raise ConfigError("mpc.temps must not be empty")
        if not self.channels:
            raise ConfigError("mpc.channels must not be empty")

        if not 0.0 <= self.pwm_min <= 1.0:
            raise ConfigError(f"mpc.pwm_min must be in [0, 1], got {self.pwm_min}")
        if not 0.0 <= self.pwm_max <= 1.0:
            raise ConfigError(f"mpc.pwm_max must be in [0, 1], got {self.pwm_max}")
        if self.pwm_min >= self.pwm_max:
            raise ConfigError(
                f"mpc.pwm_min must be < mpc.pwm_max, got {self.pwm_min} >= {self.pwm_max}"
            )
        if self.d_pwm_max <= 0:
            raise ConfigError(f"mpc.d_pwm_max must be > 0, got {self.d_pwm_max}")

        if set(self.fallback_pwm) != set(self.channels):
            raise ConfigError(
                "mpc.fallback_pwm keys must be exactly mpc.channels: "
                f"{sorted(self.fallback_pwm)} != {sorted(self.channels)}"
            )
        for ch, value in self.fallback_pwm.items():
            # (pwm_min, pwm_max]: equal to pwm_min is a rejected config, never a comment.
            if not self.pwm_min < value <= self.pwm_max:
                raise ConfigError(
                    f"mpc.fallback_pwm[{ch!r}]={value} must be in ({self.pwm_min}, {self.pwm_max}]"
                )

        if self.fallback_hold_s < 0:
            raise ConfigError(f"mpc.fallback_hold_s must be >= 0, got {self.fallback_hold_s}")
        if self.confirm_s < 2 * self.dt:
            raise ConfigError(
                f"mpc.confirm_s must be >= 2 * dt ({2 * self.dt}), got {self.confirm_s}"
            )
        if self.confirm_s > self.fallback_hold_s:
            raise ConfigError(
                f"mpc.confirm_s ({self.confirm_s}) must be <= mpc.fallback_hold_s "
                f"({self.fallback_hold_s}): a real Jump would ramp high before it confirms"
            )
        if self.dT_max_c_per_s <= 0:
            raise ConfigError(f"mpc.dT_max_c_per_s must be > 0, got {self.dT_max_c_per_s}")
        if self.stuck_s < 2 * self.dt:
            raise ConfigError(f"mpc.stuck_s must be >= 2 * dt ({2 * self.dt}), got {self.stuck_s}")
        if self.stuck_eps_c <= 0:
            raise ConfigError(f"mpc.stuck_eps_c must be > 0, got {self.stuck_eps_c}")
        if not 0.0 < self.stuck_pwm_net <= 1.0:
            raise ConfigError(f"mpc.stuck_pwm_net must be in (0, 1], got {self.stuck_pwm_net}")
        if self.stuck_sibling_dT_c <= 0:
            raise ConfigError(f"mpc.stuck_sibling_dT_c must be > 0, got {self.stuck_sibling_dT_c}")

        if self.temp_min_c >= self.temp_max_c:
            raise ConfigError(
                f"mpc.temp_min_c ({self.temp_min_c}) must be < mpc.temp_max_c ({self.temp_max_c})"
            )
        # Conservative reading: a controller with no target has nothing to
        # regulate, so an empty setpoints map is rejected (spec only demands
        # keys ⊆ temps). A setpoint outside the gate's valid range can never
        # be reached by a trusted reading, so it is rejected too.
        if not self.setpoints and self.topology is None:
            raise ConfigError("mpc.setpoints must not be empty")
        for name, value in self.setpoints.items():
            if name not in self.temps:
                raise ConfigError(f"mpc.setpoints key {name!r} is not in mpc.temps {self.temps}")
            if not self.temp_min_c < value < self.temp_max_c:
                raise ConfigError(
                    f"mpc.setpoints[{name!r}]={value} must lie inside "
                    f"({self.temp_min_c}, {self.temp_max_c})"
                )

        for name, value in self.weights.items():
            if name not in self.temps:
                raise ConfigError(f"mpc.weights key {name!r} is not in mpc.temps {self.temps}")
            if value < 0:
                raise ConfigError(f"mpc.weights[{name!r}] must be >= 0, got {value}")
        if self.weight_pwm < 0:
            raise ConfigError(f"mpc.weight_pwm must be >= 0, got {self.weight_pwm}")
        if self.weight_dpwm < 0:
            raise ConfigError(f"mpc.weight_dpwm must be >= 0, got {self.weight_dpwm}")

        if self.pi_kp <= 0:
            raise ConfigError(f"mpc.pi_kp must be > 0, got {self.pi_kp}")
        if self.pi_ki < 0:
            raise ConfigError(f"mpc.pi_ki must be >= 0, got {self.pi_ki}")

        if self.channel_temps and self.topology is not None:
            raise ConfigError(
                "mpc.channel_temps must be absent with mpc.topology: a channel controls "
                "the setpoint temperatures of the zones that list it"
            )
        if self.channel_temps:
            if set(self.channel_temps) != set(self.channels):
                raise ConfigError(
                    "mpc.channel_temps keys must be exactly mpc.channels when given: "
                    f"{sorted(self.channel_temps)} != {sorted(self.channels)}"
                )
            for ch, names in self.channel_temps.items():
                if not names:
                    raise ConfigError(f"mpc.channel_temps[{ch!r}] must not be empty")
                for name in names:
                    if name not in self.setpoints:
                        raise ConfigError(
                            f"mpc.channel_temps[{ch!r}] lists {name!r}, which has no setpoint"
                        )

        if self.solver_max_iter < 1:
            raise ConfigError(f"mpc.solver_max_iter must be >= 1, got {self.solver_max_iter}")

        if self.mpc_tau_s <= 0:
            raise ConfigError(f"mpc.mpc_tau_s must be > 0, got {self.mpc_tau_s}")
        if self.mpc_gain_c_per_pwm <= 0:
            raise ConfigError(f"mpc.mpc_gain_c_per_pwm must be > 0, got {self.mpc_gain_c_per_pwm}")
        if not 0.0 < self.mpc_estimator_gain <= 1.0:
            raise ConfigError(
                f"mpc.mpc_estimator_gain must be in (0, 1], got {self.mpc_estimator_gain}"
            )
        if self.solver is SolverKind.MPC:
            # The QP is strictly convex only through the effort / move penalties
            # (with more channels than temperatures the tracking term alone is
            # singular), and a cost without any tracking weight regulates nothing.
            if self.weight_pwm + self.weight_dpwm <= 0:
                raise ConfigError("mpc.weight_pwm + mpc.weight_dpwm must be > 0 with solver 'mpc'")
            if not any(self.weight_for(name) > 0 for name in self.setpoints):
                raise ConfigError(
                    "mpc.weights: at least one setpoint temperature needs a weight > 0 "
                    "with solver 'mpc'"
                )

        self._validate_model_keys()

        if self.topology is None:
            for name in ("sensors", "drive_classes", "fans", "fan_models"):
                if getattr(self, name):
                    raise ConfigError(f"mpc.{name} requires mpc.topology (DAS layout)")
            if self.zones is not None:
                raise ConfigError("mpc.zones requires mpc.topology (DAS layout)")
            if self.noise is not None:
                raise ConfigError("mpc.noise requires mpc.topology (DAS layout)")
            if self.estimator is not None:
                raise ConfigError("mpc.estimator requires mpc.topology (DAS layout)")
        else:
            self._validate_das()
        object.__setattr__(self, "_derived", self._derive())
        if self.topology is not None and self.setpoints:
            for ch in self.channels:
                if not self.temps_for_channel(ch):
                    raise ConfigError(
                        f"mpc.setpoints: channel {ch!r} controls no setpoint temperature; with "
                        "mpc.topology a channel controls the setpoint sensors of its zones "
                        f"{list(self._derived.layout.channel_zones[ch])}"
                    )

    def _validate_model_keys(self) -> None:
        """Rules for the ``model_*`` keys of the thermal model's identification."""
        for name in ("model_shadow", "model_use_rpm"):
            if getattr(self, name) and self.topology is None:
                raise ConfigError(f"mpc.{name}: true requires mpc.topology (DAS layout)")
        if self.model_window_s <= 0 or self.model_window_s > 600:
            raise ConfigError(f"mpc.model_window_s must be in (0, 600], got {self.model_window_s}")
        if self.model_shadow and self.model_window_s < 2 * self.dt:
            raise ConfigError(
                f"mpc.model_window_s must be >= 2 * dt ({2 * self.dt}) with model_shadow, "
                f"got {self.model_window_s}"
            )
        if not 0.99 < self.model_lambda <= 1.0:
            raise ConfigError(f"mpc.model_lambda must be in (0.99, 1], got {self.model_lambda}")
        if self.model_p_trace_max <= 0:
            raise ConfigError(f"mpc.model_p_trace_max must be > 0, got {self.model_p_trace_max}")
        if not 0.0 < self.model_converged_rel_se < 1.0:
            raise ConfigError(
                f"mpc.model_converged_rel_se must be in (0, 1), got {self.model_converged_rel_se}"
            )
        if self.model_max_pred_err_c <= 0:
            raise ConfigError(
                f"mpc.model_max_pred_err_c must be > 0, got {self.model_max_pred_err_c}"
            )

    def _validate_das(self) -> None:
        """Section 7 rules for ``topology``, ``sensors``, ``drive_classes``, ``fans``,
        ``fan_models`` and ``zones`` (plus the module docstring's conservative ones)."""
        topo = self.topology
        assert topo is not None
        dt = self.dt

        # topology.zones
        if not topo.zones:
            raise ConfigError("mpc.topology.zones must not be empty")
        for z, spec in topo.zones.items():
            if not isinstance(spec, ZoneSpec):
                raise ConfigError(f"mpc.topology.zones[{z!r}] must be a zone entry")
            if not spec.channels:
                raise ConfigError(f"mpc.topology.zones.{z}.channels must not be empty")
            for ch in spec.channels:
                if ch not in self.channels:
                    raise ConfigError(
                        f"mpc.topology.zones.{z}.channels lists {ch!r}, not in mpc.channels"
                    )
            for other in spec.coupled_to:
                if other == z:
                    raise ConfigError(f"mpc.topology.zones.{z}.coupled_to must not list itself")
                if other not in topo.zones:
                    raise ConfigError(
                        f"mpc.topology.zones.{z}.coupled_to lists unknown zone {other!r}"
                    )
                if z not in topo.zones[other].coupled_to:
                    raise ConfigError(
                        f"mpc.topology.zones: coupling must be symmetric, {z!r} lists {other!r} "
                        f"but {other!r} does not list {z!r}"
                    )
            if spec.inlet != _MIX:
                inlet = self.sensors.get(spec.inlet)
                if inlet is None or inlet.role != "inlet":
                    raise ConfigError(
                        f"mpc.topology.zones.{z}.inlet must be 'mix' or a sensor of role "
                        f"inlet, got {spec.inlet!r}"
                    )
        for ch in self.channels:
            if not any(ch in spec.channels for spec in topo.zones.values()):
                raise ConfigError(f"mpc.channels: {ch!r} appears in no mpc.topology zone")

        # drive_classes
        for name, dc in self.drive_classes.items():
            if not self.temp_min_c < dc.limit_c < self.temp_max_c:
                raise ConfigError(
                    f"mpc.drive_classes.{name}.limit_c={dc.limit_c} must lie inside "
                    f"({self.temp_min_c}, {self.temp_max_c})"
                )
            if dc.comfort_c < 0:
                raise ConfigError(f"mpc.drive_classes.{name}.comfort_c must be >= 0")
            if dc.tau_d_s <= 0:
                raise ConfigError(f"mpc.drive_classes.{name}.tau_d_s must be > 0")
        if topo.default_class not in self.drive_classes:
            raise ConfigError(
                f"mpc.topology.default_class {topo.default_class!r} is not in mpc.drive_classes "
                f"{sorted(self.drive_classes)}"
            )

        # topology.bays
        for b, bay in topo.bays.items():
            if not isinstance(bay, BaySpec):
                raise ConfigError(f"mpc.topology.bays[{b!r}] must be a bay entry")
            if bay.zone not in topo.zones:
                raise ConfigError(f"mpc.topology.bays.{b}.zone {bay.zone!r} is not a zone")
            if bay.drive_class is not None and bay.drive_class not in self.drive_classes:
                raise ConfigError(
                    f"mpc.topology.bays.{b}.class {bay.drive_class!r} is not in mpc.drive_classes"
                )
            if not (isinstance(bay.occupied, bool) or bay.occupied == "auto"):
                raise ConfigError(f"mpc.topology.bays.{b}.occupied must be true, false or auto")
            if bay.limit_c is not None and not self.temp_min_c < bay.limit_c < self.temp_max_c:
                raise ConfigError(
                    f"mpc.topology.bays.{b}.limit_c={bay.limit_c} must lie inside "
                    f"({self.temp_min_c}, {self.temp_max_c})"
                )
        serial_bays: dict[str, str] = {}
        for b, bay in topo.bays.items():
            if bay.serial is None:
                continue
            if bay.serial in serial_bays:
                # a declared serial wins over the association by correlation; two bays
                # declaring it would make SMART data of one drive calibrate both
                raise ConfigError(
                    f"mpc.topology.bays: serial {bay.serial!r} is declared on both "
                    f"{serial_bays[bay.serial]!r} and {b!r}"
                )
            serial_bays[bay.serial] = b

        # sensors
        if set(self.sensors) != set(self.temps):
            raise ConfigError(
                "mpc.sensors keys must be exactly mpc.temps with mpc.topology: "
                f"missing {sorted(set(self.temps) - set(self.sensors))}, "
                f"extra {sorted(set(self.sensors) - set(self.temps))}"
            )
        for name, sp in self.sensors.items():
            where = f"mpc.sensors.{name}"
            if sp.role not in SENSOR_ROLES:
                raise ConfigError(f"{where}.role must be one of {list(SENSOR_ROLES)}")
            if sp.role in ("zone_air", "drive_proximal") and sp.zone is None:
                raise ConfigError(f"{where}.zone is required for role {sp.role}")
            if sp.zone is not None and sp.zone not in topo.zones:
                raise ConfigError(f"{where}.zone {sp.zone!r} is not a zone")
            if sp.role == "drive_proximal":
                if sp.bay is None:
                    raise ConfigError(f"{where}.bay is required for role drive_proximal")
                if sp.bay not in topo.bays:
                    raise ConfigError(f"{where}.bay {sp.bay!r} is not a bay")
                if topo.bays[sp.bay].zone != sp.zone:
                    raise ConfigError(
                        f"{where}: bay {sp.bay!r} belongs to zone {topo.bays[sp.bay].zone!r}, "
                        f"not {sp.zone!r}"
                    )
            elif sp.bay is not None:
                raise ConfigError(f"{where}.bay: only drive_proximal sensors sit in a bay")
            if sp.quant_c <= 0:
                raise ConfigError(f"{where}.quant_c must be > 0, got {sp.quant_c}")
            if sp.stuck_s is None or sp.stuck_s < 2 * dt:
                raise ConfigError(f"{where}.stuck_s must be >= 2 * dt ({2 * dt}), got {sp.stuck_s}")
            if sp.stuck_eps_c is None or sp.stuck_eps_c < sp.quant_c:
                raise ConfigError(
                    f"{where}.stuck_eps_c must be >= quant_c ({sp.quant_c}), got {sp.stuck_eps_c}"
                )
            if sp.stuck_decimate is not None and sp.stuck_decimate < 1:
                raise ConfigError(f"{where}.stuck_decimate must be >= 1")
            if sp.tau_s is not None and sp.tau_s <= 0:
                raise ConfigError(f"{where}.tau_s must be > 0")
            if not isinstance(sp.redundant, bool):
                raise ConfigError(f"{where}.redundant must be a bool")

        def has_primary(role: str, *, zone: str | None = None, bay: str | None = None) -> bool:
            return any(
                sp.role == role
                and not sp.redundant
                and (zone is None or sp.zone == zone)
                and (bay is None or sp.bay == bay)
                for sp in self.sensors.values()
            )

        for z in topo.zones:
            if not has_primary("zone_air", zone=z):
                raise ConfigError(
                    f"mpc.sensors: zone {z!r} needs a zone_air sensor that is not redundant"
                )
        for b in topo.bays:
            if not has_primary("drive_proximal", bay=b):
                raise ConfigError(
                    f"mpc.sensors: bay {b!r} needs a drive_proximal sensor that is not redundant"
                )
        if not has_primary("inlet"):
            raise ConfigError("mpc.sensors: at least one inlet sensor that is not redundant")
        for name in self.setpoints:
            if self.sensors[name].role not in ("zone_air", "drive_proximal"):
                raise ConfigError(
                    f"mpc.setpoints[{name!r}]: with mpc.topology a setpoint must sit on a "
                    "zone_air or drive_proximal sensor"
                )

        # fans / fan_models
        if set(self.fans) != set(self.channels):
            raise ConfigError(
                "mpc.fans keys must be exactly mpc.channels with mpc.topology: "
                f"{sorted(self.fans)} != {sorted(self.channels)}"
            )
        for name, fm in self.fan_models.items():
            where = f"mpc.fan_models.{name}"
            if fm.rpm_max <= 0:
                raise ConfigError(f"{where}.rpm_max must be > 0, got {fm.rpm_max}")
            if not 0.0 <= fm.deadband < 0.5:
                raise ConfigError(f"{where}.deadband must be in [0, 0.5), got {fm.deadband}")
            if not 0.5 <= fm.exponent <= 1.5:
                raise ConfigError(f"{where}.exponent must be in [0.5, 1.5], got {fm.exponent}")
            if not math.isfinite(fm.noise_db_at_max):
                raise ConfigError(f"{where}.noise_db_at_max must be finite")
        for ch, fan in self.fans.items():
            where = f"mpc.fans.{ch}"
            if fan.model not in self.fan_models:
                raise ConfigError(f"{where}.model {fan.model!r} is not in mpc.fan_models")
            if fan.count < 1:
                raise ConfigError(f"{where}.count must be >= 1, got {fan.count}")
            if fan.noise_weight < 0:
                raise ConfigError(f"{where}.noise_weight must be >= 0, got {fan.noise_weight}")
            for lo, hi in fan.forbidden_pwm:
                if not self.pwm_min <= lo < hi <= self.pwm_max:
                    raise ConfigError(
                        f"{where}.forbidden_pwm band [{lo}, {hi}] must satisfy "
                        f"pwm_min <= lo < hi <= pwm_max ({self.pwm_min}, {self.pwm_max})"
                    )
                if hi - lo > 0.2 + 1e-12:
                    raise ConfigError(f"{where}.forbidden_pwm band [{lo}, {hi}] is wider than 0.2")

        # zones policy
        policy = self.zones
        if policy is None or policy.trust_rule not in TRUST_RULES:
            raise ConfigError(f"mpc.zones.trust_rule must be one of {list(TRUST_RULES)}")
        if policy.fault_coupling not in FAULT_COUPLINGS:
            raise ConfigError(f"mpc.zones.fault_coupling must be one of {list(FAULT_COUPLINGS)}")

        noise = self.noise
        if not isinstance(noise, NoiseSpec):
            raise ConfigError("mpc.noise must be a noise entry")
        if not 3.0 <= noise.exponent <= 7.0:
            raise ConfigError(f"mpc.noise.exponent must be in [3, 7], got {noise.exponent}")
        if noise.weight_noise < 0:
            raise ConfigError(f"mpc.noise.weight_noise must be >= 0, got {noise.weight_noise}")
        if noise.band_hysteresis < 0:
            raise ConfigError(
                f"mpc.noise.band_hysteresis must be >= 0, got {noise.band_hysteresis}"
            )
        if not isinstance(self.estimator, EstimatorSpec):
            raise ConfigError("mpc.estimator must be an estimator entry")
        self.estimator.validate(dt)

    def _derive(self) -> _Derived:
        """Zone layout and per-temperature Stuck parameters (validated config only)."""
        channels = tuple(self.channels)
        topo = self.topology
        if topo is None:
            n = self.stuck_ticks
            zone = IMPLICIT_ZONE
            layout = ZoneLayout(
                implicit=True,
                zones=(zone,),
                zone_channels={zone: channels},
                coupled={zone: ()},
                channel_zones={ch: (zone,) for ch in channels},
                sensor_zone=dict.fromkeys(self.temps, zone),
                required_groups={zone: tuple((t, (t,)) for t in self.temps)},
                reach={zone: channels},
                served={ch: (zone,) for ch in channels},
            )
            stuck = {
                t: StuckParams(
                    ticks=n,
                    eps_c=self.stuck_eps_c,
                    decimate=1,
                    samples=n,
                    channels=channels,
                    siblings=tuple(o for o in self.temps if o != t),
                )
                for t in self.temps
            }
            return _Derived(layout=layout, stuck=stuck, window_ticks=n, slow_samples={})

        sensors = self.sensors
        zones = tuple(topo.zones)
        zone_channels = {z: tuple(topo.zones[z].channels) for z in zones}
        coupled = {z: tuple(topo.zones[z].coupled_to) for z in zones}
        declared = self.zones is None or self.zones.fault_coupling == "declared"
        reach: dict[str, tuple[str, ...]] = {}
        for z in zones:
            touched = set(zone_channels[z])
            if declared:
                for other in coupled[z]:
                    touched.update(zone_channels[other])
            reach[z] = tuple(ch for ch in channels if ch in touched)
        groups: dict[str, tuple[tuple[str, tuple[str, ...]], ...]] = {}
        for z in zones:
            air = tuple(
                t for t in self.temps if sensors[t].role == "zone_air" and sensors[t].zone == z
            )
            zone_groups: list[tuple[str, tuple[str, ...]]] = [("zone_air", air)]
            for b, bay in topo.bays.items():
                if bay.zone != z or not bay.constrained:
                    continue
                members = tuple(
                    t
                    for t in self.temps
                    if sensors[t].role == "drive_proximal" and sensors[t].bay == b
                )
                zone_groups.append((f"bay:{b}", members))
            for t in self.temps:
                if t in self.setpoints and sensors[t].zone == z:
                    zone_groups.append((f"setpoint:{t}", (t,)))
            groups[z] = tuple(zone_groups)

        stuck: dict[str, StuckParams] = {}
        for t in self.temps:
            sp = sensors[t]
            assert sp.stuck_s is not None and sp.stuck_eps_c is not None
            n = max(2, math.ceil(sp.stuck_s / self.dt))
            k = sp.stuck_decimate or max(1, n // STUCK_WINDOW_SAMPLES)
            samples = n if k == 1 else math.ceil(n / k)
            if samples < 2:
                raise ConfigError(
                    f"mpc.sensors.{t}.stuck_decimate={k} leaves fewer than 2 samples in a "
                    f"{n}-tick Stuck window"
                )
            stuck[t] = StuckParams(
                ticks=n,
                eps_c=sp.stuck_eps_c,
                decimate=k,
                samples=samples,
                channels=zone_channels[sp.zone] if sp.zone is not None else (),
                siblings=tuple(
                    o
                    for o in self.temps
                    if o != t and sensors[o].zone == sp.zone and sensors[o].role == sp.role
                ),
            )
        # At least 3: the decimated windows take their median3 value from the newest
        # three dense samples, so that value equals the one the gate checked.
        window_ticks = max([3, *(p.ticks for p in stuck.values() if p.decimate == 1)])
        slow_samples: dict[int, int] = {}
        for p in stuck.values():
            if p.decimate > 1:
                slow_samples[p.decimate] = max(slow_samples.get(p.decimate, 0), p.samples)
        layout = ZoneLayout(
            implicit=False,
            zones=zones,
            zone_channels=zone_channels,
            coupled=coupled,
            channel_zones={
                ch: tuple(z for z in zones if ch in zone_channels[z]) for ch in channels
            },
            sensor_zone={t: sensors[t].zone for t in self.temps},
            required_groups=groups,
            reach=reach,
            served={ch: _served_zones(ch, zones, zone_channels, coupled) for ch in channels},
        )
        return _Derived(
            layout=layout,
            stuck=stuck,
            window_ticks=window_ticks,
            slow_samples=dict(sorted(slow_samples.items())),
        )

    # -- derived tick quantities (section 3) ---------------------------------

    @property
    def dT_max_tick(self) -> float:  # noqa: N802 - matches the spec's name
        """Gate slew limit per tick, degrees C: ``dT_max_c_per_s * dt``."""
        return self.dT_max_c_per_s * self.dt

    @property
    def confirm_ticks(self) -> int:
        """Consecutive trusted ticks before ``auto``: ``max(2, ceil(confirm_s / dt))``."""
        return max(2, math.ceil(self.confirm_s / self.dt))

    @property
    def stuck_ticks(self) -> int:
        """Stuck window length in ticks: ``max(2, ceil(stuck_s / dt))``."""
        return max(2, math.ceil(self.stuck_s / self.dt))

    # -- derived zone layout and Stuck sizing ---------------------------------

    @property
    def is_das(self) -> bool:
        """``True`` when ``topology`` is present (zoned DAS), ``False`` in legacy mode."""
        return self.topology is not None

    @property
    def zone_layout(self) -> ZoneLayout:
        """The zones (legacy mode: the one implicit zone), see :class:`ZoneLayout`."""
        return self._derived.layout

    def stuck_params(self, name: str) -> StuckParams:
        """Stuck rule parameters of temperature ``name`` (:class:`StuckParams`).

        With ``topology``: window ``max(2, ceil(sensors.<name>.stuck_s / dt))``
        ticks, band ``sensors.<name>.stuck_eps_c``, and a decimation of
        ``stuck_decimate`` or, when that is absent,
        ``max(1, ticks // STUCK_WINDOW_SAMPLES)`` -- so a 1800 s window at
        ``dt = 5`` keeps 60 samples, one every 6 ticks.
        """
        return self._derived.stuck[name]

    @property
    def window_ticks(self) -> int:
        """Length of ``MpcState.window``: ``stuck_ticks`` in legacy mode; with
        ``topology`` the longest window of a sensor that is not decimated (>= 3)."""
        return self._derived.window_ticks

    @property
    def slow_window_samples(self) -> dict[int, int]:
        """Decimated Stuck windows: decimation factor -> stored samples kept (empty in legacy)."""
        return dict(self._derived.slow_samples)

    def bay_class(self, bay: str) -> str:
        """Drive class of ``bay``: its declared ``class`` or ``topology.default_class``."""
        if self.topology is None:
            raise KeyError(bay)
        cls = self.topology.bays[bay].drive_class or self.topology.default_class
        assert cls is not None
        return cls

    @property
    def regulates_drive_limits(self) -> bool:
        """DAS mode that regulates drive limits: ``topology`` present and ``setpoints`` empty.

        A zoned config that still declares ``setpoints`` keeps the per-zone setpoint
        regulation of the zones milestone (``pi`` on ``T - setpoint``).
        """
        return self.topology is not None and not self.setpoints

    def bay_limit(self, bay: str) -> float:
        """Absolute limit of ``bay``: its class ``limit_c``, tightened by ``bays.<bay>.limit_c``."""
        if self.topology is None:
            raise KeyError(bay)
        limit = self.drive_classes[self.bay_class(bay)].limit_c
        own = self.topology.bays[bay].limit_c
        return limit if own is None else min(limit, own)

    def bay_comfort(self, bay: str) -> float:
        """Comfort band of ``bay``: its class ``comfort_c``."""
        return self.drive_classes[self.bay_class(bay)].comfort_c

    def temps_for_channel(self, channel: str) -> tuple[str, ...]:
        """Temperatures (all with setpoints) that ``channel`` controls.

        Resolves the ``channel_temps`` default: every setpoint temperature,
        in ``temps`` order. With ``topology``: the setpoint temperatures whose
        sensor sits in a zone that lists ``channel``.
        """
        if channel not in self.channels:
            raise KeyError(channel)
        if self.channel_temps:
            return self.channel_temps[channel]
        if self.topology is not None:
            zones = self._derived.layout.channel_zones[channel]
            return tuple(
                t for t in self.temps if t in self.setpoints and self.sensors[t].zone in zones
            )
        return tuple(t for t in self.temps if t in self.setpoints)

    def weight_for(self, temp: str) -> float:
        """Tracking weight for a temperature; 1.0 when not listed in ``weights``."""
        return self.weights.get(temp, 1.0)

    # -- (de)serialisation ---------------------------------------------------

    def to_dict(self) -> dict[str, Any]:
        out: dict[str, Any] = {}
        for f in dataclasses.fields(self):
            value = getattr(self, f.name)
            if isinstance(value, Enum):
                value = value.value
            elif isinstance(value, tuple):
                value = list(value)
            elif isinstance(value, dict):
                value = {k: _plain_entry(v) for k, v in value.items()}
            elif hasattr(value, "to_dict"):
                value = value.to_dict()
            out[f.name] = value
        return out

    @classmethod
    def from_mapping(cls, data: Mapping[str, Any]) -> MpcConfig:
        """Build from a YAML-style mapping. Unknown and missing keys are errors."""
        if not isinstance(data, Mapping):
            raise ConfigError(f"mpc section must be a mapping, got {type(data).__name__}")
        known = {f.name: f for f in dataclasses.fields(cls)}
        unknown = sorted(set(data) - set(known))
        if unknown:
            raise ConfigError(f"unknown mpc keys: {unknown}")
        required = [
            f.name
            for f in known.values()
            if f.default is dataclasses.MISSING and f.default_factory is dataclasses.MISSING
        ]
        missing = [name for name in required if name not in data]
        if missing:
            raise ConfigError(f"missing mpc keys: {missing}")
        return cls(**dict(data))
