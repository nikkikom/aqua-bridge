"""Control intents (PROJECT.md section 6).

HTTP, MQTT and the Digole UI never write PWM themselves. They build an
*intent*, hand it to a :class:`ControlSurface` (implemented by the glue
loop), and read back a :class:`ControlSnapshot`. The loop turns intents
into :class:`aqua_bridge.model.MpcCommand` through the same ``apply()``
path the solver uses, so every invariant (``pwm_min``/``pwm_max``,
``d_pwm_max``) is enforced in one place.

Error mapping for HTTP:

* :class:`IntentInvalid`  -> 400 / 422 (bad shape, unknown channel, out of range)
* :class:`IntentConflict` -> 409 (legal request, wrong state: raw PWM while ``auto``)

Intent constructors do *structural* validation (types, finite numbers,
``pwm`` in ``[0, 1]``) and raise :class:`IntentInvalid`. Config-dependent
checks (channel exists, ``pwm`` within ``[pwm_min, pwm_max]``) belong to
``ControlSurface.submit``.
"""

from __future__ import annotations

import json
import math
import numbers
from collections.abc import Mapping
from dataclasses import dataclass, field
from enum import StrEnum
from typing import Any, Protocol, runtime_checkable

from aqua_bridge.model import FaultReason, Mode, MpcCommand, PlantObservation

__all__ = [
    "INTENT_KINDS",
    "ClearOverride",
    "ControlMode",
    "ControlSnapshot",
    "ControlSurface",
    "Intent",
    "IntentConflict",
    "IntentError",
    "IntentInvalid",
    "Preset",
    "SetMode",
    "SetPreset",
    "SetPwm",
    "SetSetpoint",
    "SolverStatus",
    "parse_intent",
]


# ---------------------------------------------------------------------------
# Enums and errors
# ---------------------------------------------------------------------------


class ControlMode(StrEnum):
    """Global control mode (``POST /api/mode``)."""

    AUTO = "auto"  # solver drives every channel; raw PWM is rejected (409)
    MANUAL = "manual"  # every channel holds a manual override
    MIXED = "mixed"  # overridden channels manual, the rest solver-driven


class Preset(StrEnum):
    """Solver aggressiveness (``POST /api/preset``)."""

    QUIET = "quiet"
    NORMAL = "normal"
    COOL = "cool"


class SolverStatus(StrEnum):
    """``/api/health`` solver field.

    * ``ok``       -- last command was ``auto`` or ``saturated``
    * ``fallback`` -- last command was ``fallback`` (gate or solver fault active)
    * ``fault``    -- no command at all (no observation yet, read/apply failing)
    """

    OK = "ok"
    FALLBACK = "fallback"
    FAULT = "fault"


class IntentError(Exception):
    """Base class for rejected intents."""


class IntentInvalid(IntentError, ValueError):
    """Malformed or out-of-range intent (HTTP 400/422)."""


class IntentConflict(IntentError):
    """Intent is well-formed but not allowed in the current state (HTTP 409)."""


# ---------------------------------------------------------------------------
# Validation helpers
# ---------------------------------------------------------------------------


def _channel(value: object) -> str:
    if not isinstance(value, str) or not value:
        raise IntentInvalid(f"channel must be a non-empty string, got {value!r}")
    return value


def _finite(name: str, value: object) -> float:
    if isinstance(value, bool) or not isinstance(value, numbers.Real):
        raise IntentInvalid(f"{name} must be a number, got {type(value).__name__}: {value!r}")
    out = float(value)
    if not math.isfinite(out):
        raise IntentInvalid(f"{name} must be finite, got {out!r}")
    return out


def _enum[E: StrEnum](kind: type[E], name: str, value: object) -> E:
    if isinstance(value, kind):
        return value
    if not isinstance(value, str):
        raise IntentInvalid(f"{name} must be a string, got {type(value).__name__}: {value!r}")
    try:
        return kind(value)
    except ValueError as exc:
        raise IntentInvalid(
            f"{name} must be one of {[m.value for m in kind]}, got {value!r}"
        ) from exc


# ---------------------------------------------------------------------------
# Intents (section 6, "Control" table)
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class SetMode:
    """``POST /api/mode`` ``{"mode": "auto"|"manual"|"mixed"}``."""

    mode: ControlMode

    def __post_init__(self) -> None:
        object.__setattr__(self, "mode", _enum(ControlMode, "mode", self.mode))


@dataclass(frozen=True)
class SetSetpoint:
    """``POST /api/setpoint`` ``{"channel": "coolant", "celsius": 35}``.

    ``channel`` is a *temperature* name (a key of ``cfg.setpoints``).
    """

    channel: str
    celsius: float

    def __post_init__(self) -> None:
        object.__setattr__(self, "channel", _channel(self.channel))
        object.__setattr__(self, "celsius", _finite("celsius", self.celsius))


@dataclass(frozen=True)
class SetPwm:
    """``POST /api/pwm`` ``{"channel": "radiator", "pwm": 0.4}``.

    Manual override; implies ``mixed``/``manual``. Raises
    :class:`IntentConflict` from ``submit`` while the mode is ``auto``.
    Construction accepts ``[0, 1]``; ``submit`` narrows to
    ``[pwm_min, pwm_max]``.
    """

    channel: str
    pwm: float

    def __post_init__(self) -> None:
        object.__setattr__(self, "channel", _channel(self.channel))
        pwm = _finite("pwm", self.pwm)
        if not 0.0 <= pwm <= 1.0:
            raise IntentInvalid(f"pwm must be in [0, 1], got {pwm}")
        object.__setattr__(self, "pwm", pwm)


@dataclass(frozen=True)
class SetPreset:
    """``POST /api/preset`` ``{"name": "quiet"|"normal"|"cool"}``."""

    name: Preset

    def __post_init__(self) -> None:
        object.__setattr__(self, "name", _enum(Preset, "name", self.name))


@dataclass(frozen=True)
class ClearOverride:
    """``POST /api/auto`` ``{"channel": "radiator"}`` or ``{}`` (all channels)."""

    channel: str | None = None

    def __post_init__(self) -> None:
        if self.channel is not None:
            object.__setattr__(self, "channel", _channel(self.channel))


Intent = SetMode | SetSetpoint | SetPwm | SetPreset | ClearOverride

# URL tail / MQTT command name -> intent class and the body keys it accepts.
INTENT_KINDS: dict[str, tuple[type, tuple[str, ...]]] = {
    "mode": (SetMode, ("mode",)),
    "setpoint": (SetSetpoint, ("channel", "celsius")),
    "pwm": (SetPwm, ("channel", "pwm")),
    "preset": (SetPreset, ("name",)),
    "auto": (ClearOverride, ("channel",)),
}


def parse_intent(kind: str, body: object) -> Intent:
    """Build an intent from a request body, e.g. ``parse_intent("pwm", json_body)``.

    ``kind`` is the last path segment of the ``/api/...`` route (or the MQTT
    command name). Unknown kind, non-object body, unknown or missing fields
    all raise :class:`IntentInvalid` so callers answer 4xx, never 500.
    """
    if kind not in INTENT_KINDS:
        raise IntentInvalid(f"unknown intent kind {kind!r}")
    cls, allowed = INTENT_KINDS[kind]
    if body is None:
        body = {}
    if not isinstance(body, Mapping):
        raise IntentInvalid(f"body must be a JSON object, got {type(body).__name__}")
    unknown = sorted(str(k) for k in body if k not in allowed)
    if unknown:
        raise IntentInvalid(f"unknown fields for {kind!r}: {unknown}")
    if cls is not ClearOverride:
        missing = [k for k in allowed if k not in body]
        if missing:
            raise IntentInvalid(f"missing fields for {kind!r}: {missing}")
    return cls(**{k: body[k] for k in allowed if k in body})


# ---------------------------------------------------------------------------
# Snapshot and surface
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class ControlSnapshot:
    """Everything ``/api/state`` and ``/api/health`` show, taken atomically.

    ``obs`` / ``last_cmd`` are ``None`` until the loop has run once.
    ``mqtt_connected`` is ``None`` when MQTT is disabled.
    """

    obs: PlantObservation | None
    last_cmd: MpcCommand | None
    control_mode: ControlMode
    setpoints: dict[str, float]
    overrides: dict[str, float]  # manual PWM per channel, only overridden channels
    preset: Preset
    channels: tuple[str, ...]
    temps: tuple[str, ...]
    pwm_min: float
    pwm_max: float
    solver_status: SolverStatus
    fault_reason: FaultReason | None
    fault_since_ts: float | None
    usb_present: bool
    mqtt_connected: bool | None
    uptime_s: float
    version: str = ""
    extra: dict[str, Any] = field(default_factory=dict)  # host stats etc., JSON-serialisable

    @staticmethod
    def solver_status_for(cmd: MpcCommand | None) -> SolverStatus:
        """Derive the health field from the last command (``None`` -> ``fault``)."""
        if cmd is None:
            return SolverStatus.FAULT
        if cmd.mode is Mode.FALLBACK:
            return SolverStatus.FALLBACK
        return SolverStatus.OK

    def state_payload(self) -> dict[str, Any]:
        """``GET /api/state``: observation + last command + mode + setpoints."""
        return {
            "obs": None if self.obs is None else self.obs.to_dict(),
            "cmd": None if self.last_cmd is None else self.last_cmd.to_dict(),
            "mode": self.control_mode.value,
            "preset": self.preset.value,
            "setpoints": dict(self.setpoints),
            "overrides": dict(self.overrides),
            "channels": list(self.channels),
            "temps": list(self.temps),
            "pwm_min": self.pwm_min,
            "pwm_max": self.pwm_max,
        }

    def health_payload(self) -> dict[str, Any]:
        """``GET /api/health``: USB present, MQTT, solver ok/fallback/fault, uptime."""
        return {
            "usb_present": self.usb_present,
            "mqtt_connected": self.mqtt_connected,
            "solver": self.solver_status.value,
            "fault_reason": None if self.fault_reason is None else self.fault_reason.value,
            "fault_since_ts": self.fault_since_ts,
            "uptime_s": self.uptime_s,
            "version": self.version,
        }

    def to_dict(self) -> dict[str, Any]:
        out = self.state_payload()
        out["health"] = self.health_payload()
        out["extra"] = dict(self.extra)
        return out

    def to_json(self) -> str:
        return json.dumps(self.to_dict(), allow_nan=False, separators=(",", ":"))


@runtime_checkable
class ControlSurface(Protocol):
    """The only interface HTTP / MQTT / Digole may use to view and steer the loop."""

    def snapshot(self) -> ControlSnapshot:
        """Current state, consistent at one instant."""
        ...

    def submit(self, intent: Intent) -> None:
        """Apply an intent. Raises IntentInvalid (4xx) or IntentConflict (409)."""
        ...
