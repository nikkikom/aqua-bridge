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
"""

from __future__ import annotations

import dataclasses
import math
import numbers
from collections.abc import Iterable, Mapping
from dataclasses import dataclass, field
from enum import Enum, StrEnum
from typing import Any

__all__ = [
    "ConfigError",
    "FaultReason",
    "Mode",
    "MpcCommand",
    "MpcConfig",
    "MpcState",
    "PlantObservation",
    "SolverKind",
    "WindowSample",
    "is_finite_number",
]


# ---------------------------------------------------------------------------
# Enums and errors
# ---------------------------------------------------------------------------


class Mode(StrEnum):
    """Command mode. ``fallback`` for the whole time a fault cause is active."""

    AUTO = "auto"
    SATURATED = "saturated"
    FALLBACK = "fallback"


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


@dataclass(frozen=True)
class PlantObservation:
    """One raw sample from the plant.

    ``temps``/``rpm``/``pwm`` use *logical* names (``coolant``, ``radiator``,
    ...). Values may be ``None`` or NaN; structure must be sound.
    ``ts`` is monotonic cycle seconds (not wall clock) and must be finite.
    """

    temps: dict[str, float | None]
    rpm: dict[str, float | None]
    pwm: dict[str, float | None]
    ts: float

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

    def to_dict(self) -> dict[str, Any]:
        return {
            "temps": dict(self.temps),
            "rpm": dict(self.rpm),
            "pwm": dict(self.pwm),
            "ts": self.ts,
        }

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

    @classmethod
    def cold(cls) -> MpcState:
        """State before the first tick: nothing known, no fault active."""
        return cls()

    @property
    def in_fault(self) -> bool:
        return self.fault_since_ts is not None

    def to_dict(self) -> dict[str, Any]:
        return {
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
        if not self.setpoints:
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

    def temps_for_channel(self, channel: str) -> tuple[str, ...]:
        """Temperatures (all with setpoints) that ``channel`` controls.

        Resolves the ``channel_temps`` default: every setpoint temperature,
        in ``temps`` order.
        """
        if channel not in self.channels:
            raise KeyError(channel)
        if self.channel_temps:
            return self.channel_temps[channel]
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
                value = {k: (list(v) if isinstance(v, tuple) else v) for k, v in value.items()}
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
