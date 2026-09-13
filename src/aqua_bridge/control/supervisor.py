"""Control supervisor: modes, manual overrides, setpoints, presets (sections 5, 6, 7).

The :class:`Supervisor` is the :class:`~aqua_bridge.control.intents.ControlSurface`
that HTTP, MQTT and the Digole UI talk to. It owns everything a human can
change at runtime and turns the solver's :class:`~aqua_bridge.model.MpcCommand`
into the command that actually goes to the fans (:meth:`Supervisor.compose`).
It never writes PWM itself: the loop calls ``sink.apply`` with what
``compose`` returns, so ``pwm_min``/``pwm_max`` and ``d_pwm_max`` are
enforced on one path for solver and human alike.

Rules (section 6 "Control", section 4.8, section 5 "Modes")
-----------------------------------------------------------
* ``SetPwm`` while the mode is ``auto`` -> :class:`IntentConflict` (HTTP 409).
  The client must ``SetMode(manual|mixed)`` first; a raw PWM never
  silently flips the mode away from auto.
* ``SetPwm`` for an unknown channel, or a PWM outside
  ``[cfg.pwm_min, cfg.pwm_max]`` -> :class:`IntentInvalid` (4xx).
* ``SetSetpoint`` only for a temperature that already has a setpoint in the
  config (``cfg.setpoints``). A temperature in ``cfg.temps`` without a
  setpoint is monitored by the gate but not regulated; giving it a target at
  runtime would change which temperatures the fans chase (``channel_temps``
  defaults to "every setpoint"), so it is rejected as invalid. The value
  must lie strictly inside ``(temp_min_c, temp_max_c)`` (model rule).
* ``SetLimit`` (DAS mode only; a legacy config answers :class:`IntentInvalid`)
  sets the absolute limit of one drive class or one bay. The bay or class must
  exist and ``limit_c`` must lie in ``(temp_min_c, configured limit]`` where the
  configured limit is the class's ``limit_c`` (class) or
  ``MpcConfig.bay_limit`` of the config as written (bay): a runtime limit can
  tighten or restore what the owner wrote, never exceed it. A bay's limit in
  force is ``min(class limit, bay limit)``, so lowering a class also lowers every
  bay of that class. Limits apply on top of the preset.
* ``SetMode(auto)`` clears every override. ``SetMode(manual)`` gives every
  channel without an override one, seeded from the last applied PWM (or
  ``fallback_pwm`` before anything was applied) so entering manual does
  not move a fan. ``SetMode(mixed)`` keeps the overrides as they are.
* ``ClearOverride(ch)`` removes one override; ``ClearOverride()`` removes
  all. When no override remains the mode becomes ``auto``; when some remain
  in ``manual`` the mode becomes ``mixed``.
* Released channels are handed to the loop once (``TickPlan.released``) so
  it can drop their integrator entries and the solver re-initialises
  bumplessly (its first output equals the override that was on the fan).

``compose`` (section 6 "Do not bypass ``d_pwm_max``")
-----------------------------------------------------
* ``mpc_cmd.mode == fallback`` -> the solver command is returned unchanged.
  A fault (untrusted sensors, broken solver) must never reduce cooling, and
  a manual override that pins a fan low would do exactly that while the
  controller is blind. Overrides are kept and resume when the fault clears.
* ``mpc_cmd.mode == degraded`` (zones) -> the same rule per channel: an
  override applies only on a channel that is *not* under fallback policy
  (``mpc_cmd.diagnostics["fallback_channels"]``); on the others the solver
  command stands and the blocked overrides are listed in
  ``diagnostics["supervisor"]["overrides_blocked"]``. A degraded command
  without a readable channel list blocks every override (conservative).
* Otherwise every overridden channel is replaced by its override, rate
  limited to ``|delta| <= d_pwm_max`` against the last *applied* PWM and
  clamped into ``[pwm_min, pwm_max]``; other channels keep the solver's
  value. The mode of the composed command is the solver's mode (there is
  no ``manual`` mode in :class:`~aqua_bridge.model.Mode`); the control mode
  and the overrides are recorded in ``diagnostics["supervisor"]``.

Presets (section 6 ``POST /api/preset``, "MPC aggressiveness")
--------------------------------------------------------------
A preset is a documented transform of the base config, applied by
:meth:`Supervisor.effective_config` before every ``step``; it never edits
the config on disk. See :data:`PRESETS`:

* ``quiet``  -- setpoints ``+2 C``, PI gains ``x0.5``, MPC move penalty ``x2``
* ``normal`` -- the config as written
* ``cool``   -- setpoints ``-2 C``, PI gains ``x2``, MPC move penalty ``x0.5``

DAS mode (``mpc.topology`` present) changes the meaning, plan section 4:

* ``quiet``  -- every class ``comfort_c`` ``-2 C`` (floored at 0), ``noise.weight_noise`` ``x2``
* ``cool``   -- every class ``comfort_c`` ``+2 C``, ``noise.weight_noise`` ``x0.5``

A smaller comfort band raises the soft target ``limit - comfort - k * sigma``
(the fans may run slower); the absolute limit and the hard target never move
with a preset. Setpoints (a zoned config may still carry them) are passed
through unchanged and PI gains and move penalty are not scaled in DAS mode.

The offset applies on top of the user's setpoint; ``snapshot().setpoints``
reports the user's values and ``extra["effective_setpoints"]`` the
offset ones. A preset or setpoint whose effective config fails validation
is rejected as :class:`IntentInvalid` at submit time, so the loop never sees
an invalid config.

Thread safety: every public method takes one re-entrant lock; ``snapshot``
copies, so an HTTP thread and the loop thread may interleave freely.
"""

from __future__ import annotations

import dataclasses
import threading
import time
from collections.abc import Callable, Mapping
from dataclasses import dataclass, field
from typing import Any

from aqua_bridge.control.intents import (
    ClearOverride,
    ControlMode,
    ControlSnapshot,
    Intent,
    IntentConflict,
    IntentInvalid,
    Preset,
    SetLimit,
    SetMode,
    SetPreset,
    SetPwm,
    SetSetpoint,
)
from aqua_bridge.model import (
    ConfigError,
    Mode,
    MpcCommand,
    MpcConfig,
    MpcState,
    PlantObservation,
)

__all__ = ["PRESETS", "PresetEffect", "RuntimeLimits", "Supervisor", "TickPlan", "apply_preset"]

_EPS = 1e-12


@dataclass(frozen=True)
class PresetEffect:
    """What a preset does to the base :class:`MpcConfig` (module docstring)."""

    setpoint_offset_c: float = 0.0  # added to every setpoint (legacy)
    gain_scale: float = 1.0  # multiplies pi_kp and pi_ki (legacy)
    move_penalty_scale: float = 1.0  # multiplies weight_dpwm (MPC solver, legacy)
    comfort_offset_c: float = 0.0  # added to every drive class comfort_c (DAS), floored at 0
    noise_weight_scale: float = 1.0  # multiplies noise.weight_noise (DAS)


PRESETS: dict[Preset, PresetEffect] = {
    Preset.QUIET: PresetEffect(
        setpoint_offset_c=2.0,
        gain_scale=0.5,
        move_penalty_scale=2.0,
        comfort_offset_c=-2.0,
        noise_weight_scale=2.0,
    ),
    Preset.NORMAL: PresetEffect(),
    Preset.COOL: PresetEffect(
        setpoint_offset_c=-2.0,
        gain_scale=2.0,
        move_penalty_scale=0.5,
        comfort_offset_c=2.0,
        noise_weight_scale=0.5,
    ),
}


@dataclass(frozen=True)
class RuntimeLimits:
    """Limits set through ``SetLimit``: drive class -> limit_c and bay -> limit_c."""

    classes: dict[str, float] = field(default_factory=dict)
    bays: dict[str, float] = field(default_factory=dict)


def apply_preset(
    cfg: MpcConfig,
    setpoints: Mapping[str, float],
    preset: Preset,
    limits: RuntimeLimits | None = None,
) -> MpcConfig:
    """``cfg`` with ``setpoints``, runtime ``limits`` and the preset transform applied.

    Legacy configs: setpoint offset, gain and move-penalty scales (``limits``
    must be empty). DAS configs: comfort offset and noise weight scale plus the
    runtime limits (module docstring). Raises
    :class:`~aqua_bridge.model.ConfigError` when the result is not a valid
    config (setpoint outside the gate's absolute range, for instance).
    """
    effect = PRESETS[Preset(preset)]
    if cfg.is_das:
        return _apply_das_preset(cfg, setpoints, effect, limits or RuntimeLimits())
    if limits is not None and (limits.classes or limits.bays):
        raise ConfigError("drive limits need a DAS config (mpc.topology)")
    return dataclasses.replace(
        cfg,
        setpoints={name: float(v) + effect.setpoint_offset_c for name, v in setpoints.items()},
        pi_kp=cfg.pi_kp * effect.gain_scale,
        pi_ki=cfg.pi_ki * effect.gain_scale,
        weight_dpwm=cfg.weight_dpwm * effect.move_penalty_scale,
    )


def _apply_das_preset(
    cfg: MpcConfig, setpoints: Mapping[str, float], effect: PresetEffect, limits: RuntimeLimits
) -> MpcConfig:
    topo = cfg.topology
    assert topo is not None and cfg.noise is not None
    classes = {
        name: dataclasses.replace(
            dc,
            limit_c=float(limits.classes.get(name, dc.limit_c)),
            comfort_c=max(0.0, dc.comfort_c + effect.comfort_offset_c),
        )
        for name, dc in cfg.drive_classes.items()
    }
    bays = {
        name: dataclasses.replace(bay, limit_c=float(limits.bays[name]))
        if name in limits.bays
        else bay
        for name, bay in topo.bays.items()
    }
    return dataclasses.replace(
        cfg,
        setpoints={name: float(v) for name, v in setpoints.items()},
        drive_classes=classes,
        topology=dataclasses.replace(topo, bays=bays),
        noise=dataclasses.replace(
            cfg.noise, weight_noise=cfg.noise.weight_noise * effect.noise_weight_scale
        ),
    )


@dataclass(frozen=True)
class TickPlan:
    """Consistent view of the supervisor for one loop tick (taken under the lock).

    The loop runs ``step`` with ``cfg`` and composes with ``overrides``;
    ``released`` names channels whose override was cleared since the last
    plan (drop their integrator entry for a bumpless return to the solver).
    """

    cfg: MpcConfig
    control_mode: ControlMode
    overrides: dict[str, float] = field(default_factory=dict)
    released: frozenset[str] = frozenset()
    preset: Preset = Preset.NORMAL


def _clamp(value: float, lo: float, hi: float) -> float:
    return lo if value < lo else hi if value > hi else value


def _fallback_channels(cmd: MpcCommand, cfg: MpcConfig) -> frozenset[str]:
    """Channels of ``cmd`` under fallback policy, where no override may apply.

    ``fallback``: every channel. ``degraded``: the list in
    ``diagnostics["fallback_channels"]``, or every channel when it is not a
    list of strings. Other modes: none.
    """
    if cmd.mode is Mode.FALLBACK:
        return frozenset(cfg.channels)
    if cmd.mode is not Mode.DEGRADED:
        return frozenset()
    listed = cmd.diagnostics.get("fallback_channels")
    if not isinstance(listed, list | tuple) or not all(isinstance(ch, str) for ch in listed):
        return frozenset(cfg.channels)
    return frozenset(listed)


class Supervisor:
    """ControlSurface implementation shared by the loop and the publishers."""

    def __init__(
        self,
        cfg: MpcConfig,
        *,
        clock: Callable[[], float] | None = None,
        version: str = "",
        preset: Preset = Preset.NORMAL,
    ) -> None:
        if not isinstance(cfg, MpcConfig):
            raise TypeError(f"cfg must be an MpcConfig, got {type(cfg).__name__}")
        self._lock = threading.RLock()
        self._base_cfg = cfg
        self._clock = clock if clock is not None else time.monotonic
        self._version = version
        self._started_at = self._clock()

        self._control_mode = ControlMode.AUTO
        self._overrides: dict[str, float] = {}
        self._setpoints: dict[str, float] = dict(cfg.setpoints)
        self._preset = Preset(preset)
        self._limits = RuntimeLimits()
        self._effective = apply_preset(cfg, self._setpoints, self._preset, self._limits)
        self._released: set[str] = set()

        # Loop-reported facts.
        self._obs: PlantObservation | None = None
        self._last_cmd: MpcCommand | None = None
        self._mpc_cmd: MpcCommand | None = None
        self._state: MpcState | None = None
        self._applied_pwm: dict[str, float] | None = None
        self._usb_present = False
        self._mqtt_connected: bool | None = None
        self._extra: dict[str, Any] = {}

    # -- read side --------------------------------------------------------

    @property
    def base_config(self) -> MpcConfig:
        return self._base_cfg

    @property
    def control_mode(self) -> ControlMode:
        with self._lock:
            return self._control_mode

    @property
    def preset(self) -> Preset:
        with self._lock:
            return self._preset

    @property
    def overrides(self) -> dict[str, float]:
        with self._lock:
            return dict(self._overrides)

    @property
    def setpoints(self) -> dict[str, float]:
        with self._lock:
            return dict(self._setpoints)

    @property
    def limits(self) -> RuntimeLimits:
        """Limits set at runtime (``SetLimit``); empty until one is set."""
        with self._lock:
            return RuntimeLimits(dict(self._limits.classes), dict(self._limits.bays))

    def effective_config(self) -> MpcConfig:
        """The config ``step`` must run with right now (setpoints, limits + preset)."""
        with self._lock:
            return self._effective

    def _limits_in_force(self) -> dict[str, Any]:
        cfg = self._effective
        if not cfg.is_das or cfg.topology is None:
            return {}
        return {
            "classes": {name: dc.limit_c for name, dc in cfg.drive_classes.items()},
            "bays": {bay: cfg.bay_limit(bay) for bay in cfg.topology.bays},
        }

    def snapshot(self) -> ControlSnapshot:
        with self._lock:
            state = self._state
            extra = dict(self._extra)
            extra["effective_setpoints"] = dict(self._effective.setpoints)
            extra["applied_pwm"] = None if self._applied_pwm is None else dict(self._applied_pwm)
            extra["mpc_mode"] = None if self._mpc_cmd is None else self._mpc_cmd.mode.value
            return ControlSnapshot(
                obs=self._obs,
                last_cmd=self._last_cmd,
                control_mode=self._control_mode,
                setpoints=dict(self._setpoints),
                overrides=dict(self._overrides),
                preset=self._preset,
                channels=tuple(self._base_cfg.channels),
                temps=tuple(self._base_cfg.temps),
                pwm_min=self._base_cfg.pwm_min,
                pwm_max=self._base_cfg.pwm_max,
                solver_status=ControlSnapshot.solver_status_for(self._mpc_cmd),
                fault_reason=None if state is None else state.fault_reason,
                fault_since_ts=None if state is None else state.fault_since_ts,
                usb_present=self._usb_present,
                mqtt_connected=self._mqtt_connected,
                uptime_s=max(0.0, self._clock() - self._started_at),
                version=self._version,
                extra=extra,
                limits=self._limits_in_force(),
            )

    # -- intents ----------------------------------------------------------

    def submit(self, intent: Intent) -> None:
        with self._lock:
            if isinstance(intent, SetMode):
                self._set_mode(intent.mode)
            elif isinstance(intent, SetPwm):
                self._set_pwm(intent.channel, intent.pwm)
            elif isinstance(intent, SetSetpoint):
                self._set_setpoint(intent.channel, intent.celsius)
            elif isinstance(intent, SetPreset):
                self._set_preset(intent.name)
            elif isinstance(intent, SetLimit):
                self._set_limit(intent)
            elif isinstance(intent, ClearOverride):
                self._clear_override(intent.channel)
            else:
                raise IntentInvalid(f"unsupported intent {type(intent).__name__}")

    def _seed_pwm(self, channel: str) -> float:
        """Override value that does not move the fan when entering manual."""
        if self._applied_pwm is not None and channel in self._applied_pwm:
            value = self._applied_pwm[channel]
        else:
            value = self._base_cfg.fallback_pwm[channel]
        return _clamp(float(value), self._base_cfg.pwm_min, self._base_cfg.pwm_max)

    def _set_mode(self, mode: ControlMode) -> None:
        mode = ControlMode(mode)
        if mode is ControlMode.AUTO:
            self._released.update(self._overrides)
            self._overrides.clear()
        elif mode is ControlMode.MANUAL:
            for ch in self._base_cfg.channels:
                self._overrides.setdefault(ch, self._seed_pwm(ch))
        self._control_mode = mode

    def _set_pwm(self, channel: str, pwm: float) -> None:
        cfg = self._base_cfg
        if channel not in cfg.channels:
            raise IntentInvalid(f"unknown channel {channel!r}; channels are {list(cfg.channels)}")
        if not cfg.pwm_min <= pwm <= cfg.pwm_max:
            raise IntentInvalid(f"pwm {pwm} outside [pwm_min={cfg.pwm_min}, pwm_max={cfg.pwm_max}]")
        if self._control_mode is ControlMode.AUTO:
            raise IntentConflict("raw PWM is rejected while mode is auto; POST /api/mode first")
        self._overrides[channel] = float(pwm)
        self._released.discard(channel)

    def _set_setpoint(self, temp: str, celsius: float) -> None:
        cfg = self._base_cfg
        if temp not in cfg.temps:
            raise IntentInvalid(f"unknown temperature {temp!r}; temps are {list(cfg.temps)}")
        if temp not in cfg.setpoints:
            raise IntentInvalid(
                f"temperature {temp!r} has no setpoint in the config and cannot be regulated"
            )
        candidate = dict(self._setpoints)
        candidate[temp] = float(celsius)
        self._effective = self._validated(candidate, self._preset, self._limits)
        self._setpoints = candidate

    def _set_preset(self, name: Preset) -> None:
        preset = Preset(name)
        self._effective = self._validated(self._setpoints, preset, self._limits)
        self._preset = preset

    def _set_limit(self, intent: SetLimit) -> None:
        cfg = self._base_cfg
        topo = cfg.topology
        if topo is None:
            raise IntentInvalid(
                "limits need a DAS config (mpc.topology); a legacy config uses /api/setpoint"
            )
        classes = dict(self._limits.classes)
        bays = dict(self._limits.bays)
        if intent.drive_class is not None:
            name = intent.drive_class
            if name not in cfg.drive_classes:
                raise IntentInvalid(
                    f"unknown drive class {name!r}; classes are {sorted(cfg.drive_classes)}"
                )
            ceiling = cfg.drive_classes[name].limit_c
            what = f"class {name!r}"
            classes[name] = intent.limit_c
        else:
            name = str(intent.bay)
            if name not in topo.bays:
                raise IntentInvalid(f"unknown bay {name!r}; bays are {sorted(topo.bays)}")
            ceiling = cfg.bay_limit(name)
            what = f"bay {name!r}"
            bays[name] = intent.limit_c
        if not cfg.temp_min_c < intent.limit_c <= ceiling:
            raise IntentInvalid(
                f"limit_c {intent.limit_c} for {what} must lie in "
                f"({cfg.temp_min_c}, {ceiling}]: a runtime limit cannot exceed the configured one"
            )
        candidate = RuntimeLimits(classes=classes, bays=bays)
        self._effective = self._validated(self._setpoints, self._preset, candidate)
        self._limits = candidate

    def _validated(
        self, setpoints: Mapping[str, float], preset: Preset, limits: RuntimeLimits
    ) -> MpcConfig:
        try:
            return apply_preset(self._base_cfg, setpoints, preset, limits)
        except ConfigError as exc:
            raise IntentInvalid(f"rejected: {exc}") from exc

    def _clear_override(self, channel: str | None) -> None:
        if channel is None:
            self._released.update(self._overrides)
            self._overrides.clear()
        else:
            if channel not in self._base_cfg.channels:
                raise IntentInvalid(f"unknown channel {channel!r}")
            if self._overrides.pop(channel, None) is not None:
                self._released.add(channel)
        if not self._overrides:
            self._control_mode = ControlMode.AUTO
        elif self._control_mode is ControlMode.MANUAL:
            self._control_mode = ControlMode.MIXED

    # -- loop side --------------------------------------------------------

    def plan_tick(self) -> TickPlan:
        """Atomic view for one tick; consumes the pending ``released`` set."""
        with self._lock:
            released = frozenset(self._released)
            self._released.clear()
            return TickPlan(
                cfg=self._effective,
                control_mode=self._control_mode,
                overrides=dict(self._overrides),
                released=released,
                preset=self._preset,
            )

    def compose(
        self,
        mpc_cmd: MpcCommand,
        plan: TickPlan,
        prev_pwm: Mapping[str, float],
    ) -> MpcCommand:
        """Final command for the sink (module docstring, *compose*).

        ``prev_pwm`` is what the last applied command put on the fans (the
        loop resolves it exactly as ``step`` does for its own rate limit).
        Pure: depends only on its arguments.
        """
        cfg = plan.cfg
        supervisor_diag: dict[str, Any] = {
            "control_mode": plan.control_mode.value,
            "preset": plan.preset.value,
            "overrides": dict(plan.overrides),
            "overrides_applied": False,
        }
        diagnostics = dict(mpc_cmd.diagnostics)
        if mpc_cmd.mode is Mode.FALLBACK or not plan.overrides:
            # Fallback wins over manual: the controller is blind and must not
            # let a human-pinned low duty reduce cooling. No override -> solver.
            diagnostics["supervisor"] = supervisor_diag
            return MpcCommand(pwm=dict(mpc_cmd.pwm), mode=mpc_cmd.mode, diagnostics=diagnostics)

        blocked = _fallback_channels(mpc_cmd, cfg)
        pwm: dict[str, float] = {}
        limited: dict[str, bool] = {}
        applied_any = False
        for ch in cfg.channels:
            if ch in plan.overrides and ch not in blocked:
                applied_any = True
                want = float(plan.overrides[ch])
                prev = float(prev_pwm[ch])
                moved = _clamp(want, prev - cfg.d_pwm_max, prev + cfg.d_pwm_max)
                limited[ch] = abs(moved - want) > _EPS
                pwm[ch] = _clamp(moved, cfg.pwm_min, cfg.pwm_max)
            else:
                pwm[ch] = float(mpc_cmd.pwm[ch])
        supervisor_diag["overrides_applied"] = applied_any
        supervisor_diag["override_rate_limited"] = limited
        if mpc_cmd.mode is Mode.DEGRADED:
            supervisor_diag["overrides_blocked"] = [
                ch for ch in cfg.channels if ch in plan.overrides and ch in blocked
            ]
        diagnostics["supervisor"] = supervisor_diag
        return MpcCommand(pwm=pwm, mode=mpc_cmd.mode, diagnostics=diagnostics)

    def record_tick(
        self,
        *,
        obs: PlantObservation | None,
        mpc_cmd: MpcCommand | None,
        cmd: MpcCommand | None,
        state: MpcState | None,
        applied: bool,
        usb_present: bool,
        extra: Mapping[str, Any] | None = None,
    ) -> None:
        """The loop reports what happened this tick (for ``snapshot``)."""
        with self._lock:
            if obs is not None:
                self._obs = obs
            if mpc_cmd is not None:
                self._mpc_cmd = mpc_cmd
            if cmd is not None:
                self._last_cmd = cmd
                if applied:
                    self._applied_pwm = dict(cmd.pwm)
            if state is not None:
                self._state = state
            self._usb_present = bool(usb_present)
            if extra:
                self._extra.update(extra)

    def set_mqtt_connected(self, connected: bool | None) -> None:
        with self._lock:
            self._mqtt_connected = connected

    def set_usb_present(self, present: bool) -> None:
        with self._lock:
            self._usb_present = bool(present)
