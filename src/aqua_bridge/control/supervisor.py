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
* ``SetBay`` (DAS mode only) overrides ``topology.bays.<bay>.occupied``,
  ``class`` and ``serial`` in the effective config; a ``null`` field restores the
  configured value. The bay and class must exist and the result must be a valid
  config (a serial may be declared on one bay only). This is the owner's
  declaration: ``occupied: false`` removes the bay's constraints like it does in
  the config file, and a declared serial wins over the estimator's association
  by correlation.
* ``Calibrate`` (DAS mode only, PROJECT.md section 8 item 23) records one drive
  temperature measured with a handheld thermometer for one bay -- the stand-in for
  SMART on an enclosure whose drives no agent can read. The bay must exist
  (:class:`IntentInvalid`) and ``drive_temp_c`` must lie in
  :attr:`MpcConfig.calibrate_range` -- ``[estimator.calibrate_min_c,
  estimator.calibrate_max_c]`` narrowed to the gate's ``temp_min_c`` / ``temp_max_c``
  (:class:`IntentInvalid`); the reading is refused as meaningless or unsafe
  (:class:`IntentConflict`, naming the bay and the reason) before the first tick, when
  the bay holds no drive, and when the bay's zone is untrusted or in fault -- a map
  fitted to a reading the gate does not believe would quietly bias every later estimate
  of that bay, and the estimate is what decides how hard the fans run. All of it is
  checked *before* a running identification experiment is ended, so a refused reading
  costs no experiment; an accepted one aborts it like every other human intent. An
  accepted reading is kept for ``estimator.smart_max_age_s`` (the SMART staleness rule)
  and handed to the loop in :attr:`TickPlan.calibrations`, which merges it into
  ``PlantObservation.inputs["calibration"]``; the estimator then treats it exactly like
  a SMART sample of that bay -- one of the 20 fresh samples an accepted calibration
  needs, not a calibration on its own. The refusals are re-checked on every tick that
  offers it, so a reading whose zone has lost the gate's trust meanwhile is held back
  until it goes stale. A second reading for the same bay replaces the first.
* ``SetMode(auto)`` clears every override. ``SetMode(manual)`` gives every
  channel without an override one, seeded from the last applied PWM (or
  ``fallback_pwm`` before anything was applied) so entering manual does
  not move a fan. ``SetMode(mixed)`` keeps the overrides as they are.
* ``ClearOverride(ch)`` removes one override; ``ClearOverride()`` removes
  all. When no override remains the mode becomes ``auto``; when some remain
  in ``manual`` the mode becomes ``mixed``.
* Released channels are handed to the loop once (``TickPlan.released``) so
  it can drop their integrator entries and the solver re-initialises
  bumplessly (its first output equals the override that was on the fan). A
  **human** override is released this way; an identification experiment's channels
  are not (section 8 item 112, below).

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
* **The spin-up floor** (:mod:`aqua_bridge.control.spinup`, PROJECT.md section 8
  item 75) is applied last, to every channel and in every mode, because it only
  ever *raises*: ``max(what was composed above, the floor)``, then the same rate
  limit and clamp. Two things put a channel there -- a kick on a channel that is
  commanded above its stall duty while its tachometer reads nothing, and the hold
  under the siblings of a fan declared failed. Neither can reduce cooling anywhere,
  which is why a fallback command is no longer returned untouched while one is in
  force; with no floor in force (every ordinary tick, and every tick of a legacy
  config, where the rule cannot run at all) ``compose`` behaves exactly as before.
  A floor that raises a channel a running identification experiment drives **aborts
  that experiment** (``spin_up:<channel>``), like any other perturbation it cannot
  see: the experiment plans its levels from the solver's demand, so a channel the
  floor lifts above the planned level is excitation the experiment did not ask for.

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
with a preset. Without setpoints (drive-limit regulation) PI gains and move
penalty are not scaled. A zoned config that still declares setpoints regulates
on them, so it gets the setpoint transform above as well (reviewer fix: the
comfort band alone would leave ``cool`` without effect on its solver).

Identification experiments (DAS plan section 5, :mod:`aqua_bridge.control.ident`)
---------------------------------------------------------------------------------
* ``Ident(start, group|channel)`` needs a DAS config (else :class:`IntentInvalid`),
  an existing target (:class:`IntentInvalid`), ``ident_enabled`` and no running
  experiment (:class:`IntentConflict`), and every precondition of
  :func:`~aqua_bridge.control.ident.check_start` on the last recorded tick
  (:class:`IntentConflict` listing the reasons). ``Ident(stop)`` aborts the
  running experiment with reason ``stop`` (a no-op without one).
* **Control mode during an experiment: ``auto``.** The experiment is an override of
  the supervisor, not of a human: ``control_mode`` stays ``auto``, ``overrides`` and
  ``snapshot().overrides`` hold only human overrides, and the experiment's levels
  are merged into ``TickPlan.overrides`` only, so ``compose`` rate limits, clamps and
  blocks them under fallback exactly like a human override. The precondition "auto
  without overrides" therefore reads the human state and is not blocked by the
  experiment itself, while a human who wants the fans back simply sends any intent.
  ``TickPlan.experiment`` and ``diagnostics["supervisor"]["experiment"]`` carry the
  experiment's status on its ticks only (absent otherwise, so legacy diagnostics
  keep their shape).
* **A rise in the solver's demand wins over the experiment's plan** (``ident_replan``,
  :mod:`~aqua_bridge.control.ident`): an experiment's own override (not a human one)
  is floored in ``compose`` with the solver's command for that tick,
  ``max(override, mpc_cmd.pwm[ch] - ident.planned_dip(experiment, ch, cfg))`` --
  nothing under ``above``, the owner-accepted ``ident_amplitude`` dip under
  ``symmetric``, read off the running experiment's own levels -- before the usual
  rate limit and clamp. The experiment's levels follow the demand from the next tick
  on; this floor closes the one tick in between, so no tick of an experiment puts
  less on a fan than the controller would have. Both the floor and the re-planning
  follow the flag the **running** experiment was started with, so a config rebuilt
  under it cannot take the floor away mid-experiment. With ``ident_replan: false``
  (the Zero W behaviour) neither happens and the levels stay frozen at the base of
  the start.
* Any other intent submitted while an experiment runs aborts it first
  (``human_intent:<kind>``), whether or not that intent is then accepted
  (conservative: the fans go back to the solver).
* After every tick (:meth:`Supervisor.record_tick`) the settle tracker advances and
  a running experiment is checked against the abort list and armed for the next
  tick. **An end (completed or aborted) returns the channels to the solver as it
  found them**: they are *not* put into ``released``, because the solver ran on every
  tick of the experiment and its integrator already holds the command it would have
  given without one. Releasing them would re-initialise the solver bumplessly at the
  PWM on the fan -- the experiment's own level -- and hand a channel released high
  exactly that level (section 8 item 112); the rate limit still moves the fan at most
  ``d_pwm_max`` per tick on the way back. A tick whose applied
  command is the loop's emergency fallback while the solver's command was not
  (``compose`` raised) counts as a tick without a solver command: it aborts with
  ``fallback`` and restarts the settle count. An unexpected error
  in this bookkeeping aborts the experiment (reason ``error``) and never raises.
  Every start and end is logged.
* **The tick a start lands on.** ``plan_tick`` marks the plan pending and
  ``record_tick`` clears it, so a start that arrives between the two -- the loop
  already holds the plan for the next tick and that tick cannot carry the overrides --
  arms offset 0 one tick further out (``ident.start``'s ``skip_ticks``). Without it
  the machine credited the levels to a tick that ran without them (section 8 item 20).
* **Settle timers across a restart.** They are the only part of the experiments that
  survives one: :meth:`Supervisor.ident_settle_snapshot` hands
  ``aqua_bridge.modelstore.ModelPersister`` ``{zone: seconds settled}`` for
  ``model.json``, and on the first zoned tick that carries a store section
  (``diagnostics["store"]``) the tracker takes back what
  :func:`aqua_bridge.control.persist.apply_seed` left of it after subtracting the
  outage. A running experiment still never resumes.
* ``snapshot().extra["experiment"]`` is :func:`~aqua_bridge.control.ident.status`
  (DAS mode only). No running experiment survives a restart.

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
import logging
import threading
import time
from collections.abc import Callable, Mapping
from dataclasses import dataclass, field
from typing import Any

from aqua_bridge.control import ident, spinup
from aqua_bridge.control.intents import (
    INTENT_KINDS,
    Calibrate,
    ClearOverride,
    ControlMode,
    ControlSnapshot,
    Ident,
    Intent,
    IntentConflict,
    IntentInvalid,
    Preset,
    SetBay,
    SetLimit,
    SetMode,
    SetPreset,
    SetPwm,
    SetSetpoint,
)
from aqua_bridge.model import (
    IDENT_AMPLITUDE_MAX,
    ConfigError,
    Mode,
    MpcCommand,
    MpcConfig,
    MpcState,
    PlantObservation,
)

__all__ = [
    "PRESETS",
    "PresetEffect",
    "RuntimeBays",
    "RuntimeLimits",
    "Supervisor",
    "TickPlan",
    "apply_preset",
]

_EPS = 1e-12
_LOG = logging.getLogger("aqua_bridge.supervisor")


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


@dataclass(frozen=True)
class RuntimeBays:
    """Bay declarations set through ``SetBay``: bay -> ``{occupied?, class?, serial?}``."""

    bays: dict[str, dict[str, Any]] = field(default_factory=dict)


def apply_preset(
    cfg: MpcConfig,
    setpoints: Mapping[str, float],
    preset: Preset,
    limits: RuntimeLimits | None = None,
    bays: RuntimeBays | None = None,
) -> MpcConfig:
    """``cfg`` with ``setpoints``, runtime ``limits`` and ``bays`` and the preset transform.

    Legacy configs: setpoint offset, gain and move-penalty scales (``limits`` and
    ``bays`` must be empty). DAS configs: comfort offset and noise weight scale
    plus the runtime limits and bay declarations (module docstring). Raises
    :class:`~aqua_bridge.model.ConfigError` when the result is not a valid
    config (setpoint outside the gate's absolute range, for instance).
    """
    effect = PRESETS[Preset(preset)]
    if cfg.is_das:
        return _apply_das_preset(
            cfg, setpoints, effect, limits or RuntimeLimits(), bays or RuntimeBays()
        )
    if limits is not None and (limits.classes or limits.bays):
        raise ConfigError("drive limits need a DAS config (mpc.topology)")
    if bays is not None and bays.bays:
        raise ConfigError("bay declarations need a DAS config (mpc.topology)")
    return dataclasses.replace(
        cfg,
        setpoints={name: float(v) + effect.setpoint_offset_c for name, v in setpoints.items()},
        pi_kp=cfg.pi_kp * effect.gain_scale,
        pi_ki=cfg.pi_ki * effect.gain_scale,
        weight_dpwm=cfg.weight_dpwm * effect.move_penalty_scale,
    )


_BAY_ATTRS = {"occupied": "occupied", "class": "drive_class", "serial": "serial"}


def _apply_das_preset(
    cfg: MpcConfig,
    setpoints: Mapping[str, float],
    effect: PresetEffect,
    limits: RuntimeLimits,
    declared: RuntimeBays,
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
    bays = {}
    for name, bay in topo.bays.items():
        changes: dict[str, Any] = {
            _BAY_ATTRS[key]: value for key, value in declared.bays.get(name, {}).items()
        }
        if name in limits.bays:
            changes["limit_c"] = float(limits.bays[name])
        bays[name] = dataclasses.replace(bay, **changes) if changes else bay
    out = dataclasses.replace(
        cfg,
        setpoints={name: float(v) for name, v in setpoints.items()},
        drive_classes=classes,
        topology=dataclasses.replace(topo, bays=bays),
        noise=dataclasses.replace(
            cfg.noise, weight_noise=cfg.noise.weight_noise * effect.noise_weight_scale
        ),
    )
    if not setpoints:
        return out
    # A zoned config that still regulates on setpoints: the solvers read the
    # setpoints, not the comfort bands, so the setpoint semantics must apply too
    # (otherwise ``cool`` would silently not cool).
    return dataclasses.replace(
        out,
        setpoints={name: float(v) + effect.setpoint_offset_c for name, v in setpoints.items()},
        pi_kp=cfg.pi_kp * effect.gain_scale,
        pi_ki=cfg.pi_ki * effect.gain_scale,
        weight_dpwm=cfg.weight_dpwm * effect.move_penalty_scale,
    )


@dataclass(frozen=True)
class TickPlan:
    """Consistent view of the supervisor for one loop tick (taken under the lock).

    The loop runs ``step`` with ``cfg`` and composes with ``overrides``;
    ``released`` names channels whose override was cleared since the last
    plan (drop their integrator entry for a bumpless return to the solver);
    ``calibrations`` are the manual drive readings the loop merges into
    ``PlantObservation.inputs["calibration"]`` before ``step``.
    """

    cfg: MpcConfig
    control_mode: ControlMode
    overrides: dict[str, float] = field(default_factory=dict)
    released: frozenset[str] = frozenset()
    preset: Preset = Preset.NORMAL
    experiment: dict[str, Any] | None = None  # ident.status while an experiment runs
    #: Manual calibrations still fresh (``Calibrate``, item 23):
    #: ``{bay: {"temp_c": float, "age_s": float}}``, the shape
    #: ``PlantObservation.inputs["calibration"]`` carries. Empty in legacy mode.
    calibrations: dict[str, dict[str, float]] = field(default_factory=dict)
    #: Spin-up floors in force this tick (:mod:`aqua_bridge.control.spinup`): the
    #: minimum PWM ``compose`` raises each named channel to -- a kick on a channel that
    #: is commanded but not turning, and the hold under the siblings of a failed fan.
    #: Empty on every ordinary tick, which is why the legacy command path is unchanged.
    spin_up_floor: dict[str, float] = field(default_factory=dict)
    #: ``{"kick": {ch: duty}, "hold": {ch: pwm}}`` behind those floors, for
    #: ``diagnostics["supervisor"]["spin_up"]``; ``None`` while nothing is in force.
    spin_up: dict[str, Any] | None = None


def _clamp(value: float, lo: float, hi: float) -> float:
    return lo if value < lo else hi if value > hi else value


def _experiment_floor(mpc_cmd: MpcCommand, plan: TickPlan, cfg: MpcConfig) -> dict[str, float]:
    """Lowest PWM an experiment channel may be composed to this tick (module docstring).

    Empty without a running experiment and for an experiment that was planned with
    ``ident_replan: false`` -- the running experiment's own flag, not the live config,
    so a config rebuilt under it can neither take the floor away nor add one to a
    frozen plan. The dip comes from that experiment's own levels
    (:func:`~aqua_bridge.control.ident.planned_dip`) for the same reason. Only channels
    the experiment itself drives are floored: a human override of the same channel
    (which aborts the experiment anyway) keeps its value."""
    experiment = plan.experiment
    if experiment is None or not experiment.get("replan"):
        return {}
    levels = experiment.get("overrides")
    if not isinstance(levels, Mapping):
        return {}
    return {
        ch: float(mpc_cmd.pwm[ch]) - ident.planned_dip(experiment, ch, cfg)
        for ch, value in levels.items()
        if ch in cfg.channels and plan.overrides.get(ch) == value and ch in mpc_cmd.pwm
    }


def _spin_floor_of(cmd: MpcCommand | None) -> dict[str, float]:
    """The spin-up floors that were in force on the composed command of this tick.

    Read back out of ``diagnostics["supervisor"]["spin_up"]``, which ``compose`` writes
    exactly when a floor applied, so this is what reached the fans rather than what the
    tracker thought before the tick."""
    if cmd is None or not isinstance(cmd.diagnostics, Mapping):
        return {}
    diag = cmd.diagnostics.get("supervisor")
    spin = diag.get("spin_up") if isinstance(diag, Mapping) else None
    if not isinstance(spin, Mapping):
        return {}
    out: dict[str, float] = {}
    for group in ("kick", "hold"):
        levels = spin.get(group)
        if not isinstance(levels, Mapping):
            continue
        for ch, value in levels.items():
            if isinstance(value, bool) or not isinstance(value, int | float):
                continue
            name = str(ch)
            out[name] = max(out.get(name, float(value)), float(value))
    return out


def _spin_raised(exp: Mapping[str, Any], floor: Mapping[str, float] | None) -> str | None:
    """A channel of ``exp`` that a spin-up floor put above the level it asked for.

    A floor at or below the experiment's own level changed nothing on that channel, so
    it is not a reason to throw an experiment away."""
    if not floor:
        return None
    overrides = exp.get("overrides") or {}
    for ch in exp.get("channels") or ():
        value = floor.get(ch)
        if value is None:
            continue
        want = overrides.get(ch)
        if want is None or float(value) > float(want) + _EPS:
            return str(ch)
    return None


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
        spin_up: spinup.SpinUpConfig | None = None,
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
        self._bays = RuntimeBays()
        #: Manual calibrations (``Calibrate``, item 23): bay -> (monotonic receipt, degC).
        #: In memory only, like every other runtime intent.
        self._calibrations: dict[str, tuple[float, float]] = {}
        self._effective = apply_preset(cfg, self._setpoints, self._preset, self._limits, self._bays)
        self._released: set[str] = set()

        # Identification experiments (module docstring). The running experiment is in
        # memory only; the settle timers come back through the model store.
        self._ident_tracker: dict[str, Any] = ident.new_tracker()
        self._ident_facts: ident.TickFacts | None = None
        self._experiment: dict[str, Any] | None = None
        self._ident_last: dict[str, Any] = {}
        #: The last ``(target, blind channels)`` the unexcitable-channel WARNING named
        #: (section 8 item 110). Most starts on a busy enclosure have one, so the line
        #: is logged when the answer *changes*, not on every start -- the item 108
        #: precedent: a warning repeated on a normal, permitted condition is a warning
        #: the owner learns to skip.
        self._ident_blind: tuple[tuple[str, str], tuple[str, ...]] | None = None
        self._ident_resume_pending = True
        #: A plan has been handed to the loop and its tick is not recorded yet.
        self._plan_pending = False

        # Loop-reported facts.
        self._obs: PlantObservation | None = None
        self._last_cmd: MpcCommand | None = None
        self._mpc_cmd: MpcCommand | None = None
        self._state: MpcState | None = None
        self._applied_pwm: dict[str, float] | None = None
        self._usb_present = False
        self._mqtt_connected: bool | None = None
        self._extra: dict[str, Any] = {}
        #: Last published device / fan health (:mod:`aqua_bridge.health`), empty
        #: until a monitor reports one.
        self._device_health: dict[str, Any] = {}

        # The spin-up kick (:mod:`aqua_bridge.control.spinup`, section 8 item 75). The
        # tracker is advanced from the loop's own report in ``record_tick`` and read in
        # ``plan_tick``; it is in memory only, like every other runtime state, because a
        # fan that does not turn is re-detected within ``confirm_s`` of the next start.
        self._spin_settings = spinup.SpinUpConfig() if spin_up is None else spin_up
        self._spin_tracker: dict[str, Any] = spinup.new_tracker()
        #: ``(channel, state) -> monotonic clock of the last line``, for the rate limit.
        self._spin_logged: dict[tuple[str, str], float] = {}

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

    @property
    def bays(self) -> RuntimeBays:
        """Bay declarations set at runtime (``SetBay``); empty until one is set."""
        with self._lock:
            return RuntimeBays({b: dict(v) for b, v in self._bays.bays.items()})

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

    def _bays_in_force(self) -> dict[str, Any]:
        cfg = self._effective
        if cfg.topology is None:
            return {}
        return {
            name: {
                "zone": bay.zone,
                "occupied": bay.occupied,
                "class": cfg.bay_class(name),
                "serial": bay.serial,
            }
            for name, bay in cfg.topology.bays.items()
        }

    def snapshot(self) -> ControlSnapshot:
        with self._lock:
            state = self._state
            extra = dict(self._extra)
            extra["effective_setpoints"] = dict(self._effective.setpoints)
            extra["applied_pwm"] = None if self._applied_pwm is None else dict(self._applied_pwm)
            extra["mpc_mode"] = None if self._mpc_cmd is None else self._mpc_cmd.mode.value
            if self._base_cfg.is_das:
                extra["experiment"] = ident.status(
                    self._experiment, self._ident_last, self._effective
                )
                # Manual calibrations still pending (item 23), so a client sees that a
                # reading was accepted before the next tick folds it into the estimator.
                extra["calibrations"] = self._fresh_calibrations()
            # Step budget alarm (control/loop.py): loop-reported via record_tick's
            # extra, promoted to dedicated ControlSnapshot fields, not duplicated here.
            step_ms_last = extra.pop("step_ms_last", 0.0)
            step_ms_max = extra.pop("step_ms_max", 0.0)
            budget_warn_count = extra.pop("budget_warn_count", 0)
            budget_alarm_count = extra.pop("budget_alarm_count", 0)
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
                device_health=dict(self._device_health),
                limits=self._limits_in_force(),
                bays=self._bays_in_force(),
                step_ms_last=step_ms_last,
                step_ms_max=step_ms_max,
                budget_warn_count=budget_warn_count,
                budget_alarm_count=budget_alarm_count,
            )

    # -- intents ----------------------------------------------------------

    @property
    def experiment(self) -> dict[str, Any] | None:
        """The running experiment (plain JSON copy), ``None`` when none runs."""
        with self._lock:
            return None if self._experiment is None else dict(self._experiment)

    def submit(self, intent: Intent) -> None:
        with self._lock:
            if isinstance(intent, Ident):
                self._ident(intent)
                return
            # A calibration is checked before the experiment is touched: it is the one
            # command whose payload a human copies off a thermometer display, so a
            # refusal (a typo, an untrusted zone) is the likely case and must not cost
            # a running identification run (item 23).
            checked = self._check_calibrate(intent) if isinstance(intent, Calibrate) else None
            if self._experiment is not None:
                kind = next(
                    (k for k, (cls, _) in INTENT_KINDS.items() if isinstance(intent, cls)),
                    type(intent).__name__,
                )
                self._end_experiment(ident.RESULT_ABORTED, f"human_intent:{kind}")
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
            elif isinstance(intent, SetBay):
                self._set_bay(intent)
            elif isinstance(intent, Calibrate):
                assert checked is not None  # set above for every Calibrate
                self._record_calibration(*checked)
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
        self._effective = self._validated(candidate, self._preset, self._limits, self._bays)
        self._setpoints = candidate

    def _set_preset(self, name: Preset) -> None:
        preset = Preset(name)
        self._effective = self._validated(self._setpoints, preset, self._limits, self._bays)
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
        self._effective = self._validated(self._setpoints, self._preset, candidate, self._bays)
        self._limits = candidate

    def _set_bay(self, intent: SetBay) -> None:
        cfg = self._base_cfg
        topo = cfg.topology
        if topo is None:
            raise IntentInvalid("bay declarations need a DAS config (mpc.topology)")
        if intent.bay not in topo.bays:
            raise IntentInvalid(f"unknown bay {intent.bay!r}; bays are {sorted(topo.bays)}")
        drive_class = intent.changes.get("class")
        if drive_class is not None and drive_class not in cfg.drive_classes:
            raise IntentInvalid(
                f"unknown drive class {drive_class!r}; classes are {sorted(cfg.drive_classes)}"
            )
        current = dict(self._bays.bays.get(intent.bay, {}))
        for key, value in intent.changes.items():
            if value is None:
                current.pop(key, None)
            else:
                current[key] = value
        declared = {b: dict(v) for b, v in self._bays.bays.items() if b != intent.bay}
        if current:
            declared[intent.bay] = current
        candidate = RuntimeBays(declared)
        self._effective = self._validated(self._setpoints, self._preset, self._limits, candidate)
        self._bays = candidate

    def _check_calibrate(self, intent: Calibrate) -> tuple[str, float, float]:
        """Check one handheld drive reading (module docstring, item 23); no state changes.

        Returns ``(bay, drive_temp_c, sample ts)`` or raises. Called before ``submit``
        ends a running experiment, so a refused reading costs nothing.
        """
        cfg = self._effective
        topo = cfg.topology
        rng = cfg.calibrate_range
        if topo is None or rng is None or not cfg.is_das:
            raise IntentInvalid(
                "manual calibration needs a DAS config (mpc.topology): a legacy config has "
                "no drive estimates to calibrate"
            )
        bay = intent.bay
        if bay not in topo.bays:
            raise IntentInvalid(f"unknown bay {bay!r}; bays are {sorted(topo.bays)}")
        temp = intent.drive_temp_c
        if not rng[0] <= temp <= rng[1]:
            raise IntentInvalid(
                f"drive_temp_c {temp} for bay {bay!r} must lie in [{rng[0]}, {rng[1]}] "
                "(mpc.estimator.calibrate_min_c / calibrate_max_c, narrowed to the gate's "
                "mpc.temp_min_c / temp_max_c)"
            )
        refusal = self._calibration_refusal(bay, topo.bays[bay].zone)
        if refusal is not None:
            raise IntentConflict(refusal)
        facts = self._ident_facts
        assert facts is not None and facts.ts is not None  # _calibration_refusal checked it
        return bay, temp, float(facts.ts)

    def _record_calibration(self, bay: str, temp: float, stamp: float) -> None:
        """Keep a checked reading for the loop to offer (``TickPlan.calibrations``)."""
        self._calibrations[bay] = (stamp, temp)
        _LOG.info("manual calibration: bay %s at %.2f degC (ts %.1f)", bay, temp, stamp)

    def _calibration_refusal(self, bay: str, zone: str) -> str | None:
        """Why ``bay`` cannot take a handheld reading right now, ``None`` when it can.

        Read against the last completed tick, the same facts the gate and the
        experiment machine act on. Both doors use it: ``submit`` turns the message
        into an :class:`IntentConflict`, and :meth:`_fresh_calibrations` re-checks it
        every tick, so a reading accepted while the zone was trusted is held back
        instead of fitting a map to a reading the gate has since stopped believing.
        """
        facts = self._ident_facts
        if facts is None or facts.mode is None or facts.ts is None or not facts.zones:
            return (
                f"no_tick: bay {bay!r} has no estimate yet; the estimator needs one "
                "completed tick before a calibration means anything"
            )
        info = facts.zones.get(zone)
        info = info if isinstance(info, Mapping) else {}
        if info.get("trusted") is not True or info.get("fault") is True:
            reasons = info.get("reasons")
            named = ", ".join(str(r) for r in reasons) if isinstance(reasons, list | tuple) else ""
            return (
                f"untrusted:{zone}: bay {bay!r} sits in zone {zone!r}, which the gate does "
                f"not trust{f' ({named})' if named else ''}; a map fitted to a reading the "
                "gate does not believe would bias every later estimate of this bay"
            )
        bay_info = facts.bays.get(bay)
        bay_info = bay_info if isinstance(bay_info, Mapping) else {}
        if bay_info.get("occupancy") == "empty":
            return (
                f"empty:{bay}: bay {bay!r} holds no drive (occupancy 'empty'), so there is "
                "nothing a drive temperature could calibrate"
            )
        return None

    def _fresh_calibrations(self) -> dict[str, dict[str, float]]:
        """Pending manual readings still within ``estimator.smart_max_age_s``.

        ``{bay: {"temp_c", "ts"}}`` on the observation clock -- the last tick's
        observation time, the same clock the estimator measures a SMART sample's age
        on, so a reading that waits for a tick keeps one fixed sample time instead of
        looking new again every tick. Stale entries are dropped here, so a reading no
        tick ever consumed (a long fallback stretch) expires by the same rule a SMART
        sample does instead of landing in the filter minutes later. A reading whose bay
        has since become uncalibratable (an untrusted zone, an emptied bay) is held
        back, not offered, until it expires -- the refusals of §6 are worth no less on
        the tick that absorbs the reading than on the one that took it. A reading is
        *not* consumed on the first plan: the estimator decides what is new from its
        sample time, exactly as it does for SMART.
        """
        cfg = self._effective
        spec, topo = cfg.estimator, cfg.topology
        facts = self._ident_facts
        now = None if facts is None else facts.ts
        if spec is None or topo is None or not self._calibrations or now is None:
            return {}
        out: dict[str, dict[str, float]] = {}
        kept: dict[str, tuple[float, float]] = {}
        for bay, (stamp, temp) in sorted(self._calibrations.items()):
            if not 0.0 <= float(now) - stamp <= spec.smart_max_age_s:
                continue  # stale: forgotten, as a SMART sample of that age would be
            kept[bay] = (stamp, temp)
            entry = topo.bays.get(bay)
            if entry is not None and self._calibration_refusal(bay, entry.zone) is None:
                out[bay] = {"temp_c": temp, "ts": stamp}
        self._calibrations = kept
        return out

    def _ident(self, intent: Ident) -> None:
        cfg = self._effective
        if not cfg.is_das:
            raise IntentInvalid("experiments need a DAS config (mpc.topology)")
        if intent.action == "stop":
            if self._experiment is not None:
                self._end_experiment(ident.RESULT_ABORTED, "stop")
            return
        kind, name = (
            ("group", intent.group)
            if intent.group is not None
            else (
                "channel",
                intent.channel,
            )
        )
        assert name is not None
        try:
            ident.target_channels(cfg, kind, name)
        except KeyError:
            known = sorted(ident.groups(cfg)) if kind == "group" else list(cfg.channels)
            raise IntentInvalid(f"unknown {kind} {name!r}; {kind}s are {known}") from None
        if not cfg.ident_enabled:
            raise IntentConflict("experiments are disabled (mpc.ident_enabled: false)")
        if self._experiment is not None:
            running = self._experiment["target"]
            raise IntentConflict(
                f"an experiment on {running['kind']} {running['name']!r} is already running"
            )
        human = self._control_mode is not ControlMode.AUTO or bool(self._overrides)
        facts = self._ident_facts
        reasons = ident.check_start(
            cfg, self._ident_tracker, facts, kind, name, human_control=human
        )
        if reasons:
            raise IntentConflict(f"cannot start the experiment: {', '.join(reasons)}")
        assert facts is not None
        # A start between plan_tick and record_tick arrives after the loop has taken the
        # plan for the tick that follows ``facts``: that tick cannot carry the overrides,
        # so offset 0 is the tick after it (section 8 item 20).
        self._experiment = ident.start(
            cfg, facts, kind, name, skip_ticks=1 if self._plan_pending else 0
        )
        reach = ident.excitation(cfg, self._experiment["levels"], self._experiment["plan_base"])
        blind = [ch for ch, e in reach.items() if not e["excitable"]]
        _LOG.info(
            "experiment started on %s %r: channels %s, base %s",
            kind,
            name,
            self._experiment["channels"],
            self._experiment["base"],
        )
        seen = ((kind, name), tuple(blind))
        if blind and seen != self._ident_blind:
            # section 8 item 110: pe_min is relative, so a channel parked high cannot
            # reach it at this amplitude. Say so instead of leaving the zone pending for
            # ever with nothing in the log, and say which remedy applies -- a larger
            # amplitude, or waiting for the solver to park the channel lower.
            capped = [ch for ch in blind if not reach[ch]["excitable_at_cap"]]
            _LOG.warning(
                "experiment on %s %r: %s cannot reach the model's PE bound from this "
                "base at mpc.ident_amplitude %.3f; the air block of any zone they serve "
                "cannot converge while they sit there. %s",
                kind,
                name,
                ", ".join(blind),
                cfg.ident_amplitude,
                (
                    f"{', '.join(capped)}: no amplitude up to the {IDENT_AMPLITUDE_MAX} "
                    "cap reaches it either, only a lower park does"
                    if capped
                    else "A larger mpc.ident_amplitude would reach it"
                ),
            )
        self._ident_blind = seen if blind else None
        if blind and not ident.holds_cover_window(cfg):
            _LOG.info(
                "experiment on %s %r: mpc.ident_hold_s %s has a hold shorter than "
                "mpc.model_window_s %.0f s, so a regression window averages two levels "
                "and the monitor reads less than the reach above (an upper bound)",
                kind,
                name,
                list(cfg.ident_hold_s),
                cfg.model_window_s,
            )

    def _end_experiment(self, result: str, reason: str | None) -> None:
        """End the running experiment: drop its overrides and remember how it went.

        The channels are **not** released (section 8 item 112). Releasing one drops its
        integrator entry, which makes ``step`` re-initialise the solver bumplessly at the
        PWM on the fan -- the experiment's own level -- and a channel released high was
        then handed that level as the solver's starting point. The solver ran on every
        tick of the experiment (the levels are an override applied after it), so the
        integrator kept here is the solver's own state, advanced normally throughout and
        never re-seeded from the fan; the release returns control near the solver's own
        demand instead of at the experiment's level, and ``step``'s rate limit still
        moves the fan at most ``d_pwm_max`` per tick.

        *Near*, not bit-exact, under the DAS MPC: its first block is solved with
        ``weight_dpwm`` against the PWM in force, which during the experiment is the
        experiment's own level, so the stored core carries a small residual pull toward
        it -- in the safe direction, more cooling than the counterfactual -- and
        ``_replay_core`` hands the same plan back for up to ``mpc_every_ticks`` ticks
        after the release. Raising either knob makes that residue larger. On the PI-DAS
        branch there is no such term and the integrator is exact.

        A human override of the same channel is unaffected: it is released by
        :meth:`_clear_override` as before."""
        exp = self._experiment
        if exp is None:
            return
        self._experiment = None
        self._ident_last = {"result": result, "reason": reason, "target": dict(exp["target"])}
        if result == ident.RESULT_COMPLETED:
            _LOG.info("experiment on %s %r completed", exp["target"]["kind"], exp["target"]["name"])
        else:
            _LOG.warning(
                "experiment on %s %r aborted: %s",
                exp["target"]["kind"],
                exp["target"]["name"],
                reason,
            )

    def _validated(
        self,
        setpoints: Mapping[str, float],
        preset: Preset,
        limits: RuntimeLimits,
        bays: RuntimeBays,
    ) -> MpcConfig:
        try:
            return apply_preset(self._base_cfg, setpoints, preset, limits, bays)
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
            self._plan_pending = True
            released = frozenset(self._released)
            self._released.clear()
            overrides = dict(self._overrides)
            experiment = None
            if self._experiment is not None:
                overrides = {**self._experiment["overrides"], **overrides}
                experiment = ident.status(self._experiment, self._ident_last, self._effective)
            spin = spinup.floors(self._spin_tracker, self._effective, self._spin_settings)
            return TickPlan(
                cfg=self._effective,
                control_mode=self._control_mode,
                overrides=overrides,
                released=released,
                preset=self._preset,
                experiment=experiment,
                calibrations=self._fresh_calibrations(),
                spin_up_floor=spin.combined(),
                spin_up=spin.to_dict() if spin else None,
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
        if plan.experiment is not None:
            supervisor_diag["experiment"] = plan.experiment
        spin_floor = plan.spin_up_floor
        if spin_floor and plan.spin_up is not None:
            supervisor_diag["spin_up"] = plan.spin_up
        diagnostics = dict(mpc_cmd.diagnostics)
        overrides_apply = mpc_cmd.mode is not Mode.FALLBACK and bool(plan.overrides)
        if not overrides_apply and not spin_floor:
            # Fallback wins over manual: the controller is blind and must not
            # let a human-pinned low duty reduce cooling. No override -> solver.
            diagnostics["supervisor"] = supervisor_diag
            return MpcCommand(pwm=dict(mpc_cmd.pwm), mode=mpc_cmd.mode, diagnostics=diagnostics)

        blocked = _fallback_channels(mpc_cmd, cfg)
        floor = _experiment_floor(mpc_cmd, plan, cfg)
        pwm: dict[str, float] = {}
        limited: dict[str, bool] = {}
        applied_any = False
        for ch in cfg.channels:
            overridden = overrides_apply and ch in plan.overrides and ch not in blocked
            if overridden:
                applied_any = True
                want = float(plan.overrides[ch])
                if ch in floor:
                    want = max(want, floor[ch])
            else:
                want = float(mpc_cmd.pwm[ch])
            if ch in spin_floor:
                # The spin-up floor is last and applies in every mode: it only raises
                # (:mod:`aqua_bridge.control.spinup`), so it can reduce cooling nowhere.
                want = max(want, spin_floor[ch])
            elif not overridden:
                pwm[ch] = want  # the solver's own value, already rate limited and clamped
                continue
            prev = float(prev_pwm[ch])
            moved = _clamp(want, prev - cfg.d_pwm_max, prev + cfg.d_pwm_max)
            if overridden:
                limited[ch] = abs(moved - want) > _EPS
            pwm[ch] = _clamp(moved, cfg.pwm_min, cfg.pwm_max)
        supervisor_diag["overrides_applied"] = applied_any
        if overrides_apply:
            supervisor_diag["override_rate_limited"] = limited
        if overrides_apply and mpc_cmd.mode is Mode.DEGRADED:
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
        ts: float | None = None,
    ) -> None:
        """The loop reports what happened this tick (for ``snapshot``).

        ``ts`` is the tick's observation clock (the blank observation's on a read
        failure); ``None`` falls back to ``obs.ts``. In DAS mode it also advances the
        experiment bookkeeping (module docstring)."""
        with self._lock:
            self._plan_pending = False
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
            self._spin_tick(obs, cmd, applied=applied, ts=ts)
            if self._base_cfg.is_das:
                # The loop's emergency path (compose or a later stage raised after step)
                # applies a fallback command while mpc_cmd still says what the solver
                # wanted: that tick is a fallback tick for the experiment (reviewer fix).
                solver_cmd = mpc_cmd
                if (
                    cmd is not None
                    and cmd.mode is Mode.FALLBACK
                    and (mpc_cmd is None or mpc_cmd.mode is not Mode.FALLBACK)
                ):
                    solver_cmd = None
                tick_ts = obs.ts if ts is None and obs is not None else ts
                self._ident_tick(solver_cmd, tick_ts, applied, spin_floor=_spin_floor_of(cmd))

    def spin_up_status(self) -> dict[str, Any]:
        """Per channel: what the spin-up rule made of it, for the health payload.

        The shape of :func:`aqua_bridge.control.spinup.status` -- state, attempts, the
        last reading, ``monitored`` and, where the rule is off for a channel, why. A
        purely read side: :class:`aqua_bridge.health.HealthMonitor` publishes it and the
        failed fans among them join the payload's ``problems``."""
        with self._lock:
            return spinup.status(self._spin_tracker)

    def _spin_tick(
        self,
        obs: PlantObservation | None,
        cmd: MpcCommand | None,
        *,
        applied: bool,
        ts: float | None,
    ) -> None:
        """Advance the spin-up tracker on this tick's facts (section 8 item 75).

        ``obs`` is ``None`` on a tick whose read failed and ``applied`` false when the
        write did, and either makes the tick a gap: what the fans did between two
        measurements is not evidence about the rotor. Never raises -- like the
        experiment bookkeeping, a diagnosis must not cost a tick; on an error the
        tracker starts over, which at worst re-runs one confirmation window."""
        try:
            live = obs is not None and cmd is not None and applied
            tick_ts = ts if ts is not None else (obs.ts if obs is not None else None)
            facts = spinup.TickFacts(
                ts=tick_ts,
                duty={} if cmd is None else cmd.pwm,
                rpm={} if obs is None else obs.rpm,
                live=live,
            )
            before = self._spin_tracker
            self._spin_tracker = spinup.advance(before, self._effective, self._spin_settings, facts)
            self._log_spin(before, self._spin_tracker)
        except Exception:  # a diagnosis must never fail the tick
            _LOG.exception("spin-up bookkeeping failed")
            self._spin_tracker = spinup.new_tracker()

    def _log_spin(self, before: Mapping[str, Any], after: Mapping[str, Any]) -> None:
        """One line per channel when it enters a kick or is declared failed.

        Rate limited to ``spin_up.log_interval_s`` per channel and state, on the same
        grounds as the fan-health lines: a warning repeated on every tick of a condition
        the owner already knows about is a warning the owner learns to skip."""
        now = self._clock()
        old = before.get("channels") or {}
        for channel, record in (after.get("channels") or {}).items():
            state = record["state"]
            if state not in (spinup.KICKING, spinup.FAILED):
                continue
            was = old.get(channel)
            if was is not None and was["state"] == state and state != spinup.KICKING:
                continue
            if was is not None and was["state"] == state and was["attempts"] == record["attempts"]:
                continue
            last = self._spin_logged.get((channel, state))
            if last is not None and now - last < self._spin_settings.log_interval_s:
                continue
            self._spin_logged[(channel, state)] = now
            if state == spinup.FAILED:
                _LOG.error("spin-up: %s (PROJECT.md section 8 item 75)", record["reason"])
            else:
                _LOG.warning("spin-up: %s (PROJECT.md section 8 item 75)", record["reason"])

    def ident_settle_snapshot(self) -> dict[str, float]:
        """The experiments' settle timers for the model store: ``{zone: seconds settled}``.

        ``aqua_bridge.__main__.build_model_store`` hands
        ``aqua_bridge.modelstore.ModelPersister`` a callable that reaches this once the
        supervisor exists; empty in legacy mode and before the first zoned tick."""
        with self._lock:
            return ident.settle_snapshot(self._ident_tracker)

    def _ident_tick(
        self,
        mpc_cmd: MpcCommand | None,
        ts: float | None,
        applied: bool,
        *,
        spin_floor: Mapping[str, float] | None = None,
    ) -> None:
        try:
            # only an experiment that re-plans reads the demand: no other tick, and no
            # tick of a frozen plan, pays for the two copies (item 52)
            exp = self._experiment
            wants = bool(exp is not None and exp.get("replan"))
            facts = ident.facts_from_tick(mpc_cmd, ts=ts, applied=applied, with_demand=wants)
            self._ident_facts = facts
            if self._ident_resume_pending and facts.store:
                self._ident_resume_pending = False
                self._ident_tracker = ident.resume_tracker(self._ident_tracker, facts)
            self._ident_tracker = ident.track(self._ident_tracker, self._effective, facts)
            if self._experiment is None:
                return
            # A spin-up floor is a perturbation the experiment cannot see: its facts are
            # the solver command, taken *before* ``compose`` applies the floor, so it
            # would go on believing the channel sits at the level it planned while the
            # fans carry the kick duty and then ramp back down at ``d_pwm_max``. Abort
            # it the way every other perturbation does (section 8 item 75).
            raised = _spin_raised(self._experiment, spin_floor)
            if raised:
                self._end_experiment(ident.RESULT_ABORTED, f"spin_up:{raised}")
                return
            outcome = ident.advance(self._experiment, self._effective, facts)
            if outcome.experiment is not None:
                self._experiment = outcome.experiment
            else:
                self._end_experiment(outcome.result or ident.RESULT_ABORTED, outcome.reason)
        except Exception:  # bookkeeping never fails the tick; the fans go back to the solver
            _LOG.exception("experiment bookkeeping failed")
            self._ident_tracker = ident.new_tracker()
            self._end_experiment(ident.RESULT_ABORTED, "error")

    def set_mqtt_connected(self, connected: bool | None) -> None:
        with self._lock:
            self._mqtt_connected = connected

    def set_device_health(self, health: Mapping[str, Any] | None) -> None:
        """What :class:`aqua_bridge.health.HealthMonitor` found this tick, for
        ``/api/state``, ``/api/health``, the MQTT state blob and the page
        (PROJECT.md section 8 items 79 and 83). Purely a view: nothing in the
        control path reads it, so a monitor that stops reporting only freezes a
        number on the page. The caller hands over ownership -- the monitor builds
        a fresh payload every tick -- so only the top level is copied."""
        with self._lock:
            self._device_health = {} if not health else dict(health)

    def set_usb_present(self, present: bool) -> None:
        with self._lock:
            self._usb_present = bool(present)
