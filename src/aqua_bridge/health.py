"""Fan-health drift rules and device health (PROJECT.md section 8 items 79 and 83).

Every status report of an aquaero or a Quadro carries, per output, the speed, the
output duty the device drives, the 12 V rail voltage and the current and power the
fan draws. The hardware adapter hands those out per commanded channel
(:meth:`~aqua_bridge.hw.aquacomputer_adapter.AquacomputerAdapter.fan_readings`) and
puts them in ``PlantObservation.inputs["fans"]``; the controller's own state
(stuck outputs, absent aquabus slots, outputs not in PWM mode, flow sensors)
arrives separately through
:meth:`~aqua_bridge.hw.sources.CompositeSource.device_health`.

Where this sits
---------------
``inputs`` is exogenous, non-gated data: it never reaches the sensor gate, never
faults anything and never enters ``mpc.step``'s arithmetic, so putting the
readings there records and publishes them without touching a single golden step
(PROJECT.md section 8 item 79 asked for exactly that). The device health does
*not* ride the observation, because a missing aquabus device makes ``read()``
raise -- the tick that most needs the diagnosis would carry none. It comes
straight off the source instead, which answers from its last status report even
while the controller is gone.

:class:`HealthMonitor` is a ``Loop.on_tick`` observer like the recorder and the
MQTT publisher: it runs after the command was applied, never raises, and only
reads. Nothing here changes a duty; a drift is a log line and a published field.

The rules
---------
All three are sustained rules -- a deviation must hold for its own ``*_fault_s``
before it is reported -- and all three ignore a channel for ``settle_s`` after its
output duty moved, because an aquabus fan's rpm in the aquaero's status report
lags the Quadro's own report by several seconds (PROJECT.md section 2) and a step
would otherwise read as drift.

rpm against the fitted curve
    Expected speed is ``rpm_max * phi(duty, deadband, exponent)`` from the
    channel's ``mpc.fan_models`` entry -- the same curve shape
    ``tools/fit_fans.py`` fits from a recording, so the fitted numbers go straight
    into ``fan_models``. A deviation is ``|rpm - expected| > rpm_tolerance_frac *
    rpm_max``. Below ``min_duty`` nothing is judged: inside and just above the
    deadband the curve says little and a stopped fan is normal. A channel whose
    model is not configured (a legacy config with no ``mpc.fans``/``fan_models``)
    is skipped.

the 12 V rail
    The block's voltage outside ``[rail_min_v, rail_max_v]``. A block reporting
    0.0 V is not judged: that is what an aquaero's empty aquabus slot reads, not a
    dead rail.

power against the duty
    Only where the device reports power at all: an aquaero reports 0 mA and 0 W
    for its *own* outputs in PWM mode however fast the fan turns, so absence of
    current is no fault there and ``power_reported`` says so per output. Expected
    power is ``count * power_w_at_max * phi(duty) ** power_exponent`` (the fan law,
    with the exponent a config key), and the rule fires when the measured power
    leaves ``+- power_tolerance_frac`` of it. ``fan_models.<m>.power_w_at_max`` has
    no default -- without it the rule is simply off for that model, since the
    figure depends on the fan and nothing may invent one.

Every threshold above is a ``fan_health:`` key with one default declared once in
:class:`FanHealthConfig`, validated there, shown in both example configs and
described in PROJECT.md section 3.
"""

from __future__ import annotations

import dataclasses
import logging
import math
from collections.abc import Callable, Mapping
from dataclasses import dataclass
from typing import Any

from aqua_bridge.control.thermal import phi
from aqua_bridge.model import ConfigError, MpcConfig

__all__ = ["FANHEALTH_KEYS", "RULES", "FanHealthConfig", "HealthMonitor", "expected_rpm"]

_LOG = logging.getLogger("aqua_bridge.health")

#: Rule names, in the order a channel's problems are reported.
RULES: tuple[str, ...] = ("rail", "rpm", "power")


def _number(name: str, value: Any, *, minimum: float | None, maximum: float | None) -> float:
    if isinstance(value, bool) or not isinstance(value, int | float):
        raise ConfigError(f"{name} must be a number, got {value!r}")
    out = float(value)
    if not math.isfinite(out):
        raise ConfigError(f"{name} must be finite, got {value!r}")
    if minimum is not None and out < minimum:
        raise ConfigError(f"{name} must be >= {minimum:g}, got {out:g}")
    if maximum is not None and out > maximum:
        raise ConfigError(f"{name} must be <= {maximum:g}, got {out:g}")
    return out


@dataclass(frozen=True, kw_only=True)
class FanHealthConfig:
    """The ``fan_health:`` section: every drift threshold, with its one default.

    Defaults are deliberately wide: the point of the first release is a visible
    number and a line in the log when a fan really stops or a rail really sags,
    not a tight production alarm. Narrow them once a recording of the real
    enclosure exists (``tools/fit_fans.py`` gives the curve the rpm rule uses).
    """

    #: Run the rules at all. The readings are recorded and published either way.
    enabled: bool = True
    #: Below this output duty no rpm or power rule fires (0..1): inside and just
    #: above a fan's deadband the curve says little.
    min_duty: float = 0.25
    #: After the output duty of a channel moves by more than ``settle_duty``, that
    #: channel is not judged for this long, seconds (>= 0). An aquabus fan's rpm in
    #: the aquaero's status report lags by several seconds (PROJECT.md section 2).
    settle_s: float = 15.0
    #: How far the output duty must move to restart ``settle_s`` (0..1).
    settle_duty: float = 0.02
    #: |rpm - expected| above this fraction of the model's ``rpm_max`` is a deviation.
    rpm_tolerance_frac: float = 0.25
    #: An rpm deviation held this long is reported, seconds (> 0).
    rpm_fault_s: float = 120.0
    #: Low end of the 12 V rail window, volts; a block reading 0.0 V is not judged
    #: (that is what an empty aquabus slot reads, not a dead rail).
    rail_min_v: float = 11.0
    #: High end of the 12 V rail window, volts (> ``rail_min_v``).
    rail_max_v: float = 13.0
    #: A rail excursion held this long is reported, seconds (> 0).
    rail_fault_s: float = 30.0
    #: Measured power outside +- this fraction of the expected power is a deviation.
    power_tolerance_frac: float = 0.5
    #: Expected power is ``power_w_at_max * phi(duty) ** this`` (the fan law, > 0).
    power_exponent: float = 3.0
    #: Expected power below this is not judged, watts (>= 0): the controllers report
    #: power in 0.01 W steps, so a small fan's is mostly quantisation.
    power_min_w: float = 0.2
    #: A power deviation held this long is reported, seconds (> 0).
    power_fault_s: float = 120.0
    #: One log line per channel and rule at most this often, seconds (> 0).
    log_interval_s: float = 300.0

    def __post_init__(self) -> None:
        if not isinstance(self.enabled, bool):
            raise ConfigError(f"fan_health.enabled must be true or false, got {self.enabled!r}")
        _number("fan_health.min_duty", self.min_duty, minimum=0.0, maximum=1.0)
        _number("fan_health.settle_s", self.settle_s, minimum=0.0, maximum=None)
        _number("fan_health.settle_duty", self.settle_duty, minimum=0.0, maximum=1.0)
        _number("fan_health.rpm_tolerance_frac", self.rpm_tolerance_frac, minimum=0.0, maximum=None)
        rail_min = _number("fan_health.rail_min_v", self.rail_min_v, minimum=0.0, maximum=None)
        rail_max = _number("fan_health.rail_max_v", self.rail_max_v, minimum=0.0, maximum=None)
        if rail_max <= rail_min:
            raise ConfigError(
                f"fan_health.rail_max_v ({rail_max:g}) must be above fan_health.rail_min_v "
                f"({rail_min:g})"
            )
        _number(
            "fan_health.power_tolerance_frac", self.power_tolerance_frac, minimum=0.0, maximum=None
        )
        _number("fan_health.power_exponent", self.power_exponent, minimum=1e-9, maximum=None)
        _number("fan_health.power_min_w", self.power_min_w, minimum=0.0, maximum=None)
        for name in ("rpm_fault_s", "rail_fault_s", "power_fault_s", "log_interval_s"):
            _number(f"fan_health.{name}", getattr(self, name), minimum=1e-9, maximum=None)

    @classmethod
    def from_section(cls, section: Mapping[str, Any] | None) -> FanHealthConfig:
        """Build from the raw ``fan_health:`` section; :class:`ConfigError` for an
        unknown key or a bad value, so a misspelt key cannot silently keep its
        default (the same rule every other section follows)."""
        raw = dict(section or {})
        unknown = sorted(str(k) for k in raw if k not in FANHEALTH_KEYS)
        if unknown:
            raise ConfigError(
                f"fan_health: unknown key(s) {unknown}; allowed: {list(FANHEALTH_KEYS)}"
            )
        return cls(**raw)


FANHEALTH_KEYS: tuple[str, ...] = tuple(f.name for f in dataclasses.fields(FanHealthConfig))


def expected_rpm(cfg: MpcConfig, channel: str, duty: float) -> float | None:
    """The fitted curve's speed for ``channel`` at output duty ``duty``, or ``None``
    when the config binds no fan model to it (a legacy config, or a channel left out
    of ``mpc.fans``)."""
    spec = cfg.fans.get(channel)
    if spec is None:
        return None
    model = cfg.fan_models.get(spec.model)
    if model is None or not model.rpm_max > 0.0:
        return None
    return model.rpm_max * phi(duty, model.deadband, model.exponent)


def _finite(value: Any) -> bool:
    return isinstance(value, int | float) and not isinstance(value, bool) and math.isfinite(value)


@dataclass
class _ChannelState:
    """What one channel's rules need to remember between ticks."""

    duty: float | None = None
    settled_at: float | None = None
    since: dict[str, float] = dataclasses.field(default_factory=dict)
    logged_at: dict[str, float] = dataclasses.field(default_factory=dict)


class HealthMonitor:
    """Turns each tick's fan readings and device health into published health.

    Build one per run and give :meth:`on_tick` to the loop (chained with the
    recorder and the MQTT publisher by
    :func:`aqua_bridge.recorder.chain_on_tick`). ``source`` is the object the loop
    reads from; a source without ``device_health`` (the simulator, a single
    ``xt6`` adapter before it is opened) simply contributes none. ``publish`` is
    called with the merged payload every tick -- ``Supervisor.set_device_health``
    in the daemon.

    :meth:`on_tick` never raises: the loop isolates it anyway, and a diagnostics
    bug must not cost a tick.
    """

    def __init__(
        self,
        cfg: MpcConfig,
        settings: FanHealthConfig,
        *,
        source: Any = None,
        publish: Callable[[dict[str, Any]], None] | None = None,
        clock: Callable[[], float] | None = None,
    ) -> None:
        self.cfg = cfg
        self.settings = settings
        self.source = source
        self.publish = publish
        self._clock = clock
        self._channels: dict[str, _ChannelState] = {}
        self.last: dict[str, Any] = {"devices": [], "fans": {}, "problems": [], "ok": True}

    # -- the rules ---------------------------------------------------------

    def _state(self, channel: str) -> _ChannelState:
        state = self._channels.get(channel)
        if state is None:
            state = self._channels[channel] = _ChannelState()
        return state

    def _settled(self, channel: str, duty: float, now: float) -> bool:
        """False while this channel is inside ``settle_s`` of a duty move."""
        s = self.settings
        state = self._state(channel)
        if state.duty is None or abs(duty - state.duty) > s.settle_duty:
            state.duty = duty
            state.settled_at = now + s.settle_s
            state.since.clear()
            return False
        state.duty = duty
        return state.settled_at is None or now >= state.settled_at

    def _sustained(self, channel: str, rule: str, deviating: bool, now: float) -> float | None:
        """Seconds this rule has been deviating once past its window, else ``None``."""
        state = self._state(channel)
        if not deviating:
            state.since.pop(rule, None)
            return None
        start = state.since.setdefault(rule, now)
        held = now - start
        window = {
            "rpm": self.settings.rpm_fault_s,
            "rail": self.settings.rail_fault_s,
            "power": self.settings.power_fault_s,
        }[rule]
        return held if held >= window else None

    def check_channel(self, channel: str, reading: Mapping[str, Any], now: float) -> dict[str, Any]:
        """One channel's verdict: the measured values plus a ``problems`` list."""
        s = self.settings
        duty = reading.get("duty")
        rpm = reading.get("rpm")
        volts = reading.get("voltage_v")
        power = reading.get("power_w")
        out: dict[str, Any] = {
            "duty": duty if _finite(duty) else None,
            "rpm": rpm if _finite(rpm) else None,
            "voltage_v": volts if _finite(volts) else None,
            "current_ma": reading.get("current_ma") if _finite(reading.get("current_ma")) else None,
            "power_w": power if _finite(power) else None,
            "expected_rpm": None,
            "expected_power_w": None,
            "problems": [],
        }
        if not _finite(duty):
            return out
        duty = float(duty)
        settled = self._settled(channel, duty, now)
        problems: list[str] = []

        # the 12 V rail: judged at any duty, since it does not depend on one
        if _finite(volts) and float(volts) > 0.0:
            volts = float(volts)
            low, high = s.rail_min_v, s.rail_max_v
            held = self._sustained(channel, "rail", not low <= volts <= high, now)
            if held is not None:
                problems.append(
                    f"{channel}: the rail reads {volts:.2f} V, outside "
                    f"{low:g}..{high:g} V, for {held:.0f} s"
                )
        else:
            self._sustained(channel, "rail", False, now)

        target = expected_rpm(self.cfg, channel, duty)
        out["expected_rpm"] = target
        judge = settled and duty >= s.min_duty
        if judge and target is not None and _finite(rpm):
            spec = self.cfg.fans[channel]
            rpm_max = self.cfg.fan_models[spec.model].rpm_max
            off = abs(float(rpm) - target)
            held = self._sustained(channel, "rpm", off > s.rpm_tolerance_frac * rpm_max, now)
            if held is not None:
                problems.append(
                    f"{channel}: {float(rpm):.0f} rpm at {duty * 100:.0f} % duty, "
                    f"{target:.0f} rpm expected from the fitted curve, for {held:.0f} s"
                )
        else:
            self._sustained(channel, "rpm", False, now)

        expected_w = self._expected_power(channel, duty)
        out["expected_power_w"] = expected_w
        if (
            judge
            and bool(reading.get("power_reported"))
            and expected_w is not None
            and expected_w >= s.power_min_w
            and _finite(power)
        ):
            power = float(power)
            off = abs(power - expected_w)
            held = self._sustained(channel, "power", off > s.power_tolerance_frac * expected_w, now)
            if held is not None:
                problems.append(
                    f"{channel}: {power:.2f} W at {duty * 100:.0f} % duty, "
                    f"{expected_w:.2f} W expected, for {held:.0f} s"
                )
        else:
            self._sustained(channel, "power", False, now)

        out["problems"] = problems
        return out

    def _expected_power(self, channel: str, duty: float) -> float | None:
        spec = self.cfg.fans.get(channel)
        if spec is None:
            return None
        model = self.cfg.fan_models.get(spec.model)
        if model is None or model.power_w_at_max is None:
            return None
        share = phi(duty, model.deadband, model.exponent) ** self.settings.power_exponent
        return spec.count * model.power_w_at_max * share

    # -- the tick ----------------------------------------------------------

    def update(self, readings: Mapping[str, Any], now: float) -> dict[str, Any]:
        """Run the rules over one tick's readings and merge in the device health."""
        fans: dict[str, Any] = {}
        problems: list[str] = []
        if self.settings.enabled:
            for channel in sorted(readings):
                reading = readings[channel]
                if not isinstance(reading, Mapping):
                    continue
                verdict = self.check_channel(channel, reading, now)
                fans[channel] = verdict
                problems.extend(verdict["problems"])
                self._log(channel, verdict["problems"], now)
        devices = self.device_health()
        payload = {
            "devices": devices.get("devices") or [],
            "fans": fans,
            "problems": [*(devices.get("problems") or []), *problems],
        }
        payload["ok"] = not payload["problems"]
        self.last = payload
        return payload

    def device_health(self) -> dict[str, Any]:
        """The source's own device health, or an empty one without such a source."""
        getter = getattr(self.source, "device_health", None)
        if not callable(getter):
            return {}
        try:
            return dict(getter())
        except Exception:  # a diagnostics path must never break a tick
            _LOG.exception("device_health failed")
            return {}

    def _log(self, channel: str, problems: list[str], now: float) -> None:
        """One warning per channel, rate limited to ``log_interval_s``."""
        if not problems:
            return
        state = self._state(channel)
        last = state.logged_at.get("log")
        if last is not None and now - last < self.settings.log_interval_s:
            return
        state.logged_at["log"] = now
        for text in problems:
            _LOG.warning("fan health: %s (PROJECT.md section 8 item 79)", text)

    def on_tick(self, result: Any = None) -> None:
        """``Loop.on_tick``: read the tick's fan readings, run the rules, publish."""
        try:
            obs = getattr(result, "obs", None)
            inputs = getattr(obs, "inputs", None) or {}
            readings = inputs.get("fans") or {}
            now = self._now(obs)
            payload = self.update(readings, now)
            if self.publish is not None:
                self.publish(payload)
        except Exception:
            _LOG.exception("fan health: this tick was not evaluated")

    def _now(self, obs: Any) -> float:
        if self._clock is not None:
            return float(self._clock())
        ts = getattr(obs, "ts", None)
        return float(ts) if _finite(ts) else 0.0
