"""Fan-health drift rules and device health (PROJECT.md section 8 items 79 and 83).

Every status report of an aquaero or a Quadro carries, per output, the speed, the
output duty the device drives, the 12 V rail voltage and the current and power the
fan draws. The hardware adapter hands those out per commanded channel
(:meth:`~aqua_bridge.hw.aquacomputer_adapter.AquacomputerAdapter.fan_readings`) and
puts them in ``PlantObservation.inputs["fans"]``; the controller's own state
(stuck outputs, absent aquabus slots, outputs not in PWM mode, flow sensors)
arrives separately through
:meth:`~aqua_bridge.hw.sources.CompositeSource.device_health` -- or, for a single
``xt6`` adapter, that controller's own
:meth:`~aqua_bridge.hw.aquacomputer_adapter.AquacomputerAdapter.device_health`,
which :meth:`HealthMonitor.device_health` publishes in the same shape.

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
of *live, uninterrupted* readings before it is reported. A channel that misses a
tick (the read failed, its aquabus slot went away) starts every window again: wall
time passing while nothing was measured is not evidence of anything.

The duty window
    An aquabus fan's rpm in the aquaero's status report lags the Quadro's own
    report by several seconds (PROJECT.md section 2), so the two duty-dependent
    rules never judge a reading against the duty of that same tick alone: they
    judge it against the *band* the output duty spanned over the last ``settle_s``
    -- ``[min duty, max duty]`` of that window, mapped through the fan curve. A
    step from 20 % to 100 % widens the band to cover both while the tachometer
    catches up and narrows back to a point once ``settle_s`` of steady duty has
    passed, and a duty that keeps moving is judged against a wider band rather
    than never judged at all. Nothing is judged until ``settle_s`` of readings has
    accumulated (at startup, and again after a gap).

rpm against the fitted curve
    Expected speed is ``rpm_max * phi(duty, deadband, exponent)`` from the
    channel's ``mpc.fan_models`` entry -- the same curve shape
    ``tools/fit_fans.py`` fits from a recording, so the fitted numbers go straight
    into ``fan_models``. A deviation is a speed further than
    ``rpm_tolerance_frac * rpm_max`` below the band's low edge or above its high
    edge. Below ``min_duty`` nothing is judged: inside and just above the deadband
    the curve says little and a stopped fan is normal. A channel whose model is
    not configured (a legacy config with no ``mpc.fans``/``fan_models``) is
    skipped.

the 12 V rail
    The block's voltage outside ``[rail_min_v, rail_max_v]``. This one does not
    depend on the duty at all, so it is judged at every duty and a duty move never
    restarts it -- a rail that sags while the solver is modulating is exactly when
    it matters. A block reporting 0.0 V is not judged: that is what an aquaero's
    empty aquabus slot reads, not a dead rail.

power against the duty
    Only where the device reports power at all: an aquaero reports 0 mA and 0 W
    for its *own* outputs in PWM mode however fast the fan turns, so absence of
    current is no fault there and ``power_reported`` says so per output. Expected
    power is ``count * power_w_at_max * phi(duty) ** power_exponent`` (the fan law,
    with the exponent a config key), taken over the same duty band, and the rule
    fires when the measured power leaves it by more than
    ``power_tolerance_frac``. ``fan_models.<m>.power_w_at_max`` has no default --
    without it the rule is simply off for that model, since the figure depends on
    the fan and nothing may invent one.

Every threshold above is a ``fan_health:`` key with one default declared once in
:class:`FanHealthConfig`, validated there, shown in both example configs and
described in PROJECT.md section 3.
"""

from __future__ import annotations

import dataclasses
import logging
import math
from collections import deque
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
    #: The rpm and power rules judge a reading against the band the output duty
    #: spanned over the last this many seconds, and judge nothing until that much
    #: has accumulated (>= 0). An aquabus fan's rpm in the aquaero's status report
    #: lags by several seconds (PROJECT.md section 2).
    settle_s: float = 15.0
    #: A speed this fraction of the model's ``rpm_max`` outside the band is a deviation.
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

    #: ``(t, duty)`` back to ``settle_s`` before the newest reading, oldest first.
    duty_window: deque[tuple[float, float]] = dataclasses.field(default_factory=deque)
    #: The update counter of the tick this channel was last read on; a channel that
    #: misses one has no live evidence across the gap (see :meth:`forget`).
    tick: int | None = None
    since: dict[str, float] = dataclasses.field(default_factory=dict)
    logged_at: dict[str, float] = dataclasses.field(default_factory=dict)

    def forget(self) -> None:
        """A gap in the readings: nothing held here was established by live data, so
        every window starts again (the rail included -- a deviation that straddles a
        ten-minute outage is not a deviation held for ten minutes)."""
        self.duty_window.clear()
        self.since.clear()

    def forget_duty(self) -> None:
        """This tick carried no output duty: only the rules that need one start again."""
        self.duty_window.clear()
        self.since.pop("rpm", None)
        self.since.pop("power", None)


class HealthMonitor:
    """Turns each tick's fan readings and device health into published health.

    Build one per run and give :meth:`on_tick` to the loop (chained with the
    recorder and the MQTT publisher by
    :func:`aqua_bridge.recorder.chain_on_tick`). ``source`` is the object the loop
    reads from -- a composite, a single ``xt6`` adapter (its one controller is
    published as a one-element ``devices`` list, see :meth:`device_health`) or a
    source with no ``device_health`` at all, such as the simulator, which simply
    contributes none. ``publish`` is called with the merged payload every tick --
    ``Supervisor.set_device_health`` in the daemon.

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
        self._tick = 0
        self.last: dict[str, Any] = {"devices": [], "fans": {}, "problems": [], "ok": True}

    # -- the rules ---------------------------------------------------------

    def _state(self, channel: str) -> _ChannelState:
        state = self._channels.get(channel)
        if state is None:
            state = self._channels[channel] = _ChannelState()
        return state

    def _duty_band(
        self, state: _ChannelState, duty: float, now: float
    ) -> tuple[float, float] | None:
        """The lowest and the highest output duty over the last ``settle_s``, or
        ``None`` while fewer than ``settle_s`` of live readings back it.

        This is what the aquabus lag costs: a reading taken now describes some duty
        of the last few seconds, so it may only be judged against all of them. A
        steady duty collapses the band to a point; a duty that keeps moving widens it
        instead of postponing the judgement for ever.
        """
        window = state.duty_window
        if window and window[-1][0] == now:
            window[-1] = (now, duty)  # one entry per instant, whatever the clock does
        else:
            window.append((now, duty))
        cutoff = now - self.settings.settle_s
        while len(window) >= 2 and window[1][0] <= cutoff:
            window.popleft()
        if window[0][0] > cutoff:
            return None
        duties = [d for _, d in window]
        return min(duties), max(duties)

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
        state = self._state(channel)
        problems: list[str] = []

        # the 12 V rail: judged at any duty and never restarted by a duty move, since
        # it does not depend on one (a rail that sags while the solver modulates is
        # exactly the case worth catching)
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

        if not _finite(duty):
            state.forget_duty()
            self._sustained(channel, "rpm", False, now)
            self._sustained(channel, "power", False, now)
            out["problems"] = problems
            return out
        duty = float(duty)
        band = self._duty_band(state, duty, now)

        target = expected_rpm(self.cfg, channel, duty)
        out["expected_rpm"] = target
        judge = band is not None and duty >= s.min_duty
        if judge and target is not None and _finite(rpm):
            assert band is not None
            spec = self.cfg.fans[channel]
            model = self.cfg.fan_models[spec.model]
            rpm_max = model.rpm_max
            floor = rpm_max * phi(band[0], model.deadband, model.exponent)
            ceiling = rpm_max * phi(band[1], model.deadband, model.exponent)
            slack = s.rpm_tolerance_frac * rpm_max
            speed = float(rpm)
            off = speed < floor - slack or speed > ceiling + slack
            held = self._sustained(channel, "rpm", off, now)
            if held is not None:
                problems.append(
                    f"{channel}: {speed:.0f} rpm at {duty * 100:.0f} % duty, "
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
            assert band is not None
            floor = self._expected_power(channel, band[0]) or 0.0
            ceiling = self._expected_power(channel, band[1]) or 0.0
            power = float(power)
            off = power < floor * (1.0 - s.power_tolerance_frac) or power > ceiling * (
                1.0 + s.power_tolerance_frac
            )
            held = self._sustained(channel, "power", off, now)
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
        """Run the rules over one tick's readings and merge in the device health.

        One call is one tick, whatever it carries: a tick a channel is missing from
        (the read raised and the loop fed a blank observation, or the channel's
        aquabus slot went away) is a gap in that channel's evidence, and every window
        of it starts again on the next live reading. Without that, an outage's wall
        time would count towards a ``*_fault_s`` nothing measured during it.
        """
        self._tick += 1
        fans: dict[str, Any] = {}
        problems: list[str] = []
        if self.settings.enabled:
            for channel in sorted(readings):
                reading = readings[channel]
                if not isinstance(reading, Mapping):
                    continue
                state = self._state(channel)
                if state.tick is not None and state.tick != self._tick - 1:
                    state.forget()
                state.tick = self._tick
                verdict = self.check_channel(channel, reading, now)
                fans[channel] = verdict
                problems.extend(verdict["problems"])
                self._log(channel, verdict["problems"], now)
        devices = self.device_health()
        payload = {
            "devices": devices["devices"],
            "fans": fans,
            "problems": [*devices["problems"], *problems],
        }
        payload["ok"] = not payload["problems"]
        self.last = payload
        return payload

    def device_health(self) -> dict[str, Any]:
        """``{devices, problems}`` from the source, or empty lists without such a source.

        Both shapes a source may answer with are accepted: a
        :class:`~aqua_bridge.hw.sources.CompositeSource` returns ``{devices,
        problems, ok}`` already, while a single
        :class:`~aqua_bridge.hw.aquacomputer_adapter.AquacomputerAdapter` -- what
        ``--source xt6``, the default and what ``deploy/install-pi.sh`` installs,
        hands the loop -- returns that one controller's own dict. It is wrapped into
        a one-element ``devices`` list here, so the same payload reaches
        ``/api/state``, the page and Home Assistant either way.
        """
        getter = getattr(self.source, "device_health", None)
        empty: dict[str, Any] = {"devices": [], "problems": []}
        if not callable(getter):
            return empty
        try:
            health = dict(getter())
        except Exception:  # a diagnostics path must never break a tick
            _LOG.exception("device_health failed")
            return empty
        problems = [str(p) for p in health.get("problems") or ()]
        if "devices" in health:
            devices = [d for d in (health.get("devices") or []) if isinstance(d, Mapping)]
        elif "label" in health or "device" in health:
            devices = [health]  # one controller, answering for itself
        else:
            devices = []
        return {"devices": devices, "problems": problems}

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
