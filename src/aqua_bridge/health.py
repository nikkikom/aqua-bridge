"""Fan health, device health and the board's own health (PROJECT.md section 8 items
79, 83 and 103).

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
    skipped -- and says so: its verdict carries ``rpm_monitored`` false with the
    reason in ``unmonitored``, since a channel nothing judges must not publish the
    same empty ``problems`` as a fan found healthy.

the 12 V rail
    The block's voltage outside ``[rail_min_v, rail_max_v]``. This one does not
    depend on the duty at all, so it is judged at every duty and a duty move never
    restarts it -- a rail that sags while the solver is modulating is exactly when
    it matters. A block reporting 0.0 V is not judged: that is what an aquaero's
    empty aquabus slot reads -- an aquabus output with nothing connected, on a bus
    device that is present, reads 0.00 V in the reports that carry that device's
    own measurements -- not a dead rail. That block's verdict then carries
    ``rail_monitored`` false with the reason, like any other rule that did not run:
    0.00 V is the one rail reading this daemon refuses to interpret, so it must not
    publish as one it checked and passed.

    **The rule cannot fire for an aquaero's aquabus outputs at all**, and that is
    deliberate: a populated aquabus block reads the bus device's rail in about one
    report in four and the *aquaero's own* rail in the rest, with nothing in a
    single report to tell them apart (PROJECT.md section 2, 2026-09-17), so the
    adapter publishes ``voltage_v`` as ``None`` there with ``rail_reported``
    false, and a reading with no voltage is skipped like any other. A sagging rail
    behind a bus device is therefore *not* detected here; it would take a device
    that reports its own outputs' rails (the Quadro does, for its own).

    That gap is **accepted and declared**, not hidden (PROJECT.md section 8 item
    117). Closing it needs a controller that reports its own outputs' rails to this
    daemon -- a Quadro on its own USB -- which the supported topology does not have
    (owner decision 2026-09-16), and no substitute signal in the aquaero's report
    survives inspection: the refresh is not even atomic per report (one capture had
    blocks 5 and 6 refreshed and block 8 not), so there is no per-report or
    per-block marker that says which reading a block is carrying. What the daemon
    does instead is say so: every channel's verdict carries ``rpm_monitored``,
    ``rail_monitored`` and ``power_monitored`` with an ``unmonitored`` mapping naming
    each rule that is off for that output and why, so an empty ``problems`` list is
    never readable as coverage the daemon does not have. All three rules declare
    themselves, and each flag means the rule *ran*: what the device does not measure
    and what the config does not describe are both gaps, and the reason says which.
    What *is* still covered on such a system is the aquaero's own outputs 1-4, and
    with them any sag common to the whole 12 V supply; what is not is a rail local
    to the bus device.

power against the duty
    Only where the device reports power at all: an aquaero reports 0 mA and 0 W
    for its *own* outputs in PWM mode however fast the fan turns, so absence of
    current is no fault there and ``power_reported`` says so per output. Expected
    power is ``count * power_w_at_max * phi(duty) ** power_exponent`` (the fan law,
    with the exponent a config key), taken over the same duty band, and the rule
    fires when the measured power leaves it by more than
    ``power_tolerance_frac``. ``fan_models.<m>.power_w_at_max`` has no default --
    without it the rule is simply off for that model, since the figure depends on
    the fan and nothing may invent one, and it is unset in both example configs
    until item 94's measurement. So a measured power is not coverage on its own:
    ``power_monitored`` is true only when the device reports the power *and* the
    channel's model says what to expect, and the missing key is named in
    ``unmonitored`` otherwise -- a seized fan drawing 0 mA at full duty must never
    publish as a power rule that ran and passed.

Every threshold above is a ``fan_health:`` key with one default declared once in
:class:`FanHealthConfig`, validated there, shown in both example configs and
described in PROJECT.md section 3.

The board itself
----------------
:class:`HostHealth` adds three rules about the Raspberry Pi the daemon runs on
(item 103, and the owner decision of 2026-09-16 in PROJECT.md section 8.1): the
board is hot, the board is throttling now, and -- only while the CPU is idle --
the board's temperature diverges from the enclosure air. Its inputs are
:func:`aqua_bridge.hostinfo.collect_hostinfo`'s ``cpu_temp_c``, ``load1`` and
``throttled``, plus the observation's own temperatures as the air reference.

The board is **not** part of the thermal model: about a watt against the drives'
tens of watts, and its reading is dominated by its own self-heating, which moves
with CPU load. It never becomes a solver input, a zone air sensor or a model
node; its verdict rides the same published health payload as the fans' and is
read by nothing that computes a duty. Its thresholds are ``host_health:`` keys,
declared once in :class:`HostHealthConfig` under the same rules.

The two rules that report a fact -- the board is hot, the board is throttling --
join the payload's daemon-wide ``problems``; the divergence rule, which is a hint
about where to look, does not, and shows only on the board's own published
verdict and its Home Assistant ``host_problem`` sensor.

The throttling rule needs a source, and on the board this daemon runs on the
cheapest one is not there: kernel 6.18 exposes no ``get_throttled`` sysfs
attribute, so ``vcgencmd`` is the only thing that knows the whole word.
:func:`host_metrics_reader` therefore gives every reader -- the control loop's
included -- the full source chain (:class:`~aqua_bridge.hostinfo.ThrottledReader`),
with the process rate limited to one run per ``host_health.vcgencmd_interval_s``
and bounded by ``host_health.vcgencmd_timeout_s``. See that function for why 3.3 ms
a minute is affordable on the loop thread and a per-tick fork was not.
"""

from __future__ import annotations

import dataclasses
import functools
import logging
import math
import time
from collections import deque
from collections.abc import Callable, Mapping, Sequence
from dataclasses import dataclass
from typing import Any

from aqua_bridge.control.thermal import phi
from aqua_bridge.hostinfo import (
    THROTTLED_BITS,
    ThrottledReader,
    collect_hostinfo,
    run_vcgencmd,
)
from aqua_bridge.model import ConfigError, MpcConfig

__all__ = [
    "FANHEALTH_KEYS",
    "HOSTHEALTH_KEYS",
    "HOST_RULES",
    "RULES",
    "FanHealthConfig",
    "HealthMonitor",
    "HostHealth",
    "HostHealthConfig",
    "default_air_temps",
    "expected_rpm",
    "host_metrics_reader",
    "validate_host_health",
]

_LOG = logging.getLogger("aqua_bridge.health")

#: Rule names, in the order a channel's problems are reported.
RULES: tuple[str, ...] = ("rail", "rpm", "power")

#: Host-health rule names, in the order the board's problems are reported.
HOST_RULES: tuple[str, ...] = ("temp", "throttled", "divergence")


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
    #: (that is what an empty aquabus slot reads, not a dead rail), and neither is
    #: one with no voltage at all (an aquaero's aquabus outputs, whose rail reading
    #: is the aquaero's own in three reports out of four).
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


#: What a channel's verdict says about a rule it cannot run, when the source gives no
#: reason of its own (a recording made before the reasons existed, a source that is not
#: an Aqua Computer controller). Naming the rule is the point: the verdict must never
#: look like a rule that ran and found nothing (PROJECT.md section 8 item 117).
_UNMONITORED_FALLBACK: Mapping[str, str] = {
    "rpm": "no fitted curve for this channel, so the rpm rule is off for it",
    "rail": "this output reports no rail voltage of its own, so the rail rule is off for it",
    "power": "this output reports no current or power of its own, so the power rule is off for it",
}

#: The rules a verdict declares, in the order the verdict lists them.
_RULES: tuple[str, ...] = ("rpm", "rail", "power")


def _unmonitored(
    reading: Mapping[str, Any],
    monitored: Mapping[str, bool],
    local: Mapping[str, str],
) -> dict[str, str]:
    """``{rule: why it did not run for this channel}`` for the channel's verdict.

    A published verdict has to say which rules it did *not* apply, or a channel whose
    rail is never read looks exactly like a channel whose rail is fine (item 117).
    A rule is off for one of two kinds of reason, and both belong here:

    * the *source* cannot feed it -- the hardware adapter's ``not_measured``, which
      knows why the field is not that output's own measurement, and which wins when
      it has something to say because it is the more fundamental reason;
    * this monitor cannot run it on what it got -- no fitted curve or no
      ``power_w_at_max`` for the channel, or the reading carried no number at all.
      Those reasons are ``local``, written by :meth:`HealthMonitor.check_channel`.

    The generic line above is the last resort, for a source that gives no reason of
    its own (a recording made before the reasons existed, a source that is not an Aqua
    Computer controller).
    """
    reasons = reading.get("not_measured")
    reasons = reasons if isinstance(reasons, Mapping) else {}
    out: dict[str, str] = {}
    for rule in _RULES:
        if monitored.get(rule, True):
            continue
        given = reasons.get(rule)
        if isinstance(given, str) and given:
            out[rule] = given
        else:
            out[rule] = local.get(rule) or _UNMONITORED_FALLBACK[rule]
    return out


# ---------------------------------------------------------------------------
# The board itself (PROJECT.md section 8 item 103)
# ---------------------------------------------------------------------------


@dataclass(frozen=True, kw_only=True)
class HostHealthConfig:
    """The ``host_health:`` section: the board's own thresholds, each with one default.

    The Raspberry Pi is **not** part of the thermal model (owner decision,
    PROJECT.md section 8.1, 2026-09-16): about a watt against the drives' tens of
    watts, and its reading is dominated by its own self-heating. Nothing here is
    ever a solver input, a zone air sensor or a model node; every rule below only
    produces a published field and a log line.
    """

    #: Run the rules at all. The board's temperature and throttling are read,
    #: published and shown either way.
    enabled: bool = True
    #: The board's temperature above this is a deviation, degC. The Pi caps its ARM
    #: clock around 80 degC and hard-throttles around 85; this sits below the first
    #: cap so the warning arrives before the board starts defending itself, and well
    #: above the 47.2 degC the owner's Zero 2 W reads at idle.
    temp_limit_c: float = 75.0
    #: A board temperature above the limit held this long is reported, seconds (> 0).
    #: A build, an update or a log rotation heats the SoC for tens of seconds; two
    #: minutes is placement or airflow, not a burst of work.
    temp_fault_s: float = 120.0
    #: The board's temperature further than this from the enclosure air is a
    #: deviation while the CPU is idle, degC (> 0). A coarse backstop, deliberately:
    #: the owner's Zero 2 W reads 47.2 degC at idle, and an un-heatsinked Zero 2 W
    #: idles roughly 20-25 degC above the air around it, so a *healthy* board is
    #: already 20-25 degC from a room-temperature reference and anything near that
    #: figure would stand permanently tripped. 40 degC clears the self-heating floor
    #: with room to spare and still catches a board in hot exhaust or an air sensor
    #: that stopped tracking. Narrow it only from a measured board-vs-air delta on
    #: the board in its finished place (PROJECT.md section 3, "The board itself").
    #: Judged on the absolute difference: air reading *above* the board is just as
    #: much evidence.
    divergence_c: float = 40.0
    #: A divergence held this long -- idle throughout -- is reported, seconds (> 0).
    #: The enclosure's air moves in minutes, so a quarter of an hour of continuous
    #: idle divergence is not a transient.
    divergence_fault_s: float = 900.0
    #: The CPU counts as idle while ``load1`` is at or below this (>= 0). Four cores,
    #: so below 0.5 the daemon's own tick is the only load and the SoC's self-heating
    #: is at its floor; above it the board's temperature says more about the CPU than
    #: about the air, and the divergence rule is not evidence of anything.
    idle_load1_max: float = 0.5
    #: The temperatures averaged as the enclosure-air reference. Empty means
    #: :func:`default_air_temps`: every ``zone_air`` sensor, else every ``inlet``
    #: sensor, else every configured temperature. Name them here when the board sits
    #: somewhere a different sensor describes better.
    air_temps: tuple[str, ...] = ()
    #: One log line for the board at most this often, seconds (> 0).
    log_interval_s: float = 300.0
    #: How often a host-metrics reader may run ``vcgencmd get_throttled``, seconds
    #: (> 0). On a kernel that exposes no ``get_throttled`` sysfs attribute -- which
    #: is the case on a Raspberry Pi Zero 2 W running kernel 6.18 -- that process is
    #: the only source of the whole word, so it runs on the control-loop thread too;
    #: this is what keeps it to one fork per minute instead of one per 5 s tick. The
    #: last word read is served in between, tagged with its age. One minute is far
    #: below the rate at which any of these conditions comes and goes (the firmware
    #: latches the since-boot half regardless) and costs 3.3 ms of one tick in 60 s.
    #: Lower it to notice throttling sooner, at one 3.3 ms fork per interval.
    vcgencmd_interval_s: float = 60.0
    #: How long one ``vcgencmd get_throttled`` may take, seconds (> 0). It guards the
    #: one way that call can hurt the loop: a VideoCore mailbox that does not answer,
    #: which would otherwise block the thread indefinitely. Two seconds is a round
    #: trip with room for a busy VideoCore; a timeout is simply ``None``, and the
    #: chain falls through to the hwmon under-voltage bit. A board where the call
    #: times out pays that once per ``vcgencmd_interval_s``, not once per tick.
    vcgencmd_timeout_s: float = 2.0

    def __post_init__(self) -> None:
        if not isinstance(self.enabled, bool):
            raise ConfigError(f"host_health.enabled must be true or false, got {self.enabled!r}")
        _number("host_health.temp_limit_c", self.temp_limit_c, minimum=0.0, maximum=None)
        _number("host_health.divergence_c", self.divergence_c, minimum=1e-9, maximum=None)
        _number("host_health.idle_load1_max", self.idle_load1_max, minimum=0.0, maximum=None)
        for name in (
            "temp_fault_s",
            "divergence_fault_s",
            "log_interval_s",
            "vcgencmd_interval_s",
            "vcgencmd_timeout_s",
        ):
            _number(f"host_health.{name}", getattr(self, name), minimum=1e-9, maximum=None)
        if isinstance(self.air_temps, str) or not isinstance(self.air_temps, list | tuple):
            raise ConfigError(
                f"host_health.air_temps must be a list of temperature names, got {self.air_temps!r}"
            )
        for name in self.air_temps:
            if not isinstance(name, str) or not name:
                raise ConfigError(
                    f"host_health.air_temps entries must be non-empty strings, got {name!r}"
                )
        object.__setattr__(self, "air_temps", tuple(self.air_temps))

    @classmethod
    def from_section(cls, section: Mapping[str, Any] | None) -> HostHealthConfig:
        """Build from the raw ``host_health:`` section; :class:`ConfigError` for an
        unknown key or a bad value, the same rule every other section follows."""
        raw = dict(section or {})
        unknown = sorted(str(k) for k in raw if k not in HOSTHEALTH_KEYS)
        if unknown:
            raise ConfigError(
                f"host_health: unknown key(s) {unknown}; allowed: {list(HOSTHEALTH_KEYS)}"
            )
        return cls(**raw)


HOSTHEALTH_KEYS: tuple[str, ...] = tuple(f.name for f in dataclasses.fields(HostHealthConfig))


def default_air_temps(cfg: MpcConfig) -> tuple[str, ...]:
    """The enclosure-air reference when ``host_health.air_temps`` is empty.

    Every ``zone_air`` sensor (the air the drives and the board actually sit in),
    else every ``inlet`` sensor, else -- a legacy config, which declares no roles --
    every configured temperature. The divergence rule is a hint either way, so a
    rough reference is better than none.

    ``cfg.temps`` and ``cfg.sensors`` are read straight off the config: a renamed
    field is an :class:`AttributeError` at startup, not a silently empty reference
    that switches the divergence rule off with nothing said.
    """
    temps = tuple(cfg.temps)
    sensors = cfg.sensors
    for role in ("zone_air", "inlet"):
        named = tuple(t for t in temps if getattr(sensors.get(t), "role", None) == role)
        if named:
            return named
    return temps


def validate_host_health(cfg: MpcConfig, settings: HostHealthConfig) -> tuple[str, ...]:
    """The air reference ``settings`` resolves to, or :class:`ConfigError` for a name
    that is not in ``mpc.temps``.

    The one ``host_health:`` check that needs the controller config, so it cannot
    live in :class:`HostHealthConfig`. Called from ``__main__.main`` before anything
    is opened -- a typo'd sensor name is a startup ``ConfigError`` (exit 2), not a
    traceback out of a half-built daemon holding hidraw handles -- and again from
    :meth:`HealthMonitor.__init__`, which is the only way to build the rules.
    """
    unknown = sorted(set(settings.air_temps) - set(cfg.temps))
    if unknown:
        raise ConfigError(
            f"host_health.air_temps names temperature(s) {unknown} that are not in mpc.temps"
        )
    return tuple(settings.air_temps) or default_air_temps(cfg)


def host_metrics_reader(
    settings: HostHealthConfig | None = None,
    *,
    subprocess_fallback: bool = True,
    clock: Callable[[], float] = time.monotonic,
) -> Callable[[], dict[str, Any]]:
    """:func:`~aqua_bridge.hostinfo.collect_hostinfo` with this config's throttling
    source chain: one :class:`~aqua_bridge.hostinfo.ThrottledReader` per caller.

    Every caller gets the same chain, the control loop's reader included, because the
    board this daemon runs on has no cheaper complete source: on a Raspberry Pi Zero 2
    W running kernel 6.18 the ``get_throttled`` sysfs attribute does not exist, so a
    reader without ``vcgencmd`` sees ``throttled: null`` forever and the "throttling
    now" rule can never fire where it matters -- which is a worse bargain than the
    cost it avoids. Measured on that board, one ``vcgencmd get_throttled`` is 3.3 ms
    median / 3.8 ms p95; at the default one poll per minute against ``dt = 5 s`` that
    is 0.07 % of the tick it lands on, 1.3 % of ``mpc.budget_ms`` (250, alarm 350),
    and nothing at all on the other eleven ticks. ``vcgencmd_timeout_s`` bounds the
    one failure that could cost more than that -- a VideoCore mailbox that never
    answers -- and a failed poll waits out the interval before it is tried again, so
    a board where the call hangs pays one timeout a minute and reads the hwmon
    under-voltage bit in between.

    ``subprocess_fallback=False`` drops ``vcgencmd`` from the chain: the reader then
    does file reads only -- the sysfs attribute where a kernel has it, and the
    ``rpi_volt`` hwmon under-voltage bit -- and starts no process at all.
    """
    s = settings or HostHealthConfig()
    runner = (
        functools.partial(run_vcgencmd, timeout_s=s.vcgencmd_timeout_s)
        if subprocess_fallback
        else None
    )
    throttled = ThrottledReader(vcgencmd=runner, poll_interval_s=s.vcgencmd_interval_s, clock=clock)
    return functools.partial(collect_hostinfo, throttled=throttled)


def _throttled_detail(throttled: Mapping[str, Any]) -> str:
    """What the "throttling now" message says after the conditions it found.

    Always which source the reading came from, so a message can be read against the
    board it came from; the raw word where one was read; how old it is where it was
    served from :class:`~aqua_bridge.hostinfo.ThrottledReader`'s cache rather than
    read this tick; and which conditions the source could not see, so a partial
    reading never looks like a whole one.
    """
    flags = [name for _, name in THROTTLED_BITS if throttled.get(f"{name}_now") is True]
    parts = [", ".join(flags) or "unknown bit"]
    if throttled.get("hex") is not None:
        parts.append(f"get_throttled {throttled['hex']}")
    source = throttled.get("source")
    if source:
        parts.append(f"source {source}")
    age_s = throttled.get("age_s")
    if isinstance(age_s, int | float) and age_s > 0.0:
        parts.append(f"read {float(age_s):.0f} s ago")
    unknown = throttled.get("unknown") or []
    if unknown:
        parts.append(f"{', '.join(str(name) for name in unknown)} not read")
    return "; ".join(parts)


class HostHealth:
    """The board's own temperature and throttling, judged by three rules.

    ``the board is hot``
        Its temperature above ``temp_limit_c`` for ``temp_fault_s``. The board
        throttles itself around 80 degC, and it is the first thing to notice when
        the Pi ends up in hot exhaust.
    ``the board is throttling``
        Any of the *now* bits of ``get_throttled`` -- under-voltage, a capped ARM
        frequency, hard throttling, the soft temperature limit
        (:func:`aqua_bridge.hostinfo.decode_throttled`). Reported the tick it is
        seen, with no window: the firmware has already latched the condition, and a
        sustained window would only delay a fact. The latched *since boot* half is
        published next to it but never warns on its own -- an under-voltage during
        boot is history, not a live problem. It fires on whatever source the board
        has: the whole word, or the ``rpi_volt`` hwmon alarm on its own, which is
        enough to report an under-voltage and says plainly that it saw nothing of
        the other three conditions. A condition nobody read is never a condition
        read as absent, so a partial source can raise this rule but never silence it.
    ``the board diverges from the enclosure air``
        Its temperature further than ``divergence_c`` from the mean of the air
        reference for ``divergence_fault_s``, **and only while the CPU is idle**
        (``load1 <= idle_load1_max``): the board's reading is dominated by its own
        self-heating, which moves with CPU load, so under load the divergence is
        evidence of nothing. Any tick that is not idle, or carries no reading,
        starts the window again. This rule is a **hint, not a verdict**: it says
        that either the air sensors or the board's placement deserve a look, never
        which.

    :meth:`check` reports the first two as ``faults`` and the hint as ``hints``,
    with ``problems`` their concatenation and ``ok`` false for either. Only the
    ``faults`` join the daemon-wide problem list (:meth:`HealthMonitor.update`): a
    hint must not make ``/api/health`` not-ok or turn on the controller-fault sensor
    in Home Assistant, which would read exactly like an aquabus device that has gone
    missing. The board's own ``host_problem`` sensor carries all of it.

    None of this steers anything: the verdict is published (``/api/state``,
    ``/api/health``, the MQTT state blob, the page) and logged, and never enters
    ``PlantObservation`` or the ``diagnostics`` the solver reads.
    """

    def __init__(self, settings: HostHealthConfig, *, air_temps: Sequence[str] = ()) -> None:
        self.settings = settings
        self.air_temps: tuple[str, ...] = tuple(settings.air_temps or air_temps)
        self._since: dict[str, float] = {}
        self._logged_at: float | None = None

    def _sustained(self, rule: str, deviating: bool, now: float) -> float | None:
        """Seconds this rule has been deviating once past its window, else ``None``."""
        if not deviating:
            self._since.pop(rule, None)
            return None
        held = now - self._since.setdefault(rule, now)
        window = {
            "temp": self.settings.temp_fault_s,
            "divergence": self.settings.divergence_fault_s,
        }[rule]
        return held if held >= window else None

    def air_c(self, temps: Mapping[str, Any] | None) -> float | None:
        """The reference air temperature: the mean of the readings that are there."""
        values = [
            float(temps[name])
            for name in self.air_temps
            if temps is not None and _finite(temps.get(name))
        ]
        return sum(values) / len(values) if values else None

    def check(
        self,
        host: Mapping[str, Any] | None,
        temps: Mapping[str, Any] | None,
        now: float,
    ) -> dict[str, Any]:
        """One tick's verdict: the measurements plus a ``problems`` list and ``ok``."""
        s = self.settings
        info = host or {}
        board = info.get("cpu_temp_c")
        board_c = float(board) if _finite(board) else None
        load = info.get("load1")
        load1 = float(load) if _finite(load) else None
        throttled = info.get("throttled")
        throttled = dict(throttled) if isinstance(throttled, Mapping) else None
        air = self.air_c(temps)
        idle = None if load1 is None else load1 <= s.idle_load1_max
        divergence = None if board_c is None or air is None else board_c - air
        out: dict[str, Any] = {
            "cpu_temp_c": board_c,
            "air_c": air,
            "air_temps": list(self.air_temps),
            "divergence_c": divergence,
            "load1": load1,
            "idle": idle,
            "throttled": throttled,
            "faults": [],
            "hints": [],
            "problems": [],
            "ok": True,
        }
        if not s.enabled:
            return out

        faults: list[str] = []
        held = self._sustained("temp", board_c is not None and board_c > s.temp_limit_c, now)
        if held is not None and board_c is not None:
            faults.append(
                f"host: the board reads {board_c:.1f} degC, above the "
                f"{s.temp_limit_c:g} degC limit, for {held:.0f} s"
            )

        # ``now`` is True only where a condition was actually read as in force: a
        # source that saw fewer conditions leaves the ones it could not read None and
        # summarises to None, never to False (hostinfo._summary). So this fires on the
        # hwmon under-voltage bit alone, and an unknown bit is never read as a fine one.
        if throttled is not None and throttled.get("now") is True:
            faults.append(f"host: the board is throttling now ({_throttled_detail(throttled)})")

        hints: list[str] = []
        diverging = idle is True and divergence is not None and abs(divergence) > s.divergence_c
        held = self._sustained("divergence", diverging, now)
        if held is not None and divergence is not None and air is not None:
            hints.append(
                f"host: the board is {divergence:+.1f} degC from the enclosure air "
                f"({air:.1f} degC) at idle, beyond {s.divergence_c:g} degC, for {held:.0f} s "
                f"-- a hint about the air sensors or the board's placement, not a verdict"
            )

        out["faults"] = faults
        out["hints"] = hints
        out["problems"] = [*faults, *hints]
        out["ok"] = not out["problems"]
        self._log(out["problems"], now)
        return out

    def _log(self, problems: list[str], now: float) -> None:
        """One warning for the board, rate limited to ``log_interval_s``."""
        if not problems:
            return
        if self._logged_at is not None and now - self._logged_at < self.settings.log_interval_s:
            return
        self._logged_at = now
        for text in problems:
            _LOG.warning("host health: %s (PROJECT.md section 8 item 103)", text)


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

    ``hostinfo`` is the host-metrics reader the board's own rules judge
    (:class:`HostHealth`, item 103): a
    :class:`~aqua_bridge.hostinfo.CachedHostInfo` bound to ``host.interval_s`` in
    the daemon, ``None`` in a test or a run that wants no host health. Its verdict
    is the payload's ``host`` key and its problems join the payload's ``problems``,
    so the one ``ok`` a Home Assistant problem sensor reads covers the board too.

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
        host_settings: HostHealthConfig | None = None,
        hostinfo: Callable[[], Mapping[str, Any]] | None = None,
    ) -> None:
        self.cfg = cfg
        self.settings = settings
        self.source = source
        self.publish = publish
        self._clock = clock
        self._channels: dict[str, _ChannelState] = {}
        self._tick = 0
        self.host_settings = host_settings or HostHealthConfig()
        # The same check ``__main__.main`` runs before anything is opened, so a bad
        # air_temps name never gets this far in the daemon (:func:`validate_host_health`).
        air_temps = validate_host_health(cfg, self.host_settings)
        self.host = HostHealth(self.host_settings, air_temps=air_temps)
        self._hostinfo = hostinfo
        self.last: dict[str, Any] = {
            "devices": [],
            "fans": {},
            "host": {},
            "problems": [],
            "ok": True,
        }

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
        """One channel's verdict: the measured values, a ``problems`` list, and which
        rules did not run at all.

        ``rpm_monitored`` / ``rail_monitored`` / ``power_monitored`` and the
        ``unmonitored`` mapping are the second half of the answer (PROJECT.md section 8
        item 117): an empty ``problems`` on a channel whose rail is never read must not
        be readable as "the rail is fine", so the verdict names each rule that is off
        for this output and why, in the same payload, wherever a human or Home
        Assistant reads it.

        A flag says whether **the rule could run on this reading**, not whether the
        device measures the field: a rule needs the measurement *and* the configuration
        it judges it against, so an output that reports power with no
        ``fan_models.<m>.power_w_at_max`` behind it is ``power_monitored`` false just
        like an output that reports none. What the flags do not carry is the two
        transient conditions of the duty-dependent rules -- the ``settle_s`` window
        still filling after a gap, and a duty below ``min_duty`` -- which hold off a
        rule for a tick or two rather than for the channel; those are in the module
        docstring, and neither is a coverage gap.
        """
        s = self.settings
        duty = reading.get("duty")
        rpm = reading.get("rpm")
        volts = reading.get("voltage_v")
        power = reading.get("power_w")
        # A source that says the voltage field is not that output's own rail carries no
        # rail at all (an aquaero's aquabus blocks, PROJECT.md section 8 item 89): the
        # value is published as None and the rule below never sees it. The flag is
        # absent from older records and from sources that publish a rail they measure,
        # and the voltage is then taken as given.
        rail_reported = bool(reading.get("rail_reported", True))
        # 0.00 V is what an aquaero's empty aquabus slot reads, so the rail rule does
        # not judge it -- which makes it a rule that did not run, not a rail found
        # healthy, and the verdict has to say which.
        rail_known = rail_reported and _finite(volts) and float(volts) > 0.0
        power_reported = bool(reading.get("power_reported"))
        power_known = power_reported and self._expected_power(channel, 1.0) is not None
        rpm_known = expected_rpm(self.cfg, channel, 1.0) is not None and _finite(rpm)
        out: dict[str, Any] = {
            "duty": duty if _finite(duty) else None,
            "rpm": rpm if _finite(rpm) else None,
            "voltage_v": volts if rail_reported and _finite(volts) else None,
            "current_ma": reading.get("current_ma") if _finite(reading.get("current_ma")) else None,
            "power_w": power if _finite(power) else None,
            "expected_rpm": None,
            "expected_power_w": None,
            "rpm_monitored": rpm_known,
            "rail_monitored": rail_known,
            "power_monitored": power_known,
            "unmonitored": _unmonitored(
                reading,
                {"rpm": rpm_known, "rail": rail_known, "power": power_known},
                self._local_reasons(
                    channel,
                    rpm=rpm,
                    volts=volts,
                    rail_reported=rail_reported,
                    power_reported=power_reported,
                ),
            ),
            "problems": [],
        }
        state = self._state(channel)
        problems: list[str] = []

        # the 12 V rail: judged at any duty and never restarted by a duty move, since
        # it does not depend on one (a rail that sags while the solver modulates is
        # exactly the case worth catching)
        if rail_known:
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
            and power_known
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

    def _local_reasons(
        self,
        channel: str,
        *,
        rpm: Any,
        volts: Any,
        rail_reported: bool,
        power_reported: bool,
    ) -> dict[str, str]:
        """Why *this monitor* could not run a rule on this channel's reading.

        The counterpart of the source's own ``not_measured`` (:func:`_unmonitored`):
        the source says what the device does not measure, and this says what the
        configuration does not describe or what this tick did not carry. Only the
        reasons that apply are built, so a fully judged channel builds none.
        """
        out: dict[str, str] = {}
        spec = self.cfg.fans.get(channel)
        model = None if spec is None else self.cfg.fan_models.get(spec.model)
        named = "" if spec is None else f" (`{spec.model}`)"
        if expected_rpm(self.cfg, channel, 1.0) is None:
            out["rpm"] = (
                f"no fan model with an `rpm_max` is configured for this channel{named} in "
                "`mpc.fans` / `mpc.fan_models`, so the rpm rule is off for it"
            )
        elif not _finite(rpm):
            out["rpm"] = "this output reported no speed on this tick, so the rpm rule did not run"
        if rail_reported and not _finite(volts):
            out["rail"] = (
                "this output reported no rail voltage on this tick, so the rail rule did not run"
            )
        elif rail_reported and not float(volts) > 0.0:
            out["rail"] = (
                "the rail reads exactly 0.00 V, which is not judged -- that is what an "
                "aquaero's empty aquabus slot reads, not a dead rail (PROJECT.md section 2)"
            )
        if power_reported and (model is None or model.power_w_at_max is None):
            out["power"] = (
                f"no `mpc.fan_models.<model>.power_w_at_max` is configured for this "
                f"channel{named}, so the power rule has nothing to judge the measured "
                "power against and is off for it (PROJECT.md section 8 items 79, 94)"
            )
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

    def update(
        self,
        readings: Mapping[str, Any],
        now: float,
        *,
        host: Mapping[str, Any] | None = None,
        temps: Mapping[str, Any] | None = None,
    ) -> dict[str, Any]:
        """Run the rules over one tick's readings and merge in the device health.

        One call is one tick, whatever it carries: a tick a channel is missing from
        (the read raised and the loop fed a blank observation, or the channel's
        aquabus slot went away) is a gap in that channel's evidence, and every window
        of it starts again on the next live reading. Without that, an outage's wall
        time would count towards a ``*_fault_s`` nothing measured during it.

        ``host`` is the tick's host metrics (:func:`aqua_bridge.hostinfo.collect_hostinfo`)
        and ``temps`` the observation's temperatures, the two the board's own rules
        need (item 103). They are judged and published here and nowhere else: neither
        reaches ``PlantObservation`` or the solver's ``diagnostics``.
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
        board = self.host.check(host, temps, now)
        payload = {
            "devices": devices["devices"],
            "fans": fans,
            "host": board,
            # Only the board's *faults* (hot, throttling now) are daemon problems.
            # The divergence rule is a hint about where to look, not a verdict, and
            # must not flip /api/health or the controller-fault sensor; it is in
            # host["problems"] and on the board's own host_problem sensor instead.
            "problems": [*devices["problems"], *problems, *board["faults"]],
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
            host = self._read_hostinfo()
            temps = getattr(obs, "temps", None)
            payload = self.update(readings, now, host=host, temps=temps)
            if self.publish is not None:
                self.publish(payload)
        except Exception:
            _LOG.exception("fan health: this tick was not evaluated")

    def _read_hostinfo(self) -> Mapping[str, Any] | None:
        """This tick's host metrics, or ``None`` without a reader or after a failure.

        The reader is a :class:`~aqua_bridge.hostinfo.CachedHostInfo` in the daemon,
        so a tick shorter than ``host.interval_s`` costs no ``/proc`` or ``/sys``
        read at all. A failure leaves the board's rules with nothing to judge this
        tick, which is exactly what they do with a missing reading anyway.
        """
        if self._hostinfo is None:
            return None
        try:
            return self._hostinfo()
        except Exception:  # the reader promises not to raise; belt and braces
            _LOG.exception("host health: the host metrics reader failed")
            return None

    def _now(self, obs: Any) -> float:
        if self._clock is not None:
            return float(self._clock())
        ts = getattr(obs, "ts", None)
        return float(ts) if _finite(ts) else 0.0
