"""The spin-up kick against the DAS truth plant, where the rotor really does not turn.

PROJECT.md section 8 item 75 and section 3 "Spin-up kick". The simulator's fans now
carry a ``start_duty``: a *stopped* rotor begins to turn only at or above it, and once
turning it keeps turning down to its dead band -- the physics item 75 records (the
owner's aquaero test fan stops at 13 % and starts at 25 %). A fan commanded between its
dead band and its start duty from a standstill therefore reports 0 rpm for ever, which
is the silently uncooled channel the owner met after a restart on 2026-09-17.

The runs here go through the real :class:`~aqua_bridge.control.loop.Loop` -- source,
``plan_tick``, ``mpc.step``, ``compose``, sink, ``record_tick``, applied-command
feedback and all -- against the truth plant, because what is being proven is that the
kick reaches the fans and that the command it produces is still a safe one.
``tests/test_spinup.py`` scripts the tachometer instead, to reach every branch of the
sequence.
"""

from __future__ import annotations

import copy
from collections.abc import Mapping
from typing import Any

import pytest

from aqua_bridge.control.loop import Loop, TickResult
from aqua_bridge.control.spinup import SpinUpChannel, SpinUpConfig, validate_spin_up
from aqua_bridge.control.supervisor import Supervisor
from aqua_bridge.model import ConfigError, MpcCommand, MpcConfig, PlantObservation
from aqua_bridge.sim.das import DasPlant, build_das_plant, topology_from_config
from das_fixtures import das_cfg

#: Short windows so a whole sequence fits in a run of 1 s ticks (``das_cfg`` has dt = 1).
FAST: dict[str, Any] = {
    "confirm_s": 4.0,
    "kick_s": 6.0,
    "verify_s": 3.0,
    "max_attempts": 2,
    "retry_s": 600.0,
    "kick_duty": 0.5,
}


def spin(**changes: Any) -> SpinUpConfig:
    return SpinUpConfig(**{**FAST, **changes})


class _Source:
    """The plant as the loop's source: this tick's observation, restricted to ``temps``."""

    def __init__(self, cfg: MpcConfig, plant: DasPlant) -> None:
        self.cfg, self.plant = cfg, plant

    def read(self) -> PlantObservation:
        raw = self.plant.observe()
        return PlantObservation(
            temps={name: raw.temps.get(name) for name in self.cfg.temps},
            rpm=raw.rpm,
            pwm=raw.pwm,
            ts=raw.ts,
        )


class _Sink:
    """The plant as the loop's sink: apply the command, then integrate one ``dt``."""

    def __init__(self, plant: DasPlant) -> None:
        self.plant = plant

    def apply(self, cmd: MpcCommand) -> None:
        self.plant.apply(cmd.pwm)
        self.plant.advance()


class SimRun:
    """One run of the daemon's loop against the truth plant, with what it did recorded."""

    def __init__(self, cfg: MpcConfig, plant: DasPlant, settings: SpinUpConfig) -> None:
        self.cfg, self.plant = cfg, plant
        self.sup = Supervisor(cfg, spin_up=settings)
        self.pwm: list[dict[str, float]] = []
        self.rpm: list[dict[str, float | None]] = []
        self.kicks: list[tuple[int, str, float]] = []
        self.holds: list[tuple[int, str, float]] = []
        self.loop = Loop(
            _Source(cfg, plant), _Sink(plant), cfg, self.sup, clock=lambda: 0.0, on_tick=self._seen
        )

    def _seen(self, result: TickResult) -> None:
        i = result.index
        self.pwm.append(dict(result.cmd.pwm))
        self.rpm.append(dict(result.obs.rpm))
        floors = result.cmd.diagnostics.get("supervisor", {}).get("spin_up") or {}
        for ch, duty in sorted((floors.get("kick") or {}).items()):
            self.kicks.append((i, ch, duty))
        for ch, duty in sorted((floors.get("hold") or {}).items()):
            self.holds.append((i, ch, duty))

    def run(self, ticks: int) -> SimRun:
        for _ in range(ticks):
            self.loop.tick()
        return self

    def states(self) -> dict[str, str]:
        return {ch: v["state"] for ch, v in self.sup.spin_up_status().items()}

    def attempts(self, channel: str) -> int:
        """How many separate kicks went out (contiguous runs of kicked ticks)."""
        ticks = sorted(t for t, ch, _ in self.kicks if ch == channel)
        return sum(1 for i, t in enumerate(ticks) if i == 0 or ticks[i - 1] != t - 1)


def build(
    *,
    fans: Mapping[str, Mapping[str, Any]] | None = None,
    settings: SpinUpConfig | None = None,
    initial_pwm: float = 0.15,
    setpoint: float | None = None,
) -> SimRun:
    """A run of the ``das_fixtures`` enclosure with ``fans`` patched into the truth plant.

    ``initial_pwm`` is the duty the fans were left at -- 0.15 is ``pwm_min``, which is
    where the controller settles on this cool plant and exactly the state a restart
    meets: every rotor stopped, every command healthy.
    """
    cfg = (
        das_cfg()
        if setpoint is None
        else das_cfg(setpoints=dict.fromkeys(("air_a", "air_b", "air_c"), setpoint))
    )
    topo = copy.deepcopy(topology_from_config(cfg))
    for ch, changes in (fans or {}).items():
        topo["fans"][ch].update(changes)
    plant = build_das_plant(topo, preset="basic", seed=0, dt=cfg.dt, initial_pwm=initial_pwm)
    return SimRun(cfg, plant, settings or spin())


# --- the scenarios -------------------------------------------------------------------


def test_a_stalled_fan_starts_on_the_first_kick_and_stays_turning() -> None:
    """The owner's case: after a restart the fans sit at a duty above the one they stop
    at but below the one they start at, and the tachometer reads nothing."""
    run = build(fans={"fa1": {"start_duty": 0.35}}).run(40)
    assert run.rpm[0]["fa1"] == 0.0, "the rotor is standing at the commanded duty"
    assert run.rpm[0]["fa2"] > 60.0, "its sibling on the same duty is turning"
    assert run.attempts("fa1") == 1 and all(ch == "fa1" for _, ch, _ in run.kicks)
    assert run.states()["fa1"] == "turning"
    assert run.rpm[-1]["fa1"] > 60.0
    # ... and the channel is back on the solver, not held at the kick duty: a fan that
    # has started keeps turning down to its dead band, item 75's other half.
    assert run.pwm[-1]["fa1"] == pytest.approx(run.pwm[-1]["fa2"])
    assert run.pwm[-1]["fa1"] == pytest.approx(run.cfg.pwm_min)


def test_a_kick_too_short_for_the_ramp_is_refused_rather_than_condemning_the_fan() -> None:
    """``d_pwm_max`` ramps the kick, so a ``kick_s`` that does not cover the ramp expires
    short of the kick duty -- and the fan is then declared failed for a kick it never
    got. ``validate_spin_up`` refuses such a config at startup; with a legal ``kick_s``
    the ramp reaches the duty and this fan, whose rotor needs 0.55 to start, starts."""
    cfg = das_cfg()  # dt 1, d_pwm_max 0.1, pwm_min 0.15: 0.15 -> 0.6 is 5 ticks of ramp
    short = spin(kick_duty=0.6, kick_s=3.0, verify_s=1.0, max_attempts=3)
    with pytest.raises(ConfigError, match="cannot deliver a kick to 0.6"):
        validate_spin_up(cfg, short)
    settings = spin(kick_duty=0.6, kick_s=6.0, verify_s=1.0, max_attempts=3)
    validate_spin_up(cfg, settings)
    run = build(fans={"fa1": {"start_duty": 0.55}}, settings=settings).run(50)
    # one kick, ramped a d_pwm_max step per tick until the rotor breaks free at 0.55
    assert [round(p["fa1"], 2) for p in run.pwm[5:11]] == [0.25, 0.35, 0.45, 0.55, 0.6, 0.6]
    assert run.attempts("fa1") == 1
    assert run.states()["fa1"] == "turning" and run.rpm[-1]["fa1"] > 60.0


def test_a_dead_fan_is_declared_failed_and_its_zone_keeps_its_cooling() -> None:
    """A seized rotor never turns, whatever the kick. After the configured attempts it
    is a failed fan with a message, and the other channels of the zones it served are
    floored where they were, so the zone cannot lose cooling because it died."""
    run = build()
    run.plant.stall("fa1")  # seized: not a start-duty problem, a dead fan
    run.run(60)
    verdict = run.sup.spin_up_status()["fa1"]
    assert verdict["state"] == "failed" and verdict["failed"] is True
    assert verdict["attempts"] == 2 and run.attempts("fa1") == 2
    assert "lost its airflow" in verdict["reason"]
    held = {ch for _, ch, _ in run.holds}
    assert "fa2" in held, "its sibling in zone za"
    assert "fb1" in held, "zone zb, which za declares coupled_to"
    assert "fc1" not in held, "an unrelated zone is not floored"
    last = run.holds[-1][0]
    for ch, duty in ((c, d) for t, c, d in run.holds if t == last):
        assert all(pwm[ch] >= duty - 1e-9 for pwm in run.pwm[-10:]), ch


def test_an_output_with_no_fan_is_never_kicked() -> None:
    """Nothing hangs on it, so 0 rpm is the truth and not a fault. The daemon knows
    from the config, never from the report."""
    run = build(
        fans={"fa1": {"start_duty": 1.0}},  # would never start if it were judged
        settings=spin(channels={"fa1": SpinUpChannel(fan=False)}),
    ).run(40)
    verdict = run.sup.spin_up_status()["fa1"]
    assert verdict["state"] == "off" and not verdict["monitored"]
    assert "nothing hangs on this output" in verdict["reason"]
    assert run.kicks == [] and run.holds == []


def test_a_fan_with_no_tachometer_is_never_kicked() -> None:
    """Two shapes of the same answer: the plant gives ``fa1`` no tach wire, so its key
    is absent from ``obs.rpm`` exactly as an ``aquacomputer:`` entry without ``rpm:``
    would leave it; ``fa2`` is declared tach-less in the config. Neither is judged, and
    each says which."""
    run = build(
        fans={"fa1": {"tach": False, "start_duty": 1.0}, "fa2": {"start_duty": 1.0}},
        settings=spin(channels={"fa2": SpinUpChannel(tachometer=False)}),
    ).run(40)
    assert "fa1" not in run.rpm[0]
    status = run.sup.spin_up_status()
    assert status["fa1"]["state"] == "off" and "no tachometer is bound" in status["fa1"]["reason"]
    assert status["fa2"]["state"] == "off" and "drives no tachometer" in status["fa2"]["reason"]
    assert run.kicks == [] and run.holds == []


def test_a_kick_while_the_solver_already_commands_more_changes_nothing() -> None:
    """The kick is a floor. Where the solver is above it the composed command is the
    solver's, tick for tick -- and the sequence still runs to an honest verdict."""
    fans = {"fa1": {"start_duty": 0.9}}
    settings = spin(kick_duty=0.2, failed_channel_floor=False)
    kicked = build(fans=fans, settings=settings, setpoint=26.0).run(60)
    plain = build(fans=fans, settings=SpinUpConfig(enabled=False), setpoint=26.0).run(60)
    assert any(ch == "fa1" for _, ch, _ in kicked.kicks), "the rule did fire on fa1"
    for i, _ch, duty in kicked.kicks:
        assert plain.pwm[i]["fa1"] >= duty, "the solver was already above the kick duty"
    assert kicked.pwm == plain.pwm, "so the kick changed no command at all"
    assert kicked.sup.spin_up_status()["fa1"]["state"] == "failed"


def test_the_kick_never_lowers_a_command_and_never_leaves_the_actuator_range() -> None:
    """The safety invariant against the plant: with the rule on, no channel is ever
    below where the same run put it with the rule off; every command is inside
    ``[pwm_min, pwm_max]``; and no step is larger than ``d_pwm_max``."""
    fans = {"fa1": {"start_duty": 0.35}, "fa2": {"start_duty": 1.0}}
    kicked = build(fans=fans).run(60)
    plain = build(fans=fans, settings=SpinUpConfig(enabled=False)).run(60)
    cfg = kicked.cfg
    for with_kick, without in zip(kicked.pwm, plain.pwm, strict=True):
        for ch in cfg.channels:
            assert with_kick[ch] >= without[ch] - 1e-9, ch
            assert cfg.pwm_min - 1e-9 <= with_kick[ch] <= cfg.pwm_max + 1e-9, ch
    for a, b in zip(kicked.pwm, kicked.pwm[1:], strict=False):
        for ch in cfg.channels:
            assert abs(b[ch] - a[ch]) <= cfg.d_pwm_max + 1e-9, ch


def test_a_healthy_enclosure_is_never_kicked() -> None:
    """No false positives on the plant every other DAS suite runs: the rule costs a
    healthy enclosure nothing."""
    run = build().run(120)
    assert run.kicks == [] and run.holds == []
    assert run.states()["fa1"] == "turning" and run.states()["fa2"] == "turning"
    # fb1 / fc1 sit inside their 0.2 dead band at pwm_min, where a standing rotor is
    # normal and the curve says so: not judged, not kicked, not alarmed.
    assert run.states()["fb1"] == "idle" and run.states()["fc1"] == "idle"


def test_the_start_duty_default_leaves_the_plant_exactly_as_it_was() -> None:
    """``start_duty`` defaults to 0, which is the physics every seeded run had before it
    existed -- so no golden file and no existing DAS run moves."""
    off = SpinUpConfig(enabled=False)
    before = build(settings=off).run(60)
    zeros = dict.fromkeys(("fa1", "fa2", "fb1", "fc1"), {"start_duty": 0.0})
    after = build(fans=zeros, settings=off).run(60)
    assert before.pwm == after.pwm
    assert before.rpm == after.rpm
