"""The spin-up kick: config, the sequence, and what it must never touch (item 75).

A fan's starting duty is above its running duty (the owner's aquaero test fan stops at
13 % and starts at 25 %), so a channel can read a healthy duty on a healthy rail while
the rotor stands still. These tests drive the supervisor the way the loop does --
``plan_tick`` / ``compose`` / ``record_tick`` -- with the tachometer scripted, so every
branch of the sequence is exercised on its own facts. ``tests/test_spinup_sim.py``
proves the same rule against the DAS truth plant, where the rotor really does not turn.
"""

from __future__ import annotations

import logging
from collections.abc import Mapping
from pathlib import Path
from typing import Any

import pytest

from aqua_bridge.config import load_config
from aqua_bridge.control import noise, spinup
from aqua_bridge.control.spinup import (
    SPINUP_CHANNEL_KEYS,
    SPINUP_KEYS,
    SpinUpChannel,
    SpinUpConfig,
    channel_kick_duty,
    validate_spin_up,
)
from aqua_bridge.control.supervisor import Supervisor
from aqua_bridge.model import ConfigError, Mode, MpcCommand, MpcConfig, PlantObservation
from das_fixtures import das_cfg, default_temps

REPO_ROOT = Path(__file__).resolve().parent.parent

#: Short windows so a sequence fits in a handful of 1 s ticks (``das_cfg`` has dt = 1).
FAST = {
    "confirm_s": 3.0,
    "kick_s": 6.0,
    "verify_s": 3.0,
    "max_attempts": 2,
    "retry_s": 60.0,
    "kick_duty": 0.6,
}


def spin(**changes: Any) -> SpinUpConfig:
    return SpinUpConfig(**{**FAST, **changes})


# --- the section ---------------------------------------------------------------------


def test_the_key_list_is_the_documented_one() -> None:
    assert set(SPINUP_KEYS) == {
        "enabled",
        "stall_duty",
        "kick_duty",
        "kick_s",
        "confirm_s",
        "verify_s",
        "max_attempts",
        "min_rpm",
        "retry_s",
        "failed_channel_floor",
        "log_interval_s",
        "channels",
    }
    assert set(SPINUP_CHANNEL_KEYS) == {"fan", "tachometer", "stall_duty", "kick_duty"}


@pytest.mark.parametrize(
    ("section", "match"),
    [
        ({"enabled": "true"}, "spin_up.enabled must be true or false"),
        ({"stall_duty": 1.5}, "spin_up.stall_duty must be <= 1"),
        ({"kick_duty": "0.5"}, "spin_up.kick_duty must be a number"),
        ({"kick_s": 0}, "spin_up.kick_s must be >="),
        ({"confirm_s": float("inf")}, "spin_up.confirm_s must be finite"),
        ({"max_attempts": 0}, "spin_up.max_attempts must be >= 1"),
        ({"max_attempts": 2.5}, "spin_up.max_attempts must be an integer"),
        ({"min_rpm": -1}, "spin_up.min_rpm must be >="),
        ({"kick_duty": 0.1, "stall_duty": 0.2}, "must be above spin_up.stall_duty"),
        ({"nope": 1}, r"spin_up: unknown key\(s\)"),
        ({"channels": {"fa1": {"nope": 1}}}, r"spin_up.channels.fa1: unknown key\(s\)"),
        ({"channels": {"fa1": {"fan": "no"}}}, "spin_up.channels.fa1.fan must be true or false"),
        ({"channels": {"fa1": 0.5}}, "spin_up.channels.fa1 must be a mapping"),
        ({"channels": {"fa1": {"kick_duty": 0.1}}}, "must be above its stall duty"),
    ],
)
def test_a_bad_spin_up_key_is_a_config_error(section: dict[str, Any], match: str) -> None:
    with pytest.raises(ConfigError, match=match):
        SpinUpConfig.from_section(section)


def test_an_unknown_channel_is_refused_against_the_controller_config() -> None:
    cfg = das_cfg()
    with pytest.raises(ConfigError, match="spin_up.channels names"):
        validate_spin_up(cfg, SpinUpConfig(channels={"nope": SpinUpChannel()}))
    validate_spin_up(cfg, SpinUpConfig(channels={"fa1": SpinUpChannel()}))


def test_a_kick_shorter_than_a_tick_is_refused() -> None:
    """A kick the daemon could never deliver is a configuration error, not a rule that
    silently never fires."""
    cfg = das_cfg(dt=5.0, confirm_s=10.0, fallback_hold_s=20.0, stuck_s=20.0)
    with pytest.raises(ConfigError, match="spin_up.kick_s .* is shorter than one tick"):
        validate_spin_up(cfg, SpinUpConfig(kick_s=2.0))
    validate_spin_up(cfg, SpinUpConfig(kick_s=2.0, enabled=False))  # off: nothing to deliver


def test_the_kick_duty_is_per_output_and_clamped_into_the_actuator_range() -> None:
    """The duty-to-rpm mapping belongs to the output, not to the fan model: measured
    2026-09-18, the same fan reads 174 rpm on an aquaero output and 255 rpm behind the
    aquabus device at the same nominal 20 %."""
    cfg = das_cfg()
    settings = SpinUpConfig(
        kick_duty=0.5,
        channels={"fa1": SpinUpChannel(kick_duty=0.35), "fb1": SpinUpChannel(kick_duty=1.0)},
    )
    assert channel_kick_duty(cfg, settings, "fa1") == pytest.approx(0.35)
    assert channel_kick_duty(cfg, settings, "fc1") == pytest.approx(0.5)  # the fallback
    assert channel_kick_duty(cfg, settings, "fb1") == pytest.approx(cfg.pwm_max)


@pytest.mark.parametrize("name", ["config.example.yaml", "config.example-das.yaml"])
def test_both_example_configs_show_every_key_at_its_default(name: str) -> None:
    """Every operator tunable is a documented key with one default, in both examples.

    The DAS example is the owner's enclosure, so its ``channels:`` carries the measured
    per-output kick duties -- the one place a shipped value is deliberately not the
    default, because a single global kick is what this item exists to refuse."""
    app = load_config(REPO_ROOT / name)
    section = app.section("spin_up")
    assert set(section) == set(SPINUP_KEYS), name
    settings = SpinUpConfig.from_section(section)
    assert settings == SpinUpConfig(channels=settings.channels), name
    validate_spin_up(app.mpc, settings)
    if name == "config.example.yaml":
        assert settings.channels == {}, "the legacy example documents the shape, not an output"
    else:
        assert set(settings.channels) == set(app.mpc.channels)
        aquaero = {channel_kick_duty(app.mpc, settings, ch) for ch in ("xt1", "xt2", "xt3", "xt4")}
        aquabus = {channel_kick_duty(app.mpc, settings, ch) for ch in ("qd1", "qd2", "qd3", "qd4")}
        assert aquaero == {0.5} and aquabus == {0.35}


def test_a_legacy_config_is_never_kicked() -> None:
    """The rule needs the channel's fitted curve to say what speed an output should
    reach, and ``mpc.fans`` / ``mpc.fan_models`` need ``mpc.topology``. Without one the
    whole section is off, with that as the reason -- so the legacy command path is
    untouched."""
    cfg = load_config(REPO_ROOT / "config.example.yaml").mpc
    sup = Supervisor(cfg, spin_up=spin())
    run = Run(sup, cfg, rpm=0.0, demand=0.5)
    run.ticks(40)
    status = sup.spin_up_status()
    assert set(status) == set(cfg.channels)
    for ch, verdict in status.items():
        assert verdict["state"] == "off" and not verdict["monitored"], ch
        assert "mpc.topology" in verdict["reason"]
    assert run.kicked == []


# --- driving the supervisor the way the loop does ------------------------------------


class Run:
    """One scripted run through ``plan_tick`` / ``compose`` / ``record_tick``.

    ``rpm`` is what the tachometer reports -- one number for every channel, or a
    callable ``(tick, channel, applied duty) -> rpm | None``; a channel whose value is
    ``None`` is left out of ``obs.rpm`` entirely, which is what a channel with no
    tachometer bound looks like. ``demand`` is the solver's command.
    """

    def __init__(
        self,
        sup: Supervisor,
        cfg: MpcConfig,
        *,
        rpm: Any,
        demand: Any = 0.3,
        mode: Mode = Mode.AUTO,
    ) -> None:
        self.sup, self.cfg, self._rpm, self._demand, self.mode = sup, cfg, rpm, demand, mode
        self.t = 0.0
        self.tick_index = 0
        self.applied: dict[str, float] = dict.fromkeys(cfg.channels, cfg.pwm_min)
        #: ``(tick, channel, duty)`` for every tick a kick floor was in force. The
        #: tracker is advanced at the *end* of a tick and read at the start of the next,
        #: so the floor lands one tick after the sequence enters its kick.
        self.kicked: list[tuple[int, str, float]] = []
        self.history: list[dict[str, float]] = []

    def _value(self, spec: Any, channel: str) -> Any:
        if callable(spec):
            return spec(self.tick_index, channel, self.applied[channel])
        if isinstance(spec, Mapping):
            return spec.get(channel)
        return spec

    def tick(self, *, live: bool = True) -> MpcCommand:
        plan = self.sup.plan_tick()
        prev = dict(self.applied)
        demand = {
            ch: max(
                self.cfg.pwm_min,
                min(
                    self.cfg.pwm_max,
                    min(prev[ch] + self.cfg.d_pwm_max, self._value(self._demand, ch)),
                ),
            )
            for ch in self.cfg.channels
        }
        mpc_cmd = MpcCommand(pwm=demand, mode=self.mode, diagnostics={})
        cmd = self.sup.compose(mpc_cmd, plan, prev)
        kicks = (plan.spin_up or {}).get("kick") or {}
        for ch, duty in sorted(kicks.items()):
            self.kicked.append((self.tick_index, ch, duty))
        self.applied = dict(cmd.pwm)
        self.history.append(dict(cmd.pwm))
        rpm = {ch: self._value(self._rpm, ch) for ch in self.cfg.channels}
        obs = PlantObservation(
            temps=default_temps(self.cfg) if self.cfg.is_das else {},
            rpm={ch: v for ch, v in rpm.items() if v is not None},
            pwm=dict(cmd.pwm),
            ts=self.t,
        )
        self.sup.record_tick(
            obs=obs if live else None,
            mpc_cmd=mpc_cmd,
            cmd=cmd,
            state=None,
            applied=live,
            usb_present=live,
            ts=self.t,
        )
        self.t += self.cfg.dt
        self.tick_index += 1
        return cmd

    def ticks(self, n: int, *, live: bool = True) -> MpcCommand:
        cmd = self.tick(live=live)
        for _ in range(n - 1):
            cmd = self.tick(live=live)
        return cmd

    def states(self) -> dict[str, str]:
        return {ch: v["state"] for ch, v in self.sup.spin_up_status().items()}

    def attempts(self, channel: str) -> int:
        """How many separate kicks went out on ``channel`` (contiguous runs of ticks)."""
        ticks = sorted(t for t, ch, _ in self.kicked if ch == channel)
        return sum(1 for i, t in enumerate(ticks) if i == 0 or ticks[i - 1] != t - 1)


# --- the sequence --------------------------------------------------------------------


def test_a_standing_fan_is_confirmed_then_kicked_and_the_kick_is_a_floor() -> None:
    cfg = das_cfg()
    sup = Supervisor(cfg, spin_up=spin())
    run = Run(sup, cfg, rpm=0.0, demand=0.3)
    run.ticks(3)
    assert run.states()["fa1"] == "confirming" and run.kicked == []
    run.tick()  # confirm_s = 3 s of readings has accumulated
    assert run.states()["fa1"] == "kicking"
    assert run.kicked == [], "the tracker is advanced at the end of a tick"
    run.tick()  # ... and read at the start of the next one, where the floor lands
    assert sorted(ch for _, ch, _ in run.kicked) == sorted(cfg.channels)
    # the floor is a floor: rate limited against the last applied PWM, never a jump
    before, after = run.history[-2], run.history[-1]
    for ch in cfg.channels:
        assert after[ch] - before[ch] == pytest.approx(cfg.d_pwm_max)
        assert cfg.pwm_min <= after[ch] <= cfg.pwm_max


def test_a_fan_that_starts_on_the_first_kick_goes_back_to_the_solver() -> None:
    cfg = das_cfg()
    sup = Supervisor(cfg, spin_up=spin())
    turning = {"on": False}

    def rpm(_tick: int, channel: str, duty: float) -> float:
        if channel != "fa1":
            return 900.0
        if duty >= 0.5:
            turning["on"] = True
        return 900.0 if turning["on"] else 0.0

    run = Run(sup, cfg, rpm=rpm, demand=0.3)
    run.ticks(14)
    assert run.states()["fa1"] == "turning"
    assert run.attempts("fa1") == 1 and all(ch == "fa1" for _, ch, _ in run.kicked)
    # ... and the channel is back at the solver's demand, not held at the kick duty
    assert run.history[-1]["fa1"] == pytest.approx(0.3)


def test_a_fan_that_needs_two_kicks_is_given_two_and_no_more_than_that() -> None:
    cfg = das_cfg()
    sup = Supervisor(cfg, spin_up=spin())
    seen = {"above": False, "kicks": 0}

    def rpm(_tick: int, channel: str, duty: float) -> float:
        """The rotor breaks free on the *second* time it is driven up to the kick duty."""
        if channel != "fa1":
            return 900.0
        above = duty >= 0.6
        seen["kicks"] += int(above and not seen["above"])
        seen["above"] = above
        return 900.0 if seen["kicks"] >= 2 else 0.0

    run = Run(sup, cfg, rpm=rpm, demand=0.3)
    run.ticks(40)
    assert sup.spin_up_status()["fa1"]["state"] == "turning"
    assert sup.spin_up_status()["fa1"]["attempts"] == 0  # cleared once it turns
    assert run.attempts("fa1") == 2


def test_a_dead_fan_is_declared_failed_after_the_configured_attempts() -> None:
    cfg = das_cfg()
    sup = Supervisor(cfg, spin_up=spin(max_attempts=2, failed_channel_floor=False))
    run = Run(sup, cfg, rpm={"fa1": 0.0, "fa2": 900.0, "fb1": 900.0, "fc1": 900.0}, demand=0.3)
    run.ticks(40)
    verdict = sup.spin_up_status()["fa1"]
    assert verdict["state"] == "failed" and verdict["attempts"] == 2
    assert "does not turn" in verdict["reason"] and "lost its airflow" in verdict["reason"]
    assert verdict["failed"] is True
    assert run.attempts("fa1") == 2  # bounded in repetitions, not one kick per tick
    assert all(ch == "fa1" for _, ch, _ in run.kicked)


def test_a_failed_fan_floors_its_siblings_and_never_lowers_them() -> None:
    """The zone must be treated as having lost that fan's airflow: more from the
    siblings, never less. The floor is what they carried when the failure was declared;
    the rise above it comes from the temperatures, through the solver."""
    cfg = das_cfg()
    sup = Supervisor(cfg, spin_up=spin(max_attempts=1))
    demand = {"fa1": 0.3, "fa2": 0.4, "fb1": 0.4, "fc1": 0.4}
    run = Run(sup, cfg, rpm={"fa1": 0.0, "fa2": 900.0, "fb1": 900.0, "fc1": 900.0}, demand=demand)
    run.ticks(20)
    assert sup.spin_up_status()["fa1"]["state"] == "failed"
    held = run.history[-1]
    # fa2 shares zone za with fa1, fb1 is in zb which za declares coupled_to; fc1 is not.
    assert held["fa2"] == pytest.approx(0.4) and held["fb1"] == pytest.approx(0.4)
    # the solver now asks for much less: the floored siblings do not follow it down
    run._demand = {"fa1": 0.3, "fa2": 0.2, "fb1": 0.2, "fc1": 0.2}
    run.ticks(20)
    end = run.history[-1]
    assert end["fa2"] == pytest.approx(0.4) and end["fb1"] == pytest.approx(0.4)
    assert end["fc1"] == pytest.approx(0.2), "a channel of an unrelated zone is not floored"


def test_the_floor_survives_a_retry_and_never_ratchets_upward() -> None:
    """``retry_s`` re-tests a fan that may have been replaced. That retry runs *under*
    the floor its failure left -- only the tachometer lifts it -- and the floor keeps
    the level it was first recorded at, so a hot spell between two retries cannot pin
    the siblings higher and higher."""
    cfg = das_cfg()
    sup = Supervisor(cfg, spin_up=spin(max_attempts=1, retry_s=10.0))
    dead = {"fa1": 0.0, "fa2": 900.0, "fb1": 900.0, "fc1": 900.0}
    run = Run(sup, cfg, rpm=dead, demand={"fa1": 0.3, "fa2": 0.4, "fb1": 0.4, "fc1": 0.4})
    run.ticks(20)
    assert sup.spin_up_status()["fa1"]["state"] == "failed"
    floor = run.history[-1]["fa2"]
    assert floor == pytest.approx(0.4)
    settled = len(run.history)
    run._demand = dict(run._demand, fa2=0.9)  # a hot spell: the solver raises the sibling
    run.ticks(20)
    assert run.history[-1]["fa2"] == pytest.approx(0.9)
    run._demand = dict(run._demand, fa2=0.2)  # and passes, across at least one retry
    run.ticks(40)
    assert run.attempts("fa1") >= 2, "the failed fan was retried"
    assert sup.spin_up_status()["fa1"]["state"] == "failed"
    assert run.history[-1]["fa2"] == pytest.approx(floor), "the floor did not ratchet"
    later = run.history[settled:]
    assert min(pwm["fa2"] for pwm in later) >= floor - 1e-12, "and never went below it"


def test_a_kick_is_a_no_op_while_the_solver_already_commands_more() -> None:
    cfg = das_cfg()
    sup = Supervisor(cfg, spin_up=spin(kick_duty=0.4))
    run = Run(sup, cfg, rpm=0.0, demand=0.8)
    run.ticks(20)
    assert any(ch == "fa1" for _, ch, _ in run.kicked), "the rule did fire"
    for pwm in run.history[6:]:
        assert pwm["fa1"] == pytest.approx(0.8), "the kick never pulled the solver down"


def test_the_kick_never_lowers_a_channel_and_never_leaves_the_actuator_range() -> None:
    """The safety invariant, on every tick of every sequence above."""
    cfg = das_cfg()
    sup_plain = Supervisor(cfg, spin_up=SpinUpConfig(enabled=False))
    plain = Run(sup_plain, cfg, rpm=0.0, demand=0.45)
    plain.ticks(40)
    sup_kick = Supervisor(cfg, spin_up=spin())
    kicked = Run(sup_kick, cfg, rpm=0.0, demand=0.45)
    kicked.ticks(40)
    for without, with_kick in zip(plain.history, kicked.history, strict=True):
        for ch in cfg.channels:
            assert with_kick[ch] >= without[ch] - 1e-12, ch
            assert cfg.pwm_min <= with_kick[ch] <= cfg.pwm_max, ch


# --- what must never be kicked and never alarmed -------------------------------------


def test_an_output_with_no_fan_is_never_kicked_and_never_alarmed() -> None:
    cfg = das_cfg()
    settings = spin(channels={"fa1": SpinUpChannel(fan=False)})
    sup = Supervisor(cfg, spin_up=settings)
    run = Run(sup, cfg, rpm=0.0, demand=0.3)
    run.ticks(40)
    verdict = sup.spin_up_status()["fa1"]
    assert verdict["state"] == "off" and not verdict["monitored"] and not verdict["failed"]
    assert "nothing hangs on this output" in verdict["reason"]
    assert all(ch != "fa1" for _, ch, _ in run.kicked)


def test_a_fan_with_no_tachometer_is_never_kicked_and_never_alarmed() -> None:
    cfg = das_cfg()
    sup = Supervisor(cfg, spin_up=spin(channels={"fa1": SpinUpChannel(tachometer=False)}))
    run = Run(sup, cfg, rpm=0.0, demand=0.3)
    run.ticks(40)
    verdict = sup.spin_up_status()["fa1"]
    assert verdict["state"] == "off" and not verdict["monitored"]
    assert "drives no tachometer" in verdict["reason"]
    assert all(ch != "fa1" for _, ch, _ in run.kicked)


def test_a_channel_with_no_tachometer_bound_says_so_rather_than_guessing() -> None:
    """``obs.rpm`` has no key for an output whose ``aquacomputer:`` entry has no
    ``rpm:`` -- exactly what the DAS simulator does for a tach-less output."""
    cfg = das_cfg()
    sup = Supervisor(cfg, spin_up=spin())
    run = Run(sup, cfg, rpm={"fa1": None, "fa2": 0.0, "fb1": 900.0, "fc1": 900.0}, demand=0.3)
    run.ticks(40)
    verdict = sup.spin_up_status()["fa1"]
    assert verdict["state"] == "off" and not verdict["monitored"]
    assert "no tachometer is bound to it" in verdict["reason"]
    assert all(ch != "fa1" for _, ch, _ in run.kicked)
    assert sup.spin_up_status()["fa2"]["state"] == "failed", "its neighbour is still judged"


def test_a_duty_below_the_stall_duty_is_not_judged() -> None:
    """Inside and just above a dead band a standing rotor is normal, and a kick there
    would be noise. ``fb1``'s model has a 0.2 dead band, so at 0.25 the curve expects
    almost nothing and the channel is left alone."""
    cfg = das_cfg()
    sup = Supervisor(cfg, spin_up=spin())
    run = Run(sup, cfg, rpm=0.0, demand=0.22)
    run.ticks(40)
    assert sup.spin_up_status()["fb1"]["state"] == "idle"
    assert all(ch != "fb1" for _, ch, _ in run.kicked)


def test_a_gap_in_the_readings_starts_the_window_again() -> None:
    """Wall time passing while nothing was measured is not evidence of anything."""
    cfg = das_cfg()
    sup = Supervisor(cfg, spin_up=spin())
    run = Run(sup, cfg, rpm=0.0, demand=0.3)
    run.ticks(3)
    assert run.states()["fa1"] == "confirming"
    run.tick(live=False)
    assert run.states()["fa1"] == "idle" and run.kicked == []
    run.ticks(3)
    assert run.states()["fa1"] == "confirming" and run.kicked == []


def test_the_rule_is_off_entirely_when_the_section_says_so() -> None:
    cfg = das_cfg()
    sup = Supervisor(cfg, spin_up=SpinUpConfig(enabled=False))
    run = Run(sup, cfg, rpm=0.0, demand=0.5)
    run.ticks(40)
    assert run.kicked == []
    for verdict in sup.spin_up_status().values():
        assert verdict["state"] == "off" and "spin_up.enabled is false" in verdict["reason"]


def test_without_a_floor_compose_returns_the_solver_command_untouched() -> None:
    """The legacy path, and every ordinary tick: no floor, no change, not even a key in
    ``diagnostics["supervisor"]``."""
    cfg = das_cfg()
    sup = Supervisor(cfg, spin_up=spin())
    run = Run(sup, cfg, rpm=900.0, demand=0.4)
    cmd = run.ticks(20)
    assert "spin_up" not in cmd.diagnostics["supervisor"]
    assert cmd.pwm == pytest.approx(run.history[-1])


def test_a_fallback_command_still_gets_the_floor_because_it_only_raises() -> None:
    cfg = das_cfg()
    sup = Supervisor(cfg, spin_up=spin())
    run = Run(sup, cfg, rpm=0.0, demand=0.3, mode=Mode.FALLBACK)
    run.ticks(6)
    assert any(ch == "fa1" for _, ch, _ in run.kicked)
    assert run.history[-1]["fa1"] > run.history[0]["fa1"]


def test_the_kick_is_logged_once_per_attempt_and_the_failure_as_an_error(caplog) -> None:
    cfg = das_cfg()
    sup = Supervisor(cfg, spin_up=spin(max_attempts=1))
    with caplog.at_level(logging.WARNING, logger="aqua_bridge.supervisor"):
        Run(sup, cfg, rpm={"fa1": 0.0, "fa2": 900.0, "fb1": 900.0, "fc1": 900.0}).ticks(30)
    kicks = [r for r in caplog.records if "kicking to" in r.getMessage()]
    fails = [r for r in caplog.records if r.levelno == logging.ERROR]
    assert len(kicks) == 1 and len(fails) == 1
    assert "fa1" in fails[0].getMessage()


# --- what a kick costs in noise ------------------------------------------------------


def test_the_noise_cost_of_a_kick_at_the_shipped_defaults() -> None:
    """PROJECT.md section 3 "Spin-up kick" reports these; pin them so the table cannot
    drift from the code. The index is ``control/noise.py``'s energetic one."""
    app = load_config(REPO_ROOT / "config.example-das.yaml")
    cfg, settings = app.mpc, SpinUpConfig.from_section(app.section("spin_up"))

    def index(duties: Mapping[str, float]) -> float:
        fr = {
            ch: noise.rpm_frac(duties[ch], noise.channel_deadband(cfg, ch)) for ch in cfg.channels
        }
        return noise.noise_db(cfg, fr)

    for base, expected in ((0.20, -7.7), (0.30, 7.3), (0.50, 22.4)):
        quiet = dict.fromkeys(cfg.channels, base)
        assert index(quiet) == pytest.approx(expected, abs=0.05)
        for ch, rise in (
            ("xt1", (23.1, 8.6, 0.0)),
            ("xt3", (20.1, 6.1, 0.0)),
            ("qd1", (10.3, 0.8, 0.0)),
        ):
            kicked = dict(quiet)
            kicked[ch] = max(base, channel_kick_duty(cfg, settings, ch))
            want = rise[(0.20, 0.30, 0.50).index(base)]
            assert index(kicked) - index(quiet) == pytest.approx(want, abs=0.05), (ch, base)


def test_the_status_is_json_safe_for_the_published_payload() -> None:
    import json

    cfg = das_cfg()
    sup = Supervisor(cfg, spin_up=spin())
    Run(sup, cfg, rpm=0.0, demand=0.3).ticks(30)
    json.dumps(sup.spin_up_status(), allow_nan=False)
    assert set(sup.spin_up_status()) == set(cfg.channels)
    assert all(v["state"] in spinup.STATES for v in sup.spin_up_status().values())


# --- what a person and Home Assistant see --------------------------------------------


def test_the_health_payload_carries_the_verdict_and_the_failed_fans() -> None:
    """The rule changes a command, so it lives in ``control/``; its verdict is published
    with the fan-health ones, because that is the one payload ``/api/state``,
    ``/api/health``, the MQTT state blob and the page already read."""
    from aqua_bridge.health import FanHealthConfig, HealthMonitor

    cfg = das_cfg()
    sup = Supervisor(cfg, spin_up=spin(max_attempts=1))
    monitor = HealthMonitor(cfg, FanHealthConfig(), clock=lambda: 0.0, spin_up=sup.spin_up_status)
    payload = monitor.update({}, 0.0)
    assert payload["ok"] and payload["problems"] == []
    Run(sup, cfg, rpm={"fa1": 0.0, "fa2": 900.0, "fb1": 900.0, "fc1": 900.0}).ticks(30)
    payload = monitor.update({}, 1.0)
    assert payload["fans"]["fa1"]["spin_up"]["state"] == "failed"
    assert payload["fans"]["fa2"]["spin_up"]["state"] == "turning"
    assert len(payload["problems"]) == 1 and "fa1" in payload["problems"][0]
    assert payload["ok"] is False


def test_a_bad_spin_up_key_exits_2_before_anything_opens(tmp_path, caplog) -> None:
    import yaml

    import aqua_bridge.__main__ as main_mod

    raw = yaml.safe_load((REPO_ROOT / "config.example.yaml").read_text())
    raw["spin_up"]["max_attempts"] = 0
    path = tmp_path / "config.yaml"
    path.write_text(yaml.safe_dump(raw))
    with caplog.at_level(logging.ERROR):
        assert main_mod.main(["--config", str(path), "--source", "sim", "--once"]) == 2
    assert "spin_up.max_attempts" in caplog.text


def test_a_spin_up_channel_typo_exits_2_before_anything_opens(tmp_path, caplog) -> None:
    """The check that needs ``mpc.channels`` runs in ``main`` too, before ``build_io``
    opens anything -- the same reason ``host_health.air_temps`` does (item 103)."""
    import yaml

    import aqua_bridge.__main__ as main_mod

    raw = yaml.safe_load((REPO_ROOT / "config.example-das.yaml").read_text())
    raw["spin_up"]["channels"]["xt9_typo"] = {"kick_duty": 0.5}
    path = tmp_path / "config.yaml"
    path.write_text(yaml.safe_dump(raw))
    with caplog.at_level(logging.ERROR):
        assert main_mod.main(["--config", str(path), "--source", "sim", "--once"]) == 2
    assert "spin_up.channels names" in caplog.text and "xt9_typo" in caplog.text
