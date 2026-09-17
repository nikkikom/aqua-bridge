"""Glue loop: section 4.3 "Loop / glue", section 9 stop path, closed-loop sim."""

from __future__ import annotations

import dataclasses
import logging
import threading
from collections.abc import Callable

import pytest

from aqua_bridge.__main__ import PlantIO, make_sim_plant
from aqua_bridge.control.intents import ClearOverride, ControlMode, SetMode, SetPwm
from aqua_bridge.control.loop import Loop, Notifier, Sink, Source, TickResult, emergency_command
from aqua_bridge.control.supervisor import Supervisor
from aqua_bridge.model import Mode, MpcCommand, MpcState, PlantObservation, SolverKind
from invariants import assert_command_safe, assert_state_finite, make_obs

# --- fakes -------------------------------------------------------------------------


class FakeSource:
    """Yields scripted observations; an item may be an exception (raised) or any object."""

    def __init__(self, items=()) -> None:
        self.items = list(items)
        self.reads = 0
        self.default: Callable[[int], object] | None = None

    def read(self) -> PlantObservation:
        self.reads += 1
        if self.items:
            item = self.items.pop(0)
        elif self.default is not None:
            item = self.default(self.reads)
        else:
            raise RuntimeError("source exhausted")
        if isinstance(item, BaseException):
            raise item
        return item  # type: ignore[return-value]


class FakeSink:
    def __init__(self) -> None:
        self.applied: list[MpcCommand] = []
        self.fail_next = 0
        self.always_fail = False
        self.on_apply: Callable[[MpcCommand], None] | None = None

    def apply(self, cmd: MpcCommand) -> None:
        if self.always_fail or self.fail_next > 0:
            self.fail_next = max(0, self.fail_next - 1)
            raise OSError("usb gone")
        self.applied.append(cmd)
        if self.on_apply is not None:
            self.on_apply(cmd)


class FakeNotifier:
    def __init__(self) -> None:
        self.ready_n = 0
        self.watchdog_n = 0
        self.stopping_n = 0

    def ready(self) -> bool:
        self.ready_n += 1
        return True

    def watchdog(self) -> bool:
        self.watchdog_n += 1
        return True

    def stopping(self) -> bool:
        self.stopping_n += 1
        return True


class FakeClock:
    def __init__(self, t0: float = 1000.0) -> None:
        self.t = t0

    def __call__(self) -> float:
        return self.t


class ScriptedClock:
    """Returns each value of ``values`` in order, then repeats the last one.

    Used to inject an exact ``step()`` wall time: ``Loop`` reads the clock once
    before and once after ``step`` (module docstring "Step budget alarm"), so two
    values per tick set that tick's elapsed step time deterministically, without
    depending on how fast ``step`` actually runs on the machine running the test.
    """

    def __init__(self, values: list[float]) -> None:
        self.values = list(values)
        self.calls = 0

    def __call__(self) -> float:
        i = min(self.calls, len(self.values) - 1)
        self.calls += 1
        return self.values[i]


def good_source(cfg, *, pwm=0.5, wobble_c=0.05):
    """Trusted observations at the setpoint.

    The temperatures wobble by ``wobble_c`` (> ``stuck_eps_c``) every other
    tick: a *frozen* reading while the commanded PWM moves by more than
    ``stuck_pwm_net`` is the gate's Stuck case (section 3 rule 3), which a
    manual override ramp through the loop would trigger on perfectly constant
    synthetic temperatures. Pass ``wobble_c=0`` to get that frozen source.
    """
    src = FakeSource()

    def default(n: int) -> PlantObservation:
        w = wobble_c if n % 2 else 0.0
        temps = {name: cfg.setpoints.get(name, 30.0) + w for name in cfg.temps}
        return make_obs(cfg, float(n) * cfg.dt, temps=temps, pwm=dict.fromkeys(cfg.channels, pwm))

    src.default = default
    return src


def make_loop(cfg, source, sink=None, notifier=None, clock=None, **kw):
    sink = FakeSink() if sink is None else sink
    notifier = FakeNotifier() if notifier is None else notifier
    clock = FakeClock() if clock is None else clock
    sup = Supervisor(cfg, clock=lambda: 0.0)
    loop = Loop(source, sink, cfg, sup, clock=clock, notifier=notifier, **kw)
    return loop, sink, notifier, sup


def check_applied_chain(cfg, sink: FakeSink, results: list[TickResult], first_prev=None):
    """Every applied command satisfies the section 4.1 invariants against the one before."""
    prev = first_prev
    for r in results:
        if not r.applied:
            continue
        if prev is None:
            prev = r.mpc_cmd.diagnostics["prev_pwm"] if r.mpc_cmd else dict(cfg.fallback_pwm)
        assert_command_safe(r.obs, cfg, r.cmd, prev)
        assert_state_finite(r.state)
        prev = r.cmd.pwm


# --- protocols --------------------------------------------------------------------


def test_fakes_satisfy_protocols(fast_cfg):
    assert isinstance(FakeSource(), Source)
    assert isinstance(FakeSink(), Sink)
    assert isinstance(FakeNotifier(), Notifier)
    io = PlantIO(make_sim_plant(fast_cfg), fast_cfg)
    assert isinstance(io, Source) and isinstance(io, Sink)


def test_loop_rejects_non_config(fast_cfg):
    with pytest.raises(TypeError):
        Loop(FakeSource(), FakeSink(), {"dt": 1}, Supervisor(fast_cfg))  # type: ignore[arg-type]


# --- nominal ------------------------------------------------------------------------


def test_nominal_ticks_apply_and_kick_watchdog(fast_cfg):
    loop, sink, notifier, sup = make_loop(fast_cfg, good_source(fast_cfg))
    results = [loop.tick() for _ in range(6)]
    assert len(sink.applied) == 6
    assert all(r.ok for r in results)
    assert notifier.watchdog_n == 6
    assert notifier.ready_n == 1 and loop.ready_sent
    assert results[0].cmd.mode is Mode.AUTO
    check_applied_chain(fast_cfg, sink, results)
    assert loop.state.last_cmd == sink.applied[-1]
    s = sup.snapshot()
    assert s.usb_present is True and s.last_cmd == sink.applied[-1]
    assert s.obs == results[-1].obs
    assert s.extra["tick"] == 5


def test_first_tick_rate_limits_against_obs_pwm(fast_cfg):
    loop, sink, *_ = make_loop(fast_cfg, good_source(fast_cfg, pwm=0.4))
    r = loop.tick()
    assert_command_safe(r.obs, fast_cfg, r.cmd, dict.fromkeys(fast_cfg.channels, 0.4))


def test_ready_sent_once_and_only_after_successful_apply(fast_cfg):
    sink = FakeSink()
    sink.fail_next = 2
    loop, sink, notifier, _ = make_loop(fast_cfg, good_source(fast_cfg), sink=sink)
    loop.tick()
    loop.tick()
    assert notifier.ready_n == 0 and not loop.ready_sent
    assert notifier.watchdog_n == 2  # loop alive: watchdog still kicked
    loop.tick()
    loop.tick()
    assert notifier.ready_n == 1


# --- read() failures (section 4.3) -----------------------------------------------------


def test_read_raises_on_cold_start_commands_fallback_pwm_exactly(fast_cfg):
    loop, sink, notifier, sup = make_loop(fast_cfg, FakeSource([RuntimeError("no usb")]))
    r = loop.tick()
    assert r.read_error and "no usb" in r.read_error
    assert r.applied and len(sink.applied) == 1
    assert r.cmd.mode is Mode.FALLBACK
    assert r.cmd.pwm == dict(fast_cfg.fallback_pwm)  # section 3 item 3
    assert notifier.watchdog_n == 1
    assert sup.snapshot().usb_present is False
    assert sup.snapshot().obs is None  # garbage never reaches the snapshot


def test_read_raises_after_good_ticks_holds_then_ramps_high(fast_cfg):
    src = good_source(fast_cfg)
    loop, sink, notifier, _ = make_loop(fast_cfg, src)
    results = [loop.tick() for _ in range(3)]
    held = sink.applied[-1].pwm
    src.default = lambda n: RuntimeError("usb dropped")
    for _ in range(int(fast_cfg.fallback_hold_s) + 1):
        results.append(loop.tick())
    for r in results[3:]:
        assert r.read_error is not None
        assert r.cmd.mode is Mode.FALLBACK
        assert r.applied
    assert results[3].cmd.pwm == held  # hold, delta = 0
    # Past fallback_hold_s the ramp toward fallback_pwm starts, at d_pwm_max per tick.
    for _ in range(10):
        results.append(loop.tick())
    assert results[-1].cmd.pwm == pytest.approx(dict(fast_cfg.fallback_pwm))
    check_applied_chain(fast_cfg, sink, results)
    assert notifier.watchdog_n == len(results)


def test_read_failure_ts_continues_observation_clock(fast_cfg):
    src = good_source(fast_cfg)
    loop, *_ = make_loop(fast_cfg, src)
    loop.tick()
    last_ts = loop.last_obs.ts
    src.default = lambda n: RuntimeError("x")
    r = loop.tick()
    assert r.obs.ts == pytest.approx(last_ts + fast_cfg.dt)
    assert r.obs.temps == {} and r.obs.pwm == {}


@pytest.mark.parametrize("garbage", [None, {"temps": {}}, 42, "obs"])
def test_read_returns_non_observation_is_fallback_tick(fast_cfg, garbage):
    loop, sink, *_ = make_loop(fast_cfg, FakeSource([garbage]))
    r = loop.tick()
    assert r.read_error is not None
    assert r.cmd.mode is Mode.FALLBACK
    assert r.cmd.pwm == dict(fast_cfg.fallback_pwm)
    assert len(sink.applied) == 1


def test_read_returns_empty_observation_is_fallback_tick(fast_cfg):
    empty = PlantObservation(temps={}, rpm={}, pwm={}, ts=1.0)
    loop, sink, *_ = make_loop(fast_cfg, FakeSource([empty]))
    r = loop.tick()
    assert r.read_error is None
    assert r.cmd.mode is Mode.FALLBACK
    assert r.cmd.pwm == dict(fast_cfg.fallback_pwm)


def test_recovery_after_read_failures_needs_confirm_ticks(fast_cfg):
    src = good_source(fast_cfg)
    loop, sink, *_ = make_loop(fast_cfg, src)
    loop.tick()
    src.items = [RuntimeError("x"), RuntimeError("y")]
    results = [loop.tick(), loop.tick()]
    for _ in range(fast_cfg.confirm_ticks + 1):
        results.append(loop.tick())
    assert results[-1].cmd.mode is Mode.AUTO
    assert any(r.cmd.mode is Mode.FALLBACK for r in results[2:-1])
    check_applied_chain(fast_cfg, sink, results, first_prev=sink.applied[0].pwm)


# --- apply() failures (section 4.3) ------------------------------------------------------


def test_apply_raises_next_tick_runs_and_rate_limits_from_applied(fast_cfg):
    src = FakeSource()
    # Hot coolant so the solver wants to move up every tick.
    src.default = lambda n: make_obs(fast_cfg, float(n), coolant=45.0)
    sink = FakeSink()
    loop, sink, notifier, sup = make_loop(fast_cfg, src, sink=sink)
    r0 = loop.tick()  # bumpless first tick: output == prev
    r1 = loop.tick()
    applied = r1.cmd.pwm
    assert applied["radiator"] > r0.cmd.pwm["radiator"]
    sink.fail_next = 1
    r2 = loop.tick()
    assert r2.apply_error is not None and not r2.applied
    assert len(sink.applied) == 2
    assert r2.mpc_cmd.pwm["radiator"] > applied["radiator"]  # it wanted to move on
    assert loop.state.last_cmd == r1.cmd  # not the command that never reached the fans
    assert loop.state.window[-1].cmd_pwm == applied
    assert loop.state.solver_memory["last_ts"] == r2.obs.ts  # state still advanced
    assert notifier.watchdog_n == 3
    assert sup.snapshot().usb_present is False
    r3 = loop.tick()
    assert r3.applied
    assert_command_safe(
        r3.obs, fast_cfg, r3.cmd, applied
    )  # limit measured from what is on the fans
    for ch in fast_cfg.channels:
        assert abs(r3.cmd.pwm[ch] - applied[ch]) <= fast_cfg.d_pwm_max + 1e-9
    assert sup.snapshot().usb_present is True


def test_apply_fails_on_first_tick_leaves_last_cmd_none(fast_cfg):
    sink = FakeSink()
    sink.fail_next = 1
    loop, sink, *_ = make_loop(fast_cfg, good_source(fast_cfg, pwm=0.3), sink=sink)
    r1 = loop.tick()
    assert not r1.applied
    assert loop.state.last_cmd is None
    assert loop.applied_cmd is None
    r2 = loop.tick()
    assert r2.applied
    # prev for the second tick is obs.pwm (0.3), as on a cold start.
    assert_command_safe(r2.obs, fast_cfg, r2.cmd, dict.fromkeys(fast_cfg.channels, 0.3))
    assert loop.state.last_cmd == r2.cmd


def test_apply_always_failing_keeps_loop_alive(fast_cfg):
    sink = FakeSink()
    sink.always_fail = True
    loop, sink, notifier, _ = make_loop(fast_cfg, good_source(fast_cfg), sink=sink)
    results = [loop.tick() for _ in range(5)]
    assert all(r.apply_error for r in results)
    assert notifier.watchdog_n == 5
    assert loop.tick_count == 5
    assert loop.status()["last_ok"] is False


# --- controller bug path --------------------------------------------------------------


def test_controller_exception_applies_emergency_ramp_and_skips_watchdog(fast_cfg, monkeypatch):
    loop, sink, notifier, sup = make_loop(fast_cfg, good_source(fast_cfg))
    r1 = loop.tick()
    state_before = loop.state

    def boom(*a, **k):
        raise ZeroDivisionError("bug")

    monkeypatch.setattr(sup, "compose", boom)
    r2 = loop.tick()
    assert r2.controller_error and "ZeroDivisionError" in r2.controller_error
    assert r2.applied and r2.cmd.mode is Mode.FALLBACK
    assert r2.cmd.diagnostics["policy"] == "emergency"
    assert_command_safe(r2.obs, fast_cfg, r2.cmd, r1.cmd.pwm)
    for ch in fast_cfg.channels:
        want = fast_cfg.fallback_pwm[ch]
        assert abs(r2.cmd.pwm[ch] - want) <= abs(r1.cmd.pwm[ch] - want)  # toward fallback
    # Not half-updated: nothing the broken tick computed is committed. The applied-command
    # mirror does follow the fans, because the emergency ramp moved them -- the next tick
    # rate-limits from what is on them, not from the command they no longer carry.
    assert dataclasses.replace(loop.state, last_cmd=None, window=()) == dataclasses.replace(
        state_before, last_cmd=None, window=()
    )
    assert loop.state.last_cmd is r2.cmd
    assert notifier.watchdog_n == 1  # only the healthy tick kicked it


def test_a_good_tick_after_an_emergency_ramp_rate_limits_from_the_fans(fast_cfg, monkeypatch):
    """The emergency ramp moves the fans while the controller's own state stands still.
    The next healthy tick must measure ``d_pwm_max`` from where the fans actually are,
    or a fan the emergency walked up comes back down in one step (uncovered by the
    section 8 item 112 change, which stopped masking it on the experiment path)."""
    loop, sink, notifier, sup = make_loop(fast_cfg, good_source(fast_cfg))
    loop.tick()

    def boom(*a, **k):
        raise ZeroDivisionError("bug")

    monkeypatch.setattr(sup, "compose", boom)
    for _ in range(6):  # ramp the fans well away from the last command the solver chose
        emergency = loop.tick()
    monkeypatch.undo()
    good = loop.tick()
    for ch in fast_cfg.channels:
        assert abs(good.cmd.pwm[ch] - emergency.cmd.pwm[ch]) <= fast_cfg.d_pwm_max + 1e-9
    assert good.mpc_cmd.diagnostics["prev_pwm"] == pytest.approx(emergency.cmd.pwm)


def test_emergency_command_ramps_and_clamps(fast_cfg):
    prev = dict.fromkeys(fast_cfg.channels, 0.5)
    cmd = emergency_command(fast_cfg, prev, "x")
    assert cmd.mode is Mode.FALLBACK
    assert_command_safe(make_obs(fast_cfg, 0.0), fast_cfg, cmd, prev)
    at_target = emergency_command(fast_cfg, dict(fast_cfg.fallback_pwm), "x")
    assert at_target.pwm == dict(fast_cfg.fallback_pwm)
    missing = emergency_command(fast_cfg, {}, "x")
    assert missing.pwm == dict(fast_cfg.fallback_pwm)


def test_emergency_command_never_lowers_a_channel_above_fallback(fast_cfg):
    """Review finding F1 (loop side): the emergency ramp is up to fallback_pwm, never down."""
    cfg = fast_cfg
    high = dict.fromkeys(cfg.channels, cfg.pwm_max)
    assert all(cfg.pwm_max > cfg.fallback_pwm[ch] for ch in cfg.channels)
    assert emergency_command(cfg, high, "x").pwm == high
    mixed = {"radiator": cfg.pwm_max, "intake": 0.3}
    out = emergency_command(cfg, mixed, "x").pwm
    assert out["radiator"] == cfg.pwm_max
    assert out["intake"] == pytest.approx(0.3 + cfg.d_pwm_max)


# --- manual overrides through the loop (section 6) -----------------------------------------


def test_override_moves_at_d_pwm_max_and_state_tracks_applied(fast_cfg):
    loop, sink, notifier, sup = make_loop(fast_cfg, good_source(fast_cfg))
    results = [loop.tick(), loop.tick()]
    start = sink.applied[-1].pwm["radiator"]
    sup.submit(SetMode(ControlMode.MIXED))
    sup.submit(SetPwm("radiator", 0.9))
    for _ in range(8):
        results.append(loop.tick())
    check_applied_chain(fast_cfg, sink, results)
    assert sink.applied[-1].pwm["radiator"] == pytest.approx(0.9)
    steps = [c.pwm["radiator"] for c in sink.applied[2:]]
    assert steps[0] == pytest.approx(start + fast_cfg.d_pwm_max)
    assert loop.state.last_cmd == sink.applied[-1]
    assert loop.state.window[-1].cmd_pwm == sink.applied[-1].pwm
    assert sink.applied[-1].diagnostics["supervisor"]["control_mode"] == "mixed"


def test_release_is_bumpless(fast_cfg):
    loop, sink, notifier, sup = make_loop(fast_cfg, good_source(fast_cfg))
    loop.tick()
    sup.submit(SetMode(ControlMode.MIXED))
    sup.submit(SetPwm("radiator", 0.9))
    results = [loop.tick() for _ in range(8)]
    assert sink.applied[-1].pwm["radiator"] == pytest.approx(0.9)
    sup.submit(ClearOverride("radiator"))
    r = loop.tick()
    assert sup.control_mode is ControlMode.AUTO
    # First solver output after release equals the PWM that was on the fan (before rate limit).
    assert r.mpc_cmd.diagnostics["target_pwm"]["radiator"] == pytest.approx(0.9)
    assert r.cmd.pwm["radiator"] == pytest.approx(0.9)
    assert set(loop.state.integrator) == set(fast_cfg.channels)
    results.append(r)
    check_applied_chain(fast_cfg, sink, results, first_prev=sink.applied[0].pwm)


def test_override_on_frozen_temps_is_stuck_and_held(fast_cfg):
    """Review finding F3: a source whose temperatures never change while a manual
    override moves the PWM by more than ``stuck_pwm_net`` is the gate's Stuck
    case; the loop then holds (fallback beats manual) instead of applying the
    override. That is the intended behaviour, hence the wobble in ``good_source``."""
    cfg = fast_cfg
    loop, sink, notifier, sup = make_loop(cfg, good_source(cfg, wobble_c=0.0))
    loop.tick()
    loop.tick()
    sup.submit(SetMode(ControlMode.MIXED))
    sup.submit(SetPwm("radiator", 0.9))
    results = [loop.tick() for _ in range(8)]
    check_applied_chain(cfg, sink, results, first_prev=sink.applied[1].pwm)
    stuck = [r for r in results if r.cmd.mode is Mode.FALLBACK]
    assert stuck, "constant temperatures under a 0.4 PWM move must trip the Stuck rule"
    gate = stuck[0].mpc_cmd.diagnostics["gate"]
    assert any(gate["stuck"].values())
    assert stuck[0].cmd.diagnostics["supervisor"]["overrides_applied"] is False
    assert sink.applied[-1].pwm["radiator"] < 0.9  # the override was not applied blind


def test_fallback_beats_override_in_loop(fast_cfg):
    src = good_source(fast_cfg)
    loop, sink, notifier, sup = make_loop(fast_cfg, src)
    loop.tick()
    held = sink.applied[-1].pwm
    sup.submit(SetMode(ControlMode.MANUAL))
    sup.submit(SetPwm("radiator", fast_cfg.pwm_min))
    src.items = [RuntimeError("usb")]
    r = loop.tick()
    assert r.cmd.mode is Mode.FALLBACK
    assert r.cmd.pwm == held  # override ignored while the controller is blind
    assert r.cmd.diagnostics["supervisor"]["overrides_applied"] is False


# --- on_tick observer hook -----------------------------------------------------------------


def test_on_tick_hook_sees_every_result_and_its_errors_do_not_break_the_tick(fast_cfg):
    seen: list[TickResult] = []

    def hook(result: TickResult) -> None:
        seen.append(result)
        if len(seen) == 2:
            raise RuntimeError("publisher bug")

    loop, sink, notifier, sup = make_loop(fast_cfg, good_source(fast_cfg), on_tick=hook)
    results = [loop.tick() for _ in range(3)]
    assert seen == results
    assert all(r.applied for r in results) and notifier.watchdog_n == 3
    assert sup.snapshot().last_cmd == results[-1].cmd  # hook runs after the supervisor update


# --- shutdown (section 9) -------------------------------------------------------------


def test_shutdown_writes_fallback_once_then_notifies_stopping(fast_cfg):
    loop, sink, notifier, sup = make_loop(fast_cfg, good_source(fast_cfg))
    loop.tick()
    assert loop.shutdown() is True
    assert sink.applied[-1].pwm == dict(fast_cfg.fallback_pwm)
    assert sink.applied[-1].mode is Mode.FALLBACK
    assert sink.applied[-1].diagnostics["policy"] == "shutdown"
    assert notifier.stopping_n == 1
    assert loop.shutdown() is False  # idempotent: one write
    assert len(sink.applied) == 2
    assert notifier.stopping_n == 1
    assert sup.snapshot().extra["shutdown"] is True


def test_shutdown_survives_sink_failure(fast_cfg):
    sink = FakeSink()
    sink.always_fail = True
    loop, sink, notifier, _ = make_loop(fast_cfg, good_source(fast_cfg), sink=sink)
    assert loop.shutdown() is False
    assert notifier.stopping_n == 1


# --- run() scheduling ---------------------------------------------------------------------


def test_run_max_ticks_without_real_sleep(fast_cfg):
    sleeps: list[float] = []

    def fake_sleep(seconds: float, stop: threading.Event) -> None:
        sleeps.append(seconds)
        clock.t += seconds

    loop, sink, notifier, _ = make_loop(fast_cfg, good_source(fast_cfg), sleep=fake_sleep)
    clock = loop.clock
    n = loop.run(threading.Event(), max_ticks=5)
    assert n == 5 and len(sink.applied) == 5
    assert len(sleeps) == 4
    assert all(s == pytest.approx(fast_cfg.dt) for s in sleeps)


def test_run_stops_on_event_set_from_sink(fast_cfg):
    stop = threading.Event()
    sink = FakeSink()
    sink.on_apply = lambda cmd: stop.set() if len(sink.applied) >= 3 else None
    loop, sink, *_ = make_loop(fast_cfg, good_source(fast_cfg), sink=sink, sleep=lambda s, e: None)
    n = loop.run(stop)
    assert n == 3
    assert loop.shutdown_done is False  # run() never shuts down by itself


def test_run_overrun_resets_schedule_instead_of_bursting(fast_cfg):
    sleeps: list[float] = []

    class SlowSource(FakeSource):
        def read(self):
            clock.t += 3 * fast_cfg.dt  # a tick that takes far longer than dt
            return super().read()

    src = SlowSource()
    src.default = lambda n: make_obs(fast_cfg, float(n))
    loop, sink, *_ = make_loop(
        fast_cfg, src, sleep=lambda s, e: sleeps.append(s) or setattr(clock, "t", clock.t + s)
    )
    clock = loop.clock
    loop.run(threading.Event(), max_ticks=3)
    assert len(sink.applied) == 3
    assert sleeps == []  # always behind: never sleeps, never doubles up ticks


def test_run_with_preset_event_returns_zero(fast_cfg):
    stop = threading.Event()
    stop.set()
    loop, sink, *_ = make_loop(fast_cfg, good_source(fast_cfg))
    assert loop.run(stop) == 0
    assert sink.applied == []


# --- closed loop against the simulator (section 4.6) -----------------------------------------


@pytest.mark.slow
def test_sim_closed_loop_200_ticks_every_applied_command_safe(cfg):
    plant = make_sim_plant(cfg, heat_w=100.0, seed=1)
    io = PlantIO(plant, cfg)
    notifier = FakeNotifier()
    sup = Supervisor(cfg)
    loop = Loop(io, io, cfg, sup, clock=FakeClock(), notifier=notifier)
    results: list[TickResult] = []
    for _ in range(200):
        results.append(loop.tick())
    assert all(r.ok for r in results)
    assert len(io.applied) == 200
    check_applied_chain(cfg, io, results)
    assert notifier.watchdog_n == 200 and notifier.ready_n == 1
    assert loop.state.last_cmd == io.applied[-1]
    tail = results[-50:]
    assert all(r.cmd.mode in (Mode.AUTO, Mode.SATURATED) for r in tail)
    # The PI regulates toward the setpoint (loosely: within 3 C after ~7 minutes of sim time).
    assert abs(tail[-1].obs.temps["coolant"] - cfg.setpoints["coolant"]) < 3.0


@pytest.mark.slow
def test_sim_closed_loop_with_override_and_release(cfg):
    plant = make_sim_plant(cfg, heat_w=100.0, seed=2)
    io = PlantIO(plant, cfg)
    sup = Supervisor(cfg)
    loop = Loop(io, io, cfg, sup, clock=FakeClock(), notifier=FakeNotifier())
    results = [loop.tick() for _ in range(40)]
    sup.submit(SetMode(ControlMode.MIXED))
    sup.submit(SetPwm("intake", 0.3))
    results += [loop.tick() for _ in range(40)]
    assert io.applied[-1].pwm["intake"] == pytest.approx(0.3)
    sup.submit(ClearOverride())
    results += [loop.tick() for _ in range(60)]
    assert all(r.ok for r in results)
    check_applied_chain(cfg, io, results)


def test_status(fast_cfg):
    loop, *_ = make_loop(fast_cfg, good_source(fast_cfg))
    assert loop.status()["ticks"] == 0 and loop.status()["last_ok"] is None
    loop.tick()
    st = loop.status()
    assert st["ticks"] == 1 and st["last_ok"] is True and st["in_fault"] is False
    assert set(st["applied_pwm"]) == set(fast_cfg.channels)


def test_initial_state_can_be_injected(fast_cfg):
    state = MpcState.cold()
    loop, *_ = make_loop(fast_cfg, good_source(fast_cfg), state=state)
    assert loop.state is state


# --- step budget alarm (module docstring "Step budget alarm") ---------------------


def test_step_budget_tracks_last_and_max_and_reaches_health(fast_cfg):
    cfg = dataclasses.replace(fast_cfg, budget_ms=1000.0, budget_alarm_ms=2000.0)
    # 3 ticks, elapsed step times 80 ms, 10 ms, 5 ms: max is not the last tick's value.
    clock = ScriptedClock([0.000, 0.080, 0.080, 0.090, 0.090, 0.095])
    loop, _sink, _notifier, sup = make_loop(cfg, good_source(cfg), clock=clock)
    for _ in range(3):
        loop.tick()
    assert loop.step_ms_last == pytest.approx(5.0)
    assert loop.step_ms_max == pytest.approx(80.0)
    assert loop.budget_warn_count == 0
    assert loop.budget_alarm_count == 0
    health = sup.snapshot().health_payload()
    assert health["step_ms_last"] == pytest.approx(5.0)
    assert health["step_ms_max"] == pytest.approx(80.0)
    assert health["budget_warn_count"] == 0
    assert health["budget_alarm_count"] == 0


def test_step_budget_exceedance_not_measured_when_step_raises(fast_cfg, monkeypatch):
    """A tick whose ``step`` raises leaves the counters untouched (module docstring)."""
    cfg = dataclasses.replace(fast_cfg, budget_ms=1000.0, budget_alarm_ms=2000.0)
    clock = ScriptedClock([0.0, 5.0])  # would be a 5000 ms step, well past both thresholds
    loop, _sink, _notifier, _sup = make_loop(cfg, good_source(cfg), clock=clock)
    monkeypatch.setattr(
        "aqua_bridge.control.loop.step", lambda *a, **k: (_ for _ in ()).throw(RuntimeError("x"))
    )
    result = loop.tick()
    assert result.controller_error is not None
    assert loop.step_ms_last == 0.0 and loop.budget_warn_count == 0


def test_step_budget_warns_past_budget_ms_and_errors_past_budget_alarm_ms(fast_cfg, caplog):
    cfg = dataclasses.replace(
        fast_cfg, budget_ms=50.0, budget_alarm_ms=100.0, budget_log_interval_s=5.0
    )
    # Ticks 1-3 warn only (60, 70, 80 ms); tick 4 alarms (150 ms). "now" (the clock
    # value after step) advances across ticks like real wall time so the interval
    # gate has something to compare against.
    clock = ScriptedClock(
        [
            0.000,
            0.060,  # tick 1: 60 ms, first warning ever -> logged
            1.000,
            1.070,  # tick 2: 70 ms, 1.01 s since the last warning -> rate limited
            6.500,
            6.580,  # tick 3: 80 ms, 6.52 s since the last warning -> logged
            7.000,
            7.150,  # tick 4: 150 ms, past the alarm budget -> logged as an error
        ]
    )
    loop, *_ = make_loop(cfg, good_source(cfg), clock=clock)
    with caplog.at_level(logging.WARNING, logger="aqua_bridge.loop"):
        for _ in range(4):
            loop.tick()
    assert loop.budget_warn_count == 4  # every tick exceeded budget_ms, alarm tick included
    assert loop.budget_alarm_count == 1
    warnings = [r for r in caplog.records if r.levelno == logging.WARNING]
    errors = [r for r in caplog.records if r.levelno == logging.ERROR]
    assert len(warnings) == 2, [r.getMessage() for r in warnings]
    assert len(errors) == 1, [r.getMessage() for r in errors]
    assert "1 exceedance" in warnings[0].getMessage()
    assert "2 exceedance" in warnings[1].getMessage()
    assert "1 exceedance" in errors[0].getMessage()


def test_step_budget_lines_name_the_config_keys_that_decide_them(fast_cfg, caplog):
    """Both lines name the key that was exceeded and the keys to change (item 73)."""
    cfg = dataclasses.replace(
        fast_cfg, budget_ms=50.0, budget_alarm_ms=100.0, budget_log_interval_s=0.0001
    )
    clock = ScriptedClock([0.0, 0.060, 1.0, 1.150])  # tick 1: 60 ms (warn); tick 2: 150 ms (alarm)
    loop, *_ = make_loop(cfg, good_source(cfg), clock=clock)
    with caplog.at_level(logging.WARNING, logger="aqua_bridge.loop"):
        loop.tick()
        loop.tick()
    warning = next(r.getMessage() for r in caplog.records if r.levelno == logging.WARNING)
    error = next(r.getMessage() for r in caplog.records if r.levelno == logging.ERROR)
    assert "mpc.budget_ms 50.0 ms" in warning
    assert "mpc.budget_alarm_ms 100.0 ms" in error
    for message in (warning, error):
        assert "raise mpc.budget_ms and mpc.budget_alarm_ms above it" in message
        # a legacy config: mpc_every_ticks is inert there, so the line does not name it
        assert "mpc_every_ticks" not in message


def test_the_das_mpc_step_budget_line_names_mpc_every_ticks(das_example_cfg, caplog):
    """With the DAS MPC, solving less often is the other lever (item 73)."""
    cfg = dataclasses.replace(
        das_example_cfg, solver=SolverKind.MPC, budget_ms=50.0, budget_alarm_ms=100.0
    )
    loop, *_ = make_loop(cfg, FakeSource(), clock=ScriptedClock([0.0, 0.060]))
    with caplog.at_level(logging.WARNING, logger="aqua_bridge.loop"):
        loop._record_step_budget(60.0, cfg, now=0.060)
    message = next(r.getMessage() for r in caplog.records if r.levelno == logging.WARNING)
    assert "solve less often with mpc.mpc_every_ticks" in message
    # the PI-like DAS form does not replay a plan, so it is not offered that lever
    caplog.clear()
    with caplog.at_level(logging.WARNING, logger="aqua_bridge.loop"):
        loop._record_step_budget(60.0, dataclasses.replace(cfg, solver=SolverKind.PI), now=99.0)
    assert "mpc_every_ticks" not in next(
        r.getMessage() for r in caplog.records if r.levelno == logging.WARNING
    )


def test_a_raised_budget_ms_moves_both_thresholds_with_it(fast_cfg, caplog):
    """The documented Zero W fallback (item 73): the alarm reads the config, nothing else.

    ``mpc.budget_ms`` must stay below ``mpc.budget_alarm_ms``, so raising it to
    1000 ms means raising the alarm above that as well -- 1250 ms keeps the
    stock 1.25 ratio (``tests/test_bench_budget.py`` checks the config itself).
    """
    cfg = dataclasses.replace(
        fast_cfg, budget_ms=1000.0, budget_alarm_ms=1250.0, budget_log_interval_s=0.0001
    )
    # 900 ms (inside the raised budget), 1100 ms (warns), 1300 ms (alarms)
    clock = ScriptedClock([0.0, 0.900, 10.0, 11.1, 20.0, 21.3])
    loop, *_ = make_loop(cfg, good_source(cfg), clock=clock)
    with caplog.at_level(logging.WARNING, logger="aqua_bridge.loop"):
        loop.tick()
        assert loop.budget_warn_count == 0 and not caplog.records
        loop.tick()
        assert (loop.budget_warn_count, loop.budget_alarm_count) == (1, 0)
        loop.tick()
        assert (loop.budget_warn_count, loop.budget_alarm_count) == (2, 1)
    assert [r.levelno for r in caplog.records] == [logging.WARNING, logging.ERROR]


def test_step_budget_counters_and_last_reach_the_mqtt_state_blob(fast_cfg):
    """``ControlSnapshot.to_dict()`` (the MQTT state blob) carries ``health``."""
    cfg = dataclasses.replace(fast_cfg, budget_ms=10.0, budget_alarm_ms=20.0)
    clock = ScriptedClock([0.0, 0.015])  # one tick, 15 ms: past budget_ms, not the alarm
    loop, *_ = make_loop(cfg, good_source(cfg), clock=clock)
    loop.tick()
    sup = loop.supervisor
    blob = sup.snapshot().to_dict()
    assert blob["health"]["step_ms_last"] == pytest.approx(15.0)
    assert blob["health"]["budget_warn_count"] == 1
    assert blob["health"]["budget_alarm_count"] == 0
