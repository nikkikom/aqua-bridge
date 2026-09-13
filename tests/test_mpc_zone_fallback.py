"""Per-zone fallback invariants (plan section 0.1, section 9 ``test_mpc_zone_fallback.py``).

* a fault in zone A never lowers a channel of ``F*(A)`` below ``prev``: hold
  ``prev`` for ``fallback_hold_s``, then ``max(prev, fallback_pwm)``;
* channels outside ``F*`` keep regulating;
* the solver never sees the sensors of a faulted zone;
* recovery is per zone after ``confirm_ticks`` and bumpless on the returning channels;
* Flicker in one zone never resets another zone's timer;
* a dropout inside a redundant group is no fault;
* a solver fault faults every zone it drives; every zone in fault is ``fallback``.

Every tick runs through :func:`invariants.checked_step`, which for a zoned
config also asserts :func:`invariants.assert_zone_step_safe` (mode per zone
rule, aggregates, fallback channels = reach of the faulted zones, none of
them below ``prev``). Fixture: ``das_fixtures`` (za <-> zb coupled, zc alone).
"""

from __future__ import annotations

import dataclasses
import math
import random
from collections.abc import Callable, Mapping
from typing import Any

import pytest
from hypothesis import given, settings
from hypothesis import strategies as st

from aqua_bridge.control.intents import ControlMode, SetMode, SetPwm
from aqua_bridge.control.loop import Loop
from aqua_bridge.control.mpc import step
from aqua_bridge.control.solver_pi import PiSolver, SolverRequest, SolverResult
from aqua_bridge.control.supervisor import Supervisor
from aqua_bridge.model import (
    FaultReason,
    Mode,
    MpcCommand,
    MpcConfig,
    MpcState,
    PlantObservation,
)
from das_fixtures import CHANNELS, SP, das_cfg, das_obs, default_temps
from invariants import TOL, checked_step

ZONE_SENSORS = {
    "za": ("air_a", "air_a2", "prox_a1", "prox_a1b", "prox_a2"),
    "zb": ("air_b", "prox_b1", "exhaust"),
    "zc": ("air_c", "prox_c1"),
}


@pytest.fixture
def dcfg() -> MpcConfig:
    return das_cfg()


class Tick(dict):
    """One recorded tick: ``obs``, ``prev`` (what the command was limited against), ``cmd``,
    ``state``."""


def drive(
    cfg: MpcConfig,
    temps_at: Callable[[int], Mapping[str, float | None]],
    ticks: int,
    *,
    state: MpcState | None = None,
    t0: int = 0,
    pwm: float = 0.5,
    **kwargs: Any,
) -> list[Tick]:
    """Closed on the command: ``obs.pwm`` echoes the previous command (first tick: ``pwm``)."""
    state = MpcState.cold() if state is None else state
    out: list[Tick] = []
    for i in range(t0, t0 + ticks):
        temps = dict(temps_at(i))
        obs_pwm = dict(state.last_cmd.pwm) if state.last_cmd is not None else pwm
        obs = das_obs(cfg, float(i) * cfg.dt, temps=temps, pwm=obs_pwm)
        prev = dict(state.last_cmd.pwm) if state.last_cmd is not None else None
        cmd, state = checked_step(obs, cfg, state, **kwargs)
        out.append(Tick(obs=obs, prev=prev, cmd=cmd, state=state))
    return out


def with_temps(cfg: MpcConfig, **patch: float | None) -> dict[str, float | None]:
    t = default_temps(cfg)
    t.update(patch)
    return t


def ramp(i: int, t0: int, rate: float = 0.5, top: float = 3.0) -> float:
    """``SP`` rising by ``rate`` degC per tick from tick ``t0`` up to ``SP + top`` (no jump)."""
    return SP + min(top, max(0.0, (i - t0 + 1) * rate))


def settled(cfg: MpcConfig, ticks: int = 5, pwm: float = 0.5, **kwargs: Any) -> MpcState:
    return drive(cfg, lambda i: default_temps(cfg), ticks, pwm=pwm, **kwargs)[-1]["state"]


# ---------------------------------------------------------------------------
# hold then high on F*, solver elsewhere
# ---------------------------------------------------------------------------


def test_zone_fault_holds_then_ramps_every_channel_of_the_closure(dcfg):
    state = settled(dcfg)
    base = dict(state.last_cmd.pwm)
    assert all(v == pytest.approx(0.5) for v in base.values())
    # za loses its setpoint sensor at t=5; zc gets hot meanwhile.
    rec = drive(
        dcfg, lambda i: with_temps(dcfg, air_a=None, air_c=ramp(i, 5)), 12, state=state, t0=5
    )
    for n, tick in enumerate(rec):
        cmd, prev = tick["cmd"], tick["prev"]
        assert cmd.mode is Mode.DEGRADED
        assert cmd.diagnostics["zones_in_fault"] == ["za"]
        assert cmd.diagnostics["fallback_channels"] == ["fa1", "fa2", "fb1"]
        elapsed = float(n) * dcfg.dt
        for ch in ("fa1", "fa2", "fb1"):
            if elapsed <= dcfg.fallback_hold_s:
                assert cmd.pwm[ch] == prev[ch], f"{ch} moved during hold at +{elapsed}s"
                assert cmd.diagnostics["policy_by_channel"][ch] == "hold"
            else:
                assert cmd.pwm[ch] == pytest.approx(min(0.8, prev[ch] + dcfg.d_pwm_max))
                assert cmd.diagnostics["policy_by_channel"][ch] == "ramp_high"
        assert cmd.diagnostics["policy_by_channel"]["fc1"] == "solver"
        assert cmd.diagnostics["zones"]["zb"]["policy"] == "coupled"
        assert cmd.diagnostics["zones"]["zb"]["in_closure"] is True
    final = rec[-1]["cmd"].pwm
    assert final["fa1"] == final["fa2"] == final["fb1"] == pytest.approx(0.8)
    # zc kept regulating: its channel rose on its own zone's error.
    assert final["fc1"] > base["fc1"] + 0.01
    assert rec[-1]["cmd"].diagnostics["policy"] == "mixed"


def test_zone_fault_never_lowers_a_channel_already_above_fallback(dcfg):
    state = settled(dcfg, pwm=1.0)

    def hot(i: int) -> dict[str, float | None]:
        return {name: ramp(i, 5, top=8.0) for name in dcfg.setpoints}

    state = drive(dcfg, lambda i: with_temps(dcfg, **hot(i)), 30, state=state, t0=5)[-1]["state"]
    assert all(v == pytest.approx(1.0) for v in state.last_cmd.pwm.values())
    rec = drive(
        dcfg, lambda i: with_temps(dcfg, **hot(i), prox_b1=None), 15, state=state, t0=35, pwm=1.0
    )
    for tick in rec:
        for ch in ("fa1", "fa2", "fb1"):
            assert tick["cmd"].pwm[ch] == pytest.approx(1.0)
    assert rec[-1]["cmd"].diagnostics["policy_by_channel"]["fb1"] == "ramp_high"


def test_isolated_zone_fault_only_touches_its_own_channel(dcfg):
    state = settled(dcfg)
    rec = drive(
        dcfg, lambda i: with_temps(dcfg, air_c=math.nan, air_a=ramp(i, 5)), 10, state=state, t0=5
    )
    for tick in rec:
        assert tick["cmd"].diagnostics["fallback_channels"] == ["fc1"]
    fa1 = [t["cmd"].pwm["fa1"] for t in rec]
    assert fa1[-1] > fa1[0]  # za regulates up on its own error
    assert rec[-1]["cmd"].pwm["fc1"] == pytest.approx(0.8)


def test_fault_coupling_none_keeps_the_neighbour_regulating():
    cfg = das_cfg(zones={"fault_coupling": "none"})
    state = settled(cfg)
    rec = drive(cfg, lambda i: with_temps(cfg, air_a=None, air_b=ramp(i, 5)), 10, state=state, t0=5)
    for tick in rec:
        assert tick["cmd"].diagnostics["fallback_channels"] == ["fa1", "fa2"]
        assert tick["cmd"].diagnostics["policy_by_channel"]["fb1"] == "solver"
    assert rec[-1]["cmd"].pwm["fb1"] > 0.51


# ---------------------------------------------------------------------------
# the solver never sees a faulted zone
# ---------------------------------------------------------------------------


class SpySolver:
    """PiSolver that records every request."""

    name = "pi"

    def __init__(self) -> None:
        self.inner = PiSolver()
        self.requests: list[SolverRequest] = []
        self.inits: list[SolverRequest] = []

    def initialise(self, cfg: MpcConfig, req: SolverRequest):
        self.inits.append(req)
        return self.inner.initialise(cfg, req)

    def solve(self, cfg: MpcConfig, req: SolverRequest) -> SolverResult:
        self.requests.append(req)
        return self.inner.solve(cfg, req)


def test_solver_request_excludes_faulted_zone_sensors(dcfg):
    spy = SpySolver()
    state = settled(dcfg, solver=spy)
    spy.requests.clear()
    drive(dcfg, lambda i: with_temps(dcfg, prox_a2=None), 8, state=state, t0=5, solver=spy)
    assert len(spy.requests) == 8
    for req in spy.requests:
        assert not set(req.temps) & set(ZONE_SENSORS["za"])
        assert set(req.temps) >= {"air_b", "air_c", "inlet", "prox_b1"}
        assert set(req.fixed_channels) == {"fa1", "fa2", "fb1"}
        assert req.zone_trust == {"za": False, "zb": True, "zc": True}
        for v in req.fixed_channels.values():
            assert v >= 0.5 - TOL
    held = spy.requests[0].fixed_channels
    assert held == {"fa1": pytest.approx(0.5), "fa2": pytest.approx(0.5), "fb1": pytest.approx(0.5)}


@pytest.mark.parametrize(
    "garbage",
    [
        {"air_a": None},
        {"air_a": 500.0, "prox_a2": math.nan},
        {"air_a": -40.0, "air_a2": None, "prox_a1": None, "prox_a1b": None},
    ],
)
def test_healthy_zone_commands_do_not_depend_on_faulted_zone_values(dcfg, garbage):
    state = settled(dcfg)

    def temps(extra: Mapping[str, float | None]) -> Callable[[int], dict[str, float | None]]:
        return lambda i: with_temps(dcfg, air_c=SP + 1.0 + 0.1 * i, **extra)

    ref = drive(dcfg, temps({"air_a": None}), 12, state=state, t0=5)
    other = drive(dcfg, temps(garbage), 12, state=state, t0=5)
    for a, b in zip(ref, other, strict=True):
        assert a["cmd"].pwm["fc1"] == b["cmd"].pwm["fc1"]
        assert a["cmd"].diagnostics["zones_in_fault"] == ["za"]
        assert b["cmd"].diagnostics["zones_in_fault"] == ["za"]


# ---------------------------------------------------------------------------
# recovery, flicker, redundancy
# ---------------------------------------------------------------------------


def test_recovery_per_zone_after_confirm_ticks_is_bumpless(dcfg):
    spy = SpySolver()
    state = settled(dcfg, solver=spy)
    faulted = drive(
        dcfg, lambda i: with_temps(dcfg, air_b=None), 10, state=state, t0=5, solver=spy
    )[-1]
    assert faulted["cmd"].pwm["fb1"] == pytest.approx(0.8)
    assert "fb1" not in faulted["state"].integrator and "fa1" not in faulted["state"].integrator
    assert "fc1" in faulted["state"].integrator
    spy.inits.clear()
    rec = drive(
        dcfg,
        lambda i: with_temps(dcfg, air_a=ramp(i, 15), air_b=ramp(i, 15)),
        4,
        state=faulted["state"],
        t0=15,
        solver=spy,
    )
    first, second = rec[0], rec[1]
    assert first["cmd"].mode is Mode.DEGRADED  # 1 of confirm_ticks=2
    assert first["cmd"].diagnostics["fallback_channels"] == ["fa1", "fa2", "fb1"]
    assert second["cmd"].mode is Mode.AUTO
    assert second["state"].zone_faults["zb"].since_ts is None
    assert second["cmd"].diagnostics["returning_to_auto"] is True
    # Bumpless on the returning channels: the first solver output is what is on the fans.
    for ch in ("fa1", "fa2", "fb1"):
        assert second["cmd"].pwm[ch] == pytest.approx(second["prev"][ch])
        assert ch in second["state"].integrator
    assert len(spy.inits) == 1
    # Then they regulate on the (hot) zone error: up from the fallback level.
    assert rec[-1]["cmd"].pwm["fa1"] > second["cmd"].pwm["fa1"]


def test_flicker_in_one_zone_never_resets_another_zone(dcfg):
    state = settled(dcfg)

    def temps(i: int) -> dict[str, float | None]:
        patch: dict[str, float | None] = {"air_c": None}  # zc down for good from t=5
        if i % 2 == 1:
            patch["air_a"] = None  # za flickers
        return with_temps(dcfg, **patch)

    rec = drive(dcfg, temps, 16, state=state, t0=5)
    zc_since = {t["state"].zone_faults["zc"].since_ts for t in rec}
    assert zc_since == {5.0}
    za = [t["state"].zone_faults["za"] for t in rec]
    assert {f.since_ts for f in za[1:]} == {5.0}  # Flicker never restarts its own hold either
    assert max(f.streak for f in za) < dcfg.confirm_ticks  # and never confirms
    assert rec[-1]["cmd"].pwm["fc1"] == pytest.approx(0.8)
    assert rec[-1]["cmd"].pwm["fa1"] == pytest.approx(0.8)
    assert all(t["cmd"].mode is Mode.DEGRADED for t in rec)  # zb never faulted


def test_dropout_inside_a_redundant_group_is_no_fault(dcfg):
    def temps(i: int) -> dict[str, float | None]:
        patch: dict[str, float | None] = {"air_a2": None, "prox_c1": None, "exhaust": None}
        if i % 3:
            patch["prox_a1b"] = math.nan
        else:
            patch["prox_a1"] = None
        return with_temps(dcfg, **patch)

    rec = drive(dcfg, temps, 20)
    assert all(t["cmd"].mode is Mode.AUTO for t in rec)
    assert all(not f.in_fault for f in rec[-1]["state"].zone_faults.values())


def test_every_zone_in_fault_is_fallback_and_recovers(dcfg):
    state = settled(dcfg)
    rec = drive(dcfg, lambda i: {}, 8, state=state, t0=5)  # empty temps: every zone blind
    assert all(t["cmd"].mode is Mode.FALLBACK for t in rec)
    assert rec[-1]["cmd"].pwm == pytest.approx(dict.fromkeys(CHANNELS, 0.8))
    back = drive(dcfg, lambda i: default_temps(dcfg), 3, state=rec[-1]["state"], t0=13)
    assert [t["cmd"].mode for t in back] == [Mode.FALLBACK, Mode.AUTO, Mode.AUTO]


def test_all_channels_fixed_by_coupling_clears_the_confirmed_zone_without_solver():
    m = das_cfg().to_dict()
    m["topology"]["zones"]["zc"]["coupled_to"] = []
    cfg = MpcConfig.from_mapping(m)
    state = settled(cfg)
    # za, zb, zc all fault; then za recovers while zb and zc stay down.
    down = drive(
        cfg, lambda i: with_temps(cfg, air_a=None, air_b=None, air_c=None), 3, state=state, t0=5
    )
    rec = drive(
        cfg, lambda i: with_temps(cfg, air_b=None, air_c=None), 3, state=down[-1]["state"], t0=8
    )
    assert rec[1]["state"].zone_faults["za"].since_ts is None  # confirmed
    assert rec[1]["cmd"].mode is Mode.DEGRADED
    assert rec[1]["cmd"].diagnostics["solver_ran"] is False  # every channel still fixed
    assert rec[1]["cmd"].diagnostics["fallback_channels"] == list(CHANNELS)


# ---------------------------------------------------------------------------
# solver faults
# ---------------------------------------------------------------------------


class BoomSolver:
    name = "pi"

    def __init__(self, fail: Callable[[SolverRequest], bool] = lambda req: True) -> None:
        self.fail = fail
        self.inner = PiSolver()

    def initialise(self, cfg, req):
        return self.inner.initialise(cfg, req)

    def solve(self, cfg, req):
        if self.fail(req):
            raise RuntimeError("boom")
        return self.inner.solve(cfg, req)


def test_solver_fault_faults_every_zone_it_drives(dcfg):
    state = settled(dcfg)
    rec = drive(dcfg, lambda i: default_temps(dcfg), 8, state=state, t0=5, solver=BoomSolver())
    first = rec[0]
    assert first["cmd"].mode is Mode.FALLBACK
    assert {f.reason for f in first["state"].zone_faults.values()} == {FaultReason.SOLVER}
    assert first["cmd"].pwm == pytest.approx(first["prev"])
    assert rec[-1]["cmd"].pwm == pytest.approx(dict.fromkeys(CHANNELS, 0.8))
    assert all(f.since_ts == 5.0 for f in rec[-1]["state"].zone_faults.values())


def test_solver_fault_keeps_the_sensor_reason_of_a_zone_already_down(dcfg):
    state = settled(dcfg)
    rec = drive(
        dcfg,
        lambda i: with_temps(dcfg, air_c=None),
        3,
        state=state,
        t0=5,
        solver=BoomSolver(lambda req: bool(req.fixed_channels)),
    )
    zf = rec[0]["state"].zone_faults
    assert zf["zc"].reason is FaultReason.SENSOR_GATE
    assert zf["za"].reason is FaultReason.SOLVER and zf["zb"].reason is FaultReason.SOLVER
    assert rec[0]["cmd"].mode is Mode.FALLBACK
    assert rec[0]["state"].fault_reason is FaultReason.SOLVER  # earliest (t=5, first zone)


def test_legacy_mpc_solver_turns_a_zone_fault_into_whole_fallback():
    cfg = das_cfg(solver="mpc", weight_dpwm=60.0)
    state = settled(cfg)
    assert state.last_cmd.mode is Mode.AUTO
    rec = drive(cfg, lambda i: with_temps(cfg, air_c=None), 10, state=state, t0=5)
    assert rec[0]["cmd"].mode is Mode.FALLBACK
    assert "fallback policy" in (rec[0]["cmd"].diagnostics["solver_error"] or "")
    assert rec[-1]["cmd"].pwm == pytest.approx(dict.fromkeys(CHANNELS, 0.8))
    back = drive(cfg, lambda i: default_temps(cfg), 4, state=rec[-1]["state"], t0=15)
    assert back[-1]["cmd"].mode is Mode.AUTO


# ---------------------------------------------------------------------------
# glue: Loop + Supervisor with overrides during a zone fault
# ---------------------------------------------------------------------------


class ScriptSource:
    def __init__(self, cfg: MpcConfig, temps_at: Callable[[int], Mapping[str, float | None]]):
        self.cfg = cfg
        self.temps_at = temps_at
        self.i = 0
        self.pwm: dict[str, float] = dict.fromkeys(cfg.channels, 0.5)

    def read(self) -> PlantObservation:
        obs = das_obs(self.cfg, float(self.i), temps=self.temps_at(self.i), pwm=self.pwm)
        self.i += 1
        return obs


class EchoSink:
    def __init__(self, source: ScriptSource) -> None:
        self.source = source
        self.applied: list[MpcCommand] = []

    def apply(self, cmd: MpcCommand) -> None:
        self.applied.append(cmd)
        self.source.pwm = dict(cmd.pwm)


def test_loop_override_is_blocked_only_on_channels_under_fallback(dcfg):
    source = ScriptSource(
        dcfg, lambda i: with_temps(dcfg, air_b=None) if i >= 5 else default_temps(dcfg)
    )
    sink = EchoSink(source)
    sup = Supervisor(dcfg)
    loop = Loop(source, sink, dcfg, sup)
    for _ in range(5):
        loop.tick()
    sup.submit(SetMode(ControlMode.MIXED))
    sup.submit(SetPwm("fa1", dcfg.pwm_min))
    sup.submit(SetPwm("fc1", dcfg.pwm_min))
    for _ in range(10):
        result = loop.tick()
        prev = sink.applied[-2].pwm
        cmd = result.cmd
        assert cmd.mode is Mode.DEGRADED
        assert cmd.pwm["fa1"] >= prev["fa1"] - TOL  # blocked: zb fault reaches fa1
        assert cmd.pwm["fc1"] <= prev["fc1"] + TOL  # applied: zc is healthy
    assert sink.applied[-1].pwm["fc1"] == pytest.approx(dcfg.pwm_min)
    assert sink.applied[-1].pwm["fa1"] == pytest.approx(0.8)
    assert result.cmd.diagnostics["supervisor"]["overrides_blocked"] == ["fa1"]


# ---------------------------------------------------------------------------
# property: random per-zone lies keep every invariant
# ---------------------------------------------------------------------------

LIES = ("ok", "ok", "ok", "none", "nan", "drop", "range", "jump", "extra")


def _lie(kind: str, temps: dict[str, float | None], name: str) -> None:
    if kind == "none":
        temps[name] = None
    elif kind == "nan":
        temps[name] = math.nan
    elif kind == "drop":
        temps.pop(name, None)
    elif kind == "range":
        temps[name] = 500.0
    elif kind == "jump":
        v = temps.get(name)
        temps[name] = None if v is None else v + 30.0
    elif kind == "extra":
        temps["gpu"] = 45.0


class _Chooser:
    """Random choices for :func:`_random_run`: Hypothesis draws, or a seeded ``random.Random``
    for long runs (a 200-tick run of draws is too large a base example to shrink)."""

    def __init__(self, data: st.DataObject | None = None, seed: int | None = None) -> None:
        self.data = data
        self.rng = random.Random(seed)

    def uniform(self, lo: float, hi: float) -> float:
        if self.data is not None:
            return self.data.draw(st.floats(lo, hi))
        return self.rng.uniform(lo, hi)

    def randint(self, lo: int, hi: int) -> int:
        if self.data is not None:
            return self.data.draw(st.integers(lo, hi))
        return self.rng.randint(lo, hi)

    def choice(self, items: tuple[str, ...]) -> str:
        if self.data is not None:
            return self.data.draw(st.sampled_from(items))
        return self.rng.choice(items)


def _random_run(
    cfg: MpcConfig, pick: _Chooser, ticks: int, *, clean_tail: int = 0, **kwargs: Any
) -> MpcState:
    """``ticks`` ticks of drifting temperatures with up to three lies per tick, then
    ``clean_tail`` honest ticks; every tick checked and replayed for determinism."""
    state = MpcState.cold()
    temps = default_temps(cfg)
    obs_pwm: float | dict[str, float] = pick.uniform(0.15, 1.0)
    for i in range(ticks + clean_tail):
        for name in cfg.temps:
            if temps[name] is not None:
                temps[name] = float(temps[name]) + pick.uniform(-0.3, 0.3)
        lied = dict(temps)
        ts = float(i) * cfg.dt
        if i < ticks:
            for _ in range(pick.randint(0, 3)):
                _lie(pick.choice(LIES), lied, pick.choice(cfg.temps))
            if pick.randint(0, 30) == 0:
                ts = max(0.0, ts - cfg.dt)  # a stalled clock now and then
        obs = das_obs(cfg, ts, temps=lied, pwm=obs_pwm)
        cmd, nxt = checked_step(obs, cfg, state, **kwargs)
        again = step(obs, cfg, state, **kwargs)
        assert again[0].to_dict() == cmd.to_dict() and again[1].to_dict() == nxt.to_dict()
        state = nxt
        obs_pwm = dict(cmd.pwm)
    return state


@pytest.mark.fuzzy
@given(data=st.data())
def test_random_zone_lies_keep_the_per_zone_invariants(data):
    _random_run(das_cfg(), _Chooser(data), data.draw(st.integers(5, 30), label="ticks"))


@pytest.mark.fuzzy
@given(data=st.data())
def test_random_zone_lies_without_coupling_keep_the_invariants(data):
    _random_run(das_cfg(zones={"fault_coupling": "none"}), _Chooser(data), 15)


@pytest.mark.parametrize("solver", ["pi", "mpc"])
@pytest.mark.parametrize("seed", [1, 2])
def test_seeded_zone_lies_then_recovery(solver, seed):
    cfg = das_cfg(solver=solver, weight_dpwm=60.0)
    state = _random_run(cfg, _Chooser(seed=seed), 60, clean_tail=cfg.confirm_ticks + 4)
    assert state.last_cmd.mode in (Mode.AUTO, Mode.SATURATED)
    assert not any(f.in_fault for f in state.zone_faults.values())


@pytest.mark.nightly
@pytest.mark.fuzzy
@settings(max_examples=100)
@given(seed=st.integers(0, 2**32 - 1), solver=st.sampled_from(["pi", "mpc"]))
def test_random_zone_lies_long_runs_both_solvers(seed, solver):
    cfg = das_cfg(solver=solver, weight_dpwm=60.0)
    state = _random_run(cfg, _Chooser(seed=seed), 400, clean_tail=cfg.confirm_ticks + 4)
    assert state.last_cmd.mode in (Mode.AUTO, Mode.SATURATED)


def test_zone_state_survives_dataclass_replace_of_the_config(dcfg):
    """A preset (``dataclasses.replace``) mid-fault keeps the per-zone timers."""
    state = drive(dcfg, lambda i: with_temps(dcfg, air_c=None), 3)[-1]["state"]
    moved = dataclasses.replace(dcfg, setpoints={"air_a": 33.0, "air_b": 33.0, "air_c": 33.0})
    cmd, nxt = checked_step(das_obs(moved, 3.0, air_c=None, pwm=state.last_cmd.pwm), moved, state)
    assert nxt.zone_faults["zc"].since_ts == 0.0 and cmd.mode is Mode.DEGRADED
