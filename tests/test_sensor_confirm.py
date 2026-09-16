"""Sensor confirmation with zones: a rejected value confirms like any other sensor.

A sensor whose value the gate rejects (``range``, ``slew``, ``stuck``) is
confirming until it has been gate-trusted on ``confirm_ticks`` consecutive
ticks (``zones.advance_confirmation``). A redundant group member that jumps is
excluded from the estimator, the solver and ``last_good_obs`` while it confirms,
its group stays trusted through the other members and the zone does not fault.
A sole member still costs ``confirm_ticks`` once (the zone's own confirmation),
not twice. Legacy mode has no confirmation state.

The counterfactual used below: until the jump confirms, a run where the member
jumps must look to the estimator exactly like a run where the member reads
nothing at all.
"""

from __future__ import annotations

import dataclasses
import json
from collections.abc import Callable, Mapping
from typing import Any
from unittest import mock

import pytest
from hypothesis import given, settings
from hypothesis import strategies as st

from aqua_bridge.control import estimator, zones
from aqua_bridge.control.gate import evaluate_gate
from aqua_bridge.control.mpc import solver_for
from aqua_bridge.model import Mode, MpcConfig, MpcState, PlantObservation
from das_fixtures import PROX_C, SP, das_cfg, das_obs, default_temps
from invariants import checked_step

#: Confirmation long enough to see the intermediate ticks (``confirm_s <= fallback_hold_s``).
CONFIRM_S = 4.0
#: A level change far above the fixture's slew limit (2 degC per tick).
JUMP_C = 15.0
SETTLE_TICKS = 6


@pytest.fixture
def ccfg() -> MpcConfig:
    cfg = das_cfg(confirm_s=CONFIRM_S)
    assert cfg.confirm_ticks == 4
    return cfg


def run(
    cfg: MpcConfig,
    temps_at: Callable[[int], Mapping[str, float | None]],
    ticks: int,
    *,
    state: MpcState | None = None,
    t0: int = 0,
    make_obs: Callable[[MpcConfig, float, dict[str, float | None], Any], PlantObservation]
    | None = None,
) -> list[tuple[Any, MpcState]]:
    """Closed on the command, every tick through ``checked_step``."""
    state = MpcState.cold() if state is None else state
    out = []
    for i in range(t0, t0 + ticks):
        pwm = dict(state.last_cmd.pwm) if state.last_cmd is not None else 0.5
        temps = dict(temps_at(i))
        if make_obs is None:
            obs = das_obs(cfg, float(i) * cfg.dt, temps=temps, pwm=pwm)
        else:
            obs = make_obs(cfg, float(i) * cfg.dt, temps, pwm)
        cmd, state = checked_step(obs, cfg, state)
        out.append((cmd, state))
    return out


def patched(cfg: MpcConfig, **patch: float | None) -> dict[str, float | None]:
    t = default_temps(cfg)
    t.update(patch)
    return t


def settled(cfg: MpcConfig) -> MpcState:
    return run(cfg, lambda i: default_temps(cfg), SETTLE_TICKS)[-1][1]


def estimator_memory(state: MpcState) -> str:
    return json.dumps(state.solver_memory["estimator"], sort_keys=True)


# ---------------------------------------------------------------------------
# a redundant member that jumps
# ---------------------------------------------------------------------------


@pytest.mark.parametrize("member", ["prox_a1b", "air_a2", "inlet"])
def test_a_jumping_redundant_member_is_not_fused_until_it_confirms(ccfg, member):
    base = settled(ccfg)
    level = default_temps(ccfg)[member] + JUMP_C  # type: ignore[operator]
    n = ccfg.confirm_ticks + 3
    jumped = run(ccfg, lambda i: patched(ccfg, **{member: level}), n, state=base, t0=SETTLE_TICKS)
    silent = run(ccfg, lambda i: patched(ccfg, **{member: None}), n, state=base, t0=SETTLE_TICKS)

    # the jump tick is a gate rejection, the next confirm_ticks - 1 ticks confirm
    for k, (cmd, _state) in enumerate(jumped):
        confirming = cmd.diagnostics["sensor_confirm"]
        if k < ccfg.confirm_ticks:
            assert confirming == {member: k}, k
        else:
            assert confirming == {}, k
        assert cmd.mode is Mode.AUTO, k  # the zone never faults
        assert cmd.diagnostics["zones_in_fault"] == []

    # until it confirms the estimator sees exactly what it sees without the member
    for k in range(ccfg.confirm_ticks):
        assert estimator_memory(jumped[k][1]) == estimator_memory(silent[k][1]), k
        assert jumped[k][0].pwm == silent[k][0].pwm, k
        good = jumped[k][1].last_good_obs
        assert good is not None and good.temps[member] == default_temps(ccfg)[member], k
    # the confirmed level change is fused on its confirm_ticks-th trusted tick
    k = ccfg.confirm_ticks
    assert estimator_memory(jumped[k][1]) != estimator_memory(silent[k][1])
    assert jumped[k][1].last_good_obs.temps[member] == level  # type: ignore[union-attr]


def test_a_confirming_sensor_is_not_reported_trusted_in_diagnostics(ccfg):
    """Item 63: ``diagnostics["gate"]["per_temp"]`` must not read ``True`` for a
    confirming sensor, even on a tick after the jump where its raw reading is already
    self-consistent (would otherwise pass the gate cleanly) -- HTTP and MQTT read this
    dict directly and must not show a value the estimator and the solver both ignore."""
    base = settled(ccfg)
    member = "prox_a1b"
    level = default_temps(ccfg)[member] + JUMP_C  # type: ignore[operator]
    n = ccfg.confirm_ticks + 2
    rec = run(ccfg, lambda i: patched(ccfg, **{member: level}), n, state=base, t0=SETTLE_TICKS)
    for k, (cmd, _state) in enumerate(rec):
        confirming = cmd.diagnostics["sensor_confirm"]
        per_temp = cmd.diagnostics["gate"]["per_temp"]
        if member in confirming:
            assert per_temp[member] is False, k
        else:
            assert per_temp[member] is True, k
    assert ccfg.confirm_ticks > 1  # the loop above exercised at least one clean-but-confirming tick


def test_the_group_stays_trusted_through_the_other_member(ccfg):
    base = settled(ccfg)
    rec = run(
        ccfg,
        lambda i: patched(ccfg, prox_a1b=PROX_C + JUMP_C),
        ccfg.confirm_ticks,
        state=base,
        t0=SETTLE_TICKS,
    )
    for cmd, state in rec:
        assert cmd.diagnostics["zones"]["za"]["trusted"] is True
        assert cmd.diagnostics["zones"]["za"]["reasons"] == []
        assert state.zone_faults["za"].since_ts is None
        assert cmd.diagnostics["solver_ran"] is True


def test_a_confirming_member_that_jumps_again_restarts(ccfg):
    base = settled(ccfg)

    def temps(i: int) -> dict[str, float | None]:
        k = i - SETTLE_TICKS
        return patched(ccfg, prox_a1b=PROX_C + (JUMP_C if k < 2 else 2 * JUMP_C))

    rec = run(ccfg, temps, ccfg.confirm_ticks + 3, state=base, t0=SETTLE_TICKS)
    counts = [cmd.diagnostics["sensor_confirm"].get("prox_a1b") for cmd, _ in rec]
    assert counts == [0, 1, 0, 1, 2, 3, None]
    assert all(cmd.mode is Mode.AUTO for cmd, _ in rec)


def test_a_dropout_starts_no_confirmation(ccfg):
    base = settled(ccfg)

    def temps(i: int) -> dict[str, float | None]:
        return patched(ccfg, prox_a1b=None if i == SETTLE_TICKS else PROX_C)

    rec = run(ccfg, temps, 3, state=base, t0=SETTLE_TICKS)
    assert all(cmd.diagnostics["sensor_confirm"] == {} for cmd, _ in rec)


def test_a_sensor_missing_since_boot_confirms_on_its_first_reading(ccfg):
    """Item 61: unlike the dropout above (which returns to a value ``last_good_obs``
    already remembers), a sensor with no reference at all -- missing since boot, while
    the rest of the system is already past its own cold start -- gets no free pass: its
    first reading confirms like any other return instead of fusing on trust alone."""
    missing_ticks = 3

    def temps(i: int) -> dict[str, float | None]:
        return patched(ccfg, prox_a1b=None if i < missing_ticks else PROX_C)

    rec = run(ccfg, temps, missing_ticks + ccfg.confirm_ticks + 2)
    for k, (cmd, state) in enumerate(rec):
        confirming = cmd.diagnostics["sensor_confirm"]
        good = state.last_good_obs
        if missing_ticks <= k < missing_ticks + ccfg.confirm_ticks:
            assert confirming.get("prox_a1b") == k - missing_ticks, k
            assert good is None or good.temps.get("prox_a1b") is None, k
        else:
            assert "prox_a1b" not in confirming, k
        assert cmd.mode is Mode.AUTO, k  # bay a1's other member (prox_a1) covers the zone
    fused_tick = missing_ticks + ccfg.confirm_ticks
    assert rec[fused_tick][1].last_good_obs.temps["prox_a1b"] == PROX_C  # type: ignore[union-attr]


def test_a_sensor_missing_since_boot_confirms_while_every_zone_is_blind(ccfg):
    """Item 61, the case ``last_good_obs is not None`` alone would miss: while every zone
    is in fault nothing is fused, so ``state.last_good_obs`` stays ``None`` for the whole
    outage and not just on tick 0. A zone-air sensor that comes back then -- a 1-Wire bus
    that was not up at boot, a DS18B20 whose first post-reset read is the 85 degC
    power-on value, which passes the range check and, with no reference, the slew check
    too -- must still confirm. This is the blindest the system ever is; it is not the
    moment to fuse a first reading on trust alone."""
    missing_ticks = 3

    def temps(i: int) -> dict[str, float | None]:
        out = patched(ccfg, air_a=None, air_b=None, air_c=None)
        if i >= missing_ticks:
            out["air_a"] = SP
        return out

    rec = run(ccfg, temps, missing_ticks + ccfg.confirm_ticks + 2)
    for k, (cmd, state) in enumerate(rec):
        confirming = cmd.diagnostics["sensor_confirm"]
        if k < missing_ticks:
            assert state.last_good_obs is None, k  # nothing trusted yet, every zone blind
            assert confirming == {}, k
        elif k < missing_ticks + ccfg.confirm_ticks:
            assert state.last_good_obs is None, k  # still blind: the reading is not fused
            assert confirming == {"air_a": k - missing_ticks}, k
        else:
            assert "air_a" not in confirming, k
    fused = rec[missing_ticks + ccfg.confirm_ticks][1].last_good_obs
    assert fused is not None and fused.temps["air_a"] == SP
    assert rec[0][0].diagnostics["zones_in_fault"] == ["za", "zb", "zc"]


def test_a_never_referenced_first_reading_confirms_on_a_time_faulted_tick(ccfg):
    """Item 61: a gate rejection starts a confirmation whatever the time status, and a
    first reading with no reference must not differ -- otherwise one badly timed tick
    (the observation repeats its timestamp) is enough for the sensor to be referenced
    from the next tick on and never confirm at all. The count still cannot *advance* on a
    time-faulted tick; it only starts there."""
    missing_ticks = 3

    def temps(i: int) -> dict[str, float | None]:
        return patched(ccfg, prox_a1b=None if i < missing_ticks else PROX_C)

    def make_obs(
        cfg: MpcConfig, ts: float, temps: dict[str, float | None], pwm: Any
    ) -> PlantObservation:
        if round(ts / cfg.dt) == missing_ticks:  # the returning tick repeats its timestamp
            ts = float(missing_ticks - 1) * cfg.dt
        return das_obs(cfg, ts, temps=temps, pwm=pwm)

    rec = run(ccfg, temps, missing_ticks + ccfg.confirm_ticks + 2, make_obs=make_obs)
    assert rec[missing_ticks][0].diagnostics["time"]["status"] == "not_advancing"
    counts = [cmd.diagnostics["sensor_confirm"].get("prox_a1b") for cmd, _ in rec]
    assert counts == [None, None, None, 0, 1, 2, 3, None, None]
    good = rec[missing_ticks + ccfg.confirm_ticks - 1][1].last_good_obs
    assert good is None or good.temps.get("prox_a1b") is None  # not fused while confirming


def test_a_dropout_while_confirming_restarts_the_count(ccfg):
    base = settled(ccfg)
    level = PROX_C + JUMP_C

    def temps(i: int) -> dict[str, float | None]:
        return patched(ccfg, prox_a1b=None if i == SETTLE_TICKS + 2 else level)

    rec = run(ccfg, temps, ccfg.confirm_ticks + 4, state=base, t0=SETTLE_TICKS)
    counts = [cmd.diagnostics["sensor_confirm"].get("prox_a1b") for cmd, _ in rec]
    # the dropout restarts the count, and the value that returns is gated against the last
    # good one, which the confirming level never replaced: a rejection once more
    assert counts == [0, 1, 0, 0, 1, 2, 3, None]
    assert all(cmd.mode is Mode.AUTO for cmd, _ in rec)


def test_losing_the_confirmed_member_while_the_other_confirms_faults_the_zone(ccfg):
    base = settled(ccfg)
    level = PROX_C + JUMP_C

    def temps(i: int) -> dict[str, float | None]:
        k = i - SETTLE_TICKS
        return patched(ccfg, prox_a1b=level, prox_a1=None if k >= 1 else PROX_C)

    rec = run(ccfg, temps, ccfg.confirm_ticks + 2, state=base, t0=SETTLE_TICKS)
    assert rec[0][0].mode is Mode.AUTO
    assert rec[1][0].diagnostics["zones_in_fault"] == ["za"]  # no confirmed member of bay a1
    assert "prox_a1b=confirming" in rec[1][0].diagnostics["zones"]["za"]["reasons"][0]
    # the zone's confirmation starts on the next tick and runs beside the member's, which
    # confirms first: the zone returns after confirm_ticks, not after two confirmations
    modes = [cmd.mode for cmd, _ in rec]
    assert modes == [Mode.AUTO] + [Mode.DEGRADED] * ccfg.confirm_ticks + [Mode.AUTO]
    assert rec[ccfg.confirm_ticks][0].diagnostics["sensor_confirm"] == {}


# ---------------------------------------------------------------------------
# a sole member: the zone's confirmation, once
# ---------------------------------------------------------------------------


def test_a_sole_member_jump_still_confirms_in_confirm_ticks(ccfg):
    base = settled(ccfg)
    rec = run(
        ccfg,
        lambda i: patched(ccfg, prox_a2=PROX_C + JUMP_C),
        ccfg.confirm_ticks + 2,
        state=base,
        t0=SETTLE_TICKS,
    )
    modes = [cmd.mode for cmd, _ in rec]
    # jump tick + confirm_ticks - 1 confirming ticks in fault, back on the confirm_ticks-th
    assert modes == [Mode.DEGRADED] * ccfg.confirm_ticks + [Mode.AUTO] * 2
    assert rec[ccfg.confirm_ticks][0].diagnostics["sensor_confirm"] == {}
    assert rec[ccfg.confirm_ticks - 1][0].diagnostics["sensor_confirm"] == {"prox_a2": 3}


def test_a_zone_in_fault_waits_for_a_confirmed_member_in_every_group(ccfg):
    # za faults on its setpoint sensor; while it confirms, prox_a1b jumps and prox_a1 drops:
    # the streak runs on the confirming member, the return waits for it to confirm.
    base = settled(ccfg)

    def temps(i: int) -> dict[str, float | None]:
        k = i - SETTLE_TICKS
        patch: dict[str, float | None] = {}
        if k == 0:
            patch["air_a"] = None
        if k >= 1:
            patch["prox_a1b"] = PROX_C + JUMP_C
        if k >= 2:
            patch["prox_a1"] = None
        return patched(ccfg, **patch)

    rec = run(ccfg, temps, 10, state=base, t0=SETTLE_TICKS)
    modes = [cmd.mode for cmd, _ in rec]
    # air_a returns at k=1, so the streak reaches confirm_ticks at k=4; prox_a1b (jump at k=1)
    # confirms only at k=5, and until then bay a1 has no confirmed member
    assert rec[4][0].diagnostics["zones"]["za"]["trusted_streak"] == ccfg.confirm_ticks
    assert rec[4][0].diagnostics["sensor_confirm"] == {"prox_a1b": 3}
    assert modes[:5] == [Mode.DEGRADED] * 5
    assert modes[5] is Mode.AUTO
    assert rec[5][0].diagnostics["sensor_confirm"] == {}


# ---------------------------------------------------------------------------
# DAS example config: the estimates the solver regulates on
# ---------------------------------------------------------------------------


def test_example_das_redundant_proximal_jump_leaves_the_estimates_alone(das_example_cfg):
    cfg = das_example_cfg
    assert cfg.regulates_drive_limits and cfg.sensors["prox_b03b"].redundant
    temps0 = {name: 30.0 if cfg.sensors[name].role in ("inlet",) else 36.0 for name in cfg.temps}
    temps0.update({n: 40.0 for n in cfg.temps if cfg.sensors[n].role == "drive_proximal"})

    def obs_at(
        c: MpcConfig, ts: float, temps: dict[str, float | None], pwm: Any
    ) -> PlantObservation:
        p = dict.fromkeys(c.channels, float(pwm)) if isinstance(pwm, int | float) else dict(pwm)
        return PlantObservation(temps=temps, rpm=dict.fromkeys(c.channels, 1000.0), pwm=p, ts=ts)

    settle = 4
    base = run(cfg, lambda i: dict(temps0), settle, make_obs=obs_at)[-1][1]
    n = cfg.confirm_ticks + 1
    jumped = run(
        cfg, lambda i: {**temps0, "prox_b03b": 60.0}, n, state=base, t0=settle, make_obs=obs_at
    )
    silent = run(
        cfg, lambda i: {**temps0, "prox_b03b": None}, n, state=base, t0=settle, make_obs=obs_at
    )
    for k in range(cfg.confirm_ticks):
        cmd_j, cmd_s = jumped[k][0], silent[k][0]
        assert cmd_j.diagnostics["zones_in_fault"] == []
        assert cmd_j.diagnostics["estimates"] == cmd_s.diagnostics["estimates"], k
        assert cmd_j.pwm == cmd_s.pwm, k
    k = cfg.confirm_ticks
    assert jumped[k][0].diagnostics["sensor_confirm"] == {}
    assert (
        jumped[k][0].diagnostics["estimates"]["b03"] != silent[k][0].diagnostics["estimates"]["b03"]
    )


# ---------------------------------------------------------------------------
# the pure function, state and legacy mode
# ---------------------------------------------------------------------------


def _gate(cfg: MpcConfig, **patch: float | None):
    obs = das_obs(cfg, 10.0, **patch)
    good = das_obs(cfg, 9.0)
    return evaluate_gate(obs, cfg, last_good_obs=good, last_raw_temps=good.temps, window=())


def test_advance_confirmation_rules(ccfg):
    jump = _gate(ccfg, prox_a1b=PROX_C + JUMP_C)
    clean = _gate(ccfg)
    drop = _gate(ccfg, prox_a1b=None)
    assert zones.advance_confirmation(None, jump, "ok", ccfg) == {"prox_a1b": 0}
    assert zones.advance_confirmation({}, drop, "ok", ccfg) == {}
    assert zones.advance_confirmation({"prox_a1b": 1}, clean, "ok", ccfg) == {"prox_a1b": 2}
    assert zones.advance_confirmation({"prox_a1b": 3}, clean, "ok", ccfg) == {}
    assert zones.advance_confirmation({"prox_a1b": 2}, drop, "ok", ccfg) == {"prox_a1b": 0}
    assert zones.advance_confirmation({"prox_a1b": 2}, clean, "gap", ccfg) == {"prox_a1b": 0}
    # malformed memory: never a shorter confirmation than one trusted tick
    for bad in (-1, 1.5, True, "2", None):
        assert zones.advance_confirmation({"prox_a1b": bad}, clean, "ok", ccfg) == {"prox_a1b": 1}
    assert zones.advance_confirmation({"prox_a1b": 10**9}, clean, "ok", ccfg) == {}
    assert zones.advance_confirmation({"prox_a1b": 10**9}, drop, "ok", ccfg) == {"prox_a1b": 0}
    assert zones.advance_confirmation(["prox_a1b"], clean, "ok", ccfg) == {}
    assert zones.advance_confirmation({"not_a_sensor": 1}, clean, "ok", ccfg) == {}


def test_confirmation_state_is_json_and_deterministic(ccfg):
    base = settled(ccfg)
    temps = patched(ccfg, prox_a1b=PROX_C + JUMP_C)
    a = run(ccfg, lambda i: temps, 2, state=base, t0=SETTLE_TICKS)[-1][1]
    b = run(ccfg, lambda i: temps, 2, state=base, t0=SETTLE_TICKS)[-1][1]
    assert a.solver_memory["sensor_confirm"] == {"prox_a1b": 1}
    blob = json.dumps(a.to_dict(), allow_nan=False)
    again = MpcState.from_dict(json.loads(blob))
    assert again.to_dict() == a.to_dict() == b.to_dict()
    obs = das_obs(ccfg, float(SETTLE_TICKS + 2), temps=temps, pwm=dict(a.last_cmd.pwm))  # type: ignore[union-attr]
    assert checked_step(obs, ccfg, again)[1].to_dict() == checked_step(obs, ccfg, a)[1].to_dict()


def test_legacy_mode_keeps_no_confirmation_state(fast_cfg):
    cfg = fast_cfg
    state = MpcState.cold()
    name = cfg.temps[0]
    for i in range(6):
        value = 40.0 + (JUMP_C if i >= 3 else 0.0)
        obs = PlantObservation(
            temps={t: (value if t == name else 40.0) for t in cfg.temps},
            rpm=dict.fromkeys(cfg.channels, 1000.0),
            pwm=dict.fromkeys(cfg.channels, 0.5),
            ts=float(i) * cfg.dt,
        )
        cmd, state = checked_step(obs, cfg, state)
        assert "sensor_confirm" not in state.solver_memory
        assert "sensor_confirm" not in cmd.diagnostics
    assert zones.advance_confirmation({name: 1}, _legacy_gate(cfg), "ok", cfg) == {}


def _legacy_gate(cfg: MpcConfig):
    obs = PlantObservation(
        temps=dict.fromkeys(cfg.temps, 99.0),
        rpm={},
        pwm=dict.fromkeys(cfg.channels, 0.5),
        ts=1.0,
    )
    good = dataclasses.replace(obs, temps=dict.fromkeys(cfg.temps, 40.0), ts=0.0)
    return evaluate_gate(obs, cfg, last_good_obs=good, last_raw_temps=good.temps, window=())


# ---------------------------------------------------------------------------
# property: nothing unconfirmed reaches the estimator, the solver or last_good_obs
# ---------------------------------------------------------------------------

#: Per-tick events on a sensor: its nominal value, the offset level, a dropout.
_EVENTS = st.sampled_from(["nominal", "nominal", "nominal", "level", "drop"])
_WATCHED = ("inlet", "air_a2", "prox_a1b", "prox_a1", "prox_a2")
PROPERTY_TICKS = 16


class _RecordingSolver:
    """Delegates to the configured solver and records every request's temperatures."""

    def __init__(self, inner: Any) -> None:
        self._inner = inner
        self.name = inner.name
        self.temps: list[set[str]] = []

    def initialise(self, cfg: MpcConfig, req: Any) -> Any:
        return self._inner.initialise(cfg, req)

    def solve(self, cfg: MpcConfig, req: Any) -> Any:
        self.temps.append(set(req.temps))
        return self._inner.solve(cfg, req)


@settings(max_examples=40)
@given(
    events=st.lists(
        st.fixed_dictionaries({name: _EVENTS for name in _WATCHED}),
        min_size=PROPERTY_TICKS,
        max_size=PROPERTY_TICKS,
    ),
    offset=st.sampled_from([JUMP_C, -JUMP_C, 3.0]),
    median3=st.booleans(),
    repeated_ts=st.sets(st.integers(1, PROPERTY_TICKS - 1), max_size=3),
)
def test_no_unconfirmed_value_is_ever_used(events, offset, median3, repeated_ts):
    cfg = das_cfg(confirm_s=CONFIRM_S, median3=median3)
    state = settled(cfg)
    nominal = default_temps(cfg)
    solver = _RecordingSolver(solver_for(cfg))
    seen: list[set[str]] = []
    real_update = estimator.update

    def spy(*args: Any, **kwargs: Any) -> Any:
        seen.append(set(kwargs["temps"]))
        return real_update(*args, **kwargs)

    expected: dict[str, int] = {}
    with mock.patch.object(estimator, "update", spy):
        for k, tick in enumerate(events):
            temps: dict[str, float | None] = dict(nominal)
            for name, event in tick.items():
                if event == "level":
                    temps[name] = nominal[name] + offset  # type: ignore[operator]
                elif event == "drop":
                    temps[name] = None
            pwm = dict(state.last_cmd.pwm)  # type: ignore[union-attr]
            ts = float(SETTLE_TICKS + k - (1 if k in repeated_ts else 0)) * cfg.dt  # a time fault
            obs = das_obs(cfg, ts, temps=temps, pwm=pwm)
            before = state.last_good_obs
            n_solves = len(solver.temps)
            cmd, state = checked_step(obs, cfg, state, solver=solver)
            gate = cmd.diagnostics["gate"]

            # the reference counter, from the gate's *raw* verdict (item 63 makes the
            # outward "per_temp" false while confirming, so the raw check is read from
            # "reasons" instead: empty means the gate itself did not reject the value)
            # and the time status alone
            time_ok = cmd.diagnostics["time"]["status"] in ("ok", "first")

            def raw_trusted(name: str, gate: Mapping[str, Any] = gate) -> bool:
                return not gate["reasons"].get(name)

            for name in cfg.temps:
                if set(gate["reasons"].get(name, ())) & {"range", "slew", "stuck"}:
                    expected[name] = 0
                elif name in expected:
                    if raw_trusted(name) and time_ok:
                        expected[name] += 1
                        if expected[name] >= cfg.confirm_ticks:
                            del expected[name]
                    else:
                        expected[name] = 0
            confirming = cmd.diagnostics["sensor_confirm"]
            assert confirming == expected, k

            assert len(seen) == k + 1 and not seen[-1] & set(confirming), k
            if len(solver.temps) > n_solves:
                assert not solver.temps[-1] & set(confirming), k
            for name in confirming:
                old = None if before is None else before.temps.get(name)
                new = None if state.last_good_obs is None else state.last_good_obs.temps.get(name)
                assert new == old, (k, name)
                # item 63: the outward diagnostics never call a confirming sensor trusted,
                # even though its raw reading may have passed the gate cleanly this tick
                assert gate["per_temp"][name] is False, (k, name)
            # with every sole member trusted and a confirmed trusted member on bay a1, za is
            # trusted: a redundant member, confirming or not, never faults the zone by itself
            groups_ok = (
                raw_trusted("air_a")
                and raw_trusted("prox_a2")
                and any(raw_trusted(m) and m not in confirming for m in ("prox_a1", "prox_a1b"))
            )
            if groups_ok and time_ok:
                assert cmd.diagnostics["zones"]["za"]["trusted"] is True, k
