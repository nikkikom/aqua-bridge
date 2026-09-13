"""Section 4.4 lying sensors, injected on top of a nominal closed loop.

Every row of the section 4.4 table, the gate rules of section 3 (Stuck,
Spike, Jump with both ``median3`` settings, Flicker, noise) and the
hold-then-ramp-high fallback policy. Written from PROJECT.md; the code was
consulted only for names (``diagnostics["trusted"]``, ``diagnostics["gate"]``
and the ``evaluate_gate`` / ``push_window`` gate API).

Every closed-loop tick runs through :func:`invariants.checked_step`.
Every gate-related test is parametrised over ``median3`` through ``gcfg``.
"""

from __future__ import annotations

import copy
import dataclasses
import json
import math
import statistics

import pytest

from aqua_bridge.control.gate import evaluate_gate, push_window
from aqua_bridge.model import FaultReason, Mode, MpcConfig, PlantObservation
from aqua_bridge.sim.plant import Plant, PlantParams, TickRecord, run_closed_loop
from invariants import TOL, checked_step, make_obs

# Legacy-shaped scenarios (coolant setpoint): the DAS cases run in test_das_core.py.
pytestmark = pytest.mark.solver_cases("pi", "mpc")

SP = 35.0


@pytest.fixture
def cfg(cfg: MpcConfig, solver_kind) -> MpcConfig:
    """Section 8: every scenario in this module runs for the PI and the MPC solver."""
    return dataclasses.replace(cfg, solver=solver_kind)


COOLANT = "coolant"
AIR = "air"
DROP = object()  # sentinel: remove the key from obs.temps


@pytest.fixture(params=[False, True], ids=["median3_off", "median3_on"])
def gcfg(cfg: MpcConfig, request) -> MpcConfig:
    """Example config with ``median3`` in both settings (section 3 / 4.4)."""
    return dataclasses.replace(cfg, median3=request.param)


# ---------------------------------------------------------------------------
# helpers
# ---------------------------------------------------------------------------


_SETTLED: dict[str, tuple[Plant, list[TickRecord]]] = {}


def settled(cfg: MpcConfig, *, ticks: int = 900, seed: int = 0, **params):
    """A nominal loop regulated at the setpoint; returns ``(plant, records)``.

    The 900-tick settle is identical for every test with the same config and
    plant, so it runs once per configuration and each test gets its own deep
    copy of the plant (records are frozen dataclasses and can be shared).
    """
    heat = params.pop("heat_w", 100.0)
    key = json.dumps(
        {"cfg": cfg.to_dict(), "ticks": ticks, "seed": seed, "heat": heat, "p": repr(params)},
        sort_keys=True,
    )
    if key not in _SETTLED:
        plant = Plant(
            PlantParams(dt=cfg.dt, heat_w=heat, **params),
            initial_pwm=0.5,
            t_coolant=40.0,
            t_air=30.0,
            seed=seed,
        )
        recs = run_closed_loop(plant, cfg, checked_step, ticks)
        assert all(r.cmd.mode is not Mode.FALLBACK for r in recs), "nominal loop must not fault"
        assert abs(recs[-1].obs.temps[COOLANT] - SP) < 0.6
        _SETTLED[key] = (plant, recs)
    plant, recs = _SETTLED[key]
    return copy.deepcopy(plant), recs


def continue_run(plant, cfg, recs, ticks, hook=None, heat_schedule=()) -> list[TickRecord]:
    return run_closed_loop(
        plant,
        cfg,
        checked_step,
        ticks,
        state=recs[-1].state,
        observe_hook=hook,
        heat_schedule=heat_schedule,
    )


def patched(obs: PlantObservation, **changes) -> PlantObservation:
    """``obs`` with temperature changes applied (``DROP`` removes a key)."""
    temps = dict(obs.temps)
    for k, v in changes.items():
        if v is DROP:
            temps.pop(k, None)
        else:
            temps[k] = v
    return dataclasses.replace(obs, temps=temps)


def trusted(rec: TickRecord) -> bool:
    return bool(rec.cmd.diagnostics["trusted"])


def untrusted_ticks(recs: list[TickRecord]) -> list[int]:
    return [i for i, r in enumerate(recs) if not trusted(r)]


def fallback_ticks(recs: list[TickRecord]) -> list[int]:
    return [i for i, r in enumerate(recs) if r.cmd.mode is Mode.FALLBACK]


def stuck_flag(rec: TickRecord, name: str) -> bool:
    return bool(rec.cmd.diagnostics["gate"]["stuck"].get(name, False))


def surfaces_at(cfg: MpcConfig) -> int:
    """Ticks after injection at which a persisting lie surfaces (median3 shifts by one)."""
    return 1 if cfg.median3 else 0


def assert_hold_then_high(episode: list[TickRecord], cfg: MpcConfig, prev: dict[str, float]):
    """Section 3 safe PWM: hold ``prev`` for ``fallback_hold_s``, then ramp toward
    ``max(prev, fallback_pwm)`` at ``d_pwm_max`` per step; never a step toward
    ``pwm_min`` because of the fault (a channel already above ``fallback_pwm``
    is held, never pulled down to it); ``mode=fallback`` throughout; one fault
    timer.
    """
    assert episode, "episode must contain at least the first fault tick"
    since = episode[0].state.fault_since_ts
    assert since is not None
    assert since == pytest.approx(episode[0].obs.ts)
    for r in episode:
        assert r.cmd.mode is Mode.FALLBACK
        assert r.state.fault_since_ts == since, "fault timer restarted inside one fault"
    for ch in cfg.channels:
        target = max(prev[ch], cfg.fallback_pwm[ch])
        lo, hi = prev[ch], target
        last = prev[ch]
        for r in episode:
            v = r.cmd.pwm[ch]
            elapsed = r.obs.ts - since
            assert lo - TOL <= v <= hi + TOL, (
                f"{ch}: {v:.4f} left [{lo:.3f}, {hi:.3f}] (prev -> ramp target) during fallback"
            )
            if elapsed < cfg.fallback_hold_s - TOL:
                assert v == pytest.approx(prev[ch], abs=TOL), f"{ch}: moved during the hold"
            elif elapsed > cfg.fallback_hold_s + TOL:
                remaining = abs(target - last)
                assert abs(v - last) == pytest.approx(min(cfg.d_pwm_max, remaining), abs=TOL), (
                    f"{ch}: ramp toward fallback_pwm not at d_pwm_max per step"
                )
            last = v


def first_index(recs: list[TickRecord], pred) -> int | None:
    return next((i for i, r in enumerate(recs) if pred(r)), None)


def cmd_before(base: list[TickRecord], recs: list[TickRecord], i: int) -> dict[str, float]:
    """The command the tick at ``recs[i]`` must hold: the previous tick's command."""
    return dict((recs[i - 1] if i > 0 else base[-1]).cmd.pwm)


def good_before(base: list[TickRecord], recs: list[TickRecord], i: int) -> dict:
    return dict((recs[i - 1] if i > 0 else base[-1]).state.last_good_obs.temps)


# ---------------------------------------------------------------------------
# Stuck
# ---------------------------------------------------------------------------


def test_stuck_when_net_pwm_moves_with_every_reading_frozen(gcfg):
    """Frozen readings while the *net* commanded PWM moves > stuck_pwm_net -> untrusted.

    Both readings freeze; the coolant lie sits 2 C above the setpoint so the
    solver keeps raising the fans. The window fills with identical samples
    while the PWM travels far more than ``stuck_pwm_net``: after
    ``stuck_ticks`` the coolant is Stuck -> fallback. It must not be treated
    as at-setpoint, and while frozen it must stay untrusted (the flag clears
    only when the value leaves the ``stuck_eps_c`` band).
    """
    cfg = gcfg
    plant, base = settled(cfg)
    frozen_c = SP + 2.0
    frozen_a = base[-1].obs.temps[AIR]
    n = 3 * cfg.stuck_ticks
    recs = continue_run(
        plant, cfg, base, n, hook=lambda i, o: patched(o, coolant=frozen_c, air=frozen_a)
    )
    fired = first_index(recs, lambda r: any(r.cmd.diagnostics["gate"]["stuck"].values()))
    assert fired is not None, "frozen readings with a large net PWM move were never flagged Stuck"
    # the air was already steady at equilibrium before the freeze, so it may be flagged as soon
    # as the net PWM crosses the threshold; the coolant no later than one full window
    assert fired <= cfg.stuck_ticks + 2, f"Stuck fired late: tick {fired}"
    assert not trusted(recs[fired]) and recs[fired].cmd.mode is Mode.FALLBACK
    assert recs[fired].state.fault_reason is FaultReason.SENSOR_GATE
    # the evidence: either the net PWM moved by more than the threshold within the window,
    # or (for the air, still at a perfect equilibrium) the coolant lie itself is a sibling
    # that moved by more than stuck_sibling_dT_c while the air did not answer
    moved = abs(recs[fired].cmd.pwm["radiator"] - base[-1].cmd.pwm["radiator"])
    flagged = {n for n, f in recs[fired].cmd.diagnostics["gate"]["stuck"].items() if f}
    sibling_lie = abs(frozen_c - base[-1].obs.temps[COOLANT]) > cfg.stuck_sibling_dT_c
    assert moved > cfg.stuck_pwm_net or (flagged == {AIR} and sibling_lie), (
        f"Stuck fired at tick {fired} on {flagged} without evidence: net PWM {moved:.3f}"
    )
    # ... and once the PWM has moved far enough, the frozen coolant itself is flagged
    later = first_index(recs, lambda r: stuck_flag(r, COOLANT))
    assert later is not None and later <= 2 * cfg.stuck_ticks
    assert abs(recs[later].cmd.pwm["radiator"] - base[-1].cmd.pwm["radiator"]) > cfg.stuck_pwm_net
    # frozen ever since -> the flag clears only when the value leaves the band, so never back
    # to auto while both readings stay frozen (the fallback hold itself must not erase the
    # evidence: a frozen reading that was Stuck once is not at-setpoint a window later)
    after = recs[fired:]
    still_auto = [fired + i for i, r in enumerate(after) if r.cmd.mode is not Mode.FALLBACK]
    assert not still_auto, f"frozen sensor was trusted again at ticks {still_auto[:5]}"
    # and the safe policy applied: never toward pwm_min (hold the command in force when the
    # fault fired, then ramp high), ends at max(that command, fallback_pwm) per channel --
    # the lie may already have driven a fan above fallback_pwm; a fault never lowers it
    pre = cmd_before(base, recs, fired)
    end = {ch: max(pre[ch], cfg.fallback_pwm[ch]) for ch in cfg.channels}
    assert recs[-1].cmd.pwm == pytest.approx(end, abs=TOL)
    for r in after:
        for ch in cfg.channels:
            assert r.cmd.pwm[ch] >= pre[ch] - TOL


def test_stuck_when_sibling_temperature_moves(gcfg):
    """Coolant frozen at the setpoint while a heat step moves the air > stuck_sibling_dT_c."""
    cfg = gcfg
    plant, base = settled(cfg)
    frozen_c = base[-1].obs.temps[COOLANT]
    air0 = base[-1].obs.temps[AIR]
    n = cfg.stuck_ticks + 20
    recs = continue_run(
        plant,
        cfg,
        base,
        n,
        hook=lambda i, o: patched(o, coolant=frozen_c),
        heat_schedule=[(0, 180.0)],
    )
    air_moved = first_index(recs, lambda r: r.obs.temps[AIR] - air0 > cfg.stuck_sibling_dT_c)
    assert air_moved is not None and air_moved < cfg.stuck_ticks, "air must move in the window"
    fired = first_index(recs, lambda r: stuck_flag(r, COOLANT))
    assert fired is not None, "frozen coolant with a moving sibling was never flagged Stuck"
    assert air_moved <= fired <= cfg.stuck_ticks + 2
    assert recs[fired].cmd.mode is Mode.FALLBACK and not trusted(recs[fired])
    # the frozen value said "at setpoint"; the controller must not believe it
    assert all(r.cmd.mode is Mode.FALLBACK for r in recs[fired:])
    assert recs[-1].cmd.pwm["radiator"] >= min(base[-1].cmd.pwm["radiator"], 0.8) - TOL


def test_stuck_negative_case_equilibrium_with_pwm_dither_stays_trusted(gcfg):
    """Frozen coolant at equilibrium, PWM dithering a few thousandths, net ~ 0 -> trusted."""
    cfg = gcfg
    plant, base = settled(cfg)
    n = 3 * cfg.stuck_ticks

    def hook(i, obs):
        wobble = 0.012 if i % 2 == 0 else -0.012  # inside the stuck_eps_c band
        return patched(obs, coolant=SP + wobble)

    recs = continue_run(plant, cfg, base, n, hook=hook)
    assert not untrusted_ticks(recs), f"dither at equilibrium rejected: {untrusted_ticks(recs)[:5]}"
    assert all(r.cmd.mode is Mode.AUTO for r in recs)
    assert not any(stuck_flag(r, COOLANT) for r in recs)
    pwm = [r.cmd.pwm["radiator"] for r in recs]
    dither = [abs(b - a) for a, b in zip(pwm, pwm[1:], strict=False)]
    assert 0 < max(dither) < 0.01, "expected a dither of a few thousandths"
    for k in range(cfg.stuck_ticks, n):
        assert abs(pwm[k] - pwm[k - cfg.stuck_ticks]) < cfg.stuck_pwm_net


def _gate_window(cfg: MpcConfig, temps_series, pwm_series):
    """Build a window from per-tick temps dicts and per-tick PWM values."""
    window = ()
    for temps, p in zip(temps_series, pwm_series, strict=True):
        window = push_window(window, temps, dict.fromkeys(cfg.channels, p), cfg.stuck_ticks)
    return window


def test_gate_stuck_flags_frozen_temp_with_net_pwm_move(gcfg):
    cfg = gcfg
    n = cfg.stuck_ticks
    temps = [{COOLANT: SP, AIR: 29.0} for _ in range(n)]
    pwm = [0.5 + 0.3 * k / (n - 1) for k in range(n)]  # net 0.3 > stuck_pwm_net
    window = _gate_window(cfg, temps, pwm)
    obs = make_obs(cfg, n * cfg.dt, coolant=SP, air=29.0)
    res = evaluate_gate(obs, cfg, last_good_obs=None, last_raw_temps=temps[-1], window=window)
    assert res.stuck[COOLANT] is True and res.stuck[AIR] is True
    assert res.trusted is False


def test_gate_stuck_flags_frozen_temp_when_sibling_moves(gcfg):
    cfg = gcfg
    n = cfg.stuck_ticks
    step_c = 2.0 * cfg.stuck_sibling_dT_c / (n - 1)  # smooth, physically plausible
    temps = [{COOLANT: SP, AIR: 29.0 + k * step_c} for k in range(n)]
    window = _gate_window(cfg, temps, [0.5] * n)
    obs = make_obs(cfg, n * cfg.dt, coolant=SP, air=temps[-1][AIR])
    res = evaluate_gate(obs, cfg, last_good_obs=None, last_raw_temps=temps[-1], window=window)
    assert res.stuck[COOLANT] is True
    assert res.stuck[AIR] is False
    assert res.per_temp[AIR] is True and res.per_temp[COOLANT] is False


def test_gate_stuck_negative_dither_and_sum_of_moves(gcfg):
    """Net, not a running sum: dithering +-0.003 sums past any threshold, net stays ~ 0."""
    cfg = gcfg
    n = cfg.stuck_ticks
    temps = [{COOLANT: SP + (0.005 if k % 2 else -0.005), AIR: 29.0} for k in range(n)]
    pwm = [0.5 + (0.003 if k % 2 else -0.003) for k in range(n)]
    assert sum(abs(b - a) for a, b in zip(pwm, pwm[1:], strict=False)) > cfg.stuck_pwm_net
    window = _gate_window(cfg, temps, pwm)
    obs = make_obs(cfg, n * cfg.dt, coolant=SP, air=29.0)
    res = evaluate_gate(obs, cfg, last_good_obs=None, last_raw_temps=temps[-1], window=window)
    assert res.stuck == {COOLANT: False, AIR: False}
    assert res.trusted is True


# ---------------------------------------------------------------------------
# Spike
# ---------------------------------------------------------------------------


@pytest.mark.parametrize("value", [125.0, -40.0], ids=["plus125", "minus40"])
def test_spike_single_sample(gcfg, value):
    """median3=false: exactly one untrusted tick, last good unchanged, PWM not chased;
    median3=true: zero untrusted ticks. Rate limit either way (checked_step)."""
    cfg = gcfg
    plant, base = settled(cfg)
    pre_pwm = base[-1].cmd.pwm
    pre_good = base[-1].state.last_good_obs
    n = cfg.confirm_ticks + 8
    recs = continue_run(
        plant, cfg, base, n, hook=lambda i, o: patched(o, coolant=value) if i == 0 else o
    )
    bad = untrusted_ticks(recs)
    if cfg.median3:
        assert bad == [], f"median3 must remove a single spike; untrusted ticks {bad}"
        assert all(r.cmd.mode is Mode.AUTO for r in recs)
    else:
        assert bad == [0], f"a single spike must cost exactly one untrusted tick, got {bad}"
        assert recs[0].cmd.mode is Mode.FALLBACK
        assert recs[0].state.last_good_obs.temps == pre_good.temps, "last good moved on a spike"
        assert recs[0].cmd.pwm == pytest.approx(pre_pwm, abs=TOL)  # hold, not chase
        first_auto = first_index(recs, lambda r: r.cmd.mode is not Mode.FALLBACK)
        assert first_auto is not None and first_auto <= cfg.confirm_ticks, (
            f"streak did not resume after the spike: first auto tick {first_auto}"
        )
        assert all(r.cmd.mode is not Mode.FALLBACK for r in recs[first_auto:])
    # the lie is never chased toward pwm_max
    for r in recs:
        for ch in cfg.channels:
            assert abs(r.cmd.pwm[ch] - pre_pwm[ch]) < 0.03
        assert abs(r.state.last_good_obs.temps[COOLANT] - SP) < 2.0


# ---------------------------------------------------------------------------
# Jump
# ---------------------------------------------------------------------------


def test_jump_confirms_after_confirm_ticks_and_leaves_fallback(gcfg):
    """+30 C that stays: first tick untrusted, next ticks trusted vs previous raw, after
    confirm_ticks the whole last_good_obs is the new level. With median3 one tick later.
    A Jump must never stay in fallback forever."""
    cfg = gcfg
    plant, base = settled(cfg)
    shift = surfaces_at(cfg)
    n = cfg.confirm_ticks + 30
    recs = continue_run(
        plant, cfg, base, n, hook=lambda i, o: patched(o, coolant=o.temps[COOLANT] + 30.0)
    )
    bad = untrusted_ticks(recs)
    assert bad == [shift], f"expected exactly one untrusted tick at {shift}, got {bad}"
    fb = fallback_ticks(recs)
    expected_fb = list(range(shift, shift + cfg.confirm_ticks))
    assert fb == expected_fb, f"fallback ticks {fb} != {expected_fb}"
    ret = recs[shift + cfg.confirm_ticks]
    assert ret.cmd.mode is not Mode.FALLBACK
    assert ret.state.fault_since_ts is None and ret.state.fault_reason is None
    # whole last_good_obs := obs at confirm
    assert ret.state.last_good_obs.temps[COOLANT] == pytest.approx(ret.obs.temps[COOLANT], abs=0.5)
    assert ret.state.last_good_obs.temps[COOLANT] > SP + 25.0
    # partial update never happened while untrusted
    for r in recs[shift : shift + cfg.confirm_ticks]:
        assert r.state.last_good_obs.temps[COOLANT] < SP + 5.0
    # never back to fallback afterwards: the new level is consistent tick to tick
    assert all(r.cmd.mode is not Mode.FALLBACK for r in recs[shift + cfg.confirm_ticks :])
    # PWM during the fallback ticks held (hold_s > confirm_s), no chase
    pre = cmd_before(base, recs, shift)
    for r in recs[shift : shift + cfg.confirm_ticks]:
        assert r.cmd.pwm == pytest.approx(pre, abs=TOL)


def test_jump_on_one_sensor_blocks_partial_last_good_update(gcfg):
    """A Jump on the air sensor: the coolant channel is fine but the tick is untrusted and
    last_good_obs does not move for either channel until the whole tick confirms."""
    cfg = gcfg
    plant, base = settled(cfg)
    shift = surfaces_at(cfg)
    recs = continue_run(
        plant,
        cfg,
        base,
        cfg.confirm_ticks + 2,
        hook=lambda i, o: patched(o, air=o.temps[AIR] + 30.0),
    )
    assert untrusted_ticks(recs) == [shift]
    good = good_before(base, recs, shift)
    for r in recs[shift : shift + cfg.confirm_ticks]:
        assert r.state.last_good_obs.temps == good
    ret = recs[shift + cfg.confirm_ticks]
    assert ret.cmd.mode is not Mode.FALLBACK
    assert ret.state.last_good_obs.temps[AIR] > good[AIR] + 25.0


# ---------------------------------------------------------------------------
# Drift
# ---------------------------------------------------------------------------


def test_drift_does_not_walk_pwm_to_the_rail_without_bound(gcfg):
    """Slow bias +0.05 C/step on the coolant reading.

    A drift is indistinguishable from real heating, so the controller follows
    it -- but never faster than the rate limit, never through an unbounded
    integrator, and with the honest ``saturated`` mode once it reaches the
    rail. Acceptable lag (documented): the PWM stays off the rail while the
    accumulated bias is below 1 C, and the integrator stays inside
    ``[pwm_min, pwm_max]`` for the whole run.
    """
    cfg = gcfg
    plant, base = settled(cfg)
    n = 400
    recs = continue_run(
        plant, cfg, base, n, hook=lambda i, o: patched(o, coolant=o.temps[COOLANT] + 0.05 * (i + 1))
    )
    assert all(r.cmd.mode is not Mode.FALLBACK for r in recs)
    for r in recs:
        for ch, v in r.state.integrator.items():
            assert cfg.pwm_min - TOL <= v <= cfg.pwm_max + TOL, f"integrator wound: {ch}={v}"
    small_bias = [r for i, r in enumerate(recs) if 0.05 * (i + 1) < 1.0]
    for r in small_bias:
        assert max(r.cmd.pwm.values()) < cfg.pwm_max - TOL, "railed on less than 1 C of bias"
    railed = [r for r in recs if max(r.cmd.pwm.values()) >= cfg.pwm_max - TOL]
    assert railed, "20 C of accumulated bias must eventually saturate the fans"
    assert all(r.cmd.mode is Mode.SATURATED for r in recs[-20:]), "railed but not honest"


# ---------------------------------------------------------------------------
# Swap
# ---------------------------------------------------------------------------


def _swap(obs: PlantObservation) -> PlantObservation:
    return patched(obs, coolant=obs.temps[AIR], air=obs.temps[COOLANT])


def test_swap_onset_disagrees_falls_back_and_never_goes_toward_min(gcfg):
    """coolant/air keys exchanged: the disagreement with history -> fallback with the
    hold-then-ramp-high policy, never a step toward pwm_min."""
    cfg = gcfg
    plant, base = settled(cfg)
    shift = surfaces_at(cfg)
    recs = continue_run(plant, cfg, base, 12, hook=lambda i, o: _swap(o))
    assert not trusted(recs[shift]) and recs[shift].cmd.mode is Mode.FALLBACK
    pre = cmd_before(base, recs, shift)
    fb = fallback_ticks(recs)
    assert fb and fb[0] == shift
    run_len = next((k for k in range(len(fb)) if fb[k] != shift + k), len(fb))
    episode = recs[shift : shift + run_len]
    assert_hold_then_high(episode, cfg, pre)
    for r in episode:
        for ch in cfg.channels:
            assert r.cmd.pwm[ch] >= min(pre[ch], cfg.fallback_pwm[ch]) - TOL


def test_swap_persisting_confirms_as_a_jump_after_the_hold(gcfg):
    """A persisting coolant/air swap: onset fallback with the hold policy, then it
    confirms like a Jump; the ramp toward ``fallback_pwm`` is never reached.

    The section 4.4 row reads "disagreement -> fallback; hold then ramp high,
    never min". Section 3 defines the gate, though: after the onset tick every
    swapped sample is within ``dT_max_tick`` of the previous raw value, so the
    exchanged pair is a Jump on both sensors ("Following samples are near
    previous raw -> those channels are trusted"), and the config rule
    ``confirm_s <= fallback_hold_s`` exists precisely so that "a real Jump"
    confirms *before* hold/ramp-high. A cross-sensor plausibility check that
    would keep a steady swap untrusted has no rule in section 3 and would
    trap any simultaneous double Jump in fallback forever, which section 3
    calls a spec bug. This test therefore encodes section 3 (see the report's
    spec_issues): fallback exactly for ``confirm_ticks`` ticks from the
    surfacing tick, PWM held (never toward ``pwm_min``), auto again on the
    confirming tick with a bumpless first command and ``last_good_obs``
    replaced as a whole.
    """
    cfg = gcfg
    plant, base = settled(cfg)
    shift = surfaces_at(cfg)
    n = shift + cfg.confirm_ticks + 12
    recs = continue_run(plant, cfg, base, n, hook=lambda i, o: _swap(o))
    fb = fallback_ticks(recs)
    assert fb == list(range(shift, shift + cfg.confirm_ticks)), f"fallback ticks {fb}"
    pre = cmd_before(base, recs, shift)
    episode = recs[shift : shift + cfg.confirm_ticks]
    assert_hold_then_high(episode, cfg, pre)
    for r in episode:
        for ch in cfg.channels:
            assert r.cmd.pwm[ch] >= min(pre[ch], cfg.fallback_pwm[ch]) - TOL
    # the hold outlives the confirmation (confirm_s <= fallback_hold_s), so no ramp happened
    assert episode[-1].cmd.pwm == pytest.approx(pre, abs=TOL)
    ret = recs[shift + cfg.confirm_ticks]
    assert trusted(ret) and ret.cmd.mode is not Mode.FALLBACK
    assert ret.state.fault_since_ts is None and ret.state.fault_reason is None
    assert ret.cmd.pwm == pytest.approx(episode[-1].cmd.pwm, abs=1e-9), "return not bumpless"
    good = ret.state.last_good_obs
    assert good is not None and good.ts == ret.obs.ts
    for name in cfg.temps:  # the whole last_good_obs moved to the swapped level
        assert abs(good.temps[name] - ret.obs.temps[name]) <= cfg.dT_max_tick
        assert abs(good.temps[name] - base[-1].state.last_good_obs.temps[name]) > cfg.dT_max_tick


# ---------------------------------------------------------------------------
# Impossible combo / impossible dT/dt
# ---------------------------------------------------------------------------


def test_impossible_combo_is_untrusted(gcfg):
    """coolant 18 C, air 90 C, PWM 1.0 for three ticks -> untrusted, held, then recovers."""
    cfg = gcfg
    plant, base = settled(cfg)
    shift = surfaces_at(cfg)

    def hook(i, obs):
        if i < 3:
            obs = patched(obs, coolant=18.0, air=90.0)
            return dataclasses.replace(obs, pwm=dict.fromkeys(cfg.channels, 1.0))
        return obs

    recs = continue_run(plant, cfg, base, 3 + cfg.confirm_ticks + 6, hook=hook)
    bad = untrusted_ticks(recs)
    assert shift in bad, f"impossible combination trusted; untrusted ticks {bad}"
    assert recs[shift].cmd.mode is Mode.FALLBACK
    pre = cmd_before(base, recs, shift)
    for i in bad:
        assert recs[i].cmd.mode is Mode.FALLBACK
        assert recs[i].cmd.pwm == pytest.approx(pre, abs=TOL)  # held, not chased
    assert recs[-1].cmd.mode is not Mode.FALLBACK, "did not recover after the lie ended"


def test_impossible_dT_dt_rejects_the_sample(gcfg):
    """+15 C in one dt -> that sample is rejected (median3 removes a single one)."""
    cfg = gcfg
    plant, base = settled(cfg)
    assert cfg.dT_max_tick < 15.0
    recs = continue_run(
        plant,
        cfg,
        base,
        cfg.confirm_ticks + 4,
        hook=lambda i, o: patched(o, coolant=o.temps[COOLANT] + 15.0) if i == 0 else o,
    )
    bad = untrusted_ticks(recs)
    assert bad == ([] if cfg.median3 else [0])
    if not cfg.median3:
        assert recs[0].cmd.mode is Mode.FALLBACK
        assert "slew" in recs[0].cmd.diagnostics["gate"]["reasons"].get(COOLANT, [])


def test_gate_impossible_dT_dt_against_both_references(gcfg):
    cfg = gcfg
    good = make_obs(cfg, 0.0, coolant=SP, air=29.0)
    window = _gate_window(cfg, [{COOLANT: SP, AIR: 29.0}] * 3, [0.5] * 3)
    res = evaluate_gate(
        make_obs(cfg, cfg.dt, coolant=SP + 15.0, air=29.0),
        cfg,
        last_good_obs=good,
        last_raw_temps={COOLANT: SP, AIR: 29.0},
        window=window,
    )
    if cfg.median3:
        assert res.trusted is True  # the median of (35, 35, 50) is 35
    else:
        assert res.trusted is False and res.per_temp[COOLANT] is False and res.per_temp[AIR]


# ---------------------------------------------------------------------------
# Dropout
# ---------------------------------------------------------------------------


@pytest.mark.parametrize("kind", ["none", "missing"])
@pytest.mark.parametrize("k", [3, 12], ids=["short", "past_hold"])
def test_dropout_then_back_resumes_without_pwm_spike(gcfg, kind, k):
    """None / missing for k steps, then back: no NaN, hold-then-high, bumpless resume."""
    cfg = gcfg
    plant, base = settled(cfg)
    pre = base[-1].cmd.pwm
    lie = None if kind == "none" else DROP
    n = k + cfg.confirm_ticks + 6
    recs = continue_run(
        plant, cfg, base, n, hook=lambda i, o: patched(o, coolant=lie) if i < k else o
    )
    assert untrusted_ticks(recs) == list(range(k))
    assert_hold_then_high(recs[:k], cfg, pre)
    # trusted again, but auto only after confirm_ticks consecutive trusted ticks
    fb = fallback_ticks(recs)
    assert fb == list(range(k + cfg.confirm_ticks - 1)) or fb == list(
        range(k + cfg.confirm_ticks)
    ), f"fallback ticks {fb} for a {k}-tick dropout with confirm_ticks={cfg.confirm_ticks}"
    ret = recs[fb[-1] + 1]
    last_fb = recs[fb[-1]].cmd.pwm
    assert ret.cmd.mode is not Mode.FALLBACK
    assert ret.cmd.pwm == pytest.approx(last_fb, abs=1e-9), "PWM spike on resume (not bumpless)"
    # None/NaN never leaks into the state (checked_step asserted finiteness); raw stores None
    for r in recs[:k]:
        assert r.state.last_raw_temps.get(COOLANT) is None
        assert r.state.window[-1].raw_temps.get(COOLANT) is None
    if k * cfg.dt > cfg.fallback_hold_s:
        assert recs[k - 1].cmd.pwm != pytest.approx(pre, abs=TOL), "past the hold: must ramp"


# ---------------------------------------------------------------------------
# Raw garbage
# ---------------------------------------------------------------------------


@pytest.mark.parametrize(
    "value",
    [32767.0, 65535.0, 35000.0, -32768.0],
    ids=["int16max", "0xFFFF", "millideg", "int16min"],
)
def test_raw_garbage_out_of_range_is_untrusted(gcfg, value):
    cfg = gcfg
    plant, base = settled(cfg)
    recs = continue_run(
        plant,
        cfg,
        base,
        2 + cfg.confirm_ticks + 6,
        hook=lambda i, o: patched(o, coolant=value) if i < 2 else o,
    )
    bad = untrusted_ticks(recs)
    assert bad, "raw garbage was trusted"
    assert bad[0] == surfaces_at(cfg)
    pre = cmd_before(base, recs, bad[0])
    for i in bad:
        assert recs[i].cmd.mode is Mode.FALLBACK
        assert recs[i].cmd.pwm == pytest.approx(pre, abs=TOL)
        assert abs(recs[i].state.last_good_obs.temps[COOLANT] - SP) < 2.0
    assert recs[-1].cmd.mode is not Mode.FALLBACK


# ---------------------------------------------------------------------------
# Flicker
# ---------------------------------------------------------------------------


@pytest.mark.parametrize("bad", ["spike", "missing", "none"])
def test_flicker_keeps_fallback_without_resetting_the_timer_or_chattering(gcfg, bad):
    """Alternate good/bad every step: mode stays fallback (streak < confirm_ticks), the
    hold timer never resets, PWM does not chatter at d_pwm_max each tick."""
    cfg = gcfg
    plant, base = settled(cfg)
    lie = {"spike": 125.0, "missing": DROP, "none": None}[bad]
    n = 40
    all_recs = continue_run(
        plant, cfg, base, n, hook=lambda i, o: patched(o, coolant=lie) if i % 2 == 0 else o
    )
    # a median of three lets an alternating spike through until the pattern fills the
    # filter (tick 2); a missing / None sample cannot be filtered and surfaces at once
    start = untrusted_ticks(all_recs)[0]
    assert start <= 2, f"flicker surfaced late: tick {start}"
    recs = all_recs[start:]
    pre = cmd_before(base, all_recs, start)
    auto = [start + i for i, r in enumerate(recs) if r.cmd.mode is not Mode.FALLBACK]
    assert not auto, f"flicker let the controller back to auto at ticks {auto}"
    assert max(r.state.trusted_streak for r in recs) < cfg.confirm_ticks
    assert_hold_then_high(recs, cfg, pre)  # includes: fault_since_ts constant, hold then ramp
    since = {r.state.fault_since_ts for r in recs}
    assert len(since) == 1
    for ch in cfg.channels:
        pwm = [r.cmd.pwm[ch] for r in recs]
        deltas = [b - a for a, b in zip([pre[ch], *pwm], pwm, strict=False)]
        signs = {math.copysign(1, d) for d in deltas if abs(d) > TOL}
        assert len(signs) <= 1, f"{ch}: PWM chattered back and forth: {pwm}"
        total_variation = sum(abs(d) for d in deltas)
        assert total_variation <= abs(cfg.fallback_pwm[ch] - pre[ch]) + TOL
    assert recs[-1].cmd.pwm == pytest.approx(cfg.fallback_pwm, abs=TOL)  # ramped high, stayed


# ---------------------------------------------------------------------------
# Constant noise
# ---------------------------------------------------------------------------


def test_gaussian_noise_sigma_0p2_rejects_under_one_percent_mode_stays_auto(gcfg):
    cfg = gcfg
    plant = Plant(
        PlantParams(dt=cfg.dt, heat_w=100.0, noise_sigma_c=0.2),
        initial_pwm=0.5,
        t_coolant=40.0,
        t_air=30.0,
        seed=42,
    )
    n = 3000
    recs = run_closed_loop(plant, cfg, checked_step, n)
    rejected = untrusted_ticks(recs)
    assert len(rejected) / n < 0.01, f"gate rejected {len(rejected)} of {n} noisy ticks"
    assert all(r.cmd.mode is not Mode.FALLBACK for r in recs), "noise flickered mode"
    tail = recs[900:]
    for ch in cfg.channels:
        pwm = [r.cmd.pwm[ch] for r in tail]
        deltas = [abs(b - a) for a, b in zip(pwm, pwm[1:], strict=False)]
        assert statistics.fmean(deltas) < 0.02, (
            f"{ch}: PWM chatter, mean |dPWM| {statistics.fmean(deltas):.4f}"
        )
        big = sum(1 for d in deltas if d > 0.5 * cfg.d_pwm_max)
        assert big / len(deltas) < 0.01, f"{ch}: {big} near-rate-limit moves on noise"
    assert abs(statistics.fmean(r.obs.temps[COOLANT] for r in tail) - SP) < 0.5


def test_sigma_2_noise_would_trip_the_slew_gate(cfg):
    """Sanity of the spec's remark: sigma = 2 C trips the slew gate constantly."""
    plant = Plant(PlantParams(dt=cfg.dt, heat_w=100.0, noise_sigma_c=2.0), initial_pwm=0.5, seed=3)
    recs = run_closed_loop(plant, cfg, checked_step, 400)
    assert len(untrusted_ticks(recs)) / len(recs) > 0.05
