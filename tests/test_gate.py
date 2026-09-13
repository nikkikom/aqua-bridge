"""Sensor gate rules in isolation (PROJECT.md section 3, "Trusted tick").

Every test runs with ``median3`` both ``False`` and ``True`` (fixture
``gcfg``). ``run_gate`` replays a sequence of raw samples through
``evaluate_gate`` the way ``mpc.step`` does -- pushing every raw sample
into the window and moving ``last_good_obs`` only on trusted ticks -- so
Spike / Jump stories can be asserted tick by tick without the solver.
"""

from __future__ import annotations

import dataclasses
import json
import math
from collections.abc import Mapping, Sequence

import pytest

from aqua_bridge.control.gate import (
    REASON_MISSING,
    REASON_NON_FINITE,
    REASON_NULL,
    REASON_RANGE,
    REASON_SLEW,
    REASON_STUCK,
    GateResult,
    evaluate_gate,
    filtered_value,
    median3_of,
    push_window,
    sanitize_temps,
)
from aqua_bridge.model import MpcConfig, PlantObservation, WindowSample
from invariants import assert_no_non_finite, make_obs

SP = 35.0  # coolant setpoint in config.example.yaml


@pytest.fixture(params=[False, True], ids=["median3=false", "median3=true"])
def gcfg(fast_cfg: MpcConfig, request: pytest.FixtureRequest) -> MpcConfig:
    return dataclasses.replace(fast_cfg, median3=request.param)


def gate(
    cfg: MpcConfig,
    obs: PlantObservation,
    *,
    last_good: PlantObservation | None = None,
    last_raw: Mapping[str, float | None] | None = None,
    window: Sequence[WindowSample] = (),
    dT_limit: float | None = None,  # noqa: N803
) -> GateResult:
    return evaluate_gate(
        obs, cfg, last_good_obs=last_good, last_raw_temps=last_raw, window=window, dT_limit=dT_limit
    )


def build_window(
    cfg: MpcConfig,
    temps_seq: Sequence[Mapping[str, float | None]],
    pwm_seq: Sequence[float | Mapping[str, float]] | None = None,
) -> tuple[tuple[WindowSample, ...], dict[str, float | None]]:
    """Window (newest last) and ``last_raw_temps`` for a history of raw samples."""
    window: tuple[WindowSample, ...] = ()
    last_raw: dict[str, float | None] = {}
    for i, temps in enumerate(temps_seq):
        pwm = 0.5 if pwm_seq is None else pwm_seq[i]
        pwm_d = dict.fromkeys(cfg.channels, float(pwm)) if not isinstance(pwm, Mapping) else pwm
        raw = sanitize_temps(temps, cfg.temps)
        window = push_window(window, raw, pwm_d, cfg.stuck_ticks)
        last_raw = raw
    return window, last_raw


def run_gate(
    cfg: MpcConfig, samples: Sequence[Mapping[str, float | None]], pwm: float = 0.5
) -> list[GateResult]:
    """Replay ``samples`` through the gate like ``step`` does (rule 5 + last_good on trust)."""
    window: tuple[WindowSample, ...] = ()
    last_raw: dict[str, float | None] | None = None
    last_good: PlantObservation | None = None
    out: list[GateResult] = []
    for i, temps in enumerate(samples):
        obs = make_obs(cfg, ts=i * cfg.dt, temps=temps)
        r = gate(cfg, obs, last_good=last_good, last_raw=last_raw, window=window)
        out.append(r)
        if r.trusted:  # like step: last_good carries the gate-filtered values
            last_good = dataclasses.replace(obs, temps=dict(r.filtered))
        window = push_window(window, r.raw, dict.fromkeys(cfg.channels, pwm), cfg.stuck_ticks)
        last_raw = r.raw
    return out


def coolant(values: Sequence[float | None], air: float = 30.0) -> list[dict[str, float | None]]:
    return [{"coolant": v, "air": air} for v in values]


# ---------------------------------------------------------------------------
# helpers
# ---------------------------------------------------------------------------


def test_median3_of_takes_middle_of_last_three():
    assert median3_of([1.0, 100.0, 2.0]) == 2.0
    assert median3_of([0.0, 1.0, 100.0, 2.0]) == 2.0  # only the last three matter
    assert median3_of([5.0]) == 5.0
    assert median3_of([5.0, 6.0]) == 6.0  # fewer than three -> current
    assert median3_of([5.0, None, 6.0]) == 6.0  # dropout disables the filter
    assert median3_of([5.0, 6.0, None]) is None
    assert median3_of([]) is None


def test_filtered_value_respects_median3_flag():
    assert filtered_value([35.0, 35.0], 125.0, median3=False) == 125.0
    assert filtered_value([35.0, 35.0], 125.0, median3=True) == 35.0


def test_sanitize_temps_restricts_and_replaces_non_finite():
    out = sanitize_temps({"coolant": math.nan, "air": math.inf, "extra": 1.0}, ("coolant", "air"))
    assert out == {"coolant": None, "air": None}
    assert sanitize_temps({}, ("coolant",)) == {"coolant": None}
    assert sanitize_temps(None, ("coolant",)) == {"coolant": None}


def test_push_window_trims_to_stuck_ticks_and_stores_none(fast_cfg):
    n = fast_cfg.stuck_ticks
    window: tuple[WindowSample, ...] = ()
    for i in range(n + 3):
        raw = sanitize_temps({"coolant": float(i), "air": math.nan}, fast_cfg.temps)
        window = push_window(window, raw, {"radiator": 0.5, "intake": 0.5}, n)
    assert len(window) == n
    assert [w.raw_temps["coolant"] for w in window] == [float(i) for i in range(3, n + 3)]
    assert all(w.raw_temps["air"] is None for w in window)


# ---------------------------------------------------------------------------
# structure: missing / extra / None / NaN never raise (rule 2 + section 4.3)
# ---------------------------------------------------------------------------


def test_cold_state_in_range_sample_is_trusted(gcfg):
    r = gate(gcfg, make_obs(gcfg, 0.0))
    assert r.trusted
    assert all(r.per_temp.values())
    assert all(v == () for v in r.reasons.values())
    assert r.filtered == {"coolant": SP, "air": 30.0}


def test_missing_key_untrusted_with_reason(gcfg):
    r = gate(gcfg, make_obs(gcfg, 0.0, temps={"air": 30.0}))
    assert not r.trusted
    assert r.per_temp == {"coolant": False, "air": True}
    assert r.reasons["coolant"] == (REASON_MISSING,)
    assert r.raw["coolant"] is None


def test_extra_unknown_key_untrusted_even_when_values_fine(gcfg):
    r = gate(gcfg, make_obs(gcfg, 0.0, temps={"coolant": SP, "air": 30.0, "gpu": 40.0}))
    assert not r.trusted
    assert r.unknown_keys == ("gpu",)
    assert all(r.per_temp.values())  # every configured temperature is fine


def test_empty_temps_untrusted(gcfg):
    r = gate(gcfg, make_obs(gcfg, 0.0, temps={}))
    assert not r.trusted
    assert r.per_temp == {"coolant": False, "air": False}
    assert all(v == (REASON_MISSING,) for v in r.reasons.values())


def test_none_value_untrusted(gcfg):
    r = gate(gcfg, make_obs(gcfg, 0.0, coolant=None))
    assert not r.trusted
    assert r.reasons["coolant"] == (REASON_NULL,)
    assert r.filtered["coolant"] is None


@pytest.mark.parametrize("bad", [math.nan, math.inf, -math.inf])
def test_non_finite_value_untrusted_and_stored_as_none(gcfg, bad):
    r = gate(gcfg, make_obs(gcfg, 0.0, coolant=bad))
    assert not r.trusted
    assert r.reasons["coolant"] == (REASON_NON_FINITE,)
    assert r.raw["coolant"] is None
    assert_no_non_finite(r.to_dict(), "gate result")
    json.dumps(r.to_dict())


# ---------------------------------------------------------------------------
# rule 2: absolute range and slew vs last_good OR last_raw
# ---------------------------------------------------------------------------


@pytest.mark.parametrize("value", [-25.0, 125.0, -20.01, 120.01, 32767.0, 65535.0, 35000.0])
def test_out_of_range_invalid_not_a_real_plant(gcfg, value):
    r = gate(gcfg, make_obs(gcfg, 0.0, coolant=value))
    assert not r.trusted
    assert REASON_RANGE in r.reasons["coolant"]


@pytest.mark.parametrize("value", [-20.0, 120.0])
def test_range_boundaries_are_valid(gcfg, value):
    r = gate(gcfg, make_obs(gcfg, 0.0, coolant=value))
    assert r.per_temp["coolant"]


def test_slew_vs_either_reference(fast_cfg):
    cfg = fast_cfg  # median3=false: references are literal last_good / last_raw
    limit = cfg.dT_max_tick
    good = make_obs(cfg, 0.0, coolant=SP)
    # Near last_raw, far from last_good -> trusted (Jump confirming).
    r = gate(
        cfg,
        make_obs(cfg, 2.0, coolant=SP + 30.0),
        last_good=good,
        last_raw={"coolant": SP + 30.0 - 0.5 * limit, "air": 30.0},
    )
    assert r.per_temp["coolant"]
    # Near last_good, far from last_raw -> trusted (recovering after a Spike).
    r = gate(
        cfg,
        make_obs(cfg, 2.0, coolant=SP + 0.5 * limit),
        last_good=good,
        last_raw={"coolant": 110.0, "air": 30.0},
    )
    assert r.per_temp["coolant"]
    # Far from both -> slew.
    r = gate(
        cfg,
        make_obs(cfg, 2.0, coolant=SP + 3 * limit),
        last_good=good,
        last_raw={"coolant": SP, "air": 30.0},
    )
    assert r.reasons["coolant"] == (REASON_SLEW,)
    # Exactly at the limit is allowed.
    r = gate(
        cfg,
        make_obs(cfg, 2.0, coolant=SP + limit),
        last_good=good,
        last_raw={"coolant": SP, "air": 30.0},
    )
    assert r.per_temp["coolant"]


def test_impossible_dT_dt_rejected(gcfg):
    samples = coolant([SP] * 4 + [SP + 15.0])
    res = run_gate(gcfg, samples)
    if gcfg.median3:
        assert res[-1].trusted  # the median hides a single new sample; it surfaces next tick
        nxt = run_gate(gcfg, samples + coolant([SP + 15.0]))
        assert not nxt[-1].trusted
        assert REASON_SLEW in nxt[-1].reasons["coolant"]
    else:
        assert not res[-1].trusted
        assert res[-1].reasons["coolant"] == (REASON_SLEW,)


def test_dT_limit_override_scales_slew(fast_cfg):
    last_raw = {"coolant": SP, "air": 30.0}
    hot = make_obs(fast_cfg, 1.0, coolant=SP + 2.5 * fast_cfg.dT_max_tick)
    assert not gate(fast_cfg, hot, last_raw=last_raw).per_temp["coolant"]
    assert gate(fast_cfg, hot, last_raw=last_raw, dT_limit=3 * fast_cfg.dT_max_tick).per_temp[
        "coolant"
    ]


def test_missing_references_do_not_block_slew_check(gcfg):
    # last_raw has None for coolant and last_good lacks the key -> no reference -> pass.
    good = make_obs(gcfg, 0.0, temps={"air": 30.0})
    r = gate(
        gcfg,
        make_obs(gcfg, 1.0, coolant=60.0),
        last_good=good,
        last_raw={"coolant": None, "air": 30.0},
    )
    assert r.per_temp["coolant"]


# ---------------------------------------------------------------------------
# Spike / Jump stories (section 3 bullets under the gate rules)
# ---------------------------------------------------------------------------


def test_spike_untrusted_ticks_count(gcfg):
    samples = coolant([SP] * 5 + [125.0] + [SP] * 5)
    res = run_gate(gcfg, samples)
    untrusted = [i for i, r in enumerate(res) if not r.trusted]
    if gcfg.median3:
        assert untrusted == []
    else:
        assert untrusted == [5]
        assert REASON_RANGE in res[5].reasons["coolant"]
    # Trusted ticks around the spike are trusted because of last_good, not the spike.
    assert res[6].trusted


def test_negative_spike_single_untrusted_tick(gcfg):
    res = run_gate(gcfg, coolant([SP] * 4 + [-40.0] + [SP] * 3))
    untrusted = [i for i, r in enumerate(res) if not r.trusted]
    assert untrusted == ([] if gcfg.median3 else [4])


def test_jump_surfaces_then_confirms_against_previous_raw(gcfg):
    samples = coolant([SP] * 5 + [SP + 30.0] * 6)
    res = run_gate(gcfg, samples)
    flags = [r.trusted for r in res]
    if gcfg.median3:
        # median hides the first new sample; the jump surfaces one tick later
        assert flags == [True] * 6 + [False] + [True] * 4
        assert res[6].reasons["coolant"] == (REASON_SLEW,)
    else:
        assert flags == [True] * 5 + [False] + [True] * 5
        assert res[5].reasons["coolant"] == (REASON_SLEW,)


def test_jump_on_one_sensor_only_flags_that_channel(gcfg):
    samples = [{"coolant": SP, "air": 30.0}] * 5 + [{"coolant": SP, "air": 60.0}] * 3
    res = run_gate(gcfg, samples)
    bad = next(r for r in res if not r.trusted)
    assert bad.per_temp == {"coolant": True, "air": False}
    assert res[-1].trusted


def test_dropout_then_back_no_nan(gcfg):
    samples = coolant([SP] * 3 + [None] * 3 + [SP] * 3)
    res = run_gate(gcfg, samples)
    assert [r.trusted for r in res] == [True] * 3 + [False] * 3 + [True] * 3
    for r in res:
        assert_no_non_finite(r.to_dict(), "gate")
        assert r.raw["coolant"] is None or math.isfinite(r.raw["coolant"])


def test_swapped_keys_disagree_with_history(gcfg):
    # coolant/air exchanged: both channels move ~30 degrees at once -> untrusted tick.
    samples = [{"coolant": 60.0, "air": 30.0}] * 4 + [{"coolant": 30.0, "air": 60.0}]
    r = run_gate(gcfg, samples)[-1]
    if gcfg.median3:
        assert r.trusted  # hidden for one tick by the median
        r = run_gate(gcfg, samples + [{"coolant": 30.0, "air": 60.0}])[-1]
    assert not r.trusted
    assert r.per_temp == {"coolant": False, "air": False}


# ---------------------------------------------------------------------------
# rule 3: Stuck (net PWM or sibling move)
# ---------------------------------------------------------------------------

AIR_DRIFT = 0.1  # per tick: air is alive (> stuck_eps_c) but moves < stuck_sibling_dT_c net


def air_at(i: int) -> float:
    return 30.0 + AIR_DRIFT * i


def frozen_history(cfg: MpcConfig, pwm_seq, air_seq=None):
    """Window with coolant frozen at SP and air drifting gently (so only coolant is frozen)."""
    n = len(pwm_seq)
    air_seq = [air_at(i) for i in range(n)] if air_seq is None else air_seq
    temps = [{"coolant": SP, "air": a} for a in air_seq]
    return build_window(cfg, temps, pwm_seq)


def pwm_ramp(n: int, start: float = 0.3, net: float = 0.4) -> list[float]:
    return [start + i * (net / (n - 1)) for i in range(n)]


def test_stuck_flags_when_net_pwm_moved(gcfg):
    n = gcfg.stuck_ticks
    window, last_raw = frozen_history(gcfg, pwm_ramp(n))  # net 0.4 > stuck_pwm_net
    obs = make_obs(gcfg, 10.0, coolant=SP, air=air_at(n))
    r = gate(gcfg, obs, last_raw=last_raw, window=window)
    assert not r.trusted
    assert r.stuck["coolant"]
    assert r.reasons["coolant"] == (REASON_STUCK,)
    assert r.per_temp["air"]  # air is alive, so the PWM move does not implicate it


def test_stuck_flags_every_frozen_temperature_when_pwm_moved(gcfg):
    n = gcfg.stuck_ticks
    window, last_raw = frozen_history(gcfg, pwm_ramp(n), air_seq=[30.0] * n)
    r = gate(gcfg, make_obs(gcfg, 10.0, coolant=SP, air=30.0), last_raw=last_raw, window=window)
    assert r.stuck == {"coolant": True, "air": True}
    assert not r.trusted


def test_stuck_negative_case_dither_at_equilibrium(gcfg):
    n = gcfg.stuck_ticks
    dither = [0.5 + 0.003 * (-1) ** i for i in range(n)]  # sum |dpwm| large, net ~0
    window, last_raw = frozen_history(gcfg, dither)
    r = gate(
        gcfg, make_obs(gcfg, 10.0, coolant=SP, air=air_at(n)), last_raw=last_raw, window=window
    )
    assert r.trusted
    assert not r.stuck["coolant"]


def test_stuck_negative_case_both_frozen_nothing_moved(gcfg):
    """Perfectly still plant, constant PWM: frozen readings are simply equilibrium."""
    n = gcfg.stuck_ticks
    window, last_raw = frozen_history(gcfg, [0.5] * n, air_seq=[30.0] * n)
    r = gate(gcfg, make_obs(gcfg, 10.0, coolant=SP, air=30.0), last_raw=last_raw, window=window)
    assert r.trusted
    assert r.stuck == {"coolant": False, "air": False}


def test_stuck_uses_net_not_sum_of_moves(gcfg):
    n = gcfg.stuck_ticks
    # Big excursion that comes back: sum of |dpwm| >> threshold, net exactly 0.
    up_down = [0.3, 0.9] * (n // 2) + ([0.3] if n % 2 else [])
    up_down[-1] = up_down[0]
    window, last_raw = frozen_history(gcfg, up_down)
    r = gate(
        gcfg, make_obs(gcfg, 10.0, coolant=SP, air=air_at(n)), last_raw=last_raw, window=window
    )
    assert not r.stuck["coolant"]


def test_stuck_flags_when_sibling_moved(gcfg):
    n = gcfg.stuck_ticks
    air = [30.0 + i * (1.5 / (n - 1)) for i in range(n)]  # net > stuck_sibling_dT_c
    window, last_raw = frozen_history(gcfg, [0.5] * n, air_seq=air)
    obs = make_obs(gcfg, 10.0, coolant=SP, air=air[-1] + 0.5)
    r = gate(gcfg, obs, last_raw=last_raw, window=window)
    assert r.stuck["coolant"]
    assert not r.stuck["air"]  # air moved, so it is not the frozen one
    assert not r.trusted


def test_sibling_spike_or_jump_is_not_plant_motion(gcfg):
    """A Spike / Jump on the sibling must not brand a calm sensor as Stuck."""
    n = gcfg.stuck_ticks
    coolant_seq = [125.0] + [SP] * (n - 1)  # spike in the oldest window slot
    temps = [{"coolant": c, "air": 30.0} for c in coolant_seq]
    window, last_raw = build_window(gcfg, temps, [0.5] * n)
    r = gate(gcfg, make_obs(gcfg, 10.0, coolant=SP, air=30.0), last_raw=last_raw, window=window)
    assert not r.stuck["air"]
    coolant_seq = [SP] * 2 + [SP + 30.0] * (n - 2)  # jump inside the window
    temps = [{"coolant": c, "air": 30.0} for c in coolant_seq]
    window, last_raw = build_window(gcfg, temps, [0.5] * n)
    obs = make_obs(gcfg, 10.0, coolant=SP + 30.0, air=30.0)
    r = gate(gcfg, obs, last_raw=last_raw, window=window)
    assert not r.stuck["air"]


def test_stuck_within_eps_band_still_counts_as_frozen(gcfg):
    n = gcfg.stuck_ticks
    eps = gcfg.stuck_eps_c
    temps = [{"coolant": SP + (eps * 0.4) * (-1) ** i, "air": air_at(i)} for i in range(n)]
    window, last_raw = build_window(gcfg, temps, pwm_ramp(n))
    obs = make_obs(gcfg, 10.0, coolant=SP + 0.3 * eps, air=air_at(n))
    r = gate(gcfg, obs, last_raw=last_raw, window=window)
    assert r.stuck["coolant"]


def test_stuck_clears_when_value_leaves_band(gcfg):
    n = gcfg.stuck_ticks
    window, last_raw = frozen_history(gcfg, pwm_ramp(n))
    if gcfg.median3:
        # the median shows the new level one sample later: feed it once more
        raw = sanitize_temps({"coolant": SP + 0.5, "air": air_at(n)}, gcfg.temps)
        window = push_window(window, raw, dict.fromkeys(gcfg.channels, 0.7), gcfg.stuck_ticks)
        last_raw = raw
    obs = make_obs(gcfg, 10.0, coolant=SP + 0.5, air=air_at(n + 1))
    r = gate(gcfg, obs, last_raw=last_raw, window=window)
    assert not r.stuck["coolant"]
    assert r.trusted


def test_stuck_needs_full_window(gcfg):
    n = gcfg.stuck_ticks
    window, last_raw = frozen_history(gcfg, pwm_ramp(n - 1))
    assert len(window) == n - 1
    obs = make_obs(gcfg, 10.0, coolant=SP, air=air_at(n - 1))
    r = gate(gcfg, obs, last_raw=last_raw, window=window)
    assert not r.stuck["coolant"]
    assert r.trusted


def test_stuck_broken_by_dropout_in_window(gcfg):
    n = gcfg.stuck_ticks
    temps = [{"coolant": SP, "air": air_at(i)} for i in range(n)]
    temps[1] = {"coolant": None, "air": air_at(1)}
    window, last_raw = build_window(gcfg, temps, pwm_ramp(n))
    obs = make_obs(gcfg, 10.0, coolant=SP, air=air_at(n))
    r = gate(gcfg, obs, last_raw=last_raw, window=window)
    assert not r.stuck["coolant"]


def test_stuck_via_run_gate_after_long_frozen_run_with_pwm_ramp(gcfg):
    """End-to-end through the replay loop: constant coolant while the command ramps 0.2 -> 0.8."""
    n = gcfg.stuck_ticks
    window: tuple[WindowSample, ...] = ()
    last_raw = None
    last_good = None
    flags = []
    pwm = 0.2
    for i in range(3 * n):
        obs = make_obs(gcfg, ts=float(i), coolant=SP, air=air_at(i))
        r = gate(gcfg, obs, last_good=last_good, last_raw=last_raw, window=window)
        flags.append(r.trusted)
        if r.trusted:
            last_good = dataclasses.replace(obs, temps=dict(r.filtered))
        pwm = min(0.8, pwm + 0.1)  # ramps 0.2 -> 0.8 over 6 ticks
        window = push_window(window, r.raw, dict.fromkeys(gcfg.channels, pwm), gcfg.stuck_ticks)
        last_raw = r.raw
    assert not all(flags), "a frozen coolant reading while PWM ramped must be flagged"
    assert flags[-1], "once the PWM settles (net ~0) the flag clears again"


# ---------------------------------------------------------------------------
# rule 4 + result serialisation
# ---------------------------------------------------------------------------


def test_whole_tick_requires_every_temperature(gcfg):
    r = gate(gcfg, make_obs(gcfg, 0.0, air=None))
    assert r.per_temp["coolant"] and not r.per_temp["air"]
    assert not r.trusted


def test_gate_result_to_dict_is_json_and_drops_empty_reasons(gcfg):
    r = gate(gcfg, make_obs(gcfg, 0.0, air=None))
    d = r.to_dict()
    text = json.dumps(d, allow_nan=False)
    assert "coolant" not in d["reasons"]
    assert d["reasons"]["air"] == [REASON_NULL]
    assert json.loads(text)["trusted"] is False


def test_gate_is_pure(gcfg):
    obs = make_obs(gcfg, 0.0)
    window, last_raw = frozen_history(gcfg, [0.5] * gcfg.stuck_ticks)
    before = (tuple(w.to_dict() for w in window), dict(last_raw), obs.to_dict())
    r1 = gate(gcfg, obs, last_raw=last_raw, window=window)
    r2 = gate(gcfg, obs, last_raw=last_raw, window=window)
    assert r1 == r2
    assert before == (tuple(w.to_dict() for w in window), dict(last_raw), obs.to_dict())
