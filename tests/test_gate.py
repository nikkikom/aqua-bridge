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
from collections.abc import Callable, Mapping, Sequence

import pytest

from aqua_bridge.control.gate import (
    REASON_MISSING,
    REASON_NON_FINITE,
    REASON_NULL,
    REASON_RANGE,
    REASON_SLEW,
    REASON_STUCK,
    GateResult,
    advance_slow_windows,
    evaluate_gate,
    filtered_value,
    median3_of,
    push_window,
    sanitize_temps,
    stuck_pwm_lag,
)
from aqua_bridge.control.mpc import step
from aqua_bridge.control.solver_pi import SolverResult
from aqua_bridge.model import Mode, MpcConfig, MpcState, PlantObservation, WindowSample
from das_fixtures import das_cfg, das_mapping, das_obs, default_temps
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


def test_stuck_pwm_lag_matches_the_previous_hardcoded_quarter_window():
    """Item 60: the default fraction (0.25) reproduces ``stuck_ticks // 4`` bit for bit,
    for every window length the config allows (``stuck_ticks >= 2``)."""
    for n in range(2, 50):
        assert stuck_pwm_lag(n) == max(0, min(n // 4, n - 2))


def test_stuck_pwm_lag_fraction_is_configurable():
    assert stuck_pwm_lag(20, 0.5) == 10
    assert stuck_pwm_lag(20, 0.0) == 0
    assert stuck_pwm_lag(3, 1.0) == 1  # capped at stuck_ticks - 2, never the newest sample


def test_stuck_pwm_lag_fraction_controls_how_old_a_pwm_move_must_be(gcfg):
    """A wider ``stuck_pwm_lag_fraction`` requires an even older commanded PWM move before
    it counts as Stuck evidence: the same net move can flag a frozen reading under the
    default fraction and stay unnoticed (not yet evidence) under a wider one."""
    cfg = dataclasses.replace(gcfg, stuck_s=8.0)
    assert cfg.stuck_ticks == 8
    pwm_seq = [0.3, 0.3, 0.3, 0.7, 0.7, 0.7, 0.7, 0.7]  # net move 3 ticks into the window
    window, last_raw = frozen_history(cfg, pwm_seq)
    obs = make_obs(cfg, 10.0, coolant=SP, air=air_at(len(pwm_seq)))

    # default fraction 0.25 -> lag 2: the move is already 5 ticks old at the lagged
    # sample, well past the 2-tick cutoff -> evidence -> flagged
    assert gate(cfg, obs, last_raw=last_raw, window=window).stuck["coolant"]

    # fraction 0.75 -> lag 6: the lagged sample sits before the move (only 3 ticks old,
    # short of the 6-tick cutoff) -> not evidence -> stays trusted
    wide = dataclasses.replace(cfg, stuck_pwm_lag_fraction=0.75)
    assert not gate(wide, obs, last_raw=last_raw, window=window).stuck["coolant"]


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


# ---------------------------------------------------------------------------
# zoned DAS: per-role Stuck sizing, zone/role-restricted evidence, decimation
# (plan section 0.2)
# ---------------------------------------------------------------------------


class DasReplay:
    """Replays samples through the gate the way ``mpc.step`` does with zones:
    decimated windows advanced from the dense window before the gate, the
    latch fed back, the dense window trimmed to ``cfg.window_ticks``."""

    def __init__(self, cfg: MpcConfig) -> None:
        self.cfg = cfg
        self.window: tuple[WindowSample, ...] = ()
        self.last_raw: dict[str, float | None] | None = None
        self.last_good: PlantObservation | None = None
        self.latch: dict[str, float] = {}
        self.slow: dict[str, list] = {}
        self.seq = 0
        self.ts = 0.0

    def tick(self, temps: Mapping[str, float | None], pwm: Mapping[str, float]) -> GateResult:
        cfg = self.cfg
        self.slow = advance_slow_windows(self.slow, self.window, self.seq, cfg)
        obs = PlantObservation(
            temps=dict(temps), rpm=dict.fromkeys(cfg.channels, 900.0), pwm=dict(pwm), ts=self.ts
        )
        r = evaluate_gate(
            obs,
            cfg,
            last_good_obs=self.last_good,
            last_raw_temps=self.last_raw,
            window=self.window,
            stuck_latch=self.latch,
            slow_windows=self.slow,
        )
        if r.trusted:
            self.last_good = dataclasses.replace(obs, temps=dict(r.filtered))
        self.window = push_window(self.window, r.raw, dict(pwm), cfg.window_ticks)
        self.last_raw = r.raw
        self.latch = dict(r.stuck_latch)
        self.seq += 1
        self.ts += cfg.dt
        return r


def _das(**sensor_overrides: Mapping[str, object]) -> MpcConfig:
    m = das_mapping()
    for name, patch in sensor_overrides.items():
        m["sensors"].setdefault(name, {}).update(patch)
    return MpcConfig.from_mapping(m)


def _q(value: float, lsb: float) -> float:
    return round(value / lsb) * lsb


def _base_temps() -> dict[str, float | None]:
    return default_temps(das_cfg())


def test_ds18b20_plateaus_next_to_a_drive_never_flag_stuck():
    """A proximal DS18B20 (1/16 degC) follows a slow drive: minutes on one code at every
    turning point while its zone's fans move hardest. The per-role window (1800 s) and
    band (1.5 LSB) never call that Stuck; a 300 s window would."""
    m = das_mapping()
    m.update(dt=5.0, confirm_s=10.0, fallback_hold_s=20.0, stuck_s=10.0)
    cfg = MpcConfig.from_mapping(m)
    m["sensors"]["prox_a2"]["stuck_s"] = 300.0
    short = MpcConfig.from_mapping(m)
    assert cfg.stuck_params("prox_a2").decimate == 6
    period = 1800.0

    def sample(i: int) -> tuple[dict[str, float | None], dict[str, float]]:
        w = 2.0 * math.pi * i * cfg.dt / period
        dither = 0.03 * (-1) ** i  # thermistor noise on the zone-air sensors
        temps: dict[str, float | None] = {
            "inlet": 25.0,
            "air_a": _q(35.0 + 0.3 * math.sin(w) + dither, 0.01),
            "air_a2": _q(35.1 + 0.3 * math.sin(w) + dither, 0.01),
            "air_b": 35.0,
            "air_c": 35.0,
            "prox_a1": _q(41.0 + 0.5 * math.sin(w + 1.0), 0.0625),
            "prox_a1b": _q(41.2 + 0.5 * math.sin(w + 1.1), 0.0625),
            "prox_a2": _q(40.0 + 0.5 * math.sin(w), 0.0625),
            "prox_b1": 40.0,
            "prox_c1": 30.0,
            "exhaust": 38.0,
        }
        fan = 0.55 + 0.4 * math.cos(w)  # moves fastest where the sensor plateaus
        return temps, {"fa1": fan, "fa2": fan, "fb1": 0.5, "fc1": 0.5}

    longest_plateau = run = 0
    last = None
    replay, replay_short = DasReplay(cfg), DasReplay(short)
    short_flags = 0
    for i in range(1100):
        temps, pwm = sample(i)
        r = replay.tick(temps, pwm)
        assert not any(r.stuck.values()), f"tick {i}: stuck {r.stuck}"
        assert r.trusted, f"tick {i}: {r.reasons}"
        short_flags += replay_short.tick(temps, pwm).stuck["prox_a2"]
        run = run + 1 if temps["prox_a2"] == last else 1
        last = temps["prox_a2"]
        longest_plateau = max(longest_plateau, run)
    assert longest_plateau * cfg.dt >= 150.0  # the scenario really has multi-minute plateaus
    assert short_flags > 0  # and a 300 s window would have branded the sensor Stuck
    # Decimated storage stays bounded: 60 samples per factor, dense window 36 ticks.
    assert {k: len(v) for k, v in replay.slow.items()} == {"2": 60, "6": 60}
    assert len(replay.window) == cfg.window_ticks == 36


def test_a_truly_idle_drive_frozen_longer_than_the_window_never_flags_stuck():
    """Item 27: the sine above turns every few minutes, so no single plateau in it comes
    close to ``stuck_s`` (1800 s) -- the scenario PROJECT.md actually describes ("An idle
    bay's DS18B20 sat inside its band for half an hour") is a reading held flat for at
    least a whole window, not just near a turning point. Hold ``prox_a2`` at one exact
    value for the entire run while its siblings and its zone's fans keep moving on the
    same sine as above: the frozen reading must still never flag, on a real plateau
    several times longer than ``stuck_s`` itself."""
    m = das_mapping()
    m.update(dt=5.0, confirm_s=10.0, fallback_hold_s=20.0, stuck_s=10.0)
    cfg = MpcConfig.from_mapping(m)
    temp_period = 1800.0
    fan_period = 240.0  # much faster than the window: every window sees several full swings
    idle_prox_a2 = _q(40.0, 0.0625)

    def sample(i: int) -> tuple[dict[str, float | None], dict[str, float]]:
        w = 2.0 * math.pi * i * cfg.dt / temp_period
        wf = 2.0 * math.pi * i * cfg.dt / fan_period
        dither = 0.03 * (-1) ** i  # thermistor noise on the zone-air sensors
        temps: dict[str, float | None] = {
            "inlet": 25.0,
            "air_a": _q(35.0 + 0.3 * math.sin(w) + dither, 0.01),
            "air_a2": _q(35.1 + 0.3 * math.sin(w) + dither, 0.01),
            "air_b": 35.0,
            "air_c": 35.0,
            "prox_a1": _q(41.0 + 0.5 * math.sin(w + 1.0), 0.0625),
            "prox_a1b": _q(41.2 + 0.5 * math.sin(w + 1.1), 0.0625),
            "prox_a2": idle_prox_a2,  # genuinely idle for the whole run, not a turning point
            "prox_b1": 40.0,
            "prox_c1": 30.0,
            "exhaust": 38.0,
        }
        fan = 0.55 + 0.4 * math.cos(wf)  # moves the zone's air throughout, unlike prox_a2
        return temps, {"fa1": fan, "fa2": fan, "fb1": 0.5, "fc1": 0.5}

    replay = DasReplay(cfg)
    ticks = 1100
    for i in range(ticks):
        temps, pwm = sample(i)
        r = replay.tick(temps, pwm)
        assert not r.stuck["prox_a2"], f"tick {i}: {r.reasons['prox_a2']}"
        assert r.trusted, f"tick {i}: {r.reasons}"
    # a real plateau, several times the length of the window it is checked against
    assert ticks * cfg.dt >= 2 * cfg.stuck_params("prox_a2").ticks * cfg.dt


def _dense_das(**extra: Mapping[str, object]) -> MpcConfig:
    """Small non-decimated windows (8 ticks at dt=1) on the sensors under test."""
    overrides: dict[str, dict[str, object]] = {
        name: {"stuck_s": 8.0, "stuck_decimate": 1}
        for name in ("air_c", "prox_a2", "prox_a1", "inlet")
    }
    for name, patch in extra.items():
        overrides.setdefault(name, {}).update(patch)
    return _das(**overrides)


def _history(
    cfg: MpcConfig,
    n: int,
    temps_at: Callable[[int], dict[str, float | None]],
    pwm_at: Callable[[int], dict[str, float]],
) -> tuple[DasReplay, GateResult]:
    replay = DasReplay(cfg)
    r = None
    for i in range(n):
        r = replay.tick(temps_at(i), pwm_at(i))
    assert r is not None
    return replay, r


def test_pwm_evidence_counts_only_the_sensors_zone_channels():
    cfg = _dense_das()
    n = cfg.stuck_params("air_c").ticks + 1

    def others_move(i: int) -> dict[str, float]:
        v = 0.2 + 0.07 * i
        return {"fa1": v, "fa2": v, "fb1": v, "fc1": 0.5}

    _, r = _history(cfg, n, lambda i: _base_temps(), others_move)
    assert not r.stuck["air_c"]  # fans of za / zb moved, air_c's own fan did not

    def own_moves(i: int) -> dict[str, float]:
        return {"fa1": 0.5, "fa2": 0.5, "fb1": 0.5, "fc1": 0.2 + 0.07 * i}

    _, r = _history(cfg, n, lambda i: _base_temps(), own_moves)
    assert r.stuck["air_c"] and r.reasons["air_c"] == (REASON_STUCK,)
    assert not r.stuck["prox_a2"]  # za's fans did not move


def test_sibling_evidence_counts_only_same_zone_role_and_bay():
    """A proximal reading answers its own drive: another bay's reading moves with that bay's
    drive heat, which never reaches this sensor, so only a sensor on the same bay counts."""
    cfg = _dense_das(prox_a1b={"stuck_s": 8.0, "stuck_decimate": 1})
    n = cfg.stuck_params("prox_a2").ticks + 1

    def still(i: int) -> dict[str, float]:
        return dict.fromkeys(cfg.channels, 0.5)

    def moving(*names: str) -> Callable[[int], dict[str, float | None]]:
        def temps(i: int) -> dict[str, float | None]:
            t = _base_temps()
            for name in names:
                # net 1.2 degC: above stuck_sibling_dT_c, below stuck_zone_air_dT_c (so a
                # zone-air sensor moving this far is not evidence either), no jump
                t[name] = float(t[name]) + 0.15 * i
            return t

        return temps

    for other in ("air_a", "prox_b1"):  # other role / other zone: no evidence
        _, r = _history(cfg, n, moving(other), still)
        assert not r.stuck["prox_a2"], other
    # same zone and role, other bay: a neighbour's activity burst is no evidence
    _, r = _history(cfg, n, moving("prox_a1", "prox_a1b"), still)
    assert not r.stuck["prox_a2"]
    # same bay: the other sensor on the same drive moved, this one did not
    _, r = _history(cfg, n, moving("prox_a1b"), still)
    assert r.stuck["prox_a1"] and r.reasons["prox_a1"] == (REASON_STUCK,)
    assert not r.stuck["prox_a1b"] and not r.stuck["prox_a2"]


def test_zone_airflow_evidence_weighs_channels_and_ignores_moves_that_cancel():
    """The PWM evidence of a zoned sensor is its zone's relative airflow: fa1 (two fans) up
    while fa2 (one fan) goes down twice as far leaves za's airflow where it was, although
    each channel moved by more than stuck_pwm_net; fa2 alone moves a third of za's air."""
    cfg = _dense_das()
    assert cfg.stuck_pwm_net == cfg.stuck_airflow_net == 0.15
    n = cfg.stuck_params("prox_a2").ticks + 1

    def fans(
        fa1: Callable[[int], float], fa2: Callable[[int], float]
    ) -> Callable[[int], dict[str, float]]:
        return lambda i: {"fa1": fa1(i), "fa2": fa2(i), "fb1": 0.5, "fc1": 0.5}

    apart = fans(lambda i: 0.3 + 0.04 * i, lambda i: 0.9 - 0.08 * i)
    _, r = _history(cfg, n, lambda i: _base_temps(), apart)
    assert not r.stuck["prox_a2"] and not r.stuck["air_c"]
    shared = fans(lambda i: 0.5, lambda i: 0.3 + 0.05 * i)  # +0.2 PWM net, a third of the air
    _, r = _history(cfg, n, lambda i: _base_temps(), shared)
    assert not r.stuck["prox_a2"]
    together = fans(lambda i: 0.3 + 0.04 * i, lambda i: 0.3 + 0.04 * i)
    _, r = _history(cfg, n, lambda i: _base_temps(), together)
    assert r.stuck["prox_a2"] and r.reasons["prox_a2"] == (REASON_STUCK,)


def test_a_short_fan_dip_at_the_window_start_is_no_airflow_evidence():
    """Block means, not single samples: a drive with a time constant of minutes does not
    answer a fan dip of one tick, so the dip must not read as a move of the whole window
    (the DAS MPC dips a fan for a tick or two). A sustained step still counts."""
    cfg = _das(prox_a2={"stuck_s": 40.0, "stuck_decimate": 1})
    n = cfg.stuck_params("prox_a2").ticks

    def stuck_at_the_end(pwm_at: Callable[[int], float]) -> bool:
        replay = DasReplay(cfg)
        r = None
        for i in range(2 * n):
            u = pwm_at(i)
            r = replay.tick(_base_temps(), {"fa1": u, "fa2": u, "fb1": 0.5, "fc1": 0.5})
        assert r is not None
        return r.stuck["prox_a2"]

    # the check on the last tick (2n - 1) sees ticks n - 1 .. 2n - 2: the dip is its oldest
    assert not stuck_at_the_end(lambda i: 0.4 if i == n - 1 else 0.9)
    assert stuck_at_the_end(lambda i: 0.4 if i < n + 5 else 0.9)


def _zone_air_drift(
    air_a: float, air_a2: float | None, peer: float = 0.0
) -> Callable[[int], dict[str, float | None]]:
    """Default (frozen) readings; the zone-air sensors of za drift by ``air_*`` over 8 ticks
    (``None``: a dropout on ``air_a2`` at every tick), and ``prox_a1`` -- a proximal reading
    of another bay of the same zone, so a ``zone_peer`` of ``prox_a2`` -- by ``peer``."""

    def temps(i: int) -> dict[str, float | None]:
        t = _base_temps()
        t["air_a"] = SP + air_a * i / 8
        t["air_a2"] = None if air_a2 is None else SP + air_a2 * i / 8
        t["prox_a1"] = float(t["prox_a1"]) + peer * i / 8
        return t

    return temps


def test_zone_air_against_the_airflow_voids_it_for_a_proximal_reading():
    """More airflow lowers a proximal reading, a warmer zone air raises it: when every
    zone-air sensor with a plausible path moved against the airflow by more than
    stuck_air_oppose_c, a still reading is plausible and the move is no evidence."""
    cfg = _dense_das(air_a={"stuck_s": 8.0}, air_a2={"stuck_s": 8.0})
    tol = cfg.stuck_air_oppose_c
    n = cfg.stuck_params("prox_a2").ticks + 1

    def up(i: int) -> dict[str, float]:
        return {"fa1": 0.3 + 0.04 * i, "fa2": 0.3 + 0.04 * i, "fb1": 0.5, "fc1": 0.5}

    def down(i: int) -> dict[str, float]:
        return {"fa1": 0.7 - 0.04 * i, "fa2": 0.7 - 0.04 * i, "fb1": 0.5, "fc1": 0.5}

    def stuck(temps: Callable[[int], dict[str, float | None]], pwm) -> bool:
        return _history(cfg, n, temps, pwm)[1].stuck["prox_a2"]

    assert not stuck(_zone_air_drift(2 * tol, 2 * tol), up)  # warmer air after more airflow
    assert not stuck(_zone_air_drift(-2 * tol, -2 * tol), down)  # cooler air after less
    assert stuck(_zone_air_drift(tol / 2, tol / 2), up)  # too little to cancel
    assert stuck(_zone_air_drift(-2 * tol, -2 * tol), up)  # cooler air adds to more airflow
    assert stuck(_zone_air_drift(2 * tol, 0.0), up)  # every zone-air sensor must agree
    assert not stuck(_zone_air_drift(2 * tol, None), up)  # a dropout has no plausible path

    def spiked(i: int) -> dict[str, float | None]:
        t = _zone_air_drift(2 * tol, None)(i)
        if i == 4:
            t["air_a"] = 90.0  # a Spike: no plausible path either, so the evidence stays
        return t

    assert stuck(spiked, up)
    # a frozen zone-air reading is flagged by the airflow and does not void it either
    _, r = _history(cfg, n, _zone_air_drift(0.0, 2 * tol), up)
    assert r.stuck["air_a"] and r.stuck["prox_a2"]  # air_a did not oppose: evidence stays


def test_a_zone_air_swing_above_the_bound_no_longer_voids_the_airflow_evidence():
    """Item 59: the cancellation has an upper bound. An airflow move can shift the
    drive-to-air difference by a few degrees C at most, so a zone-air swing larger than
    stuck_air_oppose_max_c cannot explain a reading that did not move at all."""
    cfg = _dense_das(air_a={"stuck_s": 8.0}, air_a2={"stuck_s": 8.0})
    lo, hi = cfg.stuck_air_oppose_c, cfg.stuck_air_oppose_max_c
    assert lo < hi
    n = cfg.stuck_params("prox_a2").ticks + 1

    def up(i: int) -> dict[str, float]:
        return {"fa1": 0.3 + 0.04 * i, "fa2": 0.3 + 0.04 * i, "fb1": 0.5, "fc1": 0.5}

    def stuck(air_a: float, air_a2: float, peer: float = 0.0) -> bool:
        return _history(cfg, n, _zone_air_drift(air_a, air_a2, peer), up)[1].stuck["prox_a2"]

    assert not stuck(2 * lo, 2 * lo)  # inside the bound: a plausible cancellation
    assert not stuck(hi - 0.5, hi - 0.5)
    assert stuck(hi + 0.5, hi + 0.5)  # too large to be cancelled: the evidence stands
    assert stuck(hi + 0.5, 2 * lo)  # one sensor past the bound is enough
    # and an excused move is not the end of the check: an opposition another bay's reading
    # followed is the item 58 evidence, so more fan activity cannot hide what less catches
    assert cfg.stuck_zone_air_dT_c < hi - 0.5
    assert stuck(hi - 0.5, hi - 0.5, peer=2.0)
    assert not stuck(2 * lo, 2 * lo, peer=2.0)  # an air move below stuck_zone_air_dT_c


def test_a_zone_air_move_at_steady_airflow_flags_a_frozen_proximal_reading():
    """Item 58: with the fans pinned or trimmed slowly there is no airflow move to measure,
    and before this rule such a reading was never flagged. A proximal reading is its zone's
    air plus the drive-to-air difference, which at constant airflow moves only with the
    bay's own power, so a zone-air move above stuck_zone_air_dT_c had to reach it -- as
    long as another proximal reading of the zone did follow it (see the lying-air test
    below), which is what tells a real air move from a drifting air sensor."""
    cfg = _dense_das(air_a={"stuck_s": 8.0}, air_a2={"stuck_s": 8.0})
    thr = cfg.stuck_zone_air_dT_c
    peer = 2.0 * cfg.stuck_sibling_dT_c  # prox_a1 follows the air: plausible corroboration
    n = cfg.stuck_params("prox_a2").ticks + 1

    def pinned(i: int) -> dict[str, float]:
        return dict.fromkeys(cfg.channels, 1.0)  # at pwm_max: nothing to measure

    def trim(i: int) -> dict[str, float]:  # a slow trim, below stuck_airflow_net
        return {"fa1": 0.5 + 0.005 * i, "fa2": 0.5 + 0.005 * i, "fb1": 0.5, "fc1": 0.5}

    def result(drift: float, pwm=pinned, other: float | None = None, peer=peer) -> GateResult:
        air_a2 = drift if other is None else other
        return _history(cfg, n, _zone_air_drift(drift, air_a2, peer), pwm)[1]

    assert not result(thr / 2).stuck["prox_a2"]  # too small to have to show
    r = result(2 * thr)
    assert r.stuck["prox_a2"] and r.reasons["prox_a2"] == (REASON_STUCK,)
    assert not r.stuck["prox_b1"]  # its own zone's air (air_b) did not move
    assert not r.stuck["exhaust"] and not r.stuck["inlet"]  # drive_proximal sensors only
    assert result(2 * thr, trim).stuck["prox_a2"]  # a trim is still a steady airflow
    assert result(2 * thr, other=0.0).stuck["prox_a2"]  # one zone-air sensor is enough
    assert result(-2 * thr).stuck["prox_a2"]  # a falling zone air counts the same


def test_a_drifting_zone_air_sensor_alone_never_flags_the_readings_of_its_zone():
    """The lie of section 4.4 on a zone_air sensor: it drifts while the zone really is
    steady, so every proximal reading of the zone is correctly still. Without corroboration
    the item 58 evidence would brand them all Stuck and fault the zone over one air sensor;
    a proximal reading of another bay moving with the air is what makes the move real."""
    cfg = _dense_das(air_a={"stuck_s": 8.0}, air_a2={"stuck_s": 8.0})
    thr = cfg.stuck_zone_air_dT_c
    n = cfg.stuck_params("prox_a2").ticks + 1

    def pinned(i: int) -> dict[str, float]:
        return dict.fromkeys(cfg.channels, 1.0)

    proximals = [t for t in cfg.temps if cfg.sensors[t].role == "drive_proximal"]
    # air_a drifts far past the threshold; nothing else in za moves
    _, r = _history(cfg, n, _zone_air_drift(4 * thr, 0.0), pinned)
    assert [t for t in proximals if r.stuck[t]] == []  # no reading of za is untrusted
    # the same air move with one other bay's reading following it: now it is evidence
    _, r = _history(cfg, n, _zone_air_drift(4 * thr, 0.0, 2.0 * cfg.stuck_sibling_dT_c), pinned)
    assert r.stuck["prox_a2"]
    # a peer that barely moved does not corroborate: the same bar as a sibling
    _, r = _history(cfg, n, _zone_air_drift(4 * thr, 0.0, 0.5 * cfg.stuck_sibling_dT_c), pinned)
    assert not r.stuck["prox_a2"]


def test_frozen_readings_are_flagged_within_the_documented_time():
    """Section 3: a reading frozen from t0 is flagged at the latest at
    max(t0 + stuck_s, t1 + stuck_s / 2) plus two decimation intervals once its zone's
    relative airflow has stepped by more than stuck_airflow_net at t1 >= t0 + stuck_s / 4,
    and not before the step is a quarter window old. A DS18B20 and a thermistor alike."""
    m = das_mapping()
    m.update(dt=5.0, confirm_s=10.0, fallback_hold_s=20.0, stuck_s=10.0)
    m["sensors"]["prox_a1b"]["quant_c"] = 0.01  # a thermistor on bay a1
    cfg = MpcConfig.from_mapping(m)
    names = ("prox_a2", "prox_a1b")
    assert cfg.stuck_params("prox_a1b").eps_c == pytest.approx(0.015)
    for t1 in (600.0, 2400.0):
        replay = DasReplay(cfg)
        first: dict[str, float] = {}
        for i in range(int((t1 + 1800.0) / cfg.dt)):
            ts = i * cfg.dt
            u = 0.3 if ts < t1 else 0.6  # za's relative airflow +0.33
            r = replay.tick(_base_temps(), {"fa1": u, "fa2": u, "fb1": 0.5, "fc1": 0.5})
            for name in names:
                if r.stuck[name]:
                    first.setdefault(name, ts)
        for name in names:
            p = cfg.stuck_params(name)
            window_s, slack_s = p.ticks * cfg.dt, 2 * p.decimate * cfg.dt
            assert name in first, (name, t1)
            assert t1 + window_s / 4 <= first[name], (name, t1, first[name])
            assert first[name] <= max(window_s, t1 + window_s / 2) + slack_s, (name, t1, first)


def test_idle_bay_plateau_longer_than_its_window_next_to_a_busy_bay_stays_trusted():
    """The report's case: an idle bay's DS18B20 sits on one code for twice stuck_s while the
    neighbouring bay's drive heats up by 3 degC and the fans answer it a little (less than
    stuck_airflow_net). Never Stuck."""
    m = das_mapping()
    m.update(dt=5.0, confirm_s=10.0, fallback_hold_s=20.0, stuck_s=10.0)
    cfg = MpcConfig.from_mapping(m)
    window_s = cfg.stuck_params("prox_a2").ticks * cfg.dt
    replay = DasReplay(cfg)
    ticks = int(2 * window_s / cfg.dt)
    for i in range(ticks):
        heat = min(1.0, i / (ticks / 2))  # the neighbour's burst ramps up over a window
        dither = 0.02 * (-1) ** i  # thermistor noise on the zone-air sensors
        temps = _base_temps()
        temps.update(
            air_a=_q(SP + 0.05 * heat + dither, 0.01),
            air_a2=_q(SP + 0.1 + 0.05 * heat + dither, 0.01),
            prox_a1=_q(41.0 + 3.0 * heat + 0.03 * (-1) ** i, 0.0625),
            prox_a1b=_q(41.2 + 3.0 * heat + 0.03 * (-1) ** i, 0.0625),
            prox_a2=40.0,
            prox_b1=_q(40.0 + 0.1 * (-1) ** i, 0.0625),
            prox_c1=_q(30.0 + 0.1 * (-1) ** i, 0.0625),
        )
        fan = 0.4 + 0.1 * heat  # the solver answers the burst: +0.11 relative airflow
        r = replay.tick(temps, {"fa1": fan, "fa2": fan, "fb1": 0.5, "fc1": 0.5})
        assert not r.stuck["prox_a2"], f"tick {i}: stuck {r.stuck}"
        assert r.per_temp["prox_a2"], f"tick {i}: {r.reasons}"


def test_inlet_without_zone_gets_no_pwm_evidence_but_other_inlets_count():
    cfg = _dense_das()
    n = cfg.stuck_params("inlet").ticks + 1
    assert cfg.stuck_params("inlet").channels == ()

    def ramp(i: int) -> dict[str, float]:
        return dict.fromkeys(cfg.channels, 0.2 + 0.07 * i)

    _, r = _history(cfg, n, lambda i: _base_temps(), ramp)
    assert not r.stuck["inlet"]

    m = das_mapping()
    m["temps"].append("inlet_b")
    m["sensors"]["inlet"].update(stuck_s=8.0, stuck_decimate=1)
    m["sensors"]["inlet_b"] = {"role": "inlet", "redundant": True}
    cfg2 = MpcConfig.from_mapping(m)
    assert cfg2.stuck_params("inlet").siblings == ("inlet_b",)

    def inlet_b_warms(i: int) -> dict[str, float | None]:
        t = _base_temps()
        t["inlet_b"] = 25.0 + 0.2 * i
        return t

    _, r = _history(cfg2, n, inlet_b_warms, lambda i: dict.fromkeys(cfg2.channels, 0.5))
    assert r.stuck["inlet"]


def test_per_sensor_band_and_latch():
    cfg = _dense_das(prox_a2={"stuck_eps_c": 0.2})
    n = cfg.stuck_params("prox_a2").ticks + 1

    def dither(i: int) -> dict[str, float | None]:
        t = _base_temps()
        t["prox_a2"] = 40.0 + 0.125 * (i % 2)  # two DS18B20 codes: inside a 0.2 band
        return t

    def ramp(i: int) -> dict[str, float]:
        return {"fa1": 0.2 + 0.07 * i, "fa2": 0.5, "fb1": 0.5, "fc1": 0.5}

    replay, r = _history(cfg, n, dither, ramp)
    assert r.stuck["prox_a2"] and replay.latch["prox_a2"] == pytest.approx(40.0)
    t = _base_temps()
    t["prox_a2"] = 40.19  # still inside the sensor's own band: latched
    assert replay.tick(t, ramp(n)).stuck["prox_a2"]
    t["prox_a2"] = 40.3  # left the band (the global 0.02 band would have cleared long ago)
    assert not replay.tick(t, ramp(n)).stuck["prox_a2"]


def test_decimated_window_flags_a_truly_frozen_sensor_once_full():
    cfg = _das(prox_a2={"stuck_s": 120.0, "stuck_decimate": 10})
    p = cfg.stuck_params("prox_a2")
    assert (p.ticks, p.decimate, p.samples) == (120, 10, 12)
    replay = DasReplay(cfg)
    first_flag = None
    for i in range(200):
        fan = min(1.0, 0.2 + 0.004 * i)
        r = replay.tick(_base_temps(), {"fa1": fan, "fa2": fan, "fb1": 0.5, "fc1": 0.5})
        # factor 10 is shared with the inlet (600 ticks / 60 samples): the longer keep wins
        assert len(replay.slow["10"]) <= cfg.slow_window_samples[10] == 60
        if r.stuck["prox_a2"] and first_flag is None:
            first_flag = i
    # Sample 10k enters the decimated window one tick after it was taken: the twelfth
    # sample (tick 110) is there at tick 111.
    assert first_flag == 111
    assert r.stuck["prox_a2"]


def test_advance_slow_windows_phase_and_purity(fast_cfg):
    cfg = _das(prox_a2={"stuck_s": 40.0, "stuck_decimate": 4})
    assert cfg.slow_window_samples[4] == 10
    window: tuple[WindowSample, ...] = ()
    slow: dict[str, list] = {}
    for seq in range(12):
        before = {k: list(v) for k, v in slow.items()}
        nxt = advance_slow_windows(slow, window, seq, cfg)
        assert slow == before  # never mutated
        slow = nxt
        raw = sanitize_temps({**_base_temps(), "prox_a2": 40.0 + seq}, cfg.temps)
        window = push_window(window, raw, dict.fromkeys(cfg.channels, 0.1 * seq), cfg.window_ticks)
    # Samples 0, 4 and 8 were taken (sample 8 at seq 9): values 40, 44, 48.
    assert [s["t"]["prox_a2"] for s in slow["4"]] == [40.0, 44.0, 48.0]
    assert [s["p"]["fa1"] for s in slow["4"]] == pytest.approx([0.0, 0.4, 0.8])
    assert advance_slow_windows(slow, (), 0, cfg) == slow  # nothing due without a sample
    assert advance_slow_windows(None, (), 0, fast_cfg) == {}  # legacy: no decimated windows


def test_step_keeps_decimated_windows_bounded_and_drops_them_on_a_gap():
    cfg = _das(prox_a2={"stuck_s": 40.0, "stuck_decimate": 4})
    state = MpcState.cold()
    for i in range(80):
        _, state = step(das_obs(cfg, float(i)), cfg, state)
    sizes = {int(k): len(v) for k, v in state.solver_memory["stuck_slow"].items()}
    assert set(sizes) == set(cfg.slow_window_samples)
    assert all(sizes[k] <= n for k, n in cfg.slow_window_samples.items())
    assert sizes[4] == 10
    assert state.solver_memory["stuck_seq"] == 80
    json.dumps(state.to_dict(), allow_nan=False)
    _, gapped = step(das_obs(cfg, 200.0), cfg, state)  # gap: history dropped
    assert gapped.solver_memory["stuck_seq"] == 1
    assert all(v == [] for v in gapped.solver_memory["stuck_slow"].values())


def test_stuck_proximal_sensor_faults_only_its_zone_through_step():
    cfg = _das(prox_a2={"stuck_s": 30.0, "stuck_decimate": 3})

    class Ramp:
        name = "pi"

        def initialise(self, cfg, req):
            return {ch: 0.0 for ch in cfg.channels if ch not in req.fixed_channels}, {}

        def solve(self, cfg, req):
            pwm = {ch: min(1.0, req.prev_pwm[ch] + 0.02) for ch in cfg.channels}
            pwm.update(req.fixed_channels)
            integrator = {ch: 0.0 for ch in cfg.channels if ch not in req.fixed_channels}
            return SolverResult(pwm=pwm, integrator=integrator)

    state = MpcState.cold()
    cmd = None
    for i in range(80):
        cmd, state = step(das_obs(cfg, float(i), pwm=0.3), cfg, state, solver=Ramp())
        if cmd.mode is Mode.DEGRADED:
            break
    assert cmd is not None and cmd.mode is Mode.DEGRADED
    assert cmd.diagnostics["zones_in_fault"] == ["za"]
    assert cmd.diagnostics["gate"]["reasons"]["prox_a2"] == [REASON_STUCK]
    assert "bay:a2:prox_a2=stuck" in cmd.diagnostics["zones"]["za"]["reasons"]


class _RampSolver:
    """Every channel rises by 0.02 per tick: net PWM evidence for the Stuck rule."""

    name = "pi"

    def initialise(self, cfg, req):
        return {ch: 0.0 for ch in cfg.channels if ch not in req.fixed_channels}, {}

    def solve(self, cfg, req):
        pwm = {ch: min(1.0, req.prev_pwm[ch] + 0.02) for ch in cfg.channels}
        pwm.update(req.fixed_channels)
        integrator = {ch: 0.0 for ch in cfg.channels if ch not in req.fixed_channels}
        return SolverResult(pwm=pwm, integrator=integrator)


def test_median3_decimated_samples_store_the_value_the_gate_checked():
    """With median3 the decimated windows must store the median3 value, even when every
    sensor is decimated (the dense window would otherwise be too short for a median)."""
    m = das_mapping()
    m["median3"] = True
    cfg = MpcConfig.from_mapping(m)
    assert all(cfg.stuck_params(t).decimate > 1 for t in cfg.temps)
    state = MpcState.cold()
    checked: list[float] = []
    for i in range(8):
        value = 36.5 if i == 3 else 35.0  # a one-tick glitch the median hides
        cmd, state = step(das_obs(cfg, float(i), air_b=value), cfg, state)
        checked.append(cmd.diagnostics["gate"]["filtered"]["air_b"])
    assert checked == [35.0] * 8
    # factor 3 took the samples of ticks 0, 3 and 6
    assert [s["t"]["air_b"] for s in state.solver_memory["stuck_slow"]["3"]] == [35.0] * 3


def test_median3_hidden_glitches_do_not_hide_a_frozen_decimated_sensor():
    """A frozen proximal sensor that glitches for one tick at every decimated sample: the
    gate (median3) never sees the glitch, so the decimated Stuck run must not see it
    either, and the sensor's zone faults."""
    m = das_mapping()
    m["median3"] = True
    for name in m["sensors"]:
        if name != "prox_a2":  # keep the dense window at its minimum
            m["sensors"][name].setdefault("stuck_decimate", 3)
    m["sensors"]["prox_a2"].update(stuck_s=30.0, stuck_decimate=3)
    cfg = MpcConfig.from_mapping(m)
    state = MpcState.cold()
    cmd = None
    for i in range(80):
        glitch = 1.5 if i % 3 == 0 and i > 0 else 0.0
        obs = das_obs(cfg, float(i), pwm=0.3, prox_a2=40.0 + glitch)
        cmd, state = step(obs, cfg, state, solver=_RampSolver())
        if cmd.mode is Mode.DEGRADED:
            break
    assert cmd is not None and cmd.mode is Mode.DEGRADED, cmd.diagnostics["gate"]["stuck"]
    assert cmd.diagnostics["zones_in_fault"] == ["za"]
    assert cmd.diagnostics["gate"]["reasons"]["prox_a2"] == [REASON_STUCK]


@pytest.mark.parametrize(
    "garbage",
    [
        [1, 2, 3] * 30,
        ["junk"] * 70,
        [{}] * 70,
        [{"t": 1, "p": {}}] * 70,
        [{"t": {}, "p": None}] * 70,
        [{"t": {"prox_a2": "hot"}, "p": {"fa1": "x"}}] * 70,
        [None] * 70,
    ],
    ids=["ints", "strings", "empty", "t-not-mapping", "p-not-mapping", "bad-values", "nulls"],
)
def test_step_tolerates_malformed_decimated_windows_in_memory(garbage):
    """``solver_memory`` is state like any other: a corrupt ``stuck_slow`` is dropped (like a
    gap), never an exception out of ``step``."""
    cfg = das_cfg()
    state = MpcState.cold()
    for i in range(5):
        _, state = step(das_obs(cfg, float(i)), cfg, state)
    mem = dict(state.solver_memory)
    mem["stuck_slow"] = {k: list(garbage) for k in mem["stuck_slow"]}
    bad = dataclasses.replace(state, solver_memory=mem)
    cmd, nxt = step(das_obs(cfg, 5.0, pwm=state.last_cmd.pwm), cfg, bad)
    assert cmd.mode is Mode.AUTO
    for samples in nxt.solver_memory["stuck_slow"].values():
        for sample in samples:
            assert isinstance(sample["t"], Mapping) and isinstance(sample["p"], Mapping)
    json.dumps(nxt.to_dict(), allow_nan=False)


def test_step_tolerates_integers_too_large_for_a_float_in_memory():
    """A JSON state can carry an integer no float holds (``10**400``). Behind a frozen reading
    the Stuck evidence reads the stored commands: such a value is unusable like a string,
    never an ``OverflowError`` out of ``step``."""
    cfg = das_cfg()
    state = MpcState.cold()
    for i in range(5):
        _, state = step(das_obs(cfg, float(i)), cfg, state)
    mem = dict(state.solver_memory)
    huge = {"t": default_temps(cfg), "p": dict.fromkeys(cfg.channels, 10**400)}
    mem["stuck_slow"] = {k: [dict(huge) for _ in range(70)] for k in mem["stuck_slow"]}
    bad = dataclasses.replace(state, solver_memory=mem)
    cmd, nxt = step(das_obs(cfg, 5.0, pwm=state.last_cmd.pwm), cfg, bad)
    assert not any(cmd.diagnostics["gate"]["stuck"].values())
    json.dumps(nxt.to_dict(), allow_nan=False)
