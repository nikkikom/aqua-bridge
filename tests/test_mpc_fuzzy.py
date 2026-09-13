"""Section 4.5 property tests with Hypothesis (``pytest.mark.fuzzy``).

Example counts come from the profiles in ``tests/conftest.py``
(``HYPOTHESIS_PROFILE=dev|ci|pi``); no test raises ``max_examples`` above
its profile. Failures shrink and print the reproduction blob
(``print_blob=True`` in every profile). The HTTP fuzz bullet belongs to the
publishers suite.
"""

from __future__ import annotations

import dataclasses
import math
from pathlib import Path

import pytest
from hypothesis import HealthCheck, assume, example, given, settings
from hypothesis import strategies as st

from aqua_bridge.config import load_config
from aqua_bridge.control.mpc import step
from aqua_bridge.model import Mode, MpcConfig, MpcState, PlantObservation, SolverKind
from aqua_bridge.sim.plant import Plant, PlantParams, run_closed_loop
from invariants import (
    TOL,
    assert_command_safe,
    assert_state_finite,
    checked_step,
    make_obs,
    obs_structurally_untrusted,
    resolve_prev_pwm,
)

pytestmark = pytest.mark.fuzzy

# Hypothesis forbids function-scoped fixtures under @given (the fixture would be built once
# per test, not per example); the example config is immutable, so load it once here.
EXAMPLE_CFG: MpcConfig = load_config(
    Path(__file__).resolve().parent.parent / "config.example.yaml"
).mpc
# Section 8: every property runs for the PI and the MPC solver. Direct parametrisation of
# ``cfg`` is allowed next to @given (it is not a function-scoped fixture).
SOLVER_CFGS = [dataclasses.replace(EXAMPLE_CFG, solver=k) for k in (SolverKind.PI, SolverKind.MPC)]
both_solvers = pytest.mark.parametrize(
    "cfg", SOLVER_CFGS, ids=[c.solver.value for c in SOLVER_CFGS]
)

SP = 35.0
COOLANT = "coolant"
AIR = "air"

# --- strategies -------------------------------------------------------------

finite = st.floats(allow_nan=False, allow_infinity=False)
temp_box = st.floats(min_value=10.0, max_value=80.0)
pwm_box = st.floats(min_value=0.0, max_value=1.0)
rpm_box = st.floats(min_value=0.0, max_value=4000.0)


def obs_in_box(cfg: MpcConfig, ts: float):
    return st.builds(
        PlantObservation,
        temps=st.fixed_dictionaries(dict.fromkeys(cfg.temps, temp_box)),
        rpm=st.fixed_dictionaries(dict.fromkeys(cfg.channels, rpm_box)),
        pwm=st.fixed_dictionaries(dict.fromkeys(cfg.channels, pwm_box)),
        ts=st.just(ts),
    )


LIES = ["spike_hi", "spike_lo", "none", "nan", "inf", "missing", "extra", "garbage", "swap", "jump"]


def apply_lie(kind: str, obs: PlantObservation) -> PlantObservation:
    temps = dict(obs.temps)
    if kind == "spike_hi":
        temps[COOLANT] = 125.0
    elif kind == "spike_lo":
        temps[COOLANT] = -40.0
    elif kind == "none":
        temps[COOLANT] = None
    elif kind == "nan":
        temps[AIR] = math.nan
    elif kind == "inf":
        temps[COOLANT] = math.inf
    elif kind == "missing":
        temps.pop(COOLANT, None)
    elif kind == "extra":
        temps["gpu"] = 45.0
    elif kind == "garbage":
        temps[COOLANT] = 32767.0
    elif kind == "swap":
        temps[COOLANT], temps[AIR] = temps.get(AIR), temps.get(COOLANT)
    elif kind == "jump":
        v = temps.get(COOLANT)
        temps[COOLANT] = None if v is None else v + 30.0
    return dataclasses.replace(obs, temps=temps)


# --- properties -------------------------------------------------------------


@both_solvers
@given(data=st.data())
def test_random_valid_obs_in_plausible_boxes_hold_invariants(cfg, data):
    """Random obs (temps 10-80, pwm 0-1, rpm 0-4000, ts += dt) -> invariants every tick."""
    n = data.draw(st.integers(min_value=1, max_value=25))
    state = MpcState.cold()
    for i in range(n):
        obs = data.draw(obs_in_box(cfg, i * cfg.dt))
        cmd, state = checked_step(obs, cfg, state)
        assert cmd.mode in tuple(Mode)


@both_solvers
@settings(suppress_health_check=[HealthCheck.too_slow, HealthCheck.data_too_large])
@given(data=st.data())
def test_random_sequences_of_small_perturbations(cfg, data):
    """20-100 steps, each obs a small perturbation of the last: invariants every tick,
    deterministic step, PWM total variation bounded by the rate limit."""
    n = data.draw(st.integers(min_value=20, max_value=100))
    temps = {name: data.draw(st.floats(min_value=25.0, max_value=45.0)) for name in cfg.temps}
    pwm = dict.fromkeys(cfg.channels, data.draw(st.floats(min_value=0.15, max_value=1.0)))
    state = MpcState.cold()
    series: dict[str, list[float]] = {ch: [] for ch in cfg.channels}
    for i in range(n):
        for name in cfg.temps:
            temps[name] += data.draw(st.floats(min_value=-0.5, max_value=0.5))
        for ch in cfg.channels:
            pwm[ch] = min(
                1.0, max(0.0, pwm[ch] + data.draw(st.floats(min_value=-0.05, max_value=0.05)))
            )
        obs = make_obs(cfg, i * cfg.dt, temps=dict(temps), pwm=dict(pwm))
        cmd, nxt = checked_step(obs, cfg, state)
        cmd2, nxt2 = step(obs, cfg, state)
        assert cmd2.to_dict() == cmd.to_dict() and nxt2.to_dict() == nxt.to_dict(), (
            "non-deterministic"
        )
        state = nxt
        for ch in cfg.channels:
            series[ch].append(cmd.pwm[ch])
        # small perturbations of a plausible plant are never a fault
        assert cmd.mode is not Mode.FALLBACK, f"tick {i}: slow drift treated as a fault"
    for ch in cfg.channels:
        s = series[ch]
        tv = sum(abs(b - a) for a, b in zip(s, s[1:], strict=False))
        assert tv <= (n - 1) * cfg.d_pwm_max + TOL


@both_solvers
@settings(suppress_health_check=[HealthCheck.too_slow, HealthCheck.data_too_large])
@given(data=st.data())
def test_random_lies_mixed_in_keep_invariants_and_fallback_when_untrusted(cfg, data):
    """With probability p apply a section 4.4 lie: invariants hold; mode=fallback whenever
    the gate says untrusted, and whenever the observation is structurally untrusted."""
    n = data.draw(st.integers(min_value=10, max_value=60))
    p = data.draw(st.floats(min_value=0.0, max_value=0.6))
    median3 = data.draw(st.booleans())
    cfg = dataclasses.replace(cfg, median3=median3)
    state = MpcState.cold()
    temps = dict.fromkeys(cfg.temps, SP)
    for i in range(n):
        for name in cfg.temps:
            temps[name] += data.draw(st.floats(min_value=-0.3, max_value=0.3))
        obs = make_obs(cfg, i * cfg.dt, temps=dict(temps))
        if data.draw(st.floats(min_value=0.0, max_value=1.0)) < p:
            obs = apply_lie(data.draw(st.sampled_from(LIES)), obs)
        prev = resolve_prev_pwm(state, obs, cfg)
        cmd, state = checked_step(obs, cfg, state)
        if not cmd.diagnostics["trusted"]:
            assert cmd.mode is Mode.FALLBACK, f"tick {i}: untrusted but mode {cmd.mode}"
            assert state.in_fault
            # an untrusted tick never steps toward pwm_min because of the fault:
            # not below prev (only the clamp into [pwm_min, pwm_max] may lower it)
            for ch in cfg.channels:
                lo = min(prev[ch], cfg.pwm_max)
                assert cmd.pwm[ch] >= lo - TOL
        if obs_structurally_untrusted(obs, cfg):
            assert cmd.mode is Mode.FALLBACK


@both_solvers
@example(setpoint=25.0, coolant=20.0, air=10.0, pwm0=0.0625)  # MPC rose off pwm_min on a cold loop
@given(
    setpoint=st.floats(min_value=25.0, max_value=45.0),
    coolant=st.floats(min_value=10.0, max_value=80.0),
    air=st.floats(min_value=10.0, max_value=80.0),
    pwm0=st.floats(min_value=0.0, max_value=1.0),
)
def test_random_setpoints_in_a_sane_band(cfg, setpoint, coolant, air, pwm0):
    """Section 4.2 "setpoint change -> PWM moves the right way", for random setpoints.

    The plant is held still, so the only thing that can move the PWM after the
    first command is the controller's own transient (PI integral, MPC disturbance
    estimator). Compared tick to tick: while the loop is more than 0.5 degC hot
    the PWM never falls, while it is more than 0.5 degC cold it never rises --
    from a rail too (a rise off ``pwm_min`` on a cold loop is as wrong as one
    from mid-range, and the first command is already inside ``[pwm_min,
    pwm_max]``, so no clamp can force a move afterwards).
    """
    cfg = dataclasses.replace(cfg, setpoints={COOLANT: setpoint})
    state = MpcState.cold()
    obs = make_obs(cfg, 0.0, coolant=coolant, air=air, pwm=dict.fromkeys(cfg.channels, pwm0))
    cmd, state = checked_step(obs, cfg, state)
    assert cmd.mode is not Mode.FALLBACK  # a valid first sample is trusted
    err = coolant - setpoint
    for i in range(1, 6):
        obs = make_obs(cfg, i * cfg.dt, coolant=coolant, air=air, pwm=dict(cmd.pwm))
        nxt, state = checked_step(obs, cfg, state)
        for ch in cfg.channels:
            if err > 0.5:
                assert nxt.pwm[ch] >= cmd.pwm[ch] - TOL, f"tick {i}: hot loop but PWM fell on {ch}"
            if err < -0.5:
                assert nxt.pwm[ch] <= cmd.pwm[ch] + TOL, f"tick {i}: cold loop but PWM rose on {ch}"
        cmd = nxt


@both_solvers
@settings(suppress_health_check=[HealthCheck.too_slow, HealthCheck.data_too_large])
@given(
    setpoint=st.floats(min_value=25.0, max_value=45.0),
    c_coolant=st.floats(min_value=1000.0, max_value=20000.0),
    c_air=st.floats(min_value=200.0, max_value=2500.0),
    g_rad=st.floats(min_value=5.0, max_value=70.0),
    g_int=st.floats(min_value=3.0, max_value=50.0),
    heat=st.floats(min_value=20.0, max_value=130.0),
    delay=st.integers(min_value=0, max_value=2),
    noise=st.floats(min_value=0.0, max_value=0.3),
    seed=st.integers(min_value=0, max_value=2**31 - 1),
    ticks=st.integers(min_value=40, max_value=150),
)
def test_random_plant_parameters_closed_loop_no_nan_pwm_in_limits(
    cfg, setpoint, c_coolant, c_air, g_rad, g_int, heat, delay, noise, seed, ticks
):
    cfg = dataclasses.replace(cfg, setpoints={COOLANT: setpoint})
    params = PlantParams(
        dt=cfg.dt,
        heat_w=heat,
        c_coolant_j_per_k=c_coolant,
        c_air_j_per_k=c_air,
        radiator_fans={"radiator": g_rad},
        intake_fans={"intake": g_int},
        delay_ticks=delay,
        noise_sigma_c=noise,
    )
    plant = Plant(params, initial_pwm=0.5, seed=seed)
    assume(plant.t_coolant < cfg.temp_max_c - 5.0)  # a plant the sensors can even report
    recs = run_closed_loop(plant, cfg, checked_step, ticks)
    assert len(recs) == ticks
    for r in recs:
        for v in r.obs.temps.values():
            assert math.isfinite(v)
        for ch in cfg.channels:
            assert cfg.pwm_min - TOL <= r.cmd.pwm[ch] <= cfg.pwm_max + TOL
    assert math.isfinite(plant.t_coolant) and math.isfinite(plant.t_air)


# --- malformed observation construction ---------------------------------------

junk = st.one_of(
    st.none(),
    st.booleans(),
    st.text(max_size=5),
    st.floats(),  # includes NaN / inf
    st.integers(min_value=-(10**6), max_value=10**6),
    st.lists(st.floats(), max_size=3),
    st.dictionaries(st.text(max_size=3), st.floats(), max_size=3),
)
junk_map = st.one_of(
    junk,
    st.dictionaries(st.one_of(st.text(max_size=8), st.integers()), junk, max_size=4),
)


@both_solvers
@settings(suppress_health_check=[HealthCheck.too_slow, HealthCheck.data_too_large])
@given(
    temps=junk_map,
    rpm=junk_map,
    pwm=junk_map,
    ts=junk,
    extra=st.dictionaries(st.sampled_from(["foo", "bar", "temps2", "mode"]), junk, max_size=2),
)
def test_malformed_observation_rejected_or_fallback_never_bad_pwm(cfg, temps, rpm, pwm, ts, extra):
    """Extra keys, wrong types: either TypeError/ValueError at construction, or a step that
    still satisfies every invariant (fallback when the data is unusable)."""
    try:
        obs = PlantObservation(temps=temps, rpm=rpm, pwm=pwm, ts=ts, **extra)
    except (TypeError, ValueError):
        return
    # constructed: then step must never raise and never emit a bad PWM
    for state in (MpcState.cold(), _warm_state(cfg)):
        prev = resolve_prev_pwm(state, obs, cfg)
        cmd, nxt = step(obs, cfg, state)
        assert_command_safe(obs, cfg, cmd, prev)
        assert_state_finite(nxt)
        if obs_structurally_untrusted(obs, cfg):
            assert cmd.mode is Mode.FALLBACK


def _warm_state(cfg: MpcConfig) -> MpcState:
    state = MpcState.cold()
    for i in range(3):
        _, state = step(make_obs(cfg, i * cfg.dt, coolant=SP), cfg, state)
    return state


@both_solvers
@given(temps=st.dictionaries(st.text(max_size=6), st.one_of(finite, st.none()), max_size=4))
def test_arbitrary_temperature_dicts_never_crash_step(cfg, temps):
    """Any str-keyed temps dict: constructed fine, stepped fine, fallback unless complete."""
    obs = PlantObservation(temps=temps, rpm={}, pwm={}, ts=0.0)
    cmd, state = checked_step(obs, cfg, MpcState.cold())
    complete = set(temps) == set(cfg.temps) and all(
        v is not None and cfg.temp_min_c <= v <= cfg.temp_max_c for v in temps.values()
    )
    if not complete:
        assert cmd.mode is Mode.FALLBACK
        assert cmd.pwm == pytest.approx(cfg.fallback_pwm)  # section 3 item 3: prev = fallback
