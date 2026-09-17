"""Section 4.2 nominal operation, closed loop through ``aqua_bridge.sim.plant``.

Written from PROJECT.md, not from the implementation. Every closed-loop
tick goes through :func:`invariants.checked_step`, so a command that
breaks a section 4.1 invariant fails the test even when the story passes.

Golden trajectories live under ``tests/golden/`` as JSON ``(t, temps, pwm)``
rows for a fixed seed and plant, one file per solver
(``<scenario>.<solver>.json``); they are compared with a numeric tolerance
and regenerated only when ``AQUA_BRIDGE_REGEN_GOLDEN=1``.
"""

from __future__ import annotations

import dataclasses
import json
import os
from collections.abc import Callable, Iterable
from pathlib import Path

import pytest

from aqua_bridge.model import Mode, MpcConfig, MpcState
from aqua_bridge.sim.plant import Plant, PlantParams, TickRecord, run_closed_loop
from invariants import TOL, checked_step, make_obs

GOLDEN_DIR = Path(__file__).resolve().parent / "golden"
REGEN_ENV = "AQUA_BRIDGE_REGEN_GOLDEN"
# Numeric, not bitwise: the Pi (any of tests/test_bench_budget.py's PI_MACHINES,
# numpy 2.2) and the dev machine may round differently in the last bits.
GOLDEN_PWM_ATOL = 1e-6
GOLDEN_TEMP_ATOL = 1e-4

# Legacy-shaped scenarios (coolant setpoint): the DAS cases run in test_das_core.py.
pytestmark = pytest.mark.solver_cases("pi", "mpc")

SP = 35.0


@pytest.fixture
def cfg(cfg: MpcConfig, solver_kind) -> MpcConfig:
    """Section 8: every scenario in this module runs for the PI and the MPC solver."""
    return dataclasses.replace(cfg, solver=solver_kind)


COOLANT = "coolant"
AIR = "air"

# ---------------------------------------------------------------------------
# helpers
# ---------------------------------------------------------------------------


def nominal_params(cfg: MpcConfig, **overrides) -> PlantParams:
    """Plant with the example setpoint reachable (~100 W: 36.6 C at PWM 0.5, 30 C at 1.0)."""
    heat = overrides.pop("heat_w", 100.0)
    return PlantParams(dt=cfg.dt, heat_w=heat, **overrides)


def run(
    plant: Plant,
    cfg: MpcConfig,
    ticks: int,
    *,
    state: MpcState | None = None,
    observe_hook=None,
    heat_schedule: Iterable[tuple[int, float]] = (),
) -> list[TickRecord]:
    """Closed loop with the invariants asserted on every tick."""
    return run_closed_loop(
        plant,
        cfg,
        checked_step,
        ticks,
        state=state,
        observe_hook=observe_hook,
        heat_schedule=heat_schedule,
    )


def pwm_of(rec: TickRecord, ch: str) -> float:
    return rec.cmd.pwm[ch]


def coolant_of(rec: TickRecord) -> float:
    return rec.obs.temps[COOLANT]


def pwm_for_equilibrium(plant: Plant, target_c: float) -> float:
    """PWM (same on every channel) whose steady-state coolant temperature is ``target_c``."""
    lo, hi = 0.0, 1.0
    for _ in range(80):
        mid = 0.5 * (lo + hi)
        t_c, _ = plant.equilibrium(dict.fromkeys(plant.channels, mid))
        if t_c > target_c:
            lo = mid  # hotter than wanted -> more fan
        else:
            hi = mid
    return 0.5 * (lo + hi)


def steps_never_exceed_rate(recs: list[TickRecord], cfg: MpcConfig) -> None:
    for a, b in zip(recs, recs[1:], strict=False):
        for ch in cfg.channels:
            assert abs(b.cmd.pwm[ch] - a.cmd.pwm[ch]) <= cfg.d_pwm_max + TOL


def assert_all_auto(recs: list[TickRecord]) -> None:
    bad = [(i, r.cmd.mode.value) for i, r in enumerate(recs) if r.cmd.mode is Mode.FALLBACK]
    assert not bad, f"nominal loop entered fallback at ticks {bad[:5]}"


# ---------------------------------------------------------------------------
# at setpoint
# ---------------------------------------------------------------------------


def test_at_setpoint_pwm_settles_with_bounded_ripple(cfg):
    """Temps already at target -> PWM settles; ripple below a bound for N steps."""
    plant = Plant(nominal_params(cfg), initial_pwm=0.5)
    pwm_eq = pwm_for_equilibrium(plant, SP)
    plant = Plant(nominal_params(cfg), initial_pwm=pwm_eq)  # equilibrium at the setpoint
    assert abs(plant.t_coolant - SP) < 1e-6

    recs = run(plant, cfg, 300)
    assert_all_auto(recs)
    steps_never_exceed_rate(recs, cfg)
    for ch in cfg.channels:
        series = [pwm_of(r, ch) for r in recs]
        ripple = max(series) - min(series)
        assert ripple < 0.02, f"{ch}: PWM ripple {ripple:.4f} at the setpoint"
    temps = [coolant_of(r) for r in recs]
    assert max(abs(t - SP) for t in temps) < 0.3


def test_at_setpoint_open_loop_output_is_constant(cfg):
    """Pure step: identical at-setpoint observations -> identical commands, mode auto."""
    state = MpcState.cold()
    first = None
    for i in range(40):
        cmd, state = checked_step(make_obs(cfg, i * cfg.dt, coolant=SP), cfg, state)
        assert cmd.mode is Mode.AUTO
        first = cmd.pwm if first is None else first
        assert cmd.pwm == pytest.approx(first, abs=1e-9)


# ---------------------------------------------------------------------------
# setpoint step up / step down
# ---------------------------------------------------------------------------


def settle(cfg: MpcConfig, ticks: int = 900, **params) -> tuple[Plant, list[TickRecord]]:
    plant = Plant(nominal_params(cfg, **params), initial_pwm=0.5)
    recs = run(plant, cfg, ticks)
    assert_all_auto(recs)
    assert abs(coolant_of(recs[-1]) - SP) < 0.5, "did not settle before the story starts"
    return plant, recs


@pytest.mark.parametrize("new_sp", [38.0, 32.0], ids=["step_up", "step_down"])
def test_setpoint_step_moves_pwm_the_right_way_and_temp_crosses(cfg, new_sp):
    """Setpoint change -> PWM moves the right way; temp crosses toward the target in time."""
    plant, settled = settle(cfg)
    before_pwm = settled[-1].cmd.pwm
    before_t = coolant_of(settled[-1])
    cfg2 = dataclasses.replace(cfg, setpoints={COOLANT: new_sp})
    recs = run(plant, cfg2, 900, state=settled[-1].state)
    assert_all_auto(recs)
    steps_never_exceed_rate([settled[-1], *recs], cfg2)

    sign = -1.0 if new_sp > SP else 1.0  # hotter target -> less fan; colder -> more fan
    early = recs[: 3 * cfg2.confirm_ticks]
    for ch in cfg2.channels:
        moved = pwm_of(early[-1], ch) - before_pwm[ch]
        assert sign * moved > 0.01, f"{ch}: PWM moved {moved:+.4f}, expected sign {sign:+.0f}"

    # Deadline: coolant covers at least half the distance to the new target within 15 min.
    deadline = int(900 / cfg2.dt)
    gap0 = abs(before_t - new_sp)
    gaps = [abs(coolant_of(r) - new_sp) for r in recs[:deadline]]
    assert min(gaps) < 0.5 * gap0, f"coolant did not approach {new_sp}: min gap {min(gaps):.2f}"
    assert abs(coolant_of(recs[-1]) - new_sp) < 0.6


# ---------------------------------------------------------------------------
# disturbance
# ---------------------------------------------------------------------------


def test_heat_disturbance_pwm_rises_then_recovers(cfg):
    """Extra heat in the plant -> PWM rises, then recovers when the heat goes away."""
    plant, settled = settle(cfg)
    base = settled[-1].cmd.pwm
    burst_on, burst_off, total = 0, 300, 1500
    recs = run(
        plant,
        cfg,
        total,
        state=settled[-1].state,
        heat_schedule=[(burst_on, 140.0), (burst_off, 100.0)],
    )
    assert_all_auto(recs)
    for ch in cfg.channels:
        during = max(pwm_of(r, ch) for r in recs[burst_on:burst_off])
        assert during > base[ch] + 0.1, f"{ch}: PWM did not rise under extra heat"
        after = pwm_of(recs[-1], ch)
        assert abs(after - base[ch]) < 0.05, (
            f"{ch}: PWM did not recover ({after:.3f} vs {base[ch]:.3f})"
        )
    assert abs(coolant_of(recs[-1]) - SP) < 0.6


# ---------------------------------------------------------------------------
# cold start / warm start
# ---------------------------------------------------------------------------


@pytest.mark.parametrize("pwm_min", [0.1, 0.15], ids=["pwm_min_le_dpwm", "example_pwm_min"])
def test_cold_start_pwm_zero_temps_high_ramps_up_without_skipping(cfg, pwm_min):
    """Obs PWM 0, temps high -> ramps up by at most d_pwm_max per tick, never down.

    Section 4.2 describes the ramp from ``obs.pwm = 0``. With the example
    ``pwm_min = 0.15 > d_pwm_max = 0.1`` no command can be both inside
    ``[pwm_min, pwm_max]`` and within ``d_pwm_max`` of 0, so section 4.1 lets
    ``prev`` fall through to ``fallback_pwm``; both readings are exercised
    and in both the PWM must rise monotonically at bounded rate.
    """
    cfg = dataclasses.replace(cfg, pwm_min=pwm_min)
    plant = Plant(nominal_params(cfg), initial_pwm=0.0, t_coolant=55.0, t_air=35.0)
    recs = run(plant, cfg, 40)
    assert_all_auto(recs)
    steps_never_exceed_rate(recs, cfg)
    for ch in cfg.channels:
        series = [pwm_of(r, ch) for r in recs]
        # the ramp: monotone up to the rail, one d_pwm_max at a time, no skipped step
        top = next(i for i, v in enumerate(series) if v >= 0.99)
        ramp = series[: top + 1]
        assert all(b >= a - TOL for a, b in zip(ramp, ramp[1:], strict=False)), (
            f"{ch}: PWM decreased during the cold-start ramp: {ramp}"
        )
        assert all(b - a <= cfg.d_pwm_max + TOL for a, b in zip(ramp, ramp[1:], strict=False))
        needed = int(round((1.0 - series[0]) / cfg.d_pwm_max))
        assert top <= needed + 1, f"{ch}: ramp took {top} ticks, expected about {needed}"
        # and it stays high for as long as the loop is far too hot
        hot = [v for v, r in zip(series, recs, strict=True) if coolant_of(r) > SP + 10.0]
        assert min(hot[top:]) > 0.9, f"{ch}: PWM dropped while the loop was far too hot"
        assert series[0] >= cfg.pwm_min - TOL


def test_warm_start_at_pwm_max_temps_falling_ramps_down(cfg):
    """Already at pwm_max, temps then fall -> PWM ramps down, bounded per tick."""
    plant = Plant(nominal_params(cfg), initial_pwm=1.0, t_coolant=SP, t_air=28.0)
    recs = run(plant, cfg, 300)
    assert_all_auto(recs)
    steps_never_exceed_rate(recs, cfg)
    assert recs[0].cmd.pwm == pytest.approx(dict.fromkeys(cfg.channels, 1.0), abs=1e-9), (
        "first tick with trusted obs.pwm at pwm_max must start bumpless from it"
    )
    temps = [coolant_of(r) for r in recs]
    assert min(temps[: int(240 / cfg.dt)]) < SP - 0.3, (
        "the plant should have cooled below the setpoint"
    )
    for ch in cfg.channels:
        series = [pwm_of(r, ch) for r in recs]
        assert series[-1] < 0.8, f"{ch}: PWM stayed high while the loop was cold"
        # "temps then fall -> ramps down": while the coolant is below the setpoint and still
        # falling, PWM must not rise (how far it undershoots before turning is the solver's
        # business: a predictive solver turns earlier than PI and undershoots less)
        for k in range(1, int(240 / cfg.dt)):
            if temps[k] < SP and temps[k] < temps[k - 1]:
                assert series[k] <= series[k - 1] + TOL, f"{ch}: PWM rose at tick {k} while cooling"


# ---------------------------------------------------------------------------
# multi-channel
# ---------------------------------------------------------------------------


def test_multi_channel_one_rises_while_the_other_holds(cfg):
    """Two fans on different temperatures: one increases while the other holds; no channel drops."""
    cfg2 = dataclasses.replace(
        cfg,
        setpoints={COOLANT: SP, AIR: 29.0},
        channel_temps={"radiator": (COOLANT,), "intake": (AIR,)},
    )
    state = MpcState.cold()
    cmds = []
    for i in range(12):
        obs = make_obs(
            cfg2, i * cfg2.dt, coolant=SP + 3.0, air=29.0, pwm=dict.fromkeys(cfg2.channels, 0.5)
        )
        cmd, state = checked_step(obs, cfg2, state)
        assert set(cmd.pwm) == set(cfg2.channels)  # no silent drop of a channel
        cmds.append(cmd)
    radiator = [c.pwm["radiator"] for c in cmds]
    intake = [c.pwm["intake"] for c in cmds]
    assert radiator[-1] > radiator[0] + 0.05, f"radiator did not rise: {radiator}"
    assert all(b >= a - TOL for a, b in zip(radiator, radiator[1:], strict=False))
    assert intake == pytest.approx([intake[0]] * len(intake), abs=1e-9), f"intake moved: {intake}"


def test_multi_channel_closed_loop_keeps_every_channel_every_tick(cfg):
    plant = Plant(nominal_params(cfg), initial_pwm={"radiator": 0.3, "intake": 0.9})
    recs = run(plant, cfg, 400)
    assert_all_auto(recs)
    for r in recs:
        assert set(r.cmd.pwm) == set(cfg.channels)
    # Both fans regulate the same error. Section 4.2 does not ask their commands
    # to become equal, and a per-channel PI keeps the split it was bumplessly
    # initialised with (0.3 / 0.9 here) except where the integrator clamp at
    # the rail equalises it; what must hold is that the split narrowed and both
    # ended within one d_pwm_max of each other.
    first = recs[0].cmd.pwm
    last = recs[-1].cmd.pwm
    gap0 = abs(first["radiator"] - first["intake"])
    gap = abs(last["radiator"] - last["intake"])
    assert gap < gap0
    assert gap < cfg.d_pwm_max


# ---------------------------------------------------------------------------
# saturation is honest
# ---------------------------------------------------------------------------


def test_saturation_is_honest_when_plant_needs_more_than_pwm_max(cfg):
    """Plant needs more than pwm_max -> mode=saturated, PWM pinned at max, no exception."""
    plant = Plant(nominal_params(cfg, heat_w=220.0), initial_pwm=0.5)
    t_at_max, _ = plant.equilibrium(dict.fromkeys(plant.channels, 1.0))
    assert t_at_max > SP + 1.0, "test plant must be unreachable at pwm_max"
    recs = run(plant, cfg, 600)
    assert_all_auto(recs)
    steps_never_exceed_rate(recs, cfg)
    saturated = [r for r in recs if r.cmd.mode is Mode.SATURATED]
    assert saturated, "plant unreachable at pwm_max but mode never became saturated"
    for r in saturated:
        assert max(r.cmd.pwm.values()) == pytest.approx(cfg.pwm_max, abs=TOL)
    tail = recs[-100:]
    assert all(r.cmd.mode is Mode.SATURATED for r in tail)
    assert all(v == pytest.approx(cfg.pwm_max, abs=TOL) for r in tail for v in r.cmd.pwm.values())


def test_saturated_mode_is_not_reported_while_pwm_is_below_max(cfg):
    for r in run(Plant(nominal_params(cfg, heat_w=220.0), initial_pwm=0.5), cfg, 300):
        if r.cmd.mode is Mode.SATURATED:
            assert max(r.cmd.pwm.values()) >= cfg.pwm_max - TOL


# ---------------------------------------------------------------------------
# golden trajectories
# ---------------------------------------------------------------------------


def _golden_regulation(cfg: MpcConfig) -> list[TickRecord]:
    plant = Plant(
        nominal_params(cfg, noise_sigma_c=0.2, delay_ticks=1),
        initial_pwm=0.5,
        t_coolant=40.0,
        t_air=30.0,
        seed=20240613,
    )
    return run(plant, cfg, 500, heat_schedule=[(200, 130.0), (350, 100.0)])


def _golden_setpoint_steps(cfg: MpcConfig) -> list[TickRecord]:
    plant = Plant(nominal_params(cfg), initial_pwm=0.5, t_coolant=SP, t_air=29.0, seed=1)
    recs = run(plant, cfg, 200)
    cfg_up = dataclasses.replace(cfg, setpoints={COOLANT: 38.0})
    recs += run(plant, cfg_up, 200, state=recs[-1].state)
    cfg_down = dataclasses.replace(cfg, setpoints={COOLANT: 33.0})
    recs += run(plant, cfg_down, 200, state=recs[-1].state)
    return recs


GOLDEN_SCENARIOS: dict[str, Callable[[MpcConfig], list[TickRecord]]] = {
    "regulation_noise_disturbance": _golden_regulation,
    "setpoint_steps": _golden_setpoint_steps,
}


def _rows(recs: list[TickRecord]) -> list[dict]:
    return [
        {
            "t": r.obs.ts,
            "temps": {k: float(v) for k, v in r.obs.temps.items()},
            "pwm": {k: float(v) for k, v in r.cmd.pwm.items()},
            "mode": r.cmd.mode.value,
        }
        for r in recs
    ]


@pytest.mark.parametrize("name", sorted(GOLDEN_SCENARIOS))
def test_golden_trajectory(cfg, name):
    """(t, temps, pwm) for a fixed seed and plant; regression with a numeric tolerance."""
    recs = GOLDEN_SCENARIOS[name](cfg)
    rows = _rows(recs)
    path = GOLDEN_DIR / f"{name}.{cfg.solver.value}.json"  # one golden per solver
    if os.environ.get(REGEN_ENV) == "1":
        GOLDEN_DIR.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps({"scenario": name, "rows": rows}, indent=0) + "\n")
        pytest.skip(f"regenerated {path}")
    assert path.is_file(), f"missing golden file {path}; run with {REGEN_ENV}=1 to create it"
    golden = json.loads(path.read_text())["rows"]
    assert len(golden) == len(rows), f"trajectory length {len(rows)} != golden {len(golden)}"
    for i, (g, r) in enumerate(zip(golden, rows, strict=True)):
        assert r["t"] == pytest.approx(g["t"], abs=1e-9), f"tick {i}: t"
        assert set(r["pwm"]) == set(g["pwm"]), f"tick {i}: pwm keys"
        for ch, v in g["pwm"].items():
            assert r["pwm"][ch] == pytest.approx(v, abs=GOLDEN_PWM_ATOL), f"tick {i}: pwm[{ch}]"
        for tname, v in g["temps"].items():
            assert r["temps"][tname] == pytest.approx(v, abs=GOLDEN_TEMP_ATOL), (
                f"tick {i}: temps[{tname}]"
            )
        assert r["mode"] == g["mode"], f"tick {i}: mode {r['mode']} != {g['mode']}"


def test_golden_scenarios_are_deterministic(cfg):
    """Same seed and plant twice -> identical rows (a golden file only makes sense then)."""
    for name, make in GOLDEN_SCENARIOS.items():
        a, b = _rows(make(cfg)), _rows(make(cfg))
        assert a == b, f"{name}: two runs with the same seed differ"
