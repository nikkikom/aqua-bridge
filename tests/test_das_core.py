"""Core suites on the zoned DAS config (plan section 9, "core suites via solver_kind").

Every ``solver_kind`` DAS case (``pi_das`` today) runs the section 4.1 invariants
on ``config.example-das.yaml``: first-tick ``prev`` rules, determinism and JSON
state, no step toward ``pwm_min`` because of a fault, honest saturation, random
per-zone lies, and closed loops on the DAS truth plant (:mod:`aqua_bridge.sim.das`)
with every tick through :func:`invariants.checked_step`. The golden trajectories
``das_regulation.<case>.json`` and ``das_hotswap.<case>.json`` pin the closed-loop
behaviour; like the legacy goldens they are compared with a numeric tolerance and
regenerated only with ``AQUA_BRIDGE_REGEN_GOLDEN=1`` (the legacy goldens are
never touched by this module).
"""

from __future__ import annotations

import dataclasses
import json
import math
import os
from collections import Counter
from pathlib import Path

import pytest
from hypothesis import given, settings
from hypothesis import strategies as st

from aqua_bridge.config import load_config
from aqua_bridge.control import estimates as est
from aqua_bridge.control.mpc import step
from aqua_bridge.hw import sources as hw_sources
from aqua_bridge.model import Mode, MpcConfig, MpcState, PlantObservation
from aqua_bridge.sim.das import (
    SENSOR_TYPES,
    DasRun,
    build_das_plant,
    run_das_closed_loop,
    topology_from_config,
)
from conftest import EXAMPLE_DAS_CONFIG
from das_fixtures import das_obs
from invariants import TOL, assert_no_non_finite, checked_step

pytestmark = pytest.mark.solver_cases("pi_das")

GOLDEN_DIR = Path(__file__).resolve().parent / "golden"
REGEN_ENV = "AQUA_BRIDGE_REGEN_GOLDEN"
GOLDEN_PWM_ATOL = 1e-6
GOLDEN_TEMP_ATOL = 1e-4


@pytest.fixture
def cfg(das_example_cfg: MpcConfig, solver_kind) -> MpcConfig:
    """The DAS example config with the case's solver."""
    assert solver_kind.das
    return dataclasses.replace(das_example_cfg, solver=solver_kind.kind)


def prox_for(t_drive: float, t_air: float = 35.0) -> float:
    """Proximal reading whose prior-map drive estimate is ``t_drive``."""
    return (1.0 - est.PRIOR_BETA) * t_drive + est.PRIOR_BETA * t_air + est.PRIOR_OFFSET_C


def all_prox(cfg: MpcConfig, value: float) -> dict[str, float]:
    return {n: value for n in cfg.temps if cfg.sensors[n].role == "drive_proximal"}


# ---------------------------------------------------------------------------
# the example config itself
# ---------------------------------------------------------------------------


def test_example_das_config_layout(cfg):
    topo = cfg.topology
    assert cfg.is_das and cfg.regulates_drive_limits
    assert len(topo.zones) == 4 and len(topo.bays) == 15 and len(cfg.channels) == 8
    assert all(1 <= fan.count <= 2 for fan in cfg.fans.values())
    assert 8 <= sum(fan.count for fan in cfg.fans.values()) <= 10
    thermistors = [n for n, s in cfg.sensors.items() if s.quant_c <= 0.02]
    assert len(thermistors) == 8 and 24 <= len(cfg.temps) <= 30
    decided = {"hdd": (50.0, 5.0), "ssd_sata": (65.0, 10.0), "nvme": (70.0, 10.0)}
    assert {n: (c.limit_c, c.comfort_c) for n, c in cfg.drive_classes.items()} == decided
    assert all(bay.occupied == "auto" for bay in topo.bays.values())
    assert cfg.zones.trust_rule == "strict"


def test_example_das_config_binds_every_name_once(tmp_path):
    """--source hwmon's startup check (hw/sources.py) accepts the example bindings."""
    app = load_config(EXAMPLE_DAS_CONFIG)
    onewire = dict(app.section("onewire"), root=str(tmp_path))  # no buses: nothing starts
    composite, release = hw_sources.build_composite_from_config(
        hwmon_section=app.hwmon,
        xt6_section=app.section("xt6"),
        onewire_section=onewire,
        channels=app.mpc.channels,
        temps=app.mpc.temps,
        dt=app.mpc.dt,
    )
    try:
        assert len(composite.hwmon) == 2
        # The Quadro's temperature inputs are not assumed: only the aquaero binds thermistors.
        quadro = app.hwmon[1]
        assert quadro["temp_map"] == {}
    finally:
        if release is not None:
            release()


def test_example_das_config_runs_on_the_simulator():
    from aqua_bridge import __main__ as main_mod

    app = load_config(EXAMPLE_DAS_CONFIG)
    src, sink, release = main_mod.build_io(app, "sim", sim_plant="das")
    assert release is None and set(src.read().temps) == set(app.mpc.temps)


# ---------------------------------------------------------------------------
# first tick, determinism, serialisation
# ---------------------------------------------------------------------------


def test_cold_untrusted_first_tick_commands_fallback_pwm_exactly(cfg):
    obs = das_obs(cfg, 0.0, pwm={}, temps={n: None for n in cfg.temps})
    obs = dataclasses.replace(obs, rpm={})
    cmd, state = checked_step(obs, cfg, MpcState.cold())
    assert cmd.pwm == cfg.fallback_pwm
    assert cmd.mode is Mode.FALLBACK
    assert cmd.diagnostics["zones_in_fault"] == list(cfg.zone_layout.zones)
    assert state.fault_since_ts == 0.0


def test_cold_trusted_first_tick_is_bumpless_from_obs_pwm(cfg):
    obs = das_obs(cfg, 0.0, pwm=0.3, **all_prox(cfg, prox_for(47.0)))
    cmd, state = checked_step(obs, cfg, MpcState.cold())
    assert cmd.mode is Mode.AUTO
    assert cmd.diagnostics["prev_source"] == "obs_pwm"
    assert cmd.diagnostics["target_pwm"] == pytest.approx(dict.fromkeys(cfg.channels, 0.3))
    assert not state.in_fault and set(state.integrator) == set(cfg.channels)


def warm_state(cfg: MpcConfig, ticks: int = 5) -> MpcState:
    state = MpcState.cold()
    for i in range(ticks):
        _, state = checked_step(das_obs(cfg, i * cfg.dt, pwm=0.5), cfg, state)
    return state


def test_same_inputs_same_outputs_and_json_round_trip(cfg):
    state = warm_state(cfg)
    obs = das_obs(cfg, 100.0, pwm=0.5, prox_b07=prox_for(49.0))
    c1, s1 = step(obs, cfg, state)
    c2, s2 = step(obs, cfg, state)
    assert c1.to_dict() == c2.to_dict() and s1.to_dict() == s2.to_dict()
    restored = MpcState.from_dict(json.loads(json.dumps(state.to_dict(), allow_nan=False)))
    assert restored == state
    c3, s3 = step(obs, cfg, restored)
    assert c3.to_dict() == c1.to_dict() and s3.to_dict() == s1.to_dict()
    json.dumps(c1.to_dict(), allow_nan=False)
    assert_no_non_finite(c1.diagnostics, "diagnostics")


# ---------------------------------------------------------------------------
# faults never reduce cooling; saturation is honest; steady at the target
# ---------------------------------------------------------------------------


def test_zone_fault_never_lowers_its_reach_and_the_rest_regulates(cfg):
    state = MpcState.cold()
    cool = all_prox(cfg, prox_for(30.0))
    for i in range(4):
        _, state = checked_step(das_obs(cfg, i * cfg.dt, pwm=0.7, **cool), cfg, state)
    last = state.last_cmd
    reach = set(cfg.zone_layout.reach["z3"])
    for i in range(4, 40):
        obs = das_obs(cfg, i * cfg.dt, pwm=last.pwm, **{**cool, "air_z3": None})
        cmd, state = checked_step(obs, cfg, state)
        assert cmd.mode is Mode.DEGRADED
        for ch in cfg.channels:
            if ch in reach:
                assert cmd.pwm[ch] >= last.pwm[ch] - TOL
        last = cmd
    assert set(cmd.diagnostics["fallback_channels"]) == reach
    outside = [ch for ch in cfg.channels if ch not in reach]
    assert outside and all(cmd.pwm[ch] < 0.7 for ch in outside)  # cool drives: regulated down
    assert all(cmd.pwm[ch] >= cfg.fallback_pwm[ch] - TOL for ch in reach)  # ramped high


def test_saturation_pins_at_pwm_max_and_reports_it(cfg):
    state = MpcState.cold()
    modes = []
    for i in range(120):  # the slow integral (pi_ki) walks the demand past the rail
        # zone air flickers by two LSBs, as a live thermistor does (a frozen one is Stuck)
        air = {f"air_{z}": 35.0 + 0.02 * (i % 2) for z in cfg.zone_layout.zones}
        obs = das_obs(cfg, i * cfg.dt, pwm=0.5, **all_prox(cfg, prox_for(60.0)), **air)
        cmd, state = checked_step(obs, cfg, state)
        modes.append(cmd.mode)
        if cmd.mode is Mode.SATURATED:
            assert all(v == pytest.approx(cfg.pwm_max) for v in cmd.pwm.values())
    assert modes[0] is Mode.AUTO and modes[-1] is Mode.SATURATED
    assert all(v <= cfg.pwm_max + TOL for v in state.integrator.values())


def test_drives_at_their_target_hold_the_output(cfg):
    # soft 42 for hdd, e = t + 3 - 42 = 0 at t = 39
    target = {n: prox_for(39.0) for n in all_prox(cfg, 0.0)}
    state = MpcState.cold()
    first = None
    for i in range(40):
        cmd, state = checked_step(das_obs(cfg, i * cfg.dt, pwm=0.45, **target), cfg, state)
        assert cmd.mode is Mode.AUTO
        first = cmd.pwm if first is None else first
        assert cmd.pwm == pytest.approx(first, abs=1e-9)


# ---------------------------------------------------------------------------
# random per-zone lies
# ---------------------------------------------------------------------------


LIES = ["none", "nan", "spike", "drop", "extra", "freeze_zone_air", "bad_ts"]


@pytest.mark.fuzzy
@settings(max_examples=25, deadline=None)
@given(data=st.data())
def test_random_zone_lies_hold_invariants(data):
    base = load_config(EXAMPLE_DAS_CONFIG).mpc
    n = data.draw(st.integers(min_value=5, max_value=30))
    p_lie = data.draw(st.floats(min_value=0.0, max_value=0.6))
    drive = data.draw(st.floats(min_value=25.0, max_value=60.0))
    state = MpcState.cold()
    ts = 0.0
    for _ in range(n):
        drive += data.draw(st.floats(min_value=-0.5, max_value=0.5))
        pwm = 0.5 if state.last_cmd is None else state.last_cmd.pwm
        temps = {**all_prox(base, prox_for(drive))}
        obs = das_obs(base, ts, pwm=pwm, **temps)
        if data.draw(st.floats(min_value=0.0, max_value=1.0)) < p_lie:
            obs = _lie(data.draw, base, obs)
        cmd, state = checked_step(obs, base, state)
        ts = obs.ts + base.dt
        faulted = set(cmd.diagnostics["zones_in_fault"])
        if cmd.mode is Mode.FALLBACK:
            assert faulted == set(base.zone_layout.zones)


def _lie(draw, cfg: MpcConfig, obs: PlantObservation) -> PlantObservation:
    kind = draw(st.sampled_from(LIES))
    name = draw(st.sampled_from(list(cfg.temps)))
    temps = dict(obs.temps)
    if kind == "none":
        temps[name] = None
    elif kind == "nan":
        temps[name] = math.nan
    elif kind == "spike":
        temps[name] = draw(st.sampled_from([-40.0, 125.0, 32767.0]))
    elif kind == "drop":
        temps.pop(name)
    elif kind == "extra":
        temps["ghost"] = 40.0
    elif kind == "freeze_zone_air":
        zone = draw(st.sampled_from(list(cfg.zone_layout.zones)))
        temps[f"air_{zone}"] = None
    elif kind == "bad_ts":
        return dataclasses.replace(obs, ts=obs.ts - cfg.dt)
    return dataclasses.replace(obs, temps=temps)


# ---------------------------------------------------------------------------
# closed loops on the DAS truth plant
# ---------------------------------------------------------------------------


def noisy_topology(cfg: MpcConfig) -> dict:
    """The config's structure with each sensor type's realistic white noise."""
    topo = topology_from_config(cfg)
    for entry in topo["sensors"].values():
        entry["noise_sigma_c"] = SENSOR_TYPES[entry["type"]].noise_sigma_c
    return topo


def run_plant(
    cfg: MpcConfig, ticks: int, controller, inlet: dict | None = None, **plant_kw
) -> DasRun:
    topology = {**noisy_topology(cfg), "inlet": inlet or {}}
    plant = build_das_plant(topology, preset="basic", dt=cfg.dt, initial_pwm=0.5, **plant_kw)
    return run_das_closed_loop(plant, cfg, controller, ticks)


def _regulation(cfg: MpcConfig, controller=checked_step) -> DasRun:
    """40 min: activity bursts in three bays, an inlet step of +3 degC."""
    return run_plant(
        cfg,
        controller=controller,
        ticks=480,
        seed=20260913,
        heat_schedule={
            "b02": [(300.0, 1.0), (1500.0, 0.0)],
            "b10": [(600.0, 1.0)],
            "b13": [(0.0, 0.5), (1800.0, 1.0)],
        },
        inlet={"base_c": 25.0, "schedule": [[1200.0, 28.0]]},
    )


def _hotswap(cfg: MpcConfig, controller=checked_step) -> DasRun:
    """40 min: a drive pulled and a warm one inserted; a bay refitted with an SSD."""
    return run_plant(
        cfg,
        controller=controller,
        ticks=480,
        seed=7,
        heat_schedule={"b06": [(0.0, 1.0)], "b12": [(0.0, 0.3)]},
        bay_schedule=[
            {"t_s": 300.0, "bay": "b06", "action": "remove"},
            {"t_s": 900.0, "bay": "b06", "action": "insert", "class": "hdd", "temp_c": 45.0},
            {"t_s": 1200.0, "bay": "b12", "action": "remove"},
            {"t_s": 1500.0, "bay": "b12", "action": "insert", "class": "ssd_sata", "temp_c": 40.0},
        ],
    )


GOLDEN_SCENARIOS = {"das_regulation": _regulation, "das_hotswap": _hotswap}
_RUNS: dict[tuple[str, str], DasRun] = {}


def scenario(name: str, cfg: MpcConfig, case) -> DasRun:
    """One checked closed-loop run per scenario and solver case per session (they are
    deterministic, :func:`test_golden_scenarios_are_deterministic`)."""
    key = (name, case.value)
    if key not in _RUNS:
        _RUNS[key] = GOLDEN_SCENARIOS[name](cfg)
    return _RUNS[key]


def _rows(run: DasRun) -> list[dict]:
    return [
        {
            "t": r.obs.ts,
            "temps": {k: (None if v is None else float(v)) for k, v in r.obs.temps.items()},
            "pwm": {k: float(v) for k, v in r.cmd.pwm.items()},
            "mode": r.cmd.mode.value,
            "zones_in_fault": list(r.cmd.diagnostics["zones_in_fault"]),
        }
        for r in run.records
    ]


@pytest.mark.parametrize("name", sorted(GOLDEN_SCENARIOS))
def test_scenarios_keep_every_drive_within_its_limit(cfg, solver_kind, name):
    run = scenario(name, cfg, solver_kind)
    assert run.violations() == 0, f"{name}: a true drive temperature crossed its limit"
    modes = Counter(r.cmd.mode for r in run.records)
    assert modes[Mode.FALLBACK] == 0, f"{name}: modes {modes}"
    assert modes[Mode.DEGRADED] == 0, f"{name}: a zone faulted on a healthy plant: {modes}"
    for a, b in zip(run.records, run.records[1:], strict=False):
        for ch in cfg.channels:
            assert abs(b.cmd.pwm[ch] - a.cmd.pwm[ch]) <= cfg.d_pwm_max + TOL


def test_hot_swap_insert_raises_the_fans_of_its_zone(cfg, solver_kind):
    run = scenario("das_hotswap", cfg, solver_kind)
    ts = run.series["ts"]
    before = next(i for i, t in enumerate(ts) if t >= 880.0)
    after = next(i for i, t in enumerate(ts) if t >= 1000.0)
    for ch in cfg.topology.zones["z1"].channels:
        assert run.series["pwm_cmd"][ch][after] > run.series["pwm_cmd"][ch][before] + 0.05, ch


def test_burst_raises_fans(cfg, solver_kind):
    run = scenario("das_regulation", cfg, solver_kind)
    ts = run.series["ts"]
    idx = {t: next(i for i, s in enumerate(ts) if s >= t) for t in (290.0, 1400.0)}
    ch = "xt1"  # zone z0 holds b02
    assert run.series["pwm_cmd"][ch][idx[1400.0]] > run.series["pwm_cmd"][ch][idx[290.0]]


@pytest.mark.parametrize("name", sorted(GOLDEN_SCENARIOS))
def test_golden_trajectory(cfg, solver_kind, name):
    rows = _rows(scenario(name, cfg, solver_kind))
    path = GOLDEN_DIR / f"{name}.{solver_kind.value}.json"
    if os.environ.get(REGEN_ENV) == "1":
        GOLDEN_DIR.mkdir(parents=True, exist_ok=True)
        lines = ",\n".join(json.dumps(r, separators=(",", ":")) for r in rows)
        path.write_text(f'{{"scenario":"{name}","rows":[\n{lines}\n]}}\n')
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
            if v is None:
                assert r["temps"][tname] is None, f"tick {i}: temps[{tname}]"
            else:
                assert r["temps"][tname] == pytest.approx(v, abs=GOLDEN_TEMP_ATOL), (
                    f"tick {i}: temps[{tname}]"
                )
        assert r["mode"] == g["mode"], f"tick {i}: mode {r['mode']} != {g['mode']}"
        assert r["zones_in_fault"] == g["zones_in_fault"], f"tick {i}: zones_in_fault"


def test_golden_scenarios_are_deterministic(cfg, solver_kind):
    """A fresh run (plain ``step``, same seed) reproduces the checked run exactly."""
    for name, make in GOLDEN_SCENARIOS.items():
        fresh = _rows(make(cfg, controller=step))
        assert fresh == _rows(scenario(name, cfg, solver_kind)), f"{name}: two runs differ"
