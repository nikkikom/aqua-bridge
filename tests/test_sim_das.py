"""DAS truth plant (``aqua_bridge.sim.das``): physics sanity, sensing, hot swap, determinism.

Plan section 3 "Simulator". These tests prove the truth plant is physically
consistent (energy balance, steady state, monotone cooling), that its
sensing models what the controller will see (quantisation, lag, SMART
cadence, tach-less outputs, splitters), that hot swap and schedules behave,
and that a run is bit-reproducible per seed.
"""

from __future__ import annotations

import copy
import dataclasses
import json
import math

import numpy as np
import pytest

from aqua_bridge import __main__ as main_mod
from aqua_bridge.config import AppConfig, ConfigError, load_config
from aqua_bridge.model import Mode, MpcCommand, MpcConfig, MpcState, PlantObservation
from aqua_bridge.sim import das
from aqua_bridge.sim.das import (
    DasPlant,
    DriveSpec,
    build_das_params,
    build_das_plant,
    default_topology,
    quantise,
    run_das_closed_loop,
)
from aqua_bridge.sim.plant import Plant
from invariants import TOL, assert_no_non_finite, checked_step


def uniform(plant: DasPlant, pwm: float) -> dict[str, float]:
    return dict.fromkeys(plant.channels, pwm)


def run_open(plant: DasPlant, ticks: int, pwm: float | dict[str, float]) -> None:
    cmd = pwm if isinstance(pwm, dict) else uniform(plant, pwm)
    for _ in range(ticks):
        plant.apply(cmd)
        plant.advance()


def simple_topology() -> dict:
    """One zone, one HDD, one fan, one sensor per role: easy to reason about."""
    return {
        "zones": {"z": {}},
        "bays": {"b": {"zone": "z", "class": "hdd", "serial": "S1"}},
        "fans": {"f": {"zone": "z"}},
        "sensors": {
            "inlet": {"role": "inlet", "type": "thermistor"},
            "air": {"role": "zone_air", "zone": "z", "type": "thermistor"},
            "prox": {"role": "drive_proximal", "zone": "z", "bay": "b", "type": "ds18b20"},
        },
    }


# ---------------------------------------------------------------------------
# builder and validation
# ---------------------------------------------------------------------------


def test_default_topology_shape():
    p = build_das_params(default_topology())
    assert len(p.zones) == 4 and len(p.bays) == 15 and len(p.fans) == 8
    assert 24 <= len(p.sensors) <= 30
    assert all(1 <= f.count <= 2 for f in p.fans)
    assert sum(s.type == "thermistor" for s in p.sensors) == 8
    assert any(not f.tach for f in p.fans)
    # the builder does not mutate its input
    top = default_topology()
    frozen = copy.deepcopy(top)
    build_das_params(top, preset="rich", seed=3)
    assert top == frozen


@pytest.mark.parametrize(
    "mutate, match",
    [
        (lambda t: t.update(bogus=1), "unknown keys"),
        (lambda t: t["zones"].clear(), "zones"),
        (lambda t: t["bays"]["b"].update(zone="nope"), "unknown zone"),
        (lambda t: t["bays"]["b"].update(**{"class": "tape"}), "unknown class"),
        (lambda t: t["bays"]["b"].update(occupied="auto"), "occupied"),
        (lambda t: t["fans"]["f"].pop("zone"), "zone or zones"),
        (lambda t: t["fans"]["f"].update(count=0), "count"),
        (lambda t: t["fans"]["f"].update(deadband=1.0), "deadband"),
        (lambda t: t["sensors"]["prox"].pop("bay"), "bay"),
        (lambda t: t["sensors"]["air"].pop("zone"), "needs a zone"),
        (lambda t: t["sensors"]["air"].update(role="ceiling"), "role"),
        (lambda t: t["sensors"]["air"].update(type="pt100"), "type"),
        (lambda t: t["sensors"]["air"].update(bay="b"), "only drive_proximal"),
        (lambda t: t["sensors"]["prox"].update(tau_s=float("nan")), "finite"),
        (lambda t: t["zones"]["z"].update(coupled_to=["z"]), "unknown zone"),
        (lambda t: t.update(bay_schedule=[{"t_s": 1, "bay": "x", "action": "remove"}]), "bay"),
        (lambda t: t.update(bay_schedule=[{"t_s": 1, "bay": "b", "action": "eject"}]), "action"),
        (lambda t: t.update(heat_schedule={"x": [[0, 1]]}), "unknown bay"),
    ],
)
def test_topology_validation(mutate, match):
    top = simple_topology()
    mutate(top)
    with pytest.raises(ValueError, match=match):
        build_das_params(top)


def test_bay_must_belong_to_sensor_zone():
    top = simple_topology()
    top["zones"]["z2"] = {}
    top["sensors"]["prox"]["zone"] = "z2"
    with pytest.raises(ValueError, match="not in zone"):
        build_das_params(top)


def test_preset_and_explicit_values():
    with pytest.raises(ValueError, match="preset"):
        build_das_params(simple_topology(), preset="fancy")
    basic = build_das_params(simple_topology())
    assert basic.delay_ticks == 0 and basic.burst_prob == 0
    assert all(s.noise_sigma_c == 0 and s.offset_c == 0 for s in basic.sensors)
    rich = build_das_params(default_topology(), preset="rich", seed=5)
    assert rich.delay_ticks == 1 and rich.burst_prob > 0
    assert len({s.beta for s in rich.sensors}) > 1
    assert rich.inlet.swing_c > 0
    # undeclared inter-zone leakage exists in the truth (z0 and z3 are not declared)
    assert rich.zones[0].kappa.get("z3", 0) >= 0 and "z3" in rich.zones[0].kappa
    # explicit topology values win over the preset
    top = default_topology()
    top["sensors"]["prox_b01"]["beta"] = 0.42
    top["fans"]["xt1"]["deadband"] = 0.25
    top["fans"]["qd4"]["tach"] = True
    top["bays"]["b01"]["g0_w_per_k"] = 0.9
    rich2 = build_das_params(top, preset="rich", seed=5)
    s = {x.name: x for x in rich2.sensors}
    f = {x.name: x for x in rich2.fans}
    b = {x.name: x for x in rich2.bays}
    assert s["prox_b01"].beta == 0.42
    assert f["xt1"].deadband == 0.25 and f["qd4"].tach
    assert b["b01"].drive.g0_w_per_k == 0.9
    # symmetric kappa
    for z in rich2.zones:
        for other, k in z.kappa.items():
            assert next(o for o in rich2.zones if o.name == other).kappa[z.name] == k


def test_rich_draws_do_not_depend_on_dict_order():
    top = default_topology()
    reordered = dict(top)
    reordered["sensors"] = dict(reversed(list(top["sensors"].items())))
    a = build_das_params(top, preset="rich", seed=11)
    b = build_das_params(reordered, preset="rich", seed=11)
    sa = {s.name: s for s in a.sensors}
    sb = {s.name: s for s in b.sensors}
    assert sa == sb


# ---------------------------------------------------------------------------
# physics: energy balance, steady state, monotone cooling
# ---------------------------------------------------------------------------


@pytest.mark.parametrize("preset", ["basic", "rich"])
def test_energy_balance_through_transients_and_hot_swap(preset):
    plant = build_das_plant(
        preset=preset,
        seed=2,
        dt=5.0,
        heat_schedule={"b05": [[100, 1.0], [700, 0.0]]},
        bay_schedule=[
            {"t_s": 300, "bay": "b02", "action": "remove"},
            {"t_s": 600, "bay": "b02", "action": "insert", "class": "nvme", "serial": "NV1"},
        ],
    )
    rng = np.random.default_rng(0)
    throughput = 0.0
    for _ in range(240):
        plant.apply({ch: float(rng.uniform(0.0, 1.0)) for ch in plant.channels})
        plant.advance()
        e = plant.energy
        throughput = max(throughput, e["in_j"])
        lhs = plant.stored_energy() - e["stored0_j"]
        rhs = e["in_j"] - e["out_j"] + e["swap_j"]
        assert lhs == pytest.approx(rhs, abs=1e-9 * max(1.0, throughput, abs(e["stored0_j"])))
    assert plant.energy["swap_j"] != 0.0


@pytest.mark.parametrize("preset", ["basic", "rich"])
def test_steady_state_is_reached_and_balances_heat(preset):
    plant = build_das_plant(preset=preset, seed=4, dt=5.0, burst_prob=0.0)
    if preset == "rich":  # freeze time-varying inputs so a steady state exists
        plant.params = dataclasses.replace(
            plant.params,
            inlet=dataclasses.replace(plant.params.inlet, drift_c_per_h=0.0, swing_c=0.0),
            fans=tuple(dataclasses.replace(f, fouling_per_day=0.0) for f in plant.params.fans),
        )
    ss = plant.steady_state(0.6)
    run_open(plant, 36 * 60, 0.6)  # three hours: several slow-HDD time constants
    for z, t in plant.t_air().items():
        assert t == pytest.approx(ss["t_air"][z], abs=0.02)
    for b, t in plant.t_drive().items():
        assert t == pytest.approx(ss["t_drive"][b], abs=0.05)
    for s, t in plant.sensor_truth().items():
        assert t == pytest.approx(ss["sensors"][s], abs=0.05)
    # heat in == heat carried out by air at the equilibrium
    heat = sum(plant.heat_w().values()) + sum(z.heat_w for z in plant.params.zones)
    q = plant.airflow()
    tin = plant.t_inlet()
    out = sum(
        (q[z.name] + z.leak_w_per_k) * (ss["t_air"][z.name] - tin[z.name])
        for z in plant.params.zones
    )
    assert out == pytest.approx(heat, rel=1e-9)
    # ordering: inlet < air < proximal sensor < drive (HDD heat flows outward)
    params = plant.params
    for bay in params.bays:
        t_d = ss["t_drive"][bay.name]
        t_a = ss["t_air"][bay.zone]
        assert t_d > t_a > tin[bay.zone]
        for s in params.sensors:
            if s.bay == bay.name:
                assert t_a + s.offset_c <= ss["sensors"][s.name] + 1e-9
                assert ss["sensors"][s.name] <= t_d + s.offset_c + 1e-9


@pytest.mark.parametrize(("preset", "seed"), [("basic", 0), ("rich", 1), ("rich", 2)])
def test_more_airflow_never_warms_anything(preset, seed):
    plant = build_das_plant(preset=preset, seed=seed)
    grid = np.linspace(0.0, 1.0, 21)
    prev = None
    for u in grid:
        ss = plant.steady_state(float(u))
        cur = [*ss["t_air"].values(), *ss["t_drive"].values()]
        if prev is not None:
            assert all(c <= p + 1e-9 for c, p in zip(cur, prev, strict=True))
        prev = cur
    # strictly cooler at full speed than at the dead band, every drive
    lo, hi = plant.steady_state(0.0), plant.steady_state(1.0)
    for b in plant.params.bay_names:
        assert hi["t_drive"][b] < lo["t_drive"][b] - 1.0
    # per channel: raising one output from 0.3 to 0.9 never warms a node
    base = uniform(plant, 0.3)
    ref = plant.steady_state(base)
    for ch in plant.channels:
        up = plant.steady_state({**base, ch: 0.9})
        for key in ("t_air", "t_drive"):
            for name, t in up[key].items():
                assert t <= ref[key][name] + 1e-9


def test_dead_band_and_exponent():
    top = simple_topology()
    top["fans"]["f"].update(deadband=0.2, exponent=1.0, rpm_max=1000)
    plant = build_das_plant(top)
    for u in (0.0, 0.1, 0.2):
        plant.apply({"f": u})
        plant.advance()
        assert plant.observe().rpm["f"] == 0.0
        assert plant.airflow()["z"] == 0.0
    plant.apply({"f": 0.6})
    plant.advance()
    assert plant.observe().rpm["f"] == pytest.approx(500.0)
    assert plant.airflow()["z"] == pytest.approx(33.0 * 0.5)
    top["fans"]["f"]["exponent"] = 1.5
    p2 = build_das_plant(top)
    p2.apply({"f": 0.6})
    p2.advance()
    assert p2.airflow()["z"] == pytest.approx(33.0 * 0.5**1.5)


# ---------------------------------------------------------------------------
# sensing: quantisation, lag, SMART, tach, splitters, noise
# ---------------------------------------------------------------------------


def test_quantise_function():
    assert quantise(25.03, 0.0625) == 25.0
    assert quantise(25.032, 0.0625) == 25.0625
    assert quantise(25.004, 0.01) == 25.0
    assert quantise(25.006, 0.01) == 25.01
    assert quantise(36.5, 1.0) == 37.0
    assert quantise(1.2345, 0.0) == 1.2345


def _is_multiple(v: float, q: float) -> bool:
    return abs(v / q - round(v / q)) < 1e-6


def test_readings_are_quantised_per_sensor_type():
    plant = build_das_plant(preset="rich", seed=7, dt=5.0)
    types = {s.name: s.type for s in plant.params.sensors}
    for _ in range(40):
        run_open(plant, 1, 0.5)
        for name, v in plant.observe().temps.items():
            assert v is not None
            q = das.SENSOR_TYPES[types[name]].quant_c
            assert _is_multiple(v, q), (name, v)
    smart = plant.observe_smart()
    assert smart
    for entry in smart.values():
        assert float(entry["temp_c"]).is_integer()
        assert entry["age_s"] >= 0


def test_ds18b20_plateaus_while_a_thermistor_moves():
    """A slow ramp: the DS18B20 sits on one code while the thermistor resolves the change."""
    top = simple_topology()
    top["inlet"] = {"drift_c_per_h": 1.5}
    top["sensors"]["inlet_ds"] = {"role": "inlet", "type": "ds18b20", "tau_s": 1.0}
    top["sensors"]["inlet"]["tau_s"] = 1.0
    plant = build_das_plant(top, dt=5.0)
    ds, th = [], []
    for _ in range(12):  # one minute: 0.025 degC of drift, less than half an LSB
        run_open(plant, 1, 0.5)
        o = plant.observe().temps
        ds.append(o["inlet_ds"])
        th.append(o["inlet"])
    assert len(set(ds)) == 1
    assert len(set(th)) > 1


def test_sensor_lag_is_first_order():
    top = simple_topology()
    top["inlet"] = {"base_c": 25.0, "schedule": [[100.5, 30.0]]}
    top["sensors"]["inlet"].update(tau_s=20.0, quant_c=0.0)
    plant = build_das_plant(top, dt=1.0)
    run_open(plant, 100, 0.5)
    t0 = plant.ts
    assert plant.sensor_truth()["inlet"] == pytest.approx(25.0, abs=1e-9)
    samples = []
    for _ in range(80):
        run_open(plant, 1, 0.5)
        samples.append((plant.ts - t0, plant.sensor_truth()["inlet"]))
    for t, v in samples:
        expected = 25.0 + 5.0 * (1.0 - math.exp(-t / 20.0))
        assert v == pytest.approx(expected, abs=0.12)
    # after one time constant ~63 %, after four ~98 %
    by_t = dict(samples)
    assert 0.58 < (by_t[20.0] - 25.0) / 5.0 < 0.68
    assert (by_t[80.0] - 25.0) / 5.0 > 0.97
    # observation is the quantised sensor node, never the ambient itself
    assert plant.observe().temps["inlet"] == pytest.approx(by_t[80.0], abs=0.01)


def test_proximal_sensor_lags_drive_and_mixes_air():
    top = simple_topology()
    top["sensors"]["prox"].update(beta=0.4, offset_c=0.5, quant_c=0.0)
    plant = build_das_plant(top, dt=2.0)
    t_d = plant.t_drive()["b"]
    t_a = plant.t_air()["z"]
    assert plant.sensor_truth()["prox"] == pytest.approx(0.6 * t_d + 0.4 * t_a + 0.5, abs=1e-9)
    # a fan step cools the air first; the sensor follows the drive slowly
    run_open(plant, 5, 1.0)
    moved_air = plant.t_air()["z"] - t_a
    moved_prox = plant.sensor_truth()["prox"] - (0.6 * t_d + 0.4 * t_a + 0.5)
    assert moved_air < 0 and moved_prox < 0
    assert abs(moved_prox) < abs(moved_air)


def test_smart_offset_cadence_and_lag():
    top = simple_topology()
    top["bays"]["b"].update(smart_offset_c=2.0, smart_tau_s=0.0, model="HDD-X")
    plant = build_das_plant(top, dt=5.0)
    seen_ts = []
    for _ in range(240):  # 20 minutes
        run_open(plant, 1, 0.5)
        s = plant.observe_smart()
        if "S1" in s and s["S1"]["age_s"] == 0.0:
            seen_ts.append(plant.ts)
            assert s["S1"]["temp_c"] == quantise(plant.t_drive()["b"] + 2.0, 1.0)
            assert s["S1"]["model"] == "HDD-X"
    gaps = np.diff(seen_ts)
    assert len(seen_ts) >= 18
    assert gaps.min() >= 30.0 - 5.0 and gaps.max() <= 60.0 + 5.0
    # a drive without serial or with smart off never reports
    top["bays"]["b"].update(smart=False)
    p2 = build_das_plant(top, dt=5.0)
    run_open(p2, 60, 0.5)
    assert p2.observe_smart() == {}


def test_tachless_output_and_splitter():
    plant = build_das_plant(default_topology(), dt=2.0)
    obs = plant.observe()
    assert "qd4" not in obs.rpm  # no tach wire: absent, like the hardware adapter
    assert set(obs.pwm) == set(plant.channels)
    run_open(plant, 3, 0.8)
    q0 = plant.airflow()["z0"]
    rpm0 = plant.observe().rpm["xt1"]
    plant.stall("xt1", fan=1)  # the second fan on the splitter: invisible to the tach
    run_open(plant, 1, 0.8)
    assert plant.observe().rpm["xt1"] == rpm0
    assert plant.airflow()["z0"] < q0
    plant.stall("xt1", fan=0)
    run_open(plant, 1, 0.8)
    assert plant.observe().rpm["xt1"] == 0.0
    plant.unstall("xt1")
    run_open(plant, 1, 0.8)
    assert plant.airflow()["z0"] == pytest.approx(q0)


def test_observe_is_idempotent_and_dropout_reads_none():
    top = simple_topology()
    top["sensors"]["air"]["dropout_prob"] = 1.0
    top["sensors"]["prox"]["noise_sigma_c"] = 0.5
    plant = build_das_plant(top)
    run_open(plant, 3, 0.5)
    a, b = plant.observe(), plant.observe()
    assert a == b
    assert a.temps["air"] is None and a.temps["prox"] is not None


def test_actuator_delay_and_fan_lag():
    top = simple_topology()
    top["fans"]["f"].update(tau_s=4.0, deadband=0.0)
    plant = build_das_plant(top, dt=2.0, delay_ticks=1, initial_pwm=0.0)
    plant.apply({"f": 1.0})
    plant.advance()
    assert plant.effective_pwm()["f"] == 0.0 and plant.observe().rpm["f"] == 0.0
    plant.apply({"f": 1.0})
    plant.advance()
    rpm = plant.observe().rpm["f"]
    assert 0 < rpm < 1500.0
    assert rpm == pytest.approx(1500.0 * (1 - math.exp(-0.5)), abs=1.0)


def test_fouling_and_inlet_drift():
    top = simple_topology()
    top["fans"]["f"].update(fouling_per_day=0.1, fouling_floor=0.6)
    top["inlet"] = {"base_c": 22.0, "drift_c_per_h": 1.0, "swing_c": 0.0}
    plant = build_das_plant(top, dt=5.0)
    f = plant.params.fans[0]
    assert plant.fouling(f, 0.0) == 1.0
    assert plant.fouling(f, 86400.0) == pytest.approx(0.9)
    assert plant.fouling(f, 86400.0 * 10) == pytest.approx(0.6)
    assert plant.ambient(3600.0) == pytest.approx(23.0)
    plant.ts = 86400.0 * 5
    run_open(plant, 2, 1.0)
    assert plant.airflow()["z"] == pytest.approx(33.0 * 0.6)


def test_activity_heat_and_bursts_are_deterministic():
    plant = build_das_plant(default_topology(), heat_schedule={"b01": [[60, 1.0], [120, 0.25]]})
    hdd = das.DRIVE_CLASSES["hdd"]
    assert plant.heat_w(0.0)["b01"] == hdd.heat_idle_w
    assert plant.heat_w(60.0)["b01"] == hdd.heat_active_w
    assert plant.heat_w(200.0)["b01"] == pytest.approx(
        hdd.heat_idle_w + 0.25 * (hdd.heat_active_w - hdd.heat_idle_w)
    )
    p1 = build_das_plant(preset="rich", seed=9)
    p2 = build_das_plant(preset="rich", seed=9)
    times = [float(t) for t in range(0, 36000, 150)]
    seq1 = [p1.activity("b03", t) for t in times]
    seq2 = [p2.activity("b03", t) for t in reversed(times)][::-1]
    assert seq1 == seq2
    assert 0 < sum(a == 1.0 for a in seq1) < len(seq1)


# ---------------------------------------------------------------------------
# hot swap
# ---------------------------------------------------------------------------


def test_hot_swap_remove_and_insert_with_class_and_serial():
    top = simple_topology()
    top["zones"]["z"]["heat_w"] = 0.0
    plant = build_das_plant(
        top,
        dt=5.0,
        bay_schedule=[
            {"t_s": 600, "bay": "b", "action": "remove"},
            {
                "t_s": 2400,
                "bay": "b",
                "action": "insert",
                "class": "ssd_sata",
                "serial": "S2",
                "model": "SSD-Y",
                "temp_c": 40.0,
            },
        ],
    )
    run_open(plant, 110, 0.5)  # ts = 550
    assert plant.occupied()["b"]
    hot_prox = plant.sensor_truth()["prox"]
    assert "S1" in plant.observe_smart()
    run_open(plant, 11, 0.5)  # the event fires at the tick boundary at 600
    assert not plant.occupied()["b"]
    assert plant.t_drive()["b"] is None and plant.margins()["b"] is None
    assert plant.heat_w()["b"] == 0.0
    run_open(plant, 300, 0.5)  # 25 minutes empty
    # nothing heats the zone: the proximal sensor now reads zone air
    assert plant.sensor_truth()["prox"] == pytest.approx(plant.t_air()["z"], abs=0.01)
    assert plant.t_air()["z"] == pytest.approx(plant.ambient(), abs=0.01)
    assert plant.sensor_truth()["prox"] < hot_prox - 5.0
    s1 = plant.observe_smart()["S1"]
    assert s1["age_s"] > 1000  # retained, stale, no longer refreshed
    run_open(plant, 60, 0.5)  # ts = 2405: inserted at the 2400 boundary
    assert plant.occupied()["b"]
    d = plant.drives[0]
    assert d.drive_class == "ssd_sata" and d.serial == "S2" and d.limit_c == 65.0
    assert plant.heat_w()["b"] == das.DRIVE_CLASSES["ssd_sata"].heat_idle_w
    run_open(plant, 14, 0.5)  # > 60 s: SMART of the new serial reports
    smart = plant.observe_smart()
    assert smart["S2"]["model"] == "SSD-Y" and smart["S2"]["age_s"] <= 60.0
    assert "bay" not in smart["S2"]  # no SES: SMART never says where a serial sits
    e = plant.energy
    assert plant.stored_energy() - e["stored0_j"] == pytest.approx(
        e["in_j"] - e["out_j"] + e["swap_j"], abs=1e-6
    )


def test_insert_raises_sensor_and_direct_api():
    plant = build_das_plant(simple_topology(), dt=5.0)
    plant.remove("b")
    run_open(plant, 360, 0.5)
    cold = plant.sensor_truth()["prox"]
    plant.insert("b", DriveSpec.nominal("hdd", serial="S9"))
    run_open(plant, 360, 0.5)
    assert plant.sensor_truth()["prox"] > cold + 3.0
    plant.remove("b")
    plant.remove("b")  # idempotent


# ---------------------------------------------------------------------------
# closed loop and determinism
# ---------------------------------------------------------------------------


def das_mpc_config(plant: DasPlant, **changes) -> MpcConfig:
    """A legacy PI config speaking the DAS plant's names (until the zoned config exists)."""
    data = load_config_mpc_dict()
    names = plant.params.sensor_names
    data.update(
        temps=list(names),
        channels=list(plant.channels),
        setpoints={"air_z0": 27.0, "air_z1": 27.0, "air_z2": 27.0, "air_z3": 27.0},
        channel_temps={ch: ["air_z0", "air_z1", "air_z2", "air_z3"] for ch in plant.channels},
        weights={},
        fallback_pwm=dict.fromkeys(plant.channels, 0.8),
        stuck_s=3600,
        dt=plant.params.dt,
        confirm_s=4 * plant.params.dt,
        fallback_hold_s=10 * plant.params.dt,
    )
    data.update(changes)
    return MpcConfig.from_mapping(data)


def load_config_mpc_dict() -> dict:
    import yaml

    from conftest import EXAMPLE_CONFIG

    return dict(yaml.safe_load(EXAMPLE_CONFIG.read_text())["mpc"])


def _run(seed: int, ticks: int = 120, hook=None, smart_hook=None):
    plant = build_das_plant(
        preset="rich",
        seed=seed,
        dt=5.0,
        bay_schedule=[
            {"t_s": 200, "bay": "b07", "action": "remove"},
            {"t_s": 400, "bay": "b07", "action": "insert", "class": "ssd_sata", "serial": "N7"},
        ],
    )
    cfg = das_mpc_config(plant)
    return (
        plant,
        cfg,
        run_das_closed_loop(
            plant, cfg, checked_step, ticks, observe_hook=hook, smart_hook=smart_hook
        ),
    )


def test_closed_loop_series_and_invariants():
    plant, cfg, run = _run(3)
    s = run.series
    assert len(run.records) == 120 and len(s["ts"]) == 120
    assert s["ts"][1] - s["ts"][0] == 5.0
    for rec in run.records:
        for ch in cfg.channels:
            assert cfg.pwm_min - TOL <= rec.cmd.pwm[ch] <= cfg.pwm_max + TOL
        assert rec.cmd.mode in set(Mode)
    assert_no_non_finite({k: v for k, v in s.items() if k not in ("t_drive", "margin_c", "obs")})
    occ = s["occupied"]["b07"]
    assert occ[0] and not all(occ) and occ[-1]
    assert any(v is None for v in s["t_drive"]["b07"])
    assert run.array("t_drive", "b07").shape == (120,)
    assert math.isfinite(run.worst_margin_c()) and run.violations() == 0
    assert s["pwm_cmd"]["xt1"][0] == run.records[0].cmd.pwm["xt1"]
    json.dumps(s, allow_nan=False)  # series are plain JSON


def test_closed_loop_is_deterministic_per_seed():
    _, _, a = _run(5)
    _, _, b = _run(5)
    _, _, c = _run(6)
    dump = lambda r: json.dumps(r.series, sort_keys=True)  # noqa: E731
    assert dump(a) == dump(b)
    assert [r.state.to_dict() for r in a.records] == [r.state.to_dict() for r in b.records]
    assert dump(a) != dump(c)


def test_observe_hook_injects_lies_and_smart_hook_edits_smart():
    seen: list[PlantObservation] = []

    def lie(i: int, obs: PlantObservation) -> PlantObservation:
        temps = dict(obs.temps)
        if i >= 10:
            temps["air_z0"] = None
        out = PlantObservation(temps=temps, rpm=obs.rpm, pwm=obs.pwm, ts=obs.ts)
        seen.append(out)
        return out

    def no_smart(i, smart):
        return {}

    _, cfg, run = _run(1, ticks=30, hook=lie, smart_hook=no_smart)
    assert all(r.obs is o for r, o in zip(run.records, seen, strict=True))
    assert run.series["obs"]["air_z0"][10:] == [None] * 20
    assert run.series["sensor_true"]["air_z0"][15] is not None
    assert all(sm == {} for sm in run.series["smart"])
    # an untrusted zone-air sensor puts the legacy controller into fallback
    assert run.records[-1].cmd.mode is Mode.FALLBACK


def test_legacy_plant_untouched_by_das_import():
    p = Plant()
    assert p.params.delay_ticks == 0 and p.params.noise_sigma_c == 0.0


# ---------------------------------------------------------------------------
# __main__ --sim-plant
# ---------------------------------------------------------------------------


def test_build_io_basic_is_the_legacy_plant(example_config_path):
    app = load_config(example_config_path)
    src, _, _ = main_mod.build_io(app, "sim")
    ref = main_mod.make_sim_plant(app.mpc)
    assert type(src.plant) is Plant and src.plant.params == ref.params
    src2, _, _ = main_mod.build_io(app, "sim", sim_plant="basic")
    assert src2.plant.params == ref.params
    rich, _, _ = main_mod.build_io(app, "sim", sim_plant="rich")
    assert rich.plant.params == dataclasses.replace(ref.params, delay_ticks=1, noise_sigma_c=0.02)


def test_build_io_das_without_topology_is_a_config_error(example_config_path):
    app = load_config(example_config_path)
    with pytest.raises(ConfigError, match="needs a DAS topology"):
        main_mod.build_io(app, "sim", sim_plant="das")


def _das_app(**sim) -> AppConfig:
    plant = build_das_plant()
    cfg = das_mpc_config(plant)
    return dataclasses.replace(
        AppConfig(mpc=cfg), extra={"sim": {"das": sim}} if sim is not None else {}
    )


def test_build_io_das_with_sim_section():
    app = _das_app(preset="rich", seed=2)
    src, sink, release = main_mod.build_io(app, "sim", sim_plant="das")
    assert src is sink and release is None
    assert isinstance(src.plant, DasPlant) and src.plant.params.seed == 2
    obs = src.read()
    assert set(obs.temps) == set(app.mpc.temps)
    ts0 = obs.ts
    src.apply(MpcCommand(pwm=dict(app.mpc.fallback_pwm), mode=Mode.AUTO))
    assert src.read().ts == pytest.approx(ts0 + app.mpc.dt)


@pytest.mark.parametrize(
    "sim, match",
    [
        ({"bogus": 1}, "unknown keys"),
        ({"seed": "x"}, "seed"),
        ({"preset": "fancy"}, "preset"),
        ({"topology": {"zones": {}}}, "zones"),
        ({"topology": simple_topology()}, "does not provide"),
    ],
)
def test_build_io_das_rejects_bad_sections(sim, match):
    with pytest.raises(ConfigError, match=match):
        main_mod.build_io(_das_app(**sim), "sim", sim_plant="das")


def test_main_sim_plant_flag(tmp_path, example_config_path, monkeypatch):
    import signal

    import yaml

    old = signal.getsignal(signal.SIGTERM), signal.getsignal(signal.SIGINT)
    try:
        args = main_mod.build_parser().parse_args(["--config", "x"])
        assert args.sim_plant is None
        rc = main_mod.main(
            ["--config", str(example_config_path), "--source", "sim", "--sim-plant", "das"]
            + ["--ticks", "1", "--sim-speed", "0"]
        )
        assert rc == 2
        with pytest.raises(SystemExit):
            main_mod.main(["--config", str(example_config_path), "--sim-plant", "rich"])
        plant = build_das_plant()
        data = yaml.safe_load(example_config_path.read_text())
        data["mpc"] = das_mpc_config(plant).to_dict()
        data["sim"] = {"das": {"preset": "basic"}}
        conf = tmp_path / "das.yaml"
        conf.write_text(yaml.safe_dump(data))
        for flavour, conf_path in (("das", conf), ("rich", example_config_path)):
            rc = main_mod.main(
                ["--config", str(conf_path), "--source", "sim", "--sim-plant", flavour]
                + ["--ticks", "3", "--sim-speed", "0"]
            )
            assert rc == 0, flavour
    finally:
        signal.signal(signal.SIGTERM, old[0])
        signal.signal(signal.SIGINT, old[1])


def test_state_roundtrip_of_a_das_run_is_json():
    _, _, run = _run(8, ticks=20)
    st = run.records[-1].state
    assert MpcState.from_dict(json.loads(json.dumps(st.to_dict()))) == st
