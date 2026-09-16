"""Zoned DAS contract: config sections, zone trust, closure, modes, compose, publishers.

Plan section 0.1 (per-zone trust and fallback), section 7 (config rows for
``topology`` / ``sensors`` / ``drive_classes`` / ``fans`` / ``fan_models`` /
``zones``, ``MpcState.zone_faults``, ``Mode.degraded``,
``PlantObservation.inputs``). The safety invariants of per-zone fallback
live in ``test_mpc_zone_fallback.py``; per-role Stuck sizing in
``test_gate.py``. Legacy mode is covered by every pre-existing suite and
the goldens, which run unchanged on configs without ``topology``.
"""

from __future__ import annotations

import dataclasses
import json
import math
from collections.abc import Callable
from pathlib import Path
from typing import Any

import pytest
import yaml

from aqua_bridge.config import load_config
from aqua_bridge.control import zones
from aqua_bridge.control.gate import evaluate_gate
from aqua_bridge.control.intents import (
    ControlMode,
    ControlSnapshot,
    SetMode,
    SetPwm,
    SolverStatus,
)
from aqua_bridge.control.mpc import step
from aqua_bridge.control.supervisor import Supervisor
from aqua_bridge.model import (
    BUILTIN_DRIVE_CLASSES,
    DAS_SECTIONS,
    IMPLICIT_ZONE,
    ConfigError,
    DriveClass,
    FaultReason,
    Mode,
    MpcCommand,
    MpcConfig,
    MpcState,
    PlantObservation,
    ZoneFault,
    ZonePolicy,
)
from aqua_bridge.publishers.mqtt_ha import state_payload
from das_fixtures import CHANNELS, SP, das_cfg, das_mapping, das_obs
from invariants import checked_step, make_obs

REPO_ROOT = Path(__file__).resolve().parent.parent


@pytest.fixture
def dcfg() -> MpcConfig:
    return das_cfg()


# ---------------------------------------------------------------------------
# config: parsing, defaults, round trip
# ---------------------------------------------------------------------------


def test_das_config_loads_and_derives_layout(dcfg):
    assert dcfg.is_das
    layout = dcfg.zone_layout
    assert not layout.implicit
    assert layout.zones == ("za", "zb", "zc")
    assert layout.zone_channels == {"za": ("fa1", "fa2"), "zb": ("fb1",), "zc": ("fc1",)}
    assert layout.coupled == {"za": ("zb",), "zb": ("za",), "zc": ()}
    assert layout.channel_zones == {
        "fa1": ("za",),
        "fa2": ("za",),
        "fb1": ("zb",),
        "fc1": ("zc",),
    }
    assert layout.sensor_zone["inlet"] is None and layout.sensor_zone["exhaust"] == "zb"
    assert layout.reach == {
        "za": ("fa1", "fa2", "fb1"),
        "zb": ("fa1", "fa2", "fb1"),
        "zc": ("fc1",),
    }
    groups = {z: dict(g) for z, g in layout.required_groups.items()}
    assert groups["za"] == {
        "zone_air": ("air_a", "air_a2"),
        "bay:a1": ("prox_a1", "prox_a1b"),
        "bay:a2": ("prox_a2",),
        "setpoint:air_a": ("air_a",),
    }
    assert groups["zb"] == {
        "zone_air": ("air_b",),
        "bay:b1": ("prox_b1",),
        "setpoint:air_b": ("air_b",),
    }
    # The proximal sensor of the empty bay c1 is not required.
    assert groups["zc"] == {"zone_air": ("air_c",), "setpoint:air_c": ("air_c",)}


def test_das_config_defaults(dcfg):
    assert dcfg.drive_classes == BUILTIN_DRIVE_CLASSES
    assert dcfg.drive_classes["hdd"] == DriveClass(limit_c=50.0, comfort_c=5.0, tau_d_s=720.0)
    assert dcfg.drive_classes["ssd_sata"].limit_c == 65.0
    assert dcfg.drive_classes["nvme"].comfort_c == 10.0
    assert dcfg.topology.default_class == "hdd"  # strictest built-in
    assert dcfg.bay_class("a1") == "hdd" and dcfg.bay_class("a2") == "ssd_sata"
    assert dcfg.bay_class("b1") == "hdd"  # no class declared -> default_class
    assert dcfg.topology.bays["b1"].occupied == "auto"
    assert dcfg.topology.zones["zb"].inlet == "mix"
    assert dcfg.zones == ZonePolicy(trust_rule="strict", fault_coupling="declared")
    prox = dcfg.sensors["prox_a2"]
    assert prox.quant_c == 0.0625 and prox.stuck_s == 1800.0
    assert prox.stuck_eps_c == pytest.approx(1.5 * 0.0625)
    assert dcfg.sensors["air_a"].stuck_s == 180.0
    assert dcfg.sensors["air_a"].stuck_eps_c == pytest.approx(0.015)
    assert dcfg.sensors["inlet"].stuck_s == 600.0 and dcfg.sensors["exhaust"].stuck_s == 600.0
    assert dcfg.fans["fb1"].count == 1 and dcfg.fans["fb1"].group is None
    assert dcfg.fans["fb1"].noise_weight == 1.0 and dcfg.fans["fb1"].forbidden_pwm == ()
    assert dcfg.fans["fa1"].count == 2 and dcfg.fans["fa1"].group == "front"
    assert dcfg.fans["fc1"].forbidden_pwm == ((0.4, 0.5),)
    assert dcfg.fan_models["p12"].deadband == 0.1 and dcfg.fan_models["p12"].exponent == 1.0
    assert dcfg.fan_models["p14"].noise_db_at_max == 0.0


def test_das_config_round_trips_through_json_and_yaml(dcfg, tmp_path):
    d = dcfg.to_dict()
    text = json.dumps(d)
    assert MpcConfig.from_mapping(json.loads(text)) == dcfg
    path = tmp_path / "das.yaml"
    path.write_text(yaml.safe_dump({"mpc": json.loads(text)}))
    assert load_config(path).mpc == dcfg
    # dataclasses.replace (what presets do) keeps the typed sections and revalidates.
    moved = dataclasses.replace(dcfg, setpoints={"air_a": 30.0, "air_b": 31.0, "air_c": 32.0})
    assert moved.zone_layout == dcfg.zone_layout
    assert moved.sensors == dcfg.sensors


def test_setpoints_may_be_empty_with_topology():
    cfg = das_cfg(setpoints={})
    assert cfg.setpoints == {}
    assert all(cfg.temps_for_channel(ch) == () for ch in cfg.channels)
    # The zone groups then only hold the air and bay groups.
    assert "setpoint:air_a" not in dict(cfg.zone_layout.required_groups["za"])


def test_temps_for_channel_uses_the_channels_zones(dcfg):
    assert dcfg.temps_for_channel("fa1") == ("air_a",)
    assert dcfg.temps_for_channel("fb1") == ("air_b",)
    two = das_cfg(setpoints={"air_a": SP, "prox_a1": 45.0, "air_b": SP, "air_c": SP})
    assert two.temps_for_channel("fa2") == ("air_a", "prox_a1")


def test_default_class_is_the_strictest_declared_class():
    classes = {
        "cold": {"limit_c": 45.0, "comfort_c": 5.0, "tau_d_s": 600.0, "models": ["^WDC .*"]},
        "warm": {"limit_c": 60.0, "comfort_c": 5.0, "tau_d_s": 300.0},
    }
    m = das_mapping()
    m["drive_classes"] = classes
    m["topology"]["bays"]["a1"]["class"] = "warm"
    m["topology"]["bays"]["a2"]["class"] = "warm"
    cfg = MpcConfig.from_mapping(m)
    assert set(cfg.drive_classes) == {"cold", "warm"}  # given classes replace the built-ins
    assert cfg.topology.default_class == "cold"
    assert cfg.drive_classes["cold"].models == ("^WDC .*",)
    m["topology"]["default_class"] = "warm"
    assert MpcConfig.from_mapping(m).topology.default_class == "warm"


def test_stuck_params_per_role_with_automatic_decimation():
    cfg = das_cfg(dt=5.0, confirm_s=10.0, fallback_hold_s=20.0, stuck_s=10.0)
    prox = cfg.stuck_params("prox_a2")
    assert (prox.ticks, prox.decimate, prox.samples) == (360, 6, 60)
    assert prox.eps_c == pytest.approx(0.09375)
    assert prox.channels == ("fa1", "fa2")
    assert prox.siblings == ()  # no other proximal sensor on bay a2 (prox_a1 is bay a1's)
    assert cfg.stuck_params("prox_a1").siblings == ("prox_a1b",)  # same zone, role and bay
    assert prox.air == ("air_a", "air_a2")  # the zone air can cancel an airflow move
    # za's airflow: fa1 carries 2 fans, fa2 one, both only in za; p12 curve (0.1, 1.0)
    assert prox.airflow == (
        ("fa1", pytest.approx(2 / 3), 0.1, 1.0),
        ("fa2", pytest.approx(1 / 3), 0.1, 1.0),
    )
    air = cfg.stuck_params("air_b")
    assert (air.ticks, air.decimate, air.samples) == (36, 1, 36)
    assert air.channels == ("fb1",) and air.siblings == () and air.air == ()
    assert air.airflow == (("fb1", 1.0, 0.2, 1.1),)
    inlet = cfg.stuck_params("inlet")
    assert (inlet.ticks, inlet.decimate, inlet.samples) == (120, 2, 60)
    assert inlet.channels == ()  # no zone: fans are not evidence for an inlet sensor
    assert inlet.airflow == () and inlet.air == ()
    exhaust = cfg.stuck_params("exhaust")
    assert exhaust.channels == ("fb1",) and exhaust.siblings == () and exhaust.air == ()
    # Dense window: the longest non-decimated sensor window (the zone-air sensors).
    assert cfg.window_ticks == 36
    assert cfg.slow_window_samples == {2: 60, 6: 60}


def test_stuck_decimate_and_window_overrides():
    m = das_mapping()
    m["sensors"]["prox_b1"].update({"stuck_s": 20.0, "stuck_decimate": 5, "stuck_eps_c": 0.2})
    m["sensors"]["air_c"].update({"stuck_s": 12.0})
    cfg = MpcConfig.from_mapping(m)
    p = cfg.stuck_params("prox_b1")
    assert (p.ticks, p.decimate, p.samples, p.eps_c) == (20, 5, 4, 0.2)
    assert cfg.stuck_params("air_c").ticks == 12


def test_legacy_config_is_one_implicit_zone(cfg):
    assert not cfg.is_das
    layout = cfg.zone_layout
    assert layout.implicit and layout.zones == (IMPLICIT_ZONE,)
    assert layout.zone_channels[IMPLICIT_ZONE] == cfg.channels
    assert layout.reach[IMPLICIT_ZONE] == cfg.channels
    assert dict(layout.required_groups[IMPLICIT_ZONE]) == {t: (t,) for t in cfg.temps}
    for name in cfg.temps:
        p = cfg.stuck_params(name)
        assert (p.ticks, p.eps_c, p.decimate, p.samples) == (
            cfg.stuck_ticks,
            cfg.stuck_eps_c,
            1,
            cfg.stuck_ticks,
        )
        assert p.channels == cfg.channels
        assert p.siblings == tuple(t for t in cfg.temps if t != name)
        assert p.airflow == () and p.air == ()  # each channel's own PWM move, no air rule
    assert cfg.window_ticks == cfg.stuck_ticks and cfg.slow_window_samples == {}
    for section in DAS_SECTIONS:
        assert getattr(cfg, section) in (None, {})


# ---------------------------------------------------------------------------
# config: rejection rules (section 7 table + module docstring)
# ---------------------------------------------------------------------------


def _set(path: str, value: Any) -> Callable[[dict[str, Any]], None]:
    keys = path.split(".")

    def mutate(m: dict[str, Any]) -> None:
        node = m
        for key in keys[:-1]:
            node = node[key]
        node[keys[-1]] = value

    return mutate


def _drop(path: str) -> Callable[[dict[str, Any]], None]:
    keys = path.split(".")

    def mutate(m: dict[str, Any]) -> None:
        node = m
        for key in keys[:-1]:
            node = node[key]
        del node[keys[-1]]

    return mutate


def _both(*muts: Callable[[dict[str, Any]], None]) -> Callable[[dict[str, Any]], None]:
    def mutate(m: dict[str, Any]) -> None:
        for mut in muts:
            mut(m)

    return mutate


REJECT = [
    pytest.param(_set("topology.zones", {}), id="zones_empty"),
    pytest.param(_set("topology.zones.zc.channels", []), id="zone_channels_empty"),
    pytest.param(_set("topology.zones.zc.channels", ["nope"]), id="zone_channel_unknown"),
    pytest.param(_set("topology.zones.zc.channels", "fc1"), id="zone_channels_not_list"),
    pytest.param(_set("topology.zones.zc.coupled_to", ["zx"]), id="coupled_unknown"),
    pytest.param(_set("topology.zones.zc.coupled_to", ["zc"]), id="coupled_self"),
    pytest.param(_set("topology.zones.zc.coupled_to", ["za"]), id="coupled_asymmetric"),
    pytest.param(_set("topology.zones.zb.inlet", "air_b"), id="inlet_wrong_role"),
    pytest.param(_set("topology.zones.zb.inlet", "ghost"), id="inlet_unknown"),
    pytest.param(_set("topology.zones.zb.fans", ["fb1"]), id="zone_unknown_key"),
    pytest.param(
        _both(_set("topology.zones.zc.channels", ["fb1"]), _set("fans.fc1", {"model": "p14"})),
        id="channel_in_no_zone",
    ),
    pytest.param(_set("topology.bays.b1.zone", "zx"), id="bay_zone_unknown"),
    pytest.param(_drop("topology.bays.b1.zone"), id="bay_zone_missing"),
    pytest.param(_set("topology.bays.b1.class", "tape"), id="bay_class_unknown"),
    pytest.param(_set("topology.bays.b1.occupied", "maybe"), id="bay_occupied_invalid"),
    pytest.param(_set("topology.bays.b1.occupied", 1), id="bay_occupied_int"),
    pytest.param(_set("topology.bays.b1.serial", 12345), id="bay_serial_not_string"),
    pytest.param(_set("topology.default_class", "tape"), id="default_class_unknown"),
    pytest.param(_set("topology.racks", {}), id="topology_unknown_key"),
    pytest.param(
        _set("drive_classes", {"hdd": {"limit_c": 150.0, "comfort_c": 5, "tau_d_s": 1}}),
        id="class_limit_above_range",
    ),
    pytest.param(
        _set("drive_classes", {"hdd": {"limit_c": -30.0, "comfort_c": 5, "tau_d_s": 1}}),
        id="class_limit_below_range",
    ),
    pytest.param(
        _set("drive_classes", {"hdd": {"limit_c": 50.0, "comfort_c": -1, "tau_d_s": 1}}),
        id="class_comfort_negative",
    ),
    pytest.param(
        _set("drive_classes", {"hdd": {"limit_c": 50.0, "comfort_c": 5, "tau_d_s": 0}}),
        id="class_tau_zero",
    ),
    pytest.param(
        _set("drive_classes", {"hdd": {"limit_c": 50.0, "comfort_c": 5}}),
        id="class_missing_tau",
    ),
    pytest.param(
        _set(
            "drive_classes",
            {"hdd": {"limit_c": 50.0, "comfort_c": 5, "tau_d_s": 9, "models": ["("]}},
        ),
        id="class_models_bad_regex",
    ),
    pytest.param(
        _set("drive_classes", {"x": {"limit_c": 50.0, "comfort_c": 5, "tau_d_s": 9}}),
        id="bay_class_gone_with_replaced_classes",
    ),
    pytest.param(_drop("sensors.exhaust"), id="sensors_missing_temp"),
    pytest.param(_set("sensors.ghost", {"role": "inlet"}), id="sensors_extra_key"),
    pytest.param(_set("sensors.exhaust.role", "ceiling"), id="sensor_role_invalid"),
    pytest.param(_drop("sensors.air_b.zone"), id="zone_air_without_zone"),
    pytest.param(_set("sensors.air_b.zone", "zx"), id="sensor_zone_unknown"),
    pytest.param(_drop("sensors.prox_b1.bay"), id="proximal_without_bay"),
    pytest.param(_set("sensors.prox_b1.bay", "bx"), id="proximal_bay_unknown"),
    pytest.param(_set("sensors.prox_b1.bay", "a2"), id="proximal_bay_in_other_zone"),
    pytest.param(_set("sensors.exhaust.bay", "b1"), id="bay_on_non_proximal"),
    pytest.param(_set("sensors.air_b.quant_c", 0.0), id="quant_zero"),
    pytest.param(_set("sensors.air_b.stuck_s", 1.5), id="stuck_s_below_2dt"),
    pytest.param(_set("sensors.air_b.stuck_eps_c", 0.005), id="stuck_eps_below_quant"),
    pytest.param(_set("sensors.air_b.stuck_decimate", 0), id="stuck_decimate_zero"),
    pytest.param(_set("sensors.air_b.stuck_decimate", 1000), id="stuck_decimate_too_coarse"),
    pytest.param(_set("sensors.air_b.redundant", "yes"), id="redundant_not_bool"),
    pytest.param(_set("sensors.air_b.tau_s", -1.0), id="tau_negative"),
    pytest.param(_set("sensors.air_b.height", 3), id="sensor_unknown_key"),
    pytest.param(_set("sensors.air_b.redundant", True), id="zone_without_primary_air"),
    pytest.param(_set("sensors.prox_b1.redundant", True), id="bay_without_primary_proximal"),
    pytest.param(_set("sensors.inlet.redundant", True), id="no_primary_inlet"),
    pytest.param(
        _both(_set("sensors.inlet.role", "exhaust"), _set("topology.zones.za.inlet", "mix")),
        id="no_inlet_sensor",
    ),
    pytest.param(_set("setpoints", {"inlet": 25.0}), id="setpoint_on_inlet"),
    pytest.param(
        _set("setpoints", {"air_a": 35.0, "air_b": 35.0}), id="channel_without_setpoint_temp"
    ),
    pytest.param(_set("channel_temps", {ch: ["air_a"] for ch in CHANNELS}), id="channel_temps"),
    pytest.param(_drop("fans.fc1"), id="fans_missing_channel"),
    pytest.param(_set("fans.fx", {"model": "p12"}), id="fans_extra_channel"),
    pytest.param(_set("fans.fb1.model", "nf-a14"), id="fan_model_unknown"),
    pytest.param(_drop("fans.fb1.model"), id="fan_model_missing"),
    pytest.param(_set("fans.fb1.count", 0), id="fan_count_zero"),
    pytest.param(_set("fans.fb1.count", 1.5), id="fan_count_not_int"),
    pytest.param(_set("fans.fb1.noise_weight", -0.1), id="fan_noise_weight_negative"),
    pytest.param(_set("fans.fb1.group", ""), id="fan_group_empty"),
    pytest.param(_set("fans.fb1.forbidden_pwm", [[0.5, 0.4]]), id="band_lo_ge_hi"),
    pytest.param(_set("fans.fb1.forbidden_pwm", [[0.1, 0.2]]), id="band_below_pwm_min"),
    pytest.param(_set("fans.fb1.forbidden_pwm", [[0.3, 0.6]]), id="band_wider_than_0p2"),
    pytest.param(_set("fans.fb1.forbidden_pwm", [[0.3]]), id="band_not_pair"),
    pytest.param(_set("fans.fb1.forbidden_pwm", [0.3, 0.4]), id="band_not_nested"),
    pytest.param(_set("fan_models.p14.rpm_max", 0), id="model_rpm_max_zero"),
    pytest.param(_set("fan_models.p14.deadband", 0.5), id="model_deadband_high"),
    pytest.param(_set("fan_models.p14.deadband", -0.1), id="model_deadband_negative"),
    pytest.param(_set("fan_models.p14.exponent", 0.4), id="model_exponent_low"),
    pytest.param(_set("fan_models.p14.exponent", 1.6), id="model_exponent_high"),
    pytest.param(_set("fan_models.p14.noise_db_at_max", math.inf), id="model_noise_not_finite"),
    pytest.param(_drop("fan_models.p14.rpm_max"), id="model_rpm_max_missing"),
    pytest.param(_set("fan_models", {}), id="fan_models_empty"),
    pytest.param(_set("zones", {"trust_rule": "loose"}), id="trust_rule_invalid"),
    pytest.param(_set("zones", {"fault_coupling": "all"}), id="fault_coupling_invalid"),
    pytest.param(_set("zones", {"strict": True}), id="zones_unknown_key"),
    pytest.param(_set("topology", ["za"]), id="topology_not_mapping"),
]


@pytest.mark.parametrize("mutate", REJECT)
def test_das_config_rejects(mutate):
    m = das_mapping()
    mutate(m)
    with pytest.raises(ConfigError):
        MpcConfig.from_mapping(m)


@pytest.mark.parametrize(
    "section, value",
    [
        ("sensors", {"coolant": {"role": "inlet"}}),
        ("drive_classes", {"hdd": {"limit_c": 50, "comfort_c": 5, "tau_d_s": 720}}),
        ("fans", {"radiator": {"model": "p12"}}),
        ("fan_models", {"p12": {"rpm_max": 1800}}),
        ("zones", {"trust_rule": "strict"}),
    ],
)
def test_das_sections_are_rejected_without_topology(cfg, section, value):
    data = cfg.to_dict()
    data[section] = value
    with pytest.raises(ConfigError, match="requires mpc.topology"):
        MpcConfig.from_mapping(data)


def test_empty_das_sections_are_legacy(cfg):
    data = cfg.to_dict()
    data.update(topology=None, sensors={}, drive_classes={}, fans={}, fan_models={}, zones=None)
    assert MpcConfig.from_mapping(data) == cfg
    data.update(sensors=None, drive_classes=None, fans=None, fan_models=None)
    assert MpcConfig.from_mapping(data) == cfg


def test_forbidden_band_edges_accepted():
    m = das_mapping()
    m["fans"]["fb1"]["forbidden_pwm"] = [[0.15, 0.35], [0.8, 1.0]]
    assert MpcConfig.from_mapping(m).fans["fb1"].forbidden_pwm == ((0.15, 0.35), (0.8, 1.0))


# ---------------------------------------------------------------------------
# zone trust (strict rule)
# ---------------------------------------------------------------------------


def verdicts_for(
    cfg: MpcConfig, obs: PlantObservation, time_status: str = "ok"
) -> dict[str, zones.ZoneTrust]:
    gate = evaluate_gate(obs, cfg, last_good_obs=None, last_raw_temps=None, window=())
    return zones.evaluate(gate, time_status, cfg)


def trusted_zones(v: dict[str, zones.ZoneTrust]) -> set[str]:
    return {z for z, t in v.items() if t.trusted}


def test_every_zone_trusted_on_a_clean_sample(dcfg):
    v = verdicts_for(dcfg, das_obs(dcfg, 0.0))
    assert trusted_zones(v) == {"za", "zb", "zc"}
    assert all(t.reasons == () for t in v.values())
    assert list(v) == ["za", "zb", "zc"]


@pytest.mark.parametrize("name", ["air_a2", "prox_a1b", "prox_a1"])
def test_one_member_of_a_redundant_group_lost_keeps_the_zone(dcfg, name):
    obs = das_obs(dcfg, 0.0, **{name: None})
    assert trusted_zones(verdicts_for(dcfg, obs)) == {"za", "zb", "zc"}


def test_whole_group_lost_faults_only_its_zone(dcfg):
    obs = das_obs(dcfg, 0.0, prox_a1=None, prox_a1b=math.nan)
    v = verdicts_for(dcfg, obs)
    assert trusted_zones(v) == {"zb", "zc"}
    assert v["za"].reasons == ("bay:a1:prox_a1=null,prox_a1b=non_finite",)


@pytest.mark.parametrize("name", ["prox_c1", "inlet", "exhaust"])
def test_sensor_outside_every_group_never_faults_a_zone(dcfg, name):
    v = verdicts_for(dcfg, das_obs(dcfg, 0.0, drop=(name,)))
    assert trusted_zones(v) == {"za", "zb", "zc"}


def test_setpoint_sensor_is_required_even_with_a_redundant_partner(dcfg):
    # air_a2 would satisfy the zone_air group, but air_a carries the setpoint.
    v = verdicts_for(dcfg, das_obs(dcfg, 0.0, air_a=200.0))
    assert trusted_zones(v) == {"zb", "zc"}
    assert v["za"].reasons == ("setpoint:air_a:air_a=range",)
    no_sp = das_cfg(setpoints={})
    assert trusted_zones(verdicts_for(no_sp, das_obs(no_sp, 0.0, air_a=200.0))) == {
        "za",
        "zb",
        "zc",
    }


def test_unknown_bay_occupancy_is_required_and_empty_bay_is_not():
    m = das_mapping()
    m["topology"]["bays"]["c1"]["occupied"] = "auto"
    cfg = MpcConfig.from_mapping(m)
    assert trusted_zones(verdicts_for(cfg, das_obs(cfg, 0.0, prox_c1=None))) == {"za", "zb"}
    m["topology"]["bays"]["c1"]["occupied"] = True
    cfg = MpcConfig.from_mapping(m)
    assert trusted_zones(verdicts_for(cfg, das_obs(cfg, 0.0, prox_c1=None))) == {"za", "zb"}


@pytest.mark.parametrize("status", ["not_advancing", "gap"])
def test_bad_time_faults_every_zone(dcfg, status):
    v = verdicts_for(dcfg, das_obs(dcfg, 0.0), time_status=status)
    assert trusted_zones(v) == set()
    assert all(t.reasons == (f"time:{status}",) for t in v.values())


def test_unknown_key_faults_every_zone(dcfg):
    obs = das_obs(dcfg, 0.0, gpu=45.0)
    v = verdicts_for(dcfg, obs)
    assert trusted_zones(v) == set()
    assert v["zc"].reasons == ("unknown_keys:gpu",)


def test_sigma_rule_applies_strict_only_without_an_estimator_update():
    """``tests/test_sigma_trust.py`` covers the rule itself."""
    cfg = das_cfg(zones={"trust_rule": "sigma"})
    assert zones.effective_trust_rule(cfg) == "strict"
    obs = das_obs(cfg, 0.0, prox_a2=None)
    assert trusted_zones(verdicts_for(cfg, obs)) == {"zb", "zc"}
    cmd, _ = step(obs, cfg, MpcState.cold())
    assert cmd.diagnostics["trust_rule"] == "sigma"
    assert cmd.diagnostics["zones_in_fault"] == []  # a2's sigma is the estimator's prior


@pytest.mark.parametrize(
    "temps",
    [
        {"coolant": 35.0, "air": 30.0},
        {"coolant": None, "air": 30.0},
        {"coolant": 35.0},
        {"coolant": 35.0, "air": 30.0, "gpu": 1.0},
        {"coolant": 500.0, "air": math.nan},
    ],
)
@pytest.mark.parametrize("status", ["first", "ok", "gap", "not_advancing"])
def test_legacy_implicit_zone_verdict_is_the_whole_tick_verdict(cfg, temps, status):
    obs = make_obs(cfg, 0.0, temps=temps)
    gate = evaluate_gate(obs, cfg, last_good_obs=None, last_raw_temps=None, window=())
    v = zones.evaluate(gate, status, cfg)
    assert list(v) == [IMPLICIT_ZONE]
    assert v[IMPLICIT_ZONE].trusted == (gate.trusted and status in ("first", "ok"))


# ---------------------------------------------------------------------------
# closure F* and channels under fallback policy
# ---------------------------------------------------------------------------


def test_closure_follows_declared_coupling(dcfg):
    assert zones.closure(["za"], dcfg) == ("za", "zb")
    assert zones.closure(["zb"], dcfg) == ("za", "zb")
    assert zones.closure(["zc"], dcfg) == ("zc",)
    assert zones.closure([], dcfg) == ()
    assert zones.closure(["zc", "za"], dcfg) == ("za", "zb", "zc")
    assert zones.fallback_channels(["za"], dcfg) == ("fa1", "fa2", "fb1")
    assert zones.fallback_channels(["zc"], dcfg) == ("fc1",)
    assert zones.fallback_channels([], dcfg) == ()


def test_closure_without_coupling_is_strictly_per_zone():
    cfg = das_cfg(zones={"fault_coupling": "none"})
    assert zones.closure(["za"], cfg) == ("za",)
    assert zones.fallback_channels(["za"], cfg) == ("fa1", "fa2")
    assert zones.fallback_channels(["zb"], cfg) == ("fb1",)


def test_closure_is_one_hop_on_a_chain():
    m = das_mapping()
    m["topology"]["zones"]["zb"]["coupled_to"] = ["za", "zc"]
    m["topology"]["zones"]["zc"]["coupled_to"] = ["zb"]
    cfg = MpcConfig.from_mapping(m)
    assert zones.closure(["za"], cfg) == ("za", "zb")  # zc exchanges air with zb, not za
    assert zones.fallback_channels(["za"], cfg) == ("fa1", "fa2", "fb1")
    assert zones.fallback_channels(["zb"], cfg) == CHANNELS


def test_shared_channel_is_under_fallback_when_any_of_its_zones_faults():
    m = das_mapping()
    m["topology"]["zones"]["zc"]["channels"] = ["fc1", "fb1"]
    cfg = MpcConfig.from_mapping(m)
    assert cfg.zone_layout.channel_zones["fb1"] == ("zb", "zc")
    assert zones.fallback_channels(["zc"], cfg) == ("fb1", "fc1")


def test_channel_elapsed_takes_the_oldest_reaching_fault(dcfg):
    out = zones.channel_fallback_elapsed({"za": 3.0, "zb": 7.0, "zc": 1.0}, dcfg)
    assert out == {"fa1": 7.0, "fa2": 7.0, "fb1": 7.0, "fc1": 1.0}
    assert zones.channel_fallback_elapsed({"zc": 2.0}, dcfg) == {"fc1": 2.0}
    assert zones.fallback_target(0.9, 0.8, 5.0, dcfg) == (0.9, "ramp_high")
    assert zones.fallback_target(0.3, 0.8, 5.0, dcfg) == (0.8, "ramp_high")
    assert zones.fallback_target(0.3, 0.8, 4.0, dcfg) == (0.3, "hold")


# ---------------------------------------------------------------------------
# step: modes, per-zone timers, zone_faults
# ---------------------------------------------------------------------------


def run(cfg: MpcConfig, observations, state: MpcState | None = None, **kwargs):
    state = MpcState.cold() if state is None else state
    out = []
    for obs in observations:
        cmd, state = checked_step(obs, cfg, state, **kwargs)
        out.append((cmd, state))
    return out


def test_clean_ticks_are_auto_with_healthy_zone_faults(dcfg):
    (cmd, st), *_ = run(dcfg, [das_obs(dcfg, 0.0)])
    assert cmd.mode is Mode.AUTO
    assert st.zone_faults == {z: ZoneFault(streak=1) for z in ("za", "zb", "zc")}
    assert cmd.diagnostics["zones_in_fault"] == [] and cmd.diagnostics["fallback_channels"] == []
    assert cmd.diagnostics["policy_by_channel"] == dict.fromkeys(CHANNELS, "solver")
    assert cmd.diagnostics["zones"]["zb"]["policy"] == "solver"
    assert not st.in_fault


def test_one_zone_fault_is_degraded_every_zone_fault_is_fallback(dcfg):
    seq = [
        das_obs(dcfg, 0.0),
        das_obs(dcfg, 1.0, air_c=None),
        das_obs(dcfg, 2.0, air_c=None, air_a=None, air_b=None),
    ]
    (c0, _), (c1, s1), (c2, s2) = run(dcfg, seq)
    assert c0.mode is Mode.AUTO
    assert c1.mode is Mode.DEGRADED
    assert s1.zone_faults["zc"] == ZoneFault(since_ts=1.0, reason="sensor_gate", streak=0, ticks=1)
    assert s1.zone_faults["za"].streak == 2 and not s1.zone_faults["za"].in_fault
    assert (s1.fault_since_ts, s1.fault_reason, s1.trusted_streak) == (
        1.0,
        FaultReason.SENSOR_GATE,
        0,
    )
    assert c1.diagnostics["fallback_channels"] == ["fc1"]
    assert c1.diagnostics["zones"]["zc"]["policy"] == "hold"
    assert c2.mode is Mode.FALLBACK
    assert c2.diagnostics["zones_in_fault"] == ["za", "zb", "zc"]
    assert s2.fault_since_ts == 1.0  # earliest zone fault
    assert c2.diagnostics["fault_ticks"] == 2


def test_zone_faults_round_trip_through_json(dcfg):
    (_, s0), (_, s1) = run(dcfg, [das_obs(dcfg, 0.0), das_obs(dcfg, 1.0, air_b=None)])
    text = json.dumps(s1.to_dict(), allow_nan=False)
    back = MpcState.from_dict(json.loads(text))
    assert back == s1
    assert back.zone_faults["zb"].reason is FaultReason.SENSOR_GATE
    cmd_a, st_a = step(das_obs(dcfg, 2.0), dcfg, s1)
    cmd_b, st_b = step(das_obs(dcfg, 2.0), dcfg, back)
    assert cmd_a.to_dict() == cmd_b.to_dict() and st_a.to_dict() == st_b.to_dict()


def test_legacy_state_and_diagnostics_have_no_zone_keys(cfg):
    cmd, st = checked_step(make_obs(cfg, 0.0), cfg, MpcState.cold())
    assert st.zone_faults == {} and "zone_faults" not in st.to_dict()
    for key in ("zones", "zones_in_fault", "fallback_channels", "policy_by_channel", "trust_rule"):
        assert key not in cmd.diagnostics
    assert "stuck_slow" not in st.solver_memory and "stuck_seq" not in st.solver_memory


def test_zone_fault_validation():
    with pytest.raises(ValueError):
        ZoneFault(since_ts=1.0)
    with pytest.raises(ValueError):
        ZoneFault(reason="solver")
    with pytest.raises(ValueError):
        ZoneFault(streak=-1)
    with pytest.raises(TypeError):
        ZoneFault(ticks=1.5)  # type: ignore[arg-type]
    with pytest.raises(TypeError):
        ZoneFault(since_ts=math.nan, reason="solver")
    with pytest.raises(TypeError):
        MpcState(zone_faults={"za": {"since_ts": None}})  # type: ignore[dict-item]
    with pytest.raises(ValueError):
        ZoneFault.from_dict({"since_ts": 1.0, "reason": "cosmic"})
    zf = ZoneFault(since_ts=2, reason=FaultReason.SOLVER, streak=1, ticks=3)
    assert ZoneFault.from_dict(json.loads(json.dumps(zf.to_dict()))) == zf


def test_per_zone_confirm_ticks_and_independent_timers(dcfg):
    assert dcfg.confirm_ticks == 2
    seq = [das_obs(dcfg, 0.0, air_b=None)]  # zb faults at t=0
    seq += [das_obs(dcfg, 1.0, air_b=None, air_c=None)]  # zc faults at t=1
    seq += [das_obs(dcfg, 2.0, air_c=None)]  # air_b returns for the first time: zb trusted 1/2
    seq += [das_obs(dcfg, 3.0, air_c=None)]  # zb's zone streak confirms, but air_b -- missing
    # since boot, no reference behind its first reading (item 61) -- runs its own
    # confirm_ticks like any other return, one tick behind the zone streak
    seq += [das_obs(dcfg, 4.0)]  # air_b confirms, zb clears; air_c returns: zc trusted 1/2
    seq += [das_obs(dcfg, 5.0)]  # zc confirms
    r = run(dcfg, seq)
    faults = [{z: st.zone_faults[z].in_fault for z in ("za", "zb", "zc")} for _, st in r]
    assert faults == [
        {"za": False, "zb": True, "zc": False},
        {"za": False, "zb": True, "zc": True},
        {"za": False, "zb": True, "zc": True},
        {"za": False, "zb": True, "zc": True},
        {"za": False, "zb": False, "zc": True},
        {"za": False, "zb": False, "zc": False},
    ]
    assert [c.mode for c, _ in r] == [Mode.DEGRADED] * 5 + [Mode.AUTO]
    assert r[2][1].zone_faults["zb"].since_ts == 0.0 and r[2][1].zone_faults["zc"].since_ts == 1.0
    assert r[3][1].fault_since_ts == 0.0  # zb: air_b is still confirming its own first reading
    assert r[4][1].fault_since_ts == 1.0  # zb cleared: the aggregate now follows zc


def test_zone_missing_from_state_starts_in_fault_when_global_fault_is_active(dcfg):
    state = MpcState(fault_since_ts=0.0, fault_reason="sensor_gate", trusted_streak=5)
    cmd, st = checked_step(das_obs(dcfg, 1.0), dcfg, state)
    assert cmd.mode is Mode.FALLBACK
    assert all(f.since_ts == 0.0 and f.streak == 1 for f in st.zone_faults.values())


# ---------------------------------------------------------------------------
# compose per channel
# ---------------------------------------------------------------------------


def degraded_cmd(pwm: float = 0.6, fallback: list[str] | None = None) -> MpcCommand:
    diag: dict[str, Any] = {"k": 1}
    if fallback is not None:
        diag["fallback_channels"] = fallback
    return MpcCommand(pwm=dict.fromkeys(CHANNELS, pwm), mode=Mode.DEGRADED, diagnostics=diag)


def test_compose_fallback_beats_manual_per_channel(dcfg):
    sup = Supervisor(dcfg)
    sup.submit(SetMode(ControlMode.MANUAL))
    for ch in CHANNELS:
        sup.submit(SetPwm(ch, dcfg.pwm_min))
    prev = dict.fromkeys(CHANNELS, 0.6)
    out = sup.compose(degraded_cmd(0.6, ["fa1", "fa2", "fb1"]), sup.plan_tick(), prev)
    assert out.mode is Mode.DEGRADED
    assert out.pwm["fa1"] == out.pwm["fa2"] == out.pwm["fb1"] == 0.6  # fallback policy wins
    assert out.pwm["fc1"] == pytest.approx(0.5)  # override applies, rate limited
    diag = out.diagnostics["supervisor"]
    assert diag["overrides_applied"] is True
    assert diag["overrides_blocked"] == ["fa1", "fa2", "fb1"]
    assert diag["override_rate_limited"] == {"fc1": True}
    assert out.diagnostics["k"] == 1


def test_compose_degraded_without_channel_list_blocks_every_override(dcfg):
    sup = Supervisor(dcfg)
    sup.submit(SetMode(ControlMode.MIXED))
    sup.submit(SetPwm("fc1", dcfg.pwm_min))
    prev = dict.fromkeys(CHANNELS, 0.6)
    for bad in (None, "fc1", [1, 2]):
        cmd = degraded_cmd(0.6, fallback=bad)  # type: ignore[arg-type]
        out = sup.compose(cmd, sup.plan_tick(), prev)
        assert out.pwm == cmd.pwm
        assert out.diagnostics["supervisor"]["overrides_applied"] is False


def test_compose_in_a_real_degraded_tick(dcfg):
    sup = Supervisor(dcfg)
    sup.submit(SetMode(ControlMode.MANUAL))
    for ch in CHANNELS:
        sup.submit(SetPwm(ch, dcfg.pwm_min))
    state = MpcState.cold()
    obs = das_obs(dcfg, 0.0, pwm=0.7, air_c=None)
    cmd, state = step(obs, dcfg, state)
    assert cmd.mode is Mode.DEGRADED
    out = sup.compose(cmd, sup.plan_tick(), dict.fromkeys(CHANNELS, 0.7))
    assert out.pwm["fc1"] == pytest.approx(0.7)  # zc in fault: hold, override blocked
    assert all(out.pwm[ch] == pytest.approx(0.6) for ch in ("fa1", "fa2", "fb1"))


# ---------------------------------------------------------------------------
# publishers: health, MQTT state, HTML banner
# ---------------------------------------------------------------------------


def test_solver_status_degraded_in_health_and_mqtt_state(dcfg):
    sup = Supervisor(dcfg, clock=lambda: 10.0)
    state = MpcState.cold()
    obs = das_obs(dcfg, 0.0, air_b=None)
    cmd, state = step(obs, dcfg, state)
    sup.record_tick(obs=obs, mpc_cmd=cmd, cmd=cmd, state=state, applied=True, usb_present=True)
    snap = sup.snapshot()
    assert ControlSnapshot.solver_status_for(cmd) is SolverStatus.DEGRADED
    assert snap.solver_status is SolverStatus.DEGRADED
    health = snap.health_payload()
    assert health["solver"] == "degraded"
    assert health["fault_reason"] == "sensor_gate" and health["fault_since_ts"] == 0.0
    blob = json.loads(json.dumps(state_payload(snap.to_dict(), {"load1": 0.1}), allow_nan=False))
    assert blob["health"]["solver"] == "degraded"
    assert blob["cmd"]["mode"] == "degraded"
    assert blob["cmd"]["diagnostics"]["zones_in_fault"] == ["zb"]
    assert blob["extra"]["mpc_mode"] == "degraded"


def test_html_shows_a_degraded_banner_naming_zones():
    html = (REPO_ROOT / "src/aqua_bridge/publishers/static/index.html").read_text()
    assert 'id="degraded"' in html
    assert 'health.solver === "degraded"' in html
    assert "zones_in_fault" in html
    # the FAULT banner stays reserved for fallback / fault
    assert 'health.solver === "fault" || health.solver === "fallback"' in html


def test_mode_enum_has_degraded():
    assert Mode("degraded") is Mode.DEGRADED
    assert MpcCommand(pwm={}, mode="degraded").mode is Mode.DEGRADED
    assert SolverStatus("degraded") is SolverStatus.DEGRADED


# ---------------------------------------------------------------------------
# PlantObservation.inputs
# ---------------------------------------------------------------------------


def test_observation_inputs_default_empty_and_omitted_from_dict():
    obs = PlantObservation(temps={"a": 1.0}, rpm={}, pwm={}, ts=0.0)
    assert obs.inputs == {}
    assert "inputs" not in obs.to_dict()
    assert PlantObservation.from_dict(obs.to_dict()) == obs


def test_observation_inputs_round_trip_and_are_isolated():
    smart = {"smart": {"WD-123": {"temp_c": 41, "age_s": 12.5, "model": "WDC WD80"}}}
    obs = PlantObservation(temps={}, rpm={}, pwm={}, ts=1.0, inputs=smart)
    smart["smart"]["WD-123"]["temp_c"] = 99  # caller's dict changes; the frozen obs does not
    assert obs.inputs["smart"]["WD-123"]["temp_c"] == 41
    back = PlantObservation.from_dict(json.loads(json.dumps(obs.to_dict(), allow_nan=False)))
    assert back == obs and back.inputs == obs.inputs


@pytest.mark.parametrize(
    "inputs, exc",
    [
        ([1, 2], TypeError),
        ({1: "x"}, TypeError),
        ({"smart": math.nan}, ValueError),
        ({"smart": {"t": math.inf}}, ValueError),
        ({"smart": object()}, ValueError),
    ],
)
def test_observation_inputs_must_be_finite_json(inputs, exc):
    with pytest.raises(exc):
        PlantObservation(temps={}, rpm={}, pwm={}, ts=0.0, inputs=inputs)


@pytest.mark.parametrize("config", ["legacy", "das"])
def test_observation_inputs_are_not_gated(cfg, config):
    c = cfg if config == "legacy" else das_cfg()
    obs = make_obs(c, 0.0) if config == "legacy" else das_obs(c, 0.0)
    with_inputs = dataclasses.replace(obs, inputs={"smart": {"s1": {"temp_c": 500}}, "junk": "x"})
    a_cmd, a_st = checked_step(obs, c, MpcState.cold())
    b_cmd, b_st = checked_step(with_inputs, c, MpcState.cold())
    assert a_cmd.to_dict() == b_cmd.to_dict() and a_st.to_dict() == b_st.to_dict()
