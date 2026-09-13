"""Config additions of the PI-DAS milestone: ``noise``, ``topology.bays.<b>.limit_c``,
the served-zone view, and the simulator topology built from a zoned config.
"""

from __future__ import annotations

import json
from typing import Any

import pytest
import yaml

from aqua_bridge.config import load_config
from aqua_bridge.model import DAS_SECTIONS, NOISE_DEFAULTS, ConfigError, MpcConfig, NoiseSpec
from aqua_bridge.sim.das import build_das_params, topology_from_config
from das_fixtures import das_cfg, das_mapping


def test_noise_defaults_with_topology_and_absent_in_legacy(cfg):
    dcfg = das_cfg()
    assert dcfg.noise == NoiseSpec()
    assert dcfg.noise.to_dict() == NOISE_DEFAULTS
    assert cfg.noise is None and "noise" in DAS_SECTIONS


def test_noise_section_parses_and_round_trips(tmp_path):
    m = das_mapping()
    m["noise"] = {"exponent": 4, "weight_noise": 0.5}
    cfg = MpcConfig.from_mapping(m)
    assert (cfg.noise.exponent, cfg.noise.weight_noise, cfg.noise.band_hysteresis) == (
        4.0,
        0.5,
        0.02,
    )
    assert MpcConfig.from_mapping(json.loads(json.dumps(cfg.to_dict()))) == cfg
    path = tmp_path / "c.yaml"
    path.write_text(yaml.safe_dump({"mpc": cfg.to_dict()}))
    assert load_config(path).mpc == cfg


@pytest.mark.parametrize(
    "noise",
    [
        {"exponent": 2.5},
        {"exponent": 7.5},
        {"weight_noise": -0.1},
        {"band_hysteresis": -0.01},
        {"weight_noise": "loud"},
        {"volume": 1},
        [],
    ],
)
def test_noise_section_rejects_bad_values(noise: Any):
    m = das_mapping()
    m["noise"] = noise
    with pytest.raises(ConfigError):
        MpcConfig.from_mapping(m)


def test_noise_requires_topology(cfg):
    data = cfg.to_dict()
    data["noise"] = {"exponent": 5}
    with pytest.raises(ConfigError, match="requires mpc.topology"):
        MpcConfig.from_mapping(data)


def test_bay_limit_parses_validates_and_tightens():
    m = das_mapping()
    m["topology"]["bays"]["b1"]["limit_c"] = 47
    cfg = MpcConfig.from_mapping(m)
    assert cfg.topology.bays["b1"].limit_c == 47.0 and cfg.bay_limit("b1") == 47.0
    assert cfg.bay_limit("a1") == 50.0 and cfg.bay_comfort("a2") == 10.0
    assert MpcConfig.from_mapping(json.loads(json.dumps(cfg.to_dict()))) == cfg
    for bad in (120.0, -30.0, "hot"):
        m["topology"]["bays"]["b1"]["limit_c"] = bad
        with pytest.raises(ConfigError):
            MpcConfig.from_mapping(m)


def test_bay_limit_on_legacy_config_is_a_key_error(cfg):
    with pytest.raises(KeyError):
        cfg.bay_limit("b01")


def test_topology_from_config_builds_the_simulated_structure(das_example_cfg):
    cfg = das_example_cfg
    topo = topology_from_config(cfg)
    assert set(topo["zones"]) == set(cfg.topology.zones)
    assert topo["zones"]["z1"]["coupled_to"] == ["z0", "z2"]
    assert set(topo["bays"]) == set(cfg.topology.bays)
    assert topo["bays"]["b01"] == {"zone": "z0", "occupied": True, "class": "hdd"}
    assert topo["fans"]["qd1"]["zones"] == {"z0": 0.5, "z1": 0.5}
    assert topo["fans"]["xt1"]["count"] == 2 and topo["fans"]["xt1"]["rpm_max"] == 1500.0
    types = {name: entry["type"] for name, entry in topo["sensors"].items()}
    assert types["air_z0"] == "thermistor" and types["prox_b01"] == "ds18b20"
    assert topo["sensors"]["prox_b03b"] == {
        "role": "drive_proximal",
        "type": "thermistor",
        "zone": "z0",
        "bay": "b03",
    }
    params = build_das_params(topo, dt=cfg.dt)
    assert params.channels == cfg.channels and set(params.sensor_names) == set(cfg.temps)


def test_topology_from_config_empty_bay_and_legacy(cfg):
    m = das_mapping()
    m["topology"]["bays"]["c1"]["serial"] = "SN1"
    topo = topology_from_config(MpcConfig.from_mapping(m))
    assert topo["bays"]["c1"] == {"zone": "zc", "occupied": False, "class": "hdd", "serial": "SN1"}
    with pytest.raises(ValueError):
        topology_from_config(cfg)
