"""Drive estimates block and the prior-map provider (``aqua_bridge.control.estimates``).

Plan sections 2 (proximal sensor model, uncalibrated sigma), 4 (soft / hard
targets per drive class) and 6 (``SolverRequest.estimates``). Uses the small
zoned config of :mod:`das_fixtures` with ``setpoints`` removed.
"""

from __future__ import annotations

import json
import math

import pytest
from hypothesis import given
from hypothesis import strategies as st

from aqua_bridge.control import estimates as est
from aqua_bridge.model import ESTIMATOR_DEFAULTS, MpcConfig
from das_fixtures import PROX_C, SP, das_cfg, das_mapping

BETA = est.PRIOR_BETA


@pytest.fixture
def lcfg() -> MpcConfig:
    """The zones fixture without setpoints: DAS limit regulation."""
    return das_cfg(setpoints={})


def trusted(cfg: MpcConfig, **overrides: float) -> dict[str, float]:
    temps = {}
    for name in cfg.temps:
        role = cfg.sensors[name].role
        temps[name] = {"zone_air": SP, "drive_proximal": PROX_C, "inlet": 25.0}.get(role, 38.0)
    temps.update(overrides)
    return temps


def test_constants_follow_the_plan(lcfg):
    assert est.PRIOR_BETA == 0.3
    assert pytest.approx(-2.1) == est.PRIOR_OFFSET_C
    assert est.K_SIGMA == 2.0
    # the uncalibrated floor is a config key with one default (item 71)
    assert ESTIMATOR_DEFAULTS["sigma_uncalibrated_c"] == 1.5
    assert est.sigma_uncalibrated_c(lcfg) == lcfg.estimator.sigma_uncalibrated_c == 1.5


@given(
    t_drive=st.floats(min_value=10.0, max_value=90.0),
    t_air=st.floats(min_value=10.0, max_value=60.0),
)
def test_prior_map_inverts_the_sensor_model(t_drive, t_air):
    """T_s = (1 - beta) T_d + beta T_a + b  ->  prior_drive_temp(T_s, T_a) == T_d."""
    t_s = (1.0 - BETA) * t_drive + BETA * t_air + est.PRIOR_OFFSET_C
    assert est.prior_drive_temp(t_s, t_air) == pytest.approx(t_drive, abs=1e-9)


def test_prior_map_is_monotone_the_conservative_way():
    base = est.prior_drive_temp(40.0, 35.0)
    assert est.prior_drive_temp(41.0, 35.0) > base  # hotter sensor -> hotter drive
    assert est.prior_drive_temp(40.0, 36.0) < base  # hotter air -> less drive excess


def test_drive_targets():
    soft, hard = est.drive_targets(50.0, 5.0, 1.5, 2.0)
    assert (soft, hard) == (42.0, 47.0)
    assert est.drive_targets(65.0, 10.0, 0.0, 2.0) == (55.0, 65.0)


def test_block_for_the_fixture(lcfg):
    block = est.prior_estimates(lcfg, trusted(lcfg))
    # c1 is occupied: false -> no constraint, no entry; a1 / b1 are auto -> unknown.
    assert list(block) == ["a1", "a2", "b1"]
    t = est.prior_drive_temp(PROX_C, SP)
    a1 = block["a1"]
    assert a1["t"] == pytest.approx(t)
    assert a1 == {
        "zone": "za",
        "class": "hdd",
        "occupancy": "unknown",
        "t": a1["t"],
        "sigma": 1.5,
        "k_sigma": 2.0,
        "margin": 3.0,
        "limit": 50.0,
        "comfort": 5.0,
        "soft": 42.0,
        "hard": 47.0,
        "source": "prior_map",
        "calibrated": False,
    }
    a2 = block["a2"]
    assert (a2["class"], a2["occupancy"], a2["soft"], a2["hard"]) == (
        "ssd_sata",
        "occupied",
        52.0,
        62.0,
    )
    # b1 declares no class: topology.default_class, the strictest (hdd).
    assert block["b1"]["class"] == "hdd" and block["b1"]["zone"] == "zb"
    json.dumps(block, allow_nan=False)


def test_hottest_proximal_and_coolest_air_are_used(lcfg):
    base = est.prior_estimates(lcfg, trusted(lcfg))["a1"]["t"]
    hot_backup = est.prior_estimates(lcfg, trusted(lcfg, prox_a1b=PROX_C + 2.0))["a1"]["t"]
    cold_backup = est.prior_estimates(lcfg, trusted(lcfg, prox_a1b=PROX_C - 2.0))["a1"]["t"]
    assert hot_backup == pytest.approx(est.prior_drive_temp(PROX_C + 2.0, SP))
    assert cold_backup == pytest.approx(base)
    cool_air = est.prior_estimates(lcfg, trusted(lcfg, air_a2=SP - 3.0))["a1"]["t"]
    assert cool_air == pytest.approx(est.prior_drive_temp(PROX_C, SP - 3.0))
    assert cool_air > base


def test_untrusted_members_are_simply_absent(lcfg):
    temps = trusted(lcfg)
    del temps["prox_a1"]
    assert "a1" in est.prior_estimates(lcfg, temps)  # the redundant member stands in
    del temps["prox_a1b"]
    assert "a1" not in est.prior_estimates(lcfg, temps)
    temps = trusted(lcfg)
    del temps["air_b"]
    assert "b1" not in est.prior_estimates(lcfg, temps)


def test_zone_filter(lcfg):
    assert list(est.prior_estimates(lcfg, trusted(lcfg), {"zb"})) == ["b1"]
    assert est.prior_estimates(lcfg, trusted(lcfg), ()) == {}


def test_bay_limit_can_only_tighten_the_class():
    m = das_mapping()
    m["setpoints"] = {}
    m["topology"]["bays"]["a1"]["limit_c"] = 45.0
    m["topology"]["bays"]["a2"]["limit_c"] = 80.0  # above ssd_sata's 65: ignored
    cfg = MpcConfig.from_mapping(m)
    assert cfg.bay_limit("a1") == 45.0 and cfg.bay_limit("a2") == 65.0
    block = est.prior_estimates(cfg, trusted(cfg))
    assert (block["a1"]["limit"], block["a1"]["soft"], block["a1"]["hard"]) == (45.0, 37.0, 42.0)
    assert block["a2"]["limit"] == 65.0


def test_legacy_config_has_no_block(cfg):
    assert est.prior_estimates(cfg, {"coolant": 35.0, "air": 30.0}) == {}


def test_block_is_deterministic_and_finite(lcfg):
    temps = trusted(lcfg, prox_a2=55.5, air_b=31.25)
    a = est.prior_estimates(lcfg, temps)
    b = est.prior_estimates(lcfg, dict(reversed(list(temps.items()))))
    assert a == b
    assert all(math.isfinite(v) for e in a.values() for v in e.values() if isinstance(v, float))
