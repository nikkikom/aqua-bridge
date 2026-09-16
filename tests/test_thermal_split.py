"""Per-channel split of a fan group's effectiveness (PROJECT.md section 8 item 13).

With ``mpc.model_split_channels`` each fan group of more than one channel in a zone
carries one extra air-block coefficient per channel beyond the first,
``Es.<zone>.<group>.<channel>``, whose regressor ``phi_ch - phi_zG`` is exactly zero
while the group's channels hold the same duty. So regulation and a whole-group
experiment leave the shared-``E`` model untouched term for term, an experiment's
single-channel phases are the only thing that moves the split, and what it learns
redistributes the group's coefficient without changing its total.

The evidence here is three-layered: the structure and the algebra (this is a
reparameterisation, not a new model), the identification on the DAS truth simulator
(single-channel phases find a group whose two channels differ by 60 %), and the model
store (a file written with the switch the other way is converted, not dropped).
"""

from __future__ import annotations

import dataclasses
import json
from collections import Counter
from typing import Any

import numpy as np
import pytest

from aqua_bridge.control import persist, thermal
from aqua_bridge.model import ConfigError, MpcConfig
from aqua_bridge.sim.das import DasPlant, build_das_plant, topology_from_config
from das_fixtures import das_cfg

# das_fixtures: zone za is served by the group "front" (fa1 count 2, fa2 count 1)
SPLIT_KEYS = ("Es.za.front.fa2",)


def split_cfg(**changes: Any) -> MpcConfig:
    return das_cfg(model_shadow=True, model_split_channels=True, **changes)


# ---------------------------------------------------------------------------
# config and structure
# ---------------------------------------------------------------------------


def test_the_switch_needs_a_zoned_config(cfg: MpcConfig) -> None:
    with pytest.raises(ConfigError, match="requires mpc.topology"):
        dataclasses.replace(cfg, model_split_channels=True)


def test_only_a_multi_channel_group_gets_split_keys() -> None:
    plain = thermal.structure(das_cfg(model_shadow=True))
    assert all(not gr.splits for zone in plain.zones.values() for gr in zone.groups)
    assert not any(k.startswith("Es.") for k in thermal.parameter_keys(plain))

    st = thermal.structure(split_cfg())
    groups = {gr.key: gr for zone in st.zones.values() for gr in zone.groups}
    assert [k for k, _ in groups["E.za.front"].splits] == list(SPLIT_KEYS)
    assert groups["E.zb.fb1"].splits == ()  # one channel: nothing to split
    # the reference channel keeps no coefficient of its own, so the split has m-1 of them
    assert len(groups["E.za.front"].splits) == len(groups["E.za.front"].weights) - 1
    assert thermal.prior_theta(split_cfg())["Es.za.front.fa2"] == 0.0


def test_the_split_keys_sit_between_the_groups_and_leak() -> None:
    st = thermal.structure(split_cfg())
    keys = st.zones["za"].air_keys
    # za also carries the weak cross-zone group fb1 (zb is coupled to za)
    assert keys == (
        "E.za.front",
        "E.za.fb1",
        "Es.za.front.fa2",
        "leak.za",
        "kappa.za.zb",
        "p_air.za",
    )


# ---------------------------------------------------------------------------
# the algebra: a reparameterisation, not a new model
# ---------------------------------------------------------------------------


def test_a_group_at_its_prior_split_is_the_shared_e_model_bit_for_bit() -> None:
    plain, split = das_cfg(model_shadow=True), split_cfg()
    st_p, st_s = thermal.structure(plain), thermal.structure(split)
    p_p = thermal.model_params(plain, st=st_p)
    p_s = thermal.model_params(split, st=st_s)
    x = np.array([30.0, 31.0, 32.0, 40.0, 41.0, 42.0, 43.0, 38.0, 39.0, 37.0, 36.0])
    for u_value in (0.0, 0.25, 0.6, 1.0):
        u = dict.fromkeys(plain.channels, u_value)
        t_in = dict.fromkeys(st_p.zones, 24.0)
        f_p = thermal.derivatives(st_p, p_p, x, u, t_in=t_in)
        f_s = thermal.derivatives(st_s, p_s, x, u, t_in=t_in)
        assert f_s.tobytes() == f_p.tobytes(), u_value
        j_p = thermal.jacobians(st_p, p_p, x, u, t_in=t_in)
        j_s = thermal.jacobians(st_s, p_s, x, u, t_in=t_in)
        assert j_s.a.tobytes() == j_p.a.tobytes()
        assert j_s.b.tobytes() == j_p.b.tobytes()


def test_a_split_redistributes_the_group_and_never_changes_its_total() -> None:
    split = split_cfg()
    st = thermal.structure(split)
    group = next(gr for gr in st.zones["za"].groups if gr.name == "front")
    theta = dict(thermal.prior_theta(split, st))
    at_prior = thermal.channel_effectiveness(group, theta)
    assert sum(at_prior.values()) == pytest.approx(theta["E.za.front"])
    assert at_prior["fa1"] == pytest.approx(2 * at_prior["fa2"])  # count 2 against count 1

    theta["Es.za.front.fa2"] = 12.0
    moved = thermal.channel_effectiveness(group, theta)
    assert sum(moved.values()) == pytest.approx(theta["E.za.front"])
    assert moved["fa2"] > at_prior["fa2"] and moved["fa1"] < at_prior["fa1"]


def test_the_split_only_bites_when_the_group_members_differ() -> None:
    split = split_cfg()
    st = thermal.structure(split)
    theta = dict(thermal.prior_theta(split, st))
    theta["Es.za.front.fa2"] = 15.0
    params = thermal.model_params(split, theta, st=st)
    together = thermal._airflow(st, params, np.array([0.6, 0.6, 0.6, 0.6]))[0][st.i_air("za")]
    plain = das_cfg(model_shadow=True)
    st_p = thermal.structure(plain)
    baseline = thermal._airflow(
        st_p, thermal.model_params(plain, st=st_p), np.array([0.6, 0.6, 0.6, 0.6])
    )[0][st_p.i_air("za")]
    assert together == baseline  # the regressors are exactly 0 under common motion
    apart = thermal._airflow(st, params, np.array([0.9, 0.3, 0.6, 0.6]))[0][st.i_air("za")]
    assert apart != baseline


def test_the_jacobian_of_a_split_group_matches_finite_differences() -> None:
    split = split_cfg()
    st = thermal.structure(split)
    theta = dict(thermal.prior_theta(split, st))
    theta["Es.za.front.fa2"] = 9.0
    params = thermal.model_params(split, theta, st=st)
    x = np.array([30.0, 31.0, 32.0, 40.0, 41.0, 42.0, 43.0, 38.0, 39.0, 37.0, 36.0])
    u = {"fa1": 0.7, "fa2": 0.4, "fb1": 0.5, "fc1": 0.55}
    t_in = dict.fromkeys(st.zones, 24.0)
    lin = thermal.jacobians(st, params, x, u, t_in=t_in)
    eps = 1e-6
    for j, ch in enumerate(st.channels):
        bumped = dict(u)
        bumped[ch] += eps
        numeric = (thermal.derivatives(st, params, x, bumped, t_in=t_in) - lin.f) / eps
        assert numeric == pytest.approx(lin.b[:, j], abs=1e-5), ch


# ---------------------------------------------------------------------------
# identification on the truth simulator
# ---------------------------------------------------------------------------


def truth_topology(cfg: MpcConfig, *, fa1: float, fa2: float) -> dict[str, Any]:
    """The DAS truth with the two channels of the ``front`` group deliberately unequal."""
    topo = topology_from_config(cfg)
    for ch, fan in topo["fans"].items():
        listed = [z for z, spec in cfg.topology.zones.items() if ch in spec.channels]
        fan["zones"] = dict.fromkeys(listed, 1.0 / len(listed))
        fan["e_w_per_k"] = {"fa1": fa1, "fa2": fa2}.get(ch, 33.0)
    for sensor in topo["sensors"].values():
        sensor["noise_sigma_c"] = 0.0
    return topo


def run_phases(
    cfg: MpcConfig, plant: DasPlant, phases: list[tuple[float, dict[str, float]]], memory: Any
) -> tuple[Any, dict[str, Any], Counter[str]]:
    """Drive the fans open-loop through ``(seconds, pwm per channel)`` phases, feeding
    ``thermal.update`` the way ``step`` does."""
    statuses: Counter[str] = Counter()
    summary: dict[str, Any] = {}
    zones_ok = set(cfg.topology.zones)
    applied = dict(cfg.fallback_pwm)
    for seconds, level in phases:
        until = plant.ts + seconds
        while plant.ts < until:
            obs = plant.observe()
            temps = {name: v for name, v in obs.temps.items() if v is not None}
            out = thermal.update(
                memory, cfg, temps=temps, u=applied, ts=plant.ts, zones_ok=zones_ok
            )
            memory, summary = out.memory, out.summary
            statuses[summary["status"]] += 1
            plant.apply(level)
            applied = dict(level)
            plant.advance()
    return memory, summary, statuses


def telegraph(
    channels: tuple[str, ...], moving: tuple[str, ...], base: float, high: float, holds: int
) -> list[tuple[float, dict[str, float]]]:
    """Two-level phases: ``moving`` alternates, every other channel holds ``base``."""
    out = []
    for i in range(holds):
        level = dict.fromkeys(channels, base)
        for ch in moving:
            level[ch] = high if i % 2 else base
        out.append((120.0, level))
    return out


@pytest.fixture(scope="module")
def split_experiment() -> tuple[MpcConfig, dict[str, Any], float, float]:
    """Whole-group phases first, then each channel of ``front`` alone -- the experiment
    ``control/ident.py`` runs -- against a truth where ``fa1`` moves 60 % more air per
    fan than ``fa2``."""
    fa1, fa2 = 42.0, 26.0
    cfg = split_cfg(model_window_s=30.0)
    plant = build_das_plant(
        truth_topology(cfg, fa1=fa1, fa2=fa2), preset="basic", seed=3, dt=cfg.dt, initial_pwm=0.5
    )
    channels = cfg.channels
    phases = telegraph(channels, ("fa1", "fa2"), 0.35, 0.8, 20)
    phases += telegraph(channels, ("fa1",), 0.35, 0.8, 24)
    phases += telegraph(channels, ("fa2",), 0.35, 0.8, 24)
    memory, summary, _ = run_phases(cfg, plant, phases, None)
    return cfg, summary, fa1, fa2


def test_single_channel_phases_split_the_group_the_right_way(split_experiment) -> None:
    """The truth is 84 W/K on fa1 (42 per fan, count 2) against 26 on fa2, a ratio of
    3.23 where the count-weighted prior says 2.0. Measured on this seed: 78.6 / 28.6,
    ratio 2.75 -- about 70 % of the way from the prior to the truth, with the group's
    total 107 against 110."""
    _, summary, fa1, fa2 = split_experiment
    per_channel = summary["zones"]["za"]["e_per_channel"]
    truth = {"fa1": 2 * fa1, "fa2": fa2}  # fa1 has count 2 in the fixture, fa2 count 1
    assert per_channel["fa1"] == pytest.approx(truth["fa1"], rel=0.15)
    assert per_channel["fa2"] == pytest.approx(truth["fa2"], rel=0.2)
    # the count-weighted prior split (2:1) is left well behind
    assert per_channel["fa1"] / per_channel["fa2"] > 2.5
    assert summary["zones"]["za"]["theta"]["Es.za.front.fa2"] < -5.0
    # the group's total is what the shared coefficient already had
    group = summary["zones"]["za"]["theta"]["E.za.front"]
    assert per_channel["fa1"] + per_channel["fa2"] == pytest.approx(group)
    assert group == pytest.approx(truth["fa1"] + truth["fa2"], rel=0.15)


def test_a_whole_group_experiment_leaves_the_split_at_its_prior() -> None:
    """Only single-channel phases carry the split: with the group moving together the
    regressor is 0 and the ridge holds the coefficient at 0."""
    cfg = split_cfg(model_window_s=30.0)
    plant = build_das_plant(
        truth_topology(cfg, fa1=42.0, fa2=26.0), preset="basic", seed=3, dt=cfg.dt, initial_pwm=0.5
    )
    phases = telegraph(cfg.channels, ("fa1", "fa2"), 0.35, 0.8, 40)
    _, summary, statuses = run_phases(cfg, plant, phases, None)
    assert statuses["error"] == 0
    assert summary["zones"]["za"]["theta"]["Es.za.front.fa2"] == pytest.approx(0.0, abs=1e-9)
    per_channel = summary["zones"]["za"]["e_per_channel"]
    assert per_channel["fa1"] == pytest.approx(2 * per_channel["fa2"])  # the prior split


# ---------------------------------------------------------------------------
# the model store keeps working across the switch
# ---------------------------------------------------------------------------


def test_a_file_written_without_the_split_is_converted_not_dropped() -> None:
    plain = das_cfg(model_shadow=True)
    saved = thermal.fresh_memory(plain, status="converged")
    saved["zones"]["za"]["air"]["theta"] = [40.0, 2.0, 1.2, 3.1, 5.0]
    saved["zones"]["za"]["air"]["P"] = (np.eye(5) * 0.3).tolist()
    stored = json.loads(json.dumps(saved))

    split = split_cfg()
    restored = thermal.restore(stored, split, stale=False)
    keys = thermal.structure(split).zones["za"].air_keys
    theta = dict(zip(keys, restored["zones"]["za"]["air"]["theta"], strict=True))
    assert theta == {
        "E.za.front": 40.0,
        "E.za.fb1": 2.0,
        "Es.za.front.fa2": 0.0,  # the new key starts at its prior: the same model
        "leak.za": 1.2,
        "kappa.za.zb": 3.1,
        "p_air.za": 5.0,
    }
    p = np.array(restored["zones"]["za"]["air"]["P"])
    assert p.shape == (6, 6)
    assert p[0, 0] == 0.3 and p[3, 3] == 0.3  # the kept keys keep their variance
    assert p[2, 2] == pytest.approx(0.25) and not p[2, 0] and not p[0, 2]
    assert restored["zones"]["za"]["status"] == "frozen"


def test_a_file_written_with_the_split_loads_with_the_switch_off() -> None:
    split = split_cfg()
    saved = thermal.fresh_memory(split, status="converged")
    saved["zones"]["za"]["air"]["theta"] = [40.0, 2.0, -6.0, 1.2, 3.1, 5.0]
    stored = json.loads(json.dumps(saved))

    plain = das_cfg(model_shadow=True)
    restored = thermal.restore(stored, plain, stale=False)
    keys = thermal.structure(plain).zones["za"].air_keys
    theta = dict(zip(keys, restored["zones"]["za"]["air"]["theta"], strict=True))
    assert theta == {
        "E.za.front": 40.0,
        "E.za.fb1": 2.0,
        "leak.za": 1.2,
        "kappa.za.zb": 3.1,
        "p_air.za": 5.0,
    }


def test_a_file_of_another_structure_is_still_refused() -> None:
    split = split_cfg()
    saved = thermal.fresh_memory(das_cfg(model_shadow=True), status="converged")
    saved["fp"] = "some other structure"
    with pytest.raises(ValueError, match="structure differs"):
        thermal.restore(saved, split, stale=False)


def test_the_converted_seed_goes_through_apply_seed() -> None:
    plain = das_cfg(model_shadow=True)
    saved = thermal.fresh_memory(plain, status="converged")
    seed = {"source": "fresh", "thermal": json.loads(json.dumps(saved))}
    mem: dict[str, Any] = {}
    split = split_cfg()
    summary = persist.apply_seed(mem, split, seed, 0.0)
    assert summary["sections"]["thermal"] == "loaded", summary["warnings"]
    assert thermal.model_status(mem["thermal"]) == "frozen"
