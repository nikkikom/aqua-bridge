"""Model store (plan section 6 *Persistence*, milestone 10): ``modelstore.py`` and
``control/persist.py``.

File level: missing, corrupt, truncated, non-finite, wrong schema, wrong fingerprint, age
(fresh, stale, clock behind the file), atomic write, the persister's interval and clean
shutdown. Content level (applied by ``mpc.step`` on its first zoned tick): coefficient
bounds and covariances, calibration entries keyed by bay and serial with their ages
re-based on the new clock, and the owner's stale rule -- a stale model is re-confirmed in
shadow while the PI-like DAS form acts, and acts only after converging for
``model_reconfirm_s`` with its prediction error in bounds; a fresh one loads ``frozen``
and acts at once.
"""

from __future__ import annotations

import copy
import dataclasses
import json
import logging
import math
import os
from pathlib import Path
from types import SimpleNamespace
from typing import Any

import pytest
import yaml
from hypothesis import given, settings
from hypothesis import strategies as st

from aqua_bridge import __main__ as main_mod
from aqua_bridge import modelstore
from aqua_bridge.config import load_config
from aqua_bridge.control import estimates as est_prior
from aqua_bridge.control import estimator, persist, thermal
from aqua_bridge.control.mpc import step
from aqua_bridge.model import (
    STORE_KEY,
    STORE_SEED_KEY,
    ConfigError,
    MpcConfig,
    MpcState,
    SolverKind,
)
from conftest import EXAMPLE_CONFIG, EXAMPLE_DAS_CONFIG
from das_fixtures import das_cfg, das_obs
from invariants import checked_step

DAY = 86400.0
NOW = 1_800_000_000.0


# ---------------------------------------------------------------------------
# helpers
# ---------------------------------------------------------------------------


def das_mapping() -> dict[str, Any]:
    return copy.deepcopy(yaml.safe_load(EXAMPLE_DAS_CONFIG.read_text())["mpc"])


def shadow_cfg(**changes: Any) -> MpcConfig:
    """The DAS example with shadow learning and the DAS MPC."""
    base = load_config(EXAMPLE_DAS_CONFIG).mpc
    return dataclasses.replace(base, **{"solver": SolverKind.MPC, "model_shadow": True, **changes})


def temps_for(cfg: MpcConfig, i: int, drive: float = 38.0) -> dict[str, float]:
    """Proximal readings for a drive temperature, zone air flickering by two LSBs."""
    out: dict[str, float] = {}
    prox = (1.0 - est_prior.PRIOR_BETA) * drive + est_prior.PRIOR_BETA * 35.0
    for name in cfg.temps:
        spec = cfg.sensors[name]
        if spec.role == "drive_proximal":
            out[name] = prox + est_prior.PRIOR_OFFSET_C
        elif spec.role == "zone_air":
            out[name] = 35.0 + 0.02 * (i % 2)
    return out


def run(
    cfg: MpcConfig, state: MpcState, ticks: int, *, t0: float = 0.0, checked: bool = True
) -> tuple[Any, MpcState, list[Any]]:
    cmds = []
    cmd = None
    for i in range(ticks):
        ts = t0 + i * cfg.dt
        prev = 0.5 if state.last_cmd is None else state.last_cmd.pwm
        obs = das_obs(cfg, ts, pwm=prev, **temps_for(cfg, i))
        cmd, state = (checked_step if checked else step)(obs, cfg, state)
        cmds.append(cmd)
    return cmd, state, cmds


def cal_entry(**changes: Any) -> dict[str, Any]:
    """A calibration entry the estimator accepts as calibrated (in memory form)."""
    entry = {
        "th": [0.8, -1.5],
        "P": [[0.004, 0.0], [0.0, 0.5]],
        "n": 40,
        "fresh": 30,
        "rms2": 0.36,
        "ts": None,
        "used": True,
    }
    entry.update(changes)
    return entry


def stored_doc(cfg: MpcConfig, *, saved_wall: float, **sections: Any) -> dict[str, Any]:
    doc = {
        "schema": modelstore.SCHEMA,
        "v": modelstore.SCHEMA_VERSION,
        "fingerprint": modelstore.fingerprint(cfg),
        "saved_wall": saved_wall,
        "thermal": None,
        "fan_curves": {},
        "calibration": {},
        "bays": {},
    }
    doc.update(sections)
    return doc


def write_doc(path: Path, doc: dict[str, Any]) -> Path:
    path.write_text(json.dumps(doc))
    return path


def load_state(path: Path, cfg: MpcConfig, now: float = NOW) -> tuple[Any, MpcState]:
    result = modelstore.load(path, cfg, now_wall=now)
    return result, modelstore.initial_state(result)


# ---------------------------------------------------------------------------
# config keys, path, fingerprint
# ---------------------------------------------------------------------------


def test_store_config_keys_have_defaults_and_rules():
    cfg = load_config(EXAMPLE_DAS_CONFIG).mpc
    assert (cfg.model_store_interval_s, cfg.model_store_max_age_days, cfg.model_reconfirm_s) == (
        600.0,
        30.0,
        3600.0,
    )
    for key, bad in (
        ("model_store_interval_s", 0),
        ("model_store_max_age_days", -1),
        ("model_reconfirm_s", -5),
    ):
        data = das_mapping()
        data[key] = bad
        with pytest.raises(ConfigError, match=key):
            MpcConfig.from_mapping(data)
    data = das_mapping()
    data["model_reconfirm_s"] = 0
    assert MpcConfig.from_mapping(data).model_reconfirm_s == 0.0


def test_store_path_cli_env_and_legacy(cfg):
    das = load_config(EXAMPLE_DAS_CONFIG).mpc
    assert modelstore.store_path(das, "/x/m.json", {}) == Path("/x/m.json")
    assert modelstore.store_path(das, None, {"STATE_DIRECTORY": "/var/lib/aqua-bridge"}) == Path(
        "/var/lib/aqua-bridge/model.json"
    )
    assert modelstore.store_path(das, None, {"STATE_DIRECTORY": "/a:/b"}) == Path("/a/model.json")
    assert modelstore.store_path(das, None, {}) is None
    # legacy: the environment is ignored, the flag is a configuration error
    assert modelstore.store_path(cfg, None, {"STATE_DIRECTORY": "/var/lib/aqua-bridge"}) is None
    with pytest.raises(ConfigError, match="model-store"):
        modelstore.store_path(cfg, "/x/m.json", {})
    assert main_mod.build_model_store(cfg, None, env={"STATE_DIRECTORY": "/tmp"}) == (None, None)


def test_fingerprint_covers_structure_not_policy():
    base = MpcConfig.from_mapping(das_mapping())
    fp = modelstore.fingerprint(base)
    policy = das_mapping()
    policy["topology"]["bays"]["b01"]["limit_c"] = 45.0
    policy["drive_classes"] = {
        "hdd": {"limit_c": 48.0, "comfort_c": 5.0, "tau_d_s": 720.0},
        "ssd_sata": {"limit_c": 65.0, "comfort_c": 10.0, "tau_d_s": 200.0},
        "nvme": {"limit_c": 70.0, "comfort_c": 10.0, "tau_d_s": 120.0},
    }
    policy["pi_kp"] = 0.03
    assert modelstore.fingerprint(MpcConfig.from_mapping(policy)) == fp
    fan = next(iter(das_mapping()["fans"]))
    for change in (
        lambda d: d.update(dt=4.0),
        lambda d: (
            d["topology"]["bays"]["b01"].update(zone="z1"),
            d["sensors"]["prox_b01"].update(zone="z1"),
        ),
        lambda d: d["fans"][fan].update(count=3),
        lambda d: d["fans"][fan].update(group="shared"),
    ):
        data = das_mapping()
        change(data)
        assert modelstore.fingerprint(MpcConfig.from_mapping(data)) != fp


# ---------------------------------------------------------------------------
# load: file level
# ---------------------------------------------------------------------------


def test_missing_file_starts_on_the_prior(tmp_path):
    cfg = shadow_cfg()
    result, state = load_state(tmp_path / "model.json", cfg)
    assert result.missing and result.source == "prior" and result.warnings == []
    cmd, state, _ = run(cfg, state, 1)
    store = cmd.diagnostics["store"]
    assert store["source"] == "prior" and store["warnings"] == []
    assert store["sections"]["thermal"] == "absent"
    assert cmd.diagnostics["thermal"]["status"] == "prior"
    assert STORE_SEED_KEY not in state.solver_memory


@pytest.mark.parametrize(
    "content",
    [
        b"",
        b"{not json",
        b"\xff\xfe garbage",
        b"[1, 2, 3]",
        b'{"schema": "aqua-bridge-model-store", "v": 1, "saved_wall": NaN}',
        b'{"schema": "aqua-bridge-model-store", "v": 1, "saved_wall": Infinity}',
    ],
    ids=["empty", "not-json", "binary", "not-an-object", "nan", "infinity"],
)
def test_corrupt_files_load_the_prior_with_a_warning(tmp_path, content):
    cfg = shadow_cfg()
    path = tmp_path / "model.json"
    path.write_bytes(content)
    result, state = load_state(path, cfg)
    assert result.source == "prior" and len(result.warnings) == 1
    cmd, _, _ = run(cfg, state, 1)
    assert cmd.diagnostics["store"]["source"] == "prior"
    assert cmd.diagnostics["store"]["warnings"] == result.warnings
    assert cmd.diagnostics["thermal"]["status"] == "prior"


def test_a_truncated_file_loads_the_prior(tmp_path):
    cfg = shadow_cfg()
    doc = stored_doc(
        cfg, saved_wall=NOW - 60, thermal=thermal.fresh_memory(cfg, status="converged")
    )
    text = json.dumps(doc)
    for cut in (len(text) // 3, len(text) // 2, len(text) - 1):
        path = tmp_path / f"model{cut}.json"
        path.write_text(text[:cut])
        result = modelstore.load(path, cfg, now_wall=NOW)
        assert result.source == "prior" and "corrupt" in result.warnings[0]


# ---------------------------------------------------------------------------
# a tools/fit_model.py report in the store's place (PROJECT.md section 8 item 15)
# ---------------------------------------------------------------------------


def fit_report(cfg: MpcConfig, *, generated_at: float, **changes: Any) -> dict[str, Any]:
    """A ``tools/fit_model.py`` report, cut down to what the loader reads."""
    doc = {
        "v": modelstore.FIT_VERSION,
        "kind": modelstore.FIT_KIND,
        "fingerprint": thermal.cached_structure(cfg).fingerprint,
        "store_fingerprint": modelstore.fingerprint(cfg),
        "memory": thermal.fresh_memory(cfg, status="converged"),
        "generated_at": generated_at,
    }
    doc.update(changes)
    return doc


def test_a_fit_report_loads_as_a_model_with_a_warning(tmp_path):
    cfg = shadow_cfg()
    path = write_doc(tmp_path / "model.json", fit_report(cfg, generated_at=NOW - 60))
    result, state = load_state(path, cfg)
    assert result.source == "fresh" and result.age_s == pytest.approx(60.0)
    assert len(result.warnings) == 1 and "fit_model.py report" in result.warnings[0]
    cmd, _, _ = run(cfg, state, 1)
    store = cmd.diagnostics["store"]
    assert store["source"] == "fresh" and store["sections"]["thermal"] == "loaded"
    # the other sections are empty: a fit knows nothing about this machine's calibrations
    assert store["sections"]["calibration"] == 0 and store["sections"]["bays"] == 0
    assert cmd.diagnostics["thermal"]["status"] == "frozen"


def test_an_old_fit_report_loads_stale_like_an_old_store_file(tmp_path):
    cfg = shadow_cfg()
    doc = fit_report(cfg, generated_at=NOW - 40 * DAY)
    result, state = load_state(write_doc(tmp_path / "model.json", doc), cfg)
    assert result.source == "stale"
    cmd, _, _ = run(cfg, state, 1)
    assert cmd.diagnostics["thermal"]["status"] == "stale"


def test_a_fit_report_for_another_structure_drops_its_model(tmp_path):
    cfg = shadow_cfg()
    other = shadow_cfg(dt=4.0)
    doc = fit_report(cfg, generated_at=NOW - 60, memory=thermal.fresh_memory(other))
    doc["memory"]["fp"] = "not this structure"
    result, state = load_state(write_doc(tmp_path / "model.json", doc), cfg)
    assert result.source == "fresh"
    cmd, _, _ = run(cfg, state, 1)
    store = cmd.diagnostics["store"]
    assert store["sections"]["thermal"] == "dropped"
    assert any("structure differs" in w for w in store["warnings"]), store["warnings"]
    assert cmd.diagnostics["thermal"]["status"] == "prior"


def _fans_with_count(cfg: MpcConfig, channel: str, count: int) -> dict[str, Any]:
    fans = dict(cfg.fans)
    fans[channel] = dataclasses.replace(fans[channel], count=count)
    return fans


def test_a_fit_report_of_another_config_is_refused_on_the_store_fingerprint(tmp_path):
    """The workflow item 15 documents -- copy the tool's model.json to $STATE_DIRECTORY --
    must be as safe as a store file: the report carries the store's own structure
    fingerprint, so a fit for a machine with a different fan count, sensor role or dt is
    refused here, not half-checked by the thermal model's own fingerprint (which covers
    neither)."""
    cfg = shadow_cfg()
    other = shadow_cfg(fans=_fans_with_count(cfg, next(iter(cfg.fans)), 4))
    assert thermal.cached_structure(other).fingerprint == thermal.cached_structure(cfg).fingerprint
    assert modelstore.fingerprint(other) != modelstore.fingerprint(cfg)
    doc = fit_report(other, generated_at=NOW - 60)
    result = modelstore.load(write_doc(tmp_path / "model.json", doc), cfg, now_wall=NOW)
    assert result.source == "prior" and "thermal" not in (result.seed or {})
    assert "another config structure" in result.warnings[0], result.warnings


@pytest.mark.parametrize(
    ("change", "needle"),
    [
        ({"v": 2}, "its version is 2"),
        ({"memory": "not a mapping"}, "no thermal memory"),
        ({"generated_at": None}, "no finite generated_at"),
        ({"store_fingerprint": None}, "no store_fingerprint"),
        ({"store_fingerprint": "0" * 64}, "another config structure"),
    ],
    ids=["version", "memory", "generated_at", "no-fingerprint", "other-fingerprint"],
)
def test_a_malformed_fit_report_loads_the_prior(tmp_path, change, needle):
    cfg = shadow_cfg()
    doc = {**fit_report(cfg, generated_at=NOW - 60), **change}
    result = modelstore.load(write_doc(tmp_path / "model.json", doc), cfg, now_wall=NOW)
    assert result.source == "prior" and len(result.warnings) == 1
    assert needle in result.warnings[0], result.warnings


def test_document_from_fit_keeps_the_fit_time_unless_told_otherwise():
    cfg = shadow_cfg()
    report = fit_report(cfg, generated_at=NOW - 3600.0)
    doc = modelstore.document_from_fit(cfg, report)
    assert doc["saved_wall"] == NOW - 3600.0
    assert doc["fingerprint"] == modelstore.fingerprint(cfg)
    assert doc["fan_curves"] == {} and doc["calibration"] == {} and doc["bays"] == {}
    assert modelstore.document_from_fit(cfg, report, wall=NOW)["saved_wall"] == NOW
    with pytest.raises(ValueError, match="not an aqua_bridge.thermal_model report"):
        modelstore.document_from_fit(cfg, {"kind": "something else"})


def test_wrong_schema_version_or_fingerprint_loads_the_prior(tmp_path):
    cfg = shadow_cfg()
    for change, needle in (
        ({"schema": "something-else"}, "unknown schema"),
        ({"v": 2}, "unknown schema"),
        ({"fingerprint": "0" * 64}, "another config"),
        ({"saved_wall": "yesterday"}, "saved_wall"),
    ):
        doc = {**stored_doc(cfg, saved_wall=NOW - 60), **change}
        result = modelstore.load(write_doc(tmp_path / "m.json", doc), cfg, now_wall=NOW)
        assert result.source == "prior", change
        assert needle in result.warnings[0], result.warnings
    # a file written for another config structure (dt changed)
    other = dataclasses.replace(cfg, dt=4.0)
    doc = stored_doc(other, saved_wall=NOW - 60)
    result = modelstore.load(write_doc(tmp_path / "m.json", doc), cfg, now_wall=NOW)
    assert result.source == "prior" and "another config" in result.warnings[0]


def test_age_decides_fresh_stale_and_a_clock_behind_the_file_is_stale(tmp_path):
    cfg = shadow_cfg(model_store_max_age_days=30.0)
    path = tmp_path / "m.json"
    for saved, source, age in (
        (NOW - 29 * DAY, "fresh", 29 * DAY),
        (NOW - 31 * DAY, "stale", 31 * DAY),
        (NOW + 3600, "stale", None),
    ):
        write_doc(path, stored_doc(cfg, saved_wall=saved))
        result = modelstore.load(path, cfg, now_wall=NOW)
        assert (result.source, result.age_s) == (source, age)
    assert "newer than the wall clock" in result.warnings[0]


def test_sections_of_the_wrong_shape_are_dropped_with_a_warning(tmp_path):
    cfg = shadow_cfg()
    doc = stored_doc(cfg, saved_wall=NOW - 60, thermal=[1, 2], calibration=5, bays="x")
    result = modelstore.load(write_doc(tmp_path / "m.json", doc), cfg, now_wall=NOW)
    assert result.source == "fresh"
    assert any("thermal" in w for w in result.warnings)
    assert any("bays" in w for w in result.warnings)
    cmd, _, _ = run(cfg, modelstore.initial_state(result), 1)
    store = cmd.diagnostics["store"]
    assert store["sections"]["thermal"] == "absent"
    assert any("calibration" in w for w in store["warnings"])


# ---------------------------------------------------------------------------
# apply: content level (bounds, finiteness, covariance)
# ---------------------------------------------------------------------------


@pytest.mark.parametrize(
    ("patch", "needle"),
    [
        (lambda m: m["zones"]["z0"]["air"]["theta"].__setitem__(0, 500.0), "outside"),
        (lambda m: m["bays"]["b01"]["theta"].__setitem__(2, 0.0), "outside"),
        (lambda m: m["bays"]["b01"]["theta"].__setitem__(1, "nan"), "not a finite"),
        (lambda m: m["bays"]["b01"]["P"][0].__setitem__(1, 5.0), "covariance"),
        (lambda m: m["bays"]["b01"]["P"][2].__setitem__(2, -1.0), "covariance"),
        (lambda m: m["bays"]["b01"].__setitem__("P", [[1.0]]), "finite"),
        (lambda m: m.__setitem__("fp", "other"), "structure"),
        (lambda m: m["zones"].pop("z1"), "missing"),
        (lambda m: m["zones"]["z0"].__setitem__("status", "dancing"), "malformed"),
    ],
    ids=[
        "E-out-of-bounds",
        "k-below-bound",
        "non-finite",
        "asymmetric-P",
        "negative-variance",
        "P-shape",
        "structure",
        "zone-missing",
        "bad-status",
    ],
)
def test_an_invalid_thermal_section_is_dropped_and_the_model_starts_at_its_prior(
    tmp_path, patch, needle
):
    cfg = shadow_cfg()
    memory = thermal.fresh_memory(cfg, status="converged")
    patch(memory)
    doc = stored_doc(cfg, saved_wall=NOW - 60, thermal=memory)
    result, state = load_state(write_doc(tmp_path / "m.json", doc), cfg)
    assert result.source == "fresh"
    cmd, _, _ = run(cfg, state, 1)
    store = cmd.diagnostics["store"]
    assert store["source"] == "fresh" and store["sections"]["thermal"] == "dropped"
    assert any(needle in w for w in store["warnings"]), store["warnings"]
    assert cmd.diagnostics["thermal"]["status"] == "prior"
    assert cmd.diagnostics["solver_diag"]["model"]["active"] == "pi_das"


def test_thermal_section_is_ignored_without_model_shadow(tmp_path):
    cfg = shadow_cfg(model_shadow=False)
    doc = stored_doc(
        cfg, saved_wall=NOW - 60, thermal=thermal.fresh_memory(shadow_cfg(), status="converged")
    )
    _, state = load_state(write_doc(tmp_path / "m.json", doc), cfg)
    cmd, state, _ = run(cfg, state, 2)
    assert cmd.diagnostics["store"]["sections"]["thermal"] == "ignored"
    assert "thermal" not in state.solver_memory


@settings(deadline=None)
@given(
    seed=st.recursive(
        st.none() | st.booleans() | st.integers() | st.floats() | st.text(max_size=5),
        lambda children: (
            st.lists(children, max_size=3)
            | st.dictionaries(
                st.sampled_from(
                    [
                        "source",
                        "thermal",
                        "calibration",
                        "fan_curves",
                        "bays",
                        "warnings",
                        "b01",
                        "x",
                    ]
                ),
                children,
                max_size=4,
            )
        ),
        max_leaves=12,
    ),
    source=st.sampled_from(["fresh", "stale", "prior", None]),
)
def test_a_malformed_seed_never_raises_out_of_step(seed, source):
    cfg = das_cfg(model_shadow=True, model_window_s=4.0)
    if isinstance(seed, dict) and source is not None:
        seed["source"] = source
    state = MpcState(solver_memory={STORE_SEED_KEY: seed})
    obs = das_obs(cfg, 0.0, pwm=0.5)
    cmd, new = checked_step(obs, cfg, state)
    assert STORE_SEED_KEY not in new.solver_memory
    json.dumps(new.to_dict(), allow_nan=False)
    assert cmd.diagnostics["store"]["source"] in persist.SOURCES


def test_legacy_step_has_no_store_diagnostics(cfg):
    from invariants import make_obs

    cmd, state = step(make_obs(cfg, ts=0.0), cfg, MpcState.cold())
    assert "store" not in cmd.diagnostics and STORE_KEY not in state.solver_memory


# ---------------------------------------------------------------------------
# fresh file: frozen, acts at once, never learns
# ---------------------------------------------------------------------------


def test_a_fresh_file_loads_converged_zones_frozen_and_the_mpc_acts_at_once(tmp_path):
    cfg = shadow_cfg()
    memory = thermal.fresh_memory(cfg, status="converged")
    doc = stored_doc(cfg, saved_wall=NOW - 3 * DAY, thermal=memory)
    _, state = load_state(write_doc(tmp_path / "m.json", doc), cfg)
    cmd, state, _ = run(cfg, state, 1)
    assert cmd.diagnostics["store"]["source"] == "fresh"
    assert cmd.diagnostics["thermal"]["status"] == "frozen"
    model = cmd.diagnostics["solver_diag"]["model"]
    assert model["active"] == "mpc" and model["status"] == "frozen"
    # frozen never moves its coefficients, but its windows keep closing (the guard scores)
    before = copy.deepcopy(state.solver_memory["thermal"])
    _, state, _ = run(cfg, state, 80, t0=cfg.dt)
    after = state.solver_memory["thermal"]
    assert after["bays"]["b01"]["w"] > before["bays"]["b01"]["w"]
    for b in before["bays"]:
        assert after["bays"][b]["theta"] == before["bays"][b]["theta"]
        assert after["bays"][b]["P"] == before["bays"][b]["P"]
    for z in before["zones"]:
        assert after["zones"][z]["air"]["theta"] == before["zones"][z]["air"]["theta"]


def test_a_fresh_file_never_makes_an_unconverged_zone_act(tmp_path):
    cfg = shadow_cfg()
    memory = thermal.fresh_memory(cfg, status="converged")
    memory["zones"]["z2"]["status"] = "learning"
    doc = stored_doc(cfg, saved_wall=NOW - 60, thermal=memory)
    _, state = load_state(write_doc(tmp_path / "m.json", doc), cfg)
    cmd, _, _ = run(cfg, state, 1)
    statuses = {z: v["status"] for z, v in cmd.diagnostics["thermal"]["zones"].items()}
    assert statuses == {"z0": "frozen", "z1": "frozen", "z2": "learning", "z3": "frozen"}
    assert cmd.diagnostics["thermal"]["status"] == "learning"
    assert cmd.diagnostics["solver_diag"]["model"]["active"] == "pi_das"


def test_overall_status_with_frozen_zones():
    assert thermal.overall_status(["frozen", "frozen"]) == "frozen"
    assert thermal.overall_status(["frozen", "converged"]) == "frozen"
    assert thermal.overall_status(["frozen", "prior"]) == "learning"
    assert thermal.overall_status(["frozen", "suspect"]) == "suspect"
    assert thermal.MODEL_STATUSES[-1] == "stale" and "frozen" in thermal.STATUSES


# ---------------------------------------------------------------------------
# stale file: shadow, PI-DAS with the loaded calibration, promotion
# ---------------------------------------------------------------------------


def _fake_convergence(monkeypatch, err2: float = 0.01) -> None:
    """Every closing window makes a learning zone converged with a small error (the truth
    of identification is tested in test_thermal_ident; here only the store's rule)."""

    def advance(zm, *args, **kwargs):
        if zm["status"] == "learning":
            zm["status"] = "converged"
        zm["err2"] = err2

    monkeypatch.setattr(thermal, "_advance_status", advance)


def test_a_stale_model_is_reconfirmed_in_shadow_before_the_mpc_acts(tmp_path, monkeypatch):
    cfg = shadow_cfg(model_reconfirm_s=300.0, model_accept_prior=True)
    memory = thermal.fresh_memory(cfg, status="converged")
    doc = stored_doc(cfg, saved_wall=NOW - 40 * DAY, thermal=memory)
    _, state = load_state(write_doc(tmp_path / "m.json", doc), cfg)
    _fake_convergence(monkeypatch)
    history = []
    for i in range(160):
        cmd, state, _ = run(cfg, state, 1, t0=i * cfg.dt)
        diag = cmd.diagnostics
        history.append(
            (
                i * cfg.dt,
                diag["thermal"]["status"],
                (diag["thermal"].get("hold") or {}).get("since_ts"),
                diag["solver_diag"]["model"]["active"],
            )
        )
    assert history[0][1:] == ("stale", None, "pi_das")
    assert cmd.diagnostics["store"]["source"] == "stale"
    # while stale the MPC never acts, even with model_accept_prior (owner's rule)
    stale = [h for h in history if h[1] == "stale"]
    assert all(h[3] == "pi_das" for h in stale)
    since = next(h[2] for h in history if h[2] is not None)
    released = next(h[0] for h in history if h[1] != "stale")
    assert released - since >= cfg.model_reconfirm_s
    assert released - since < cfg.model_reconfirm_s + 2 * cfg.dt
    first_mpc = next(h[0] for h in history if h[3] == "mpc")
    assert first_mpc > released  # then the validity gate's own dwell
    assert history[-1][1:] == ("converged", None, "mpc")


def test_the_stale_hold_restarts_when_convergence_or_the_error_fails():
    cfg = shadow_cfg(model_reconfirm_s=100.0)
    mem = thermal.fresh_memory(cfg, status="converged")
    for zm in mem["zones"].values():
        zm["err2"] = 0.04
    mem["hold"] = {"since": None}
    thermal._advance_hold(mem, cfg, 10.0)
    assert mem["hold"] == {"since": 10.0}
    mem["zones"]["z1"]["err2"] = 4.0  # prediction error 2 degC > 1: restart
    thermal._advance_hold(mem, cfg, 50.0)
    assert mem["hold"] == {"since": None}
    mem["zones"]["z1"]["err2"] = 0.04
    thermal._advance_hold(mem, cfg, 60.0)
    mem["zones"]["z2"]["status"] = "learning"  # a zone left converged: restart
    thermal._advance_hold(mem, cfg, 70.0)
    assert mem["hold"] == {"since": None}
    mem["zones"]["z2"]["status"] = "converged"
    thermal._advance_hold(mem, cfg, 80.0)
    thermal._advance_hold(mem, cfg, 20.0)  # clock stepped back: count from now
    assert mem["hold"] == {"since": 20.0}
    thermal._advance_hold(mem, cfg, 119.0)
    assert "hold" in mem
    thermal._advance_hold(mem, cfg, 120.0)
    assert "hold" not in mem and thermal.model_status(mem) == "converged"


def test_the_stale_hold_releases_with_model_freeze_on():
    """PROJECT.md section 8 item 16: with ``model_freeze`` a re-confirmed zone enters
    ``frozen``, not ``converged``. The stale hold must still count that as re-confirmed,
    or a stale file with the switch on would never act again."""
    cfg = shadow_cfg(model_reconfirm_s=100.0, model_freeze=True)
    mem = thermal.fresh_memory(cfg, status="frozen")
    for zm in mem["zones"].values():
        zm["err2"] = 0.04
    mem["hold"] = {"since": None}
    thermal._advance_hold(mem, cfg, 10.0)
    assert mem["hold"] == {"since": 10.0}
    thermal._advance_hold(mem, cfg, 110.0)
    assert "hold" not in mem and thermal.model_status(mem) == "frozen"


def test_a_saved_pending_hold_stays_pending_in_a_fresh_file(tmp_path):
    cfg = shadow_cfg()
    memory = thermal.fresh_memory(cfg, status="converged")
    memory["hold"] = {"since": 1234.0}
    doc = stored_doc(cfg, saved_wall=NOW - 60, thermal=memory)
    _, state = load_state(write_doc(tmp_path / "m.json", doc), cfg)
    cmd, state, _ = run(cfg, state, 1)
    assert cmd.diagnostics["store"]["source"] == "fresh"
    assert cmd.diagnostics["thermal"]["status"] == "stale"
    assert state.solver_memory["thermal"]["hold"] == {"since": None}
    assert {v["status"] for v in cmd.diagnostics["thermal"]["zones"].values()} == {"learning"}


def test_a_thermal_reset_keeps_a_pending_hold(monkeypatch):
    cfg = shadow_cfg()
    seed = {"source": "stale", "thermal": thermal.fresh_memory(cfg, status="converged")}
    state = MpcState(solver_memory={STORE_SEED_KEY: seed})

    def boom(*args, **kwargs):
        raise FloatingPointError("broken")

    monkeypatch.setattr(thermal, "update", boom)
    cmd, state, _ = run(cfg, state, 2, checked=False)
    assert cmd.diagnostics["thermal"]["status"] == "stale"
    assert state.solver_memory["thermal"]["error"].startswith("FloatingPointError")


# ---------------------------------------------------------------------------
# calibration: keyed by bay and serial, re-based ages, expiry, stale inflation
# ---------------------------------------------------------------------------


def serial_cfg(serial: str = "SER-A", **changes: Any) -> MpcConfig:
    data = das_mapping()
    data["topology"]["bays"]["b01"]["serial"] = serial
    data.update(changes)
    return MpcConfig.from_mapping(data)


def saved_calibration(cfg: MpcConfig, memory_ts: float, entries: dict[str, Any]) -> dict:
    """The file's calibration section for estimator entries at controller time ``memory_ts``."""
    est_mem = {"cal": {"b01": entries}}
    doc = modelstore.build_document(cfg, {"estimator": est_mem}, ts=memory_ts, wall=NOW - 600)
    return doc["calibration"]


def test_calibration_is_keyed_by_serial_and_survives_a_new_clock(tmp_path):
    cfg = serial_cfg("SER-A")
    # saved by a daemon whose clock read 5e6 s; SER-A sampled 100 s before the save
    section = saved_calibration(
        cfg,
        5_000_000.0,
        {
            "SER-A": cal_entry(ts=5_000_000.0 - 100.0, rms2=0.49),
            "SER-B": cal_entry(ts=5_000_000.0 - 50.0, th=[0.5, 1.0]),
        },
    )
    entry = section["b01"]["SER-A"]
    assert entry["last_sample_wall"] == pytest.approx(NOW - 700.0)
    assert entry["expires_wall"] == pytest.approx(NOW - 700.0 + 30 * DAY)
    doc = stored_doc(cfg, saved_wall=NOW - 600, calibration=section)
    _, state = load_state(write_doc(tmp_path / "m.json", doc), cfg)
    # the new process's clock starts near 0
    cmd, state, _ = run(cfg, state, 1, t0=12.0)
    cal = state.solver_memory["estimator"]["cal"]["b01"]
    assert set(cal) == {"SER-A", "SER-B"}
    assert cal["SER-A"]["ts"] == pytest.approx(12.0 - 700.0)
    assert cal["SER-B"]["th"] == [0.5, 1.0]
    bay = cmd.diagnostics["bays"]["b01"]
    assert bay["serial"] == "SER-A" and bay["calibrated"] is True
    assert bay["calibration"]["slope"] == 0.8
    assert bay["sigma_cal_c"] == pytest.approx(0.7)
    assert cmd.diagnostics["store"]["sections"]["calibration"] == 2
    # the same file on a config that declares SER-B in b01 uses SER-B's entry
    cmd_b, _, _ = run(
        serial_cfg("SER-B"),
        modelstore.initial_state(
            modelstore.load(tmp_path / "m.json", serial_cfg("SER-B"), now_wall=NOW)
        ),
        1,
    )
    assert cmd_b.diagnostics["bays"]["b01"]["calibration"]["slope"] == 0.5


def test_an_expired_or_undated_calibration_restarts_its_fresh_count(tmp_path):
    cfg = serial_cfg("SER-A")
    for last, expires in ((NOW - 31 * DAY, NOW - DAY), (None, None), (NOW + 100.0, None)):
        entry = {k: v for k, v in cal_entry().items() if k != "ts"}
        entry.update(last_sample_wall=last, expires_wall=expires)
        doc = stored_doc(cfg, saved_wall=NOW - 60, calibration={"b01": {"SER-A": entry}})
        _, state = load_state(write_doc(tmp_path / "m.json", doc), cfg)
        cmd, state, _ = run(cfg, state, 1)
        restored = state.solver_memory["estimator"]["cal"]["b01"]["SER-A"]
        assert restored["fresh"] == 0 and restored["used"] is True
        assert cmd.diagnostics["bays"]["b01"]["calibrated"] is False
        assert cmd.diagnostics["bays"]["b01"]["sigma_cal_c"] == est_prior.sigma_uncalibrated_c(cfg)


def test_a_stale_file_inflates_sigma_cal_until_smart_confirms_it(tmp_path):
    cfg = serial_cfg("SER-A", model_store_max_age_days=1.0)
    entry = {k: v for k, v in cal_entry(rms2=0.49).items() if k != "ts"}
    entry.update(last_sample_wall=NOW - 3600.0, expires_wall=NOW + 29 * DAY)
    doc = stored_doc(cfg, saved_wall=NOW - 2 * DAY, calibration={"b01": {"SER-A": entry}})
    result, state = load_state(write_doc(tmp_path / "m.json", doc), cfg)
    assert result.source == "stale"
    cmd, state, _ = run(cfg, state, 1)
    bay = cmd.diagnostics["bays"]["b01"]
    assert bay["calibrated"] is True
    assert bay["sigma_cal_c"] == pytest.approx(2.0 * 0.7)
    restored = state.solver_memory["estimator"]["cal"]["b01"]["SER-A"]
    assert (restored["inflate"], restored["confirm"]) == (2.0, estimator.CAL_MIN_SAMPLES)
    # every accepted SMART sample counts; after CAL_MIN_SAMPLES the inflation is gone
    e = dict(restored)
    for i in range(estimator.CAL_MIN_SAMPLES - 1):
        e, _ = estimator.calibration_update(e, 10.0, 6.5, float(i))
        assert e["inflate"] == 2.0
    e, _ = estimator.calibration_update(e, 10.0, 6.5, 99.0)
    assert "inflate" not in e and "confirm" not in e
    # a pending inflation survives a save and a fresh reload
    doc2 = modelstore.build_document(cfg, state.solver_memory, ts=0.0, wall=NOW)
    assert doc2["calibration"]["b01"]["SER-A"]["inflate"] == 2.0
    _, state2 = load_state(write_doc(tmp_path / "m2.json", doc2), cfg, now=NOW + 60)
    cmd2, state2, _ = run(cfg, state2, 1)
    assert cmd2.diagnostics["store"]["source"] == "fresh"
    assert cmd2.diagnostics["bays"]["b01"]["sigma_cal_c"] == pytest.approx(1.4)


@pytest.mark.parametrize(
    ("change", "needle"),
    [
        ({"th": [0.1, -1.0]}, "slope"),
        ({"th": [0.8, 25.0]}, "offset"),
        ({"th": [0.8]}, "th"),
        ({"P": [[0.01, 0.5], [0.0, 1.0]]}, "covariance"),
        ({"P": [[-0.01, 0.0], [0.0, 1.0]]}, "covariance"),
        ({"n": 5, "fresh": 9}, "fresh"),
        ({"rms2": -1.0}, "rms2"),
        ({"used": "yes"}, "used"),
    ],
)
def test_a_bad_calibration_entry_is_dropped_and_the_others_kept(tmp_path, change, needle):
    cfg = serial_cfg("SER-A")
    good = {k: v for k, v in cal_entry().items() if k != "ts"}
    good.update(last_sample_wall=NOW - 60, expires_wall=NOW + DAY)
    bad = {**good, **change}
    calibration = {"b01": {"SER-A": good, "SER-X": bad}, "b99": {"SER-Z": good}}
    doc = stored_doc(cfg, saved_wall=NOW - 60, calibration=calibration)
    _, state = load_state(write_doc(tmp_path / "m.json", doc), cfg)
    cmd, state, _ = run(cfg, state, 1)
    assert set(state.solver_memory["estimator"]["cal"]) == {"b01"}
    assert set(state.solver_memory["estimator"]["cal"]["b01"]) == {"SER-A"}
    warnings = cmd.diagnostics["store"]["warnings"]
    assert any("SER-X" in w and needle in w for w in warnings), warnings
    assert any("b99" in w for w in warnings)


def test_bays_are_reported_and_occupancy_restarts_unknown(tmp_path):
    cfg = serial_cfg("SER-A")
    bays = {"b02": {"occupancy": "empty", "class": "hdd", "serial": "S2", "association": "x"}}
    doc = stored_doc(cfg, saved_wall=NOW - 60, bays=bays)
    _, state = load_state(write_doc(tmp_path / "m.json", doc), cfg)
    cmd, _, _ = run(cfg, state, 1)
    assert cmd.diagnostics["store"]["bays"] == bays
    assert cmd.diagnostics["bays"]["b02"]["occupancy"] != "empty"


def test_fan_curves_are_validated_and_kept(tmp_path):
    cfg = shadow_cfg()
    model = next(iter(cfg.fan_models))
    curves = {
        model: {"rpm_max": 1500.0, "deadband": 0.15, "exponent": 1.0},
        "nope": {"rpm_max": 1.0, "deadband": 0.1, "exponent": 1.0},
    }
    doc = stored_doc(cfg, saved_wall=NOW - 60, fan_curves=curves)
    _, state = load_state(write_doc(tmp_path / "m.json", doc), cfg)
    cmd, state, _ = run(cfg, state, 1)
    assert state.solver_memory["fan_curves"] == {model: curves[model]}
    assert any("nope" in w for w in cmd.diagnostics["store"]["warnings"])


# ---------------------------------------------------------------------------
# save: document, atomic write, persister
# ---------------------------------------------------------------------------


def test_round_trip_through_a_running_controller(tmp_path):
    cfg = shadow_cfg()
    _, state, _ = run(cfg, MpcState.cold(), 30)
    doc = modelstore.build_document(
        cfg, state.solver_memory, ts=state.solver_memory["last_ts"], wall=NOW
    )
    json.dumps(doc, allow_nan=False)
    path = tmp_path / "model.json"
    modelstore.write_atomic(path, json.dumps(doc).encode())
    result, restored = load_state(path, cfg, now=NOW + 5)
    assert result.source == "fresh" and result.warnings == []
    cmd, _, _ = run(cfg, restored, 1, t0=3.0)
    assert cmd.diagnostics["store"]["sections"]["thermal"] == "loaded"
    assert cmd.diagnostics["store"]["warnings"] == []


def test_write_is_atomic(tmp_path, monkeypatch):
    path = tmp_path / "model.json"
    path.write_bytes(b"old")
    calls: list[str] = []
    real_fsync, real_replace = os.fsync, os.replace
    monkeypatch.setattr(os, "fsync", lambda fd: (calls.append("fsync"), real_fsync(fd))[1])
    monkeypatch.setattr(
        os, "replace", lambda a, b: (calls.append("replace"), real_replace(a, b))[1]
    )
    modelstore.write_atomic(path, b"new content")
    assert path.read_bytes() == b"new content"
    assert calls[:2] == ["fsync", "replace"]  # data on disk before the rename
    assert [p.name for p in tmp_path.iterdir()] == ["model.json"]

    def broken(a, b):
        raise OSError("disk full")

    monkeypatch.setattr(os, "replace", broken)
    with pytest.raises(OSError):
        modelstore.write_atomic(path, b"half")
    assert path.read_bytes() == b"new content"  # the previous file is untouched
    assert [p.name for p in tmp_path.iterdir()] == ["model.json"]  # no temporary left


class FakeClock:
    def __init__(self) -> None:
        self.now = 0.0

    def __call__(self) -> float:
        return self.now


def test_persister_writes_at_most_every_interval_and_at_close(tmp_path, caplog):
    cfg = shadow_cfg(model_store_interval_s=60.0)
    path = tmp_path / "model.json"
    clock = FakeClock()
    persister = modelstore.ModelPersister(cfg, path, clock=clock, wall=lambda: NOW)
    seed = modelstore.load(path, cfg, now_wall=NOW)
    state = modelstore.initial_state(seed)
    caplog.set_level(logging.INFO, logger="aqua_bridge.modelstore")
    for i in range(40):
        clock.now = i * cfg.dt
        cmd, state, _ = run(cfg, state, 1, t0=i * cfg.dt)
        persister.on_tick(SimpleNamespace(state=state, mpc_cmd=cmd))
        assert path.exists() == (clock.now >= 60.0), i
        if clock.now >= 60.0 and clock.now < 120.0:
            assert persister.writes == 1
    assert persister.writes == 3  # at 60, 120, 180 s
    assert "loaded as prior" in caplog.text
    mtime_doc = json.loads(path.read_text())
    assert persister.close() is True and persister.writes == 4
    assert persister.close() is False  # nothing new
    doc = json.loads(path.read_text())
    assert doc["saved_wall"] == NOW and doc["fingerprint"] == modelstore.fingerprint(cfg)
    assert doc["thermal"]["zones"] and mtime_doc["schema"] == modelstore.SCHEMA
    assert set(doc["bays"]) == set(cfg.topology.bays)


def test_persister_never_writes_an_unapplied_seed_and_never_raises(tmp_path):
    cfg = shadow_cfg(model_store_interval_s=1.0)
    path = tmp_path / "model.json"
    path.write_text("keep me")
    clock = FakeClock()
    persister = modelstore.ModelPersister(cfg, path, clock=clock)
    pending = MpcState(solver_memory={STORE_SEED_KEY: {"source": "prior"}})
    clock.now = 100.0
    persister.on_tick(SimpleNamespace(state=pending, mpc_cmd=None))
    assert persister.close() is False and path.read_text() == "keep me"
    persister.on_tick(SimpleNamespace(state=None, mpc_cmd=None))
    persister.on_tick(object())
    # a directory that does not exist: logged, counted, never raised
    broken = modelstore.ModelPersister(cfg, tmp_path / "missing" / "model.json", clock=clock)
    _, state, _ = run(cfg, MpcState.cold(), 1)
    clock.now = 200.0
    broken.on_tick(SimpleNamespace(state=state, mpc_cmd=None))
    assert broken.errors == 1 and broken.writes == 0 and "Error" in broken.last_error
    with pytest.raises(ValueError):
        modelstore.ModelPersister(load_config(EXAMPLE_CONFIG).mpc, path)


def test_calibration_times_in_the_future_of_the_snapshot_are_undated():
    cfg = serial_cfg("SER-A")
    section = saved_calibration(cfg, 100.0, {"SER-A": cal_entry(ts=150.0)})
    assert section["b01"]["SER-A"]["last_sample_wall"] is None
    assert section["b01"]["SER-A"]["expires_wall"] is None
    assert math.isfinite(NOW)


# ---------------------------------------------------------------------------
# daemon wiring
# ---------------------------------------------------------------------------


@pytest.fixture
def restore_signals():
    import signal

    old_term = signal.getsignal(signal.SIGTERM)
    old_int = signal.getsignal(signal.SIGINT)
    yield
    signal.signal(signal.SIGTERM, old_term)
    signal.signal(signal.SIGINT, old_int)


def test_main_loads_and_saves_the_store(tmp_path, restore_signals):
    store = tmp_path / "model.json"
    argv = [
        "--config",
        str(EXAMPLE_DAS_CONFIG),
        "--source",
        "sim",
        "--sim-plant",
        "das",
        "--ticks",
        "3",
        "--sim-speed",
        "0",
        "--model-store",
        str(store),
    ]
    assert main_mod.main(argv) == 0
    doc = json.loads(store.read_text())  # written at the clean stop
    cfg = load_config(EXAMPLE_DAS_CONFIG).mpc
    assert doc["fingerprint"] == modelstore.fingerprint(cfg)
    assert modelstore.load(store, cfg, now_wall=doc["saved_wall"] + 1).source == "fresh"
    assert main_mod.main(argv) == 0  # and loads it again


def test_main_uses_state_directory_for_das_and_ignores_it_for_legacy(
    tmp_path, restore_signals, monkeypatch
):
    monkeypatch.setenv("STATE_DIRECTORY", str(tmp_path))
    legacy = [
        "--config",
        str(EXAMPLE_CONFIG),
        "--source",
        "sim",
        "--ticks",
        "2",
        "--sim-speed",
        "0",
    ]
    assert main_mod.main(legacy) == 0
    assert list(tmp_path.iterdir()) == []
    das = [
        "--config",
        str(EXAMPLE_DAS_CONFIG),
        "--source",
        "sim",
        "--sim-plant",
        "das",
        "--ticks",
        "2",
        "--sim-speed",
        "0",
    ]
    assert main_mod.main(das) == 0
    assert (tmp_path / "model.json").exists()


def test_main_model_store_with_a_legacy_config_exits_2(tmp_path, restore_signals):
    argv = [
        "--config",
        str(EXAMPLE_CONFIG),
        "--source",
        "sim",
        "--ticks",
        "1",
        "--model-store",
        str(tmp_path / "m.json"),
    ]
    assert main_mod.main(argv) == 2


def test_http_model_view_shows_what_the_store_loaded(tmp_path):
    import asyncio

    from aqua_bridge.control.supervisor import Supervisor
    from test_das_intents import _post_all, needs_socket

    if needs_socket.args[0]:
        pytest.skip(needs_socket.kwargs["reason"])
    cfg = shadow_cfg()
    doc = stored_doc(
        cfg, saved_wall=NOW - 60, thermal=thermal.fresh_memory(cfg, status="converged")
    )
    _, state = load_state(write_doc(tmp_path / "m.json", doc), cfg)
    sup = Supervisor(cfg)
    obs = das_obs(cfg, 0.0, pwm=0.5, **temps_for(cfg, 0))
    cmd, state = step(obs, sup.effective_config(), state)
    sup.record_tick(obs=obs, mpc_cmd=cmd, cmd=cmd, state=state, applied=True, usb_present=True)
    ((status, body),) = asyncio.run(_post_all(sup, [("GET /api/model", None)]))
    assert status == 200
    assert body["store"]["source"] == "fresh"
    assert body["store"]["sections"]["thermal"] == "loaded"
    assert body["thermal"]["status"] == "frozen"
