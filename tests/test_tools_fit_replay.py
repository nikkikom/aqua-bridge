"""Tests for ``tools/fit_model.py``, ``tools/fit_fans.py`` and ``tools/replay.py``
(plan sections 5, 10, 11 milestone 8), against synthetic recordings from
:mod:`aqua_bridge.sim.das`.

``tools/`` is not on ``pythonpath`` (only ``src``/``tests`` are, per
``pyproject.toml``), so this file adds it to ``sys.path`` itself, the same way a
standalone script is expected to be run (matches ``tests/test_w1_commission.py``).
"""

from __future__ import annotations

import dataclasses
import json
import sys
from pathlib import Path
from typing import Any

import pytest

TOOLS_DIR = Path(__file__).resolve().parent.parent / "tools"
if str(TOOLS_DIR) not in sys.path:
    sys.path.insert(0, str(TOOLS_DIR))

import fit_fans  # noqa: E402 -- must follow the sys.path tweak above
import fit_model  # noqa: E402
import replay  # noqa: E402

from aqua_bridge.__main__ import PlantIO  # noqa: E402
from aqua_bridge.config import load_config  # noqa: E402
from aqua_bridge.control import thermal  # noqa: E402
from aqua_bridge.control.loop import Loop  # noqa: E402
from aqua_bridge.control.supervisor import Supervisor  # noqa: E402
from aqua_bridge.model import MpcConfig  # noqa: E402
from aqua_bridge.recorder import Recorder, iter_records, thermal_inputs  # noqa: E402
from aqua_bridge.sim import das as simdas  # noqa: E402
from das_fixtures import das_cfg  # noqa: E402

REPO_ROOT = Path(__file__).resolve().parent.parent
EXAMPLE_CONFIG = REPO_ROOT / "config.example.yaml"


def _small_das_cfg(**changes: Any) -> MpcConfig:
    return das_cfg(setpoints={}, model_window_s=8.0, **changes)


def _record(cfg: MpcConfig, path: Path, ticks: int, *, seed: int = 0) -> list[dict[str, Any]]:
    """A real closed loop against the DAS truth plant, recorded to ``path``."""
    topology = simdas.topology_from_config(cfg)
    heat_schedule = {"a2": [(0.0, 0.2), (40.0, 1.0), (80.0, 0.2), (120.0, 1.0), (160.0, 0.2)]}
    plant = simdas.build_das_plant(
        topology=topology,
        preset="basic",
        seed=seed,
        dt=cfg.dt,
        initial_pwm=dict(cfg.fallback_pwm),
        heat_schedule=heat_schedule,
    )
    io = PlantIO(plant, cfg)
    supervisor = Supervisor(cfg, version="test")
    rec = Recorder(cfg, path, max_bytes=50_000_000)
    loop = Loop(io, io, cfg, supervisor, on_tick=rec.on_tick)
    for _ in range(ticks):
        loop.tick()
    rec.close()
    return iter_records(path)


@pytest.fixture(scope="module")
def recording(
    tmp_path_factory: pytest.TempPathFactory,
) -> tuple[MpcConfig, Path, list[dict[str, Any]]]:
    cfg = _small_das_cfg()
    path = tmp_path_factory.mktemp("fit_replay") / "rec.jsonl"
    records = _record(cfg, path, 260)
    return cfg, path, records


@pytest.fixture(scope="module")
def config_path(tmp_path_factory: pytest.TempPathFactory) -> Path:
    """A YAML config file on disk matching the ``recording`` fixture's ``cfg``, for the
    CLI (``--config``) tests."""
    import yaml

    from das_fixtures import das_mapping

    mapping = das_mapping()
    mapping["setpoints"] = {}
    mapping["model_window_s"] = 8.0
    path = tmp_path_factory.mktemp("fit_replay_cfg") / "config.yaml"
    path.write_text(yaml.safe_dump({"mpc": mapping}), encoding="utf-8")
    return path


# ---------------------------------------------------------------------------
# fit_model.py
# ---------------------------------------------------------------------------


def test_fit_needs_a_das_config(cfg: MpcConfig) -> None:
    with pytest.raises(ValueError, match="zoned"):
        fit_model.fit(cfg, [{"das": False}] * 25)


def test_fit_needs_enough_records(recording: tuple[MpcConfig, Path, list[dict[str, Any]]]) -> None:
    cfg, _, records = recording
    with pytest.raises(ValueError, match="not enough"):
        fit_model.fit(cfg, records[:5])


def test_fit_end_to_end_on_a_synthetic_recording(
    recording: tuple[MpcConfig, Path, list[dict[str, Any]]],
) -> None:
    cfg, _, records = recording
    payload = fit_model.fit(cfg, records, horizon_ticks=8, max_chunks=40)

    assert payload["n_records"] == len(records)
    assert payload["n_train"] + payload["n_holdout"] == len(records)
    assert payload["summary"]["status"] in thermal.STATUSES
    st = thermal.structure(cfg)
    assert payload["fingerprint"] == st.fingerprint
    # the model store's own structure fingerprint travels with the report, so a report
    # copied into $STATE_DIRECTORY is checked on the rule a store file is checked on
    from aqua_bridge import modelstore

    assert payload["store_fingerprint"] == modelstore.fingerprint(cfg)
    assert set(payload["bay_ranking"][0]) >= {
        "bay",
        "zone",
        "class",
        "limit_c",
        "margin_c",
        "tau_d_s",
        "rank_combined",
    }
    assert {row["bay"] for row in payload["bay_ranking"]} == set(st.bays)
    # combined ranking is sorted ascending
    combined = [row["rank_combined"] for row in payload["bay_ranking"]]
    assert combined == sorted(combined)
    assert payload["parameters_pinned"]  # at least reports every identified key
    for key in thermal.parameter_keys(st):
        assert key in payload["parameters_pinned"]
    # every zone that produced any window has a refinement report
    for z in st.zones:
        assert z in payload["refinement"]
    assert payload["holdout"]["n_records"] == payload["n_holdout"]
    # whole payload is finite JSON, the on-disk contract
    json.dumps(payload, allow_nan=False)


def test_fit_is_deterministic(recording: tuple[MpcConfig, Path, list[dict[str, Any]]]) -> None:
    cfg, _, records = recording
    a = fit_model.fit(cfg, records, horizon_ticks=8, max_chunks=40)
    b = fit_model.fit(cfg, records, horizon_ticks=8, max_chunks=40)
    assert a["memory"] == b["memory"]
    assert a["bay_ranking"] == b["bay_ranking"]


def test_fit_with_zero_holdout_frac_scores_nothing_out_of_sample(
    recording: tuple[MpcConfig, Path, list[dict[str, Any]]],
) -> None:
    cfg, _, records = recording
    payload = fit_model.fit(cfg, records, holdout_frac=0.0, horizon_ticks=8, max_chunks=40)
    assert payload["n_holdout"] == 0
    assert payload["holdout"]["equation_error"] is None
    assert payload["holdout"]["air_rmse_c"] == {}


def test_refine_air_reports_skipped_without_data(recording) -> None:
    cfg, _, records = recording
    st = thermal.structure(cfg)
    theta = thermal.prior_theta(cfg, st)
    theta_out, report = fit_model.refine_air(cfg, st, theta, records[:1])
    assert theta_out == theta  # nothing to refine from one record
    assert all(v["status"] == "skipped" for v in report.values())


def test_refine_air_reduces_or_holds_the_rollout_error(
    recording: tuple[MpcConfig, Path, list[dict[str, Any]]],
) -> None:
    cfg, _, records = recording
    st = thermal.structure(cfg)
    mem = None
    result = None
    for rec in records:
        result = thermal.update(mem, cfg, learn=True, **thermal_inputs(cfg, rec))
        mem = result.memory
    theta = thermal.theta_from_memory(cfg, mem, st=st)
    _, report = fit_model.refine_air(cfg, st, theta, records, horizon_ticks=8, max_chunks=40)
    for _z, info in report.items():
        if info["status"] != "ok":
            continue
        assert info["rmse_c_after"] <= info["rmse_c_before"] + 1e-9


def test_rank_bays_orders_by_margin_and_dynamics(
    recording: tuple[MpcConfig, Path, list[dict[str, Any]]],
) -> None:
    cfg, _, records = recording
    st = thermal.structure(cfg)
    theta = thermal.prior_theta(cfg, st)
    rows = fit_model.rank_bays(cfg, st, theta, records)
    by_bay = {r["bay"]: r for r in rows}
    assert set(by_bay) == set(st.bays)
    # a1/b1 (hdd, tau_d_s = 720 * (0.3+0.5) = 576) are slower than a2 (ssd_sata,
    # tau_d_s = 200 * 0.8 = 160): fastest dynamics ranks a2 first among occupied bays.
    occupied_by_speed = sorted(
        (b for b in ("a1", "a2", "b1")), key=lambda b: by_bay[b]["rank_dynamics"]
    )
    assert occupied_by_speed[0] == "a2"


def test_rank_bays_never_raises_with_no_observations() -> None:
    cfg = _small_das_cfg()
    st = thermal.structure(cfg)
    theta = thermal.prior_theta(cfg, st)
    rows = fit_model.rank_bays(cfg, st, theta, [])
    assert {r["bay"] for r in rows} == set(st.bays)
    assert all(r["margin_c"] is None for r in rows)
    # nothing observed: every bay ties at the same (least-urgent) margin rank
    assert len({r["rank_margin"] for r in rows}) == 1


def test_parameters_pinned_reports_every_key_with_a_pinned_flag(
    recording: tuple[MpcConfig, Path, list[dict[str, Any]]],
) -> None:
    cfg, _, records = recording
    payload = fit_model.fit(cfg, records, horizon_ticks=8, max_chunks=40)
    for _key, info in payload["parameters_pinned"].items():
        assert isinstance(info["pinned"], bool)
        if info["pinned"]:
            assert info["rel_se"] is not None
            assert info["rel_se"] < cfg.model_converged_rel_se


def test_fit_model_cli_writes_model_json(
    recording: tuple[MpcConfig, Path, list[dict[str, Any]]], config_path: Path, tmp_path: Path
) -> None:
    _, rec_path, _ = recording
    out = tmp_path / "model.json"
    code = fit_model.main(
        ["--config", str(config_path), "--topology", "--out", str(out), str(rec_path)]
    )
    assert code == 0
    doc = json.loads(out.read_text(encoding="utf-8"))
    assert doc["kind"] == "aqua_bridge.thermal_model"
    assert doc["recordings"] == [str(rec_path)]


def test_fit_model_cli_store_out_writes_a_loadable_model(
    recording: tuple[MpcConfig, Path, list[dict[str, Any]]], config_path: Path, tmp_path: Path
) -> None:
    """PROJECT.md section 8 item 15: ``--store-out`` converts the fit into a model store
    document, and the daemon's own loader takes it -- fresh, thermal loaded, carrying the
    coefficients the fit produced."""
    from aqua_bridge import modelstore
    from aqua_bridge.control import persist

    cfg, rec_path, _ = recording
    out = tmp_path / "report.json"
    store = tmp_path / "model.json"
    code = fit_model.main(
        [
            "--config",
            str(config_path),
            "--topology",
            "--out",
            str(out),
            "--store-out",
            str(store),
            str(rec_path),
        ]
    )
    assert code == 0
    doc = json.loads(store.read_text(encoding="utf-8"))
    assert doc["schema"] == modelstore.SCHEMA and doc["v"] == modelstore.SCHEMA_VERSION
    assert doc["fingerprint"] == modelstore.fingerprint(cfg)

    shadow = dataclasses.replace(load_config(config_path).mpc, model_shadow=True)
    result = modelstore.load(store, shadow, now_wall=doc["saved_wall"] + 60.0)
    assert result.source == "fresh" and not result.warnings
    mem: dict[str, Any] = {}
    summary = persist.apply_seed(mem, shadow, result.seed, 0.0)
    assert summary["sections"]["thermal"] == "loaded"
    fitted = json.loads(out.read_text(encoding="utf-8"))
    st = thermal.structure(shadow)
    assert thermal.theta_from_memory(shadow, mem["thermal"], st=st) == pytest.approx(
        thermal.theta_from_memory(shadow, fitted["memory"], st=st)
    )


def test_fit_model_cli_store_out_refuses_to_overwrite_the_report(
    recording: tuple[MpcConfig, Path, list[dict[str, Any]]], config_path: Path, tmp_path: Path
) -> None:
    _, rec_path, _ = recording
    out = tmp_path / "same.json"
    code = fit_model.main(
        [
            "--config",
            str(config_path),
            "--topology",
            "--out",
            str(out),
            "--store-out",
            str(out),
            str(rec_path),
        ]
    )
    assert code == 2
    assert json.loads(out.read_text(encoding="utf-8"))["kind"] == "aqua_bridge.thermal_model"


def test_fit_model_cli_requires_topology_flag(config_path: Path, tmp_path: Path) -> None:
    with pytest.raises(SystemExit):
        fit_model.main(["--config", str(config_path), "--out", str(tmp_path / "m.json"), "x.jsonl"])


def test_fit_model_cli_rejects_a_legacy_config(tmp_path: Path) -> None:
    empty = tmp_path / "empty.jsonl"
    empty.write_text("", encoding="utf-8")
    code = fit_model.main(
        [
            "--config",
            str(EXAMPLE_CONFIG),
            "--topology",
            "--out",
            str(tmp_path / "m.json"),
            str(empty),
        ]
    )
    assert code == 2


# ---------------------------------------------------------------------------
# fit_fans.py
# ---------------------------------------------------------------------------


def test_fit_one_model_recovers_a_known_curve() -> None:
    from aqua_bridge.control.thermal import phi

    rng_pairs = [
        (u, 1800.0 * phi(u, 0.12, 1.0))
        for u in [0.0, 0.1, 0.15, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0] * 3
    ]
    fitted = fit_fans.fit_one_model(rng_pairs)
    assert fitted is not None
    assert fitted["rpm_max"] == pytest.approx(1800.0, rel=0.02)
    assert fitted["deadband"] == pytest.approx(0.125, abs=0.02)
    assert fitted["rmse_frac"] < 0.01
    assert fitted["source"] == "fitted"


def test_fit_one_model_needs_enough_samples() -> None:
    assert fit_fans.fit_one_model([(0.5, 900.0)] * 3) is None


def test_fit_fan_curves_falls_back_to_prior_without_tach_data(
    recording: tuple[MpcConfig, Path, list[dict[str, Any]]],
) -> None:
    cfg, _, records = recording
    # strip every rpm reading: no model has any tach data in this recording
    blind = [{**r, "rpm": {}} for r in records]
    curves = fit_fans.fit_fan_curves(cfg, blind)
    assert set(curves) == set(cfg.fan_models)
    for model, info in curves.items():
        assert info["source"] == "prior"
        assert info["rpm_max"] == cfg.fan_models[model].rpm_max
        assert info["channels_with_tach"] == []


def test_fit_fan_curves_fits_models_with_tach_data(
    recording: tuple[MpcConfig, Path, list[dict[str, Any]]],
) -> None:
    cfg, _, records = recording
    curves = fit_fans.fit_fan_curves(cfg, records)
    assert set(curves) == set(cfg.fan_models)
    for _model, info in curves.items():
        assert info["source"] == "fitted"
        assert info["rpm_max"] > 0
        assert info["n_samples"] > 0
        json.dumps(info, allow_nan=False)


def test_fit_fans_cli_writes_fan_curves_json(
    recording: tuple[MpcConfig, Path, list[dict[str, Any]]], config_path: Path, tmp_path: Path
) -> None:
    _, rec_path, _ = recording
    out = tmp_path / "fan_curves.json"
    code = fit_fans.main(["--config", str(config_path), "--out", str(out), str(rec_path)])
    assert code == 0
    doc = json.loads(out.read_text(encoding="utf-8"))
    assert doc["kind"] == "aqua_bridge.fan_curves"
    assert set(doc["models"]) == set(load_config(config_path).mpc.fan_models)


def test_fit_fans_cli_rejects_a_config_without_fan_models(tmp_path: Path) -> None:
    empty = tmp_path / "empty.jsonl"
    empty.write_text('{"pwm": {}, "rpm": {}}\n', encoding="utf-8")
    code = fit_fans.main(
        ["--config", str(EXAMPLE_CONFIG), "--out", str(tmp_path / "f.json"), str(empty)]
    )
    assert code == 2


# ---------------------------------------------------------------------------
# replay.py
# ---------------------------------------------------------------------------


def test_replay_learns_from_scratch_by_default(
    recording: tuple[MpcConfig, Path, list[dict[str, Any]]],
) -> None:
    cfg, _, records = recording
    report = replay.replay(cfg, records)
    assert report["learn"] is True
    assert report["n_records"] == len(records)
    assert report["summary"]["status"] in thermal.STATUSES
    assert set(report["pred_err_series"]) == set(thermal.structure(cfg).zones)
    json.dumps(report, allow_nan=False)


def test_replay_frozen_never_changes_theta(
    recording: tuple[MpcConfig, Path, list[dict[str, Any]]],
) -> None:
    cfg, _, records = recording
    payload = fit_model.fit(cfg, records, horizon_ticks=8, max_chunks=40)
    before = json.dumps(payload["memory"], sort_keys=True)
    report = replay.replay(cfg, records, memory=payload["memory"], learn=False)
    st = thermal.structure(cfg)
    for z in st.zones:
        assert report["summary"]["zones"][z]["theta"] == payload["summary"]["zones"][z]["theta"]
    for b in st.bays:
        assert report["summary"]["bays"][b]["theta"] == payload["summary"]["bays"][b]["theta"]
    # fitting again from the same (unmutated) memory gives the same theta back
    assert json.dumps(payload["memory"], sort_keys=True) == before


def test_replay_frozen_needs_a_model() -> None:
    with pytest.raises(SystemExit):
        replay.main(["--config", str(EXAMPLE_CONFIG), "--frozen", "x.jsonl"])


def test_replay_cli_end_to_end(
    recording: tuple[MpcConfig, Path, list[dict[str, Any]]], config_path: Path, tmp_path: Path
) -> None:
    _, rec_path, _ = recording
    out = tmp_path / "replay.json"
    code = replay.main(["--config", str(config_path), "--out", str(out), str(rec_path)])
    assert code == 0
    doc = json.loads(out.read_text(encoding="utf-8"))
    assert doc["n_records"] > 0
    assert doc["learn"] is True


def test_replay_cli_rejects_a_legacy_config(tmp_path: Path) -> None:
    empty = tmp_path / "empty.jsonl"
    empty.write_text('{"das": false}\n' * 30, encoding="utf-8")
    code = replay.main(["--config", str(EXAMPLE_CONFIG), str(empty)])
    assert code == 2


def test_replay_cli_needs_usable_records(config_path: Path, tmp_path: Path) -> None:
    empty = tmp_path / "empty.jsonl"
    empty.write_text('{"das": false}\n', encoding="utf-8")
    code = replay.main(["--config", str(config_path), str(empty)])
    assert code == 2
