"""``tools/bench_model_store.py`` (PROJECT.md section 8 item 48): model.json write
timing against a scratch path only, never the daemon's own store."""

from __future__ import annotations

import importlib.util
import json
from pathlib import Path

import pytest

from conftest import REPO_ROOT


def _load_tool():
    spec = importlib.util.spec_from_file_location(
        "bench_model_store", REPO_ROOT / "tools" / "bench_model_store.py"
    )
    assert spec is not None and spec.loader is not None
    tool = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(tool)
    return tool


@pytest.mark.parametrize(
    "path",
    [
        "/etc/aqua-bridge/model.json",
        "/opt/aqua-bridge/state/model.json",
        "/var/lib/aqua-bridge/model.json",
        str(Path.home() / "model.json"),
    ],
)
def test_refuses_a_path_outside_a_scratch_directory(path):
    tool = _load_tool()
    with pytest.raises(SystemExit, match="refusing --path"):
        tool.ensure_scratch_path(Path(path))


def test_accepts_a_path_under_tmp():
    tool = _load_tool()
    accepted = tool.ensure_scratch_path(Path("/tmp/aqua-bridge-bench-model-store-test.json"))
    assert accepted == Path("/tmp/aqua-bridge-bench-model-store-test.json").resolve()


def test_bench_tool_writes_times_a_size_and_cleans_up_by_default(capsys):
    tool = _load_tool()
    assert tool.main(["--warm-ticks", "5", "--repeats", "3"]) == 0
    report = json.loads(capsys.readouterr().out)
    path = Path(report["path"])
    assert not path.exists()  # cleaned up: --keep was not passed
    result = report["result"]
    assert result["repeats"] == 3
    assert len(result["times_ms"]) == 3
    assert result["size_bytes"] > 0
    assert result["min_ms"] <= result["median_ms"] <= result["max_ms"]
    assert result["min_ms"] <= result["mean_ms"] <= result["max_ms"]
    assert result["p99_ms"] >= result["median_ms"]
    assert result["spread_ms"] == pytest.approx(result["max_ms"] - result["min_ms"])
    assert result["p99_fraction_of_dt"] == pytest.approx(result["p99_ms"] / (result["dt_s"] * 1e3))
    assert set(result["sections"]) <= {
        "bays",
        "calibration",
        "fan_curves",
        "fingerprint",
        "ident_settle",
        "manual_calibration",
        "thermal",
    }


def test_bench_tool_keep_leaves_the_scratch_file_behind(capsys):
    tool = _load_tool()
    assert tool.main(["--warm-ticks", "5", "--repeats", "2", "--keep"]) == 0
    report = json.loads(capsys.readouterr().out)
    path = Path(report["path"])
    try:
        assert path.exists()
        assert path.stat().st_size == report["result"]["size_bytes_on_disk"]
    finally:
        path.unlink(missing_ok=True)


def test_bench_tool_refuses_a_non_scratch_path_before_running_anything(capsys):
    tool = _load_tool()
    with pytest.raises(SystemExit, match="refusing --path"):
        tool.main(["--warm-ticks", "5", "--repeats", "2", "--path", "/etc/aqua-bridge/model.json"])


def test_warm_up_populates_calibration_and_fan_curves(capsys):
    """Section 8 item 133: the warm-up's whole point is a document at a representative
    steady-state size, not one sitting at the store's near-empty floor. 900 ticks (75
    simulated minutes) is enough for at least one bay's SMART calibration to accept and,
    with the dwell scan the warm-up always appends, for the one fan model to reach an
    accepted curve -- both are asserted non-empty here rather than merely present-or-not,
    so a warm-up that silently regressed to the item's original, unpopulated state would
    fail this test the same way it would fail a reviewer reading the file by hand."""
    tool = _load_tool()
    assert tool.main(["--warm-ticks", "900", "--repeats", "2"]) == 0
    result = json.loads(capsys.readouterr().out)["result"]
    assert result["fan_curve_online_forced"] is True
    assert result["calibrated_bays"] > 0
    assert result["fan_curve_fit_accepted"] == ["case120"]
    # bigger than item 48's originally-measured floor (1424 bytes) by a comfortable
    # margin, not simply "greater than zero" -- a document this small could still be
    # missing a whole section.
    assert result["size_bytes"] > 2000


def test_bench_tool_refuses_a_legacy_config(capsys):
    tool = _load_tool()
    with pytest.raises(SystemExit):
        tool.main(
            [
                "--config",
                str(REPO_ROOT / "config.example.yaml"),
                "--warm-ticks",
                "5",
                "--repeats",
                "2",
            ]
        )
    assert "mpc.topology" in capsys.readouterr().err
