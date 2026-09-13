"""Tests for :mod:`aqua_bridge.recorder` (plan sections 5, 10, 11 milestone 8).

``record_from_tick``/``thermal_inputs`` are pure and tested directly against
hand-built :class:`~aqua_bridge.control.loop.TickResult` / record shapes;
:class:`Recorder` is tested against real files (rotation, malformed-write
tolerance); the end-to-end test drives the real :class:`~aqua_bridge.control.
loop.Loop` against :mod:`aqua_bridge.sim.das`'s truth plant and replays the
recording through :func:`aqua_bridge.control.thermal.update`, the same
integration ``tools/fit_model.py``/``tools/replay.py`` build on.
"""

from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Any

import pytest

from aqua_bridge.__main__ import PlantIO
from aqua_bridge.control import thermal
from aqua_bridge.control.loop import Loop, TickResult
from aqua_bridge.control.supervisor import Supervisor
from aqua_bridge.model import Mode, MpcCommand, MpcConfig, MpcState, PlantObservation
from aqua_bridge.recorder import (
    RECORD_VERSION,
    Recorder,
    chain_on_tick,
    iter_records,
    record_from_tick,
    thermal_inputs,
)
from aqua_bridge.sim import das as simdas
from das_fixtures import das_cfg

# ---------------------------------------------------------------------------
# helpers
# ---------------------------------------------------------------------------


def _obs(ts: float = 10.0) -> PlantObservation:
    return PlantObservation(
        temps={"air_a": 30.0, "prox_a1": 35.0},
        rpm={"fa1": 1200.0},
        pwm={"fa1": 0.6},
        ts=ts,
    )


def _das_diagnostics(**overrides: Any) -> dict[str, Any]:
    base: dict[str, Any] = {
        "time": {"status": "ok"},
        "gate": {
            "filtered": {"air_a": 30.0, "prox_a1": 35.0, "inlet": 25.0},
            "per_temp": {"air_a": True, "prox_a1": True, "inlet": False},
        },
        "zones": {
            "za": {"trusted": True, "fault": False},
            "zb": {"trusted": True, "fault": True},  # trusted this tick, but still faulted
            "zc": {"trusted": False, "fault": False},
        },
        "bays": {
            "a1": {
                "occupancy": "occupied",
                "class": "hdd",
                "serial": "S1",
                "calibration": {"slope": 0.72, "offset_c": -1.9, "accepted_once": True},
            },
            "a2": {"occupancy": "empty", "class": "ssd_sata", "serial": None, "calibration": None},
        },
        "prev_pwm": {"fa1": 0.55, "fb1": 0.8},
    }
    base.update(overrides)
    return base


def _result(cmd: MpcCommand, *, index: int = 3, applied: bool = True, **kw: Any) -> TickResult:
    return TickResult(
        index=index,
        obs=_obs(),
        mpc_cmd=cmd,
        cmd=cmd,
        state=MpcState.cold(),
        applied=applied,
        **kw,
    )


def _small_das_cfg(**changes: Any) -> MpcConfig:
    return das_cfg(setpoints={}, model_window_s=8.0, **changes)


# ---------------------------------------------------------------------------
# record_from_tick
# ---------------------------------------------------------------------------


def test_record_from_tick_extracts_das_fields() -> None:
    cfg = _small_das_cfg()
    cmd = MpcCommand(
        pwm={ch: 0.5 for ch in cfg.channels}, mode=Mode.AUTO, diagnostics=_das_diagnostics()
    )
    rec = record_from_tick(_result(cmd), cfg)

    assert rec["v"] == RECORD_VERSION
    assert rec["das"] is True
    assert rec["i"] == 3
    assert rec["mode"] == "auto"
    assert rec["applied"] is True
    # per_temp True, time ok -> trusted; "inlet" is False, dropped.
    assert rec["trusted_temps"] == {"air_a": 30.0, "prox_a1": 35.0}
    # za: trusted & not faulted; zb: trusted but faulted, excluded; zc: not trusted.
    assert rec["zones_ok"] == ["za"]
    assert set(rec["bays"]) == {"a1", "a2"}
    assert rec["prev"] == {"fa1": 0.55, "fb1": 0.8}
    assert rec["cmd"] == {ch: 0.5 for ch in cfg.channels}
    assert rec["read_error"] is None and rec["controller_error"] is None
    # round trips through json with no NaN/Infinity tokens
    json.dumps(rec, allow_nan=False)


def test_record_from_tick_legacy_config_has_no_das_fields(cfg: MpcConfig) -> None:
    diag = _das_diagnostics()  # a das-shaped diagnostics dict would never occur live, but
    # record_from_tick must ignore it when the *config* is legacy regardless.
    cmd = MpcCommand(pwm=dict.fromkeys(cfg.channels, 0.5), mode=Mode.AUTO, diagnostics=diag)
    rec = record_from_tick(_result(cmd), cfg)
    assert rec["das"] is False
    assert rec["trusted_temps"] == {}
    assert rec["zones_ok"] == []
    assert rec["bays"] == {}
    # temps/rpm/pwm/prev/cmd are still populated: legacy recordings still feed fit_fans.
    assert rec["temps"] == {"air_a": 30.0, "prox_a1": 35.0}
    assert rec["prev"] == {"fa1": 0.55, "fb1": 0.8}


def test_record_from_tick_tolerates_emergency_diagnostics() -> None:
    """A controller-error tick's command carries ``{"policy", "controller_error"}``
    only (loop.emergency_command) -- no gate/zones/bays/prev_pwm at all."""
    cfg = _small_das_cfg()
    cmd = MpcCommand(
        pwm=dict.fromkeys(cfg.channels, 0.8),
        mode=Mode.FALLBACK,
        diagnostics={"policy": "emergency", "controller_error": "boom"},
    )
    rec = record_from_tick(_result(cmd, controller_error="boom: boom"), cfg)
    assert rec["das"] is True
    assert rec["trusted_temps"] == {} and rec["zones_ok"] == [] and rec["bays"] == {}
    assert rec["prev"] == {}
    assert rec["controller_error"] == "boom: boom"
    json.dumps(rec, allow_nan=False)


@pytest.mark.parametrize(
    "gate",
    [
        None,
        "not a mapping",
        {"filtered": "nope", "per_temp": {"air_a": True}},
        {"filtered": {"air_a": 30.0}, "per_temp": "nope"},
    ],
)
def test_record_from_tick_tolerates_malformed_gate(gate: Any) -> None:
    cfg = _small_das_cfg()
    diag = _das_diagnostics(gate=gate)
    cmd = MpcCommand(pwm=dict.fromkeys(cfg.channels, 0.5), mode=Mode.AUTO, diagnostics=diag)
    rec = record_from_tick(_result(cmd), cfg)
    assert rec["trusted_temps"] == {}
    json.dumps(rec, allow_nan=False)


@pytest.mark.parametrize(
    "zones",
    [None, "nope", {"za": "nope"}, {"za": {"trusted": "yes", "fault": False}}],
)
def test_record_from_tick_tolerates_malformed_zones(zones: Any) -> None:
    cfg = _small_das_cfg()
    diag = _das_diagnostics(zones=zones)
    cmd = MpcCommand(pwm=dict.fromkeys(cfg.channels, 0.5), mode=Mode.AUTO, diagnostics=diag)
    rec = record_from_tick(_result(cmd), cfg)
    # the contract under test is only: never raises, always a list of strings,
    # regardless of what a broken "zones" diagnostics block contains.
    assert isinstance(rec["zones_ok"], list)
    assert all(isinstance(z, str) for z in rec["zones_ok"])
    json.dumps(rec, allow_nan=False)


def test_record_from_tick_sanitizes_non_finite_values() -> None:
    cfg = _small_das_cfg()
    obs = PlantObservation(
        temps={"air_a": float("nan"), "prox_a1": 35.0},
        rpm={"fa1": None},
        pwm={"fa1": float("inf")},
        ts=1.0,
    )
    cmd = MpcCommand(
        pwm=dict.fromkeys(cfg.channels, 0.5), mode=Mode.AUTO, diagnostics=_das_diagnostics()
    )
    result = TickResult(index=0, obs=obs, mpc_cmd=cmd, cmd=cmd, state=MpcState.cold(), applied=True)
    rec = record_from_tick(result, cfg)
    assert rec["temps"]["air_a"] is None
    assert rec["rpm"]["fa1"] is None
    assert rec["pwm"]["fa1"] is None
    json.dumps(rec, allow_nan=False)


# ---------------------------------------------------------------------------
# thermal_inputs
# ---------------------------------------------------------------------------


def test_thermal_inputs_extracts_occupancy_class_and_calibration() -> None:
    cfg = _small_das_cfg()
    rec = record_from_tick(
        _result(
            MpcCommand(
                pwm=dict.fromkeys(cfg.channels, 0.5), mode=Mode.AUTO, diagnostics=_das_diagnostics()
            )
        ),
        cfg,
    )
    ti = thermal_inputs(cfg, rec)
    assert ti["occupancy"] == {"a1": "occupied", "a2": "empty"}
    assert ti["classes"] == {"a1": "hdd", "a2": "ssd_sata"}
    assert ti["maps"] == {"a1": (0.72, -1.9)}  # a2 has no serial/calibration
    assert ti["u"] == {"fa1": 0.55, "fb1": 0.8}
    assert ti["zones_ok"] == ["za"]
    assert ti["temps"] == {"air_a": 30.0, "prox_a1": 35.0}


@pytest.mark.parametrize(
    "bays",
    [
        {},
        {"a1": "not a mapping"},
        {"a1": {"serial": "S1", "calibration": "nope"}},
        {
            "a1": {
                "serial": "S1",
                "calibration": {"accepted_once": False, "slope": 0.7, "offset_c": -2},
            }
        },
        {
            "a1": {
                "serial": "S1",
                "calibration": {"accepted_once": True, "slope": "nan", "offset_c": -2},
            }
        },
        {
            "a1": {
                "serial": None,
                "calibration": {"accepted_once": True, "slope": 0.7, "offset_c": -2},
            }
        },
    ],
)
def test_thermal_inputs_never_raises_on_malformed_bays(bays: Any) -> None:
    cfg = _small_das_cfg()
    ti = thermal_inputs(
        cfg, {"bays": bays, "ts": 1.0, "prev": {}, "trusted_temps": {}, "zones_ok": []}
    )
    assert ti["maps"] is None or "a1" not in ti["maps"]


def test_thermal_inputs_is_ready_for_thermal_update() -> None:
    """Feeding ``thermal_inputs`` straight into ``thermal.update`` never raises, on a
    hand-built record too (not just recorder-produced ones)."""
    cfg = _small_das_cfg()
    rec = {
        "ts": 5.0,
        "trusted_temps": {},
        "prev": {},
        "zones_ok": [],
        "bays": {},
        "rpm": {},
    }
    result = thermal.update(None, cfg, learn=True, **thermal_inputs(cfg, rec))
    assert result.summary["status"] == "prior"


# ---------------------------------------------------------------------------
# Recorder: file writing, rotation, robustness
# ---------------------------------------------------------------------------


def test_recorder_writes_one_json_line_per_tick(tmp_path: Path) -> None:
    cfg = _small_das_cfg()
    rec = Recorder(cfg, tmp_path / "sub" / "rec.jsonl")
    for i in range(5):
        cmd = MpcCommand(
            pwm=dict.fromkeys(cfg.channels, 0.5), mode=Mode.AUTO, diagnostics=_das_diagnostics()
        )
        rec.on_tick(_result(cmd, index=i))
    rec.close()
    records = iter_records(tmp_path / "sub" / "rec.jsonl")
    assert [r["i"] for r in records] == [0, 1, 2, 3, 4]


def test_recorder_rotates_and_keeps_backup_count(tmp_path: Path) -> None:
    cfg = _small_das_cfg()
    path = tmp_path / "rec.jsonl"
    rec = Recorder(cfg, path, max_bytes=400, backup_count=2)
    n = 60
    for i in range(n):
        cmd = MpcCommand(
            pwm=dict.fromkeys(cfg.channels, 0.5), mode=Mode.AUTO, diagnostics=_das_diagnostics()
        )
        rec.on_tick(_result(cmd, index=i))
    rec.close()
    assert path.exists()
    assert (path.with_name(path.name + ".1")).exists()
    assert (path.with_name(path.name + ".2")).exists()
    assert not (path.with_name(path.name + ".3")).exists()
    # oldest kept -> newest: .2, .1, the live file (empty when a rotation landed on
    # the very last write, since _rotate reopens an empty file right away)
    kept = [
        r["i"]
        for name in (".2", ".1", "")
        for r in iter_records(path.with_name(path.name + name) if name else path)
    ]
    assert kept  # something survived rotation
    assert kept[-1] == n - 1  # the newest tick is never lost
    assert kept == list(range(kept[0], n))  # a contiguous suffix, oldest dropped first


def test_recorder_on_tick_swallows_a_write_failure(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    cfg = _small_das_cfg()
    rec = Recorder(cfg, tmp_path / "rec.jsonl")
    cmd = MpcCommand(
        pwm=dict.fromkeys(cfg.channels, 0.5), mode=Mode.AUTO, diagnostics=_das_diagnostics()
    )

    def _boom(*a: Any, **kw: Any) -> Any:
        raise OSError("disk full")

    monkeypatch.setattr(Path, "open", _boom)
    rec.on_tick(_result(cmd))  # must not raise
    assert not (tmp_path / "rec.jsonl").exists()


def test_recorder_on_tick_swallows_a_broken_result(tmp_path: Path) -> None:
    cfg = _small_das_cfg()
    rec = Recorder(cfg, tmp_path / "rec.jsonl")
    rec.on_tick(object())  # type: ignore[arg-type]  # not a TickResult at all
    rec.close()


def test_recorder_close_is_idempotent_and_stops_further_writes(tmp_path: Path) -> None:
    cfg = _small_das_cfg()
    path = tmp_path / "rec.jsonl"
    rec = Recorder(cfg, path)
    cmd = MpcCommand(
        pwm=dict.fromkeys(cfg.channels, 0.5), mode=Mode.AUTO, diagnostics=_das_diagnostics()
    )
    rec.on_tick(_result(cmd, index=0))
    rec.close()
    rec.close()  # idempotent
    rec.on_tick(_result(cmd, index=1))  # a no-op after close
    records = iter_records(path)
    assert [r["i"] for r in records] == [0]


def test_iter_records_skips_malformed_lines(tmp_path: Path) -> None:
    path = tmp_path / "rec.jsonl"
    path.write_text('{"i": 0}\nnot json\n[]\n{"i": 1}\n\n', encoding="utf-8")
    records = iter_records(path)
    assert [r["i"] for r in records] == [0, 1]


def test_chain_on_tick_calls_every_hook_and_isolates_failures() -> None:
    calls: list[str] = []

    def ok(result: TickResult) -> None:
        calls.append("ok")

    def boom(result: TickResult) -> None:
        calls.append("boom")
        raise RuntimeError("nope")

    combined = chain_on_tick(ok, None, boom, ok)
    cfg = _small_das_cfg()
    cmd = MpcCommand(pwm=dict.fromkeys(cfg.channels, 0.5), mode=Mode.AUTO)
    combined(_result(cmd))  # must not raise despite boom()
    assert calls == ["ok", "boom", "ok"]


def test_chain_on_tick_with_no_hooks_is_a_safe_no_op() -> None:
    combined = chain_on_tick(None, None)
    cfg = _small_das_cfg()
    cmd = MpcCommand(pwm=dict.fromkeys(cfg.channels, 0.5), mode=Mode.AUTO)
    combined(_result(cmd))


# ---------------------------------------------------------------------------
# end to end: a real Loop against sim/das.py, replayed through thermal.update
# ---------------------------------------------------------------------------


def test_recorded_das_closed_loop_replays_through_thermal_update(tmp_path: Path) -> None:
    cfg = _small_das_cfg()
    topology = simdas.topology_from_config(cfg)
    heat_schedule = {"a2": [(0.0, 0.2), (50.0, 1.0), (100.0, 0.2), (150.0, 1.0), (200.0, 0.2)]}
    plant = simdas.build_das_plant(
        topology=topology,
        preset="basic",
        seed=1,
        dt=cfg.dt,
        initial_pwm=dict(cfg.fallback_pwm),
        heat_schedule=heat_schedule,
    )
    io = PlantIO(plant, cfg)
    supervisor = Supervisor(cfg, version="test")
    path = tmp_path / "rec.jsonl"
    recorder = Recorder(cfg, path, max_bytes=10_000_000)
    loop = Loop(io, io, cfg, supervisor, on_tick=recorder.on_tick)

    ticks = 250
    for _ in range(ticks):
        result = loop.tick()
        assert result.controller_error is None  # the recorder must never break the loop
    recorder.close()

    records = iter_records(path)
    assert len(records) == ticks
    assert all(r["das"] for r in records)
    # every record round trips (it was written by json.dumps(allow_nan=False) already,
    # but iter_records re-parses it from disk, so this also proves no line was skipped)
    assert [r["i"] for r in records] == list(range(ticks))

    st = thermal.structure(cfg)
    mem: Any = None
    result = None
    for rec in records:
        result = thermal.update(mem, cfg, learn=True, **thermal_inputs(cfg, rec))
        mem = result.memory
    assert result is not None
    summary = result.summary
    assert summary["status"] in thermal.STATUSES
    assert summary["error"] is None
    # at least one zone actually saw closed windows: the recording carried real signal
    assert any(z["windows"] > 0 for z in summary["zones"].values())
    for z in summary["zones"].values():
        if z["pred_err_c"] is not None:
            assert math.isfinite(z["pred_err_c"])
    # the whole memory is finite JSON (thermal.py's own contract)
    json.dumps(mem, allow_nan=False)
    json.dumps(summary, allow_nan=False)
    # structure fingerprint matches: a fresh structure() call agrees with what the
    # recording was made against
    assert mem["fp"] == st.fingerprint
