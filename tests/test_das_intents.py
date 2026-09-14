"""DAS-mode intents and views: ``SetLimit`` (``POST /api/limit``, MQTT ``cmd/limit``),
``SetBay`` (``POST /api/bay``, MQTT ``cmd/bay``), ``GET /api/estimate``, ``GET /api/bays``,
``GET /api/model`` and the HA ``model_status`` / ``model_pred_err_c`` sensors,
the Home Assistant drive entities and the DAS preset semantics (plan sections 4 and 7).
Legacy configs keep ``/api/setpoint`` and the legacy presets unchanged.
"""

from __future__ import annotations

import asyncio
import dataclasses
import json
import math
import socket
from typing import Any

import pytest
from hypothesis import given, settings
from hypothesis import strategies as st

from aqua_bridge.control.intents import (
    INTENT_KINDS,
    ControlMode,
    IntentError,
    IntentInvalid,
    Preset,
    SetBay,
    SetLimit,
    SetPreset,
    SetSetpoint,
    parse_intent,
)
from aqua_bridge.control.mpc import step
from aqua_bridge.control.supervisor import (
    PRESETS,
    RuntimeBays,
    RuntimeLimits,
    Supervisor,
    apply_preset,
)
from aqua_bridge.model import ConfigError, MpcConfig, MpcState
from aqua_bridge.publishers.mqtt_ha import build_discovery_entities, command_topics, parse_command
from das_fixtures import das_obs

NODE = "aqua-bridge"


@pytest.fixture
def dcfg(das_example_cfg: MpcConfig) -> MpcConfig:
    return das_example_cfg


# ---------------------------------------------------------------------------
# intent parsing
# ---------------------------------------------------------------------------


def test_parse_limit_by_bay_and_by_class():
    assert parse_intent("limit", {"bay": "b03", "limit_c": 45}) == SetLimit(limit_c=45.0, bay="b03")
    assert parse_intent("limit", {"class": "hdd", "limit_c": 48.5}) == SetLimit(
        limit_c=48.5, drive_class="hdd"
    )
    assert "limit" in INTENT_KINDS


@pytest.mark.parametrize(
    "body",
    [
        {"limit_c": 45},
        {"bay": "b03", "class": "hdd", "limit_c": 45},
        {"bay": "b03"},
        {"bay": "b03", "limit_c": "hot"},
        {"bay": "b03", "limit_c": math.nan},
        {"bay": "b03", "limit_c": True},
        {"bay": "", "limit_c": 45},
        {"bay": 3, "limit_c": 45},
        {"bay": "b03", "limit_c": 45, "extra": 1},
        [],
    ],
)
def test_parse_limit_rejects_malformed_bodies(body):
    with pytest.raises(IntentInvalid):
        parse_intent("limit", body)


# ---------------------------------------------------------------------------
# Supervisor: limits
# ---------------------------------------------------------------------------


def test_class_limit_tightens_every_bay_of_the_class(dcfg):
    sup = Supervisor(dcfg)
    sup.submit(SetLimit(limit_c=46.0, drive_class="hdd"))
    eff = sup.effective_config()
    assert eff.drive_classes["hdd"].limit_c == 46.0
    assert all(eff.bay_limit(b) == 46.0 for b in eff.topology.bays)
    assert sup.base_config.drive_classes["hdd"].limit_c == 50.0  # base untouched
    assert sup.limits == RuntimeLimits(classes={"hdd": 46.0})
    snap = sup.snapshot()
    assert snap.limits["classes"]["hdd"] == 46.0 and snap.limits["bays"]["b01"] == 46.0
    assert snap.state_payload()["limits"] == snap.limits


def test_bay_limit_and_class_limit_combine_to_the_lower(dcfg):
    sup = Supervisor(dcfg)
    sup.submit(SetLimit(limit_c=44.0, bay="b03"))
    eff = sup.effective_config()
    assert eff.bay_limit("b03") == 44.0 and eff.bay_limit("b04") == 50.0
    sup.submit(SetLimit(limit_c=42.0, drive_class="hdd"))
    eff = sup.effective_config()
    assert eff.bay_limit("b03") == 42.0 and eff.bay_limit("b04") == 42.0
    # restoring the class leaves the bay's own limit
    sup.submit(SetLimit(limit_c=50.0, drive_class="hdd"))
    eff = sup.effective_config()
    assert eff.bay_limit("b03") == 44.0 and eff.bay_limit("b04") == 50.0
    sup.submit(SetLimit(limit_c=50.0, bay="b03"))  # the configured value is accepted
    assert sup.effective_config().bay_limit("b03") == 50.0


@pytest.mark.parametrize(
    "intent, match",
    [
        (SetLimit(limit_c=50.5, drive_class="hdd"), "cannot exceed"),
        (SetLimit(limit_c=66.0, drive_class="ssd_sata"), "cannot exceed"),
        (SetLimit(limit_c=55.0, bay="b01"), "cannot exceed"),
        (SetLimit(limit_c=-20.0, bay="b01"), "must lie in"),
        (SetLimit(limit_c=40.0, bay="b99"), "unknown bay"),
        (SetLimit(limit_c=40.0, drive_class="tape"), "unknown drive class"),
    ],
)
def test_invalid_limits_are_rejected_and_change_nothing(dcfg, intent, match):
    sup = Supervisor(dcfg)
    before = sup.effective_config()
    with pytest.raises(IntentInvalid, match=match):
        sup.submit(intent)
    assert sup.effective_config() is before
    assert sup.limits == RuntimeLimits()


def test_a_bay_limit_ceiling_is_the_configured_bay_limit(dcfg):
    m = dcfg.to_dict()
    m["topology"]["bays"]["b02"]["limit_c"] = 45.0
    sup = Supervisor(MpcConfig.from_mapping(m))
    with pytest.raises(IntentInvalid, match="cannot exceed"):
        sup.submit(SetLimit(limit_c=47.0, bay="b02"))
    sup.submit(SetLimit(limit_c=45.0, bay="b02"))


def test_legacy_config_rejects_limits_and_keeps_its_payload(cfg):
    sup = Supervisor(cfg)
    with pytest.raises(IntentInvalid, match="DAS config"):
        sup.submit(SetLimit(limit_c=40.0, drive_class="hdd"))
    snap = sup.snapshot()
    assert snap.limits == {}
    assert "limits" not in snap.state_payload()
    with pytest.raises(ConfigError):
        apply_preset(cfg, cfg.setpoints, Preset.NORMAL, RuntimeLimits(classes={"hdd": 40.0}))


def test_das_config_has_no_setpoint_to_set(dcfg):
    with pytest.raises(IntentInvalid, match="no setpoint"):
        Supervisor(dcfg).submit(SetSetpoint(channel="air_z0", celsius=30.0))


@settings(max_examples=60, deadline=None)
@given(
    body=st.dictionaries(
        st.sampled_from(["bay", "class", "limit_c", "x"]),
        st.one_of(
            st.floats(allow_nan=True, allow_infinity=True),
            st.integers(-1000, 1000),
            st.sampled_from(["b01", "b15", "hdd", "nvme", "", "zz"]),
            st.none(),
            st.booleans(),
        ),
    )
)
def test_fuzzed_limit_bodies_never_raise_unexpected_or_exceed(body: dict[str, Any]):
    from aqua_bridge.config import load_config
    from conftest import EXAMPLE_DAS_CONFIG

    base = load_config(EXAMPLE_DAS_CONFIG).mpc
    sup = Supervisor(base)
    try:
        sup.submit(parse_intent("limit", body))
    except IntentError:
        return
    eff = sup.effective_config()
    for bay in eff.topology.bays:
        assert base.temp_min_c < eff.bay_limit(bay) <= base.bay_limit(bay)


# ---------------------------------------------------------------------------
# presets in DAS mode
# ---------------------------------------------------------------------------


def test_das_presets_shift_comfort_and_noise_weight(dcfg):
    quiet = apply_preset(dcfg, {}, Preset.QUIET)
    cool = apply_preset(dcfg, {}, Preset.COOL)
    normal = apply_preset(dcfg, {}, Preset.NORMAL)
    assert normal == dcfg
    for name, dc in dcfg.drive_classes.items():
        assert quiet.drive_classes[name].comfort_c == dc.comfort_c - 2.0
        assert cool.drive_classes[name].comfort_c == dc.comfort_c + 2.0
        assert quiet.drive_classes[name].limit_c == cool.drive_classes[name].limit_c == dc.limit_c
    assert quiet.noise.weight_noise == 2.0 * dcfg.noise.weight_noise
    assert cool.noise.weight_noise == 0.5 * dcfg.noise.weight_noise
    # nothing of the legacy transform in DAS mode
    for c in (quiet, cool):
        assert (c.pi_kp, c.pi_ki, c.weight_dpwm) == (dcfg.pi_kp, dcfg.pi_ki, dcfg.weight_dpwm)
    assert PRESETS[Preset.QUIET].comfort_offset_c == -2.0
    assert PRESETS[Preset.COOL].noise_weight_scale == 0.5


def test_quiet_comfort_is_floored_at_zero(dcfg):
    m = dcfg.to_dict()
    m["drive_classes"]["nvme"]["comfort_c"] = 1.0
    quiet = apply_preset(MpcConfig.from_mapping(m), {}, Preset.QUIET)
    assert quiet.drive_classes["nvme"].comfort_c == 0.0


def test_legacy_presets_are_unchanged(cfg):
    quiet = apply_preset(cfg, cfg.setpoints, Preset.QUIET)
    assert quiet == dataclasses.replace(
        cfg,
        setpoints={k: v + 2.0 for k, v in cfg.setpoints.items()},
        pi_kp=cfg.pi_kp * 0.5,
        pi_ki=cfg.pi_ki * 0.5,
        weight_dpwm=cfg.weight_dpwm * 2.0,
    )


def test_limits_survive_a_preset_change(dcfg):
    sup = Supervisor(dcfg)
    sup.submit(SetLimit(limit_c=45.0, bay="b07"))
    sup.submit(SetPreset(name=Preset.COOL))
    eff = sup.effective_config()
    assert eff.bay_limit("b07") == 45.0 and eff.drive_classes["hdd"].comfort_c == 7.0


def test_quiet_asks_for_less_fan_than_cool_on_the_same_drives(dcfg):
    """Same observation: the quiet soft target is 4 degC above the cool one.

    The estimator sees each preset's own previous command, so after the first tick the
    two runs' estimates (and sigma, inside soft) differ by a few thousandths of a degree:
    the target ``limit - comfort`` differs by exactly 4, the error by 4 up to that."""
    obs = das_obs(dcfg, 0.0, pwm=0.5, **{f"prox_b{n:02d}": 42.0 for n in range(1, 16)})
    out, target = {}, {}
    for preset in (Preset.QUIET, Preset.COOL):
        eff = apply_preset(dcfg, {}, preset)
        state = MpcState.cold()
        for i in range(3):
            obs = dataclasses.replace(obs, ts=float(i) * eff.dt)
            cmd, state = step(obs, eff, state)
        diag = cmd.diagnostics["solver_diag"]
        out[preset] = diag["error"]
        target[preset] = {}
        for ch, bay in diag["worst_bay"].items():
            entry = cmd.diagnostics["estimates"][bay]
            assert diag["error"][ch] == pytest.approx(entry["t_c"] - entry["soft_c"])
            target[preset][ch] = entry["soft_c"] + entry["margin_c"]
    for ch in dcfg.channels:
        assert target[Preset.QUIET][ch] == pytest.approx(target[Preset.COOL][ch] + 4.0)
        assert out[Preset.COOL][ch] == pytest.approx(out[Preset.QUIET][ch] + 4.0, abs=0.01)


# ---------------------------------------------------------------------------
# MQTT
# ---------------------------------------------------------------------------


def test_mqtt_limit_topics_and_parsing(dcfg, cfg):
    topics = command_topics(NODE, dcfg)
    assert topics["limit/b01"] == "aqua-bridge/cmd/limit/b01"
    assert topics["limit/class/hdd"] == "aqua-bridge/cmd/limit/class/hdd"
    assert not any(k.startswith("setpoint/") for k in topics)
    assert not any(k.startswith("limit") for k in command_topics(NODE, cfg))
    assert parse_command(NODE, "aqua-bridge/cmd/limit/b01", b"44.5") == SetLimit(
        limit_c=44.5, bay="b01"
    )
    assert parse_command(NODE, "aqua-bridge/cmd/limit/class/hdd", "47") == SetLimit(
        limit_c=47.0, drive_class="hdd"
    )
    for topic, payload in (
        ("aqua-bridge/cmd/limit/b01", b"hot"),
        ("aqua-bridge/cmd/limit/b01", b"nan"),
        ("aqua-bridge/cmd/limit/", b"40"),
        ("aqua-bridge/cmd/limit/class/", b"40"),
    ):
        assert parse_command(NODE, topic, payload) is None, topic


def test_mqtt_limit_entities_in_das_mode_only(dcfg, cfg):
    entities = build_discovery_entities(
        dcfg, node_id=NODE, discovery_prefix="homeassistant", control_mode=ControlMode.AUTO
    )
    limits = {e.object_id: e for e in entities if e.object_id.startswith("limit_")}
    assert set(limits) == {"limit_hdd", "limit_ssd_sata", "limit_nvme"}
    hdd = limits["limit_hdd"].payload
    assert hdd["command_topic"] == "aqua-bridge/cmd/limit/class/hdd"
    assert hdd["max"] == 50.0 and hdd["min"] == dcfg.temp_min_c
    assert "limits.classes.hdd" in hdd["value_template"]
    assert not any(e.object_id.startswith("setpoint_") for e in entities)
    legacy = build_discovery_entities(
        cfg, node_id=NODE, discovery_prefix="homeassistant", control_mode=ControlMode.AUTO
    )
    assert not any(e.object_id.startswith("limit_") for e in legacy)


# ---------------------------------------------------------------------------
# HTTP
# ---------------------------------------------------------------------------


def _can_bind_localhost() -> bool:
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.bind(("127.0.0.1", 0))
    except OSError:
        return False
    return True


needs_socket = pytest.mark.skipif(
    not _can_bind_localhost(), reason="binding a localhost TCP socket is denied here (sandbox)"
)


async def _post_all(sup: Supervisor, posts: list[tuple[str, Any]]) -> list[tuple[int, Any]]:
    from aiohttp.test_utils import TestClient, TestServer

    from aqua_bridge.publishers.http import create_app

    client = TestClient(TestServer(create_app(sup)))
    await client.start_server()
    out = []
    try:
        for path, body in posts:
            if path.startswith("GET "):
                resp = await client.get(path[4:])
            else:
                resp = await client.post(
                    path,
                    data=json.dumps(body).encode(),
                    headers={"Content-Type": "application/json"},
                )
            out.append((resp.status, await resp.json()))
    finally:
        await client.close()
    return out


@needs_socket
def test_http_limit_on_a_das_config(dcfg):
    sup = Supervisor(dcfg)
    results = asyncio.run(
        _post_all(
            sup,
            [
                ("/api/limit", {"class": "hdd", "limit_c": 47}),
                ("/api/limit", {"bay": "b05", "limit_c": 43.5}),
                ("/api/limit", {"bay": "b05", "limit_c": 70}),
                ("/api/limit", {"bay": "b05"}),
                ("/api/setpoint", {"channel": "air_z0", "celsius": 30}),
                ("GET /api/state", None),
            ],
        )
    )
    statuses = [s for s, _ in results]
    assert statuses == [200, 200, 400, 400, 400, 200]
    assert "cannot exceed" in results[2][1]["error"]
    state = results[-1][1]
    assert state["limits"]["classes"]["hdd"] == 47.0
    assert state["limits"]["bays"]["b05"] == 43.5 and state["limits"]["bays"]["b06"] == 47.0


@needs_socket
def test_http_limit_on_a_legacy_config_is_400_and_setpoint_still_works(cfg):
    sup = Supervisor(cfg)
    temp = next(iter(cfg.setpoints))
    results = asyncio.run(
        _post_all(
            sup,
            [
                ("/api/limit", {"class": "hdd", "limit_c": 47}),
                ("/api/setpoint", {"channel": temp, "celsius": 33.0}),
                ("GET /api/state", None),
            ],
        )
    )
    assert [s for s, _ in results] == [400, 200, 200]
    assert "limits" not in results[-1][1]
    assert sup.setpoints[temp] == 33.0


def test_presets_on_a_zoned_setpoint_config_keep_the_setpoint_semantics():
    """A zoned config that still regulates on setpoints (not on drive limits) must not
    lose its presets: ``cool`` lowers the setpoints and ``quiet`` raises them, exactly as
    in legacy mode, otherwise ``cool`` would be a silent no-op for the PI solver."""
    from das_fixtures import das_cfg

    zcfg = das_cfg()
    assert zcfg.is_das and zcfg.setpoints and not zcfg.regulates_drive_limits
    for preset, effect in PRESETS.items():
        eff = apply_preset(zcfg, zcfg.setpoints, preset)
        assert eff.setpoints == {k: v + effect.setpoint_offset_c for k, v in zcfg.setpoints.items()}
        assert eff.pi_kp == pytest.approx(zcfg.pi_kp * effect.gain_scale)
        assert eff.pi_ki == pytest.approx(zcfg.pi_ki * effect.gain_scale)
        assert eff.weight_dpwm == pytest.approx(zcfg.weight_dpwm * effect.move_penalty_scale)


# ---------------------------------------------------------------------------
# bays: SetBay (POST /api/bay, MQTT cmd/bay/<bay>), GET /api/estimate, GET /api/bays
# ---------------------------------------------------------------------------


def test_parse_bay_intent():
    assert parse_intent("bay", {"bay": "b03", "occupied": False}) == SetBay(
        bay="b03", changes={"occupied": False}
    )
    intent = parse_intent(
        "bay", {"bay": "b03", "class": "ssd_sata", "serial": "X1", "occupied": "auto"}
    )
    assert intent.changes == {"occupied": "auto", "class": "ssd_sata", "serial": "X1"}
    assert parse_intent("bay", {"bay": "b03", "serial": None}).changes == {"serial": None}
    assert "bay" in INTENT_KINDS


@pytest.mark.parametrize(
    "body",
    [
        {},
        {"bay": "b03"},
        {"occupied": True},
        {"bay": "b03", "occupied": "yes"},
        {"bay": "b03", "occupied": 1},
        {"bay": "b03", "class": 3},
        {"bay": "b03", "serial": ""},
        {"bay": "", "occupied": True},
        {"bay": "b03", "occupied": True, "limit_c": 40},
        "b03",
    ],
)
def test_parse_bay_rejects_malformed_bodies(body):
    with pytest.raises(IntentInvalid):
        parse_intent("bay", body)


def test_set_bay_declares_occupancy_class_and_serial_and_null_restores(dcfg):
    sup = Supervisor(dcfg)
    sup.submit(SetBay(bay="b04", changes={"occupied": False, "class": "ssd_sata"}))
    sup.submit(SetBay(bay="b05", changes={"serial": "WD-1"}))
    eff = sup.effective_config()
    assert eff.topology.bays["b04"].occupied is False and eff.bay_class("b04") == "ssd_sata"
    assert eff.topology.bays["b05"].serial == "WD-1"
    assert sup.base_config.topology.bays["b04"].occupied == "auto"  # base untouched
    snap = sup.snapshot()
    assert snap.bays["b04"] == {
        "zone": "z0",
        "occupied": False,
        "class": "ssd_sata",
        "serial": None,
    }
    assert snap.state_payload()["bays"]["b05"]["serial"] == "WD-1"
    sup.submit(SetBay(bay="b04", changes={"occupied": None}))
    eff = sup.effective_config()
    assert eff.topology.bays["b04"].occupied == "auto" and eff.bay_class("b04") == "ssd_sata"
    sup.submit(SetBay(bay="b04", changes={"class": None}))
    assert "b04" not in sup.bays.bays and sup.effective_config().bay_class("b04") == "hdd"
    # declarations survive presets and limits
    sup.submit(SetPreset(name=Preset.COOL))
    sup.submit(SetLimit(limit_c=45.0, bay="b05"))
    eff = sup.effective_config()
    assert eff.topology.bays["b05"].serial == "WD-1" and eff.bay_limit("b05") == 45.0


@pytest.mark.parametrize(
    "intent, match",
    [
        (SetBay(bay="b99", changes={"occupied": True}), "unknown bay"),
        (SetBay(bay="b01", changes={"class": "tape"}), "unknown drive class"),
    ],
)
def test_set_bay_rejects_unknown_names(dcfg, intent, match):
    sup = Supervisor(dcfg)
    before = sup.effective_config()
    with pytest.raises(IntentInvalid, match=match):
        sup.submit(intent)
    assert sup.effective_config() is before


def test_set_bay_rejects_a_serial_declared_twice_and_changes_nothing(dcfg):
    sup = Supervisor(dcfg)
    sup.submit(SetBay(bay="b01", changes={"serial": "S"}))
    before = sup.effective_config()
    with pytest.raises(IntentInvalid, match="declared on both"):
        sup.submit(SetBay(bay="b02", changes={"serial": "S"}))
    assert sup.effective_config() is before and "b02" not in sup.bays.bays


def test_set_bay_on_a_legacy_config_is_invalid(cfg):
    sup = Supervisor(cfg)
    with pytest.raises(IntentInvalid, match="DAS config"):
        sup.submit(SetBay(bay="b01", changes={"occupied": True}))
    assert "bays" not in sup.snapshot().state_payload()
    with pytest.raises(ConfigError):
        apply_preset(cfg, cfg.setpoints, Preset.NORMAL, None, RuntimeBays({"b": {"serial": "x"}}))


def test_a_bay_declared_empty_at_runtime_loses_its_constraints_in_step(dcfg):
    sup = Supervisor(dcfg)
    obs = das_obs(dcfg, 0.0, pwm=0.5)
    cmd, state = step(obs, sup.effective_config(), MpcState.cold())
    assert "b07" in cmd.diagnostics["estimates"]
    sup.submit(SetBay(bay="b07", changes={"occupied": False}))
    cmd, state = step(dataclasses.replace(obs, ts=dcfg.dt), sup.effective_config(), state)
    assert "b07" not in cmd.diagnostics["estimates"]
    assert cmd.diagnostics["bays"]["b07"]["occupancy"] == "empty"


def test_mqtt_bay_topics_and_parsing(dcfg, cfg):
    topics = command_topics(NODE, dcfg)
    assert topics["bay/b07"] == "aqua-bridge/cmd/bay/b07"
    assert not any(k.startswith("bay/") for k in command_topics(NODE, cfg))
    for payload, occupied in (("occupied", True), (b"empty", False), (" auto ", "auto")):
        assert parse_command(NODE, "aqua-bridge/cmd/bay/b07", payload) == SetBay(
            bay="b07", changes={"occupied": occupied}
        )
    for topic, payload in (
        ("aqua-bridge/cmd/bay/b07", b"yes"),
        ("aqua-bridge/cmd/bay/b07", b""),
        ("aqua-bridge/cmd/bay/", b"empty"),
    ):
        assert parse_command(NODE, topic, payload) is None, (topic, payload)


def test_mqtt_drive_entities_in_das_mode_only(dcfg, cfg):
    entities = build_discovery_entities(
        dcfg, node_id=NODE, discovery_prefix="homeassistant", control_mode=ControlMode.AUTO
    )
    by_id = {e.object_id: e for e in entities}
    assert len(by_id) == len(entities)
    for bay in dcfg.topology.bays:
        temp = by_id[f"drive_temp_{bay}"]
        assert temp.component == "sensor" and temp.payload["device_class"] == "temperature"
        assert f"estimates.{bay}.t_c" in temp.payload["value_template"]
        assert (
            f"estimates.{bay}.limit_margin_c"
            in by_id[f"drive_margin_{bay}"].payload["value_template"]
        )
        assert f"estimates.{bay}.sigma_c" in by_id[f"drive_sigma_{bay}"].payload["value_template"]
        occupied = by_id[f"bay_occupied_{bay}"]
        assert occupied.component == "binary_sensor"
        assert (
            occupied.config_topic == f"homeassistant/binary_sensor/{NODE}/bay_occupied_{bay}/config"
        )
        assert f"bays.{bay}.occupancy" in occupied.payload["value_template"]
        assert occupied.payload["device_class"] == "occupancy"
    legacy = build_discovery_entities(
        cfg, node_id=NODE, discovery_prefix="homeassistant", control_mode=ControlMode.AUTO
    )
    assert not any(e.object_id.startswith(("drive_", "bay_")) for e in legacy)


def test_limit_margin_is_the_hard_target_minus_the_estimate(dcfg):
    cmd, _ = step(das_obs(dcfg, 0.0, pwm=0.5), dcfg, MpcState.cold())
    for entry in cmd.diagnostics["estimates"].values():
        assert entry["limit_margin_c"] == pytest.approx(entry["hard_c"] - entry["t_c"])


@needs_socket
def test_http_bay_estimate_and_bays_on_a_das_config(dcfg):
    sup = Supervisor(dcfg)
    obs = das_obs(dcfg, 0.0, pwm=0.5)
    before = asyncio.run(_post_all(sup, [("GET /api/estimate", None), ("GET /api/bays", None)]))
    assert before[0] == (200, {"estimates": {}, "estimator": {}})
    assert before[1][0] == 200 and before[1][1]["bays"]["b01"]["estimator"] is None
    cmd, state = step(obs, sup.effective_config(), MpcState.cold())
    sup.record_tick(obs=obs, mpc_cmd=cmd, cmd=cmd, state=state, applied=True, usb_present=True)
    results = asyncio.run(
        _post_all(
            sup,
            [
                ("/api/bay", {"bay": "b02", "occupied": False, "serial": "WD-9"}),
                ("/api/bay", {"bay": "b02", "class": "tape"}),
                ("/api/bay", {"bay": "b77", "occupied": True}),
                ("/api/bay", {"bay": "b02", "occupied": "maybe"}),
                ("/api/bay", {"bay": "b03", "serial": "WD-9"}),
                ("GET /api/estimate", None),
                ("GET /api/bays", None),
            ],
        )
    )
    assert [s for s, _ in results] == [200, 400, 400, 400, 400, 200, 200]
    estimate = results[5][1]
    assert set(estimate["estimates"]) == set(dcfg.topology.bays)
    assert estimate["estimates"]["b01"]["source"] == "estimator"
    assert estimate["estimator"]["status"] == "ok" and set(estimate["estimator"]["zones"]) == {
        "z0",
        "z1",
        "z2",
        "z3",
    }
    bays = results[6][1]["bays"]
    assert bays["b02"]["declared"] == {
        "zone": "z0",
        "occupied": False,
        "class": "hdd",
        "serial": "WD-9",
    }
    assert bays["b02"]["estimator"]["occupancy"] == "occupied"  # seen before the declaration
    assert "candidates" in bays["b01"]["estimator"]


def test_mqtt_thermal_model_entities_in_das_mode_only(dcfg, cfg):
    entities = build_discovery_entities(
        dcfg, node_id=NODE, discovery_prefix="homeassistant", control_mode=ControlMode.AUTO
    )
    by_id = {e.object_id: e for e in entities}
    status = by_id["model_status"]
    assert status.component == "sensor"
    assert "diagnostics.thermal.status" in status.payload["value_template"]
    assert "default('off')" in status.payload["value_template"]
    assert "unit_of_measurement" not in status.payload and "state_class" not in status.payload
    err = by_id["model_pred_err_c"]
    assert "diagnostics.thermal.pred_err_c" in err.payload["value_template"]
    assert err.payload["unit_of_measurement"] == "°C"
    legacy = build_discovery_entities(
        cfg, node_id=NODE, discovery_prefix="homeassistant", control_mode=ControlMode.AUTO
    )
    assert not any(e.object_id.startswith("model_") for e in legacy)


@needs_socket
def test_http_model_view_on_a_das_config(dcfg):
    for shadow in (False, True):
        c = dataclasses.replace(dcfg, model_shadow=shadow)
        sup = Supervisor(c)
        obs = das_obs(c, 0.0, pwm=0.5)
        cmd, state = step(obs, sup.effective_config(), MpcState.cold())
        sup.record_tick(obs=obs, mpc_cmd=cmd, cmd=cmd, state=state, applied=True, usb_present=True)
        ((status, body),) = asyncio.run(_post_all(sup, [("GET /api/model", None)]))
        assert status == 200
        assert body["parameters"]["E"]["unit"] == "W/K" and body["parameters"]["k"]["lo"] == 0.05
        assert set(body["calibration"]) == set(c.topology.bays)
        assert body["calibration"]["b01"]["calibrated"] is False
        assert body["store"] == {"source": "off"}  # no model store behind this state
        if shadow:
            assert body["thermal"]["status"] == "prior"
            assert set(body["thermal"]["zones"]) == {"z0", "z1", "z2", "z3"}
            assert "E.z0.xt1" in body["thermal"]["zones"]["z0"]["theta"]
            assert set(body["thermal"]["bays"]["b01"]["theta"]) == {"q_s.b01", "g0.b01", "k.b01"}
        else:
            assert body["thermal"] == {"status": "off"}


@needs_socket
def test_http_estimate_and_bays_on_a_legacy_config_are_404(cfg):
    results = asyncio.run(
        _post_all(
            Supervisor(cfg),
            [
                ("GET /api/estimate", None),
                ("GET /api/bays", None),
                ("/api/bay", {"bay": "b01", "occupied": True}),
                ("GET /api/model", None),
            ],
        )
    )
    assert [s for s, _ in results] == [404, 404, 400, 404]
    assert "DAS config" in results[0][1]["error"]
    assert "DAS config" in results[3][1]["error"]


@needs_socket
@settings(max_examples=40, deadline=None)
@given(
    body=st.dictionaries(
        st.sampled_from(["bay", "occupied", "class", "serial", "x"]),
        st.one_of(
            st.sampled_from(["b01", "b15", "hdd", "auto", "", "zz"]),
            st.booleans(),
            st.none(),
            st.integers(-3, 3),
            st.floats(allow_nan=True, allow_infinity=True),
        ),
    )
)
def test_fuzzed_bay_bodies_never_5xx(body: dict[str, Any]):
    from aqua_bridge.config import load_config
    from conftest import EXAMPLE_DAS_CONFIG

    sup = Supervisor(load_config(EXAMPLE_DAS_CONFIG).mpc)
    ((status, payload),) = asyncio.run(_post_all(sup, [("/api/bay", body)]))
    assert status in (200, 400), (status, payload)
