"""Tests for aqua_bridge.publishers.mqtt_ha (PROJECT.md section 7)."""

from __future__ import annotations

import json
from typing import Any

import pytest

from aqua_bridge.control.intents import (
    Calibrate,
    ClearOverride,
    ControlMode,
    IntentConflict,
    IntentInvalid,
    SetBay,
    SetMode,
    SetPreset,
    SetPwm,
    SetSetpoint,
)
from aqua_bridge.control.supervisor import Supervisor
from aqua_bridge.model import MpcConfig
from aqua_bridge.publishers.inputs import SmartInbox, smart_topic_filter
from aqua_bridge.publishers.mqtt_ha import (
    MqttEntity,
    MqttSetupError,
    availability_topic,
    build_discovery_entities,
    command_topics,
    host_sensor_specs,
    parse_command,
    state_payload,
    state_topic,
    topic_matches,
    validate_mqtt_section,
)

NODE_ID = "aqua-bridge"
PREFIX = "homeassistant"


# --- topics ------------------------------------------------------------


def test_availability_and_state_topics() -> None:
    assert availability_topic(NODE_ID) == "aqua-bridge/status"
    assert state_topic(NODE_ID) == "aqua-bridge/state"


def test_command_topics_cover_setpoints_and_channels(cfg: MpcConfig) -> None:
    topics = command_topics(NODE_ID, cfg)
    assert topics["mode"] == "aqua-bridge/cmd/mode"
    assert topics["preset"] == "aqua-bridge/cmd/preset"
    assert topics["auto"] == "aqua-bridge/cmd/auto"
    for temp in cfg.setpoints:
        assert topics[f"setpoint/{temp}"] == f"aqua-bridge/cmd/setpoint/{temp}"
    for ch in cfg.channels:
        assert topics[f"pwm/{ch}"] == f"aqua-bridge/cmd/pwm/{ch}"


# --- discovery entities --------------------------------------------------


def test_discovery_has_host_sensors(cfg: MpcConfig) -> None:
    entities = build_discovery_entities(
        cfg, node_id=NODE_ID, discovery_prefix=PREFIX, control_mode=ControlMode.AUTO
    )
    host_ids = {oid for oid, *_ in host_sensor_specs()}
    found = {e.object_id for e in entities if e.object_id.startswith("host_")}
    # "host_problem" is the board's own health binary sensor (item 103), not a metric
    assert found == {f"host_{oid}" for oid in host_ids} | {"host_problem"}


def test_discovery_has_a_sensor_per_temp_and_fan(cfg: MpcConfig) -> None:
    entities = build_discovery_entities(
        cfg, node_id=NODE_ID, discovery_prefix=PREFIX, control_mode=ControlMode.AUTO
    )
    object_ids = {e.object_id for e in entities}
    for temp in cfg.temps:
        assert f"temp_{temp}" in object_ids
    for ch in cfg.channels:
        assert f"rpm_{ch}" in object_ids
        assert f"pwm_{ch}" in object_ids


def test_discovery_has_one_device_problem_sensor_in_both_modes(
    cfg: MpcConfig, das_example_cfg: MpcConfig
) -> None:
    """Item 83: one Home Assistant entity for the whole daemon's device health, with
    the detail as its attributes from the same retained state topic."""
    for config in (cfg, das_example_cfg):
        entities = build_discovery_entities(
            config, node_id=NODE_ID, discovery_prefix=PREFIX, control_mode=ControlMode.AUTO
        )
        found = [e for e in entities if e.object_id == "device_problem"]
        assert len(found) == 1
        entity = found[0]
        assert entity.component == "binary_sensor"
        assert entity.config_topic == f"{PREFIX}/binary_sensor/{NODE_ID}/device_problem/config"
        assert entity.payload["device_class"] == "problem"
        assert entity.payload["entity_category"] == "diagnostic"
        assert "value_json.health.device_health.ok" in entity.payload["value_template"]
        assert entity.payload["json_attributes_topic"] == f"{NODE_ID}/state"
        assert "value_json.device_health" in entity.payload["json_attributes_template"]


def test_the_device_problem_template_reads_the_published_state_blob(cfg: MpcConfig) -> None:
    """The two keys the template and the attributes name must exist in the blob the
    daemon actually publishes (ControlSnapshot.to_dict())."""
    from aqua_bridge.control.intents import ControlSnapshot, Preset, SolverStatus

    snapshot = ControlSnapshot(
        obs=None,
        last_cmd=None,
        control_mode=ControlMode.AUTO,
        setpoints={},
        overrides={},
        preset=Preset.NORMAL,
        channels=cfg.channels,
        temps=cfg.temps,
        pwm_min=cfg.pwm_min,
        pwm_max=cfg.pwm_max,
        solver_status=SolverStatus.FAULT,
        fault_reason=None,
        fault_since_ts=None,
        usb_present=False,
        mqtt_connected=None,
        uptime_s=0.0,
        device_health={"devices": [], "fans": {}, "problems": ["x"], "ok": False},
    )
    blob = state_payload(snapshot.to_dict(), {})
    assert blob["health"]["device_health"] == {"ok": False, "problems": ["x"]}
    assert blob["device_health"]["problems"] == ["x"]


def test_discovery_has_one_host_problem_sensor_in_both_modes(
    cfg: MpcConfig, das_example_cfg: MpcConfig
) -> None:
    """Item 103: the board gets its own problem entity, so a hot or throttling Pi is not
    read as a controller fault; its attributes are the host half of the same blob."""
    for config in (cfg, das_example_cfg):
        entities = build_discovery_entities(
            config, node_id=NODE_ID, discovery_prefix=PREFIX, control_mode=ControlMode.AUTO
        )
        found = [e for e in entities if e.object_id == "host_problem"]
        assert len(found) == 1
        entity = found[0]
        assert entity.component == "binary_sensor"
        assert entity.config_topic == f"{PREFIX}/binary_sensor/{NODE_ID}/host_problem/config"
        assert entity.payload["device_class"] == "problem"
        assert entity.payload["entity_category"] == "diagnostic"
        assert "value_json.device_health.host.ok" in entity.payload["value_template"]
        assert entity.payload["json_attributes_topic"] == f"{NODE_ID}/state"
        assert "value_json.device_health.host" in entity.payload["json_attributes_template"]


def test_the_host_problem_template_and_the_host_key_read_the_published_state_blob(
    cfg: MpcConfig,
) -> None:
    """Every key item 103's entity names must exist in the blob the daemon publishes: the
    board's verdict under ``device_health.host``, its metrics under ``host``."""
    from aqua_bridge.control.intents import ControlSnapshot, Preset, SolverStatus

    board = {
        "cpu_temp_c": 82.0,
        "air_c": 27.0,
        "divergence_c": 55.0,
        "load1": 0.1,
        "idle": True,
        "throttled": {"hex": "0x4", "now": True, "throttled_now": True},
        "problems": ["host: the board is throttling now (throttled, get_throttled 0x4)"],
        "ok": False,
    }
    snapshot = ControlSnapshot(
        obs=None,
        last_cmd=None,
        control_mode=ControlMode.AUTO,
        setpoints={},
        overrides={},
        preset=Preset.NORMAL,
        channels=cfg.channels,
        temps=cfg.temps,
        pwm_min=cfg.pwm_min,
        pwm_max=cfg.pwm_max,
        solver_status=SolverStatus.FAULT,
        fault_reason=None,
        fault_since_ts=None,
        usb_present=False,
        mqtt_connected=None,
        uptime_s=0.0,
        device_health={
            "devices": [],
            "fans": {},
            "host": board,
            "problems": list(board["problems"]),
            "ok": False,
        },
    )
    blob = state_payload(snapshot.to_dict(), {"cpu_temp_c": 82.0, "throttled": {"hex": "0x4"}})
    assert blob["device_health"]["host"]["ok"] is False
    assert blob["device_health"]["host"]["throttled"]["hex"] == "0x4"
    # the board's problems are in the one list the daemon-wide sensor reads too
    assert blob["health"]["device_health"] == {"ok": False, "problems": list(board["problems"])}
    assert blob["host"]["throttled"]["hex"] == "0x4"


def test_discovery_has_one_aquabus_problem_sensor_in_both_modes(
    cfg: MpcConfig, das_example_cfg: MpcConfig
) -> None:
    """Item 129: the aquabus gets its own problem entity, keyed on ``lost`` (not on
    the mere absence of a device), with the per-controller detail as its
    attributes."""
    for config in (cfg, das_example_cfg):
        entities = build_discovery_entities(
            config, node_id=NODE_ID, discovery_prefix=PREFIX, control_mode=ControlMode.AUTO
        )
        found = [e for e in entities if e.object_id == "aquabus_problem"]
        assert len(found) == 1
        entity = found[0]
        assert entity.component == "binary_sensor"
        assert entity.config_topic == f"{PREFIX}/binary_sensor/{NODE_ID}/aquabus_problem/config"
        assert entity.payload["device_class"] == "problem"
        assert entity.payload["entity_category"] == "diagnostic"
        assert "value_json.device_health.devices" in entity.payload["value_template"]
        assert "aquabus.lost" in entity.payload["value_template"]
        assert entity.payload["json_attributes_topic"] == f"{NODE_ID}/state"
        assert "aquabus" in entity.payload["json_attributes_template"]


def test_the_aquabus_problem_template_reads_the_published_state_blob(cfg: MpcConfig) -> None:
    """The device list the template reads exists in ``ControlSnapshot.to_dict()``,
    each device carrying the ``aquabus`` block :meth:`AquacomputerAdapter.bus_device`
    publishes (``state``, ``present``, ``seen``, ``absent_s``, ``lost``,
    ``temps_missing``)."""
    from aqua_bridge.control.intents import ControlSnapshot, Preset, SolverStatus

    devices = [
        {
            "label": "aquaero",
            "aquabus": {
                "state": "lost",
                "present": False,
                "seen": True,
                "absent_s": 42.0,
                "lost": True,
                "temps_missing": [],
            },
        }
    ]
    snapshot = ControlSnapshot(
        obs=None,
        last_cmd=None,
        control_mode=ControlMode.AUTO,
        setpoints={},
        overrides={},
        preset=Preset.NORMAL,
        channels=cfg.channels,
        temps=cfg.temps,
        pwm_min=cfg.pwm_min,
        pwm_max=cfg.pwm_max,
        solver_status=SolverStatus.FAULT,
        fault_reason=None,
        fault_since_ts=None,
        usb_present=False,
        mqtt_connected=None,
        uptime_s=0.0,
        device_health={"devices": devices, "fans": {}, "problems": [], "ok": True},
    )
    blob = state_payload(snapshot.to_dict(), {})
    assert blob["device_health"]["devices"][0]["aquabus"]["lost"] is True
    assert blob["device_health"]["devices"][0]["aquabus"]["state"] == "lost"


def test_discovery_has_a_model_block_sensor_per_zone_and_an_unexcitable_sensor(
    das_example_cfg: MpcConfig,
) -> None:
    """Item 121: ``pe_diag``/``blocked`` (items 110, 111) and ``excitation`` reach
    Home Assistant as *value* sensors, not as a ``device_class: problem`` binary
    sensor -- a zone still learning is not a fault."""
    entities = {
        e.object_id: e
        for e in build_discovery_entities(
            das_example_cfg, node_id=NODE_ID, discovery_prefix=PREFIX, control_mode=ControlMode.AUTO
        )
    }
    for zone in das_example_cfg.topology.zones:
        oid = f"model_block_{zone}"
        assert oid in entities
        entity = entities[oid]
        assert entity.component == "sensor"
        assert entity.payload.get("device_class") is None
        assert f"thermal.zones.{zone}.blocked" in entity.payload["value_template"]
        assert f"thermal.zones.{zone}" in entity.payload["json_attributes_template"]
        assert entity.payload["entity_category"] == "diagnostic"

    assert "unexcitable_channels" in entities
    unexcitable = entities["unexcitable_channels"]
    assert unexcitable.component == "sensor"
    assert unexcitable.payload.get("device_class") is None
    assert "extra.experiment.unexcitable" in unexcitable.payload["value_template"]
    assert "extra.experiment.excitation" in unexcitable.payload["json_attributes_template"]


def test_model_block_and_excitation_templates_read_the_published_state_blob(
    das_example_cfg: MpcConfig,
) -> None:
    """The paths the two new sensors read exist in the blob the daemon actually
    publishes: ``cmd.diagnostics.thermal.zones.<zone>`` (:func:`aqua_bridge.control.
    thermal.summary`) and ``extra.experiment`` (:func:`aqua_bridge.control.ident.
    status`)."""
    from aqua_bridge.control.intents import ControlSnapshot, Preset, SolverStatus
    from aqua_bridge.model import Mode, MpcCommand

    zone = next(iter(das_example_cfg.topology.zones))
    thermal_zone = {
        "status": "learning",
        "pred_err_c": 0.4,
        "pe_diag": {"g0": 0.02},
        "blocked": [f"pe:{zone}", "pred_err"],
    }
    cmd = MpcCommand(
        pwm=dict.fromkeys(das_example_cfg.channels, 0.5),
        mode=Mode.AUTO,
        diagnostics={"thermal": {"zones": {zone: thermal_zone}}},
    )
    snapshot = ControlSnapshot(
        obs=None,
        last_cmd=cmd,
        control_mode=ControlMode.AUTO,
        setpoints={},
        overrides={},
        preset=Preset.NORMAL,
        channels=das_example_cfg.channels,
        temps=das_example_cfg.temps,
        pwm_min=das_example_cfg.pwm_min,
        pwm_max=das_example_cfg.pwm_max,
        solver_status=SolverStatus.OK,
        fault_reason=None,
        fault_since_ts=None,
        usb_present=True,
        mqtt_connected=None,
        uptime_s=0.0,
        extra={
            "experiment": {
                "running": False,
                "unexcitable": ["xt1"],
                "excitation": {"xt1": {"excitable": False}},
            }
        },
    )
    blob = state_payload(snapshot.to_dict(), {})
    zone_blob = blob["cmd"]["diagnostics"]["thermal"]["zones"][zone]
    assert zone_blob["blocked"] == [f"pe:{zone}", "pred_err"]
    assert blob["extra"]["experiment"]["unexcitable"] == ["xt1"]


def test_discovery_has_a_setpoint_number_per_setpoint(cfg: MpcConfig) -> None:
    entities = build_discovery_entities(
        cfg, node_id=NODE_ID, discovery_prefix=PREFIX, control_mode=ControlMode.AUTO
    )
    numbers = {e.object_id: e for e in entities if e.component == "number"}
    for temp in cfg.setpoints:
        oid = f"setpoint_{temp}"
        assert oid in numbers
        assert numbers[oid].payload["command_topic"] == f"{NODE_ID}/cmd/setpoint/{temp}"


def test_pwm_number_entities_only_in_manual(cfg: MpcConfig) -> None:
    auto_entities = build_discovery_entities(
        cfg, node_id=NODE_ID, discovery_prefix=PREFIX, control_mode=ControlMode.AUTO
    )
    manual_entities = build_discovery_entities(
        cfg, node_id=NODE_ID, discovery_prefix=PREFIX, control_mode=ControlMode.MANUAL
    )
    mixed_entities = build_discovery_entities(
        cfg, node_id=NODE_ID, discovery_prefix=PREFIX, control_mode=ControlMode.MIXED
    )

    auto_ids = {e.object_id for e in auto_entities}
    manual_ids = {e.object_id for e in manual_entities}
    mixed_ids = {e.object_id for e in mixed_entities}

    pwm_cmd_ids = {f"pwm_cmd_{ch}" for ch in cfg.channels}
    assert not (pwm_cmd_ids & auto_ids)
    assert not (pwm_cmd_ids & mixed_ids)
    assert pwm_cmd_ids <= manual_ids

    for ch in cfg.channels:
        entity = next(e for e in manual_entities if e.object_id == f"pwm_cmd_{ch}")
        assert entity.component == "number"
        assert entity.payload["command_topic"] == f"{NODE_ID}/cmd/pwm/{ch}"


def test_discovery_config_topics_use_discovery_prefix(cfg: MpcConfig) -> None:
    entities = build_discovery_entities(
        cfg, node_id=NODE_ID, discovery_prefix="custom_prefix", control_mode=ControlMode.AUTO
    )
    for e in entities:
        assert e.config_topic.startswith(f"custom_prefix/{e.component}/{NODE_ID}/")
        assert e.config_topic.endswith("/config")


def test_discovery_entities_carry_availability_and_device(cfg: MpcConfig) -> None:
    entities = build_discovery_entities(
        cfg, node_id=NODE_ID, discovery_prefix=PREFIX, control_mode=ControlMode.MANUAL
    )
    for e in entities:
        assert e.payload["availability_topic"] == availability_topic(NODE_ID)
        assert e.payload["payload_available"] == "online"
        assert e.payload["payload_not_available"] == "offline"
        assert e.payload["device"]["identifiers"] == [NODE_ID]


def test_discovery_unique_ids_are_unique(cfg: MpcConfig) -> None:
    entities = build_discovery_entities(
        cfg, node_id=NODE_ID, discovery_prefix=PREFIX, control_mode=ControlMode.MANUAL
    )
    unique_ids = [e.payload["unique_id"] for e in entities]
    assert len(unique_ids) == len(set(unique_ids))


# --- state payload -----------------------------------------------------


def test_state_payload_merges_host_under_host_key() -> None:
    snapshot_dict = {"mode": "auto", "obs": None}
    host = {"cpu_temp_c": 42.0}
    merged = state_payload(snapshot_dict, host)
    assert merged["mode"] == "auto"
    assert merged["host"] == {"cpu_temp_c": 42.0}
    # Inputs are not mutated.
    assert "host" not in snapshot_dict


# --- inbound command parsing ---------------------------------------------


def test_parse_mode_command(cfg: MpcConfig) -> None:
    intent = parse_command(NODE_ID, f"{NODE_ID}/cmd/mode", b"manual")
    assert intent == SetMode(mode="manual")


def test_parse_preset_command(cfg: MpcConfig) -> None:
    intent = parse_command(NODE_ID, f"{NODE_ID}/cmd/preset", "cool")
    assert intent == SetPreset(name="cool")


def test_parse_auto_command_all_channels() -> None:
    intent = parse_command(NODE_ID, f"{NODE_ID}/cmd/auto", b"")
    assert intent == ClearOverride(channel=None)


def test_parse_auto_command_one_channel() -> None:
    intent = parse_command(NODE_ID, f"{NODE_ID}/cmd/auto", b"radiator")
    assert intent == ClearOverride(channel="radiator")


def test_parse_setpoint_command() -> None:
    intent = parse_command(NODE_ID, f"{NODE_ID}/cmd/setpoint/coolant", b"33.5")
    assert intent == SetSetpoint(channel="coolant", celsius=33.5)


def test_parse_pwm_command() -> None:
    intent = parse_command(NODE_ID, f"{NODE_ID}/cmd/pwm/radiator", b"0.6")
    assert intent == SetPwm(channel="radiator", pwm=0.6)


def test_parse_unknown_topic_returns_none() -> None:
    assert parse_command(NODE_ID, "some/other/topic", b"x") is None


def test_parse_command_wrong_node_id_returns_none() -> None:
    assert parse_command(NODE_ID, "other-node/cmd/mode", b"auto") is None


def test_parse_command_never_raises_on_garbage_payloads() -> None:
    garbage = [
        (f"{NODE_ID}/cmd/mode", b"not-a-mode"),
        (f"{NODE_ID}/cmd/preset", b""),
        (f"{NODE_ID}/cmd/setpoint/coolant", b"not-a-number"),
        (f"{NODE_ID}/cmd/setpoint/coolant", b"NaN"),
        (f"{NODE_ID}/cmd/pwm/radiator", b"5.0"),  # out of [0, 1]
        (f"{NODE_ID}/cmd/pwm/radiator", b"\xff\xfe"),  # not valid utf-8
        (f"{NODE_ID}/cmd/unknown_kind", b"whatever"),
        (f"{NODE_ID}/cmd/setpoint/", b"1.0"),
    ]
    for topic, payload in garbage:
        assert parse_command(NODE_ID, topic, payload) is None


def test_parse_setpoint_nan_is_rejected() -> None:
    # float("nan") parses fine in Python but is not finite; SetSetpoint rejects it.
    assert parse_command(NODE_ID, f"{NODE_ID}/cmd/setpoint/coolant", b"nan") is None


def test_parse_command_accepts_str_payload_too() -> None:
    intent = parse_command(NODE_ID, f"{NODE_ID}/cmd/mode", "auto")
    assert intent == SetMode(mode="auto")


# --- MqttClient callbacks (no broker; paho only constructs a client) -------


class _Msg:
    def __init__(self, topic: str, payload: bytes) -> None:
        self.topic = topic
        self.payload = payload


def _client(cfg: MpcConfig, **kw):
    pytest.importorskip("paho.mqtt.client")
    from aqua_bridge.publishers.mqtt_ha import MqttClient

    return MqttClient(node_id=NODE_ID, discovery_prefix=PREFIX, cfg=cfg, host="localhost", **kw)


def test_on_message_swallows_intent_errors_from_the_surface(cfg: MpcConfig) -> None:
    """Review finding F6: paho 2.x has suppress_exceptions=False, so an IntentConflict
    (raw PWM while auto) or IntentInvalid raised out of on_message would kill the network
    thread. The callback must log and drop, like the HTTP 4xx/409 path."""
    sup = Supervisor(cfg)
    client = _client(cfg, on_intent=sup.submit)
    assert client.client.suppress_exceptions is False
    assert sup.control_mode is ControlMode.AUTO
    # raw PWM while auto -> IntentConflict inside submit
    assert (
        client._on_message(client.client, None, _Msg(f"{NODE_ID}/cmd/pwm/radiator", b"0.5")) is None
    )
    assert sup.overrides == {}
    # out-of-range setpoint -> IntentInvalid inside submit
    client._on_message(client.client, None, _Msg(f"{NODE_ID}/cmd/setpoint/coolant", b"500"))
    assert sup.setpoints == dict(cfg.setpoints)
    # a valid command still goes through
    client._on_message(client.client, None, _Msg(f"{NODE_ID}/cmd/mode", b"manual"))
    assert sup.control_mode is ControlMode.MANUAL


@pytest.mark.parametrize("exc", [IntentConflict("x"), IntentInvalid("y"), RuntimeError("bug")])
def test_on_message_never_raises_whatever_the_surface_does(cfg: MpcConfig, exc) -> None:
    def boom(intent):
        raise exc

    client = _client(cfg, on_intent=boom)
    client._on_message(client.client, None, _Msg(f"{NODE_ID}/cmd/mode", b"manual"))


def test_connection_change_callback_tracks_connect_and_disconnect(cfg: MpcConfig) -> None:
    seen: list[bool] = []

    class _Paho:
        def __init__(self) -> None:
            self.subscribed: list[str] = []
            self.published: list[str] = []

        def subscribe(self, topic):
            self.subscribed.append(topic)

        def publish(self, topic, payload=None, qos=0, retain=False):
            self.published.append(topic)

    client = _client(cfg, on_connection_change=seen.append)
    fake = _Paho()
    client.client = fake  # publish_availability goes through self.client
    client._on_connect(fake, None, {}, 0)
    assert client.connected is True and seen == [True]
    assert set(fake.subscribed) == set(command_topics(NODE_ID, cfg).values())
    assert fake.published == [availability_topic(NODE_ID)]
    client._on_disconnect(fake, None, {}, 0)
    assert client.connected is False and seen == [True, False]


def test_publish_discovery_deletes_pwm_numbers_when_leaving_manual(cfg: MpcConfig) -> None:
    published: list[tuple[str, str]] = []

    class _Paho:
        def publish(self, topic, payload=None, qos=0, retain=False):
            assert retain is True
            published.append((topic, payload))

    client = _client(cfg)
    client.client = _Paho()
    client.publish_discovery(control_mode=ControlMode.MANUAL)
    manual_topics = {t for t, p in published if p != ""}
    pwm_topics = {t for t in manual_topics if "/number/" in t and "pwm" in t}
    assert pwm_topics, "manual mode must publish PWM number entities"
    assert not [t for t, p in published if p == ""]  # nothing stale in manual
    published.clear()
    client.publish_discovery(control_mode=ControlMode.AUTO)
    deleted = {t for t, p in published if p == ""}
    assert deleted == pwm_topics
    kept = {t for t, p in published if p != ""}
    assert kept == manual_topics - pwm_topics


# --- topic_matches (pure) ---------------------------------------------------------------


@pytest.mark.parametrize(
    ("filter_", "topic", "expected"),
    [
        ("aqua-bridge/in/smart/+", "aqua-bridge/in/smart/S1", True),
        ("aqua-bridge/in/smart/+", "aqua-bridge/in/smart/S1/extra", False),
        ("aqua-bridge/in/smart/+", "aqua-bridge/in/smart", False),
        ("aqua-bridge/cmd/mode", "aqua-bridge/cmd/mode", True),
        ("aqua-bridge/cmd/mode", "aqua-bridge/cmd/preset", False),
        ("aqua-bridge/#", "aqua-bridge/in/smart/S1", True),
        ("aqua-bridge/#", "aqua-bridge", True),  # MQTT spec: "#" also matches its parent level
        ("aqua-bridge/#", "aqua-bridgex", False),
        ("other-node/in/smart/+", "aqua-bridge/in/smart/S1", False),
    ],
)
def test_topic_matches(filter_: str, topic: str, expected: bool) -> None:
    assert topic_matches(filter_, topic) is expected


# --- add_topic_handler (the SMART inbox's MQTT wiring, milestone smart-agent) -----------


def test_add_topic_handler_is_subscribed_on_connect(cfg: MpcConfig) -> None:
    client = _client(cfg)
    seen: list[tuple[str, bytes]] = []
    client.add_topic_handler("aqua-bridge/in/smart/+", lambda t, p: seen.append((t, p)))

    class _Paho:
        def __init__(self) -> None:
            self.subscribed: list[str] = []

        def subscribe(self, topic):
            self.subscribed.append(topic)

        def publish(self, topic, payload=None, qos=0, retain=False):
            pass

    fake = _Paho()
    client.client = fake
    client._on_connect(fake, None, {}, 0)
    assert "aqua-bridge/in/smart/+" in fake.subscribed
    assert set(command_topics(NODE_ID, cfg).values()) <= set(fake.subscribed)


def test_add_topic_handler_routes_matching_messages_there_not_to_parse_command(
    cfg: MpcConfig,
) -> None:
    client = _client(cfg)
    seen: list[tuple[str, bytes]] = []
    client.add_topic_handler("aqua-bridge/in/smart/+", lambda t, p: seen.append((t, p)))

    client._on_message(client.client, None, _Msg("aqua-bridge/in/smart/S1", b'{"x": 1}'))

    assert seen == [("aqua-bridge/in/smart/S1", b'{"x": 1}')]


def test_cmd_topics_still_go_through_parse_command_when_a_handler_is_registered(
    cfg: MpcConfig,
) -> None:
    sup = Supervisor(cfg)
    client = _client(cfg, on_intent=sup.submit)
    client.add_topic_handler("aqua-bridge/in/smart/+", lambda t, p: None)

    client._on_message(client.client, None, _Msg(f"{NODE_ID}/cmd/mode", b"manual"))
    assert sup.control_mode is ControlMode.MANUAL


def test_add_topic_handler_subscribes_immediately_if_already_connected(cfg: MpcConfig) -> None:
    client = _client(cfg)

    class _Paho:
        def __init__(self) -> None:
            self.subscribed: list[str] = []

        def subscribe(self, topic):
            self.subscribed.append(topic)

    fake = _Paho()
    client.client = fake
    client.connected = True
    client.add_topic_handler("aqua-bridge/in/smart/+", lambda t, p: None)
    assert fake.subscribed == ["aqua-bridge/in/smart/+"]


# --- identification experiments (DAS plan section 7) --------------------


def test_ident_topic_and_entity_in_das_mode_only(cfg: MpcConfig) -> None:
    from aqua_bridge.control.intents import Ident
    from test_ident_experiment import ident_cfg

    das = ident_cfg()
    assert command_topics(NODE_ID, das)["ident"] == "aqua-bridge/cmd/ident"
    assert "ident" not in command_topics(NODE_ID, cfg)
    ents = {
        e.object_id: e
        for e in build_discovery_entities(
            das, node_id=NODE_ID, discovery_prefix=PREFIX, control_mode=ControlMode.AUTO
        )
    }
    running = ents["ident_running"]
    assert running.component == "binary_sensor"
    assert running.config_topic == "homeassistant/binary_sensor/aqua-bridge/ident_running/config"
    assert "extra.experiment.running" in running.payload["value_template"]
    legacy = build_discovery_entities(
        cfg, node_id=NODE_ID, discovery_prefix=PREFIX, control_mode=ControlMode.AUTO
    )
    assert "ident_running" not in {e.object_id for e in legacy}
    topic = "aqua-bridge/cmd/ident"
    assert parse_command(NODE_ID, topic, b"stop") == Ident("stop")
    assert parse_command(NODE_ID, topic, "start:group:front") == Ident("start", group="front")
    assert parse_command(NODE_ID, topic, "start:channel:fa1") == Ident("start", channel="fa1")
    assert parse_command(NODE_ID, topic, " start:fb1 ") == Ident("start", channel="fb1")
    for garbage in (b"", b"start", b"start:", b"go:fa1", b"start:group:", b"\xff", b"stop:fa1"):
        assert parse_command(NODE_ID, topic, garbage) is None


def test_ident_command_reaches_the_supervisor_through_on_message() -> None:
    from test_ident_experiment import Rig, ident_cfg

    rig = Rig(ident_cfg())
    rig.ticks(8)
    client = _client(rig.cfg, on_intent=rig.sup.submit)
    topic = f"{NODE_ID}/cmd/ident"
    client._on_message(client.client, None, _Msg(topic, b"start:group:front"))
    assert rig.sup.experiment is not None
    client._on_message(client.client, None, _Msg(topic, b"start:group:front"))  # 409, dropped
    assert rig.sup.experiment is not None
    client._on_message(client.client, None, _Msg(topic, b"stop"))
    assert rig.sup.experiment is None


def test_a_retained_ident_start_never_starts_an_experiment() -> None:
    # Reviewer finding: a start left retained on the broker (``mosquitto_pub -r``) is
    # redelivered on every (re)connect and would start experiments nobody asked for,
    # e.g. after a broker restart. Only a live start counts; a retained stop is harmless.
    from test_ident_experiment import Rig, ident_cfg

    rig = Rig(ident_cfg())
    rig.ticks(8)
    client = _client(rig.cfg, on_intent=rig.sup.submit)
    topic = f"{NODE_ID}/cmd/ident"
    retained = _Msg(topic, b"start:group:front")
    retained.retain = True  # type: ignore[attr-defined]
    client._on_message(client.client, None, retained)
    assert rig.sup.experiment is None
    client._on_message(client.client, None, _Msg(topic, b"start:group:front"))
    assert rig.sup.experiment is not None
    stop = _Msg(topic, b"stop")
    stop.retain = True  # type: ignore[attr-defined]
    client._on_message(client.client, None, stop)
    assert rig.sup.experiment is None


# --- validate_mqtt_section (item 57) ----------------------------------------------------


def test_validate_mqtt_section_defaults_when_absent() -> None:
    assert validate_mqtt_section(None) == {
        "enabled": False,
        "allow_calibrate": False,  # item 105: MQTT calibration is off unless asked for
        "host": "localhost",
        "username": "",
        "password": "",
        "discovery_prefix": "homeassistant",
        "node_id": "aqua-bridge",
        "port": 1883,
    }
    assert validate_mqtt_section({}) == validate_mqtt_section(None)


def test_validate_mqtt_section_accepts_every_key_explicit() -> None:
    section = {
        "enabled": True,
        "allow_calibrate": True,
        "host": "broker.local",
        "username": "u",
        "password": "p",
        "discovery_prefix": "ha",
        "node_id": "node1",
        "port": 1884,
    }
    assert validate_mqtt_section(section) == section


def test_validate_mqtt_section_null_string_falls_back_to_default() -> None:
    # A bare `username:` line in YAML parses as None; that has always meant "unset",
    # not a type error, unlike a wrong-typed value such as a number or a bool.
    values = validate_mqtt_section({"username": None, "password": None})
    assert values["username"] == "" and values["password"] == ""


@pytest.mark.parametrize(
    ("section", "match"),
    [
        ({"enabled": "true"}, "mqtt.enabled"),  # the item 57 example, verbatim
        ({"enabled": "yes"}, "mqtt.enabled"),
        ({"enabled": 1}, "mqtt.enabled"),
        ({"host": 5}, "mqtt.host"),
        ({"username": 5}, "mqtt.username"),
        ({"password": 5}, "mqtt.password"),
        ({"discovery_prefix": 5}, "mqtt.discovery_prefix"),
        ({"node_id": 5}, "mqtt.node_id"),
        ({"port": "1883"}, "mqtt.port"),
        ({"port": 0}, "mqtt.port"),
        ({"port": 70000}, "mqtt.port"),
        ({"port": True}, "mqtt.port"),
    ],
)
def test_validate_mqtt_section_rejects_bad_types(section: dict, match: str) -> None:
    with pytest.raises(MqttSetupError, match=match):
        validate_mqtt_section(section)


# --- the whole Discovery contract, offline (item 21) ------------------------------------
#
# Item 21 confirms these against the owner's Home Assistant. Everything the live run
# can be told in advance is pinned here, so the live half is a short confirmation:
# the entity table (component, unit, device class, state class), the fixed keys of
# every payload, which entities carry a command topic, that those commands reach the
# supervisor, that a SMART message from the broker reaches the inbox, the qos/retain
# flags of every publish, the availability transitions, and node_id / discovery_prefix
# coming from the config section.


class _FakePaho:
    """Records everything the wrapper does to its paho client (no broker)."""

    def __init__(self) -> None:
        self.subscribed: list[str] = []
        self.published: list[tuple[str, Any, int, bool]] = []
        self.disconnected = 0
        self.loops: list[str] = []

    def subscribe(self, topic: str) -> None:
        self.subscribed.append(topic)

    def publish(self, topic: str, payload: Any = None, qos: int = 0, retain: bool = False) -> None:
        self.published.append((topic, payload, qos, retain))

    def disconnect(self) -> None:
        self.disconnected += 1

    def loop_start(self) -> None:
        self.loops.append("start")

    def loop_stop(self) -> None:
        self.loops.append("stop")

    @property
    def topics(self) -> list[str]:
        return [t for t, *_ in self.published]


def _entities(
    config: MpcConfig, *, mode: ControlMode, node_id: str = NODE_ID, prefix: str = PREFIX
) -> dict[str, MqttEntity]:
    return {
        e.object_id: e
        for e in build_discovery_entities(
            config, node_id=node_id, discovery_prefix=prefix, control_mode=mode
        )
    }


def _expected_entity_table(
    config: MpcConfig,
) -> dict[str, tuple[str, str | None, str | None, str | None]]:
    """``object_id -> (component, unit, device_class, state_class)``: the table item 21
    reads off Home Assistant's device page, written down so the live run only has to
    agree with it."""
    table: dict[str, tuple[str, str | None, str | None, str | None]] = {}
    for object_id, _name, unit, device_class in host_sensor_specs():
        table[f"host_{object_id}"] = ("sensor", unit, device_class, "measurement")
    for temp in config.temps:
        table[f"temp_{temp}"] = ("sensor", "°C", "temperature", "measurement")
    for ch in config.channels:
        table[f"rpm_{ch}"] = ("sensor", "rpm", None, "measurement")
        table[f"pwm_{ch}"] = ("sensor", "%", None, "measurement")
    table["device_problem"] = ("binary_sensor", None, "problem", None)
    table["host_problem"] = ("binary_sensor", None, "problem", None)
    table["aquabus_problem"] = ("binary_sensor", None, "problem", None)
    for temp in config.setpoints:
        table[f"setpoint_{temp}"] = ("number", "°C", None, None)
    for drive_class in config.drive_classes:
        table[f"limit_{drive_class}"] = ("number", "°C", None, None)
    if config.topology is not None:
        table["model_status"] = ("sensor", None, None, None)
        table["model_pred_err_c"] = ("sensor", "°C", None, "measurement")
        table["noise_db"] = ("sensor", "dB", None, "measurement")
        table["ident_running"] = ("binary_sensor", None, "running", None)
        table["unexcitable_channels"] = ("sensor", None, None, None)
        for zone in config.topology.zones:
            table[f"zone_status_{zone}"] = ("sensor", None, None, None)
            table[f"model_block_{zone}"] = ("sensor", None, None, None)
        for bay in config.topology.bays:
            table[f"drive_temp_{bay}"] = ("sensor", "°C", "temperature", "measurement")
            table[f"drive_margin_{bay}"] = ("sensor", "°C", None, "measurement")
            table[f"drive_sigma_{bay}"] = ("sensor", "°C", None, "measurement")
            table[f"bay_occupied_{bay}"] = ("binary_sensor", None, "occupancy", None)
    return table


def test_the_announced_entities_are_exactly_the_table_home_assistant_shows(
    cfg: MpcConfig, das_example_cfg: MpcConfig
) -> None:
    for config in (cfg, das_example_cfg):
        table = _expected_entity_table(config)
        pwm_cmd = {f"pwm_cmd_{ch}": ("number", None, None, None) for ch in config.channels}
        for mode in ControlMode:
            want = table | (pwm_cmd if mode is ControlMode.MANUAL else {})
            got = _entities(config, mode=mode)
            assert set(got) == set(want), mode
            for object_id, (component, unit, device_class, state_class) in want.items():
                payload = got[object_id].payload
                assert got[object_id].component == component, object_id
                assert payload.get("unit_of_measurement") == unit, object_id
                assert payload.get("device_class") == device_class, object_id
                assert payload.get("state_class") == state_class, object_id


def test_every_discovery_payload_carries_topic_unique_id_device_and_availability(
    cfg: MpcConfig, das_example_cfg: MpcConfig
) -> None:
    device = {
        "identifiers": [NODE_ID],
        "name": NODE_ID,
        "manufacturer": "aqua-bridge",
        "model": "aquaero 6 XT + Quadro",
    }
    for config in (cfg, das_example_cfg):
        for mode in ControlMode:
            for object_id, entity in _entities(config, mode=mode).items():
                payload = entity.payload
                assert entity.config_topic == (
                    f"{PREFIX}/{entity.component}/{NODE_ID}/{object_id}/config"
                )
                assert payload["unique_id"] == f"{NODE_ID}_{object_id}" == payload["object_id"]
                assert payload["state_topic"] == state_topic(NODE_ID)
                assert payload["name"]
                assert "value_json." in payload["value_template"]
                assert payload["device"] == device
                assert payload["availability_topic"] == availability_topic(NODE_ID)
                assert payload["payload_available"] == "online"
                assert payload["payload_not_available"] == "offline"
                # It is published as JSON, so it has to survive a round trip unchanged.
                assert json.loads(json.dumps(payload)) == payload


def test_numbers_are_the_only_commandable_entities_and_their_topics_are_subscribed(
    cfg: MpcConfig, das_example_cfg: MpcConfig
) -> None:
    for config in (cfg, das_example_cfg):
        subscribed = set(command_topics(NODE_ID, config).values())
        for mode in ControlMode:
            for object_id, entity in _entities(config, mode=mode).items():
                topic = entity.payload.get("command_topic")
                if entity.component == "number":
                    assert topic in subscribed, object_id
                else:
                    assert topic is None, object_id


def test_number_entities_carry_the_range_and_step_section_7_documents(
    cfg: MpcConfig, das_example_cfg: MpcConfig
) -> None:
    manual = _entities(cfg, mode=ControlMode.MANUAL)
    for temp in cfg.setpoints:
        payload = manual[f"setpoint_{temp}"].payload
        assert (payload["min"], payload["max"], payload["step"]) == (
            cfg.temp_min_c,
            cfg.temp_max_c,
            0.5,
        )
    for ch in cfg.channels:
        payload = manual[f"pwm_cmd_{ch}"].payload
        assert (payload["min"], payload["max"], payload["step"]) == (cfg.pwm_min, cfg.pwm_max, 0.01)
    das = _entities(das_example_cfg, mode=ControlMode.AUTO)
    for name, drive_class in das_example_cfg.drive_classes.items():
        payload = das[f"limit_{name}"].payload
        assert (payload["min"], payload["max"], payload["step"]) == (
            das_example_cfg.temp_min_c,
            drive_class.limit_c,
            0.5,
        )


def test_a_setpoint_number_command_reaches_the_supervisor(cfg: MpcConfig) -> None:
    sup = Supervisor(cfg)
    client = _client(cfg, on_intent=sup.submit)
    temp = next(iter(cfg.setpoints))
    topic = _entities(cfg, mode=ControlMode.AUTO)[f"setpoint_{temp}"].payload["command_topic"]
    client._on_message(client.client, None, _Msg(topic, b"31.5"))
    assert sup.setpoints[temp] == 31.5


def test_a_limit_number_command_reaches_the_supervisor(das_example_cfg: MpcConfig) -> None:
    sup = Supervisor(das_example_cfg)
    client = _client(das_example_cfg, on_intent=sup.submit)
    name, drive_class = next(iter(das_example_cfg.drive_classes.items()))
    entities = _entities(das_example_cfg, mode=ControlMode.AUTO)
    topic = entities[f"limit_{name}"].payload["command_topic"]
    tighter = drive_class.limit_c - 3.0
    client._on_message(client.client, None, _Msg(topic, str(tighter).encode()))
    assert sup.effective_config().drive_classes[name].limit_c == tighter


def test_a_pwm_number_command_acts_only_in_manual(cfg: MpcConfig) -> None:
    sup = Supervisor(cfg)
    client = _client(cfg, on_intent=sup.submit)
    ch = cfg.channels[0]
    topic = _entities(cfg, mode=ControlMode.MANUAL)[f"pwm_cmd_{ch}"].payload["command_topic"]
    client._on_message(client.client, None, _Msg(topic, b"0.42"))
    assert sup.overrides == {}  # auto: refused like HTTP's 409, logged and dropped
    sup.submit(SetMode(mode="manual"))
    client._on_message(client.client, None, _Msg(topic, b"0.42"))
    assert sup.overrides[ch] == 0.42


def test_a_smart_message_from_the_broker_reaches_the_inbox(cfg: MpcConfig) -> None:
    """The SMART half of item 21: the agent's retained message on the broker ends up in
    the inbox the estimator reads, over the daemon's one connection."""
    inbox = SmartInbox()
    client = _client(cfg)
    client.add_topic_handler(smart_topic_filter(NODE_ID), inbox.on_message)
    paho = _FakePaho()
    client.client = paho
    client._on_connect(paho, None, {}, 0)
    assert smart_topic_filter(NODE_ID) in paho.subscribed

    sample = {"serial": "S1", "model": "WDC WD40", "temp_c": 38.5, "ts_wall": 1_700_000_000.0}
    retained = _Msg(f"{NODE_ID}/in/smart/S1", json.dumps(sample).encode())
    retained.retain = True  # type: ignore[attr-defined]
    client._on_message(paho, None, retained)
    snapshot = inbox.snapshot()
    assert snapshot["S1"]["temp_c"] == 38.5
    assert snapshot["S1"]["model"] == "WDC WD40"
    assert inbox.accepted == 1

    client._on_message(paho, None, _Msg(f"{NODE_ID}/in/smart/S2", b"not json"))
    assert "S2" not in inbox.snapshot() and inbox.rejected == 1


def test_every_publish_uses_the_qos_and_retain_flags_of_section_7(cfg: MpcConfig) -> None:
    client = _client(cfg)
    paho = _FakePaho()
    client.client = paho

    client._on_connect(paho, None, {}, 0)
    assert paho.published == [(availability_topic(NODE_ID), "online", 1, True)]

    paho.published.clear()
    client.publish_discovery(control_mode=ControlMode.MANUAL)
    assert paho.published
    for topic, payload, qos, retain in paho.published:
        assert topic.startswith(f"{PREFIX}/") and topic.endswith("/config")
        assert (qos, retain) == (1, True)
        assert json.loads(payload)["unique_id"]

    paho.published.clear()
    client.publish_discovery(control_mode=ControlMode.AUTO)
    deleted = [entry for entry in paho.published if entry[1] == ""]
    assert deleted and all((qos, retain) == (1, True) for _, _, qos, retain in deleted)

    paho.published.clear()
    client.publish_state({"mode": "auto"}, {"cpu_temp_c": 41.0})
    topic, payload, qos, retain = paho.published[0]
    assert (topic, qos, retain) == (state_topic(NODE_ID), 0, True)
    assert json.loads(payload) == {"mode": "auto", "host": {"cpu_temp_c": 41.0}}

    paho.published.clear()
    client.disconnect()
    assert paho.published == [(availability_topic(NODE_ID), "offline", 1, True)]
    assert paho.disconnected == 1


def test_the_last_will_is_a_retained_offline_on_the_availability_topic(
    cfg: MpcConfig, monkeypatch: pytest.MonkeyPatch
) -> None:
    """A daemon killed without a clean stop must still go unavailable in Home Assistant.

    Asserted on the ``will_set`` call the wrapper makes, which is what ``MqttClient``
    promises, rather than on paho's private ``_will*`` attributes: the package allows
    any paho 2.x and the Pi installs the distro build, so library internals renamed in
    a later release would fail here looking like an aqua-bridge regression.
    """
    mqtt = pytest.importorskip("paho.mqtt.client")
    wills: list[tuple[Any, ...]] = []

    class _RecordingPaho(_FakePaho):
        def __init__(self, *_args: Any, **_kwargs: Any) -> None:
            super().__init__()

        def username_pw_set(self, username: str, password: str | None = None) -> None:
            self.auth = (username, password)

        def will_set(self, topic: str, payload: Any = None, qos: int = 0, retain: bool = False):
            wills.append((topic, payload, qos, retain))

    monkeypatch.setattr(mqtt, "Client", _RecordingPaho)
    _client(cfg)
    assert wills == [(availability_topic(NODE_ID), "offline", 1, True)]


def test_the_publisher_lifecycle_and_the_node_id_and_prefix_from_the_config(
    cfg: MpcConfig,
) -> None:
    """online -> Discovery -> state -> offline on one connection, with every topic under
    the configured node_id / discovery_prefix (item 21 runs against a broker where both
    differ from the defaults)."""
    pytest.importorskip("paho.mqtt.client")
    from aqua_bridge.config import AppConfig
    from aqua_bridge.publishers.runtime import MqttService

    node_id, prefix = "attic-bridge", "ha-test"
    sup = Supervisor(cfg)
    app = AppConfig(
        mpc=cfg,
        mqtt={
            "enabled": True,
            "host": "broker.example",
            "node_id": node_id,
            "discovery_prefix": prefix,
        },
        host={"interval_s": 5},
    )
    service = MqttService.from_config(app, sup, hostinfo=lambda: {"cpu_temp_c": 41.0})
    client = service.client
    paho = _FakePaho()
    client.client = paho

    client._on_connect(paho, None, {}, 0)  # what paho's network thread calls
    assert sup.snapshot().mqtt_connected is True
    assert set(paho.subscribed) == set(command_topics(node_id, cfg).values())
    service.on_tick(None)

    assert paho.topics[0] == f"{node_id}/status"
    assert paho.topics[-1] == f"{node_id}/state"
    assert all(t.startswith((f"{node_id}/", f"{prefix}/")) for t in paho.topics)
    assert any(t.startswith(f"{prefix}/sensor/{node_id}/") for t in paho.topics)
    state = json.loads(paho.published[-1][1])
    assert state["mode"] == "auto" and state["host"] == {"cpu_temp_c": 41.0}

    service.stop()
    assert paho.published[-1] == (f"{node_id}/status", "offline", 1, True)
    assert paho.loops[-1] == "stop"


# --- cmd/calibrate: the MQTT counterpart of POST /api/calibrate (item 105) ---------------


def _calibrate_rig():
    from test_ident_experiment import Rig, ident_cfg

    rig = Rig(ident_cfg())
    rig.ticks(8)
    return rig


def _manual_memory(rig) -> dict:
    """The estimator's per-bay manual calibrations after the rig's last tick."""
    return dict(rig.loop.state.solver_memory.get("estimator", {}).get("manual", {}))


def test_calibrate_topics_exist_only_under_the_opt_in(das_example_cfg: MpcConfig) -> None:
    """Item 105: without ``mqtt.allow_calibrate`` there is no topic to subscribe to, so
    the daemon never even asks the broker for the messages."""
    bays = das_example_cfg.topology.bays  # type: ignore[union-attr]
    off = command_topics(NODE_ID, das_example_cfg)
    assert not any(t.startswith("calibrate/") for t in off)
    on = command_topics(NODE_ID, das_example_cfg, allow_calibrate=True)
    assert set(on) - set(off) == {f"calibrate/{bay}" for bay in bays}
    for bay in bays:
        assert on[f"calibrate/{bay}"] == f"{NODE_ID}/cmd/calibrate/{bay}"


def test_calibrate_topics_are_das_only(cfg: MpcConfig) -> None:
    # a legacy config has no bays, so the opt-in adds nothing to subscribe to
    assert command_topics(NODE_ID, cfg, allow_calibrate=True) == command_topics(NODE_ID, cfg)


def test_parse_command_refuses_a_calibration_without_the_opt_in() -> None:
    topic = f"{NODE_ID}/cmd/calibrate/a1"
    assert parse_command(NODE_ID, topic, b"41.0") is None
    assert parse_command(NODE_ID, topic, b"41.0", allow_calibrate=False) is None
    assert parse_command(NODE_ID, topic, b"41.0", allow_calibrate=True) == Calibrate(
        bay="a1", drive_temp_c=41.0
    )
    # garbage is still garbage with the opt-in on
    for bad in (b"", b"warm", b"\xff", b"nan"):
        assert parse_command(NODE_ID, topic, bad, allow_calibrate=True) is None
    empty_bay = f"{NODE_ID}/cmd/calibrate/"
    assert parse_command(NODE_ID, empty_bay, b"41.0", allow_calibrate=True) is None


def test_a_calibration_over_mqtt_reaches_the_estimator_under_the_opt_in() -> None:
    rig = _calibrate_rig()
    client = _client(rig.cfg, on_intent=rig.sup.submit, allow_calibrate=True)
    assert client.allow_calibrate is True
    client._on_message(client.client, None, _Msg(f"{NODE_ID}/cmd/calibrate/a1", b"41.0"))
    stamp = rig.t
    assert rig.sup.plan_tick().calibrations == {"a1": {"temp_c": 41.0, "ts": stamp}}
    result = rig.ticks(1)[0]
    assert result.obs.inputs["calibration"] == {"a1": {"temp_c": 41.0, "ts": stamp}}
    assert result.mpc_cmd.diagnostics["estimator"]["manual_fresh"] == ["a1"]
    assert set(_manual_memory(rig)) == {"a1"}


def test_a_calibration_over_mqtt_is_refused_without_the_opt_in() -> None:
    """The refusal happens before any intent exists, so nothing downstream sees it: no
    supervisor state, no observation input, no RLS row in the estimator."""
    rig = _calibrate_rig()
    submitted: list[Any] = []

    def on_intent(intent: Any) -> None:
        submitted.append(intent)
        rig.sup.submit(intent)

    client = _client(rig.cfg, on_intent=on_intent)
    assert client.allow_calibrate is False
    topic = f"{NODE_ID}/cmd/calibrate/a1"
    assert client._on_message(client.client, None, _Msg(topic, b"41.0")) is None
    assert submitted == []
    assert rig.sup.plan_tick().calibrations == {}
    result = rig.ticks(1)[0]
    assert not result.obs.inputs.get("calibration")
    diagnostics = result.mpc_cmd.diagnostics["estimator"]
    assert diagnostics["manual_fresh"] == []
    assert diagnostics["manual_used"] == 0 and diagnostics["manual_rejected"] == 0
    assert _manual_memory(rig) == {}
    # an ordinary DAS command on the same connection still goes through
    client._on_message(client.client, None, _Msg(f"{NODE_ID}/cmd/bay/a1", b"empty"))
    assert submitted == [SetBay(bay="a1", changes={"occupied": False})]


def test_a_retained_calibration_is_ignored() -> None:
    """A retained number would be refitted into the bay's map on every reconnect and
    every restart -- the failure item 104 is about, arriving over the wire."""
    rig = _calibrate_rig()
    client = _client(rig.cfg, on_intent=rig.sup.submit, allow_calibrate=True)
    topic = f"{NODE_ID}/cmd/calibrate/a1"
    retained = _Msg(topic, b"41.0")
    retained.retain = True  # type: ignore[attr-defined]
    client._on_message(client.client, None, retained)
    assert rig.sup.plan_tick().calibrations == {}
    client._on_message(client.client, None, _Msg(topic, b"41.0"))
    assert set(rig.sup.plan_tick().calibrations) == {"a1"}


def test_the_opt_in_needs_an_authenticated_broker_connection() -> None:
    """Item 105: MQTT has no authentication of its own, only the broker's ACL, and an
    anonymous connection cannot be given one. Refused by name at startup."""
    with pytest.raises(MqttSetupError, match="mqtt.username"):
        validate_mqtt_section({"enabled": True, "allow_calibrate": True})
    with pytest.raises(MqttSetupError, match="mqtt.username"):
        validate_mqtt_section({"enabled": True, "allow_calibrate": True, "username": ""})
    ok = validate_mqtt_section({"enabled": True, "allow_calibrate": True, "username": "u"})
    assert ok["allow_calibrate"] is True
    with pytest.raises(MqttSetupError, match="mqtt.allow_calibrate"):
        validate_mqtt_section({"allow_calibrate": "true"})


def test_no_discovery_entity_is_published_for_calibration(das_example_cfg: MpcConfig) -> None:
    """Fifteen more ``number`` entities, each of whose states Home Assistant republishes
    on restart, is exactly the retained-message problem in another costume (item 105)."""
    entities = build_discovery_entities(
        das_example_cfg, node_id=NODE_ID, discovery_prefix=PREFIX, control_mode=ControlMode.AUTO
    )
    assert not any("calibrat" in e.object_id for e in entities)
