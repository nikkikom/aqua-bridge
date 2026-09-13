"""Tests for aqua_bridge.publishers.mqtt_ha (PROJECT.md section 7)."""

from __future__ import annotations

import pytest

from aqua_bridge.control.intents import (
    ClearOverride,
    ControlMode,
    IntentConflict,
    IntentInvalid,
    SetMode,
    SetPreset,
    SetPwm,
    SetSetpoint,
)
from aqua_bridge.control.supervisor import Supervisor
from aqua_bridge.model import MpcConfig
from aqua_bridge.publishers.mqtt_ha import (
    availability_topic,
    build_discovery_entities,
    command_topics,
    host_sensor_specs,
    parse_command,
    state_payload,
    state_topic,
    topic_matches,
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
    assert found == {f"host_{oid}" for oid in host_ids}


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
