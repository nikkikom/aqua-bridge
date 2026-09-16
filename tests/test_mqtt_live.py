"""Opt-in integration test of the whole publisher against a **real** MQTT broker
(PROJECT.md section 7, section 8 item 21). Marker: ``mqtt_live``.

Skipped unless ``$AQUA_BRIDGE_MQTT_TEST_HOST`` names a broker, so CI stays offline
and the rest of the suite never touches a network:

    AQUA_BRIDGE_MQTT_TEST_HOST=broker.lan \\
    AQUA_BRIDGE_MQTT_TEST_USERNAME=aqua-bridge \\
    AQUA_BRIDGE_MQTT_TEST_PASSWORD=... \\
        .venv/bin/python -m pytest -m mqtt_live -q

``AQUA_BRIDGE_MQTT_TEST_PORT`` (1883) completes the set. The test publishes under
its own ``node_id`` (``aqua-bridge-test-<pid>``), never the daemon's, so it can run
against the owner's Home Assistant broker while the daemon is live without touching
its topics or its entities; every retained message it leaves is deleted at the end.

What it proves that the offline suite cannot: that a real broker accepts the LWT,
the retained Discovery configs and the retained state blob; that a command published
by somebody else (Home Assistant, ``mosquitto_pub``) arrives and reaches the
supervisor; that ``in/smart`` reaches the inbox on the same connection; and that
``tools/ha_check.py``'s checklist comes back clean against what the daemon actually
left on the broker.
"""

from __future__ import annotations

import importlib.util
import json
import os
import sys
import time
from pathlib import Path
from typing import Any

import pytest

from aqua_bridge.config import AppConfig
from aqua_bridge.control.intents import ControlMode
from aqua_bridge.control.supervisor import Supervisor
from aqua_bridge.model import MpcConfig
from aqua_bridge.publishers.inputs import SmartInbox, smart_topic_filter
from aqua_bridge.publishers.mqtt_ha import (
    availability_topic,
    build_discovery_entities,
    state_topic,
)

pytestmark = pytest.mark.mqtt_live

#: Naming a broker here opts the whole module in; without it every test skips.
HOST = os.environ.get("AQUA_BRIDGE_MQTT_TEST_HOST", "")
PORT = int(os.environ.get("AQUA_BRIDGE_MQTT_TEST_PORT", "1883"))
USERNAME = os.environ.get("AQUA_BRIDGE_MQTT_TEST_USERNAME", "")
PASSWORD = os.environ.get("AQUA_BRIDGE_MQTT_TEST_PASSWORD", "")

#: How long any single step (connect, a message's round trip) may take, seconds.
STEP_TIMEOUT_S = 10.0
#: How long the checker listens for the retained tree, seconds.
COLLECT_S = 3.0

PREFIX = "homeassistant-aqua-bridge-test"

_TOOLS = Path(__file__).resolve().parent.parent / "tools"


def _load_ha_check() -> Any:
    spec = importlib.util.spec_from_file_location("ha_check", _TOOLS / "ha_check.py")
    assert spec is not None and spec.loader is not None
    module = sys.modules.get("ha_check")
    if module is None:
        module = importlib.util.module_from_spec(spec)
        sys.modules["ha_check"] = module
        spec.loader.exec_module(module)
    return module


def _wait_until(predicate, *, timeout_s: float = STEP_TIMEOUT_S) -> bool:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.05)
    return predicate()


def _plain_client(client_id: str) -> Any:
    """A bare paho client for the other side of the broker (what Home Assistant is)."""
    import paho.mqtt.client as mqtt

    client = mqtt.Client(callback_api_version=mqtt.CallbackAPIVersion.VERSION2, client_id=client_id)
    if USERNAME:
        client.username_pw_set(USERNAME, PASSWORD or None)
    client.connect(HOST, PORT, keepalive=30)
    client.loop_start()
    return client


@pytest.fixture
def live(cfg: MpcConfig):
    """The daemon's MQTT publisher, connected to the real broker under a test node id."""
    if not HOST:
        pytest.skip("set AQUA_BRIDGE_MQTT_TEST_HOST to run the live MQTT test")
    pytest.importorskip("paho.mqtt.client")
    from aqua_bridge.publishers.runtime import MqttService

    node_id = f"aqua-bridge-test-{os.getpid()}"
    sup = Supervisor(cfg)
    inbox = SmartInbox()
    app = AppConfig(
        mpc=cfg,
        mqtt={
            "enabled": True,
            "host": HOST,
            "port": PORT,
            "username": USERNAME,
            "password": PASSWORD,
            "node_id": node_id,
            "discovery_prefix": PREFIX,
        },
        host={"interval_s": 5},
    )
    service = MqttService.from_config(app, sup, hostinfo=lambda: {"cpu_temp_c": 41.0})
    service.client.add_topic_handler(smart_topic_filter(node_id), inbox.on_message)
    service.start()
    assert _wait_until(lambda: service.client.connected), f"no connection to {HOST}:{PORT}"
    try:
        yield service, sup, inbox, node_id, app
    finally:
        try:
            service.stop()
        finally:
            _clean_retained(cfg, node_id)


def _clean_retained(cfg: MpcConfig, node_id: str) -> None:
    """Delete every retained message this test left, so the broker is as it was."""
    topics = {availability_topic(node_id), state_topic(node_id), f"{node_id}/in/smart/TESTSERIAL"}
    for mode in ControlMode:
        topics.update(
            entity.config_topic
            for entity in build_discovery_entities(
                cfg, node_id=node_id, discovery_prefix=PREFIX, control_mode=mode
            )
        )
    client = _plain_client(f"aqua-bridge-test-clean-{os.getpid()}")
    try:
        for topic in sorted(topics):
            client.publish(topic, payload="", qos=1, retain=True).wait_for_publish(
                timeout=STEP_TIMEOUT_S
            )
    finally:
        client.loop_stop()
        client.disconnect()


def test_the_publisher_and_the_check_tool_against_a_real_broker(live, cfg: MpcConfig) -> None:
    service, sup, inbox, node_id, app = live
    ha = _load_ha_check()
    service.on_tick(None)  # availability is already out; this adds Discovery and the state

    # 1. a command from the other side of the broker reaches the supervisor
    other = _plain_client(f"aqua-bridge-test-ha-{os.getpid()}")
    try:
        temp = next(iter(cfg.setpoints))
        other.publish(f"{node_id}/cmd/setpoint/{temp}", payload="31.5", qos=1).wait_for_publish(
            timeout=STEP_TIMEOUT_S
        )
        assert _wait_until(lambda: sup.setpoints[temp] == 31.5), sup.setpoints

        # 2. a SMART reading reaches the inbox on the same connection
        sample = {"serial": "TESTSERIAL", "model": "test", "temp_c": 38.5, "ts_wall": time.time()}
        other.publish(
            f"{node_id}/in/smart/TESTSERIAL", payload=json.dumps(sample), qos=1, retain=True
        ).wait_for_publish(timeout=STEP_TIMEOUT_S)
        assert _wait_until(lambda: "TESTSERIAL" in inbox.snapshot()), inbox.rejected
    finally:
        other.loop_stop()
        other.disconnect()

    service.on_tick(None)  # the state blob with the new setpoint

    # 3. what the broker now holds is what tools/ha_check.py expects
    messages = ha.collect_messages(
        host=HOST,
        port=PORT,
        username=USERNAME,
        password=PASSWORD,
        filters=[ha.discovery_filter(PREFIX, node_id), f"{node_id}/#"],
        wait_s=COLLECT_S,
        client_id=f"aqua-bridge-test-check-{os.getpid()}",
    )
    report = ha.build_report(app, messages, node_id=node_id, discovery_prefix=PREFIX)
    assert report.problems == [], report.text()
    seen = ha.latest_by_topic(messages)
    assert seen[availability_topic(node_id)].text == "online"
    assert seen[availability_topic(node_id)].retain is True
    state = seen[state_topic(node_id)].json()[1]
    assert state["setpoints"][next(iter(cfg.setpoints))] == 31.5
    assert state["host"] == {"cpu_temp_c": 41.0}

    # 4. a clean stop leaves the availability topic retained-offline
    service.stop()
    after = ha.collect_messages(
        host=HOST,
        port=PORT,
        username=USERNAME,
        password=PASSWORD,
        filters=[availability_topic(node_id)],
        wait_s=COLLECT_S,
        client_id=f"aqua-bridge-test-after-{os.getpid()}",
    )
    offline = ha.latest_by_topic(after)[availability_topic(node_id)]
    assert offline.text == "offline" and offline.retain is True
