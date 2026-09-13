"""MQTT Discovery + Home Assistant bridge (PROJECT.md section 7).

Pure, broker-independent functions build the Discovery config payloads and
the state/command topics; :class:`MqttClient` is a thin ``paho-mqtt`` 2.x
wrapper around them, not exercised against a live broker (see
``tests/test_mqtt_ha.py``).

Topic layout (``node_id`` from ``config.yaml`` ``mqtt.node_id``):

* ``{node_id}/status``            -- LWT, ``online``/``offline``
* ``{node_id}/state``             -- retained JSON, one big blob every tick
  (``ControlSnapshot.to_dict()`` plus a ``host`` key for host metrics)
* ``{node_id}/cmd/mode``          -- raw string, one of :class:`ControlMode`
* ``{node_id}/cmd/preset``        -- raw string, one of :class:`Preset`
* ``{node_id}/cmd/auto``          -- raw string channel name, or empty = all
* ``{node_id}/cmd/setpoint/<temp>``   -- raw number, target Celsius
* ``{node_id}/cmd/pwm/<channel>``     -- raw number, manual PWM 0..1

Discovery config topics follow the standard
``{discovery_prefix}/{component}/{node_id}/{object_id}/config``.

HA sends a setpoint while Auto, never raw PWM (section 7); PWM number
entities are only published to Discovery while the global mode is
``manual`` -- :func:`build_discovery_entities` takes the current
``control_mode`` and the caller (the loop) is responsible for re-publishing
Discovery (and, for HA, deleting the stale ones with an empty retained
payload) whenever the mode changes into or out of ``manual``.

A caller that needs one more inbound topic on this same connection (the DAS
plan's SMART inbox, ``{node_id}/in/smart/+``, section 1) registers it with
:meth:`MqttClient.add_topic_handler` instead of opening a second broker
connection -- see that method's docstring.
"""

from __future__ import annotations

import json
import logging
from collections.abc import Callable, Mapping
from dataclasses import dataclass
from typing import Any

from aqua_bridge.control.intents import (
    ClearOverride,
    ControlMode,
    Intent,
    IntentError,
    SetMode,
    SetPreset,
    SetPwm,
    SetSetpoint,
)
from aqua_bridge.model import MpcConfig

__all__ = [
    "MqttClient",
    "MqttEntity",
    "availability_topic",
    "build_discovery_entities",
    "command_topics",
    "host_sensor_specs",
    "parse_command",
    "state_payload",
    "state_topic",
    "topic_matches",
]

_LOG = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# Topics
# ---------------------------------------------------------------------------


def availability_topic(node_id: str) -> str:
    return f"{node_id}/status"


def state_topic(node_id: str) -> str:
    return f"{node_id}/state"


def command_topics(node_id: str, cfg: MpcConfig) -> dict[str, str]:
    """Every inbound command topic this bridge subscribes to."""
    topics = {
        "mode": f"{node_id}/cmd/mode",
        "preset": f"{node_id}/cmd/preset",
        "auto": f"{node_id}/cmd/auto",
    }
    for temp in cfg.setpoints:
        topics[f"setpoint/{temp}"] = f"{node_id}/cmd/setpoint/{temp}"
    for ch in cfg.channels:
        topics[f"pwm/{ch}"] = f"{node_id}/cmd/pwm/{ch}"
    return topics


def topic_matches(topic_filter: str, topic: str) -> bool:
    """MQTT topic-filter matching (``+`` one level, trailing ``#`` many),
    enough of the spec for :meth:`MqttClient.add_topic_handler` to route an
    inbound message to the right handler without a broker in the loop
    (pure, unit-tested directly rather than only through a fake client).
    """
    filter_parts = topic_filter.split("/")
    topic_parts = topic.split("/")
    for i, part in enumerate(filter_parts):
        if part == "#":
            return True  # matches this level and everything after
        if i >= len(topic_parts):
            return False
        if part != "+" and part != topic_parts[i]:
            return False
    return len(filter_parts) == len(topic_parts)


def _device_block(node_id: str) -> dict[str, Any]:
    return {
        "identifiers": [node_id],
        "name": node_id,
        "manufacturer": "aqua-bridge",
        "model": "aquaero 6 XT + Quadro",
    }


# ---------------------------------------------------------------------------
# Discovery entities
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class MqttEntity:
    """One MQTT Discovery entity: its config topic and payload."""

    component: str  # "sensor" | "number" | "binary_sensor"
    object_id: str  # unique within node_id
    config_topic: str
    payload: dict[str, Any]


def _availability(node_id: str) -> dict[str, Any]:
    return {
        "availability_topic": availability_topic(node_id),
        "payload_available": "online",
        "payload_not_available": "offline",
    }


def _sensor(
    *,
    discovery_prefix: str,
    node_id: str,
    object_id: str,
    name: str,
    value_template: str,
    unit: str | None = None,
    device_class: str | None = None,
    state_class: str | None = "measurement",
) -> MqttEntity:
    unique_id = f"{node_id}_{object_id}"
    payload: dict[str, Any] = {
        "name": name,
        "unique_id": unique_id,
        "object_id": unique_id,
        "state_topic": state_topic(node_id),
        "value_template": value_template,
        "device": _device_block(node_id),
        **_availability(node_id),
    }
    if unit is not None:
        payload["unit_of_measurement"] = unit
    if device_class is not None:
        payload["device_class"] = device_class
    if state_class is not None:
        payload["state_class"] = state_class
    topic = f"{discovery_prefix}/sensor/{node_id}/{object_id}/config"
    return MqttEntity("sensor", object_id, topic, payload)


def host_sensor_specs() -> tuple[tuple[str, str, str | None, str | None], ...]:
    """``(object_id, name, unit, device_class)`` for every host sensor.

    ``value_template`` is derived from ``object_id`` (``host.<object_id>``
    in the state payload), so this is also the single source of truth for
    :func:`build_discovery_entities` and :func:`state_payload`.
    """
    return (
        ("cpu_temp_c", "CPU temperature", "°C", "temperature"),
        ("load1", "Load average (1m)", None, None),
        ("mem_used_pct", "RAM used", "%", None),
        ("disk_used_pct", "Disk used", "%", None),
        ("wifi_rssi_dbm", "Wi-Fi signal", "dBm", "signal_strength"),
        ("uptime_s", "Uptime", "s", "duration"),
    )


def build_discovery_entities(
    cfg: MpcConfig,
    *,
    node_id: str,
    discovery_prefix: str,
    control_mode: ControlMode,
) -> list[MqttEntity]:
    """Every Discovery entity for the current ``control_mode``.

    PWM number entities are included only when ``control_mode`` is
    :attr:`ControlMode.MANUAL` (section 7); everything else is always
    present.
    """
    entities: list[MqttEntity] = []

    for object_id, name, unit, device_class in host_sensor_specs():
        entities.append(
            _sensor(
                discovery_prefix=discovery_prefix,
                node_id=node_id,
                object_id=f"host_{object_id}",
                name=f"Host {name}",
                value_template=f"{{{{ value_json.host.{object_id} }}}}",
                unit=unit,
                device_class=device_class,
            )
        )

    for temp in cfg.temps:
        entities.append(
            _sensor(
                discovery_prefix=discovery_prefix,
                node_id=node_id,
                object_id=f"temp_{temp}",
                name=f"{temp} temperature",
                value_template=f"{{{{ value_json.obs.temps.{temp} }}}}",
                unit="°C",
                device_class="temperature",
            )
        )

    for ch in cfg.channels:
        entities.append(
            _sensor(
                discovery_prefix=discovery_prefix,
                node_id=node_id,
                object_id=f"rpm_{ch}",
                name=f"{ch} fan RPM",
                value_template=f"{{{{ value_json.obs.rpm.{ch} }}}}",
                unit="rpm",
            )
        )
        entities.append(
            _sensor(
                discovery_prefix=discovery_prefix,
                node_id=node_id,
                object_id=f"pwm_{ch}",
                name=f"{ch} fan PWM",
                value_template=f"{{{{ (value_json.cmd.pwm.{ch} | float * 100) | round(0) }}}}",
                unit="%",
            )
        )

    for temp in cfg.setpoints:
        object_id = f"setpoint_{temp}"
        unique_id = f"{node_id}_{object_id}"
        payload = {
            "name": f"{temp} setpoint",
            "unique_id": unique_id,
            "object_id": unique_id,
            "state_topic": state_topic(node_id),
            "value_template": f"{{{{ value_json.setpoints.{temp} }}}}",
            "command_topic": f"{node_id}/cmd/setpoint/{temp}",
            "min": cfg.temp_min_c,
            "max": cfg.temp_max_c,
            "step": 0.5,
            "unit_of_measurement": "°C",
            "device": _device_block(node_id),
            **_availability(node_id),
        }
        topic = f"{discovery_prefix}/number/{node_id}/{object_id}/config"
        entities.append(MqttEntity("number", object_id, topic, payload))

    if control_mode is ControlMode.MANUAL:
        for ch in cfg.channels:
            object_id = f"pwm_cmd_{ch}"
            unique_id = f"{node_id}_{object_id}"
            payload = {
                "name": f"{ch} manual PWM",
                "unique_id": unique_id,
                "object_id": unique_id,
                "state_topic": state_topic(node_id),
                "value_template": f"{{{{ value_json.cmd.pwm.{ch} }}}}",
                "command_topic": f"{node_id}/cmd/pwm/{ch}",
                "min": cfg.pwm_min,
                "max": cfg.pwm_max,
                "step": 0.01,
                "device": _device_block(node_id),
                **_availability(node_id),
            }
            topic = f"{discovery_prefix}/number/{node_id}/{object_id}/config"
            entities.append(MqttEntity("number", object_id, topic, payload))

    return entities


# ---------------------------------------------------------------------------
# State payload
# ---------------------------------------------------------------------------


def state_payload(snapshot_dict: Mapping[str, Any], host: Mapping[str, Any]) -> dict[str, Any]:
    """The one retained JSON blob published to :func:`state_topic`.

    ``snapshot_dict`` is ``ControlSnapshot.to_dict()``; ``host`` is
    ``aqua_bridge.hostinfo.collect_hostinfo()``'s return value.
    """
    out = dict(snapshot_dict)
    out["host"] = dict(host)
    return out


# ---------------------------------------------------------------------------
# Inbound command parsing
# ---------------------------------------------------------------------------


def parse_command(node_id: str, topic: str, payload: bytes | str) -> Intent | None:
    """Turn one inbound MQTT message into an :class:`Intent`, or ``None``.

    Never raises: a garbage topic or payload (wrong type, not a number,
    unknown channel/enum value) is logged and rejected by returning
    ``None`` -- the caller simply does not call ``surface.submit()``.
    """
    try:
        text = payload.decode("utf-8") if isinstance(payload, bytes | bytearray) else str(payload)
    except UnicodeDecodeError:
        _LOG.warning("mqtt: undecodable payload on %s", topic)
        return None
    text = text.strip()

    prefix = f"{node_id}/cmd/"
    if not topic.startswith(prefix):
        return None
    tail = topic[len(prefix) :]

    try:
        if tail == "mode":
            return SetMode(mode=text)
        if tail == "preset":
            return SetPreset(name=text)
        if tail == "auto":
            return ClearOverride(channel=text or None)
        if tail.startswith("setpoint/"):
            channel = tail[len("setpoint/") :]
            return SetSetpoint(channel=channel, celsius=float(text))
        if tail.startswith("pwm/"):
            channel = tail[len("pwm/") :]
            return SetPwm(channel=channel, pwm=float(text))
    except (IntentError, ValueError) as exc:
        _LOG.info("mqtt: rejected command on %s: %s", topic, exc)
        return None

    _LOG.info("mqtt: unknown command topic %s", topic)
    return None


# ---------------------------------------------------------------------------
# Thin paho-mqtt 2.x wrapper
# ---------------------------------------------------------------------------


class MqttClient:
    """Wraps ``paho.mqtt.client.Client`` (CallbackAPIVersion.VERSION2).

    Kept thin on purpose: connection handling and the actual publish/tick
    loop belong to the glue loop. This class exists so the loop does not
    need to know Discovery topic shapes or command parsing, and so tests
    can drive ``on_connect``/``on_message`` directly without a broker.
    """

    def __init__(
        self,
        *,
        node_id: str,
        discovery_prefix: str,
        cfg: MpcConfig,
        host: str,
        port: int = 1883,
        username: str = "",
        password: str = "",
        on_intent: Callable[[Intent], None] | None = None,
        on_connection_change: Callable[[bool], None] | None = None,
    ) -> None:
        import paho.mqtt.client as mqtt

        self.node_id = node_id
        self.discovery_prefix = discovery_prefix
        self.cfg = cfg
        self._on_intent = on_intent
        self._on_connection_change = on_connection_change
        self.connected = False
        # topic filter -> handler, e.g. the SMART inbox on "{node_id}/in/smart/+"
        # (add_topic_handler docstring); empty by default, so a client built
        # without one behaves exactly as before.
        self._extra_handlers: dict[str, Callable[[str, bytes], None]] = {}
        self.client = mqtt.Client(
            callback_api_version=mqtt.CallbackAPIVersion.VERSION2,
            client_id=f"aqua-bridge-{node_id}",
        )
        if username:
            self.client.username_pw_set(username, password or None)
        self.client.will_set(availability_topic(node_id), payload="offline", qos=1, retain=True)
        self.client.on_connect = self._on_connect
        self.client.on_disconnect = self._on_disconnect
        self.client.on_message = self._on_message
        self._host = host
        self._port = port

    # -- connection -----------------------------------------------------

    def connect(self) -> None:
        self.client.connect(self._host, self._port)

    def connect_async(self) -> None:
        """Connect from the network thread (``loop_start``) with automatic
        reconnects, so a broker that is down at boot does not stall the
        daemon and a broker restart is picked up without help."""
        self.client.reconnect_delay_set(min_delay=1, max_delay=60)
        self.client.connect_async(self._host, self._port)

    def loop_start(self) -> None:
        self.client.loop_start()

    def loop_stop(self) -> None:
        self.client.loop_stop()

    def disconnect(self) -> None:
        self.publish_availability(False)
        self.client.disconnect()

    # -- callbacks (also callable directly from tests) -------------------

    def _on_connect(self, client, userdata, flags, reason_code, properties=None) -> None:
        for topic in command_topics(self.node_id, self.cfg).values():
            client.subscribe(topic)
        for topic_filter in self._extra_handlers:
            client.subscribe(topic_filter)
        self.publish_availability(True)
        self._set_connected(True)

    def _on_disconnect(self, client, userdata, flags, reason_code=None, properties=None) -> None:
        self._set_connected(False)

    def _set_connected(self, connected: bool) -> None:
        self.connected = connected
        if self._on_connection_change is not None:
            try:
                self._on_connection_change(connected)
            except Exception:  # a listener bug must not kill the paho network thread
                _LOG.exception("mqtt: on_connection_change failed")

    def add_topic_handler(self, topic_filter: str, handler: Callable[[str, bytes], None]) -> None:
        """Routes messages on ``topic_filter`` (an MQTT filter, ``+``/``#``
        allowed) to ``handler(topic, payload)`` on this same connection --
        one broker connection and one network thread for the whole daemon,
        rather than every inbound data source opening its own (the DAS
        plan's SMART inbox is the first user: ``smart_topic_filter(node_id)``
        -> ``SmartInbox.on_message``). Subscribed immediately if already
        connected, and on every future ``on_connect`` alongside the command
        topics; call before :meth:`connect`/:meth:`connect_async` for a
        topic that must be subscribed from the very first connection.
        ``handler`` must not raise: :meth:`_on_message` does not catch
        exceptions from it, matching the plan's requirement that a SMART
        listener "never raises into the MQTT thread" -- the handler itself
        (:meth:`aqua_bridge.publishers.inputs.SmartInbox.on_message`) is the
        one that promises this, not this wrapper.
        """
        self._extra_handlers[topic_filter] = handler
        if self.connected:
            self.client.subscribe(topic_filter)

    def _on_message(self, client, userdata, msg) -> None:
        """Never raises: paho 2.x runs callbacks on its network thread with
        ``suppress_exceptions=False`` by default, so an exception here would
        end that thread (no more publishes; the LWT eventually flips to
        ``offline``). A rejected intent -- raw PWM while auto, an
        out-of-range setpoint -- is logged and dropped, mirroring the HTTP
        4xx/409 path (section 6). A message matching a filter registered via
        :meth:`add_topic_handler` (and not a ``cmd/`` topic) is routed there
        instead of through :func:`parse_command`."""
        prefix = f"{self.node_id}/cmd/"
        if not msg.topic.startswith(prefix):
            for topic_filter, handler in self._extra_handlers.items():
                if topic_matches(topic_filter, msg.topic):
                    handler(msg.topic, msg.payload)
                    return
        intent = parse_command(self.node_id, msg.topic, msg.payload)
        if intent is None or self._on_intent is None:
            return
        try:
            self._on_intent(intent)
        except IntentError as exc:
            _LOG.info("mqtt: rejected %s on %s: %s", intent, msg.topic, exc)
        except Exception:  # a surface bug must not kill the paho network thread
            _LOG.exception("mqtt: on_intent failed for %s on %s", intent, msg.topic)

    # -- publishing -------------------------------------------------------

    def publish_availability(self, online: bool) -> None:
        self.client.publish(
            availability_topic(self.node_id),
            payload="online" if online else "offline",
            qos=1,
            retain=True,
        )

    def publish_discovery(self, *, control_mode: ControlMode) -> None:
        """Publish every entity for ``control_mode`` and delete (empty retained
        payload) the ones that exist only in another mode -- the PWM number
        entities when leaving ``manual`` (section 7)."""

        def entities(mode: ControlMode) -> list[MqttEntity]:
            return build_discovery_entities(
                self.cfg,
                node_id=self.node_id,
                discovery_prefix=self.discovery_prefix,
                control_mode=mode,
            )

        current = entities(control_mode)
        current_topics = {e.config_topic for e in current}
        for entity in current:
            self.client.publish(
                entity.config_topic,
                payload=json.dumps(entity.payload),
                qos=1,
                retain=True,
            )
        stale: set[str] = set()
        for other in ControlMode:
            if other is control_mode:
                continue
            stale.update(e.config_topic for e in entities(other))
        for topic in sorted(stale - current_topics):
            self.client.publish(topic, payload="", qos=1, retain=True)

    def publish_state(self, snapshot_dict: Mapping[str, Any], host: Mapping[str, Any]) -> None:
        payload = state_payload(snapshot_dict, host)
        self.client.publish(
            state_topic(self.node_id),
            payload=json.dumps(payload, allow_nan=False),
            qos=0,
            retain=True,
        )
