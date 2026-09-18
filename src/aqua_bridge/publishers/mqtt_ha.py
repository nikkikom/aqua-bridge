"""MQTT Discovery + Home Assistant bridge (PROJECT.md section 7).

Pure, broker-independent functions build the Discovery config payloads and
the state/command topics; :class:`MqttClient` is a thin ``paho-mqtt`` 2.x
wrapper around them, not exercised against a live broker (see
``tests/test_mqtt_ha.py``).

Topic layout (``node_id`` from ``config.yaml`` ``mqtt.node_id``):

* ``{node_id}/status``            -- LWT, ``online``/``offline``
* ``{node_id}/state``             -- retained JSON, one big blob every tick
  (``ControlSnapshot.to_dict()`` plus a ``host`` key: the host metrics and the
  board's decoded ``throttled`` word, :mod:`aqua_bridge.hostinfo`)
* ``{node_id}/cmd/mode``          -- raw string, one of :class:`ControlMode`
* ``{node_id}/cmd/preset``        -- raw string, one of :class:`Preset`
* ``{node_id}/cmd/auto``          -- raw string channel name, or empty = all
* ``{node_id}/cmd/setpoint/<temp>``   -- raw number, target Celsius
* ``{node_id}/cmd/pwm/<channel>``     -- raw number, manual PWM 0..1
* ``{node_id}/cmd/limit/<bay>``       -- raw number, drive limit of one bay (DAS mode)
* ``{node_id}/cmd/limit/class/<class>`` -- raw number, limit of a drive class (DAS mode)
* ``{node_id}/cmd/bay/<bay>``         -- raw string ``occupied`` | ``empty`` | ``auto``,
  the bay's declared occupancy (DAS mode; ``POST /api/bay`` for class and serial)
* ``{node_id}/cmd/ident``             -- raw string ``start:group:<group>`` |
  ``start:channel:<channel>`` | ``start:<channel>`` | ``stop``: start or stop an
  identification experiment (DAS mode, ``POST /api/ident``); a retained ``start`` is
  ignored (it would be redelivered on every reconnect), only a live one starts
* ``{node_id}/cmd/calibrate/<bay>``   -- raw number, one hand-measured drive temperature
  in degC (DAS mode, ``POST /api/calibrate``). **Off by default** and only subscribed,
  parsed and accepted with ``mqtt.allow_calibrate: true``; see *Manual calibration over
  MQTT* below

Manual calibration over MQTT (PROJECT.md section 8 items 23 and 105)
--------------------------------------------------------------------
``cmd/calibrate/<bay>`` exists but is **off unless the owner turns it on**, and this
module refuses it in two independent places when it is off: the topic is not in
:func:`command_topics`, so nothing subscribes to it, and :func:`parse_command` returns
``None`` for it, so a message that arrives anyway (a wildcard subscription, a replayed
session) produces no intent and never reaches the supervisor or the estimator.

Why off by default, when ``cmd/limit`` and ``cmd/bay`` are on: MQTT has no
authentication of its own -- every inbound topic relies entirely on the broker's ACL
(section 7, *Security*), where the HTTPS route has its own (section 6). A limit or a
declared occupancy is *policy*: it is a number the gate re-clamps every tick, it is
visible in Home Assistant as the entity that carries it, and the safety core bounds
what it can do. A calibration is not policy but *measurement*: it fits the bay's
sensor-to-drive map, so a wrong reading biases every later estimate of that bay -- and
an estimate biased low makes the controller run the fans slower than the drive needs.
It is the one inbound topic whose payload can quietly reduce cooling, so it asks for a
deliberate ``true`` in ``config.yaml`` rather than riding in on the broker's ACL by
default.

Its guards, when it *is* on:

* ``mqtt.allow_calibrate: true`` -- the explicit opt-in, checked in
  :func:`validate_mqtt_section` like every other scalar of the section
* ``mqtt.username`` must be set with it, or the section is a named
  :class:`MqttSetupError` at startup. An anonymous connection cannot be given an ACL of
  its own, so "the broker's ACL" would be no guard at all. Read it for exactly what it
  is: a *configured username is required*, checked once when the config is parsed. It is
  not a check that the connection is authenticated -- nothing here inspects the CONNACK
  or the broker's ACL, and a broker running ``allow_anonymous true``, or one that takes
  the username without checking the password, would accept the same calibration. The
  deliberate ``true`` plus a named account is a speed bump the owner sees at startup;
  the authentication itself is the broker's job to enforce
* a **retained** calibration is ignored and logged, exactly as a retained experiment
  ``start`` is: a retained number would be redelivered on every reconnect and refitted
  into the bay's map on every restart, which is the failure item 104 is about
* no Discovery entity. A ``number`` per bay would be fifteen more entities, and Home
  Assistant restores a ``number``'s state on restart by publishing it again -- the
  retained-message problem in another costume. The topic is for a script or an
  automation that publishes one live reading, not for a slider

Everything past :func:`parse_command` is the ordinary :class:`Calibrate` path: the
supervisor's own checks (the bay, the range, ``no_tick`` / ``empty`` / ``untrusted``)
apply unchanged, and a refused reading costs no running identification experiment.

Device health (PROJECT.md section 8 items 79 and 83) is one binary sensor,
``device_problem`` (``device_class: problem``, diagnostic): on whenever
``value_json.health.device_health.ok`` is false, that is whenever any controller
reports a stuck output, an aquabus slot with no device behind it, a commanded
output not in PWM mode or an unconfigured controller block, or whenever a fan has
drifted from its fitted curve, its rail has sagged or its power is out of line
with its duty. Its attributes are the whole ``device_health`` blob from the same
retained state topic -- per controller and per channel, with the flow sensors and
(once another change publishes one) the aquaero's active profile -- so the detail
is one tap away in Home Assistant without a second entity per output.

The Raspberry Pi the daemon runs on has its own binary sensor, ``host_problem``
(PROJECT.md section 8 item 103): on whenever ``value_json.device_health.host.ok``
is false, that is whenever the board has been above ``host_health.temp_limit_c``
for ``temp_fault_s``, is throttling now, has sat further than ``divergence_c``
from the enclosure air for ``divergence_fault_s`` (only while its CPU is idle),
the card the daemon runs from has been below ``disk_free_min_gb`` free for
``disk_free_fault_s``, or that card's filesystem has gone read-only. Its
attributes are the host half of the same blob: the board's temperature, the air
reference, the load average, the decoded ``get_throttled`` word, and the disk
free space and read-only state (``disk_free_gb``, ``disk_used_pct``,
``read_only`` -- ``disk_free_gb`` also has its own sensor, ``host_disk_free_gb``,
so it can be charted and alerted on directly rather than read only from these
attributes). The board -- and the card it runs from -- are a health signal
only -- never a solver input, never a zone air sensor -- so they get a sensor of
their own rather than being read as a controller fault. Only the *facts* -- the
board is hot, it is throttling now, the filesystem is read-only -- also join the
daemon-wide ``health.device_health.problems`` list behind ``device_problem``;
the divergence rule and the free-space rule are *hints*, not verdicts, so each
turns on ``host_problem`` alone and leaves ``device_problem`` for something that
is actually broken. The free-space rule is a hint for the same reason
divergence is: a filling card can sit below its threshold for days, and a
daemon-wide flag latched that long would read exactly like a missing aquabus
device.

The aquaero's aquabus (PROJECT.md section 8 items 92, 114, 115, 129) has its own
binary sensor, ``aquabus_problem`` (``device_class: problem``, diagnostic, both
modes): on whenever some controller's ``value_json.device_health.devices[].aquabus.
lost`` is true -- a device that had answered has now been missing for
``bus_absent_s``. A bus that has simply never had anything on it (``state:
"never_seen"``, the normal state of an aquaero with nothing on its aquabus) is not
a problem and never turns this on; neither is a report or two of ``"empty"`` before
``bus_absent_s`` has passed. Its attributes are the ``aquabus`` block of every
controller in ``device_health.devices`` (``state``, ``present``, ``seen``,
``absent_s``, ``lost``, ``temps_missing``), so the state a person needs --
``never_seen`` apart from ``lost`` -- is one tap away without reading the journal.

DAS mode also publishes, per zone, one sensor ``model_block_<zone>``
(PROJECT.md section 8 items 110, 111, 121): state is the zone's
``value_json.cmd.diagnostics.thermal.zones.<zone>.blocked`` list joined with ", ",
or ``"none"`` once the zone has nothing left to wait on -- naming the gate and the
group or bay it names (``windows:<block>``, ``pe:<block>``, ``rel_se:<coefficient>``,
``pred_err``) rather than leaving a zone that has sat in ``learning`` for a day
looking the same as one that just started. Its attributes are the whole per-zone
``diagnostics["thermal"]["zones"][<zone>]`` block, so ``pe_diag`` (the measured
diagonal beside ``pe_min``, naming which group is not moving) and every other field
of the identification summary are one tap away too. This is a value, not a fault:
there is no ``device_class`` and nothing here turns a problem sensor on, because a
zone still learning is not broken (module docstring, ``zone_status_<zone>`` below,
which is the solver's own trust/fault state and stays separate).

DAS mode also publishes one sensor ``unexcitable_channels``
(PROJECT.md section 8 item 121, :func:`aqua_bridge.control.ident.excitation`):
state is ``value_json.extra.experiment.unexcitable`` joined with ", ", or ``"none"``
when every channel can clear the model's PE bound at ``ident_amplitude`` from where
it sits. Its attributes are the full per-channel ``excitation`` mapping (``rel_swing``,
``pe_reach``, ``pe_bound``, ``excitable``, ``pe_reach_at_cap``, ``excitable_at_cap``).

DAS mode (``mpc.topology``) subscribes to the limit and bay topics and adds one
``limit_<class>`` number entity per drive class (state from
``value_json.limits.classes.<class>``, range ``temp_min_c`` .. the configured
limit); setpoint topics and numbers exist only for temperatures with a
setpoint, so a DAS config without setpoints publishes none. A tail after
``limit/`` that starts with ``class/`` is a class, anything else a bay name.

Per bay, DAS mode also publishes the estimator's view from
``value_json.cmd.diagnostics``: sensors ``drive_temp_<bay>`` (estimated drive
temperature, ``estimates.<bay>.t_c``), ``drive_margin_<bay>`` (degrees left to the
limit after the uncertainty margin, ``estimates.<bay>.limit_margin_c`` =
``limit - k * sigma - t``; negative means the drive may be over its limit) and
``drive_sigma_<bay>`` (``estimates.<bay>.sigma_c``), and a binary sensor
``bay_occupied_<bay>`` (``bays.<bay>.occupancy``: ``empty`` is off, ``occupied``
and ``unknown`` are on, the conservative reading). An empty bay has no estimate,
so its drive sensors read ``None`` (unknown in Home Assistant).

DAS mode also publishes the zoned thermal model's identification
(``value_json.cmd.diagnostics.thermal``, :mod:`aqua_bridge.control.thermal`): sensors
``model_status`` (``prior`` | ``learning`` | ``converged`` | ``suspect`` | ``error``,
``off`` without ``model_shadow``) and ``model_pred_err_c`` (the worst zone's
one-window prediction error, degC; ``None`` until a window has closed), and the
fan noise index ``noise_db`` (``value_json.cmd.diagnostics.noise.db_index``,
:mod:`aqua_bridge.control.noise`: energetic total from the fans' speed; an index,
absolute only with datasheet ``noise_db_at_max`` values).

DAS mode also publishes the binary sensor ``ident_running``
(``value_json.extra.experiment.running``, :mod:`aqua_bridge.control.ident`): on while
an identification experiment commands fans.

DAS mode also publishes one sensor ``zone_status_<zone>`` per zone (plan section 8,
``GET /api/zones``): state from ``value_json.cmd.diagnostics.zones.<zone>.policy``
(``solver`` while the zone is trusted and fault-free, ``hold`` / ``ramp_high`` while
its own fault holds or ramps its channels, ``coupled`` while it only carries another
faulted zone's channels), ``off`` before the first DAS tick.

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
    Calibrate,
    ClearOverride,
    ControlMode,
    Ident,
    Intent,
    IntentError,
    SetBay,
    SetLimit,
    SetMode,
    SetPreset,
    SetPwm,
    SetSetpoint,
)
from aqua_bridge.model import MpcConfig

__all__ = [
    "MqttClient",
    "MqttEntity",
    "MqttSetupError",
    "availability_topic",
    "build_discovery_entities",
    "command_topics",
    "host_sensor_specs",
    "parse_command",
    "state_payload",
    "state_topic",
    "topic_matches",
    "validate_mqtt_section",
]

_LOG = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# Config validation (item 57): stdlib-only like httpauth.HttpSettings, so a
# minimal install without paho-mqtt can still tell a bad mqtt: section from a
# deliberately disabled one before anything here is used against a broker.
# ---------------------------------------------------------------------------


class MqttSetupError(ValueError):
    """The ``mqtt:`` section has a bad value; the caller logs it and leaves MQTT off."""


#: Every mqtt: boolean, all defaulting to ``false``. ``allow_calibrate`` is the opt-in
#: of item 105 (module docstring, *Manual calibration over MQTT*).
_MQTT_BOOL_KEYS: tuple[str, ...] = ("enabled", "allow_calibrate")

#: (key, default) for every mqtt: string scalar; a YAML-null value falls back
#: to the default rather than erroring (a bare ``username:`` line is common).
_MQTT_STRING_KEYS: tuple[tuple[str, str], ...] = (
    ("host", "localhost"),
    ("username", ""),
    ("password", ""),
    ("discovery_prefix", "homeassistant"),
    ("node_id", "aqua-bridge"),
)


def validate_mqtt_section(section: Mapping[str, Any] | None) -> dict[str, Any]:
    """Type-checks the ``mqtt:`` section, :class:`MqttSetupError` naming the key.

    Item 57: ``mqtt.enabled: "true"`` (a string, not a bool) must not silently
    read as *off* -- every scalar is checked, not only ``enabled``. Returns a
    plain ``dict`` (defaults filled in) rather than a dataclass, since
    :class:`MqttClient` already owns the rest of the connection's shape.
    """
    data = dict(section or {})
    values: dict[str, Any] = {}
    for name in _MQTT_BOOL_KEYS:
        value = data.get(name, False)
        if not isinstance(value, bool):
            raise MqttSetupError(f"mqtt.{name} must be true or false, got {value!r}")
        values[name] = value
    for name, default in _MQTT_STRING_KEYS:
        value = data.get(name, default)
        if value is None:
            value = default
        if not isinstance(value, str):
            raise MqttSetupError(f"mqtt.{name} must be a string, got {type(value).__name__}")
        values[name] = value
    port = data.get("port", 1883)
    if isinstance(port, bool) or not isinstance(port, int) or not 1 <= port <= 65535:
        raise MqttSetupError(f"mqtt.port must be an integer in [1, 65535], got {port!r}")
    values["port"] = port
    if values["allow_calibrate"] and not values["username"]:
        # Item 105: the inbound topics have no authentication of their own, only the
        # broker's ACL, and an anonymous connection cannot be given one. Refused here,
        # once and by name, rather than per message.
        raise MqttSetupError(
            "mqtt.allow_calibrate: true needs mqtt.username: an anonymous broker "
            "connection cannot be given an ACL of its own, and cmd/calibrate has no "
            "authentication besides the broker's"
        )
    return values


# ---------------------------------------------------------------------------
# Topics
# ---------------------------------------------------------------------------


def availability_topic(node_id: str) -> str:
    return f"{node_id}/status"


def state_topic(node_id: str) -> str:
    return f"{node_id}/state"


def command_topics(
    node_id: str, cfg: MpcConfig, *, allow_calibrate: bool = False
) -> dict[str, str]:
    """Every inbound command topic this bridge subscribes to.

    ``allow_calibrate`` is ``mqtt.allow_calibrate`` (item 105, module docstring): without
    it there is no ``cmd/calibrate/<bay>`` topic at all, so nothing subscribes to one."""
    topics = {
        "mode": f"{node_id}/cmd/mode",
        "preset": f"{node_id}/cmd/preset",
        "auto": f"{node_id}/cmd/auto",
    }
    for temp in cfg.setpoints:
        topics[f"setpoint/{temp}"] = f"{node_id}/cmd/setpoint/{temp}"
    if cfg.topology is not None:
        for bay in cfg.topology.bays:
            topics[f"limit/{bay}"] = f"{node_id}/cmd/limit/{bay}"
        for drive_class in cfg.drive_classes:
            topics[f"limit/class/{drive_class}"] = f"{node_id}/cmd/limit/class/{drive_class}"
        for bay in cfg.topology.bays:
            topics[f"bay/{bay}"] = f"{node_id}/cmd/bay/{bay}"
        topics["ident"] = f"{node_id}/cmd/ident"
        if allow_calibrate:
            for bay in cfg.topology.bays:
                topics[f"calibrate/{bay}"] = f"{node_id}/cmd/calibrate/{bay}"
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


def _sensor_with_attributes(
    *,
    discovery_prefix: str,
    node_id: str,
    object_id: str,
    name: str,
    value_template: str,
    json_attributes_template: str,
) -> MqttEntity:
    """A diagnostic-category text sensor whose state is a short summary and whose
    ``json_attributes`` carry the detail behind it, one tap away -- the same shape as
    the ``device_problem``/``host_problem`` binary sensors below, for a *value*
    rather than a condition (PROJECT.md section 8 item 121: ``pe_diag``, ``blocked``
    and ``excitation`` are readings about the model, not faults, so they get a
    ``sensor``, never a ``binary_sensor`` with ``device_class: problem``)."""
    unique_id = f"{node_id}_{object_id}"
    payload: dict[str, Any] = {
        "name": name,
        "unique_id": unique_id,
        "object_id": unique_id,
        "state_topic": state_topic(node_id),
        "value_template": value_template,
        "json_attributes_topic": state_topic(node_id),
        "json_attributes_template": json_attributes_template,
        "entity_category": "diagnostic",
        "device": _device_block(node_id),
        **_availability(node_id),
    }
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
        ("disk_free_gb", "Disk free", "GB", None),
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

    # Device health (PROJECT.md section 8 item 83): one problem sensor for the whole
    # daemon. Its attributes carry the detail -- per controller the stuck outputs,
    # the absent aquabus slots, the outputs not in PWM mode and the flow sensors,
    # per channel the fan-health verdict -- from the same retained state topic.
    object_id = "device_problem"
    unique_id = f"{node_id}_{object_id}"
    entities.append(
        MqttEntity(
            "binary_sensor",
            object_id,
            f"{discovery_prefix}/binary_sensor/{node_id}/{object_id}/config",
            {
                "name": "Controller problem",
                "unique_id": unique_id,
                "object_id": unique_id,
                "state_topic": state_topic(node_id),
                "value_template": (
                    "{{ 'OFF' if value_json.health.device_health.ok | default(true) else 'ON' }}"
                ),
                "payload_on": "ON",
                "payload_off": "OFF",
                "device_class": "problem",
                "entity_category": "diagnostic",
                "json_attributes_topic": state_topic(node_id),
                "json_attributes_template": (
                    "{{ value_json.device_health | default({}) | tojson }}"
                ),
                "device": _device_block(node_id),
                **_availability(node_id),
            },
        )
    )

    # The board itself (PROJECT.md section 8 item 103): its own problem sensor, so a hot
    # or throttling Pi is not read as a controller fault. Its attributes are the host
    # half of the same blob -- the board's temperature, the enclosure-air reference it
    # is compared against, the load average and the decoded get_throttled word.
    object_id = "host_problem"
    unique_id = f"{node_id}_{object_id}"
    entities.append(
        MqttEntity(
            "binary_sensor",
            object_id,
            f"{discovery_prefix}/binary_sensor/{node_id}/{object_id}/config",
            {
                "name": "Board problem",
                "unique_id": unique_id,
                "object_id": unique_id,
                "state_topic": state_topic(node_id),
                "value_template": (
                    "{{ 'OFF' if value_json.device_health.host.ok | default(true) else 'ON' }}"
                ),
                "payload_on": "ON",
                "payload_off": "OFF",
                "device_class": "problem",
                "entity_category": "diagnostic",
                "json_attributes_topic": state_topic(node_id),
                "json_attributes_template": (
                    "{{ value_json.device_health.host | default({}) | tojson }}"
                ),
                "device": _device_block(node_id),
                **_availability(node_id),
            },
        )
    )

    # The aquabus itself (PROJECT.md section 8 items 92, 114, 115, 129): on only when
    # a device that had answered has now been missing for bus_absent_s -- a healthy
    # aquaero with nothing on its bus ("never_seen") never turns this on, which is why
    # the template checks "lost" and not merely the absence of a device.
    object_id = "aquabus_problem"
    unique_id = f"{node_id}_{object_id}"
    entities.append(
        MqttEntity(
            "binary_sensor",
            object_id,
            f"{discovery_prefix}/binary_sensor/{node_id}/{object_id}/config",
            {
                "name": "Aquabus problem",
                "unique_id": unique_id,
                "object_id": unique_id,
                "state_topic": state_topic(node_id),
                "value_template": (
                    "{{ 'ON' if (value_json.device_health.devices | default([]) "
                    "| selectattr('aquabus.lost', 'equalto', true) | list | length > 0) "
                    "else 'OFF' }}"
                ),
                "payload_on": "ON",
                "payload_off": "OFF",
                "device_class": "problem",
                "entity_category": "diagnostic",
                "json_attributes_topic": state_topic(node_id),
                "json_attributes_template": (
                    "{{ (value_json.device_health.devices | default([]) "
                    "| map(attribute='aquabus') | list) | tojson }}"
                ),
                "device": _device_block(node_id),
                **_availability(node_id),
            },
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

    for drive_class, dc in cfg.drive_classes.items():  # empty in legacy mode
        object_id = f"limit_{drive_class}"
        unique_id = f"{node_id}_{object_id}"
        payload = {
            "name": f"{drive_class} drive limit",
            "unique_id": unique_id,
            "object_id": unique_id,
            "state_topic": state_topic(node_id),
            "value_template": f"{{{{ value_json.limits.classes.{drive_class} }}}}",
            "command_topic": f"{node_id}/cmd/limit/class/{drive_class}",
            "min": cfg.temp_min_c,
            "max": dc.limit_c,
            "step": 0.5,
            "unit_of_measurement": "°C",
            "device": _device_block(node_id),
            **_availability(node_id),
        }
        topic = f"{discovery_prefix}/number/{node_id}/{object_id}/config"
        entities.append(MqttEntity("number", object_id, topic, payload))

    if cfg.topology is not None:
        thermal = "value_json.cmd.diagnostics.thermal"
        entities.append(
            _sensor(
                discovery_prefix=discovery_prefix,
                node_id=node_id,
                object_id="model_status",
                name="Thermal model status",
                value_template=f"{{{{ {thermal}.status | default('off') }}}}",
                state_class=None,
            )
        )
        entities.append(
            _sensor(
                discovery_prefix=discovery_prefix,
                node_id=node_id,
                object_id="model_pred_err_c",
                name="Thermal model prediction error",
                value_template=f"{{{{ {thermal}.pred_err_c | default(None) }}}}",
                unit="°C",
            )
        )
        entities.append(
            _sensor(
                discovery_prefix=discovery_prefix,
                node_id=node_id,
                object_id="noise_db",
                name="Fan noise index",
                value_template=("{{ value_json.cmd.diagnostics.noise.db_index | default(None) }}"),
                unit="dB",
            )
        )
        entities.append(
            _sensor_with_attributes(
                discovery_prefix=discovery_prefix,
                node_id=node_id,
                object_id="unexcitable_channels",
                name="Unexcitable channels",
                value_template=(
                    "{{ (value_json.extra.experiment.unexcitable | default([]) "
                    "| join(', ')) or 'none' }}"
                ),
                json_attributes_template=(
                    "{{ value_json.extra.experiment.excitation | default({}) | tojson }}"
                ),
            )
        )
        object_id = "ident_running"
        unique_id = f"{node_id}_{object_id}"
        payload = {
            "name": "Identification experiment running",
            "unique_id": unique_id,
            "object_id": unique_id,
            "state_topic": state_topic(node_id),
            "value_template": (
                "{{ 'ON' if value_json.extra.experiment.running | default(false) else 'OFF' }}"
            ),
            "payload_on": "ON",
            "payload_off": "OFF",
            "device_class": "running",
            "device": _device_block(node_id),
            **_availability(node_id),
        }
        topic = f"{discovery_prefix}/binary_sensor/{node_id}/{object_id}/config"
        entities.append(MqttEntity("binary_sensor", object_id, topic, payload))

        for zone in cfg.topology.zones:
            entities.append(
                _sensor(
                    discovery_prefix=discovery_prefix,
                    node_id=node_id,
                    object_id=f"zone_status_{zone}",
                    name=f"{zone} zone status",
                    value_template=(
                        f"{{{{ value_json.cmd.diagnostics.zones.{zone}.policy | default('off') }}}}"
                    ),
                    state_class=None,
                )
            )
            thermal_zone = f"value_json.cmd.diagnostics.thermal.zones.{zone}"
            entities.append(
                _sensor_with_attributes(
                    discovery_prefix=discovery_prefix,
                    node_id=node_id,
                    object_id=f"model_block_{zone}",
                    name=f"{zone} zone convergence block",
                    value_template=(
                        f"{{{{ ({thermal_zone}.blocked | default([]) | join(', ')) or 'none' }}}}"
                    ),
                    json_attributes_template=f"{{{{ {thermal_zone} | default({{}}) | tojson }}}}",
                )
            )

    bays = {} if cfg.topology is None else cfg.topology.bays
    for bay in bays:  # empty in legacy mode
        estimate = f"value_json.cmd.diagnostics.estimates.{bay}"
        for object_id, name, key, device_class in (
            (f"drive_temp_{bay}", f"{bay} drive temperature", "t_c", "temperature"),
            (f"drive_margin_{bay}", f"{bay} drive margin to limit", "limit_margin_c", None),
            (f"drive_sigma_{bay}", f"{bay} drive temperature sigma", "sigma_c", None),
        ):
            entities.append(
                _sensor(
                    discovery_prefix=discovery_prefix,
                    node_id=node_id,
                    object_id=object_id,
                    name=name,
                    value_template=f"{{{{ {estimate}.{key} | default(None) }}}}",
                    unit="°C",
                    device_class=device_class,
                )
            )
        object_id = f"bay_occupied_{bay}"
        unique_id = f"{node_id}_{object_id}"
        payload = {
            "name": f"{bay} occupied",
            "unique_id": unique_id,
            "object_id": unique_id,
            "state_topic": state_topic(node_id),
            "value_template": (
                f"{{{{ 'OFF' if value_json.cmd.diagnostics.bays.{bay}.occupancy "
                "| default('unknown') == 'empty' else 'ON' }}"
            ),
            "payload_on": "ON",
            "payload_off": "OFF",
            "device_class": "occupancy",
            "device": _device_block(node_id),
            **_availability(node_id),
        }
        topic = f"{discovery_prefix}/binary_sensor/{node_id}/{object_id}/config"
        entities.append(MqttEntity("binary_sensor", object_id, topic, payload))

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


#: ``cmd/bay/<bay>`` payload -> ``SetBay`` ``occupied`` value.
_BAY_PAYLOADS: dict[str, bool | str] = {"occupied": True, "empty": False, "auto": "auto"}


def _parse_ident(text: str) -> Ident:
    """``cmd/ident`` payload: ``stop`` | ``start:group:<g>`` | ``start:channel:<ch>`` |
    ``start:<ch>``. Raises ``ValueError`` / ``IntentError`` for anything else."""
    if text == "stop":
        return Ident(action="stop")
    action, sep, rest = text.partition(":")
    if action != "start" or not sep or not rest:
        raise ValueError(f"ident payload must be 'stop' or 'start:...', got {text!r}")
    kind, sep, name = rest.partition(":")
    if sep and kind == "group":
        return Ident(action="start", group=name)
    if sep and kind == "channel":
        return Ident(action="start", channel=name)
    return Ident(action="start", channel=rest)


def parse_command(
    node_id: str, topic: str, payload: bytes | str, *, allow_calibrate: bool = False
) -> Intent | None:
    """Turn one inbound MQTT message into an :class:`Intent`, or ``None``.

    Never raises: a garbage topic or payload (wrong type, not a number,
    unknown channel/enum value) is logged and rejected by returning
    ``None`` -- the caller simply does not call ``surface.submit()``.

    ``allow_calibrate`` is ``mqtt.allow_calibrate`` (item 105, module docstring). Without
    it a ``cmd/calibrate/<bay>`` message is refused here as well as never subscribed to,
    and the refusal returns ``None`` before any :class:`Calibrate` is constructed, so
    nothing downstream -- supervisor, estimator, running experiment -- is touched by it.
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
        if tail.startswith("limit/class/"):
            return SetLimit(limit_c=float(text), drive_class=tail[len("limit/class/") :])
        if tail.startswith("limit/"):
            return SetLimit(limit_c=float(text), bay=tail[len("limit/") :])
        if tail.startswith("bay/"):
            occupied = _BAY_PAYLOADS.get(text)
            if occupied is None:
                raise ValueError(
                    f"bay payload must be one of {sorted(_BAY_PAYLOADS)}, got {text!r}"
                )
            return SetBay(bay=tail[len("bay/") :], changes={"occupied": occupied})
        if tail.startswith("calibrate/"):
            if not allow_calibrate:
                _LOG.warning(
                    "mqtt: refused a calibration on %s; MQTT calibration is off "
                    "(set mqtt.allow_calibrate: true to enable it)",
                    topic,
                )
                return None
            return Calibrate(bay=tail[len("calibrate/") :], drive_temp_c=float(text))
        if tail == "ident":
            return _parse_ident(text)
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
        allow_calibrate: bool = False,
        on_intent: Callable[[Intent], None] | None = None,
        on_connection_change: Callable[[bool], None] | None = None,
    ) -> None:
        import paho.mqtt.client as mqtt

        self.node_id = node_id
        self.discovery_prefix = discovery_prefix
        self.cfg = cfg
        #: ``mqtt.allow_calibrate`` (item 105, module docstring): off by default.
        self.allow_calibrate = bool(allow_calibrate)
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
        topics = command_topics(self.node_id, self.cfg, allow_calibrate=self.allow_calibrate)
        for topic in topics.values():
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
        intent = parse_command(
            self.node_id, msg.topic, msg.payload, allow_calibrate=self.allow_calibrate
        )
        if intent is None or self._on_intent is None:
            return
        if isinstance(intent, Ident) and intent.action == "start" and getattr(msg, "retain", False):
            # A retained start is redelivered on every (re)connect: an experiment starts
            # only from a live, explicit command (reviewer fix; a retained stop is kept).
            _LOG.info("mqtt: ignored retained experiment start on %s", msg.topic)
            return
        if isinstance(intent, Calibrate) and getattr(msg, "retain", False):
            # Same reason, and worse: a retained reading would be refitted into the bay's
            # sensor-to-drive map on every reconnect and every restart (items 104, 105).
            _LOG.info("mqtt: ignored retained calibration on %s", msg.topic)
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
