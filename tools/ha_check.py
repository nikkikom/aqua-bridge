#!/usr/bin/env python3
"""Read-only MQTT / Home Assistant check against the live broker (PROJECT.md sections 7,
10 and section 8 item 21).

Run it on the Pi, as the service user, with the daemon's own config file::

    .venv/bin/python tools/ha_check.py --config /etc/aqua-bridge/config.yaml

It reads the ``mqtt:`` section for the broker, the credentials, ``node_id`` and
``discovery_prefix``, connects with its own client id (never the daemon's --
a duplicate client id would kick the daemon off the broker), subscribes to

* ``{discovery_prefix}/+/{node_id}/+/config`` -- the Discovery configs this daemon
  announces (not the whole ``{discovery_prefix}/#`` tree, which on a real Home
  Assistant broker carries every other integration's entities; a leftover config
  under a *previous* ``node_id`` is therefore invisible here, and HA is the place
  to spot that one)
* ``{node_id}/#`` -- the daemon's own status, state, command and ``in/smart`` topics

collects for ``--wait`` seconds (retained messages arrive immediately; a live state
blob within one tick) and prints a checklist:

* the availability topic and what it carries;
* the state topic: retained, JSON, mode, solver, uptime, host metrics;
* Discovery: which entities were announced, which expected ones are **missing**,
  which were announced but are **not expected** (a stale retained config, e.g. the
  PWM numbers left behind after a mode change), and which differ from the payload
  this build would publish;
* what Home Assistant would make of it: the state value every entity's own
  ``value_template`` reads out of the retained state blob (``unknown`` when that path
  is absent or null; a template that transforms the value, such as PWM in %, is not
  rendered -- the raw value is what tells you the blob and the templates agree);
* the SMART inbox topics and the age of each sample against the estimator's
  ``smart_max_age_s``;
* retained payloads left on ``{node_id}/cmd/#`` (a broker redelivers those on every
  reconnect; section 7).

Exit code: ``0`` everything expected is there, ``1`` something expected is missing or
wrong, ``2`` it could not run at all (bad config, MQTT disabled, no ``paho-mqtt``,
broker unreachable). ``--strict`` also fails on the warnings.

**It publishes nothing** unless ``--send`` is given, and even then it names the exact
topic and payload and waits for a typed ``yes``. It never prints the broker
credentials.
"""

from __future__ import annotations

import argparse
import contextlib
import json
import os
import re
import sys
import threading
import time
from collections.abc import Callable, Iterable, Mapping, Sequence
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, TextIO

from aqua_bridge.config import AppConfig, ConfigError, load_config
from aqua_bridge.control.intents import ControlMode
from aqua_bridge.model import MpcConfig
from aqua_bridge.publishers.inputs import DEFAULT_MAX_AGE_S, smart_topic_filter
from aqua_bridge.publishers.mqtt_ha import (
    MqttEntity,
    MqttSetupError,
    availability_topic,
    build_discovery_entities,
    command_topics,
    host_sensor_specs,
    state_topic,
    topic_matches,
    validate_mqtt_section,
)

__all__ = [
    "CONNECT_TIMEOUT_S",
    "DEFAULT_CONFIG",
    "DEFAULT_SEND_WAIT_S",
    "DEFAULT_WAIT_S",
    "Message",
    "Report",
    "build_parser",
    "build_report",
    "collect_messages",
    "command_topic_for",
    "confirm_send",
    "discovery_filter",
    "entity_state",
    "expected_entities",
    "latest_by_topic",
    "main",
    "resolve_path",
    "template_paths",
]

#: Where ``install-pi.sh`` puts the daemon's config (PROJECT.md section 10).
DEFAULT_CONFIG = Path("/etc/aqua-bridge/config.yaml")
#: ``--wait`` default, seconds: retained messages arrive at once, so this only has to
#: cover one state tick (``mpc.dt``, 2--5 s) plus the broker's round trip.
DEFAULT_WAIT_S = 20.0
#: ``--send-wait`` default, seconds: how long to re-read the state topic after a sent
#: command before printing what changed.
DEFAULT_SEND_WAIT_S = 10.0
#: How long ``--send`` waits for the broker's CONNACK before giving up, seconds.
CONNECT_TIMEOUT_S = 10.0


# ---------------------------------------------------------------------------
# Messages
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class Message:
    """One message seen on the broker."""

    topic: str
    payload: bytes
    retain: bool = False

    @property
    def text(self) -> str:
        return self.payload.decode("utf-8", errors="replace")

    def json(self) -> tuple[bool, Any]:
        """``(parsed_ok, value)`` -- never raises."""
        try:
            return True, json.loads(self.payload.decode("utf-8"))
        except (UnicodeDecodeError, ValueError):
            return False, None


def latest_by_topic(messages: Iterable[Message]) -> dict[str, Message]:
    """The last message seen per topic (a retained one, then any live update)."""
    out: dict[str, Message] = {}
    for message in messages:
        out[message.topic] = message
    return out


def discovery_filter(discovery_prefix: str, node_id: str) -> str:
    return f"{discovery_prefix}/+/{node_id}/+/config"


# ---------------------------------------------------------------------------
# What the daemon should be publishing
# ---------------------------------------------------------------------------


def expected_entities(
    cfg: MpcConfig, *, node_id: str, discovery_prefix: str, control_mode: ControlMode
) -> dict[str, MqttEntity]:
    """``config_topic -> entity`` for the control mode the daemon reports."""
    return {
        entity.config_topic: entity
        for entity in build_discovery_entities(
            cfg, node_id=node_id, discovery_prefix=discovery_prefix, control_mode=control_mode
        )
    }


_TEMPLATE_PATH = re.compile(r"value_json((?:\.[A-Za-z_][A-Za-z0-9_]*)+)")


def template_paths(value_template: str) -> list[str]:
    """Every ``value_json.a.b.c`` path a Discovery template reads, without the prefix."""
    return sorted({match.group(1)[1:] for match in _TEMPLATE_PATH.finditer(value_template)})


def resolve_path(blob: Any, path: str) -> tuple[str, Any]:
    """``("ok", value)`` | ``("null", None)`` | ``("missing", None)`` for one dotted path."""
    node: Any = blob
    for part in path.split("."):
        if not isinstance(node, Mapping) or part not in node:
            return "missing", None
        node = node[part]
    if node is None:
        return "null", None
    return "ok", node


def entity_id(entity: MqttEntity) -> str:
    """The entity id Home Assistant derives from the Discovery ``object_id``."""
    slug = re.sub(r"[^a-z0-9_]+", "_", str(entity.payload["object_id"]).lower())
    return f"{entity.component}.{slug}"


def entity_state(entity: MqttEntity, state: Any) -> tuple[str, list[str]]:
    """``(what Home Assistant would show, template paths absent from the state blob)``.

    Both templates count for the missing paths: the value and, where an entity has
    one, the JSON attributes (the ``device_problem`` sensor's detail, item 83).
    """
    missing: list[str] = []
    values: list[str] = []
    for path in template_paths(entity.payload.get("value_template", "")):
        status, value = resolve_path(state, path)
        if status == "missing":
            missing.append(path)
        elif status == "null":
            values.append("null")
        else:
            values.append(json.dumps(value, ensure_ascii=False))
    for path in template_paths(entity.payload.get("json_attributes_template", "")):
        if resolve_path(state, path)[0] == "missing":
            missing.append(path)
    if missing or not values:
        return "unknown", missing
    unit = entity.payload.get("unit_of_measurement")
    shown = " ".join(values)
    return f"{shown} {unit}" if unit else shown, missing


# ---------------------------------------------------------------------------
# The report
# ---------------------------------------------------------------------------


@dataclass
class Report:
    """Checklist lines plus the problems and warnings that decide the exit code."""

    lines: list[str] = field(default_factory=list)
    problems: list[str] = field(default_factory=list)
    warnings: list[str] = field(default_factory=list)

    def head(self, text: str) -> None:
        if self.lines:
            self.lines.append("")
        self.lines.append(text)

    def note(self, text: str) -> None:
        self.lines.append(f"         {text}")

    def ok(self, text: str) -> None:
        self.lines.append(f"  [ok]   {text}")

    def warn(self, text: str) -> None:
        self.warnings.append(text)
        self.lines.append(f"  [warn] {text}")

    def fail(self, text: str) -> None:
        self.problems.append(text)
        self.lines.append(f"  [FAIL] {text}")

    def text(self) -> str:
        return "\n".join(self.lines)


def _payload_diff(announced: Mapping[str, Any], expected: Mapping[str, Any]) -> list[str]:
    keys = set(announced) | set(expected)
    return sorted(k for k in keys if announced.get(k) != expected.get(k))


def _mode_of(state: Any) -> ControlMode | None:
    if not isinstance(state, Mapping):
        return None
    try:
        return ControlMode(state.get("mode"))
    except ValueError:
        return None


def _check_availability(report: Report, seen: Mapping[str, Message], node_id: str) -> None:
    topic = availability_topic(node_id)
    report.head(f"availability  {topic}")
    message = seen.get(topic)
    if message is None:
        report.fail(
            f"nothing on {topic}: the daemon has never connected to this broker "
            "(every entity is 'unavailable' in Home Assistant)"
        )
        return
    retained = "retained" if message.retain else "live"
    if message.text == "online":
        report.ok(f'"online" ({retained})')
    elif message.text == "offline":
        report.fail(
            f'"offline" ({retained}): the daemon is stopped, or its last will fired '
            "-- every entity is 'unavailable' in Home Assistant"
        )
    else:
        report.fail(f"unexpected payload {message.text!r} (expected 'online' or 'offline')")


def _check_state(report: Report, seen: Mapping[str, Message], node_id: str) -> Any:
    topic = state_topic(node_id)
    report.head(f"state  {topic}")
    message = seen.get(topic)
    if message is None:
        report.fail(f"nothing on {topic}: no entity has a value")
        return None
    parsed, state = message.json()
    if not parsed or not isinstance(state, Mapping):
        report.fail(f"the payload on {topic} is not a JSON object")
        return None
    if not message.retain:
        report.warn(
            f"{topic} arrived live but not retained: Home Assistant would show nothing "
            "until the next tick after a restart"
        )
    mode = _mode_of(state)
    health = state.get("health") if isinstance(state.get("health"), Mapping) else {}
    uptime = health.get("uptime_s")
    uptime_text = f"{uptime:.0f}s" if isinstance(uptime, int | float) else str(uptime)
    report.ok(
        f"retained JSON: mode={state.get('mode')} solver={health.get('solver')} "
        f"uptime={uptime_text} version={health.get('version')!r}"
    )
    if mode is None:
        report.fail(f"'mode' is {state.get('mode')!r}, not one of {[m.value for m in ControlMode]}")
    if state.get("obs") is None:
        report.warn("'obs' is null: the daemon has not completed a tick yet")
    host = state.get("host")
    if not isinstance(host, Mapping):
        report.fail("no 'host' key: the host sensors have no value")
    else:
        names = [name for name, *_ in host_sensor_specs()]
        readable = [name for name in names if host.get(name) is not None]
        line = f"host metrics: {len(readable)} of {len(names)} readable"
        if readable:
            report.ok(line)
        else:
            report.warn(f"{line} (every host sensor is unknown)")
    device_health = health.get("device_health") if isinstance(health, Mapping) else None
    if isinstance(device_health, Mapping):
        if device_health.get("ok"):
            report.ok("device health ok (the 'Controller problem' binary sensor is off)")
        else:
            report.warn(f"device health problems: {device_health.get('problems')}")
    return state


def _check_discovery(
    report: Report,
    seen: Mapping[str, Message],
    *,
    cfg: MpcConfig,
    node_id: str,
    discovery_prefix: str,
    mode: ControlMode,
    state: Any,
    verbose: bool,
) -> dict[str, MqttEntity]:
    wanted = expected_entities(
        cfg, node_id=node_id, discovery_prefix=discovery_prefix, control_mode=mode
    )
    announced = {
        topic: message
        for topic, message in seen.items()
        if topic_matches(discovery_filter(discovery_prefix, node_id), topic) and message.payload
    }
    report.head(f"discovery  {discovery_filter(discovery_prefix, node_id)}")
    report.note(f"expected for control mode {mode.value}: {len(wanted)} entities")
    present = sorted(set(wanted) & set(announced))
    report.ok(f"{len(present)} of {len(wanted)} expected entities announced")
    for topic in sorted(set(wanted) - set(announced)):
        report.fail(f"missing: {topic}")
    for topic in sorted(set(announced) - set(wanted)):
        report.fail(
            f"announced but not expected: {topic} "
            "(a stale retained config from another mode or an older config)"
        )
    for topic in present:
        parsed, payload = announced[topic].json()
        if not parsed or not isinstance(payload, Mapping):
            report.fail(f"the config on {topic} is not a JSON object")
            continue
        if not announced[topic].retain:
            report.warn(f"{topic} is not retained: Home Assistant loses it on a restart")
        diff = _payload_diff(payload, wanted[topic].payload)
        if diff:
            report.fail(
                f"the config on {topic} differs from this build in {diff} "
                "(the running daemon is another version or another config)"
            )
    report.head("home assistant would show")
    report.note(
        "the state value each entity's template reads; a template that transforms it "
        "(PWM in %, the problem sensor's ON/OFF) is not rendered here"
    )
    unknown: list[str] = []
    for topic in present:
        entity = wanted[topic]
        shown, missing = entity_state(entity, state)
        line = f"{entity_id(entity):<44} {shown}"
        command = entity.payload.get("command_topic")
        if command:
            line = f"{line}   <- {command}"
        if missing:
            unknown.append(f"{entity_id(entity)}: {', '.join(missing)} not in the state blob")
        if verbose:
            report.note(line)
    if not unknown:
        report.ok(f"every one of the {len(present)} announced entities has a value")
    else:
        report.warn(f"{len(unknown)} entities would read 'unknown':")
        for line in unknown:
            report.note(line)
    if not verbose:
        report.note("--verbose prints every entity with its value")
    return wanted


def _check_smart(
    report: Report, seen: Mapping[str, Message], *, cfg: MpcConfig, node_id: str, now: float
) -> None:
    topic_filter = smart_topic_filter(node_id)
    report.head(f"smart inbox  {topic_filter}")
    topics = sorted(t for t in seen if topic_matches(topic_filter, t))
    if not topics and cfg.topology is None:
        report.note("legacy config: no SMART path (the inbox only feeds the DAS estimator)")
        return
    if not topics:
        report.warn(
            "no SMART readings on the broker (tools/smart_agent.py on the PC publishes them; "
            "the estimator works without them)"
        )
        return
    # A legacy config has no estimator section; the inbox's own default applies then.
    max_age = DEFAULT_MAX_AGE_S if cfg.estimator is None else cfg.estimator.smart_max_age_s
    for topic in topics:
        parsed, payload = seen[topic].json()
        if not parsed or not isinstance(payload, Mapping):
            report.fail(f"the payload on {topic} is not a JSON object")
            continue
        serial = payload.get("serial")
        ts_wall = payload.get("ts_wall")
        age = None if not isinstance(ts_wall, int | float) else now - float(ts_wall)
        age_text = "unknown age" if age is None else f"{age:.0f}s old"
        line = f"{serial}: {payload.get('temp_c')} degC, {age_text} ({payload.get('model')})"
        if age is not None and age > max_age:
            report.warn(f"{line} -- older than the {max_age:g}s the estimator accepts")
        else:
            report.ok(line)


def _check_retained_commands(report: Report, seen: Mapping[str, Message], node_id: str) -> None:
    prefix = f"{node_id}/cmd/"
    report.head(f"commands  {prefix}#")
    retained = sorted(t for t, m in seen.items() if t.startswith(prefix) and m.retain and m.payload)
    if not retained:
        report.ok("no retained command on the broker")
        return
    for topic in retained:
        report.warn(
            f"retained payload on {topic}: a broker redelivers it on every reconnect "
            "(section 7; a retained experiment start is ignored, the rest is not)"
        )


def build_report(
    app: AppConfig,
    messages: Sequence[Message],
    *,
    node_id: str,
    discovery_prefix: str,
    verbose: bool = False,
    now: Callable[[], float] = time.time,
) -> Report:
    """The whole checklist from what was seen on the broker. Pure: no I/O."""
    report = Report()
    seen = latest_by_topic(messages)
    report.head(f"listened: {len(messages)} message(s) on {len(seen)} topic(s)")
    _check_availability(report, seen, node_id)
    state = _check_state(report, seen, node_id)
    mode = _mode_of(state)
    if mode is None:
        report.note("no control mode on the state topic: checking the 'auto' entity set")
        mode = ControlMode.AUTO
    _check_discovery(
        report,
        seen,
        cfg=app.mpc,
        node_id=node_id,
        discovery_prefix=discovery_prefix,
        mode=mode,
        state=state,
        verbose=verbose,
    )
    _check_smart(report, seen, cfg=app.mpc, node_id=node_id, now=now())
    _check_retained_commands(report, seen, node_id)
    return report


# ---------------------------------------------------------------------------
# The broker (paho)
# ---------------------------------------------------------------------------


def _build_client(
    *,
    client_id: str,
    username: str,
    password: str,
    filters: Sequence[str] = (),
    on_message: Callable[[Message], None] | None = None,
) -> tuple[Any, list[Any], threading.Event]:
    """``(client, connect_reason_codes, answered)``: a paho 2.x client that subscribes
    to ``filters`` on connect. No last will, no retained anything -- this tool is a
    listener."""
    import paho.mqtt.client as mqtt

    reason_codes: list[Any] = []
    answered = threading.Event()

    def _on_connect(client, userdata, flags, reason_code, properties=None) -> None:
        reason_codes.append(reason_code)
        answered.set()
        for topic_filter in filters:
            client.subscribe(topic_filter, qos=1)

    def _on_message(client, userdata, msg) -> None:
        if on_message is not None:
            on_message(Message(msg.topic, bytes(msg.payload), bool(msg.retain)))

    client = mqtt.Client(callback_api_version=mqtt.CallbackAPIVersion.VERSION2, client_id=client_id)
    if username:
        client.username_pw_set(username, password or None)
    client.on_connect = _on_connect
    client.on_message = _on_message
    return client, reason_codes, answered


def _refused(reason_codes: Sequence[Any]) -> str | None:
    """The broker's refusal, or ``None`` when it accepted the connection."""
    if not reason_codes:
        return "the broker never answered CONNECT (wrong port, or a TLS-only listener)"
    code = reason_codes[-1]
    if getattr(code, "is_failure", False) or (isinstance(code, int) and code != 0):
        return f"the broker refused the connection: {code}"
    return None


def collect_messages(
    *,
    host: str,
    port: int,
    username: str,
    password: str,
    filters: Sequence[str],
    wait_s: float,
    client_id: str,
    sleep: Callable[[float], None] = time.sleep,
) -> list[Message]:
    """Subscribe to ``filters`` and collect for ``wait_s`` seconds. Publishes nothing."""
    messages: list[Message] = []
    client, reason_codes, _answered = _build_client(
        client_id=client_id,
        username=username,
        password=password,
        filters=filters,
        on_message=messages.append,
    )
    client.connect(host, port, keepalive=60)
    client.loop_start()
    try:
        sleep(wait_s)
    finally:
        client.loop_stop()
        # Nothing left to do about a failed disconnect; the broker times the session out.
        with contextlib.suppress(Exception):
            client.disconnect()
    refused = _refused(reason_codes)
    if refused is not None:
        raise ConnectionError(refused)
    return messages


def publish_command(
    *,
    host: str,
    port: int,
    username: str,
    password: str,
    topic: str,
    payload: str,
    client_id: str,
) -> None:
    """Publish exactly one command, qos 0, **not** retained (section 7)."""
    client, reason_codes, answered = _build_client(
        client_id=client_id, username=username, password=password
    )
    client.connect(host, port, keepalive=60)
    client.loop_start()
    try:
        answered.wait(CONNECT_TIMEOUT_S)
        refused = _refused(reason_codes)
        if refused is not None:
            raise ConnectionError(refused)
        info = client.publish(topic, payload=payload, qos=0, retain=False)
        info.wait_for_publish(timeout=10.0)
    finally:
        client.loop_stop()
        with contextlib.suppress(Exception):
            client.disconnect()


# ---------------------------------------------------------------------------
# --send
# ---------------------------------------------------------------------------


def command_topic_for(node_id: str, cfg: MpcConfig, given: str) -> str | None:
    """The full command topic for ``given`` (a full topic or a ``cmd/...`` tail), or
    ``None`` when it is not a topic this daemon subscribes to."""
    topics = set(command_topics(node_id, cfg).values())
    for candidate in (given, f"{node_id}/{given}", f"{node_id}/cmd/{given}"):
        if candidate in topics:
            return candidate
    return None


def confirm_send(topic: str, payload: str, *, out: TextIO, stdin: TextIO) -> bool:
    """Name exactly what is about to be published and require a typed ``yes``."""
    print("", file=out)
    print("--send: about to publish ONE command to the live broker", file=out)
    print(f"    topic:   {topic}", file=out)
    print(f"    payload: {payload}", file=out)
    print("    qos 0, not retained", file=out)
    print(
        "The daemon acts on it immediately and Home Assistant will show the result.",
        file=out,
    )
    print('Type "yes" to send, anything else to abort: ', end="", file=out)
    out.flush()
    answer = stdin.readline().strip().lower()
    return answer == "yes"


def _print_state_after(report_out: TextIO, messages: Sequence[Message], node_id: str) -> None:
    message = latest_by_topic(messages).get(state_topic(node_id))
    if message is None:
        print("  no state blob arrived after the command", file=report_out)
        return
    parsed, state = message.json()
    if not parsed or not isinstance(state, Mapping):
        print("  the state blob after the command is not a JSON object", file=report_out)
        return
    for key in ("mode", "setpoints", "overrides", "limits", "bays"):
        if key in state:
            print(f"  {key}: {json.dumps(state[key], ensure_ascii=False)}", file=report_out)


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="ha_check",
        description="Read-only MQTT / Home Assistant check driven by the daemon's config.",
    )
    parser.add_argument(
        "--config",
        type=Path,
        default=DEFAULT_CONFIG,
        help=f"config.yaml whose mqtt: section names the broker (default: {DEFAULT_CONFIG})",
    )
    parser.add_argument(
        "--wait",
        type=float,
        default=DEFAULT_WAIT_S,
        metavar="S",
        help=f"seconds to listen before printing the checklist (default: {DEFAULT_WAIT_S:g})",
    )
    parser.add_argument(
        "--verbose",
        action="store_true",
        help="print every entity with its value, not only the "
        "ones Home Assistant would show as unknown",
    )
    parser.add_argument("--strict", action="store_true", help="also exit non-zero on the warnings")
    parser.add_argument(
        "--send",
        nargs=2,
        metavar=("TOPIC", "PAYLOAD"),
        default=None,
        help="after the checklist, publish ONE command (a full topic or a 'cmd/...' tail, "
        "e.g. cmd/setpoint/coolant 31.5) after naming it and reading a typed 'yes'",
    )
    parser.add_argument(
        "--send-wait",
        type=float,
        default=DEFAULT_SEND_WAIT_S,
        metavar="S",
        help=f"seconds to re-read the state topic after --send (default: {DEFAULT_SEND_WAIT_S:g})",
    )
    return parser


def main(
    argv: list[str] | None = None,
    *,
    out: TextIO | None = None,
    stdin: TextIO | None = None,
    collect: Callable[..., list[Message]] | None = None,
    publish: Callable[..., None] | None = None,
) -> int:
    args = build_parser().parse_args(argv)
    out = sys.stdout if out is None else out
    stdin = sys.stdin if stdin is None else stdin
    collect = collect_messages if collect is None else collect
    publish = publish_command if publish is None else publish

    try:
        app = load_config(args.config)
    except ConfigError as exc:
        print(f"config: {exc}", file=out)
        return 2
    try:
        mqtt_cfg = validate_mqtt_section(app.section("mqtt"))
    except MqttSetupError as exc:
        print(f"config: {exc}", file=out)
        return 2
    if not mqtt_cfg["enabled"]:
        print("config: mqtt.enabled is false -- this daemon publishes nothing", file=out)
        return 2

    node_id = mqtt_cfg["node_id"]
    discovery_prefix = mqtt_cfg["discovery_prefix"]
    # Never the daemon's own client id ("aqua-bridge-{node_id}"): a duplicate id makes
    # the broker drop the other session, i.e. this check would knock the daemon off.
    client_id = f"aqua-bridge-ha-check-{os.getpid()}"
    filters = [discovery_filter(discovery_prefix, node_id), f"{node_id}/#"]

    print(f"broker:    {mqtt_cfg['host']}:{mqtt_cfg['port']}", file=out)
    print(
        f"           username {'set' if mqtt_cfg['username'] else 'not set'} "
        f"(the password is never printed)",
        file=out,
    )
    print(f"node_id:   {node_id}", file=out)
    print(f"discovery: {discovery_prefix}", file=out)
    print(f"listening: {args.wait:g}s on {', '.join(filters)}", file=out)

    kwargs = {
        "host": mqtt_cfg["host"],
        "port": mqtt_cfg["port"],
        "username": mqtt_cfg["username"],
        "password": mqtt_cfg["password"],
        "client_id": client_id,
    }
    try:
        messages = collect(filters=filters, wait_s=args.wait, **kwargs)
    except ImportError:
        print("paho-mqtt is not installed (pip install 'aqua-bridge[mqtt]')", file=out)
        return 2
    except Exception as exc:  # refused, DNS, bad credentials, TLS ...
        print(f"broker: {type(exc).__name__}: {exc}", file=out)
        return 2

    report = build_report(
        app,
        messages,
        node_id=node_id,
        discovery_prefix=discovery_prefix,
        verbose=args.verbose,
    )
    print(report.text(), file=out)
    print("", file=out)
    print(f"result: {len(report.problems)} problem(s), {len(report.warnings)} warning(s)", file=out)
    status = 1 if report.problems or (args.strict and report.warnings) else 0

    if args.send is not None:
        given, payload = args.send
        topic = command_topic_for(node_id, app.mpc, given)
        if topic is None:
            print("", file=out)
            print(f"--send: {given!r} is not a command topic this daemon subscribes to", file=out)
            print(f"        (see {node_id}/cmd/... in PROJECT.md section 7)", file=out)
            return 2
        if not confirm_send(topic, payload, out=out, stdin=stdin):
            print("aborted: nothing was published", file=out)
            return status
        try:
            publish(topic=topic, payload=payload, **kwargs)
        except Exception as exc:
            print(f"broker: {type(exc).__name__}: {exc}", file=out)
            return 2
        print(f"published {payload!r} to {topic}", file=out)
        try:
            after = collect(filters=[state_topic(node_id)], wait_s=args.send_wait, **kwargs)
        except Exception as exc:
            print(f"broker: {type(exc).__name__}: {exc}", file=out)
            return 2
        print(f"state after {args.send_wait:g}s:", file=out)
        _print_state_after(out, after, node_id)
    return status


if __name__ == "__main__":
    sys.exit(main())
