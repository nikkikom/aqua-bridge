"""Tests for tools/ha_check.py, the live MQTT / Home Assistant check (§8 item 21).

No broker: the "broker view" every test starts from is exactly what the daemon
publishes, captured off a real :class:`~aqua_bridge.publishers.mqtt_ha.MqttClient`
with a fake paho underneath. So a clean run here means the tool agrees with the
publisher, and each failure case is one thing the live run has to catch.
"""

from __future__ import annotations

import importlib.util
import json
import sys
from io import StringIO
from pathlib import Path
from typing import Any

import pytest
import yaml

from aqua_bridge.config import AppConfig, load_config
from aqua_bridge.control.intents import ControlMode, SetMode
from aqua_bridge.control.supervisor import Supervisor
from aqua_bridge.model import MpcConfig
from aqua_bridge.publishers.mqtt_ha import host_sensor_specs
from test_mqtt_ha import _FakePaho

_TOOLS = Path(__file__).resolve().parent.parent / "tools"
NODE_ID = "attic-bridge"
PREFIX = "ha-test"


def _load_ha_check():
    spec = importlib.util.spec_from_file_location("ha_check", _TOOLS / "ha_check.py")
    assert spec is not None and spec.loader is not None
    module = sys.modules.get("ha_check")
    if module is None:
        module = importlib.util.module_from_spec(spec)
        sys.modules["ha_check"] = module
        spec.loader.exec_module(module)
    return module


ha = _load_ha_check()

HOST_METRICS = {name: 1.0 for name, *_ in host_sensor_specs()}


# --- a broker view built from what the daemon actually publishes -------------------------


def _ticked_supervisor(cfg: MpcConfig) -> Supervisor:
    """A supervisor that has seen two ticks, so the state blob has obs and cmd."""
    from aqua_bridge.__main__ import PlantIO, make_sim_plant
    from aqua_bridge.control.loop import Loop

    sup = Supervisor(cfg)
    io = PlantIO(make_sim_plant(cfg), cfg)
    loop = Loop(io, io, cfg, sup)
    loop.tick()
    loop.tick()
    return sup


def _daemon_messages(
    cfg: MpcConfig,
    sup: Supervisor,
    *,
    mode: ControlMode = ControlMode.AUTO,
    host: dict[str, Any] | None = None,
) -> list[Any]:
    """Every retained message the daemon leaves on the broker for one connection."""
    pytest.importorskip("paho.mqtt.client")
    from aqua_bridge.publishers.mqtt_ha import MqttClient

    client = MqttClient(
        node_id=NODE_ID, discovery_prefix=PREFIX, cfg=cfg, host="broker.example", port=1883
    )
    paho = _FakePaho()
    client.client = paho
    client._on_connect(paho, None, {}, 0)
    client.publish_discovery(control_mode=mode)
    client.publish_state(sup.snapshot().to_dict(), HOST_METRICS if host is None else host)
    return [
        ha.Message(topic, str(payload).encode(), True)
        for topic, payload, _qos, _retain in paho.published
        if payload != ""  # the deletions leave nothing on the broker
    ]


def _report(cfg: MpcConfig, messages: list[Any], **kwargs: Any) -> Any:
    return ha.build_report(
        AppConfig(mpc=cfg),
        messages,
        node_id=NODE_ID,
        discovery_prefix=PREFIX,
        now=lambda: 1_700_000_000.0,
        **kwargs,
    )


def _topic_of(messages: list[Any], needle: str) -> str:
    return next(m.topic for m in messages if needle in m.topic)


def _without(messages: list[Any], topic: str) -> list[Any]:
    return [m for m in messages if m.topic != topic]


# --- the clean run -----------------------------------------------------------------------


def test_a_healthy_broker_view_has_no_problems(cfg: MpcConfig) -> None:
    sup = _ticked_supervisor(cfg)
    report = _report(cfg, _daemon_messages(cfg, sup))
    assert report.problems == []
    text = report.text()
    assert '"online"' in text
    assert "retained JSON: mode=auto" in text
    assert f"host metrics: {len(HOST_METRICS)} of {len(HOST_METRICS)} readable" in text
    assert "announced entities has a value" in text


def test_every_announced_entity_resolves_to_a_value_after_a_tick(cfg: MpcConfig) -> None:
    """The "what Home Assistant would make of it" half: each entity's own value_template
    is resolved against the retained state blob the daemon published."""
    sup = _ticked_supervisor(cfg)
    report = _report(cfg, _daemon_messages(cfg, sup), verbose=True)
    text = report.text()
    assert report.warnings == []
    for ch in cfg.channels:
        assert f"sensor.{NODE_ID.replace('-', '_')}_rpm_{ch}" in text
    assert f"number.{NODE_ID.replace('-', '_')}_setpoint_coolant" in text
    assert f"{NODE_ID}/cmd/setpoint/coolant" in text  # the command topic is shown


def test_a_fresh_daemon_reads_unknown_but_is_not_a_problem(cfg: MpcConfig) -> None:
    """Before the first tick obs is null, so the temperature and RPM sensors would read
    'unknown' in Home Assistant: a warning, not a missing entity."""
    report = _report(cfg, _daemon_messages(cfg, Supervisor(cfg)))
    assert report.problems == []
    assert any("would read 'unknown'" in w for w in report.warnings)
    assert any("'obs' is null" in w for w in report.warnings)


def test_the_das_entity_set_is_complete(das_example_cfg: MpcConfig) -> None:
    sup = Supervisor(das_example_cfg)
    messages = _daemon_messages(das_example_cfg, sup)
    report = _report(das_example_cfg, messages)
    assert report.problems == []
    announced = [m.topic for m in messages if m.topic.startswith(f"{PREFIX}/")]
    for bay in das_example_cfg.topology.bays:
        assert f"{PREFIX}/sensor/{NODE_ID}/drive_temp_{bay}/config" in announced
        assert f"{PREFIX}/binary_sensor/{NODE_ID}/bay_occupied_{bay}/config" in announced
    for zone in das_example_cfg.topology.zones:
        assert f"{PREFIX}/sensor/{NODE_ID}/zone_status_{zone}/config" in announced
    for name in das_example_cfg.drive_classes:
        assert f"{PREFIX}/number/{NODE_ID}/limit_{name}/config" in announced


# --- what the live run has to catch ------------------------------------------------------


def test_a_missing_entity_is_a_problem(cfg: MpcConfig) -> None:
    messages = _daemon_messages(cfg, _ticked_supervisor(cfg))
    gone = _topic_of(messages, "/number/")
    report = _report(cfg, _without(messages, gone))
    assert report.problems == [f"missing: {gone}"]


def test_a_stale_retained_pwm_number_is_a_problem(cfg: MpcConfig) -> None:
    """Leaving manual deletes the PWM numbers with an empty retained payload; one left
    behind is an entity Home Assistant shows and nobody can use."""
    sup = _ticked_supervisor(cfg)
    auto = _daemon_messages(cfg, sup)
    sup.submit(SetMode(mode="manual"))
    manual = _daemon_messages(cfg, sup, mode=ControlMode.MANUAL)
    stale = next(m for m in manual if "pwm_cmd_" in m.topic)
    report = _report(cfg, [*auto, stale])
    assert len(report.problems) == 1
    assert "announced but not expected" in report.problems[0]
    assert stale.topic in report.problems[0]


def test_a_payload_from_another_build_is_a_problem(cfg: MpcConfig) -> None:
    messages = _daemon_messages(cfg, _ticked_supervisor(cfg))
    topic = _topic_of(messages, "/sensor/")
    changed = []
    for message in messages:
        if message.topic != topic:
            changed.append(message)
            continue
        payload = json.loads(message.payload)
        payload["unit_of_measurement"] = "K"
        changed.append(ha.Message(topic, json.dumps(payload).encode(), True))
    report = _report(cfg, changed)
    assert len(report.problems) == 1
    assert "unit_of_measurement" in report.problems[0]


def test_no_availability_and_offline_availability_are_problems(cfg: MpcConfig) -> None:
    messages = _daemon_messages(cfg, _ticked_supervisor(cfg))
    status = f"{NODE_ID}/status"
    assert "never connected" in _report(cfg, _without(messages, status)).problems[0]
    offline = [*_without(messages, status), ha.Message(status, b"offline", True)]
    assert "last will" in _report(cfg, offline).problems[0]


def test_no_state_topic_is_a_problem_and_the_auto_entity_set_is_checked(cfg: MpcConfig) -> None:
    messages = _without(_daemon_messages(cfg, _ticked_supervisor(cfg)), f"{NODE_ID}/state")
    report = _report(cfg, messages)
    assert any("no entity has a value" in p for p in report.problems)
    assert "checking the 'auto' entity set" in report.text()


def test_smart_readings_fresh_and_stale(das_example_cfg: MpcConfig) -> None:
    now = 1_700_000_000.0
    max_age = das_example_cfg.estimator.smart_max_age_s
    messages = _daemon_messages(das_example_cfg, Supervisor(das_example_cfg))
    for serial, ts in (("FRESH1", now - 10.0), ("OLD1", now - max_age - 60.0)):
        payload = {"serial": serial, "model": "m", "temp_c": 38.0, "ts_wall": ts}
        messages.append(
            ha.Message(f"{NODE_ID}/in/smart/{serial}", json.dumps(payload).encode(), True)
        )
    report = _report(das_example_cfg, messages)
    assert report.problems == []
    assert "FRESH1: 38.0 degC, 10s old" in report.text()
    assert any("OLD1" in w and f"{max_age:g}s the estimator accepts" in w for w in report.warnings)


def test_a_retained_command_on_the_broker_is_a_warning(cfg: MpcConfig) -> None:
    messages = _daemon_messages(cfg, _ticked_supervisor(cfg))
    messages.append(ha.Message(f"{NODE_ID}/cmd/mode", b"manual", True))
    report = _report(cfg, messages)
    assert report.problems == []
    assert any("redelivers it on every reconnect" in w for w in report.warnings)


# --- the pure helpers --------------------------------------------------------------------


def test_template_paths_reads_every_path_a_template_names() -> None:
    assert ha.template_paths("{{ value_json.obs.temps.coolant }}") == ["obs.temps.coolant"]
    assert ha.template_paths(
        "{{ 'OFF' if value_json.health.device_health.ok | default(true) else 'ON' }}"
    ) == ["health.device_health.ok"]
    assert ha.template_paths("{{ (value_json.cmd.pwm.x | float * 100) | round(0) }}") == [
        "cmd.pwm.x"
    ]
    assert ha.template_paths("nothing here") == []


@pytest.mark.parametrize(
    ("path", "expected"),
    [
        ("a.b", ("ok", 1)),
        ("a.c", ("null", None)),
        ("a.d", ("missing", None)),
        ("z", ("missing", None)),
    ],
)
def test_resolve_path(path: str, expected: tuple[str, Any]) -> None:
    assert ha.resolve_path({"a": {"b": 1, "c": None}}, path) == expected


def test_command_topic_for_accepts_a_tail_or_a_full_topic(cfg: MpcConfig) -> None:
    assert ha.command_topic_for(NODE_ID, cfg, "cmd/mode") == f"{NODE_ID}/cmd/mode"
    assert ha.command_topic_for(NODE_ID, cfg, "mode") == f"{NODE_ID}/cmd/mode"
    assert ha.command_topic_for(NODE_ID, cfg, f"{NODE_ID}/cmd/mode") == f"{NODE_ID}/cmd/mode"
    assert ha.command_topic_for(NODE_ID, cfg, "cmd/nonsense") is None
    assert ha.command_topic_for(NODE_ID, cfg, f"{NODE_ID}/state") is None


# --- the CLI -----------------------------------------------------------------------------


@pytest.fixture
def config_file(tmp_path: Path, example_config_path: Path) -> Path:
    data = yaml.safe_load(example_config_path.read_text(encoding="utf-8"))
    data["mqtt"] = {
        "enabled": True,
        "host": "broker.example",
        "port": 1883,
        "username": "bridge-user",
        "password": "s3cret-do-not-print",
        "discovery_prefix": PREFIX,
        "node_id": NODE_ID,
    }
    path = tmp_path / "config.yaml"
    path.write_text(yaml.safe_dump(data), encoding="utf-8")
    return path


class _Broker:
    """Stands in for the broker: hands the tool a message list, records publishes."""

    def __init__(self, messages: list[Any]) -> None:
        self.messages = messages
        self.published: list[dict[str, Any]] = []
        self.collected: list[dict[str, Any]] = []

    def collect(self, **kwargs: Any) -> list[Any]:
        self.collected.append(kwargs)
        return list(self.messages)

    def publish(self, **kwargs: Any) -> None:
        self.published.append(kwargs)


def _run(config_file: Path, broker: _Broker, *args: str, stdin: str = "") -> tuple[int, str]:
    out = StringIO()
    code = ha.main(
        ["--config", str(config_file), *args],
        out=out,
        stdin=StringIO(stdin),
        collect=broker.collect,
        publish=broker.publish,
    )
    return code, out.getvalue()


def test_main_exits_zero_on_a_healthy_broker_and_publishes_nothing(
    cfg: MpcConfig, config_file: Path
) -> None:
    broker = _Broker(_daemon_messages(cfg, _ticked_supervisor(cfg)))
    code, text = _run(config_file, broker, "--wait", "0")
    assert code == 0, text
    assert broker.published == []
    assert "0 problem(s)" in text


def test_main_exits_one_when_an_entity_is_missing(cfg: MpcConfig, config_file: Path) -> None:
    messages = _daemon_messages(cfg, _ticked_supervisor(cfg))
    broker = _Broker(_without(messages, _topic_of(messages, "/number/")))
    code, text = _run(config_file, broker, "--wait", "0")
    assert code == 1
    assert "missing:" in text


def test_strict_also_fails_on_warnings(cfg: MpcConfig, config_file: Path) -> None:
    broker = _Broker(_daemon_messages(cfg, Supervisor(cfg)))  # no tick: unknown values
    assert _run(config_file, broker, "--wait", "0")[0] == 0
    assert _run(config_file, broker, "--wait", "0", "--strict")[0] == 1


def test_the_credentials_are_never_printed(cfg: MpcConfig, config_file: Path) -> None:
    broker = _Broker(_daemon_messages(cfg, _ticked_supervisor(cfg)))
    _code, text = _run(config_file, broker, "--wait", "0")
    assert "s3cret-do-not-print" not in text
    assert "bridge-user" not in text
    assert "username set" in text


def test_it_never_uses_the_daemons_client_id(cfg: MpcConfig, config_file: Path) -> None:
    """A second session with the daemon's client id would make the broker drop the
    daemon -- a read-only check must never do that."""
    broker = _Broker(_daemon_messages(cfg, _ticked_supervisor(cfg)))
    _run(config_file, broker, "--wait", "0")
    client_id = broker.collected[0]["client_id"]
    assert client_id != f"aqua-bridge-{NODE_ID}"
    assert client_id.startswith("aqua-bridge-ha-check-")


def test_a_disabled_mqtt_section_exits_two(tmp_path: Path, example_config_path: Path) -> None:
    path = tmp_path / "config.yaml"
    path.write_text(example_config_path.read_text(encoding="utf-8"), encoding="utf-8")
    out = StringIO()
    code = ha.main(["--config", str(path), "--wait", "0"], out=out, stdin=StringIO())
    assert code == 2
    assert "mqtt.enabled is false" in out.getvalue()


def test_a_missing_config_exits_two(tmp_path: Path) -> None:
    out = StringIO()
    assert ha.main(["--config", str(tmp_path / "nope.yaml")], out=out, stdin=StringIO()) == 2
    assert "config:" in out.getvalue()


# --- --send ------------------------------------------------------------------------------


def test_send_names_the_command_and_does_nothing_without_a_typed_yes(
    cfg: MpcConfig, config_file: Path
) -> None:
    broker = _Broker(_daemon_messages(cfg, _ticked_supervisor(cfg)))
    code, text = _run(
        config_file, broker, "--wait", "0", "--send", "cmd/setpoint/coolant", "31.5", stdin="no\n"
    )
    assert broker.published == []
    assert f"topic:   {NODE_ID}/cmd/setpoint/coolant" in text
    assert "payload: 31.5" in text
    assert "qos 0, not retained" in text
    assert "aborted: nothing was published" in text
    assert code == 0


def test_send_publishes_exactly_one_command_after_a_typed_yes(
    cfg: MpcConfig, config_file: Path
) -> None:
    broker = _Broker(_daemon_messages(cfg, _ticked_supervisor(cfg)))
    code, text = _run(
        config_file,
        broker,
        "--wait",
        "0",
        "--send-wait",
        "0",
        "--send",
        "cmd/setpoint/coolant",
        "31.5",
        stdin="yes\n",
    )
    assert code == 0
    assert len(broker.published) == 1
    sent = broker.published[0]
    assert sent["topic"] == f"{NODE_ID}/cmd/setpoint/coolant"
    assert sent["payload"] == "31.5"
    assert "state after" in text
    assert broker.collected[-1]["filters"] == [f"{NODE_ID}/state"]


def test_send_refuses_a_topic_the_daemon_does_not_subscribe_to(
    cfg: MpcConfig, config_file: Path
) -> None:
    broker = _Broker(_daemon_messages(cfg, _ticked_supervisor(cfg)))
    code, text = _run(config_file, broker, "--wait", "0", "--send", "state", "x", stdin="yes\n")
    assert code == 2
    assert broker.published == []
    assert "is not a command topic" in text


def test_the_subscriptions_are_the_daemons_topics_only(cfg: MpcConfig, config_file: Path) -> None:
    broker = _Broker(_daemon_messages(cfg, _ticked_supervisor(cfg)))
    _run(config_file, broker, "--wait", "0")
    assert broker.collected[0]["filters"] == [
        f"{PREFIX}/+/{NODE_ID}/+/config",
        f"{NODE_ID}/#",
    ]


def test_the_tool_reads_the_broker_from_the_config_section(cfg: MpcConfig, config_file: Path) -> None:
    broker = _Broker([])
    _run(config_file, broker, "--wait", "3")
    call = broker.collected[0]
    assert call["host"] == "broker.example" and call["port"] == 1883
    assert call["username"] == "bridge-user" and call["password"] == "s3cret-do-not-print"
    assert call["wait_s"] == 3.0
    assert load_config(config_file).section("mqtt")["node_id"] == NODE_ID
