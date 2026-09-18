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


def _das_healthy_state(cfg: MpcConfig, blob: dict[str, Any]) -> dict[str, Any]:
    """The state blob of a healthy DAS tick with the shipped ``model_shadow: false``:
    obs and cmd filled, ``cmd.diagnostics`` carrying zones, bays, estimates and noise
    but **no** ``thermal`` branch -- control/mpc.py only adds that one when the shadow
    model runs (config.example-das.yaml ships it off)."""
    bays = cfg.topology.bays
    return {
        **blob,
        "obs": {
            "temps": dict.fromkeys(cfg.temps, 30.0),
            "rpm": dict.fromkeys(cfg.channels, 900.0),
        },
        "cmd": {
            "pwm": dict.fromkeys(cfg.channels, 0.4),
            "diagnostics": {
                "zones": {zone: {"policy": "quiet"} for zone in cfg.topology.zones},
                "bays": {bay: {"occupancy": "occupied"} for bay in bays},
                "estimates": {
                    bay: {"t_c": 38.0, "limit_margin_c": 6.0, "sigma_c": 1.0} for bay in bays
                },
                "noise": {"db_index": 31.0},
            },
        },
    }


def test_a_das_without_the_shadow_model_is_not_unknown_anywhere(
    das_example_cfg: MpcConfig,
) -> None:
    """`model_shadow: false` is what config.example-das.yaml ships, so a healthy live
    DAS has no `cmd.diagnostics.thermal` at all -- and `model_status`'s template says to
    read `off` then (section 7). A path with a `| default(...)` behind it is not an
    entity reading 'unknown', or `--strict` would fail a healthy deployment."""
    messages = _daemon_messages(das_example_cfg, Supervisor(das_example_cfg))
    topic = f"{NODE_ID}/state"
    published = json.loads(next(m for m in messages if m.topic == topic).payload)
    blob = _das_healthy_state(das_example_cfg, published)
    patched = [
        *_without(messages, topic),
        ha.Message(topic, json.dumps(blob).encode(), True),
        ha.Message(
            f"{NODE_ID}/in/smart/S1",
            json.dumps(
                {"serial": "S1", "model": "m", "temp_c": 38.0, "ts_wall": 1_700_000_000.0}
            ).encode(),
            True,
        ),
    ]
    report = _report(das_example_cfg, patched)
    assert report.problems == []
    assert report.warnings == []
    text = report.text()
    assert "cmd.diagnostics.thermal.status -> off" in text
    assert "read their template's | default(...)" in text


def test_a_template_default_is_read_instead_of_a_missing_path(das_example_cfg: MpcConfig) -> None:
    """The unit behind it: `entity_state` renders the Jinja fallback rather than calling
    the entity unknown, and says which path it fell back for."""
    entities = ha.expected_entities(
        das_example_cfg,
        node_id=NODE_ID,
        discovery_prefix=PREFIX,
        control_mode=ControlMode.AUTO,
    )
    model_status = next(e for e in entities.values() if e.object_id == "model_status")
    blob = {"cmd": {"diagnostics": {"noise": {"db_index": 30.0}}}}
    assert ha.entity_state(model_status, blob) == (
        "off",
        [],
        ["cmd.diagnostics.thermal.status -> off"],
    )
    rpm = next(e for e in entities.values() if e.object_id.startswith("rpm_"))
    shown, missing, defaulted = ha.entity_state(rpm, blob)
    assert shown == "unknown" and missing and defaulted == []


def test_aquabus_problem_state_is_a_scalar_never_a_serial_number(
    das_example_cfg: MpcConfig,
) -> None:
    """Item 129's review: ``entity_state`` renders whatever a template's path
    resolves to with ``json.dumps`` -- and this daemon's hard constraints treat a
    device serial number as private data that ends up in issues and commit
    messages (module docstring). The template must resolve to the daemon's own
    ``aquabus_lost`` scalar, never to the per-controller ``devices`` list (which
    carries ``serial``), so a rendered checklist line can never contain one."""
    entities = ha.expected_entities(
        das_example_cfg,
        node_id=NODE_ID,
        discovery_prefix=PREFIX,
        control_mode=ControlMode.AUTO,
    )
    aquabus_problem = next(e for e in entities.values() if e.object_id == "aquabus_problem")
    blob = {
        "device_health": {
            "aquabus_lost": True,
            "devices": [
                {
                    "label": "aquaero",
                    "serial": "12345-54321",
                    "aquabus": {"state": "lost", "lost": True, "bound": True},
                }
            ],
        }
    }
    shown, missing, defaulted = ha.entity_state(aquabus_problem, blob)
    assert missing == [] and defaulted == []
    # ha_check does not evaluate the Jinja if/else -- it shows the raw resolved
    # value of the one path the template reads (json.dumps'd), which is the point:
    # that value must be the scalar "aquabus_lost", not the "devices" list.
    assert shown == "true"
    assert "12345-54321" not in shown and "serial" not in shown


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


def test_template_path_defaults_reads_the_jinja_fallback_of_each_path() -> None:
    assert ha.template_path_defaults("{{ value_json.obs.temps.coolant }}") == {
        "obs.temps.coolant": None
    }
    assert ha.template_path_defaults(
        "{{ value_json.cmd.diagnostics.thermal.status | default('off') }}"
    ) == {"cmd.diagnostics.thermal.status": "'off'"}
    assert ha.template_path_defaults(
        "{{ 'OFF' if value_json.cmd.diagnostics.bays.b01.occupancy "
        "| default('unknown') == 'empty' else 'ON' }}"
    ) == {"cmd.diagnostics.bays.b01.occupancy": "'unknown'"}
    assert ha.template_path_defaults("{{ value_json.device_health | default({}) | tojson }}") == {
        "device_health": "{}"
    }


@pytest.mark.parametrize(
    ("literal", "shown"),
    [("'off'", "off"), ('"off"', "off"), ("None", "None"), ("true", "true"), ("{}", "{}")],
)
def test_render_default(literal: str, shown: str) -> None:
    assert ha.render_default(literal) == shown


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


def test_the_broker_host_is_printed_only_under_verbose(cfg: MpcConfig, config_file: Path) -> None:
    """The output of this tool is what gets pasted into an issue or a commit message;
    the owner's broker host lives in private.md, not in the repo."""
    broker = _Broker(_daemon_messages(cfg, _ticked_supervisor(cfg)))
    _code, quiet = _run(config_file, broker, "--wait", "0")
    assert "broker.example" not in quiet
    assert str(config_file) in quiet
    _code, loud = _run(config_file, broker, "--wait", "0", "--verbose")
    assert "broker.example:1883" in loud


@pytest.mark.parametrize("flag", ["--wait", "--send-wait"])
def test_a_negative_wait_is_an_argument_error_not_a_broker_error(
    config_file: Path, flag: str
) -> None:
    """time.sleep(-1) would otherwise surface through main's broad except as
    'broker: ValueError', blaming the broker for a typo."""
    broker = _Broker([])
    with pytest.raises(SystemExit) as excinfo:
        _run(config_file, broker, flag, "-1")
    assert excinfo.value.code == 2
    assert broker.collected == []


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


@pytest.mark.parametrize(
    ("given", "caution"),
    [
        ("cmd/mode", "changes the daemon's control mode until it is changed back"),
        ("cmd/pwm/radiator", "overrides the solver on that channel until it is cleared"),
        ("cmd/setpoint/coolant", None),
    ],
)
def test_send_says_what_outlives_the_message(
    cfg: MpcConfig, config_file: Path, given: str, caution: str | None
) -> None:
    """`cmd/mode manual` and a PWM override stand until something clears them: a
    declined or forgotten `cmd/mode auto` leaves the daemon out of auto, so the
    confirmation has to say so before the typed yes, not after."""
    broker = _Broker(_daemon_messages(cfg, _ticked_supervisor(cfg)))
    _code, text = _run(config_file, broker, "--wait", "0", "--send", given, "manual", stdin="no\n")
    if caution is None:
        assert "NOTE:" not in text
    else:
        assert caution in text
        assert "clears every override" in text


def test_the_subscriptions_are_the_daemons_topics_only(cfg: MpcConfig, config_file: Path) -> None:
    broker = _Broker(_daemon_messages(cfg, _ticked_supervisor(cfg)))
    _run(config_file, broker, "--wait", "0")
    assert broker.collected[0]["filters"] == [
        f"{PREFIX}/+/{NODE_ID}/+/config",
        f"{NODE_ID}/#",
    ]


# --- the broker layer, with a fake paho client -------------------------------------------


class _PahoMsg:
    def __init__(self, topic: str, payload: bytes, retain: bool) -> None:
        self.topic = topic
        self.payload = payload
        self.retain = retain


def _fake_paho(
    monkeypatch: pytest.MonkeyPatch,
    *,
    connack: Any = 0,
    suback: Any = 1,
    answer_connect: bool = True,
    deliver: tuple[tuple[str, bytes, bool], ...] = (),
) -> list[Any]:
    """Replace ``paho.mqtt.client.Client`` with one that answers CONNECT and SUBSCRIBE
    the way a broker would. Returns the list of clients the tool built."""
    mqtt = pytest.importorskip("paho.mqtt.client")
    made: list[Any] = []

    class _Info:
        def __init__(self) -> None:
            self.waited: list[float | None] = []

        def wait_for_publish(self, timeout: float | None = None) -> None:
            self.waited.append(timeout)

    class _Client:
        def __init__(self, *_args: Any, **kwargs: Any) -> None:
            self.client_id = kwargs.get("client_id")
            self.endpoint: tuple[Any, ...] = ()
            self.subscribed: list[tuple[str, int]] = []
            self.published: list[tuple[str, str, int, bool]] = []
            self.infos: list[_Info] = []
            self.loops: list[str] = []
            self.disconnected = 0
            self._mid = 0
            made.append(self)

        def username_pw_set(self, username: str, password: str | None = None) -> None:
            self.auth = (username, password)

        def connect(self, host: str, port: int, keepalive: int = 60) -> None:
            self.endpoint = (host, port, keepalive)

        def loop_start(self) -> None:
            self.loops.append("start")
            if not answer_connect:
                return
            self.on_connect(self, None, {}, connack)
            for topic, payload, retain in deliver:
                self.on_message(self, None, _PahoMsg(topic, payload, retain))

        def subscribe(self, topic_filter: str, qos: int = 0) -> tuple[int, int]:
            self._mid += 1
            self.subscribed.append((topic_filter, qos))
            if suback is not None:
                self.on_subscribe(self, None, self._mid, [suback])
            return 0, self._mid

        def publish(self, topic: str, payload: str = "", qos: int = 0, retain: bool = False):
            self.published.append((topic, payload, qos, retain))
            info = _Info()
            self.infos.append(info)
            return info

        def loop_stop(self) -> None:
            self.loops.append("stop")

        def disconnect(self) -> None:
            self.disconnected += 1

    monkeypatch.setattr(mqtt, "Client", _Client)
    return made


_COLLECT = {
    "host": "broker.example",
    "port": 1883,
    "username": "u",
    "password": "p",
    "filters": ["a/#"],
    "client_id": "ha-check-test",
}


def test_collect_messages_returns_what_the_broker_delivered(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    made = _fake_paho(monkeypatch, deliver=(("a/state", b"{}", True),))
    slept: list[float] = []
    messages = ha.collect_messages(**_COLLECT, wait_s=7.0, sleep=slept.append)
    assert [(m.topic, m.payload, m.retain) for m in messages] == [("a/state", b"{}", True)]
    assert slept == [7.0]
    client = made[0]
    assert client.client_id == "ha-check-test"
    assert client.endpoint == ("broker.example", 1883, ha.KEEPALIVE_S)
    assert client.subscribed == [("a/#", 1)]
    assert client.published == []  # read-only
    assert client.disconnected == 1


def test_collect_messages_does_not_charge_the_connect_round_trip_to_the_wait(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """A broker that never answers CONNECT is diagnosed as that, and at the connect
    budget -- not read out of an empty collection window as "wrong port"."""
    _fake_paho(monkeypatch, answer_connect=False)
    slept: list[float] = []
    with pytest.raises(ConnectionError, match="did not answer CONNECT within 0.01s"):
        ha.collect_messages(**_COLLECT, wait_s=900.0, sleep=slept.append, timeout_s=0.01)
    assert slept == []


def test_collect_messages_reports_a_refusal_before_the_collection_window(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """A bad password is known at CONNACK; waiting out --wait first only delays it."""
    _fake_paho(monkeypatch, connack=5)
    slept: list[float] = []
    with pytest.raises(ConnectionError, match="refused the connection: 5"):
        ha.collect_messages(**_COLLECT, wait_s=900.0, sleep=slept.append)
    assert slept == []


def test_a_refused_subscription_is_not_read_as_a_silent_daemon(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """Section 7 asks for an ACL granting *publish* on the daemon's topics. A
    publish-only account connects fine and is subscribed to nothing; without the SUBACK
    the tool would blame the daemon for the silence and report every entity missing."""
    _fake_paho(monkeypatch, suback=128)
    slept: list[float] = []
    with pytest.raises(ConnectionError, match="refused the subscription to a/# "):
        ha.collect_messages(**_COLLECT, wait_s=900.0, sleep=slept.append)
    assert slept == []


def test_publish_command_sends_one_unretained_message_on_the_connect_budget(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    made = _fake_paho(monkeypatch)
    ha.publish_command(
        host="broker.example",
        port=1883,
        username="u",
        password="p",
        topic="a/cmd/mode",
        payload="auto",
        client_id="ha-check-test",
    )
    client = made[0]
    assert client.published == [("a/cmd/mode", "auto", 0, False)]
    assert client.infos[0].waited == [ha.CONNECT_TIMEOUT_S]
    assert client.subscribed == []
    assert client.disconnected == 1


def test_publish_command_publishes_nothing_when_the_broker_refuses(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    made = _fake_paho(monkeypatch, connack=5)
    with pytest.raises(ConnectionError, match="refused the connection"):
        ha.publish_command(
            host="broker.example",
            port=1883,
            username="u",
            password="p",
            topic="a/cmd/mode",
            payload="auto",
            client_id="ha-check-test",
        )
    assert made[0].published == []


def test_the_tool_reads_the_broker_from_the_config_section(
    cfg: MpcConfig, config_file: Path
) -> None:
    broker = _Broker([])
    _run(config_file, broker, "--wait", "3")
    call = broker.collected[0]
    assert call["host"] == "broker.example" and call["port"] == 1883
    assert call["username"] == "bridge-user" and call["password"] == "s3cret-do-not-print"
    assert call["wait_s"] == 3.0
    assert load_config(config_file).section("mqtt")["node_id"] == NODE_ID
