"""aqua_bridge.publishers.runtime: HTTP thread and MQTT per-tick publisher (sections 6, 7)."""

from __future__ import annotations

import json
import socket
import urllib.request
from typing import Any

import pytest

from aqua_bridge.config import AppConfig
from aqua_bridge.control.intents import ControlMode, SetMode
from aqua_bridge.control.supervisor import Supervisor
from aqua_bridge.model import MpcConfig
from aqua_bridge.publishers.runtime import HttpService, MqttService


def _can_bind_localhost() -> bool:
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.bind(("127.0.0.1", 0))
    except OSError:
        return False
    return True


needs_bind = pytest.mark.skipif(
    not _can_bind_localhost(), reason="binding a localhost TCP socket is denied here (sandbox)"
)


def _app(cfg: MpcConfig, **sections: dict[str, Any]) -> AppConfig:
    return AppConfig(mpc=cfg, **sections)


# --- HttpService -------------------------------------------------------------------------


@needs_bind
def test_http_service_serves_state_and_stops(cfg: MpcConfig):
    sup = Supervisor(cfg)
    service = HttpService(sup, _app(cfg, http={"bind": "127.0.0.1", "port": 0, "enabled": True}))
    assert service.start(timeout_s=10.0) is True
    try:
        assert service.running
        (host, port, *_rest) = service.addresses[0]
        with urllib.request.urlopen(f"http://{host}:{port}/api/health", timeout=5) as resp:
            body = json.loads(resp.read())
        assert body["solver"] == "fault"  # no tick yet
        assert body["mqtt_connected"] is None
    finally:
        service.stop()
    assert not service.running
    service.stop()  # idempotent


def test_http_service_start_failure_is_reported_not_raised(cfg: MpcConfig):
    async def boom(surface, app_cfg):
        raise OSError(98, "address in use")

    service = HttpService(Supervisor(cfg), _app(cfg), run=boom)
    assert service.start(timeout_s=5.0) is False
    assert service.error is not None and "address in use" in service.error
    assert not service.running
    service.stop()


# --- MqttService --------------------------------------------------------------------------


class FakeMqttClient:
    def __init__(self, **kwargs: Any) -> None:
        self.kwargs = kwargs
        self.connected = False
        self.calls: list[str] = []
        self.discovery_modes: list[ControlMode] = []
        self.states: list[tuple[dict, dict]] = []
        self.fail_publish = False

    def connect_async(self) -> None:
        self.calls.append("connect_async")

    def loop_start(self) -> None:
        self.calls.append("loop_start")

    def loop_stop(self) -> None:
        self.calls.append("loop_stop")

    def disconnect(self) -> None:
        self.calls.append("disconnect")

    def publish_discovery(self, *, control_mode: ControlMode) -> None:
        self.discovery_modes.append(control_mode)

    def publish_state(self, snapshot_dict, host) -> None:
        if self.fail_publish:
            raise ConnectionError("broker gone")
        self.states.append((dict(snapshot_dict), dict(host)))


class FakeClock:
    def __init__(self) -> None:
        self.t = 0.0

    def __call__(self) -> float:
        return self.t


@pytest.fixture
def mqtt(cfg: MpcConfig):
    sup = Supervisor(cfg)
    client = FakeMqttClient()
    clock = FakeClock()
    calls = {"hostinfo": 0}

    def hostinfo():
        calls["hostinfo"] += 1
        return {"cpu_temp_c": 40.0 + calls["hostinfo"]}

    service = MqttService(client, sup, host_interval_s=5.0, hostinfo=hostinfo, clock=clock)
    return service, client, sup, clock, calls


def test_nothing_is_published_while_disconnected(mqtt):
    service, client, *_ = mqtt
    service.on_tick(None)
    assert client.states == [] and client.discovery_modes == []


def test_discovery_once_per_connect_then_state_every_tick(mqtt):
    service, client, sup, clock, calls = mqtt
    client.connected = True
    service.on_tick(None)
    service.on_tick(None)
    assert client.discovery_modes == [ControlMode.AUTO]
    assert len(client.states) == 2
    snapshot, host = client.states[-1]
    assert snapshot["mode"] == "auto" and set(snapshot["setpoints"]) == set(sup.setpoints)
    assert host == {"cpu_temp_c": 41.0}
    # reconnect -> Discovery again (HA may have restarted and lost the retained configs)
    service.connection_changed(True)
    service.on_tick(None)
    assert client.discovery_modes == [ControlMode.AUTO, ControlMode.AUTO]


def test_discovery_republished_when_crossing_the_manual_boundary(mqtt):
    service, client, sup, *_ = mqtt
    client.connected = True
    service.on_tick(None)
    sup.submit(SetMode(ControlMode.MIXED))
    service.on_tick(None)
    assert client.discovery_modes == [ControlMode.AUTO]  # mixed: PWM numbers still hidden
    sup.submit(SetMode(ControlMode.MANUAL))
    service.on_tick(None)
    sup.submit(SetMode(ControlMode.AUTO))
    service.on_tick(None)
    assert client.discovery_modes == [ControlMode.AUTO, ControlMode.MANUAL, ControlMode.AUTO]


def test_host_metrics_refresh_every_interval(mqtt):
    service, client, sup, clock, calls = mqtt
    client.connected = True
    for _ in range(3):  # t = 0, 2, 4: one collection at t=0, then cached
        service.on_tick(None)
        clock.t += 2.0
    assert calls["hostinfo"] == 1
    assert all(host == {"cpu_temp_c": 41.0} for _, host in client.states)
    clock.t = 10.0  # >= interval since the last collection -> refreshed
    service.on_tick(None)
    assert calls["hostinfo"] == 2
    assert client.states[-1][1] == {"cpu_temp_c": 42.0}


def test_publish_failure_is_swallowed_and_counted(mqtt):
    service, client, *_ = mqtt
    client.connected = True
    client.fail_publish = True
    service.on_tick(None)
    assert service.publish_errors == 1


def test_from_config_wires_supervisor_and_connection_state(cfg: MpcConfig):
    sup = Supervisor(cfg)
    app = _app(
        cfg,
        mqtt={
            "enabled": True,
            "host": "broker",
            "port": 1884,
            "username": "u",
            "password": "p",
            "node_id": "node",
            "discovery_prefix": "ha",
        },
        host={"interval_s": 7},
    )
    service = MqttService.from_config(app, sup, client_factory=FakeMqttClient)
    client = service.client
    assert client.kwargs["host"] == "broker" and client.kwargs["port"] == 1884
    assert client.kwargs["node_id"] == "node" and client.kwargs["discovery_prefix"] == "ha"
    assert client.kwargs["username"] == "u" and client.kwargs["password"] == "p"
    assert client.kwargs["cfg"] is sup.base_config
    assert client.kwargs["on_intent"] == sup.submit
    assert service.host_interval_s == 7.0
    service.start()
    assert client.calls == ["connect_async", "loop_start"]
    assert sup.snapshot().mqtt_connected is False
    client.kwargs["on_connection_change"](True)
    assert sup.snapshot().mqtt_connected is True
    client.connected = True
    service.on_tick(None)
    assert client.discovery_modes == [ControlMode.AUTO]
    client.kwargs["on_connection_change"](False)
    assert sup.snapshot().mqtt_connected is False
    service.stop()
    assert client.calls[-2:] == ["disconnect", "loop_stop"]
