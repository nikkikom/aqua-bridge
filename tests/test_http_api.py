"""Tests for aqua_bridge.publishers.http (PROJECT.md sections 4.8, 6).

Runs the real aiohttp app against a stub ControlSurface with
aiohttp.test_utils, driven from asyncio.run (no pytest-aiohttp plugin).
"""

from __future__ import annotations

import asyncio
import json
import math
import socket
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import pytest
from aiohttp.test_utils import TestClient, TestServer
from hypothesis import given
from hypothesis import strategies as st

from aqua_bridge.config import AppConfig, load_config
from aqua_bridge.control.intents import (
    ClearOverride,
    ControlMode,
    ControlSnapshot,
    Intent,
    IntentConflict,
    IntentInvalid,
    Preset,
    SetMode,
    SetPreset,
    SetPwm,
    SetSetpoint,
    SolverStatus,
)
from aqua_bridge.model import FaultReason, Mode, MpcCommand, MpcConfig, PlantObservation
from aqua_bridge.publishers.http import create_app
from http_fixtures import client_auth, make_authenticator

_EXAMPLE_CFG = load_config(Path(__file__).resolve().parent.parent / "config.example.yaml").mpc


def _can_bind_localhost() -> bool:
    """aiohttp's TestServer binds a real localhost socket; some sandboxes deny that
    with PermissionError. Skip the module explicitly there instead of failing --
    CI (GitHub runners, the Pi) binds fine and runs everything."""
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.bind(("127.0.0.1", 0))
    except OSError:
        return False
    return True


pytestmark = pytest.mark.skipif(
    not _can_bind_localhost(),
    reason="binding a localhost TCP socket is denied here (sandbox); aiohttp TestServer needs it",
)


# ---------------------------------------------------------------------------
# Stub ControlSurface
# ---------------------------------------------------------------------------


@dataclass
class StubSurface:
    """A minimal, faithful-enough ControlSurface for HTTP tests.

    Applies intents itself (auto -> pwm conflict, unknown channel, out of
    range) exactly the way the real loop's submit() must, and records every
    intent that was accepted so tests can assert none out-of-range leaked
    through.
    """

    cfg: MpcConfig
    control_mode: ControlMode = ControlMode.AUTO
    preset: Preset = Preset.NORMAL
    accepted: list[Intent] = field(default_factory=list)
    obs: PlantObservation | None = None
    last_cmd: MpcCommand | None = None
    device_health: dict[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        self.setpoints: dict[str, float] = dict(self.cfg.setpoints)
        self.overrides: dict[str, float] = {}

    def snapshot(self) -> ControlSnapshot:
        return ControlSnapshot(
            obs=self.obs,
            last_cmd=self.last_cmd,
            control_mode=self.control_mode,
            setpoints=dict(self.setpoints),
            overrides=dict(self.overrides),
            preset=self.preset,
            channels=self.cfg.channels,
            temps=self.cfg.temps,
            pwm_min=self.cfg.pwm_min,
            pwm_max=self.cfg.pwm_max,
            solver_status=ControlSnapshot.solver_status_for(self.last_cmd),
            fault_reason=FaultReason.SENSOR_GATE if self.last_cmd is None else None,
            fault_since_ts=None,
            usb_present=True,
            mqtt_connected=None,
            uptime_s=1.0,
            version="test",
            device_health=dict(self.device_health),
        )

    def submit(self, intent: Intent) -> None:
        if isinstance(intent, SetMode):
            self.control_mode = intent.mode
        elif isinstance(intent, SetSetpoint):
            if intent.channel not in self.cfg.setpoints:
                raise IntentInvalid(f"unknown setpoint channel {intent.channel!r}")
            self.setpoints[intent.channel] = intent.celsius
        elif isinstance(intent, SetPwm):
            if self.control_mode is ControlMode.AUTO:
                raise IntentConflict("cannot set raw PWM while in auto mode")
            if intent.channel not in self.cfg.channels:
                raise IntentInvalid(f"unknown channel {intent.channel!r}")
            if not (self.cfg.pwm_min <= intent.pwm <= self.cfg.pwm_max):
                raise IntentInvalid(
                    f"pwm {intent.pwm} out of range [{self.cfg.pwm_min}, {self.cfg.pwm_max}]"
                )
            self.overrides[intent.channel] = intent.pwm
        elif isinstance(intent, SetPreset):
            self.preset = intent.name
        elif isinstance(intent, ClearOverride):
            if intent.channel is None:
                self.overrides.clear()
            else:
                if intent.channel not in self.cfg.channels:
                    raise IntentInvalid(f"unknown channel {intent.channel!r}")
                self.overrides.pop(intent.channel, None)
        else:  # pragma: no cover - defensive
            raise IntentInvalid(f"unknown intent {intent!r}")
        self.accepted.append(intent)


def _run(coro):
    return asyncio.run(coro)


# A fixed reader, not the real aqua_bridge.hostinfo.collect_hostinfo: the HTTP
# suites must stay deterministic and independent of the sandbox's /proc, /sys.
_HOST_STUB = {"cpu_temp_c": 42.0, "load1": 0.5, "uptime_s": 100.0}


async def _client(surface: StubSurface) -> TestClient:
    app = create_app(surface, cfg=None, auth=make_authenticator(), hostinfo=lambda: _HOST_STUB)
    server = TestServer(app)
    client = TestClient(server, **client_auth())
    await client.start_server()
    return client


@pytest.fixture
def surface(cfg: MpcConfig) -> StubSurface:
    return StubSurface(cfg=cfg)


# ---------------------------------------------------------------------------
# GET endpoints
# ---------------------------------------------------------------------------


def test_get_state_shape_matches_model(surface: StubSurface, cfg: MpcConfig) -> None:
    async def scenario() -> None:
        surface.obs = PlantObservation(
            temps={t: 30.0 for t in cfg.temps},
            rpm={c: 1000.0 for c in cfg.channels},
            pwm={c: 0.5 for c in cfg.channels},
            ts=1.0,
        )
        surface.last_cmd = MpcCommand(pwm=dict.fromkeys(cfg.channels, 0.5), mode=Mode.AUTO)
        client = await _client(surface)
        try:
            resp = await client.get("/api/state")
            assert resp.status == 200
            body = await resp.json()
        finally:
            await client.close()
        expected = surface.snapshot().state_payload()
        expected["host"] = _HOST_STUB
        assert body == expected
        assert body["obs"] == surface.obs.to_dict()
        assert body["cmd"] == surface.last_cmd.to_dict()
        assert body["host"] == _HOST_STUB
        assert set(body) == {
            "obs",
            "cmd",
            "mode",
            "preset",
            "setpoints",
            "overrides",
            "channels",
            "temps",
            "pwm_min",
            "pwm_max",
            "device_health",
            "host",
        }

    _run(scenario())


def test_get_state_and_health_carry_the_device_health(surface: StubSurface) -> None:
    """Items 79 and 83: /api/state the whole blob, /api/health only {ok, problems}."""
    health = {
        "devices": [{"label": "aquaero", "stuck_channels": ["qd3"], "flows": {"flow1": 0}}],
        "fans": {"qd3": {"rpm": 0.0, "expected_rpm": 1100.0, "problems": ["qd3: 0 rpm"]}},
        "problems": ["aquaero: qd3 do not follow the written duty", "qd3: 0 rpm"],
        "ok": False,
    }

    async def scenario() -> None:
        surface.device_health = health
        client = await _client(surface)
        try:
            state = await (await client.get("/api/state")).json()
            api_health = await (await client.get("/api/health")).json()
        finally:
            await client.close()
        assert state["device_health"] == health
        assert api_health["device_health"] == {"ok": False, "problems": health["problems"]}

    _run(scenario())


def test_get_state_and_health_carry_the_board_s_own_health(surface: StubSurface) -> None:
    """Item 103: the board's verdict is the device_health ``host`` key, and its problems
    are in the one list /api/health shows -- and nowhere near the observation."""
    board = {
        "cpu_temp_c": 82.0,
        "air_c": 27.0,
        "air_temps": ["air_z1"],
        "divergence_c": 55.0,
        "load1": 0.1,
        "idle": True,
        "throttled": {"hex": "0x4", "now": True, "throttled_now": True},
        "problems": ["host: the board is throttling now (throttled, get_throttled 0x4)"],
        "ok": False,
    }
    health = {
        "devices": [],
        "fans": {},
        "host": board,
        "problems": list(board["problems"]),
        "ok": False,
    }

    async def scenario() -> None:
        surface.device_health = health
        client = await _client(surface)
        try:
            state = await (await client.get("/api/state")).json()
            api_health = await (await client.get("/api/health")).json()
        finally:
            await client.close()
        assert state["device_health"]["host"] == board
        assert api_health["device_health"] == {"ok": False, "problems": board["problems"]}
        # a health signal only: never in the observation the solver reads
        assert state["obs"] is None or "cpu_temp_c" not in (state["obs"].get("temps") or {})

    _run(scenario())


def test_device_health_is_empty_and_ok_before_the_first_tick(surface: StubSurface) -> None:
    async def scenario() -> None:
        client = await _client(surface)
        try:
            state = await (await client.get("/api/state")).json()
            api_health = await (await client.get("/api/health")).json()
        finally:
            await client.close()
        assert state["device_health"] == {}
        assert api_health["device_health"] == {"ok": True, "problems": []}

    _run(scenario())


def test_get_state_host_reader_error_is_swallowed(surface: StubSurface) -> None:
    def broken() -> dict:
        raise RuntimeError("no /proc here")

    async def scenario() -> None:
        app = create_app(surface, cfg=None, auth=make_authenticator(), hostinfo=broken)
        server = TestServer(app)
        client = TestClient(server, **client_auth())
        await client.start_server()
        try:
            resp = await client.get("/api/state")
            assert resp.status == 200
            body = await resp.json()
        finally:
            await client.close()
        assert body["host"] == {}

    _run(scenario())


def test_get_state_host_cache_reads_interval_from_cfg(surface: StubSurface, cfg: MpcConfig) -> None:
    """``host.interval_s`` (section 7) gates the cache in ``create_app`` too, not
    only the MQTT publisher: ``0`` refreshes on every request, the section's
    default (5.0) does not move within one fast test."""
    calls = {"n": 0}

    def counting() -> dict:
        calls["n"] += 1
        return {"cpu_temp_c": float(calls["n"])}

    async def scenario(host_section: dict, expected_calls: int) -> None:
        app_cfg = AppConfig(mpc=cfg, host=host_section)
        app = create_app(surface, app_cfg, auth=make_authenticator(), hostinfo=counting)
        server = TestServer(app)
        client = TestClient(server, **client_auth())
        await client.start_server()
        try:
            for _ in range(3):
                resp = await client.get("/api/state")
                assert resp.status == 200
        finally:
            await client.close()
        assert calls["n"] == expected_calls

    calls["n"] = 0
    _run(scenario({"interval_s": 0}, 3))
    calls["n"] = 0
    _run(scenario({}, 1))  # default 5.0 s: cached across three requests in one test


def test_get_health_shape(surface: StubSurface) -> None:
    async def scenario() -> None:
        client = await _client(surface)
        try:
            resp = await client.get("/api/health")
            assert resp.status == 200
            body = await resp.json()
        finally:
            await client.close()
        assert set(body) == {
            "usb_present",
            "mqtt_connected",
            "solver",
            "fault_reason",
            "fault_since_ts",
            "uptime_s",
            "version",
            "step_ms_last",
            "step_ms_max",
            "budget_warn_count",
            "budget_alarm_count",
            "device_health",
        }
        assert body["solver"] == SolverStatus.FAULT.value  # no last_cmd yet

    _run(scenario())


def test_get_index_serves_html(surface: StubSurface) -> None:
    async def scenario() -> None:
        client = await _client(surface)
        try:
            resp = await client.get("/")
            assert resp.status == 200
            assert "text/html" in resp.headers["Content-Type"]
            text = await resp.text()
        finally:
            await client.close()
        assert "<html" in text.lower()

    _run(scenario())


# ---------------------------------------------------------------------------
# POST happy paths
# ---------------------------------------------------------------------------


def test_post_mode_happy_path(surface: StubSurface) -> None:
    async def scenario() -> None:
        client = await _client(surface)
        try:
            resp = await client.post("/api/mode", json={"mode": "manual"})
            assert resp.status == 200
        finally:
            await client.close()
        assert surface.control_mode is ControlMode.MANUAL

    _run(scenario())


def test_post_setpoint_happy_path(surface: StubSurface, cfg: MpcConfig) -> None:
    temp = next(iter(cfg.setpoints))

    async def scenario() -> None:
        client = await _client(surface)
        try:
            resp = await client.post("/api/setpoint", json={"channel": temp, "celsius": 33.5})
            assert resp.status == 200
        finally:
            await client.close()
        assert surface.setpoints[temp] == pytest.approx(33.5)

    _run(scenario())


def test_post_pwm_happy_path_in_manual(surface: StubSurface, cfg: MpcConfig) -> None:
    surface.control_mode = ControlMode.MANUAL
    ch = cfg.channels[0]

    async def scenario() -> None:
        client = await _client(surface)
        try:
            resp = await client.post("/api/pwm", json={"channel": ch, "pwm": 0.5})
            assert resp.status == 200
        finally:
            await client.close()
        assert surface.overrides[ch] == pytest.approx(0.5)

    _run(scenario())


def test_post_pwm_in_auto_returns_409(surface: StubSurface, cfg: MpcConfig) -> None:
    ch = cfg.channels[0]

    async def scenario() -> None:
        client = await _client(surface)
        try:
            resp = await client.post("/api/pwm", json={"channel": ch, "pwm": 0.5})
            assert resp.status == 409
            body = await resp.json()
        finally:
            await client.close()
        assert "error" in body
        assert surface.accepted == []

    _run(scenario())


def test_post_preset_happy_path(surface: StubSurface) -> None:
    async def scenario() -> None:
        client = await _client(surface)
        try:
            resp = await client.post("/api/preset", json={"name": "cool"})
            assert resp.status == 200
        finally:
            await client.close()
        assert surface.preset is Preset.COOL

    _run(scenario())


def test_post_auto_clears_one_channel(surface: StubSurface, cfg: MpcConfig) -> None:
    surface.control_mode = ControlMode.MANUAL
    ch = cfg.channels[0]
    surface.overrides[ch] = 0.7

    async def scenario() -> None:
        client = await _client(surface)
        try:
            resp = await client.post("/api/auto", json={"channel": ch})
            assert resp.status == 200
        finally:
            await client.close()
        assert ch not in surface.overrides

    _run(scenario())


def test_post_auto_clears_all_with_empty_body(surface: StubSurface, cfg: MpcConfig) -> None:
    surface.control_mode = ControlMode.MANUAL
    surface.overrides.update(dict.fromkeys(cfg.channels, 0.6))

    async def scenario() -> None:
        client = await _client(surface)
        try:
            resp = await client.post("/api/auto", json={})
            assert resp.status == 200
        finally:
            await client.close()
        assert surface.overrides == {}

    _run(scenario())


# ---------------------------------------------------------------------------
# §4.8: malformed JSON, unknown channel, out-of-range, never 5xx
# ---------------------------------------------------------------------------


def test_malformed_json_is_4xx(surface: StubSurface) -> None:
    async def scenario() -> None:
        client = await _client(surface)
        try:
            resp = await client.post(
                "/api/mode",
                data=b"{not json",
                headers={"Content-Type": "application/json"},
            )
            assert 400 <= resp.status < 500
        finally:
            await client.close()

    _run(scenario())


def test_pwm_unknown_channel_is_4xx(surface: StubSurface) -> None:
    surface.control_mode = ControlMode.MANUAL

    async def scenario() -> None:
        client = await _client(surface)
        try:
            resp = await client.post("/api/pwm", json={"channel": "does-not-exist", "pwm": 0.5})
            assert 400 <= resp.status < 500
        finally:
            await client.close()
        assert surface.accepted == []

    _run(scenario())


def test_pwm_out_of_construction_range_is_4xx(surface: StubSurface, cfg: MpcConfig) -> None:
    surface.control_mode = ControlMode.MANUAL
    ch = cfg.channels[0]

    async def scenario() -> None:
        client = await _client(surface)
        try:
            resp = await client.post("/api/pwm", json={"channel": ch, "pwm": 5.0})
            assert 400 <= resp.status < 500
        finally:
            await client.close()
        assert surface.accepted == []

    _run(scenario())


def test_pwm_out_of_configured_range_is_4xx(surface: StubSurface, cfg: MpcConfig) -> None:
    # 0.05 is inside [0, 1] (construction accepts it) but below cfg.pwm_min.
    surface.control_mode = ControlMode.MANUAL
    ch = cfg.channels[0]
    assert cfg.pwm_min > 0.05

    async def scenario() -> None:
        client = await _client(surface)
        try:
            resp = await client.post("/api/pwm", json={"channel": ch, "pwm": 0.05})
            assert 400 <= resp.status < 500
        finally:
            await client.close()
        assert surface.accepted == []

    _run(scenario())


def test_setpoint_missing_field_is_4xx(surface: StubSurface) -> None:
    async def scenario() -> None:
        client = await _client(surface)
        try:
            resp = await client.post("/api/setpoint", json={"channel": "coolant"})
            assert 400 <= resp.status < 500
        finally:
            await client.close()

    _run(scenario())


def test_unknown_intent_kind_route_is_404(surface: StubSurface) -> None:
    async def scenario() -> None:
        client = await _client(surface)
        try:
            resp = await client.post("/api/frobnicate", json={})
            assert resp.status == 404
        finally:
            await client.close()

    _run(scenario())


def test_body_not_an_object_is_4xx(surface: StubSurface) -> None:
    async def scenario() -> None:
        client = await _client(surface)
        try:
            resp = await client.post("/api/mode", json=["auto"])
            assert 400 <= resp.status < 500
        finally:
            await client.close()

    _run(scenario())


# ---------------------------------------------------------------------------
# §4.5: fuzz JSON bodies for /api/pwm and /api/setpoint
# ---------------------------------------------------------------------------

_json_scalar = st.one_of(
    st.none(),
    st.booleans(),
    st.integers(),
    st.floats(allow_nan=True, allow_infinity=True),
    st.text(),
)
_json_value = st.recursive(
    _json_scalar,
    lambda children: st.one_of(
        st.lists(children, max_size=4),
        st.dictionaries(st.text(max_size=8), children, max_size=4),
    ),
    max_leaves=8,
)
_fuzz_body = st.one_of(
    _json_value,
    st.dictionaries(st.sampled_from(["channel", "pwm", "celsius", "extra"]), _json_value),
)


def _dumps_allow_nan(value: Any) -> str:
    # Python's json module happily emits NaN/Infinity literals (not valid
    # JSON, but valid for what we're fuzzing: aiohttp's client still sends
    # the bytes and the server's json.loads() accepts them the same way).
    return json.dumps(value, allow_nan=True)


@pytest.mark.fuzzy
@given(body=_fuzz_body)
def test_fuzz_pwm_never_5xx_and_never_leaks_bad_pwm(body: Any) -> None:
    surface = StubSurface(cfg=_EXAMPLE_CFG, control_mode=ControlMode.MANUAL)

    async def scenario() -> None:
        client = await _client(surface)
        try:
            resp = await client.post(
                "/api/pwm",
                data=_dumps_allow_nan(body).encode("utf-8"),
                headers={"Content-Type": "application/json"},
            )
            assert resp.status < 500
        finally:
            await client.close()

    _run(scenario())

    for intent in surface.accepted:
        if isinstance(intent, SetPwm):
            assert math.isfinite(intent.pwm)
            assert surface.cfg.pwm_min <= intent.pwm <= surface.cfg.pwm_max


@pytest.mark.fuzzy
@given(body=_fuzz_body)
def test_fuzz_setpoint_never_5xx_and_never_leaks_bad_value(
    body: Any,
) -> None:
    surface = StubSurface(cfg=_EXAMPLE_CFG)

    async def scenario() -> None:
        client = await _client(surface)
        try:
            resp = await client.post(
                "/api/setpoint",
                data=_dumps_allow_nan(body).encode("utf-8"),
                headers={"Content-Type": "application/json"},
            )
            assert resp.status < 500
        finally:
            await client.close()

    _run(scenario())

    for intent in surface.accepted:
        if isinstance(intent, SetSetpoint):
            assert math.isfinite(intent.celsius)
            assert intent.channel in surface.cfg.setpoints


# ---------------------------------------------------------------------------
# POST /api/ident (DAS plan sections 5 and 7) against the real Supervisor
# ---------------------------------------------------------------------------


async def _post_real(surface: Any, posts: list[tuple[str, Any]]) -> list[tuple[int, Any]]:
    client = TestClient(
        TestServer(create_app(surface, cfg=None, auth=make_authenticator())), **client_auth()
    )
    await client.start_server()
    out = []
    try:
        for path, body in posts:
            if path.startswith("GET "):
                resp = await client.get(path[4:])
            else:
                resp = await client.post(
                    path,
                    data=_dumps_allow_nan(body).encode("utf-8"),
                    headers={"Content-Type": "application/json"},
                )
            out.append((resp.status, await resp.json()))
    finally:
        await client.close()
    return out


def test_post_ident_start_stop_and_refusals() -> None:
    from test_ident_experiment import Rig, ident_cfg

    rig = Rig(ident_cfg())
    results = _run(
        _post_real(
            rig.sup,
            [
                ("/api/ident", {"action": "start", "group": "front"}),  # no tick yet
                ("/api/ident", {"action": "start", "group": "rear"}),
                ("/api/ident", {"action": "start"}),
                ("/api/ident", ["start"]),
            ],
        )
    )
    assert [s for s, _ in results] == [409, 400, 400, 400]
    assert "no_tick" in results[0][1]["error"]
    rig.ticks(8)
    results = _run(
        _post_real(
            rig.sup,
            [
                ("/api/ident", {"action": "start", "group": "front"}),
                ("/api/ident", {"action": "start", "channel": "fb1"}),
                ("GET /api/model", None),
                ("/api/ident", {"action": "stop"}),
                ("GET /api/model", None),
            ],
        )
    )
    assert [s for s, _ in results] == [200, 409, 200, 200, 200]
    assert "already running" in results[1][1]["error"]
    assert results[2][1]["experiment"]["running"] is True
    assert results[2][1]["experiment"]["target"] == {"kind": "group", "name": "front"}
    assert results[4][1]["experiment"]["last_abort_reason"] == "stop"


def test_post_ident_disabled_is_409_and_legacy_is_400(cfg: MpcConfig) -> None:
    from aqua_bridge.control.supervisor import Supervisor
    from test_ident_experiment import Rig, ident_cfg

    rig = Rig(ident_cfg(ident_enabled=False))
    rig.ticks(8)
    ((status, body),) = _run(
        _post_real(rig.sup, [("/api/ident", {"action": "start", "group": "front"})])
    )
    assert status == 409 and "ident_enabled" in body["error"]
    legacy = Supervisor(cfg)
    ((status, body),) = _run(
        _post_real(legacy, [("/api/ident", {"action": "start", "channel": cfg.channels[0]})])
    )
    assert status == 400 and "DAS" in body["error"]


@pytest.mark.fuzzy
@given(
    body=st.one_of(
        _json_value,
        st.dictionaries(st.sampled_from(["action", "group", "channel", "extra"]), _json_value),
        st.fixed_dictionaries(
            {"action": st.sampled_from(["start", "stop"])},
            optional={
                "group": st.sampled_from(["front", "fb1", "nope", ""]),
                "channel": st.sampled_from(["fa1", "fc1", "front", 3]),
            },
        ),
    )
)
def test_fuzz_ident_never_5xx(body: Any) -> None:
    from aqua_bridge.control.supervisor import Supervisor
    from test_ident_experiment import ident_cfg

    sup = Supervisor(ident_cfg())
    ((status, _),) = _run(_post_real(sup, [("/api/ident", body)]))
    assert status in (200, 400, 409)
