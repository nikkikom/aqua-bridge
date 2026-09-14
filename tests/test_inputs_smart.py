"""Tests for aqua_bridge.publishers.inputs.SmartInbox (the DAS plan, section 1
"SMART path" / section 7 "POST /api/in/smart").
"""

from __future__ import annotations

import asyncio
import json
import math
import socket
from typing import Any

import pytest
from hypothesis import given
from hypothesis import strategies as st

from aqua_bridge.publishers.inputs import DEFAULT_MAX_AGE_S, SmartInbox, smart_topic_filter


class _FakeClock:
    def __init__(self, t: float = 1000.0) -> None:
        self.t = t

    def __call__(self) -> float:
        return self.t


_VALID = {
    "serial": "WD-ABC123",
    "model": "WDC WD40EFRX",
    "temp_c": 34.5,
    "ts_wall": 1_700_000_000.0,
}


# --- record(): accept ------------------------------------------------------------------


def test_record_accepts_a_well_formed_sample() -> None:
    clock = _FakeClock(100.0)
    inbox = SmartInbox(max_age_s=60.0, clock=clock)
    assert inbox.record(dict(_VALID)) is True
    assert inbox.accepted == 1
    assert inbox.rejected == 0
    snap = inbox.snapshot()
    assert snap == {
        "WD-ABC123": {
            "temp_c": pytest.approx(34.5),
            "age_s": pytest.approx(0.0),
            "model": "WDC WD40EFRX",
        }
    }


def test_record_accepts_without_optional_model_or_ts_wall() -> None:
    inbox = SmartInbox(max_age_s=60.0, clock=_FakeClock())
    assert inbox.record({"serial": "S1", "temp_c": 40.0}) is True
    snap = inbox.snapshot()
    assert snap["S1"]["model"] is None


def test_record_accepts_integer_temp_c() -> None:
    inbox = SmartInbox(max_age_s=60.0, clock=_FakeClock())
    assert inbox.record({"serial": "S1", "temp_c": 40}) is True
    assert inbox.snapshot()["S1"]["temp_c"] == pytest.approx(40.0)


def test_record_overwrites_the_same_serial_and_refreshes_receipt_time() -> None:
    clock = _FakeClock(0.0)
    inbox = SmartInbox(max_age_s=60.0, clock=clock)
    inbox.record({"serial": "S1", "temp_c": 30.0})
    clock.t = 10.0
    inbox.record({"serial": "S1", "temp_c": 35.0})
    snap = inbox.snapshot()
    assert snap["S1"]["temp_c"] == pytest.approx(35.0)
    assert snap["S1"]["age_s"] == pytest.approx(0.0)


# --- record(): reject -------------------------------------------------------------------


@pytest.mark.parametrize(
    "payload",
    [
        None,
        [],
        "a string",
        123,
        {},
        {"temp_c": 30.0},  # missing serial
        {"serial": "", "temp_c": 30.0},  # empty serial
        {"serial": 5, "temp_c": 30.0},  # serial not a string
        {"serial": "S1"},  # missing temp_c
        {"serial": "S1", "temp_c": "hot"},  # temp_c not a number
        {"serial": "S1", "temp_c": True},  # bool is not a real temperature
        {"serial": "S1", "temp_c": math.nan},
        {"serial": "S1", "temp_c": math.inf},
        {"serial": "S1", "temp_c": -math.inf},
        {"serial": "S1", "temp_c": 30.0, "model": 5},  # model wrong type
        {"serial": "S1", "temp_c": 30.0, "ts_wall": "now"},  # ts_wall wrong type
        {"serial": "S1", "temp_c": 30.0, "ts_wall": math.nan},
        {"serial": "S1", "temp_c": 30.0, "ts_wall": True},
    ],
)
def test_record_rejects_malformed_payloads(payload: Any) -> None:
    inbox = SmartInbox(max_age_s=60.0, clock=_FakeClock())
    assert inbox.record(payload) is False
    assert inbox.rejected == 1
    assert inbox.accepted == 0
    assert inbox.snapshot() == {}


def test_rejected_payload_does_not_clobber_an_existing_good_entry() -> None:
    inbox = SmartInbox(max_age_s=60.0, clock=_FakeClock())
    inbox.record({"serial": "S1", "temp_c": 30.0})
    inbox.record({"serial": "S1", "temp_c": "garbage"})
    assert inbox.accepted == 1
    assert inbox.rejected == 1
    assert inbox.snapshot()["S1"]["temp_c"] == pytest.approx(30.0)


# --- staleness / snapshot() -------------------------------------------------------------


def test_snapshot_excludes_a_sample_older_than_max_age_s() -> None:
    clock = _FakeClock(0.0)
    inbox = SmartInbox(max_age_s=10.0, clock=clock)
    inbox.record({"serial": "S1", "temp_c": 30.0})
    clock.t = 5.0
    assert "S1" in inbox.snapshot()
    clock.t = 10.01
    assert inbox.snapshot() == {}


def test_snapshot_age_s_uses_receipt_time_not_ts_wall() -> None:
    """ts_wall is wall-clock and untrusted for staleness (module docstring);
    only the injected monotonic clock at record() time counts."""
    clock = _FakeClock(1000.0)
    inbox = SmartInbox(max_age_s=600.0, clock=clock)
    # ts_wall claims a time far in the past/future; must not affect age_s.
    inbox.record({"serial": "S1", "temp_c": 30.0, "ts_wall": 1.0})
    clock.t = 1005.0
    assert inbox.snapshot()["S1"]["age_s"] == pytest.approx(5.0)


def test_default_max_age_s_matches_the_plan_default() -> None:
    assert DEFAULT_MAX_AGE_S == 300.0


def test_max_age_s_must_be_positive() -> None:
    with pytest.raises(ValueError, match="max_age_s"):
        SmartInbox(max_age_s=0.0)
    with pytest.raises(ValueError, match="max_age_s"):
        SmartInbox(max_age_s=-1.0)


# --- on_message() (the MQTT shape) -------------------------------------------------------


def test_on_message_accepts_valid_json_bytes() -> None:
    inbox = SmartInbox(max_age_s=60.0, clock=_FakeClock())
    inbox.on_message("aqua-bridge/in/smart/S1", json.dumps(_VALID).encode("utf-8"))
    assert "WD-ABC123" in inbox.snapshot()
    assert inbox.rejected == 0


def test_on_message_accepts_str_payload_too() -> None:
    inbox = SmartInbox(max_age_s=60.0, clock=_FakeClock())
    inbox.on_message("t", json.dumps({"serial": "S1", "temp_c": 30.0}))
    assert "S1" in inbox.snapshot()


@pytest.mark.parametrize(
    "payload",
    [b"{not json", b"", b"null", b"[1,2,3]", b"\xff\xfe garbage", "not json either"],
)
def test_on_message_never_raises_on_garbage(payload: Any) -> None:
    inbox = SmartInbox(max_age_s=60.0, clock=_FakeClock())
    inbox.on_message("aqua-bridge/in/smart/S1", payload)  # must not raise
    assert inbox.snapshot() == {}
    assert inbox.rejected == 1


def test_smart_topic_filter_shape() -> None:
    assert smart_topic_filter("aqua-bridge") == "aqua-bridge/in/smart/+"


# --- fuzz: record() / on_message() never raise -------------------------------------------

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
        st.lists(children, max_size=4), st.dictionaries(st.text(max_size=8), children, max_size=4)
    ),
    max_leaves=8,
)


@pytest.mark.fuzzy
@given(payload=_json_value)
def test_fuzz_record_never_raises(payload: Any) -> None:
    inbox = SmartInbox(max_age_s=60.0, clock=_FakeClock())
    inbox.record(payload)  # must not raise, whatever the shape


@pytest.mark.fuzzy
@given(body=st.binary(max_size=64))
def test_fuzz_on_message_never_raises_on_arbitrary_bytes(body: bytes) -> None:
    inbox = SmartInbox(max_age_s=60.0, clock=_FakeClock())
    inbox.on_message("aqua-bridge/in/smart/x", body)  # must not raise


# --- POST /api/in/smart (the HTTP twin, plan section 7) -----------------------------------


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


def _run(coro):
    return asyncio.run(coro)


class _StubSurface:
    """The bare minimum create_app needs from a ControlSurface; unused by
    the /api/in/smart route itself."""


async def _smart_client(smart_inbox: Any):
    from aiohttp.test_utils import TestClient, TestServer

    from aqua_bridge.publishers.http import create_app
    from http_fixtures import client_auth, make_authenticator

    app = create_app(_StubSurface(), cfg=None, auth=make_authenticator(), smart_inbox=smart_inbox)
    server = TestServer(app)
    client = TestClient(server, **client_auth())
    await client.start_server()
    return client


@needs_bind
def test_post_in_smart_stores_a_valid_body() -> None:
    inbox = SmartInbox(max_age_s=60.0, clock=_FakeClock())

    async def scenario() -> None:
        client = await _smart_client(inbox)
        try:
            resp = await client.post("/api/in/smart", json=dict(_VALID))
            assert resp.status == 200
            body = await resp.json()
            assert body == {"ok": True}
        finally:
            await client.close()

    _run(scenario())
    assert "WD-ABC123" in inbox.snapshot()


@needs_bind
def test_post_in_smart_rejects_a_malformed_body_as_4xx() -> None:
    inbox = SmartInbox(max_age_s=60.0, clock=_FakeClock())

    async def scenario() -> None:
        client = await _smart_client(inbox)
        try:
            resp = await client.post("/api/in/smart", json={"serial": "S1"})
            assert 400 <= resp.status < 500
        finally:
            await client.close()

    _run(scenario())
    assert inbox.snapshot() == {}


@needs_bind
def test_post_in_smart_malformed_json_is_4xx_not_5xx() -> None:
    inbox = SmartInbox(max_age_s=60.0, clock=_FakeClock())

    async def scenario() -> None:
        client = await _smart_client(inbox)
        try:
            resp = await client.post(
                "/api/in/smart", data=b"{not json", headers={"Content-Type": "application/json"}
            )
            assert 400 <= resp.status < 500
        finally:
            await client.close()

    _run(scenario())


@needs_bind
def test_post_in_smart_without_an_inbox_is_404_not_5xx() -> None:
    async def scenario() -> None:
        client = await _smart_client(None)
        try:
            resp = await client.post("/api/in/smart", json=dict(_VALID))
            assert resp.status == 404
        finally:
            await client.close()

    _run(scenario())


@needs_bind
@pytest.mark.fuzzy
@given(body=_json_value)
def test_fuzz_post_in_smart_never_5xx(body: Any) -> None:
    inbox = SmartInbox(max_age_s=60.0, clock=_FakeClock())

    async def scenario() -> None:
        client = await _smart_client(inbox)
        try:
            resp = await client.post(
                "/api/in/smart",
                data=json.dumps(body, allow_nan=True).encode("utf-8"),
                headers={"Content-Type": "application/json"},
            )
            assert resp.status < 500
        finally:
            await client.close()

    _run(scenario())
