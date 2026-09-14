"""HTTPS and basic auth for the API (PROJECT.md sections 4.8, 6).

Credentials file parsing and hashing, the ``http:`` settings, the
authenticator's cache, backoff and reload, TLS over a certificate generated
with ``openssl`` (skipped with a reason where it is missing), 401 on every
route without or with wrong credentials, plain HTTP refused, the startup
refusal paths of :class:`~aqua_bridge.publishers.runtime.HttpService`, and
fuzzed ``Authorization`` headers never answered with a 5xx.
"""

from __future__ import annotations

import asyncio
import base64
import json
import logging
import os
import socket
import ssl
import urllib.error
import urllib.request
from pathlib import Path
from typing import Any

import aiohttp
import pytest
import yaml
from aiohttp.test_utils import TestClient, TestServer
from hypothesis import given, settings
from hypothesis import strategies as st

from aqua_bridge.config import AppConfig, load_config
from aqua_bridge.control.supervisor import Supervisor
from aqua_bridge.model import MpcConfig
from aqua_bridge.publishers import httpauth
from aqua_bridge.publishers.http import create_app, run_http
from aqua_bridge.publishers.httpauth import (
    HASH_SCHEME,
    BasicAuthenticator,
    HttpSettings,
    HttpSetupError,
    build_ssl_context,
    hash_password,
    load_credentials,
    parse_authorization,
    parse_credentials,
    parse_hash,
    update_credentials_text,
    verify_password,
)
from aqua_bridge.publishers.runtime import HttpService
from http_fixtures import (
    TEST_ITERATIONS,
    TEST_PASSWORD,
    TEST_USER,
    auth_settings,
    basic_header,
    make_authenticator,
    make_tls_files,
    write_credentials,
)

REPO_ROOT = Path(__file__).resolve().parent.parent


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


class FakeClock:
    def __init__(self) -> None:
        self.t = 1000.0

    def __call__(self) -> float:
        return self.t


# ---------------------------------------------------------------------------
# Hashes and the credentials file
# ---------------------------------------------------------------------------


def test_hash_roundtrip_per_user_salt_and_stored_iterations() -> None:
    a = hash_password(TEST_PASSWORD, TEST_ITERATIONS)
    b = hash_password(TEST_PASSWORD, TEST_ITERATIONS)
    assert a != b  # fresh salt per hash
    assert a.startswith(f"{HASH_SCHEME}${TEST_ITERATIONS}$")
    assert verify_password(TEST_PASSWORD, a) and verify_password(TEST_PASSWORD, b)
    assert not verify_password(TEST_PASSWORD + "x", a)
    assert not verify_password("", a)
    # The iteration count travels with the hash: a hash written with another
    # count still verifies, and the count is what parse_hash reports.
    other = hash_password(TEST_PASSWORD, TEST_ITERATIONS * 2)
    assert parse_hash(other)[0] == TEST_ITERATIONS * 2
    assert verify_password(TEST_PASSWORD, other)
    # Non-ASCII passwords are UTF-8 encoded.
    assert verify_password("pässwörd ✓", hash_password("pässwörd ✓", TEST_ITERATIONS))


@pytest.mark.parametrize(
    "encoded",
    [
        "",
        "plain",
        "md5$1000$AAAAAAAAAAAAAAAAAAAAAA$" + "A" * 43,
        f"{HASH_SCHEME}$0$AAAAAAAAAAAAAAAAAAAAAA$" + "A" * 43,
        f"{HASH_SCHEME}$-5$AAAAAAAAAAAAAAAAAAAAAA$" + "A" * 43,
        f"{HASH_SCHEME}$1000$short$" + "A" * 43,
        f"{HASH_SCHEME}$1000$AAAAAAAAAAAAAAAAAAAAAA$tooshort",
        f"{HASH_SCHEME}$1000$AAAAAAAAAAAAAAAAAAAAAA$" + "A" * 42 + "=",
        f"{HASH_SCHEME}$1000$AAAAAAAAAAAAAAAAAAAAAA$" + "A" * 43 + "$extra",
    ],
)
def test_malformed_hash_never_verifies(encoded: str) -> None:
    with pytest.raises(ValueError):
        parse_hash(encoded)
    assert verify_password(TEST_PASSWORD, encoded) is False


def test_parse_credentials_skips_comments_and_blank_lines() -> None:
    h1 = hash_password("one", TEST_ITERATIONS)
    h2 = hash_password("two", TEST_ITERATIONS)
    users = parse_credentials(f"# users\n\nalice:{h1}\n  \nbob.smith@lan:{h2}\n")
    assert users == {"alice": h1, "bob.smith@lan": h2}


@pytest.mark.parametrize(
    ("text", "message"),
    [
        ("alice\n", "expected user:hash"),
        ("alice:notahash\n", r"line 1 \(alice\)"),
        ("al ice:{h}\n", "invalid user name"),
        (":{h}\n", "invalid user name"),
        ("alice:{h}\nalice:{h}\n", "duplicate user"),
    ],
)
def test_parse_credentials_rejects(text: str, message: str) -> None:
    with pytest.raises(ValueError, match=message):
        parse_credentials(text.format(h=hash_password("pw", TEST_ITERATIONS)))


def test_update_credentials_text_replaces_or_appends_and_keeps_comments() -> None:
    h1, h2, h3 = (hash_password(p, TEST_ITERATIONS) for p in ("1", "2", "3"))
    text = f"# keep me\nalice:{h1}\nbob:{h2}\n"
    updated = update_credentials_text(text, "alice", h3)
    assert updated.splitlines()[0] == "# keep me"
    assert parse_credentials(updated) == {"alice": h3, "bob": h2}
    appended = update_credentials_text(updated, "carol", h1)
    assert parse_credentials(appended) == {"alice": h3, "bob": h2, "carol": h1}
    with pytest.raises(ValueError):
        update_credentials_text("garbage\n", "alice", h1)
    with pytest.raises(ValueError):
        update_credentials_text(text, "bad name", h1)


def test_load_credentials_checks_mode_and_content(tmp_path: Path) -> None:
    path = write_credentials(tmp_path / "users")
    assert set(load_credentials(path)) == {TEST_USER}
    os.chmod(path, 0o600)
    assert set(load_credentials(path)) == {TEST_USER}
    for mode in (0o644, 0o660, 0o604, 0o642):
        os.chmod(path, mode)
        with pytest.raises(HttpSetupError, match="chmod 0640"):
            load_credentials(path)
    os.chmod(path, 0o640)
    path.write_text("# nobody\n")
    with pytest.raises(HttpSetupError, match="no users"):
        load_credentials(path)
    path.write_text("alice:broken\n")
    with pytest.raises(HttpSetupError, match="line 1"):
        load_credentials(path)
    with pytest.raises(HttpSetupError, match="credentials file"):
        load_credentials(tmp_path / "missing")
    with pytest.raises(HttpSetupError, match="not a regular file"):
        load_credentials(tmp_path)


# ---------------------------------------------------------------------------
# Settings
# ---------------------------------------------------------------------------


@pytest.mark.parametrize("name", ["config.example.yaml", "config.example-das.yaml"])
def test_example_configs_show_every_http_key_with_its_default(name: str) -> None:
    section = load_config(REPO_ROOT / name).section("http")
    assert set(section) == {f for f in HttpSettings.__dataclass_fields__}
    assert HttpSettings.from_section(section) == HttpSettings()


def test_settings_defaults_when_section_is_empty() -> None:
    assert HttpSettings.from_section(None) == HttpSettings()
    assert HttpSettings.from_section({}) == HttpSettings()


@pytest.mark.parametrize(
    ("section", "message"),
    [
        ({"tls_certificate": "/x"}, "unknown key"),
        ({"enabled": "yes"}, "http.enabled"),
        ({"port": "8443"}, "http.port"),
        ({"port": 70000}, "http.port"),
        ({"port": True}, "http.port"),
        ({"bind": ""}, "http.bind"),
        ({"tls_cert": None}, "http.tls_cert"),
        ({"tls_key": 5}, "http.tls_key"),
        ({"credentials_file": "  "}, "http.credentials_file"),
        ({"realm": 'a"b'}, "http.realm"),
        ({"realm": ""}, "http.realm"),
        ({"realm": "x\r\nSet-Cookie: a=b"}, "http.realm"),
        ({"hash_iterations": 10}, "http.hash_iterations"),
        ({"hash_iterations": 1e6}, "http.hash_iterations"),
        ({"auth_cache_s": -1}, "http.auth_cache_s"),
        ({"auth_backoff_s": float("nan")}, "http.auth_backoff_s"),
        ({"auth_backoff_max_s": "1"}, "http.auth_backoff_max_s"),
        ({"auth_fail_limit": -1}, "http.auth_fail_limit"),
        ({"auth_fail_limit": 2.5}, "http.auth_fail_limit"),
        ({"auth_backoff_s": 10, "auth_backoff_max_s": 5}, "auth_backoff_max_s must be >="),
    ],
)
def test_settings_reject_invalid_values(section: dict[str, Any], message: str) -> None:
    with pytest.raises(HttpSetupError, match=message):
        HttpSettings.from_section(section)


# ---------------------------------------------------------------------------
# Authorization header and the authenticator
# ---------------------------------------------------------------------------


def _basic_raw(raw: bytes) -> str:
    return "Basic " + base64.b64encode(raw).decode("ascii")


def test_parse_authorization_cases() -> None:
    assert parse_authorization(basic_header("alice", "p:w")) == ("alice", "p:w")
    assert parse_authorization("basic " + basic_header("alice", "")[6:]) == ("alice", "")
    for header in (
        None,
        "",
        "Basic",
        "Basic ",
        "Bearer abc",
        "Basic !!!",
        "Basic " + base64.b64encode(b"alice:pw").decode()[:-1],
        _basic_raw(b"no-colon"),
        _basic_raw(b":pw"),
        _basic_raw(b"bad name:pw"),
        _basic_raw(b"alice:\xff\xfe"),
        "Basic YWxpY2U6cHc=é",
    ):
        assert parse_authorization(header) is None, header


def test_authenticator_accepts_the_user_and_rejects_everything_else() -> None:
    auth = make_authenticator(auth_fail_limit=0)
    assert auth.users == (TEST_USER,)
    ok = auth.verify(basic_header(), "c")
    assert ok.ok and ok.user == TEST_USER
    for header in (None, basic_header(TEST_USER, "wrong"), basic_header("nobody", TEST_PASSWORD)):
        decision = auth.verify(header, "c")
        assert not decision.ok and decision.retry_after_s is None


def test_authenticator_caches_verified_credentials(monkeypatch: pytest.MonkeyPatch) -> None:
    clock = FakeClock()
    settings_ = auth_settings()
    auth = BasicAuthenticator(settings_, clock=clock)
    calls = []
    real = httpauth._derive
    monkeypatch.setattr(httpauth, "_derive", lambda *a: calls.append(1) or real(*a))
    assert auth.fast(basic_header(), "c") is None  # nothing cached yet
    assert auth.verify(basic_header(), "c").ok
    assert len(calls) == 1
    for _ in range(5):
        decision = auth.fast(basic_header(), "c")
        assert decision is not None and decision.ok
    assert len(calls) == 1
    # A wrong password is never served from the cache.
    assert auth.fast(basic_header(TEST_USER, "wrong"), "c") is None
    clock.t += settings_.auth_cache_s + 1.0
    assert auth.fast(basic_header(), "c") is None  # expired
    uncached = BasicAuthenticator(auth_settings(auth_cache_s=0), clock=clock)
    assert uncached.verify(basic_header(), "c").ok
    assert uncached.fast(basic_header(), "c") is None


def test_authenticator_backoff_per_client_doubles_and_resets() -> None:
    clock = FakeClock()
    s = auth_settings()
    auth = BasicAuthenticator(s, clock=clock)
    wrong = basic_header(TEST_USER, "wrong")
    for _ in range(s.auth_fail_limit):
        decision = auth.verify(wrong, "attacker")
        assert not decision.ok and decision.retry_after_s is None
    # Locked out now, even with the right password; another client is not.
    locked = auth.verify(basic_header(), "attacker")
    assert not locked.ok and locked.retry_after_s == pytest.approx(s.auth_backoff_s)
    assert auth.verify(basic_header(), "friend").ok
    clock.t += s.auth_backoff_s
    assert not auth.verify(wrong, "attacker").ok  # one more failure: the wait doubles
    assert auth.verify(basic_header(), "attacker").retry_after_s == pytest.approx(
        2 * s.auth_backoff_s
    )
    # The wait never exceeds the maximum.
    for _ in range(40):
        clock.t += s.auth_backoff_max_s
        auth.verify(wrong, "attacker")
    wait = auth.verify(basic_header(), "attacker").retry_after_s
    assert wait is not None and wait <= s.auth_backoff_max_s
    clock.t += s.auth_backoff_max_s
    assert auth.verify(basic_header(), "attacker").ok  # success resets the count
    assert auth.verify(wrong, "attacker").retry_after_s is None
    assert auth.verify(basic_header(), "attacker").ok
    # A malformed header counts as a failure; a missing one does not.
    for _ in range(s.auth_fail_limit):
        assert not auth.verify(None, "browser").ok
    assert auth.verify(basic_header(), "browser").ok
    for _ in range(s.auth_fail_limit):
        auth.verify("Basic !!!", "fuzzer")
    assert auth.verify(basic_header(), "fuzzer").retry_after_s is not None


def test_authenticator_rereads_a_changed_credentials_file(tmp_path: Path) -> None:
    creds = write_credentials(tmp_path / "users")
    auth = BasicAuthenticator(
        auth_settings(credentials_file=str(creds), auth_fail_limit=0), clock=FakeClock()
    )
    assert auth.verify(basic_header(), "c").ok
    write_credentials(creds, {"other": "pw2"})
    os.utime(creds, ns=(1, 1))  # a different mtime even on a coarse filesystem clock
    assert not auth.verify(basic_header(), "c").ok  # TEST_USER is gone
    assert auth.verify(basic_header("other", "pw2"), "c").ok
    # An invalid or missing file denies everyone (fail closed) until fixed.
    creds.write_text("broken\n")
    assert not auth.verify(basic_header("other", "pw2"), "c").ok
    creds.unlink()
    assert not auth.verify(basic_header("other", "pw2"), "c").ok
    write_credentials(creds)
    assert auth.verify(basic_header(), "c").ok


def test_authenticator_refuses_to_start_without_usable_credentials(tmp_path: Path) -> None:
    with pytest.raises(HttpSetupError, match="credentials file"):
        BasicAuthenticator(auth_settings(credentials_file=str(tmp_path / "missing")))


_FUZZ_AUTH = make_authenticator(auth_fail_limit=0)


@pytest.mark.fuzzy
@given(header=st.one_of(st.none(), st.text(max_size=80), st.binary(max_size=60).map(_basic_raw)))
def test_fuzz_authenticator_never_raises_and_never_accepts(header: str | None) -> None:
    decision = _FUZZ_AUTH.verify(header, "fuzz")
    assert not decision.ok


# ---------------------------------------------------------------------------
# TLS context
# ---------------------------------------------------------------------------


@pytest.fixture(scope="module")
def tls_files(tmp_path_factory: pytest.TempPathFactory) -> tuple[Path, Path]:
    files = make_tls_files(tmp_path_factory.mktemp("tls"))
    if files is None:
        pytest.skip("openssl is not installed; the TLS tests need it to make a certificate")
    return files


def test_build_ssl_context_refusals(tmp_path: Path, tls_files: tuple[Path, Path]) -> None:
    cert, key = tls_files
    assert isinstance(
        build_ssl_context(HttpSettings(tls_cert=str(cert), tls_key=str(key))), ssl.SSLContext
    )
    cases = [
        (HttpSettings(tls_cert=str(tmp_path / "none.pem"), tls_key=str(key)), "does not exist"),
        (HttpSettings(tls_cert=str(cert), tls_key=str(tmp_path / "none.pem")), "TLS key"),
    ]
    garbage = tmp_path / "garbage.pem"
    garbage.write_text("not a certificate\n")
    os.chmod(garbage, 0o640)
    cases.append((HttpSettings(tls_cert=str(garbage), tls_key=str(key)), "cannot load"))
    cases.append((HttpSettings(tls_cert=str(cert), tls_key=str(garbage)), "cannot load"))
    loose = tmp_path / "loose.pem"
    loose.write_bytes(key.read_bytes())
    os.chmod(loose, 0o644)
    cases.append((HttpSettings(tls_cert=str(cert), tls_key=str(loose)), "chmod 0640"))
    for settings_, message in cases:
        with pytest.raises(HttpSetupError, match=message):
            build_ssl_context(settings_)


# ---------------------------------------------------------------------------
# Over the wire
# ---------------------------------------------------------------------------


def _http_section(tls_files: tuple[Path, Path], **overrides: Any) -> dict[str, Any]:
    cert, key = tls_files
    settings_ = auth_settings()
    section: dict[str, Any] = {
        "enabled": True,
        "bind": "127.0.0.1",
        "port": 0,
        "tls_cert": str(cert),
        "tls_key": str(key),
        "credentials_file": settings_.credentials_file,
        "hash_iterations": settings_.hash_iterations,
    }
    section.update(overrides)
    return section


def _client_ssl(cert: Path) -> ssl.SSLContext:
    return ssl.create_default_context(cafile=str(cert))


_GET_ROUTES = (
    "/",
    "/api/state",
    "/api/health",
    "/api/estimate",
    "/api/bays",
    "/api/model",
    "/index.html",
    "/static/index.html",
    "/no/such/path",
)
_POST_ROUTES = (
    "/api/mode",
    "/api/setpoint",
    "/api/pwm",
    "/api/preset",
    "/api/auto",
    "/api/limit",
    "/api/bay",
    "/api/ident",
    "/api/in/smart",
)


@needs_bind
def test_every_route_needs_auth_over_tls(cfg: MpcConfig, tls_files: tuple[Path, Path]) -> None:
    cert, _key = tls_files
    app_cfg = AppConfig(mpc=cfg, http=_http_section(tls_files, auth_fail_limit=0))
    settings_ = HttpSettings.from_section(app_cfg.http)
    wrong = (
        {},
        {"Authorization": basic_header(TEST_USER, "wrong")},
        {"Authorization": basic_header("nobody", TEST_PASSWORD)},
        {"Authorization": "Bearer " + TEST_PASSWORD},
    )
    good = {"Authorization": basic_header()}

    async def scenario() -> None:
        runner = await run_http(Supervisor(cfg), app_cfg)
        try:
            host, port = runner.addresses[0][:2]
            base = f"https://{host}:{port}"
            async with aiohttp.ClientSession() as session:
                for method, routes in (("GET", _GET_ROUTES), ("POST", _POST_ROUTES)):
                    for path in routes:
                        for headers in wrong:
                            async with session.request(
                                method, base + path, headers=headers, ssl=_client_ssl(cert)
                            ) as resp:
                                assert resp.status == 401, (method, path, headers)
                                challenge = resp.headers["WWW-Authenticate"]
                                assert challenge.startswith("Basic ")
                                assert f'realm="{settings_.realm}"' in challenge
                        async with session.request(
                            method, base + path, headers=good, json={}, ssl=_client_ssl(cert)
                        ) as resp:
                            assert resp.status not in (401, 429) and resp.status < 500, (
                                method,
                                path,
                                resp.status,
                            )
                for path in ("/", "/api/state", "/api/health"):
                    async with session.get(base + path, headers=good, ssl=_client_ssl(cert)) as r:
                        assert r.status == 200
                async with session.post(
                    base + "/api/mode", headers=good, json={"mode": "auto"}, ssl=_client_ssl(cert)
                ) as resp:
                    assert resp.status == 200 and await resp.json() == {"ok": True}
        finally:
            await runner.cleanup()

    asyncio.run(scenario())


@needs_bind
def test_plain_http_is_refused_and_backoff_answers_429(
    cfg: MpcConfig, tls_files: tuple[Path, Path]
) -> None:
    cert, _key = tls_files
    app_cfg = AppConfig(mpc=cfg, http=_http_section(tls_files))
    settings_ = HttpSettings.from_section(app_cfg.http)

    async def scenario() -> None:
        runner = await run_http(Supervisor(cfg), app_cfg)
        try:
            host, port = runner.addresses[0][:2]

            def plain() -> bytes:
                with socket.create_connection((host, port), timeout=5) as sock:
                    sock.sendall(
                        b"GET /api/health HTTP/1.1\r\nHost: x\r\nAuthorization: "
                        + basic_header().encode()
                        + b"\r\n\r\n"
                    )
                    chunks = []
                    try:
                        while chunk := sock.recv(4096):
                            chunks.append(chunk)
                    except OSError:
                        pass
                    return b"".join(chunks)

            reply = await asyncio.to_thread(plain)
            assert not reply.startswith(b"HTTP/")
            async with aiohttp.ClientSession() as session:
                with pytest.raises(aiohttp.ClientError):
                    async with session.get(
                        f"http://{host}:{port}/api/health",
                        headers={"Authorization": basic_header()},
                    ) as resp:
                        await resp.read()
                wrong = {"Authorization": basic_header(TEST_USER, "wrong")}
                base = f"https://{host}:{port}/api/health"
                for _ in range(settings_.auth_fail_limit):
                    async with session.get(base, headers=wrong, ssl=_client_ssl(cert)) as resp:
                        assert resp.status == 401
                async with session.get(
                    base, headers={"Authorization": basic_header()}, ssl=_client_ssl(cert)
                ) as resp:
                    assert resp.status == 429
                    assert int(resp.headers["Retry-After"]) >= 1
        finally:
            await runner.cleanup()

    asyncio.run(scenario())


@needs_bind
def test_http_service_serves_over_tls_and_stops(
    cfg: MpcConfig, tls_files: tuple[Path, Path]
) -> None:
    cert, _key = tls_files
    service = HttpService(Supervisor(cfg), AppConfig(mpc=cfg, http=_http_section(tls_files)))
    assert service.start(timeout_s=10.0) is True
    try:
        host, port = service.addresses[0][:2]
        request = urllib.request.Request(
            f"https://{host}:{port}/api/health", headers={"Authorization": basic_header()}
        )
        with urllib.request.urlopen(request, timeout=5, context=_client_ssl(cert)) as resp:
            assert json.loads(resp.read())["solver"] == "fault"
        with pytest.raises(urllib.error.HTTPError) as err:
            urllib.request.urlopen(
                f"https://{host}:{port}/api/health", timeout=5, context=_client_ssl(cert)
            )
        assert err.value.code == 401
    finally:
        service.stop()
    assert not service.running


def _refusal_cases(tmp_path: Path, tls_files: tuple[Path, Path]) -> list[tuple[dict, str]]:
    empty = tmp_path / "empty-users"
    empty.write_text("# no users yet\n")
    os.chmod(empty, 0o640)
    loose = write_credentials(tmp_path / "loose-users")
    os.chmod(loose, 0o644)
    return [
        (_http_section(tls_files, tls_cert=str(tmp_path / "no-cert.pem")), "TLS certificate"),
        (_http_section(tls_files, tls_key=str(tmp_path / "no-key.pem")), "TLS key"),
        (_http_section(tls_files, credentials_file=str(tmp_path / "none")), "credentials file"),
        (_http_section(tls_files, credentials_file=str(empty)), "no users"),
        (_http_section(tls_files, credentials_file=str(loose)), "chmod 0640"),
        (_http_section(tls_files, tls="yes"), "unknown key"),
        (_http_section(tls_files, port="https"), "http.port"),
    ]


def test_http_service_refuses_to_start_without_tls_or_credentials(
    cfg: MpcConfig,
    tmp_path: Path,
    tls_files: tuple[Path, Path],
    caplog: pytest.LogCaptureFixture,
) -> None:
    for section, message in _refusal_cases(tmp_path, tls_files):
        caplog.clear()
        service = HttpService(Supervisor(cfg), AppConfig(mpc=cfg, http=section))
        with caplog.at_level(logging.ERROR, logger="aqua_bridge.publishers.runtime"):
            assert service.start(timeout_s=10.0) is False
        assert service.error is not None and message in service.error, (message, service.error)
        assert not service.running and service.addresses == []
        assert any(
            "not started" in r.getMessage() and message in r.getMessage() for r in caplog.records
        )
        service.stop()


def test_create_app_requires_an_authenticator(cfg: MpcConfig) -> None:
    with pytest.raises(TypeError):
        create_app(Supervisor(cfg), auth=None)  # type: ignore[arg-type]


_header_text = st.text(alphabet=st.characters(min_codepoint=0x20, max_codepoint=0x7E), max_size=60)
_headers = st.one_of(
    _header_text,
    _header_text.map(lambda t: "Basic " + t),
    st.binary(max_size=48).map(_basic_raw),
    st.tuples(st.sampled_from([TEST_USER, "nobody", ""]), _header_text).map(
        lambda up: basic_header(*up) if up[0] else _basic_raw(b":" + up[1].encode())
    ),
)


_FUZZ_CFG = load_config(REPO_ROOT / "config.example.yaml").mpc


@needs_bind
@pytest.mark.fuzzy
@settings(max_examples=40)
@given(
    headers=st.lists(_headers, min_size=1, max_size=8),
    path=st.sampled_from(_GET_ROUTES + _POST_ROUTES),
)
def test_fuzz_authorization_headers_never_5xx(headers: list[str], path: str) -> None:
    surface = Supervisor(_FUZZ_CFG)

    async def scenario() -> None:
        client = TestClient(TestServer(create_app(surface, auth=make_authenticator())))
        await client.start_server()
        try:
            for header in headers:
                method = "POST" if path in _POST_ROUTES else "GET"
                resp = await client.request(method, path, headers={"Authorization": header})
                assert resp.status in (401, 429), (header, resp.status)
                await resp.read()
        finally:
            await client.close()

    asyncio.run(scenario())


# ---------------------------------------------------------------------------
# tools/http_user.py
# ---------------------------------------------------------------------------


def _tool():
    import sys

    tools = str(REPO_ROOT / "tools")
    if tools not in sys.path:
        sys.path.insert(0, tools)
    import http_user

    return http_user


def _tool_config(tmp_path: Path, creds: Path) -> Path:
    data = yaml.safe_load((REPO_ROOT / "config.example.yaml").read_text())
    data["http"]["credentials_file"] = str(creds)
    data["http"]["hash_iterations"] = TEST_ITERATIONS
    conf = tmp_path / "config.yaml"
    conf.write_text(yaml.safe_dump(data))
    return conf


def test_http_user_tool_creates_and_updates_users(tmp_path: Path, capsys) -> None:
    import io

    tool = _tool()
    creds = tmp_path / "etc" / "http-users"
    conf = _tool_config(tmp_path, creds)
    rc = tool.main(["--config", str(conf), "--stdin", "alice"], stdin=io.StringIO("s3cret\n"))
    assert rc == 0
    assert oct(os.stat(creds).st_mode & 0o777) == oct(tool.FILE_MODE)
    users = load_credentials(creds)
    assert parse_hash(users["alice"])[0] == TEST_ITERATIONS
    assert verify_password("s3cret", users["alice"])
    creds.write_text("# admins\n" + creds.read_text())
    os.chmod(creds, 0o640)
    rc = tool.main(["--config", str(conf), "--stdin", "bob"], stdin=io.StringIO("pw-b\r\n"))
    assert rc == 0
    rc = tool.main(["--config", str(conf), "--stdin", "alice"], stdin=io.StringIO("new pw\n"))
    assert rc == 0
    text = creds.read_text()
    assert text.startswith("# admins\n")
    users = load_credentials(creds)
    assert set(users) == {"alice", "bob"}
    assert verify_password("new pw", users["alice"]) and verify_password("pw-b", users["bob"])
    # The daemon's authenticator accepts what the tool wrote.
    auth = BasicAuthenticator(
        HttpSettings.from_section(load_config(conf).section("http")), clock=FakeClock()
    )
    assert auth.verify(basic_header("bob", "pw-b"), "c").ok
    capsys.readouterr()


def test_http_user_tool_refusals(tmp_path: Path, monkeypatch, capsys) -> None:
    import io

    tool = _tool()
    creds = tmp_path / "http-users"
    conf = _tool_config(tmp_path, creds)
    # No password option on the command line at all.
    with pytest.raises(SystemExit):
        tool.build_parser().parse_args(["alice", "--password", "x"])
    assert tool.main(["--config", str(conf), "--stdin", "alice"], stdin=io.StringIO("\n")) == 1
    assert "empty password" in capsys.readouterr().err
    assert tool.main(["--config", str(conf), "--stdin", "bad name"], stdin=io.StringIO("x\n")) == 1
    assert "invalid user name" in capsys.readouterr().err
    assert not creds.exists()
    creds.write_text("garbage\n")
    assert tool.main(["--config", str(conf), "--stdin", "alice"], stdin=io.StringIO("x\n")) == 1
    assert "fix the file by hand" in capsys.readouterr().err
    assert creds.read_text() == "garbage\n"  # never clobbered
    answers = iter(["one", "two"])
    monkeypatch.setattr(tool.getpass, "getpass", lambda _prompt="": next(answers))
    assert tool.main(["--config", str(conf), "--file", str(tmp_path / "f"), "alice"]) == 1
    assert "do not match" in capsys.readouterr().err
    answers = iter(["same", "same"])
    assert tool.main(["--config", str(conf), "--file", str(tmp_path / "f"), "alice"]) == 0
    assert verify_password("same", load_credentials(tmp_path / "f")["alice"])
