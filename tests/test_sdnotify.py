"""sd_notify: stdlib-only, no-op when unset, never raises (PROJECT.md section 9)."""

from __future__ import annotations

import contextlib
import os
import socket
import tempfile
from pathlib import Path

import pytest

from aqua_bridge.sdnotify import (
    NullNotifier,
    SdNotifier,
    notify_ready,
    notify_stopping,
    notify_watchdog,
    resolve_notify_socket,
    sd_notify,
    watchdog_seconds,
)


class FakeSocket:
    def __init__(self, *, fail: Exception | None = None) -> None:
        self.sent: list[tuple[bytes, str]] = []
        self.closed = False
        self.fail = fail

    def sendto(self, payload: bytes, address: str) -> int:
        if self.fail is not None:
            raise self.fail
        self.sent.append((payload, address))
        return len(payload)

    def close(self) -> None:
        self.closed = True


# --- address resolution --------------------------------------------------------


def test_unset_env_resolves_to_none():
    assert resolve_notify_socket({}) is None
    assert resolve_notify_socket({"NOTIFY_SOCKET": ""}) is None


def test_abstract_namespace_gets_nul_prefix():
    assert resolve_notify_socket({"NOTIFY_SOCKET": "@/org/systemd/notify"}) == (
        "\0/org/systemd/notify"
    )


def test_absolute_path_passes_through():
    assert resolve_notify_socket({"NOTIFY_SOCKET": "/run/systemd/notify"}) == "/run/systemd/notify"


def test_relative_path_is_ignored():
    assert resolve_notify_socket({"NOTIFY_SOCKET": "notify.sock"}) is None


# --- no-op when unset ------------------------------------------------------------


def test_noop_without_notify_socket_and_no_socket_created():
    created: list[FakeSocket] = []

    def factory() -> FakeSocket:
        s = FakeSocket()
        created.append(s)
        return s

    assert sd_notify("READY=1", env={}, socket_factory=factory) is False  # type: ignore[arg-type]
    assert created == []
    n = SdNotifier(env={}, socket_factory=factory)  # type: ignore[arg-type]
    assert n.enabled is False
    assert n.ready() is False and n.watchdog() is False and n.stopping() is False
    assert created == []


def test_module_functions_default_to_process_env(monkeypatch):
    monkeypatch.delenv("NOTIFY_SOCKET", raising=False)
    assert notify_ready() is False
    assert notify_watchdog() is False
    assert notify_stopping() is False


def test_null_notifier():
    n = NullNotifier()
    assert n.enabled is False
    assert (n.ready(), n.watchdog(), n.stopping()) == (False, False, False)


# --- payloads through a fake socket -----------------------------------------------


def test_payloads_and_address_with_fake_socket():
    sock = FakeSocket()
    env = {"NOTIFY_SOCKET": "@notify"}
    n = SdNotifier(env=env, socket_factory=lambda: sock)  # type: ignore[arg-type]
    assert n.enabled is True
    assert n.ready() is True
    assert n.watchdog() is True
    assert n.stopping() is True
    assert [p for p, _ in sock.sent] == [b"READY=1", b"WATCHDOG=1", b"STOPPING=1"]
    assert all(addr == "\0notify" for _, addr in sock.sent)
    assert sock.closed is True


def test_module_functions_accept_env_and_factory():
    sock = FakeSocket()
    kw = {"env": {"NOTIFY_SOCKET": "/tmp/x"}, "socket_factory": lambda: sock}
    assert notify_ready(**kw) and notify_watchdog(**kw) and notify_stopping(**kw)
    assert [p for p, _ in sock.sent] == [b"READY=1", b"WATCHDOG=1", b"STOPPING=1"]


def test_send_failure_returns_false_and_closes():
    sock = FakeSocket(fail=OSError("ECONNREFUSED"))
    n = SdNotifier(env={"NOTIFY_SOCKET": "/tmp/x"}, socket_factory=lambda: sock)  # type: ignore[arg-type]
    assert n.watchdog() is False
    assert sock.closed is True


def test_socket_creation_failure_returns_false():
    def factory() -> FakeSocket:
        raise OSError("no sockets")

    assert sd_notify("READY=1", env={"NOTIFY_SOCKET": "/tmp/x"}, socket_factory=factory) is False  # type: ignore[arg-type]


def test_dead_socket_path_returns_false_without_raising(tmp_path: Path):
    missing = str(tmp_path / "nobody-listens.sock")
    assert sd_notify("READY=1", env={"NOTIFY_SOCKET": missing}) is False


# --- real AF_UNIX datagram socket -------------------------------------------------


@pytest.fixture
def listener():
    """A bound AF_UNIX datagram socket in a short temp dir (macOS caps paths at ~104 bytes)."""
    if not hasattr(socket, "AF_UNIX"):
        pytest.skip("no AF_UNIX on this platform")
    tmp = tempfile.mkdtemp(prefix="sdn")
    path = os.path.join(tmp, "notify.sock")
    if len(path.encode()) > 100:
        pytest.skip(f"temp path too long for AF_UNIX: {path}")
    srv = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
    try:
        srv.bind(path)
    except PermissionError:
        srv.close()
        pytest.skip("sandbox forbids binding AF_UNIX sockets")
    srv.settimeout(2.0)
    try:
        yield srv, path
    finally:
        srv.close()
        with contextlib.suppress(OSError):
            os.unlink(path)
            os.rmdir(tmp)


def test_real_socket_receives_ready_watchdog_stopping(listener):
    srv, path = listener
    n = SdNotifier(env={"NOTIFY_SOCKET": path})
    assert n.enabled is True
    assert n.ready() is True
    assert srv.recv(64) == b"READY=1"
    assert n.watchdog() is True
    assert srv.recv(64) == b"WATCHDOG=1"
    assert n.stopping() is True
    assert srv.recv(64) == b"STOPPING=1"


def test_real_socket_module_functions_with_env_override(listener):
    srv, path = listener
    env = {"NOTIFY_SOCKET": path}
    assert notify_watchdog(env=env) is True
    assert srv.recv(64) == b"WATCHDOG=1"
    assert sd_notify("STATUS=hello\nWATCHDOG=1", env=env) is True
    assert srv.recv(64) == b"STATUS=hello\nWATCHDOG=1"


def test_watchdog_seconds_from_the_systemd_environment():
    assert watchdog_seconds({}) is None
    assert watchdog_seconds({"WATCHDOG_USEC": "30000000"}) == 30.0
    assert watchdog_seconds({"WATCHDOG_USEC": "30000000", "WATCHDOG_PID": "42"}, pid=42) == 30.0
    assert watchdog_seconds({"WATCHDOG_USEC": "30000000", "WATCHDOG_PID": "7"}, pid=42) is None
    for bad in ("", "0", "-5", "30s", "3.5"):
        assert watchdog_seconds({"WATCHDOG_USEC": bad}) is None
