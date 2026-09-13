"""Stdlib-only ``sd_notify`` (PROJECT.md section 9).

systemd hands the service a datagram socket path in ``$NOTIFY_SOCKET``.
The daemon sends ``READY=1`` once it is running, ``WATCHDOG=1`` on every
loop (period shorter than ``WatchdogSec=``) and ``STOPPING=1`` from its
SIGTERM handler after writing ``fallback_pwm``.

* ``$NOTIFY_SOCKET`` unset -> every call is a silent no-op (development,
  tests, ``--source sim`` on a laptop).
* A leading ``@`` selects the Linux abstract namespace and is replaced by
  a NUL byte.
* Nothing here may raise into the control loop: every socket error is
  swallowed and reported as ``False``.
"""

from __future__ import annotations

import contextlib
import os
import socket
from collections.abc import Callable, Mapping

__all__ = [
    "NullNotifier",
    "SdNotifier",
    "notify_ready",
    "notify_stopping",
    "notify_watchdog",
    "resolve_notify_socket",
    "sd_notify",
]

SocketFactory = Callable[[], socket.socket]


def resolve_notify_socket(env: Mapping[str, str] | None = None) -> str | None:
    """The address ``sd_notify`` sends to, or ``None`` when not under systemd.

    ``@name`` (abstract namespace) becomes ``"\\0name"``; a filesystem path is
    returned as is. Anything else (relative path, empty string) is ignored,
    mirroring ``sd_notify(3)``.
    """
    env = os.environ if env is None else env
    raw = env.get("NOTIFY_SOCKET", "")
    if not raw:
        return None
    if raw.startswith("@"):
        return "\0" + raw[1:]
    if raw.startswith("/"):
        return raw
    return None


def _default_socket() -> socket.socket:
    return socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)


def sd_notify(
    state: str,
    *,
    env: Mapping[str, str] | None = None,
    socket_factory: SocketFactory | None = None,
) -> bool:
    """Send one ``sd_notify`` message; ``True`` if it was handed to the kernel.

    ``state`` is the raw payload (``"READY=1"``, ``"WATCHDOG=1"``, several
    lines joined by ``"\\n"``). Returns ``False`` -- never raises -- when
    ``$NOTIFY_SOCKET`` is unset or the send fails for any reason.
    """
    address = resolve_notify_socket(env)
    if address is None:
        return False
    if not hasattr(socket, "AF_UNIX"):
        return False
    payload = state.encode("utf-8")
    try:
        sock = (socket_factory or _default_socket)()
    except OSError:
        return False
    try:
        sock.sendto(payload, address)
    except (OSError, ValueError):
        return False
    finally:
        with contextlib.suppress(OSError):
            sock.close()
    return True


def notify_ready(**kw: object) -> bool:
    """``READY=1`` -- required once under ``Type=notify``."""
    return sd_notify("READY=1", **kw)  # type: ignore[arg-type]


def notify_watchdog(**kw: object) -> bool:
    """``WATCHDOG=1`` -- every loop; period must stay below ``WatchdogSec=``."""
    return sd_notify("WATCHDOG=1", **kw)  # type: ignore[arg-type]


def notify_stopping(**kw: object) -> bool:
    """``STOPPING=1`` -- from the SIGTERM handler, after ``fallback_pwm`` is written."""
    return sd_notify("STOPPING=1", **kw)  # type: ignore[arg-type]


class SdNotifier:
    """Object form of the three notifications, for injection into the loop.

    ``env`` / ``socket_factory`` are resolved on every call, so a test can
    point the notifier at a temporary socket without touching ``os.environ``.
    """

    def __init__(
        self,
        *,
        env: Mapping[str, str] | None = None,
        socket_factory: SocketFactory | None = None,
    ) -> None:
        self._env = env
        self._factory = socket_factory

    @property
    def enabled(self) -> bool:
        """``True`` when ``$NOTIFY_SOCKET`` names a usable address."""
        return resolve_notify_socket(self._env) is not None

    def _send(self, state: str) -> bool:
        return sd_notify(state, env=self._env, socket_factory=self._factory)

    def ready(self) -> bool:
        return self._send("READY=1")

    def watchdog(self) -> bool:
        return self._send("WATCHDOG=1")

    def stopping(self) -> bool:
        return self._send("STOPPING=1")


class NullNotifier:
    """Notifier that does nothing; the default when no socket is configured."""

    enabled = False

    def ready(self) -> bool:
        return False

    def watchdog(self) -> bool:
        return False

    def stopping(self) -> bool:
        return False
