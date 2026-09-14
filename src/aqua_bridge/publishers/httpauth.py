"""HTTPS and HTTP basic auth for the API and the page (PROJECT.md section 6).

Everything here is stdlib only (no aiohttp), so ``tools/http_user.py`` can use
it without the HTTP stack:

* :class:`HttpSettings` -- the ``http:`` config section, parsed and validated.
  It is validated when the HTTP service starts, not when ``config.yaml`` is
  loaded: a bad ``http:`` section keeps the API off and never stops the fans.
* The credentials file: one ``user:hash`` line per user (``#`` comments and
  blank lines allowed). ``hash`` is ``pbkdf2_sha256$<iterations>$<salt>$<key>``
  with the salt and derived key in unpadded URL-safe base64; every user has
  their own random salt and the iteration count travels with the hash, so
  raising ``http.hash_iterations`` only affects users written afterwards.
  :func:`hash_password`, :func:`verify_password`, :func:`parse_credentials`,
  :func:`update_credentials_text`, :func:`load_credentials`.
* :func:`build_ssl_context` -- the TLS server context (TLS 1.2 or newer) from
  ``http.tls_cert`` / ``http.tls_key``.
* :class:`BasicAuthenticator` -- checks an ``Authorization`` header: a cache of
  verified credentials (``http.auth_cache_s``, so the page's 2 s poll does not
  run PBKDF2 each time), a per-client backoff after ``http.auth_fail_limit``
  consecutive failures (``http.auth_backoff_s`` doubling up to
  ``http.auth_backoff_max_s``), a global limit on uncached password checks
  (``http.auth_verify_max`` per ``http.auth_verify_window_s`` and at most
  ``http.auth_verify_pending_max`` admitted at once, across every client, so
  requests from many source addresses cannot keep the single core busy), one
  worker thread at a lowered priority (``http.auth_verify_nice``) for the
  checks, and a reload of the credentials file whenever it changes on disk (a
  missing or invalid file then denies everyone).

The private files (key, credentials) must not be readable by others nor
writable by the group: ``0640 root:<service user>`` is the intended mode.
"""

from __future__ import annotations

import base64
import binascii
import hashlib
import hmac
import logging
import math
import os
import re
import secrets
import ssl
import stat
import sys
import threading
import time
from collections import deque
from collections.abc import Callable, Mapping
from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass, field, fields
from pathlib import Path
from typing import Any

__all__ = [
    "HASH_SCHEME",
    "MIN_HASH_ITERATIONS",
    "MAX_NICE",
    "AuthDecision",
    "AuthTicket",
    "BasicAuthenticator",
    "HttpSettings",
    "HttpSetupError",
    "build_ssl_context",
    "check_private_file",
    "hash_password",
    "load_credentials",
    "lower_thread_priority",
    "parse_authorization",
    "parse_credentials",
    "parse_hash",
    "update_credentials_text",
    "valid_username",
    "verify_password",
]

_LOG = logging.getLogger(__name__)

HASH_SCHEME = "pbkdf2_sha256"
# Format constants of the hash string, not operator settings.
_SALT_BYTES = 16
_KEY_BYTES = 32
# Validation floor for http.hash_iterations (a smaller count is a mistake).
MIN_HASH_ITERATIONS = 1000
# The largest nice value Linux accepts (validation bound of http.auth_verify_nice).
MAX_NICE = 19

_USERNAME = re.compile(r"^[A-Za-z0-9._@+-]{1,64}$")
# A realm is quoted into WWW-Authenticate: printable ASCII without quote or backslash.
_REALM = re.compile(r"^[\x20-\x21\x23-\x5b\x5d-\x7e]{1,64}$")


class HttpSetupError(ValueError):
    """The HTTP service cannot start: bad settings, TLS files or credentials."""


# ---------------------------------------------------------------------------
# Settings
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class HttpSettings:
    """The ``http:`` section. Every default is documented in PROJECT.md section 6."""

    #: Must be ``true`` for the daemon to start the service.
    enabled: bool = False
    #: Listen address.
    bind: str = "0.0.0.0"
    #: TLS port (there is no plain-HTTP listener).
    port: int = 8443
    #: PEM certificate (chain) served to clients.
    tls_cert: str = "/etc/aqua-bridge/tls/cert.pem"
    #: PEM private key of ``tls_cert``; mode 0640 root:<service user>.
    tls_key: str = "/etc/aqua-bridge/tls/key.pem"
    #: ``user:hash`` lines; mode 0640 root:<service user>.
    credentials_file: str = "/etc/aqua-bridge/http-users"
    #: Basic-auth realm shown by browsers.
    realm: str = "aqua-bridge"
    #: PBKDF2-HMAC-SHA256 iterations for hashes written by tools/http_user.py.
    hash_iterations: int = 100_000
    #: Seconds a verified user:password stays cached (0 = verify every request).
    auth_cache_s: float = 300.0
    #: Consecutive failed logins from one client before the backoff starts (0 = no backoff).
    auth_fail_limit: int = 5
    #: First backoff after the limit, doubled with every further failure.
    auth_backoff_s: float = 1.0
    #: Longest backoff; a client's failure count is forgotten once this long has passed
    #: without a failure after its backoff ended.
    auth_backoff_max_s: float = 300.0
    #: Uncached password checks (PBKDF2) admitted per ``auth_verify_window_s`` across
    #: every client; beyond it a request that needs one gets 429 (0 = no window limit).
    auth_verify_max: int = 6
    #: The sliding window of ``auth_verify_max``, seconds (> 0).
    auth_verify_window_s: float = 60.0
    #: Uncached checks admitted and unfinished at once (one runs, the rest wait); beyond
    #: it 429 (>= 1).
    auth_verify_pending_max: int = 2
    #: Added to the nice value of the thread that runs the checks (Linux; 0 = unchanged).
    auth_verify_nice: int = 10

    @classmethod
    def from_section(cls, section: Mapping[str, Any] | None) -> HttpSettings:
        """Parse and validate the raw ``http:`` mapping. Raises :class:`HttpSetupError`."""
        data = dict(section or {})
        known = {f.name for f in fields(cls)}
        unknown = sorted(str(k) for k in data if k not in known)
        if unknown:
            raise HttpSetupError(f"http: unknown key(s): {', '.join(unknown)}")
        defaults = cls()
        values: dict[str, Any] = {}

        def get(name: str) -> Any:
            return data.get(name, getattr(defaults, name))

        enabled = get("enabled")
        if not isinstance(enabled, bool):
            raise HttpSetupError("http.enabled must be true or false")
        values["enabled"] = enabled
        for name in ("bind", "tls_cert", "tls_key", "credentials_file"):
            value = get(name)
            if not isinstance(value, str) or not value.strip():
                raise HttpSetupError(f"http.{name} must be a non-empty string")
            values[name] = value
        realm = get("realm")
        if not isinstance(realm, str) or not _REALM.match(realm):
            raise HttpSetupError(
                "http.realm must be 1-64 printable ASCII characters without '\"' or '\\'"
            )
        values["realm"] = realm
        port = get("port")
        if isinstance(port, bool) or not isinstance(port, int) or not 0 <= port <= 65535:
            raise HttpSetupError("http.port must be an integer in [0, 65535]")
        values["port"] = port
        iterations = get("hash_iterations")
        if (
            isinstance(iterations, bool)
            or not isinstance(iterations, int)
            or iterations < MIN_HASH_ITERATIONS
        ):
            raise HttpSetupError(
                f"http.hash_iterations must be an integer >= {MIN_HASH_ITERATIONS}"
            )
        values["hash_iterations"] = iterations
        fail_limit = get("auth_fail_limit")
        if isinstance(fail_limit, bool) or not isinstance(fail_limit, int) or fail_limit < 0:
            raise HttpSetupError("http.auth_fail_limit must be an integer >= 0")
        values["auth_fail_limit"] = fail_limit
        for name, low, high in (
            ("auth_verify_max", 0, None),
            ("auth_verify_pending_max", 1, None),
            ("auth_verify_nice", 0, MAX_NICE),
        ):
            value = get(name)
            if (
                isinstance(value, bool)
                or not isinstance(value, int)
                or value < low
                or (high is not None and value > high)
            ):
                bound = f">= {low}" if high is None else f"in [{low}, {high}]"
                raise HttpSetupError(f"http.{name} must be an integer {bound}")
            values[name] = value
        window = get("auth_verify_window_s")
        if (
            isinstance(window, bool)
            or not isinstance(window, int | float)
            or not math.isfinite(float(window))
            or float(window) <= 0.0
        ):
            raise HttpSetupError("http.auth_verify_window_s must be a finite number > 0")
        values["auth_verify_window_s"] = float(window)
        for name in ("auth_cache_s", "auth_backoff_s", "auth_backoff_max_s"):
            value = get(name)
            if isinstance(value, bool) or not isinstance(value, int | float):
                raise HttpSetupError(f"http.{name} must be a number")
            value = float(value)
            if not math.isfinite(value) or value < 0.0:
                raise HttpSetupError(f"http.{name} must be a finite number >= 0")
            values[name] = value
        if values["auth_backoff_max_s"] < values["auth_backoff_s"]:
            raise HttpSetupError("http.auth_backoff_max_s must be >= http.auth_backoff_s")
        return cls(**values)


# ---------------------------------------------------------------------------
# Hashes and the credentials file
# ---------------------------------------------------------------------------


def _b64encode(raw: bytes) -> str:
    return base64.urlsafe_b64encode(raw).rstrip(b"=").decode("ascii")


def _b64decode(text: str) -> bytes:
    if not re.fullmatch(r"[A-Za-z0-9_-]+", text):
        raise ValueError("not unpadded URL-safe base64")
    return base64.urlsafe_b64decode(text + "=" * (-len(text) % 4))


def _derive(password: str, salt: bytes, iterations: int) -> bytes:
    return hashlib.pbkdf2_hmac(
        "sha256", password.encode("utf-8"), salt, iterations, dklen=_KEY_BYTES
    )


def hash_password(password: str, iterations: int, *, salt: bytes | None = None) -> str:
    """``pbkdf2_sha256$<iterations>$<salt>$<key>`` with a fresh random salt."""
    if isinstance(iterations, bool) or not isinstance(iterations, int) or iterations < 1:
        raise ValueError("iterations must be a positive integer")
    if salt is None:
        salt = secrets.token_bytes(_SALT_BYTES)
    key = _derive(password, salt, iterations)
    return f"{HASH_SCHEME}${iterations}${_b64encode(salt)}${_b64encode(key)}"


def parse_hash(encoded: str) -> tuple[int, bytes, bytes]:
    """``(iterations, salt, key)`` of a hash string. Raises :class:`ValueError`."""
    parts = encoded.split("$")
    if len(parts) != 4 or parts[0] != HASH_SCHEME:
        raise ValueError(f"hash is not {HASH_SCHEME}$<iterations>$<salt>$<key>")
    if not parts[1].isdigit() or parts[1].startswith("0"):
        raise ValueError("hash iteration count is not a positive integer")
    iterations = int(parts[1])
    salt = _b64decode(parts[2])
    key = _b64decode(parts[3])
    if len(salt) < _SALT_BYTES:
        raise ValueError(f"hash salt is shorter than {_SALT_BYTES} bytes")
    if len(key) != _KEY_BYTES:
        raise ValueError(f"hash key is not {_KEY_BYTES} bytes")
    return iterations, salt, key


def verify_password(password: str, encoded: str) -> bool:
    """Constant-time check of ``password`` against a hash string (``False`` if malformed)."""
    try:
        iterations, salt, key = parse_hash(encoded)
    except ValueError:
        return False
    return hmac.compare_digest(_derive(password, salt, iterations), key)


def valid_username(user: str) -> bool:
    return isinstance(user, str) and bool(_USERNAME.match(user))


def _credential_lines(text: str) -> list[tuple[int, str, str | None, str | None]]:
    """``(line number, raw line, user | None, hash | None)``; ``None`` for comments/blank."""
    out = []
    for number, raw in enumerate(text.splitlines(), start=1):
        line = raw.strip()
        if not line or line.startswith("#"):
            out.append((number, raw, None, None))
            continue
        user, sep, encoded = line.partition(":")
        if not sep:
            raise ValueError(f"line {number}: expected user:hash")
        out.append((number, raw, user, encoded))
    return out


def parse_credentials(text: str) -> dict[str, str]:
    """``{user: hash}`` of a credentials file's text. Raises :class:`ValueError`."""
    users: dict[str, str] = {}
    for number, _raw, user, encoded in _credential_lines(text):
        if user is None or encoded is None:
            continue
        if not valid_username(user):
            raise ValueError(f"line {number}: invalid user name (allowed: A-Z a-z 0-9 . _ @ + -)")
        if user in users:
            raise ValueError(f"line {number}: duplicate user {user!r}")
        try:
            parse_hash(encoded)
        except ValueError as exc:
            raise ValueError(f"line {number} ({user}): {exc}") from None
        users[user] = encoded
    return users


def update_credentials_text(text: str, user: str, encoded: str) -> str:
    """``text`` with ``user``'s line replaced (or appended); comments are kept."""
    if not valid_username(user):
        raise ValueError(f"invalid user name {user!r} (allowed: A-Z a-z 0-9 . _ @ + -)")
    parse_hash(encoded)
    parse_credentials(text)  # refuse to rewrite a file that is already invalid
    lines: list[str] = []
    replaced = False
    for _number, raw, line_user, _hash in _credential_lines(text):
        if line_user == user:
            lines.append(f"{user}:{encoded}")
            replaced = True
        else:
            lines.append(raw)
    if not replaced:
        lines.append(f"{user}:{encoded}")
    result = "\n".join(lines) + "\n"
    parse_credentials(result)
    return result


def check_private_file(path: str | os.PathLike[str], what: str) -> os.stat_result:
    """A regular file owned by root or this process's user, not readable by others and
    not writable by the group or others."""
    try:
        st = os.stat(path)
    except OSError as exc:
        raise HttpSetupError(f"{what} {path}: {exc.strerror or exc}") from exc
    if not stat.S_ISREG(st.st_mode):
        raise HttpSetupError(f"{what} {path} is not a regular file")
    if st.st_uid not in (0, os.getuid()):
        raise HttpSetupError(
            f"{what} {path} belongs to uid {st.st_uid}; its owner must be root or the "
            "service user (chown root:<service user>)"
        )
    mode = stat.S_IMODE(st.st_mode)
    if mode & 0o027:
        raise HttpSetupError(
            f"{what} {path} has mode {mode:04o}; it must not be readable by others or "
            "writable by the group (chmod 0640, owner root, group the service user)"
        )
    return st


def load_credentials(path: str | os.PathLike[str]) -> dict[str, str]:
    """Read, permission-check and parse the credentials file. Raises :class:`HttpSetupError`."""
    check_private_file(path, "credentials file")
    try:
        text = Path(path).read_text(encoding="utf-8")
    except (OSError, UnicodeDecodeError) as exc:
        raise HttpSetupError(f"credentials file {path}: {exc}") from exc
    try:
        users = parse_credentials(text)
    except ValueError as exc:
        raise HttpSetupError(f"credentials file {path}: {exc}") from exc
    if not users:
        raise HttpSetupError(
            f"credentials file {path} has no users; create one with tools/http_user.py"
        )
    return users


# ---------------------------------------------------------------------------
# TLS
# ---------------------------------------------------------------------------


def build_ssl_context(settings: HttpSettings) -> ssl.SSLContext:
    """Server context from ``tls_cert`` / ``tls_key``. Raises :class:`HttpSetupError`."""
    cert = Path(settings.tls_cert)
    try:
        if not cert.is_file():
            raise HttpSetupError(f"TLS certificate {cert} does not exist or is not a file")
    except OSError as exc:
        raise HttpSetupError(f"TLS certificate {cert}: {exc}") from exc
    check_private_file(settings.tls_key, "TLS key")
    ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
    ctx.minimum_version = ssl.TLSVersion.TLSv1_2
    try:
        ctx.load_cert_chain(certfile=str(cert), keyfile=settings.tls_key)
    except (OSError, ssl.SSLError) as exc:
        raise HttpSetupError(
            f"cannot load TLS certificate {cert} with key {settings.tls_key}: {exc}"
        ) from exc
    return ctx


# ---------------------------------------------------------------------------
# Authorization header and the authenticator
# ---------------------------------------------------------------------------


def parse_authorization(header: str | None) -> tuple[str, str] | None:
    """``(user, password)`` of a ``Basic`` header, ``None`` when absent or malformed."""
    if not isinstance(header, str):
        return None
    scheme, _, token = header.strip().partition(" ")
    if scheme.lower() != "basic":
        return None
    token = token.strip()
    if not token or not token.isascii():
        return None
    try:
        raw = base64.b64decode(token, validate=True)
        text = raw.decode("utf-8")
    except (binascii.Error, ValueError):
        return None
    user, sep, password = text.partition(":")
    if not sep or not valid_username(user):
        return None
    return user, password


@dataclass(frozen=True)
class AuthDecision:
    """``ok`` with ``user``; otherwise ``retry_after_s`` is set while the client backs
    off or the global limit on uncached checks refuses (both answer 429)."""

    ok: bool
    user: str | None = None
    retry_after_s: float | None = None


@dataclass(frozen=True, eq=False)
class AuthTicket:
    """An admitted uncached check: pass it to :meth:`BasicAuthenticator.check` once."""

    user: str
    password: str = field(repr=False)
    client: str = ""


def lower_thread_priority(increment: int) -> None:
    """Add ``increment`` to the calling thread's nice value (capped at :data:`MAX_NICE`).

    Linux only: there ``setpriority(PRIO_PROCESS, <thread id>)`` applies to that one
    thread. Elsewhere, and on an error (logged), the priority is left as it is.
    """
    if increment <= 0 or not sys.platform.startswith("linux"):
        return
    try:
        tid = threading.get_native_id()
        current = os.getpriority(os.PRIO_PROCESS, tid)
        os.setpriority(os.PRIO_PROCESS, tid, min(MAX_NICE, current + increment))
    except OSError as exc:
        _LOG.warning("http: cannot lower the password-check thread's priority: %s", exc)


class BasicAuthenticator:
    """Basic-auth checks with a cache, per-client backoff, a global limit on
    uncached checks and credentials reload.

    Thread-safe. :meth:`fast` answers without PBKDF2 (a cache hit, a missing or
    malformed header, a client backing off) or returns ``None``. :meth:`begin`
    adds the global admission of uncached checks: a decision (the ``fast``
    answers, or 429 when ``http.auth_verify_max`` checks already started within
    ``http.auth_verify_window_s`` or ``http.auth_verify_pending_max`` are admitted
    and unfinished) or an :class:`AuthTicket`. :meth:`check` runs a ticket's
    PBKDF2 (serialised, one at a time) and releases its slot; call it exactly
    once per ticket. The server runs it on :attr:`executor`, one worker thread
    whose nice value is raised by ``http.auth_verify_nice``. :meth:`verify` is
    :meth:`begin` plus :meth:`check` in the calling thread.
    """

    def __init__(
        self,
        settings: HttpSettings,
        *,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        self.settings = settings
        self._clock = clock
        self._lock = threading.Lock()
        self._verify_lock = threading.Lock()
        self._cache_key = secrets.token_bytes(32)
        self._cache: dict[bytes, tuple[str, float]] = {}
        self._failures: dict[str, tuple[int, float]] = {}
        # Start times of the uncached checks admitted within the window, oldest first.
        self._admitted: deque[float] = deque()
        self._pending = 0
        self._last_check_s = 0.0
        # Refusals by the global limit not logged yet, and when the last line was logged.
        self._refused = 0
        self._refused_logged_at: float | None = None
        self._executor: ThreadPoolExecutor | None = None
        self._dummy_hash = ""
        self._signature: tuple[int, int, int, int] | None = None
        self._users: dict[str, str] = {}
        self._reload_error: str | None = None
        # Refuse to start without a valid file: raises HttpSetupError.
        self._set_users_locked(load_credentials(settings.credentials_file))
        self._signature = self._stat_signature()

    @property
    def executor(self) -> ThreadPoolExecutor:
        """The one worker thread for :meth:`check` (started on first use)."""
        with self._lock:
            if self._executor is None:
                self._executor = ThreadPoolExecutor(
                    max_workers=1,
                    thread_name_prefix="aqua-bridge-auth",
                    initializer=lower_thread_priority,
                    initargs=(self.settings.auth_verify_nice,),
                )
            return self._executor

    def close(self) -> None:
        """Let the worker thread exit once its queue is done (a later use starts a new one)."""
        with self._lock:
            executor, self._executor = self._executor, None
        if executor is not None:
            executor.shutdown(wait=False)

    @property
    def users(self) -> tuple[str, ...]:
        with self._lock:
            return tuple(sorted(self._users))

    def _set_users_locked(self, users: dict[str, str]) -> None:
        """Install ``users`` and the stand-in hash an unknown name is checked against.

        The stand-in has a random salt and key and the highest iteration count of
        the stored hashes, so an unknown name costs one derivation, like a known
        one (``http.hash_iterations`` only governs hashes written later and may
        differ from what the file holds).
        """
        self._users = users
        iterations = max(
            (parse_hash(encoded)[0] for encoded in users.values()),
            default=self.settings.hash_iterations,
        )
        self._dummy_hash = (
            f"{HASH_SCHEME}${iterations}$"
            f"{_b64encode(secrets.token_bytes(_SALT_BYTES))}$"
            f"{_b64encode(secrets.token_bytes(_KEY_BYTES))}"
        )

    def _stat_signature(self) -> tuple[int, int, int, int] | None:
        try:
            st = os.stat(self.settings.credentials_file)
        except OSError:
            return None
        return (st.st_mtime_ns, st.st_size, st.st_ino, st.st_mode)

    def _refresh_locked(self) -> None:
        signature = self._stat_signature()
        if signature == self._signature and signature is not None:
            return
        self._signature = signature
        self._cache.clear()
        try:
            users = load_credentials(self.settings.credentials_file)
        except HttpSetupError as exc:
            self._set_users_locked({})
            message = str(exc)
            if message != self._reload_error:
                _LOG.error("http: credentials unusable, denying every request: %s", message)
            self._reload_error = message
            return
        self._set_users_locked(users)
        if self._reload_error is not None:
            _LOG.info("http: credentials file readable again (%d users)", len(self._users))
        self._reload_error = None
        _LOG.info("http: credentials reloaded (%d users)", len(self._users))

    def _cache_digest(self, user: str, password: str) -> bytes:
        msg = user.encode("utf-8") + b"\0" + password.encode("utf-8", "surrogatepass")
        return hmac.new(self._cache_key, msg, hashlib.sha256).digest()

    def _wait_s(self, count: int) -> float:
        """The backoff after ``count`` consecutive failures (0 below the limit)."""
        limit = self.settings.auth_fail_limit
        if limit <= 0 or count < limit:
            return 0.0
        return min(
            self.settings.auth_backoff_s * 2.0 ** min(count - limit, 64),
            self.settings.auth_backoff_max_s,
        )

    def _backoff_locked(self, client: str, now: float) -> float | None:
        """Seconds the client still has to wait, or ``None``."""
        entry = self._failures.get(client)
        if entry is None:
            return None
        count, last = entry
        wait = self._wait_s(count)
        if wait <= 0.0:
            return None
        remaining = last + wait - now
        return remaining if remaining > 0.0 else None

    def _record_failure_locked(self, client: str, now: float) -> None:
        # A count is forgotten auth_backoff_max_s after its backoff ended, so a
        # client retrying right when the longest wait ends stays at that wait.
        horizon = self.settings.auth_backoff_max_s
        for key, (count, last) in list(self._failures.items()):
            if now - last > self._wait_s(count) + horizon:
                del self._failures[key]
        count, _last = self._failures.get(client, (0, now))
        self._failures[client] = (count + 1, now)

    def fast(self, header: str | None, client: str) -> AuthDecision | None:
        """A decision that needs no key derivation, or ``None`` (call :meth:`verify`)."""
        now = self._clock()
        with self._lock:
            self._refresh_locked()
            wait = self._backoff_locked(client, now)
            if wait is not None:
                return AuthDecision(False, retry_after_s=wait)
            creds = parse_authorization(header)
            if creds is None:
                if header is not None:
                    self._record_failure_locked(client, now)
                return AuthDecision(False)
            if self.settings.auth_cache_s > 0.0:
                hit = self._cache.get(self._cache_digest(*creds))
                if hit is not None and hit[1] > now and hit[0] in self._users:
                    self._failures.pop(client, None)
                    return AuthDecision(True, user=creds[0])
        return None

    def _admit_locked(self, now: float) -> float | None:
        """Reserve a slot for one uncached check: ``None``, or the seconds to wait."""
        s = self.settings
        horizon = now - s.auth_verify_window_s
        while self._admitted and self._admitted[0] <= horizon:
            self._admitted.popleft()
        wait: float | None = None
        if s.auth_verify_max > 0 and len(self._admitted) >= s.auth_verify_max:
            wait = self._admitted[0] + s.auth_verify_window_s - now
        if self._pending >= s.auth_verify_pending_max:
            # The queue drains one check at a time; the last check's duration is the estimate.
            wait = max(wait or 0.0, self._pending * self._last_check_s)
        if wait is not None:
            self._refused += 1
            # At most one line per window, however long a flood lasts.
            if self._refused_logged_at is None or now - self._refused_logged_at >= (
                s.auth_verify_window_s
            ):
                _LOG.warning(
                    "http: %d request(s) refused with 429: uncached password checks over the "
                    "limit (%d per %g s, %d at once)",
                    self._refused,
                    s.auth_verify_max,
                    s.auth_verify_window_s,
                    s.auth_verify_pending_max,
                )
                self._refused_logged_at = now
                self._refused = 0
            return max(wait, 0.0)
        self._admitted.append(now)
        self._pending += 1
        return None

    def begin(self, header: str | None, client: str) -> AuthDecision | AuthTicket:
        """:meth:`fast`, then the global admission of an uncached check. Never raises."""
        decision = self.fast(header, client)
        if decision is not None:
            return decision
        creds = parse_authorization(header)
        if creds is None:  # pragma: no cover - fast() answered already
            return AuthDecision(False)
        with self._lock:
            wait = self._admit_locked(self._clock())
        if wait is not None:
            # Nothing was checked, so this is not a failure of the client.
            return AuthDecision(False, retry_after_s=wait)
        return AuthTicket(user=creds[0], password=creds[1], client=client)

    def verify(self, header: str | None, client: str) -> AuthDecision:
        """:meth:`begin` and :meth:`check` in the calling thread (may run PBKDF2). Never
        raises."""
        outcome = self.begin(header, client)
        if isinstance(outcome, AuthDecision):
            return outcome
        return self.check(outcome)

    def check(self, ticket: AuthTicket) -> AuthDecision:
        """Run an admitted check (may run PBKDF2) and release its slot. Never raises."""
        try:
            with self._verify_lock:
                started = self._clock()
                try:
                    return self._check_serialised(ticket)
                finally:
                    elapsed = self._clock() - started
                    with self._lock:
                        self._last_check_s = max(0.0, elapsed)
        finally:
            with self._lock:
                self._pending -= 1

    def _check_serialised(self, ticket: AuthTicket) -> AuthDecision:
        """The body of :meth:`check`, under the verify lock."""
        user, password, client = ticket.user, ticket.password, ticket.client
        with self._lock:
            now = self._clock()
            wait = self._backoff_locked(client, now)
            if wait is not None:
                return AuthDecision(False, retry_after_s=wait)
            if self.settings.auth_cache_s > 0.0:
                # A check queued behind one for the same credentials needs no derivation.
                hit = self._cache.get(self._cache_digest(user, password))
                if hit is not None and hit[1] > now and hit[0] in self._users:
                    self._failures.pop(client, None)
                    return AuthDecision(True, user=user)
            encoded = self._users.get(user)
            dummy = self._dummy_hash
            signature = self._signature
        if encoded is None:
            # One derivation at the stored cost, so a probe cannot tell names apart.
            verify_password(password, dummy)
            ok = False
        else:
            ok = verify_password(password, encoded)
        now = self._clock()
        with self._lock:
            if ok and self._signature == signature and user in self._users:
                self._failures.pop(client, None)
                if self.settings.auth_cache_s > 0.0:
                    for key, (_u, expiry) in list(self._cache.items()):
                        if expiry <= now:
                            del self._cache[key]
                    self._cache[self._cache_digest(user, password)] = (
                        user,
                        now + self.settings.auth_cache_s,
                    )
                return AuthDecision(True, user=user)
            self._record_failure_locked(client, now)
            return AuthDecision(False)
