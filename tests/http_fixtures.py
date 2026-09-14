"""Shared helpers for the HTTP suites: a credentials file, an authenticator, TLS files.

Every app built by :func:`aqua_bridge.publishers.http.create_app` needs a
:class:`~aqua_bridge.publishers.httpauth.BasicAuthenticator`; the plain-HTTP
``TestServer`` suites use :func:`make_authenticator` plus :func:`client_auth`
so their requests carry valid credentials. The TLS suites generate a
self-signed certificate with ``openssl`` (:func:`make_tls_files`).
"""

from __future__ import annotations

import atexit
import base64
import os
import shutil
import subprocess
import tempfile
from pathlib import Path
from typing import Any

from aqua_bridge.publishers.httpauth import (
    MIN_HASH_ITERATIONS,
    BasicAuthenticator,
    HttpSettings,
    hash_password,
)

# Test-only parameters, not operator settings.
TEST_USER = "tester"
TEST_PASSWORD = "correct horse battery staple"
# The validation floor keeps the suites fast; the daemon's default is far higher.
TEST_ITERATIONS = MIN_HASH_ITERATIONS

_SHARED_DIR: Path | None = None


def _shared_dir() -> Path:
    global _SHARED_DIR
    if _SHARED_DIR is None:
        _SHARED_DIR = Path(tempfile.mkdtemp(prefix="aqua-bridge-http-"))
        atexit.register(shutil.rmtree, _SHARED_DIR, True)
    return _SHARED_DIR


def write_credentials(path: Path, users: dict[str, str] | None = None) -> Path:
    """A 0640 credentials file with ``users`` (``{user: password}``)."""
    users = {TEST_USER: TEST_PASSWORD} if users is None else users
    lines = [f"{u}:{hash_password(p, TEST_ITERATIONS)}" for u, p in users.items()]
    path.write_text("# test users\n" + "".join(line + "\n" for line in lines), encoding="utf-8")
    os.chmod(path, 0o640)
    return path


def auth_settings(directory: Path | None = None, **overrides: Any) -> HttpSettings:
    """Settings whose credentials file holds :data:`TEST_USER`."""
    directory = _shared_dir() if directory is None else directory
    creds = directory / "http-users"
    if not creds.exists():
        write_credentials(creds)
    section: dict[str, Any] = {
        "credentials_file": str(creds),
        "hash_iterations": TEST_ITERATIONS,
    }
    section.update(overrides)
    return HttpSettings.from_section(section)


def make_authenticator(**overrides: Any) -> BasicAuthenticator:
    """A fresh authenticator (own cache and backoff state) over the shared test user."""
    return BasicAuthenticator(auth_settings(**overrides))


def basic_header(user: str = TEST_USER, password: str = TEST_PASSWORD) -> str:
    """An ``Authorization: Basic ...`` value."""
    token = base64.b64encode(f"{user}:{password}".encode()).decode("ascii")
    return f"Basic {token}"


def client_auth() -> dict[str, Any]:
    """``TestClient`` keyword arguments that send :data:`TEST_USER`'s credentials."""
    return {"headers": {"Authorization": basic_header()}}


def make_tls_files(directory: Path) -> tuple[Path, Path] | None:
    """A self-signed certificate for 127.0.0.1 / localhost and its 0640 key.

    ``None`` when ``openssl`` is not installed (callers skip with a reason).
    """
    openssl = shutil.which("openssl")
    if openssl is None:
        return None
    cert = directory / "cert.pem"
    key = directory / "key.pem"
    subprocess.run(
        [
            openssl,
            "req",
            "-x509",
            "-newkey",
            "ec",
            "-pkeyopt",
            "ec_paramgen_curve:prime256v1",
            "-nodes",
            "-days",
            "2",
            "-subj",
            "/CN=localhost",
            "-addext",
            "subjectAltName=DNS:localhost,IP:127.0.0.1",
            "-keyout",
            str(key),
            "-out",
            str(cert),
        ],
        check=True,
        capture_output=True,
    )
    os.chmod(key, 0o640)
    return cert, key
