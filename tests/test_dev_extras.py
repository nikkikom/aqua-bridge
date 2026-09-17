"""``dev`` pulls ``http`` and ``mqtt`` (PROJECT.md section 8 item 118).

Before this, ``pip install -e ".[dev]"`` in a clean venv left ``aiohttp`` and
``paho-mqtt`` out, and a full-suite run then hit two different failure shapes
depending on the test: a bare top-level ``import aiohttp``
(``tests/test_http_api.py``, ``tests/test_http_auth.py``, ...) is a collection
error, and ``pytest.importorskip("paho.mqtt.client")``
(``tests/test_mqtt_ha.py``, ``tests/test_ha_check.py``,
``tests/test_mqtt_live.py``) is a silent skip -- the same shape as item 108.
Both are the same root cause: a clean ``[dev]`` install does not actually
carry what the documented full suite (PROJECT.md section 4) needs.

This test reads ``pyproject.toml`` directly (not the running interpreter's
installed packages, which say nothing about what a *fresh* venv would pull)
and fails loudly, naming the missing extra, if ``dev`` ever stops asking for
``http`` and ``mqtt``.
"""

from __future__ import annotations

import tomllib
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent


def _pyproject() -> dict:
    with open(REPO_ROOT / "pyproject.toml", "rb") as f:
        return tomllib.load(f)


def test_dev_extra_declares_http_and_mqtt() -> None:
    extras = _pyproject()["project"]["optional-dependencies"]
    assert "http" in extras, "pyproject.toml lost the 'http' extra"
    assert "mqtt" in extras, "pyproject.toml lost the 'mqtt' extra"
    dev = extras["dev"]
    self_ref = [d for d in dev if d.replace(" ", "").startswith("aqua-bridge[")]
    assert self_ref, (
        "pyproject.toml's 'dev' extra no longer pulls 'http'/'mqtt' "
        '(item 118): a plain `pip install -e ".[dev]"` would again leave '
        "aiohttp/paho-mqtt out of a clean venv, and the full suite in "
        "PROJECT.md section 4 would fail (aiohttp) or silently skip "
        "(paho-mqtt) instead of running. Either restore a self-referential "
        "'aqua-bridge[http,mqtt]' entry in 'dev', or update this test and "
        "PROJECT.md item 118 together with the new install story."
    )
    pulled = {name.strip() for name in self_ref[0].split("[", 1)[1].rstrip("]").split(",")}
    missing = {"http", "mqtt"} - pulled
    assert not missing, (
        f"'dev' extra's self-reference is missing {sorted(missing)}: {self_ref[0]!r}"
    )


def test_http_and_mqtt_extras_have_actual_packages() -> None:
    extras = _pyproject()["project"]["optional-dependencies"]
    assert any("aiohttp" in dep for dep in extras["http"])
    assert any("paho-mqtt" in dep for dep in extras["mqtt"])
