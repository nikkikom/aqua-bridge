"""HTTP view and control (PROJECT.md section 6).

``create_app`` wires GET ``/api/state``, GET ``/api/health``, GET ``/`` (the
static single-page UI) and the five ``POST /api/{mode,setpoint,pwm,preset,
auto}`` intents against a :class:`aqua_bridge.control.intents.ControlSurface`.

HTTP never computes PWM itself: every POST body becomes an
:class:`~aqua_bridge.control.intents.Intent` via
:func:`~aqua_bridge.control.intents.parse_intent` and is handed to
``surface.submit()``. All of the actual arithmetic (channel exists, PWM
range, auto/manual conflict) lives in ``submit``; this module only turns
its exceptions into the right HTTP status.

No authentication in v1: bind is LAN-only (``0.0.0.0:8080`` by default,
see ``config.example.yaml``). Section 6: "No auth on the local network in
MVP; add a bearer token before exposing the API further."
"""

from __future__ import annotations

import json
import logging
from pathlib import Path
from typing import Any

from aiohttp import web

from aqua_bridge.config import AppConfig
from aqua_bridge.control.intents import (
    ControlSurface,
    IntentConflict,
    IntentError,
    IntentInvalid,
    parse_intent,
)

__all__ = ["create_app", "run_http"]

_LOG = logging.getLogger(__name__)

_STATIC_DIR = Path(__file__).resolve().parent / "static"
_INDEX_HTML = _STATIC_DIR / "index.html"

_SURFACE_KEY = web.AppKey("surface", ControlSurface)
_CFG_KEY: web.AppKey[Any] = web.AppKey("cfg")

# URL tail -> intent kind (identical today, kept separate so the route table
# and aqua_bridge.control.intents.INTENT_KINDS can diverge later).
_POST_KINDS = ("mode", "setpoint", "pwm", "preset", "auto")


def _error(status: int, message: str) -> web.Response:
    return web.json_response({"error": message}, status=status)


async def _read_json_body(request: web.Request) -> Any:
    """Parse the request body as JSON. An empty body means ``{}``."""
    raw = await request.read()
    if not raw.strip():
        return {}
    try:
        text = raw.decode("utf-8")
    except UnicodeDecodeError as exc:
        raise IntentInvalid(f"body is not valid UTF-8: {exc}") from exc
    try:
        return json.loads(text)
    except json.JSONDecodeError as exc:
        raise IntentInvalid(f"body is not valid JSON: {exc}") from exc


def _make_intent_handler(kind: str):
    async def handler(request: web.Request) -> web.Response:
        surface: ControlSurface = request.app[_SURFACE_KEY]
        try:
            body = await _read_json_body(request)
            intent = parse_intent(kind, body)
            surface.submit(intent)
        except IntentConflict as exc:
            return _error(409, str(exc))
        except IntentInvalid as exc:
            return _error(400, str(exc))
        except IntentError as exc:  # pragma: no cover - defensive, no other subclass today
            return _error(400, str(exc))
        return web.json_response({"ok": True})

    handler.__name__ = f"post_{kind}"
    return handler


async def _get_state(request: web.Request) -> web.Response:
    surface: ControlSurface = request.app[_SURFACE_KEY]
    snapshot = surface.snapshot()
    return web.json_response(snapshot.state_payload())


async def _get_health(request: web.Request) -> web.Response:
    surface: ControlSurface = request.app[_SURFACE_KEY]
    snapshot = surface.snapshot()
    return web.json_response(snapshot.health_payload())


async def _get_index(request: web.Request) -> web.Response:
    try:
        text = _INDEX_HTML.read_text(encoding="utf-8")
    except OSError:
        _LOG.warning("static/index.html not found at %s", _INDEX_HTML)
        return _error(404, "static UI not installed")
    return web.Response(text=text, content_type="text/html")


def create_app(surface: ControlSurface, cfg: AppConfig | None = None) -> web.Application:
    """Build the aiohttp application. ``cfg`` is accepted for parity with
    :func:`run_http` and future per-instance config; today the app needs
    nothing from it beyond what ``surface`` already carries.
    """
    app = web.Application()
    app[_SURFACE_KEY] = surface
    app[_CFG_KEY] = cfg
    app.router.add_get("/api/state", _get_state)
    app.router.add_get("/api/health", _get_health)
    app.router.add_get("/", _get_index)
    for kind in _POST_KINDS:
        app.router.add_post(f"/api/{kind}", _make_intent_handler(kind))
    return app


async def run_http(surface: ControlSurface, cfg: AppConfig) -> web.AppRunner:
    """Start the HTTP server per ``cfg.http`` (``bind``/``port``).

    Returns the started :class:`aiohttp.web.AppRunner`; the caller owns its
    lifetime and must ``await runner.cleanup()`` on shutdown.
    """
    http_cfg = cfg.section("http")
    bind = http_cfg.get("bind", "0.0.0.0")
    port = int(http_cfg.get("port", 8080))
    app = create_app(surface, cfg)
    runner = web.AppRunner(app)
    await runner.setup()
    site = web.TCPSite(runner, bind, port)
    await site.start()
    return runner
