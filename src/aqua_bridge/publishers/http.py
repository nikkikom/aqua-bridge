"""HTTP view and control (PROJECT.md section 6).

``create_app`` wires GET ``/api/state``, GET ``/api/health``, GET ``/`` (the
static single-page UI), the DAS views GET ``/api/estimate``, GET ``/api/bays`` and
GET ``/api/model``, and the seven ``POST /api/{mode,setpoint,pwm,preset,auto,limit,
bay}`` intents against a :class:`aqua_bridge.control.intents.ControlSurface`.

HTTP never computes PWM itself: every POST body becomes an
:class:`~aqua_bridge.control.intents.Intent` via
:func:`~aqua_bridge.control.intents.parse_intent` and is handed to
``surface.submit()``. All of the actual arithmetic (channel exists, PWM
range, auto/manual conflict) lives in ``submit``; this module only turns
its exceptions into the right HTTP status.

``POST /api/limit`` ``{"bay": ..., "limit_c": ...}`` or ``{"class": ...,
"limit_c": ...}`` is the DAS-mode intent (drive limits); a legacy config
answers 400 and keeps ``POST /api/setpoint``. The route exists for both so a
client learns the reason from the body instead of a 404. ``POST /api/bay``
``{"bay": ..., "occupied"?: true|false|"auto", "class"?: ..., "serial"?: ...}``
(a ``null`` field restores the configured value) works the same way.

DAS views (plan sections 1 and 7), read from the snapshot, never computed here:

* ``GET /api/estimate`` -- ``{"estimates": {bay: ...}, "estimator": {...}}``: the
  per-bay estimates (``t_c``, ``sigma_c``, ``margin_c``, ``soft_c``, ``hard_c``,
  ``limit_c``, ``occupancy``, ``class``, ``calibrated``, ``q_w``, ...) and the
  estimator summary (status, per zone air estimate, SMART counters) of the last
  command; both empty before the first tick.
* ``GET /api/bays`` -- ``{"bays": {bay: {"declared": {zone, occupied, class,
  serial}, "estimator": {occupancy, class, serial, association, calibration,
  candidates, ...} | null}}}``: the declarations in force and what the estimator
  made of them, including the serial candidates of the association by
  correlation with their scores (confirm one with ``POST /api/bay {bay, serial}``).
* ``GET /api/model`` -- ``{"thermal": {...}, "parameters": {kind: {unit, lo, hi, prior,
  identified_from}}, "calibration": {bay: {serial, calibrated, sigma_cal_c,
  calibration}}, "store": {...}}``: the zoned thermal model's identification summary of the last
  command (``diagnostics["thermal"]``: status, prediction error, per zone and bay the
  coefficients with their relative standard errors, see
  :mod:`aqua_bridge.control.thermal`; ``{"status": "off"}`` without
  ``model_shadow``), the static parameter table (units, bounds, priors) and the
  estimator's SMART calibration per bay, and what the model store loaded at start
  (``diagnostics["store"]``: source ``fresh`` | ``stale`` | ``prior``, sections,
  warnings; ``{"source": "off"}`` without a store, see :mod:`aqua_bridge.control.persist`).

A legacy config answers all three with 404 and a reason.

No authentication in v1: bind is LAN-only (``0.0.0.0:8080`` by default,
see ``config.example.yaml``). Section 6: "No auth on the local network in
MVP; add a bearer token before exposing the API further."

``POST /api/in/smart`` (the DAS plan, section 1 "SMART path") is the
non-MQTT twin of the PC-side SMART agent: same JSON body
(``{serial, model, temp_c, ts_wall}``), fed into the same
:class:`~aqua_bridge.publishers.inputs.SmartInbox` the MQTT client feeds.
It is only registered when ``create_app`` is given ``smart_inbox``; a
malformed body is a 400 (matching every other POST route here), never a
5xx, and ``SmartInbox.record`` itself never raises.
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
from aqua_bridge.control.thermal import PARAMETERS

__all__ = ["create_app", "run_http"]

_LOG = logging.getLogger(__name__)

_STATIC_DIR = Path(__file__).resolve().parent / "static"
_INDEX_HTML = _STATIC_DIR / "index.html"

_SURFACE_KEY = web.AppKey("surface", ControlSurface)
_CFG_KEY: web.AppKey[Any] = web.AppKey("cfg")
_SMART_KEY: web.AppKey[Any] = web.AppKey("smart_inbox")

# URL tail -> intent kind (identical today, kept separate so the route table
# and aqua_bridge.control.intents.INTENT_KINDS can diverge later).
_POST_KINDS = ("mode", "setpoint", "pwm", "preset", "auto", "limit", "bay")


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


_NOT_DAS = "estimates, bays and the thermal model need a DAS config (mpc.topology)"


def _diagnostics(snapshot: Any) -> dict[str, Any]:
    cmd = snapshot.last_cmd
    return {} if cmd is None else dict(cmd.diagnostics)


async def _get_estimate(request: web.Request) -> web.Response:
    surface: ControlSurface = request.app[_SURFACE_KEY]
    snapshot = surface.snapshot()
    if not snapshot.bays:
        return _error(404, _NOT_DAS)
    diag = _diagnostics(snapshot)
    return web.json_response(
        {"estimates": diag.get("estimates") or {}, "estimator": diag.get("estimator") or {}}
    )


async def _get_bays(request: web.Request) -> web.Response:
    surface: ControlSurface = request.app[_SURFACE_KEY]
    snapshot = surface.snapshot()
    if not snapshot.bays:
        return _error(404, _NOT_DAS)
    seen = _diagnostics(snapshot).get("bays") or {}
    return web.json_response(
        {
            "bays": {
                bay: {"declared": dict(declared), "estimator": seen.get(bay)}
                for bay, declared in snapshot.bays.items()
            }
        }
    )


async def _get_model(request: web.Request) -> web.Response:
    surface: ControlSurface = request.app[_SURFACE_KEY]
    snapshot = surface.snapshot()
    if not snapshot.bays:
        return _error(404, _NOT_DAS)
    diag = _diagnostics(snapshot)
    seen = diag.get("bays") or {}
    return web.json_response(
        {
            "thermal": diag.get("thermal") or {"status": "off"},
            "parameters": {kind: spec.to_dict() for kind, spec in PARAMETERS.items()},
            "calibration": {
                bay: {
                    "serial": info.get("serial"),
                    "calibrated": info.get("calibrated"),
                    "sigma_cal_c": info.get("sigma_cal_c"),
                    "calibration": info.get("calibration"),
                }
                for bay, info in seen.items()
            },
            "store": diag.get("store") or {"source": "off"},
        }
    )


async def _post_in_smart(request: web.Request) -> web.Response:
    inbox = request.app.get(_SMART_KEY)
    if inbox is None:
        return _error(404, "smart inbox not configured")
    try:
        body = await _read_json_body(request)
    except IntentInvalid as exc:
        return _error(400, str(exc))
    try:
        ok = inbox.record(body)
    except Exception:  # SmartInbox.record promises not to raise; belt and braces
        _LOG.exception("smart: inbox.record raised")
        return _error(400, "invalid SMART payload")
    if not ok:
        return _error(400, "invalid SMART payload")
    return web.json_response({"ok": True})


async def _get_index(request: web.Request) -> web.Response:
    try:
        text = _INDEX_HTML.read_text(encoding="utf-8")
    except OSError:
        _LOG.warning("static/index.html not found at %s", _INDEX_HTML)
        return _error(404, "static UI not installed")
    return web.Response(text=text, content_type="text/html")


def create_app(
    surface: ControlSurface, cfg: AppConfig | None = None, *, smart_inbox: Any = None
) -> web.Application:
    """Build the aiohttp application. ``cfg`` is accepted for parity with
    :func:`run_http` and future per-instance config; today the app needs
    nothing from it beyond what ``surface`` already carries. ``smart_inbox``
    (a :class:`~aqua_bridge.publishers.inputs.SmartInbox`, duck-typed --
    only ``.record(dict) -> bool`` is used) wires ``POST /api/in/smart``;
    left ``None`` that route answers 404, never a 5xx.
    """
    app = web.Application()
    app[_SURFACE_KEY] = surface
    app[_CFG_KEY] = cfg
    app[_SMART_KEY] = smart_inbox
    app.router.add_get("/api/state", _get_state)
    app.router.add_get("/api/health", _get_health)
    app.router.add_get("/api/estimate", _get_estimate)
    app.router.add_get("/api/bays", _get_bays)
    app.router.add_get("/api/model", _get_model)
    app.router.add_get("/", _get_index)
    for kind in _POST_KINDS:
        app.router.add_post(f"/api/{kind}", _make_intent_handler(kind))
    app.router.add_post("/api/in/smart", _post_in_smart)
    return app


async def run_http(
    surface: ControlSurface, cfg: AppConfig, *, smart_inbox: Any = None
) -> web.AppRunner:
    """Start the HTTP server per ``cfg.http`` (``bind``/``port``).

    Returns the started :class:`aiohttp.web.AppRunner`; the caller owns its
    lifetime and must ``await runner.cleanup()`` on shutdown.
    """
    http_cfg = cfg.section("http")
    bind = http_cfg.get("bind", "0.0.0.0")
    port = int(http_cfg.get("port", 8080))
    app = create_app(surface, cfg, smart_inbox=smart_inbox)
    runner = web.AppRunner(app)
    await runner.setup()
    site = web.TCPSite(runner, bind, port)
    await site.start()
    return runner
