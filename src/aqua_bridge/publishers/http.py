"""HTTP view and control (PROJECT.md section 6).

``create_app`` wires GET ``/api/state``, GET ``/api/health``, GET ``/`` (the
static single-page UI), the DAS views GET ``/api/estimate``, GET ``/api/bays`` and
GET ``/api/model``, and the eight ``POST /api/{mode,setpoint,pwm,preset,auto,limit,
bay,ident}`` intents against a :class:`aqua_bridge.control.intents.ControlSurface`.

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

``POST /api/ident`` ``{"action": "start", "group": ...}`` | ``{"action": "start",
"channel": ...}`` | ``{"action": "stop"}`` starts or stops an identification
experiment (DAS mode, :mod:`aqua_bridge.control.ident`): 200 on success, 400 for a
malformed body, an unknown group or channel or a legacy config, 409 when
``ident_enabled`` is false, an experiment already runs or a precondition fails
(the body's ``error`` names every failed precondition, e.g. ``settle:z1``,
``mode:degraded``, ``start_band:b03``). Its status (``ControlSnapshot.extra["experiment"]``:
running, target, phase, level, elapsed and remaining seconds, the last result and
abort reason) is in ``GET /api/model`` under ``experiment`` and in the MQTT state blob.

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
  estimator's SMART calibration per bay, what the model store loaded at start
  (``diagnostics["store"]``: source ``fresh`` | ``stale`` | ``prior``, sections,
  warnings; ``{"source": "off"}`` without a store, see :mod:`aqua_bridge.control.persist`),
  and ``"experiment"``: the identification experiment's status.

A legacy config answers all three with 404 and a reason.

HTTPS and basic auth (section 6, :mod:`aqua_bridge.publishers.httpauth`):
:func:`run_http` listens only with a TLS context (``http.tls_cert`` /
``http.tls_key``); there is no plain-HTTP listener. ``create_app`` requires a
:class:`~aqua_bridge.publishers.httpauth.BasicAuthenticator` and puts every
route behind it, ``GET /`` and unknown paths included: no or wrong credentials
answer ``401`` with ``WWW-Authenticate: Basic realm=...``, a client backing off
after repeated failures answers ``429`` with ``Retry-After``. A credentials
check that needs PBKDF2 runs in a worker thread so the event loop keeps serving.

``POST /api/in/smart`` (the DAS plan, section 1 "SMART path") is the
non-MQTT twin of the PC-side SMART agent: same JSON body
(``{serial, model, temp_c, ts_wall}``), fed into the same
:class:`~aqua_bridge.publishers.inputs.SmartInbox` the MQTT client feeds.
It is only registered when ``create_app`` is given ``smart_inbox``; a
malformed body is a 400 (matching every other POST route here), never a
5xx, and ``SmartInbox.record`` itself never raises.
"""

from __future__ import annotations

import asyncio
import json
import logging
import math
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
from aqua_bridge.publishers.httpauth import (
    BasicAuthenticator,
    HttpSettings,
    build_ssl_context,
)

__all__ = ["create_app", "run_http"]

_LOG = logging.getLogger(__name__)

_STATIC_DIR = Path(__file__).resolve().parent / "static"
_INDEX_HTML = _STATIC_DIR / "index.html"

_SURFACE_KEY = web.AppKey("surface", ControlSurface)
_CFG_KEY: web.AppKey[Any] = web.AppKey("cfg")
_SMART_KEY: web.AppKey[Any] = web.AppKey("smart_inbox")
_AUTH_KEY = web.AppKey("auth", BasicAuthenticator)

# URL tail -> intent kind (identical today, kept separate so the route table
# and aqua_bridge.control.intents.INTENT_KINDS can diverge later).
_POST_KINDS = ("mode", "setpoint", "pwm", "preset", "auto", "limit", "bay", "ident")


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


@web.middleware
async def _auth_middleware(request: web.Request, handler: Any) -> web.StreamResponse:
    """Every route, ``GET /`` and unknown paths included, needs valid credentials."""
    auth: BasicAuthenticator = request.app[_AUTH_KEY]
    header = request.headers.get("Authorization")
    client = request.remote or ""
    decision = auth.fast(header, client)
    if decision is None:
        decision = await asyncio.to_thread(auth.verify, header, client)
    if decision.ok:
        return await handler(request)
    if decision.retry_after_s is not None:
        return web.json_response(
            {"error": "too many failed logins; retry later"},
            status=429,
            headers={"Retry-After": str(max(1, math.ceil(decision.retry_after_s)))},
        )
    realm = auth.settings.realm
    return web.json_response(
        {"error": "authentication required"},
        status=401,
        headers={"WWW-Authenticate": f'Basic realm="{realm}", charset="UTF-8"'},
    )


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
            "experiment": snapshot.extra.get("experiment"),
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
    surface: ControlSurface,
    cfg: AppConfig | None = None,
    *,
    auth: BasicAuthenticator,
    smart_inbox: Any = None,
) -> web.Application:
    """Build the aiohttp application. ``cfg`` is accepted for parity with
    :func:`run_http` and future per-instance config; today the app needs
    nothing from it beyond what ``surface`` already carries. ``auth`` is
    required: there is no unauthenticated app. ``smart_inbox``
    (a :class:`~aqua_bridge.publishers.inputs.SmartInbox`, duck-typed --
    only ``.record(dict) -> bool`` is used) wires ``POST /api/in/smart``;
    left ``None`` that route answers 404, never a 5xx.
    """
    if not isinstance(auth, BasicAuthenticator):
        raise TypeError("create_app needs a BasicAuthenticator")
    app = web.Application(middlewares=[_auth_middleware])
    app[_AUTH_KEY] = auth
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
    """Start the HTTPS server per ``cfg.http``.

    Validates the section (:class:`~aqua_bridge.publishers.httpauth.HttpSettings`),
    loads the TLS certificate and key and the credentials file first; any
    problem raises :class:`~aqua_bridge.publishers.httpauth.HttpSetupError`
    before a socket is opened, so nothing ever listens without TLS and auth.
    Returns the started :class:`aiohttp.web.AppRunner`; the caller owns its
    lifetime and must ``await runner.cleanup()`` on shutdown.
    """
    settings = HttpSettings.from_section(cfg.section("http"))
    ssl_context = build_ssl_context(settings)
    auth = BasicAuthenticator(settings)
    app = create_app(surface, cfg, auth=auth, smart_inbox=smart_inbox)
    runner = web.AppRunner(app)
    await runner.setup()
    try:
        site = web.TCPSite(runner, settings.bind, settings.port, ssl_context=ssl_context)
        await site.start()
    except BaseException:
        await runner.cleanup()
        raise
    return runner
