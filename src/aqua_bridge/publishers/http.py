"""HTTP view and control (PROJECT.md section 6).

``create_app`` wires GET ``/api/state``, GET ``/api/health``, GET ``/`` (the
static single-page UI), the DAS views GET ``/api/estimate``, GET ``/api/bays``,
GET ``/api/model`` and GET ``/api/zones``, and the nine ``POST /api/{mode,setpoint,
pwm,preset,auto,limit,bay,ident,calibrate}`` intents against a
:class:`aqua_bridge.control.intents.ControlSurface`.

``GET /api/state`` carries a top-level ``device_health`` key (PROJECT.md section 8
items 79 and 83): per controller its stuck outputs, the aquabus slots with no
device behind them, the commanded outputs not in PWM mode, unconfigured
controller blocks, the flow sensors, its aquabus (``state``, ``present``,
``seen``, ``absent_s``, ``lost``, ``bound``, ``temps_missing``; PROJECT.md
section 8 items 92, 114, 115, 129) and -- once another change publishes one --
the aquaero's active profile; per channel the rpm, output duty, 12 V rail
voltage, current and power the status report gives, with the fitted curve's
expected rpm and power and any drift found; plus ``problems``, ``ok`` and
``aquabus_lost`` (whether any controller's aquabus is ``lost`` and ``bound`` --
one scalar so a consumer does not have to walk ``devices`` to answer the same
question the daemon's own ``problems`` list already answered).
``GET /api/health`` carries the same ``ok`` and ``problems`` under
``device_health``, the pair a Home Assistant problem sensor needs. Both are empty
and ``ok`` before the first tick and with a source that has no device health
(the simulator).

``device_health`` also carries a ``host`` key (PROJECT.md section 8 item 103): the
board's own temperature, the enclosure-air reference it is compared against, the
load average, the decoded ``get_throttled`` word, the free space and read-only
state of the card the daemon runs from (``disk_free_gb``, ``disk_used_pct``,
``read_only``), and this board's own ``faults``, ``hints``, ``problems`` and
``ok``. Only its *faults* -- the board is hot, the board is throttling now, the
filesystem has gone read-only -- join the top-level ``problems`` list that
``/api/health`` shows; the divergence rule and the free-space rule are hints,
not verdicts (the free-space rule for the same reason: a filling card can sit
below its threshold for days, and a daemon-wide flag latched that long would
read exactly like a missing aquabus device), so each stays in
``device_health.host`` and does not make the daemon not-ok. The board -- and the
card it runs from -- are a health signal and nothing else: neither is in
``PlantObservation``, in the ``diagnostics`` the solver reads, and no rule here
can change a duty.

``GET /api/state`` also carries a top-level ``host`` key (host machine metrics,
:mod:`aqua_bridge.hostinfo`): the same numbers as the MQTT state blob's ``host``
key, refreshed at most every ``host.interval_s`` seconds (default 5.0) through a
:class:`~aqua_bridge.hostinfo.CachedHostInfo` owned by this app -- polled every 2 s
by the page, so an uncached read on every request would mean unnecessary
``/proc``/``/sys`` I/O per poll for no fresher data than the refresh interval already
gives.

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
running, target, phase, level, the base at the start, the anchor the levels are drawn
around now (``plan_base``) with ``levels`` and ``replan``, elapsed and remaining seconds,
the last result and abort reason) is in ``GET /api/model`` under ``experiment`` and in
the MQTT state blob.

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
* ``GET /api/zones`` -- ``{"zones": {zone: {...}}, "zones_in_fault": [...],
  "degraded": bool}``: per-zone trust, fault state and reason, and which of the
  zone's channels are currently under fallback policy (``diagnostics["zones"]``,
  intersected per zone with ``diagnostics["fallback_channels"]``, see
  :mod:`aqua_bridge.control.mpc`); ``degraded`` mirrors ``/api/health``'s
  ``solver == "degraded"``. Both empty/false before the first tick.
* ``GET /api/model`` -- ``{"thermal": {...}, "parameters": {kind: {unit, lo, hi, prior,
  identified_from}}, "calibration": {bay: {serial, calibrated, sigma_cal_c,
  calibration_source, calibration}}, "manual_calibrations": {bay: {temp_c, ts}},
  "store": {...}}``: the zoned thermal model's identification summary of the last
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
after repeated failures, or a request that needs an uncached password check
while the global limit on those checks refuses (``http.auth_verify_max``,
``http.auth_verify_pending_max``), answers ``429`` with ``Retry-After``; a
cached login waits for neither. An admitted check (PBKDF2) runs on the
authenticator's one low-priority worker thread, so the event loop keeps
serving.

``POST /api/calibrate`` ``{"bay": ..., "drive_temp_c": ...}`` (DAS mode, PROJECT.md
section 8 item 23) is the handheld-thermometer stand-in for SMART: one measured drive
temperature for one bay, through the same auth, the same rate limit and the same intent
path as every other command. 200 on success; 400 for a malformed body, an unknown bay,
a temperature outside ``mpc.estimator.calibrate_min_c`` / ``calibrate_max_c`` or a
legacy config; 409 when the reading would be meaningless or unsafe -- before the first
tick, for a bay the estimator calls ``empty``, or while the bay's zone is untrusted or
in fault (the ``error`` names which and why, e.g. ``untrusted:za``, ``empty:b03``,
``no_tick``). The accepted reading shows up in ``GET /api/model`` under
``manual_calibrations`` while it is still offered to the estimator (for
``mpc.estimator.smart_max_age_s``; the estimator folds it in once, by sample time,
exactly as it does a repeated SMART reading), and in ``GET /api/bays`` and
``GET /api/model`` as the bay's ``calibration`` with ``calibration_source: "manual"``.
One reading calibrates nothing on its own: a map is accepted at
:data:`~aqua_bridge.control.estimator.CAL_MIN_SAMPLES` fresh samples, so the 200 body
carries ``calibration`` with the bay's ``fresh_samples`` (as of the last tick),
``samples_required``, ``calibrated`` and ``calibration_source`` for the operator to
see how far along the bay is.

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
from collections.abc import Callable, Mapping
from pathlib import Path
from typing import Any

from aiohttp import web

from aqua_bridge.config import AppConfig
from aqua_bridge.control.estimator import CAL_MIN_SAMPLES
from aqua_bridge.control.intents import (
    Calibrate,
    ControlSurface,
    IntentConflict,
    IntentError,
    IntentInvalid,
    SolverStatus,
    parse_intent,
)
from aqua_bridge.control.thermal import PARAMETERS
from aqua_bridge.health import HostHealthConfig, host_metrics_reader
from aqua_bridge.hostinfo import CachedHostInfo
from aqua_bridge.publishers.httpauth import (
    AuthDecision,
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
_HOST_KEY = web.AppKey("host_cache", CachedHostInfo)

# URL tail -> intent kind (identical today, kept separate so the route table
# and aqua_bridge.control.intents.INTENT_KINDS can diverge later).
_POST_KINDS = (
    "mode",
    "setpoint",
    "pwm",
    "preset",
    "auto",
    "limit",
    "bay",
    "ident",
    "calibrate",
)


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
    outcome = auth.begin(header, client)
    if isinstance(outcome, AuthDecision):
        decision = outcome
    else:
        # The admitted PBKDF2 check runs on the authenticator's one low-priority worker
        # thread. Shielded: a cancelled request still lets the check run and free its slot.
        future = auth.executor.submit(auth.check, outcome)
        decision = await asyncio.shield(asyncio.wrap_future(future))
    if decision.ok:
        return await handler(request)
    if decision.retry_after_s is not None:
        return web.json_response(
            {"error": "too many login attempts; retry later"},
            status=429,
            headers={"Retry-After": str(max(1, math.ceil(decision.retry_after_s)))},
        )
    realm = auth.settings.realm
    return web.json_response(
        {"error": "authentication required"},
        status=401,
        headers={"WWW-Authenticate": f'Basic realm="{realm}", charset="UTF-8"'},
    )


def _calibration_progress(surface: ControlSurface, bay: str) -> dict[str, Any]:
    """What ``POST /api/calibrate`` tells the operator about the bay's map (item 23).

    An accepted calibration needs ``CAL_MIN_SAMPLES`` fresh samples, so one handheld
    reading leaves the estimate exactly where it was; without this the 200 would read
    as "this bay is calibrated now". The counts are those of the last completed tick --
    the reading just taken is folded in on the next one, so ``fresh_samples`` is one
    short of what this reading will make it.
    """
    info = (_diagnostics(surface.snapshot()).get("bays") or {}).get(bay) or {}
    cal = info.get("calibration") or {}
    return {
        "bay": bay,
        "calibrated": bool(info.get("calibrated")),
        "calibration_source": info.get("calibration_source"),
        "fresh_samples": cal.get("fresh_samples", 0),
        "samples_required": CAL_MIN_SAMPLES,
    }


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
        payload: dict[str, Any] = {"ok": True}
        if isinstance(intent, Calibrate):
            payload["calibration"] = _calibration_progress(surface, intent.bay)
        return web.json_response(payload)

    handler.__name__ = f"post_{kind}"
    return handler


async def _get_state(request: web.Request) -> web.Response:
    surface: ControlSurface = request.app[_SURFACE_KEY]
    snapshot = surface.snapshot()
    payload = snapshot.state_payload()
    payload["host"] = request.app[_HOST_KEY].get()
    return web.json_response(payload)


async def _get_health(request: web.Request) -> web.Response:
    surface: ControlSurface = request.app[_SURFACE_KEY]
    snapshot = surface.snapshot()
    return web.json_response(snapshot.health_payload())


_NOT_DAS = "estimates, bays, zones and the thermal model need a DAS config (mpc.topology)"


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


async def _get_zones(request: web.Request) -> web.Response:
    surface: ControlSurface = request.app[_SURFACE_KEY]
    snapshot = surface.snapshot()
    if not snapshot.bays:
        return _error(404, _NOT_DAS)
    diag = _diagnostics(snapshot)
    zones_diag = diag.get("zones") or {}
    fallback_channels = set(diag.get("fallback_channels") or [])
    zones_out: dict[str, Any] = {}
    for zone, info in zones_diag.items():
        info = dict(info)
        channels = list(info.get("channels") or ())
        info["channels"] = channels
        info["channels_under_fallback"] = [ch for ch in channels if ch in fallback_channels]
        zones_out[zone] = info
    return web.json_response(
        {
            "zones": zones_out,
            "zones_in_fault": list(diag.get("zones_in_fault") or []),
            "degraded": snapshot.solver_status is SolverStatus.DEGRADED,
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
            "manual_calibrations": snapshot.extra.get("calibrations") or {},
            "calibration": {
                bay: {
                    "serial": info.get("serial"),
                    "calibrated": info.get("calibrated"),
                    "sigma_cal_c": info.get("sigma_cal_c"),
                    "calibration_source": info.get("calibration_source"),
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
    hostinfo: Callable[[], Mapping[str, Any]] | None = None,
) -> web.Application:
    """Build the aiohttp application. ``cfg`` is accepted for parity with
    :func:`run_http` and future per-instance config; today the app reads only
    ``cfg.host["interval_s"]`` (the ``host.interval_s`` key documented in section
    7) from it, for the ``/api/state`` host-metrics cache below -- ``cfg=None``
    (most tests) uses that key's default, 5.0 s. ``auth`` is required: there is
    no unauthenticated app. ``smart_inbox`` (a
    :class:`~aqua_bridge.publishers.inputs.SmartInbox`, duck-typed -- only
    ``.record(dict) -> bool`` is used) wires ``POST /api/in/smart``; left
    ``None`` that route answers 404, never a 5xx. ``hostinfo`` is the reader
    :class:`~aqua_bridge.hostinfo.CachedHostInfo` wraps for ``GET /api/state``'s
    ``host`` key; left ``None`` it is
    :func:`aqua_bridge.hostinfo.collect_hostinfo` with the throttling source chain
    this config asks for (:func:`aqua_bridge.health.host_metrics_reader`): the
    ``get_throttled`` sysfs attribute where the kernel has one, else a ``vcgencmd``
    run at most every ``host_health.vcgencmd_interval_s`` and bounded by
    ``host_health.vcgencmd_timeout_s``, else the ``rpi_volt`` hwmon under-voltage
    bit. This server has its own thread and its own reader, so its polling is
    independent of the control loop's. Tests inject a fixed dict.
    """
    if not isinstance(auth, BasicAuthenticator):
        raise TypeError("create_app needs a BasicAuthenticator")
    app = web.Application(middlewares=[_auth_middleware])
    app[_AUTH_KEY] = auth

    async def _close_auth(_app: web.Application) -> None:
        auth.close()

    app.on_cleanup.append(_close_auth)
    app[_SURFACE_KEY] = surface
    app[_CFG_KEY] = cfg
    app[_SMART_KEY] = smart_inbox
    host_interval_s = 5.0 if cfg is None else float(cfg.section("host").get("interval_s", 5.0))
    if hostinfo is None:
        host_settings = (
            HostHealthConfig()
            if cfg is None
            else HostHealthConfig.from_section(cfg.section("host_health"))
        )
        hostinfo = host_metrics_reader(host_settings)
    app[_HOST_KEY] = CachedHostInfo(interval_s=host_interval_s, reader=hostinfo)
    app.router.add_get("/api/state", _get_state)
    app.router.add_get("/api/health", _get_health)
    app.router.add_get("/api/estimate", _get_estimate)
    app.router.add_get("/api/bays", _get_bays)
    app.router.add_get("/api/zones", _get_zones)
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
