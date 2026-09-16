"""Run the publishers next to the control loop (PROJECT.md sections 6, 7, 9).

The loop (:class:`aqua_bridge.control.loop.Loop`) knows nothing about HTTP or
MQTT; this module owns their threads and their lifetime:

* :class:`HttpService` -- the aiohttp app from
  :func:`aqua_bridge.publishers.http.run_http` on its own asyncio event loop
  in a daemon thread. ``start()`` blocks until the server listens or fails;
  a failure (invalid ``http:`` settings, a missing or unreadable TLS
  certificate, key or credentials file, port in use, bind denied) is logged
  and reported, never raised into the control path: the daemon keeps
  controlling the fans without the API rather than restarting in a loop.
* :class:`MqttService` -- a :class:`~aqua_bridge.publishers.mqtt_ha.MqttClient`
  connected asynchronously (paho's network thread, automatic reconnects) plus
  a per-tick publisher: ``on_tick`` (the loop's hook) publishes the retained
  state blob every tick, Discovery once after every (re)connect and whenever
  the control mode crosses the ``manual`` boundary (section 7: PWM number
  entities exist only in manual), and refreshes the host metrics every
  ``host.interval_s``. Connection changes are mirrored into
  ``Supervisor.set_mqtt_connected`` for ``/api/health``.

Both services are optional (``http.enabled`` / ``mqtt.enabled`` in
``config.yaml``) and both swallow their own exceptions: a publisher bug or
a dead broker must never touch ``read -> step -> apply``.

``HttpService`` optionally takes ``smart_inbox`` (a
:class:`~aqua_bridge.publishers.inputs.SmartInbox`) to wire ``POST
/api/in/smart``; the MQTT side of the same inbox is wired by the caller
(``__main__.start_publishers``) via ``MqttClient.add_topic_handler`` on the
client this module already builds, not by this module -- one inbox, two
transports, neither owned here.
"""

from __future__ import annotations

import asyncio
import logging
import threading
import time
from collections.abc import Callable, Mapping
from typing import Any, Protocol

from aqua_bridge.config import AppConfig
from aqua_bridge.control.intents import ControlMode, ControlSurface
from aqua_bridge.control.supervisor import Supervisor
from aqua_bridge.health import HostHealthConfig, host_metrics_reader
from aqua_bridge.publishers.http import run_http
from aqua_bridge.publishers.mqtt_ha import MqttClient, validate_mqtt_section

__all__ = ["HttpService", "MqttClientLike", "MqttService"]

_LOG = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# HTTP
# ---------------------------------------------------------------------------


class HttpService:
    """aiohttp server on a private event loop in a daemon thread."""

    def __init__(
        self,
        surface: ControlSurface,
        app_cfg: AppConfig,
        *,
        smart_inbox: Any = None,
        run: Callable[..., Any] | None = None,
    ) -> None:
        self._surface = surface
        self._app_cfg = app_cfg
        self._smart_inbox = smart_inbox
        # Resolved at call time so tests can monkeypatch the module attribute.
        self._run = run if run is not None else run_http
        self._thread: threading.Thread | None = None
        self._loop: asyncio.AbstractEventLoop | None = None
        self._runner: Any = None
        self._started = threading.Event()
        self.error: str | None = None

    @property
    def running(self) -> bool:
        return self._runner is not None and self._thread is not None and self._thread.is_alive()

    @property
    def addresses(self) -> list[Any]:
        """Bound socket addresses (``(host, port)`` tuples) once running."""
        if self._runner is None:
            return []
        try:
            return list(self._runner.addresses)
        except Exception:  # pragma: no cover - aiohttp internals
            return []

    def start(self, timeout_s: float = 10.0) -> bool:
        """Start the server thread; ``True`` once it listens, ``False`` on failure."""
        if self._thread is not None:
            return self.running
        self._thread = threading.Thread(target=self._main, name="aqua-bridge-http", daemon=True)
        self._thread.start()
        if not self._started.wait(timeout_s):
            self.error = f"http server did not start within {timeout_s} s"
        if self.error is not None:
            _LOG.error(
                "http: HTTPS API not started (fan control continues without it): %s", self.error
            )
            return False
        _LOG.info("http: HTTPS API listening on %s", self.addresses)
        return True

    def _main(self) -> None:
        loop = asyncio.new_event_loop()
        self._loop = loop
        asyncio.set_event_loop(loop)
        # Only passed when actually configured, so a caller-supplied ``run``
        # with the old two-argument shape (tests, mainly) keeps working
        # unchanged -- see test_http_service_start_failure_is_reported_not_raised.
        kwargs = {} if self._smart_inbox is None else {"smart_inbox": self._smart_inbox}
        try:
            self._runner = loop.run_until_complete(
                self._run(self._surface, self._app_cfg, **kwargs)
            )
        except Exception as exc:  # bind failure etc.: report, never propagate
            self.error = f"{type(exc).__name__}: {exc}"
            self._started.set()
            loop.close()
            return
        self._started.set()
        try:
            loop.run_forever()
        finally:
            try:
                loop.run_until_complete(self._runner.cleanup())
            except Exception:  # pragma: no cover - shutdown noise
                _LOG.exception("http: cleanup failed")
            self._runner = None
            loop.close()

    def stop(self, timeout_s: float = 5.0) -> None:
        loop, thread = self._loop, self._thread
        if loop is None or thread is None or not thread.is_alive():
            return
        try:
            loop.call_soon_threadsafe(loop.stop)
        except RuntimeError:  # loop already closed
            return
        thread.join(timeout_s)
        if thread.is_alive():
            _LOG.warning("http: server thread did not stop within %.1f s", timeout_s)


# ---------------------------------------------------------------------------
# MQTT
# ---------------------------------------------------------------------------


class MqttClientLike(Protocol):
    """What :class:`MqttService` needs from a client (tests inject a fake)."""

    connected: bool

    def connect_async(self) -> None: ...

    def loop_start(self) -> None: ...

    def loop_stop(self) -> None: ...

    def disconnect(self) -> None: ...

    def publish_discovery(self, *, control_mode: ControlMode) -> None: ...

    def publish_state(self, snapshot_dict: Mapping[str, Any], host: Mapping[str, Any]) -> None: ...


class MqttService:
    """Per-tick MQTT publisher over one :class:`MqttClientLike`."""

    def __init__(
        self,
        client: MqttClientLike,
        supervisor: Supervisor,
        *,
        host_interval_s: float = 5.0,
        hostinfo: Callable[[], Mapping[str, Any]] | None = None,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        self.client = client
        self.supervisor = supervisor
        self.host_interval_s = max(0.0, float(host_interval_s))
        # Left None: collect_hostinfo with the default throttling source chain --
        # the get_throttled sysfs attribute, a vcgencmd rate limited to one run per
        # host_health.vcgencmd_interval_s, then the rpi_volt hwmon under-voltage bit
        # (item 103). This publisher has its own reader and its own thread.
        self._hostinfo = hostinfo if hostinfo is not None else host_metrics_reader()
        self._clock = clock
        self._host: dict[str, Any] | None = None
        self._host_at = 0.0
        self._published_mode: ControlMode | None = None
        self._need_discovery = True
        self._lock = threading.Lock()
        self.publish_errors = 0

    @classmethod
    def from_config(
        cls,
        app_cfg: AppConfig,
        supervisor: Supervisor,
        *,
        client_factory: Callable[..., MqttClientLike] | None = None,
        **kwargs: Any,
    ) -> MqttService:
        """Build the client from the ``mqtt:`` / ``host:`` sections.

        Raises :class:`~aqua_bridge.publishers.mqtt_ha.MqttSetupError` (item 57) for a
        mistyped scalar -- ``host.interval_s`` is the one value not covered by
        :func:`~aqua_bridge.publishers.mqtt_ha.validate_mqtt_section` (it belongs to
        ``host:``, not ``mqtt:``) and keeps its looser ``float()`` coercion.
        """
        if client_factory is None:
            client_factory = MqttClient  # looked up at call time (monkeypatchable)
        mqtt_cfg = validate_mqtt_section(app_cfg.section("mqtt"))
        host_cfg = app_cfg.section("host")
        # The host reader's vcgencmd cadence and timeout come from documented keys,
        # not from numbers in the source (item 103); a caller-supplied reader wins.
        kwargs.setdefault(
            "hostinfo",
            host_metrics_reader(HostHealthConfig.from_section(app_cfg.section("host_health"))),
        )
        service: MqttService | None = None

        def on_connection_change(connected: bool) -> None:
            supervisor.set_mqtt_connected(connected)
            if service is not None:
                service.connection_changed(connected)

        client = client_factory(
            node_id=mqtt_cfg["node_id"],
            discovery_prefix=mqtt_cfg["discovery_prefix"],
            cfg=supervisor.base_config,
            host=mqtt_cfg["host"],
            port=mqtt_cfg["port"],
            username=mqtt_cfg["username"],
            password=mqtt_cfg["password"],
            allow_calibrate=mqtt_cfg["allow_calibrate"],
            on_intent=supervisor.submit,
            on_connection_change=on_connection_change,
        )
        service = cls(
            client, supervisor, host_interval_s=float(host_cfg.get("interval_s", 5.0)), **kwargs
        )
        return service

    # -- lifecycle --------------------------------------------------------

    def start(self) -> None:
        self.supervisor.set_mqtt_connected(False)
        self.client.connect_async()
        self.client.loop_start()

    def stop(self) -> None:
        try:
            self.client.disconnect()  # publishes availability=offline first
        except Exception:
            _LOG.exception("mqtt: disconnect failed")
        try:
            self.client.loop_stop()
        except Exception:
            _LOG.exception("mqtt: loop_stop failed")

    def connection_changed(self, connected: bool) -> None:
        """Called from the paho network thread; Discovery is re-sent on the next tick."""
        with self._lock:
            if connected:
                self._need_discovery = True

    # -- per tick -----------------------------------------------------------

    def _host_metrics(self) -> dict[str, Any]:
        now = self._clock()
        if self._host is None or now - self._host_at >= self.host_interval_s:
            try:
                self._host = dict(self._hostinfo())
            except Exception:  # hostinfo promises not to raise; belt and braces
                _LOG.exception("hostinfo failed")
                self._host = {}
            self._host_at = now
        return self._host

    def on_tick(self, _result: object = None) -> None:
        """Publish state (and Discovery when due). Never raises."""
        try:
            if not self.client.connected:
                return
            snapshot = self.supervisor.snapshot()
            mode = snapshot.control_mode
            with self._lock:
                need = self._need_discovery
                self._need_discovery = False
            crossed_manual = self._published_mode is not None and (
                (mode is ControlMode.MANUAL) != (self._published_mode is ControlMode.MANUAL)
            )
            if need or self._published_mode is None or crossed_manual:
                self.client.publish_discovery(control_mode=mode)
                self._published_mode = mode
            self.client.publish_state(snapshot.to_dict(), self._host_metrics())
        except Exception:
            self.publish_errors += 1
            _LOG.exception("mqtt: publish failed")
