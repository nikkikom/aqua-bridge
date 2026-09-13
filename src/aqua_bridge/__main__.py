"""``python -m aqua_bridge --config /etc/aqua-bridge/config.yaml`` (sections 9, 10, 11).

Builds the source/sink pair, the :class:`~aqua_bridge.control.supervisor.Supervisor`
and the :class:`~aqua_bridge.control.loop.Loop`, installs the SIGTERM/SIGINT
handler (which only sets the stop event and records the signal number; the
main thread then runs the section 9 stop path -- log, write ``fallback_pwm``,
``STOPPING=1``), and ticks at ``cfg.dt`` on ``time.monotonic``.

``--source xt6`` (default) imports :mod:`aqua_bridge.hw.xt6` lazily -- the
hardware adapter is the only place that knows sysfs. ``--source sim`` drives
the RC plant from :mod:`aqua_bridge.sim.plant` instead, for a laptop or CI.

HTTP (``http.enabled``) and MQTT (``mqtt.enabled``) run next to the loop via
:mod:`aqua_bridge.publishers.runtime`; both attach to the supervisor's
``ControlSurface``. A publisher that fails to start (port in use, broker
down) is logged and the daemon keeps controlling the fans without it.

Exit codes: ``0`` clean stop, ``2`` bad arguments or config, ``3`` the
source/sink could not be built.
"""

from __future__ import annotations

import argparse
import json
import logging
import signal
import sys
import threading
import time
from collections.abc import Callable, Sequence
from typing import Any

from aqua_bridge.config import AppConfig, ConfigError, load_config
from aqua_bridge.control.loop import Loop, Sink, Source
from aqua_bridge.control.supervisor import Supervisor
from aqua_bridge.model import MpcCommand, MpcConfig, PlantObservation
from aqua_bridge.sdnotify import SdNotifier

__all__ = [
    "PlantIO",
    "build_io",
    "build_parser",
    "main",
    "make_sim_plant",
    "start_publishers",
    "stop_publishers",
]

_LOG = logging.getLogger("aqua_bridge.main")

try:
    from importlib.metadata import version as _pkg_version

    VERSION = _pkg_version("aqua-bridge")
except Exception:  # not installed (tests run from the source tree)
    VERSION = "0.0.0+src"


# ---------------------------------------------------------------------------
# Simulated source/sink
# ---------------------------------------------------------------------------


def make_sim_plant(cfg: MpcConfig, *, heat_w: float = 100.0, seed: int = 0) -> Any:
    """An RC plant whose channel and temperature names follow ``cfg``.

    Channels whose name contains ``"intake"`` move case air; every other
    channel is a radiator fan. The first temperature with a setpoint is the
    coolant, the first other temperature (if any) the air.
    """
    from aqua_bridge.sim.plant import Plant, PlantParams

    coolant = next(iter(cfg.setpoints))
    others = [t for t in cfg.temps if t != coolant]
    air = others[0] if others else "air"
    radiator = {ch: 25.0 for ch in cfg.channels if "intake" not in ch}
    intake = {ch: 20.0 for ch in cfg.channels if "intake" in ch}
    if not radiator:  # every channel is an intake: give the loop something to cool with
        radiator = dict(intake)
    params = PlantParams(
        dt=cfg.dt,
        coolant=coolant,
        air=air,
        heat_w=heat_w,
        radiator_fans=radiator,
        intake_fans=intake,
    )
    return Plant(params, initial_pwm=dict(cfg.fallback_pwm), seed=seed)


class PlantIO:
    """Source and sink over one :class:`aqua_bridge.sim.plant.Plant`.

    ``read`` reports the plant's current observation restricted to
    ``cfg.temps``; ``apply`` feeds the command in and advances the plant one
    ``dt``, so the observation clock is the plant's ``ts``.
    """

    def __init__(self, plant: Any, cfg: MpcConfig) -> None:
        self.plant = plant
        self.cfg = cfg
        self.applied: list[MpcCommand] = []

    def read(self) -> PlantObservation:
        obs = self.plant.observe()
        temps = {name: obs.temps.get(name) for name in self.cfg.temps}
        return PlantObservation(temps=temps, rpm=dict(obs.rpm), pwm=dict(obs.pwm), ts=obs.ts)

    def apply(self, cmd: MpcCommand) -> None:
        self.applied.append(cmd)
        self.plant.apply(cmd.pwm)
        self.plant.advance()


# ---------------------------------------------------------------------------
# Wiring
# ---------------------------------------------------------------------------


def build_io(
    app: AppConfig, source: str, *, clock: Callable[[], float] = time.monotonic
) -> tuple[Source, Sink, Callable[[], None] | None]:
    """``(source, sink, release)`` for ``--source``; ``release`` runs at exit if not None."""
    if source == "sim":
        io = PlantIO(make_sim_plant(app.mpc), app.mpc)
        return io, io, None
    if source == "xt6":
        try:
            from aqua_bridge.hw import xt6
        except ImportError as exc:
            raise RuntimeError(f"hardware adapter unavailable: {exc}") from exc
        builder = getattr(xt6, "build_map_from_config", None)
        if builder is None:
            raise RuntimeError("aqua_bridge.hw.xt6.build_map_from_config is missing")
        # Cross-check xt6.map / xt6.temp_map against mpc.channels / mpc.temps
        # here, before the loop starts (ConfigError, exit code 2).
        hwmon_map = builder(app.section("xt6"), channels=app.mpc.channels, temps=app.mpc.temps)
        adapter = xt6.Xt6Adapter(hwmon_map, clock=clock)
        return adapter, adapter, None
    raise RuntimeError(f"unknown source {source!r}")


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(prog="aqua_bridge", description=__doc__.split("\n\n")[0])
    p.add_argument("--config", required=True, help="path to config.yaml")
    p.add_argument(
        "--source",
        choices=("xt6", "sim"),
        default="xt6",
        help="xt6: aquaero over hwmon (default); sim: RC plant simulator",
    )
    p.add_argument("--once", action="store_true", help="one tick, print the command as JSON, exit")
    p.add_argument("--ticks", type=int, default=None, metavar="N", help="stop after N ticks")
    p.add_argument(
        "--sim-speed",
        type=float,
        default=1.0,
        metavar="X",
        help="sim only: X=1 real time, X=0 as fast as possible",
    )
    p.add_argument("--log-level", default="INFO", help="DEBUG, INFO, WARNING, ERROR")
    return p


def _make_sleep(speed: float) -> Callable[[float, threading.Event], None] | None:
    if speed == 1.0:
        return None
    if speed <= 0:
        return lambda seconds, stop: None
    return lambda seconds, stop: stop.wait(seconds / speed)


def main(argv: Sequence[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    logging.basicConfig(
        level=getattr(logging, str(args.log_level).upper(), logging.INFO),
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
        stream=sys.stderr,
    )
    if args.ticks is not None and args.ticks < 1:
        parser.error("--ticks must be >= 1")
    if args.sim_speed < 0:
        parser.error("--sim-speed must be >= 0")

    try:
        app = load_config(args.config)
    except ConfigError as exc:
        _LOG.error("config: %s", exc)
        return 2
    cfg = app.mpc

    try:
        source, sink, release = build_io(app, args.source)
    except ConfigError as exc:
        _LOG.error("config: %s", exc)
        return 2
    except Exception as exc:
        _LOG.error("cannot build source %r: %s", args.source, exc)
        return 3

    notifier = SdNotifier()
    supervisor = Supervisor(cfg, version=VERSION)
    sleep = _make_sleep(args.sim_speed) if args.source == "sim" else None
    loop = Loop(source, sink, cfg, supervisor, clock=time.monotonic, notifier=notifier, sleep=sleep)

    http_service, mqtt_service = start_publishers(app, supervisor)
    if mqtt_service is not None:
        loop.on_tick = mqtt_service.on_tick

    stop = threading.Event()
    # Signal numbers the handler saw, logged by the main thread once the loop has
    # returned. The handler itself does nothing but set the event and append here:
    # a Python signal handler runs between two bytecodes of the main thread, so a
    # log call from it re-enters the stderr BufferedWriter whenever the signal
    # lands inside a write (the startup banner below, a tick log line) and raises
    # "RuntimeError: reentrant call inside <_io.BufferedWriter name='<stderr>'>".
    # On the Pi that exception left stop.set() unreached, the daemon ignored
    # SIGTERM and systemd had to SIGKILL it -- no fallback_pwm write. No logging,
    # no I/O, nothing that can raise in the handler. A second signal during
    # shutdown only appends another number and is otherwise ignored.
    signals_seen: list[int] = []

    def _on_signal(signum: int, _frame: object) -> None:
        stop.set()
        signals_seen.append(signum)

    signal.signal(signal.SIGTERM, _on_signal)
    signal.signal(signal.SIGINT, _on_signal)

    # Keep this banner *after* the handlers are installed: it is the externally
    # visible "SIGTERM now runs the stop path" marker (tests/test_main.py waits
    # for it before signalling the subprocess).
    _LOG.info(
        "aqua-bridge %s: source=%s dt=%.3g channels=%s notify=%s",
        VERSION,
        args.source,
        cfg.dt,
        list(cfg.channels),
        "on" if notifier.enabled else "off",
    )
    max_ticks = 1 if args.once else args.ticks
    exit_code = 0
    try:
        loop.run(stop, max_ticks=max_ticks)
        if args.once and loop.last_result is not None:
            r = loop.last_result
            payload = {
                "obs": r.obs.to_dict(),
                "cmd": r.cmd.to_dict(),
                "applied": r.applied,
                "read_error": r.read_error,
                "apply_error": r.apply_error,
            }
            print(json.dumps(payload, allow_nan=False, indent=2, sort_keys=True))
    except Exception:
        _LOG.exception("loop crashed")
        exit_code = 1
    finally:
        if signals_seen:
            _LOG.info("signal %s: stopping", _signal_name(signals_seen[0]))
        # Section 9 stop path: the only place fallback_pwm is written at exit.
        loop.shutdown()
        if release is not None:
            try:
                release()
            except Exception:
                _LOG.exception("release failed")
        stop_publishers(http_service, mqtt_service)
    if len(signals_seen) > 1:
        _LOG.info("%d further signal(s) during shutdown ignored", len(signals_seen) - 1)
    _LOG.info("exit %d after %d ticks", exit_code, loop.tick_count)
    return exit_code


def _signal_name(signum: int) -> str:
    try:
        return signal.Signals(signum).name
    except ValueError:
        return str(signum)


def start_publishers(app: AppConfig, supervisor: Supervisor) -> tuple[Any, Any]:
    """``(http_service | None, mqtt_service | None)`` per ``http.enabled`` / ``mqtt.enabled``.

    Every failure is logged and leaves that publisher off; the control loop
    must start regardless.
    """
    http_service = mqtt_service = None
    if app.section("http").get("enabled") is True:
        try:
            from aqua_bridge.publishers.runtime import HttpService

            service = HttpService(supervisor, app)
            if service.start():
                http_service = service
        except Exception:
            _LOG.exception("http: not started")
    if app.section("mqtt").get("enabled") is True:
        try:
            from aqua_bridge.publishers.runtime import MqttService

            mqtt_service = MqttService.from_config(app, supervisor)
            mqtt_service.start()
            _LOG.info("mqtt: connecting to %s:%s", app.mqtt.get("host"), app.mqtt.get("port", 1883))
        except Exception:
            _LOG.exception("mqtt: not started")
            mqtt_service = None
    return http_service, mqtt_service


def stop_publishers(http_service: Any, mqtt_service: Any) -> None:
    for name, service in (("mqtt", mqtt_service), ("http", http_service)):
        if service is None:
            continue
        try:
            service.stop()
        except Exception:
            _LOG.exception("%s: stop failed", name)


if __name__ == "__main__":
    sys.exit(main())
