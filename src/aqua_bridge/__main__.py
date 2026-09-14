"""``python -m aqua_bridge --config /etc/aqua-bridge/config.yaml`` (sections 9, 10, 11).

Builds the source/sink pair, the :class:`~aqua_bridge.control.supervisor.Supervisor`
and the :class:`~aqua_bridge.control.loop.Loop`, installs the SIGTERM/SIGINT
handler (which only sets the stop event and records the signal number; the
main thread then runs the section 9 stop path -- log, write ``fallback_pwm``,
``STOPPING=1``), and ticks at ``cfg.dt`` on ``time.monotonic``.

``--source xt6`` (default) imports :mod:`aqua_bridge.hw.xt6` lazily -- the
hardware adapter is the only place that knows sysfs. ``--source hwmon``
imports :mod:`aqua_bridge.hw.sources` instead and builds the DAS composite
source/sink (several hwmon devices from ``hwmon:``/``xt6:`` plus an optional
1-Wire bus from ``onewire:``, plan section 1 and section 12 Q1); it is a
separate choice rather than a generalisation of ``xt6`` so a config without
the new sections drives ``--source xt6`` bit for bit as today ("legacy
mode"). ``--source sim`` drives a simulated plant instead, for a laptop or
CI; ``--sim-plant`` picks it:

* ``basic`` (default): the RC plant from :mod:`aqua_bridge.sim.plant`,
  exactly as before the flag existed.
* ``rich``: the same RC plant with one tick of actuator delay and 0.02 degC
  sensor noise (a legacy config against a less ideal plant).
* ``das``: the DAS truth plant from :mod:`aqua_bridge.sim.das`. It needs a
  DAS topology: an optional top-level ``sim.das`` section
  (``{topology?, preset?, seed?}``) gives one explicitly; without
  ``sim.das.topology`` a zoned ``mpc`` config (``mpc.topology``) supplies
  the structure (:func:`aqua_bridge.sim.das.topology_from_config`, so
  ``config.example-das.yaml`` runs as it is), and a legacy ``mpc`` config
  with a ``sim.das`` section gets the simulator's default topology. Every
  ``mpc.temps`` / ``mpc.channels`` name must exist in that topology. A
  legacy config without ``sim.das``, or missing names: :class:`ConfigError`,
  exit code 2.

HTTP (``http.enabled``) and MQTT (``mqtt.enabled``) run next to the loop via
:mod:`aqua_bridge.publishers.runtime`; both attach to the supervisor's
``ControlSurface``. A publisher that fails to start (port in use, broker
down, or a bad scalar anywhere in its section -- item 57) is logged and the
daemon keeps controlling the fans without it (:func:`start_publishers`). One
:class:`~aqua_bridge.publishers.inputs.SmartInbox` is built here every run
and threaded through both: ``build_io`` gives it to ``--source hwmon``'s
``CompositeSource``, ``start_publishers`` wires ``POST /api/in/smart`` and
the MQTT ``{node_id}/in/smart/+`` topic into it (plan section 1).

Model store (:mod:`aqua_bridge.modelstore`): with a zoned DAS config the daemon loads
``--model-store PATH``, else ``$STATE_DIRECTORY/model.json`` (systemd's
``StateDirectory=aqua-bridge``), into the first state and keeps it current through a
:class:`~aqua_bridge.modelstore.ModelPersister` on ``on_tick``, written once more at a
clean stop. A legacy config ignores ``STATE_DIRECTORY``; ``--model-store`` with one is a
configuration error (exit code 2).

Exit codes: ``0`` clean stop, ``2`` bad arguments or config, ``3`` the
source/sink could not be built.
"""

from __future__ import annotations

import argparse
import json
import logging
import os
import signal
import sys
import threading
import time
from collections.abc import Callable, Mapping, Sequence
from typing import Any

from aqua_bridge.config import AppConfig, ConfigError, load_config
from aqua_bridge.control.loop import Loop, Sink, Source
from aqua_bridge.control.supervisor import Supervisor
from aqua_bridge.model import MpcCommand, MpcConfig, MpcState, PlantObservation
from aqua_bridge.modelstore import ModelPersister, initial_state, store_path
from aqua_bridge.modelstore import load as load_model_store
from aqua_bridge.publishers.httpauth import HttpSettings, HttpSetupError
from aqua_bridge.publishers.inputs import SmartInbox, smart_topic_filter
from aqua_bridge.publishers.mqtt_ha import MqttSetupError, validate_mqtt_section
from aqua_bridge.recorder import DEFAULT_BACKUP_COUNT, DEFAULT_MAX_BYTES, Recorder, chain_on_tick
from aqua_bridge.sdnotify import SdNotifier

__all__ = [
    "PlantIO",
    "build_io",
    "build_model_store",
    "build_parser",
    "build_recorder",
    "main",
    "make_das_sim_plant",
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


def make_sim_plant(
    cfg: MpcConfig, *, heat_w: float = 100.0, seed: int = 0, rich: bool = False
) -> Any:
    """An RC plant whose channel and temperature names follow ``cfg``.

    Channels whose name contains ``"intake"`` move case air; every other
    channel is a radiator fan. The first temperature with a setpoint is the
    coolant, the first other temperature (if any) the air. ``rich`` adds one
    tick of actuator delay and 0.02 degC sensor noise.
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
        **({"delay_ticks": 1, "noise_sigma_c": 0.02} if rich else {}),
    )
    return Plant(params, initial_pwm=dict(cfg.fallback_pwm), seed=seed)


SIM_PLANTS: tuple[str, ...] = ("basic", "rich", "das")


def make_das_sim_plant(app: AppConfig) -> Any:
    """A DAS truth plant from ``sim.das`` and / or the zoned ``mpc`` config (module docstring).

    Raises :class:`ConfigError` with a clear message when no topology is
    available, the section is malformed, or the ``mpc`` names do not exist in
    the topology.
    """
    from aqua_bridge.sim.das import build_das_plant, default_topology, topology_from_config

    sim = app.section("sim")
    das = sim.get("das") if isinstance(sim, dict) else None
    if das is None and app.mpc.is_das:
        das = {}
    if not isinstance(das, dict):
        raise ConfigError(
            "--sim-plant das needs a DAS topology: add a top-level section "
            "'sim: {das: {topology: {zones, bays, fans, sensors}, preset: basic|rich, seed: 0}}' "
            "(the zoned mpc config is not available yet)"
        )
    unknown = sorted(set(das) - {"topology", "preset", "seed"})
    if unknown:
        raise ConfigError(f"sim.das: unknown keys {unknown}")
    topology = das.get("topology")
    seed = das.get("seed", 0)
    if isinstance(seed, bool) or not isinstance(seed, int):
        raise ConfigError("sim.das.seed must be an integer")
    cfg = app.mpc
    try:
        plant = build_das_plant(
            (topology_from_config(cfg) if cfg.is_das else default_topology())
            if topology is None
            else topology,
            preset=str(das.get("preset", "basic")),
            seed=seed,
            dt=cfg.dt,
            initial_pwm=dict(cfg.fallback_pwm),
        )
    except (TypeError, ValueError) as exc:
        raise ConfigError(f"sim.das: {exc}") from exc
    problems = []
    missing_t = [t for t in cfg.temps if t not in plant.params.sensor_names]
    missing_c = [c for c in cfg.channels if c not in plant.params.channels]
    if missing_t:
        problems.append(f"mpc.temps {missing_t}")
    if missing_c:
        problems.append(f"mpc.channels {missing_c}")
    if problems:
        raise ConfigError("sim.das topology does not provide " + "; ".join(problems))
    return plant


class PlantIO:
    """Source and sink over one simulated plant (``sim.plant.Plant`` or ``sim.das.DasPlant``).

    ``read`` reports the plant's current observation restricted to
    ``cfg.temps`` (plus the DAS plant's SMART view as ``inputs["smart"]`` when a
    drive reports one); ``apply`` feeds the command in and advances the plant
    one ``dt``, so the observation clock is the plant's ``ts``.
    """

    def __init__(self, plant: Any, cfg: MpcConfig) -> None:
        self.plant = plant
        self.cfg = cfg
        self.applied: list[MpcCommand] = []

    def read(self) -> PlantObservation:
        obs = self.plant.observe()
        temps = {name: obs.temps.get(name) for name in self.cfg.temps}
        observe_smart = getattr(self.plant, "observe_smart", None)
        smart = observe_smart() if callable(observe_smart) else {}
        return PlantObservation(
            temps=temps,
            rpm=dict(obs.rpm),
            pwm=dict(obs.pwm),
            ts=obs.ts,
            inputs={"smart": smart} if smart else {},
        )

    def apply(self, cmd: MpcCommand) -> None:
        self.applied.append(cmd)
        self.plant.apply(cmd.pwm)
        self.plant.advance()


# ---------------------------------------------------------------------------
# Wiring
# ---------------------------------------------------------------------------


def build_io(
    app: AppConfig,
    source: str,
    *,
    clock: Callable[[], float] = time.monotonic,
    sim_plant: str = "basic",
    smart: Any = None,
) -> tuple[Source, Sink, Callable[[], None] | None]:
    """``(source, sink, release)`` for ``--source``; ``release`` runs at exit if not None.

    ``smart`` (a :class:`~aqua_bridge.publishers.inputs.SmartInbox` or
    ``None``) is only meaningful for ``--source hwmon``, where it becomes
    ``CompositeSource.smart`` and so shows up in ``PlantObservation.inputs
    ["smart"]`` every tick; the ``sim``/``xt6`` sources have no ``inputs``
    concept and silently ignore it (SMART data with no drive-adjacent DAS
    hardware to correlate it against is a later milestone's problem, not a
    reason to error here).
    """
    if source == "sim":
        if sim_plant == "basic":
            plant = make_sim_plant(app.mpc)
        elif sim_plant == "rich":
            plant = make_sim_plant(app.mpc, rich=True)
        elif sim_plant == "das":
            plant = make_das_sim_plant(app)
        else:
            raise RuntimeError(f"unknown sim plant {sim_plant!r}")
        io = PlantIO(plant, app.mpc)
        return io, io, None
    if source == "xt6":
        try:
            from aqua_bridge.hw import xt6
        except ImportError as exc:
            raise RuntimeError(f"hardware adapter unavailable: {exc}") from exc
        builder = getattr(xt6, "build_map_from_config", None)
        if builder is None:
            raise RuntimeError("aqua_bridge.hw.xt6.build_map_from_config is missing")
        # Cross-check xt6.fans / xt6.temp_map against mpc.channels / mpc.temps
        # here, before the loop starts (ConfigError, exit code 2).
        hwmon_map = builder(app.section("xt6"), channels=app.mpc.channels, temps=app.mpc.temps)
        adapter = xt6.Xt6Adapter(hwmon_map, clock=clock)
        return adapter, adapter, None
    if source == "hwmon":
        try:
            from aqua_bridge.hw import sources as hw_sources
        except ImportError as exc:
            raise RuntimeError(f"hardware adapter unavailable: {exc}") from exc
        composite, release = hw_sources.build_composite_from_config(
            hwmon_section=app.hwmon,
            xt6_section=app.section("xt6"),
            onewire_section=app.section("onewire"),
            channels=app.mpc.channels,
            temps=app.mpc.temps,
            dt=app.mpc.dt,
            smart=smart,
            clock=clock,
        )
        return composite, composite, release
    raise RuntimeError(f"unknown source {source!r}")


def build_recorder(app: AppConfig, cfg: MpcConfig, cli_path: str | None) -> Recorder | None:
    """A :class:`~aqua_bridge.recorder.Recorder` from ``--record`` or the config's
    top-level ``record_path`` (``--record`` wins; neither given: ``None``, no
    recording). ``record_max_bytes``/``record_backup_count`` are read the same way,
    from :attr:`AppConfig.extra` -- flat top-level scalars, not a typed section, the
    same treatment the interim ``sim.das`` section got before it had one (recorder.py
    module docstring)."""
    path = cli_path if cli_path is not None else app.extra.get("record_path")
    if path is None:
        return None
    max_bytes = app.extra.get("record_max_bytes", DEFAULT_MAX_BYTES)
    backup_count = app.extra.get("record_backup_count", DEFAULT_BACKUP_COUNT)
    try:
        return Recorder(cfg, str(path), max_bytes=int(max_bytes), backup_count=int(backup_count))
    except (TypeError, ValueError) as exc:
        raise ConfigError(f"record_max_bytes/record_backup_count must be integers: {exc}") from exc


def build_model_store(
    cfg: MpcConfig,
    cli_path: str | None,
    *,
    env: Mapping[str, str] | None = None,
    wall: Callable[[], float] = time.time,
) -> tuple[MpcState | None, ModelPersister | None]:
    """``(initial state, persister)`` for the model store, or ``(None, None)`` without one
    (module docstring). Raises :class:`ConfigError` for ``--model-store`` with a legacy
    config; a missing or unusable file is not an error (the controller starts on its
    prior and the reason is logged)."""
    path = store_path(cfg, cli_path, os.environ if env is None else env)
    if path is None:
        return None, None
    result = load_model_store(path, cfg, now_wall=wall())
    if result.missing:
        _LOG.info("model store: %s does not exist yet; starting on the prior", path)
    else:
        age = "unknown" if result.age_s is None else f"{result.age_s / 86400.0:.2f} days"
        _LOG.info("model store: %s is %s (age %s)", path, result.source, age)
    return initial_state(result), ModelPersister(cfg, path, wall=wall)


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(prog="aqua_bridge", description=__doc__.split("\n\n")[0])
    p.add_argument("--config", required=True, help="path to config.yaml")
    p.add_argument(
        "--source",
        choices=("xt6", "hwmon", "sim"),
        default="xt6",
        help=(
            "xt6: single aquaero over hwmon (default); hwmon: the DAS composite "
            "(hwmon: devices + onewire:); sim: RC plant simulator"
        ),
    )
    p.add_argument(
        "--sim-plant",
        choices=SIM_PLANTS,
        default=None,
        help="sim only: basic RC plant (default), rich RC plant, das truth plant",
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
    p.add_argument(
        "--record",
        metavar="PATH",
        default=None,
        help="record every tick as JSONL to PATH (overrides the config's record_path)",
    )
    p.add_argument(
        "--model-store",
        metavar="PATH",
        default=None,
        help=(
            "DAS configs: load and keep the thermal model and calibrations in PATH "
            "(default $STATE_DIRECTORY/model.json when set)"
        ),
    )
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
    if args.sim_plant is not None and args.source != "sim":
        parser.error("--sim-plant needs --source sim")

    try:
        app = load_config(args.config)
        cfg = app.mpc
        recorder = build_recorder(app, cfg, args.record)
        initial, persister = build_model_store(cfg, args.model_store)
    except ConfigError as exc:
        _LOG.error("config: %s", exc)
        return 2

    # Built regardless of --source: the MQTT/HTTP inbound side (below) works
    # the same whichever plant is behind the loop, and only --source hwmon's
    # CompositeSource actually reads it into PlantObservation.inputs (the
    # module docstring on build_io).
    smart_inbox = SmartInbox()

    try:
        source, sink, release = build_io(
            app, args.source, sim_plant=args.sim_plant or "basic", smart=smart_inbox
        )
    except ConfigError as exc:
        _LOG.error("config: %s", exc)
        return 2
    except Exception as exc:
        _LOG.error("cannot build source %r: %s", args.source, exc)
        return 3

    notifier = SdNotifier()
    supervisor = Supervisor(cfg, version=VERSION)
    sleep = _make_sleep(args.sim_speed) if args.source == "sim" else None
    loop = Loop(
        source,
        sink,
        cfg,
        supervisor,
        clock=time.monotonic,
        notifier=notifier,
        sleep=sleep,
        state=initial,
    )

    http_service, mqtt_service = start_publishers(app, supervisor, smart_inbox=smart_inbox)
    hooks = [h.on_tick if h is not None else None for h in (mqtt_service, recorder, persister)]
    if any(h is not None for h in hooks):
        loop.on_tick = chain_on_tick(*hooks)

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
        if recorder is not None:
            recorder.close()
        if persister is not None:
            persister.close()
    if len(signals_seen) > 1:
        _LOG.info("%d further signal(s) during shutdown ignored", len(signals_seen) - 1)
    _LOG.info("exit %d after %d ticks", exit_code, loop.tick_count)
    return exit_code


def _signal_name(signum: int) -> str:
    try:
        return signal.Signals(signum).name
    except ValueError:
        return str(signum)


def start_publishers(
    app: AppConfig, supervisor: Supervisor, *, smart_inbox: Any = None
) -> tuple[Any, Any]:
    """``(http_service | None, mqtt_service | None)`` per ``http.enabled`` / ``mqtt.enabled``.

    Every failure is logged and leaves that publisher off; the control loop
    must start regardless. ``smart_inbox`` (optional) is wired into both:
    ``HttpService`` gets it directly (``POST /api/in/smart``), and the MQTT
    client gets ``add_topic_handler(smart_topic_filter(node_id), ...)`` on
    the same connection it already builds for commands (one broker
    connection for the whole daemon).

    Item 57: ``enabled`` (and every other scalar in ``http:`` / ``mqtt:``) is parsed by
    :class:`~aqua_bridge.publishers.httpauth.HttpSettings` /
    :func:`~aqua_bridge.publishers.mqtt_ha.validate_mqtt_section` *before* deciding
    whether to start -- a value such as ``enabled: "true"`` (a string, not a bool)
    raises there and is logged by name instead of silently reading as ``False``
    (``"true" is True`` is ``False``, which used to leave the section looking merely
    disabled, with no log line at all).
    """
    http_service = mqtt_service = None
    try:
        http_settings = HttpSettings.from_section(app.section("http"))
    except HttpSetupError as exc:
        _LOG.error("http: not started: %s", exc)
        http_settings = None
    if http_settings is not None and http_settings.enabled:
        try:
            from aqua_bridge.publishers.runtime import HttpService

            service = HttpService(supervisor, app, smart_inbox=smart_inbox)
            if service.start():
                http_service = service
        except Exception:
            _LOG.exception("http: not started")
    try:
        mqtt_settings = validate_mqtt_section(app.section("mqtt"))
    except MqttSetupError as exc:
        _LOG.error("mqtt: not started: %s", exc)
        mqtt_settings = None
    if mqtt_settings is not None and mqtt_settings["enabled"]:
        try:
            from aqua_bridge.publishers.runtime import MqttService

            mqtt_service = MqttService.from_config(app, supervisor)
            if smart_inbox is not None:
                # Its own try/except: a client that does not support extra
                # topic handlers (a test fake, some future alternate
                # implementation) must not take the whole MQTT service down
                # with it -- commands/state still matter without SMART.
                try:
                    mqtt_service.client.add_topic_handler(
                        smart_topic_filter(mqtt_settings["node_id"]), smart_inbox.on_message
                    )
                except Exception:
                    _LOG.exception("mqtt: smart inbox wiring failed")
            mqtt_service.start()
            _LOG.info("mqtt: connecting to %s:%s", mqtt_settings["host"], mqtt_settings["port"])
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
