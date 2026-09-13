#!/usr/bin/env python3
"""PC-side SMART temperature agent (the DAS plan, section 1 "SMART path").

Runs on the machine that actually has the drives attached over SATA/SAS/
NVMe (not the Pi -- the DAS has no SES backplane, so the Pi cannot read
SMART for drives it does not itself see, plan section 1). Every
``--interval`` seconds it discovers the attached drives, reads one SMART
attribute snapshot from each with ``smartctl -j``, and publishes a retained
MQTT message per drive to ``{node_id}/in/smart/<serial>`` --
:class:`~aqua_bridge.publishers.inputs.SmartInbox` on the Pi side is the
consumer (either via this MQTT topic or the ``POST /api/in/smart`` twin, in
case a given SMART source runs somewhere MQTT is inconvenient).

Never wakes a standby drive: SATA/SAS reads use ``smartctl -n standby``, so
a drive that is asleep skips the SMART query entirely (smartctl checks the
power mode with a lightweight command first) and simply produces no
reading this cycle -- not an error, just nothing to publish for that
serial until it spins up on its own. NVMe has no equivalent power-mode
concern for this purpose (plan section 1), so NVMe devices are read with
plain ``smartctl -j -A`` every cycle.

No SES, so this agent does not and cannot know which bay a serial sits in
(association is the estimator milestone's job, plan section 1 point 4);
it publishes by serial only.

Run standalone (see ``deploy/aqua-bridge-smart-agent.service`` for a
systemd *user* unit example -- this runs on the PC, not as the Pi's system
service)::

    python tools/smart_agent.py --mqtt broker.lan --node-id aqua-bridge --interval 60

Every function here down to :func:`poll_devices` is pure/injectable
(explicit ``runner``/``clock`` parameters) so the whole cycle is testable
against ``smartctl -j`` fixtures without a real disk or broker
(``tests/test_smart_agent.py``).
"""

from __future__ import annotations

import argparse
import glob
import json
import logging
import math
import os
import signal
import subprocess
import sys
import threading
import time
from collections.abc import Callable, Mapping, Sequence
from dataclasses import dataclass
from typing import Any

__all__ = [
    "DEFAULT_INTERVAL_S",
    "DEFAULT_NVME_GLOB",
    "DEFAULT_SATA_GLOB",
    "DEFAULT_SMARTCTL",
    "Device",
    "SmartReading",
    "build_mqtt_client",
    "build_parser",
    "build_smartctl_args",
    "discover_devices",
    "extract_reading",
    "main",
    "poll_devices",
    "publish_readings",
    "reading_payload",
    "run_smartctl_json",
    "smart_topic",
]

_LOG = logging.getLogger("smart_agent")

DEFAULT_SATA_GLOB = "/dev/disk/by-id/*"
#: A single-namespace assumption (plan section 1 does not go further); a
#: drive with more than one active namespace needs an explicit glob.
DEFAULT_NVME_GLOB = "/dev/nvme*n1"
DEFAULT_INTERVAL_S = 60.0
DEFAULT_SMARTCTL = "smartctl"
DEFAULT_TIMEOUT_S = 10.0


@dataclass(frozen=True)
class Device:
    path: str
    kind: str  # "sata" | "nvme"


@dataclass(frozen=True)
class SmartReading:
    serial: str
    model: str | None
    temp_c: float


# ---------------------------------------------------------------------------
# Discovery
# ---------------------------------------------------------------------------


def discover_devices(
    *, sata_glob: str = DEFAULT_SATA_GLOB, nvme_glob: str = DEFAULT_NVME_GLOB
) -> list[Device]:
    """Every SATA/SAS ``by-id`` path and NVMe namespace device to poll.

    ``/dev/disk/by-id`` lists several symlinks per physical disk (``ata-...``,
    ``wwn-...``, ``scsi-...``); duplicates that resolve to the same block
    device are kept once (first by sorted path), and partition symlinks
    (name contains ``-part``) are skipped -- smartctl wants the whole-disk
    node. The default NVMe glob (``nvme*n1``) already excludes partition
    nodes (``nvme0n1p1`` ends in ``p1``, not ``n1``) without extra filtering.
    """
    seen_real: set[str] = set()
    out: list[Device] = []
    for path in sorted(glob.glob(sata_glob)):
        if "-part" in os.path.basename(path):
            continue
        try:
            real = os.path.realpath(path)
        except OSError:
            real = path
        if real in seen_real:
            continue
        seen_real.add(real)
        out.append(Device(path=path, kind="sata"))
    for path in sorted(glob.glob(nvme_glob)):
        out.append(Device(path=path, kind="nvme"))
    return out


# ---------------------------------------------------------------------------
# smartctl
# ---------------------------------------------------------------------------


def build_smartctl_args(device: Device, *, smartctl: str = DEFAULT_SMARTCTL) -> list[str]:
    """``smartctl`` argv for one device. ``-n standby`` (SATA/SAS only) is
    what keeps a sleeping drive asleep -- it makes smartctl check the power
    mode first and skip the attribute read entirely if the drive reports
    standby, per the module docstring."""
    args = [smartctl, "-j"]
    if device.kind == "sata":
        args += ["-n", "standby"]
    args += ["-A", device.path]
    return args


def run_smartctl_json(
    args: Sequence[str],
    *,
    timeout_s: float = DEFAULT_TIMEOUT_S,
    runner: Callable[..., subprocess.CompletedProcess] = subprocess.run,
) -> dict[str, Any] | None:
    """Runs ``smartctl`` and parses its JSON stdout.

    Never raises: a missing binary, a timeout, a nonzero exit with no
    parseable JSON, or JSON that is not an object all become ``None`` --
    "no reading this cycle", never a crash. smartctl routinely exits
    nonzero for reasons that are not failures here (SMART status bits,
    ``-n standby`` finding the drive asleep) while still printing a valid
    JSON object, so ``returncode`` is deliberately not checked -- only
    whether ``stdout`` parses as a JSON object.
    """
    try:
        proc = runner(list(args), capture_output=True, text=True, timeout=timeout_s)
    except (OSError, subprocess.TimeoutExpired) as exc:
        _LOG.warning("smartctl failed to run (%s): %s", args, exc)
        return None
    try:
        data = json.loads(proc.stdout)
    except (json.JSONDecodeError, TypeError):
        _LOG.debug(
            "smartctl produced no JSON for %s (exit %r): %r",
            args,
            getattr(proc, "returncode", None),
            (proc.stdout or "")[:200],
        )
        return None
    if not isinstance(data, dict):
        return None
    return data


def extract_reading(data: Mapping[str, Any]) -> SmartReading | None:
    """Pulls ``{serial, model, temp_c}`` out of smartctl's JSON.

    ATA and NVMe both report a top-level ``temperature.current`` and
    ``serial_number`` in smartmontools' JSON schema (>= 7.0), so one
    extractor covers both. ``None`` when the drive did not report a
    temperature this cycle -- standby (``-n standby`` short-circuits before
    smartctl ever populates ``temperature``), a transient read error, or a
    device smartctl could not identify -- never raises on an odd shape.
    """
    if not isinstance(data, Mapping):
        return None
    serial = data.get("serial_number")
    if not isinstance(serial, str) or not serial:
        return None
    temp = data.get("temperature")
    if not isinstance(temp, Mapping):
        return None
    current = temp.get("current")
    if isinstance(current, bool) or not isinstance(current, int | float):
        return None
    temp_c = float(current)
    if not math.isfinite(temp_c):
        return None
    model = data.get("model_name")
    if not isinstance(model, str):
        model = None
    return SmartReading(serial=serial, model=model, temp_c=temp_c)


def poll_devices(
    devices: Sequence[Device],
    *,
    smartctl: str = DEFAULT_SMARTCTL,
    timeout_s: float = DEFAULT_TIMEOUT_S,
    runner: Callable[..., subprocess.CompletedProcess] = subprocess.run,
) -> list[SmartReading]:
    """One ``smartctl`` read per device; devices with no reading this cycle
    (standby, a read error) are simply absent from the result -- never an
    exception, so one bad drive never stops the rest."""
    readings: list[SmartReading] = []
    for device in devices:
        args = build_smartctl_args(device, smartctl=smartctl)
        data = run_smartctl_json(args, timeout_s=timeout_s, runner=runner)
        if data is None:
            continue
        reading = extract_reading(data)
        if reading is not None:
            readings.append(reading)
    return readings


# ---------------------------------------------------------------------------
# Publishing
# ---------------------------------------------------------------------------


def smart_topic(node_id: str, serial: str) -> str:
    return f"{node_id}/in/smart/{serial}"


def reading_payload(
    reading: SmartReading, *, now: Callable[[], float] = time.time
) -> dict[str, Any]:
    """``{serial, model, temp_c, ts_wall}`` -- the plan's exact SMART payload
    shape. ``ts_wall`` is informational wall-clock only; the Pi-side
    ``SmartInbox`` stamps its own monotonic receipt time and never trusts
    this one for staleness."""
    return {
        "serial": reading.serial,
        "model": reading.model,
        "temp_c": reading.temp_c,
        "ts_wall": now(),
    }


def publish_readings(
    client: Any,
    node_id: str,
    readings: Sequence[SmartReading],
    *,
    now: Callable[[], float] = time.time,
) -> None:
    """Publishes one retained message per reading. ``client`` is duck-typed
    (needs only ``.publish(topic, payload=..., qos=..., retain=...)``, the
    same call shape ``paho.mqtt.client.Client`` and
    :class:`~aqua_bridge.publishers.mqtt_ha.MqttClient` both use) so tests
    inject a fake without a broker."""
    for reading in readings:
        payload = json.dumps(reading_payload(reading, now=now), allow_nan=False)
        client.publish(smart_topic(node_id, reading.serial), payload=payload, qos=1, retain=True)


def build_mqtt_client(*, host: str, port: int, username: str, password: str, node_id: str) -> Any:
    """A connected, network-threaded ``paho-mqtt`` 2.x client. Imported
    lazily (same reason as
    :class:`~aqua_bridge.publishers.mqtt_ha.MqttClient`): this agent has no
    other use for ``paho-mqtt`` at import time, and the module must stay
    importable (for the pure functions above) without it installed."""
    import paho.mqtt.client as mqtt

    client = mqtt.Client(
        callback_api_version=mqtt.CallbackAPIVersion.VERSION2,
        client_id=f"aqua-bridge-smart-agent-{node_id}",
    )
    if username:
        client.username_pw_set(username, password or None)
    client.reconnect_delay_set(min_delay=1, max_delay=60)
    client.connect_async(host, port)
    client.loop_start()
    return client


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(prog="smart_agent", description=__doc__.split("\n\n")[0])
    p.add_argument("--mqtt", required=True, dest="host", metavar="HOST", help="MQTT broker host")
    p.add_argument("--port", type=int, default=1883)
    p.add_argument("--node-id", default="aqua-bridge", help="matches config.yaml mqtt.node_id")
    p.add_argument(
        "--interval", type=float, default=DEFAULT_INTERVAL_S, help="seconds between polling cycles"
    )
    p.add_argument("--username", default="")
    p.add_argument("--password", default="")
    p.add_argument("--sata-glob", default=DEFAULT_SATA_GLOB)
    p.add_argument("--nvme-glob", default=DEFAULT_NVME_GLOB)
    p.add_argument("--smartctl", default=DEFAULT_SMARTCTL, help="smartctl binary path/name")
    p.add_argument("--timeout", type=float, default=DEFAULT_TIMEOUT_S, help="smartctl timeout, s")
    p.add_argument(
        "--once", action="store_true", help="one polling cycle, then exit (cron use, testing)"
    )
    p.add_argument("--log-level", default="INFO", help="DEBUG, INFO, WARNING, ERROR")
    return p


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    logging.basicConfig(
        level=getattr(logging, str(args.log_level).upper(), logging.INFO),
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
        stream=sys.stderr,
    )
    if args.interval <= 0:
        parser.error("--interval must be > 0")
    if args.timeout <= 0:
        parser.error("--timeout must be > 0")

    try:
        client = build_mqtt_client(
            host=args.host,
            port=args.port,
            username=args.username,
            password=args.password,
            node_id=args.node_id,
        )
    except Exception:
        _LOG.exception("cannot start MQTT client for %s:%s", args.host, args.port)
        return 1

    stop = threading.Event()

    def _on_signal(signum: int, _frame: object) -> None:
        stop.set()

    signal.signal(signal.SIGTERM, _on_signal)
    signal.signal(signal.SIGINT, _on_signal)

    _LOG.info(
        "smart_agent: node_id=%s mqtt=%s:%s interval=%.0fs",
        args.node_id,
        args.host,
        args.port,
        args.interval,
    )
    try:
        while True:
            devices = discover_devices(sata_glob=args.sata_glob, nvme_glob=args.nvme_glob)
            readings = poll_devices(devices, smartctl=args.smartctl, timeout_s=args.timeout)
            publish_readings(client, args.node_id, readings)
            _LOG.info(
                "smart_agent: polled %d device(s), published %d reading(s)",
                len(devices),
                len(readings),
            )
            if args.once or stop.wait(args.interval):
                break
    finally:
        try:
            client.loop_stop()
            client.disconnect()
        except Exception:
            _LOG.exception("mqtt: shutdown failed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
