"""Tests for tools/smart_agent.py (the DAS plan, section 1 "SMART path").

All pure/injectable: no real smartctl, no real disks, no real MQTT broker.
"""

from __future__ import annotations

import importlib.util
import subprocess
import sys
from pathlib import Path
from typing import Any

import pytest

_TOOLS = Path(__file__).resolve().parent.parent / "tools"


def _load_smart_agent():
    spec = importlib.util.spec_from_file_location("smart_agent", _TOOLS / "smart_agent.py")
    assert spec is not None and spec.loader is not None
    module = sys.modules.get("smart_agent")
    if module is None:
        module = importlib.util.module_from_spec(spec)
        sys.modules["smart_agent"] = module
        spec.loader.exec_module(module)
    return module


sa = _load_smart_agent()


# --- discover_devices --------------------------------------------------------------------


def test_discover_devices_finds_sata_and_nvme(tmp_path: Path) -> None:
    by_id = tmp_path / "by-id"
    by_id.mkdir()
    target = tmp_path / "sda"
    target.write_text("")
    (by_id / "ata-Samsung_SSD").symlink_to(target)

    nvme = tmp_path / "nvme0n1"
    nvme.write_text("")

    devices = sa.discover_devices(sata_glob=str(by_id / "*"), nvme_glob=str(tmp_path / "nvme*n1"))
    assert [d.kind for d in devices] == ["sata", "nvme"]
    assert devices[0].path.endswith("ata-Samsung_SSD")
    assert devices[1].path == str(nvme)


def test_discover_devices_dedups_by_id_aliases_of_the_same_disk(tmp_path: Path) -> None:
    by_id = tmp_path / "by-id"
    by_id.mkdir()
    target = tmp_path / "sda"
    target.write_text("")
    (by_id / "ata-Model_Serial").symlink_to(target)
    (by_id / "wwn-0x5000").symlink_to(target)

    devices = sa.discover_devices(sata_glob=str(by_id / "*"), nvme_glob=str(tmp_path / "nvme*n1"))
    assert len(devices) == 1
    # sorted() picks the alphabetically first alias
    assert devices[0].path.endswith("ata-Model_Serial")


def test_discover_devices_skips_partitions(tmp_path: Path) -> None:
    by_id = tmp_path / "by-id"
    by_id.mkdir()
    disk = tmp_path / "sda"
    disk.write_text("")
    part = tmp_path / "sda1"
    part.write_text("")
    (by_id / "ata-Disk").symlink_to(disk)
    (by_id / "ata-Disk-part1").symlink_to(part)

    devices = sa.discover_devices(sata_glob=str(by_id / "*"), nvme_glob=str(tmp_path / "nvme*n1"))
    assert len(devices) == 1
    assert devices[0].path.endswith("ata-Disk")


def test_discover_devices_default_nvme_glob_excludes_partitions(tmp_path: Path) -> None:
    (tmp_path / "nvme0n1").write_text("")
    (tmp_path / "nvme0n1p1").write_text("")
    devices = sa.discover_devices(
        sata_glob=str(tmp_path / "no-such-dir" / "*"), nvme_glob=str(tmp_path / "nvme*n1")
    )
    assert [d.path for d in devices] == [str(tmp_path / "nvme0n1")]


def test_discover_devices_empty_when_nothing_matches(tmp_path: Path) -> None:
    devices = sa.discover_devices(
        sata_glob=str(tmp_path / "by-id" / "*"), nvme_glob=str(tmp_path / "nvme*n1")
    )
    assert devices == []


# --- build_smartctl_args ------------------------------------------------------------------


def test_sata_args_include_n_standby() -> None:
    args = sa.build_smartctl_args(sa.Device(path="/dev/sda", kind="sata"), smartctl="smartctl")
    assert args == ["smartctl", "-j", "-n", "standby", "-A", "/dev/sda"]


def test_nvme_args_omit_n_standby() -> None:
    args = sa.build_smartctl_args(sa.Device(path="/dev/nvme0n1", kind="nvme"), smartctl="smartctl")
    assert args == ["smartctl", "-j", "-A", "/dev/nvme0n1"]


# --- run_smartctl_json: smartctl -j fixtures, including standby exit codes ----------------


class _Proc:
    def __init__(self, stdout: str, returncode: int = 0) -> None:
        self.stdout = stdout
        self.returncode = returncode


_ATA_JSON = (
    '{"serial_number": "WD-ABC123", "model_name": "WDC WD40EFRX-68N32N0", '
    '"temperature": {"current": 34}}'
)
_NVME_JSON = (
    '{"serial_number": "S4EWNX0N", "model_name": "Samsung SSD 980", "temperature": {"current": 41}}'
)
# smartctl -n standby, drive asleep: it exits without ever reading SMART
# attributes, so there is no "temperature" key at all -- this is the exact
# shape run_smartctl_json/extract_reading must turn into "no reading",
# never an error. The real exit status for the "device was in the
# specified mode" case is smartctl's own detail; a fixed value of 2 here
# stands in for it (this module deliberately does not gate on returncode).
_STANDBY_JSON = '{"serial_number": "WD-ABC123", "power_mode": "STANDBY"}'


def test_run_smartctl_json_parses_valid_output_regardless_of_returncode() -> None:
    def runner(args, **kw):
        return _Proc(_ATA_JSON, returncode=4)  # smartctl sets status bits even on success

    data = sa.run_smartctl_json(["smartctl", "-j", "-A", "/dev/sda"], runner=runner)
    assert data is not None
    assert data["serial_number"] == "WD-ABC123"


def test_run_smartctl_json_standby_exit_code_still_parses_as_no_temperature() -> None:
    def runner(args, **kw):
        return _Proc(_STANDBY_JSON, returncode=2)

    data = sa.run_smartctl_json(
        ["smartctl", "-j", "-n", "standby", "-A", "/dev/sda"], runner=runner
    )
    assert data is not None
    assert "temperature" not in data
    assert sa.extract_reading(data) is None


def test_run_smartctl_json_garbage_stdout_is_none() -> None:
    def runner(args, **kw):
        return _Proc("not json at all", returncode=1)

    assert sa.run_smartctl_json(["smartctl"], runner=runner) is None


def test_run_smartctl_json_non_object_json_is_none() -> None:
    def runner(args, **kw):
        return _Proc("[1, 2, 3]", returncode=0)

    assert sa.run_smartctl_json(["smartctl"], runner=runner) is None


def test_run_smartctl_json_missing_binary_is_none_not_raise() -> None:
    def runner(args, **kw):
        raise FileNotFoundError("no such file: smartctl")

    assert sa.run_smartctl_json(["smartctl"], runner=runner) is None


def test_run_smartctl_json_timeout_is_none_not_raise() -> None:
    def runner(args, **kw):
        raise subprocess.TimeoutExpired(cmd=args, timeout=kw.get("timeout", 1))

    assert sa.run_smartctl_json(["smartctl"], runner=runner) is None


# --- extract_reading ------------------------------------------------------------------------


def test_extract_reading_ata() -> None:
    import json as _json

    reading = sa.extract_reading(_json.loads(_ATA_JSON))
    assert reading == sa.SmartReading(
        serial="WD-ABC123", model="WDC WD40EFRX-68N32N0", temp_c=pytest.approx(34.0)
    )


def test_extract_reading_nvme() -> None:
    import json as _json

    reading = sa.extract_reading(_json.loads(_NVME_JSON))
    assert reading.serial == "S4EWNX0N"
    assert reading.temp_c == pytest.approx(41.0)


@pytest.mark.parametrize(
    "data",
    [
        {},
        {"temperature": {"current": 30}},  # no serial
        {"serial_number": "S1"},  # no temperature block
        {"serial_number": "S1", "temperature": {}},  # no current
        {"serial_number": "S1", "temperature": {"current": "hot"}},
        {"serial_number": "S1", "temperature": {"current": True}},
        {"serial_number": "", "temperature": {"current": 30}},
        {"serial_number": 5, "temperature": {"current": 30}},
        "not a mapping",
        None,
    ],
)
def test_extract_reading_returns_none_for_malformed_data(data: Any) -> None:
    assert sa.extract_reading(data) is None


def test_extract_reading_falls_back_to_none_model() -> None:
    reading = sa.extract_reading({"serial_number": "S1", "temperature": {"current": 30}})
    assert reading == sa.SmartReading(serial="S1", model=None, temp_c=30.0)


# --- poll_devices ---------------------------------------------------------------------------


def test_poll_devices_collects_readings_and_skips_unreadable() -> None:
    devices = [
        sa.Device(path="/dev/sda", kind="sata"),
        sa.Device(path="/dev/sdb", kind="sata"),  # asleep this cycle
        sa.Device(path="/dev/nvme0n1", kind="nvme"),
    ]

    def runner(args, **kw):
        path = args[-1]
        if path == "/dev/sda":
            return _Proc(_ATA_JSON)
        if path == "/dev/sdb":
            return _Proc(_STANDBY_JSON)
        if path == "/dev/nvme0n1":
            return _Proc(_NVME_JSON)
        raise AssertionError(f"unexpected args {args}")

    readings = sa.poll_devices(devices, runner=runner)
    assert {r.serial for r in readings} == {"WD-ABC123", "S4EWNX0N"}


def test_poll_devices_never_raises_on_a_broken_device() -> None:
    devices = [sa.Device(path="/dev/sda", kind="sata")]

    def runner(args, **kw):
        raise OSError("gone")

    assert sa.poll_devices(devices, runner=runner) == []


# --- payload / publish -----------------------------------------------------------------------


def test_reading_payload_shape_and_injected_clock() -> None:
    reading = sa.SmartReading(serial="S1", model="M1", temp_c=33.5)
    payload = sa.reading_payload(reading, now=lambda: 1_700_000_000.0)
    assert payload == {"serial": "S1", "model": "M1", "temp_c": 33.5, "ts_wall": 1_700_000_000.0}


def test_smart_topic_shape() -> None:
    assert sa.smart_topic("aqua-bridge", "S1") == "aqua-bridge/in/smart/S1"


class _FakeMqttClient:
    def __init__(self) -> None:
        self.published: list[tuple[str, str, int, bool]] = []
        self.stopped = False
        self.disconnected = False

    def publish(self, topic, payload=None, qos=0, retain=False):
        self.published.append((topic, payload, qos, retain))

    def loop_stop(self) -> None:
        self.stopped = True

    def disconnect(self) -> None:
        self.disconnected = True


def test_publish_readings_retains_one_message_per_reading() -> None:
    client = _FakeMqttClient()
    readings = [
        sa.SmartReading(serial="S1", model="M1", temp_c=30.0),
        sa.SmartReading(serial="S2", model=None, temp_c=45.0),
    ]
    sa.publish_readings(client, "aqua-bridge", readings, now=lambda: 42.0)
    assert len(client.published) == 2
    topic0, payload0, qos0, retain0 = client.published[0]
    assert topic0 == "aqua-bridge/in/smart/S1"
    assert qos0 == 1
    assert retain0 is True
    import json as _json

    assert _json.loads(payload0) == {"serial": "S1", "model": "M1", "temp_c": 30.0, "ts_wall": 42.0}


# --- CLI / main --------------------------------------------------------------------------


def test_build_parser_requires_mqtt() -> None:
    with pytest.raises(SystemExit):
        sa.build_parser().parse_args([])


def test_build_parser_defaults() -> None:
    args = sa.build_parser().parse_args(["--mqtt", "broker.lan"])
    assert args.host == "broker.lan"
    assert args.port == 1883
    assert args.node_id == "aqua-bridge"
    assert args.interval == sa.DEFAULT_INTERVAL_S
    assert args.once is False


def test_main_rejects_non_positive_interval() -> None:
    with pytest.raises(SystemExit):
        sa.main(["--mqtt", "broker.lan", "--interval", "0"])


def test_main_once_polls_and_publishes_then_shuts_down(monkeypatch: pytest.MonkeyPatch) -> None:
    fake_client = _FakeMqttClient()
    monkeypatch.setattr(sa, "build_mqtt_client", lambda **kw: fake_client)
    monkeypatch.setattr(
        sa, "discover_devices", lambda **kw: [sa.Device(path="/dev/sda", kind="sata")]
    )
    readings = [sa.SmartReading(serial="S1", model="M1", temp_c=33.0)]
    monkeypatch.setattr(sa, "poll_devices", lambda devices, **kw: readings)

    rc = sa.main(["--mqtt", "broker.lan", "--node-id", "test-node", "--once"])

    assert rc == 0
    assert len(fake_client.published) == 1
    topic, payload, qos, retain = fake_client.published[0]
    assert topic == "test-node/in/smart/S1"
    assert qos == 1 and retain is True
    import json as _json

    body = _json.loads(payload)
    assert body["serial"] == "S1" and body["model"] == "M1" and body["temp_c"] == 33.0
    assert isinstance(body["ts_wall"], float)
    assert fake_client.stopped is True
    assert fake_client.disconnected is True


def test_main_returns_1_when_mqtt_client_cannot_be_built(monkeypatch: pytest.MonkeyPatch) -> None:
    def boom(**kw):
        raise OSError("connection refused")

    monkeypatch.setattr(sa, "build_mqtt_client", boom)
    rc = sa.main(["--mqtt", "broker.lan", "--once"])
    assert rc == 1


def test_main_once_with_no_devices_publishes_nothing(monkeypatch: pytest.MonkeyPatch) -> None:
    fake_client = _FakeMqttClient()
    monkeypatch.setattr(sa, "build_mqtt_client", lambda **kw: fake_client)
    monkeypatch.setattr(sa, "discover_devices", lambda **kw: [])
    monkeypatch.setattr(sa, "poll_devices", lambda devices, **kw: [])

    rc = sa.main(["--mqtt", "broker.lan", "--once"])
    assert rc == 0
    assert fake_client.published == []
    assert fake_client.stopped is True
