"""``python -m aqua_bridge``: argument handling, sim run, SIGTERM stop path (sections 9, 11)."""

from __future__ import annotations

import dataclasses
import json
import logging
import os
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path

import pytest
import yaml

from aqua_bridge import __main__ as main_mod
from aqua_bridge import hostinfo as hostinfo_mod
from aqua_bridge.config import ConfigError, load_config
from aqua_bridge.health import FanHealthConfig
from aqua_bridge.model import Mode, MpcCommand

REPO = Path(__file__).resolve().parent.parent


@pytest.fixture
def restore_signals():
    old_term = signal.getsignal(signal.SIGTERM)
    old_int = signal.getsignal(signal.SIGINT)
    yield
    signal.signal(signal.SIGTERM, old_term)
    signal.signal(signal.SIGINT, old_int)


@pytest.fixture
def recorded_applies(monkeypatch):
    """Capture every command the sim sink receives during main()."""
    applied: list[MpcCommand] = []
    original = main_mod.PlantIO.apply

    def apply(self, cmd):
        applied.append(cmd)
        original(self, cmd)

    monkeypatch.setattr(main_mod.PlantIO, "apply", apply)
    return applied


# --- parser ---------------------------------------------------------------------------


def test_config_is_required():
    with pytest.raises(SystemExit) as exc:
        main_mod.build_parser().parse_args([])
    assert exc.value.code == 2


def test_defaults():
    args = main_mod.build_parser().parse_args(["--config", "x.yaml"])
    assert args.source == "xt6" and args.once is False and args.ticks is None
    assert args.sim_speed == 1.0


def test_bad_source_rejected():
    with pytest.raises(SystemExit):
        main_mod.build_parser().parse_args(["--config", "x", "--source", "hid"])


def test_ticks_must_be_positive(example_config_path, restore_signals):
    with pytest.raises(SystemExit):
        main_mod.main(["--config", str(example_config_path), "--source", "sim", "--ticks", "0"])


def test_missing_config_exits_2(tmp_path, restore_signals):
    assert main_mod.main(["--config", str(tmp_path / "nope.yaml"), "--source", "sim"]) == 2


def test_invalid_config_exits_2(tmp_path, restore_signals):
    bad = tmp_path / "bad.yaml"
    bad.write_text("mpc:\n  dt: 0\n")
    assert main_mod.main(["--config", str(bad), "--source", "sim"]) == 2


# --- sim runs -------------------------------------------------------------------------


def test_sim_ticks_5_exits_0_and_writes_fallback_at_exit(
    example_config_path, restore_signals, recorded_applies
):
    rc = main_mod.main(
        [
            "--config",
            str(example_config_path),
            "--source",
            "sim",
            "--ticks",
            "5",
            "--sim-speed",
            "0",
        ]
    )
    assert rc == 0
    cfg = load_config(example_config_path).mpc
    assert len(recorded_applies) == 6  # 5 ticks + the shutdown write
    assert recorded_applies[-1].pwm == dict(cfg.fallback_pwm)
    assert recorded_applies[-1].mode is Mode.FALLBACK
    assert recorded_applies[-1].diagnostics["policy"] == "shutdown"
    for c in recorded_applies[:5]:
        assert c.diagnostics["policy"] != "shutdown"


def test_once_prints_command_json(example_config_path, restore_signals, recorded_applies, capsys):
    rc = main_mod.main(
        ["--config", str(example_config_path), "--source", "sim", "--once", "--sim-speed", "0"]
    )
    assert rc == 0
    out = json.loads(capsys.readouterr().out)
    assert set(out) == {"obs", "cmd", "applied", "read_error", "apply_error"}
    assert out["applied"] is True
    assert set(out["cmd"]["pwm"]) == {"radiator", "intake"}
    assert len(recorded_applies) == 2  # one tick + shutdown


def test_sim_speed_scales_sleep():
    assert main_mod._make_sleep(1.0) is None
    ev = threading.Event()
    t0 = time.monotonic()
    main_mod._make_sleep(0)(10.0, ev)
    assert time.monotonic() - t0 < 0.5
    t0 = time.monotonic()
    main_mod._make_sleep(100.0)(1.0, ev)  # 1 s of sim time at 100x -> ~10 ms
    assert time.monotonic() - t0 < 0.5


def test_sim_plant_follows_config_names(cfg):
    plant = main_mod.make_sim_plant(cfg)
    obs = plant.observe()
    assert set(obs.temps) == set(cfg.temps)
    assert set(obs.pwm) == set(cfg.channels)
    io = main_mod.PlantIO(plant, cfg)
    o = io.read()
    assert set(o.temps) == set(cfg.temps)
    ts0 = o.ts
    io.apply(MpcCommand(pwm=dict(cfg.fallback_pwm), mode=Mode.AUTO))
    assert io.read().ts == pytest.approx(ts0 + cfg.dt)


# --- recorder wiring (milestone record-and-fit) --------------------------------------


def test_build_recorder_is_none_without_record_path_or_flag(example_config_path):
    app = load_config(example_config_path)
    assert main_mod.build_recorder(app, app.mpc, None) is None


def test_build_recorder_cli_flag_takes_precedence_over_config_key(tmp_path, example_config_path):
    import yaml

    data = yaml.safe_load(example_config_path.read_text())
    data["record_path"] = str(tmp_path / "from_config.jsonl")
    conf = tmp_path / "rec.yaml"
    conf.write_text(yaml.safe_dump(data))
    app = load_config(conf)

    from_config = main_mod.build_recorder(app, app.mpc, None)
    assert from_config is not None and from_config.path == tmp_path / "from_config.jsonl"

    from_cli = main_mod.build_recorder(app, app.mpc, str(tmp_path / "from_cli.jsonl"))
    assert from_cli.path == tmp_path / "from_cli.jsonl"


def test_build_recorder_reads_rotation_settings(tmp_path, example_config_path):
    import yaml

    data = yaml.safe_load(example_config_path.read_text())
    data["record_path"] = str(tmp_path / "rec.jsonl")
    data["record_max_bytes"] = 123
    data["record_backup_count"] = 4
    conf = tmp_path / "rec.yaml"
    conf.write_text(yaml.safe_dump(data))
    app = load_config(conf)

    rec = main_mod.build_recorder(app, app.mpc, None)
    assert rec.max_bytes == 123 and rec.backup_count == 4


def test_build_recorder_rejects_non_integer_rotation_settings(tmp_path, example_config_path):
    import yaml

    data = yaml.safe_load(example_config_path.read_text())
    data["record_path"] = str(tmp_path / "rec.jsonl")
    data["record_max_bytes"] = "lots"
    conf = tmp_path / "rec.yaml"
    conf.write_text(yaml.safe_dump(data))
    app = load_config(conf)

    with pytest.raises(main_mod.ConfigError):
        main_mod.build_recorder(app, app.mpc, None)


def test_main_records_ticks_via_cli_flag(tmp_path, example_config_path, restore_signals):
    out = tmp_path / "rec.jsonl"
    rc = main_mod.main(
        [
            "--config",
            str(example_config_path),
            "--source",
            "sim",
            "--ticks",
            "3",
            "--sim-speed",
            "0",
            "--record",
            str(out),
        ]
    )
    assert rc == 0
    lines = out.read_text(encoding="utf-8").splitlines()
    assert len(lines) == 3
    assert [json.loads(x)["i"] for x in lines] == [0, 1, 2]


def test_main_records_ticks_via_config_key(tmp_path, example_config_path, restore_signals):
    import yaml

    out = tmp_path / "rec.jsonl"
    data = yaml.safe_load(example_config_path.read_text())
    data["record_path"] = str(out)
    conf = tmp_path / "rec.yaml"
    conf.write_text(yaml.safe_dump(data))
    rc = main_mod.main(
        ["--config", str(conf), "--source", "sim", "--ticks", "2", "--sim-speed", "0"]
    )
    assert rc == 0
    assert len(out.read_text(encoding="utf-8").splitlines()) == 2


def test_main_without_record_path_or_flag_writes_nothing(
    tmp_path, example_config_path, restore_signals
):
    rc = main_mod.main(
        [
            "--config",
            str(example_config_path),
            "--source",
            "sim",
            "--ticks",
            "2",
            "--sim-speed",
            "0",
        ]
    )
    assert rc == 0
    assert not any(tmp_path.iterdir())  # nothing written anywhere under our scratch dir


def test_build_health_monitor_wires_the_source_and_the_supervisor(example_config_path):
    """Items 79 and 83: the monitor is an on_tick observer that reads the source's own
    device health and publishes everything through Supervisor.set_device_health."""
    from aqua_bridge.control.supervisor import Supervisor

    app = load_config(example_config_path)
    sup = Supervisor(app.mpc)
    src, _sink, _release = main_mod.build_io(app, "composite")
    monitor = main_mod.build_health_monitor(app, sup, src)
    assert monitor is not None and monitor.source is src
    assert monitor.settings == FanHealthConfig()
    monitor.on_tick(None)  # never raises, and publishes even with no observation
    assert sup.snapshot().device_health["ok"] is True


def test_build_health_monitor_publishes_the_single_xt6_controller(example_config_path):
    """--source xt6 is the argparse default and what deploy/install-pi.sh installs: its
    bare adapter answers for itself, not with a {devices, problems} dict, and the whole
    controller must still reach /api/state (PROJECT.md section 8 items 83, 91)."""
    from aqua_bridge.control.supervisor import Supervisor

    app = load_config(example_config_path)
    sup = Supervisor(app.mpc)
    src, _sink, _release = main_mod.build_io(app, "xt6")
    monitor = main_mod.build_health_monitor(app, sup, src)
    assert monitor is not None
    monitor.on_tick(None)
    (device,) = sup.snapshot().device_health["devices"]
    assert device["label"] == src.binding.label and device["device"] == "aquaero"
    assert "flows" in device and "not_pwm_channels" in device and "stuck_channels" in device


def test_build_health_monitor_is_none_when_disabled_with_a_source_that_has_none(
    example_config_path, tmp_path
):
    from aqua_bridge.control.supervisor import Supervisor

    app = load_config(example_config_path)
    app = dataclasses.replace(app, fan_health={"enabled": False}, host_health={"enabled": False})
    sup = Supervisor(app.mpc)
    sim_src, _sink, _release = main_mod.build_io(app, "sim")
    assert main_mod.build_health_monitor(app, sup, sim_src) is None
    # ... but a composite still gets one: its device health is published either way
    composite, _s, _r = main_mod.build_io(app, "composite")
    assert main_mod.build_health_monitor(app, sup, composite) is not None


def test_a_bad_fan_health_key_exits_2_before_anything_opens(tmp_path, example_config_path, caplog):
    raw = yaml.safe_load(example_config_path.read_text())
    raw["fan_health"]["rpm_fault_s"] = -1
    path = tmp_path / "config.yaml"
    path.write_text(yaml.safe_dump(raw))
    with caplog.at_level(logging.ERROR):
        assert main_mod.main(["--config", str(path), "--source", "sim", "--once"]) == 2
    assert "fan_health.rpm_fault_s" in caplog.text


def test_a_host_health_air_temps_typo_exits_2_before_anything_opens(
    tmp_path, example_config_path, caplog
):
    """Item 97: the one host_health check that needs mpc.temps runs in main() too.

    Left to HealthMonitor.__init__ it would fire after build_io had opened the
    hardware and outside main's ConfigError handling: a raw traceback, exit 1 instead
    of the documented 2, the hidraw handles never released and the section 9 stop path
    -- the only place fallback_pwm is written at exit -- skipped.
    """
    raw = yaml.safe_load(example_config_path.read_text())
    raw["host_health"]["air_temps"] = ["nope_typo"]
    path = tmp_path / "config.yaml"
    path.write_text(yaml.safe_dump(raw))
    with caplog.at_level(logging.ERROR):
        assert main_mod.main(["--config", str(path), "--source", "sim", "--once"]) == 2
    assert "host_health.air_temps" in caplog.text and "nope_typo" in caplog.text


def test_the_tick_reader_polls_vcgencmd_on_the_configured_cadence(example_config_path, monkeypatch):
    """Item 97: the control tick reads the whole get_throttled word, on a cadence.

    The board this daemon runs on exposes no get_throttled sysfs attribute (a Zero 2 W
    on kernel 6.18), so a tick reader without vcgencmd would leave the "throttling now"
    rule unable to fire on the one thread that matters. It runs on the loop thread, so
    the process is rate limited by host_health.vcgencmd_interval_s and bounded by
    host_health.vcgencmd_timeout_s: one 3.3 ms fork a minute, not one per 5 s tick.
    """
    from aqua_bridge.control.supervisor import Supervisor

    spawned: list[tuple[list[str], float]] = []

    class _Proc:
        returncode = 0
        stdout = "throttled=0x50000\n"

    def fake_run(argv, **kwargs):
        spawned.append((list(argv), kwargs.get("timeout")))
        return _Proc()

    # A machine that does have vcgencmd -- no process is started either way: the run
    # itself is faked, so this test never shells out.
    monkeypatch.setattr(hostinfo_mod.shutil, "which", lambda name: f"/usr/bin/{name}")
    monkeypatch.setattr(hostinfo_mod.subprocess, "run", fake_run)

    raw = yaml.safe_load(example_config_path.read_text())
    raw["host_health"]["vcgencmd_interval_s"] = 60.0
    raw["host_health"]["vcgencmd_timeout_s"] = 0.25
    raw["host"] = {**raw.get("host", {}), "interval_s": 0.0}  # every tick refreshes
    path = example_config_path.parent / "_host_health_cadence.yaml"
    path.write_text(yaml.safe_dump(raw))
    try:
        app = load_config(path)
    finally:
        path.unlink()
    sup = Supervisor(app.mpc)
    src, _sink, _release = main_mod.build_io(app, "sim")
    monitor = main_mod.build_health_monitor(app, sup, src)
    assert monitor is not None

    for _ in range(12):  # a minute of dt = 5 s ticks
        monitor.on_tick(None)
    assert [(argv[1], timeout) for argv, timeout in spawned] == [("get_throttled", 0.25)], (
        "the tick must run vcgencmd once per vcgencmd_interval_s, not once per tick"
    )
    throttled = sup.snapshot().device_health["host"]["throttled"]
    assert throttled is not None, "the rule's source must not be null on the tick path"
    assert throttled["source"] == "vcgencmd" and throttled["under_voltage_since_boot"] is True


def test_the_publishers_reader_shares_the_documented_cadence(monkeypatch):
    """The HTTP and MQTT readers are built the same way, from the same keys, and each
    keeps its own cache -- they are on their own threads, not the loop's."""
    from aqua_bridge.health import HostHealthConfig, host_metrics_reader

    spawned: list[float] = []

    class _Proc:
        returncode = 0
        stdout = "throttled=0x0\n"

    def fake_run(argv, **kwargs):
        spawned.append(kwargs.get("timeout"))
        return _Proc()

    monkeypatch.setattr(hostinfo_mod.shutil, "which", lambda name: f"/usr/bin/{name}")
    monkeypatch.setattr(hostinfo_mod.subprocess, "run", fake_run)

    now = [0.0]
    reader = host_metrics_reader(
        HostHealthConfig(vcgencmd_timeout_s=0.25, vcgencmd_interval_s=60.0),
        clock=lambda: now[0],
    )
    for tick in range(0, 120, 5):
        now[0] = float(tick)
        assert reader()["throttled"]["source"] == "vcgencmd"
    assert spawned == [0.25, 0.25]


def test_build_io_sim_and_unknown(example_config_path):
    app = load_config(example_config_path)
    src, sink, release = main_mod.build_io(app, "sim")
    assert src is sink and release is None
    with pytest.raises(RuntimeError):
        main_mod.build_io(app, "bogus")


def test_build_io_xt6_constructs_adapter_without_touching_hardware(example_config_path):
    from aqua_bridge.hw.aquacomputer_adapter import AquacomputerAdapter

    app = load_config(example_config_path)
    src, sink, release = main_mod.build_io(app, "xt6")
    assert isinstance(src, AquacomputerAdapter) and src is sink and release is None
    assert not src.is_open  # the first read() finds the device
    assert src.binding.pwm_map == {"radiator": 1, "intake": 2}


def test_build_io_composite_constructs_composite_without_touching_hardware(example_config_path):
    """The example config's xt6: section alone (no aquacomputer:/onewire.sensors) is a
    valid single-device composite -- the DAS plan section 12 Q1 generalisation of xt6."""
    app = load_config(example_config_path)
    src, sink, release = main_mod.build_io(app, "composite")
    assert hasattr(src, "read") and hasattr(sink, "apply")
    assert [device.is_open for device in src.devices] == [False]
    assert release is None  # no onewire.sensors in config.example.yaml -> nothing to stop


def test_build_io_composite_forwards_smart_into_the_composite(example_config_path):
    """milestone smart-agent: build_io's smart= reaches CompositeSource.smart
    (and so PlantObservation.inputs["smart"]) for --source composite."""
    from aqua_bridge.publishers.inputs import SmartInbox

    app = load_config(example_config_path)
    smart = SmartInbox()
    src, _sink, _release = main_mod.build_io(app, "composite", smart=smart)
    assert src.smart is smart


def test_build_io_sim_ignores_smart(example_config_path):
    """sim/xt6 have no inputs concept; smart= is accepted but has no effect."""
    from aqua_bridge.publishers.inputs import SmartInbox

    app = load_config(example_config_path)
    src, sink, release = main_mod.build_io(app, "sim", smart=SmartInbox())
    assert src is sink and release is None


def test_source_hwmon_is_no_longer_a_choice(example_config_path, capsys):
    with pytest.raises(SystemExit) as exc:
        main_mod.build_parser().parse_args(
            ["--config", str(example_config_path), "--source", "hwmon"]
        )
    assert exc.value.code == 2
    assert "'composite'" in capsys.readouterr().err
    with pytest.raises(RuntimeError):
        main_mod.build_io(load_config(example_config_path), "hwmon")


def test_build_io_checks_the_hardware_worst_case_against_the_watchdog(example_config_path):
    """Review finding: a tick blocked by usbhid timeouts must not outlast WatchdogSec,
    or systemd kills the daemon without its fallback write."""
    from aqua_bridge.hw.aquacomputer_adapter import AquacomputerTiming

    app = load_config(example_config_path)
    worst = (
        app.mpc.dt
        + app.mpc.budget_alarm_ms / 1000.0
        + AquacomputerTiming.for_kind(app.xt6["device"]).worst_case_tick_s()
    )
    for source in ("xt6", "composite"):
        with pytest.raises(ConfigError, match="systemd watchdog"):
            main_mod.build_io(app, source, watchdog_s=worst)
        main_mod.build_io(app, source, watchdog_s=worst + 1.0)
    main_mod.build_io(app, "sim", watchdog_s=0.001)  # no hardware: nothing to check


def test_main_reads_the_watchdog_from_the_environment(
    tmp_path, example_config_path, restore_signals, monkeypatch, caplog
):
    monkeypatch.setenv("WATCHDOG_USEC", "1000000")  # 1 s: far below the worst case
    monkeypatch.delenv("WATCHDOG_PID", raising=False)
    with caplog.at_level("ERROR"):
        assert main_mod.main(["--config", str(example_config_path), "--source", "xt6"]) == 2
    assert "systemd watchdog" in caplog.text


def test_composite_source_missing_binding_exits_2(tmp_path, example_config_path, restore_signals):
    """The same F4-style guarantee as xt6, now enforced across the whole device fleet."""
    import yaml

    data = yaml.safe_load(example_config_path.read_text())
    data["xt6"]["fans"] = {"radiator": {"pwm": "pwm1", "rpm": "fan1"}}
    bad = tmp_path / "bad_map.yaml"
    bad.write_text(yaml.safe_dump(data))
    assert main_mod.main(["--config", str(bad), "--source", "composite"]) == 2


def test_hwmon_era_config_exits_2_with_the_rename_hint(
    tmp_path, example_config_path, restore_signals, caplog
):
    import yaml

    data = yaml.safe_load(example_config_path.read_text())
    data["xt6"]["hwmon_name"] = "aquaero"
    bad = tmp_path / "hwmon_name.yaml"
    bad.write_text(yaml.safe_dump(data))
    with caplog.at_level("ERROR"):
        assert main_mod.main(["--config", str(bad), "--source", "xt6"]) == 2
    assert "xt6.hwmon_name is no longer supported" in caplog.text

    data = yaml.safe_load(example_config_path.read_text())
    data["hwmon"] = [{"name": "quadro", "fans": {}}]
    bad2 = tmp_path / "hwmon_section.yaml"
    bad2.write_text(yaml.safe_dump(data))
    assert main_mod.main(["--config", str(bad2), "--source", "composite"]) == 2


def test_xt6_map_not_matching_mpc_channels_exits_2(tmp_path, example_config_path, restore_signals):
    """Review finding F4: a channel missing from xt6.fans (or a temp_map key not in
    mpc.temps) is a config error at startup, not a silently unwritten fan."""
    import yaml

    data = yaml.safe_load(example_config_path.read_text())
    data["xt6"]["fans"] = {"radiator": {"pwm": "pwm1", "rpm": "fan1"}}
    bad = tmp_path / "bad_map.yaml"
    bad.write_text(yaml.safe_dump(data))
    assert main_mod.main(["--config", str(bad), "--source", "xt6"]) == 2

    data = yaml.safe_load(example_config_path.read_text())
    data["xt6"]["temp_map"]["ambient"] = "temp3"
    bad2 = tmp_path / "bad_temps.yaml"
    bad2.write_text(yaml.safe_dump(data))
    assert main_mod.main(["--config", str(bad2), "--source", "xt6"]) == 2


# --- publishers wired into main -----------------------------------------------------------


class _FakeMqttClient:
    instances: list[_FakeMqttClient] = []

    def __init__(self, **kwargs):
        self.kwargs = kwargs
        self.connected = True
        self.calls: list[str] = []
        self.states: list[dict] = []
        self.discovery: list[object] = []
        self.extra_handlers: dict[str, object] = {}
        _FakeMqttClient.instances.append(self)

    def add_topic_handler(self, topic_filter, handler):
        self.extra_handlers[topic_filter] = handler

    def connect_async(self):
        self.calls.append("connect_async")

    def loop_start(self):
        self.calls.append("loop_start")

    def loop_stop(self):
        self.calls.append("loop_stop")

    def disconnect(self):
        self.calls.append("disconnect")

    def publish_discovery(self, *, control_mode):
        self.discovery.append(control_mode)

    def publish_state(self, snapshot_dict, host):
        self.states.append(dict(snapshot_dict))


def test_main_wires_mqtt_when_enabled(tmp_path, example_config_path, restore_signals, monkeypatch):
    import yaml

    import aqua_bridge.publishers.runtime as runtime

    _FakeMqttClient.instances.clear()
    monkeypatch.setattr(runtime, "MqttClient", _FakeMqttClient)
    data = yaml.safe_load(example_config_path.read_text())
    data["mqtt"]["enabled"] = True
    conf = tmp_path / "mqtt.yaml"
    conf.write_text(yaml.safe_dump(data))
    rc = main_mod.main(
        ["--config", str(conf), "--source", "sim", "--ticks", "3", "--sim-speed", "0"]
    )
    assert rc == 0
    (client,) = _FakeMqttClient.instances
    assert client.kwargs["host"] == data["mqtt"]["host"]
    assert client.calls == ["connect_async", "loop_start", "disconnect", "loop_stop"]
    assert len(client.states) == 3  # one retained state blob per tick
    assert client.states[-1]["cmd"]["pwm"].keys() == {"radiator", "intake"}
    assert client.discovery  # Discovery published once connected
    # milestone smart-agent: the SMART inbox is wired onto this same client.
    (topic_filter,) = client.extra_handlers
    assert topic_filter == f"{data['mqtt']['node_id']}/in/smart/+"


def test_main_wires_http_when_enabled(tmp_path, example_config_path, restore_signals, monkeypatch):
    import yaml

    import aqua_bridge.publishers.runtime as runtime

    started: dict[str, object] = {}

    class _Runner:
        addresses = [("127.0.0.1", 8080)]

        async def cleanup(self):
            started["cleaned"] = True

    async def fake_run(surface, app_cfg, **kwargs):
        started["surface"] = surface
        started["port"] = app_cfg.section("http").get("port")
        started["kwargs"] = kwargs
        return _Runner()

    monkeypatch.setattr(runtime, "run_http", fake_run)
    data = yaml.safe_load(example_config_path.read_text())
    data["http"]["enabled"] = True
    conf = tmp_path / "http.yaml"
    conf.write_text(yaml.safe_dump(data))
    rc = main_mod.main(
        ["--config", str(conf), "--source", "sim", "--ticks", "2", "--sim-speed", "0"]
    )
    assert rc == 0
    assert started["port"] == data["http"]["port"]
    assert hasattr(started["surface"], "submit") and hasattr(started["surface"], "snapshot")
    assert started.get("cleaned") is True
    # milestone smart-agent: main() always builds a SmartInbox and hands it to HttpService.
    from aqua_bridge.publishers.inputs import SmartInbox

    assert isinstance(started["kwargs"]["smart_inbox"], SmartInbox)


def test_main_keeps_controlling_when_a_publisher_fails_to_start(
    tmp_path, example_config_path, restore_signals, monkeypatch, recorded_applies
):
    import yaml

    import aqua_bridge.publishers.runtime as runtime

    async def boom(surface, app_cfg):
        raise OSError("address in use")

    def no_client(**kwargs):
        raise ConnectionError("no broker")

    monkeypatch.setattr(runtime, "run_http", boom)
    monkeypatch.setattr(runtime, "MqttClient", no_client)
    data = yaml.safe_load(example_config_path.read_text())
    data["http"]["enabled"] = True
    data["mqtt"]["enabled"] = True
    conf = tmp_path / "both.yaml"
    conf.write_text(yaml.safe_dump(data))
    rc = main_mod.main(
        ["--config", str(conf), "--source", "sim", "--ticks", "2", "--sim-speed", "0"]
    )
    assert rc == 0
    assert len(recorded_applies) == 3  # 2 ticks + shutdown: the loop ran regardless


def test_main_keeps_controlling_when_https_has_no_certificate_or_credentials(
    tmp_path, example_config_path, restore_signals, recorded_applies, caplog
):
    """http.enabled with a missing certificate, key and credentials file (the real
    run_http, nothing monkeypatched): the API stays off with a clear error and the
    loop runs regardless (section 6)."""
    import logging

    import yaml

    data = yaml.safe_load(example_config_path.read_text())
    data["http"].update(
        enabled=True,
        bind="127.0.0.1",
        port=0,
        tls_cert=str(tmp_path / "missing-cert.pem"),
        tls_key=str(tmp_path / "missing-key.pem"),
        credentials_file=str(tmp_path / "missing-users"),
    )
    conf = tmp_path / "https.yaml"
    conf.write_text(yaml.safe_dump(data))
    with caplog.at_level(logging.ERROR):
        rc = main_mod.main(
            ["--config", str(conf), "--source", "sim", "--ticks", "2", "--sim-speed", "0"]
        )
    assert rc == 0
    assert len(recorded_applies) == 3  # 2 ticks + shutdown
    assert any(
        "HTTPS API not started" in r.getMessage() and "missing-cert.pem" in r.getMessage()
        for r in caplog.records
    )


def test_main_http_enabled_bad_type_is_logged_and_api_stays_off(
    tmp_path, example_config_path, restore_signals, recorded_applies, caplog
):
    """Item 57: http.enabled: "true" (a string, not a bool) used to compare unequal to
    ``True`` and leave the API off with no log line at all; it must now be reported by
    name, same as any other bad ``http:`` value, while the loop keeps running."""
    import yaml

    data = yaml.safe_load(example_config_path.read_text())
    data["http"]["enabled"] = "true"
    conf = tmp_path / "http-bad-enabled.yaml"
    conf.write_text(yaml.safe_dump(data))
    with caplog.at_level(logging.ERROR):
        rc = main_mod.main(
            ["--config", str(conf), "--source", "sim", "--ticks", "2", "--sim-speed", "0"]
        )
    assert rc == 0
    assert len(recorded_applies) == 3  # 2 ticks + shutdown: the loop ran regardless
    assert any("http.enabled must be true or false" in r.getMessage() for r in caplog.records)


def test_main_mqtt_enabled_bad_type_is_logged_and_mqtt_stays_off(
    tmp_path, example_config_path, restore_signals, recorded_applies, caplog, monkeypatch
):
    """Item 57: same silent-off bug as http.enabled, for mqtt.enabled -- and the
    daemon must never even attempt to reach the configured broker over it."""
    import yaml

    import aqua_bridge.publishers.runtime as runtime

    _FakeMqttClient.instances.clear()
    monkeypatch.setattr(runtime, "MqttClient", _FakeMqttClient)
    data = yaml.safe_load(example_config_path.read_text())
    data["mqtt"]["enabled"] = "true"
    conf = tmp_path / "mqtt-bad-enabled.yaml"
    conf.write_text(yaml.safe_dump(data))
    with caplog.at_level(logging.ERROR):
        rc = main_mod.main(
            ["--config", str(conf), "--source", "sim", "--ticks", "2", "--sim-speed", "0"]
        )
    assert rc == 0
    assert len(recorded_applies) == 3
    assert not _FakeMqttClient.instances  # never even tried to connect
    assert any("mqtt.enabled must be true or false" in r.getMessage() for r in caplog.records)


# --- SIGTERM stop path (section 9) ------------------------------------------------------


def test_sigterm_in_process_runs_shutdown(
    example_config_path, restore_signals, recorded_applies, monkeypatch
):
    cfg = load_config(example_config_path).mpc
    # Fire SIGTERM shortly after the first sink write, not on a wall-clock timer
    # started before main(): main() installs its handler before the first tick,
    # so by then the signal can only reach the section-9 stop path. A timer armed
    # before main() raced handler installation and, on a slow single core, could
    # deliver the raw default SIGTERM and kill the whole pytest process.
    armed = threading.Event()
    original_apply = main_mod.PlantIO.apply

    def apply_then_kill(self, cmd):
        original_apply(self, cmd)
        if not armed.is_set():
            armed.set()
            # Real-time sim (dt = 2 s): the loop is inside its sleep when SIGTERM arrives.
            threading.Timer(0.3, lambda: os.kill(os.getpid(), signal.SIGTERM)).start()

    monkeypatch.setattr(main_mod.PlantIO, "apply", apply_then_kill)
    t0 = time.monotonic()
    rc = main_mod.main(["--config", str(example_config_path), "--source", "sim"])
    assert rc == 0
    assert time.monotonic() - t0 < cfg.dt + 2.0
    assert recorded_applies[-1].pwm == dict(cfg.fallback_pwm)
    assert recorded_applies[-1].diagnostics["policy"] == "shutdown"
    assert len(recorded_applies) >= 2


class _ReentrantStderr:
    """Stand-in for the daemon's stderr ``BufferedWriter`` (the logging stream *and*
    ``sys.stderr``, as in a real process).

    Like ``io.BufferedWriter`` it raises ``RuntimeError("reentrant call ...")`` when
    ``write`` is entered while another ``write`` is still on the stack. ``fire``
    is called from inside the write of the first line containing ``trigger`` --
    that is where the Pi's SIGTERM landed: the startup banner logged right after
    the handlers were installed.
    """

    def __init__(self, trigger: str, fire) -> None:
        self.trigger = trigger
        self.fire = fire
        self.text: list[str] = []
        self.depth = 0
        self.fired = False
        self.reentrant_writes = 0

    def write(self, s: str) -> int:
        if self.depth:
            self.reentrant_writes += 1
            raise RuntimeError("reentrant call inside <_io.BufferedWriter name='<stderr>'>")
        self.depth += 1
        try:
            self.text.append(s)
            if not self.fired and self.trigger in s:
                self.fired = True
                self.fire()
        finally:
            self.depth -= 1
        return len(s)

    def flush(self) -> None:
        pass

    def __str__(self) -> str:
        return "".join(self.text)


@pytest.fixture
def installed_handlers(monkeypatch):
    """Let ``main()`` install its real handlers, and hand them to the test by signal."""
    handlers: dict[int, object] = {}
    real_signal = signal.signal

    def record(signum, handler):
        handlers[signum] = handler
        return real_signal(signum, handler)

    monkeypatch.setattr(signal, "signal", record)
    return handlers


def test_sigterm_inside_a_stderr_write_still_runs_the_stop_path(
    example_config_path, restore_signals, recorded_applies, installed_handlers, monkeypatch, caplog
):
    """Regression for the Pi hang (3/3): SIGTERM delivered while the main thread is
    inside the startup banner's stderr write.

    The old handler logged before ``stop.set()``; that log call re-entered the
    stderr ``BufferedWriter`` and raised, logging's ``handleError`` re-entered it
    once more trying to report that, the exception unwound out of the handler
    before ``stop.set()`` and was swallowed by the outer ``emit`` -- so ``main()``
    went on into ``loop.run(stop)`` with the event never set and the daemon
    ignored SIGTERM until systemd's SIGKILL. Deterministic: the handler is invoked
    from inside the banner write itself, ``--ticks`` bounds the run either way.
    """
    cfg = load_config(example_config_path).mpc
    stream = _ReentrantStderr(
        "source=sim", lambda: installed_handlers[signal.SIGTERM](signal.SIGTERM, None)
    )
    handler = logging.StreamHandler(stream)
    handler.setFormatter(logging.Formatter("%(levelname)s %(name)s: %(message)s"))
    logger = logging.getLogger("aqua_bridge")
    logger.addHandler(handler)
    # basicConfig() in main() is a no-op under pytest (the root logger already has
    # handlers), so the level is set here; caplog restores it.
    caplog.set_level(logging.INFO, logger="aqua_bridge")
    monkeypatch.setattr(sys, "stderr", stream)
    try:
        rc = main_mod.main(
            [
                "--config",
                str(example_config_path),
                "--source",
                "sim",
                "--ticks",
                "3",
                "--sim-speed",
                "0",
            ]
        )
    finally:
        logger.removeHandler(handler)
    err = str(stream)
    assert stream.fired, "the banner was never logged: " + err
    assert rc == 0, err
    assert stream.reentrant_writes == 0, "the signal handler wrote to stderr:\n" + err
    # the stop event was set before any tick could run: only the shutdown write
    assert [c.diagnostics.get("policy") for c in recorded_applies] == ["shutdown"], err
    assert recorded_applies[-1].pwm == dict(cfg.fallback_pwm)
    assert recorded_applies[-1].mode is Mode.FALLBACK
    assert "signal SIGTERM: stopping" in err
    assert "shutdown: wrote fallback_pwm" in err
    assert err.index("signal SIGTERM: stopping") < err.index("shutdown: wrote fallback_pwm")
    assert "exit 0 after 0 ticks" in err


def test_second_sigterm_during_shutdown_is_harmless(
    example_config_path, restore_signals, recorded_applies, installed_handlers, monkeypatch
):
    """SIGTERM inside the first tick's sink write, then another one inside the
    shutdown write, both while any log call from the handler would raise the
    reentrancy error: one stop path, one fallback_pwm write, exit 0."""
    cfg = load_config(example_config_path).mpc
    raising = threading.Event()  # while set, any use of the module logger raises
    messages: list[str] = []
    real_log = main_mod._LOG

    class _Log:
        def __getattr__(self, name):
            attr = getattr(real_log, name)
            if not callable(attr):
                return attr

            def call(msg, *args, **kwargs):
                if raising.is_set():
                    raise RuntimeError("reentrant call inside <_io.BufferedWriter name='<stderr>'>")
                messages.append(msg % args if args else str(msg))
                return attr(msg, *args, **kwargs)

            return call

    monkeypatch.setattr(main_mod, "_LOG", _Log())
    original_apply = main_mod.PlantIO.apply

    def apply_and_signal(self, cmd):
        original_apply(self, cmd)
        policy = cmd.diagnostics.get("policy")
        if policy != "shutdown" and len(self.applied) > 1:
            return
        raising.set()
        try:
            installed_handlers[signal.SIGTERM](signal.SIGTERM, None)
        finally:
            raising.clear()

    monkeypatch.setattr(main_mod.PlantIO, "apply", apply_and_signal)
    rc = main_mod.main(
        [
            "--config",
            str(example_config_path),
            "--source",
            "sim",
            "--ticks",
            "5",
            "--sim-speed",
            "0",
        ]
    )
    assert rc == 0, messages
    assert [c.diagnostics.get("policy") for c in recorded_applies] == ["solver", "shutdown"], (
        messages
    )
    assert recorded_applies[-1].pwm == dict(cfg.fallback_pwm)
    assert messages.count("signal SIGTERM: stopping") == 1, messages
    assert "1 further signal(s) during shutdown ignored" in messages
    assert "exit 0 after 1 ticks" in messages


STARTUP_TIMEOUT_S = 120.0  # bounds a daemon that never logs its banner; not tuned to any machine
STOP_TIMEOUT_S = 30.0  # the stop path is one sink write; dt=2 s bounds the loop's sleep


@pytest.mark.slow
def test_sigterm_subprocess_exits_0_and_logs_fallback_write(example_config_path):
    env = dict(os.environ)
    env["PYTHONPATH"] = str(REPO / "src") + os.pathsep + env.get("PYTHONPATH", "")
    env.pop("NOTIFY_SOCKET", None)
    proc = subprocess.Popen(
        [
            sys.executable,
            "-m",
            "aqua_bridge",
            "--config",
            str(example_config_path),
            "--source",
            "sim",
        ],
        env=env,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.PIPE,
        text=True,
    )
    # main() logs its startup banner only after the SIGTERM/SIGINT handlers are
    # installed, so the banner on stderr is the marker that SIGTERM will reach the
    # section-9 stop path. A fixed pre-signal sleep raced daemon startup and lost
    # on the Pi's single armv6 core (raw SIGTERM, returncode -15). The timeouts
    # only bound a hung or dead daemon: the test polls the process so an early
    # death fails at once, and a daemon that ignores SIGTERM is killed and
    # reported with its stderr instead of stalling the suite.
    lines: list[str] = []
    started = threading.Event()

    def pump_stderr() -> None:
        assert proc.stderr is not None
        for line in proc.stderr:
            lines.append(line)
            if "aqua-bridge " in line and "source=sim" in line:
                started.set()

    def fail(what: str) -> None:
        pytest.fail(f"{what} (returncode={proc.returncode}); daemon stderr:\n" + "".join(lines))

    reader = threading.Thread(target=pump_stderr, daemon=True)
    reader.start()
    try:
        deadline = time.monotonic() + STARTUP_TIMEOUT_S
        while not started.wait(0.1):
            if proc.poll() is not None:
                fail("daemon exited before logging its startup banner")
            if time.monotonic() > deadline:
                fail(f"daemon did not log its startup banner within {STARTUP_TIMEOUT_S:.0f} s")
        proc.send_signal(signal.SIGTERM)
        try:
            proc.wait(timeout=STOP_TIMEOUT_S)
        except subprocess.TimeoutExpired:
            proc.kill()
            proc.wait()
            fail(f"daemon ignored SIGTERM for {STOP_TIMEOUT_S:.0f} s (section 9 stop path hung)")
    finally:
        if proc.poll() is None:
            proc.kill()
        proc.wait()
        reader.join(timeout=10.0)
    err = "".join(lines)
    assert proc.returncode == 0, err
    assert "signal SIGTERM: stopping" in err, err
    assert "shutdown: wrote fallback_pwm" in err, err
    assert "exit 0" in err, err
