"""Tests for aqua_bridge.hw.aquacomputer_adapter against fake controllers, plus one live test.

PROJECT.md section 3 (Track B) / section 4.7 / section 2 ("USB spike results",
"hidraw check") / section 8 items 85 (aquabus outputs) and 86 (no save report on
a write).
"""

from __future__ import annotations

import errno
import math
import shutil
import subprocess

import pytest

from aqua_bridge.hw.aquacomputer import (
    AQUAERO,
    DUTY_MAX,
    QUADRO,
    SOURCE_UNCONFIGURED,
    DeviceKind,
    capture_channel,
    channel_holds,
    channel_state,
    control_duty,
    finalize_control_report,
    patch_duties,
    software_sensor_report,
)
from aqua_bridge.hw.aquacomputer_adapter import (
    AquacomputerAdapter,
    AquacomputerTiming,
    DeviceBinding,
    DeviceUnavailable,
)
from aqua_bridge.hw.hidraw import HIDRAW_QUEUE_FULL, FeatureReportError
from aqua_bridge.model import Mode, MpcCommand
from aquacomputer_fakes import (
    FakeBus,
    FakeClock,
    FakeController,
    FakeSleep,
    aquabus_aquaero,
    aquabus_aquaero_all_configured,
    fixture_bytes,
)

LOGGER = "aqua_bridge.hw.aquacomputer"
AQUAERO_T = AquacomputerTiming.for_kind(AQUAERO)
QUADRO_T = AquacomputerTiming.for_kind(QUADRO)
AQUAERO_GAP_S = AQUAERO_T.ctrl_gap_ms / 1000.0
#: Write limiting for the tests that exercise it (the defaults write every change).
LIMITS = {"write_min_interval_s": 30.0, "write_deadband": 50}


def _cmd(**pwm: float) -> MpcCommand:
    return MpcCommand(pwm=pwm, mode=Mode.AUTO)


def _timing(kind: DeviceKind, **overrides) -> AquacomputerTiming:
    return AquacomputerTiming.for_kind(kind, **overrides)


def _aquaero_binding(**timing) -> DeviceBinding:
    return DeviceBinding(
        kind=AQUAERO,
        pwm_map={"xt1": 1, "xt2": 2},
        fan_map={"xt2": 2},
        temp_map={"inlet": "temp6", "open": "temp1"},
        timing=_timing(AQUAERO, **timing),
    )


def _quadro_binding(**timing) -> DeviceBinding:
    return DeviceBinding(
        kind=QUADRO,
        pwm_map={"qd1": 1, "qd3": 3},
        fan_map={"qd3": 3},
        temp_map={"air": "temp2"},
        timing=_timing(QUADRO, **timing),
    )


def _setup(binding: DeviceBinding, **controller):
    clock = FakeClock()
    sleep = FakeSleep(clock)
    device = FakeController(binding.kind, clock, **controller)
    bus = FakeBus(device)
    adapter = AquacomputerAdapter(binding, clock=clock, sleep=sleep, opener=bus)
    return adapter, device, bus, clock, sleep


def _messages(caplog, level: str) -> list[str]:
    return [r.getMessage() for r in caplog.records if r.name == LOGGER and r.levelname == level]


# --- read ---------------------------------------------------------------------------------


def test_read_maps_the_aquaero_status_report() -> None:
    adapter, _device, _bus, clock, _ = _setup(_aquaero_binding())
    obs = adapter.read()
    assert obs.temps == {"inlet": pytest.approx(22.26), "open": None}
    assert obs.rpm == {"xt2": 120.0}
    # Output duty from the status report (firmware controllers: 100 %, preset: 14.12 %).
    assert obs.pwm == {"xt1": 1.0, "xt2": pytest.approx(0.1412)}
    assert obs.ts == clock()
    assert adapter.last_status is not None and adapter.last_status.kind == "aquaero"


def test_read_maps_the_quadro_status_report_and_reports_the_output_not_the_command() -> None:
    adapter, device, _bus, _clock, _ = _setup(_quadro_binding())
    device.duty_override = {2: 902}
    obs = adapter.read()
    assert obs.temps == {"air": pytest.approx(21.89)}
    assert obs.rpm == {"qd3": 119.0}
    assert obs.pwm == {"qd1": 1.0, "qd3": pytest.approx(0.0902)}


def test_nothing_is_opened_at_construction() -> None:
    adapter, device, bus, _clock, _ = _setup(_aquaero_binding())
    assert bus.opened == [] and not adapter.is_open and device.ops == []


def test_read_uses_the_newest_status_report_and_skips_other_reports() -> None:
    adapter, device, _bus, clock, _ = _setup(_aquaero_binding())
    adapter.read()
    device.pending.append(bytes([0x02]) + bytes(20))  # another input report
    device.duty_override = {0: 5000}
    device.emit()
    device.duty_override = {0: 2500}
    device.emit()
    device.pending.append(device.status()[:-1])  # truncated: not a status report
    clock.advance(1.0)
    assert adapter.read().pwm["xt1"] == pytest.approx(0.25)


def test_stale_status_report_raises_device_unavailable_and_recovers() -> None:
    adapter, device, bus, clock, _ = _setup(_aquaero_binding())
    max_age = AQUAERO_T.status_max_age_s
    adapter.read()
    clock.advance(max_age * 0.9)
    adapter.read()  # still young enough without a new report
    clock.advance(max_age * 0.2)
    with pytest.raises(DeviceUnavailable, match="no status report"):
        adapter.read()
    device.emit()
    assert adapter.read().pwm["xt1"] == 1.0
    assert bus.opened == [device.node]  # the node stayed open throughout


def test_open_waits_for_the_first_status_report() -> None:
    adapter, device, _bus, clock, _ = _setup(_aquaero_binding(), report_delay_s=0.8)
    start = clock()
    adapter.read()
    assert clock() - start == pytest.approx(0.8)
    assert not device.closed


def test_a_silent_device_blocks_only_the_first_read_and_still_takes_writes() -> None:
    """Review finding: a device whose control path works but sends no status report
    must still get the fallback write, and must not block every tick."""
    max_age = AQUAERO_T.status_max_age_s
    adapter, device, _bus, clock, _ = _setup(_aquaero_binding(), report_delay_s=max_age * 100)
    start = clock()
    with pytest.raises(DeviceUnavailable, match="no status report within"):
        adapter.read()
    assert clock() - start == pytest.approx(max_age)
    assert adapter.is_open and not device.closed
    before = clock()
    with pytest.raises(DeviceUnavailable, match="no status report within"):
        adapter.read()
    assert clock() == before  # no second wait
    adapter.apply(_cmd(xt1=0.8, xt2=0.8))
    assert len(device.sets()) == 1


def test_apply_opens_without_waiting_for_a_status_report() -> None:
    adapter, device, bus, clock, _ = _setup(_quadro_binding(), report_delay_s=1e9)
    start = clock()
    adapter.apply(_cmd(qd1=0.8, qd3=0.8))
    assert clock() == start and bus.opened == [device.node]
    assert [op.what for op in device.ops] == ["get", "set"]


def test_absent_device_raises_device_unavailable() -> None:
    clock = FakeClock()
    adapter = AquacomputerAdapter(_aquaero_binding(), clock=clock, opener=FakeBus())
    with pytest.raises(DeviceUnavailable):
        adapter.read()
    with pytest.raises(DeviceUnavailable):
        adapter.apply(_cmd(xt1=0.5, xt2=0.5))


def test_configured_serial_must_match_the_status_report() -> None:
    binding = DeviceBinding(kind=QUADRO, pwm_map={"qd1": 1}, serial="12345-54321")
    adapter, device, bus, _clock, _ = _setup(binding, status_serial="00000-00001")
    with pytest.raises(DeviceUnavailable, match="00000-00001.*12345-54321"):
        adapter.read()
    assert device.closed and not adapter.is_open
    device.status_serial = None  # the status report now carries the HID serial
    device.pending.clear()
    assert adapter.read().pwm == {"qd1": 1.0}
    assert bus.opened == [device.node, device.node]


def test_a_rejected_serial_blocks_writes_until_the_right_device_reports() -> None:
    """Second review: read() closed the node, but the same tick's apply() reopened it
    and wrote without any serial check."""
    binding = DeviceBinding(kind=QUADRO, pwm_map={"qd1": 1}, serial="12345-54321")
    adapter, device, bus, clock, _ = _setup(binding, status_serial="11111-22222")
    for _ in range(3):
        with pytest.raises(DeviceUnavailable, match="11111-22222"):
            adapter.read()
        with pytest.raises(DeviceUnavailable, match="nothing is written"):
            adapter.apply(_cmd(qd1=0.8))
        clock.advance(1.0)
    assert device.ops == []
    device.status_serial = None
    device.pending.clear()
    adapter.read()
    adapter.apply(_cmd(qd1=0.8))
    assert len(device.sets()) == 1


def test_a_silent_device_with_a_configured_serial_still_takes_writes() -> None:
    binding = DeviceBinding(kind=QUADRO, pwm_map={"qd1": 1}, serial="12345-54321")
    adapter, device, _bus, _clock, _ = _setup(binding, report_delay_s=1e9)
    with pytest.raises(DeviceUnavailable, match="no status report within"):
        adapter.read()
    adapter.apply(_cmd(qd1=0.8))
    assert len(device.sets()) == 1


def test_replug_with_a_new_node_reopens_and_gets_the_control_report_again() -> None:
    adapter, device, bus, clock, _ = _setup(_aquaero_binding())
    adapter.apply(_cmd(xt1=0.5, xt2=0.5))
    assert len(device.gets()) == 1 and len(device.sets()) == 1

    device.gone = True
    with pytest.raises(DeviceUnavailable):
        adapter.read()
    assert device.closed and not adapter.is_open

    replugged = FakeController(AQUAERO, clock, node="/dev/hidraw7", ctrl=bytearray(device.ctrl))
    bus.controllers = [replugged]
    clock.advance(5.0)
    adapter.read()
    assert bus.opened == ["/dev/hidraw3", "/dev/hidraw7"]
    adapter.apply(_cmd(xt1=0.5, xt2=0.5))
    assert len(replugged.gets()) == 1  # cache invalidated by the reopen
    assert replugged.sets() == []  # the device still holds the duties


def test_replug_that_reset_the_configuration_is_written_again() -> None:
    adapter, device, bus, clock, _ = _setup(_aquaero_binding())
    adapter.apply(_cmd(xt1=0.5, xt2=0.5))
    device.gone = True
    with pytest.raises(DeviceUnavailable):
        adapter.apply(_cmd(xt1=0.6, xt2=0.6))
    replugged = FakeController(AQUAERO, clock, node="/dev/hidraw9")  # firmware settings again
    bus.controllers = [replugged]
    clock.advance(5.0)
    adapter.apply(_cmd(xt1=0.6, xt2=0.6))
    assert len(replugged.sets()) == 1
    assert channel_holds(AQUAERO, replugged.ctrl, 0, 6000)
    assert channel_holds(AQUAERO, replugged.ctrl, 1, 6000)


# --- a full kernel queue -------------------------------------------------------------------


def test_a_full_queue_is_stale_and_the_queue_is_read_again() -> None:
    """Review finding: the kernel queue holds 63 reports and drops new ones while full,
    so a full drain may be minutes old; it must not refresh the status time."""
    adapter, device, _bus, clock, _ = _setup(_aquaero_binding())
    adapter.read()
    clock.advance(AQUAERO_T.status_max_age_s * 10)
    device.duty_override = {0: 1234}
    device.emit(HIDRAW_QUEUE_FULL)
    with pytest.raises(DeviceUnavailable, match="no status report"):
        adapter.read()
    assert device.pending == []

    # Reports that arrive while the full queue is being drained are recent.
    device.emit(HIDRAW_QUEUE_FULL)
    original = device.read_reports

    def drain_then_arrive() -> list[bytes]:
        out = original()
        if len(out) >= HIDRAW_QUEUE_FULL:
            device.duty_override = {0: 4321}
            device.emit()
        return out

    device.read_reports = drain_then_arrive  # type: ignore[method-assign]
    assert adapter.read().pwm["xt1"] == pytest.approx(0.4321)


def test_full_queue_reports_are_no_duty_evidence() -> None:
    adapter, device, clock = _commanded_quadro(status_max_age_s=1000.0)
    device.duty_override = {0: 9000}
    for _ in range(4):
        clock.advance(QUADRO_T.duty_mismatch_s)
        device.emit(HIDRAW_QUEUE_FULL)
        adapter.read()
    assert adapter.control_report is not None  # no mismatch was timed on them


# --- apply --------------------------------------------------------------------------------


def test_changed_command_is_one_set_with_the_driver_bytes_and_no_save_report() -> None:
    """Item 86: a SET takes effect without the save report, which would store every
    write in the controller's memory."""
    binding = DeviceBinding(kind=AQUAERO, pwm_map={"xt2": 2})
    adapter, device, _bus, _clock, _ = _setup(binding)
    adapter.apply(_cmd(xt2=0.1412))
    assert [op.what for op in device.ops] == ["get", "set"]
    assert device.sets()[0].data == fixture_bytes("aquaero-ctrl-after-writes.bin")
    assert device.saves() == []


def test_quadro_write_matches_the_driver_bytes_including_the_checksum() -> None:
    binding = DeviceBinding(kind=QUADRO, pwm_map={"qd3": 3})
    adapter, device, _bus, _clock, _ = _setup(binding)
    adapter.apply(_cmd(qd3=0.0902))
    assert [op.what for op in device.ops] == ["get", "set"]
    assert device.sets()[0].data == fixture_bytes("quadro-ctrl-after-writes.bin")
    assert device.saves() == []


def test_several_changed_channels_go_out_in_one_set() -> None:
    binding = DeviceBinding(kind=QUADRO, pwm_map={f"qd{n}": n for n in range(1, 5)})
    adapter, device, _bus, _clock, _ = _setup(binding)
    adapter.apply(_cmd(qd1=0.3, qd2=0.4, qd3=1.0, qd4=0.25))
    assert len(device.sets()) == 1 and device.saves() == []
    assert [control_duty(QUADRO, device.ctrl, k) for k in range(4)] == [3000, 4000, 10000, 2500]


def test_unchanged_command_sends_nothing_and_needs_no_control_read() -> None:
    binding = DeviceBinding(kind=QUADRO, pwm_map={f"qd{n}": n for n in range(1, 5)})
    adapter, device, _bus, clock, _ = _setup(binding)
    full = _cmd(qd1=1.0, qd2=1.0, qd3=1.0, qd4=1.0)  # what the firmware report holds
    adapter.apply(full)
    assert [op.what for op in device.ops] == ["get"]
    for _ in range(5):
        clock.advance(5.0)
        device.emit()
        adapter.read()
        adapter.apply(full)
    assert [op.what for op in device.ops] == ["get"]


def test_aquaero_channel_on_a_firmware_controller_is_written_even_at_the_same_duty() -> None:
    """Preset 100 % is not enough: the source must point at the preset and the
    limits must be 0 / 100 %."""
    binding = DeviceBinding(kind=AQUAERO, pwm_map={"xt1": 1})
    adapter, device, _bus, _clock, _ = _setup(binding)
    assert control_duty(AQUAERO, device.ctrl, 0) == DUTY_MAX
    assert not channel_state(AQUAERO, device.ctrl, 0).on_duty
    adapter.apply(_cmd(xt1=1.0))
    assert len(device.sets()) == 1
    assert channel_holds(AQUAERO, device.ctrl, 0, DUTY_MAX)


def test_only_changed_channels_are_patched() -> None:
    binding = DeviceBinding(kind=QUADRO, pwm_map={"qd1": 1, "qd2": 2})
    adapter, device, _bus, clock, _ = _setup(binding)
    adapter.apply(_cmd(qd1=0.5, qd2=0.5))
    external = bytearray(device.ctrl)
    patch_duties(QUADRO, external, {3: 1234})  # a channel this adapter does not own
    finalize_control_report(QUADRO, external)
    clock.advance(1.0)
    adapter.apply(_cmd(qd1=0.5, qd2=0.7))
    # Written from the cache: the unowned channel keeps the value of the cache.
    assert control_duty(QUADRO, device.ctrl, 0) == 5000
    assert control_duty(QUADRO, device.ctrl, 1) == 7000
    assert control_duty(QUADRO, device.ctrl, 3) == DUTY_MAX


def test_extra_channels_in_the_command_are_ignored() -> None:
    binding = DeviceBinding(kind=QUADRO, pwm_map={"qd1": 1})
    adapter, device, _bus, _clock, _ = _setup(binding)
    adapter.apply(_cmd(qd1=0.5, xt1=0.1))
    assert control_duty(QUADRO, device.ctrl, 0) == 5000


def test_aquaero_outputs_not_in_pwm_mode_are_reported_once_per_open(caplog) -> None:
    """Hardware 2026-09-15: the mode word at block +0x0E (0x0502 PWM, 0x0501 DC voltage).
    The firmware fixture has outputs 3 and 4 in DC mode. Reported, never written."""
    binding = DeviceBinding(kind=AQUAERO, pwm_map={f"xt{n}": n for n in range(1, 5)})
    adapter, device, bus, clock, _ = _setup(binding)
    with caplog.at_level("WARNING", logger=LOGGER):
        adapter.apply(_cmd(xt1=0.5, xt2=0.5, xt3=0.5, xt4=0.5))
        clock.advance(1.0)
        adapter.apply(_cmd(xt1=0.6, xt2=0.6, xt3=0.6, xt4=0.6))
    warnings = [m for m in _messages(caplog, "WARNING") if "mode" in m]
    assert len(warnings) == 2
    assert "pwm3 (xt3) is in DC voltage mode (mode word 0x0501)" in warnings[0]
    assert "pwm4 (xt4)" in warnings[1]
    assert [device.ctrl[b + 0x0E : b + 0x10].hex() for b in (0x20C, 0x220, 0x234, 0x248)] == [
        "0502",
        "0502",
        "0501",
        "0501",
    ]  # the mode is never written
    caplog.clear()
    adapter.close()
    clock.advance(1.0)
    with caplog.at_level("WARNING", logger=LOGGER):
        adapter.apply(_cmd(xt1=0.7, xt2=0.7, xt3=0.7, xt4=0.7))
    assert len([m for m in _messages(caplog, "WARNING") if "mode" in m]) == 2  # a new open


def test_quadro_outputs_get_no_mode_warning(caplog) -> None:
    binding = DeviceBinding(kind=QUADRO, pwm_map={"qd1": 1})
    adapter, _device, _bus, _clock, _ = _setup(binding)
    with caplog.at_level("WARNING", logger=LOGGER):
        adapter.apply(_cmd(qd1=0.5))
    assert not [m for m in _messages(caplog, "WARNING") if "mode" in m]


# --- the gap between control operations ------------------------------------------------------


def test_aquaero_waits_the_gap_before_every_get_and_set() -> None:
    """Owner decision 2026-09-15: 100 ms on the aquaero, timed from every control
    operation."""
    binding = DeviceBinding(kind=AQUAERO, pwm_map={"xt1": 1})
    adapter, device, _bus, clock, sleep = _setup(binding)
    adapter.apply(_cmd(xt1=0.5))
    get, set_ = device.ops
    assert get.t == 100.0  # nothing before it: no wait
    assert set_.t - get.t == pytest.approx(AQUAERO_GAP_S)
    adapter.apply(_cmd(xt1=0.6))  # right away
    assert device.sets()[1].t - set_.t == pytest.approx(AQUAERO_GAP_S)
    assert sleep.calls == [pytest.approx(AQUAERO_GAP_S)] * 2
    clock.advance(AQUAERO_GAP_S * 2)
    adapter.apply(_cmd(xt1=0.7))
    assert len(sleep.calls) == 2  # enough time passed: no wait


def test_quadro_writes_back_to_back_without_a_gap() -> None:
    binding = DeviceBinding(kind=QUADRO, pwm_map={"qd1": 1})
    adapter, device, _bus, _clock, sleep = _setup(binding)
    for duty in (0.5, 0.6, 0.7):
        adapter.apply(_cmd(qd1=duty))
    assert len(device.sets()) == 3 and sleep.calls == []


def test_a_failed_operation_counts_for_the_gap() -> None:
    binding = DeviceBinding(kind=AQUAERO, pwm_map={"xt1": 1})
    adapter, device, _bus, _clock, _ = _setup(binding)
    device.failures = [FeatureReportError("EPIPE", errno.EPIPE)]
    adapter.apply(_cmd(xt1=0.5))
    failed_get, retry_get, set_ = device.ops
    assert (failed_get.what, retry_get.what) == ("get", "get")
    assert retry_get.t - failed_get.t == pytest.approx(AQUAERO_GAP_S)
    assert set_.t - retry_get.t == pytest.approx(AQUAERO_GAP_S)


# --- failures, retries and the budget ---------------------------------------------------------


def test_failed_get_is_retried_from_a_fresh_get() -> None:
    binding = DeviceBinding(kind=QUADRO, pwm_map={"qd1": 1})
    adapter, device, _bus, _clock, _ = _setup(binding)
    device.failures = [FeatureReportError("ENODATA", errno.ENODATA)]
    adapter.apply(_cmd(qd1=0.5))
    assert [op.what for op in device.ops] == ["get", "get", "set"]


def test_corrupt_control_report_is_retried() -> None:
    binding = DeviceBinding(kind=QUADRO, pwm_map={"qd1": 1})
    adapter, device, _bus, _clock, _ = _setup(binding)
    bad = bytearray(device.ctrl)
    bad[0x200] ^= 0xFF  # checksum no longer matches
    calls = {"n": 0}
    original = device.get_feature

    def flaky(report_id: int, size: int) -> bytes:
        calls["n"] += 1
        data = original(report_id, size)
        return bytes(bad) if calls["n"] == 1 else data

    device.get_feature = flaky  # type: ignore[method-assign]
    adapter.apply(_cmd(qd1=0.5))
    assert calls["n"] == 2 and len(device.sets()) == 1


def test_retries_exhausted_raise_device_unavailable_and_keep_the_node_open() -> None:
    binding = DeviceBinding(kind=QUADRO, pwm_map={"qd1": 1}, timing=_timing(QUADRO, ctrl_retries=2))
    adapter, device, _bus, clock, _ = _setup(binding)
    device.failures = [FeatureReportError("EPIPE", errno.EPIPE) for _ in range(3)]
    with pytest.raises(DeviceUnavailable, match="failed 3 time"):
        adapter.apply(_cmd(qd1=0.5))
    assert len(device.gets()) == 3 and device.sets() == []
    assert adapter.is_open and adapter.control_report is None
    clock.advance(1.0)
    adapter.apply(_cmd(qd1=0.5))  # next tick works again
    assert len(device.sets()) == 1


def test_zero_retries_raise_on_the_first_failure() -> None:
    binding = DeviceBinding(kind=QUADRO, pwm_map={"qd1": 1}, timing=_timing(QUADRO, ctrl_retries=0))
    adapter, device, _bus, _clock, _ = _setup(binding)
    device.failures = [FeatureReportError("EPIPE", errno.EPIPE)]
    with pytest.raises(DeviceUnavailable):
        adapter.apply(_cmd(qd1=0.5))
    assert len(device.gets()) == 1


def test_no_control_operation_starts_once_the_budget_is_spent() -> None:
    """Review finding: usbhid transfers time out after 5 s each; with retries a tick
    could outlast the systemd watchdog. ctrl_budget_s stops new operations."""
    budget = QUADRO_T.ctrl_budget_s
    binding = DeviceBinding(kind=QUADRO, pwm_map={"qd1": 1}, timing=_timing(QUADRO, ctrl_retries=5))
    adapter, device, _bus, clock, _ = _setup(binding, op_delay_s=budget * 0.6)
    device.failures = [FeatureReportError("ETIMEDOUT", errno.ETIMEDOUT) for _ in range(6)]
    start = clock()
    with pytest.raises(
        DeviceUnavailable, match="ctrl_budget_s.*spent before the control report GET"
    ):
        adapter.apply(_cmd(qd1=0.5))
    assert len(device.gets()) == 2  # the second started inside the budget, the third did not
    assert clock() - start == pytest.approx(budget * 1.2)


def test_budget_spent_before_the_set_leaves_a_rewrite_pending() -> None:
    budget = QUADRO_T.ctrl_budget_s
    binding = DeviceBinding(kind=QUADRO, pwm_map={"qd1": 1, "qd2": 2})
    adapter, device, _bus, clock, _ = _setup(binding, op_delay_s=budget)
    with pytest.raises(DeviceUnavailable, match="before the control report SET"):
        adapter.apply(_cmd(qd1=0.5, qd2=1.0))
    assert device.sets() == []
    device.op_delay_s = 0.0
    clock.advance(1.0)
    adapter.apply(_cmd(qd1=0.5, qd2=1.0))
    assert len(device.sets()) == 1 and device.saves() == []
    assert device.last_set_duties()[:2] == [5000, 10000]  # every channel, held or not


def test_failed_set_rewrites_every_channel_on_the_retry() -> None:
    """A failed SET may or may not have reached the device: the retry sends every
    configured channel."""
    binding = DeviceBinding(kind=QUADRO, pwm_map={"qd1": 1, "qd2": 2})
    adapter, device, _bus, _clock, _ = _setup(binding)
    original = device.set_feature
    state = {"failed": False}

    def fail_set_once(data: bytes) -> None:
        if not state["failed"]:
            state["failed"] = True
            device.ops.append(type(device.ops[0])("set", device.clock(), bytes(data)))
            raise FeatureReportError("EPIPE", errno.EPIPE)
        original(data)

    device.set_feature = fail_set_once  # type: ignore[method-assign]
    adapter.apply(_cmd(qd1=0.5, qd2=1.0))
    assert [op.what for op in device.ops] == ["get", "set", "get", "set"]
    # The retry rewrites every configured channel although the device already holds them.
    retry = device.sets()[1].data
    assert control_duty(QUADRO, retry, 0) == 5000 and control_duty(QUADRO, retry, 1) == DUTY_MAX


def test_vanished_device_during_apply_closes_without_retrying() -> None:
    binding = DeviceBinding(kind=QUADRO, pwm_map={"qd1": 1})
    adapter, device, _bus, _clock, _ = _setup(binding)
    adapter.read()
    device.failures = [DeviceUnavailable("gone")]
    with pytest.raises(DeviceUnavailable, match="gone"):
        adapter.apply(_cmd(qd1=0.5))
    assert len(device.gets()) == 1 and not adapter.is_open


class _RawCommand:
    """Duck-typed command that skips MpcCommand's own validation."""

    def __init__(self, pwm) -> None:
        self.pwm = pwm


@pytest.mark.parametrize(
    "pwm",
    [
        {"xt1": 0.5},  # xt2 missing
        {"xt1": math.nan, "xt2": 0.2},
        {"xt1": math.inf, "xt2": 0.2},
        {"xt1": 1.5, "xt2": 0.2},
        {"xt1": -0.1, "xt2": 0.2},
        {"xt1": True, "xt2": 0.2},
        {"xt1": "0.5", "xt2": 0.2},
    ],
    ids=["missing", "nan", "inf", "above", "below", "bool", "string"],
)
def test_invalid_command_raises_value_error_before_touching_the_device(pwm) -> None:
    adapter, device, bus, _clock, _ = _setup(_aquaero_binding())
    with pytest.raises(ValueError):
        adapter.apply(_RawCommand(pwm))  # type: ignore[arg-type]
    assert bus.opened == [] and device.ops == []


# --- write limiting (USB traffic only since item 86; off by default) --------------------------


def _limited_quadro(**timing):
    binding = DeviceBinding(
        kind=QUADRO, pwm_map={"qd1": 1, "qd2": 2}, timing=_timing(QUADRO, **{**LIMITS, **timing})
    )
    adapter, device, _bus, clock, _ = _setup(binding)
    adapter.apply(_cmd(qd1=0.5, qd2=0.5))  # the first write: nothing written before it
    assert len(device.sets()) == 1
    return adapter, device, clock


def test_a_rise_is_written_at_once() -> None:
    adapter, device, clock = _limited_quadro()
    clock.advance(1.0)
    adapter.apply(_cmd(qd1=0.5001, qd2=0.5))
    assert len(device.sets()) == 2 and control_duty(QUADRO, device.ctrl, 0) == 5001


def test_a_fall_waits_for_the_minimum_interval() -> None:
    adapter, device, clock = _limited_quadro()
    interval = LIMITS["write_min_interval_s"]
    clock.advance(interval / 3)
    device.emit()
    obs = adapter.read()
    adapter.apply(_cmd(qd1=0.3, qd2=0.5))
    assert len(device.sets()) == 1
    assert obs.pwm["qd1"] == 0.5  # the controller sees what the fan gets
    clock.advance(interval / 3)
    adapter.apply(_cmd(qd1=0.3, qd2=0.5))
    assert len(device.sets()) == 1
    clock.advance(interval / 3)
    adapter.apply(_cmd(qd1=0.3, qd2=0.5))
    assert len(device.sets()) == 2 and control_duty(QUADRO, device.ctrl, 0) == 3000


def test_a_fall_inside_the_deadband_is_not_written_on_its_own() -> None:
    adapter, device, clock = _limited_quadro()
    clock.advance(LIMITS["write_min_interval_s"] * 10)
    just_inside = (5000 - (LIMITS["write_deadband"] - 1)) / DUTY_MAX
    adapter.apply(_cmd(qd1=just_inside, qd2=0.5))
    assert len(device.sets()) == 1
    at_the_band = (5000 - LIMITS["write_deadband"]) / DUTY_MAX
    adapter.apply(_cmd(qd1=at_the_band, qd2=0.5))
    assert len(device.sets()) == 2


def test_a_write_carries_every_pending_change() -> None:
    adapter, device, clock = _limited_quadro()
    clock.advance(1.0)
    adapter.apply(_cmd(qd1=0.3, qd2=0.5))  # a fall, deferred
    adapter.apply(_cmd(qd1=0.3, qd2=0.6))  # a rise: goes out with the pending fall
    assert len(device.sets()) == 2
    assert device.last_set_duties()[:2] == [3000, 6000]


def test_the_default_limits_write_every_change() -> None:
    """Item 86: writes are not saved, so the defaults write every fall at once."""
    binding = DeviceBinding(kind=QUADRO, pwm_map={"qd1": 1, "qd2": 2})
    assert (binding.timing.write_min_interval_s, binding.timing.write_deadband) == (0, 0)
    adapter, device, _bus, _clock, _ = _setup(binding)
    for duty in (0.5, 0.4999, 0.4998, 0.4997):
        adapter.apply(_cmd(qd1=duty, qd2=0.5))
    assert len(device.sets()) == 4 and device.saves() == []


# --- never lower a fan during a fault (second review) ----------------------------------------

FAULT_MODES = [Mode.FALLBACK, Mode.DEGRADED]


def _ab_quadro(**timing):
    """qd1 ("a") written at 60 %, qd2 ("b") at 30 %, then a fall of a to 40 % deferred."""
    timing = _timing(QUADRO, **{**LIMITS, **timing})
    binding = DeviceBinding(kind=QUADRO, pwm_map={"a": 1, "b": 2}, timing=timing)
    adapter, device, _bus, clock, _ = _setup(binding)
    adapter.read()
    adapter.apply(_cmd(a=0.6, b=0.3))
    clock.advance(1.0)
    adapter.apply(_cmd(a=0.4, b=0.3))  # the loop now counts 40 % as applied
    assert len(device.sets()) == 1 and control_duty(QUADRO, device.ctrl, 0) == 6000
    return adapter, device, clock


def _fault(mode: Mode, **pwm: float) -> MpcCommand:
    return MpcCommand(pwm=pwm, mode=mode)


@pytest.mark.parametrize("mode", FAULT_MODES, ids=lambda m: m.value)
def test_a_rise_during_a_fault_does_not_carry_a_deferred_fall(mode: Mode) -> None:
    adapter, device, clock = _ab_quadro()
    clock.advance(1.0)
    adapter.apply(_fault(mode, a=0.45, b=0.35))  # b rises: a write goes out
    assert len(device.sets()) == 2
    assert device.last_set_duties()[:2] == [6000, 3500]  # a is not lowered


@pytest.mark.parametrize("mode", FAULT_MODES, ids=lambda m: m.value)
def test_a_deferred_fall_does_not_mature_during_a_fault(mode: Mode) -> None:
    adapter, device, clock = _ab_quadro()
    clock.advance(LIMITS["write_min_interval_s"])
    adapter.apply(_fault(mode, a=0.4, b=0.3))
    assert len(device.sets()) == 1 and control_duty(QUADRO, device.ctrl, 0) == 6000
    adapter.apply(_cmd(a=0.4, b=0.3))  # back in AUTO the fall goes out as before
    assert len(device.sets()) == 2 and control_duty(QUADRO, device.ctrl, 0) == 4000


@pytest.mark.parametrize("mode", FAULT_MODES, ids=lambda m: m.value)
def test_a_forced_rewrite_during_a_fault_does_not_lower_a_fan(mode: Mode) -> None:
    adapter, device, clock = _ab_quadro()
    assert device.power_cycles is not None
    device.power_cycles += 1
    clock.advance(1.0)
    device.emit()
    adapter.read()  # invalidates the cache and forces a rewrite
    assert adapter.control_report is None
    adapter.apply(_fault(mode, a=0.4, b=0.3))
    assert [op.what for op in device.ops[-2:]] == ["get", "set"]
    assert device.last_set_duties()[:2] == [6000, 3000]


@pytest.mark.parametrize("mode", FAULT_MODES, ids=lambda m: m.value)
def test_the_rewrite_after_a_failed_write_during_a_fault_does_not_lower_a_fan(mode: Mode) -> None:
    adapter, device, clock = _ab_quadro()
    clock.advance(1.0)
    device.failures = [FeatureReportError("EPIPE", errno.EPIPE)]  # the SET below fails
    adapter.apply(_fault(mode, a=0.4, b=0.35))
    assert device.last_set_duties()[:2] == [6000, 3500]
    assert control_duty(QUADRO, device.ctrl, 0) == 6000


def test_auto_still_falls_as_before() -> None:
    adapter, device, clock = _ab_quadro()
    clock.advance(1.0)
    adapter.apply(_cmd(a=0.4, b=0.35))  # the rise carries the pending fall in AUTO
    assert device.last_set_duties()[:2] == [4000, 3500]


@pytest.mark.parametrize("with_status", [True, False], ids=["status", "no-status"])
def test_a_channel_on_a_firmware_controller_is_not_lowered_during_a_fault(
    with_status: bool,
) -> None:
    """Its preset says nothing about the output: the floor is the reported output duty,
    or 100 % before any status report."""
    binding = DeviceBinding(kind=AQUAERO, pwm_map={"xt1": 1})
    adapter, device, _bus, _clock, _ = _setup(binding, report_delay_s=None if with_status else 1e9)
    if with_status:
        assert adapter.read().pwm["xt1"] == 1.0
    adapter.apply(_fault(Mode.FALLBACK, xt1=0.8))
    assert channel_holds(AQUAERO, device.ctrl, 0, DUTY_MAX)


# --- keeping the cache honest ---------------------------------------------------------------


def _commanded_quadro(**timing):
    binding = DeviceBinding(
        kind=QUADRO, pwm_map={"qd1": 1, "qd2": 2}, timing=_timing(QUADRO, **timing)
    )
    adapter, device, _bus, clock, _ = _setup(binding)
    adapter.read()  # as the loop does: read, then apply
    adapter.apply(_cmd(qd1=0.5, qd2=0.5))
    assert len(device.sets()) == 1
    return adapter, device, clock


def _tick(adapter, device, clock, seconds: float, **pwm: float) -> None:
    clock.advance(seconds)
    device.emit()
    adapter.read()
    adapter.apply(_cmd(**pwm))


def test_duty_mismatch_longer_than_the_limit_rereads_and_rewrites_every_channel(caplog) -> None:
    adapter, device, clock = _commanded_quadro()
    device.duty_override = {0: 5000 + QUADRO_T.duty_mismatch_tolerance + 1}
    _tick(adapter, device, clock, 1.0, qd1=0.5, qd2=0.5)  # mismatch starts
    _tick(adapter, device, clock, QUADRO_T.duty_mismatch_s, qd1=0.5, qd2=0.5)  # not longer
    assert len(device.gets()) == 1 and len(device.sets()) == 1
    with caplog.at_level("WARNING", logger=LOGGER):
        clock.advance(0.5)
        device.emit()
        adapter.read()
    assert adapter.control_report is None
    assert "pwm1 (qd1)" in caplog.text and "50.00 %" in caplog.text and "51.01 %" in caplog.text
    adapter.apply(_cmd(qd1=0.5, qd2=0.5))
    assert len(device.gets()) == 2 and len(device.sets()) == 2
    assert device.last_set_duties()[:2] == [5000, 5000]  # every configured channel


def test_brief_duty_mismatch_does_not_rewrite() -> None:
    adapter, device, clock = _commanded_quadro()
    step = QUADRO_T.duty_mismatch_s * 0.4
    device.duty_override = {0: 9000}
    _tick(adapter, device, clock, step, qd1=0.5, qd2=0.5)
    _tick(adapter, device, clock, step, qd1=0.5, qd2=0.5)
    device.duty_override = {}
    _tick(adapter, device, clock, step, qd1=0.5, qd2=0.5)  # back in line: timer cleared
    device.duty_override = {0: 9000}
    _tick(adapter, device, clock, step, qd1=0.5, qd2=0.5)
    _tick(adapter, device, clock, step, qd1=0.5, qd2=0.5)
    assert len(device.gets()) == 1 and len(device.sets()) == 1


def test_duty_mismatch_within_tolerance_is_no_mismatch() -> None:
    adapter, device, clock = _commanded_quadro()
    device.duty_override = {0: 5000 - QUADRO_T.duty_mismatch_tolerance}
    for _ in range(10):
        _tick(adapter, device, clock, QUADRO_T.duty_mismatch_s, qd1=0.5, qd2=0.5)
    assert len(device.gets()) == 1 and len(device.sets()) == 1


def test_mismatch_fires_while_another_channel_changes_every_tick(caplog) -> None:
    """Review finding: a SET that did not change the mismatching channel used to
    restart its timer, so with dt <= duty_mismatch_s and a command changing every
    tick the rule never fired."""
    dt = QUADRO_T.duty_mismatch_s / 2.5
    adapter, device, clock = _commanded_quadro()
    device.duty_override = {0: 9000}
    with caplog.at_level("WARNING", logger=LOGGER):
        for n in range(1, 5):
            _tick(adapter, device, clock, dt, qd1=0.5, qd2=0.5 + 0.01 * n)
    assert len(device.sets()) == 5  # qd2 rose every tick
    assert len(_messages(caplog, "WARNING")) == 1
    assert len(device.gets()) == 2  # the re-read before the rewrite


def test_a_write_that_changes_the_channel_restarts_its_timer() -> None:
    limit = QUADRO_T.duty_mismatch_s
    adapter, device, clock = _commanded_quadro()
    device.duty_override = {0: 9000}
    _tick(adapter, device, clock, 1.0, qd1=0.5, qd2=0.5)  # mismatch starts
    _tick(adapter, device, clock, limit * 0.5, qd1=0.6, qd2=0.5)  # qd1 itself changes
    _tick(adapter, device, clock, 1.0, qd1=0.6, qd2=0.5)  # new mismatch starts here
    _tick(adapter, device, clock, limit * 0.8, qd1=0.6, qd2=0.5)
    assert adapter.control_report is not None  # past the old start, not past the new one
    clock.advance(limit * 0.3)
    device.emit()
    adapter.read()
    assert adapter.control_report is None


def test_no_new_report_is_no_evidence() -> None:
    adapter, device, clock = _commanded_quadro(status_max_age_s=1000.0)
    device.duty_override = {0: 9000}
    for _ in range(5):
        clock.advance(QUADRO_T.duty_mismatch_s)
        adapter.read()  # the report from the first open only
    assert adapter.control_report is not None


def test_a_channel_the_device_ignores_is_rewritten_once_then_reported_stuck(caplog) -> None:
    """Hardware 2026-09-15: outputs without a fan stayed at 100 % whatever was written.
    One rewrite, then one error, no rewrite loop; stuck until the device follows."""
    dt = QUADRO_T.duty_mismatch_s / 2
    adapter, device, clock = _commanded_quadro(ctrl_refresh_s=0)
    device.ignores = {0: DUTY_MAX}
    with caplog.at_level("INFO", logger=LOGGER):
        for _ in range(30):
            _tick(adapter, device, clock, dt, qd1=0.5, qd2=0.5)
        assert len(_messages(caplog, "WARNING")) == 1
        assert len(_messages(caplog, "ERROR")) == 1
        assert "pwm1 (qd1) still reports 100.00 %" in _messages(caplog, "ERROR")[0]
        assert adapter.stuck_channels == ("qd1",)
        assert len(device.gets()) == 2 and len(device.sets()) == 2
        # A changed command is still written; the channel stays stuck, quietly.
        for _ in range(10):
            _tick(adapter, device, clock, dt, qd1=0.6, qd2=0.5)
        assert len(device.sets()) == 3
        assert len(_messages(caplog, "WARNING")) == 1 and len(_messages(caplog, "ERROR")) == 1
        assert adapter.stuck_channels == ("qd1",)
        # The device follows again: no longer stuck.
        device.ignores = {}
        _tick(adapter, device, clock, dt, qd1=0.6, qd2=0.5)
    assert adapter.stuck_channels == ()
    assert any("follows its duty again" in m for m in _messages(caplog, "INFO"))


def test_configured_mismatch_limits_are_used() -> None:
    adapter, device, clock = _commanded_quadro(duty_mismatch_tolerance=2000, duty_mismatch_s=1.0)
    device.duty_override = {0: 6500}
    for _ in range(3):
        _tick(adapter, device, clock, 2.0, qd1=0.5, qd2=0.5)
    assert len(device.sets()) == 1
    device.duty_override = {0: 7500}
    _tick(adapter, device, clock, 2.0, qd1=0.5, qd2=0.5)  # starts
    clock.advance(2.0)
    device.emit()
    adapter.read()
    assert adapter.control_report is None


def test_quadro_power_cycle_rereads_and_rewrites(caplog) -> None:
    adapter, device, clock = _commanded_quadro()
    assert device.power_cycles is not None
    device.power_cycles += 1
    with caplog.at_level("WARNING", logger=LOGGER):
        clock.advance(1.0)
        device.emit()
        adapter.read()
    assert "power-cycle count changed" in caplog.text
    assert adapter.control_report is None
    adapter.apply(_cmd(qd1=0.5, qd2=0.5))
    assert len(device.gets()) == 2 and len(device.sets()) == 2
    clock.advance(1.0)
    device.emit()
    adapter.read()  # the new count is the reference now
    adapter.apply(_cmd(qd1=0.5, qd2=0.5))
    assert len(device.gets()) == 2 and len(device.sets()) == 2


def test_periodic_refresh_rereads_and_rewrites_a_changed_channel(caplog) -> None:
    adapter, device, clock = _commanded_quadro()
    external = bytearray(device.ctrl)
    patch_duties(QUADRO, external, {1: 8000, 3: 4321})  # qd2 (owned) and an unowned channel
    finalize_control_report(QUADRO, external)
    device.ctrl = external
    device.duty_override = {1: 5000}  # the status report has not caught up either way
    _tick(adapter, device, clock, QUADRO_T.ctrl_refresh_s / 2, qd1=0.5, qd2=0.5)
    assert len(device.gets()) == 1
    with caplog.at_level("WARNING", logger=LOGGER):
        _tick(adapter, device, clock, QUADRO_T.ctrl_refresh_s / 2, qd1=0.5, qd2=0.5)
    assert len(device.gets()) == 2 and len(device.sets()) == 2
    assert "pwm2 (qd2)" in caplog.text
    assert control_duty(QUADRO, device.ctrl, 1) == 5000
    assert control_duty(QUADRO, device.ctrl, 3) == 4321  # the external change elsewhere stays


def test_periodic_refresh_without_drift_writes_nothing() -> None:
    adapter, device, clock = _commanded_quadro()
    _tick(adapter, device, clock, QUADRO_T.ctrl_refresh_s, qd1=0.5, qd2=0.5)
    assert len(device.gets()) == 2 and len(device.sets()) == 1


def test_periodic_refresh_disabled_with_zero() -> None:
    adapter, device, clock = _commanded_quadro(ctrl_refresh_s=0)
    for _ in range(5):
        _tick(adapter, device, clock, 3600.0, qd1=0.5, qd2=0.5)
    assert len(device.gets()) == 1


# --- release ------------------------------------------------------------------------------


def test_release_restores_the_captured_aquaero_fields() -> None:
    binding = DeviceBinding(kind=AQUAERO, pwm_map={"xt2": 2})
    adapter, device, _bus, clock, _ = _setup(binding)
    firmware = fixture_bytes("aquaero-ctrl-firmware.bin")
    adapter.apply(_cmd(xt2=0.1412))
    clock.advance(5.0)
    adapter.apply(_cmd(xt2=0.3))
    adapter.release()
    assert bytes(device.ctrl) == firmware
    assert [op.what for op in device.ops[-2:]] == ["get", "set"]  # live, not saved
    assert device.saves() == []
    ops = len(device.ops)
    adapter.release()  # nothing written since: no-op
    assert len(device.ops) == ops


def test_release_restores_only_written_channels_and_keeps_other_changes() -> None:
    binding = DeviceBinding(kind=QUADRO, pwm_map={"qd1": 1, "qd3": 3})
    adapter, device, _bus, clock, _ = _setup(binding)
    firmware = fixture_bytes("quadro-ctrl-firmware.bin")
    adapter.apply(_cmd(qd1=1.0, qd3=0.0902))  # qd1 already holds 100 %: only qd3 written
    external = bytearray(device.ctrl)
    patch_duties(QUADRO, external, {1: 4321})
    finalize_control_report(QUADRO, external)
    device.ctrl = external
    clock.advance(1.0)
    adapter.release()
    assert capture_channel(QUADRO, device.ctrl, 2) == capture_channel(QUADRO, firmware, 2)
    assert control_duty(QUADRO, device.ctrl, 1) == 4321


def test_release_without_a_write_is_a_noop() -> None:
    adapter, device, bus, _clock, _ = _setup(_aquaero_binding())
    adapter.release()
    assert bus.opened == [] and device.ops == []
    adapter.read()
    adapter.release()
    assert device.ops == []


def test_release_restores_what_the_first_get_saw_even_after_a_replug() -> None:
    binding = DeviceBinding(kind=AQUAERO, pwm_map={"xt2": 2})
    adapter, device, bus, clock, _ = _setup(binding)
    adapter.apply(_cmd(xt2=0.5))
    device.gone = True
    with pytest.raises(DeviceUnavailable):
        adapter.read()
    replugged = FakeController(AQUAERO, clock, node="/dev/hidraw5", ctrl=bytearray(device.ctrl))
    bus.controllers = [replugged]
    clock.advance(5.0)
    adapter.apply(_cmd(xt2=0.6))
    adapter.release()
    assert bytes(replugged.ctrl) == fixture_bytes("aquaero-ctrl-firmware.bin")


# --- save (item 86: commissioning only) -----------------------------------------------------


@pytest.mark.parametrize("kind", [AQUAERO, QUADRO], ids=lambda k: k.name)
def test_save_sends_exactly_one_save_report(kind: DeviceKind, caplog) -> None:
    binding = DeviceBinding(kind=kind, pwm_map={"a": 1})
    adapter, device, _bus, _clock, sleep = _setup(binding)
    adapter.apply(_cmd(a=0.5))
    assert device.saves() == []
    with caplog.at_level("WARNING", logger=LOGGER):
        adapter.save()
    assert [op.what for op in device.ops] == ["get", "set", "save"]
    assert device.saves()[0].data == kind.save_report
    if kind is AQUAERO:
        assert device.saves()[0].t - device.sets()[0].t == pytest.approx(AQUAERO_GAP_S)
    message = next(m for m in _messages(caplog, "WARNING") if "save report" in m)
    assert ("not verified" in message) is (kind is QUADRO)
    adapter.apply(_cmd(a=0.6))  # later writes are live again
    assert len(device.saves()) == 1 and len(device.sets()) == 2


def test_save_is_not_retried() -> None:
    binding = DeviceBinding(kind=QUADRO, pwm_map={"a": 1}, timing=_timing(QUADRO, ctrl_retries=3))
    adapter, device, _bus, _clock, _ = _setup(binding)
    device.failures = [FeatureReportError("EPIPE", errno.EPIPE)]
    with pytest.raises(DeviceUnavailable, match="save report failed 1 time"):
        adapter.save()
    assert [op.what for op in device.ops] == ["save"]


def test_nothing_the_daemon_does_sends_the_save_report() -> None:
    """Writes, forced rewrites, the periodic refresh and release() all stay live."""
    adapter, device, clock = _commanded_quadro()
    device.duty_override = {0: 9000}
    for _ in range(6):
        _tick(adapter, device, clock, QUADRO_T.duty_mismatch_s, qd1=0.5, qd2=0.6)
    _tick(adapter, device, clock, QUADRO_T.ctrl_refresh_s, qd1=0.7, qd2=0.6)
    adapter.release()
    assert len(device.sets()) >= 3 and device.saves() == []


# --- control_snapshot (item 88: the commissioning tool's read of what save() would store) ----


def test_control_snapshot_is_a_get_only_and_caches_the_report() -> None:
    binding = DeviceBinding(kind=AQUAERO, pwm_map={"a": 1})
    adapter, device, _bus, _clock, _sleep = _setup(binding)
    assert adapter.control_report is None

    report = adapter.control_snapshot()

    assert report == bytes(device.ctrl)
    assert adapter.control_report == report
    assert [op.what for op in device.ops] == ["get"]
    assert device.sets() == [] and device.saves() == []


def test_control_snapshot_is_not_retried() -> None:
    binding = DeviceBinding(kind=QUADRO, pwm_map={"a": 1}, timing=_timing(QUADRO, ctrl_retries=3))
    adapter, device, _bus, _clock, _sleep = _setup(binding)
    device.failures = [FeatureReportError("EPIPE", errno.EPIPE)]
    with pytest.raises(DeviceUnavailable, match="control report GET failed 1 time"):
        adapter.control_snapshot()
    assert [op.what for op in device.ops] == ["get"]

    report = adapter.control_snapshot()  # the fake device answers normally now
    assert report == bytes(device.ctrl) and [op.what for op in device.ops] == ["get", "get"]


# --- software-sensor heartbeat and the active profile (item 84) ------------------------------

#: The owner's configuration: software sensor 1, a heartbeat well below its alarm.
HEARTBEAT = {"heartbeat_sensor": 1, "heartbeat_value_c": 20.0}


def _heartbeat_aquaero(**timing):
    binding = DeviceBinding(
        kind=AQUAERO,
        pwm_map={"xt1": 1, "xt2": 2},
        timing=_timing(AQUAERO, **{**HEARTBEAT, **timing}),
    )
    return _setup(binding)


def test_the_heartbeat_report_carries_the_configured_sensor_only(caplog) -> None:
    """Report 0x07, 17 bytes: the id then eight centi-degC values, the configured
    sensor's own and 0x7FFF ("no data") for the other seven, so the device keeps
    using its own values for them (PROJECT.md section 8 item 84)."""
    adapter, device, _bus, clock, _ = _heartbeat_aquaero()
    assert adapter.heartbeat_on and adapter.heartbeat_ok is None
    adapter.read()
    with caplog.at_level("INFO", logger=LOGGER):
        adapter.apply(_cmd(xt1=0.5, xt2=0.5))
    (write,) = device.writes()
    assert write.data == software_sensor_report(AQUAERO, {1: 20.0})
    assert len(write.data) == 17 and write.data[0] == 0x07
    assert write.data[1:3] == b"\x07\xd0"  # 20.00 degC, big-endian centi-degC
    assert write.data[3:] == b"\x7f\xff" * 7
    assert [op.what for op in device.ops] == ["get", "set", "write"]  # after the duty work
    assert adapter.heartbeat_ok is True
    assert device.soft_sensors == {1: 20.0}  # and the device reports it back
    clock.advance(1.0)
    device.emit()
    # It reads back in the status report -- but as softN, which no config may bind
    # (PROJECT.md section 8 item 113), so it arrives nowhere near an observation.
    adapter.read()
    assert adapter.last_status is not None
    assert adapter.last_status.temp("soft1") == pytest.approx(20.0)
    assert [m for m in _messages(caplog, "INFO") if "heartbeat" in m] == [
        "aquaero: writing the software-sensor heartbeat of 20.00 degC to soft1 every write"
    ]


def test_the_heartbeat_goes_out_every_tick_and_adds_no_control_report_read() -> None:
    """One heartbeat per apply(), whether or not a duty changed, and no GET per tick:
    a profile switch is found by the periodic refresh and the duty verification."""
    adapter, device, _bus, clock, _ = _heartbeat_aquaero()
    for _ in range(10):
        adapter.read()
        adapter.apply(_cmd(xt1=0.5, xt2=0.5))
        clock.advance(1.0)
        device.emit()
    assert len(device.writes()) == 10  # every tick, although only the first changed a duty
    assert len(device.sets()) == 1 and len(device.gets()) == 1


def test_the_heartbeat_is_off_by_default() -> None:
    adapter, device, _bus, _clock, _ = _setup(_aquaero_binding())
    assert not adapter.heartbeat_on and adapter.heartbeat_ok is None
    adapter.read()
    adapter.apply(_cmd(xt1=0.5, xt2=0.5))
    assert device.writes() == [] and len(device.sets()) == 1
    assert adapter.heartbeat_ok is None


def test_the_heartbeat_waits_the_control_gap_and_is_not_repeated_on_a_retry() -> None:
    adapter, device, _bus, _clock, _ = _heartbeat_aquaero()
    device.failures = [FeatureReportError("ETIMEDOUT")]  # the first GET fails and is retried
    adapter.apply(_cmd(xt1=0.5, xt2=0.5))
    assert len(device.writes()) == 1 and len(device.gets()) == 2 and len(device.sets()) == 1
    write, last_set = device.writes()[0], device.sets()[-1]
    assert write.t - last_set.t == pytest.approx(AQUAERO_GAP_S)


def test_a_failed_heartbeat_does_not_fail_the_tick_and_is_logged_once(caplog) -> None:
    adapter, device, _bus, clock, _ = _heartbeat_aquaero()
    with caplog.at_level("INFO", logger=LOGGER):
        adapter.read()
        adapter.apply(_cmd(xt1=0.5, xt2=0.5))
        device.write_failures = [FeatureReportError("EPIPE") for _ in range(3)]
        for duty in (0.6, 0.7, 0.8):
            clock.advance(1.0)
            device.emit()
            adapter.read()
            adapter.apply(_cmd(xt1=duty, xt2=0.5))  # the duties still go out
        assert len(device.sets()) == 4 and adapter.heartbeat_ok is False
        clock.advance(1.0)
        device.emit()
        adapter.read()
        adapter.apply(_cmd(xt1=0.9, xt2=0.5))
    assert adapter.heartbeat_ok is True
    (error,) = [m for m in _messages(caplog, "ERROR") if "heartbeat" in m]
    assert "soft1" in error and "EPIPE" in error and "falls back" in error
    assert [m for m in _messages(caplog, "INFO") if "heartbeat" in m] == [
        "aquaero: writing the software-sensor heartbeat of 20.00 degC to soft1 every write",
        "aquaero: writing the software-sensor heartbeat of 20.00 degC to soft1 again",
    ]


def test_a_tick_that_cannot_command_the_duties_sends_no_heartbeat(caplog) -> None:
    """The failure the controller's own watchdog uniquely covers: the daemon runs and
    the node answers, but every control operation fails. Feeding the software sensor
    then would hold the watchdog shut while nothing commands the fans, so the
    heartbeat goes out only behind an apply() that reached the device (item 84)."""
    adapter, device, _bus, clock, _ = _heartbeat_aquaero()
    adapter.read()
    adapter.apply(_cmd(xt1=0.5, xt2=0.5))
    assert len(device.writes()) == 1 and adapter.heartbeat_ok is True
    device.failures = [FeatureReportError("EPIPE") for _ in range(20)]
    with caplog.at_level("WARNING", logger=LOGGER):
        for duty in (0.6, 0.7, 0.8):
            clock.advance(1.0)
            device.emit()
            adapter.read()  # the observation still comes through
            with pytest.raises(DeviceUnavailable, match="control report write failed"):
                adapter.apply(_cmd(xt1=duty, xt2=0.5))
    assert len(device.writes()) == 1  # none since the last duty that went out
    assert device.soft_sensors == {1: 20.0}  # the real device now runs down its own timeout
    # None was attempted, so there is no heartbeat failure either: only the write failed.
    assert [m for m in _messages(caplog, "ERROR") if "heartbeat" in m] == []
    assert adapter.heartbeat_ok is True  # the state of the last one actually sent


def test_a_budget_spent_on_the_duties_leaves_the_heartbeat_unsent() -> None:
    """Same rule through the budget path: the control work fails, so nothing feeds the
    software sensor. The worst case per tick is still worst_case_tick_s."""
    timing = _timing(AQUAERO, **HEARTBEAT, ctrl_budget_s=2.0)
    binding = DeviceBinding(kind=AQUAERO, pwm_map={"xt1": 1}, timing=timing)
    adapter, device, _bus, clock, _ = _setup(binding, op_delay_s=2.5)
    with pytest.raises(DeviceUnavailable, match="ctrl_budget_s"):
        adapter.apply(_cmd(xt1=0.5))
    assert device.writes() == [] and len(device.gets()) == 1 and device.sets() == []
    assert clock() - 100.0 <= timing.worst_case_tick_s()


def test_a_heartbeat_with_no_budget_left_drops_the_heartbeat_not_the_tick(caplog) -> None:
    """The heartbeat runs last and inside ctrl_budget_s: a duty write that spent the
    budget leaves it unsent, and that is logged, not raised -- the duties went out."""
    timing = _timing(AQUAERO, **HEARTBEAT, ctrl_budget_s=1.95)
    binding = DeviceBinding(kind=AQUAERO, pwm_map={"xt1": 1}, timing=timing)
    adapter, device, _bus, _clock, _ = _setup(binding, op_delay_s=0.9)
    with caplog.at_level("ERROR", logger=LOGGER):
        adapter.apply(_cmd(xt1=0.5))  # GET, then SET, then no budget for the write
    assert len(device.sets()) == 1 and device.writes() == []
    assert adapter.heartbeat_ok is False
    (error,) = [m for m in _messages(caplog, "ERROR") if "heartbeat" in m]
    assert "ctrl_budget_s spent" in error


def test_the_heartbeat_stops_when_the_serial_does_not_match() -> None:
    """Nothing is written to a device that is not the configured one -- the heartbeat
    included, so the controller's own watchdog takes over (the safe direction)."""
    binding = DeviceBinding(
        kind=AQUAERO,
        pwm_map={"xt1": 1},
        serial="12345-54321",
        timing=_timing(AQUAERO, **HEARTBEAT),
    )
    adapter, device, _bus, _clock, _ = _setup(binding, status_serial="54321-12345")
    with pytest.raises(DeviceUnavailable, match="not the configured serial"):
        adapter.read()
    with pytest.raises(DeviceUnavailable, match="nothing is written"):
        adapter.apply(_cmd(xt1=0.5))
    assert device.writes() == [] and device.sets() == []


def _profile_switch(device: FakeController, profile: int, duty: int) -> None:
    """The alarm selects another profile: byte 0x06 changes and the saved profile's
    duties come back (every live write is gone)."""
    device.ctrl[0x06] = profile - 1
    patch_duties(AQUAERO, device.ctrl, dict.fromkeys(range(AQUAERO.pwm_count), duty))


def test_a_changed_profile_byte_makes_the_next_write_send_every_channel(caplog) -> None:
    adapter, device, _bus, clock, _ = _setup(
        DeviceBinding(kind=AQUAERO, pwm_map={"xt1": 1, "xt2": 2})
    )
    adapter.read()
    adapter.apply(_cmd(xt1=0.5, xt2=0.6))
    assert adapter.active_profile == 1 and len(device.sets()) == 1
    with caplog.at_level("WARNING", logger=LOGGER):
        _profile_switch(device, 2, 10000)  # profile 2: every output at 100 %
        _tick(adapter, device, clock, 1.0, xt1=0.5, xt2=0.6)  # the mismatch starts
        assert len(device.gets()) == 1  # still no control read per tick
        _tick(adapter, device, clock, AQUAERO_T.duty_mismatch_s + 0.5, xt1=0.5, xt2=0.6)
    assert len(device.gets()) == 2 and len(device.sets()) == 2
    assert device.last_set_duties()[:2] == [5000, 6000]  # every configured channel again
    assert adapter.active_profile == 2
    assert [m for m in _messages(caplog, "WARNING") if "profile" in m] == [
        "aquaero: the active profile changed from profile 1 to profile 2; the switch reloads "
        "the saved profile, so every duty written live is gone: writing every configured "
        "channel again (PROJECT.md section 8 item 84)"
    ]


def test_a_profile_switch_the_duties_do_not_betray_is_found_by_the_periodic_refresh(
    caplog,
) -> None:
    """The reloaded profile happens to drive the same duty, so no duty verification
    fires; ctrl_refresh_s finds it, and until then the fans run the profile the alarm
    chose, which is the safe one."""
    refresh = AQUAERO_T.ctrl_refresh_s
    adapter, device, _bus, clock, _ = _setup(
        DeviceBinding(kind=AQUAERO, pwm_map={"xt1": 1, "xt2": 2})
    )
    adapter.read()
    adapter.apply(_cmd(xt1=0.5, xt2=0.5))
    _profile_switch(device, 2, 5000)
    with caplog.at_level("WARNING", logger=LOGGER):
        _tick(adapter, device, clock, refresh / 2, xt1=0.5, xt2=0.5)
        assert len(device.gets()) == 1 and [m for m in _messages(caplog, "WARNING")] == []
        _tick(adapter, device, clock, refresh / 2 + 1.0, xt1=0.5, xt2=0.5)
    assert len(device.gets()) == 2 and adapter.active_profile == 2
    assert len(device.sets()) == 2 and device.last_set_duties()[:2] == [5000, 5000]
    assert len([m for m in _messages(caplog, "WARNING") if "profile" in m]) == 1


def test_the_profile_is_only_read_where_the_kind_has_one() -> None:
    adapter, device, _bus, _clock, _ = _setup(_quadro_binding())
    adapter.apply(_cmd(qd1=0.5, qd3=0.5))
    assert adapter.active_profile is None and len(device.gets()) == 1


# --- aquabus outputs 5-8 (item 85) ----------------------------------------------------------


def _aquabus(binding: DeviceBinding, **fields):
    clock = FakeClock()
    sleep = FakeSleep(clock)
    device = aquabus_aquaero(clock, **fields)
    adapter = AquacomputerAdapter(binding, clock=clock, sleep=sleep, opener=FakeBus(device))
    return adapter, device, clock


def _aquabus_all_configured(binding: DeviceBinding, **fields):
    clock = FakeClock()
    sleep = FakeSleep(clock)
    device = aquabus_aquaero_all_configured(clock, **fields)
    adapter = AquacomputerAdapter(binding, clock=clock, sleep=sleep, opener=FakeBus(device))
    return adapter, device, clock


_QUADRO_ON_AQUABUS = DeviceBinding(
    kind=AQUAERO,
    pwm_map={"xt1": 1, "qd1": 5, "qd2": 6, "qd3": 7, "qd4": 8},
    fan_map={"qd3": 7},
    temp_map={"quadro_t2": "bus2"},
)


def test_aquabus_outputs_read_like_the_aquaeros_own() -> None:
    adapter, _device, _clock = _aquabus(_QUADRO_ON_AQUABUS)
    obs = adapter.read()
    assert obs.temps == {"quadro_t2": pytest.approx(24.14)}
    assert obs.rpm == {"qd3": 1105.0}
    assert obs.pwm == {
        "xt1": 0.25,
        "qd1": pytest.approx(0.8861),
        "qd2": 1.0,
        "qd3": 1.0,
        "qd4": 1.0,
    }


def test_aquabus_output_7_write_reproduces_the_hardware_bytes() -> None:
    """Hardware 2026-09-15: 9.02 % on aquaero output 7 through the aquaero; one SET,
    byte-identical to the control report read back afterwards, no save report."""
    binding = DeviceBinding(kind=AQUAERO, pwm_map={"qd3": 7})
    adapter, device, _clock = _aquabus(binding)
    adapter.read()
    adapter.apply(_cmd(qd3=0.0902))
    assert [op.what for op in device.ops] == ["get", "set"]
    assert device.sets()[0].data == fixture_bytes("aquaero-ctrl-aquabus-after-fan7-write.bin")
    device.emit()
    assert adapter.read().pwm == {"qd3": pytest.approx(0.0902)}


def test_all_eight_aquaero_outputs_go_out_in_one_set() -> None:
    binding = DeviceBinding(kind=AQUAERO, pwm_map={f"o{n}": n for n in range(1, 9)})
    adapter, device, _clock = _aquabus_all_configured(binding)
    adapter.read()
    duties = {f"o{n}": n / 10 for n in range(1, 9)}
    adapter.apply(_cmd(**duties))
    assert len(device.sets()) == 1
    assert device.last_set_duties() == [n * 1000 for n in range(1, 9)]
    assert all(channel_holds(AQUAERO, device.ctrl, k, (k + 1) * 1000) for k in range(8))


def test_aquabus_modes_get_no_warning_and_a_block_without_a_source_is_left_out(caplog) -> None:
    """Blocks 5-7 read mode 0x0500 (not interpreted): no "not PWM" warning. Block 8 has
    no control source (0xFFFF): the daemon will not write that block blind, so it leaves
    *that channel* out of the write -- the other three go out in the same SET, block 8
    keeps every field the controller held, and the channel is logged once and reported
    (PROJECT.md section 8 item 89)."""
    binding = DeviceBinding(kind=AQUAERO, pwm_map={f"qd{n}": n + 4 for n in range(1, 5)})
    adapter, device, _clock = _aquabus(binding)
    with caplog.at_level("WARNING", logger=LOGGER):
        adapter.apply(_cmd(qd1=0.5, qd2=0.5, qd3=0.5, qd4=0.5))
    assert not [m for m in _messages(caplog, "WARNING") if "not PWM" in m]
    assert len(device.sets()) == 1 and device.saves() == []
    assert [channel_holds(AQUAERO, device.ctrl, k, 5000) for k in (4, 5, 6)] == [True] * 3
    state = channel_state(AQUAERO, device.ctrl, 7)  # untouched: no source, no duty written
    assert (state.duty, state.source, state.on_duty) == (10000, SOURCE_UNCONFIGURED, False)
    errors = [m for m in _messages(caplog, "ERROR") if "no control source" in m]
    assert len(errors) == 1 and "pwm8 (qd4)" in errors[0]
    assert adapter.unconfigured_channels == ("qd4",)
    assert any("not commanded" in p for p in adapter.device_health()["problems"])
    # and it stays out, without a second log line, while the block reads that way
    with caplog.at_level("WARNING", logger=LOGGER):
        adapter.apply(_cmd(qd1=0.6, qd2=0.6, qd3=0.6, qd4=0.6))
    assert len(device.sets()) == 2
    assert channel_state(AQUAERO, device.ctrl, 7).source == SOURCE_UNCONFIGURED
    assert len([m for m in _messages(caplog, "ERROR") if "no control source" in m]) == 1


def test_a_block_without_a_source_never_stops_the_heartbeat_or_the_other_channels() -> None:
    """One unconfigured block must not take a whole controller to the aquaero's watchdog
    fallback: ``apply()`` does not raise, the seven configured outputs are written, and
    the software-sensor heartbeat that keeps the watchdog quiet still goes out
    (PROJECT.md section 8 item 89)."""
    binding = DeviceBinding(
        kind=AQUAERO,
        pwm_map={f"o{n}": n for n in range(1, 9)},
        timing=_timing(AQUAERO, **HEARTBEAT),
    )
    adapter, device, _clock = _aquabus(binding)
    adapter.apply(_cmd(**{f"o{n}": n / 10 for n in range(1, 9)}))
    assert len(device.sets()) == 1 and len(device.writes()) == 1  # duties, then heartbeat
    assert device.last_set_duties()[:7] == [n * 1000 for n in range(1, 8)]
    assert channel_state(AQUAERO, device.ctrl, 7).source == SOURCE_UNCONFIGURED
    assert adapter.unconfigured_channels == ("o8",)


def test_a_block_without_a_source_that_is_not_commanded_is_no_problem() -> None:
    """Only a *commanded* channel is left out: the same controller with its aquabus
    outputs 5-7 bound and block 8 left out of the config works normally."""
    binding = DeviceBinding(kind=AQUAERO, pwm_map={f"qd{n}": n + 4 for n in range(1, 4)})
    adapter, device, _clock = _aquabus(binding)
    adapter.apply(_cmd(qd1=0.5, qd2=0.5, qd3=0.5))
    assert len(device.sets()) == 1
    assert adapter.unconfigured_channels == ()
    assert adapter.device_health()["problems"] == []


def test_control_snapshot_shows_a_block_without_a_source_instead_of_raising() -> None:
    """``control_snapshot()`` is the commissioning tool whose job is to show the owner
    the block that is wrong (item 88): it returns that report, and the channel is in
    ``unconfigured_channels`` for the caller to print."""
    binding = DeviceBinding(kind=AQUAERO, pwm_map={f"qd{n}": n + 4 for n in range(1, 5)})
    adapter, device, _clock = _aquabus(binding)
    snapshot = adapter.control_snapshot()
    assert snapshot == bytes(device.ctrl) and device.sets() == []
    assert adapter.unconfigured_channels == ("qd4",)


def test_release_restores_and_reports_success_with_an_unconfigured_block(caplog) -> None:
    """A restore that went out is a success even when the controller still holds a block
    with no control source: the adopt of the restored report raises nothing (item 89)."""
    binding = DeviceBinding(kind=AQUAERO, pwm_map={f"qd{n}": n + 4 for n in range(1, 5)})
    adapter, device, _clock = _aquabus(binding)
    adapter.apply(_cmd(qd1=0.5, qd2=0.5, qd3=0.5, qd4=0.5))
    with caplog.at_level("INFO", logger=LOGGER):
        adapter.release()
    assert len(device.sets()) == 2
    assert any("restored the captured control settings" in m for m in _messages(caplog, "INFO"))


def test_an_output_without_a_device_on_aquabus_faults_only_its_own_channel(caplog) -> None:
    """Nothing on aquabus: fan blocks 5-8 read rpm 0xFFFF. That channel's rpm and duty are
    None, every other channel and every temperature still comes through, read() does not
    raise, and apply() writes every channel (PROJECT.md section 8 item 90)."""
    binding = DeviceBinding(
        kind=AQUAERO,
        pwm_map={"xt1": 1, "qd1": 5},
        fan_map={"xt1": 1, "qd1": 5},
        temp_map={"inlet": "temp6"},
    )
    adapter, device, _bus, clock, _ = _setup(binding)  # the plain aquaero fixture
    assert adapter.absent_channels == ()
    with caplog.at_level("INFO", logger=LOGGER):
        obs = adapter.read()
        assert obs.rpm == {"xt1": 0.0, "qd1": None}
        assert obs.pwm == {"xt1": pytest.approx(1.0), "qd1": None}
        assert obs.temps == {"inlet": pytest.approx(22.26)}
        assert adapter.absent_channels == ("qd1",)
        adapter.apply(_fault(Mode.FALLBACK, xt1=0.8, qd1=0.8))
        assert len(device.sets()) == 1
        assert channel_holds(AQUAERO, device.ctrl, 0, 10000)  # never below what it held
        assert channel_holds(AQUAERO, device.ctrl, 4, 8861)  # the empty slot's reported duty
        adapter.apply(_cmd(xt1=0.8, qd1=0.3))  # AUTO: written as commanded
        assert channel_holds(AQUAERO, device.ctrl, 4, 3000)
        # No duty evidence from an empty slot: no mismatch rewrite, no stuck channel.
        for _ in range(5):
            clock.advance(AQUAERO_T.duty_mismatch_s)
            device.emit()
            assert adapter.read().pwm["qd1"] is None
        assert adapter.control_report is not None and adapter.stuck_channels == ()
        # The Quadro is plugged into aquabus again.
        device.status_template = fixture_bytes("aquaero-status-aquabus-fan7-100.bin")
        device.emit()
        obs = adapter.read()
    assert obs.rpm == {"xt1": 349.0, "qd1": 0.0} and obs.pwm["qd1"] == pytest.approx(0.3)
    assert adapter.absent_channels == ()
    absent_error, bus_error = _messages(caplog, "ERROR")
    assert absent_error.startswith("aquaero: no device behind pwm5 (qd1), fan5 (qd1)")
    assert "commands no reachable output" not in absent_error  # xt1 is still commanded
    # The bus device itself is gone for longer than bus_absent_s: item 92's own signal,
    # which item 90's per-channel line does not carry.
    assert bus_error.startswith("aquaero: no device has answered on its aquabus")
    assert [m for m in _messages(caplog, "INFO") if "again" in m] == [
        "aquaero: a device is behind pwm5 (qd1), fan5 (qd1) again",
        "aquaero: a device answers on aquabus again "
        "(nothing of this controller reads its aquabus temperature slots)",
    ]


def test_every_commanded_output_absent_is_named_once_and_still_reads(caplog) -> None:
    """The other half of item 90: every commanded output on the missing aquabus device.
    The controller is still not unavailable -- its temperatures are what makes the fans
    of the other controllers ramp -- but the error says it commands nothing reachable."""
    binding = DeviceBinding(
        kind=AQUAERO,
        pwm_map={"qd1": 5, "qd2": 6},
        fan_map={"qd1": 5},
        temp_map={"inlet": "temp6"},
    )
    adapter, device, _bus, clock, _ = _setup(binding)
    with caplog.at_level("INFO", logger=LOGGER):
        for _ in range(3):  # the state change is logged once, not once per tick
            clock.advance(1.0)
            device.emit()
            obs = adapter.read()
    assert obs.pwm == {"qd1": None, "qd2": None} and obs.rpm == {"qd1": None}
    assert obs.temps == {"inlet": pytest.approx(22.26)}
    assert adapter.absent_channels == ("qd1", "qd2")
    (error,) = _messages(caplog, "ERROR")
    assert "no device behind pwm5 (qd1), pwm6 (qd2), fan5 (qd1)" in error
    assert "commands no reachable output now" in error


def test_a_bound_aquabus_tachometer_without_a_device_faults_the_rpm_only() -> None:
    binding = DeviceBinding(kind=AQUAERO, pwm_map={"xt1": 1}, fan_map={"xt1": 7})
    adapter, device, _bus, _clock, _ = _setup(binding)
    obs = adapter.read()
    assert obs.rpm == {"xt1": None} and obs.pwm == {"xt1": pytest.approx(1.0)}
    assert adapter.absent_channels == ("xt1",)
    adapter.apply(_cmd(xt1=0.5))  # the output itself is the aquaero's own
    assert len(device.sets()) == 1


def test_rpm_0xffff_on_the_aquaeros_own_outputs_is_no_absent_device() -> None:
    """Only the aquabus slots 5-8 can be empty; the check never covers outputs 1-4."""
    binding = DeviceBinding(kind=AQUAERO, pwm_map={"xt1": 1}, fan_map={"xt1": 1})
    status = bytearray(fixture_bytes("aquaero-status-aquabus-fan7-100.bin"))
    speed = AQUAERO.fan_blocks[0] + AQUAERO.fan_layout.speed
    status[speed : speed + 2] = b"\xff\xff"
    adapter, _device, _bus, _clock, _ = _setup(binding, status_template=bytes(status))
    assert adapter.read().rpm == {"xt1": 65535.0}
    assert adapter.absent_channels == ()


def test_apply_before_any_status_report_cannot_know_about_aquabus() -> None:
    binding = DeviceBinding(kind=AQUAERO, pwm_map={"qd1": 5})
    adapter, device, _bus, _clock, _ = _setup(binding, report_delay_s=1e9)
    adapter.apply(_cmd(qd1=0.5))
    assert len(device.sets()) == 1


# --- the device that leaves aquabus (PROJECT.md section 8 item 92) --------------------------


def _bus_device_gone(template: bytes) -> bytes:
    """The status report item 92 saw: every aquabus fan block reads rpm 0xFFFF while the
    aquabus temperature slots still hold the value they read before the device left.

    Built from the capture rather than captured: on 2026-09-15 the aquaero kept reporting
    24.12 degC in ``bus2`` for over an hour after the Quadro was unplugged from aquabus,
    and no capture of that hour exists. Everything else in the report is the live one.
    """
    status = bytearray(template)
    for number in AQUAERO.aquabus_outputs:
        speed = AQUAERO.fan_blocks[number - 1] + AQUAERO.fan_layout.speed
        status[speed : speed + 2] = b"\xff\xff"
    return bytes(status)


def _bus_binding() -> DeviceBinding:
    """An aquaero with the Quadro's output 3 commanded and the Quadro's own thermistor
    (aquabus slot 2) bound next to one of the aquaero's own."""
    return DeviceBinding(
        kind=AQUAERO,
        pwm_map={"xt1": 1, "qd3": 7},
        fan_map={"qd3": 7},
        temp_map={"inlet": "temp6", "quadro_air": "bus2"},
    )


def test_a_bus_device_that_leaves_makes_its_temperature_slots_missing_not_frozen(caplog) -> None:
    """Item 92 end to end on one controller. While the Quadro answers, its thermistor in
    ``bus2`` is an ordinary reading. From the first report in which every aquabus block
    reads rpm 0xFFFF it reads as **missing**, although the aquaero keeps serving its last
    value there -- that is the whole point: a frozen number would reach the solver as a
    measurement. The *report* waits for ``bus_absent_s`` of such reports, so one skipped
    aquabus poll or a Quadro re-enumerating is not announced as a lost device; then one
    error line, one entry in ``problems``, and both clear when the device answers again.
    """
    adapter, device, clock = _aquabus_all_configured(_bus_binding())
    obs = adapter.read()
    assert obs.temps["quadro_air"] == pytest.approx(23.68)
    assert adapter.bus_device["present"] is True and adapter.bus_device["seen"] is True
    assert adapter.device_health()["problems"] == []

    with caplog.at_level("INFO", logger=LOGGER):
        device.status_template = _bus_device_gone(device.status_template)
        clock.advance(1.0)
        device.emit()
        obs = adapter.read()
        # Immediately missing, and only that input: the aquaero's own sensor is untouched.
        assert obs.temps["quadro_air"] is None
        assert obs.temps["inlet"] is not None
        assert obs.pwm["xt1"] is not None and obs.pwm["qd3"] is None
        health = adapter.device_health()
        assert health["aquabus"]["present"] is False and health["aquabus"]["lost"] is False
        assert health["aquabus"]["temps_missing"] == ["quadro_air"]
        assert [m for m in _messages(caplog, "ERROR") if "item 92" in m] == []
        # ... until it has read that way for bus_absent_s of live reports.
        for _ in range(int(AQUAERO_T.bus_absent_s) + 2):
            clock.advance(1.0)
            device.emit()
            assert adapter.read().temps["quadro_air"] is None
        health = adapter.device_health()
        assert health["aquabus"]["lost"] is True and health["aquabus"]["state"] == "lost"
        assert health["aquabus"]["absent_s"] >= AQUAERO_T.bus_absent_s
        assert "refresh_reports" not in health["aquabus"]  # withdrawn, item 115
        (problem,) = [p for p in health["problems"] if "item 92" in p]
        assert problem == (
            "aquaero: the device on its aquabus stopped answering; the temperatures it fed "
            "are reported as missing rather than as the frozen value of their slots: "
            "quadro_air (PROJECT.md section 8 item 92)"
        )
        # The Quadro is back on the bus.
        device.status_template = fixture_bytes("aquaero-status-aquabus-block7-no-power.bin")
        clock.advance(1.0)
        device.emit()
        assert adapter.read().temps["quadro_air"] == pytest.approx(23.68)
    assert adapter.bus_device["lost"] is False and adapter.bus_device["present"] is True
    assert [p for p in adapter.device_health()["problems"] if "item 92" in p] == []
    bus_errors = [m for m in _messages(caplog, "ERROR") if "item 92" in m]
    assert len(bus_errors) == 1  # once per state change, not once per tick
    assert "the device on its aquabus stopped answering" in bus_errors[0]
    assert "reported as missing from here on: quadro_air" in bus_errors[0]
    assert "hold or raise" in bus_errors[0]
    assert [m for m in _messages(caplog, "INFO") if "answers on aquabus again" in m] == [
        "aquaero: a device answers on aquabus again; its temperature slots are readings once more"
    ]


def test_a_report_whose_electrical_sample_missed_is_never_read_as_a_missing_device() -> None:
    """The line between item 92 and item 115. An aquabus block's electrical fields hold
    one sample taken inside the output's PWM cycle, so at the captures' 20 % duty three
    reports in four read 0 mA and the aquaero's own rail with the fan turning; the two
    captured reports are one such pair. Over four times ``bus_absent_s`` of them nothing
    is ever judged absent, because presence is read from the speed field, which every
    report carries -- not from a voltage or a current (item 116)."""
    adapter, device, clock = _aquabus_all_configured(_bus_binding())
    measuring = fixture_bytes("aquaero-status-aquabus-block7-power.bin")
    substituted = fixture_bytes("aquaero-status-aquabus-block7-no-power.bin")
    for i in range(4 * int(AQUAERO_T.bus_absent_s)):
        device.status_template = measuring if i % 4 == 0 else substituted
        clock.advance(1.0)
        device.emit()
        obs = adapter.read()
        assert obs.temps["quadro_air"] is not None
        assert adapter.bus_device["present"] is True
    assert adapter.bus_device == {
        "state": "present",
        "present": True,
        "seen": True,
        "absent_s": None,
        "lost": False,
        "temps_missing": [],
    }
    assert adapter.device_health()["problems"] == []


def test_one_transient_report_without_the_bus_device_reports_no_loss(caplog) -> None:
    """A single ``0xFFFF`` costs the aquabus temperatures one tick of ``None`` -- the safe
    direction, and the same price item 90 pays for rpm and duty -- and is reported as
    nothing: ``bus_absent_s`` is exactly what keeps a blip out of the health payload."""
    adapter, device, clock = _aquabus_all_configured(_bus_binding())
    live = device.status_template
    with caplog.at_level("INFO", logger=LOGGER):
        adapter.read()
        device.status_template = _bus_device_gone(live)
        clock.advance(1.0)
        device.emit()
        assert adapter.read().temps["quadro_air"] is None
        device.status_template = live
        for _ in range(3):
            clock.advance(1.0)
            device.emit()
            assert adapter.read().temps["quadro_air"] == pytest.approx(23.68)
    assert adapter.bus_device["lost"] is False and adapter.bus_device["absent_s"] is None
    assert adapter.device_health()["problems"] == []
    assert [m for m in _messages(caplog, "ERROR") if "item 92" in m] == []


def test_reopening_the_node_does_not_restart_the_clock_on_an_empty_bus(caplog) -> None:
    """A transport reopen is the daemon's business, not the bus's. The bus state and
    ``absent_s`` are one pair: reporting ``lost`` next to an ``absent_s`` counting up
    from zero would read as a device that had just gone, or as a flapping rule, in
    exactly the case an operator consults it. So the empty run keeps being measured
    from the first report that showed the bus empty, across the reopen, and the loss is
    not announced a second time for the same absence."""
    adapter, device, clock = _aquabus_all_configured(_bus_binding())
    adapter.read()
    device.status_template = _bus_device_gone(device.status_template)
    with caplog.at_level("INFO", logger=LOGGER):
        for _ in range(int(AQUAERO_T.bus_absent_s) + 2):
            clock.advance(1.0)
            device.emit()
            adapter.read()
        assert adapter.bus_device["state"] == "lost"
        before = adapter.bus_device["absent_s"]
        adapter.close()
        clock.advance(30.0)
        device.emit()
        obs = adapter.read()  # a new open, with the bus still empty
    assert obs.temps["quadro_air"] is None
    health = adapter.device_health()["aquabus"]
    assert health["state"] == "lost" and health["lost"] is True
    assert health["absent_s"] == pytest.approx(before + 30.0 + 1.0, abs=1.0)
    assert len([m for m in _messages(caplog, "ERROR") if "item 92" in m]) == 1


def test_a_controller_with_nothing_bound_on_aquabus_reports_the_state_but_no_problem(
    caplog,
) -> None:
    """The aquaero's own outputs and thermistors, with nothing of this daemon's on the
    bus: an empty aquabus is that controller's normal state and no problem of the
    daemon's. The state is still published, so an owner can see what the bus looks
    like -- as ``never_seen``, never as ``lost``: a bus no device was ever on has
    lost nothing, and ``lost`` is the field a one-line consumer would show.
    """
    binding = DeviceBinding(kind=AQUAERO, pwm_map={"xt1": 1}, temp_map={"inlet": "temp6"})
    adapter, device, _bus, clock, _ = _setup(binding)  # the plain aquaero: empty aquabus
    with caplog.at_level("INFO", logger=LOGGER):
        for _ in range(int(AQUAERO_T.bus_absent_s) + 2):
            clock.advance(1.0)
            device.emit()
            adapter.read()
    health = adapter.device_health()
    assert health["aquabus"]["present"] is False and health["aquabus"]["seen"] is False
    assert health["aquabus"]["state"] == "never_seen"
    assert health["aquabus"]["lost"] is False and health["aquabus"]["temps_missing"] == []
    assert health["problems"] == []
    assert [m for m in _messages(caplog, "ERROR") if "item 92" in m] == []


def test_a_bus_that_was_never_there_is_reported_as_that_and_not_as_a_departure(caplog) -> None:
    """The other half: the daemon starts with the Quadro already off the bus and an
    aquabus output commanded. It says so -- and says that none has answered since it
    started reading, which is what the owner needs to tell "unplugged just now" from
    "never came up"."""
    binding = DeviceBinding(kind=AQUAERO, pwm_map={"qd1": 5}, temp_map={"inlet": "temp6"})
    adapter, device, _bus, clock, _ = _setup(binding)
    with caplog.at_level("INFO", logger=LOGGER):
        for _ in range(int(AQUAERO_T.bus_absent_s) + 2):
            clock.advance(1.0)
            device.emit()
            adapter.read()
    health = adapter.device_health()
    assert health["aquabus"]["seen"] is False and health["aquabus"]["lost"] is False
    assert health["aquabus"]["state"] == "never_seen"  # nothing was lost here
    assert [p for p in health["problems"] if "item 92" in p] == [
        "aquaero: no device answers on its aquabus, and none has since this daemon "
        "started reading it (PROJECT.md section 8 item 92)"
    ]
    (error,) = [m for m in _messages(caplog, "ERROR") if "item 92" in m]
    assert "no device has answered on its aquabus, and none has since this daemon" in error
    assert "bus_absent_s = 10" in error


def test_the_quadro_itself_never_judges_an_aquabus() -> None:
    """A kind with no aquabus outputs cannot see a bus at all: it publishes the state as
    unknown and never reports a loss, whatever its own outputs read."""
    adapter, _device, _bus, _clock, _ = _setup(_quadro_binding())
    adapter.read()
    assert adapter.bus_device == {
        "state": "unknown",
        "present": None,
        "seen": False,
        "absent_s": None,
        "lost": False,
        "temps_missing": [],
    }
    assert adapter.device_health()["problems"] == []


def test_the_stuck_error_carries_the_hint(caplog) -> None:
    dt = QUADRO_T.duty_mismatch_s / 2
    adapter, device, clock = _commanded_quadro(ctrl_refresh_s=0)
    calls = []

    def hint() -> str:
        calls.append(1)
        return "the Quadro is probably on aquabus"

    adapter.stuck_hint = hint
    device.ignores = {0: DUTY_MAX}
    with caplog.at_level("WARNING", logger=LOGGER):
        for _ in range(12):
            _tick(adapter, device, clock, dt, qd1=0.5, qd2=0.5)
    (error,) = _messages(caplog, "ERROR")
    assert error.endswith("; the Quadro is probably on aquabus") and len(calls) == 1
    assert not any("probably" in m for m in _messages(caplog, "WARNING"))


# --- DeviceBinding / timing model -----------------------------------------------------------


def test_binding_rejects_numbers_outside_the_kind() -> None:
    with pytest.raises(ValueError, match="pwm1..pwm4"):
        DeviceBinding(kind=QUADRO, pwm_map={"a": 5})
    with pytest.raises(ValueError, match="fan1..fan4"):
        DeviceBinding(kind=QUADRO, pwm_map={"a": 1}, fan_map={"a": 5})
    with pytest.raises(ValueError, match="pwm1..pwm8"):
        DeviceBinding(kind=AQUAERO, pwm_map={"a": 9})
    DeviceBinding(kind=AQUAERO, pwm_map={"a": 8}, fan_map={"a": 8})
    with pytest.raises(ValueError, match="temperature inputs temp1..temp4, not"):
        DeviceBinding(kind=QUADRO, pwm_map={}, temp_map={"t": "bus1"})
    with pytest.raises(ValueError, match="share one"):
        DeviceBinding(kind=AQUAERO, pwm_map={}, temp_map={"t": "bus1", "u": "bus1"})
    # A busN is a reading only while a device answers on aquabus, which is judged from
    # the aquabus fan blocks: without one of them bound the binding is refused (item 92).
    with pytest.raises(ValueError, match="no aquabus output"):
        DeviceBinding(kind=AQUAERO, pwm_map={}, temp_map={"t": "bus1", "u": "virt4"})
    DeviceBinding(
        kind=AQUAERO,
        pwm_map={"qd1": 5},
        temp_map={"t": "bus1", "u": "virt4"},
    )
    DeviceBinding(kind=AQUAERO, pwm_map={}, temp_map={"u": "virt4"})
    # A softN software sensor is never a bindable input (item 113), busN aside.
    with pytest.raises(ValueError, match="software sensor"):
        DeviceBinding(kind=AQUAERO, pwm_map={"qd1": 5}, temp_map={"v": "soft8"})
    with pytest.raises(ValueError, match="share one"):
        DeviceBinding(kind=AQUAERO, pwm_map={"a": 1, "b": 1})
    with pytest.raises(ValueError, match="not pwm_map channels"):
        DeviceBinding(kind=AQUAERO, pwm_map={"a": 1}, fan_map={"b": 1})


def test_timing_defaults_are_the_documented_ones() -> None:
    assert DeviceBinding(kind=QUADRO, pwm_map={"a": 1}).timing == QUADRO_T
    assert (AQUAERO_T.ctrl_gap_ms, QUADRO_T.ctrl_gap_ms) == (100.0, 0.0)
    t = QUADRO_T
    assert (t.status_max_age_s, t.ctrl_retries, t.ctrl_budget_s, t.ctrl_refresh_s) == (
        3.0,
        1,
        5.0,
        60.0,
    )
    assert (t.duty_mismatch_tolerance, t.duty_mismatch_s) == (100, 5.0)
    assert (t.write_min_interval_s, t.write_deadband) == (0.0, 0)


# --- live device ----------------------------------------------------------------------------


def _service_active() -> bool:
    systemctl = shutil.which("systemctl")
    if systemctl is None:
        return False
    try:
        result = subprocess.run(
            [systemctl, "is-active", "--quiet", "aqua-bridge"], check=False, timeout=10
        )
    except (OSError, subprocess.SubprocessError):
        return False
    return result.returncode == 0


@pytest.mark.hardware
def test_live_read_and_reapply_what_the_device_holds(aquaero_hidraw) -> None:
    """Live USB test: skipped unless conftest finds an aquaero hidraw node, and while
    the aqua-bridge service runs (two writers).

    Reads a status report, then reads the control report right before re-applying
    the duty of every channel that already follows its preset (so nothing changes:
    no SET is sent). A channel on a firmware controller is left out: commanding it
    would reassign it.
    """
    from aqua_bridge.hw.hidraw import HidrawTransport

    if _service_active():
        pytest.skip("the aqua-bridge service is active; stop it before the live test")

    own = [n for n in range(1, AQUAERO.pwm_count + 1) if n not in AQUAERO.aquabus_outputs]
    reader = AquacomputerAdapter(
        DeviceBinding(
            kind=AQUAERO,
            pwm_map={f"ch{n}": n for n in own},  # 5-8 only with a device on aquabus
            temp_map={f"t_{name}": name for name in AQUAERO.temp_names},
        )
    )
    try:
        obs = reader.read()
    finally:
        reader.close()
    assert all(v is None or -40.0 < v < 125.0 for v in obs.temps.values())
    assert all(v is not None and 0.0 <= v <= 1.0 for v in obs.pwm.values())
    status = reader.last_status
    assert status is not None
    assert all(0 <= fan.rpm < 20000 for fan in status.fans[: len(own)])

    sent: list[bytes] = []

    def opener(kind, serial):
        real = HidrawTransport.open(aquaero_hidraw)
        original = real.set_feature

        def recording(data: bytes) -> None:
            sent.append(bytes(data))
            original(data)

        real.set_feature = recording  # type: ignore[method-assign]
        return real

    transport = HidrawTransport.open(aquaero_hidraw)
    try:
        ctrl = transport.get_feature(AQUAERO.ctrl_report_id, AQUAERO.ctrl_size)
    finally:
        transport.close()
    held = {
        f"ch{k + 1}": control_duty(AQUAERO, ctrl, k)
        for k in range(len(own))
        if channel_state(AQUAERO, ctrl, k).on_duty
    }
    if not held:
        pytest.skip("no aquaero channel follows its preset; nothing safe to re-apply")
    writer = AquacomputerAdapter(
        DeviceBinding(kind=AQUAERO, pwm_map={ch: int(ch[2:]) for ch in held}), opener=opener
    )
    try:
        writer.apply(MpcCommand(pwm={ch: d / DUTY_MAX for ch, d in held.items()}, mode=Mode.AUTO))
    finally:
        writer.close()
    assert sent == []
