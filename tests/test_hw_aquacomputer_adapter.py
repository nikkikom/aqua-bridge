"""Tests for aqua_bridge.hw.aquacomputer_adapter against fake controllers, plus one live test.

PROJECT.md section 3 (Track B) / section 4.7 / section 2 ("USB spike results").
"""

from __future__ import annotations

import errno
import math

import pytest

from aqua_bridge.hw.aquacomputer import (
    AQUAERO,
    DUTY_MAX,
    QUADRO,
    capture_channel,
    channel_holds,
    channel_state,
    control_duty,
    finalize_control_report,
    patch_duties,
)
from aqua_bridge.hw.aquacomputer_adapter import (
    AquacomputerAdapter,
    AquacomputerTiming,
    DeviceBinding,
    DeviceUnavailable,
)
from aqua_bridge.hw.hidraw import FeatureReportError
from aqua_bridge.model import Mode, MpcCommand
from aquacomputer_fakes import FakeBus, FakeClock, FakeController, FakeSleep, fixture_bytes

DEFAULTS = AquacomputerTiming()
GAP_S = DEFAULTS.ctrl_gap_ms / 1000.0


def _cmd(**pwm: float) -> MpcCommand:
    return MpcCommand(pwm=pwm, mode=Mode.AUTO)


def _aquaero_binding(**timing) -> DeviceBinding:
    return DeviceBinding(
        kind=AQUAERO,
        pwm_map={"xt1": 1, "xt2": 2},
        fan_map={"xt2": 2},
        temp_map={"inlet": 6, "virtual": 9, "open": 1},
        timing=AquacomputerTiming(**timing),
    )


def _quadro_binding(**timing) -> DeviceBinding:
    return DeviceBinding(
        kind=QUADRO,
        pwm_map={"qd1": 1, "qd3": 3},
        fan_map={"qd3": 3},
        temp_map={"air": 2},
        timing=AquacomputerTiming(**timing),
    )


def _setup(binding: DeviceBinding, **controller):
    clock = FakeClock()
    sleep = FakeSleep(clock)
    device = FakeController(binding.kind, clock, **controller)
    bus = FakeBus(device)
    adapter = AquacomputerAdapter(binding, clock=clock, sleep=sleep, opener=bus)
    return adapter, device, bus, clock, sleep


# --- read ---------------------------------------------------------------------------------


def test_read_maps_the_aquaero_status_report() -> None:
    adapter, _device, _bus, clock, _ = _setup(_aquaero_binding())
    obs = adapter.read()
    assert obs.temps == {
        "inlet": pytest.approx(22.26),
        "virtual": pytest.approx(40.0),
        "open": None,
    }
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
    adapter.read()
    clock.advance(DEFAULTS.status_max_age_s * 0.9)
    adapter.read()  # still young enough without a new report
    clock.advance(DEFAULTS.status_max_age_s * 0.2)
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


def test_open_without_a_status_report_in_time_closes_and_raises() -> None:
    adapter, device, _bus, clock, _ = _setup(
        _aquaero_binding(), report_delay_s=DEFAULTS.status_max_age_s * 2
    )
    start = clock()
    with pytest.raises(DeviceUnavailable, match="no status report within"):
        adapter.read()
    assert clock() - start == pytest.approx(DEFAULTS.status_max_age_s)
    assert device.closed and not adapter.is_open


def test_absent_device_raises_device_unavailable() -> None:
    clock = FakeClock()
    adapter = AquacomputerAdapter(_aquaero_binding(), clock=clock, opener=FakeBus())
    with pytest.raises(DeviceUnavailable):
        adapter.read()
    with pytest.raises(DeviceUnavailable):
        adapter.apply(_cmd(xt1=0.5, xt2=0.5))


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


# --- apply --------------------------------------------------------------------------------


def test_changed_command_is_one_set_plus_one_secondary_report_with_the_driver_bytes() -> None:
    binding = DeviceBinding(kind=AQUAERO, pwm_map={"xt2": 2})
    adapter, device, _bus, _clock, _ = _setup(binding)
    adapter.apply(_cmd(xt2=0.1412))
    assert [op.what for op in device.ops] == ["get", "set", "secondary"]
    assert device.sets()[0].data == fixture_bytes("aquaero-ctrl-after-writes.bin")
    assert device.secondaries()[0].data == AQUAERO.secondary_report


def test_quadro_write_matches_the_driver_bytes_including_the_checksum() -> None:
    binding = DeviceBinding(kind=QUADRO, pwm_map={"qd3": 3})
    adapter, device, _bus, _clock, _ = _setup(binding)
    adapter.apply(_cmd(qd3=0.0902))
    assert [op.what for op in device.ops] == ["get", "set", "secondary"]
    assert device.sets()[0].data == fixture_bytes("quadro-ctrl-after-writes.bin")
    assert device.secondaries()[0].data == QUADRO.secondary_report


def test_several_changed_channels_go_out_in_one_set() -> None:
    binding = DeviceBinding(kind=QUADRO, pwm_map={f"qd{n}": n for n in range(1, 5)})
    adapter, device, _bus, _clock, _ = _setup(binding)
    adapter.apply(_cmd(qd1=0.3, qd2=0.4, qd3=1.0, qd4=0.25))
    assert len(device.sets()) == 1 and len(device.secondaries()) == 1
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


def test_gap_after_a_set_is_honoured_before_the_next_control_operation() -> None:
    binding = DeviceBinding(kind=QUADRO, pwm_map={"qd1": 1})
    adapter, device, _bus, clock, sleep = _setup(binding)
    adapter.apply(_cmd(qd1=0.5))
    assert sleep.calls == []  # no SET before: the first GET and SET do not wait
    clock.advance(GAP_S / 4)
    adapter.apply(_cmd(qd1=0.6))
    assert sleep.calls == [pytest.approx(GAP_S * 3 / 4)]
    first, second = device.sets()
    assert second.t - first.t >= GAP_S - 1e-9
    clock.advance(GAP_S * 2)
    adapter.apply(_cmd(qd1=0.7))
    assert len(sleep.calls) == 1  # enough time passed: no wait


def test_get_right_after_a_set_waits_for_the_gap() -> None:
    binding = DeviceBinding(kind=QUADRO, pwm_map={"qd1": 1}, timing=AquacomputerTiming())
    adapter, device, _bus, _clock, sleep = _setup(binding)
    adapter.apply(_cmd(qd1=0.5))
    device.failures = [FeatureReportError("EPIPE", errno.EPIPE)]
    adapter.apply(_cmd(qd1=0.6))  # SET fails; the retry GET must wait
    set_t = device.ops[3].t
    retry_get = device.ops[4]
    assert retry_get.what == "get" and retry_get.t - set_t >= GAP_S - 1e-9
    assert sleep.calls


def test_failed_get_is_retried_from_a_fresh_get() -> None:
    binding = DeviceBinding(kind=QUADRO, pwm_map={"qd1": 1})
    adapter, device, _bus, _clock, _ = _setup(binding)
    device.failures = [FeatureReportError("ENODATA", errno.ENODATA)]
    adapter.apply(_cmd(qd1=0.5))
    assert [op.what for op in device.ops] == ["get", "get", "set", "secondary"]


def test_corrupt_control_report_is_retried() -> None:
    binding = DeviceBinding(kind=QUADRO, pwm_map={"qd1": 1})
    adapter, device, _bus, _clock, _ = _setup(binding)
    good = bytes(device.ctrl)
    bad = bytearray(good)
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
    binding = DeviceBinding(
        kind=QUADRO, pwm_map={"qd1": 1}, timing=AquacomputerTiming(ctrl_retries=2)
    )
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
    binding = DeviceBinding(
        kind=QUADRO, pwm_map={"qd1": 1}, timing=AquacomputerTiming(ctrl_retries=0)
    )
    adapter, device, _bus, _clock, _ = _setup(binding)
    device.failures = [FeatureReportError("EPIPE", errno.EPIPE)]
    with pytest.raises(DeviceUnavailable):
        adapter.apply(_cmd(qd1=0.5))
    assert len(device.gets()) == 1


def test_failed_secondary_report_rewrites_on_the_retry() -> None:
    """The cache only counts as written once SET and secondary report both went out."""
    binding = DeviceBinding(kind=QUADRO, pwm_map={"qd1": 1, "qd2": 2})
    adapter, device, _bus, _clock, _ = _setup(binding)
    original = device.set_feature

    def fail_secondary_once(data: bytes) -> None:
        if data[0] != QUADRO.ctrl_report_id and not fail_secondary_once.done:
            fail_secondary_once.done = True
            device.ops.append(type(device.ops[0])("secondary", device.clock(), bytes(data)))
            raise FeatureReportError("EPIPE", errno.EPIPE)
        original(data)

    fail_secondary_once.done = False
    device.set_feature = fail_secondary_once  # type: ignore[method-assign]
    adapter.apply(_cmd(qd1=0.5, qd2=1.0))
    assert [op.what for op in device.ops] == ["get", "set", "secondary", "get", "set", "secondary"]
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


class _RawCommand:
    """Duck-typed command that skips MpcCommand's own validation."""

    def __init__(self, pwm) -> None:
        self.pwm = pwm


# --- keeping the cache honest ---------------------------------------------------------------


def _commanded_quadro(**timing):
    binding = DeviceBinding(
        kind=QUADRO, pwm_map={"qd1": 1, "qd2": 2}, timing=AquacomputerTiming(**timing)
    )
    adapter, device, bus, clock, sleep = _setup(binding)
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
    device.duty_override = {0: 5000 + DEFAULTS.duty_mismatch_tolerance + 1}
    _tick(adapter, device, clock, 1.0, qd1=0.5, qd2=0.5)  # mismatch starts
    _tick(adapter, device, clock, DEFAULTS.duty_mismatch_s, qd1=0.5, qd2=0.5)  # not longer
    assert len(device.gets()) == 1 and len(device.sets()) == 1
    with caplog.at_level("WARNING", logger="aqua_bridge.hw.aquacomputer"):
        clock.advance(0.5)
        device.emit()
        adapter.read()
    assert adapter.control_report is None
    assert "pwm1 (qd1)" in caplog.text and "50.00 %" in caplog.text and "51.01 %" in caplog.text
    adapter.apply(_cmd(qd1=0.5, qd2=0.5))
    assert len(device.gets()) == 2
    rewrite = device.sets()[-1]
    assert len(device.sets()) == 2  # every configured channel, though the report holds them
    assert (
        control_duty(QUADRO, rewrite.data, 0) == 5000
        and control_duty(QUADRO, rewrite.data, 1) == 5000
    )


def test_brief_duty_mismatch_does_not_rewrite() -> None:
    adapter, device, clock = _commanded_quadro()
    step = DEFAULTS.duty_mismatch_s * 0.4
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
    device.duty_override = {0: 5000 - DEFAULTS.duty_mismatch_tolerance}
    for _ in range(10):
        _tick(adapter, device, clock, DEFAULTS.duty_mismatch_s, qd1=0.5, qd2=0.5)
    assert len(device.gets()) == 1 and len(device.sets()) == 1


def test_duty_mismatch_timer_starts_at_the_last_set() -> None:
    """A mismatch that began before a write only counts from that write, and a
    report from before the write is no evidence at all (possible only while that
    report is younger than status_max_age_s, hence the long one here)."""
    limit = DEFAULTS.duty_mismatch_s
    adapter, device, clock = _commanded_quadro(status_max_age_s=10 * limit)
    device.duty_override = {1: 9000}  # qd2 mismatches; the write below changes only qd1
    clock.advance(1.0)
    device.emit()
    adapter.read()  # mismatch starts
    clock.advance(limit * 0.7)
    adapter.apply(_cmd(qd1=0.6, qd2=0.5))  # a SET, no new report yet
    set_t = clock()
    clock.advance(limit * 0.7)
    adapter.read()  # only the report from before the SET: no evidence, however old
    assert adapter.control_report is not None
    device.emit()
    adapter.read()  # new report: 1.4 limits since the start, 0.7 since the SET
    assert adapter.control_report is not None
    clock.advance(set_t + limit * 1.1 - clock())
    device.emit()
    adapter.read()
    assert adapter.control_report is None


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
    with caplog.at_level("WARNING", logger="aqua_bridge.hw.aquacomputer"):
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
    _tick(adapter, device, clock, DEFAULTS.ctrl_refresh_s / 2, qd1=0.5, qd2=0.5)
    assert len(device.gets()) == 1
    with caplog.at_level("WARNING", logger="aqua_bridge.hw.aquacomputer"):
        _tick(adapter, device, clock, DEFAULTS.ctrl_refresh_s / 2, qd1=0.5, qd2=0.5)
    assert len(device.gets()) == 2 and len(device.sets()) == 2
    assert "pwm2 (qd2)" in caplog.text
    assert control_duty(QUADRO, device.ctrl, 1) == 5000
    assert control_duty(QUADRO, device.ctrl, 3) == 4321  # the external change elsewhere stays


def test_periodic_refresh_without_drift_writes_nothing() -> None:
    adapter, device, clock = _commanded_quadro()
    _tick(adapter, device, clock, DEFAULTS.ctrl_refresh_s, qd1=0.5, qd2=0.5)
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
    assert device.ops[-1].what == "secondary" and device.ops[-2].what == "set"
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


# --- DeviceBinding / timing model -----------------------------------------------------------


def test_binding_rejects_numbers_outside_the_kind() -> None:
    with pytest.raises(ValueError, match="pwm1..pwm4"):
        DeviceBinding(kind=QUADRO, pwm_map={"a": 5})
    with pytest.raises(ValueError, match="fan1..fan5"):
        DeviceBinding(kind=QUADRO, pwm_map={"a": 1}, fan_map={"a": 6})
    DeviceBinding(kind=AQUAERO, pwm_map={"a": 1}, fan_map={"a": 6})
    with pytest.raises(ValueError, match="share one"):
        DeviceBinding(kind=AQUAERO, pwm_map={"a": 1, "b": 1})
    with pytest.raises(ValueError, match="not pwm_map channels"):
        DeviceBinding(kind=AQUAERO, pwm_map={"a": 1}, fan_map={"b": 1})


def test_timing_defaults_are_the_documented_ones() -> None:
    timing = AquacomputerTiming()
    assert (timing.status_max_age_s, timing.ctrl_gap_ms, timing.ctrl_retries) == (3.0, 200.0, 1)
    assert (timing.ctrl_refresh_s, timing.duty_mismatch_tolerance, timing.duty_mismatch_s) == (
        60.0,
        100,
        5.0,
    )


# --- live device ----------------------------------------------------------------------------


@pytest.mark.hardware
def test_live_read_and_reapply_what_the_device_holds(aquaero_hidraw) -> None:
    """Live USB test: skipped unless conftest finds an aquaero hidraw node.

    Reads a status report and re-applies the duty of every channel that
    already follows its preset (so nothing changes: no SET is sent), then
    checks the values are plausible. A channel on a firmware controller is
    left out: commanding it would reassign it.
    """
    from aqua_bridge.hw.hidraw import HidrawTransport

    transport = HidrawTransport.open(aquaero_hidraw)
    try:
        ctrl = transport.get_feature(AQUAERO.ctrl_report_id, AQUAERO.ctrl_size)
    finally:
        transport.close()
    held = {
        f"ch{k + 1}": control_duty(AQUAERO, ctrl, k)
        for k in range(AQUAERO.pwm_count)
        if channel_state(AQUAERO, ctrl, k).on_duty
    }

    sent: list[bytes] = []

    def opener(kind, serial):
        real = HidrawTransport.open(aquaero_hidraw)
        original = real.set_feature

        def recording(data: bytes) -> None:
            sent.append(bytes(data))
            original(data)

        real.set_feature = recording  # type: ignore[method-assign]
        return real

    binding = DeviceBinding(
        kind=AQUAERO,
        pwm_map={ch: int(ch[2:]) for ch in held} or {"ch1": 1},
        temp_map={f"t{n}": n for n in range(1, AQUAERO.temp_count + 1)},
    )
    adapter = AquacomputerAdapter(binding, opener=opener)
    try:
        obs = adapter.read()
        assert all(v is None or -40.0 < v < 125.0 for v in obs.temps.values())
        assert all(v is not None and 0.0 <= v <= 1.0 for v in obs.pwm.values())
        status = adapter.last_status
        assert status is not None and all(0 <= fan.rpm < 20000 for fan in status.fans)
        if not held:
            pytest.skip("no aquaero channel follows its preset; nothing safe to re-apply")
        adapter.apply(MpcCommand(pwm={ch: d / DUTY_MAX for ch, d in held.items()}, mode=Mode.AUTO))
        assert sent == []
    finally:
        adapter.close()
