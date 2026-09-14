"""Tests for aqua_bridge.hw.aquacomputer: report layouts against captured reports.

The fixtures in tests/fixtures/aquacomputer/ are real reports (serial bytes
zeroed) and the Linux driver's hwmon readings taken about a second later
(PROJECT.md section 2, "USB spike results"; section 4.7).
"""

from __future__ import annotations

import json
from pathlib import Path

import pytest
from hypothesis import given
from hypothesis import strategies as st

from aqua_bridge.hw.aquacomputer import (
    AQUAERO,
    DUTY_MAX,
    KINDS,
    QUADRO,
    DeviceKind,
    ReportError,
    capture_channel,
    channel_holds,
    channel_state,
    check_control_report,
    control_duty,
    crc16_usb,
    decode_status,
    finalize_control_report,
    is_status_report,
    kind_by_name,
    patch_duties,
    restore_channel,
)

FIXTURES = Path(__file__).resolve().parent / "fixtures" / "aquacomputer"


def _bin(name: str) -> bytes:
    return (FIXTURES / name).read_bytes()


def _readings(name: str) -> dict[str, int | None]:
    return json.loads((FIXTURES / name).read_text())


# --- status report ----------------------------------------------------------------------


@pytest.mark.parametrize(
    ("kind", "status", "readings"),
    [
        (AQUAERO, "aquaero-status.bin", "aquaero-driver-readings.json"),
        (QUADRO, "quadro-status.bin", "quadro-driver-readings.json"),
    ],
    ids=["aquaero", "quadro"],
)
def test_status_decodes_like_the_driver(kind: DeviceKind, status: str, readings: str) -> None:
    report = decode_status(kind, _bin(status))
    driver = _readings(readings)
    assert report.kind == kind.name
    assert len(report.temps) == kind.temp_count == 20
    for n in range(1, kind.temp_count + 1):
        expected = driver[f"temp{n}_input"]
        value = report.temp(n)
        if expected is None:
            assert value is None, f"temp{n}"
        else:
            # The driver had already taken the next report for one Quadro input.
            assert value is not None and abs(value * 1000 - expected) <= 20, f"temp{n}"
    for n in range(1, kind.fan_input_count + 1):
        assert report.fan_input(n) == driver[f"fan{n}_input"], f"fan{n}"
    for k, fan in enumerate(report.fans):
        assert fan.voltage_cv * 10 == driver[f"in{k}_input"], f"in{k}"
        assert fan.current_ma == driver[f"curr{k + 1}_input"], f"curr{k + 1}"
        assert fan.power_cw * 10000 == driver[f"power{k + 1}_input"], f"power{k + 1}"
    assert report.serial == "00000-00000"  # zeroed in the fixture


def test_aquaero_status_output_duty_field() -> None:
    """Captured with pwm2 commanded to 14.12 %, every other output at 100 %."""
    report = decode_status(AQUAERO, _bin("aquaero-status.bin"))
    assert [report.duty(n) for n in range(1, 5)] == [10000, 1412, 10000, 10000]
    assert report.power_cycles is None
    assert report.firmware > 0


def test_quadro_status_output_duty_and_power_cycles() -> None:
    """Captured with pwm3 commanded to 9.02 %; the Quadro's duty agrees with the
    driver's pwmN (its control report readback)."""
    report = decode_status(QUADRO, _bin("quadro-status.bin"))
    driver = _readings("quadro-driver-readings.json")
    assert [report.duty(n) for n in range(1, 5)] == [10000, 10000, 902, 10000]
    for n in range(1, 5):
        assert round(report.duty(n) * 255 / DUTY_MAX) == driver[f"pwm{n}"]
    assert isinstance(report.power_cycles, int) and report.power_cycles > 0


def test_negative_temperature_is_signed_not_655_degc() -> None:
    data = bytearray(_bin("aquaero-status.bin"))
    data[0x65:0x67] = (-250).to_bytes(2, "big", signed=True)
    assert decode_status(AQUAERO, bytes(data)).temp(1) == pytest.approx(-2.5)


@pytest.mark.parametrize("kind", [AQUAERO, QUADRO], ids=lambda k: k.name)
def test_status_with_wrong_length_or_id_is_rejected(kind: DeviceKind) -> None:
    good = _bin(f"{kind.name}-status.bin")
    assert is_status_report(kind, good)
    with pytest.raises(ReportError, match="bytes"):
        decode_status(kind, good[:-1])
    wrong_id = bytes([0x02]) + good[1:]
    assert not is_status_report(kind, wrong_id)
    with pytest.raises(ReportError, match="report id"):
        decode_status(kind, wrong_id)


def test_status_of_the_other_kind_is_rejected() -> None:
    assert not is_status_report(QUADRO, _bin("aquaero-status.bin"))
    with pytest.raises(ReportError):
        decode_status(QUADRO, _bin("aquaero-status.bin"))


def test_channel_numbers_out_of_range_raise() -> None:
    report = decode_status(QUADRO, _bin("quadro-status.bin"))
    with pytest.raises(IndexError):
        report.temp(21)
    with pytest.raises(IndexError):
        report.fan_input(6)
    with pytest.raises(IndexError):
        report.duty(0)


# --- control report ----------------------------------------------------------------------


@pytest.mark.parametrize("name", ["quadro-ctrl-firmware.bin", "quadro-ctrl-after-writes.bin"])
def test_crc16_usb_matches_both_quadro_reports(name: str) -> None:
    data = _bin(name)
    assert crc16_usb(data[1:-2]) == int.from_bytes(data[-2:], "big")
    check_control_report(QUADRO, data)


def test_crc16_usb_check_value() -> None:
    # The catalogue check value of CRC-16/USB over b"123456789".
    assert crc16_usb(b"123456789") == 0xB4C8


def test_quadro_control_report_with_a_bad_checksum_is_rejected() -> None:
    data = bytearray(_bin("quadro-ctrl-firmware.bin"))
    data[0x100] ^= 0x01
    with pytest.raises(ReportError, match="checksum"):
        check_control_report(QUADRO, bytes(data))


@pytest.mark.parametrize("kind", [AQUAERO, QUADRO], ids=lambda k: k.name)
def test_control_report_with_wrong_length_or_id_is_rejected(kind: DeviceKind) -> None:
    good = _bin(f"{kind.name}-ctrl-firmware.bin")
    check_control_report(kind, good)
    with pytest.raises(ReportError, match="bytes"):
        check_control_report(kind, good + b"\x00")
    with pytest.raises(ReportError, match="report id"):
        check_control_report(kind, bytes([0x01]) + good[1:])
    with pytest.raises(ReportError):
        finalize_control_report(kind, bytearray(good[:-1]))


def test_aquaero_patch_reproduces_the_driver_write_byte_for_byte() -> None:
    buf = bytearray(_bin("aquaero-ctrl-firmware.bin"))
    patch_duties(AQUAERO, buf, {1: 1412})
    finalize_control_report(AQUAERO, buf)
    assert bytes(buf) == _bin("aquaero-ctrl-after-writes.bin")


def test_quadro_patch_reproduces_the_driver_write_byte_for_byte() -> None:
    buf = bytearray(_bin("quadro-ctrl-firmware.bin"))
    patch_duties(QUADRO, buf, {2: 902})
    finalize_control_report(QUADRO, buf)
    assert bytes(buf) == _bin("quadro-ctrl-after-writes.bin")


def test_aquaero_channel_state_before_and_after_the_write() -> None:
    firmware = _bin("aquaero-ctrl-firmware.bin")
    after = _bin("aquaero-ctrl-after-writes.bin")
    before = channel_state(AQUAERO, firmware, 1)
    assert before.source == 0x59 and not before.on_duty
    assert not channel_holds(AQUAERO, firmware, 1, control_duty(AQUAERO, firmware, 1))
    state = channel_state(AQUAERO, after, 1)
    assert (state.duty, state.source, state.min_power, state.max_power) == (1412, 0x5D, 0, 10000)
    assert state.on_duty
    assert channel_holds(AQUAERO, after, 1, 1412)
    assert not channel_holds(AQUAERO, after, 1, 1413)
    # Firmware controllers on every other channel.
    assert {channel_state(AQUAERO, after, k).source for k in (0, 2, 3)} <= {0x58, 0x59}


def test_quadro_channel_state_has_no_aquaero_fields() -> None:
    after = _bin("quadro-ctrl-after-writes.bin")
    state = channel_state(QUADRO, after, 2)
    assert (state.duty, state.source, state.min_power, state.max_power) == (902, None, None, None)
    assert state.on_duty and channel_holds(QUADRO, after, 2, 902)
    assert control_duty(QUADRO, after, 0) == 10000


@pytest.mark.parametrize(
    ("kind", "k", "duty"), [(AQUAERO, 1, 1412), (QUADRO, 2, 902)], ids=["aquaero", "quadro"]
)
def test_capture_and_restore_undo_a_write(kind: DeviceKind, k: int, duty: int) -> None:
    firmware = _bin(f"{kind.name}-ctrl-firmware.bin")
    snapshot = capture_channel(kind, firmware, k)
    buf = bytearray(_bin(f"{kind.name}-ctrl-after-writes.bin"))
    assert channel_holds(kind, buf, k, duty)
    restore_channel(buf, snapshot)
    finalize_control_report(kind, buf)
    assert bytes(buf) == firmware


@pytest.mark.parametrize("duty", [-1, DUTY_MAX + 1, 1.5, True])
def test_patch_rejects_a_duty_outside_the_field(duty) -> None:
    buf = bytearray(_bin("quadro-ctrl-firmware.bin"))
    with pytest.raises(ValueError):
        patch_duties(QUADRO, buf, {0: duty})
    assert bytes(buf) == _bin("quadro-ctrl-firmware.bin")


def test_patch_rejects_an_unknown_channel() -> None:
    with pytest.raises(IndexError):
        patch_duties(AQUAERO, bytearray(_bin("aquaero-ctrl-firmware.bin")), {4: 0})


def test_secondary_reports_and_kind_table() -> None:
    assert AQUAERO.secondary_report == bytes.fromhex("06 00 02 00 00 00 00")
    assert QUADRO.secondary_report == bytes.fromhex("02 00 00 00 02 00 00 00 00 34 C6")
    assert (AQUAERO.product_id, AQUAERO.interface) == (0xF001, 2)
    assert (QUADRO.product_id, QUADRO.interface) == (0xF00D, 1)
    assert (AQUAERO.fan_input_count, QUADRO.fan_input_count) == (6, 5)
    assert AQUAERO.pwm_count == QUADRO.pwm_count == 4
    assert kind_by_name("quadro") is QUADRO and set(KINDS) == {"aquaero", "quadro"}
    with pytest.raises(ValueError, match="supported"):
        kind_by_name("octo")


@given(
    kind=st.sampled_from([AQUAERO, QUADRO]),
    duties=st.dictionaries(st.integers(0, 3), st.integers(0, DUTY_MAX), min_size=1),
)
def test_patched_duties_read_back_and_hold(kind: DeviceKind, duties: dict[int, int]) -> None:
    buf = bytearray(_bin(f"{kind.name}-ctrl-firmware.bin"))
    patch_duties(kind, buf, duties)
    finalize_control_report(kind, buf)
    check_control_report(kind, bytes(buf))
    for k, duty in duties.items():
        assert control_duty(kind, buf, k) == duty
        assert channel_holds(kind, buf, k, duty)
    untouched = set(range(4)) - set(duties)
    firmware = _bin(f"{kind.name}-ctrl-firmware.bin")
    for k in untouched:
        assert capture_channel(kind, buf, k) == capture_channel(kind, firmware, k)


@given(st.integers(0, DUTY_MAX))
def test_duty_fraction_round_trips(duty: int) -> None:
    """The adapter reports duty / 10000 and commands round(value * 10000)."""
    assert round((duty / DUTY_MAX) * DUTY_MAX) == duty
