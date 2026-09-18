"""Tests for aqua_bridge.hw.aquacomputer: report layouts against captured reports.

The fixtures in tests/fixtures/aquacomputer/ are real reports (serial bytes
zeroed) and the Linux driver's hwmon readings taken about a second later
(PROJECT.md section 2, "USB spike results", "hidraw check"; section 4.7). The
``aquaero-*-aquabus-*`` and ``aquaero-status-no-aquabus`` captures are from
firmware 2104 with the Quadro on the aquaero's aquabus and without it
(PROJECT.md section 8 item 85). ``aquaero-status-aquabus-block7-power.bin``,
``-block7-no-power.bin`` and ``aquaero-ctrl-aquabus-all-on-preset1.bin`` are the
read-only verification of 2026-09-17 in the final wiring (PROJECT.md section 2,
"aquabus fields checked against the live devices"; section 8 items 89 and 35):
two status reports one second apart, so the aquabus blocks' electrical fields
can be pinned in both of their states, and the control report that goes with
them.
"""

from __future__ import annotations

import json
from pathlib import Path

import pytest
from hypothesis import given
from hypothesis import strategies as st

from aqua_bridge.hw import aquacomputer
from aqua_bridge.hw.aquacomputer import (
    AQUAERO,
    AQUAERO_CTRL_BLOCKS,
    CONTROL_BLOCK_UNDECODED,
    DUTY_MAX,
    FAN_ABSENT_RPM,
    KINDS,
    QUADRO,
    SENSOR_NOT_CONNECTED,
    SOFT_SENSOR_REPORT_ID,
    SOURCE_UNCONFIGURED,
    TEMP_MAX_C,
    TEMP_MIN_C,
    DeviceKind,
    ReportError,
    active_profile,
    aquabus_present,
    capture_channel,
    channel_holds,
    channel_state,
    check_control_report,
    control_duty,
    crc16_usb,
    decode_status,
    finalize_control_report,
    format_undecoded_words,
    is_status_report,
    kind_by_name,
    output_mode,
    patch_duties,
    restore_channel,
    software_sensor_report,
    software_sensor_settings,
)

FIXTURES = Path(__file__).resolve().parent / "fixtures" / "aquacomputer"

#: The Linux driver's hwmon attribute names -> the names used here. The driver
#: numbers the temperatures in one run and the flow sensors after the fans.
DRIVER_NAMES = {
    "aquaero": {
        **{f"temp{n}": f"temp{n}" for n in range(1, 9)},
        **{f"temp{8 + n}": f"soft{n}" for n in range(1, 9)},
        **{f"temp{16 + n}": f"virt{n}" for n in range(1, 5)},
        **{f"fan{n}": f"fan{n}" for n in range(1, 5)},
        "fan5": "flow1",
        "fan6": "flow2",
    },
    "quadro": {
        **{f"temp{n}": f"temp{n}" for n in range(1, 5)},
        **{f"temp{4 + n}": f"soft{n}" for n in range(1, 17)},
        **{f"fan{n}": f"fan{n}" for n in range(1, 5)},
        "fan5": "flow1",
    },
}


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
    names = DRIVER_NAMES[kind.name]
    assert report.kind == kind.name
    for attribute, name in names.items():
        expected = driver[f"{attribute}_input"]
        if name.startswith("fan"):
            assert report.rpm(int(name[3:])) == expected, attribute
        elif name.startswith("flow"):
            assert report.flow(int(name[4:])) == expected, attribute
        elif expected is None:
            assert report.temp(name) is None, attribute
        else:
            # The driver had already taken the next report for one Quadro input.
            value = report.temp(name)
            assert value is not None and abs(value * 1000 - expected) <= 20, attribute
    # Every temperature the driver has is covered; the aquabus slots are new.
    driver_temps = {name for name in names.values() if not name.startswith(("fan", "flow"))}
    assert set(report.temps) - driver_temps == {f"bus{n}" for n in range(1, 9)} & set(
        kind.temp_names
    )
    for k in range(4):
        fan = report.fans[k]
        assert fan.voltage_cv * 10 == driver[f"in{k}_input"], f"in{k}"
        assert fan.current_ma == driver[f"curr{k + 1}_input"], f"curr{k + 1}"
        assert fan.power_cw * 10000 == driver[f"power{k + 1}_input"], f"power{k + 1}"
    assert report.serial == "00000-00000"  # zeroed in the fixture


def test_temperature_group_names_in_report_order() -> None:
    assert AQUAERO.temp_names == (
        *(f"temp{n}" for n in range(1, 9)),
        *(f"bus{n}" for n in range(1, 9)),
        *(f"soft{n}" for n in range(1, 9)),
        *(f"virt{n}" for n in range(1, 5)),
    )
    assert [(g.prefix, g.offset, g.count) for g in AQUAERO.temp_groups] == [
        ("temp", 0x65, 8),
        ("bus", 0x75, 8),
        ("soft", 0x85, 8),
        ("virt", 0x95, 4),
    ]
    assert QUADRO.temp_names == (
        *(f"temp{n}" for n in range(1, 5)),
        *(f"soft{n}" for n in range(1, 17)),
    )
    assert AQUAERO.describe_temps() == "temp1..temp8, bus1..bus8, soft1..soft8, virt1..virt4"
    assert list(decode_status(AQUAERO, _bin("aquaero-status.bin")).temps) == list(
        AQUAERO.temp_names
    )


def test_aquaero_status_output_duty_field() -> None:
    """Captured with pwm2 commanded to 14.12 %, every other output at 100 %."""
    report = decode_status(AQUAERO, _bin("aquaero-status.bin"))
    assert [report.duty(n) for n in range(1, 5)] == [10000, 1412, 10000, 10000]
    assert report.power_cycles is None
    assert report.firmware > 0


def test_aquaero_without_a_device_on_aquabus() -> None:
    """Firmware 2104, the Quadro not on aquabus: fans 5-8 read rpm 0xFFFF and 0 V, the
    aquabus temperature slots and flow 3 no data. Software sensors 1 and 2 are enabled
    and show their fallback values 40 and 50 degC."""
    report = decode_status(AQUAERO, _bin("aquaero-status-no-aquabus.bin"))
    assert report.firmware == 2104
    assert report.temp("temp6") == pytest.approx(22.42)
    assert [report.temp(f"bus{n}") for n in range(1, 9)] == [None] * 8
    assert report.temp("soft1") == pytest.approx(40.0)
    assert report.temp("soft2") == pytest.approx(50.0)
    assert [report.temp(f"soft{n}") for n in range(3, 9)] == [None] * 6
    assert [report.temp(f"virt{n}") for n in range(1, 5)] == [None] * 4
    assert [fan.present for fan in report.fans] == [True] * 4 + [False] * 4
    assert [report.rpm(n) for n in range(5, 9)] == [FAN_ABSENT_RPM] * 4
    assert [report.fans[k].voltage_cv for k in range(4, 8)] == [0] * 4
    assert [report.rpm(n) for n in range(1, 5)] == [353, 121, 0, 365]
    assert [report.duty(n) for n in range(1, 5)] == [2500, 1412, 10000, 2500]
    assert report.flows == (0, 0, None)


@pytest.mark.parametrize(
    ("name", "duty", "rpm", "bus2"),
    [
        ("aquaero-status-aquabus-fan7-100.bin", 10000, 1105, 24.14),
        ("aquaero-status-aquabus-fan7-902.bin", 902, 128, 24.28),
    ],
    ids=["100", "9.02"],
)
def test_aquaero_with_the_quadro_on_aquabus(name: str, duty: int, rpm: int, bus2: float) -> None:
    """The Quadro's outputs 1-4 in the aquaero's fan blocks 5-8 (a fan on its output 3,
    so fan 7), its sensor 2 in aquabus slot 2, its flow in flow 3."""
    report = decode_status(AQUAERO, _bin(name))
    assert all(fan.present for fan in report.fans)
    assert [report.rpm(n) for n in (5, 6, 8)] == [0, 0, 0]
    assert (report.rpm(7), report.duty(7)) == (rpm, duty)
    assert report.temp("bus2") == pytest.approx(bus2)
    assert [report.temp(f"bus{n}") for n in (1, 3, 4, 5, 6, 7, 8)] == [None] * 7
    assert (report.temp("soft1"), report.temp("soft2")) == (
        pytest.approx(40.0),
        pytest.approx(50.0),
    )
    assert report.flow(3) == 0
    if duty == DUTY_MAX:
        fan7 = report.fans[6]
        assert (fan7.voltage_cv, fan7.current_ma, fan7.power_cw) == (1210, 27, 32)
        assert fan7.voltage_v == pytest.approx(12.10) and fan7.power_w == pytest.approx(0.32)


# --- the aquabus fields against the live devices (items 89, 35; 2026-09-17) -------------


def test_the_aquabus_blocks_electrical_fields_are_not_a_per_report_reading() -> None:
    """Two status reports one second apart, the Quadro on aquabus, every output on
    preset 1 at 20.00 %, a fan on the Quadro's output 3 (aquaero block 7) turning at
    255 rpm in both. In one the block's electrical group carries a sample taken in the
    on phase of the 20 % duty -- block 7 at 6 mA / 0.07 W, the three outputs with no fan
    at 0.00 V -- and the next carries what a sample in the off phase reads: the rail
    voltage on all four and 0 mA / 0 W on the turning fan. So a single report says
    nothing about an aquabus output's draw *or its rail*, and both ``reports_power`` and
    ``reports_rail`` are False for the aquaero's aquabus outputs.

    ``reports_power`` is False for every output of both kinds (2026-09-18): the aquaero
    measures no current on its own four, and the Quadro -- the device behind these very
    numbers -- measures one it samples inside the PWM cycle, so the share of reports
    carrying a non-zero current follows the duty (4 of 14 at 25 %, 16 of 16 at 60 %) and
    no single report may be judged. The aquaero's own four do report their own rail.
    """
    with_power = decode_status(AQUAERO, _bin("aquaero-status-aquabus-block7-power.bin"))
    without = decode_status(AQUAERO, _bin("aquaero-status-aquabus-block7-no-power.bin"))
    for report in (with_power, without):
        assert all(fan.present for fan in report.fans)
        assert [report.duty(n) for n in range(1, 9)] == [2000] * 8
        assert report.rpm(7) == 255
    seven_on, seven_off = with_power.fans[6], without.fans[6]
    assert (seven_on.current_ma, seven_on.power_cw) == (6, 7)
    assert (seven_off.current_ma, seven_off.power_cw) == (0, 0)
    assert seven_on.voltage_cv == 1210 and seven_off.voltage_cv == 1209
    # The idle aquabus outputs read 0.00 V in the report that carries measurements.
    assert [with_power.fans[k].voltage_cv for k in (4, 5, 7)] == [0, 0, 0]
    assert [without.fans[k].voltage_cv for k in (4, 5, 7)] == [1209, 1209, 1209]
    assert [AQUAERO.reports_power(n) for n in range(1, 9)] == [False] * 8
    assert not AQUAERO.own_outputs_report_power and not AQUAERO.aquabus_outputs_report_power
    assert [QUADRO.reports_power(n) for n in range(1, 5)] == [False] * 4
    # Two different facts behind the one refusal, and the reason says which.
    assert "0 mA and 0 W for its own outputs" in AQUAERO.no_power_reason(1)
    assert "sampled inside the PWM cycle" in QUADRO.no_power_reason(1)
    assert "the bus device's own sample" in AQUAERO.no_power_reason(7)
    assert not AQUAERO.own_outputs_measures_current and QUADRO.own_outputs_measures_current
    # The rail goes the same way: block 7's 1210 is the Quadro's and 1209 the aquaero's
    # own, one second apart, at one unchanged duty -- so no aquabus block's voltage is
    # that output's rail, while the aquaero's own blocks 1-4 report theirs.
    assert [AQUAERO.reports_rail(n) for n in range(1, 9)] == [True] * 4 + [False] * 4
    assert not AQUAERO.aquabus_outputs_report_rail
    assert all(QUADRO.reports_rail(n) for n in range(1, 5))
    assert [with_power.fans[k].voltage_cv for k in range(4)] == [1205, 1206, 1207, 1205]


def test_the_unidentified_u16_of_a_fan_block_is_read_but_named_nothing() -> None:
    """Item 114. The ``u16`` at ``+0x0A`` is decoded raw and carries no unit: it is not
    the current (26 against that block's 6 mA) and not the power (7 cW), and it is 0 on
    every one of the aquaero's own blocks and 0 on an aquabus block whose sample fell in
    the off phase.

    The suspicion that used to be recorded here -- a current over the output's on-time,
    of which the current field is the duty average -- is **withdrawn** (2026-09-18): the
    current field is not an average over the report at all but one sample taken inside
    the PWM cycle, so the two numbers are two coordinates of one instant and no duty
    relation can be fitted to them. What is pinned instead is the pair that no scaling of
    the current survives: at one duty this field read 26 with 6 mA, and at full duty 27
    with 27 mA.
    """
    with_power = decode_status(AQUAERO, _bin("aquaero-status-aquabus-block7-power.bin"))
    without = decode_status(AQUAERO, _bin("aquaero-status-aquabus-block7-no-power.bin"))
    full = decode_status(AQUAERO, _bin("aquaero-status-aquabus-fan7-100.bin"))
    seven = with_power.fans[6]
    assert seven.unidentified_raw == 26
    assert (seven.duty, seven.current_ma, seven.power_cw) == (2000, 6, 7)
    assert full.fans[6].unidentified_raw == 27
    assert (full.fans[6].duty, full.fans[6].current_ma) == (10000, 27)
    # Two duties, two ratios to the current field: 4.33 at 20 % and 1.00 at 100 %.
    # No constant scaling of the current explains both, and the 2026-09-18 run adds a
    # report that read 2 mA with this field at 0.
    assert seven.unidentified_raw / seven.current_ma == pytest.approx(4.33, abs=0.01)
    assert full.fans[6].unidentified_raw == full.fans[6].current_ma
    # 0 on the aquaero's own blocks, and on an aquabus block whose sample missed.
    assert [with_power.fans[k].unidentified_raw for k in range(4)] == [0] * 4
    assert [without.fans[k].unidentified_raw for k in range(4, 8)] == [0] * 4
    # The Quadro's own blocks have no such field at all.
    assert all(
        fan.unidentified_raw is None
        for fan in decode_status(QUADRO, _bin("quadro-status.bin")).fans
    )
    assert QUADRO.fan_layout.unidentified is None and AQUAERO.fan_layout.unidentified == 0x0A


def test_no_kind_carries_an_aquabus_refresh_interval_any_more() -> None:
    """Item 115, corrected 2026-09-18. The kind used to carry a mean "aquabus refresh
    interval" of 4 reports / 4.0 s, measured by counting the reports whose aquabus block
    held a non-zero current. That count is now known to follow the output's duty -- 4 of
    14 at 25 %, 16 of 16 at 60 % -- which no fixed poll can produce, so the number was
    withdrawn rather than re-rounded, together with the two module constants and the
    ``device_health`` key that published it. Nothing was bounded by it: presence is read
    from the speed field, which every report carries."""
    assert not hasattr(AQUAERO, "aquabus_refresh_reports")
    assert not hasattr(AQUAERO, "aquabus_refresh_s")
    assert not hasattr(aquacomputer, "AQUABUS_REFRESH_REPORTS")
    assert not hasattr(aquacomputer, "AQUABUS_REFRESH_S")
    assert AQUAERO.aquabus_temp_names == tuple(f"bus{i}" for i in range(1, 9))
    assert QUADRO.aquabus_temp_names == ()


# --- the controller block, and the start boost that is not in it yet -------------------


def _ctrl_block(data: bytes, k: int) -> bytes:
    """Controller block ``k`` (0-based) of an aquaero control report, raw.

    The offsets are spelled out here rather than taken from the module, because what
    these tests pin is that the module's layout matches the captures.
    """
    base = 0x20C + 20 * k
    return data[base : base + 20]


def test_the_aquaero_has_twelve_controller_blocks_and_the_spare_four_are_the_default() -> None:
    """The block array runs to ``k`` = 11 (4 own outputs + 8 aquabus), ending at
    ``0x2FB``. Blocks 9-12 of both captures are byte-identical to each other and to the
    block 8 that the 2026-09-15 capture caught while it was unconfigured -- source
    ``0xFFFF``, minimum power 35.00 %, maximum 100.00 % -- which is what makes them
    readable as the firmware's default for an output nothing is assigned to, and the
    reference a later capture's changed block is diffed against.

    This project drives the first eight (its own four and one Quadro's four); the last
    four are recorded, not used. Where the array *ends* is pinned too: the 20 bytes at
    ``0x2FC`` are not another controller block, which is what makes twelve the count
    rather than the first number that happened to be looked at."""
    firmware = _bin("aquaero-ctrl-firmware.bin")
    owner = _bin("aquaero-ctrl-aquabus-all-on-preset1.bin")
    assert AQUAERO_CTRL_BLOCKS == 12
    assert 0x20C + 20 * AQUAERO_CTRL_BLOCKS == 0x2FC <= AQUAERO.ctrl_size
    assert AQUAERO.pwm_count == 8  # what the daemon commands, not what the report holds
    default = _ctrl_block(firmware, 8)
    for report in (firmware, owner):
        assert [_ctrl_block(report, k) for k in range(8, 12)] == [default] * 4
        # The series ends at 0x2FB: what follows is a different structure, not a
        # thirteenth block holding the default an unassigned output would hold.
        assert _ctrl_block(report, AQUAERO_CTRL_BLOCKS) != default
        assert int.from_bytes(_ctrl_block(report, AQUAERO_CTRL_BLOCKS)[0x10:0x12], "big") != (
            SOURCE_UNCONFIGURED
        )
    # Block 8 was unconfigured when the firmware capture was taken: the same bytes.
    assert _ctrl_block(firmware, 7) == default
    assert int.from_bytes(default[0x04:0x06], "big") == 3500  # minimum power 35.00 %
    assert int.from_bytes(default[0x06:0x08], "big") == DUTY_MAX
    assert int.from_bytes(default[0x10:0x12], "big") == SOURCE_UNCONFIGURED


def test_the_undecoded_words_of_a_controller_block_are_read_and_named_nothing() -> None:
    """The six ``u16`` of a controller block nobody here has identified are decoded raw
    into ``ChannelState.undecoded`` so a probe can print them and a later capture can be
    diffed against this one. The aquaero's per-output **start boost** is one of them and
    a read-only capture cannot say which.

    What the captures do show, and what is pinned here so a later one can be compared
    against it: ``+0x08``, ``+0x0A``, ``+0x0C`` and ``+0x12`` hold one value across all
    eight outputs, so there is nothing in them to tell one output's configuration from
    another's -- ``+0x08`` is a per-output centi-percent field that read 50.00 % on the
    seven outputs the owner had configured and 100.00 % on the unconfigured eighth, and
    reads 100.00 % everywhere since; ``+0x0A`` and ``+0x0C`` both read 2 in every block
    of every capture, so a boost duration in seconds and a tachometer's pulses per
    revolution could not be told apart even if both were there. ``+0x00`` and ``+0x02``
    do differ between outputs, and that is pinned as well -- but in a way nothing
    connects to a boost (they read like a per-fan rpm pair, and no capture pairs either
    with a boost setting known from the device's own menu)."""
    firmware = _bin("aquaero-ctrl-firmware.bin")
    owner = _bin("aquaero-ctrl-aquabus-all-on-preset1.bin")
    assert CONTROL_BLOCK_UNDECODED == (0x00, 0x02, 0x08, 0x0A, 0x0C, 0x12)
    states = [channel_state(AQUAERO, owner, k) for k in range(AQUAERO.pwm_count)]
    for k, state in enumerate(states):
        assert [offset for offset, _ in state.undecoded] == list(CONTROL_BLOCK_UNDECODED)
        block = _ctrl_block(owner, k)
        assert [value for _, value in state.undecoded] == [
            int.from_bytes(block[offset : offset + 2], "big") for offset in CONTROL_BLOCK_UNDECODED
        ]
    # Four of the six hold one value across all eight outputs: nothing in them
    # separates one output's configuration from another's.
    for offset, value in ((0x08, DUTY_MAX), (0x0A, 2), (0x0C, 2), (0x12, 1000)):
        assert {dict(state.undecoded)[offset] for state in states} == {value}
    before = [channel_state(AQUAERO, firmware, k) for k in range(AQUAERO.pwm_count)]
    for offset, value in ((0x0A, 2), (0x0C, 2), (0x12, 1000)):
        assert {dict(state.undecoded)[offset] for state in before} == {value}
    # +0x08 was 50.00 % on the seven configured outputs of the earlier capture.
    assert [dict(state.undecoded)[0x08] for state in before] == [5000] * 7 + [DUTY_MAX]
    # The two that do differ between outputs -- nothing here reads them as a boost.
    assert [dict(state.undecoded)[0x00] for state in states] == [100] + [450] * 7
    assert [dict(state.undecoded)[0x00] for state in before] == [500] * 4 + [450, 300, 500, 450]
    assert [dict(state.undecoded)[0x02] for state in before] == [
        1000,
        1000,
        1600,
        1600,
        2800,
        2000,
        1500,
        2000,
    ]
    # The Quadro's channels have no such block at all.
    assert all(
        state.undecoded == ()
        for state in (
            channel_state(QUADRO, _bin("quadro-ctrl-firmware.bin"), k)
            for k in range(QUADRO.pwm_count)
        )
    )


def test_a_duty_write_touches_none_of_the_undecoded_words() -> None:
    """Whatever those six words are -- the start boost among them -- the daemon never
    moves one: a duty write touches the preset, the control source and the two power
    limits, and nothing else in the block. Pinned so that decoding one later cannot
    quietly turn into writing it."""
    data = bytearray(_bin("aquaero-ctrl-aquabus-all-on-preset1.bin"))
    patch_duties(AQUAERO, data, {k: 7000 for k in range(AQUAERO.pwm_count)})
    finalize_control_report(AQUAERO, data)
    for k in range(AQUAERO.pwm_count):
        before = channel_state(AQUAERO, _bin("aquaero-ctrl-aquabus-all-on-preset1.bin"), k)
        assert channel_state(AQUAERO, data, k).undecoded == before.undecoded
        touched = set(AQUAERO.ctrl_channels[k].offsets())
        assert touched.isdisjoint(offset for _, offset in AQUAERO.ctrl_channels[k].undecoded)


def test_the_undecoded_words_are_printed_for_a_tool_and_nothing_else() -> None:
    """They reach a human through the probe's listing and no further: the line names
    them by their offset in the block, with no unit and no name."""
    owner = _bin("aquaero-ctrl-aquabus-all-on-preset1.bin")
    line = format_undecoded_words(channel_state(AQUAERO, owner, 0))
    assert line == "not decoded: +0x00 100  +0x02 2000  +0x08 10000  +0x0A 2  +0x0C 2  +0x12 1000"
    assert "boost" not in line
    quadro = channel_state(QUADRO, _bin("quadro-ctrl-firmware.bin"), 0)
    assert format_undecoded_words(quadro) == ""


def test_aquabus_presence_is_judged_from_the_speed_field_alone() -> None:
    """Item 92, and the line between it and items 115 and 116. A device answers on
    aquabus while any of blocks 5-8 has one behind it; every block reading speed
    ``0xFFFF`` is the empty bus. The refresh gap must not look like an absence: in the
    report that refreshed nothing every aquabus block reads 0 mA and three of them the
    aquaero's own rail where the measuring report had 0.00 V, and presence is unchanged
    in both."""
    with_power = decode_status(AQUAERO, _bin("aquaero-status-aquabus-block7-power.bin"))
    without = decode_status(AQUAERO, _bin("aquaero-status-aquabus-block7-no-power.bin"))
    empty = decode_status(AQUAERO, _bin("aquaero-status-no-aquabus.bin"))
    assert aquabus_present(AQUAERO, with_power) is True
    assert aquabus_present(AQUAERO, without) is True  # a refresh gap is not an absence
    assert [with_power.fans[k].voltage_cv for k in (4, 5, 7)] == [0, 0, 0]  # item 116
    assert aquabus_present(AQUAERO, empty) is False
    assert all(report.fans[k].rpm == FAN_ABSENT_RPM for k in range(4, 8) for report in (empty,))
    # One slot answering is a device on the bus: its other outputs simply have no fan.
    one_left = bytearray(_bin("aquaero-status-no-aquabus.bin"))
    speed = AQUAERO.fan_blocks[5] + AQUAERO.fan_layout.speed
    one_left[speed : speed + 2] = (0).to_bytes(2, "big")
    assert aquabus_present(AQUAERO, decode_status(AQUAERO, bytes(one_left))) is True
    # A kind with no aquabus outputs cannot say, and a report of another kind is refused.
    assert aquabus_present(QUADRO, decode_status(QUADRO, _bin("quadro-status.bin"))) is None
    with pytest.raises(ValueError, match="status report of a quadro, not a aquaero"):
        aquabus_present(AQUAERO, decode_status(QUADRO, _bin("quadro-status.bin")))


def test_the_aquaeros_own_blocks_report_no_current() -> None:
    """The aquaero's own outputs 1-4 in PWM mode report 0 mA and 0 W however fast the
    fan turns -- three of the four were turning in both captures (item 79)."""
    for name in (
        "aquaero-status-aquabus-block7-power.bin",
        "aquaero-status-aquabus-block7-no-power.bin",
    ):
        report = decode_status(AQUAERO, _bin(name))
        assert [report.rpm(n) for n in range(1, 5)] == [350, 176, 0, 372]
        assert all(
            report.fans[k].current_ma == 0 and report.fans[k].power_cw == 0 for k in range(4)
        )


def test_the_live_temperature_groups_and_their_not_connected_sentinel() -> None:
    """2026-09-17, the final wiring: one thermistor on the aquaero's sensor 6, the
    Quadro's sensor 2 in aquabus slot 2, all eight software sensors enabled (sensor 1
    fed by the daemon's heartbeat at 20.00 degC, the others at their 50.00 degC
    fallback), no virtual sensor configured. So 10 of the aquaero's 28 temperature
    slots carry a value and the other 18 read the 0x7FFF sentinel and decode as None
    -- the count the table in PROJECT.md section 2 states."""
    report = decode_status(AQUAERO, _bin("aquaero-status-aquabus-block7-no-power.bin"))
    assert report.firmware == 2104
    populated = sorted(name for name, value in report.temps.items() if value is not None)
    assert len(report.temps) == 28 and len(populated) == 10
    assert populated == ["bus2", *(f"soft{n}" for n in range(1, 9)), "temp6"]
    assert report.temp("temp6") == pytest.approx(23.05)
    assert [report.temp(f"temp{n}") for n in (1, 2, 3, 4, 5, 7, 8)] == [None] * 7
    assert report.temp("bus2") == pytest.approx(23.68)
    assert [report.temp(f"bus{n}") for n in (1, 3, 4, 5, 6, 7, 8)] == [None] * 7
    assert report.temp("soft1") == pytest.approx(20.0)
    assert [report.temp(f"soft{n}") for n in range(2, 9)] == [pytest.approx(50.0)] * 7
    assert [report.temp(f"virt{n}") for n in range(1, 5)] == [None] * 4
    # flow1 and flow2 are the aquaero's own with nothing connected, flow3 the Quadro's
    # on aquabus: a present-but-empty flow sensor reads 0, an absent slot 0x7FFF.
    assert report.flows == (0, 0, 0)
    assert report.flow(3) == 0


def test_the_live_control_report_every_block_on_preset_1() -> None:
    """The owner's profile 1 as the controller holds it (2026-09-17): all eight blocks
    point at preset 1 (source 0x5C) with limits 0 / 100 %, so only channel 0 -- whose
    own preset id *is* 0x5C -- reads as following its own preset. Preset 1 holds
    20.00 %, which every output's status duty shows."""
    data = _bin("aquaero-ctrl-aquabus-all-on-preset1.bin")
    check_control_report(AQUAERO, data)
    assert active_profile(AQUAERO, data) == 1
    states = [channel_state(AQUAERO, data, k) for k in range(8)]
    assert [s.source for s in states] == [0x5C] * 8
    assert [(s.min_power, s.max_power) for s in states] == [(0, 10000)] * 8
    assert [s.on_duty for s in states] == [True] + [False] * 7
    assert [s.unconfigured for s in states] == [False] * 8
    assert control_duty(AQUAERO, data, 0) == 2000
    assert channel_holds(AQUAERO, data, 0, 2000)


def test_the_aquabus_mode_word_is_read_but_never_interpreted() -> None:
    """One Quadro's four identical PWM outputs, in one report: block 5 reads 0x0000 and
    blocks 6-8 read 0x0002, and the same blocks read 0x0500 in the 2026-09-15 capture.
    The word therefore says nothing about an aquabus output, and every aquabus mode is
    reported uninterpreted whatever its low byte (item 89)."""
    data = _bin("aquaero-ctrl-aquabus-all-on-preset1.bin")
    modes = [output_mode(AQUAERO, data, k) for k in range(8)]
    assert [m.raw for m in modes] == [0x0002] * 4 + [0x0000] + [0x0002] * 3
    assert [m.interpreted for m in modes] == [True] * 4 + [False] * 4
    # Block 6's low byte is 0x02, the aquaero's own PWM code, and is still not a mode.
    assert [m.name for m in modes] == ["pwm"] * 4 + ["unknown"] * 4
    assert not any(m.is_pwm for m in modes[4:])
    assert [channel_state(AQUAERO, data, k).mode for k in range(8)] == modes


def test_only_the_speed_field_says_whether_a_bus_device_is_there() -> None:
    """Item 116: an aquabus block in a non-measuring report looks exactly like an empty
    slot on the voltage field, and only on that field.

    The three captures together separate the two cases the voltage cannot. With **no
    device on aquabus**, blocks 5-8 read rpm 0xFFFF and 0.00 V. With the **Quadro
    present**, its three outputs with no fan read 0.00 V in the report that carries the
    bus device's measurements -- byte for byte what an empty slot reads -- and 12.09 V
    one second later, at the same duty, with nothing having changed on the bus. So
    0.00 V is neither necessary nor sufficient for absence, and a rule keyed on it
    would call a present device absent in one report and present in the next.

    The speed field carries no substitute in either report: absent is 0xFFFF in both
    captures with nothing on the bus, present is 0 rpm (no fan) or the fan's 255 rpm in
    both captures with the Quadro on it. That is why absence needs no confirmation
    window: the sentinel already outlasts the refresh. Every absence judgement in this
    project reads :attr:`FanStatus.present`, which reads only that field.
    """
    empty_bus = [
        decode_status(AQUAERO, _bin(f"aquaero-status{s}.bin")) for s in ("", "-no-aquabus")
    ]
    measuring = decode_status(AQUAERO, _bin("aquaero-status-aquabus-block7-power.bin"))
    substituted = decode_status(AQUAERO, _bin("aquaero-status-aquabus-block7-no-power.bin"))
    aquabus = [n - 1 for n in AQUAERO.aquabus_outputs]
    assert aquabus == [4, 5, 6, 7]

    for report in empty_bus:  # nothing on the bus: the sentinel, and 0.00 V with it
        assert [report.fans[k].rpm for k in aquabus] == [FAN_ABSENT_RPM] * 4
        assert [report.fans[k].voltage_cv for k in aquabus] == [0] * 4
        assert [report.fans[k].present for k in aquabus] == [False] * 4

    # The Quadro is on the bus in both of these, one second apart at an unchanged duty.
    assert [measuring.fans[k].rpm for k in aquabus] == [0, 0, 255, 0]
    assert [substituted.fans[k].rpm for k in aquabus] == [0, 0, 255, 0]
    assert [measuring.fans[k].present for k in aquabus] == [True] * 4
    assert [substituted.fans[k].present for k in aquabus] == [True] * 4
    # ... while the voltage of those same present outputs reads both ways within a second
    assert [measuring.fans[k].voltage_cv for k in aquabus] == [0, 0, 1210, 0]
    assert [substituted.fans[k].voltage_cv for k in aquabus] == [1209, 1209, 1209, 1209]
    # 0.00 V on a present output is indistinguishable from 0.00 V on an absent one.
    assert measuring.fans[4].voltage_cv == empty_bus[0].fans[4].voltage_cv == 0
    assert measuring.fans[4].present and not empty_bus[0].fans[4].present


def test_the_software_sensor_settings_of_five_captured_control_reports() -> None:
    """Item 113: five bytes per sensor from 0x177 -- enabled, fallback temperature,
    timeout -- and what they explain about the status reports beside them.

    In the 2026-09-17 capture all eight sensors are enabled: sensor 1 carries the
    owner's watchdog (30 s, falling back to 90.00 degC, the alarm that drives every
    output to 100 %) and the other seven a 300 s / 50.00 degC fallback that nothing
    writes -- which is exactly the steady 50.00 degC their status reports show. In the
    earlier captures sensor 1 holds the 300 s / 40.00 degC of item 84's experiment, and
    sensors 3-8 are disabled, which is why those status reports show 0x7FFF for them.
    A disabled slot is therefore the only one a status report marks; an enabled slot
    nothing feeds reads its fallback and nothing says so.
    """
    live = software_sensor_settings(AQUAERO, _bin("aquaero-ctrl-aquabus-all-on-preset1.bin"))
    assert [s.name for s in live] == [f"soft{n}" for n in range(1, 9)]
    assert all(s.enabled for s in live)
    assert (live[0].fallback_c, live[0].timeout_s) == (pytest.approx(90.0), 30)
    assert [(s.fallback_c, s.timeout_s) for s in live[1:]] == [(pytest.approx(50.0), 300)] * 7

    status = decode_status(AQUAERO, _bin("aquaero-status-aquabus-block7-no-power.bin"))
    # The fed slot does not read its fallback; the seven unfed ones read exactly theirs.
    assert not live[0].reads_fallback(status.temp("soft1"))
    assert all(s.reads_fallback(status.temp(s.name)) for s in live[1:])
    assert live[0].reads_fallback(None) is False

    earlier = software_sensor_settings(AQUAERO, _bin("aquaero-ctrl-firmware.bin"))
    assert [s.enabled for s in earlier] == [True, True] + [False] * 6
    assert (earlier[0].fallback_c, earlier[0].timeout_s) == (pytest.approx(40.0), 300)
    old_status = decode_status(AQUAERO, _bin("aquaero-status.bin"))
    # Disabled is the one state the status report shows: 0x7FFF, decoded as None.
    for s in earlier:
        assert (old_status.temp(s.name) is None) is not s.enabled
    # Enabled-and-unfed still reads a plain number: soft2 shows its 50.00 degC fallback.
    assert old_status.temp("soft2") == pytest.approx(earlier[1].fallback_c)

    # A kind whose settings are not known simply has none, and names no soft slot.
    assert software_sensor_settings(QUADRO, _bin("quadro-ctrl-firmware.bin")) == ()
    assert AQUAERO.soft_sensor_names == tuple(f"soft{n}" for n in range(1, 9))
    assert QUADRO.soft_sensor_names == tuple(f"soft{n}" for n in range(1, 17))


def test_software_sensor_settings_reject_a_report_that_is_not_one() -> None:
    with pytest.raises(ReportError):
        software_sensor_settings(AQUAERO, _bin("aquaero-status.bin"))


def test_quadro_status_output_duty_and_power_cycles() -> None:
    """Captured with pwm3 commanded to 9.02 %; the Quadro's duty agrees with the
    driver's pwmN (its control report readback)."""
    report = decode_status(QUADRO, _bin("quadro-status.bin"))
    driver = _readings("quadro-driver-readings.json")
    assert [report.duty(n) for n in range(1, 5)] == [10000, 10000, 902, 10000]
    for n in range(1, 5):
        assert round(report.duty(n) * 255 / DUTY_MAX) == driver[f"pwm{n}"]
    assert isinstance(report.power_cycles, int) and report.power_cycles > 0
    assert all(fan.present for fan in report.fans)


def test_negative_temperature_is_signed_not_655_degc() -> None:
    data = bytearray(_bin("aquaero-status.bin"))
    data[0x65:0x67] = (-250).to_bytes(2, "big", signed=True)
    assert decode_status(AQUAERO, bytes(data)).temp("temp1") == pytest.approx(-2.5)


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


def test_channel_numbers_and_names_out_of_range_raise() -> None:
    report = decode_status(QUADRO, _bin("quadro-status.bin"))
    with pytest.raises(KeyError, match="temp5"):
        report.temp("temp5")
    with pytest.raises(KeyError, match="bus1"):
        report.temp("bus1")  # the Quadro has no aquabus slots
    with pytest.raises(IndexError, match="fan1..fan4"):
        report.rpm(5)
    with pytest.raises(IndexError, match="flow1..flow1"):
        report.flow(2)
    with pytest.raises(IndexError, match="pwm1..pwm4"):
        report.duty(0)
    aquaero = decode_status(AQUAERO, _bin("aquaero-status.bin"))
    with pytest.raises(IndexError, match="pwm1..pwm8"):
        aquaero.duty(9)


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


def test_aquaero_aquabus_output_7_patch_reproduces_the_hardware_write() -> None:
    """Hardware 2026-09-15: duty 9.02 % on aquaero output 7 (the Quadro's output 3 on
    aquabus) changed only min power, source and preset 7."""
    before = _bin("aquaero-ctrl-aquabus-before-fan7-write.bin")
    after = _bin("aquaero-ctrl-aquabus-after-fan7-write.bin")
    assert [i for i in range(len(before)) if before[i] != after[i]] == [
        0x288,
        0x289,
        0x295,
        0x568,
        0x569,
    ]
    buf = bytearray(before)
    patch_duties(AQUAERO, buf, {6: 902})
    finalize_control_report(AQUAERO, buf)
    assert bytes(buf) == after
    state = channel_state(AQUAERO, before, 6)
    assert (state.source, state.min_power, state.duty, state.on_duty) == (0x59, 3996, 10000, False)
    state = channel_state(AQUAERO, after, 6)
    assert (state.duty, state.source, state.min_power, state.max_power) == (902, 0x62, 0, 10000)
    assert state.on_duty and state.aquabus and channel_holds(AQUAERO, after, 6, 902)
    snapshot = capture_channel(AQUAERO, before, 6)
    restore_channel(buf, snapshot)
    finalize_control_report(AQUAERO, buf)
    assert bytes(buf) == before


def test_aquaero_control_blocks_of_all_eight_outputs() -> None:
    """Blocks at 0x20C + 20k, presets at 0x55C + 2k with id 0x5C + k; 5-8 on aquabus."""
    for k, channel in enumerate(AQUAERO.ctrl_channels):
        base = 0x20C + 20 * k
        assert (channel.duty, channel.source, channel.preset_id) == (
            0x55C + 2 * k,
            base + 0x10,
            0x5C + k,
        )
        assert (channel.min_power, channel.max_power, channel.mode) == (
            base + 4,
            base + 6,
            base + 0x0E,
        )
        assert channel.aquabus == (k >= 4)
    assert AQUAERO.fan_blocks == tuple(0x167 + 12 * k for k in range(8))
    assert AQUAERO.fan_blocks[4:] == (0x197, 0x1A3, 0x1AF, 0x1BB)
    assert AQUAERO.aquabus_outputs == (5, 6, 7, 8) and QUADRO.aquabus_outputs == ()


def test_aquaero_channel_state_before_and_after_the_write() -> None:
    firmware = _bin("aquaero-ctrl-firmware.bin")
    after = _bin("aquaero-ctrl-after-writes.bin")
    before = channel_state(AQUAERO, firmware, 1)
    assert before.source == 0x59 and not before.on_duty and not before.aquabus
    assert not channel_holds(AQUAERO, firmware, 1, control_duty(AQUAERO, firmware, 1))
    state = channel_state(AQUAERO, after, 1)
    assert (state.duty, state.source, state.min_power, state.max_power) == (1412, 0x5D, 0, 10000)
    assert state.on_duty
    assert channel_holds(AQUAERO, after, 1, 1412)
    assert not channel_holds(AQUAERO, after, 1, 1413)
    # Firmware controllers on every other channel.
    assert {channel_state(AQUAERO, after, k).source for k in (0, 2, 3)} <= {0x58, 0x59}


def test_aquaero_output_mode_word() -> None:
    """The firmware fixtures: outputs 1-2 in PWM mode (0x0502), 3-4 in DC mode (0x0501;
    output 4 was switched to PWM before the aquabus captures), aquabus blocks 5-7 0x0500
    (low byte 0, not interpreted) and block 8 unconfigured (mode 0, source 0xFFFF); a
    duty write does not touch the mode."""
    output_4 = {"dc": 0x0501, "pwm": 0x0502}
    for name, fourth in (
        ("aquaero-ctrl-firmware.bin", "dc"),
        ("aquaero-ctrl-after-writes.bin", "dc"),
        ("aquaero-ctrl-aquabus-before-fan7-write.bin", "pwm"),
        ("aquaero-ctrl-aquabus-after-fan7-write.bin", "pwm"),
    ):
        data = _bin(name)
        modes = [output_mode(AQUAERO, data, k) for k in range(8)]
        assert [m.raw for m in modes] == [0x0502, 0x0502, 0x0501, output_4[fourth]] + [
            0x0500
        ] * 3 + [0]
        assert [m.name for m in modes] == ["pwm", "pwm", "dc", fourth] + ["unknown"] * 4
        states = [channel_state(AQUAERO, data, k) for k in range(8)]
        assert [s.mode for s in states] == modes
        assert [s.unconfigured for s in states] == [False] * 7 + [True]
        assert states[7].source == SOURCE_UNCONFIGURED
    patched = bytearray(_bin("aquaero-ctrl-firmware.bin"))
    patched[0x248 + 0x0E : 0x248 + 0x10] = (0x0503).to_bytes(2, "big")
    odd = output_mode(AQUAERO, patched, 3)
    assert odd is not None and odd.name == "unknown" and not odd.is_pwm


def test_writing_the_unconfigured_block_sets_the_same_fields() -> None:
    buf = bytearray(_bin("aquaero-ctrl-aquabus-before-fan7-write.bin"))
    patch_duties(AQUAERO, buf, {7: 5000})
    state = channel_state(AQUAERO, buf, 7)
    assert (state.duty, state.source, state.min_power, state.max_power) == (5000, 0x63, 0, 10000)
    assert state.on_duty and not state.unconfigured and state.mode is not None
    assert state.mode.raw == 0  # the mode is never written


def test_quadro_has_no_known_output_mode() -> None:
    data = _bin("quadro-ctrl-firmware.bin")
    assert output_mode(QUADRO, data, 0) is None
    state = channel_state(QUADRO, data, 0)
    assert state.mode is None and not state.unconfigured and not state.aquabus


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
        patch_duties(AQUAERO, bytearray(_bin("aquaero-ctrl-firmware.bin")), {8: 0})
    with pytest.raises(IndexError):
        patch_duties(QUADRO, bytearray(_bin("quadro-ctrl-firmware.bin")), {4: 0})


def test_save_reports_and_kind_table() -> None:
    """The save report: aquaero report 6 (verified to save, PROJECT.md section 8 item 84);
    the Quadro's equals the Farbwerk 360's documented save report (not verified)."""
    assert AQUAERO.save_report == bytes.fromhex("06 00 02 00 00 00 00")
    assert QUADRO.save_report == bytes.fromhex("02 00 00 00 02 00 00 00 00 34 C6")
    assert AQUAERO.save_verified and not QUADRO.save_verified
    assert (AQUAERO.product_id, AQUAERO.interface) == (0xF001, 2)
    assert (QUADRO.product_id, QUADRO.interface) == (0xF00D, 1)
    assert (AQUAERO.pwm_count, AQUAERO.fan_count, AQUAERO.flow_count) == (8, 8, 3)
    assert (QUADRO.pwm_count, QUADRO.fan_count, QUADRO.flow_count) == (4, 4, 1)
    assert (AQUAERO.temp_count, QUADRO.temp_count) == (28, 20)
    assert kind_by_name("quadro") is QUADRO and set(KINDS) == {"aquaero", "quadro"}
    with pytest.raises(ValueError, match="supported"):
        kind_by_name("octo")


@given(
    kind=st.sampled_from([AQUAERO, QUADRO]),
    data=st.data(),
)
def test_patched_duties_read_back_and_hold(kind: DeviceKind, data) -> None:
    duties = data.draw(
        st.dictionaries(st.integers(0, kind.pwm_count - 1), st.integers(0, DUTY_MAX), min_size=1)
    )
    buf = bytearray(_bin(f"{kind.name}-ctrl-firmware.bin"))
    patch_duties(kind, buf, duties)
    finalize_control_report(kind, buf)
    check_control_report(kind, bytes(buf))
    for k, duty in duties.items():
        assert control_duty(kind, buf, k) == duty
        assert channel_holds(kind, buf, k, duty)
    untouched = set(range(kind.pwm_count)) - set(duties)
    firmware = _bin(f"{kind.name}-ctrl-firmware.bin")
    for k in untouched:
        assert capture_channel(kind, buf, k) == capture_channel(kind, firmware, k)


@given(st.integers(0, DUTY_MAX))
def test_duty_fraction_round_trips(duty: int) -> None:
    """The adapter reports duty / 10000 and commands round(value * 10000)."""
    assert round((duty / DUTY_MAX) * DUTY_MAX) == duty


# --- software sensors and profiles (item 84) ----------------------------------------------


def test_software_sensor_report_sets_one_sensor_and_leaves_the_others_alone() -> None:
    """Report 0x07, 17 bytes: the id then eight centi-degC s16 values; every slot the
    call does not name reads 0x7FFF ("no data"), so the device keeps its own value
    (verified on the aquaero, PROJECT.md section 8 item 84)."""
    report = software_sensor_report(AQUAERO, {1: 20.0})
    assert len(report) == 1 + 2 * 8 == 17
    assert report[0] == SOFT_SENSOR_REPORT_ID == 0x07
    assert report == bytes.fromhex("07 07d0" + " 7fff" * 7)
    assert software_sensor_report(AQUAERO, {8: 90.0}) == bytes.fromhex("07" + " 7fff" * 7 + " 2328")
    both = software_sensor_report(AQUAERO, {1: -1.5, 3: 0.0})
    assert both == bytes.fromhex("07 ff6a 7fff 0000" + " 7fff" * 5)
    assert software_sensor_report(AQUAERO, {}) == bytes.fromhex("07" + " 7fff" * 8)


def test_software_sensor_report_rejects_what_it_cannot_carry() -> None:
    for number in (0, 9, True, 1.0, "1"):
        with pytest.raises(ValueError, match="software sensor number"):
            software_sensor_report(AQUAERO, {number: 20.0})
    for value in (TEMP_MIN_C - 0.01, TEMP_MAX_C + 0.01, float("nan"), float("inf")):
        with pytest.raises(ValueError, match="can carry"):
            software_sensor_report(AQUAERO, {1: value})
    with pytest.raises(ValueError, match="must be a number"):
        software_sensor_report(AQUAERO, {1: "20"})
    # The top value is "no data", so it is not a temperature the report can carry.
    assert round(TEMP_MAX_C * 100) == SENSOR_NOT_CONNECTED - 1
    with pytest.raises(ValueError, match="software-sensor report is not known"):
        software_sensor_report(QUADRO, {1: 20.0})  # the Quadro's report is unknown


def test_the_software_sensor_report_writes_where_the_status_report_reads_soft() -> None:
    """The slot a value lands in is the one the status report shows as softN."""
    soft = next(group for group in AQUAERO.temp_groups if group.prefix == "soft")
    assert (soft.count, AQUAERO.soft_sensor_count) == (8, 8)
    assert AQUAERO.soft_sensor_report_id == SOFT_SENSOR_REPORT_ID
    assert (QUADRO.soft_sensor_report_id, QUADRO.soft_sensor_count) == (None, None)
    status = bytearray(_bin("aquaero-status.bin"))
    report = software_sensor_report(AQUAERO, {2: 33.33})
    status[soft.offset + 2 : soft.offset + 4] = report[3:5]
    assert decode_status(AQUAERO, bytes(status)).temp("soft2") == pytest.approx(33.33)


@pytest.mark.parametrize(
    "name",
    [
        "aquaero-ctrl-firmware.bin",
        "aquaero-ctrl-after-writes.bin",
        "aquaero-ctrl-aquabus-before-fan7-write.bin",
    ],
)
def test_every_captured_aquaero_control_report_runs_profile_1(name: str) -> None:
    """Byte 0x06 is the active profile, 0-based; it read 0 in every capture taken
    before the owner configured profiles (PROJECT.md section 8 item 84)."""
    data = _bin(name)
    assert AQUAERO.profile_offset is not None and data[AQUAERO.profile_offset] == 0
    assert active_profile(AQUAERO, data) == 1


def test_the_profile_byte_is_read_where_the_kind_has_one() -> None:
    data = bytearray(_bin("aquaero-ctrl-firmware.bin"))
    assert AQUAERO.profile_offset == 0x06
    for raw in range(4):
        data[0x06] = raw
        assert active_profile(AQUAERO, bytes(data)) == raw + 1
    assert QUADRO.profile_offset is None
    assert active_profile(QUADRO, _bin("quadro-ctrl-firmware.bin")) is None
