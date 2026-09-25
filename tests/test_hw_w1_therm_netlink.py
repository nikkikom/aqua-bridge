"""Tests for :mod:`aqua_bridge.hw.w1_therm_netlink`: the CRC, the decode, one cycle.

The driver no longer checks a scratchpad's CRC for us (a netlink read is raw bus
bytes), so most of this file is about what a scratchpad has to survive before it
becomes a temperature. Captured bytes and the fake socket come from
``tests/w1_netlink_fakes.py``; nothing here opens a socket and nothing sleeps.
"""

from __future__ import annotations

import errno

import pytest

from aqua_bridge.hw.w1_netlink import (
    W1Command,
    W1NetlinkStatusError,
    W1NetlinkTimeout,
    W1Reply,
)
from aqua_bridge.hw.w1_therm_netlink import (
    CONVERSION_TIME_S,
    SCRATCHPAD_BYTES,
    ScratchpadCrcError,
    ScratchpadError,
    W1Therm,
    crc8,
    decode_scratchpad,
    reg_num_from_rom_name,
    rom_name_from_reg_num,
)
from w1_netlink_fakes import (
    CAPTURED,
    CAPTURED_SCRATCHPAD,
    PLACEHOLDER_ID,
    PLACEHOLDER_ROM,
    fake_transport,
    restamp,
    scratchpad,
)

# -- the CRC-8 ---------------------------------------------------------------------


@pytest.mark.parametrize(
    "rom", ["28-000000000001", "28-00000000000a", "28-0000000000ff", "10-0123456789ab"]
)
def test_the_rom_checksum_is_the_crc_of_the_seven_bytes_before_it(rom: str) -> None:
    """The same CRC-8 that covers a scratchpad covers a ROM id's last byte, which is
    how an identifier can be rebuilt from a sysfs directory name at all. Checked
    against the owner's three sensors on the board, whose stored CRCs it
    reproduces; here against placeholders, since a real ROM id is a hardware
    identifier."""
    reg_num = reg_num_from_rom_name(rom)
    assert len(reg_num) == 8
    assert crc8(reg_num[:7]) == reg_num[7]
    assert rom_name_from_reg_num(reg_num) == rom


@pytest.mark.parametrize("bad", ["28-00000000001", "28000000000001", "zz-000000000001", "", "28-"])
def test_a_name_that_is_not_a_rom_id_is_refused(bad: str) -> None:
    with pytest.raises(ValueError, match="ROM name"):
        reg_num_from_rom_name(bad)


def test_the_captured_scratchpad_passes_its_own_crc() -> None:
    assert crc8(CAPTURED_SCRATCHPAD[:8]) == CAPTURED_SCRATCHPAD[8]
    assert crc8(CAPTURED_SCRATCHPAD) == 0  # a full block including its CRC comes to zero


@pytest.mark.parametrize("position", range(SCRATCHPAD_BYTES))
def test_one_flipped_bit_anywhere_in_a_scratchpad_is_caught(position: int) -> None:
    """Every byte, the stored CRC included: none of them may pass unnoticed."""
    corrupt = bytearray(CAPTURED_SCRATCHPAD)
    corrupt[position] ^= 0x01
    with pytest.raises(ScratchpadCrcError, match="CRC-8"):
        decode_scratchpad(bytes(corrupt))


def test_an_all_zero_scratchpad_is_refused_although_it_passes_the_crc() -> None:
    """The one checksum-valid scratchpad that is not a reading. CRC-8 of eight zero
    bytes is zero, so nine zeros pass -- and on the board, reading one of the
    family-00 phantoms of the unterminated bus returns exactly nine zeros, which
    would decode to a plausible 0.0 C and quietly ask for less cooling."""
    zeros = bytes(SCRATCHPAD_BYTES)
    assert crc8(zeros[:8]) == zeros[8]
    with pytest.raises(ScratchpadError, match="all-zero"):
        decode_scratchpad(zeros)


@pytest.mark.parametrize("length", [0, 1, 8, 10])
def test_a_scratchpad_of_the_wrong_length_is_refused(length: int) -> None:
    with pytest.raises(ScratchpadError, match="9 bytes"):
        decode_scratchpad(bytes(length))


# -- the decode --------------------------------------------------------------------


@pytest.mark.parametrize(
    ("raw", "config", "bits", "expected"),
    [
        pytest.param(0x0163, 0x7F, 12, 22.1875, id="12 bit, the board's own reading"),
        pytest.param(0x07D0, 0x7F, 12, 125.0, id="12 bit, the datasheet's maximum"),
        pytest.param(-0x0370, 0x7F, 12, -55.0, id="12 bit, the datasheet's minimum"),
        pytest.param(-0x0001, 0x7F, 12, -0.0625, id="12 bit, one LSB below zero"),
        pytest.param(0x0191, 0x5F, 11, 25.0, id="11 bit, undefined bit 0 set"),
        pytest.param(0x0193, 0x3F, 10, 25.0, id="10 bit, undefined bits 1:0 set"),
        pytest.param(0x0197, 0x1F, 9, 25.0, id="9 bit, undefined bits 2:0 set"),
        pytest.param(0x0550, 0x7F, 12, 85.0, id="the power-on value of a sensor never converted"),
    ],
)
def test_a_reading_follows_the_resolution_the_sensor_reports(
    raw: int, config: int, bits: int, expected: float
) -> None:
    """The resolution comes from the sensor's own config register, not from what
    config asked for, and the bits the datasheet leaves undefined below 12 bit are
    masked off (a real DS18B20 zeroes them, so the value is the same either way).
    85.0 C is a real value in the ABI -- a sensor that has never finished a
    conversion -- and it errs toward cooling, so it is passed through."""
    reading = decode_scratchpad(scratchpad(raw, config))
    assert reading.temperature_c == pytest.approx(expected)
    assert reading.resolution_bits == bits
    assert reading.config == config
    assert reading.conversion_s == CONVERSION_TIME_S[bits]


@pytest.mark.parametrize(
    ("raw", "config", "bits", "expected"),
    [
        pytest.param(0x0190, 0x80, 13, 25.0, id="13 bit"),
        pytest.param(0x0190, 0xA0, 14, 25.0, id="14 bit"),
        pytest.param(0x0190, 0xA3, 14, 25.046875, id="14 bit, the extra bits in the config"),
    ],
)
def test_a_clone_in_thirteen_or_fourteen_bit_mode_decodes_by_its_own_rule(
    raw: int, config: int, bits: int, expected: float
) -> None:
    """Config bit R2 means a GX20MH01, whose two lowest temperature bits live in the
    config register and whose LSB is 2^-6 C. Decoded with the plain rule a reading
    like this would be four times too large; the branch mirrors
    ``w1_DS18B20_convert_temp()``."""
    reading = decode_scratchpad(scratchpad(raw, config))
    assert reading.resolution_bits == bits
    assert reading.temperature_c == pytest.approx(expected)


# -- one cycle over a fake transport -----------------------------------------------


class _FakeTransport:
    """The transport's surface, with scripted answers per slave.

    ``answers`` maps a slave identifier to a scratchpad, or to an exception to
    raise for it. ``master`` is raised, if set, instead of answering the
    conversion command.
    """

    def __init__(
        self,
        answers: dict[bytes, bytes | Exception],
        *,
        master: Exception | None = None,
    ) -> None:
        self.answers = answers
        self.master = master
        self.master_calls: list[tuple[int, tuple[W1Command, ...]]] = []
        self.slave_calls: list[bytes] = []

    def master_command(
        self, master_id: int, commands: tuple[W1Command, ...], *, timeout_s: float | None = None
    ) -> tuple[W1Reply, ...]:
        self.master_calls.append((master_id, tuple(commands)))
        if self.master is not None:
            raise self.master
        return ()

    def slave_command(
        self, slave_id: bytes, commands: tuple[W1Command, ...], *, timeout_s: float | None = None
    ) -> tuple[W1Reply, ...]:
        self.slave_calls.append(slave_id)
        answer = self.answers[slave_id]
        if isinstance(answer, Exception):
            raise answer
        return (
            W1Reply(
                seq=1,
                ack=2,
                msg_type=5,
                status=0,
                target=slave_id,
                payload=b"",
                commands=((0, answer),),
            ),
        )


class _Sleeps:
    def __init__(self) -> None:
        self.calls: list[float] = []

    def __call__(self, seconds: float) -> None:
        self.calls.append(seconds)


def _ids(count: int) -> dict[str, bytes]:
    return {
        f"28-{index:012x}": reg_num_from_rom_name(f"28-{index:012x}")
        for index in range(1, count + 1)
    }


def test_one_cycle_converts_once_for_the_whole_bus_then_reads_eachscratchpad() -> None:
    """The point of the tier: one conversion whatever the sensor count, and Skip ROM
    + Convert T is what makes it bus-wide. The wait is the caller's number and
    nothing sleeps for real."""
    targets = _ids(3)
    transport = _FakeTransport(dict.fromkeys(targets.values(), CAPTURED_SCRATCHPAD))
    sleeps = _Sleeps()
    read = W1Therm(transport, sleeper=sleeps).read_bus(2, targets, conversion_s=0.75)

    assert transport.master_calls == [(2, (W1Command(5), W1Command(1, bytes([0xCC, 0x44]))))], (
        "one W1_CMD_RESET and one W1_CMD_WRITE of Skip ROM, Convert T"
    )
    assert sleeps.calls == [0.75]
    assert transport.slave_calls == list(targets.values())
    assert read.failures == {} and read.crc_failures == ()
    assert not read.all_failed
    assert [r.temperature_c for r in read.readings.values()] == [pytest.approx(22.1875)] * 3


def test_a_sensor_with_a_bad_crc_loses_its_own_reading_and_no_one_elses() -> None:
    """A single lying sensor must not blind a zone that has others."""
    targets = _ids(3)
    ids = list(targets.values())
    answers: dict[bytes, bytes | Exception] = dict.fromkeys(ids, CAPTURED_SCRATCHPAD)
    answers[ids[1]] = CAPTURED_SCRATCHPAD[:8] + bytes([CAPTURED_SCRATCHPAD[8] ^ 0xFF])
    read = W1Therm(_FakeTransport(answers), sleeper=_Sleeps()).read_bus(
        2, targets, conversion_s=0.19
    )

    names = list(targets)
    assert read.readings[names[1]] is None
    assert read.crc_failures == (names[1],)
    assert "CRC-8" in read.failures[names[1]]
    assert read.readings[names[0]] is not None and read.readings[names[2]] is not None
    assert not read.all_failed


def test_a_sensor_that_does_not_answer_match_rom_is_one_sensors_failure() -> None:
    """A sensor pulled out between the search and the read: ``-ENODEV`` for it, a
    reading for everyone else. The owner does exactly this while the daemon runs."""
    targets = _ids(2)
    ids = list(targets.values())
    answers: dict[bytes, bytes | Exception] = {
        ids[0]: W1NetlinkStatusError(errno.ENODEV),
        ids[1]: CAPTURED_SCRATCHPAD,
    }
    read = W1Therm(_FakeTransport(answers), sleeper=_Sleeps()).read_bus(
        2, targets, conversion_s=0.19
    )
    assert read.readings[list(targets)[0]] is None
    assert "ENODEV" in read.failures[list(targets)[0]]
    assert read.crc_failures == ()  # not a CRC problem, and not counted as one
    assert read.readings[list(targets)[1]] is not None


def test_every_sensor_failing_is_reported_as_a_bus_symptom() -> None:
    """``all_failed`` is what the caller drops a tier on: one sensor failing is the
    sensor's, all of them failing is the bus's."""
    targets = _ids(2)
    answers: dict[bytes, bytes | Exception] = dict.fromkeys(
        targets.values(), bytes(SCRATCHPAD_BYTES)
    )
    read = W1Therm(_FakeTransport(answers), sleeper=_Sleeps()).read_bus(
        2, targets, conversion_s=0.19
    )
    assert read.all_failed
    assert set(read.crc_failures) == set(targets)


def test_a_reply_that_never_arrives_ends_the_cycle_instead_of_costing_every_sensor() -> None:
    """A timeout says nothing about *which* sensor is at fault, and charging it to
    each in turn would cost ``timeout_s`` per sensor before anyone noticed. So it
    propagates on the first one."""
    targets = _ids(12)
    ids = list(targets.values())
    answers: dict[bytes, bytes | Exception] = dict.fromkeys(ids, CAPTURED_SCRATCHPAD)
    answers[ids[0]] = W1NetlinkTimeout("no reply within 1.0 s")
    transport = _FakeTransport(answers)
    with pytest.raises(W1NetlinkTimeout):
        W1Therm(transport, sleeper=_Sleeps()).read_bus(2, targets, conversion_s=0.19)
    assert transport.slave_calls == [ids[0]], "it did not go on to the other eleven"


def test_a_conversion_that_fails_is_the_buss_failure_and_nothing_is_read() -> None:
    targets = _ids(2)
    transport = _FakeTransport(
        dict.fromkeys(targets.values(), CAPTURED_SCRATCHPAD),
        master=W1NetlinkStatusError(errno.ENODEV),
    )
    sleeps = _Sleeps()
    with pytest.raises(W1NetlinkStatusError):
        W1Therm(transport, sleeper=sleeps).read_bus(2, targets, conversion_s=0.19)
    assert transport.slave_calls == []
    assert sleeps.calls == [], "nothing waited for a conversion that never started"


def test_a_negative_conversion_time_is_refused() -> None:
    with pytest.raises(ValueError, match="conversion_s"):
        W1Therm(_FakeTransport({}), sleeper=_Sleeps()).read_bus(2, {}, conversion_s=-1.0)


def test_a_scratchpad_read_that_returns_too_few_bytes_is_refused() -> None:
    """The kernel filling less than nine bytes is not a reading to decode."""
    targets = _ids(1)
    answers: dict[bytes, bytes | Exception] = dict.fromkeys(targets.values(), b"\x01\x02")
    read = W1Therm(_FakeTransport(answers), sleeper=_Sleeps()).read_bus(
        2, targets, conversion_s=0.19
    )
    assert read.all_failed
    assert "2 bytes" in read.failures[list(targets)[0]]


def test_the_captured_exchange_decodes_to_the_temperature_the_board_reported() -> None:
    """End to end over the real transport and the real framing, on the bytes the
    board actually sent: 22.1875 C at 12 bit."""
    transport, sock = fake_transport(
        [
            restamp(CAPTURED["scratchpad_status_write"], 1),
            restamp(CAPTURED["scratchpad_data"], 1),
            restamp(CAPTURED["scratchpad_status_read"], 1),
        ]
    )
    reading = W1Therm(transport).read_temperature(PLACEHOLDER_ID)
    assert reading.temperature_c == pytest.approx(22.1875)
    assert reading.resolution_bits == 12
    assert sock.sent == [restamp(CAPTURED["scratchpad_request"], 1)], (
        f"the request for {PLACEHOLDER_ROM} is the captured one, at this transport's own seq"
    )
