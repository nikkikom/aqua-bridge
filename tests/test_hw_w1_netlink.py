"""Tests for :mod:`aqua_bridge.hw.w1_netlink`: the framing, against captured bytes.

The bytes, where they came from and what was edited out of them: see
``tests/w1_netlink_fakes.py``. **Nothing here opens a socket.**
"""

from __future__ import annotations

import errno
import struct

import pytest

from aqua_bridge.hw.w1_netlink import (
    CN_W1_IDX,
    CN_W1_VAL,
    W1_LIST_MASTERS,
    W1_MASTER_CMD,
    W1_SLAVE_CMD,
    W1Netlink,
    W1NetlinkProtocolError,
    W1NetlinkStatusError,
    W1NetlinkTimeout,
    W1NetlinkUnavailable,
    master_target,
    pack_request,
    parse_datagram,
    read_command,
    reset_command,
    write_command,
)
from aqua_bridge.hw.w1_therm_netlink import crc8
from w1_netlink_fakes import (
    CAPTURED,
    CAPTURED_SCRATCHPAD,
    PLACEHOLDER_ID,
    FakeSocket,
    StepClock,
    fake_transport,
    restamp,
)

# -- framing, against the captured requests ---------------------------------------


def test_the_list_masters_request_is_the_captured_one() -> None:
    """A master listing carries no command record at all, and the kernel takes it:
    ``W1_LIST_MASTERS`` is handled before the "no command" check in
    ``w1_cn_callback``."""
    assert (
        pack_request(msg_type=W1_LIST_MASTERS, target=bytes(8), commands=(), seq=1)
        == CAPTURED["list_masters_request"]
    )


def test_the_bus_wide_convert_request_is_the_captured_one() -> None:
    """Reset then Skip ROM + Convert T on a master addressed by id."""
    built = pack_request(
        msg_type=W1_MASTER_CMD,
        target=master_target(1),
        commands=(reset_command(), write_command(bytes([0xCC, 0x44]))),
        seq=2,
    )
    assert built == CAPTURED["convert_request"]


def test_the_scratchpad_request_is_the_captured_one() -> None:
    """Read Scratchpad, then a read of nine bytes -- which go on the wire as nine
    bytes of space: the kernel reads into the request's own buffer and hands that
    buffer back, and rejects a command whose length runs past the message."""
    built = pack_request(
        msg_type=W1_SLAVE_CMD,
        target=PLACEHOLDER_ID,
        commands=(write_command(bytes([0xBE])), read_command(9)),
        seq=3,
    )
    assert built == CAPTURED["scratchpad_request"]
    # The nine reserved bytes are really there, and really zero.
    assert built.endswith(bytes(9))


def test_the_lengths_in_a_request_agree_with_each_other() -> None:
    """``cn_call_callback`` drops a packet whose netlink length is shorter than the
    ``cn_msg`` it claims, so the three nested lengths have to agree."""
    built = pack_request(
        msg_type=W1_SLAVE_CMD,
        target=PLACEHOLDER_ID,
        commands=(write_command(b"\xbe"), read_command(9)),
        seq=7,
    )
    nl_len = struct.unpack_from("=I", built, 0)[0]
    idx, val, seq, ack, cn_len, flags = struct.unpack_from("=IIIIHH", built, 16)
    msg_type, status, msg_len = struct.unpack_from("=BBH", built, 36)
    assert nl_len == len(built)
    assert (idx, val) == (CN_W1_IDX, CN_W1_VAL)
    assert (seq, ack, flags) == (7, 0, 0)
    assert cn_len == nl_len - 16 - 20
    assert msg_len == cn_len - 12
    assert (msg_type, status) == (W1_SLAVE_CMD, 0)


def test_a_request_refuses_a_target_that_is_not_eight_bytes() -> None:
    with pytest.raises(ValueError, match="8 bytes"):
        pack_request(msg_type=W1_SLAVE_CMD, target=b"\x28", commands=(reset_command(),), seq=1)


@pytest.mark.parametrize("bad", [0, -1])
def test_a_read_of_no_bytes_is_refused(bad: int) -> None:
    with pytest.raises(ValueError, match="positive count"):
        read_command(bad)


# -- framing, against the captured replies ----------------------------------------


def test_the_master_list_is_read_from_the_record_body_not_from_commands() -> None:
    """A ``W1_LIST_MASTERS`` reply puts bare u32 ids at ``msg->data``. Read as
    command records instead, id 2 would decode as a command number and the ids
    would be lost -- which is exactly what happened before this test existed."""
    (reply,) = parse_datagram(CAPTURED["list_masters_reply"])
    assert reply.msg_type == W1_LIST_MASTERS
    assert reply.status == 0
    assert reply.ack == reply.seq + 1 and not reply.is_status
    assert reply.commands == ()
    assert reply.payload == bytes.fromhex("0200000001000000")


def test_a_status_record_is_the_requests_ack_and_a_data_record_is_seq_plus_one() -> None:
    """How the kernel marks the two (``w1_netlink_queue_status`` versus
    ``w1_netlink_queue_cmd``), and the only thing the transport uses to tell a
    per-command status from read data."""
    (status,) = parse_datagram(CAPTURED["scratchpad_status_read"])
    (data,) = parse_datagram(CAPTURED["scratchpad_data"])
    assert status.is_status and status.ack == 0 and status.status == 0
    assert status.commands == ((0, b""),)  # the command echoed back with no payload
    assert not data.is_status and data.ack == data.seq + 1
    assert data.commands == ((0, CAPTURED_SCRATCHPAD),)
    assert data.data() == CAPTURED_SCRATCHPAD
    assert crc8(CAPTURED_SCRATCHPAD[:8]) == CAPTURED_SCRATCHPAD[8]


def test_a_reply_record_carries_the_target_it_was_asked_about() -> None:
    (data,) = parse_datagram(CAPTURED["scratchpad_data"])
    assert data.target == PLACEHOLDER_ID


def test_two_records_bundled_into_one_datagram_are_both_read() -> None:
    """``cn_netlink_send_mult`` can put several ``cn_msg`` records in one skb, so
    the parser walks them by their own lengths. Built here by concatenating two
    captured status payloads under one netlink header, which is the shape the
    kernel sends when the caller asks for ``W1_CN_BUNDLE``."""
    first = CAPTURED["convert_status_reset"]
    second = CAPTURED["convert_status_write"]
    body = first[16:] + second[16:]
    datagram = struct.pack("=IHHII", 16 + len(body), 3, 0, 2, 0) + body
    replies = parse_datagram(datagram)
    assert [r.commands[0][0] for r in replies] == [5, 1]  # W1_CMD_RESET, W1_CMD_WRITE
    assert all(r.is_status and r.status == 0 for r in replies)


@pytest.mark.parametrize(
    "datagram",
    [
        pytest.param(CAPTURED["scratchpad_data"][:-20], id="netlink header outruns the buffer"),
        pytest.param(
            CAPTURED["scratchpad_data"][:16]
            + struct.pack("=IIIIHH", CN_W1_IDX, CN_W1_VAL, 3, 4, 0xFFF0, 0),
            id="cn_msg outruns the datagram",
        ),
        pytest.param(
            struct.pack("=IHHII", 16 + 20 + 12, 3, 0, 3, 0)
            + struct.pack("=IIIIHH", CN_W1_IDX, CN_W1_VAL, 3, 4, 12, 0)
            + struct.pack("=BBH8s", W1_SLAVE_CMD, 0, 0xFF, bytes(8)),
            id="w1_netlink_msg outruns its cn_msg",
        ),
    ],
)
def test_a_truncated_datagram_is_a_protocol_error(datagram: bytes) -> None:
    """Better an exception the caller can act on than a silently short reading."""
    with pytest.raises(W1NetlinkProtocolError):
        parse_datagram(datagram)


def test_a_netlink_level_error_is_a_protocol_error() -> None:
    """NLMSG_ERROR means netlink itself refused the packet; there is no w1 record in it."""
    datagram = struct.pack("=IHHII", 16 + 4, 2, 0, 1, 0) + struct.pack("=i", -errno.EINVAL)
    with pytest.raises(W1NetlinkProtocolError, match="refused"):
        parse_datagram(datagram)


def test_a_connector_message_for_somebody_else_is_ignored() -> None:
    """Another connector user's id on this socket is not our reply and not an error."""
    body = struct.pack("=IIIIHH", 0x9, 0x1, 1, 0, 12, 0) + struct.pack(
        "=BBH8s", W1_SLAVE_CMD, 0, 0, bytes(8)
    )
    datagram = struct.pack("=IHHII", 16 + len(body), 3, 0, 1, 0) + body
    assert parse_datagram(datagram) == []


# -- the socket: requests, bounds and failures -------------------------------------


def test_the_captured_session_replays_through_the_transport_in_order() -> None:
    """One transport, the three exchanges the board actually had, in the order it
    had them -- so the sequence numbers line up with the capture without editing."""
    transport, sock = fake_transport(
        [
            CAPTURED["list_masters_reply"],
            CAPTURED["convert_status_reset"],
            CAPTURED["convert_status_write"],
            CAPTURED["scratchpad_status_write"],
            CAPTURED["scratchpad_data"],
            CAPTURED["scratchpad_status_read"],
        ]
    )
    assert transport.list_masters() == (2, 1)
    transport.master_command(1, (reset_command(), write_command(bytes([0xCC, 0x44]))))
    replies = transport.slave_command(PLACEHOLDER_ID, (write_command(b"\xbe"), read_command(9)))
    assert sock.sent == [
        CAPTURED["list_masters_request"],
        CAPTURED["convert_request"],
        CAPTURED["scratchpad_request"],
    ]
    assert b"".join(r.data() for r in replies if not r.is_status) == CAPTURED_SCRATCHPAD
    assert sock.bound == (0, 0), "groups=0, and a port id the kernel picks"


def test_a_master_that_answers_nothing_times_out_inside_the_bound() -> None:
    """The failure a kernel with no w1 connector gives: the request is dropped in
    ``cn_rx_skb`` and nothing ever answers. Every wait is bounded, so this raises
    instead of hanging the reader thread."""
    transport, sock = fake_transport([], timeout_s=0.5)
    with pytest.raises(W1NetlinkTimeout, match="within 0.5 s"):
        transport.master_command(2, (reset_command(),))
    assert sock.timeouts and all(0 < t <= 0.5 for t in sock.timeouts)


def test_a_reply_that_stops_halfway_times_out() -> None:
    """Two commands, one status back, then silence: the request is not complete and
    the caller is told so rather than being handed half an answer."""
    transport, _sock = fake_transport([restamp(CAPTURED["convert_status_reset"], 1)], timeout_s=0.5)
    with pytest.raises(W1NetlinkTimeout, match=r"1/2 status"):
        transport.master_command(1, (reset_command(), write_command(bytes([0xCC, 0x44]))))


def test_replies_for_another_request_do_not_hold_the_deadline_open() -> None:
    """A late reply to a request that already timed out is dropped, and dropping it
    does not extend this request's budget: the clock, not the traffic, ends the wait."""
    stale = restamp(CAPTURED["convert_status_reset"], 99)
    transport, _sock = fake_transport([stale] * 40, timeout_s=0.5, clock=StepClock(step=0.2))
    with pytest.raises(W1NetlinkTimeout, match=r"0/1 status"):
        transport.master_command(1, (reset_command(),))


def test_a_nonzero_status_raises_with_the_errno_in_it() -> None:
    """An id nothing answers is ``-ENODEV``, which the kernel reports as
    ``(u8)-error``. Measured on the board with both a master id and a ROM id that
    do not exist."""
    enodev = bytearray(restamp(CAPTURED["convert_status_reset"], 1))
    enodev[16 + 20 + 1] = errno.ENODEV  # w1_netlink_msg.status
    transport, _sock = fake_transport([bytes(enodev)])
    with pytest.raises(W1NetlinkStatusError) as caught:
        transport.master_command(1, (reset_command(),))
    assert caught.value.status == errno.ENODEV
    assert "ENODEV" in str(caught.value)


def test_a_reset_with_no_presence_pulse_is_status_255() -> None:
    """``w1_reset_bus`` returns 1, not an errno, and the kernel sends ``(u8)-1``.
    It still has to become an exception and not a reading."""
    empty_bus = bytearray(restamp(CAPTURED["convert_status_reset"], 1))
    empty_bus[16 + 20 + 1] = 255
    transport, _sock = fake_transport([bytes(empty_bus)])
    with pytest.raises(W1NetlinkStatusError) as caught:
        transport.master_command(1, (reset_command(),))
    assert caught.value.status == 255


def test_a_request_before_open_is_refused() -> None:
    transport = W1Netlink(timeout_s=1.0, socket_factory=lambda *a, **kw: FakeSocket())
    with pytest.raises(W1NetlinkUnavailable, match="not open"):
        transport.master_command(1, (reset_command(),))


def test_a_socket_that_cannot_be_opened_or_bound_is_unavailable() -> None:
    def refuse(*_a: object, **_kw: object) -> FakeSocket:
        raise OSError(errno.EPROTONOSUPPORT, "no connector in this kernel")

    with pytest.raises(W1NetlinkUnavailable, match="cannot open"):
        W1Netlink(timeout_s=1.0, socket_factory=refuse).open()

    class _RefusingBind(FakeSocket):
        def bind(self, address: tuple[int, int]) -> None:
            raise OSError(errno.EPERM, "CAP_NET_ADMIN")

    sock = _RefusingBind()
    with pytest.raises(W1NetlinkUnavailable, match="cannot bind"):
        W1Netlink(timeout_s=1.0, socket_factory=lambda *a, **kw: sock).open()
    assert sock.closed, "a socket that could not be bound is not left open"


def test_with_no_netlink_family_there_is_nothing_to_open() -> None:
    """What every test in this suite runs with (``tests/conftest.py``), and what a
    kernel without the connector looks like from here."""
    with pytest.raises(W1NetlinkUnavailable, match="AF_NETLINK"):
        W1Netlink(timeout_s=1.0).open()


def test_open_is_idempotent_and_close_is_safe_twice() -> None:
    sock = FakeSocket()
    transport = W1Netlink(timeout_s=1.0, socket_factory=lambda *a, **kw: sock)
    transport.open()
    transport.open()
    assert transport.is_open
    transport.close()
    transport.close()
    assert not transport.is_open and sock.closed


def test_the_context_manager_closes_the_socket_even_on_a_failure() -> None:
    sock = FakeSocket()
    with pytest.raises(W1NetlinkTimeout):  # noqa: SIM117 - the nesting is the point
        with W1Netlink(timeout_s=0.1, socket_factory=lambda *a, **kw: sock) as transport:
            transport.master_command(2, (reset_command(),))
    assert sock.closed


def test_sequence_numbers_advance_and_never_collide_with_a_data_ack() -> None:
    """A status record is told from read data by ``ack`` alone, so ``seq + 1`` must
    never be able to wrap onto the request's own ack of 0."""
    sock = FakeSocket()
    transport = W1Netlink(timeout_s=0.01, socket_factory=lambda *a, **kw: sock)
    transport.open()
    seen = []
    for _ in range(4):
        with pytest.raises(W1NetlinkTimeout):
            transport.master_command(1, (reset_command(),))
        seen.append(struct.unpack_from("=I", sock.sent[-1], 16 + 8)[0])
    assert seen == [1, 2, 3, 4]
    transport._seq = 0xFFFFFFFE  # the last value before the wrap
    for _ in range(2):
        with pytest.raises(W1NetlinkTimeout):
            transport.master_command(1, (reset_command(),))
    wrapped = struct.unpack_from("=I", sock.sent[-1], 16 + 8)[0]
    assert wrapped == 1, "the wrap skips 0 and 0xffffffff"
