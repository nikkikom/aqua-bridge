"""Byte sequences captured on the board's 1-Wire netlink socket, and fakes for it.

Every sequence in :data:`CAPTURED` came off the owner's Pi (kernel
6.18.50+rpt-rpi-v8), recorded by wrapping the transport's own socket -- request
and reply both -- while it listed the bus masters, ran a bus-wide Convert T and
read a scratchpad. One edit: the 8-byte slave identifier is replaced by the
placeholder for ROM ``28-000000000001``, because a real ROM id is a hardware
identifier and this repository is public. The scratchpad's own nine bytes are
not an identifier and are kept exactly as the sensor produced them (22.1875 C at
12 bit, CRC valid), as is the ``W1_LIST_MASTERS`` reply with the uninitialised
kernel bytes in the two fields the kernel never sets there.

:class:`FakeSocket` is what keeps the suite offline: the transport takes a socket
factory, so tests drive the real framing over prepared bytes.
``tests/conftest.py`` removes the netlink family from the module as well, so even
the unguarded path cannot open one.
"""

from __future__ import annotations

import struct

from aqua_bridge.hw.w1_netlink import W1Netlink
from aqua_bridge.hw.w1_therm_netlink import crc8, reg_num_from_rom_name

#: ROM id used in place of the owner's (see the module docstring).
PLACEHOLDER_ROM = "28-000000000001"
PLACEHOLDER_ID = reg_num_from_rom_name(PLACEHOLDER_ROM)

CAPTURED: dict[str, bytes] = {
    # seq 1: W1_LIST_MASTERS, no command records at all.
    "list_masters_request": bytes.fromhex(
        "30000000030000000100000000000000"
        "030000000100000001000000000000000c000000"
        "060000000000000000000000"
    ),
    # Its reply: ack = seq + 1, master ids 2 and 1 as bare u32 at msg->data.
    # "315f" (cn_msg.flags) and "6d6173746572320 0" (the id field) are
    # uninitialised kernel bytes: w1_process_command_root kmallocs its page.
    "list_masters_reply": bytes.fromhex(
        "38000000030000000100000000000000"
        "030000000100000001000000020000001400315f"
        "060008006d617374657232000200000001000000"
    ),
    # seq 2: W1_MASTER_CMD on master id 1, W1_CMD_RESET then W1_CMD_WRITE cc 44
    # (Skip ROM, Convert T).
    "convert_request": bytes.fromhex(
        "3a000000030000000200000000000000"
        "030000000100000002000000000000001600000004000a000100000000000000"
        "05000000"
        "01000200cc44"
    ),
    "convert_status_reset": bytes.fromhex(
        "34000000030000000200000000000000"
        "030000000100000002000000000000001000000004000400010000000000000005000000"
    ),
    "convert_status_write": bytes.fromhex(
        "34000000030000000200000000000000"
        "030000000100000002000000000000001000000004000400010000000000000001000000"
    ),
    # seq 3: W1_SLAVE_CMD, W1_CMD_WRITE be (Read Scratchpad) then W1_CMD_READ of
    # nine bytes -- sent as nine bytes of space for the kernel to fill.
    "scratchpad_request": bytes.fromhex(
        "42000000030000000300000000000000"
        "030000000100000003000000000000001e00000005001200" + PLACEHOLDER_ID.hex() + ""
        "01000100be"
        "00000900000000000000000000"
    ),
    "scratchpad_status_write": bytes.fromhex(
        "34000000030000000300000000000000"
        "03000000010000000300000000000000100000000500040" + "0" + PLACEHOLDER_ID.hex() + "01000000"
    ),
    # The data record: ack = seq + 1, and the nine bytes the sensor returned.
    "scratchpad_data": bytes.fromhex(
        "3d000000030000000300000000000000"
        "03000000010000000300000004000000190000000500" + "0d00" + PLACEHOLDER_ID.hex() + ""
        "00000900630100007fff7f0057"
        "000000"
    ),
    "scratchpad_status_read": bytes.fromhex(
        "34000000030000000300000000000000"
        "030000000100000003000000000000001000000005000400" + PLACEHOLDER_ID.hex() + "00000000"
    ),
}

#: The nine bytes of the captured scratchpad: 22.1875 C, 12 bit, CRC 0x57.
CAPTURED_SCRATCHPAD = bytes.fromhex("630100007fff7f0057")


def restamp(datagram: bytes, seq: int) -> bytes:
    """The same datagram with a new sequence number.

    A fresh transport numbers its first request 1, so a captured reply to
    request 3 has to be re-stamped to be a reply to the request a test actually
    makes. Both places the number appears are rewritten -- the netlink header
    and the ``cn_msg`` -- and a ``cn_msg.ack`` that was ``seq + 1`` (a data
    record) follows it, while an ack of 0 (a status record) stays 0.
    """
    old_seq = struct.unpack_from("=I", datagram, 8)[0]
    out = bytearray(datagram)
    struct.pack_into("=I", out, 8, seq)
    struct.pack_into("=I", out, 16 + 8, seq)
    old_ack = struct.unpack_from("=I", datagram, 16 + 12)[0]
    if old_ack == old_seq + 1:
        struct.pack_into("=I", out, 16 + 12, seq + 1)
    return bytes(out)


class FakeSocket:
    """A socket that hands back prepared datagrams, and records what it was told."""

    def __init__(self, replies: list[bytes | Exception] | None = None) -> None:
        self.replies: list[bytes | Exception] = list(replies or [])
        self.sent: list[bytes] = []
        self.timeouts: list[float] = []
        self.bound: tuple[int, int] | None = None
        self.closed = False

    def bind(self, address: tuple[int, int]) -> None:
        self.bound = address

    def settimeout(self, timeout: float) -> None:
        self.timeouts.append(timeout)

    def sendto(self, data: bytes, address: tuple[int, int]) -> int:
        self.sent.append(bytes(data))
        return len(data)

    def recv(self, size: int) -> bytes:
        if not self.replies:
            raise TimeoutError("fake socket: nothing more to hand back")
        item = self.replies.pop(0)
        if isinstance(item, Exception):
            raise item
        return item

    def close(self) -> None:
        self.closed = True


class StepClock:
    """Monotonic seconds that advance a fixed step per call, so no test sleeps."""

    def __init__(self, step: float = 0.25, start: float = 1000.0) -> None:
        self.t = start
        self.step = step

    def __call__(self) -> float:
        now = self.t
        self.t += self.step
        return now


def fake_transport(
    replies: list[bytes | Exception] | None = None, **kwargs: object
) -> tuple[W1Netlink, FakeSocket]:
    """An open transport over a :class:`FakeSocket`, and that socket."""
    sock = FakeSocket(replies)
    transport = W1Netlink(
        timeout_s=kwargs.pop("timeout_s", 1.0),  # type: ignore[arg-type]
        socket_factory=lambda *a, **kw: sock,
        **kwargs,  # type: ignore[arg-type]
    )
    transport.open()
    return transport, sock


def list_masters_reply(seq: int, ids: tuple[int, ...]) -> bytes:
    """A ``W1_LIST_MASTERS`` reply for ``ids``, shaped like the captured one.

    The bare ``u32`` ids go at ``w1_netlink_msg.data`` with no command record,
    and the two fields ``w1_process_command_root`` leaves uninitialised are
    filled here with the bytes the board happened to send, so a parser that
    reads them fails these tests too.
    """
    body = b"".join(struct.pack("=I", i) for i in ids)
    junk_target = bytes.fromhex("6d61737465723200")
    msg = struct.pack("=BBH8s", 6, 0, len(body), junk_target) + body
    cn = struct.pack("=IIIIHH", 3, 1, seq, seq + 1, len(msg), 0x5F31) + msg
    return struct.pack("=IHHII", 16 + len(cn), 3, 0, seq, 0) + cn


def convert_exchange(seq: int) -> list[bytes]:
    """The two status datagrams a bus-wide Convert T is answered with."""
    return [
        restamp(CAPTURED["convert_status_reset"], seq),
        restamp(CAPTURED["convert_status_write"], seq),
    ]


def scratchpad_exchange(seq: int, slave_id: bytes, scratchpad: bytes) -> list[bytes]:
    """The three datagrams one scratchpad read is answered with.

    The captured datagrams with this slave's identifier and these nine bytes
    patched in, so a test can give each sensor its own reading (and its own
    corruption) without hand-assembling a packet.
    """
    out = []
    for name in ("scratchpad_status_write", "scratchpad_data", "scratchpad_status_read"):
        raw = bytearray(restamp(CAPTURED[name], seq))
        raw[40:48] = slave_id  # w1_netlink_msg.id
        if name == "scratchpad_data":
            raw[52 : 52 + len(scratchpad)] = scratchpad  # the W1_CMD_READ payload
        out.append(bytes(raw))
    return out


def scratchpad(temp_raw: int, config: int = 0x7F, *, crc: int | None = None) -> bytes:
    """A DS18B20 scratchpad with a valid CRC unless ``crc`` overrides it.

    ``temp_raw`` is the signed 16-bit value the sensor stores (LSB 2^-4 C at 9 to
    12 bit), ``config`` its config register (0x7F is 12 bit).
    """
    body = temp_raw.to_bytes(2, "little", signed=True) + bytes(
        [0x4B, 0x46, config, 0xFF, 0x0C, 0x10]
    )
    return body + bytes([crc8(body) if crc is None else crc])
