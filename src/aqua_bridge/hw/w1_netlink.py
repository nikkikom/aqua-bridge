"""1-Wire transport over the kernel's netlink connector (``CN_W1_IDX``).

PROJECT.md section 3 (Track B), section 8 item 38 (why the sysfs bulk read is
not enough) and section 12 risk 5 ("bulk 1-Wire on a busy single core").

This module knows sockets and byte layouts. It knows nothing about
thermometers: :mod:`aqua_bridge.hw.w1_therm_netlink` puts DS18B20 commands
*into* it. Every call here is a request the kernel answers or a bounded wait
that raises -- nothing blocks for longer than the caller's timeout, because
this sits under a control loop that must never block (PROJECT.md section 3,
``step()`` and the reader threads).

Why a socket at all, when ``w1_therm`` already exposes a bulk read in sysfs
(section 8 item 38): that attribute is created on **one bus master
system-wide** (``bulk_read_device_counter`` in
``drivers/w1/slaves/w1_therm.c`` is a file-scope global, and the attribute is
added on the master of the first bulk-capable slave to attach anywhere), and
``trigger_bulk_read()`` refuses -- ``-ENODEV``, reported only in the kernel's
own log -- when *any* slave on that master has ``family_data == NULL``, which
is exactly what the family-``00`` phantoms of an unterminated bit-banged bus
are. Netlink addresses a master by its **id**, never walks a slave list and
never looks at ``family_data``, so neither defect applies. It needs no root
(below) and no kernel patch, and it takes the same ``dev->bus_mutex`` as the
sysfs paths, so the two coexist.

Sources read for this ABI -- the board's own kernel is 6.18.50+rpt-rpi-v8:

* ``/usr/include/linux/connector.h`` **on the board**: ``CN_W1_IDX = 0x3``,
  ``CN_W1_VAL = 0x1``, ``struct cn_msg``, ``CONNECTOR_MAX_MSG_SIZE = 16384``.
* ``drivers/w1/w1_netlink.h`` (raspberrypi/linux ``rpi-6.18.y``): the message
  types, the command numbers, ``struct w1_netlink_msg``, ``struct
  w1_netlink_cmd``.
* ``drivers/w1/w1_netlink.c`` (same tree): ``w1_cn_callback()`` (how a request
  is validated and split), ``w1_process_cb()`` (which takes ``bus_mutex`` and
  issues ``w1_reset_select_slave()`` itself for a slave command),
  ``w1_netlink_queue_cmd()`` / ``w1_netlink_queue_status()`` (the reply
  framing below), ``w1_process_command_root()`` (``W1_LIST_MASTERS``).
* ``drivers/connector/connector.c`` (same tree): ``cn_bind()``,
  ``cn_rx_skb()``, ``cn_netlink_send_mult()``.
* ``drivers/w1/w1_int.c`` and ``include/linux/w1.h`` (same tree): a master's
  sysfs name is ``w1_bus_master%u`` of ``dev->id`` -- the very id
  ``W1_LIST_MASTERS`` returns and a master command carries -- and
  ``struct w1_reg_num`` is the 8 raw bytes a slave's sysfs ``id`` attribute
  hands out (family, 48-bit id, CRC-8).

**No privilege is needed.** ``cn_bind()`` returns ``-EPERM`` to a process
without ``CAP_NET_ADMIN``, but the kernel calls it only for the multicast
groups in ``nl_groups``; binding with ``groups=0`` never reaches it. Requests
go in through ``cn_rx_skb()``, which checks sizes and nothing else, and the
reply is unicast back to this socket's port id. Verified as an ordinary user
on the board.

Wire format of one request (all little-endian on both the Pi and a dev
machine; ``struct`` format ``=`` so nothing is padded)::

    struct nlmsghdr   16 B  len, type=NLMSG_DONE, flags=0, seq, pid=0
    struct cn_msg     20 B  idx=CN_W1_IDX, val=CN_W1_VAL, seq, ack, len, flags
    struct w1_netlink_msg
                      12 B  type, status=0, len, id[8]
                            id[8] is {u32 master_id, u32 reserved} for
                            W1_MASTER_CMD and the slave's 8-byte reg_num for
                            W1_SLAVE_CMD
    struct w1_netlink_cmd
                       4 B  cmd, res=0, len            } one per command,
    payload          len B  the bytes of that command   } back to back

``cn_msg.len`` counts everything after itself, ``w1_netlink_msg.len``
everything after itself (all the commands), and ``nlmsghdr.len`` the whole
packet; ``cn_call_callback()`` drops a packet whose lengths disagree.
A ``W1_CMD_READ`` carries ``len`` **zero bytes of space**, not an empty
payload: the kernel reads into the request's own buffer and sends that buffer
back, and ``w1_process_cb()`` rejects (``-E2BIG``) a command whose declared
length runs past the message.

Reply framing, from ``w1_netlink_queue_cmd`` versus
``w1_netlink_queue_status``: every command produces one **status** record
carrying ``w1_netlink_msg.status`` (``(u8)-errno``, so 0 means success) with
``cn_msg.ack`` equal to the **request's ack**, and a command that returns data
(``W1_CMD_READ``, ``W1_CMD_TOUCH``, a search, ``W1_CMD_LIST_SLAVES``) also
produces a **data** record with ``cn_msg.ack == request seq + 1``. Both carry
the request's ``cn_msg.seq``, which is how :class:`W1Netlink` tells its own
replies from a late one belonging to a request that already timed out. So the
request's ack is 0 here and ``seq`` never takes the value that would make
``seq + 1`` wrap onto it. A ``W1_LIST_MASTERS`` reply is the one that carries
neither kind of record: bare ``u32`` master ids sit at ``w1_netlink_msg.data``
with no command header, and that record's ``id[8]`` and its ``cn_msg.flags``
hold whatever was in the page the kernel allocated
(``w1_process_command_root`` uses ``kmalloc``, not ``kzalloc``, and sets
neither), so nothing here may read those two fields. One datagram may hold
several ``cn_msg`` records
(``cn_netlink_send_mult`` sends one skb spanning all of them), so the parser
walks them by their own lengths rather than assuming one.

A failure the kernel reports *before* it reaches the bus -- an unknown master
id or slave id (``-ENODEV``), a message with no command in it (``-EPROTO``) --
comes back through ``w1_netlink_send_error()`` as a status record with no
command attached. It is the same exception as any other nonzero status.
"""

from __future__ import annotations

import errno as _errno
import logging
import os
import socket
import struct
import time
from collections.abc import Callable, Sequence
from dataclasses import dataclass

__all__ = [
    "CN_W1_IDX",
    "CN_W1_VAL",
    "CONNECTOR_MAX_MSG_SIZE",
    "NETLINK_CONNECTOR",
    "W1Command",
    "W1Netlink",
    "W1NetlinkError",
    "W1NetlinkProtocolError",
    "W1NetlinkStatusError",
    "W1NetlinkTimeout",
    "W1NetlinkUnavailable",
    "W1Reply",
    "master_target",
    "netlink_family",
    "pack_request",
    "parse_datagram",
    "read_command",
    "reset_command",
    "write_command",
]

_LOG = logging.getLogger("aqua_bridge.hw.w1_netlink")

#: ``NETLINK_CONNECTOR`` from ``include/uapi/linux/netlink.h``. Python's
#: :mod:`socket` does not define it.
NETLINK_CONNECTOR = 11
#: ``CN_W1_IDX`` / ``CN_W1_VAL`` from ``include/uapi/linux/connector.h``.
CN_W1_IDX = 0x3
CN_W1_VAL = 0x1
#: ``CONNECTOR_MAX_MSG_SIZE``: the largest ``cn_msg`` payload the kernel will
#: send or accept, and so the only receive buffer size that cannot truncate a
#: reply (a netlink datagram longer than the buffer loses its tail).
CONNECTOR_MAX_MSG_SIZE = 16384

_NLMSG_DONE = 3
_NLMSG_ERROR = 2

# enum w1_netlink_message_types (drivers/w1/w1_netlink.h).
W1_SLAVE_ADD = 0
W1_SLAVE_REMOVE = 1
W1_MASTER_ADD = 2
W1_MASTER_REMOVE = 3
W1_MASTER_CMD = 4
W1_SLAVE_CMD = 5
W1_LIST_MASTERS = 6

# enum w1_commands (drivers/w1/w1_netlink.h).
W1_CMD_READ = 0
W1_CMD_WRITE = 1
W1_CMD_SEARCH = 2
W1_CMD_ALARM_SEARCH = 3
W1_CMD_TOUCH = 4
W1_CMD_RESET = 5
W1_CMD_SLAVE_ADD = 6
W1_CMD_SLAVE_REMOVE = 7
W1_CMD_LIST_SLAVES = 8

#: Commands whose reply carries data back (``w1_process_command_io`` queues the
#: command itself for these, and only these, plus the searches).
_DATA_COMMANDS = frozenset(
    {W1_CMD_READ, W1_CMD_TOUCH, W1_CMD_SEARCH, W1_CMD_ALARM_SEARCH, W1_CMD_LIST_SLAVES}
)

_NLMSG_HDR = struct.Struct("=IHHII")  # len, type, flags, seq, pid
_CN_MSG = struct.Struct("=IIIIHH")  # idx, val, seq, ack, len, flags
_W1_MSG = struct.Struct("=BBH8s")  # type, status, len, id[8]
_W1_CMD = struct.Struct("=BBH")  # cmd, res, len
_W1_MST = struct.Struct("=II")  # master id, reserved
_NLMSG_HDRLEN = _NLMSG_HDR.size
_TARGET_LEN = 8

#: The request's ``cn_msg.ack``. Status replies echo it and data replies carry
#: ``seq + 1``, so the two are told apart by it (module docstring).
_REQUEST_ACK = 0
#: ``seq`` cycles through 1..0xFFFFFFFE: never 0, and never the value whose
#: ``seq + 1`` would wrap onto :data:`_REQUEST_ACK`.
_SEQ_MIN = 1
_SEQ_MAX = 0xFFFFFFFE

_AF_NETLINK = getattr(socket, "AF_NETLINK", None)
#: ``AF_NETLINK``'s value on Linux, for the one case where this module needs a
#: family number without :mod:`socket` naming one: a caller that injects its own
#: ``socket_factory`` (a test) on a platform that has no netlink.
_AF_NETLINK_LINUX = 16


def netlink_family() -> int | None:
    """``AF_NETLINK`` on this platform, or ``None`` where there is no netlink.

    Also the one seam the test suite patches so that it can never open a
    netlink socket (``tests/conftest.py::no_netlink_socket``): with no family
    there is nothing to open, which is the same
    :class:`W1NetlinkUnavailable` a kernel without the connector raises. A
    caller that injects ``socket_factory`` does not go through it.
    """
    return _AF_NETLINK


class W1NetlinkError(Exception):
    """Base for every failure of this transport. Callers act on these."""


class W1NetlinkUnavailable(W1NetlinkError):
    """No usable connector socket: not Linux, no ``CONFIG_CONNECTOR``, bind refused."""


class W1NetlinkTimeout(W1NetlinkError):
    """The bounded wait for a reply expired.

    Also what a kernel with no ``w1`` connector callback registered looks
    like: ``cn_rx_skb()`` drops the request and nothing ever answers.
    """


class W1NetlinkStatusError(W1NetlinkError):
    """The kernel answered with a nonzero ``w1_netlink_msg.status``.

    ``status`` is the kernel's ``(u8)-error``, so it is an errno for a real
    failure (19, ``ENODEV``, for an id nothing answers) and 255 where the
    kernel returned a positive value instead of an errno -- which for
    ``W1_CMD_RESET`` is ``w1_reset_bus()`` reporting no presence pulse, i.e.
    an empty or broken bus.
    """

    def __init__(self, status: int, *, command: int | None = None) -> None:
        name = _errno.errorcode.get(status, str(status))
        try:
            detail = os.strerror(status)
        except (ValueError, OverflowError):  # pragma: no cover - platform dependent
            detail = "not an errno"
        where = "" if command is None else f" for command {command}"
        super().__init__(f"kernel status {name} ({detail}){where}")
        self.status = status
        self.command = command


class W1NetlinkProtocolError(W1NetlinkError):
    """A reply that does not parse as connector/w1 records."""


@dataclass(frozen=True)
class W1Command:
    """One ``struct w1_netlink_cmd`` and its payload.

    ``data`` is what goes on the wire: the bytes to write for
    :func:`write_command`, and the *space* the kernel is to fill for
    :func:`read_command` (module docstring).
    """

    cmd: int
    data: bytes = b""

    @property
    def returns_data(self) -> bool:
        return self.cmd in _DATA_COMMANDS


def reset_command() -> W1Command:
    """A bus reset (``W1_CMD_RESET``)."""
    return W1Command(W1_CMD_RESET)


def write_command(payload: bytes) -> W1Command:
    """Write ``payload`` on the bus (``W1_CMD_WRITE``)."""
    if not payload:
        raise ValueError("write_command needs at least one byte")
    return W1Command(W1_CMD_WRITE, bytes(payload))


def read_command(count: int) -> W1Command:
    """Read ``count`` bytes from the bus (``W1_CMD_READ``)."""
    if count <= 0:
        raise ValueError(f"read_command needs a positive count, got {count}")
    return W1Command(W1_CMD_READ, bytes(count))


def master_target(master_id: int) -> bytes:
    """The 8 id bytes of a ``W1_MASTER_CMD``: ``{u32 id, u32 reserved}``."""
    if not 0 <= master_id <= 0xFFFFFFFF:
        raise ValueError(f"master id out of range: {master_id}")
    return _W1_MST.pack(master_id, 0)


@dataclass(frozen=True)
class W1Reply:
    """One ``w1_netlink_msg`` out of a reply datagram.

    ``is_status`` distinguishes the per-command status record from the record
    that carries read data; see the module docstring for how the kernel marks
    them.

    ``payload`` is the record's raw body, parsed into ``commands`` only for the
    two message types that carry ``struct w1_netlink_cmd`` records
    (``W1_MASTER_CMD``, ``W1_SLAVE_CMD``): a ``W1_LIST_MASTERS`` reply puts
    bare ``u32`` master ids at ``msg->data`` with no command header, so reading
    it as commands would decode the ids as command numbers.
    """

    seq: int
    ack: int
    msg_type: int
    status: int
    target: bytes
    payload: bytes
    commands: tuple[tuple[int, bytes], ...]

    @property
    def is_status(self) -> bool:
        return self.ack == _REQUEST_ACK

    def data(self) -> bytes:
        """Every command payload in this record, concatenated."""
        return b"".join(payload for _cmd, payload in self.commands)


def pack_request(
    *,
    msg_type: int,
    target: bytes,
    commands: Sequence[W1Command],
    seq: int,
    ack: int = _REQUEST_ACK,
    flags: int = 0,
) -> bytes:
    """One request datagram: netlink header, ``cn_msg``, one ``w1_netlink_msg``, commands.

    Pure and side-effect free, so the framing is testable against captured
    bytes without a socket.
    """
    if len(target) != _TARGET_LEN:
        raise ValueError(f"target must be {_TARGET_LEN} bytes, got {len(target)}")
    body = b"".join(_W1_CMD.pack(c.cmd, 0, len(c.data)) + c.data for c in commands)
    if len(body) > 0xFFFF:
        raise ValueError(f"command block too long: {len(body)} bytes")
    w1_msg = _W1_MSG.pack(msg_type, 0, len(body), bytes(target)) + body
    if len(w1_msg) > CONNECTOR_MAX_MSG_SIZE:
        raise ValueError(f"request too long for the connector: {len(w1_msg)} bytes")
    cn = _CN_MSG.pack(CN_W1_IDX, CN_W1_VAL, seq, ack, len(w1_msg), flags) + w1_msg
    return _NLMSG_HDR.pack(_NLMSG_HDRLEN + len(cn), _NLMSG_DONE, 0, seq, 0) + cn


def _parse_commands(body: bytes) -> tuple[tuple[int, bytes], ...]:
    out: list[tuple[int, bytes]] = []
    at = 0
    while at + _W1_CMD.size <= len(body):
        cmd, _res, length = _W1_CMD.unpack_from(body, at)
        start = at + _W1_CMD.size
        end = start + length
        if end > len(body):
            raise W1NetlinkProtocolError(
                f"w1_netlink_cmd {cmd} claims {length} bytes, {len(body) - start} left"
            )
        out.append((cmd, bytes(body[start:end])))
        at = end
    return tuple(out)


def _parse_cn_payload(payload: bytes) -> list[W1Reply]:
    replies: list[W1Reply] = []
    at = 0
    while at + _CN_MSG.size <= len(payload):
        idx, val, seq, ack, length, _flags = _CN_MSG.unpack_from(payload, at)
        start = at + _CN_MSG.size
        end = start + length
        if end > len(payload):
            raise W1NetlinkProtocolError(
                f"cn_msg claims {length} bytes, {len(payload) - start} left"
            )
        body = payload[start:end]
        at = end
        if (idx, val) != (CN_W1_IDX, CN_W1_VAL):
            _LOG.debug("w1_netlink: ignoring connector message %d.%d", idx, val)
            continue
        inner = 0
        while inner + _W1_MSG.size <= len(body):
            msg_type, status, mlen, target = _W1_MSG.unpack_from(body, inner)
            cmd_start = inner + _W1_MSG.size
            cmd_end = cmd_start + mlen
            if cmd_end > len(body):
                raise W1NetlinkProtocolError(
                    f"w1_netlink_msg claims {mlen} bytes, {len(body) - cmd_start} left"
                )
            record = bytes(body[cmd_start:cmd_end])
            replies.append(
                W1Reply(
                    seq=seq,
                    ack=ack,
                    msg_type=msg_type,
                    status=status,
                    target=bytes(target),
                    payload=record,
                    commands=(
                        _parse_commands(record) if msg_type in (W1_MASTER_CMD, W1_SLAVE_CMD) else ()
                    ),
                )
            )
            inner = cmd_end
    return replies


def parse_datagram(buf: bytes) -> list[W1Reply]:
    """Every w1 record in one received datagram, in order.

    Pure, so reply handling is testable against captured bytes. Raises
    :class:`W1NetlinkProtocolError` on a truncated or inconsistent packet and
    on a netlink-level ``NLMSG_ERROR``.
    """
    replies: list[W1Reply] = []
    at = 0
    while at + _NLMSG_HDRLEN <= len(buf):
        nlen, ntype, _nflags, _nseq, _npid = _NLMSG_HDR.unpack_from(buf, at)
        if nlen < _NLMSG_HDRLEN or at + nlen > len(buf):
            raise W1NetlinkProtocolError(
                f"netlink header claims {nlen} bytes, {len(buf) - at} left"
            )
        payload = buf[at + _NLMSG_HDRLEN : at + nlen]
        if ntype == _NLMSG_ERROR:
            code = struct.unpack_from("=i", payload)[0] if len(payload) >= 4 else 0
            raise W1NetlinkProtocolError(f"netlink refused the request: error {code}")
        replies.extend(_parse_cn_payload(payload))
        at += (nlen + 3) & ~3  # NLMSG_ALIGN
    return replies


class W1Netlink:
    """One connector socket, and requests on it that either answer or raise.

    Not thread-safe by design: a netlink socket carries one request/reply
    exchange at a time, so each reader thread owns its own instance (the
    kernel serialises the buses themselves on ``dev->bus_mutex``).

    Parameters
    ----------
    timeout_s:
        Bound on every receive. The default for :meth:`master_command` and
        :meth:`slave_command`; a caller may pass a tighter one per call. There
        is no unbounded wait anywhere in this class.
    socket_factory:
        Called as ``socket_factory(family, type, proto)``. Injected so tests
        drive the framing and the timeout path without opening a socket.
    clock:
        Monotonic seconds, for the receive deadline. Injected in tests.
    """

    def __init__(
        self,
        *,
        timeout_s: float,
        socket_factory: Callable[..., socket.socket] | None = None,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        if not timeout_s > 0:
            raise ValueError(f"timeout_s must be > 0, got {timeout_s}")
        self._timeout_s = float(timeout_s)
        self._socket_factory = socket_factory
        self._clock = clock
        self._sock: socket.socket | None = None
        self._seq = _SEQ_MIN

    # -- lifetime -------------------------------------------------------------

    @property
    def is_open(self) -> bool:
        return self._sock is not None

    def open(self) -> None:
        """Opens and binds the connector socket. Idempotent.

        Binds with ``groups=0`` (no multicast, so no ``CAP_NET_ADMIN``) and
        port id 0, which lets the kernel pick a unique one -- two reader
        threads in one process could not both bind the process id.
        """
        if self._sock is not None:
            return
        factory = self._socket_factory
        family = netlink_family()
        if factory is None:
            if family is None:
                raise W1NetlinkUnavailable("this platform has no AF_NETLINK")
            factory = socket.socket
        try:
            sock = factory(
                family if family is not None else _AF_NETLINK_LINUX,
                socket.SOCK_RAW,
                NETLINK_CONNECTOR,
            )
        except OSError as exc:
            raise W1NetlinkUnavailable(f"cannot open a connector socket: {exc}") from exc
        try:
            sock.bind((0, 0))
        except OSError as exc:
            sock.close()
            raise W1NetlinkUnavailable(f"cannot bind the connector socket: {exc}") from exc
        self._sock = sock

    def close(self) -> None:
        """Closes the socket. Idempotent; safe if never opened."""
        sock, self._sock = self._sock, None
        if sock is not None:
            try:
                sock.close()
            except OSError as exc:  # pragma: no cover - close(2) on a netlink socket
                _LOG.debug("w1_netlink: close failed: %s", exc)

    def __enter__(self) -> W1Netlink:
        self.open()
        return self

    def __exit__(self, *_exc: object) -> None:
        self.close()

    # -- requests -------------------------------------------------------------

    def list_masters(self, *, timeout_s: float | None = None) -> tuple[int, ...]:
        """Every bus master id the kernel knows (``W1_LIST_MASTERS``).

        The probe that decides whether this path exists at all: the handler
        answers even with no master attached, so a reply proves the ``w1``
        connector is registered and listening, and a
        :class:`W1NetlinkTimeout` proves it is not.
        """
        replies = self._request(
            msg_type=W1_LIST_MASTERS,
            target=bytes(_TARGET_LEN),
            commands=(),
            expect_status=0,
            expect_data=1,
            timeout_s=timeout_s,
        )
        ids: list[int] = []
        for reply in replies:
            if reply.msg_type != W1_LIST_MASTERS or reply.is_status:
                continue
            raw = reply.payload
            ids.extend(struct.unpack_from("=I", raw, at)[0] for at in range(0, len(raw) - 3, 4))
        return tuple(ids)

    def master_command(
        self,
        master_id: int,
        commands: Sequence[W1Command],
        *,
        timeout_s: float | None = None,
    ) -> tuple[W1Reply, ...]:
        """Runs ``commands`` on the master itself (``W1_MASTER_CMD``).

        The bus is addressed by id, so this reaches a master whose slave list
        holds phantoms and a master the sysfs bulk attribute was never created
        on (module docstring).
        """
        return self._request(
            msg_type=W1_MASTER_CMD,
            target=master_target(master_id),
            commands=commands,
            timeout_s=timeout_s,
        )

    def slave_command(
        self,
        slave_id: bytes,
        commands: Sequence[W1Command],
        *,
        timeout_s: float | None = None,
    ) -> tuple[W1Reply, ...]:
        """Runs ``commands`` on one slave (``W1_SLAVE_CMD``).

        ``slave_id`` is the 8-byte ``struct w1_reg_num``. The kernel resets
        the bus and sends Match ROM itself (``w1_process_cb`` ->
        ``w1_reset_select_slave``), so ``commands`` starts at the device
        function command. A slave that does not answer the Match ROM is a
        single ``-ENODEV`` status, i.e. :class:`W1NetlinkStatusError`.
        """
        if len(slave_id) != _TARGET_LEN:
            raise ValueError(f"slave id must be {_TARGET_LEN} bytes, got {len(slave_id)}")
        return self._request(
            msg_type=W1_SLAVE_CMD,
            target=bytes(slave_id),
            commands=commands,
            timeout_s=timeout_s,
        )

    # -- the one bounded wait ---------------------------------------------------

    def _next_seq(self) -> int:
        seq = self._seq
        self._seq = _SEQ_MIN if seq >= _SEQ_MAX else seq + 1
        return seq

    def _request(
        self,
        *,
        msg_type: int,
        target: bytes,
        commands: Sequence[W1Command],
        expect_status: int | None = None,
        expect_data: int | None = None,
        timeout_s: float | None = None,
    ) -> tuple[W1Reply, ...]:
        if self._sock is None:
            raise W1NetlinkUnavailable("the connector socket is not open")
        if expect_status is None:
            expect_status = len(commands)
        if expect_data is None:
            expect_data = sum(1 for c in commands if c.returns_data)
        budget = self._timeout_s if timeout_s is None else float(timeout_s)
        if not budget > 0:
            raise ValueError(f"timeout_s must be > 0, got {budget}")
        seq = self._next_seq()
        packet = pack_request(msg_type=msg_type, target=target, commands=commands, seq=seq)
        try:
            self._sock.sendto(packet, (0, 0))
        except OSError as exc:
            raise W1NetlinkUnavailable(f"cannot send on the connector socket: {exc}") from exc

        deadline = self._clock() + budget
        collected: list[W1Reply] = []
        statuses = 0
        payloads = 0
        while statuses < expect_status or payloads < expect_data:
            left = deadline - self._clock()
            if left <= 0:
                raise W1NetlinkTimeout(
                    f"no reply to w1 message type {msg_type} within {budget} s "
                    f"({statuses}/{expect_status} status, {payloads}/{expect_data} data)"
                )
            try:
                self._sock.settimeout(left)
                buf = self._sock.recv(_NLMSG_HDRLEN + CONNECTOR_MAX_MSG_SIZE)
            except TimeoutError as exc:
                raise W1NetlinkTimeout(
                    f"no reply to w1 message type {msg_type} within {budget} s "
                    f"({statuses}/{expect_status} status, {payloads}/{expect_data} data)"
                ) from exc
            except OSError as exc:
                raise W1NetlinkUnavailable(f"cannot read the connector socket: {exc}") from exc
            for reply in parse_datagram(buf):
                if reply.seq != seq:
                    # A reply to a request that already timed out, or another
                    # listener's traffic: not ours, and not a reason to fail.
                    _LOG.debug(
                        "w1_netlink: dropping a record for seq %d while waiting for %d",
                        reply.seq,
                        seq,
                    )
                    continue
                collected.append(reply)
                if reply.status:
                    cmd = reply.commands[0][0] if reply.commands else None
                    raise W1NetlinkStatusError(reply.status, command=cmd)
                if reply.is_status:
                    statuses += 1
                else:
                    payloads += 1
        return tuple(collected)
