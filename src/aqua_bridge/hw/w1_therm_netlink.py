"""DS18B20 readings over :mod:`aqua_bridge.hw.w1_netlink`.

PROJECT.md section 3 (Track B), section 8 item 38. One cycle on a bus is:

1. one bus-wide conversion -- ``W1_MASTER_CMD`` carrying ``W1_CMD_RESET`` then
   ``W1_CMD_WRITE`` of ``{0xCC, 0x44}`` (Skip ROM, Convert T), so every sensor
   on that bus converts at the same time, exactly as the kernel's own bulk
   read does;
2. the conversion time (the sensor's own ``conv_time``, 750 ms at 12 bit);
3. one ``W1_SLAVE_CMD`` per sensor carrying ``W1_CMD_WRITE`` of ``{0xBE}``
   (Read Scratchpad) and ``W1_CMD_READ`` of 9 bytes. The kernel resets and
   sends Match ROM itself, so nothing here addresses a ROM by hand.

So the cost of a cycle is one conversion for the whole bus plus a scratchpad
read per sensor, on **every** bus -- which is the point: the kernel's sysfs
bulk read exists on only one master system-wide and one phantom slave turns
its trigger into a silent no-op (:mod:`aqua_bridge.hw.w1_netlink`).

**The CRC is ours now.** ``w1_therm``'s ``temperature`` attribute checks the
scratchpad's CRC-8 in the kernel and reports EIO when it fails; a scratchpad
read over netlink is raw bus bytes and nobody has checked anything.
:func:`decode_scratchpad` verifies it before decoding, and rejects an all-zero
scratchpad on its own: CRC-8 of eight zero bytes is zero, so a bus held low
by a wiring fault produces nine zeros that *pass* the checksum and decode to a
plausible 0.0 C. Every other failure -- a short read, a bad CRC, a sensor that
does not answer Match ROM -- is reported for that one sensor, never for the
bus -- with one exception argued at :meth:`W1Therm.read_bus`: a reply that
never arrives ends the cycle rather than being charged to a sensor -- so a
single bad sensor cannot blind a zone that has others
(PROJECT.md section 3: "One flaky DS18B20 must not put all 8-10 fans on
``fallback_pwm``").

The CRC-8 is the Maxim/Dallas one (polynomial x^8 + x^5 + x^4 + 1, reflected
as 0x8C), the same function that covers a ROM id's last byte --
:func:`reg_num_from_rom_name` builds a slave identifier with it, and it was
checked against the three sensors on the owner's board, whose stored ROM CRCs
it reproduces.

Decoding follows the resolution the sensor itself reports in its config
register (scratchpad byte 4, bits 7:5 plus 9), not what config asked for:
a ``resolution`` write that failed, or a sensor swapped in while the daemon
runs, cannot make a reading wrong, only differently quantised. The two
branches are the ones ``w1_DS18B20_convert_temp()`` in
``drivers/w1/slaves/w1_therm.c`` uses: bit R2 (0x80) set means a GX20MH01
clone in 13/14-bit mode, whose low two bits live in the config register and
whose LSB is 2^-6 C; otherwise a signed 16-bit value with an LSB of 2^-4 C.
Unlike the kernel this also masks the bits the datasheet leaves undefined
below 12 bit (a real DS18B20 zeroes them, so the value is the same).

One reading this layer does **not** second-guess: a DS18B20 that has never
completed a conversion reports its power-on 85.0 C. That is a real value in
the ABI and it errs toward cooling, so it is passed through rather than
filtered -- the estimator sees the step and PROJECT.md section 3's fault
handling owns it.
"""

from __future__ import annotations

import logging
import time
from collections.abc import Callable, Mapping
from dataclasses import dataclass, field

from aqua_bridge.hw.w1_netlink import (
    W1Netlink,
    W1NetlinkStatusError,
    read_command,
    reset_command,
    write_command,
)

__all__ = [
    "CONVERSION_TIME_S",
    "SCRATCHPAD_BYTES",
    "BusRead",
    "Scratchpad",
    "ScratchpadCrcError",
    "ScratchpadError",
    "W1Therm",
    "crc8",
    "decode_scratchpad",
    "reg_num_from_rom_name",
    "rom_name_from_reg_num",
]

_LOG = logging.getLogger("aqua_bridge.hw.w1_therm_netlink")

#: 1-Wire ROM commands (DS18B20 datasheet).
SKIP_ROM = 0xCC
CONVERT_T = 0x44
READ_SCRATCHPAD = 0xBE
#: A DS18B20 scratchpad is 8 bytes plus its CRC.
SCRATCHPAD_BYTES = 9

#: Seconds a conversion takes per resolution, the same numbers
#: ``w1_DS18B20_convert_time()`` reports through a sensor's ``conv_time``
#: attribute (95/190/375/750 ms), so both read paths wait the same.
CONVERSION_TIME_S = {9: 0.095, 10: 0.190, 11: 0.375, 12: 0.750}

_CONFIG_BYTE = 4
_RESOLUTION_MASK = 0xE0
_RESOLUTION_SHIFT = 5
_RESOLUTION_MIN = 9
_RESOLUTION_MAX = 14
#: Config-register bit R2: a GX20MH01 in 13/14-bit mode (``w1_therm.c``).
_EXTENDED_RESOLUTION = 0x80
#: Bits of the raw value the datasheet leaves undefined at each resolution.
_UNDEFINED_BITS = {9: 0x0007, 10: 0x0003, 11: 0x0001, 12: 0x0000}

_ROM_NAME_HEX = 12  # "28-" plus 12 hex digits of the 48-bit id


def _crc8_table() -> tuple[int, ...]:
    table = []
    for value in range(256):
        crc = value
        for _ in range(8):
            crc = (crc >> 1) ^ 0x8C if crc & 1 else crc >> 1
        table.append(crc)
    return tuple(table)


_CRC8_TABLE = _crc8_table()


def crc8(data: bytes) -> int:
    """Maxim/Dallas CRC-8 over ``data`` (0 for an empty input)."""
    crc = 0
    for byte in data:
        crc = _CRC8_TABLE[crc ^ byte]
    return crc


def reg_num_from_rom_name(rom: str) -> bytes:
    """The 8-byte ``struct w1_reg_num`` of a sysfs ROM name such as ``28-0000...01``.

    The sysfs directory name is ``%02x-%012llx`` of the family and the 48-bit
    id, so it does not carry the ROM's CRC byte; that byte is exactly the
    CRC-8 of the seven before it, which is how a 1-Wire search validates a
    ROM in the first place. Preferred source is the slave's own ``id``
    attribute (the raw struct); this is the fallback for when that file cannot
    be read, and what tests use to build an identifier without a sysfs tree.
    """
    family_hex, _, id_hex = rom.partition("-")
    if len(family_hex) != 2 or len(id_hex) != _ROM_NAME_HEX:
        raise ValueError(f"not a 1-Wire ROM name: {rom!r}")
    try:
        family = int(family_hex, 16)
        ident = int(id_hex, 16)
    except ValueError as exc:
        raise ValueError(f"not a 1-Wire ROM name: {rom!r}") from exc
    head = bytes([family]) + ident.to_bytes(6, "little")
    return head + bytes([crc8(head)])


def rom_name_from_reg_num(reg_num: bytes) -> str:
    """The sysfs ROM name of an 8-byte ``struct w1_reg_num`` (its CRC byte is not in it)."""
    if len(reg_num) != 8:
        raise ValueError(f"a w1_reg_num is 8 bytes, got {len(reg_num)}")
    ident = int.from_bytes(reg_num[1:7], "little")
    return f"{reg_num[0]:02x}-{ident:012x}"


class ScratchpadError(Exception):
    """A scratchpad that cannot be trusted. Always about one sensor."""


class ScratchpadCrcError(ScratchpadError):
    """The scratchpad's own CRC-8 does not match its bytes."""


@dataclass(frozen=True)
class Scratchpad:
    """One verified scratchpad: the temperature and what the sensor said about itself."""

    temperature_c: float
    resolution_bits: int
    config: int
    raw: bytes

    @property
    def conversion_s(self) -> float:
        """The conversion time this sensor's own resolution needs."""
        return CONVERSION_TIME_S.get(self.resolution_bits, CONVERSION_TIME_S[12])


def _signed16(value: int) -> int:
    return value - 0x10000 if value & 0x8000 else value


def decode_scratchpad(data: bytes) -> Scratchpad:
    """Verifies a 9-byte scratchpad and decodes its temperature.

    Raises :class:`ScratchpadCrcError` when the CRC-8 disagrees and
    :class:`ScratchpadError` on a short read or an all-zero scratchpad (module
    docstring: zeros pass CRC-8).
    """
    if len(data) != SCRATCHPAD_BYTES:
        raise ScratchpadError(f"a scratchpad is {SCRATCHPAD_BYTES} bytes, got {len(data)}")
    if not any(data):
        raise ScratchpadError("all-zero scratchpad: a bus held low, not a reading")
    if crc8(data[:8]) != data[8]:
        raise ScratchpadCrcError(
            f"scratchpad CRC-8 {crc8(data[:8]):#04x} does not match the stored {data[8]:#04x}"
        )
    config = data[_CONFIG_BYTE]
    bits = min(
        ((config & _RESOLUTION_MASK) >> _RESOLUTION_SHIFT) + _RESOLUTION_MIN, _RESOLUTION_MAX
    )
    raw = int.from_bytes(data[:2], "little")
    if config & _EXTENDED_RESOLUTION:
        # GX20MH01 in 13/14-bit mode: two more bits out of the config register,
        # LSB 2^-6 C, and the shift truncates to 16 bits as the kernel's does.
        value = _signed16(((raw << 2) | (config & 0x03)) & 0xFFFF) / 64.0
    else:
        value = _signed16(raw & ~_UNDEFINED_BITS.get(bits, 0)) / 16.0
    return Scratchpad(temperature_c=value, resolution_bits=bits, config=config, raw=bytes(data))


@dataclass(frozen=True)
class BusRead:
    """What one netlink cycle produced on one bus.

    ``readings`` has an entry for every requested ROM id; a sensor that failed
    is ``None`` and carries its reason in ``failures``. The bus itself
    succeeded -- a bus-level failure is an exception, not a value here.
    """

    readings: dict[str, Scratchpad | None] = field(default_factory=dict)
    failures: dict[str, str] = field(default_factory=dict)
    crc_failures: tuple[str, ...] = ()

    @property
    def all_failed(self) -> bool:
        """True when every requested sensor failed: a bus symptom, not a sensor one."""
        return bool(self.readings) and len(self.failures) == len(self.readings)


class W1Therm:
    """DS18B20 commands over one :class:`~aqua_bridge.hw.w1_netlink.W1Netlink`.

    Parameters
    ----------
    transport:
        An open transport. This class never opens or closes it: its owner
        does, because the socket's lifetime is the reader thread's.
    sleeper:
        Called as ``sleeper(seconds)`` to wait a conversion out. The reader
        thread passes its stop event's ``wait`` so shutdown does not have to
        sit through a conversion; tests pass a recorder and never sleep.
    """

    def __init__(
        self,
        transport: W1Netlink,
        *,
        sleeper: Callable[[float], object] = time.sleep,
    ) -> None:
        self._transport = transport
        self._sleeper = sleeper

    def convert_all(self, master_id: int, *, timeout_s: float | None = None) -> None:
        """Skip ROM + Convert T on ``master_id``: every sensor on the bus, one command.

        Raises whatever the transport raises -- this is a bus-level operation
        and its failure is the bus's, not a sensor's.
        """
        self._transport.master_command(
            master_id,
            (reset_command(), write_command(bytes([SKIP_ROM, CONVERT_T]))),
            timeout_s=timeout_s,
        )

    def read_scratchpad(self, slave_id: bytes, *, timeout_s: float | None = None) -> bytes:
        """The 9 raw scratchpad bytes of one sensor (unverified; see :func:`decode_scratchpad`)."""
        replies = self._transport.slave_command(
            slave_id,
            (write_command(bytes([READ_SCRATCHPAD])), read_command(SCRATCHPAD_BYTES)),
            timeout_s=timeout_s,
        )
        data = b"".join(reply.data() for reply in replies if not reply.is_status)
        if len(data) != SCRATCHPAD_BYTES:
            raise ScratchpadError(
                f"scratchpad read returned {len(data)} bytes, expected {SCRATCHPAD_BYTES}"
            )
        return data

    def read_temperature(self, slave_id: bytes, *, timeout_s: float | None = None) -> Scratchpad:
        """One sensor's verified reading (no conversion: reads the last one)."""
        return decode_scratchpad(self.read_scratchpad(slave_id, timeout_s=timeout_s))

    def read_bus(
        self,
        master_id: int,
        targets: Mapping[str, bytes],
        *,
        conversion_s: float,
        timeout_s: float | None = None,
    ) -> BusRead:
        """One full cycle on ``master_id``: convert every sensor, wait, read each scratchpad.

        ``targets`` maps a name the caller cares about (a ROM id, as
        ``hw/onewire.py`` uses) to that slave's 8-byte identifier.
        ``conversion_s`` is the wait, which the caller takes from the sensors'
        own ``conv_time``; the returned readings carry the resolution each
        sensor actually reports, so the caller can correct a wait that was too
        short for what is really on the bus.

        A failure of the conversion, of the socket, or of a *reply arriving at
        all* propagates: the caller decides what a broken bus means
        (``hw/onewire.py`` drops that bus a tier and still reads it another way
        in the same cycle). Only what one sensor can be blamed for becomes an
        entry in :attr:`BusRead.failures`: a Match ROM nothing answers
        (``-ENODEV``) or a scratchpad that fails its CRC. A
        :class:`~aqua_bridge.hw.w1_netlink.W1NetlinkTimeout` is deliberately
        *not* one of those -- the kernel not answering within the bound says
        nothing about which sensor is at fault, and charging it to each sensor
        in turn would let one wedged bus spend ``timeout_s`` per sensor before
        anyone noticed. It ends the cycle on the first one instead.
        """
        if conversion_s < 0:
            raise ValueError(f"conversion_s must be >= 0, got {conversion_s}")
        self.convert_all(master_id, timeout_s=timeout_s)
        self._sleeper(conversion_s)
        readings: dict[str, Scratchpad | None] = {}
        failures: dict[str, str] = {}
        crc_failures: list[str] = []
        for name, slave_id in targets.items():
            try:
                readings[name] = self.read_temperature(slave_id, timeout_s=timeout_s)
            except (W1NetlinkStatusError, ScratchpadError) as exc:
                readings[name] = None
                failures[name] = str(exc)
                if isinstance(exc, ScratchpadError):
                    crc_failures.append(name)
                _LOG.debug("w1_therm_netlink: %s failed: %s", name, exc)
        return BusRead(readings=readings, failures=failures, crc_failures=tuple(crc_failures))
