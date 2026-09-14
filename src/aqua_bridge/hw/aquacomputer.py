"""Aqua Computer aquaero 5/6 and Quadro HID report layouts (PROJECT.md section 3, Track B).

Pure: stdlib only, no I/O. :mod:`aqua_bridge.hw.hidraw` moves the bytes,
:mod:`aqua_bridge.hw.aquacomputer_adapter` decides when. Everything here is a
protocol constant (report ids, sizes, offsets, CRC parameters), not a tunable.

Layouts were captured on the real devices and matched against the values the
Linux ``aquacomputer_d5next`` driver reports (PROJECT.md section 2, "USB spike
results"). All multi-byte fields are big-endian; offsets count from byte 0,
the report id. Sizes include the id byte.

Status report (input report ``0x01``, sent unsolicited about once per second):

* aquaero, 903 bytes: serial ``u16`` pair at ``0x07``, firmware at ``0x0B``;
  temperatures 8 sensors at ``0x65``, 8 virtual at ``0x85``, 4 calculated
  virtual at ``0x95``; fan blocks at ``0x167 0x173 0x17F 0x18B`` with speed
  ``+0``, output duty ``+2``, voltage ``+4``, current ``+6``, power ``+8``;
  2 flow sensors at ``0xF9``.
* Quadro, 220 bytes: serial at ``0x03``, firmware at ``0x0D``, power-cycle
  count ``u32`` at ``0x18``; temperatures 4 sensors at ``0x34``, 16 virtual at
  ``0x3C``; fan blocks at ``0x70 0x7D 0x8A 0x97`` with output duty ``+0``,
  voltage ``+2``, current ``+4``, power ``+6``, speed ``+8``; flow at ``0x6E``.

Units: temperature centi-degC (``0x7FFF`` = nothing connected), duty
centi-percent ``0..10000``, voltage centi-volt, current mA, power centi-watt,
speed rpm, flow the raw value the driver reports as ``fanN_input``.

Channel numbers follow the driver's hwmon attributes, so the config keeps
``pwmN`` / ``fanN`` / ``tempN``: aquaero ``temp1..8`` sensors, ``temp9..16``
virtual, ``temp17..20`` calculated, ``fan1..4`` fans, ``fan5..6`` flow; Quadro
``temp1..4`` sensors, ``temp5..20`` virtual, ``fan1..4`` fans, ``fan5`` flow;
``pwm1..4`` on both. Unlike the driver, temperatures decode as signed ``s16``
(``0x7FFF`` excepted), so a sub-zero reading is not 655 degC.

Control report (a HID feature report holding the device's configuration):
aquaero id ``0x0B``, 2707 bytes, no checksum; Quadro id ``0x03``, 961 bytes,
CRC-16/USB over ``[1 : size - 2)`` stored at ``[size - 2 : size]``. A duty is
commanded the way the driver does it: Quadro channel ``k`` has its duty at
``0x37 0x8C 0xE1 0x136``; aquaero channel ``k`` gets its manual preset
(``0x55C + 2k``) set to the duty, its control source (controller block
``0x20C 0x220 0x234 0x248`` ``+0x10``) set to the preset id ``0x5C + k``,
minimum power (``+0x04``) to 0 and maximum power (``+0x06``) to 100 %. Every
control report SET is followed by a secondary feature report (the official
software sends it too).
"""

from __future__ import annotations

import struct
from collections.abc import Mapping
from dataclasses import dataclass

__all__ = [
    "AQUAERO",
    "DUTY_MAX",
    "KINDS",
    "QUADRO",
    "STATUS_REPORT_ID",
    "VENDOR_ID",
    "ChannelSnapshot",
    "ChannelState",
    "ControlChannel",
    "DeviceKind",
    "FanLayout",
    "FanStatus",
    "ReportError",
    "StatusReport",
    "capture_channel",
    "channel_holds",
    "channel_state",
    "check_control_report",
    "control_duty",
    "crc16_usb",
    "decode_status",
    "finalize_control_report",
    "is_status_report",
    "kind_by_name",
    "patch_duties",
    "restore_channel",
]

#: USB vendor id of Aqua Computer GmbH & Co. KG.
VENDOR_ID = 0x0C70
#: Input report id of the status report on both devices.
STATUS_REPORT_ID = 0x01
#: A temperature field holding this value has nothing connected.
SENSOR_NOT_CONNECTED = 0x7FFF
#: Duties are centi-percent: 10000 is 100 %.
DUTY_MAX = 10000


class ReportError(ValueError):
    """A report does not have the id, length or checksum its layout requires."""


@dataclass(frozen=True)
class FanLayout:
    """Offsets of the fields inside one fan block of a status report."""

    speed: int
    duty: int
    voltage: int
    current: int
    power: int


@dataclass(frozen=True)
class ControlChannel:
    """Where one PWM output lives in the control report.

    ``duty`` is the ``u16`` centi-percent value the daemon commands. The
    aquaero only follows it while the channel's control source is its manual
    preset and its power limits are 0 % / 100 %; those fields are ``None`` on
    the Quadro, whose duty field is the output setting itself.
    """

    duty: int
    source: int | None = None
    preset_id: int | None = None
    min_power: int | None = None
    max_power: int | None = None

    def pinned(self) -> tuple[tuple[int, int], ...]:
        """``(offset, value)`` of every ``u16`` besides the duty that must hold
        for the duty to be in effect (empty on the Quadro)."""
        out: list[tuple[int, int]] = []
        if self.source is not None and self.preset_id is not None:
            out.append((self.source, self.preset_id))
        if self.min_power is not None:
            out.append((self.min_power, 0))
        if self.max_power is not None:
            out.append((self.max_power, DUTY_MAX))
        return tuple(out)

    def offsets(self) -> tuple[int, ...]:
        """Every ``u16`` offset a duty write touches, the duty first."""
        return (self.duty, *(offset for offset, _ in self.pinned()))


@dataclass(frozen=True)
class DeviceKind:
    """Everything that differs between the supported controllers."""

    name: str
    product_id: int
    #: USB interface number of the status/control HID interface.
    interface: int
    status_size: int
    serial_offset: int
    firmware_offset: int
    power_cycles_offset: int | None
    #: ``(offset, count)`` of each temperature block, in ``tempN`` order.
    temp_blocks: tuple[tuple[int, int], ...]
    #: Start of each fan block, in ``fanN`` / ``pwmN`` order.
    fan_blocks: tuple[int, ...]
    fan_layout: FanLayout
    #: Flow sensors, numbered as ``fanN`` after the fans.
    flow_offsets: tuple[int, ...]
    ctrl_report_id: int
    ctrl_size: int
    ctrl_checksum: bool
    ctrl_channels: tuple[ControlChannel, ...]
    #: The feature report sent after every control report SET.
    secondary_report: bytes

    @property
    def temp_count(self) -> int:
        return sum(count for _, count in self.temp_blocks)

    @property
    def fan_input_count(self) -> int:
        """Number of ``fanN`` inputs: fans, then flow sensors."""
        return len(self.fan_blocks) + len(self.flow_offsets)

    @property
    def pwm_count(self) -> int:
        return len(self.ctrl_channels)


_AQUAERO_CTRL_BLOCKS = (0x20C, 0x220, 0x234, 0x248)
_AQUAERO_PRESET_START = 0x55C
_AQUAERO_PRESET_ID = 0x5C

AQUAERO = DeviceKind(
    name="aquaero",
    product_id=0xF001,
    interface=2,
    status_size=903,
    serial_offset=0x07,
    firmware_offset=0x0B,
    power_cycles_offset=None,
    temp_blocks=((0x65, 8), (0x85, 8), (0x95, 4)),
    fan_blocks=(0x167, 0x173, 0x17F, 0x18B),
    fan_layout=FanLayout(speed=0x00, duty=0x02, voltage=0x04, current=0x06, power=0x08),
    flow_offsets=(0xF9, 0xFB),
    ctrl_report_id=0x0B,
    ctrl_size=0xA93,
    ctrl_checksum=False,
    ctrl_channels=tuple(
        ControlChannel(
            duty=_AQUAERO_PRESET_START + 2 * k,
            source=base + 0x10,
            preset_id=_AQUAERO_PRESET_ID + k,
            min_power=base + 0x04,
            max_power=base + 0x06,
        )
        for k, base in enumerate(_AQUAERO_CTRL_BLOCKS)
    ),
    secondary_report=bytes((0x06, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00)),
)

QUADRO = DeviceKind(
    name="quadro",
    product_id=0xF00D,
    interface=1,
    status_size=220,
    serial_offset=0x03,
    firmware_offset=0x0D,
    power_cycles_offset=0x18,
    temp_blocks=((0x34, 4), (0x3C, 16)),
    fan_blocks=(0x70, 0x7D, 0x8A, 0x97),
    fan_layout=FanLayout(speed=0x08, duty=0x00, voltage=0x02, current=0x04, power=0x06),
    flow_offsets=(0x6E,),
    ctrl_report_id=0x03,
    ctrl_size=0x3C1,
    ctrl_checksum=True,
    ctrl_channels=tuple(ControlChannel(duty=offset) for offset in (0x37, 0x8C, 0xE1, 0x136)),
    secondary_report=bytes((0x02, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x34, 0xC6)),
)

#: Supported kinds by config name (``device: aquaero`` / ``device: quadro``).
KINDS: dict[str, DeviceKind] = {kind.name: kind for kind in (AQUAERO, QUADRO)}


def kind_by_name(name: str) -> DeviceKind:
    try:
        return KINDS[name]
    except KeyError:
        raise ValueError(f"unknown device kind {name!r}; supported: {sorted(KINDS)}") from None


def _u16(data: bytes | bytearray, offset: int) -> int:
    return int.from_bytes(data[offset : offset + 2], "big")


def _put_u16(buf: bytearray, offset: int, value: int) -> None:
    buf[offset : offset + 2] = value.to_bytes(2, "big")


# ---------------------------------------------------------------------------
# Status report
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class FanStatus:
    """One output as the status report describes it (raw device units)."""

    rpm: int
    #: Output duty the device actually drives, centi-percent.
    duty: int
    voltage_cv: int
    current_ma: int
    power_cw: int

    @property
    def voltage_v(self) -> float:
        return self.voltage_cv / 100.0

    @property
    def power_w(self) -> float:
        return self.power_cw / 100.0


@dataclass(frozen=True)
class StatusReport:
    """A decoded status report. Index 0 of every tuple is channel number 1."""

    kind: str
    serial: str
    firmware: int
    #: ``temps[i]`` is ``temp{i+1}`` in degC; ``None`` when nothing is connected.
    temps: tuple[float | None, ...]
    #: ``fans[k]`` is ``fan{k+1}`` and output ``pwm{k+1}``.
    fans: tuple[FanStatus, ...]
    #: ``flows[j]`` is ``fan{len(fans)+j+1}``.
    flows: tuple[int, ...]
    #: Quadro only: increments when the device is power-cycled.
    power_cycles: int | None

    def temp(self, number: int) -> float | None:
        """``tempN`` (1-based) in degC."""
        if not 1 <= number <= len(self.temps):
            raise IndexError(f"{self.kind} has temp1..temp{len(self.temps)}, not temp{number}")
        return self.temps[number - 1]

    def fan_input(self, number: int) -> int:
        """``fanN`` (1-based) as the driver numbers it: fan rpm, then flow."""
        count = len(self.fans) + len(self.flows)
        if not 1 <= number <= count:
            raise IndexError(f"{self.kind} has fan1..fan{count}, not fan{number}")
        if number <= len(self.fans):
            return self.fans[number - 1].rpm
        return self.flows[number - 1 - len(self.fans)]

    def duty(self, number: int) -> int:
        """Output duty of ``pwmN`` (1-based), centi-percent."""
        if not 1 <= number <= len(self.fans):
            raise IndexError(f"{self.kind} has pwm1..pwm{len(self.fans)}, not pwm{number}")
        return self.fans[number - 1].duty


def _check_report(data: bytes | bytearray, report_id: int, size: int, what: str) -> None:
    if len(data) != size:
        raise ReportError(f"{what}: expected {size} bytes, got {len(data)}")
    if data[0] != report_id:
        raise ReportError(f"{what}: expected report id 0x{report_id:02X}, got 0x{data[0]:02X}")


def is_status_report(kind: DeviceKind, data: bytes | bytearray) -> bool:
    """True when ``data`` has the status report's id and this kind's length."""
    return len(data) == kind.status_size and data[0] == STATUS_REPORT_ID


def _temperature(data: bytes | bytearray, offset: int) -> float | None:
    raw = _u16(data, offset)
    if raw == SENSOR_NOT_CONNECTED:
        return None
    (signed,) = struct.unpack_from(">h", data, offset)
    return signed / 100.0


def decode_status(kind: DeviceKind, data: bytes | bytearray) -> StatusReport:
    """Decodes one status report; :class:`ReportError` for a wrong id or length."""
    _check_report(data, STATUS_REPORT_ID, kind.status_size, f"{kind.name} status report")
    temps = tuple(
        _temperature(data, offset + 2 * i)
        for offset, count in kind.temp_blocks
        for i in range(count)
    )
    layout = kind.fan_layout
    fans = tuple(
        FanStatus(
            rpm=_u16(data, base + layout.speed),
            duty=_u16(data, base + layout.duty),
            voltage_cv=_u16(data, base + layout.voltage),
            current_ma=_u16(data, base + layout.current),
            power_cw=_u16(data, base + layout.power),
        )
        for base in kind.fan_blocks
    )
    serial = f"{_u16(data, kind.serial_offset):05d}-{_u16(data, kind.serial_offset + 2):05d}"
    power_cycles = None
    if kind.power_cycles_offset is not None:
        (power_cycles,) = struct.unpack_from(">I", data, kind.power_cycles_offset)
    return StatusReport(
        kind=kind.name,
        serial=serial,
        firmware=_u16(data, kind.firmware_offset),
        temps=temps,
        fans=fans,
        flows=tuple(_u16(data, offset) for offset in kind.flow_offsets),
        power_cycles=power_cycles,
    )


# ---------------------------------------------------------------------------
# Control report
# ---------------------------------------------------------------------------


def _make_crc16_table() -> tuple[int, ...]:
    table = []
    for byte in range(256):
        crc = byte
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
        table.append(crc)
    return tuple(table)


_CRC16_TABLE = _make_crc16_table()


def crc16_usb(data: bytes | bytearray | memoryview) -> int:
    """CRC-16/USB: reflected polynomial 0x8005 (0xA001), init 0xFFFF, xorout 0xFFFF."""
    crc = 0xFFFF
    table = _CRC16_TABLE
    for byte in bytes(data):
        crc = (crc >> 8) ^ table[(crc ^ byte) & 0xFF]
    return crc ^ 0xFFFF


def _stored_crc(kind: DeviceKind, data: bytes | bytearray) -> tuple[int, int]:
    size = kind.ctrl_size
    return crc16_usb(memoryview(data)[1 : size - 2]), _u16(data, size - 2)


def check_control_report(kind: DeviceKind, data: bytes | bytearray) -> None:
    """Raises :class:`ReportError` unless ``data`` is a complete control report
    of this kind (id, length, and on the Quadro a matching checksum)."""
    _check_report(data, kind.ctrl_report_id, kind.ctrl_size, f"{kind.name} control report")
    if kind.ctrl_checksum:
        computed, stored = _stored_crc(kind, data)
        if computed != stored:
            raise ReportError(
                f"{kind.name} control report: checksum 0x{stored:04X} does not match "
                f"the contents (0x{computed:04X})"
            )


def _channel(kind: DeviceKind, k: int) -> ControlChannel:
    if not 0 <= k < kind.pwm_count:
        raise IndexError(f"{kind.name} has output channels 0..{kind.pwm_count - 1}, not {k}")
    return kind.ctrl_channels[k]


@dataclass(frozen=True)
class ChannelState:
    """One output's fields in a control report (``None`` where the kind has none)."""

    duty: int
    source: int | None
    min_power: int | None
    max_power: int | None
    #: The duty is in effect: aquaero on its own preset with limits 0 / 100 %.
    on_duty: bool


def control_duty(kind: DeviceKind, data: bytes | bytearray, k: int) -> int:
    """Commanded duty of channel ``k`` (0-based), centi-percent."""
    return _u16(data, _channel(kind, k).duty)


def channel_state(kind: DeviceKind, data: bytes | bytearray, k: int) -> ChannelState:
    channel = _channel(kind, k)

    def field(offset: int | None) -> int | None:
        return None if offset is None else _u16(data, offset)

    return ChannelState(
        duty=_u16(data, channel.duty),
        source=field(channel.source),
        min_power=field(channel.min_power),
        max_power=field(channel.max_power),
        on_duty=all(_u16(data, offset) == value for offset, value in channel.pinned()),
    )


def channel_holds(kind: DeviceKind, data: bytes | bytearray, k: int, duty: int) -> bool:
    """Channel ``k`` already commands ``duty`` (and, on the aquaero, follows it)."""
    channel = _channel(kind, k)
    if _u16(data, channel.duty) != duty:
        return False
    return all(_u16(data, offset) == value for offset, value in channel.pinned())


def patch_duties(kind: DeviceKind, buf: bytearray, duties: Mapping[int, int]) -> None:
    """Writes each ``{channel k: duty}`` into ``buf`` the way the driver does.

    Call :func:`finalize_control_report` afterwards (the Quadro's checksum).
    """
    for k, duty in duties.items():
        if isinstance(duty, bool) or not isinstance(duty, int) or not 0 <= duty <= DUTY_MAX:
            raise ValueError(f"duty for channel {k} must be an int in 0..{DUTY_MAX}, got {duty!r}")
        channel = _channel(kind, k)
        _put_u16(buf, channel.duty, duty)
        for offset, value in channel.pinned():
            _put_u16(buf, offset, value)


def finalize_control_report(kind: DeviceKind, buf: bytearray) -> None:
    """Makes a patched control report ready to SET: checks its id and length and
    stores the Quadro's checksum (the aquaero report has none)."""
    _check_report(buf, kind.ctrl_report_id, kind.ctrl_size, f"{kind.name} control report")
    if kind.ctrl_checksum:
        computed, _ = _stored_crc(kind, buf)
        _put_u16(buf, kind.ctrl_size - 2, computed)


#: ``(offset, bytes)`` of every field a duty write touches on one channel.
ChannelSnapshot = tuple[tuple[int, bytes], ...]


def capture_channel(kind: DeviceKind, data: bytes | bytearray, k: int) -> ChannelSnapshot:
    """The bytes a duty write on channel ``k`` would overwrite, for :func:`restore_channel`."""
    return tuple(
        (offset, bytes(data[offset : offset + 2])) for offset in _channel(kind, k).offsets()
    )


def restore_channel(buf: bytearray, snapshot: ChannelSnapshot) -> None:
    """Puts captured fields back (then :func:`finalize_control_report`)."""
    for offset, value in snapshot:
        buf[offset : offset + len(value)] = value
