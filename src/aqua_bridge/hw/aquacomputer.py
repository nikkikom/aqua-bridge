"""Aqua Computer aquaero 5/6 and Quadro HID report layouts (PROJECT.md section 3, Track B).

Pure: stdlib only, no I/O. :mod:`aqua_bridge.hw.hidraw` moves the bytes,
:mod:`aqua_bridge.hw.aquacomputer_adapter` decides when. Everything here is a
protocol constant (report ids, sizes, offsets, CRC parameters), not a tunable.

Layouts were captured on the real devices and matched against the values the
Linux ``aquacomputer_d5next`` driver reports (PROJECT.md section 2, "USB spike
results", "hidraw check"). All multi-byte fields are big-endian; offsets count
from byte 0, the report id. Sizes include the id byte.

Status report (input report ``0x01``, sent unsolicited about once per second):

* aquaero, 903 bytes: serial ``u16`` pair at ``0x07``, firmware at ``0x0B``;
  temperatures: 8 physical sensors at ``0x65`` (``temp1..8``), 8 aquabus
  temperature slots at ``0x75`` (``bus1..8``; a Quadro on aquabus puts its
  sensors 1-4 in slots 1-4), 8 software sensors at ``0x85`` (``soft1..8``, set
  by the host with output report ``0x07``; the Linux driver calls them virtual
  sensors) and 4 virtual sensors at ``0x95`` (``virt1..4``; the driver's
  calculated virtual sensors); fan blocks at ``0x167 + 12k`` for ``k`` 0..7
  with speed ``+0``, output duty ``+2``, voltage ``+4``, current ``+6``, power
  ``+8`` (blocks 1-4 the aquaero's own outputs, 5-8 the outputs of a device on
  its aquabus, a Quadro's outputs 1-4); 3 flow sensors at ``0xF9``. A fan block
  whose speed reads ``0xFFFF`` has no device behind it.
* Quadro, 220 bytes: serial at ``0x03``, firmware at ``0x0D``, power-cycle
  count ``u32`` at ``0x18``; temperatures: 4 physical sensors at ``0x34``
  (``temp1..4``) and 16 software sensors at ``0x3C`` (``soft1..16``; the
  driver's virtual sensors, named after the aquaero's, not verified on the
  Quadro); fan blocks at ``0x70 0x7D 0x8A 0x97`` with output duty ``+0``,
  voltage ``+2``, current ``+4``, power ``+6``, speed ``+8``; flow at ``0x6E``.

Units: temperature centi-degC (``0x7FFF`` = no data), duty centi-percent
``0..10000``, voltage centi-volt, current mA, power centi-watt, speed rpm, flow
the raw value the driver reports as a ``fanN_input``.

Config names: ``pwmN`` for outputs, ``fanN`` for the tachometer of output N,
``flowN`` for flow sensors and the temperature group prefixes above. The Linux
driver numbered the temperatures in one run (aquaero ``temp9..16`` software,
``temp17..20`` virtual; Quadro ``temp5..20`` software) and the flow sensors as
``fanN`` after the fans; those names are not used here. Unlike the driver,
temperatures decode as signed ``s16`` (``0x7FFF`` excepted), so a sub-zero
reading is not 655 degC.

Control report (a HID feature report holding the device's configuration):
aquaero id ``0x0B``, 2707 bytes, no checksum; Quadro id ``0x03``, 961 bytes,
CRC-16/USB over ``[1 : size - 2)`` stored at ``[size - 2 : size]``. A duty is
commanded the way the driver does it: Quadro channel ``k`` has its duty at
``0x37 0x8C 0xE1 0x136``; aquaero channel ``k`` (0..7) gets its manual preset
(``0x55C + 2k``) set to the duty, its control source (controller block
``0x20C + 20k``, ``+0x10``) set to the preset id ``0x5C + k``, minimum power
(``+0x04``) to 0 and maximum power (``+0x06``) to 100 %. Verified on the
aquaero's outputs 1-4 and on output 7, a Quadro's output 3 on aquabus.

A SET of the control report takes effect in the next status report and is not
kept over a power cycle (PROJECT.md section 8 item 84). The save report
(:attr:`DeviceKind.save_report`, the "secondary report" the official software
and the driver send after every SET) stores the configuration in the
controller's memory; the adapter sends it only on an explicit request. It is
verified to save on the aquaero; the Quadro's is byte-identical to the
Farbwerk 360's documented save report, not verified on the Quadro.

Byte ``0x06`` of the aquaero's control report is the profile it runs
(:func:`active_profile`, 0 = profile 1). An alarm action can select a profile,
and the switch reloads the *saved* profile, so live duties are lost with it
(PROJECT.md section 8 item 84).

The aquaero's eight software temperature sensors are set by the host with HID
*output* report ``0x07`` (:func:`software_sensor_report`, 17 bytes: the id then
eight centi-degC ``s16`` values, ``0x7FFF`` = no data). With the sensor enabled,
a fallback temperature and a timeout configured on the device, a value written
every tick is the heartbeat of a hardware watchdog: the device falls back to the
configured temperature when the host stops, and an alarm on it can select a safe
profile. The Quadro's software-sensor report is not known.

The aquaero controller block also holds the output mode, a ``u16`` at
``+0x0E``: low byte ``0x01`` drives the output as a DC voltage, ``0x02`` as PWM
(verified on the Pi 2026-09-15 by switching one output; the high byte is not
interpreted). On the aquabus blocks 5-8 the low byte is 0 and not interpreted
(the device on the bus drives the output). Block 8 read unconfigured (source
``0xFFFF``, mode ``0x0000``) with a Quadro on aquabus. The mode is decoded for
diagnostics only; nothing here writes it. The Quadro's mode field is not known.
"""

from __future__ import annotations

import struct
from collections.abc import Mapping
from dataclasses import dataclass

__all__ = [
    "AQUAERO",
    "DUTY_MAX",
    "FAN_ABSENT_RPM",
    "KINDS",
    "QUADRO",
    "SENSOR_NOT_CONNECTED",
    "SOFT_SENSOR_REPORT_ID",
    "SOURCE_UNCONFIGURED",
    "STATUS_REPORT_ID",
    "TEMP_MAX_C",
    "TEMP_MIN_C",
    "VENDOR_ID",
    "ChannelSnapshot",
    "ChannelState",
    "ControlChannel",
    "DeviceKind",
    "FanLayout",
    "FanStatus",
    "OutputMode",
    "ReportError",
    "StatusReport",
    "TempGroup",
    "active_profile",
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
    "output_mode",
    "patch_duties",
    "restore_channel",
    "software_sensor_report",
]

#: USB vendor id of Aqua Computer GmbH & Co. KG.
VENDOR_ID = 0x0C70
#: Input report id of the status report on both devices.
STATUS_REPORT_ID = 0x01
#: A temperature field holding this value has nothing connected.
SENSOR_NOT_CONNECTED = 0x7FFF
#: A fan block whose speed holds this value has no device behind it (an aquabus
#: slot of the aquaero with nothing on the bus).
FAN_ABSENT_RPM = 0xFFFF
#: An aquaero controller block with this control source is not configured.
SOURCE_UNCONFIGURED = 0xFFFF
#: Duties are centi-percent: 10000 is 100 %.
DUTY_MAX = 10000
#: HID OUTPUT report that sets the aquaero's software temperature sensors
#: (:func:`software_sensor_report`); the Quadro's is not known.
SOFT_SENSOR_REPORT_ID = 0x07
#: Temperature range a centi-degC ``s16`` field can carry. The top value
#: (``0x7FFF``) is :data:`SENSOR_NOT_CONNECTED`, so 327.66 degC is the largest.
TEMP_MIN_C = -327.68
TEMP_MAX_C = 327.66


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
class TempGroup:
    """One run of temperature fields in a status report, named ``{prefix}1..{prefix}{count}``."""

    prefix: str
    offset: int
    count: int
    #: What the group is, for tools and error messages.
    description: str

    def names(self) -> tuple[str, ...]:
        return tuple(f"{self.prefix}{i}" for i in range(1, self.count + 1))


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
    #: aquaero: the ``u16`` output mode word (read only, :func:`output_mode`).
    mode: int | None = None
    #: The output belongs to a device on the aquaero's aquabus (aquaero outputs 5-8).
    aquabus: bool = False

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
    #: The temperature groups in report order (config names ``{prefix}N``).
    temp_groups: tuple[TempGroup, ...]
    #: Start of each fan block, in ``fanN`` / ``pwmN`` order.
    fan_blocks: tuple[int, ...]
    fan_layout: FanLayout
    #: Flow sensors, ``flowN`` in this order.
    flow_offsets: tuple[int, ...]
    ctrl_report_id: int
    ctrl_size: int
    ctrl_checksum: bool
    ctrl_channels: tuple[ControlChannel, ...]
    #: The feature report that stores the configuration in the controller's
    #: memory; never part of a normal write (module docstring).
    save_report: bytes
    #: The save report was seen to persist the configuration over a power cycle.
    save_verified: bool
    #: HID OUTPUT report id that sets this kind's software temperature sensors
    #: (:func:`software_sensor_report`); ``None`` where the report is not known.
    soft_sensor_report_id: int | None = None
    #: How many software sensors that report carries (``None`` with no report).
    soft_sensor_count: int | None = None
    #: Offset of the active-profile byte in the control report (``None`` where
    #: the kind has no profiles or the byte is not known).
    profile_offset: int | None = None

    @property
    def temp_names(self) -> tuple[str, ...]:
        """Every temperature input's config name, in report order."""
        return tuple(name for group in self.temp_groups for name in group.names())

    @property
    def temp_count(self) -> int:
        return sum(group.count for group in self.temp_groups)

    @property
    def fan_count(self) -> int:
        """Number of tachometers ``fanN``, one per output."""
        return len(self.fan_blocks)

    @property
    def flow_count(self) -> int:
        return len(self.flow_offsets)

    @property
    def pwm_count(self) -> int:
        return len(self.ctrl_channels)

    @property
    def aquabus_outputs(self) -> tuple[int, ...]:
        """Output numbers (1-based) that belong to a device on the aquaero's aquabus."""
        return tuple(k + 1 for k, channel in enumerate(self.ctrl_channels) if channel.aquabus)

    def describe_temps(self) -> str:
        """``temp1..temp8, bus1..bus8, ...``, for messages."""
        return ", ".join(f"{g.prefix}1..{g.prefix}{g.count}" for g in self.temp_groups)


#: aquaero outputs: 1-4 its own, 5-8 the outputs of a device on its aquabus.
_AQUAERO_OUTPUTS = 8
_AQUAERO_OWN_OUTPUTS = 4
_AQUAERO_FAN_BLOCK_START = 0x167
_AQUAERO_FAN_BLOCK_SIZE = 12
_AQUAERO_CTRL_BLOCK_START = 0x20C
_AQUAERO_CTRL_BLOCK_SIZE = 20
_AQUAERO_PRESET_START = 0x55C
_AQUAERO_PRESET_ID = 0x5C


def _aquaero_channel(k: int) -> ControlChannel:
    base = _AQUAERO_CTRL_BLOCK_START + _AQUAERO_CTRL_BLOCK_SIZE * k
    return ControlChannel(
        duty=_AQUAERO_PRESET_START + 2 * k,
        source=base + 0x10,
        preset_id=_AQUAERO_PRESET_ID + k,
        min_power=base + 0x04,
        max_power=base + 0x06,
        mode=base + 0x0E,
        aquabus=k >= _AQUAERO_OWN_OUTPUTS,
    )


AQUAERO = DeviceKind(
    name="aquaero",
    product_id=0xF001,
    interface=2,
    status_size=903,
    serial_offset=0x07,
    firmware_offset=0x0B,
    power_cycles_offset=None,
    temp_groups=(
        TempGroup("temp", 0x65, 8, "physical sensors"),
        TempGroup("bus", 0x75, 8, "aquabus temperature slots"),
        TempGroup("soft", 0x85, 8, "software sensors"),
        TempGroup("virt", 0x95, 4, "virtual sensors"),
    ),
    fan_blocks=tuple(
        _AQUAERO_FAN_BLOCK_START + _AQUAERO_FAN_BLOCK_SIZE * k for k in range(_AQUAERO_OUTPUTS)
    ),
    fan_layout=FanLayout(speed=0x00, duty=0x02, voltage=0x04, current=0x06, power=0x08),
    flow_offsets=(0xF9, 0xFB, 0xFD),
    ctrl_report_id=0x0B,
    ctrl_size=0xA93,
    ctrl_checksum=False,
    ctrl_channels=tuple(_aquaero_channel(k) for k in range(_AQUAERO_OUTPUTS)),
    save_report=bytes((0x06, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00)),
    save_verified=True,
    soft_sensor_report_id=SOFT_SENSOR_REPORT_ID,
    soft_sensor_count=8,
    profile_offset=0x06,
)

QUADRO = DeviceKind(
    name="quadro",
    product_id=0xF00D,
    interface=1,
    status_size=220,
    serial_offset=0x03,
    firmware_offset=0x0D,
    power_cycles_offset=0x18,
    temp_groups=(
        TempGroup("temp", 0x34, 4, "physical sensors"),
        TempGroup("soft", 0x3C, 16, "software sensors"),
    ),
    fan_blocks=(0x70, 0x7D, 0x8A, 0x97),
    fan_layout=FanLayout(speed=0x08, duty=0x00, voltage=0x02, current=0x04, power=0x06),
    flow_offsets=(0x6E,),
    ctrl_report_id=0x03,
    ctrl_size=0x3C1,
    ctrl_checksum=True,
    ctrl_channels=tuple(ControlChannel(duty=offset) for offset in (0x37, 0x8C, 0xE1, 0x136)),
    save_report=bytes((0x02, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x34, 0xC6)),
    save_verified=False,
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
    def present(self) -> bool:
        """False when the block has no device behind it (speed ``0xFFFF``)."""
        return self.rpm != FAN_ABSENT_RPM

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
    #: Every temperature input by config name (``temp1``, ``bus2``, ``soft1``, ...)
    #: in report order, in degC; ``None`` where the field holds no data.
    temps: Mapping[str, float | None]
    #: ``fans[k]`` is tachometer ``fan{k+1}`` and output ``pwm{k+1}``.
    fans: tuple[FanStatus, ...]
    #: ``flows[j]`` is ``flow{j+1}``.
    flows: tuple[int, ...]
    #: Quadro only: increments when the device is power-cycled.
    power_cycles: int | None

    def temp(self, name: str) -> float | None:
        """The temperature input ``name`` (``temp1``, ``bus2``, ...) in degC."""
        try:
            return self.temps[name]
        except KeyError:
            raise KeyError(f"{self.kind} has no temperature input {name!r}") from None

    def rpm(self, number: int) -> int:
        """Speed of tachometer ``fanN`` (1-based), rpm; ``0xFFFF`` without a device."""
        return self._fan(number, "fan").rpm

    def flow(self, number: int) -> int:
        """Flow sensor ``flowN`` (1-based), raw."""
        if not 1 <= number <= len(self.flows):
            raise IndexError(f"{self.kind} has flow1..flow{len(self.flows)}, not flow{number}")
        return self.flows[number - 1]

    def duty(self, number: int) -> int:
        """Output duty of ``pwmN`` (1-based), centi-percent."""
        return self._fan(number, "pwm").duty

    def _fan(self, number: int, role: str) -> FanStatus:
        count = len(self.fans)
        if not 1 <= number <= count:
            raise IndexError(f"{self.kind} has {role}1..{role}{count}, not {role}{number}")
        return self.fans[number - 1]


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
    temps = {
        f"{group.prefix}{i + 1}": _temperature(data, group.offset + 2 * i)
        for group in kind.temp_groups
        for i in range(group.count)
    }
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
# Software sensors (HID OUTPUT report)
# ---------------------------------------------------------------------------


def _centi_degrees(value: float) -> int:
    """``value`` degC as the centi-degC ``s16`` the reports carry."""
    if isinstance(value, bool) or not isinstance(value, int | float):
        raise ValueError(f"temperature must be a number, got {value!r}")
    if not TEMP_MIN_C <= value <= TEMP_MAX_C:
        raise ValueError(
            f"temperature {value!r} is outside the {TEMP_MIN_C} .. {TEMP_MAX_C} degC a "
            "centi-degC s16 field can carry"
        )
    return round(value * 100.0)


def software_sensor_report(kind: DeviceKind, values: Mapping[int, float]) -> bytes:
    """The HID OUTPUT report that sets the software sensors ``{number: degC}``.

    ``number`` is 1-based, as in the config name ``softN``. Every slot this call
    does not name carries :data:`SENSOR_NOT_CONNECTED` ("no data"), which leaves
    the device's own value for that sensor alone -- the report sets all of them
    at once, so writing one sensor must not blank the other seven. The report is
    an *output* report (it goes to the device node with ``write``), not a feature
    report, and it changes no field of the control report.

    Verified on the aquaero (PROJECT.md section 8 item 84): report ``0x07``, 17
    bytes, the eight values big-endian in 1/100 degC. :class:`ValueError` for a
    kind whose report is not known (the Quadro) or a value the field cannot
    carry.
    """
    report_id, count = kind.soft_sensor_report_id, kind.soft_sensor_count
    if report_id is None or count is None:
        raise ValueError(f"the {kind.name}'s software-sensor report is not known")
    buf = bytearray(1 + 2 * count)
    buf[0] = report_id
    for i in range(count):
        _put_u16(buf, 1 + 2 * i, SENSOR_NOT_CONNECTED)
    for number, value in values.items():
        if isinstance(number, bool) or not isinstance(number, int) or not 1 <= number <= count:
            raise ValueError(
                f"{kind.name} software sensor number must be an int in 1..{count}, got {number!r}"
            )
        struct.pack_into(">h", buf, 1 + 2 * (number - 1), _centi_degrees(value))
    return bytes(buf)


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


#: Low byte of the aquaero output mode word.
OUTPUT_MODE_DC = 0x01
OUTPUT_MODE_PWM = 0x02


@dataclass(frozen=True)
class OutputMode:
    """An aquaero output's mode word: ``name`` is ``"pwm"``, ``"dc"`` or ``"unknown"``."""

    raw: int

    @property
    def name(self) -> str:
        low = self.raw & 0xFF
        return {OUTPUT_MODE_PWM: "pwm", OUTPUT_MODE_DC: "dc"}.get(low, "unknown")

    @property
    def is_pwm(self) -> bool:
        return self.name == "pwm"


@dataclass(frozen=True)
class ChannelState:
    """One output's fields in a control report (``None`` where the kind has none)."""

    duty: int
    source: int | None
    min_power: int | None
    max_power: int | None
    #: The duty is in effect: aquaero on its own preset with limits 0 / 100 %.
    on_duty: bool
    #: aquaero: PWM or DC voltage (``None`` on the Quadro, whose mode field is unknown).
    mode: OutputMode | None = None
    #: An aquaero aquabus output (5-8), whose mode word is not interpreted.
    aquabus: bool = False

    @property
    def unconfigured(self) -> bool:
        """An aquaero block with source ``0xFFFF`` and mode word 0 (seen on block 8 with
        a Quadro on aquabus); writing it like the others is not verified."""
        return self.source == SOURCE_UNCONFIGURED and self.mode is not None and self.mode.raw == 0


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
        mode=output_mode(kind, data, k),
        aquabus=channel.aquabus,
    )


def active_profile(kind: DeviceKind, data: bytes | bytearray) -> int | None:
    """The profile the controller runs, 1-based, or ``None`` where the kind has no
    known profile byte.

    The aquaero holds it in byte ``0x06`` of the control report (0 = profile 1,
    verified on the Pi 2026-09-15: a temperature alarm on a software sensor
    switched it to 1 = profile 2 and back). A profile switch reloads the saved
    profile, so every duty written live is gone after it (PROJECT.md section 8
    item 84).
    """
    offset = kind.profile_offset
    return None if offset is None else data[offset] + 1


def output_mode(kind: DeviceKind, data: bytes | bytearray, k: int) -> OutputMode | None:
    """Channel ``k``'s output mode, or ``None`` where the kind has no known mode field."""
    offset = _channel(kind, k).mode
    return None if offset is None else OutputMode(_u16(data, offset))


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
