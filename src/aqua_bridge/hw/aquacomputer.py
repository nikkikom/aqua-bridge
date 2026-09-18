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
  whose speed reads ``0xFFFF`` has no device behind it; a flow sensor reading
  ``0x7FFF`` has none either. The ``u16`` at ``+0x0A`` of a fan block is **not
  identified**: it is read into :attr:`FanStatus.unidentified_raw` so a
  diagnostic tool can show it, carries no unit and is published by nothing
  (PROJECT.md section 2, 2026-09-17, section 8 item 114; see the end of this
  docstring).
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

Those three settings are in the control report, five bytes per sensor from
``0x177``: enabled (``u8``), fallback temperature (``s16``, centi-degC), timeout
(``u16``, s) -- :func:`software_sensor_settings`. **A software sensor is not a
measurement.** The status report shows one of three things in a ``softN`` slot
and only tells two of them apart: ``0x7FFF`` while the sensor is *disabled*, the
value a host last wrote, and -- once that host stops for longer than the
timeout -- the *configured fallback*, which is a steady number that never goes
stale and never reads ``0x7FFF``. On the owner's aquaero all eight sensors are
enabled, one is fed, and ``soft2..soft8`` read their configured 50.00 degC
through all 90 reports of the 2026-09-17 run (PROJECT.md section 2, section 8
item 113). Nothing in a status report separates that from a live reading, so no
``softN`` may be bound as a temperature input: the adapter refuses the binding
and publishes each slot's settings, its reading and whether the two are the same
number, so a human can see what the slot is.

The aquaero controller block also holds the output mode, a ``u16`` at
``+0x0E``: low byte ``0x01`` drives the output as a DC voltage, ``0x02`` as PWM
(verified on the Pi 2026-09-15 by switching one output; the high byte is not
interpreted). On the aquabus blocks 5-8 the word does not describe the output:
the four blocks of one Quadro's four identical PWM outputs read ``0x0000`` on
block 5 and ``0x0002`` on blocks 6-8 in the same report, and ``0x0500`` on
blocks 5-7 in an earlier capture (PROJECT.md section 2, 2026-09-17), so
:func:`output_mode` marks an aquabus block's word *uninterpreted* and its
:attr:`OutputMode.name` is ``"unknown"`` whatever the low byte holds. The mode
is decoded for diagnostics only; nothing here writes it. The Quadro's mode field
is not known.

A block whose control source reads ``0xFFFF`` has nothing assigned to drive its
output (:attr:`ChannelState.unconfigured`; seen on block 8 with a Quadro on
aquabus, 2026-09-15). Whether writing such a block the way a configured one is
written makes the output follow has never been observed, so the adapter leaves
that one channel out of its writes, and reports it, rather than writing it blind
-- the other channels of the same controller are written as usual.

**A fan block's current and power are a sample taken inside the PWM cycle, not
an average over the report.** A fan that is driven by a chopped supply draws its
current only during the on phase, and the controller samples that current at one
instant, so the share of reports carrying a non-zero current rises with the
*duty*. Measured on the owner's hardware (2026-09-18, one fan on the
Quadro's output 3 behind the aquaero's aquabus, duty driven live): 4 of 14
reports non-zero at 25 % (3-6 mA), 16 of 16 at 60 %, 18 of 20 at 100 % (16-44
mA), and 11 of 45 at 20 % on a later run. The relation is monotone and not a
share equal to the duty -- 16 of 16 is well above 60 %, 18 of 20 below 100 % --
but it moves with the duty, which no fixed poll of the bus can do. A single
report's 0 mA is therefore no evidence
about a fan at any duty, and even a non-zero one is one sample of a switching
load: :attr:`DeviceKind.reports_power` is False for every output of both kinds,
and nothing here publishes a current or a power a rule could judge. The aquaero's
own outputs 1-4 are a separate, stronger case: they read 0 mA in *every* report
at every duty with the fan turning (three fans at 20 % over 45 reports,
2026-09-18), because the aquaero measures no current on its own PWM outputs.

This replaces the earlier reading of the same field ("the aquaero refreshes an
aquabus block about once in four reports"), which counted the non-zero currents
of a 20 % duty run and read the 23-in-90 as a bus poll interval. It was
measured honestly and it is wrong: a fixed poll cannot give 16 of 16 at a 60 %
duty. The constants that carried the number (``AQUABUS_REFRESH_REPORTS``,
``AQUABUS_REFRESH_S``, ``DeviceKind.aquabus_refresh_reports`` /
``aquabus_refresh_s``) are withdrawn rather than re-rounded, because nothing has
measured a bus poll interval on a field that is not duty-sampled (PROJECT.md
section 2, section 8 item 115). Nothing was bounded by them.

The voltage field of an aquabus block alternates in step with the current, and is
no more a per-report reading than it is: over 45 consecutive reports block 7's
current was non-zero in 11 and block 5's voltage (an output with no fan) read
0.00 V in exactly those same 11, and in no other (2026-09-18). The four blocks
therefore carry one instant between them, not a refresh each. *What* the voltage's
alternation is, that measurement does not settle: an output with no fan draws no
current in either phase, so its 0.00 V is not itself a phase of a PWM cycle, and
"the aquaero does not put the bus device's values in every report" accounts for
the field as well as the sampling does. The conclusion is the same under either,
and it is the one the captures have always carried: in some reports block 7 reads
12.10 V (the Quadro's rail) and the three outputs with no fan read 0.00 V, in the
rest all four read the aquaero's own 12.09 V, and nothing in a single report says
which. So :attr:`DeviceKind.reports_rail` is False for those outputs
(:meth:`DeviceKind.reports_rail`) and nothing publishes that field as the output's
rail. The aquaero's own blocks 1-4 do report their own rail and are unaffected.
What would separate the two accounts is the no-fan block's 0.00 V share at a 60 %
duty -- the sampling account has it rise with the duty exactly as the current's
share does, a relay account has it stay near a quarter -- which needs a duty
change, i.e. a write, and is the owner's call (PROJECT.md section 8 item 115).
``tools/aquabus_watch.py`` prints that share per block.

Speed and output duty are in **every** report and take no part in any of this --
block 7 read 251-253 rpm and duty 2000 in all 45 -- which is what makes an absent
bus device judgeable at all: :func:`aquabus_present` reads the speed fields of
the aquabus blocks and nothing else. At what interval the aquaero polls the bus
*behind* those two fields is not known and was never measured; it is not the
number above.

A bus device can also **leave the bus while the controller runs** (2026-09-15,
PROJECT.md section 8 item 92): its fan blocks then read speed ``0xFFFF`` and its
flow slot ``0x7FFF``, but the aquabus temperature slots ``bus1..8`` keep the last
value they read -- an hour later one still read 24.12 degC. Nothing in such a
report marks that temperature as old. So the aquabus temperature slots
(:attr:`DeviceKind.aquabus_temp_names`, :attr:`TempGroup.aquabus`) are a reading
only while :func:`aquabus_present` says a device answers, and the adapter reports
them as missing otherwise rather than passing a frozen number on as a
measurement.

That substitution is also why **absence is judged on the speed field and on
nothing else** (:attr:`FanStatus.present`, PROJECT.md section 8 items 90, 116).
An aquabus output with no fan reads **0.00 V** in a report that carries the bus
device's measurements -- exactly what an aquabus slot with no device on the bus
at all reads -- and the same output reads 12.09 V one second later, so the
voltage field separates the two in neither direction and would judge one slot
both ways within a second. The speed field carries no substitute: an absent bus
reads ``0xFFFF`` and a present one 0 rpm (no fan) or the fan's speed, in the
sampling report and the non-sampling report alike. Absence therefore gets no
confirmation window -- the sentinel is invariant across the sampling -- while a
rule keyed on the voltage would both lose a present device and gain an absent
one. What the captures cannot say is how soon a device that leaves the bus starts
reading ``0xFFFF``, or whether a bus hiccup can show it for a single report: all
of them come from a healthy, uninterrupted bus, and that transition is untested
hardware work (PROJECT.md section 8 item 96). Pinned on the two fixtures captured
one second apart (``tests/test_hw_aquacomputer.py``).

The ``u16`` at ``+0x0A`` of a fan block stays unidentified (item 114). What is
known: it is 0 on every block of the aquaero's own outputs 1-4 and on an aquabus
block in a report whose sample fell in the off phase, it is not the current (it
read 26 with the same block's current field at 6 mA) and not the power (7 cW
there). The reading that used to be *suspected* -- a current over the output's
on-time, of which the current field is the duty average -- is **withdrawn**: it
was built on the current field being an average over the report, and that field
is now known to be one sample taken inside the PWM cycle. Pairs of the two are
therefore two coordinates of one instantaneous sample, and fitting a duty
relation to them means nothing. The 2026-09-18 run adds a pair no scaling of the
current survives at all: one report read 2 mA with this field at 0, next to nine
that read 6 mA with 26 and one that read 1 mA with 3. It is therefore decoded
raw, named ``unidentified``, and no caller may treat it as a measurement.

**The aquaero's controller blocks are twelve, not eight**, at ``0x20C + 20k`` for
``k`` 0..11, ending at ``0x2FB``. Blocks 1-4 are its own outputs and 5-8 a
Quadro's four on aquabus, which is what this project drives; that 9-12 are four
more aquabus outputs is an inference from their position, not something a capture
shows.
Blocks 9-12 of the owner's controller hold the firmware's default for an output
nothing is assigned to, byte for byte what block 8 held while it was unconfigured
in 2026-09-15's capture: minimum power 35.00 %, maximum 100.00 %, ``+0x08``
100.00 %, ``+0x0A`` 2, ``+0x0C`` 2, mode word ``0x0000``, source ``0xFFFF``,
``+0x12`` 1000.

Six ``u16`` of a controller block are **not identified**:
``+0x00``, ``+0x02``, ``+0x08``, ``+0x0A``, ``+0x0C`` and ``+0x12``
(:data:`CONTROL_BLOCK_UNDECODED`). They are read into
:attr:`ChannelState.undecoded` so a diagnostic tool can print them and a later
capture can be diffed against this one; nothing publishes or judges them, and no
write touches them. The aquaero's firmware has a per-output **start boost** (a
duty held for a time when an output goes from stopped to running), and this block
is where the rest of an output's settings live -- but a read-only capture cannot
say which word it is, or even that it is here at all (PROJECT.md section 2, "The
controller's own start boost"). Four of the six -- ``+0x08``, ``+0x0A``, ``+0x0C``
and ``+0x12`` -- hold one value across all eight outputs in every capture, so
there is no difference between outputs to read in them. The two that do differ,
``+0x00`` (100 on output 1 and 450 on the rest today; 500, 450 and 300 in
2026-09-15's capture) and ``+0x02`` (1000 to 2800 across the seven outputs the
owner had configured by hand then), differ in a way nothing connects to a boost:
they read like a per-fan rpm pair, and no capture pairs either of them with a
boost setting known from the device's own menu. What the shapes allow, and no
more: ``+0x08`` is
a per-output centi-percent field that reads 100.00 % in every unconfigured block
and read 50.00 % on the seven outputs the owner had configured in 2026-09-15's
capture; ``+0x0A`` and ``+0x0C`` both read 2 in every block of every capture, so
even a boost duration in seconds could not be told from a tachometer's pulses per
revolution. Naming any of them needs a capture with the boost set differently on
two outputs, which only the owner can produce.
"""

from __future__ import annotations

import struct
from collections.abc import Mapping
from dataclasses import dataclass

__all__ = [
    "AQUAERO",
    "AQUAERO_CTRL_BLOCKS",
    "CONTROL_BLOCK_UNDECODED",
    "DUTY_MAX",
    "FAN_ABSENT_RPM",
    "KINDS",
    "QUADRO",
    "SENSOR_NOT_CONNECTED",
    "SOFT_SENSOR_PREFIX",
    "SOFT_SENSOR_REPORT_ID",
    "SOFT_SENSOR_SETTINGS_SIZE",
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
    "SoftSensorSettings",
    "StatusReport",
    "TempGroup",
    "active_profile",
    "aquabus_present",
    "capture_channel",
    "channel_holds",
    "channel_state",
    "check_control_report",
    "control_duty",
    "crc16_usb",
    "decode_status",
    "finalize_control_report",
    "format_channel_state",
    "format_percent",
    "format_undecoded_words",
    "is_status_report",
    "kind_by_name",
    "output_mode",
    "patch_duties",
    "restore_channel",
    "software_sensor_report",
    "software_sensor_settings",
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
#: Config-name prefix of the software temperature sensors (``soft1``, ``soft2``, ...).
#: A slot under this prefix is whatever a host last wrote or the device's configured
#: fallback, never a measurement, so no config may bind one (module docstring).
SOFT_SENSOR_PREFIX = "soft"
#: Bytes per software-sensor entry in the aquaero's control report: enabled (``u8``),
#: fallback temperature (``s16`` centi-degC), timeout (``u16`` s).
SOFT_SENSOR_SETTINGS_SIZE = 5
#: Temperature range a centi-degC ``s16`` field can carry. The top value
#: (``0x7FFF``) is :data:`SENSOR_NOT_CONNECTED`, so 327.66 degC is the largest.
TEMP_MIN_C = -327.68
TEMP_MAX_C = 327.66

#: How many controller blocks the aquaero's control report holds at
#: ``0x20C + 20k``. This project drives the first 8 (its own four outputs and one
#: Quadro's four on aquabus); what blocks 9-12 are for is an inference (four more
#: aquabus outputs) and not something a capture shows. They read the firmware's
#: default for an unassigned output --
#: source ``0xFFFF``, minimum power 35.00 %, maximum 100.00 % -- byte for byte
#: what block 8 read while it was unconfigured in 2026-09-15's capture, and the
#: series ends at ``0x2FB``: the 20 bytes at ``0x2FC`` are a different structure
#: (PROJECT.md section 2, "The controller's own start boost"). A recorded layout
#: fact of the report, not a tunable and not a count any code here consumes --
#: what the daemon drives is :attr:`DeviceKind.ctrl_channels`. It is written down,
#: and pinned against the captures in ``tests/test_hw_aquacomputer.py``, so that a
#: later capture is read against the whole array and not against the first eight.
AQUAERO_CTRL_BLOCKS = 12

#: Offsets inside an aquaero controller block whose ``u16`` this project has
#: **not** identified, read into :attr:`ChannelState.undecoded` for a diagnostic
#: tool and published, judged and written by nothing. The firmware's per-output
#: start boost could be one of them and cannot be picked out read-only: four of
#: the six hold one value across all eight outputs, and the two that do differ
#: between outputs differ in no way a boost setting is known to explain (module
#: docstring, PROJECT.md section 2, "The controller's own start boost").
CONTROL_BLOCK_UNDECODED = (0x00, 0x02, 0x08, 0x0A, 0x0C, 0x12)


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
    #: Offset of a ``u16`` in the block that is read but **not identified**
    #: (the aquaero's ``+0x0A``, PROJECT.md section 8 item 114); ``None`` where
    #: the kind's block has no such field. It is decoded into
    #: :attr:`FanStatus.unidentified_raw` for diagnostics and is published by
    #: nothing: see that attribute.
    unidentified: int | None = None


@dataclass(frozen=True)
class TempGroup:
    """One run of temperature fields in a status report, named ``{prefix}1..{prefix}{count}``."""

    prefix: str
    offset: int
    count: int
    #: What the group is, for tools and error messages.
    description: str
    #: The slots belong to a device on the controller's aquabus, not to the
    #: controller itself (the aquaero's ``bus1..8``). Such a slot keeps the last
    #: value it read when the device leaves the bus instead of reading
    #: ``0x7FFF``, so it may only be trusted while :func:`aquabus_present` says a
    #: device answers (PROJECT.md section 8 item 92).
    aquabus: bool = False

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
    #: ``(offset inside the controller block, offset in the report)`` of every
    #: ``u16`` of the block this project has not identified
    #: (:data:`CONTROL_BLOCK_UNDECODED`; empty on a kind with no such block). Read
    #: for diagnostics into :attr:`ChannelState.undecoded`; no write touches them.
    undecoded: tuple[tuple[int, int], ...] = ()

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
    #: Offset of the software-sensor settings in the control report, five bytes per
    #: sensor (:func:`software_sensor_settings`); ``None`` where they are not known.
    soft_sensor_settings_offset: int | None = None
    #: Offset of the active-profile byte in the control report (``None`` where
    #: the kind has no profiles or the byte is not known).
    profile_offset: int | None = None
    #: One status report's current and power for the kind's *own* outputs are a
    #: number a rule may judge. False on the aquaero, which reports 0 mA and 0 W
    #: for its own outputs in PWM mode however fast the fan turns; False on the
    #: Quadro, whose outputs do measure a current but sample it inside the PWM
    #: cycle, so the reports carrying a non-zero one track the duty -- 4 of 14 at
    #: 25 %, 16 of 16 at 60 % (2026-09-18, measured through the aquaero on the
    #: Quadro's own output, PROJECT.md section 2). A hardware fact, not a tunable.
    own_outputs_report_power: bool = True
    #: The kind's own outputs measure a current at all. True on the Quadro, whose
    #: measurement is real but sampled inside the PWM cycle; False on the aquaero,
    #: which puts 0 mA in its own blocks whatever the fan does. It separates two
    #: reasons a rule cannot run (:meth:`no_power_reason`), never whether it may.
    own_outputs_measures_current: bool = True
    #: The same for the kind's aquabus outputs. False on the aquaero: the values
    #: its blocks 5-8 carry are the bus device's own samples, taken inside the PWM
    #: cycle, so a single report's 0 mA is no evidence at any duty (PROJECT.md
    #: section 2). A hardware fact.
    aquabus_outputs_report_power: bool = True
    #: The voltage field of an aquabus block is that *output's* 12 V rail. False on
    #: the aquaero: that field alternates in step with the current, holding
    #: the bus device's rail in some reports and the aquaero's own in the rest,
    #: indistinguishable in a single report (PROJECT.md section 2), so the field
    #: may not be published as that output's rail. A hardware fact.
    aquabus_outputs_report_rail: bool = True

    @property
    def temp_names(self) -> tuple[str, ...]:
        """Every temperature input's config name, in report order."""
        return tuple(name for group in self.temp_groups for name in group.names())

    @property
    def temp_count(self) -> int:
        return sum(group.count for group in self.temp_groups)

    @property
    def soft_sensor_names(self) -> tuple[str, ...]:
        """Config names of this kind's software sensors (``soft1``, ``soft2``, ...).

        These are the host-written slots: a value some host put there, or the
        configured fallback once that host stopped, and a status report does not say
        which (module docstring). Nothing may bind one as a temperature input, so this
        is the list the config validation refuses.
        """
        for group in self.temp_groups:
            if group.prefix == SOFT_SENSOR_PREFIX:
                return group.names()
        return ()

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

    @property
    def aquabus_temp_names(self) -> tuple[str, ...]:
        """Config names of the temperature inputs a device on aquabus fills
        (the aquaero's ``bus1..bus8``); empty on a kind with no such group.

        These are the readings that freeze instead of going missing when the bus
        device leaves, so a caller must judge them against :func:`aquabus_present`
        (PROJECT.md section 8 item 92).
        """
        return tuple(name for group in self.temp_groups if group.aquabus for name in group.names())

    def reports_power(self, number: int) -> bool:
        """Output ``number`` (1-based) reports a current and a power one status
        report may be judged on.

        False on every output of both supported kinds, for two different reasons.
        The aquaero measures no current on its own outputs 1-4: they read 0 mA and
        0 W in every report however fast the fan turns (PROJECT.md section 8 item
        79). Behind a bus device the current *is* measured, but it is sampled
        inside the PWM cycle, so at a low duty most reports read 0 mA with the fan
        turning and even a non-zero one is a single sample of a switching load
        (module docstring, PROJECT.md section 8 items 89, 115). Absence of current
        is therefore evidence of nothing here, and a caller reading False must say
        the rule did not run rather than pass it.
        """
        if not 1 <= number <= self.pwm_count:
            raise IndexError(f"{self.name} has pwm1..pwm{self.pwm_count}, not pwm{number}")
        if self.ctrl_channels[number - 1].aquabus:
            return self.aquabus_outputs_report_power
        return self.own_outputs_report_power

    def no_power_reason(self, number: int) -> str:
        """Why output ``number``'s current and power may not be judged from one
        status report, as a clause for a message; ``""`` where they may.

        Two different facts, and the caller should not have to know which kind is
        which: the aquaero measures nothing on its own outputs, while a device that
        does measure its outputs samples the current inside the PWM cycle. Both
        end in "no rule may run on this field", for reasons a reader deserves to
        see apart.
        """
        if self.reports_power(number):
            return ""
        if self.ctrl_channels[number - 1].aquabus:
            return (
                "that block's current is the bus device's own sample, taken inside the PWM "
                "cycle, so at a low duty most reports read 0 mA with the fan turning"
            )
        if self.own_outputs_measures_current:
            return (
                "its current is sampled inside the PWM cycle, so at a low duty most reports "
                "read 0 mA with the fan turning"
            )
        return (
            f"the {self.name} reports 0 mA and 0 W for its own outputs in PWM mode however "
            "fast the fan turns"
        )

    def reports_rail(self, number: int) -> bool:
        """Output ``number`` (1-based) reports its *own* 12 V rail voltage.

        False only for the aquaero's aquabus outputs 5-8: the voltage field of
        such a block alternates in step with the block's current, holding the bus
        device's rail in the reports that carry that device's electrical sample
        and the aquaero's own in the rest, and
        nothing in a single report tells the two apart (module docstring;
        PROJECT.md section 2, section 8 item 89). A kind always reports the rail
        of its own outputs.
        """
        if not 1 <= number <= self.pwm_count:
            raise IndexError(f"{self.name} has pwm1..pwm{self.pwm_count}, not pwm{number}")
        if self.ctrl_channels[number - 1].aquabus:
            return self.aquabus_outputs_report_rail
        return True

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
        undecoded=tuple((offset, base + offset) for offset in CONTROL_BLOCK_UNDECODED),
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
        TempGroup("bus", 0x75, 8, "aquabus temperature slots", aquabus=True),
        TempGroup("soft", 0x85, 8, "software sensors"),
        TempGroup("virt", 0x95, 4, "virtual sensors"),
    ),
    fan_blocks=tuple(
        _AQUAERO_FAN_BLOCK_START + _AQUAERO_FAN_BLOCK_SIZE * k for k in range(_AQUAERO_OUTPUTS)
    ),
    fan_layout=FanLayout(
        speed=0x00, duty=0x02, voltage=0x04, current=0x06, power=0x08, unidentified=0x0A
    ),
    flow_offsets=(0xF9, 0xFB, 0xFD),
    ctrl_report_id=0x0B,
    ctrl_size=0xA93,
    ctrl_checksum=False,
    ctrl_channels=tuple(_aquaero_channel(k) for k in range(_AQUAERO_OUTPUTS)),
    save_report=bytes((0x06, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00)),
    save_verified=True,
    soft_sensor_report_id=SOFT_SENSOR_REPORT_ID,
    soft_sensor_count=8,
    soft_sensor_settings_offset=0x177,
    profile_offset=0x06,
    own_outputs_report_power=False,
    own_outputs_measures_current=False,
    aquabus_outputs_report_power=False,
    aquabus_outputs_report_rail=False,
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
    # The Quadro does measure its outputs' current -- it is the device behind the
    # numbers the aquaero relays for its aquabus blocks -- but it samples it inside
    # the PWM cycle, so no single report may be judged on it at any duty
    # (PROJECT.md section 2, 2026-09-18). Measured through the aquaero, on the
    # Quadro's own output; a Quadro read over its own USB has not been watched this
    # way, and the safe direction is not to judge.
    own_outputs_report_power=False,
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
    #: The ``u16`` of :attr:`FanLayout.unidentified` (``None`` where the kind's
    #: block has none). **Not a measurement and not a unit**: what it is has not
    #: been established (PROJECT.md section 8 item 114, module docstring), so it
    #: is decoded only so that a diagnostic tool can show the raw number, and
    #: nothing -- not ``fan_readings``, not the recorder, not a health rule --
    #: may publish or judge it. Naming it needs a duty the owner must set.
    unidentified_raw: int | None = None

    @property
    def present(self) -> bool:
        """False when the block has no device behind it (speed ``0xFFFF``).

        **The speed field is the only one that answers this** (module docstring,
        PROJECT.md section 8 items 90, 116). The voltage cannot: an aquabus output
        with no fan reads 0.00 V in a report carrying the bus device's measurements
        and 12.09 V in the next one, and an aquabus slot with no device at all reads
        0.00 V too -- so 0.00 V is neither necessary nor sufficient for absence,
        while ``0xFFFF`` is both and does not move between reports. The current and
        the power are substituted the same way. Every absence judgement in this
        project goes through this property for that reason. Every capture behind
        that comes from a healthy bus, so the *transition* -- how soon a device
        that leaves shows ``0xFFFF``, whether a hiccup can show it for one report
        -- is untested (module docstring, item 96).
        """
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
    #: ``flows[j]`` is ``flow{j+1}``, raw; ``None`` where the field holds no data
    #: (``0x7FFF``: the aquaero's flow 3 with nothing on its aquabus, 2026-09-15).
    flows: tuple[int | None, ...]
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

    def flow(self, number: int) -> int | None:
        """Flow sensor ``flowN`` (1-based), raw; ``None`` where it holds no data."""
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


def _flow(data: bytes | bytearray, offset: int) -> int | None:
    raw = _u16(data, offset)
    return None if raw == SENSOR_NOT_CONNECTED else raw


def decode_status(kind: DeviceKind, data: bytes | bytearray) -> StatusReport:
    """Decodes one status report; :class:`ReportError` for a wrong id or length."""
    _check_report(data, STATUS_REPORT_ID, kind.status_size, f"{kind.name} status report")
    temps = {
        f"{group.prefix}{i + 1}": _temperature(data, group.offset + 2 * i)
        for group in kind.temp_groups
        for i in range(group.count)
    }
    layout = kind.fan_layout
    unidentified = layout.unidentified
    fans = tuple(
        FanStatus(
            rpm=_u16(data, base + layout.speed),
            duty=_u16(data, base + layout.duty),
            voltage_cv=_u16(data, base + layout.voltage),
            current_ma=_u16(data, base + layout.current),
            power_cw=_u16(data, base + layout.power),
            unidentified_raw=(None if unidentified is None else _u16(data, base + unidentified)),
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
        flows=tuple(_flow(data, offset) for offset in kind.flow_offsets),
        power_cycles=power_cycles,
    )


def aquabus_present(kind: DeviceKind, status: StatusReport) -> bool | None:
    """Whether a device answers on the controller's aquabus, from one status report.

    ``True`` when at least one of the kind's aquabus fan blocks has a device
    behind it, ``False`` when every one of them reads speed ``0xFFFF``, and
    ``None`` on a kind with no aquabus outputs (the Quadro), which cannot say.

    The evidence is the speed field alone, and deliberately so. ``0xFFFF`` there
    means *nothing on aquabus at all*, not an output with no fan: with the
    Quadro present, its outputs with no fan read 0 rpm (PROJECT.md section 2,
    2026-09-17). Speed and output duty are in every report, while the blocks'
    electrical fields are sampled inside the PWM cycle -- so at a 20 % duty a
    block reads 0.00 V and 0 mA with its fan turning in three reports out of
    four, and judging presence on a voltage or a current would call a live device
    absent in most of them (item 116). Across 90 reports at that duty in 2026-09-17
    and 45 more in 2026-09-18 the speed field never took part in that
    substitution.

    What this cannot see: a bus device with **no fan outputs** is
    indistinguishable from an empty bus here, so on such a device it answers
    ``False`` while the device is in fact answering. In the supported topology
    the bus device is a Quadro, whose four outputs fill blocks 5-8 (PROJECT.md
    section 2, "Supported topology").

    The caller that matters is the aquabus temperature slots: they keep the last
    value they read when the device leaves the bus instead of reading
    ``0x7FFF``, so they are a reading only while this says ``True``
    (:attr:`DeviceKind.aquabus_temp_names`, PROJECT.md section 8 item 92).
    """
    if status.kind != kind.name:
        raise ValueError(f"status report of a {status.kind}, not a {kind.name}")
    outputs = kind.aquabus_outputs
    if not outputs:
        return None
    return any(status.fans[number - 1].present for number in outputs)


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


@dataclass(frozen=True)
class SoftSensorSettings:
    """How the device has one software sensor configured (from the control report).

    ``number`` is 1-based, as in the config name ``soft{number}``. A **disabled**
    sensor shows ``0x7FFF`` ("no data") in every status report. An **enabled** one
    shows the value a host last wrote until ``timeout_s`` passes with no write, and
    ``fallback_c`` for ever after -- a steady number that reads exactly like a
    measurement (module docstring, PROJECT.md section 8 item 113).
    """

    number: int
    enabled: bool
    #: The temperature the device shows once nothing has written the sensor for
    #: ``timeout_s``, degC.
    fallback_c: float
    #: How long a written value survives without a refresh, seconds.
    timeout_s: int

    @property
    def name(self) -> str:
        """The config name of the slot, ``soft1`` .. ``soft8``."""
        return f"{SOFT_SENSOR_PREFIX}{self.number}"

    def reads_fallback(self, value: float | None) -> bool:
        """``value`` (a ``softN`` reading in degC) is exactly the configured fallback.

        Both sides are the same centi-degC field, so the comparison is exact at that
        quantisation and needs no tolerance -- and therefore no config key. True is a
        strong hint that nothing is feeding the slot and not a proof: a host writing
        that very number would read the same.
        """
        if value is None:
            return False
        return round(value * 100.0) == round(self.fallback_c * 100.0)


def software_sensor_settings(
    kind: DeviceKind, data: bytes | bytearray
) -> tuple[SoftSensorSettings, ...]:
    """Every software sensor's settings from a control report, ``soft1`` first.

    Five bytes per sensor from :attr:`DeviceKind.soft_sensor_settings_offset`:
    enabled (``u8``), fallback temperature (``s16`` centi-degC), timeout (``u16``
    seconds). Empty for a kind whose settings are not known (the Quadro).

    Read against five captured aquaero control reports (PROJECT.md section 2,
    2026-09-17): the owner's watchdog sensor reads enabled with a 30 s timeout and a
    90.00 degC fallback -- the alarm that drives every output to 100 % -- while the
    earlier captures show the same sensor at the 300 s / 40.00 degC of the item 84
    experiment and the sensors the owner had not enabled yet reading disabled, which
    is exactly what those status reports show as ``0x7FFF``.
    """
    offset, count = kind.soft_sensor_settings_offset, kind.soft_sensor_count
    if offset is None or count is None:
        return ()
    _check_report(data, kind.ctrl_report_id, kind.ctrl_size, f"{kind.name} control report")
    out = []
    for i in range(count):
        base = offset + SOFT_SENSOR_SETTINGS_SIZE * i
        (fallback,) = struct.unpack_from(">h", data, base + 1)
        out.append(
            SoftSensorSettings(
                number=i + 1,
                enabled=data[base] != 0,
                fallback_c=fallback / 100.0,
                timeout_s=_u16(data, base + 3),
            )
        )
    return tuple(out)


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
    """An aquaero output's mode word: ``name`` is ``"pwm"``, ``"dc"`` or ``"unknown"``.

    ``interpreted`` is False where the word is read but means nothing about the
    output -- the aquabus blocks 5-8, whose word was measured taking different
    values on four identical outputs of one Quadro (module docstring). The name
    of an uninterpreted word is ``"unknown"`` whatever its low byte holds, so
    nothing downstream can mistake it for a drive mode.
    """

    raw: int
    interpreted: bool = True

    @property
    def name(self) -> str:
        if not self.interpreted:
            return "unknown"
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
    #: ``(offset inside the controller block, raw u16)`` of every word of the
    #: block this project has not identified (:data:`CONTROL_BLOCK_UNDECODED`;
    #: empty on the Quadro). **Not a measurement and not a setting this code
    #: understands**: it is read so a diagnostic tool can print it and a later
    #: capture can be diffed against this one -- the firmware's per-output start
    #: boost is in here somewhere and a read-only capture cannot say which word
    #: (module docstring). Nothing publishes, judges or writes these.
    undecoded: tuple[tuple[int, int], ...] = ()

    @property
    def unconfigured(self) -> bool:
        """An aquaero block with no control source assigned (``0xFFFF``): nothing on the
        device drives that output.

        Seen on block 8 with a Quadro on aquabus (2026-09-15), where the mode word was
        ``0x0000`` as well; the source alone decides here, because a block with no source
        is unconfigured whatever else it holds. Writing such a block the way a configured
        one is written has never been observed to make the output follow, so the adapter
        leaves *that channel* out of its writes and reports it, while every other channel
        of the same controller is written as usual (PROJECT.md section 8 item 89).
        """
        return self.source == SOURCE_UNCONFIGURED


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
        undecoded=tuple((inner, _u16(data, offset)) for inner, offset in channel.undecoded),
    )


def format_percent(centi: int) -> str:
    """Centi-percent as a printable percentage: ``4250`` -> ``"42.50 %"``."""
    return f"{centi / 100:.2f} %"


def format_channel_state(state: ChannelState, k: int, *, name: str = "") -> str:
    """One output's line for a tool's control-report listing, without indent or
    newline: ``pwm3<name>  duty 42.50 %`` plus whichever fields the kind has --
    control source, power limits and whether the duty is in effect, then the
    output mode (or "unconfigured").

    Shared by ``tools/aquacomputer_probe.py`` and
    ``tools/aquacomputer_commission.py`` so that what is learned about the
    control report is printed the same way by both. ``name`` is appended to the
    channel number (the commissioning tool names the config's channel there).
    """
    line = f"pwm{k + 1}{name}  duty {format_percent(state.duty):>8}"
    if state.source is not None:
        line += f"  source 0x{state.source:04X}"
        if state.min_power is not None and state.max_power is not None:
            line += (
                f"  min {format_percent(state.min_power)}  max {format_percent(state.max_power)}"
            )
        line += "  (follows its preset)" if state.on_duty else "  (does not follow its preset)"
    if state.unconfigured:
        line += "  (unconfigured: no control source, not commanded)"
    elif state.mode is not None and not state.mode.interpreted:
        line += f"  mode 0x{state.mode.raw:04X} (aquabus, not interpreted)"
    elif state.mode is not None:
        line += f"  mode {state.mode.name} (0x{state.mode.raw:04X})"
    return line


def format_undecoded_words(state: ChannelState) -> str:
    """One output's undecoded controller-block words for a tool's listing, without
    indent or newline: ``not decoded: +0x00 450  +0x02 2000  ...``; ``""`` where the
    kind has none.

    These are the six ``u16`` of an aquaero controller block nobody here has
    identified (:data:`CONTROL_BLOCK_UNDECODED`). Printing them is what makes the
    next capture useful: the firmware's per-output start boost is decoded by
    nothing and could be one of them, and the way to find out is to set the boost
    differently on two outputs and diff this line between them (module docstring,
    PROJECT.md section 2, "The controller's own start boost"). A duty write touches
    none of them -- eight channel writes left all six unchanged between the
    2026-09-17 and 2026-09-18 captures -- so what this prints is the controller's
    own configuration.
    """
    if not state.undecoded:
        return ""
    words = "  ".join(f"+0x{inner:02X} {value}" for inner, value in state.undecoded)
    return f"not decoded: {words}"


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
    """Channel ``k``'s output mode, or ``None`` where the kind has no known mode field.

    On an aquabus channel the word is returned uninterpreted (module docstring):
    it is read for diagnostics and names no drive mode.
    """
    channel = _channel(kind, k)
    offset = channel.mode
    if offset is None:
        return None
    return OutputMode(_u16(data, offset), interpreted=not channel.aquabus)


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
