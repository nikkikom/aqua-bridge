"""Fakes for the hidraw adapter tests: a clock, a sleep, and controllers behind a fake bus.

A :class:`FakeController` holds a control report (initially the captured
firmware report) and emits status reports whose output duty follows it, the
way the real devices do; tests override single fields to simulate a
configuration changed behind the daemon's back. It implements
:class:`~aqua_bridge.hw.hidraw.HidTransport` itself; :class:`FakeBus` is the
adapter's ``opener``. The plain aquaero fixtures have nothing on aquabus (fan
blocks 5-8 read rpm 0xFFFF); :func:`aquabus_aquaero` is an aquaero with the
Quadro on its aquabus (the captured aquabus reports).
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path

from aqua_bridge.hw.aquacomputer import (
    DeviceKind,
    channel_state,
    check_control_report,
    decode_status,
)
from aqua_bridge.hw.hidraw import DeviceUnavailable, HidrawInfo

FIXTURES = Path(__file__).resolve().parent / "fixtures" / "aquacomputer"


def fixture_bytes(name: str) -> bytes:
    return (FIXTURES / name).read_bytes()


class FakeClock:
    def __init__(self, t: float = 100.0) -> None:
        self.t = t

    def __call__(self) -> float:
        return self.t

    def advance(self, seconds: float) -> None:
        self.t += seconds


class FakeSleep:
    """Records every sleep and advances the clock by it."""

    def __init__(self, clock: FakeClock) -> None:
        self.clock = clock
        self.calls: list[float] = []

    def __call__(self, seconds: float) -> None:
        self.calls.append(seconds)
        self.clock.advance(seconds)


def _put_u16(buf: bytearray, offset: int, value: int) -> None:
    buf[offset : offset + 2] = value.to_bytes(2, "big")


@dataclass
class Op:
    what: str  # "get" | "set" | "save"
    t: float
    data: bytes


@dataclass
class FakeController:
    kind: DeviceKind
    clock: FakeClock
    node: str = "/dev/hidraw3"
    serial: str = "12345-54321"
    ctrl: bytearray = field(default_factory=bytearray)
    status_template: bytes = b""
    #: Output duty reported instead of what the control report says, per channel index.
    duty_override: dict[int, int] = field(default_factory=dict)
    power_cycles: int | None = None
    pending: list[bytes] = field(default_factory=list)
    ops: list[Op] = field(default_factory=list)
    #: Raised by the next feature report operations, in order.
    failures: list[BaseException] = field(default_factory=list)
    gone: bool = False
    closed: bool = False
    #: wait_readable() makes a report arrive after this many seconds (None: never).
    report_delay_s: float | None = None
    #: Seconds each feature report operation takes (advances the clock).
    op_delay_s: float = 0.0
    #: Serial written into status reports (None: the HID serial above).
    status_serial: str | None = None
    #: The Quadro/aquaero ignores writes to these channels and keeps this duty (start boost).
    ignores: dict[int, int] = field(default_factory=dict)
    open_count: int = 0

    def __post_init__(self) -> None:
        if not self.ctrl:
            self.ctrl = bytearray(fixture_bytes(f"{self.kind.name}-ctrl-firmware.bin"))
        if not self.status_template:
            self.status_template = fixture_bytes(f"{self.kind.name}-status.bin")
        if self.power_cycles is None:
            self.power_cycles = decode_status(self.kind, self.status_template).power_cycles

    # -- simulation -------------------------------------------------------

    @property
    def info(self) -> HidrawInfo:
        return HidrawInfo(
            name=Path(self.node).name,
            node=Path(self.node),
            vendor_id=0x0C70,
            product_id=self.kind.product_id,
            interface=self.kind.interface,
            serial=self.serial,
            hid_name=f"Aqua Computer {self.kind.name}",
        )

    def output_duty(self, k: int) -> int:
        if k in self.duty_override:
            return self.duty_override[k]
        if k in self.ignores:
            return self.ignores[k]
        state = channel_state(self.kind, self.ctrl, k)
        if state.on_duty:
            return state.duty
        template = decode_status(self.kind, self.status_template)
        return template.fans[k].duty

    def status(self) -> bytes:
        buf = bytearray(self.status_template)
        layout = self.kind.fan_layout
        for k, base in enumerate(self.kind.fan_blocks):
            _put_u16(buf, base + layout.duty, self.output_duty(k))
        serial = self.status_serial if self.status_serial is not None else self.serial
        first, second = (int(part) for part in serial.split("-"))
        _put_u16(buf, self.kind.serial_offset, first)
        _put_u16(buf, self.kind.serial_offset + 2, second)
        if self.kind.power_cycles_offset is not None and self.power_cycles is not None:
            offset = self.kind.power_cycles_offset
            buf[offset : offset + 4] = self.power_cycles.to_bytes(4, "big")
        return bytes(buf)

    def emit(self, count: int = 1) -> None:
        for _ in range(count):
            self.pending.append(self.status())

    def sets(self) -> list[Op]:
        return [op for op in self.ops if op.what == "set"]

    def last_set_duties(self) -> list[int]:
        from aqua_bridge.hw.aquacomputer import control_duty

        data = self.sets()[-1].data
        return [control_duty(self.kind, data, k) for k in range(self.kind.pwm_count)]

    def gets(self) -> list[Op]:
        return [op for op in self.ops if op.what == "get"]

    def saves(self) -> list[Op]:
        return [op for op in self.ops if op.what == "save"]

    # -- HidTransport -----------------------------------------------------

    def _check_usable(self) -> None:
        if self.gone:
            raise DeviceUnavailable(f"{self.node} is gone")
        assert not self.closed, "used after close()"

    def read_reports(self) -> list[bytes]:
        self._check_usable()
        out, self.pending = self.pending, []
        return out

    def wait_readable(self, timeout_s: float) -> bool:
        self._check_usable()
        if self.pending:
            return True
        if self.report_delay_s is not None and self.report_delay_s <= timeout_s:
            self.clock.advance(self.report_delay_s)
            self.emit()
            return True
        self.clock.advance(timeout_s)
        return False

    def get_feature(self, report_id: int, size: int) -> bytes:
        self._check_usable()
        assert (report_id, size) == (self.kind.ctrl_report_id, self.kind.ctrl_size)
        self.clock.advance(self.op_delay_s)
        self.ops.append(Op("get", self.clock(), b""))
        if self.failures:
            raise self.failures.pop(0)
        return bytes(self.ctrl)

    def set_feature(self, data: bytes) -> None:
        self._check_usable()
        what = "set" if data[0] == self.kind.ctrl_report_id else "save"
        self.clock.advance(self.op_delay_s)
        self.ops.append(Op(what, self.clock(), bytes(data)))
        if self.failures:
            raise self.failures.pop(0)
        if what == "set":
            check_control_report(self.kind, data)
            self.ctrl = bytearray(data)
        else:
            assert data == self.kind.save_report

    def close(self) -> None:
        self.closed = True


def aquabus_aquaero(clock: FakeClock, **fields) -> FakeController:
    """An aquaero with the Quadro on its aquabus: every fan block 5-8 has a device
    (captured with a fan on the Quadro's output 3 = aquaero output 7)."""
    from aqua_bridge.hw.aquacomputer import AQUAERO

    fields.setdefault(
        "ctrl", bytearray(fixture_bytes("aquaero-ctrl-aquabus-before-fan7-write.bin"))
    )
    fields.setdefault("status_template", fixture_bytes("aquaero-status-aquabus-fan7-100.bin"))
    return FakeController(AQUAERO, clock, **fields)


class FakeBus:
    """The adapter's ``opener``: returns the first present controller of a kind."""

    def __init__(self, *controllers: FakeController) -> None:
        self.controllers = list(controllers)
        self.opened: list[str] = []

    def __call__(self, kind: DeviceKind, serial: str | None) -> FakeController:
        for controller in self.controllers:
            if controller.kind is not kind or controller.gone:
                continue
            if serial is not None and controller.serial != serial:
                continue
            controller.closed = False
            controller.open_count += 1
            if not controller.pending and controller.report_delay_s is None:
                controller.emit()
            self.opened.append(controller.node)
            return controller
        raise DeviceUnavailable(f"no {kind.name} on the fake bus")
