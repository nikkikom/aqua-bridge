"""Tests for aqua_bridge.hw.hidraw: discovery on a fake sysfs tree, ioctl encoding, and
the transport over a datagram socket pair (no hardware, no Linux needed).

PROJECT.md section 3 (Track B) / section 4.7.
"""

from __future__ import annotations

import errno
import os
import socket
from pathlib import Path

import pytest

from aqua_bridge.hw.aquacomputer import AQUAERO, QUADRO
from aqua_bridge.hw.hidraw import (
    AmbiguousDevice,
    DeviceUnavailable,
    FeatureReportError,
    HidrawInfo,
    HidrawTransport,
    find_device,
    hidiocgfeature,
    hidiocsfeature,
    list_hidraw_devices,
    matches_kind,
    open_device,
    parse_uevent,
)


def _hidraw(root: Path, name: str, product: int | None, interface: int, serial: str = "") -> None:
    device = root / name / "device"
    device.mkdir(parents=True)
    lines = ["DRIVER=hid-generic"]
    if product is not None:
        lines.append(f"HID_ID=0003:00000C70:0000{product:04X}")
        lines.append("HID_NAME=Aqua Computer GmbH & Co. KG device")
    lines.append(f"HID_PHYS=usb-20980000.usb-1.1/input{interface}")
    if serial:
        lines.append(f"HID_UNIQ={serial}")
    lines.append("MODALIAS=hid:b0003g0001v00000C70p0000F001")
    lines.append("a line without an equals sign")
    (device / "uevent").write_text("\n".join(lines) + "\n")


@pytest.fixture
def sysfs(tmp_path: Path) -> Path:
    root = tmp_path / "sys" / "class" / "hidraw"
    _hidraw(root, "hidraw0", 0xF001, 0, "12345-54321")  # aquaero keyboard interface
    _hidraw(root, "hidraw1", 0xF001, 1, "12345-54321")  # aquaero mouse interface
    _hidraw(root, "hidraw10", 0xF001, 2, "12345-54321")  # aquaero status/control
    _hidraw(root, "hidraw2", 0xF00D, 1, "00000-11111")  # Quadro
    _hidraw(root, "hidraw3", None, 0)  # some other HID device
    (root / "hidraw4").mkdir()  # no uevent: skipped
    return root


def test_parse_uevent_splits_on_the_first_equals_and_skips_other_lines() -> None:
    fields = parse_uevent("A=1\nB=x=y\nno equals here\n\nC=\n")
    assert fields == {"A": "1", "B": "x=y", "C": ""}


def test_list_hidraw_devices_in_number_order(sysfs: Path, tmp_path: Path) -> None:
    devices = list_hidraw_devices(sysfs, tmp_path / "dev")
    assert [d.name for d in devices] == ["hidraw0", "hidraw1", "hidraw2", "hidraw3", "hidraw10"]
    aquaero = devices[-1]
    assert aquaero.node == tmp_path / "dev" / "hidraw10"
    assert (aquaero.vendor_id, aquaero.product_id, aquaero.interface) == (0x0C70, 0xF001, 2)
    assert aquaero.serial == "12345-54321"
    assert devices[3].vendor_id is None and devices[3].serial == ""


def test_missing_sysfs_root_lists_nothing(tmp_path: Path) -> None:
    assert list_hidraw_devices(tmp_path / "nope") == []


def test_find_device_picks_the_status_interface(sysfs: Path, tmp_path: Path) -> None:
    info = find_device(AQUAERO, sysfs_root=sysfs, dev_dir=tmp_path / "dev")
    assert info.name == "hidraw10" and info.interface == 2
    quadro = find_device(QUADRO, sysfs_root=sysfs, dev_dir=tmp_path / "dev")
    assert quadro.name == "hidraw2" and quadro.interface == 1


def test_find_device_by_serial(sysfs: Path) -> None:
    _hidraw(sysfs, "hidraw11", 0xF001, 2, "99999-00001")
    with pytest.raises(AmbiguousDevice, match=r"\['12345-54321', '99999-00001'\]") as info:
        find_device(AQUAERO, sysfs_root=sysfs)
    assert "serial:" in str(info.value)
    assert isinstance(info.value, DeviceUnavailable)
    assert find_device(AQUAERO, "99999-00001", sysfs_root=sysfs).name == "hidraw11"
    assert find_device(AQUAERO, "12345-54321", sysfs_root=sysfs).name == "hidraw10"


def test_find_device_not_found(sysfs: Path, tmp_path: Path) -> None:
    with pytest.raises(DeviceUnavailable, match="no quadro with serial '00000-22222'") as exc:
        find_device(QUADRO, "00000-22222", sysfs_root=sysfs)
    assert "00000-11111" in str(exc.value)
    empty = tmp_path / "empty"
    empty.mkdir()
    with pytest.raises(DeviceUnavailable, match="no aquaero found"):
        find_device(AQUAERO, sysfs_root=empty)


def test_matches_kind_needs_vendor_product_and_interface() -> None:
    base = dict(name="hidraw0", node=Path("/dev/hidraw0"), serial="", hid_name="")
    assert matches_kind(
        HidrawInfo(vendor_id=0x0C70, product_id=0xF00D, interface=1, **base), QUADRO
    )
    assert not matches_kind(
        HidrawInfo(vendor_id=0x0C71, product_id=0xF00D, interface=1, **base), QUADRO
    )
    assert not matches_kind(
        HidrawInfo(vendor_id=0x0C70, product_id=0xF00D, interface=None, **base), QUADRO
    )


def test_ioctl_request_numbers() -> None:
    # _IOC(_IOC_WRITE|_IOC_READ, 'H', nr, len) with the asm-generic layout.
    assert hidiocgfeature(0xA93) == 0xCA934807
    assert hidiocsfeature(0x3C1) == 0xC3C14806
    with pytest.raises(ValueError):
        hidiocgfeature(0)
    with pytest.raises(ValueError):
        hidiocsfeature(1 << 13)


# --- transport ------------------------------------------------------------------------------


def _info(node: Path) -> HidrawInfo:
    return HidrawInfo(
        name=node.name,
        node=node,
        vendor_id=0x0C70,
        product_id=0xF00D,
        interface=1,
        serial="00000-11111",
        hid_name="quadro",
    )


@pytest.fixture
def socket_pair():
    device_side, host_side = socket.socketpair(socket.AF_UNIX, socket.SOCK_DGRAM)
    host_side.setblocking(False)
    yield device_side, host_side
    device_side.close()
    host_side.close()


def test_read_reports_drains_every_queued_report_without_blocking(socket_pair) -> None:
    device_side, host_side = socket_pair
    transport = HidrawTransport(_info(Path("/dev/hidraw2")), host_side.fileno())
    assert transport.read_reports() == []
    assert not transport.wait_readable(0.0)
    reports = [bytes([1]) + bytes([n]) * 219 for n in range(3)]
    for report in reports:
        device_side.send(report)
    assert transport.wait_readable(1.0)
    assert transport.read_reports() == reports
    assert transport.read_reports() == []


def test_end_of_file_raises_device_unavailable() -> None:
    read_end, write_end = os.pipe()
    os.set_blocking(read_end, False)
    transport = HidrawTransport(_info(Path("/dev/hidraw2")), read_end)
    try:
        assert transport.read_reports() == []
        os.close(write_end)
        with pytest.raises(DeviceUnavailable, match="end of file"):
            transport.read_reports()
    finally:
        transport.close()


class _FakeIoctl:
    def __init__(self, reply: bytes = b"", result: int | None = None, error: int | None = None):
        self.calls: list[tuple[int, int, bytes]] = []
        self.reply = reply
        self.result = result
        self.error = error

    def __call__(self, fd: int, request: int, buf: bytearray, mutate: bool) -> int:
        assert mutate is True
        self.calls.append((fd, request, bytes(buf)))
        if self.error is not None:
            raise OSError(self.error, os.strerror(self.error))
        if self.reply:
            buf[: len(self.reply)] = self.reply
            return len(self.reply) if self.result is None else self.result
        return len(buf) if self.result is None else self.result


def test_get_feature_passes_the_report_id_and_length() -> None:
    reply = bytes([0x03]) + bytes(range(256)) * 3 + bytes(QUADRO.ctrl_size - 1 - 768)
    ioctl = _FakeIoctl(reply=reply)
    transport = HidrawTransport(_info(Path("/dev/hidraw2")), 42, ioctl=ioctl)
    data = transport.get_feature(QUADRO.ctrl_report_id, QUADRO.ctrl_size)
    ((fd, request, sent),) = ioctl.calls
    assert fd == 42 and request == hidiocgfeature(QUADRO.ctrl_size)
    assert sent[0] == 0x03 and len(sent) == QUADRO.ctrl_size
    assert data == reply


def test_short_get_feature_returns_what_arrived() -> None:
    ioctl = _FakeIoctl(reply=bytes([0x03, 1, 2]), result=3)
    transport = HidrawTransport(_info(Path("/dev/hidraw2")), 42, ioctl=ioctl)
    assert transport.get_feature(0x03, QUADRO.ctrl_size) == bytes([0x03, 1, 2])


def test_set_feature_sends_the_buffer_and_checks_the_count() -> None:
    ioctl = _FakeIoctl()
    transport = HidrawTransport(_info(Path("/dev/hidraw2")), 7, ioctl=ioctl)
    transport.set_feature(QUADRO.secondary_report)
    ((_, request, sent),) = ioctl.calls
    assert request == hidiocsfeature(len(QUADRO.secondary_report))
    assert sent == QUADRO.secondary_report
    short = HidrawTransport(_info(Path("/dev/hidraw2")), 7, ioctl=_FakeIoctl(result=3))
    with pytest.raises(FeatureReportError, match="sent 3 of 11"):
        short.set_feature(QUADRO.secondary_report)


@pytest.mark.parametrize(
    ("err", "expected"),
    [
        (errno.ENODEV, DeviceUnavailable),
        (errno.ESHUTDOWN, DeviceUnavailable),
        (errno.EPIPE, FeatureReportError),
        (errno.ENODATA, FeatureReportError),
        (errno.EIO, FeatureReportError),
        (errno.ETIMEDOUT, FeatureReportError),
    ],
    ids=lambda v: getattr(v, "__name__", errno.errorcode.get(v, str(v))),
)
def test_feature_report_errors_are_classified(err: int, expected: type[Exception]) -> None:
    transport = HidrawTransport(_info(Path("/dev/hidraw2")), 7, ioctl=_FakeIoctl(error=err))
    with pytest.raises(expected) as exc:
        transport.get_feature(0x03, QUADRO.ctrl_size)
    if expected is FeatureReportError:
        assert exc.value.errno == err
    with pytest.raises(expected):
        transport.set_feature(QUADRO.secondary_report)


def test_closed_transport_is_unavailable() -> None:
    transport = HidrawTransport(_info(Path("/dev/hidraw2")), os.open(os.devnull, os.O_RDONLY))
    transport.close()
    transport.close()  # idempotent
    with pytest.raises(DeviceUnavailable):
        transport.read_reports()
    with pytest.raises(DeviceUnavailable):
        transport.get_feature(0x03, 16)


def test_open_missing_node_raises_device_unavailable(tmp_path: Path) -> None:
    with pytest.raises(DeviceUnavailable, match="cannot open"):
        HidrawTransport.open(_info(tmp_path / "hidraw2"))


@pytest.mark.skipif(os.geteuid() == 0, reason="root ignores file permissions")
def test_open_without_permission_names_the_udev_rule(tmp_path: Path) -> None:
    node = tmp_path / "hidraw2"
    node.write_bytes(b"")
    node.chmod(0)
    with pytest.raises(DeviceUnavailable, match="plugdev"):
        HidrawTransport.open(_info(node))


def test_open_device_finds_and_opens_the_node(sysfs: Path, tmp_path: Path) -> None:
    dev = tmp_path / "dev"
    dev.mkdir()
    (dev / "hidraw2").write_bytes(b"")
    transport = open_device(QUADRO, sysfs_root=sysfs, dev_dir=dev)
    try:
        assert transport.info.node == dev / "hidraw2"
        with pytest.raises(DeviceUnavailable):  # a regular file reads end of file
            transport.read_reports()
    finally:
        transport.close()
