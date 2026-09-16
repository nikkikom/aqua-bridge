"""Linux hidraw transport for the Aqua Computer controllers (PROJECT.md section 3, Track B).

Discovery reads ``<sysfs_root>/hidrawN/device/uevent`` (default sysfs root
``/sys/class/hidraw``; both it and the device-node directory ``/dev`` are
injectable so tests use a fake tree)::

    HID_ID=0003:00000C70:0000F001
    HID_NAME=Aqua Computer GmbH & Co. KG aquaero
    HID_PHYS=usb-20980000.usb-1.1/input2
    HID_UNIQ=12345-54321

``HID_ID`` is bus:vendor:product, the trailing ``inputN`` of ``HID_PHYS`` the
USB interface number and ``HID_UNIQ`` the serial. A device matches a
:class:`~aqua_bridge.hw.aquacomputer.DeviceKind` by vendor, product and
interface (the aquaero's interfaces 0 and 1 are keyboard and mouse and never
send a status report). ``hidrawN`` numbers change on re-plug, so a device is
found again by these attributes, never by number.

:class:`HidrawTransport` opens ``/dev/hidrawN`` non-blocking. Input reports
(one per ``read``) are drained with :meth:`HidrawTransport.read_reports`;
feature reports go through ``HIDIOCGFEATURE`` / ``HIDIOCSFEATURE`` and an output
report (the aquaero's software sensors) through ``write``. Errors
that mean the device node is gone raise :class:`DeviceUnavailable`; any other
failed feature report raises :class:`FeatureReportError` (the caller retries).

This module must not import :mod:`aqua_bridge.control`.
"""

from __future__ import annotations

import contextlib
import errno
import os
import re
import select
from collections.abc import Callable, Iterable
from dataclasses import dataclass
from pathlib import Path
from typing import Protocol, runtime_checkable

from aqua_bridge.hw.aquacomputer import VENDOR_ID, DeviceKind

try:  # Linux and macOS have fcntl; only the ioctl calls themselves are Linux-specific.
    import fcntl

    _default_ioctl: Callable[..., int] | None = fcntl.ioctl
except ImportError:  # pragma: no cover - Windows
    _default_ioctl = None

__all__ = [
    "DEFAULT_DEV_DIR",
    "DEFAULT_SYSFS_ROOT",
    "HIDRAW_BUFFER_SIZE",
    "HIDRAW_QUEUE_FULL",
    "USB_CTRL_TIMEOUT_S",
    "AmbiguousDevice",
    "DeviceUnavailable",
    "FeatureReportError",
    "HidTransport",
    "HidrawInfo",
    "HidrawTransport",
    "find_device",
    "hidiocgfeature",
    "hidiocsfeature",
    "list_hidraw_devices",
    "matches_kind",
    "open_device",
    "parse_uevent",
]

DEFAULT_SYSFS_ROOT = Path("/sys/class/hidraw")
DEFAULT_DEV_DIR = Path("/dev")

#: Larger than any input report of the supported devices (a shorter buffer
#: would silently truncate a report).
_READ_SIZE = 4096
#: The kernel's per-open-file hidraw report queue (``HIDRAW_BUFFER_SIZE`` in
#: drivers/hid/hidraw.c). It is a ring buffer that holds one report less than
#: its size and drops new reports while it is full.
HIDRAW_BUFFER_SIZE = 64
#: A drain returning this many reports found the queue full: newer reports were
#: dropped, so what it returned may be arbitrarily old.
HIDRAW_QUEUE_FULL = HIDRAW_BUFFER_SIZE - 1
#: usbhid control transfer timeout (``USB_CTRL_GET_TIMEOUT`` /
#: ``USB_CTRL_SET_TIMEOUT``, 5000 ms): the longest one feature report GET or SET
#: can block.
USB_CTRL_TIMEOUT_S = 5.0
#: errno values meaning the device node is gone (unplugged, hub dropout).
_GONE_ERRNOS = frozenset({errno.ENODEV, errno.ENXIO, errno.ENOENT, errno.ESHUTDOWN})


class DeviceUnavailable(RuntimeError):
    """The configured device cannot be used right now (absent, gone, unreadable,
    or silent). The loop's fallback runs; the next call looks for it again."""


class AmbiguousDevice(DeviceUnavailable):
    """Several devices match and no serial selects one of them."""


class FeatureReportError(RuntimeError):
    """A feature report GET or SET failed while the device node still exists."""

    def __init__(self, message: str, errno_value: int | None = None) -> None:
        super().__init__(message)
        self.errno = errno_value


# ---------------------------------------------------------------------------
# ioctl request numbers (Linux asm-generic _IOC encoding)
# ---------------------------------------------------------------------------

_IOC_READ_WRITE = 3
_IOC_SIZE_MAX = (1 << 13) - 1


def _ioc(nr: int, length: int) -> int:
    if not 0 < length <= _IOC_SIZE_MAX:
        raise ValueError(f"feature report length {length} outside 1..{_IOC_SIZE_MAX}")
    return (_IOC_READ_WRITE << 30) | (length << 16) | (ord("H") << 8) | nr


def hidiocsfeature(length: int) -> int:
    """``HIDIOCSFEATURE(len)``: send a feature report (``buf[0]`` = report id)."""
    return _ioc(0x06, length)


def hidiocgfeature(length: int) -> int:
    """``HIDIOCGFEATURE(len)``: get a feature report (``buf[0]`` = report id)."""
    return _ioc(0x07, length)


# ---------------------------------------------------------------------------
# Discovery
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class HidrawInfo:
    """One ``hidrawN`` node as sysfs describes it."""

    name: str
    node: Path
    vendor_id: int | None
    product_id: int | None
    interface: int | None
    serial: str
    hid_name: str

    def describe(self) -> str:
        serial = self.serial or "no serial"
        return f"{self.node} ({serial}, interface {self.interface})"


def parse_uevent(text: str) -> dict[str, str]:
    """``KEY=value`` lines; a line without ``=`` is skipped, the value may contain ``=``."""
    out: dict[str, str] = {}
    for line in text.splitlines():
        key, sep, value = line.partition("=")
        if sep:
            out[key.strip()] = value.strip()
    return out


_HID_ID = re.compile(r"^[0-9A-Fa-f]+:([0-9A-Fa-f]+):([0-9A-Fa-f]+)$")
_INTERFACE = re.compile(r"input(\d+)$")


def _info_from_uevent(name: str, node: Path, fields: dict[str, str]) -> HidrawInfo:
    vendor = product = interface = None
    match = _HID_ID.match(fields.get("HID_ID", ""))
    if match:
        vendor, product = int(match.group(1), 16), int(match.group(2), 16)
    phys = _INTERFACE.search(fields.get("HID_PHYS", ""))
    if phys:
        interface = int(phys.group(1))
    return HidrawInfo(
        name=name,
        node=node,
        vendor_id=vendor,
        product_id=product,
        interface=interface,
        serial=fields.get("HID_UNIQ", ""),
        hid_name=fields.get("HID_NAME", ""),
    )


def _natural_key(name: str) -> tuple[str, int]:
    match = re.match(r"^(\D*)(\d+)$", name)
    return (match.group(1), int(match.group(2))) if match else (name, -1)


def list_hidraw_devices(
    sysfs_root: Path = DEFAULT_SYSFS_ROOT, dev_dir: Path = DEFAULT_DEV_DIR
) -> list[HidrawInfo]:
    """Every ``hidrawN`` with a readable ``device/uevent``, in number order."""
    root = Path(sysfs_root)
    try:
        entries = sorted((p.name for p in root.iterdir()), key=_natural_key)
    except OSError:
        return []
    out = []
    for name in entries:
        try:
            text = (root / name / "device" / "uevent").read_text()
        except OSError:
            continue
        out.append(_info_from_uevent(name, Path(dev_dir) / name, parse_uevent(text)))
    return out


def matches_kind(info: HidrawInfo, kind: DeviceKind) -> bool:
    return (
        info.vendor_id == VENDOR_ID
        and info.product_id == kind.product_id
        and info.interface == kind.interface
    )


def find_device(
    kind: DeviceKind,
    serial: str | None = None,
    *,
    sysfs_root: Path = DEFAULT_SYSFS_ROOT,
    dev_dir: Path = DEFAULT_DEV_DIR,
    devices: Iterable[HidrawInfo] | None = None,
) -> HidrawInfo:
    """The one hidraw node of ``kind`` (with ``serial`` when given).

    Raises :class:`DeviceUnavailable` when none matches and
    :class:`AmbiguousDevice` when several match without a serial.
    """
    found = list(devices) if devices is not None else list_hidraw_devices(sysfs_root, dev_dir)
    candidates = [info for info in found if matches_kind(info, kind)]
    if serial is not None:
        selected = [info for info in candidates if info.serial == serial]
        if not selected:
            others = sorted(info.serial for info in candidates)
            raise DeviceUnavailable(
                f"no {kind.name} with serial {serial!r} under {sysfs_root}"
                + (f" (found serials {others})" if others else "")
            )
        candidates = selected
    if not candidates:
        raise DeviceUnavailable(
            f"no {kind.name} found under {sysfs_root} (USB {VENDOR_ID:04x}:"
            f"{kind.product_id:04x}, interface {kind.interface})"
        )
    if len(candidates) > 1:
        serials = sorted(info.serial for info in candidates)
        raise AmbiguousDevice(
            f"{len(candidates)} {kind.name} devices found (serials {serials}); "
            f"select one with 'serial:' in its config entry"
        )
    return candidates[0]


# ---------------------------------------------------------------------------
# Transport
# ---------------------------------------------------------------------------


@runtime_checkable
class HidTransport(Protocol):
    """What the adapter needs from an open device (a fake in tests)."""

    @property
    def info(self) -> HidrawInfo: ...

    def read_reports(self) -> list[bytes]:
        """Every input report queued since the last call, oldest first; never blocks."""
        ...

    def wait_readable(self, timeout_s: float) -> bool:
        """Blocks until an input report is queued or ``timeout_s`` passed."""
        ...

    def get_feature(self, report_id: int, size: int) -> bytes: ...

    def set_feature(self, data: bytes) -> None: ...

    def write_report(self, data: bytes) -> None:
        """Sends one HID *output* report (``data[0]`` = report id)."""
        ...

    def close(self) -> None: ...


class HidrawTransport:
    """One open ``/dev/hidrawN`` (non-blocking)."""

    def __init__(
        self,
        info: HidrawInfo,
        fd: int,
        *,
        ioctl: Callable[..., int] | None = None,
    ) -> None:
        self._info = info
        self._fd: int | None = fd
        self._ioctl = ioctl if ioctl is not None else _default_ioctl

    @classmethod
    def open(cls, info: HidrawInfo, *, ioctl: Callable[..., int] | None = None) -> HidrawTransport:
        flags = os.O_RDWR | os.O_NONBLOCK | getattr(os, "O_CLOEXEC", 0)
        try:
            fd = os.open(info.node, flags)
        except PermissionError as exc:
            raise DeviceUnavailable(
                f"cannot open {info.node}: {exc} (the udev rule deploy/99-aquacomputer.rules "
                "gives group plugdev read/write; is the service user in it?)"
            ) from exc
        except OSError as exc:
            raise DeviceUnavailable(f"cannot open {info.node}: {exc}") from exc
        return cls(info, fd, ioctl=ioctl)

    @property
    def info(self) -> HidrawInfo:
        return self._info

    def _require_fd(self) -> int:
        if self._fd is None:
            raise DeviceUnavailable(f"{self._info.node} is closed")
        return self._fd

    def read_reports(self) -> list[bytes]:
        fd = self._require_fd()
        reports: list[bytes] = []
        while True:
            try:
                data = os.read(fd, _READ_SIZE)
            except BlockingIOError:
                return reports
            except OSError as exc:
                # EIO/ENODEV: disconnected. Anything else on a read is no better:
                # the adapter closes the node either way and opens it again.
                raise DeviceUnavailable(f"reading {self._info.node} failed: {exc}") from exc
            if not data:  # EOF: the node went away
                raise DeviceUnavailable(f"{self._info.node} is gone (end of file)")
            reports.append(data)

    def wait_readable(self, timeout_s: float) -> bool:
        fd = self._require_fd()
        try:
            readable, _, _ = select.select([fd], [], [], max(0.0, timeout_s))
        except OSError as exc:
            raise DeviceUnavailable(f"waiting on {self._info.node} failed: {exc}") from exc
        return bool(readable)

    def _call(self, request: int, buf: bytearray, what: str) -> int:
        fd = self._require_fd()
        if self._ioctl is None:  # pragma: no cover - no fcntl on this platform
            raise FeatureReportError(f"{what}: ioctl is not available on this platform")
        try:
            return int(self._ioctl(fd, request, buf, True))
        except OSError as exc:
            if exc.errno in _GONE_ERRNOS:
                raise DeviceUnavailable(f"{self._info.node} is gone: {what}: {exc}") from exc
            raise FeatureReportError(f"{what} on {self._info.node}: {exc}", exc.errno) from exc

    def get_feature(self, report_id: int, size: int) -> bytes:
        buf = bytearray(size)
        buf[0] = report_id
        count = self._call(hidiocgfeature(size), buf, f"GET feature report 0x{report_id:02X}")
        return bytes(buf[: max(0, min(count, size))])

    def set_feature(self, data: bytes) -> None:
        buf = bytearray(data)
        what = f"SET feature report 0x{buf[0]:02X}"
        count = self._call(hidiocsfeature(len(buf)), buf, what)
        if count != len(buf):
            raise FeatureReportError(
                f"{what} on {self._info.node}: sent {count} of {len(buf)} bytes"
            )

    def write_report(self, data: bytes) -> None:
        """Sends one HID output report with ``write`` (``data[0]`` = the report id).

        An output report goes out over the device's interrupt OUT endpoint (the
        kernel falls back to a SET_REPORT control transfer), so this call is
        synchronous like a feature report even on the non-blocking node: the
        caller must treat it as one device operation and give it a time budget.
        """
        fd = self._require_fd()
        what = f"OUTPUT report 0x{data[0]:02X}"
        try:
            count = os.write(fd, data)
        except OSError as exc:
            if exc.errno in _GONE_ERRNOS:
                raise DeviceUnavailable(f"{self._info.node} is gone: {what}: {exc}") from exc
            raise FeatureReportError(f"{what} on {self._info.node}: {exc}", exc.errno) from exc
        if count != len(data):
            raise FeatureReportError(
                f"{what} on {self._info.node}: wrote {count} of {len(data)} bytes"
            )

    def close(self) -> None:
        if self._fd is not None:
            fd, self._fd = self._fd, None
            with contextlib.suppress(OSError):
                os.close(fd)


def open_device(
    kind: DeviceKind,
    serial: str | None = None,
    *,
    sysfs_root: Path = DEFAULT_SYSFS_ROOT,
    dev_dir: Path = DEFAULT_DEV_DIR,
) -> HidrawTransport:
    """Finds the device (:func:`find_device`) and opens its node."""
    info = find_device(kind, serial, sysfs_root=sysfs_root, dev_dir=dev_dir)
    return HidrawTransport.open(info)
