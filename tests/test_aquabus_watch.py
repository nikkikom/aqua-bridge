"""Tests for tools/aquabus_watch.py: the read-only aquabus measurement (items 114, 115, 92).

What is checked here is the tool's own arithmetic and its presence/sampling
classification, over the captured reports, played back at the cadence the two
fixtures give: one report whose electrical sample fell in the on phase of a 20 %
duty, then three whose sample missed. That ratio is a property of *this playback*
and is written out literally below -- the hardware's ratio is a function of the
duty (4 of 14 at 25 %, 16 of 16 at 60 %, PROJECT.md section 2), not a constant any
code carries, which is exactly why no constant is imported here any more.

``tools/`` is not on ``pythonpath``, so this file adds it to ``sys.path`` itself (the
same way tests/test_aquacomputer_probe.py does).
"""

from __future__ import annotations

import io
import sys
from pathlib import Path

import pytest

from aqua_bridge.hw.aquacomputer import AQUAERO
from aquacomputer_fakes import FakeClock, fixture_bytes

TOOLS_DIR = Path(__file__).resolve().parent.parent / "tools"
if str(TOOLS_DIR) not in sys.path:
    sys.path.insert(0, str(TOOLS_DIR))

import aquabus_watch  # noqa: E402 -- must follow the sys.path tweak above

MEASURING = "aquaero-status-aquabus-block7-power.bin"
SUBSTITUTED = "aquaero-status-aquabus-block7-no-power.bin"
NO_DEVICE = "aquaero-status-no-aquabus.bin"


class ScriptedController:
    """Hands out one prepared status report per ``wait_readable``, a second apart."""

    def __init__(self, clock: FakeClock, reports: list[bytes]) -> None:
        self.clock = clock
        self.reports = list(reports)
        self.pending: list[bytes] = []
        self.closed = False

    def wait_readable(self, timeout_s: float) -> bool:
        self.clock.advance(1.0)
        if self.reports:
            self.pending.append(self.reports.pop(0))
        return bool(self.pending)

    def read_reports(self) -> list[bytes]:
        out, self.pending = self.pending, []
        return out

    def close(self) -> None:
        self.closed = True


def _hidraw(root: Path, name: str, product: int, interface: int, serial: str) -> None:
    device = root / name / "device"
    device.mkdir(parents=True)
    (device / "uevent").write_text(
        f"HID_ID=0003:00000C70:0000{product:04X}\n"
        f"HID_NAME=Aqua Computer GmbH & Co. KG\n"
        f"HID_PHYS=usb-20980000.usb-1.1/input{interface}\n"
        f"HID_UNIQ={serial}\n"
    )


@pytest.fixture
def rig(tmp_path: Path):
    sysfs = tmp_path / "hidraw"
    dev = tmp_path / "dev"
    _hidraw(sysfs, "hidraw2", 0xF001, 2, "12345-54321")
    return sysfs, dev


def _run(rig, reports: list[bytes], **kwargs) -> tuple[int, str, ScriptedController]:
    sysfs, dev = rig
    clock = FakeClock()
    controller = ScriptedController(clock, reports)
    out = io.StringIO()
    code = aquabus_watch.watch(
        reports=len(reports),
        sysfs_root=sysfs,
        dev_dir=dev,
        opener=lambda info: controller,
        clock=clock,
        out=out,
        **kwargs,
    )
    return code, out.getvalue(), controller


#: The cadence the two fixtures play back: one report whose electrical sample landed
#: in the on phase of the output's 20 % duty, then three whose sample missed.
_CADENCE = ("m", "s", "s", "s")


def _sampling_pattern(count: int) -> list[bytes]:
    """The reports in that cadence, `count` of them."""
    measuring, substituted = fixture_bytes(MEASURING), fixture_bytes(SUBSTITUTED)
    return [measuring if _CADENCE[i % len(_CADENCE)] == "m" else substituted for i in range(count)]


def test_it_measures_the_share_of_sampling_reports_and_closes_the_device(rig) -> None:
    """Item 115: how many reports carried a sample **against that block's duty**, how far
    apart they were, and whether the blocks are sampled together. The share is the
    measurement -- read on its own it says nothing, because it follows the duty, and the
    duty is printed beside it so a second run at another duty can be compared."""
    code, text, controller = _run(rig, _sampling_pattern(40))
    assert code == 0 and controller.closed
    assert "40 status reports in 39.0 s" in text
    assert "every 1.00 s (min 1, max 1, sd 0.00)" in text
    assert "pwm7  duty  20.00 %  10 of 40 reports (25%)  every 4.00 reports" in text
    assert "every 4.00 s (min 4, max 4, sd 0.00)" in text
    assert (
        "10 of 10 sampling reports carried every block: one sampling instant for the "
        "whole device" in text
    )


def test_it_reports_a_healthy_bus_as_quiet(rig) -> None:
    """Item 92: the rule is meant to stay silent on a healthy bus, and this is how a run
    on the hardware shows that -- presence is read from the speed field, so a report whose
    electrical sample missed never counts as an absence."""
    _code, text, _controller = _run(rig, _sampling_pattern(40))
    assert "a device answers on aquabus in 40 of 40 reports" in text
    assert "no report showed the bus empty: the bus-absent rule stays quiet" in text
    assert "aquabus temperature slots with a value: bus2 23.68..23.68 degC" in text


def test_it_measures_how_long_the_bus_was_empty(rig) -> None:
    """A run over a bus device that goes away: the longest stretch with no device, which
    is what ``bus_absent_s`` is judged against."""
    reports = _sampling_pattern(6) + [fixture_bytes(NO_DEVICE)] * 9 + _sampling_pattern(4)
    _code, text, _controller = _run(rig, reports)
    assert "a device answers on aquabus in 10 of 19 reports" in text
    assert "longest stretch with no device: 9 reports, 8.0 s" in text


def test_the_unidentified_field_is_shown_with_the_duty_and_the_current(rig) -> None:
    """Item 114: the raw ``u16`` next to the duty, current and power of the same sample.
    The tool names nothing and weights nothing -- the four are one instant inside the PWM
    cycle, so it prints the combinations and how often each occurred."""
    _code, text, _controller = _run(rig, _sampling_pattern(8))
    assert "the unidentified u16 at +0x0A" in text
    assert "pwm7  duty  20.00 %     6 mA   0.07 W  +0x0A    26  x2" in text
    assert "+0x0A x duty" not in text  # the withdrawn relation is not offered to a reader
    # An output with no fan is in the same sample, and measures 0 mA and 0 there.
    assert "pwm5  duty  20.00 %     0 mA   0.00 W  +0x0A     0" in text


def test_the_raw_listing_marks_the_reports_that_carried_a_sample(rig) -> None:
    _code, text, _controller = _run(rig, _sampling_pattern(5), raw=True)
    assert "every report, aquabus blocks (rpm, duty, V, mA, cW, +0x0A):" in text
    assert text.count("*") >= 2 * len(AQUAERO.aquabus_outputs)  # two sampling reports


def test_no_device_found_is_exit_1(rig, tmp_path: Path) -> None:
    empty = tmp_path / "none"
    empty.mkdir()
    out = io.StringIO()
    code = aquabus_watch.watch(sysfs_root=empty, dev_dir=tmp_path / "dev", reports=1, out=out)
    assert code == 1 and "no aquaero" in out.getvalue()


def test_main_rejects_a_non_positive_run(capsys) -> None:
    assert aquabus_watch.main(["--reports", "0"]) == 2
    assert "--reports must be > 0" in capsys.readouterr().err
