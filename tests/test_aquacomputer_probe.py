"""Tests for tools/aquacomputer_probe.py against a fake sysfs tree and fake controllers.

``tools/`` is not on ``pythonpath``, so this file adds it to ``sys.path`` itself
(the same way tests/test_w1_commission.py does).
"""

from __future__ import annotations

import io
import sys
from pathlib import Path

import pytest

from aqua_bridge.hw.aquacomputer import AQUAERO, QUADRO
from aqua_bridge.hw.hidraw import DeviceUnavailable, FeatureReportError
from aquacomputer_fakes import FakeClock, FakeController

TOOLS_DIR = Path(__file__).resolve().parent.parent / "tools"
if str(TOOLS_DIR) not in sys.path:
    sys.path.insert(0, str(TOOLS_DIR))

import aquacomputer_probe  # noqa: E402 -- must follow the sys.path tweak above


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
    _hidraw(sysfs, "hidraw0", 0xF001, 0, "12345-54321")  # keyboard interface: not listed
    _hidraw(sysfs, "hidraw2", 0xF001, 2, "12345-54321")
    _hidraw(sysfs, "hidraw5", 0xF00D, 1, "00000-11111")
    clock = FakeClock()
    controllers = {
        dev / "hidraw2": FakeController(AQUAERO, clock, node=str(dev / "hidraw2")),
        dev / "hidraw5": FakeController(
            QUADRO, clock, node=str(dev / "hidraw5"), serial="00000-11111"
        ),
    }

    def opener(info):
        controller = controllers[info.node]
        controller.emit()
        return controller

    return sysfs, dev, clock, controllers, opener


def _run(rig, **kwargs) -> tuple[int, str]:
    sysfs, dev, clock, _controllers, opener = rig
    out = io.StringIO()
    kwargs.setdefault("opener", opener)
    code = aquacomputer_probe.probe(sysfs_root=sysfs, dev_dir=dev, clock=clock, out=out, **kwargs)
    return code, out.getvalue()


def test_lists_and_decodes_both_devices_without_writing(rig) -> None:
    code, text = _run(rig)
    assert code == 0
    assert "aquaero  serial 12345-54321    interface 2" in text
    assert "quadro   serial 00000-11111    interface 1" in text
    assert "hidraw0" not in text
    # aquaero status: a connected input, a virtual sensor, the commanded output, flow
    assert "temp6     22.26" in text and "temp9     40.00" in text
    assert "pwm2/fan2    120 rpm  duty  14.12 %  12.09 V" in text
    assert "fan5 (flow)  0" in text
    # aquaero control report: source and limits
    assert "pwm2  duty   0.00 %  source 0x59  min 50.00 %  max 100.00 %  (does not follow" in text
    assert "preset)  mode pwm (0x0502)" in text and "mode dc (0x0501)" in text
    # Quadro: power cycles, duty from the control report
    assert "power cycles:" in text and "pwm3  duty 100.00 %" in text
    for controller in rig[3].values():
        assert controller.sets() == [] and controller.secondaries() == []
        assert len(controller.gets()) == 1 and controller.closed


def test_filters_by_kind_and_serial(rig) -> None:
    code, text = _run(rig, kinds=(QUADRO,))
    assert code == 0 and "aquaero" not in text
    code, text = _run(rig, serial="12345-54321")
    assert code == 0 and "quadro" not in text
    code, text = _run(rig, serial="99999-99999")
    assert code == 1 and "with serial '99999-99999'" in text


def test_nothing_found(tmp_path: Path) -> None:
    out = io.StringIO()
    assert aquacomputer_probe.probe(sysfs_root=tmp_path, out=out) == 1
    assert "no aquaero, quadro found" in out.getvalue()


def test_failures_are_reported_and_other_devices_still_probed(rig) -> None:
    _sysfs, dev, _clock, controllers, opener = rig
    controllers[dev / "hidraw2"].failures = [FeatureReportError("EPIPE")]
    silent = controllers[dev / "hidraw5"]

    def opener_without_quadro_report(info):
        if info.node == dev / "hidraw5":
            return silent  # never emits a status report
        return opener(info)

    code, text = _run(rig, opener=opener_without_quadro_report, timeout_s=1.5)
    assert code == 3
    assert "control report: EPIPE" in text
    assert "no status report within 1.5 s" in text


def test_open_failure_is_reported(rig) -> None:
    def refuse(info):
        raise DeviceUnavailable(f"cannot open {info.node}: permission denied")

    code, text = _run(rig, opener=refuse)
    assert code == 3 and "cannot open" in text


def test_main_parses_arguments(rig, capsys) -> None:
    sysfs, dev, _clock, _controllers, opener = rig
    code = aquacomputer_probe.main(
        ["--sysfs-root", str(sysfs), "--dev-dir", str(dev), "--device", "quadro"], opener=opener
    )
    assert code == 0 and "quadro" in capsys.readouterr().out
    assert aquacomputer_probe.main(["--timeout", "0"]) == 2
    with pytest.raises(SystemExit):
        aquacomputer_probe.main(["--device", "octo"])
