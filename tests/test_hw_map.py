"""Tests for aqua_bridge.hw.map against a fake hwmon tree (no hardware).

PROJECT.md section 4.7 / section 3 (Track B).
"""

from __future__ import annotations

import ast
from pathlib import Path

import pytest

from aqua_bridge.hw.map import DeviceUnavailable, HwmonMap


def _make_device(root: Path, dev_name: str, name: str, files: dict[str, str]) -> Path:
    dev = root / dev_name
    dev.mkdir(parents=True)
    (dev / "name").write_text(name + "\n")
    for fname, content in files.items():
        (dev / fname).write_text(content)
    return dev


def _aquaero_files() -> dict[str, str]:
    return {
        "temp1_input": "35000",
        "temp2_input": "28000",
        "pwm1": "128",
        "pwm2": "64",
        "fan1_input": "900",
    }


def test_resolves_device_by_name_not_number(tmp_path: Path) -> None:
    root = tmp_path / "hwmon"
    _make_device(root, "hwmon0", "other_chip", {"temp1_input": "1000"})
    dev1 = _make_device(root, "hwmon1", "aquaero", _aquaero_files())

    hmap = HwmonMap(
        hwmon_name="aquaero",
        pwm_map={"radiator": "pwm1", "intake": "pwm2"},
        temp_map={"coolant": "temp1", "air": "temp2"},
        fan_map={"radiator": "fan1"},
        root=root,
    )
    resolved = hmap.resolve()
    assert resolved.device_dir == dev1
    assert resolved.pwm["radiator"] == dev1 / "pwm1"
    assert resolved.pwm["intake"] == dev1 / "pwm2"
    assert resolved.temp["coolant"] == dev1 / "temp1_input"
    assert resolved.fan["radiator"] == dev1 / "fan1_input"


def test_renumbering_after_replug_is_transparent(tmp_path: Path) -> None:
    root = tmp_path / "hwmon"
    dev1 = _make_device(root, "hwmon1", "aquaero", _aquaero_files())

    hmap = HwmonMap(
        hwmon_name="aquaero",
        pwm_map={"radiator": "pwm1"},
        temp_map={"coolant": "temp1"},
        root=root,
    )
    first = hmap.resolve()
    assert first.device_dir == dev1

    # Simulate a re-plug: the device disappears and reappears renumbered.
    new_dev = root / "hwmon7"
    dev1.rename(new_dev)

    second = hmap.resolve()
    assert second.device_dir == new_dev
    assert second.pwm["radiator"] == new_dev / "pwm1"


def test_missing_device_raises_device_unavailable(tmp_path: Path) -> None:
    root = tmp_path / "hwmon"
    root.mkdir()
    _make_device(root, "hwmon0", "not_aquaero", {})

    hmap = HwmonMap(hwmon_name="aquaero", pwm_map={}, temp_map={}, root=root)
    with pytest.raises(DeviceUnavailable):
        hmap.resolve()


def test_missing_root_raises_device_unavailable(tmp_path: Path) -> None:
    hmap = HwmonMap(hwmon_name="aquaero", pwm_map={}, temp_map={}, root=tmp_path / "nope")
    with pytest.raises(DeviceUnavailable):
        hmap.resolve()


def test_missing_mapped_attribute_raises_clear_error(tmp_path: Path) -> None:
    root = tmp_path / "hwmon"
    _make_device(root, "hwmon0", "aquaero", {"temp1_input": "1000"})

    hmap = HwmonMap(
        hwmon_name="aquaero",
        pwm_map={"radiator": "pwm1"},  # pwm1 does not exist in the fake tree
        temp_map={},
        root=root,
    )
    with pytest.raises(ValueError, match="pwm1"):
        hmap.resolve()


def test_duplicate_targets_rejected_at_construction() -> None:
    with pytest.raises(ValueError):
        HwmonMap(
            hwmon_name="aquaero",
            pwm_map={"radiator": "pwm1", "intake": "pwm1"},
            temp_map={},
        )


def test_duplicate_targets_checked_per_mapping_not_across_namespaces() -> None:
    # "pwm1" and "temp1" are different attribute namespaces, so reusing the
    # digit across pwm_map/temp_map is fine; duplicating within one mapping
    # (two temps both pointing at temp1) is rejected.
    HwmonMap(
        hwmon_name="aquaero",
        pwm_map={"radiator": "pwm1"},
        temp_map={"coolant": "temp1"},
    )
    with pytest.raises(ValueError):
        HwmonMap(
            hwmon_name="aquaero",
            pwm_map={},
            temp_map={"coolant": "temp1", "air": "temp1"},
        )


def test_default_root_is_sys_class_hwmon() -> None:
    hmap = HwmonMap(hwmon_name="aquaero", pwm_map={}, temp_map={})
    assert str(hmap.root) == "/sys/class/hwmon"


def test_hw_modules_do_not_import_control() -> None:
    """Static check (section 3): hardware must not import the MPC side."""
    hw_dir = Path(__file__).resolve().parent.parent / "src" / "aqua_bridge" / "hw"
    for path in sorted(hw_dir.glob("*.py")):
        tree = ast.parse(path.read_text(), filename=str(path))
        for node in ast.walk(tree):
            if isinstance(node, ast.Import):
                names = [alias.name for alias in node.names]
            elif isinstance(node, ast.ImportFrom):
                names = [node.module] if node.module else []
            else:
                continue
            for name in names:
                assert name is not None
                assert not name.startswith("aqua_bridge.control"), (
                    f"{path} imports {name!r}, hardware must not import control/mpc"
                )
                assert name != "aqua_bridge.control"
