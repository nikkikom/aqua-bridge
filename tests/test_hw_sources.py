"""Tests for aqua_bridge.hw.sources: CompositeSource + build_composite_from_config.

PROJECT.md section 3 (Track B) / the DAS plan section 1 and section 12 Q1
(the Quadro possibly needing its own USB device).
"""

from __future__ import annotations

from pathlib import Path

import pytest

from aqua_bridge.hw.map import DeviceUnavailable, HwmonMap
from aqua_bridge.hw.onewire import W1Source
from aqua_bridge.hw.sources import CompositeSource, build_composite_from_config
from aqua_bridge.hw.xt6 import Xt6Adapter
from aqua_bridge.model import ConfigError, Mode, MpcCommand


class _FakeClock:
    def __init__(self, t: float = 100.0) -> None:
        self.t = t

    def __call__(self) -> float:
        return self.t


def _make_hwmon_device(root: Path, dev_name: str, name: str, files: dict[str, str]) -> Path:
    dev = root / dev_name
    dev.mkdir(parents=True)
    (dev / "name").write_text(name + "\n")
    for fname, content in files.items():
        (dev / fname).write_text(content)
    return dev


def _adapter(dev_dir: Path, root: Path, hwmon_name: str, pwm_map, temp_map, fan_map=None):
    hmap = HwmonMap(
        hwmon_name=hwmon_name, pwm_map=pwm_map, temp_map=temp_map, fan_map=fan_map or {}, root=root
    )
    return Xt6Adapter(hmap, clock=_FakeClock())


# --- CompositeSource: read/apply merging ------------------------------------------------


def test_requires_at_least_one_hwmon_device() -> None:
    with pytest.raises(ValueError, match="at least one hwmon device"):
        CompositeSource([])


def test_read_merges_two_hwmon_devices(tmp_path: Path) -> None:
    root = tmp_path / "hwmon"
    dev_a = _make_hwmon_device(
        root, "hwmon0", "aquaero", {"temp1_input": "35000", "pwm1": "128", "fan1_input": "900"}
    )
    dev_b = _make_hwmon_device(root, "hwmon1", "quadro", {"temp1_input": "28000", "pwm1": "64"})
    adapter_a = _adapter(
        dev_a, root, "aquaero", {"radiator": "pwm1"}, {"air_z0": "temp1"}, {"radiator": "fan1"}
    )
    adapter_b = _adapter(dev_b, root, "quadro", {"exhaust": "pwm1"}, {"air_z1": "temp1"})
    composite = CompositeSource([adapter_a, adapter_b], clock=_FakeClock(50.0))

    obs = composite.read()

    assert obs.temps == {"air_z0": pytest.approx(35.0), "air_z1": pytest.approx(28.0)}
    assert obs.pwm == {"radiator": pytest.approx(128 / 255), "exhaust": pytest.approx(64 / 255)}
    assert obs.rpm == {"radiator": pytest.approx(900.0)}
    assert obs.ts == 50.0


def test_read_merges_onewire_temps(tmp_path: Path) -> None:
    root = tmp_path / "hwmon"
    dev = _make_hwmon_device(root, "hwmon0", "aquaero", {"temp1_input": "35000", "pwm1": "128"})
    adapter = _adapter(dev, root, "aquaero", {"radiator": "pwm1"}, {"air_z0": "temp1"})

    class _FakeOnewire:
        def read(self):
            return {"prox_b01": 41.5}

    composite = CompositeSource([adapter], _FakeOnewire(), clock=_FakeClock(1.0))
    obs = composite.read()
    assert obs.temps == {"air_z0": pytest.approx(35.0), "prox_b01": pytest.approx(41.5)}


def test_read_propagates_a_vanished_device(tmp_path: Path) -> None:
    import shutil

    root = tmp_path / "hwmon"
    dev = _make_hwmon_device(root, "hwmon0", "aquaero", {"temp1_input": "35000", "pwm1": "128"})
    adapter = _adapter(dev, root, "aquaero", {"radiator": "pwm1"}, {"air_z0": "temp1"})
    composite = CompositeSource([adapter], clock=_FakeClock())
    composite.read()  # resolves fine once

    shutil.rmtree(dev)
    with pytest.raises(DeviceUnavailable):
        composite.read()


def test_apply_writes_each_channel_to_its_own_device(tmp_path: Path) -> None:
    root = tmp_path / "hwmon"
    dev_a = _make_hwmon_device(root, "hwmon0", "aquaero", {"pwm1": "0", "temp1_input": "1000"})
    dev_b = _make_hwmon_device(root, "hwmon1", "quadro", {"pwm1": "0", "temp1_input": "1000"})
    adapter_a = _adapter(dev_a, root, "aquaero", {"radiator": "pwm1"}, {})
    adapter_b = _adapter(dev_b, root, "quadro", {"exhaust": "pwm1"}, {})
    composite = CompositeSource([adapter_a, adapter_b], clock=_FakeClock())

    composite.apply(MpcCommand(pwm={"radiator": 0.5, "exhaust": 1.0}, mode=Mode.AUTO))

    assert (dev_a / "pwm1").read_text() == str(round(0.5 * 255))
    assert (dev_b / "pwm1").read_text() == str(round(1.0 * 255))


def test_apply_stops_at_first_failing_device_but_earlier_writes_stand(tmp_path: Path) -> None:
    import shutil

    root = tmp_path / "hwmon"
    dev_a = _make_hwmon_device(root, "hwmon0", "aquaero", {"pwm1": "0", "temp1_input": "1000"})
    dev_b = _make_hwmon_device(root, "hwmon1", "quadro", {"pwm1": "0", "temp1_input": "1000"})
    adapter_a = _adapter(dev_a, root, "aquaero", {"radiator": "pwm1"}, {})
    adapter_b = _adapter(dev_b, root, "quadro", {"exhaust": "pwm1"}, {})
    composite = CompositeSource([adapter_a, adapter_b], clock=_FakeClock())
    shutil.rmtree(dev_b)

    with pytest.raises(DeviceUnavailable):
        composite.apply(MpcCommand(pwm={"radiator": 0.5, "exhaust": 1.0}, mode=Mode.AUTO))

    assert (dev_a / "pwm1").read_text() == str(round(0.5 * 255))  # device A's write stands


# --- build_composite_from_config: binding checks -----------------------------------------


_XT6_SECTION = {
    "hwmon_name": "aquaero",
    "fans": {"radiator": {"pwm": "pwm1", "rpm": "fan1"}},
    "temp_map": {"air_z0": "temp1"},
}


def test_single_xt6_device_is_accepted(tmp_path: Path) -> None:
    section = dict(_XT6_SECTION, root=str(tmp_path / "hwmon"))
    composite, release = build_composite_from_config(
        hwmon_section=(),
        xt6_section=section,
        onewire_section={},
        channels=("radiator",),
        temps=("air_z0",),
        dt=5.0,
    )
    assert isinstance(composite, CompositeSource)
    assert len(composite.hwmon) == 1
    assert composite.onewire is None
    assert release is None


def test_hwmon_list_of_two_devices_is_accepted(tmp_path: Path) -> None:
    root = str(tmp_path / "hwmon")
    hwmon_section = (
        {
            "name": "aquaero",
            "fans": {"radiator": {"pwm": "pwm1"}},
            "temp_map": {"air_z0": "temp1"},
            "root": root,
        },
        {
            "name": "quadro",
            "fans": {"exhaust": {"pwm": "pwm1"}},
            "temp_map": {"air_z1": "temp1"},
            "root": root,
        },
    )
    composite, release = build_composite_from_config(
        hwmon_section=hwmon_section,
        xt6_section={},
        onewire_section={},
        channels=("radiator", "exhaust"),
        temps=("air_z0", "air_z1"),
        dt=5.0,
    )
    assert len(composite.hwmon) == 2
    assert release is None


def test_xt6_and_hwmon_list_combine(tmp_path: Path) -> None:
    root = str(tmp_path / "hwmon")
    hwmon_section = (
        {
            "name": "quadro",
            "fans": {"exhaust": {"pwm": "pwm1"}},
            "temp_map": {"air_z1": "temp1"},
            "root": root,
        },
    )
    xt6_section = dict(_XT6_SECTION, root=root)
    composite, _release = build_composite_from_config(
        hwmon_section=hwmon_section,
        xt6_section=xt6_section,
        onewire_section={},
        channels=("radiator", "exhaust"),
        temps=("air_z0", "air_z1"),
        dt=5.0,
    )
    assert len(composite.hwmon) == 2


def test_onewire_fills_remaining_temps_and_release_stops_it(tmp_path: Path) -> None:
    root = str(tmp_path / "hwmon")
    w1_root = tmp_path / "w1"
    (w1_root / "w1_bus_master1" / "28-000000000001").mkdir(parents=True)
    (w1_root / "w1_bus_master1" / "therm_bulk_read").write_text("1")
    (w1_root / "w1_bus_master1" / "28-000000000001" / "temperature").write_text("22000")
    xt6_section = dict(_XT6_SECTION, root=root)
    onewire_section = {"sensors": {"prox_b01": "28-000000000001"}, "root": str(w1_root)}

    composite, release = build_composite_from_config(
        hwmon_section=(),
        xt6_section=xt6_section,
        onewire_section=onewire_section,
        channels=("radiator",),
        temps=("air_z0", "prox_b01"),
        dt=5.0,
    )
    assert isinstance(composite.onewire, W1Source)
    assert release is not None
    release()  # stops the reader thread(s); must not raise


def test_missing_rom_at_startup_does_not_block_the_build(tmp_path: Path) -> None:
    root = str(tmp_path / "hwmon")
    xt6_section = dict(_XT6_SECTION, root=root)
    onewire_section = {"sensors": {"prox_b01": "28-nope"}, "root": str(tmp_path / "w1_empty")}

    composite, release = build_composite_from_config(
        hwmon_section=(),
        xt6_section=xt6_section,
        onewire_section=onewire_section,
        channels=("radiator",),
        temps=("air_z0", "prox_b01"),
        dt=5.0,
    )
    assert composite.onewire.missing_roms() == ["28-nope"]
    if release is not None:
        release()


# --- CompositeSource: SMART wiring (plan section 1, milestone smart-agent) --------------


class _FakeSmart:
    def __init__(self, data: dict) -> None:
        self._data = data

    def snapshot(self) -> dict:
        return self._data


def test_read_has_no_inputs_key_when_smart_is_not_configured(tmp_path: Path) -> None:
    root = tmp_path / "hwmon"
    dev = _make_hwmon_device(root, "hwmon0", "aquaero", {"temp1_input": "35000", "pwm1": "128"})
    adapter = _adapter(dev, root, "aquaero", {"radiator": "pwm1"}, {"air_z0": "temp1"})
    composite = CompositeSource([adapter], clock=_FakeClock())
    obs = composite.read()
    assert obs.inputs == {}
    assert composite.smart is None


def test_read_puts_smart_snapshot_into_inputs(tmp_path: Path) -> None:
    root = tmp_path / "hwmon"
    dev = _make_hwmon_device(root, "hwmon0", "aquaero", {"temp1_input": "35000", "pwm1": "128"})
    adapter = _adapter(dev, root, "aquaero", {"radiator": "pwm1"}, {"air_z0": "temp1"})
    smart = _FakeSmart({"WD-ABC123": {"temp_c": 34.0, "age_s": 5.0, "model": "WDC WD40"}})
    composite = CompositeSource([adapter], smart=smart, clock=_FakeClock())

    obs = composite.read()

    assert obs.inputs == {
        "smart": {"WD-ABC123": {"temp_c": 34.0, "age_s": 5.0, "model": "WDC WD40"}}
    }


def test_read_reflects_an_empty_smart_snapshot(tmp_path: Path) -> None:
    root = tmp_path / "hwmon"
    dev = _make_hwmon_device(root, "hwmon0", "aquaero", {"temp1_input": "35000", "pwm1": "128"})
    adapter = _adapter(dev, root, "aquaero", {"radiator": "pwm1"}, {"air_z0": "temp1"})
    composite = CompositeSource([adapter], smart=_FakeSmart({}), clock=_FakeClock())
    obs = composite.read()
    assert obs.inputs == {"smart": {}}


def test_build_composite_from_config_passes_smart_through(tmp_path: Path) -> None:
    root = tmp_path / "hwmon"
    _make_hwmon_device(root, "hwmon0", "aquaero", {"temp1_input": "35000", "pwm1": "128"})
    section = dict(_XT6_SECTION, root=str(root))
    smart = _FakeSmart({"S1": {"temp_c": 30.0, "age_s": 1.0, "model": None}})
    composite, _release = build_composite_from_config(
        hwmon_section=(),
        xt6_section=section,
        onewire_section={},
        channels=("radiator",),
        temps=("air_z0",),
        dt=5.0,
        smart=smart,
    )
    assert composite.smart is smart
    assert composite.read().inputs == {
        "smart": {"S1": {"temp_c": 30.0, "age_s": 1.0, "model": None}}
    }


def test_no_device_configured_is_config_error() -> None:
    with pytest.raises(ConfigError, match="no hwmon device configured"):
        build_composite_from_config(
            hwmon_section=(),
            xt6_section={},
            onewire_section={},
            channels=("a",),
            temps=("t",),
            dt=5.0,
        )


def test_missing_temp_binding_is_config_error(tmp_path: Path) -> None:
    section = dict(_XT6_SECTION, root=str(tmp_path / "hwmon"))
    with pytest.raises(ConfigError, match=r"\['air_z1'\]"):
        build_composite_from_config(
            hwmon_section=(),
            xt6_section=section,
            onewire_section={},
            channels=("radiator",),
            temps=("air_z0", "air_z1"),
            dt=5.0,
        )


def test_missing_channel_binding_is_config_error(tmp_path: Path) -> None:
    section = dict(_XT6_SECTION, root=str(tmp_path / "hwmon"))
    with pytest.raises(ConfigError, match=r"\['intake'\]"):
        build_composite_from_config(
            hwmon_section=(),
            xt6_section=section,
            onewire_section={},
            channels=("radiator", "intake"),
            temps=("air_z0",),
            dt=5.0,
        )


def test_temp_bound_twice_across_devices_is_config_error(tmp_path: Path) -> None:
    root = str(tmp_path / "hwmon")
    hwmon_section = (
        {
            "name": "aquaero",
            "fans": {"radiator": {"pwm": "pwm1"}},
            "temp_map": {"air_z0": "temp1"},
            "root": root,
        },
        {
            "name": "quadro",
            "fans": {"exhaust": {"pwm": "pwm1"}},
            "temp_map": {"air_z0": "temp1"},
            "root": root,
        },
    )
    with pytest.raises(ConfigError, match="bound twice"):
        build_composite_from_config(
            hwmon_section=hwmon_section,
            xt6_section={},
            onewire_section={},
            channels=("radiator", "exhaust"),
            temps=("air_z0",),
            dt=5.0,
        )


def test_extra_bound_temp_name_is_config_error(tmp_path: Path) -> None:
    section = dict(_XT6_SECTION, root=str(tmp_path / "hwmon"))
    with pytest.raises(ConfigError, match="not in mpc.temps"):
        build_composite_from_config(
            hwmon_section=(),
            xt6_section=section,
            onewire_section={},
            channels=("radiator",),
            temps=(),  # air_z0 bound but not declared
            dt=5.0,
        )


def test_malformed_device_spec_is_config_error() -> None:
    with pytest.raises(ConfigError, match="hwmon\\[0\\] must be a mapping"):
        build_composite_from_config(
            hwmon_section=("not-a-mapping",),
            xt6_section={},
            onewire_section={},
            channels=(),
            temps=(),
            dt=5.0,
        )
