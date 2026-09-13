"""Tests for aqua_bridge.hw.xt6 against a fake hwmon tree, plus one live test.

PROJECT.md section 4.7 / section 3 (Track B) / section 2 (Risk, USB spike).
"""

from __future__ import annotations

import math
from pathlib import Path

import pytest

from aqua_bridge.hw.map import DeviceUnavailable, HwmonMap
from aqua_bridge.hw.xt6 import Xt6Adapter, build_map_from_config
from aqua_bridge.model import ConfigError, Mode, MpcCommand


def _make_device(root: Path, dev_name: str, name: str, files: dict[str, str]) -> Path:
    dev = root / dev_name
    dev.mkdir(parents=True)
    (dev / "name").write_text(name + "\n")
    for fname, content in files.items():
        (dev / fname).write_text(content)
    return dev


def _adapter(dev_dir: Path, root: Path, *, fan_map: dict[str, str] | None = None) -> Xt6Adapter:
    hmap = HwmonMap(
        hwmon_name="aquaero",
        pwm_map={"radiator": "pwm1", "intake": "pwm2"},
        temp_map={"coolant": "temp1", "air": "temp2"},
        fan_map=fan_map or {"radiator": "fan1"},
        root=root,
    )
    clock = _FakeClock()
    return Xt6Adapter(hmap, clock)


class _FakeClock:
    def __init__(self) -> None:
        self.t = 100.0

    def __call__(self) -> float:
        return self.t


def _basic_tree(tmp_path: Path) -> tuple[Path, Path]:
    root = tmp_path / "hwmon"
    dev = _make_device(
        root,
        "hwmon3",
        "aquaero",
        {
            "temp1_input": "35125",
            "temp2_input": "28000",
            "pwm1": "128",
            "pwm2": "64",
            "fan1_input": "913",
        },
    )
    return root, dev


def test_read_converts_millidegrees_and_pwm_and_rpm(tmp_path: Path) -> None:
    root, dev = _basic_tree(tmp_path)
    adapter = _adapter(dev, root)

    obs = adapter.read()

    assert obs.temps["coolant"] == pytest.approx(35.125)
    assert obs.temps["air"] == pytest.approx(28.0)
    assert obs.pwm["radiator"] == pytest.approx(128 / 255)
    assert obs.pwm["intake"] == pytest.approx(64 / 255)
    assert obs.rpm["radiator"] == pytest.approx(913.0)
    assert obs.ts == 100.0


def test_garbage_value_becomes_none_not_exception(tmp_path: Path) -> None:
    root, dev = _basic_tree(tmp_path)
    (dev / "temp1_input").write_text("not-a-number\n")
    (dev / "pwm2").write_text("")
    adapter = _adapter(dev, root)

    obs = adapter.read()

    assert obs.temps["coolant"] is None
    assert obs.pwm["intake"] is None
    # Unaffected channels still read fine.
    assert obs.temps["air"] == pytest.approx(28.0)


def test_missing_single_file_becomes_none(tmp_path: Path) -> None:
    root, dev = _basic_tree(tmp_path)
    (dev / "fan1_input").unlink()
    adapter = _adapter(dev, root)

    obs = adapter.read()

    assert obs.rpm["radiator"] is None


def test_device_gone_raises_device_unavailable_on_read(tmp_path: Path) -> None:
    root, dev = _basic_tree(tmp_path)
    adapter = _adapter(dev, root)
    adapter.read()  # resolves fine once

    import shutil

    shutil.rmtree(dev)

    with pytest.raises(DeviceUnavailable):
        adapter.read()


def test_apply_rounds_and_writes_only_configured_channels(tmp_path: Path) -> None:
    root, dev = _basic_tree(tmp_path)
    adapter = _adapter(dev, root)
    cmd = MpcCommand(pwm={"radiator": 0.5, "intake": 1.0}, mode=Mode.AUTO)

    adapter.apply(cmd)

    assert (dev / "pwm1").read_text() == str(round(0.5 * 255))
    assert (dev / "pwm2").read_text() == str(round(1.0 * 255))


def test_apply_sets_pwm_enable_before_first_write(tmp_path: Path) -> None:
    root, dev = _basic_tree(tmp_path)
    (dev / "pwm1_enable").write_text("0")
    adapter = _adapter(dev, root)
    cmd = MpcCommand(pwm={"radiator": 0.2, "intake": 0.2}, mode=Mode.AUTO)

    adapter.apply(cmd)

    assert (dev / "pwm1_enable").read_text() == "1"


def test_apply_does_not_rewrite_enable_if_already_manual(tmp_path: Path) -> None:
    root, dev = _basic_tree(tmp_path)
    (dev / "pwm1_enable").write_text("1")
    adapter = _adapter(dev, root)
    cmd = MpcCommand(pwm={"radiator": 0.2, "intake": 0.2}, mode=Mode.AUTO)

    adapter.apply(cmd)

    assert (dev / "pwm1_enable").read_text() == "1"


def test_release_restores_original_enable_value(tmp_path: Path) -> None:
    root, dev = _basic_tree(tmp_path)
    (dev / "pwm1_enable").write_text("0")
    adapter = _adapter(dev, root)
    cmd = MpcCommand(pwm={"radiator": 0.2, "intake": 0.2}, mode=Mode.AUTO)

    adapter.apply(cmd)
    assert (dev / "pwm1_enable").read_text() == "1"

    adapter.release()
    assert (dev / "pwm1_enable").read_text() == "0"


def test_release_without_apply_is_a_noop(tmp_path: Path) -> None:
    root, dev = _basic_tree(tmp_path)
    adapter = _adapter(dev, root)
    adapter.release()  # must not raise


def test_apply_rejects_nan_pwm_without_writing(tmp_path: Path) -> None:
    root, dev = _basic_tree(tmp_path)
    adapter = _adapter(dev, root)
    before = (dev / "pwm1").read_text()
    cmd = MpcCommand(pwm={"radiator": math.nan, "intake": 0.2}, mode=Mode.AUTO)

    with pytest.raises(ValueError):
        adapter.apply(cmd)

    # Clamping a bad command is not allowed to hide it: nothing was written.
    assert (dev / "pwm1").read_text() == before


def test_apply_rejects_out_of_range_pwm(tmp_path: Path) -> None:
    root, dev = _basic_tree(tmp_path)
    adapter = _adapter(dev, root)
    cmd = MpcCommand(pwm={"radiator": 1.5, "intake": 0.2}, mode=Mode.AUTO)

    with pytest.raises(ValueError):
        adapter.apply(cmd)


def test_apply_rejects_missing_channel(tmp_path: Path) -> None:
    root, dev = _basic_tree(tmp_path)
    adapter = _adapter(dev, root)
    cmd = MpcCommand(pwm={"radiator": 0.5}, mode=Mode.AUTO)  # "intake" missing

    with pytest.raises(ValueError):
        adapter.apply(cmd)


def test_apply_device_gone_raises_device_unavailable(tmp_path: Path) -> None:
    root, dev = _basic_tree(tmp_path)
    adapter = _adapter(dev, root)

    import shutil

    shutil.rmtree(dev)
    cmd = MpcCommand(pwm={"radiator": 0.5, "intake": 0.5}, mode=Mode.AUTO)

    with pytest.raises(DeviceUnavailable):
        adapter.apply(cmd)


def test_replug_that_resets_pwm_enable_is_put_back_to_manual(tmp_path: Path) -> None:
    """Review finding F5: after a USB dropout / re-plug the device comes back
    renumbered with ``pwmK_enable`` at the firmware value; every apply() must
    re-assert manual mode, not only the first one."""
    root, dev = _basic_tree(tmp_path)
    (dev / "pwm1_enable").write_text("0")
    (dev / "pwm2_enable").write_text("0")
    adapter = _adapter(dev, root)
    adapter.apply(MpcCommand(pwm={"radiator": 0.6, "intake": 0.6}, mode=Mode.AUTO))
    assert (dev / "pwm1_enable").read_text() == "1"

    new_dev = root / "hwmon7"
    dev.rename(new_dev)  # re-plug: new hwmon number, firmware defaults restored
    (new_dev / "pwm1_enable").write_text("0")
    (new_dev / "pwm2_enable").write_text("2")

    adapter.apply(MpcCommand(pwm={"radiator": 0.9, "intake": 0.9}, mode=Mode.AUTO))

    assert (new_dev / "pwm1").read_text() == str(round(0.9 * 255))
    assert (new_dev / "pwm1_enable").read_text() == "1"
    assert (new_dev / "pwm2_enable").read_text() == "1"
    # release() still restores what was there before the *first* write
    adapter.release()
    assert (new_dev / "pwm1_enable").read_text() == "0"
    assert (new_dev / "pwm2_enable").read_text() == "0"


def test_apply_rechecks_enable_every_tick_but_writes_only_when_needed(tmp_path: Path) -> None:
    root, dev = _basic_tree(tmp_path)
    enable = dev / "pwm1_enable"
    enable.write_text("1")
    adapter = _adapter(dev, root)
    cmd = MpcCommand(pwm={"radiator": 0.2, "intake": 0.2}, mode=Mode.AUTO)
    adapter.apply(cmd)
    before = enable.stat().st_mtime_ns
    enable.write_text("0")  # firmware took the channel back between ticks
    adapter.apply(cmd)
    assert enable.read_text() == "1"
    assert enable.stat().st_mtime_ns >= before


# --- build_map_from_config: startup cross-check against the mpc section (F4) ---------


_SECTION = {
    "hwmon_name": "aquaero",
    "fans": {
        "radiator": {"pwm": "pwm1", "rpm": "fan1"},
        "intake": {"pwm": "pwm2"},
    },
    "temp_map": {"coolant": "temp1", "air": "temp2"},
}
_CHANNELS = ("radiator", "intake")
_TEMPS = ("coolant", "air")


def test_build_map_matching_mpc_section_is_accepted() -> None:
    hmap = build_map_from_config(_SECTION, channels=_CHANNELS, temps=_TEMPS)
    assert hmap.pwm_map == {"radiator": "pwm1", "intake": "pwm2"}
    assert hmap.fan_map == {"radiator": "fan1"}  # rpm is optional per fan
    assert hmap.temp_map == _SECTION["temp_map"]
    # without the mpc tuples the check is skipped (mapping-only callers)
    assert build_map_from_config(_SECTION).pwm_map == hmap.pwm_map


def test_fans_entry_names_the_channel_once_for_pwm_and_rpm() -> None:
    """The point of the syntax: PWM and tachometer share one key, so they cannot drift
    apart into two differently spelt channels."""
    hmap = build_map_from_config(_SECTION, channels=_CHANNELS, temps=_TEMPS)
    assert set(hmap.fan_map) <= set(hmap.pwm_map)


def test_fans_parse_from_one_line_yaml_flow_mappings() -> None:
    import yaml

    section = yaml.safe_load(
        """
hwmon_name: aquaero
fans:
  radiator: {pwm: pwm1, rpm: fan1}
  intake:   {pwm: pwm2, rpm: fan2}
temp_map: {coolant: temp1, air: temp2}
"""
    )
    hmap = build_map_from_config(section, channels=_CHANNELS, temps=_TEMPS)
    assert hmap.pwm_map == {"radiator": "pwm1", "intake": "pwm2"}
    assert hmap.fan_map == {"radiator": "fan1", "intake": "fan2"}


def test_build_map_rejects_channel_missing_from_fans() -> None:
    """A channel in mpc.channels absent from xt6.fans would be silently never written."""
    section = dict(_SECTION, fans={"radiator": {"pwm": "pwm1"}})
    with pytest.raises(ConfigError, match="xt6.fans.*missing \\['intake'\\]"):
        build_map_from_config(section, channels=_CHANNELS, temps=_TEMPS)


def test_build_map_rejects_extra_channel_in_fans() -> None:
    section = dict(_SECTION, fans={**_SECTION["fans"], "pump": {"pwm": "pwm3"}})
    with pytest.raises(ConfigError, match="extra \\['pump'\\]"):
        build_map_from_config(section, channels=_CHANNELS, temps=_TEMPS)


def test_misspelt_channel_in_fans_is_a_startup_error() -> None:
    section = dict(
        _SECTION, fans={"radaitor": {"pwm": "pwm1", "rpm": "fan1"}, "intake": {"pwm": "pwm2"}}
    )
    with pytest.raises(ConfigError, match="missing \\['radiator'\\], extra \\['radaitor'\\]"):
        build_map_from_config(section, channels=_CHANNELS, temps=_TEMPS)


@pytest.mark.parametrize("legacy", ["map", "fan_map"])
def test_legacy_map_keys_are_rejected_with_a_hint(legacy: str) -> None:
    section = dict(_SECTION, **{legacy: {"radiator": "pwm1"}})
    with pytest.raises(ConfigError, match=f"xt6.{legacy} is no longer supported.*xt6.fans"):
        build_map_from_config(section, channels=_CHANNELS, temps=_TEMPS)


@pytest.mark.parametrize(
    ("fans", "match"),
    [
        (None, "xt6.fans is required"),
        (["radiator"], "xt6.fans must be a mapping"),
        ({"radiator": "pwm1", "intake": {"pwm": "pwm2"}}, "xt6.fans.radiator must be a mapping"),
        (
            {"radiator": {"rpm": "fan1"}, "intake": {"pwm": "pwm2"}},
            "xt6.fans.radiator.pwm is required",
        ),
        (
            {"radiator": {"pwm": "pwm1", "rmp": "fan1"}, "intake": {"pwm": "pwm2"}},
            "unknown key\\(s\\) \\['rmp'\\]",
        ),
        ({"radiator": {"pwm": "fan1"}, "intake": {"pwm": "pwm2"}}, "xt6.fans.radiator.pwm must be"),
        (
            {"radiator": {"pwm": "pwm1", "rpm": "pwm1"}, "intake": {"pwm": "pwm2"}},
            "xt6.fans.radiator.rpm must be",
        ),
        (
            {"radiator": {"pwm": "pwm1", "rpm": 1}, "intake": {"pwm": "pwm2"}},
            "xt6.fans.radiator.rpm must be",
        ),
        ({"radiator": {"pwm": "pwm1"}, "intake": {"pwm": "pwm1"}}, "same hwmon attribute 'pwm1'"),
        (
            {"radiator": {"pwm": "pwm1", "rpm": "fan1"}, "intake": {"pwm": "pwm2", "rpm": "fan1"}},
            "same hwmon attribute 'fan1'",
        ),
    ],
    ids=[
        "missing",
        "not_mapping",
        "entry_is_bare_string",
        "no_pwm",
        "typo_in_key",
        "pwm_names_a_tachometer",
        "rpm_names_a_pwm",
        "rpm_not_string",
        "two_fans_one_pwm",
        "two_fans_one_tachometer",
    ],
)
def test_malformed_fans_entry_is_config_error(fans, match: str) -> None:
    section = {k: v for k, v in _SECTION.items() if k != "fans"}
    if fans is not None:
        section["fans"] = fans
    with pytest.raises(ConfigError, match=match):
        build_map_from_config(section, channels=_CHANNELS, temps=_TEMPS)


def test_temp_map_attribute_must_be_a_temp_input() -> None:
    section = dict(_SECTION, temp_map={"coolant": "fan1", "air": "temp2"})
    with pytest.raises(ConfigError, match="xt6.temp_map.coolant must be"):
        build_map_from_config(section, channels=_CHANNELS, temps=_TEMPS)


@pytest.mark.parametrize(
    "temp_map",
    [
        {"coolant": "temp1", "air": "temp2", "ambient": "temp3"},  # extra -> every tick untrusted
        {"coolant": "temp1"},  # missing -> every tick untrusted
    ],
    ids=["extra", "missing"],
)
def test_build_map_rejects_temp_map_not_equal_to_mpc_temps(temp_map) -> None:
    """Every observation would carry an unknown / missing key: permanent fallback with no
    startup error (section 3 gate rule 2)."""
    section = dict(_SECTION, temp_map=temp_map)
    with pytest.raises(ConfigError, match="xt6.temp_map"):
        build_map_from_config(section, channels=_CHANNELS, temps=_TEMPS)


@pytest.mark.parametrize(
    "section",
    [
        {},
        {"hwmon_name": ""},
        {"hwmon_name": "aquaero", "fans": ["pwm1"]},
        {"hwmon_name": "aquaero", "fans": {"a": {"pwm": "pwm1"}, "b": {"pwm": "pwm1"}}},
        {"hwmon_name": "aquaero", "fans": {"a": {"pwm": "pwm1"}}, "temp_map": ["temp1"]},
    ],
    ids=["no_name", "empty_name", "fans_not_mapping", "duplicate_target", "temp_map_not_mapping"],
)
def test_build_map_malformed_section_is_config_error(section) -> None:
    with pytest.raises(ConfigError):
        build_map_from_config(section)


def test_silently_unwritten_channel_is_caught_at_build_time(tmp_path: Path) -> None:
    """The adapter itself only writes mapped channels (documented); the guard is the
    startup check, so the two together never leave a configured fan untouched."""
    root, dev = _basic_tree(tmp_path)
    hmap = HwmonMap(
        hwmon_name="aquaero",
        pwm_map={"radiator": "pwm1"},
        temp_map={"coolant": "temp1", "air": "temp2"},
        root=root,
    )
    Xt6Adapter(hmap, _FakeClock()).apply(
        MpcCommand(pwm={"radiator": 0.8, "intake": 0.8}, mode=Mode.FALLBACK)
    )
    assert (dev / "pwm2").read_text() == "64"  # untouched: exactly what the check prevents
    with pytest.raises(ConfigError):
        build_map_from_config(
            {
                "hwmon_name": "aquaero",
                "fans": {"radiator": {"pwm": "pwm1"}},
                "temp_map": hmap.temp_map,
            },
            channels=("radiator", "intake"),
            temps=("coolant", "air"),
        )


@pytest.mark.hardware
def test_live_read_and_writeback(aquaero_hwmon: Path) -> None:
    """Live USB test: skipped off-Pi by conftest's aquaero_hwmon fixture.

    Reads the real device once and writes back only the PWM value already
    in effect (never changes fan speed), then releases control back to
    firmware curves.
    """
    pwm_files = sorted(aquaero_hwmon.glob("pwm[0-9]"))
    assert pwm_files, "expected at least one pwmN file on the live aquaero device"

    pwm_map = {f"ch{i}": p.name for i, p in enumerate(pwm_files)}
    temp_files = sorted(aquaero_hwmon.glob("temp[0-9]_input"))
    temp_map = {f"t{i}": p.name.removesuffix("_input") for i, p in enumerate(temp_files)}

    hmap = HwmonMap(
        hwmon_name="aquaero",
        pwm_map=pwm_map,
        temp_map=temp_map,
        root=aquaero_hwmon.parent,
    )
    adapter = Xt6Adapter(hmap, clock=lambda: 0.0)

    obs = adapter.read()
    assert obs.pwm  # at least the configured channels are present

    current_pwm = {ch: (value if value is not None else 0.0) for ch, value in obs.pwm.items()}
    cmd = MpcCommand(pwm=current_pwm, mode=Mode.AUTO)
    adapter.apply(cmd)
    adapter.release()
