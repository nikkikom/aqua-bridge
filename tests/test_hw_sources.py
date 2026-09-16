"""Tests for aqua_bridge.hw.sources: CompositeSource + build_composite_from_config.

PROJECT.md section 3 (Track B) / the DAS plan section 1 and section 12 Q1
(the Quadro on its own USB port).
"""

from __future__ import annotations

from pathlib import Path

import pytest

from aqua_bridge.control.loop import Loop, TickResult
from aqua_bridge.control.supervisor import Supervisor
from aqua_bridge.hw.aquacomputer import AQUAERO, QUADRO, control_duty
from aqua_bridge.hw.aquacomputer_adapter import AquacomputerAdapter, DeviceBinding
from aqua_bridge.hw.hidraw import DeviceUnavailable
from aqua_bridge.hw.onewire import W1Source
from aqua_bridge.hw.sources import CompositeSource, build_composite_from_config
from aqua_bridge.model import ConfigError, Mode, MpcCommand
from aquacomputer_fakes import (
    FakeBus,
    FakeClock,
    FakeController,
    FakeSleep,
    aquabus_aquaero,
    fixture_bytes,
)


def _messages(caplog, level: str) -> list[str]:
    return [r.getMessage() for r in caplog.records if r.levelname == level]


def _fleet(clock: FakeClock | None = None):
    clock = clock or FakeClock()
    aquaero = FakeController(AQUAERO, clock, node="/dev/hidraw2")
    quadro = FakeController(QUADRO, clock, node="/dev/hidraw5", serial="00000-11111")
    bus = FakeBus(aquaero, quadro)
    sleep = FakeSleep(clock)
    a = AquacomputerAdapter(
        DeviceBinding(
            kind=AQUAERO,
            pwm_map={"radiator": 2},
            fan_map={"radiator": 2},
            temp_map={"air_z0": "temp6"},
        ),
        clock=clock,
        sleep=sleep,
        opener=bus,
    )
    q = AquacomputerAdapter(
        DeviceBinding(kind=QUADRO, pwm_map={"exhaust": 1}, temp_map={"air_z1": "temp2"}),
        clock=clock,
        sleep=sleep,
        opener=bus,
    )
    return a, q, aquaero, quadro, clock


# --- CompositeSource: read/apply merging ------------------------------------------------


def test_requires_at_least_one_device() -> None:
    with pytest.raises(ValueError, match="at least one Aqua Computer device"):
        CompositeSource([])


def test_read_merges_two_controllers() -> None:
    a, q, _aquaero, _quadro, clock = _fleet(FakeClock(50.0))
    composite = CompositeSource([a, q], clock=clock)

    obs = composite.read()

    assert obs.temps == {"air_z0": pytest.approx(22.26), "air_z1": pytest.approx(21.89)}
    assert obs.pwm == {"radiator": pytest.approx(0.1412), "exhaust": 1.0}
    assert obs.rpm == {"radiator": 120.0}
    assert obs.ts == 50.0


def test_read_merges_onewire_temps() -> None:
    a, _q, _aquaero, _quadro, clock = _fleet()

    class _FakeOnewire:
        def read(self):
            return {"prox_b01": 41.5}

    composite = CompositeSource([a], _FakeOnewire(), clock=clock)
    obs = composite.read()
    assert obs.temps == {"air_z0": pytest.approx(22.26), "prox_b01": pytest.approx(41.5)}


def test_read_propagates_a_vanished_device() -> None:
    a, q, _aquaero, quadro, clock = _fleet()
    composite = CompositeSource([a, q], clock=clock)
    composite.read()

    quadro.gone = True
    with pytest.raises(DeviceUnavailable, match="quadro"):
        composite.read()


def test_read_with_the_first_device_gone_still_drains_the_others() -> None:
    """Review finding: a failing first device must not leave the next device's hidraw
    queue undrained; the observation is still lost, the error names the failed device."""
    a, q, aquaero, quadro, clock = _fleet()
    composite = CompositeSource([a, q], clock=clock)
    composite.read()
    aquaero.gone = True
    quadro.emit(3)
    clock.advance(1.0)

    with pytest.raises(
        DeviceUnavailable, match=r"read failed on 1 of 2 device\(s\): aquaero"
    ) as exc:
        composite.read()

    assert quadro.pending == []  # read anyway
    assert "quadro" not in str(exc.value)
    assert isinstance(exc.value.__cause__, DeviceUnavailable)
    assert q.last_status is not None


def test_read_names_every_failed_device_and_chains_the_first() -> None:
    a, q, aquaero, quadro, clock = _fleet()
    composite = CompositeSource([a, q], clock=clock)
    composite.read()
    aquaero.gone = True
    quadro.gone = True
    with pytest.raises(DeviceUnavailable, match="2 of 2") as exc:
        composite.read()
    assert "aquaero" in str(exc.value) and "quadro" in str(exc.value)
    assert "hidraw2" in str(exc.value.__cause__)  # the aquaero, first in the list


def test_apply_writes_each_channel_to_its_own_device_with_one_set_each() -> None:
    a, q, aquaero, quadro, clock = _fleet()
    composite = CompositeSource([a, q], clock=clock)

    composite.apply(MpcCommand(pwm={"radiator": 0.5, "exhaust": 0.25}, mode=Mode.AUTO))

    assert len(aquaero.sets()) == 1 and len(quadro.sets()) == 1
    assert control_duty(AQUAERO, aquaero.ctrl, 1) == 5000
    assert control_duty(QUADRO, quadro.ctrl, 0) == 2500


def test_apply_with_the_first_device_gone_still_writes_the_others() -> None:
    """Review finding: the fallback ramp and the stop write must reach every healthy
    controller even when an earlier one in the list has vanished."""
    a, q, aquaero, quadro, clock = _fleet()
    composite = CompositeSource([a, q], clock=clock)
    composite.apply(MpcCommand(pwm={"radiator": 0.3, "exhaust": 0.3}, mode=Mode.AUTO))
    aquaero.gone = True
    clock.advance(1.0)

    fallback = MpcCommand(pwm={"radiator": 0.8, "exhaust": 0.8}, mode=Mode.FALLBACK)
    with pytest.raises(
        DeviceUnavailable, match=r"apply failed on 1 of 2 device\(s\): aquaero"
    ) as exc:
        composite.apply(fallback)

    assert control_duty(QUADRO, quadro.ctrl, 0) == 8000  # the healthy device got it
    assert len(quadro.sets()) == 2
    assert isinstance(exc.value.__cause__, DeviceUnavailable)


def test_apply_with_the_last_device_gone_keeps_the_earlier_write() -> None:
    a, q, aquaero, quadro, clock = _fleet()
    composite = CompositeSource([a, q], clock=clock)
    quadro.gone = True

    with pytest.raises(DeviceUnavailable, match="quadro"):
        composite.apply(MpcCommand(pwm={"radiator": 0.5, "exhaust": 1.0}, mode=Mode.AUTO))

    assert control_duty(AQUAERO, aquaero.ctrl, 1) == 5000


def test_apply_with_a_bad_command_raises_value_error_and_sends_nothing() -> None:
    a, q, aquaero, quadro, clock = _fleet()
    composite = CompositeSource([a, q], clock=clock)
    with pytest.raises(ValueError, match="2 of 2"):
        composite.apply(MpcCommand(pwm={"radiator": float("nan")}, mode=Mode.AUTO))
    assert aquaero.ops == [] and quadro.ops == []


# --- CompositeSource: SMART wiring (plan section 1, milestone smart-agent) --------------


class _FakeSmart:
    def __init__(self, data: dict) -> None:
        self._data = data

    def snapshot(self) -> dict:
        return self._data


def test_read_has_no_smart_key_when_smart_is_not_configured() -> None:
    a, _q, _aquaero, _quadro, clock = _fleet()
    composite = CompositeSource([a], clock=clock)
    assert "smart" not in composite.read().inputs
    assert composite.smart is None


def test_read_puts_smart_snapshot_into_inputs() -> None:
    a, _q, _aquaero, _quadro, clock = _fleet()
    smart = _FakeSmart({"WD-ABC123": {"temp_c": 34.0, "age_s": 5.0, "model": "WDC WD40"}})
    composite = CompositeSource([a], smart=smart, clock=clock)
    assert composite.read().inputs["smart"] == {
        "WD-ABC123": {"temp_c": 34.0, "age_s": 5.0, "model": "WDC WD40"}
    }


def test_read_reflects_an_empty_smart_snapshot() -> None:
    a, _q, _aquaero, _quadro, clock = _fleet()
    composite = CompositeSource([a], smart=_FakeSmart({}), clock=clock)
    assert composite.read().inputs["smart"] == {}


# --- build_composite_from_config: binding checks -----------------------------------------


_XT6_SECTION = {
    "device": "aquaero",
    "fans": {"radiator": {"pwm": "pwm1", "rpm": "fan1"}},
    "temp_map": {"air_z0": "temp1"},
}
_QUADRO_ENTRY = {
    "device": "quadro",
    "fans": {"exhaust": {"pwm": "pwm1"}},
    "temp_map": {"air_z1": "temp1"},
}


def _build(**overrides):
    kwargs = dict(
        aquacomputer_section=(),
        xt6_section={},
        onewire_section={},
        channels=("radiator",),
        temps=("air_z0",),
        dt=5.0,
    )
    kwargs.update(overrides)
    return build_composite_from_config(**kwargs)


def test_single_xt6_device_is_accepted_and_nothing_is_opened() -> None:
    bus = FakeBus()
    composite, release = _build(xt6_section=dict(_XT6_SECTION, prefer="hwmon"), opener=bus)
    assert isinstance(composite, CompositeSource)
    assert len(composite.devices) == 1 and composite.devices[0].kind is AQUAERO
    assert composite.onewire is None and release is None
    assert bus.opened == []


def test_aquacomputer_list_of_two_devices_is_accepted() -> None:
    composite, release = _build(
        aquacomputer_section=(dict(_XT6_SECTION), _QUADRO_ENTRY),
        channels=("radiator", "exhaust"),
        temps=("air_z0", "air_z1"),
    )
    assert [d.kind for d in composite.devices] == [AQUAERO, QUADRO]
    assert release is None


def test_timing_keys_reach_each_adapter() -> None:
    composite, _ = _build(
        aquacomputer_section=(
            dict(_XT6_SECTION, ctrl_gap_ms=50),
            dict(_QUADRO_ENTRY, ctrl_retries=4),
        ),
        channels=("radiator", "exhaust"),
        temps=("air_z0", "air_z1"),
    )
    assert composite.devices[0].timing.ctrl_gap_ms == 50
    assert composite.devices[1].timing.ctrl_retries == 4


def test_xt6_and_aquacomputer_list_combine() -> None:
    composite, _release = _build(
        aquacomputer_section=(_QUADRO_ENTRY,),
        xt6_section=_XT6_SECTION,
        channels=("radiator", "exhaust"),
        temps=("air_z0", "air_z1"),
    )
    assert len(composite.devices) == 2


def test_two_devices_of_one_kind_need_distinct_serials() -> None:
    second = {"device": "aquaero", "fans": {"exhaust": {"pwm": "pwm1"}}, "temp_map": {}}
    kwargs = dict(channels=("radiator", "exhaust"), temps=("air_z0",))
    with pytest.raises(ConfigError, match="can both open the same aquaero.*distinct 'serial:'"):
        _build(aquacomputer_section=(_XT6_SECTION, second), **kwargs)
    with pytest.raises(ConfigError, match="can both open the same aquaero"):
        _build(
            aquacomputer_section=(
                dict(_XT6_SECTION, serial="12345-54321"),
                dict(second, serial="12345-54321"),
            ),
            **kwargs,
        )
    with pytest.raises(ConfigError, match="can both open the same aquaero"):
        _build(
            aquacomputer_section=(dict(second, serial="12345-54321"),),
            xt6_section=_XT6_SECTION,
            **kwargs,
        )
    composite, _ = _build(
        aquacomputer_section=(
            dict(_XT6_SECTION, serial="12345-54321"),
            dict(second, serial="00000-00001"),
        ),
        **kwargs,
    )
    assert [d.binding.serial for d in composite.devices] == ["12345-54321", "00000-00001"]


def test_onewire_fills_remaining_temps_and_release_stops_it(tmp_path: Path) -> None:
    w1_root = tmp_path / "w1"
    (w1_root / "w1_bus_master1" / "28-000000000001").mkdir(parents=True)
    (w1_root / "w1_bus_master1" / "therm_bulk_read").write_text("1")
    (w1_root / "w1_bus_master1" / "28-000000000001" / "temperature").write_text("22000")
    onewire_section = {"sensors": {"prox_b01": "28-000000000001"}, "root": str(w1_root)}

    composite, release = _build(
        xt6_section=_XT6_SECTION, onewire_section=onewire_section, temps=("air_z0", "prox_b01")
    )
    assert isinstance(composite.onewire, W1Source)
    assert release is not None
    release()  # stops the reader thread(s); must not raise


def test_missing_rom_at_startup_does_not_block_the_build(tmp_path: Path) -> None:
    onewire_section = {"sensors": {"prox_b01": "28-nope"}, "root": str(tmp_path / "w1_empty")}
    composite, release = _build(
        xt6_section=_XT6_SECTION, onewire_section=onewire_section, temps=("air_z0", "prox_b01")
    )
    assert composite.onewire.missing_roms() == ["28-nope"]
    if release is not None:
        release()


def test_build_composite_from_config_passes_smart_clock_sleep_and_opener_through() -> None:
    clock = FakeClock(7.0)
    device = FakeController(AQUAERO, clock)
    bus = FakeBus(device)
    smart = _FakeSmart({"S1": {"temp_c": 30.0, "age_s": 1.0, "model": None}})
    composite, _release = _build(xt6_section=_XT6_SECTION, smart=smart, clock=clock, opener=bus)
    assert composite.smart is smart
    obs = composite.read()
    assert obs.inputs["smart"] == {"S1": {"temp_c": 30.0, "age_s": 1.0, "model": None}}
    assert obs.ts == 7.0 and bus.opened == [device.node]


def test_build_checks_the_summed_worst_case_against_the_watchdog() -> None:
    from aqua_bridge.hw.aquacomputer_adapter import AquacomputerTiming

    dt, step = 5.0, 0.75  # _build's dt, a step bound
    total = (
        dt
        + step
        + AquacomputerTiming.for_kind(AQUAERO).worst_case_tick_s()
        + AquacomputerTiming.for_kind(QUADRO).worst_case_tick_s()
    )
    kwargs = dict(
        aquacomputer_section=(dict(_XT6_SECTION), _QUADRO_ENTRY),
        channels=("radiator", "exhaust"),
        temps=("air_z0", "air_z1"),
        step_bound_s=step,
    )
    with pytest.raises(ConfigError, match="aquacomputer\\[0\\] .*aquacomputer\\[1\\]"):
        _build(watchdog_s=total, **kwargs)
    _build(watchdog_s=total + 0.5, **kwargs)
    shorter = (dict(_XT6_SECTION, ctrl_budget_s=4.0), _QUADRO_ENTRY)  # a smaller budget fits
    _build(watchdog_s=total, **dict(kwargs, aquacomputer_section=shorter))


def test_no_device_configured_is_config_error() -> None:
    with pytest.raises(ConfigError, match="no Aqua Computer device configured.*'aquacomputer:'"):
        _build(channels=("a",), temps=("t",))


def test_missing_temp_binding_is_config_error() -> None:
    with pytest.raises(ConfigError, match=r"\['air_z1'\]"):
        _build(xt6_section=_XT6_SECTION, temps=("air_z0", "air_z1"))


def test_missing_channel_binding_is_config_error() -> None:
    with pytest.raises(ConfigError, match=r"\['intake'\]"):
        _build(xt6_section=_XT6_SECTION, channels=("radiator", "intake"))


def test_temp_bound_twice_across_devices_is_config_error() -> None:
    with pytest.raises(ConfigError, match="bound twice.*aquacomputer\\[0\\] \\(aquaero\\)"):
        _build(
            aquacomputer_section=(_XT6_SECTION, dict(_QUADRO_ENTRY, temp_map={"air_z0": "temp1"})),
            channels=("radiator", "exhaust"),
        )


def test_extra_bound_temp_name_is_config_error() -> None:
    with pytest.raises(ConfigError, match="not in mpc.temps"):
        _build(xt6_section=_XT6_SECTION, temps=())  # air_z0 bound but not declared


def test_malformed_device_spec_is_config_error() -> None:
    with pytest.raises(ConfigError, match="aquacomputer\\[0\\] must be a mapping"):
        _build(aquacomputer_section=("not-a-mapping",), channels=(), temps=())


def test_hwmon_style_entry_gets_the_rename_hint() -> None:
    entry = {"name": "quadro", "fans": {"exhaust": {"pwm": "pwm1"}}, "temp_map": {}}
    with pytest.raises(ConfigError, match="aquacomputer\\[0\\].name was renamed to 'device'"):
        _build(aquacomputer_section=(entry,), channels=("exhaust",), temps=())


# --- a Quadro on the aquaero's aquabus (PROJECT.md section 8 item 85) --------------------------


def test_aquabus_outputs_and_quadro_outputs_over_usb_are_refused_together() -> None:
    """A Quadro on aquabus ignores writes over its own USB: commanding aquaero pwm5..8 and
    Quadro outputs in one config leaves one of them without effect."""
    aquabus = {
        "device": "aquaero",
        "fans": {
            "radiator": {"pwm": "pwm1", "rpm": "fan1"},
            "rear": {"pwm": "pwm7", "rpm": "fan7"},
        },
        "temp_map": {"air_z0": "temp1", "air_z1": "bus2"},
    }
    with pytest.raises(
        ConfigError,
        match=r"aquacomputer\[0\] \(aquaero\) commands aquabus outputs pwm7 .*"
        r"aquacomputer\[1\] \(quadro\) commands the outputs of whichever Quadro is attached"
        r".*needs 'serial:' in its entry",
    ):
        _build(
            aquacomputer_section=(aquabus, dict(_QUADRO_ENTRY, temp_map={})),
            channels=("radiator", "rear", "exhaust"),
            temps=("air_z0", "air_z1"),
        )
    with pytest.raises(ConfigError, match="commands aquabus outputs pwm7"):
        _build(
            aquacomputer_section=(dict(_QUADRO_ENTRY, temp_map={}),),
            xt6_section=aquabus,
            channels=("radiator", "rear", "exhaust"),
            temps=("air_z0", "air_z1"),
        )
    # The Quadro entry may still read its own sensors over USB, commanding nothing.
    sensors_only = {"device": "quadro", "fans": {}, "temp_map": {"air_z2": "temp2"}}
    composite, _ = _build(
        aquacomputer_section=(aquabus, sensors_only),
        channels=("radiator", "rear"),
        temps=("air_z0", "air_z1", "air_z2"),
    )
    assert [d.kind for d in composite.devices] == [AQUAERO, QUADRO]
    # And the aquaero's own outputs next to a commanding Quadro stay allowed.
    _build(
        aquacomputer_section=(dict(_XT6_SECTION), _QUADRO_ENTRY),
        channels=("radiator", "exhaust"),
        temps=("air_z0", "air_z1"),
    )


def test_a_second_quadro_named_by_serial_next_to_aquabus_outputs_is_accepted_with_a_warning(
    caplog,
) -> None:
    """Review finding: Quadro A on the aquaero's aquabus (pwm5..8) and Quadro B on its own
    USB port is a valid topology. Which Quadro is on aquabus is unknown before opening, so
    an entry with a serial is taken as the second one, with a warning."""
    aquabus = {
        "device": "aquaero",
        "fans": {"rear": {"pwm": "pwm5", "rpm": "fan5"}},
        "temp_map": {"air_z0": "temp1"},
    }
    quadro_b = dict(_QUADRO_ENTRY, serial="00000-22222", temp_map={})
    with caplog.at_level("WARNING", logger="aqua_bridge.hw.sources"):
        composite, _ = _build(
            aquacomputer_section=(aquabus, quadro_b),
            channels=("rear", "exhaust"),
            temps=("air_z0",),
        )
    assert [d.binding.serial for d in composite.devices] == [None, "00000-22222"]
    (warning,) = [r.getMessage() for r in caplog.records if r.name == "aqua_bridge.hw.sources"]
    assert "aquacomputer[0] (aquaero) commands aquabus outputs pwm5" in warning
    assert "aquacomputer[1] (quadro 00000-22222) commands Quadro outputs over USB" in warning
    assert "not the one on the aquaero's aquabus" in warning


def _stuck_quadro_next_to(aquaero_device: FakeController, caplog) -> list[str]:
    clock = aquaero_device.clock
    quadro = FakeController(QUADRO, clock, node="/dev/hidraw5", serial="00000-11111")
    quadro.ignores = {0: 10000}  # a Quadro on aquabus ignores its own fan settings
    bus = FakeBus(aquaero_device, quadro)
    sleep = FakeSleep(clock)
    a = AquacomputerAdapter(
        DeviceBinding(kind=AQUAERO, pwm_map={"radiator": 1}), clock=clock, sleep=sleep, opener=bus
    )
    q = AquacomputerAdapter(
        DeviceBinding(kind=QUADRO, pwm_map={"exhaust": 1}), clock=clock, sleep=sleep, opener=bus
    )
    composite = CompositeSource([a, q], clock=clock)
    step = q.timing.duty_mismatch_s / 2
    with caplog.at_level("ERROR", logger="aqua_bridge.hw.aquacomputer"):
        for _ in range(12):
            clock.advance(step)
            aquaero_device.emit()
            quadro.emit()
            composite.read()
            composite.apply(MpcCommand(pwm={"radiator": 0.5, "exhaust": 0.5}, mode=Mode.AUTO))
    assert q.stuck_channels == ("exhaust",)
    return [r.getMessage() for r in caplog.records if r.levelname == "ERROR"]


def test_a_stuck_quadro_next_to_an_aquaero_with_an_aquabus_device_is_explained(caplog) -> None:
    clock = FakeClock()
    (error,) = _stuck_quadro_next_to(aquabus_aquaero(clock, node="/dev/hidraw2"), caplog)
    assert "pwm1 (exhaust) still reports 100.00 %" in error
    assert "reports a device on its aquabus (pwm5, pwm6, pwm7, pwm8)" in error
    assert "probably on the aquaero's aquabus" in error and "(pwm5..pwm8)" in error


def test_a_stuck_quadro_next_to_an_aquaero_without_an_aquabus_device_gets_no_hint(caplog) -> None:
    clock = FakeClock()
    (error,) = _stuck_quadro_next_to(FakeController(AQUAERO, clock, node="/dev/hidraw2"), caplog)
    assert "still reports" in error and "aquabus" not in error


# --- the loop over an aquaero whose aquabus slot is empty (PROJECT.md section 8 item 90) -------


class _Notifier:
    def __init__(self) -> None:
        self.ready_n = 0

    def ready(self) -> bool:
        self.ready_n += 1
        return True

    def watchdog(self) -> bool:
        return True

    def stopping(self) -> bool:
        return True


def _aquabus_loop(cfg, device: FakeController):
    """The example config's channels on the aquaero: radiator on its own output 1,
    intake on aquabus output 5 (with its tachometer)."""
    clock = device.clock
    adapter = AquacomputerAdapter(
        DeviceBinding(
            kind=AQUAERO,
            pwm_map={"radiator": 1, "intake": 5},
            fan_map={"intake": 5},
            temp_map={"coolant": "temp6", "air": "temp7"},
        ),
        clock=clock,
        sleep=FakeSleep(clock),
        opener=FakeBus(device),
    )
    composite = CompositeSource([adapter], clock=clock)
    notifier = _Notifier()
    loop = Loop(
        composite,
        composite,
        cfg,
        Supervisor(cfg, clock=lambda: 0.0),
        clock=clock,
        notifier=notifier,
    )

    physical = next(g for g in AQUAERO.temp_groups if g.prefix == "temp").offset

    def run(ticks: int) -> list[TickResult]:
        results = []
        for _ in range(ticks):
            clock.advance(cfg.dt)
            # A frozen temperature while the fans move would be the gate's stuck case.
            status = bytearray(device.status_template)
            for offset in (physical + 2 * 5, physical + 2 * 6):  # temp6, temp7
                value = 3000 + 5 * (loop.tick_count % 2)
                status[offset : offset + 2] = value.to_bytes(2, "big")
            device.status_template = bytes(status)
            device.emit()
            results.append(loop.tick())
        return results

    return adapter, loop, notifier, run


def test_an_empty_aquabus_slot_does_not_blind_the_rest_of_the_controller(fast_cfg, caplog) -> None:
    """Item 90: the Quadro drops off aquabus mid-run. The channel it served reports no
    rpm and no duty, but the aquaero's own thermistors and output keep working, the loop
    never runs its fallback for it, and the error is logged once, not once per tick."""
    cfg = fast_cfg
    device = aquabus_aquaero(FakeClock(), node="/dev/hidraw2")
    adapter, loop, notifier, run = _aquabus_loop(cfg, device)
    good = run(20)
    assert all(r.ok for r in good[-3:]) and notifier.ready_n == 1
    assert good[-1].obs.pwm["intake"] is not None and good[-1].obs.rpm["intake"] is not None

    with caplog.at_level("INFO", logger="aqua_bridge.hw.aquacomputer"):
        device.status_template = fixture_bytes("aquaero-status.bin")  # the Quadro left aquabus
        absent = run(20)
        assert all(r.ok and r.read_error is None for r in absent)
        assert all(r.cmd.mode is Mode.AUTO for r in absent)
        for result in absent:
            assert result.obs.rpm["intake"] is None and result.obs.pwm["intake"] is None
            assert result.obs.pwm["radiator"] is not None
            assert set(result.obs.temps) == {"coolant", "air"}
            assert all(value is not None for value in result.obs.temps.values())
        assert adapter.absent_channels == ("intake",)
        # Both channels are still commanded, the healthy one by the solver.
        assert control_duty(AQUAERO, device.ctrl, 0) == round(absent[-1].cmd.pwm["radiator"] * 1e4)
        assert control_duty(AQUAERO, device.ctrl, 4) == round(absent[-1].cmd.pwm["intake"] * 1e4)

        device.status_template = fixture_bytes("aquaero-status-aquabus-fan7-100.bin")  # back
        back = run(3)
    assert all(r.ok for r in back) and adapter.absent_channels == ()
    assert back[-1].obs.rpm["intake"] is not None
    errors = [r.getMessage() for r in caplog.records if r.levelname == "ERROR"]
    assert len(errors) == 1 and "no device behind pwm5 (intake), fan5 (intake)" in errors[0]
    assert [m for m in _messages(caplog, "INFO") if "again" in m] == [
        "aquaero: a device is behind pwm5 (intake), fan5 (intake) again"
    ]
    assert loop.shutdown() is True


def test_every_commanded_channel_absent_is_logged_but_still_not_a_failed_read(
    fast_cfg, caplog
) -> None:
    """The other half of item 90: with every commanded output on the missing device the
    controller says so once, and its temperatures still reach the solver -- they are what
    makes the remaining fans of the other controllers ramp."""
    clock = FakeClock()
    device = FakeController(AQUAERO, clock, node="/dev/hidraw2")  # nothing on aquabus
    adapter = AquacomputerAdapter(
        DeviceBinding(
            kind=AQUAERO,
            pwm_map={"intake": 5, "exhaust": 6},
            fan_map={"intake": 5},
            temp_map={"coolant": "temp6", "air": "temp7"},
        ),
        clock=clock,
        sleep=FakeSleep(clock),
        opener=FakeBus(device),
    )
    with caplog.at_level("ERROR", logger="aqua_bridge.hw.aquacomputer"):
        obs = CompositeSource([adapter], clock=clock).read()
    assert obs.pwm == {"intake": None, "exhaust": None} and obs.rpm == {"intake": None}
    # temp7 has nothing connected in this fixture; temp6 still reaches the solver.
    assert obs.temps == {"coolant": pytest.approx(22.26), "air": None}
    assert adapter.absent_channels == ("exhaust", "intake")
    (error,) = [r.getMessage() for r in caplog.records if r.levelname == "ERROR"]
    assert "commands no reachable output now" in error


def test_a_daemon_started_with_an_empty_aquabus_slot_becomes_ready(fast_cfg) -> None:
    device = FakeController(AQUAERO, FakeClock(), node="/dev/hidraw2")  # nothing on aquabus
    _adapter, _loop, notifier, run = _aquabus_loop(fast_cfg, device)
    (first,) = run(1)
    assert first.read_error is None and first.applied
    assert first.obs.pwm == {"radiator": pytest.approx(1.0), "intake": None}
    assert notifier.ready_n == 1


# --- device health and fan readings (items 79, 83) --------------------------------------


def test_read_merges_the_fan_readings_of_every_controller() -> None:
    """The per-output electrical readings ride obs.inputs["fans"], not temps/rpm/pwm,
    so nothing the solver sees changes (PROJECT.md section 8 item 79)."""
    a, q, _aquaero, _quadro, clock = _fleet(FakeClock(50.0))
    obs = CompositeSource([a, q], clock=clock).read()

    assert set(obs.inputs["fans"]) == {"radiator", "exhaust"}
    radiator = obs.inputs["fans"]["radiator"]
    assert radiator["device"] == "aquaero" and radiator["output"] == "pwm2"
    assert radiator["rpm"] == 120.0 and radiator["duty"] == pytest.approx(0.1412)
    assert radiator["voltage_v"] == pytest.approx(12.09)
    # the aquaero reports 0 mA / 0 W for its own outputs in PWM mode: not a fault
    assert radiator["current_ma"] == 0.0 and radiator["power_reported"] is False
    assert obs.inputs["fans"]["exhaust"]["power_reported"] is True
    # and none of it reached the observation the gate and the solver read
    assert set(obs.temps) == {"air_z0", "air_z1"} and set(obs.rpm) == {"radiator"}


def test_an_aquabus_output_reports_its_real_current_and_power() -> None:
    clock = FakeClock()
    device = aquabus_aquaero(clock)
    adapter = AquacomputerAdapter(
        DeviceBinding(kind=AQUAERO, pwm_map={"qd3": 7}, fan_map={"qd3": 7}),
        clock=clock,
        sleep=FakeSleep(clock),
        opener=FakeBus(device),
    )
    reading = adapter.read().inputs["fans"]["qd3"]
    assert reading["aquabus"] is True and reading["power_reported"] is True
    assert (reading["rpm"], reading["current_ma"]) == (1105.0, 27.0)
    assert reading["power_w"] == pytest.approx(0.32)


def test_an_absent_aquabus_slot_contributes_no_fan_reading() -> None:
    """A slot with rpm 0xFFFF reads 0 V and 0 W: the whole block is meaningless, so it
    is left out rather than published as a dead fan on a dead rail."""
    clock = FakeClock()
    adapter = AquacomputerAdapter(
        DeviceBinding(kind=AQUAERO, pwm_map={"qd3": 7, "radiator": 2}),
        clock=clock,
        sleep=FakeSleep(clock),
        opener=FakeBus(FakeController(AQUAERO, clock)),  # nothing on aquabus
    )
    with pytest.raises(DeviceUnavailable, match="no device behind pwm7"):
        adapter.read()
    assert set(adapter.fan_readings()) == {"radiator"}


def test_device_health_lists_the_absent_channels_after_a_failed_read() -> None:
    """The tick that most needs the diagnosis is the one whose read() raised, so the
    device health does not ride the observation (PROJECT.md section 8 item 83)."""
    clock = FakeClock()
    adapter = AquacomputerAdapter(
        DeviceBinding(kind=AQUAERO, pwm_map={"qd3": 7}, serial="12345-54321"),
        clock=clock,
        sleep=FakeSleep(clock),
        opener=FakeBus(FakeController(AQUAERO, clock)),
    )
    with pytest.raises(DeviceUnavailable):
        adapter.read()
    health = adapter.device_health()
    assert health["absent_channels"] == ["qd3"] and health["stuck_channels"] == []
    assert health["device"] == "aquaero" and health["open"] is True
    assert health["status_age_s"] == 0.0 and health["firmware"] == 2104
    assert health["problems"] == ["aquaero 12345-54321: no device on aquabus behind qd3"]


def test_device_health_reports_the_flow_sensors_and_no_active_profile_yet() -> None:
    """Flow is published here, never in the observation (owner decision 2026-09-16,
    PROJECT.md section 8.1, item 91); 0x7FFF reads as null."""
    a, _q, _aquaero, _quadro, _clock = _fleet()
    a.read()
    health = a.device_health()
    assert health["flows"] == {"flow1": 0, "flow2": 0, "flow3": None}
    assert "active_profile" not in health  # another change adds it


def test_device_health_reports_an_active_profile_once_one_exists() -> None:
    """Read defensively: this change does not depend on the one that publishes it."""
    a, _q, _aquaero, _quadro, _clock = _fleet()
    a.read()
    a.active_profile = 2
    assert a.device_health()["active_profile"] == 2


def test_device_health_lists_an_own_output_not_in_pwm_mode() -> None:
    """The aquaero's own outputs 1-4 have a mode word; an aquabus output's is not
    interpreted, so only the former can be listed (items 83, 85)."""
    clock = FakeClock()
    device = FakeController(AQUAERO, clock)
    mode = 0x20C + 20 * 1 + 0x0E  # controller block 2 (output 2)
    device.ctrl[mode : mode + 2] = (0x0001).to_bytes(2, "big")  # low byte 1 = DC voltage
    adapter = AquacomputerAdapter(
        DeviceBinding(kind=AQUAERO, pwm_map={"radiator": 2}),
        clock=clock,
        sleep=FakeSleep(clock),
        opener=FakeBus(device),
    )
    adapter.apply(MpcCommand(pwm={"radiator": 0.4}, mode=Mode.AUTO))
    health = adapter.device_health()
    assert health["not_pwm_channels"] == ["radiator"]
    assert health["problems"] == ["aquaero: radiator are not in PWM mode"]


def test_composite_device_health_merges_every_controller_and_its_problems() -> None:
    a, q, _aquaero, _quadro, clock = _fleet()
    composite = CompositeSource([a, q], clock=clock)
    composite.read()
    health = composite.device_health()
    assert [d["device"] for d in health["devices"]] == ["aquaero", "quadro"]
    assert health["ok"] is True and health["problems"] == []


def test_composite_device_health_survives_a_controller_that_raises() -> None:
    a, q, _aquaero, _quadro, clock = _fleet()

    def boom() -> dict[str, object]:
        raise RuntimeError("no")

    a.device_health = boom  # type: ignore[method-assign]
    health = CompositeSource([a, q], clock=clock).device_health()
    assert [d["device"] for d in health["devices"]] == ["quadro"]
