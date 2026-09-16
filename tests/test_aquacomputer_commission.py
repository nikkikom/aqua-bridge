"""Tests for tools/aquacomputer_commission.py against fake controllers and a fake systemctl.

``tools/`` is not on ``pythonpath``, so this file adds it to ``sys.path`` itself
(the same way tests/test_aquacomputer_probe.py and tests/test_w1_commission.py do).
No hardware, no real ``systemctl``: PROJECT.md section 8 item 88.
"""

from __future__ import annotations

import io
import subprocess
import sys
from pathlib import Path

import pytest
import yaml

from aqua_bridge.hw.aquacomputer import (
    AQUAERO,
    QUADRO,
    finalize_control_report,
    patch_duties,
)
from aqua_bridge.hw.hidraw import FeatureReportError
from aqua_bridge.model import ConfigError
from aquacomputer_fakes import FakeBus, FakeClock, FakeSleep, aquabus_aquaero
from aquacomputer_fakes import FakeController as _FC

TOOLS_DIR = Path(__file__).resolve().parent.parent / "tools"
if str(TOOLS_DIR) not in sys.path:
    sys.path.insert(0, str(TOOLS_DIR))

import aquacomputer_commission as tool  # noqa: E402 -- must follow the sys.path tweak above


def _proc(stdout: str) -> subprocess.CompletedProcess:
    return subprocess.CompletedProcess(["systemctl"], 0, stdout=stdout, stderr="")


def _runner(state: str):
    return lambda *args, **kwargs: _proc(state)


def _raising_runner(exc: BaseException):
    def runner(*args, **kwargs):
        raise exc

    return runner


@pytest.fixture
def config_path(tmp_path: Path, example_config_path: Path) -> Path:
    data = yaml.safe_load(example_config_path.read_text())
    data.pop("xt6", None)
    data["aquacomputer"] = [
        {
            "device": "aquaero",
            "fans": {"radiator": {"pwm": "pwm1", "rpm": "fan1"}, "intake": {"pwm": "pwm2"}},
            "temp_map": {"coolant": "temp1", "air": "temp2"},
        },
        {
            "device": "quadro",
            "serial": "12345-54321",
            "fans": {"exhaust": {"pwm": "pwm1"}},
        },
    ]
    path = tmp_path / "config.yaml"
    path.write_text(yaml.safe_dump(data))
    return path


@pytest.fixture
def rig():
    clock = FakeClock()
    sleep = FakeSleep(clock)
    aquaero = _FC(AQUAERO, clock, node="/dev/hidraw2")
    quadro = _FC(QUADRO, clock, node="/dev/hidraw5", serial="12345-54321")
    bus = FakeBus(aquaero, quadro)
    return clock, sleep, aquaero, quadro, bus


def _run(config_path: Path, rig, *, in_text: str | None = None, **kwargs) -> tuple[int, str]:
    _clock, sleep, _aquaero, _quadro, bus = rig
    out = io.StringIO()
    in_ = io.StringIO(in_text) if in_text is not None else None
    kwargs.setdefault("device", "aquaero")
    kwargs.setdefault("save", False)
    kwargs.setdefault("runner", _runner("inactive\n"))
    kwargs.setdefault("sleep", sleep)
    code = tool.commission(config_path=str(config_path), opener=bus, out=out, in_=in_, **kwargs)
    return code, out.getvalue()


# --- systemd gate ---------------------------------------------------------------------------


def test_refuses_while_the_unit_might_be_running(config_path, rig) -> None:
    code, text = _run(config_path, rig, runner=_runner("active\n"))
    assert code == 3 and "refusing" in text and "aqua-bridge.service" in text
    _clock, _sleep, _aquaero, _quadro, bus = rig
    assert bus.opened == []  # refused before ever touching a device


@pytest.mark.parametrize(
    "state", ["activating\n", "reloading\n", "deactivating\n", "unknown\n", "maintenance\n"]
)
def test_refuses_for_every_state_that_is_not_certainly_stopped(config_path, rig, state) -> None:
    """Only ``inactive`` and ``failed`` mean the daemon holds nothing. ``deactivating``
    is a ``systemctl stop`` that returned while the daemon's own shutdown writes (the
    ``fallback_pwm`` write, then the ``release()`` SET) are still in flight, and an
    unrecognised answer is no answer at all: both fail closed."""
    code, _text = _run(config_path, rig, runner=_runner(state))
    assert code == 3
    _clock, _sleep, _aquaero, _quadro, bus = rig
    assert bus.opened == []


def test_allows_when_inactive_or_failed(config_path, rig) -> None:
    assert _run(config_path, rig, runner=_runner("inactive\n"))[0] == 0
    assert _run(config_path, rig, runner=_runner("failed\n"))[0] == 0


def test_an_undeterminable_state_refuses_too(config_path, rig) -> None:
    """A missing systemctl, a timeout or a blank answer: fail closed, not open."""
    code, _text = _run(config_path, rig, runner=_raising_runner(FileNotFoundError("systemctl")))
    assert code == 3
    code, _text = _run(
        config_path, rig, runner=_raising_runner(subprocess.TimeoutExpired("systemctl", 5.0))
    )
    assert code == 3
    code, _text = _run(config_path, rig, runner=_runner(""))
    assert code == 3


# --- device selection ------------------------------------------------------------------------


def test_config_error_when_the_device_is_not_configured(tmp_path, example_config_path) -> None:
    data = yaml.safe_load(example_config_path.read_text())  # xt6: aquaero only
    path = tmp_path / "config.yaml"
    path.write_text(yaml.safe_dump(data))
    out = io.StringIO()
    code = tool.commission(
        config_path=str(path), device="quadro", save=False, runner=_runner("inactive\n"), out=out
    )
    assert code == 2 and "no configured quadro entry" in out.getvalue()


def test_config_error_when_several_entries_of_one_kind_match(tmp_path, example_config_path) -> None:
    data = yaml.safe_load(example_config_path.read_text())
    data.pop("xt6", None)
    data["aquacomputer"] = [
        {"device": "aquaero", "fans": {"a": {"pwm": "pwm1"}}},
        {"device": "aquaero", "serial": "11111-11111", "fans": {"a": {"pwm": "pwm1"}}},
    ]
    path = tmp_path / "config.yaml"
    path.write_text(yaml.safe_dump(data))
    out = io.StringIO()
    code = tool.commission(
        config_path=str(path), device="aquaero", save=False, runner=_runner("inactive\n"), out=out
    )
    assert code == 2
    assert "2 configured aquaero entries match" in out.getvalue()
    assert "--serial" in out.getvalue()


def test_serial_selects_the_right_entry(config_path, rig) -> None:
    code, text = _run(config_path, rig, device="quadro", serial="12345-54321")
    assert code == 0 and "quadro" in text
    _clock, _sleep, _aquaero, quadro, _bus = rig
    assert quadro.gets() and not quadro.sets() and not quadro.saves()


def test_select_binding_rejects_a_config_error_directly(config_path) -> None:
    from aqua_bridge.config import load_config

    app = load_config(config_path)
    with pytest.raises(ConfigError, match="no configured"):
        tool.select_binding(app, device="aquaero", serial="99999-99999")


# --- the report and confirmation ---------------------------------------------------------------


def test_shows_the_report_and_writes_nothing_without_save(config_path, rig) -> None:
    code, text = _run(config_path, rig)
    assert code == 0
    assert "control report held on the controller now" in text
    assert "duty" in text and "active profile:" in text
    assert "(dry run: pass --save" in text
    _clock, _sleep, aquaero, _quadro, _bus = rig
    assert [op.what for op in aquaero.ops] == ["get"]
    assert not aquaero.sets() and not aquaero.saves()


def test_quadros_report_names_the_unverified_save(config_path, rig) -> None:
    _code, text = _run(config_path, rig, device="quadro", serial="12345-54321")
    assert "not verified to persist" in text


def test_aquaeros_report_does_not_carry_the_unverified_note(config_path, rig) -> None:
    _code, text = _run(config_path, rig)
    assert "not verified to persist" not in text


def test_save_without_confirmation_is_declined_and_writes_nothing(config_path, rig) -> None:
    code, text = _run(config_path, rig, save=True, in_text="no\n")
    assert code == 5 and "aborted" in text
    _clock, _sleep, aquaero, _quadro, _bus = rig
    assert aquaero.saves() == []


def test_save_with_no_typed_answer_at_all_is_declined(config_path, rig) -> None:
    code, text = _run(config_path, rig, save=True, in_text="")  # EOF: readline() -> ""
    assert code == 5 and "aborted" in text


def test_save_requires_both_the_flag_and_a_typed_yes(config_path, rig) -> None:
    code, text = _run(config_path, rig, save=True, in_text="yes\n")
    assert code == 0 and "saved." in text
    _clock, _sleep, aquaero, _quadro, bus = rig
    # the report, the re-read that confirms nothing moved, then the save -- one open
    assert [op.what for op in aquaero.ops] == ["get", "get", "save"]
    assert aquaero.saves()[0].data == AQUAERO.save_report
    assert bus.opened == ["/dev/hidraw2"] and aquaero.open_count == 1


def test_save_flag_alone_never_saves_without_a_confirmation_prompt(config_path, rig) -> None:
    """``--save`` without a stdin to answer never falls through to a save."""
    code, _text = _run(config_path, rig, save=True, in_text="")
    assert code == 5
    _clock, _sleep, aquaero, _quadro, _bus = rig
    assert aquaero.saves() == []


class _MutatingReader(io.StringIO):
    """Answers the confirmation prompt and changes the controller while doing it:
    the seconds the operator spends reading the report are exactly the window in
    which the aquaero's own alarm can select another profile."""

    def __init__(self, text: str, mutate) -> None:
        super().__init__(text)
        self._mutate = mutate

    def readline(self, *args, **kwargs) -> str:  # type: ignore[override]
        self._mutate()
        return super().readline(*args, **kwargs)


def _alarm_selects_profile_2(controller) -> None:
    """The documented aquaero fallback (PROJECT.md section 2): the heartbeat times
    out, the temperature alarm selects profile 2, and the switch reloads that
    profile's saved settings -- every output at 100 %."""
    buf = bytearray(controller.ctrl)
    assert AQUAERO.profile_offset is not None
    buf[AQUAERO.profile_offset] = 1  # 0 = profile 1
    patch_duties(AQUAERO, buf, {0: 10000, 1: 10000})
    finalize_control_report(AQUAERO, buf)
    controller.ctrl = buf


def test_a_controller_that_changes_during_the_prompt_is_never_saved(config_path, rig) -> None:
    """What is saved is what was shown: the report is read again right before the
    save report goes out, and a mismatch aborts the run."""
    _clock, sleep, aquaero, _quadro, bus = rig
    out = io.StringIO()
    reader = _MutatingReader("yes\n", lambda: _alarm_selects_profile_2(aquaero))
    code = tool.commission(
        config_path=str(config_path),
        device="aquaero",
        save=True,
        runner=_runner("inactive\n"),
        opener=bus,
        sleep=sleep,
        out=out,
        in_=reader,
    )
    text = out.getvalue()
    assert code == 6
    assert aquaero.saves() == []
    assert [op.what for op in aquaero.ops] == ["get", "get"]  # no save between the two reads
    assert "refusing" in text and "changed between the report above" in text
    assert "active profile: 1" in text and "active profile: 2" in text  # what was shown, and now


def test_a_failed_re_read_before_the_save_is_a_device_error(config_path, rig) -> None:
    _clock, sleep, aquaero, _quadro, bus = rig
    out = io.StringIO()
    reader = _MutatingReader(
        "yes\n", lambda: aquaero.failures.append(FeatureReportError("HIDIOCGFEATURE: EIO"))
    )
    code = tool.commission(
        config_path=str(config_path),
        device="aquaero",
        save=True,
        runner=_runner("inactive\n"),
        opener=bus,
        sleep=sleep,
        out=out,
        in_=reader,
    )
    assert code == 4 and "device error" in out.getvalue()
    assert aquaero.saves() == []


def test_dry_run_never_saves_even_with_a_confirming_stdin(config_path, rig) -> None:
    """The explicit ``--save`` flag is required; a stray 'yes' on stdin is not enough."""
    code, text = _run(config_path, rig, save=False, in_text="yes\n")
    assert code == 0 and "dry run" in text
    _clock, _sleep, aquaero, _quadro, _bus = rig
    assert aquaero.saves() == []


# --- hardware errors --------------------------------------------------------------------------


def test_device_error_when_nothing_answers(config_path, rig) -> None:
    _clock, sleep, _aquaero, _quadro, _bus = rig
    empty_bus = FakeBus()  # no controllers at all
    out = io.StringIO()
    code = tool.commission(
        config_path=str(config_path),
        device="aquaero",
        save=False,
        runner=_runner("inactive\n"),
        opener=empty_bus,
        sleep=sleep,
        out=out,
    )
    assert code == 4 and "device error" in out.getvalue()


def test_device_error_when_the_configured_serial_does_not_match(tmp_path, example_config_path):
    data = yaml.safe_load(example_config_path.read_text())
    data.pop("xt6", None)
    data["aquacomputer"] = [
        {"device": "aquaero", "serial": "99999-99999", "fans": {"a": {"pwm": "pwm1"}}}
    ]
    path = tmp_path / "config.yaml"
    path.write_text(yaml.safe_dump(data))
    clock = FakeClock()
    sleep = FakeSleep(clock)
    aquaero = _FC(AQUAERO, clock, node="/dev/hidraw2", serial="12345-54321")
    bus = FakeBus(aquaero)
    out = io.StringIO()
    code = tool.commission(
        config_path=str(path),
        device="aquaero",
        save=False,
        runner=_runner("inactive\n"),
        opener=bus,
        sleep=sleep,
        out=out,
    )
    assert code == 4 and "device error" in out.getvalue()


def test_aquabus_output_is_shown_as_not_commanded_by_this_entry(tmp_path, example_config_path):
    data = yaml.safe_load(example_config_path.read_text())
    data.pop("xt6", None)
    data["aquacomputer"] = [
        {"device": "aquaero", "fans": {"radiator": {"pwm": "pwm1", "rpm": "fan1"}}}
    ]
    path = tmp_path / "config.yaml"
    path.write_text(yaml.safe_dump(data))
    clock = FakeClock()
    sleep = FakeSleep(clock)
    controller = aquabus_aquaero(clock, node="/dev/hidraw2")
    bus = FakeBus(controller)
    out = io.StringIO()
    code = tool.commission(
        config_path=str(path),
        device="aquaero",
        save=False,
        runner=_runner("inactive\n"),
        opener=bus,
        sleep=sleep,
        out=out,
    )
    text = out.getvalue()
    assert code == 0
    assert "pwm1 (radiator)" in text
    assert "pwm7 (not commanded by this config entry)" in text


# --- argument parsing -------------------------------------------------------------------------


def test_build_parser_requires_config_and_device(config_path) -> None:
    with pytest.raises(SystemExit):
        tool.build_parser().parse_args([])
    with pytest.raises(SystemExit):
        tool.build_parser().parse_args(["--config", str(config_path)])
    with pytest.raises(SystemExit):
        tool.build_parser().parse_args(["--config", str(config_path), "--device", "octo"])
    args = tool.build_parser().parse_args(
        ["--config", str(config_path), "--device", "quadro", "--save", "--unit", "foo.service"]
    )
    assert args.device == "quadro" and args.save is True and args.unit == "foo.service"


def test_main_wires_commission_together(config_path, rig) -> None:
    _clock, _sleep, _aquaero, _quadro, bus = rig
    # no --runner on the CLI: real systemctl, almost certainly absent or inactive here, so this
    # only exercises that main() reaches commission() and returns its exit code, 0 or 3.
    code = tool.main(["--config", str(config_path), "--device", "aquaero"], opener=bus)
    assert code in (0, 3)
