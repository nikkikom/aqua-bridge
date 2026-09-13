"""Static checks on the deploy files (PROJECT.md section 9): systemd unit, udev rules,
install scripts. No Pi needed; nothing here talks to systemd or udev."""

from __future__ import annotations

import re
import shutil
import subprocess
from pathlib import Path

import pytest

DEPLOY = Path(__file__).resolve().parent.parent / "deploy"
UNIT = DEPLOY / "aqua-bridge.service"
RULES = DEPLOY / "99-aquacomputer.rules"


def _unit_values(text: str) -> dict[str, list[str]]:
    out: dict[str, list[str]] = {}
    for line in text.splitlines():
        line = line.strip()
        if not line or line.startswith(("#", "[")):
            continue
        key, _, value = line.partition("=")
        out.setdefault(key.strip(), []).append(value.strip())
    return out


@pytest.fixture(scope="module")
def unit() -> dict[str, list[str]]:
    return _unit_values(UNIT.read_text())


@pytest.fixture(scope="module")
def rules() -> list[str]:
    return [
        line.strip()
        for line in RULES.read_text().splitlines()
        if line.strip() and not line.strip().startswith("#")
    ]


# --- systemd unit (section 9) ---------------------------------------------------------


def test_unit_notify_watchdog_restart_and_no_execstop(unit):
    assert unit["Type"] == ["notify"]
    assert unit["NotifyAccess"] == ["main"]
    assert unit["Restart"] == ["always"]
    assert "Wants" in unit and "network-online.target" in unit["Wants"][0]
    assert "After" in unit and "network-online.target" in unit["After"][0]
    assert float(unit["WatchdogSec"][0]) > 0
    assert "ExecStop" not in unit, "section 9: the SIGTERM handler is the only stop path"


def test_unit_execstart_is_the_venv_interpreter_running_the_module_directly(unit):
    (exec_start,) = unit["ExecStart"]
    assert exec_start.startswith("/opt/aqua-bridge/.venv/bin/python -m aqua_bridge")
    assert "sh -c" not in exec_start and "bash" not in exec_start


def test_unit_has_an_explicit_start_timeout(unit):
    """READY=1 is gated on the first successful apply; a device absent at boot must
    surface as a bounded start timeout, not systemd's implicit default."""
    (timeout,) = unit["TimeoutStartSec"]
    assert float(timeout) >= float(unit["WatchdogSec"][0])


def test_unit_gives_the_model_store_a_state_directory(unit):
    """Milestone model-store: systemd creates /var/lib/aqua-bridge owned by User= and
    exports it as $STATE_DIRECTORY, which the daemon reads for model.json; ExecStart
    passes no --model-store, so the unit and the default agree."""
    assert unit["StateDirectory"] == ["aqua-bridge"]
    (exec_start,) = unit["ExecStart"]
    assert "--model-store" not in exec_start
    source = (DEPLOY.parent / "src" / "aqua_bridge" / "modelstore.py").read_text()
    assert '"STATE_DIRECTORY"' in source and 'FILENAME = "model.json"' in source


# --- udev rules vs the service account's groups (review finding F2) ---------------------


def test_hwmon_pwm_attributes_are_made_writable_for_a_service_group(unit, rules):
    """The daemon writes /sys/class/hwmon/hwmonN/pwmK (root:root 0644 by default) as a
    non-root user. A udev rule on the aquaero's hwmon device must hand the pwm attributes
    to a group the unit actually puts the service user in."""
    groups = set()
    for value in unit.get("SupplementaryGroups", []):
        groups.update(value.split())
    assert groups, "unit runs as a non-root user without any supplementary group"
    hwmon_rules = [r for r in rules if 'SUBSYSTEM=="hwmon"' in r]
    assert hwmon_rules, "no udev rule for the hwmon device"
    (rule,) = hwmon_rules
    assert 'ATTRS{idVendor}=="0c70"' in rule
    assert "pwm" in rule and "g+w" in rule
    granted = {g for g in groups if re.search(rf"chgrp {g}\b", rule)}
    assert granted, f"hwmon rule grants none of the unit's groups {sorted(groups)}"


def test_usb_and_hidraw_rules_use_a_unit_group(unit, rules):
    groups = set()
    for value in unit.get("SupplementaryGroups", []):
        groups.update(value.split())
    for subsystem in ("usb", "hidraw"):
        matching = [r for r in rules if f'SUBSYSTEM=="{subsystem}"' in r]
        assert matching, f"no {subsystem} rule"
        for r in matching:
            group = re.search(r'GROUP="([^"]+)"', r)
            assert group and group.group(1) in groups


def test_install_script_triggers_udev_for_an_already_attached_device():
    text = (DEPLOY / "install-pi.sh").read_text()
    assert "udevadm control --reload-rules" in text
    assert "udevadm trigger" in text and "subsystem-match=hwmon" in text


# --- SMART agent example unit (milestone smart-agent) -----------------------------------


def test_smart_agent_unit_is_a_user_unit_example_not_installed_by_install_pi():
    unit_text = (DEPLOY / "aqua-bridge-smart-agent.service").read_text()
    values = _unit_values(unit_text)
    (exec_start,) = values["ExecStart"]
    assert "smart_agent.py" in exec_start
    assert "--mqtt" in exec_start and "--node-id" in exec_start
    assert "WantedBy" in values  # has an [Install] section like any enable-able unit
    # Not the Pi's system-level daemon unit: no User=/SupplementaryGroups= (those are
    # a systemd *system* unit concept; this is meant for `systemctl --user`).
    assert "User" not in values
    install_text = (DEPLOY / "install-pi.sh").read_text()
    assert "aqua-bridge-smart-agent" not in install_text


@pytest.mark.parametrize("script", ["install-pi.sh", "host-usb.sh"])
def test_shell_scripts_parse(script):
    bash = shutil.which("bash")
    if bash is None:
        pytest.skip("bash not available")
    subprocess.run([bash, "-n", str(DEPLOY / script)], check=True)
    shellcheck = shutil.which("shellcheck")
    if shellcheck is None:
        pytest.skip("shellcheck not installed (bash -n passed)")
    subprocess.run([shellcheck, str(DEPLOY / script)], check=True)
