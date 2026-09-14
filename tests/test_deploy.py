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
DAS_DROPIN = DEPLOY / "aqua-bridge-das.conf"


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
def das_dropin() -> dict[str, list[str]]:
    return _unit_values(DAS_DROPIN.read_text())


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


def test_install_script_creates_a_self_signed_certificate_but_never_overwrites_or_adds_users():
    from aqua_bridge.publishers.httpauth import HttpSettings

    text = (DEPLOY / "install-pi.sh").read_text()
    defaults = HttpSettings()
    tls_dir = Path(defaults.tls_cert).parent
    assert Path(defaults.tls_key).parent == tls_dir
    config_dir = re.search(r'^CONFIG_DIR="([^"]+)"$', text, re.MULTILINE)
    assert config_dir is not None
    assert 'TLS_DIR="$CONFIG_DIR/tls"' in text
    assert tls_dir == Path(config_dir.group(1)) / "tls"
    assert Path(defaults.credentials_file).parent == Path(config_dir.group(1))
    assert f'TLS_CERT="$TLS_DIR/{Path(defaults.tls_cert).name}"' in text
    assert f'TLS_KEY="$TLS_DIR/{Path(defaults.tls_key).name}"' in text
    assert "openssl req -x509" in text
    # Guarded: nothing is generated when either file exists.
    assert 'if sudo test -e "$TLS_CERT" || sudo test -e "$TLS_KEY"; then' in text
    assert 'sudo chown root:"$USER_ACCOUNT" "$TLS_KEY"' in text
    assert 'sudo chmod 640 "$TLS_KEY"' in text
    # Users are never created by the script; it prints the tool's command instead.
    assert "tools/http_user.py" in text
    assert "--stdin" not in text and "http-users" not in text
    assert "openssl" in (DEPLOY / "packages-rpi.txt").read_text().split()


# --- DAS install path (section 8 item 7) -------------------------------------------------


def test_das_dropin_only_overrides_execstart(das_dropin):
    """The drop-in must change nothing but ExecStart=: everything else (Type=,
    NotifyAccess=, StateDirectory=, Restart=, WatchdogSec=, ...) is inherited from the
    base unit unchanged."""
    assert set(das_dropin) == {"ExecStart"}
    text = DAS_DROPIN.read_text()
    assert "[Service]" in text
    assert "[Unit]" not in text and "[Install]" not in text


def test_das_dropin_clears_then_sets_execstart_with_source_hwmon(unit, das_dropin):
    """An empty ExecStart= clears the base unit's before the real one is set (systemd
    drop-in semantics); the replacement is the base unit's ExecStart= plus exactly
    ``--source hwmon``, nothing else changed (PROJECT.md section 10)."""
    cleared, replacement = das_dropin["ExecStart"]
    assert cleared == "", "a drop-in must clear ExecStart= before setting a new one"
    (base_exec,) = unit["ExecStart"]
    assert replacement == f"{base_exec} --source hwmon"


def test_install_script_das_flag_installs_das_config_and_dropin_never_enabling():
    text = (DEPLOY / "install-pi.sh").read_text()
    assert "--das" in text
    assert "DAS_DROPIN_SRC=" in text and "aqua-bridge-das.conf" in text
    assert 'DAS_DROPIN_DST="$DROPIN_DIR/das.conf"' in text
    assert "config.example-das.yaml" in text
    # The config step still only installs when config.yaml is absent, --das or not.
    assert text.count('if [[ ! -f "$CONFIG_DIR/config.yaml" ]]; then') == 1
    assert 'sudo install -m 644 "$DAS_DROPIN_SRC" "$DAS_DROPIN_DST"' in text
    # The drop-in install is gated on --das and precedes daemon-reload/verify.
    unit_section = text[text.index("== systemd unit ==") :]
    gate_pos = unit_section.index('if [[ "$DAS_MODE" -eq 1 ]]; then')
    dropin_pos = unit_section.index("DAS_DROPIN_DST")
    verify_pos = unit_section.index("systemd-analyze verify")
    assert gate_pos < dropin_pos < verify_pos
    # Never enables or starts the service, --das or not: the only "systemctl enable"
    # text in the whole script is the printed instructions after provisioning.
    before_summary = text.split("cat <<EOF")[0]
    assert "systemctl enable" not in before_summary
    assert "systemctl start" not in before_summary


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
