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


def test_hidraw_nodes_are_readable_and_writable_for_a_service_group(unit, rules):
    """The daemon opens /dev/hidrawN (root:root 0600 by default) read/write as a non-root
    user: a udev rule on the Aqua Computer hidraw nodes must give a group the unit puts the
    service user in both permissions."""
    groups = set()
    for value in unit.get("SupplementaryGroups", []):
        groups.update(value.split())
    (rule,) = [r for r in rules if 'SUBSYSTEM=="hidraw"' in r]
    assert 'ATTRS{idVendor}=="0c70"' in rule
    assert 'MODE="0660"' in rule
    group = re.search(r'GROUP="([^"]+)"', rule)
    assert group and group.group(1) in groups


def test_optional_driver_hwmon_rule_still_grants_a_service_group(unit, rules):
    """Kept for the optional aquacomputer_d5next driver (not installed by install-pi.sh):
    its pwm attributes go to a group the unit puts the service user in."""
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
    assert "udevadm trigger" in text and "subsystem-match=hidraw" in text


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


def test_das_dropin_clears_then_sets_execstart_with_source_composite(unit, das_dropin):
    """An empty ExecStart= clears the base unit's before the real one is set (systemd
    drop-in semantics); the replacement is the base unit's ExecStart= plus exactly
    ``--source composite``, nothing else changed (PROJECT.md section 10)."""
    cleared, replacement = das_dropin["ExecStart"]
    assert cleared == "", "a drop-in must clear ExecStart= before setting a new one"
    (base_exec,) = unit["ExecStart"]
    assert replacement == f"{base_exec} --source composite"


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


@pytest.mark.parametrize("script", ["install-pi.sh", "host-usb.sh", "install-aquacomputer-dkms.sh"])
def test_shell_scripts_parse(script):
    bash = shutil.which("bash")
    if bash is None:
        pytest.skip("bash not available")
    subprocess.run([bash, "-n", str(DEPLOY / script)], check=True)
    shellcheck = shutil.which("shellcheck")
    if shellcheck is None:
        pytest.skip("shellcheck not installed (bash -n passed)")
    subprocess.run([shellcheck, str(DEPLOY / script)], check=True)


# --- optional aquacomputer_d5next DKMS package (section 9) -----------------------------

DKMS_PKG = DEPLOY / "dkms" / "aquacomputer_d5next"
DKMS_SCRIPT = DEPLOY / "install-aquacomputer-dkms.sh"


def _packages() -> list[str]:
    return [
        line.strip()
        for line in (DEPLOY / "packages-rpi.txt").read_text().splitlines()
        if line.strip() and not line.strip().startswith("#")
    ]


def test_install_script_does_not_run_the_driver_build():
    """The daemon uses hidraw; the DKMS package stays in deploy/ for later, run by hand."""
    for line in (DEPLOY / "install-pi.sh").read_text().splitlines():
        stripped = line.strip()
        if not stripped or stripped.startswith("#"):
            continue
        assert "install-aquacomputer-dkms" not in stripped, line
        assert "dkms" not in stripped.split(), line


def test_packages_leave_out_the_optional_driver_build_tools():
    packages = _packages()
    for name in ("dkms", "curl", "patch"):
        assert name not in packages, name


def test_dkms_script_names_its_packages_and_fails_early_without_them():
    script = DKMS_SCRIPT.read_text()
    header, body = script.split("set -euo pipefail", 1)
    assert "OPTIONAL" in header and "install-pi.sh does not run this script" in header
    assert "apt-get install -y dkms patch curl linux-headers-rpi-v6" in header
    check = body.index("required=(dkms patch)")
    assert check < body.index("dkms status") and check < body.index("curl -fsSL")
    assert "required+=(curl)" in body and "error: missing" in body


def test_dkms_script_exits_with_a_clear_message_when_dkms_is_missing(tmp_path):
    bash = shutil.which("bash")
    if bash is None:
        pytest.skip("bash not available")
    if Path("/usr/sbin/dkms").exists():
        pytest.skip("dkms is installed in /usr/sbin on this machine")
    bin_dir = tmp_path / "bin"
    bin_dir.mkdir()
    for tool in ("uname", "dirname"):
        found = shutil.which(tool)
        if found is None:
            pytest.skip(f"{tool} not available")
        (bin_dir / tool).symlink_to(found)
    result = subprocess.run(
        [bash, str(DKMS_SCRIPT), "--source", str(tmp_path / "driver.c")],
        env={"PATH": str(bin_dir)},
        capture_output=True,
        text=True,
        check=False,
    )
    assert result.returncode == 1, result.stderr
    assert "error: missing dkms patch" in result.stderr
    assert "apt-get install" in result.stderr


def test_dkms_conf_is_a_template_the_script_fills_in():
    conf = _unit_values((DKMS_PKG / "dkms.conf").read_text())
    assert conf["PACKAGE_NAME"] == ['"aquacomputer_d5next"']
    assert conf["PACKAGE_VERSION"] == ['"@PKGVER@"']
    assert conf["BUILT_MODULE_NAME[0]"] == ['"aquacomputer_d5next"']
    assert conf["AUTOINSTALL"] == ['"yes"']
    makefile = (DKMS_PKG / "Makefile").read_text()
    assert re.search(r"^obj-m\s*:=\s*aquacomputer_d5next\.o$", makefile, re.MULTILINE)
    script = DKMS_SCRIPT.read_text()
    assert "s/@PKGVER@/$PKG_VER/" in script
    assert '"$PKG_DIR"/*.patch' in script


def test_dkms_script_never_unloads_a_loaded_module():
    # The daemon may be using the hwmon device; a reload is left to a reboot.
    for line in DKMS_SCRIPT.read_text().splitlines():
        stripped = line.strip()
        if stripped.startswith(("#", "echo", '"')):
            continue
        assert "rmmod" not in stripped and "modprobe -r" not in stripped, line


def _hunk_sides(patch_text: str) -> tuple[list[str], list[str]]:
    """Old and new side of every hunk, in order (context lines on both)."""
    old: list[str] = []
    new: list[str] = []
    in_hunk = False
    for line in patch_text.splitlines():
        if line.startswith("@@"):
            in_hunk = True
            continue
        if not in_hunk or line.startswith(("--- ", "+++ ")):
            continue
        tag, body = line[:1], line[1:]
        if tag == " ":
            old.append(body)
            new.append(body)
        elif tag == "-":
            old.append(body)
        elif tag == "+":
            new.append(body)
        elif line == "":
            old.append("")
            new.append("")
    return old, new


@pytest.mark.parametrize("patch_file", sorted(DKMS_PKG.glob("*.patch")), ids=lambda p: p.name)
def test_driver_patch_is_well_formed_and_applies_to_its_own_context(patch_file, tmp_path):
    text = patch_file.read_text()
    assert text.startswith("SPDX-License-Identifier: GPL-2.0")
    assert "\n--- a/aquacomputer_d5next.c\n+++ b/aquacomputer_d5next.c\n@@ " in text
    patch = shutil.which("patch")
    if patch is None:
        pytest.skip("patch not installed")
    old, new = _hunk_sides(text)
    target = tmp_path / "aquacomputer_d5next.c"
    target.write_text("\n".join(old) + "\n")
    subprocess.run([patch, "-s", "-p1", "-d", str(tmp_path)], input=text.encode(), check=True)
    assert target.read_text() == "\n".join(new) + "\n"


def test_at_least_one_driver_patch_exists():
    assert sorted(DKMS_PKG.glob("*.patch"))
