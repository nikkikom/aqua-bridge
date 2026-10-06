"""Static checks on the deploy files (PROJECT.md section 9): systemd unit, udev rules,
install scripts. No Pi needed; nothing here talks to systemd or udev."""

from __future__ import annotations

import re
import shutil
import subprocess
import time
from pathlib import Path

import pytest

DEPLOY = Path(__file__).resolve().parent.parent / "deploy"
UNIT = DEPLOY / "aqua-bridge.service"
RULES = DEPLOY / "99-aquacomputer.rules"
W1_RULES = DEPLOY / "99-w1-therm.rules"
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


def _rule_lines(path: Path) -> list[str]:
    return [
        line.strip()
        for line in path.read_text().splitlines()
        if line.strip() and not line.strip().startswith("#")
    ]


@pytest.fixture(scope="module")
def rules() -> list[str]:
    return _rule_lines(RULES)


@pytest.fixture(scope="module")
def w1_rules() -> list[str]:
    return _rule_lines(W1_RULES)


# --- systemd unit (section 9) ---------------------------------------------------------


def test_unit_notify_watchdog_restart_and_no_execstop(unit):
    assert unit["Type"] == ["notify"]
    assert unit["NotifyAccess"] == ["main"]
    assert unit["Restart"] == ["always"]
    assert float(unit["WatchdogSec"][0]) > 0
    assert "ExecStop" not in unit, "section 9: the SIGTERM handler is the only stop path"


def test_the_unit_never_waits_for_the_network(unit):
    """Section 2, "the network is outside the cooling path": the daemon drives the fans
    over USB and needs no network, so nothing in this unit may order it behind one.
    `Wants=`/`After=network-online.target` used to be here; with the router off,
    NetworkManager-wait-online spends its full 30 s default and the daemon's first
    write to the controllers is that much later, inside the aquaero's own 30 s
    software-sensor window."""
    for key, values in unit.items():  # every directive, not only the ordering ones
        for value in values:
            assert "network" not in value, f"{key}={value} orders cooling behind the network"


def test_the_unit_keeps_restarting_instead_of_being_parked(unit):
    """Section 2: a failure never reduces cooling. systemd's default start rate limit
    (5 starts in 10 s) parks the unit in "failed", which is the one state in which
    nothing writes the controllers and nothing writes the aquaero's software sensor
    again. RestartSec keeps "never park" from spinning the board."""
    assert unit["StartLimitIntervalSec"] == ["0"]
    assert 0 < float(unit["RestartSec"][0]) < 30.0  # well inside the aquaero's timeout


def test_unit_execstart_is_the_venv_interpreter_running_the_module_directly(unit):
    (exec_start,) = unit["ExecStart"]
    assert exec_start.startswith("/opt/aqua-bridge/.venv/bin/python -m aqua_bridge")
    assert "sh -c" not in exec_start and "bash" not in exec_start


def test_unit_has_an_explicit_start_timeout(unit):
    """READY=1 is gated on the first successful apply; a device absent at boot must
    surface as a bounded start timeout, not systemd's implicit default."""
    (timeout,) = unit["TimeoutStartSec"]
    assert float(timeout) >= float(unit["WatchdogSec"][0])


def test_the_unit_watchdog_leaves_the_margin_the_daemons_own_bound_does_not_count(unit):
    """Section 2, watchdog layering. `check_watchdog` bounds mpc.dt + the step bound +
    the controllers' worst-case I/O and says in so many words that the publishers and
    the recorder are not counted -- so the rest of WatchdogSec is their margin, and it
    has to be more than the largest single thing they can do on the loop thread. That
    is one `vcgencmd get_throttled` at host_health.vcgencmd_timeout_s (the MQTT
    publisher refreshes the host metrics from `on_tick`). Checked for the DAS example
    as it ships and for the two-controller alternative it describes.

    The timings come from the example's own entries (`from_section`, exactly what
    config.py builds), not from `for_kind(name)` alone: the example spells its timing
    keys out, and defaults that happen to equal them today would hide the day they
    stop -- which is the one thing this test exists to notice."""
    from aqua_bridge.config import load_config
    from aqua_bridge.health import HostHealthConfig
    from aqua_bridge.hw.aquacomputer_adapter import KINDS, AquacomputerTiming

    app = load_config(DEPLOY.parent / "config.example-das.yaml")
    watchdog = float(unit["WatchdogSec"][0])
    publisher_worst = HostHealthConfig.from_section(app.section("host_health")).vcgencmd_timeout_s
    shipped = [
        AquacomputerTiming.from_section(entry, f"aquacomputer[{i}]", KINDS[entry["device"]])
        for i, entry in enumerate(app.aquacomputer)
    ]
    assert [entry["device"] for entry in app.aquacomputer] == ["aquaero"]  # Quadro on aquabus
    # The alternative that section describes: the Quadro on its own USB port, at its
    # defaults, next to the aquaero the example declares.
    alternative = [*shipped, AquacomputerTiming.for_kind("quadro")]
    for timings in (shipped, alternative):
        controllers = sum(timing.worst_case_tick_s() for timing in timings)
        bound = app.mpc.dt + app.mpc.budget_alarm_ms / 1000.0 + controllers
        assert watchdog - bound >= publisher_worst, timings


def test_unit_gives_the_model_store_a_state_directory(unit):
    """Milestone model-store: systemd creates /var/lib/aqua-bridge owned by User= and
    exports it as $STATE_DIRECTORY, which the daemon reads for model.json; ExecStart
    passes no --model-store, so the unit and the default agree."""
    assert unit["StateDirectory"] == ["aqua-bridge"]
    (exec_start,) = unit["ExecStart"]
    assert "--model-store" not in exec_start
    source = (DEPLOY.parent / "src" / "aqua_bridge" / "modelstore.py").read_text()
    assert '"STATE_DIRECTORY"' in source and 'FILENAME = "model.json"' in source


def test_unit_gives_the_onewire_reader_lock_a_runtime_directory(unit):
    """PROJECT.md item 39: tools/w1_commission.py --check refuses to measure
    while the daemon is reading the same buses by trying to take the same
    cross-process lock (hw/onewire.py's ReaderLock) the daemon holds while its
    reader threads run, at onewire.lock_path's default of
    /run/aqua-bridge/onewire.lock. RuntimeDirectory=aqua-bridge is what makes
    that path writable by User= without running the daemon as root."""
    assert unit["RuntimeDirectory"] == ["aqua-bridge"]


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


# --- the DS18B20 buses (PROJECT.md §9 "udev", §8 item 38) -------------------------------


def test_w1_rules_grant_a_unit_group_the_attributes_the_daemon_writes(unit, w1_rules):
    """hw/onewire.py writes each sensor's resolution and probes the master's
    therm_bulk_read; both are created root-owned, and the unit runs as a
    non-root user. Without the grant every sensor stays at 12 bit, where a
    cycle no longer fits dt."""
    groups = set()
    for value in unit.get("SupplementaryGroups", []):
        groups.update(value.split())
    assert groups, "unit runs as a non-root user without any supplementary group"
    assert w1_rules, "no udev rule for the w1 subsystem"
    for rule in w1_rules:
        assert 'SUBSYSTEM=="w1"' in rule
        assert "g+w" in rule
        granted = {g for g in groups if re.search(rf"chgrp {g}\b", rule)}
        assert granted, f"w1 rule grants none of the unit's groups {sorted(groups)}: {rule}"
    joined = " ".join(w1_rules)
    assert "therm_bulk_read" in joined
    assert "resolution" in joined


def test_w1_rules_cover_both_the_master_and_the_slave_attributes(w1_rules):
    """therm_bulk_read appears on a master only once its first w1_therm slave has
    attached, so the slave rule has to fix the parent's attribute too -- the
    master rule alone can fire before the file exists."""
    master = [r for r in w1_rules if 'KERNEL=="w1_bus_master*"' in r]
    slave = [r for r in w1_rules if 'KERNEL=="28-*"' in r]
    assert master and slave
    assert "therm_bulk_read" in slave[0], "the slave rule must reach the parent's attribute"
    for rule in w1_rules:
        # The kernel re-announces devices on every bus search; a rule that only
        # matched "add" would miss them and miss install-pi.sh's change trigger.
        assert 'ACTION=="add|change"' in rule


def test_install_script_installs_and_triggers_the_w1_rules():
    text = (DEPLOY / "install-pi.sh").read_text()
    assert "99-w1-therm.rules" in text
    assert "--subsystem-match=w1" in text


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


def test_install_script_does_not_duplicate_the_board_scripts_journal_cap():
    """The disk-free rule's default is argued partly from a journal cap (health.py's
    HostHealthConfig.disk_free_min_gb docstring), and the cap it means is
    install-board-watchdogs.sh's JOURNAL_MAX_USE
    (test_board_script_takes_every_value_from_a_variable_with_a_default), not a
    second one here: two drop-ins governing one journal from two different
    defaults is the failure mode, not a feature, and this script does not touch
    the controllers or config.yaml either."""
    text = (DEPLOY / "install-pi.sh").read_text()
    assert "JOURNAL_MAX_USE" not in text
    assert "journald.conf.d" not in text
    assert "systemd-journald" not in text


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


@pytest.mark.parametrize(
    "script",
    [
        "install-pi.sh",
        "host-usb.sh",
        "install-aquacomputer-dkms.sh",
        "install-board-watchdogs.sh",
        "aqua-net-recover.sh",
        "aqua-net-watch.sh",
        "aqua-radio-recover.sh",
    ],
)
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


# --- board watchdogs and the network-recovery timer (section 2, "Watchdog layering") ---

BOARD_SCRIPT = DEPLOY / "install-board-watchdogs.sh"
NET_SCRIPT = DEPLOY / "aqua-net-recover.sh"
NET_UNIT = DEPLOY / "aqua-net-recover.service"
NET_TIMER = DEPLOY / "aqua-net-recover.timer"


def _shell_default(script: Path, name: str) -> str:
    """The default of a ``NAME="${NAME:-v}"`` / ``: "${NAME:=v}"`` knob."""
    text = script.read_text()
    for pattern in (
        rf'^{name}="\$\{{{name}:-([^}}]*)\}}"$',
        rf'^: "\$\{{{name}:=([^}}]*)\}}"$',
    ):
        match = re.search(pattern, text, re.MULTILINE)
        if match is not None:
            return match.group(1)
    raise AssertionError(f"{name} is not a documented variable at the top of {script.name}")


def test_board_script_takes_every_value_from_a_variable_with_a_default():
    """Every threshold the board script writes comes from a knob at the top, so the
    numbers in the drop-ins are the ones its header argues for, in one place."""
    knobs = {
        "SOC_WATCHDOG_SEC": "60",
        "REBOOT_WATCHDOG_SEC": "120",
        "JOURNAL_STORAGE": "persistent",
        "JOURNAL_MAX_USE": "1G",
        "JOURNAL_MAX_FILE_SIZE": "16M",
        "JOURNAL_MAX_RETENTION": "30day",
        "JOURNAL_SYNC_INTERVAL": "5m",
        "WIFI_POWERSAVE": "off",
        "NET_IFACE": "wlan0",
        "NET_CONNECTION": "",
        "NET_AUTOCONNECT_RETRIES": "0",
        "NET_RECOVER_INTERVAL": "5min",
        "NET_WATCH_INTERVAL": "15",
        "NET_WATCH_OUTAGE_INTERVAL": "30",
    }
    for name, default in knobs.items():
        assert _shell_default(BOARD_SCRIPT, name) == default
    text = BOARD_SCRIPT.read_text()
    for directive, name in (
        ("RuntimeWatchdogSec", "SOC_WATCHDOG_SEC"),
        ("RebootWatchdogSec", "REBOOT_WATCHDOG_SEC"),
        ("Storage", "JOURNAL_STORAGE"),
        ("SystemMaxUse", "JOURNAL_MAX_USE"),
        ("SystemMaxFileSize", "JOURNAL_MAX_FILE_SIZE"),
        ("MaxRetentionSec", "JOURNAL_MAX_RETENTION"),
        ("SyncIntervalSec", "JOURNAL_SYNC_INTERVAL"),
    ):
        assert f"{directive}=${{{name}}}" in text, directive


def test_journald_dropin_sorts_after_the_raspberry_pi_os_volatile_storage_dropin():
    """Raspberry Pi OS ships /usr/lib/systemd/journald.conf.d/40-rpi-volatile-
    storage.conf (Storage=volatile); systemd merges journald.conf.d fragments in
    lexical order of filename *across* /usr/lib, /run and /etc, later wins. The
    old name here, 20-aqua-journal-limits.conf, sorted before the vendor's and
    lost -- the defect measured on the owner's board on 2026-09-25. 99- is chosen
    to sort after any conventionally-numbered vendor drop-in: a three-digit
    prefix such as 100- still sorts *before* 99- as a string, since '1' < '9'."""
    text = BOARD_SCRIPT.read_text()
    match = re.search(r'^JOURNALD_DROPIN="([^"]+)"$', text, re.MULTILINE)
    assert match is not None
    dropin_name = Path(match.group(1)).name
    vendor_name = "40-rpi-volatile-storage.conf"
    assert dropin_name > vendor_name
    assert dropin_name == "99-aqua-journal-limits.conf"
    assert dropin_name > "100-a-later-vendor-file.conf"  # the string-sort quirk above


def test_journald_dropin_sets_storage_explicitly_from_a_validated_knob():
    """§9's finding: the caps were meaningless because the drop-in never set
    Storage= at all, leaving it to systemd's Storage=auto (persistent only when
    /var/log/journal already exists, which nothing on a stock image creates)."""
    text = BOARD_SCRIPT.read_text()
    assert "Storage=${JOURNAL_STORAGE}" in text
    assert "persistent | volatile | auto" in text
    assert "JOURNAL_STORAGE must be 'persistent', 'volatile' or 'auto'" in text


def test_board_script_cleans_up_the_hand_made_journald_dropins():
    """A board that already carries the by-hand workaround (10-persistent.conf,
    99-aqua-persistent.conf) must not end up with four drop-ins saying the same
    thing once this script installs its own equivalent."""
    text = BOARD_SCRIPT.read_text()
    assert '"/etc/systemd/journald.conf.d/10-persistent.conf"' in text
    assert '"/etc/systemd/journald.conf.d/99-aqua-persistent.conf"' in text
    assert 'for legacy in "${LEGACY_JOURNALD_DROPINS[@]}"; do' in text
    assert 'remove_path "$legacy"' in text


def test_board_script_cleans_up_its_own_old_journald_dropin_name():
    """A board that already carries this script's previous, losing name
    (20-aqua-journal-limits.conf) must not end up with two journald drop-ins
    saying the same thing once a re-run installs the new one -- the same gap
    PR #69 closed for the watchdog drop-in's own previous name."""
    text = BOARD_SCRIPT.read_text()
    assert '"/etc/systemd/journald.conf.d/20-aqua-journal-limits.conf"' in text
    assert 'for legacy in "${LEGACY_JOURNALD_DROPINS[@]}"; do' in text
    assert 'remove_path "$legacy"' in text


def test_board_script_verifies_journald_storage_instead_of_trusting_the_write():
    """Writing the drop-in and declaring victory is what produced the defect:
    both --check and a real run must report the effective Storage= and whether
    /var/log/journal is populated, and say plainly when that does not match
    JOURNAL_STORAGE. The effective-Storage= check delegates to systemd's own
    systemd-analyze cat-config rather than a second copy of the merge-order rule
    kept here to drift out of sync with the real one."""
    text = BOARD_SCRIPT.read_text()
    assert "systemd-analyze cat-config systemd/journald.conf" in text
    assert "effective Storage=:" in text
    assert "/var/log/journal populated:" in text
    assert 'if [[ "$effective" == "$JOURNAL_STORAGE" ]]; then' in text
    assert "WARNING: asked for JOURNAL_STORAGE=" in text
    # Called once for --check (before it reports and exits) and once after a
    # real apply (after journald has actually been restarted) -- bare call
    # lines, not the definition (journald_storage_report() {) or the comment
    # naming it.
    calls = [m.start() for m in re.finditer(r"^\s*journald_storage_report\s*$", text, re.MULTILINE)]
    assert len(calls) == 2
    check_pos, apply_pos = calls
    def_pos = text.index("journald_storage_report() {")
    check_exit = text.index("== check: changes pending, nothing was written ==")
    assert def_pos < check_pos < check_exit
    restart_line = text.index("as_root systemctl restart systemd-journald")
    done_marker = text.index("== done ==")
    assert restart_line < apply_pos < done_marker


def test_watchdog_dropin_sorts_after_the_raspberry_pi_os_enable_watchdog_dropin():
    """Raspberry Pi OS ships /usr/lib/systemd/system.conf.d/40-rpi-enable-watchdog.conf
    (RuntimeWatchdogSec=1m, RebootWatchdogSec=2m); systemd merges system.conf.d
    fragments in lexical order of filename *across* /usr/lib, /run and /etc, later
    wins -- the same rule as journald.conf.d. The old name here,
    10-aqua-watchdog.conf, sorted *before* the vendor's and lost: the vendor's
    values applied last and were what the board actually ran, invisible only
    because 60 s/120 s happen to equal the vendor's 1 m/2 m. 99- is chosen for the
    same reason as the journald drop-in: a three-digit prefix such as 100- still
    sorts *before* 99- as a string, since '1' < '9'."""
    text = BOARD_SCRIPT.read_text()
    match = re.search(r'^SYSTEM_DROPIN="([^"]+)"$', text, re.MULTILINE)
    assert match is not None
    dropin_name = Path(match.group(1)).name
    vendor_name = "40-rpi-enable-watchdog.conf"
    assert dropin_name > vendor_name
    assert dropin_name == "99-aqua-watchdog.conf"
    assert dropin_name > "100-a-later-vendor-file.conf"  # the string-sort quirk above


def test_board_script_cleans_up_its_own_old_watchdog_dropin_name():
    """A board that already carries this script's previous, losing name
    (10-aqua-watchdog.conf) must not end up with two watchdog drop-ins once a
    re-run installs the new one."""
    text = BOARD_SCRIPT.read_text()
    assert '"/etc/systemd/system.conf.d/10-aqua-watchdog.conf"' in text
    assert 'for legacy in "${LEGACY_SYSTEM_DROPINS[@]}"; do' in text
    assert 'remove_path "$legacy"' in text


def test_board_script_verifies_the_watchdog_instead_of_trusting_the_write():
    """Writing the drop-in and declaring victory is what produced the defect: both
    --check and a real run must report the effective RuntimeWatchdogUSec and
    RebootWatchdogUSec (from systemctl show) and say plainly when they do not
    match SOC_WATCHDOG_SEC/REBOOT_WATCHDOG_SEC. Both sides are compared as
    microseconds (systemd-analyze timespan), not as literal strings, since
    systemd normalizes a value like "60s" to "1min" on its own."""
    text = BOARD_SCRIPT.read_text()
    assert "systemctl show -p RuntimeWatchdogUSec -p RebootWatchdogUSec" in text
    assert "RuntimeWatchdogUSec: " in text
    assert "RebootWatchdogUSec:  " in text
    assert "LC_ALL=C systemd-analyze timespan" in text
    assert "OK: RuntimeWatchdogSec matches SOC_WATCHDOG_SEC" in text
    assert "OK: RebootWatchdogSec matches REBOOT_WATCHDOG_SEC" in text
    assert "WARNING: asked for SOC_WATCHDOG_SEC=" in text
    assert "WARNING: asked for REBOOT_WATCHDOG_SEC=" in text
    # Called once for --check (before it reports and exits) and once after a real
    # apply (after the daemon-reexec that makes RuntimeWatchdogSec take effect) --
    # bare call lines, not the definition (watchdog_report() {) or the comment
    # naming it.
    calls = [m.start() for m in re.finditer(r"^\s*watchdog_report\s*$", text, re.MULTILINE)]
    assert len(calls) == 2
    check_pos, apply_pos = calls
    def_pos = text.index("watchdog_report() {")
    check_exit = text.index("== check: changes pending, nothing was written ==")
    assert def_pos < check_pos < check_exit
    reexec_line = text.index("as_root systemctl daemon-reexec")
    done_marker = text.index("== done ==")
    assert reexec_line < apply_pos < done_marker


def test_board_script_sets_and_verifies_the_autoconnect_retries():
    """The setting whose silent default cost four days off the network. NetworkManager's
    `connection.autoconnect-retries` is 4 unless something says otherwise, and four
    consecutive association failures then block autoconnect for the profile until a
    manual activation, a NetworkManager restart or a reboot resets it. So the script
    writes it *and* reads the effective value back -- the same "verify, do not assume"
    the watchdog and journald settings got after each of them was written successfully
    and had no effect. The report is called once for --check (before anything is written,
    so it describes the board as it is) and once after the apply."""
    text = BOARD_SCRIPT.read_text()
    assert _shell_default(BOARD_SCRIPT, "NET_AUTOCONNECT_RETRIES") == "0"  # 0 = forever
    assert "nmcli connection modify" in text
    assert "connection.autoconnect-retries" in text
    assert "0 = retry forever" in text
    assert "would set:" in text  # --check reports it and writes nothing
    assert "connection.autoconnect-retries (" in text  # the effective value, per profile
    assert "OK: matches NET_AUTOCONNECT_RETRIES=" in text
    assert "WARNING: asked for NET_AUTOCONNECT_RETRIES=" in text
    calls = [
        m.start() for m in re.finditer(r"^\s*autoconnect_retries_report\s*$", text, re.MULTILINE)
    ]
    assert len(calls) == 2
    check_pos, apply_pos = calls
    def_pos = text.index("autoconnect_retries_report() {")
    check_exit = text.index("== check: changes pending, nothing was written ==")
    assert def_pos < check_pos < check_exit
    modify_pos = text.index("nmcli connection modify")
    done_marker = text.index("== done ==")
    assert modify_pos < apply_pos < done_marker


def test_board_script_refuses_a_nonsense_autoconnect_retries():
    """A knob that decides whether the board ever comes back on its own does not get to
    be a typo that NetworkManager rejects later, out of sight."""
    text = BOARD_SCRIPT.read_text()
    assert 'if [[ ! "$NET_AUTOCONNECT_RETRIES" =~ ^(-1|[0-9]+)$ ]]; then' in text
    assert "error: NET_AUTOCONNECT_RETRIES must be a whole number" in text


def test_the_soc_watchdog_sits_above_the_service_watchdog(unit):
    """The order is the point (section 2): a slow tick must get a daemon restart, which
    is cheap, before the board gets a reset, which costs a whole boot and spends the
    aquaero's own timeout on the way."""
    soc = float(_shell_default(BOARD_SCRIPT, "SOC_WATCHDOG_SEC"))
    assert soc > float(unit["WatchdogSec"][0])
    assert float(_shell_default(BOARD_SCRIPT, "REBOOT_WATCHDOG_SEC")) >= soc


def test_the_timer_ships_the_interval_the_board_script_installs():
    """install-board-watchdogs.sh rewrites OnBootSec=/OnUnitActiveSec= from
    NET_RECOVER_INTERVAL; the file in the repository has to already say that, so
    reading deploy/ tells the truth about the board."""
    interval = _shell_default(BOARD_SCRIPT, "NET_RECOVER_INTERVAL")
    values = _unit_values(NET_TIMER.read_text())
    assert values["OnBootSec"] == [interval]
    assert values["OnUnitActiveSec"] == [interval]


def _code_lines(path: Path) -> list[str]:
    return [
        line.strip()
        for line in path.read_text().splitlines()
        if line.strip() and not line.strip().startswith("#")
    ]


#: A word in command position: start of line, or after ;, &, |, or a shell keyword.
def _runs_command(lines: list[str], word: str) -> bool:
    pattern = re.compile(rf"(?:^|[;&|]\s*|\b(?:then|else|do)\s+){word}\b")
    return any(pattern.search(line) for line in lines)


def test_the_board_script_leaves_the_controllers_and_the_daemon_alone():
    """It installs watchdogs and journald limits. The service watchdog belongs to
    deploy/aqua-bridge.service, which this script only *reads* (to print the layering),
    and the controllers belong to the daemon."""
    lines = _code_lines(BOARD_SCRIPT)
    assert not _runs_command(lines, "reboot") and not _runs_command(lines, "shutdown")
    for line in lines:
        assert "aqua-heartbeat" not in line and "hidraw" not in line, line
        if "aqua-bridge.service" in line:
            assert line.startswith('UNIT_SRC="'), line
        if "systemctl" in line:
            assert "aqua-bridge" not in line, line


def test_network_recovery_never_escalates_beyond_re_associating():
    """The owner's constraint: switching off the router must not degrade cooling. So no
    reboot in any form, nothing touching aqua-bridge or the heartbeat, no controller."""
    lines = _code_lines(NET_SCRIPT)
    for word in ("reboot", "shutdown", "systemctl", "poweroff", "halt"):
        assert not _runs_command(lines, word), f"aqua-net-recover.sh runs {word}"
    for line in lines:
        assert "aqua-bridge" not in line and "aqua-heartbeat" not in line, line
        assert "hidraw" not in line, line
    for path in (NET_UNIT, NET_TIMER):
        for key, values in _unit_values(path.read_text()).items():
            if not key.startswith("Exec"):
                continue  # ordering is checked by test_the_recovery_unit_is_wired_...
            for value in values:
                for forbidden in ("reboot", "systemctl", "aqua-bridge.service", "hidraw"):
                    assert forbidden not in value, f"{path.name}: {key}={value}"
    script = NET_SCRIPT.read_text()
    assert "device disconnect" in script and "device connect" in script
    # The disconnected branch's one action, and the reason it is allowed: a manual
    # activation is also what resets NetworkManager's autoconnect retry counter.
    assert "connection up id" in script


def test_the_recovery_unit_is_wired_to_nothing_that_cools():
    values = _unit_values(NET_UNIT.read_text())
    assert values["Type"] == ["oneshot"]
    assert "Restart" not in values  # a failed check waits for the timer, never loops
    assert values["RuntimeDirectory"] == ["aqua-net-recover"]  # counters on tmpfs
    assert values["RuntimeDirectoryPreserve"] == ["yes"]
    for key in ("Wants", "Requires", "After", "Before", "Conflicts", "PartOf"):
        for value in values.get(key, []):
            assert "aqua-bridge" not in value and "aqua-heartbeat" not in value
            assert "network-online" not in value  # it exists for the offline case


def test_one_check_cannot_outlast_the_units_start_timeout():
    """Every wait in the script is a knob, and the unit's TimeoutStartSec is above what
    those knobs allow one run to cost. Without the explicit --wait, nmcli's own default
    for `device connect` alone (90 s) is already above systemd's default start timeout,
    and a run killed part-way through is a run that finished none of its bookkeeping."""
    script = NET_SCRIPT.read_text()
    # Every blocking nmcli carries the wait: the two of the up-but-dead branch and
    # the one activation of the disconnected branch. nmcli's own default for
    # `connection up` is the same 90 s as for `device connect`.
    assert script.count('nmcli -w "$AQUA_NET_NMCLI_WAIT_S" device') == 2
    assert script.count('nmcli -w "$AQUA_NET_NMCLI_WAIT_S" connection up') == 1
    assert script.count('nmcli -w "$AQUA_NET_NMCLI_WAIT_S"') == 3
    wait = float(_shell_default(NET_SCRIPT, "AQUA_NET_NMCLI_WAIT_S"))
    ping_deadline = float(_shell_default(NET_SCRIPT, "AQUA_NET_PING_DEADLINE_S"))
    timeout = float(_unit_values(NET_UNIT.read_text())["TimeoutStartSec"][0])
    assert timeout >= 2 * wait + ping_deadline
    minutes = re.fullmatch(r"(\d+)min", _shell_default(BOARD_SCRIPT, "NET_RECOVER_INTERVAL"))
    assert minutes is not None and timeout < 60 * int(minutes.group(1))  # no run overlaps itself


def test_no_net_recover_is_an_off_switch_and_not_a_skipped_step():
    """Section 9 *Board hardening*: the flag has to be able to turn the recovery off
    again. Skipping the install alone would leave a timer that an earlier run enabled
    still running the copy of the script installed back then, on its old thresholds."""
    text = BOARD_SCRIPT.read_text()
    assert "systemctl disable --now aqua-net-recover.timer" in text
    for dst in ('"$NET_TIMER_DST"', '"$NET_UNIT_DST"', '"$NET_RECOVER_DST"'):
        assert f"remove_path {dst}" in text


#: Executables the script may never reach for. Stubbed alongside nmcli/ip/ping so a
#: run that called one shows up in the call log, rather than being ruled out only by
#: reading the source.
FORBIDDEN_TOOLS = ("reboot", "shutdown", "poweroff", "halt", "systemctl")


def _stub_bin(
    root: Path,
    *,
    connected: bool,
    gateway: str,
    ping_ok: bool,
    connect_rc: int = 0,
    connect_delay_s: float = 0.0,
    profile: str = "board-wifi",
    up_rc: int = 0,
) -> Path:
    """nmcli / ip / ping stubs under ``root`` that log every call to ``root/calls.log``.

    ``connect_rc``/``connect_delay_s``/``up_rc`` are the branches the guards exist for:
    with the router off, neither `nmcli device connect` nor `nmcli connection up`
    returns 0 in a millisecond -- they fail, or they take long enough that systemd kills
    the unit part-way through.

    ``profile`` is the NetworkManager profile name the stub reports, and nothing in the
    script may know it in advance: a board's Wi-Fi profile is normally named after the
    SSID, which stays out of this repository.
    """
    bin_dir = root / "bin"
    bin_dir.mkdir(parents=True)
    state = "100 (connected)" if connected else "30 (disconnected)"
    # GENERAL.CONNECTION is "--" on a device that is not connected, which is exactly the
    # case that has to fall through to discovering the profile some other way.
    bound = profile if connected else "--"
    (bin_dir / "nmcli").write_text(
        "#!/bin/sh\n"
        f'echo "nmcli $*" >> "{root}/calls.log"\n'
        'case "$*" in\n'
        f'  *"connection up"*) exit {up_rc} ;;\n'
        f"  *\"device show\"*) printf 'GENERAL.STATE:{state}\\n'"
        f"'GENERAL.CONNECTION:{bound}\\nGENERAL.TYPE:wifi\\n' ;;\n"
        '  *"connection.interface-name"*) echo "connection.interface-name:wlan0" ;;\n'
        f'  *"NAME,TYPE"*) echo "{profile}:wifi" ;;\n'
        f'  *"device connect"*) sleep {connect_delay_s}; exit {connect_rc} ;;\n'
        "esac\n"
        "exit 0\n"
    )
    route = f"default via {gateway} proto dhcp metric 600" if gateway else ""
    (bin_dir / "ip").write_text(f'#!/bin/sh\necho "{route}"\nexit 0\n')
    (bin_dir / "ping").write_text(
        f'#!/bin/sh\necho "ping $*" >> "{root}/calls.log"\nexit {0 if ping_ok else 1}\n'
    )
    for name in FORBIDDEN_TOOLS:
        (bin_dir / name).write_text(f'#!/bin/sh\necho "{name} $*" >> "{root}/calls.log"\nexit 0\n')
    for name in ("nmcli", "ip", "ping", *FORBIDDEN_TOOLS):
        (bin_dir / name).chmod(0o755)
    (root / "calls.log").write_text("")
    return bin_dir


def _run_recovery(
    root: Path, bin_dir: Path, runs: int, extra_env: dict[str, str] | None = None
) -> list[str]:
    bash = shutil.which("bash")
    if bash is None:  # pragma: no cover - bash is in packages-rpi.txt and in CI
        pytest.skip("bash not available")
    env = {
        "PATH": f"{bin_dir}:/usr/bin:/bin",
        "AQUA_NET_STATE_DIR": str(root / "state"),
        "AQUA_NET_PING_DEADLINE_S": "1",
        **(extra_env or {}),
    }
    return [
        subprocess.run(
            [bash, str(NET_SCRIPT)], env=env, capture_output=True, text=True, check=True
        ).stdout
        for _ in range(runs)
    ]


def test_a_reachable_gateway_re_associates_nothing(tmp_path):
    bin_dir = _stub_bin(tmp_path, connected=True, gateway="192.0.2.1", ping_ok=True)
    _run_recovery(tmp_path, bin_dir, runs=5)
    calls = (tmp_path / "calls.log").read_text()
    assert "ping" in calls
    assert "device disconnect" not in calls and "device connect" not in calls


def test_a_router_that_is_simply_off_stops_bouncing_instead_of_looping(tmp_path):
    """The failure the owner forbade, seen from the other side: with nothing answering,
    the script re-associates AQUA_NET_MAX_BOUNCES times and then goes quiet, rather
    than bouncing the interface for as long as the router stays off."""
    bin_dir = _stub_bin(tmp_path, connected=True, gateway="192.0.2.1", ping_ok=False)
    bounces = int(_shell_default(NET_SCRIPT, "AQUA_NET_MAX_BOUNCES"))
    checks = int(_shell_default(NET_SCRIPT, "AQUA_NET_FAIL_CHECKS"))
    outputs = _run_recovery(tmp_path, bin_dir, runs=checks * (bounces + 4))
    calls = (tmp_path / "calls.log").read_text().splitlines()
    assert sum(1 for line in calls if "device disconnect" in line) == bounces
    assert sum(1 for line in calls if "device connect" in line) == bounces
    assert sum(1 for out in outputs if "stopping here" in out) == 1  # said once, not per run
    # And then nothing at all, rather than a "(1/2); waiting" line every second run in a
    # journal this same script caps at 200 MB.
    said_it = next(i for i, out in enumerate(outputs) if "stopping here" in out)
    assert [out for out in outputs[said_it + 1 :] if out.strip()] == []


def test_a_re_association_that_fails_is_still_an_attempt(tmp_path):
    """With the router off, `nmcli device connect` does not succeed -- and the give-up
    guard is written for exactly that branch. A failing re-association must move
    `bounces` all the same, or the script keeps bouncing the interface for as long as
    the router stays off, which is the loop the owner forbade."""
    bin_dir = _stub_bin(tmp_path, connected=True, gateway="192.0.2.1", ping_ok=False, connect_rc=1)
    bounces = int(_shell_default(NET_SCRIPT, "AQUA_NET_MAX_BOUNCES"))
    checks = int(_shell_default(NET_SCRIPT, "AQUA_NET_FAIL_CHECKS"))
    outputs = _run_recovery(tmp_path, bin_dir, runs=checks * (bounces + 4))
    calls = (tmp_path / "calls.log").read_text().splitlines()
    assert sum(1 for line in calls if "device connect" in line) == bounces
    assert sum(1 for out in outputs if "stopping here" in out) == 1
    assert sum(1 for out in outputs if "device connect" in out and "failed" in out) == bounces


def test_a_re_association_restores_autoconnect_before_it_reconnects(tmp_path):
    """`nmcli device disconnect` is an alias for `device down`, which also prevents the
    device from auto-activating until a manual activation (nmcli(1)). Restoring
    autoconnect *between* the two is what keeps a connect that fails -- or a run systemd
    kills mid-connect -- from leaving the board off the network until somebody logs in
    locally, with this script's own state guard then standing down every five minutes
    because "NetworkManager is already retrying"."""
    bin_dir = _stub_bin(tmp_path, connected=True, gateway="192.0.2.1", ping_ok=False, connect_rc=1)
    _run_recovery(tmp_path, bin_dir, runs=int(_shell_default(NET_SCRIPT, "AQUA_NET_FAIL_CHECKS")))
    calls = [line for line in (tmp_path / "calls.log").read_text().splitlines() if "nmcli" in line]
    order = [
        i
        for i, line in enumerate(calls)
        if "device disconnect" in line or "autoconnect yes" in line or "device connect" in line
    ]
    assert len(order) == 3 and order == sorted(order)
    assert "device disconnect" in calls[order[0]]
    assert "autoconnect yes" in calls[order[1]]
    assert "device connect" in calls[order[2]]


def test_a_re_association_killed_part_way_through_still_counts_as_a_bounce(tmp_path):
    """systemd's TimeoutStartSec is a real end to a run. The counters therefore move
    before the nmcli pair, not after: a killed run that advanced nothing would leave
    `bounces` at zero forever, and the give-up guard would never engage in the one case
    it was written for."""
    bash = shutil.which("bash")
    if bash is None:  # pragma: no cover - bash is in packages-rpi.txt and in CI
        pytest.skip("bash not available")
    bin_dir = _stub_bin(
        tmp_path,
        connected=True,
        gateway="192.0.2.1",
        ping_ok=False,
        connect_rc=1,
        connect_delay_s=30.0,
    )
    checks = int(_shell_default(NET_SCRIPT, "AQUA_NET_FAIL_CHECKS"))
    _run_recovery(tmp_path, bin_dir, runs=checks - 1)  # the failed checks before the bounce
    state = tmp_path / "state"
    env = {
        "PATH": f"{bin_dir}:/usr/bin:/bin",
        "AQUA_NET_STATE_DIR": str(state),
        "AQUA_NET_PING_DEADLINE_S": "1",
    }
    proc = subprocess.Popen([bash, str(NET_SCRIPT)], env=env, stdout=subprocess.DEVNULL)
    try:
        deadline = time.monotonic() + 30.0
        while "device connect" not in (tmp_path / "calls.log").read_text():
            assert proc.poll() is None and time.monotonic() < deadline, "never re-associated"
            time.sleep(0.05)
    finally:
        proc.kill()  # what systemd's TimeoutStartSec comes down to
        proc.wait(timeout=30)
    assert (state / "bounces").read_text().strip() == "1"
    calls = (tmp_path / "calls.log").read_text()
    assert "autoconnect yes" in calls  # restored before the connect, so NM retries alone


def test_no_gateway_is_a_no_op(tmp_path):
    """With the router off long enough the lease is gone, and a connected interface with
    no default route has nothing to probe: there is no address to ping and none may be
    written down here."""
    no_gateway = _stub_bin(tmp_path / "a", connected=True, gateway="", ping_ok=False)
    _run_recovery(tmp_path / "a", no_gateway, runs=3)
    assert "ping" not in (tmp_path / "a" / "calls.log").read_text()


def test_a_disconnected_interface_is_activated_instead_of_left_to_networkmanager(tmp_path):
    """The defect of 2026-09-30, and the one expectation in this file that was wrong:
    this test used to assert that a disconnected device is "NetworkManager's own retry to
    make" and therefore a no-op. NetworkManager had stopped retrying -- a disconnect
    storm spent its four default `connection.autoconnect-retries` in about three minutes,
    after which it blocks autoconnect for the profile until something resets it -- and the
    board sat unreachable for four days while this timer logged "standing by" every five
    minutes. The action for that state is a manual activation of the profile, which is
    also what resets NetworkManager's retry counter."""
    bin_dir = _stub_bin(tmp_path, connected=False, gateway="192.0.2.1", ping_ok=False)
    (out,) = _run_recovery(tmp_path, bin_dir, runs=1)
    calls = (tmp_path / "calls.log").read_text()
    assert "connection up id board-wifi" in calls
    assert "ping" not in calls  # a link that is down has nothing to probe
    assert "device disconnect" not in calls  # that is the other branch's action
    assert "disconnected" in out and "activating its profile" in out
    assert "standing by" not in out


@pytest.mark.parametrize("profile", ["board-wifi", "some-other-profile"])
def test_the_profile_to_activate_is_whatever_networkmanager_reports(tmp_path, profile):
    """A Wi-Fi profile is normally named after the SSID, so no name may be a literal in
    deploy/: the script activates whatever profile nmcli reports for the interface."""
    root = tmp_path / profile
    bin_dir = _stub_bin(root, connected=False, gateway="", ping_ok=False, profile=profile)
    _run_recovery(root, bin_dir, runs=1)
    assert f"connection up id {profile}" in (root / "calls.log").read_text()


def test_the_connection_name_is_a_variable_in_both_scripts():
    """Both the knob and every use of it: an operator override with a documented default
    of "discover it", and no profile name spelled out in a command."""
    assert _shell_default(NET_SCRIPT, "AQUA_NET_CONNECTION") == ""
    assert _shell_default(BOARD_SCRIPT, "NET_CONNECTION") == ""
    uses = 0
    for path in (NET_SCRIPT, BOARD_SCRIPT):
        for line in _code_lines(path):
            for match in re.finditer(r"connection (?:up id|modify) (\S+)", line):
                uses += 1
                assert match.group(1).startswith('"$'), f"{path.name}: {line}"
    assert uses == 2  # one activation in the script, one modify in the installer


def test_a_disconnected_interface_is_retried_at_its_own_rate_not_the_timers(tmp_path):
    """AQUA_NET_RECONNECT_MIN_S, not the timer, decides how hard an absent access point
    is knocked on -- so the owner may run the timer as often as they like for faster
    detection without turning the cadence into retry pressure."""
    bin_dir = _stub_bin(tmp_path, connected=False, gateway="", ping_ok=False)
    _run_recovery(tmp_path, bin_dir, runs=5)  # five checks well inside one interval
    calls = (tmp_path / "calls.log").read_text().splitlines()
    assert sum(1 for line in calls if "connection up" in line) == 1
    other = _stub_bin(tmp_path / "fast", connected=False, gateway="", ping_ok=False)
    _run_recovery(tmp_path / "fast", other, runs=5, extra_env={"AQUA_NET_RECONNECT_MIN_S": "0"})
    calls = (tmp_path / "fast" / "calls.log").read_text().splitlines()
    assert sum(1 for line in calls if "connection up" in line) == 5


def test_a_link_down_for_hours_says_so_loudly_instead_of_in_identical_lines(tmp_path):
    """Four days of `wlan0 is not connected (30 (disconnected)); NetworkManager owns
    that, standing` every five minutes, with nothing happening behind them, is what this
    replaced. A long outage now gets one line an hour that names how long and how many
    attempts -- and words it so that reading it can never be mistaken for a reason to
    escalate, because with the router off that line is the entire response."""
    bin_dir = _stub_bin(tmp_path, connected=False, gateway="", ping_ok=False)
    state = tmp_path / "state"
    state.mkdir()
    (state / "down-since").write_text(f"{int(time.time()) - 5 * 3600}\n")
    (state / "down-tries").write_text("50\n")
    outputs = _run_recovery(tmp_path, bin_dir, runs=3)
    loud = [out for out in outputs if "WARNING" in out]
    assert len(loud) == 1  # once an hour, not once a run
    assert "5h0m" in loud[0] and "51 activation attempt" in loud[0]
    assert "no reboot" in loud[0] and "fans are unaffected" in loud[0]


def test_the_access_point_being_absent_is_a_wait_and_never_an_escalation(tmp_path):
    """The owner's standing rule: switching off the home router must never degrade
    cooling. Every activation then fails, and the only allowed response is to keep
    waiting -- no counter of failures may ever unlock a bigger hammer, because there is
    no bigger hammer here to unlock."""
    bin_dir = _stub_bin(tmp_path, connected=False, gateway="", ping_ok=False, up_rc=1)
    outputs = _run_recovery(tmp_path, bin_dir, runs=6, extra_env={"AQUA_NET_RECONNECT_MIN_S": "0"})
    calls = (tmp_path / "calls.log").read_text()
    assert calls.count("connection up") == 6  # still only ever this, six failures deep
    assert any("wait and not a fault" in out for out in outputs)


@pytest.mark.parametrize(
    ("connected", "ping_ok", "up_rc"), [(True, False, 0), (False, False, 0), (False, False, 1)]
)
def test_no_state_of_the_link_ever_reboots_or_restarts_anything(
    tmp_path, connected, ping_ok, up_rc
):
    """The behavioural twin of test_network_recovery_never_escalates_beyond_re_associating,
    which reads the source: reboot, shutdown, poweroff, halt and systemctl are all on PATH
    as logging stubs, so a run that reached for one would show up in the call log. Checked
    for a link that is up but dead, a disconnected link that comes back, and one that
    never does."""
    bin_dir = _stub_bin(
        tmp_path, connected=connected, gateway="192.0.2.1", ping_ok=ping_ok, up_rc=up_rc
    )
    _run_recovery(tmp_path, bin_dir, runs=8, extra_env={"AQUA_NET_RECONNECT_MIN_S": "0"})
    calls = (tmp_path / "calls.log").read_text()
    for forbidden in FORBIDDEN_TOOLS:
        assert forbidden not in calls


def test_the_link_coming_back_clears_the_outage_and_says_what_it_took(tmp_path):
    """The payoff line: the journal has to show that the activations were what ended the
    outage, and the counters have to reset so the next one starts from zero."""
    down = _stub_bin(tmp_path / "down", connected=False, gateway="", ping_ok=False)
    _run_recovery(tmp_path / "down", down, runs=1)
    up = _stub_bin(tmp_path / "up", connected=True, gateway="192.0.2.1", ping_ok=True)
    # The same state dir, with a connected device in front of it now.
    (out,) = _run_recovery(tmp_path / "down", up, runs=1)
    assert "is connected again after 1 activation attempt(s)" in out
    assert (tmp_path / "down" / "state" / "down-tries").read_text().strip() == "0"


# --- the network diagnostic watcher (section 9, "Board hardening") ---------------------

WATCH_SCRIPT = DEPLOY / "aqua-net-watch.sh"
WATCH_UNIT = DEPLOY / "aqua-net-watch.service"

#: Executables the watcher may never reach for. Stubbed alongside the read-only ones
#: so a run that called one shows up in the call log rather than being ruled out only
#: by reading the source. `modprobe`/`rmmod` are on the list because the 2026-10-06
#: fault was a radio that never registered, and reloading the driver -- the one thing
#: that might bring it back -- is deliberately not this unit's decision.
WATCH_FORBIDDEN = (
    "reboot",
    "shutdown",
    "poweroff",
    "halt",
    "systemctl",
    "modprobe",
    "rmmod",
    "insmod",
    "ping",
    "arping",
    "dhclient",
    "ifconfig",
    "iwconfig",
)


def _route_hex(addr: str) -> str:
    """An IPv4 address as /proc/net/route spells a gateway: little-endian hex."""
    return "".join(f"{int(octet):02X}" for octet in reversed(addr.split(".")))


def _watch_stage(
    path: Path,
    *,
    dev: bool = True,
    operstate: str = "up",
    carrier: str = "1",
    wpa: str = "COMPLETED",
    bssid: str = "aa:bb:cc:dd:ee:ff",
    ssid: str = "a-network",
    freq: str = "2437",
    v4: str = "192.0.2.23/24",
    gw: str = "192.0.2.1",
    nud: str = "REACHABLE",
    lladdr: bool = True,
    rx: int = 1000,
    tx: int = 900,
    throttled: str = "0x0",
    volts: str = "1.3250V",
    temp: str = "47.1",
    uv: str = "0",
    wpa_err: str = "",
    iw_silent: bool = False,
) -> None:
    """One sampled state of the board, as a fake /sys + /proc + stub data under ``path``.

    ``dev=False`` is the lowest rung: no ``/sys/class/net/wlan0`` at all, which is what
    the 2026-10-06 boot looked like from userspace after the radio's firmware download
    failed its checksum. Everything above it then has to read "na" and not "DOWN".
    """
    (path / "sys/class/net/lo").mkdir(parents=True, exist_ok=True)
    (path / "sys/class/net/lo/operstate").write_text("unknown\n")
    if dev:
        stats = path / "sys/class/net/wlan0/statistics"
        stats.mkdir(parents=True, exist_ok=True)
        (path / "sys/class/net/wlan0/operstate").write_text(f"{operstate}\n")
        (path / "sys/class/net/wlan0/carrier").write_text(f"{carrier}\n")
        for name, value in (
            ("rx_packets", rx),
            ("tx_packets", tx),
            ("rx_errors", 0),
            ("tx_errors", 0),
            ("rx_dropped", 0),
            ("tx_dropped", 0),
        ):
            (stats / name).write_text(f"{value}\n")
    zone = path / "sys/class/thermal/thermal_zone0"
    zone.mkdir(parents=True, exist_ok=True)
    (zone / "temp").write_text("47000\n")
    hwmon = path / "sys/class/hwmon/hwmon0"
    hwmon.mkdir(parents=True, exist_ok=True)
    (hwmon / "in0_lcrit_alarm").write_text(f"{uv}\n")

    proc = path / "proc/net"
    proc.mkdir(parents=True, exist_ok=True)
    wireless = "Inter-|  sta-|   Quality\n face | tus | link level noise\n"
    if dev:
        wireless += " wlan0: 0000   58.  -62.  -256        0      0      0      0      0        0\n"
    (proc / "wireless").write_text(wireless)
    route = "Iface\tDestination\tGateway\tFlags\tRefCnt\tUse\tMetric\tMask\tMTU\tWindow\tIRTT\n"
    if dev and gw:
        route += f"wlan0\t00000000\t{_route_hex(gw)}\t0003\t0\t0\t600\t00000000\t0\t0\t0\n"
    (proc / "route").write_text(route)
    (path / "proc/modules").write_text(
        "brcmfmac 319488 0 - Live 0x0\ncfg80211 806912 1 brcmfmac, Live 0x0\n"
    )
    (path / "proc/loadavg").write_text("0.10 0.20 0.30 1/80 999\n")
    (path / "proc/meminfo").write_text("MemTotal:  444444 kB\nMemAvailable:  222222 kB\n")
    (path / "proc/uptime").write_text("12345.67 11111.11\n")
    random = path / "proc/sys/kernel/random"
    random.mkdir(parents=True, exist_ok=True)
    (random / "boot_id").write_text("11111111-2222-3333-4444-555555555555\n")

    data = path / "data"
    data.mkdir(parents=True, exist_ok=True)
    addr = ""
    if dev:
        if v4:
            addr += f"3: wlan0    inet {v4} brd 192.0.2.255 scope global dynamic wlan0\n"
        addr += "3: wlan0    inet6 fd00:2:3::17/64 scope global\n"
        addr += "3: wlan0    inet6 fe80::1/64 scope link\n"
    (data / "addr").write_text(addr)
    neigh = ""
    if dev and gw:
        neigh = f"{gw} lladdr aa:bb:cc:dd:ee:ff {nud}\n" if lladdr else f"{gw} {nud}\n"
    (data / "neigh4").write_text(neigh)
    status = ""
    if dev and bssid:
        status += f"bssid={bssid}\nfreq={freq}\nssid={ssid}\n"
    if dev:
        status += f"wpa_state={wpa}\n"
    (data / "wpa").write_text(status)
    # What wpa_cli says instead of a status when it cannot reach the supplicant.
    # On the board it was exactly this, for days, and the first version of this
    # script captured it and threw it away.
    (data / "wpa_err").write_text(f"{wpa_err}\n" if wpa_err else "")
    iw = ""
    if dev and not iw_silent:
        iw = (
            f"Connected to {bssid} (on wlan0)\n\tSSID: {ssid}\n\tfreq: {freq}\n\tsignal: -62 dBm\n"
            if bssid
            else "Not connected.\n"
        )
    (data / "iw").write_text(iw)
    (data / "vc_throttled").write_text(f"throttled={throttled}\n")
    (data / "vc_volts").write_text(f"volt={volts}\n")
    (data / "vc_temp").write_text(f"temp={temp}'C\n")


def _watch_board(base: Path, stages: list[dict[str, object]]) -> Path:
    """A board whose state walks through ``stages``, one step per sample.

    The step is driven by the stub `sleep` on PATH, which the script calls exactly once
    per loop and nothing else calls: it moves a symlink that is the root of the whole
    fake /sys and /proc, so a sample sees one stage whole, and the tests have no
    wall-clock dependence at all (the stub returns at once, so they also run fast).
    """
    for index, spec in enumerate(stages):
        _watch_stage(base / "stages" / str(index), **spec)  # type: ignore[arg-type]
    (base / "stage").write_text("0")
    (base / "root").symlink_to("stages/0")

    bin_dir = base / "bin"
    bin_dir.mkdir(parents=True)
    (bin_dir / "sleep").write_text(
        "#!/bin/sh\n"
        'echo "sleep $*" >> "$CALLS"\n'
        'n=$(cat "$W_BASE/stage")\n'
        "next=$((n + 1))\n"
        '[ -d "$W_BASE/stages/$next" ] || next="$n"\n'
        'echo "$next" > "$W_BASE/stage"\n'
        'ln -sfn "stages/$next" "$W_BASE/root"\n'
    )
    (bin_dir / "ip").write_text(
        "#!/bin/sh\n"
        'echo "ip $*" >> "$CALLS"\n'
        'case "$*" in\n'
        '  *"-o addr show dev"*) cat "$W_ROOT/data/addr" ;;\n'
        '  *"neigh show"*) cat "$W_ROOT/data/neigh4" ;;\n'
        "esac\n"
    )
    (bin_dir / "wpa_cli").write_text(
        "#!/bin/sh\n"
        'echo "wpa_cli $*" >> "$CALLS"\n'
        'if [ -s "$W_ROOT/data/wpa_err" ]; then cat "$W_ROOT/data/wpa_err" >&2; exit 255; fi\n'
        'case "$*" in\n'
        '  *status*) cat "$W_ROOT/data/wpa" ;;\n'
        '  *signal_poll*) echo "RSSI=-62" ;;\n'
        '  *scan_results*) echo "aa:bb:cc:dd:ee:ff\t2437\t-62\t[ESS]\ta-network" ;;\n'
        "esac\n"
    )
    # `iw dev <iface> link` is the fallback source: it reads the association the
    # driver already holds and starts nothing.
    (bin_dir / "iw").write_text('#!/bin/sh\necho "iw $*" >> "$CALLS"\ncat "$W_ROOT/data/iw"\n')
    (bin_dir / "vcgencmd").write_text(
        "#!/bin/sh\n"
        'echo "vcgencmd $*" >> "$CALLS"\n'
        'case "$1 $2" in\n'
        '  "get_throttled ") cat "$W_ROOT/data/vc_throttled" ;;\n'
        '  "measure_volts core") cat "$W_ROOT/data/vc_volts" ;;\n'
        '  "measure_temp ") cat "$W_ROOT/data/vc_temp" ;;\n'
        "esac\n"
    )
    (bin_dir / "journalctl").write_text(
        "#!/bin/sh\n"
        'echo "journalctl $*" >> "$CALLS"\n'
        'case "$*" in\n'
        "  *--list-boots*)\n"
        '    printf "IDX BOOT ID FIRST ENTRY LAST ENTRY\\n"\n'
        '    printf " -1 bbbb2222 Mon 2026-10-05 02:07:01 UTC Mon 2026-10-05 02:07:07 UTC\\n"\n'
        '    printf "  0 cccc3333 Mon 2026-10-05 02:07:13 UTC Mon 2026-10-05 03:00:00 UTC\\n"\n'
        "    ;;\n"
        '  *"-k -b"*)\n'
        '    printf "kernel: brcmfmac: verifymemory: Downloaded RAM image is corrupted\\n"\n'
        '    printf "kernel: brcmfmac: dongle image file download failed\\n"\n'
        '    printf "kernel: mmc1: Controller never released inhibit bit(s).\\n"\n'
        "    ;;\n"
        '  *-k*) echo "kernel: brcmfmac: nothing to report" ;;\n'
        '  *) echo "a journal line" ;;\n'
        "esac\n"
    )
    (bin_dir / "nmcli").write_text(
        '#!/bin/sh\necho "nmcli $*" >> "$CALLS"\necho "GENERAL.STATE:100 (connected)"\n'
    )
    for name in WATCH_FORBIDDEN:
        (bin_dir / name).write_text(f'#!/bin/sh\necho "{name} $*" >> "$CALLS"\nexit 0\n')
    for entry in bin_dir.iterdir():
        entry.chmod(0o755)
    (base / "calls.log").write_text("")
    return bin_dir


def _run_watch(
    base: Path, bin_dir: Path, samples: int, extra_env: dict[str, str] | None = None
) -> list[str]:
    bash = shutil.which("bash")
    if bash is None:  # pragma: no cover - bash is in packages-rpi.txt and in CI
        pytest.skip("bash not available")
    env = {
        "PATH": f"{bin_dir}:/usr/bin:/bin",
        "AQUA_NETWATCH_ROOT": str(base / "root"),
        "AQUA_NETWATCH_IFACE": "wlan0",
        "AQUA_NETWATCH_STATE_DIR": str(base / "state"),
        "AQUA_NETWATCH_SNAPSHOT_MIN_S": "0",
        "AQUA_NETWATCH_FULL_MIN_S": "0",
        "W_BASE": str(base),
        "W_ROOT": str(base / "root"),
        "CALLS": str(base / "calls.log"),
        **(extra_env or {}),
    }
    done = subprocess.run(
        [bash, str(WATCH_SCRIPT), "--samples", str(samples)],
        env=env,
        capture_output=True,
        text=True,
        check=True,
    )
    return done.stdout.splitlines()


def _kinds(lines: list[str], kind: str) -> list[str]:
    return [line for line in lines if line.startswith(f"{kind} ")]


def test_watcher_knobs_are_documented_variables_with_one_default_each():
    """Section 9: no operator tunable is a literal in the body. Every cadence,
    threshold, interface name and path is a knob at the top with its reasoning."""
    knobs = {
        "AQUA_NETWATCH_IFACE": "wlan0",
        "AQUA_NETWATCH_INTERVAL_S": "15",
        "AQUA_NETWATCH_OUTAGE_INTERVAL_S": "30",
        "AQUA_NETWATCH_IDLE_EVERY": "2",
        "AQUA_NETWATCH_SNAPSHOT_MIN_S": "120",
        "AQUA_NETWATCH_FULL_MIN_S": "900",
        "AQUA_NETWATCH_SNAPSHOT_LINES": "25",
        "AQUA_NETWATCH_SCAN_MAX": "20",
        "AQUA_NETWATCH_SILENT_SAMPLES": "8",
        "AQUA_NETWATCH_LOUD_EVERY_S": "3600",
        "AQUA_NETWATCH_CMD_TIMEOUT_S": "5",
        "AQUA_NETWATCH_DRIVER_MODULE": "brcmfmac",
        "AQUA_NETWATCH_STATE_DIR": "/var/lib/aqua-net-watch",
        "AQUA_NETWATCH_BOOT_LIST": "10",
        "AQUA_NETWATCH_THERMAL_ZONE": "thermal_zone0",
        "AQUA_NETWATCH_WPA_UNIT": "wpa_supplicant.service",
        "AQUA_NETWATCH_WPA_CTRL": "",
        "AQUA_NETWATCH_ROOT": "",
    }
    for name, default in knobs.items():
        assert _shell_default(WATCH_SCRIPT, name) == default, name
    # The two it shares with the board script and the unit, and the interface name,
    # which each file carries its own copy of on purpose (a unit started by systemd
    # has no environment to inherit one from).
    assert _shell_default(WATCH_SCRIPT, "AQUA_NETWATCH_IFACE") == _shell_default(
        NET_SCRIPT, "AQUA_NET_IFACE"
    )


def test_the_watcher_unit_ships_the_cadences_the_board_script_installs():
    """Three files carry these two numbers -- the script's default, the unit's
    Environment=, and the installer's knob -- so all three have to agree or reading
    deploy/ tells the truth about none of them."""
    values = _unit_values(WATCH_UNIT.read_text())
    environment = dict(item.split("=", 1) for item in values["Environment"])
    assert environment["AQUA_NETWATCH_INTERVAL_S"] == _shell_default(
        BOARD_SCRIPT, "NET_WATCH_INTERVAL"
    )
    assert environment["AQUA_NETWATCH_OUTAGE_INTERVAL_S"] == _shell_default(
        BOARD_SCRIPT, "NET_WATCH_OUTAGE_INTERVAL"
    )
    assert environment["AQUA_NETWATCH_INTERVAL_S"] == _shell_default(
        WATCH_SCRIPT, "AQUA_NETWATCH_INTERVAL_S"
    )
    assert environment["AQUA_NETWATCH_OUTAGE_INTERVAL_S"] == _shell_default(
        WATCH_SCRIPT, "AQUA_NETWATCH_OUTAGE_INTERVAL_S"
    )
    text = BOARD_SCRIPT.read_text()
    for variable, directive in (
        ("NET_WATCH_INTERVAL", "AQUA_NETWATCH_INTERVAL_S"),
        ("NET_WATCH_OUTAGE_INTERVAL", "AQUA_NETWATCH_OUTAGE_INTERVAL_S"),
    ):
        assert f"Environment={directive}=${variable}" in text, directive


def test_the_watcher_unit_is_wired_to_nothing_that_cools():
    values = _unit_values(WATCH_UNIT.read_text())
    assert values["Type"] == ["simple"]
    for key in ("Wants", "Requires", "After", "Before", "Conflicts", "PartOf", "BindsTo"):
        for value in values.get(key, []):
            assert "aqua-bridge" not in value and "aqua-heartbeat" not in value
            assert "aqua-net-recover" not in value  # the two must not sequence each other
            assert "network-online" not in value  # it exists for the offline case
    # No way for an observer to escalate: no watchdog to miss, no failure action, and
    # the start rate limit switched off so systemd cannot park it as failed either.
    assert "WatchdogSec" not in values
    assert "OnFailure" not in values
    assert "StartLimitAction" not in values
    assert values["StartLimitIntervalSec"] == ["0"]
    assert values["Restart"] == ["always"]
    # The marker has to outlive a reboot, which is the whole mechanism of the boot
    # report, so it is StateDirectory= (on /var) and never RuntimeDirectory= (tmpfs,
    # erased by exactly the event it exists to detect).
    assert values["StateDirectory"] == ["aqua-net-watch"]
    assert "RuntimeDirectory" not in values
    assert _shell_default(WATCH_SCRIPT, "AQUA_NETWATCH_STATE_DIR") == "/var/lib/aqua-net-watch"


def test_the_watcher_unit_cannot_send_a_packet_or_open_a_controller():
    """The inertness promise, enforced by systemd and not only by the script. No
    address family that can carry a packet -- the watcher's whole value depends on NOT
    keeping the gateway's ARP entry warm and NOT keeping the radio out of power save --
    no capabilities, and a private /dev whose one physical node is the firmware mailbox
    the rail is measured through."""
    values = _unit_values(WATCH_UNIT.read_text())
    families = values["RestrictAddressFamilies"][0].split()
    assert set(families) == {"AF_UNIX", "AF_NETLINK"}
    assert values["CapabilityBoundingSet"] == [""]
    assert values["AmbientCapabilities"] == [""]
    # NOT PrivateDevices=yes, measured on the board: a private /dev is a fresh
    # tmpfs that DeviceAllow= does not bind a physical node back into, so the
    # firmware tool could not be reached and every sample read "thr=? volt=?" --
    # the rail measurement silently missing, which is worse than never having
    # written it. DevicePolicy=closed keeps the guarantee that mattered: only the
    # API pseudo-devices and the nodes named here, so a controller's hidraw node
    # is as unopenable as it was.
    assert "PrivateDevices" not in values
    assert values["DevicePolicy"] == ["closed"]
    assert values["DeviceAllow"] == ["/dev/vcio r", "/dev/vchiq rw"]
    for allowed in values["DeviceAllow"]:
        assert "hidraw" not in allowed and "usb" not in allowed, allowed
    assert values["ProtectSystem"] == ["strict"]
    assert values["NoNewPrivileges"] == ["yes"]
    assert values["ProtectKernelModules"] == ["yes"]
    # ProcSubset=pid would hide /proc/net/route, /proc/net/wireless and /proc/modules,
    # which are three of the things this unit exists to read.
    assert "ProcSubset" not in values
    # wpa_cli has to create a socket of its own before the supplicant can reply to
    # it, and under ProtectSystem=strict there was nowhere to put it; a private
    # /tmp is writable but invisible to the supplicant, which is the other half of
    # the same fault. Both candidate paths from the binary's own strings are
    # granted, tolerating absence, and nothing else is.
    assert values["PrivateTmp"] == ["no"]
    writable = values["ReadWritePaths"][0].split()
    assert set(writable) == {"-/tmp", "-/run/wpa_supplicant"}
    # And specifically NOT the state of anything else on the board: the model
    # store and the recovery script's counters stay out of reach.
    for path in writable:
        assert "aqua-bridge" not in path and "aqua-net-recover" not in path, path
    for key in ("ExecStart", "ExecStop", "ExecStartPre", "ExecReload"):
        for value in values.get(key, []):
            assert "aqua-net-watch.sh" in value, value


def test_the_watcher_never_runs_anything_that_acts():
    """Read from the source, the twin of the behavioural check below. Note modprobe:
    the 2026-10-06 fault was a radio that never registered, and reloading the driver --
    the one thing that might bring it back -- is the owner's decision and a separate
    unit, deliberately not this one's."""
    lines = _code_lines(WATCH_SCRIPT)
    for word in WATCH_FORBIDDEN:
        assert not _runs_command(lines, word), f"aqua-net-watch.sh runs {word}"
    for line in lines:
        assert "aqua-bridge" not in line or line.startswith("NET_WATCH"), line
        assert "aqua-heartbeat" not in line and "hidraw" not in line, line
    # Code only, never the comments: the header argues at length about the commands it
    # does NOT run, and naming them there is the point.
    code = "\n".join(lines) + "\n"
    # The read-only supplicant verbs, and not the ones that make the radio act.
    for verb in ("status", "signal_poll", "scan_results"):
        assert f'"${{WPA_ARGS[@]}}" {verb}' in code, verb
    for verb in ("scan", "reassociate", "reconnect", "disconnect", "set_network", "save_config"):
        assert f'"${{WPA_ARGS[@]}}" {verb}\n' not in code, verb
    # nmcli is read twice and written never; "device wifi list" is left out on purpose
    # because it can trigger a scan.
    assert "nmcli -t device show" in code and "nmcli -t connection show --active" in code
    for forbidden in ("connection up", "connection modify", "device connect", "device wifi"):
        assert forbidden not in code, forbidden


def test_the_watcher_names_every_rung_of_the_ladder_lowest_first():
    """The ladder is the diagnosis, and its order is load-bearing: absence of the
    interface is the lowest rung, below the association, because on 2026-10-06 there was
    no netdev at all and the four layers above it had nothing to report."""
    text = WATCH_SCRIPT.read_text()
    assert "LAYERS=(iface assoc v4 rt gw)" in text
    # v6 and the rail can be transitions but are never rungs: a changed IPv6 prefix is
    # not an outage (the owner is explicit that it is a distraction), and a latched
    # throttle bit would otherwise keep a board that sagged once degraded all boot.
    assert "EXTRA_LAYERS=(v6 power)" in text


def test_a_healthy_board_samples_without_dumping_anything(tmp_path):
    """One snapshot, at the start, and then nothing: a dump per sample is not a
    diagnosis, it is a full journal. The idle cadence also applies, so a steady board
    costs one line every AQUA_NETWATCH_IDLE_EVERY samples."""
    stages = [{} for _ in range(8)]
    bin_dir = _watch_board(tmp_path, stages)
    lines = _run_watch(tmp_path, bin_dir, samples=8, extra_env={"AQUA_NETWATCH_IDLE_EVERY": "4"})
    assert len(_kinds(lines, "snapshot begin")) == 1
    assert _kinds(lines, "CHANGE") == []
    assert _kinds(lines, "DEGRADED") == []
    assert _kinds(lines, "POWER") == []
    samples = _kinds(lines, "sample")
    assert all("layers=ok" in line for line in samples), samples
    # The first sample always prints; after that one in four.
    assert 2 <= len(samples) <= 4, samples


def test_an_association_that_drops_under_a_healthy_address_says_which_held(tmp_path):
    """The question the whole file exists to answer. Four failures look identical from
    outside the board, and the one line that tells them apart names what moved next to
    what did not: here the association goes while the address, the route and the
    gateway are all still fine."""
    stages = [
        {},
        {"wpa": "DISCONNECTED", "bssid": "", "operstate": "down", "carrier": "0"},
        {"wpa": "DISCONNECTED", "bssid": "", "operstate": "down", "carrier": "0"},
    ]
    bin_dir = _watch_board(tmp_path, stages)
    lines = _run_watch(tmp_path, bin_dir, samples=3)
    (change,) = _kinds(lines, "CHANGE")
    assert "layers=assoc" in change
    assert "wpa=COMPLETED->DISCONNECTED" in change
    assert "bss=aa:bb:cc:dd:ee:ff->none" in change
    # The interface held too: it existed throughout, which is the rung below.
    assert "held=iface,v4,rt,gw" in change
    (degraded,) = _kinds(lines, "DEGRADED")
    assert "first=assoc" in degraded
    assert len(_kinds(lines, "snapshot begin")) == 2  # the start one, and this


def test_an_address_that_goes_under_a_healthy_association_says_which_held(tmp_path):
    """The second of the four: associated throughout, and the lease is what vanished.
    A DHCP fault, not a radio one -- and told apart from the first only by this line."""
    stages = [{}, {"v4": ""}]
    bin_dir = _watch_board(tmp_path, stages)
    lines = _run_watch(tmp_path, bin_dir, samples=2)
    (change,) = _kinds(lines, "CHANGE")
    assert "layers=v4" in change and "v4=192.0.2.23/24->none" in change
    assert "held=iface,assoc,rt,gw" in change
    assert "first=v4" in _kinds(lines, "DEGRADED")[0]


def test_a_route_that_goes_leaves_the_gateway_unknowable_rather_than_down(tmp_path):
    """The third: address and association hold, the default route is what went. The
    gateway then reads "na" and not "DOWN", because whether it answers is unknowable
    with no route to it -- and four spurious DOWNs would bury the rung that moved."""
    stages = [{}, {"gw": ""}]
    bin_dir = _watch_board(tmp_path, stages)
    lines = _run_watch(tmp_path, bin_dir, samples=2)
    (change,) = _kinds(lines, "CHANGE")
    assert "layers=rt," in change or change.rstrip().endswith("layers=rt")
    assert "held=iface,assoc,v4" in change
    assert "rt:DOWN" in _kinds(lines, "sample")[-1]
    assert "gw:na" in _kinds(lines, "sample")[-1]
    assert "first=rt" in _kinds(lines, "DEGRADED")[0]


def test_a_gateway_that_stops_answering_is_the_only_thing_that_moved(tmp_path):
    """The fourth: everything on this board holds and the other end of the link stops
    answering. The kernel's own neighbour verdict is where that shows up, and the
    watcher reads it without sending a packet of its own -- aqua-net-recover.sh's
    five-minutely ping is what refreshes it."""
    stages = [{}, {"nud": "FAILED", "lladdr": False}]
    bin_dir = _watch_board(tmp_path, stages)
    lines = _run_watch(tmp_path, bin_dir, samples=2)
    (change,) = _kinds(lines, "CHANGE")
    assert "layers=gw" in change and "nud=REACHABLE->FAILED" in change
    assert "held=iface,assoc,v4,rt" in change
    assert "first=gw" in _kinds(lines, "DEGRADED")[0]
    assert "ping" not in (tmp_path / "calls.log").read_text()


def test_an_absent_interface_is_the_lowest_rung_and_carries_the_kernels_verdict(tmp_path):
    """2026-10-06: the brcmfmac firmware download over SDIO failed its checksum during
    early boot, so the driver never registered a netdev. A watcher that only read
    /sys/class/net/wlan0/operstate would have logged "no such file" for thirty hours.
    The first sample has to say exactly what is wrong instead -- and the verdict was
    printed once, in early boot, so it is read from the TOP of this boot's kernel log
    and not from a tail of the recent ring buffer."""
    bin_dir = _watch_board(tmp_path, [{"dev": False}, {"dev": False}])
    lines = _run_watch(tmp_path, bin_dir, samples=2)
    (absent,) = _kinds(lines, "ABSENT")
    assert "does not exist" in absent
    assert "module=brcmfmac:loaded" in absent  # loaded, and still no interface
    assert "netdevs=lo" in absent  # and here is what does exist
    assert "Downloaded RAM image is corrupted" in absent  # the verdict, quoted
    assert "first=iface" in _kinds(lines, "DEGRADED")[0]
    first_sample = _kinds(lines, "sample")[0]
    assert "dev=absent" in first_sample
    assert "layers=iface:DOWN,assoc:na,v4:na,rt:na,gw:na" in first_sample
    # No counters either, rather than zeroes: a zeroed counter would read as a delta of
    # minus the whole counter on the way out and plus the whole counter on the way back,
    # which is two lies about traffic that never happened.
    assert "rx=- tx=-" in first_sample and "rxe=- " in first_sample
    # The rail is still measured, though, and that is the point: on 2026-10-06 the
    # radio and the supply were one question, and this is the sample that has to carry
    # the answer even with no interface to look at.
    assert "thr=0x0" in first_sample and "volt=1.3250V" in first_sample
    body = [line for line in lines if line.startswith("snapshot [")]
    assert any("[absent] verdict=kernel-logged-a-failure" in line for line in body)
    assert any("[netdevs] lo" in line for line in body)
    assert any("[modules] brcmfmac" in line for line in body)
    assert any("[bootradio] " in line and "download failed" in line for line in body)
    # And it fixes nothing: reloading the driver is the owner's decision.
    assert "reloading the driver" in absent
    calls = (tmp_path / "calls.log").read_text()
    for forbidden in WATCH_FORBIDDEN:
        assert forbidden not in calls
    # Nothing to ask about an interface that does not exist, so the per-sample forks
    # are not made at all; the only ip/wpa_cli calls are the snapshot's.
    assert "wpa_cli -i wlan0 status" not in calls


def test_a_roam_between_two_access_points_is_visible_at_a_glance(tmp_path):
    """Since 2026-10-05 two access points broadcast one SSID, so the BSSID is the only
    field that can tell a roam from a reconnect -- which is why it is on every sample."""
    stages = [{}, {"bssid": "11:22:33:44:55:66", "freq": "2412"}]
    bin_dir = _watch_board(tmp_path, stages)
    lines = _run_watch(tmp_path, bin_dir, samples=2)
    (roam,) = _kinds(lines, "ROAM")
    assert "bss=aa:bb:cc:dd:ee:ff->11:22:33:44:55:66" in roam
    assert "fq=2437->2412" in roam
    assert "association never dropped" in roam
    assert _kinds(lines, "DEGRADED") == []  # a roam is not an outage


def test_a_changed_ipv6_prefix_is_not_an_outage_and_costs_no_dump(tmp_path):
    """The owner is explicit that the ISP re-dialling and changing the IPv6 global
    prefix is a distraction and not the fault being hunted. So the digest carries
    presence per scope, never the prefixes: a new global replacing an old one is not a
    state change and must not spend a snapshot."""
    stages: list[dict[str, object]] = [{}, {}]
    bin_dir = _watch_board(tmp_path, stages)
    # Rewrite stage 1's addresses with a different global prefix, same scopes.
    addr = (tmp_path / "stages/1/data/addr").read_text()
    (tmp_path / "stages/1/data/addr").write_text(addr.replace("fd00:2:3::17", "fd00:9:9::42"))
    lines = _run_watch(tmp_path, bin_dir, samples=2)
    assert _kinds(lines, "CHANGE") == []
    assert len(_kinds(lines, "snapshot begin")) == 1


def test_the_rail_is_on_every_sample_and_a_sag_is_a_transition_of_its_own(tmp_path):
    """The 2026-10-06 forensics moved the question: the board did not fail at the
    network, it died, twenty times, about six seconds into each boot. The firmware's
    throttle flags only latch within the boot they happen in, so every clean reading we
    have was taken after the reset that cleared them. This is the measurement that
    closes that gap -- on every sample, so that the last line before a death carries
    the rail, and as a transition with a full dump behind it."""
    stages = [{}, {"throttled": "0x50005", "volts": "1.2000V", "uv": "1"}]
    bin_dir = _watch_board(tmp_path, stages)
    lines = _run_watch(tmp_path, bin_dir, samples=2)
    first = _kinds(lines, "sample")[0]
    assert "thr=0x0" in first and "volt=1.3250V" in first and "uv=0" in first
    assert "temp=47.1" in first
    (power,) = _kinds(lines, "POWER")
    assert "thr=0x0->0x50005" in power
    assert "now=under-voltage,throttled" in power
    assert "since-boot=under-voltage,throttled" in power
    assert "uv=0->1" in power
    assert "this boot only" in power
    begins = _kinds(lines, "snapshot begin")
    assert len(begins) == 2 and "kind=full" in begins[1] and "POWER" in begins[1]
    # A sag is not a rung of the ladder: the board is not "degraded" and the cadence
    # does not slow, or a board that sagged once would stay degraded until its reset.
    assert _kinds(lines, "DEGRADED") == []


def test_the_boot_report_distinguishes_a_shutdown_from_a_power_loss(tmp_path):
    """A kernel cannot log its own power loss, so it is recorded from the other side:
    ExecStop= writes a marker, and the next start either finds it or does not. That one
    line turns "did it reboot or did it lose power" from an inference into a fact."""
    bash = shutil.which("bash")
    if bash is None:  # pragma: no cover - bash is in packages-rpi.txt and in CI
        pytest.skip("bash not available")
    bin_dir = _watch_board(tmp_path, [{}])
    env = {
        "PATH": f"{bin_dir}:/usr/bin:/bin",
        "AQUA_NETWATCH_ROOT": str(tmp_path / "root"),
        "AQUA_NETWATCH_IFACE": "wlan0",
        "AQUA_NETWATCH_STATE_DIR": str(tmp_path / "state"),
        "W_BASE": str(tmp_path),
        "W_ROOT": str(tmp_path / "root"),
        "CALLS": str(tmp_path / "calls.log"),
    }

    def run(*args: str) -> list[str]:
        return subprocess.run(
            [bash, str(WATCH_SCRIPT), *args], env=env, capture_output=True, text=True, check=True
        ).stdout.splitlines()

    # No marker: the previous boot did not stop this unit, so it was not shut down.
    (boot,) = _kinds(run("--once"), "BOOT")
    assert "previous=previous-boot-ended-WITHOUT-a-clean-stop" in boot
    assert "boots-in-journal=2" in boot
    assert "prev-boot=bbbb2222" in boot  # from journalctl --list-boots
    assert "2026-10-05 02:07:07" in boot  # ...and when that boot's journal stops
    assert "thr=0x0" in boot and "volt=1.3250V" in boot  # this boot's rail, at boot

    # The start cleared it, so a second start says the same thing: a marker is good for
    # exactly one boot.
    (boot,) = _kinds(run("--once"), "BOOT")
    assert "previous=previous-boot-ended-WITHOUT-a-clean-stop" in boot

    # What ExecStop= runs. The marker names this boot, so the next start inside the same
    # boot is a restart of the unit and not a verdict on a board.
    assert any("clean stop recorded" in line for line in run("--mark-clean-stop"))
    assert (tmp_path / "state/last-clean-stop").is_file()
    (boot,) = _kinds(run("--once"), "BOOT")
    assert "previous=unit-restarted-within-this-boot" in boot

    # A different boot id with the marker in place is the real clean-shutdown case.
    run("--mark-clean-stop")
    boot_id = tmp_path / "stages/0/proc/sys/kernel/random/boot_id"
    boot_id.write_text("99999999-8888-7777-6666-555555555555\n")
    (boot,) = _kinds(run("--once"), "BOOT")
    assert "previous=previous-shutdown-was-CLEAN" in boot
    # ...and --boot-report leaves the marker alone, so looking does not consume it.
    run("--mark-clean-stop")
    run("--boot-report")
    assert (tmp_path / "state/last-clean-stop").is_file()


def test_the_boot_report_lists_the_boots_so_a_run_of_short_ones_is_one_block(tmp_path):
    """Twenty boots of about six seconds each, six seconds apart, is the signature the
    owner reconstructed by hand. Printed verbatim with each boot's first and last entry
    side by side, it is one block to look at."""
    bin_dir = _watch_board(tmp_path, [{}])
    lines = _run_watch(tmp_path, bin_dir, samples=1)
    body = [line for line in lines if line.startswith("boot [")]
    assert any("[boots] " in line and "02:07:01" in line for line in body)
    assert any("[verdict] " in line and "power" in line for line in body)
    assert any("[radio] " in line and "module=brcmfmac" in line for line in body)
    assert any("[power] " in line and "latch within THIS boot only" in line for line in body)
    assert len(_kinds(lines, "boot report begin")) == 1


def test_a_link_that_comes_back_on_the_other_access_point_says_so(tmp_path):
    """The payoff line: how long, what went in which order, what came back in which
    order, and whether the board ended up on the same access point it started on."""
    stages = [
        {},
        {
            "wpa": "DISCONNECTED",
            "bssid": "",
            "operstate": "down",
            "carrier": "0",
            "v4": "",
            "gw": "",
        },
        {"bssid": "11:22:33:44:55:66", "freq": "2412"},
    ]
    bin_dir = _watch_board(tmp_path, stages)
    lines = _run_watch(tmp_path, bin_dir, samples=3)
    (recovered,) = _kinds(lines, "RECOVERED")
    assert "lost=assoc,v4,rt" in recovered
    assert "back=assoc,v4,rt" in recovered
    assert "bssid=changed(aa:bb:cc:dd:ee:ff->11:22:33:44:55:66)" in recovered
    assert "fq=2437->2412" in recovered
    assert "after=" in recovered


def test_a_link_that_is_up_and_carrying_nothing_gets_its_own_line(tmp_path):
    """The 2026-09-17 shape: associated, addressed, routed, gateway answering -- and
    not one packet arriving, for hours, because the radio was in power save. All of the
    ladder reads healthy through it, which is exactly why it needs a check of its own."""
    stages = [{"rx": 1000} for _ in range(6)]
    bin_dir = _watch_board(tmp_path, stages)
    lines = _run_watch(
        tmp_path, bin_dir, samples=6, extra_env={"AQUA_NETWATCH_SILENT_SAMPLES": "3"}
    )
    (silent,) = _kinds(lines, "SILENT")  # once per episode, not once per sample
    assert "associated and carrying nothing" in silent
    assert _kinds(lines, "DEGRADED") == []


@pytest.mark.parametrize(
    "stages",
    [
        [{}, {"wpa": "4WAY_HANDSHAKE", "v4": "", "gw": ""}, {}],
        [{}, {"dev": False}, {}],
        [{}, {"nud": "FAILED", "lladdr": False}, {}],
        [{}, {"throttled": "0x50005", "uv": "1"}, {}],
    ],
)
def test_no_state_of_the_board_ever_makes_the_watcher_act(tmp_path, stages):
    """The behavioural twin of test_the_watcher_never_runs_anything_that_acts, which
    reads the source. Every forbidden executable is on the stub PATH as a logger, so a
    run that reached for one shows up in the call log -- checked for a handshake that
    never completes, an interface that disappears and comes back, a gateway that stops
    answering, and a rail that sags."""
    bin_dir = _watch_board(tmp_path, stages)
    _run_watch(tmp_path, bin_dir, samples=len(stages))
    calls = (tmp_path / "calls.log").read_text()
    for forbidden in WATCH_FORBIDDEN:
        assert forbidden not in calls, forbidden
    # nmcli and wpa_cli are on the list of things it may run, but only to read.
    for line in calls.splitlines():
        if line.startswith("nmcli "):
            assert "device show" in line or "connection show --active" in line, line
        if line.startswith("wpa_cli "):
            assert line.endswith((" status", " signal_poll", " scan_results")), line


def test_the_watcher_shares_no_state_with_the_recovery_script():
    """The two must not be able to interfere. The recovery script keeps counters under
    /run/aqua-net-recover; the watcher writes exactly one file, the clean-stop marker,
    under its own StateDirectory, and reads the other's journal only to attribute a
    change to it rather than to a fault."""
    assert _shell_default(NET_SCRIPT, "AQUA_NET_STATE_DIR") == "/run/aqua-net-recover"
    assert _shell_default(WATCH_SCRIPT, "AQUA_NETWATCH_STATE_DIR") == "/var/lib/aqua-net-watch"
    watch_code = _code_lines(WATCH_SCRIPT)
    for line in watch_code:
        # The watcher touches the recovery script's name in exactly one way: it tails
        # its journal, so a change the recovery script caused is attributed to it in the
        # same snapshot rather than looking like a fault of its own.
        if "aqua-net-recover" in line:
            # Either the journal tail, or a continuation of a quoted journal message
            # (the ABSENT line names the recovery script to say why it cannot help).
            assert "journalctl -u aqua-net-recover.service" in line or line.startswith('"'), line
    assert any("journalctl -u aqua-net-recover.service" in line for line in watch_code)
    for line in _code_lines(NET_SCRIPT):
        assert "aqua-net-watch" not in line, line
    # One write, in one function, called from one place: the ExecStop= path.
    assert "\n".join(watch_code).count('> "$BOOT_MARKER"') == 1


def test_no_net_watch_is_an_off_switch_and_not_a_skipped_step():
    """Same argument as --no-net-recover, and more pointed: this one is a long-running
    service, so an earlier run's copy keeps sampling on its old cadence until something
    stops it."""
    text = BOARD_SCRIPT.read_text()
    assert "systemctl disable --now aqua-net-watch.service" in text
    for dst in ('"$NET_WATCH_UNIT_DST"', '"$NET_WATCH_DST"'):
        assert f"remove_path {dst}" in text
    # And a changed script or unit is picked up, which a oneshot timer never needs.
    assert "systemctl restart aqua-net-watch.service" in text
    assert "systemctl enable --now aqua-net-watch.service" in text


#: What wpa_cli actually printed on the board, for days, instead of a status.
WPA_SANDBOX_ERROR = (
    "Failed to connect to non-global ctrl_ifname: wlan0  error: Read-only file system"
)


def test_an_unreachable_supplicant_is_a_degraded_source_not_a_degraded_network(tmp_path):
    """The defect that made the instrument useless as shipped. With wpa_cli unable to
    reach the supplicant from inside the unit's sandbox, every sample read
    `assoc:DOWN` and the first one announced `DEGRADED first=assoc` -- on a board that
    was associated, had its address and route, and was reaching its gateway. An
    instrument that reports a fault continuously cannot show the fault it exists to
    catch: the real event arrives into a log that has been crying wolf for days, and the
    DEGRADED/RECOVERED pair the owner greps for is already spent.

    The rule: a degraded SOURCE is never a degraded NETWORK. When the only evidence
    available is the kernel's own link state plus a BSSID, "connected to this BSSID with
    the carrier up, address and route present, gateway answering" is a working network
    and the unit says so."""
    stages = [{"wpa_err": WPA_SANDBOX_ERROR} for _ in range(4)]
    bin_dir = _watch_board(tmp_path, stages)
    lines = _run_watch(tmp_path, bin_dir, samples=4, extra_env={"AQUA_NETWATCH_IDLE_EVERY": "1"})
    samples = _kinds(lines, "sample")
    assert len(samples) == 4
    for line in samples:
        assert "layers=ok" in line, line
        # The marker stays: a reader can see which source spoke, and that the
        # handshake state is not in this line.
        assert "wpa=ASSOCIATED(iw)" in line, line
    assert _kinds(lines, "DEGRADED") == []
    assert _kinds(lines, "RECOVERED") == []
    # ...and the reason is named once, with what wpa_cli actually said, rather than
    # swallowed. Bisecting the unit's sandbox from the outside is what it cost the
    # first time.
    (notice,) = _kinds(lines, "WPASRC")
    assert "src=iw" in notice
    assert WPA_SANDBOX_ERROR in notice
    assert "CANNOT see a" in notice and "4-way-handshake" in notice
    assert "degraded source, not a degraded network" in notice
    assert "PrivateTmp=no" in notice  # where to look if it persists


def test_the_fallback_still_calls_a_real_loss_of_association_down(tmp_path):
    """The other half of the rule: the association rung is reserved for evidence that
    the board is genuinely not associated, and the fallback can still supply that. `iw`
    answering "Not connected" is positive evidence, not a missing source."""
    stages = [
        {"wpa_err": WPA_SANDBOX_ERROR},
        {"wpa_err": WPA_SANDBOX_ERROR, "bssid": "", "v4": "", "gw": ""},
    ]
    bin_dir = _watch_board(tmp_path, stages)
    lines = _run_watch(tmp_path, bin_dir, samples=2)
    assert "layers=ok" in _kinds(lines, "sample")[0]
    last = _kinds(lines, "sample")[-1]
    assert "wpa=UNASSOCIATED(iw)" in last
    assert "assoc:DOWN" in last
    (degraded,) = _kinds(lines, "DEGRADED")
    assert "first=assoc" in degraded


def test_a_link_with_no_source_at_all_is_judged_by_the_kernel_and_believed(tmp_path):
    """Neither source answering is still not a reason to invent a fault. The kernel's
    carrier is then the only evidence there is, and it is positive evidence."""
    stages = [{"wpa_err": WPA_SANDBOX_ERROR, "iw_silent": True} for _ in range(2)]
    bin_dir = _watch_board(tmp_path, stages)
    lines = _run_watch(tmp_path, bin_dir, samples=2)
    for line in _kinds(lines, "sample"):
        assert "layers=ok" in line, line
        assert "wpa=?" in line and "bss=-" in line, line
    assert _kinds(lines, "DEGRADED") == []
    (notice,) = _kinds(lines, "WPASRC")
    assert "src=none" in notice


def test_a_carrier_that_drops_is_down_whatever_the_source_says(tmp_path):
    """No source may override the kernel: cfg80211 drops the carrier the moment an
    association goes, so a stale BSSID from any source cannot hold the rung up."""
    stages = [{}, {"carrier": "0", "operstate": "down"}]
    bin_dir = _watch_board(tmp_path, stages)
    lines = _run_watch(tmp_path, bin_dir, samples=2)
    assert "assoc:DOWN" in _kinds(lines, "sample")[-1]
    assert "first=assoc" in _kinds(lines, "DEGRADED")[0]


def test_a_handshake_in_progress_is_still_down_when_the_supplicant_can_be_asked(tmp_path):
    """The reason wpa_cli is worth the two sandbox exceptions it costs: it is the only
    source that can see a 4-way handshake that never completes, which is the shape of
    the 2026-09-30 storm. With it reachable, anything short of COMPLETED is a real
    DOWN -- the fallback's laxer rule must not leak into this branch."""
    stages = [{}, {"wpa": "4WAY_HANDSHAKE"}]
    bin_dir = _watch_board(tmp_path, stages)
    lines = _run_watch(tmp_path, bin_dir, samples=2)
    last = _kinds(lines, "sample")[-1]
    assert "wpa=4WAY_HANDSHAKE" in last and "assoc:DOWN" in last
    assert "first=assoc" in _kinds(lines, "DEGRADED")[0]
    assert _kinds(lines, "WPASRC") == []  # the richer source answered, so no notice


def test_the_watcher_reads_the_firmware_mailbox_and_nothing_else_in_dev():
    """The rail is the whole point of the power sampling, and it is read through
    /dev/vcio. A measurement nobody notices is missing is worse than none, so the unit
    ships the setting that was verified to work on the board rather than documenting a
    drop-in for it."""
    values = _unit_values(WATCH_UNIT.read_text())
    assert values["DevicePolicy"] == ["closed"]
    assert any(allowed.startswith("/dev/vcio") for allowed in values["DeviceAllow"])
    script = WATCH_SCRIPT.read_text()
    for reading in ("get_throttled", "measure_volts core", "measure_temp"):
        assert f"vcgencmd {reading}" in script, reading
    # Still optional: a board without the tool keeps sampling, with the die
    # temperature and the undervoltage bit from /sys.
    assert "HAVE_VCGENCMD" in script
    assert _shell_default(WATCH_SCRIPT, "AQUA_NETWATCH_THERMAL_ZONE") == "thermal_zone0"


def test_the_board_script_refuses_an_outage_cadence_faster_than_the_healthy_one():
    """An outage is the long part, and sampling it faster than health would turn the
    one case that lasts for days into the one that fills the journal."""
    text = BOARD_SCRIPT.read_text()
    assert 'if [[ "$NET_WATCH_OUTAGE_INTERVAL" -lt "$NET_WATCH_INTERVAL" ]]; then' in text
    script = WATCH_SCRIPT.read_text()
    assert (
        'if [[ "$AQUA_NETWATCH_OUTAGE_INTERVAL_S" -lt "$AQUA_NETWATCH_INTERVAL_S" ]]; then'
        in script
    )


# --- the radio recovery unit: an interface that does not exist (section 9) -------------

RADIO_SCRIPT = DEPLOY / "aqua-radio-recover.sh"
RADIO_UNIT = DEPLOY / "aqua-radio-recover.service"

#: Executables the radio recovery may never reach for. Stubbed alongside `modprobe`,
#: `reboot` and `sleep` so a run that called one shows up in the call log rather than
#: being ruled out only by reading the source. The network ones are on the list because
#: every state of an interface that EXISTS belongs to aqua-net-recover.sh, and
#: `systemctl` because this unit may never stop, restart or mask the two services that
#: cool the enclosure -- its one escalation is a plain `reboot`, nothing finer-grained.
RADIO_FORBIDDEN = (
    "systemctl",
    "shutdown",
    "poweroff",
    "halt",
    "nmcli",
    "ip",
    "ping",
    "wpa_cli",
    "iwconfig",
    "ifconfig",
    "dhclient",
    "aqua-bridge",
    "aqua-heartbeat",
)

#: The nine knobs the unit carries as Environment= and the installer writes from its own
#: table: script default, unit value and installer default all have to be this.
RADIO_INSTALLED_KNOBS = {
    "AQUA_RADIO_IFACE": ("RADIO_IFACE", "wlan0"),
    "AQUA_RADIO_GRACE_S": ("RADIO_GRACE", "90"),
    "AQUA_RADIO_INTERVAL_S": ("RADIO_INTERVAL", "60"),
    "AQUA_RADIO_ATTEMPTS": ("RADIO_ATTEMPTS", "3"),
    "AQUA_RADIO_ATTEMPT_DELAY_S": ("RADIO_ATTEMPT_DELAY", "30"),
    "AQUA_RADIO_SETTLE_S": ("RADIO_SETTLE", "20"),
    "AQUA_RADIO_REBOOT": ("RADIO_REBOOT", "on"),
    "AQUA_RADIO_REBOOT_BUDGET": ("RADIO_REBOOT_BUDGET", "2"),
    "AQUA_RADIO_REBOOT_WINDOW_S": ("RADIO_REBOOT_WINDOW", "86400"),
}


def _radio_board(
    base: Path,
    *,
    present: bool,
    uptime: float = 600.0,
    cure_at: int | None = None,
    modules: tuple[str, ...] = ("brcmfmac", "brcmfmac_cyw"),
) -> Path:
    """A board whose wireless interface is present or absent, as a fake /sys + /proc.

    ``present=False`` is the whole trigger: no ``/sys/class/net/wlan0`` at all, which is
    what the 2026-10-06 boot looked like from userspace after the radio's firmware
    download failed its read-back verification.

    The step from absent to present is driven by the stub ``modprobe``, not by a clock
    or by the stub ``sleep``: a reload either brings the netdev back or it does not, and
    that is the only thing in this script that can change the board's state.
    ``cure_at=N`` makes the Nth load succeed, ``cure_at=None`` never cures -- the chip
    the kernel has given up on, which is the case the escalation exists for.
    """
    root = base / "root"
    (root / "sys/class/net/lo").mkdir(parents=True, exist_ok=True)
    (root / "sys/class/net/lo/operstate").write_text("unknown\n")
    if present:
        (root / "sys/class/net/wlan0").mkdir(parents=True, exist_ok=True)
    (root / "proc").mkdir(parents=True, exist_ok=True)
    (root / "proc/uptime").write_text(f"{uptime:.2f} {uptime / 2:.2f}\n")
    # Both modules loaded and still no netdev: that IS the firmware-download failure,
    # and the verdict line has to be able to say so.
    (root / "proc/modules").write_text(
        "".join(f"{name} 319488 0 - Live 0x0\n" for name in modules)
        + "cfg80211 806912 1 brcmfmac, Live 0x0\nbrcmutil 16384 1 brcmfmac, Live 0x0\n"
    )

    bin_dir = base / "bin"
    bin_dir.mkdir(parents=True)
    if cure_at is not None:
        (base / "cure_at").write_text(str(cure_at))
    # A load of the FIRST module of the pair counts as one attempt (the script loads
    # them in the reverse of the removal order, so brcmfmac comes first); the netdev
    # appears on the attempt the test asked for, and a removal takes it away again.
    (bin_dir / "modprobe").write_text(
        "#!/bin/sh\n"
        'echo "modprobe $*" >> "$CALLS"\n'
        'if [ "$1" = "-r" ]; then\n'
        '  rm -rf "$W_ROOT/sys/class/net/wlan0"\n'
        "  exit 0\n"
        "fi\n"
        'if [ "$1" = "brcmfmac" ]; then\n'
        '  n=$(cat "$W_BASE/loads" 2>/dev/null || echo 0)\n'
        '  n=$((n + 1)); echo "$n" > "$W_BASE/loads"\n'
        '  if [ -f "$W_BASE/cure_at" ] && [ "$n" -ge "$(cat "$W_BASE/cure_at")" ]; then\n'
        '    mkdir -p "$W_ROOT/sys/class/net/wlan0"\n'
        "  fi\n"
        "fi\n"
        "exit 0\n"
    )
    # Instant, so the tests have no wall-clock dependence; the waits are bounded by an
    # iteration count as well as by the clock precisely so that this works.
    (bin_dir / "sleep").write_text('#!/bin/sh\necho "sleep $*" >> "$CALLS"\n')
    (bin_dir / "reboot").write_text('#!/bin/sh\necho "reboot $*" >> "$CALLS"\nexit 0\n')
    for name in RADIO_FORBIDDEN:
        (bin_dir / name).write_text(f'#!/bin/sh\necho "{name} $*" >> "$CALLS"\nexit 0\n')
    for entry in bin_dir.iterdir():
        entry.chmod(0o755)
    (base / "calls.log").write_text("")
    return bin_dir


def _run_radio(
    base: Path,
    bin_dir: Path,
    checks: int,
    extra_env: dict[str, str] | None = None,
    state_dir: Path | None = None,
) -> list[str]:
    bash = shutil.which("bash")
    if bash is None:  # pragma: no cover - bash is in packages-rpi.txt and in CI
        pytest.skip("bash not available")
    env = {
        "PATH": f"{bin_dir}:/usr/bin:/bin",
        "AQUA_RADIO_ROOT": str(base / "root"),
        "AQUA_RADIO_IFACE": "wlan0",
        "AQUA_RADIO_STATE_DIR": str(state_dir or base / "state"),
        # Six looks for the netdev instead of twenty: the bound under test is the
        # attempt count and the escalation, not how patient one attempt is.
        "AQUA_RADIO_SETTLE_S": "6",
        "W_BASE": str(base),
        "W_ROOT": str(base / "root"),
        "CALLS": str(base / "calls.log"),
        **(extra_env or {}),
    }
    done = subprocess.run(
        [bash, str(RADIO_SCRIPT), "--checks", str(checks)],
        env=env,
        capture_output=True,
        text=True,
        check=True,
    )
    return done.stdout.splitlines()


def test_radio_knobs_are_documented_variables_with_one_default_each():
    """Section 9: no operator tunable is a literal in the body. The interface name, the
    grace, the cadence, the attempt count, both delays, the escalation switch and the
    reboot budget and its window are all knobs at the top with their reasoning."""
    knobs = {
        "AQUA_RADIO_IFACE": "wlan0",
        "AQUA_RADIO_MODULES": "brcmfmac_cyw brcmfmac",
        "AQUA_RADIO_GRACE_S": "90",
        "AQUA_RADIO_INTERVAL_S": "60",
        "AQUA_RADIO_ATTEMPTS": "3",
        "AQUA_RADIO_ATTEMPT_DELAY_S": "30",
        "AQUA_RADIO_SETTLE_S": "20",
        "AQUA_RADIO_SETTLE_POLL_S": "1",
        "AQUA_RADIO_REBOOT": "on",
        "AQUA_RADIO_REBOOT_BUDGET": "2",
        "AQUA_RADIO_REBOOT_WINDOW_S": "86400",
        "AQUA_RADIO_LOUD_EVERY_S": "3600",
        "AQUA_RADIO_STATE_DIR": "/var/lib/aqua-radio-recover",
        "AQUA_RADIO_ROOT": "",
    }
    for name, default in knobs.items():
        assert _shell_default(RADIO_SCRIPT, name) == default, name
    # One interface name across all three units, each file carrying its own copy
    # because a unit started by systemd has no environment to inherit one from.
    assert (
        _shell_default(RADIO_SCRIPT, "AQUA_RADIO_IFACE")
        == _shell_default(NET_SCRIPT, "AQUA_NET_IFACE")
        == _shell_default(WATCH_SCRIPT, "AQUA_NETWATCH_IFACE")
    )
    # The removal order is the module topology and not a preference: brcmfmac_cyw
    # depends on brcmfmac, so a removal has to name both, dependents first -- and the
    # load then goes the other way, which is the order the two-second cure was
    # measured with. brcmutil and cfg80211 stay loaded and are not in the list.
    modules = _shell_default(RADIO_SCRIPT, "AQUA_RADIO_MODULES").split()
    assert modules == ["brcmfmac_cyw", "brcmfmac"]
    assert "brcmutil" not in modules and "cfg80211" not in modules
    # The grace has to be far above the few seconds the driver takes to register the
    # netdev and far below the thirty hours the fault cost unattended.
    grace = int(_shell_default(RADIO_SCRIPT, "AQUA_RADIO_GRACE_S"))
    assert 30 <= grace <= 600
    # One check's worst case -- every attempt failing -- stays inside a couple of
    # minutes, so a reboot is never more than that away from the ladder starting.
    attempts = int(_shell_default(RADIO_SCRIPT, "AQUA_RADIO_ATTEMPTS"))
    settle = int(_shell_default(RADIO_SCRIPT, "AQUA_RADIO_SETTLE_S"))
    delay = int(_shell_default(RADIO_SCRIPT, "AQUA_RADIO_ATTEMPT_DELAY_S"))
    assert attempts * settle + (attempts - 1) * delay <= 180


def test_the_radio_unit_ships_the_knobs_the_board_script_installs():
    """Three files carry these nine numbers -- the script's default, the unit's
    Environment=, and the installer's knob -- so all three have to agree or reading
    deploy/ tells the truth about none of them. The same three-way agreement the
    watcher's two cadences already have."""
    values = _unit_values(RADIO_UNIT.read_text())
    environment = dict(item.split("=", 1) for item in values["Environment"])
    assert set(environment) == set(RADIO_INSTALLED_KNOBS)
    text = BOARD_SCRIPT.read_text()
    for directive, (variable, default) in RADIO_INSTALLED_KNOBS.items():
        assert environment[directive] == default, directive
        assert _shell_default(RADIO_SCRIPT, directive) == default, directive
        assert _shell_default(BOARD_SCRIPT, variable) == default, variable
        assert f"Environment={directive}=${variable}" in text, directive


def test_the_radio_unit_is_wired_to_nothing_that_cools():
    """It loads a kernel module and, bounded, reboots the board. Nothing in it may
    order, delay, start or stop the daemon that drives the fans, and nothing may
    sequence it against the other two network units (section 2)."""
    values = _unit_values(RADIO_UNIT.read_text())
    assert values["Type"] == ["simple"]
    for key in ("Wants", "Requires", "After", "Before", "Conflicts", "PartOf", "BindsTo"):
        for value in values.get(key, []):
            assert "aqua-bridge" not in value and "aqua-heartbeat" not in value
            assert "aqua-net-recover" not in value and "aqua-net-watch" not in value
            assert "network-online" not in value  # it exists for the offline case
    # The one escalation is the script's own, recorded in its own ledger. systemd must
    # not be able to add a second, unrecorded board reset behind its back.
    assert "WatchdogSec" not in values
    assert "OnFailure" not in values
    assert "StartLimitAction" not in values
    assert values["StartLimitIntervalSec"] == ["0"]
    assert values["Restart"] == ["always"]
    for key in ("ExecStart", "ExecStop", "ExecStartPre", "ExecReload"):
        for value in values.get(key, []):
            assert "aqua-radio-recover.sh" in value, value


def test_the_radio_reboot_ledger_is_on_var_and_never_a_tmpfs():
    """The whole anti-loop mechanism. A count of reboots kept on a tmpfs is erased by
    exactly the event it is counting, so the ledger is a StateDirectory= on /var and the
    script's default path is inside it."""
    values = _unit_values(RADIO_UNIT.read_text())
    assert values["StateDirectory"] == ["aqua-radio-recover"]
    assert "RuntimeDirectory" not in values
    state_dir = _shell_default(RADIO_SCRIPT, "AQUA_RADIO_STATE_DIR")
    assert state_dir == "/var/lib/aqua-radio-recover"
    assert state_dir.startswith("/var/")
    assert not state_dir.startswith("/run")
    # And the append is read back before the reboot is ordered, not after: an
    # unrecordable reboot is the one that would repeat on every boot forever.
    script = RADIO_SCRIPT.read_text()
    assert script.index("record_reboot()") < script.index("if ! record_reboot; then")
    assert script.index("if ! record_reboot; then") < script.rindex("\n  reboot\n")


def test_the_radio_unit_may_load_a_module_and_reboot_and_nothing_else():
    """The exception to inertness, enforced by systemd and not only by the script: two
    capabilities and two syscall sets for the two allowed actions, no address family
    that can carry a packet, and a private /dev with no node added back -- so it cannot
    open a hidraw device, the aquaero or the Quadro even by accident."""
    values = _unit_values(RADIO_UNIT.read_text())
    assert set(values["CapabilityBoundingSet"][0].split()) == {"CAP_SYS_MODULE", "CAP_SYS_BOOT"}
    assert values["NoNewPrivileges"] == ["yes"]
    # Said out loud rather than left to a default, because aqua-net-watch.service sets
    # the opposite and a reader comparing the two should see the difference stated.
    assert values["ProtectKernelModules"] == ["no"]
    filters = values["SystemCallFilter"][0].split()
    assert "@module" in filters and "@reboot" in filters and "@system-service" in filters
    families = values["RestrictAddressFamilies"][0].split()
    assert set(families) == {"AF_UNIX", "AF_NETLINK"}
    assert "AF_INET" not in families and "AF_INET6" not in families
    assert values["PrivateDevices"] == ["yes"]
    # Not one physical node, unlike the watcher, which needs /dev/vcio and
    # /dev/vchiq for the rail and had to drop PrivateDevices= to get them.
    assert "DeviceAllow" not in values
    assert values["ProtectSystem"] == ["strict"]
    assert values["ProtectHome"] == ["yes"]
    # ProcSubset=pid would hide /proc/uptime and /proc/modules, the two files it reads.
    assert "ProcSubset" not in values


def test_the_radio_script_runs_nothing_but_modprobe_and_reboot():
    """Read from the source, the twin of the behavioural checks below. `reboot` is the
    one word on no other script's allowed list and on this one's, because the owner
    approved that escalation explicitly; everything else is still forbidden."""
    lines = _code_lines(RADIO_SCRIPT)
    for word in RADIO_FORBIDDEN:
        assert not _runs_command(lines, word), f"aqua-radio-recover.sh runs {word}"
    for line in lines:
        assert "aqua-bridge" not in line and "aqua-heartbeat" not in line, line
        assert "hidraw" not in line, line
    # The two it may run, and the module order it runs them in.
    assert _runs_command(lines, "modprobe")
    assert _runs_command(lines, "reboot")
    code = "\n".join(lines) + "\n"
    assert 'modprobe -r "${MODULES[@]}"' in code
    assert 'modprobe "${MODULES[i]}"' in code
    # Exactly one reboot in command position in the whole script: the escalation.
    assert len([line for line in lines if re.match(r"^reboot$", line)]) == 1


def test_the_two_recovery_units_cannot_act_on_the_same_board_state():
    """The division of labour, stated in both headers and true in both bodies: an
    interface that does not exist is the radio unit's case and only its case, and every
    state of an interface that does exist is aqua-net-recover's. Neither can reach the
    other's."""
    radio = RADIO_SCRIPT.read_text()
    net = NET_SCRIPT.read_text()
    # Both headers name the other and name the one fact that divides them.
    assert "aqua-net-recover" in radio and "aqua-radio-recover" in net
    assert "division of labour" in radio and "division of labour" in net
    # The radio unit's trigger: the netdev's existence, and nothing else.
    assert '[[ -e "$NETCLASS/$IFACE" ]]' in radio
    # aqua-net-recover's two actions both require a device NetworkManager has a state
    # for, and it now hands the no-device case over instead of counting it as its own.
    assert 'if [[ -z "$state_code" ]]; then' in net
    assert "aqua-radio-recover.service owns an absent interface" in net
    # Neither script reaches for the other's one action.
    net_lines = _code_lines(NET_SCRIPT)
    assert not _runs_command(net_lines, "modprobe") and not _runs_command(net_lines, "reboot")
    radio_lines = _code_lines(RADIO_SCRIPT)
    assert not _runs_command(radio_lines, "nmcli") and not _runs_command(radio_lines, "ping")


def test_an_interface_that_exists_is_never_touched(tmp_path):
    """The narrow trigger. While the netdev is there the unit does nothing whatsoever --
    no module, no reboot, not even a journal line -- whatever the link is doing, because
    every state of an interface that exists belongs to aqua-net-recover.sh."""
    bin_dir = _radio_board(tmp_path, present=True)
    lines = _run_radio(tmp_path, bin_dir, checks=5)
    calls = (tmp_path / "calls.log").read_text()
    assert "modprobe" not in calls and "reboot" not in calls
    assert not _kinds(lines, "ABSENT") and not _kinds(lines, "RELOAD")
    assert not _kinds(lines, "HOLD") and not _kinds(lines, "GAVEUP")
    # One "started" line per boot and nothing else: the steady-state journal cost.
    assert [line for line in lines if not line.startswith("sleep ")] == [
        line for line in lines if line.startswith("started:")
    ]


def test_an_absence_inside_the_grace_period_is_a_boot_and_not_a_fault(tmp_path):
    """The driver registers the netdev a few seconds into boot, so an absence before
    AQUA_RADIO_GRACE_S of uptime is a boot in progress. Reloading a driver while the
    boot is still bringing interfaces up is how a working board gets broken."""
    bin_dir = _radio_board(tmp_path, present=False, uptime=12.0)
    lines = _run_radio(tmp_path, bin_dir, checks=3)
    calls = (tmp_path / "calls.log").read_text()
    assert "modprobe" not in calls and "reboot" not in calls
    assert len(_kinds(lines, "HOLD")) == 1  # and rate-limited after that, not per check
    assert "up=12s" in _kinds(lines, "HOLD")[0]
    assert not _kinds(lines, "ABSENT")
    # The grace is read from /proc/uptime, so it is measured from boot and a restart of
    # the unit hours later cannot re-arm it.
    assert "up=" in _kinds(lines, "HOLD")[0]


def test_an_absent_interface_past_the_grace_is_reloaded_and_comes_back(tmp_path):
    """The measured cure: remove both modules (dependents first, because brcmfmac_cyw
    depends on brcmfmac), load them back the other way round, and the netdev returns.
    One attempt, no reboot."""
    bin_dir = _radio_board(tmp_path, present=False, cure_at=1)
    lines = _run_radio(tmp_path, bin_dir, checks=2)
    calls = [line for line in (tmp_path / "calls.log").read_text().splitlines()]
    modprobes = [line for line in calls if line.startswith("modprobe")]
    assert modprobes == [
        "modprobe -r brcmfmac_cyw brcmfmac",
        "modprobe brcmfmac",
        "modprobe brcmfmac_cyw",
    ]
    assert "reboot" not in "\n".join(calls)
    assert len(_kinds(lines, "RELOAD")) == 1
    assert "result=present" in _kinds(lines, "RELOAD")[0]
    (recovered,) = _kinds(lines, "RECOVERED")
    assert "after=1 reload attempt(s)" in recovered
    # The verdict carries what was absent and what did exist.
    (absent,) = _kinds(lines, "ABSENT")
    assert "iface=wlan0" in absent and "netdevs=lo" in absent
    assert "mod=brcmfmac_cyw=loaded,brcmfmac=loaded" in absent
    # And the second check, with the interface back, says and does nothing.
    assert len(_kinds(lines, "ABSENT")) == 1


def test_a_reload_that_does_not_help_is_retried_the_configured_number_of_times(tmp_path):
    """Three attempts means three, with AQUA_RADIO_ATTEMPT_DELAY_S between them and not
    before the first or after the last -- and then the ladder ends."""
    bin_dir = _radio_board(tmp_path, present=False)
    lines = _run_radio(tmp_path, bin_dir, checks=1, extra_env={"AQUA_RADIO_ATTEMPTS": "3"})
    calls = (tmp_path / "calls.log").read_text().splitlines()
    assert calls.count("modprobe -r brcmfmac_cyw brcmfmac") == 3
    assert [line for line in calls if line == "sleep 30"] == ["sleep 30"] * 2
    reloads = _kinds(lines, "RELOAD")
    assert len(reloads) == 3
    assert all("result=still-absent" in line for line in reloads)
    assert "attempt=3/3" in reloads[-1]


def test_a_reload_that_takes_two_attempts_is_not_an_escalation(tmp_path):
    """The mmc1 "Controller never released inhibit bit(s)" line is why the attempt count
    is above one: an SDIO host that needed another go is still a cure, not a reboot."""
    bin_dir = _radio_board(tmp_path, present=False, cure_at=2)
    lines = _run_radio(tmp_path, bin_dir, checks=1)
    assert "reboot" not in (tmp_path / "calls.log").read_text()
    assert len(_kinds(lines, "RELOAD")) == 2
    assert "after=2 reload attempt(s)" in _kinds(lines, "RECOVERED")[0]


def test_the_board_is_rebooted_only_after_the_attempts_are_exhausted(tmp_path):
    """The escalation the owner approved explicitly: with no radio the board is
    unreachable until somebody walks to it. It comes after the whole ladder and after
    the ledger has been written, never before."""
    bin_dir = _radio_board(tmp_path, present=False)
    lines = _run_radio(tmp_path, bin_dir, checks=1)
    calls = (tmp_path / "calls.log").read_text().splitlines()
    assert calls.count("reboot ") == 1
    # Every attempt first, and the reboot last of all.
    assert calls.index("reboot ") == len(calls) - 1
    assert calls.count("modprobe -r brcmfmac_cyw brcmfmac") == 3
    (escalation,) = _kinds(lines, "REBOOT")
    assert "tried=3 reload attempt(s)" in escalation
    assert "reboot 1 of 2 allowed" in escalation
    assert not _kinds(lines, "GAVEUP")
    # Written BEFORE the reboot, on /var: one entry, so the next one knows.
    ledger = (tmp_path / "state" / "reboots").read_text().split()
    assert len(ledger) == 1 and ledger[0].isdigit()


def test_the_reboot_escalation_can_be_switched_off_entirely(tmp_path):
    """A knob, defaulting to on, with a real off switch. With it off the unit still
    reloads the driver and still says hourly that it has given up; it simply never
    reboots."""
    bin_dir = _radio_board(tmp_path, present=False)
    lines = _run_radio(tmp_path, bin_dir, checks=1, extra_env={"AQUA_RADIO_REBOOT": "off"})
    calls = (tmp_path / "calls.log").read_text()
    assert "reboot" not in calls
    assert calls.count("modprobe -r brcmfmac_cyw brcmfmac") == 3  # still tried
    (gave_up,) = _kinds(lines, "GAVEUP")
    assert "switched off" in gave_up and "reboot=off" in gave_up
    assert not (tmp_path / "state" / "reboots").exists()
    # A budget of zero says the same thing from the other direction.
    other = tmp_path / "zero"
    other.mkdir()
    bin_dir = _radio_board(other, present=False)
    lines = _run_radio(other, bin_dir, checks=1, extra_env={"AQUA_RADIO_REBOOT_BUDGET": "0"})
    assert "reboot" not in (other / "calls.log").read_text()
    assert _kinds(lines, "GAVEUP")


def test_a_spent_reboot_budget_refuses_to_reboot_again(tmp_path):
    """A board stuck in a reboot loop never finishes booting, so nobody can log in to
    fix it even standing next to it -- which is worse than a board with no network. The
    budget is a hard stop, and the ledger it counts is on /var precisely so the reboot
    it counts cannot erase it."""
    bin_dir = _radio_board(tmp_path, present=False)
    state = tmp_path / "state"
    state.mkdir()
    now = int(time.time())
    (state / "reboots").write_text(f"{now - 10}\n{now - 20}\n")
    lines = _run_radio(tmp_path, bin_dir, checks=1)
    assert "reboot" not in (tmp_path / "calls.log").read_text()
    (gave_up,) = _kinds(lines, "GAVEUP")
    assert "budget for this window is spent" in gave_up and "budget=2/2" in gave_up
    # It keeps reloading the driver, because that costs two seconds and might work.
    assert (tmp_path / "calls.log").read_text().count("modprobe -r") == 3


def test_a_reboot_budget_window_that_has_passed_allows_one_again(tmp_path):
    """The window slides, so the worst case is the budget per window and not a unit that
    has given up for good after a bad day months ago. Expired entries are pruned, so the
    ledger cannot grow without bound either."""
    bin_dir = _radio_board(tmp_path, present=False)
    state = tmp_path / "state"
    state.mkdir()
    now = int(time.time())
    (state / "reboots").write_text(f"{now - 90_000}\n{now - 100_000}\n")
    lines = _run_radio(tmp_path, bin_dir, checks=1)
    assert (tmp_path / "calls.log").read_text().count("reboot ") == 1
    assert _kinds(lines, "REBOOT")
    assert len((state / "reboots").read_text().split()) == 1  # the two stale ones pruned


def test_a_reboot_that_cannot_be_recorded_is_not_ordered(tmp_path):
    """The guard that matters most. A reboot whose record did not persist -- no state
    directory, a read-only /var, a full card -- is exactly the reboot that would repeat
    on every boot forever, so one that cannot be recorded is refused outright."""
    bin_dir = _radio_board(tmp_path, present=False)
    blocker = tmp_path / "blocker"
    blocker.write_text("not a directory\n")  # mkdir -p of a path under a file cannot work
    lines = _run_radio(tmp_path, bin_dir, checks=1, state_dir=blocker / "state")
    assert "reboot" not in (tmp_path / "calls.log").read_text()
    (gave_up,) = _kinds(lines, "GAVEUP")
    assert "could NOT be recorded" in gave_up


def test_a_board_that_has_given_up_says_so_without_filling_the_journal(tmp_path):
    """Silence is the defect this whole family of scripts exists against -- thirty hours
    with no interface produced three supplicant lines and nothing else -- but one line
    per check for days is its own kind of silence. The notice is rate-limited to
    AQUA_RADIO_LOUD_EVERY_S and the per-attempt lines are not."""
    bin_dir = _radio_board(tmp_path, present=False)
    lines = _run_radio(tmp_path, bin_dir, checks=3, extra_env={"AQUA_RADIO_REBOOT": "off"})
    assert len(_kinds(lines, "GAVEUP")) == 1
    assert len(_kinds(lines, "ABSENT")) == 3  # the verdict itself is per check
    assert len(_kinds(lines, "RELOAD")) == 9


@pytest.mark.parametrize(
    ("present", "uptime", "cure_at", "extra"),
    [
        (True, 600.0, None, {}),  # a healthy board
        (False, 12.0, None, {}),  # absent inside the grace
        (False, 600.0, 1, {}),  # absent, cured by the first reload
        (False, 600.0, None, {"AQUA_RADIO_REBOOT": "off"}),  # absent, never cured
        (False, 600.0, 3, {"AQUA_RADIO_ATTEMPTS": "5"}),  # absent, cured late
    ],
)
def test_no_state_of_the_board_makes_it_touch_a_controller_or_a_service(
    tmp_path, present, uptime, cure_at, extra
):
    """The behavioural twin of the source check. systemctl, the two aqua services, every
    network tool, shutdown, poweroff and halt are all on PATH as logging stubs, so a run
    that reached for one shows up in the call log. The network is outside the cooling
    path and stays there: the only two things this unit may run are modprobe and the
    bounded reboot, which is why `reboot` is not on this list."""
    bin_dir = _radio_board(tmp_path, present=present, uptime=uptime, cure_at=cure_at)
    _run_radio(tmp_path, bin_dir, checks=4, extra_env=extra)
    calls = (tmp_path / "calls.log").read_text()
    for forbidden in RADIO_FORBIDDEN:
        assert forbidden not in calls, forbidden


def test_the_radio_check_flag_reports_without_loading_or_rebooting_anything(tmp_path):
    """--check is the safe thing to run by hand on a live board, in the shape
    install-board-watchdogs.sh --check already has: it says what it sees and what a real
    run would do, and it changes nothing."""
    bash = shutil.which("bash")
    if bash is None:  # pragma: no cover - bash is in packages-rpi.txt and in CI
        pytest.skip("bash not available")
    bin_dir = _radio_board(tmp_path, present=False)
    done = subprocess.run(
        [bash, str(RADIO_SCRIPT), "--check"],
        env={
            "PATH": f"{bin_dir}:/usr/bin:/bin",
            "AQUA_RADIO_ROOT": str(tmp_path / "root"),
            "AQUA_RADIO_STATE_DIR": str(tmp_path / "state"),
            "W_BASE": str(tmp_path),
            "W_ROOT": str(tmp_path / "root"),
            "CALLS": str(tmp_path / "calls.log"),
        },
        capture_output=True,
        text=True,
        check=True,
    )
    assert (tmp_path / "calls.log").read_text() == ""
    assert "absent" in done.stdout and "would reload the driver" in done.stdout
    assert not (tmp_path / "state").exists()


@pytest.mark.parametrize(
    ("name", "value"),
    [
        ("AQUA_RADIO_REBOOT", "maybe"),
        ("AQUA_RADIO_ATTEMPTS", "-1"),
        ("AQUA_RADIO_GRACE_S", "0"),
        ("AQUA_RADIO_REBOOT_WINDOW_S", "soon"),
    ],
)
def test_the_radio_script_refuses_a_nonsense_knob(tmp_path, name, value):
    """A unit that may reboot the board must not start on a typo in a budget."""
    bash = shutil.which("bash")
    if bash is None:  # pragma: no cover - bash is in packages-rpi.txt and in CI
        pytest.skip("bash not available")
    done = subprocess.run(
        [bash, str(RADIO_SCRIPT), "--once"],
        env={"PATH": "/usr/bin:/bin", name: value},
        capture_output=True,
        text=True,
    )
    assert done.returncode == 2
    assert name in done.stderr


def test_the_board_script_refuses_a_nonsense_radio_knob():
    """The same refusals on the installer's side of the three-way agreement."""
    text = BOARD_SCRIPT.read_text()
    assert 'echo "error: $name must be a whole number (0 = none), got' in text
    assert "error: RADIO_REBOOT must be 'on' or 'off'" in text
    for name in ("RADIO_GRACE", "RADIO_INTERVAL", "RADIO_ATTEMPT_DELAY", "RADIO_SETTLE"):
        assert name in text


def test_no_radio_recover_is_an_off_switch_and_not_a_skipped_step():
    """Same argument as --no-net-watch: this one is a long-running service too, so an
    earlier run's copy keeps checking with the attempt count and the reboot budget it
    was installed with until something stops it."""
    text = BOARD_SCRIPT.read_text()
    assert "systemctl disable --now aqua-radio-recover.service" in text
    for dst in ('"$RADIO_UNIT_DST"', '"$RADIO_DST"'):
        assert f"remove_path {dst}" in text
    assert "systemctl enable --now aqua-radio-recover.service" in text
    assert "systemctl restart aqua-radio-recover.service" in text
    # The ledger is evidence, not configuration: removing the unit must not hand the
    # next install a budget that starts again from zero.
    assert "the reboot ledger under /var/lib/aqua-radio-recover is left alone" in text


def test_no_radio_reboot_is_the_escalations_own_off_switch():
    """The escalation knob defaults to on and the flag writes "off" into the installed
    unit -- so it is a change to the unit, which the installer then restarts, and not a
    step that was skipped while the old unit kept its old escalation."""
    text = BOARD_SCRIPT.read_text()
    assert "--no-radio-reboot)" in text
    assert "RADIO_REBOOT=off" in text
    assert "[--no-radio-recover] [--no-radio-reboot]" in text
    assert _shell_default(BOARD_SCRIPT, "RADIO_REBOOT") == "on"
    assert "Environment=AQUA_RADIO_REBOOT=$RADIO_REBOOT" in text
