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
        "JOURNAL_MAX_USE": "200M",
        "JOURNAL_MAX_FILE_SIZE": "16M",
        "JOURNAL_MAX_RETENTION": "30day",
        "JOURNAL_SYNC_INTERVAL": "5m",
        "WIFI_POWERSAVE": "off",
        "NET_RECOVER_INTERVAL": "5min",
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
    99-aqua-persistent.conf) must not end up with three drop-ins saying the same
    thing once this script installs its own equivalent."""
    text = BOARD_SCRIPT.read_text()
    assert '"/etc/systemd/journald.conf.d/10-persistent.conf"' in text
    assert '"/etc/systemd/journald.conf.d/99-aqua-persistent.conf"' in text
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
    assert script.count('nmcli -w "$AQUA_NET_NMCLI_WAIT_S" device') == 2
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


def _stub_bin(
    root: Path,
    *,
    connected: bool,
    gateway: str,
    ping_ok: bool,
    connect_rc: int = 0,
    connect_delay_s: float = 0.0,
) -> Path:
    """nmcli / ip / ping stubs under ``root`` that log every call to ``root/calls.log``.

    ``connect_rc``/``connect_delay_s`` are the branches the guards exist for: with the
    router off, `nmcli device connect` does not return 0 in a millisecond -- it fails, or
    it takes long enough that systemd kills the unit part-way through.
    """
    bin_dir = root / "bin"
    bin_dir.mkdir(parents=True)
    state = "100 (connected)" if connected else "30 (disconnected)"
    (bin_dir / "nmcli").write_text(
        "#!/bin/sh\n"
        f'echo "nmcli $*" >> "{root}/calls.log"\n'
        'case "$*" in\n'
        f'  *"device show"*) echo "GENERAL.STATE:{state}" ;;\n'
        f'  *"device connect"*) sleep {connect_delay_s}; exit {connect_rc} ;;\n'
        "esac\n"
        "exit 0\n"
    )
    route = f"default via {gateway} proto dhcp metric 600" if gateway else ""
    (bin_dir / "ip").write_text(f'#!/bin/sh\necho "{route}"\nexit 0\n')
    (bin_dir / "ping").write_text(
        f'#!/bin/sh\necho "ping $*" >> "{root}/calls.log"\nexit {0 if ping_ok else 1}\n'
    )
    for name in ("nmcli", "ip", "ping"):
        (bin_dir / name).chmod(0o755)
    (root / "calls.log").write_text("")
    return bin_dir


def _run_recovery(root: Path, bin_dir: Path, runs: int) -> list[str]:
    bash = shutil.which("bash")
    if bash is None:  # pragma: no cover - bash is in packages-rpi.txt and in CI
        pytest.skip("bash not available")
    env = {
        "PATH": f"{bin_dir}:/usr/bin:/bin",
        "AQUA_NET_STATE_DIR": str(root / "state"),
        "AQUA_NET_PING_DEADLINE_S": "1",
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


def test_no_gateway_and_no_carrier_are_both_no_ops(tmp_path):
    """With the router off long enough the lease is gone, and a disconnected interface
    is NetworkManager's own retry to make: neither is this script's business."""
    no_gateway = _stub_bin(tmp_path / "a", connected=True, gateway="", ping_ok=False)
    _run_recovery(tmp_path / "a", no_gateway, runs=3)
    assert "ping" not in (tmp_path / "a" / "calls.log").read_text()
    disconnected = _stub_bin(tmp_path / "b", connected=False, gateway="192.0.2.1", ping_ok=False)
    _run_recovery(tmp_path / "b", disconnected, runs=3)
    calls = (tmp_path / "b" / "calls.log").read_text()
    assert "ping" not in calls and "device disconnect" not in calls
