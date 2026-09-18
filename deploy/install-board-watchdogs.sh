#!/usr/bin/env bash
# Idempotent board hardening for a Raspberry Pi running aqua-bridge: the SoC
# hardware watchdog, journald limits, Wi-Fi power save off, and the Wi-Fi
# re-association timer (PROJECT.md §2 "Watchdog layering", §9).
#
# It does NOT touch aqua-bridge.service, the controllers, or the daemon's
# config: the service watchdog lives in deploy/aqua-bridge.service and is
# installed by deploy/install-pi.sh. Run this once on a freshly written card,
# and again after changing any variable below; a re-run changes nothing that
# already matches.
#
# Usage:
#   sudo deploy/install-board-watchdogs.sh            # apply
#   deploy/install-board-watchdogs.sh --check         # report only, change nothing
#   sudo deploy/install-board-watchdogs.sh --no-net-recover
#
# --check needs no root and writes nothing; it prints what would change.
# --no-net-recover installs no Wi-Fi script, unit or timer, and *removes* the
# ones a previous run installed: it stops and disables aqua-net-recover.timer
# and deletes the three files, so the flag is a real off switch and not just a
# skipped step (a timer left enabled would keep running an old copy of the
# script with whatever thresholds it was installed with). The watchdog and
# journald settings are installed either way.
#
# ---------------------------------------------------------------------------
# Knobs. Every one can be overridden from the environment, e.g.
#   sudo SOC_WATCHDOG_SEC=90 deploy/install-board-watchdogs.sh
# ---------------------------------------------------------------------------
#
# The SoC (BCM2835) watchdog systemd pings, in seconds: the board resets if
# systemd stops pinging for this long. It must sit ABOVE the service watchdog
# (WatchdogSec= in deploy/aqua-bridge.service), because restarting one daemon is
# the cheaper recovery and has to get its chance first; and far above a tick
# (measured on the owner's Zero 2 W: step() p99 82 ms, and a worst-case tick
# bound of 19 s for config.example-das.yaml). 60 s is one service-watchdog
# period plus margin, and systemd then pings every 30 s. It protects against a
# kernel or systemd hang; nothing a userspace process does can reach it. It does
# NOT protect the fans: a reset costs at least the 28 s this board takes from
# power to its first controller write, longer than the aquaero's own 30 s
# software-sensor timeout, so every reset cashes out as the aquaero alarm --
# every fan at 100 %. Loud, never under-cooled. Raising this only leaves a hung
# board hung for longer.
SOC_WATCHDOG_SEC="${SOC_WATCHDOG_SEC:-60}"
# The same watchdog during a reboot or shutdown, in seconds: if shutdown itself
# hangs this long the board resets instead of sitting there. 120 s is well above
# a healthy shutdown here and matches what Raspberry Pi OS ships in
# /usr/lib/systemd/system.conf.d/40-rpi-enable-watchdog.conf.
REBOOT_WATCHDOG_SEC="${REBOOT_WATCHDOG_SEC:-120}"
# Journald caps. The journal is persistent so a network outage or a watchdog
# reset can be read afterwards -- which means it now competes for the card.
# Left alone, SystemMaxUse defaults to 10 % of the filesystem (about 1.5 GB of
# the owner's 15 GB card) and SystemMaxFileSize to an eighth of that. 200M is a
# fiftieth of the free space and far more than this daemon's steady logging
# needs; 16M files keep one rotation cheap on an SD card.
JOURNAL_MAX_USE="${JOURNAL_MAX_USE:-200M}"
JOURNAL_MAX_FILE_SIZE="${JOURNAL_MAX_FILE_SIZE:-16M}"
# Discard entries older than this even when the size cap is not reached: a
# journal that only rotates by size keeps years of nothing on an idle board.
JOURNAL_MAX_RETENTION="${JOURNAL_MAX_RETENTION:-30day}"
# How often journald fsyncs to the card. systemd's own default is 5m; it is
# named here because it is the knob that decides card wear, and lowering it
# buys freshness with writes.
JOURNAL_SYNC_INTERVAL="${JOURNAL_SYNC_INTERVAL:-5m}"
# Wi-Fi power save: "off" writes the NetworkManager drop-in that disables it for
# every wireless profile (the 2026-09-17 outage: BCM43430 associated, power save
# on, unreachable for hours). "keep" installs no drop-in and removes none.
WIFI_POWERSAVE="${WIFI_POWERSAVE:-off}"
# How often aqua-net-recover.timer runs one check (a systemd time span). The
# script re-associates only after two failed checks, so this is half the
# reaction time.
NET_RECOVER_INTERVAL="${NET_RECOVER_INTERVAL:-5min}"

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
UNIT_SRC="$SCRIPT_DIR/aqua-bridge.service"
SYSTEM_DROPIN="/etc/systemd/system.conf.d/10-aqua-watchdog.conf"
JOURNALD_DROPIN="/etc/systemd/journald.conf.d/20-aqua-journal-limits.conf"
NM_DROPIN="/etc/NetworkManager/conf.d/10-aqua-wifi-powersave.conf"
NET_RECOVER_SRC="$SCRIPT_DIR/aqua-net-recover.sh"
NET_RECOVER_DST="/usr/local/lib/aqua-bridge/aqua-net-recover.sh"
NET_UNIT_SRC="$SCRIPT_DIR/aqua-net-recover.service"
NET_UNIT_DST="/etc/systemd/system/aqua-net-recover.service"
NET_TIMER_SRC="$SCRIPT_DIR/aqua-net-recover.timer"
NET_TIMER_DST="/etc/systemd/system/aqua-net-recover.timer"

CHECK_ONLY=0
NET_RECOVER=1

usage() {
  echo "Usage: $0 [--check] [--no-net-recover]" >&2
  exit 2
}

for arg in "$@"; do
  case "$arg" in
    --check)
      CHECK_ONLY=1
      ;;
    --no-net-recover)
      NET_RECOVER=0
      ;;
    *)
      usage
      ;;
  esac
done

for name in SOC_WATCHDOG_SEC REBOOT_WATCHDOG_SEC; do
  if [[ ! "${!name}" =~ ^[1-9][0-9]*$ ]]; then
    echo "error: $name must be a whole number of seconds, got '${!name}'" >&2
    exit 2
  fi
done
if [[ "$REBOOT_WATCHDOG_SEC" -lt "$SOC_WATCHDOG_SEC" ]]; then
  echo "error: REBOOT_WATCHDOG_SEC ($REBOOT_WATCHDOG_SEC) below SOC_WATCHDOG_SEC" \
    "($SOC_WATCHDOG_SEC): a shutdown would be reset sooner than a hang" >&2
  exit 2
fi
case "$WIFI_POWERSAVE" in
  off | keep) ;;
  *)
    echo "error: WIFI_POWERSAVE must be 'off' or 'keep', got '$WIFI_POWERSAVE'" >&2
    exit 2
    ;;
esac

CHANGED=0
#: set by install_text to 1 when that one file differed, 0 when it matched.
LAST_WROTE=0

as_root() {
  if [[ "$(id -u)" -eq 0 ]]; then
    "$@"
  else
    sudo "$@"
  fi
}

# remove_path <destination>. The counterpart of install_text for --no-net-recover:
# silent when there is nothing there, so a re-run changes nothing.
remove_path() {
  local path="$1"
  if [[ ! -e "$path" ]]; then
    return 0
  fi
  CHANGED=1
  if [[ "$CHECK_ONLY" -eq 1 ]]; then
    echo "  would remove: $path"
    return 0
  fi
  as_root rm -f "$path"
  echo "  removed: $path"
}

# install_text <destination> <mode>, content on stdin. Writes only when the
# content differs, so a re-run is silent and leaves the mtime alone. Never call
# it on the right-hand side of a pipe: it sets shell variables.
install_text() {
  local dst="$1" mode="$2" tmp
  LAST_WROTE=0
  tmp="$(mktemp)"
  cat > "$tmp"
  if [[ -f "$dst" ]] && cmp -s "$tmp" "$dst"; then
    echo "  unchanged: $dst"
    rm -f "$tmp"
    return 0
  fi
  LAST_WROTE=1
  CHANGED=1
  if [[ "$CHECK_ONLY" -eq 1 ]]; then
    echo "  would write: $dst"
    if [[ -f "$dst" ]]; then
      diff -u "$dst" "$tmp" || true
    fi
    rm -f "$tmp"
    return 0
  fi
  as_root install -d -m 755 "$(dirname "$dst")"
  as_root install -m "$mode" "$tmp" "$dst"
  rm -f "$tmp"
  echo "  wrote: $dst"
}

echo "== hardware watchdog =="
if [[ ! -e /dev/watchdog ]]; then
  echo "error: /dev/watchdog is absent. On Raspberry Pi OS the bcm2835_wdt" \
    "driver is in the base device tree and needs no overlay, so a board" \
    "without it is either not a Pi or runs a cut-down kernel. Nothing" \
    "installed." >&2
  exit 1
fi
if [[ -r /sys/class/watchdog/watchdog0/identity ]]; then
  echo "  device: $(cat /sys/class/watchdog/watchdog0/identity)" \
    "(current timeout $(cat /sys/class/watchdog/watchdog0/timeout) s)"
fi
# Raspberry Pi OS already ships RuntimeWatchdogSec in
# /usr/lib/systemd/system.conf.d/40-rpi-enable-watchdog.conf. This drop-in is in
# /etc, which wins, so the value the board runs is the one argued above and does
# not move when raspberrypi-sys-mods is upgraded.
install_text "$SYSTEM_DROPIN" 644 <<EOF
# Written by deploy/install-board-watchdogs.sh (PROJECT.md §2, §9).
# Overrides /usr/lib/systemd/system.conf.d/40-rpi-enable-watchdog.conf.
[Manager]
RuntimeWatchdogSec=${SOC_WATCHDOG_SEC}s
RebootWatchdogSec=${REBOOT_WATCHDOG_SEC}s
EOF
NEEDS_REEXEC="$LAST_WROTE"

echo "== journald limits =="
install_text "$JOURNALD_DROPIN" 644 <<EOF
# Written by deploy/install-board-watchdogs.sh (PROJECT.md §2, §9).
# The journal is persistent on this board; without these caps it would grow to
# systemd's default of 10 % of the filesystem.
[Journal]
SystemMaxUse=${JOURNAL_MAX_USE}
SystemMaxFileSize=${JOURNAL_MAX_FILE_SIZE}
MaxRetentionSec=${JOURNAL_MAX_RETENTION}
SyncIntervalSec=${JOURNAL_SYNC_INTERVAL}
EOF
NEEDS_JOURNALD_RESTART="$LAST_WROTE"

NEEDS_NM_RELOAD=0
echo "== Wi-Fi power save =="
if [[ "$WIFI_POWERSAVE" == "off" ]]; then
  # wifi.powersave=2 is NetworkManager's "disable" (0 default, 1 ignore,
  # 2 disable, 3 enable). A drop-in rather than an edit of the connection
  # profile: the profile carries the SSID, which stays off this public
  # repository, and a profile written later by the imager gets this too.
  install_text "$NM_DROPIN" 644 <<'EOF'
# Written by deploy/install-board-watchdogs.sh (PROJECT.md §2).
# 2026-09-17: the BCM43430 with power save on stayed associated and answered
# nothing for hours ("brcmf_cfg80211_set_power_mgmt: power save enabled").
# The daemon kept cooling throughout; this only makes the board reachable.
[connection]
wifi.powersave = 2
EOF
  NEEDS_NM_RELOAD="$LAST_WROTE"
else
  echo "  WIFI_POWERSAVE=keep: leaving $NM_DROPIN alone"
fi

if [[ "$NET_RECOVER" -eq 1 ]]; then
  echo "== Wi-Fi re-association timer =="
  install_text "$NET_RECOVER_DST" 755 < "$NET_RECOVER_SRC"
  install_text "$NET_UNIT_DST" 644 < "$NET_UNIT_SRC"
  timer_tmp="$(mktemp)"
  sed -e "s|^OnBootSec=.*|OnBootSec=$NET_RECOVER_INTERVAL|" \
    -e "s|^OnUnitActiveSec=.*|OnUnitActiveSec=$NET_RECOVER_INTERVAL|" \
    "$NET_TIMER_SRC" > "$timer_tmp"
  install_text "$NET_TIMER_DST" 644 < "$timer_tmp"
  rm -f "$timer_tmp"
else
  echo "== Wi-Fi re-association timer: off (--no-net-recover) =="
  # Not just "skip": an earlier run may have enabled the timer, and a timer left
  # enabled keeps running the copy of the script installed back then, with the
  # thresholds it had back then. The flag has to be able to turn it off again.
  if [[ -e "$NET_TIMER_DST" ]]; then
    CHANGED=1
    if [[ "$CHECK_ONLY" -eq 1 ]]; then
      echo "  would stop and disable aqua-net-recover.timer"
    else
      as_root systemctl disable --now aqua-net-recover.timer || true
      echo "  stopped and disabled aqua-net-recover.timer"
    fi
  fi
  remove_path "$NET_TIMER_DST"
  remove_path "$NET_UNIT_DST"
  remove_path "$NET_RECOVER_DST"
  echo "  (nothing here touches the fans either way: the network is outside the" \
    "cooling path, PROJECT.md §2)"
fi

if [[ "$CHECK_ONLY" -eq 1 ]]; then
  echo
  if [[ "$CHANGED" -eq 1 ]]; then
    echo "== check: changes pending, nothing was written =="
  else
    echo "== check: the board already matches this script =="
  fi
  exit 0
fi

echo "== apply =="
as_root systemctl daemon-reload
if [[ "$NEEDS_REEXEC" -eq 1 ]]; then
  # system.conf is only re-read when PID 1 re-executes. This restarts no
  # service and does not interrupt the control loop.
  as_root systemctl daemon-reexec
fi
echo "  RuntimeWatchdogUSec now: $(systemctl show -p RuntimeWatchdogUSec --value)"
echo "  RebootWatchdogUSec now:  $(systemctl show -p RebootWatchdogUSec --value)"
if [[ "$NEEDS_JOURNALD_RESTART" -eq 1 ]]; then
  as_root systemctl restart systemd-journald
fi
# Idempotent and cheap when the journal is already inside the cap: it only
# removes archived files, and it is what brings an already bloated one down.
as_root journalctl --vacuum-size="$JOURNAL_MAX_USE" > /dev/null
echo "  journal on disk now: $(journalctl --disk-usage)"
if [[ "$NEEDS_NM_RELOAD" -eq 1 ]] && systemctl is-active --quiet NetworkManager; then
  # Reload, never restart: a restart drops the active connection, and this
  # script has to be safe to run over ssh. The setting takes effect on the next
  # activation of the interface (a reconnect or a reboot).
  as_root systemctl reload NetworkManager
  echo "  NetworkManager reloaded; power save is off from the next association on"
fi
if [[ "$NET_RECOVER" -eq 1 ]]; then
  as_root systemctl enable --now aqua-net-recover.timer
  echo "  aqua-net-recover.timer: $(systemctl is-active aqua-net-recover.timer)"
fi

service_watchdog="?"
if [[ -r "$UNIT_SRC" ]]; then
  service_watchdog="$(sed -n 's/^WatchdogSec=//p' "$UNIT_SRC" | head -n 1)"
fi
cat <<EOF

== done ==
  Layer         Catches                                Value
  SoC           a kernel or systemd hang               ${SOC_WATCHDOG_SEC} s -> board reset
  service       a tick that stops pinging systemd      ${service_watchdog:-?} s -> daemon restart
  controller    nothing writing the heartbeat at all   the aquaero's own software-sensor
                                                       timeout -> every fan 100 %

The last line lives on the aquaero and is independent of this board; its value
is set on the device (30 s on the owner's controller), not here. Both watchdogs
above it fire LATER than that 30 s, and deliberately so (the service watchdog
has to stay above the daemon's own worst-case tick): while the daemon writes
the heartbeat itself, a kill by either of them has already cost the alarm --
every fan at 100 % -- before the recovery starts. Loud, never under-cooled.
PROJECT.md §2 has the timeline.

Nothing above reacts to the network. aqua-net-recover only re-associates the
Wi-Fi interface: it never reboots, never restarts aqua-bridge and never touches
a controller (PROJECT.md §2).
EOF
if [[ "$NET_RECOVER" -eq 1 ]]; then
  echo "Try it by hand with"
  echo "  ${NET_RECOVER_DST} --dry-run"
else
  echo "It is not installed (--no-net-recover)."
fi
