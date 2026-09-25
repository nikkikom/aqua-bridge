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
# Two of the drop-ins below compete with vendor files Raspberry Pi OS ships in
# the same *.conf.d directories. systemd merges *.conf.d fragments in lexical
# order of filename ACROSS /usr/lib, /run and /etc -- later wins, the
# directory a file lives in does not decide it -- so a drop-in that wants to
# win has to sort after the vendor's, by name, not just live in /etc. The SoC
# watchdog's competes with /usr/lib/systemd/system.conf.d/40-rpi-enable-
# watchdog.conf; journald's competes with /usr/lib/systemd/journald.conf.d/
# 40-rpi-volatile-storage.conf. Both sections below name their drop-in to sort
# after the vendor's and then VERIFY the effective result instead of trusting
# the write -- systemctl show for the watchdog, systemd-analyze cat-config for
# journald -- and say plainly when it does not match what was asked for.
# Writing a file and declaring victory is what let a vendor drop-in win
# silently before this fix (PROJECT.md §9).
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
# That vendor file (RuntimeWatchdogSec=1m, RebootWatchdogSec=2m) is exactly why
# this script's own drop-in (below) has to sort after it by filename: systemd
# merges system.conf.d the same way it merges journald.conf.d (lexical order
# of filename across /usr/lib, /run and /etc, later wins), and this script's
# drop-in used to be named 10-aqua-watchdog.conf, which sorts BEFORE
# 40-rpi-enable-watchdog.conf and lost -- invisible only because 60 s/120 s
# above happen to equal the vendor's 1 m/2 m; an operator who set
# SOC_WATCHDOG_SEC=90 would have gotten a script that reported success and a
# board that stayed at 60 s. Found and fixed the same way as the journald
# drop-in below (PROJECT.md §9).
# Where journald keeps the journal. Raspberry Pi OS ships its own
# /usr/lib/systemd/journald.conf.d/40-rpi-volatile-storage.conf (Storage=
# volatile), so the caps below are meaningless unless something sets Storage=
# explicitly -- "auto" (systemd's compiled-in default when nothing sets it)
# only goes persistent if /var/log/journal already exists, which nothing on a
# stock image creates. Measured on the owner's board (2026-09-25): the vendor
# file beat this script's old drop-in (below) and the journal stayed in
# /run/log/journal, in RAM, through a five-day outage that left nothing to
# read afterward. "persistent" is the only value that makes the rest of this
# block mean anything; "auto"/"volatile" are accepted for a board that
# genuinely wants a RAM-only journal, with every cap below then bounding RAM,
# not the card.
JOURNAL_STORAGE="${JOURNAL_STORAGE:-persistent}"
# Journald caps: with the journal persistent, it now competes for the card.
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
# 99- so this sorts after any conventionally-numbered vendor drop-in in
# systemd's cross-directory merge (later filename wins; a three-digit prefix
# like "100-" still sorts *before* "99-" as a string, since '1' < '9') --
# specifically /usr/lib/systemd/system.conf.d/40-rpi-enable-watchdog.conf,
# which is what beat this file's old name, 10-aqua-watchdog.conf (PROJECT.md
# §9 "Board hardening").
SYSTEM_DROPIN="/etc/systemd/system.conf.d/99-aqua-watchdog.conf"
# This script's own previous name for $SYSTEM_DROPIN; a run that installs the
# new one removes it so a board does not end up with two drop-ins saying the
# same thing (and, before this fix, disagreeing about which one systemd uses).
LEGACY_SYSTEM_DROPINS=(
  "/etc/systemd/system.conf.d/10-aqua-watchdog.conf"
)
# 99- so this sorts after any conventionally-numbered vendor drop-in in
# systemd's cross-directory merge (later filename wins; a three-digit prefix
# like "100-" still sorts *before* "99-" as a string, since '1' < '9') --
# specifically /usr/lib/systemd/journald.conf.d/40-rpi-volatile-storage.conf,
# which is what beat this file's old name, 20-aqua-journal-limits.conf
# (PROJECT.md §9 "Board hardening").
JOURNALD_DROPIN="/etc/systemd/journald.conf.d/99-aqua-journal-limits.conf"
# Hand-made on the board before this script set Storage= itself, plus this
# script's own previous name for $JOURNALD_DROPIN; a run that installs the
# current one removes all three so four drop-ins never end up saying the same
# thing.
LEGACY_JOURNALD_DROPINS=(
  "/etc/systemd/journald.conf.d/10-persistent.conf"
  "/etc/systemd/journald.conf.d/99-aqua-persistent.conf"
  "/etc/systemd/journald.conf.d/20-aqua-journal-limits.conf"
)
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
case "$JOURNAL_STORAGE" in
  persistent | volatile | auto) ;;
  *)
    echo "error: JOURNAL_STORAGE must be 'persistent', 'volatile' or 'auto'," \
      "got '$JOURNAL_STORAGE'" >&2
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

# watchdog_report. Prints what systemd is actually enforcing, not what was
# asked for: RuntimeWatchdogUSec and RebootWatchdogUSec from systemctl show --
# the two questions a writer of $SYSTEM_DROPIN cannot answer by looking at its
# own write, the same principle as journald_storage_report below. Both sides
# are normalized to microseconds with systemd-analyze timespan (LC_ALL=C, so
# the label line reads the ASCII "us:" rather than the locale-dependent "μs:")
# so a systemd-normalized string like "1min" compares equal to this script's
# own "60s", not unequal as literal text. Needs no root; --check calls it
# before anything is written, so it reports the board's *current* state, not a
# preview of this run. This is the check that was missing when the vendor
# drop-in won silently.
watchdog_report() {
  local show runtime_val reboot_val
  show="$(systemctl show -p RuntimeWatchdogUSec -p RebootWatchdogUSec 2> /dev/null || true)"
  runtime_val="$(printf '%s\n' "$show" | sed -n 's/^RuntimeWatchdogUSec=//p')"
  reboot_val="$(printf '%s\n' "$show" | sed -n 's/^RebootWatchdogUSec=//p')"
  echo "  RuntimeWatchdogUSec: ${runtime_val:-unknown}"
  echo "  RebootWatchdogUSec:  ${reboot_val:-unknown}"
  if ! command -v systemd-analyze > /dev/null 2>&1; then
    echo "  (systemd-analyze not found, cannot verify against SOC_WATCHDOG_SEC/REBOOT_WATCHDOG_SEC)"
    return 0
  fi
  local runtime_us reboot_us want_runtime_us want_reboot_us
  runtime_us="$(LC_ALL=C systemd-analyze timespan "${runtime_val:-0}" 2> /dev/null \
    | sed -n 's/^[[:space:]]*us:[[:space:]]*//p')"
  reboot_us="$(LC_ALL=C systemd-analyze timespan "${reboot_val:-0}" 2> /dev/null \
    | sed -n 's/^[[:space:]]*us:[[:space:]]*//p')"
  want_runtime_us="$(LC_ALL=C systemd-analyze timespan "${SOC_WATCHDOG_SEC}s" 2> /dev/null \
    | sed -n 's/^[[:space:]]*us:[[:space:]]*//p')"
  want_reboot_us="$(LC_ALL=C systemd-analyze timespan "${REBOOT_WATCHDOG_SEC}s" 2> /dev/null \
    | sed -n 's/^[[:space:]]*us:[[:space:]]*//p')"
  if [[ -n "$runtime_us" && "$runtime_us" == "$want_runtime_us" ]]; then
    echo "  OK: RuntimeWatchdogSec matches SOC_WATCHDOG_SEC=$SOC_WATCHDOG_SEC"
  else
    echo "  WARNING: asked for SOC_WATCHDOG_SEC=$SOC_WATCHDOG_SEC" \
      "(RuntimeWatchdogSec=${SOC_WATCHDOG_SEC}s), systemd is actually running" \
      "RuntimeWatchdogUSec=${runtime_val:-unknown} -- some drop-in with a" \
      "lexically later filename is winning; 'systemd-analyze cat-config" \
      "systemd/system.conf' shows which one. PROJECT.md §9 'Board hardening'."
  fi
  if [[ -n "$reboot_us" && "$reboot_us" == "$want_reboot_us" ]]; then
    echo "  OK: RebootWatchdogSec matches REBOOT_WATCHDOG_SEC=$REBOOT_WATCHDOG_SEC"
  else
    echo "  WARNING: asked for REBOOT_WATCHDOG_SEC=$REBOOT_WATCHDOG_SEC" \
      "(RebootWatchdogSec=${REBOOT_WATCHDOG_SEC}s), systemd is actually running" \
      "RebootWatchdogUSec=${reboot_val:-unknown} -- some drop-in with a" \
      "lexically later filename is winning; 'systemd-analyze cat-config" \
      "systemd/system.conf' shows which one. PROJECT.md §9 'Board hardening'."
  fi
}

# journald_storage_report. Prints what journald is actually doing, not what was
# asked for: the effective Storage= from the same cross-directory merge systemd
# itself does (systemd-analyze cat-config, so this script carries no second copy
# of that ordering rule to drift from the real one), and whether /var/log/journal
# holds anything -- the two questions a writer of $JOURNALD_DROPIN cannot answer
# by looking at its own write. Needs no root; --check calls it before anything is
# written, so it reports the board's *current* state, not a preview of this run.
# This is the check that was missing when the vendor drop-in won silently.
journald_storage_report() {
  local effective cat_config
  if command -v systemd-analyze > /dev/null 2>&1; then
    cat_config="$(systemd-analyze cat-config systemd/journald.conf 2> /dev/null || true)"
    effective="$(printf '%s\n' "$cat_config" | sed -n 's/^[[:space:]]*Storage[[:space:]]*=[[:space:]]*//p' | tail -n 1)"
    : "${effective:=auto}" # unset anywhere in the merge: systemd's compiled-in default
  else
    effective="unknown (systemd-analyze not found)"
  fi
  local populated=no
  if find /var/log/journal -name '*.journal' -print -quit 2> /dev/null | grep -q .; then
    populated=yes
  fi
  echo "  effective Storage=: $effective"
  echo "  /var/log/journal populated: $populated"
  if [[ "$effective" == "$JOURNAL_STORAGE" ]]; then
    echo "  OK: matches JOURNAL_STORAGE=$JOURNAL_STORAGE"
  else
    echo "  WARNING: asked for JOURNAL_STORAGE=$JOURNAL_STORAGE, journald is" \
      "actually running Storage=$effective -- some drop-in with a lexically" \
      "later filename is winning; 'systemd-analyze cat-config" \
      "systemd/journald.conf' shows which one. PROJECT.md §9 'Board hardening'."
  fi
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
# Raspberry Pi OS already ships RuntimeWatchdogSec/RebootWatchdogSec in
# /usr/lib/systemd/system.conf.d/40-rpi-enable-watchdog.conf, and this file is
# named to sort after it in systemd's cross-directory merge (later filename
# wins, the directory does not decide it) -- otherwise the vendor's values win
# and the ones argued above never take effect (PROJECT.md §9 "Board
# hardening").
install_text "$SYSTEM_DROPIN" 644 <<EOF
# Written by deploy/install-board-watchdogs.sh (PROJECT.md §2, §9).
# Named to sort after /usr/lib/systemd/system.conf.d/40-rpi-enable-watchdog.conf
# in systemd's cross-directory merge; without that, the vendor's file applies
# last and wins.
[Manager]
RuntimeWatchdogSec=${SOC_WATCHDOG_SEC}s
RebootWatchdogSec=${REBOOT_WATCHDOG_SEC}s
EOF
NEEDS_REEXEC="$LAST_WROTE"
# This script's own previous, losing name for this drop-in; clean it up so a
# re-run does not leave two drop-ins saying the same thing.
for legacy in "${LEGACY_SYSTEM_DROPINS[@]}"; do
  if [[ -e "$legacy" ]]; then
    NEEDS_REEXEC=1
  fi
  remove_path "$legacy"
done

echo "== journald limits =="
# Storage= is set here explicitly rather than left to systemd's "auto"
# default, and this file is named to sort after
# /usr/lib/systemd/journald.conf.d/40-rpi-volatile-storage.conf (Storage=
# volatile), which otherwise wins and makes every cap below bound RAM, not the
# card (PROJECT.md §9 "Board hardening").
install_text "$JOURNALD_DROPIN" 644 <<EOF
# Written by deploy/install-board-watchdogs.sh (PROJECT.md §2, §9).
# Storage= explicit: Raspberry Pi OS's own 40-rpi-volatile-storage.conf sets
# Storage=volatile, and without an override here that wins and the caps below
# bound nothing on the SD card. Without them, a persistent journal would grow
# to systemd's default of 10 % of the filesystem.
[Journal]
Storage=${JOURNAL_STORAGE}
SystemMaxUse=${JOURNAL_MAX_USE}
SystemMaxFileSize=${JOURNAL_MAX_FILE_SIZE}
MaxRetentionSec=${JOURNAL_MAX_RETENTION}
SyncIntervalSec=${JOURNAL_SYNC_INTERVAL}
EOF
NEEDS_JOURNALD_RESTART="$LAST_WROTE"
# Hand-made before this script wrote Storage= itself, plus this script's own
# previous name for $JOURNALD_DROPIN; clean them up so a re-run does not leave
# four drop-ins saying the same thing.
for legacy in "${LEGACY_JOURNALD_DROPINS[@]}"; do
  if [[ -e "$legacy" ]]; then
    NEEDS_JOURNALD_RESTART=1
  fi
  remove_path "$legacy"
done

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
  echo "== SoC watchdog (current board state, before any change) =="
  watchdog_report
  echo
  echo "== journald storage (current board state, before any change) =="
  journald_storage_report
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
echo "== SoC watchdog =="
watchdog_report
if [[ "$NEEDS_JOURNALD_RESTART" -eq 1 ]]; then
  as_root systemctl restart systemd-journald
fi
# Idempotent and cheap when the journal is already inside the cap: it only
# removes archived files, and it is what brings an already bloated one down.
as_root journalctl --vacuum-size="$JOURNAL_MAX_USE" > /dev/null
echo "  journal on disk now: $(journalctl --disk-usage)"
echo "== journald storage =="
journald_storage_report
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
