#!/usr/bin/env bash
# Idempotent board hardening for a Raspberry Pi running aqua-bridge: the SoC
# hardware watchdog, journald limits, Wi-Fi power save off, unlimited
# NetworkManager autoconnect retries, the Wi-Fi re-association timer, the radio
# recovery unit for an interface that does not exist at all, and the network
# diagnostic watcher (PROJECT.md §2 "Watchdog layering", §9).
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
#   sudo deploy/install-board-watchdogs.sh --no-net-watch
#   sudo deploy/install-board-watchdogs.sh --no-radio-recover
#   sudo deploy/install-board-watchdogs.sh --no-radio-reboot
#
# --check needs no root and writes nothing; it prints what would change.
# --no-net-recover installs no Wi-Fi script, unit or timer, and *removes* the
# ones a previous run installed: it stops and disables aqua-net-recover.timer
# and deletes the three files, so the flag is a real off switch and not just a
# skipped step (a timer left enabled would keep running an old copy of the
# script with whatever thresholds it was installed with). The watchdog and
# journald settings are installed either way.
# --no-net-watch does the same for the diagnostic watcher, and for the same
# reason: a long-running service left enabled keeps running the copy of the
# script installed back then.
# --no-radio-recover does the same for the radio recovery unit.
# --no-radio-reboot keeps that unit but switches off its one escalation, by
# writing Environment=AQUA_RADIO_REBOOT=off into the installed unit instead of
# the default "on" -- also a real off switch and not a skipped step: the unit
# file changes, so a re-run with the flag rewrites it and restarts the service,
# and a re-run without it puts the escalation back.
#
# The three network units do different jobs and none needs the others; their
# triggers cannot overlap, so they cannot fight.
# aqua-net-recover acts on an interface that EXISTS: it re-associates a dead
# link and re-activates a profile NetworkManager has given up on. It never loads
# a module and never reboots.
# aqua-radio-recover acts only when the interface does NOT exist -- the
# 2026-10-06 fault, where the brcmfmac firmware download failed its read-back
# verification in early boot and no netdev was ever registered, so there is
# nothing to associate, activate or probe. It reloads the Wi-Fi driver, which
# cured it in two measured seconds, and after a bounded number of failed
# reloads it reboots the board within a budget it records on /var. It runs no
# nmcli, sends no packet and touches no service.
# aqua-net-watch only ever observes: it samples the interface, the association,
# the address, the route, the gateway's neighbour entry and the SoC's own power
# and temperature figures, and records whether the previous boot ended cleanly.
# It never sends a packet, never loads a module and never touches a service.
#
# Three settings here are verified rather than assumed, because all three have
# already been written successfully and had no effect. Two are drop-ins that
# compete with vendor files (next paragraph). The third is
# connection.autoconnect-retries on the Wi-Fi profile: left at NetworkManager's
# default of 4, four consecutive association failures block autoconnect for
# that profile until something resets it -- which cost the owner's board four
# days off the network on 2026-09-30 while it kept cooling perfectly throughout
# (PROJECT.md §9 "Board hardening"). That default is silent, so this script
# reads the effective value back from nmcli and says plainly when it is not
# what was asked for.
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
# a 15 GB card) and SystemMaxFileSize to an eighth of that; 16M files keep one
# rotation cheap on an SD card.
#
# The size cap was 200M until 2026-10-06, when it turned out to be the reason
# an outage could not be explained. A boot that ends without a clean stop
# leaves its journal unrotated, and journald allocates one 8M file per boot, so
# a board that resets repeatedly spends the cap on near-empty files: twenty such
# boots left 160M of them, and the eviction that made room threw away every
# record from before the episode -- including the boots that would have shown
# how it started. The cap has to be large enough that a reset loop cannot
# outbid the history it is evidence for. 1G is a fifteenth of that card and
# still bounded; the retention cap below, not this one, is what keeps an idle
# board from hoarding.
JOURNAL_MAX_USE="${JOURNAL_MAX_USE:-1G}"
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
# The interface whose NetworkManager profile gets NET_AUTOCONNECT_RETRIES below:
# wlan0 on a Raspberry Pi Zero 2 W. aqua-net-recover.sh carries its own
# AQUA_NET_IFACE with the same default rather than inheriting this one, because
# that script runs from a systemd timer with no environment and has to stand
# alone; set both if the board's Wi-Fi interface is named something else.
NET_IFACE="${NET_IFACE:-wlan0}"
# The NetworkManager profile(s) to set connection.autoconnect-retries on. Empty
# means discover them (nm_connections below), and empty is the default on
# purpose: a Wi-Fi profile is normally named after the SSID, and no SSID may be
# written down in this public repository. Set it to one profile name on a board
# where the discovery would pick the wrong one of several.
NET_CONNECTION="${NET_CONNECTION:-}"
# connection.autoconnect-retries on that profile. 0 means "retry forever", which
# is the whole point of setting it: NetworkManager's own default is 4, and four
# consecutive failures block autoconnect for the profile until a manual
# activation, a NetworkManager restart or a reboot resets it. On 2026-09-30 a
# disconnect storm on the owner's board spent those four attempts in about three
# minutes, and the board then sat with the radio idle for four days --
# wpa_supplicant logged nothing at all, not one scan, not one association
# attempt -- until it was power-cycled. The daemon kept cooling the whole time;
# only the link was gone. -1 is accepted and means "leave it to NetworkManager's
# global default", i.e. opt out of this fix deliberately.
NET_AUTOCONNECT_RETRIES="${NET_AUTOCONNECT_RETRIES:-0}"
# How often aqua-net-recover.timer runs one check (a systemd time span). It is
# the detection latency for both of that script's cases, and with the
# disconnected case now acting (it used to stand down), this interval is the
# worst case for how long a board whose profile NetworkManager gave up on stays
# unreachable: one interval plus one association, about six minutes, against the
# four days it cost before. 5 min is kept rather than shortened, for three
# reasons. The up-but-dead case deliberately needs two failed checks (ten
# minutes) before it bounces a link, and shortening the interval shortens that
# patience too -- which is where the risk of bouncing a live link lives. One run
# is bounded at aqua-net-recover.service's TimeoutStartSec=90, a fifth of this
# interval, so a slow nmcli can never leave runs queued behind each other. And
# retry pressure is no longer this knob's business: AQUA_NET_RECONNECT_MIN_S in
# the script throttles activations on its own, so a faster timer here buys
# detection latency and cannot turn into hammering an absent access point.
# Lowering it to 1min is safe and costs five times the wakeups on a board whose
# single core belongs to the control loop; the four days were not a cadence
# problem anyway -- the timer ran about 1150 times and did nothing.
NET_RECOVER_INTERVAL="${NET_RECOVER_INTERVAL:-5min}"
# Seconds between two samples of deploy/aqua-net-watch.sh while the link is
# healthy, written into aqua-net-watch.service as
# Environment=AQUA_NETWATCH_INTERVAL_S. 15 s is the resolution of the one answer
# that watcher exists to give: which of the interface, the association, the IPv4
# address, the default route and the gateway stops working FIRST, since four of
# those five failures look identical from outside the board and want four
# different fixes. It is also how stale the newest sample line may be when the
# board dies without warning -- and after 2026-10-06 that line, with the rail's
# last known voltage and throttle flags on it, is the evidence. It must stay well
# below the time each layer takes to notice the one beneath it (a DHCP client
# gives up in seconds, ARP in tens of seconds) or two layers failing in sequence
# collapse into one sample and the ordering, which is the whole point, is lost.
# The script's own default is the same 15, and tests/test_deploy.py checks that
# this, the unit and the script agree.
NET_WATCH_INTERVAL="${NET_WATCH_INTERVAL:-15}"
# The same while any layer is bad (Environment=AQUA_NETWATCH_OUTAGE_INTERVAL_S).
# Slower on purpose and never faster than the above: by then the transitions have
# already been snapshotted, and what the next hours -- or, on 2026-10-06, thirty
# of them -- need is the evolution on record without filling a capped journal.
NET_WATCH_OUTAGE_INTERVAL="${NET_WATCH_OUTAGE_INTERVAL:-30}"
# --- the radio recovery unit (deploy/aqua-radio-recover.{sh,service}) ---------
# Every one of these nine is written into aqua-radio-recover.service as an
# Environment= line; the script carries the same default for each, and
# tests/test_deploy.py checks that all three files agree. The script's header
# argues each of them in full -- this table is where the board's copy comes from.
#
# The interface whose ABSENCE that unit acts on. Same default as NET_IFACE above
# and as the two scripts' own, and each file carries its own copy because a unit
# started by systemd has no environment to inherit one from.
RADIO_IFACE="${RADIO_IFACE:-wlan0}"
# Seconds of uptime before a missing interface counts as a fault rather than as a
# boot in progress (Environment=AQUA_RADIO_GRACE_S). The driver registers the
# netdev a few seconds in, so 90 s is thirty times that and the unit cannot act
# while the boot is still bringing interfaces up.
RADIO_GRACE="${RADIO_GRACE:-90}"
# Seconds between two checks (Environment=AQUA_RADIO_INTERVAL_S). A check on a
# healthy board is one [[ -e ]] and one read of /proc/uptime with no fork, so this
# is detection latency and nothing else: one minute against the thirty hours the
# fault cost when nothing acted.
RADIO_INTERVAL="${RADIO_INTERVAL:-60}"
# Driver reloads before the escalation (Environment=AQUA_RADIO_ATTEMPTS). The
# measured cure worked on the first attempt in two seconds; 3 covers an SDIO host
# that needed another go without turning a dead chip into an endless rmmod cycle.
RADIO_ATTEMPTS="${RADIO_ATTEMPTS:-3}"
# Seconds between two reload attempts (Environment=AQUA_RADIO_ATTEMPT_DELAY_S).
RADIO_ATTEMPT_DELAY="${RADIO_ATTEMPT_DELAY:-30}"
# Seconds a reload is given for the netdev to come back before that attempt counts
# as failed (Environment=AQUA_RADIO_SETTLE_S). Measured: one second.
RADIO_SETTLE="${RADIO_SETTLE:-20}"
# Whether that unit's escalation to a reboot is enabled at all, "on" or "off"
# (Environment=AQUA_RADIO_REBOOT). On by default, because the alternative the
# owner lived through is a board that cools perfectly and cannot be reached for
# thirty hours; --no-radio-reboot sets it to off. With it off the unit still
# reloads the driver and still says hourly that it has given up -- it simply never
# reboots.
RADIO_REBOOT="${RADIO_REBOOT:-on}"
# Reboots that unit may order within RADIO_REBOOT_WINDOW
# (Environment=AQUA_RADIO_REBOOT_BUDGET). 2: the first reboot is the one with a
# real chance (a cold start of the SDIO host is a different draw of the same dice
# as a modprobe), the second is the benefit of the doubt, and a third in one day
# would be a board that reboots itself all day and still has no radio -- which is
# worse than the fault, because a board in a reboot loop never finishes booting
# and cannot be fixed even from the console. 0 means never reboot, the same as
# RADIO_REBOOT=off.
RADIO_REBOOT_BUDGET="${RADIO_REBOOT_BUDGET:-2}"
# The window that budget is counted in, in seconds
# (Environment=AQUA_RADIO_REBOOT_WINDOW_S). 86400 -- one day -- so the worst case
# the unit can produce is two reboots a day of an otherwise healthy board, each
# about half a minute, with an hourly line in between saying it has given up. The
# ledger on /var holds timestamps, so the window slides rather than resetting.
RADIO_REBOOT_WINDOW="${RADIO_REBOOT_WINDOW:-86400}"

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
NET_WATCH_SRC="$SCRIPT_DIR/aqua-net-watch.sh"
NET_WATCH_DST="/usr/local/lib/aqua-bridge/aqua-net-watch.sh"
NET_WATCH_UNIT_SRC="$SCRIPT_DIR/aqua-net-watch.service"
NET_WATCH_UNIT_DST="/etc/systemd/system/aqua-net-watch.service"
RADIO_SRC="$SCRIPT_DIR/aqua-radio-recover.sh"
RADIO_DST="/usr/local/lib/aqua-bridge/aqua-radio-recover.sh"
RADIO_UNIT_SRC="$SCRIPT_DIR/aqua-radio-recover.service"
RADIO_UNIT_DST="/etc/systemd/system/aqua-radio-recover.service"

CHECK_ONLY=0
NET_RECOVER=1
NET_WATCH=1
RADIO_RECOVER=1

usage() {
  echo "Usage: $0 [--check] [--no-net-recover] [--no-net-watch]" \
    "[--no-radio-recover] [--no-radio-reboot]" >&2
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
    --no-net-watch)
      NET_WATCH=0
      ;;
    --no-radio-recover)
      RADIO_RECOVER=0
      ;;
    --no-radio-reboot)
      # The knob's off switch, not a skipped step: the installed unit's
      # Environment=AQUA_RADIO_REBOOT= line changes, so this rewrites it and
      # restarts the service, and a re-run without the flag puts it back.
      RADIO_REBOOT=off
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
if [[ ! "$NET_AUTOCONNECT_RETRIES" =~ ^(-1|[0-9]+)$ ]]; then
  echo "error: NET_AUTOCONNECT_RETRIES must be a whole number (0 = forever," \
    "-1 = NetworkManager's global default), got '$NET_AUTOCONNECT_RETRIES'" >&2
  exit 2
fi
for name in NET_WATCH_INTERVAL NET_WATCH_OUTAGE_INTERVAL; do
  if [[ ! "${!name}" =~ ^[1-9][0-9]*$ ]]; then
    echo "error: $name must be a whole number of seconds, got '${!name}'" >&2
    exit 2
  fi
done
for name in RADIO_GRACE RADIO_INTERVAL RADIO_ATTEMPT_DELAY RADIO_SETTLE \
  RADIO_REBOOT_WINDOW; do
  if [[ ! "${!name}" =~ ^[1-9][0-9]*$ ]]; then
    echo "error: $name must be a whole number of seconds, got '${!name}'" >&2
    exit 2
  fi
done
# The two where 0 is meaningful and means the same thing from two directions: no
# reload attempt at all, and no reboot ever.
for name in RADIO_ATTEMPTS RADIO_REBOOT_BUDGET; do
  if [[ ! "${!name}" =~ ^(0|[1-9][0-9]*)$ ]]; then
    echo "error: $name must be a whole number (0 = none), got '${!name}'" >&2
    exit 2
  fi
done
case "$RADIO_REBOOT" in
  on | off) ;;
  *)
    echo "error: RADIO_REBOOT must be 'on' or 'off' (--no-radio-reboot sets" \
      "'off'), got '$RADIO_REBOOT'" >&2
    exit 2
    ;;
esac
if [[ "$RADIO_SETTLE" -ge "$RADIO_REBOOT_WINDOW" ]]; then
  echo "error: RADIO_SETTLE ($RADIO_SETTLE) is not below RADIO_REBOOT_WINDOW" \
    "($RADIO_REBOOT_WINDOW): one wait for the netdev would outlast the window the" \
    "reboot budget is counted in" >&2
  exit 2
fi
if [[ "$NET_WATCH_OUTAGE_INTERVAL" -lt "$NET_WATCH_INTERVAL" ]]; then
  echo "error: NET_WATCH_OUTAGE_INTERVAL ($NET_WATCH_OUTAGE_INTERVAL) below" \
    "NET_WATCH_INTERVAL ($NET_WATCH_INTERVAL): an outage is the long part and" \
    "must not be sampled faster than health" >&2
  exit 2
fi
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

# nm_connections. The NetworkManager profiles connection.autoconnect-retries is
# set on and read back from -- never a profile name written down here, since a
# Wi-Fi profile is normally named after the SSID and this repository is public.
# In order: NET_CONNECTION, if the owner named one; else the profile
# NetworkManager has on $NET_IFACE right now; else every profile that names
# $NET_IFACE in connection.interface-name; else every profile of $NET_IFACE's own
# type (a board with one Wi-Fi profile, which is this one). Several at once on
# purpose, unlike aqua-net-recover.sh's single pick: this runs once, by hand, and
# the setting missing from whichever profile activates next is exactly the defect
# it exists to prevent. Prints nothing when nmcli knows of no candidate.
nm_connections() {
  if [[ -n "$NET_CONNECTION" ]]; then
    printf '%s\n' "$NET_CONNECTION"
    return 0
  fi
  local show bound dev_type line name ctype iface_of
  show="$(nmcli -t -f GENERAL.CONNECTION,GENERAL.TYPE device show "$NET_IFACE" 2> /dev/null || true)"
  bound="$(printf '%s\n' "$show" | sed -n 's/^GENERAL\.CONNECTION://p' | head -n 1)"
  bound="${bound//\\:/:}"
  if [[ -n "$bound" && "$bound" != "--" ]]; then
    printf '%s\n' "$bound"
    return 0
  fi
  dev_type="$(printf '%s\n' "$show" | sed -n 's/^GENERAL\.TYPE://p' | head -n 1)"
  local by_iface=() by_type=()
  # nmcli -t escapes a ':' inside a value as '\:', so the type is split off the
  # end (it never contains one) rather than by IFS.
  while IFS= read -r line; do
    [[ -n "$line" ]] || continue
    ctype="${line##*:}"
    name="${line%:*}"
    name="${name//\\:/:}"
    [[ -n "$name" ]] || continue
    iface_of="$(nmcli -t -f connection.interface-name connection show "$name" 2> /dev/null \
      | sed -n 's/^connection\.interface-name://p' | head -n 1)"
    if [[ "$iface_of" == "$NET_IFACE" ]]; then
      by_iface+=("$name")
    elif [[ -n "$dev_type" && "$ctype" == "$dev_type" ]]; then
      by_type+=("$name")
    fi
  done < <(nmcli -t -f NAME,TYPE connection show 2> /dev/null || true)
  if [[ "${#by_iface[@]}" -gt 0 ]]; then
    printf '%s\n' "${by_iface[@]}"
  elif [[ "${#by_type[@]}" -gt 0 ]]; then
    printf '%s\n' "${by_type[@]}"
  fi
}

# autoconnect_retries_of <profile>. The value NetworkManager actually has stored,
# first field only: nmcli prints some integer properties as "-1 (default)".
autoconnect_retries_of() {
  nmcli -t -f connection.autoconnect-retries connection show "$1" 2> /dev/null \
    | sed -n 's/^connection\.autoconnect-retries://p' | head -n 1 | awk '{ print $1 }'
}

# autoconnect_retries_report. Prints what NetworkManager will actually do after
# a failed association, not what was asked for -- read back from nmcli, the one
# question a writer of this setting cannot answer by looking at its own write,
# the same principle as watchdog_report and journald_storage_report. This is the
# setting whose silent default of 4 cost four days off the network, so a board
# where it did not take has to say so. Needs no root; --check calls it before
# anything is written, so it reports the board's *current* state.
autoconnect_retries_report() {
  if ! command -v nmcli > /dev/null 2>&1; then
    echo "  (nmcli not found, cannot verify connection.autoconnect-retries)"
    return 0
  fi
  local names=() name value
  while IFS= read -r name; do
    [[ -n "$name" ]] || continue
    names+=("$name")
  done < <(nm_connections)
  if [[ "${#names[@]}" -eq 0 ]]; then
    echo "  WARNING: NetworkManager knows no profile for NET_IFACE=$NET_IFACE, so" \
      "connection.autoconnect-retries is set nowhere and NetworkManager's own" \
      "default of 4 is what this board will use: four consecutive association" \
      "failures and autoconnect for the profile is blocked until a manual" \
      "activation, a NetworkManager restart or a reboot. Name the profile in" \
      "NET_CONNECTION and re-run. PROJECT.md §9 'Board hardening'."
    return 0
  fi
  for name in "${names[@]}"; do
    value="$(autoconnect_retries_of "$name")"
    echo "  connection.autoconnect-retries ($name): ${value:-unknown}"
    if [[ -n "$value" && "$value" == "$NET_AUTOCONNECT_RETRIES" ]]; then
      if [[ "$NET_AUTOCONNECT_RETRIES" == "0" ]]; then
        echo "  OK: matches NET_AUTOCONNECT_RETRIES=0 (0 = retry forever)"
      else
        echo "  OK: matches NET_AUTOCONNECT_RETRIES=$NET_AUTOCONNECT_RETRIES"
      fi
    else
      echo "  WARNING: asked for NET_AUTOCONNECT_RETRIES=$NET_AUTOCONNECT_RETRIES," \
        "NetworkManager has connection.autoconnect-retries=${value:-unknown} on" \
        "'$name'. -1 means NetworkManager's global default, which is 4: four" \
        "consecutive association failures and autoconnect for this profile is" \
        "blocked until a manual activation, a NetworkManager restart or a" \
        "reboot -- the four days of 2026-09-30. PROJECT.md §9 'Board hardening'."
    fi
  done
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

echo "== Wi-Fi autoconnect retries =="
# Not a drop-in: connection.autoconnect-retries is a per-profile property and
# NetworkManager.conf's [connection] defaults section does not cover it, so this
# is an nmcli write against whatever profile(s) nm_connections finds. It changes
# nothing about cooling and nothing about the active connection -- the property
# only decides how long NetworkManager keeps retrying after a failure -- so it is
# safe to run over ssh and safe with the router off.
if ! command -v nmcli > /dev/null 2>&1; then
  echo "  nmcli not found; nothing to set"
else
  NM_CONNECTIONS=()
  while IFS= read -r conn_name; do
    [[ -n "$conn_name" ]] || continue
    NM_CONNECTIONS+=("$conn_name")
  done < <(nm_connections)
  if [[ "${#NM_CONNECTIONS[@]}" -eq 0 ]]; then
    echo "  no NetworkManager profile found for NET_IFACE=$NET_IFACE; nothing to set" \
      "(the report below says what that leaves the board running)"
  fi
  for conn_name in "${NM_CONNECTIONS[@]}"; do
    conn_retries="$(autoconnect_retries_of "$conn_name")"
    if [[ "$conn_retries" == "$NET_AUTOCONNECT_RETRIES" ]]; then
      echo "  unchanged: '$conn_name' connection.autoconnect-retries=$conn_retries"
      continue
    fi
    CHANGED=1
    if [[ "$CHECK_ONLY" -eq 1 ]]; then
      echo "  would set: '$conn_name' connection.autoconnect-retries" \
        "${conn_retries:-unknown} -> $NET_AUTOCONNECT_RETRIES"
      continue
    fi
    if as_root nmcli connection modify "$conn_name" \
      connection.autoconnect-retries "$NET_AUTOCONNECT_RETRIES"; then
      echo "  set: '$conn_name' connection.autoconnect-retries=$NET_AUTOCONNECT_RETRIES"
    else
      echo "  WARNING: could not set connection.autoconnect-retries on '$conn_name'"
    fi
  done
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

NEEDS_WATCH_RESTART=0
if [[ "$NET_WATCH" -eq 1 ]]; then
  echo "== network diagnostic watcher =="
  install_text "$NET_WATCH_DST" 755 < "$NET_WATCH_SRC"
  NEEDS_WATCH_RESTART="$LAST_WROTE"
  watch_tmp="$(mktemp)"
  sed -e "s|^Environment=AQUA_NETWATCH_INTERVAL_S=.*|Environment=AQUA_NETWATCH_INTERVAL_S=$NET_WATCH_INTERVAL|" \
    -e "s|^Environment=AQUA_NETWATCH_OUTAGE_INTERVAL_S=.*|Environment=AQUA_NETWATCH_OUTAGE_INTERVAL_S=$NET_WATCH_OUTAGE_INTERVAL|" \
    "$NET_WATCH_UNIT_SRC" > "$watch_tmp"
  install_text "$NET_WATCH_UNIT_DST" 644 < "$watch_tmp"
  rm -f "$watch_tmp"
  if [[ "$LAST_WROTE" -eq 1 ]]; then
    NEEDS_WATCH_RESTART=1
  fi
else
  echo "== network diagnostic watcher: off (--no-net-watch) =="
  # Same argument as the timer above, and more pointed: this one is a
  # long-running service, so an earlier run's copy keeps sampling on its old
  # cadence until something stops it.
  if [[ -e "$NET_WATCH_UNIT_DST" ]]; then
    CHANGED=1
    if [[ "$CHECK_ONLY" -eq 1 ]]; then
      echo "  would stop and disable aqua-net-watch.service"
    else
      as_root systemctl disable --now aqua-net-watch.service || true
      echo "  stopped and disabled aqua-net-watch.service"
    fi
  fi
  remove_path "$NET_WATCH_UNIT_DST"
  remove_path "$NET_WATCH_DST"
  echo "  (the clean-stop marker under /var/lib/aqua-net-watch is left alone: it" \
    "is evidence, not configuration, and 'systemctl clean --what=state" \
    "aqua-net-watch.service' removes it)"
fi

NEEDS_RADIO_RESTART=0
if [[ "$RADIO_RECOVER" -eq 1 ]]; then
  echo "== radio recovery (an interface that does not exist) =="
  install_text "$RADIO_DST" 755 < "$RADIO_SRC"
  NEEDS_RADIO_RESTART="$LAST_WROTE"
  radio_tmp="$(mktemp)"
  # Every knob the unit carries is rewritten from the table at the top of this
  # script, including AQUA_RADIO_REBOOT, which is what makes --no-radio-reboot a
  # change to the installed unit rather than a skipped step.
  sed -e "s|^Environment=AQUA_RADIO_IFACE=.*|Environment=AQUA_RADIO_IFACE=$RADIO_IFACE|" \
    -e "s|^Environment=AQUA_RADIO_GRACE_S=.*|Environment=AQUA_RADIO_GRACE_S=$RADIO_GRACE|" \
    -e "s|^Environment=AQUA_RADIO_INTERVAL_S=.*|Environment=AQUA_RADIO_INTERVAL_S=$RADIO_INTERVAL|" \
    -e "s|^Environment=AQUA_RADIO_ATTEMPTS=.*|Environment=AQUA_RADIO_ATTEMPTS=$RADIO_ATTEMPTS|" \
    -e "s|^Environment=AQUA_RADIO_ATTEMPT_DELAY_S=.*|Environment=AQUA_RADIO_ATTEMPT_DELAY_S=$RADIO_ATTEMPT_DELAY|" \
    -e "s|^Environment=AQUA_RADIO_SETTLE_S=.*|Environment=AQUA_RADIO_SETTLE_S=$RADIO_SETTLE|" \
    -e "s|^Environment=AQUA_RADIO_REBOOT=.*|Environment=AQUA_RADIO_REBOOT=$RADIO_REBOOT|" \
    -e "s|^Environment=AQUA_RADIO_REBOOT_BUDGET=.*|Environment=AQUA_RADIO_REBOOT_BUDGET=$RADIO_REBOOT_BUDGET|" \
    -e "s|^Environment=AQUA_RADIO_REBOOT_WINDOW_S=.*|Environment=AQUA_RADIO_REBOOT_WINDOW_S=$RADIO_REBOOT_WINDOW|" \
    "$RADIO_UNIT_SRC" > "$radio_tmp"
  install_text "$RADIO_UNIT_DST" 644 < "$radio_tmp"
  rm -f "$radio_tmp"
  if [[ "$LAST_WROTE" -eq 1 ]]; then
    NEEDS_RADIO_RESTART=1
  fi
  if [[ "$RADIO_REBOOT" == "off" ]]; then
    echo "  escalation: AQUA_RADIO_REBOOT=off -- the driver is still reloaded," \
      "a radio that stays missing is reported hourly, and the board is never" \
      "rebooted from here"
  else
    echo "  escalation: at most $RADIO_REBOOT_BUDGET reboot(s) per" \
      "${RADIO_REBOOT_WINDOW}s, recorded in /var/lib/aqua-radio-recover/reboots" \
      "(never a tmpfs, so the count survives the reboot it is counting)"
  fi
else
  echo "== radio recovery: off (--no-radio-recover) =="
  # Same argument as the watcher: a long-running service left enabled keeps
  # running the copy of the script installed back then, with the attempt count
  # and the reboot budget it had back then. The flag has to be able to turn it
  # off again.
  if [[ -e "$RADIO_UNIT_DST" ]]; then
    CHANGED=1
    if [[ "$CHECK_ONLY" -eq 1 ]]; then
      echo "  would stop and disable aqua-radio-recover.service"
    else
      as_root systemctl disable --now aqua-radio-recover.service || true
      echo "  stopped and disabled aqua-radio-recover.service"
    fi
  fi
  remove_path "$RADIO_UNIT_DST"
  remove_path "$RADIO_DST"
  echo "  (the reboot ledger under /var/lib/aqua-radio-recover is left alone: it" \
    "is evidence, not configuration, and a budget deleted by hand is a budget" \
    "that starts again from zero. 'systemctl clean --what=state" \
    "aqua-radio-recover.service' removes it.)"
  echo "  (an interface that does not exist is then nobody's case: nothing on" \
    "this board will reload the driver, and the fans are unaffected either way," \
    "PROJECT.md §2)"
fi

if [[ "$CHECK_ONLY" -eq 1 ]]; then
  echo
  echo "== SoC watchdog (current board state, before any change) =="
  watchdog_report
  echo
  echo "== journald storage (current board state, before any change) =="
  journald_storage_report
  echo
  echo "== Wi-Fi autoconnect retries (current board state, before any change) =="
  autoconnect_retries_report
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
echo "== Wi-Fi autoconnect retries =="
autoconnect_retries_report
if [[ "$NET_RECOVER" -eq 1 ]]; then
  as_root systemctl enable --now aqua-net-recover.timer
  echo "  aqua-net-recover.timer: $(systemctl is-active aqua-net-recover.timer)"
fi
if [[ "$NET_WATCH" -eq 1 ]]; then
  echo "== network diagnostic watcher =="
  as_root systemctl enable --now aqua-net-watch.service
  # A long-running service keeps running whatever it was started with, so a
  # changed script or unit has to be picked up explicitly. Restarting it is
  # harmless by construction: it only reads, and its own ExecStop= records the
  # stop so the restart shows up in its boot report as a restart rather than as a
  # board that lost power.
  if [[ "$NEEDS_WATCH_RESTART" -eq 1 ]]; then
    as_root systemctl restart aqua-net-watch.service
    echo "  restarted to pick up the new script/unit"
  fi
  echo "  aqua-net-watch.service: $(systemctl is-active aqua-net-watch.service)"
  echo "  read an outage out of it with:"
  echo "    journalctl -t aqua-net-watch -o short-iso --since -2h"
fi
if [[ "$RADIO_RECOVER" -eq 1 ]]; then
  echo "== radio recovery =="
  as_root systemctl enable --now aqua-radio-recover.service
  # A long-running service keeps running whatever it was started with, so a
  # changed script or unit -- a different attempt count, a different reboot
  # budget, --no-radio-reboot -- has to be picked up explicitly. Restarting it is
  # harmless: it loads nothing on a board whose interface exists, and its next
  # check is one [[ -e ]].
  if [[ "$NEEDS_RADIO_RESTART" -eq 1 ]]; then
    as_root systemctl restart aqua-radio-recover.service
    echo "  restarted to pick up the new script/unit"
  fi
  echo "  aqua-radio-recover.service: $(systemctl is-active aqua-radio-recover.service)"
  echo "  read its whole history -- every absence, reload and escalation, across"
  echo "  every boot the journal holds -- with:"
  echo "    journalctl -t aqua-radio-recover -o short-iso --since -7d"
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

aqua-net-watch is not a watchdog at all and is not a layer above: it is the
instrument. It samples, every ${NET_WATCH_INTERVAL} s, which of the interface's
existence, the association, the IPv4 address, the default route and the gateway
is working, together with the SoC's throttle flags, core voltage and
temperature; it dumps everything on a change; and it reports at startup whether
the previous boot ended cleanly or the board lost power. It sends no packet,
loads no module, touches no service and writes nothing but a three-line
clean-stop marker. It cannot fight aqua-net-recover because it cannot act.

Nothing above reacts to the network. aqua-net-recover only re-associates the
Wi-Fi interface and brings its NetworkManager profile up again: it never
reboots, never restarts aqua-bridge and never touches a controller
(PROJECT.md §2). With the access point off, both are no-ops that cost a journal
line -- waiting is the whole response, and there is no escalation above it.

aqua-radio-recover is the one exception to that inertness, and a narrow one. It
acts on exactly one condition -- ${RADIO_IFACE} does not EXIST -- which is the
2026-10-06 fault: the brcmfmac firmware download to the Wi-Fi chip failed its
read-back verification in early boot, no netdev was ever registered, and the
board was unreachable for thirty hours while cooling perfectly. Every state of an
interface that does exist stays aqua-net-recover's, so the two can never act at
the same time. It waits ${RADIO_GRACE} s of uptime (the driver registers the
netdev a few seconds in), then reloads the Wi-Fi driver -- measured on this
board: two seconds, end to end, and NetworkManager reconnected by itself -- up to
${RADIO_ATTEMPTS} times, ${RADIO_ATTEMPT_DELAY} s apart, each with ${RADIO_SETTLE} s
to come back. Only then, and only with AQUA_RADIO_REBOOT=on
(currently ${RADIO_REBOOT}), does it reboot the board, at most
${RADIO_REBOOT_BUDGET} time(s) per ${RADIO_REBOOT_WINDOW} s, counted in a ledger on
/var that survives the reboot it is counting and refusing the reboot outright if
that ledger cannot be written. Past the budget it says hourly that it has given
up. It loads a module and it reboots; it opens no controller, stops or restarts
no service of any kind, and sends no packet.

connection.autoconnect-retries=${NET_AUTOCONNECT_RETRIES} on the Wi-Fi profile
is the other half of that: NetworkManager's default of 4 is what let the board
stop trying altogether on 2026-09-30, and the report above reads the effective
value back rather than trusting this script's own write.
EOF
if [[ "$NET_RECOVER" -eq 1 ]]; then
  echo "Try the recovery script by hand with"
  echo "  ${NET_RECOVER_DST} --dry-run"
else
  echo "The recovery script is not installed (--no-net-recover)."
fi
if [[ "$NET_WATCH" -eq 1 ]]; then
  echo "Try the watcher by hand with"
  echo "  ${NET_WATCH_DST} --once"
  echo "  ${NET_WATCH_DST} --boot-report"
else
  echo "The watcher is not installed (--no-net-watch)."
fi
if [[ "$RADIO_RECOVER" -eq 1 ]]; then
  echo "Ask the radio recovery what it sees, without it touching anything, with"
  echo "  ${RADIO_DST} --check"
else
  echo "The radio recovery is not installed (--no-radio-recover)."
fi
