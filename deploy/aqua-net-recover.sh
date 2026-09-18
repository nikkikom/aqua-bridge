#!/usr/bin/env bash
# Re-associate the board's Wi-Fi interface when NetworkManager believes it is
# connected but the link carries nothing -- and do nothing else, ever.
# PROJECT.md §2, "Watchdog layering: the network is outside the cooling path".
#
# The rule this script exists to obey: the owner switching off the home router
# must not change what the fans do. So it
#   * never reboots the board and never powers anything down,
#   * never stops, starts or restarts aqua-bridge.service (a restart is a jolt
#     on the fans, §9 "Deploy side effect"), and never touches
#     aqua-heartbeat.service,
#   * never opens a controller and never writes one,
#   * gives up quietly after AQUA_NET_MAX_BOUNCES fruitless re-associations, so
#     a router that is simply off costs one journal line and then silence --
#     not an endless bounce loop, and not a line every timer period either.
# The failure it does fix is the one seen on 2026-09-17: the BCM43430 in power
# save, associated and answering nothing for hours. Power save is now off
# (deploy/install-board-watchdogs.sh writes the NetworkManager drop-in); this is
# the belt to that braces, and it is the only thing in the repository that
# reacts to the network at all.
#
# Usage:
#   deploy/aqua-net-recover.sh            # one check (the systemd timer runs this)
#   deploy/aqua-net-recover.sh --dry-run  # report only, re-associate nothing
#
# Knobs (environment; defaults below). aqua-net-recover.service passes none of
# them, so these defaults are what the board runs unless the owner adds an
# override with "systemctl edit aqua-net-recover.service".
set -euo pipefail

# The interface to watch: wlan0 on a Raspberry Pi Zero 2 W.
: "${AQUA_NET_IFACE:=wlan0}"
# Consecutive failed probes before one re-association. 2, at the timer's 5 min
# cadence, means a link must be dead for about ten minutes: long enough that a
# single lost probe (a busy VideoCore, a roaming AP) never bounces a live link.
: "${AQUA_NET_FAIL_CHECKS:=2}"
# Re-associations tried without the link coming back before this script stops
# trying and waits for a probe to succeed on its own. 3 covers "the driver is
# wedged"; more than that is not a driver fault, it is a network that is gone.
: "${AQUA_NET_MAX_BOUNCES:=3}"
# ICMP echos per probe, and the deadline for the whole probe, in seconds. Three
# echos in 5 s: a gateway on the same LAN answers the first in single-digit ms,
# and the deadline keeps one check far below the timer's interval.
: "${AQUA_NET_PING_COUNT:=3}"
: "${AQUA_NET_PING_DEADLINE_S:=5}"
# How long each of the two nmcli device commands may take, in seconds. nmcli's
# own defaults are 10 s for "device disconnect" and 90 s for "device connect"
# (nmcli(1) on this board), which together are longer than a systemd start
# timeout: the unit would be SIGKILLed part-way through the re-association, and
# a killed run is a run that never finished what it was doing. 20 s is far more
# than an association on this board needs and keeps one whole check --
# 2*20 s + AQUA_NET_PING_DEADLINE_S -- inside aqua-net-recover.service's
# TimeoutStartSec=90, which tests/test_deploy.py checks against these defaults.
: "${AQUA_NET_NMCLI_WAIT_S:=20}"
# Where the counters live. On tmpfs on purpose: they mean nothing across a
# reboot and must not wear the card. aqua-net-recover.service sets
# RuntimeDirectory=aqua-net-recover, which is exactly this path.
: "${AQUA_NET_STATE_DIR:=/run/aqua-net-recover}"

DRY_RUN=0
for arg in "$@"; do
  case "$arg" in
    --dry-run)
      DRY_RUN=1
      ;;
    *)
      echo "Usage: $0 [--dry-run]" >&2
      exit 2
      ;;
  esac
done

log() {
  echo "aqua-net-recover: $*"
}

counter() {
  local file="$AQUA_NET_STATE_DIR/$1"
  local value=0
  if [[ -r "$file" ]]; then
    read -r value < "$file" || value=0
  fi
  if [[ ! "$value" =~ ^[0-9]+$ ]]; then
    value=0
  fi
  echo "$value"
}

set_counter() {
  if [[ "$DRY_RUN" -eq 1 ]]; then
    return 0
  fi
  mkdir -p "$AQUA_NET_STATE_DIR"
  printf '%s\n' "$2" > "$AQUA_NET_STATE_DIR/$1"
}

for tool in nmcli ip ping; do
  if ! command -v "$tool" > /dev/null 2>&1; then
    log "$tool not found; this board is not the one this unit was written for, nothing to do"
    exit 0
  fi
done

# GENERAL.STATE reads like "100 (connected)". Anything else -- disconnected,
# unavailable, connecting -- is not this script's business: either
# NetworkManager is bringing the interface up on its own, or somebody took it
# down by hand ("nmcli device down", which also blocks autoconnect until a
# manual activation, nmcli(1)). Either way a second hand on the same interface
# only gets in the way, and this script never leaves the interface in that
# state itself: it restores autoconnect before it re-associates, below.
state="$(nmcli -t -f GENERAL.STATE device show "$AQUA_NET_IFACE" 2> /dev/null \
  | cut -d: -f2- || true)"
if [[ "$state" != 100* ]]; then
  log "$AQUA_NET_IFACE is not connected (${state:-unknown});" \
    "NetworkManager owns that, standing by"
  exit 0
fi

# The probe target is whatever gateway this interface was handed, never a
# hardcoded address: this repository is public and carries no LAN addresses,
# and a board with no lease has nothing to probe in the first place.
gateway="$(ip -4 route show default dev "$AQUA_NET_IFACE" 2> /dev/null \
  | awk '{ print $3; exit }' || true)"
if [[ -z "$gateway" ]]; then
  log "$AQUA_NET_IFACE has no default gateway; nothing to probe (cooling is unaffected)"
  exit 0
fi

if ping -n -q -I "$AQUA_NET_IFACE" -c "$AQUA_NET_PING_COUNT" \
  -w "$AQUA_NET_PING_DEADLINE_S" "$gateway" > /dev/null 2>&1; then
  if [[ "$(counter fails)" != "0" || "$(counter bounces)" != "0" ]]; then
    log "$AQUA_NET_IFACE reaches its gateway again"
  fi
  set_counter fails 0
  set_counter bounces 0
  if [[ "$DRY_RUN" -eq 0 ]]; then
    rm -f "$AQUA_NET_STATE_DIR/gave-up"
  fi
  exit 0
fi

fails="$(($(counter fails) + 1))"
if [[ "$fails" -lt "$AQUA_NET_FAIL_CHECKS" ]]; then
  log "gateway unreachable on $AQUA_NET_IFACE ($fails/$AQUA_NET_FAIL_CHECKS); waiting"
  set_counter fails "$fails"
  exit 0
fi

bounces="$(counter bounces)"
if [[ "$bounces" -ge "$AQUA_NET_MAX_BOUNCES" ]]; then
  if [[ ! -e "$AQUA_NET_STATE_DIR/gave-up" ]]; then
    log "gateway still unreachable after $bounces re-association(s):" \
      "treating the network itself as down (the router is off) and stopping here." \
      "Nothing is restarted, nothing is rebooted, the fans are unaffected."
    if [[ "$DRY_RUN" -eq 0 ]]; then
      mkdir -p "$AQUA_NET_STATE_DIR"
      touch "$AQUA_NET_STATE_DIR/gave-up"
    fi
  fi
  # Hold "fails" at the threshold rather than resetting it. Resetting would send
  # the next run down the "(1/N); waiting" branch and print a line every second
  # timer period for as long as the router stays off -- in a unit whose whole
  # promise is that a router switched off costs one journal line. Held here,
  # every later check falls straight through to this branch and says nothing
  # until a probe succeeds (which clears the latch and both counters).
  set_counter fails "$AQUA_NET_FAIL_CHECKS"
  exit 0
fi

log "gateway $gateway unreachable after $fails check(s);" \
  "re-associating $AQUA_NET_IFACE (attempt $((bounces + 1))/$AQUA_NET_MAX_BOUNCES)"
if [[ "$DRY_RUN" -eq 1 ]]; then
  log "--dry-run: nmcli device disconnect/connect $AQUA_NET_IFACE not run"
  exit 0
fi
# The counters move BEFORE the nmcli pair, not after: an nmcli that hangs long
# enough for systemd to kill this unit still has to count as the attempt it
# was, or "bounces" never reaches AQUA_NET_MAX_BOUNCES and the give-up above --
# the whole guard against bouncing a dead network forever -- is dead code in
# exactly the case it was written for.
set_counter fails 0
set_counter bounces "$((bounces + 1))"
# All three are best effort: a failed re-association is a journal line, never a
# failed unit and never an escalation to anything larger. Each nmcli carries an
# explicit --wait, because nmcli's own default for "device connect" is 90 s,
# which alone would outlast the unit's start timeout.
nmcli -w "$AQUA_NET_NMCLI_WAIT_S" device disconnect "$AQUA_NET_IFACE" > /dev/null 2>&1 \
  || log "nmcli device disconnect $AQUA_NET_IFACE failed"
# "device disconnect" is an alias for "device down", which also prevents the
# device from automatically activating further connections until a manual one
# (nmcli(1)). Restore that here, between the two, and not after the connect: if
# this run is killed part-way through the connect, NetworkManager still retries
# on its own instead of leaving the board off the network until somebody logs in
# locally. Idempotent, and it only ever undoes this script's own disconnect.
nmcli device set "$AQUA_NET_IFACE" autoconnect yes > /dev/null 2>&1 \
  || log "nmcli device set $AQUA_NET_IFACE autoconnect yes failed"
nmcli -w "$AQUA_NET_NMCLI_WAIT_S" device connect "$AQUA_NET_IFACE" > /dev/null 2>&1 \
  || log "nmcli device connect $AQUA_NET_IFACE failed"
