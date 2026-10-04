#!/usr/bin/env bash
# Keep the board's Wi-Fi interface reachable -- and do nothing else, ever.
# PROJECT.md §2, "Watchdog layering: the network is outside the cooling path".
#
# Two failures, two states, one action each:
#   * NetworkManager says the device is CONNECTED and the link carries nothing
#     (the 2026-09-17 outage: the BCM43430 in power save, associated and
#     answering nothing for hours). The probe is a ping of this interface's own
#     gateway; the action is "nmcli device disconnect/connect".
#   * NetworkManager says the device is DISCONNECTED and is not retrying (the
#     2026-09-30 outage: a disconnect storm spent the four autoconnect retries
#     NetworkManager allows by default, NetworkManager blocked autoconnect for
#     the profile, and the board sat there for four days -- wpa_supplicant
#     silent, not one association attempt). The action is to bring the profile
#     up again with "nmcli connection up", which is a manual activation and so
#     also resets NetworkManager's retry counter for it.
# This script used to stand down in the second case -- "NetworkManager owns
# that, standing by" -- which is why that outage lasted four days and cost a
# power cycle: it logged that same line every five minutes for four days while
# the one thing that could have acted was this. It no longer stands down.
# deploy/install-board-watchdogs.sh sets connection.autoconnect-retries=0 on the
# profile so NetworkManager does not give up in the first place; this is the
# belt to that braces, for a board whose profile was written before that ran.
#
# The rule this script exists to obey: the owner switching off the home router
# must not change what the fans do. So it
#   * never reboots the board and never powers anything down,
#   * never stops, starts or restarts aqua-bridge.service (a restart is a jolt
#     on the fans, §9 "Deploy side effect"), and never touches
#     aqua-heartbeat.service,
#   * never opens a controller and never writes one,
#   * treats an absent access point as a WAIT, never as a fault to escalate:
#     re-associating and activating a profile are the only two things it may
#     do, both are harmless with the router off (a failed activation is a
#     journal line and nothing else), and no number of failures ever unlocks a
#     bigger hammer -- there is no bigger hammer here to unlock,
#   * gives up quietly after AQUA_NET_MAX_BOUNCES fruitless re-associations of
#     a link that is up but dead, so that case costs one journal line and then
#     silence -- not an endless bounce loop,
#   * keeps retrying the DISCONNECTED case for as long as it lasts, because
#     nothing else will, but at its own rate (AQUA_NET_RECONNECT_MIN_S) and
#     saying so out loud once every AQUA_NET_LOUD_EVERY_S: four days of a stuck
#     link must never again read as four days of identical lines with nothing
#     happening behind them.
# It is the only thing in the repository that reacts to the network at all.
#
# Usage:
#   deploy/aqua-net-recover.sh            # one check (the systemd timer runs this)
#   deploy/aqua-net-recover.sh --dry-run  # report only, change nothing
#
# Knobs (environment; defaults below). aqua-net-recover.service passes none of
# them, so these defaults are what the board runs unless the owner adds an
# override with "systemctl edit aqua-net-recover.service".
set -euo pipefail

# The interface to watch: wlan0 on a Raspberry Pi Zero 2 W.
: "${AQUA_NET_IFACE:=wlan0}"
# The NetworkManager profile to activate when the device is disconnected. Empty
# means discover it (see connection_name below) -- and empty is the default on
# purpose: a Wi-Fi profile is usually named after the SSID, and this repository
# is public, so no name may be written down here. Set it only on a board where
# the discovery picks the wrong one of several profiles.
: "${AQUA_NET_CONNECTION:=}"
# Consecutive failed probes before one re-association of a link that is up but
# dead. 2, at the timer's 5 min cadence, means a link must be dead for about
# ten minutes: long enough that a single lost probe (a busy VideoCore, a
# roaming AP) never bounces a live link.
: "${AQUA_NET_FAIL_CHECKS:=2}"
# Re-associations of an up-but-dead link tried without it coming back before
# this script stops trying and waits for a probe to succeed on its own. 3
# covers "the driver is wedged"; more than that is not a driver fault, it is a
# network that is gone. It does NOT bound the disconnected case below: there
# nothing else is retrying, so neither may this stop.
: "${AQUA_NET_MAX_BOUNCES:=3}"
# Seconds between two "nmcli connection up" attempts on a disconnected device.
# This, not the timer, decides how hard an absent access point is knocked on:
# the timer is free to run as often as the owner likes without turning into
# retry pressure. 240 s sits just below the timer's 5 min default (minus its
# AccuracySec=30s), so on a stock board every run of the timer may act and the
# timer alone sets the cadence; a timer set faster than that is throttled here.
: "${AQUA_NET_RECONNECT_MIN_S:=240}"
# How often a link that stays down gets a loud journal line (seconds). The
# first check of an outage says what it is doing; after that, one line an hour
# naming how long it has been down and how many attempts it has made. The
# defect this replaced was silence, so the floor on this is "noticeable";
# hourly is 24 lines a day against a journal capped at 200 MB.
: "${AQUA_NET_LOUD_EVERY_S:=3600}"
# ICMP echos per probe, and the deadline for the whole probe, in seconds. Three
# echos in 5 s: a gateway on the same LAN answers the first in single-digit ms,
# and the deadline keeps one check far below the timer's interval.
: "${AQUA_NET_PING_COUNT:=3}"
: "${AQUA_NET_PING_DEADLINE_S:=5}"
# How long each blocking nmcli command may take, in seconds. nmcli's own
# defaults are 10 s for "device disconnect" and 90 s for "device connect" and
# "connection up" (nmcli(1) on this board), which are longer than a systemd
# start timeout: the unit would be SIGKILLed part-way through the activation,
# and a killed run is a run that never finished what it was doing. 20 s is far
# more than an association on this board needs and keeps one whole check inside
# aqua-net-recover.service's TimeoutStartSec=90. A check takes one branch or
# the other, never both, so the worst case is the up-but-dead one:
# 2*20 s + AQUA_NET_PING_DEADLINE_S = 45 s (the disconnected branch costs one
# 20 s activation), which tests/test_deploy.py checks against these defaults.
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

# Seconds since the epoch, without forking date(1): bash's own printf has had
# %(fmt)T since 4.2 and this board runs 5.x.
now_s() {
  local now
  printf -v now '%(%s)T' -1
  echo "$now"
}

# A duration a person reads at a glance in a journal, not a seconds count.
duration_text() {
  local s="$1"
  if [[ "$s" -ge 86400 ]]; then
    printf '%dd%dh' "$((s / 86400))" "$(((s % 86400) / 3600))"
  elif [[ "$s" -ge 3600 ]]; then
    printf '%dh%dm' "$((s / 3600))" "$(((s % 3600) / 60))"
  else
    printf '%dm' "$((s / 60))"
  fi
}

for tool in nmcli ip ping; do
  if ! command -v "$tool" > /dev/null 2>&1; then
    log "$tool not found; this board is not the one this unit was written for, nothing to do"
    exit 0
  fi
done

# GENERAL.STATE reads like "100 (connected)", GENERAL.CONNECTION like the
# profile's name or "--", GENERAL.TYPE like "wifi". One nmcli call for all
# three: the state decides the branch and the other two feed the discovery of
# the profile to activate.
device_show="$(nmcli -t -f GENERAL.STATE,GENERAL.CONNECTION,GENERAL.TYPE \
  device show "$AQUA_NET_IFACE" 2> /dev/null || true)"
device_field() {
  printf '%s\n' "$device_show" | sed -n "s/^$1://p" | head -n 1
}
state="$(device_field GENERAL.STATE)"
# "100 (connected)" -> "100". nmcli's words are localized, its codes are not.
state_code="${state%% *}"

# The profile to hand "nmcli connection up", never a name written down here.
# In order: the knob, if the owner set one; the profile NetworkManager has on
# this interface right now (the interface was connected a moment ago and the
# name is still there); the first profile that names this interface in
# connection.interface-name; and, failing that, the only profile of this
# device's type -- a board with exactly one Wi-Fi profile, which is this one.
# Ambiguity is not resolved by guessing: with several candidates of the same
# type and none naming the interface, this prints nothing and the caller says
# so rather than activating something nobody asked for.
connection_name() {
  if [[ -n "$AQUA_NET_CONNECTION" ]]; then
    printf '%s\n' "$AQUA_NET_CONNECTION"
    return 0
  fi
  local bound
  bound="$(device_field GENERAL.CONNECTION)"
  bound="${bound//\\:/:}"
  if [[ -n "$bound" && "$bound" != "--" ]]; then
    printf '%s\n' "$bound"
    return 0
  fi
  local dev_type line name ctype iface_of
  dev_type="$(device_field GENERAL.TYPE)"
  local by_iface=() by_type=()
  # nmcli -t escapes a ':' inside a value as '\:', so the fields are split off
  # the end (TYPE never contains one) rather than by IFS.
  while IFS= read -r line; do
    [[ -n "$line" ]] || continue
    ctype="${line##*:}"
    name="${line%:*}"
    name="${name//\\:/:}"
    [[ -n "$name" ]] || continue
    iface_of="$(nmcli -t -f connection.interface-name connection show "$name" 2> /dev/null \
      | sed -n 's/^connection\.interface-name://p' | head -n 1)"
    if [[ "$iface_of" == "$AQUA_NET_IFACE" ]]; then
      by_iface+=("$name")
    elif [[ -n "$dev_type" && "$ctype" == "$dev_type" ]]; then
      by_type+=("$name")
    fi
  done < <(nmcli -t -f NAME,TYPE connection show 2> /dev/null || true)
  if [[ "${#by_iface[@]}" -ge 1 ]]; then
    printf '%s\n' "${by_iface[0]}"
    return 0
  fi
  if [[ "${#by_type[@]}" -eq 1 ]]; then
    printf '%s\n' "${by_type[0]}"
    return 0
  fi
  return 1
}

# One line, once every AQUA_NET_LOUD_EVERY_S, for a link that has been down
# long enough that a person reading the journal should see it -- and worded so
# that seeing it can never be mistaken for a reason to escalate: with the
# router off this line IS the whole response, by design.
stuck_notice() {
  local now="$1" since="$2" tries="$3" down last_loud
  down="$((now - since))"
  if [[ "$down" -lt "$AQUA_NET_LOUD_EVERY_S" ]]; then
    return 0
  fi
  last_loud="$(counter down-last-loud)"
  if [[ "$last_loud" -ne 0 && "$((now - last_loud))" -lt "$AQUA_NET_LOUD_EVERY_S" ]]; then
    return 0
  fi
  set_counter down-last-loud "$now"
  log "WARNING: $AQUA_NET_IFACE has been off the network for $(duration_text "$down")" \
    "(${state:-unknown}) across $tries activation attempt(s), and is still down." \
    "If the access point is simply off, this line is the whole response and" \
    "nothing further happens: no reboot, no service restart, no controller is" \
    "touched, the fans are unaffected (PROJECT.md §2). If it is not off, this" \
    "board needs a look."
}

clear_down_state() {
  set_counter down-since 0
  set_counter down-tries 0
  set_counter down-last-try 0
  set_counter down-last-loud 0
}

if [[ "$state_code" != "100" ]]; then
  # The link is DOWN. This is the branch that used to stand down and leave it
  # to NetworkManager; NetworkManager had given up, and four days went by.
  now="$(now_s)"
  since="$(counter down-since)"
  first=0
  if [[ "$since" -eq 0 ]]; then
    since="$now"
    first=1
    set_counter down-since "$since"
  fi
  tries="$(counter down-tries)"

  if [[ "$state_code" != "30" ]]; then
    # 10 unmanaged, 20 unavailable (rfkill, no firmware), 40..90 an activation
    # already in progress. Activating a profile fixes none of those, and
    # getting in the way of an activation NetworkManager is already running
    # only slows it down. No action -- but counted in the same outage and
    # reported by stuck_notice, so a device parked here is never silent.
    if [[ "$first" -eq 1 ]]; then
      log "$AQUA_NET_IFACE is not connected (${state:-unknown}): NetworkManager is" \
        "either working on it or the radio is unavailable, so there is nothing" \
        "to activate from here; watching"
    fi
    stuck_notice "$now" "$since" "$tries"
    exit 0
  fi

  # 30 (disconnected): nothing is working on it. NetworkManager spends its own
  # autoconnect retries within about three minutes of a disconnect storm, so a
  # device still sitting here when a check arrives is a device nobody is
  # retrying -- act on the first such check rather than waiting for a second.
  last_try="$(counter down-last-try)"
  if [[ "$last_try" -ne 0 && "$((now - last_try))" -lt "$AQUA_NET_RECONNECT_MIN_S" ]]; then
    stuck_notice "$now" "$since" "$tries"
    exit 0
  fi
  if ! connection="$(connection_name)" || [[ -z "$connection" ]]; then
    if [[ "$first" -eq 1 ]]; then
      log "$AQUA_NET_IFACE is disconnected (${state:-unknown}) but no single" \
        "NetworkManager profile could be identified for it; set" \
        "AQUA_NET_CONNECTION to the one to activate. Nothing else is done."
    fi
    stuck_notice "$now" "$since" "$tries"
    exit 0
  fi
  tries="$((tries + 1))"
  log "$AQUA_NET_IFACE is disconnected (${state:-unknown}) and nothing is retrying it" \
    "(down $(duration_text "$((now - since))")); activating its profile again" \
    "(attempt $tries). This also resets NetworkManager's autoconnect retries."
  if [[ "$DRY_RUN" -eq 1 ]]; then
    log "--dry-run: nmcli connection up for $AQUA_NET_IFACE not run"
    exit 0
  fi
  # The bookkeeping moves BEFORE the nmcli, for the same reason as in the
  # up-but-dead branch below: an activation that hangs until systemd kills this
  # unit still has to count as the attempt it was, or the rate limit above
  # never engages and a faster timer turns into a flood.
  set_counter down-tries "$tries"
  set_counter down-last-try "$now"
  # Best effort. A failed activation is a journal line, never a failed unit and
  # never an escalation: with the access point absent this is the expected
  # outcome and the right response is to wait for the next check.
  if ! nmcli -w "$AQUA_NET_NMCLI_WAIT_S" connection up id "$connection" > /dev/null 2>&1; then
    log "activating the profile for $AQUA_NET_IFACE failed; the access point may" \
      "simply be absent, which is a wait and not a fault. Retrying no sooner" \
      "than ${AQUA_NET_RECONNECT_MIN_S}s from now."
  fi
  stuck_notice "$now" "$since" "$tries"
  exit 0
fi

# From here the device is connected, so whatever outage was running is over.
if [[ "$(counter down-since)" != "0" ]]; then
  log "$AQUA_NET_IFACE is connected again after $(counter down-tries) activation" \
    "attempt(s) and $(duration_text "$(($(now_s) - $(counter down-since)))") down"
  clear_down_state
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
  # timer period for as long as the router stays off -- in a branch whose whole
  # promise is that a router switched off costs one journal line. Held here,
  # every later check falls straight through to this branch and says nothing
  # until a probe succeeds (which clears the latch and both counters), or until
  # the device drops to "disconnected" and the branch above takes over.
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
