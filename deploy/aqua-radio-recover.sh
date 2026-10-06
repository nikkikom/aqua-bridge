#!/usr/bin/env bash
# Bring the board's wireless interface back when it does not EXIST -- and do
# nothing else, ever.
# PROJECT.md §2, "Watchdog layering: the network is outside the cooling path"; §9.
#
# Why it exists. On 2026-10-06 the board booted, the daemon's heartbeat ran, the
# sensors and fans were fine, and there was no wireless interface at all: the
# brcmfmac driver's firmware download to the Wi-Fi chip over SDIO failed its
# read-back verification during early boot, so the driver never registered a
# netdev.
#
#   brcmfmac: brcmf_sdio_verifymemory: Downloaded RAM image is corrupted,
#             block offset is 440320, len is 1891
#   brcmfmac: brcmf_sdio_download_firmware: dongle image file download failed
#   mmc1: Controller never released inhibit bit(s).
#
# With no netdev there is nothing for anything above to work with: NetworkManager
# has no device, wpa_supplicant has nothing to associate, and
# aqua-net-recover.sh finds no connection to bring up. The board stayed
# unreachable for thirty hours in exactly that state while running perfectly. It
# is a known, open defect for this board (raspberrypi/linux issue 5770) with no
# fix, so the only recovery is to make the driver download the firmware again.
#
# Measured on the owner's board, by hand, before this script was written:
#
#   START present=yes
#   after-rmmod present=no           # modprobe -r brcmfmac_cyw brcmfmac
#   iface-back-after=1s present=yes  # modprobe brcmfmac ; modprobe brcmfmac_cyw
#   ipv4-after=1s addr=wlan0 UP <addr>/24
#   DONE total=2s
#
# Two seconds, end to end. The firmware re-downloaded and verified, NetworkManager
# reconnected by itself with nothing prompting it, and the board never reset.
# That measurement is why this unit exists and why a reboot is its LAST resort
# and not its first: the cure costs two seconds and a boot costs at least the 28 s
# this board takes from power to its first controller write -- which is longer
# than the aquaero's own 30 s software-sensor timeout, so every reset cashes out
# as the aquaero alarm, every fan at 100 %. Loud, never under-cooled, but not
# something to spend when two seconds of modprobe would have done.
#
# ---------------------------------------------------------------------------
# The division of labour with aqua-net-recover.sh -- one fact, two owners
# ---------------------------------------------------------------------------
# The two units are disjoint by their trigger, not by timing, and the dividing
# line is a single fact that cannot be true for both of them at once: does
# /sys/class/net/$AQUA_RADIO_IFACE exist?
#
#   * It does NOT exist -> this unit. There is no device, so there is nothing to
#     associate, disconnect, activate or probe; the only thing that can help is
#     reloading the driver, which is the one action here. This unit runs no
#     nmcli, no ping, no ip, no wpa_cli -- it cannot act on a link even if it
#     wanted to, and systemd denies it AF_INET so it cannot send a packet at all.
#   * It DOES exist -> aqua-net-recover.sh, in every case: connected and
#     carrying nothing, disconnected with nothing retrying it, unmanaged,
#     unavailable, mid-activation. That script never loads a module and never
#     reboots, and when NetworkManager reports no device for the interface it now
#     says so once and names this unit as the owner of that case rather than
#     counting it as an outage it is working on.
#
# So the two can never act on the same board state, and neither can undo the
# other's work: the moment this unit succeeds, the netdev exists and the case
# belongs to the other one; while the netdev is missing, the other one has
# nothing it can do. aqua-net-watch.sh observes both and acts on neither.
#
# ---------------------------------------------------------------------------
# What it will and will not do
# ---------------------------------------------------------------------------
# The owner's rule stands: nothing about the network may change what the fans do.
# So this script
#   * never opens a controller, never writes to the aquaero or the Quadro, and
#     never holds a hidraw device (the unit gets a private /dev with no node
#     added back, so it could not),
#   * never stops, starts, restarts, reloads or masks aqua-bridge.service or
#     aqua-heartbeat.service, and runs no systemctl at all,
#   * never touches the network: no nmcli, no ip, no ping, no wpa_cli, no
#     firewall, no route,
#   * does exactly two things that are not reads: it reloads the Wi-Fi driver,
#     and -- after AQUA_RADIO_ATTEMPTS failed reloads, if
#     AQUA_RADIO_REBOOT is on and the budget below allows it -- it reboots the
#     board. The owner approved the reboot explicitly: with no radio the board is
#     unreachable until somebody walks to it, and a reboot costs the aquaero
#     alarm, which is safe if loud.
#
# ---------------------------------------------------------------------------
# Why the reboot cannot loop
# ---------------------------------------------------------------------------
# A board in a reboot loop is strictly worse than a board with no network: it
# never finishes booting, so nobody can log in to fix it even standing next to
# it, and the fans spend the whole time on the controller's alarm profile. Four
# separate things bound it, and three of them hold even if the others are wrong:
#
#   1. A persistent ledger. Every reboot this script orders is appended, as an
#      epoch timestamp, to $AQUA_RADIO_STATE_DIR/reboots -- which is a
#      StateDirectory= on /var and deliberately NOT a RuntimeDirectory= on
#      tmpfs. A counter on tmpfs would be erased by exactly the event it is
#      counting, which is the whole mechanism of a reboot loop.
#   2. The ledger is written BEFORE the reboot is ordered, and read back. If the
#      append cannot be persisted and verified -- no state directory, a
#      read-only /var, a full card -- the reboot is REFUSED and said so loudly.
#      An unrecordable reboot is exactly the one that could repeat forever.
#   3. A budget in a window: at most AQUA_RADIO_REBOOT_BUDGET reboots per
#      AQUA_RADIO_REBOOT_WINDOW_S. Past that this script logs that it has given
#      up and keeps logging it, hourly, rather than rebooting again.
#   4. The grace and the ladder in front of it. A reboot cannot be ordered until
#      AQUA_RADIO_GRACE_S of uptime has passed and AQUA_RADIO_ATTEMPTS reloads
#      have each failed with AQUA_RADIO_SETTLE_S to come back and
#      AQUA_RADIO_ATTEMPT_DELAY_S between them. At the defaults that is 90 s plus
#      3·20 s plus 2·30 s = 210 s of a booted, running, cooling board before any
#      reboot -- so even a budget set absurdly high could not turn into a board
#      that never finishes booting.
#
# The worst case at the defaults is therefore: two reboots in any 24 h, never
# closer together than about three and a half minutes of a fully booted board,
# and then an hourly line saying it has given up and will not reboot again. The
# daemon cools throughout, including across both boots.
#
# Usage:
#   deploy/aqua-radio-recover.sh             # watch forever (what the unit runs)
#   deploy/aqua-radio-recover.sh --once      # one check and exit
#   deploy/aqua-radio-recover.sh --checks N  # N checks and exit
#   deploy/aqua-radio-recover.sh --check     # report only, change nothing
#
# --check is the safe thing to run by hand on a live board: it prints what is
# absent or present, how the grace stands, what the ledger holds and what one
# check WOULD do, and it loads no module and reboots nothing.
#
# ---------------------------------------------------------------------------
# How to read it
# ---------------------------------------------------------------------------
# One journal identifier covers the lot, so the whole history -- every absence,
# every reload, every escalation, across every boot the journal still holds -- is
# one command:
#
#   journalctl -t aqua-radio-recover -o short-iso --since -7d
#
# Line kinds, one per outcome:
#   started    one line per start of the unit: the interface, the cadence, the
#              ladder and the ledger as it stands.
#   HOLD       the interface is absent but the grace has not passed yet. The
#              driver normally registers the netdev a few seconds into boot, so
#              an absence before AQUA_RADIO_GRACE_S of uptime is a boot in
#              progress and not a fault. Repeated at most once every
#              AQUA_RADIO_LOUD_EVERY_S.
#   ABSENT     the verdict: the interface does not exist, the grace has passed.
#              Carries which netdevs DO exist, whether each driver module is
#              loaded, the uptime, and the escalation state.
#   RELOAD     one per attempt: the removal's exit status, the load's exit
#              status, how long the reload took, and whether the netdev came
#              back. This is the line the two-second measurement above is
#              checked against on a live board.
#   RECOVERED  the interface is back. How many attempts it took, how long it was
#              absent, how long the reload took.
#   REBOOT     the attempts are exhausted and a reboot is being ordered. Carries
#              the ledger: which reboot of the budget this is, and in what
#              window.
#   GAVEUP     the attempts are exhausted and the board is NOT being rebooted,
#              with the reason: the escalation is switched off, the budget is
#              spent, or the ledger could not be written. Repeated at most once
#              every AQUA_RADIO_LOUD_EVERY_S for as long as it lasts, because the
#              one thing worse than a board with no radio is a board with no
#              radio and a silent journal.
#
# ---------------------------------------------------------------------------
# What it costs
# ---------------------------------------------------------------------------
# While the interface exists -- which is all of the time, on a healthy board --
# one check is a single [[ -e ]] on /sys/class/net/<iface> and a read of
# /proc/uptime, both done by the shell with no fork, every
# AQUA_RADIO_INTERVAL_S. There is no network traffic, no nmcli, no journal line
# and nothing written. The whole steady-state cost is one "started" line per boot.
#
# ---------------------------------------------------------------------------
# Knobs (environment; defaults below). aqua-radio-recover.service passes the nine
# an owner is likely to want -- deploy/install-board-watchdogs.sh writes them
# from its own RADIO_* knob table -- and none of the others, so for anything else
# an override is "systemctl edit aqua-radio-recover.service".
# ---------------------------------------------------------------------------
#
# Deliberately NOT "set -e". This is a long-running unit whose whole job happens
# during a fault, and the one thing it may never do is die part-way through that
# fault because a /sys read returned EINVAL while the driver was reloading, or
# because a modprobe exited non-zero. Every command whose result matters is
# checked where it is called -- the reload is verified by the netdev appearing,
# never by an exit status -- and -u and pipefail stay on to catch the mistakes
# that are mistakes.
set -uo pipefail

# The interface whose ABSENCE is the one and only trigger: wlan0 on a Raspberry
# Pi Zero 2 W. It carries its own default rather than inheriting
# install-board-watchdogs.sh's NET_IFACE, for the same reason aqua-net-recover.sh
# and aqua-net-watch.sh do: it runs from a systemd unit with no environment and
# has to stand alone. All three defaults are the same name and
# tests/test_deploy.py checks that.
: "${AQUA_RADIO_IFACE:=wlan0}"
# The driver modules, in REMOVAL order -- dependents first. On this board
# brcmfmac_cyw depends on brcmfmac, so a removal has to name both or it fails
# with "Module brcmfmac is in use by: brcmfmac_cyw"; they are then loaded again in
# the reverse of this order, brcmfmac first and brcmfmac_cyw second, which is the
# order the owner measured the two-second cure with. brcmutil and cfg80211 are
# deliberately NOT here: nothing needs them unloaded, they are shared with the
# rest of the wireless stack, and the measured cure left them loaded.
: "${AQUA_RADIO_MODULES:=brcmfmac_cyw brcmfmac}"
# Seconds of UPTIME before an absent interface counts as a fault. The driver
# registers the netdev a few seconds into boot -- the firmware verdict, good or
# bad, is printed within about three seconds and NetworkManager's startup
# completes within about ten -- so anything earlier than this is a boot in
# progress, not a missing radio, and reloading a driver while the boot is still
# bringing interfaces up is how a working board gets broken. 90 s is thirty times
# the measured registration time and still well inside the thirty hours the
# defect costs when nothing acts. It is measured from boot, out of /proc/uptime,
# and NOT from this unit's start, on purpose: a restart of the unit hours later
# must not re-arm a grace that the boot satisfied long ago.
: "${AQUA_RADIO_GRACE_S:=90}"
# Seconds between two checks. A check on a healthy board is one [[ -e ]] and one
# read of /proc/uptime with no fork, so this could be far lower at no cost; 60 s
# is chosen as the detection latency it buys -- one minute against thirty hours
# -- and because nothing about this fault is sub-minute. It bounds how long a
# radio that dies mid-run (as opposed to at boot) stays missing before the first
# reload: one interval.
: "${AQUA_RADIO_INTERVAL_S:=60}"
# Reload attempts before the escalation. The measured cure worked on the first
# attempt in about two seconds; 3 covers "the SDIO host needed another go",
# which is exactly what the mmc1 "Controller never released inhibit bit(s)" line
# suggests can happen, without turning a chip that is genuinely dead into an
# endless rmmod/modprobe cycle. Set it to 0 to reload nothing -- which, with
# AQUA_RADIO_REBOOT=off, makes this unit a pure observer.
: "${AQUA_RADIO_ATTEMPTS:=3}"
# Seconds between two reload attempts. Long enough that the SDIO host and the
# chip are not being hammered -- the firmware download itself is a sub-second
# transfer and the whole measured cure was two seconds -- and short enough that
# three attempts plus the grace still fit inside four minutes.
: "${AQUA_RADIO_ATTEMPT_DELAY_S:=30}"
# Seconds a reload is given for the netdev to appear before that attempt counts
# as failed. Measured: one second. 20 is twenty times that, which covers a board
# that is busy with a control tick and an SDIO host that needs a retry, and it is
# the bound that keeps one whole check (3·20 s + 2·30 s = 120 s) well under two
# minutes.
: "${AQUA_RADIO_SETTLE_S:=20}"
# How often the netdev is looked for while waiting out AQUA_RADIO_SETTLE_S. One
# second, which is the resolution the two-second measurement was taken at, and a
# fork-free [[ -e ]] either way. The wait is bounded by
# AQUA_RADIO_SETTLE_S / AQUA_RADIO_SETTLE_POLL_S iterations as well as by the
# clock, so it ends after that many looks whatever the clock does.
: "${AQUA_RADIO_SETTLE_POLL_S:=1}"
# Whether the escalation to a reboot is enabled at all: "on" or "off". On by
# default, because the alternative the owner lived through is a board that cools
# perfectly and cannot be reached for thirty hours; "off" turns this unit into
# "reload the driver and, if that does not work, say so hourly and never reboot",
# which is a legitimate setting for a board sitting next to its owner.
# deploy/install-board-watchdogs.sh --no-radio-reboot writes "off" into the unit.
: "${AQUA_RADIO_REBOOT:=on}"
# Reboots this script may order within AQUA_RADIO_REBOOT_WINDOW_S. 2, because the
# first reboot is the one that has a real chance (a cold start of the SDIO host
# is a different draw of the same dice as a modprobe) and the second is the
# benefit of the doubt; a third in one day would be a board that reboots itself
# all day and still has no radio, which is the failure mode that is worse than the
# fault. 0 is accepted and means "never reboot", the same as
# AQUA_RADIO_REBOOT=off.
: "${AQUA_RADIO_REBOOT_BUDGET:=2}"
# The window that budget is counted in, in seconds. 86400 -- one day -- so the
# absolute worst case this unit can produce is two reboots a day of an otherwise
# healthy board, each one taking about half a minute, with an hourly line saying
# it has given up in between. The ledger holds timestamps, so the window slides
# rather than resetting on a boundary.
: "${AQUA_RADIO_REBOOT_WINDOW_S:=86400}"
# How often a condition that persists gets another line (seconds). It is the
# cadence of the HOLD and GAVEUP notices, and the defect it exists against is
# silence: thirty hours with no interface produced three supplicant lines and
# nothing else. Same default and same reasoning as aqua-net-recover.sh's
# AQUA_NET_LOUD_EVERY_S and aqua-net-watch.sh's AQUA_NETWATCH_LOUD_EVERY_S.
: "${AQUA_RADIO_LOUD_EVERY_S:=3600}"
# Where the reboot ledger lives. On /var and NOT on tmpfs, which is the whole
# anti-loop mechanism: a count erased by the reboot it is counting cannot bound
# anything. It matches aqua-radio-recover.service's StateDirectory=, which is
# also what makes this the one writable path under ProtectSystem=strict, and
# tests/test_deploy.py checks that the two agree.
: "${AQUA_RADIO_STATE_DIR:=/var/lib/aqua-radio-recover}"
# Prefix for /sys and /proc. Empty on a board. It exists so tests/test_deploy.py
# can run this script against a fabricated /sys and /proc on a machine that has a
# working wireless interface, or none at all, which is how the escalation ladder
# is tested without breaking a radio to do it. Same seam, same reason, as
# aqua-net-watch.sh's AQUA_NETWATCH_ROOT.
: "${AQUA_RADIO_ROOT:=}"

CHECK_ONLY=0
CHECKS_WANTED=0
while [[ "$#" -gt 0 ]]; do
  case "$1" in
    --check)
      CHECK_ONLY=1
      ;;
    --once)
      CHECKS_WANTED=1
      ;;
    --checks)
      shift
      if [[ "${1:-}" =~ ^[1-9][0-9]*$ ]]; then
        CHECKS_WANTED="$1"
      else
        echo "error: --checks needs a positive whole number" >&2
        exit 2
      fi
      ;;
    *)
      echo "Usage: $0 [--check | --once | --checks N]" >&2
      exit 2
      ;;
  esac
  shift
done

for name in AQUA_RADIO_GRACE_S AQUA_RADIO_INTERVAL_S AQUA_RADIO_ATTEMPT_DELAY_S \
  AQUA_RADIO_SETTLE_S AQUA_RADIO_SETTLE_POLL_S AQUA_RADIO_LOUD_EVERY_S \
  AQUA_RADIO_REBOOT_WINDOW_S; do
  if [[ ! "${!name}" =~ ^[1-9][0-9]*$ ]]; then
    echo "error: $name must be a positive whole number, got '${!name}'" >&2
    exit 2
  fi
done
# The two where 0 is meaningful, and means the same thing from two directions:
# no reload attempt at all, and no reboot ever.
for name in AQUA_RADIO_ATTEMPTS AQUA_RADIO_REBOOT_BUDGET; do
  if [[ ! "${!name}" =~ ^(0|[1-9][0-9]*)$ ]]; then
    echo "error: $name must be a whole number (0 = none), got '${!name}'" >&2
    exit 2
  fi
done
case "$AQUA_RADIO_REBOOT" in
  on | off) ;;
  *)
    echo "error: AQUA_RADIO_REBOOT must be 'on' or 'off', got '$AQUA_RADIO_REBOOT'" >&2
    exit 2
    ;;
esac
if [[ "$AQUA_RADIO_SETTLE_POLL_S" -gt "$AQUA_RADIO_SETTLE_S" ]]; then
  echo "error: AQUA_RADIO_SETTLE_POLL_S ($AQUA_RADIO_SETTLE_POLL_S) above" \
    "AQUA_RADIO_SETTLE_S ($AQUA_RADIO_SETTLE_S): the wait for the netdev would" \
    "look for it once and no sooner than after the wait was over" >&2
  exit 2
fi

IFACE="$AQUA_RADIO_IFACE"
SYS="${AQUA_RADIO_ROOT}/sys"
PROC="${AQUA_RADIO_ROOT}/proc"
NETCLASS="$SYS/class/net"
LEDGER="$AQUA_RADIO_STATE_DIR/reboots"
MODULES=()
read -r -a MODULES <<< "$AQUA_RADIO_MODULES"

log() {
  printf '%s\n' "$*"
}

# Seconds since the epoch into $NOW, without forking date(1): bash's printf has
# had %(fmt)T since 4.2 and this board runs 5.x.
now_s() {
  printf -v NOW '%(%s)T' -1
}

# A duration a person reads at a glance in a journal, not a seconds count. Same
# shape as aqua-net-recover.sh's and aqua-net-watch.sh's, deliberately, so three
# lines about one outage from three units read the same way.
duration_text() {
  local s="$1"
  if [[ "$s" -ge 86400 ]]; then
    printf '%dd%dh' "$((s / 86400))" "$(((s % 86400) / 3600))"
  elif [[ "$s" -ge 3600 ]]; then
    printf '%dh%dm' "$((s / 3600))" "$(((s % 3600) / 60))"
  elif [[ "$s" -ge 60 ]]; then
    printf '%dm%ds' "$((s / 60))" "$((s % 60))"
  else
    printf '%ds' "$s"
  fi
}

# The one fact this unit turns on. No fork, so the healthy case costs nothing.
iface_present() {
  [[ -e "$NETCLASS/$IFACE" ]]
}

# Whole seconds of uptime, out of /proc/uptime ("12345.67 11111.11"). The grace
# is measured from boot and not from this unit's start, so a restart of the unit
# does not re-arm it; an unreadable /proc/uptime reads as 0, which holds off
# rather than acts.
uptime_s() {
  local line=0
  if [[ -r "$PROC/uptime" ]]; then
    read -r line _ < "$PROC/uptime" || line=0
  fi
  printf '%s\n' "${line%%.*}"
}

# Which netdevs DO exist, for the verdict line: on 2026-10-06 the answer was "lo"
# and nothing else, and that one word is most of the diagnosis.
netdev_list() {
  local names=() entry
  for entry in "$NETCLASS"/*; do
    [[ -e "$entry" ]] || continue
    names+=("${entry##*/}")
  done
  if [[ "${#names[@]}" -eq 0 ]]; then
    printf 'none\n'
    return 0
  fi
  local joined
  printf -v joined '%s,' "${names[@]}"
  printf '%s\n' "${joined%,}"
}

# Each driver module as loaded or not, read out of /proc/modules with no fork.
# "brcmfmac=loaded,brcmfmac_cyw=no": a module that is loaded and still has no
# netdev is the firmware-download failure, and a module that is not loaded at all
# is a different fault (a missing blob, a blacklist) that a reload will not cure
# -- the reload is tried anyway, because trying costs two seconds, but the line
# says which one it was looking at.
module_state() {
  local loaded=() name field
  if [[ -r "$PROC/modules" ]]; then
    while read -r field _; do
      loaded+=("$field")
    done < "$PROC/modules"
  fi
  local out="" state found
  for name in "${MODULES[@]}"; do
    state=no
    for found in ${loaded[@]+"${loaded[@]}"}; do
      if [[ "$found" == "$name" ]]; then
        state=loaded
        break
      fi
    done
    out+="${out:+,}$name=$state"
  done
  printf '%s\n' "${out:-none}"
}

# --- the reboot ledger, which is the whole reason a reboot cannot loop ---------
#
# One epoch timestamp per line, on /var. REBOOTS_IN_WINDOW is how many of them
# fall inside AQUA_RADIO_REBOOT_WINDOW_S of now, and reading it also rewrites the
# file without the expired ones, so the file cannot grow without bound either.
REBOOTS_IN_WINDOW=0
prune_ledger() {
  REBOOTS_IN_WINDOW=0
  local kept=() stamp
  if [[ -r "$LEDGER" ]]; then
    while read -r stamp _; do
      [[ "$stamp" =~ ^[0-9]+$ ]] || continue
      now_s
      if [[ "$((NOW - stamp))" -lt "$AQUA_RADIO_REBOOT_WINDOW_S" ]]; then
        kept+=("$stamp")
      fi
    done < "$LEDGER"
  fi
  REBOOTS_IN_WINDOW="${#kept[@]}"
  if [[ "$CHECK_ONLY" -eq 1 ]]; then
    return 0
  fi
  # Rewritten only when something expired: a healthy board must not write to the
  # card on a schedule, and an untouched ledger is also one fewer thing that can
  # go wrong between a read and the reboot it authorises.
  local lines=0
  if [[ -r "$LEDGER" ]]; then
    while read -r _; do
      lines="$((lines + 1))"
    done < "$LEDGER"
  fi
  if [[ "$lines" -ne "$REBOOTS_IN_WINDOW" ]] && mkdir -p "$AQUA_RADIO_STATE_DIR" 2> /dev/null; then
    if [[ "${#kept[@]}" -eq 0 ]]; then
      : > "$LEDGER" 2> /dev/null || true
    else
      printf '%s\n' "${kept[@]}" > "$LEDGER" 2> /dev/null || true
    fi
  fi
}

# record_reboot. Appends now to the ledger and READS IT BACK, and fails if the
# count did not go up. This is the guard that matters most: a reboot whose
# record did not persist -- no state directory, /var read-only, the card full --
# is exactly the reboot that would repeat on every boot forever, so one that
# cannot be recorded is not ordered at all.
record_reboot() {
  local want="$((REBOOTS_IN_WINDOW + 1))"
  mkdir -p "$AQUA_RADIO_STATE_DIR" 2> /dev/null || return 1
  now_s
  printf '%s\n' "$NOW" >> "$LEDGER" 2> /dev/null || return 1
  local seen=0 stamp
  while read -r stamp _; do
    [[ "$stamp" =~ ^[0-9]+$ ]] || continue
    seen="$((seen + 1))"
  done < "$LEDGER" || return 1
  [[ "$seen" -ge "$want" ]]
}

# One line, at most once every AQUA_RADIO_LOUD_EVERY_S per kind, for a condition
# that persists. The state is a shell variable in one long-running process, so
# there is nothing on the card to keep and nothing to reset.
LAST_LOUD_HOLD=0
LAST_LOUD_GAVEUP=0
loud_due() {
  local last=0
  case "$1" in
    hold) last="$LAST_LOUD_HOLD" ;;
    gaveup) last="$LAST_LOUD_GAVEUP" ;;
  esac
  now_s
  if [[ "$last" -ne 0 && "$((NOW - last))" -lt "$AQUA_RADIO_LOUD_EVERY_S" ]]; then
    return 1
  fi
  case "$1" in
    hold) LAST_LOUD_HOLD="$NOW" ;;
    gaveup) LAST_LOUD_GAVEUP="$NOW" ;;
  esac
  return 0
}

ladder_text() {
  printf 'attempts=%s settle=%ss delay=%ss reboot=%s budget=%s/%s-in-%s' \
    "$AQUA_RADIO_ATTEMPTS" "$AQUA_RADIO_SETTLE_S" "$AQUA_RADIO_ATTEMPT_DELAY_S" \
    "$AQUA_RADIO_REBOOT" "$REBOOTS_IN_WINDOW" "$AQUA_RADIO_REBOOT_BUDGET" \
    "$(duration_text "$AQUA_RADIO_REBOOT_WINDOW_S")"
}

# Wait for the netdev to appear, bounded twice over: by AQUA_RADIO_SETTLE_S of
# clock and by AQUA_RADIO_SETTLE_S / AQUA_RADIO_SETTLE_POLL_S looks. Two bounds
# because one of them is a wall clock this script does not control -- and because
# tests/test_deploy.py runs it with a sleep that returns at once, where only the
# iteration bound ends the loop.
settle_wait() {
  local tries="$((AQUA_RADIO_SETTLE_S / AQUA_RADIO_SETTLE_POLL_S))"
  [[ "$tries" -ge 1 ]] || tries=1
  local started i
  now_s
  started="$NOW"
  for ((i = 0; i < tries; i++)); do
    if iface_present; then
      return 0
    fi
    now_s
    if [[ "$((NOW - started))" -ge "$AQUA_RADIO_SETTLE_S" ]]; then
      break
    fi
    sleep "$AQUA_RADIO_SETTLE_POLL_S"
  done
  iface_present
}

# One reload: remove the modules as listed (dependents first, which is why the
# knob is in removal order), then load them in the reverse of that order, which
# is brcmfmac before brcmfmac_cyw -- the order the two-second cure was measured
# with. Both exit statuses are reported and NEITHER is the verdict: the verdict is
# whether the netdev came back, which is the only thing that matters and the only
# thing a corrupted firmware download shows up in.
RELOAD_TOOK=0
RELOAD_RM_RC=0
RELOAD_LOAD_RC=0
reload_driver() {
  local started i rc
  now_s
  started="$NOW"
  RELOAD_RM_RC=0
  RELOAD_LOAD_RC=0
  modprobe -r "${MODULES[@]}" > /dev/null 2>&1 || RELOAD_RM_RC="$?"
  for ((i = "${#MODULES[@]}" - 1; i >= 0; i--)); do
    rc=0
    modprobe "${MODULES[i]}" > /dev/null 2>&1 || rc="$?"
    if [[ "$rc" -ne 0 ]]; then
      RELOAD_LOAD_RC="$rc"
    fi
  done
  local ok=1
  if settle_wait; then
    ok=0
  fi
  now_s
  RELOAD_TOOK="$((NOW - started))"
  return "$ok"
}

# The escalation, and the end of the ladder. Called only when the interface is
# still absent after AQUA_RADIO_ATTEMPTS reloads.
escalate() {
  local absent_for="$1" tried="$2"
  prune_ledger
  if [[ "$AQUA_RADIO_REBOOT" != "on" || "$AQUA_RADIO_REBOOT_BUDGET" -eq 0 ]]; then
    if loud_due gaveup; then
      log "GAVEUP iface=$IFACE absent=$(duration_text "$absent_for") tried=$tried" \
        "$(ladder_text): the reboot escalation is switched off" \
        "(AQUA_RADIO_REBOOT=$AQUA_RADIO_REBOOT," \
        "AQUA_RADIO_REBOOT_BUDGET=$AQUA_RADIO_REBOOT_BUDGET), so this line is the" \
        "whole response and nothing further happens: no reboot, no service is" \
        "touched, no controller is written, the fans are unaffected" \
        "(PROJECT.md §2). This board is unreachable over the network until" \
        "somebody looks at it."
    fi
    return 0
  fi
  if [[ "$REBOOTS_IN_WINDOW" -ge "$AQUA_RADIO_REBOOT_BUDGET" ]]; then
    if loud_due gaveup; then
      log "GAVEUP iface=$IFACE absent=$(duration_text "$absent_for") tried=$tried" \
        "$(ladder_text): the reboot budget for this window is spent and this" \
        "script will NOT reboot again. A board stuck in a reboot loop never" \
        "finishes booting and cannot be fixed even from the console, which is" \
        "worse than a board with no network, so the budget is a hard stop and not" \
        "a suggestion. Reloading the driver is still tried on every check. The" \
        "radio on this board is a known open defect (raspberrypi/linux 5770);" \
        "this board needs a look. The fans are unaffected (PROJECT.md §2)."
    fi
    return 0
  fi
  if ! record_reboot; then
    if loud_due gaveup; then
      log "GAVEUP iface=$IFACE absent=$(duration_text "$absent_for") tried=$tried" \
        "$(ladder_text): a reboot is due but it could NOT be recorded in" \
        "$LEDGER, so it is refused. An unrecorded reboot is the one that repeats" \
        "on every boot forever, and a board that never finishes booting is worse" \
        "than a board with no network. Check that" \
        "$AQUA_RADIO_STATE_DIR exists and is writable (the unit's" \
        "StateDirectory= creates it) and that the card is neither full nor" \
        "read-only. The fans are unaffected (PROJECT.md §2)."
    fi
    return 0
  fi
  log "REBOOT iface=$IFACE absent=$(duration_text "$absent_for")" \
    "tried=$tried reload attempt(s), all of which left $IFACE absent;" \
    "$(ladder_text); rebooting now. This is reboot" \
    "$((REBOOTS_IN_WINDOW + 1)) of $AQUA_RADIO_REBOOT_BUDGET allowed in" \
    "$(duration_text "$AQUA_RADIO_REBOOT_WINDOW_S") and it is recorded in" \
    "$LEDGER, which is on /var and survives the reboot: after the budget is" \
    "spent this script gives up loudly instead of rebooting again. The daemon is" \
    "stopped the ordinary way; the aquaero's own software-sensor timeout takes" \
    "every fan to 100 % while the board is down, which is loud and safe" \
    "(PROJECT.md §2)."
  reboot
}

# One check: the whole of this script's behaviour, in the order it is argued in
# the header. ABSENT_SINCE is 0 while the interface exists.
ABSENT_SINCE=0
check() {
  local up
  up="$(uptime_s)"
  if iface_present; then
    if [[ "$ABSENT_SINCE" -ne 0 ]]; then
      now_s
      log "RECOVERED iface=$IFACE present again after" \
        "$(duration_text "$((NOW - ABSENT_SINCE))") absent, without this script" \
        "having to act on this check"
      ABSENT_SINCE=0
    fi
    # The whole of the healthy case: nothing is logged, nothing is written,
    # nothing is run. Every state in which the interface EXISTS belongs to
    # aqua-net-recover.sh, including a link that is down, disconnected or dead.
    return 0
  fi

  now_s
  if [[ "$ABSENT_SINCE" -eq 0 ]]; then
    ABSENT_SINCE="$NOW"
  fi
  local absent_for="$((NOW - ABSENT_SINCE))"

  if [[ "$up" -lt "$AQUA_RADIO_GRACE_S" ]]; then
    if loud_due hold; then
      log "HOLD iface=$IFACE up=$(duration_text "$up") grace=${AQUA_RADIO_GRACE_S}s:" \
        "absent, but the boot is still bringing interfaces up and the driver" \
        "normally registers the netdev a few seconds in. Not acting."
    fi
    return 0
  fi

  prune_ledger
  log "ABSENT iface=$IFACE up=$(duration_text "$up")" \
    "absent=$(duration_text "$absent_for") netdevs=$(netdev_list)" \
    "mod=$(module_state) $(ladder_text)"

  if [[ "$CHECK_ONLY" -eq 1 ]]; then
    log "--check: $IFACE does not exist and the grace has passed, so a real run" \
      "would reload the driver now (modprobe -r $AQUA_RADIO_MODULES, then load" \
      "it back the other way round) up to $AQUA_RADIO_ATTEMPTS time(s). Nothing" \
      "was loaded, unloaded or rebooted."
    return 0
  fi

  if ! command -v modprobe > /dev/null 2>&1; then
    log "GAVEUP iface=$IFACE absent=$(duration_text "$absent_for")" \
      "$(ladder_text): modprobe is not on PATH, so the driver cannot be" \
      "reloaded from here and nothing else would help. This board is not the one" \
      "this unit was written for."
    return 0
  fi

  local attempt=0
  while [[ "$attempt" -lt "$AQUA_RADIO_ATTEMPTS" ]]; do
    attempt="$((attempt + 1))"
    if reload_driver; then
      log "RELOAD iface=$IFACE attempt=$attempt/$AQUA_RADIO_ATTEMPTS" \
        "rm_rc=$RELOAD_RM_RC load_rc=$RELOAD_LOAD_RC took=${RELOAD_TOOK}s" \
        "result=present"
      now_s
      log "RECOVERED iface=$IFACE after=$attempt reload attempt(s)" \
        "absent=$(duration_text "$((NOW - ABSENT_SINCE))")" \
        "took=${RELOAD_TOOK}s: the driver downloaded its firmware again and the" \
        "netdev is back. NetworkManager reconnects on its own from here, and" \
        "every state of a link that EXISTS belongs to aqua-net-recover.sh. No" \
        "reboot, no service touched, no controller written."
      ABSENT_SINCE=0
      LAST_LOUD_GAVEUP=0
      return 0
    fi
    log "RELOAD iface=$IFACE attempt=$attempt/$AQUA_RADIO_ATTEMPTS" \
      "rm_rc=$RELOAD_RM_RC load_rc=$RELOAD_LOAD_RC took=${RELOAD_TOOK}s" \
      "result=still-absent"
    if [[ "$attempt" -lt "$AQUA_RADIO_ATTEMPTS" ]]; then
      sleep "$AQUA_RADIO_ATTEMPT_DELAY_S"
    fi
  done

  now_s
  escalate "$((NOW - ABSENT_SINCE))" "$attempt"
}

# A SIGTERM from systemd is a stop, not a fault: say so and go, so that a restart
# to pick up a new copy of the script is one readable line in the journal.
on_term() {
  log "stopping on SIGTERM; $IFACE is $(iface_present && echo present || echo absent)"
  exit 0
}
trap on_term TERM INT

prune_ledger
if [[ "$CHECK_ONLY" -eq 1 ]]; then
  log "check: iface=$IFACE $(iface_present && echo present || echo absent)" \
    "up=$(duration_text "$(uptime_s)") grace=${AQUA_RADIO_GRACE_S}s" \
    "netdevs=$(netdev_list) mod=$(module_state) $(ladder_text)"
  check
  log "--check: nothing was loaded, unloaded, written or rebooted."
  exit 0
fi

log "started: watching $IFACE every ${AQUA_RADIO_INTERVAL_S}s, and acting on one" \
  "condition only -- the interface not existing. Every state of an interface that" \
  "DOES exist belongs to aqua-net-recover.service. Ladder: $(ladder_text);" \
  "grace=${AQUA_RADIO_GRACE_S}s of uptime; modules='$AQUA_RADIO_MODULES'" \
  "(removed in that order, loaded in the reverse); ledger=$LEDGER. It never" \
  "touches a controller, a service or the network."

checks=0
while true; do
  check
  checks="$((checks + 1))"
  if [[ "$CHECKS_WANTED" -ne 0 && "$checks" -ge "$CHECKS_WANTED" ]]; then
    break
  fi
  sleep "$AQUA_RADIO_INTERVAL_S"
done
