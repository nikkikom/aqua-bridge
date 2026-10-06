#!/usr/bin/env bash
# Watch the board's network from the inside and answer one question: WHAT FAILS
# FIRST. It observes and logs. It never acts.
# PROJECT.md §2, "Watchdog layering: the network is outside the cooling path"; §9.
#
# Why it exists. The board has lost IPv4 reachability over and over since it was
# first set up, and every diagnosis so far has been a post-mortem from whatever
# the journal happened to hold afterwards. Those outages are not all one fault,
# and they are told apart by the ORDER in which things stopped working -- which
# is exactly what no post-mortem had. Five states, lowest rung first; a sample
# names all five, and the lowest one that is bad is the diagnosis:
#
#   iface  the interface does not exist. No netdev, nothing to configure,
#         nothing to recover. This is what 2026-10-06 turned out to be: the
#         brcmfmac firmware download to the Wi-Fi chip over SDIO failed its
#         verification during early boot ("Downloaded RAM image is corrupted",
#         "dongle image file download failed", "mmc1: Controller never released
#         inhibit bit(s)"), so the driver never registered wlan0 at all.
#         NetworkManager's log for that boot ends at "startup complete" with
#         only loopback up; the supplicant logged three lines and then nothing
#         for thirty hours. The board itself ran perfectly throughout -- the
#         heartbeat kept feeding the controller and the fans were fine.
#         Nothing could recover it, because there was nothing to recover:
#         aqua-net-recover.sh finds no device, nmcli finds no device, the
#         supplicant has nothing to re-associate. The verdict was printed ONCE,
#         by the kernel, at boot, hours before anyone looked -- which is why
#         this script reads it out of this boot's kernel log and puts it on the
#         FIRST line it writes, instead of logging "no such file" for a day and
#         a half. See "The absent-interface case" below.
#   assoc  the interface exists and the association drops: no BSSID, wpa_state
#         leaves COMPLETED. A radio, AP or supplicant problem -- the shape of
#         2026-09-30, where 18 disconnects in three minutes (reason=0
#         locally_generated, reason=15 4-way-handshake timeout, one reason=16
#         group-key timeout) ended with the supplicant temp-disabling the SSID
#         and NetworkManager's four default autoconnect retries spent.
#   v4     the association holds and the IPv4 address disappears.
#         A DHCP problem: a lease that expired, a server that stopped answering.
#   rt     the address holds and the default route goes.
#         A NetworkManager or DHCP-option problem, not a radio one.
#   gw     all of that holds and the gateway simply stops answering.
#         A problem on the other end of the link, not on this board.
#
# Every line is built so the layer that moved first is the one you read first. A
# change line names the layers that moved AND the layers that held -- "held=
# v4,rt,gw" next to "wpa=COMPLETED->DISCONNECTED" is a whole diagnosis in one
# line, and so is "layers=iface:DOWN,assoc:na,v4:na,rt:na,gw:na".
#
# And underneath all five rungs, the rail. The 2026-10-06 forensics moved the
# question: the last healthy boot did not fail at the network, it DIED -- the
# journal stops mid-stream on a routine line eleven seconds after the interface
# had re-associated and taken a lease, with no service shutdown, no shutdown
# target, no panic, no OOM, no watchdog expiry. Then twenty boots in a row each
# died about six seconds in, to the second; the SoC watchdog's timeout is sixty
# seconds, so it was not that either. The boot that finally survived is the one
# whose radio firmware download over SDIO failed its checksum. A supply that
# collapses and retries looks exactly like that, and so does a marginal rail
# corrupting a bulk transfer -- but there is no positive evidence for it, because
# the firmware's throttle flags only accumulate within the current boot, so every
# clean reading we have was taken after a reset and says nothing about the event.
# That is not an argument to win, it is a measurement to take, and this is where
# it goes. So:
#
#   * every sample carries the SoC's own figures next to the network ones -- the
#     firmware's throttle/undervoltage flag word, the core voltage and the die
#     temperature -- and a change in that flag word is a state transition in its
#     own right, with a full snapshot behind it, exactly like a network one. The
#     point is partly the transition and partly the last line before a death:
#     whatever the journal's final sample says the rail was doing is evidence
#     nobody had before;
#   * and the unit reports, once at startup, whether the PREVIOUS boot ended
#     cleanly. The kernel cannot log its own power loss, so it is recorded from
#     the other side: ExecStop= writes a marker under the state directory (on
#     /var, not /run, so it outlives a reboot), and the next start either finds
#     it -- clean shutdown -- or does not -- the board was reset or lost power.
#     "Did it reboot or did it lose power" stops being an inference about every
#     future event and becomes a recorded fact. That report is also where this
#     boot's wireless-driver kernel lines and boot-time throttle flags go, and
#     where the journal's list of boots goes, so a run of six-second boots is
#     one line to find.
#
# Inert, and that is a hard promise, not a style. It never reconnects, never
# re-associates, never scans, never reloads a driver, never reboots, never
# starts, stops or restarts a service, never touches a fan controller and never
# writes to the aquaero. Its entire output is stdout, which systemd puts in the
# journal, with exactly one exception: at a clean stop it writes a marker file of
# three lines under $AQUA_NETWATCH_STATE_DIR, which is the whole mechanism of the
# boot report above and the only thing it ever writes. It also sends NO PACKETS
# AT ALL -- see "Not interfering with aqua-net-recover.sh" below, which is the
# reason, and it is a better one than tidiness.
#
# Usage:
#   deploy/aqua-net-watch.sh               # sample forever (the systemd unit)
#   deploy/aqua-net-watch.sh --samples 3   # three samples and exit, by hand
#   deploy/aqua-net-watch.sh --once        # one sample and exit
#   deploy/aqua-net-watch.sh --boot-report # the boot report alone, keeping the marker
#   deploy/aqua-net-watch.sh --mark-clean-stop   # what the unit's ExecStop= runs
#
# ---------------------------------------------------------------------------
# How to read it
# ---------------------------------------------------------------------------
# All of it is one journal unit, so the whole record of an outage is:
#
#   journalctl -u aqua-net-watch.service -o short-iso --since -2h
#
# and the skeleton of it -- every state change, every snapshot boundary, with
# the sample lines left out -- is:
#
#   journalctl -u aqua-net-watch.service -o short-iso --since -2h \
#     | grep -E ' (ABSENT|BOOT|CHANGE|DEGRADED|POWER|RECOVERED|ROAM|SILENT|snapshot (begin|end)) '
#
# Line kinds:
#   sample     one per interval while anything is moving, one per
#              AQUA_NETWATCH_IDLE_EVERY intervals while nothing is. Fields:
#              n= sample number, dev= present or absent, op= operstate,
#              car= carrier, wpa= wpa_state, bss= BSSID, ssid= SSID, fq= MHz,
#              sig= dBm, v4= address/prefix, rt= default gateway, nud= that
#              gateway's neighbour state, v6= count of global/ULA/link-local v6
#              addresses, rx=/tx= packet delta since the previous sample,
#              rxe/txe/rxd/txd= error and drop totals, thr= the firmware's
#              throttle flag word, uv= the kernel's undervoltage alarm bit,
#              volt= core voltage, temp= die temperature, layers= "ok" when all
#              five are good, else the five verdicts in ladder order.
#   ABSENT     the interface does not exist. Carries the verdict: whether the
#              driver module is loaded, which netdevs do exist, and the first
#              firmware/SDIO failure the kernel logged this boot, verbatim.
#              Repeated once every AQUA_NETWATCH_LOUD_EVERY_S for as long as it
#              lasts, so thirty hours of a missing radio is never thirty hours
#              of silence.
#   CHANGE     a layer moved. Names every component that moved as old->new, the
#              layers it belongs to, and held= the layers that did not move.
#   DEGRADED   the first sample in which any layer is bad. first= names which.
#   RECOVERED  all five good again. after= how long, lost= the order they went
#              in, back= the order they came back in, bssid= same or changed.
#   ROAM       the BSSID changed with the association never dropping. With two
#              access points on one SSID since 2026-10-05 this is now a real
#              event and not a curiosity.
#   SILENT     every layer reads good and the interface has received nothing for
#              AQUA_NETWATCH_SILENT_SAMPLES samples. This is the 2026-09-17
#              shape: associated, addressed, routed, and carrying nothing.
#   POWER      the firmware's throttle flag word or the undervoltage alarm bit
#              moved. Decodes the word into what is happening now and what has
#              happened since this boot, and takes a full snapshot.
#   BOOT       one headline at startup: whether the previous boot ended cleanly,
#              how many boots the journal holds, the previous boot's id and last
#              timestamp, and this boot's rail. The block behind it is tagged
#              "boot [<section>]".
#   snapshot   a full dump, bracketed by "snapshot begin"/"snapshot end". Every
#              body line is tagged with its section, so one section of one
#              snapshot is "grep 'snapshot \[scan\]'".
#
# One identifier covers the lot, so the whole story of an outage -- the boot
# report, the samples leading in, the snapshot at the transition, the recovery --
# is one command:
#
#   journalctl -t aqua-net-watch -o short-iso --since -2h
#
# ---------------------------------------------------------------------------
# The absent-interface case
# ---------------------------------------------------------------------------
# A watcher that only read /sys/class/net/<iface>/operstate would have printed
# "no such file" every fifteen seconds for thirty hours and said nothing useful.
# So absence is the lowest rung of the ladder and is treated as its own
# diagnosis, not as "down":
#
#   * the layers above it report "na", not "DOWN". The address being missing
#     from an interface that does not exist is not evidence of anything, and
#     four spurious DOWNs would bury the one that matters;
#   * the first sample that finds it missing says so with the verdict attached:
#     is $AQUA_NETWATCH_DRIVER_MODULE loaded, what netdevs do exist, and the
#     first line this boot's kernel log holds that matches
#     $AQUA_NETWATCH_FW_FAIL_GREP -- the firmware-download failure itself,
#     quoted;
#   * its snapshot reads this boot's radio kernel lines from the TOP, not the
#     tail. The firmware verdict is printed once, during early boot, long
#     before the first sample, so a tail of the recent ring buffer would miss
#     it entirely. It also lists /sys/class/net, /proc/modules for the driver
#     and its dependencies, the SDIO and MMC devices the chip hangs off, and
#     the firmware blobs on disk with their sizes;
#   * and it repeats, hourly, for as long as it lasts.
#
# It does not fix it, on purpose. Recovering a radio that never registered means
# reloading the driver module, which is a decision for the owner and a separate
# unit -- and a wrong one here would be a script that unloads a kernel module on
# a board whose only job is to keep cooling a disk enclosure.
#
# ---------------------------------------------------------------------------
# What it costs
# ---------------------------------------------------------------------------
# CPU: about eleven short-lived processes per sample (one "ip -o addr", one
# "ip -4 neigh", one "wpa_cli status", three vcgencmd reads sharing one subshell,
# and the timeout(1) wrappers around them) -- and four of those are the rail,
# which is the measurement the whole episode now turns on. None at all of the
# network ones while the interface is absent, since there is nothing to ask.
# Everything else -- presence, operstate, carrier, the packet and error counters,
# signal, the default route, the IPv6 scope counts, the die temperature, the
# undervoltage alarm bit, whether the driver module is loaded -- is read from
# /sys and /proc by the shell itself, with no fork. At the 15 s default that is
# still well under 1 % of one core, and the unit runs at Nice=19 with a low
# CPUWeight, so it yields to the control loop whenever the two actually compete.
#
# Journal: a sample line is about 190 bytes, of which the four power fields are
# about 35. Steady state is one line every 30 s (AQUA_NETWATCH_IDLE_EVERY=2 at
# 15 s), so about 2880 lines -- call it 1.3 MB of journal -- a day, of which the
# power fields are roughly 0.25 MB. One full snapshot per start of the unit, one
# boot report. A day with a handful of episodes stays under 2 MB. The ceiling, a
# day of nothing but flapping, is bounded on purpose and not by luck: sample
# lines cannot exceed one per AQUA_NETWATCH_OUTAGE_INTERVAL_S (1.4 MB/day), full
# snapshots cannot exceed one per AQUA_NETWATCH_FULL_MIN_S (about 6 MB/day), and
# brief ones one per AQUA_NETWATCH_SNAPSHOT_MIN_S (about 9 MB/day) -- 17 MB/day,
# in a case that would have told us the answer within its first hour. A day of
# absent interface is the cheapest case of all: 2880 sample lines, one snapshot
# and 24 hourly notices, under 1.4 MB.
#
# ---------------------------------------------------------------------------
# Not interfering with aqua-net-recover.sh
# ---------------------------------------------------------------------------
# That script is the only thing in this repository that acts on the network, and
# it stays that way. The two are kept apart by construction, not by timing:
#
#   * This one sends no packets. Not even a ping. A ping every 15 s would
#     refresh the gateway's ARP entry and keep the link busy, which would hide
#     the two symptoms we are hunting: it would mask a stale or failed
#     neighbour entry, and traffic is exactly what keeps the BCM43430 out of the
#     power-save state that made the link go deaf for hours on 2026-09-17. An
#     observer that keeps the patient awake is not observing. Instead,
#     aqua-net-recover.sh's own five-minutely ping IS the active probe, and this
#     script reads its result out of the kernel's neighbour table for free.
#   * It runs no command that changes anything: no nmcli verb other than
#     "device show"/"connection show", no "wpa_cli scan" (which makes the radio
#     act) -- only "status", "signal_poll" and "scan_results", which reads the
#     cache the supplicant already has -- no modprobe, rmmod or driver bind, and
#     no systemctl, reboot, ip-link or ip-address write of any kind.
#   * It shares no state with it. aqua-net-recover.sh keeps counters under
#     /run/aqua-net-recover; this script keeps its state in shell variables in
#     one long-running process and writes no files at all, so it cannot
#     corrupt, race or reset the other's bookkeeping.
#   * Nothing orders the two units against each other, and neither is wired to
#     aqua-bridge.service or aqua-heartbeat.service in any direction.
#   * It reads aqua-net-recover.service's journal into every full snapshot. So
#     when the recovery script re-associates the link, the resulting change is
#     attributed to it in the same snapshot rather than looking like a fault of
#     its own. The two cooperate in the record; they do not compete on the wire.
#
# ---------------------------------------------------------------------------
# Knobs (environment; defaults below). aqua-net-watch.service passes the two
# cadences -- deploy/install-board-watchdogs.sh writes them from
# NET_WATCH_INTERVAL and NET_WATCH_OUTAGE_INTERVAL -- and none of the others, so
# for anything else an override is "systemctl edit aqua-net-watch.service".
# ---------------------------------------------------------------------------
#
# Deliberately NOT "set -e". This is a long-running observer, and the one thing
# it may never do is die part-way through the outage it was installed to watch
# because a /sys read returned EINVAL while the interface was resetting. Every
# command whose failure matters is checked where it is called; -u and pipefail
# stay on to catch the mistakes that are mistakes.
set -uo pipefail

# The interface to watch: wlan0 on a Raspberry Pi Zero 2 W. It carries its own
# default rather than inheriting install-board-watchdogs.sh's NET_IFACE, for the
# same reason aqua-net-recover.sh does: it runs from a systemd unit with no
# environment and has to stand alone.
: "${AQUA_NETWATCH_IFACE:=wlan0}"
# Seconds between samples while the link is healthy. 15 s is the resolution of
# the answer: it is the worst case for how far apart two layers can fail and
# still be reported as having failed together, so it has to be well below the
# time the next layer takes to notice the one before it (a DHCP client gives up
# in seconds, ARP in tens of seconds). Below about 10 s the forks start to show
# on this board for no extra evidence; above about 30 s a disconnect and the
# lease loss behind it collapse into one sample and the ordering is lost, which
# is the whole point of the script.
: "${AQUA_NETWATCH_INTERVAL_S:=15}"
# Seconds between samples while any layer is bad. Slower on purpose: once the
# link is down the interesting transitions have already been snapshotted, and
# the job of the following hours (or, on 2026-10-06, thirty of them) is to keep
# the evolution on record without filling a capped journal. Must not be below
# AQUA_NETWATCH_INTERVAL_S -- an outage is not the time to sample faster than
# health, because an outage is the long part.
: "${AQUA_NETWATCH_OUTAGE_INTERVAL_S:=30}"
# While nothing at all is moving, print one sample line every this many samples
# instead of every sample. The sampling itself still happens every interval, so
# detection latency is unaffected; this only decides how much of "the board was
# fine" is written down. It was 4 -- one line a minute -- while the question was
# which network layer failed first, because a change always prints regardless.
# It is 2 because the question is now the rail: when the board dies without
# warning, the newest sample line in the journal IS the evidence, and this knob
# is how stale that line may be. At 2 it is at most 30 s old, for 1.3 MB of
# journal a day. 1 writes every sample and halves the staleness again; 4 restores
# the old cost. A sample in which the throttle word, the undervoltage bit or any
# network layer moved is printed whatever this says, and so is every sample while
# the firmware reports a live throttle condition.
: "${AQUA_NETWATCH_IDLE_EVERY:=2}"
# Floor between any two snapshots, in seconds. The 2026-09-30 storm was 18
# disconnects in three minutes: the change lines record all 18 in one line each,
# and this floor keeps the dumps behind them to two or three instead of 18.
: "${AQUA_NETWATCH_SNAPSHOT_MIN_S:=120}"
# Floor between two FULL snapshots, in seconds. A full one is taken when the
# unit starts, when the link first goes bad, when it comes back, when the BSSID
# changes under a live association, and when the interface turns out not to
# exist; everything else gets the brief one (addresses, routes, neighbours,
# supplicant status and signal -- the sections that answer "which layer" and
# nothing more). This is what bounds the worst case: a full snapshot is roughly
# five times a brief one.
: "${AQUA_NETWATCH_FULL_MIN_S:=900}"
# Lines kept from each tail in a full snapshot: the kernel ring buffer filtered
# to the wireless driver, and the NetworkManager, wpa_supplicant and
# aqua-net-recover journals. 25 is about two minutes of a busy NetworkManager
# and the whole of a quiet one.
: "${AQUA_NETWATCH_SNAPSHOT_LINES:=25}"
# Lines kept from the supplicant's cached scan results in a full snapshot. This
# is the section that shows the second access point: since 2026-10-05 two of
# them broadcast one SSID, so a roam is possible for the first time and the
# BSSIDs and their signal levels are evidence. 20 covers a flat's worth of
# neighbours.
: "${AQUA_NETWATCH_SCAN_MAX:=20}"
# Consecutive samples with every layer good and not one packet received before
# one SILENT line and one snapshot. On a home LAN something broadcasts
# constantly (ARP, mDNS, router advertisements), so zero received packets across
# 8 samples -- two minutes at the default -- is not a quiet network, it is a
# link that has gone deaf while still claiming to be up. That is the 2026-09-17
# shape, and all five layers read healthy throughout it.
: "${AQUA_NETWATCH_SILENT_SAMPLES:=8}"
# How often a condition that persists gets another line (seconds). It is the
# cadence of the hourly ABSENT notice, and the defect it exists against is
# silence: thirty hours with no interface produced three supplicant lines and
# nothing else, and a journal that says nothing for thirty hours is a journal
# nobody can diagnose from. Same default and same reasoning as
# aqua-net-recover.sh's AQUA_NET_LOUD_EVERY_S.
: "${AQUA_NETWATCH_LOUD_EVERY_S:=3600}"
# Seconds any one external command may take before it is killed. wpa_cli talks
# over a unix socket to a supplicant that may itself be wedged, and a sampler
# that blocks on that is a sampler that stops sampling exactly when it matters.
: "${AQUA_NETWATCH_CMD_TIMEOUT_S:=5}"
# Extended regex the kernel log is filtered through for a snapshot's radio
# sections. brcmfmac is this board's Wi-Fi driver and the chip hangs off SDIO on
# the mmc1 host, so sdio and mmc are in here too: "mmc1: Controller never
# released inhibit bit(s)" is part of the 2026-10-06 verdict and names neither
# the driver nor the interface. The rest is the generic wireless stack, listed so
# the same script says something useful on a board with a different radio.
: "${AQUA_NETWATCH_DRIVER_GREP:=brcmfmac|cfg80211|mac80211|ieee80211|wlan|sdio|mmc[0-9]}"
# Of those lines, the ones that are a verdict rather than chatter. Matched
# against this boot's kernel log when the interface is absent, and the first hit
# is quoted on the ABSENT line itself. The 2026-10-06 trio -- "Downloaded RAM
# image is corrupted", "dongle image file download failed", "Controller never
# released inhibit bit(s)" -- is matched by corrupt, "download failed" and
# inhibit respectively.
: "${AQUA_NETWATCH_FW_FAIL_GREP:=corrupt|verifymemory|download failed|firmware.*(fail|load)|inhibit|timed out|timeout}"
# The kernel module that is supposed to create the interface. Whether it is
# loaded is the first thing worth knowing when there is no interface, and it is
# read from /proc/modules, which costs nothing. It is never loaded, unloaded or
# reloaded from here.
: "${AQUA_NETWATCH_DRIVER_MODULE:=brcmfmac}"
# The firmware blobs the driver downloads to the chip, listed with their sizes in
# an absent-interface snapshot: "Downloaded RAM image is corrupted" is a verdict
# on the transfer, and the file on disk is the other half of it. 43430 is the
# chip on a Zero 2 W; widen it for a board with a different radio. At most
# AQUA_NETWATCH_SNAPSHOT_LINES files are listed however wide it is set, so a
# glob that matches a whole firmware tree cannot turn a snapshot into a
# directory listing.
: "${AQUA_NETWATCH_FIRMWARE_GLOB:=/lib/firmware/brcm/brcmfmac43430*}"
# Where the clean-stop marker lives. On /var and not /run on purpose: the whole
# point of it is to survive a reboot, so that a start which does not find it means
# the board was reset or lost power rather than shut down. It matches
# aqua-net-watch.service's StateDirectory=, which is also what makes this one path
# writable under ProtectSystem=strict.
: "${AQUA_NETWATCH_STATE_DIR:=/var/lib/aqua-net-watch}"
# Boots listed verbatim in the boot report, newest last. The 2026-10-05 signature
# was twenty boots of about six seconds each, six seconds apart: with the first
# and last entry of each boot printed side by side, that is one block to look at
# rather than a reconstruction. 10 covers it and costs ten lines once per start.
: "${AQUA_NETWATCH_BOOT_LIST:=10}"
# The thermal zone whose temperature is read from sysfs, which costs no fork and
# works with no /dev at all. vcgencmd's measure_temp overrides it when available,
# because that is the firmware's own figure, but this is what keeps a temperature
# on the line when vcgencmd is not installed or cannot reach the mailbox.
: "${AQUA_NETWATCH_THERMAL_ZONE:=thermal_zone0}"
# The systemd unit whose journal is tailed for the supplicant's own view.
: "${AQUA_NETWATCH_WPA_UNIT:=wpa_supplicant.service}"
# wpa_cli's control-interface directory. Empty means "whatever wpa_cli was
# compiled to use", which is right on Raspberry Pi OS, where NetworkManager
# starts wpa_supplicant with -O /run/wpa_supplicant.
: "${AQUA_NETWATCH_WPA_CTRL:=}"
# Prefix for /sys and /proc. Empty on a board. It exists so tests/test_deploy.py
# can run this sampler against a fabricated /sys and /proc on a machine with no
# wireless interface at all, which is how the state machine is tested without an
# outage to wait for.
: "${AQUA_NETWATCH_ROOT:=}"

SAMPLES_WANTED=0
MODE=watch
while [[ "$#" -gt 0 ]]; do
  case "$1" in
    --once)
      SAMPLES_WANTED=1
      ;;
    --mark-clean-stop)
      MODE=mark
      ;;
    --boot-report)
      MODE=report
      ;;
    --samples)
      shift
      if [[ "${1:-}" =~ ^[1-9][0-9]*$ ]]; then
        SAMPLES_WANTED="$1"
      else
        echo "error: --samples needs a positive whole number" >&2
        exit 2
      fi
      ;;
    *)
      echo "Usage: $0 [--once | --samples N | --boot-report | --mark-clean-stop]" >&2
      exit 2
      ;;
  esac
  shift
done

for name in AQUA_NETWATCH_INTERVAL_S AQUA_NETWATCH_OUTAGE_INTERVAL_S \
  AQUA_NETWATCH_IDLE_EVERY AQUA_NETWATCH_SNAPSHOT_LINES AQUA_NETWATCH_SCAN_MAX \
  AQUA_NETWATCH_SILENT_SAMPLES AQUA_NETWATCH_LOUD_EVERY_S AQUA_NETWATCH_CMD_TIMEOUT_S; do
  if [[ ! "${!name}" =~ ^[1-9][0-9]*$ ]]; then
    echo "error: $name must be a positive whole number, got '${!name}'" >&2
    exit 2
  fi
done
# The two snapshot floors, where 0 is meaningful: it takes every dump that is
# asked for. That is what tests/test_deploy.py runs with, and it is a legitimate
# setting for a board being watched by hand for an hour -- just not one to leave
# behind, since the floors are what bound the journal.
for name in AQUA_NETWATCH_SNAPSHOT_MIN_S AQUA_NETWATCH_FULL_MIN_S; do
  if [[ ! "${!name}" =~ ^(0|[1-9][0-9]*)$ ]]; then
    echo "error: $name must be a whole number of seconds (0 = no floor)," \
      "got '${!name}'" >&2
    exit 2
  fi
done
if [[ "$AQUA_NETWATCH_OUTAGE_INTERVAL_S" -lt "$AQUA_NETWATCH_INTERVAL_S" ]]; then
  echo "error: AQUA_NETWATCH_OUTAGE_INTERVAL_S ($AQUA_NETWATCH_OUTAGE_INTERVAL_S)" \
    "below AQUA_NETWATCH_INTERVAL_S ($AQUA_NETWATCH_INTERVAL_S): an outage is the" \
    "long part and must not be sampled faster than health" >&2
  exit 2
fi

IFACE="$AQUA_NETWATCH_IFACE"
SYS="${AQUA_NETWATCH_ROOT}/sys"
PROC="${AQUA_NETWATCH_ROOT}/proc"
NETCLASS="$SYS/class/net"
NETDIR="$NETCLASS/$IFACE"
STATS="$NETDIR/statistics"

log() {
  printf '%s\n' "$*"
}

# Seconds since the epoch into $NOW, without forking date(1): bash's printf has
# had %(fmt)T since 4.2 and this board runs 5.x. Called twice a sample, so the
# fork it saves is not nothing.
now_s() {
  printf -v NOW '%(%s)T' -1
}

# A duration a person reads at a glance in a journal, not a seconds count. Same
# shape as aqua-net-recover.sh's, deliberately, so two lines about one outage
# from two units read the same way.
duration_text() {
  local s="$1"
  if [[ "$s" -ge 86400 ]]; then
    printf '%dd%dh' "$((s / 86400))" "$(((s % 86400) / 3600))"
  elif [[ "$s" -ge 3600 ]]; then
    printf '%dh%dm' "$((s / 3600))" "$(((s % 3600) / 60))"
  else
    printf '%dm%ds' "$((s / 60))" "$((s % 60))"
  fi
}

# The first line of $1 into $REPLY_LINE, or "" if it is not there or not
# readable -- which is the normal case for several of these while an interface
# is being reset, and is never an error. No fork, no subshell.
read_line() {
  REPLY_LINE=""
  [[ -r "$1" ]] || return 0
  read -r REPLY_LINE < "$1" 2> /dev/null || REPLY_LINE=""
}

# A /proc/net/route gateway (little-endian hex) into $REPLY_IP as a dotted quad.
# Arithmetic expansion only: no fork.
hex_to_ip() {
  local h="$1"
  if [[ ! "$h" =~ ^[0-9A-Fa-f]{8}$ ]]; then
    REPLY_IP="?"
    return 0
  fi
  REPLY_IP="$((16#${h:6:2})).$((16#${h:4:2})).$((16#${h:2:2})).$((16#${h:0:2}))"
}

have() {
  command -v "$1" > /dev/null 2>&1
}

HAVE_TIMEOUT=0
have timeout && HAVE_TIMEOUT=1
HAVE_IP=0
have ip && HAVE_IP=1
HAVE_WPA_CLI=0
have wpa_cli && HAVE_WPA_CLI=1
HAVE_IW=0
have iw && HAVE_IW=1
HAVE_NMCLI=0
have nmcli && HAVE_NMCLI=1
HAVE_JOURNALCTL=0
have journalctl && HAVE_JOURNALCTL=1
HAVE_VCGENCMD=0
have vcgencmd && HAVE_VCGENCMD=1

# $CAP <- the output of a read-only command, bounded in time, never fatal. Every
# external command this script runs goes through here or through boot_kernel_log
# below, which is also what makes "it only observes" checkable by reading two
# functions instead of the whole file.
capture() {
  CAP=""
  if [[ "$HAVE_TIMEOUT" -eq 1 ]]; then
    CAP="$(timeout -k 1 "$AQUA_NETWATCH_CMD_TIMEOUT_S" "$@" 2>&1)" || true
  else
    CAP="$("$@" 2>&1)" || true
  fi
  return 0
}

# This boot's kernel log, filtered to the radio, oldest first, into $CAP. Read
# from the journal rather than with dmesg(1): journald holds every kernel line
# and is persistent on this board (install-board-watchdogs.sh), so this still
# works after the ring buffer has wrapped, and it needs no capability -- which
# is what lets the unit run with PrivateDevices=yes and an empty
# CapabilityBoundingSet. Oldest first is the point: the firmware verdict is
# printed once, in early boot, and a tail would never show it.
boot_kernel_log() {
  CAP=""
  [[ "$HAVE_JOURNALCTL" -eq 1 ]] || return 0
  if [[ "$HAVE_TIMEOUT" -eq 1 ]]; then
    CAP="$(timeout -k 1 "$AQUA_NETWATCH_CMD_TIMEOUT_S" journalctl -k -b --no-pager 2> /dev/null \
      | grep -Ei -- "$AQUA_NETWATCH_DRIVER_GREP")" || true
  else
    CAP="$(journalctl -k -b --no-pager 2> /dev/null \
      | grep -Ei -- "$AQUA_NETWATCH_DRIVER_GREP")" || true
  fi
  return 0
}

# The three firmware readings in ONE subshell rather than three: the throttle
# flag word, the core voltage and the die temperature. They are what the owner's
# "it was not a power sag" has never had evidence either way for, and the reason
# is in the knobs above -- the sticky bits are cleared by the reset that ends the
# event, so they can only be caught while the board is still up. vcgencmd needs
# /dev/vcio, which aqua-net-watch.service allows with DeviceAllow= and nothing
# else; if it is missing or blocked, every field here stays "?" and the sampler
# carries on, because this must not be a hard dependency.
vc_readings() {
  VC=""
  [[ "$HAVE_VCGENCMD" -eq 1 ]] || return 0
  if [[ "$HAVE_TIMEOUT" -eq 1 ]]; then
    VC="$( {
      timeout -k 1 "$AQUA_NETWATCH_CMD_TIMEOUT_S" vcgencmd get_throttled
      timeout -k 1 "$AQUA_NETWATCH_CMD_TIMEOUT_S" vcgencmd measure_volts core
      timeout -k 1 "$AQUA_NETWATCH_CMD_TIMEOUT_S" vcgencmd measure_temp
    } 2> /dev/null )" || true
  else
    VC="$( {
      vcgencmd get_throttled
      vcgencmd measure_volts core
      vcgencmd measure_temp
    } 2> /dev/null )" || true
  fi
  return 0
}

# $1 is a throttle flag word; $THR_NOW is what is happening right now and
# $THR_STICKY what has happened since this boot. Bit layout is the firmware's
# (raspberrypi documentation, "get_throttled"): the low nibble is live state, the
# same bits at 0x10000 are the latched "has occurred" ones -- latched within this
# boot only, which is exactly why a reading taken after the reset proves nothing.
throttle_decode() {
  THR_NOW=""
  THR_STICKY=""
  local w="$1" n
  if [[ ! "$w" =~ ^0x[0-9A-Fa-f]{1,8}$ ]]; then
    THR_NOW="?"
    THR_STICKY="?"
    return 0
  fi
  n="$((w))"
  ((n & 0x1)) && THR_NOW+="under-voltage,"
  ((n & 0x2)) && THR_NOW+="arm-freq-capped,"
  ((n & 0x4)) && THR_NOW+="throttled,"
  ((n & 0x8)) && THR_NOW+="soft-temp-limit,"
  ((n & 0x10000)) && THR_STICKY+="under-voltage,"
  ((n & 0x20000)) && THR_STICKY+="arm-freq-capped,"
  ((n & 0x40000)) && THR_STICKY+="throttled,"
  ((n & 0x80000)) && THR_STICKY+="soft-temp-limit,"
  THR_NOW="${THR_NOW%,}"
  THR_STICKY="${THR_STICKY%,}"
  [[ -n "$THR_NOW" ]] || THR_NOW="none"
  [[ -n "$THR_STICKY" ]] || THR_STICKY="none"
  return 0
}

# thr/uv/volt/temp into CUR. thr and uv are in the digest and so can be a
# transition; volt and temp are not, because they move on every sample of a
# perfectly healthy board.
power_sample() {
  local path line v
  CUR[thr]="?"
  CUR[uv]="?"
  CUR[volt]="?"
  CUR[temp]="?"
  read_line "$SYS/class/thermal/$AQUA_NETWATCH_THERMAL_ZONE/temp"
  if [[ "$REPLY_LINE" =~ ^-?[0-9]+$ ]]; then
    CUR[temp]="$((REPLY_LINE / 1000)).$(((REPLY_LINE % 1000) / 100))"
  fi
  # The kernel's own undervoltage alarm (the rpi_volt hwmon driver), which needs
  # no /dev and so is the floor under vcgencmd: even with the mailbox
  # unreachable, this bit is on every sample line.
  for path in "$SYS"/class/hwmon/hwmon*/in0_lcrit_alarm; do
    [[ -r "$path" ]] || continue
    read_line "$path"
    [[ -n "$REPLY_LINE" ]] && CUR[uv]="$REPLY_LINE"
    break
  done
  vc_readings
  [[ -n "$VC" ]] || return 0
  while IFS= read -r line; do
    case "$line" in
      throttled=*) CUR[thr]="${line#throttled=}" ;;
      volt=*) CUR[volt]="${line#volt=}" ;;
      temp=*)
        v="${line#temp=}"
        CUR[temp]="${v%\'C}"
        ;;
    esac
  done <<< "$VC"
  return 0
}

wpa_cli_args() {
  WPA_ARGS=(wpa_cli)
  if [[ -n "$AQUA_NETWATCH_WPA_CTRL" ]]; then
    WPA_ARGS+=(-p "$AQUA_NETWATCH_WPA_CTRL")
  fi
  WPA_ARGS+=(-i "$IFACE")
}
wpa_cli_args

# --- the sampled state ---------------------------------------------------------
#
# The digest. dev is the interface's existence, then the association, then the
# address, the route and the gateway's neighbour entry: that is the ladder whose
# ORDER is the diagnosis. v6 rides along as three booleans, and thr/uv are the
# rail; neither is a rung -- see EXTRA_LAYERS below.
COMP_NAMES=(dev op wpa bss ssid v4 rt nud v6 thr uv)
COMP_LAYER=(iface assoc assoc assoc assoc v4 rt gw v6 power power)
#: The five layers, lowest rung first. A line that names them in this order is a
#: line you read downwards, and the lowest bad one is the diagnosis.
LAYERS=(iface assoc v4 rt gw)
#: Reported and able to be a transition, but never a rung of the ladder: a
#: changed IPv6 scope is not an outage, and a latched throttle bit would make a
#: board that sagged once "degraded" for the rest of the boot.
EXTRA_LAYERS=(v6 power)

declare -A CUR=() PREV=() OK=()
declare -A CNT=() PCNT=()
declare -A LOST_AT=() BACK_AT=()

SEQ=0
SINCE_PRINT=0
LAST_SNAP=0
LAST_FULL=0
LAST_LOUD=0
DEGRADED=0
OUTAGE_SINCE=0
OUTAGE_LOST=""
OUTAGE_BACK=""
OUTAGE_BSS=""
OUTAGE_FQ=""
OUTAGE_SAMPLES=0
OUTAGE_SNAPS=0
SILENT_RUN=0
SILENT_SAID=0
WPA_WARNED=0
SNAP_THIS_SAMPLE=0
FW_VERDICT=""
FW_LINE=""

# Whether $AQUA_NETWATCH_DRIVER_MODULE is loaded, into $MODSTATE. /proc/modules,
# so no fork and no modinfo.
module_state() {
  MODSTATE="not-loaded"
  [[ -r "$PROC/modules" ]] || {
    MODSTATE="unknown"
    return 0
  }
  local name size used
  while read -r name size used _; do
    [[ "$name" == "$AQUA_NETWATCH_DRIVER_MODULE" ]] || continue
    MODSTATE="loaded(size=$size,used=$used)"
    return 0
  done < "$PROC/modules"
  return 0
}

# The netdevs that DO exist, into $NETDEVS. When the one we want is missing, the
# list says at a glance whether the radio is gone or merely renamed.
netdev_list() {
  NETDEVS=""
  local d
  for d in "$NETCLASS"/*; do
    [[ -e "$d" ]] || continue
    NETDEVS+="${d##*/},"
  done
  NETDEVS="${NETDEVS%,}"
  [[ -n "$NETDEVS" ]] || NETDEVS="none"
}

# The kernel's verdict on the radio for this boot, into $FW_VERDICT and the
# quoted $FW_LINE. Computed at most once per absent episode: it reads the whole
# boot's kernel log, which is the one expensive thing in this script, and its
# answer cannot change without a reboot.
firmware_verdict() {
  [[ -z "$FW_VERDICT" ]] || return 0
  FW_VERDICT="unknown"
  FW_LINE=""
  if [[ "$HAVE_JOURNALCTL" -eq 0 ]]; then
    FW_VERDICT="unknown(no-journalctl)"
    return 0
  fi
  boot_kernel_log
  if [[ -z "$CAP" ]]; then
    FW_VERDICT="no-radio-lines-this-boot"
    return 0
  fi
  local line
  while IFS= read -r line; do
    if [[ "$line" =~ ($AQUA_NETWATCH_FW_FAIL_GREP) ]]; then
      FW_VERDICT="kernel-logged-a-failure"
      FW_LINE="$line"
      return 0
    fi
  done <<< "$CAP"
  FW_VERDICT="radio-lines-but-no-failure-matched"
  return 0
}

sample() {
  local k v out line fam addr rest rif dest gwhex mask a b c

  SEQ=$((SEQ + 1))

  # The rail first, and unconditionally: it is the one reading that is worth
  # having even when there is no interface at all, and on this board the two
  # questions turned out to be the same question.
  power_sample

  # The lowest rung first, and it decides whether anything above it is even
  # asked. "absent" is not "down": there is no netdev, so there is nothing to
  # ask ip, nothing to ask the supplicant, and no fork worth making.
  if [[ -d "$NETDIR" ]]; then
    CUR[dev]="present"
    read_line "$NETDIR/operstate"
    CUR[op]="${REPLY_LINE:-?}"
    read_line "$NETDIR/carrier"
    CUR[car]="${REPLY_LINE:-?}"
  else
    CUR[dev]="absent"
    CUR[op]="-"
    CUR[car]="-"
  fi

  # Left EMPTY when there is no interface, not zeroed: a zero would read as a
  # delta of minus the whole counter on the way out and plus the whole counter on
  # the way back, which is two lies about traffic that never happened.
  for k in rx_packets tx_packets rx_errors tx_errors rx_dropped tx_dropped; do
    if [[ "${CUR[dev]}" == "present" ]]; then
      read_line "$STATS/$k"
      CNT[$k]="${REPLY_LINE:-0}"
    else
      CNT[$k]=""
    fi
  done

  # Signal and link quality from /proc/net/wireless, which costs nothing. The
  # supplicant's own signal_poll is finer but needs a fork; it is in the
  # snapshots, where one more fork does not matter.
  CUR[sig]="?"
  CUR[linkq]="?"
  if [[ "${CUR[dev]}" == "present" && -r "$PROC/net/wireless" ]]; then
    while read -r dev _ linkq level _; do
      [[ "$dev" == "$IFACE:" ]] || continue
      CUR[linkq]="${linkq%.}"
      CUR[sig]="${level%.}"
      break
    done < "$PROC/net/wireless"
  fi

  # Addresses. One "ip -o addr" gives both families; the v4 answer is the first
  # address that is not a 169.254/16 autoconfiguration one, because an IPv4
  # link-local address is the absence of a lease wearing a costume.
  CUR[v4]="-"
  local v6g=0 v6u=0 v6l=0
  if [[ "${CUR[dev]}" == "present" && "$HAVE_IP" -eq 1 ]]; then
    capture ip -o addr show dev "$IFACE"
    out="$CAP"
    if [[ -n "$out" ]]; then
      while read -r _ _ fam addr _; do
        case "$fam" in
          inet)
            [[ "$addr" == 169.254.* ]] && continue
            [[ "${CUR[v4]}" == "-" ]] && CUR[v4]="$addr"
            ;;
          inet6)
            case "${addr,,}" in
              fe80:*) v6l=$((v6l + 1)) ;;
              fc* | fd*) v6u=$((v6u + 1)) ;;
              *) v6g=$((v6g + 1)) ;;
            esac
            ;;
        esac
      done <<< "$out"
    fi
  fi
  CUR[v6count]="g${v6g}/u${v6u}/l${v6l}"
  # The digest carries presence per scope, not the prefixes. The owner is
  # explicit that the ISP re-dialling and changing the IPv6 global prefix is a
  # distraction and not the fault being hunted, so a new prefix replacing an old
  # one must not look like a state change and must not cost a snapshot.
  CUR[v6]="g$((v6g > 0 ? 1 : 0))u$((v6u > 0 ? 1 : 0))l$((v6l > 0 ? 1 : 0))"

  # The default route, from /proc/net/route: destination and mask both zero, on
  # this interface. Free, and the snapshot carries the whole table for the cases
  # this cannot see.
  CUR[rt]="-"
  if [[ "${CUR[dev]}" == "present" && -r "$PROC/net/route" ]]; then
    while read -r rif dest gwhex _ _ _ _ mask _; do
      [[ "$rif" == "$IFACE" ]] || continue
      [[ "$dest" == "00000000" && "$mask" == "00000000" ]] || continue
      hex_to_ip "$gwhex"
      CUR[rt]="$REPLY_IP"
      break
    done < "$PROC/net/route"
  fi

  # That gateway's neighbour entry. nud= is the kernel's own verdict on whether
  # the other end of the link is answering, and it is the one field here that
  # this script gets for free off somebody else's work: aqua-net-recover.sh
  # pings the same gateway every five minutes, and this is where the result of
  # that ping lands. A STALE entry on an idle link is normal; an entry that is
  # FAILED, INCOMPLETE or gone is the gateway not answering.
  CUR[nud]="-"
  CUR[nudc]="na"
  if [[ "${CUR[rt]}" != "-" && "$HAVE_IP" -eq 1 ]]; then
    CUR[nud]="none"
    CUR[nudc]="no"
    capture ip -4 neigh show dev "$IFACE"
    out="$CAP"
    if [[ -n "$out" ]]; then
      while read -r addr rest; do
        [[ "$addr" == "${CUR[rt]}" ]] || continue
        CUR[nud]="${rest##* }"
        if [[ "$rest" == *lladdr* ]]; then
          CUR[nudc]="yes"
        else
          CUR[nudc]="no"
        fi
        break
      done <<< "$out"
    fi
  fi

  # The supplicant's view: wpa_state, BSSID, SSID, frequency. The BSSID is on
  # every sample on purpose -- with two access points on one SSID since
  # 2026-10-05 it is the only field that can tell a roam from a reconnect, and a
  # roam is a newly possible failure mode on top of the old one.
  CUR[wpa]="-"
  CUR[bss]="-"
  CUR[ssid]="-"
  CUR[fq]="-"
  if [[ "${CUR[dev]}" == "present" ]]; then
    CUR[wpa]="?"
    if [[ "$HAVE_WPA_CLI" -eq 1 ]]; then
      capture "${WPA_ARGS[@]}" status
      out="$CAP"
      if [[ -n "$out" ]]; then
        while IFS= read -r line; do
          k="${line%%=*}"
          v="${line#*=}"
          [[ "$k" != "$line" ]] || continue
          case "$k" in
            wpa_state) CUR[wpa]="$v" ;;
            bssid) [[ -n "$v" ]] && CUR[bss]="$v" ;;
            # Whitespace out of the SSID: one field per token keeps the line
            # greppable, and the snapshot has it verbatim.
            ssid) [[ -n "$v" ]] && CUR[ssid]="${v//[[:space:]]/_}" ;;
            freq) CUR[fq]="$v" ;;
          esac
        done <<< "$out"
      fi
    fi
    # A supplicant that cannot be asked is not an excuse to stop reporting the
    # BSSID. "iw dev link" reads the association the driver already has and
    # starts nothing; "nmcli device wifi list" is NOT used as a fallback,
    # because it can trigger a scan, and making the radio act is the one thing
    # forbidden here.
    if [[ "${CUR[bss]}" == "-" && "$HAVE_IW" -eq 1 ]]; then
      capture iw dev "$IFACE" link
      out="$CAP"
      if [[ -n "$out" ]]; then
        while read -r a b c _; do
          case "$a $b" in
            "Connected to") CUR[bss]="$c" ;;
            "SSID: "*) CUR[ssid]="${b//[[:space:]]/_}" ;;
            "freq: "*) CUR[fq]="$b" ;;
          esac
        done <<< "$out"
        [[ "${CUR[wpa]}" == "?" && "${CUR[bss]}" != "-" ]] && CUR[wpa]="ASSOCIATED(iw)"
      fi
    fi
    if [[ "${CUR[wpa]}" == "?" && "$WPA_WARNED" -eq 0 ]]; then
      WPA_WARNED=1
      log "note: the interface exists but no wpa_state is available (wpa_cli and" \
        "iw both silent); the association layer is judged from operstate and" \
        "carrier alone, which cannot see a 4-way-handshake failure. Everything" \
        "else is unaffected."
    fi
  fi

  # --- the five verdicts, lowest rung first ---
  # 1 good, 0 bad, "na" not applicable. "na" matters: with no interface, the
  # address being missing is not evidence of anything, and four spurious DOWNs
  # would bury the one rung that is the diagnosis.
  if [[ "${CUR[dev]}" == "present" ]]; then
    OK[iface]=1
  else
    OK[iface]=0
    OK[assoc]="na"
    OK[v4]="na"
    OK[rt]="na"
    OK[gw]="na"
    return 0
  fi
  if [[ "${CUR[wpa]}" == "?" ]]; then
    if [[ "${CUR[op]}" == "up" && "${CUR[car]}" == "1" ]]; then OK[assoc]=1; else OK[assoc]=0; fi
  elif [[ "${CUR[op]}" == "up" && "${CUR[wpa]}" == "COMPLETED" && "${CUR[bss]}" != "-" ]]; then
    OK[assoc]=1
  else
    OK[assoc]=0
  fi
  if [[ "${CUR[v4]}" != "-" ]]; then OK[v4]=1; else OK[v4]=0; fi
  if [[ "${CUR[rt]}" != "-" ]]; then OK[rt]=1; else OK[rt]=0; fi
  # "na", not "DOWN", when there is no default route: whether the gateway
  # answers is then not unknown-and-bad but simply unknowable, and the rung below
  # already carries the fault. Same rule as the layers above an absent interface.
  if [[ "${CUR[nudc]}" == "yes" ]]; then
    OK[gw]=1
  elif [[ "${CUR[nudc]}" == "na" ]]; then
    OK[gw]="na"
  else
    OK[gw]=0
  fi
  return 0
}

# One sample line. Short enough that a month of them is still a thing a person
# scrolls through, and every field named so that grep finds it. "layers=ok" when
# all five are good keeps the common line short and makes a bad one jump out.
sample_line() {
  local d k layer verdicts="" all_ok=1
  local -a deltas=()
  for k in rx tx; do
    if [[ -z "${CNT[${k}_packets]}" ]]; then
      deltas+=("$k=-")  # no interface, so no counter to difference
    elif [[ -n "${PCNT[${k}_packets]:-}" ]]; then
      d="$((CNT[${k}_packets] - PCNT[${k}_packets]))"
      deltas+=("$k=+$d")
    else
      deltas+=("$k=+?")  # the first sample, or the first after one with no interface
    fi
  done
  for layer in "${LAYERS[@]}"; do
    case "${OK[$layer]}" in
      1) verdicts+="${layer}:up," ;;
      na)
        verdicts+="${layer}:na,"
        all_ok=0
        ;;
      *)
        verdicts+="${layer}:DOWN,"
        all_ok=0
        ;;
    esac
  done
  [[ "$all_ok" -eq 1 ]] && verdicts="ok,"
  log "sample n=$SEQ dev=${CUR[dev]} op=${CUR[op]} car=${CUR[car]} wpa=${CUR[wpa]}" \
    "bss=${CUR[bss]} ssid=${CUR[ssid]} fq=${CUR[fq]} sig=${CUR[sig]} v4=${CUR[v4]}" \
    "rt=${CUR[rt]} nud=${CUR[nud]} v6=${CUR[v6count]} ${deltas[0]} ${deltas[1]}" \
    "rxe=${CNT[rx_errors]:--} txe=${CNT[tx_errors]:--} rxd=${CNT[rx_dropped]:--}" \
    "txd=${CNT[tx_dropped]:--} thr=${CUR[thr]} uv=${CUR[uv]} volt=${CUR[volt]}" \
    "temp=${CUR[temp]} layers=${verdicts%,}"
}

# --- snapshots -----------------------------------------------------------------

SNAP_LINES=0
#: "snapshot" for a dump, "boot" for the boot report: same tagging, two blocks a
#: reader greps for by name.
SNAP_PREFIX="snapshot"

# Every body line of a snapshot goes through here, tagged with its section, so
# "grep 'snapshot \[route4\]'" is one section of one dump and nothing else.
snap_emit() {
  local section="$1" line
  while IFS= read -r line; do
    [[ -n "$line" ]] || continue
    printf '%s [%s] %s\n' "$SNAP_PREFIX" "$section" "$line"
    SNAP_LINES=$((SNAP_LINES + 1))
  done
}

# snap_cmd <section> <max-lines> <command...>: a bounded, read-only command.
snap_cmd() {
  local section="$1" max="$2"
  shift 2
  if ! have "$1"; then
    snap_emit "$section" <<< "($1 not on this board)"
    return 0
  fi
  capture "$@"
  if [[ -z "$CAP" ]]; then
    snap_emit "$section" <<< "(no output)"
    return 0
  fi
  snap_emit "$section" <<< "$(printf '%s\n' "$CAP" | head -n "$max")"
}

snap_file() {
  local section="$1" max="$2" path="$3"
  if [[ ! -r "$path" ]]; then
    snap_emit "$section" <<< "($path not readable)"
    return 0
  fi
  snap_emit "$section" <<< "$(head -n "$max" "$path")"
}

# The sections that answer "which layer", and nothing more. Every snapshot has
# these; a brief snapshot is only these.
snap_brief_sections() {
  netdev_list
  module_state
  snap_emit ladder <<< "dev=${CUR[dev]} netdevs=$NETDEVS module=$AQUA_NETWATCH_DRIVER_MODULE:$MODSTATE"
  snap_cmd addr 20 ip -o addr show dev "$IFACE"
  snap_cmd route4 20 ip -4 route show
  snap_cmd route6 20 ip -6 route show
  snap_cmd neigh4 20 ip -4 neigh show
  snap_cmd neigh6 20 ip -6 neigh show
  if [[ "$HAVE_WPA_CLI" -eq 1 && "${CUR[dev]}" == "present" ]]; then
    snap_cmd wpa 30 "${WPA_ARGS[@]}" status
    snap_cmd signal 15 "${WPA_ARGS[@]}" signal_poll
  fi
  snap_file wireless 10 "$PROC/net/wireless"
}

# Read from the TOP of this boot's radio kernel lines, not the tail. The
# 2026-10-06 verdict -- a firmware image that failed verification on its way
# into the chip -- was printed once, in early boot, and a tail of the recent
# ring buffer is exactly where it is not. In every full snapshot, not only the
# absent ones: it is the context for any episode, and it is bounded.
snap_boot_radio() {
  local lines="$AQUA_NETWATCH_SNAPSHOT_LINES"
  boot_kernel_log
  if [[ -z "$CAP" ]]; then
    snap_emit bootradio <<< "(no radio lines in this boot's kernel log)"
    return 0
  fi
  snap_emit bootradio <<< "$(printf '%s\n' "$CAP" | head -n "$lines")"
}

# Only when there is no interface: everything that says why there is no
# interface. The four questions are "is the driver loaded", "what netdevs exist
# instead", "did the chip's bus come up" and "is the firmware on disk".
snap_absent_sections() {
  local path
  firmware_verdict
  snap_emit absent <<< "verdict=$FW_VERDICT"
  [[ -n "$FW_LINE" ]] && snap_emit absent <<< "first-failure: $FW_LINE"
  module_state
  snap_emit absent <<< "module $AQUA_NETWATCH_DRIVER_MODULE: $MODSTATE"
  if [[ -r "$PROC/modules" ]]; then
    snap_emit modules <<< "$(grep -E "^(${AQUA_NETWATCH_DRIVER_MODULE}|brcmutil|cfg80211|mac80211|mmc_core|mmc_block|sdhci[a-z_]*) " \
      "$PROC/modules" 2> /dev/null)"
  fi
  netdev_list
  snap_emit netdevs <<< "$NETDEVS"
  for path in "$SYS"/bus/sdio/devices/* "$SYS"/class/mmc_host/*; do
    [[ -e "$path" ]] || continue
    snap_emit bus <<< "${path}"
  done
  local -a fw=()
  # shellcheck disable=SC2206  # the glob is a documented knob, by design
  for path in ${AQUA_NETWATCH_FIRMWARE_GLOB}; do
    [[ -e "$path" ]] || continue
    fw+=("$path")
    [[ "${#fw[@]}" -lt "$AQUA_NETWATCH_SNAPSHOT_LINES" ]] || break
  done
  if [[ "${#fw[@]}" -eq 0 ]]; then
    snap_emit firmware <<< "(nothing matches $AQUA_NETWATCH_FIRMWARE_GLOB)"
  else
    snap_cmd firmware "$AQUA_NETWATCH_SNAPSHOT_LINES" ls -l "${fw[@]}"
  fi
}

snap_full_sections() {
  local lines="$AQUA_NETWATCH_SNAPSHOT_LINES" k path
  # scan_results reads the cache the supplicant already holds. "wpa_cli scan"
  # would make the radio act and is never run from here. This is the section
  # that shows both access points on the one SSID.
  if [[ "$HAVE_WPA_CLI" -eq 1 && "${CUR[dev]}" == "present" ]]; then
    snap_cmd scan "$AQUA_NETWATCH_SCAN_MAX" "${WPA_ARGS[@]}" scan_results
  fi
  if [[ "$HAVE_NMCLI" -eq 1 ]]; then
    snap_cmd nmdev 60 nmcli -t device show "$IFACE"
    snap_cmd nmconn 20 nmcli -t connection show --active
  fi
  # The lease, if one can be read at all. NetworkManager's internal DHCP client
  # keeps no classic lease file, so its DHCP4 options above are usually the
  # better answer; these are for a board using dhclient.
  local found=0
  for path in /var/lib/NetworkManager/*"$IFACE"*.lease /var/lib/NetworkManager/*.lease \
    /run/NetworkManager/*.lease /var/lib/dhcp/*.leases; do
    [[ -r "$path" ]] || continue
    snap_emit lease <<< "== $path =="
    snap_emit lease <<< "$(tail -n "$lines" "$path")"
    found=1
    break
  done
  if [[ "$found" -eq 0 ]]; then
    snap_emit lease <<< "(no readable lease file; see the DHCP4 lines of [nmdev])"
  fi
  snap_boot_radio
  if [[ "$HAVE_JOURNALCTL" -eq 1 ]]; then
    # The recent end of the same filter, which is where a mid-session driver
    # event shows up -- as against snap_boot_radio, which is where a boot-time
    # one does.
    snap_emit kmsg <<< "$(journalctl -k --no-pager -n 400 2> /dev/null \
      | grep -Ei -- "$AQUA_NETWATCH_DRIVER_GREP" | tail -n "$lines")"
    snap_cmd nmlog "$lines" journalctl -u NetworkManager.service --no-pager -n "$lines"
    snap_cmd wpalog "$lines" journalctl -u "$AQUA_NETWATCH_WPA_UNIT" --no-pager -n "$lines"
    # Read, deliberately: when the one script that acts has just re-associated
    # the link, the change this snapshot is about belongs to it and not to a
    # fault, and that has to be visible in the same dump.
    snap_cmd recoverlog "$lines" journalctl -u aqua-net-recover.service --no-pager -n "$lines"
  fi
  snap_file loadavg 1 "$PROC/loadavg"
  snap_emit mem <<< "$(grep -E '^(MemTotal|MemFree|MemAvailable|Buffers|Cached|SwapFree):' \
    "$PROC/meminfo" 2> /dev/null)"
  snap_file uptime 1 "$PROC/uptime"
  for k in rx_packets tx_packets rx_bytes tx_bytes rx_errors tx_errors rx_dropped \
    tx_dropped rx_over_errors rx_crc_errors rx_frame_errors collisions; do
    read_line "$STATS/$k"
    [[ -n "$REPLY_LINE" ]] && snap_emit counters <<< "$k $REPLY_LINE"
  done
  read_line "$NETDIR/carrier_changes"
  [[ -n "$REPLY_LINE" ]] && snap_emit counters <<< "carrier_changes $REPLY_LINE"
  # The rail, in full. The sample line has the same four fields; here they come
  # with the flag word decoded, because "0x50005" is not something to decode by
  # hand at two in the morning.
  throttle_decode "${CUR[thr]}"
  snap_emit power <<< "throttled=${CUR[thr]} now=$THR_NOW since-boot=$THR_STICKY"
  snap_emit power <<< "core-volts=${CUR[volt]} temp=${CUR[temp]} undervolt-alarm=${CUR[uv]}"
  snap_emit power <<< "(the since-boot bits latch within this boot only and are cleared by a reset)"
  for path in "$SYS"/class/hwmon/hwmon*/in0_lcrit_alarm; do
    [[ -r "$path" ]] || continue
    read_line "$path"
    snap_emit power <<< "$path $REPLY_LINE"
  done
  for path in "$SYS"/class/thermal/*/temp; do
    [[ -r "$path" ]] || continue
    read_line "$path"
    snap_emit power <<< "$path $REPLY_LINE"
  done
}

# snapshot <full|brief> <reason...>
snapshot() {
  local kind="$1"
  shift
  SNAP_LINES=0
  log "snapshot begin n=$SEQ kind=$kind reason=$*"
  snap_brief_sections
  if [[ "${CUR[dev]}" == "absent" ]]; then
    snap_absent_sections
  fi
  if [[ "$kind" == "full" ]]; then
    snap_full_sections
  fi
  log "snapshot end n=$SEQ kind=$kind lines=$SNAP_LINES"
  OUTAGE_SNAPS=$((OUTAGE_SNAPS + 1))
}

# want_snapshot <full|brief> <reason...>: the floors live here, in one place, so
# the ceiling on what this script writes per day is one function and not a habit.
want_snapshot() {
  local kind="$1"
  shift
  # Counted whether or not the floor lets it through: one attempt per sample is
  # the budget, and a second attempt in the same sample would only print a
  # second "deferred" line.
  SNAP_THIS_SAMPLE=1
  now_s
  if [[ "$kind" == "full" ]] \
    && [[ "$LAST_FULL" -eq 0 || "$((NOW - LAST_FULL))" -ge "$AQUA_NETWATCH_FULL_MIN_S" ]]; then
    LAST_FULL="$NOW"
    LAST_SNAP="$NOW"
    snapshot full "$@"
    return 0
  fi
  if [[ "$LAST_SNAP" -eq 0 || "$((NOW - LAST_SNAP))" -ge "$AQUA_NETWATCH_SNAPSHOT_MIN_S" ]]; then
    LAST_SNAP="$NOW"
    snapshot brief "$@"
    return 0
  fi
  log "snapshot deferred n=$SEQ wanted=$kind reason=$* (floor" \
    "${AQUA_NETWATCH_SNAPSHOT_MIN_S}s/${AQUA_NETWATCH_FULL_MIN_S}s; the CHANGE line" \
    "above has the ordering, which is what the floor protects)"
}

# The headline for the lowest rung, with the verdict on it rather than a
# pointer to one. Said on the first sample that finds no interface and then once
# every AQUA_NETWATCH_LOUD_EVERY_S, because the failure it names lasted thirty
# hours and produced no other line in the whole system.
absent_notice() {
  local now="$1" first="$2" down=0
  if [[ "$first" -eq 0 ]]; then
    if [[ "$LAST_LOUD" -ne 0 && "$((now - LAST_LOUD))" -lt "$AQUA_NETWATCH_LOUD_EVERY_S" ]]; then
      return 0
    fi
  fi
  LAST_LOUD="$now"
  [[ "$OUTAGE_SINCE" -ne 0 ]] && down="$((now - OUTAGE_SINCE))"
  firmware_verdict
  module_state
  netdev_list
  log "ABSENT n=$SEQ iface=$IFACE does not exist: the driver never registered a" \
    "netdev, so there is nothing to associate, address, route or recover" \
    "(absent $(duration_text "$down"))." \
    "module=$AQUA_NETWATCH_DRIVER_MODULE:$MODSTATE netdevs=$NETDEVS" \
    "kernel=$FW_VERDICT${FW_LINE:+: \"$FW_LINE\"}." \
    "Nothing here fixes this and nothing else on the board will either:" \
    "aqua-net-recover.sh finds no device, nmcli finds no device, the supplicant" \
    "has nothing to re-associate. Bringing a radio back that never registered" \
    "means reloading the driver, which is the owner's decision and not this" \
    "unit's (PROJECT.md §9). Cooling is unaffected (§2)."
}

# --- the boot report -----------------------------------------------------------
#
# The kernel cannot log its own power loss: when the rail collapses, the last
# thing the journal holds is whatever happened to be mid-write, which is exactly
# what the 2026-10-06 journal holds and exactly why nobody could tell a reset
# from a shutdown afterwards. So it is recorded from the other side. On a clean
# stop the unit writes a three-line marker; on the next start, the marker either
# is there -- the board was shut down -- or is not -- the board was reset or lost
# power. Either way the marker is cleared, so the next start answers for the next
# boot and not for this one.

BOOT_MARKER="$AQUA_NETWATCH_STATE_DIR/last-clean-stop"

# This boot's id, in the 32-hex form journalctl prints, into $BOOT_ID.
this_boot_id() {
  BOOT_ID="unknown"
  read_line "$PROC/sys/kernel/random/boot_id"
  [[ -n "$REPLY_LINE" ]] && BOOT_ID="${REPLY_LINE//-/}"
}

# What aqua-net-watch.service's ExecStop= runs, and the only write this script
# ever makes. A stop that cannot write it says so, because a silently missing
# marker would read as a power loss on the next start -- which is the one wrong
# answer this whole mechanism must not give.
mark_clean_stop() {
  this_boot_id
  now_s
  if ! mkdir -p "$AQUA_NETWATCH_STATE_DIR" 2> /dev/null; then
    log "WARNING: cannot create $AQUA_NETWATCH_STATE_DIR, so this clean stop goes" \
      "unrecorded and the next start will report it as a boot that ended without" \
      "one. Nothing else is affected."
    return 0
  fi
  if ! {
    printf 'boot_id=%s\n' "$BOOT_ID"
    printf 'stopped_at=%s\n' "$NOW"
    printf 'stopped_iso=%(%Y-%m-%dT%H:%M:%S%z)T\n' "$NOW"
  } > "$BOOT_MARKER" 2> /dev/null; then
    log "WARNING: cannot write $BOOT_MARKER; the next start will report this boot" \
      "as having ended without a clean stop."
    return 0
  fi
  log "clean stop recorded for boot $BOOT_ID. If the next start does not find" \
    "$BOOT_MARKER, the board was reset or lost power rather than shut down."
  return 0
}

# boot_report <clear 0|1>. One headline, then a tagged block. The headline is the
# line that turns "did it reboot or did it lose power" from an inference into a
# recorded fact, so everything needed to answer it is on that one line.
boot_report() {
  local clear="$1" line verdict prev_id="-" prev_range="-" marker_boot="" marker_iso=""
  local boots="?" uptime="?"
  this_boot_id
  if [[ -r "$BOOT_MARKER" ]]; then
    while IFS= read -r line; do
      case "$line" in
        boot_id=*) marker_boot="${line#boot_id=}" ;;
        stopped_iso=*) marker_iso="${line#stopped_iso=}" ;;
      esac
    done < "$BOOT_MARKER"
    if [[ "$marker_boot" == "$BOOT_ID" ]]; then
      # Not a reboot at all: this unit was restarted inside one boot, by
      # Restart=always or by hand. Saying "clean" here would be a lie about a
      # boot that has not ended yet.
      verdict="unit-restarted-within-this-boot"
    else
      verdict="previous-shutdown-was-CLEAN"
    fi
  else
    verdict="previous-boot-ended-WITHOUT-a-clean-stop"
  fi
  read_line "$PROC/uptime"
  [[ -n "$REPLY_LINE" ]] && uptime="${REPLY_LINE%% *}"
  # journalctl's own list of boots: the previous boot's id and the timestamp of
  # its last entry, which for a board that lost power is the moment it died.
  local -a boot_lines=()
  if [[ "$HAVE_JOURNALCTL" -eq 1 ]]; then
    capture journalctl --list-boots --no-pager
    if [[ -n "$CAP" ]]; then
      local idx id rest
      while IFS= read -r line; do
        # The index column is right-aligned, so the leading whitespace has to go
        # before anything can be matched against it; read does that for free.
        read -r idx id rest <<< "$line"
        case "$idx" in
          0 | -[0-9]*) ;;
          *) continue ;;  # the header line, and anything else unexpected
        esac
        boot_lines+=("$line")
        if [[ "$idx" == "-1" ]]; then
          prev_id="$id"
          prev_range="$rest"
        fi
      done <<< "$CAP"
      boots="${#boot_lines[@]}"
    fi
  fi
  power_sample
  throttle_decode "${CUR[thr]}"
  log "BOOT previous=$verdict boots-in-journal=$boots boot=$BOOT_ID" \
    "uptime=${uptime}s prev-boot=$prev_id prev-last-entry=\"$prev_range\"" \
    "${marker_iso:+clean-stop-was=$marker_iso }thr=${CUR[thr]} now=$THR_NOW" \
    "since-boot=$THR_STICKY uv=${CUR[uv]} volt=${CUR[volt]} temp=${CUR[temp]}"
  SNAP_LINES=0
  SNAP_PREFIX="boot"
  log "boot report begin boot=$BOOT_ID"
  case "$verdict" in
    previous-boot-ended-WITHOUT-a-clean-stop)
      snap_emit verdict <<< "No clean-stop marker in $BOOT_MARKER. The previous boot did not stop this unit, so it was not shut down: it was reset, power-cycled, or it lost power. A kernel cannot log its own power loss, which is why this is recorded from the other side."
      ;;
    previous-shutdown-was-CLEAN)
      snap_emit verdict <<< "Clean-stop marker found for boot $marker_boot, written at ${marker_iso:-unknown}. The previous boot stopped this unit through systemd, so it was shut down or rebooted on purpose."
      ;;
    *)
      snap_emit verdict <<< "Clean-stop marker found for THIS boot ($BOOT_ID), written at ${marker_iso:-unknown}: the unit was restarted inside one boot, not the board. This says nothing about how the previous boot ended."
      ;;
  esac
  snap_emit this <<< "boot_id=$BOOT_ID uptime=${uptime}s state_dir=$AQUA_NETWATCH_STATE_DIR"
  snap_emit previous <<< "boot_id=$prev_id last-entry=$prev_range"
  # Verbatim, newest last. Twenty boots a few seconds long, six seconds apart, is
  # the signature of a supply that collapses and retries, and in this block it is
  # one thing to look at instead of a reconstruction.
  if [[ "${#boot_lines[@]}" -gt 0 ]]; then
    local start=0
    [[ "${#boot_lines[@]}" -gt "$AQUA_NETWATCH_BOOT_LIST" ]] \
      && start="$((${#boot_lines[@]} - AQUA_NETWATCH_BOOT_LIST))"
    for line in "${boot_lines[@]:start}"; do
      snap_emit boots <<< "$line"
    done
  else
    snap_emit boots <<< "(journalctl --list-boots said nothing; is the journal persistent?)"
  fi
  snap_emit power <<< "thr=${CUR[thr]} now=$THR_NOW since-boot=$THR_STICKY uv=${CUR[uv]} volt=${CUR[volt]} temp=${CUR[temp]}"
  snap_emit power <<< "(read at startup, and the since-boot bits latch within THIS boot only: a clean reading here says nothing about the boot that ended, because the reset that ended it cleared them. That is the gap this unit exists to close.)"
  module_state
  netdev_list
  local present=no
  [[ -d "$NETDIR" ]] && present=yes
  snap_emit radio <<< "iface=$IFACE present=$present netdevs=$NETDEVS module=$AQUA_NETWATCH_DRIVER_MODULE:$MODSTATE"
  snap_boot_radio
  log "boot report end boot=$BOOT_ID lines=$SNAP_LINES"
  SNAP_PREFIX="snapshot"
  if [[ "$clear" -eq 1 && -e "$BOOT_MARKER" ]]; then
    rm -f "$BOOT_MARKER" 2> /dev/null \
      || log "WARNING: could not clear $BOOT_MARKER; the next start may report a clean stop that was not one"
  fi
  return 0
}

# --- the state machine ---------------------------------------------------------
#
# A change is a change in the DIGEST, which is COMP_NAMES above and nothing else.
# Signal, core voltage, temperature, the packet counters and the exact neighbour
# state all move on every sample of a perfectly healthy board and are
# deliberately not in it: a snapshot per sample is not a diagnosis, it is a full
# journal. The neighbour state enters the digest coarsely -- an entry with a MAC,
# or not -- because REACHABLE/STALE/DELAY/PROBE cycling is what a healthy idle
# link does. The throttle flag word and the undervoltage bit ARE in it, because
# they do not move on a healthy board at all, and the moment one of them does is
# the moment this whole unit was written for.
decide() {
  local i name layer old new changed=0 silent_now=0 held="" layers_text="" first=""
  SNAP_THIS_SAMPLE=0
  local -a moved=()
  local -A layer_moved=()

  for i in "${!COMP_NAMES[@]}"; do
    name="${COMP_NAMES[$i]}"
    layer="${COMP_LAYER[$i]}"
    old="${PREV[$name]:-}"
    new="${CUR[$name]}"
    [[ -n "$old" ]] || continue
    [[ "$old" != "$new" ]] || continue
    changed=1
    # "-" is this script's word for "absent" everywhere else, but inside a
    # transition it collides with the arrow: "op=-->up" is a worse thing to read
    # at two in the morning than "op=none->up".
    [[ "$old" == "-" ]] && old="none"
    [[ "$new" == "-" ]] && new="none"
    moved+=("$name=${old}->${new}")
    if [[ -z "${layer_moved[$layer]:-}" ]]; then
      layer_moved[$layer]=1
    fi
  done

  # Only a layer that is 0 is a fault; an "na" layer above an absent interface is
  # not. Computed here because what gets printed depends on it.
  local bad=0
  for layer in "${LAYERS[@]}"; do
    [[ "${OK[$layer]}" == "0" ]] && bad=1
  done

  # A live throttle condition -- undervoltage, capping, throttling or the soft
  # temperature limit happening right now, the low nibble of the flag word -- is
  # worth every sample it lasts, whatever the idle cadence says.
  local live=0
  if [[ "${CUR[thr]}" =~ ^0x[0-9A-Fa-f]{1,8}$ ]] && ((CUR[thr] & 0xF)); then
    live=1
  fi
  [[ "${CUR[uv]}" == "1" ]] && live=1

  # The sample line first, then what moved in it: every sample while anything is
  # moving or anything is bad, one in AQUA_NETWATCH_IDLE_EVERY while the board
  # is simply fine.
  SINCE_PRINT=$((SINCE_PRINT + 1))
  if [[ "$changed" -eq 1 || "$bad" -eq 1 || "$live" -eq 1 \
    || "$SINCE_PRINT" -ge "$AQUA_NETWATCH_IDLE_EVERY" ]]; then
    sample_line
    SINCE_PRINT=0
  fi

  # held= is the point of the whole file: the layers that are still good and did
  # not move while something else did. "wpa=COMPLETED->DISCONNECTED ...
  # held=v4,rt,gw" is the association dropping under a healthy address, route
  # and gateway, which is one of the five diagnoses, read off one line.
  if [[ "$changed" -eq 1 ]]; then
    for layer in "${LAYERS[@]}"; do
      [[ -z "${layer_moved[$layer]:-}" ]] || continue
      [[ "${OK[$layer]}" == "1" ]] || continue
      held+="${layer},"
    done
    for layer in "${LAYERS[@]}" "${EXTRA_LAYERS[@]}"; do
      [[ -n "${layer_moved[$layer]:-}" ]] && layers_text+="${layer},"
    done
    held="${held%,}"
    log "CHANGE n=$SEQ layers=${layers_text%,} ${moved[*]} held=${held:-none}"
  fi

  # The rail moving is a transition in its own right, with a full snapshot
  # behind it, and it is deliberately reported BEFORE the network blocks below:
  # if a sag took the link with it, the sag is the cause and the snapshot should
  # say so on its own first line. It is not a rung of the ladder, so it never
  # puts the board into the degraded state and never slows the cadence -- the
  # latched "has occurred" bits would otherwise keep a board that sagged once
  # degraded until its next reset.
  if [[ -n "${layer_moved[power]:-}" ]]; then
    throttle_decode "${CUR[thr]}"
    log "POWER n=$SEQ thr=${PREV[thr]:-?}->${CUR[thr]} now=$THR_NOW" \
      "since-boot=$THR_STICKY uv=${PREV[uv]:-?}->${CUR[uv]} volt=${CUR[volt]}" \
      "temp=${CUR[temp]} (the latched bits are cleared by a reset, so this is" \
      "this boot only)"
    want_snapshot full "POWER:thr=${PREV[thr]:-?}->${CUR[thr]}"
  fi

  # Entering and leaving the degraded state, and the ordering inside it.
  now_s
  if [[ "$bad" -eq 1 ]]; then
    for layer in "${LAYERS[@]}"; do
      if [[ "${OK[$layer]}" == "0" && -z "${LOST_AT[$layer]:-}" ]]; then
        LOST_AT[$layer]="$NOW"
        unset "BACK_AT[$layer]"
        first+="${layer},"
        OUTAGE_LOST+="${layer},"
      fi
    done
    if [[ "$DEGRADED" -eq 0 ]]; then
      DEGRADED=1
      OUTAGE_SINCE="$NOW"
      OUTAGE_SAMPLES=0
      OUTAGE_SNAPS=0
      OUTAGE_BSS="${PREV[bss]:-${CUR[bss]}}"
      OUTAGE_FQ="${PREV[fq]:-${CUR[fq]}}"
      SILENT_RUN=0
      SILENT_SAID=0
      log "DEGRADED n=$SEQ first=${first%,} dev=${CUR[dev]} op=${CUR[op]}" \
        "wpa=${CUR[wpa]} bss=${CUR[bss]} v4=${CUR[v4]} rt=${CUR[rt]} nud=${CUR[nud]}"
      if [[ "${CUR[dev]}" == "absent" ]]; then
        absent_notice "$NOW" 1
      fi
      want_snapshot full "DEGRADED:${first%,}"
    elif [[ -n "$first" ]]; then
      log "DEGRADED n=$SEQ also=${first%,} (down $(duration_text "$((NOW - OUTAGE_SINCE))"))"
      want_snapshot brief "DEGRADED:also:${first%,}"
    elif [[ "${CUR[dev]}" == "absent" ]]; then
      absent_notice "$NOW" 0
    fi
    OUTAGE_SAMPLES=$((OUTAGE_SAMPLES + 1))
  else
    for layer in "${LAYERS[@]}"; do
      if [[ -n "${LOST_AT[$layer]:-}" && -z "${BACK_AT[$layer]:-}" ]]; then
        BACK_AT[$layer]="$NOW"
        OUTAGE_BACK+="${layer},"
      fi
    done
    if [[ "$DEGRADED" -eq 1 ]]; then
      DEGRADED=0
      local bss_text="same"
      if [[ "$OUTAGE_BSS" != "${CUR[bss]}" ]]; then
        bss_text="changed(${OUTAGE_BSS}->${CUR[bss]})"
      fi
      local fq_text=""
      [[ "$OUTAGE_FQ" != "${CUR[fq]}" ]] && fq_text=" fq=${OUTAGE_FQ}->${CUR[fq]}"
      log "RECOVERED n=$SEQ after=$(duration_text "$((NOW - OUTAGE_SINCE))")" \
        "lost=${OUTAGE_LOST%,} back=${OUTAGE_BACK%,} bssid=${bss_text}${fq_text}" \
        "samples=$OUTAGE_SAMPLES snapshots=$OUTAGE_SNAPS"
      want_snapshot full "RECOVERED"
      OUTAGE_LOST=""
      OUTAGE_BACK=""
      LOST_AT=()
      BACK_AT=()
      LAST_LOUD=0
      FW_VERDICT=""
      FW_LINE=""
      SILENT_RUN=0
      SILENT_SAID=0
    fi
  fi

  # A roam: the BSSID moved and the association never dropped. Newly possible
  # since 2026-10-05, when a second access point on the same SSID appeared, and
  # worth its own line because it is the one event that is invisible in every
  # other field.
  if [[ -n "${PREV[bss]:-}" && "${PREV[bss]}" != "-" && "${CUR[bss]}" != "-" \
    && "${PREV[bss]}" != "${CUR[bss]}" && "${OK[assoc]}" == "1" && "$bad" -eq 0 ]]; then
    log "ROAM n=$SEQ bss=${PREV[bss]}->${CUR[bss]} fq=${PREV[fq]:-?}->${CUR[fq]}" \
      "sig=${PREV[sig]:-?}->${CUR[sig]} ssid=${CUR[ssid]} (association never dropped)"
    want_snapshot full "ROAM"
  fi

  # Associated, addressed, routed, gateway answering -- and nothing arriving.
  # All five layers call this healthy, which is why it needs a check of its own.
  if [[ "$bad" -eq 0 && -n "${CNT[rx_packets]}" && -n "${PCNT[rx_packets]:-}" ]]; then
    if [[ "$((CNT[rx_packets] - PCNT[rx_packets]))" -le 0 ]]; then
      SILENT_RUN=$((SILENT_RUN + 1))
      if [[ "$SILENT_RUN" -ge "$AQUA_NETWATCH_SILENT_SAMPLES" && "$SILENT_SAID" -eq 0 ]]; then
        SILENT_SAID=1
        silent_now=1
      fi
    else
      SILENT_RUN=0
      SILENT_SAID=0
    fi
  fi
  if [[ "$silent_now" -eq 1 ]]; then
    log "SILENT n=$SEQ rx has not moved for $SILENT_RUN samples while every layer" \
      "reads up (op=${CUR[op]} wpa=${CUR[wpa]} bss=${CUR[bss]} v4=${CUR[v4]}" \
      "rt=${CUR[rt]} nud=${CUR[nud]}): the link is associated and carrying nothing"
    want_snapshot full "SILENT"
  fi

  # A bare change -- one that did not also enter or leave an outage, roam or go
  # silent -- gets the brief dump. Those four take the full one themselves, and
  # taking a brief one as well would mean two dumps of one event.
  if [[ "$changed" -eq 1 && "$SNAP_THIS_SAMPLE" -eq 0 ]]; then
    want_snapshot brief "CHANGE:${moved[*]}"
  fi

  remember
}

# This sample becomes the previous one. fq and sig are remembered although they
# are not in the digest, because the ROAM line reports them as old->new.
remember() {
  local k
  for k in "${COMP_NAMES[@]}"; do
    PREV[$k]="${CUR[$k]}"
  done
  PREV[fq]="${CUR[fq]}"
  PREV[sig]="${CUR[sig]}"
  for k in "${!CNT[@]}"; do
    PCNT[$k]="${CNT[$k]}"
  done
}

if [[ "$MODE" == "mark" ]]; then
  mark_clean_stop
  exit 0
fi
if [[ "$MODE" == "report" ]]; then
  # By hand: report and leave the marker alone, so running this does not consume
  # the one piece of evidence the next real start needs.
  boot_report 0
  exit 0
fi

STOPPING=0
on_term() {
  STOPPING=1
  log "stopping after $SEQ samples (nothing on this board was changed; this unit" \
    "only ever read). aqua-net-watch.service's ExecStop= records the clean stop" \
    "separately, which is what the next start reads."
  exit 0
}
trap on_term TERM INT

log "started: watching $IFACE every ${AQUA_NETWATCH_INTERVAL_S}s" \
  "(${AQUA_NETWATCH_OUTAGE_INTERVAL_S}s while degraded), one line per" \
  "${AQUA_NETWATCH_IDLE_EVERY} samples while healthy, snapshot floors" \
  "${AQUA_NETWATCH_SNAPSHOT_MIN_S}s/${AQUA_NETWATCH_FULL_MIN_S}s." \
  "It observes only: no packet is sent, no module loaded, no service touched." \
  "The only file it ever writes is the" \
  "clean-stop marker, at stop. The questions it answers are which of" \
  "iface/assoc/v4/rt/gw fails first, what the rail was doing at the time, and" \
  "whether the previous boot ended cleanly."

# Before the first sample, because it is about the boot and not about the link,
# and because a board that is going to die again should have said this first.
boot_report 1

# The first sample stands on its own: it has no predecessor to compare against,
# so it prints its line, takes its baseline snapshot, and -- if the board is
# already broken when the unit starts, which is the normal case after a boot
# that produced no interface -- says so at once rather than waiting for a change
# that already happened.
sample
sample_line
SINCE_PRINT=0
now_s
for layer in "${LAYERS[@]}"; do
  if [[ "${OK[$layer]}" == "0" ]]; then
    DEGRADED=1
    LOST_AT[$layer]="$NOW"
    OUTAGE_LOST+="${layer},"
  fi
done
if [[ "$DEGRADED" -eq 1 ]]; then
  OUTAGE_SINCE="$NOW"
  OUTAGE_BSS="${CUR[bss]}"
  OUTAGE_FQ="${CUR[fq]}"
  log "DEGRADED n=$SEQ first=${OUTAGE_LOST%,} (already so at the first sample, so" \
    "whatever ordering there was happened before this unit started; the snapshot" \
    "below reads this boot's kernel log from the top, which is where a" \
    "boot-time verdict is)"
  if [[ "${CUR[dev]}" == "absent" ]]; then
    absent_notice "$NOW" 1
  fi
fi
want_snapshot full "start"
remember

while [[ "$STOPPING" -eq 0 ]]; do
  if [[ "$SAMPLES_WANTED" -ne 0 && "$SEQ" -ge "$SAMPLES_WANTED" ]]; then
    break
  fi
  if [[ "$DEGRADED" -eq 1 ]]; then
    sleep "$AQUA_NETWATCH_OUTAGE_INTERVAL_S"
  else
    sleep "$AQUA_NETWATCH_INTERVAL_S"
  fi
  sample
  decide
done
