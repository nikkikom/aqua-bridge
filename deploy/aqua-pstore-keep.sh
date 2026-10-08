#!/usr/bin/env bash
# Keep every pstore record this boot found, before systemd-pstore moves it to
# the one path it always uses and the next failed boot overwrites it.
# PROJECT.md §2, "Watchdog layering"; §9.
#
# Why it exists. The board dies without logging anything: the journal stops
# mid-stream on a routine line, with no service shutdown, no shutdown target, no
# panic, no OOM and no watchdog expiry -- and then a run of boots that each die
# about eleven seconds in (PROJECT.md §9, the network watcher's rail section).
# A kernel cannot log its own death to a journal on an SD card, so the console is
# preserved in DRAM across the reset instead, by ramoops:
#
#   dtoverlay=ramoops,total-size=0x80000,record-size=0x8000,console-size=0x40000
#
# in /boot/firmware/config.txt. That part works. What destroyed the evidence
# twice is what happens NEXT. The reset is followed by one or more boots that
# die about eleven seconds in, and on this board systemd-pstore archived the
# console record to one fixed path -- /var/lib/systemd/pstore/console-ramoops-0
# -- so each of those near-empty records overwrote the record from the boot that
# actually failed. Measured, twice: a 25930-byte record and then a 191-byte
# record, both from eleven-second boots, each one replacing what was there. The
# single most valuable piece of evidence this board can produce was collected
# correctly by the kernel and then thrown away by the archiver, at boot, before
# anybody could look.
#
# So: copy first, archive second. This script runs once per boot, BEFORE
# systemd-pstore, and writes every record it finds to a name that no later boot
# can collide with -- a UTC stamp, this boot's id, and the record's own name. It
# never unlinks anything in /sys/fs/pstore, so systemd-pstore still does its job
# afterwards and its own archive is unaffected; this is a second copy, not a
# replacement.
#
# Timing is load-bearing. systemd-pstore ran at about 10.45 s in the boots that
# died at about 10.5 to 11.1 s, which is how close the archiver was to the death
# itself: a keeper that ran after it, or slowly, would be racing the very fault
# it is collecting. So this unit is ordered Before=systemd-pstore.service, runs
# at Nice=-5 (the only negative nice in this repository), and has exactly one
# thing to do -- copy at most AQUA_PSTORE_MAX_TOTAL_HINT of records, which is
# what the ramoops total-size above can hold. It is bounded by its unit's
# TimeoutStartSec=20 rather than by trust.
#
# Nothing here is in the cooling path (PROJECT.md §2). It runs once, at boot,
# inside sysinit; the daemon starts later, from multi-user.target. The worst case
# it can add to that is its own start timeout, and a board takes at least 28 s
# from power to its first controller write anyway -- which is already longer than
# the aquaero's own software-sensor timeout, so the alarm profile covers the
# window whatever happens here. It opens no controller, writes to no device,
# touches no service and sends no packet.
#
# Usage:
#   deploy/aqua-pstore-keep.sh            # keep what is there (what the unit runs)
#   deploy/aqua-pstore-keep.sh --check    # report only, copy and delete nothing
#
# --check is the safe thing to run by hand on a live board: it prints what is in
# /sys/fs/pstore, what has already been kept, what a real run WOULD copy and what
# it WOULD prune, and it writes and removes nothing.
#
# ---------------------------------------------------------------------------
# How to read it
# ---------------------------------------------------------------------------
# One journal identifier, so every boot's answer -- including the boots that had
# nothing to keep -- is one command:
#
#   journalctl -t aqua-pstore-keep -o short-iso --since -7d
#
# and the records themselves are files, newest last:
#
#   ls -l /var/lib/aqua-pstore
#   less /var/lib/aqua-pstore/<stamp>-<boot>-console-ramoops-0
#
# Line kinds:
#   started   the directories, the bounds, and how many records are already held.
#   KEPT      one per record copied: its name, the bytes written, the destination.
#             This is the line that says the evidence survived.
#   NOTHING   /sys/fs/pstore held no record. On a healthy board the unit does not
#             even run (ConditionDirectoryNotEmpty=), so this line is what a hand
#             run and a race against systemd-pstore look like -- and it names the
#             likely reason when the directory is not there at all, because
#             "ramoops is not enabled" and "the board did not crash" produce the
#             same empty directory and want very different responses.
#   SKIPPED   a destination that already exists. Nothing is ever overwritten.
#   PRUNED    one per copy removed by age or by the count bound, with its age.
#   WARNING   a copy that failed, naming the record and what was left behind.
#   done      kept, pruned, held, and the bytes.
#
# ---------------------------------------------------------------------------
# Knobs (environment; defaults below). aqua-pstore-keep.service passes the two
# bounds -- deploy/install-board-watchdogs.sh writes them from PSTORE_KEEP_DAYS
# and PSTORE_KEEP_MAX -- and none of the others, so for anything else an override
# is "systemctl edit aqua-pstore-keep.service".
# ---------------------------------------------------------------------------
#
# Deliberately NOT "set -e". This runs once per boot, on the boot path, and the
# one thing it may never do is abandon the remaining records because the first
# one could not be read. Every command whose result matters is checked where it
# is called; -u and pipefail stay on to catch the mistakes that are mistakes.
set -uo pipefail

# Where the kernel exposes the records it preserved across the reset. Reading it
# is all this script does to /sys: it never unlinks a record, which is
# systemd-pstore's job and is what makes this a second copy rather than a
# competing archiver. The unit sets ProtectKernelTunables=yes, so the whole of
# /sys is read-only to it and that promise is enforced by the kernel and not only
# by this line.
: "${AQUA_PSTORE_SRC_DIR:=/sys/fs/pstore}"
# Where the copies go. It matches aqua-pstore-keep.service's StateDirectory=,
# which is also what makes this the one writable path under
# ProtectSystem=strict, and tests/test_deploy.py checks that the two agree. On
# /var and never /run: a crash record on a tmpfs would be erased by exactly the
# event it is evidence of.
: "${AQUA_PSTORE_KEEP_DIR:=/var/lib/aqua-pstore}"
# Days a copy is kept before the prune at the end of a run removes it. The
# directory has to be bounded by something, because a board in a reset loop
# produces a record per boot and the one thing worse than losing the evidence is
# filling the card the journal is also capped against (§9 "Board hardening": a
# reset loop must not outbid the history it is evidence for). 30 matches
# JOURNAL_MAX_RETENTION in deploy/install-board-watchdogs.sh, so a record and
# the journal lines around it age out together -- a record whose journal is gone
# is half a diagnosis. 0 means never prune by age.
: "${AQUA_PSTORE_KEEP_DAYS:=30}"
# Copies kept at most, newest first, whatever their age. This is the bound that
# holds when the clock does not: the age prune compares mtimes, and a board that
# resets every eleven seconds before timesyncd has run has whatever time systemd
# restored from its last known one. 200 at the ramoops total-size above is a
# worst case of about 100 MB and a typical case of a few MB (the records measured
# on this board were 191 and 25930 bytes), against a 15 GB card whose journal is
# capped at 1G. 0 means never prune by count.
: "${AQUA_PSTORE_KEEP_MAX:=200}"
# Where this boot's id is read from, for the middle field of each kept name. It
# is a path and not a value so that tests/test_deploy.py can run this script
# against a fabricated /proc -- the same seam, for the same reason, as
# aqua-net-watch.sh's AQUA_NETWATCH_ROOT. An unreadable boot id is never fatal:
# the name then says "noboot" and the copy still happens.
: "${AQUA_PSTORE_BOOT_ID_PATH:=/proc/sys/kernel/random/boot_id}"

CHECK_ONLY=0
while [[ "$#" -gt 0 ]]; do
  case "$1" in
    --check)
      CHECK_ONLY=1
      ;;
    *)
      echo "Usage: $0 [--check]" >&2
      exit 2
      ;;
  esac
  shift
done

for name in AQUA_PSTORE_KEEP_DAYS AQUA_PSTORE_KEEP_MAX; do
  if [[ ! "${!name}" =~ ^(0|[1-9][0-9]*)$ ]]; then
    echo "error: $name must be a whole number (0 = no bound), got '${!name}'" >&2
    exit 2
  fi
done

# ---------------------------------------------------------------------------
# The pruning, and the one thing it may never become
# ---------------------------------------------------------------------------
# A deletion target built out of a shell variable IS a deletion of the root the
# moment that variable is empty: "rm -rf $DIR/*" with $DIR unset expands to
# "rm -rf /*". The first version of this script, written by hand on the board,
# was caught doing exactly that by a safety check before it ever ran. Five things
# now stand between a knob and rm(1), and each of the first four holds on its own
# if the rest are wrong:
#
#   1. The knobs control the AGE and the COUNT, not the path.
#      AQUA_PSTORE_KEEP_DAYS and AQUA_PSTORE_KEEP_MAX are what an operator is
#      expected to turn, and neither of them can name a file.
#   2. An EMPTY knob never even reaches the path: ': "${X:=default}"'
#      substitutes the default when the variable is empty as well as when it is
#      unset, so "AQUA_PSTORE_KEEP_DIR=" is /var/lib/aqua-pstore and never a
#      bare "/". That alone is what the version written on the board was missing.
#   3. What is left is validated once, here, before anything is copied or
#      deleted, and refused outright unless it is absolute and at least two
#      components deep: "/", "/var", a relative path and anything containing
#      ".." each exit 2 with a message and remove nothing. A path that is wrong
#      is a refusal to run, never a deletion.
#   4. The path is never expanded into a glob and never handed to "rm -r".
#      EVERY deletion in this script is one "rm -f --" of one regular file whose
#      full path came out of find(1), and a path that came out of find cannot be
#      the empty string.
#   5. find is restricted to -maxdepth 1, -type f and $KEEP_GLOB -- this
#      script's own naming scheme -- so even a directory pointed somewhere it
#      should not be can only lose files this script could have written. It
#      cannot recurse, cannot follow a symlink out of the directory and cannot
#      touch a directory entry of any other shape.
KEEP_DIR="${AQUA_PSTORE_KEEP_DIR%/}"
case "$KEEP_DIR" in
  *..*)
    echo "error: AQUA_PSTORE_KEEP_DIR must not contain '..', got" \
      "'$AQUA_PSTORE_KEEP_DIR': this directory is pruned, and a prune target is" \
      "never built out of a path that can climb" >&2
    exit 2
    ;;
  /*/?*) ;;
  *)
    echo "error: AQUA_PSTORE_KEEP_DIR must be an absolute path at least two" \
      "components deep (the default is /var/lib/aqua-pstore), got" \
      "'$AQUA_PSTORE_KEEP_DIR'. Refused rather than pruned: an empty or" \
      "near-root path here is a deletion of the board." >&2
    exit 2
    ;;
esac
SRC_DIR="${AQUA_PSTORE_SRC_DIR%/}"
#: The naming scheme, as a glob, and the only thing the prune is allowed to
#: match: <8-digit date>T<6-digit time>Z-<boot>-<record name>, plus the
#: ".partial" a failed copy leaves behind. Not a knob -- it is the other half of
#: the name built in keep_one below, and the two have to agree or the prune stops
#: bounding anything. tests/test_deploy.py checks that they do.
KEEP_GLOB='????????T??????Z-*'

log() {
  printf '%s\n' "$*"
}

# This boot's id, shortened to eight hex digits, into $BOOT_ID. It is the field
# that makes two records from two boots distinguishable even when the clock says
# they happened at the same second -- which is the normal case on a board that
# resets before timesyncd has run, and the normal case is exactly the one that
# overwrote the evidence twice.
BOOT_ID="noboot"
boot_id() {
  local line
  [[ -r "$AQUA_PSTORE_BOOT_ID_PATH" ]] || return 0
  read -r line < "$AQUA_PSTORE_BOOT_ID_PATH" 2> /dev/null || return 0
  line="${line//-/}"
  line="${line//[^0-9a-fA-F]/}"
  [[ -n "$line" ]] || return 0
  BOOT_ID="${line:0:8}"
}

# The stamp every kept name starts with. UTC on purpose, and TZ is set rather
# than assumed: these names are read side by side with journal timestamps from
# several boots, and a local-time stamp that moves twice a year is a name you
# cannot sort. Fixed width, so the names sort as the clock does.
export TZ=UTC
STAMP=""
stamp() {
  printf -v STAMP '%(%Y%m%dT%H%M%SZ)T' -1
}

# The size of a file in bytes into $SIZE, or "?" -- never fatal, because a
# number nobody could read is not a reason to stop reporting what was kept.
SIZE="?"
size_of() {
  SIZE="$(stat -c %s -- "$1" 2> /dev/null)" || SIZE="?"
  [[ -n "$SIZE" ]] || SIZE="?"
}

KEPT=0
KEPT_BYTES=0
SKIPPED=0
PRUNED=0
FAILED=0

# keep_one <source record>. Copy it to a name no other boot can collide with,
# without ever unlinking or overwriting anything. The copy lands as
# "<name>.partial" and is renamed only once cp has succeeded, so a run that is
# killed part-way through -- by the very fault it is collecting -- leaves a file
# that says so in its name instead of a truncated record that does not.
keep_one() {
  local src="$1" name dest
  name="${src##*/}"
  # Into a filename, so anything that is not plainly a filename goes. pstore's
  # own names (dmesg-ramoops-0, console-ramoops-0, pmsg-ramoops-0) are untouched
  # by this; a backend that invents something else cannot smuggle a path
  # separator through it.
  name="${name//[^A-Za-z0-9._-]/_}"
  dest="$KEEP_DIR/${STAMP}-${BOOT_ID}-${name}"
  size_of "$src"
  if [[ -e "$dest" ]]; then
    SKIPPED=$((SKIPPED + 1))
    log "SKIPPED ${name} (${SIZE} bytes): $dest already exists and nothing here" \
      "ever overwrites a kept record. Two runs inside one second of one boot is" \
      "the only way to reach this, and the record is already kept."
    return 0
  fi
  if [[ "$CHECK_ONLY" -eq 1 ]]; then
    log "--check: would keep ${name} (${SIZE} bytes) -> $dest"
    return 0
  fi
  if ! cp -- "$src" "${dest}.partial" 2> /dev/null; then
    FAILED=$((FAILED + 1))
    log "WARNING could not copy ${name} (${SIZE} bytes) from $src;" \
      "${dest}.partial is whatever was written before it failed and is left" \
      "alone rather than removed. systemd-pstore will still archive the record" \
      "itself -- nothing here has unlinked it -- so the evidence is not lost," \
      "only this second copy of it."
    return 0
  fi
  if ! mv -- "${dest}.partial" "$dest" 2> /dev/null; then
    FAILED=$((FAILED + 1))
    log "WARNING kept ${name} (${SIZE} bytes) as ${dest}.partial but could not" \
      "rename it to $dest. The content is there; the name says it was not" \
      "finished."
    return 0
  fi
  size_of "$dest"
  KEPT=$((KEPT + 1))
  [[ "$SIZE" =~ ^[0-9]+$ ]] && KEPT_BYTES=$((KEPT_BYTES + SIZE))
  log "KEPT ${name} (${SIZE} bytes) -> $dest"
}

# Every record in $SRC_DIR, or one line saying there was none and why that is
# ambiguous. The directory being absent and the board never having crashed look
# identical from here and want opposite responses, so they are told apart.
keep_all() {
  local entry found=0
  if [[ ! -d "$SRC_DIR" ]]; then
    log "NOTHING $SRC_DIR does not exist, so there is no preserved record to" \
      "keep. That is either a kernel with no pstore backend registered or" \
      "ramoops not enabled: it is enabled with" \
      "'dtoverlay=ramoops,total-size=0x80000,record-size=0x8000,console-size=0x40000'" \
      "in /boot/firmware/config.txt and a reboot, and without it a board that" \
      "dies mid-journal leaves nothing at all behind (PROJECT.md §9)."
    return 0
  fi
  for entry in "$SRC_DIR"/*; do
    [[ -f "$entry" ]] || continue
    found=1
    keep_one "$entry"
  done
  if [[ "$found" -eq 0 ]]; then
    log "NOTHING $SRC_DIR is empty: this boot's kernel preserved no record, so" \
      "the previous boot ended without one -- a clean shutdown, or a death the" \
      "console never reached. The unit's ConditionDirectoryNotEmpty= means a" \
      "normal boot does not even run this, so this line is a hand run or a race" \
      "with systemd-pstore, which archives and unlinks the same files."
  fi
}

# prune. The age bound first, then the count bound, and every deletion is one
# "rm -f --" of one path that came out of find -- see the four-point argument
# above the KEEP_DIR validation, which is the whole reason this function looks
# the way it does and not like a one-line glob.
prune() {
  local path kept_paths=() line
  if [[ "$AQUA_PSTORE_KEEP_DAYS" -gt 0 ]]; then
    while IFS= read -r -d '' path; do
      [[ -n "$path" ]] || continue
      PRUNED=$((PRUNED + 1))
      if [[ "$CHECK_ONLY" -eq 1 ]]; then
        log "--check: would prune ${path##*/} (older than" \
          "${AQUA_PSTORE_KEEP_DAYS} days)"
        continue
      fi
      if rm -f -- "$path" 2> /dev/null; then
        log "PRUNED ${path##*/} (older than ${AQUA_PSTORE_KEEP_DAYS} days)"
      else
        PRUNED=$((PRUNED - 1))
        log "WARNING could not remove $path"
      fi
    done < <(find "$KEEP_DIR" -maxdepth 1 -type f -name "$KEEP_GLOB" \
      -mtime +"$AQUA_PSTORE_KEEP_DAYS" -print0 2> /dev/null)
  fi
  # The count bound, newest first by mtime -- not by name, because the name's
  # stamp is only as good as the clock was at boot, and this bound exists for
  # exactly the case where it was not good.
  if [[ "$AQUA_PSTORE_KEEP_MAX" -le 0 ]]; then
    return 0
  fi
  while IFS= read -r -d '' line; do
    [[ -n "$line" ]] || continue
    kept_paths+=("${line#*$'\t'}")
  done < <(find "$KEEP_DIR" -maxdepth 1 -type f -name "$KEEP_GLOB" \
    -printf '%T@\t%p\0' 2> /dev/null | sort -z -k1,1rn)
  local index=0
  for path in "${kept_paths[@]}"; do
    index=$((index + 1))
    [[ "$index" -gt "$AQUA_PSTORE_KEEP_MAX" ]] || continue
    [[ -n "$path" ]] || continue
    PRUNED=$((PRUNED + 1))
    if [[ "$CHECK_ONLY" -eq 1 ]]; then
      log "--check: would prune ${path##*/} (over the ${AQUA_PSTORE_KEEP_MAX}" \
        "newest)"
      continue
    fi
    if rm -f -- "$path" 2> /dev/null; then
      log "PRUNED ${path##*/} (over the ${AQUA_PSTORE_KEEP_MAX} newest)"
    else
      PRUNED=$((PRUNED - 1))
      log "WARNING could not remove $path"
    fi
  done
}

# How many kept records the directory holds, into $HELD.
HELD=0
held_count() {
  local path
  HELD=0
  for path in "$KEEP_DIR"/*; do
    [[ -f "$path" ]] || continue
    HELD=$((HELD + 1))
  done
}

boot_id
stamp
if [[ "$CHECK_ONLY" -eq 0 ]]; then
  # systemd's StateDirectory= has already made this, with the right mode; this
  # is for a hand run outside the unit, and a failure here is fatal because
  # there is nowhere to keep anything.
  if ! mkdir -p -- "$KEEP_DIR" 2> /dev/null; then
    echo "error: cannot create $KEEP_DIR; nothing was copied" >&2
    exit 1
  fi
fi
held_count
log "started: src=$SRC_DIR keep=$KEEP_DIR boot=$BOOT_ID stamp=$STAMP" \
  "held=$HELD bounds=${AQUA_PSTORE_KEEP_DAYS}d/${AQUA_PSTORE_KEEP_MAX}" \
  "(it copies and never unlinks a record; systemd-pstore archives them after" \
  "this and is unaffected)"
keep_all
prune
held_count
if [[ "$CHECK_ONLY" -eq 1 ]]; then
  log "--check: nothing was copied, written or removed. held=$HELD"
  exit 0
fi
log "done: kept=$KEPT bytes=$KEPT_BYTES skipped=$SKIPPED pruned=$PRUNED" \
  "failed=$FAILED held=$HELD"
exit 0
