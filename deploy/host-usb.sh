#!/usr/bin/env bash
# Idempotently enable the Pi's USB port as a host port (dwc2, dr_mode=host)
# so a powered hub + the aquaero 6 XT can be attached (PROJECT.md §2, §9).
#
# Usage:
#   deploy/host-usb.sh            # apply the change if missing
#   deploy/host-usb.sh --check    # report only, make no changes
set -euo pipefail

CONFIG_TXT="/boot/firmware/config.txt"
OVERLAY_LINE="dtoverlay=dwc2,dr_mode=host"
CHECK_ONLY=0

for arg in "$@"; do
  case "$arg" in
    --check)
      CHECK_ONLY=1
      ;;
    *)
      echo "Usage: $0 [--check]" >&2
      exit 2
      ;;
  esac
done

if [[ ! -f "$CONFIG_TXT" ]]; then
  echo "error: $CONFIG_TXT not found (not a Raspberry Pi OS bootfs?)" >&2
  exit 1
fi

# Already present anywhere in the file: nothing to do, no reboot needed.
if grep -qxF "$OVERLAY_LINE" "$CONFIG_TXT"; then
  echo "already present: $OVERLAY_LINE"
  echo "reboot needed: no"
  exit 0
fi

if [[ "$CHECK_ONLY" -eq 1 ]]; then
  echo "missing: $OVERLAY_LINE"
  echo "reboot needed: yes (not applied, --check mode)"
  exit 0
fi

timestamp="$(date +%Y%m%d%H%M%S)"
backup="${CONFIG_TXT}.bak.${timestamp}"
sudo cp -p "$CONFIG_TXT" "$backup"
echo "backup written: $backup"

if grep -qxF '[all]' "$CONFIG_TXT"; then
  # Append the overlay line right after the [all] section header.
  sudo awk -v line="$OVERLAY_LINE" '
    { print }
    $0 == "[all]" && !done { print line; done = 1 }
  ' "$CONFIG_TXT" | sudo tee "${CONFIG_TXT}.new" > /dev/null
  sudo mv "${CONFIG_TXT}.new" "$CONFIG_TXT"
else
  # No [all] section yet: add one at the end of the file.
  {
    echo ""
    echo "[all]"
    echo "$OVERLAY_LINE"
  } | sudo tee -a "$CONFIG_TXT" > /dev/null
fi

echo "added: $OVERLAY_LINE"
echo "reboot needed: yes"
