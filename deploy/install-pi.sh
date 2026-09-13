#!/usr/bin/env bash
# Idempotent provisioning for a Raspberry Pi running aqua-bridge
# (PROJECT.md §10, steps 8-9). Does NOT enable or start the service:
# the USB spike (Quadro writable via XT6? does XT6 revert without the
# daemon writing?) must be confirmed manually first.
#
# Usage:
#   deploy/install-pi.sh --user <account>
#
# Run this from the checked-out repo on the Pi (e.g. after
# `rsync -az ./ USER@PI-HOST:/opt/aqua-bridge/`, see PROJECT.md §11) with
# the code already present at /opt/aqua-bridge, or run it once to create
# /opt/aqua-bridge and rsync/clone the code into it afterwards, then
# re-run to install the venv, config, udev rule and unit.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
INSTALL_DIR="/opt/aqua-bridge"
CONFIG_DIR="/etc/aqua-bridge"
UDEV_RULE_SRC="$SCRIPT_DIR/99-aquacomputer.rules"
UDEV_RULE_DST="/etc/udev/rules.d/99-aquacomputer.rules"
UNIT_SRC="$SCRIPT_DIR/aqua-bridge.service"
UNIT_DST="/etc/systemd/system/aqua-bridge.service"
PACKAGES_FILE="$SCRIPT_DIR/packages-rpi.txt"

USER_ACCOUNT=""

usage() {
  echo "Usage: $0 --user <account>" >&2
  exit 2
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --user)
      [[ $# -ge 2 ]] || usage
      USER_ACCOUNT="$2"
      shift 2
      ;;
    *)
      usage
      ;;
  esac
done

if [[ -z "$USER_ACCOUNT" ]]; then
  usage
fi

if ! id "$USER_ACCOUNT" > /dev/null 2>&1; then
  echo "error: user '$USER_ACCOUNT' does not exist (create it first, see PROJECT.md §9)" >&2
  exit 1
fi

echo "== apt packages =="
if [[ -f "$PACKAGES_FILE" ]]; then
  # shellcheck disable=SC2046
  sudo apt-get update
  # shellcheck disable=SC2046
  sudo apt-get install -y $(grep -vE '^#|^$' "$PACKAGES_FILE" | xargs)
else
  echo "error: $PACKAGES_FILE not found" >&2
  exit 1
fi

echo "== install directory =="
sudo install -d -o "$USER_ACCOUNT" -g "$USER_ACCOUNT" "$INSTALL_DIR"

echo "== venv =="
if [[ ! -x "$INSTALL_DIR/.venv/bin/python" ]]; then
  sudo -u "$USER_ACCOUNT" python3 -m venv --system-site-packages "$INSTALL_DIR/.venv"
else
  echo "venv already present: $INSTALL_DIR/.venv"
fi

echo "== project install =="
if [[ -f "$INSTALL_DIR/pyproject.toml" ]]; then
  pip_log="$(mktemp)"
  # --no-deps: numpy/pyyaml/aiohttp/paho-mqtt come from apt via
  # --system-site-packages (PROJECT.md §9); a forgotten --no-deps could
  # try to build numpy from source on the Zero W.
  # SC2024: intentional. The log is a mktemp file owned by the invoking user,
  # so the redirect must stay outside sudo; only pip runs as the service user.
  # shellcheck disable=SC2024
  if ! sudo -u "$USER_ACCOUNT" "$INSTALL_DIR/.venv/bin/pip" install -e "$INSTALL_DIR" --no-deps \
      > "$pip_log" 2>&1; then
    cat "$pip_log"
    rm -f "$pip_log"
    echo "error: pip install -e failed" >&2
    exit 1
  fi
  if grep -qiE 'Downloading numpy|Building wheel for numpy' "$pip_log"; then
    cat "$pip_log"
    rm -f "$pip_log"
    echo "error: pip tried to fetch/build numpy from PyPI instead of using apt's" \
      "python3-numpy; fix the pyproject.toml version ranges (PROJECT.md §9)" >&2
    exit 1
  fi
  cat "$pip_log"
  rm -f "$pip_log"
else
  echo "warning: $INSTALL_DIR/pyproject.toml not found yet;" \
    "rsync/clone the code into $INSTALL_DIR and re-run this script" >&2
fi

echo "== config =="
sudo install -d -m 755 "$CONFIG_DIR"
if [[ ! -f "$CONFIG_DIR/config.yaml" ]]; then
  sudo install -m 640 -o root -g "$USER_ACCOUNT" \
    "$REPO_DIR/config.example.yaml" "$CONFIG_DIR/config.yaml"
  echo "installed: $CONFIG_DIR/config.yaml (edit host, credentials, channel names)"
else
  echo "already present, left untouched: $CONFIG_DIR/config.yaml"
fi

echo "== udev rule =="
sudo install -m 644 "$UDEV_RULE_SRC" "$UDEV_RULE_DST"
sudo udevadm control --reload-rules
# Re-run the rules for a device that is already attached: the hwmon rule
# (group write on pwmK) only fires on an add event.
sudo udevadm trigger --action=add --subsystem-match=hwmon
sudo udevadm trigger --action=add --subsystem-match=usb --attr-match=idVendor=0c70
sudo udevadm settle || true

echo "== systemd unit =="
sudo sed -e "s/^User=.*/User=$USER_ACCOUNT/" "$UNIT_SRC" | sudo tee "$UNIT_DST" > /dev/null
sudo systemctl daemon-reload
sudo systemd-analyze verify "$UNIT_DST"

cat <<EOF

== done ==
Provisioning complete. The service is installed but NOT enabled or
started. Before "systemctl enable --now aqua-bridge":
  1. Confirm the USB spike (PROJECT.md §2, §13): lsusb, sensors, pwm
     list; is the Quadro writable via the XT6; does the XT6 revert to
     its own curve once the Pi stops writing.
  2. Edit $CONFIG_DIR/config.yaml (host, MQTT credentials, channel
     names) to match this Pi.
  3. Then: sudo systemctl enable --now aqua-bridge
EOF
