#!/usr/bin/env bash
# Idempotent provisioning for a Raspberry Pi running aqua-bridge
# (PROJECT.md §10, steps 8-9). Does NOT enable or start the service:
# the USB spike (Quadro writable via XT6? does XT6 revert without the
# daemon writing?) must be confirmed manually first.
#
# Usage:
#   deploy/install-pi.sh --user <account> [--tls-days <days>] [--das]
#
# --tls-days: validity of the self-signed HTTPS certificate created when
# /etc/aqua-bridge/tls/ has none (default 3650). Existing TLS files and the
# HTTP credentials file are never overwritten; users are not created (the
# script prints the tools/http_user.py command).
#
# --das: install config.example-das.yaml instead of config.example.yaml
# (still only when /etc/aqua-bridge/config.yaml is absent; an existing config
# is never overwritten either way), and install a systemd drop-in
# (deploy/aqua-bridge-das.conf -> aqua-bridge.service.d/das.conf) that adds
# --source hwmon to the unit's ExecStart= (PROJECT.md section 10). Without
# --das the unit runs --source xt6 as before, matching the legacy behaviour
# bit for bit. Either way the service is only installed and verified, never
# enabled or started.
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
DAS_DROPIN_SRC="$SCRIPT_DIR/aqua-bridge-das.conf"
DROPIN_DIR="$UNIT_DST.d"
DAS_DROPIN_DST="$DROPIN_DIR/das.conf"
PACKAGES_FILE="$SCRIPT_DIR/packages-rpi.txt"
# Must match the http: defaults in src/aqua_bridge/publishers/httpauth.py
# (HttpSettings; tests/test_deploy.py checks it).
TLS_DIR="$CONFIG_DIR/tls"
TLS_CERT="$TLS_DIR/cert.pem"
TLS_KEY="$TLS_DIR/key.pem"
TLS_DAYS=3650

USER_ACCOUNT=""
DAS_MODE=0

usage() {
  echo "Usage: $0 --user <account> [--tls-days <days>] [--das]" >&2
  exit 2
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --user)
      [[ $# -ge 2 ]] || usage
      USER_ACCOUNT="$2"
      shift 2
      ;;
    --tls-days)
      [[ $# -ge 2 ]] || usage
      [[ "$2" =~ ^[1-9][0-9]*$ ]] || usage
      TLS_DAYS="$2"
      shift 2
      ;;
    --das)
      DAS_MODE=1
      shift
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
  if [[ "$DAS_MODE" -eq 1 ]]; then
    CONFIG_EXAMPLE="$REPO_DIR/config.example-das.yaml"
  else
    CONFIG_EXAMPLE="$REPO_DIR/config.example.yaml"
  fi
  sudo install -m 640 -o root -g "$USER_ACCOUNT" \
    "$CONFIG_EXAMPLE" "$CONFIG_DIR/config.yaml"
  echo "installed: $CONFIG_DIR/config.yaml (edit host, credentials, channel names)"
else
  echo "already present, left untouched: $CONFIG_DIR/config.yaml"
fi

echo "== HTTPS certificate =="
sudo install -d -m 755 "$TLS_DIR"
if sudo test -e "$TLS_CERT" || sudo test -e "$TLS_KEY"; then
  # Never overwrite: an owner-provided certificate or a previous run's key stays.
  if ! sudo test -e "$TLS_CERT" || ! sudo test -e "$TLS_KEY"; then
    echo "warning: only one of $TLS_CERT / $TLS_KEY exists; left untouched," \
      "the HTTPS API will not start until both are present" >&2
  else
    echo "already present, left untouched: $TLS_CERT, $TLS_KEY"
  fi
else
  tls_host="$(hostname)"
  # umask 077 in the subshell: the key is never world-readable, not even briefly.
  (
    umask 077
    sudo openssl req -x509 -newkey ec -pkeyopt ec_paramgen_curve:prime256v1 -nodes \
      -days "$TLS_DAYS" -subj "/CN=$tls_host" \
      -addext "subjectAltName=DNS:$tls_host,DNS:$tls_host.local" \
      -keyout "$TLS_KEY" -out "$TLS_CERT"
  )
  sudo chown root:"$USER_ACCOUNT" "$TLS_KEY"
  sudo chmod 640 "$TLS_KEY"
  sudo chown root:root "$TLS_CERT"
  sudo chmod 644 "$TLS_CERT"
  echo "created self-signed certificate: $TLS_CERT (key $TLS_KEY, $TLS_DAYS days)"
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
if [[ "$DAS_MODE" -eq 1 ]]; then
  # --source hwmon (the DAS composite hwmon: + onewire: source) instead of
  # the base unit's implicit --source xt6; PROJECT.md section 10.
  sudo install -d -m 755 "$DROPIN_DIR"
  sudo install -m 644 "$DAS_DROPIN_SRC" "$DAS_DROPIN_DST"
  echo "installed drop-in: $DAS_DROPIN_DST (--source hwmon)"
fi
sudo systemctl daemon-reload
# systemd-analyze verify resolves the unit's drop-in directory the same way
# systemd itself does, so a --das install is checked with das.conf merged in.
sudo systemd-analyze verify "$UNIT_DST"

DAS_NOTE=""
if [[ "$DAS_MODE" -eq 1 ]]; then
  DAS_NOTE="  DAS mode: $DAS_DROPIN_DST makes the unit run --source hwmon;
     fill in hwmon: / onewire: in $CONFIG_DIR/config.yaml before enabling."
fi

cat <<EOF

== done ==
Provisioning complete. The service is installed but NOT enabled or
started. Before "systemctl enable --now aqua-bridge":
  1. Confirm the USB spike (PROJECT.md §2, §13): lsusb, sensors, pwm
     list; is the Quadro writable via the XT6; does the XT6 revert to
     its own curve once the Pi stops writing.
  2. Edit $CONFIG_DIR/config.yaml (host, MQTT credentials, channel
     names) to match this Pi.
$DAS_NOTE
  3. For the HTTPS API (http.enabled: true), create a user; the password
     is prompted for, never passed on the command line:
       sudo $INSTALL_DIR/.venv/bin/python $INSTALL_DIR/tools/http_user.py \\
         --config $CONFIG_DIR/config.yaml --group $USER_ACCOUNT <name>
     A certificate this script created is self-signed: browsers warn
     until you trust $TLS_CERT.
  4. Then: sudo systemctl enable --now aqua-bridge
EOF
