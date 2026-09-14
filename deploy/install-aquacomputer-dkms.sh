#!/usr/bin/env bash
# Build and install the aquacomputer_d5next hwmon driver with DKMS
# (PROJECT.md §9, "Kernel module aquacomputer_d5next (optional)").
#
# OPTIONAL. The daemon talks to the aquaero and the Quadro over hidraw and does
# not use this driver; deploy/install-pi.sh does not run this script. It is
# kept for experiments through hwmon and in case the driver path is revived.
# A loaded driver still leaves the hidraw nodes to the daemon, but reading a
# pwmN attribute issues a control report read of its own.
#
# Needs (not in deploy/packages-rpi.txt):
#   sudo apt-get install -y dkms patch curl linux-headers-rpi-v6 make gcc
# curl only when the source is downloaded (no --source).
#
# Raspberry Pi OS kernels are built without CONFIG_SENSORS_AQUACOMPUTER_D5NEXT,
# so the aquaero and the Quadro show up only as raw HID devices and there is
# no hwmon directory for the daemon. This script downloads the driver source
# of the matching kernel version from the kernel.org stable tree, applies the
# patches in deploy/dkms/aquacomputer_d5next/ (see the header of each) and
# installs the module with DKMS.
#
# Usage:
#   deploy/install-aquacomputer-dkms.sh [--kernel <release>] [--tag <vX.Y[.Z]>]
#                                       [--source <aquacomputer_d5next.c>]
#
# --kernel: kernel release to build for (default: the running one, uname -r).
#   Use it after a kernel upgrade, before rebooting into the new kernel.
# --tag: stable tag to download (default: derived from the release,
#   6.18.39+rpt-rpi-v6 -> v6.18.39).
# --source: use this driver source file instead of downloading it.
#
# Idempotent: nothing is rebuilt when this package version is already
# installed for the kernel. Other versions of the module are removed from
# DKMS first. A loaded module is never unloaded (the daemon may be using it):
# the script says when a reboot is needed to pick up the new build.
#
# DKMS AUTOINSTALL rebuilds the same source for a new kernel. That source
# belongs to the old kernel version; re-run this script with --kernel for the
# new release so the driver matches it.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_DIR="$SCRIPT_DIR/dkms/aquacomputer_d5next"
MODULE="aquacomputer_d5next"
# Bump when a patch in $PKG_DIR is added or changed, so DKMS sees a new version.
PATCH_LEVEL=1
STABLE_URL="https://git.kernel.org/pub/scm/linux/kernel/git/stable/linux.git/plain/drivers/hwmon/$MODULE.c"

KREL="$(uname -r)"
TAG=""
SOURCE_FILE=""

usage() {
  echo "Usage: $0 [--kernel <release>] [--tag <vX.Y[.Z]>] [--source <file>]" >&2
  exit 2
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --kernel)
      [[ $# -ge 2 ]] || usage
      KREL="$2"
      shift 2
      ;;
    --tag)
      [[ $# -ge 2 ]] || usage
      [[ "$2" =~ ^v[0-9]+\.[0-9]+(\.[0-9]+)?$ ]] || usage
      TAG="$2"
      shift 2
      ;;
    --source)
      [[ $# -ge 2 ]] || usage
      SOURCE_FILE="$2"
      shift 2
      ;;
    *)
      usage
      ;;
  esac
done

required=(dkms patch)
if [[ -z "$SOURCE_FILE" ]]; then
  required+=(curl)
fi
missing=()
for tool in "${required[@]}"; do
  # dkms lives in /usr/sbin, which is not on every user's PATH.
  command -v "$tool" > /dev/null 2>&1 || [[ -x "/usr/sbin/$tool" ]] || missing+=("$tool")
done
if [[ ${#missing[@]} -gt 0 ]]; then
  echo "error: missing ${missing[*]}; install with: sudo apt-get install -y ${missing[*]}" \
    "(this optional driver's packages are not in deploy/packages-rpi.txt)" >&2
  exit 1
fi

if [[ -z "$TAG" ]]; then
  if [[ "$KREL" =~ ^([0-9]+)\.([0-9]+)(\.([0-9]+))? ]]; then
    sublevel="${BASH_REMATCH[4]:-0}"
    TAG="v${BASH_REMATCH[1]}.${BASH_REMATCH[2]}"
    # Stable tags omit a zero sublevel: 6.18.0 is tagged v6.18.
    if [[ "$sublevel" != "0" ]]; then
      TAG="$TAG.$sublevel"
    fi
  else
    echo "error: cannot derive a kernel version from '$KREL'; pass --tag" >&2
    exit 1
  fi
fi
PKG_VER="${TAG#v}-aqb$PATCH_LEVEL"
SRC_DST="/usr/src/$MODULE-$PKG_VER"

HEADERS="/lib/modules/$KREL/build"
if [[ ! -f "$HEADERS/Makefile" ]]; then
  echo "error: no kernel headers for $KREL in $HEADERS" \
    "(linux-headers-rpi-v6, deploy/packages-rpi.txt)" >&2
  exit 1
fi
if grep -q "^CONFIG_SENSORS_AQUACOMPUTER_D5NEXT=y" "$HEADERS/.config" 2> /dev/null; then
  echo "warning: $KREL has $MODULE built in; a DKMS module cannot replace it," \
    "so the patches in $PKG_DIR are not applied" >&2
  exit 0
fi
if ! grep -qE "^CONFIG_USB_HID=(y|m)" "$HEADERS/.config" 2> /dev/null; then
  echo "error: $KREL is built without CONFIG_USB_HID, which $MODULE needs" >&2
  exit 1
fi

dkms_versions() {
  # "aquacomputer_d5next/6.18.39-aqb1, 6.18.39+rpt-rpi-v6, armv6l: installed"
  # or "aquacomputer_d5next/6.18.39-aqb1: added" -> 6.18.39-aqb1
  sudo dkms status -m "$MODULE" | sed -n "s|^$MODULE/\([^,:]*\).*|\1|p" | sort -u
}

echo "== $MODULE $PKG_VER for $KREL =="
if sudo dkms status -m "$MODULE" -v "$PKG_VER" -k "$KREL" | grep -q ": installed"; then
  echo "already installed"
else
  while read -r version; do
    [[ -n "$version" ]] || continue
    echo "removing $MODULE $version from DKMS"
    sudo dkms remove -m "$MODULE" -v "$version" --all
    sudo rm -rf "/usr/src/$MODULE-$version"
  done < <(dkms_versions)

  work="$(mktemp -d)"
  trap 'rm -rf "$work"' EXIT
  if [[ -n "$SOURCE_FILE" ]]; then
    cp "$SOURCE_FILE" "$work/$MODULE.c"
  else
    echo "downloading $MODULE.c ($TAG)"
    curl -fsSL --retry 3 -o "$work/$MODULE.c" "$STABLE_URL?h=$TAG"
  fi
  if ! grep -q "USB_PRODUCT_ID_AQUAERO" "$work/$MODULE.c"; then
    echo "error: $work/$MODULE.c does not look like the $MODULE driver" >&2
    exit 1
  fi

  shopt -s nullglob
  for patch_file in "$PKG_DIR"/*.patch; do
    name="$(basename "$patch_file")"
    if patch --dry-run -s -p1 -d "$work" < "$patch_file" > /dev/null; then
      patch -s -p1 -d "$work" < "$patch_file"
      echo "applied $name"
    elif patch --dry-run -s -R -p1 -d "$work" < "$patch_file" > /dev/null; then
      echo "skipped $name: already in the $TAG source"
    else
      echo "error: $name does not apply to $MODULE.c from $TAG; update the patch" >&2
      exit 1
    fi
  done
  shopt -u nullglob

  sudo install -d -m 755 "$SRC_DST"
  sudo install -m 644 "$work/$MODULE.c" "$PKG_DIR/Makefile" "$SRC_DST/"
  sed "s/@PKGVER@/$PKG_VER/" "$PKG_DIR/dkms.conf" | sudo tee "$SRC_DST/dkms.conf" > /dev/null
  sudo dkms add -m "$MODULE" -v "$PKG_VER"
  sudo dkms install -m "$MODULE" -v "$PKG_VER" -k "$KREL"
fi

if [[ "$KREL" != "$(uname -r)" ]]; then
  echo "built for $KREL; it loads after booting that kernel"
elif [[ -d "/sys/module/$MODULE" ]]; then
  loaded="$(cat "/sys/module/$MODULE/srcversion" 2> /dev/null || true)"
  installed="$(sudo modinfo -k "$KREL" -F srcversion "$MODULE" 2> /dev/null || true)"
  if [[ "$loaded" != "$installed" ]]; then
    echo "note: a different build of $MODULE is loaded; reboot (or stop" \
      "aqua-bridge, then rmmod and modprobe $MODULE) to use $PKG_VER"
  else
    echo "$MODULE $PKG_VER is loaded"
  fi
else
  sudo modprobe "$MODULE"
  echo "loaded $MODULE $PKG_VER"
fi
