#!/usr/bin/env bash
# Offline install of the Former web stack on the robot. No network needed.
# Run on the robot after unpacking the bundle:  ./install.sh
#
# Installs to /opt/former-webstack, adds the invoking user to `dialout`,
# drops the udev rule + Chromium serial policy, and wires up autostart
# (desktop kiosk) + an optional systemd unit for the servers alone.

set -euo pipefail

BASE=/opt/former-webstack
HERE=$(cd "$(dirname "$0")" && pwd)
RUN_USER=${SUDO_USER:-$USER}
RUN_HOME=$(getent passwd "$RUN_USER" | cut -d: -f6)

say() { printf '\n== %s\n' "$*"; }
need_root() { [ "$(id -u)" = 0 ] || { echo "re-running with sudo"; exec sudo -E "$0" "$@"; }; }
need_root "$@"

[ -f "$HERE/BUNDLE-INFO.txt" ] && cat "$HERE/BUNDLE-INFO.txt"

# --- sanity: arch / suite match -----------------------------------------
want_arch=$(sed -n 's/^target arch: *//p' "$HERE/BUNDLE-INFO.txt" 2>/dev/null || true)
have_arch=$(dpkg --print-architecture)
if [ -n "$want_arch" ] && [ "$want_arch" != "$have_arch" ]; then
  echo "!! bundle is for '$want_arch' but this machine is '$have_arch'. Aborting." >&2
  exit 1
fi

# --- Chromium (offline .debs) ----------------------------------------
if ls "$HERE"/chromium-debs/*.deb >/dev/null 2>&1; then
  if command -v chromium >/dev/null 2>&1 || command -v chromium-browser >/dev/null 2>&1; then
    say "Chromium already present — skipping .deb install"
  else
    say "installing Chromium from bundled .debs"
    apt-get install -y --no-download "$HERE"/chromium-debs/*.deb \
      || dpkg -i "$HERE"/chromium-debs/*.deb \
      || apt-get -f install -y --no-download
  fi
else
  command -v chromium >/dev/null 2>&1 || command -v chromium-browser >/dev/null 2>&1 || {
    echo "!! no Chromium and none bundled. Install it (apt/local mirror) then re-run." >&2
    exit 1
  }
fi

# --- stack + Node ----------------------------------------------------
say "installing stack to $BASE"
mkdir -p "$BASE"
rm -rf "$BASE/stack" "$BASE/node"
cp -R "$HERE/stack" "$BASE/stack"
cp -R "$HERE/vendor/node" "$BASE/node"
"$BASE/node/bin/node" --version

# --- serial access -------------------------------------------------
say "serial access"
usermod -aG dialout "$RUN_USER" || true
if [ ! -e /dev/ttyMOTOR ]; then
  cp "$HERE/99-former-serial.rules" /etc/udev/rules.d/99-former-serial.rules
  udevadm control --reload-rules && udevadm trigger || true
  echo "installed udev rule (installing ROAS former_bringup is the fuller option)"
else
  echo "/dev/ttyMOTOR already exists — leaving udev rules alone"
fi

# --- Chromium managed policy: allow serial without a prompt --------
say "Chromium serial policy"
POLDIR=/etc/chromium/policies/managed
mkdir -p "$POLDIR"
cp "$HERE/serial-policy.json" "$POLDIR/former-serial.json"
# some builds read from chromium-browser's dir instead
mkdir -p /etc/chromium-browser/policies/managed 2>/dev/null || true
cp "$HERE/serial-policy.json" /etc/chromium-browser/policies/managed/former-serial.json 2>/dev/null || true

# --- launcher + autostart ---------------------------------------
say "launcher + autostart"
install -m 0755 "$HERE/kiosk-launch.sh" "$BASE/kiosk-launch.sh"

AUTOSTART="$RUN_HOME/.config/autostart"
mkdir -p "$AUTOSTART"
cat > "$AUTOSTART/former-kiosk.desktop" <<EOF
[Desktop Entry]
Type=Application
Name=Former host bridge (kiosk)
Exec=$BASE/kiosk-launch.sh
X-GNOME-Autostart-enabled=true
EOF
chown -R "$RUN_USER:$RUN_USER" "$RUN_HOME/.config/autostart"

# optional systemd unit for the servers alone (headless / no desktop)
sed "s|__USER__|$RUN_USER|g; s|__BASE__|$BASE|g" "$HERE/former-webstack.service" \
  > /etc/systemd/system/former-webstack.service
systemctl daemon-reload
echo "systemd unit installed but NOT enabled. To run the servers headless:"
echo "  systemctl enable --now former-webstack.service"

# --- reminder about the ROS driver -----------------------------
if systemctl list-unit-files 2>/dev/null | grep -q '^former_bringup'; then
  say "NOTE: former_bringup.service exists and owns /dev/ttyMOTOR"
  echo "  it and this stack cannot hold the serial port at the same time:"
  echo "  systemctl disable --now former_bringup.service"
fi

cat <<EOF

== done.
1. log out / back in (or reboot) so the 'dialout' group takes effect
2. plug in the Roboteq adapter; check:  ls -l /dev/ttyMOTOR
3. the kiosk starts on next desktop login, opening host.html in hardware mode
   -- or run it now:  $BASE/kiosk-launch.sh
4. first run only: Chromium's serial port picker appears once; pick the
   motor adapter. The managed policy suppresses it afterwards.
EOF
