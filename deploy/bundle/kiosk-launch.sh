#!/usr/bin/env sh
# Starts the two Node servers (signaling + static dashboard) and opens
# Chromium in kiosk mode on host.html (hardware/WebSerial mode). Invoked by
# the desktop autostart entry install.sh creates; also fine to run by hand.

set -eu
BASE="${FORMER_BASE:-/opt/former-webstack}"
export PATH="$BASE/node/bin:$PATH"

DASH_PORT="${DASHBOARD_PORT:-5173}"
SIG_PORT="${SIGNALING_PORT:-9770}"

# servers (bind 0.0.0.0 so a remote operator laptop can reach them)
SIGNALING_PORT="$SIG_PORT" node "$BASE/stack/apps/signaling-server/src/index.js" &
DASHBOARD_PORT="$DASH_PORT" node "$BASE/stack/scripts/serve-dashboard.mjs" &

# wait for the static server
i=0
until curl -sf "http://localhost:$DASH_PORT/apps/dashboard/host.html" >/dev/null 2>&1; do
  i=$((i + 1)); [ "$i" -gt 50 ] && { echo "dashboard server did not come up" >&2; exit 1; }
  sleep 0.2
done

CHROME="$(command -v chromium || command -v chromium-browser || echo /usr/bin/chromium)"
exec "$CHROME" \
  --kiosk \
  --user-data-dir="$BASE/chrome-profile" \
  --password-store=basic \
  --no-first-run \
  "http://localhost:$DASH_PORT/apps/dashboard/host.html"
