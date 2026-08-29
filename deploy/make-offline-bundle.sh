#!/usr/bin/env bash
# Build an offline install bundle for the robot, on a laptop WITH internet.
# The robot needs no network access — copy the resulting .tar.gz over LAN
# (scp/rsync) or USB and run install.sh on it.
#
# The whole web stack is dependency-light: the only external runtime
# package is `ws` (used by the signaling server, zero transitive deps).
# Everything else is Node built-ins + relative imports, no build step. So
# the heavy parts of the bundle are just Node itself and Chromium .debs.
#
# Usage:
#   deploy/make-offline-bundle.sh [--suite bookworm] [--arch amd64]
#                                 [--node-version 22.19.0] [--skip-chromium]
#
#   --suite / --arch   Debian release + arch of the ROBOT (for Chromium
#                      .debs). Find them on the robot with:
#                          . /etc/os-release; echo $VERSION_CODENAME
#                          dpkg --print-architecture
#   --skip-chromium    Don't fetch Chromium .debs (handle Chromium some
#                      other way — local apt mirror, IT, already installed).
#
# Chromium .debs are fetched inside a `debian:<suite>` Docker container so
# the dependency closure matches the robot regardless of this laptop's OS.
# Needs Docker for that step; --skip-chromium if you don't have it.

set -euo pipefail

SUITE=bookworm
ARCH=amd64
NODE_VERSION=22.19.0
SKIP_CHROMIUM=0

while [ $# -gt 0 ]; do
  case "$1" in
    --suite) SUITE=$2; shift 2 ;;
    --arch) ARCH=$2; shift 2 ;;
    --node-version) NODE_VERSION=$2; shift 2 ;;
    --skip-chromium) SKIP_CHROMIUM=1; shift ;;
    *) echo "unknown arg: $1" >&2; exit 2 ;;
  esac
done

case "$ARCH" in
  amd64) NODE_ARCH=x64 ;;
  arm64) NODE_ARCH=arm64 ;;
  armhf) NODE_ARCH=armv7l ;;
  *) echo "unsupported --arch: $ARCH" >&2; exit 2 ;;
esac

HERE=$(cd "$(dirname "$0")" && pwd)
WEB=$(cd "$HERE/.." && pwd)
STAMP=$(date +%Y%m%d)
OUT="$WEB/dist"
NAME="former-webstack-offline-$STAMP"
STAGE="$OUT/$NAME"

echo "== staging $STAGE"
rm -rf "$STAGE"
mkdir -p "$STAGE/vendor" "$STAGE/chromium-debs"

# --- the web stack (source only) -------------------------------------------
echo "== copying stack"
mkdir -p "$STAGE/stack"
# copy everything except the things the robot doesn't need / can't use
tar -C "$WEB" \
  --exclude=node_modules --exclude=.git --exclude=dist --exclude=deploy \
  -cf - . | tar -C "$STAGE/stack" -xf -

# --- the one npm dependency: ws ------------------------------------------
if [ -d "$WEB/node_modules/ws" ]; then
  echo "== vendoring ws from node_modules"
  mkdir -p "$STAGE/stack/node_modules"
  cp -R "$WEB/node_modules/ws" "$STAGE/stack/node_modules/ws"
else
  echo "== node_modules/ws not found — run 'npm install' in web/ first" >&2
  exit 1
fi

# --- Node runtime ---------------------------------------------------------
NODE_PKG="node-v$NODE_VERSION-linux-$NODE_ARCH"
NODE_TAR="$NODE_PKG.tar.xz"
echo "== fetching $NODE_TAR"
curl -fL "https://nodejs.org/dist/v$NODE_VERSION/$NODE_TAR" -o "$OUT/$NODE_TAR"
tar -C "$STAGE/vendor" -xf "$OUT/$NODE_TAR"
mv "$STAGE/vendor/$NODE_PKG" "$STAGE/vendor/node"

# --- Chromium .debs -----------------------------------------------------
if [ "$SKIP_CHROMIUM" = 1 ]; then
  echo "== skipping Chromium .debs (--skip-chromium)"
  echo "handle Chromium separately: apt install chromium fonts-liberation, a local mirror, or IT." \
    > "$STAGE/chromium-debs/README-EMPTY.txt"
else
  if ! command -v docker >/dev/null 2>&1; then
    echo "!! docker not found — needed to fetch Chromium .debs for '$SUITE/$ARCH'." >&2
    echo "   re-run with --skip-chromium, or on a real $SUITE machine run:" >&2
    echo "     apt-get install -y --no-install-recommends --download-only \\" >&2
    echo "       -o Dir::Cache::archives=\$PWD/chromium-debs \\" >&2
    echo "       chromium chromium-sandbox fonts-liberation setserial" >&2
    exit 1
  fi
  echo "== fetching Chromium .debs via debian:$SUITE ($ARCH)"
  docker run --rm --platform "linux/$ARCH" -v "$STAGE/chromium-debs:/debs" "debian:$SUITE" \
    bash -c '
      set -e
      apt-get update
      apt-get install -y --no-install-recommends --download-only \
        -o Dir::Cache::archives=/debs \
        chromium chromium-sandbox fonts-liberation setserial
      chmod -R a+rw /debs
    '
  # apt leaves partial/ lock dirs behind
  find "$STAGE/chromium-debs" -maxdepth 1 -type d -name 'partial' -exec rm -rf {} +
  rm -f "$STAGE/chromium-debs/lock"
fi

# --- install-side files -------------------------------------------------
echo "== adding install.sh + unit files"
cp "$HERE/bundle/"* "$STAGE/"
chmod +x "$STAGE/install.sh" "$STAGE/kiosk-launch.sh"

# --- record what this bundle targets ----------------------------------
cat > "$STAGE/BUNDLE-INFO.txt" <<EOF
built:        $(date -u +%Y-%m-%dT%H:%M:%SZ)
target suite: $SUITE
target arch:  $ARCH
node:         v$NODE_VERSION ($NODE_ARCH)
chromium:     $([ "$SKIP_CHROMIUM" = 1 ] && echo "NOT bundled (--skip-chromium)" || echo "$(ls "$STAGE/chromium-debs"/*.deb 2>/dev/null | wc -l) .deb files")
EOF
cat "$STAGE/BUNDLE-INFO.txt"

# --- pack -------------------------------------------------------------
echo "== packing"
tar -C "$OUT" -czf "$OUT/$NAME.tar.gz" "$NAME"
rm -rf "$STAGE"
echo
echo "done: web/dist/$NAME.tar.gz"
echo "copy to the robot and run:  tar xzf $NAME.tar.gz && cd $NAME && ./install.sh"
