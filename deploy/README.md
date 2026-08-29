# deploy/ — offline install bundle for the robot

The robot (Former 2.0, Debian) has **no internet** on the company network,
so the stack is installed from a bundle built on a laptop and copied over
LAN (`scp`/`rsync`) or USB.

## Build (on a laptop with internet)

```sh
# find the robot's Debian release + arch first:
#   ssh robot '. /etc/os-release; echo $VERSION_CODENAME; dpkg --print-architecture'

deploy/make-offline-bundle.sh --suite bookworm --arch amd64
# -> web/dist/former-webstack-offline-<date>.tar.gz
```

Prereqs on the build machine: `bash`, `curl`, `tar`, and **Docker** (used
to fetch Chromium `.deb`s in a `debian:<suite>` container so the
dependency closure matches the robot). No Docker? Pass `--skip-chromium`
and install Chromium on the robot another way (local apt mirror, IT, or
it's already there).

`npm install` must have been run in `web/` once so `node_modules/ws` is
present to vendor (`ws` is the stack's only external runtime dependency).

## Copy + install (on the robot)

```sh
scp web/dist/former-webstack-offline-*.tar.gz robot:~
ssh robot
tar xzf former-webstack-offline-*.tar.gz && cd former-webstack-offline-* && ./install.sh
```

Details, post-install steps, and how it runs: `bundle/README.md` (also
lands at the top of the unpacked bundle).

## Files

- `make-offline-bundle.sh` — the builder (laptop side)
- `bundle/` — everything that goes into every bundle verbatim
  (`install.sh`, `kiosk-launch.sh`, systemd unit, udev rule, Chromium
  policy, the bundle-side README)
