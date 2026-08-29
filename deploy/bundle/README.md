# Former web stack — offline bundle

Everything needed to run the browser-side robot stack on the Former 2.0
with **no network access on the robot**. Built on a laptop by
`deploy/make-offline-bundle.sh`; see `BUNDLE-INFO.txt` for what this
particular bundle targets.

## What's inside

| path | what |
|---|---|
| `stack/` | the web monorepo, source only (no build step). `node_modules/ws` vendored — the only external dependency. |
| `vendor/node/` | pinned Node.js runtime for the target arch |
| `chromium-debs/` | Chromium + deps as `.deb` (unless the bundle was built `--skip-chromium`) |
| `install.sh` | offline installer — run on the robot |
| `kiosk-launch.sh` | starts the servers + Chromium kiosk on `host.html` |
| `former-webstack.service` | optional systemd unit for the two Node servers alone |
| `99-former-serial.rules` | udev names (`/dev/ttyMOTOR`) — copy of ROAS `former_bringup` |
| `serial-policy.json` | Chromium managed policy so WebSerial doesn't re-prompt |

## Install (on the robot)

```sh
tar xzf former-webstack-offline-*.tar.gz
cd former-webstack-offline-*
./install.sh          # re-execs itself with sudo
```

Then:

1. **Log out / back in** (or reboot) so the `dialout` group applies.
2. **Free the serial port** — the Former's ROS bringup owns `/dev/ttyMOTOR`:
   ```sh
   sudo systemctl disable --now former_bringup.service
   ```
3. Plug in the Roboteq adapter, confirm `ls -l /dev/ttyMOTOR`.
4. The **kiosk** starts on the next desktop login (`host.html`, hardware
   mode). Run it now with `/opt/former-webstack/kiosk-launch.sh`.
5. **First run only:** Chromium shows the serial port picker once — pick the
   motor adapter. The managed policy suppresses it after that.

## How it runs

- On the robot: `serve-dashboard.mjs` serves the static pages on
  `:5173`, `signaling-server` listens on `:9770`, Chromium opens
  `http://localhost:5173/apps/dashboard/host.html` and (hardware mode)
  opens `/dev/ttyMOTOR` via WebSerial, sending the manifest's Roboteq
  bring-up sequence.
- Operator laptop: open `http://<robot-ip>:5173/apps/dashboard/index.html`,
  **operator** mode, signaling `ws://<robot-ip>:9770`. Data-channel-only
  WebRTC works over plain http; if your Chromium disagrees, front it with
  TLS.
- Safety is unchanged: the operator owns the keepalive, the Roboteq serial
  watchdog (RWD) stops the motors if it goes quiet.

## Updating the stack later

Re-run `install.sh` from a newer bundle — it replaces `stack/` and
`node/` in place and leaves the profile / policy / udev alone. Or, for a
code-only change, `rsync` the new `stack/` over `/opt/former-webstack/stack/`.
