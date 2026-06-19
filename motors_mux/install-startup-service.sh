#!/bin/sh
# install-startup-service.sh — idempotently install the Venus run-on-boot systemd unit
# on the PYNQ. Safe to re-run (re-asserts the same state). Run ON the board, from the
# motors_newest app dir (where student-startup.sh + student-startup.service live).
#
# Does NOT enable or start the service — that is a deliberate, motion-safe bench step
# (a calibrated reboot drives the robot within ~8s). Enable only after the staged test:
#   sudo systemctl enable --now student-startup.service   # arm for standalone duty
#   sudo systemctl disable --now student-startup.service  # disarm for bench sessions
#
# Single-instance: the enabled service and a manual `./main` launch CANNOT coexist
# (the libpynq hardware opens once). Disable the service before any manual bench launch.
set -e

APP_DIR="$HOME/libpynq-5EID0-2023-v0.3.0/applications/motors_newest"
UNIT="student-startup.service"
DEST="/etc/systemd/system/$UNIT"

cd "$APP_DIR" || { echo "app dir not found: $APP_DIR" >&2; exit 1; }

[ -f "$UNIT" ] || { echo "$UNIT not found in $APP_DIR (scp it here first)" >&2; exit 1; }
chmod +x student-startup.sh

# install -m 0644 is idempotent: overwrites in place with the correct mode.
sudo -n install -m 0644 "$UNIT" "$DEST"
sudo -n systemctl daemon-reload

echo "Installed $DEST (NOT enabled)."
echo "Verify:  systemctl status $UNIT --no-pager"
echo "Arm:     sudo systemctl enable --now $UNIT   (only after the staged motion-safe test)"
systemctl status "$UNIT" --no-pager || true
