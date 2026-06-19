#!/bin/sh
# student-startup.sh — F1 run-on-boot launcher (manual §4.6, §3.2: no cable).
# Launches the explore firmware so the robot is fully autonomous from power-on.
# New PYNQ image: single-level app dir.
APP_DIR="$HOME/libpynq-5EID0-2023-v0.3.0/applications/motors_newest"
LOG="$HOME/student-startup.log"

cd "$APP_DIR" || { echo "$(date): app dir not found: $APP_DIR" >>"$LOG"; exit 1; }

# Single-instance guard: the libpynq hardware can be opened only once. If a ./main
# is already running (e.g. a manual bench launch), stand down quietly rather than
# fight for the hardware or trip the service's on-failure restart loop.
if pgrep -x main >/dev/null 2>&1; then
    echo "$(date): ./main already running (pid $(pgrep -x main | tr '\n' ' ')) - not starting a 2nd instance" >>"$LOG"
    exit 0
fi

# Optional: rebuild if the binary is missing (normally pre-built by `make`).
if [ ! -x ./main ]; then
    echo "$(date): ./main missing, attempting build" >>"$LOG"
    make >>"$LOG" 2>&1
fi

echo "$(date): launching ./main" >>"$LOG"
exec ./main >>"$LOG" 2>&1
