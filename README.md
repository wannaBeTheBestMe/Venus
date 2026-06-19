# Venus — autonomous explore-and-map robot (CBL Team 6)

A PYNQ-Z2 differential-drive robot that autonomously explores a black-tape-bounded arena, finds and
classifies rock samples (size / colour / temperature), avoids cliffs and mountains, and streams its
findings to a **PyQt6 ground-station UI** that draws a live map.

This README is the **operational "how to run it" guide**. For the *how it works*:
- **[OVERVIEW.md](OVERVIEW.md)** — plain-language description of the robot and mission.
- **[ALGORITHM.md](ALGORITHM.md)** — technical reference: the EXPLORE loop, sensors, calibration, and the
  **full command + message protocol** (§10).

Active code: firmware in `motors_newest/` (C on libpynq); ground station in `UI_code.py` (PyQt6).

---

## 1. Prerequisites
- **Robot:** PYNQ-Z2 reachable over SSH at `student@10.43.0.1` (Mac link interface `en6` → board `10.43.0.1`).
  Passwordless key at `~/.ssh/pynq_key` (password is `student` if the key isn't installed).
- **Ground station (laptop):** the venv `Venus/.venv` with PyQt6 + paho-mqtt + numpy. Create it once:
  ```bash
  sh .claude/skills/launch-robot-stack/setup_venv.sh    # makes Venus/.venv and installs the 3 deps
  ```
- MQTT broker `mqtt.ics.ele.tue.nl:1883`, topics `/pynqbridge/{41|80}/send|recv` (via the on-robot ESP32).

## 2. Build & deploy the firmware (on the board)
libpynq lives **on the PYNQ**, so the firmware is built there. Stop any running `./main` first (you cannot
overwrite a running binary):
```bash
# from this Venus/ directory
ssh -i ~/.ssh/pynq_key -o StrictHostKeyChecking=no student@10.43.0.1 "pkill -f motors_newest/main; true"
scp -i ~/.ssh/pynq_key motors_newest/*.c motors_newest/*.h motors_newest/student-startup.sh \
    student@10.43.0.1:~/libpynq-5EID0-2023-v0.3.0/applications/motors_newest/
ssh -i ~/.ssh/pynq_key student@10.43.0.1 \
    "cd ~/libpynq-5EID0-2023-v0.3.0/applications/motors_newest && make clean && make"
```
The Makefile's `end.mk` applies the needed Linux capabilities (`setcap cap_sys_rawio,cap_sys_nice+ep`).
Verify with `getcap ./main`.

## 3. Run
**Easiest — the launcher skill** (opens two iTerm2 tabs: firmware over SSH + the UI in its venv):
```
/launch-robot-stack
```
**Manual — two terminals:**
```bash
# Terminal A (robot firmware)
ssh -i ~/.ssh/pynq_key student@10.43.0.1 \
    "cd ~/libpynq-5EID0-2023-v0.3.0/applications/motors_newest && ./main"
# Terminal B (ground station)
source Venus/.venv/bin/activate && python UI_code.py
```
**Autonomy on boot:** when `./main` starts *and a calibration is loaded*, it auto-enters EXPLORE after a
~5 s settle. Send **`S`** or **`HOLD`** in that window to drop to interactive command mode instead.
The UI only **listens** — operator commands go to the robot over MQTT.

## 4. First-time calibration — REQUIRED
The cliff monitor and autonomy are gated on a loaded black/white calibration. Before anything else:
```
CALBLACK     # follow the prompts: show the WHITE floor, then the BLACK tape
```
It persists to `/home/student/calblack.cfg` and reloads on boot. `CALRESET` reverts to compile-time defaults.
If you see **"cliff monitor INACTIVE until calibrated"**, run `CALBLACK`.

## 5. Common commands (send from the UI command box)
Full list + message protocol in **[ALGORITHM.md](ALGORITHM.md) §10**.

| Command | Does |
|---|---|
| `EXPLORE` | Run the full autonomous explore-and-map mission |
| `CALBLACK` / `CALRESET` | Calibrate / reset black-tape detection |
| `SWEEPQ` | 180° object sweep (reports detected objects) |
| `O` | Sweep → approach the first object → classify (size/colour/temp) |
| `RSC` | Re-sweep relocalization (heading-drift + POSFIX check) |
| `F` / `FB` | Forward until cliff / forward until an obstacle |
| `R` / `L` / `U` | Turn right 90° / left 90° / 180° |
| `MTN` | Mountain-avoidance test (strafe; halts on tape) |
| `SHL,n` / `SHR,n` | Strafe left / right n cycles |
| `TEMP` | Live temperature readout (until `S`) |
| `S` / `HOLD` | Stop the current action |
| `LOGON` / `LOGOFF` | Wireless log stream on / off |

Colour-sensor (TCS3200) demo commands also exist — `RGB`, `HSV`, `CMYK`, `SCALE,<0-3>`, `INTEG,<n>`, `WB`,
`CHROMA`, `DOM`, `NEAREST` — see ALGORITHM.md.

## 6. Two boards
A single binary serves both robots; it reads the robot id from **`~/robot_id`** at boot (falls back to `41`).
The UI supports robots 41 and 80 simultaneously; `MQTT_SAT.py` is the sole inter-robot relay.

## 7. Troubleshooting
- **Board unreachable:** `ping 10.43.0.1`; on a host-key error after an IP reuse, `ssh-keygen -R 10.43.0.1`.
- **UI won't start / `ModuleNotFoundError`:** run `setup_venv.sh`, then `Venus/.venv/bin/python -c "import PyQt6, paho.mqtt.client, numpy"`.
- **Robot won't move / "cliff monitor INACTIVE":** run `CALBLACK` (uncalibrated boots skip autonomy by design).
- **`make` fails with "text file busy" (ETXTBSY):** a `./main` is still running — stop it first (§2).
