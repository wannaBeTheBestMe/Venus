# Autonomous explore-and-map robot — algorithm & system overview

> **MAINTENANCE:** This file is the living description of how the robot works. **Keep it updated
> whenever the firmware's or UI's functionality changes** (new/changed commands, sensors,
> detection logic, hazard handling, calibration, message protocol, etc.). It targets someone new
> to the project who needs the important details. Active firmware: `motors_newest/`; UI:
> `UI_code.py`. (Earlier boundary-tracing work — `motors_xshut` / the "Increment 1/2" plans — is
> archived and superseded by the explore-and-map system described here.)

## 1. The system
A **PYNQ-Z2** drives a **differential-drive robot** (two stepper wheels + trailing caster).
Firmware is C on **libpynq**, in `motors_newest/` (`main.c`, `main_header.h`, `explore.h`,
`3_sensors_header.h`, `shuffle.h`). There is no on-board autonomy switch: the robot runs a UART
command loop. Commands in and telemetry/logs out travel wirelessly: firmware `send_message()` →
UART → an **ESP32 "pynqbridge"** → **MQTT** (`mqtt.ics.ele.tue.nl`, topics
`/pynqbridge/{41|80}/send|recv`) → a **PyQt6 ground-station UI** (`UI_code.py`) that integrates
pose and draws the map.

## 2. Mission context
A ~**150×150 cm arena**. Explore it; find and classify **rocks** (small cubes); avoid **mountains**
(cardboard boxes ≥30 cm — don't collide); respect **cliffs**. Cliffs and the arena **boundary are
black tape**; the boundary is conceptually a cliff and the footprint crossing it = falling off =
mission loss. Isolated ~10×10 cm black patches are interior cliffs to route around. (Telling a
small cliff from the true boundary is **deferred**; for now *all black = do-not-drive-onto*.)

## 3. Sensors
- **Downward TCS3200** — black detection (cliffs/boundary), **clear (unfiltered) channel** only
  (black → low reading). Mounted **ahead of the wheels** → lookahead before a wheel reaches an edge.
- **Forward VL53L0X** — objects/obstacles ahead. Range ≤ 500 mm, **wide ~25° cone**.
- **Overhead VL53L0X** (≤ 90 mm) — object **size** classification once approached.
- **Front TCS34725** — rock **color** (red/green/blue/black/white).
- **Thermistor on ADC0** — rock **temperature** (°C; Steinhart-Hart). Read at the close/stop
  position during classification; standalone live readout via `TEMP` (loops until `S`).

## 4. Low-level conventions
- **Stepper motion is non-blocking** and buffers only one "next" command — issuing faster than
  steps execute silently drops them. Two helpers tame this (`main_header.h`):
  - `stepper_halt()` = `stepper_reset()`+`stepper_enable()` — cancels in-flight + queued steps now.
  - `move_batch_until(steps, speed, stop_fn)` — issues one batch, **polls to completion**, halts the
    instant `stop_fn()` fires (prevents step-loss *and* gives prompt stops).
- **Calibration:** 1600 steps/rev; `MOVE_UNIT = 500` steps (~6 cm). In-place **turns run at
  `SPEED_ULTRA_SLOW`** (no slip) using no-slip-calibrated counts `TURN_90_STEPS_US = 625` /
  `TURN_180_STEPS_US = 1250` (the old SPEED_TURN-era 800/1500 over-rotate ~1.2× at this speed).
  `sweep_rotate` (SWEEPQ) still uses the old `TURN_90_STEPS = 800` and slightly over-covers — left as-is.
- **Orientation** (`orientation_t`): quadrant `ort` (1=N,2=E,3=S,4=W) + `theta` (0–90°);
  `get_heading = (ort−1)·90 + theta`. Firmware updates it on *deliberate* turns; the UI keeps live
  (x,y,heading) by integrating `ODOM`/`ORT`.
- **Black calibration** (`CALBLACK`): operator shows the **white floor** then **black tape**;
  firmware stores clear-channel min/max, **persists to `/home/student/calblack.cfg`**, loads at
  boot. `CALRESET` restores compile-time defaults and deletes the file.

## 5. Global cliff-stop (safety spine)
A **background thread** continuously reads the downward sensor and sets `g_black` on black; it is
the sole owner of that sensor (commands needing raw color reads pause it). **Every
forward-translating command halts the instant `g_black` is set**, so the robot stops before a wheel
crosses an edge (the ahead-of-wheels mounting gives the margin). It **auto-starts at boot only if a
calibration is loaded** (an uncalibrated monitor could read the floor as black and freeze motion).
Recovery from a black halt is a **reverse or turn** — intentionally *not* gated.

**Latched-flag lifecycle (avoid stale-disable across commands):** both `g_stop` (operator `S`,
latched in `exp_check`) and `g_black` (sticky-set by the monitor, never self-cleared) persist
until explicitly reset. To stop a stale flag from silently disabling a freshly-issued command:
`g_stop` is reset at **command dispatch** (every command); `g_black` is acked on-halt by the fast
forward commands (`F`/`FB`/`O`/`MOVE`/`R`/`L` — self-heal) and **cleared at entry** by the slow
diagnostic commands (`EXP1`/`ADV`/`SWEEPQ`) where the monitor safely re-asserts a real cliff within
~one poll. `g_black` is **never** cleared globally at dispatch (that would open a re-assert window
on the fast `SPEED_FAST` moves).

## 6. The EXPLORE algorithm (`run_explore`, loops until operator `S`)
1. **Sweep.** From the current pose ("scan origin"), rotate **in place through 180°** about the
   current heading, in fine increments; the forward sensor profiles the surroundings and the robot
   extracts **discrete objects** (bearing-relative-to-center + distance). Mark the scanned
   semicircle explored (`REGION`).
2. **If objects found**, for each: rotate to its bearing → **approach** in two phases (monitored;
   `heading_update` silently re-aims at the object without disturbing tracked heading): coarse to
   **40 mm** at `SPEED_ULTRA_SLOW`, then a fine creep to **15 mm** at `SPEED_ULTRA_ULTRA_SLOW`
   (slowest), then a fixed open-loop **~5 mm** nudge so the **overhead sensor parks over the rock** → 
   **classify** (front color + overhead size). **Rock** → send `FOUND_ROCK,size,color`. **Mountain**
   → run avoidance. Then **return to the scan origin** by **driving straight in reverse** along the
   approach path (no about-face turn — eliminates the 180° turn error that skewed the return),
   then restore heading, and continue.
3. **If none**, drive **forward ~300 mm** (monitored) and loop.

**Hazard handlers:**
- **Cliff** (`handle_black`): halt immediately (cancel in-flight steps **first**), log a 10×10 cm
  **no-go box** (`NOGO`), back straight off, turn toward the interior — never circle to the far side
  of an unknown black streak (could be the boundary).
- **Mountain** (`avoid_mountain`): back up, report, **shuffle sideways**, re-check forward; repeat
  until clear.

## 7. Sweep object detection (the subtle part)
The forward cone smears each object across ~25°, so naive "any in-range run = one object" merges
neighbors. The detector (`sweep_collect`):
- **0.5° steps, slow, blocking** (truly covers 180°, samples settled);
- **raw** forward reads — deliberately **no** sample-to-sample delta rejection (big jumps are real
  as the beam pans between objects at different ranges);
- groups readings while within a distance tolerance of the object's running mean, and **splits on a
  sustained shift to a new distance level** (separates overlapping-cone objects at different ranges);
- requires an object to **persist** several steps (rejects spikes); reports **bearing at the nearest
  reading** + that minimum distance.
- *Physical limit:* two objects at ~equal distance within one cone-width can still merge (sensor
  resolution, not a bug).

## 8. Object classification (`classify_object`)
Approach two-phase to ~15 mm at the slowest speed, then a fixed open-loop ~5 mm nudge (forward sensor is unreliable below ~15 mm), parking the overhead over the rock. Then: front color sensor → color; overhead sensor (10 readings) → size; thermistor (ADC0) → temperature. A rock is reported as `FOUND_ROCK,size,color,temp` (temp `n/a` if the ADC read is invalid).
All forward approaches share one function, `approach_object` (`EXP1`, `EXPLORE`, `O`, and `FB`), so the coarse-then-fine creep, cap, and cliff/stop handling are identical everywhere.
Hardened against bad sensor data: overhead sampling is **bounded** (`EXP_OH_MAX_ATTEMPTS`) so a
stuck/disconnected sensor can't hang; **out-of-range readings are excluded from the distance
average** (counted only via the "tall / no-top" rule); returns an error rather than guessing if too
few valid samples. Output: size ≈ 3 cm or 6 cm, or **mountain** (overhead average in a calibrated
bin), or error.

## 9. UI (`UI_code.py`)
PyQt6 scene (±150 cm, robot starts at center). Integrates pose from `ODOM`/`ORT`; renders the robot,
fitted **boundary** polygons (total-least-squares), **cliffs** (red X), **no-go boxes**,
**mountains** (amber ring), **rocks** (`FOUND_ROCK`), **explored semicircles** (`REGION`), and
**swept objects** (`OBJ` markers with a distance label + a line from the robot). A "DATA STREAM"
panel shows all wireless **`LOG`** lines.

## 10. Commands
- **`EXPLORE`** — the full autonomous loop (stop with `S`).
- **Per-behavior test sub-commands:** `CALBLACK`/`CALRESET`, `CLIFFCHK` (live black readings),
  `MON`/`MONOFF` (cliff monitor), `HU2` (drift-correction), `SWEEPQ` (sweep + report objects),
  `EXP1` (approach+classify one object), `RET` (out-and-back), `ADV` (monitored 300 mm advance),
  `MTN` (mountain avoidance), `NUDGE` (the open-loop ~5 mm final approach step in isolation).
- **Manual moves** — `F`, `FB`, `O`, `MOVE`, `R`/`L`, `STOPBLACK` — all honor the global cliff-stop.
- **Logging:** `LOGON`/`LOGOFF` toggle the wireless `LOG` mirror (off for the scored demo).

### Message protocol (firmware → UI), selected
`ODOM,l,r` (pose increment) · `ORT,ort,theta` (orientation) · `STEPS,n` · `REGION,radius_cm` ·
`OBJN,n` then `OBJ,rel_deg,dist_mm` (swept objects) · `FOUND_ROCK,size,color[,temp]` ·
`MOUNTAIN,size_cm` · `NOGO` (10×10 cm cliff box) · `EXPLORE_DONE` · `LOG,<text>`.

## 11. Known limitations / caveats
- **Dead-reckoning drift** accumulates over hops (no loop-closure/SLAM); the explored overlay
  slowly desyncs.
- **Boundary vs small-cliff** not yet classified — all black treated as do-not-cross.
- **Angular resolution** capped by the forward sensor's ~25° cone (close, equal-distance objects can
  merge).
- **Sweep reach** is ≤ ~0.5 m, so a sweep only sees nearby objects; coverage comes from 300 mm hops.
- Overhead classification is **hardened** (bounded attempts; out-of-range excluded from the
  distance average; returns an error instead of guessing/hanging). All callers — `EXP1`, `O`, and
  `EXPLORE` — share this via `classify_object`, and the `O`/`EXP1` approach loops are step-capped
  (`EXP_APPROACH_CAP`).

---
*Keep this file current — see the MAINTENANCE note at the top.*
