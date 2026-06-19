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
`3_sensors_header.h`, `shuffle.h`). The robot is **autonomous on boot** once the boot service is
installed+enabled: the launcher `student-startup.sh` + systemd unit `student-startup.service` are
defined in `motors_newest/` and installed on a board via `install-startup-service.sh` (enabling is a
deliberate motion-safe step — a calibrated reboot drives within ~8 s). It launches `./main` on
power-up (no cable), and after `READY`
— provided a calibration is loaded — the robot **auto-enters EXPLORE** following a ~5 s
settle/opt-out window during which the laptop only listens. A wireless `S`/`HOLD` in that window
drops to the interactive UART command loop instead; uncalibrated boots skip autorun and fall through
to the command loop. Commands in and telemetry/logs out travel wirelessly: firmware
`send_message()` → UART → an **ESP32 "pynqbridge"** → **MQTT** (`mqtt.ics.ele.tue.nl`, topics
`/pynqbridge/{41|80}/send|recv`) → a **PyQt6 ground-station UI** (`UI_code.py`) that integrates
pose and draws the map.

The UI supports **two simultaneous robots** (modules 41 and 80). Each robot has an independent
per-robot state dict (`self.bots[robot_name]`): heading, provisional boundary crossings, sweep
markers, boundary segments, and black contacts. A shared `self.cliffs` layer merges hazards from
both robots onto one map. Robot attribution is topic-based (MQTT topics `/pynqbridge/41/send` and
`/pynqbridge/80/send` each carry a `robot_name`); payloads are wire-identical between the two.
The relay between robots is performed exclusively by `MQTT_SAT.py` (loop-guarded with a `[B]`
prefix); the UI does not relay.

## 2. Mission context
A ~**150×150 cm arena**. Explore it; find and classify **rocks** (small cubes); avoid **mountains**
(cardboard boxes ≥30 cm — don't collide); respect **cliffs**. Cliffs and the arena **boundary are
black tape**; the boundary is conceptually a cliff and the footprint crossing it = falling off =
mission loss. Isolated ~10×10 cm black patches are interior cliffs to route around. Each black contact is
reported (`BLACKPT,heading,run`) and the UI now **classifies** black offline at end of run: a
**continuous run** of contacts (the robot tracing the tape loop) is fit as the **boundary polygon**;
an **isolated patch** (a single, lone contact) is drawn as an **interior cliff** (red X). Safety is
unchanged — *all black still halts forward motion the instant it is seen*; classification is purely
a mapping/visualization layer.

## 3. Sensors
- **Downward TCS3200** — a full-feature colour sensor used primarily for **black detection**
  (cliffs/boundary). Managed entirely by **`motors_newest/tcs3200.h`** (header-only C module,
  feature-parity with github.com/nthnn/TCS3200). The module owns all GPIO primitives, calibration
  arrays, colour-space conversions, and the global state struct `g_tcs`. `main_header.h` and `main.c`
  call the module API; `detect_black()` in `main_header.h` is re-based onto
  `tcs3200_read_clear_fast()`.

  Black detection uses the **clear (unfiltered) channel** only (black → low reading). Mounted
  **ahead of the wheels** → lookahead before a wheel reaches an edge. Each read is a **median of
  `DETECT_SAMPLES` pulse measurements** (`tcs3200_median_pulse_n`): the single busy-wait pulse
  timing (`tcs3200_pulseIn_LOW`) is corrupted when the OS preempts the monitor thread
  mid-measurement — the gap inflates the reading, and an inflated clear pulse maps toward black
  (false positive on white). The median rejects those outliers in both directions, removing the
  false positives without risking a false negative. (Filter-settle happens *before* the read;
  `detect_black` keeps CLEAR selected and re-asserts it idempotently with no delay, so cliff
  polling stays fast.) `integration_time` = median sample count N (API parity with the original).
  Frequency scaling (S0/S1) is controlled via `tcs3200_frequency_scaling()`; boot default is
  `TCS3200_SCALING_2PCT` (S0=H, S1=L — preserves the original hardcoded GPIO levels).
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
  `SPEED_ULTRA_SLOW`** (no slip) using no-slip-calibrated counts `TURN_90_STEPS_US = 640` /
  `TURN_180_STEPS_US = 1280` (empirically tuned via the 9×U realignment + U×2 confirm method).
  `sweep_rotate` (SWEEPQ) now shares this calibration (`TURN_180_STEPS_US/180` steps/°) and
  accumulates the fractional step across the 0.5° increments, so the sweep covers exactly 180°,
  returns to origin, and reported bearings match the physical heading.
- **Orientation** (`orientation_t`): quadrant `ort` (1=N,2=E,3=S,4=W) + `theta` (0–90°);
  `get_heading = (ort−1)·90 + theta`. Firmware updates it on *deliberate* turns; the UI keeps live
  (x,y,heading) by integrating `STEPS`/`ORT` (`STEPS,n` = forward displacement as a **`MOVE_UNIT`
  chunk count**, converted in the UI via `CM_PER_UNIT = MOVE_UNIT·CM_PER_STEP` ≈ 6.15 cm/chunk —
  NOT raw steps; `ORT,ort,theta` = heading snaps).
- **Black calibration** (`CALBLACK`): operator shows the **white floor** then **black tape**;
  firmware stores clear-channel min/max, **persists to `/home/student/calblack.cfg`**, loads at
  boot. `CALRESET` restores compile-time defaults and deletes the file.

## 5. Global cliff-stop (safety spine)
A **background thread** continuously reads the downward sensor and sets `g_black` on black — but only
after **`BLACK_CONFIRM` (3) consecutive** black reads (debounce); any white read resets the run. So a
single stray reading can't latch a false cliff, while a real cliff (read black continuously) still
latches within ~3 polls (tens of ms ≈ sub-mm of travel at explore speed, far inside the wheel-lead
margin). It is the sole owner of that sensor (commands needing raw color reads pause it). **Every
forward-translating command halts the instant `g_black` is set**, so the robot stops before a wheel
crosses an edge (the ahead-of-wheels mounting gives the margin). It **auto-starts at boot only if a
calibration is loaded** (an uncalibrated monitor could read the floor as black and freeze motion).
The **same `cal_loaded` guard** that gates the cliff-monitor auto-start also gates F1 AUTORUN —
uncalibrated → monitor OFF *and* no autonomous explore (a single consistent safety gate).
Recovery from a black halt is a **reverse or turn** — intentionally *not* gated.

**In-place turns suspend cliff latching** via a re-entrant counter `g_black_suspend` (> 0 → monitor
skips `detect_black`, resets the consecutive-black debounce, and loops). Guard helpers
`turn_guard_begin/end` bracket each reorientation turn (R, L, U, the two `turn_180()` calls in the
MOVE handler, and the `exp_rotate_deg` / `exp_rotate_rel` explore helpers). Only the **outermost**
begin/end pair clears `g_black` — nested turns don't churn it. `turn_guard_end` clears `g_black` so
the monitor re-confirms a real ahead-cliff (within `BLACK_CONFIRM` ≈ 60 ms) before the next forward
command relies on it. The **sweep rotation** (`sweep_rotate`) is intentionally **not** gated —
`sweep_collect` and `escape_scan` rely on black during the sweep for abort / side-selection logic.
Tradeoff: a cliff directly under a wheel mid-rotation is not caught until the turn ends; mitigant is
that turns are in-place and follow a stop (no translation toward an edge), and `turn_guard_end`
re-arms detection before any forward move — the same risk already accepted for recovery turns.

**Latched-flag lifecycle (avoid stale-disable across commands):** both `g_stop` (operator `S`,
latched in `exp_check`) and `g_black` (sticky-set by the monitor, never self-cleared) persist
until explicitly reset. To stop a stale flag from silently disabling a freshly-issued command:
`g_stop` is reset at **command dispatch** (every command); `g_black` is acked on-halt by the fast
forward commands (`F`/`FB`/`O`/`MOVE`/`R`/`L` — self-heal) and **cleared at entry** by the slow
diagnostic commands (`EXP1`/`ADV`/`SWEEPQ`) where the monitor safely re-asserts a real cliff within
~one poll. `g_black` is **never** cleared globally at dispatch (that would open a re-assert window
on the fast `SPEED_FAST` moves).

## 6. The EXPLORE algorithm (`run_explore`, loops until operator `S`)
On entry: emits `EXPLORE` (UI shows mission started).
1. **Sweep.** From the current pose ("scan origin"), rotate **in place through 180°** about the
   current heading, in fine increments; the forward sensor profiles the surroundings and the robot
   extracts **discrete objects** (bearing-relative-to-center + distance). Mark the scanned
   semicircle explored (`REGION`). After each sweep: emits `OBJN,n` + per-object `OBJ,rel_deg,dist_mm`
   (UI renders sweep dots every cycle, not just on SWEEPQ).
2. **If objects found**, for each: rotate to its bearing → **approach** in two phases (monitored;
   `heading_update` silently re-aims at the object without disturbing tracked heading): coarse to
   **40 mm** at `SPEED_ULTRA_SLOW`, then a fine creep to **15 mm** at `SPEED_ULTRA_ULTRA_SLOW`
   (slowest), then a fixed open-loop **~5 mm** nudge so the **overhead sensor parks over the rock** → 
   **classify** (front color + overhead size). **Rock** → send `FOUND_ROCK,size,color`. **Mountain**
   → run avoidance. After approach (ADV_DONE): emits `STEPS,moved` + `ORT` (UI marker drives to
   rock). Then **return to the scan origin** by **driving straight in reverse** along the
   approach path (no about-face turn — eliminates the 180° turn error that skewed the return),
   then restore heading. After normal return: emits `STEPS,-moved` + `ORT` (UI marker retraces back
   along the trail; negative sign drives the marker in reverse). **Re-sweep loop-closure**
   (`resweep_correct`): after each return, sweep again and match the still-present rocks against the
   original sweep — **refresh** the not-yet-visited objects' bearings from the new sweep (so the next
   approach aims where the rock actually is now, robust to rotation **and** small position drift) and,
   when ≥2 rocks match, apply the **median bearing offset** to the tracked heading so `ori`/UI stay
   truthful and the next return physically re-centres; emits `DRIFT,deg,n` unconditionally when
   correction is applied (both in-loop and RSC paths). When ≥2 non-collinear matched pairs are
   available (bearing spread ≥20°), it also computes a **translation fix** (`POSFIX,dx_mm,dy_mm`):
   the least-squares mean of the per-pair displacement vectors in the robot's local frame (dx=right,
   dy=fwd). Emitted during `RSC` diagnostics; the UI handler applies it to the tracked robot
   position. Collinear rock geometry (spread <20°) is logged as "unobservable, skipped" and no fix
   is applied. Then continue.
3. **If none** — F2 boustrophedon (lawnmower) coverage:
   1. **Lane run:** advance in `EXP_ADVANCE_MM` (300 mm) hops via `advance_monitored` until a
      boundary cliff (`ADV_BLACK`) or mountain (`ADV_MOUNTAIN`) blocks progress, or until the hop
      budget (`EXP_BOUS_HOP_BUDGET = 60`) is reached. On each successful hop (ADV_DONE, moved>0):
      emits `STEPS,moved` + `ORT` (UI marker advances one hop).
   2. **Lane transition:** at each boundary, capture the lane heading *before* the hazard handler runs
      (`lane_heading = get_heading(ori)`), run the existing hazard handler (recoil + interior turn),
      `g_black_ack()` any stale cliff latch, then call `bous_sidestep(ori, lane_heading, ±90°)`. The
      sidestep targets **absolute** headings via `rotate_to_heading`: it turns to `lane_heading ± 90°`
      (a true lateral step), advances one `EXP_ADVANCE_MM` lateral hop (`advance_monitored`,
      cliff-safe), then turns to face the reversed lane (`lane_heading + 180°`). On success it emits
      `ORT` **then** `STEPS,moved` (so the marker steps in the lateral heading) and alternates the
      preferred direction so coverage is symmetric. On a blocked sidestep the hazard is handled, the
      latch cleared, and `lane_heading` restored so the opposite direction is tried cleanly.
      *(Targeting the pre-hazard `lane_heading` is what makes this a true lawnmower, not an inward
      spiral: `handle_black` rotates the body ~90° on its own, so a delta-based turn would compound.)*
   3. **Termination — `EXPLORE_DONE`:**
      - *Primary:* both sidestep directions blocked for `EXP_BOUS_EXHAUST_LANES` (2) **consecutive**
        lane-ends — arena exhausted; logs `BOUS: N consecutive blocked lanes`. Between the first such
        event and the limit, `escape_trap` relocates the robot to open space and coverage resumes; the
        `g_bous_blocked` counter resets on any forward hop or successful sidestep (so a concave corner
        doesn't falsely declare the arena done).
      - *Secondary:* `g_bous_hops >= EXP_BOUS_HOP_BUDGET` (60) — hard budget bound (guarantees a stop
        even on a logic error; logs `BOUS: hop budget exhausted`).
      - *Tertiary:* existing `escape_trap` give-up (`EXP_TRAP_MAX_TRIES` exceeded) — inescapably
        cornered; returns `ADV_STOP`.
      - *Operator S/HOLD:* `g_stop` set inside `advance_monitored`/`exp_check()`.
      All `break` paths fall through to the single `EXPLORE_DONE` emit.
   The lane transition targets **absolute headings** off the pre-hazard `lane_heading` (via
   `rotate_to_heading`), so `handle_black`'s own ~90° interior turn cannot corrupt the lateral
   direction. No raw `stepper_steps` anywhere in the new code.

**Hazard handlers:**
- **Cliff** (`handle_black`): halt immediately (cancel in-flight steps **first**), log a 10×10 cm
  **no-go box** (`NOGO`), back straight off, turn toward the interior — never circle to the far side
  of an unknown black streak (could be the boundary). Each cliff halt also emits
  **`BLACKPT,heading,run`** alongside `NOGO` (additive — same pose, no behavior change), where `run`
  is the count of consecutive black contacts without intervening forward progress (`g_black_run`,
  reset on a net-progress hop). The UI uses `run` to tell a boundary trace (long run) from a lone
  interior cliff (run = 1). `g_black_run` is distinct from F3's `g_haz_streak` (the two coexist:
  streak = cliff+mountain trap detector, run = black-only continuation hint).
- **Mountain** (`avoid_mountain`): back up, report, **shuffle sideways**, re-check forward; repeat
  until clear.
  > **BUG-2 fix (2026-06-18):** `shuffle_sideways` (and its sub-primitives `steps_blocking`,
  > `shuffle_step_left`, `shuffle_step_right`) are now cliff-gated. `steps_blocking` polls `g_black`
  > inside its wait loop and calls `stepper_halt()` on detection. `shuffle_sideways` clears the stale
  > latch once at entry (matching the `escape_scan` / `g_black_ack` discipline) then returns 1 on
  > abort. `avoid_mountain` breaks immediately on a black hit rather than continuing to strafe. All
  > callers (`avoid_mountain`, `SHL`, `SHR`, `MTN`) are covered by the single primitive gate. The
  > sideways strafe is now as cliff-safe as the forward motion primitives.
- **Trap/corner escape** (`escape_trap`, F3): consecutive hazards without net forward progress
  (`g_haz_streak` ≥ `EXP_TRAP_HAZ_LIMIT`) mean the robot is boxed in / ping-ponging between the cliff and
  mountain handlers. It then runs a **bounded in-place open-direction scan** (`escape_scan`: pivots ±90°,
  returning to centre between halves so it never circles past an unknown black streak), rotates to the
  **most-open heading**, and commits a longer (`EXP_TRAP_COMMIT_MM`) monitored move to break out. Bounded
  by `EXP_TRAP_MAX_TRIES` — on give-up it stops the run. Emits `TRAP` / `TRAP_OK` / `TRAP_FAIL`. UI
  handlers added (UIFB): `TRAP` → amber "[TRAP] escaping", `TRAP_OK` → cyan "[TRAP_OK] escaped",
  `TRAP_FAIL` → red "[TRAP_FAIL] mission stopped" (previously emitted but silently ignored by the UI).

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
- *Logging:* on normal completion `sweep_collect` logs `SWEEP: N obj, closest=… mm`; if it aborts
  early it logs the **reason at the source** — `SWEEP: aborted on BLACK/STOP at ~<deg>` (so every
  caller — `RSC`, `SWEEPQ`, `EXPLORE`, the re-sweep — explains a mid-sweep stop instead of failing
  silently). Callers no longer disguise a `−2/−3` abort as "0 objects."

## 8. Object classification (`classify_object`)
Approach two-phase to ~15 mm at the slowest speed, then a fixed open-loop ~5 mm nudge (forward sensor is unreliable below ~15 mm), parking the overhead over the rock. Then: front color sensor → color; overhead sensor (10 readings) → size; thermistor (ADC0) → temperature. A rock is reported as `FOUND_ROCK,size,color,temp` (temp `n/a` if the ADC read is invalid).
All forward approaches share one function, `approach_object` (`EXP1`, `EXPLORE`, `O`, and `FB`), so the coarse-then-fine creep, cap, and cliff/stop handling are identical everywhere.
Hardened against bad sensor data: overhead sampling is **bounded** (`EXP_OH_MAX_ATTEMPTS`) so a
stuck/disconnected sensor can't hang; **out-of-range readings are excluded from the distance
average** (counted only via the "tall / no-top" rule); returns an error rather than guessing if too
few valid samples. Output: size ≈ 3 cm or 6 cm, or **mountain** (overhead average in a calibrated
bin), or error.

## 9. UI (`UI_code.py`)
PyQt6 scene (±150 cm, both robots start at center). Two robot markers (Robot41=cyan,
Robot80=magenta) each have independent pose state (`self.bots[robot_name]["angle"]`, `.marker`,
`.pending_pt`, `.sweep_items`, `.boundary_segments`, `.corner_angles`, `.tape_hits`,
`.black_contacts`). A shared `self.cliffs` layer collects hazard points from both robots.
Integrates pose from `STEPS`/`ORT` (`STEPS,n` forward displacement as a `MOVE_UNIT` chunk count, converted via `CM_PER_UNIT`;
`ORT,ort,theta` heading snaps); renders the robot, fitted **boundary** polygons
(total-least-squares per-robot), **cliffs** (red X, shared), **no-go boxes**, **mountains**
(amber ring), **rocks** (`FOUND_ROCK`), **explored semicircles** (`REGION`), and **swept objects**
(`OBJ` markers with a distance label + a line from the robot — per-robot, cleared per sweep). A
"DATA STREAM" panel shows all wireless **`LOG`** lines color-coded by source robot.

At **`EXPLORE_DONE`** the UI runs `classify_black_contacts` per robot: it clusters that robot's
`BLACKPT` contacts (per-robot, since odometry drift differs), classifies each cluster as a boundary
run or a lone interior cliff, and draws onto the **shared** boundary-polygon / `cliffs` layers.
Live contacts show as faint amber dots as they arrive.

## 10. Commands
- **`EXPLORE`** — the full autonomous loop (stop with `S`).
- **TCS3200 diagnostic/demo commands** (added with feature-parity refactor; pause the cliff monitor
  while reading, then restore it): `RGB` (emit `RGB,r,g,b` — calibrated 0-255 per channel);
  `HSV` (emit `HSV,h,s,v`); `CMYK` (emit `CMYK,c,m,y,k`); `SCALE,<0-3>` (set frequency scaling:
  0=power-down, 1=2 %, 2=20 % default, 3=100 %); `INTEG,<n>` (set integration time = median
  sample count, 1-16); `WB` (capture white-balance reference, emit `WB,r,g,b`); `CHROMA` (emit
  `CHROMA,f`); `DOM` (emit `DOM,RED|GREEN|BLUE` — dominant channel); `NEAREST` (emit
  `NEAREST,WHITE|BLACK|RED|GREEN|BLUE` — classify current colour against 5 arena references by
  Euclidean RGB distance).
- **Per-behavior test sub-commands:** `CALBLACK`/`CALRESET`, `CLIFFCHK` (live black readings),
  `MON`/`MONOFF` (cliff monitor), `HU2` (drift-correction), `SWEEPQ` (sweep + report objects),
  `EXP1` (approach+classify one object), `RET` (out-and-back), `ADV` (monitored 300 mm advance),
  `MTN` (mountain avoidance), `NUDGE` (the open-loop ~5 mm final approach step in isolation),
  `RSC` (re-sweep drift check: sweep → approach one rock → return → re-sweep, reports `DRIFT`/`DOBJ`;
  also emits `POSFIX,dx_mm,dy_mm` — translation fix, integers, robot-local frame — when ≥2
  non-collinear matched rocks are available; skips with a log if geometry is degenerate. Like
  `SWEEPQ`, RSC **isolates its in-place sweeps from the cliff monitor** (`exp_mon_stop`+`g_black=0`
  around each sweep — an in-place rotation can't translate toward a cliff) and **re-arms the monitor
  for the translating approach + return**, so it no longer freezes mid-sweep near tape. A sweep that
  does abort is now logged with its reason — see §7),
  `ESCAPE` (F3 trap/corner-escape routine in isolation: forces a stuck state and runs the open-direction
  scan + commit-move escape).
- **Manual moves** — `F`, `FB`, `O`, `MOVE`, `R`/`L`, `STOPBLACK` — all honor the global cliff-stop.
- **Autonomy / boot:** no command is needed for normal operation — after `READY` (calibration
  loaded) the robot auto-enters EXPLORE following a ~5 s settle window; a wireless `S`/`HOLD` during
  that window opts out to manual command mode (uncalibrated boots skip autorun). An optional
  `HELLO,<id>` may be emitted near `READY` (advisory; attribution is topic-based). The run-on-boot
  launcher is `student-startup.sh`, wired to boot by the systemd unit `student-startup.service`
  (both in `motors_newest/`; install via `install-startup-service.sh`, then `enable` per the staged test).
- **Logging:** `LOGON`/`LOGOFF` toggle the wireless `LOG` mirror (off for the scored demo).

### Message protocol (firmware → UI), selected
`STEPS,n` (forward displacement as a `MOVE_UNIT` **chunk count**; UI converts via
`CM_PER_UNIT = MOVE_UNIT·CM_PER_STEP` ≈ 6.15 cm/chunk — NOT raw steps. Firmware does **not** emit
`ODOM` — a legacy/dead `ODOM` handler lingers in the UI but is never exercised) ·
`ORT,ort,theta` (orientation snap; firmware `ort` 1=N, 2=E, 3=S, 4=W; UI maps to scene degrees
`{1:0, 2:90, 3:180, 4:270}` then adds `theta` — the old `{1:90,2:180,3:270,4:0}` map was a 90°
systematic error, fixed in F8) ·
`REGION,radius_cm` ·
`OBJN,n` then `OBJ,rel_deg,dist_mm` (swept objects) · `FOUND_ROCK,size,color[,temp]` ·
`MOUNTAIN,size_cm` · `NOGO` (10×10 cm cliff box — emitted by `handle_black` in the mission AND by the
test/diagnostic commands `F`/`FB`/`MOVE`/`R`/`L`/`O`/`SHL`/`SHR`/`STOPBLACK`/`EXP1`/`ADV`/`NUDGE` when
they halt on black, via the pure-UI helper `report_nogo`, so the operator sees the cliff regardless of
which command tripped it) ·
`BLACKPT,heading,run` (F5; per black contact, paired with `NOGO`, additive — heading in deg,
run = consecutive-black count) ·
`POSFIX,dx_mm,dy_mm` (F4 translation fix; dx=right, dy=fwd, mm, ints; non-collinear rocks only) ·
`EXPLORE` (UIFB; autonomous mission started — UI shows "[EXPLORE] mission started") ·
`DRIFT,deg,n` (UIFB; heading corrected by re-sweep, emitted unconditionally when nd≥MIN_REFS, both
in-loop and RSC paths; UI shows "[DRIFT] heading corrected X° from N refs" in amber) ·
`TRAP`/`TRAP_OK`/`TRAP_FAIL` (F3 trap-escape status; UIFB: now handled by UI — amber/cyan/red) ·
`HELLO,<id>` (optional boot identification ping near `READY`; advisory) ·
`EXPLORE_DONE` · `LOG,<text>`.
Note: `OBJN,n` / `OBJ,rel_deg,dist_mm` / `STEPS,n` / `ORT,ort,theta` now also apply within the
EXPLORE command (not only SWEEPQ/O/FB/F) — see §6 emit notes.

**Attribution:** each message is attributed by MQTT topic (Robot41 = `/pynqbridge/41/send`,
Robot80 = `/pynqbridge/80/send`). Payloads are format-identical; `robot_name` is set by the
`MQTTWorker` that received the message. Inter-robot relay is handled solely by `MQTT_SAT.py`.

## 11. Known limitations / caveats
- **Dead-reckoning drift**: within a region the per-object **re-sweep loop-closure**
  (`resweep_correct`) corrects accumulated **angular** drift (heading) AND computes a **translation
  fix** (`POSFIX`, F4) from the bearing+distance displacement of matched rocks. The UI applies the
  POSFIX to realign the robot overlay. Requires ≥2 non-collinear rocks (bearing spread ≥20°);
  degenerate geometry is skipped. Between-hop drift over large distances (no rocks in view) and
  global SLAM remain unsolved.
- **Boundary vs interior cliff** is now classified **offline at end of run** from `BLACKPT` contact
  runs (continuous run → boundary polygon; lone patch → cliff X). It is a heuristic on run length +
  spatial clustering, not a guarantee: a boundary explored in short disconnected bursts, or a large
  interior patch grazed repeatedly, can be mis-binned. The live cliff-stop is unaffected — all black
  always halts forward motion.
- **Angular resolution** capped by the forward sensor's ~25° cone (close, equal-distance objects can
  merge).
- **Sweep reach** is ≤ ~0.5 m, so a sweep only sees nearby objects; coverage comes from 300 mm hops.
- Overhead classification is **hardened** (bounded attempts; out-of-range excluded from the
  distance average; returns an error instead of guessing/hanging). All callers — `EXP1`, `O`, and
  `EXPLORE` — share this via `classify_object`, and the `O`/`EXP1` approach loops are step-capped
  (`EXP_APPROACH_CAP`).

---
*Keep this file current — see the MAINTENANCE note at the top.*
