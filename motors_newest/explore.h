#pragma once
// ======================================================
// AUTONOMOUS EXPLORE-AND-MAP
// ------------------------------------------------------
// Sweep 180-deg semicircles, queue objects seen by the forward
// distance sensor, drive to + classify each (reusing the O-command
// logic), return to the scan origin, mark the region explored, then
// advance 300 mm and repeat. A background pthread watches the
// downward TCS3200 (detect_black) and raises an abort flag the instant
// black (cliff/boundary) is seen. Terminates on operator "S".
//
// MUST be included AFTER main_header.h and shuffle.h (uses their
// motion/sensor helpers; does not re-include shuffle.h which lacks a
// include guard).
// ======================================================

#include <pthread.h>
#include <math.h>

// ---- tuning ----
#define EXP_SWEEP_RANGE_MM    400     // forward detection range during a sweep
#define EXP_REGION_RADIUS_CM  40      // explored semicircle radius drawn in UI
#define EXP_ADVANCE_MM        300     // forward hop when a sweep finds nothing
#define EXP_APPROACH_STOP_MM  40      // coarse stop distance (phase 1) when approaching an object
#define EXP_APPROACH_FINAL_MM 15      // fine stop distance (phase 2); forward sensor reliable to ~here
#define EXP_FINAL_NUDGE_MM    5       // extra open-loop forward nudge after the 15 mm creep
#define EXP_FINAL_NUDGE_STEPS ((EXP_FINAL_NUDGE_MM * MOVE_UNIT) / EXP_MM_PER_UNIT)  // ~40 steps (~5 mm)
#define EXP_APPROACH_STEP     200     // phase-1 step between heading updates (was a full MOVE_UNIT;
                                      // smaller = re-aim more often, ~matches the old 4 s cadence)
#define EXP_FINE_STEP         120     // small batch for the slow final creep
#define EXP_FINE_CAP          12      // max fine-creep batches (bounds overrun if sensor goes -1)
#define EXP_MOUNTAIN_NEAR_MM  120     // forward proximity that trips obstacle avoidance
#define EXP_NOGO_SIZE_CM      10      // assumed small-cliff bounding box
#define EXP_MAX_OBJ           12      // max objects queued per sweep
#define EXP_CLUSTER_GAP_DEG   3       // >this many blank degrees closes an object cluster
#define EXP_MM_PER_UNIT       61      // ~MOVE_UNIT(500) * 0.0123 cm/step * 10
#define EXP_MON_SLEEP_MS      20
#define EXP_APPROACH_CAP      40      // max phase-1 steps before giving up (steps now smaller)
#define SWEEP_STEP_DEG        0.5f    // in-place rotation increment per sweep sample (finer = slower)
#define SWEEP_SETTLE_MS       60      // settle after each step before sampling the forward sensor
#define SWEEP_NEAR_TOL_MM     70      // readings within this of the object's mean = same rock
#define SWEEP_MIN_OBJ_STEPS   4       // an object must persist this many steps to count (noise filter)
#define SWEEP_GAP_STEPS       4       // out-of-range steps that close an object
#define SWEEP_SEG_CONFIRM     3       // sustained off-band steps that split into a new distance level
#define EXP_OH_SAMPLES        10      // overhead readings used to classify an object
#define EXP_OH_MAX_ATTEMPTS   40      // cap on overhead read attempts (sensor-stuck timeout)
#define EXP_RESWEEP_MATCH_DEG 12.0f   // max bearing diff to pair a re-swept rock with an original
#define EXP_RESWEEP_MATCH_MM  80      // max distance diff to confirm that pairing
#define EXP_RESWEEP_MIN_REFS  2       // matched rocks needed to trust a heading-drift correction

// ---- F3: trap / corner escape ----
#define EXP_TRAP_HAZ_LIMIT    3      // consecutive hazards w/o net progress -> "stuck", escape
#define EXP_TRAP_SCAN_DEG     180    // total in-place scan arc (+/- 90 deg about the heading)
#define EXP_TRAP_SCAN_STEP    10.0f  // coarse scan increment (deg) -- fast, just locating openness
#define EXP_TRAP_OPEN_MM      300    // a heading counts as "open" only if forward range >= this
#define EXP_TRAP_COMMIT_MM    450    // longer clear move committed to break out (> the 300 mm hop)
#define EXP_TRAP_MAX_TRIES    3      // escape attempts before giving up (stop + log) -- bounded

// ---- F2: boustrophedon coverage ----
// Total hop budget across the entire mission (guards against any infinite loop).
// 1.5 m arena / EXP_ADVANCE_MM = ~5 lane hops per direction; 60 >> that.
#define EXP_BOUS_HOP_BUDGET     60
// How many consecutive lane-start failures (both sidestep directions blocked, moved==0)
// before we declare the arena exhausted and emit EXPLORE_DONE.
#define EXP_BOUS_EXHAUST_LANES  2

// ---- F4: POSFIX translation estimation ----
#define EXP_POSFIX_MIN_SPAN_DEG  20.0f   // min bearing spread (deg) to avoid collinear degeneracy
#define EXP_DEG2RAD              0.01745329f  // pi/180; avoids importing full math header just for this

// advance_monitored / approach_object return codes
#define ADV_DONE      0
#define ADV_BLACK     1
#define ADV_STOP      2
#define ADV_MOUNTAIN  3

typedef struct { float rel_deg; int32_t dist_mm; } exp_obj_t;

// ------------------------------------------------------
// Background black-monitor thread (GPIO TCS3200 only -> no I2C
// contention with the forward/overhead distance sensors).
// ------------------------------------------------------
static volatile int g_black   = 0;   // set by the monitor thread
static volatile int g_mon_run = 0;
static volatile int g_stop    = 0;   // operator "S"
static pthread_t    g_mon_thread;

static void *exp_black_monitor(void *arg)
{
    (void)arg;
    while (g_mon_run)
    {
        if (detect_black()) g_black = 1;
        sleep_msec(EXP_MON_SLEEP_MS);
    }
    return NULL;
}

static void exp_mon_start(void)
{
    if (!g_mon_run) { g_black = 0; g_mon_run = 1;
                      pthread_create(&g_mon_thread, NULL, exp_black_monitor, NULL); }
}

static void exp_mon_stop(void)
{
    if (g_mon_run) { g_mon_run = 0; pthread_join(g_mon_thread, NULL); }
}

// 0 = continue, 1 = black seen, 2 = operator stop
static int exp_check(void)
{
    if (g_black) return 1;
    if (uart_has_data(UART0))
    {
        char m[MAX_MSG_LEN];
        read_uart_message(UART0, m);
        if (strcmp(m, "S") == 0 || strcmp(m, "HOLD") == 0) { g_stop = 1; return 2; }
    }
    if (g_stop) return 2;
    return 0;
}

// ------------------------------------------------------
// Rotation helpers (ori bookkeeping kept in sync).
// ------------------------------------------------------

// Proportional in-place turn by signed degrees (+ = CW), TURN_90 calibration.
static void exp_rotate_deg(orientation_t *ori, float deg)
{
    int s = (int)(fabsf(deg) * (float)TURN_90_STEPS_US / 90.0f);   // no-slip count (see turn_180)
    if (s <= 0) return;
    stepper_set_speed(SPEED_ULTRA_SLOW, SPEED_ULTRA_SLOW);
    if (deg >= 0) stepper_steps(-s, s);   // CW
    else          stepper_steps(s, -s);   // CCW
    wait_steps_done();
    rotate_orientation(ori, deg);
}

// Rotate by a relative offset using the SAME micro-step primitive the sweep
// measures with (turn_degree == +1 deg), so approach matches the measurement
// regardless of micro-step calibration.
static void exp_rotate_rel(orientation_t *ori, float rel)
{
    int n = (int)(rel < 0 ? -rel : rel);
    for (int i = 0; i < n; i++)
    {
        if (rel >= 0) { turn_degree();           rotate_orientation(ori, 1.0f); }
        else          { turn_degree_other_way(); rotate_orientation(ori, -1.0f); }
    }
}

// ------------------------------------------------------
// Queueing 180-deg sweep. Clusters contiguous in-range readings into objects
// (centroid relative heading + nearest distance). Returns object count, or
// -2 (black) / -3 (operator stop). Leaves heading restored to the origin.
// ------------------------------------------------------
// Carries the fractional step across the fine 0.5-deg sweep increments so 360 of them
// sum to exactly TURN_180_STEPS_US (1280) with no truncation drift. Reset per sweep.
static float g_sweep_residual = 0.0f;

// Blocking IN-PLACE rotation by signed deg (+ = CW). Polls stepper_steps_done so no
// steps are lost (unlike the non-blocking turn_degree), then settles before sampling.
// Uses the same no-slip calibration as turn_180 (TURN_180_STEPS_US/180 steps per deg),
// accumulating the fractional remainder so the reported bearing matches the physical heading.
static void sweep_rotate(orientation_t *ori, float deg)
{
    float want = fabsf(deg) * (float)TURN_180_STEPS_US / 180.0f + g_sweep_residual;
    int   s    = (int)(want + 0.5f);     // round to nearest whole step
    g_sweep_residual = want - (float)s;  // keep remainder for the next increment
    if (s < 0) s = 0;
    stepper_set_speed(SPEED_ULTRA_SLOW, SPEED_ULTRA_SLOW);   // slow + smooth for fine sampling
    if (s > 0)
    {
        if (deg >= 0) stepper_steps(-s, s);   // in-place CW
        else          stepper_steps(s, -s);   // in-place CCW
    }
    while (!stepper_steps_done()) sleep_msec(2);
    sleep_msec(SWEEP_SETTLE_MS);
    rotate_orientation(ori, deg);
}

static int sweep_collect(orientation_t *ori, exp_obj_t *objs, int maxn)
{
    read_distance_forward_raw();
    g_sweep_residual = 0.0f;              // independent runs: no carry from a prior sweep

    int steps = (int)(180.0f / SWEEP_STEP_DEG);

    sweep_rotate(ori, -90.0f);            // go to one extreme (single blocking move)

    int     n = 0;
    int     in_obj = 0, obj_cnt = 0, off = 0, gap = 0;
    long    obj_sum_dist = 0;             // running sum for the object's mean distance
    int32_t obj_min = 0;
    float   obj_min_rel = 0.0f;           // bearing at the nearest reading (= object bearing)
    int32_t closest = -1;                 // diagnostic: nearest in-range reading seen

    for (int i = 0; i < steps; i++)
    {
        int chk = exp_check();
        if (chk == 1) { log_msg("SWEEP: aborted on BLACK at ~%.0f deg (objs=%d)", -90.0f + (float)i * SWEEP_STEP_DEG, n); return -2; }
        if (chk == 2) { log_msg("SWEEP: aborted on STOP at ~%.0f deg (objs=%d)",  -90.0f + (float)i * SWEEP_STEP_DEG, n); return -3; }

        sweep_rotate(ori, SWEEP_STEP_DEG);

        float   rel = -90.0f + (float)(i + 1) * SWEEP_STEP_DEG;
        int32_t d   = read_distance_forward_raw();   // no delta-reject; we filter by persistence
        if (d > 0 && (closest < 0 || d < closest)) closest = d;

        int in_range = (d > 0 && d < EXP_SWEEP_RANGE_MM);

        if (in_range)
        {
            gap = 0;
            if (!in_obj)
            {
                in_obj = 1; obj_cnt = 1; obj_sum_dist = d; obj_min = d; obj_min_rel = rel; off = 0;
            }
            else
            {
                int32_t ref = (int32_t)(obj_sum_dist / obj_cnt);
                if (labs((long)d - (long)ref) <= SWEEP_NEAR_TOL_MM)
                {
                    obj_cnt++; obj_sum_dist += d;
                    if (d < obj_min) { obj_min = d; obj_min_rel = rel; }
                    off = 0;
                }
                else if (++off >= SWEEP_SEG_CONFIRM)
                {
                    // sustained new distance level -> close this rock, start the next
                    if (obj_cnt >= SWEEP_MIN_OBJ_STEPS && n < maxn)
                    { objs[n].rel_deg = obj_min_rel; objs[n].dist_mm = obj_min; n++; }
                    in_obj = 1; obj_cnt = 1; obj_sum_dist = d; obj_min = d; obj_min_rel = rel; off = 0;
                }
            }
        }
        else if (in_obj && ++gap >= SWEEP_GAP_STEPS)
        {
            if (obj_cnt >= SWEEP_MIN_OBJ_STEPS && n < maxn)
            { objs[n].rel_deg = obj_min_rel; objs[n].dist_mm = obj_min; n++; }
            in_obj = 0; off = 0;
        }
    }
    if (in_obj && obj_cnt >= SWEEP_MIN_OBJ_STEPS && n < maxn)
    { objs[n].rel_deg = obj_min_rel; objs[n].dist_mm = obj_min; n++; }

    sweep_rotate(ori, -90.0f);            // restore to the original (center) heading

    log_msg("SWEEP: %d obj, closest=%d mm", n, (int)closest);
    return n;
}

// move_batch_until stop fns (poll the g_black flag set by the monitor thread,
// NOT detect_black, to avoid GPIO contention with that thread).
static int exp_stop_advance(void)            // cliff or a mountain within range
{
    if (g_black) return 1;
    int32_t d = read_distance_forward();
    return (d > 0 && d < EXP_MOUNTAIN_NEAR_MM);
}
static int exp_stop_approach(void)           // cliff or object within coarse stop distance
{
    if (g_black) return 1;
    int32_t d = read_distance_forward();
    return (d >= 0 && d <= EXP_APPROACH_STOP_MM);
}
static int exp_stop_final(void)              // cliff or object within fine stop distance
{
    if (g_black) return 1;
    int32_t d = read_distance_forward();
    return (d >= 0 && d <= EXP_APPROACH_FINAL_MM);
}
static int stop_on_gblack(void) { return g_black; }   // cliff-only (for the open-loop final nudge)

// Open-loop forward nudge ~EXP_FINAL_NUDGE_MM at the slowest speed (dead-reckoned, since the
// forward sensor is unreliable below ~15 mm). Halts early only on a cliff. Returns 1 if a cliff
// halted it, else 0. Shared by approach_object's final step and the NUDGE test command.
static int final_nudge(void)
{
    return move_batch_until(EXP_FINAL_NUDGE_STEPS, SPEED_ULTRA_ULTRA_SLOW, stop_on_gblack);
}

// Global cliff-stop helpers for manual forward commands (poll the monitor's flag).
static int stop_black_or_S(void) { return g_black || stop_on_uart_S(); }
static void g_black_ack(void) { g_black = 0; }   // re-arm after handling a black halt
// F5: count of CONSECUTIVE black contacts with no intervening forward progress. Distinct from
// g_haz_streak (which also counts mountains) — g_black_run is a BLACK-ONLY continuation hint the
// UI uses to tell a boundary loop (long continuous run) from a lone interior cliff patch (run==1).
// Declared here (ahead of handle_black) so handle_black can increment it.
static int g_black_run = 0;

// ------------------------------------------------------
// Approach the object the robot is currently facing, stopping ~50 mm away.
// Re-pinpoints each move-unit with the silent heading_update(). *moved = units.
// ------------------------------------------------------
static int approach_object(orientation_t *ori, int *moved)
{
    (void)ori;   // heading_update() is a silent physical drift-correction; ori unchanged
    int count = 0;

    // --- Phase 1: coarse approach at SPEED_ULTRA_SLOW until <= EXP_APPROACH_STOP_MM ---
    while (1)
    {
        int chk = exp_check();
        if (chk == 1) { stepper_halt(); *moved = count; return ADV_BLACK; }
        if (chk == 2) { stepper_halt(); *moved = count; return ADV_STOP;  }

        int32_t dist = read_distance_forward();
        if (dist >= 0 && dist <= EXP_APPROACH_STOP_MM) break;   // reached coarse stop -> phase 2

        // step in EXP_APPROACH_STEP (< MOVE_UNIT) so heading_update runs more often;
        // halts mid-batch on cliff (g_black) or on reaching the coarse stop
        if (move_batch_until(EXP_APPROACH_STEP, SPEED_ULTRA_SLOW, exp_stop_approach))
        {
            if (g_black) { *moved = count; return ADV_BLACK; }
            break;          // reached coarse stop mid-batch -> phase 2
        }
        count++;
        heading_update();   // silent drift-correction (re-aims at the object; leaves ori intact)

        if (count >= EXP_APPROACH_CAP) { *moved = count; return ADV_DONE; }
    }

    // --- Phase 2: fine creep at SPEED_ULTRA_ULTRA_SLOW until <= EXP_APPROACH_FINAL_MM ---
    // No heading_update here: the object is close and centred; the slower speed lets
    // move_batch_until halt within ~1 mm of the target so the overhead lands over the rock.
    int arrived = 0;
    for (int fine = 0; fine < EXP_FINE_CAP; fine++)
    {
        int chk = exp_check();
        if (chk == 1) { stepper_halt(); *moved = count; return ADV_BLACK; }
        if (chk == 2) { stepper_halt(); *moved = count; return ADV_STOP;  }

        int32_t dist = read_distance_forward();
        if (dist >= 0 && dist <= EXP_APPROACH_FINAL_MM) { arrived = 1; break; }

        if (move_batch_until(EXP_FINE_STEP, SPEED_ULTRA_ULTRA_SLOW, exp_stop_final))
        {
            if (g_black) { *moved = count; return ADV_BLACK; }
            arrived = 1; break;          // halted at the fine stop distance
        }
    }

    // Manual open-loop nudge: forward ~EXP_FINAL_NUDGE_MM in steps so the overhead lands over the
    // rock (the forward sensor is unreliable below ~15 mm, so we dead-reckon the last bit).
    // Only when we actually confirmed the 15 mm stop; halts early only on a cliff.
    if (arrived)
    {
        if (final_nudge()) { *moved = count; return ADV_BLACK; }   // cliff during the nudge
    }
    *moved = count;
    return ADV_DONE;
}

// ------------------------------------------------------
// Forward hop in MOVE_UNIT chunks, watching black + forward obstacles.
// ------------------------------------------------------
static int advance_monitored(orientation_t *ori, int mm, int *moved)
{
    (void)ori;
    int units = mm / EXP_MM_PER_UNIT;
    if (units < 1) units = 1;
    int count = 0;

    for (; count < units; )
    {
        int chk = exp_check();
        if (chk == 1) { stepper_halt(); *moved = count; return ADV_BLACK; }
        if (chk == 2) { stepper_halt(); *moved = count; return ADV_STOP;  }

        int32_t d = read_distance_forward();
        if (d > 0 && d < EXP_MOUNTAIN_NEAR_MM) { *moved = count; return ADV_MOUNTAIN; }

        // halts mid-batch on cliff (g_black) or a mountain coming into range
        if (move_batch_until(MOVE_UNIT, SPEED_ULTRA_SLOW, exp_stop_advance))
        {
            *moved = count;
            return g_black ? ADV_BLACK : ADV_MOUNTAIN;
        }
        count++;
    }
    *moved = count;
    return ADV_DONE;
}

// ------------------------------------------------------
// Reverse dead-reckoning back to the scan origin: about-face, drive back the
// counted units, then trim heading to the recorded origin heading.
// ------------------------------------------------------
static void return_to_origin(orientation_t *ori, float origin_heading, int units)
{
    // Reverse straight back along the approach path - NO about-face turn. The 180 turn
    // was the main RET drift source (any angle error skewed the whole return path); reversing
    // keeps the current heading and retraces the path, so only straight-line slip remains.
    // (The trailing caster leads while reversing; SPEED_ULTRA_SLOW keeps it stable.)
    for (int i = 0; i < units; i++) { move_forward(-MOVE_UNIT, SPEED_ULTRA_SLOW); }   // move_forward blocks

    // Restore heading to the scan-center heading - a single rotation, done at the origin so its
    // error doesn't shift the return position. For RET (no bearing offset) delta = 0 -> no turn.
    float d = origin_heading - get_heading(ori);
    while (d > 180.0f)  d -= 360.0f;
    while (d < -180.0f) d += 360.0f;
    exp_rotate_deg(ori, d);
}

// ------------------------------------------------------
// Classify the object directly ahead (reuses the O-command logic).
// Returns size: -1 = mountain, -2 = color error, else 3 or 6. *out_color set
// for rocks.
// ------------------------------------------------------
// Returns: -1 mountain, -2 color-sensor error, -3 overhead-sensor error, else 3 or 6.
static int classify_object(Cal *cal, enum e_rock_color *out_color, float *out_temp)
{
    if (out_temp) *out_temp = read_temperature();   // read at the close (stopped) position
    enum e_rock_color rock_color = identify_rock_color(cal);
    if (rock_color == ERROR) return -2;

    // Collect up to EXP_OH_SAMPLES readings, but bounded by EXP_OH_MAX_ATTEMPTS so a
    // disconnected/stuck overhead sensor ({-1,false}) can't spin us forever.
    int collected = 0;       // total readings kept (valid distance OR out-of-range)
    int noor      = 0;       // out-of-range readings (flag_black) = tall/no-top
    long sum      = 0;       // sum of VALID distances only (never the -1 OOR sentinel)
    int  nv       = 0;       // count of valid distances
    int  oor_recent = 0;     // OOR among the last 5 kept readings (tall-black rule)

    for (int attempts = 0; collected < EXP_OH_SAMPLES && attempts < EXP_OH_MAX_ATTEMPTS; attempts++)
    {
        struct dist_oh_t nr = read_distance_overhead();
        if (nr.dist_oh == -1 && nr.flag_black == false) continue;   // pure disconnect: retry (bounded)

        int is_oor = (nr.flag_black);          // out of range = tall / no top in view
        if (is_oor) noor++;
        else { sum += nr.dist_oh; nv++; }

        collected++;
        if (collected > EXP_OH_SAMPLES - 5) oor_recent += is_oor ? 1 : 0;  // last 5 kept
    }

    if (collected < 5) return -3;              // sensor effectively unusable -> error, don't guess

    // Tall/black rock: the last readings were all out-of-range (no top seen) and front is black.
    if (oor_recent >= 5 && rock_color == BLACK) { *out_color = BLACK; return 6; }

    if (nv == 0)                               // all out-of-range -> tall object, can't measure height
    { *out_color = rock_color; return 6; }

    int avg = (int)(sum / nv);                 // average over VALID distances only

    if (avg >= 68 && avg <= 72)      return -1;            // mountain
    else if (avg >= 34)              { *out_color = rock_color; return 3; }
    else                             { *out_color = rock_color; return 6; }
}

static const char *rock_color_str(enum e_rock_color c)
{
    switch (c)
    {
        case RED:   return "red";
        case GREEN: return "green";
        case BLUE:  return "blue";
        case BLACK: return "black";
        case WHITE: return "white";
        default:    return "gray";
    }
}

// ------------------------------------------------------
// Hazard handlers.
// ------------------------------------------------------
static void handle_black(orientation_t *ori)
{
    stepper_halt();                          // CANCEL any in-flight forward steps FIRST, else the
                                             // reverse below is queued behind them and the robot
                                             // rolls further onto the cliff before backing off.
    exp_mon_stop();                          // pause monitor while we clear the edge
    send_message("NOGO");                    // UI logs a 10x10 cm no-go box at current pose (UNCHANGED)
    g_black_run++;                           // F5: extend the current black-contact run
    {
        char bm[48];
        snprintf(bm, sizeof bm, "BLACKPT,%.1f,%d", get_heading(ori), g_black_run);
        send_message(bm);                    // F5: per-contact report alongside NOGO (additive)
    }
    send_orientation(ori);

    move_forward(-2 * MOVE_UNIT, SPEED_ULTRA_SLOW);  // back straight off (known-safe direction)
    wait_motion(1500);
    exp_rotate_deg(ori, 90.0f);              // turn toward the interior; never circle the far side

    g_black = 0;
    exp_mon_start();
}

static void avoid_mountain(orientation_t *ori)
{
    stepper_halt();                          // cancel in-flight forward steps before reversing
    char m[48];
    snprintf(m, sizeof m, "MOUNTAIN,%d", 30);
    send_message(m);
    send_orientation(ori);

    move_forward(-MOVE_UNIT, SPEED_ULTRA_SLOW);    // back up a little
    wait_motion(600);

    for (int tries = 0; tries < 8; tries++)
    {
        if (g_stop) break;
        int hit = shuffle_sideways(ori, 2, true);   // strafe (toward one side); cliff-gated
        if (hit)
        {
            // Strafe was aborted on black/cliff tape — stop strafing immediately.
            // handle_black (called by the outer advance_with_escape loop) will back
            // off and reorient; avoid continuing to drive into the boundary.
            log_msg("avoid_mountain: strafe aborted on black - deferring to handle_black");
            break;
        }
        int32_t d = read_distance_forward();
        if (!(d > 0 && d < EXP_MOUNTAIN_NEAR_MM)) break;   // path ahead is clear
    }
}

// ------------------------------------------------------
// F3: trap / corner escape.
// ------------------------------------------------------
// Consecutive hazards (cliff/mountain) without net forward progress = the robot is
// ping-ponging in a corner or boxed in. We count them; at EXP_TRAP_HAZ_LIMIT we run a
// deliberate escape instead of letting handle_black/avoid_mountain oscillate forever.
static int g_haz_streak = 0;                 // F3 trap detector (reset on real forward progress)
// (g_black_run is declared earlier, just after g_black_ack, so handle_black can use it.)

static void haz_note(void)  { g_haz_streak++; }
static void haz_reset(void) { g_haz_streak = 0; }
static int  haz_stuck(void) { return g_haz_streak >= EXP_TRAP_HAZ_LIMIT; }

// F2: boustrophedon state (reset at run_explore entry).
static int g_bous_hops         = 0;  // total hops consumed (counts toward HOP_BUDGET)
static int g_bous_sidestep_dir = 1;  // +1 = try CW first, -1 = try CCW first (alternates
                                     //   so arena coverage is symmetric on re-enters)
static int g_bous_blocked      = 0;  // consecutive both-sidesteps-blocked events; arena
                                     //   exhausted (-> EXPLORE_DONE) at EXP_BOUS_EXHAUST_LANES

// Bounded in-place scan for the most-open heading. Pivots up to EXP_TRAP_SCAN_DEG/2 to EACH
// side of the current heading, returning to centre between halves so the body never circles
// PAST an unknown black streak (cliff-safety: same rule as handle_black's "never circle to the
// far side"). Samples the forward range at each step and keeps the heading with the largest
// free range (out-of-range = fully open) that clears EXP_TRAP_OPEN_MM and is not a near
// obstacle. Cliff-safe: if the monitor trips mid-pivot, that side is abandoned and the pivot
// reversed back over the arc we just traversed (known-safe) rather than continued onto the tape.
// Returns 1 and sets *best_rel (signed deg, + = CW) on success; 0 if no opening was found.
static int escape_scan(orientation_t *ori, float *best_rel)
{
    g_sweep_residual = 0.0f;                  // independent run: no carry from a prior sweep
    read_distance_forward_raw();              // prime the sensor
    g_black = 0;                              // clear a stale latch; monitor re-asserts a real cliff

    int   half      = EXP_TRAP_SCAN_DEG / 2;
    int   found     = 0;
    long  best_open = -1;
    float best      = 0.0f;

    // straight ahead (rel 0)
    {
        int32_t d    = read_distance_forward_raw();
        long    open = (d < 0) ? (long)MAX_RANGE_MM + 1 : (long)d;
        if (open >= EXP_TRAP_OPEN_MM) { best_open = open; best = 0.0f; found = 1; }
    }

    for (int side = 0; side < 2; side++)
    {
        float dir = (side == 0) ? +EXP_TRAP_SCAN_STEP : -EXP_TRAP_SCAN_STEP;  // CW then CCW
        float acc = 0.0f;

        for (float a = EXP_TRAP_SCAN_STEP; a <= (float)half + 0.01f; a += EXP_TRAP_SCAN_STEP)
        {
            int chk = exp_check();
            if (chk == 2) break;                       // operator stop
            if (chk == 1) { g_black = 0; break; }      // cliff this way -> abandon this side

            sweep_rotate(ori, dir);                    // blocking pivot one step (settles, tracks ori)
            acc += dir;

            if (g_black) { g_black = 0; break; }        // sensor saw tape now-ahead -> blocked

            int32_t d           = read_distance_forward_raw();
            long    open        = (d < 0) ? (long)MAX_RANGE_MM + 1 : (long)d;
            int     near_obst   = (d > 0 && d < EXP_MOUNTAIN_NEAR_MM);
            if (!near_obst && open >= EXP_TRAP_OPEN_MM && open > best_open)
            { best_open = open; best = acc; found = 1; }
        }

        sweep_rotate(ori, -acc);                       // back to centre heading before the other half
    }

    *best_rel = best;
    return found;
}

// Run the escape: scan for an opening, rotate to it, commit a longer monitored move to break
// out. Cliff-safe and bounded (EXP_TRAP_MAX_TRIES, then stop + log). Resets the hazard streak on
// success. Returns ADV_DONE on escape, ADV_STOP on operator-stop or give-up.
static int escape_trap(orientation_t *ori)
{
    send_message("TRAP");                     // UI: trap detected, attempting escape
    send_orientation(ori);
    log_msg("TRAP: boxed in (haz streak=%d) - escaping", g_haz_streak);

    for (int attempt = 0; attempt < EXP_TRAP_MAX_TRIES; attempt++)
    {
        if (g_stop) return ADV_STOP;

        float best_rel = 0.0f;
        if (!escape_scan(ori, &best_rel))
        {
            // No opening visible from here. Back straight off (known-safe; reverse is never
            // cliff-gated, matching handle_black recovery) and re-scan from a new vantage.
            if (g_stop) return ADV_STOP;
            log_msg("TRAP: no opening (attempt %d) - backing off", attempt + 1);
            move_forward(-2 * MOVE_UNIT, SPEED_ULTRA_SLOW);
            wait_motion(1200);
            continue;
        }

        log_msg("TRAP: opening at rel=%.1f - committing", best_rel);
        exp_rotate_deg(ori, best_rel);
        send_orientation(ori);

        int moved = 0;
        int r = advance_monitored(ori, EXP_TRAP_COMMIT_MM, &moved);
        if (r == ADV_STOP) return ADV_STOP;

        if (r == ADV_DONE && moved > 0)
        {
            log_msg("TRAP: escaped (%d units)", moved);
            haz_reset();
            send_message("TRAP_OK");
            return ADV_DONE;
        }

        // Hit a hazard while committing -> clear it (cliff-safe handlers) and try once more.
        if      (r == ADV_BLACK)    handle_black(ori);
        else if (r == ADV_MOUNTAIN) avoid_mountain(ori);
    }

    log_msg("TRAP: escape failed after %d tries - stopping", EXP_TRAP_MAX_TRIES);
    send_message("TRAP_FAIL");
    g_stop = 1;                               // bounded: never spin forever
    return ADV_STOP;
}

// One monitored hop with hazard handling + F3 trap detection. Returns 1 to keep exploring,
// 0 to stop the loop (operator S or trap give-up). Used for both advance sites in run_explore.
static int advance_with_escape(orientation_t *ori)
{
    int moved = 0;
    int r = advance_monitored(ori, EXP_ADVANCE_MM, &moved);

    if (r == ADV_STOP) return 0;
    if (r == ADV_DONE)
    {
        if (moved > 0)
        {
            haz_reset();
            g_black_run = 0;
            /* Gap-8: marker advances every hop (consistent with F/O: emit moved chunk count) */
            {
                char _stp[64];
                snprintf(_stp, sizeof _stp, "STEPS,%d", moved);
                send_message(_stp);
                send_orientation(ori);
            }
        }
        return 1;
    }  // net progress -> not stuck; F5: break the black run too

    if (r == ADV_BLACK)    handle_black(ori);
    else                   avoid_mountain(ori);                   // ADV_MOUNTAIN
    haz_note();

    if (haz_stuck() && escape_trap(ori) == ADV_STOP) return 0;
    return 1;
}

// ------------------------------------------------------
// Re-sweep loop-closure (drift correction between objects).
// After return_to_origin, sweep again and match the still-present rocks against the IMMUTABLE
// original sweep `objs0` (true-origin frame). The robust median bearing offset is the accumulated
// angular drift: apply -drift to `ori` so heading/UI stay truthful and the next return_to_origin
// physically re-centres (closed loop, no runaway -- return_to_origin resets ori to origin each cycle
// and this re-corrects it to truth). Separately refresh the NOT-yet-visited working objects from the
// new sweep so the next approach aims where the rock actually is now (robust to rotation AND small
// position drift). `report` also emits DRIFT/DOBJ diagnostics (the RSC test command).
// Returns ADV_DONE / ADV_BLACK / ADV_STOP (mirrors the in-loop sweep_collect convention).
// ------------------------------------------------------
static float exp_median(float *a, int len)
{
    for (int i = 1; i < len; i++)            // insertion sort (len <= EXP_MAX_OBJ)
    {
        float v = a[i]; int j = i - 1;
        while (j >= 0 && a[j] > v) { a[j + 1] = a[j]; j--; }
        a[j + 1] = v;
    }
    return (len & 1) ? a[len / 2] : 0.5f * (a[len / 2 - 1] + a[len / 2]);
}

static int resweep_correct(orientation_t *ori, exp_obj_t *objs, const exp_obj_t *objs0,
                           int n, int visited_k, int report)
{
    exp_obj_t robjs[EXP_MAX_OBJ];
    int rn = sweep_collect(ori, robjs, EXP_MAX_OBJ);
    if (rn == -2) return ADV_BLACK;
    if (rn == -3) return ADV_STOP;

    // greedy one-to-one match: each original rock -> nearest-in-bearing re-swept rock, gated by
    // EXP_RESWEEP_MATCH_DEG and validated by EXP_RESWEEP_MATCH_MM (the distance gate disambiguates
    // two rocks that fall within the bearing tolerance of each other).
    int   rused[EXP_MAX_OBJ];
    int   match_of[EXP_MAX_OBJ];             // re-swept index paired with original i, or -1
    float deltas[EXP_MAX_OBJ];
    int   nd = 0;
    for (int r = 0; r < rn; r++) rused[r] = 0;
    for (int i = 0; i < n; i++)
    {
        int   best = -1;
        float bestab = EXP_RESWEEP_MATCH_DEG;
        for (int r = 0; r < rn; r++)
        {
            if (rused[r]) continue;
            float db = robjs[r].rel_deg - objs0[i].rel_deg;
            float ab = db < 0 ? -db : db;
            long  dd = labs((long)robjs[r].dist_mm - (long)objs0[i].dist_mm);
            if (ab <= bestab && dd <= EXP_RESWEEP_MATCH_MM) { best = r; bestab = ab; }
        }
        match_of[i] = best;
        if (best >= 0) { rused[best] = 1; deltas[nd++] = robjs[best].rel_deg - objs0[i].rel_deg; }
    }

    // (a) heading/UI correction from the robust drift estimate (needs >= MIN_REFS references)
    float drift = 0.0f;
    if (nd >= EXP_RESWEEP_MIN_REFS)
    {
        float tmp[EXP_MAX_OBJ];
        for (int i = 0; i < nd; i++) tmp[i] = deltas[i];
        drift = exp_median(tmp, nd);         // measured offset = -e (drift since region start)
        rotate_orientation(ori, -drift);     // make ori truthful (true heading = ori + e)
        log_msg("RESWEEP: matched=%d drift=%.1f", nd, drift);
        send_orientation(ori);
        /* Gap-7: emit DRIFT unconditionally when correction applies (both report=0 and report=1) */
        {
            char _dm[64];
            snprintf(_dm, sizeof _dm, "DRIFT,%.1f,%d", drift, nd);
            send_message(_dm);
        }
    }
    else
        log_msg("RESWEEP: matched=%d (<%d) - no heading correction", nd, EXP_RESWEEP_MIN_REFS);

    // (b) F4: translation estimation — least-squares mean of per-pair displacement vectors
    // Requires >= EXP_RESWEEP_MIN_REFS matched pairs AND non-collinear geometry (bearing
    // spread >= EXP_POSFIX_MIN_SPAN_DEG). If degenerate, log and skip rather than emit
    // a bogus fix. dx = rightward mm, dy = forward mm (robot-local frame).
    int   posfix_ok = 0;
    float posfix_dx = 0.0f, posfix_dy = 0.0f;
    if (nd >= EXP_RESWEEP_MIN_REFS)
    {
        // Collect bearing span across all matched originals to test collinearity.
        float bear_min = 999.0f, bear_max = -999.0f;
        float sum_dx   = 0.0f,   sum_dy  = 0.0f;
        int   cnt      = 0;

        for (int i = 0; i < n; i++)
        {
            if (match_of[i] < 0) continue;
            float ai = objs0[i].rel_deg   * EXP_DEG2RAD;  // initial bearing (rad)
            float bi = (robjs[match_of[i]].rel_deg - drift) * EXP_DEG2RAD;  // re-swept bearing drift-corrected (rad)
            float ri = (float)objs0[i].dist_mm;
            float si = (float)robjs[match_of[i]].dist_mm;

            // Cartesian displacement this pair observes (robot-local: x=right, y=fwd)
            sum_dx += si * sinf(bi) - ri * sinf(ai);
            sum_dy += si * cosf(bi) - ri * cosf(ai);
            cnt++;

            float bear = objs0[i].rel_deg;
            if (bear < bear_min) bear_min = bear;
            if (bear > bear_max) bear_max = bear;
        }

        float span = bear_max - bear_min;
        if (span < EXP_POSFIX_MIN_SPAN_DEG)
        {
            log_msg("POSFIX unobservable, skipped (bearing span=%.1f deg < %.1f)", span, EXP_POSFIX_MIN_SPAN_DEG);
        }
        else
        {
            posfix_dx = sum_dx / (float)cnt;
            posfix_dy = sum_dy / (float)cnt;
            posfix_ok = 1;
            log_msg("POSFIX: dx=%.0f dy=%.0f mm (n=%d span=%.1f)", posfix_dx, posfix_dy, cnt, span);
        }
    }

    if (report)
    {
        /* Gap-3: RSC gives same sweep-complete visual as SWEEPQ */
        {
            char m[64];
            snprintf(m, sizeof m, "OBJN,%d", rn);
            send_message(m);
            for (int _gi = 0; _gi < rn; _gi++)
            {
                snprintf(m, sizeof m, "OBJ,%.1f,%d",
                         robjs[_gi].rel_deg, (int)robjs[_gi].dist_mm);
                send_message(m);
            }
        }
        /* Gap-7 cleanup: DRIFT now emitted unconditionally above (inside nd>=MIN_REFS block);
           removed here to avoid double-emit on the RSC path. char m[64] kept for DOBJ/POSFIX. */
        char m[64];
        for (int i = 0; i < n; i++)
            if (match_of[i] >= 0)
            {
                sprintf(m, "DOBJ,%.1f,%.1f,%.1f", objs0[i].rel_deg, robjs[match_of[i]].rel_deg,
                        robjs[match_of[i]].rel_deg - objs0[i].rel_deg);
                send_message(m);
            }
        // F4: emit translation fix if geometry was non-degenerate.
        // Negate: posfix_dx/dy = (resweep - original) = negative when robot drifted
        // in the positive direction; the UI applies the values directly, so we
        // flip the sign so a rightward drift of +D emits POSFIX,+D,...
        if (posfix_ok)
        {
            sprintf(m, "POSFIX,%d,%d", -(int)posfix_dx, -(int)posfix_dy);
            send_message(m);
        }
    }

    // (b) re-acquire: refresh the not-yet-visited working objects from the new sweep (current-centre
    // bearings), so the next approach aims where the rock is now. Unmatched -> keep stale, log it.
    for (int i = visited_k + 1; i < n; i++)
    {
        if (match_of[i] >= 0)
        {
            objs[i].rel_deg = robjs[match_of[i]].rel_deg;
            objs[i].dist_mm = robjs[match_of[i]].dist_mm;
        }
        else
            log_msg("RESWEEP: obj %d unmatched, keeping stale bearing %.1f", i, objs[i].rel_deg);
    }

    return ADV_DONE;
}

// ------------------------------------------------------
// F2: boustrophedon sidestep — turn 90°, advance one lane-width, turn back.
// Returns 1 if the sidestep made net progress (moved > 0), 0 if blocked.
// Cliff-safe: all lateral motion goes through advance_monitored.
// dir: +90.0f = try CW (right), -90.0f = try CCW (left).
// Uses heading-restoration so that handle_black's implicit 90° interior turn
// cannot leave the robot on a wrong heading after a failure path.
// On success: ori ends up facing (heading_before + 180°) — the reverse lane
//   direction, ready to drive back the other way in boustrophedon fashion.
// On failure: ori is restored to heading_before so the caller can try the
//   opposite direction cleanly.
// ------------------------------------------------------
// Rotate to an ABSOLUTE target heading by the shortest direction. Robust no
// matter what a hazard handler did to the current heading in between, because
// it re-reads get_heading(ori) and normalises the delta to [-180,180].
static void rotate_to_heading(orientation_t *ori, float target)
{
    float delta = target - get_heading(ori);
    while (delta >  180.0f) delta -= 360.0f;
    while (delta < -180.0f) delta += 360.0f;
    exp_rotate_deg(ori, delta);
}

// Sidestep into the next boustrophedon lane. `lane_heading` is the ORIGINAL lane
// direction captured by the caller BEFORE any hazard handler rotated the body,
// so every turn here targets an ABSOLUTE heading (handle_black turns ~90°
// interior on its own — a delta-based turn off the corrupted current heading
// would spiral, not lawnmower). `dir` = +90 (CW) / -90 (CCW) picks the lateral
// side. On success: steps one lane-width laterally and leaves the body facing
// the REVERSE lane (lane_heading+180). Returns 1 on a successful lateral hop,
// 0 if blocked (and restores lane_heading so the caller can try the other side).
static int bous_sidestep(orientation_t *ori, float lane_heading, float dir)
{
    rotate_to_heading(ori, lane_heading + dir);   // face laterally (absolute)

    int moved = 0;
    int r = advance_monitored(ori, EXP_ADVANCE_MM, &moved);

    if (r == ADV_STOP) { g_stop = 1; return 0; }

    if (r == ADV_DONE && moved > 0)   // moved>0 always holds for ADV_DONE; kept for clarity
    {
        // UI-feedback: ORT FIRST so the marker steps in the LATERAL heading,
        // THEN STEPS (moved directly, matching O/FB/advance_with_escape).
        char sm[32];
        snprintf(sm, sizeof sm, "STEPS,%d", moved);
        send_orientation(ori);
        send_message(sm);
        haz_reset();
        g_black_run = 0;
        rotate_to_heading(ori, lane_heading + 180.0f);  // face the reverse lane
        send_orientation(ori);                          // ORT for the new lane heading
        return 1;
    }

    // Blocked: run the hazard handler, drop any stale cliff latch, then restore
    // the lane heading (absolute) so the caller can try the other side cleanly.
    // (No haz_note() here — the main-hop hazard already counted; counting each
    // blocked sidestep would trip escape_trap prematurely.)
    if (r == ADV_BLACK)         handle_black(ori);
    else if (r == ADV_MOUNTAIN) avoid_mountain(ori);
    if (g_black) g_black_ack();
    rotate_to_heading(ori, lane_heading);
    return 0;
}

// ------------------------------------------------------
// Top-level explore loop (the EXPLORE command).
// ------------------------------------------------------
static void run_explore(orientation_t *ori, Cal *cal)
{
    log_msg("EXPLORE: start");
    send_message("EXPLORE");          /* UI: mission started */
    g_stop = 0;
    haz_reset();                  // F3: start each EXPLORE with a clean hazard streak
    g_black_run = 0;              // F5: start each EXPLORE with a clean black-contact run
    g_bous_hops         = 0;     // F2: reset hop budget
    g_bous_sidestep_dir = 1;     // F2: start with CW sidestep preference
    g_bous_blocked      = 0;     // F2: reset arena-exhaustion counter
    exp_mon_start();

    while (!g_stop)
    {
        float      origin_heading = get_heading(ori);
        exp_obj_t  objs[EXP_MAX_OBJ];
        int        n = sweep_collect(ori, objs, EXP_MAX_OBJ);

        if (n == -2) {                                  // black during sweep
            handle_black(ori); haz_note();
            if (haz_stuck() && escape_trap(ori) == ADV_STOP) break;
            continue;
        }
        if (n == -3) break;                             // operator stop

        char rm[48];
        snprintf(rm, sizeof rm, "REGION,%d", EXP_REGION_RADIUS_CM);
        send_message(rm);
        send_orientation(ori);

        /* Gap-2: per-cycle sweep result → UI renders object dots each cycle */
        {
            char sm[64];
            snprintf(sm, sizeof sm, "OBJN,%d", n);
            send_message(sm);
            for (int _gi = 0; _gi < n; _gi++)
            {
                snprintf(sm, sizeof sm, "OBJ,%.1f,%d",
                         objs[_gi].rel_deg, (int)objs[_gi].dist_mm);
                send_message(sm);
            }
        }

        if (n == 0)
        {
            // F2: boustrophedon coverage — advance until boundary, then sidestep.
            // Counts toward the global hop budget (guarantees termination).

            // Termination path 2: hop budget exhausted (hard bound, fires last).
            if (g_bous_hops >= EXP_BOUS_HOP_BUDGET)
            {
                log_msg("BOUS: hop budget exhausted (%d) -> EXPLORE_DONE", EXP_BOUS_HOP_BUDGET);
                break;
            }
            g_bous_hops++;

            // --- Try to advance one hop in the current lane direction ---
            int moved = 0;
            int r = advance_monitored(ori, EXP_ADVANCE_MM, &moved);

            if (r == ADV_STOP) break;

            if (r == ADV_DONE && moved > 0)
            {
                // Successful hop: emit STEPS for UI-feedback gap 8 (moved directly, not *MOVE_UNIT).
                char sm[32];
                snprintf(sm, sizeof sm, "STEPS,%d", moved);
                send_message(sm);
                send_orientation(ori);
                haz_reset();
                g_black_run   = 0;
                g_bous_blocked = 0;     // made forward progress -> not exhausted
                continue;   // sweep at the new position (back to top of while loop)
            }

            // Hop was blocked (moved==0 or ADV_BLACK/ADV_MOUNTAIN) → lane boundary.
            // Capture the lane heading BEFORE the hazard handler rotates the body
            // (handle_black turns ~90° interior on its own); the sidestep targets
            // this absolute heading so it steps laterally, not into the interior.
            float lane_heading = get_heading(ori);
            if (r == ADV_BLACK)         { handle_black(ori);   haz_note(); }
            else if (r == ADV_MOUNTAIN) { avoid_mountain(ori); haz_note(); }
            // ADV_DONE with moved==0: boundary immediately ahead, no hazard handler needed.
            if (g_black) g_black_ack();   // drop any stale cliff latch before re-aiming the sidestep

            // Termination path 3: inescapable corner — existing trap give-up.
            if (haz_stuck() && escape_trap(ori) == ADV_STOP) break;

            // --- Sidestep into the next lane: preferred direction, then alternate ---
            float dir1 =  g_bous_sidestep_dir * 90.0f;   // e.g. +90 = CW
            float dir2 = -g_bous_sidestep_dir * 90.0f;   // e.g. -90 = CCW

            if (bous_sidestep(ori, lane_heading, dir1))
            {
                g_bous_sidestep_dir = -g_bous_sidestep_dir;  // alternate for symmetry
                g_bous_blocked      = 0;
                continue;   // sweep at the new lane start
            }
            if (g_stop) break;

            if (bous_sidestep(ori, lane_heading, dir2))
            {
                g_bous_sidestep_dir = -g_bous_sidestep_dir;  // concave / asymmetric boundary
                g_bous_blocked      = 0;
                continue;
            }
            if (g_stop) break;

            // Both lateral lanes blocked. In a convex arena this means the reachable
            // strip is covered; in a concave one a single corner can falsely look
            // exhausted, so require EXP_BOUS_EXHAUST_LANES consecutive both-blocked
            // events, and between them try a trap-escape to relocate to open space.
            if (++g_bous_blocked >= EXP_BOUS_EXHAUST_LANES)
            {
                log_msg("BOUS: %d consecutive blocked lanes -> EXPLORE_DONE", g_bous_blocked);
                break;                                  // Termination path 1: arena exhausted
            }
            if (escape_trap(ori) == ADV_STOP) break;    // could not relocate -> done/stopped
            continue;                                   // relocated -> resume coverage
        }

        // immutable copy of the first sweep (true-origin frame) for drift estimation each hop
        exp_obj_t objs0[EXP_MAX_OBJ];
        for (int i = 0; i < n; i++) objs0[i] = objs[i];

        for (int k = 0; k < n && !g_stop; k++)
        {
            exp_rotate_rel(ori, objs[k].rel_deg);
            send_orientation(ori);   // face the rock BEFORE approaching, so the STEPS trail
                                     // and the FOUND_ROCK dot land at the correct map position

            int moved = 0, r = approach_object(ori, &moved);
            if (r == ADV_STOP) break;
            if (r == ADV_BLACK) { handle_black(ori); haz_note(); return_to_origin(ori, origin_heading, moved); continue; }
            /* Gap-5: marker drives to rock (consistent with O/FB: emit moved, not moved*MOVE_UNIT) */
            {
                char _stp[64];
                snprintf(_stp, sizeof _stp, "STEPS,%d", moved);
                send_message(_stp);
                send_orientation(ori);
            }

            enum e_rock_color col = NONE;
            float temp = TEMP_INVALID;
            int size = classify_object(cal, &col, &temp);
            if (size == -1)
            {
                avoid_mountain(ori);
            }
            else if (size > 0)
            {
                char fm[64];
                snprintf(fm, sizeof fm, "FOUND_ROCK,%d,%s,%.1f", size, rock_color_str(col), temp);
                send_message(fm);
                send_orientation(ori);
            }
            else  // -2 color error / -3 overhead sensor error: skip, don't fabricate a rock
            {
                log_msg("classify error (%d) - object skipped", size);
            }

            return_to_origin(ori, origin_heading, moved);
            /* Gap-6: marker returns along trail */
            {
                char _stp[64];
                snprintf(_stp, sizeof _stp, "STEPS,%d", -moved);
                send_message(_stp);
                send_orientation(ori);
            }
            if (g_black) { handle_black(ori); haz_note(); continue; }

            // re-sweep loop-closure: re-acquire the remaining targets + correct heading drift
            int rc = resweep_correct(ori, objs, objs0, n, k, 0);
            if      (rc == ADV_STOP)  break;
            else if (rc == ADV_BLACK) { handle_black(ori); haz_note(); }
        }

        if (!g_stop)
        {
            if (!advance_with_escape(ori)) break;   // F3: hop + trap detection
        }
    }

    // leave the monitor running: it is the global cliff-stop, active outside EXPLORE too
    send_message("EXPLORE_DONE");
    log_msg("EXPLORE: done");
}
