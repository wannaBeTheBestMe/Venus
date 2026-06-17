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
        if (strcmp(m, "S") == 0) { g_stop = 1; return 2; }
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
    int s = (int)(fabsf(deg) * (float)TURN_90_STEPS / 90.0f);
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
// Blocking IN-PLACE rotation by signed deg (+ = CW). Polls stepper_steps_done so no
// steps are lost (unlike the non-blocking turn_degree), then settles before sampling.
static void sweep_rotate(orientation_t *ori, float deg)
{
    int s = (int)(fabsf(deg) * (float)TURN_90_STEPS / 90.0f);
    if (s < 1) s = 1;
    stepper_set_speed(SPEED_ULTRA_SLOW, SPEED_ULTRA_SLOW);   // slow + smooth for fine sampling
    if (deg >= 0) stepper_steps(-s, s);   // in-place CW
    else          stepper_steps(s, -s);   // in-place CCW
    while (!stepper_steps_done()) sleep_msec(2);
    sleep_msec(SWEEP_SETTLE_MS);
    rotate_orientation(ori, deg);
}

static int sweep_collect(orientation_t *ori, exp_obj_t *objs, int maxn)
{
    read_distance_forward_raw();

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
        if (chk == 1) return -2;
        if (chk == 2) return -3;

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
    send_message("NOGO");                    // UI logs a 10x10 cm no-go box at current pose
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
        shuffle_sideways(ori, 2, true);      // strafe (toward one side)
        int32_t d = read_distance_forward();
        if (!(d > 0 && d < EXP_MOUNTAIN_NEAR_MM)) break;   // path ahead is clear
    }
}

// ------------------------------------------------------
// Top-level explore loop (the EXPLORE command).
// ------------------------------------------------------
static void run_explore(orientation_t *ori, Cal *cal)
{
    log_msg("EXPLORE: start");
    g_stop = 0;
    exp_mon_start();

    while (!g_stop)
    {
        float      origin_heading = get_heading(ori);
        exp_obj_t  objs[EXP_MAX_OBJ];
        int        n = sweep_collect(ori, objs, EXP_MAX_OBJ);

        if (n == -2) { handle_black(ori); continue; }   // black during sweep
        if (n == -3) break;                             // operator stop

        char rm[48];
        snprintf(rm, sizeof rm, "REGION,%d", EXP_REGION_RADIUS_CM);
        send_message(rm);
        send_orientation(ori);

        if (n == 0)
        {
            int moved = 0, r = advance_monitored(ori, EXP_ADVANCE_MM, &moved);
            if      (r == ADV_BLACK)    handle_black(ori);
            else if (r == ADV_STOP)     break;
            else if (r == ADV_MOUNTAIN) avoid_mountain(ori);
            continue;
        }

        for (int k = 0; k < n && !g_stop; k++)
        {
            exp_rotate_rel(ori, objs[k].rel_deg);

            int moved = 0, r = approach_object(ori, &moved);
            if (r == ADV_STOP) break;
            if (r == ADV_BLACK) { handle_black(ori); return_to_origin(ori, origin_heading, moved); continue; }

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
            if (g_black) handle_black(ori);
        }

        if (!g_stop)
        {
            int moved = 0, r = advance_monitored(ori, EXP_ADVANCE_MM, &moved);
            if      (r == ADV_BLACK)    handle_black(ori);
            else if (r == ADV_STOP)     break;
            else if (r == ADV_MOUNTAIN) avoid_mountain(ori);
        }
    }

    // leave the monitor running: it is the global cliff-stop, active outside EXPLORE too
    send_message("EXPLORE_DONE");
    log_msg("EXPLORE: done");
}
