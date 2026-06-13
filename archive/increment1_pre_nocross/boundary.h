#pragma once
// ======================================================
// BOUNDARY-FINDING + CLIFF-MAPPING
// ------------------------------------------------------
// Two-state reactive tracker (FOLLOW + SEARCH) with a
// provisional cliff/boundary discrimination window.
//
// The firmware does NOT track (x,y): it streams ODOM,l,r
// increments so the ground-station UI keeps the live pose,
// and emits classification markers (PROV/BPT/CLIFF/CORNER/
// LINE_LOST) that the UI logs at its current integrated
// position. The UI does the offline geometric fit.
//
// Depends only on libpynq (stepper_steps / stepper_steps_done
// / sleep_msec / uart_*) plus helpers from main_header.h
// (detect_black, send_message, read_uart_message,
//  orientation_t, rotate_orientation). Deliberately
// independent of the untested shuffle.h trims — only the
// blocking pattern (poll stepper_steps_done + caster dwell)
// is reused, reimplemented locally below.
// ======================================================

// ---- caster / motion ----
#define BND_CASTER_SETTLE_MS  150
#define BND_FOLLOW_SPEED      SPEED_SLOW     // forward wiggle (higher = slower)
#define BND_TURN_SPEED        SPEED_TURN     // in-place rotation

// ---- FOLLOW tuning ----
#define BND_BASE_STEP         60    // forward steps per wiggle tick
#define BND_WIGGLE_BIAS       18    // differential trim -> shallow arc
#define BND_LOST_THRESHOLD    8     // consecutive white ticks -> SEARCH

// ---- cliff discrimination ----
#define BND_CONFIRM_WINDOW    10    // ticks to re-see black -> boundary

// ---- SEARCH tuning ----
#define BND_SEARCH_DELTA0     8.0f    // first half-sweep amplitude (deg)
#define BND_SEARCH_GROW       6.0f    // amplitude growth per round (deg)
#define BND_MAX_SEARCH_ANGLE  165.0f  // give up past this -> LINE_LOST
#define BND_SEARCH_PROBE_DEG  4.0f    // rotation granularity within a half-sweep
#define BND_SEARCH_BACKSTEP   40      // backward steps once sweep > 60 deg

// ------------------------------------------------------
// Low-level: blocking step + caster dwell, streams ODOM.
// Convention (matches turn_right_90 in main_header.h):
//   right/CW turn  = stepper_steps(-s, +s), rotate_orientation(+deg)
// ------------------------------------------------------
static void bnd_send(const char *s)
{
    char m[64];
    snprintf(m, sizeof m, "%s", s);
    send_message(m);
}

static void bnd_bstep(int l, int r, int speed)
{
    stepper_set_speed(speed, speed);
    stepper_steps(l, r);

    while (!stepper_steps_done())
    {
        sleep_msec(5);
    }

    sleep_msec(BND_CASTER_SETTLE_MS);

    char m[48];
    snprintf(m, sizeof m, "ODOM,%d,%d", l, r);   // keep UI pose live
    send_message(m);
}

// forward wiggle tick; +bias steers one way, -bias the other
static void bnd_fwd(int bias)
{
    bnd_bstep(BND_BASE_STEP + bias, BND_BASE_STEP - bias, BND_FOLLOW_SPEED);
}

// in-place rotation by signed degrees (+ = right/CW), updates ori.
// Sign carries through s: +deg -> (-s,+s) = right, -deg -> (+|s|,-|s|) = left.
static void bnd_rotate(orientation_t *ori, float deg, int speed)
{
    int s = (int)(deg * (float)TURN_90_STEPS / 90.0f);
    bnd_bstep(-s, s, speed);
    rotate_orientation(ori, deg);
}

// non-blocking abort check: operator sends "S" to stop the trace
static int bnd_abort(void)
{
    if (uart_has_data(UART0))
    {
        char msg[MAX_MSG_LEN];
        read_uart_message(UART0, msg);
        if (strcmp(msg, "S") == 0) return 1;
    }
    return 0;
}

// ------------------------------------------------------
// Confirmation window (cliff vs boundary).
// Called immediately after a provisional black detection,
// with `bias` already flipped (so we curve away from the
// hit — correct cliff-avoidance). Sends PROV at once so the
// UI buffers the original crossing position, then wiggles up
// to CONFIRM_WINDOW ticks. A real boundary is re-crossed
// (must see white first, then black again) -> BPT (confirm).
// An isolated cliff is not re-hit -> CLIFF (reclassify).
//   returns 1 = boundary confirmed, 0 = cliff/timeout
//   *abort set to 1 if operator stopped.
// ------------------------------------------------------
static int bnd_confirm(int bias, int *abort)
{
    bnd_send("PROV");

    int seen_white = 0;

    for (int i = 0; i < BND_CONFIRM_WINDOW; i++)
    {
        if (bnd_abort()) { *abort = 1; return 0; }

        bnd_fwd(bias);

        int b = detect_black();

        if (!b)
        {
            seen_white = 1;          // left the patch
        }
        else if (seen_white)
        {
            bnd_send("BPT");         // re-crossed a real line
            return 1;
        }
    }

    bnd_send("CLIFF");               // never re-crossed -> isolated patch
    return 0;
}

// ------------------------------------------------------
// SEARCH: expanding alternating in-place sweep for a corner
// of arbitrary angle. Sweeps +delta toward the seed side,
// then back past center by a growing amount, alternating and
// expanding each round. detect_black() is checked along every
// probe rotation, so the whole swept arc is scanned. The net
// heading offset from search-start when black is re-found is
// the corner angle. A recovery hit is wrapped in the same
// provisional logic (a sweep can land on a cliff too).
//   returns  1 = re-found (CORNER sent), 0 = LINE_LOST, -1 = abort
// ------------------------------------------------------
static int bnd_search(orientation_t *ori, int seed_sign)
{
    float offset = 0.0f;                 // signed heading offset from start
    float amp    = BND_SEARCH_DELTA0;
    int   dir    = (seed_sign >= 0) ? 1 : -1;

    while (amp <= BND_MAX_SEARCH_ANGLE)
    {
        float target = dir * amp;
        float step   = (target > offset) ? BND_SEARCH_PROBE_DEG
                                         : -BND_SEARCH_PROBE_DEG;

        while ((step > 0.0f && offset < target) ||
               (step < 0.0f && offset > target))
        {
            if (bnd_abort()) return -1;

            float d = step;
            if ((step > 0.0f && offset + d > target) ||
                (step < 0.0f && offset + d < target))
            {
                d = target - offset;     // clamp final probe to the target
            }

            bnd_rotate(ori, d, BND_TURN_SPEED);
            offset += d;

            if (detect_black())
            {
                int abort = 0;
                if (bnd_confirm(BND_WIGGLE_BIAS, &abort))
                {
                    char m[48];
                    float ang = (offset < 0.0f) ? -offset : offset;
                    snprintf(m, sizeof m, "CORNER,%.1f", ang);
                    send_message(m);
                    return 1;
                }
                if (abort) return -1;
                // it was a cliff — keep sweeping
            }
        }

        // once the sweep is wide, back up so the vertex (now behind the
        // sensor's rotation radius) comes back into reach
        if (amp > 60.0f)
        {
            if (bnd_abort()) return -1;
            bnd_bstep(-BND_SEARCH_BACKSTEP, -BND_SEARCH_BACKSTEP, BND_FOLLOW_SPEED);
        }

        dir  = -dir;                 // alternate side
        amp += BND_SEARCH_GROW;      // expand
    }

    bnd_send("LINE_LOST");
    return 0;
}

// ------------------------------------------------------
// Top-level: run the boundary trace (entered by "BMAP").
// FOLLOW until LOST_THRESHOLD whites -> SEARCH for the
// corner -> resume FOLLOW. Ends on operator "S" or LINE_LOST,
// then signals BMAP_DONE so the UI runs the offline fit.
// ------------------------------------------------------
static void run_boundary_trace(orientation_t *ori)
{
    fprintf(stderr, "BMAP: start\n");

    int bias       = BND_WIGGLE_BIAS;     // seed steer direction
    int white_run  = 0;
    int prev_black = detect_black();

    while (1)
    {
        if (bnd_abort()) break;

        bnd_fwd(bias);
        int b = detect_black();

        if (b)
        {
            if (!prev_black)                 // white -> black crossing
            {
                bias = -bias;                // flip / curve away
                int abort = 0;
                bnd_confirm(bias, &abort);   // PROV + BPT/CLIFF
                if (abort) break;
                prev_black = detect_black();  // resync after the window
            }
            else
            {
                prev_black = 1;
            }
            white_run = 0;
        }
        else
        {
            prev_black = 0;
            white_run++;

            if (white_run > BND_LOST_THRESHOLD)
            {
                int seed_sign = (bias >= 0) ? 1 : -1;
                int r = bnd_search(ori, seed_sign);

                if (r <= 0) break;            // abort or LINE_LOST -> stop

                // re-found the line; reseed the wiggle toward the found side
                bias       = BND_WIGGLE_BIAS * seed_sign;
                white_run  = 0;
                prev_black = 1;
            }
        }
    }

    bnd_send("BMAP_DONE");
    fprintf(stderr, "BMAP: done\n");
}
