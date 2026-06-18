#pragma once
// ======================================================
// SIDEWAYS SHUFFLE — per-direction calibrated trims
// Calibrated from SHR,30 and SHL,30 runs at SPEED_ULTRA_SLOW.
// IMPORTANT: trims compensate speed-dependent step loss —
// if SHUFFLE_SPEED changes, recalibrate both directions.
// ======================================================

#define SHUFFLE_SPEED        SPEED_ULTRA_SLOW
#define CASTER_SETTLE_MS     150

// Nominal: outer turns 270 (≈30.4°), moves 120. Trims cancel measured drift:
//   SHR: +0.55°/cycle CCW heading, +0.27 cm/cycle forward creep
//   SHL: +0.70°/cycle CCW heading, −0.39 cm/cycle (backward) creep

#define SHR_TURN_OUT   270
#define SHR_TURN_MID   535   // 540 − 5  → cancels CCW heading drift
#define SHR_FWD        108   // 120 − 12 ┐ sum kept at 240 so lateral
#define SHR_BWD        133   // 120 + 13 ┘ distance per cycle is unchanged

#define SHL_TURN_OUT   270
#define SHL_TURN_MID   546   // 540 + 6  → cancels CCW heading drift
#define SHL_FWD        138   // 120 + 18 ┐
#define SHL_BWD        102   // 120 − 18 ┘

// g_black is explicitly defined in explore.h (included after shuffle.h in main.c).
// This tentative definition makes it visible to the cliff-gated shuffle primitives;
// explore.h's initialiser (= 0) is the one that takes effect at runtime.
static volatile int g_black;

// Returns 1 if a cliff (g_black) was detected mid-sub-move, 0 otherwise.
// Halts the steppers immediately on black and does NOT settle (abort path).
static int steps_blocking(int left, int right)
{
    if (g_black) return 1;               // cliff already latched before we even start

    stepper_steps(left, right);

    while (!stepper_steps_done())
    {
        if (g_black)                     // cliff seen mid-move
        {
            stepper_halt();              // cancel remaining steps immediately
            return 1;
        }
        sleep_msec(5);
    }

    if (g_black) { stepper_halt(); return 1; }   // cliff on final poll before settle

    sleep_msec(CASTER_SETTLE_MS);
    return 0;
}

// Returns 1 if any sub-move was aborted on black, 0 if all five completed.
static int shuffle_step_right(void)
{
    stepper_set_speed(SHUFFLE_SPEED, SHUFFLE_SPEED);

    if (steps_blocking(-SHR_TURN_OUT,  SHR_TURN_OUT))  return 1;  // rotate right θ
    if (steps_blocking( SHR_FWD,       SHR_FWD))       return 1;  // forward (trimmed)
    if (steps_blocking( SHR_TURN_MID, -SHR_TURN_MID))  return 1;  // rotate left ~2θ (trimmed)
    if (steps_blocking(-SHR_BWD,      -SHR_BWD))       return 1;  // backward (trimmed)
    if (steps_blocking(-SHR_TURN_OUT,  SHR_TURN_OUT))  return 1;  // rotate right θ
    return 0;
}

// Returns 1 if any sub-move was aborted on black, 0 if all five completed.
static int shuffle_step_left(void)
{
    stepper_set_speed(SHUFFLE_SPEED, SHUFFLE_SPEED);

    if (steps_blocking( SHL_TURN_OUT, -SHL_TURN_OUT))  return 1;  // rotate left θ
    if (steps_blocking( SHL_FWD,       SHL_FWD))       return 1;  // forward (trimmed)
    if (steps_blocking(-SHL_TURN_MID,  SHL_TURN_MID))  return 1;  // rotate right ~2θ (trimmed)
    if (steps_blocking(-SHL_BWD,      -SHL_BWD))       return 1;  // backward (trimmed)
    if (steps_blocking( SHL_TURN_OUT, -SHL_TURN_OUT))  return 1;  // rotate left θ
    return 0;
}

// Strafe sideways `cycles` times.  Returns 0 if all cycles completed normally,
// 1 if a cliff (g_black) aborted the strafe early.
// Stale-latch discipline: clears g_black once before the first sub-move so the
// gate responds only to tape seen *during* this strafe (not a latch from a prior
// motion).  The monitor thread re-asserts within EXP_MON_SLEEP_MS (20 ms) if
// tape is actually present.
static int shuffle_sideways(orientation_t *ori, int cycles, bool to_left)
{
    g_black = 0;           // discard stale latch; monitor re-raises if tape is present

    for (int i = 0; i < cycles; i++)
    {
        int hit = to_left ? shuffle_step_left() : shuffle_step_right();
        if (hit) return 1;                     // abort — caller decides recovery

        print_orientation(ori);                // net rotation per cycle ≈ 0 by design
    }
    return 0;
}