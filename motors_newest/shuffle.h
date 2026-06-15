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

static void steps_blocking(int left, int right)
{
    stepper_steps(left, right);

    while (!stepper_steps_done())
    {
        sleep_msec(5);
    }

    sleep_msec(CASTER_SETTLE_MS);
}

static void shuffle_step_right(void)
{
    stepper_set_speed(SHUFFLE_SPEED, SHUFFLE_SPEED);

    steps_blocking(-SHR_TURN_OUT,  SHR_TURN_OUT);   // rotate right θ
    steps_blocking( SHR_FWD,       SHR_FWD);        // forward (trimmed)
    steps_blocking( SHR_TURN_MID, -SHR_TURN_MID);   // rotate left ~2θ (trimmed)
    steps_blocking(-SHR_BWD,      -SHR_BWD);        // backward (trimmed)
    steps_blocking(-SHR_TURN_OUT,  SHR_TURN_OUT);   // rotate right θ
}

static void shuffle_step_left(void)
{
    stepper_set_speed(SHUFFLE_SPEED, SHUFFLE_SPEED);

    steps_blocking( SHL_TURN_OUT, -SHL_TURN_OUT);   // rotate left θ
    steps_blocking( SHL_FWD,       SHL_FWD);        // forward (trimmed)
    steps_blocking(-SHL_TURN_MID,  SHL_TURN_MID);   // rotate right ~2θ (trimmed)
    steps_blocking(-SHL_BWD,      -SHL_BWD);        // backward (trimmed)
    steps_blocking( SHL_TURN_OUT, -SHL_TURN_OUT);   // rotate left θ
}

static void shuffle_sideways(orientation_t *ori, int cycles, bool to_left)
{
    for (int i = 0; i < cycles; i++)
    {
        if (to_left)  shuffle_step_left();
        else          shuffle_step_right();

        print_orientation(ori); // net rotation per cycle ≈ 0 by design
    }
}