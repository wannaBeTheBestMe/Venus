#pragma once
// ============================================================
// tcs3200.h  —  Full-feature C module for the downward TCS3200
//               colour sensor (frequency-output, PYNQ-Z2 GPIO).
//
// Feature parity: every public feature of github.com/nthnn/TCS3200,
// implemented in C with the project's existing robust read primitive
// (median_pulse_n) kept as the default.
//
// CRITICAL SAFETY NOTE: detect_black() in main_header.h is re-based
// onto tcs3200_read_clear_fast() — a dedicated, settle-free variant
// that preserves the exact original observable behaviour (median of
// DETECT_SAMPLES on CLEAR only, fast path, fail-safe-to-white).
// Do NOT move or alter detect_black without an explicit audit.
//
// Scope: DOWNWARD TCS3200 only.  The front TCS34725 (I²C) lives
//        entirely in 3_sensors_header.h — this file does NOT touch it.
// ============================================================

#include <libpynq.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

// ============================================================
// 1. Channel constants  (S2/S3 encoding — same as original)
// ============================================================
#define TCS3200_RED    0
#define TCS3200_GREEN  1
#define TCS3200_BLUE   2
#define TCS3200_CLEAR  3

// ============================================================
// 2. Frequency-scaling constants  (S0/S1 encoding)
// Levels per TAOS TCS3200 datasheet Table 1 (matches nthnn lib):
// ============================================================
#define TCS3200_SCALING_PWR_DOWN  0   // S0=L S1=L — power down
#define TCS3200_SCALING_2PCT      1   // S0=L S1=H — 2 % output
#define TCS3200_SCALING_20PCT     2   // S0=H S1=L — 20 % (boot default)
#define TCS3200_SCALING_100PCT    3   // S0=H S1=H — 100 %

// ============================================================
// 3. Colour-space structs
// ============================================================
typedef struct { uint8_t r, g, b; } RGBColor;
typedef struct { float   h, s, v; } HSVColor;
typedef struct { float   c, m, y, k; } CMYKColor;
typedef struct { float   x, y, z; } CIE1931Color;

typedef void (*tcs3200_cb)(void);

// ============================================================
// 4. Module state struct
// ============================================================
typedef struct {
    // --- pin assignments ---
    int s0, s1, s2, s3, out;

    // --- per-channel calibration (white-ref pulse → 255, dark-ref pulse → 0) ---
    long cal_min[4];   // white reference  (smallest pulse = bright)
    long cal_max[4];   // dark  reference  (largest  pulse = dark)

    // --- runtime settings ---
    int  scaling;          // TCS3200_SCALING_* — also drives S0/S1 directly
    int  integ_samples;    // integration_time == median sample count N (default DETECT_SAMPLES)
    bool robust;           // true  = median  (default, safe, keeps false-black fix)
                           // false = single read (literal parity demo mode)

    // --- white-balance reference (0 = uncalibrated / off) ---
    RGBColor white_bal;

    // --- threshold interrupt targets + callbacks ---
    RGBColor up_thr, lo_thr;
    tcs3200_cb up_cb, lo_cb;
} tcs3200_t;

// ============================================================
// 5. Module-level state (single global instance; sensor is a
//    singleton — only one TCS3200 on this board).
// ============================================================

// Fallback defaults if this header is ever included before the
// main_header.h #defines are seen (shouldn't happen in normal builds).
#ifndef DETECT_SAMPLES
#define DETECT_SAMPLES 5
#endif
#ifndef CAL_SAMPLES
#define CAL_SAMPLES 5
#endif
#ifndef SENSOR_SETTLE_MS
#define SENSOR_SETTLE_MS 10
#endif

static tcs3200_t g_tcs = {
    .s0 = S0, .s1 = S1, .s2 = S2, .s3 = S3, .out = SENSOR_OUT,
    .cal_min   = { RED_MIN,  GREEN_MIN, BLUE_MIN,  CLEAR_MIN  },
    .cal_max   = { RED_MAX,  GREEN_MAX, BLUE_MAX,  CLEAR_MAX  },
    .scaling   = TCS3200_SCALING_20PCT,
    .integ_samples = DETECT_SAMPLES,
    .robust    = true,
    .white_bal = { 0, 0, 0 },
    .up_thr    = { 0, 0, 0 },
    .lo_thr    = { 0, 0, 0 },
    .up_cb     = NULL,
    .lo_cb     = NULL
};

// ============================================================
// 6. Low-level primitives  (moved from main_header.h;
//    main_header.h now includes tcs3200.h instead).
// ============================================================

// Time one LOW pulse on a GPIO pin.  Returns 0 on timeout.
static uint32_t tcs3200_pulseIn_LOW(int pin)
{
    const uint32_t TIMEOUT_US = 1000000;
    uint32_t elapsed = 0;

    // wait for any current LOW to finish
    while (gpio_get_level(pin) == GPIO_LEVEL_LOW)
    {
        if (++elapsed > TIMEOUT_US) return 0;
    }
    elapsed = 0;
    // wait for HIGH to finish
    while (gpio_get_level(pin) == GPIO_LEVEL_HIGH)
    {
        if (++elapsed > TIMEOUT_US) return 0;
    }

    struct timespec t0, t1;
    clock_gettime(CLOCK_MONOTONIC, &t0);
    while (gpio_get_level(pin) == GPIO_LEVEL_LOW) {}
    clock_gettime(CLOCK_MONOTONIC, &t1);

    return (uint32_t)(
        (t1.tv_sec  - t0.tv_sec)  * 1000000UL +
        (t1.tv_nsec - t0.tv_nsec) / 1000UL
    );
}

// Linear map (identical to original map_value in main_header.h).
static long tcs3200_map_value(long x, long in_min, long in_max,
                               long out_min, long out_max)
{
    if (in_max == in_min) return out_min;
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// Clamp a long into [0, 255].
static int tcs3200_clamp255(long v)
{
    if (v < 0)   return 0;
    if (v > 255) return 255;
    return (int)v;
}

// Select a channel (S2/S3) and settle.
// settle=true  → wait SENSOR_SETTLE_MS  (calibration / colour reads)
// settle=false → idempotent re-assert only, no delay  (detect_black fast path)
static void tcs3200_select_channel(int ch, bool settle)
{
    switch (ch)
    {
        case TCS3200_RED:
            gpio_set_level(g_tcs.s2, GPIO_LEVEL_LOW);
            gpio_set_level(g_tcs.s3, GPIO_LEVEL_LOW);
            break;
        case TCS3200_GREEN:
            gpio_set_level(g_tcs.s2, GPIO_LEVEL_HIGH);
            gpio_set_level(g_tcs.s3, GPIO_LEVEL_HIGH);
            break;
        case TCS3200_BLUE:
            gpio_set_level(g_tcs.s2, GPIO_LEVEL_LOW);
            gpio_set_level(g_tcs.s3, GPIO_LEVEL_HIGH);
            break;
        default: // CLEAR
            gpio_set_level(g_tcs.s2, GPIO_LEVEL_HIGH);
            gpio_set_level(g_tcs.s3, GPIO_LEVEL_LOW);
            break;
    }
    if (settle) delay_ms(SENSOR_SETTLE_MS);
}

// Median of n LOW-pulse reads on the already-selected channel.
// Drops timeouts (0).  Returns 0 only if EVERY read timed out
// (maps to white → no false cliff).
static long tcs3200_median_pulse_n(int n)
{
    long s[16];
    int  m = 0;
    if (n > 16) n = 16;
    for (int i = 0; i < n; i++)
    {
        long f = (long)tcs3200_pulseIn_LOW(g_tcs.out);
        if (f > 0) s[m++] = f;
    }
    if (m == 0) return 0;
    for (int i = 1; i < m; i++)   // insertion sort (tiny m)
    {
        long k = s[i]; int j = i - 1;
        while (j >= 0 && s[j] > k) { s[j + 1] = s[j]; j--; }
        s[j + 1] = k;
    }
    return s[m / 2];
}

// Core read on a channel after select+settle.
// Uses median (robust=true) or single-pulse (robust=false).
static long tcs3200_read_raw(int ch)
{
    tcs3200_select_channel(ch, true);
    if (g_tcs.robust)
        return tcs3200_median_pulse_n(g_tcs.integ_samples);
    else
        return (long)tcs3200_pulseIn_LOW(g_tcs.out);
}

// ============================================================
// 7. begin / frequency_scaling / integration_time
// ============================================================

// Set S0/S1 to match a TCS3200_SCALING_* value.
static void tcs3200_apply_scaling(int scaling)
{
    switch (scaling)
    {
        case TCS3200_SCALING_PWR_DOWN:
            gpio_set_level(g_tcs.s0, GPIO_LEVEL_LOW);
            gpio_set_level(g_tcs.s1, GPIO_LEVEL_LOW);
            break;
        case TCS3200_SCALING_2PCT:
            gpio_set_level(g_tcs.s0, GPIO_LEVEL_LOW);
            gpio_set_level(g_tcs.s1, GPIO_LEVEL_HIGH);
            break;
        case TCS3200_SCALING_20PCT:
            gpio_set_level(g_tcs.s0, GPIO_LEVEL_HIGH);
            gpio_set_level(g_tcs.s1, GPIO_LEVEL_LOW);
            break;
        case TCS3200_SCALING_100PCT:
            gpio_set_level(g_tcs.s0, GPIO_LEVEL_HIGH);
            gpio_set_level(g_tcs.s1, GPIO_LEVEL_HIGH);
            break;
        default:   // treat unknown as 20 % (S0=H, S1=L)
            gpio_set_level(g_tcs.s0, GPIO_LEVEL_HIGH);
            gpio_set_level(g_tcs.s1, GPIO_LEVEL_LOW);
            break;
    }
}

// Initialise GPIO directions and apply the default scaling.
// Call once from main(), replacing the inline S0..SENSOR_OUT setup.
static void tcs3200_begin(int s0, int s1, int s2, int s3, int out, int scaling)
{
    g_tcs.s0  = s0;
    g_tcs.s1  = s1;
    g_tcs.s2  = s2;
    g_tcs.s3  = s3;
    g_tcs.out = out;
    g_tcs.scaling = scaling;

    gpio_set_direction(s0,  GPIO_DIR_OUTPUT);
    gpio_set_direction(s1,  GPIO_DIR_OUTPUT);
    gpio_set_direction(s2,  GPIO_DIR_OUTPUT);
    gpio_set_direction(s3,  GPIO_DIR_OUTPUT);
    gpio_set_direction(out, GPIO_DIR_INPUT);

    tcs3200_apply_scaling(scaling);

    // leave CLEAR selected (S2=H, S3=L) — matches detect_black fast-path assumption
    tcs3200_select_channel(TCS3200_CLEAR, false);
}

// frequency_scaling get/set — DRIVES S0/S1 directly (not just stored).
static int  tcs3200_frequency_scaling_get(void) { return g_tcs.scaling; }
static void tcs3200_frequency_scaling(int scaling)
{
    g_tcs.scaling = scaling;
    tcs3200_apply_scaling(scaling);
}

// integration_time get/set — equals the median sample count N.
// Minimum 1; capped at 16 (median_pulse_n array size).
static int  tcs3200_integration_time_get(void) { return g_tcs.integ_samples; }
static void tcs3200_integration_time(int n)
{
    if (n < 1)  n = 1;
    if (n > 16) n = 16;
    g_tcs.integ_samples = n;
}

// Enable robust-median mode (default: true) or literal-single-read mode.
static void tcs3200_set_robust(bool r) { g_tcs.robust = r; }

// ============================================================
// 8. Per-channel 0-255 reads  (select + read + calibrated map)
// ============================================================

static uint8_t tcs3200_read_red(void)
{
    long raw = tcs3200_read_raw(TCS3200_RED);
    return (uint8_t)tcs3200_clamp255(
        tcs3200_map_value(raw, g_tcs.cal_min[TCS3200_RED],
                               g_tcs.cal_max[TCS3200_RED], 255, 0));
}

static uint8_t tcs3200_read_green(void)
{
    long raw = tcs3200_read_raw(TCS3200_GREEN);
    return (uint8_t)tcs3200_clamp255(
        tcs3200_map_value(raw, g_tcs.cal_min[TCS3200_GREEN],
                               g_tcs.cal_max[TCS3200_GREEN], 255, 0));
}

static uint8_t tcs3200_read_blue(void)
{
    long raw = tcs3200_read_raw(TCS3200_BLUE);
    return (uint8_t)tcs3200_clamp255(
        tcs3200_map_value(raw, g_tcs.cal_min[TCS3200_BLUE],
                               g_tcs.cal_max[TCS3200_BLUE], 255, 0));
}

static uint8_t tcs3200_read_clear(void)
{
    long raw = tcs3200_read_raw(TCS3200_CLEAR);
    return (uint8_t)tcs3200_clamp255(
        tcs3200_map_value(raw, g_tcs.cal_min[TCS3200_CLEAR],
                               g_tcs.cal_max[TCS3200_CLEAR], 255, 0));
}

// ============================================================
// 9. FAST CLEAR READ — used exclusively by detect_black.
//    NO settle (channel kept selected), median of DETECT_SAMPLES,
//    returns raw mapped value (not a full RGB read).
//    SAFETY: preserves original detect_black behaviour EXACTLY.
// ============================================================
static int tcs3200_read_clear_fast(void)
{
    // Idempotent re-assert CLEAR (S2=H, S3=L) — no delay.
    gpio_set_level(g_tcs.s2, GPIO_LEVEL_HIGH);
    gpio_set_level(g_tcs.s3, GPIO_LEVEL_LOW);
    long freq = tcs3200_median_pulse_n(DETECT_SAMPLES);
    return tcs3200_clamp255(
        tcs3200_map_value(freq, g_tcs.cal_min[TCS3200_CLEAR],
                                g_tcs.cal_max[TCS3200_CLEAR], 255, 0));
}

// ============================================================
// 10. RGB struct read (with optional white-balance scaling)
// ============================================================
static RGBColor tcs3200_read_rgb(void)
{
    RGBColor c;
    c.r = tcs3200_read_red();
    c.g = tcs3200_read_green();
    c.b = tcs3200_read_blue();

    // Apply white balance if calibrated (any channel non-zero)
    if (g_tcs.white_bal.r || g_tcs.white_bal.g || g_tcs.white_bal.b)
    {
        float sr = g_tcs.white_bal.r ? (255.0f / g_tcs.white_bal.r) : 1.0f;
        float sg = g_tcs.white_bal.g ? (255.0f / g_tcs.white_bal.g) : 1.0f;
        float sb = g_tcs.white_bal.b ? (255.0f / g_tcs.white_bal.b) : 1.0f;
        int rv = (int)(c.r * sr + 0.5f);
        int gv = (int)(c.g * sg + 0.5f);
        int bv = (int)(c.b * sb + 0.5f);
        c.r = (uint8_t)(rv > 255 ? 255 : rv);
        c.g = (uint8_t)(gv > 255 ? 255 : gv);
        c.b = (uint8_t)(bv > 255 ? 255 : bv);
    }
    return c;
}

// ============================================================
// 11. Calibration  (superset of original: median + persistence)
// ============================================================

// Median raw for calibration: select + settle + median(CAL_SAMPLES).
// Matches read_channel_raw from main_header.h exactly.
static long tcs3200_cal_raw(int ch)
{
    tcs3200_select_channel(ch, true);
    return tcs3200_median_pulse_n(CAL_SAMPLES);
}

// Fill cal_max (dark reference) for all channels — sensor pointed at black.
static void tcs3200_calibrate_dark(void)
{
    for (int ch = 0; ch < 4; ch++)
    {
        long v = tcs3200_cal_raw(ch);
        if (v > 0) g_tcs.cal_max[ch] = v;
    }
}

// Fill cal_min (white/light reference) for all channels — sensor pointed at white.
static void tcs3200_calibrate_light(void)
{
    for (int ch = 0; ch < 4; ch++)
    {
        long v = tcs3200_cal_raw(ch);
        if (v > 0) g_tcs.cal_min[ch] = v;
    }
}

// Full calibration sequence: light first, then dark.
// (Matches original calibrate() = calibrate_light() + calibrate_dark().)
static void tcs3200_calibrate(void)
{
    tcs3200_calibrate_light();
    tcs3200_calibrate_dark();
}

// White-balance get/set:  set stores the current RGB as the white reference.
static RGBColor tcs3200_white_balance_get(void) { return g_tcs.white_bal; }
static void     tcs3200_white_balance(void)
{
    // Read raw 0-255 WITHOUT white-balance applied (clear g_tcs.white_bal first).
    g_tcs.white_bal = (RGBColor){0, 0, 0};
    g_tcs.white_bal.r = tcs3200_read_red();
    g_tcs.white_bal.g = tcs3200_read_green();
    g_tcs.white_bal.b = tcs3200_read_blue();
}

// ============================================================
// 12. Colour-space conversions  (direct from nthnn/TCS3200 math)
// ============================================================

static HSVColor tcs3200_read_hsv(void)
{
    RGBColor rgb = tcs3200_read_rgb();
    float r = rgb.r / 255.0f;
    float g = rgb.g / 255.0f;
    float b = rgb.b / 255.0f;

    float cmax = r > g ? (r > b ? r : b) : (g > b ? g : b);
    float cmin = r < g ? (r < b ? r : b) : (g < b ? g : b);
    float delta = cmax - cmin;

    HSVColor hsv;
    hsv.v = cmax;
    hsv.s = (cmax > 1e-6f) ? (delta / cmax) : 0.0f;

    if (delta < 1e-6f)
    {
        hsv.h = 0.0f;
    }
    else if (cmax == r)
    {
        hsv.h = 60.0f * fmodf((g - b) / delta, 6.0f);
    }
    else if (cmax == g)
    {
        hsv.h = 60.0f * ((b - r) / delta + 2.0f);
    }
    else
    {
        hsv.h = 60.0f * ((r - g) / delta + 4.0f);
    }
    if (hsv.h < 0.0f) hsv.h += 360.0f;

    return hsv;
}

static CMYKColor tcs3200_read_cmyk(void)
{
    RGBColor rgb = tcs3200_read_rgb();
    float r = rgb.r / 255.0f;
    float g = rgb.g / 255.0f;
    float b = rgb.b / 255.0f;

    float k = 1.0f - (r > g ? (r > b ? r : b) : (g > b ? g : b));
    CMYKColor cmyk;
    if (k >= 1.0f - 1e-6f)
    {
        cmyk.c = 0.0f; cmyk.m = 0.0f; cmyk.y = 0.0f; cmyk.k = 1.0f;
    }
    else
    {
        float inv = 1.0f / (1.0f - k);
        cmyk.c = (1.0f - r - k) * inv;
        cmyk.m = (1.0f - g - k) * inv;
        cmyk.y = (1.0f - b - k) * inv;
        cmyk.k = k;
    }
    return cmyk;
}

// sRGB linearisation + ITU-R BT.709 / CIE XYZ D65 matrix
// (same as the original nthnn/TCS3200 port).
static CIE1931Color tcs3200_read_cie1931(void)
{
    RGBColor rgb = tcs3200_read_rgb();

    // sRGB → linear
    float r = rgb.r / 255.0f;
    float g = rgb.g / 255.0f;
    float b = rgb.b / 255.0f;

    r = (r <= 0.04045f) ? (r / 12.92f) : powf((r + 0.055f) / 1.055f, 2.4f);
    g = (g <= 0.04045f) ? (g / 12.92f) : powf((g + 0.055f) / 1.055f, 2.4f);
    b = (b <= 0.04045f) ? (b / 12.92f) : powf((b + 0.055f) / 1.055f, 2.4f);

    CIE1931Color cie;
    cie.x = r * 0.4124564f + g * 0.3575761f + b * 0.1804375f;
    cie.y = r * 0.2126729f + g * 0.7151522f + b * 0.0721750f;
    cie.z = r * 0.0193339f + g * 0.1191920f + b * 0.9503041f;
    return cie;
}

// Chroma = Euclidean distance in CIE-XYZ from the D65 white point.
// (Matches the original nthnn/TCS3200: chroma =
//  sqrt((x-0.95047)^2 + (y-1.0)^2 + (z-1.08883)^2), (x,y,z)=read_cie1931().)
static float tcs3200_get_chroma(void)
{
    CIE1931Color cie = tcs3200_read_cie1931();
    float dx = cie.x - 0.95047f;
    float dy = cie.y - 1.0f;
    float dz = cie.z - 1.08883f;
    return sqrtf(dx*dx + dy*dy + dz*dz);
}

// Dominant channel: 0=R, 1=G, 2=B.
static int tcs3200_dominant(void)
{
    RGBColor rgb = tcs3200_read_rgb();
    if (rgb.r >= rgb.g && rgb.r >= rgb.b) return TCS3200_RED;
    if (rgb.g >= rgb.r && rgb.g >= rgb.b) return TCS3200_GREEN;
    return TCS3200_BLUE;
}

// ============================================================
// 13. Monitor (loop) + threshold interrupts
// ============================================================

// Run one monitor cycle: read RGB; fire callbacks when all channels
// cross their respective thresholds.
static void tcs3200_loop(void)
{
    RGBColor c = tcs3200_read_rgb();

    if (g_tcs.up_cb &&
        c.r >= g_tcs.up_thr.r &&
        c.g >= g_tcs.up_thr.g &&
        c.b >= g_tcs.up_thr.b)
    {
        g_tcs.up_cb();
    }

    if (g_tcs.lo_cb &&
        c.r <= g_tcs.lo_thr.r &&
        c.g <= g_tcs.lo_thr.g &&
        c.b <= g_tcs.lo_thr.b)
    {
        g_tcs.lo_cb();
    }
}

static void tcs3200_upper_bound_interrupt(RGBColor thr, tcs3200_cb cb)
{
    g_tcs.up_thr = thr;
    g_tcs.up_cb  = cb;
}

static void tcs3200_lower_bound_interrupt(RGBColor thr, tcs3200_cb cb)
{
    g_tcs.lo_thr = thr;
    g_tcs.lo_cb  = cb;
}

static void tcs3200_clear_upper_bound_interrupt(void)
{
    g_tcs.up_cb = NULL;
    g_tcs.up_thr = (RGBColor){0, 0, 0};
}

static void tcs3200_clear_lower_bound_interrupt(void)
{
    g_tcs.lo_cb = NULL;
    g_tcs.lo_thr = (RGBColor){0, 0, 0};
}

// ============================================================
// 14. Nearest-colour  (C analogue of the C++ template)
//     refs  — array of RGBColor reference colours
//     ids   — parallel array of integer label IDs
//     n     — number of entries
//     Returns the ID whose reference is closest to the current reading
//             by Euclidean RGB distance.
// ============================================================
static int tcs3200_nearest_color(const RGBColor refs[], const int ids[], int n)
{
    if (n <= 0) return -1;
    RGBColor c = tcs3200_read_rgb();
    int   best_id   = ids[0];
    float best_dist = 1e30f;
    for (int i = 0; i < n; i++)
    {
        float dr = (float)c.r - refs[i].r;
        float dg = (float)c.g - refs[i].g;
        float db = (float)c.b - refs[i].b;
        float d  = dr*dr + dg*dg + db*db;
        if (d < best_dist) { best_dist = d; best_id = ids[i]; }
    }
    return best_id;
}

// ============================================================
// 15. Calibration persistence (superset: not in the original)
//     Used by main_header.h CALBLACK / cal_save / cal_load.
// ============================================================

// Expose the global cal arrays for use by CALBLACK in main.c.
// (Legacy macros cal_min / cal_max are now aliases into g_tcs.)
#define cal_min      (g_tcs.cal_min)
#define cal_max      (g_tcs.cal_max)
#define cal_black_thresh  g_cal_black_thresh
static int g_cal_black_thresh = 150;

// CAL_CLEAR is still channel 3.
#ifndef CAL_CLEAR
#define CAL_CLEAR TCS3200_CLEAR
#endif
