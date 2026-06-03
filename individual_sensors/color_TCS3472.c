#include <libpynq.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

/* ─── TCS34725 registers ────────────────────────────────────────────────── */
#define TCS_ADDR    0x29
#define REG_ENABLE  0x80
#define REG_ATIME   0x81
#define REG_WTIME   0x83
#define REG_CONTROL 0x8F
#define REG_ID      0x92
#define REG_CDATAL  0x94

/* ─── Sensor settings ───────────────────────────────────────────────────── */
#define ATIME   0x00    /* 614.4 ms integration                              */
#define WTIME   0xFF    /* 2.4 ms wait (WEN not set, informational only)     */
#define GAIN    0x00    /* 1x gain                                           */

/* ─── Warmup ────────────────────────────────────────────────────────────── */
/*
 * Number of samples averaged during open-air warmup.
 * At 614 ms/sample, 8 samples ≈ 5 seconds.
 */
#define WARMUP_SAMPLES  8

/*
 * WHITE_FACTOR — defines the clear-channel ceiling above which a reading
 * is unconditionally treated as pure white (255 255 255).
 *
 */
#define WHITE_FACTOR  7.00f

/*
 * BLACK_FACTOR — fraction of ambient clear that defines "black".
 */
#define BLACK_FACTOR  0.30f

/* ─── Lux coefficients (TCS34725 datasheet) ─────────────────────────────── */
#define LUX_R  (-0.32466f)
#define LUX_G  ( 1.57837f)
#define LUX_B  (-0.73191f)

/* ─── Calibration state ─────────────────────────────────────────────────── */
typedef struct {
    /*
     */
    float r_white_ratio;   /* R/C on white surface (from ambient)           */
    float g_white_ratio;   /* G/C on white surface (from ambient)           */
    float b_white_ratio;   /* B/C on white surface (from ambient)           */

    /* Clear channel brightness bounds */
    float clear_white;     /* ceiling: at or above this → 255 255 255       */
    float clear_black;     /* floor: below this → TOO DARK                  */
} Cal;

/* ─── Helpers ───────────────────────────────────────────────────────────── */
static void write8(uint8_t reg, uint8_t val)
{
    iic_write_register(IIC0, TCS_ADDR, reg, &val, 1);
}

static int read_all(uint16_t *c, uint16_t *r, uint16_t *g, uint16_t *b)
{
    uint8_t d[8] = {0};
    if (iic_read_register(IIC0, TCS_ADDR, REG_CDATAL, d, 8) != 0)
        return 1;
    *c = (uint16_t)(d[0] | (d[1] << 8));
    *r = (uint16_t)(d[2] | (d[3] << 8));
    *g = (uint16_t)(d[4] | (d[5] << 8));
    *b = (uint16_t)(d[6] | (d[7] << 8));
    return 0;
}

static int clamp255(float v)
{
    if (v <   0.0f) return 0;
    if (v > 255.0f) return 255;
    return (int)v;
}

/* ─── Warmup ────────────────────────────────────────────────────────────── */
static Cal run_warmup(void)
{
    printf("Warmup: point sensor at open air, keep still...\n");

    uint32_t sum_r = 0, sum_g = 0, sum_b = 0, sum_c = 0;
    int valid = 0;

    while (valid < WARMUP_SAMPLES) {
        uint16_t c, r, g, b;
        if (read_all(&c, &r, &g, &b) != 0) {
            printf("  read error, retrying...\n");
            sleep_msec(200);
            continue;
        }
        if (c >= 65535) {
            printf("  saturated — reduce gain\n");
            sleep_msec(200);
            continue;
        }
        if (c == 0) {
            printf("  no light — waiting...\n");
            sleep_msec(500);
            continue;
        }
        sum_r += r; sum_g += g; sum_b += b; sum_c += c;
        valid++;
        printf("  sample %d/%d  R=%5u G=%5u B=%5u C=%5u\n",
               valid, WARMUP_SAMPLES, r, g, b, c);
    }

    float avg_r = (float)sum_r / WARMUP_SAMPLES;
    float avg_g = (float)sum_g / WARMUP_SAMPLES;
    float avg_b = (float)sum_b / WARMUP_SAMPLES;
    float avg_c = (float)sum_c / WARMUP_SAMPLES;

    printf("  Ambient: R=%.0f G=%.0f B=%.0f C=%.0f\n",
           avg_r, avg_g, avg_b, avg_c);

    Cal cal;

    /*
     */
    cal.r_white_ratio = avg_r / avg_c;
    cal.g_white_ratio = avg_g / avg_c;
    cal.b_white_ratio = avg_b / avg_c;

    /*
     * Clear channel bounds.
     * clear_white: anything at or above this threshold is forced to 255 255 255.
     *   WHITE_FACTOR = 1.80 means "80% above ambient" is the hard white ceiling.
     * clear_black: black surfaces absorb ~90% → clear ≈ 10% of ambient.
     */
    cal.clear_white = avg_c * WHITE_FACTOR;
    cal.clear_black = avg_c * BLACK_FACTOR;

    printf("  r_white_ratio=%.4f  g_white_ratio=%.4f  b_white_ratio=%.4f\n",
           cal.r_white_ratio, cal.g_white_ratio, cal.b_white_ratio);
    printf("  clear_black=%.0f  clear_white=%.0f\n",
           cal.clear_black, cal.clear_white);
    printf("Warmup done.\n\n");

    return cal;
}

/* ─── Apply calibration ─────────────────────────────────────────────────── */
static void apply_cal(const Cal *cal,
                       uint16_t r, uint16_t g, uint16_t b, uint16_t c,
                       int *r_out, int *g_out, int *b_out)
{
    float fc = (float)c;

    /*
     * STEP 0 — Hard white clamp.
     */
    if (fc >= cal->clear_white) {
        *r_out = 255;
        *g_out = 255;
        *b_out = 255;
        return;
    }

    /*
     * STEP 1 — Luminance from clear channel.
     *
     */
    float range = cal->clear_white - cal->clear_black;
    float luma  = (fc - cal->clear_black) / range;
    if (luma < 0.0f) luma = 0.0f;
    if (luma > 1.0f) luma = 1.0f;

    /*
     * STEP 2 — Per-channel ratio relative to white reference.
     *
     */
    float r_rel = (cal->r_white_ratio > 0.0f)
                  ? (((float)r / fc) / cal->r_white_ratio)
                  : 0.0f;
    float g_rel = (cal->g_white_ratio > 0.0f)
                  ? (((float)g / fc) / cal->g_white_ratio)
                  : 0.0f;
    float b_rel = (cal->b_white_ratio > 0.0f)
                  ? (((float)b / fc) / cal->b_white_ratio)
                  : 0.0f;

    /*
     * STEP 3 — Scale to 0-255 and apply luminance.
     *
     */
    *r_out = clamp255(r_rel * luma * 255.0f);
    *g_out = clamp255(g_rel * luma * 255.0f);
    *b_out = clamp255(b_rel * luma * 255.0f);
}

/* ═══════════════════════════════════════════════════════════════════════════
 * main
 * ═══════════════════════════════════════════════════════════════════════════ */
int main(void)
{
    pynq_init();
    switchbox_set_pin(IO_AR_SCL, SWB_IIC0_SCL);
    switchbox_set_pin(IO_AR_SDA, SWB_IIC0_SDA);
    iic_init(IIC0);

    uint8_t id = 0;
    if (iic_read_register(IIC0, TCS_ADDR, REG_ID, &id, 1)) {
        printf("ERROR: cannot read sensor — check wiring\n");
        pynq_destroy();
        return EXIT_FAILURE;
    }
    if (id != 0x44 && id != 0x4D)
        printf("WARNING: unexpected sensor ID 0x%02X\n", id);
    else
        printf("Sensor ID 0x%02X — OK\n", id);

    write8(REG_ENABLE,  0x03);
    write8(REG_ATIME,   ATIME);
    write8(REG_WTIME,   WTIME);
    write8(REG_CONTROL, GAIN);
    sleep_msec(1300);

    Cal cal = run_warmup();

    while (1) {
        uint16_t c, r, g, b;

        if (read_all(&c, &r, &g, &b) != 0) {
            printf("ERROR: read failed\n");
            sleep_msec(500);
            continue;
        }

        if (c >= 65535) {
            printf("SATURATED — reduce gain\n");
            sleep_msec(500);
            continue;
        }

        if (c < (uint16_t)cal.clear_black) {
            printf("TOO DARK (clear=%u < floor %.0f)\n", c, cal.clear_black);
            sleep_msec(500);
            continue;
        }

        float lux = LUX_R*(float)r + LUX_G*(float)g + LUX_B*(float)b;
        if (lux < 0.0f) lux = 0.0f;

        int r_out, g_out, b_out;
        apply_cal(&cal, r, g, b, c, &r_out, &g_out, &b_out);

        printf("RAW R=%5u G=%5u B=%5u C=%5u  "
               "Lux=%7.1f  "
               "CAL R=%3d G=%3d B=%3d\n",
               r, g, b, c, lux, r_out, g_out, b_out);

        sleep_msec(100);
    }

    pynq_destroy();
    return EXIT_SUCCESS;
}