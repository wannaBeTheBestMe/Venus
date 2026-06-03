#include <libpynq.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>

/* ─── Distance sensors ──────────────────────────────────────────────────── */
#define SYSRANGE_START        0x00
#define XSHUT_PIN_1           IO_AR0
#define XSHUT_PIN_2           IO_AR1
#define CALIBRATION_OFFSET_MM -27
#define DEFAULT_ADDRESS       0x29
#define SENSOR1_ADDRESS       0x30
#define SENSOR2_ADDRESS       0x31
#define REG_ADDR_CONFIG       0x8A
#define MAX_RETRIES           10
#define RETRY_DELAY_MS        200

/* ─── TCS34725 registers ────────────────────────────────────────────────── */
#define TCS_ADDR    0x29
#define REG_ENABLE  0x80
#define REG_ATIME   0x81
#define REG_WTIME   0x83
#define REG_CONTROL 0x8F
#define REG_ID      0x92
#define REG_CDATAL  0x94

/* ─── Color sensor settings ─────────────────────────────────────────────── */
#define ATIME   0x00
#define WTIME   0xFF
#define GAIN    0x00

#define WARMUP_SAMPLES  8
#define WHITE_FACTOR    7.00f
#define BLACK_FACTOR    0.30f

#define LUX_R  (-0.32466f)
#define LUX_G  ( 1.57837f)
#define LUX_B  (-0.73191f)

/* ─── Calibration state ─────────────────────────────────────────────────── */
typedef struct {
    float r_white_ratio;
    float g_white_ratio;
    float b_white_ratio;
    float clear_white;
    float clear_black;
} Cal;

/* ═══════════════════════════════════════════════════════════════════════════
 * Distance sensor helpers
 * ═══════════════════════════════════════════════════════════════════════════ */
bool init_sensor(uint8_t addr)
{
    uint8_t id = 0x00;
    if (iic_read_register(IIC0, addr, 0xC0, &id, 1))
    {
        printf("ERROR: cannot read distance sensor at 0x%02X\n", addr);
        return false;
    }
    if (id != 0xEE)
        printf("WARNING: unexpected sensor ID 0x%02X at 0x%02X (expected 0xEE)\n", id, addr);
    else
        printf("Distance sensor ID 0x%02X at address 0x%02X — OK\n", id, addr);

    uint8_t zero = 0x00;
    iic_write_register(IIC0, addr, 0x28, &zero, 1);

    uint8_t start = 0x01;
    if (iic_write_register(IIC0, addr, SYSRANGE_START, &start, 1))
    {
        printf("Failed to start distance sensor at 0x%02X\n", addr);
        return false;
    }

    printf("Distance sensor running on address 0x%02X\n", addr);
    return true;
}

bool change_address(uint8_t from, uint8_t to)
{
    printf("Attempting to change address 0x%02X -> 0x%02X...\n", from, to);
    uint8_t new_addr_data = to;
    bool failed = true;
    for (int attempt = 1; attempt <= MAX_RETRIES; attempt++)
    {
        failed = iic_write_register(IIC0, from, REG_ADDR_CONFIG, &new_addr_data, 1);
        if (!failed)
        {
            printf("Success! Sensor moved to 0x%02X\n", to);
            uint8_t readback = 0x00;
            if (!iic_read_register(IIC0, to, REG_ADDR_CONFIG, &readback, 1))
                printf("Address readback: 0x%02X — %s\n", readback,
                       readback == to ? "MATCH" : "MISMATCH");
            else
                printf("Could not read back address register from 0x%02X\n", to);
            return true;
        }
        printf("[%d/%d] Failed, retrying in %d ms...\n", attempt, MAX_RETRIES, RETRY_DELAY_MS);
        sleep_msec(RETRY_DELAY_MS);
    }
    printf("Error: Could not change address after %d attempts.\n", MAX_RETRIES);
    return false;
}

void read_distance_sensors(void)
{
    uint8_t trig = 0x01;
    iic_write_register(IIC0, SENSOR1_ADDRESS, 0x00, &trig, 1);
    iic_write_register(IIC0, SENSOR2_ADDRESS, 0x00, &trig, 1);
    sleep_msec(50);

    uint8_t data1[2], data2[2];
    int32_t dist1 = -1, dist2 = -1;

    if (!iic_read_register(IIC0, SENSOR1_ADDRESS, 0x1E, data1, 2))
    {
        dist1 = (int32_t)((data1[0] << 8) | data1[1]) + CALIBRATION_OFFSET_MM;
        if (dist1 < 0) dist1 = 0;
    }

    if (!iic_read_register(IIC0, SENSOR2_ADDRESS, 0x1E, data2, 2))
    {
        dist2 = (int32_t)((data2[0] << 8) | data2[1]) + CALIBRATION_OFFSET_MM;
        if (dist2 < 0) dist2 = 0;
    }

    printf("Dist1 (0x30): %4d mm  |  Dist2 (0x31): %4d mm", (int)dist1, (int)dist2);

    uint8_t clear = 0x01;
    iic_write_register(IIC0, SENSOR1_ADDRESS, 0x0B, &clear, 1);
    iic_write_register(IIC0, SENSOR2_ADDRESS, 0x0B, &clear, 1);
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Color sensor helpers
 * ═══════════════════════════════════════════════════════════════════════════ */
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

    printf("  Ambient: R=%.0f G=%.0f B=%.0f C=%.0f\n", avg_r, avg_g, avg_b, avg_c);

    Cal cal;
    cal.r_white_ratio = avg_r / avg_c;
    cal.g_white_ratio = avg_g / avg_c;
    cal.b_white_ratio = avg_b / avg_c;
    cal.clear_white   = avg_c * WHITE_FACTOR;
    cal.clear_black   = avg_c * BLACK_FACTOR;

    printf("  r_white_ratio=%.4f  g_white_ratio=%.4f  b_white_ratio=%.4f\n",
           cal.r_white_ratio, cal.g_white_ratio, cal.b_white_ratio);
    printf("  clear_black=%.0f  clear_white=%.0f\n", cal.clear_black, cal.clear_white);
    printf("Warmup done.\n\n");

    return cal;
}

static void apply_cal(const Cal *cal,
                      uint16_t r, uint16_t g, uint16_t b, uint16_t c,
                      int *r_out, int *g_out, int *b_out)
{
    float fc = (float)c;

    if (fc >= cal->clear_white) {
        *r_out = 255; *g_out = 255; *b_out = 255;
        return;
    }

    float range = cal->clear_white - cal->clear_black;
    float luma  = (fc - cal->clear_black) / range;
    if (luma < 0.0f) luma = 0.0f;
    if (luma > 1.0f) luma = 1.0f;

    float r_rel = (cal->r_white_ratio > 0.0f)
                  ? (((float)r / fc) / cal->r_white_ratio) : 0.0f;
    float g_rel = (cal->g_white_ratio > 0.0f)
                  ? (((float)g / fc) / cal->g_white_ratio) : 0.0f;
    float b_rel = (cal->b_white_ratio > 0.0f)
                  ? (((float)b / fc) / cal->b_white_ratio) : 0.0f;

    *r_out = clamp255(r_rel * luma * 255.0f);
    *g_out = clamp255(g_rel * luma * 255.0f);
    *b_out = clamp255(b_rel * luma * 255.0f);
}

/* ═══════════════════════════════════════════════════════════════════════════
 * main
 * ═══════════════════════════════════════════════════════════════════════════ */
int main(void)
{
    printf("System Start\n");
    pynq_init();

    gpio_set_direction(XSHUT_PIN_1, GPIO_DIR_OUTPUT);
    gpio_set_direction(XSHUT_PIN_2, GPIO_DIR_OUTPUT);

    switchbox_set_pin(IO_AR_SCL, SWB_IIC0_SCL);
    switchbox_set_pin(IO_AR_SDA, SWB_IIC0_SDA);
    iic_init(IIC0);

    /* ── Distance sensor setup ── */

    // Hold both in reset
    gpio_set_level(XSHUT_PIN_1, GPIO_LEVEL_LOW);
    gpio_set_level(XSHUT_PIN_2, GPIO_LEVEL_LOW);
    sleep_msec(10);

    // Wake sensor 1, move to 0x30
    printf("\n--- Initializing distance sensor 1 ---\n");
    gpio_set_level(XSHUT_PIN_1, GPIO_LEVEL_HIGH);
    sleep_msec(10);
    if (!change_address(DEFAULT_ADDRESS, SENSOR1_ADDRESS))
    {
        pynq_destroy();
        return EXIT_FAILURE;
    }

    // Wake sensor 2, move to 0x31
    printf("\n--- Initializing distance sensor 2 ---\n");
    gpio_set_level(XSHUT_PIN_2, GPIO_LEVEL_HIGH);
    sleep_msec(10);
    if (!change_address(DEFAULT_ADDRESS, SENSOR2_ADDRESS))
    {
        pynq_destroy();
        return EXIT_FAILURE;
    }

    // Init ranging on both
    if (!init_sensor(SENSOR1_ADDRESS) || !init_sensor(SENSOR2_ADDRESS))
    {
        pynq_destroy();
        return EXIT_FAILURE;
    }

    /* ── Color sensor setup — 0x29 is now free ── */
    printf("\n--- Initializing color sensor ---\n");
    uint8_t id = 0;
    if (iic_read_register(IIC0, TCS_ADDR, REG_ID, &id, 1)) {
        printf("ERROR: cannot read color sensor — check wiring\n");
        pynq_destroy();
        return EXIT_FAILURE;
    }
    if (id != 0x44 && id != 0x4D)
        printf("WARNING: unexpected color sensor ID 0x%02X\n", id);
    else
        printf("Color sensor ID 0x%02X — OK\n", id);

    write8(REG_ENABLE,  0x03);
    write8(REG_ATIME,   ATIME);
    write8(REG_WTIME,   WTIME);
    write8(REG_CONTROL, GAIN);
    sleep_msec(1300);

    Cal cal = run_warmup();

    /* ── Main loop ── */
    printf("\n--- All sensors running ---\n");
    while (1)
    {
        // Distance
        read_distance_sensors();

        // Color
        uint16_t c, r, g, b;
        if (read_all(&c, &r, &g, &b) != 0) {
            printf("  | Color: ERROR\n");
            sleep_msec(500);
            continue;
        }
        if (c >= 65535) {
            printf("  | Color: SATURATED\n");
            sleep_msec(500);
            continue;
        }
        if (c < (uint16_t)cal.clear_black) {
            printf("  | Color: TOO DARK\n");
            sleep_msec(500);
            continue;
        }

        float lux = LUX_R*(float)r + LUX_G*(float)g + LUX_B*(float)b;
        if (lux < 0.0f) lux = 0.0f;

        int r_out, g_out, b_out;
        apply_cal(&cal, r, g, b, c, &r_out, &g_out, &b_out);

        printf("  | Lux=%7.1f  CAL R=%3d G=%3d B=%3d\n",
               lux, r_out, g_out, b_out);

        sleep_msec(100);
    }

    pynq_destroy();
    return EXIT_SUCCESS;
}