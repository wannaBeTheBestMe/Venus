#pragma once
#include <libpynq.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

/* ─── Temperature (thermistor on ADC0) ──────────────────────────────────── */
// From Venus/individual_sensors/temperature.c: ADC voltage divider -> resistance
// -> Steinhart-Hart -> degrees C. Requires adc_init() at startup.
#define TEMP_INVALID  (-100.0f)   // sentinel: no valid reading

static int r_to_t(double r_t)
{
    const double A = 0.0007984;
    const double B = 0.0002664;
    const double C = 0.00000013009;
    double lnR = log(r_t * 1000);
    double t = 1.0 / (A + B * lnR + C * lnR * lnR * lnR);
    return (int)(t - 273.15);
}

// Average a few ADC samples; returns degrees C, or TEMP_INVALID if no usable read.
static float read_temperature(void)
{
    const double v_ref = 3.3, R2 = 0.33;
    double sum = 0.0;
    int n = 0;
    for (int i = 0; i < 8; i++)
    {
        double v_out = adc_read_channel(ADC0);
        if (v_out > 0.05 && v_out < v_ref)          // guard div-by-zero / rail
        {
            double r_t = (v_ref - v_out) * (R2 / v_out);
            int t = r_to_t(r_t);
            if (t > -20 && t < 120) { sum += t; n++; }   // drop implausible
        }
        sleep_msec(5);
    }
    return (n > 0) ? (float)(sum / n) : TEMP_INVALID;
}

/* ─── Distance sensors ──────────────────────────────────────────────────── */
#define SYSRANGE_START        0x00
#define XSHUT_PIN_1           IO_AR10
#define XSHUT_PIN_2           IO_AR9
#define CALIBRATION_OFFSET_MM -27
#define DEFAULT_ADDRESS       0x29
#define SENSOR1_ADDRESS       0x30
#define SENSOR2_ADDRESS       0x31
#define REG_ADDR_CONFIG       0x8A
#define MAX_RETRIES           10
#define RETRY_DELAY_MS        200

/* ─── Dual-backend globals (XSHUT vs PCA9548A mux) ─────────────────────── */
// Resolved ONCE in all_sensors_init(); zero per-read overhead beyond the
// mux's own channel-select write. PCA9548A_ADDR / SENSOR_ADDR / TCS_REG_*
// / VL53_REG_ID are defined in main_header.h BEFORE this file is included,
// so they are visible here.
enum { BACKEND_XSHUT = 0, BACKEND_MUX = 1 };
static int g_sensor_backend = BACKEND_XSHUT;

// Mux channel assignments. -1 = not found / not applicable (XSHUT path).
static int g_ch_forward  = -1;   // forward VL53 mux channel
static int g_ch_overhead = -1;   // overhead VL53 mux channel
static int g_ch_color    = -1;   // TCS34725 mux channel

// Probe one byte from an I2C address: returns 0 (ACK) or nonzero (NAK/error).
// Used to detect the mux at boot.
static int mux_probe(uint8_t addr)
{
    uint8_t dummy = 0;
    return iic_read_register(IIC0, addr, 0x00, &dummy, 1);
}

// Select a single mux channel (write 1<<ch to the mux control register).
// Returns 0 on success, nonzero on I2C error.
static int mux_select_channel(uint8_t ch)
{
    uint8_t mux_state = (uint8_t)(1u << ch);
    return iic_write_register(IIC0, PCA9548A_ADDR, 0x00, &mux_state, 1);
}

// Scan all 8 mux channels, probing for TCS34725 (color) and VL53L0X (distance).
// Forward role = lower-numbered VL53 channel; overhead = higher.
// Writes results into *vl_lo, *vl_hi, *col (-1 = not found).
static void mux_scan_vl53_color(int *vl_lo, int *vl_hi, int *col)
{
    *vl_lo = -1; *vl_hi = -1; *col = -1;
    int vl53_found = 0;   // count of VL53s seen so far

    for (int ch = 0; ch < 8; ch++)
    {
        if (mux_select_channel((uint8_t)ch) != 0)
            continue;   // channel select failed; skip
        sleep_msec(10);

        uint8_t id = 0;

        // Probe for TCS34725 color sensor
        if (*col < 0 &&
            iic_read_register(IIC0, SENSOR_ADDR, TCS_REG_ID, &id, 1) == 0)
        {
            if (id == 0x44 || id == 0x4D)
            {
                *col = ch;
                printf("MUX scan: TCS34725 (color) ch%d ID=0x%02X\n", ch, id);
            }
        }

        // Probe for VL53L0X distance sensor
        id = 0;
        if (iic_read_register(IIC0, SENSOR_ADDR, VL53_REG_ID, &id, 1) == 0)
        {
            if (id == 0xEE)
            {
                if (vl53_found == 0) { *vl_lo = ch; }
                else if (vl53_found == 1) { *vl_hi = ch; }
                printf("MUX scan: VL53L0X #%d ch%d ID=0x%02X\n", vl53_found + 1, ch, id);
                vl53_found++;
            }
        }
    }

    // Deselect all channels
    uint8_t off = 0;
    iic_write_register(IIC0, PCA9548A_ADDR, 0x00, &off, 1);

    printf("MUX scan result: vl_lo(fwd)=ch%d  vl_hi(oh)=ch%d  color=ch%d\n",
           *vl_lo, *vl_hi, *col);
}

// Shared VL53L0X transport: trigger → wait → read 0x1E → clear interrupt.
// MUX path: selects ch first (SENSOR_ADDR 0x29); XSHUT path: uses addr directly.
// Returns raw distance in mm, or -1 on any I2C error.
static int32_t vl53_read_raw_mm(uint8_t addr, int ch)
{
    if (g_sensor_backend == BACKEND_MUX)
    {
        if (ch < 0) return -1;
        if (mux_select_channel((uint8_t)ch) != 0) return -1;
        addr = SENSOR_ADDR;   // all mux sensors share 0x29
    }

    uint8_t trig = 0x01;
    if (iic_write_register(IIC0, addr, 0x00, &trig, 1)) return -1;

    // MUX sensors need a slightly longer settle (60 ms per Motors/ reference);
    // XSHUT sensors are fine at 30 ms (existing behaviour preserved).
    sleep_msec((g_sensor_backend == BACKEND_MUX) ? 60 : 30);

    uint8_t data[2];
    if (iic_read_register(IIC0, addr, 0x1E, data, 2)) return -1;

    int32_t raw = (int32_t)((data[0] << 8) | data[1]);

    // Clear interrupt flag
    uint8_t clr = 0x01;
    iic_write_register(IIC0, addr, 0x0B, &clr, 1);

    return raw;
}

// #define INTER_READING_DELAY_DIST_FORW_MS 5
// #define INTER_READING_DELAY_DIST_OVERH_MS 40

// #define SENSOR_TIMEOUT_MS 100

#define MAX_FORWARD_DELTA_MM 100

#define MAX_RANGE_MM 500 // For the forward dist sensor
#define MAX_OVERHEAD_RANGE_MM 90

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
#define BLACK_FACTOR    0.10f

#define LUX_R  (-0.32466f)
#define LUX_G  ( 1.57837f)
#define LUX_B  (-0.73191f)

typedef struct {
    float lux;
    int r, g, b;
    bool valid;
} ColorReading;

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

// int32_t read_distance_forward(void)
// {
//     uint8_t trig = 0x01;
//     if (iic_write_register(IIC0, SENSOR2_ADDRESS, 0x00, &trig, 1))
//     {
//         printf("  | Forward dist: SENSOR DISCONNECTED\n");
//         return -1;
//     }
// 
//     sleep_msec(30);
// 
//     uint8_t data[2];
//     if (iic_read_register(IIC0, SENSOR2_ADDRESS, 0x1E, data, 2))
//     {
//         printf("  | Forward dist: SENSOR DISCONNECTED\n");
//         return -1;
//     }
// 
//     int32_t dist = (int32_t)((data[0] << 8) | data[1]) + CALIBRATION_OFFSET_MM;
// 
//     if (dist < 0 || dist > MAX_RANGE_MM)
//     {
//         printf("  | Forward dist: OUT OF RANGE\n");
//         return -1;
//     }
// 
//     uint8_t clear = 0x01;
//     iic_write_register(IIC0, SENSOR2_ADDRESS, 0x0B, &clear, 1);
// 
//     printf("  | Forward dist: %4d mm\n", (int)dist);
//     return dist;
// }

int32_t read_distance_forward(void)
{
    static int32_t last_valid = -1;

    int32_t raw = vl53_read_raw_mm(SENSOR2_ADDRESS, g_ch_forward);
    if (raw < 0)
    {
        printf("  | Forward dist: SENSOR DISCONNECTED\n");
        return -1;
    }

    int32_t dist = raw + CALIBRATION_OFFSET_MM;

    if (dist < 0 || dist > MAX_RANGE_MM)
    {
        printf("  | Forward dist: OUT OF RANGE\n");
        last_valid = -1;
        return -1;
    }

    // Reject implausible jumps
    if (last_valid >= 0 && abs(dist - last_valid) > MAX_FORWARD_DELTA_MM)
    {
        printf("  | Forward dist: REJECTED (jump %d -> %d mm)\n",
               (int)last_valid, (int)dist);
        return -1;
    }

    last_valid = dist;
    printf("  | Forward dist: %4d mm\n", (int)dist);
    return dist;
}

// Raw forward distance: same read + range check, but NO delta-rejection and no
// last_valid state. Used by the object sweep, where the distance legitimately jumps
// as the beam pans across objects at different ranges (the sweep does its own
// persistence/segmentation filtering instead). Returns mm, or -1 if invalid/out of range.
int32_t read_distance_forward_raw(void)
{
    int32_t raw = vl53_read_raw_mm(SENSOR2_ADDRESS, g_ch_forward);
    if (raw < 0) return -1;

    int32_t dist = raw + CALIBRATION_OFFSET_MM;

    if (dist < 0 || dist > MAX_RANGE_MM) return -1;
    return dist;
}

struct dist_oh_t {
	int32_t dist_oh;
	bool flag_black;
};

int32_t read_distance_overhead_simple(void)
{
    int32_t raw = vl53_read_raw_mm(SENSOR1_ADDRESS, g_ch_overhead);
    if (raw < 0)
    {
        printf("  | Overhead dist: SENSOR DISCONNECTED\n");
        return -1;
    }

    int32_t dist = raw + CALIBRATION_OFFSET_MM;

    if (dist < 0 || dist > MAX_OVERHEAD_RANGE_MM)
    {
        printf("  | Overhead dist: OUT OF RANGE\n");
        return -1;
    }

    printf("  | Overhead dist: %4d mm\n", (int)dist);
    return dist;
}

struct dist_oh_t read_distance_overhead(void)
{
	struct dist_oh_t my_dist_oh;
	my_dist_oh.dist_oh = -1;
	my_dist_oh.flag_black = false;

    int32_t raw = vl53_read_raw_mm(SENSOR1_ADDRESS, g_ch_overhead);
    if (raw < 0)
    {
        printf("  | Overhead dist: SENSOR DISCONNECTED\n");
        return my_dist_oh;
    }

    int32_t dist = raw + CALIBRATION_OFFSET_MM;

    if (dist < 0 || dist > MAX_OVERHEAD_RANGE_MM)
    {
        printf("  | Overhead dist, %d: OUT OF RANGE\n", dist);
	my_dist_oh.flag_black = true;
        return my_dist_oh;
    }

    // printf("  | Overhead dist: %4d mm\n", (int)dist);
    my_dist_oh.dist_oh = dist;
    return my_dist_oh;
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
    if (g_sensor_backend == BACKEND_MUX && g_ch_color >= 0) mux_select_channel((uint8_t)g_ch_color);
    iic_write_register(IIC0, TCS_ADDR, reg, &val, 1);
}

static int read_all(uint16_t *c, uint16_t *r, uint16_t *g, uint16_t *b)
{
    if (g_sensor_backend == BACKEND_MUX && g_ch_color >= 0) mux_select_channel((uint8_t)g_ch_color);
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

ColorReading read_color(const Cal *cal)
{
    ColorReading result = {0};
    uint16_t c, r, g, b;

    if (read_all(&c, &r, &g, &b) != 0) {
        printf("  | Color: ERROR\n");
        return result;
    }
    if (c >= 65535) {
        printf("  | Color: SATURATED\n");
        return result;
    }
    if (c < (uint16_t)cal->clear_black) {
        printf("  | Color: TOO DARK\n");
        return result;
    }

    result.lux = LUX_R*(float)r + LUX_G*(float)g + LUX_B*(float)b;
    if (result.lux < 0.0f) result.lux = 0.0f;

    apply_cal(cal, r, g, b, c, &result.r, &result.g, &result.b);
    result.valid = true;

    if (result.valid == true)
    {
        printf("  | Lux=%7.1f  CAL R=%3d G=%3d B=%3d\n",
           result.lux, result.r, result.g, result.b);
    }

    return result;
}


// bool all_sensors_init()
// {
// 	    gpio_set_direction(XSHUT_PIN_1, GPIO_DIR_OUTPUT);
//     gpio_set_direction(XSHUT_PIN_2, GPIO_DIR_OUTPUT);
// 
//     switchbox_set_pin(IO_AR_SCL, SWB_IIC0_SCL);
//     switchbox_set_pin(IO_AR_SDA, SWB_IIC0_SDA);
//     iic_init(IIC0);
// 
//     /* ── Distance sensor setup ── */
// 
//     // Hold both in reset
//     gpio_set_level(XSHUT_PIN_1, GPIO_LEVEL_LOW);
//     gpio_set_level(XSHUT_PIN_2, GPIO_LEVEL_LOW);
//     sleep_msec(10);
// 
//     // Wake sensor 1, move to 0x30
//     printf("\n--- Initializing distance sensor 1 ---\n");
//     gpio_set_level(XSHUT_PIN_1, GPIO_LEVEL_HIGH);
//     sleep_msec(10);
//     if (!change_address(DEFAULT_ADDRESS, SENSOR1_ADDRESS))
//     {
// 	return false;
//         // pynq_destroy();
//         // return EXIT_FAILURE;
//     }
// 
//     // Wake sensor 2, move to 0x31
//     printf("\n--- Initializing distance sensor 2 ---\n");
//     gpio_set_level(XSHUT_PIN_2, GPIO_LEVEL_HIGH);
//     sleep_msec(10);
//     if (!change_address(DEFAULT_ADDRESS, SENSOR2_ADDRESS))
//     {
// 	return false;
//         // pynq_destroy();
//         // return EXIT_FAILURE;
//     }
// 
//     // Init ranging on both
//     if (!init_sensor(SENSOR1_ADDRESS) || !init_sensor(SENSOR2_ADDRESS))
//     {
// 	return false;
//         // pynq_destroy();
//         // return EXIT_FAILURE;
//     }
// 
//     /* ── Color sensor setup — 0x29 is now free ── */
//     printf("\n--- Initializing color sensor ---\n");
//     uint8_t id = 0;
//     if (iic_read_register(IIC0, TCS_ADDR, REG_ID, &id, 1)) {
//         printf("ERROR: cannot read color sensor — check wiring\n");
// 	return false;
//         // pynq_destroy();
//         // return EXIT_FAILURE;
//     }
//     if (id != 0x44 && id != 0x4D)
//         printf("WARNING: unexpected color sensor ID 0x%02X\n", id);
//     else
//         printf("Color sensor ID 0x%02X — OK\n", id);
// 
//     write8(REG_ENABLE,  0x03);
//     write8(REG_ATIME,   ATIME);
//     write8(REG_WTIME,   WTIME);
//     write8(REG_CONTROL, GAIN);
//     sleep_msec(1300);
// 
//     return true;
// }

// bool all_sensors_init()
// {
//     gpio_set_direction(XSHUT_PIN_1, GPIO_DIR_OUTPUT);
//     gpio_set_direction(XSHUT_PIN_2, GPIO_DIR_OUTPUT);
// 
//     switchbox_set_pin(IO_AR_SCL, SWB_IIC0_SCL);
//     switchbox_set_pin(IO_AR_SDA, SWB_IIC0_SDA);
//     iic_init(IIC0);
// 
//     /* ── Distance sensor setup ── */
// 
//     // Hold both in reset
//     gpio_set_level(XSHUT_PIN_1, GPIO_LEVEL_LOW);
//     gpio_set_level(XSHUT_PIN_2, GPIO_LEVEL_LOW);
//     sleep_msec(10);
// 
//     // Wake sensor 1, move to 0x30
//     printf("\n--- Initializing distance sensor 1 ---\n");
//     gpio_set_level(XSHUT_PIN_1, GPIO_LEVEL_HIGH);
//     sleep_msec(10);
//     if (!change_address(DEFAULT_ADDRESS, SENSOR1_ADDRESS))
//     {
//         return false;
//     }
// 
//     // Wake sensor 2, move to 0x31
//     printf("\n--- Initializing distance sensor 2 ---\n");
//     gpio_set_level(XSHUT_PIN_2, GPIO_LEVEL_HIGH);
//     sleep_msec(10);
//     if (!change_address(DEFAULT_ADDRESS, SENSOR2_ADDRESS))
//     {
//         return false;
//     }
// 
//     // Init ranging on both
//     if (!init_sensor(SENSOR1_ADDRESS) || !init_sensor(SENSOR2_ADDRESS))
//     {
//         return false;
//     }
// 
//     // Start continuous mode on both distance sensors
//     printf("\n--- Starting continuous measurement mode ---\n");
//     uint8_t period = 0x04; // 90ms measurement period
//     uint8_t cont   = 0x02; // continuous mode flag
// 
//     iic_write_register(IIC0, SENSOR1_ADDRESS, 0x04, &period, 1);
//     if (iic_write_register(IIC0, SENSOR1_ADDRESS, 0x00, &cont, 1))
//     {
//         printf("ERROR: could not start continuous mode on sensor 1\n");
//         return false;
//     }
//     printf("Sensor 1 continuous mode started\n");
// 
//     iic_write_register(IIC0, SENSOR2_ADDRESS, 0x04, &period, 1);
//     if (iic_write_register(IIC0, SENSOR2_ADDRESS, 0x00, &cont, 1))
//     {
//         printf("ERROR: could not start continuous mode on sensor 2\n");
//         return false;
//     }
//     printf("Sensor 2 continuous mode started\n");
// 
//     /* ── Color sensor setup ── */
//     printf("\n--- Initializing color sensor ---\n");
//     uint8_t id = 0;
//     if (iic_read_register(IIC0, TCS_ADDR, REG_ID, &id, 1))
//     {
//         printf("ERROR: cannot read color sensor — check wiring\n");
//         return false;
//     }
//     if (id != 0x44 && id != 0x4D)
//         printf("WARNING: unexpected color sensor ID 0x%02X\n", id);
//     else
//         printf("Color sensor ID 0x%02X — OK\n", id);
// 
//     write8(REG_ENABLE,  0x03);
//     write8(REG_ATIME,   ATIME);
//     write8(REG_WTIME,   WTIME);
//     write8(REG_CONTROL, GAIN);
//     sleep_msec(1300);
// 
//     return true;
// }

bool all_sensors_init()
{
    gpio_set_direction(XSHUT_PIN_1, GPIO_DIR_OUTPUT);
    gpio_set_direction(XSHUT_PIN_2, GPIO_DIR_OUTPUT);

    switchbox_set_pin(IO_AR_SCL, SWB_IIC0_SCL);
    switchbox_set_pin(IO_AR_SDA, SWB_IIC0_SDA);
    iic_init(IIC0);

    /* ── Auto-detect backend: probe PCA9548A mux at 0x70 ── */
    if (mux_probe(PCA9548A_ADDR) == 0)
    {
        /* ══════════════════════════════════════════════════
         * MUX backend (old robot with PCA9548A)
         * ══════════════════════════════════════════════════ */
        g_sensor_backend = BACKEND_MUX;
        printf("\n--- MUX detected at 0x%02X — scanning channels ---\n", PCA9548A_ADDR);

        mux_scan_vl53_color(&g_ch_forward, &g_ch_overhead, &g_ch_color);

        // ── Override from ~/sensor_channels if present ──
        // File format: "forward_ch overhead_ch [color_ch]"
        // Per-board file, analogous to ~/robot_id and calblack.cfg.
        {
            const char *home = getenv("HOME");
            if (!home) home = "/home/student";
            char sc_path[128];
            snprintf(sc_path, sizeof sc_path, "%s/sensor_channels", home);
            FILE *sc = fopen(sc_path, "r");
            if (sc)
            {
                int fwd = -1, oh = -1, col = -1;
                int n = fscanf(sc, "%d %d %d", &fwd, &oh, &col);
                fclose(sc);
                if (n >= 2 && fwd >= 0 && fwd <= 7 && oh >= 0 && oh <= 7)
                {
                    printf("~/sensor_channels override: fwd=ch%d oh=ch%d", fwd, oh);
                    g_ch_forward  = fwd;
                    g_ch_overhead = oh;
                    if (n >= 3 && col >= 0 && col <= 7) { g_ch_color = col; printf(" color=ch%d", col); }
                    printf("\n");
                }
                else
                {
                    printf("~/sensor_channels: parse error (n=%d) — using scan result\n", n);
                }
            }
        }

        // ── Validate: forward VL53 is mandatory ──
        if (g_ch_forward < 0)
        {
            printf("ERROR: MUX backend: no forward VL53 found — cannot init\n");
            return false;
        }
        if (g_ch_overhead < 0)
            printf("WARNING: MUX backend: no overhead VL53 — size classification unavailable\n");

        // ── Enable TCS34725 color sensor on its mux channel ──
        if (g_ch_color >= 0)
        {
            mux_select_channel((uint8_t)g_ch_color);
            // write8 will self-select the color channel via the gate at its top
            uint8_t id = 0;
            if (iic_read_register(IIC0, TCS_ADDR, REG_ID, &id, 1))
                printf("WARNING: MUX color sensor: cannot read ID\n");
            else
                printf("MUX color sensor ID 0x%02X — %s\n", id,
                       (id == 0x44 || id == 0x4D) ? "OK" : "unexpected");
            write8(REG_ENABLE,  0x03);
            write8(REG_ATIME,   ATIME);
            write8(REG_WTIME,   WTIME);
            write8(REG_CONTROL, GAIN);
            sleep_msec(1300);
        }
        else
        {
            printf("WARNING: MUX backend: no color sensor found\n");
        }

        {
            char lbuf[128];
            snprintf(lbuf, sizeof lbuf,
                     "SENSOR BACKEND: MUX (fwd ch%d / oh ch%d / color ch%d)",
                     g_ch_forward, g_ch_overhead, g_ch_color);
            printf("%s\n", lbuf);
            // log_msg not yet available at this point (called from main before UART);
            // use printf — the operator sees this in the boot log.
        }

        return true;
    }
    else
    {
        /* ══════════════════════════════════════════════════
         * XSHUT backend (new robot — existing behaviour,
         * preserved VERBATIM below this comment)
         * ══════════════════════════════════════════════════ */
        g_sensor_backend = BACKEND_XSHUT;
        printf("\n--- No MUX at 0x%02X — XSHUT backend ---\n", PCA9548A_ADDR);

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
            return false;
        }

        // Wake sensor 2, move to 0x31
        printf("\n--- Initializing distance sensor 2 ---\n");
        gpio_set_level(XSHUT_PIN_2, GPIO_LEVEL_HIGH);
        sleep_msec(10);
        if (!change_address(DEFAULT_ADDRESS, SENSOR2_ADDRESS))
        {
            return false;
        }

        // Init ranging on both
        if (!init_sensor(SENSOR1_ADDRESS) || !init_sensor(SENSOR2_ADDRESS))
        {
            return false;
        }

        /* ── Color sensor setup ── */
        printf("\n--- Initializing color sensor ---\n");
        uint8_t id = 0;
        if (iic_read_register(IIC0, TCS_ADDR, REG_ID, &id, 1))
        {
            printf("ERROR: cannot read color sensor — check wiring\n");
            return false;
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

        printf("SENSOR BACKEND: XSHUT\n");
        return true;
    }
}
