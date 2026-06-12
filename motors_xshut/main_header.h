#pragma once
#include <libpynq.h>
#include <stepper.h>
#include <time.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#define S0 IO_AR4
#define S1 IO_AR5
#define S2 IO_AR6
#define S3 IO_AR7
#define SENSOR_OUT IO_AR8

#define VL53L0X_ADDR 0x29
#define SYSRANGE_START       0x00
#define RESULT_RANGE_STATUS  0x14
#define CALIBRATION_OFFSET_MM -27

#define RED_MIN    25
#define RED_MAX    72
#define GREEN_MIN  30
#define GREEN_MAX  90
#define BLUE_MIN   25
#define BLUE_MAX   70

#define MAX_MSG_LEN 256

#define TURN_90_STEPS   800
#define TURN_180_STEPS  1500
#define MOVE_UNIT       500

#define SPEED_FAST      5700
#define SPEED_MEDIUM    7000
#define SPEED_SLOW      9000
#define SPEED_TURN      4000

#define SPEED_ULTRA_SLOW 36000

#define PCA9548A_ADDR 0x70
#define SENSOR_ADDR   0x29

// TCS3472
#define TCS_REG_ID      0x92
#define TCS_REG_ENABLE  0x80
#define TCS_REG_ATIME   0x81
#define TCS_REG_CONTROL 0x8F
#define TCS_REG_CDATAL  0x94

// VL53L0X
#define VL53_REG_ID        0xC0
#define VL53_SYSRANGE      0x00
#define VL53_DISTANCE_REG  0x1E
#define VL53_CLEAR_INT     0x0B

#include "3_sensors_header.h"

// ======================================================
// ORIENTATION STRUCT
// ======================================================

typedef struct
{
    int ort;
    // 1 = NORTH
    // 2 = EAST
    // 3 = SOUTH
    // 4 = WEST

    float theta;
    // local angle inside quadrant
    // 0 -> 89.999

} orientation_t;


enum e_rock_color {
  ERROR,
  NONE,
  WHITE,
  BLACK,
  RED,
  GREEN,
  BLUE
};


static const char* ort_to_string(int ort)
{
    switch(ort)
    {
        case 1: return "NORTH";
        case 2: return "EAST";
        case 3: return "SOUTH";
        case 4: return "WEST";
        default: return "UNKNOWN";
    }
}

static void rotate_orientation(orientation_t *o, float delta_deg)
{
    o->theta += delta_deg;

    while(o->theta >= 90.0f)
    {
        o->theta -= 90.0f;
        o->ort++;
        if(o->ort > 4) o->ort = 1;
    }

    while(o->theta < 0.0f)
    {
        o->theta += 90.0f;
        o->ort--;
        if(o->ort < 1) o->ort = 4;
    }
}

static float get_heading(orientation_t *o)
{
    return ((o->ort - 1) * 90.0f) + o->theta;
}

static void print_orientation(orientation_t *o)
{
    printf("ORT=%s | theta=%.2f | heading=%.2f\n",
           ort_to_string(o->ort),
           o->theta,
           get_heading(o));
}

// ======================================================
// UART
// ======================================================

void read_uart_message(uart_index_t uart, char msg[])
{
    uint32_t len = 0;

    for (int i = 0; i < 4; i++)
    {
        while (!uart_has_data(uart));

        uint8_t byte = uart_recv(uart);

        len |= ((uint32_t)byte << (8 * i));
    }

    if (len >= MAX_MSG_LEN)
    {
        len = MAX_MSG_LEN - 1;
    }

    for (uint32_t i = 0; i < len; i++)
    {
        while (!uart_has_data(uart));

        msg[i] = uart_recv(uart);
    }

    msg[len] = '\0';
}

void send_message(char *msg)
{
    uint32_t msg_size = strlen(msg) + 1;

    uart_send(UART0, (msg_size)       & 0xFF);
    uart_send(UART0, (msg_size >> 8)  & 0xFF);
    uart_send(UART0, (msg_size >> 16) & 0xFF);
    uart_send(UART0, (msg_size >> 24) & 0xFF);

    for (uint32_t i = 0; i < msg_size; i++)
    {
        uart_send(UART0, msg[i]);
    }

    fprintf(stderr, "Sent: %s\n", msg);
}

static void send_orientation(orientation_t *ori)
{
    char msg[64];

    sprintf(msg, "ORT,%d,%.2f", ori->ort, ori->theta);

    send_message(msg);

    fprintf(stderr, "SENT ORT=%d THETA=%.2f\n", ori->ort, ori->theta);
}

// ======================================================
// TIMING
// ======================================================

static void delay_ms(int ms)
{
    struct timespec ts =
    {
        .tv_sec  = ms / 1000,
        .tv_nsec = (ms % 1000) * 1000000L
    };

    nanosleep(&ts, NULL);
}

static void wait_motion(int ms)
{
    sleep_msec(ms);
}

// ======================================================
// COLOR SENSOR (TCS frequency-based)
// ======================================================

static uint32_t pulseIn_LOW(int pin)
{
    const uint32_t TIMEOUT_US = 1000000;

    uint32_t elapsed = 0;

    while (gpio_get_level(pin) == GPIO_LEVEL_LOW)
    {
        elapsed++;
        if (elapsed > TIMEOUT_US) return 0;
    }

    elapsed = 0;

    while (gpio_get_level(pin) == GPIO_LEVEL_HIGH)
    {
        elapsed++;
        if (elapsed > TIMEOUT_US) return 0;
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

static long map_value(long x,
                      long in_min,
                      long in_max,
                      long out_min,
                      long out_max)
{
    return (x - in_min) *
           (out_max - out_min) /
           (in_max - in_min) +
           out_min;
}

static int detect_black(void)
{
    long freq;
    int r, g, b;

    gpio_set_level(S2, GPIO_LEVEL_LOW);
    gpio_set_level(S3, GPIO_LEVEL_LOW);
    freq = (long)pulseIn_LOW(SENSOR_OUT);
    r = clamp255(map_value(freq, RED_MIN, RED_MAX, 255, 0));
    delay_ms(10);

    gpio_set_level(S2, GPIO_LEVEL_HIGH);
    gpio_set_level(S3, GPIO_LEVEL_HIGH);
    freq = (long)pulseIn_LOW(SENSOR_OUT);
    g = clamp255(map_value(freq, GREEN_MIN, GREEN_MAX, 255, 0));
    delay_ms(10);

    gpio_set_level(S2, GPIO_LEVEL_LOW);
    gpio_set_level(S3, GPIO_LEVEL_HIGH);
    freq = (long)pulseIn_LOW(SENSOR_OUT);
    b = clamp255(map_value(freq, BLUE_MIN, BLUE_MAX, 255, 0));
    delay_ms(10);

    printf("R=%d G=%d B=%d\n", r, g, b);

    if (r < 150 && g < 150 && b < 150) return 1;

    return 0;
}

// ======================================================
// MOTION
// ======================================================

static void move_forward(int steps, int speed)
{
    stepper_set_speed(speed, speed);
    stepper_steps(steps, steps);
}

static void turn_left_90(void)
{
    move_forward(MOVE_UNIT, SPEED_TURN);
    stepper_set_speed(SPEED_TURN, SPEED_TURN);
    stepper_steps(TURN_90_STEPS, -TURN_90_STEPS);
}

static void turn_right_90(void)
{
    stepper_set_speed(SPEED_TURN, SPEED_TURN);
    stepper_steps(-TURN_90_STEPS, TURN_90_STEPS);
}

static void turn_180(void)
{
    stepper_set_speed(SPEED_TURN, SPEED_TURN);
    stepper_steps(TURN_180_STEPS, -TURN_180_STEPS);
}

// ======================================================
// MUX (unused in non-mux build, kept for compatibility)
// ======================================================

int tcs_channel = -1;
int vl53_channel = -1;

bool mux_select_channel(uint8_t channel)
{
    uint8_t mux_state = 1 << channel;
    return iic_write_register(IIC0, PCA9548A_ADDR, 0x00, &mux_state, 1);
}

int detect_sensors(void)
{
    for (int ch = 0; ch < 8; ch++)
    {
        mux_select_channel(ch);
        sleep_msec(50);

        uint8_t id = 0;

        if (!iic_read_register(IIC0, SENSOR_ADDR, TCS_REG_ID, &id, 1))
        {
            if (id == 0x44 || id == 0x4D)
            {
                tcs_channel = ch;
                printf("TCS3472 found on channel %d, ID = 0x%02X\n", ch, id);
            }
        }

        if (!iic_read_register(IIC0, SENSOR_ADDR, VL53_REG_ID, &id, 1))
        {
            if (id == 0xEE)
            {
                vl53_channel = ch;
                printf("VL53L0X found on channel %d, ID = 0x%02X\n", ch, id);
            }
        }
    }

    if (tcs_channel < 0) { printf("ERROR: TCS3472 not found\n"); return 1; }
    if (vl53_channel < 0) { printf("ERROR: VL53L0X not found\n"); return 1; }

    return 0;
}

// ======================================================
// TURN DEGREE (arc movement primitive)
// ======================================================

static void turn_degree(void)
{
    stepper_set_speed(30000, 15000);
    stepper_steps(100, 10);
    sleep_msec(40);
}

static void turn_degree_other_way(void)
{
    stepper_set_speed(15000, 30000);
    stepper_steps(10, 100);
    sleep_msec(40);
}


// ======================================================
// SWEEP WITH ORIENTATION TRACKING
// ======================================================

static int sweep_right_for_object(orientation_t *ori)
{
    read_distance_forward();

    while(1)
    {
        turn_degree();

        rotate_orientation(ori, 1.0f);

        print_orientation(ori);

        int32_t dist = read_distance_forward();

        printf("DIST = %d mm\n", (int)dist);

        if(dist > 0 && dist < 400)
        {
            printf("OBJECT DETECTED\n");
            printf("TARGET HEADING = %.2f\n", get_heading(ori));
	    turn_degree_other_way();
	    // turn_degree_other_way();
            return dist;
        }

        if(ori->theta >= 89.0f)
        {
            printf("SWEEP COMPLETE — no object found\n");
            return -1;
        }

        sleep_msec(300);
    }
}

static void heading_update(void)
{
	read_distance_forward();

        int32_t dist = read_distance_forward();
	for (int i = 0; i < 3; i++) {
		if (dist > 0 && dist < 400) {
			return;
		} else {
	  	dist = read_distance_forward();
		    turn_degree();
		}
	}

	// turn_degree_other_way();

        sleep_msec(300);

	for (int i = 0; i < 8; i++) {
		if (dist > 0 && dist < 400) {
			return;
		} else {
	  	dist = read_distance_forward();
		    turn_degree_other_way();
		}
	}

        sleep_msec(300);
}

static void turn_minute(void)
{
    stepper_set_speed(30000, 15000);
    stepper_steps(10, 1);
    sleep_msec(40);
}

static void sweep_minute(void)
{
    for(int i = 0; i < 4; i++)
    {
        turn_degree_other_way();
    }

    read_distance_forward();

    while(1)
    {
        turn_minute();

        int32_t dist = read_distance_forward();

        printf("DIST = %d mm\n", (int)dist);

        if(dist > 0 && dist < 400)
        {
            printf("OBJECT DETECTED\n");
            return;
        }

        sleep_msec(300);
    }
}

static void send_dist()
{
    char msg[64];
    int32_t dist = read_distance_forward();

    sprintf(msg, "SCAN,%d,", dist);

    send_message(msg);

    fprintf(stderr, "SENT DIST=%d\n", dist);
}


enum e_rock_color identify_rock_color(Cal* p_cal)
{
  enum e_rock_color rock_color;
  
   ColorReading color = read_color(p_cal);

   if (!color.valid) { printf("Color front readings not valid.\n"); }

//               printf("Lux=%7.1f  CAL R=%3d G=%3d B=%3d\n",
//                      color.lux, color.r, color.g, color.b);
          
   float total = (float)(color.r + color.g + color.b);
   if (total <= 0) { return ERROR; }
   
   float red_ratio = (float)color.r/total;
   float green_ratio = (float)color.g/total;
   float blue_ratio = (float)color.b/total;
   
   float rg_diff = (float)abs(color.r - color.g);
   float rb_diff = (float)abs(color.r - color.b);
   float gb_diff = (float)abs(color.g - color.b);
   
   bool diff_is_small = (rg_diff < 5) && (rb_diff < 5) && (gb_diff < 5);
   
   if (diff_is_small && color.r > 200) {
    rock_color = WHITE;
     return rock_color;
   } else if (diff_is_small && color.r < 50) {
    rock_color = BLACK;
     return rock_color;
   }
   
   if (red_ratio > green_ratio && red_ratio > blue_ratio) { rock_color = RED; return rock_color; }
   else if (green_ratio > red_ratio && green_ratio > blue_ratio) { rock_color = GREEN; return rock_color; }
   else if (blue_ratio > green_ratio && blue_ratio > red_ratio) { rock_color = BLUE; return rock_color; }
   
   rock_color = ERROR; // In case all color ratios are equalratios ar
   return rock_color;   
         
   wait_motion(100);
}

void print_rock_color(enum e_rock_color rock_color) {
    switch (rock_color)
    {
        case ERROR:
            printf("ERROR\n");
            break;
        case NONE:
            printf("NONE\n");
            break;
        case BLACK:
            printf("BLACK\n");
            break;
        case WHITE:
            printf("WHITE\n");
            break;
        case RED:
            printf("RED\n");
            break;
        case GREEN:
            printf("GREEN\n");
            break;
        case BLUE:
            printf("BLUE\n");
            break;
        default: ;
    }
}
