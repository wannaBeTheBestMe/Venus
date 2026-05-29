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
#define CALIBRATION_OFFSET_MM  -27


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
    // Add +1 to include the string null-terminator (\0) in the payload
    uint32_t msg_size = 0;
    msg_size = strlen(msg)+1; 

    // Send length (4 bytes, LITTLE-endian)
    uart_send(UART0, (msg_size) & 0xFF);
    uart_send(UART0, (msg_size >> 8) & 0xFF);
    uart_send(UART0, (msg_size >> 16) & 0xFF);
    uart_send(UART0, (msg_size >> 24) & 0xFF);

    // Send payload (including the null terminator)
    for (uint32_t i = 0; i < msg_size; i++) {
        uart_send(UART0, msg[i]);
    }

    // Debug output
    fprintf(stderr, "Sent: %s\n", msg);
}

static void delay_ms(int ms)
{
    struct timespec ts =
    {
        .tv_sec = ms / 1000,
        .tv_nsec = (ms % 1000) * 1000000L
    };

    nanosleep(&ts, NULL);
}

static uint32_t pulseIn_LOW(int pin)
{
    const uint32_t TIMEOUT_US = 1000000;

    uint32_t elapsed = 0;

    while (gpio_get_level(pin) == GPIO_LEVEL_LOW)
    {
        elapsed++;

        if (elapsed > TIMEOUT_US)
        {
            return 0;
        }
    }

    elapsed = 0;

    while (gpio_get_level(pin) == GPIO_LEVEL_HIGH)
    {
        elapsed++;

        if (elapsed > TIMEOUT_US)
        {
            return 0;
        }
    }

    struct timespec t0, t1;

    clock_gettime(CLOCK_MONOTONIC, &t0);

    while (gpio_get_level(pin) == GPIO_LEVEL_LOW)
    {
    }

    clock_gettime(CLOCK_MONOTONIC, &t1);

    uint32_t us =
        (uint32_t)(
            (t1.tv_sec - t0.tv_sec) * 1000000UL +
            (t1.tv_nsec - t0.tv_nsec) / 1000UL
        );

    return us;
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

static int clamp255(long v)
{
    if (v < 0)
    {
        return 0;
    }

    if (v > 255)
    {
        return 255;
    }

    return (int)v;
}

static void wait_motion(int ms)
{
    sleep_msec(ms);
}

static void move_forward(int steps, int speed)
{   
    stepper_set_speed(speed, speed);
    stepper_steps(steps, steps);
}

//static void move_backward(int steps, int speed)
//{
//    stepper_set_speed(speed, speed);
//    stepper_steps(-steps, -steps);
//}

static void turn_left_90(void)
{
    move_forward(MOVE_UNIT,SPEED_TURN);
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

    if (r < 150 && g < 150 && b < 150)
    {
        return 1;
    }

    return 0;
}


static int read_distance(void)
{
    uint8_t start = 0x01;

    // Start measurement
    if(iic_write_register(
            IIC0,
            VL53L0X_ADDR,
            SYSRANGE_START,
            &start,
            1))
    {
        printf("Failed to start sensor\n");
        return -1;
    }

    sleep_msec(50);

    uint8_t data[2];

    // Read distance result
    if(iic_read_register(
            IIC0,
            VL53L0X_ADDR,
            0x1E,
            data,
            2))
    {
        printf("Read error\n");
        return -1;
    }

    int32_t distance =
        (int32_t)((data[0] << 8) | data[1]);

    distance += CALIBRATION_OFFSET_MM;

    if(distance < 0)
    {
        distance = 0;
    }

    printf("Distance: %d mm\n", (int)distance);

    // Clear interrupt
    uint8_t clear = 0x01;

    iic_write_register(
        IIC0,
        VL53L0X_ADDR,
        0x0B,
        &clear,
        1
    );

    return distance;
}

static void sweep_right_until_wall(void)
{
    while(1)
    {
        // curved movement
        stepper_set_speed(30000, 15000);
        stepper_steps(100, 10);

        sleep_msec(40);

        int dist = read_distance();

        // stop condition
        if(dist > 0 && dist < 300)
        {
            printf("OBJECT DETECTED\n");
            break;
        }
    }
}