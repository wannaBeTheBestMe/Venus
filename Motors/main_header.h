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
// #define CALIBRATION_OFFSET_MM  -27
#define FRONT_OFFSET_MM   -27
#define HEIGHT_OFFSET_MM  -12


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

#define PCA9548A_ADDR 0x70
#define SENSOR_ADDR   0x29

// MUX Channels
#define TCS_CHANNEL        0
#define VL53_FRONT_CHANNEL 1
#define VL53_HEIGHT_CHANNEL 2

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

#define HEIGHT_SENSOR_MOUNT_MM 100


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


static void send_orientation(orientation_t *ori)
{
    char msg[64];

    sprintf(msg,
            "ORT,%d,%.2f",
            ori->ort,
            ori->theta);

    send_message(msg);

    fprintf(stderr,
            "SENT ORT=%d THETA=%.2f\n",
            ori->ort,
            ori->theta);
}

// ======================================================
// ORT STRING
// ======================================================

static const char* ort_to_string(int ort)
{
    switch(ort)
    {
        case 1:
            return "NORTH";

        case 2:
            return "EAST";

        case 3:
            return "SOUTH";

        case 4:
            return "WEST";

        default:
            return "UNKNOWN";
    }
}

// ======================================================
// ROTATION UPDATE
// ======================================================

static void rotate_orientation(orientation_t *o,
                               float delta_deg)
{
    o->theta += delta_deg;

    // CLOCKWISE
    while(o->theta >= 90.0f)
    {
        o->theta -= 90.0f;

        o->ort++;

        if(o->ort > 4)
        {
            o->ort = 1;
        }
    }

    // COUNTER CLOCKWISE
    while(o->theta < 0.0f)
    {
        o->theta += 90.0f;

        o->ort--;

        if(o->ort < 1)
        {
            o->ort = 4;
        }
    }
}

// ======================================================
// ABSOLUTE HEADING
// ======================================================

static float get_heading(orientation_t *o)
{
    return ((o->ort - 1) * 90.0f) + o->theta;
}

// ======================================================
// SNAP TO CARDINAL
// ======================================================

// static void snap_orientation(orientation_t *o)
// {
//     if(o->theta >= 45.0f)
//     {
//         o->theta = 0.0f;

//         o->ort++;

//         if(o->ort > 4)
//         {
//             o->ort = 1;
//         }
//     }
//     else
//     {
//         o->theta = 0.0f;
//     }
// }

// ======================================================
// PRINT
// ======================================================

static void print_orientation(orientation_t *o)
{
    printf(
        "ORT=%s | theta=%.2f | heading=%.2f\n",
        ort_to_string(o->ort),
        o->theta,
        get_heading(o)
    );
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

static void turn_degree(void)
{
    stepper_set_speed(30000, 15000);

    stepper_steps(100, 10);

    sleep_msec(40);
}


int tcs_channel = -1;
int vl53_channel = -1;
int vl53_channel2 = -1;

bool mux_select_channel(uint8_t channel)
{
    uint8_t mux_state = 1 << channel;
    return iic_write_register(IIC0, PCA9548A_ADDR, 0x00, &mux_state, 1);
}

 int detect_sensors(void)
 {
     for (int ch = 0; ch < 8; ch++) {
         mux_select_channel(ch);
         sleep_msec(50);
         

         uint8_t id = 0;

         id = 0;

        int err = iic_read_register(IIC0, SENSOR_ADDR, TCS_REG_ID, &id, 1);

        printf("CH %d TCS err=%d ID=0x%02X\n", ch, err, id);

        if (err == 0) {
            if (id == 0x44 || id == 0x4D) {
        tcs_channel = ch;
        printf("TCS3472 found on channel %d, ID = 0x%02X\n", ch, id);
            }
        }

         if (!iic_read_register(IIC0, SENSOR_ADDR, VL53_REG_ID, &id, 1)) {
             if (id == 0xEE) {
                 if (vl53_channel == -1) {
                     vl53_channel = ch;
                     printf("VL53L0X #1 found on channel %d, ID = 0x%02X\n", ch, id);
                 }
                 else if (vl53_channel2 == -1) {
                     vl53_channel2 = ch;
                     printf("VL53L0X #2 found on channel %d, ID = 0x%02X\n", ch, id);
                 }   
             }
         }
     }

     if (tcs_channel < 0) {
         printf("ERROR: TCS3472 not found\n");
         return 1;
     }

    if (vl53_channel < 0) {
        printf("ERROR: VL53L0X #1 not found\n");
        return 1;
    }

    if (vl53_channel2 < 0) {
        printf("WARNING: VL53L0X #2 not found\n");
    }

     return 0;
}

int vl53_read_distance_from_channel(int channel);
int vl53_read_distance_calibrated(int channel, int offset)
{
    int sum = 0;
    int count = 0;

    for(int i = 0; i < 5; i++)
    {
        int d = vl53_read_distance_from_channel(channel);

        if(d > 0 && d < 1200)
        {
            sum += d;
            count++;
        }

        sleep_msec(30);
    }

    if(count == 0)
    {
        return -1;
    }

    int avg = sum / count;
    avg += offset;

    if(avg < 0)
    {
        avg = 0;
    }

    return avg;
}

void tcs_write8(uint8_t reg, uint8_t value)
{
    iic_write_register(IIC0, SENSOR_ADDR, reg, &value, 1);
}

int tcs_init(void)
{
    tcs_channel = TCS_CHANNEL;

    mux_select_channel(tcs_channel);

    tcs_write8(TCS_REG_ENABLE, 0x03);
    tcs_write8(TCS_REG_ATIME, 0x00);
    tcs_write8(TCS_REG_CONTROL, 0x00);

    sleep_msec(1300);

    printf("TCS3472 initialized on channel %d\n", tcs_channel);
    return 0;
}

int tcs_read_rgb(void)
{
    mux_select_channel(tcs_channel);

    uint8_t data[8];

    if (iic_read_register(IIC0, SENSOR_ADDR, TCS_REG_CDATAL, data, 8)) {
        printf("TCS3472 read error\n");
        return 1;
    }

    uint16_t c = data[0] | (data[1] << 8);
    uint16_t r = data[2] | (data[3] << 8);
    uint16_t g = data[4] | (data[5] << 8);
    uint16_t b = data[6] | (data[7] << 8);

    printf("Color sensor: R=%u G=%u B=%u C=%u\n", r, g, b, c);

    return 0;
}

int vl53_init(void)
{
    uint8_t start = 0x01;

    mux_select_channel(VL53_FRONT_CHANNEL);
    sleep_msec(5);
    iic_write_register(IIC0, SENSOR_ADDR, VL53_SYSRANGE, &start, 1);

    mux_select_channel(VL53_HEIGHT_CHANNEL);
    sleep_msec(5);
    iic_write_register(IIC0, SENSOR_ADDR, VL53_SYSRANGE, &start, 1);

    printf("VL53 sensors initialized on channels 1 and 2\n");

    return 0;
}

int vl53_read_distance_from_channel(int channel)
{
    mux_select_channel(channel);
    sleep_msec(5);

    uint8_t start = 0x01;

    if (iic_write_register(IIC0, SENSOR_ADDR, VL53_SYSRANGE, &start, 1)) {
        printf("VL53 trigger error on channel %d\n", channel);
        return -1;
    }

    sleep_msec(60);

    uint8_t data[2];

    if (iic_read_register(IIC0, SENSOR_ADDR, VL53_DISTANCE_REG, data, 2)) {
        printf("VL53 read error on channel %d\n", channel);
        return -1;
    }

    int distance = ((int)data[0] << 8) | data[1];

    // distance += CALIBRATION_OFFSET_MM;

    if (distance < 0) {
        distance = 0;
    }

    uint8_t clear = 0x01;
    iic_write_register(IIC0, SENSOR_ADDR, VL53_CLEAR_INT, &clear, 1);

    return distance;
}

void object_measurement_sequence(int front_dist)
{
    printf("Front distance = %d mm\n", front_dist);

    // printf("Object detected in range 30-40 mm\n");

    int height_dist =
        vl53_read_distance_calibrated(VL53_HEIGHT_CHANNEL,HEIGHT_OFFSET_MM);

    int object_height = HEIGHT_SENSOR_MOUNT_MM - height_dist;

    if(object_height < 0)
    {
        object_height = 0;
    }

    printf("Height sensor distance = %d mm\n", height_dist);
    printf("Object height = %d mm\n", object_height);

    tcs_read_rgb();

    printf("Measurement sequence complete\n");
}

int vl53_read_distance(void)
{
    if (vl53_channel < 0) {
        printf("VL53 #1 not available\n");
        return -1;
    }

    return vl53_read_distance_from_channel(vl53_channel);
}

int vl53_read_distance_2(void)
{
    if (vl53_channel2 < 0) {
        printf("VL53 #2 not available\n");
        return -1;
    }

    return vl53_read_distance_from_channel(vl53_channel2);
}



static int sweep_right_for_object(orientation_t *ori)
{
    while(1)
    {
        turn_degree();

        rotate_orientation(ori, 1.0f);

        print_orientation(ori);

        int dist = vl53_read_distance();

        printf("DIST = %d mm\n", dist);

        // if(dist > 0 && dist < 300)
        // {
        //     printf("OBJECT DETECTED\n");

        //     printf("TARGET HEADING = %.2f\n",
        //            get_heading(ori));

        //     return dist;
        //     break;
        // }

        if(dist >= 30 && dist <= 50)
        {
            printf("OBJECT DETECTED 30-40 mm\n");

            object_measurement_sequence(dist);

            printf("TARGET HEADING = %.2f\n",
            get_heading(ori));

                return dist;
        }

        if(ori->theta >= 89.0f)
        {
            printf("SWEEP COMPLETE\n");

            return -1;
        }

        sleep_msec(100);
    }
}
