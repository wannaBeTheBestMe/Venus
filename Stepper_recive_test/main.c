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

#define RED_MIN    25
#define RED_MAX    72
#define GREEN_MIN  30
#define GREEN_MAX  90
#define BLUE_MIN   25
#define BLUE_MAX   70
#define MAX_MSG_LEN 256

void read_uart_message(uart_index_t uart, char *msg)
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

    struct timespec t0, t1;

    uint32_t elapsed;

    elapsed = 0;

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

static long map(long x,
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

int main(void)
{
    pynq_init();

    // -----------------------------
    // Stepper setup
    // -----------------------------
    stepper_init();
    stepper_enable();

    // -----------------------------
    // UART setup
    // -----------------------------
    switchbox_set_pin(IO_AR0, SWB_UART0_RX);
    switchbox_set_pin(IO_AR1, SWB_UART0_TX);

    uart_init(UART0);
    uart_reset_fifos(UART0);

    sleep_msec(3000);

    fprintf(stderr, "READY\n");

    char MSG[MAX_MSG_LEN];

    gpio_set_direction(S0, GPIO_DIR_OUTPUT);
    gpio_set_direction(S1, GPIO_DIR_OUTPUT);
    gpio_set_direction(S2, GPIO_DIR_OUTPUT);
    gpio_set_direction(S3, GPIO_DIR_OUTPUT);

    gpio_set_direction(SENSOR_OUT, GPIO_DIR_INPUT);

    gpio_set_level(S0, GPIO_LEVEL_HIGH);
    gpio_set_level(S1, GPIO_LEVEL_LOW);

    while (1)
    {
        if (uart_has_data(UART0))
        {
            // -----------------------------
            // Read message
            // -----------------------------
            read_uart_message(UART0, MSG);

            fprintf(stderr, "MSG = %s\n", MSG);

            // =====================================================
            // COMMAND FORMAT:
            //
            // MOVE,X,Y
            //
            // Example:
            // MOVE,3,5
            // =====================================================

            if (strncmp(MSG, "MOVE", 4) == 0)
            {
                int x;
                int y;

                if (sscanf(MSG, "MOVE,%d,%d", &x, &y) == 2)
                {
                    fprintf(stderr, "X = %d\n", x);
                    fprintf(stderr, "Y = %d\n", y);

                    // ---------------------------------
                    // MOVE IN X DIRECTION
                    // ---------------------------------

                    stepper_set_speed(9000, 9000);

                    stepper_steps(x * 250, x * 250);

                    sleep_msec(2000);

                    // ---------------------------------
                    // 90 DEGREE TURN
                    // ---------------------------------
                    if(y != 0)
                    {
                        stepper_steps(675, -675);
                        sleep_msec(2000);
                    }

                    // ---------------------------------
                    // MOVE IN Y DIRECTION
                    // ---------------------------------

                    stepper_steps(y * 250, y * 250);

                    sleep_msec(2000);

                    fprintf(stderr, "MOVE COMPLETE\n");
                }
                else
                {
                    fprintf(stderr, "INVALID MOVE FORMAT\n");
                }
            }

            else if (strcmp(MSG, "UTURN") == 0)
            {
                //MOVE FORWARD//

                stepper_set_speed(9000, 9000);
                stepper_steps(10 * 250, 10 * 250);
                sleep_msec(2000);

                // 180 DEGREE TURN//

                stepper_set_speed(4000,4000);
                stepper_steps(1500, -1500);
                sleep_msec(1000);  

                //MOVE FORWARD AGAIN//

                stepper_set_speed(9000, 9000);
                stepper_steps(10 * 250, 10 * 250);
                sleep_msec(2000);
            }

            else if (strcmp(MSG, "STOPBLACK") == 0)
            {
                int running = 1;

                do
                {
                    stepper_set_speed(7000, 7000);

                    stepper_steps(100, 100);

                    sleep_msec(10);

                    long freq;
                    int r, g, b;

                    gpio_set_level(S2, GPIO_LEVEL_LOW);
                    gpio_set_level(S3, GPIO_LEVEL_LOW);

                    freq = (long)pulseIn_LOW(SENSOR_OUT);

                    r = clamp255(map(freq, RED_MIN, RED_MAX, 255, 0));

                    delay_ms(10);

                    gpio_set_level(S2, GPIO_LEVEL_HIGH);
                    gpio_set_level(S3, GPIO_LEVEL_HIGH);

                    freq = (long)pulseIn_LOW(SENSOR_OUT);

                    g = clamp255(map(freq, GREEN_MIN, GREEN_MAX, 255, 0));

                    delay_ms(10);

                    gpio_set_level(S2, GPIO_LEVEL_LOW);
                    gpio_set_level(S3, GPIO_LEVEL_HIGH);

                    freq = (long)pulseIn_LOW(SENSOR_OUT);

                    b = clamp255(map(freq, BLUE_MIN, BLUE_MAX, 255, 0));

                    delay_ms(10);

                    printf("R=%d G=%d B=%d\n", r, g, b);

                    if ((r < 150 && g < 150 && b < 150) || strcmp(MSG,"STOP") == 0)
                    {
                        running = 0;

                        fprintf(stderr, "BLACK DETECTED\n");
                    }

                } while (running);
            }

            else if(strcmp(MSG, "LGOAROUND") == 0)
            {
                // going around object//
                int running = 1;
                do
                {
                    stepper_set_speed(9000, 9000);

                    stepper_steps(100, 100);

                    sleep_msec(50);

                    if (uart_has_data(UART0))
                    {
                        char STOP_MSG[MAX_MSG_LEN];

                        read_uart_message(UART0, STOP_MSG);

                        if (strcmp(STOP_MSG, "TURN") == 0)
                        {
                            // turn left
                            stepper_steps(675, -675);
                            sleep_msec(2000);

                            // move forward//
                            stepper_steps(1000, 1000);
                            sleep_msec(2000);
                            
                            //turn right
                            stepper_steps(-675, 675);
                            sleep_msec(2000);

                            // move forward//
                            stepper_steps(1000, 1000);
                            sleep_msec(2000);

                             //turn right
                            stepper_steps(-675, 675);
                            sleep_msec(2000);

                            // move forward//
                            stepper_steps(1000, 1000);
                            sleep_msec(2000);

                            // turn left
                            stepper_steps(675, -675);
                            sleep_msec(2000);

                        }

                        if (strcmp(STOP_MSG, "STOP") == 0)
                        {
                            running = 0;
                        }
                    }

                } while (running);

            }

            else
            {
                fprintf(stderr, "UNKNOWN COMMAND\n");
            }

            fflush(stderr);
        }

        sleep_msec(10);
    }

    stepper_destroy();
    pynq_destroy();

    return 0;
}
