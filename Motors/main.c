#include "main_header.h"
int main(void)
{
    pynq_init();

    stepper_init();
    stepper_enable();

    // MQTT
    switchbox_set_pin(IO_AR0, SWB_UART0_RX);
    switchbox_set_pin(IO_AR1, SWB_UART0_TX);

    // MUX
    switchbox_set_pin(IO_AR_SCL, SWB_IIC0_SCL);
    switchbox_set_pin(IO_AR_SDA, SWB_IIC0_SDA);

    iic_init(IIC0);

    if (detect_sensors())
    {
        pynq_destroy();
        return EXIT_FAILURE;
    }

    tcs_init();
    vl53_init();

    uart_init(UART0);
    uart_reset_fifos(UART0);

    sleep_msec(3000);

    fprintf(stderr, "READY\n");

    char MSG[MAX_MSG_LEN];

    orientation_t ori =
    {
        .ort = 1,
        .theta = 0.0f
    };

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
            read_uart_message(UART0, MSG);

            fprintf(stderr, "MSG = %s\n", MSG);

            // =====================================================
            // FORWARD
            // =====================================================

            if(strcmp(MSG, "F") == 0)
            {
                long count = 0;

                while(1)
                {
                    move_forward(MOVE_UNIT, SPEED_MEDIUM);

                    count++;

                    wait_motion(100);

                    int dist = vl53_read_distance();

                    printf("DIST = %d\n", dist);

                    if(uart_has_data(UART0))
                    {
                        read_uart_message(UART0, MSG);

                        if(strcmp(MSG, "S") == 0)
                        {
                            char stp[64];

                            sprintf(stp, "STEPS,%ld", count);

                            send_message(stp);

                            send_orientation(&ori);

                            print_orientation(&ori);

                            break;
                        }
                    }
                }
            }

            // =====================================================
            // RIGHT
            // =====================================================

            else if(strcmp(MSG, "R") == 0)
            {
                move_forward(MOVE_UNIT, SPEED_TURN);

                turn_right_90();

                wait_motion(100);

                rotate_orientation(&ori, 90.0f);

                ori.theta = 0.0f;

                print_orientation(&ori);

                send_orientation(&ori);
            }

            // =====================================================
            // LEFT
            // =====================================================

            else if(strcmp(MSG, "L") == 0)
            {
                move_forward(MOVE_UNIT, SPEED_TURN);

                turn_left_90();

                wait_motion(100);

                rotate_orientation(&ori, -90.0f);

                ori.theta = 0.0f;

                print_orientation(&ori);

                send_orientation(&ori);
            }

            // =====================================================
            // U TURN
            // =====================================================

            else if(strcmp(MSG, "U") == 0)
            {
                turn_180();

                wait_motion(100);

                rotate_orientation(&ori, 180.0f);

                ori.theta = 0.0f;

                print_orientation(&ori);

                send_orientation(&ori);
            }

            // =====================================================
            // STOP BLACK
            // =====================================================

            else if(strcmp(MSG, "STOPBLACK") == 0)
            {
                while(1)
                {
                    move_forward(MOVE_UNIT, SPEED_MEDIUM);

                    wait_motion(100);

                    if(detect_black())
                    {
                        fprintf(stderr, "BLACK DETECTED\n");

                        send_orientation(&ori);

                        break;
                    }
                }
            }

            // =====================================================
            // SWEEP
            // =====================================================

            else if(strcmp(MSG, "RSWEEP") == 0)
            {
                sweep_right_for_object(&ori);
                send_orientation(&ori);
            }

            else if(strcmp(MSG,"D") == 0)
            {
                for(int i = 0; i < 90; i++)
                {
                    turn_degree();

                    rotate_orientation(&ori, 1.0f);

                    print_orientation(&ori);

                    send_orientation(&ori);

                    wait_motion(20);
                }
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

