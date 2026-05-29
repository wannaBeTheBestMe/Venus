#include "main_header.h"

int main(void)
{
    pynq_init();

    stepper_init();
    stepper_enable();

    switchbox_set_pin(IO_AR0, SWB_UART0_RX);
    switchbox_set_pin(IO_AR1, SWB_UART0_TX);

        switchbox_set_pin(IO_AR_SCL, SWB_IIC0_SCL);
    switchbox_set_pin(IO_AR_SDA, SWB_IIC0_SDA);

    iic_init(IIC0);

    uart_init(UART0);
    uart_reset_fifos(UART0);


    sleep_msec(3000);


    fprintf(stderr, "READY\n");

    char MSG[MAX_MSG_LEN];
    int ort = 1;

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

            if(strcmp(MSG, "F") == 0)
            {
                long count = 0;
                while(1)
                {
                    move_forward(MOVE_UNIT, SPEED_MEDIUM);
                    count++;
                    wait_motion(100);
                    read_distance();

                    if(uart_has_data(UART0))
                    {
                        read_uart_message(UART0, MSG);

                        fprintf(stderr, "NEW MSG = %s\n", MSG);
                        if(strcmp(MSG,"S") == 0)
                        {
                            char stp[64];
                            

                            sprintf(stp, "STEPS,%ld,%d", count,ort);

                            send_message(stp);

                            fprintf(stderr, "%s\n", stp);
                            break;
                        }
                    }
                }
            }

            else if (strncmp(MSG, "MOVE", 4) == 0)
            {
                int x, y;

                if (sscanf(MSG, "MOVE,%d,%d", &x, &y) == 2)
                {
                    fprintf(stderr, "X = %d\n", x);
                    fprintf(stderr, "Y = %d\n", y);

                    if (x > 0)
                    {
                        move_forward(x * MOVE_UNIT, SPEED_FAST);
                        wait_motion(2000);
                    }

                    if (x < 0)
                    {
                        turn_180();
                        wait_motion(2000);

                        move_forward((-x) * MOVE_UNIT, SPEED_FAST);
                        wait_motion(2000);
                    }

                    if (y > 0)
                    {
                        turn_right_90();
                        wait_motion(2000);
                    }
                    else if (y < 0)
                    {
                        turn_left_90();
                        wait_motion(2000);

                        y = -y;
                    }

                    if (y != 0)
                    {
                        move_forward(y * MOVE_UNIT, SPEED_FAST);
                        wait_motion(2000);
                    }

                    fprintf(stderr, "MOVE COMPLETE\n");
                }
            }

            else if(strcmp(MSG,"U") == 0)
            {
                turn_180();
                if(ort > 2)
                {
                    ort = ort - 2;
                }
                else 
                {
                    ort = ort + 2;
                }
            }

            
            else if(strcmp(MSG, "R") == 0)
            {
                move_forward(MOVE_UNIT,SPEED_TURN);
                turn_right_90();
                wait_motion(100);
                ort++;

                if(ort > 4)
                {
                    ort = 1;
                }
            }

            else if(strcmp(MSG,"L") == 0)
            {
                move_forward(MOVE_UNIT,SPEED_TURN);
                turn_left_90();
                wait_motion(100);
                ort--;

                if(ort < 1)
                {
                    ort = 4;
                }
            }

            else if (strcmp(MSG, "STOPBLACK") == 0)
            {
                int running = 1;

                while (running)
                {
                    move_forward(MOVE_UNIT, SPEED_MEDIUM);

                    if (detect_black())
                    {
                        running = 0;

                        fprintf(stderr, "BLACK DETECTED\n");
                    }

                    wait_motion(1000);
                }
            }

            else if(strcmp(MSG,"RSWEEP") == 0)
            {
                sweep_right_until_wall();
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
