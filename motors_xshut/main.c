#include "main_header.h"


int main(void)
{
    pynq_init();

    stepper_init();
    stepper_enable();

    //MQTT
    switchbox_set_pin(IO_AR0, SWB_UART0_RX);
    switchbox_set_pin(IO_AR1, SWB_UART0_TX);

//    //MUX
//    switchbox_set_pin(IO_AR_SCL, SWB_IIC0_SCL);
//    switchbox_set_pin(IO_AR_SDA, SWB_IIC0_SDA);

    iic_init(IIC0);

//    if (detect_sensors())
//    {
//        pynq_destroy();
//        return EXIT_FAILURE;
//    }

//    tcs_init();
//    vl53_init();

    bool sensors_initialised = all_sensors_init();
    if (!sensors_initialised) {
	 printf("Error with sensor initialisation\n");
         pynq_destroy();
         return EXIT_FAILURE;
    }


    Cal cal = run_warmup();


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

            // =========================
            // FORWARD UNTIL STOP
            // =========================
            if(strcmp(MSG, "F") == 0)
            {
                long count = 0;

                while(1)
                {
                    move_forward(MOVE_UNIT, SPEED_MEDIUM);

                    count++;

                    wait_motion(100);

//                    vl53_read_distance();

                    if(uart_has_data(UART0))
                    {
                        read_uart_message(UART0, MSG);

                        fprintf(stderr, "NEW MSG = %s\n", MSG);

                        if(strcmp(MSG,"S") == 0)
                        {
                            char stp[64];

                            sprintf(stp, "STEPS,%ld", count);

                            send_message(stp);

                            fprintf(stderr, "%s\n", stp);

                            send_orientation(ort);

                            break;
                        }
                    }
                }
            }

            // =========================
            // MOVE X,Y
            // =========================
            else if (strncmp(MSG, "MOVE", 4) == 0)
            {
                int x, y;

                if (sscanf(MSG, "MOVE,%d,%d", &x, &y) == 2)
                {
                    fprintf(stderr, "X = %d\n", x);
                    fprintf(stderr, "Y = %d\n", y);

                    // X movement
                    if (x > 0)
                    {
                        move_forward(x * MOVE_UNIT, SPEED_FAST);

                        wait_motion(2000);
                    }

                    if (x < 0)
                    {
                        turn_180();

                        wait_motion(2000);

                        ort += 2;

                        if(ort > 4)
                            ort -= 4;

                        send_orientation(ort);

                        move_forward((-x) * MOVE_UNIT, SPEED_FAST);

                        wait_motion(2000);
                    }

                    // Y direction
                    if (y > 0)
                    {
                        turn_right_90();

                        wait_motion(2000);

                        ort++;

                        if(ort > 4)
                            ort = 1;

                        send_orientation(ort);
                    }
                    else if (y < 0)
                    {
                        turn_left_90();

                        wait_motion(2000);

                        ort--;

                        if(ort < 1)
                            ort = 4;

                        send_orientation(ort);

                        y = -y;
                    }

                    // Y movement
                    if (y != 0)
                    {
                        move_forward(y * MOVE_UNIT, SPEED_FAST);

                        wait_motion(2000);
                    }

                    fprintf(stderr, "MOVE COMPLETE\n");

                    send_orientation(ort);
                }
            }

            // =========================
            // TURN 180
            // =========================
            else if(strcmp(MSG,"U") == 0)
            {
                turn_180();

                wait_motion(100);

                if(ort > 2)
                    ort -= 2;
                else
                    ort += 2;

                send_orientation(ort);
            }

            // =========================
            // RIGHT
            // =========================
            else if(strcmp(MSG, "R") == 0)
            {
                move_forward(MOVE_UNIT, SPEED_TURN);

                turn_right_90();

                wait_motion(100);

                ort++;

                if(ort > 4)
                    ort = 1;

                send_orientation(ort);
            }

            // =========================
            // LEFT
            // =========================
            else if(strcmp(MSG,"L") == 0)
            {
                move_forward(MOVE_UNIT, SPEED_TURN);

                turn_left_90();

                wait_motion(100);

                ort--;

                if(ort < 1)
                    ort = 4;

                send_orientation(ort);
            }

            // =========================
            // STOP ON BLACK
            // =========================
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

                        send_orientation(ort);
                    }

                    wait_motion(1000);
                }
            }

            // =========================
            // SWEEP
            // =========================
            else if(strcmp(MSG,"RSWEEP") == 0)
            {
                sweep_right_until_wall();
            }

            // =========================
            // ALL FUNCTIONALITIES: 
            // =========================

            // FUNCTIONALITIES: 
	    // - STOP ON BLACK
	    // - STOP ON CUBE DETECTION
	    // - 

	    /*
	     Repeating this:
	       1. Scouts for cube
	       2. Goes to cube
	    */

            else if (strcmp(MSG, "ALL") == 0)
            {
                int running = 1;

                while (running)
                {
                    // move_forward(MOVE_UNIT, SPEED_SLOW);

                    // if (detect_black())
                    // {
                    //     running = 0;

                    //     fprintf(stderr, "BLACK DETECTED\n");

                    //     send_orientation(ort);
                    // }

		    int32_t dist_forward = read_distance_overhead();
                    if (dist_forward >= 0) {
                        // wall/object ahead within 300mm
			printf("%4d\n", dist_forward);
                    }

		    // ColorReading color = read_color(&cal);
                    // if (color.valid) {
                    //     // use color.lux, color.r, color.g, color.b

		    //    printf("  | Lux=%7.1f  CAL R=%3d G=%3d B=%3d\n", color.lux, color.r, color.g, color.b);
                    // }
                    
                    // sleep_msec(100);

                    wait_motion(1000);
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

