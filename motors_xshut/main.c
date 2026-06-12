
#include "main_header.h"
#include "shuffle.h"

struct rock_data_t {
	enum e_rock_color color;
	int32_t size;
	float temp;
};

int main(void)
{
    pynq_init();

    stepper_init();
    stepper_enable();

    switchbox_set_pin(IO_AR0, SWB_UART0_RX);
    switchbox_set_pin(IO_AR1, SWB_UART0_TX);

    iic_init(IIC0);

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

    orientation_t ori =
    {
        .ort   = 1,
        .theta = 0.0f
    };

    gpio_set_direction(S0, GPIO_DIR_OUTPUT);
    gpio_set_direction(S1, GPIO_DIR_OUTPUT);
    gpio_set_direction(S2, GPIO_DIR_OUTPUT);
    gpio_set_direction(S3, GPIO_DIR_OUTPUT);

    gpio_set_direction(SENSOR_OUT, GPIO_DIR_INPUT);

    gpio_set_level(S0, GPIO_LEVEL_HIGH);
    gpio_set_level(S1, GPIO_LEVEL_LOW);

    uint16_t c, r, g, b;

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

                    if(uart_has_data(UART0))
                    {
                        read_uart_message(UART0, MSG);

                        fprintf(stderr, "NEW MSG = %s\n", MSG);

                        if(strcmp(MSG, "S") == 0)
                        {
                            char stp[64];

                            sprintf(stp, "STEPS,%ld", count);

                            send_message(stp);

                            fprintf(stderr, "%s\n", stp);

                            print_orientation(&ori);

                            send_orientation(&ori);

                            break;
                        }
                    }
                }
            }

            // =========================
            // FORWARD UNTIL BLOCK
            // =========================
             else if(strcmp(MSG, "FB") == 0)
             {
		 // turn_degree_other_way();

		 read_distance_forward();

                 long count = 0;

                 while(1)
                 {
                     int32_t dist = read_distance_forward();

                     if(dist >= 0 && dist <= 50)
                     {
                         // char stp[64];
                         // sprintf(stp, "STEPS,%ld", count);
                         // send_message(stp);
                         // fprintf(stderr, "%s\n", stp);
                         print_orientation(&ori);
                         send_orientation(&ori);
                         break;
                     }

                     move_forward(MOVE_UNIT, SPEED_ULTRA_SLOW);
                     wait_motion(3000);
                     count++;

                    if(uart_has_data(UART0))
                    {
                        read_uart_message(UART0, MSG);

			if(strcmp(MSG, "S") == 0)
                        {
                        char stp[64];
                        
                        sprintf(stp, "STEPS,%ld", count);
                        
                        send_message(stp);
                        
                        fprintf(stderr, "%s\n", stp);
                        
                        print_orientation(&ori);
                        
                        send_orientation(&ori);
                        
                        break;
                        }


                        // if(strcmp(MSG, "S") == 0)
                        // {
                        //     send_orientation(&ori);

                        //     char stp[64];
                        //     sprintf(stp, "STEPS,%ld", count);
			// 	send_message(stp);
                        //     break;
                        // }
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

                    if (x > 0)
                    {
                        move_forward(x * MOVE_UNIT, SPEED_FAST);

                        wait_motion(2000);
                    }

                    if (x < 0)
                    {
                        turn_180();

                        wait_motion(2000);

                        rotate_orientation(&ori, 180.0f);

                        ori.theta = 0.0f;

                        print_orientation(&ori);

                        send_orientation(&ori);

                        move_forward((-x) * MOVE_UNIT, SPEED_FAST);

                        wait_motion(2000);
                    }

                    if (y > 0)
                    {
                        turn_right_90();

                        wait_motion(2000);

                        rotate_orientation(&ori, 90.0f);

                        ori.theta = 0.0f;

                        print_orientation(&ori);

                        send_orientation(&ori);
                    }
                    else if (y < 0)
                    {
                        turn_left_90();

                        wait_motion(2000);

                        rotate_orientation(&ori, -90.0f);

                        ori.theta = 0.0f;

                        y = -y;

                        print_orientation(&ori);

                        send_orientation(&ori);
                    }

                    if (y != 0)
                    {
                        move_forward(y * MOVE_UNIT, SPEED_FAST);

                        wait_motion(2000);
                    }

                    fprintf(stderr, "MOVE COMPLETE\n");

                    print_orientation(&ori);

                    send_orientation(&ori);
                }
            }

            // =========================
            // TURN 180
            // =========================
            else if(strcmp(MSG, "U") == 0)
            {
                turn_180();

                wait_motion(100);

                rotate_orientation(&ori, 180.0f);

                ori.theta = 0.0f;

                print_orientation(&ori);

                send_orientation(&ori);
            }

            // =========================
            // RIGHT
            // =========================
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

            // =========================
            // LEFT
            // =========================
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

            // =========================
            // STOP ON BLACK
            // =========================
            else if (strcmp(MSG, "STOPBLACK") == 0)
            {
                while(1)
                {
                    move_forward(MOVE_UNIT, SPEED_MEDIUM);

                    if (detect_black())
                    {
                        fprintf(stderr, "BLACK DETECTED\n");

                        print_orientation(&ori);

                        send_orientation(&ori);

                        break;
                    }

                    wait_motion(100);
                }
            }

            // =========================
            // SWEEP
            // =========================
            else if(strcmp(MSG, "RSWEEP") == 0 || strcmp(MSG, "RS") == 0)
            {
                // sweep_right_until_wall();
		// sweep_right_for_object();
		sweep_right_for_object(&ori);
		send_orientation(&ori);
            }

            // =========================
            // FWD
            // =========================
            else if (strcmp(MSG, "FWD") == 0)
            {
                while(1)
                {
                    int32_t dist_fwd = read_distance_forward();
		    // printf("Read forward dist sensor\n"); 

                    if (dist_fwd >= 0)
                    {
                        printf("Forward: %4d mm\n", (int)dist_fwd);
                    }

                    // ColorReading color = read_color(&cal);

                    // if (color.valid)
                    // {
                    //     printf("Lux=%7.1f  CAL R=%3d G=%3d B=%3d\n",
                    //            color.lux, color.r, color.g, color.b);
                    // }

                    if(uart_has_data(UART0))
                    {
                        read_uart_message(UART0, MSG);

                        if(strcmp(MSG, "S") == 0)
                        {
                            send_orientation(&ori);
                            break;
                        }
                    }

                    wait_motion(100);
                }
            }

            // =========================
            // OH
            // =========================
            else if (strcmp(MSG, "OH") == 0)
            {
                while(1)
                {
                    int32_t dist_oh = read_distance_overhead_simple();
		    // printf("Read overhead dist sensor\n"); 

                    if (dist_oh >= 0)
                    {
                        printf("Overhead: %4d mm\n", (int)dist_oh);
                    }

                    // ColorReading color = read_color(&cal);

                    // if (color.valid)
                    // {
                    //     printf("Lux=%7.1f  CAL R=%3d G=%3d B=%3d\n",
                    //            color.lux, color.r, color.g, color.b);
                    // }

                    if(uart_has_data(UART0))
                    {
                        read_uart_message(UART0, MSG);

                        if(strcmp(MSG, "S") == 0)
                        {
                            send_orientation(&ori);
                            break;
                        }
                    }

                    wait_motion(100);
                }
            }


	    // HEADING UPDATE
            else if (strcmp(MSG, "HU") == 0)
            {
		heading_update();

                    wait_motion(100);
            }

	    // FIND AND MAP NEXT OBJECT
        else if (strcmp(MSG, "O") == 0)
        {
		struct rock_data_t rock_data;
		rock_data.color = NONE;
		rock_data.size = -1;
		rock_data.temp = -1.0f;
		
		// Should back up a little first to not displace the current object?

	        // RSWEEP 
		sweep_right_for_object(&ori);
		print_orientation(&ori);
		send_orientation(&ori);

		// Moves forward (but only one move unit)
		read_distance_forward();

                 long count = 0;
                 int32_t dist = read_distance_forward();

		char stp[64];

		 while (1)
		 {
		 dist = read_distance_forward();

                     if(dist >= 0 && dist <= 50)
                     {
                         print_orientation(&ori);
                         send_orientation(&ori);

                        sprintf(stp, "STEPS,%ld", count);
                        send_message(stp);
                        fprintf(stderr, "%s\n", stp);
                         break;
                     }

                     move_forward(MOVE_UNIT, SPEED_ULTRA_SLOW);
                     wait_motion(4000);
                     count++;

		     heading_update();


                    if(uart_has_data(UART0))
                    {
                        read_uart_message(UART0, MSG);

                        if(strcmp(MSG, "S") == 0)
                        {
                            send_orientation(&ori);
                            break;
                        }
                    }

                    wait_motion(100);
		 }


		// Front color sensor reading
                enum e_rock_color rock_color = identify_rock_color(&cal);
                if (rock_color == ERROR) { printf("Color front: total error\n"); continue; }

		    // OH dist reading
		int32_t num_readings = 10;

		    struct dist_oh_t p_dist_oh[num_readings];
			struct dist_oh_t new_reading;
			int32_t i = 0;

            // Getting the readings
			while (i < num_readings)
			{
			    new_reading = read_distance_overhead();
			    if (new_reading.dist_oh == -1 && new_reading.flag_black == false) {
				    continue;
			    }
			    p_dist_oh[i] = new_reading;
			    i++;
			}

            // Special method for checking if big black rock
			int32_t flag_not_black_count = 0;
			for (int j = num_readings-5; j < num_readings; j++) {
				if (p_dist_oh[j].flag_black == false) {
					flag_not_black_count++;
				}
			}

            // Directly storing that this rock is a big black rock if true, else
            // proceeding with normal method of taking the average overhead distance
            // reading with bins
			if (flag_not_black_count == 0 && rock_color == BLACK) {
				rock_data.color = BLACK;
				rock_data.size = 6;
			} else {

				float sum = 0.0f;
				for (int32_t j = 0; j < num_readings; j++) {
					sum += (float)p_dist_oh[j].dist_oh;
				}

				int32_t avg_dist_oh = (int32_t)(sum / (float)i);

				printf("AVG Overhead dist: %4d mm\n", (int)avg_dist_oh);

				// Classifying rock as 3x3x3 or 6x6x6 (based on overhead dist reading)
				if (avg_dist_oh >= 68 && avg_dist_oh <= 72) { // Mountain
					// "mountain"
				} else if (avg_dist_oh >= 34) { // Small rock
					rock_data.color = rock_color;
					rock_data.size = 3;
				} else { // Big rock
					rock_data.color = rock_color;
					rock_data.size = 6;
				}
			}

            if (rock_data.size == -1) {
                printf("Mountain\n");
            } else {
                print_rock_color(rock_data.color);
                printf("Rock size: %d\n", rock_data.size);
            }
            }

            else if (strcmp(MSG, "CF") == 0)
            {
            
		          while (1) {
                enum e_rock_color rock_color = identify_rock_color(&cal);
                if (rock_color == ERROR) { printf("Color front: total error\n"); continue; }
                
                print_rock_color(rock_color);
                
                if(uart_has_data(UART0))
                {
                  read_uart_message(UART0, MSG);

                  if(strcmp(MSG, "S") == 0)
                  {
                    send_orientation(&ori);
                    break;
                  }
                }
              }
            }

        // =========================
        // SHUFFLE LEFT
        // =========================
        else if(strncmp(MSG, "SHL", 3) == 0)
        {
            int cycles = 5; // default
            sscanf(MSG, "SHL,%d", &cycles);

            shuffle_sideways(&ori, cycles, true);

            print_orientation(&ori);
            send_orientation(&ori);
        }

        // =========================
        // SHUFFLE RIGHT
        // =========================
        else if(strncmp(MSG, "SHR", 3) == 0)
        {
            int cycles = 5; // default
            sscanf(MSG, "SHR,%d", &cycles);

            shuffle_sideways(&ori, cycles, false);

            print_orientation(&ori);
            send_orientation(&ori);
        }

	    // SEND FORWARD DISTANCE SENSOR READING
            else if (strcmp(MSG, "SENDDIST") == 0)
            {
		    while (1) {
		    send_dist();

                    wait_motion(100);
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
