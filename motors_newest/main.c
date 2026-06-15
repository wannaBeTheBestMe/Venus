
#include "main_header.h"
#include "shuffle.h"
#include "explore.h"

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
    send_message("LOG,READY");   // always wireless: key boot signal, independent of LOGON/LOGOFF

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

            log_msg("MSG = %s", MSG);

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

                        log_msg("NEW MSG = %s", MSG);

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
                    log_msg("MOVE X=%d Y=%d", x, y);

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

                    log_msg("MOVE COMPLETE");

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
                        log_msg("BLACK DETECTED");

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

	        // RSWEEP (now a 180-deg symmetric scan; restores heading to center on no-object)
		int sweep_dist = sweep_right_for_object(&ori);
		print_orientation(&ori);
		send_orientation(&ori);

		// Nothing across the full 180-deg scan: heading is back at center, so don't
		// drive forward into empty space — report and wait for the next command.
		if (sweep_dist < 0)
		{
			send_message("NO_OBJECT");
			continue;
		}

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

				log_msg("AVG Overhead dist: %d mm", (int)avg_dist_oh);

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
                log_msg("Mountain");
                send_message("MOUNTAIN,30");
            } else {
                print_rock_color(rock_data.color);
                log_msg("Rock color=%s size=%d", rock_color_str(rock_data.color), (int)rock_data.size);
                char fm[64];
                sprintf(fm, "FOUND_ROCK,%d,%s,%.1f",
                        (int)rock_data.size, rock_color_str(rock_data.color), rock_data.temp);
                send_message(fm);
            }
            send_orientation(&ori);
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

        // =========================
        // EXPLORE & MAP (big command)
        // =========================
        else if (strcmp(MSG, "EXPLORE") == 0)
        {
            run_explore(&ori, &cal);
            print_orientation(&ori);
            send_orientation(&ori);
        }

        // ---- test sub-command: queueing sweep, report object list ----
        else if (strcmp(MSG, "SWEEPQ") == 0)
        {
            exp_obj_t objs[EXP_MAX_OBJ];
            int n = sweep_collect(&ori, objs, EXP_MAX_OBJ);
            char m[64];
            sprintf(m, "OBJN,%d", (n < 0) ? 0 : n);
            send_message(m);
            for (int k = 0; k < n; k++)
            {
                sprintf(m, "OBJ,%.1f,%d", objs[k].rel_deg, (int)objs[k].dist_mm);
                send_message(m);
                fprintf(stderr, "OBJ rel=%.1f dist=%d\n", objs[k].rel_deg, (int)objs[k].dist_mm);
            }
            send_orientation(&ori);
        }

        // ---- test sub-command: loop reading downward black sensor ----
        else if (strcmp(MSG, "CLIFFCHK") == 0)
        {
            while (1)
            {
                int b = detect_black();
                fprintf(stderr, "BLACK=%d\n", b);
                send_message(b ? "BLACK,1" : "BLACK,0");
                if (uart_has_data(UART0))
                {
                    read_uart_message(UART0, MSG);
                    if (strcmp(MSG, "S") == 0) break;
                }
                wait_motion(200);
            }
        }

        // ---- test sub-commands: start/stop the black-monitor thread ----
        else if (strcmp(MSG, "MON") == 0)    { exp_mon_start(); send_message("MON,1"); }
        else if (strcmp(MSG, "MONOFF") == 0) { exp_mon_stop();  send_message("MON,0"); }

        // ---- test sub-command: tracked heading update ----
        else if (strcmp(MSG, "HU2") == 0)
        {
            print_orientation(&ori);
            heading_update_tracked(&ori);
            print_orientation(&ori);
            send_orientation(&ori);
        }

        // ---- test sub-command: explore the single object straight ahead ----
        else if (strcmp(MSG, "EXP1") == 0)
        {
            int moved = 0;
            int r = approach_object(&ori, &moved);
            if (r == ADV_DONE)
            {
                enum e_rock_color col = NONE;
                int size = classify_object(&cal, &col);
                if (size == -1) { send_message("MOUNTAIN,30"); }
                else if (size > 0)
                {
                    char fm[64];
                    sprintf(fm, "FOUND_ROCK,%d,%s,%.1f", size, rock_color_str(col), -1.0f);
                    send_message(fm);
                }
            }
            send_orientation(&ori);
        }

        // ---- test sub-command: drive out n units then return to origin ----
        else if (strncmp(MSG, "RET", 3) == 0)
        {
            int n = 3;
            sscanf(MSG, "RET,%d", &n);
            float oh = get_heading(&ori);
            for (int i = 0; i < n; i++) { move_forward(MOVE_UNIT, SPEED_SLOW); wait_motion(800); }
            return_to_origin(&ori, oh, n);
            print_orientation(&ori);
            send_orientation(&ori);
        }

        // ---- test sub-command: monitored 300 mm advance ----
        else if (strcmp(MSG, "ADV") == 0)
        {
            int moved = 0;
            int r = advance_monitored(&ori, EXP_ADVANCE_MM, &moved);
            char m[48];
            sprintf(m, "ADVR,%d,%d", r, moved);
            send_message(m);
            send_orientation(&ori);
        }

        // ---- test sub-command: mountain-avoid routine ----
        else if (strcmp(MSG, "MTN") == 0)
        {
            avoid_mountain(&ori);
            send_orientation(&ori);
        }

        // ---- wireless logging toggle (testing tool; LOGOFF for the demo) ----
        else if (strcmp(MSG, "LOGON") == 0)
        {
            g_log_wireless = 1;
            send_message("LOG,wireless logging ON");
        }
        else if (strcmp(MSG, "LOGOFF") == 0)
        {
            send_message("LOG,wireless logging OFF");   // send confirmation before disabling
            g_log_wireless = 0;
        }

            else
            {
                log_msg("UNKNOWN COMMAND: %s", MSG);
            }

            fflush(stderr);
        }

        sleep_msec(10);
    }

    stepper_destroy();

    pynq_destroy();

    return 0;
}




