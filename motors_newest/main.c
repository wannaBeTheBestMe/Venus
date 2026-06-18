
#include "main_header.h"
#include "shuffle.h"
#include "explore.h"

struct rock_data_t {
	enum e_rock_color color;
	int32_t size;
	float temp;
};

// Bounded UART frame reader for the AUTORUN soft-stop window. Mirrors
// read_uart_message's framing (4-byte LE length + payload, MAX_MSG_LEN clamp)
// but never busy-waits forever: each per-byte wait is capped by budget_ms.
// Returns 1 with a complete message in msg, or 0 on timeout/partial frame
// (so a stray/corrupt byte during boot can't hang autonomy). main_header.h:157.
static int autorun_read_bounded(char msg[], int budget_ms)
{
	int spent = 0;
	uint32_t len = 0;
	for (int i = 0; i < 4; i++)
	{
		while (!uart_has_data(UART0))
		{
			if (spent >= budget_ms) return 0;
			sleep_msec(2); spent += 2;
		}
		len |= ((uint32_t)uart_recv(UART0) << (8 * i));
	}
	if (len >= MAX_MSG_LEN) len = MAX_MSG_LEN - 1;
	for (uint32_t i = 0; i < len; i++)
	{
		while (!uart_has_data(UART0))
		{
			if (spent >= budget_ms) return 0;
			sleep_msec(2); spent += 2;
		}
		msg[i] = uart_recv(UART0);
	}
	msg[len] = '\0';
	return 1;
}

int main(void)
{
    pynq_init();

    stepper_init();
    stepper_enable();

    switchbox_set_pin(IO_AR0, SWB_UART0_RX);
    switchbox_set_pin(IO_AR1, SWB_UART0_TX);

    iic_init(IIC0);

    adc_init();   // thermistor (rock temperature) on ADC0

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

    int cal_loaded = cal_load();   // restore persisted CALBLACK calibration if present

    fprintf(stderr, "READY\n");
    send_message("LOG,READY");   // always wireless: key boot signal, independent of LOGON/LOGOFF
    if (cal_loaded)
    {
        send_message("LOG,Loaded saved clear calibration");
        exp_mon_start();                       // global cliff-stop active (calibrated)
        send_message("LOG,cliff monitor ACTIVE");
    }
    else
    {
        // do NOT auto-start: an uncalibrated monitor could read the floor as black
        // and block all forward motion. CALBLACK will calibrate and start it.
        send_message("LOG,Run CALBLACK - cliff monitor INACTIVE until calibrated");
    }

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

    // =====================================================================
    // F1 AUTORUN — full autonomy + run-on-boot (manual §1.3, §3.2, §4.6)
    // After READY, auto-enter the EXPLORE mission with NO operator command.
    // The laptop only listens. A short pre-explore window lets an operator
    // drop to interactive command mode with a single wireless S/HOLD.
    //
    // GUARD: only auto-run if calibrated. cal_loaded (main.c:40) is 1 iff a
    // valid CALBLACK was restored; it also gates the cliff monitor. Uncalibrated
    // => monitor OFF => forward motion unsafe => skip autorun, fall through to
    // the command loop so the operator can run CALBLACK.
    // =====================================================================
    if (cal_loaded)
    {
        // Optional identification ping (F1 optional; coordinate with F8 — see notes).
        // Board id read from ~/robot_id at runtime so a single binary serves both boards.
        // Falls back to "41" if the file is missing or empty (safe default).
        {
            char robot_id[16] = "41";
            const char *home = getenv("HOME");
            if (!home) home = "/home/student";
            char id_path[128];
            snprintf(id_path, sizeof id_path, "%s/robot_id", home);
            FILE *id_f = fopen(id_path, "r");
            if (id_f)
            {
                char tmp[16];
                if (fgets(tmp, sizeof tmp, id_f))
                {
                    // strip trailing newline/whitespace
                    int tl = (int)strlen(tmp);
                    while (tl > 0 && (tmp[tl-1] == '\n' || tmp[tl-1] == '\r' || tmp[tl-1] == ' '))
                        tmp[--tl] = '\0';
                    if (tl > 0) snprintf(robot_id, sizeof robot_id, "%s", tmp);
                }
                fclose(id_f);
            }
            char hello_msg[32];
            snprintf(hello_msg, sizeof hello_msg, "HELLO,%s", robot_id);
            send_message(hello_msg);
        }
        send_message("LOG,AUTORUN: exploring in 5s - send S or HOLD to take manual control");

        // ---- soft-stop window: bounded, non-blocking poll for an early S/HOLD ----
        // ~5 s settle. NEVER use read_uart_message unguarded (it blocks on a
        // partial frame, main_header.h:157). Only read AFTER uart_has_data(); and
        // cap the whole window with a deadline so a malformed frame can't hang boot.
        const int   AUTORUN_WINDOW_MS = 5000;   // pre-explore settle + opt-out window
        const int   AUTORUN_POLL_MS   = 50;
        int         autorun_aborted   = 0;
        int         waited            = 0;

        while (waited < AUTORUN_WINDOW_MS)
        {
            if (uart_has_data(UART0))
            {
                // Bounded read: a real frame completes in ~1 ms; a partial/corrupt
                // frame returns 0 after at most this small budget instead of
                // blocking boot. Keep the budget << the window so a glitch can't
                // meaningfully stretch the ~5 s settle.
                if (autorun_read_bounded(MSG, 300)
                    && (strcmp(MSG, "S") == 0 || strcmp(MSG, "HOLD") == 0))
                {
                    autorun_aborted = 1;
                    break;
                }
                // any other early/partial frame: ignore, keep settling
            }
            sleep_msec(AUTORUN_POLL_MS);
            waited += AUTORUN_POLL_MS;
        }

        if (autorun_aborted)
        {
            g_stop = 0;   // don't carry the opt-out S into the first manual command
            send_message("LOG,AUTORUN cancelled - interactive command mode");
        }
        else
        {
            send_message("LOG,AUTORUN: entering EXPLORE (autonomous)");
            run_explore(&ori, &cal);          // self-contained: resets g_stop, starts monitor
            print_orientation(&ori);
            send_orientation(&ori);
            // run_explore returns only on operator S / trap give-up; then we
            // fall through to the command loop below for any manual follow-up.
            send_message("LOG,AUTORUN: EXPLORE returned - interactive command mode");
        }
    }
    else
    {
        // Uncalibrated: cliff monitor is OFF (main.c:54). Do NOT auto-explore.
        send_message("LOG,AUTORUN skipped - uncalibrated (run CALBLACK)");
    }
    // ===================== end F1 AUTORUN =====================

    while (1)
    {
        if (uart_has_data(UART0))
        {
            read_uart_message(UART0, MSG);

            log_msg("MSG = %s", MSG);

            // A freshly-issued command must not inherit a stale operator-stop. g_stop
            // latches (exp_check keeps returning STOP once S is seen) and was only
            // cleared inside run_explore, which silently disabled EXP1/SWEEPQ/ADV after
            // any earlier S-stop. Clear it here at dispatch (safe: a running command
            // polls UART itself; EXPLORE manages g_stop internally).
            g_stop = 0;

            // =========================
            // FORWARD UNTIL STOP
            // =========================
            if(strcmp(MSG, "F") == 0)
            {
                long count = 0;

                while(1)
                {
                    // one batch at a time (no queue-ahead); halts on cliff (g_black) or "S"
                    int stopped = move_batch_until(MOVE_UNIT, SPEED_ULTRA_SLOW, stop_black_or_S);
                    count++;

                    if (stopped)
                    {
                        if (g_black)
                        {
                            log_msg("BLACK DETECTED");
                            print_orientation(&ori);
                            send_orientation(&ori);
                            g_black_ack();
                            break;
                        }
                        char stp[64];
                        sprintf(stp, "STEPS,%ld", count);
                        send_message(stp);
                        log_msg("NEW MSG = S");
                        print_orientation(&ori);
                        send_orientation(&ori);
                        break;
                    }
                }
            }

            // =========================
            // FORWARD UNTIL BLOCK
            // =========================
             else if(strcmp(MSG, "FB") == 0)
             {
                int moved = 0;
                int r = approach_object(&ori, &moved);
                if (r == ADV_BLACK) { log_msg("FB: aborted - black/cliff"); g_black_ack(); }
                { char stp[64]; sprintf(stp, "STEPS,%d", moved); send_message(stp); }
                print_orientation(&ori);
                send_orientation(&ori);
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

                    int mv_stop = 0;   // set if a cliff (g_black) or S aborts a forward leg

                    if (x > 0)
                    {
                        mv_stop = move_batch_until(x * MOVE_UNIT, SPEED_ULTRA_SLOW, stop_black_or_S);
                        if (mv_stop && g_black) { log_msg("BLACK DETECTED"); g_black_ack(); }
                    }

                    if (x < 0)
                    {
                        turn_180();

                        wait_motion(2000);

                        rotate_orientation(&ori, 180.0f);

                        ori.theta = 0.0f;

                        print_orientation(&ori);

                        send_orientation(&ori);

                        mv_stop = move_batch_until((-x) * MOVE_UNIT, SPEED_ULTRA_SLOW, stop_black_or_S);
                        if (mv_stop && g_black) { log_msg("BLACK DETECTED"); g_black_ack(); }
                    }

                    if (!mv_stop && y > 0)
                    {
                        turn_right_90();

                        wait_motion(2000);

                        rotate_orientation(&ori, 90.0f);

                        ori.theta = 0.0f;

                        print_orientation(&ori);

                        send_orientation(&ori);
                    }
                    else if (!mv_stop && y < 0)
                    {
                        turn_left_90();

                        wait_motion(2000);

                        rotate_orientation(&ori, -90.0f);

                        ori.theta = 0.0f;

                        y = -y;

                        print_orientation(&ori);

                        send_orientation(&ori);
                    }

                    if (!mv_stop && y != 0)
                    {
                        mv_stop = move_batch_until(y * MOVE_UNIT, SPEED_ULTRA_SLOW, stop_black_or_S);
                        if (mv_stop && g_black) { log_msg("BLACK DETECTED"); g_black_ack(); }
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
                if (move_batch_until(MOVE_UNIT, SPEED_ULTRA_SLOW, stop_black_or_S))
                {
                    if (g_black) { log_msg("BLACK DETECTED"); g_black_ack(); }
                    send_orientation(&ori);
                }
                else
                {
                    turn_right_90();
                    wait_motion(100);
                    rotate_orientation(&ori, 90.0f);
                    ori.theta = 0.0f;
                    print_orientation(&ori);
                    send_orientation(&ori);
                }
            }

            // =========================
            // LEFT
            // =========================
            else if(strcmp(MSG, "L") == 0)
            {
                if (move_batch_until(MOVE_UNIT, SPEED_ULTRA_SLOW, stop_black_or_S))
                {
                    if (g_black) { log_msg("BLACK DETECTED"); g_black_ack(); }
                    send_orientation(&ori);
                }
                else
                {
                    turn_left_90();
                    wait_motion(100);
                    rotate_orientation(&ori, -90.0f);
                    ori.theta = 0.0f;
                    print_orientation(&ori);
                    send_orientation(&ori);
                }
            }

            // =========================
            // STOP ON BLACK
            // =========================
            else if (strcmp(MSG, "STOPBLACK") == 0)
            {
                // take exclusive TCS3200 access (this reads the sensor directly);
                // restore the global monitor to its prior state afterward.
                int mon_was_on = g_mon_run;
                exp_mon_stop();

                while(1)
                {
                    // move_batch_until halts the instant black appears mid-batch.
                    if (detect_black() ||
                        move_batch_until(MOVE_UNIT, SPEED_ULTRA_SLOW, detect_black))
                    {
                        stepper_halt();
                        log_msg("BLACK DETECTED");
                        print_orientation(&ori);
                        send_orientation(&ori);
                        break;
                    }
                }

                if (mon_was_on) exp_mon_start();
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

                // Approach the object (two-phase: coarse 40 mm -> fine creep to 25 mm)
                int moved = 0;
                int r = approach_object(&ori, &moved);
                if (r != ADV_DONE)
                {
                    if (r == ADV_BLACK) { log_msg("O: aborted - black/cliff"); g_black_ack(); }
                    else if (r == ADV_STOP) log_msg("O: aborted - stop");
                    send_orientation(&ori);
                    continue;
                }
                { char stp[64]; sprintf(stp, "STEPS,%d", moved); send_message(stp); }
                send_orientation(&ori);


                // Classify the object directly ahead (front color + overhead),
                // hardened against stuck/out-of-range overhead readings.
                enum e_rock_color col = NONE;
                int size = classify_object(&cal, &col, &rock_data.temp);
                if (size == -2) { log_msg("Color front: total error"); continue; }
                if (size == -3) { log_msg("Overhead sensor error - object skipped"); continue; }
                rock_data.size  = size;                 // -1 mountain, or 3/6
                rock_data.color = (size > 0) ? col : NONE;

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

            if (shuffle_sideways(&ori, cycles, true)) g_black_ack();  // clear cliff latch if the strafe aborted on black

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

            if (shuffle_sideways(&ori, cycles, false)) g_black_ack();  // clear cliff latch if the strafe aborted on black

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
            // isolate the diagnostic sweep from the global cliff monitor: an in-place
            // rotation doesn't translate toward a cliff, so don't let g_black abort it.
            int mon_was_on = g_mon_run;
            exp_mon_stop();
            g_black = 0;

            exp_obj_t objs[EXP_MAX_OBJ];
            int n = sweep_collect(&ori, objs, EXP_MAX_OBJ);
            // (sweep_collect now logs the abort reason at the source for every caller.)

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

            if (mon_was_on) exp_mon_start();
        }

        // ---- test sub-command: drift check (sweep -> approach one rock -> return -> re-sweep) ----
        else if (strcmp(MSG, "RSC") == 0)
        {
            // Isolate the IN-PLACE sweeps from the cliff monitor (like SWEEPQ): an in-place
            // rotation doesn't translate toward a cliff, so g_black must not abort it. The
            // approach + return DO translate, so the monitor is re-armed around them.
            int mon_was_on = g_mon_run;
            exp_mon_stop(); g_black = 0;

            float     origin_heading = get_heading(&ori);
            exp_obj_t objs[EXP_MAX_OBJ];
            int n = sweep_collect(&ori, objs, EXP_MAX_OBJ);
            if (n < 0)      { log_msg("RSC: sweep aborted (%s) - no drift check", (n == -2) ? "black/cliff" : "stop"); }
            else if (n < 2) { log_msg("RSC: need >=2 objects (got %d)", n); }
            else
            {
                exp_obj_t objs0[EXP_MAX_OBJ];
                for (int i = 0; i < n; i++) objs0[i] = objs[i];

                exp_rotate_rel(&ori, objs[0].rel_deg);   // aim at the first rock
                if (mon_was_on) exp_mon_start();         // re-arm: approach + return must be cliff-guarded
                int moved = 0;
                int r = approach_object(&ori, &moved);
                return_to_origin(&ori, origin_heading, moved);
                if (mon_was_on) { exp_mon_stop(); g_black = 0; }   // isolate the re-sweep too
                if (r == ADV_DONE)
                {
                    int rr = resweep_correct(&ori, objs, objs0, n, 0, 1);  // report=1
                    if (rr == ADV_BLACK || rr == ADV_STOP)
                        log_msg("RSC: re-sweep aborted (%s)", (rr == ADV_BLACK) ? "black" : "stop");
                }
                else log_msg("RSC: approach aborted (%d) - no drift report", r);
            }
            if (mon_was_on) exp_mon_start();   // always restore the monitor
            send_orientation(&ori);
        }

        // ---- test sub-command: loop reading downward black sensor ----
        else if (strcmp(MSG, "CLIFFCHK") == 0)
        {
            int mon_was_on = g_mon_run;   // this reads the TCS3200 directly
            exp_mon_stop();
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
            if (mon_was_on) exp_mon_start();
        }

        // ---- test sub-commands: start/stop the black-monitor thread ----
        else if (strcmp(MSG, "MON") == 0)    { exp_mon_start(); send_message("MON,1"); }
        else if (strcmp(MSG, "MONOFF") == 0) { exp_mon_stop();  send_message("MON,0"); }

        // ---- test sub-command: tracked heading update ----
        else if (strcmp(MSG, "HU2") == 0)
        {
            log_msg("HU2: ori before/after should MATCH (silent drift-correction)");
            print_orientation(&ori);
            heading_update();      // silent: re-aims at the object, leaves ori unchanged
            print_orientation(&ori);
            send_orientation(&ori);
        }

        // ---- test sub-command: explore the single object straight ahead ----
        else if (strcmp(MSG, "EXP1") == 0)
        {
            g_black_ack();   // clear any stale cliff latch; the slow approach + monitor
                             // re-assert a real cliff within sub-mm (sensor leads wheels)
            int moved = 0;
            int r = approach_object(&ori, &moved);
            if (r == ADV_DONE)
            {
                enum e_rock_color col = NONE;
                float temp = TEMP_INVALID;
                int size = classify_object(&cal, &col, &temp);
                if (size == -1) { send_message("MOUNTAIN,30"); }
                else if (size > 0)
                {
                    char fm[64];
                    sprintf(fm, "FOUND_ROCK,%d,%s,%.1f", size, rock_color_str(col), temp);
                    send_message(fm);
                }
                else { log_msg("classify error (%d) - object skipped", size); }
            }
            else if (r == ADV_BLACK) log_msg("EXP1: aborted - black/cliff (recalibrate / move off tape)");
            else if (r == ADV_STOP)  log_msg("EXP1: aborted - stop");
            send_orientation(&ori);
        }

        // ---- test sub-command: drive out n units then return to origin ----
        else if (strncmp(MSG, "RET", 3) == 0)
        {
            int n = 3;
            sscanf(MSG, "RET,%d", &n);
            float oh = get_heading(&ori);
            for (int i = 0; i < n; i++) { move_forward(MOVE_UNIT, SPEED_ULTRA_SLOW); wait_motion(800); }
            return_to_origin(&ori, oh, n);
            print_orientation(&ori);
            send_orientation(&ori);
        }

        // ---- test sub-command: monitored 300 mm advance ----
        else if (strcmp(MSG, "ADV") == 0)
        {
            g_black_ack();   // clear stale cliff latch; monitor re-asserts a real cliff (slow move)
            int moved = 0;
            int r = advance_monitored(&ori, EXP_ADVANCE_MM, &moved);
            char m[48];
            sprintf(m, "ADVR,%d,%d", r, moved);
            send_message(m);
            if (r == ADV_BLACK) log_msg("ADV: aborted - black/cliff (recalibrate / move off tape)");
            else if (r == ADV_STOP) log_msg("ADV: aborted - stop");
            send_orientation(&ori);
        }

        // ---- test sub-command: mountain-avoid routine ----
        else if (strcmp(MSG, "MTN") == 0)
        {
            avoid_mountain(&ori);
            send_orientation(&ori);
        }

        // ---- test sub-command: F3 trap/corner escape routine in isolation ----
        else if (strcmp(MSG, "ESCAPE") == 0)
        {
            int mon_was_on = g_mon_run;
            if (!mon_was_on) exp_mon_start();    // escape_scan/advance need the live cliff monitor
            g_haz_streak = EXP_TRAP_HAZ_LIMIT;   // pretend we're boxed in, force the escape
            int r = escape_trap(&ori);
            log_msg("ESCAPE: result=%d (0=done, 2=stop/fail)", r);
            haz_reset();
            if (!mon_was_on) exp_mon_stop();
            send_orientation(&ori);
        }

        // ---- test sub-command: the open-loop ~5 mm final nudge in isolation ----
        else if (strcmp(MSG, "NUDGE") == 0)
        {
            g_black_ack();                 // slow tiny move; clear stale cliff latch (monitor re-asserts)
            int hit = final_nudge();
            if (hit) log_msg("NUDGE: halted on black/cliff");
            else     log_msg("NUDGE: done (~%d mm)", EXP_FINAL_NUDGE_MM);
            send_orientation(&ori);
        }

        // ---- test sub-command: read the thermistor (rock temperature) ----
        else if (strcmp(MSG, "TEMP") == 0)
        {
            while (1)
            {
                float t = read_temperature();
                char m[48];
                sprintf(m, "TEMP,%.1f", t);
                send_message(m);
                log_msg("TEMP %.1f C", t);

                if (uart_has_data(UART0))
                {
                    read_uart_message(UART0, MSG);
                    if (strcmp(MSG, "S") == 0) break;
                }
                wait_motion(500);
            }
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

        // ---- dynamic TCS3200 black calibration (white + black references) ----
        else if (strcmp(MSG, "CALBLACK") == 0)
        {
            int mon_was_on = g_mon_run;
            exp_mon_stop();   // monitor thread shares S2/S3 + cal globals; must be off

            long white[4], black[4];

            log_msg("CAL: place WHITE under the sensor (arena floor)");
            for (int s = 5; s >= 1; s--) { log_msg("CAL: sampling WHITE in %d...", s); sleep_msec(1000); }
            int okw = read_channel_refs(white, 2000);

            log_msg("CAL: now show BLACK (tape) - switching in 3s");
            for (int s = 3; s >= 1; s--) { log_msg("CAL: %d...", s); sleep_msec(1000); }

            log_msg("CAL: place BLACK under the sensor (tape)");
            for (int s = 5; s >= 1; s--) { log_msg("CAL: sampling BLACK in %d...", s); sleep_msec(1000); }
            int okb = read_channel_refs(black, 2000);

            const long MIN_CONTRAST = 8;   // us; black clear-pulse must clearly exceed white
            // Black detection uses the CLEAR channel only -> validate that channel.
            int valid = okw && okb &&
                        (black[CAL_CLEAR] > white[CAL_CLEAR] + MIN_CONTRAST);

            if (!valid)
            {
                log_msg("CALBLACK FAILED (low clear contrast / bad sample) - keeping old cal");
                log_msg("CAL clear W=%ld B=%ld", white[CAL_CLEAR], black[CAL_CLEAR]);
            }
            else
            {
                cal_min[CAL_CLEAR] = white[CAL_CLEAR];
                cal_max[CAL_CLEAR] = black[CAL_CLEAR];
                log_msg("CALBLACK DONE CLEAR[%ld,%ld] (ref RGB W[%ld,%ld,%ld] B[%ld,%ld,%ld])",
                        cal_min[CAL_CLEAR], cal_max[CAL_CLEAR],
                        white[0], white[1], white[2], black[0], black[1], black[2]);
                cal_save();   // persist across reboots
            }

            // resume the global cliff monitor: after a successful calibration it is
            // safe to run; on failure, only restore it if it was already on.
            if (valid || mon_was_on) { exp_mon_start(); send_message("LOG,cliff monitor ACTIVE"); }
        }

        // ---- restore the compile-time default calibration ----
        else if (strcmp(MSG, "CALRESET") == 0)
        {
            cal_min[0] = RED_MIN;   cal_max[0] = RED_MAX;
            cal_min[1] = GREEN_MIN; cal_max[1] = GREEN_MAX;
            cal_min[2] = BLUE_MIN;  cal_max[2] = BLUE_MAX;
            cal_min[CAL_CLEAR] = CLEAR_MIN; cal_max[CAL_CLEAR] = CLEAR_MAX;
            cal_black_thresh = 150;
            cal_forget();   // delete the saved file so reboot uses defaults too
            log_msg("CAL reset to defaults (saved calibration cleared)");
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

    adc_destroy();

    pynq_destroy();

    return 0;
}




