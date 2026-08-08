#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <malloc.h>
#include <time.h>
#include <unistd.h>
#include <string.h>

#include "vehicle.h"

#include "command_line.h"
#include "odyssey_data_struct.h"
#include "odyssey_data_logger.h"
#include "odyssey_mission.h"
#include "odyssey_model.h"
#include "odyssey_sim.h"
#include "sim_behavior_logger.h"
#include "sim_config.h"
#include "sim_exit.h"
#include "sim_log.h"
#include "sim_state.h"

command_line_options_t options;

/* Forward declarations for functions defined in other translation units */
int  load_mission(char *, const char *);
void lhs(void);

extern struct vehicle vvv;	/* from vehicle.c */

int main(int argc, char *argv[]) {
    odyssey_config_t odyssey_config;
    sim_config_t sim_config;
    sim_state_t sim_state;

    int mission_started = 0;
    char data_file_name[PATH_SIZE];
    FILE *datafile = NULL;

    if (parse_command_line(argc, argv, &options) != 0) {
        print_command_line_help(argv[0]);
        return EXIT_FAILURE;
    }

    if (options.show_help) {
        print_command_line_help(argv[0]);
        return EXIT_SUCCESS;
    }

    if (!open_sim_log()) exit_sim(EXIT_FAILURE, mission_started, datafile, &odyssey_config);
    if (!init_sim_config(&sim_config)) exit_sim(EXIT_FAILURE, mission_started, datafile, &odyssey_config);
    if (!init_odyssey_sim(&odyssey_config, &options, data_file_name, sizeof(data_file_name), &datafile)) exit_sim(EXIT_FAILURE, mission_started, datafile, &odyssey_config);
    if (!init_sim_state(&sim_state)) exit_sim(EXIT_FAILURE, mission_started, datafile, &odyssey_config);

    lhs();  // Initialize Odyssey dynamics
    write_to_sim_log("Odyssey dynamics initialized.\n");

    if (!init_mission_time(&sim_state)) exit_sim(EXIT_FAILURE, mission_started, datafile, &odyssey_config);

/* 
   This is the MAIN LOOP: initialize time, run loop, increment time at end
   for next cycle.
*/
    mission_started = 1;
    while (sim_state.current_cycle < sim_config.num_cycles) {
	    if (vvp->abort_mission) {
            write_to_sim_log("%8.2f: Mission aborted because vvp->abort_mission = %d\n", sim_state.sim_time, vvp->abort_mission);
            exit_sim(EXIT_SUCCESS, mission_started, datafile, &odyssey_config);
        }

        if (update_odyssey_state(sim_config.timestep) != 0) {
            write_to_sim_log("Failed to update Odyssey state.\n");
            exit_sim(EXIT_FAILURE, mission_started, datafile, &odyssey_config);
        }

	    if (sim_state.current_cycle % sim_config.control_period == 0) {
            if (update_odyssey_sim_cycle() < 0) {
                write_to_sim_log("Failed to update Odyssey cycle.\n");
                exit_sim(EXIT_FAILURE, mission_started, datafile, &odyssey_config);
            }

            capture_behavior_activity();
	        log_data(datafile);
	    }
	
        if (update_sim_state(&sim_state, sim_config.timestep) != 0) exit_sim(EXIT_FAILURE, mission_started, datafile, &odyssey_config);

        // Update clock for next control cycle
        update_sensor(M_DT, sim_config.control_period * sim_config.timestep);
    }

    exit_sim(EXIT_FAILURE, mission_started, datafile, &odyssey_config);
    return EXIT_FAILURE;
}