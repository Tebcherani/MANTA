#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <malloc.h>
#include <time.h>
#include <unistd.h>
#include <string.h>

#include "diagnostic.h"
#include "vehicle.h"

#include "command_line.h"
#include "odyssey_config.h"
#include "odyssey_data_logger.h"
#include "odyssey_log.h"
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
int  load_mission(char *);
void lhs(void);

extern struct vehicle vvv;	/* from vehicle.c */
FILE *datafile;
char DataFileName[ODY_PATH_MAX];

int main(int argc, char *argv[]) {
    sim_config_t sim_config;
    sim_state_t sim_state;

    int mission_started = 0;

    if (parse_command_line(argc, argv, &options) != 0) {
        print_command_line_help(argv[0]);
        return EXIT_FAILURE;
    }

    if (options.show_help) {
        print_command_line_help(argv[0]);
        return EXIT_SUCCESS;
    }

    if (open_sim_log() != 0) exit_sim(EXIT_FAILURE, mission_started);
    if (init_sim_config(&sim_config) != 0) exit_sim(EXIT_FAILURE, mission_started);
    if (open_odyssey_log() != 0) exit_sim(EXIT_FAILURE, mission_started);
    if (init_odyssey_config() != 0) exit_sim(EXIT_FAILURE, mission_started);
    init_data_structure();
    if (init_sim_state(&sim_state) != 0) exit_sim(EXIT_FAILURE, mission_started);
    if (init_odyssey_model() != 0) exit_sim(EXIT_FAILURE, mission_started);
    if (load_mission((char *)options.mission_file) != 0) exit_sim(EXIT_FAILURE, mission_started);

    write_to_sim_log("Mission loaded: %d states, current state %d\n", vvp->nstates, vvp->cstate);

    if (load_faults(options.fault_file ? options.fault_file : vvp->fault_file_name) != 0) exit_sim(EXIT_FAILURE, mission_started);
    if (init_odyssey_sim() != 0) exit_sim(EXIT_FAILURE, mission_started);
    if (init_odyssey_data_logger(!options.nolog, DataFileName, &datafile) != 0) exit_sim(EXIT_FAILURE, mission_started);
    lhs();  // Initialize Odyssey dynamics

/* 
   This is the MAIN LOOP: initialize time, run loop, increment time at end
   for next cycle.
*/
	if (init_mission_time(&sim_state) != 0) exit_sim(EXIT_FAILURE, mission_started);

    mission_started = 1;
    while (sim_state.current_cycle < sim_config.num_cycles) {
	    if (vvp->abort_mission) {
            write_to_sim_log("%8.2f: Mission aborted because vvp->abort_mission = %d\n", sim_state.sim_time, vvp->abort_mission);
            exit_sim(EXIT_SUCCESS, mission_started);
        }

        if (update_odyssey_state(sim_config.timestep) != 0) exit_sim(EXIT_FAILURE, mission_started);

	    if (sim_state.current_cycle % sim_config.control_period == 0) {
            if (update_odyssey_cycle() != 0) {
                write_to_sim_log("Failed to update Odyssey state.\n");
                exit_sim(EXIT_FAILURE, mission_started);
            }

            capture_behavior_activity();
	        if (!options.nolog) log_data(datafile);

	        if (update_odyssey_mission()) exit_sim(EXIT_SUCCESS, mission_started);
	    }
	
        if (update_sim_state(&sim_state, sim_config.timestep) != 0) exit_sim(EXIT_FAILURE, mission_started);

        // Update clock for next control cycle
        update(M_DT, sim_config.control_period * sim_config.timestep);
    }

    exit_sim(EXIT_FAILURE, mission_started);
    return EXIT_FAILURE;
}