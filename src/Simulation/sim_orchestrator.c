#include <stdlib.h>

#include "odyssey.h"
#include "odyssey_sim.h"
#include "sim_config.h"
#include "sim_exit.h"
#include "sim_log.h"
#include "sim_state.h"
#include "world_sim.h"

int main(int argc, char *argv[]) {
    odyssey_update_status_t cycle_status;
    sim_config_t sim_config;
    sim_state_t sim_state;

    const world_state_t *world_state;

    if (!open_sim_log()) exit_sim(EXIT_FAILURE);
    if (!init_sim_config(&sim_config)) exit_sim(EXIT_FAILURE);
    
    if (!init_world_sim()) exit_sim(EXIT_FAILURE);
    world_state = get_world_state();
    if (world_state == NULL) exit_sim(EXIT_FAILURE);

    if (!init_odyssey_sim()) exit_sim(EXIT_FAILURE);
    if (!init_sim_state(&sim_state)) exit_sim(EXIT_FAILURE);
    if (!update_odyssey_sim_time(sim_state.sim_time)) exit_sim(EXIT_FAILURE);
    if (!start_odyssey_sim(sim_state.calendar_start_time)) exit_sim(EXIT_FAILURE);

/* 
   This is the MAIN LOOP: initialize time, run loop, increment time at end
   for next cycle.
*/
    while (sim_state.current_cycle < sim_config.num_cycles) {	    
        if (odyssey_abort_requested()) {
            write_to_sim_log("%8.2f: Mission aborted.\n", sim_state.sim_time);
            exit_sim(EXIT_SUCCESS);
        }

        if (!update_odyssey_sim_state(world_state, sim_config.timestep)) {
            write_to_sim_log("Failed to update Odyssey state.\n");
            exit_sim(EXIT_FAILURE);
        }

	    if (sim_state.current_cycle % sim_config.control_period == 0) {
            cycle_status = update_odyssey_sim_cycle(world_state, sim_config.control_period * sim_config.timestep);

            if (cycle_status == ODYSSEY_UPDATE_ERROR) {
                write_to_sim_log("Failed to update Odyssey cycle.\n");
                exit_sim(EXIT_FAILURE);
            }

            if (cycle_status == ODYSSEY_UPDATE_MISSION_COMPLETE) {
                write_to_sim_log("%8.2f: Mission completed.\n", sim_state.sim_time);
                exit_sim(EXIT_SUCCESS);
            }
	    }
	
        if (!update_sim_state(&sim_state, sim_config.timestep)) exit_sim(EXIT_FAILURE);
        if (!update_odyssey_sim_time(sim_state.sim_time)) exit_sim(EXIT_FAILURE);
    }

    exit_sim(EXIT_SUCCESS);
    return EXIT_SUCCESS;
}