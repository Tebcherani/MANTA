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
#include "odyssey_log.h"
#include "odyssey_model.h"
#include "odyssey_state.h"
#include "sim_config.h"
#include "sim_exit.h"
#include "sim_log.h"
#include "sim_state.h"
#include "world_state.h"

command_line_options_t options;

/* Forward declarations for functions defined in other translation units */
int  load_mission(char *);
int  stack_busy(struct command *);
void handle_modem(void);
void lhs(void);

extern struct vehicle vvv;	/* from vehicle.c */
FILE *datafile;
char DataFileName[ODY_PATH_MAX];

int main(int argc, char *argv[]) {
    sim_config_t sim_config;
    sim_state_t sim_state;
    world_state_t world_state;
    odyssey_state_t odyssey_state;

    if (parse_command_line(argc, argv, &options) != 0) {
        print_command_line_help(argv[0]);
        exit_sim(EXIT_FAILURE);
    }

    if (options.show_help) {
        print_command_line_help(argv[0]);
        exit_sim(EXIT_SUCCESS);
    }

    if (open_sim_log() != 0) exit_sim(EXIT_FAILURE);
    if (init_sim_config(&sim_config) != 0) exit_sim(EXIT_FAILURE);
    if (init_sim_state(&sim_state) != 0) exit_sim(EXIT_FAILURE);
    if (init_world_state(&world_state) != 0) exit_sim(EXIT_FAILURE);
    if (open_odyssey_log() != 0) exit_sim(EXIT_FAILURE);
    if (init_odyssey_config() != 0) exit_sim(EXIT_FAILURE);
    init_data_structure();
    if (init_odyssey_model() != 0) exit_sim(EXIT_FAILURE);
    if (load_mission((char *)options.mission_file) != 0) exit_sim(EXIT_FAILURE);

    write_to_sim_log("Mission loaded: %d states, current state %d\n", vvp->nstates, vvp->cstate);

    if (options.parse_only) {
        write_to_sim_log("Mission parsed successfully. Exiting (--parse).\n");
        exit_sim(EXIT_SUCCESS);
    }

    if (load_faults(options.fault_file ? options.fault_file : vvp->fault_file_name) != 0) exit_sim(EXIT_FAILURE);
    if (init_odyssey_state(&odyssey_state) != 0) exit_sim(EXIT_FAILURE);
    if (init_odyssey_data_logger(!options.nolog, DataFileName, &datafile) != 0) exit_sim(EXIT_FAILURE);
    lhs();  // Initialize Odyssey dynamics

    struct tm *tp;
    UseSim = 1;

/* 
   This is the MAIN LOOP: initialize time, run loop, increment time at end
   for next cycle.
*/
	if (init_mission_time(&sim_state) != 0) exit_sim(EXIT_FAILURE);
    if (update_sim_state(&sim_state, 0.0) != 0) exit_sim(EXIT_FAILURE);

    while (sim_state.current_cycle < sim_config.num_cycles) {
	    if (vvp->abort_mission) {
            write_to_sim_log("%8.2f: Mission aborted because vvp->abort_mission = %d\n", sim_state.sim_time, vvp->abort_mission);
            exit_sim(EXIT_SUCCESS);
        }

        if (update_world_state(&world_state, sim_config.timestep) != 0) exit_sim(EXIT_FAILURE);
        if (update_odyssey_state(&odyssey_state, sim_config.timestep) != 0) exit_sim(EXIT_FAILURE);

	    if (sim_state.current_cycle % sim_config.control_period == 0) {
            simSensors();
            sensor_processing();
	        apply_faults();
            layered_control();
            dynamic_control();

            capture_behavior_activity();
	        if (!options.nolog) log_data(datafile);

	        if (update_odyssey_mission()) exit_sim(EXIT_SUCCESS);
	    }
	
        if (update_sim_state(&sim_state, sim_config.timestep) != 0) exit_sim(EXIT_FAILURE);

        // Update clock for next control cycle
        update(M_DT, sim_config.control_period * sim_config.timestep);
    }

    exit_sim(EXIT_FAILURE);
}


void exit_gracefully( int exit_status )
{
    char tmp[256], *tmp1, *tmp2;
    long size = 0;

    handle_modem();      /* send any remaining modem messages */
    
    if ( datafile )
    {
	fflush( datafile );
	size = ftell( datafile );
	fclose( datafile );
    }

    if (options.behavior_dump_file[0]) {
        dump_behavior_log_json(options.behavior_dump_file);
    }

    sprintf( msg_buf, "Mission finished -- data file: %s\n", 
	     vvp->data_file_name); 
    write_message( TERM | SLOG );

#ifdef ODY_MODEM
    load_modem( msg_buf );   /* jleonard 6/28/96 */
    write_modem(); 
#endif
				/* 01/10/94 adding an "auto_parse" option */
    tmp1 = getenv("ODY_AUTO_PARSE");
    tmp2 = getenv("ODY_PARSE_OPTION");

    if ( tmp1 )
    {
        if ( tmp2 )
	    sprintf( tmp, "parse -o %s %s", tmp2, vvp->data_file_name );
        else sprintf( tmp, "parse %s", vvp->data_file_name );

        system( tmp );
    }

#ifdef ODY_MODEM
    if ( modemlog != NULL )
	fclose( modemlog );
#endif
				/* generate entry in tabular file */
    write_tab();

    exit( 0 );
}
