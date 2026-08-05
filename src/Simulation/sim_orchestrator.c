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
#include "sim_log.h"
#include "sim_state.h"
#include "world_state.h"

static command_line_options_t options;

/* Forward declarations for functions defined in other translation units */
int  load_mission( char * );
int  hirestime( double * );
int  stack_busy( struct command * );
void handle_modem( void );

extern struct vehicle vvv;	/* from vehicle.c */
FILE *datafile;
char DataFileName[ODY_PATH_MAX];

/* In-memory behavior activity capture */
#define MAX_BEHAVIOR_LOG 100000
struct behavior_log_entry {
    double time;
    int state;
    int priority;
    int behavior_id;
    int heading_mode;
    int depth_mode;
    int speed_mode;
    double c0_heading;
    double c0_depth;
    double c0_speed;
} behavior_log[MAX_BEHAVIOR_LOG];
static int behavior_log_count = 0;

/* Capture behavior activity for verification */
static void capture_behavior_activity( void )
{
    int i;
    if (behavior_log_count >= MAX_BEHAVIOR_LOG) return;
    
    behavior_log[behavior_log_count].time = sensor(M_PRESENT_TIME);
    behavior_log[behavior_log_count].state = vvp->cstate;
    behavior_log[behavior_log_count].heading_mode = vvp->c[0].heading_mode;
    behavior_log[behavior_log_count].depth_mode = vvp->c[0].depth_mode;
    behavior_log[behavior_log_count].speed_mode = vvp->c[0].speed_mode;
    behavior_log[behavior_log_count].c0_heading = vvp->c[0].heading[0];
    behavior_log[behavior_log_count].c0_depth = vvp->c[0].depth[0];
    behavior_log[behavior_log_count].c0_speed = vvp->c[0].speed[0];
    
    /* Log active behavior at highest priority */
    behavior_log[behavior_log_count].behavior_id = -1;
    behavior_log[behavior_log_count].priority = -1;
    for (i = 0; i < MAX_COMMAND; i++) {
        if (vvp->sta[vvp->cstate].c[i].behavior != -1) {
            behavior_log[behavior_log_count].behavior_id = vvp->sta[vvp->cstate].c[i].behavior;
            behavior_log[behavior_log_count].priority = i;
            break;
        }
    }
    
    behavior_log_count++;
}

/* Dump behavior log as JSON for mission verification */
static void dump_behavior_log_json( const char *fname )
{
    FILE *fp;
    int i;
    
    fp = fopen(fname, "w");
    if (!fp) {
        sprintf( msg_buf, "Cannot open %s for behavior dump\n", fname );
        write_message( TERM | SLOG );
        return;
    }
    
    fprintf(fp, "[\n");
    for (i = 0; i < behavior_log_count; i++) {
        fprintf(fp, "  {\n");
        fprintf(fp, "    \"time\": %.2f,\n", behavior_log[i].time);
        fprintf(fp, "    \"state\": %d,\n", behavior_log[i].state);
        fprintf(fp, "    \"behavior_id\": %d,\n", behavior_log[i].behavior_id);
        fprintf(fp, "    \"priority\": %d,\n", behavior_log[i].priority);
        fprintf(fp, "    \"heading_mode\": %d,\n", behavior_log[i].heading_mode);
        fprintf(fp, "    \"depth_mode\": %d,\n", behavior_log[i].depth_mode);
        fprintf(fp, "    \"speed_mode\": %d,\n", behavior_log[i].speed_mode);
        fprintf(fp, "    \"c0_heading\": %.4f,\n", behavior_log[i].c0_heading);
        fprintf(fp, "    \"c0_depth\": %.4f,\n", behavior_log[i].c0_depth);
        fprintf(fp, "    \"c0_speed\": %.4f\n", behavior_log[i].c0_speed);
        fprintf(fp, "  }%s\n", (i < behavior_log_count-1) ? "," : "");
    }
    fprintf(fp, "]\n");
    fclose(fp);
    
    sprintf( msg_buf, "Behavior log dumped to %s (%d entries)\n", fname, behavior_log_count );
    write_message( TERM | SLOG );
}

int main(int argc, char *argv[]) {
    sim_config_t sim_config;
    sim_state_t sim_state;
    world_state_t world_state;
    odyssey_state_t odyssey_state;

    if (parse_command_line(argc, argv, &options) != 0) {
        print_command_line_help(argv[0]);
        return EXIT_FAILURE;
    }

    if (options.show_help) {
        print_command_line_help(argv[0]);
        return EXIT_SUCCESS;
    }

    if (open_sim_log() != 0) return EXIT_FAILURE;

    if (init_sim_config(&sim_config) != 0) {
        close_sim_log();
        return EXIT_FAILURE;
    }

    if (init_sim_state(&sim_state) != 0) {
        close_sim_log();
        return EXIT_FAILURE;
    }

    if (init_world_state(&world_state) != 0) {
        close_sim_log();
        return EXIT_FAILURE;
    }

    if (open_odyssey_log() != 0) {
        close_sim_log();
        return EXIT_FAILURE;
    }

    if (init_odyssey_config() != 0) {
        close_odyssey_log();
        close_sim_log();
        return EXIT_FAILURE;
    }

    init_data_structure();

    if (init_odyssey_model() != 0) {
        close_sim_log();
        return EXIT_FAILURE;
    }

    if (load_mission((char *)options.mission_file) != 0) {
        close_odyssey_log();
        close_sim_log();
        return EXIT_FAILURE;
    }
    write_to_sim_log("Mission loaded: %d states, current state %d\n", vvp->nstates, vvp->cstate);

    if (options.parse_only) {
        write_to_sim_log("Mission parsed successfully. Exiting (--parse).\n");
        close_odyssey_log();
        close_sim_log();
        return EXIT_SUCCESS;
    }

    struct tm *tp;
    UseSim = 1;

				/* load fault injection file if specified
				   as second argument or via data structure */
    load_faults(options.fault_file ? options.fault_file : vvp->fault_file_name );

				/* use data structure initial values to set
                                   initial position */
    update( REAL_X, sensor(M_POS_N) );
    update( REAL_Y, sensor(M_POS_E) );
    update( REAL_Z, sensor(M_DEPTH) );
    
    // Initialize Odyssey state
    if (init_odyssey_state(&odyssey_state) != 0) {
        close_sim_log();
        return EXIT_FAILURE;
    }

    /* Initialize data logging (skip if nolog_mode) */
    if (!options.nolog) {
        datafile = init_data_logging( DataFileName );
    } else {
        datafile = NULL;
        sprintf( msg_buf, "Logging disabled (--nolog mode)\n");
        write_message( TERM | SLOG );
    }

    init_simulation();

/* 
   This is the MAIN LOOP: initialize time, run loop, increment time at end
   for next cycle.
*/
	// Determine mission start time to nearest second in calendar format
    tp = gmtime(&sim_state.calendar_start_time);
    strftime(vvp->mission_date, 30, "%b %d, %Y %H:%M:%S", tp);
    hirestime(&vvp->mission_start_time);

    sensor(M_PRESENT_TIME) = 0;
    sensor_time(M_PRESENT_TIME) = (long) (1000*sensor(M_PRESENT_TIME));
    while (sim_state.current_cycle < sim_config.num_cycles) {
        // Update world state
        if (update_world_state(&world_state, sim_config.timestep) != 0) {
            close_sim_log();
            return EXIT_FAILURE;
        }

	    if (vvp->abort_mission) {
            write_to_sim_log("%8.2f: Mission aborted because vvp->abort_mission = %d\n", sim_state.sim_time, vvp->abort_mission);
            exit_gracefully(0);
        }

        // Update Odyssey state
        if (update_odyssey_state(&odyssey_state, sim_config.timestep) != 0) {
            close_sim_log();
            return EXIT_FAILURE;
        }

	    if (sim_state.current_cycle % sim_config.control_period == 0) {
            /* to call sensor_LBL, sensor_USBL, ... */
            simSensors();
            sensor_processing();

			/* inject faults: after sensors so control
				   sees corrupted values; before control laws
				   so vs_* changes affect this cycle's physics */
	        apply_faults();

            layered_control();
            dynamic_control();

            /* Capture behavior activity for mission verification */
            capture_behavior_activity();

	        if (!options.nolog) log_data( datafile );

	        if(!stack_busy(&vvp->c[0])) {
                sprintf( msg_buf, "% 8.2f: state %d completed\n",
                    sensor(M_PRESENT_TIME), vvp->cstate++ );
                write_message( TERM | SLOG );

                if( vvp->cstate >= vvp->nstates )
                    exit_gracefully( 0 );
	        }
	    }
	
        // Update simulation state
        if (update_sim_state(&sim_state, sim_config.timestep) != 0) {
            close_sim_log();
            return EXIT_FAILURE;
        }

        // Update clock for next control cycle
        sensor(M_PRESENT_TIME) = sim_state.sim_time;
        sensor_time(M_PRESENT_TIME) = (long int) (1000 * sensor(M_PRESENT_TIME));
        update(M_DT, sim_config.control_period * sim_config.timestep);
    }

    exit_gracefully(0);

    // Close logs
    if (close_odyssey_log() != 0) {
        close_sim_log();
        return EXIT_FAILURE;
    }

    if (close_sim_log() != 0) {
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
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
