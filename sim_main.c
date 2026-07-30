/*----------------------------------------------------------------------*
   Copyright (C) 1994-1997, Massachusetts Institute of Technology.
	 Sea Grant Autonomous Underwater Vehicles Laboratory.
			 All rights reserved.
  $Id: sim_main.c,v 2.22 1998/02/03 04:25:07 bamoran Exp $
 *----------------------------------------------------------------------*/

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <malloc.h>
#include <time.h>
#include <unistd.h>

#include <string.h>
#include <unistd.h>

#include "diagnostic.h"
#include "filters.h"
#include "paths.h"
#include "vehicle.h"

#include "sim_config.h"
#include "sim_log.h"

/* Forward declarations for functions defined in other translation units */
int  load_mission( char * );
int  load_vehicle_def( const char * );
int  hirestime( double * );
int  stack_busy( struct command * );
void handle_modem( void );
void get_argos( long );

extern struct vehicle vvv;	/* from vehicle.c */
FILE *datafile;
char DataFileName[ODY_PATH_MAX];
double gasdev();

static char tmp2[80];
static double north0, east0;

/* CLI flags for ksim / mission verification modes */
static int parse_only_mode = 0;  /* --parse: load mission and exit */
static int nolog_mode = 0;        /* --nolog: skip data file logging */
static char behavior_dump_file[256] = "";  /* --dump-behavior: JSON output file */

/* In-memory behavior activity capture (max 100k cycles for ksim) */
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

/* Capture behavior activity for ksim verification */
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

int main( int argc, char *argv[], char *envp[] )
{
    sim_config_t sim_config;

    if (open_sim_log() != 0) return EXIT_FAILURE;

    if (init_sim(&sim_config) != 0) {
        close_sim_log();
        return EXIT_FAILURE;
    }

    char c, message[MAX_MODEM];
    unsigned char c1;
    int i, j, n, pause_mode=0, dum, k; 
    double amps = 1.35, del13 = 0.0, del24 = 0.0;
    double last_report = 0.0;
    time_t start_time;		/* used for syslog, datalog */
    struct tm *tp;
    char *tmp1, *tmp3;
    char *mission_file = NULL, *fault_file = NULL;
    char *vehicle_file = "Vehicle/Borealis.vd";
				/* e is the force/moment vector, p is the
				   position/orientation vector */
    double e[7] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    double p[7] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    ody_init_paths( argv[0] );

  /* del13 is rudder angle in radian, del24 is elevator angle in radians */
  /* I'm pretty sure that positive rudder causes a right turn and that
     positive elevator causes a dive */

    open_syslog( 0 );		/* null ptr for default */

    strcpy( msg_buf, "MIT Sea Grant Odyssey IIB vehicle simulator\n" );
    write_message( TERM | SLOG );
    write_rcs( "$Revision: 2.22 $", "$Date: 1998/02/03 04:25:07 $" );

    init_data_structure();

    UseSim = 1;

    /* Parse command-line arguments */
    if ( argc < 2 )
    {
	fprintf( stderr, "usage: %s mission [fault_file]\n", argv[0] );
        fprintf( stderr, "       %s --vehicle FILE mission          (load vehicle definition)\n", argv[0] );
        fprintf( stderr, "       %s --parse mission                 (parse only, no run)\n", argv[0] );
        fprintf( stderr, "       %s --nolog mission                 (run without logging)\n", argv[0] );
        fprintf( stderr, "       %s --dump-behavior FILE mission    (run and dump behavior log)\n", argv[0] );
        fprintf( stderr, "       %s --help|-h                     (show this message)\n", argv[0] );
	exit( 1 );
    }
    
    /* Process CLI flags */
    i = 1;
    while (i < argc && argv[i][0] == '-') {
        if (!strcmp(argv[i], "--parse")) {
            parse_only_mode = 1;
            i++;
        } else if (!strcmp(argv[i], "--nolog")) {
            nolog_mode = 1;
            i++;
        } else if (!strcmp(argv[i], "--dump-behavior")) {
            if (i+1 < argc) {
                strcpy(behavior_dump_file, argv[i+1]);
                i += 2;
            } else {
                fprintf(stderr, "Error: --dump-behavior requires filename\n");
                exit(1);
            }
        } else if (!strcmp(argv[i], "--vehicle")) {
            if (i+1 < argc) {
                vehicle_file = argv[i+1];
                i += 2;
            } else {
                fprintf(stderr, "Error: --vehicle requires filename\n");
                exit(1);
            }
        } else if (!strcmp(argv[i], "--help") || !strcmp(argv[i], "-h")) {
            fprintf( stderr, "usage: %s mission [fault_file]\n", argv[0] );
            fprintf( stderr, "       %s --vehicle FILE mission          (load vehicle definition)\n", argv[0] );
            fprintf( stderr, "       %s --parse mission                 (parse only, no run)\n", argv[0] );
            fprintf( stderr, "       %s --nolog mission                 (run without logging)\n", argv[0] );
            fprintf( stderr, "       %s --dump-behavior FILE mission    (run and dump behavior log)\n", argv[0] );
            fprintf( stderr, "       %s --help|-h                     (show this message)\n", argv[0] );
            exit(0);
        } else {
            fprintf(stderr, "Unknown option: %s\n", argv[i]);
            exit(1);
        }
    }
    
    /* Remaining positional args */
    if (i < argc) mission_file = argv[i++];
    if (i < argc) fault_file = argv[i++];
    
    if (!mission_file) {
        fprintf(stderr, "Error: mission file required\n");
        exit(1);
    }

    if ( vehicle_file && load_vehicle_def(vehicle_file) == -1 )
	exit( 1 );
    
    if ( load_mission(mission_file) == -1 )
	exit( 1 );
    sprintf( msg_buf, 
	     "vvp->nstates = %d, vvp->cstate = %d after load_mission()\n",
	     vvp->nstates, vvp->cstate );
    write_message( TERM | SLOG );

    /* If --parse mode, just exit after successful mission load */
    if (parse_only_mode) {
        sprintf( msg_buf, "Mission parse successful (--parse mode, exiting)\n");
        write_message( TERM | SLOG );
        exit(0);
    }

				/* load fault injection file if specified
				   as second argument or via data structure */
    load_faults( fault_file ? fault_file : vvp->fault_file_name );

				/* use data structure initial values to set
                                   initial position */
    update( REAL_X, sensor(M_POS_N) );
    update( REAL_Y, sensor(M_POS_E) );
    update( REAL_Z, sensor(M_DEPTH) );
    p[1] = sensor(REAL_X);
    p[2] = sensor(REAL_Y);
    p[3] = sensor(REAL_Z);
    p[6] = sensor(REAL_PSI); 
    e[1] = sensor(REAL_U);
    e[2] = sensor(REAL_V);
    e[3] = sensor(REAL_W);

    sprintf( msg_buf, "sim initial position: "
	     "%.2lf %.2lf %.2lf %.2lf %.2lf %.2lf\n",
	     p[1], p[2], p[3], p[4], p[5], p[6] );
    write_message( MonitorSim | SLOG );

    /* Initialize data logging (skip if nolog_mode) */
    if (!nolog_mode) {
        datafile = init_data_logging( DataFileName );
    } else {
        datafile = NULL;
        sprintf( msg_buf, "Logging disabled (--nolog mode)\n");
        write_message( TERM | SLOG );
    }

    init_simulation();

#ifdef SIM_MODEM_INTERFACE     /* try to change attributes of stdin */
    stdin_ptr = stdin;
    n = tcgetattr(fileno(stdin_ptr), &termios_p);

				/* set MIN to zero so that reads won't block */
    termios_p.c_cc[VEOF] = 0;

/* Use termio to set in raw (non-canonical) mode so that chars are
   immediately received by the program.  This will let the user type "p"
   for pause and "r" for resume, for example */
				/* Turn off canonical input (see termio) */
    termios_p.c_lflag &= ~0000002;
    n = tcsetattr(fileno(stdin), TCSANOW, &termios_p);
#endif

/* 
   This is the MAIN LOOP: initialize time, run loop, increment time at end
   for next cycle.
*/
				/* determine mission start time to nearest
				   second in calendar format */
    start_time = time( NULL );
    tp = gmtime( &start_time );

    strftime( vvp->mission_date, 30, "%b %d, %Y %H:%M:%S", tp );
    sprintf( msg_buf, "Calendar time: %d (%s UTC)\n", start_time,
	     vvp->mission_date );
    write_message( TERM | SLOG );

				/* grab mission start time, also has effect
				   of reinitializing millisec*/
    hirestime( &vvp->mission_start_time );

    sensor(M_PRESENT_TIME) = 0;
    sensor_time(M_PRESENT_TIME) = (long) (1000*sensor(M_PRESENT_TIME));
    for ( i = 0; i < sim_config.num_cycles; i++ )
    {
	if ( vvp->abort_mission )
	{
	    sprintf( msg_buf, "% 8.2f: ABORT mission because "
		     "vvp->abort_mission = %d\n",
		     sensor(M_PRESENT_TIME), vvp->abort_mission );
	    write_message( TERM | SLOG );
	    exit_gracefully( 0 );
	}

	amps = sensor(C_THRUSTER_CURRENT);
	del13 = sensor(C_RUDDER_ANGLE);
	del24 = sensor(C_ELEVATOR_ANGLE);

	motion( e, p, amps, del13, del24, sim_config.timestep);
	    
	if (fabs(e[1])+fabs(e[2])+fabs(e[3]) +
	    fabs(e[4])+fabs(e[5])+fabs(e[6]) > 1.e2)
	{
	    strcpy( msg_buf, "UNSTABLE SIMULATION -- ABORTED\n" );
	    write_message( TERM | SLOG );
	    exit( 1 );
	}

	if ( i % 10 == 0 )
	{
				/* to call sensor_LBL, sensor_USBL, ... */
	    simSensors();
	    sensor_processing();

				/* inject faults: after sensors so control
				   sees corrupted values; before control laws
				   so vs_* changes affect this cycle's physics */
	    apply_faults();

	    layered_control();

	    dynamic_control();

	    /* Capture behavior activity for ksim/mission verification */
	    capture_behavior_activity();

	    if (!nolog_mode)
	        log_data( datafile );

	    if( !stack_busy(&vvp->c[0]) )
	    {
		sprintf( msg_buf, "% 8.2f: state %d completed\n",
			 sensor(M_PRESENT_TIME), vvp->cstate++ );
		write_message( TERM | SLOG );

		if( vvp->cstate >= vvp->nstates )
		    exit_gracefully( 0 );
	    }
	}

/* see if user has pressed any keys, i.e. to pause the program or
   interrogate values in the data structure (simulated modem) */

#ifdef SIM_MODEM_INTERFACE
	j = read(fileno(stdin_ptr),  &c);
	if (j>0) {
				/* do switch(c) to determine appropriate
                                   action (e.g. pause, resume) */
	    switch(c) {
	    case 'd':
		if (pause_mode) fprintf(stderr, "\nstatus paused: ");
		fprintf(stderr, "m_depth=%.2lf\n", sensor(M_DEPTH));
		break;
	    case 'p':
		fprintf(stderr, "\npausing\n");
		pause_mode = 1;
		break;
	    case 'r':
		fprintf(stderr, "\nresuming\n");
		pause_mode = 0;
		break;
	    case 'x':
		fprintf(stderr, "\nCalling exit_gracefully()\n");
		(void) exit_gracefully( 0 );
		pause_mode = 0;
		break;
	    case 't':
		if (pause_mode) fprintf(stderr, "\nstatus paused: ");
		fprintf(stderr, "m_present_time=%.2lf\n", sensor(M_PRESENT_TIME));
		break;
	    default:
		if (pause_mode) fprintf(stderr, "\nstatus paused: ");
		fprintf(stderr, "\nt=%.2lf sim_modem unrecognized char is %c\n", 
			sensor(M_PRESENT_TIME), c);
		break;
	    }
	}
#endif
				/* update clock for next control cycle */
	sensor( M_PRESENT_TIME ) += sim_config.timestep;
	sensor_time(M_PRESENT_TIME) =
	    (long int) (1000 * sensor(M_PRESENT_TIME));
	update( M_DT, 10*sim_config.timestep);
    }
    exit_gracefully( 0 );

    // Close simulation log
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

    if (behavior_dump_file[0]) {
        dump_behavior_log_json(behavior_dump_file);
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

    if ( syslog!=NULL ) fclose( syslog );

#ifdef ODY_MODEM
    if ( modemlog != NULL )
	fclose( modemlog );
#endif
				/* generate entry in tabular file */
    write_tab();
#ifdef ARGOS_MSG
    get_argos( size );
#endif
    exit( 0 );
}
