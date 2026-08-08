/*----------------------------------------------------------------------*
   Copyright (C) 1994-1997, Massachusetts Institute of Technology.
	 Sea Grant Autonomous Underwater Vehicles Laboratory.
			 All rights reserved.
  $Id: log_data.c,v 2.22 1998/02/03 04:25:07 bamoran Exp $
 *----------------------------------------------------------------------*/

#include <errno.h>		/* ANSI */
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include "common_defs.h"
#include "paths.h"
#include "vehicle.h"
#include "odyssey_sim.h"
#include "paths.h"
#include "odyssey_data_struct.h"
#include "odyssey_sim.h"


int CycleCount=0;

/*------------------------------------------------------------------*
 | log_data
 *------------------------------------------------------------------*/ 

int log_data( FILE *datafile )
{
    short int flag=10001, i;
    static int first = 1;
    static float last_elapsed_time=0.0;   /* for debugging data logging */
    float temp_float;
    int my_start_time;

#ifdef NO_DATA_LOGGING
    return 0;
#endif

    my_start_time = millisec();
				/* write timestamp on first call, special
				   9999 format to parse this line in the
				   data file */
    if ( first )
    {
	fprintf( datafile, "9999 %.2f (%s)\xa", vvp->mission_start_time,
		 vvp->mission_date );
		 fflush( datafile );    /* ensure 9999 marker reaches disk */
				/* initialize tabular record keeping */
	init_tab();
    }
				/* keep track of max depth */
    update_tab( read_sensor(M_DEPTH) );

				/* first write the flag (use 10001) */
    fwrite( (void *) &flag, 2, 1, datafile);

    fwrite( (void *) &last_elapsed_time, 4, 1, datafile);  

/* need to modify to write only those values that have changed since
   the last read (use >= to write every value the first time through)
   Doesn't seem to work, use first */

    for ( i = 0; i <= vvp->last_s; i++ )
    {
	if ( sensor_time(i) >= sensor_time(M_PRESENT_TIME) )
	{
	    fwrite( (void *) &i, 2, 1, datafile );

	    temp_float = (float) read_sensor(i); 
	    fwrite((void *) &temp_float, 4, 1, datafile); 
	}
    }
    if ( first )

	first = 0;

    last_elapsed_time = (float) (millisec() - my_start_time);

    return 0;
}



/*-------------------------------------------------------------------------
  The following functions automate some of the record keeping of the
  missions.  A file in the data directory VEH_NAME.tab (Xanthos,
  Amphitrite, Wodyssey, etc) will contain a tabulated summary of items such
  as run name, start time, initial position, duration, max depth.

  B. A. Moran 28-May-1997
  -----------------------------------------------------------------------*/

static float north0, east0, max_depth;

void init_tab( void )
{
    north0 = read_sensor(M_POS_N);
    east0  = read_sensor(M_POS_E);
}

void update_tab( float depth )
{
    if ( depth > max_depth )
	max_depth = depth;
}

int write_tab(const odyssey_config_t *config)
{
    FILE *ftab;
    char *run;
    int error = -1;
    char msg_buf[256];

    snprintf( msg_buf, 256, "%s/%s.tab", config->data_dir, VEHICLE_NAME );

				/* open tab-file for append */
    ftab = fopen( msg_buf, "a" );

    if ( ftab )
    {
				/* extract run name from data file name */
	run = strrchr( vvp->data_file_name, '/' );
	if ( !run++ ) run = vvp->data_file_name;

	if ( strlen(run) > 0 )
	{
				/* write entry to file and close */
	    fprintf( ftab, "%s %.2f\t% 8.2f % 7.1f % 7.1f %6.1f\n", run,
		     vvp->mission_start_time, read_sensor(M_PRESENT_TIME), north0,
		     east0, max_depth );
	    fclose( ftab );
	}
	error = 0;
    }

    return error;
}
