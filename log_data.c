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

#include "diagnostic.h"
#include "paths.h"
#include "vehicle.h"
#include "odyssey_config.h"
#include "paths.h"
#include "odyssey_log.h"


static int loggerVersion = 2;	/* file format version number */
const char *get_fault_file_name( void );
const char *get_fault_file_path( void );
const char *get_mission_file_name( void );
const char *get_mission_file_path( void );

int CycleCount=0;

/*------------------------------------------------------------------*
 | log_data
 *------------------------------------------------------------------*/ 

int log_data( FILE *datafile )
{
    short int flag=10001, i;
    static int last_log_time = 0, first = 1;
    static float last_elapsed_time=0.0;   /* for debugging data logging */
    float temp_float;
    int num_vars_written=0;
    int dummy, beh_id, offset, j, index;
    int my_start_time, my_end_time;
    time_t ansi_time;

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
    update_tab( sensor(M_DEPTH) );

				/* first write the flag (use 10001) */
    dummy = fwrite( (void *) &flag, 2, 1, datafile);
    if ( dummy < 1 ) 
	fprintf( syslog, "cycle %d fwrite of flag returns %d in log_data\n", 
		 CycleCount, dummy );

    dummy = fwrite( (void *) &last_elapsed_time, 4, 1, datafile);  
    if (dummy < 1)
	fprintf( syslog, "cycle %d fwrite of rtime returns %d in log_data\n", 
		 CycleCount, dummy );

/* need to modify to write only those values that have changed since
   the last read (use >= to write every value the first time through)
   Doesn't seem to work, use first */

    for ( i = 0; i <= vvp->last_s; i++ )
    {
	if ( sensor_time(i) >= sensor_time(M_PRESENT_TIME) )
	{
	    num_vars_written++;
	    dummy = fwrite( (void *) &i, 2, 1, datafile );

	    if (dummy<1)
		fprintf(syslog, "cycle %d fwrite ind %d ret "
			"%d in log_data\n", CycleCount, i, dummy);

	    temp_float = (float) sensor(i); 
	    fwrite((void *) &temp_float, 4, 1, datafile); 

	    if (dummy<1) 
		fprintf(syslog, "cycle %d fwrite val %d ret "
			"%d in log_data\n", CycleCount, i, dummy);
	}
    }
    if ( first )

	first = 0;

    last_elapsed_time = (float) (millisec() - my_start_time);
    last_log_time = sensor_time(M_PRESENT_TIME);

    return 0;
}


/*------------------------------------------------------------------*
 | write_fault_header
 |
 | Copies the active *.fd file verbatim into the ODB ASCII header so
 | the fault scenario is self-documenting in the data file.  The block
 | is wrapped in FAULT_FILE_BEGIN/END sentinels.  Every embedded line
 | is prefixed "# fd>" so all existing parsers skip it cleanly.
 | If no fault file was loaded the delimiters are still written with
 | a one-line note — the header remains valid either way.
 *------------------------------------------------------------------*/

static void write_fault_header( FILE *datafile )
{
    FILE       *ffd;
    char        fd_path[256];
    char        line[256];
    int         n_lines = 0;
    const char *fname   = get_fault_file_name();

    fprintf( datafile,
        "# ===== FAULT_FILE_BEGIN ================================\xa" );

    if ( fname == NULL || fname[0] == '\0' ) {
        fprintf( datafile, "# (no fault file loaded)\xa" );
    } else {
        fprintf( datafile, "# fault file: %s\xa", fname );
        strncpy( fd_path, get_fault_file_path(), sizeof(fd_path) - 1 );
        fd_path[sizeof(fd_path) - 1] = '\0';
        if ( fd_path[0] == '\0' )
            ody_join_path( fd_path, sizeof(fd_path), SFAULT_DIR, fname );

        if ( (ffd = fopen(fd_path, "r")) == NULL ) {
            fprintf( datafile,
                "# WARNING: could not open %s for embedding\xa", fd_path );
        } else {
            while ( fgets(line, sizeof(line), ffd) ) {
                line[ strcspn(line, "\r\n") ] = '\0';
                fprintf( datafile, "# fd> %s\xa", line );
                n_lines++;
            }
            fclose( ffd );
            fprintf( datafile, "# (%d lines)\xa", n_lines );
        }
    }

    fprintf( datafile,
        "# ===== FAULT_FILE_END ==================================\xa" );
}


/*------------------------------------------------------------------*
 | write_mission_header
 |
 | Copies the active mission file verbatim into the ODB ASCII header so
 | a run can be reproduced from the data file.  The block is wrapped in
 | MISSION_FILE_BEGIN/END sentinels.  Every embedded line is prefixed
 | "# mi>" so existing parsers continue to skip it as a comment.
 *------------------------------------------------------------------*/

static void write_mission_header( FILE *datafile )
{
    FILE       *fmi;
    char        mi_path[ODY_PATH_MAX];
    char        line[MAX_LINE];
    int         n_lines = 0;
    const char *fname   = get_mission_file_name();

    fprintf( datafile,
        "# ===== MISSION_FILE_BEGIN ==============================\xa" );

    if ( fname == NULL || fname[0] == '\0' ) {
        fprintf( datafile, "# (no mission file loaded)\xa" );
    } else {
        fprintf( datafile, "# mission file: %s\xa", fname );
        strncpy( mi_path, get_mission_file_path(), sizeof(mi_path) - 1 );
        mi_path[sizeof(mi_path) - 1] = '\0';
        if ( mi_path[0] == '\0' )
            ody_join_path( mi_path, sizeof(mi_path), MISSION_DIR, fname );

        if ( (fmi = fopen(mi_path, "r")) == NULL ) {
            fprintf( datafile,
                "# WARNING: could not open %s for embedding\xa", mi_path );
        } else {
            while ( fgets(line, sizeof(line), fmi) ) {
                line[ strcspn(line, "\r\n") ] = '\0';
                fprintf( datafile, "# mi> %s\xa", line );
                n_lines++;
            }
            fclose( fmi );
            fprintf( datafile, "# (%d lines)\xa", n_lines );
        }
    }

    fprintf( datafile,
        "# ===== MISSION_FILE_END ================================\xa" );
}


/*------------------------------------------------------------------*
 | init_data_logging()
 *------------------------------------------------------------------*/ 

FILE *init_data_logging( char *DataFileName )
{
    FILE *datafile;
    int i=0, j, beh_id, offset;
    char time_string[30], fname_string[32];
    int dummy;

    time_t t;  
    struct tm *tp; 
  
    sprintf( msg_buf, "Data Logger (binary file v%d)\n", loggerVersion );
    write_message( TERM | SLOG );

/* Generate unique file name for binary data by concatenation of
   appropriate prefix char, year (00-99), day (001-366), run number
   (01-99), followed by the extension .obd (Odyssey Binary Data) */

    t = time( NULL );
    tp = gmtime( &t );
    strftime( fname_string, sizeof(fname_string), "%y%j", tp );  

    RunNumber = 0;
    do {
	++RunNumber;
	
	snprintf( DataFileName, ODY_PATH_MAX, "%s/%c%s%02d.obd",
		  DATA_DIR, PREFIX_CHAR, fname_string, RunNumber );

#define MODE F_OK		/* UNIX test for existence using access() */

    } while ( access(DataFileName, MODE) != -1 );

    if ( (datafile = fopen(DataFileName, "wb")) == NULL )
    {
	sprintf( msg_buf, "%s: %s\n", DataFileName, strerror(errno) );
	write_message( TERM | SLOG );
	exit_gracefully( 1 );
    }
				/* record name */
    strcpy( vvp->data_file_name, DataFileName );
    sprintf( msg_buf, "\tdatafile: %s\n", DataFileName );
    write_message( TERM | SLOG );

				/* version number */
    fprintf( datafile, "MIT_SG_AUV_LAB_v%d %s\xa", loggerVersion,
	     VEHICLE_NAME );

				/* record creation times */
    strftime( time_string, sizeof(time_string), "%Y/%m/%d %H:%M:%S GMT", tp );
    fprintf( datafile, "%s     # OS-9 file creation time\xa", time_string );
    fprintf( datafile, "%s     # Structure creation time\xa",
	     vvp->creation_date );

				/* embed fault file in header for
				   self-documentation of fault scenario */
    write_fault_header( datafile );
    write_mission_header( datafile );

				/* write sensor index, name, init values */
    for ( i = 0; i <= vvp->last_s; i++ )
    {
	fprintf( datafile, "%d %-29s", i, sensor_name(i) );
				/* sensor_time initialized to -1 in
				   vehicle.c by edit_struct if initial
				   value unspecified */
	if ( sensor_time(i) >= 0L )
	    fprintf( datafile, " %.8g\xa", sensor(i) );
	else
	    fputc( '\xa', datafile );
    }

				/* new mod jleonard 11/2/94: write behavior
                                   priorities and arguments */
    for ( i = 0; i < MAX_COMMAND; i++ )
    {
	beh_id = vvp->c[i].behavior;
      
	if (beh_id!=-1) {
	    offset = vvp->last_s + 1 + (MAX_B_ARG * (i-1));
	    fprintf(datafile, "%d %-30s \t%-d\n", offset, vvp->c[i].b.name, i);
	    sprintf( msg_buf, "%d %-30s \t%-d\n", offset,
		     vvp->c[i].b.name, i );
	    write_message( MonitorDataLogging );

				/* now write the argument values for this
                                   behavior */
	    for (j=0;j<MAX_B_ARG;j++) { 
		if ((vvp->c[i].b.arg[j].name!=NULL)
		    &&(strlen(vvp->c[i].b.arg[j].name))) {
		    fprintf(datafile, "%d %s_%-30s \t%-g\xa", offset + 1 + j,
			    vvp->c[i].b.name, vvp->c[i].b.arg[j].name, 
			    vvp->c[i].b.arg[j].x);
                  
		    sprintf( msg_buf, "%d %s_%-30s \t%-g\n",
			     offset + 1 + j, vvp->c[i].b.name,
			     vvp->c[i].b.arg[j].name,
			     vvp->c[i].b.arg[j].x );
		    write_message( MonitorDataLogging );
		}
	    }
	}
    }
    fflush( datafile );
    sprintf( msg_buf, "wrote datafile ascii portion\n");
    write_message( MonitorDataLogging );
    return datafile;
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
    north0 = sensor(M_POS_N);
    east0  = sensor(M_POS_E);
}

void update_tab( float depth )
{
    if ( depth > max_depth )
	max_depth = depth;
}

int write_tab( void )
{
    FILE *ftab;
    char *run;
    int error = -1;

    snprintf( msg_buf, 256, "%s/%s.tab", DATA_DIR, VEHICLE_NAME );

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
		     vvp->mission_start_time, sensor(M_PRESENT_TIME), north0,
		     east0, max_depth );
	    fclose( ftab );
				/* generate affirmative diagnostic */
	    sprintf( msg_buf, "tabular entry appended to %s/%s.tab\n",
		     DATA_DIR, VEHICLE_NAME );
	    write_message( TERM | SLOG );
	}
	error = 0;
    }
    else
    {
				/* negative diagnostic */
	sprintf( msg_buf + strlen(msg_buf), ": %s\n", strerror(errno) );
	write_message( TERM | SLOG );
    }

    return error;
}
