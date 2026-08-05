/*----------------------------------------------------------------------*
   Copyright (C) 1994-1997, Massachusetts Institute of Technology.
	 Sea Grant Autonomous Underwater Vehicles Laboratory.
			 All rights reserved.
  $Id: diagnostic.c,v 2.21 1998/01/20 00:31:51 bamoran Exp $
 *----------------------------------------------------------------------*/

#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "diagnostic.h"
#include "odyssey_log.h"
#include "paths.h"
#include "vehicle.h"

char msg_buf[256], syslog_file[256];
FILE *syslog, *term = NULL;


int open_syslog( const char *name )
{
    if ( !term ) term = stderr; /* set default terminal output */
                                /* no name means try env then the */
    if ( !name )                /* default if that fails */
    {
        name = getenv( "LOGDIR" );
        if ( name )
            sprintf( syslog_file, "%s/system.log", name );
        else			/* from paths.h */
            strcpy( syslog_file, SYSLOG_FILE );
    }
    else
        strcpy( syslog_file, name );
    fprintf( term, "opening %s as system log... ", syslog_file );
    fflush( term );
    syslog = fopen( syslog_file, "w" );
    if ( !syslog )
    {
        fputc( '\n', term );
        perror( syslog_file );
        syslog = term;		/* serious jleonard kludge */
        return -1;
    }
    fputs( "done\n", term );
    return 0;
}


/*-----------------------------------------------------------------------*
  write_message: send contents of msg_buf to appropriate destinations
  e.g., TERM, SLOG (defined in diagnostic.h)

  monitor_msg: compose and send string that reports name & value of
  particular sensor
 *-----------------------------------------------------------------------*/

void write_message( int monitor )
{
                                /* this checks bits of monitor */
    if ( monitor & TERM )
        fputs( msg_buf, term );
    if ( monitor & SLOG && syslog )
        fputs( msg_buf, syslog );
    return;
}

void monitor_msg( int sensor_id, int Monitor )
{
    sprintf( msg_buf, "% 8.2f: %29s % .3g\n", sensor(M_PRESENT_TIME),
	     sensor_name(sensor_id), sensor(sensor_id) );
    write_message( Monitor );
}

/*----------------------------------------------------------------------*
  Revision control message strings
 *----------------------------------------------------------------------*/

void write_rcs( const char *rev, const char *date )
{
    sprintf( msg_buf, "\t%.*s", strlen(rev+1)-2, rev+1 );
    if ( date )
	sprintf( &msg_buf[strlen(msg_buf)], ", %.*s", strlen(date+1)-2,
		 date+1 );
    strcat( msg_buf, "\n" );
    write_message( TERM | SLOG );
}


/*-------------------------------------------------------------------------
  Data structure diagnostic routines
  -----------------------------------------------------------------------*/

void dump_b_arg( FILE *fp, struct b_arg *ba )
{
    fprintf( fp, "% 29s % .6g\n", ba->name, ba->x );
}

void dump_behavior( FILE *fp, struct behavior *b )
{
    register int arg;

    fprintf( fp, "%s %d % .2g % .2g 0x%x 0x%x\n", b->name, b->state,
	     b->start_time, b->end_time, b->fcn, b->arg );
    for ( arg = 0; arg < MAX_B_ARG; arg++ )
    {
	fprintf( fp, "  arg % 2d: ", arg );
	dump_b_arg( fp, &b->arg[arg] );
    }
}
