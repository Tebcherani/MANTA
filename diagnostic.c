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

 *----------------------------------------------------------------------*/

static int dump_count = 0;

void init_dump( int Monitor )
{
    if ( Monitor & DUMP )
    {
	dump_count = 0;
	fputs( "0000000 ", syslog );
    }
}

int dump_char( char c, int Monitor )
{
    if ( Monitor & DUMP )
    {
	++dump_count;
	if ( !iscntrl(c) )
	    fprintf( syslog, " %c ", c );
	else
	    fprintf( syslog, "%02x ", c );
	if ( dump_count && !(dump_count % 0x10) )
	    fprintf( syslog, "\n%07x ", dump_count );
    }
}

void stop_dump( int Monitor )
{
    if ( Monitor & DUMP )
    {
	fputc( '\n', syslog );
    }
}

int dump_buf( char *buf, int len, int Monitor )
{
    if ( Monitor & DUMP )
    {
	int i;
	init_dump( Monitor );
	for ( i = 0; i < len; i++ )
	    dump_char( buf[i], Monitor );
	stop_dump( Monitor );
    }
}

/*-----------------------------------------------------------------------*
  Error messages, for now just making them up as I go, maybe later make it
  more systematic but for now this should do JJL 6-Feb-94
 *-----------------------------------------------------------------------*/

static char* status_tab[] = {
    "OK",
    "Not Selected",
    "Not Connected",
    "Uninitialized",
    "Couldn't open serial port",
    "Comms OK but no signals from sensor",
    "Sail1 error",
    "Fin comms OK but index failed",
    "Couldn't fork process"
};


void status_msg( enum ody_dev_status status, char *string )
{
    if ( status == STATUS_OK || status == STATUS_NOT_SELECTED )
	strcpy( string, status_tab[status] );
    else
	sprintf( string, "%*s", 15+strlen(status_tab[status]),
		 status_tab[status] );
    return;
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

void dump_sensor( FILE *fp, struct sensor *s )
{
    fprintf( fp, "% 29s % d % .6g\n", s->name, s->time, s->x );
}

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

void dump_command( FILE *fp, struct command *c )
{
    register int arg;
    fprintf( fp, "d_mode % d h_mode % d s_mode % d\n",
	     c->depth_mode, c->heading_mode, c->speed_mode );
    for ( arg = 0; arg < MAX_C_ARG; arg++ )
	fprintf( fp, "% 17.3f % 9.3f % 9.3f\n", c->depth[arg],
		 c->heading[arg], c->speed[arg] );
    fprintf( fp, "    beh % d: ", c->behavior );
    dump_behavior( fp, &c->b );
}

void dump_state( FILE *fp, struct state *s )
{
    register int cmd;
    fprintf( fp, "%s\n", s->name );
    for ( cmd = 0; cmd < MAX_COMMAND; cmd++ )
    {
	fprintf( fp, "cmd %d: ", cmd );
	dump_command( fp, &s->c[cmd] );
    }
}

void dump_data_structure( FILE *fp )
{
				/* local vars */
    register int sen, beh, cmd, sta;

    fprintf( fp, "c_date: %s\nm_date: %s\nf_name: %s\n",
	     vvp->creation_date, vvp->mission_date, vvp->data_file_name );

    fprintf( fp, "last_s: %d s: 0x%08x\n", vvp->last_s, vvp->s );
    for ( sen = 0; sen <= vvp->last_s; sen++ )
    {
	fprintf( fp, "  sen % 3d: ", sen );
	dump_sensor( fp, &vvp->s[sen] );
    }

    fprintf( fp, "last_b: %d b: 0x%08x\n", vvp->last_b, vvp->b );
    for ( beh = 0; beh <= vvp->last_b; beh++ )
    {
	fprintf( fp, "beh % 2d: ", beh );
	dump_behavior( fp, &vvp->b[beh] );
    }

    fprintf( fp, "last_c: %d c: 0x%08x\n", vvp->last_c, vvp->c );
    for ( cmd = 0; cmd <= vvp->last_c; cmd++ )
    {
	fprintf( fp, "cmd % d: ", cmd );
	dump_command( fp, &vvp->c[cmd] );
    }
    for ( sta = 0; sta < vvp->nstates; sta++ )
    {
	fprintf( fp, "sta %d: ", sta );
	dump_state( fp, &vvp->sta[sta] );
    }
}
