/*----------------------------------------------------------------------*
   Copyright (C) 1994-1997, Massachusetts Institute of Technology.
	 Sea Grant Autonomous Underwater Vehicles Laboratory.
			 All rights reserved.
  $Id: lbl.c,v 2.23 1998/02/03 04:25:07 bamoran Exp $
 *----------------------------------------------------------------------*/

#include <errno.h>		/* ANSI */
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "diagnostic.h"
#include "ody_serio.h"
#include "vehicle.h"

int  ody_open_port( char *name, int bau, int par, unsigned char eor );
int  edge8000_gate( void );
int  edge8000_trxs( void );
int  edge8000_ping( void );
void cyl_nav( double rBucky, double rLower );

#if defined EDGEPS8000		/* EdgeTech PS8000 Transceiver */

int ErrorLBL = STATUS_NOT_SELECTED, MonitorLBL, StatusLBL, UseLBL;
int LBLTransponderMode;

#define PS_MAX_REPLY 102	/* Length of $CCEGG,ARV,... */

				/* globals for serial i/o */
static ody_dev_t dev = { "EdgeTech PS8000" };
static serio_t serio = { EDGEPS8000, -1, B9600, N81, '\xa' };

static char rbuf[2*PS_MAX_REPLY + 1];
static char reply[PS_MAX_REPLY + 1];
static double next_ping_time;	/* time to ping next LBL cycle */


/*-----------------------------------------------------------------------*
  init_PS8000: initialize the EdgeTech PS8000 acoustic transceiver
 *-----------------------------------------------------------------------*/

int init_PS8000( void )
{
    if ( !UseLBL )
        return STATUS_NOT_SELECTED;

                                /* write RCS message */
    strcat( strcpy(msg_buf, dev.name), " ...\n" );
    write_message( TERM | SLOG );
    write_rcs( "$Revision: 2.23 $", "$Date: 1998/02/03 04:25:07 $" );

    dev.ser = serio_init( &serio, rbuf, sizeof(rbuf) );
    sprintf( msg_buf, "\tinput buffer 0x%08x, size %d\n", dev.ser->ibuf,
	     dev.ser->isize );
    write_message( TERM | SLOG );

                                /* open & verify serial port */
    dev.ser->path = ody_open_port( dev.ser->dev, dev.ser->bau,
				   dev.ser->par, dev.ser->eor );
    if ( dev.ser->path == -1 )
    {
	sprintf( msg_buf, "\t%s: %s\n", dev.ser->dev, strerror(errno) );
	write_message( TERM | SLOG );
	return STATUS_SERIAL_PORT;
    }
    sprintf( msg_buf, "\t%s: opened on %d\n", dev.ser->dev,
	     dev.ser->path );
    write_message( TERM | SLOG );

                                /* put in idle mode */
    send( dev.ser->path, "$M,0\xd" );
    if ( confirm(dev.ser->path, "$OK\xd\xa", 500) == -1 )
    {
        sprintf( msg_buf, "\nLBL Error idle ($M,0)\n\n");
        write_message( TERM | SLOG );
        return -1;
    }
                                /* ping lockout */
    send( dev.ser->path, "$V,1,0.03\xd" );
    if ( confirm(dev.ser->path, "$OK\xd\xa", 500) == -1 )
    {
        sprintf( msg_buf, "LBL Warning lockout ($V,1,0.03)\n" );
        write_message( TERM | SLOG );
    }
                                /* output power high */
    send( dev.ser->path, "$V,3,2\xd" );
    if ( confirm(dev.ser->path, "$OK\x0d\x0a", 500) == -1 )
    {
        strcpy( msg_buf, "LBL Warning power level ($V,3,2)\n" );
        write_message( TERM | SLOG );
    }
                                /* disable emergency slave */
    send( dev.ser->path, "$V,4,1\xd" );
    if ( confirm(dev.ser->path, "$OK\xd\xa", 500) == -1 )
    {
        strcpy( msg_buf, "LBL Error emergency slave ($V,4,1)\n" );
        write_message( TERM | SLOG );
        return -1;
    }
                                /* slave frequency */
    send( dev.ser->path, "$V,5,09.0\xd" );
    if ( confirm(dev.ser->path, "$OK\xd\xa", 500) == -1 )
    {
        strcpy( msg_buf, "LBL Error slave freq ($V,5,09.0)\n" );
        write_message( TERM | SLOG );
        return -1;
    }
                                /* set low/high range gates */
    return edge8000_gate();
}


int edge8000_mode( int mode, int slave )
{
    char strbuf[8];

				/* reenable emergency slave */
    if ( slave )
    {
	send( dev.ser->path, "$V,4,0\xd" );
	strcpy( msg_buf, "PS8000: reenable emergency slave... " );

	if ( confirm(dev.ser->path, "$OK\xd\xa", 100) == -1 )
	{
	    strcat( msg_buf, "Error!\n" );
	    write_message( TERM | SLOG );
	    return -1;
	}
    }
    else
	*msg_buf = '\0';

				/* set operation mode */
    sprintf( strbuf, "$M,%d\xd", 1 + mode );
    send( dev.ser->path, strbuf );
    sprintf( msg_buf + strlen(msg_buf), "set mode ($M,%d)... ", 1 + mode );

    if ( confirm(dev.ser->path, "$OK\xd\xa", 100) == -1 )
    {
        strcat( msg_buf, "Error!\n" );
        write_message( TERM | SLOG );
        return -1;
    }
				/* flush diagnostics */
    strcat( msg_buf, "OK\n" );
    write_message( TERM | SLOG );

    return 0;
}


int edge8000_gate( void )
{
    char strbuf[192];
    double low, high;
    int i;
				/* empirical observations indicate timeout
				   should be about 0.5 s shorter than cycle */
    if ( (high = sensor(U_LBL_CYCLE) - 0.5) > sensor(U_LBL_TIMEOUT) )
	high = sensor( U_LBL_TIMEOUT );

    if ( (low = sensor(U_LBL_LOCKOUT)) > high - 0.5 )
	low = 0;

    strcpy( strbuf, "$V,2" );
    for ( i = 0; i < 10; i++ )
        sprintf( strbuf + strlen(strbuf), ",%07.4f,%07.4f", low, high );
    strcat( strbuf, "\xd" );
    send( dev.ser->path, strbuf );

    sprintf( msg_buf, "% 8.2f: PS8000 gates [%.4f, %.4f] ... ",
	     sensor(M_PRESENT_TIME), low, high );

    if ( confirm(dev.ser->path, "$OK\xd\xa", 200) == -1 )
    {
        strcat( msg_buf, "Error!\n" );
        write_message( TERM | SLOG );
        return -1;
    }
    strcat( msg_buf, "OK\n" );
    write_message( MonitorLBL | SLOG );
    return 0;
}




int ctrl_PS8000( void )
{
				/* prototypes */
    int edge8000_arvQ( double );
    void edge8000_sltQ( double );
				/* local vars */
    double read_time;
                                /* read line, process if data available */
    if ( serio_gets(reply, sizeof(reply), dev.ser) )
    {
	read_time = 0.001*millisec();
				/* check for arrivals */
        if ( !edge8000_arvQ(read_time) )
                                /* not arrivals, was it SLT? */
            edge8000_sltQ( read_time );
    }
    else
    {
				/* choose between XS or ping */
	if ( sensor(EDGE_XS_DATABYTE) > 0 )
	    return edge8000_trxs();

	if ( sensor(M_PRESENT_TIME) >= next_ping_time )
	    return edge8000_ping();
    }
    return 0;
}


/*----------------------------------------------------------------------*
  Ser No     t1   t2   t3   t4   t5   t6   t7   t8   t9   ta     Veh
  ???????    7.5  8.0 13.5 13.0 11.0 10.5 10.0  9.5  9.0  8.5   Borealis
  ???????    8.0 11.5 13.5 13.0 11.0 10.5 10.0  9.5  9.0  8.5   Testes
  M967016   11.0  9.0 13.5  8.0  8.5  9.5 10.0 10.5 12.5 13.0   not inst
 *----------------------------------------------------------------------*/

int edge8000_arvQ( double when )
{
				/* NB: ta refers to channel 10 */
    float t1, t2, t3, t4, t5, t6, t7, t8, t9, ta;
    int nconv, gotA, gotB;
    char *sor;
#ifdef WHOI_USBL
    double rBucky, rLower;
#endif

    if ( sensor(M_PRESENT_TIME) < sensor(U_LBL_TIMEOUT) )

	return 0;

    if ( sor = strstr(reply, "$CCEGG,ARV,") )
    {
        nconv = sscanf( sor, "$CCEGG,ARV,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f",
                        &t1, &t2, &t3, &t4, &t5, &t6, &t7, &t8, &t9, &ta );
	if ( nconv < 10  )
	{
	    sprintf( msg_buf, "% 8.2f: PS8000 scan error %d (%s)\n",
		     sensor(M_PRESENT_TIME), nconv, sor );
	    write_message( MonitorLBL | SLOG );
	    return -1;
	}
				/* store the values */
#if   defined _WHOI
        update( M_LBL_1T, t6 );	/* 10.5 kHz */
        update( M_LBL_2T, t8 );	/*  9.5 kHz */
        update( M_LBL_3T, ta );	/*  8.5 kHz */
        update( M_LBL_4T, t1 );	/*  8.0 kHz */
#elif defined _BOREALIS
        update( M_LBL_1T, t6 );	/* 10.5 kHz */
        update( M_LBL_2T, t8 );	/*  9.5 kHz */
        update( M_LBL_3T, ta );	/*  8.5 kHz */
        update( M_LBL_4T, t2 );	/*  8.0 kHz */
#endif
	update( LBL_NEWDATA, 1 );

#ifdef WHOI_USBL
	update( M_BUCKY_RAW, t3 );
	update( M_LOWER_RAW, t4 );
	update( M_CNAV_STATUS, 0 );

	if ( sensor(M_BUCKY_RAW) > 0 )
	    rBucky = 0.5*( sensor(M_BUCKY_RAW) - sensor(U_BUCKY_TAT) )
		* sensor(U_SPEED_OF_SOUND);
	else rBucky = 0;

	rLower = 0.5*( sensor(M_LOWER_RAW) - sensor(U_LOWER_TAT) )
	    * sensor(U_SPEED_OF_SOUND);

	if ( rLower >= 0 )
	    cyl_nav( rBucky, rLower );

	if ( rBucky > 0 )
	{
	    rBucky += sensor(M_USBL_RDOT_MEAN)*( sensor(M_PRESENT_TIME) -
						 sensor(M_LBL_PING_TIME) -
						 0.5*sensor(M_BUCKY_RAW) );
	    update( M_USBL_RANGE, rBucky );
	}
#endif

        sprintf( msg_buf, "% 8.2f: %2d ARV, %8.4f %8.4f %8.4f "
                 "%8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f\n",
                 when, nconv, t1, t2, t3, t4, t5, t6, t7, t8, t9, ta );
        write_message( MonitorLBL );

    /* REMOTE ABORT: if 10.5 kHz reply occurs on t8 in 1-1.5 range
       (gotA) and 10.0 kHz reply occurs on t7 in 2-2.5 range (gotB),
       then trigger abort mission. */

	gotA = (1.0 <= t3) && (t3 <= 1.5);
	gotB = (2.0 <= t7) && (t7 <= 2.5);

	if ( gotA && gotB )
	{
	    update( C_ABORT_MISSION, 1 );
	    sprintf( msg_buf, "% 8.2f: LBL detects abort (A %4.2f, "
		     "B %4.2f)\n", when, t3, t7 );
	    write_message( TERM | SLOG );
	}
	return 1;
    }
    return 0;
}

/*-----------------------------------------------------------------------*
  edge8000_sltQ: detect slave transmit.  If emergency mode is enabled,
  PS8000 will enter slave mode after 60 seconds of inactivity.  When in
  slave mode, it still allow "master" pings, but prints SLT rather than
  MAT.  This function checks the desired mode (LBLTransponderMode) and
  attempts to restore master mode upon detection of SLT.  We log the ping
  time only if this was a true slave detect because master pings are logged
  upon confirmation of the X,ff.f command.
 *-----------------------------------------------------------------------*/

void edge8000_sltQ( double when )
{
    double nextCycle;

    if ( strstr(reply, "$CCEGG,SLT,") )
    {
				/* attempt to restore master */
	if ( !LBLTransponderMode )
	{
	    sprintf( msg_buf, "% 8.2f: PS8000 unwanted SLT\n", when );
	    write_message( MonitorLBL | SLOG );
	    return;
	}
				/* advance next ping by integral # cycles */
	nextCycle = ceil((when + sensor(U_LBL_TIMEOUT))/sensor(U_LBL_CYCLE));
	next_ping_time = nextCycle*sensor(U_LBL_CYCLE);
				/* log SLT detection */
        update( M_LBL_PING_TIME, when );
				/* diagnostics */
        sprintf( msg_buf, "% 8.2f: PING! (SLT)\n", when );
        write_message( MonitorLBL );
    }
}

int edge8000_trxs( void )
{
    char trxs[32];

    if ( sensor(M_PRESENT_TIME) < sensor(M_LBL_PING_TIME) +
	 sensor(U_LBL_TIMEOUT) )
	return 0;
				/* put in idle mode for XS */
    send( dev.ser->path, "$M,0\xd" );
    if ( confirm(dev.ser->path, "$OK\xd\xa", 100) == -1 )
    {
        sprintf( msg_buf, "% 8.2f: PS8000 error ($M,0)\n",
                 sensor(M_PRESENT_TIME) );
        write_message( MonitorLBL | SLOG );
        return -1;
    }
    sprintf( trxs, "$TR,XS,%d,%d\xd", (int) sensor(EDGE_AM200_BACS),
	     (int) sensor(EDGE_XS_DATABYTE) );

    send( dev.ser->path, trxs );
    sprintf( msg_buf, "% 8.2f: PS8000 %.*s... ", sensor(M_PRESENT_TIME),
	     strlen(trxs) - 2, trxs );

    if ( confirm(dev.ser->path, "$UV,OK\xd\xa", 100) == -1 )
    {
	strcat( msg_buf, "Error!\n" );
	write_message( MonitorLBL | SLOG );
    }
    else
    {
	update( EDGE_TR_XS_TIME, 0.001*millisec() );
	update( EDGE_XS_DATABYTE, 0 );
	strcat( msg_buf, "OK\n" );
	write_message( MonitorLBL );
    }

    /* It takes the PS8000 about 35 sec to send the databyte and reply to
       the mode restoration command.  Suppress navigation pings during this
       interval by advancing the next ping time the smallest integral
       number of cycles >= 35 sec */

    next_ping_time += sensor(U_LBL_CYCLE)*ceil( 35/sensor(U_LBL_CYCLE) );

    return 0;
}

int edge8000_ping( void )
{
    if ( sensor(C_LBL_CONTROL) > 0 )
    {
	send( dev.ser->path, "$X,09.0\xd" );
	if ( confirm(dev.ser->path, "$OK\xd\xa", 80) == -1 )
	{
	    sprintf( msg_buf, "% 8.2f: PS8000 error ($X,09.0)\n",
		     sensor(M_PRESENT_TIME)  );
	    write_message( MonitorLBL | SLOG );
	    return -1;
	}
				/* log ping and advance cycle */
	update( M_LBL_PING_TIME, 0.001*millisec() );
	next_ping_time += sensor(U_LBL_CYCLE);
	sprintf( msg_buf, "% 8.2f: PING!\n", sensor(M_LBL_PING_TIME) );
	write_message( MonitorLBL );
    }
    return 0;
}

#endif /* EDGEPS8000, MITSG_LBL */
