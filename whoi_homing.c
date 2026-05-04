/*-----------------------------------------------------------------------*
  Copyright (C) 1994-1998, Massachusetts Institute of Technology.
  Proprietary to Sea Grant AUV Laboratory.  All rights reserved.
  $Id: whoi_homing.c,v 2.22 1998/02/03 04:18:33 bamoran Exp $
 *-----------------------------------------------------------------------*/

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "diagnostic.h"
#include "vehicle.h"

int stack_busy( struct command *cmd );
int getXSDataByte( void );

#ifdef WHOI_USBL
				/* prototypes */
void home_in( struct behavior *b, int priority, int level );
void search( struct behavior *b, int priority, int level );

				/* storage for static variables this
				   behavior uses, handles up to NWH
				   instances of whoi_homing */
#define NWH 10

static double T_START[NWH], T_LAST_TM[NWH];
static int WAS_CLOSING[NWH], GOT_FIRST_GOOD[NWH], GOT_FIRST_BAD[NWH];
static double LAST_TM[NWH] = { -1e6, -1e6, -1e6, -1e6, -1e6, -1e6,
			       -1e6, -1e6, -1e6, -1e6 };
static double T_LOCK[NWH] = { -1e4, -1e4, -1e4, -1e4, -1e4, -1e4,
			      -1e4, -1e4, -1e4, -1e4 };
static double AZ_LAST[NWH], EL_LAST[NWH], LAST_HDG[NWH], REALIGN[NWH];
static double T_SRCH[NWH], SRCH_HDG[NWH];
static struct behavior *BB[NWH];
static int idreg;

/*-----------------------------------------------------------------------*
  whoi_homing is the basic behavior to handle all the layered control
  software issues.

  behavior: whoi_homing
    b_arg:                        time(s) 0
    b_arg:                     speed(m/s) 0
    b_arg:                       depth(m) 0
    b_arg:                   heading(rad) 0
    b_arg:                         gain() 0
    b_arg:                  direction_k() 0
    b_arg:            heading_offset(rad) 0
    b_arg:         pole_transient_time(s) 0
    b_arg:               bailout_range(m) -1e6
    b_arg:           cnav_enable_range(m) 100
 *-----------------------------------------------------------------------*/

enum { TIME, SPEED, DEPTH, HEADING, GAIN, K_DIR, HDG_OFF, TRNSNT, BAIL, CNAV };

int whoi_homing( struct behavior *b, int priority, int level )
{
    struct command *cmd = &vvp->c[priority];
    int pole_secure, bailout, new_data;
    double rBucky;

				/* dynamic registry for statics to allow
				   multiple instances of whoi_homing to
				   occur in a single mission */
    for ( idreg = 0; idreg < NWH; idreg++ )
    {
				/* store values for unfamiliar b */
	if ( !BB[idreg] ) BB[idreg] = b;
				/* grab values for familiar b */
	if ( b == BB[idreg] ) break;
    }
				/* if any other behavior above us in stack
				   is controlling vehicle, don't execute */
    if ( b->state == B_UNINITIALIZED )
    {
	if ( stack_busy(cmd) )
	    return 1;
				/* submit request to check MIO switches */
	update_add( LATCH_CHECK, 1 );

				/* initialize the backup values */
	LAST_HDG[idreg] = bArg(HEADING);
	T_START[idreg] = sensor(M_PRESENT_TIME) ;
	LAST_TM[idreg] = sensor(M_USBL_TIME) ;
	T_LAST_TM[idreg] = sensor(M_PRESENT_TIME) ;

				/* state is now active */
	b->state = B_ACTIVE;
    }
				/* check to see if time has elapsed or MIO
				   switches indicate pole is in jaws and
				   stable for desired transient time */
    pole_secure = (sensor(M_POLE_PRESENT) > 0) &&
	(0.001*sensor_time(M_POLE_PRESENT) + bArg(TRNSNT) <=
	 sensor(M_PRESENT_TIME));

    rBucky = 0.5*sensor(U_SPEED_OF_SOUND)*(sensor(M_BUCKY_RAW) -
					   sensor(U_BUCKY_TAT));
    bailout = ( rBucky > 0 && rBucky <= bArg(BAIL) );

    if ( manage_time(b, bArg(TIME)) == 1 || pole_secure || bailout )
    {
	b->state = B_COMPLETE;
				/* release request to check MIO switches */
	update_add( LATCH_CHECK, -1 );
				/* compose start of exit message, remainder
				   to be determined by exit condition */
	sprintf( msg_buf, "% 8.2f: %s ", sensor(M_PRESENT_TIME),
		 b->name );

	if ( pole_secure )
	{
#ifdef EDGEPS8000
				/* send "I'm home" to dock */
	    update( EDGE_XS_DATABYTE, getXSDataByte() );
#endif
	    strcat( msg_buf, "done with pole secure\n" );
	}
	else if ( bailout )

	    sprintf( msg_buf+strlen(msg_buf), "bails with Rb %.1f < %.1f\n",
		     rBucky, bArg(BAIL) );
	else
	    return 1;
				/* write exit message */
	write_message( MonitorLayeredControl | SLOG );
	return 1;
    }
				/* set the modes for the command vector */
    cmd->depth_mode = cmd->heading_mode = cmd->speed_mode = 1;
    				/* load default depth, speed, & head */
    cmd->speed[0] = bArg(SPEED);
    cmd->depth[0] = bArg(DEPTH);
    cmd->heading[0] = bArg(HEADING);

				/* look for new usbl data and nonzero az */
    new_data = sensor(M_USBL_TIME) > LAST_TM[idreg];
    if ( new_data && fabs(sensor(M_USBL_AZIMUTH)) > D2R(0.01) )

	T_LAST_TM[idreg] = sensor(M_PRESENT_TIME);

				/* got a new hit --> home_in */
    if ( new_data && sensor(M_PRESENT_TIME) - T_START[idreg]
	 > 2*sensor(UO_USBL_UPDATE) )

	home_in( b, priority, level );

				/* didn't get a new hit; if it's a while
				   since the last hit, go straight, then
				   circle --> search */
    else if ( (sensor(M_PRESENT_TIME) - T_LAST_TM[idreg] > 
	       10*sensor(UO_USBL_UPDATE))
	      &&
	      (sensor(M_PRESENT_TIME) - T_START[idreg] >
	       2*sensor(UO_USBL_UPDATE)) )

	search( b, priority, level );
				/* didn't get a hit, but got one not too
				   long ago, or it's the beginning of the
				   behavior: use the backup values */
    else

	cmd->heading[0] = LAST_HDG[idreg] ;

				/* update the backup values */
    LAST_HDG[idreg] = cmd->heading[0];
    LAST_TM[idreg] = sensor( M_USBL_TIME );

    return 0;
}

/*-----------------------------------------------------------------------*
  home_in: handles the inbound portion of homing.  Also detects missed
  approaches and performs suitable maneuver in an attempt to realign with
  the approach direction of the missed attempt.
 *-----------------------------------------------------------------------*/

void home_in( struct behavior *b, int priority, int level )
{
    struct command *cmd = &vvp->c[priority];
    double az, el, goal_dpt, rBucky;
    int use_cnav;

				/* validate data & use if OK */
    if ( (fabs(sensor(M_INERTIAL_AZ)) > D2R(0.01)) &&
	 (fabs(sensor(M_INERTIAL_AZ)) < D2R(89.0)) )

	az = sensor(M_INERTIAL_AZ);

    else			/* otherwise use old values */
	az = AZ_LAST[idreg];

				/* validate data & use if OK */
    if ( (fabs(sensor(M_INERTIAL_EL)) > D2R(0.01)) &&
	 (fabs(sensor(M_INERTIAL_EL)) < D2R(89.0)) )

	el = sensor(M_INERTIAL_EL);

    else			/* otherwise use old values */
	el = EL_LAST[idreg];

				/* if range rate > 0.5 but were closing,
				   note time, store recip hdg, and set flag */
    if ( sensor(M_USBL_RDOT_MEAN) > 0.5 && WAS_CLOSING[idreg] )
    {
	T_LOCK[idreg] = sensor(M_PRESENT_TIME);
	REALIGN[idreg] = modPi( sensor(M_YAW) - PI );
	WAS_CLOSING[idreg] = 0;

	sprintf( msg_buf, "% 8.2f: %s missed, turn 180 (%03d -> %03d T)\n",
		 sensor(M_PRESENT_TIME), b->name, (int) R2D(sensor(M_YAW)),
		 (int) R2D(REALIGN[idreg]) );
	write_message( MonitorLayeredControl | SLOG );
    }
				/* go straight for specified time */
    if ( sensor(M_PRESENT_TIME) < T_LOCK[idreg] + sensor(UO_T_STRAIGHT) )  

	cmd->heading[0] = REALIGN[idreg];

				/* then turn around for 15 seconds */
    else if ( sensor(M_PRESENT_TIME) < T_LOCK[idreg] +
	      sensor(UO_T_STRAIGHT) + 15 )

	cmd->heading[0] = modPi( REALIGN[idreg] + PI );

    else			/* otherwise, do regular homing */
    {
				/* if it's the first hit after no data,
				   say so */
	if ( !GOT_FIRST_GOOD[idreg] )
	{
	    GOT_FIRST_BAD[idreg] = 0;
	    GOT_FIRST_GOOD[idreg] = 1;

	    sprintf( msg_buf, "% 8.2f: %s 1st good\n",
		     sensor(M_PRESENT_TIME), b->name );
	    write_message( MonitorLayeredControl | SLOG );
	}
				/* negative range rate implies closing,
				   make note here to prevent confusion
				   during the realign maneuver above */
	if ( sensor(M_USBL_RDOT_MEAN) < -0.5 )
	    WAS_CLOSING[idreg] = 1;

				/* command heading to beacon (BAM, JGB
				   offset to address USBL calibration
				   issues) */
	cmd->heading[0] = modPi( az + sensor(M_YAW) + bArg(HDG_OFF) );

				/* do cyl_nav depth */
	rBucky = 0.5*sensor(U_SPEED_OF_SOUND)*(sensor(M_BUCKY_RAW) -
					       sensor(U_BUCKY_TAT));

	goal_dpt = sensor(M_DEPTH) - sensor(M_CNAV_AXIAL) +
	    sensor(U_DOCK_ZOFF);
	update( M_DOCK_D, goal_dpt );

	use_cnav = ( rBucky > 0 && rBucky < bArg(CNAV) );

	if ( use_cnav && sensor(M_CNAV_STATUS) != 0 )
	{
				/* command the goal here */
	    cmd->depth_mode = D_PITCH;
	    cmd->depth[0] = sensor(M_PITCH) + el;
	}
    }

    AZ_LAST[idreg] = az;
    EL_LAST[idreg] = el;

    return;
}

/*----------------------------------------------------*/

void search(  struct behavior *b, int priority, int level )
{
    struct command *cmd = &vvp->c[priority];

				/* if first time step with no data, save
				   time, set heading, speed, and depth */
    if ( !GOT_FIRST_BAD[idreg] )
    {
	GOT_FIRST_GOOD[idreg] = 0;
	GOT_FIRST_BAD[idreg] = 1; 

	T_SRCH[idreg] = sensor(M_PRESENT_TIME);
	SRCH_HDG[idreg] = sensor(M_YAW);

	sprintf( msg_buf, "% 8.2f: %s 1st bad hit, go straight (%03d T)\n",
		 sensor(M_PRESENT_TIME), b->name, (int) R2D(SRCH_HDG[idreg]) );
	write_message( MonitorLayeredControl | SLOG );
    }
				/* increment desired heading if straight
				   time has elapsed */
    if ( sensor(M_PRESENT_TIME) >= T_SRCH[idreg] + sensor(UO_T_STRAIGHT) )
	
	SRCH_HDG[idreg] += sensor(UO_SEARCH_HR)*2;

    cmd->heading[0] = SRCH_HDG[idreg];

    return;
}

/*-----------------------------------------------------------------------*
  cyl_nav: D. Yoerger's cylindrical navigation solution to the vertical
  component of the AUV homing problem.  After much deliberation (I'm being
  nice) we've decided to add a 2nd LBL transponder about 100 m below the
  docking station.  This approach uses two ranges to calculate a
  cylindrical solution, axisymmetric about the possibly tilted mooring
  wire, that will define a goal depth for the vehicle to attain.

  Rbucky: acoustic range to homing beacon ("Bucky" ball)
  Rlower: acoustic range to the LOWER xpndr

  B. A. Moran, 12-Jan-1998
 *-----------------------------------------------------------------------*/

void cyl_nav( double Rbucky, double Rlower )
{
    double ztemp, arg, vbl;

    vbl = sensor( U_DOCK_VBL );
				/* if we're on the baseline, use the range
				   to the lower xpndr as the axial value */
    if ( Rbucky + Rlower < vbl )
    {
    	update( M_CNAV_AXIAL, vbl - Rlower );
        update( M_CNAV_RADIAL, 0 );
	update( M_CNAV_STATUS, 1 );
	return;
    }
				/* quadratic form of solution, checks
                                   discrimanent for existence */
    ztemp = (-Rlower*Rlower + Rbucky*Rbucky + vbl*vbl)/(2.0*vbl) ;
    arg = Rbucky*Rbucky - ztemp*ztemp ;

				/* imaginary sol'n would arise if circles
				   fail to intersect */
    if ( arg < 0 )
    {
	update( M_CNAV_STATUS, 0 );
	return;
    }
				/* store real, valid sol'n in data struct */
    update( M_CNAV_RADIAL, sqrt(arg) );
    update( M_CNAV_AXIAL, ztemp );
    update( M_CNAV_STATUS, 1 );
}

#endif /* WHOI_USBL */
