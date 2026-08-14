/*-----------------------------------------------------------------------*
  Copyright (C) 1994-1998, Massachusetts Institute of Technology.
  Proprietary to Sea Grant AUV Laboratory.  All rights reserved.
  $Id: whoi_homing.c,v 2.22 1998/02/03 04:18:33 bamoran Exp $
 *-----------------------------------------------------------------------*/

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "layered_control.h"
#include "odyssey_data_struct.h"
#include "odyssey_mission.h"
#include "vehicle.h"
#include "unit_utils.h"
#include "angle_utils.h"

int getXSDataByte( void );

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
	add_to_sensor( LATCH_CHECK, 1 );

				/* initialize the backup values */
	LAST_HDG[idreg] = bArg(HEADING);
	T_START[idreg] = read_sensor(M_PRESENT_TIME) ;
	LAST_TM[idreg] = read_sensor(M_USBL_TIME) ;
	T_LAST_TM[idreg] = read_sensor(M_PRESENT_TIME) ;

				/* state is now active */
	b->state = B_ACTIVE;
    }
				/* check to see if time has elapsed or MIO
				   switches indicate pole is in jaws and
				   stable for desired transient time */
    pole_secure = (read_sensor(M_POLE_PRESENT) > 0) &&
	(0.001*sensor_time(M_POLE_PRESENT) + bArg(TRNSNT) <=
	 read_sensor(M_PRESENT_TIME));

    rBucky = 0.5*read_sensor(U_SPEED_OF_SOUND)*(read_sensor(M_BUCKY_RAW) -
					   read_sensor(U_BUCKY_TAT));
    bailout = ( rBucky > 0 && rBucky <= bArg(BAIL) );

    if ( manage_time(b, bArg(TIME)) == 1 || pole_secure || bailout )
    {
	b->state = B_COMPLETE;
				/* release request to check MIO switches */
	add_to_sensor( LATCH_CHECK, -1 );

	return 1;
    }
				/* set the modes for the command vector */
    cmd->depth_mode = cmd->heading_mode = cmd->speed_mode = 1;
    				/* load default depth, speed, & head */
    cmd->speed[0] = bArg(SPEED);
    cmd->depth[0] = bArg(DEPTH);
    cmd->heading[0] = bArg(HEADING);

				/* look for new usbl data and nonzero az */
    new_data = read_sensor(M_USBL_TIME) > LAST_TM[idreg];
    if ( new_data && fabs(read_sensor(M_USBL_AZIMUTH)) > convert_deg_to_rad(0.01) )

	T_LAST_TM[idreg] = read_sensor(M_PRESENT_TIME);

				/* got a new hit --> home_in */
    if ( new_data && read_sensor(M_PRESENT_TIME) - T_START[idreg]
	 > 2*read_sensor(UO_USBL_UPDATE) )

	home_in( b, priority, level );

				/* didn't get a new hit; if it's a while
				   since the last hit, go straight, then
				   circle --> search */
    else if ( (read_sensor(M_PRESENT_TIME) - T_LAST_TM[idreg] > 
	       10*read_sensor(UO_USBL_UPDATE))
	      &&
	      (read_sensor(M_PRESENT_TIME) - T_START[idreg] >
	       2*read_sensor(UO_USBL_UPDATE)) )

	search( b, priority, level );
				/* didn't get a hit, but got one not too
				   long ago, or it's the beginning of the
				   behavior: use the backup values */
    else

	cmd->heading[0] = LAST_HDG[idreg] ;

				/* update the backup values */
    LAST_HDG[idreg] = cmd->heading[0];
    LAST_TM[idreg] = read_sensor( M_USBL_TIME );

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
    if ( (fabs(read_sensor(M_INERTIAL_AZ)) > convert_deg_to_rad(0.01)) &&
	 (fabs(read_sensor(M_INERTIAL_AZ)) < convert_deg_to_rad(89.0)) )

	az = read_sensor(M_INERTIAL_AZ);

    else			/* otherwise use old values */
	az = AZ_LAST[idreg];

				/* validate data & use if OK */
    if ( (fabs(read_sensor(M_INERTIAL_EL)) > convert_deg_to_rad(0.01)) &&
	 (fabs(read_sensor(M_INERTIAL_EL)) < convert_deg_to_rad(89.0)) )

	el = read_sensor(M_INERTIAL_EL);

    else			/* otherwise use old values */
	el = EL_LAST[idreg];

				/* if range rate > 0.5 but were closing,
				   note time, store recip hdg, and set flag */
    if ( read_sensor(M_USBL_RDOT_MEAN) > 0.5 && WAS_CLOSING[idreg] )
    {
	T_LOCK[idreg] = read_sensor(M_PRESENT_TIME);
	REALIGN[idreg] = wrap_angle( read_sensor(M_YAW) - M_PI );
	WAS_CLOSING[idreg] = 0;
    }
				/* go straight for specified time */
    if ( read_sensor(M_PRESENT_TIME) < T_LOCK[idreg] + read_sensor(UO_T_STRAIGHT) )  

	cmd->heading[0] = REALIGN[idreg];

				/* then turn around for 15 seconds */
    else if ( read_sensor(M_PRESENT_TIME) < T_LOCK[idreg] +
	      read_sensor(UO_T_STRAIGHT) + 15 )

	cmd->heading[0] = wrap_angle( REALIGN[idreg] + M_PI );

    else			/* otherwise, do regular homing */
    {
				/* if it's the first hit after no data,
				   say so */
	if ( !GOT_FIRST_GOOD[idreg] )
	{
	    GOT_FIRST_BAD[idreg] = 0;
	    GOT_FIRST_GOOD[idreg] = 1;
	}
				/* negative range rate implies closing,
				   make note here to prevent confusion
				   during the realign maneuver above */
	if ( read_sensor(M_USBL_RDOT_MEAN) < -0.5 )
	    WAS_CLOSING[idreg] = 1;

				/* command heading to beacon (BAM, JGB
				   offset to address USBL calibration
				   issues) */
	cmd->heading[0] = wrap_angle( az + read_sensor(M_YAW) + bArg(HDG_OFF) );

	rBucky = 0.5*read_sensor(U_SPEED_OF_SOUND)*(read_sensor(M_BUCKY_RAW) -
					       read_sensor(U_BUCKY_TAT));

	goal_dpt = read_sensor(M_DEPTH) - read_sensor(M_CNAV_AXIAL) +
	    read_sensor(U_DOCK_ZOFF);
	update_sensor( M_DOCK_D, goal_dpt );

	use_cnav = ( rBucky > 0 && rBucky < bArg(CNAV) );

	if ( use_cnav && read_sensor(M_CNAV_STATUS) != 0 )
	{
				/* command the goal here */
	    cmd->depth_mode = D_PITCH;
	    cmd->depth[0] = read_sensor(M_PITCH) + el;
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

	T_SRCH[idreg] = read_sensor(M_PRESENT_TIME);
	SRCH_HDG[idreg] = read_sensor(M_YAW);
    }
				/* increment desired heading if straight
				   time has elapsed */
    if ( read_sensor(M_PRESENT_TIME) >= T_SRCH[idreg] + read_sensor(UO_T_STRAIGHT) )
	
	SRCH_HDG[idreg] += read_sensor(UO_SEARCH_HR)*2;

    cmd->heading[0] = SRCH_HDG[idreg];

    return;
}