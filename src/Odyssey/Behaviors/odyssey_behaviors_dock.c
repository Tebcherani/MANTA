/*----------------------------------------------------------------------*
   Copyright (C) 1994-1997, Massachusetts Institute of Technology.
	 Sea Grant Autonomous Underwater Vehicles Laboratory.
			 All rights reserved.
  $Id: docking.c,v 2.22 1998/02/03 04:19:06 bamoran Exp $
 *----------------------------------------------------------------------*/

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "odyssey_config.h"
#include "odyssey_data_struct.h"
#include "odyssey_mission.h"
#include "vehicle.h"

/*----------------------------------------------------------------------*
  DOCK:  B.A. Moran, F.S. Hover, and H. Singh,  9 May 1997
  Just an idle time ... to be followed with undock() behavior.

  behavior: dock
    b_arg:                   idle_time(s) 30
    b_arg:                 thrust_time(s)  0
    b_arg:                     speed(m/s)  0
  *---------------------------------------------------------------------*/

int dock(struct behavior *b, int priority, int level) {
    enum {IDLETIME, FWD_TIME, SPEED};
    struct command *cmd = &vvp->c[priority];

/* If any other behavior above us don't execute.  Dock wants to do this
   because without such a feature, multiple dock behaviors simultaneously
   start on the first latch event.  CAVEAT: only whoi_homing knows to clear
   the command stack when the latch event occurs.  A setpoint, eg, will
   continue to command actuation until it times out.  Once that occurs,
   whoi_homing should immediately satisfy it's own exit criteria and
   release the stack directly to its companion dock behavior. */

    if (b->state == B_UNINITIALIZED) {
		if (stack_busy(cmd)) return -1;

		if (read_sensor(M_POLE_PRESENT) == 0) {
	    	b->state = B_COMPLETE;
	    	return 1;
		}

		update_sensor(C_LBL_CONTROL, 0);	// Stop pinging LBL
				/* request MIO for latch */
	add_to_sensor( LATCH_CHECK, 1 );
    }

    if ( manage_time(b, bArg(IDLETIME)) == 1 )
    {
				/* restart LBL */
	update_sensor( C_LBL_CONTROL, 1 );
				/* release MIO latch request */
	add_to_sensor( LATCH_CHECK, -1 );
	b->state = B_COMPLETE;
	return 1;
    }
				/* zero the fins */
    cmd->depth_mode = cmd->heading_mode = 0;
    cmd->depth[0] = cmd->heading[0] = 0;
				/* mode for speed */
    cmd->speed_mode = 1;
				/* apply thrust during beginning */
    if ( read_sensor(M_PRESENT_TIME) - b->start_time < bArg(FWD_TIME) )
	cmd->speed[0] = bArg(SPEED);
    else
	cmd->speed[0] = 0.;

    return 0;
}

/*------------------------------------------------------------------
  UNDOCK:  B.A. Moran, F.S. Hover, and H. Singh,  9 May 1997

  Start the latch motor & apply fwd thrust, then apply rev thrust. Repeat
  in cycleTime if still docked.

  behavior: undock
    b_arg:                  cycle_time(s) 60
    b_arg:              fwd_time_start(s) 0
    b_arg:                fwd_time_dur(s) 10
    b_arg:              rev_time_start(s) 11
    b_arg:                rev_time_dur(s) 19
    b_arg:                     speed(m/s) 0.6
    b_arg:                    rudder(rad) 0
    b_arg:                  elevator(rad) 0
 *-----------------------------------------------------------------*/

int undock( struct behavior *b, int priority, int level )
{
    enum { CYCLETIME, FWDSTART, FWDDUR, REVSTART, REVDUR, SPEED, RUDDER,
	   ELEVATOR };
    register struct command *cmd = &vvp->c[priority];
    double elapsed;

    if ( b->state == B_UNINITIALIZED )
    {
	if ( stack_busy(cmd) )
	    return 1;

	if ( read_sensor(M_POLE_PRESENT) == 0 )
	{
	    b->state = B_COMPLETE;
	    return 0;
	}
				/* borrow elapsed to calc total duration */
	if ( (elapsed = bArg(FWDSTART) + bArg(FWDDUR)) < 
	     bArg(REVSTART) + bArg(REVDUR) )
	    elapsed = bArg(REVSTART) + bArg(REVDUR);

				/* initialize start & end times */
	manage_time( b, elapsed );
				/* tell latch motor to start cycle */
	update_sensor( LATCH_START, 1 );
				/* submit request for i/o & logging */
	add_to_sensor( LATCH_CHECK, 1 );
				/* restart lbl pinging */
	update_sensor( C_LBL_CONTROL, 1 ) ;

	b->state = B_ACTIVE;
    }
				/* proper use of elapsed */
    elapsed = read_sensor(M_PRESENT_TIME) - b->start_time;

    cmd->heading_mode = 0;
    cmd->heading[0] = bArg(RUDDER);
    cmd->depth_mode = 0;
    cmd->depth[0] = bArg(ELEVATOR);
    cmd->speed_mode = 1;
				/* time for actuation has expired */
    if ( manage_time(b, 0) )
    {
				/* stop thruster */
	cmd->speed[0] = 0;

				/* done if pole absent, otherwise recycle
                                   behavior */
	if ( read_sensor(M_POLE_PRESENT) == 0 )
	{
				/* release request for i/o & logging */
	    add_to_sensor( LATCH_CHECK, -1 );
	    b->state = B_COMPLETE;
	    return 1;
	}
				/* pole present, recycle after cycletime
                                   elapsed */
	if ( elapsed > bArg(CYCLETIME) )
	{
	    b->state = B_UNINITIALIZED;
	    b->start_time = -1;
				/* release request here, will resubmit upon
				   reinitialization of behavior */
	    add_to_sensor( LATCH_CHECK, -1 );
	}
	return 0;
    }
				/* apply thrust */
    cmd->speed[0] = 0;

    if ( (elapsed > bArg(FWDSTART)) && 
	 (elapsed < bArg(FWDSTART) + bArg(FWDDUR)) )

	cmd->speed[0] += fabs(bArg(SPEED));

    else if ( (elapsed > bArg(REVSTART)) &&
	      (elapsed < bArg(REVSTART) + bArg(REVDUR)) ) 

	cmd->speed[0] -= fabs(bArg(SPEED)) ;

    return 0;
}