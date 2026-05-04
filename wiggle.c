/*-----------------------------------------------------------------------*
  Copyright (C) 1994-1998, Massachusetts Institute of Technology.
  Proprietary to Sea Grant AUV Laboratory.  All rights reserved.
  $Id: wiggle.c,v 2.22 1998/02/03 04:19:22 bamoran Exp $
 *-----------------------------------------------------------------------*/

#include <stdio.h>
#include <math.h>
#include <string.h>

#include "diagnostic.h"
#include "vehicle.h"

int stack_busy( struct command *cmd );

/*-----------------------------------------------------------------------*
  WIGGLE: B. A. Moran, 02-Feb-1998

  behavior: wiggle
    b_arg:                        time(s) 30
    b_arg:                      period(s) 6
    b_arg:          rudder_amplitude(rad) 0
    b_arg:        elevator_amplitude(rad) 0
    b_arg:         thruster_current(amps) 0
 *-----------------------------------------------------------------------*/

int wiggle( struct behavior *b, int priority, int level )
{
    register struct command *cmd = &vvp->c[priority];
    enum { TIME, PERIOD, R_AMP, E_AMP, THRUST };

    double reduced_time;

    if ( b->state == B_UNINITIALIZED )
    {
	if ( stack_busy(cmd) )
	    return -1;

	b->state = B_ACTIVE;
    }

    if ( manage_time(b, bArg(TIME)) == 1 )
    {
	b->state = B_COMPLETE;
	return 1;
    }
				/* reduced time corresponds to fraction of
				   this cycle elapsed (0 <= rt < 1)  */
    reduced_time = fmod(sensor(M_PRESENT_TIME) - b->start_time,
			bArg(PERIOD)) / bArg(PERIOD);
#ifdef DEBUG
    fprintf( stderr, "% 8.2f: reduced %4.2f\n", sensor(M_PRESENT_TIME),
	     reduced_time );
#endif

    cmd->speed_mode = S_CURRENT;
    cmd->speed[0] = bArg(THRUST);

    cmd->depth_mode = D_ELEVATOR;
    cmd->depth[0] = fabs( bArg(E_AMP) );
				/* change elevator during 2nd half of cycle */
    if ( 0.5 <= reduced_time )
	cmd->depth[0] *= -1;

    cmd->heading_mode = H_RUDDER;
    cmd->heading[0] = fabs( bArg(R_AMP) );
				/* change rudder during mid half of cycle */
    if ( 0.25 <= reduced_time && reduced_time < 0.75 )
	cmd->heading[0] *= -1;
    
    sprintf( msg_buf, "% 8.2f: %s E % .1f R % .1f (deg)\n",
	     sensor(M_PRESENT_TIME), b->name, R2D(cmd->depth[0]),
	     R2D(cmd->heading[0]) );
    write_message( MonitorLayeredControl );
    return 0;
}
