/*----------------------------------------------------------------------*
   Copyright (C) 1994-1997, Massachusetts Institute of Technology.
	 Sea Grant Autonomous Underwater Vehicles Laboratory.
			 All rights reserved.
  $Id: waypoint.c,v 2.22 1998/02/03 04:25:07 bamoran Exp $
 *----------------------------------------------------------------------*/

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "common_defs.h"
#include "odyssey_data_struct.h"
#include "odyssey_mission.h"
#include "vehicle.h"
#include "angle_utils.h"
#include "unit_utils.h"

/*----------------------------------------------------------------------*
  WAYPOINT_2D: attempts to achieve a position using the specified direction
  of approach.

behavior: waypoint_2d
  b_arg:                       wpt_n(m)  0
  b_arg:                       wpt_e(m)  0
  b_arg:                       depth(m)  0
  b_arg:                     speed(m/s)  0
  b_arg:                     capture(m)  1
  b_arg:                  circle_mode()  1
  b_arg:                   heading(rad) -1
  b_arg:                    gain(rad/m)  0.0175
  b_arg:                     timeout(s)  0
  b_arg:             abort_on_timeout()  0
 *----------------------------------------------------------------------*/

int waypoint_2d( struct behavior *b, int priority, int level )
{
    enum { NRTH, EAST, DPTH, SPD, CAPTURE, MODE, HDG, GAIN, TIME, ABORT };

    register struct command *cmd = &vvp->c[priority];
    double goal, n_est, e_est, dn, de, dtw, xte, kxte;
				/* statics to conserve flops, checked
				   against moving arguments for validity */
    static double psi_t = 0, spsi = 0, cpsi = 1;

    switch ( b->state )
    {
    case B_UNINITIALIZED:
				/* enter only on clear stack */
	if ( stack_busy(cmd) )
	    return 1;
				/* compute "direct to" trackline */
	if ( bArg(HDG) < 0 )
	{
	    bArg(HDG) = M_PI + wrap_angle( atan2(bArg(EAST) - read_sensor(M_POS_E),
				    bArg(NRTH) - read_sensor(M_POS_N)) - M_PI );
	}

	b->state = B_ACTIVE;

    case B_ACTIVE:

	if ( manage_time(b, bArg(TIME)) == 1 )
	{
	    if ( bArg(ABORT) > 0 )
	    {
		update_sensor( C_ABORT_MISSION, 1 );
	    }
	    b->state = B_COMPLETE;
	    return 0;
	}
				/* remember values to save flops, must be
				   validated before using */
	if ( psi_t != bArg(HDG) )
	{
	    psi_t = bArg(HDG);
	    spsi = sin( psi_t );
	    cpsi = cos( psi_t );
	}
				/* distance to waypoint in N, E coords */
	n_est = read_sensor(M_POS_N);
	e_est = read_sensor(M_POS_E);
	dn = n_est - bArg(NRTH);
	de = e_est - bArg(EAST);

				/* transform into dtw, xte coordinates */
	if ( bArg(MODE) != 0 )
	{
	    dtw = sqrt( dn*dn + de*de );
	    goal = fabs( bArg(CAPTURE) );
	}
	else
	{
	    dtw = -de*spsi - dn*cpsi;
	    goal = -bArg(CAPTURE);
	}
	xte = -de*cpsi + dn*spsi;
				/* update values in data structure */
	update_sensor( WP_XTE, xte );
	update_sensor( WP_DTW, dtw );
	
				/* load outputs */
	if ( dtw <= goal )
	{
	    b->state = B_COMPLETE;
	    cmd->depth_mode = cmd->heading_mode = cmd->speed_mode = -1;
	}
	else
	{
	    cmd->depth_mode = cmd->heading_mode = cmd->speed_mode = 1;

	    cmd->depth[0]   = bArg( DPTH );
	    cmd->speed[0]   = bArg( SPD );

				/* apply gain only if within 90 degrees of
				   desired */
	    kxte   = bArg(GAIN) * xte ;

	    if ( fabs(kxte) < convert_deg_to_rad(90) )
		cmd->heading[0] = wrap_angle( psi_t + kxte );
	    else
		cmd->heading[0] = atan2( -de, -dn );
	}

	break;
    }
}
