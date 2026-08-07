/*----------------------------------------------------------------------*
   Copyright (C) 1994-1997, Massachusetts Institute of Technology.
	 Sea Grant Autonomous Underwater Vehicles Laboratory.
			 All rights reserved.
  $Id: sim_usbl.c,v 2.21 1998/01/20 00:31:51 bamoran Exp $
 *----------------------------------------------------------------------*/

#include <math.h>
#include <stdio.h>

#include "odyssey_data.h"
#include "vehicle.h"

int simulate_usbl( void )
{
#ifdef WHOI_USBL
    double xpos, ypos, zpos;
    double range, azim, elev;
    double dock_x, dock_y, dock_z;
    double usbl_hit_time = 0.5;

    dock_x = sensor(SIM_DOCK_N);
    dock_y = sensor(SIM_DOCK_E);
    dock_z = sensor(SIM_DOCK_D);

    xpos = sensor(REAL_X);
    ypos = sensor(REAL_Y);
    zpos = sensor(REAL_Z);

    range = sqrt(pow(dock_x - xpos,2.0) + pow(dock_y - ypos,2.0) +
		 pow(dock_z - zpos,2));
    azim = modPi( atan2(dock_y - ypos, dock_x - xpos) - sensor(M_YAW) );
    elev = atan2( dock_z - zpos, sqrt(pow(dock_x - xpos,2.0) +
				      pow(dock_y - ypos,2.0)) );

    if ((fmod(sensor(M_PRESENT_TIME),usbl_hit_time) < .02) && 
	((cos(elev) > 0.707) && (cos(azim) > 0.707)))
    {
	update(M_USBL_RANGE, range);
	update(M_USBL_AZIMUTH, azim);
	update(M_USBL_ELEVATION, elev);
    }
#endif /* WHOI_USBL */

    return 0;
}
