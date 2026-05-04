/*----------------------------------------------------------------------*
   Copyright (C) 1994-1997, Massachusetts Institute of Technology.
	 Sea Grant Autonomous Underwater Vehicles Laboratory.
			 All rights reserved.
  $Id: edc_homing.c,v 2.21 1998/01/20 00:31:51 bamoran Exp $
 *----------------------------------------------------------------------*/

#include <stdio.h>
#include <math.h>
#include <string.h>

#include "diagnostic.h"
#include "vehicle.h"

/* ----------------------------------------------------------------
   EDC_HOMING: (08/23/95, B.A.Moran, J.G.Bellingham, M.Feezor)

   homes in on EDC docking mechanism at commanded speed.  when axial
   range estimate falls within range_cutout, behavior commands speed
   of zero, but does not terminate mission.

   Arguments are:
    0  "elev_gain(num)", 0.000000, 0, 0,
    1  "azim_gain(num)", 0.000000, 0, 0,
    2  "range_cutout(m)", 0.000000, 0, 0,
    3  "speed(m/s)", 0.000000, 0, 0,
 * ---------------------------------------------------------------- */

int edc_homing( struct behavior *b, int priority, int level )
{
#ifdef _EDC
    const int ElevGain = 0, AzimGain = 1, RangeCutout = 2, Speed = 3;

                                /* Don't run this behavior if no lock */
                                /* obtained */
    if ( sensor(E_EDC_AXIAL_RANGE) > 10 )
        return 0;
                                /* If complete, shut vehicle down by */
                                /* commanding zero speed */
    if ( sensor(E_EDC_AXIAL_RANGE) < b->arg[RangeCutout].x)
    {
        vvp->c[priority].speed[0] = 0.0;
        sprintf( msg_buf, "t=%.2f s: EDC_HOMING complete with axial "
                "range value %.2f m\n", 0.001*millisec(),
                sensor(E_EDC_AXIAL_RANGE) );
        write_message( TERM | SLOG );
        return 1;
    }
                                /* Load the output commands... */
    vvp->c[priority].heading_mode = 1;
    vvp->c[priority].depth_mode = 2;
    vvp->c[priority].speed_mode = 1;
  
    vvp->c[priority].heading[0] = sensor(E_EDC_B_BEARING);
    vvp->c[priority].depth[0] = sensor(E_EDC_B_INCLINATION);
    vvp->c[priority].speed[0] = b->arg[Speed].x;

#endif
    return 0;
}
