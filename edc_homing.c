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
    return 0;
}
