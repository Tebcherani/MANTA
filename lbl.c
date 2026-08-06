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
#include "vehicle.h"

#if defined EDGEPS8000		/* EdgeTech PS8000 Transceiver */

int ErrorLBL = STATUS_NOT_SELECTED, MonitorLBL, UseLBL;
int LBLTransponderMode;

#endif /* EDGEPS8000, MITSG_LBL */
