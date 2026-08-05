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
void cyl_nav( double rBucky, double rLower );

#if defined EDGEPS8000		/* EdgeTech PS8000 Transceiver */

int ErrorLBL = STATUS_NOT_SELECTED, MonitorLBL, StatusLBL, UseLBL;
int LBLTransponderMode;

				/* globals for serial i/o */
static ody_dev_t dev = { "EdgeTech PS8000" };
static serio_t serio = { EDGEPS8000, -1, B9600, N81, '\xa' };

#endif /* EDGEPS8000, MITSG_LBL */
