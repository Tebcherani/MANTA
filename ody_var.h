/*----------------------------------------------------------------------*
   Copyright (C) 1994-1998, Massachusetts Institute of Technology.
	 Sea Grant Autonomous Underwater Vehicles Laboratory.
			 All rights reserved.
  $Id: ody_var.h,v 2.23 1998/02/03 04:25:07 bamoran Exp $
 *----------------------------------------------------------------------*/

#ifndef ODY_VAR_H
#define ODY_VAR_H

#include "ody_defs.h"
#include "ody_types.h"

extern const struct behavior bTemplate[];
extern const struct sensor sss[];
extern const int num_behaviors, num_sensors;

EXTERN struct vehicle vvv, *vvp;

/* Globals of serial port file numbers (Perhaps use a structure) */
EXTERN int RunNumber;
EXTERN int DeadManSwitchEnabled;  /* for aborting mission from RF modem */
EXTERN int NewMissionFileEnabled; /* for reading new mission during run */
EXTERN char MissionFileName[60];  

/*---------------------------------------------------------------------*
 | For each vehicle system, we have four flags: Error, Status, Use, and
 | Monitor.   Error flags refer primarily to initialization, and will 
 | prevent a mission from proceeeding for a critical mission system. 
 | Status flags refer to sensor errors that occur during the main loop.
 |<In general, these won't turn off the device, because in most cases 
 | we could get in more trouble by simply turning the system off instead
 | of hoping the problem was a temporary glitch
 *---------------------------------------------------------------------*/
				/* simulator mode */
extern int MonitorSim;

EXTERN int MonitorDataLogging;
EXTERN int MonitorLayeredControl;

#ifdef EDGEPS8000
extern int LBLTransponderMode;  /* If mode = 0, then in pinger mode */
extern int ErrorLBL, MonitorLBL, UseLBL;
#endif

				/* whoi_usbl */
EXTERN int UseModem, MonitorModem, StatusModem, ErrorModem;

#endif /* ODY_VAR_H */
