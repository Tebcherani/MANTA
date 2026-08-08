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
EXTERN char MissionFileName[60];  

EXTERN int MonitorDataLogging;
EXTERN int MonitorLayeredControl;

				/* whoi_usbl */
EXTERN int UseModem, MonitorModem, StatusModem, ErrorModem;

#endif /* ODY_VAR_H */
