/*----------------------------------------------------------------------*
   Copyright (C) 1994-1997, Massachusetts Institute of Technology.
	 Sea Grant Autonomous Underwater Vehicles Laboratory.
			 All rights reserved.
  $Id: paths.h,v 2.21 1998/01/20 00:31:51 bamoran Exp $
 *----------------------------------------------------------------------*/

#ifndef PATHS_H
#define PATHS_H


/*----------------------------------------------------------------------
  If the ODY file system maintains its relative structure, then the only
  change needed is to ODY_ROOT.  The ANSI-C standard specifies that the
  preprocessor concatenates quoted strings when separated by whitespace.
  The following code relies on this feature.
  ----------------------------------------------------------------------*/

#include "ody_name.h"		/* for vehicle identification */

#ifdef _OSK
#define ODY_ROOT "/dd/usr/auvlab/ody"
#else
#define ODY_ROOT "."
#endif

#define INIT_DIR     ODY_ROOT "/Init"
#define DATA_DIR     ODY_ROOT "/SData"
#define SYSLOG_FILE  ODY_ROOT "/system.log"
#define MISSION_DIR  ODY_ROOT "/SMission"
#define VEHICLE_DIR  ODY_ROOT "/Vehicle"
#define LBL_LOGFILE_NAME ODY_ROOT "/lbl.log"

#endif
