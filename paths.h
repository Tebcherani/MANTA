/*----------------------------------------------------------------------*
   Copyright (C) 1994-1997, Massachusetts Institute of Technology.
	 Sea Grant Autonomous Underwater Vehicles Laboratory.
			 All rights reserved.
  $Id: paths.h,v 2.21 1998/01/20 00:31:51 bamoran Exp $
 *----------------------------------------------------------------------*/

#ifndef PATHS_H
#define PATHS_H


/*----------------------------------------------------------------------
  Runtime paths are rooted at the simulator directory by default.  This
  lets sim/ksim be launched from any working directory while preserving
  the traditional DATA_DIR, MISSION_DIR, etc. names used throughout the
  code.  ODY_ROOT in the environment overrides the detected root.
  ----------------------------------------------------------------------*/

#include "ody_name.h"		/* for vehicle identification */

#define ODY_PATH_MAX 512

extern char ody_root[ODY_PATH_MAX];
extern char ody_init_dir[ODY_PATH_MAX];
extern char ody_mission_dir[ODY_PATH_MAX];
extern char ody_lbl_logfile_name[ODY_PATH_MAX];

void ody_init_paths(void);
int ody_path_is_explicit( const char *path );
int ody_file_exists( const char *path );
void ody_join_path( char *out, int out_size, const char *dir,
		    const char *name );

#define ODY_ROOT ody_root
#define INIT_DIR ody_init_dir
#define MISSION_DIR ody_mission_dir
#define LBL_LOGFILE_NAME ody_lbl_logfile_name

#endif
