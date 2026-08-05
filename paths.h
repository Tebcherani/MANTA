/*----------------------------------------------------------------------*
   Copyright (C) 1994-1997, Massachusetts Institute of Technology.
	 Sea Grant Autonomous Underwater Vehicles Laboratory.
			 All rights reserved.
  $Id: paths.h,v 2.21 1998/01/20 00:31:51 bamoran Exp $
 *----------------------------------------------------------------------*/

#ifndef PATHS_H
#define PATHS_H

#include "ody_name.h"		/* for vehicle identification */

int ody_path_is_explicit( const char *path );
int ody_file_exists( const char *path );
void ody_join_path( char *out, int out_size, const char *dir,
		    const char *name );

#endif
