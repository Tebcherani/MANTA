/*----------------------------------------------------------------------*
   Copyright (C) 1994-1997, Massachusetts Institute of Technology.
	 Sea Grant Autonomous Underwater Vehicles Laboratory.
			 All rights reserved.
  $Id: fileman.c,v 2.21 1998/01/20 00:31:51 bamoran Exp $
 *----------------------------------------------------------------------*/

#include <errno.h>
#include <stdio.h>

#include "diagnostic.h"

unsigned InitDataFileSize = 2;	/* default data file size */

/*----------------------------------------------------------------------*
  SET_FILESIZE: preallocate a file to a specific number of megabytes.  If
  it fails due to "media full" error, it iteratively reduces the request by
  1 MB and retries until success.
 *----------------------------------------------------------------------*/

int set_filesize( FILE *fp, unsigned sizeMB )
{
    return 0;
}