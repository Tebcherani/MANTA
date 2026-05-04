/*----------------------------------------------------------------------*
   Copyright (C) 1994-1997, Massachusetts Institute of Technology.
	 Sea Grant Autonomous Underwater Vehicles Laboratory.
			 All rights reserved.
  $Id: fileman.c,v 2.21 1998/01/20 00:31:51 bamoran Exp $
 *----------------------------------------------------------------------*/

#ifdef _OSK
#include <sg_codes.h>
#include <types.h>
#endif

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
#ifdef _OSK
    error_code error;
    path_id path;
    u_int32 sizeB;

    sizeB = 1024*1024*sizeMB;
    path = (path_id) _fileno( fp );

    while ( error = _os_ss_size(path, sizeB) )
    {
        sprintf( msg_buf, "file resize error: %s\n"
                "reducing to %u MB\n", strerror(error), --sizeMB );
	write_message( TERM | SLOG );
        sizeB = 1024*1024*sizeMB;
    }
    if ( error )
	return -1;
#endif /* _OSK */
    return 0;
}


/*----------------------------------------------------------------------*
  TRUNCATE_FILE: shrink preallocated file down to its "true" size.  Flushes
  the buffer and then sets the size of the file to be the position of EOF.
  Error diagnostics as well as the final size of the file are recorded.
 *----------------------------------------------------------------------*/

int truncate_file( FILE *fp )
{
#ifdef _OSK
    error_code error;
    path_id path;
    u_int32 position;

    fflush( fp );
    path = _fileno( fp );

    if ( error = _os_gs_pos( path, &position ) )
        sprintf( msg_buf, "truncate_file: %s\n", strerror( error ) );
    else
    {
        if ( error = _os_ss_size( path, position ) )
            sprintf( msg_buf, "truncate_file: %s\n", strerror( error ) );
        else
            sprintf( msg_buf, "data file truncated to %u bytes\n", position );
    }
    write_message( TERM | SLOG );
    if ( error )
	return -1;
    return position;
#else
    return 0;
#endif /* _OSK */
}
