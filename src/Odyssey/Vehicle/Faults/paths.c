/*----------------------------------------------------------------------*
   Runtime path handling for the Odyssey simulator.
 *----------------------------------------------------------------------*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "paths.h"


static int has_dir_separator( const char *path )
{
    return ( strchr(path, '/') != NULL ) || ( strchr(path, '\\') != NULL );
}

static void copy_path( char *out, int out_size, const char *in )
{
    strncpy( out, in, out_size - 1 );
    out[out_size - 1] = '\0';
}

void ody_join_path( char *out, int out_size, const char *dir,
		    const char *name )
{
    int len;

    if ( !dir || !dir[0] )
    {
	copy_path( out, out_size, name ? name : "" );
	return;
    }
    if ( !name || !name[0] )
    {
	copy_path( out, out_size, dir );
	return;
    }

    len = strlen( dir );
    if ( dir[len - 1] == '/' || dir[len - 1] == '\\' )
	snprintf( out, out_size, "%s%s", dir, name );
    else
	snprintf( out, out_size, "%s/%s", dir, name );
}

int ody_path_is_explicit( const char *path )
{
    if ( !path || !path[0] )
	return 0;
    if ( has_dir_separator(path) )
	return 1;
    if ( path[0] == '.' )
	return 1;
    if ( strlen(path) >= 2 && path[1] == ':' )
	return 1;
    return 0;
}

int ody_file_exists( const char *path )
{
    FILE *fp;

    if ( !path || !path[0] )
	return 0;
    fp = fopen( path, "r" );
    if ( !fp )
	return 0;
    fclose( fp );
    return 1;
}
