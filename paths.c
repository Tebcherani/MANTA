/*----------------------------------------------------------------------*
   Runtime path handling for the Odyssey simulator.
 *----------------------------------------------------------------------*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifndef _OSK
#include <unistd.h>
#endif

#include "paths.h"

char ody_root[ODY_PATH_MAX] = ".";
char ody_init_dir[ODY_PATH_MAX] = "./Init";
char ody_data_dir[ODY_PATH_MAX];
char ody_syslog_file[ODY_PATH_MAX];
char ody_mission_dir[ODY_PATH_MAX] = "./SMission";
char ody_sfault_dir[ODY_PATH_MAX] = "./SFault";
char ody_lbl_logfile_name[ODY_PATH_MAX] = "./lbl.log";

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

static void set_root_from_argv0( const char *argv0 )
{
    char cwd[ODY_PATH_MAX];
    char tmp[ODY_PATH_MAX];
    char *slash, *backslash, *sep;

#ifdef _OSK
    copy_path( ody_root, ODY_PATH_MAX, "/dd/usr/auvlab/ody" );
    return;
#else
    if ( getcwd(cwd, sizeof(cwd)) == NULL )
	copy_path( cwd, sizeof(cwd), "." );

    if ( !argv0 || !has_dir_separator(argv0) )
    {
	copy_path( ody_root, ODY_PATH_MAX, cwd );
	return;
    }

    copy_path( tmp, sizeof(tmp), argv0 );
    slash = strrchr( tmp, '/' );
    backslash = strrchr( tmp, '\\' );
    sep = slash > backslash ? slash : backslash;
    if ( sep )
	*sep = '\0';

    if ( tmp[0] == '\0' )
	copy_path( ody_root, ODY_PATH_MAX, cwd );
    else if ( strlen(tmp) >= 2 && tmp[1] == ':' )
	copy_path( ody_root, ODY_PATH_MAX, tmp );
    else if ( tmp[0] == '/' || tmp[0] == '\\' )
	copy_path( ody_root, ODY_PATH_MAX, tmp );
    else
	ody_join_path( ody_root, ODY_PATH_MAX, cwd, tmp );
#endif
}

void ody_init_paths(void)
{
    char *env_root;
    const char *odyssey_log_path, *odyssey_data_dir;

    env_root = getenv("ODY_ROOT");
	copy_path(ody_root, ODY_PATH_MAX, env_root);

    ody_join_path( ody_init_dir, ODY_PATH_MAX, ody_root, "Init" );
    ody_join_path( ody_mission_dir, ODY_PATH_MAX, ody_root, "SMission" );
    ody_join_path( ody_sfault_dir, ODY_PATH_MAX, ody_root, "SFault" );
    ody_join_path( ody_lbl_logfile_name, ODY_PATH_MAX, ody_root, "lbl.log" );

    odyssey_log_path = getenv("ODYSSEY_LOG_PATH");
    copy_path(ody_syslog_file, ODY_PATH_MAX, odyssey_log_path);

    odyssey_data_dir = getenv("ODYSSEY_DATA_DIR");
    copy_path(ody_data_dir, ODY_PATH_MAX, odyssey_data_dir);
}
