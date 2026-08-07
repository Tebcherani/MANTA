/*----------------------------------------------------------------------*
   Copyright (C) 1994-1997, Massachusetts Institute of Technology.
	 Sea Grant Autonomous Underwater Vehicles Laboratory.
			 All rights reserved.
  $Id: load_mission.c,v 2.23 1998/02/03 04:25:07 bamoran Exp $
 *----------------------------------------------------------------------*/

#include <errno.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "diagnostic.h"
#include "odyssey_config.h"
#include "odyssey_data.h"
#include "paths.h"
#include "vehicle.h"
#include "odyssey_log.h"
#include "common_defs.h"


				/* prototypes */
int convert_units( const char *, const char *, const double,
			  double * );
				/* externals local to this file */
static char linebuf[MAX_LINE], fullpath[MAX_LINE];
static char loaded_mission_fname[MAX_FILE_NAME] = "";
static char loaded_mission_path[MAX_LINE] = "";

const char *get_mission_file_name( void )
{
    return loaded_mission_fname;
}

const char *get_mission_file_path( void )
{
    return loaded_mission_path;
}


/*-------------------------------------------------------------------------
  LOAD_MISSION: new function that reads a combined init/mission file with
  the init portion at the top and the mission at the bottom.  Assumes the
  file is of the init variety until it encounters "state:" whereupon it
  switches modes to the standard load_state type of reading.

  The motivation for this mod is the desire to have the process of
  configuring the vehicle be compressed into a single file in prep for web
  based interfaces.

  B. A. Moran, 29-May-1997
  -----------------------------------------------------------------------*/

int load_mission( const char *fname )
{
				/* prototypes */
    int read_flags( FILE *fptr );
    int read_state( FILE *fptr );
				/* locals */
    FILE *fptr;
    int error = -1;

				/* construct full path to mission file */
    if ( ody_path_is_explicit(fname) || ody_file_exists(fname) )
	strcpy( fullpath, fname );
    else
	ody_join_path( fullpath, sizeof(fullpath), MISSION_DIR, fname );
    strncpy( loaded_mission_fname, fname, sizeof(loaded_mission_fname) - 1 );
    loaded_mission_fname[sizeof(loaded_mission_fname) - 1] = '\0';
    strncpy( loaded_mission_path, fullpath, sizeof(loaded_mission_path) - 1 );
    loaded_mission_path[sizeof(loaded_mission_path) - 1] = '\0';
    if ( (fptr = fopen(fullpath, "r")) == NULL )
    {
	sprintf( msg_buf, "%s: %s\n", fullpath, strerror(errno) );
	write_message( TERM | SLOG );
	return error;
    }
    sprintf( msg_buf, "mission file: %s ...\n", fullpath );
    write_message( TERM | SLOG );

				/* read flags then state (flags error
				   prevents reading state) */
    if ( !read_flags(fptr) && !read_state(fptr) )
	error = 0;
    else
    {
	sprintf( msg_buf, "%s: file error ...\n", fname );
	write_message( TERM | SLOG );
    }
				/* close file, return error value */
    fclose( fptr );
    return error;
}

/*-------------------------------------------------------------------------
  LOAD_VEHICLE_DEF: load vehicle physical parameters (vs_*) from a
  vehicle definition (.vd) file.  Call BEFORE load_mission().

  File format mirrors mission-file sensor lines:

      # comments start with # % or *
      vehicle: <name>            <- must match VEHICLE_NAME or load fails
      vs_<name>(<units>) <value>
      vs_<name>(<units>) <value>
      ...

  Rules:
   - Header 'vehicle:' must match this build's VEHICLE_NAME.
   - Names must begin with 'vs_' (catches typos that would otherwise
     silently clobber an m_/c_/u_ sensor).
   - Unknown vs_* name -> hard error (catches stale references).
   - vs_* not mentioned in the file -> keeps its compile-time default
     from edit_struct.c (so old .vd files keep working as the data
     structure grows).
  -----------------------------------------------------------------------*/

int load_vehicle_def( const char *fname )
{
    FILE *fptr;
    char line[MAX_LINE], name[NAME_SIZE], header[NAME_SIZE];
    char *expect, *actual;
    double value;
    int index, lineno = 0, n_set = 0, header_seen = 0;

    if ( ody_path_is_explicit(fname) || ody_file_exists(fname) )
	strcpy( fullpath, fname );
    else
	ody_join_path( fullpath, sizeof(fullpath), VEHICLE_DIR, fname );
    if ( (fptr = fopen(fullpath, "r")) == NULL ) {
        sprintf( msg_buf, "%s: %s\n", fullpath, strerror(errno) );
        write_message( TERM | SLOG );
        return -1;
    }
    sprintf( msg_buf, "vehicle definition: %s ...\n", fullpath );
    write_message( TERM | SLOG );

    while ( fgets(line, sizeof(line), fptr) != NULL ) {
        lineno++;
				/* skip blank lines and comments */
        if ( line[0] == '\n' || line[0] == '\r' ||
             line[0] == '#'  || line[0] == '%' || line[0] == '*' )
            continue;

				/* first real line must be the vehicle header */
        if ( !header_seen ) {
            if ( sscanf(line, " vehicle: %s", header) != 1 ) {
                sprintf( msg_buf, "%s: missing 'vehicle:' header at "
                         "line %d\n", fname, lineno );
                write_message( TERM | SLOG );
                fclose(fptr);
                return -1;
            }
            if ( strcmp(header, VEHICLE_NAME) != 0 ) {
                sprintf( msg_buf, "%s: definition is for '%s', this "
                         "build is '%s'\n", fname, header, VEHICLE_NAME );
                write_message( TERM | SLOG );
                fclose(fptr);
                return -1;
            }
            header_seen = 1;
            continue;
        }

				/* parse "<name> <value>" */
        if ( sscanf(line, " %s %lg", name, &value) != 2 ) {
            sprintf( msg_buf, "%s line %d: cannot parse: %s",
                     fname, lineno, line );
            write_message( TERM | SLOG );
            fclose(fptr);
            return -1;
        }

				/* only vs_* fields allowed */
        if ( strncmp(name, "vs_", 3) != 0 ) {
            sprintf( msg_buf, "%s line %d: '%s' is not a vs_* parameter\n",
                     fname, lineno, name );
            write_message( TERM | SLOG );
            fclose(fptr);
            return -1;
        }

				/* look up by name */
        if ( (index = match_sensor(name)) < 0 ) {
            sprintf( msg_buf, "%s line %d: unknown parameter '%s'\n",
                     fname, lineno, name );
            write_message( TERM | SLOG );
            fclose(fptr);
            return -1;
        }

				/* convert units if given */
        expect = strchr( vvp->s[index].name, '(' );
        actual = strchr( name, '(' );
        if ( expect && actual &&
             convert_units( expect, actual, value, &value ) ) {
            sprintf( msg_buf, "%s line %d: unknown units in '%s'\n",
                     fname, lineno, name );
            write_message( TERM | SLOG );
            fclose(fptr);
            return -1;
        }

        update( index, value );
        n_set++;
    }

    fclose( fptr );

    if ( !header_seen ) {
        sprintf( msg_buf, "%s: empty vehicle definition file\n", fname );
        write_message( TERM | SLOG );
        return -1;
    }
    sprintf( msg_buf, "vehicle definition: %d parameters set\n", n_set );
    write_message( TERM | SLOG );
    return 0;
}

/*-------------------------------------------------------------------------
  READ_FLAGS: this is a pretty good copy of load_init from earlier
  revisions, but we now explicitly look for "state:"
  -----------------------------------------------------------------------*/

int read_flags( FILE *fptr )
{
    int value;
    double dvalue;

    while ( !skip_comments(fptr) )
    {
        if ( fgets(linebuf, sizeof(linebuf), fptr) == NULL )
            return -1;
				/* new for merged init/mission files: if
				   "state:" or "behavior:" is present in
				   line, then rewind file by strlen this
				   line and bail. this is the correct
				   behavior when using layered control */
	if ( strstr(linebuf, "state:") || strstr(linebuf, "behavior:") ||
	     strstr(linebuf, "sensor:") )
	{
	    fseek( fptr, -strlen(linebuf), SEEK_CUR );
	    return 0;
	}
				/* parse the init file to look for various
				   codes and take appropriate actions */
	if ( sscanf(linebuf, "UseSim %d", &value) == 1 )
	{
	    fprintf(stderr, "Ignoring obsolete UseSim setting: %d\n", value);
		fprintf(syslog, "Ignoring obsolete UseSim setting: %d\n", value);
    	continue;
	}
	if ( (sscanf(linebuf, "UseAltimeter %d", &value))==1) {
	    fprintf(stderr, "Ignoring obsolete UseAltimeter setting: %d\n", value);
		fprintf(syslog, "Ignoring obsolete UseAltimeter setting: %d\n", value);
    	continue;
	}
	if ( (sscanf(linebuf, "UseSail1 %d", &value))==1) {
	    fprintf(stderr, "Ignoring obsolete UseSail1 setting: %d\n", value);
		fprintf(syslog, "Ignoring obsolete UseSail1 setting: %d\n", value);
    	continue;
	}
	if ( (sscanf(linebuf, "UseWatchdog %d", &value))==1) {
	    fprintf(stderr, "Ignoring obsolete UseWatchdog setting: %d\n", value);
		fprintf(syslog, "Ignoring obsolete UseWatchdog setting: %d\n", value);
    	continue;
	}
	if ( (sscanf(linebuf, "UseThruster %d", &value))==1) {
	    fprintf(stderr, "Ignoring obsolete UseThruster setting: %d\n", value);
		fprintf(syslog, "Ignoring obsolete UseThruster setting: %d\n", value);
    	continue;
	}
	if ( sscanf(linebuf, "FinHECalibrate %d", &value) == 1 )
	{
	    fprintf(stderr, "Ignoring obsolete FinHECalibrate setting: %d\n", value);
		fprintf(syslog, "Ignoring obsolete FinHECalibrate setting: %d\n", value);
    	continue;
	}
	if ( (sscanf(linebuf, "HallEffectIndex %d", &value))==1) {
	    fprintf(stderr, "Ignoring obsolete HallEffectIndex setting: %d\n", value);
		fprintf(syslog, "Ignoring obsolete HallEffectIndex setting: %d\n", value);
    	continue;
	}
	if ( (sscanf(linebuf, "UseElevator %d", &value))==1) {
	    fprintf(stderr, "Ignoring obsolete UseElevator setting: %d\n", value);
		fprintf(syslog, "Ignoring obsolete UseElevator setting: %d\n", value);
    	continue;
	}
	if ( (sscanf(linebuf, "UseRudder %d", &value))==1) {
	    fprintf(stderr, "Ignoring obsolete UseRudder setting: %d\n", value);
		fprintf(syslog, "Ignoring obsolete UseRudder setting: %d\n", value);
    	continue;
	}
	if ( (sscanf(linebuf, "UseParosci %d", &value))==1) {
	    fprintf(stderr, "Ignoring obsolete UseParosci setting: %d\n", value);
		fprintf(syslog, "Ignoring obsolete UseParosci setting: %d\n", value);
    	continue;
	}
	if ( (sscanf(linebuf, "UseKVH_DGC %d", &value))==1) {
	    fprintf(stderr, "Ignoring obsolete UseKVH_DGC setting: %d\n", value);
		fprintf(syslog, "Ignoring obsolete UseKVH_DGC setting: %d\n", value);
    	continue;
	}
	if ( (sscanf(linebuf, "UseKVH_DGI %d", &value))==1) {
	    fprintf(stderr, "Ignoring obsolete UseKVH_DGI setting: %d\n", value);
		fprintf(syslog, "Ignoring obsolete UseKVH_DGI setting: %d\n", value);
    	continue;
	}
	if ( (sscanf(linebuf, "MonitorKVH_DGC %d", &value))==1) {
	    fprintf(stderr, "Ignoring obsolete MonitorKVH_DGC setting: %d\n", value);
		fprintf(syslog, "Ignoring obsolete MonitorKVH_DGC setting: %d\n", value);
    	continue;
	}
	if ( (sscanf(linebuf, "MonitorKVH_DGI %d", &value))==1) {
	    fprintf(stderr, "Ignoring obsolete MonitorKVH_DGI setting: %d\n", value);
		fprintf(syslog, "Ignoring obsolete MonitorKVH_DGI setting: %d\n", value);
    	continue;
	}
	if ( (sscanf(linebuf, "MonitorAltimeter %d", &value))==1) {
	    fprintf(stderr, "Ignoring obsolete MonitorAltimeter setting: %d\n", value);
		fprintf(syslog, "Ignoring obsolete MonitorAltimeter setting: %d\n", value);
    	continue;
	}
#ifdef EDGEPS8000
	if ( (sscanf(linebuf, "UseLBL %d", &value))==1) {
	    UseLBL = value;
	    fprintf(stderr, "UseLBL   %d\n", UseLBL);
	    fprintf(syslog, "UseLBL   %d\n", UseLBL);
	    continue;
	}
	if ( (sscanf(linebuf, "MonitorLBL %d", &value))==1) {
	    MonitorLBL = value;
	    fprintf(stderr, "MonitorLBL   %d\n", MonitorLBL);
	    fprintf(syslog, "MonitorLBL   %d\n", MonitorLBL);
	    continue;
	}
	if ( (sscanf(linebuf, "LBLTransponderMode %d", &value))==1) {
	    LBLTransponderMode = value;
	    fprintf(stderr, "LBLTransponderMode %d\n", LBLTransponderMode);
	    fprintf(syslog, "LBLTransponderMode %d\n", LBLTransponderMode);
	    continue;
	}
#endif

	if ( (sscanf(linebuf, "UseCondTemp %d", &value))==1)
	{
	    fprintf(stderr, "Ignoring obsolete UseCondTemp setting: %d\n", value);
		fprintf(syslog, "Ignoring obsolete UseCondTemp setting: %d\n", value);
    	continue;
	}
	if ( (sscanf(linebuf, "MonitorCondTemp %d", &value))==1) {
	    fprintf(stderr, "Ignoring obsolete MonitorCondTemp setting: %d\n", value);
		fprintf(syslog, "Ignoring obsolete MonitorCondTemp setting: %d\n", value);
    	continue;
	}

	if ( (sscanf(linebuf, "MonitorLayeredControl  %d", &value))==1) {
	    MonitorLayeredControl = value;
	    fprintf(stderr, "MonitorLayeredControl    %d\n", 
		    MonitorLayeredControl);
	    fprintf(syslog, "MonitorLayeredControl    %d\n", 
		    MonitorLayeredControl);
	    continue;
	}
	if ( (sscanf(linebuf, "MonitorDataLogging  %d", &value))==1) {
	    MonitorDataLogging = value;
	    fprintf(stderr, "MonitorDataLogging    %d\n", 
		    MonitorDataLogging);
	    fprintf(syslog, "MonitorDataLogging    %d\n", 
		    MonitorDataLogging);
	    continue;
	}
	if ( (sscanf(linebuf, "MonitorWatchdog %d", &value))==1) {
	    fprintf(stderr, "Ignoring obsolete MonitorWatchdog setting: %d\n", value);
		fprintf(syslog, "Ignoring obsolete MonitorWatchdog setting: %d\n", value);
    	continue;
	}
	if ( (sscanf(linebuf, "MonitorThruster %d", &value))==1) {
	    fprintf(stderr, "Ignoring obsolete MonitorThruster setting: %d\n", value);
		fprintf(syslog, "Ignoring obsolete MonitorThruster setting: %d\n", value);
    	continue;
	}
	if ( (sscanf(linebuf, "MonitorElevator %d", &value))==1) {
	    fprintf(stderr, "Ignoring obsolete MonitorElevator setting: %d\n", value);
		fprintf(syslog, "Ignoring obsolete MonitorElevator setting: %d\n", value);
    	continue;
	}
	if ( (sscanf(linebuf, "MonitorRudder %d", &value))==1) {
	    fprintf(stderr, "Ignoring obsolete MonitorRudder setting: %d\n", value);
		fprintf(syslog, "Ignoring obsolete MonitorRudder setting: %d\n", value);
    	continue;
	}
	if ( (sscanf(linebuf, "MonitorParosci %d", &value))==1) {
	    fprintf(stderr, "Ignoring obsolete MonitorParosci setting: %d\n", value);
		fprintf(syslog, "Ignoring obsolete MonitorParosci setting: %d\n", value);
    	continue;
	}
	if ( (sscanf(linebuf, "Countdown %d", &value))==1) {
	    fprintf(stderr, "Ignoring obsolete Countdown setting: %d\n", value);
		fprintf(syslog, "Ignoring obsolete Countdown setting: %d\n", value);
    	continue;
	}
	if ( (sscanf(linebuf, "CommandedTime %d", &value))==1) {
	    fprintf(stderr, "Ignoring obsolete CommandedTime setting: %d\n", value);
		fprintf(syslog, "Ignoring obsolete CommandedTime setting: %d\n", value);
    	continue;
	}
	if ( (sscanf(linebuf, "CommandedSpeed %lf", &dvalue))==1) {
	    fprintf(stderr, "Ignoring obsolete CommandedSpeed setting: %d\n", value);
		fprintf(syslog, "Ignoring obsolete CommandedSpeed setting: %d\n", value);
    	continue;
	}
	if ( (sscanf(linebuf, "CommandedHeading %lf", &dvalue))==1) {
	    fprintf(stderr, "Ignoring obsolete CommandedHeading setting: %d\n", value);
		fprintf(syslog, "Ignoring obsolete CommandedHeading setting: %d\n", value);
    	continue;
	}
	if ( (sscanf(linebuf, "CommandedHeadingDegrees %lf", &dvalue))==1) {
	    fprintf(stderr, "Ignoring obsolete CommandedHeadingDegrees setting: %d\n", value);
		fprintf(syslog, "Ignoring obsolete CommandedHeadingDegrees setting: %d\n", value);
    	continue;
	}
	if ( (sscanf(linebuf, "CommandedDepth %lf", &dvalue))==1) {
	    fprintf(stderr, "Ignoring obsolete CommandedDepth setting: %d\n", value);
		fprintf(syslog, "Ignoring obsolete CommandedDepth setting: %d\n", value);
    	continue;
	}
	if ( (sscanf(linebuf, "DebugMode %d", &value))==1) {
	    fprintf(stderr, "Ignoring obsolete DebugMode setting: %d\n", value);
		fprintf(syslog, "Ignoring obsolete DebugMode setting: %d\n", value);
    	continue;
	}
	if ( (sscanf(linebuf, "CommsDebugMode %d", &value))==1) {
	    fprintf(stderr, "Ignoring obsolete CommsDebugMode setting: %d\n", value);
		fprintf(syslog, "Ignoring obsolete CommsDebugMode setting: %d\n", value);
    	continue;
	}
	if ( (sscanf(linebuf, "TimingDebugMode %d", &value))==1) {
	    fprintf(stderr, "Ignoring obsolete TimingDebugMode setting: %d\n", value);
		fprintf(syslog, "Ignoring obsolete TimingDebugMode setting: %d\n", value);
    	continue;
	}
	if ( (sscanf(linebuf, "WatchdogTimeoutMinutes %d", &value))==1) {
	    fprintf(stderr, "Ignoring obsolete WatchdogTimeoutMinutes setting: %d\n", value);
		fprintf(syslog, "Ignoring obsolete WatchdogTimeoutMinutes setting: %d\n", value);
    	continue;
	}
	if ( (sscanf(linebuf, "WatchdogTimeoutHours %d", &value))==1) {
	    fprintf(stderr, "Ignoring obsolete WatchdogTimeoutHours setting: %d\n", value);
		fprintf(syslog, "Ignoring obsolete WatchdogTimeoutHours setting: %d\n", value);
    	continue;
	}
	if (sscanf(linebuf, "InitDataFileSize %d", &value) == 1) {
    	fprintf(stderr, "Ignoring obsolete InitDataFileSize setting: %d\n", value);
		fprintf(syslog, "Ignoring obsolete InitDataFileSize setting: %d\n", value);
    	continue;
	}
	if ( (sscanf(linebuf, "UseKalmanFilter %d", &value))==1) {
	    fprintf(stderr, "Ignoring obsolete UseKalmanFilter setting: %d\n", value);
		fprintf(syslog, "Ignoring obsolete UseKalmanFilter setting: %d\n", value);
    	continue;
	}
	if ( (sscanf(linebuf, "MonitorSim %d", &value))==1)
	{
	    MonitorSim = value;
	    fprintf(stderr, "MonitorSim  %d\n", MonitorSim);
	    fprintf(syslog, "MonitorSim  %d\n", MonitorSim);
	    continue;
	}
	if ( (sscanf(linebuf, "AltimeterFrequency %d", &value))==1)
	{
	    fprintf(stderr, "Ignoring obsolete AltimeterFrequency setting: %d\n", value);
		fprintf(syslog, "Ignoring obsolete AltimeterFrequency setting: %d\n", value);
    	continue;
	}
	if ( (sscanf(linebuf, "UseOld200 %d", &value))==1)
	{
	    fprintf(stderr, "Ignoring obsolete UseOld200 setting: %d\n", value);
		fprintf(syslog, "Ignoring obsolete UseOld200 setting: %d\n", value);
    	continue;
	}
	if ( (sscanf(linebuf, "DeadManSwitchEnabled %d", 
		     &value))==1)
	{
	    DeadManSwitchEnabled = value;
	    fprintf(stderr, "DeadManSwitchEnabled  %d\n", 
		    DeadManSwitchEnabled);
	    fprintf(syslog, "DeadManSwitchEnabled  %d\n", 
		    DeadManSwitchEnabled);
	    continue;
	}
	if ( (sscanf(linebuf, "NewMissionFileEnabled %d", 
		     &value))==1)
	{
	    NewMissionFileEnabled = value;
	    fprintf(stderr, "NewMissionFileEnabled  %d\n", 
		    NewMissionFileEnabled);
	    fprintf(syslog, "NewMissionFileEnabled  %d\n", 
		    NewMissionFileEnabled);
	    continue;
	}
	if ( (sscanf(linebuf, "AltimeterPortNumber %d", &value))==1)
	{
	    fprintf(stderr, "Ignoring obsolete AltimeterPortNumber setting: %d\n", value);
		fprintf(syslog, "Ignoring obsolete AltimeterPortNumber setting: %d\n", value);
    	continue;
	}
	if ( (sscanf(linebuf, "UsePort %d", &value))==1)
	{
	    fprintf(stderr, "Ignoring obsolete UsePort setting: %d\n", value);
		fprintf(syslog, "Ignoring obsolete UsePort setting: %d\n", value);
    	continue;
	}
	if ( (sscanf(linebuf, "UseStarboard %d", &value))==1)
	{
	    fprintf(stderr, "Ignoring obsolete UseStarboard setting: %d\n", value);
		fprintf(syslog, "Ignoring obsolete UseStarboard setting: %d\n", value);
    	continue;
	}

	if ( sscanf(linebuf, "UseModem %d", &value) == 1 )
	{
	    UseModem = value;
	    sprintf( msg_buf, "UseModem %d\n", value );
	    write_message( TERM | SLOG );
	    continue;
	}
	if ( sscanf(linebuf, "MonitorModem %d", &value) == 1 )
	{
	    MonitorModem = value;
	    sprintf( msg_buf, "MonitorModem   %d\n", value );
	    write_message( TERM | SLOG );
	    continue;
	}

#if defined WHOI_LATCH
	if ( sscanf(linebuf, "UseLatchMotor %d", &value) == 1 )
	{
	    fprintf(stderr, "Ignoring obsolete UseLatchMotor setting: %d\n", value);
		fprintf(syslog, "Ignoring obsolete UseLatchMotor setting: %d\n", value);
    	continue;
	}
	if ( sscanf(linebuf, "MonitorLatchMotor %d", &value) == 1 )
	{
	    fprintf(stderr, "Ignoring obsolete MonitorLatchMotor setting: %d\n", value);
		fprintf(syslog, "Ignoring obsolete MonitorLatchMotor setting: %d\n", value);
    	continue;
	}
#endif /* WHOI_LATCH */

	if ( strlen(linebuf) > 1 )
	{
	    fprintf( stderr, 
		     "read flags %s: no match, exiting ...\n", linebuf );
	    exit_gracefully( -1 );
	}
    }
    return 0;
}


/*-------------------------------------------------------------------------
  READ_STATE: this is the old guts of load_state, with the new load_state
  calling this function immediately after opening the file.
  -----------------------------------------------------------------------*/

static int count;		/* needed for load_state to return line
				   count because counting occurs here */

int read_state( FILE *fptr )
{
    char name[MAX_STATE_NAME], *expect, *actual;
    int priority = 0, index, b_index = -1;
    double value;
				/* Mar 1 94, reset all the behavior slots
                                   in the command structure to -1 before
                                   reading in new behaviors */

    while ( fgets(linebuf, sizeof(linebuf), fptr) != NULL )
    {
        count++;
        if (sscanf(linebuf, " state: %s\n", &name) == 1)
	{
            strncpy( vvp->sta[vvp->nstates].name, name, MAX_STATE_NAME );
            vvp->nstates++;
            if ( vvp->nstates >= MAX_STATES )
	    {
                sprintf( msg_buf, "*** max number of states exceeded\n");
                write_message( TERM | SLOG );
                exit_gracefully( -1 );
            }
            if (vvp->nstates==1)
		vvp->cstate = 0;
            else
		vvp->cstate++;

            sprintf( msg_buf, "\n *** state %d ***\n%s\n", vvp->nstates, 
		     linebuf );
            write_message( TERM | SLOG );
				/* For SCLC, when you see the word state,
                                   increment the state_counter and write
                                   subsequent behaviors into the next
                                   command array */
        }
				/* If sensor value, load it... */
        else if ( sscanf(linebuf, "  sensor: %s %lg", &name, &value) == 2 )
	{
            if ( (index = match_sensor(name)) >= 0 )
	    {
				/* convert units if given */
		expect = strchr( vvp->s[index].name, '(' );
		actual = strchr( name, '(');
		if ( expect && actual &&
		     convert_units( expect, actual, value, &value ) )
		{
		    sprintf( msg_buf, "load_mission %s: unknown "
			     "units, exiting ...\n", name );
		    write_message( TERM | SLOG );
		    exit_gracefully( -1 );
		}
                update( index, value );
		
                sprintf( msg_buf, "  sensor: %29s %lg\n", 
			 vvp->s[index].name, vvp->s[index].x );
                write_message( TERM | SLOG );
            } else {
                sprintf( msg_buf, 
                        "failed to match sensor %s, exiting\n", name );
                write_message( TERM | SLOG );
                exit_gracefully(-1);
            }
        }
				/* If a behavior name, get priority */

        else if (sscanf(linebuf, " behavior: %s", &name) == 1 )
	{
	    priority++;
            if ( priority >= MAX_COMMAND )
	    {
                sprintf( msg_buf, "load_mission: priority %d >= "
			 "MAX_COMMAND (%d), exiting ...\n", priority,
			 MAX_COMMAND );
                write_message( TERM | SLOG );
                exit_gracefully( -1 );
            }

            if ( (b_index = match_behavior(name)) >= 0 )
	    {
                vvp->sta[vvp->cstate].c[priority].behavior = b_index;

                sprintf( msg_buf, " behavior: %s %d \n",
			 vvp->b[b_index].name, priority );
                write_message( TERM | SLOG );

				/* copy default arguments from bTemplate
				   into current state (BAM, 2/29/96) */

                memcpy( &vvp->sta[vvp->cstate].c[priority].b,
                        &bTemplate[b_index], sizeof(struct behavior) );

                assign_behavior_fcn_ptr( name,
		    &(vvp->sta[vvp->cstate].c[priority].b) );

            } else {
                sprintf( msg_buf, "failed to match behavior %s, "
			 "exiting ...\n", name );
                write_message( TERM | SLOG );
                exit_gracefully(-1);
            }
				/* If line is a behavior argument, then
                                   load it */
        }
	else if (sscanf(linebuf, "  b_arg: %s %lg", &name, &value) == 2 )
	{
            if ( b_index >= 0 )
                if ( (index = match_behavior_arg(name, b_index)) >= 0 )
		{
                    expect = strchr( vvp->b[b_index].arg[index].name, '(' );
                    actual = strchr( name, '(' );

                    if ( expect && actual &&
			 convert_units( expect, actual, value, &value ) )
                    {
                        sprintf( msg_buf, 
				 "load_mission: error in b_arg units "
				 "%s, exiting ...\n", name );
			write_message( TERM | SLOG );
                        exit_gracefully( -1 );
                    }
                    vvp->b[b_index].arg[index].x =
                        vvp->sta[vvp->cstate].c[priority].b.arg[index].x = value;

                    strcpy( vvp->sta[vvp->cstate].c[priority].b.arg[index].name, 
                           vvp->b[b_index].arg[index].name );
                } else {
                    sprintf( msg_buf, 
			     "can't match behavior argument %s\n", linebuf );
                    write_message( TERM | SLOG );
                    exit_gracefully( -1 );
                }
            sprintf( msg_buf, "  b_arg: %30s %lg\n",
		     vvp->b[b_index].arg[index].name,
		     vvp->b[b_index].arg[index].x);
            write_message( TERM | SLOG );
        }
    }
				/* good place to set position estimators to
				   agree with the master vehicle position */
    update( M_DR_NORTH, sensor(M_POS_N) );
    update( M_DR_EAST,  sensor(M_POS_E) );

    return 0;
}


/*-----------------------------------------------------------------------*
  Find the sensor with the given name, return -1 if none found...
  (JGB, 1/30/94)
 *-----------------------------------------------------------------------*/

static char vvvName[NAME_SIZE], thisName[NAME_SIZE];

int match_sensor( const char *name )
{
    int i;
    char *eor;

    strcpy( thisName, name );
				/* terminate name where units begin */
    if ( (eor = strchr(thisName, '(')) ) *eor = '\0';

    for ( i = 0; i < MAX_SENSOR; i++ )
    {
	strcpy( vvvName, sss[i].name );
	if ( (eor = strchr(vvvName, '(')) ) *eor = '\0';
	if ( strcmp(vvvName, thisName) == 0 ) return i;
    }
    return -1;
}

/* -----------------------------------------------------------------------
   |Find the behavior with the given name...return -1 if none found...
   |(JGB, 1/30/94)
   -----------------------------------------------------------------------*/

int match_behavior( const char *name )
{
    int i = 0;

    for ( i = 0; i < MAX_BEHAVIOR; i++ )
        if ( strcmp(vvp->b[i].name, name) == 0 ) 
            return i;
    
    return -1;
}

int match_behavior_arg( const char *name, int b_index )
{
    int i;
    char *loc;

    strcpy( thisName, name );
    if ( (loc = strchr(thisName, '(')) != NULL )
	*loc = '\0';

    for ( i = 0; i < B_ARG_SIZE; i++ )
    {
	strcpy( vvvName, vvp->b[b_index].arg[i].name );
	if ( (loc = strchr(vvvName, '(')) != NULL )
	    *loc = '\0';

        if ( strcmp(vvvName, thisName) == 0 )
            return i;
    }
    return -1;
}

/*----------------------------------------------------------------------*
  CONVERT_UNITS: compare expected and actual units, convert if they
  differ.  Upon successful conversion, return 0.  Upon unknown
  conversion, return -1, recommended to exit_gracefully when this occurs.

  B. Moran, 18 jan 95
 *----------------------------------------------------------------------*/

int convert_units( const char *expected, const char *given,
			  const double inval, double *outval )
{
    int error = 0;
                                /* if they differ, then check for cases */
    if ( strcmp(expected, given) != 0 )
    {
        if ( !strncmp( expected, "(rad", 4 ) &&
             !strncmp( given, "(deg", 4 ) )
            *outval = D2R(inval);
        else if ( !strncmp( expected, "(deg", 4 ) &&
                  !strncmp( given, "(rad", 4 ) )
            *outval = R2D(inval);
	else if ( !strncmp(expected, "(m", 2) &&
		  !strncmp(given, "(km", 3) )
	    *outval = 1000*inval;
	else if ( !strncmp(expected, "(km", 3) &&
		  !strncmp(given, "(m", 2) )
	    *outval = 0.001*inval;
        else
            error = -1;		/* differ, but unknown conversion */
    }
    else                        /* otherwise just transfer directly */
        *outval = inval;

    return error;
}


/*--------------------------------------------------------------------*
  routine to skip a line of input if it begins with a *, #, or % 
  returns EOF if encountered, else returns 0; 
 *--------------------------------------------------------------------*/

int skip_comments( FILE *file )
{
    char line[MAX_LINE];
    int c;
				/* test first character */
    c = fgetc(file);
    if ( c == EOF )
	return c;
				/* if not comment, restore file */
    if ( !strchr("%#*", c) )
    {
	ungetc(c, file);
	return 0;
    }
				/* eat rest of line */
    if ( fgets(line, sizeof(line), file) == NULL )
    {
	fprintf(stderr, "skip_comments() eof encountered\n");
	return EOF;
    }
    fprintf(syslog, "%s\n", line);
				/* recursive call for next comment */
    return skip_comments(file);
}
