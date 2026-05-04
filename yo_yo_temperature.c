/*-----------------------------------------------------------------------*
  Copyright (C) 1994-1998, Massachusetts Institute of Technology.
  Proprietary to Sea Grant AUV Laboratory.  All rights reserved.
  $Id: layered_control.c,v 2.30 1998/08/05 00:01:55 bamoran Exp $
 *-----------------------------------------------------------------------*/
/* -------------------------------------------------------------------------- 
|  YO_YO_Temperature:                                                                  
|   This behavior causes the vehicle to move up and down between two
|   specified temperature extremes (bottom_temp and top_temp).  Maximum
|   and minimum depths, as well as a minimum altitude, provide bounds on
|   vehicle depths.  Every so many cycles, the behavior triggers a yo_yo
|   cycles which is purely depth triggered.
|
|   All vertical plane control is depth control.  This behavior does not 
|   control either heading or speed, so some other behavior must handle
|   horizontal plane motion.
|
|   Termination occurs when either the vehicle has completed the requested
|   number of yo-yo cyles, or the behavior has run for max_time, which ever
|   comes first.  Note:  one complete cycle consists of both an up and down
|   motion.
|
|   If one wishes to disable the bottom triggered termination of a down leg
|   (say, because the altimeter is not working properly) then set the
|   variable min_altitude to a negative value.  
|  
|  Arguments are:
|    0    not an input....
|    1    "start_time(s)", 0.000000, 0, 0, 
|    2    "top_depth(m)", 0.000000, 0, 0, 
|    3    "bottom_depth(m)", 0.000000, 0, 0, 
|    4    "top_temperature(deg C)", 0.000000, 0, 0,
|    5    "bottom_temperature(deg C)", 0.000000, 0, 0,
|    6    "min_altitude(m)", 0.000000, 0, 0, 
|    7    "cycles(int)", 0.000000, 0, 0, 
|    8    "cycles_temp(int)", 0.000000, 0, 0,
|    9    "max_time(s)", 0.000000, 0, 0, 
|
|   There are a few arguments worth explaining.  For example, this behavior
|   assumes the top temperature is GREATER than the bottom temperature.  It
|   will not work for a temperature inversion (its too simple).  Also, the
|   argument "cycles_temp" is the number of cycles of temperature
|   triggered profiles between each depth triggered profile.
|
|   This is built from the original yo_yo behavior written 7/11/94.
|
|  J. G. Bellingham, 9/21/98
  -------------------------------------------------------------------------- */

int yo_yo( struct behavior *b, int priority, int level )
{
    int do_temp;
    enum { C_DPTH, START_TIME, T_DPTH, B_DPTH, T_TEMP, B_TEMP, MIN_ALT, 
	CYCLES, CYCLES_TEMP, TIMEOUT };

    /* If start time not reached yet, exit the behavior */
    if (sensor(m_present_time) < bArg(START_TIME))
	return 0;  /* CHECK THIS RETURN VALUE... */

    /* if behavior has timed out, return 1 */
    if ( manage_time(b, bArg(TIMEOUT)) == 1 )
        return 1;

    /* Make sure to initialize commanded depth first time through... */    
    if (bArg(C_DEPTH) != bArg(T_DEPTH)) && (bArg(C_DEPTH) != bArg(B_DEPTH))
        bArg(C_DEPTH) = bArg(B_DEPTH);

  /* jleonard 6/21/96 changing so that if number of cycles elapses, then we
     will set the modes to -1 to cause shut down of the vehicle unless
     another behavior up the stack takes control.  This means that you
     cannot do a long setpoint with a single cycle yoyo at the start with
     the yoyo then dropping out and the setpoint resuming control.  But I
     think this way is safer because of our altimeter problems
     (i.e. usually, we set # cycles to a very large value, and if we
     execute that many cycles, something has gone seriously wrong and we
     should end the mission. */

    /* If total # of cycles elapsed, try to end mission */
    if ( bArg(CYCLES) < 0.25 )
    {
        shut_down( priority );
        sprintf( msg_buf, "% 8.2f: %s shuts down on cycle count\n",
                 sensor(M_PRESENT_TIME), b->name );
        write_message( MonitorLayeredControl | SLOG );
        return 1;
    }

    /* Is this a nominally temperature triggered cycle? */
    do_temp = 1;
    if (bArg(CYCLES_TEMP) > 0)
    	if (fmod(floor(bArg(CYCLES)+0.55), bArg(CYCLES_TEMP)) < 0.25 )
    	    do_temp = 0;
 
    /* Going down? */
    if (bArg(C_DEPTH) == bArg(B_DEPTH))
    {
        /* Reached bottom temperature, depth, or minimum altitude? 
           Note:  disregard zero altimeter reading... */
        if ((sensor(M_DEPTH) > bArg(B_DEPTH)-0.5) ||
           ((sensor(M_ALTITUDE) > 0.0) && (sensor(M_ALTITUDE) < bArg(MIN_ALT)))) ||
		(do_temp && (sensor(WATER_TEMP) < bArg(B_TEMP)))
        {
            bArg(C_DEPTH) = bArg(T_DEPTH);
            bArg(CYCLES) = bArg(CYCLES) - 0.5;
            sprintf( msg_buf, "% 8.2f: %s down-leg ends at dpth %.2f m\n",
                 sensor(M_PRESENT_TIME), b->name, sensor(M_DEPTH) );
            write_message( MonitorLayeredControl | SLOG );
        }
    }
    else /* Must be going up.... */
    
         /* Reached top temperature or depth? */
        if (sensor(M_DEPTH) < bArg(T_DEPTH)+0.5) ||
	    (do_temp && (sensor(WATER_TEMP) > bArg(T_TEMP)))
        {
            bArg(C_DEPTH) = bArg(B_DEPTH);
            bArg(CYCLES) = bArg(CYCLES) - 0.5;
            sprintf( msg_buf, "% 8.2f: %s up-leg ends at %.2f m\n",
               csensor(M_PRESENT_TIME), b->name, sensor(M_DEPTH) );
            write_message( MonitorLayeredControl | SLOG );
        }
    }

    /* set output command */
    vvp->c[priority].depth_mode = DM_DEPTH;
    vvp->c[priority].depth = bArg(C_DEPTH);    

    return(0);
}
