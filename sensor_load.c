/*----------------------------------------------------------------------*
   Copyright (C) 1994-1997, Massachusetts Institute of Technology.
	 Sea Grant Autonomous Underwater Vehicles Laboratory.
			 All rights reserved.
  $Id: sensor_load.c,v 2.23 1998/02/03 04:25:07 bamoran Exp $
 *----------------------------------------------------------------------*/

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "common_defs.h"
#include "diagnostic.h"
#include "odyssey_data.h"
#include "vehicle.h"
#include "random_utils.h"

/* #define KALMAN_LBL 1 /* comment out to have all tofs output together */

#define USBL_BEACONS 4
double usbl_north[USBL_BEACONS] = {-100.0, 50.0, 0.0, 0.0};
double usbl_east[USBL_BEACONS] = {100.0, -50.0, 0.0, 0.0};
double usbl_depth[USBL_BEACONS] = {5.0, 10.0, 0.0, 0.0};
double usbl_if[USBL_BEACONS] = {19.0, 19.0, 0.0, 0.0};
double usbl_rf[USBL_BEACONS] = {21.0, 25.0, 0.0, 0.0};
/*--- if=beacons interogate freq (19kHz),rf=reply frequency (21 or 25kHz) */


void load_water_speed( void );
void load_water_param();

void simLBL()
{
#ifdef KALMAN_LBL
    void simLBL4EKF();
    simLBL4EKF();
#else
    void simEdgeTechLBL();
    simEdgeTechLBL();
#endif
}

/*
 * Calculate water speed sensor readings...
 */

void load_water_speed( void )
{
    double std_normal_random_num();
    update( M_WATER_SPEED, sensor(REAL_U) + 0.02*std_normal_random_num() );
}

/* ------------------------------------------------------------ */
/* Calculate sensed temperature and conductivity readings...
*/

void load_water_param(void)
{
    update( WATER_TEMP, 10 * exp(-pow(sensor(REAL_Z)/30,2)) );
    sim_conduct();
}

/*--------------------------------------------------------------------------*
  Calculate sensed LBL readings...                         

  jl: 10/5/94 making changes for spherical lbl, reporting round-trip
  travel times instead of arrival times 
 
  jl: 10/25/94 changing so that times 1,2,3,4 are used 
  (from now on, M_LBL_0T corresponds to the bottom return)

  jv: 7/13/95 simulating the times of flight output as observed in JdF'95
  data. 
  The tof are set to 0 except:
      * at the sampling time corresponding to the vehicle ping and the
        sampling time immediately following, where the tof are set to -1.0
      * at the sampling time corresponding to the LBL timeout and the
        sampling time immediately following where the tof are set to the
        measured values.
  M_LBL_PING_TIME is set to 0 except:
      * at the sampling time corresponding to the vehicle ping and the
        sampling time immediately following, where M_LBL_PING_TIME is set
	to the sampling time corresponding to the vehicle ping.
  if a tof drops out 0.0 is written in the structure.
 *--------------------------------------------------------------------------*/

static int MonitorLBLSim = 2;	/* output to logfile */

void simEdgeTechLBL()
{
    static int uninitialized = 1;
    static int num_beacons, update_times;
    static int gotReply[5], cycle_number = 0;
    static double memo1[5];
    static double north[5], east[5], depth[5], tat[5];
    static double last_cycle_time, cycle_time, lbl_cycle, next_ping = 0;
    static double sound_speed, timeout, percent_bias, percent_dropout;
    static double bias_amplitude, noise_amplitude;
    static double north_ping, east_ping, depth_ping;

    int i, j, offset;
    double dist1, dist2, time_of_flight;
    double since_cycle, present_time;
    double noise, bias;

    if (uninitialized)
    {
        lbl_cycle = sensor(U_LBL_CYCLE);
        sound_speed = sensor(U_SPEED_OF_SOUND);
        num_beacons = (int) sensor(U_LBL_BEACON_NUMBER);
        timeout = sensor(U_LBL_TIMEOUT);
        percent_bias = sensor(REAL_LBL_FRAC_BIAS);
        percent_dropout = sensor(REAL_LBL_FRAC_DROPOUT);
        bias_amplitude = sensor(REAL_LBL_BIAS_AMPLITUDE); /* mSec */
        noise_amplitude = sensor(REAL_LBL_TIMING_NOISE);  /* mSec */
        last_cycle_time = 0.0;

        strcpy( msg_buf, "simEdgeTechLBL:\n" );
        write_message( MonitorLBLSim | SLOG );        
        sprintf( msg_buf, 
		 "lbl_cycle %.1f, sos %.1lf, %d beacons, timeout %.1lf\n",
		 lbl_cycle, sound_speed, num_beacons, timeout ); 
        write_message( MonitorLBLSim | SLOG );        
        sprintf( msg_buf, 
		 "percent_bias=%lf, bias_amplitude=%lf, noise_amplitude=%lf\n",
		 percent_bias, bias_amplitude, noise_amplitude );
        write_message( MonitorLBLSim | SLOG );

        for ( i = 1, offset = 0; i <= num_beacons; i++ )
	{
            gotReply[i] = 0;
				/* store beacon coordinates */
            north[i] = sensor( U_LBL_1NORTH + offset );
            east[i]  = sensor( U_LBL_1EAST  + offset );
            depth[i] = sensor( U_LBL_1DEPTH + offset );
	    tat[i]   = sensor( U_LBL_1TAT   + offset );
            offset += U_LBL_2NORTH - U_LBL_1NORTH;

            sprintf( msg_buf, "\tX%d: N%7.1lf E%7.1lf D%.1lf T%.3f\n",
		     i, north[i], east[i], depth[i], tat[i] );
            write_message( MonitorLBLSim | SLOG );        
        }
        uninitialized = 0;
    }
				/* PING! (only if c_lbl_control is on, but
				   update next_ping regardless) */
    present_time = sensor(M_PRESENT_TIME);
    if ( present_time >= next_ping )
    {
	next_ping += sensor(U_LBL_CYCLE);
	if ( sensor(C_LBL_CONTROL) > 0 )
	{
	    update( M_LBL_PING_TIME, present_time );
	    sprintf( msg_buf, "% 8.2f: LBL PING, next at % 8.2f\n",
		     sensor(M_LBL_PING_TIME), next_ping );
	    write_message( MonitorLBLSim | SLOG );
	    for ( i = 0; i <= num_beacons; i++ )
		update( M_LBL_0T+i, -1.0 );

	    cycle_number++;
	    update_times = 1;
	    north_ping = sensor(REAL_X);
	    east_ping = sensor(REAL_Y);
	    depth_ping = sensor(REAL_Z);
	    memset( gotReply, 0, sizeof(gotReply) );
	}
    }
				/* compute time of ping and time elapsed
                                   since ping */
    cycle_time = sensor(M_LBL_PING_TIME);
    since_cycle = present_time - cycle_time;

				/* if between ping time and before LBL
                                   timeout */
    if ( (present_time > cycle_time) && (since_cycle < timeout) &&
	 (cycle_number > 0) )
    {
        for( i = 1; i <= num_beacons; i++ )
	{
            if ( !gotReply[i] )
	    {
				/* distance from position at ping to beacon */
                dist1  = (north_ping-north[i])*(north_ping-north[i]);
                dist1 += (east_ping-east[i])*(east_ping-east[i]);
                dist1 += (depth_ping-depth[i])*(depth_ping-depth[i]);
                dist1  = sqrt(dist1);
				/* add distance from beacon to current
                                   position */
                dist2  = (sensor(REAL_X)-north[i])*(sensor(REAL_X)-north[i]);
                dist2 += (sensor(REAL_Y)-east[i])*(sensor(REAL_Y)-east[i]);
                dist2 += (sensor(REAL_Z)-depth[i])*(sensor(REAL_Z)-depth[i]);
                dist2  = sqrt(dist2);
				/* time required to travel distance1 +
                                   distance2 + tat */
                time_of_flight = (dist1+dist2)/sound_speed + tat[i];

				/* add noise */
                noise = std_normal_random_num()*noise_amplitude/1000.0;
                time_of_flight += noise;

				/* add bias */
                if ( drand48() < percent_bias )
		{
                    bias = drand48()*bias_amplitude/1000.0;
                    time_of_flight += bias;
                }

                if ( time_of_flight < since_cycle )
		{
                    gotReply[i] = 1;
                    if ( drand48() < percent_dropout )
				/* beacon i drops out */
                        memo1[i] = 0.0;
                    else	/* beacon i gets calculated value */
                        memo1[i] = time_of_flight;
                }
            } /* if memo... */
        } /* for numbeacons */
    } /* if present... */
				/* if LBL has timed out: update structure */
    if ( update_times && (since_cycle > timeout) )
    {
	for ( i = 1; i <= num_beacons; i++ )
	{
	    if ( i == 1 )
	    {
		sprintf( msg_buf, "% 8.2f: simEdgeTechLBL updates ...\n",
			 sensor(M_PRESENT_TIME) );
		write_message( MonitorLBLSim & SLOG );
	    }
	    update( M_LBL_0T+i, memo1[i] );
	    sprintf( msg_buf, "\tM_LBL_%dT %.4f\n", i, memo1[i] );
	    write_message( MonitorLBLSim & SLOG );
	}
	update( LBL_NEWDATA, 1 );
	update_times = 0;
    }
}

/*--------------------------------------------------------------------------*/

void simLBL4EKF()
{
    static int uninitialized = 1;
    static int num_beacons, time_received[5];
    static double north[5], east[5], depth[5], tat[5];
    static double last_cycle_time,cycle_time,lbl_cycle;
    static double sound_speed,timeout,percent_bias, percent_dropout;
    static double bias_amp,noise_amp;
    static double north_ping,east_ping,depth_ping;

    int i,j, offset;
    double dist1,dist2,time_of_flight;
    double since_cycle,cycle_number,present_time;
    double noise, bias;
    
    if (uninitialized)
    {
        uninitialized = 0;
        lbl_cycle = sensor(U_LBL_CYCLE);
        sound_speed = sensor(U_SPEED_OF_SOUND);
        num_beacons = (int) sensor(U_LBL_BEACON_NUMBER);
        timeout = sensor(U_LBL_TIMEOUT);
        percent_bias = sensor(REAL_LBL_FRAC_BIAS);
        percent_dropout = sensor(REAL_LBL_FRAC_DROPOUT);
        bias_amp = sensor(REAL_LBL_BIAS_AMPLITUDE); /* mSec */
        noise_amp = sensor(REAL_LBL_TIMING_NOISE);  /* mSec */
        last_cycle_time=0.0;
        
        strcpy( msg_buf, "simLBL4EKF:\n");
        write_message( MonitorLBLSim | SLOG );        
        sprintf( msg_buf,
		 "\tlbl_cycle %.1g, timeout %.1g, C %g, num_beacons %d\n",
		 lbl_cycle, timeout, sound_speed, num_beacons ); 
        write_message( MonitorLBLSim | SLOG );        
        sprintf( msg_buf,
		 "\tpercent_bias %g, bias_amp %g, noise_amp %g\n",
		 percent_bias, bias_amp, noise_amp );
        write_message( MonitorLBLSim | SLOG );        

				/* store beacon coordinates */
        for ( i = 1, offset = 0; i <= num_beacons; i++ )
	{
            time_received[i] = 0;
            north[i] = sensor( U_LBL_1NORTH + offset );
            east[i] = sensor( U_LBL_1EAST + offset );
            depth[i] = sensor( U_LBL_1DEPTH + offset );
            offset += U_LBL_2NORTH - U_LBL_1NORTH;
            sprintf( msg_buf, "\tLBL%d: N%7.1lf E%7.1lf D%.1lf\n", i,
		     north[i], east[i], depth[i] );
            write_message( MonitorLBLSim | SLOG );        
        }
    }

				/* set M_LBL_PING_TIME and times to 0 (will
                                   be overwritten later when required) */
    update( M_LBL_PING_TIME, 0.0 );
    for ( i = 1; i <= num_beacons; i++ )
        update( M_LBL_0T+i,0.0);
    
    /*--- compute time of ping and time elapsed since ping */
    present_time = sensor(M_PRESENT_TIME);
    since_cycle = lbl_cycle * modf((present_time/lbl_cycle),&cycle_number);
    cycle_time = cycle_number*lbl_cycle;

    /*--- if vehicle just pinged set M_LBL_PING_TIME  */
    if ( cycle_time != last_cycle_time ) {
        update(M_LBL_PING_TIME,cycle_time);

        last_cycle_time = cycle_time;
        north_ping = sensor(REAL_X);
        east_ping = sensor(REAL_Y);
        depth_ping = sensor(REAL_Z);
    }

    /*--- if between ping time and before LBL timeout */
    if ( (present_time > cycle_time) && (since_cycle < timeout) && (cycle_number > 0.0) ) {
        for(i=1; i<=num_beacons; i++) {
            if (time_received[i]==0) {
                
                /*--- distance from position at ping to beacon */
                dist1  = (north_ping-north[i])*(north_ping-north[i]);
                dist1 += (east_ping-east[i])*(east_ping-east[i]);
                dist1 += (depth_ping-depth[i])*(depth_ping-depth[i]);
                dist1  = sqrt(dist1);

                /*--- add distance from beacon to current position */
                dist2  = (sensor(REAL_X)-north[i])*(sensor(REAL_X)-north[i]);
                dist2 += (sensor(REAL_Y)-east[i])*(sensor(REAL_Y)-east[i]);
                dist2 += (sensor(REAL_Z)-depth[i])*(sensor(REAL_Z)-depth[i]);
                dist2  = sqrt(dist2);

                /*--- time required to travel dist1 + dist2 */
                time_of_flight = (dist1+dist2)/sound_speed;

                /*--- add noise and turn around time measurement errors */
                noise = std_normal_random_num()*noise_amp/1000.0;
                time_of_flight += noise + tat[i];

				/* add bias */
                bias = 0.0;
                if ( drand48() < percent_bias ) {
                    bias = drand48()*bias_amp/1000.0;
                    time_of_flight += bias;
                }

                if (time_of_flight < since_cycle)
		{
                    time_received[i]=1;  /* pulse from beacon i drops out */
                    if ( !(drand48() < percent_dropout) )
		    {
                        update( M_LBL_0T+i, time_of_flight );
			update( LBL_NEWDATA, 1 );
                    }
                }
            }
        }
    }
				/* reset update and memo for next lbl cycle */
    if ( since_cycle > (timeout+0.3) )
        for (i=1; i<=num_beacons; i++)
            time_received[i]=0;
}

/* ============================================================ */
/* Display & clear modem send string.                           */

int handle_modem(void)
{
    int monitor_high_level_control = 0;

    if( strlen(vvp->modem_send) > 0 )
    {
	sprintf( msg_buf, "\nModem t=%.2lf %s\n", 
		 sensor(M_PRESENT_TIME), vvp->modem_send ); 
	write_message( monitor_high_level_control );
	strcpy( vvp->modem_send, "" ); 
    }

    return 0;
}

/*-----------------------------------------------------------------------*
  getXSDataByte: set appropriate bits of databyte that is to be sent
  through edgetech channel
 *-----------------------------------------------------------------------*/

int getXSDataByte( void )
{
    int databyte = 0;

#ifdef WHOI_LATCH
				/* LSB is presence of pole */
    databyte |= sensor(M_POLE_PRESENT) > 0;
				/* bits 1-2 are range from dock */
    if ( sensor(M_USBL_RANGE) > 1000 )
	databyte |= 6;
    else if ( sensor(M_USBL_RANGE) > 500 )
	databyte |= 4;
    else if ( sensor(M_USBL_RANGE) > 50 )
	databyte |= 2;
				/* bit 3 always high during missions */
    databyte |= 8;
#endif
				/* MSB high iff all else low */
    if ( !databyte )
	databyte = 128;

    return databyte;
}
