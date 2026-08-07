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

#ifndef drand48
#define drand48() ((double)rand()/RAND_MAX)
#endif

#define USE_KVH 1 /* comment out to use sensor_AHRS.c + modify makefile */
/* #define KALMAN_LBL 1 /* comment out to have all tofs output together */

#define G        (double) 9.81

/* all these globals should be replaced with entries in vvv in future */

/* Earths magnetic field orientation... bx,by,bz are northerly easterly,
   and downward components in gauss intensity = 0.55 Gauss, inclination =
   60 deg */

static double earth_bx = 0.275, earth_by = 0.0, earth_bz = 0.476;

#define USBL_BEACONS 4
double usbl_north[USBL_BEACONS] = {-100.0, 50.0, 0.0, 0.0};
double usbl_east[USBL_BEACONS] = {100.0, -50.0, 0.0, 0.0};
double usbl_depth[USBL_BEACONS] = {5.0, 10.0, 0.0, 0.0};
double usbl_if[USBL_BEACONS] = {19.0, 19.0, 0.0, 0.0};
double usbl_rf[USBL_BEACONS] = {21.0, 25.0, 0.0, 0.0};
/*--- if=beacons interogate freq (19kHz),rf=reply frequency (21 or 25kHz) */


void init_rotation();
void load_accel();
void load_angular_rate();
void load_depth();
void load_water_speed( void );
void load_magnetic();
void load_water_param();
double gasdev();

#ifdef USE_KVH
  void load_compass();
  void load_inclinometers(double,double); 
  void low_pass_filter(double[2],double[2]);
#endif

double mr[4][4];

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

/* ------------------------------------------------------------ */
/* Initialize rotation matrices */

void init_rotation(void)
{
  double psi, theta, phi; /* yaw, pitch and roll angles, respectively */
  
  psi = sensor(REAL_PSI);
  theta = sensor(REAL_THETA);
  phi = sensor(REAL_PHI);
  
  /* To find the body frame representation of a vector in the earth frame do:*/
  /* xb[i] = mr[i][1]*xe[1] + mr[i][2]*xe[2] + mr[i][3]*xe[3] */
  /*                                                                        */
  /* To find the earth frame representation of a vector in the body frame do:*/
  /* xe[i] = mr[1][i]*xb[1] + mr[2][i]*xb[2] + mr[3][i]*xb[3]         */
  /*                                                                        */
  
  mr[1][1] = cos(psi)*cos(theta);
  mr[1][2] = cos(theta)*sin(psi);
  mr[1][3] = -sin(theta);
  
  mr[2][1] = -(cos(phi)*sin(psi)) + cos(psi)*sin(phi)*sin(theta);
  mr[2][2] = cos(phi)*cos(psi) + sin(phi)*sin(psi)*sin(theta);
  mr[2][3] = cos(theta)*sin(phi); 
  
  mr[3][1] = sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(theta);
  mr[3][2] = -(cos(psi)*sin(phi)) + cos(phi)*sin(psi)*sin(theta);
  mr[3][3] = cos(phi)*cos(theta);
}
/* ------------------------------------------------------------ */
/* Calculate sensed acceleration of vehicle...
** (JV 6/13/95) changed computation of acceleration to avoid using
** the rotation matrix twice (used the direct expression of accel)
*/

#ifdef USE_KVH
double aax,aay; /* JV 5/30 to simulate flux gate compass measurements */
#endif

void load_accel(void)
{
    static double dx=1.390,dy=0.108,dz=0.076;
    double ax,ay,az;
    double p,q,r,pdot,qdot,rdot;
    double u,v,w,udot,vdot,wdot;
    double noise;
    double theta,phi;
				/* load real velocities and acceleration
                                   (linear and angular) */
    u    = sensor(REAL_U);     v = sensor(REAL_V);     w = sensor(REAL_W);
    udot = sensor(REAL_UD); vdot = sensor(REAL_VD); wdot = sensor(REAL_WD);
    p    = sensor(REAL_P);     q = sensor(REAL_Q);     r = sensor(REAL_R);
    pdot = sensor(REAL_PD); qdot = sensor(REAL_QD); rdot = sensor(REAL_RD);

				/* non gravitational acceleration at
                                   location of AHRS in vehicle frame */
    ax = udot + q*w-r*v + qdot*dz-rdot*dy - (q*q+r*r)*dx + p*q*dy + p*r*dz;
    ay = vdot + r*u-p*w + rdot*dx-pdot*dz - (p*p+r*r)*dy + p*q*dx + q*r*dz;
    az = wdot + p*v-q*u + pdot*dy-qdot*dx - (p*p+q*q)*dz + q*r*dy + p*r*dx;

#ifdef USE_KVH
				/* compute non gravitational acc in local
                                   horizontal plane for compass */
    theta = sensor(REAL_THETA);
    phi = sensor(REAL_PHI);
    aax = cos(theta)*ax + sin(phi)*sin(theta)*ay + cos(phi)*sin(theta)*az;
    aay = cos(phi)*ay - sin(phi)*az;
#endif
				/* add gravity */
    ax = ax + sin(theta)*G;
    ay = ay - cos(theta)*sin(phi)*G;
    az = az - cos(theta)*cos(phi)*G;

#ifdef USE_KVH
    load_inclinometers( ax, ay );
#else
				/* update data structure */
    noise = gasdev()*sensor(REAL_ACC_NOISE);
    update(M_ACCEL_X, ax + noise);

    noise = gasdev()*sensor(REAL_ACC_NOISE);
    update(M_ACCEL_Y, ay + noise);

    noise = gasdev()*sensor(REAL_ACC_NOISE);
    update(M_ACCEL_Z, az + noise);
#endif

}

/* ------------------------------------------------------------ */
/* Calculate sensed magnetometer readings...
*/

void load_magnetic(void)
{        
#ifdef USE_KVH
    load_compass();
#else
    double mx,my,mz;
				/* Rotate to vehicle frame */
    mx = mr[1][1]*earth_bx + mr[1][2]*earth_by + mr[1][3]*earth_bz;
    my = mr[2][1]*earth_bx + mr[2][2]*earth_by + mr[2][3]*earth_bz;
    mz = mr[3][1]*earth_bx + mr[3][2]*earth_by + mr[3][3]*earth_bz;
  
    update(M_MAG_X, mx);
    update(M_MAG_Y, my);
    update(M_MAG_Z, mz);
#endif
}

/* ------------------------------------------------------------ */
/* Calculate angular rate readings...
*/

void load_angular_rate(void)
{
int constant_drift=1;
double p,q,r;
double noise, rate_noise_level=0.00001745;
double T=50, dt=0.02, std=0.005; /* Gauss-Markov processes */
static double bias1=0.0, bias2=0.0, bias3=0.0;


  if (constant_drift==1) {
      bias1=4.3663e-4;
      bias2=8.7266e-4;
      bias3=-8.7266e-4;
  }
  else {
      bias1 = bias1 * (1-dt/T) + gasdev() * std * dt;
      bias2 = bias2 * (1-dt/T) + gasdev() * std * dt;
      bias3 = bias3 * (1-dt/T) + gasdev() * std * dt;
  }

  noise = rate_noise_level * gasdev(); 
  p=sensor(REAL_P) + noise + bias1;

  noise = rate_noise_level * gasdev();  
  q=sensor(REAL_Q) + noise + bias2;

  noise = rate_noise_level * gasdev();  
  r=sensor(REAL_R) + noise + bias3;
  
  /*--- update data structure */
  update(M_ROLL_RATE, p);
  update(M_PITCH_RATE, q);
  update(M_YAW_RATE, r);
  
}

/* ------------------------------------------------------------ */
/* Calculate depth sensor readings...
*/

void load_depth(void)
{
    double noise;

    noise = gasdev()*sensor(REAL_DEPTH_NOISE);
    update( M_DEPTH, sensor(REAL_Z) + noise );
}

/*
 * Calculate water speed sensor readings...
 */

void load_water_speed( void )
{
    double gasdev();
    update( M_WATER_SPEED, sensor(REAL_U) + 0.02*gasdev() );
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

  drand48() outputs values uniformly distributed between 0 and 1

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
                noise = gasdev()*noise_amplitude/1000.0;
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
                noise = gasdev()*noise_amp/1000.0;
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

#ifdef USE_KVH
/*-------------------------------------------------------------------------*/

void load_inclinometers( double ax, double ay )
{
double inclino_noise=0.001745; /* 0.1 degree */
double raw_pitch_roll[2],filtered_pitch_roll[2];

    raw_pitch_roll[0]=asin(ax/G);
    raw_pitch_roll[1]=-asin(ay/(G*cos(raw_pitch_roll[0])));
    low_pass_filter(raw_pitch_roll,filtered_pitch_roll);

    filtered_pitch_roll[0]+=inclino_noise*gasdev();
    filtered_pitch_roll[1]+=inclino_noise*gasdev();

    update(M_INCLINO_PITCH,filtered_pitch_roll[0]);
    update(M_INCLINO_ROLL,filtered_pitch_roll[1]);
}

/*-------------------------------------------------------------------------*
(JV 5/31/95)
Simulation of the magnetic heading measured by the KVH DGC
  . a 3-axis magnetometer (s3am) suspended in the DGC housing is considered.
  . the s3am pitches and rolls with respect to the horizontal because
    of the vehicle accelerations (linear and angular)
  . to compute the s3am pitch and roll, the motion of a 2D linear damped
    oscillator is first determined and the x-y displcaements converted
    into angles.
*-------------------------------------------------------------------------*/

void load_compass()
{
static int first=1;
static double compass_noise=0.001745; /* 0.1 deg */
static double dx=0.0, dy=0.0, dxp=0.0, dyp=0.0;
static double beta, omega0, omega1;
double dt=0.02, mass=0.1, kx=100.0, bxp=0.5, length=0.015;
double psi, theta, phi;
double mx, my, mz;
double mg_yaw;
double cst1, cst2, cc, ss;
double Mmg[4][4];


    if (first) {
       first = 0;
       beta = bxp/(2.0*mass); omega0=sqrt(kx/mass);
       omega1 = sqrt(omega0*omega0-beta*beta);
       if ((beta*beta-omega0*omega0) >= 0.0) {
           printf("WARNING !! sensor_load.c: compass model not underdamped\n");
           printf("beta^2=%lf omega0^2=%lf\n",beta*beta,omega0*omega0);
       }
    }

    /* compute pitch and roll of mg sensor given accelerations */
    cc = cos(omega1*dt);
    ss = sin(omega1*dt);

    cst1 = aax/(omega0*omega0)+dx;
    cst2 = (dxp+beta*(aax/(omega0*omega0)+dx))/omega1;
    dx = exp(-beta*dt)*(cst1*cc+cst2*ss)-aax/(omega0*omega0);
    dxp = -beta*exp(-beta*dt)*(cst1*cc+cst2*ss)
    +exp(-beta*dt)*(-cst1*omega1*ss+cst2*omega1*cc);

    cst1 = aay/(omega0*omega0)+dy;
    cst2 = (dyp+beta*(aay/(omega0*omega0)+dy))/omega1;
    dy = exp(-beta*dt)*(cst1*cc+cst2*ss)-aay/(omega0*omega0);
    dyp = -beta*exp(-beta*dt)*(cst1*cc+cst2*ss)
    +exp(-beta*dt)*(-cst1*omega1*ss+cst2*omega1*cc);

    theta = dx/length; phi = -dy/length;

    /* compute the magnetic readings given the orientation of mg sensor */
    psi=sensor(REAL_PSI);
    Mmg[1][1] = cos(psi)*cos(theta);
    Mmg[1][2] = cos(theta)*sin(psi);
    Mmg[1][3] = -sin(theta);
    Mmg[2][1] = -(cos(phi)*sin(psi)) + cos(psi)*sin(phi)*sin(theta);
    Mmg[2][2] = cos(phi)*cos(psi) + sin(phi)*sin(psi)*sin(theta);
    Mmg[2][3] = cos(theta)*sin(phi); 
    Mmg[3][1] = sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(theta);
    Mmg[3][2] = -(cos(psi)*sin(phi)) + cos(phi)*sin(psi)*sin(theta);
    Mmg[3][3] = cos(phi)*cos(theta);

    mx = Mmg[1][1]*earth_bx + Mmg[1][2]*earth_by + Mmg[1][3]*earth_bz;
    my = Mmg[2][1]*earth_bx + Mmg[2][2]*earth_by + Mmg[2][3]*earth_bz;
    mz = Mmg[3][1]*earth_bx + Mmg[3][2]*earth_by + Mmg[3][3]*earth_bz;

    /* compute heading from readings */
    if ( (mx>0.0) && (my<=0.0) ) mg_yaw = -atan(my/mx);
    if ( (mx<=0.0) && (my<0.0) ) mg_yaw = atan(mx/my)+PI/2.0;
    if ( (mx<0.0) && (my>=0.0) ) mg_yaw = -atan(my/mx)+PI;
    if ( (mx>=0.0) && (my>0.0) ) mg_yaw = atan(mx/my)+3.0*PI/2.0;

    /* add noise and write to the structure */
    mg_yaw += compass_noise*gasdev();

    mg_yaw -=        sensor( UO_MAGNETIC_VARIATION ); /* jleonard 6/29/96 */

    update(M_COMPASS_YAW,mg_yaw);
}

/*-------------------------------------------------------------------------
Low-pass filter for inclinometers simulation
cut-off frequency=1Hz, for a sampling frequency of 50Hz
J. Vaganay - 5/12/95
-------------------------------------------------------------------------*/

void low_pass_filter( double s[2], double sf[2] )
{
static int count=1;
static double sp[2][3];  /* previous non filtered signals */
static double sfp[2][3]; /* previous filtered signals */
static double a[3],b[3]; /* filter coefficients */
int i,j;

 for (i=0;i<=1;i++) {
     if(count==1) {
         sp[i][1]=s[i];
         sfp[i][1]=s[i];
         sf[i]=s[i];
         a[1]=-1.82269492519631; a[2]=0.83718165127602;
         b[0]=0.00362168151493;  b[1]=0.00724336302986;
         b[2]=0.00362168151493;
     }
      
     if(count==2) {
         sp[i][2]=sp[i][1];
         sfp[i][2]=sfp[i][1];
         sp[i][1]=s[i];
         sfp[i][1]=s[i];
         sf[i]=s[i];
     }
      
     if(count>2) {
         sf[i]=b[0]*s[i];
         for(j=1;j<=2;j++)
             sf[i]+=b[j]*sp[i][j]-a[j]*sfp[i][j];
         for(j=2;j>1;j--) {
             sp[i][j]=sp[i][j-1];
             sfp[i][j]=sfp[i][j-1];
         }
         sp[i][1]=s[i];
         sfp[i][1]=sf[i];
     }
 }
 if(count<=2) count++;

}
#endif

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
