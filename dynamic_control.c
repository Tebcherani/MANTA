/*----------------------------------------------------------------------*
   Copyright (C) 1994-1998, Massachusetts Institute of Technology.
	 Sea Grant Autonomous Underwater Vehicles Laboratory.
			 All rights reserved.
  $Id: dynamic_control.c,v 2.22 1998/02/03 04:25:07 bamoran Exp $
 *----------------------------------------------------------------------*/

#include <stdio.h>
#include <math.h>

#include "common_defs.h"
#include "odyssey_data_struct.h"
#include "vehicle.h"
#include "whoi_traj.h"
#include "angle_utils.h"

#define FIN_LIMIT D2R(30)	/* maximum fin deflection */
#define P_INT_LIM D2R(10)	/* limit on pitch integrator */
#define H_INT_LIM D2R( 5)	/* limit on hdg */

				/* local function prototypes */
static double pitch_control( double pitch_now, double pitch_des, double
			     pitch_rate, double kpp, double kdp, double
			     kip, double dt );

static const double rho = 1.025e3;  /* kg/m^3 seawater density */

				/* global variables */
static traj_t h_traj, d_traj;
static int MonitorTraj = 0;

/*-----------------------------------------------------------------------*
  dynamic_control: PID controller of heading and depth (inner & outer loops
  for the latter) and simple cubic power law for speed
 *-----------------------------------------------------------------------*/

void dynamic_control( void )
{
    double kpp, kdp, kip, kph, kdh, kih;
    double des_pitch, pitch, pitch_rate, pitch_limit;
    double des_depth;
    double des_head, heading_rate;
    double des_speed, des_omega, LHS; /* speed feedforward */
    double thruster_current = 0.0, Del13 = 0.0, Del24 = 0.0;
    double err_depth;
    double rudder ;
    double hdg_err, proportional, derivative;
    double dt;
				/* UO_CONTROL_PITCH_LIM is specified as
				   degrees in edit_struct, but the rest of
				   the controller is radian based.  Later
				   in limit check compares a radian value
				   to "30" */
    pitch_limit = D2R( read_sensor(UO_CONTROL_PITCH_LIM) );
    kph = read_sensor( UO_CONTROL_KPH );
    kdh = read_sensor( UO_CONTROL_KDH );
    kih = read_sensor( UO_CONTROL_KIH );
    kpp = read_sensor( UO_CONTROL_KPP );
    kdp = read_sensor( UO_CONTROL_KDP );
    kip = read_sensor( UO_CONTROL_KIP );

    dt = limit( read_sensor(M_DT), .3, 0 );

				/* load present vehicle state */
    heading_rate = read_sensor(M_YAW_RATE);
    pitch = read_sensor(M_PITCH);
    pitch_rate = read_sensor(M_PITCH_RATE);

/*-------------------------------------------------------------------
  OPEN LOOP FOR THE SPEED CONTROL
  Different modes of speed control are indicated by the value of
  variable "vvp->c[0].speed_mode" and control is handled accordingly.

  value of (vvp->c[0].speed_mode)
      0:  vvp->c[0].speed[0] carries the desired thruster amperage.
      1:  vvp->c[0].speed[0] carries the desired vehicle speed (m/s),
	  control is open loop...
      2:  vvp->c[0].speed[0] carries the desired vehicle speed (m/s),
          the thruster rotation rate is used for feedback...
      3:  vvp->c[0].speed[0] carries the desired vehicle speed (m/s),
          the water speed sensor is used for feedback...

  Only modes 0 and 1 are implemented at this time...
 *-------------------------------------------------------------------*/

    switch ( vvp->c[0].speed_mode )
    {

    case S_CURRENT:

	thruster_current = vvp->c[0].speed[0];
	update_sensor( C_THRUSTER_CURRENT, vvp->c[0].speed[0] );
        break;

    case S_SPEED:

        des_speed = vvp->c[0].speed[0];
	update_sensor( C_SPEED, vvp->c[0].speed[0] );

        LHS = -read_sensor(VS_XUABU)*pow(des_speed,2.)*( 2.455 - 2.175*des_speed + 
					 0.75*pow(des_speed,2.) ) ;
        des_omega = ( read_sensor(VS_THRUSTER_DUCT_AREA)*rho*read_sensor(VS_THRUSTER_ETA)*read_sensor(VS_THRUSTER_PITCH)*des_speed + 
		      sqrt( pow(read_sensor(VS_THRUSTER_DUCT_AREA)*rho*read_sensor(VS_THRUSTER_ETA)*read_sensor(VS_THRUSTER_PITCH)*des_speed,2.) +
			    4.*LHS*read_sensor(VS_THRUSTER_DUCT_AREA)*rho*read_sensor(VS_THRUSTER_ETA)*read_sensor(VS_THRUSTER_ETA)*read_sensor(VS_THRUSTER_PITCH)*read_sensor(VS_THRUSTER_PITCH) ) ) /
	    (2.*read_sensor(VS_THRUSTER_DUCT_AREA)*rho*pow(read_sensor(VS_THRUSTER_ETA)*read_sensor(VS_THRUSTER_PITCH),2.));

        thruster_current = (.5*rho*read_sensor(VS_THRUSTER_DUCT_AREA)*pow(read_sensor(VS_THRUSTER_ETA)*read_sensor(VS_THRUSTER_PITCH),3.)*des_omega*des_omega -
                            .5*read_sensor(VS_THRUSTER_DUCT_AREA)*rho*read_sensor(VS_THRUSTER_ETA)*read_sensor(VS_THRUSTER_PITCH)*pow(des_speed,2.))/read_sensor(VS_THRUSTER_KT);
				/* make sign same as desired speed */
	thruster_current *= sgn(des_speed);

    break;
    }
				/* limit to +- 5 amps, update */
    thruster_current = limit( thruster_current, 5.0, -5.0 );
    update_sensor( C_THRUSTER_CURRENT, thruster_current );    


/*-------------------------------------------------------------------
  Different modes of depth control are indicated by the value of
  variable "vvp->c[0].depth_mode" and control is handled accordingly.
  
  value of (vvp->c[0].depth_mode)
     0:  vvp->c[0].depth[0] carries the desired elevator angle.
     1:  vvp->c[0].depth[0] carries the desired depth
     2:  vvp->c[0].depth[0] carries the desired pitch angle

  Trajectory generator added 8 March 1996 F. Hover (FSH).  Integrator
  security features by D. Yoerger (DRY) with FSH.
  -------------------------------------------------------------------*/

    if ( !d_traj.init )
	init_traj( &d_traj, read_sensor(M_DEPTH), read_sensor(UO_TAU), 0.2 );

    switch ( vvp->c[0].depth_mode )
    {
    case D_ELEVATOR:

	Del24 = limit( vvp->c[0].depth[0], FIN_LIMIT, -FIN_LIMIT );
	d_traj.init = FALSE;

        break;

    case D_DEPTH:		/* map desired depth to pitch cmd */

	update_sensor( C_DEPTH, vvp->c[0].depth[0] );

        des_depth = do_traj( read_sensor(C_DEPTH), read_sensor(UO_MAX_DR), &d_traj );
	update_sensor( T_DEPTH, des_depth );
	err_depth = read_sensor(M_DEPTH) - des_depth;

	/* FSH & DRY integrator prevents adjustments when desired depth
	   rate is high.  (Also clips at +-5 deg, BAM 10-Jan-1998) */

	if ( fabs(d_traj.xdot) < .01 )
	{
	    add_to_sensor( D_INTEGRAL, read_sensor(UO_DEPTH_INT_K)*err_depth*dt );
	    update_sensor( D_INTEGRAL, limit(read_sensor(D_INTEGRAL), D2R(5), -D2R(5)) );
	}

	des_pitch = read_sensor(UO_DEPTH_MAP_K)*err_depth + read_sensor(D_INTEGRAL);
	des_pitch = limit( des_pitch, pitch_limit, -pitch_limit );	
	update_sensor( C_PITCH, des_pitch );

	Del24 = pitch_control( pitch, des_pitch, pitch_rate, kpp, kdp,
			       kip, dt );
	Del24 = limit( Del24, FIN_LIMIT, -FIN_LIMIT);

	break;

    case D_PITCH:		/* dynamic control of pitch */

        des_pitch = limit( vvp->c[0].depth[0], pitch_limit, -pitch_limit );
        update_sensor( C_PITCH, des_pitch );

        Del24 = pitch_control( pitch, des_pitch, pitch_rate, kpp, kdp,
			       kip, dt );        
        Del24 = limit( Del24, FIN_LIMIT, -FIN_LIMIT );

	d_traj.init = FALSE;
        break;
    }
    update_sensor( C_ELEVATOR_ANGLE, Del24 );

/* -------------------------------------------------------------------
   |   PID HEADING CONTROL LAW ALGORITHM
   |   Different modes of speed control are indicated by the value of
   |   variable "vvp->c[0].heading_mode" and control is handled 
   |   accordingly.
   |
   |    value of (vvp->c[0].heading_mode)
   |     0:  vvp->c[0].heading[0] carries the desired rudder angle.
   |     1:  vvp->c[0].heading[0] carries the desired heading
   |
   |    Only modes 0 and 1 are implemented at this time...
   |                              J. G. Bellingham 1/30/94
   |    FSH Mods 7 March 1996
   -------------------------------------------------------------------*/

    if ( !h_traj.init )
	init_traj( &h_traj, read_sensor(M_YAW), read_sensor(UO_TAU), 0.2 );

    switch ( vvp->c[0].heading_mode )
    {
    case H_RUDDER:

        Del13 = limit( vvp->c[0].heading[0], FIN_LIMIT, -FIN_LIMIT );
        update_sensor( C_RUDDER_ANGLE, Del13 );

	h_traj.init = FALSE;
        break;
        
    case 1:			/* heading control mode */

	update_sensor( C_HEADING, vvp->c[0].heading[0] );
	des_head = do_hdg_traj( read_sensor(C_HEADING), read_sensor(UO_MAX_HR),
				&h_traj );

	update_sensor( T_HDG, des_head );
	update_sensor( T_HDG_RATE, h_traj.xdot );

	hdg_err = wrap_angle( read_sensor(M_YAW) - des_head );
	proportional = kph * hdg_err;
        derivative = kdh * (heading_rate - h_traj.xdot);

        rudder = read_sensor(C_RUDDER_ANGLE);

	if ( fabs(h_traj.xdot) < D2R(1) )
	{

	    if ( (fabs(rudder) < FIN_LIMIT) || (fabs(kih) <= 1e-6) )
		add_to_sensor( H_INTEGRAL, kih*dt*hdg_err );
	    else
		update_sensor( H_INTEGRAL, sgn(rudder)*FIN_LIMIT - 
		    proportional - derivative );

#ifdef H_INT_LIM
				/* clip the integral term to prevent its
				   influence from growing out of bounds */
	    update_sensor( H_INTEGRAL, limit(read_sensor(H_INTEGRAL),
				      H_INT_LIM, -H_INT_LIM) );
#endif
	}

	Del13 = proportional + derivative + read_sensor(H_INTEGRAL);
	Del13 = limit( Del13, FIN_LIMIT, -FIN_LIMIT );

	update_sensor( C_RUDDER_ANGLE, Del13 );
	break;
    }
}

/* ==================================================================
|    PITCH_CONTROL:  
|      Executes control of vehicle pitch...      
|      9/22 jl adding proportional term (to add damping) based on 
|      the oscillations seen in charles river run "sep16_11".
|      ... makes a great improvement in simulation, but this 
|      relies on depth rate from simulator, make sure the real 
|      vehicle code calculates the depth rate properly.
|      FSH rewrite 8 March 1996.
| ================================================================== */

static double pitch_control( double pitch_now, double pitch_des, double
			     pitch_rate, double kpp, double kdp, double
			     kip, double dt )
{
    double x1, x2, elevator;
    
    x1 = kpp * (pitch_now - pitch_des);
    x2 = kdp * pitch_rate;

    elevator = read_sensor(C_ELEVATOR_ANGLE) ;

    if ((fabs(elevator) < FIN_LIMIT) || (fabs(kip) < 1.e-6))
	add_to_sensor( P_INTEGRAL, kip*dt*(pitch_now - pitch_des) );
    else
	update_sensor( P_INTEGRAL, sgn(elevator)*FIN_LIMIT - x1 - x2 );

#ifdef P_INT_LIM
				/* limit the integrator to prevent windup */
    update_sensor( P_INTEGRAL, limit(read_sensor(P_INTEGRAL), P_INT_LIM, -P_INT_LIM) );
#endif

    return x1 + x2 + read_sensor(P_INTEGRAL);
}
  
/* ==================================================================
|    LIMIT:  
|      Makes sure a variable falls within defined limits...      
| ================================================================== */

double limit( double value, double max, double min )
{
    double dummy;

    if ( max < min )
    {
        dummy = max;
        max = min;
        min = dummy;
    }

    if (value > max)
        value = max;
    if (value < min)
        value = min;
    return value;
}

/*===================================================================*/

double sgn( double x )
{
    if ( x < 0 )
	return -1;
    else
	return  1;
}
