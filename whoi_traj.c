/*----------------------------------------------------------------------*
   Copyright (C) 1994-1998, Massachusetts Institute of Technology.
	 Sea Grant Autonomous Underwater Vehicles Laboratory.
			 All rights reserved.
  $Id: whoi_traj.c,v 2.21 1998/01/20 00:31:51 bamoran Exp $
 *----------------------------------------------------------------------*/

/*-----------------------------------------------------------------------*
 | File: whoi_traj.c
 | Authors: Dana Yoeger, Franz Hover of WHOI.
 | Created: 3-11/96
 *-----------------------------------------------------------------------*/

#include <math.h>

#include "whoi_traj.h"

/* keeps heading between 0-2PI, only corrects once though */
void wrap_heading(double *h)
{
    while(*h > TWOPI)*h -= TWOPI ;
    while(*h < 0)*h += TWOPI ;

}
/* corrects the difference between 2 headings to keep the value between
+- pi.  In other words, it takes the shortest distance around the circle
*/
double delta_heading(double dh)
{
    double result ;
    result = dh ;
    while(result > PI)result -= TWOPI ;
    while(result < -PI)result += TWOPI ;
    return(result) ;
}


void init_traj( traj_t *t, double x, double tau, double dt )
{
    if ( t->init )
	return;

    t->x = x;
    t->tau = tau;
    t->dt = dt;

    t->xdot = 0.0;
    t->alpha = exp(-dt/tau) ;
    t->beta = 1.0 - t->alpha ;

    t->init = TRUE;
}

double trajsgn( double x )
{
    if ( x < 0.0 )
	return -1;
    return 1;
}

/*-----------------------------------------------------------------------*
  do_hdg_traj: executes the hdg trajectory generator for one time step
  inputs:
      xf is the final, or goal heading in radians (0-2PI)
      xdotmax is the max desired hdg rate (rad/sec)
  outputs:
      the trajectory structure will have updated x and xdot elements
 *-----------------------------------------------------------------------*/

double do_hdg_traj( double xf, double xdotmax, traj_t *t )
{
    double dx, new_x, last_x, speed ;

    speed = fabs( xdotmax );
    dx = delta_heading( xf - t->x );
    last_x = xf - dx ;
    t->xdot = t->alpha*t->xdot + t->beta*speed*trajsgn(dx) ;

    if ( fabs(dx/t->tau) > speed )
	new_x = last_x + t->xdot*t->dt ;
    else
    {
	new_x = t->alpha*last_x + t->beta*xf ;
	t->xdot = delta_heading(new_x - t->x) / t->dt ;
    }
    wrap_heading(&new_x) ;
    t->x = new_x ;
    return new_x ;
}

/*-----------------------------------------------------------------------*
  executes the general trajectory generator for one time step
  inputs:
      xf is the final, or goal position
      xdotmax is the max desired rate
  outputs:
      the trajectory structure will have updated x and xdot elements
 *-----------------------------------------------------------------------*/

double do_traj( double xf, double xdotmax, traj_t *t )
{
    double dx, new_x, last_x, speed;

    speed = fabs(xdotmax);
    dx = xf - t->x ;
    last_x = t->x ;

    t->xdot = t->alpha*t->xdot + t->beta*speed*trajsgn(dx);

    if(fabs(dx/t->tau) > speed)
	new_x = last_x + t->xdot*t->dt ;
    else
    {
	new_x = t->alpha*last_x + t->beta*xf ;
	t->xdot = (new_x-t->x)/t->dt ;
    }
    t->x = new_x ;
    return new_x;
}
