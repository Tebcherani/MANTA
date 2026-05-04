/*----------------------------------------------------------------------*
   Copyright (C) 1994-1998, Massachusetts Institute of Technology.
	 Sea Grant Autonomous Underwater Vehicles Laboratory.
			 All rights reserved.
  $Id: whoi_traj.h,v 2.21 1998/01/20 00:31:51 bamoran Exp $
 *----------------------------------------------------------------------*/
/*-----------------------------------------------------------------------*
 | File: whoi_traj.h
 | Authors: Dana Yoerger, Franz Hover of WHOI.
 | Created: 3-11/96
 *-----------------------------------------------------------------------*/

#ifndef WHOI_TRAJ_H
#define WHOI_TRAJ_H

typedef enum { FALSE, TRUE } bool_t;

typedef struct
{
    bool_t init;
    double dt, tau, alpha, beta, x, xdot;

} traj_t;

#ifndef PI
#define PI 3.14159
#endif

#ifndef TWOPI
#define TWOPI (2*PI)
#endif

void init_traj( traj_t *traj, double x, double tau, double dt );
double do_traj( double xf, double xdotmax, traj_t *traj );
double do_hdg_traj( double xf, double xdotmax, traj_t *traj );

#endif /* WHOI_TRAJ_H */
