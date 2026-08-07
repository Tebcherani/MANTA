/*----------------------------------------------------------------------*
   Copyright (C) 1994-1997, Massachusetts Institute of Technology.
	 Sea Grant Autonomous Underwater Vehicles Laboratory.
			 All rights reserved.
  $Id: nav_math.h,v 2.21 1998/01/20 00:31:51 bamoran Exp $
  This file based on ABE LBL navigator from D. R. Yoerger
 *----------------------------------------------------------------------*/

#ifndef NAV_MATH_H
#define NAV_MATH_H

typedef struct
{
    double x,y,z ;
} vector_t ;

typedef enum {NAV_OFF, NAV_GOOD, NAV_STALE, NAV_TIMED_OUT} fix_status_t ;

typedef struct
{
    double range, bearing, range_rate ;
    vector_t position ;
    double time, last_time ;
    int new, num_fixes ;
    fix_status_t fix_status ;
} fix_t ;

double sgn(double x);
void copy_vector(vector_t *dest, vector_t *source);
double det3(double a[][3]);
double oline(vector_t *x0, vector_t *x1, vector_t *x) ;
double median(double x[],int n) ;
double atan2_fix(double x, double y);
int matinv3(double a[][3], double ainv[][3], double min_det, double *pdet);

#endif /* NAV_MATH_H */