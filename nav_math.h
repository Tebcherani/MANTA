/*----------------------------------------------------------------------*
   Copyright (C) 1994-1997, Massachusetts Institute of Technology.
	 Sea Grant Autonomous Underwater Vehicles Laboratory.
			 All rights reserved.
  $Id: nav_math.h,v 2.21 1998/01/20 00:31:51 bamoran Exp $
  This file based on ABE LBL navigator from D. R. Yoerger
 *----------------------------------------------------------------------*/

#ifndef NAV_MATH_H
#define NAV_MATH_H

#include "odyssey_nav.h"

double sgn(double x);
double det3(double a[][3]);
double median(double x[],int n) ;
double atan2_fix(double x, double y);
int matinv3(double a[][3], double ainv[][3], double min_det, double *pdet);
void copy_vector(vector_t *dest, vector_t *source);

#endif /* NAV_MATH_H */