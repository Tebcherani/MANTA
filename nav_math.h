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
void init_fix(fix_t *fix);
void copy_vector(vector_t *dest, vector_t *source);
double shape(double x, double y, int n, double x0, double y0, double rx, double ry) ;
void init_rls2_matrices(double psi[], double R[2][2], double Theta[]) ;
void update_rls2_soln(double R[2][2], double Theta[], int n, double gamma,
		double psi[], double y, double *perror);
double det3(double a[][3]);
void init_rls3_matrices(double R[][3], double Theta[]) ;
void update_rls3_soln(double R[][3], double Theta[], int n, double gamma,
		double psi[], double y, double *perror) ;
void init_ls3_matrices(double AtA[][3], double AtB[]) ;
void update_ls3_matrices(double a[], double b,double AtA[][3], double AtB[]);
int solve_ls3(double AtA[][3], double AtB[], double x[], double *pdet);
double ls3_error(double Theta[], double x[], double y) ;
double oline(vector_t *x0, vector_t *x1, vector_t *x) ;
double median(double x[],int n) ;
double atan2_fix(double x, double y);
void nav2_depth(vector_t *pa, vector_t *pb, double ra, double rb, double depth,
		int nside, vector_t *pd);
double gauss2d(double x, double y) ;
int matinv3(double a[][3], double ainv[][3], double min_det, double *pdet);
int matinv2(double a[2][2], double ainv[2][2], double min_det, double *pdet);
#ifdef LBLSIM
double gasdev(long int *idum) ;
#else
double gasdev(int *idum) ;
#endif
void read_ss(char *filename, double dsv[], double sv[], int *nsv);
double cal_cbar(double dsv[], double sv[], int nsv, double d_a,double d_b);

#endif /* NAV_MATH_H */