/*----------------------------------------------------------------------*
   Copyright (C) 1994-1997, Massachusetts Institute of Technology.
	 Sea Grant Autonomous Underwater Vehicles Laboratory.
			 All rights reserved.
  $Id: lbl.h,v 2.21 1998/01/20 00:31:51 bamoran Exp $
  This file based on ABE LBL navigator from D. R. Yoerger
 *----------------------------------------------------------------------*/

#include "odyssey_nav.h"

#ifndef LBL_H
#define LBL_H

#define XP_A 0
#define XP_B 1
#define XP_C 2

int num_good(lbl_t *lbl);
int lbl_inc_index(lbl_t *l);
int lbl_set_baseline(char *string,lbl_t *l);
void baseline_string(lbl_t *lbl, char string[]);
int lbl_set_side(char *string,lbl_t *l);
void compute_lbl_fix(double r[],lbl_t *lbl, double t, fix_t *lbl_fix,
	 int noisy);
#define LBL_RNOISE 1.0

/* defines sonar properties*/

#define ALPHA   PI/3    
/*mounting angle from the horizontal plane [60deg]*/

#define BETA    PI/10   
/*beam width of the robertson [18deg]*/

#define MAX_RANGE       40      
/*maximum range of the robertson [40m] */

/* defines map properties */
/* grid cell's size [m per cell] */
#define CELL_SIZE       0.25    

/*map length*/
#define MAP_L   100

/* locus of the sonar in the map [map units]*/
#define SONAR_I 75
#define MASS 1000.0
#define BB 500.0
#define DT 1.0
/* stern and bow of the vehcle in grid coordinates */
#define VEHICLE_BOW  75
#define VEHICLE_STERN 93
typedef struct{
	double y_min, y_max ;
	double force_x, force_y ;
	double depth_new, depth_old, depth_dot ;
	double rnd_error ;
	double time_new, time_old ;
	double speed, dt ;
	int direction ;
	double range ;
	int counts ;
	double x_break ;
	double q1, q2 ;
	double bfloor, bceiling ;
	double a, b, r1, r2 ;
	int k1, k2;
	double coef ;
	double max_depth_dot ;
	double map[MAP_L] ;
	int enforce_ceiling ;
} bottom_follow_t ;
void init_bottom(bottom_follow_t *b);
void ad_depth1(double d_depth, double map[]);
void incl_rng1(double range,double map[]);
void make_force1(double x, double y, bottom_follow_t *b);
void move_map1(double  speed, double dt, double  *pt_rnd_error, double map[]) ;
void sum_force1(bottom_follow_t *b);
void do_ralf(bottom_follow_t *b, double t, double *pdepth_set);
range_status_t check_range(double r, double t, int channel, lbl_t *lbl);
void make_fix(lbl_t *lbl, double t, fix_t *fix);
void get_indices(lbl_t *lbl);
void chside(vector_t *xhat, lbl_t *lbl);
void ABmat(double r[], lbl_t *lbl, double A[NXP][3], double B[]);
void psuedo3(double A[NXP][3], double B[NXP], double x[]);

#endif /* LBL_H */