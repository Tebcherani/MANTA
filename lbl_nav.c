/*----------------------------------------------------------------------*
   Copyright (C) 1994-1997, Massachusetts Institute of Technology.
	 Sea Grant Autonomous Underwater Vehicles Laboratory.
			 All rights reserved.
  $Id: lbl_nav.c,v 2.22 1998/02/03 04:20:11 bamoran Exp $
 *----------------------------------------------------------------------*/

#include <math.h>
#include <stdio.h>		/* ANSI */
#include <stdlib.h>
#include <string.h>

#include "ody_defs.h"
#include "diagnostic.h"
#include "nav_math.h"
#include "lbl.h"
#include "vehicle.h"
#include "odyssey_log.h"

int  lsquick2( double r[], lbl_t *lbl, double *perror );
int  findwrst( lbl_t *lbl, double r[] );
int  nav2_xy( vector_t *pa, vector_t *pb, double ra, double rb, lbl_side_t side, double min_angle, vector_t *pos );

/*----------------------------------------------------------------------*
  New LBL navigation using Dana Yoerger's ABE algorithm installed on CCB97
  cruise aboard R/V Cape Hatteras 15-May-1997.
 *----------------------------------------------------------------------*/

void LBL_position( void )
{
    void ABE_LBL( void );

    if ( sensor(U_LBL_BEACON_NUMBER) < 2 )
    {
					/* best guess of vehicle position */
	update_add( M_POS_N, sensor(M_DR_DN) );
	update_add( M_POS_E, sensor(M_DR_DE) );

	return;
    }

    ABE_LBL();
}

#include "nav_math.h"
#include "lbl.h"

/*----------------------------------------------------------------------*
  Simple LBL position that uses median filters to validate the raw returns
  and least squares to calculate an estimated position based on only the
  validated returns.  Handles 2 range solutions using logic to determine
  which side of baseline.  Uses DR for baseline crossings.

  Originally by D. R. Yoerger for ABE.

  B. A. Moran, 08-May-1997
 *----------------------------------------------------------------------*/

static double slant[NXP];	/* slant ranges derived from tt's */

void ABE_LBL()
{
    fix_t fix;			/* result of computation */
    static lbl_t lblenv;	/* LBL parameters: xpndrs, sound speed,
				   ring buffers, max ranges, pos est,
				   convergence parms */
    static int uninitialized = 1;
    static int numXP;		/* number active transponders */
    double dx, dy;
    int ii, jj, offset, numGood;

    if ( uninitialized )
    {
				/* store lbl environment */
	lblenv.range_tol = sensor(LBL_RANGE_TOL);
	lblenv.fix_tol = sensor(LBL_FIX_ERR);
	lblenv.timeout = 99999;
	lblenv.min_angle = sensor(LBL_MIN_ANGLE);
	lblenv.alpha = 0.5;
	lblenv.num_steps_pinv = 10;
	lblenv.max_iter = 40;
				/* store beacon count as static int */
	numXP = (int) sensor(U_LBL_BEACON_NUMBER);
				/* xponder data for fix calcs */
	for ( ii = 0, offset = 0; ii < numXP; ii++ )
	{
	    lblenv.xp[ii].x = sensor(U_LBL_1NORTH + offset);
	    lblenv.xp[ii].y = sensor(U_LBL_1EAST  + offset);
	    lblenv.xp[ii].z = sensor(U_LBL_1DEPTH + offset);
	    lblenv.tat[ii]  = sensor(U_LBL_1TAT   + offset);
	    lblenv.max_range[ii] = 4000;
	    offset += U_LBL_2NORTH - U_LBL_1NORTH;
	    fprintf( syslog, "\tX%d: N %.1f E %.1f D %.1f T %.3g\n",
		     ii + 1, lblenv.xp[ii].x, lblenv.xp[ii].y,
		     lblenv.xp[ii].z, lblenv.tat[ii] );
	    for ( jj = 0; jj < LBL_NUM_RANGES; jj++ )
	    {
		lblenv.range[ii][jj] = -100;
		lblenv.time[ii][jj]  = -100;
	    }
	    lblenv.good_time[ii] = -100;
	    lblenv.good[ii] = RANGE_OLD;
	}
				/* initialize Dana's struct to zero */
	memset( &fix, 0, sizeof(fix_t) );
	uninitialized = 0;
    }
				/* calculate DR updates, will be used
				   in presence or absence of LBL hit */
    dx = sensor( M_DR_DN );
    dy = sensor( M_DR_DE );

    if ( sensor(LBL_NEWDATA) > 0 )
    {
	for ( ii = 0; ii < numXP; ii++ )
	{
	    if ( sensor(M_LBL_1T + ii) > 0 )
	    {
		slant[ii] = 0.5*(sensor(M_LBL_1T + ii) - lblenv.tat[ii]);
		slant[ii] *= sensor(U_SPEED_OF_SOUND);
	    }
	    else
		slant[ii] = 0.;
	}

	lblenv.depth  = sensor( M_DEPTH );
	lblenv.xhat.x = sensor( M_POS_N );
	lblenv.xhat.y = sensor( M_POS_E );

	sprintf( msg_buf, "% 8.2f: LBL inputs\n"
		 "\t(N%7.1f E%7.1f D %6.1f)\n", sensor(M_PRESENT_TIME),
		 sensor(M_POS_N), sensor(M_POS_E), sensor(M_DEPTH) );

	compute_lbl_fix( slant, &lblenv, sensor(M_PRESENT_TIME), &fix, 1 );

				/* log the fix and its status */
	update( LBL_FIX_ERR, lblenv.error );
	update( LBL_FIXSTAT, fix.fix_status );

				/* indicate we've used the data */
	update( LBL_NEWDATA, 0 );

				/* count and log good hits */
	for ( ii = 0, numGood = 0; ii < numXP; ii++ )
	{
	    if ( lblenv.good[ii] == RANGE_GOOD )
	    {
		numGood++;
		update( M_LBL_VALID1 + ii, 1 );
	    }
	    else
		update( M_LBL_VALID1 + ii, 0 );
	}

	update( M_LBL_NUM_USED, numGood );

	sprintf( msg_buf+strlen(msg_buf), "\tstat %d, good %d\n",
		 fix.fix_status, numGood );

	if ( fix.fix_status == NAV_GOOD )
	{
	    update( M_LBL_NORTHFIX, fix.position.x );
	    update( M_LBL_EASTFIX,  fix.position.y );

	    dx += sensor(LBL_FIXGAIN)*(sensor(M_LBL_NORTHFIX) - 
				       sensor(M_POS_N));
	    dy += sensor(LBL_FIXGAIN)*(sensor(M_LBL_EASTFIX) - 
				       sensor(M_POS_E));

	    sprintf( msg_buf+strlen(msg_buf)-1,
		     ", err %.1f (N%7.1f E%7.1f)\n", 
		     lblenv.error, fix.position.x, fix.position.y );
	}
	write_message( MonitorLBL );
    }
    update_add( M_POS_N, dx );
    update_add( M_POS_E, dy );

    return;
}

void compute_lbl_fix( double rxyz[], lbl_t *lbl, double t, fix_t *fix,
		      int noisy )
{
    double r[NXP], dz;
    int i, i0,i1, ngood, iworst, result;

    lbl_inc_index(lbl) ;
    lbl->new_fix = 0;
				/* convert from slant ranges into xy ranges */
    for(i=0;i<NXP;i++)
    {
	dz = lbl->xp[i].z - lbl->depth ;
	r[i] = 0.0 ;
	if( rxyz[i] > fabs(dz) )
	    r[i] = sqrt(rxyz[i]*rxyz[i] - dz*dz) ; 
    }
    lbl->position[lbl->index].z = lbl->depth ;
    /* ranges are declared not good if they are: too long, fail the median
       filter test, they are too old */
    for (i=0; i < NXP; i++)
    {
	lbl->good[i]=check_range(r[i],t,i,lbl) ;
	if( (t-lbl->good_time[i]) > lbl->timeout)
	    lbl->good[i] = RANGE_OLD ;
    }
    lbl->status[lbl->index] = 0 ;
				/* try to get a soln with more than 2 ranges */
    while( (ngood=num_good(lbl)) > 2)
    {
	lsquick2(r,lbl,&(lbl->error));
				/* set the strarting point for ls_soln to
                                   the lsquick2 solution */

		/* if the error is below the set tolerance, soln is good */
	if(lbl->error < lbl->fix_tol)
	{
	    lbl->lost = 0 ;	/* definitely not lost! */
	    lbl->new_fix = lbl->status[lbl->index] = 1 ;
	    lbl->baseline.i0=lbl->baseline.i1=lbl->side=0;
				/* we believe in LS fixes that meet the
                                   error criteria ! */
	    lbl->xhat.x = lbl->position[lbl->index].x ;
	    lbl->xhat.y = lbl->position[lbl->index].y ;
#ifdef LBLPRT
	    if(noisy)printf("good soln with %d ranges\n",ngood) ;
#endif
	    break ;
	}
	else
	{
	    iworst = findwrst(lbl,r); /* pick worst range based on xhat */
	    lbl->good[iworst] = RANGE_HIGH_ERROR ;  /* make it not good */
#ifdef LBLPRT
	    if(noisy)printf("range %d is bad\n",iworst) ;
#endif
	}
	/* the while loop will exit if ngood is not > 2 */	
    }
				/* only do the 2-range soln if not lost */
    if( (ngood == 2) && (!lbl->lost))
    {
#ifdef LBLPRT
	if(noisy)printf("computing 2 range solution ") ;
#endif
				/* get the indices of the first two good
                                   ranges */
	get_indices(lbl) ;
				/* determine the baseline side based on
                                   xhat */
	chside(&(lbl->xhat), lbl) ;
	i0 = lbl->baseline.i0 ;
	i1 = lbl->baseline.i1 ;
#ifdef LBLPRT
	if(noisy)printf("2 range soln with xpndrs %d %d ranges %.1lf %.1lf\n",
			i0,i1,r[i0],r[i1]) ;
#endif
		/* crunch the 2-range soln, it returns false if you're too
			close to the baseline */
	if((result=nav2_xy(lbl->xp+i0, lbl->xp+i1,
			   lbl->good_range[i0], lbl->good_range[i1], 
			   lbl->side,lbl->min_angle,
			   &(lbl->position[lbl->index])))==1)
	{
	    lbl->new_fix = lbl->status[lbl->index] = 1 ;
	}
	else 
	{
	    lbl->new_fix =  0;
	    lbl->status[lbl->index] = result ;
	    lbl->side = ON_BASELINE;
	}
    }
    if(lbl->new_fix)
    {
	make_fix(lbl,t,fix) ;
	fix->num_fixes++ ;
	fix->fix_status = NAV_GOOD ;
    }
    else
    {
	fix->new = 0 ;
	fix->fix_status = NAV_STALE ;
    }
    if(fix->time-fix->last_time > lbl->timeout)fix->fix_status = NAV_TIMED_OUT ;
#ifdef LBLPRT
    if(noisy)
    {
	printf("index: %d\n",lbl->index) ;
	printf("     ra     rb     rc    rd     x       y      z     ta     tb     tc  status\n") ;
	for(i=0;i<LBL_NUM_RANGES;i++)
	    printf("%d %6.1lf %6.1lf %6.1lf %6.1lf %6.1lf %6.1lf %6.1lf %6.1lf %6.1lf %6.1lf   %d\n",i,
		   lbl->range[0][i],lbl->range[1][i],lbl->range[2][i],lbl->range[3][i],
		   lbl->position[i].x,lbl->position[i].y,lbl->position[i].z,	
		   lbl->time[0][i],lbl->time[1][i],lbl->time[2][i],lbl->status[i]) ;
    }
#endif
}

int lbl_inc_index(lbl_t *lbl)
{
    (lbl->index)++ ;
    return(lbl->index = lbl->index%LBL_NUM_RANGES) ;
}


/*----------------------------------------------------------------------*
  Dana uses a median filter that 
 *----------------------------------------------------------------------*/

#ifdef RDOT_CHECK
#else
range_status_t check_range(double r, double t, int channel, lbl_t *lbl)
{
    double med ;
    int result ;
    if(r < 1.0)result=RANGE_OLD ;
    else
    {
	if(r > lbl->max_range[channel])result = RANGE_TOO_LONG ;
	else
	{
	    med = median(lbl->range[channel],LBL_NUM_RANGES) ;
	    if( fabs(med-r) < lbl->range_tol)
	    {
		lbl->good_range[channel] = r ;
		lbl->good_time[channel] = t ;
		result = RANGE_GOOD ;
	    }
	    else result = RANGE_FAILED_MEDIAN ;
	}
    }
				/* ranges and times go in whether they pass
                                   the tests or not */
    lbl->time[channel][lbl->index] = t ;
    lbl->range[channel][lbl->index] = r ;
    return(result) ;
}
#endif
	
void make_fix(lbl_t *lbl, double t, fix_t *fix)
{
    copy_vector(&(fix->position),&(lbl->position[lbl->index])) ;
    fix->last_time = fix->time ;
    fix->time = t ;
    fix->new = 1 ;
}

/* returns the number of ranges that have been judged good */
int num_good(lbl_t *lbl)
{
    int i, result ;
    result = 0;
    for (i=0;i<NXP;i++)if(lbl->good[i] == RANGE_GOOD)result++ ;
    return(result);
}

/* finds the indices of the first two good ranges*/
void get_indices(lbl_t *lbl)
{
    int i ;

    for (i=0;i<NXP;i++)
    {
	if(lbl->good[i] == RANGE_GOOD)
	{
	    lbl->baseline.i0 = i ;
	    break ;
	}
    }
    for( i = lbl->baseline.i0+1; i < NXP; i++)
    {
	if(lbl->good[i] == RANGE_GOOD)
	{
	    lbl->baseline.i1 = i ;
	    break ;
	}
    }
}

int lsquick2(double r[], lbl_t *lbl, double *perror)
{
    int ngood, result, ind, i ;
    double A[NXP][3], B[NXP], x1[3], rtest[NXP] ;
    double x, y, dx, dy, dr, error;
    ngood = num_good(lbl) ;
    if(ngood	 < 3)
    {
	result = -1 ;
	return(result) ;
    }

    ABmat(r,lbl,A,B) ;
    psuedo3(A,B,x1);
    ind = lbl->index ;
    x=lbl->position[ind].x = x1[1] ;
    y=lbl->position[ind].y = x1[2] ;
    result = ngood;
    error = 0.0 ;
    for(i=0;i<NXP;i++)
    {
	dx = x-lbl->xp[i].x ;
	dy = y-lbl->xp[i].y ;
	rtest[i] = sqrt(dx*dx + dy*dy) ;
	dr = r[i]-rtest[i];
	if(lbl->good[i] == RANGE_GOOD)error += dr*dr ;
    }
    error = sqrt(error/(double)ngood) ;
    *perror = error ;
    return(1) ;
}

/*
returns the index of the range that disagrees with the estimated
position by the largest amount
*/
int findwrst(lbl_t *lbl,double r[])
{
    double dx, dy, biggest, rdiff[NXP], rtest[NXP] ;
    int i,iw ;
    for(i=0;i<NXP;i++)
    {
	dx = lbl->xhat.x-lbl->xp[i].x ;
	dy = lbl->xhat.y-lbl->xp[i].y ;
	rtest[i] = sqrt(dx*dx + dy*dy) ;
	rdiff[i] = (lbl->good[i] == RANGE_GOOD) ? fabs(rtest[i]-r[i]) : 0.0;
    }
    iw = 0 ;
    biggest = rdiff[0] ;
    for(i=1;i<NXP;i++)
    {
	if(rdiff[i] > biggest)
	{
	    iw = i ;
	    biggest = rdiff[i];
	}
    }
    return(iw) ;
}

/*
this routine is used when 2 xpndr nav is being used.
if changes the baseline side based on an estimated position
contained in xhat
*/
void chside(vector_t *xhat, lbl_t *lbl)
{
    int i0, i1 ;
    double distance_from_baseline ;
    i0 = lbl->baseline.i0 ;
    i1 = lbl->baseline.i1 ;
    distance_from_baseline = oline(&(lbl->xp[i0]), &(lbl->xp[i1]), xhat);
    lbl->side = (distance_from_baseline < 0.0) ? CW : CCW ;
}

/* 
computes a deterministic xy solution from 2 ranges, the two
xpndr positions, the two ranges, and the baseline side
must be provided
*/
int nav2_xy( vector_t *pa, vector_t *pb, double ra, double rb,
	     lbl_side_t nside, double min_angle, vector_t *pd )
{

    double ra2,rb2 ;
    double x0,y0,dx,dy,theta ;
    double s,c,rab,rab2 ;
    double xa, ya, xb, yb, maga, magb, x2, test ;
    double arg;
				/* net rotation parameters */
    dx = pb->x-pa->x ;
    dy = pb->y-pa->y ;
    theta = atan2_fix(dy,dx) ;
    s = sin(theta) ;
    c = cos(theta) ;

    ra2 = ra*ra ;
    rb2 = rb*rb ;

				/* compute solution in net coordinates */
    rab2 = dx*dx+dy*dy ;
    rab = sqrt(rab2) ;
    if (rab2==0) x0=0;
    else x0 = (rab2+ra2-rb2)/(2.0*sqrt(rab2));
    arg=ra2-x0*x0;

    if (arg<0.0)return(-1);
    y0 = sqrt(arg) ;
				/* determine the cross product of the unit
                                   vectors pointing at the xpndrs */
    maga = sqrt(x0*x0 + y0*y0) ;
    if(maga < 1.0)return(-2) ;
    xa = x0/maga ;
    ya = y0/maga ;
    x2 = x0-rab ;
    magb = sqrt(x2*x2 + y0*y0) ;
    if(magb < 1.0)return(-3) ;
    xb = x2/magb ;
    yb = y0/magb ;
    test = xa*yb - xb*ya ;
				/* angle has to be between min_angle and
                                   pi-min_angle */
    if(fabs(test) < sin(min_angle))return(-4) ;
    if(nside == CW)y0 = -y0 ;

				/* transform to world coordinates */
    pd->x = pa->x + c*x0 - s*y0 ;
    pd->y = pa->y + s*x0 + c*y0 ;
    return(1) ;
}

void ABmat(double r[], lbl_t *lbl, double A[NXP][3], double B[])
{
    int i ;
    double good, xpx, xpy ;
    for(i = 0; i < NXP;i++)
    {
	good = (lbl->good[i] == RANGE_GOOD) ? 1.0 : 0.0;
	xpx = lbl->xp[i].x ;
	xpy = lbl->xp[i].y ;
	A[i][0] = good ;
	A[i][1] = -2.0*good*xpx ;
	A[i][2] = -2.0*good*xpy ;
	B[i] = good*(r[i]*r[i] - xpx*xpx - xpy*xpy);
    }
}
void psuedo3(double A[NXP][3], double B[NXP], double x[])
{
    double AtA[3][3], AtAinv[3][3], AtB[3], det ;
    int i, j, k ;
				/* combined loop AtA, AtB */
    for(i=0;i<3;i++)
    {
				/* compute Atranspose*A */
	for(j=0;j<3;j++)
	{
	    AtA[i][j] = 0.0 ;
	    for(k=0;k<NXP;k++)
		AtA[i][j] += A[k][i]*A[k][j];
	}
				/* compute Atranspose*B */
	AtB[i] = 0.0 ;
	for(j=0;j<NXP;j++)
	    AtB[i] += A[j][i]*B[j];
    }
				/* invert Atranspose*A */
    matinv3(AtA,AtAinv,.00001,&det) ;
				/* x = inv(At*A)*At*B */
    for(i=0;i<3;i++)
    {
	x[i] = 0.0 ;
	for(j=0;j<3;j++)
	    x[i] += AtAinv[i][j]*AtB[j];
    }
}

void copy_vector( vector_t *dest, vector_t *source )
{
				/* why not memcpy? */
    dest->x = source->x ;
    dest->y = source->y ;
    dest->z = source->z ;
}

double det3(double a[][3])
{
    double result ;
    result = a[0][0] * a[1][1] * a[2][2] +
	a[0][1] * a[1][2] * a[2][0] +
	a[0][2] * a[1][0] * a[2][1] -
	a[0][2] * a[1][1] * a[2][0] -
	a[0][0] * a[1][2] * a[2][1] -
	a[0][1] * a[1][0] * a[2][2];
    return(result) ;
}

int matinv3(double a[][3], double ainv[][3], double min_det, double *pdet)
{
    double det ;
    int i;
    int j;
    int result;

    *pdet = det = det3(a);

    if (fabs(det) > min_det) {
	result = 1;
	ainv[0][0] =  (a[1][1] * a[2][2] - a[1][2] * a[2][1]) / det;
	ainv[1][0] = -(a[1][0] * a[2][2] - a[1][2] * a[2][0]) / det;
	ainv[2][0] =  (a[1][0] * a[2][1] - a[1][1] * a[2][0]) / det;
	ainv[0][1] = -(a[0][1] * a[2][2] - a[0][2] * a[2][1]) / det;
	ainv[1][1] =  (a[0][0] * a[2][2] - a[0][2] * a[2][0]) / det;
	ainv[2][1] = -(a[0][0] * a[2][1] - a[0][1] * a[2][0]) / det;
	ainv[0][2] =  (a[0][1] * a[1][2] - a[0][2] * a[1][1]) / det;
	ainv[1][2] = -(a[0][0] * a[1][2] - a[0][2] * a[1][0]) / det;
	ainv[2][2] =  (a[0][0] * a[1][1] - a[0][1] * a[1][0]) / det;
    }
    else {
	result = 0;
	for (i = 0; i < 3; ++i) {
	    for (j = 0; j < 3; ++j) {
		ainv[i][j] = 0.0;
            }
	}
    }
    return result;
}

double atan2_fix(double x, double y)
{
    if ( (fabs(x)<.0001) && (fabs(y)<.0001) ) return (0.0);
    return (atan2(x,y));
}

void sort(int n, double ra[])
{
    int l,j,ir,i;
    double rra;

    l=(n >> 1)+1;
    ir=n;
    for (;;) {
	if (l > 1)
	    rra=ra[--l];
	else {
	    rra=ra[ir];
	    ra[ir]=ra[1];
	    if (--ir == 1) {
		ra[1]=rra;
		return;
	    }
	}
	i=l;
	j=l << 1;
	while (j <= ir) {
	    if (j < ir && ra[j] < ra[j+1]) ++j;
	    if (rra < ra[j]) {
		ra[i]=ra[j];
		j += (i=j);
	    }
	    else j=ir+1;
	}
	ra[i]=rra;
    }
}

double store[10] ;
double median(double x[],int n)
{
    int n2,n2p;
    int i ;
    double result ;
    for(i=0;i<n;i++)store[i+1]=x[i] ;
    sort(n,store);
    n2p=(n2=n/2)+1;
    result=(n % 2 ? store[n2p] : 0.5*(store[n2]+store[n2p]));
    return(result) ;
}

#define IA 16807
#define IM 2147483647
#define AM (1.0/IM)
#define IQ 127773
#define IR 2836
#define NTAB 32
#define NDIV (1+(IM-1)/NTAB)
#define EPS 1.2e-7
#define RNMX (1.0-EPS)

#ifdef LBLSIM
#define LONG long
#else
#define LONG
#endif

double oline(vector_t *x0, vector_t *x1, vector_t *x)
{
    double alpha ;
    alpha = atan2(x1->x-x0->x, x1->y-x0->y) ;
    return(-cos(alpha)*(x->x-x0->x) + sin(alpha)*(x->y-x0->y)) ;
}
