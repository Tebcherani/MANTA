/*----------------------------------------------------------------------*
   Copyright (C) 1994-1997, Massachusetts Institute of Technology.
	 Sea Grant Autonomous Underwater Vehicles Laboratory.
			 All rights reserved.
  $Id: ekf.h,v 2.21 1998/01/20 00:31:51 bamoran Exp $
 *----------------------------------------------------------------------*/

#include <stdlib.h>

/********** axis codes for rotation matrices *******/
#define X_AXIS 0
#define Y_AXIS 1
#define Z_AXIS 2

/*************these for mat_add and qmat_add.  ZERO is a bit weird***/
#define PLUS 1.0
#define MINUS -1.0
#define ZERO 0.0

/*******************these for lud stuff****************/
#define TINY (1.0e-20)
#define SMALL (1.0e-10)

/*********************general matrix types ***************/
typedef double *dvec;
typedef int *ivec;
typedef double **dmat;

typedef struct matrix
{
   int r;
   int c;
   dmat mat;
} *matrix_t;


/*******************extern hack ***************/

/* #define EXTERN extern   *** remo jleonard 01/13/96 */
#define TRUE        -1
#define FALSE        0

/***************important matrix macro************/

#define e(m,i,j) m->mat[i][j]

/***************external matrix stuff**************/
extern void cberrorN (char *msg, int n);
extern void cberror (char *msg);

extern int lubksb (double **a, int n, int *indx, double *b);
extern int ludcmp (double **a, int n, int *indx, double *d);
extern void dvec_free (dvec v);
extern void ivec_free (ivec v);
extern void mat_free (matrix_t m);

extern int qmat_add (matrix_t a,
                        matrix_t b,
                        matrix_t c,
                        int row,
                        int col,
                        double sign);
extern int qmat_const (matrix_t m1, double con);
extern int qmat_scale (matrix_t m1, double s);
extern int qmat_scalmpy (matrix_t m1, matrix_t m2, double s, int r, int c);
extern int qmat_cpy (matrix_t m1, matrix_t m2, int r, int c);
extern int qmat_inv (matrix_t m, matrix_t mi, int n);
extern int qmat_trans (matrix_t a, matrix_t b, int r, int c);
extern int qmat_rot (matrix_t rot, int axis, double theta);
extern int qmat_xlate (matrix_t xlate, double x, double y, double z);
extern int qmat_mpy (matrix_t a,
                        matrix_t b,
                        matrix_t C,
                        int r1,
                        int c1,
                        int c2);

extern void ivec_pr (ivec vec, int n);
extern int mat_pr (char *msg, matrix_t m);

extern double quad_form (matrix_t v, matrix_t m);
extern double qquad_form (matrix_t m, matrix_t v, int dim);
extern double determ (matrix_t m);
extern double dot_prod (matrix_t v1, matrix_t v2);

extern ivec ivector (int n);

extern matrix_t mat_add (matrix_t a, matrix_t b, double sign);
extern matrix_t mat_new_diag (int n, double v);
extern matrix_t mat_cpy (matrix_t m);
extern matrix_t mat_mpy (matrix_t a, matrix_t b);
extern matrix_t mat_trans (matrix_t a);
extern matrix_t mat_rd (char *msg);
extern matrix_t mat_new (int r, int c);
extern matrix_t mat_inv (matrix_t m);

extern dvec dvector (int n);

/*********************external random routines ******************/
extern void set_seed (void);
extern double N (double m, double std);
extern double U (double m, double std);
extern double UR (double m, double range);
extern double C (double p, double m1, double std1, double m2, double std2);

/*****************basics ****************/
#define max(x,y) x>y?x:y
#define min(x,y) x>y?y:x
/* #define abs(x) x>0?x:-x */

#define deg_to_rad ( (double) M_PI/180.0)

/****************error codes***************/
#define OK 1
#define        NILARG1 -1
#define NILARG2 -2
#define NILARG3 -3
#define NILARG4 -4
#define SINGULAR -5
#define NONSQUARE -6
#define NONCONFORM -7

#define MAX_XDIM 7 /* north, east, depth, roll, pitch, yaw, speed */
#define MAX_ZDIM 6 /* north, east, depth, roll, pitch, yaw */

