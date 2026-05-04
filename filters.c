/*-----------------------------------------------------------------------*
   Copyright (C) 1997-1998, Massachusetts Institute of Technology.
	 Sea Grant Autonomous Underwater Vehicles Laboratory.
			 All rights reserved.
  $Id: filters.c,v 1.1 1998/01/20 00:27:31 bamoran Exp $
 *-----------------------------------------------------------------------*/

#include <float.h>
#include <stdlib.h>

#include "filters.h"

/*-----------------------------------------------------------------------*
  RingBuf functions that initialize, add values to, and access the current
  value in the RingBuf_t structure.
 *-----------------------------------------------------------------------*/

void RB_Init( RingBuf_t *rb, double *b, int size )
{
    rb->buf = b;
    rb->bufsiz = size;
    rb->curpos = 0;
}

int RB_AddVal( RingBuf_t *rb, double val )
{
    rb->buf[(rb->curpos = ++rb->numval%rb->bufsiz)] = val;
    return rb->curpos;
}

double RB_CurVal( RingBuf_t *rb )
{
    return rb->buf[rb->curpos];
}


/*-----------------------------------------------------------------------*
  RB_muFilt: RingBuf traversal function that (1) uses 'accept' to validate
  each element, (2) accumulates the result of applying 'el_apply' to each
  valid element and (3) applies 'acc_apply' to the final accumulation.  Use
  NULL in place of any of these arguments in order to accept all elements
  in their raw form and return the raw accumulation to the calling
  function.  The 'pOK' argument will hold the valid element count.  A NULL
  ptr here causes the accumulation to be divided by the valid count.  See
  RB_TEST below for calling examples to calculate mean, variance, rms.

  B. A. Moran, 11-Jan-1998
 *-----------------------------------------------------------------------*/

double RB_muFilt( RingBuf_t *rb, int *pOK, int (*accept)(double),
		  double (*el_apply)(double),
		  double(*acc_apply)(double) )
{
    register int i, numOK = 0;
    double accum = 0;

    for ( i = 0; i < rb->bufsiz; i++ )
    {
	if ( accept && !accept(rb->buf[i]) )
	    continue;
	accum += el_apply ? el_apply(rb->buf[i]) : rb->buf[i];
	numOK++;
    }
    if ( !pOK )
	accum /= numOK;
    else
	*pOK = numOK;

    return acc_apply ? acc_apply(accum) : accum;
}

/*-----------------------------------------------------------------------*
  Self test of RB loading occurs by compiling this file standalone with a
  compiler flag similar to -DRB_TEST=10, which both includes the code below
  and defines the max buffer size.
 *-----------------------------------------------------------------------*/

#if RB_TEST
#include <math.h>
#include <stdio.h>

double buf[RB_TEST];
double mean;

main( int argc, char **argv )
{
    RingBuf_t rb;
    double var, rms, sqrerr( double );
    int i, c, nOK, size = RB_TEST, nonzero( double );

    if ( argc > 1 )
    {
	size = atoi( argv[1] );
	if ( !size || size > RB_TEST ) size = RB_TEST;
    }
				/* init RingBuf with runtime size */
    RB_Init( &rb, buf, size );
				/* add values */
    for ( i = 0; i < size; i++ )
    {
	c = RB_AddVal( &rb, i );
	printf( "% 3d: %.1f\n", i, buf[c] );
    }
				/* run the filter */
    mean = 0;
    rms  = RB_muFilt( &rb, NULL, NULL, sqrerr, sqrt);
    mean = RB_muFilt( &rb, NULL, NULL, NULL, NULL );
    var  = RB_muFilt( &rb, NULL, NULL, sqrerr, NULL);
    printf( "Avg = %.3g, Var = %.4g (%.4g), RMS = %.4g\n",
	    mean, var, sqrt(var), rms );

    exit( 0 );
}

double sqrerr( double val )
{
    return (val - mean)*(val - mean);
}

int nonzero( double val )
{
    if ( val != 0 )
	return 1;
    return 0;
}

#endif
