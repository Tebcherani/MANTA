/*-----------------------------------------------------------------------*
   Copyright (C) 1997-1998, Massachusetts Institute of Technology.
	 Sea Grant Autonomous Underwater Vehicles Laboratory.
			 All rights reserved.
  $Id: filters.h,v 1.1 1998/01/20 00:27:18 bamoran Exp $
 *-----------------------------------------------------------------------*/

#ifndef _FILTERS_H_
#define _FILTERS_H_

typedef struct {

    double *buf;		/* location of buffer */
    int bufsiz;			/* size of buffer */

    int curpos;			/* current position (most recent) */
    int numval;			/* value counter (always growing) */

} RingBuf_t;

				/* function prototypes */
void RB_Init( RingBuf_t *rb, double *b, int size );
int RB_AddVal( RingBuf_t *rb, double val );
double RB_CurVal( RingBuf_t *rb );
double RB_muFilt( RingBuf_t *rb, int *numOK, int (*accept)(double),
		  double (*el_apply)(double), double(*acc_apply)(double) );

#endif /* _FILTERS_H */
