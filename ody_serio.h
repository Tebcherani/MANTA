/*-----------------------------------------------------------------------*
  Copyright (C) 1998, Massachusetts Institute of Technology.
  Proprietary to Sea Grant AUV Laboratory.  All rights reserved.
  $Id: ody_serio.h,v 1.2 1998/02/03 04:25:07 bamoran Exp $
 *-----------------------------------------------------------------------*/

#ifndef ODY_SERIO_H
#define ODY_SERIO_H

#include "ody_serial.h"

typedef struct {

    char *dev;			/* OS-9 device, eg, "/t80" */
    int path;			/* path id or device descriptor */
    baud_t bau;			/* baud rate */
    unsigned char par;		/* device parity */
    unsigned char eor;		/* end of record character */

    char *ibuf;			/* input buffer */
    char *obuf;			/* output buffer */

    unsigned isize, osize;	/* buffer sizes */

    char *base, *curp;		/* ptrs for read append new data */

    unsigned ictr, octr;	/* counters for i/o bytes */
    unsigned ierr, oerr;	/* counters for i/o error */

    int ody_errno;  /* renamed: conflicts with system macro */			/* errno from last error */

} serio_t;

typedef struct {

    char *name;			/* common name, e.g., "Thruster" */
    serio_t *ser;		/* serial device for associated SAIL bus */

    int (*init)( void );	/* functions to handle device */
    int (*start)( void );
    int (*halt)( void );
    int (*exit)( void );

} ody_dev_t;

				/* function prototypes */
int serio_read( serio_t *s );
char *serio_gets( char *s, int n, serio_t *serio );
int serio_flush( serio_t *serbuf );
serio_t *serio_init( serio_t *serbuf, char *buf, unsigned bufsiz );

#endif /* ODY_SERIO_H */
