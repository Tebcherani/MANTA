/*----------------------------------------------------------------------*
   Copyright (C) 1994-1998, Massachusetts Institute of Technology.
	 Sea Grant Autonomous Underwater Vehicles Laboratory.
			 All rights reserved.
  $Id: ody_serial.h,v 2.21 1998/01/20 00:31:51 bamoran Exp $
 *----------------------------------------------------------------------*/

#ifndef ODY_SERIAL_H
#define ODY_SERIAL_H

#define NONE   0		/* parity: None, Odd, Even */
#define ODD    1
#define EVEN   3
#define BIT8   (0<<2)		/* bits per character */
#define BIT7   (1<<2)
#define BIT6   (2<<2)
#define BIT5   (3<<2)
#define STOP1  (0<<4)		/* stop bits */
#define STOP15 (1<<4)
#define STOP2  (2<<4)
#define N81    (NONE|BIT8|STOP1)
#define E81    (EVEN|BIT8|STOP1)
#define N82    (NONE|BIT8|STOP2)
#define E71    (EVEN|BIT7|STOP1)

typedef enum {
    B50, B75, B110, B134_5, B150, B300, B600, B1200, B1800, B2000,
    B2400, B3600, B4800, B7200, B9600, B19200, B38400
} baud_t;

typedef struct {

    char *device;		/* OS-9 device, eg, "/t80" */
    int path;			/* path id or device descriptor */
    baud_t bau;			/* baud rate */
    unsigned char par;		/* device parity */
    unsigned char eor;		/* end of record character */
    char *name;			/* common name, e.g., "Thruster" */

} ody_serial_t;

#endif
