/*----------------------------------------------------------------------*
   Copyright (C) 1994-1997, Massachusetts Institute of Technology.
	 Sea Grant Autonomous Underwater Vehicles Laboratory.
			 All rights reserved.
  $Id: sensor_load.c,v 2.23 1998/02/03 04:25:07 bamoran Exp $
 *----------------------------------------------------------------------*/

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "common_defs.h"
#include "odyssey_data_struct.h"
#include "vehicle.h"

static int MonitorLBLSim = 2;	/* output to logfile */

/* ============================================================ */
/* Display & clear modem send string.                           */

int handle_modem(void)
{
    int monitor_high_level_control = 0;

    if( strlen(vvp->modem_send) > 0 )
    {
	strcpy( vvp->modem_send, "" ); 
    }

    return 0;
}

/*-----------------------------------------------------------------------*
  getXSDataByte: set appropriate bits of databyte that is to be sent
  through edgetech channel
 *-----------------------------------------------------------------------*/

int getXSDataByte( void )
{
    int databyte = 0;

				/* LSB is presence of pole */
    databyte |= read_sensor(M_POLE_PRESENT) > 0;
				/* bits 1-2 are range from dock */
    if ( read_sensor(M_USBL_RANGE) > 1000 )
	databyte |= 6;
    else if ( read_sensor(M_USBL_RANGE) > 500 )
	databyte |= 4;
    else if ( read_sensor(M_USBL_RANGE) > 50 )
	databyte |= 2;
				/* bit 3 always high during missions */
    databyte |= 8;
				/* MSB high iff all else low */
    if ( !databyte )
	databyte = 128;

    return databyte;
}
