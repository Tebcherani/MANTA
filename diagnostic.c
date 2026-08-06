#include <stdio.h>

#include "diagnostic.h"
#include "odyssey_log.h"
#include "vehicle.h"

char msg_buf[256];

/*-----------------------------------------------------------------------*
  write_message: send contents of msg_buf to appropriate destinations
  e.g., TERM, SLOG (defined in diagnostic.h)

  monitor_msg: compose and send string that reports name & value of
  particular sensor
 *-----------------------------------------------------------------------*/

void write_message(int monitor)
{
    if (monitor & TERM) fputs(msg_buf, stderr);
    if ((monitor & SLOG) && syslog != NULL) fputs(msg_buf, syslog);
    return;
}

void monitor_msg( int sensor_id, int Monitor )
{
    sprintf( msg_buf, "% 8.2f: %29s % .3g\n", sensor(M_PRESENT_TIME),
	     sensor_name(sensor_id), sensor(sensor_id) );
    write_message( Monitor );
}