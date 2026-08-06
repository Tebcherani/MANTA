/*----------------------------------------------------------------------*
   Copyright (C) 1994-1997, Massachusetts Institute of Technology.
	 Sea Grant Autonomous Underwater Vehicles Laboratory.
			 All rights reserved.
  $Id: diagnostic.h,v 2.21 1998/01/20 00:31:51 bamoran Exp $
 *----------------------------------------------------------------------*/

#ifndef DIAGNOSTIC_H
#define DIAGNOSTIC_H

#define TERM 001		/* for terminal monitoring */
#define SLOG 002		/* for syslog monitoring */
#define DUMP 004		/* hex dump to syslog file */

				/* status codes */
enum ody_dev_status {
    STATUS_OK, STATUS_NOT_SELECTED, STATUS_NOT_CONNECTED,
    STATUS_UNINITIALIZED, STATUS_SERIAL_PORT, STATUS_NO_SIGNALS,
    STATUS_SAIL1_COMMS, STATUS_INDEX_FAILED, STATUS_FORK_FAILED,
    STATUS_UNKNOWN
};

extern char msg_buf[];		/* buffer for storing diagnostics */

                                /* prototypes */
void write_message(int monitor);

#endif /* DIAGNOSTIC_H */
