/*----------------------------------------------------------------------*
   Copyright (C) 1994-1997, Massachusetts Institute of Technology.
	 Sea Grant Autonomous Underwater Vehicles Laboratory.
			 All rights reserved.
  $Id: getopt.h,v 2.21 1998/01/20 00:31:51 bamoran Exp $
 *----------------------------------------------------------------------*/

#ifndef GETOPT_H
#define GETOPT_H

extern char *optarg;
extern int optopt, optind, opterr;

extern int getopt( int argc, char *const argv[], const char *optstring );

#endif
