/*----------------------------------------------------------------------*
   Copyright (C) 1994-1997, Massachusetts Institute of Technology.
	 Sea Grant Autonomous Underwater Vehicles Laboratory.
			 All rights reserved.
  $Id: load_mission.c,v 2.23 1998/02/03 04:25:07 bamoran Exp $
 *----------------------------------------------------------------------*/

#include <errno.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "odyssey_sim.h"
#include "odyssey_data_struct.h"
#include "paths.h"
#include "vehicle.h"
#include "common_defs.h"
#include "unit_utils.h"

static int skip_comments(FILE *file);

/*--------------------------------------------------------------------*
  routine to skip a line of input if it begins with a *, #, or % 
  returns EOF if encountered, else returns 0; 
 *--------------------------------------------------------------------*/

static int skip_comments( FILE *file )
{
    char line[MAX_LINE];
    int c;
				/* test first character */
    c = fgetc(file);
    if ( c == EOF )
	return c;
				/* if not comment, restore file */
    if ( !strchr("%#*", c) )
    {
	ungetc(c, file);
	return 0;
    }
				/* eat rest of line */
    if ( fgets(line, sizeof(line), file) == NULL )
    {
	fprintf(stderr, "skip_comments() eof encountered\n");
	return EOF;
    }
				/* recursive call for next comment */
    return skip_comments(file);
}
