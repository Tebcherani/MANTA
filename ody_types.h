/*----------------------------------------------------------------------*
  Copyright (C) 1994-1998, Massachusetts Institute of Technology.
  Proprietary to Sea Grant AUV Laboratory.  All rights reserved.
  $Id: ody_types.h,v 2.23 1998/02/03 04:25:07 bamoran Exp $
 *----------------------------------------------------------------------*/

#ifndef ODY_TYPES_H
#define ODY_TYPES_H

#include "ody_defs.h"
#include "common_defs.h"
#include "odyssey_control.h"

struct sensor {
    char name[NAME_SIZE];	/* name AND UNITS!! */
    double x;			/* present value */
    long time;			/* when updated last (ms) */
    unsigned long numHits;	/* counter for number of updates */
    double m1, m2;		/* 1st and 2nd moments for statistics */
    double min, max;		/* minimum and maximum values attained */
};

struct state {			/* For SCLC */
    char name[MAX_STATE_NAME];
    struct command c[MAX_COMMAND];
};

struct vehicle {
    char creation_date[NAME_SIZE];
    char mission_date[NAME_SIZE];
    char data_file_name[MAX_FILE_NAME];
    char fault_file_name[MAX_FILE_NAME];  /* bare filename of loaded *.fd, or "" */
    int last_s;
    int abort_mission;
    double mission_start_time;
    double rotation[3][3];
    char modem_send[500];
    struct sensor s[MAX_SENSOR];
    struct behavior b[MAX_BEHAVIOR];
    struct command c[MAX_COMMAND];
    struct state sta[MAX_STATES];   /* state array for SCLC */
    int cstate, nstates;
};

#endif /* ODY_TYPES_H */
