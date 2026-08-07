/*----------------------------------------------------------------------*
  Copyright (C) 1994-1998, Massachusetts Institute of Technology.
  Proprietary to Sea Grant AUV Laboratory.  All rights reserved.
  $Id: ody_types.h,v 2.23 1998/02/03 04:25:07 bamoran Exp $
 *----------------------------------------------------------------------*/

#ifndef ODY_TYPES_H
#define ODY_TYPES_H

#include "ody_defs.h"

struct sensor {
    char name[MAX_CHARS];	/* name AND UNITS!! */
    double x;			/* present value */
    long time;			/* when updated last (ms) */
    unsigned long numHits;	/* counter for number of updates */
    double m1, m2;		/* 1st and 2nd moments for statistics */
    double min, max;		/* minimum and maximum values attained */
};

struct b_arg {
    char name[MAX_CHARS];	/* name AND UNITS!! */
    double x;			/* present value */
};

				/* the state of a behavior and its universe
				   of allowed values */
typedef enum {
    B_ERROR=-1, B_UNINITIALIZED, B_ACTIVE, B_COMPLETE
} b_state_t;

typedef enum { D_ELEVATOR, D_DEPTH, D_PITCH } dpt_mode_t;
typedef enum { H_RUDDER, H_HEADING } hdt_mode_t;
typedef enum { S_CURRENT, S_SPEED } spd_mode_t;

struct behavior {
    char name[MAX_CHARS];
    b_state_t state;		/* state it's in, see above enum typedef */
    double start_time;		/* when it started */
    double end_time;		/* when it will end */
				/* the function that this behavior executes */
    int (*fcn)(struct behavior *, int, int);
				/* its vector of arguments */
    struct b_arg arg[MAX_B_ARG];
};

struct command {
    int behavior;
    struct behavior b;
    dpt_mode_t depth_mode;
    double depth[MAX_C_ARG];
    hdt_mode_t heading_mode;
    double heading[MAX_C_ARG];
    spd_mode_t speed_mode;
    double speed[MAX_C_ARG];
};

struct state {			/* For SCLC */
    char name[MAX_STATE_NAME];
    struct command c[MAX_COMMAND];
};

struct vehicle {
    char creation_date[MAX_CHARS];
    char mission_date[MAX_CHARS];
    char data_file_name[MAX_FILE_NAME];
    char fault_file_name[MAX_FILE_NAME];  /* bare filename of loaded *.fd, or "" */
    int last_s, last_b, last_c;
    int abort_mission;
    double mission_start_time;
    double rotation[3][3];
    char modem_send[500], modem_rec[500];
    char latch_send[ 80], latch_rec[ 80];
    struct sensor s[MAX_SENSOR];
    struct behavior b[MAX_BEHAVIOR];
    struct command c[MAX_COMMAND];
    struct state sta[MAX_STATES];   /* state array for SCLC */
    int cstate, nstates;
};

#endif /* ODY_TYPES_H */
