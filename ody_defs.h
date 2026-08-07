#ifndef ODY_DEFS_H
#define ODY_DEFS_H

#include "common_defs.h"

#define D2R(d) (d*PI/180.0)
#define R2D(r) (r*180.0/PI)

#define MAX_CHARS     64	/* names include units, e.g. sensor(unit) */
#define MAX_B_ARG     10
#define MAX_C_ARG     3

#define MAX_SENSOR      550
#define MAX_BEHAVIOR    30
#define MAX_COMMAND     30
#define MAX_FILE_NAME   512
#define MAX_STATES      3
#define MAX_STATE_NAME  128
#define MAX_LINE        196

#define sensor(i)       (vvp->s[(i)].x)
#define sensor_time(i)  (vvp->s[(i)].time)
#define sensor_name(i)  (vvp->s[(i)].name)
#define sensor_min(i)   (vvp->s[(i)].min)
#define sensor_max(i)   (vvp->s[(i)].max)
#define sensor_m1(i)    (vvp->s[(i)].m1)
#define sensor_m2(i)    (vvp->s[(i)].m2)
#define sensor_count(i) (vvp->s[(i)].numHits)
#define bArg(i)         (b->arg[(i)].x)

#define EXTERN extern		/* EXTERN hack jleonard */

#define VEHICLE_NAME "Simulator"
#define PREFIX_CHAR  'S'

#define EDGEPS8000     "/t82"	/* EdgeTech PS-8000 transceiver */
#define WHOI_USBL      "/t84"	/* WHOI USBL homing system */
#define WHOI_LATCH		/* latch motor and switches */

#endif
