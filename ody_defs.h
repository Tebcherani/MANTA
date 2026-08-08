#ifndef ODY_DEFS_H
#define ODY_DEFS_H

#include "common_defs.h"

#define MAX_SENSOR      550
#define MAX_BEHAVIOR    30
#define MAX_COMMAND     30
#define MAX_FILE_NAME   512
#define MAX_STATES      3
#define MAX_STATE_NAME  128
#define MAX_LINE        196

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

#endif
