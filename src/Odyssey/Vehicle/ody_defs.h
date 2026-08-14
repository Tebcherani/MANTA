#ifndef ODY_DEFS_H
#define ODY_DEFS_H

#define MAX_SENSOR      550
#define MAX_BEHAVIOR    30
#define MAX_COMMAND     30
#define MAX_FILE_NAME   512
#define MAX_STATES      3
#define MAX_STATE_NAME  128
#define MAX_LINE        196

#define sensor_time(i)  (vvp->s[(i)].time)
#define sensor_name(i)  (vvp->s[(i)].name)
#define bArg(i)         (b->arg[(i)].x)

#define EXTERN extern		/* EXTERN hack jleonard */

#define PREFIX_CHAR  'S'

#endif
