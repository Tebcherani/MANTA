/*----------------------------------------------------------------------*
   Copyright (C) 1994-1998, Massachusetts Institute of Technology.
	 Sea Grant Autonomous Underwater Vehicles Laboratory.
			 All rights reserved.
  $Id: ody_defs.h,v 2.24 1998/02/03 04:20:33 bamoran Exp $
 *----------------------------------------------------------------------*/

#ifndef ODY_DEFS_H
#define ODY_DEFS_H

#include "ody_name.h"		/* identifies which vehicle */

#ifndef PI
#define PI     3.14159265358979323846
#endif

#define D2R(d) (d*PI/180.0)
#define R2D(r) (r*180.0/PI)

#define MAX_CHARS     30	/* slightly cryptic, needs revision */
#define MAX_B_ARG     10
#define MAX_C_ARG     3

#define MAX_SENSOR      550
#define MAX_BEHAVIOR    30
#define MAX_COMMAND     30
#define MAX_MODEM       500
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

#define SAIL1_TIMEOUT   500	/* 500 mSecs */

#define SHARED       0x8000	/* Data module definitions */
#define SINGLE_USER  0x00
#define DATAMOD      0x400
#define RD_WRT_DATA  0x09

#if defined _AMPHITRITE		/* VEHICLE CONFIGURATIONS */

#define VEHICLE_NAME "Amphitrite"
#define PREFIX_CHAR  'A'

#define KVH_DGC        "/t80"	/* KVH DGC digital gyrocompass */
#define KVH_DGI        "/t81"
#define SBE_25         "/t83"	/* Sea Bird Electronics CTD */
#define ODY_SIDESCAN   "/t84"	/* Oceanetics SideScan sonar */
#define EDGEPS8000     "/t82"	/* EdgeTech PS-8000 transceiver */
#define RDI_WH         "/t85"	/* RD Instruments Workhorse ADCP */
#define PAROSCI        "/t86"    
#define SAIL1          "/t87"        

#elif defined _BOREALIS

#define VEHICLE_NAME "Borealis"
#define PREFIX_CHAR  'B'

#define FINCAL_TC9801		/* which set of coeffs in fins.h */

#define KVH_DGC        "/t80"	/* KVH DGC digital gyrocompass */
#define KVH_DGI        "/t81"
#define EDGEPS8000     "/t82"	/* EdgeTech PS-8000 transceiver */
#define SBE_25         "/t83"	/* Sea Bird Electronics CTD */
#define WHOI_USBL      "/t84"	/* WHOI USBL homing system */
#define SAIL2          "/t85"
#define TT8_WDOG		/* TattleTale 8 based Watchdog */
#define AEL_BATCON		/* WHOI AEL (A. Bradley) battery controller */
#define PAROSCI        "/t86"    
#define SAIL1          "/t87"        
#define WHOI_LATCH		/* latch motor and switches */
#define SONTEK_ADV     "/t1"	/* SonTek ADV Ocean */
#define ARGOS_MSG

#elif defined _SIMULATOR

#define VEHICLE_NAME "Simulator"
#define PREFIX_CHAR  'S'

#define EDGEPS8000     "/t82"	/* EdgeTech PS-8000 transceiver */
#define SBE_25         "/t83"	/* Sea Bird CTD */
#define WHOI_USBL      "/t84"	/* WHOI USBL homing system */
#define WHOI_LATCH		/* latch motor and switches */
#define PAROSCI        "/t86"    
#define SAIL1          "/t87"        
#define ARGOS_MSG

#elif defined _WHOI

#define VEHICLE_NAME "Wodyssey"
#define PREFIX_CHAR  'W'

#define FINCAL_TC9802		/* which set of coeffs in fins.h */
#define KVH_DGC        "/t80"	/* KVH DGC digital gyrocompass */
#define KVH_DGI        "/t81"
#define EDGEPS8000     "/t82"	/* EdgeTech PS-8000 transceiver */
#define SBE_25         "/t83"	/* Sea Bird CTD */
#define WHOI_USBL      "/t84"	/* WHOI USBL homing system */
#define PAROSCI        "/t86"
#define SAIL1          "/t87"
#define EDC_COUPLE		/* dummy load for now, but  */
#define WHOI_LATCH		/* latch motor and switches */
#define ARGOS_MSG

#elif defined _XANTHOS

#define VEHICLE_NAME "Xanthos"
#define PREFIX_CHAR  'X'

#define KVH_DGC        "/t80"	/* KVH DGC digital gyrocompass */
#define KVH_DGI        "/t81"
#define EDGEPS8000     "/t82"	/* EdgeTech PS-8000 transceiver */
#define PAROSCI        "/t86"    
#define SAIL1          "/t87"        

#endif /* VEHICLE CONFIGURATIONS */

#endif
