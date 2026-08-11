/*----------------------------------------------------------------------*
   Copyright (C) 1994-1997, Massachusetts Institute of Technology.
	 Sea Grant Autonomous Underwater Vehicles Laboratory.
			 All rights reserved.
  $Id: ody_func.h,v 2.22 1998/02/03 04:21:57 bamoran Exp $
 *----------------------------------------------------------------------*/

#ifndef ODY_FUNC_H
#define ODY_FUNC_H

#include <stdio.h>

#include "odyssey_sim.h"

int  handle_modem(void);
void update_water_conductivity(void);

struct behavior;
                                /* dynamic_control.c */
void dynamic_control( void );
double limit( double, double, double );

                                /* layered_control.c */
void layered_control( void );
void resolve( int priority );
void shut_down( int );
int manage_time( struct behavior *b, double duration );

			/* load_fault.c */
int load_faults(const char *fname, const odyssey_config_t *config);
void        apply_faults       ( void );
const char *get_fault_file_name( void );

int millisec( void );

				/* sim.c — full prototypes required by modern GCC */
double thruster_torque( double real_amps, double w_speed );
double thruster_force( double real_amps, double w_speed );

const char *get_fault_file_path(void);

#endif /* ODY_FUNC_H */
