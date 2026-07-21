/*----------------------------------------------------------------------*
   Copyright (C) 1994-1997, Massachusetts Institute of Technology.
	 Sea Grant Autonomous Underwater Vehicles Laboratory.
			 All rights reserved.
  $Id: update.c,v 2.21 1998/01/20 00:31:51 bamoran Exp $
 *----------------------------------------------------------------------*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "diagnostic.h"		/* MITSG */
#include "vehicle.h"

int assign_behavior_fcn_ptr( const char *name, struct behavior *b );
int hirestime( double *hires );

/* ============================================================ */
/* UTILITY ROUTINE FOR LOADING VEHICLE STRUCTURE.               */

/* Feb6 94 JL: porting to o+r OS-9 system.  We shouldn't use rtime,
   use vvp->[M_PRESENT_TIME].x instead */

void update( int index, double val )
{
    vvp->s[index].x = val; 
    vvp->s[index].time = vvp->s[M_PRESENT_TIME].time;

				/* increase hit counter */
    (vvp->s[index].numHits)++;
				/* recursive forms for 1st & 2nd moments */
    vvp->s[index].m1 += (val - vvp->s[index].m1)/vvp->s[index].numHits;
    vvp->s[index].m2 += (val*val - vvp->s[index].m2)/vvp->s[index].numHits;

				/* store minimum & maximum values */
    if ( val < vvp->s[index].min ) vvp->s[index].min = val;
    if ( val > vvp->s[index].max ) vvp->s[index].max = val;
}

void update_add( int index, double value )
{
    update( index, sensor(index) + value );
}

extern char *creation_date;
extern int *sindex[];

static void copy_name( char *dest, size_t dest_size, const char *src )
{
    if ( dest_size == 0 )
        return;
    strncpy( dest, src, dest_size - 1 );
    dest[dest_size - 1] = '\0';
}

/**
 * Initialize the vehicle runtime data structure.
 *
 * Copies the sensor and behavior templates into the active vehicle data
 * structure, initializes command and state behavior assignments, associates
 * behavior names with their function pointers, and sets default timing and
 * mission-control values.
 *
 * Also initializes the vehicle rotation matrix to the identity matrix,
 * records the data-structure creation date, and starts the mission clock.
 *
 * Returns 1 on success. Exits the program if the mission clock cannot
 * be initialized.
 */
int init_data_structure() {
    int i, j, loud = 0;
    char *tmp;

    strcpy(msg_buf, "Initializing vehicle data structure ...\n");
    write_message(TERM | SLOG);

    // Select vvv as the active vehicle data structure.
    vvp = &vvv;

    sprintf(msg_buf, "\t%d sensors, %d behaviors ...\n", num_sensors, num_behaviors);
    write_message(TERM | SLOG);

    // Copy the sensor templates into the active vehicle state.
    for (i = 0; i < num_sensors; i++) {
        copy_name( vvv.s[i].name, sizeof(vvv.s[i].name), sss[i].name);
        vvv.s[i].x = sss[i].x;
        vvv.s[i].time = sss[i].time;
        
        if (loud) {
            sprintf(msg_buf, "  sensor: %30s %.4g\n", vvv.s[i].name, vvv.s[i].x);
            write_message(SLOG);
        }
    }
    
    // Copy the behavior templates and their default arguments. 
    for (i = 0 ; i < num_behaviors; i++) {
        copy_name(vvv.b[i].name, sizeof(vvv.b[i].name), bTemplate[i].name);
        
        for (j = 0; j < MAX_B_ARG; j++) {
            copy_name(vvv.b[i].arg[j].name, sizeof(vvv.b[i].arg[j].name), bTemplate[i].arg[j].name);
            vvv.b[i].arg[j].x = bTemplate[i].arg[j].x;
        }
    }
    
    // Mark all command slots as having no assigned behavior.
    for (i = 0; i < MAX_COMMAND; i++)
        vvv.c[i].behavior = -1;

    // Mark every state command slot as having no assigned behavior.
    for (j = 0; j < MAX_STATES; j++)
        for (i=0;i<MAX_COMMAND;i++)
            vvp->sta[j].c[i].behavior = -1;

    // Populate the sensor index mappings.
    for (i = 0; i < num_sensors; i++)
        *(sindex[i]) = i;

    // Associate each behavior with its implementation function.
    for (i = 0; i < num_behaviors; i++)
        assign_behavior_fcn_ptr( bTemplate[i].name, &(vvv.b[i]) );

    // Set counts and default mission-control values.
    vvv.last_s = num_sensors - 1;
    vvv.last_b = num_behaviors - 1;
    vvv.comm_synchro = 1;	
    vvv.abort_mission = 0;	
    vvv.control_cycle_time = 0.2;
    vvv.comm_cycle_time = 1.0;	

    // Initialize the vehicle rotation matrix to the identity matrix.
    vvv.rotation[0][0] = 1;     
    vvv.rotation[1][1] = 1;     
    vvv.rotation[2][2] = 1;     

    // Store the data structure creation date.
    tmp = strcpy(vvv.creation_date, creation_date);
    if (tmp == NULL) {
        strcpy(msg_buf, "Error copying creation date\n");
        write_message( TERM | SLOG );
    }
				
    /* Initialize the mission clock. It will be reset again immediately
     * before the vehicle is launched.
     */
    if (hirestime(&vvp->mission_start_time) == -1) {
        strcpy(msg_buf, "\nfatal error starting mission clock\n");
        write_message(TERM | SLOG);
        exit(-1);
    }

    return 1;
}
