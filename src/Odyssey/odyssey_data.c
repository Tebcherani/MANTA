#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "diagnostic.h"
#include "odyssey_data.h"
#include "vehicle.h"

int assign_behavior_fcn_ptr(const char *name, struct behavior *b);
int hirestime(double *hires);

/* ============================================================ */
/* UTILITY ROUTINE FOR LOADING VEHICLE STRUCTURE.               */

/* Feb6 94 JL: porting to o+r OS-9 system.  We shouldn't use rtime,
   use vvp->[M_PRESENT_TIME].x instead */

void update(int index, double val) {
    vvp->s[index].x = val; 
    vvp->s[index].time = vvp->s[M_PRESENT_TIME].time;

	// Increase hit counter
    (vvp->s[index].numHits)++;

	// Recursive forms for 1st & 2nd moments
    vvp->s[index].m1 += (val - vvp->s[index].m1)/vvp->s[index].numHits;
    vvp->s[index].m2 += (val*val - vvp->s[index].m2)/vvp->s[index].numHits;

	// Store minimum & maximum values
    if ( val < vvp->s[index].min ) vvp->s[index].min = val;
    if ( val > vvp->s[index].max ) vvp->s[index].max = val;
}

void update_add(int index, double value) {
    update(index, sensor(index) + value);
}

extern char *creation_date;
extern int *sindex[];

static void copy_name(char *dest, size_t dest_size, const char *src) {
    if (dest_size == 0) return;
    strncpy(dest, src, dest_size - 1);
    dest[dest_size - 1] = '\0';
}

int init_data_structure() {
    int i, j, loud = 0;
    char *tmp;

    vvp = &vvv;


    // Initialize vvv.s[] from sss
    for (i = 0; i < num_sensors; i++) {
        copy_name(vvv.s[i].name, sizeof(vvv.s[i].name), sss[i].name);
        vvv.s[i].x = sss[i].x;
        vvv.s[i].time = sss[i].time;
    }
    
    // Initialize vvv.b[] from bTemplate
    for (i = 0 ; i < num_behaviors; i++) {
        copy_name(vvv.b[i].name, sizeof(vvv.b[i].name), bTemplate[i].name);
        
        for (j = 0; j < MAX_B_ARG; j++) {
            copy_name(vvv.b[i].arg[j].name, sizeof(vvv.b[i].arg[j].name), bTemplate[i].arg[j].name);
            vvv.b[i].arg[j].x = bTemplate[i].arg[j].x;
        }
    }
    
    // Initialize vvv.c[] to all zeros (& behaviors to -1)
    for (i = 0; i < MAX_COMMAND; i++) vvv.c[i].behavior = -1;

    for (j = 0; j < MAX_STATES; j++)
        for (i=0;i<MAX_COMMAND;i++) vvp->sta[j].c[i].behavior = -1;

    // Initialize sindex and bindex arrays
    for (i = 0; i < num_sensors; i++) *(sindex[i]) = i;

    for (i = 0; i < num_behaviors; i++) assign_behavior_fcn_ptr( bTemplate[i].name, &(vvv.b[i]) );

    vvv.last_s = num_sensors - 1;
    vvv.last_b = num_behaviors - 1;
    vvv.comm_synchro = 1;	/* Sync flag for COMM process  */
    vvv.abort_mission = 0;	/* added jleonard 11/13/95 */
    vvv.control_cycle_time = 0.2; /* Cycle time for control loop */
    vvv.comm_cycle_time = 1.0;	/* Cycle time for comm */
    vvv.rotation[0][0] = 1;     
    vvv.rotation[1][1] = 1;     
    vvv.rotation[2][2] = 1;     

    tmp = strcpy(vvv.creation_date, creation_date);

	// Grab start time for init, will be grabbed again prior to actual launch
    if ( hirestime(&vvp->mission_start_time) == -1 ) exit (-1);

    return 1;
}
