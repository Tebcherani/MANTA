#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "odyssey_behaviors.h"
#include "odyssey_data_struct.h"
#include "odyssey_defs.h"
#include "sim_log.h"
#include "vehicle.h"

static void copy_name(char *dest, size_t dest_size, const char *src);

int hirestime(double *hires);

extern char *creation_date;
extern int *sindex[];

int init_odyssey_data_structure() {
    int i, j = 0;

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
        
        for (j = 0; j < B_ARG_SIZE; j++) {
            copy_name(vvv.b[i].arg[j].name, sizeof(vvv.b[i].arg[j].name), bTemplate[i].arg[j].name);
            vvv.b[i].arg[j].x = bTemplate[i].arg[j].x;
        }
    }
    
    // Initialize vvv.c[] to all zeros (& behaviors to -1)
    for (i = 0; i < MAX_COMMAND; i++) vvv.c[i].behavior = -1;

    for (j = 0; j < MAX_STATES; j++)
        for (i = 0; i<MAX_COMMAND; i++) vvp->sta[j].c[i].behavior = -1;

    // Initialize sindex and bindex arrays
    for (i = 0; i < num_sensors; i++) *(sindex[i]) = i;

    for (i = 0; i < num_behaviors; i++) {
        if (assign_behavior_fcn_ptr(bTemplate[i].name, &(vvv.b[i])) != 0) {
            write_to_sim_log("[odyssey_data_struct] Failed to assign behavior function pointer.\n");
            return 0;
        }
    }

    vvv.last_s = num_sensors - 1;
    vvv.abort_mission = 0;
    vvv.rotation[0][0] = 1;     
    vvv.rotation[1][1] = 1;     
    vvv.rotation[2][2] = 1;     

    strcpy(vvv.creation_date, creation_date);

    write_to_sim_log("[odyssey_data_struct] Ody data struct initialized.\n");
    return 1;
}

int match_sensor(const char *name) {
    int i;
    char *eor;
    char thisName[sizeof(vvp->s[0].name)];
    char vvvName[sizeof(vvp->s[0].name)];

    if (name == NULL)
        return -1;

    copy_name(thisName, sizeof(thisName), name);

    if ((eor = strchr(thisName, '(')) != NULL)
        *eor = '\0';

    for (i = 0; i < num_sensors; i++) {
        copy_name(vvvName, sizeof(vvvName), sss[i].name);

        if ((eor = strchr(vvvName, '(')) != NULL)
            *eor = '\0';

        if (strcmp(vvvName, thisName) == 0)
            return i;
    }

    return -1;
}

double read_sensor(int sensor_name) {
    return vvp->s[sensor_name].x;
}

void update_sensor(int index, double val) {
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

void add_to_sensor(int index, double value) {
    update_sensor(index, read_sensor(index) + value);
}

static void copy_name(char *dest, size_t dest_size, const char *src) {
    if (dest_size == 0) return;
    strncpy(dest, src, dest_size - 1);
    dest[dest_size - 1] = '\0';
}
