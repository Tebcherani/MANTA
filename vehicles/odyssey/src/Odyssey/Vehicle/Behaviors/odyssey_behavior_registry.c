#include <string.h>
#include "ody_types.h"
#include "odyssey_behaviors.h"

extern int mission_timer(struct behavior *, int, int);
extern int acquire_heading(struct behavior *, int, int);
extern int ascend(struct behavior *, int, int);
extern int broadcast(struct behavior *, int, int);
extern int chirp_fin(struct behavior *, int, int);
extern int depth_envelope(struct behavior *, int, int);
extern int descend(struct behavior *, int, int);
extern int dock(struct behavior *, int, int);
extern int undock(struct behavior *, int, int);
extern int whoi_homing(struct behavior *, int, int);
extern int set_elevator(struct behavior *, int, int);
extern int set_rudder(struct behavior *, int, int);
extern int setmode(struct behavior *, int, int);
extern int setpoint(struct behavior *, int, int);
extern int setvector(struct behavior *, int, int);
extern int survey_dead_reckon(struct behavior *, int, int);
extern int waypoint_2d(struct behavior *, int, int);
extern int wiggle(struct behavior *, int, int);
extern int launch(struct behavior *, int, int);
extern int yo_yo(struct behavior *, int, int);

int assign_behavior_fcn_ptr(const char *name, struct behavior *b)
{
    if (name == NULL || b == NULL)
        return -1;

    b->fcn = NULL;

    if (strcmp("mission_timer", name) == 0) b->fcn = mission_timer;
    if (strcmp("acquire_heading", name) == 0) b->fcn = acquire_heading;
    if (strcmp("ascend", name) == 0) b->fcn = ascend;
    if (strcmp("broadcast", name) == 0) b->fcn = broadcast;
    if (strcmp("chirp_fin", name) == 0) b->fcn = chirp_fin;
    if (strcmp("depth_envelope", name) == 0) b->fcn = depth_envelope;
    if (strcmp("descend", name) == 0) b->fcn = descend;
    if (strcmp("dock", name) == 0) b->fcn = dock;
    if (strcmp("undock", name) == 0) b->fcn = undock;
    if (strcmp("whoi_homing", name) == 0) b->fcn = whoi_homing;
    if (strcmp("set_elevator", name) == 0) b->fcn = set_elevator;
    if (strcmp("set_rudder", name) == 0) b->fcn = set_rudder;
    if (strcmp("setmode", name) == 0) b->fcn = setmode;
    if (strcmp("setpoint", name) == 0) b->fcn = setpoint;
    if (strcmp("setvector", name) == 0) b->fcn = setvector;
    if (strcmp("survey_dead_reckon", name) == 0) b->fcn = survey_dead_reckon;
    if (strcmp("waypoint_2d", name) == 0) b->fcn = waypoint_2d;
    if (strcmp("wiggle", name) == 0) b->fcn = wiggle;
    if (strcmp("launch", name) == 0) b->fcn = launch;
    if (strcmp("yo_yo", name) == 0) b->fcn = yo_yo;

    if (b->fcn == NULL)
        return -1;

    return 0;
}
