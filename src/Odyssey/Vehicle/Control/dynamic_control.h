#ifndef DYNAMIC_CONTROL_H
#define DYNAMIC_CONTROL_H

#include "world_sim.h"

void dynamic_control(const world_state_t *);
double limit(double, double, double);

#endif