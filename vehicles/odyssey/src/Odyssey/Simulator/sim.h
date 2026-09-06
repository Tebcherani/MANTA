#ifndef SIM_H
#define SIM_H

#include "world_sim.h"

double thruster_torque(const world_state_t *, double real_amps, double w_speed);
double thruster_force(const world_state_t *, double real_amps, double w_speed);

#endif
