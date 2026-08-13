#ifndef ODYSSEY_SENSORS_H
#define ODYSSEY_SENSORS_H

#include "world_sim.h"

void update_ahrs_sensors(const world_state_t *);
void update_depth_sensor(void);
void update_water_sensors(void);
void update_usbl_sensor(void);
void update_lbl_sensor(void);

int update_odyssey_sensors(const world_state_t *);

#endif /* ODYSSEY_SENSORS_H */