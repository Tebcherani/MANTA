#ifndef ODYSSEY_SENSORS_H
#define ODYSSEY_SENSORS_H

#include "sensor_compass.h"
#include "world_sim.h"

int init_odyssey_sensors(void);
int update_odyssey_sensors(const world_state_t *, double);

void update_ahrs_sensors(const world_state_t *, double);
void update_depth_sensor(void);
void update_water_sensors(void);
void update_usbl_sensor(void);
void update_lbl_sensor(void);

compass_state_t *get_odyssey_compass_state(void);

#endif /* ODYSSEY_SENSORS_H */