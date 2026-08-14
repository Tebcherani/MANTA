#include "odyssey_sensors.h"

#include "vehicle.h"

int update_odyssey_sensors(const world_state_t *world_state) {
    update_ahrs_sensors(world_state);
    update_depth_sensor();
    update_water_sensors();
    update_usbl_sensor();
    update_lbl_sensor();

    return 1;
}