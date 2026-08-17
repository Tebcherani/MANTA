#include "odyssey_sensors.h"
#include "sensor_compass.h"

#include "vehicle.h"

static compass_state_t compass_state;

int init_odyssey_sensors(void) {
    if (!init_compass_state(&compass_state)) {
        write_to_sim_log("[odyssey_sensors] Failed to initialize compass state.\n");
        return 0;
    }

    write_to_sim_log("[odyssey_sensors] Odyssey sensors initialized.\n");
    return 1;
}

int update_odyssey_sensors(const world_state_t *world_state, double timestep) {
    update_ahrs_sensors(world_state, timestep);
    update_odyssey_depth();
    update_water_sensors();
    update_usbl_sensor();
    update_lbl_sensor();

    return 1;
}

compass_state_t *get_odyssey_compass_state(void) {
    return &compass_state;
}