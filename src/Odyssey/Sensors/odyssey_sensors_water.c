#include "ody_defs.h"
#include "odyssey_data.h"
#include "odyssey_sensors.h"
#include "vehicle.h"

static void update_water_speed(void);
static void update_water_temp(void);

static void update_water_speed(void) {
    double std_normal_random_num();
    update(M_WATER_SPEED, sensor(REAL_U) + 0.02*std_normal_random_num());
}

static void update_water_temp(void) {
    update(WATER_TEMP, 10 * exp(-pow(sensor(REAL_Z)/30,2)));
    update_water_conductivity();
}

void update_water_sensors(void) {
    update_water_speed();
    update_water_temp();
}