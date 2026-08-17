#include <math.h>

#include "odyssey.h"
#include "odyssey_sensors.h"
#include "random_utils.h"
#include "sensor_water_conductivity.h"
#include "sensor_water_speed.h"
#include "sensor_water_temp.h"

#include "vehicle.h"

static void update_water_conductivity_sensor(void);
static void update_water_speed_sensor(void);
static void update_water_temp_sensor(void);

static void update_water_conductivity_sensor(void) {
    water_conductivity_input_t input;
    double measured_conductivity;

    input.true_depth = read_sensor(REAL_Z);

    measured_conductivity = update_water_conductivity(&input);
    update_sensor(WATER_COND, measured_conductivity);
}

static void update_water_speed_sensor(void) {
    water_speed_input_t input;
    double measured_speed;

    input.true_speed = read_sensor(REAL_U);
    input.noise_std_dev = 0.02;

    measured_speed = update_water_speed(&input);
    update_sensor(M_WATER_SPEED, measured_speed);
}

static void update_water_temp_sensor(void) {
    water_temp_input_t input;
    double measured_temp;

    input.true_depth = read_sensor(REAL_Z);

    measured_temp = update_water_temp(&input);
    update_sensor(WATER_TEMP, measured_temp);
}

void update_water_sensors(void) {
    update_water_speed_sensor();
    update_water_temp_sensor();
    update_water_conductivity_sensor();
}