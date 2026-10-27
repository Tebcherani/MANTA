#include "odyssey.h"
#include "odyssey_sensors.h"
#include "random_utils.h"
#include "sensor_depth.h"

#include "vehicle.h"

void update_odyssey_depth(void) {
    depth_input_t input;
    double measured_depth_m;

    input.true_depth_m = read_sensor(REAL_Z);
    input.noise_std_dev_m = read_sensor(REAL_DEPTH_NOISE);

    measured_depth_m = update_depth(&input);

    update_sensor(M_DEPTH, measured_depth_m);
}
