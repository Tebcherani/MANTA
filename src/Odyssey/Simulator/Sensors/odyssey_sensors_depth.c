#include "odyssey.h"
#include "odyssey_sensors.h"
#include "random_utils.h"
#include "sensor_depth.h"

#include "vehicle.h"

void update_depth_sensor(void) {
    depth_input_t input;
    double measured_depth;

    input.true_depth = read_sensor(REAL_Z);
    input.noise_std_dev = read_sensor(REAL_DEPTH_NOISE);

    measured_depth = update_depth(&input);

    update_sensor(M_DEPTH, measured_depth);
}
