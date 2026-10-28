#include <math.h>
#include <stddef.h>

#include "sensor_water_temp.h"
#include "sim_log.h"

double update_water_temp(double true_depth_m) {
    return 10 * exp(-pow(true_depth_m / 30, 2));
}