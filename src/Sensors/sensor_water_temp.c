#include <math.h>
#include <stddef.h>

#include "sensor_water_temp.h"
#include "sim_log.h"

double update_water_temp(const water_temp_input_t *input) {
    double measured_temperature;

    if (input == NULL) {
        write_to_sim_log("[sensor_water_tempe] Failed to update water temp: input pointer is NULL.\n");
        return 0.0;
    }

    return 10 * exp(-pow(input->true_depth / 30, 2));
}