#include <stddef.h>

#include "random_utils.h"
#include "sensor_water_speed.h"
#include "sim_log.h"

double update_water_speed(const water_speed_input_t *input) {
    if (input == NULL) {
        write_to_sim_log("[sensor_water_speed] Failed to update water speed: input pointer is NULL.\n");
        return 0.0;
    }
    
    return input->true_speed + input->noise_std_dev * std_normal_random_num();
}