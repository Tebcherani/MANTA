#include <stddef.h>

#include "random_utils.h"
#include "sensor_depth.h"
#include "sim_log.h"

double update_depth(const depth_input_t *input) {
    if (input == NULL) {
        write_to_sim_log("[sensor_depth] Failed to update depth: input pointer is NULL.\n");
        return 0.0;
    }

    return input->true_depth + std_normal_random_num() * input->noise_std_dev;
}