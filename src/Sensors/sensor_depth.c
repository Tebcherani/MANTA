#include "random_utils.h"
#include "sensor_depth.h"

double update_depth(const depth_input_t *input) {
    return input->true_depth + std_normal_random_num() * input->noise_std_dev;
}