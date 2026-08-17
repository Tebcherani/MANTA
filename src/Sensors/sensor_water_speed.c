#include "random_utils.h"
#include "sensor_water_speed.h"

double update_water_speed(const water_speed_input_t *input) {
    return input->true_speed + input->noise_std_dev * std_normal_random_num();
}