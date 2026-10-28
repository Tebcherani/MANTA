#include <stddef.h>

#include "random_utils.h"
#include "sensor_gyro.h"
#include "sim_log.h"

gyro_result_t update_gyro(const gyro_config_t *config, const gyro_input_t *input) {
    gyro_result_t result;

    result.success = 0;
    result.measurement.roll_rate_radps = 0.0;
    result.measurement.pitch_rate_radps = 0.0;
    result.measurement.yaw_rate_radps = 0.0;

    if (config == NULL) {
        write_to_sim_log("[sensor_gyro] Failed to update gyro: config pointer is NULL.\n");
        return result;
    }

    if (input == NULL) {
        write_to_sim_log("[sensor_gyro] Failed to update gyro: input pointer is NULL.\n");
        return result;
    }

    result.measurement.roll_rate_radps = input->roll_rate_radps + config->roll_bias_radps + config->noise_std_dev_radps * std_normal_random_num();
    result.measurement.pitch_rate_radps = input->pitch_rate_radps + config->pitch_bias_radps + config->noise_std_dev_radps * std_normal_random_num();
    result.measurement.yaw_rate_radps = input->yaw_rate_radps + config->yaw_bias_radps + config->noise_std_dev_radps * std_normal_random_num();

    result.success = 1;
    return result;
}