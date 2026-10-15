#include <math.h>
#include <stddef.h>

#include "angle_utils.h"
#include "sensor_usbl.h"
#include "sim_log.h"

usbl_result_t update_usbl(const usbl_config_t *config, const usbl_input_t *input) {
    usbl_result_t result;
    double north_difference;
    double east_difference;
    double down_difference;
    double horizontal_range;

    result.success = 0;
    result.has_measurement = 0;
    result.measurement.range = 0.0;
    result.measurement.azimuth = 0.0;
    result.measurement.elevation = 0.0;

    if (config == NULL) {
        write_to_sim_log("[sensor_usbl] Failed to update USBL: config pointer is NULL.\n");
        return result;
    }

    if (input == NULL) {
        write_to_sim_log("[sensor_usbl] Failed to update USBL: input pointer is NULL.\n");
        return result;
    }

    if (config->hit_interval <= 0.0) {
        write_to_sim_log("[sensor_usbl] Failed to update USBL: hit interval must be greater than zero.\n");
        return result;
    }

    north_difference = input->target_north - input->vehicle_north;
    east_difference = input->target_east - input->vehicle_east;
    down_difference = input->target_down - input->vehicle_down;

    horizontal_range = sqrt(north_difference * north_difference + east_difference * east_difference);

    result.measurement.range = sqrt(north_difference * north_difference + east_difference * east_difference + down_difference * down_difference);
    result.measurement.azimuth = wrap_angle(atan2(east_difference, north_difference) - input->vehicle_yaw);
    result.measurement.elevation = atan2(down_difference, horizontal_range);

    // Produces a hit once every hit_interval, provided the current timestep falls within hit_window
    if (fmod(input->sim_time_s, config->hit_interval) >= config->hit_window) {
        result.success = 1;
        return result;
    }

    if (cos(result.measurement.elevation) <= config->min_direction_cos || cos(result.measurement.azimuth) <= config->min_direction_cos) {
        result.success = 1;
        return result;
    }

    result.has_measurement = 1;
    result.success = 1;
    return result;
}