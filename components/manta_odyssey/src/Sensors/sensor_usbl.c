#include <math.h>
#include <stddef.h>

#include "angle_utils.h"
#include "sensor_usbl.h"
#include "sim_log.h"

usbl_result_t update_usbl(const usbl_config_t *config, const usbl_input_t *input) {
    usbl_result_t result;
    double north_difference_m;
    double east_difference_m;
    double down_difference_m;
    double horizontal_range_m;

    result.success = 0;
    result.has_measurement = 0;
    result.measurement.range_m = 0.0;
    result.measurement.azimuth_rad = 0.0;
    result.measurement.elevation_rad = 0.0;

    if (config == NULL) {
        write_to_sim_log("[sensor_usbl] Failed to update USBL: config pointer is NULL.\n");
        return result;
    }

    if (input == NULL) {
        write_to_sim_log("[sensor_usbl] Failed to update USBL: input pointer is NULL.\n");
        return result;
    }

    if (config->hit_interval_s <= 0.0) {
        write_to_sim_log("[sensor_usbl] Failed to update USBL: hit interval must be greater than zero.\n");
        return result;
    }

    north_difference_m = input->target_north_m - input->vehicle_north_m;
    east_difference_m = input->target_east_m - input->vehicle_east_m;
    down_difference_m = input->target_down_m - input->vehicle_down_m;

    horizontal_range_m = sqrt(north_difference_m * north_difference_m + east_difference_m * east_difference_m);

    result.measurement.range_m = sqrt(north_difference_m * north_difference_m + east_difference_m * east_difference_m + down_difference_m * down_difference_m);
    result.measurement.azimuth_rad = wrap_angle(atan2(east_difference_m, north_difference_m) - input->vehicle_yaw_rad);
    result.measurement.elevation_rad = atan2(down_difference_m, horizontal_range_m);

    // Produces a hit once every hit_interval, provided the current timestep falls within hit_window
    if (fmod(input->sim_time_s, config->hit_interval_s) >= config->hit_window_s) {
        result.success = 1;
        return result;
    }

    if (cos(result.measurement.elevation_rad) <= config->min_direction_cos || cos(result.measurement.azimuth_rad) <= config->min_direction_cos) {
        result.success = 1;
        return result;
    }

    result.has_measurement = 1;
    result.success = 1;
    return result;
}