#include "odyssey.h"
#include "sensor_usbl.h"

#include "vehicle.h"

int update_usbl_sensor(void)
{
    usbl_config_t config;
    usbl_input_t input;
    usbl_result_t result;

    config.hit_interval_s = 0.5;
    config.hit_window_s = 0.02;
    config.min_direction_cos = 0.707;

    input.target_north_m = read_sensor(SIM_DOCK_N);
    input.target_east_m = read_sensor(SIM_DOCK_E);
    input.target_down_m = read_sensor(SIM_DOCK_D);

    input.vehicle_north_m = read_sensor(REAL_X);
    input.vehicle_east_m = read_sensor(REAL_Y);
    input.vehicle_down_m = read_sensor(REAL_Z);
    input.vehicle_yaw_rad = read_sensor(M_YAW);

    input.sim_time_s = read_sensor(M_PRESENT_TIME);

    result = update_usbl(&config, &input);

    if (!result.success) return 0;
    if (!result.has_measurement) return 1;

    update_sensor(M_USBL_RANGE, result.measurement.range_m);
    update_sensor(M_USBL_AZIMUTH, result.measurement.azimuth_rad);
    update_sensor(M_USBL_ELEVATION, result.measurement.elevation_rad);

    return 1;
}