#include "odyssey.h"
#include "sensor_usbl.h"

#include "vehicle.h"

int update_usbl_sensor(void)
{
    usbl_config_t config;
    usbl_input_t input;
    usbl_result_t result;

    config.hit_interval = 0.5;
    config.hit_window = 0.02;
    config.min_direction_cos = 0.707;

    input.target_north = read_sensor(SIM_DOCK_N);
    input.target_east = read_sensor(SIM_DOCK_E);
    input.target_down = read_sensor(SIM_DOCK_D);

    input.vehicle_north = read_sensor(REAL_X);
    input.vehicle_east = read_sensor(REAL_Y);
    input.vehicle_down = read_sensor(REAL_Z);
    input.vehicle_yaw = read_sensor(M_YAW);

    input.sim_time = read_sensor(M_PRESENT_TIME);

    result = update_usbl(&config, &input);

    if (!result.success) return 0;
    if (!result.has_measurement) return 1;

    update_sensor(M_USBL_RANGE, result.measurement.range);
    update_sensor(M_USBL_AZIMUTH, result.measurement.azimuth);
    update_sensor(M_USBL_ELEVATION, result.measurement.elevation);

    return 1;
}