#include <math.h>
#include <stddef.h>

#include "sensor_accelerometer.h"
#include "sim_log.h"

accelerometer_result_t update_accelerometer(const accelerometer_config_t *config, const true_vehicle_motion_t *vehicle) {
    accelerometer_result_t result;
    
    if (config == NULL) {
        write_to_sim_log("[sensor_accelerometer] Failed to update accelerometer: config pointer is NULL.\n");
        result.success = 0;
        return result;
    }

    if (vehicle == NULL) {
        write_to_sim_log("[sensor_accelerometer] Failed to update accelerometer: vehicle motion pointer is NULL.\n");
        result.success = 0;
        return result;
    }

    // Compute acceleration at the physical accelerometer mounting location
    result.measurement.accel_x_mps2 = vehicle->acceleration_x_mps2 + vehicle->angular_rate_y_radps * vehicle->velocity_z_mps - vehicle->angular_rate_z_radps * vehicle->velocity_y_mps
        + vehicle->angular_acceleration_y_radps2 * config->mount_z_m - vehicle->angular_acceleration_z_radps2 * config->mount_y_m
        - (vehicle->angular_rate_y_radps * vehicle->angular_rate_y_radps + vehicle->angular_rate_z_radps * vehicle->angular_rate_z_radps) * config->mount_x_m
        + vehicle->angular_rate_x_radps * vehicle->angular_rate_y_radps * config->mount_y_m + vehicle->angular_rate_x_radps * vehicle->angular_rate_z_radps * config->mount_z_m;

    result.measurement.accel_y_mps2 = vehicle->acceleration_y_mps2 + vehicle->angular_rate_z_radps * vehicle->velocity_x_mps - vehicle->angular_rate_x_radps * vehicle->velocity_z_mps
        + vehicle->angular_acceleration_z_radps2 * config->mount_x_m - vehicle->angular_acceleration_x_radps2 * config->mount_z_m
        - (vehicle->angular_rate_x_radps * vehicle->angular_rate_x_radps + vehicle->angular_rate_z_radps * vehicle->angular_rate_z_radps) * config->mount_y_m
        + vehicle->angular_rate_x_radps * vehicle->angular_rate_y_radps * config->mount_x_m + vehicle->angular_rate_y_radps * vehicle->angular_rate_z_radps * config->mount_z_m;

    result.measurement.accel_z_mps2 = vehicle->acceleration_z_mps2 + vehicle->angular_rate_x_radps * vehicle->velocity_y_mps - vehicle->angular_rate_y_radps * vehicle->velocity_x_mps
        + vehicle->angular_acceleration_x_radps2 * config->mount_y_m - vehicle->angular_acceleration_y_radps2 * config->mount_x_m
        - (vehicle->angular_rate_x_radps * vehicle->angular_rate_x_radps + vehicle->angular_rate_y_radps * vehicle->angular_rate_y_radps) * config->mount_z_m
        + vehicle->angular_rate_y_radps * vehicle->angular_rate_z_radps * config->mount_y_m + vehicle->angular_rate_x_radps * vehicle->angular_rate_z_radps * config->mount_x_m;

    // Add the gravity components that an accelerometer mounted in the vehicle body frame would measure
    result.measurement.accel_x_mps2 += sin(vehicle->pitch_rad) * vehicle->gravity_mps2;
    result.measurement.accel_y_mps2 -= cos(vehicle->pitch_rad) * sin(vehicle->roll_rad) * vehicle->gravity_mps2;
    result.measurement.accel_z_mps2 -= cos(vehicle->pitch_rad) * cos(vehicle->roll_rad) * vehicle->gravity_mps2;

    result.success = 1;
    return result;
}