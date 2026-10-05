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
    result.measurement.accel_x = vehicle->acceleration_x + vehicle->angular_rate_y * vehicle->velocity_z - vehicle->angular_rate_z * vehicle->velocity_y
        + vehicle->angular_acceleration_y * config->mount_z - vehicle->angular_acceleration_z * config->mount_y 
        - (vehicle->angular_rate_y * vehicle->angular_rate_y + vehicle->angular_rate_z * vehicle->angular_rate_z) * config->mount_x
        + vehicle->angular_rate_x * vehicle->angular_rate_y * config->mount_y + vehicle->angular_rate_x * vehicle->angular_rate_z * config->mount_z;

    result.measurement.accel_y = vehicle->acceleration_y + vehicle->angular_rate_z * vehicle->velocity_x - vehicle->angular_rate_x * vehicle->velocity_z
        + vehicle->angular_acceleration_z * config->mount_x - vehicle->angular_acceleration_x * config->mount_z 
        - (vehicle->angular_rate_x * vehicle->angular_rate_x + vehicle->angular_rate_z * vehicle->angular_rate_z) * config->mount_y
        + vehicle->angular_rate_x * vehicle->angular_rate_y * config->mount_x + vehicle->angular_rate_y * vehicle->angular_rate_z * config->mount_z;

    result.measurement.accel_z = vehicle->acceleration_z + vehicle->angular_rate_x * vehicle->velocity_y - vehicle->angular_rate_y * vehicle->velocity_x
        + vehicle->angular_acceleration_x * config->mount_y - vehicle->angular_acceleration_y * config->mount_x
        - (vehicle->angular_rate_x * vehicle->angular_rate_x + vehicle->angular_rate_y * vehicle->angular_rate_y) * config->mount_z
        + vehicle->angular_rate_y * vehicle->angular_rate_z * config->mount_y + vehicle->angular_rate_x * vehicle->angular_rate_z * config->mount_x;

    // Add the gravity components that an accelerometer mounted in the vehicle body frame would measure
    result.measurement.accel_x += sin(vehicle->pitch) * vehicle->gravity;
    result.measurement.accel_y -= cos(vehicle->pitch) * sin(vehicle->roll) * vehicle->gravity;
    result.measurement.accel_z -= cos(vehicle->pitch) * cos(vehicle->roll) * vehicle->gravity;

    result.success = 1;
    return result;
}