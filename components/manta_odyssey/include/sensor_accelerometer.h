#ifndef SENSOR_ACCELEROMETER_H
#define SENSOR_ACCELEROMETER_H

typedef struct {
    double mount_x_m;
    double mount_y_m;
    double mount_z_m;
} accelerometer_config_t;

typedef struct {
    double velocity_x_mps;
    double velocity_y_mps;
    double velocity_z_mps;

    double acceleration_x_mps2;
    double acceleration_y_mps2;
    double acceleration_z_mps2;

    double angular_rate_x_radps;
    double angular_rate_y_radps;
    double angular_rate_z_radps;

    double angular_acceleration_x_radps2;
    double angular_acceleration_y_radps2;
    double angular_acceleration_z_radps2;

    double pitch_rad;
    double roll_rad;

    double gravity_mps2;
} true_vehicle_motion_t;

typedef struct {
    double accel_x_mps2;
    double accel_y_mps2;
    double accel_z_mps2;
} accelerometer_measurement_t;

typedef struct {
    int success;
    accelerometer_measurement_t measurement;
} accelerometer_result_t;

accelerometer_result_t update_accelerometer(const accelerometer_config_t *config, const true_vehicle_motion_t *vehicle);

#endif