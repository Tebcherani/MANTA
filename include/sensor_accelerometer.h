#ifndef SENSOR_ACCELEROMETER_H
#define SENSOR_ACCELEROMETER_H

typedef struct {
    double mount_x;
    double mount_y;
    double mount_z;
} accelerometer_config_t;

typedef struct {
    double velocity_x;
    double velocity_y;
    double velocity_z;

    double acceleration_x;
    double acceleration_y;
    double acceleration_z;

    double angular_rate_x;
    double angular_rate_y;
    double angular_rate_z;

    double angular_acceleration_x;
    double angular_acceleration_y;
    double angular_acceleration_z;

    double pitch;
    double roll;

    double gravity;
} true_vehicle_motion_t;

typedef struct {
    double accel_x;
    double accel_y;
    double accel_z;
} accelerometer_measurement_t;

typedef struct {
    int success;
    accelerometer_measurement_t measurement;
} accelerometer_result_t;

accelerometer_result_t update_accelerometer(const accelerometer_config_t *config, const true_vehicle_motion_t *vehicle);

#endif