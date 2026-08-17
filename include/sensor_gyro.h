#ifndef SENSOR_GYRO_H
#define SENSOR_GYRO_H

typedef struct {
    double roll_rate;
    double pitch_rate;
    double yaw_rate;
} gyro_input_t;

typedef struct {
    double roll_bias;
    double pitch_bias;
    double yaw_bias;

    double noise_std_dev;
} gyro_config_t;

typedef struct {
    double roll_rate;
    double pitch_rate;
    double yaw_rate;
} gyro_measurement_t;

typedef struct {
    int success;
    gyro_measurement_t measurement;
} gyro_result_t;

gyro_result_t update_gyro(
    const gyro_config_t *config,
    const gyro_input_t *input);

#endif