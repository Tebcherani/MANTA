#ifndef SENSOR_GYRO_H
#define SENSOR_GYRO_H

typedef struct {
    double roll_rate_radps;
    double pitch_rate_radps;
    double yaw_rate_radps;
} gyro_input_t;

typedef struct {
    double roll_bias_radps;
    double pitch_bias_radps;
    double yaw_bias_radps;

    double noise_std_dev_radps;
} gyro_config_t;

typedef struct {
    double roll_rate_radps;
    double pitch_rate_radps;
    double yaw_rate_radps;
} gyro_measurement_t;

typedef struct {
    int success;
    gyro_measurement_t measurement;
} gyro_result_t;

gyro_result_t update_gyro(
    const gyro_config_t *config,
    const gyro_input_t *input);

#endif