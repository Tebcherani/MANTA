#ifndef SENSOR_INCLINOMETER_H
#define SENSOR_INCLINOMETER_H

typedef struct {
    // Filter history for pitch and roll
    // Index 0 = pitch
    // Index 1 = roll
    double previous_raw[2][3];
    double previous_filtered[2][3];
    int sample_count;
} inclinometer_state_t;

typedef struct {
    double accel_x;
    double accel_y;
    double gravity;
} inclinometer_input_t;

typedef struct {
    double pitch;
    double roll;
} inclinometer_measurement_t;

typedef struct {
    int success;
    inclinometer_measurement_t measurement;
} inclinometer_result_t;

int init_inclinometer_state(
    inclinometer_state_t *state);

inclinometer_result_t update_inclinometer(
    inclinometer_state_t *state,
    const inclinometer_input_t *input);

#endif