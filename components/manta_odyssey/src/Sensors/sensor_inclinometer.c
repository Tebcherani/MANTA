#include <math.h>
#include <stddef.h>

#include "random_utils.h"
#include "sensor_inclinometer.h"
#include "sim_log.h"

static void low_pass_filter(inclinometer_state_t *, const double [2], double [2]);

int init_inclinometer_state(inclinometer_state_t *state) {
    int axis, sample;

    if (state == NULL) {
        write_to_sim_log("[sensor_inclinometer] Failed to initialize inclinometer: state pointer is NULL.\n");
        return 0;
    }

    state->sample_count = 1;

    for (axis = 0; axis < 2; axis++) {
        for (sample = 0; sample < 3; sample++) {
            state->previous_raw[axis][sample] = 0.0;
            state->previous_filtered[axis][sample] = 0.0;
        }
    }

    write_to_sim_log("[sensor_inclinometer] Inclinometer state initialized.\n");

    return 1;
}


static void low_pass_filter(inclinometer_state_t *state, const double raw[2], double filtered[2]) {
    const double a1 = -1.82269492519631;
    const double a2 = 0.83718165127602;

    const double b0 = 0.00362168151493;
    const double b1 = 0.00724336302986;
    const double b2 = 0.00362168151493;

    int axis, j;

    for (axis = 0; axis <= 1; axis++) {
        if (state->sample_count == 1) {
            state->previous_raw[axis][1] = raw[axis];
            state->previous_filtered[axis][1] = raw[axis];

            filtered[axis] = raw[axis];
        }

        if (state->sample_count == 2) {
            state->previous_raw[axis][2] = state->previous_raw[axis][1];
            state->previous_filtered[axis][2] = state->previous_filtered[axis][1];

            state->previous_raw[axis][1] = raw[axis];
            state->previous_filtered[axis][1] = raw[axis];

            filtered[axis] = raw[axis];
        }

        if (state->sample_count > 2) {
            filtered[axis] = b0 * raw[axis];

            for (j = 1; j <= 2; j++) {
                filtered[axis] += (j == 1 ? b1 : b2) * state->previous_raw[axis][j];
                filtered[axis] -= (j == 1 ? a1 : a2) * state->previous_filtered[axis][j];
            }

            for (j = 2; j > 1; j--) {
                state->previous_raw[axis][j] = state->previous_raw[axis][j - 1];
                state->previous_filtered[axis][j] = state->previous_filtered[axis][j - 1];
            }

            state->previous_raw[axis][1] = raw[axis];
            state->previous_filtered[axis][1] = filtered[axis];
        }
    }

    if (state->sample_count <= 2) {
        state->sample_count++;
    }
}


inclinometer_result_t update_inclinometer(inclinometer_state_t *state, const inclinometer_input_t *input) {
    const double inclinometer_noise = 0.001745;
    inclinometer_result_t result;
    double raw_pitch_roll[2];
    double filtered_pitch_roll[2];

    result.success = 0;
    result.measurement.pitch = 0.0;
    result.measurement.roll = 0.0;

    if (state == NULL) {
        write_to_sim_log("[sensor_inclinometer] Failed to update inclinometer: state pointer is NULL.\n");
        return result;
    }

    if (input == NULL) {
        write_to_sim_log("[sensor_inclinometer] Failed to update inclinometer: input pointer is NULL.\n");
        return result;
    }

    if (input->gravity == 0.0) {
        write_to_sim_log("[sensor_inclinometer] Failed to update inclinometer: gravity is zero.\n");
        return result;
    }

    raw_pitch_roll[0] = asin(input->accel_x / input->gravity);
    raw_pitch_roll[1] = -asin(input->accel_y / (input->gravity * cos(raw_pitch_roll[0])));

    low_pass_filter(state, raw_pitch_roll, filtered_pitch_roll);

    result.measurement.pitch = filtered_pitch_roll[0] + inclinometer_noise * std_normal_random_num();
    result.measurement.roll = filtered_pitch_roll[1] + inclinometer_noise * std_normal_random_num();

    result.success = 1;
    return result;
}