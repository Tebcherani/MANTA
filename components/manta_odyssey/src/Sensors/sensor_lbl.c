#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#include "random_utils.h"
#include "sensor_lbl.h"
#include "sim_log.h"

static double calculate_distance(double, double, double, double, double, double);

static double calculate_distance(double north_1_m, double east_1_m, double depth_1_m, double north_2_m, double east_2_m, double depth_2_m) {
    double north_difference_m = north_1_m - north_2_m;
    double east_difference_m = east_1_m - east_2_m;
    double depth_difference_m = depth_1_m - depth_2_m;

    return sqrt(north_difference_m * north_difference_m + east_difference_m * east_difference_m + depth_difference_m * depth_difference_m);
}


int init_lbl_state(lbl_state_t *state) {
    if (state == NULL) {
        write_to_sim_log("[sensor_lbl] Failed to initialize LBL state: state pointer is NULL.\n");
        return 0;
    }

    memset(state, 0, sizeof(*state));
    state->next_ping_s = 0.0;
    state->ping_time_s = 0.0;

    write_to_sim_log("[sensor_lbl] LBL state initialized.\n");

    return 1;
}


lbl_result_t update_lbl(lbl_state_t *state, const lbl_config_t *config, const lbl_input_t *input) {
    lbl_result_t result;

    int beacon_index;
    double since_ping_s;
    double outgoing_distance_m, return_distance_m;
    double time_of_flight_s;
    double noise_s, bias_s;

    memset(&result, 0, sizeof(result));

    if (state == NULL) {
        write_to_sim_log("[sensor_lbl] Failed to update LBL: state pointer is NULL.\n");
        return result;
    }

    if (config == NULL) {
        write_to_sim_log("[sensor_lbl] Failed to update LBL: config pointer is NULL.\n");
        return result;
    }

    if (input == NULL) {
        write_to_sim_log("[sensor_lbl] Failed to update LBL: input pointer is NULL.\n");
        return result;
    }

    if (config->sound_speed_mps <= 0.0) {
        write_to_sim_log("[sensor_lbl] Failed to update LBL: sound speed must be greater than zero.\n");
        return result;
    }

    if (
        config->num_beacons < 0 ||
        config->num_beacons > LBL_MAX_BEACONS) {

        write_to_sim_log("[sensor_lbl] Failed to update LBL: invalid number of beacons.\n");
        return result;
    }

    if (input->present_time_s >= state->next_ping_s) {
        state->next_ping_s += config->cycle_interval_s;

        if (input->enabled) {
            state->ping_time_s = input->present_time_s;

            state->cycle_number++;
            state->update_pending_b = 1;

            state->ping_north_m = input->vehicle_north_m;
            state->ping_east_m = input->vehicle_east_m;
            state->ping_depth_m = input->vehicle_depth_m;

            memset(state->got_reply_b, 0, sizeof(state->got_reply_b));

            result.ping_started = 1;
            result.ping_time_s = state->ping_time_s;

            for (beacon_index = 0; beacon_index <= config->num_beacons; beacon_index++) result.travel_time_s[beacon_index] = -1.0;
        }
    }

    since_ping_s = input->present_time_s - state->ping_time_s;

    if (input->present_time_s > state->ping_time_s && since_ping_s < config->timeout_s && state->cycle_number > 0) {
        for (beacon_index = 1; beacon_index <= config->num_beacons; beacon_index++) {
            if (!state->got_reply_b[beacon_index]) {
                outgoing_distance_m = calculate_distance(state->ping_north_m, state->ping_east_m, state->ping_depth_m,
                        config->beacons[beacon_index].north_m, config->beacons[beacon_index].east_m, config->beacons[beacon_index].depth_m);
                return_distance_m = calculate_distance(input->vehicle_north_m, input->vehicle_east_m, input->vehicle_depth_m,
                        config->beacons[beacon_index].north_m, config->beacons[beacon_index].east_m, config->beacons[beacon_index].depth_m);

                time_of_flight_s = (outgoing_distance_m + return_distance_m) / config->sound_speed_mps + config->beacons[beacon_index].turnaround_time_s;
                
                noise_s = std_normal_random_num() * config->noise_amplitude_ms / 1000.0;
                time_of_flight_s += noise_s;

                if (drand48() < config->bias_probability) {
                    bias_s = drand48() * config->bias_amplitude_ms /1000.0;
                    time_of_flight_s += bias_s;
                }

                if (time_of_flight_s < since_ping_s) {
                    state->got_reply_b[beacon_index] = 1;

                    if (drand48() < config->dropout_probability) state->travel_time_s[beacon_index] = 0.0;
                    else state->travel_time_s[beacon_index] = time_of_flight_s;
                }
            }
        }
    }

    if (state->update_pending_b && since_ping_s > config->timeout_s) {
        result.new_data = 1;
        for (beacon_index = 1; beacon_index <= config->num_beacons; beacon_index++) result.travel_time_s[beacon_index] = state->travel_time_s[beacon_index];
        state->update_pending_b = 0;
    }

    result.success = 1;
    return result;
}