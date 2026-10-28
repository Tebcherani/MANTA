#ifndef SENSOR_LBL_H
#define SENSOR_LBL_H

#define LBL_MAX_BEACONS 4

typedef struct {
    double north_m;
    double east_m;
    double depth_m;
    double turnaround_time_s;
} lbl_beacon_t;


typedef struct {
    double sound_speed_mps;
    int num_beacons;
    lbl_beacon_t beacons[LBL_MAX_BEACONS + 1];
    double cycle_interval_s;
    double timeout_s;
    double bias_probability;
    double dropout_probability;
    double bias_amplitude_ms;
    double noise_amplitude_ms;
} lbl_config_t;


typedef struct {
    double present_time_s;
    int enabled;
    double vehicle_north_m;
    double vehicle_east_m;
    double vehicle_depth_m;
} lbl_input_t;


typedef struct {
    double next_ping_s;
    double ping_time_s;
    int cycle_number;
    int update_pending_b;
    int got_reply_b[LBL_MAX_BEACONS + 1];
    double travel_time_s[LBL_MAX_BEACONS + 1];

    double ping_north_m;
    double ping_east_m;
    double ping_depth_m;
} lbl_state_t;


typedef struct {
    int success;
    int ping_started;
    int new_data;

    double ping_time_s;
    double travel_time_s[LBL_MAX_BEACONS + 1];
} lbl_result_t;


int init_lbl_state(lbl_state_t *state);
lbl_result_t update_lbl(lbl_state_t *state, const lbl_config_t *config, const lbl_input_t *input);

#endif