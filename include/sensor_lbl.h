#ifndef SENSOR_LBL_H
#define SENSOR_LBL_H

#define LBL_MAX_BEACONS 4

typedef struct {
    double north;
    double east;
    double depth;
    double turnaround_time;
} lbl_beacon_t;


typedef struct {
    double sound_speed;
    int num_beacons;
    lbl_beacon_t beacons[LBL_MAX_BEACONS + 1];
    double cycle_interval;
    double timeout;
    double bias_probability;
    double dropout_probability;
    double bias_amplitude_ms;
    double noise_amplitude_ms;
} lbl_config_t;


typedef struct {
    double present_time;
    int enabled;
    double vehicle_north;
    double vehicle_east;
    double vehicle_depth;
} lbl_input_t;


typedef struct {
    double next_ping;
    double ping_time;
    int cycle_number;
    int update_pending;
    int got_reply[LBL_MAX_BEACONS + 1];
    double travel_time[LBL_MAX_BEACONS + 1];

    double ping_north;
    double ping_east;
    double ping_depth;
} lbl_state_t;


typedef struct {
    int success;
    int ping_started;
    int new_data;

    double ping_time;
    double travel_time[LBL_MAX_BEACONS + 1];
} lbl_result_t;


int init_lbl_state(lbl_state_t *state);
lbl_result_t update_lbl(lbl_state_t *state, const lbl_config_t *config, const lbl_input_t *input);

#endif