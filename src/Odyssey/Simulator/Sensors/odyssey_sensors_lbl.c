#include "odyssey.h"
#include "odyssey_sensors.h"
#include "sensor_lbl.h"
#include "sim_log.h"

#include "vehicle.h"


static lbl_state_t lbl_state;
static int lbl_initialized = 0;


static int init_odyssey_lbl_state(void) {
    if (lbl_initialized) return 1;

    if (!init_lbl_state(&lbl_state)) {
        write_to_sim_log("[odyssey_sensors_lbl] Failed to initialize LBL state.\n");
        return 0;
    }

    lbl_initialized = 1;
    return 1;
}


void update_lbl_sensor(void) {
    lbl_config_t config;
    lbl_input_t input;
    lbl_result_t result;
    int beacon_index, offset;

    if (!init_odyssey_lbl_state()) return;

    config.sound_speed = read_sensor(U_SPEED_OF_SOUND);
    config.num_beacons = (int)read_sensor(U_LBL_BEACON_NUMBER);
    config.cycle_interval = read_sensor(U_LBL_CYCLE);
    config.timeout = read_sensor(U_LBL_TIMEOUT);
    config.bias_probability = read_sensor(REAL_LBL_FRAC_BIAS);
    config.dropout_probability = read_sensor(REAL_LBL_FRAC_DROPOUT);
    config.bias_amplitude_ms = read_sensor(REAL_LBL_BIAS_AMPLITUDE);
    config.noise_amplitude_ms = read_sensor(REAL_LBL_TIMING_NOISE);

    for (beacon_index = 1, offset = 0; beacon_index <= config.num_beacons; beacon_index++) {
        config.beacons[beacon_index].north = read_sensor(U_LBL_1NORTH + offset);
        config.beacons[beacon_index].east = read_sensor(U_LBL_1EAST + offset);
        config.beacons[beacon_index].depth = read_sensor(U_LBL_1DEPTH + offset);
        config.beacons[beacon_index].turnaround_time = read_sensor(U_LBL_1TAT + offset);
        offset += U_LBL_2NORTH - U_LBL_1NORTH;
    }

    input.present_time = read_sensor(M_PRESENT_TIME);
    input.enabled = read_sensor(C_LBL_CONTROL) > 0;
    input.vehicle_north = read_sensor(REAL_X);
    input.vehicle_east = read_sensor(REAL_Y);
    input.vehicle_depth = read_sensor(REAL_Z);

    result = update_lbl(&lbl_state, &config, &input);

    if (!result.success) {
        write_to_sim_log("[odyssey_sensors_lbl] Failed to update LBL sensor.\n");
        return;
    }

    if (result.ping_started) {
        update_sensor(M_LBL_PING_TIME, result.ping_time);
        for (beacon_index = 0; beacon_index <= config.num_beacons; beacon_index++) update_sensor(M_LBL_0T + beacon_index, result.travel_time[beacon_index]);
    }

    if (result.new_data) {
        for (beacon_index = 1; beacon_index <= config.num_beacons; beacon_index++) update_sensor(M_LBL_0T + beacon_index, result.travel_time[beacon_index]);
        update_sensor(LBL_NEWDATA, 1);
    }
}