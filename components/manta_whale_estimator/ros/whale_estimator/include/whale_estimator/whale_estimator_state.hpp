#ifndef MANTA_WHALE_ESTIMATOR_STATE_HPP
#define MANTA_WHALE_ESTIMATOR_STATE_HPP

#include "whale_estimator_types.hpp"

/* Configuration, latest input/output, and filter history for one vehicle. */
typedef struct {
    whale_estimator_config_t config;
    int configured_b;
    int shutdown_requested_b;

    int initialized_b;
    double measurement_time_s;
    whale_axis_filter_t north;
    whale_axis_filter_t east;
    whale_position_fix_t recent_fixes[WHALE_VELOCITY_WINDOW_SIZE];
    int recent_fix_count;

    whale_position_fix_t received_fix;
    int fix_accepted_b;
    double prediction_time_s;
    whale_prediction_t prediction;
} whale_estimator_state_t;

#endif /* MANTA_WHALE_ESTIMATOR_STATE_HPP */
