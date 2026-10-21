#ifndef MANTA_WHALE_ESTIMATOR_STATE_HPP
#define MANTA_WHALE_ESTIMATOR_STATE_HPP

#include <cstdint>
#include <string>

#include "whale_estimator_types.hpp"

/* Configuration, latest input/output, and filter history for one vehicle. */
typedef struct {
    whale_estimator_config_t config;
    int estimator_configured_b;
    int shutdown_requested_b;
    int received_shutdown_b;

    int estimator_initialized_b;
    double measurement_time_s;
    whale_estimate_one_axis_t north;
    whale_estimate_one_axis_t east;
    whale_bearing_position_t recent_fixes[WHALE_VELOCITY_WINDOW_SIZE];
    int recent_fix_count;

    whale_bearing_position_t received_bearing_whale_position;
    std::string received_vehicle_id;
    std::uint64_t received_call_cycle;
    int bearing_whale_position_accepted_b;

    /* Working values for the current north/east axis filter update. */
    whale_estimate_one_axis_t axis_update;
    double axis_update_elapsed_s;
    int axis_update_is_north_b;
} whale_estimator_state_t;

#endif /* MANTA_WHALE_ESTIMATOR_STATE_HPP */
