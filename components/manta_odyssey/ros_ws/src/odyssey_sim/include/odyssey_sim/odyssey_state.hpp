#ifndef ODYSSEY_SIM__ODYSSEY_STATE_HPP_
#define ODYSSEY_SIM__ODYSSEY_STATE_HPP_

#include <cstdint>
#include <ctime>
#include <deque>
#include <map>
#include <set>
#include <string>

#include "acoustics_interfaces/msg/bearing_transmission.hpp"
#include "acoustics_interfaces/msg/bearing_observation.hpp"
#include "acoustics_interfaces/msg/whale_call_processed.hpp"
#include "manta_interfaces/msg/sim_step.hpp"
#include "manta_interfaces/msg/sim_step_result.hpp"
#include "whale_estimator_interfaces/msg/whale_kalman_estimate.hpp"

struct OdysseyState {
    struct BearingPosition {
        std::uint64_t cycle;
        double sim_time_s;
        double north_m;
        double east_m;
    };

    // Call detections
    std::deque<BearingPosition> bearing_positions;
    bool latest_bearing_available_b = false;
    bool latest_bearing_unsent_b = false;
    acoustics_interfaces::msg::BearingTransmission latest_bearing;
    bool last_received_local_bearing_available_b = false;
    acoustics_interfaces::msg::BearingObservation last_received_local_bearing;
    bool last_received_shared_bearing_available_b = false;
    acoustics_interfaces::msg::BearingTransmission last_received_shared_bearing;
    bool last_received_whale_estimate_available_b = false;
    whale_estimator_interfaces::msg::WhaleKalmanEstimate last_received_whale_estimate;

    // Local measurements retained so delayed remote bearings can find their match.
    std::map<std::uint64_t, acoustics_interfaces::msg::BearingTransmission> local_bearings_by_cycle;

    // Remote measurements that arrived before the corresponding local measurement.
    std::map<std::uint64_t, acoustics_interfaces::msg::BearingTransmission::SharedPtr> pending_shared_bearings;

    // Calls already successfully triangulated.
    std::set<std::uint64_t> processed_bearing_cycles;
    std::set<std::uint64_t> triangulation_attempted_cycles;
    manta_interfaces::msg::SimStepResult pending_step_result;
    bool pending_step_result_b = false;
    bool detector_complete_for_step_b = false;
    bool bearing_received_for_step_b = false;
    bool bearing_expected_for_step_b = false;
    bool triangulation_expected_for_step_b = false;
    bool peer_exchange_pending_b = false;
    std::uint64_t peer_exchange_cycle = 0;
    std::uint64_t current_physics_cycle = 0;
    acoustics_interfaces::msg::WhaleCallProcessed received_whale_call_processed;

    double communication_interval_s = 0.0;
    std::uint64_t next_communication_index = 1;

    // Mission
    bool log_decimation_configured_b = false;
    bool metrics_written_b = false;
    bool mission_finished_b = false;
    bool tracking_enabled_b = false;

    // Simulation
    bool calendar_start_time_received_b = false;
    bool odyssey_initialized_b = false;
    bool odyssey_sim_started_b = false;
    bool ready_published_b = false;
    bool sim_time_initialized_b = false;
    bool sim_time_received_b = false;
    bool startup_ready_b = false;

    manta_interfaces::msg::SimStep pending_sim_step;
    std::uint64_t completed_cycle = 0;
    time_t calendar_start_time = 0;
    double sim_time_s = 0.0;

    // Vehicle
    std::string vehicle_id;
    std::string data_directory;
    double output_sample_period_s = 2.0;
    double next_prediction_log_time_s = 0.0;
};

#endif
