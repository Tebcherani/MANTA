#ifndef WHALE_DETECTION_SIM__WHALE_DETECTION_STATE_HPP_
#define WHALE_DETECTION_SIM__WHALE_DETECTION_STATE_HPP_

#include <cstdint>
#include <random>
#include <string>
#include <unordered_map>
#include <vector>

#include "acoustics_interfaces/msg/whale_detection.hpp"
#include "acoustics_interfaces/msg/bearing_observation.hpp"
#include "acoustics_interfaces/msg/whale_call_processed.hpp"
#include "manta_interfaces/msg/vehicle_state.hpp"
#include "whale_interfaces/msg/whale_call.hpp"

// Configuration and runtime inputs used by the whale detector.
struct WhaleDetectionState {
    // Configuration
    std::vector<std::string> vehicle_ids;
    double detection_range_m = 11000.0;
    double bearing_std_rad = 0.0;
    std::int64_t bearing_noise_seed = 0;
    std::string data_directory = "/odyssey/output/Data";

    // Detection noise
    std::mt19937_64 rng;
    std::normal_distribution<double> normal{0.0, 1.0};

    // Simulation
    bool shutdown_requested_b = false;
    bool pending_whale_call_available_b = false;
    std::vector<std::string> bearing_receiver_ids;
    std::unordered_map<std::string, manta_interfaces::msg::VehicleState> latest_vehicle_states;    
    whale_interfaces::msg::WhaleCall received_whale_call;
    whale_interfaces::msg::WhaleCall pending_whale_call;
    std::string current_vehicle_id;
    double vehicle_to_whale_range_m = 0.0;
    bool bearing_observation_available_b = false;
    acoustics_interfaces::msg::BearingObservation bearing_observation;
    acoustics_interfaces::msg::WhaleCallProcessed whale_call_processed;
    bool whale_detection_available_b = false;
    acoustics_interfaces::msg::WhaleDetection whale_detection;
};

#endif  // WHALE_DETECTION_SIM__WHALE_DETECTION_STATE_HPP_
