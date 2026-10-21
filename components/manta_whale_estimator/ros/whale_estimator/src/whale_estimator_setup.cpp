#include "whale_estimator/whale_estimator_node.hpp"
#include "whale_estimator/whale_estimator_state_json.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>

void WhaleEstimatorNode::setup() {
    load_config();
    state_log_.open_in_log_directory(vehicle_id_ + "_whale_estimator_state.jsonl");
    state_log_.record(state_);
}

/**
 * Load and validate the vehicle identity and estimator tuning parameters.
 * Parameters are declared here so the node's setup and configuration remain
 * separate from its runtime callbacks.
 */
void WhaleEstimatorNode::load_config() {
    vehicle_id_ = declare_parameter<std::string>("vehicle_id", "");
    if (vehicle_id_.empty()) throw std::runtime_error("vehicle_id must be set.");
    
    auto &config = state_.config;
    const double missing = std::numeric_limits<double>::quiet_NaN();
    config.velocity_window_max_age_s = declare_parameter<double>("velocity_window_max_age_s", missing);
    config.acceleration_psd_m2_s3 = declare_parameter<double>("acceleration_psd_m2_s3", missing);
    config.velocity_return_time_s = declare_parameter<double>("whale_velocity_return_time_s", missing);
    config.nominal_north_velocity_mps = declare_parameter<double>("whale_baseline_velocity_north_mps", missing);
    config.nominal_east_velocity_mps = declare_parameter<double>("whale_baseline_velocity_east_mps", missing);
    config.initial_velocity_variance_m2ps2 = declare_parameter<double>("initial_velocity_variance_m2ps2", missing);
    
    if (!std::isfinite(config.velocity_window_max_age_s) || config.velocity_window_max_age_s <= 0.0 ||
        !std::isfinite(config.acceleration_psd_m2_s3) || config.acceleration_psd_m2_s3 < 0.0 ||
        !std::isfinite(config.velocity_return_time_s) || config.velocity_return_time_s <= 0.0 ||
        !std::isfinite(config.nominal_north_velocity_mps) || !std::isfinite(config.nominal_east_velocity_mps) ||
        !std::isfinite(config.initial_velocity_variance_m2ps2) || config.initial_velocity_variance_m2ps2 < 0.0) {
        throw std::runtime_error("Invalid whale estimator configuration.");
    }
    
    state_.configured_b = 1;
}
