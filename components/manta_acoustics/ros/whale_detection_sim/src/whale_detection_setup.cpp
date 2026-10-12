#include "whale_detection_sim/whale_detection_node.hpp"

#include <cmath>
#include <cstdint>
#include <filesystem>
#include <stdexcept>

/**
 * Load and validate the whale-detection configuration.
 *
 * @throws std::runtime_error If no vehicles are configured or the detection range is not positive.
 */
void WhaleDetectionNode::load_config() {
    // Load the vehicles whose states will be evaluated for whale detections.
    vehicle_ids_ = declare_parameter<std::vector<std::string>>("vehicle_ids", std::vector<std::string>{});
    if (vehicle_ids_.empty()) throw std::runtime_error("vehicle_ids must contain at least one vehicle.");

    // Load the maximum horizontal whale-detection distance in metres.
    detection_range_ = declare_parameter<double>("detection_range", 11000.0);
    if (!std::isfinite(detection_range_) || detection_range_ <= 0.0) throw std::runtime_error("detection_range must be finite and greater than zero.");

    // Configure angular measurement uncertainty.
    const double bearing_std_degrees = declare_parameter<double>("bearing_std_degrees", 0.0);
    if (!std::isfinite(bearing_std_degrees) || bearing_std_degrees < 0.0) throw std::runtime_error("bearing_std_degrees must be finite and nonnegative.");
    bearing_std_rad_ = bearing_std_degrees * std::acos(-1.0) / 180.0;

    // Initialize detection noise parameters
    position_std_m_ = declare_parameter<double>("call_position_std_m", 0.0);
    if (!std::isfinite(position_std_m_) || position_std_m_ < 0.0) throw std::runtime_error("call_position_std_m must be finite and nonnegative.");

    const auto seed = declare_parameter<int64_t>("call_position_seed", 0);
    if (seed < 0) throw std::runtime_error("call_position_seed cannot be negative.");
    rng_.seed(static_cast<std::uint64_t>(seed));
}

/**
 * Open the CSV file used to record whale detections.
 *
 * @throws std::runtime_error If the detection results file cannot be opened.
 */
void WhaleDetectionNode::open_detection_log() {
    // Load the directory where simulation results will be written.
    const std::string data_directory = declare_parameter<std::string>("data_directory", "/odyssey/output/Data");

    // Ensure the output directory exists before opening the results file.
    std::filesystem::create_directories(data_directory);
    detection_log_.open(data_directory + "/whale_detections.csv", std::ios::out | std::ios::trunc);
    if (!detection_log_.is_open()) throw std::runtime_error("Failed to open whale detection results file.");
    
    // Write the column names before recording any detection events.
    detection_log_ << "cycle,sim_time,vehicle_id,whale_id,vehicle_north,vehicle_east,whale_north,whale_east,distance\n";
}