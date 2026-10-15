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
    detection_range_m_ = declare_parameter<double>("detection_range_m", 11000.0);
    if (!std::isfinite(detection_range_m_) || detection_range_m_ <= 0.0) throw std::runtime_error("detection_range_m must be finite and greater than zero.");

    // Configure angular measurement uncertainty.
    const double bearing_std_deg = declare_parameter<double>("bearing_std_deg", 0.0);
    if (!std::isfinite(bearing_std_deg) || bearing_std_deg < 0.0) throw std::runtime_error("bearing_std_deg must be finite and nonnegative.");
    bearing_std_rad_ = bearing_std_deg * std::acos(-1.0) / 180.0;

    const auto seed = declare_parameter<int64_t>("bearing_noise_seed", 0);
    if (seed < 0) throw std::runtime_error("bearing_noise_seed cannot be negative.");
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
    whale_detection_log_.open(data_directory + "/whale_detections.csv", std::ios::out | std::ios::trunc);
    if (!whale_detection_log_.is_open()) throw std::runtime_error("Failed to open whale detection results file.");
    
    // Write the column names before recording any detection events.
    whale_detection_log_ << "cycle,sim_time_s,vehicle_id,whale_id,vehicle_north_m,vehicle_east_m,whale_north_m,whale_east_m,vehicle_to_whale_range_m\n";
}