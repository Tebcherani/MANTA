#include "whale_detection_sim/whale_detection_node.hpp"

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

    // Load the maximum three-dimensional whale-detection distance.
    detection_range_ = declare_parameter<double>("detection_range", 500.0);  
    if (detection_range_ <= 0.0) throw std::runtime_error("detection_range must be greater than zero.");
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
    detection_log_ << "cycle,sim_time,vehicle_id,whale_id,vehicle_north,vehicle_east,vehicle_depth,whale_north,whale_east,whale_depth,distance\n";
}