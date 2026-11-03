#include "odyssey_sim/odyssey_node.hpp"

#include <filesystem>
#include <fstream>
#include <iomanip>

extern "C" {
    #include "odyssey_sim.h"
}

/**
 * Write the vehicle's final mass, energy, and speed metrics to a CSV file.
 *
 * The output is first written to a temporary file and then renamed so another process cannot read a partially written metrics file.
 *
 * @return true if the metrics were already written or were written successfully; otherwise, false.
 */
bool OdysseyNode::write_final_metrics() {
    // Do not write the same final metrics more than once.
    if (odyssey_state_.metrics_written_b) return true;

    // Retrieve final metrics from the native Odyssey simulator.
    odyssey_metrics_t metrics;

    if (!get_odyssey_metrics(&metrics)) return false;

    // Construct the final and temporary output paths.
    const std::filesystem::path directory(odyssey_state_.data_directory);
    const std::filesystem::path final_path = directory / (odyssey_state_.vehicle_id + "_metrics.csv");
    const std::filesystem::path temporary_path = directory / (odyssey_state_.vehicle_id + "_metrics.csv.tmp");

    // Ensure the configured output directory exists.
    std::error_code error;
    std::filesystem::create_directories(directory, error);

    if (error) return false;

    // Write to a temporary file so incomplete output is never treated as final.
    std::ofstream file(temporary_path, std::ios::out | std::ios::trunc);

    if (!file.is_open()) return false;

    file << "vehicle_id,mass,energy_consumed,mean_speed\n";
    file << odyssey_state_.vehicle_id << ',' << std::setprecision(15) << metrics.mass_kg << ',' << metrics.energy_consumed_Wh << ',' << metrics.mean_speed_mps << '\n';

    // Explicitly close the stream so write failures are detected before rename.
    file.close();

    if (!file) return false;

    // Replace any previous result with the newly completed metrics file.
    std::filesystem::remove(final_path, error);
    if (error) return false;
    std::filesystem::rename(temporary_path, final_path, error);
    if (error) return false;
    odyssey_state_.metrics_written_b = true;

    RCLCPP_INFO(get_logger(),"Saved final vehicle metrics: %s", final_path.string().c_str());

    return true;
}

/**
 * Publish the vehicle state after a completed simulation batch.
 *
 * Reads the completed cycle and simulation time from Odyssey state.
 */
void OdysseyNode::publish_vehicle_state() {
    manta_interfaces::msg::VehicleState state;

    // Identify the vehicle and completed simulation point.
    state.vehicle_id = odyssey_state_.vehicle_id;
    state.cycle = odyssey_state_.completed_cycle;
    state.sim_time_s = odyssey_state_.sim_time_s;

    // Read the vehicle's current position from the native Odyssey simulator.
    state.vehicle_north_m = get_odyssey_sim_north();
    state.vehicle_east_m = get_odyssey_sim_east();
    state.depth = get_odyssey_sim_depth();

    // Save the position and time used by the bearing detector.
    odyssey_state_.bearing_positions.push_back({state.cycle, state.sim_time_s, state.vehicle_north_m, state.vehicle_east_m});

    // Make the completed state available to the detection node.
    vehicle_state_pub_->publish(state);
}

void OdysseyNode::open_whale_prediction_log() {
    if (!odyssey_state_.tracking_enabled_b) return;

    std::error_code error;
    std::filesystem::create_directories(odyssey_state_.data_directory, error);
    if (error) {
        throw std::runtime_error("Failed to create prediction-log directory.");
    }

    const std::filesystem::path path =
        std::filesystem::path(odyssey_state_.data_directory) /
        (odyssey_state_.vehicle_id + "_whale_predictions.csv");

    whale_prediction_log_.open(path, std::ios::out | std::ios::trunc);
    if (!whale_prediction_log_.is_open()) {
        throw std::runtime_error("Failed to open whale-prediction log.");
    }

    whale_prediction_log_
        << "sim_time_s,has_estimate,measurement_time,predicted_north,predicted_east\n";
}

void OdysseyNode::log_whale_prediction() {
    const double simulation_time_s = odyssey_state_.sim_time_s;
    if (!whale_prediction_log_.is_open()) return;
    if (simulation_time_s + 1.0e-9 < odyssey_state_.next_prediction_log_time_s) return;

    while (odyssey_state_.next_prediction_log_time_s <= simulation_time_s + 1.0e-9) {
        odyssey_state_.next_prediction_log_time_s += odyssey_state_.output_sample_period_s;
    }

    double measurement_time_s = 0.0;
    double predicted_north_m = 0.0;
    double predicted_east_m = 0.0;

    const bool has_estimate = get_odyssey_sim_whale_prediction(simulation_time_s, &measurement_time_s, &predicted_north_m, &predicted_east_m);

    whale_prediction_log_ << std::setprecision(17)
                          << simulation_time_s << ','
                          << (has_estimate ? 1 : 0) << ',';

    if (has_estimate) {
        whale_prediction_log_ << measurement_time_s << ','
                              << predicted_north_m << ','
                              << predicted_east_m;
    } else {
        whale_prediction_log_ << ",,";
    }

    whale_prediction_log_ << '\n';
}