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
    if (metrics_written_) return true;

    // Retrieve final metrics from the native Odyssey simulator.
    odyssey_metrics_t metrics;

    if (!get_odyssey_metrics(&metrics)) return false;

    // Construct the final and temporary output paths.
    const std::filesystem::path directory(data_directory_);
    const std::filesystem::path final_path = directory / (vehicle_id_ + "_metrics.csv");
    const std::filesystem::path temporary_path = directory / (vehicle_id_ + "_metrics.csv.tmp");

    // Ensure the configured output directory exists.
    std::error_code error;
    std::filesystem::create_directories(directory, error);

    if (error) return false;

    // Write to a temporary file so incomplete output is never treated as final.
    std::ofstream file(temporary_path, std::ios::out | std::ios::trunc);

    if (!file.is_open()) return false;

    file << "vehicle_id,mass,energy_consumed,mean_speed\n";
    file << vehicle_id_ << ',' << std::setprecision(15) << metrics.mass << ',' << metrics.energy_consumed << ',' << metrics.mean_speed << '\n';

    // Explicitly close the stream so write failures are detected before rename.
    file.close();

    if (!file) return false;

    // Replace any previous result with the newly completed metrics file.
    std::filesystem::remove(final_path, error);
    if (error) return false;
    std::filesystem::rename(temporary_path, final_path, error);
    if (error) return false;
    metrics_written_ = true;

    RCLCPP_INFO(get_logger(),"Saved final vehicle metrics: %s", final_path.string().c_str());

    return true;
}

/**
 * Publish the vehicle state after a completed simulation batch.
 *
 * @param completed_cycle Final simulation cycle completed by the vehicle.
 * @param completed_sim_time Simulation time corresponding to that cycle.
 */
void OdysseyNode::publish_vehicle_state(std::uint64_t completed_cycle, double completed_sim_time) {
    manta_interfaces::msg::VehicleState state;

    // Identify the vehicle and completed simulation point.
    state.vehicle_id = vehicle_id_;
    state.cycle = completed_cycle;
    state.sim_time = completed_sim_time;

    // Read the vehicle's current position from the native Odyssey simulator.
    state.north = get_odyssey_sim_north();
    state.east = get_odyssey_sim_east();
    state.depth = get_odyssey_sim_depth();

    // Make the completed state available to the detection node.
    vehicle_state_pub_->publish(state);
}