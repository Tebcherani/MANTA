#include "odyssey_sim/odyssey_node.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

extern "C" {
    #include "odyssey.h"
    #include "odyssey_behaviors.h"
    #include "odyssey_sim.h"
    #include "world_sim.h"
}

//----------------------------------------------------
// Handle functions
//----------------------------------------------------

void OdysseyNode::handle_shared_bearing(const acoustics_interfaces::msg::BearingTransmission::SharedPtr msg) {
    if (!odyssey_state_.tracking_enabled_b) return;
    if (msg->receiver_id == odyssey_state_.vehicle_id) return;
    if (msg->receiver_id != "auv_1" && msg->receiver_id != "auv_2") return;

    const std::uint64_t call_cycle = msg->call_cycle;

    // Process each call only once.
    if (odyssey_state_.processed_bearing_cycles.count(call_cycle) != 0) return;

    const auto local_it = odyssey_state_.local_bearings_by_cycle.find(call_cycle);

    if (local_it == odyssey_state_.local_bearings_by_cycle.end()) {
        // Wait for our measurement of this exact call.
        odyssey_state_.pending_shared_bearings[call_cycle] = msg;
        return;
    }

    const auto &local = local_it->second;
    const auto &remote = *msg;

    if (!std::isfinite(local.bearing_rad) || !std::isfinite(remote.bearing_rad) || !std::isfinite(local.receiver_north_m) || !std::isfinite(local.receiver_east_m) || !std::isfinite(remote.receiver_north_m) || !std::isfinite(remote.receiver_east_m)) {
        RCLCPP_WARN(get_logger(), "Cannot estimate from non-finite bearings.");
        return;
    }
    if (!std::isfinite(local.bearing_std_rad) || local.bearing_std_rad < 0.0 ||
        !std::isfinite(remote.bearing_std_rad) || remote.bearing_std_rad < 0.0) {
        RCLCPP_WARN(get_logger(), "Cannot estimate position uncertainty from invalid bearing standard deviations.");
        return;
    }

    /*
    * Intersect two bearing rays in the north/east plane.
    *
    * Let p_local and p_remote be the AUV positions at measurement time.
    * A bearing theta, measured clockwise from north, gives the unit
    * direction vector u = (cos(theta), sin(theta)).
    *
    * The rays are:
    *   p_local  + r_local  * u_local
    *   p_remote + r_remote * u_remote
    *
    * At their intersection:
    *   p_remote - p_local = r_local * u_local - r_remote * u_remote = displacement
    *
    * Define the 2D cross product:
    *   cross(a, b) = a_n * b_e - a_e * b_n
    * 
    * Cross product measures how "parallel vs. perpendicular" two vectors are (0 = parallel). 
    * Take the cross product of both sides with u_remote:
    *   r_local * cross(u_local, u_remote) - r_remote * cross(u_remote, u_remote) = cross(disp, u_remote)
    *   => r_local * cross(u_local, u_rempte) - r_remote * 0 = cross(disp, u_remote)
    *   => r_local * cross(u_local, u_remote) = cross(disp, u_remote)
    *   => r_local = cross(disp, u_remote) / cross(u_local, u_remote)
    * 
    * Note that determinant = cross(u_local, u_remote). Then:
    *   r_local  = cross(displacement, u_remote) / determinant
    *   r_remote = cross(displacement, u_local)  / determinant
    *
    * The estimated position is:
    *   p_estimated = p_local + r_local * u_local
    */

    // Bearing directions, clockwise from north. These are unit vectors in the north and east directions.
    const double local_n = std::cos(local.bearing_rad);
    const double local_e = std::sin(local.bearing_rad);

    const double remote_n = std::cos(remote.bearing_rad);
    const double remote_e = std::sin(remote.bearing_rad);

    // Use each receiver's position when its bearing was measured.
    const double disp_n_m = remote.receiver_north_m - local.receiver_north_m;
    const double disp_e_m = remote.receiver_east_m - local.receiver_east_m;

    const double determinant = local_n * remote_e - local_e * remote_n;

    if (!std::isfinite(determinant) ||
        std::abs(determinant) < 1.0e-6) {
        RCLCPP_WARN(get_logger(), "Cannot triangulate nearly parallel bearing lines.");
        return;
    }

    const double local_range_m = (disp_n_m * remote_e - disp_e_m * remote_n) / determinant;
    const double remote_range_m = (disp_n_m * local_e - disp_e_m * local_n) / determinant;

    if (!std::isfinite(local_range_m) || !std::isfinite(remote_range_m) || local_range_m < 0.0 || remote_range_m < 0.0) {
        RCLCPP_WARN(get_logger(), "Bearing rays have no valid forward intersection.");
        return;
    }

    const double estimated_north_m = local.receiver_north_m + local_range_m * local_n;
    const double estimated_east_m = local.receiver_east_m + local_range_m * local_e;

    // Propagate each independent bearing's angular variance through the
    // intersection equations to get north/east position measurement variance.
    const double local_tangent_n = -local_e;
    const double local_tangent_e = local_n;
    const double remote_tangent_n = -remote_e;
    const double remote_tangent_e = remote_n;
    const double range_numerator = disp_n_m * remote_e - disp_e_m * remote_n;
    const double range_numerator_remote_derivative = disp_n_m * remote_n + disp_e_m * remote_e;
    const double determinant_local_derivative = local_tangent_n * remote_e - local_tangent_e * remote_n;
    const double determinant_remote_derivative = local_n * remote_tangent_e - local_e * remote_tangent_n;
    const double determinant_squared = determinant * determinant;
    const double local_range_local_derivative = -range_numerator * determinant_local_derivative / determinant_squared;
    const double local_range_remote_derivative =
        (range_numerator_remote_derivative * determinant - range_numerator * determinant_remote_derivative) / determinant_squared;
    const double north_local_derivative = local_range_local_derivative * local_n + local_range_m * local_tangent_n;
    const double east_local_derivative = local_range_local_derivative * local_e + local_range_m * local_tangent_e;
    const double north_remote_derivative = local_range_remote_derivative * local_n;
    const double east_remote_derivative = local_range_remote_derivative * local_e;
    const double local_bearing_variance_rad2 = local.bearing_std_rad * local.bearing_std_rad;
    const double remote_bearing_variance_rad2 = remote.bearing_std_rad * remote.bearing_std_rad;
    const double north_variance_m2 = north_local_derivative * north_local_derivative * local_bearing_variance_rad2 +
        north_remote_derivative * north_remote_derivative * remote_bearing_variance_rad2;
    const double east_variance_m2 = east_local_derivative * east_local_derivative * local_bearing_variance_rad2 +
        east_remote_derivative * east_remote_derivative * remote_bearing_variance_rad2;

    if (!std::isfinite(estimated_north_m) ||
        !std::isfinite(estimated_east_m) || !std::isfinite(north_variance_m2) ||
        !std::isfinite(east_variance_m2) || north_variance_m2 < 0.0 || east_variance_m2 < 0.0) {
        return;
    }

    // Record the call only after triangulation succeeds.
    odyssey_state_.processed_bearing_cycles.insert(call_cycle);

    // Both bearings describe the same call; use its measurement time.
    odyssey_interfaces::msg::WhaleBearingPositionEstimate fix;
    fix.vehicle_id = odyssey_state_.vehicle_id;
    fix.call_cycle = call_cycle;
    fix.sim_time_s = local.sim_time_s;
    fix.north_m = estimated_north_m;
    fix.east_m = estimated_east_m;
    fix.north_variance_m2 = north_variance_m2;
    fix.east_variance_m2 = east_variance_m2;
    whale_fix_pub_->publish(fix);

    RCLCPP_INFO(get_logger(), "Vehicle '%s' received a bearing from '%s': whale estimate north %.2f m, east %.2f m at %.2f s.", odyssey_state_.vehicle_id.c_str(), msg->receiver_id.c_str(), estimated_north_m, estimated_east_m, odyssey_state_.sim_time_s);
}

void OdysseyNode::handle_whale_estimate(const whale_estimator_interfaces::msg::WhaleKalmanEstimate::SharedPtr msg) {
    if (msg->vehicle_id != odyssey_state_.vehicle_id) return;
    set_whale_trailing_estimate(msg->whale_kalman_north_m, msg->whale_kalman_east_m, msg->measurement_time_s,
        msg->whale_kalman_velocity_north_mps, msg->whale_kalman_velocity_east_mps,
        msg->whale_kalman_velocity_variance_north_m2ps2, msg->whale_kalman_velocity_variance_east_m2ps2,
        msg->whale_baseline_velocity_north_mps, msg->whale_baseline_velocity_east_mps,
        msg->whale_velocity_return_time_s);
}

/**
 * Shut down the Odyssey ROS node when requested by the orchestrator.
 *
 * False shutdown messages are ignored. A true value begins the normal ROS 2 shutdown process, which eventually invokes the OdysseyNode destructor.
 *
 * @param msg ROS message indicating whether simulation shutdown was requested.
 */
void OdysseyNode::handle_shutdown(const std_msgs::msg::Bool::SharedPtr msg) {
    if (!msg->data) return; // Ignore messages that do not request shutdown.
    RCLCPP_INFO(get_logger(), "Vehicle '%s' received simulation shutdown.", odyssey_state_.vehicle_id.c_str());
    rclcpp::shutdown();     // Stop the ROS executor and begin normal node destruction.
}

/**
 * Advance the native Odyssey simulation through one requested batch of cycles.
 *
 * The callback converts the ROS world-state message to Odyssey's native world structure, updates vehicle dynamics at every simulation step, runs control
 * logic at the configured control period, publishes the final vehicle state, and acknowledges the completed batch to the orchestrator.
 *
 * @param msg ROS message describing the simulation batch to execute.
 */
void OdysseyNode::handle_sim_step(const manta_interfaces::msg::SimStep::SharedPtr msg) {
    manta_interfaces::msg::SimStepResult result;
    world_state_t world_state;

    // Prepare the result returned to the orchestrator.
    result.vehicle_id = odyssey_state_.vehicle_id;
    result.completed_cycle = msg->current_cycle;
    result.status = manta_interfaces::msg::SimStepResult::RUNNING;
    result.detail = "";

    // A completed vehicle no longer participates in simulation cycles.
    if (odyssey_state_.mission_finished_b) return;

    // Convert the ROS world snapshot to Odyssey's native world structure.
    world_state.gravity_mps2 = msg->world.gravity_mps2;
    world_state.magnetic_north_gauss = msg->world.magnetic_north_gauss;
    world_state.magnetic_east_gauss = msg->world.magnetic_east_gauss;
    world_state.magnetic_down_gauss = msg->world.magnetic_down_gauss;
    world_state.seawater_density_kgpm3 = msg->world.seawater_density_kgpm3;

    // Validate values used by the cycle loop and modulo operation.
    if (msg->control_period == 0) {
        result.status = manta_interfaces::msg::SimStepResult::ERROR;
        result.detail = "Simulation control period cannot be zero.";
        step_result_pub_->publish(result);
        return;
    }

    if (msg->step_count == 0) {
        result.status = manta_interfaces::msg::SimStepResult::ERROR;
        result.detail = "Simulation step count cannot be zero.";
        step_result_pub_->publish(result);
        return;
    }

    // Convert the requested output period to a number of control updates.
    if (!odyssey_state_.log_decimation_configured_b) {
        const double control_interval_s = static_cast<double>(msg->control_period) * msg->timestep_s;
        const unsigned int log_decimation = static_cast<unsigned int>(std::max(1.0, std::ceil(odyssey_state_.output_sample_period_s / control_interval_s)));

        set_odyssey_log_decimation(log_decimation);
        odyssey_state_.log_decimation_configured_b = true;

        RCLCPP_INFO(
            get_logger(), "Vehicle '%s' will record output every %.3f simulated seconds (%u control updates).", 
            odyssey_state_.vehicle_id.c_str(), static_cast<double>(log_decimation) * control_interval_s, log_decimation
        );
    }

    // Run every native Odyssey cycle contained in this ROS batch.
    for (std::uint32_t offset = 0; offset < msg->step_count; ++offset) {
        const std::uint64_t cycle = msg->current_cycle + offset;
        result.completed_cycle = cycle;

        if (odyssey_abort_requested()) {
            odyssey_state_.mission_finished_b = true;
            result.status = manta_interfaces::msg::SimStepResult::ABORTED;
            result.detail = "Odyssey requested mission abort.";
            step_result_pub_->publish(result);
            return;
        }

        // Dynamics continue running at the native simulation timestep.
        if (!update_odyssey_sim_state(&world_state, msg->timestep_s)) {
            result.status = manta_interfaces::msg::SimStepResult::ERROR;
            result.detail = "Failed to update Odyssey simulation state.";
            step_result_pub_->publish(result);
            return;
        }

        // Run sensors, navigation, control, and mission logic when due.
        if (cycle % msg->control_period == 0) {
            const double control_timestep_s = static_cast<double>(msg->control_period) * msg->timestep_s;
            const odyssey_update_status_t cycle_status = update_odyssey_sim_cycle(&world_state, control_timestep_s);

            if (cycle_status == ODYSSEY_UPDATE_ERROR) {
                result.status = manta_interfaces::msg::SimStepResult::ERROR;
                result.detail = "Failed to update Odyssey control cycle.";
                step_result_pub_->publish(result);
                return;
            }

            if (cycle_status == ODYSSEY_UPDATE_MISSION_COMPLETE) {
                odyssey_state_.mission_finished_b = true;
                result.status = manta_interfaces::msg::SimStepResult::MISSION_COMPLETE;
                result.detail = "Odyssey mission completed.";
                step_result_pub_->publish(result);
                return;
            }
        }

        const double next_sim_time_s = msg->sim_time_s + static_cast<double>(offset + 1) * msg->timestep_s;

        if (!update_odyssey_sim_time(next_sim_time_s)) {
            result.status = manta_interfaces::msg::SimStepResult::ERROR;
            result.detail = "Failed to update Odyssey simulation time.";
            step_result_pub_->publish(result);
            return;
        }

        odyssey_state_.sim_time_s = next_sim_time_s;
        log_whale_prediction(next_sim_time_s);
        publish_bearing(next_sim_time_s);
    }

    // Publish the vehicle state produced by the completed batch.
    result.completed_cycle = msg->current_cycle + msg->step_count - 1;
    const double completed_sim_time_s = msg->sim_time_s + static_cast<double>(msg->step_count) * msg->timestep_s;
    publish_vehicle_state(result.completed_cycle, completed_sim_time_s);

    // Acknowledge the completed batch after publishing its resulting state.
    step_result_pub_->publish(result);
}

/**
 * Initialize Odyssey when the orchestrator announces that startup is ready.
 * 
 * Duplicate or false startup messages are ignored after Odyssey has already been initialized.
 *
 * Reads the startup-ready value stored in state.
 * 
 * @throws std::runtime_error if the native Odyssey simulator cannot initialize.
 */
void OdysseyNode::handle_startup() {
    if (!odyssey_state_.startup_ready_b || odyssey_state_.odyssey_initialized_b) return;                                                 // Ignore false messages and duplicate startup notifications.
    if (!init_odyssey_sim()) throw std::runtime_error("Failed to initialize " + odyssey_state_.vehicle_id + "."); // Initialize the native Odyssey simulator.
    odyssey_state_.odyssey_initialized_b = true;                                                                    // Record successful initialization.
    RCLCPP_INFO(get_logger(), "Initialized '%s' simulator.", odyssey_state_.vehicle_id.c_str());
    try_finish_init();                                                                              // Complete startup if the required time values have also arrived.
}

void OdysseyNode::handle_bearing_observation(const acoustics_interfaces::msg::BearingObservation::SharedPtr msg) {
    if (!odyssey_state_.tracking_enabled_b || msg->receiver_id != odyssey_state_.vehicle_id) return;

    // Find this vehicle's position at the measurement time.
    const auto position = std::find_if(odyssey_state_.bearing_positions.rbegin(), odyssey_state_.bearing_positions.rend(), [&msg](const BearingPosition &sample) {return sample.cycle == msg->call_cycle;});

    if (position == odyssey_state_.bearing_positions.rend()) {
        RCLCPP_ERROR(get_logger(), "No vehicle position available for bearing at cycle %llu.", static_cast<unsigned long long>(msg->call_cycle));
        return;
    }

    // Share the sensor measurement with this vehicle's own position.
    acoustics_interfaces::msg::BearingTransmission shared;
    shared.receiver_id = odyssey_state_.vehicle_id;
    shared.call_cycle = msg->call_cycle;
    shared.sim_time_s = msg->sim_time_s;
    shared.bearing_rad = msg->bearing_rad;
    shared.bearing_std_rad = msg->bearing_std_rad;
    shared.receiver_north_m = position->north_m;
    shared.receiver_east_m = position->east_m;

    odyssey_state_.latest_bearing = shared;
    odyssey_state_.latest_bearing_available_b = true;
    odyssey_state_.latest_bearing_unsent_b = true;

    // Retain this local measurement for a matching remote bearing.
    odyssey_state_.local_bearings_by_cycle[shared.call_cycle] = shared;

    // A remote measurement of this call may already be waiting.
    const auto pending_it = odyssey_state_.pending_shared_bearings.find(shared.call_cycle);

    if (pending_it != odyssey_state_.pending_shared_bearings.end()) {
        auto pending = pending_it->second;
        odyssey_state_.pending_shared_bearings.erase(pending_it);
        handle_shared_bearing(pending);
    }

    if (odyssey_state_.communication_interval_s > 0.0) {
        // Confirm local storage so the orchestrator can continue.
        acoustics_interfaces::msg::BearingProcessed stored;
        stored.vehicle_id = odyssey_state_.vehicle_id;
        stored.call_cycle = msg->call_cycle;
        stored.sim_time_s = msg->sim_time_s;
        stored.estimate_valid_b = false;
        bearing_processed_pub_->publish(stored);

        return;
    }

    // Zero interval preserves immediate sharing and estimation.
    bearing_pub_->publish(shared);
    odyssey_state_.latest_bearing_unsent_b = false;

    // Acknowledge reception even when only one AUV detected the call.
    acoustics_interfaces::msg::BearingProcessed received;
    received.vehicle_id = odyssey_state_.vehicle_id;
    received.call_cycle = msg->call_cycle;
    received.sim_time_s = msg->sim_time_s;
    received.estimate_valid_b = false;
    bearing_processed_pub_->publish(received);
}

//----------------------------------------------------
// Helper functions
//----------------------------------------------------

/**
 * Complete staged Odyssey initialization when all required startup data is available.
 *
 * Initialization occurs incrementally because the native Odyssey simulator, initial simulation time, and calendar start time arrive through separate
 * startup operations and ROS messages. This method safely returns until the required state is available, initializes the native simulation clock, starts
 * the Odyssey simulator, and finally notifies the orchestrator that the vehicle is ready.
 *
 * @throws std::runtime_error If the native simulation time cannot be initialized or the Odyssey simulator cannot be started.
 */
void OdysseyNode::try_finish_init() {
    if (!odyssey_state_.odyssey_initialized_b) return;
    if (!odyssey_state_.sim_time_received_b) return;

    // Initialize the native Odyssey simulation clock once.
    if (!odyssey_state_.sim_time_initialized_b) {
        if (!update_odyssey_sim_time(odyssey_state_.sim_time_s)) throw std::runtime_error("Failed to initialize " + odyssey_state_.vehicle_id + " simulation time.");
        odyssey_state_.sim_time_initialized_b = true;
        RCLCPP_INFO(get_logger(), "Initialized '%s' simulation time.", odyssey_state_.vehicle_id.c_str());
    }

    if (!odyssey_state_.calendar_start_time_received_b) return;

    // Start the native Odyssey simulator once all startup data is available.
    if (!odyssey_state_.odyssey_sim_started_b) {
        if (!start_odyssey_sim(odyssey_state_.calendar_start_time)) throw std::runtime_error("Failed to start " + odyssey_state_.vehicle_id + " simulator.");
        odyssey_state_.odyssey_sim_started_b = true;
        RCLCPP_INFO(get_logger(), "Started '%s' simulator.", odyssey_state_.vehicle_id.c_str());
    }

    // Notify the orchestrator once the vehicle is fully initialized.
    if (!odyssey_state_.ready_published_b) {
        std_msgs::msg::Bool ready_message;        
        ready_message.data = true;
        ready_pub_->publish(ready_message);
        odyssey_state_.ready_published_b = true;
        RCLCPP_INFO(get_logger(), "Vehicle '%s' initialized and ready.", odyssey_state_.vehicle_id.c_str());
    }
}

void OdysseyNode::publish_bearing(double simulation_time_s) {
    if (!odyssey_state_.tracking_enabled_b || odyssey_state_.communication_interval_s <= 0.0) return;

    while (
        simulation_time_s + 1.0e-9 >=
        static_cast<double>(odyssey_state_.next_communication_index) * odyssey_state_.communication_interval_s
    ) {
        if (odyssey_state_.latest_bearing_available_b && odyssey_state_.latest_bearing_unsent_b) {
            bearing_pub_->publish(odyssey_state_.latest_bearing);
            odyssey_state_.latest_bearing_unsent_b = false;
        }

        ++odyssey_state_.next_communication_index;
    }
}
