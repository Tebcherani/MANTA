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

/**
 * Store the simulation's calendar start time and continue initialization.
 *
 * This callback runs when the orchestrator publishes the calendar timestamp used as the beginning of the simulated mission.
 * 
 *  @param msg ROS message containing the calendar start time as a Unix timestamp.
 */
void OdysseyNode::handle_calendar_start_time(const std_msgs::msg::Int64::SharedPtr msg) {
    calendar_start_time_ = static_cast<time_t>(msg->data);  // Convert the timestamp carried by the ROS message to the native time type.
    calendar_start_time_received_ = true;                   // Record that this required startup value has been received.
    try_finish_init();                                      // Complete startup if all other required initialization data is available.
}

/**
 * Collect the shared bearings needed by this vehicle's estimator.
 *
 * Measurements are grouped by call cycle and receiver ID. Once both AUV
 * measurements arrive, verify their call times before triangulation.
 *
 * @param msg Bearing and measurement-position data shared by an AUV.
 */
void OdysseyNode::handle_shared_bearing(const acoustics_interfaces::msg::BearingMsg::SharedPtr msg) {
    // Only tracking vehicles collect bearings for localization.
    if (!tracking_enabled_) return;

    // Retrieve or create the measurement pair for this call cycle.
    auto &pair = bearing_pairs_[msg->call_cycle];

    // Keep only the first measurement received from each AUV.
    if (pair.find(msg->receiver_id) != pair.end()) return;

    pair.emplace(msg->receiver_id, *msg);

    // Wait until both vehicles have shared their measurements.
    if (pair.count("auv_1") == 0 || pair.count("auv_2") == 0) return;

    const auto &first = pair.at("auv_1");
    const auto &second = pair.at("auv_2");

    // Report that this vehicle has finished processing this bearing pair.
    const auto acknowledge = [this, &msg](bool estimate_valid) {
        acoustics_interfaces::msg::BearingProcessed processed;
        processed.vehicle_id = vehicle_id_;
        processed.call_cycle = msg->call_cycle;
        processed.sim_time = msg->sim_time;
        processed.estimate_valid = estimate_valid;

        bearing_processed_pub_->publish(processed);
    };

    // Reject measurements with different call times.
    if (std::abs(first.sim_time - second.sim_time) > 1.0e-9) {
        RCLCPP_ERROR(get_logger(), "Bearings at cycle %llu have different call times.", static_cast<unsigned long long>(msg->call_cycle));
        acknowledge(false);
        bearing_pairs_.erase(msg->call_cycle);
        return;
    }

    // This vehicle now has the pair needed to estimate the whale position.
    RCLCPP_INFO(get_logger(), "Vehicle '%s' has both bearings for cycle %llu.", vehicle_id_.c_str(), static_cast<unsigned long long>(msg->call_cycle));

    // Convert bearings clockwise from north into unit direction vectors.
    const double first_n = std::cos(first.bearing_rad);
    const double first_e = std::sin(first.bearing_rad);
    const double second_n = std::cos(second.bearing_rad);
    const double second_e = std::sin(second.bearing_rad);

    // Calculate the displacement between the two measurement positions.
    const double baseline_n = second.receiver_north - first.receiver_north;
    const double baseline_e = second.receiver_east - first.receiver_east;

    // Nearly parallel bearing lines do not give a stable intersection.
    const double determinant = first_n * second_e - first_e * second_n;

    if (!std::isfinite(determinant) || std::abs(determinant) < 1.0e-6) {
        RCLCPP_WARN(get_logger(), "Cannot triangulate nearly parallel bearings.");
        acknowledge(false);
        bearing_pairs_.erase(msg->call_cycle);
        return;
    }

    // Solve for the distance along each vehicle's bearing ray.
    const double first_range = (baseline_n * second_e - baseline_e * second_n) / determinant;
    const double second_range = (baseline_n * first_e - baseline_e * first_n) / determinant;

    // A valid intersection must lie ahead of both receivers.
    if (!std::isfinite(first_range) || !std::isfinite(second_range) ||
        first_range < 0.0 || second_range < 0.0) {
        RCLCPP_WARN(get_logger(), "Bearing rays have no valid forward intersection.");
        acknowledge(false);
        bearing_pairs_.erase(msg->call_cycle);
        return;
    }

    // Estimate the whale position from the intersection.
    const double estimated_north = first.receiver_north + first_range * first_n;
    const double estimated_east = first.receiver_east + first_range * first_e;

    RCLCPP_INFO(get_logger(), "Vehicle '%s' estimated whale position: north %.2f m, east %.2f m.",vehicle_id_.c_str(), estimated_north, estimated_east);

    // Apply immediate updates, or retain the estimate for the next update boundary.
    if (call_update_interval_ <= 0.0) {
        set_whale_trailing_target(estimated_north, estimated_east);
    } else if (!pending_call_available_ || first.sim_time >= pending_call_time_) {
        pending_call_north_ = estimated_north;
        pending_call_east_ = estimated_east;
        pending_call_time_ = first.sim_time;
        pending_call_available_ = true;
    }

    // Acknowledge after the estimated target has been applied or stored.
    acknowledge(true);

    // Remove the collected pair after processing.
    bearing_pairs_.erase(msg->call_cycle);
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
    RCLCPP_INFO(get_logger(), "Vehicle '%s' received simulation shutdown.", vehicle_id_.c_str());
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
    result.vehicle_id = vehicle_id_;
    result.completed_cycle = msg->current_cycle;
    result.status = manta_interfaces::msg::SimStepResult::RUNNING;
    result.detail = "";

    // A completed vehicle no longer participates in simulation cycles.
    if (mission_finished_) return;

    // Schedule the first periodic call-waypoint update.
    if (tracking_enabled_ && call_update_interval_ > 0.0 && next_call_update_time_ <= 0.0) {
        next_call_update_time_ = msg->sim_time + call_update_interval_;
    }

    // Convert the ROS world snapshot to Odyssey's native world structure.
    world_state.gravity = msg->world.gravity;
    world_state.magnetic_north = msg->world.magnetic_north;
    world_state.magnetic_east = msg->world.magnetic_east;
    world_state.magnetic_down = msg->world.magnetic_down;
    world_state.seawater_density = msg->world.seawater_density;

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
    if (!log_decimation_configured_) {
        const double control_interval = static_cast<double>(msg->control_period) * msg->timestep;
        const unsigned int log_decimation = static_cast<unsigned int>(std::max(1.0, std::ceil(output_sample_period_ / control_interval)));

        set_odyssey_log_decimation(log_decimation);
        log_decimation_configured_ = true;

        RCLCPP_INFO(
            get_logger(), "Vehicle '%s' will record output every %.3f simulated seconds (%u control updates).", 
            vehicle_id_.c_str(), static_cast<double>(log_decimation) * control_interval, log_decimation
        );
    }

    // Run every native Odyssey cycle contained in this ROS batch.
    for (std::uint32_t offset = 0; offset < msg->step_count; ++offset) {
        const std::uint64_t cycle = msg->current_cycle + offset;
        result.completed_cycle = cycle;
        
        const double current_sim_time = msg->sim_time + static_cast<double>(offset) * msg->timestep;

        // At each configured update boundary, use the most recently received call.
        if (tracking_enabled_ && call_update_interval_ > 0.0 && current_sim_time + 1.0e-9 >= next_call_update_time_) {
            if (pending_call_available_) {
                set_whale_trailing_target(pending_call_north_, pending_call_east_);
                pending_call_available_ = false;

                RCLCPP_INFO(
                    get_logger(),
                    "Vehicle '%s' updated its waypoint at %.1f s using the latest call from %.1f s.",
                    vehicle_id_.c_str(),
                    current_sim_time,
                    pending_call_time_
                );
            }

            do {
                next_call_update_time_ += call_update_interval_;
            } while (next_call_update_time_ <= current_sim_time + 1.0e-9);
        }

        if (odyssey_abort_requested()) {
            mission_finished_ = true;
            result.status = manta_interfaces::msg::SimStepResult::ABORTED;
            result.detail = "Odyssey requested mission abort.";
            step_result_pub_->publish(result);
            return;
        }

        // Dynamics continue running at the native simulation timestep.
        if (!update_odyssey_sim_state(&world_state, msg->timestep)) {
            result.status = manta_interfaces::msg::SimStepResult::ERROR;
            result.detail = "Failed to update Odyssey simulation state.";
            step_result_pub_->publish(result);
            return;
        }

        // Run sensors, navigation, control, and mission logic when due.
        if (cycle % msg->control_period == 0) {
            const double control_timestep = static_cast<double>(msg->control_period) * msg->timestep;
            const odyssey_update_status_t cycle_status = update_odyssey_sim_cycle(&world_state, control_timestep);

            if (cycle_status == ODYSSEY_UPDATE_ERROR) {
                result.status = manta_interfaces::msg::SimStepResult::ERROR;
                result.detail = "Failed to update Odyssey control cycle.";
                step_result_pub_->publish(result);
                return;
            }

            if (cycle_status == ODYSSEY_UPDATE_MISSION_COMPLETE) {
                mission_finished_ = true;
                result.status = manta_interfaces::msg::SimStepResult::MISSION_COMPLETE;
                result.detail = "Odyssey mission completed.";
                step_result_pub_->publish(result);
                return;
            }
        }

        const double next_sim_time = msg->sim_time + static_cast<double>(offset + 1) * msg->timestep;

        if (!update_odyssey_sim_time(next_sim_time)) {
            result.status = manta_interfaces::msg::SimStepResult::ERROR;
            result.detail = "Failed to update Odyssey simulation time.";
            step_result_pub_->publish(result);
            return;
        }
    }

    // Publish the vehicle state produced by the completed batch.
    result.completed_cycle = msg->current_cycle + msg->step_count - 1;
    const double completed_sim_time = msg->sim_time + static_cast<double>(msg->step_count) * msg->timestep;
    publish_vehicle_state(result.completed_cycle, completed_sim_time);

    // Acknowledge the completed batch after publishing its resulting state.
    step_result_pub_->publish(result);
}

/**
 * Store the initial simulation time and continue Odyssey initialization.
 *
 * This callback receives the current simulated mission time from the orchestrator during startup.
 *
 * @param msg ROS message containing the current simulation time in seconds.
 */
void OdysseyNode::handle_sim_time(const std_msgs::msg::Float64::SharedPtr msg) {
    sim_time_ = msg->data;       // Store the initial simulated mission time.
    sim_time_received_ = true;   // Record that this required startup value has arrived.
    try_finish_init();           // Complete startup if the other required values are available.
}

/**
 * Initialize Odyssey when the orchestrator announces that startup is ready.
 * 
 * Duplicate or false startup messages are ignored after Odyssey has already been initialized.
 *
 * @param msg ROS message whose value indicates whether startup may begin.
 * 
 * @throws std::runtime_error if the native Odyssey simulator cannot initialize.
 */
void OdysseyNode::handle_startup(const std_msgs::msg::Bool::SharedPtr msg) {
    if (!msg->data || odyssey_initialized_) return;                                                 // Ignore false messages and duplicate startup notifications.
    if (!init_odyssey_sim()) throw std::runtime_error("Failed to initialize " + vehicle_id_ + "."); // Initialize the native Odyssey simulator.
    odyssey_initialized_ = true;                                                                    // Record successful initialization.
    RCLCPP_INFO(get_logger(), "Initialized '%s' simulator.", vehicle_id_.c_str());
    try_finish_init();                                                                              // Complete startup if the required time values have also arrived.
}

void OdysseyNode::handle_bearing_observation(const acoustics_interfaces::msg::BearingObservation::SharedPtr msg) {
    if (!tracking_enabled_ || msg->receiver_id != vehicle_id_) return;

    // Find this vehicle's position at the measurement time.
    const auto position = std::find_if(bearing_positions_.rbegin(), bearing_positions_.rend(), [&msg](const BearingPosition &sample) {return sample.cycle == msg->call_cycle;});

    if (position == bearing_positions_.rend()) {
        RCLCPP_ERROR(get_logger(), "No vehicle position available for bearing at cycle %llu.", static_cast<unsigned long long>(msg->call_cycle));
        return;
    }

    // Share the sensor measurement with this vehicle's own position.
    acoustics_interfaces::msg::BearingMsg shared;
    shared.receiver_id = vehicle_id_;
    shared.call_cycle = msg->call_cycle;
    shared.sim_time = msg->sim_time;
    shared.bearing_rad = msg->bearing_rad;
    shared.bearing_std_rad = msg->bearing_std_rad;
    shared.receiver_north = position->north;
    shared.receiver_east = position->east;

    // Feed our own measurement directly into this vehicle's estimator.
    handle_shared_bearing(std::make_shared<acoustics_interfaces::msg::BearingMsg>(shared));

    // Share the measurement with the other vehicle.
    bearing_pub_->publish(shared);
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
    if (!odyssey_initialized_) return;
    if (!sim_time_received_) return;

    // Initialize the native Odyssey simulation clock once.
    if (!sim_time_initialized_) {
        if (!update_odyssey_sim_time(sim_time_)) throw std::runtime_error("Failed to initialize " + vehicle_id_ + " simulation time.");
        sim_time_initialized_ = true;
        RCLCPP_INFO(get_logger(), "Initialized '%s' simulation time.", vehicle_id_.c_str());
    }

    if (!calendar_start_time_received_) return;

    // Start the native Odyssey simulator once all startup data is available.
    if (!odyssey_sim_started_) {
        if (!start_odyssey_sim(calendar_start_time_)) throw std::runtime_error("Failed to start " + vehicle_id_ + " simulator.");
        odyssey_sim_started_ = true;
        RCLCPP_INFO(get_logger(), "Started '%s' simulator.", vehicle_id_.c_str());
    }

    // Notify the orchestrator once the vehicle is fully initialized.
    if (!ready_published_) {
        std_msgs::msg::Bool ready_message;        
        ready_message.data = true;
        ready_pub_->publish(ready_message);
        ready_published_ = true;
        RCLCPP_INFO(get_logger(), "Vehicle '%s' initialized and ready.", vehicle_id_.c_str()); 
    }
}