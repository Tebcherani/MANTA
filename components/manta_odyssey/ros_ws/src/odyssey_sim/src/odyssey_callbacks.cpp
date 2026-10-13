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

void OdysseyNode::handle_shared_bearing(const acoustics_interfaces::msg::BearingMsg::SharedPtr msg) {
    if (!tracking_enabled_) return;
    if (msg->receiver_id == vehicle_id_) return;
    if (msg->receiver_id != "auv_1" && msg->receiver_id != "auv_2") return;
    if (!latest_bearing_available_) return;

    const auto &local = latest_bearing_;
    const auto &remote = *msg;

    // Ignore retransmissions of a pair already processed.
    if (processed_bearing_pair_available_ && local.sim_time == processed_local_bearing_time_ && remote.sim_time == processed_remote_bearing_time_) return;

    if (!std::isfinite(local.bearing_rad) || !std::isfinite(remote.bearing_rad) || !std::isfinite(local.receiver_north) || !std::isfinite(local.receiver_east) || !std::isfinite(remote.receiver_north) || !std::isfinite(remote.receiver_east)) {
        RCLCPP_WARN(get_logger(), "Cannot estimate from non-finite bearings.");
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
    const double disp_n = remote.receiver_north - local.receiver_north;
    const double disp_e = remote.receiver_east - local.receiver_east;

    const double determinant = local_n * remote_e - local_e * remote_n;

    if (!std::isfinite(determinant) ||
        std::abs(determinant) < 1.0e-6) {
        RCLCPP_WARN(get_logger(), "Cannot triangulate nearly parallel bearing lines.");
        return;
    }

    const double local_range = (disp_n * remote_e - disp_e * remote_n) / determinant;
    const double remote_range = (disp_n * local_e - disp_e * local_n) / determinant;

    if (!std::isfinite(local_range) || !std::isfinite(remote_range) || local_range < 0.0 || remote_range < 0.0) {
        RCLCPP_WARN(get_logger(), "Bearing rays have no valid forward intersection.");
        return;
    }

    const double estimated_north = local.receiver_north + local_range * local_n;
    const double estimated_east = local.receiver_east + local_range * local_e;

    if (!std::isfinite(estimated_north) ||
        !std::isfinite(estimated_east)) {
        return;
    }

    // Record the pair only after triangulation succeeds.
    processed_local_bearing_time_ = local.sim_time;
    processed_remote_bearing_time_ = remote.sim_time;
    processed_bearing_pair_available_ = true;

    // Treat the intersection as an estimate at local reception time.
    // The native function also updates velocity from successive estimates.
    set_whale_trailing_target(estimated_north, estimated_east, sim_time_);

    RCLCPP_INFO(get_logger(), "Vehicle '%s' received a bearing from '%s': whale estimate north %.2f m, east %.2f m at %.2f s.",
        vehicle_id_.c_str(), msg->receiver_id.c_str(), estimated_north, estimated_east, sim_time_);
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

        sim_time_ = next_sim_time;
        publish_bearing(next_sim_time);
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

    latest_bearing_ = shared;
    latest_bearing_available_ = true;

    if (communication_interval_ > 0.0) {
        // Confirm local storage so the orchestrator can continue.
        acoustics_interfaces::msg::BearingProcessed stored;
        stored.vehicle_id = vehicle_id_;
        stored.call_cycle = msg->call_cycle;
        stored.sim_time = msg->sim_time;
        stored.estimate_valid = false;
        bearing_processed_pub_->publish(stored);

        return;
    }

    // Zero interval preserves immediate sharing and estimation.
    bearing_pub_->publish(shared);

    // Acknowledge reception even when only one AUV detected the call.
    acoustics_interfaces::msg::BearingProcessed received;
    received.vehicle_id = vehicle_id_;
    received.call_cycle = msg->call_cycle;
    received.sim_time = msg->sim_time;
    received.estimate_valid = false;
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

void OdysseyNode::publish_bearing(double simulation_time) {
    if (!tracking_enabled_ || communication_interval_ <= 0.0) return;

    while (simulation_time + 1.0e-9 >= static_cast<double>(next_communication_index_) * communication_interval_) {
        if (latest_bearing_available_) bearing_pub_->publish(latest_bearing_);
        ++next_communication_index_;
    }
}