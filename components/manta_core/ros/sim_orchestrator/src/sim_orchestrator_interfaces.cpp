#include "sim_orchestrator/sim_orchestrator_node.hpp"

/**
 * Create service clients used by the simulation orchestrator.
 */
void SimOrchestratorNode::create_clients() {
    next_whale_call_client_ = create_client<whale_interfaces::srv::GetNextWhaleCall>("/sim/whales/get_next_call");  // Request the first scheduled whale call after a given simulation time.
}

/**
 * Create publishers used to coordinate the simulation and whale-call events.
 *
 * Simulation coordination topics retain their latest message so nodes that subscribe during startup can receive the current simulation state. Whale
 * calls use a deeper queue because multiple event messages may be processed.
 *
 * @param qos Reliable, retained QoS used by simulation coordination topics.
 */
void SimOrchestratorNode::create_publishers(const rclcpp::QoS &qos) {
    calendar_start_time_pub_ = create_publisher<std_msgs::msg::Int64>("/sim/calendar_start_time", qos); // Publish the calendar timestamp used to initialize the vehicle simulators.
    shutdown_pub_ = create_publisher<std_msgs::msg::Bool>("/sim/shutdown", qos);                        // Request an orderly shutdown of every simulation node.
    sim_step_pub_ = create_publisher<manta_interfaces::msg::SimStep>("/sim/step", qos);                 // Publish batches of simulation cycles for each node to process.
    sim_time_pub_ = create_publisher<std_msgs::msg::Float64>("/sim/time", qos);                         // Publish the initial shared simulation time.
    startup_ready_pub_ = create_publisher<std_msgs::msg::Bool>("/sim/startup_ready", qos);              // Notify simulation nodes that coordinated startup is complete.
    whale_call_pub_ = create_publisher<whale_interfaces::msg::WhaleCall>("/sim/whales/calls", 100);     // Publish individual whale-call events for the detection node to evaluate.
}

/**
 * Create the reliable, retained QoS used by simulation coordination topics.
 *
 * A queue depth of one retains only the latest message. Transient-local durability makes that message available to late subscribers, and reliable
 * delivery prevents it from being silently dropped.
 */
rclcpp::QoS SimOrchestratorNode::create_qos() const {
    rclcpp::QoS qos(1);
    qos.transient_local();
    qos.reliable();
    return qos;
}

/**
 * Create all ROS publishers and subscribers used by the simulation orchestrator node.
 */
void SimOrchestratorNode::create_ros_interfaces() {
    const rclcpp::QoS qos = create_qos();
    create_publishers(qos);
    create_subscribers(qos);
    create_clients();
}

/**
 * Create subscriptions for vehicle readiness, step results, and whale-call
 * processing acknowledgements.
 *
 * Each configured vehicle has its own readiness and step-result topics.
 * Whale-call processing acknowledgements are shared across the simulation.
 *
 * @param qos Reliable, retained QoS used by simulation coordination topics.
 */
void SimOrchestratorNode::create_subscribers(const rclcpp::QoS &qos) {
    // Create the readiness and step-result subscriptions for every vehicle.
    for (const std::string &vehicle_id : state_.vehicle_ids) {
        const std::string ready_topic = "/" + vehicle_id + "/ready";
        
        // Receive notification that this vehicle has completed initialization.
        auto subscription = create_subscription<std_msgs::msg::Bool>(
            ready_topic, qos, [this, vehicle_id](const std_msgs::msg::Bool::SharedPtr msg) {
                state_.vehicle_states.at(vehicle_id).ready_b = msg->data;   // Store the vehicle's latest readiness state.
                check_all_vehicles_initialized();                   // Begin the simulation if the entire fleet is ready.
                log_state_if_changed();
            }
        );
        ready_subs_.push_back(subscription);

        if (state_.wait_for_whale_estimators_b) {
            const std::string estimator_ready_topic = "/" + vehicle_id + "/whale_estimator/ready";
            auto estimator_subscription = create_subscription<std_msgs::msg::Bool>(
                estimator_ready_topic, qos,
                [this, vehicle_id](const std_msgs::msg::Bool::SharedPtr msg) {
                    if (msg->data) state_.ready_whale_estimators.insert(vehicle_id);
                    else state_.ready_whale_estimators.erase(vehicle_id);
                    check_all_vehicles_initialized();
                    log_state_if_changed();
                }
            );
            whale_estimator_ready_subs_.push_back(estimator_subscription);
        }
    }

    for (const std::string &vehicle_id : state_.vehicle_ids) {
        const std::string result_topic = "/" + vehicle_id + "/step_result";
        // Receive this vehicle's result for each simulation batch.
        auto subscription = create_subscription<manta_interfaces::msg::SimStepResult>(
            result_topic, qos, [this, vehicle_id](const manta_interfaces::msg::SimStepResult::SharedPtr msg) {
                state_.received_step_result = *msg;
                state_.expected_step_vehicle_id = vehicle_id;
                handle_step_result();
                log_state_if_changed();
            }
        );
        step_result_subs_.push_back(subscription);
    }

    // Receive confirmation that the current whale call has been evaluated.
    whale_call_processed_sub_ = create_subscription<acoustics_interfaces::msg::WhaleCallProcessed>(
        "/sim/whales/call_processed", 100, [this](const acoustics_interfaces::msg::WhaleCallProcessed::SharedPtr msg) {
            state_.received_whale_call_processed = *msg;
            handle_whale_call_processed();
            log_state_if_changed();
        }
    );

    // Receive completion acknowledgments from each vehicle's bearing estimator.
    bearing_processed_sub_ = create_subscription<acoustics_interfaces::msg::BearingProcessed>(
        "/sim/bearing_processed", rclcpp::QoS(100).reliable(), [this](const acoustics_interfaces::msg::BearingProcessed::SharedPtr msg) {
            state_.received_bearing_processed = *msg;
            handle_bearing_processed();
            log_state_if_changed();
        }
    );
}
