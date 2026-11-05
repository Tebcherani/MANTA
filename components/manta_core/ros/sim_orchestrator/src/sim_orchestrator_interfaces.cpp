#include "sim_orchestrator/sim_orchestrator_node.hpp"

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
    whale_step_pub_ = create_publisher<manta_interfaces::msg::SimStep>("/sim/whales/step", rclcpp::QoS(1).reliable());
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
}

/**
 * Create subscriptions for vehicle readiness and step results.
 *
 * Each configured vehicle has its own readiness and step-result topics.
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

    whale_step_result_sub_ = create_subscription<whale_interfaces::msg::WhaleStepResult>(
        "/sim/whales/step_result", rclcpp::QoS(1).reliable(),
        [this](const whale_interfaces::msg::WhaleStepResult::SharedPtr msg) {
            state_.received_whale_step_result = *msg;
            handle_whale_step_result();
            log_state_if_changed();
        });

    whale_ready_sub_ = create_subscription<std_msgs::msg::Bool>(
        "/sim/whales/ready", qos, [this](const std_msgs::msg::Bool::SharedPtr msg) {
            state_.whale_ready_b = msg->data;
            check_all_vehicles_initialized();
            log_state_if_changed();
        });

    detection_ready_sub_ = create_subscription<std_msgs::msg::Bool>(
        "/sim/whales/detector_ready", qos, [this](const std_msgs::msg::Bool::SharedPtr msg) {
            state_.detection_ready_b = msg->data;
            check_all_vehicles_initialized();
            log_state_if_changed();
        });
}
