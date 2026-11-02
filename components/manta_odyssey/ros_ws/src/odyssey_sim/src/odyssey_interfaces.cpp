#include "odyssey_sim/odyssey_node.hpp"

/**
 * Create publishers used to report vehicle readiness, step completion, and vehicle state.
 *
 * @param qos Reliable, retained QoS used for readiness, step results, and vehicle states.
 */
void OdysseyNode::create_publishers(const rclcpp::QoS &qos) {
    bearing_processed_pub_ = create_publisher<acoustics_interfaces::msg::BearingProcessed>("/sim/bearing_processed", rclcpp::QoS(100).reliable());
    bearing_pub_ = create_publisher<acoustics_interfaces::msg::BearingTransmission>("/comms/bearings", rclcpp::QoS(100).reliable());
    ready_pub_ = create_publisher<std_msgs::msg::Bool>(odyssey_state_.vehicle_id + "/ready", qos);
    step_result_pub_ = create_publisher<manta_interfaces::msg::SimStepResult>("/" + odyssey_state_.vehicle_id + "/step_result", qos);
    vehicle_state_pub_ = create_publisher<manta_interfaces::msg::VehicleState>("/" + odyssey_state_.vehicle_id + "/state", qos);
    whale_fix_pub_ = create_publisher<odyssey_interfaces::msg::WhaleBearingPositionEstimate>("/" + odyssey_state_.vehicle_id + "/whale/bearing_position_estimate", rclcpp::QoS(100).reliable().transient_local());
}

/**
 * Create the reliable, retained QoS used by simulation topics.
 *
 * A queue depth of one retains only the latest message. Transient-local durability allows late subscribers to receive that message, while reliable
 * delivery prevents it from being silently dropped.
 */
rclcpp::QoS OdysseyNode::create_qos() const {
    rclcpp::QoS qos(1);
    qos.transient_local();
    qos.reliable();

    return qos;
}

/**
 * Create reliable, non-retained QoS for live whale data.
 *
 * Whale detections and states describe live simulation events and therefore
 * do not need transient-local durability.
 */
rclcpp::QoS OdysseyNode::create_whale_qos() const {
    rclcpp::QoS qos(100);
    qos.reliable();
    qos.durability_volatile();

    return qos;
}

/**
 * Create all ROS publishers and subscribers used by the Odyssey node.
 */
void OdysseyNode::create_ros_interfaces() {
    const rclcpp::QoS qos = create_qos();
    const rclcpp::QoS whale_qos = create_whale_qos();
    create_publishers(qos);
    create_subscribers(qos, whale_qos);
}

/**
 * Create subscriptions used to receive simulation startup, step, time, and shutdown messages.
 *
 * Startup and step subscriptions store received values before processing; other subscriptions forward messages to their handlers.
 *
 * @param qos Reliable, retained QoS used for simulation startup and step messages.
 * @param whale_qos Reliable, non-retained QoS used for live bearing and estimate messages.
 */
void OdysseyNode::create_subscribers(const rclcpp::QoS &qos, const rclcpp::QoS &whale_qos) {
    calendar_start_time_sub_ = create_subscription<std_msgs::msg::Int64>( "/sim/calendar_start_time", qos, [this](const std_msgs::msg::Int64::SharedPtr msg) {
        odyssey_state_.calendar_start_time = static_cast<time_t>(msg->data);  // Convert the timestamp carried by the ROS message to the native time type.
        odyssey_state_.calendar_start_time_received_b = true;                   // Record that this required startup value has been received.
        try_finish_init();                                      // Complete startup if all other required initialization data is available.
    });
    shared_bearing_sub_ = create_subscription<acoustics_interfaces::msg::BearingTransmission>("/comms/bearings", whale_qos, [this](const acoustics_interfaces::msg::BearingTransmission::SharedPtr msg) {
        if (msg->receiver_id == odyssey_state_.vehicle_id) return;
        handle_shared_bearing(msg);
    });
    startup_sub_ = create_subscription<std_msgs::msg::Bool>("/sim/startup_ready", qos, [this](const std_msgs::msg::Bool::SharedPtr msg) {
        odyssey_state_.startup_ready_b = msg->data;
        handle_startup();
    });
    sim_sim_step_sub_ = create_subscription<manta_interfaces::msg::SimStep>("/sim/step", qos, [this](const manta_interfaces::msg::SimStep::SharedPtr msg) {
        odyssey_state_.pending_sim_step = *msg;
        handle_sim_step();
    });
    sim_time_sub_ = create_subscription<std_msgs::msg::Float64>("/sim/time", qos, [this](const std_msgs::msg::Float64::SharedPtr msg) {
        odyssey_state_.sim_time_s = msg->data;       // Store the initial simulated mission time.
        odyssey_state_.sim_time_received_b = true;   // Record that this required startup value has arrived.
        try_finish_init();           // Complete startup if the other required values are available.
    });
    shutdown_sub_ = create_subscription<std_msgs::msg::Bool>("/sim/shutdown", qos, [this](const std_msgs::msg::Bool::SharedPtr msg) {handle_shutdown(msg);});
    bearing_sub_ = create_subscription<acoustics_interfaces::msg::BearingObservation>("/" + odyssey_state_.vehicle_id + "/sensors/bearing", whale_qos, [this](const acoustics_interfaces::msg::BearingObservation::SharedPtr msg) {handle_bearing_observation(msg);});
    whale_estimate_sub_ = create_subscription<whale_estimator_interfaces::msg::WhaleKalmanEstimate>("/" + odyssey_state_.vehicle_id + "/whale/kalman_estimate", whale_qos, [this](const whale_estimator_interfaces::msg::WhaleKalmanEstimate::SharedPtr msg) {handle_whale_estimate(msg);});
}
