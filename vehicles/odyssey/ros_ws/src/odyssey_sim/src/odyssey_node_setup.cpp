#include "odyssey_sim/odyssey_node.hpp"

#include <cstdlib>
#include <stdexcept>

//-------------------------------------------------------------
// Configuration
//-------------------------------------------------------------

/**
 * Load and validate the vehicle-specific runtime configuration.
 *
 * The vehicle identifier and output directory are supplied through environment variables because each generated Docker service has different values. The
 * output sample period is supplied as a ROS parameter.
 */
void OdysseyNode::load_config() {
    const char *vehicle_id_env = std::getenv("ODYSSEY_VEHICLE_ID");
    if (vehicle_id_env == nullptr || vehicle_id_env[0] == '\0') throw std::runtime_error("ODYSSEY_VEHICLE_ID is not set.");
    vehicle_id_ = vehicle_id_env;

    const char *data_directory_env = std::getenv("ODYSSEY_DATA_DIR");
    if (data_directory_env == nullptr || data_directory_env[0] == '\0') throw std::runtime_error("ODYSSEY_DATA_DIR is not set.");
    data_directory_ = data_directory_env;

    output_sample_period_ = declare_parameter<double>("output_sample_period", 2.0);
    if (output_sample_period_ <= 0.0) throw std::runtime_error("output_sample_period must be greater than zero.");
}

//-------------------------------------------------------------
// ROS Interfaces
//-------------------------------------------------------------

/**
 * Create publishers used to report vehicle readiness, step completion, and vehicle state.
 *
 * @param startup_qos QoS used for retained readiness messages.
 * @param step_qos QoS used for simulation-step results and vehicle states.
 */
void OdysseyNode::create_publishers(const rclcpp::QoS &qos) {
    ready_pub_ = create_publisher<std_msgs::msg::Bool>(vehicle_id_ + "/ready", qos);
    step_result_pub_ = create_publisher<manta_interfaces::msg::SimStepResult>("/" + vehicle_id_ + "/step_result", qos);
    vehicle_state_pub_ = create_publisher<manta_interfaces::msg::VehicleState>("/" + vehicle_id_ + "/state", qos);
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
 * Create all ROS publishers and subscribers used by the Odyssey node.
 */
void OdysseyNode::create_ros_interfaces() {
    const rclcpp::QoS qos = create_qos();
    create_publishers(qos);
    create_subscribers(qos);
}

/**
 * Create subscriptions used to receive simulation startup, step, time, and shutdown messages.
 *
 * Each lambda only forwards the received message to its corresponding callback handler.
 *
 * @param startup_qos QoS used for retained simulation-state messages.
 * @param step_qos QoS used for simulation-step messages.
 */
void OdysseyNode::create_subscribers(const rclcpp::QoS &qos) {
    calendar_start_time_sub_ = create_subscription<std_msgs::msg::Int64>(
        "/sim/calendar_start_time", qos, [this](const std_msgs::msg::Int64::SharedPtr msg) {handle_calendar_start_time(msg);}
    );
 
    startup_sub_ = create_subscription<std_msgs::msg::Bool>(
        "/sim/startup_ready", qos, [this](const std_msgs::msg::Bool::SharedPtr msg) {handle_startup(msg);}
    );

    sim_step_sub_ = create_subscription<manta_interfaces::msg::SimStep>(
        "/sim/step", qos, [this](const manta_interfaces::msg::SimStep::SharedPtr msg) {handle_sim_step(msg);}
    );

    sim_time_sub_ = create_subscription<std_msgs::msg::Float64>(
        "/sim/time", qos, [this](const std_msgs::msg::Float64::SharedPtr msg) {handle_sim_time(msg);}
    );

    shutdown_sub_ = create_subscription<std_msgs::msg::Bool>(
        "/sim/shutdown", qos, [this](const std_msgs::msg::Bool::SharedPtr msg) {handle_shutdown(msg);}
    );
}