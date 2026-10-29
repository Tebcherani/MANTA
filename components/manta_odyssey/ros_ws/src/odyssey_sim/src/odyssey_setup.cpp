#include "odyssey_sim/odyssey_node.hpp"

#include <cmath>
#include <cstdlib>
#include <stdexcept>

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

    output_sample_period_s_ = declare_parameter<double>("output_sample_period_s", 2.0);
    if (output_sample_period_s_ <= 0.0) throw std::runtime_error("output_sample_period_s must be greater than zero.");

    tracking_enabled_b_ = declare_parameter<bool>("tracking_enabled", false);

    communication_interval_s_ = declare_parameter<double>("communication_interval_s", 0.0);
    if (!std::isfinite(communication_interval_s_) || communication_interval_s_ < 0.0) throw std::runtime_error("communication_interval_s must be finite and nonnegative.");
}
