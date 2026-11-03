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
    odyssey_state_.vehicle_id = vehicle_id_env;

    const char *data_directory_env = std::getenv("ODYSSEY_DATA_DIR");
    if (data_directory_env == nullptr || data_directory_env[0] == '\0') throw std::runtime_error("ODYSSEY_DATA_DIR is not set.");
    odyssey_state_.data_directory = data_directory_env;

    odyssey_state_.output_sample_period_s = declare_parameter<double>("output_sample_period_s", 2.0);
    if (odyssey_state_.output_sample_period_s <= 0.0) throw std::runtime_error("output_sample_period_s must be greater than zero.");

    odyssey_state_.tracking_enabled_b = declare_parameter<bool>("tracking_enabled", false);

    odyssey_state_.communication_interval_s = declare_parameter<double>("communication_interval_s", 0.0);
    if (!std::isfinite(odyssey_state_.communication_interval_s) || odyssey_state_.communication_interval_s < 0.0) throw std::runtime_error("communication_interval_s must be finite and nonnegative.");
}
