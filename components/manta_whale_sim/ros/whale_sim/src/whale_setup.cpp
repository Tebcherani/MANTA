#include "whale_sim/whale_node.hpp"

#include <cmath>
#include <cstdint>
#include <filesystem>
#include <stdexcept>


/**
 * Generate the aggregate calling schedule observed across the simulated whale population.
 *
 * The population alternates between exponentially distributed active and silent
 * periods. Calls within active bouts follow a homogeneous Poisson process.
 */
std::vector<double> WhaleNode::generate_call_times(double start_time, double end_time) {
    std::vector<double> call_times;

    if (end_time <= start_time) return call_times;

    std::exponential_distribution<double> active_duration(1.0 / bout_mean_duration_);
    std::exponential_distribution<double> silent_duration(1.0 / bout_mean_silent_duration_);
    std::exponential_distribution<double> call_interval(bout_active_call_rate_per_hour_ / 3600.0);

    const double active_probability = bout_mean_duration_ / (bout_mean_duration_ + bout_mean_silent_duration_);
    std::bernoulli_distribution initial_state(active_probability);

    bool active = initial_state(generator_);
    double interval_start = start_time;

    while (interval_start < end_time) {
        double duration;

        do {
            duration = active ? active_duration(generator_) : silent_duration(generator_);
        } while (duration <= 0.0);

        const double interval_end = std::min(interval_start + duration, end_time);

        if (active) {
            double time_until_call;

            do {
                time_until_call = call_interval(generator_);
            } while (time_until_call <= 0.0);

            double call_time = interval_start + time_until_call;

            while (call_time < interval_end) {
                call_times.push_back(call_time);

                do {
                    time_until_call = call_interval(generator_);
                } while (time_until_call <= 0.0);

                call_time += time_until_call;
            }
        }

        interval_start = interval_end;
        active = !active;
    }

    return call_times;
}

/**
 * Generate one whale's movement trajectory from its activation time to the end of the mission.
 */
std::vector<WhaleNode::TrajectoryPoint> WhaleNode::generate_trajectory(double speed) {
    std::vector<TrajectoryPoint> trajectory;

    const double heading_std_radians = heading_std_degrees_ * std::acos(-1.0) / 180.0;
    const double innovation_scale = heading_std_radians * std::sqrt(1.0 - heading_correlation_ * heading_correlation_);

    std::normal_distribution<double> standard_normal(0.0, 1.0);

    double time = 0.0;
    double north = 0.0;
    double east = 0.0;
    double heading_error = 0.0;

    while (true) {
        const double velocity_north = speed * std::cos(heading_error);
        const double velocity_east = speed * std::sin(heading_error);

        trajectory.push_back({time, north, east, velocity_north, velocity_east});

        if (time >= mission_duration_) break;

        const double next_turn_time = (std::floor(time / turn_interval_) + 1.0) * turn_interval_;
        const double next_time = std::min(next_turn_time, mission_duration_);
        const double duration = next_time - time;

        north += velocity_north * duration;
        east += velocity_east * duration;
        time = next_time;

        if (time >= mission_duration_) {
            trajectory.push_back({time, north, east, velocity_north, velocity_east});
            break;
        }

        heading_error = heading_correlation_ * heading_error + innovation_scale * standard_normal(generator_);
    }

    return trajectory;
}

/**
 * Initialize every whale's position, movement trajectory, and call schedule.
 */
void WhaleNode::initialize_whales() {
    Whale whale{};

    whale.id = "whale_1";
    whale.active = true;
    whale.complete = false;
    whale.next_call_index = 0;

    const double speed = sample_speed();

    whale.trajectory = generate_trajectory(speed);

    if (whale.trajectory.empty()) throw std::runtime_error("Generated whale trajectory is empty.");

    whale.call_times = generate_call_times(0.0, mission_duration_);

    const TrajectoryPoint &first = whale.trajectory.front();
    whale.north = first.north;
    whale.east = first.east;
    whale.velocity_north = first.velocity_north;
    whale.velocity_east = first.velocity_east;

    whales_.clear();
    whales_.push_back(std::move(whale));
}

/**
 * Load whale scenario parameters supplied by the generated Compose file.
 */
void WhaleNode::load_config() {
    // Scenario size and picket-line geometry.
    mission_duration_ = declare_parameter<double>("mission_duration", 604800.0);

    // Whale movement.
    speed_mean_ = declare_parameter<double>("speed_mean", 1.5);
    speed_std_ = declare_parameter<double>("speed_std", 0.2);
    turn_interval_ = declare_parameter<double>("turn_interval", 60.0);              // Update whale heading every turn_interval s
    heading_correlation_ = declare_parameter<double>("heading_correlation", 0.95);  // Controls how strongly the whale’s next heading resembles its current heading
    heading_std_degrees_ = declare_parameter<double>("heading_std_degrees", 15.0);

    // Whale calling behavior.
    bout_mean_duration_ = declare_parameter<double>("bout_mean_duration", 1842.0);
    bout_mean_silent_duration_ = declare_parameter<double>("bout_mean_silent_duration", 7860.0);
    bout_active_call_rate_per_hour_ = declare_parameter<double>("bout_active_call_rate_per_hour", 40.1);

    // Output sampling.
    whale_sample_period_ = declare_parameter<double>("sample_period", 60.0);
}

/**
 * Open the CSV file used to record whale trajectories.
 *
 * @throws std::runtime_error If the trajectory file cannot be opened.
 */
void WhaleNode::open_whale_log() {
    const std::string data_directory = declare_parameter<std::string>("data_directory", "/odyssey/output/Data");
    std::filesystem::create_directories(data_directory);
        whale_log_.open(data_directory + "/whales.csv", std::ios::out | std::ios::trunc);
        if (!whale_log_.is_open()) throw std::runtime_error("Failed to open whale trajectory log.");
        whale_log_ << "cycle,sim_time,whale_id,north,east\n";
}

/** 
 * Draw a physically valid positive whale speed.
 */
double WhaleNode::sample_speed() {
    std::normal_distribution<double> distribution(speed_mean_, speed_std_);
    double speed;

    do {
        speed = distribution(generator_);
    } while (speed <= 0.0);

    return speed;
}

/**
 * Seed the random-number generator used for whale behavior.
 */
void WhaleNode::seed_generator() {
    const int64_t random_seed = declare_parameter<int64_t>("random_seed", 42);
    generator_.seed(static_cast<std::mt19937::result_type>(random_seed));
}

/**
 * Validate the loaded whale-simulation configuration.
 *
 * @throws std::runtime_error If any configured value is invalid.
 */
void WhaleNode::validate_config() const {
    if (mission_duration_ <= 0.0) throw std::runtime_error("mission_duration must be greater than zero.");
    if (whale_sample_period_ <= 0.0) throw std::runtime_error("sample_period must be greater than zero.");

    if (speed_mean_ <= 0.0) throw std::runtime_error("speed_mean must be greater than zero.");
    if (turn_interval_ <= 0.0) throw std::runtime_error("turn_interval must be greater than zero.");
    if (heading_correlation_ < 0.0 || heading_correlation_ >= 1.0) throw std::runtime_error("heading_correlation must be in [0, 1).");
    if (heading_std_degrees_ < 0.0) throw std::runtime_error("heading_std_degrees cannot be negative.");

    if (bout_mean_duration_ <= 0.0) throw std::runtime_error("Mean calling-bout duration must be positive.");
    if (bout_mean_silent_duration_ <= 0.0) throw std::runtime_error("Mean silent duration between calling bouts must be positive.");
    if (bout_active_call_rate_per_hour_ <= 0.0) throw std::runtime_error("Active-bout call rate must be positive.");
}