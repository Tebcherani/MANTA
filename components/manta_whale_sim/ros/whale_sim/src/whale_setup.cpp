#include "whale_sim/whale_node.hpp"

#include <cmath>
#include <cstdint>
#include <filesystem>
#include <stdexcept>

/**
 * Assign each population-level call to one whale active at that call time.
 */
void WhaleNode::assign_population_call_times() {
    for (Whale &whale : whales_) {
        whale.call_times.clear();
        whale.next_call_index = 0;
    }

    if (whales_.empty()) return;

    double first_start_time = mission_duration_;
    double final_end_time = 0.0;

    for (const Whale &whale : whales_) {
        if (whale.trajectory.empty()) continue;

        first_start_time = std::min(first_start_time, whale.start_time);
        final_end_time = std::max(final_end_time, whale.trajectory.back().time);
    }

    const std::vector<double> population_calls = generate_population_call_times(first_start_time, final_end_time);

    for (double call_time : population_calls) {
        std::vector<std::size_t> active_whales;

        for (std::size_t whale_index = 0; whale_index < whales_.size(); ++whale_index) {
            const Whale &whale = whales_[whale_index];

            if (whale.trajectory.empty()) continue;
            if (call_time < whale.start_time) continue;
            if (call_time > whale.trajectory.back().time) continue;

            active_whales.push_back(whale_index);
        }

        if (active_whales.empty()) continue;

        std::uniform_int_distribution<std::size_t> select_whale(0, active_whales.size() - 1);
        whales_[active_whales[select_whale(generator_)]].call_times.push_back(call_time);
    }
}

/**
 * Create a whale with its fixed properties and inactive initial state.
 */
WhaleNode::Whale WhaleNode::create_whale(std::size_t whale_index, std::size_t group_index, double start_time) const {
    Whale whale;

    whale.id = "whale_" + std::to_string(whale_index + 1);
    whale.group_index = group_index;
    whale.start_time = start_time;
    whale.active = false;
    whale.complete = false;

    whale.call_times.clear();
    whale.next_call_index = 0;

    return whale;
}

/**
 * Generate the aggregate calling schedule observed across the simulated whale population.
 *
 * The population alternates between exponentially distributed active and silent
 * periods. Calls within active bouts follow a homogeneous Poisson process.
 */
std::vector<double> WhaleNode::generate_population_call_times(double start_time, double end_time) {
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
std::vector<WhaleNode::TrajectoryPoint> WhaleNode::generate_trajectory(double initial_north, double initial_east, double initial_heading, double speed, double start_time, const WhaleGroup &group) {
    std::vector<TrajectoryPoint> trajectory;

    // Calculate the individual AR(1) heading-error innovation scale.
    const double heading_std_radians = heading_std_degrees_ * std::acos(-1.0) / 180.0;
    const double heading_innovation_scale = heading_std_radians * std::sqrt(1.0 - heading_correlation_ * heading_correlation_);

    // Weight the shared and individual heading processes so their combined
    // variance equals the configured heading variance.
    const double group_weight = std::sqrt(shared_heading_fraction_);
    const double individual_weight = std::sqrt(1.0 - shared_heading_fraction_);

    std::normal_distribution<double> standard_normal(0.0, 1.0);

    double time = start_time;
    double north = initial_north;
    double east = initial_east;
    double individual_heading_error = 0.0;

    while (true) {
        // Combine the group and individual heading errors for this interval.
        const std::size_t group_heading_index = std::min(static_cast<std::size_t>(std::floor(time / turn_interval_)), group.heading_errors.size() - 1);
        const double combined_heading_error = group_weight * group.heading_errors[group_heading_index] + individual_weight * individual_heading_error;
        const double heading = initial_heading + combined_heading_error;

        // Convert the whale's speed and heading into north/east velocity.
        const double velocity_north = speed * std::cos(heading);
        const double velocity_east = speed * std::sin(heading);

        trajectory.push_back({time, north, east, velocity_north, velocity_east});

        if (time >= mission_duration_) break;

        // End the segment at the next shared movement boundary so all group
        // members respond to heading changes at the same simulation time.
        const double next_turn_time = (std::floor(time / turn_interval_) + 1.0) * turn_interval_;
        const double next_time = std::min(next_turn_time, mission_duration_);
        const double duration = next_time - time;

        north += velocity_north * duration;
        east += velocity_east * duration;
        time = next_time;

        // Preserve the final mission position without extrapolating beyond it.
        if (time >= mission_duration_) {
            trajectory.push_back({time, north, east, velocity_north, velocity_east});
            break;
        }

        // Advance the whale's individual correlated heading-error process.
        individual_heading_error = heading_correlation_ * individual_heading_error + heading_innovation_scale * standard_normal(generator_);
    }

    return trajectory;
}

/**
 * Return the whale's round-robin group assignment to keep group sizes balanced.
 */
const WhaleNode::WhaleGroup &WhaleNode::get_group_for_whale(std::size_t whale_index) const {
    if (whale_groups_.empty()) throw std::runtime_error("Cannot assign a whale because no whale groups exist.");
    const std::size_t group_index = whale_index % whale_groups_.size();
    return whale_groups_[group_index];
}

/**
 * Return the whale's round-robin group index.
 */
std::size_t WhaleNode::get_group_index_for_whale(std::size_t whale_index) const {
    if (whale_groups_.empty()) throw std::runtime_error("Cannot assign a whale because no whale groups exist.");
    return whale_index % whale_groups_.size();
}

/**
 * Initialize shared movement and calling behavior for every whale group.
 */
void WhaleNode::initialize_whale_groups() {
    // Choose enough groups to approximately satisfy the configured mean size.
    const int group_count = std::max(1, static_cast<int>(std::ceil(static_cast<double>(whale_count_) / group_mean_size_)));

    // Calculate the number of shared heading intervals in the mission.
    const std::size_t heading_count = static_cast<std::size_t>(std::ceil(mission_duration_ / turn_interval_)) + 1;

    // Convert heading variation to radians and calculate the AR(1) innovation scale.
    const double heading_std_radians = heading_std_degrees_ * std::acos(-1.0) / 180.0;
    const double heading_innovation_scale = heading_std_radians * std::sqrt(1.0 - heading_correlation_ * heading_correlation_);

    // Create distributions used to generate group-level properties.
    std::normal_distribution<double> standard_normal(0.0, 1.0);

    whale_groups_.clear();
    whale_groups_.reserve(group_count);

    for (int index = 0; index < group_count; ++index) {
        WhaleGroup group;

        // Generate the group's correlated heading-error process.
        group.heading_errors.reserve(heading_count);

        double heading_error = 0.0;
        group.heading_errors.push_back(heading_error);

        for (std::size_t heading_index = 1; heading_index < heading_count; ++heading_index) {
            heading_error = heading_correlation_ * heading_error + heading_innovation_scale * standard_normal(generator_);
            group.heading_errors.push_back(heading_error);
        }

        whale_groups_.push_back(std::move(group));
    }

    RCLCPP_INFO(get_logger(), "Initialized %zu whale groups for %d whales.", whale_groups_.size(), whale_count_);
}

/**
 * Generate the whale's movement and calls and set its initial published state.
 */
void WhaleNode::initialize_whale_dynamics(Whale &whale, const WhaleInitialState &initial_state, double speed, const WhaleGroup &group) {
    whale.trajectory = generate_trajectory(initial_state.north, initial_state.east, initial_state.heading, speed, whale.start_time, group);
    
    if (whale.trajectory.empty()) throw std::runtime_error("Generated whale trajectory is empty.");
    
    whale.call_times.clear();
    whale.next_call_index = 0;

    const TrajectoryPoint &first_point = whale.trajectory.front();
    whale.north = first_point.north;
    whale.east = first_point.east;
    whale.velocity_north = first_point.velocity_north;
    whale.velocity_east = first_point.velocity_east;
}

/**
 * Initialize every whale's position, movement trajectory, and call schedule.
 */
void WhaleNode::initialize_whales() {
    initialize_whale_groups();

    whales_.clear();
    whales_.reserve(whale_count_);

    for (int index = 0; index < whale_count_; ++index) {
        const std::size_t group_index = get_group_index_for_whale(index);
        const WhaleGroup &group = whale_groups_[group_index];
        const double whale_start_time = 0.0;
        const double whale_speed = sample_speed();

        const double heading_radians = initial_heading_degrees_ * std::acos(-1.0) / 180.0;
        const WhaleInitialState initial_state{initial_north_, initial_east_, heading_radians};

        Whale whale = create_whale(index, group_index, whale_start_time);
        initialize_whale_dynamics(whale, initial_state, whale_speed, group);
        whales_.push_back(std::move(whale));
    }

    assign_population_call_times();
}

/**
 * Load whale scenario parameters supplied by the generated Compose file.
 */
void WhaleNode::load_config() {
    // Scenario size and picket-line geometry.
    whale_count_ = declare_parameter<int>("count", 1);
    mission_duration_ = declare_parameter<double>("mission_duration", 604800.0);

    // Whale config
    initial_north_ = declare_parameter<double>("initial_north", 0.0);
    initial_east_ = declare_parameter<double>("initial_east", 0.0);
    initial_heading_degrees_ = declare_parameter<double>("initial_heading_degrees", 0.0);

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

    // Whale grouping behavior.
    group_mean_size_ = declare_parameter<double>("group_mean_size", 3.0);
    shared_heading_fraction_ = declare_parameter<double>("shared_heading_fraction", 0.75);

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
        whale_log_ << "cycle,sim_time,whale_id,group_id,north,east\n";
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
    if (whale_count_ < 1) throw std::runtime_error("count must be at least one.");
    if (mission_duration_ <= 0.0) throw std::runtime_error("mission_duration must be greater than zero.");
    if (whale_sample_period_ <= 0.0) throw std::runtime_error("sample_period must be greater than zero.");

    if (!std::isfinite(initial_north_) || !std::isfinite(initial_east_) || !std::isfinite(initial_heading_degrees_)) throw std::runtime_error("Fixed whale initialization values must be finite.");

    if (speed_mean_ <= 0.0) throw std::runtime_error("speed_mean must be greater than zero.");
    if (turn_interval_ <= 0.0) throw std::runtime_error("turn_interval must be greater than zero.");
    if (heading_correlation_ < 0.0 || heading_correlation_ >= 1.0) throw std::runtime_error("heading_correlation must be in [0, 1).");
    if (heading_std_degrees_ < 0.0) throw std::runtime_error("heading_std_degrees cannot be negative.");

    if (bout_mean_duration_ <= 0.0) throw std::runtime_error("Mean calling-bout duration must be positive.");
    if (bout_mean_silent_duration_ <= 0.0) throw std::runtime_error("Mean silent duration between calling bouts must be positive.");
    if (bout_active_call_rate_per_hour_ <= 0.0) throw std::runtime_error("Active-bout call rate must be positive.");

    if (group_mean_size_ < 1.0) throw std::runtime_error("group_mean_size must be at least one.");
    if (shared_heading_fraction_ < 0.0 || shared_heading_fraction_ > 1.0) throw std::runtime_error("shared_heading_fraction must be in [0, 1].");
}