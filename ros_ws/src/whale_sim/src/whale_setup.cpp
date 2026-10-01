#include "whale_sim/whale_node.hpp"

#include <cmath>
#include <cstdint>
#include <filesystem>
#include <stdexcept>

/**
 * Calculate the whale's starting position and heading toward the picket line.
 */
WhaleNode::WhaleInitialState WhaleNode::compute_initial_state(const WhaleGroup &group, const LineGeometry &line, double crossing_distance, double speed) const {
    const double crossing_north = line_start_north_ + line.along_north * crossing_distance;
    const double crossing_east = line_start_east_ + line.along_east * crossing_distance;
    const double initial_north = crossing_north + group.side * line.perpendicular_north * start_distance_;
    const double initial_east = crossing_east + group.side * line.perpendicular_east * start_distance_;
    const double velocity_north = -group.side * line.perpendicular_north * speed;
    const double velocity_east = -group.side * line.perpendicular_east * speed;

    return {initial_north, initial_east, std::atan2(velocity_east, velocity_north)};
}

/**
 * Compute the picket line's length and parallel/perpendicular unit vectors.
 */
WhaleNode::LineGeometry WhaleNode::compute_line_geometry() const {
    // Express the picket line as a north-east displacement vector.    
    const double line_north = line_end_north_ - line_start_north_;
    const double line_east = line_end_east_ - line_start_east_;
    const double line_length = std::hypot(line_north, line_east);

    // A zero-length line cannot define parallel or perpendicular directions.
    if (line_length <= 0.0) throw std::runtime_error("Picket-line start and end points must be different.");

    // Normalize the displacement to obtain the direction along the line.
    const double along_north = line_north / line_length;
    const double along_east = line_east / line_length;

    // Rotate the along-line unit vector 90 degrees counterclockwise to obtain the perpendicular unit vector in north-east coordinates.
    return {line_length, along_north, along_east, -along_east, along_north};
}

/**
 * Create a whale with its fixed properties and inactive initial state.
 */
WhaleNode::Whale WhaleNode::create_whale(std::size_t whale_index, std::size_t group_index, const WhaleGroup &group, double start_time, double call_rate) const {
    Whale whale;

    whale.id = "whale_" + std::to_string(whale_index + 1);
    whale.group_index = group_index;
    whale.depth = whale_depth_;
    whale.start_time = start_time;
    whale.call_rate_per_hour = call_rate;
    whale.active = false;
    whale.complete = false;
    whale.starting_side = group.side;

    return whale;
}

/**
 * Generate one whale's scheduled call times while it remains active.
 */
std::vector<double> WhaleNode::generate_call_times(double start_time, double end_time, double call_rate_per_hour, const WhaleGroup &group) {
    if (group.call_rate_multipliers.empty()) throw std::runtime_error("Whale group contains no call-rate multipliers.");
    if (end_time < start_time) throw std::runtime_error("Whale call interval ends before its start time.");

    std::vector<double> call_times;
    double interval_start = start_time;

    while (interval_start < end_time) {
        // Select the group's call-activity multiplier for this interval.
        const std::size_t activity_index = std::min(static_cast<std::size_t>(std::floor(interval_start / call_activity_interval_)), group.call_rate_multipliers.size() - 1);

        // Limit this interval to the next activity boundary or the end of the whale's active trajectory.
        const double next_activity_boundary = (std::floor(interval_start / call_activity_interval_) + 1.0) * call_activity_interval_;
        const double interval_end = std::min(next_activity_boundary, end_time);
        const double duration = interval_end - interval_start;

        // Convert the whale's hourly rate to the active rate for this interval.
        const double calls_per_second = call_rate_per_hour * group.call_rate_multipliers[activity_index] / 3600.0;

        // Draw the number of calls from a piecewise-constant Poisson process.
        std::poisson_distribution<int> call_count(calls_per_second * duration);
        const int count = call_count(generator_);

        if (count > 0) {
            // Distribute the generated calls uniformly throughout the interval.
            std::uniform_real_distribution<double> call_time(interval_start, interval_end);

            for (int index = 0; index < count; ++index) {
                call_times.push_back(call_time(generator_));
            }
        }

        interval_start = interval_end;
    }

    // Calls sampled within an interval are not necessarily generated in order.
    std::sort(call_times.begin(), call_times.end());

    return call_times;
}

/**
 * Generate one whale's complete movement trajectory through the monitored corridor.
 */
std::vector<WhaleNode::TrajectoryPoint> WhaleNode::generate_trajectory(double initial_north, double initial_east, double crossing_heading, double speed, double start_time, const WhaleGroup &group) {
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
        const double heading = crossing_heading + combined_heading_error;

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
 *
 * @param line_length Total horizontal length of the picket line.
 */
void WhaleNode::initialize_whale_groups(double line_length) {
    // Choose enough groups to approximately satisfy the configured mean size.
    const int group_count = std::max(1, static_cast<int>(std::ceil(static_cast<double>(whale_count_) / group_mean_size_)));

    // Calculate the number of movement and calling intervals in the mission.
    const std::size_t heading_count = static_cast<std::size_t>(std::ceil(mission_duration_ / turn_interval_)) + 1;
    const std::size_t call_activity_count = static_cast<std::size_t>(std::ceil(mission_duration_ / call_activity_interval_)) + 1;

    // Convert heading variation to radians and calculate the AR(1) innovation scale.
    const double heading_std_radians = heading_std_degrees_ * std::acos(-1.0) / 180.0;
    const double heading_innovation_scale = heading_std_radians * std::sqrt(1.0 - heading_correlation_ * heading_correlation_);
    const double call_innovation_scale = std::sqrt(1.0 - call_activity_correlation_ * call_activity_correlation_);

    // Create distributions used to generate group-level properties.
    std::uniform_real_distribution<double> crossing_distance(0.0, line_length);
    std::uniform_real_distribution<double> start_time(0.0, mission_duration_);
    std::bernoulli_distribution starting_side(0.5);
    std::normal_distribution<double> standard_normal(0.0, 1.0);

    whale_groups_.clear();
    whale_groups_.reserve(group_count);

    for (int index = 0; index < group_count; ++index) {
        WhaleGroup group;

        // Generate the group's crossing location, starting side, and start time.
        group.crossing_distance = crossing_distance(generator_);
        group.side = starting_side(generator_) ? 1.0 : -1.0;
        group.start_time = start_time(generator_);

        // Generate the group's correlated heading-error process. Each group
        // initially travels directly toward its crossing location.
        group.heading_errors.reserve(heading_count);

        double heading_error = 0.0;
        group.heading_errors.push_back(heading_error);

        for (std::size_t heading_index = 1; heading_index < heading_count; ++heading_index) {
            heading_error = heading_correlation_ * heading_error + heading_innovation_scale * standard_normal(generator_);
            group.heading_errors.push_back(heading_error);
        }

        // Generate the group's correlated calling-activity process.
        group.call_rate_multipliers.reserve(call_activity_count);

        // Starting in the stationary distribution avoids making the beginning
        // of the simulation artificially quieter than the remainder.
        double group_call_activity = standard_normal(generator_);

        for (std::size_t activity_index = 0; activity_index < call_activity_count; ++activity_index) {
            if (activity_index > 0) {
                group_call_activity = call_activity_correlation_ * group_call_activity + call_innovation_scale * standard_normal(generator_);
            }

            // A lognormal multiplier keeps call rates positive. Subtracting
            // sigma squared over two keeps the expected multiplier equal to one.
            const double multiplier = std::exp(group_log_rate_std_ * group_call_activity - 0.5 * group_log_rate_std_ * group_log_rate_std_);
            group.call_rate_multipliers.push_back(multiplier);
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

    const double end_time = whale.trajectory.back().time;
    whale.call_times = generate_call_times(whale.start_time, end_time, whale.call_rate_per_hour, group);
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
    const LineGeometry line = compute_line_geometry();  // Calculate picket-line geometry and initialize shared whale groups.
    initialize_whale_groups(line.length);

    std::uniform_real_distribution<double> call_rate(call_rate_min_, call_rate_max_);

    whales_.clear();
    whales_.reserve(whale_count_);

    // Loop through all whales
    for (int index = 0; index < whale_count_; ++index) {
        const std::size_t group_index = get_group_index_for_whale(index);
        const WhaleGroup &group = whale_groups_[group_index];

        const double crossing_distance = sample_crossing_distance(group, line.length);
        const double whale_start_time = sample_start_time(group);
        const double whale_speed = sample_speed();

        const double whale_call_rate = call_rate(generator_);
        const WhaleInitialState initial_state = compute_initial_state(group, line, crossing_distance, whale_speed);
        
        Whale whale = create_whale(index, group_index, group, whale_start_time, whale_call_rate);

        initialize_whale_dynamics(whale, initial_state, whale_speed, group);
        whales_.push_back(std::move(whale));
    }
}

/**
 * Load whale scenario parameters supplied by the generated Compose file.
 */
void WhaleNode::load_config() {
    // Scenario size and picket-line geometry.
    whale_count_ = declare_parameter<int>("count", 46);
    line_start_north_ = declare_parameter<double>("line_start_north", 0.0);
    line_start_east_ = declare_parameter<double>("line_start_east", -63000.0);
    line_end_north_ = declare_parameter<double>("line_end_north", 0.0);
    line_end_east_ = declare_parameter<double>("line_end_east", 63000.0);
    start_distance_ = declare_parameter<double>("start_distance", 2000.0);
    whale_depth_ = declare_parameter<double>("depth", 20.0);
    mission_duration_ = declare_parameter<double>("mission_duration", 604800.0);

    // Whale movement.
    speed_mean_ = declare_parameter<double>("speed_mean", 1.5);
    speed_std_ = declare_parameter<double>("speed_std", 0.2);
    turn_interval_ = declare_parameter<double>("turn_interval", 60.0);              // Update whale heading every turn_interval s
    heading_correlation_ = declare_parameter<double>("heading_correlation", 0.95);  // Controls how strongly the whale’s next heading resembles its current heading
    heading_std_degrees_ = declare_parameter<double>("heading_std_degrees", 15.0);

    // Whale calling behavior.
    call_rate_min_ = declare_parameter<double>("call_rate_min", 4.8);
    call_rate_max_ = declare_parameter<double>("call_rate_max", 8.0);
    call_activity_interval_ = declare_parameter<double>("call_activity_interval", 600.0);
    call_activity_correlation_ = declare_parameter<double>("call_activity_correlation", 0.90);
    group_log_rate_std_ = declare_parameter<double>("group_log_rate_std", 0.50);

    // Whale grouping behavior.
    group_mean_size_ = declare_parameter<double>("group_mean_size", 3.0);
    group_position_std_ = declare_parameter<double>("group_position_std", 1000.0);
    group_start_time_std_ = declare_parameter<double>("group_start_time_std", 600.0);
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
        whale_log_ << "cycle,sim_time,whale_id,group_id,north,east,depth\n";
}

/**
 * Draw a crossing location near the group center and within the picket line.
 */
double WhaleNode::sample_crossing_distance(const WhaleGroup &group, double line_length) {
    std::normal_distribution<double> offset(0.0, group_position_std_);
    double distance;

    do {
        distance = group.crossing_distance + offset(generator_);
    } while (distance < 0.0 || distance > line_length);

    return distance;
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
 * Draw an activation time near the group time and within the mission.
 */
double WhaleNode::sample_start_time(const WhaleGroup &group) {
    std::normal_distribution<double> offset(0.0, group_start_time_std_);
    double time;

    do {
        time = group.start_time + offset(generator_);
    } while (time < 0.0 || time >= mission_duration_);

    return time;
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
    const double line_length = std::hypot(
        line_end_north_ - line_start_north_,
        line_end_east_ - line_start_east_
    );

    if (whale_count_ < 1) throw std::runtime_error("count must be at least one.");
    if (line_length <= 0.0) throw std::runtime_error("Picket line must have nonzero length.");
    if (start_distance_ <= 0.0) throw std::runtime_error("start_distance must be greater than zero.");
    if (mission_duration_ <= 0.0) throw std::runtime_error("mission_duration must be greater than zero.");
    if (whale_sample_period_ <= 0.0) throw std::runtime_error("sample_period must be greater than zero.");

    if (speed_mean_ <= 0.0) throw std::runtime_error("speed_mean must be greater than zero.");
    if (turn_interval_ <= 0.0) throw std::runtime_error("turn_interval must be greater than zero.");
    if (heading_correlation_ < 0.0 || heading_correlation_ >= 1.0) throw std::runtime_error("heading_correlation must be in [0, 1).");
    if (heading_std_degrees_ < 0.0) throw std::runtime_error("heading_std_degrees cannot be negative.");

    if (call_rate_min_ <= 0.0) throw std::runtime_error("call_rate_min must be greater than zero.");
    if (call_rate_max_ < call_rate_min_) throw std::runtime_error("call_rate_max must be greater than or equal to call_rate_min.");
    if (call_activity_interval_ <= 0.0) throw std::runtime_error("call_activity_interval must be greater than zero.");
    if (call_activity_correlation_ < 0.0 || call_activity_correlation_ >= 1.0) throw std::runtime_error("call_activity_correlation must be in [0, 1).");
    if (group_log_rate_std_ < 0.0) throw std::runtime_error("group_log_rate_std cannot be negative.");

    if (group_mean_size_ < 1.0) throw std::runtime_error("group_mean_size must be at least one.");
    if (group_position_std_ < 0.0) throw std::runtime_error("group_position_std cannot be negative.");
    if (group_start_time_std_ < 0.0) throw std::runtime_error("group_start_time_std cannot be negative.");
    if (shared_heading_fraction_ < 0.0 || shared_heading_fraction_ > 1.0) throw std::runtime_error("shared_heading_fraction must be in [0, 1].");
}