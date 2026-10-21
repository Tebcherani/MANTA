#include "whale_sim/whale_node.hpp"

#include <cmath>
#include <cstdint>
#include <filesystem>
#include <stdexcept>

/**
 * Assign each population-level call to one whale active at that call time.
 */
void WhaleNode::assign_population_whale_call_times_s() {
    for (Whale &whale : sim_state_.whales) {
        whale.whale_call_times_s.clear();
        whale.next_call_index = 0;
    }

    if (sim_state_.whales.empty()) return;

    sim_state_.call_schedule_window.start_time_s = sim_state_.mission_duration_s;
    sim_state_.call_schedule_window.end_time_s = 0.0;

    for (const Whale &whale : sim_state_.whales) {
        if (whale.trajectory.empty()) continue;

        sim_state_.call_schedule_window.start_time_s = std::min(sim_state_.call_schedule_window.start_time_s, whale.start_time_s);
        sim_state_.call_schedule_window.end_time_s = std::max(sim_state_.call_schedule_window.end_time_s, whale.trajectory.back().sim_time_s);
    }

    const std::vector<double> population_calls = generate_population_whale_call_times_s();

    for (double call_time : population_calls) {
        std::vector<std::size_t> active_whales;

        for (std::size_t whale_index = 0; whale_index < sim_state_.whales.size(); ++whale_index) {
            const Whale &whale = sim_state_.whales[whale_index];

            if (whale.trajectory.empty()) continue;
            if (call_time < whale.start_time_s) continue;
            if (call_time > whale.trajectory.back().sim_time_s) continue;

            active_whales.push_back(whale_index);
        }

        if (active_whales.empty()) continue;

        std::uniform_int_distribution<std::size_t> select_whale(0, active_whales.size() - 1);
        sim_state_.whales[active_whales[select_whale(sim_state_.rng)]].whale_call_times_s.push_back(call_time);
    }
}

/**
 * Calculate the whale's starting position and heading toward the picket line.
 */
WhaleInitialState WhaleNode::compute_initial_state() const {
    const WhaleGroup &group = sim_state_.whale_groups.at(sim_state_.whale_generation.group_index);
    const LineGeometry &line = sim_state_.line_geometry;
    const double whale_group_crossing_distance_m = sim_state_.whale_generation.group_crossing_distance_m;
    const double speed = sim_state_.whale_generation.speed_mps;
    const double crossing_north = line.picket_line_start_north_m + line.picket_line_unit_vector_north * whale_group_crossing_distance_m;
    const double crossing_east = line.picket_line_start_east_m + line.picket_line_unit_vector_east * whale_group_crossing_distance_m;
    const double start_distance = sim_state_.whale_initial_state.whale_start_distance_from_picket_line_m;
    const double initial_north = crossing_north + group.whale_group_starting_side * line.perpendicular_to_picket_line_unit_vector_north * start_distance;
    const double initial_east = crossing_east + group.whale_group_starting_side * line.perpendicular_to_picket_line_unit_vector_east * start_distance;
    const double velocity_north = -group.whale_group_starting_side * line.perpendicular_to_picket_line_unit_vector_north * speed;
    const double velocity_east = -group.whale_group_starting_side * line.perpendicular_to_picket_line_unit_vector_east * speed;

    return {initial_north, initial_east, std::atan2(velocity_east, velocity_north), start_distance};
}

/**
 * Compute the picket line's length and parallel/perpendicular unit vectors.
 */
LineGeometry WhaleNode::compute_line_geometry() const {
    // Express the picket line as a north-east displacement vector.    
    const LineGeometry &configured_line = sim_state_.line_geometry;
    const double line_north = configured_line.picket_line_end_north_m - configured_line.picket_line_start_north_m;
    const double line_east = configured_line.picket_line_end_east_m - configured_line.picket_line_start_east_m;
    const double line_length = std::hypot(line_north, line_east);

    // A zero-length line cannot define parallel or perpendicular directions.
    if (line_length <= 0.0) throw std::runtime_error("Picket-line start and end points must be different.");

    // Normalize the displacement to obtain the direction along the line.
    const double along_north = line_north / line_length;
    const double along_east = line_east / line_length;

    // Rotate the along-line unit vector 90 degrees counterclockwise to obtain the perpendicular unit vector in north-east coordinates.
    return {configured_line.picket_line_start_north_m, configured_line.picket_line_start_east_m,
            configured_line.picket_line_end_north_m, configured_line.picket_line_end_east_m,
            line_length, along_north, along_east, -along_east, along_north};
}

/**
 * Create a whale with its fixed properties and inactive initial state.
 */
Whale WhaleNode::create_whale() const {
    const WhaleGroup &group = sim_state_.whale_groups.at(sim_state_.whale_generation.group_index);
    Whale whale;

    whale.id = "whale_" + std::to_string(sim_state_.whale_generation.whale_index + 1);
    whale.group_index = static_cast<int>(sim_state_.whale_generation.group_index);
    whale.start_time_s = sim_state_.whale_generation.start_time_s;
    whale.whale_active_b = false;
    whale.whale_complete_b = false;
    whale.whale_starting_side = group.whale_group_starting_side;

    whale.whale_call_times_s.clear();
    whale.next_call_index = 0;

    return whale;
}

/**
 * Generate the aggregate calling schedule observed across the simulated whale population.
 *
 * The population alternates between exponentially distributed active and silent
 * periods. Calls within active bouts follow a homogeneous Poisson process.
 */
std::vector<double> WhaleNode::generate_population_whale_call_times_s() {
    std::vector<double> whale_call_times_s;
    const double start_time_s = sim_state_.call_schedule_window.start_time_s;
    const double end_time = sim_state_.call_schedule_window.end_time_s;

    if (end_time <= start_time_s) return whale_call_times_s;

    std::exponential_distribution<double> active_duration(1.0 / sim_state_.whale_call_config.whale_call_bout_mean_duration_s);
    std::exponential_distribution<double> silent_duration(1.0 / sim_state_.whale_call_config.whale_call_bout_mean_silent_duration_s);
    std::exponential_distribution<double> call_interval(sim_state_.whale_call_config.whale_active_b_call_bout_rate_callph / 3600.0);

    const double active_probability = sim_state_.whale_call_config.whale_call_bout_mean_duration_s / (sim_state_.whale_call_config.whale_call_bout_mean_duration_s + sim_state_.whale_call_config.whale_call_bout_mean_silent_duration_s);
    std::bernoulli_distribution initial_state(active_probability);

    bool active = initial_state(sim_state_.rng);
    double interval_start = start_time_s;

    while (interval_start < end_time) {
        double duration;

        do {
            duration = active ? active_duration(sim_state_.rng) : silent_duration(sim_state_.rng);
        } while (duration <= 0.0);

        const double interval_end = std::min(interval_start + duration, end_time);

        if (active) {
            double time_until_call;

            do {
                time_until_call = call_interval(sim_state_.rng);
            } while (time_until_call <= 0.0);

            double call_time = interval_start + time_until_call;

            while (call_time < interval_end) {
                whale_call_times_s.push_back(call_time);

                do {
                    time_until_call = call_interval(sim_state_.rng);
                } while (time_until_call <= 0.0);

                call_time += time_until_call;
            }
        }

        interval_start = interval_end;
        active = !active;
    }

    return whale_call_times_s;
}

/**
 * Generate one whale's movement trajectory from its activation time to the end of the mission.
 */
std::vector<WhaleWaypoint> WhaleNode::generate_trajectory() {
    std::vector<WhaleWaypoint> trajectory;
    const WhaleInitialState &initial_state = sim_state_.whale_generation.initial_state;
    const LineGeometry &line = sim_state_.line_geometry;
    const WhaleGroup &group = sim_state_.whale_groups.at(sim_state_.whale_generation.group_index);
    const double speed = sim_state_.whale_generation.speed_mps;
    const double start_time_s = sim_state_.whales.at(sim_state_.whale_generation.whale_index).start_time_s;

    // Calculate the individual AR(1) heading-error innovation scale.
    const double heading_std_radians = sim_state_.whale_motion_config.whale_heading_std_deg * std::acos(-1.0) / 180.0;
    const double heading_innovation_scale = heading_std_radians * std::sqrt(1.0 - sim_state_.whale_motion_config.whale_heading_correlation * sim_state_.whale_motion_config.whale_heading_correlation);

    // Weight the shared and individual heading processes so their combined
    // variance equals the configured heading variance.
    const double group_weight = std::sqrt(sim_state_.whale_group_config.whale_group_heading_fraction);
    const double individual_weight = std::sqrt(1.0 - sim_state_.whale_group_config.whale_group_heading_fraction);

    std::normal_distribution<double> standard_normal(0.0, 1.0);

    double time = start_time_s;
    double north = initial_state.whale_north_m;
    double east = initial_state.whale_east_m;
    double individual_heading_error = 0.0;

    while (true) {
        // Combine the group and individual heading errors for this interval.
        const std::size_t group_heading_index = std::min(static_cast<std::size_t>(std::floor(time / sim_state_.whale_motion_config.whale_turn_interval_s)), group.whale_group_heading_errors_rad.size() - 1);
        const double combined_heading_error = group_weight * group.whale_group_heading_errors_rad[group_heading_index] + individual_weight * individual_heading_error;
        const double heading = initial_state.whale_heading_rad + combined_heading_error;

        // Convert the whale's speed and heading into north/east velocity.
        const double velocity_north = speed * std::cos(heading);
        const double velocity_east = speed * std::sin(heading);

        trajectory.push_back({time, north, east, velocity_north, velocity_east});

        if (time >= sim_state_.mission_duration_s) break;

        // End the segment at the next shared movement boundary so all group
        // members respond to heading changes at the same simulation time.
        const double next_turn_time = (std::floor(time / sim_state_.whale_motion_config.whale_turn_interval_s) + 1.0) * sim_state_.whale_motion_config.whale_turn_interval_s;
        const double next_time = std::min(next_turn_time, sim_state_.mission_duration_s);
        const double duration = next_time - time;

        north += velocity_north * duration;
        east += velocity_east * duration;
        time = next_time;

        // End nonpersistent whale trajectories after crossing through the monitored corridor.
        if (sim_state_.whale_initialization_mode == "picket_line" && !sim_state_.whales_active_entire_mission_b) {
            const double relative_north = north - line.picket_line_start_north_m;
            const double relative_east = east - line.picket_line_start_east_m;
            const double signed_distance = relative_north * line.perpendicular_to_picket_line_unit_vector_north + relative_east * line.perpendicular_to_picket_line_unit_vector_east;

            if (group.whale_group_starting_side * signed_distance <= -sim_state_.whale_initial_state.whale_start_distance_from_picket_line_m) {
                trajectory.push_back({time, north, east, 0.0, 0.0});
                break;
            }
        }

        // Preserve the final mission position without extrapolating beyond it.
        if (time >= sim_state_.mission_duration_s) {
            trajectory.push_back({time, north, east, velocity_north, velocity_east});
            break;
        }

        // Advance the whale's individual correlated heading-error process.
        individual_heading_error = sim_state_.whale_motion_config.whale_heading_correlation * individual_heading_error + heading_innovation_scale * standard_normal(sim_state_.rng);
    }

    return trajectory;
}

/**
 * Return the current whale's round-robin group index to keep group sizes balanced.
 */
std::size_t WhaleNode::get_group_index_for_whale() const {
    if (sim_state_.whale_groups.empty()) throw std::runtime_error("Cannot assign a whale because no whale groups exist.");
    return sim_state_.whale_generation.whale_index % sim_state_.whale_groups.size();
}

/**
 * Initialize shared movement and calling behavior for every whale group.
 */
void WhaleNode::initialize_whale_groups() {
    const double line_length = sim_state_.whale_initialization_mode == "picket_line" ? sim_state_.line_geometry.picket_line_length_m : 0.0;
    // Choose enough groups to approximately satisfy the configured mean size.
    const int group_count = std::max(1, static_cast<int>(std::ceil(static_cast<double>(sim_state_.whale_count) / sim_state_.whale_group_config.whale_group_mean_size)));

    // Calculate the number of shared heading intervals in the mission.
    const std::size_t heading_count = static_cast<std::size_t>(std::ceil(sim_state_.mission_duration_s / sim_state_.whale_motion_config.whale_turn_interval_s)) + 1;

    // Convert heading variation to radians and calculate the AR(1) innovation scale.
    const double heading_std_radians = sim_state_.whale_motion_config.whale_heading_std_deg * std::acos(-1.0) / 180.0;
    const double heading_innovation_scale = heading_std_radians * std::sqrt(1.0 - sim_state_.whale_motion_config.whale_heading_correlation * sim_state_.whale_motion_config.whale_heading_correlation);

    // Create distributions used to generate group-level properties.
    std::uniform_real_distribution<double> whale_group_crossing_distance_m(0.0, line_length);
    std::uniform_real_distribution<double> start_time_distribution(0.0, sim_state_.mission_duration_s);
    std::bernoulli_distribution starting_side(0.5);
    std::normal_distribution<double> standard_normal(0.0, 1.0);

    sim_state_.whale_groups.clear();
    sim_state_.whale_groups.reserve(group_count);

    for (int index = 0; index < group_count; ++index) {
        WhaleGroup group;

        // Generate the group's crossing location, starting side, and start time.
        group.whale_group_crossing_distance_m = whale_group_crossing_distance_m(sim_state_.rng);
        group.whale_group_starting_side = starting_side(sim_state_.rng) ? 1.0 : -1.0;
        group.whale_group_start_time_s = start_time_distribution(sim_state_.rng);

        // Generate the group's correlated heading-error process. Each group
        // initially travels directly toward its crossing location.
        group.whale_group_heading_errors_rad.reserve(heading_count);

        double heading_error = 0.0;
        group.whale_group_heading_errors_rad.push_back(heading_error);

        for (std::size_t heading_index = 1; heading_index < heading_count; ++heading_index) {
            heading_error = sim_state_.whale_motion_config.whale_heading_correlation * heading_error + heading_innovation_scale * standard_normal(sim_state_.rng);
            group.whale_group_heading_errors_rad.push_back(heading_error);
        }

        sim_state_.whale_groups.push_back(std::move(group));
    }

    RCLCPP_INFO(get_logger(), "Initialized %zu whale groups for %d whales.", sim_state_.whale_groups.size(), sim_state_.whale_count);
}

/**
 * Generate the whale's movement and calls and set its initial published state.
 */
void WhaleNode::initialize_whale_dynamics() {
    Whale &whale = sim_state_.whales.at(sim_state_.whale_generation.whale_index);
    whale.trajectory = generate_trajectory();
    
    if (whale.trajectory.empty()) throw std::runtime_error("Generated whale trajectory is empty.");
    
    whale.whale_call_times_s.clear();
    whale.next_call_index = 0;

    const WhaleWaypoint &first_point = whale.trajectory.front();
    whale.whale_north_m = first_point.whale_north_m;
    whale.whale_east_m = first_point.whale_east_m;
    whale.whale_velocity_north_mps = first_point.whale_velocity_north_mps;
    whale.whale_velocity_east_mps = first_point.whale_velocity_east_mps;
}

/**
 * Initialize every whale's position, movement trajectory, and call schedule.
 */
void WhaleNode::initialize_whales() {
    if (sim_state_.whale_initialization_mode == "picket_line") {
        sim_state_.line_geometry = compute_line_geometry();
    }
    initialize_whale_groups();

    sim_state_.whales.clear();
    sim_state_.whales.reserve(sim_state_.whale_count);

    for (int index = 0; index < sim_state_.whale_count; ++index) {
        sim_state_.whale_generation = WhaleGenerationContext{};
        sim_state_.whale_generation.whale_index = static_cast<std::size_t>(index);
        sim_state_.whale_generation.group_index = get_group_index_for_whale();
        sim_state_.whale_generation.start_time_s = sim_state_.whales_active_entire_mission_b ? 0.0 : sample_start_time();
        sim_state_.whale_generation.speed_mps = sample_speed();

        if (sim_state_.whale_initialization_mode == "fixed") {
            sim_state_.whale_generation.initial_state = sim_state_.whale_initial_state;
        } else {
            sim_state_.whale_generation.group_crossing_distance_m = sample_whale_group_crossing_distance_m();
            sim_state_.whale_generation.initial_state = compute_initial_state();
        }

        sim_state_.whales.push_back(create_whale());
        initialize_whale_dynamics();
    }

    assign_population_whale_call_times_s();
}

/**
 * Load whale scenario parameters supplied by the generated Compose file.
 */
void WhaleNode::load_config() {
    // Scenario size and picket-line geometry.
    sim_state_.whale_count = declare_parameter<int>("count", 46);
    sim_state_.line_geometry.picket_line_start_north_m = declare_parameter<double>("line_start_north", 0.0);
    sim_state_.line_geometry.picket_line_start_east_m = declare_parameter<double>("line_start_east", -63000.0);
    sim_state_.line_geometry.picket_line_end_north_m = declare_parameter<double>("line_end_north", 0.0);
    sim_state_.line_geometry.picket_line_end_east_m = declare_parameter<double>("line_end_east", 63000.0);
    sim_state_.whale_initial_state.whale_start_distance_from_picket_line_m = declare_parameter<double>("start_distance", 2000.0);
    sim_state_.mission_duration_s = declare_parameter<double>("mission_duration", 604800.0);

    // Whale config
    sim_state_.whales_active_entire_mission_b = declare_parameter<bool>("active_entire_mission", false);
    sim_state_.whale_initialization_mode = declare_parameter<std::string>("initialization_mode", "picket_line");
    sim_state_.whale_initial_state.whale_north_m = declare_parameter<double>("initial_north", 0.0);
    sim_state_.whale_initial_state.whale_east_m = declare_parameter<double>("initial_east", 0.0);
    sim_state_.whale_initial_state.whale_heading_rad = declare_parameter<double>("initial_heading_degrees", 0.0) * std::acos(-1.0) / 180.0;

    // Whale movement.
    sim_state_.whale_motion_config.whale_speed_mean_mps = declare_parameter<double>("speed_mean", 1.5);
    sim_state_.whale_motion_config.whale_speed_std_mps = declare_parameter<double>("speed_std", 0.2);
    sim_state_.whale_motion_config.whale_turn_interval_s = declare_parameter<double>("turn_interval", 60.0);              // Update whale heading every turn_interval s
    sim_state_.whale_motion_config.whale_heading_correlation = declare_parameter<double>("heading_correlation", 0.95);  // Controls how strongly the whale’s next heading resembles its current heading
    sim_state_.whale_motion_config.whale_heading_std_deg = declare_parameter<double>("heading_std_degrees", 15.0);

    // Whale calling behavior.
    sim_state_.whale_call_config.whale_call_bout_mean_duration_s = declare_parameter<double>("bout_mean_duration", 1842.0);
    sim_state_.whale_call_config.whale_call_bout_mean_silent_duration_s = declare_parameter<double>("bout_mean_silent_duration", 7860.0);
    sim_state_.whale_call_config.whale_active_b_call_bout_rate_callph = declare_parameter<double>("bout_active_call_rate_per_hour", 40.1);

    // Whale grouping behavior.
    sim_state_.whale_group_config.whale_group_mean_size = declare_parameter<double>("group_mean_size", 3.0);
    sim_state_.whale_group_config.whale_group_position_std_m = declare_parameter<double>("group_position_std", 1000.0);
    sim_state_.whale_group_config.whale_group_start_time_std_s = declare_parameter<double>("group_start_time_std", 600.0);
    sim_state_.whale_group_config.whale_group_heading_fraction = declare_parameter<double>("shared_heading_fraction", 0.75);

    // Output sampling.
    sim_state_.whale_sample_period_s = declare_parameter<double>("sample_period", 60.0);
}

/**
 * Open the CSV file used to record whale trajectories.
 *
 * @throws std::runtime_error If the trajectory file cannot be opened.
 */
void WhaleNode::open_whale_log() {
    sim_state_.data_directory = declare_parameter<std::string>("data_directory", "/odyssey/output/Data");
    std::filesystem::create_directories(sim_state_.data_directory);
        whale_log_.open(sim_state_.data_directory + "/whales.csv", std::ios::out | std::ios::trunc);
        if (!whale_log_.is_open()) throw std::runtime_error("Failed to open whale trajectory log.");
        whale_log_ << "cycle,sim_time_s,whale_id,group_id,north,east\n";
}

/**
 * Draw a crossing location near the group center and within the picket line.
 */
double WhaleNode::sample_whale_group_crossing_distance_m() {
    const WhaleGroup &group = sim_state_.whale_groups.at(sim_state_.whale_generation.group_index);
    const double line_length = sim_state_.line_geometry.picket_line_length_m;
    std::normal_distribution<double> offset(0.0, sim_state_.whale_group_config.whale_group_position_std_m);
    double distance;

    do {
        distance = group.whale_group_crossing_distance_m + offset(sim_state_.rng);
    } while (distance < 0.0 || distance > line_length);

    return distance;
}

/** 
 * Draw a physically valid positive whale speed.
 */
double WhaleNode::sample_speed() {
    std::normal_distribution<double> distribution(sim_state_.whale_motion_config.whale_speed_mean_mps, sim_state_.whale_motion_config.whale_speed_std_mps);
    double speed;

    do {
        speed = distribution(sim_state_.rng);
    } while (speed <= 0.0);

    return speed;
}

/**
 * Draw an activation time near the group time and within the mission.
 */
double WhaleNode::sample_start_time() {
    const WhaleGroup &group = sim_state_.whale_groups.at(sim_state_.whale_generation.group_index);
    std::normal_distribution<double> offset(0.0, sim_state_.whale_group_config.whale_group_start_time_std_s);
    double time;

    do {
        time = group.whale_group_start_time_s + offset(sim_state_.rng);
    } while (time < 0.0 || time >= sim_state_.mission_duration_s);

    return time;
}

/**
 * Seed the random-number generator used for whale behavior.
 */
void WhaleNode::seed_generator() {
    sim_state_.random_seed = declare_parameter<int64_t>("random_seed", 42);
    sim_state_.rng.seed(static_cast<std::mt19937::result_type>(sim_state_.random_seed));
}

/**
 * Validate the loaded whale-simulation configuration.
 *
 * @throws std::runtime_error If any configured value is invalid.
 */
void WhaleNode::validate_config() const {
    const double line_length = std::hypot(
        sim_state_.line_geometry.picket_line_end_north_m - sim_state_.line_geometry.picket_line_start_north_m,
        sim_state_.line_geometry.picket_line_end_east_m - sim_state_.line_geometry.picket_line_start_east_m
    );

    if (sim_state_.whale_count < 1) throw std::runtime_error("count must be at least one.");
    if (line_length <= 0.0) throw std::runtime_error("Picket line must have nonzero length.");
    if (sim_state_.whale_initial_state.whale_start_distance_from_picket_line_m <= 0.0) throw std::runtime_error("start_distance must be greater than zero.");
    if (sim_state_.mission_duration_s <= 0.0) throw std::runtime_error("mission_duration must be greater than zero.");
    if (sim_state_.whale_sample_period_s <= 0.0) throw std::runtime_error("sample_period must be greater than zero.");

    if (sim_state_.whale_initialization_mode != "picket_line" && sim_state_.whale_initialization_mode != "fixed") throw std::runtime_error("Whale initialization_mode must be 'picket_line' or 'fixed'.");
    if (!std::isfinite(sim_state_.whale_initial_state.whale_north_m) || !std::isfinite(sim_state_.whale_initial_state.whale_east_m) || !std::isfinite(sim_state_.whale_initial_state.whale_heading_rad)) throw std::runtime_error("Fixed whale initialization values must be finite.");

    if (sim_state_.whale_motion_config.whale_speed_mean_mps <= 0.0) throw std::runtime_error("speed_mean must be greater than zero.");
    if (sim_state_.whale_motion_config.whale_turn_interval_s <= 0.0) throw std::runtime_error("turn_interval must be greater than zero.");
    if (sim_state_.whale_motion_config.whale_heading_correlation < 0.0 || sim_state_.whale_motion_config.whale_heading_correlation >= 1.0) throw std::runtime_error("heading_correlation must be in [0, 1).");
    if (sim_state_.whale_motion_config.whale_heading_std_deg < 0.0) throw std::runtime_error("heading_std_degrees cannot be negative.");

    if (sim_state_.whale_call_config.whale_call_bout_mean_duration_s <= 0.0) throw std::runtime_error("Mean calling-bout duration must be positive.");
    if (sim_state_.whale_call_config.whale_call_bout_mean_silent_duration_s <= 0.0) throw std::runtime_error("Mean silent duration between calling bouts must be positive.");
    if (sim_state_.whale_call_config.whale_active_b_call_bout_rate_callph <= 0.0) throw std::runtime_error("Active-bout call rate must be positive.");

    if (sim_state_.whale_group_config.whale_group_mean_size < 1.0) throw std::runtime_error("group_mean_size must be at least one.");
    if (sim_state_.whale_group_config.whale_group_position_std_m < 0.0) throw std::runtime_error("group_position_std cannot be negative.");
    if (sim_state_.whale_group_config.whale_group_start_time_std_s < 0.0) throw std::runtime_error("group_start_time_std cannot be negative.");
    if (sim_state_.whale_group_config.whale_group_heading_fraction < 0.0 || sim_state_.whale_group_config.whale_group_heading_fraction > 1.0) throw std::runtime_error("shared_heading_fraction must be in [0, 1].");
}
