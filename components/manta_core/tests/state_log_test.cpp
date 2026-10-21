// Standalone regression test: compile with the core, whale, detector, and
// generated ROS interface include directories; no ROS runtime is required.
#include <cassert>

#include "whale_sim/whale_state_json.hpp"
#include "whale_detection_sim/whale_detection_state_json.hpp"

template<class T>
std::string snapshot(const T &state) {
    std::ostringstream out;
    write_json(out, state);
    return out.str();
}

template<class T, class Change>
void check_change(const T &original, Change change) {
    T changed = original;
    change(changed);
    assert(snapshot(original) != snapshot(changed));
}

int main(int argc, char **argv) {
    assert(argc == 2);
    WhaleState whale;
    whale.whales.emplace_back();
    whale.whales.front().trajectory.emplace_back();
    whale.whale_groups.emplace_back();
    check_change(whale, [](auto &s) { s.whales.front().trajectory.front().whale_east_m = 2.0; });
    check_change(whale, [](auto &s) { s.whales.front().next_call_index++; });
    check_change(whale, [](auto &s) { s.whales.front().whale_call_times_s.push_back(1.0); });
    check_change(whale, [](auto &s) { s.whale_groups.front().whale_group_heading_errors_rad.push_back(0.1); });
    check_change(whale, [](auto &s) { s.whale_group_config.whale_group_mean_size = 2.0; });
    check_change(whale, [](auto &s) { s.whale_motion_config.whale_speed_mean_mps = 1.5; });
    check_change(whale, [](auto &s) { s.whale_call_config.whale_call_bout_mean_duration_s = 60.0; });
    check_change(whale, [](auto &s) { s.pending_sim_step.step_count = 2; });
    check_change(whale, [](auto &s) { s.whale_call_after_time_s = 10.0; });
    check_change(whale, [](auto &s) { s.whale_generation.speed_mps = 1.5; });
    check_change(whale, [](auto &s) { s.trajectory_query.sim_time_s = 10.0; });
    check_change(whale, [](auto &s) { s.call_schedule_window.end_time_s = 60.0; });
    check_change(whale, [](auto &s) { s.current_whale_states.whales.emplace_back(); });
    check_change(whale, [](auto &s) { ++s.random_seed; });
    auto advanced_whale = whale;
    advanced_whale.rng();
    assert(snapshot(whale) == snapshot(advanced_whale));
    check_change(whale, [](auto &s) { s.next_whale_call.whale_id = "whale_1"; });
    check_change(whale, [](auto &s) { s.shutdown_requested_b = true; });
    check_change(whale, [](auto &s) { s.sim_time_s = std::nextafter(0.0, 1.0); });

    WhaleDetectionState detector;
    check_change(detector, [](auto &s) { s.current_vehicle_id = "auv_2"; });
    check_change(detector, [](auto &s) { s.vehicle_to_whale_range_m = 42.0; });
    check_change(detector, [](auto &s) { s.bearing_observation.bearing_rad = 0.5; });
    check_change(detector, [](auto &s) { s.bearing_observation_available_b = true; });
    check_change(detector, [](auto &s) { s.whale_call_processed.bearing_receiver_ids.push_back("auv_1"); });
    detector.latest_vehicle_states["auv_1"].vehicle_id = "auv_1";
    check_change(detector, [](auto &s) { s.latest_vehicle_states.at("auv_1").depth = 4.0; });
    check_change(detector, [](auto &s) { ++s.bearing_noise_seed; });
    auto advanced_detector = detector;
    advanced_detector.normal(advanced_detector.rng);
    assert(snapshot(detector) == snapshot(advanced_detector));
    assert(snapshot(detector).find("\"rng\"") == std::string::npos);
    assert(snapshot(detector).find("\"normal\"") == std::string::npos);
    check_change(detector, [](auto &s) { s.pending_whale_call.cycle++; });
    check_change(detector, [](auto &s) { s.whale_detection.vehicle_to_whale_range_m = 1.0; });
    check_change(detector, [](auto &s) { s.whale_detection_available_b = true; });
    check_change(detector, [](auto &s) { s.detection_range_m = std::nextafter(s.detection_range_m, 0.0); });
    check_change(detector, [](auto &s) { s.data_directory = "quote\"\\\n\t\001"; });

    auto reordered = detector;
    detector.latest_vehicle_states["auv_2"].depth = 5.0;
    reordered.latest_vehicle_states.clear();
    reordered.latest_vehicle_states["auv_2"].depth = 5.0;
    reordered.latest_vehicle_states["auv_1"].vehicle_id = "auv_1";
    assert(snapshot(detector) == snapshot(reordered));

    manta::StateLog log;
    log.open(argv[1]);
    log.record(detector);
    log.record(reordered);  // Same values in another map order: no duplicate.
    detector.data_directory = "quote\"\\\n\t\001";
    log.record(detector);
    log.record(detector);  // Repeated state: no duplicate.
    detector.whale_detection_available_b = true;
    log.record(detector);
    std::ifstream input(argv[1]);  // Records must already be flushed.
    std::string line;
    int count = 0;
    while (std::getline(input, line)) ++count;
    assert(count == 3);

    manta::StateLog whale_log;
    whale_log.open(std::string(argv[1]) + ".whale");
    const auto initial_encoder = [](std::ostream &out, const WhaleState &state) { write_json(out, state); };
    whale.whales.front().whale_call_times_s.push_back(60.0);
    whale.whale_groups.front().whale_group_heading_errors_rad.push_back(0.1);
    whale_log.record(whale, write_whale_runtime_json, initial_encoder);
    whale_log.record(whale, write_whale_runtime_json, initial_encoder);
    whale.sim_time_s = 10.0;
    whale.whales.front().whale_north_m = 12.0;
    whale_log.record(whale, write_whale_runtime_json, initial_encoder);
    whale_log.record(whale, write_whale_runtime_json, initial_encoder);
    std::ifstream whale_input(std::string(argv[1]) + ".whale");
    std::string initial_line, update_line, extra_line;
    assert(std::getline(whale_input, initial_line));
    assert(std::getline(whale_input, update_line));
    assert(!std::getline(whale_input, extra_line));
    assert(initial_line.find("\"trajectory\"") != std::string::npos);
    assert(initial_line.find("\"whale_call_times_s\"") != std::string::npos);
    assert(initial_line.find("\"whale_group_heading_errors_rad\"") != std::string::npos);
    assert(update_line.find("\"trajectory\"") == std::string::npos);
    assert(update_line.find("\"whale_call_times_s\"") == std::string::npos);
    assert(update_line.find("\"whale_group_heading_errors_rad\"") == std::string::npos);
    assert(update_line.find("\"whale_north_m\":12") != std::string::npos);

    const std::string log_directory = std::string(argv[1]) + ".logs";
    assert(setenv("ROS_LOG_DIR", log_directory.c_str(), 1) == 0);
    manta::StateLog routed_log;
    routed_log.open_in_log_directory("whale_state.jsonl");
    routed_log.record(whale);
    assert(std::filesystem::is_regular_file(std::filesystem::path(log_directory) / "whale_state.jsonl"));

    // Log failures must be reported instead of silently losing records.
    manta::StateLog unopened;
    bool failed = false;
    try { unopened.record(whale); }
    catch (const std::runtime_error &) { failed = true; }
    assert(failed);
    return 0;
}
