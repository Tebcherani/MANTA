import os
import subprocess
import sys
import yaml

from manta_core.manta_config import get_whale_count, load_plugin
from pathlib import Path
from paths import ROOT, DOCKER_COMPOSE_PATH, ENV_PATH


def print_config_summary(config: dict, config_path: Path) -> None:
    print(f"Loaded configuration: {config_path}")
    print(f"Configured vehicles: {len(config['vehicles'])}")

    for vehicle in config["vehicles"]:
        print(f"  - {vehicle['id']} ({vehicle['type']})")


#-------------------------------------------------------
# Compose file generation
#-------------------------------------------------------

def vehicle_service(vehicle: dict, data_dir: str, log_dir: str, sample_period: float, ros_domain_id: int, tracking_enabled: bool, call_update_interval: float | None) -> dict:
    """Create the runtime service for one configured vehicle."""

    vehicle_id = vehicle["id"]
    vehicle_type = vehicle["type"]
    manifest = load_plugin(vehicle_type)
    runtime = manifest.get("runtime", {})
    environment_names = runtime.get("environment", {})
    config_environment = runtime.get("configuration_environment", {})

    try:
        image = runtime["image"]
        ros_package = runtime["ros_package"]
        executable = runtime["executable"]
        vehicle_id_env = environment_names["vehicle_id"]
        data_dir_env = environment_names["data_directory"]
        log_dir_env = environment_names["log_directory"]
        simulation_log_env = environment_names["simulation_log"]
    except KeyError as error:
        raise ValueError(f"Plugin '{vehicle_type}' is missing runtime setting: {error.args[0]}") from error

    environment = {
        "ROS_DOMAIN_ID": str(ros_domain_id),
        vehicle_id_env: vehicle_id,
        data_dir_env: data_dir,
        log_dir_env: log_dir,
        simulation_log_env: f"{log_dir}/{vehicle_id}.log",
    }

    for field, environment_name in config_environment.items():
        value = vehicle.get(field, "")
        environment[environment_name] = container_path(value) if value else ""

    parameters = [
        f"-p output_sample_period:={sample_period}",
        f"-p tracking_enabled:={str(tracking_enabled).lower()}",
    ]

    if call_update_interval is not None:
        parameters.append(f"-p call_update_interval:={call_update_interval}")

    arguments = " ".join(parameters)

    return {
        "image": image,
        "environment": environment,
        "volumes": [
            {
                "type": "bind",
                "source": str(ROOT),
                "target": "/odyssey",
            }
        ],
        "command": [
            "/bin/bash",
            "-lc",
            "source /opt/ros/jazzy/setup.bash\n"
            "source /odyssey/.build/ros/install/setup.bash\n"
            f"exec ros2 run {ros_package} {executable} --ros-args {arguments}",
        ],
    }


def whale_service(config: dict, log_dir: str, ros_domain_id: int) -> dict:
    scenario = config["scenario"]
    scenario_type = scenario["type"]
    whales = scenario["whales"]
    movement = whales["movement"]

    if scenario_type == "trailing":
        grouping = {
            "mean_group_size": 1.0,
            "position_std": 0.0,
            "start_time_std": 0.0,
            "shared_heading_fraction": 0.75,
        }
    else:
        grouping = whales["grouping"]

    bouts = whales["calling"]["bouts"]
    monte_carlo = config.get("monte_carlo", {})

    params = {
        "count": get_whale_count(config),
        "mission_duration": scenario["duration"],
        "active_entire_mission": (
            True if scenario_type == "trailing"
            else whales.get("active_entire_mission", False)
        ),
        "bout_mean_duration": bouts["mean_duration"],
        "bout_mean_silent_duration": bouts["mean_silent_duration"],
        "bout_active_call_rate_per_hour": bouts["active_call_rate_per_hour"],
        "turn_interval": movement["turn_interval"],
        "heading_correlation": movement["heading_correlation"],
        "heading_std_degrees": movement["heading_std_degrees"],
        "speed_mean": movement["speed_mean"],
        "speed_std": movement["speed_std"],
        "group_mean_size": grouping["mean_group_size"],
        "group_position_std": grouping["position_std"],
        "group_start_time_std": grouping["start_time_std"],
        "shared_heading_fraction": grouping["shared_heading_fraction"],
        "data_directory": container_path(config["output"]["data_directory"]),
        "random_seed": monte_carlo.get("random_seed", 42),
        "sample_period": config["output"].get("whale_sample_period", 60.0),
    }

    if scenario_type == "picket_line":
        line = scenario["picket_line"]

        params.update({
            "initialization_mode": "picket_line",
            "line_start_north": line["start"]["north"],
            "line_start_east": line["start"]["east"],
            "line_end_north": line["end"]["north"],
            "line_end_east": line["end"]["east"],
            "start_distance": whales["start_distance"],
        })

    elif scenario_type == "trailing":
        params["initialization_mode"] = "fixed"

    else:
        raise ValueError(f"Whale simulation is not configured for scenario '{scenario_type}'.")

    arguments = " ".join(f'-p "{name}:={value}"' for name, value in params.items())

    return {
        "image": "odyssey_sim_dev",
        "environment": {
            "ROS_DOMAIN_ID": str(ros_domain_id),
            "ROS_LOG_DIR": log_dir,
        },
        "volumes": [
            {
                "type": "bind",
                "source": str(ROOT),
                "target": "/odyssey",
            }
        ],
        "command": [
            "/bin/bash",
            "-lc",
            "source /opt/ros/jazzy/setup.bash\n"
            "source /odyssey/.build/ros/install/setup.bash\n"
            f"exec ros2 run whale_sim whale_node --ros-args {arguments}",
        ],
        "stop_grace_period": "5s",
    }


def whale_detection_service(config: dict, log_dir: str, ros_domain_id: int) -> dict:
    """Create the whale-detection service for range-based or all-call operation."""

    vehicle_ids = [vehicle["id"] for vehicle in config["vehicles"]]
    vehicle_list = f"[{', '.join(vehicle_ids)}]"

    scenario = config["scenario"]
    detection = scenario.get("detection", {})

    receive_all_calls = (
        True if scenario["type"] == "trailing"
        else bool(detection.get("receive_all_calls", False))
    )

    data_directory = container_path(config["output"]["data_directory"])
    call_position_std_m = float(config["scenario"].get("trailing", {}).get("call_position_std_m", 0.0))
    call_position_seed = int(config.get("monte_carlo", {}).get("random_seed", 0))

    parameters = [
        f'-p "vehicle_ids:={vehicle_list}"',
        f'-p "receive_all_calls:={str(receive_all_calls).lower()}"',
        f'-p "data_directory:={data_directory}"',
        f'-p "call_position_std_m:={call_position_std_m}"',
        f'-p "call_position_seed:={call_position_seed}"',
    ]

    if not receive_all_calls:
        if "radius" not in detection: raise ValueError("Range-based whale detection requires scenario.detection.radius.")
        parameters.append(f'-p "detection_range:={detection["radius"]}"')

    arguments = " ".join(parameters)

    return {
        "image": "odyssey_sim_dev",
        "environment": {
            "ROS_DOMAIN_ID": str(ros_domain_id),
            "ROS_LOG_DIR": log_dir,
        },
        "volumes": [
            {
                "type": "bind",
                "source": str(ROOT),
                "target": "/odyssey",
            }
        ],
        "command": [
            "/bin/bash",
            "-lc",
            "source /opt/ros/jazzy/setup.bash\n"
            "source /odyssey/.build/ros/install/setup.bash\n"
            f"exec ros2 run whale_detection_sim whale_detection_node --ros-args {arguments}",
        ],
    }