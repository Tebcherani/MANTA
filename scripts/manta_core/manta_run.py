import os
import subprocess
import sys
import yaml

from manta_core.manta_config import load_plugin
from pathlib import Path
from paths import ROOT, DOCKER_COMPOSE_PATH, ENV_PATH, PICKET_LINE_MISSION_GENERATOR_PATH


def container_path(configured_path: str) -> str:
    """Convert a repository-relative host path to its `/odyssey` container path."""
    host_path = (ROOT / configured_path).resolve()

    try:
        relative_path = host_path.relative_to(ROOT.resolve())
    except ValueError as error:
        raise ValueError(f"Path must remain inside MANTA: {host_path}") from error

    return f"/odyssey/{relative_path.as_posix()}"


def generate_missions(config: dict, config_path: Path) -> None:
    """Generate mission file for the picket line scenario with the given configuration."""
    scenario = config.get("scenario", {})
    if scenario.get("type") != "picket_line": return
    result = subprocess.run([sys.executable, str(PICKET_LINE_MISSION_GENERATOR_PATH), str(config_path)], cwd=ROOT, check=False)
    if result.returncode != 0: raise ValueError("Failed to generate picket-line missions.")


def prepare_runtime(sim_config: dict, config_path: Path) -> tuple[Path, Path]:
    """Generate missions, runtime environment, and Docker Compose configuration."""

    generate_missions(sim_config, config_path)
    print_config_summary(sim_config, config_path)

    write_runtime_env(sim_config)
    compose_path = write_runtime_compose(sim_config, DOCKER_COMPOSE_PATH)

    print(f"Generated environment: {ENV_PATH}")
    print(f"Generated Compose file: {compose_path}")

    return compose_path


def print_config_summary(config: dict, config_path: Path) -> None:
    print(f"Loaded configuration: {config_path}")
    print(f"Configured vehicles: {len(config['vehicles'])}")

    for vehicle in config["vehicles"]:
        print(f"  - {vehicle['id']} ({vehicle['type']})")


#-------------------------------------------------------
# Compose file generation
#-------------------------------------------------------

def vehicle_service(vehicle: dict, data_dir: str, log_dir: str, sample_period: float, ros_domain_id: int) -> dict:
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
        raise ValueError(
            f"Plugin '{vehicle_type}' is missing runtime setting: "
            f"{error.args[0]}"
        ) from error

    environment = {
        "ROS_DOMAIN_ID": str(ros_domain_id),
        vehicle_id_env: vehicle_id,
        data_dir_env: data_dir,
        log_dir_env: log_dir,
        simulation_log_env: f"{log_dir}/{vehicle_id}.log",
    }

    for field, environment_name in config_environment.items():
        value = vehicle.get(field, "")
        environment[environment_name] = (container_path(value) if value else "")

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
            "source /odyssey/ros_ws/install/setup.bash\n"
            f"ros2 run {ros_package} {executable} --ros-args "
            f"-p output_sample_period:={sample_period}",
        ],
    }


def whale_service(config: dict, log_dir: str, ros_domain_id: int) -> dict:
    scenario = config["scenario"]
    line = scenario["picket_line"]
    whales = scenario["whales"]
    movement = whales["movement"]
    grouping = whales["grouping"]
    calling = whales["calling"]
    bouts = calling["bouts"]
    monte_carlo = config.get("monte_carlo", {})

    params = {
        "count": whales["count"],
        "line_start_north": line["start"]["north"],
        "line_start_east": line["start"]["east"],
        "line_end_north": line["end"]["north"],
        "line_end_east": line["end"]["east"],
        "start_distance": whales["start_distance"],
        "depth": whales["depth"],
        "mission_duration": scenario["duration"],
        "bout_mean_duration": bouts["mean_duration"],
        "bout_mean_silent_duration": bouts["mean_silent_duration"],
        "bout_active_call_rate_per_hour": bouts["active_call_rate_per_hour"],
        "turn_interval": movement["turn_interval"],
        "heading_correlation": movement["heading_correlation"],
        "heading_std_degrees": movement["heading_std_degrees"],
        "group_mean_size": grouping["mean_group_size"],
        "group_position_std": grouping["position_std"],
        "group_start_time_std": grouping["start_time_std"],
        "shared_heading_fraction": grouping["shared_heading_fraction"],
        "data_directory": container_path(config["output"]["data_directory"]),
        "random_seed": monte_carlo.get("random_seed", 42),
        "sample_period": config["output"].get("whale_sample_period", 60.0),
    }

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
            "source /odyssey/ros_ws/install/setup.bash\n"
            f"exec ros2 run whale_sim whale_node --ros-args {arguments}",
        ],
        "stop_grace_period": "5s",
    }


def whale_detection_service(config: dict, log_dir: str, ros_domain_id: int) -> dict:
    vehicle_ids = [vehicle["id"] for vehicle in config["vehicles"]]
    vehicle_list = f"[{', '.join(vehicle_ids)}]"
    detection_range = config["scenario"]["detection"]["radius"]
    data_directory = container_path(config["output"]["data_directory"])

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
            "source /odyssey/ros_ws/install/setup.bash\n"
            "ros2 run whale_detection_sim whale_detection_node "
            f'--ros-args -p "vehicle_ids:={vehicle_list}" '
            f'-p "detection_range:={detection_range}" '
            f'-p "data_directory:={data_directory}"',
        ],
    }


def write_runtime_compose(config: dict, compose_path: Path | None = None, project_name: str = "manta", ros_domain_id: int = 1) -> Path:
    if compose_path is None: compose_path = DOCKER_COMPOSE_PATH
    compose_path.parent.mkdir(parents=True, exist_ok=True)

    data_dir = container_path(config["output"]["data_directory"])
    log_dir = container_path(config["output"]["log_directory"])
    vehicle_ids = [vehicle["id"] for vehicle in config["vehicles"]]
    simulation = config["simulation"]
    sample_period = float(config["output"].get("auv_sample_period", 2.0))

    services = {
        vehicle["id"]: vehicle_service(vehicle, data_dir, log_dir, sample_period, ros_domain_id)
        for vehicle in config["vehicles"]
    }

    if config.get("scenario", {}).get("type") == "picket_line":
        services["whale_sim"] = whale_service(config, log_dir, ros_domain_id)
        services["whale_detection_sim"] = whale_detection_service(config, log_dir, ros_domain_id)

    vehicle_list = f"[{', '.join(vehicle_ids)}]"

    services["sim_orchestrator"] = {
        "image": "odyssey_sim_dev",
        "environment": {
            "ROS_DOMAIN_ID": str(ros_domain_id),
            "ROS_LOG_DIR": log_dir,
            "SIM_LOG_PATH": f"{log_dir}/sim_orchestrator.log",
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
            "source /odyssey/ros_ws/install/setup.bash\n"
            "ros2 run sim_orchestrator sim_orchestrator_node "
            f'--ros-args -p "vehicle_ids:={vehicle_list}" '
            f'-p "data_directory:={data_dir}" '
            f'-p "timestep:={simulation["timestep"]}" '
            f'-p "control_period:={simulation["control_period"]}" '
            f'-p "max_steps:={simulation["max_steps"]}" '
            f'-p "steps_per_message:='
            f'{simulation.get("steps_per_message", 1)}"',
        ],
    }

    document = {
        "name": project_name,
        "services": services,
    }

    compose_path.write_text(yaml.safe_dump(document, sort_keys=False), encoding="utf-8")
    return compose_path


#-------------------------------------------------------
# Environment file generation
#-------------------------------------------------------

def write_runtime_env(config: dict) -> None:
    """Write Docker Compose environment variables for the configured MANTA fleet."""
    ENV_PATH.parent.mkdir(parents=True, exist_ok=True)
    vehicle_ids = [vehicle["id"] for vehicle in config["vehicles"]]

    lines = [
        f"HOST_UID={os.getuid()}",
        f"HOST_GID={os.getgid()}",
        f"MANTA_VEHICLE_IDS=[{', '.join(vehicle_ids)}]",
        f"MANTA_DATA_DIR={container_path(config['output']['data_directory'])}",
        f"MANTA_LOG_DIR={container_path(config['output']['log_directory'])}",
    ]

    for vehicle in config["vehicles"]:
        prefix = "".join(character.upper() if character.isalnum() else "_" for character in vehicle["id"])
        fault_path = container_path(vehicle["fault"]) if vehicle.get("fault") else ""

        lines.extend([
            f"{prefix}_TYPE={vehicle['type']}",
            f"{prefix}_MISSION={container_path(vehicle['mission'])}",
            f"{prefix}_MODEL={container_path(vehicle['model'])}",
            f"{prefix}_FAULT={fault_path}",
        ])

    ENV_PATH.write_text("\n".join(lines) + "\n", encoding="utf-8")