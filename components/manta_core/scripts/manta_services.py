"""Shared ROS Compose service construction for MANTA."""

import shlex

from pathlib import Path
from manta_load import container_path, load_plugin

ROOT = Path(__file__).resolve().parents[3]


def ros_service(package: str, executable: str, parameters: dict, log_dir: str, ros_domain_id: int = 1, extra_environment: dict | None = None, image: str = "odyssey_sim_dev", params_file: str | None = None) -> dict:
    """Create a Compose service for a ROS simulation node."""
    arguments = []

    if params_file is not None:
        arguments.extend(["--params-file", params_file])

    for name, value in parameters.items():
        if isinstance(value, bool):
            value = str(value).lower()

        arguments.extend(["-p", f"{name}:={value}"])

    command = shlex.join(["ros2", "run", package, executable, "--ros-args", *arguments])

    environment = {
        "ROS_DOMAIN_ID": str(ros_domain_id),
        "ROS_LOG_DIR": log_dir,
    }
    environment.update(extra_environment or {})

    return {
        "image": image,
        "environment": environment,
        "volumes": [{"type": "bind", "source": str(ROOT), "target": "/odyssey"}],
        "command": [
            "/bin/bash",
            "-lc",
            "source /opt/ros/jazzy/setup.bash\n"
            "source /odyssey/.build/ros/install/setup.bash\n"
            f"exec {command}",
        ],
    }


def vehicle_service(vehicle: dict, data_dir: str, log_dir: str, sample_period_s: float, ros_domain_id: int, tracking_enabled: bool, communication_interval_s: float | None = None) -> dict:
    """Create a vehicle service using its plugin manifest."""
    vehicle_id = vehicle["id"]
    vehicle_type = vehicle["type"]

    # Read the vehicle plugin's executable and environment mappings.
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
            f"Plugin '{vehicle_type}' is missing runtime setting: {error.args[0]}"
        ) from error

    # Supply the environment variables expected by this vehicle plugin.
    environment = {
        vehicle_id_env: vehicle_id,
        data_dir_env: data_dir,
        log_dir_env: log_dir,
        simulation_log_env: f"{log_dir}/{vehicle_id}.log",
    }

    # Convert configured vehicle files to container paths.
    for field, environment_name in config_environment.items():
        value = vehicle.get(field, "")
        environment[environment_name] = container_path(value) if value else ""

    parameters = {
        "output_sample_period_s": sample_period_s,
        "tracking_enabled": tracking_enabled,
    }

    if communication_interval_s is not None:
        parameters["communication_interval_s"] = communication_interval_s

    return ros_service(
        package=ros_package,
        executable=executable,
        parameters=parameters,
        log_dir=log_dir,
        ros_domain_id=ros_domain_id,
        extra_environment=environment,
        image=image,
        params_file=container_path(runtime["params_file"]) if runtime.get("params_file") else None,
    )