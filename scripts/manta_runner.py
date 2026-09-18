#!/usr/bin/env python3

import argparse
import copy
import csv
import math
import os
import signal
import subprocess
import sys
import threading
from concurrent.futures import ThreadPoolExecutor, as_completed
from pathlib import Path
from statistics import NormalDist, mean, stdev

import yaml


stop_requested = threading.Event()


def handle_interrupt(signal_number, frame) -> None:
    """Stop every active Monte Carlo project and suppress retries."""

    if stop_requested.is_set():
        return

    print("\nStopping MANTA Monte Carlo...", file=sys.stderr)
    stop_requested.set()

    cleanup_script = (
        Path(__file__).resolve().parent
        / "cleanup_monte_carlo.sh"
    )

    subprocess.run(
        ["bash", str(cleanup_script)],
        check=False,
    )


def cleanup_monte_carlo_projects(root: Path) -> None:
    """Remove all MANTA Monte Carlo containers and networks."""

    cleanup_script = root / "scripts" / "cleanup_monte_carlo.sh"

    if not cleanup_script.is_file():
        raise ValueError(
            f"Monte Carlo cleanup script is missing: {cleanup_script}"
        )

    result = subprocess.run(
        ["bash", str(cleanup_script)],
        cwd=root,
        check=False,
    )

    if result.returncode != 0:
        raise ValueError(
            "Failed to clean up MANTA Monte Carlo Docker projects."
        )


def load_config(path: Path) -> dict:
    if not path.is_file():
        raise ValueError(f"Configuration file does not exist: {path}")

    try:
        with path.open("r", encoding="utf-8") as file:
            config = yaml.safe_load(file)
    except yaml.YAMLError as error:
        raise ValueError(f"Invalid YAML in {path}: {error}") from error

    if not isinstance(config, dict):
        raise ValueError("The configuration must contain a YAML mapping.")

    return config


def load_plugin(root: Path, plugin_id: str) -> dict:
    path = root / "vehicles" / plugin_id / "manta_plugin.yaml"

    if not path.is_file():
        raise ValueError(f"Vehicle plugin '{plugin_id}' was not found: {path}")

    manifest = load_config(path)
    plugin = manifest.get("plugin")

    if not isinstance(plugin, dict):
        raise ValueError(f"Plugin '{plugin_id}' has no 'plugin' section.")

    if plugin.get("id") != plugin_id:
        raise ValueError(
            f"Plugin manifest ID '{plugin.get('id')}' does not match "
            f"configured type '{plugin_id}'."
        )

    if plugin.get("kind") != "vehicle":
        raise ValueError(f"Plugin '{plugin_id}' is not a vehicle plugin.")

    return manifest


def validate_config(config: dict, root: Path) -> None:
    vehicles = config.get("vehicles")
    output = config.get("output")

    if not isinstance(vehicles, list) or not vehicles:
        raise ValueError("'vehicles' must be a nonempty YAML list.")

    if not isinstance(output, dict):
        raise ValueError("'output' must be a YAML mapping.")

    for field in ("data_directory", "log_directory"):
        if not isinstance(output.get(field), str) or not output[field]:
            raise ValueError(f"'output.{field}' must be a nonempty string.")

    for field in ("auv_sample_period", "whale_sample_period"):
        value = output.get(field)

        if value is not None and (
            not isinstance(value, (int, float)) or value <= 0
        ):
            raise ValueError(f"'output.{field}' must be greater than zero.")

    vehicle_ids = set()

    for vehicle in vehicles:
        if not isinstance(vehicle, dict):
            raise ValueError("Every vehicle must be a YAML mapping.")

        vehicle_id = vehicle.get("id")
        vehicle_type = vehicle.get("type")

        if not isinstance(vehicle_id, str) or not vehicle_id:
            raise ValueError("Every vehicle must have a nonempty string 'id'.")

        if vehicle_id in vehicle_ids:
            raise ValueError(f"Duplicate vehicle ID: {vehicle_id}")

        vehicle_ids.add(vehicle_id)

        if not isinstance(vehicle_type, str) or not vehicle_type:
            raise ValueError(
                f"Vehicle '{vehicle_id}' must have a nonempty string 'type'."
            )

        manifest = load_plugin(root, vehicle_type)
        plugin_config = manifest.get("configuration", {})
        required_fields = plugin_config.get("required", [])
        optional_fields = plugin_config.get("optional", [])

        if not isinstance(required_fields, list) or not isinstance(
            optional_fields, list
        ):
            raise ValueError(
                f"Plugin '{vehicle_type}' configuration fields must be YAML lists."
            )

        for field in required_fields:
            value = vehicle.get(field)

            if not isinstance(value, str) or not value:
                raise ValueError(
                    f"Vehicle '{vehicle_id}' must define a nonempty "
                    f"'{field}' path."
                )

        for field in required_fields + optional_fields:
            value = vehicle.get(field, "")

            if not isinstance(value, str):
                raise ValueError(
                    f"Vehicle '{vehicle_id}' field '{field}' must be a string."
                )

            if value:
                path = (root / value).resolve()

                if not path.is_file():
                    raise ValueError(
                        f"Vehicle '{vehicle_id}' {field} file does not exist: "
                        f"{path}"
                    )


def validate_monte_carlo_config(config: dict) -> dict:
    monte_carlo = config.get("monte_carlo", {})

    if not isinstance(monte_carlo, dict):
        raise ValueError("'monte_carlo' must be a YAML mapping.")

    integer_fields = (
        "min_realizations",
        "max_realizations",
        "batch_size",
        "random_seed",
    )

    for field in integer_fields:
        if not isinstance(monte_carlo.get(field), int):
            raise ValueError(
                f"'monte_carlo.{field}' must be an integer."
            )

    min_realizations = monte_carlo["min_realizations"]
    max_realizations = monte_carlo["max_realizations"]
    batch_size = monte_carlo["batch_size"]

    if min_realizations < 2:
        raise ValueError(
            "'monte_carlo.min_realizations' must be at least 2."
        )

    if max_realizations < min_realizations:
        raise ValueError(
            "'monte_carlo.max_realizations' must be greater than "
            "or equal to min_realizations."
        )

    if batch_size < 1:
        raise ValueError(
            "'monte_carlo.batch_size' must be at least 1."
        )

    workers = monte_carlo.get("workers")

    if workers is not None and (
        not isinstance(workers, int) or workers < 1
    ):
        raise ValueError(
            "'monte_carlo.workers' must be a positive integer."
        )

    realization_timeout = monte_carlo.get("realization_timeout", 60)
    max_retries = monte_carlo.get("max_retries", 2)

    if not isinstance(realization_timeout, int) or realization_timeout < 1:
        raise ValueError(
            "'monte_carlo.realization_timeout' must be a positive integer."
        )

    if not isinstance(max_retries, int) or max_retries < 0:
        raise ValueError(
            "'monte_carlo.max_retries' cannot be negative."
        )

    numeric_fields = (
        "confidence_level",
        "target_half_width",
        "stability_tolerance",
    )

    for field in numeric_fields:
        if not isinstance(monte_carlo.get(field), (int, float)):
            raise ValueError(
                f"'monte_carlo.{field}' must be numeric."
            )

    confidence_level = float(monte_carlo["confidence_level"])
    target_half_width = float(monte_carlo["target_half_width"])
    stability_tolerance = float(monte_carlo["stability_tolerance"])

    if not 0.0 < confidence_level < 1.0:
        raise ValueError(
            "'monte_carlo.confidence_level' must be between 0 and 1."
        )

    if target_half_width <= 0.0:
        raise ValueError(
            "'monte_carlo.target_half_width' must be greater than zero."
        )

    if stability_tolerance < 0.0:
        raise ValueError(
            "'monte_carlo.stability_tolerance' cannot be negative."
        )

    return monte_carlo


def print_summary(config: dict, path: Path) -> None:
    print(f"Loaded configuration: {path}")
    print(f"Configured vehicles: {len(config['vehicles'])}")

    for vehicle in config["vehicles"]:
        print(f"  - {vehicle['id']} ({vehicle['type']})")


def container_path(root: Path, configured_path: str) -> str:
    host_path = (root / configured_path).resolve()

    try:
        relative_path = host_path.relative_to(root.resolve())
    except ValueError as error:
        raise ValueError(f"Path must remain inside MANTA: {host_path}") from error

    return f"/odyssey/{relative_path.as_posix()}"


def generate_scenario_missions(config: dict, config_path: Path, root: Path) -> None:
    scenario = config.get("scenario", {})

    if scenario.get("type") != "picket_line":
        return

    generator = (
        root
        / "missions"
        / "picket_line"
        / "generate_picket_line_mission.py"
    )

    result = subprocess.run(
        [sys.executable, str(generator), str(config_path)],
        cwd=root,
        check=False,
    )

    if result.returncode != 0:
        raise ValueError("Failed to generate picket-line missions.")


def write_runtime_env(
    config: dict,
    root: Path,
    path: Path | None = None,
) -> Path:
    if path is None:
        path = root / "generated" / "manta.env"

    path.parent.mkdir(parents=True, exist_ok=True)
    vehicle_ids = [vehicle["id"] for vehicle in config["vehicles"]]

    lines = [
        f"HOST_UID={os.getuid()}",
        f"HOST_GID={os.getgid()}",
        f"MANTA_VEHICLE_IDS=[{', '.join(vehicle_ids)}]",
        f"MANTA_DATA_DIR={container_path(root, config['output']['data_directory'])}",
        f"MANTA_LOG_DIR={container_path(root, config['output']['log_directory'])}",
    ]

    for vehicle in config["vehicles"]:
        prefix = "".join(
            character.upper() if character.isalnum() else "_"
            for character in vehicle["id"]
        )

        lines.extend(
            [
                f"{prefix}_TYPE={vehicle['type']}",
                f"{prefix}_MISSION={container_path(root, vehicle['mission'])}",
                f"{prefix}_MODEL={container_path(root, vehicle['model'])}",
                (
                    f"{prefix}_FAULT="
                    f"{container_path(root, vehicle['fault']) if vehicle.get('fault') else ''}"
                ),
            ]
        )

    path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return path


def vehicle_service(
    vehicle: dict,
    root: Path,
    data_dir: str,
    log_dir: str,
    sample_period: float,
    ros_domain_id: int,
) -> dict:
    vehicle_id = vehicle["id"]
    vehicle_type = vehicle["type"]
    manifest = load_plugin(root, vehicle_type)
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
        environment[environment_name] = (
            container_path(root, value) if value else ""
        )

    return {
        "image": image,
        "environment": environment,
        "volumes": [
            {
                "type": "bind",
                "source": str(root),
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


def whale_service(config: dict, root: Path, log_dir: str, ros_domain_id: int) -> dict:
    scenario = config["scenario"]
    line = scenario["picket_line"]
    whales = scenario["whales"]
    movement = whales["movement"]
    grouping = whales["grouping"]
    calling = whales["calling"]
    monte_carlo = scenario.get("monte_carlo", {})

    params = {
        "count": whales["count"],
        "line_start_north": line["start"]["north"],
        "line_start_east": line["start"]["east"],
        "line_end_north": line["end"]["north"],
        "line_end_east": line["end"]["east"],
        "speed_min": movement["speed_min"],
        "speed_max": movement["speed_max"],
        "start_distance": whales["start_distance"],
        "depth": whales["depth"],
        "mission_duration": scenario["duration"],
        "call_rate_min": calling["call_rate_min"],
        "call_rate_max": calling["call_rate_max"],
        "call_activity_interval": calling["activity_interval"],
        "call_activity_correlation": calling["activity_correlation"],
        "group_log_rate_std": calling["group_log_rate_std"],
        "turn_interval": movement["turn_interval"],
        "heading_correlation": movement["heading_correlation"],
        "heading_std_degrees": movement["heading_std_degrees"],
        "group_mean_size": grouping["mean_group_size"],
        "group_position_std": grouping["position_std"],
        "group_start_time_std": grouping["start_time_std"],
        "shared_heading_fraction": grouping["shared_heading_fraction"],
        "data_directory": container_path(root, config["output"]["data_directory"]),
        "random_seed": monte_carlo.get("random_seed", 42),
        "sample_period": config["output"].get("whale_sample_period", 60.0),
    }

    arguments = " ".join(
        f'-p "{name}:={value}"' for name, value in params.items()
    )

    return {
        "image": "odyssey_sim_dev",
        "environment": {
            "ROS_DOMAIN_ID": str(ros_domain_id),
            "ROS_LOG_DIR": log_dir,
        },
        "volumes": [
            {
                "type": "bind",
                "source": str(root),
                "target": "/odyssey",
            }
        ],
        "command": [
            "/bin/bash",
            "-lc",
            "source /opt/ros/jazzy/setup.bash\n"
            "source /odyssey/ros_ws/install/setup.bash\n"
            f"ros2 run whale_sim whale_node --ros-args {arguments}",
        ],
    }


def whale_detection_service(
    config: dict,
    root: Path,
    log_dir: str,
    ros_domain_id: int,
) -> dict:
    vehicle_ids = [vehicle["id"] for vehicle in config["vehicles"]]
    vehicle_list = f"[{', '.join(vehicle_ids)}]"
    detection_range = config["scenario"]["detection"]["radius"]
    data_directory = container_path(
        root, config["output"]["data_directory"]
    )

    return {
        "image": "odyssey_sim_dev",
        "environment": {
            "ROS_DOMAIN_ID": str(ros_domain_id),
            "ROS_LOG_DIR": log_dir,
        },
        "volumes": [
            {
                "type": "bind",
                "source": str(root),
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


def write_runtime_compose(
    config: dict,
    root: Path,
    path: Path | None = None,
    project_name: str = "manta",
    ros_domain_id: int = 1,
) -> Path:
    if path is None:
        path = root / "generated" / "compose.ros.yaml"

    path.parent.mkdir(parents=True, exist_ok=True)

    data_dir = container_path(
        root, config["output"]["data_directory"]
    )
    log_dir = container_path(
        root, config["output"]["log_directory"]
    )
    vehicle_ids = [vehicle["id"] for vehicle in config["vehicles"]]
    simulation = config["simulation"]
    sample_period = float(
        config["output"].get("auv_sample_period", 2.0)
    )

    services = {
        vehicle["id"]: vehicle_service(
            vehicle,
            root,
            data_dir,
            log_dir,
            sample_period,
            ros_domain_id,
        )
        for vehicle in config["vehicles"]
    }

    if config.get("scenario", {}).get("type") == "picket_line":
        services["whale_sim"] = whale_service(
            config,
            root,
            log_dir,
            ros_domain_id,
        )
        services["whale_detection_sim"] = whale_detection_service(
            config,
            root,
            log_dir,
            ros_domain_id,
        )

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
                "source": str(root),
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
            f'-p "max_cycles:={simulation["max_cycles"]}" '
            f'-p "steps_per_message:='
            f'{simulation.get("steps_per_message", 1)}"',
        ],
    }

    document = {
        "name": project_name,
        "services": services,
    }

    path.write_text(
        yaml.safe_dump(document, sort_keys=False),
        encoding="utf-8",
    )

    return path


def summarize_realization(
    data_directory: Path,
    whale_count: int,
) -> tuple[int, float]:
    detections_path = data_directory / "whale_detections.csv"

    if not detections_path.is_file():
        raise ValueError(
            f"Detection results were not created: {detections_path}"
        )

    detected_whales = set()

    with detections_path.open(
        "r",
        encoding="utf-8",
        newline="",
    ) as file:
        reader = csv.DictReader(file)

        if not reader.fieldnames or "whale_id" not in reader.fieldnames:
            raise ValueError(
                f"Detection results do not contain a whale_id column: "
                f"{detections_path}"
            )

        for row in reader:
            whale_id = row.get("whale_id", "").strip()

            if whale_id:
                detected_whales.add(whale_id)

    unique_count = len(detected_whales)
    detection_fraction = unique_count / whale_count
    return unique_count, detection_fraction


def summarize_vehicle_metrics(
    data_directory: Path,
    vehicle_ids: list[str],
) -> tuple[float, float, float]:
    """Combine final metrics written by every vehicle in one realization."""

    total_mass_kg = 0.0
    total_energy_wh = 0.0
    vehicle_speeds = []
    required = {
        "vehicle_id",
        "mass_kg",
        "energy_consumed_wh",
        "mean_speed_mps",
    }

    for vehicle_id in vehicle_ids:
        path = data_directory / f"{vehicle_id}_metrics.csv"

        if not path.is_file():
            raise ValueError(f"Vehicle metrics were not created: {path}")

        with path.open("r", encoding="utf-8", newline="") as file:
            reader = csv.DictReader(file)

            if not reader.fieldnames or not required.issubset(
                reader.fieldnames
            ):
                raise ValueError(
                    f"Vehicle metrics have an invalid header: {path}"
                )

            row = next(reader, None)

        if row is None:
            raise ValueError(f"Vehicle metrics contain no data row: {path}")

        if row["vehicle_id"] != vehicle_id:
            raise ValueError(
                f"Expected metrics for '{vehicle_id}', but {path} "
                f"contains '{row['vehicle_id']}'."
            )

        try:
            mass_kg = float(row["mass_kg"])
            energy_wh = float(row["energy_consumed_wh"])
            speed_mps = float(row["mean_speed_mps"])
        except (TypeError, ValueError) as error:
            raise ValueError(
                f"Vehicle metrics contain invalid numbers: {path}"
            ) from error

        values = (mass_kg, energy_wh, speed_mps)

        if not all(math.isfinite(value) for value in values):
            raise ValueError(
                f"Vehicle metrics contain non-finite values: {path}"
            )

        if mass_kg <= 0.0:
            raise ValueError(f"Vehicle mass must be positive: {path}")

        if energy_wh < 0.0 or speed_mps < 0.0:
            raise ValueError(
                f"Vehicle energy and speed cannot be negative: {path}"
            )

        total_mass_kg += mass_kg
        total_energy_wh += energy_wh
        vehicle_speeds.append(speed_mps)

    if not vehicle_speeds:
        raise ValueError(
            "A realization must contain at least one vehicle metric."
        )

    return total_mass_kg, total_energy_wh, mean(vehicle_speeds)


def calculate_monte_carlo_statistics(
    detection_fractions: list[float],
    confidence_level: float,
) -> tuple[float, float]:
    if not detection_fractions:
        raise ValueError("At least one realization is required.")

    sample_mean = mean(detection_fractions)

    if len(detection_fractions) < 2:
        return sample_mean, math.inf

    standard_error = stdev(detection_fractions) / math.sqrt(
        len(detection_fractions)
    )
    z_score = NormalDist().inv_cdf(
        0.5 + confidence_level / 2.0
    )
    half_width = z_score * standard_error

    return sample_mean, half_width


def monte_carlo_paths(
    output_directory: Path,
    realization: int,
) -> tuple[Path, Path, Path, Path]:
    run_name = f"realization_{realization:04d}"
    run_directory = output_directory / run_name
    data_directory = run_directory / "Data"
    log_directory = run_directory / "Logs"
    console_log = run_directory / "console.log"

    return run_directory, data_directory, log_directory, console_log


def run_monte_carlo_realization(
    base_config: dict,
    root: Path,
    realization: int,
    output_directory: Path,
    project_offset: int = 0,
) -> dict:
    config = copy.deepcopy(base_config)
    monte_carlo = validate_monte_carlo_config(config)
    seed = monte_carlo["random_seed"] + realization - 1

    timeout = int(monte_carlo.get("realization_timeout", 60))
    max_retries = int(monte_carlo.get("max_retries", 2))

    (
        run_directory,
        data_directory,
        log_directory,
        _,
    ) = monte_carlo_paths(output_directory, realization)

    data_directory.mkdir(parents=True, exist_ok=True)
    log_directory.mkdir(parents=True, exist_ok=True)

    config["monte_carlo"]["random_seed"] = seed
    config["output"]["data_directory"] = str(
        data_directory.relative_to(root)
    )
    config["output"]["log_directory"] = str(
        log_directory.relative_to(root)
    )

    project_name = f"manta-mc-{project_offset + realization:06d}"
    ros_domain_id = 10 + ((realization - 1) % 200)
    compose_path = (
        root
        / "generated"
        / output_directory.relative_to(root / "output")
        / f"compose_{realization:04d}.yaml"
    )

    write_runtime_compose(
        config,
        root,
        path=compose_path,
        project_name=project_name,
        ros_domain_id=ros_domain_id,
    )

    up_command = [
        "docker",
        "compose",
        "--project-name",
        project_name,
        "-f",
        str(compose_path),
        "up",
        "--remove-orphans",
    ]

    down_command = [
        "docker",
        "compose",
        "--project-name",
        project_name,
        "-f",
        str(compose_path),
        "down",
        "--remove-orphans",
    ]

    last_error = "unknown failure"
    vehicle_ids = [vehicle["id"] for vehicle in config["vehicles"]]

    for attempt in range(1, max_retries + 2):
        if stop_requested.is_set():
            raise InterruptedError(
                "Monte Carlo was stopped by the user."
            )

        console_log = run_directory / f"console_attempt_{attempt}.log"
        detections_path = data_directory / "whale_detections.csv"

        # Never allow a retry to accept a partial or stale result file.
        detections_path.unlink(missing_ok=True)

        for vehicle_id in vehicle_ids:
            (data_directory / f"{vehicle_id}_metrics.csv").unlink(
                missing_ok=True
            )
            (data_directory / f"{vehicle_id}_metrics.csv.tmp").unlink(
                missing_ok=True
            )

        print(
            f"Starting realization {realization}, attempt {attempt} "
            f"(seed {seed}, ROS domain {ros_domain_id})..."
        )

        try:
            with console_log.open("w", encoding="utf-8") as log:
                result = subprocess.run(
                    up_command,
                    cwd=root,
                    stdout=log,
                    stderr=subprocess.STDOUT,
                    timeout=timeout,
                    check=False,
                )

            if stop_requested.is_set():
                raise InterruptedError(
                    "Monte Carlo was stopped by the user."
                )

            if result.returncode != 0:
                raise ValueError(
                    f"Docker Compose exited with code {result.returncode}"
                )

            whale_count = int(config["scenario"]["whales"]["count"])
            detected_count, detection_fraction = summarize_realization(
                data_directory,
                whale_count,
            )
            (
                total_mass_kg,
                total_energy_wh,
                mean_vehicle_speed_mps,
            ) = summarize_vehicle_metrics(data_directory, vehicle_ids)

            print(
                f"Completed realization {realization}: "
                f"{detected_count}/{whale_count} whales "
                f"({detection_fraction:.1%})"
            )

            return {
                "realization": realization,
                "seed": seed,
                "detected_whales": detected_count,
                "whale_count": whale_count,
                "detection_fraction": detection_fraction,
                "total_mass_kg": total_mass_kg,
                "total_energy_wh": total_energy_wh,
                "mean_vehicle_speed_mps": mean_vehicle_speed_mps,
            }

        except subprocess.TimeoutExpired:
            last_error = f"timed out after {timeout} seconds"

        except ValueError as error:
            last_error = str(error)

        finally:
            try:
                subprocess.run(
                    down_command,
                    cwd=root,
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                    timeout=30,
                    check=False,
                )
            except subprocess.TimeoutExpired:
                last_error += "; Docker cleanup also timed out"

        if stop_requested.is_set():
            raise InterruptedError(
                "Monte Carlo was stopped by the user."
            )

        if attempt <= max_retries:
            print(
                f"Retrying realization {realization}: {last_error}. "
                f"See {console_log}",
                file=sys.stderr,
            )

    raise ValueError(
        f"Realization {realization} failed after {max_retries + 1} "
        f"attempts: {last_error}"
    )


def load_monte_carlo_summary(path: Path, base_seed: int) -> list[dict]:
    if not path.is_file():
        return []

    rows = []

    with path.open("r", encoding="utf-8", newline="") as file:
        reader = csv.DictReader(file)

        required = {
            "realization",
            "seed",
            "detected_whales",
            "whale_count",
            "detection_fraction",
            "total_mass_kg",
            "total_energy_wh",
            "mean_vehicle_speed_mps",
        }

        if not reader.fieldnames or not required.issubset(
            reader.fieldnames
        ):
            raise ValueError(
                f"Monte Carlo summary has an invalid header: {path}"
            )

        for row in reader:
            realization = int(row["realization"])
            seed = int(row["seed"])
            expected_seed = base_seed + realization - 1

            if seed != expected_seed:
                raise ValueError(
                    f"Existing realization {realization} uses seed {seed}, "
                    f"but the current configuration expects {expected_seed}."
                )

            rows.append(
                {
                    "realization": realization,
                    "seed": seed,
                    "detected_whales": int(row["detected_whales"]),
                    "whale_count": int(row["whale_count"]),
                    "detection_fraction": float(
                        row["detection_fraction"]
                    ),
                    "total_mass_kg": float(row["total_mass_kg"]),
                    "total_energy_wh": float(row["total_energy_wh"]),
                    "mean_vehicle_speed_mps": float(
                        row["mean_vehicle_speed_mps"]
                    ),
                }
            )

    rows.sort(key=lambda row: row["realization"])
    return rows


def write_monte_carlo_summary(path: Path, rows: list[dict]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)

    with path.open("w", encoding="utf-8", newline="") as file:
        writer = csv.DictWriter(
            file,
            fieldnames=(
                "realization",
                "seed",
                "detected_whales",
                "whale_count",
                "detection_fraction",
                "total_mass_kg",
                "total_energy_wh",
                "mean_vehicle_speed_mps",
            ),
        )
        writer.writeheader()

        for row in sorted(
            rows,
            key=lambda item: item["realization"],
        ):
            writer.writerow(row)


def monte_carlo_convergence(
    fractions: list[float],
    batch_size: int,
    min_realizations: int,
    confidence_level: float,
    target_half_width: float,
    stability_tolerance: float,
) -> tuple[bool, float, float, float]:
    """Calculate convergence using the current and preceding cumulative means."""

    if not fractions:
        return False, math.nan, math.inf, math.inf

    sample_mean, half_width = calculate_monte_carlo_statistics(
        fractions,
        confidence_level,
    )

    previous_count = len(fractions) - batch_size
    previous_mean = (
        mean(fractions[:previous_count])
        if previous_count > 0
        else None
    )
    mean_change = (
        abs(sample_mean - previous_mean)
        if previous_mean is not None
        else math.inf
    )

    converged = (
        len(fractions) >= min_realizations
        and half_width <= target_half_width
        and mean_change <= stability_tolerance
    )

    return converged, sample_mean, half_width, mean_change


def run_monte_carlo(
    config: dict,
    root: Path,
    output_directory: Path | None = None,
    project_offset: int = 0,
) -> int:
    stop_requested.clear()
    signal.signal(signal.SIGINT, handle_interrupt)
    signal.signal(signal.SIGTERM, handle_interrupt)

    monte_carlo = validate_monte_carlo_config(config)
    min_realizations = monte_carlo["min_realizations"]
    max_realizations = monte_carlo["max_realizations"]
    batch_size = monte_carlo["batch_size"]
    confidence_level = float(monte_carlo["confidence_level"])
    target_half_width = float(monte_carlo["target_half_width"])
    stability_tolerance = float(
        monte_carlo["stability_tolerance"]
    )
    base_seed = monte_carlo["random_seed"]
    recovery_cycles = int(monte_carlo.get("recovery_cycles", 3))

    if recovery_cycles < 0:
        raise ValueError(
            "'monte_carlo.recovery_cycles' cannot be negative."
        )

    cpu_count = os.cpu_count() or 1
    vehicle_count = len(config["vehicles"])
    default_workers = max(1, cpu_count // max(1, vehicle_count))
    workers = int(
        monte_carlo.get(
            "workers",
            min(batch_size, default_workers),
        )
    )
    # A realization contains one CPU-heavy Odyssey process per vehicle.
    # Reduce parallel realizations as the fleet grows to avoid oversubscribing
    # the machine and triggering Docker exit code 137.
    workers = max(
        1,
        min(workers, batch_size, default_workers, 200),
    )

    if output_directory is None:
        output_directory = root / "output" / "monte_carlo"
    summary_path = output_directory / "summary.csv"
    rows = load_monte_carlo_summary(summary_path, base_seed)

    completed = {
        row["realization"]: row
        for row in rows
    }

    if rows:
        print(
            f"Resuming Monte Carlo experiment with "
            f"{len(rows)} completed realizations."
        )

    print(
        f"Monte Carlo settings: minimum {min_realizations}, "
        f"maximum {max_realizations}, batch size {batch_size}, "
        f"workers {workers}."
    )

    fractions = [
        completed[index]["detection_fraction"]
        for index in sorted(completed)
    ]

    if fractions:
        converged, sample_mean, half_width, mean_change = (
            monte_carlo_convergence(
                fractions,
                batch_size,
                min_realizations,
                confidence_level,
                target_half_width,
                stability_tolerance,
            )
        )

        print(
            f"Current {len(fractions)} realizations: mean detection "
            f"{sample_mean:.2%}, CI half-width {half_width:.2%}, "
            f"mean change {mean_change:.2%}."
        )

        if converged:
            print(
                f"Saved results already satisfy convergence after "
                f"{len(fractions)} realizations."
            )
            print(f"Summary: {summary_path}")
            return 0

    next_realization = 1

    while next_realization in completed:
        next_realization += 1

    consecutive_recoveries = 0

    while len(completed) < max_realizations:
        # Stop precisely at the next batch-size checkpoint. For example,
        # resuming with 50 completed runs and batch_size=25 schedules only
        # runs 51-75; resuming with 74 schedules only run 75.
        remainder = len(completed) % batch_size
        runs_needed = batch_size if remainder == 0 else batch_size - remainder
        runs_needed = min(runs_needed, max_realizations - len(completed))

        batch = []
        candidate = next_realization

        while (
            len(batch) < runs_needed
            and candidate <= max_realizations
        ):
            if candidate not in completed:
                batch.append(candidate)
            candidate += 1

        if not batch:
            break

        print(
            f"Running {len(batch)} realization(s) to reach checkpoint "
            f"{len(completed) + len(batch)}."
        )

        failures = []
        with ThreadPoolExecutor(max_workers=workers) as executor:
            futures = {
                executor.submit(
                    run_monte_carlo_realization,
                    config,
                    root,
                    realization,
                    output_directory,
                    project_offset,
                ): realization
                for realization in batch
            }

            for future in as_completed(futures):
                realization = futures[future]

                try:
                    row = future.result()
                except InterruptedError:
                    continue
                except Exception as error:
                    failures.append(
                        f"Realization {realization} failed: {error}"
                    )
                    continue

                completed[realization] = row
                rows = [
                    completed[index]
                    for index in sorted(completed)
                ]
                write_monte_carlo_summary(summary_path, rows)

        if stop_requested.is_set():
            print("Monte Carlo stopped.")
            return 130

        if failures:
            consecutive_recoveries += 1

            print(
                "One or more realizations exhausted their local retries:",
                file=sys.stderr,
            )

            for failure in failures:
                print(f"  {failure}", file=sys.stderr)

            if consecutive_recoveries > recovery_cycles:
                raise ValueError(
                    "Monte Carlo exceeded the configured global recovery "
                    f"limit of {recovery_cycles}.\n"
                    + "\n".join(failures)
                )

            print(
                "Stopping all MANTA Monte Carlo Docker projects before "
                f"recovery {consecutive_recoveries}/{recovery_cycles}...",
                file=sys.stderr,
            )
            cleanup_monte_carlo_projects(root)

            # Successful results from this batch are already in summary.csv.
            # Begin again at the first missing realization so only failed or
            # unfinished samples are rerun with their original seeds.
            next_realization = 1

            while next_realization in completed:
                next_realization += 1

            continue

        consecutive_recoveries = 0

        fractions = [
            completed[index]["detection_fraction"]
            for index in sorted(completed)
        ]

        converged, sample_mean, half_width, mean_change = (
            monte_carlo_convergence(
                fractions,
                batch_size,
                min_realizations,
                confidence_level,
                target_half_width,
                stability_tolerance,
            )
        )

        confidence_percent = 100.0 * confidence_level

        print(
            f"After {len(fractions)} realizations: "
            f"mean detection {sample_mean:.2%}, "
            f"{confidence_percent:.1f}% CI half-width "
            f"{half_width:.2%}, "
            f"mean change {mean_change:.2%}."
        )

        if converged:
            print(
                f"Monte Carlo convergence reached after "
                f"{len(fractions)} realizations."
            )
            print(f"Summary: {summary_path}")
            return 0

        next_realization = 1

        while next_realization in completed:
            next_realization += 1

    fractions = [
        completed[index]["detection_fraction"]
        for index in sorted(completed)
    ]

    sample_mean, half_width = calculate_monte_carlo_statistics(
        fractions,
        confidence_level,
    )

    print(
        f"Reached maximum of {len(fractions)} realizations. "
        f"Mean detection: {sample_mean:.2%}; "
        f"CI half-width: {half_width:.2%}."
    )
    print(f"Summary: {summary_path}")

    return 0


def make_sweep_config(
    base_config: dict,
    vehicle_count: int,
) -> dict:
    """Create one fleet configuration by cloning the first vehicle plugin."""

    if not base_config.get("vehicles"):
        raise ValueError("The sweep requires at least one vehicle template.")

    config = copy.deepcopy(base_config)
    template = copy.deepcopy(base_config["vehicles"][0])
    vehicles = []

    for index in range(1, vehicle_count + 1):
        vehicle = copy.deepcopy(template)
        vehicle["id"] = f"odyssey_{index}"
        vehicle["mission"] = (
            f"./generated/missions/picket_line_odyssey_{index}.mi"
        )
        vehicles.append(vehicle)

    config["vehicles"] = vehicles
    return config


def write_sweep_summary(path: Path, rows: list[dict]) -> None:
    """Save fleet-level means and confidence intervals."""

    path.parent.mkdir(parents=True, exist_ok=True)

    with path.open("w", encoding="utf-8", newline="") as file:
        writer = csv.DictWriter(
            file,
            fieldnames=(
                "vehicle_count",
                "realizations",
                "whale_count",
                "mean_detected_whales",
                "mean_detection",
                "ci_lower",
                "ci_upper",
                "ci_half_width",
                "confidence_level",
                "mean_total_mass_kg",
                "mass_ci_lower_kg",
                "mass_ci_upper_kg",
                "mass_ci_half_width_kg",
                "mean_total_energy_wh",
                "energy_ci_lower_wh",
                "energy_ci_upper_wh",
                "energy_ci_half_width_wh",
                "mean_vehicle_speed_mps",
                "speed_ci_lower_mps",
                "speed_ci_upper_mps",
                "speed_ci_half_width_mps",
            ),
        )
        writer.writeheader()
        writer.writerows(sorted(rows, key=lambda row: row["vehicle_count"]))


def plot_sweep_results(root: Path, summary_path: Path, plot_path: Path) -> None:
    """Generate the saved fleet-size confidence-interval plot."""

    python = root / "review" / ".venv" / "bin" / "python"
    script = (
        root
        / "review"
        / "manta_review"
        / "plot_sweep_metrics.py"
    )

    if not python.is_file():
        raise ValueError(f"Review Python is missing: {python}")

    if not script.is_file():
        raise ValueError(f"Sweep plotting script is missing: {script}")

    result = subprocess.run(
        [str(python), str(script), str(summary_path), str(plot_path)],
        cwd=root,
        check=False,
    )

    if result.returncode != 0:
        raise ValueError("Failed to plot Monte Carlo sweep results.")


def run_monte_carlo_sweep(
    base_config: dict,
    config_path: Path,
    root: Path,
    minimum_vehicles: int = 1,
    maximum_vehicles: int = 10,
) -> int:
    """Run adaptive Monte Carlo experiments for each fleet size."""

    sweep_directory = root / "output" / "monte_carlo_sweep"
    generated_directory = root / "generated" / "monte_carlo_sweep"
    summary_path = sweep_directory / "sweep_summary.csv"
    generated_directory.mkdir(parents=True, exist_ok=True)

    sweep_rows = []

    if summary_path.is_file():
        with summary_path.open("r", encoding="utf-8", newline="") as file:
            sweep_rows = list(csv.DictReader(file))

        for row in sweep_rows:
            row["vehicle_count"] = int(row["vehicle_count"])
            row["realizations"] = int(row["realizations"])

    completed_counts = {
        int(row["vehicle_count"]): row
        for row in sweep_rows
    }

    for vehicle_count in range(minimum_vehicles, maximum_vehicles + 1):
        if stop_requested.is_set():
            return 130

        print(
            f"\n=== Monte Carlo fleet sweep: "
            f"{vehicle_count} vehicle(s) ==="
        )

        config = make_sweep_config(base_config, vehicle_count)
        fleet_config_path = (
            generated_directory
            / f"simulation_vehicles_{vehicle_count:02d}.yaml"
        )
        fleet_config_path.write_text(
            yaml.safe_dump(config, sort_keys=False),
            encoding="utf-8",
        )

        generate_scenario_missions(
            config,
            fleet_config_path,
            root,
        )
        validate_config(config, root)

        fleet_directory = (
            sweep_directory
            / f"vehicles_{vehicle_count:02d}"
        )
        result = run_monte_carlo(
            config,
            root,
            output_directory=fleet_directory,
            project_offset=vehicle_count * 10000,
        )

        if result != 0:
            return result

        monte_carlo = validate_monte_carlo_config(config)
        rows = load_monte_carlo_summary(
            fleet_directory / "summary.csv",
            monte_carlo["random_seed"],
        )
        fractions = [row["detection_fraction"] for row in rows]
        masses = [row["total_mass_kg"] for row in rows]
        energies = [row["total_energy_wh"] for row in rows]
        speeds = [row["mean_vehicle_speed_mps"] for row in rows]
        whale_count = int(config["scenario"]["whales"]["count"])
        confidence_level = float(monte_carlo["confidence_level"])
        sample_mean, half_width = calculate_monte_carlo_statistics(
            fractions,
            confidence_level,
        )
        mean_mass, mass_half_width = calculate_monte_carlo_statistics(
            masses,
            confidence_level,
        )
        mean_energy, energy_half_width = calculate_monte_carlo_statistics(
            energies,
            confidence_level,
        )
        mean_speed, speed_half_width = calculate_monte_carlo_statistics(
            speeds,
            confidence_level,
        )

        completed_counts[vehicle_count] = {
            "vehicle_count": vehicle_count,
            "realizations": len(rows),
            "whale_count": whale_count,
            "mean_detected_whales": sample_mean * whale_count,
            "mean_detection": sample_mean,
            "ci_lower": max(0.0, sample_mean - half_width),
            "ci_upper": min(1.0, sample_mean + half_width),
            "ci_half_width": half_width,
            "confidence_level": monte_carlo["confidence_level"],
            "mean_total_mass_kg": mean_mass,
            "mass_ci_lower_kg": max(0.0, mean_mass - mass_half_width),
            "mass_ci_upper_kg": mean_mass + mass_half_width,
            "mass_ci_half_width_kg": mass_half_width,
            "mean_total_energy_wh": mean_energy,
            "energy_ci_lower_wh": max(
                0.0,
                mean_energy - energy_half_width,
            ),
            "energy_ci_upper_wh": mean_energy + energy_half_width,
            "energy_ci_half_width_wh": energy_half_width,
            "mean_vehicle_speed_mps": mean_speed,
            "speed_ci_lower_mps": max(
                0.0,
                mean_speed - speed_half_width,
            ),
            "speed_ci_upper_mps": mean_speed + speed_half_width,
            "speed_ci_half_width_mps": speed_half_width,
        }
        sweep_rows = list(completed_counts.values())
        write_sweep_summary(summary_path, sweep_rows)

        print(
            f"Fleet size {vehicle_count}: mean {sample_mean:.2%}, "
            f"CI half-width {half_width:.2%} from {len(rows)} runs."
        )

    print(f"Sweep data: {summary_path}")
    print("Run './manta review sweep' to generate the sweep plots.")
    return 0


def parse_args(root: Path) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Build, run, review, and analyze MANTA simulations."
    )

    parser.add_argument(
        "command",
        nargs="?",
        default="generate",
        choices=(
            "generate",
            "build",
            "dev",
            "run",
            "down",
            "review",
            "monte-carlo",
            "monte-carlo-sweep",
        ),
    )

    parser.add_argument(
        "plot",
        nargs="?",
        default="tracks",
        choices=(
            "tracks",
            "energy",
            "sweep",
        ),
        help="Plot to show with the review command.",
    )

    parser.add_argument(
        "-c",
        "--config",
        default=str(root / "config" / "simulation.yaml"),
        help="Simulation configuration file.",
    )

    return parser.parse_args()


def run_review(root: Path, plot: str) -> int:
    scripts = {
        "tracks": "plot_tracks.py",
        "energy": "plot_energy.py",
        "sweep": "plot_sweep_metrics.py",
    }

    python = root / "review" / ".venv" / "bin" / "python"
    script = root / "review" / "manta_review" / scripts[plot]

    if not python.is_file():
        print(f"Review Python is missing: {python}", file=sys.stderr)
        return 1

    if not script.is_file():
        print(f"Review script is missing: {script}", file=sys.stderr)
        return 1

    command = [str(python), str(script)]

    if plot == "sweep":
        sweep_directory = root / "output" / "monte_carlo_sweep"
        summary_path = sweep_directory / "sweep_summary.csv"
        output_path = sweep_directory / "detection_vs_fleet_size.png"

        if not summary_path.is_file():
            print(
                f"Monte Carlo sweep summary is missing: {summary_path}",
                file=sys.stderr,
            )
            return 1

        command.extend(
            [str(summary_path), str(output_path), "--show"]
        )

    return subprocess.run(
        command,
        cwd=script.parent,
        check=False,
    ).returncode


def run_build(
    root: Path,
    env_path: Path,
) -> int:
    build_image = [
        "docker",
        "compose",
        "--env-file",
        str(env_path),
        "-f",
        str(root / "compose.dev.yaml"),
        "build",
    ]

    result = subprocess.run(
        build_image,
        cwd=root,
        check=False,
    )

    if result.returncode != 0:
        return result.returncode

    build_inside_container = [
        "docker",
        "compose",
        "--env-file",
        str(env_path),
        "-f",
        str(root / "compose.dev.yaml"),
        "run",
        "--rm",
        "dev",
        "/bin/bash",
        "-lc",
        "source /opt/ros/jazzy/setup.bash && "
        "cd /odyssey/vehicles/odyssey && "
        "make generate_odyssey && "
        "make libraries && "
        "cd /odyssey/ros_ws && "
        "colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release",
    ]

    return subprocess.run(
        build_inside_container,
        cwd=root,
        check=False,
    ).returncode


def main() -> int:
    root = Path(__file__).resolve().parent.parent
    args = parse_args(root)

    if args.command == "review":
        return run_review(root, args.plot)

    try:
        config_path = Path(args.config).resolve()
        config = load_config(config_path)

        if args.command == "down":
            compose_path = root / "generated" / "compose.ros.yaml"

            if not compose_path.is_file():
                print("No generated MANTA Compose file exists.")
                return 0

            return subprocess.run(
                [
                    "docker",
                    "compose",
                    "-f",
                    str(compose_path),
                    "down",
                    "--remove-orphans",
                ],
                cwd=root,
                check=False,
            ).returncode

        if args.command == "monte-carlo-sweep":
            validate_config(config, root)
            print_summary(config, config_path)

            monte_carlo = validate_monte_carlo_config(config)
            sweep = config.get("sweep", {})

            if not isinstance(sweep, dict):
                raise ValueError(
                    "'sweep' must be a YAML mapping."
                )

            minimum_vehicles = sweep.get("min_vehicles", 1)
            maximum_vehicles = sweep.get("max_vehicles", 10)

            if (
                not isinstance(minimum_vehicles, int)
                or minimum_vehicles < 1
            ):
                raise ValueError(
                    "'sweep.min_vehicles' "
                    "must be an integer of at least 1."
                )

            if (
                not isinstance(maximum_vehicles, int)
                or maximum_vehicles < minimum_vehicles
            ):
                raise ValueError(
                    "'sweep.max_vehicles' must be "
                    "an integer greater than or equal to min_vehicles."
                )

            return run_monte_carlo_sweep(
                config,
                config_path,
                root,
                minimum_vehicles=minimum_vehicles,
                maximum_vehicles=maximum_vehicles,
            )

        generate_scenario_missions(config, config_path, root)
        validate_config(config, root)
        print_summary(config, config_path)

        env_path = write_runtime_env(config, root)
        compose_path = write_runtime_compose(config, root)

        print(f"Generated environment: {env_path}")
        print(f"Generated Compose file: {compose_path}")

        if args.command == "generate":
            return 0

        if args.command == "build":
            return run_build(root, env_path)

        if args.command == "monte-carlo":
            return run_monte_carlo(config, root)

        commands = {
            "dev": [
                "docker",
                "compose",
                "--env-file",
                str(env_path),
                "-f",
                str(root / "compose.dev.yaml"),
                "run",
                "--rm",
                "dev",
            ],
            "run": [
                "docker",
                "compose",
                "-f",
                str(compose_path),
                "up",
                "--remove-orphans",
            ],
        }

        return subprocess.run(
            commands[args.command],
            cwd=root,
            check=False,
        ).returncode

    except ValueError as error:
        print(
            f"MANTA configuration error: {error}",
            file=sys.stderr,
        )
        return 1

    except FileNotFoundError as error:
        print(
            f"Required command was not found: {error.filename}",
            file=sys.stderr,
        )
        return 1

    except KeyboardInterrupt:
        print("\nMANTA command interrupted.", file=sys.stderr)
        return 130


if __name__ == "__main__":
    raise SystemExit(main())
