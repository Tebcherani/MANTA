#!/usr/bin/env python3

import copy
import csv
import math
import os
import signal
import subprocess
import sys
import threading
import yaml

from manta_core.manta_dataclasses import MonteCarloConfig
from manta_core.manta_config import load_plugin

from concurrent.futures import ThreadPoolExecutor, as_completed
from pathlib import Path
from statistics import NormalDist, mean, stdev


stop_requested = threading.Event()


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


def container_path(root: Path, configured_path: str) -> str:
    host_path = (root / configured_path).resolve()

    try:
        relative_path = host_path.relative_to(root.resolve())
    except ValueError as error:
        raise ValueError(f"Path must remain inside MANTA: {host_path}") from error

    return f"/odyssey/{relative_path.as_posix()}"


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
    monte_carlo = MonteCarloConfig.from_dict(config.get("monte_carlo", {}))
    seed = monte_carlo.random_seed + realization - 1

    timeout = int(monte_carlo.realization_timeout)
    max_retries = int(monte_carlo.max_retries)

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


def write_sweep_summary(path: Path, rows: list[dict]) -> None:
    """Save fleet-level means and confidence intervals."""

    path.parent.mkdir(parents=True, exist_ok=True)

    with path.open("w", encoding="utf-8", newline="") as file:
        writer = csv.DictWriter(
            file,
            fieldnames=(
                "fleet_size",
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
        writer.writerows(sorted(rows, key=lambda row: row["fleet_size"]))


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