#!/usr/bin/env python3

import copy
import csv
import math
import subprocess
import sys
import threading

from pathlib import Path
from statistics import mean

from manta_core.manta_dataclasses import MonteCarloConfig, MonteCarloResult
from manta_core.manta_run import write_runtime_compose


stop_requested = threading.Event()


def check_stop_requested() -> None:
    """Raise an interruption when the user has requested Monte Carlo shutdown."""

    if stop_requested.is_set(): raise InterruptedError("Monte Carlo was stopped by the user.")


def collect_realization_result(config: dict, data_directory: Path, vehicle_ids: list[str], realization: int, seed: int) -> MonteCarloResult:
    """Read simulation outputs and construct the completed realization result."""

    whale_count = int(config["scenario"]["whales"]["count"])
    detected_count, detection_fraction = summarize_realization(data_directory, whale_count)
    total_mass, total_energy, mean_vehicle_speed = summarize_vehicle_metrics(data_directory, vehicle_ids)

    return MonteCarloResult(
        realization=realization,
        seed=seed,
        detected_whales=detected_count,
        whale_count=whale_count,
        detection_fraction=detection_fraction,
        total_mass=total_mass,
        total_energy=total_energy,
        mean_vehicle_speed=mean_vehicle_speed,
    )


def create_docker_commands(compose_path: Path, project_name: str) -> tuple[list[str], list[str]]:
    """Create the Docker Compose startup and shutdown commands."""

    base_command = ["docker", "compose", "--project-name", project_name, "-f", str(compose_path)]
    up_command = base_command + ["up", "--remove-orphans"]
    down_command = base_command + ["down", "--remove-orphans"]

    return up_command, down_command


def get_realization_paths(root: Path, output_directory: Path, realization: int) -> tuple[Path, Path, Path, Path]:
    """Create and return the data, log, console, and Compose paths for one realization."""

    run_directory = output_directory / f"realization_{realization:04d}"
    data_directory = run_directory / "Data"
    log_directory = run_directory / "Logs"
    compose_path = root / "generated" / output_directory.relative_to(root / "output") / f"compose_{realization:04d}.yaml"

    data_directory.mkdir(parents=True, exist_ok=True)
    log_directory.mkdir(parents=True, exist_ok=True)
    compose_path.parent.mkdir(parents=True, exist_ok=True)

    return run_directory, data_directory, log_directory, compose_path


def prepare_realization_config(base_config: dict, root: Path, data_directory: Path, log_directory: Path, seed: int) -> dict:
    """Create a realization-specific simulation configuration."""

    config = copy.deepcopy(base_config)
    config["monte_carlo"]["random_seed"] = seed
    config["output"]["data_directory"] = str(data_directory.relative_to(root))
    config["output"]["log_directory"] = str(log_directory.relative_to(root))

    return config


def prepare_realization(base_config: dict, root: Path, realization: int, output_directory: Path, project_offset: int) -> tuple[dict, MonteCarloConfig, int, Path, Path, list[str], list[str], list[str], int]:
    """Prepare the configuration, paths, and Docker commands for one realization."""

    monte_carlo = MonteCarloConfig.from_dict(base_config.get("monte_carlo", {}))
    seed = monte_carlo.random_seed + realization - 1
    run_directory, data_directory, log_directory, compose_path = get_realization_paths(root, output_directory, realization)
    config = prepare_realization_config(base_config, root, data_directory, log_directory, seed)
    vehicle_ids = [vehicle["id"] for vehicle in config["vehicles"]]
    project_name = f"manta-mc-{project_offset + realization:06d}"
    ros_domain_id = 10 + ((realization - 1) % 200)

    write_runtime_compose(config, compose_path=compose_path, project_name=project_name, ros_domain_id=ros_domain_id)

    up_command, down_command = create_docker_commands(compose_path, project_name)

    return config, monte_carlo, seed, run_directory, data_directory, vehicle_ids, up_command, down_command, ros_domain_id


def remove_stale_results(data_directory: Path, vehicle_ids: list[str]) -> None:
    """Remove partial results left by a previous realization attempt."""

    (data_directory / "whale_detections.csv").unlink(missing_ok=True)

    for vehicle_id in vehicle_ids:
        (data_directory / f"{vehicle_id}_metrics.csv").unlink(missing_ok=True)
        (data_directory / f"{vehicle_id}_metrics.csv.tmp").unlink(missing_ok=True)


def run_compose_attempt(up_command: list[str], root: Path, console_log: Path, timeout: int) -> None:
    """Run Docker Compose once and raise an error if it fails or times out."""

    with console_log.open("w", encoding="utf-8") as log:
        result = subprocess.run(up_command, cwd=root, stdout=log, stderr=subprocess.STDOUT, timeout=timeout, check=False)

    if result.returncode != 0: raise ValueError(f"Docker Compose exited with code {result.returncode}")


def stop_compose_project(down_command: list[str], root: Path) -> str | None:
    """Stop a realization's Docker Compose project and report cleanup failure."""

    try:
        subprocess.run(down_command, cwd=root, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=30, check=False)
    except subprocess.TimeoutExpired:
        return "Docker cleanup timed out"

    return None


def summarize_realization(data_directory: Path, whale_count: int) -> tuple[int, float]:
    """Count the unique whales detected during one realization."""

    detections_path = data_directory / "whale_detections.csv"

    if not detections_path.is_file(): raise ValueError(f"Detection results were not created: {detections_path}")

    with detections_path.open("r", encoding="utf-8", newline="") as file:
        reader = csv.DictReader(file)

        if not reader.fieldnames or "whale_id" not in reader.fieldnames: raise ValueError(f"Detection results do not contain a whale_id column: {detections_path}")

        detected_whales = {row["whale_id"].strip() for row in reader if row.get("whale_id", "").strip()}

    detected_count = len(detected_whales)

    return detected_count, detected_count / whale_count


def summarize_vehicle_metrics(data_directory: Path, vehicle_ids: list[str]) -> tuple[float, float, float]:
    """Combine the final metrics written by every vehicle in one realization."""

    total_mass = 0.0
    total_energy = 0.0
    vehicle_speeds = []
    required_fields = {"vehicle_id", "mass", "energy_consumed", "mean_speed"}

    for vehicle_id in vehicle_ids:
        metrics_path = data_directory / f"{vehicle_id}_metrics.csv"

        if not metrics_path.is_file(): raise ValueError(f"Vehicle metrics were not created: {metrics_path}")

        with metrics_path.open("r", encoding="utf-8", newline="") as file:
            reader = csv.DictReader(file)

            if not reader.fieldnames or not required_fields.issubset(reader.fieldnames): raise ValueError(f"Vehicle metrics have an invalid header: {metrics_path}")

            row = next(reader, None)

        if row is None: raise ValueError(f"Vehicle metrics contain no data row: {metrics_path}")
        if row["vehicle_id"] != vehicle_id: raise ValueError(f"Expected metrics for '{vehicle_id}', but {metrics_path} contains '{row['vehicle_id']}'.")

        try:
            mass = float(row["mass"])
            energy = float(row["energy_consumed"])
            speed = float(row["mean_speed"])
        except (TypeError, ValueError) as error:
            raise ValueError(f"Vehicle metrics contain invalid numbers: {metrics_path}") from error

        if not all(math.isfinite(value) for value in (mass, energy, speed)): raise ValueError(f"Vehicle metrics contain non-finite values: {metrics_path}")
        if mass <= 0.0: raise ValueError(f"Vehicle mass must be positive: {metrics_path}")
        if energy < 0.0 or speed < 0.0: raise ValueError(f"Vehicle energy and speed cannot be negative: {metrics_path}")

        total_mass += mass
        total_energy += energy
        vehicle_speeds.append(speed)

    if not vehicle_speeds: raise ValueError("A realization must contain at least one vehicle metric.")

    return total_mass, total_energy, mean(vehicle_speeds)


#--------------------------------------------------
# Main realization code
#--------------------------------------------------

def run_realization_attempt(config: dict, root: Path, realization: int, attempt: int, seed: int, ros_domain_id: int, run_directory: Path, data_directory: Path, vehicle_ids: list[str], up_command: list[str], down_command: list[str], timeout: int) -> MonteCarloResult:
    """Run one attempt and always stop its Docker Compose project afterward."""

    check_stop_requested()
    remove_stale_results(data_directory, vehicle_ids)

    console_log = run_directory / f"console_attempt_{attempt}.log"
    print(f"Starting realization {realization}, attempt {attempt} (seed {seed}, ROS domain {ros_domain_id})...")

    try:
        run_compose_attempt(up_command, root, console_log, timeout)
        check_stop_requested()
        result = collect_realization_result(config, data_directory, vehicle_ids, realization, seed)
        print(f"Completed realization {realization}: {result.detected_whales}/{result.whale_count} whales ({result.detection_fraction:.1%})")
        return result
    finally:
        cleanup_error = stop_compose_project(down_command, root)
        if cleanup_error: print(f"Realization {realization}: {cleanup_error}.", file=sys.stderr)


def run_monte_carlo_realization(base_config: dict, root: Path, realization: int, output_directory: Path, project_offset: int = 0) -> MonteCarloResult:
    """Run one Monte Carlo realization, retrying failed attempts as configured."""

    config, monte_carlo, seed, run_dir, data_dir, vehicle_ids, up_command, down_command, ros_domain_id = prepare_realization(base_config, root, realization, output_directory, project_offset)
    last_error = "unknown failure"

    for attempt in range(1, monte_carlo.max_retries + 2):
        try:
            return run_realization_attempt(config, root, realization, attempt, seed, ros_domain_id, run_dir, data_dir, vehicle_ids, up_command, down_command, monte_carlo.realization_timeout)
        except InterruptedError:
            raise
        except subprocess.TimeoutExpired:
            last_error = f"timed out after {monte_carlo.realization_timeout} seconds"
        except ValueError as error:
            last_error = str(error)

        check_stop_requested()

        if attempt <= monte_carlo.max_retries:
            console_log = run_dir / f"console_attempt_{attempt}.log"
            print(f"Retrying realization {realization}: {last_error}. See {console_log}", file=sys.stderr)

    raise ValueError(f"Realization {realization} failed after {monte_carlo.max_retries + 1} attempts: {last_error}")