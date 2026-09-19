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

from manta_core.manta_config import load_plugin
from manta_core.manta_dataclasses import MonteCarloConfig

from concurrent.futures import ThreadPoolExecutor, as_completed
from pathlib import Path
from statistics import NormalDist, mean, stdev


stop_requested = threading.Event()


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

    total_mass = 0.0
    total_energy = 0.0
    vehicle_speeds = []
    required = {
        "vehicle_id",
        "mass",
        "energy_consumed",
        "mean_speed",
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
            mass = float(row["mass"])
            energy = float(row["energy_consumed"])
            speed = float(row["mean_speed"])
        except (TypeError, ValueError) as error:
            raise ValueError(
                f"Vehicle metrics contain invalid numbers: {path}"
            ) from error

        values = (mass, energy, speed)

        if not all(math.isfinite(value) for value in values):
            raise ValueError(
                f"Vehicle metrics contain non-finite values: {path}"
            )

        if mass <= 0.0:
            raise ValueError(f"Vehicle mass must be positive: {path}")

        if energy < 0.0 or speed < 0.0:
            raise ValueError(
                f"Vehicle energy and speed cannot be negative: {path}"
            )

        total_mass += mass
        total_energy += energy
        vehicle_speeds.append(speed)

    if not vehicle_speeds:
        raise ValueError(
            "A realization must contain at least one vehicle metric."
        )

    return total_mass, total_energy, mean(vehicle_speeds)


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
                total_mass,
                total_energy,
                mean_vehicle_speed,
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
                "total_mass": total_mass,
                "total_energy": total_energy,
                "mean_vehicle_speed": mean_vehicle_speed,
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