#!/usr/bin/env python3

import argparse
import csv
import json
import numpy as np

from collections import defaultdict
from pathlib import Path
from missions.trailing.review.parse_obd import read_obd


def read_whale_centroids(path: Path) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Calculate the whale-group centroid at every logged simulation time."""

    positions_by_time = defaultdict(list)

    if not path.is_file():
        raise ValueError(f"Whale trajectory file does not exist: {path}")

    with path.open("r", encoding="utf-8", newline="") as file:
        reader = csv.DictReader(file)
        required = {"sim_time", "north", "east"}

        if not reader.fieldnames or not required.issubset(reader.fieldnames):
            raise ValueError("whales.csv must contain sim_time, north, and east columns.")

        for row in reader:
            try:
                sim_time = float(row["sim_time"])
                north = float(row["north"])
                east = float(row["east"])
            except (TypeError, ValueError):
                continue

            if not np.isfinite(sim_time) or not np.isfinite(north) or not np.isfinite(east):
                continue

            positions_by_time[sim_time].append((north, east))

    times = np.asarray(sorted(positions_by_time), dtype=float)

    if times.size < 2:
        raise ValueError("Whale trajectory data contain insufficient time samples.")

    centroid_north = np.asarray([np.mean([position[0] for position in positions_by_time[time]]) for time in times], dtype=float)
    centroid_east = np.asarray([np.mean([position[1] for position in positions_by_time[time]]) for time in times], dtype=float)

    return times, centroid_north, centroid_east


def compute_tracking_metrics(data_directory: Path, vehicle_id: str) -> dict[str, float]:
    """Calculate centroid-distance metrics for one trailing realization."""

    whale_path = data_directory / "whales.csv"
    vehicle_path = data_directory / f"{vehicle_id}.obd"

    if not vehicle_path.is_file():
        raise ValueError(f"Vehicle result does not exist: {vehicle_path}")

    dataset = read_obd(vehicle_path)

    if "time" not in dataset.coords:
        raise ValueError(f"Vehicle {vehicle_id} has no simulation-time coordinate.")

    if "real_x" in dataset and "real_y" in dataset:
        vehicle_north = np.asarray(dataset["real_x"].values, dtype=float)
        vehicle_east = np.asarray(dataset["real_y"].values, dtype=float)
    elif "m_pos_n" in dataset and "m_pos_e" in dataset:
        vehicle_north = np.asarray(dataset["m_pos_n"].values, dtype=float)
        vehicle_east = np.asarray(dataset["m_pos_e"].values, dtype=float)
    else:
        raise ValueError(f"Vehicle {vehicle_id} has no position data.")

    vehicle_time = np.asarray(dataset.coords["time"].values, dtype=float)
    valid_vehicle = np.isfinite(vehicle_time) & np.isfinite(vehicle_north) & np.isfinite(vehicle_east)
    vehicle_time = vehicle_time[valid_vehicle]
    vehicle_north = vehicle_north[valid_vehicle]
    vehicle_east = vehicle_east[valid_vehicle]

    if vehicle_time.size < 2:
        raise ValueError(f"Vehicle {vehicle_id} has insufficient position samples.")

    centroid_time, centroid_north, centroid_east = read_whale_centroids(whale_path)
    overlap = (centroid_time >= vehicle_time[0]) & (centroid_time <= vehicle_time[-1])
    centroid_time = centroid_time[overlap]
    centroid_north = centroid_north[overlap]
    centroid_east = centroid_east[overlap]

    if centroid_time.size < 2:
        raise ValueError("Vehicle and whale trajectories have insufficient overlapping times.")

    interpolated_north = np.interp(centroid_time, vehicle_time, vehicle_north)
    interpolated_east = np.interp(centroid_time, vehicle_time, vehicle_east)
    centroid_distance = np.hypot(interpolated_north - centroid_north, interpolated_east - centroid_east)

    integrated_error_m_seconds = float(np.trapezoid(centroid_distance, centroid_time))
    duration_seconds = float(centroid_time[-1] - centroid_time[0])

    return {
        "mean_centroid_distance_km": integrated_error_m_seconds / duration_seconds / 1000.0,
        "integrated_tracking_error_km_hours": integrated_error_m_seconds / 3_600_000.0,
    }


def main() -> int:
    """Calculate and print one trailing realization's tracking metrics."""

    parser = argparse.ArgumentParser()
    parser.add_argument("data_directory", type=Path)
    parser.add_argument("vehicle_id")
    args = parser.parse_args()

    metrics = compute_tracking_metrics(args.data_directory.resolve(), args.vehicle_id)
    print(json.dumps(metrics))

    return 0


if __name__ == "__main__":
    raise SystemExit(main())