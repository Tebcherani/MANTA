#!/usr/bin/env python3

import argparse
import csv
import json
import numpy as np

from collections import defaultdict
from pathlib import Path
from parse_obd import read_obd


def read_whale_positions(path: Path) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Calculate the whale position at every logged simulation time."""

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

    whale_north = np.asarray([np.mean([position[0] for position in positions_by_time[time]]) for time in times], dtype=float)
    whale_east = np.asarray([np.mean([position[1] for position in positions_by_time[time]]) for time in times], dtype=float)

    return times, whale_north, whale_east

def compute_prediction_metrics(data_directory: Path, vehicle_id: str) -> dict[str, float]:
    """Compare logged whale predictions with whale truth at the same time."""

    whale_time, whale_north, whale_east = read_whale_positions(
        data_directory / "whales.csv"
    )
    prediction_path = data_directory / f"{vehicle_id}_whale_predictions.csv"

    if not prediction_path.is_file():
        raise ValueError(f"Prediction file does not exist: {prediction_path}")

    total_samples = 0
    errors_m = []

    with prediction_path.open("r", encoding="utf-8", newline="") as file:
        reader = csv.DictReader(file)
        required = {
            "sim_time",
            "has_estimate",
            "predicted_north",
            "predicted_east",
        }

        if not reader.fieldnames or not required.issubset(reader.fieldnames):
            raise ValueError(
                f"{prediction_path.name} must contain "
                "sim_time, has_estimate, predicted_north, and predicted_east."
            )

        for row in reader:
            total_samples += 1

            try:
                sim_time = float(row["sim_time"])
                has_estimate = int(row["has_estimate"])
            except (TypeError, ValueError):
                continue

            if has_estimate != 1 or not np.isfinite(sim_time):
                continue

            if sim_time < whale_time[0] or sim_time > whale_time[-1]:
                continue

            try:
                predicted_north = float(row["predicted_north"])
                predicted_east = float(row["predicted_east"])
            except (TypeError, ValueError):
                continue

            if not np.isfinite(predicted_north) or not np.isfinite(predicted_east):
                continue

            true_north = np.interp(sim_time, whale_time, whale_north)
            true_east = np.interp(sim_time, whale_time, whale_east)

            errors_m.append(
                np.hypot(predicted_north - true_north, predicted_east - true_east)
            )

    coverage = float(len(errors_m) / total_samples) if total_samples else 0.0

    if not errors_m:
        return {
            "mean_prediction_error_km": None,
            "rmse_prediction_error_km": None,
            "prediction_coverage_fraction": 0.0,
        }

    errors_m = np.asarray(errors_m, dtype=float)

    return {
        "mean_prediction_error_km": float(np.mean(errors_m) / 1000.0),
        "rmse_prediction_error_km": float(np.sqrt(np.mean(errors_m**2)) / 1000.0),
        "prediction_coverage_fraction": coverage,
    }


def main() -> int:
    """Calculate one vehicle's whale-prediction metrics."""

    parser = argparse.ArgumentParser()
    parser.add_argument("data_directory", type=Path)
    parser.add_argument("vehicle_id")
    args = parser.parse_args()

    metrics = compute_prediction_metrics(
        args.data_directory.resolve(),
        args.vehicle_id,
    )
    print(json.dumps(metrics))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())