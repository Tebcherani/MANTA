#!/usr/bin/env python3
"""Sweep the trailing mission's call-update interval."""

import argparse
import copy
import csv
import sys
from dataclasses import asdict, replace
from pathlib import Path
from statistics import mean

import yaml

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "components" / "manta_core" / "scripts"))

from manta_dataclasses import MonteCarloConfig, MonteCarloResult, TrailingSweepConfig, TrailingSweepResult
from manta_load import load_yaml
from manta_missions import generate_missions
from manta_monte_carlo import compute_monte_carlo_statistics, load_completed_results, run_monte_carlo
from manta_realization import summarize_prediction_metrics


def interval_directory_name(interval: float) -> str:
    """Create a stable directory name for one call-update interval."""
    interval = float(interval)

    if interval.is_integer():
        return f"interval_{int(interval):06d}"

    return f"interval_{interval:g}".replace(".", "_")


def load_completed_trailing_sweep_results(summary_path: Path) -> dict:
    """Load completed results indexed by interval and vehicle."""
    if not summary_path.is_file():
        return {}

    with summary_path.open("r", encoding="utf-8", newline="") as file:
        reader = csv.DictReader(file)

        if "vehicle_id" not in (reader.fieldnames or []):
            raise ValueError("Existing sweep summary has no vehicle IDs. Move output/trailing/sweep_comms aside and rerun.")

        results = {}
        for row in reader:
            result = TrailingSweepResult.from_csv_row(row)
            results[(result.call_update_interval, row["vehicle_id"])] = result

    return results


def prepare_interval_config(sim_config: dict, interval: float, interval_directory: Path, mission_directory: Path) -> tuple[dict, Path]:
    """Create the configuration and mission paths for one interval."""
    interval_config = copy.deepcopy(sim_config)
    interval_config["scenario"]["trailing"]["communication_interval"] = interval

    for vehicle in interval_config["vehicles"]:
        mission_path = mission_directory / f"{vehicle['id']}.mi"
        vehicle["mission"] = f"./{mission_path.relative_to(ROOT).as_posix()}"

    interval_directory.mkdir(parents=True, exist_ok=True)
    interval_config_path = interval_directory / "simulation.yaml"
    interval_config_path.write_text(yaml.safe_dump(interval_config, sort_keys=False), encoding="utf-8")

    return interval_config, interval_config_path


def summarize_interval_results(
    interval: float,
    results: list[MonteCarloResult],
    confidence_level: float,
) -> TrailingSweepResult:
    """Summarize prediction error and coverage across realizations."""

    if not results:
        raise ValueError(f"Interval {interval:g} s has no Monte Carlo results.")

    errors = [
        result.mean_prediction_error_km
        for result in results
        if result.mean_prediction_error_km is not None
    ]

    coverage = mean(
        result.prediction_coverage_fraction or 0.0
        for result in results
    )

    if errors:
        mean_error, half_width = compute_monte_carlo_statistics(
            errors, confidence_level
        )
        lower = max(0.0, mean_error - half_width)
        upper = mean_error + half_width
    else:
        mean_error = lower = upper = half_width = None

    return TrailingSweepResult(
        call_update_interval=interval,
        realizations=len(results),
        prediction_realizations=len(errors),
        prediction_coverage_fraction=coverage,
        mean_prediction_error_km=mean_error,
        prediction_error_ci_lower_km=lower,
        prediction_error_ci_upper_km=upper,
        prediction_error_ci_half_width_km=half_width,
        confidence_level=confidence_level,
    )


def write_trailing_sweep_summary(path: Path, results: dict) -> None:
    """Save one summary row per interval and vehicle."""
    path.parent.mkdir(parents=True, exist_ok=True)

    fieldnames = (
        "vehicle_id",
        "call_update_interval",
        "realizations",
        "mean_prediction_error_km",
        "prediction_error_ci_lower_km",
        "prediction_error_ci_upper_km",
        "prediction_error_ci_half_width_km",
        "confidence_level",
        "prediction_realizations",
        "prediction_coverage_fraction",
    )

    with path.open("w", encoding="utf-8", newline="") as file:
        writer = csv.DictWriter(file, fieldnames=fieldnames)
        writer.writeheader()

        for interval, vehicle_id in sorted(results):
            writer.writerow({"vehicle_id": vehicle_id, **asdict(results[(interval, vehicle_id)])})


def run_monte_carlo_sweep(sim_config: dict) -> int:
    """Run a Monte Carlo experiment for each configured update interval."""
    if sim_config.get("scenario", {}).get("type") != "trailing":
        raise ValueError("The call-update interval sweep requires the trailing scenario.")

    sweep_config = TrailingSweepConfig.from_dict(sim_config.get("sweep", {}))
    monte_carlo_config = MonteCarloConfig.from_dict(sim_config.get("monte_carlo", {}))

    sweep_output_dir = ROOT / "output" / "trailing" / "sweep_comms"
    sweep_output_dir.mkdir(parents=True, exist_ok=True)

    # Prevent results from different configurations being mixed on resume.
    snapshot_path = sweep_output_dir / "sweep_config.yaml"

    if snapshot_path.is_file():
        if load_yaml(snapshot_path) != sim_config:
            raise ValueError(f"Configuration changed. Move {sweep_output_dir} aside before starting a new experiment.")
    else:
        # Existing results without a snapshot cannot be verified.
        if any(sweep_output_dir.iterdir()):
            raise ValueError(f"Existing sweep data has no configuration snapshot. Move {sweep_output_dir} aside before starting.")

        snapshot_path.write_text(yaml.safe_dump(sim_config, sort_keys=False), encoding="utf-8")

    summary_path = sweep_output_dir / "sweep_summary.csv"
    completed_results = load_completed_trailing_sweep_results(summary_path)

    for interval_index, interval in enumerate(sweep_config.call_update_intervals):
        interval = float(interval)
        print(f"\n=== Trailing Monte Carlo sweep: {interval:g} s call-update interval ===")

        vehicle_ids = [vehicle["id"] for vehicle in sim_config["vehicles"]]

        if all((interval, vehicle_id) in completed_results for vehicle_id in vehicle_ids):
            print(f"Skipping {interval:g} s; sweep results already exist.")
            continue

        # Keep generated missions and realization outputs within this sweep.
        interval_name = interval_directory_name(interval)
        interval_directory = sweep_output_dir / interval_name
        mission_directory = sweep_output_dir / "missions" / interval_name

        interval_config, interval_config_path = prepare_interval_config(
            sim_config, interval, interval_directory, mission_directory
        )
        generate_missions(interval_config, interval_config_path, mission_directory)

        project_offset = (interval_index + 1) * 10000
        status = run_monte_carlo(interval_config, interval_directory, project_offset)

        if status != 0:
            return status

        # Aggregate the completed realizations and save the sweep checkpoint.
        realization_results = list(
            load_completed_results(interval_directory / "summary.csv", monte_carlo_config.random_seed).values()
        )
        for vehicle_id in vehicle_ids:
            vehicle_results = []

            for result in realization_results:
                data_directory = (interval_directory / f"realization_{result.realization:04d}" / "Data")
                prediction_error, coverage = summarize_prediction_metrics(
                    ROOT, data_directory, vehicle_id
                )
                vehicle_results.append(
                    replace(
                        result,
                        mean_prediction_error_km=prediction_error,
                        prediction_coverage_fraction=coverage,
                    )
                )
            
            sweep_result = summarize_interval_results(interval, vehicle_results, monte_carlo_config.confidence_level)
            completed_results[(interval, vehicle_id)] = sweep_result

            error_text = (
                f"{sweep_result.mean_prediction_error_km:.3f} km"
                if sweep_result.mean_prediction_error_km is not None
                else "undefined (no predictions)"
            )
            print(
                f"{vehicle_id}, interval {interval:g} s: "
                f"mean prediction error {error_text}; "
                f"coverage {sweep_result.prediction_coverage_fraction:.1%}; "
                f"{sweep_result.prediction_realizations}/"
                f"{sweep_result.realizations} realizations had predictions."
            )

        write_trailing_sweep_summary(summary_path, completed_results)

    print(f"Sweep data: {summary_path}")
    print("Run './trailing review sweep comms' to generate the sweep plots.")

    return 0


def main() -> int:
    """Load the supplied configuration and start the sweep."""
    parser = argparse.ArgumentParser(description="Sweep trailing call-update intervals.")
    parser.add_argument("config_path", type=Path)
    args = parser.parse_args()

    try:
        config = load_yaml(args.config_path.resolve())
        return run_monte_carlo_sweep(config)

    except (ValueError, KeyError, OSError) as error:
        print(f"Trailing communications sweep error: {error}", file=sys.stderr)
        return 1

    except KeyboardInterrupt:
        print("\nTrailing communications sweep interrupted.", file=sys.stderr)
        return 130


if __name__ == "__main__":
    raise SystemExit(main())