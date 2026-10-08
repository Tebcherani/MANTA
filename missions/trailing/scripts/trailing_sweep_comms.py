import argparse
import copy
import csv
import sys
from dataclasses import asdict
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "components" / "manta_core" / "scripts"))

from manta_dataclasses import MonteCarloConfig, MonteCarloResult, TrailingSweepConfig, TrailingSweepResult
from manta_load import load_yaml
from manta_missions import generate_missions
from manta_monte_carlo import compute_monte_carlo_statistics, load_completed_results, run_monte_carlo


def interval_directory_name(interval: float) -> str:
    """Create a stable directory name for one call-update interval."""

    if interval.is_integer():
        return f"interval_{int(interval):06d}"

    return f"interval_{interval:g}".replace(".", "_")


def load_completed_trailing_sweep_results(summary_path: Path) -> dict[float, TrailingSweepResult]:
    """Load completed call-update interval results."""

    if not summary_path.is_file():
        return {}

    with summary_path.open("r", encoding="utf-8", newline="") as file:
        reader = csv.DictReader(file)
        results = [TrailingSweepResult.from_csv_row(row) for row in reader]

    return {result.call_update_interval: result for result in results}


def prepare_interval_config(sim_config: dict, interval: float, interval_directory: Path, mission_directory: Path) -> tuple[dict, Path]:
    """Create and save one trailing call-update interval configuration."""

    interval_config = copy.deepcopy(sim_config)
    interval_config["scenario"]["trailing"]["call_update_interval"] = interval

    for vehicle in interval_config["vehicles"]:
        vehicle["mission"] = f"./{(mission_directory / f'{vehicle['id']}.mi').relative_to(ROOT)}"

    interval_directory.mkdir(parents=True, exist_ok=True)
    interval_config_path = interval_directory / "simulation.yaml"
    interval_config_path.write_text(yaml.safe_dump(interval_config, sort_keys=False), encoding="utf-8")

    return interval_config, interval_config_path


def summarize_interval_results(interval: float, results: list[MonteCarloResult], confidence_level: float) -> TrailingSweepResult:
    """Aggregate tracking metrics for one call-update interval."""

    if not results:
        raise ValueError(f"Call-update interval {interval:g} s has no Monte Carlo results.")

    if any(result.mean_distance_km is None for result in results):
        raise ValueError(f"Call-update interval {interval:g} s contains missing distance results.")

    if any(result.integrated_tracking_error_km_hours is None for result in results):
        raise ValueError(f"Call-update interval {interval:g} s contains missing integrated-error results.")

    distances = [result.mean_distance_km for result in results]
    integrated_errors = [result.integrated_tracking_error_km_hours for result in results]

    mean_distance, distance_half_width = compute_monte_carlo_statistics(distances, confidence_level)
    mean_integrated_error, integrated_half_width = compute_monte_carlo_statistics(integrated_errors, confidence_level)

    return TrailingSweepResult(
        call_update_interval=interval,
        realizations=len(results),
        mean_distance_km=mean_distance,
        distance_ci_lower_km=max(0.0, mean_distance - distance_half_width),
        distance_ci_upper_km=mean_distance + distance_half_width,
        distance_ci_half_width_km=distance_half_width,
        mean_integrated_tracking_error_km_hours=mean_integrated_error,
        integrated_error_ci_lower_km_hours=max(0.0, mean_integrated_error - integrated_half_width),
        integrated_error_ci_upper_km_hours=mean_integrated_error + integrated_half_width,
        integrated_error_ci_half_width_km_hours=integrated_half_width,
        confidence_level=confidence_level,
    )


def write_trailing_sweep_summary(path: Path, results: dict[float, TrailingSweepResult]) -> None:
    """Write the call-update sweep summary."""

    path.parent.mkdir(parents=True, exist_ok=True)

    with path.open("w", encoding="utf-8", newline="") as file:
        writer = csv.DictWriter(
            file,
            fieldnames=(
                "call_update_interval",
                "realizations",
                "mean_distance_km",
                "distance_ci_lower_km",
                "distance_ci_upper_km",
                "distance_ci_half_width_km",
                "mean_integrated_tracking_error_km_hours",
                "integrated_error_ci_lower_km_hours",
                "integrated_error_ci_upper_km_hours",
                "integrated_error_ci_half_width_km_hours",
                "confidence_level",
            ),
        )
        writer.writeheader()

        for interval in sorted(results):
            writer.writerow(asdict(results[interval]))


def run_monte_carlo_sweep(sim_config: dict) -> int:
    """Run one Monte Carlo experiment for every trailing call-update interval."""

    if sim_config.get("scenario", {}).get("type") != "trailing":
        raise ValueError("The call-update interval sweep requires the trailing scenario.")

    vehicles = sim_config.get("vehicles", [])

    if len(vehicles) != 1:
        raise ValueError("The trailing call-update interval sweep requires exactly one vehicle.")

    sweep_config = TrailingSweepConfig.from_dict(sim_config.get("sweep", {}))
    monte_carlo_config = MonteCarloConfig.from_dict(sim_config.get("monte_carlo", {}))
    sweep_output_dir = ROOT / "output" / "trailing" / "sweep_comms"
    summary_path = sweep_output_dir / "sweep_summary.csv"
    completed_results = load_completed_trailing_sweep_results(summary_path)

    sweep_output_dir.mkdir(parents=True, exist_ok=True)

    for interval_index, interval in enumerate(sweep_config.call_update_intervals):
        print(f"\n=== Trailing Monte Carlo sweep: {interval:g} s call-update interval ===")

        if interval in completed_results:
            print(f"Skipping {interval:g} s; sweep results already exist.")
            continue

        interval_name = interval_directory_name(interval)
        interval_directory = sweep_output_dir / interval_name
        mission_directory = sweep_output_dir / "missions" / interval_name
        interval_config, interval_config_path = prepare_interval_config(sim_config, interval, interval_directory, mission_directory)
        generate_missions(interval_config, interval_config_path, mission_directory)
        project_offset = (interval_index + 1) * 10000
        status = run_monte_carlo(interval_config, interval_directory, project_offset)

        if status != 0:
            return status

        realization_results = list(load_completed_results(interval_directory / "summary.csv", monte_carlo_config.random_seed).values())
        sweep_result = summarize_interval_results(interval, realization_results, monte_carlo_config.confidence_level)
        completed_results[interval] = sweep_result
        write_trailing_sweep_summary(summary_path, completed_results)

        print(
            f"Interval {interval:g} s: "
            f"mean distance {sweep_result.mean_distance_km:.3f} km, "
            f"integrated error {sweep_result.mean_integrated_tracking_error_km_hours:.3f} km·h "
            f"from {sweep_result.realizations} realizations."
        )

    print(f"Sweep data: {summary_path}")
    print("Run './manta review trailing sweep' to generate the sweep plots.")

    return 0