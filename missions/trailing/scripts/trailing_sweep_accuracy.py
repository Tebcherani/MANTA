#!/usr/bin/env python3
"""Sweep Gaussian bearing measurement standard deviation in degrees.
Configure the inclusive range in degrees under sweep.accuracy:
    min_std_degrees: 0
    max_std_degrees: 10
    step_std_degrees: 1
Save separate offset-corrected tracking results for each vehicle.
"""
import argparse
import copy
import csv
import math
import sys
from dataclasses import replace
from decimal import Decimal, InvalidOperation
from pathlib import Path
import yaml
ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "components" / "manta_core" / "scripts"))
from manta_dataclasses import MonteCarloConfig
from manta_missions import generate_missions
from manta_monte_carlo import (
    compute_monte_carlo_statistics,
    load_completed_results,
    run_monte_carlo,
)
from manta_realization import summarize_tracking_metrics

SUMMARY_FIELDS = (
    "vehicle_id",
    "bearing_std_degrees",
    "call_update_interval",
    "realizations",
    "mean_distance_km",
    "distance_ci_lower_km",
    "distance_ci_upper_km",
    "distance_ci_half_width_km",
    "confidence_level",
)

def validated_values(config):
    """Build an inclusive, evenly spaced range in degrees."""
    settings = config.get("sweep", {}).get("accuracy")
    if not isinstance(settings, dict):
        raise ValueError("sweep.accuracy must be a YAML mapping.")
    values = {}
    for key in ("min_std_degrees", "max_std_degrees", "step_std_degrees"):
        raw = settings.get(key)
        if raw is None or isinstance(raw, bool):
            raise ValueError(f"sweep.accuracy.{key} must be a finite number.")
        try:
            value = Decimal(str(raw))
        except InvalidOperation as error:
            raise ValueError(
                f"sweep.accuracy.{key} must be a finite number."
            ) from error
        if not value.is_finite():
            raise ValueError(f"sweep.accuracy.{key} must be a finite number.")
        values[key] = value
    start = values["min_std_degrees"]
    stop = values["max_std_degrees"]
    step = values["step_std_degrees"]
    if start < 0 or stop < start or step <= 0:
        raise ValueError(
            "Require 0 <= min_std_degrees <= max_std_degrees and step_std_degrees > 0."
        )
    steps = (stop - start) / step
    if steps != steps.to_integral_value():
        raise ValueError(
            "max_std_degrees must be reachable from min_std_degrees in whole steps."
        )
    result = tuple(
        float(start + index * step)
        for index in range(int(steps) + 1)
    )
    if (
        any(not math.isfinite(value) for value in result)
        or len(set(result)) != len(result)
    ):
        raise ValueError(
            "Range cannot be represented as distinct finite degree values."
        )
    return result

def summarize_accuracy_results(
    vehicle_id,
    std_degrees,
    interval,
    results,
    confidence_level,
):
    """Aggregate one vehicle's formation error across realizations."""
    if not results:
        raise ValueError(
            f"{vehicle_id}, bearing standard deviation {std_degrees:g} degrees "
            "has no results."
        )
    samples = [result.mean_distance_km for result in results]
    if any(
        value is None or not math.isfinite(value) or value < 0
        for value in samples
    ):
        raise ValueError(
            f"{vehicle_id}, bearing standard deviation {std_degrees:g} degrees "
            "has invalid formation errors."
        )
    mean_distance, half_width = compute_monte_carlo_statistics(
        samples,
        confidence_level,
    )
    return {
        "vehicle_id": vehicle_id,
        "bearing_std_degrees": std_degrees,
        "call_update_interval": interval,
        "realizations": len(results),
        "mean_distance_km": mean_distance,
        "distance_ci_lower_km": max(0.0, mean_distance - half_width),
        "distance_ci_upper_km": mean_distance + half_width,
        "distance_ci_half_width_km": half_width,
        "confidence_level": confidence_level,
    }

def load_completed_accuracy_results(path):
    """Load summary rows indexed by uncertainty and vehicle."""
    if not path.is_file():
        return {}
    with path.open("r", encoding="utf-8", newline="") as file:
        reader = csv.DictReader(file)
        fields = set(reader.fieldnames or [])
        if "vehicle_id" not in fields:
            raise ValueError(
                f"Existing accuracy summary has no vehicle IDs: {path}. "
                "Move the old sweep_accuracy directory aside and rerun. "
                "Position-noise simulations cannot be reused for bearing noise."
            )
        missing = set(SUMMARY_FIELDS) - fields
        if missing:
            raise ValueError(
                f"Accuracy summary is missing columns: {', '.join(sorted(missing))}"
            )
        rows = {}
        for row in reader:
            key = (
                float(row["bearing_std_degrees"]),
                row["vehicle_id"],
            )
            if key in rows:
                raise ValueError(
                    f"Duplicate accuracy summary row for {key}: {path}"
                )
            rows[key] = row
    return rows

def write_summary(path, rows):
    """Atomically save one row per uncertainty level and vehicle."""
    if not rows:
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(".csv.tmp")
    with temporary.open("w", encoding="utf-8", newline="") as file:
        writer = csv.DictWriter(file, fieldnames=SUMMARY_FIELDS)
        writer.writeheader()
        for key in sorted(rows):
            writer.writerow(rows[key])
    temporary.replace(path)

def run_monte_carlo_sweep(sim_config):
    """Run accuracy levels and summarize each vehicle separately."""
    if sim_config.get("scenario", {}).get("type") != "trailing":
        raise ValueError("Accuracy sweep requires the trailing scenario.")
    vehicles = sim_config.get("vehicles", [])
    if len(vehicles) != 2:
        raise ValueError("Accuracy sweep requires exactly two vehicles.")
    vehicle_ids = [vehicle["id"] for vehicle in vehicles]
    if len(set(vehicle_ids)) != len(vehicle_ids):
        raise ValueError("Vehicle IDs must be unique.")
    values = validated_values(sim_config)
    mc = MonteCarloConfig.from_dict(sim_config.get("monte_carlo", {}))
    interval = float(
        sim_config["scenario"]["trailing"]["communication_interval"]
    )
    if not math.isfinite(interval) or interval < 0:
        raise ValueError(
            "communication_interval must be finite and nonnegative."
        )
    output = ROOT / "output" / "trailing" / "sweep_accuracy"
    output.mkdir(parents=True, exist_ok=True)
    # Prevent reuse of simulations from a different configuration.
    snapshot = output / "sweep_config.yaml"
    if snapshot.is_file():
        with snapshot.open("r", encoding="utf-8") as file:
            saved_config = yaml.safe_load(file)
        if saved_config != sim_config:
            raise ValueError(
                f"Configuration changed. Move {output} aside "
                "before starting a new experiment."
            )
    else:
        if any(output.iterdir()):
            raise ValueError(
                f"Existing accuracy data has no configuration snapshot. "
                f"Move {output} aside before starting a new experiment."
            )
        snapshot.write_text(
            yaml.safe_dump(sim_config, sort_keys=False),
            encoding="utf-8",
        )
    summary = output / "sweep_summary.csv"
    rows = load_completed_accuracy_results(summary)
    for index, std_degrees in enumerate(values):
        print(
            f"\n=== Accuracy sweep: bearing sigma={std_degrees:g} degrees; "
            f"communication interval={interval:g} s ==="
        )
        if all(
            (std_degrees, vehicle_id) in rows
            for vehicle_id in vehicle_ids
        ):
            print("Skipping completed accuracy level.")
            continue
        name = (
            "bearing_std_deg_"
            + repr(std_degrees).replace(".", "_").replace("+", "")
        )
        directory = output / name
        missions = output / "missions" / name
        config = copy.deepcopy(sim_config)
        config["scenario"]["detection"]["bearing_std_degrees"] = std_degrees
        for vehicle in config["vehicles"]:
            mission_path = missions / f"{vehicle['id']}.mi"
            vehicle["mission"] = (
                f"./{mission_path.relative_to(ROOT).as_posix()}"
            )
        directory.mkdir(parents=True, exist_ok=True)
        config_path = directory / "simulation.yaml"
        config_path.write_text(
            yaml.safe_dump(config, sort_keys=False),
            encoding="utf-8",
        )
        generate_missions(config, config_path, missions)
        status = run_monte_carlo(
            config,
            directory,
            (index + 1) * 10000,
        )
        if status != 0:
            return status
        results = list(
            load_completed_results(
                directory / "summary.csv",
                mc.random_seed,
            ).values()
        )
        for vehicle_id in vehicle_ids:
            vehicle_results = []
            for result in results:
                data_directory = (
                    directory
                    / f"realization_{result.realization:04d}"
                    / "Data"
                )
                mean_distance = summarize_tracking_metrics(
                    ROOT,
                    data_directory,
                    vehicle_id,
                    config,
                )
                vehicle_results.append(
                    replace(
                        result,
                        mean_distance_km=mean_distance,
                    )
                )
            row = summarize_accuracy_results(
                vehicle_id,
                std_degrees,
                interval,
                vehicle_results,
                mc.confidence_level,
            )
            rows[(std_degrees, vehicle_id)] = row
            print(
                f"{vehicle_id}: mean formation error "
                f"{row['mean_distance_km']:.3f} km "
                f"from {row['realizations']} realizations."
            )
        write_summary(summary, rows)
    print(f"\nAccuracy sweep data: {summary}")
    print("Run './trailing review sweep accuracy' to generate the plot.")
    return 0

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("config", type=Path)
    args = parser.parse_args()
    try:
        with args.config.open("r", encoding="utf-8") as file:
            config = yaml.safe_load(file)
        if not isinstance(config, dict):
            raise ValueError("Simulation YAML must contain a mapping.")
        return run_monte_carlo_sweep(config)
    except (ValueError, KeyError, OSError, yaml.YAMLError) as error:
        print(f"Trailing accuracy sweep error: {error}", file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        print("\nTrailing accuracy sweep interrupted.", file=sys.stderr)
        return 130

if __name__ == "__main__":
    raise SystemExit(main())