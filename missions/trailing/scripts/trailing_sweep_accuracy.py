"""Sweep call-location standard deviation using a YAML-defined kilometre range.

Run from MANTA: python3 missions/trailing/sweep_call_accuracy.py CONFIG
YAML: accuracy: {min_std_km: 0, max_std_km: 10, step_std_km: 1}
Values describe Gaussian standard deviation per horizontal axis, not bounds.
"""
import argparse
import copy
import csv
import math
import sys
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


def validated_values(config):
    """Build an inclusive, evenly spaced range in metres without float drift."""
    settings = config.get("sweep", {}).get("accuracy")
    if not isinstance(settings, dict):
        raise ValueError("sweep.accuracy must be a YAML mapping.")
    values = {}
    for key in ("min_std_km", "max_std_km", "step_std_km"):
        raw = settings.get(key)
        if raw is None or isinstance(raw, bool):
            raise ValueError(f"sweep.accuracy.{key} must be a finite number.")
        try:
            value = Decimal(str(raw))
        except InvalidOperation as error:
            raise ValueError(f"sweep.accuracy.{key} must be a finite number.") from error
        if not value.is_finite():
            raise ValueError(f"sweep.accuracy.{key} must be a finite number.")
        values[key] = value
    start, stop, step = (values[key] for key in ("min_std_km", "max_std_km", "step_std_km"))
    if start < 0 or stop < start or step <= 0:
        raise ValueError("Require 0 <= min_std_km <= max_std_km and step_std_km > 0.")
    steps = (stop - start) / step
    if steps != steps.to_integral_value():
        raise ValueError("max_std_km must be reachable from min_std_km in whole steps.")
    result = tuple(float((start + index * step) * 1000) for index in range(int(steps) + 1))
    if any(not math.isfinite(value) for value in result) or len(set(result)) != len(result):
        raise ValueError("Range cannot be represented as distinct finite metre values.")
    return result


def summarize_accuracy_results(std_m, interval, results, confidence_level):
    """Aggregate true-trajectory tracking metrics for one uncertainty level."""
    if not results:
        raise ValueError(f"Position standard deviation {std_m:g} m has no results.")
    metrics = {}
    for attribute, label in (("mean_prediction_error_km", "centroid distance")):
        samples = [getattr(result, attribute) for result in results]
        if any(value is None or not math.isfinite(value) or value < 0 for value in samples):
            raise ValueError(f"Position standard deviation {std_m:g} m has invalid {label} results.")
        metrics[attribute] = compute_monte_carlo_statistics(samples, confidence_level)
    distance, distance_half_width = metrics["mean_prediction_error_km"]
    return {
        "call_position_std_km": std_m / 1000.0,
        "call_position_std_m": std_m,
        "call_update_interval": interval,
        "realizations": len(results),
        "mean_prediction_error_km": distance,
        "prediction_error_ci_lower_km": max(0.0, distance - distance_half_width),
        "prediction_error_ci_upper_km": distance + distance_half_width,
        "prediction_error_ci_half_width_km": distance_half_width,
        "confidence_level": confidence_level,
    }


def write_summary(path, rows):
    if not rows:
        return
    temporary = path.with_suffix(".csv.tmp")
    with temporary.open("w", encoding="utf-8", newline="") as file:
        writer = csv.DictWriter(file, fieldnames=list(next(iter(rows.values()))))
        writer.writeheader()
        for value in sorted(rows):
            writer.writerow(rows[value])
    temporary.replace(path)


def run_monte_carlo_sweep(sim_config):
    if sim_config.get("scenario", {}).get("type") != "trailing":
        raise ValueError("Accuracy sweep requires the trailing scenario.")
    if len(sim_config.get("vehicles", [])) != 1:
        raise ValueError("Accuracy sweep requires exactly one vehicle.")
    values = validated_values(sim_config)
    mc = MonteCarloConfig.from_dict(sim_config.get("monte_carlo", {}))
    interval = float(sim_config["scenario"]["trailing"].get("call_update_interval", 0.0))
    if not math.isfinite(interval) or interval < 0:
        raise ValueError("call_update_interval must be finite and nonnegative.")
    output = ROOT / "output" / "trailing" / "sweep_accuracy"
    output.mkdir(parents=True, exist_ok=True)
    # Refuse to silently reuse results from another experiment configuration.
    snapshot = output / "sweep_config.yaml"
    encoded = yaml.safe_dump(sim_config, sort_keys=True)
    if snapshot.exists() and snapshot.read_text(encoding="utf-8") != encoded:
        raise ValueError(f"Configuration changed. Move {output} aside before starting a new experiment.")
    snapshot.write_text(encoded, encoding="utf-8")
    summary = output / "sweep_summary.csv"
    rows = {}
    if summary.exists():
        with summary.open(encoding="utf-8", newline="") as file:
            rows = {float(row["call_position_std_m"]): row for row in csv.DictReader(file)}
    for index, std_m in enumerate(values):
        print(f"\nAccuracy sweep: sigma={std_m:g} m per axis; updates={interval:g} s")
        if std_m in rows:
            print("Skipping completed accuracy level.")
            continue
        name = "std_m_" + repr(std_m).replace(".", "_").replace("+", "")
        directory = output / name
        missions = output / "missions" / name
        config = copy.deepcopy(sim_config)
        config["scenario"]["trailing"]["call_position_std_m"] = std_m
        for vehicle in config["vehicles"]:
            vehicle["mission"] = f"./{(missions / (vehicle['id'] + '.mi')).relative_to(ROOT)}"
        directory.mkdir(parents=True, exist_ok=True)
        config_path = directory / "simulation.yaml"
        config_path.write_text(yaml.safe_dump(config, sort_keys=False), encoding="utf-8")
        generate_missions(config, config_path, missions)
        status = run_monte_carlo(config, directory, (index + 1) * 10000)
        if status != 0:
            return status
        results = list(load_completed_results(directory / "summary.csv", mc.random_seed).values())
        row = summarize_accuracy_results(std_m, interval, results, mc.confidence_level)
        rows[std_m] = row
        write_summary(summary, rows)
        print(f"Mean formation error: {row['mean_prediction_error_km']:.3f} km")
    print(f"Accuracy sweep data: {summary}")
    return 0


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("config", type=Path)
    args = parser.parse_args()
    with args.config.open(encoding="utf-8") as file:
        config = yaml.safe_load(file)
    if not isinstance(config, dict):
        raise ValueError("Simulation YAML must contain a mapping.")
    raise SystemExit(run_monte_carlo_sweep(config))
