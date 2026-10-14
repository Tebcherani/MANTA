#!/usr/bin/env python3
"""Plot per-realization fleet-average formation errors by communication interval."""

import csv
import math
from pathlib import Path

import matplotlib.pyplot as plt
import yaml


ROOT = Path(__file__).resolve().parents[3]
SWEEP_DIR = ROOT / "output" / "trailing" / "sweep_comms"
OUTPUT_PATH = SWEEP_DIR / "sweep_metrics_boxplot.png"


def read_interval_results(interval_directory: Path) -> tuple[float, list[float]]:
    """Read an interval's communication period and per-realization errors."""
    config_path = interval_directory / "simulation.yaml"
    summary_path = interval_directory / "summary.csv"

    if not config_path.is_file():
        raise ValueError(f"Interval config does not exist: {config_path}")
    if not summary_path.is_file():
        raise ValueError(f"Realization summary does not exist: {summary_path}")

    with config_path.open("r", encoding="utf-8") as file:
        config = yaml.safe_load(file)

    try:
        interval = float(config["scenario"]["trailing"]["communication_interval"])
    except (KeyError, TypeError, ValueError) as error:
        raise ValueError(f"Invalid communication interval in {config_path}") from error

    if not math.isfinite(interval) or interval < 0:
        raise ValueError(f"Invalid communication interval in {config_path}")

    errors = []
    seen_realizations = set()

    with summary_path.open("r", encoding="utf-8", newline="") as file:
        reader = csv.DictReader(file)
        required = {"realization", "mean_distance_km"}

        if not reader.fieldnames or not required.issubset(reader.fieldnames):
            raise ValueError(
                f"{summary_path} must contain realization and mean_distance_km columns."
            )

        for line_number, row in enumerate(reader, start=2):
            try:
                realization = int(row["realization"])
                error_km = float(row["mean_distance_km"])
            except (TypeError, ValueError) as error:
                raise ValueError(
                    f"Invalid value on line {line_number} of {summary_path}"
                ) from error

            if realization in seen_realizations:
                raise ValueError(
                    f"Duplicate realization {realization} in {summary_path}"
                )
            seen_realizations.add(realization)

            if not math.isfinite(error_km) or error_km < 0:
                raise ValueError(
                    f"Invalid formation error on line {line_number} of {summary_path}"
                )

            errors.append(error_km)

    if not errors:
        raise ValueError(f"No realization errors found in {summary_path}")

    return interval, errors


def main() -> int:
    """Read interval summaries and save a box plot."""
    if not SWEEP_DIR.is_dir():
        print(f"Sweep output directory does not exist: {SWEEP_DIR}")
        return 1

    interval_directories = [
        path
        for path in SWEEP_DIR.glob("interval_*")
        if path.is_dir()
        and (path / "simulation.yaml").is_file()
        and (path / "summary.csv").is_file()
    ]

    if not interval_directories:
        print(f"No completed interval summaries found in {SWEEP_DIR}")
        return 1

    try:
        interval_results = [
            read_interval_results(path)
            for path in interval_directories
        ]
    except (OSError, ValueError, yaml.YAMLError) as error:
        print(f"Could not plot trailing sweep: {error}")
        return 1

    interval_results.sort(key=lambda result: result[0])
    intervals = [result[0] for result in interval_results]
    errors_by_interval = [result[1] for result in interval_results]

    fig, ax = plt.subplots(figsize=(12, 6), constrained_layout=True)

    ax.boxplot(
        errors_by_interval,
        showfliers=True,
        patch_artist=True,
        boxprops={"facecolor": "lightsteelblue", "edgecolor": "steelblue"},
        medianprops={"color": "darkred", "linewidth": 1.5},
        whiskerprops={"color": "steelblue"},
        capprops={"color": "steelblue"},
        flierprops={
            "marker": ".",
            "markerfacecolor": "darkorange",
            "markeredgecolor": "darkorange",
            "alpha": 0.5,
        },
    )

    ax.set_xticks(range(1, len(intervals) + 1))
    ax.set_xticklabels(
        [f"{interval / 60:g}" for interval in intervals],
        rotation=45,
        ha="right",
    )
    ax.set_xlabel("Communication Interval (min)")
    ax.set_ylabel("Per-Realization Fleet-Average Formation Error (km)")
    ax.set_title("Formation Error Distribution by Communication Interval")
    ax.grid(axis="y", alpha=0.3)

    OUTPUT_PATH.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(OUTPUT_PATH, dpi=200, bbox_inches="tight")
    plt.close(fig)

    print(f"Saved: {OUTPUT_PATH}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())