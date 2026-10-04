#!/usr/bin/env python3

import csv
import math
import matplotlib.pyplot as plt

from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]
SUMMARY_PATH = ROOT / "output" / "trailing" / "monte_carlo_sweep" / "sweep_summary.csv"


def read_sweep_summary() -> list[dict[str, float]]:
    """Read and validate the trailing call-update sweep summary."""

    if not SUMMARY_PATH.is_file():
        raise ValueError(f"Trailing sweep summary does not exist: {SUMMARY_PATH}")

    required = {
        "call_update_interval",
        "realizations",
        "mean_centroid_distance_km",
        "centroid_distance_ci_lower_km",
        "centroid_distance_ci_upper_km",
        "mean_integrated_tracking_error_km_hours",
        "integrated_error_ci_lower_km_hours",
        "integrated_error_ci_upper_km_hours",
    }

    rows = []

    with SUMMARY_PATH.open("r", encoding="utf-8", newline="") as file:
        reader = csv.DictReader(file)

        if not reader.fieldnames or not required.issubset(reader.fieldnames):
            missing = sorted(required.difference(reader.fieldnames or []))
            raise ValueError(f"Trailing sweep summary is missing columns: {', '.join(missing)}")

        for line_number, source in enumerate(reader, start=2):
            try:
                row = {
                    "call_update_interval": float(source["call_update_interval"]),
                    "realizations": int(source["realizations"]),
                    "mean_centroid_distance_km": float(source["mean_centroid_distance_km"]),
                    "centroid_distance_ci_lower_km": float(source["centroid_distance_ci_lower_km"]),
                    "centroid_distance_ci_upper_km": float(source["centroid_distance_ci_upper_km"]),
                    "mean_integrated_tracking_error_km_hours": float(source["mean_integrated_tracking_error_km_hours"]),
                    "integrated_error_ci_lower_km_hours": float(source["integrated_error_ci_lower_km_hours"]),
                    "integrated_error_ci_upper_km_hours": float(source["integrated_error_ci_upper_km_hours"]),
                }
            except (TypeError, ValueError) as error:
                raise ValueError(f"Invalid value on line {line_number} of {SUMMARY_PATH}.") from error

            if row["call_update_interval"] < 0.0:
                raise ValueError(f"Negative call-update interval on line {line_number}.")
            if row["realizations"] < 1:
                raise ValueError(f"Invalid realization count on line {line_number}.")
            if not all(math.isfinite(value) for name, value in row.items() if name != "realizations"):
                raise ValueError(f"Non-finite sweep value on line {line_number}.")

            rows.append(row)

    if not rows:
        raise ValueError(f"Trailing sweep summary contains no results: {SUMMARY_PATH}")

    rows.sort(key=lambda row: row["call_update_interval"])

    return rows


def plot_metric(ax, intervals: list[float], means: list[float], lower: list[float], upper: list[float], color: str, title: str, ylabel: str) -> None:
    """Plot one trailing metric with confidence-interval error bars."""

    lower_error = [mean - bound for mean, bound in zip(means, lower)]
    upper_error = [bound - mean for mean, bound in zip(means, upper)]

    ax.errorbar(intervals, means, yerr=[lower_error, upper_error], fmt="o", color=color, markersize=7, elinewidth=1.8, capsize=5, capthick=1.5)
    ax.set_title(title)
    ax.set_xlabel("Call waypoint update interval (hours)")
    ax.set_ylabel(ylabel)
    ax.grid(True, alpha=0.3)


def main() -> int:
    """Plot tracking performance against whale-call update interval."""

    try:
        rows = read_sweep_summary()
    except (OSError, ValueError) as error:
        print(f"Could not plot trailing sweep: {error}")
        return 1

    intervals_hours = [row["call_update_interval"] / 3600.0 for row in rows]
    tick_labels = ["Every call" if row["call_update_interval"] == 0.0 else f"{row['call_update_interval'] / 3600.0:g}" for row in rows]

    distance_means = [row["mean_centroid_distance_km"] for row in rows]
    distance_lower = [row["centroid_distance_ci_lower_km"] for row in rows]
    distance_upper = [row["centroid_distance_ci_upper_km"] for row in rows]

    fig, ax = plt.subplots(figsize=(10, 6), constrained_layout=True)
    fig.suptitle("Effect of Whale-Call Update Interval on AUV Group Tracking", fontsize=14)

    plot_metric(
        ax,
        intervals_hours,
        distance_means,
        distance_lower,
        distance_upper,
        "tab:blue",
        "AUV Distance from Whale-Group Centroid",
        "Mean centroid distance (km)",
    )

    ax.set_xlabel("Call waypoint update interval (h)")
    ax.set_xticks(intervals_hours)
    ax.set_xticklabels(tick_labels, rotation=45, ha="right")
    ax.set_ylim(bottom=0.0)

    mission_duration_hours = 24.0

    integrated_axis = ax.secondary_yaxis(
        "right",
        functions=(
            lambda distance: distance * mission_duration_hours,
            lambda integrated_error: integrated_error / mission_duration_hours,
        ),
    )

    integrated_axis.set_ylabel("Integrated centroid tracking error (km·h)")

    plt.show()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())