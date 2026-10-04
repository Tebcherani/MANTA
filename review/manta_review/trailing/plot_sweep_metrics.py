#!/usr/bin/env python3

import csv
import math
import matplotlib.pyplot as plt

from matplotlib.ticker import PercentFormatter
from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]
SUMMARY_PATH = ROOT / "output" / "trailing" / "monte_carlo_sweep" / "sweep_summary.csv"
OUTPUT_PATH = ROOT / "output" / "trailing" / "monte_carlo_sweep" / "sweep_metrics.png"


METRICS = (
    {   # Detection plot
        "mean": "mean_detection",
        "lower": "ci_lower",
        "upper": "ci_upper",
        "title": "Whale Detection",
        "ylabel": "Unique whales detected (%)",
        "color": "tab:blue",
        "percent": True,
    },
    {   # Fleet mass plot
        "mean": "mean_total_mass",
        "lower": "mass_ci_lower",
        "upper": "mass_ci_upper",
        "title": "Total Fleet Mass",
        "ylabel": "Mass (kg)",
        "color": "tab:green",
        "percent": False,
    },
    {   # Energy plot
        "mean": "mean_total_energy",
        "lower": "energy_ci_lower",
        "upper": "energy_ci_upper",
        "title": "Total Fleet Energy Consumption",
        "ylabel": "Energy consumed (Wh)",
        "color": "tab:orange",
        "percent": False,
    },
    {   # Vehicle speed plot
        "mean": "mean_vehicle_speed",
        "lower": "speed_ci_lower",
        "upper": "speed_ci_upper",
        "title": "Mean Vehicle Speed",
        "ylabel": "Speed (m/s)",
        "color": "tab:red",
        "percent": False,
    },
)


def get_required_summary_columns() -> set[str]:
    """Return every sweep-summary CSV column required by the plots."""

    columns = {"fleet_size"}
    for metric in METRICS:
        columns.update((metric["mean"], metric["lower"], metric["upper"]))

    return columns


def plot_metric(ax, rows: list[dict[str, float]], metric: dict) -> None:
    """Plot one fleet metric with error bars and a shaded confidence interval."""

    fleet_sizes = [row["fleet_size"] for row in rows]
    means = [row[metric["mean"]] for row in rows]
    lower_bounds = [row[metric["lower"]] for row in rows]
    upper_bounds = [row[metric["upper"]] for row in rows]
    lower_errors = [mean_value - lower_value for mean_value, lower_value in zip(means, lower_bounds)]
    upper_errors = [upper_value - mean_value for mean_value, upper_value in zip(means, upper_bounds)]

    ax.errorbar(fleet_sizes, means, yerr=[lower_errors, upper_errors], fmt="o", color=metric["color"], markersize=6, elinewidth=2.0, capsize=6, capthick=2.0, label="Mean")

    if metric["percent"]:
        ax.yaxis.set_major_formatter(PercentFormatter(1.0))
    else:
        ax.ticklabel_format(axis="y", style="plain", useOffset=False)

    ax.set_title(metric["title"])
    ax.set_xlabel("Number of Odyssey vehicles")
    ax.set_ylabel(metric["ylabel"])
    ax.set_xticks(fleet_sizes)
    ax.grid(True, alpha=0.3)
    ax.legend()

    if metric["percent"]:
        ax.set_ylim(0.0, 1.0)
        ax.yaxis.set_major_formatter(PercentFormatter(1.0))


def read_summary() -> list[dict[str, float]]:
    """Read already-aggregated fleet statistics from sweep_summary.csv."""

    if not SUMMARY_PATH.is_file():
        raise ValueError(f"Sweep summary does not exist: {SUMMARY_PATH}")

    rows = []

    with SUMMARY_PATH.open("r", encoding="utf-8", newline="") as file:
        reader = csv.DictReader(file)
        if not reader.fieldnames:
            raise ValueError(f"Sweep summary has no header: {SUMMARY_PATH}")

        columns = get_required_summary_columns()
        missing = sorted(columns.difference(reader.fieldnames))
        if missing:
            raise ValueError(f"Sweep summary is missing metric columns: {', '.join(missing)}. Run the sweep with manta_runner_v2.py first.")

        for line_number, source in enumerate(reader, start=2):
            try:
                row = {"fleet_size": int(source["fleet_size"])}

                for metric in METRICS:
                    row[metric["mean"]] = float(source[metric["mean"]])
                    row[metric["lower"]] = float(source[metric["lower"]])
                    row[metric["upper"]] = float(source[metric["upper"]])
            except (TypeError, ValueError) as error:
                raise ValueError(f"Invalid numeric value on line {line_number} of {SUMMARY_PATH}.") from error

            values = [value for name, value in row.items() if name != "fleet_size"]

            if row["fleet_size"] < 1:
                raise ValueError(f"Vehicle count must be positive on line {line_number}.")

            if not all(math.isfinite(value) for value in values):
                raise ValueError(f"Non-finite metric on line {line_number} of {SUMMARY_PATH}.")

            for metric in METRICS:
                mean_value = row[metric["mean"]]
                lower_value = row[metric["lower"]]
                upper_value = row[metric["upper"]]

                if lower_value > mean_value or mean_value > upper_value:
                    raise ValueError(f"Invalid confidence interval for {metric['title']} on line {line_number}.")

            rows.append(row)

    if not rows:
        raise ValueError(f"Sweep summary contains no results: {SUMMARY_PATH}")

    rows.sort(key=lambda row: row["fleet_size"])

    fleet_sizes = [row["fleet_size"] for row in rows]
    if len(fleet_sizes) != len(set(fleet_sizes)):
        raise ValueError("Sweep summary contains duplicate vehicle counts.")

    return rows


def main() -> int:
    try:
        rows = read_summary()
    except (OSError, ValueError) as error:
        print(f"Could not plot sweep metrics: {error}")
        return 1

    fig, axes = plt.subplots(2, 2, figsize=(13, 9), constrained_layout=True)
    fig.suptitle("MANTA Fleet-Size Monte Carlo Sweep", fontsize=15)

    for ax, metric in zip(axes.flat, METRICS):
        plot_metric(ax, rows, metric)

    OUTPUT_PATH.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(OUTPUT_PATH, dpi=200, bbox_inches="tight")
    print(f"Saved fleet-sweep metrics plot: {OUTPUT_PATH}")

    plt.close(fig)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
