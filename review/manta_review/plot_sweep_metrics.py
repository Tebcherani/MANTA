#!/usr/bin/env python3

import argparse
import csv
import math
from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.ticker import PercentFormatter


METRICS = (
    {
        "mean": "mean_detection",
        "lower": "ci_lower",
        "upper": "ci_upper",
        "title": "Whale Detection",
        "ylabel": "Unique whales detected (%)",
        "color": "tab:blue",
        "percent": True,
    },
    {
        "mean": "mean_total_mass_kg",
        "lower": "mass_ci_lower_kg",
        "upper": "mass_ci_upper_kg",
        "title": "Total Fleet Mass",
        "ylabel": "Mass (kg)",
        "color": "tab:green",
        "percent": False,
    },
    {
        "mean": "mean_total_energy_wh",
        "lower": "energy_ci_lower_wh",
        "upper": "energy_ci_upper_wh",
        "title": "Total Fleet Energy Consumption",
        "ylabel": "Energy consumed (Wh)",
        "color": "tab:orange",
        "percent": False,
    },
    {
        "mean": "mean_vehicle_speed_mps",
        "lower": "speed_ci_lower_mps",
        "upper": "speed_ci_upper_mps",
        "title": "Mean Vehicle Speed",
        "ylabel": "Speed (m/s)",
        "color": "tab:red",
        "percent": False,
    },
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Plot detection, mass, energy, and speed from a MANTA "
            "fleet-size sweep summary."
        )
    )
    parser.add_argument(
        "summary",
        type=Path,
        help="Path to output/monte_carlo_sweep/sweep_summary.csv.",
    )
    parser.add_argument(
        "output",
        type=Path,
        help="Path where the combined figure will be saved.",
    )
    parser.add_argument(
        "--show",
        action="store_true",
        help="Display the figure after saving it.",
    )
    return parser.parse_args()


def required_columns() -> set[str]:
    columns = {"vehicle_count"}

    for metric in METRICS:
        columns.update(
            (metric["mean"], metric["lower"], metric["upper"])
        )

    return columns


def read_summary(path: Path) -> list[dict[str, float]]:
    """Read already-aggregated fleet statistics from sweep_summary.csv."""

    if not path.is_file():
        raise ValueError(f"Sweep summary does not exist: {path}")

    rows = []

    with path.open("r", encoding="utf-8", newline="") as file:
        reader = csv.DictReader(file)
        required = required_columns()

        if not reader.fieldnames:
            raise ValueError(f"Sweep summary has no header: {path}")

        missing = sorted(required.difference(reader.fieldnames))

        if missing:
            raise ValueError(
                "Sweep summary is missing the new metric columns: "
                + ", ".join(missing)
                + ". Run the sweep with manta_runner_v2.py first."
            )

        for line_number, source in enumerate(reader, start=2):
            try:
                row = {
                    "vehicle_count": int(source["vehicle_count"]),
                }

                for metric in METRICS:
                    row[metric["mean"]] = float(source[metric["mean"]])
                    row[metric["lower"]] = float(source[metric["lower"]])
                    row[metric["upper"]] = float(source[metric["upper"]])
            except (TypeError, ValueError) as error:
                raise ValueError(
                    f"Invalid numeric value on line {line_number} of {path}."
                ) from error

            values = [
                value
                for name, value in row.items()
                if name != "vehicle_count"
            ]

            if row["vehicle_count"] < 1:
                raise ValueError(
                    f"Vehicle count must be positive on line {line_number}."
                )

            if not all(math.isfinite(value) for value in values):
                raise ValueError(
                    f"Non-finite metric on line {line_number} of {path}."
                )

            for metric in METRICS:
                mean_value = row[metric["mean"]]
                lower_value = row[metric["lower"]]
                upper_value = row[metric["upper"]]

                if lower_value > mean_value or mean_value > upper_value:
                    raise ValueError(
                        f"Invalid confidence interval for {metric['title']} "
                        f"on line {line_number}."
                    )

            rows.append(row)

    if not rows:
        raise ValueError(f"Sweep summary contains no results: {path}")

    rows.sort(key=lambda row: row["vehicle_count"])

    vehicle_counts = [row["vehicle_count"] for row in rows]
    if len(vehicle_counts) != len(set(vehicle_counts)):
        raise ValueError("Sweep summary contains duplicate vehicle counts.")

    return rows


def plot_metric(ax, rows: list[dict[str, float]], metric: dict) -> None:
    vehicles = [row["vehicle_count"] for row in rows]
    means = [row[metric["mean"]] for row in rows]
    lower_errors = [
        row[metric["mean"]] - row[metric["lower"]]
        for row in rows
    ]
    upper_errors = [
        row[metric["upper"]] - row[metric["mean"]]
        for row in rows
    ]

    ax.errorbar(
        vehicles,
        means,
        yerr=[lower_errors, upper_errors],
        color=metric["color"],
        marker="o",
        markersize=5,
        linewidth=1.8,
        capsize=4,
    )
    ax.set_title(metric["title"])
    ax.set_xlabel("Number of Odyssey vehicles")
    ax.set_ylabel(metric["ylabel"])
    ax.set_xticks(vehicles)
    ax.grid(True, alpha=0.3)

    if metric["percent"]:
        ax.set_ylim(0.0, 1.0)
        ax.yaxis.set_major_formatter(PercentFormatter(1.0))


def main() -> int:
    args = parse_args()

    try:
        rows = read_summary(args.summary)
    except (OSError, ValueError) as error:
        print(f"Could not plot sweep metrics: {error}")
        return 1

    fig, axes = plt.subplots(2, 2, figsize=(13, 9), constrained_layout=True)

    for ax, metric in zip(axes.flat, METRICS):
        plot_metric(ax, rows, metric)

    fig.suptitle("MANTA Fleet-Size Monte Carlo Sweep", fontsize=15)

    args.output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.output, dpi=200, bbox_inches="tight")
    print(f"Saved fleet-sweep metrics plot: {args.output}")

    if args.show:
        plt.show()
    else:
        plt.close(fig)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
