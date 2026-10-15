#!/usr/bin/env python3
"""Plot each vehicle's mean distance from the whale."""

import csv
import math
from pathlib import Path

import matplotlib.pyplot as plt


ROOT = Path(__file__).resolve().parents[3]
SUMMARY_PATH = ROOT / "output/trailing/sweep_comms/sweep_summary.csv"
OUTPUT_PATH = ROOT / "output/trailing/sweep_comms/sweep_metrics.png"


def read_sweep_summary() -> list[dict]:
    """Load and validate the trailing prediction-error summary."""

    if not SUMMARY_PATH.is_file():
        raise ValueError(f"Sweep summary does not exist: {SUMMARY_PATH}")

    required = {
        "vehicle_id",
        "realizations",
        "prediction_realizations",
        "call_update_interval",
        "prediction_coverage_fraction",
        "mean_prediction_error_km",
        "prediction_error_ci_lower_km",
        "prediction_error_ci_upper_km",
    }

    rows = []

    with SUMMARY_PATH.open("r", encoding="utf-8", newline="") as file:
        reader = csv.DictReader(file)

        if not required.issubset(reader.fieldnames or []):
            missing = sorted(required.difference(reader.fieldnames or []))
            raise ValueError(f"Missing columns: {', '.join(missing)}")

        for line_number, source in enumerate(reader, start=2):
            try:
                row = {
                    "vehicle_id": source["vehicle_id"].strip(),
                    "realizations": int(source["realizations"]),
                    "prediction_realizations": int(source["prediction_realizations"]),
                    "call_update_interval": float(source["call_update_interval"]),
                    "prediction_coverage_fraction": float(
                        source["prediction_coverage_fraction"]
                    ),
                    "mean_prediction_error_km": (
                        float(source["mean_prediction_error_km"])
                        if source["mean_prediction_error_km"].strip()
                        else None
                    ),
                    "prediction_error_ci_lower_km": (
                        float(source["prediction_error_ci_lower_km"])
                        if source["prediction_error_ci_lower_km"].strip()
                        else None
                    ),
                    "prediction_error_ci_upper_km": (
                        float(source["prediction_error_ci_upper_km"])
                        if source["prediction_error_ci_upper_km"].strip()
                        else None
                    ),
                }
            except (TypeError, ValueError, AttributeError) as error:
                raise ValueError(f"Invalid value on line {line_number}.") from error

            if not row["vehicle_id"] or row["realizations"] < 1:
                raise ValueError(f"Invalid vehicle or realization count on line {line_number}.")

            if not 0 <= row["prediction_realizations"] <= row["realizations"]:
                raise ValueError(f"Invalid prediction count on line {line_number}.")

            if not math.isfinite(row["call_update_interval"]) or row["call_update_interval"] < 0:
                raise ValueError(f"Invalid communication interval on line {line_number}.")

            coverage = row["prediction_coverage_fraction"]
            if not math.isfinite(coverage) or not 0.0 <= coverage <= 1.0:
                raise ValueError(f"Invalid prediction coverage on line {line_number}.")

            error_fields = (
                row["mean_prediction_error_km"],
                row["prediction_error_ci_lower_km"],
                row["prediction_error_ci_upper_km"],
            )

            if row["prediction_realizations"] == 0:
                if any(value is not None for value in error_fields):
                    raise ValueError(
                        f"Expected blank prediction errors on line {line_number}."
                    )
            else:
                if any(value is None for value in error_fields):
                    raise ValueError(
                        f"Missing prediction errors on line {line_number}."
                    )

                mean_error, lower, upper = error_fields
                if not all(math.isfinite(value) for value in error_fields):
                    raise ValueError(f"Non-finite prediction error on line {line_number}.")

                if not 0.0 <= lower <= mean_error <= upper:
                    raise ValueError(f"Invalid prediction-error interval on line {line_number}.")

            rows.append(row)

    if not rows:
        raise ValueError("Sweep summary contains no results.")

    return rows


def main() -> int:
    try:
        rows = read_sweep_summary()
    except (OSError, ValueError) as error:
        print(f"Could not plot trailing sweep: {error}")
        return 1

    fig, ax = plt.subplots(figsize=(10, 6), constrained_layout=True)

    plotted_vehicle = False

    for vehicle_id in sorted({row["vehicle_id"] for row in rows}):
        vehicle_rows = sorted(
            [
                row
                for row in rows
                if row["vehicle_id"] == vehicle_id
                and row["mean_prediction_error_km"] is not None
            ],
            key=lambda row: row["call_update_interval"],
        )

        if not vehicle_rows:
            print(f"{vehicle_id}: no realizations had valid whale predictions.")
            continue

        means = [row["mean_prediction_error_km"] for row in vehicle_rows]

        ax.errorbar(
            [row["call_update_interval"] / 60 for row in vehicle_rows],
            means,
            yerr=[
                [
                    row["mean_prediction_error_km"]
                    - row["prediction_error_ci_lower_km"]
                    for row in vehicle_rows
                ],
                [
                    row["prediction_error_ci_upper_km"]
                    - row["mean_prediction_error_km"]
                    for row in vehicle_rows
                ],
            ],
            fmt="o-",
            capsize=5,
            label=vehicle_id,
        )
        plotted_vehicle = True

    if plotted_vehicle:
        ax.legend(title="Vehicle")
    else:
        ax.text(
            0.5,
            0.5,
            "No valid whale predictions",
            ha="center",
            va="center",
            transform=ax.transAxes,
        )

    intervals = sorted({row["call_update_interval"] for row in rows})
    #ax.set_xticks([interval / 60 for interval in intervals])
    #ax.set_xticklabels(
    #    [
    #        "Every call" if interval == 0 else f"{interval / 60:g}"
    #        for interval in intervals
    #    ],
    #    rotation=45,
    #   ha="right",
    #)

    ax.set_title("Effect of Communication Interval on Trailing")
    ax.set_xlabel("Communication Interval (min)")
    ax.set_ylabel("Mean Whale Prediction Error (km)")
    #ax.set_ylim(bottom=0)
    ax.grid(True, alpha=0.3)
    ax.legend(title="Vehicle")

    OUTPUT_PATH.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(OUTPUT_PATH, dpi=200, bbox_inches="tight")
    plt.close(fig)

    print(f"Saved: {OUTPUT_PATH}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())