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
    if not SUMMARY_PATH.is_file():
        raise ValueError(f"Sweep summary does not exist: {SUMMARY_PATH}")

    numeric_fields = (
        "call_update_interval",
        "mean_distance_km",
        "distance_ci_lower_km",
        "distance_ci_upper_km",
    )
    required = {"vehicle_id", "realizations", *numeric_fields}
    rows = []
    seen = set()

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
                    **{
                        field: float(source[field])
                        for field in numeric_fields
                    },
                }
            except (TypeError, ValueError, AttributeError) as error:
                raise ValueError(
                    f"Invalid value on line {line_number}."
                ) from error

            if not row["vehicle_id"] or row["realizations"] < 1:
                raise ValueError(f"Invalid vehicle or count on line {line_number}.")

            if not all(math.isfinite(row[field]) for field in numeric_fields):
                raise ValueError(f"Non-finite value on line {line_number}.")

            if row["call_update_interval"] < 0:
                raise ValueError(f"Negative interval on line {line_number}.")

            if not (
                0 <= row["distance_ci_lower_km"]
                <= row["mean_distance_km"]
                <= row["distance_ci_upper_km"]
            ):
                raise ValueError(f"Invalid confidence interval on line {line_number}.")

            key = (row["call_update_interval"], row["vehicle_id"])
            if key in seen:
                raise ValueError(f"Duplicate interval/vehicle on line {line_number}.")
            seen.add(key)
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

    for vehicle_id in sorted({row["vehicle_id"] for row in rows}):
        vehicle_rows = sorted(
            [row for row in rows if row["vehicle_id"] == vehicle_id],
            key=lambda row: row["call_update_interval"],
        )

        means = [row["mean_distance_km"] for row in vehicle_rows]

        ax.errorbar(
            [row["call_update_interval"] / 3600 for row in vehicle_rows],
            means,
            yerr=[
                [
                    row["mean_distance_km"] - row["distance_ci_lower_km"]
                    for row in vehicle_rows
                ],
                [
                    row["distance_ci_upper_km"] - row["mean_distance_km"]
                    for row in vehicle_rows
                ],
            ],
            fmt="o-",
            capsize=5,
            label=vehicle_id,
        )

    intervals = sorted({row["call_update_interval"] for row in rows})
    ax.set_xticks([interval / 3600 for interval in intervals])
    ax.set_xticklabels(
        [
            "Every call" if interval == 0 else f"{interval / 3600:g}"
            for interval in intervals
        ],
        rotation=45,
        ha="right",
    )

    ax.set_title("Effect of Call Update Interval on AUV Tracking")
    ax.set_xlabel("Call waypoint update interval (h)")
    ax.set_ylabel("Mean distance from whale (km)")
    ax.set_ylim(bottom=0)
    ax.grid(True, alpha=0.3)
    ax.legend(title="Vehicle")

    OUTPUT_PATH.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(OUTPUT_PATH, dpi=200, bbox_inches="tight")
    plt.close(fig)

    print(f"Saved: {OUTPUT_PATH}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())