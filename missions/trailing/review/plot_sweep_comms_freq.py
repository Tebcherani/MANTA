#!/usr/bin/env python3
"""Box plots of paired prediction-error differences against immediate sharing."""

import csv
import math
from functools import lru_cache
from pathlib import Path
from statistics import mean, median

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from matplotlib.patches import Patch

from trailing_metrics import compute_prediction_metrics

ROOT = Path(__file__).resolve().parents[3]
SUMMARY_PATH = ROOT / "output/trailing/sweep_comms/sweep_summary.csv"
OUTPUT_PATH = ROOT / "output/trailing/sweep_comms/sweep_metrics.png"


def read_sweep_summary() -> list[dict]:
    """Read completed vehicle/interval settings; aggregate means cannot be paired."""
    with SUMMARY_PATH.open(newline="", encoding="utf-8") as file:
        reader = csv.DictReader(file)
        required = {"vehicle_id", "call_update_interval"}
        if not required.issubset(reader.fieldnames or []):
            raise ValueError("Sweep summary must contain vehicle_id and call_update_interval.")
        settings = set()
        for line, row in enumerate(reader, start=2):
            vehicle = row["vehicle_id"].strip()
            interval = float(row["call_update_interval"])
            if not vehicle or not math.isfinite(interval) or interval < 0:
                raise ValueError(f"Invalid vehicle/interval on summary line {line}.")
            if (vehicle, interval) in settings:
                raise ValueError(f"Duplicate vehicle/interval on summary line {line}.")
            settings.add((vehicle, interval))
    if not settings:
        raise ValueError("Sweep summary contains no results.")
    return [dict(vehicle_id=v, call_update_interval=t) for v, t in sorted(settings)]


def interval_directory_name(interval: float) -> str:
    if interval.is_integer():
        return f"interval_{int(interval):06d}"
    return f"interval_{interval:g}".replace(".", "_")


@lru_cache(maxsize=None)
def read_seed_errors(interval: float, vehicle_id: str) -> dict[int, float | None]:
    """Use completed realization IDs to locate logs, and seeds to pair runs."""
    directory = SUMMARY_PATH.parent / interval_directory_name(interval)
    with (directory / "summary.csv").open(newline="", encoding="utf-8") as file:
        reader = csv.DictReader(file)
        if not {"realization", "seed"}.issubset(reader.fieldnames or []):
            raise ValueError(f"Missing realization/seed columns in {directory / 'summary.csv'}.")
        errors = {}
        for row in reader:
            seed = int(row["seed"])
            realization = int(row["realization"])
            if seed in errors:
                raise ValueError(f"Duplicate seed {seed} in {directory / 'summary.csv'}.")
            data = directory / f"realization_{realization:04d}" / "Data"
            # Reuse the sweep's per-vehicle error calculation, including its
            # treatment of samples without predictions; do not use fleet means.
            error = compute_prediction_metrics(data, vehicle_id)["mean_prediction_error_km"]
            if error is not None and (not math.isfinite(error) or error < 0):
                raise ValueError(f"Invalid prediction error for seed {seed} in {data}.")
            errors[seed] = error
    return errors


def paired_differences(baseline: dict, comparison: dict) -> tuple[list, str]:
    shared = baseline.keys() & comparison.keys()
    paired = sorted(
        [(seed, baseline[seed] - comparison[seed]) for seed in shared
         if baseline[seed] is not None and comparison[seed] is not None],
        key=lambda pair: (pair[1], pair[0]),
    )
    both_missing = sum(baseline[s] is None and comparison[s] is None for s in shared)
    baseline_missing = sum(baseline[s] is None and comparison[s] is not None for s in shared)
    comparison_missing = sum(baseline[s] is not None and comparison[s] is None for s in shared)
    unpaired = len(baseline.keys() ^ comparison.keys())
    counts = (
        f"No estimate: both {both_missing}; immediate only {baseline_missing}; "
        f"comparison only {comparison_missing}. Unmatched seeds: {unpaired}."
    )
    return paired, counts


def plot_pairs(ax, vehicle: str, interval: float, position: float,
               width: float, color: str) -> str:
    baseline = read_seed_errors(0.0, vehicle)
    comparison = read_seed_errors(interval, vehicle)
    pairs, excluded = paired_differences(baseline, comparison)
    if not pairs:
        print(f"{vehicle}, {interval:g} s: no valid pairs. {excluded}")
        return f"{vehicle}: no valid pairs"

    values = [difference for _, difference in pairs]
    average, middle = mean(values), median(values)
    ax.boxplot(
        [values], positions=[position], widths=width, patch_artist=True,
        manage_ticks=False, showmeans=True, showfliers=False, whis=1.5,
        boxprops=dict(facecolor=color, edgecolor="#456782"),
        medianprops=dict(color="#1f2937", linewidth=2),
        meanprops=dict(marker="D", markerfacecolor="#d97706",
                       markeredgecolor="#d97706", markersize=5),
        whiskerprops=dict(color="#456782"),
        capprops=dict(color="#456782"),
    )
    wins = sum(value > 0 for value in values)
    ties = sum(value == 0 for value in values)
    print(f"{vehicle}, {interval:g} s: {len(pairs)} pairs; delayed better {wins}; "
          f"ties {ties}; mean difference {average:+.4f} km; "
          f"median {middle:+.4f} km. {excluded}")
    shared = baseline.keys() & comparison.keys()
    return f"{vehicle}: {len(pairs)}/{len(shared)} valid pairs"


def main() -> int:
    fig = None
    try:
        rows = read_sweep_summary()
        settings = {(row["vehicle_id"], row["call_update_interval"]) for row in rows}
        comparisons = sorted((v, t) for v, t in settings if t > 0)
        if not comparisons:
            raise ValueError("Paired plots need immediate sharing and at least one nonzero interval.")
        if any((v, 0.0) not in settings for v, _ in comparisons):
            raise ValueError("Each vehicle needs an immediate-sharing baseline.")
        vehicles = sorted({v for v, _ in comparisons})
        intervals = sorted({t for _, t in comparisons})
        fig, ax = plt.subplots(figsize=(10, 6))
        ax.axhline(0, color="#555555", linewidth=1)
        ax.set_title("Paired prediction-error differences", fontsize=16, pad=38)
        ax.text(0.5, 1.03,
                "Above zero: delayed sharing is better · Below zero: immediate sharing is better",
                ha="center", transform=ax.transAxes, fontsize=10)
        palette = ["#b4d3e8", "#eed2a7", "#b9d9c7", "#d7c5e6"]
        group_width = 0.72
        slot = group_width / len(vehicles)
        sample_notes = []
        for index, interval in enumerate(intervals):
            notes = []
            for vehicle_index, vehicle in enumerate(vehicles):
                if (vehicle, interval) not in settings:
                    continue
                position = index - group_width / 2 + slot * (vehicle_index + 0.5)
                notes.append(plot_pairs(ax, vehicle, interval, position,
                                        slot * 0.82, palette[vehicle_index % len(palette)]))
            sample_notes.append(f"{interval:g} s — " + "; ".join(notes))
        ax.set_xticks(range(len(intervals)), [f"{t:g}" for t in intervals])
        ax.set_xlim(-0.6, len(intervals) - 0.4)
        ax.set_xlabel("Communication interval (simulated seconds)")
        ax.set_ylabel("Immediate − delayed prediction error (km)")
        ax.grid(axis="y", alpha=0.18)
        ax.spines[["top", "right"]].set_visible(False)
        ax.margins(y=0.18)
        handles = [Patch(facecolor=palette[i % len(palette)], edgecolor="#456782",
                         label=vehicle) for i, vehicle in enumerate(vehicles)]
        handles += [Line2D([], [], color="#1f2937", linewidth=2, label="Median"),
                    Line2D([], [], linestyle="none", marker="D", color="#d97706", label="Mean")]
        ax.legend(handles=handles, loc="upper left", frameon=False, ncol=2)
        footer = "\n".join(sample_notes + ["Runs without estimates are excluded, not assigned zero error. Outlier markers are hidden."])
        fig.text(0.12, 0.02, footer, fontsize=8, va="bottom", color="#555555")
        fig.subplots_adjust(top=0.82, bottom=min(0.42, 0.19 + 0.025 * len(intervals)), left=0.12, right=0.98)
        OUTPUT_PATH.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(OUTPUT_PATH, dpi=180, bbox_inches="tight")
        print(f"Saved: {OUTPUT_PATH}")
        return 0
    except (OSError, ValueError, KeyError) as error:
        print(f"Could not plot trailing sweep: {error}")
        return 1
    finally:
        if fig is not None:
            plt.close(fig)


if __name__ == "__main__":
    raise SystemExit(main())
