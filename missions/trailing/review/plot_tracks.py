#!/usr/bin/env python3
"""Plot trailing mission tracks and report whale-prediction metrics."""

import csv
from collections import defaultdict
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.ticker import FuncFormatter
from parse_obd import read_obd


ROOT = Path(__file__).resolve().parents[3]
DATA_DIR = ROOT / "output" / "trailing" / "Data"
WHALE_CALLS_PATH = DATA_DIR / "whale_calls.csv"
WHALE_DETECTIONS_PATH = DATA_DIR / "whale_detections.csv"
WHALE_TRAJECTORY_PATH = DATA_DIR / "whales.csv"

WHALE_COLOR = "tab:red"
WHALE_CALL_COLOR = "tab:red"

plt.rcParams.update({"font.size": 14, "axes.titlesize": 17, "axes.labelsize": 15, "xtick.labelsize": 13, "ytick.labelsize": 13, "legend.fontsize": 12, "figure.titlesize": 18})


def vehicle_id(path: Path) -> str:
    """Extract the vehicle ID from an Odyssey result filename."""
    return path.stem


def find_latest_vehicle_results() -> dict[str, Path]:
    """Find the newest Odyssey result for each vehicle."""
    latest = {}

    for path in DATA_DIR.rglob("*.obd"):
        name = vehicle_id(path)
        if name not in latest or path.stat().st_mtime > latest[name].stat().st_mtime:
            latest[name] = path

    return latest


def read_vehicle_positions(dataset, name: str) -> tuple[np.ndarray, np.ndarray]:
    """Read horizontal truth positions, falling back to navigation positions."""
    if "real_x" in dataset and "real_y" in dataset:
        return np.asarray(dataset["real_x"].values, dtype=float), np.asarray(dataset["real_y"].values, dtype=float)

    if "m_pos_n" in dataset and "m_pos_e" in dataset:
        return np.asarray(dataset["m_pos_n"].values, dtype=float), np.asarray(dataset["m_pos_e"].values, dtype=float)

    raise ValueError(f"Vehicle {name} has no position data.")


def plot_vehicle_tracks(ax, latest: dict[str, Path]) -> int:
    """Plot each vehicle trajectory with a separate color."""
    plotted = 0

    for index, (name, path) in enumerate(sorted(latest.items())):
        print(f"Plotting vehicle {name}: {path.name}")

        try:
            dataset = read_obd(path)
            north, east = read_vehicle_positions(dataset, name)
        except (OSError, ValueError) as error:
            print(f"Skipping {name}: {error}")
            continue

        if all(variable in dataset for variable in ("real_u", "real_v", "real_w")):
            forward = np.asarray(dataset["real_u"].values, dtype=float)
            lateral = np.asarray(dataset["real_v"].values, dtype=float)
            vertical = np.asarray(dataset["real_w"].values, dtype=float)
            speed = np.sqrt(forward * forward + lateral * lateral + vertical * vertical)
            speed = speed[np.isfinite(speed)]

            if speed.size:
                print(f"  Actual speed: median={np.median(speed):.2f} m/s, mean={np.mean(speed):.2f} m/s, 95th={np.percentile(speed, 95):.2f} m/s, max={np.max(speed):.2f} m/s")

        valid = np.isfinite(north) & np.isfinite(east)
        north = north[valid]
        east = east[valid]

        if not north.size:
            print(f"Skipping {name}: no valid positions.")
            continue

        color = f"C{index % 10}"
        ax.plot(east, north, color=color, linewidth=2.0, label=name, zorder=4)
        ax.scatter(east[0], north[0], color=color, marker="o", s=40, zorder=6)
        ax.scatter(east[-1], north[-1], color=color, marker="s", s=40, zorder=6)
        plotted += 1

    return plotted


def read_whale_detections() -> dict[tuple[int, str], list[tuple[str, float, float]]]:
    """Read logged detections grouped by cycle and whale ID."""
    detections_by_call = defaultdict(list)

    if not WHALE_DETECTIONS_PATH.is_file():
        print(f"No whale detection file found: {WHALE_DETECTIONS_PATH}")
        return {}

    try:
        with WHALE_DETECTIONS_PATH.open("r", encoding="utf-8", newline="") as file:
            reader = csv.DictReader(file)
            required = {"cycle", "vehicle_id", "whale_id", "whale_north", "whale_east"}

            if not reader.fieldnames or not required.issubset(reader.fieldnames):
                raise ValueError("whale_detections.csv must contain cycle, vehicle_id, whale_id, whale_north, and whale_east columns.")

            for row in reader:
                name = row.get("vehicle_id", "")
                whale = row.get("whale_id", "")

                try:
                    cycle = int(row["cycle"])
                    north = float(row["whale_north"])
                    east = float(row["whale_east"])
                except (TypeError, ValueError):
                    continue

                if name and whale and np.isfinite(north) and np.isfinite(east):
                    detections_by_call[(cycle, whale)].append((name, north, east))

    except (OSError, ValueError) as error:
        print(f"Could not read whale detections: {error}")

    return dict(detections_by_call)


def read_whale_tracks(path: Path) -> dict[str, tuple[list[float], list[float]]]:
    """Read whale trajectories grouped by whale ID."""
    tracks = defaultdict(lambda: ([], []))

    with path.open("r", encoding="utf-8", newline="") as file:
        reader = csv.DictReader(file)
        required = {"whale_id", "north", "east"}

        if not reader.fieldnames or not required.issubset(reader.fieldnames):
            raise ValueError("whales.csv must contain whale_id, north, and east columns.")

        for row in reader:
            try:
                north = float(row["north"])
                east = float(row["east"])
            except (TypeError, ValueError):
                continue

            if not row["whale_id"] or not np.isfinite(north) or not np.isfinite(east):
                continue

            track_north, track_east = tracks[row["whale_id"]]
            track_north.append(north)
            track_east.append(east)

    return dict(tracks)


def read_whale_positions(path: Path) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Read the single trailing whale's position at each logged time."""
    positions_by_time = {}
    whale_ids = set()

    with path.open("r", encoding="utf-8", newline="") as file:
        reader = csv.DictReader(file)
        required = {"whale_id", "sim_time", "north", "east"}

        if not reader.fieldnames or not required.issubset(reader.fieldnames):
            raise ValueError("whales.csv must contain whale_id, sim_time, north, and east columns.")

        for row in reader:
            try:
                sim_time = float(row["sim_time"])
                north = float(row["north"])
                east = float(row["east"])
            except (TypeError, ValueError):
                continue

            if not row["whale_id"] or not np.isfinite(sim_time) or not np.isfinite(north) or not np.isfinite(east):
                continue

            whale_ids.add(row["whale_id"])
            positions_by_time[sim_time] = (north, east)

    if len(whale_ids) != 1:
        raise ValueError(f"Prediction error requires exactly one whale; found {len(whale_ids)}.")

    times = np.asarray(sorted(positions_by_time), dtype=float)
    north = np.asarray([positions_by_time[time][0] for time in times], dtype=float)
    east = np.asarray([positions_by_time[time][1] for time in times], dtype=float)

    return times, north, east


def compute_prediction_error(
    latest: dict[str, Path],
    name: str,
) -> tuple[float, float, float]:
    """Return mean error, RMSE, and coverage for logged whale predictions."""
    if name not in latest:
        raise ValueError(f"No Odyssey result file found for {name}.")

    data_directory = latest[name].parent
    prediction_path = data_directory / f"{name}_whale_predictions.csv"
    whale_path = data_directory / "whales.csv"

    if not prediction_path.is_file():
        raise ValueError(f"No whale prediction file found: {prediction_path}")
    if not whale_path.is_file():
        raise ValueError(f"No whale trajectory file found: {whale_path}")

    whale_time, whale_north, whale_east = read_whale_positions(whale_path)
    errors_m = []
    total_samples = 0

    with prediction_path.open("r", encoding="utf-8", newline="") as file:
        reader = csv.DictReader(file)
        required = {
            "sim_time",
            "has_estimate",
            "predicted_north",
            "predicted_east",
        }

        if not reader.fieldnames or not required.issubset(reader.fieldnames):
            raise ValueError(
                f"{prediction_path.name} must contain "
                "sim_time, has_estimate, predicted_north, and predicted_east."
            )

        for row in reader:
            total_samples += 1

            try:
                sim_time = float(row["sim_time"])
                has_estimate = int(row["has_estimate"])
            except (TypeError, ValueError):
                continue

            if has_estimate != 1 or not np.isfinite(sim_time):
                continue

            # Interpolate truth only within the logged whale trajectory.
            if sim_time < whale_time[0] or sim_time > whale_time[-1]:
                continue

            try:
                predicted_north = float(row["predicted_north"])
                predicted_east = float(row["predicted_east"])
            except (TypeError, ValueError):
                continue

            if not np.isfinite(predicted_north) or not np.isfinite(predicted_east):
                continue

            true_north = np.interp(sim_time, whale_time, whale_north)
            true_east = np.interp(sim_time, whale_time, whale_east)
            errors_m.append(
                np.hypot(predicted_north - true_north, predicted_east - true_east)
            )

    if not errors_m:
        raise ValueError(f"No valid whale predictions found for {name}.")

    errors_m = np.asarray(errors_m, dtype=float)
    return (
        float(np.mean(errors_m)),
        float(np.sqrt(np.mean(errors_m ** 2))),
        float(len(errors_m) / total_samples),
    )


def plot_whale_tracks(ax) -> int:
    """Plot logged whale trajectories."""
    if not WHALE_TRAJECTORY_PATH.is_file():
        print(f"No whale trajectory file found: {WHALE_TRAJECTORY_PATH}")
        return 0

    try:
        tracks = read_whale_tracks(WHALE_TRAJECTORY_PATH)
    except (OSError, ValueError) as error:
        print(f"Could not read whale trajectories: {error}")
        return 0

    plotted = 0

    for whale, (north, east) in sorted(tracks.items()):
        if not north:
            continue

        print(f"Plotting whale {whale}.")
        label = "Whale trajectory" if plotted == 0 else None
        ax.plot(east, north, color=WHALE_COLOR, linestyle="--", linewidth=1.0, alpha=0.55, label=label, zorder=1)
        ax.scatter(east[0], north[0], color=WHALE_COLOR, marker="x", s=25, alpha=0.7, zorder=2)
        plotted += 1

    return plotted


def plot_whale_calls(ax, detections_by_call: dict[tuple[int, str], list[tuple[str, float, float]]]) -> int:
    """Plot calls without any logged detection as red dots."""
    if not WHALE_CALLS_PATH.is_file():
        print(f"No whale call file found: {WHALE_CALLS_PATH}")
        return 0

    north = []
    east = []

    try:
        with WHALE_CALLS_PATH.open("r", encoding="utf-8", newline="") as file:
            reader = csv.DictReader(file)
            required = {"cycle", "whale_id", "north", "east"}

            if not reader.fieldnames or not required.issubset(reader.fieldnames):
                raise ValueError("whale_calls.csv must contain cycle, whale_id, north, and east columns.")

            for row in reader:
                whale = row.get("whale_id", "")

                try:
                    cycle = int(row["cycle"])
                    call_north = float(row["north"])
                    call_east = float(row["east"])
                except (TypeError, ValueError):
                    continue

                if not whale or not np.isfinite(call_north) or not np.isfinite(call_east):
                    continue

                if (cycle, whale) not in detections_by_call:
                    north.append(call_north)
                    east.append(call_east)

    except (OSError, ValueError) as error:
        print(f"Could not read whale calls: {error}")
        return 0

    if north:
        ax.scatter(east, north, color=WHALE_CALL_COLOR, marker=".", s=12, alpha=0.5, label="Undetected whale calls", zorder=5)

    print(f"Plotting {len(north)} undetected whale calls.")
    return len(north)


def plot_detection_locations(ax, detections_by_call: dict[tuple[int, str], list[tuple[str, float, float]]]) -> int:
    """Plot logged whale call locations, not triangulated estimates."""
    detections_by_vehicle = defaultdict(lambda: ([], []))

    for detections in detections_by_call.values():
        for name, north, east in detections:
            detection_north, detection_east = detections_by_vehicle[name]
            detection_north.append(north)
            detection_east.append(east)

    detection_count = sum(len(north) for north, _ in detections_by_vehicle.values())

    for name, (north, east) in sorted(detections_by_vehicle.items()):
        ax.scatter(east, north, color=WHALE_COLOR, marker="D", s=45, linewidths=1.0, label=f"Calls received by {name}", zorder=7)

    print(f"Plotting {detection_count} logged detections.")
    return detection_count


def main() -> int:
    """Plot mission tracks and report whale-prediction metrics for each AUV."""
    detections_by_call = read_whale_detections()
    latest = find_latest_vehicle_results()
    prediction_labels = []

    for name in sorted(latest):
        try:
            mean_error_m, rmse_m, coverage = compute_prediction_error(latest, name)
        except (OSError, ValueError) as error:
            print(f"Could not calculate prediction error for {name}: {error}")
            prediction_labels.append(f"{name}: unavailable")
            continue

        print(f"{name} mean whale prediction error: {mean_error_m / 1000.0:.3f} km")
        print(f"{name} whale prediction RMSE: {rmse_m / 1000.0:.3f} km")
        print(f"{name} prediction coverage: {coverage:.1%}")
        prediction_labels.append(f"{name}: {mean_error_m / 1000.0:.2f} km")

    fig, ax = plt.subplots(figsize=(10, 8))
    fleet_size = plot_vehicle_tracks(ax, latest)
    whale_count = plot_whale_tracks(ax)
    call_count = plot_whale_calls(ax, detections_by_call)
    detection_count = plot_detection_locations(ax, detections_by_call)

    if fleet_size == 0 and whale_count == 0 and call_count == 0 and detection_count == 0:
        print(f"No simulation results found in {DATA_DIR}.")
        plt.close(fig)
        return 1

    title = "Vehicle and Whale Tracks"
    if prediction_labels:
        title += "\nMean whale prediction error — " + "; ".join(prediction_labels)

    ax.set_title(title)
    km_formatter = FuncFormatter(lambda value, _: f"{value / 1000:g}")
    ax.xaxis.set_major_formatter(km_formatter)
    ax.yaxis.set_major_formatter(km_formatter)
    ax.set_xlabel("East (km)")
    ax.set_ylabel("North (km)")
    ax.set_aspect("equal", adjustable="box")
    ax.margins(x=0.03, y=0.05)
    ax.grid(True, alpha=0.3)
    ax.legend(ncols=2)
    fig.tight_layout()
    plt.show()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())