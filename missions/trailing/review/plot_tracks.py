#!/usr/bin/env python3

import csv
import matplotlib.pyplot as plt
import numpy as np

from collections import defaultdict
from pathlib import Path
from parse_obd import read_obd

ROOT = Path(__file__).resolve().parents[3]
DATA_DIR = ROOT / "output" / "trailing" / "Data"
WHALE_CALLS_PATH = DATA_DIR / "whale_calls.csv"
WHALE_DETECTIONS_PATH = DATA_DIR / "whale_detections.csv"
WHALE_TRAJECTORY_PATH = DATA_DIR / "whales.csv"

AUV_COLOR = "tab:blue"
WHALE_COLOR = "tab:red"
WHALE_CALL_COLOR = "tab:red"

plt.rcParams.update({
    "font.size": 14,
    "axes.titlesize": 17,
    "axes.labelsize": 15,
    "xtick.labelsize": 13,
    "ytick.labelsize": 13,
    "legend.fontsize": 12,
    "figure.titlesize": 18,
})


def find_latest_vehicle_results() -> dict[str, Path]:
    """Find the newest Odyssey result for each vehicle."""

    latest = {}

    for path in DATA_DIR.rglob("*.obd"):
        name = vehicle_id(path)
        if name and (name not in latest or path.stat().st_mtime > latest[name].stat().st_mtime):
            latest[name] = path

    return latest


def plot_vehicle_tracks(ax, latest: dict[str, Path]) -> tuple[int, dict[str, str]]:
    """Plot the latest trajectory from each Odyssey vehicle."""

    plotted = 0
    vehicle_colors = {}

    for name, path in sorted(latest.items()):
        print(f"Plotting vehicle {name}: {path.name}")

        try:
            dataset = read_obd(path)
        except Exception as error:
            print(f"Skipping {name}: {error}")
            continue

        if all(variable in dataset for variable in ("real_u", "real_v", "real_w")):
            forward = np.asarray(dataset["real_u"].values, dtype=float)
            lateral = np.asarray(dataset["real_v"].values, dtype=float)
            vertical = np.asarray(dataset["real_w"].values, dtype=float)

            speed = np.sqrt(forward * forward + lateral * lateral + vertical * vertical)
            speed = speed[np.isfinite(speed)]
            if speed.size:
                print(
                    f"  Actual speed: "
                    f"median={np.median(speed):.2f} m/s, "
                    f"mean={np.mean(speed):.2f} m/s, "
                    f"95th={np.percentile(speed, 95):.2f} m/s, "
                    f"max={np.max(speed):.2f} m/s"
                )

        if "real_x" in dataset and "real_y" in dataset:
            north = dataset["real_x"].values
            east = dataset["real_y"].values
        elif "m_pos_n" in dataset and "m_pos_e" in dataset:
            north = dataset["m_pos_n"].values
            east = dataset["m_pos_e"].values
        else:
            print(f"Skipping {name}: no position data.")
            continue

        valid = np.isfinite(north) & np.isfinite(east)
        north = north[valid]
        east = east[valid]

        if not north.size:
            print(f"Skipping {name}: no valid positions.")
            continue

        line, = ax.plot(east, north, color=AUV_COLOR, linewidth=2.0, label=name, zorder=4)
        color = line.get_color()
        vehicle_colors[name] = color
        ax.scatter(east[0], north[0], color=color, marker="o", s=40, zorder=6)  # Vehicle starting position
        ax.scatter(east[-1], north[-1], color=color, marker="s", s=40, zorder=6)    # Vehicle ending position

        plotted += 1

    return plotted, vehicle_colors


def read_whale_detections() -> dict[tuple[int, str], list[tuple[str, float, float]]]:
    """Read received whale calls grouped by simulation cycle and whale ID."""

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
                vehicle_id = row.get("vehicle_id", "")
                whale_id = row.get("whale_id", "")

                try:
                    cycle = int(row["cycle"])
                    north = float(row["whale_north"])
                    east = float(row["whale_east"])
                except (TypeError, ValueError):
                    continue

                if not vehicle_id or not whale_id or not np.isfinite(north) or not np.isfinite(east):
                    continue

                detections_by_call[(cycle, whale_id)].append((vehicle_id, north, east))

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

            if not np.isfinite(north) or not np.isfinite(east):
                continue

            track_north, track_east = tracks[row["whale_id"]]
            track_north.append(north)
            track_east.append(east)

    return tracks


def read_whale_centroids(path: Path) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Calculate the whale-group centroid at every logged simulation time."""

    positions_by_time = defaultdict(list)

    with path.open("r", encoding="utf-8", newline="") as file:
        reader = csv.DictReader(file)
        required = {"sim_time", "north", "east"}

        if not reader.fieldnames or not required.issubset(reader.fieldnames):
            raise ValueError("whales.csv must contain sim_time, north, and east columns.")

        for row in reader:
            try:
                sim_time = float(row["sim_time"])
                north = float(row["north"])
                east = float(row["east"])
            except (TypeError, ValueError):
                continue

            if not np.isfinite(sim_time) or not np.isfinite(north) or not np.isfinite(east):
                continue

            positions_by_time[sim_time].append((north, east))

    times = np.asarray(sorted(positions_by_time), dtype=float)
    centroid_north = np.asarray([np.mean([position[0] for position in positions_by_time[time]]) for time in times], dtype=float)
    centroid_east = np.asarray([np.mean([position[1] for position in positions_by_time[time]]) for time in times], dtype=float)

    return times, centroid_north, centroid_east


def compute_centroid_tracking_error(latest: dict[str, Path]) -> tuple[float, float]:
    """Calculate integrated and mean AUV distance from the whale-group centroid."""

    if not WHALE_TRAJECTORY_PATH.is_file():
        raise ValueError(f"No whale trajectory file found: {WHALE_TRAJECTORY_PATH}")

    if not latest:
        raise ValueError("No Odyssey result files were found.")

    vehicle_id, path = sorted(latest.items())[0]
    dataset = read_obd(path)

    if "time" not in dataset.coords:
        raise ValueError(f"Vehicle {vehicle_id} has no simulation-time coordinate.")

    if "real_x" in dataset and "real_y" in dataset:
        vehicle_north = np.asarray(dataset["real_x"].values, dtype=float)
        vehicle_east = np.asarray(dataset["real_y"].values, dtype=float)
    elif "m_pos_n" in dataset and "m_pos_e" in dataset:
        vehicle_north = np.asarray(dataset["m_pos_n"].values, dtype=float)
        vehicle_east = np.asarray(dataset["m_pos_e"].values, dtype=float)
    else:
        raise ValueError(f"Vehicle {vehicle_id} has no position data.")

    vehicle_time = np.asarray(dataset.coords["time"].values, dtype=float)
    valid_vehicle = np.isfinite(vehicle_time) & np.isfinite(vehicle_north) & np.isfinite(vehicle_east)
    vehicle_time = vehicle_time[valid_vehicle]
    vehicle_north = vehicle_north[valid_vehicle]
    vehicle_east = vehicle_east[valid_vehicle]

    if vehicle_time.size < 2:
        raise ValueError(f"Vehicle {vehicle_id} has insufficient position samples.")

    centroid_time, centroid_north, centroid_east = read_whale_centroids(WHALE_TRAJECTORY_PATH)
    overlap = (centroid_time >= vehicle_time[0]) & (centroid_time <= vehicle_time[-1])
    centroid_time = centroid_time[overlap]
    centroid_north = centroid_north[overlap]
    centroid_east = centroid_east[overlap]

    if centroid_time.size < 2:
        raise ValueError("Vehicle and whale trajectories have insufficient overlapping times.")

    interpolated_north = np.interp(centroid_time, vehicle_time, vehicle_north)
    interpolated_east = np.interp(centroid_time, vehicle_time, vehicle_east)
    centroid_distance = np.hypot(interpolated_north - centroid_north, interpolated_east - centroid_east)

    integrated_error_m_seconds = float(np.trapezoid(centroid_distance, centroid_time))
    duration = float(centroid_time[-1] - centroid_time[0])
    mean_distance_m = integrated_error_m_seconds / duration
    integrated_error_km_hours = integrated_error_m_seconds / 3_600_000.0

    return integrated_error_km_hours, mean_distance_m


def vehicle_id(path: Path) -> str | None:
    """Extract the vehicle ID from an Odyssey result filename."""
    return path.stem


def plot_whale_tracks(ax) -> int:
    """Plot every whale trajectory using the same color."""

    if not WHALE_TRAJECTORY_PATH.is_file():
        print(f"No whale trajectory file found: {WHALE_TRAJECTORY_PATH}")
        return 0

    try:
        tracks = read_whale_tracks(WHALE_TRAJECTORY_PATH)
    except (OSError, ValueError) as error:
        print(f"Could not read whale trajectories: {error}")
        return 0

    plotted = 0
    label_used = False

    for whale_id, (north, east) in sorted(tracks.items()):
        if not north:
            continue

        print(f"Plotting whale {whale_id}.")
        label = None if label_used else "Whale trajectories"
        ax.plot(east, north, color=WHALE_COLOR, linestyle="--", linewidth=1.0, alpha=0.55, label=label, zorder=1)
        ax.scatter(east[0], north[0], color=WHALE_COLOR, marker="x", s=25, alpha=0.7, zorder=2)
        label_used = True
        plotted += 1

    return plotted


def plot_whale_calls(ax, detections_by_call: dict[tuple[int, str], list[tuple[str, float, float]]]) -> int:
    """Plot only undetected whale calls as red dots."""

    if not WHALE_CALLS_PATH.is_file():
        print(f"No whale call file found: {WHALE_CALLS_PATH}")
        return 0

    north = []
    east = []
    call_count = 0

    try:
        with WHALE_CALLS_PATH.open("r", encoding="utf-8", newline="") as file:
            reader = csv.DictReader(file)
            required = {"cycle", "whale_id", "north", "east"}

            if not reader.fieldnames or not required.issubset(reader.fieldnames):
                raise ValueError("whale_calls.csv must contain cycle, whale_id, north, and east columns.")

            for row in reader:
                whale_id = row.get("whale_id", "")

                try:
                    cycle = int(row["cycle"])
                    call_north = float(row["north"])
                    call_east = float(row["east"])
                except (TypeError, ValueError):
                    continue

                if not whale_id or not np.isfinite(call_north) or not np.isfinite(call_east):
                    continue

                call_count += 1

                if (cycle, whale_id) not in detections_by_call:
                    north.append(call_north)
                    east.append(call_east)

    except (OSError, ValueError) as error:
        print(f"Could not read whale calls: {error}")
        return 0

    if north:
        ax.scatter(east, north, color=WHALE_CALL_COLOR, marker=".", s=12, alpha=0.5, label="Undetected whale calls", zorder=5)

    print(f"Plotting {call_count} whale calls.")
    return call_count


def plot_detection_locations(ax, detections_by_call: dict[tuple[int, str], list[tuple[str, float, float]]]) -> int:
    """Plot detected whale calls as solid red diamonds."""

    detections_by_vehicle = defaultdict(lambda: ([], []))

    for detections in detections_by_call.values():
        for vehicle_id, north, east in detections:
            detection_north, detection_east = detections_by_vehicle[vehicle_id]
            detection_north.append(north)
            detection_east.append(east)

    detection_count = sum(len(north) for north, _ in detections_by_vehicle.values())

    if detection_count == 0:
        print("No whale detections found.")
        return 0

    for vehicle_id, (north, east) in sorted(detections_by_vehicle.items()):
        ax.scatter(east, north, color=WHALE_COLOR, marker="D", s=45, linewidths=1.0, label=f"Calls received by {vehicle_id}", zorder=7)

    print(f"Plotting {detection_count} received whale calls.")
    return detection_count


def main() -> int:
    detections_by_call = read_whale_detections()
    latest = find_latest_vehicle_results()

    try:
        integrated_error, mean_distance = compute_centroid_tracking_error(latest)
    except (OSError, ValueError) as error:
        print(f"Could not calculate centroid tracking error: {error}")
        integrated_error = float("nan")
        mean_distance = float("nan")

    fig, ax = plt.subplots(figsize=(10, 8))
    fleet_size, vehicle_colors = plot_vehicle_tracks(ax, latest)
    whale_count = plot_whale_tracks(ax)
    call_count = plot_whale_calls(ax, detections_by_call)
    detection_count = plot_detection_locations(ax, detections_by_call)

    if (fleet_size == 0 and whale_count == 0 and call_count == 0 and detection_count == 0):
        print("No simulation results found in output/picket_line/Data.")
        plt.close(fig)
        return 1

    if np.isfinite(integrated_error):
        print(f"Mean centroid distance: {mean_distance / 1000.0:.3f} km")
        print(f"Integrated centroid tracking error: {integrated_error:.3f} km·h")

    if np.isfinite(integrated_error):
        ax.set_title(f"Vehicle and Whale Tracks — Mean Centroid Distance: {mean_distance / 1000.0:.2f} km")
    else:
        ax.set_title("Vehicle and Whale Tracks")
    ax.set_xlabel("East (m)")
    ax.set_ylabel("North (m)")
    ax.set_aspect("auto")
    ax.margins(x=0.03, y=0.05)
    ax.grid(True, alpha=0.3)
    ax.legend(ncols=2)
    fig.tight_layout()
    plt.show()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())