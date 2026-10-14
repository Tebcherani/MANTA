#!/usr/bin/env python3
"""Plot trailing mission tracks and report distance metrics for every AUV."""

import csv
from collections import defaultdict
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import yaml
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


def read_whale_detections() -> dict[int, list[tuple[str, float, float]]]:
    """Read logged detections grouped by cycle and whale ID."""
    detections_by_call = defaultdict(list)

    if not WHALE_DETECTIONS_PATH.is_file():
        print(f"No whale detection file found: {WHALE_DETECTIONS_PATH}")
        return {}

    try:
        with WHALE_DETECTIONS_PATH.open("r", encoding="utf-8", newline="") as file:
            reader = csv.DictReader(file)
            required = {"cycle", "vehicle_id", "whale_north", "whale_east"}

            if not reader.fieldnames or not required.issubset(reader.fieldnames):
                raise ValueError("whale_detections.csv must contain cycle, vehicle_id, whale_north, and whale_east columns.")

            for row in reader:
                name = row.get("vehicle_id", "")

                try:
                    cycle = int(row["cycle"])
                    north = float(row["whale_north"])
                    east = float(row["whale_east"])
                except (TypeError, ValueError):
                    continue

                if name and np.isfinite(north) and np.isfinite(east):
                    detections_by_call[cycle].append((name, north, east))

    except (OSError, ValueError) as error:
        print(f"Could not read whale detections: {error}")

    return dict(detections_by_call)


def read_whale_tracks(path: Path) -> tuple[list[float], list[float]]:
    """Read the single whale's trajectory."""
    north = []
    east = []

    with path.open("r", encoding="utf-8", newline="") as file:
        reader = csv.DictReader(file)
        required = {"north", "east"}

        if not reader.fieldnames or not required.issubset(reader.fieldnames):
            raise ValueError("whales.csv must contain north and east columns.")

        for row in reader:
            try:
                n = float(row["north"])
                e = float(row["east"])
            except (TypeError, ValueError):
                continue

            if np.isfinite(n) and np.isfinite(e):
                north.append(n)
                east.append(e)

    return north, east


def read_whale_positions(path: Path) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Read the single trailing whale's position at each logged time."""
    positions_by_time = {}

    with path.open("r", encoding="utf-8", newline="") as file:
        reader = csv.DictReader(file)
        required = {"sim_time", "north", "east"}

        if not reader.fieldnames or not required.issubset(reader.fieldnames):
            raise ValueError("whales.csv must contain whale_id, sim_time, north, and east columns.")

        for row in reader:
            try:
                sim_time = float(row["sim_time"])
                north = float(row["north"])
                east = float(row["east"])
            except (TypeError, ValueError):
                continue

            if not np.isfinite(sim_time) or not np.isfinite(north) or not np.isfinite(east):
                continue

            positions_by_time[sim_time] = (north, east)

    if not positions_by_time:
        raise ValueError("whales.csv contains no valid trajectory samples.")

    times = np.asarray(sorted(positions_by_time), dtype=float)
    north = np.asarray([positions_by_time[time][0] for time in times], dtype=float)
    east = np.asarray([positions_by_time[time][1] for time in times], dtype=float)

    return times, north, east


def compute_tracking_error(latest: dict[str, Path], name: str) -> tuple[float, float]:
    """Return integrated distance (km·h) and time-weighted mean distance (m)."""
    if not WHALE_TRAJECTORY_PATH.is_file():
        raise ValueError(f"No whale trajectory file found: {WHALE_TRAJECTORY_PATH}")

    if name not in latest:
        raise ValueError(f"No Odyssey result file found for {name}.")

    dataset = read_obd(latest[name])

    if "time" not in dataset.coords:
        raise ValueError(f"Vehicle {name} has no simulation-time coordinate.")

    vehicle_north, vehicle_east = read_vehicle_positions(dataset, name)
    vehicle_time = np.asarray(dataset.coords["time"].values, dtype=float)
    valid = np.isfinite(vehicle_time) & np.isfinite(vehicle_north) & np.isfinite(vehicle_east)
    vehicle_time = vehicle_time[valid]
    vehicle_north = vehicle_north[valid]
    vehicle_east = vehicle_east[valid]

    # Interpolation requires ordered timestamps without duplicates.
    order = np.argsort(vehicle_time, kind="stable")
    vehicle_time = vehicle_time[order]
    vehicle_north = vehicle_north[order]
    vehicle_east = vehicle_east[order]
    vehicle_time, indices = np.unique(vehicle_time, return_index=True)
    vehicle_north = vehicle_north[indices]
    vehicle_east = vehicle_east[indices]

    if vehicle_time.size < 2:
        raise ValueError(f"Vehicle {name} has insufficient position samples.")

    whale_time, whale_north, whale_east = read_whale_positions(WHALE_TRAJECTORY_PATH)
    overlap = (whale_time >= vehicle_time[0]) & (whale_time <= vehicle_time[-1])
    whale_time = whale_time[overlap]
    whale_north = whale_north[overlap]
    whale_east = whale_east[overlap]

    if whale_time.size < 2:
        raise ValueError(f"Vehicle {name} and whale trajectories have insufficient overlapping times.")

    interpolated_north = np.interp(whale_time, vehicle_time, vehicle_north)
    interpolated_east = np.interp(whale_time, vehicle_time, vehicle_east)
    
    # Read the same formation settings used to generate the missions.
    config_path = ROOT / "missions" / "trailing" / "config.yaml"
    with config_path.open("r", encoding="utf-8") as file:
        config = yaml.safe_load(file)

    trailing = config["scenario"]["trailing"]
    vehicle_ids = [vehicle["id"] for vehicle in config["vehicles"]]
    vehicle_index = vehicle_ids.index(name)

    following_distance_m = float(trailing["following_distance_m"])
    side_offset_m = (-0.5 if vehicle_index == 0 else 0.5) * float(trailing["vehicle_separation_m"])

    # Remove the commanded offset before calculating distance.
    north_error = interpolated_north - whale_north + following_distance_m
    east_error = interpolated_east - whale_east - side_offset_m
    distance = np.hypot(north_error, east_error)

    integrated_distance_m_seconds = float(np.trapezoid(distance, whale_time))
    duration = float(whale_time[-1] - whale_time[0])
    mean_distance_m = integrated_distance_m_seconds / duration
    integrated_distance_km_hours = integrated_distance_m_seconds / 3_600_000.0

    return integrated_distance_km_hours, mean_distance_m


def plot_whale_tracks(ax) -> int:
    """Plot the single whale's trajectory."""
    if not WHALE_TRAJECTORY_PATH.is_file():
        print(f"No whale trajectory file found: {WHALE_TRAJECTORY_PATH}")
        return 0

    try:
        north, east = read_whale_tracks(WHALE_TRAJECTORY_PATH)
    except (OSError, ValueError) as error:
        print(f"Could not read whale trajectory: {error}")
        return 0

    if not north:
        return 0

    ax.plot(
        east, north, color=WHALE_COLOR, linestyle="--",
        linewidth=1.0, alpha=0.55,
        label="Whale trajectory", zorder=1,
    )
    ax.scatter(
        east[0], north[0], color=WHALE_COLOR,
        marker="x", s=25, alpha=0.7, zorder=2,
    )
    return 1


def plot_whale_calls(ax, detections_by_call: dict[int, list[tuple[str, float, float]]]) -> int:
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
                try:
                    cycle = int(row["cycle"])
                    call_north = float(row["north"])
                    call_east = float(row["east"])
                except (TypeError, ValueError):
                    continue

                if not np.isfinite(call_north) or not np.isfinite(call_east):
                    continue

                if cycle not in detections_by_call:
                    north.append(call_north)
                    east.append(call_east)

    except (OSError, ValueError) as error:
        print(f"Could not read whale calls: {error}")
        return 0

    if north:
        ax.scatter(east, north, color=WHALE_CALL_COLOR, marker=".", s=12, alpha=0.5, label="Undetected whale calls", zorder=5)

    print(f"Plotting {len(north)} undetected whale calls.")
    return len(north)


def plot_detection_locations(ax, detections_by_call: dict[int, list[tuple[str, float, float]]]) -> int:
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
    """Plot mission results and report separate distance metrics for all AUVs."""
    detections_by_call = read_whale_detections()
    latest = find_latest_vehicle_results()
    distance_labels = []

    for name in sorted(latest):
        try:
            integrated_distance, mean_distance = compute_tracking_error(latest, name)
        except (OSError, ValueError) as error:
            print(f"Could not calculate distance for {name}: {error}")
            distance_labels.append(f"{name}: unavailable")
            continue

        print(f"{name} mean distance error: {mean_distance / 1000.0:.3f} km")
        print(f"{name} integrated distance error: {integrated_distance:.3f} km·h")
        distance_labels.append(f"{name}: {mean_distance / 1000.0:.2f} km")

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
    if distance_labels:
        title += "\nMean distance error — " + "; ".join(distance_labels)

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