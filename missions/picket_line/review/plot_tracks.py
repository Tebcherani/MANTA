#!/usr/bin/env python3

import csv
import matplotlib.pyplot as plt
import numpy as np

from collections import defaultdict
from pathlib import Path
from parse_obd import read_obd

ROOT = Path(__file__).resolve().parents[3]
DATA_DIR = ROOT / "output" / "picket_line" / "Data"
WHALE_CALLS_PATH = DATA_DIR / "whale_calls.csv"
WHALE_DETECTIONS_PATH = DATA_DIR / "whale_detections.csv"
WHALE_TRAJECTORY_PATH = DATA_DIR / "whales.csv"

WHALE_COLOR = "tab:gray"
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

        line, = ax.plot(east, north, linewidth=1.5, label=name, zorder=4)
        color = line.get_color()
        vehicle_colors[name] = color
        ax.scatter(east[0], north[0], color=color, marker="o", s=40, zorder=6)  # Vehicle starting position
        ax.scatter(east[-1], north[-1], color=color, marker="s", s=40, zorder=6)    # Vehicle ending position

        plotted += 1

    return plotted, vehicle_colors


def read_whale_detections() -> tuple[set[str], dict[tuple[int, str], list[tuple[str, float, float]]]]:
    """Read detections and group them by simulation cycle and whale ID."""

    detected_whales = set()
    detections_by_call = defaultdict(list)

    if not WHALE_DETECTIONS_PATH.is_file():
        print(f"No whale detection file found: {WHALE_DETECTIONS_PATH}")
        return detected_whales, {}

    try:
        with WHALE_DETECTIONS_PATH.open("r", encoding="utf-8", newline="") as file:
            reader = csv.DictReader(file)
            required = {"cycle", "vehicle_id", "whale_id", "whale_north_m", "whale_east_m"}

            if not reader.fieldnames or not required.issubset(reader.fieldnames):
                raise ValueError("whale_detections.csv must contain cycle, vehicle_id, whale_id, whale_north_m, and whale_east_m columns.")

            for row in reader:
                vehicle_id = row.get("vehicle_id", "")
                whale_id = row.get("whale_id", "")

                try:
                    cycle = int(row["cycle"])
                    north = float(row["whale_north_m"])
                    east = float(row["whale_east_m"])
                except (TypeError, ValueError):
                    continue

                if not vehicle_id or not whale_id or not np.isfinite(north) or not np.isfinite(east):
                    continue

                detected_whales.add(whale_id)
                detections_by_call[(cycle, whale_id)].append((vehicle_id, north, east))

    except (OSError, ValueError) as error:
        print(f"Could not read whale detections: {error}")

    return detected_whales, dict(detections_by_call)


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


def plot_detection_locations(ax, detections_by_call: dict[tuple[int, str], list[tuple[str, float, float]]], vehicle_colors: dict[str, str]) -> int:
    """Mark each detection using the detecting vehicle's color."""

    detections_by_vehicle = defaultdict(lambda: ([], []))

    for detections in detections_by_call.values():
        for vehicle_id, north, east in detections:
            vehicle_north_m, vehicle_east_m = detections_by_vehicle[vehicle_id]
            vehicle_north_m.append(north)
            vehicle_east_m.append(east)

    detection_count = sum(len(north) for north, _ in detections_by_vehicle.values())

    if detection_count == 0:
        print("No whale detections found.")
        return 0

    for vehicle_id, (north, east) in sorted(detections_by_vehicle.items()):
        color = vehicle_colors.get(vehicle_id, "tab:blue")
        ax.scatter(east, north, color=color, marker="D", s=45, linewidths=0.5, label=f"{vehicle_id} detections", zorder=7)
    
    print(f"Plotting {detection_count} whale detections.")
    return detection_count


def main() -> int:
    detected_whales, detections_by_call = read_whale_detections()
    latest = find_latest_vehicle_results()

    fig, ax = plt.subplots(figsize=(10, 8))
    fleet_size, vehicle_colors = plot_vehicle_tracks(ax, latest)
    whale_count = plot_whale_tracks(ax)
    call_count = plot_whale_calls(ax, detections_by_call)
    detection_count = plot_detection_locations(ax, detections_by_call, vehicle_colors)

    if (fleet_size == 0 and whale_count == 0 and call_count == 0 and detection_count == 0):
        print("No simulation results found in output/picket_line/Data.")
        plt.close(fig)
        return 1

    print(f"Detected {len(detected_whales)} unique whales.")

    ax.set_title(f"Vehicle and Whale Tracks — {len(detected_whales)} Whales Detected")
    ax.set_xlabel("East (m)")
    ax.set_ylabel("North (m)")
    ax.set_aspect("equal", adjustable="box")
    ax.margins(x=0.03, y=0.05)
    ax.grid(True, alpha=0.3)
    ax.legend(ncols=2)
    fig.tight_layout()
    plt.show()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())