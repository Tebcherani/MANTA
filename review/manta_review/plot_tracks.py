#!/usr/bin/env python3

import csv
import matplotlib.pyplot as plt
import numpy as np

from collections import defaultdict
from pathlib import Path
from parse_obd import read_obd


ROOT = Path(__file__).resolve().parents[2]
DATA_DIR = ROOT / "output" / "Data"
WHALE_CALLS_PATH = DATA_DIR / "whale_calls.csv"
WHALE_DETECTIONS_PATH = DATA_DIR / "whale_detections.csv"
WHALE_TRAJECTORY_PATH = DATA_DIR / "whales.csv"


def find_latest_vehicle_results() -> dict[str, Path]:
    """Find the newest Odyssey result for each vehicle."""

    latest = {}

    for path in DATA_DIR.rglob("*.obd"):
        name = vehicle_id(path)
        if name and (name not in latest or path.stat().st_mtime > latest[name].stat().st_mtime):
            latest[name] = path

    return latest


def plot_vehicle_tracks(ax, latest: dict[str, Path]) -> int:
    """Plot the latest trajectory from each Odyssey vehicle."""

    plotted = 0

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
        ax.scatter(east[0], north[0], color=color, marker="o", s=40, zorder=6)  # Vehicle starting position
        ax.scatter(east[-1], north[-1], color=color, marker="s", s=40, zorder=6)    # Vehicle ending position

        plotted += 1

    return plotted


def read_whale_detections() -> tuple[set[str], list[float], list[float]]:
    """Read detected whale IDs and detection locations."""

    detected_whales = set()
    detection_north = []
    detection_east = []

    if not WHALE_DETECTIONS_PATH.is_file():
        print(f"No whale detection file found: {WHALE_DETECTIONS_PATH}")
        return detected_whales, detection_north, detection_east

    try:
        with WHALE_DETECTIONS_PATH.open("r", encoding="utf-8", newline="") as file:
            reader = csv.DictReader(file)
            required = {"whale_id", "whale_north", "whale_east"}

            if not reader.fieldnames or not required.issubset(reader.fieldnames):
                raise ValueError("whale_detections.csv must contain whale_id, whale_north, and whale_east columns.")

            for row in reader:
                whale_id = row.get("whale_id", "")

                try:
                    north = float(row["whale_north"])
                    east = float(row["whale_east"])
                except (TypeError, ValueError):
                    continue

                if not whale_id or not np.isfinite(north) or not np.isfinite(east):
                    continue

                detected_whales.add(whale_id)
                detection_north.append(north)
                detection_east.append(east)

    except (OSError, ValueError) as error:
        print(f"Could not read whale detections: {error}")

    return detected_whales, detection_north, detection_east


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


def plot_whale_tracks(ax, detected_whales: set[str]) -> int:
    """Plot detected whales in blue and undetected whales in gray."""

    if not WHALE_TRAJECTORY_PATH.is_file():
        print(f"No whale trajectory file found: {WHALE_TRAJECTORY_PATH}")
        return 0

    try:
        tracks = read_whale_tracks(WHALE_TRAJECTORY_PATH)
    except (OSError, ValueError) as error:
        print(f"Could not read whale trajectories: {error}")
        return 0

    plotted = 0
    detected_label_used = False
    undetected_label_used = False

    for whale_id, (north, east) in sorted(tracks.items()):
        if not north:
            continue

        detected = whale_id in detected_whales

        if detected:
            color = "tab:blue"
            linewidth = 1.5
            alpha = 0.9
            zorder = 3
            label = None if detected_label_used else "Detected whales"
            detected_label_used = True
        else:
            color = "tab:gray"
            linewidth = 1.0
            alpha = 0.45
            zorder = 1
            label = None if undetected_label_used else "Undetected whales"
            undetected_label_used = True

        print(f"Plotting whale {whale_id}: {'detected' if detected else 'undetected'}")
        ax.plot(east, north, color=color, linestyle="--", linewidth=linewidth, alpha=alpha, label=label, zorder=zorder)
        ax.scatter(east[0], north[0], color=color, marker="x", s=25, alpha=alpha, zorder=zorder + 1)   # Whale starting position
        plotted += 1

    return plotted


def plot_whale_calls(ax) -> int:
    """Plot every generated whale call as a red dot."""

    if not WHALE_CALLS_PATH.is_file():
        print(f"No whale call file found: {WHALE_CALLS_PATH}")
        return 0

    north = []
    east = []

    try:
        with WHALE_CALLS_PATH.open("r", encoding="utf-8", newline="") as file:
            reader = csv.DictReader(file)
            required = {"north", "east"}

            if not reader.fieldnames or not required.issubset(reader.fieldnames):
                raise ValueError("whale_calls.csv must contain north and east columns.")

            for row in reader:
                try:
                    call_north = float(row["north"])
                    call_east = float(row["east"])
                except (TypeError, ValueError):
                    continue

                if np.isfinite(call_north) and np.isfinite(call_east):
                    north.append(call_north)
                    east.append(call_east)

    except (OSError, ValueError) as error:
        print(f"Could not read whale calls: {error}")
        return 0

    if not north:
        print("No valid whale calls found.")
        return 0

    print(f"Plotting {len(north)} whale calls.")
    ax.scatter(east, north, color="tab:red", marker=".", s=12, alpha=0.5, label="Whale calls", zorder=5)

    return len(north)


def plot_detection_locations(ax, north: list[float], east: list[float]) -> int:
    """Mark the locations where AUVs detected whale calls."""

    if not north:
        print("No whale detections found.")
        return 0

    print(f"Plotting {len(north)} whale detections.")
    ax.scatter(east, north, color="tab:blue", edgecolors="white", marker="D", s=35, linewidths=0.6, label="Detections", zorder=7)

    return len(north)


def main() -> int:
    detected_whales, detection_north, detection_east = read_whale_detections()
    latest = find_latest_vehicle_results()

    fig, ax = plt.subplots(figsize=(10, 8))
    fleet_size = plot_vehicle_tracks(ax, latest)
    whale_count = plot_whale_tracks(ax, detected_whales)
    call_count = plot_whale_calls(ax)
    detection_count = plot_detection_locations(ax, detection_north, detection_east)

    if (fleet_size == 0 and whale_count == 0 and call_count == 0 and detection_count == 0):
        print("No simulation results found in output/Data.")
        plt.close(fig)
        return 1

    print(f"Detected {len(detected_whales)} unique whales.")

    ax.set_title(f"Vehicle and Whale Tracks — {len(detected_whales)} Whales Detected")
    ax.set_xlabel("East (m)")
    ax.set_ylabel("North (m)")
    ax.set_aspect("equal", adjustable="datalim")
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize="small", ncols=2)
    fig.tight_layout()
    plt.show()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())