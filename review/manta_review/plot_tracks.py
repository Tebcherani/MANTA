#!/usr/bin/env python3

import csv
from collections import defaultdict
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from parse_obd import read_obd


def vehicle_id(path: Path) -> str | None:
    """Extract the vehicle ID from an Odyssey result filename."""

    if "_S" not in path.stem:
        return None

    return path.stem.rsplit("_S", 1)[0]


def find_latest_vehicle_results(
    data_directory: Path,
) -> dict[str, Path]:
    """Find the newest Odyssey result for each vehicle."""

    latest = {}

    for path in data_directory.rglob("*.obd"):
        name = vehicle_id(path)

        if name and (
            name not in latest
            or path.stat().st_mtime
            > latest[name].stat().st_mtime
        ):
            latest[name] = path

    return latest


def plot_vehicle_tracks(
    ax,
    latest: dict[str, Path],
) -> int:
    """Plot the latest trajectory from each Odyssey vehicle."""

    plotted = 0

    for name, path in sorted(latest.items()):
        print(f"Plotting vehicle {name}: {path.name}")

        try:
            ds = read_obd(path)
        except Exception as error:
            print(f"Skipping {name}: {error}")
            continue

        if "real_x" in ds and "real_y" in ds:
            north = ds["real_x"].values
            east = ds["real_y"].values
        elif "m_pos_n" in ds and "m_pos_e" in ds:
            north = ds["m_pos_n"].values
            east = ds["m_pos_e"].values
        else:
            print(f"Skipping {name}: no position data.")
            continue

        valid = np.isfinite(north) & np.isfinite(east)
        north = north[valid]
        east = east[valid]

        if not north.size:
            print(f"Skipping {name}: no valid positions.")
            continue

        line, = ax.plot(
            east,
            north,
            linewidth=1.5,
            label=name,
            zorder=3,
        )

        color = line.get_color()

        # Circle: vehicle starting position.
        ax.scatter(
            east[0],
            north[0],
            color=color,
            marker="o",
            s=40,
            zorder=5,
        )

        # Square: vehicle ending position.
        ax.scatter(
            east[-1],
            north[-1],
            color=color,
            marker="s",
            s=40,
            zorder=5,
        )

        plotted += 1

    return plotted


def read_whale_tracks(
    path: Path,
) -> dict[str, tuple[list[float], list[float]]]:
    """Read whale trajectories grouped by whale ID."""

    tracks = defaultdict(lambda: ([], []))

    with path.open(
        "r",
        encoding="utf-8",
        newline="",
    ) as file:
        reader = csv.DictReader(file)

        required = {"whale_id", "north", "east"}

        if (
            not reader.fieldnames
            or not required.issubset(reader.fieldnames)
        ):
            raise ValueError(
                "whales.csv must contain whale_id, "
                "north, and east columns."
            )

        for row in reader:
            try:
                north = float(row["north"])
                east = float(row["east"])
            except (TypeError, ValueError):
                continue

            if (
                not np.isfinite(north)
                or not np.isfinite(east)
            ):
                continue

            track_north, track_east = tracks[row["whale_id"]]
            track_north.append(north)
            track_east.append(east)

    return tracks


def plot_whale_tracks(ax, path: Path) -> int:
    """Plot all recorded whale ground-truth trajectories."""

    if not path.is_file():
        print(f"No whale trajectory file found: {path}")
        return 0

    try:
        tracks = read_whale_tracks(path)
    except (OSError, ValueError) as error:
        print(f"Could not read whale trajectories: {error}")
        return 0

    plotted = 0
    whale_color = "tab:gray"

    for whale_id, (north, east) in sorted(tracks.items()):
        if not north:
            continue

        print(f"Plotting whale {whale_id}")

        # Only the first whale receives a label, producing one
        # shared legend entry for every whale trajectory.
        label = "Whales" if plotted == 0 else None

        ax.plot(
            east,
            north,
            color=whale_color,
            linestyle="--",
            linewidth=1,
            alpha=0.6,
            label=label,
            zorder=1,
        )

        # Triangle: whale starting position.
        ax.scatter(
            east[0],
            north[0],
            color=whale_color,
            marker="^",
            s=25,
            alpha=0.8,
            zorder=2,
        )

        # X: whale ending position.
        ax.scatter(
            east[-1],
            north[-1],
            color=whale_color,
            marker="x",
            s=30,
            alpha=0.8,
            zorder=2,
        )

        plotted += 1

    return plotted


def plot_whale_calls(ax, path: Path) -> int:
    """Plot every published whale call as a red marker."""

    if not path.is_file():
        print(f"No whale call file found: {path}")
        return 0

    north = []
    east = []

    try:
        with path.open(
            "r",
            encoding="utf-8",
            newline="",
        ) as file:
            reader = csv.DictReader(file)

            required = {"north", "east"}

            if (
                not reader.fieldnames
                or not required.issubset(reader.fieldnames)
            ):
                raise ValueError(
                    "whale_calls.csv must contain "
                    "north and east columns."
                )

            for row in reader:
                try:
                    call_north = float(row["north"])
                    call_east = float(row["east"])
                except (TypeError, ValueError):
                    continue

                if (
                    np.isfinite(call_north)
                    and np.isfinite(call_east)
                ):
                    north.append(call_north)
                    east.append(call_east)

    except (OSError, ValueError) as error:
        print(f"Could not read whale calls: {error}")
        return 0

    if not north:
        print("No valid whale calls found.")
        return 0

    # All calls share one color and one legend entry.
    ax.scatter(
        east,
        north,
        color="tab:red",
        marker=".",
        s=12,
        alpha=0.55,
        label="Whale calls",
        zorder=4,
    )

    print(f"Plotting {len(north)} whale calls.")
    return len(north)


def main() -> int:
    root = Path(__file__).resolve().parents[2]
    data_directory = root / "output" / "Data"

    fig, ax = plt.subplots(figsize=(10, 8))

    latest = find_latest_vehicle_results(data_directory)

    vehicle_count = plot_vehicle_tracks(
        ax,
        latest,
    )

    whale_count = plot_whale_tracks(
        ax,
        data_directory / "whales.csv",
    )

    call_count = plot_whale_calls(
        ax,
        data_directory / "whale_calls.csv",
    )

    if (
        vehicle_count == 0
        and whale_count == 0
        and call_count == 0
    ):
        print(
            "No vehicle tracks, whale trajectories, "
            "or whale calls found in output/Data."
        )
        plt.close(fig)
        return 1

    ax.set_title(
        "Vehicle Tracks, Whale Trajectories, and Calls"
    )
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