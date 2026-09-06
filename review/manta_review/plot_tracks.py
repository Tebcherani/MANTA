from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from parse_obd import read_obd


def vehicle_id(path: Path) -> str | None:
    if "_S" not in path.stem:
        return None
    return path.stem.rsplit("_S", 1)[0]


def main() -> int:
    root = Path(__file__).resolve().parents[2]
    files = (root / "output" / "Data").rglob("*.obd")
    latest = {}

    for path in files:
        name = vehicle_id(path)

        if name and (name not in latest or path.stat().st_mtime > latest[name].stat().st_mtime):
            latest[name] = path

    if not latest:
        print("No vehicle results found in output/Data.")
        return 1

    fig, ax = plt.subplots(figsize=(9, 7))

    for name, path in sorted(latest.items()):
        print(f"Plotting {name}: {path.name}")

        try:
            ds = read_obd(path)
        except Exception as error:
            print(f"Skipping {name}: {error}")
            continue

        if "real_x" in ds and "real_y" in ds:
            north, east = ds["real_x"].values, ds["real_y"].values
        elif "m_pos_n" in ds and "m_pos_e" in ds:
            north, east = ds["m_pos_n"].values, ds["m_pos_e"].values
        else:
            print(f"Skipping {name}: no position data.")
            continue

        valid = np.isfinite(north) & np.isfinite(east)
        north, east = north[valid], east[valid]

        if not north.size:
            print(f"Skipping {name}: no valid positions.")
            continue

        line, = ax.plot(east, north, linewidth=1, label=name)
        color = line.get_color()
        ax.scatter(east[0], north[0], color=color, marker="o", s=40)
        ax.scatter(east[-1], north[-1], color=color, marker="s", s=40)

    ax.set_title("Latest Vehicle Tracks")
    ax.set_xlabel("East (m)")
    ax.set_ylabel("North (m)")
    ax.set_aspect("equal", adjustable="datalim")
    ax.grid(True, alpha=0.3)
    ax.legend()
    fig.tight_layout()
    plt.show()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())