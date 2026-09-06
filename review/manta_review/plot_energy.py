from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from parse_obd import read_obd


def vehicle_id(path: Path) -> str | None:
    if "_S" not in path.stem:
        return None
    return path.stem.rsplit("_S", 1)[0]


def latest_results(directory: Path) -> dict[str, Path]:
    latest = {}

    for path in directory.rglob("*.obd"):
        name = vehicle_id(path)

        if name and (name not in latest or path.stat().st_mtime > latest[name].stat().st_mtime):
            latest[name] = path

    return latest


def main() -> int:
    root = Path(__file__).resolve().parents[2]
    latest = latest_results(root / "output" / "Data")

    if not latest:
        print("No vehicle results found in output/Data.")
        return 1

    fig, ax = plt.subplots(figsize=(9, 6))
    plotted = 0

    for name, path in sorted(latest.items()):
        print(f"Plotting {name}: {path.name}")

        try:
            ds = read_obd(path)
        except Exception as error:
            print(f"Skipping {name}: {error}")
            continue

        if "real_battery_energy_used" not in ds:
            print(f"Skipping {name}: real_battery_energy_used was not logged.")
            continue

        if "m_present_time" in ds:
            time = ds["m_present_time"].values.astype(float)
            valid_time = np.isfinite(time)

            if valid_time.any():
                time = time - time[valid_time][0]
        elif "time" in ds:
            time = ds["time"].values.astype(float)
        else:
            print(f"Skipping {name}: no time field.")
            continue

        energy_wh = ds["real_battery_energy_used"].values.astype(float) / 3600.0
        valid = np.isfinite(time) & np.isfinite(energy_wh)

        if not valid.any():
            print(f"Skipping {name}: no valid energy samples.")
            continue

        ax.plot(time[valid], energy_wh[valid], linewidth=1.5, label=name)
        plotted += 1

    if not plotted:
        print("No energy results could be plotted.")
        plt.close(fig)
        return 1

    ax.set_title("Latest Vehicle Energy Consumption")
    ax.set_xlabel("Mission time (s)")
    ax.set_ylabel("Energy consumed (Wh)")
    ax.grid(True, alpha=0.3)
    ax.legend()
    fig.tight_layout()
    plt.show()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())