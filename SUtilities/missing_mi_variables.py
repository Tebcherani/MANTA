from __future__ import annotations

import argparse
import csv
from pathlib import Path
import sys

from auv_mission_tools import (
    default_behaviors_file,
    default_vehicle_h,
    iter_input_files,
    parse_behavior_defs,
    parse_vehicle_variables,
    scan_mission_file,
    semicolon_join,
)


FIELDNAMES = (
    "mission_file",
    "sensor_count",
    "missing_sensor_count",
    "missing_sensors",
    "behavior_count",
    "missing_behavior_count",
    "missing_behaviors",
    "b_arg_count",
    "missing_b_arg_count",
    "missing_b_args",
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Find *.mi sensors, behaviors, and behavior arguments that the "
            "current simulator does not define."
        )
    )
    parser.add_argument("folder", type=Path, help="Folder containing *.mi files.")
    parser.add_argument(
        "--vehicle-h",
        type=Path,
        default=default_vehicle_h(),
        help="Path to simulator vehicle.h.",
    )
    parser.add_argument(
        "--behaviors",
        type=Path,
        default=default_behaviors_file(),
        help="Path to generated simulator behaviors file.",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=None,
        help="CSV output path. Default: <folder>/missing_mi_variables.csv. Use '-' for stdout.",
    )
    parser.add_argument(
        "--recursive",
        action="store_true",
        help="Search subfolders for *.mi files.",
    )
    return parser.parse_args()


def write_rows(rows: list[dict], output) -> None:
    if output == "-":
        writer = csv.DictWriter(sys.stdout, fieldnames=FIELDNAMES, lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)
        return

    output = Path(output)
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=FIELDNAMES, lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)


def main() -> int:
    args = parse_args()
    folder = args.folder

    if not folder.is_dir():
        print("ERROR: folder does not exist: {0}".format(folder), file=sys.stderr)
        return 2

    vehicle_variables = parse_vehicle_variables(args.vehicle_h)
    behavior_defs = parse_behavior_defs(args.behaviors)
    mi_files = iter_input_files(folder, ".mi", recursive=args.recursive)

    if not mi_files:
        print("No .mi files found in {0}".format(folder))
        return 1

    rows = []
    files_with_missing = 0

    for mi_path in mi_files:
        scan = scan_mission_file(mi_path, vehicle_variables, behavior_defs)
        has_missing = bool(
            scan.missing_sensors or scan.missing_behaviors or scan.missing_b_args
        )
        if has_missing:
            files_with_missing += 1

        rows.append(
            {
                "mission_file": mi_path.name,
                "sensor_count": scan.sensor_count,
                "missing_sensor_count": len(scan.missing_sensors),
                "missing_sensors": semicolon_join(scan.missing_sensors),
                "behavior_count": scan.behavior_count,
                "missing_behavior_count": len(scan.missing_behaviors),
                "missing_behaviors": semicolon_join(scan.missing_behaviors),
                "b_arg_count": scan.b_arg_count,
                "missing_b_arg_count": len(scan.missing_b_args),
                "missing_b_args": semicolon_join(scan.missing_b_args),
            }
        )

    output = str(args.output) if args.output else str(folder / "missing_mi_variables.csv")
    write_rows(rows, output)

    if output != "-":
        print("Wrote {0}".format(output))
    print(
        "Scanned {0} mission files; {1} have missing entries".format(
            len(mi_files), files_with_missing
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
