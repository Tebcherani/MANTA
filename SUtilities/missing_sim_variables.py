from __future__ import annotations

import argparse
import csv
from pathlib import Path
import sys

from auv_mission_tools import (
    default_behaviors_file,
    default_vehicle_h,
    field_sensor_variables_from_obd,
    iter_input_files,
    parse_behavior_defs,
    parse_vehicle_variables,
    semicolon_join,
)


FIELDNAMES = (
    "obd_file",
    "header_terminated",
    "field_variable_count",
    "missing_variable_count",
    "missing_variables",
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Find field *.obd header variables that are not present in the "
            "current simulator vehicle.h."
        )
    )
    parser.add_argument("folder", type=Path, help="Folder containing *.obd files.")
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
        help="CSV output path. Default: <folder>/missing_sim_variables.csv. Use '-' for stdout.",
    )
    parser.add_argument(
        "--recursive",
        action="store_true",
        help="Search subfolders for *.obd files.",
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
    obd_files = iter_input_files(folder, ".obd", recursive=args.recursive)

    if not obd_files:
        print("No .obd files found in {0}".format(folder))
        return 1

    rows = []
    files_with_missing = 0

    for obd_path in obd_files:
        field_variables, header_terminated = field_sensor_variables_from_obd(
            obd_path,
            behavior_defs,
        )
        missing = [name for name in field_variables if name not in vehicle_variables]
        if missing:
            files_with_missing += 1

        rows.append(
            {
                "obd_file": obd_path.name,
                "header_terminated": int(header_terminated),
                "field_variable_count": len(field_variables),
                "missing_variable_count": len(missing),
                "missing_variables": semicolon_join(missing),
            }
        )

    output = str(args.output) if args.output else str(folder / "missing_sim_variables.csv")
    write_rows(rows, output)

    if output != "-":
        print("Wrote {0}".format(output))
    print(
        "Scanned {0} OBD files; {1} have missing variables".format(
            len(obd_files), files_with_missing
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
