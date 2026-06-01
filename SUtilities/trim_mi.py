from __future__ import annotations

import argparse
from pathlib import Path
import sys

from auv_mission_tools import (
    default_behaviors_file,
    default_vehicle_h,
    iter_input_files,
    parse_behavior_defs,
    parse_vehicle_variables,
    semicolon_join,
    trim_mission_text,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Create NAME_trim.mi files with unsupported simulator entries removed."
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
        "--recursive",
        action="store_true",
        help="Search subfolders for *.mi files.",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Report what would be written without creating files.",
    )
    parser.add_argument(
        "--no-overwrite",
        action="store_true",
        help="Skip an output file if it already exists.",
    )
    parser.add_argument(
        "--keep-missing-behaviors",
        action="store_true",
        help="Keep behavior blocks that are not in the simulator behavior table.",
    )
    parser.add_argument(
        "--keep-missing-b-args",
        action="store_true",
        help="Keep unsupported b_arg lines for known behaviors.",
    )
    return parser.parse_args()


def trim_path(mi_path: Path) -> Path:
    return mi_path.with_name(mi_path.stem + "_trim.mi")


def main() -> int:
    args = parse_args()
    folder = args.folder

    if not folder.is_dir():
        print("ERROR: folder does not exist: {0}".format(folder), file=sys.stderr)
        return 2

    vehicle_variables = parse_vehicle_variables(args.vehicle_h)
    behavior_defs = parse_behavior_defs(args.behaviors)
    mi_files = [
        p
        for p in iter_input_files(folder, ".mi", recursive=args.recursive)
        if not p.stem.lower().endswith("_trim")
    ]

    if not mi_files:
        print("No source .mi files found in {0}".format(folder))
        return 1

    written = 0
    skipped = 0

    for mi_path in mi_files:
        out_path = trim_path(mi_path)
        if args.no_overwrite and out_path.exists():
            print("SKIP: {0} exists".format(out_path))
            skipped += 1
            continue

        result = trim_mission_text(
            mi_path.read_text(encoding="latin-1"),
            vehicle_variables,
            behavior_defs,
            drop_missing_behaviors=not args.keep_missing_behaviors,
            drop_missing_b_args=not args.keep_missing_b_args,
        )

        removed_total = (
            len(result.removed_sensors)
            + len(result.removed_behaviors)
            + len(result.removed_b_args)
        )

        if args.dry_run:
            print(
                "DRY RUN: {0} -> {1} ({2} entries removed)".format(
                    mi_path.name, out_path.name, removed_total
                )
            )
        else:
            out_path.write_text(result.text, encoding="latin-1")
            print(
                "Wrote {0} ({1} entries removed)".format(out_path, removed_total)
            )

        if result.removed_sensors:
            print("  sensors: {0}".format(semicolon_join(result.removed_sensors)))
        if result.removed_behaviors:
            print("  behaviors: {0}".format(semicolon_join(result.removed_behaviors)))
        if result.removed_b_args:
            print("  b_args: {0}".format(semicolon_join(result.removed_b_args)))

        written += 1

    print("Done: {0} processed, {1} skipped".format(written, skipped))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
