from __future__ import annotations

import argparse
from pathlib import Path
import sys

from auv_mission_tools import extract_mission_from_slg_file, iter_input_files


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Extract Odyssey mission scripts from *.slg files."
    )
    parser.add_argument("folder", type=Path, help="Folder containing *.slg files.")
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=None,
        help="Optional folder for generated *.mi files. Default: beside each *.slg.",
    )
    parser.add_argument(
        "--recursive",
        action="store_true",
        help="Search subfolders for *.slg files.",
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
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    folder = args.folder

    if not folder.is_dir():
        print("ERROR: folder does not exist: {0}".format(folder), file=sys.stderr)
        return 2

    slg_files = iter_input_files(folder, ".slg", recursive=args.recursive)
    if not slg_files:
        print("No .slg files found in {0}".format(folder))
        return 1

    written = 0
    skipped = 0
    failed = 0

    if args.output_dir and not args.dry_run:
        args.output_dir.mkdir(parents=True, exist_ok=True)

    for slg_path in slg_files:
        mission_text = extract_mission_from_slg_file(slg_path)
        if not mission_text:
            print("WARNING: no mission block found in {0}".format(slg_path.name))
            failed += 1
            continue

        out_dir = args.output_dir if args.output_dir else slg_path.parent
        out_path = out_dir / (slg_path.stem + ".mi")

        if args.no_overwrite and out_path.exists():
            print("SKIP: {0} exists".format(out_path))
            skipped += 1
            continue

        if args.dry_run:
            print("DRY RUN: {0} -> {1}".format(slg_path, out_path))
        else:
            out_path.write_text(mission_text, encoding="latin-1")
            print("Wrote {0}".format(out_path))
        written += 1

    print(
        "Done: {0} extracted, {1} skipped, {2} failed".format(
            written, skipped, failed
        )
    )
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
