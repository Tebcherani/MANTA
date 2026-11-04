#!/usr/bin/env python3
"""Main command-line entry point for MANTA."""

import argparse
import sys
from pathlib import Path

# Support dependencies still in the original scripts folder.
ROOT = Path(__file__).resolve().parents[3]
sys.path.append(str(ROOT / "scripts"))


def main() -> int:
    parser = argparse.ArgumentParser(description="Manage MANTA simulations.")
    parser.add_argument("command", choices=("down", "build", "docs"))
    if sys.argv[1:2] == ["docs"]:
        from manta_docs import run_docs
        return run_docs(sys.argv[2:])
    args = parser.parse_args()

    if args.command == "down":
        from manta_down import run_down
        return run_down()

    if args.command == "build":
        from manta_build import run_build
        return run_build()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
