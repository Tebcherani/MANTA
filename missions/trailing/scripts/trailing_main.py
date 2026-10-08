#!/usr/bin/env python3
"""Command-line entry point for the trailing mission."""

import argparse
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]

sys.path.insert(0, str(ROOT / "components" / "manta_core" / "scripts"))

from manta_load import load_yaml


def main() -> int:
    parser = argparse.ArgumentParser(description="Run and review the MANTA trailing mission.")
    commands = parser.add_subparsers(dest="command", required=True)

    commands.add_parser("run", help="Run the simulation.")

    review_parser = commands.add_parser("review", help="Review simulation results.")
    review_parser.add_argument("plot", choices=("tracks",))

    args = parser.parse_args()
    config_path = ROOT / "missions" / "trailing" / "config.yaml"

    try:
        config = load_yaml(config_path)

        if config["scenario"]["type"] != "trailing":
            raise ValueError("Expected a trailing scenario.")

        if args.command == "review":
            # Review existing results without preparing a new simulation.
            plot_path = (ROOT / config["review"][args.plot]).resolve()

            if not plot_path.is_relative_to(ROOT):
                raise ValueError("Review script must be inside MANTA.")
            if not plot_path.is_file():
                raise ValueError(f"Review script does not exist: {plot_path}")

            command = [sys.executable, str(plot_path)]
            return subprocess.run(command, cwd=ROOT, check=False).returncode

        # Load simulation helpers only for the run command.
        from trailing_services import create_services
        from manta_runtime import prepare_runtime

        services = create_services(config)
        compose_path = prepare_runtime(config, config_path, services)
        command = ["docker", "compose", "-f", str(compose_path), "up", "--remove-orphans"]

        return subprocess.run(command, cwd=ROOT, check=False).returncode

    except (ValueError, KeyError) as error:
        print(f"Trailing configuration error: {error}", file=sys.stderr)
        return 1

    except FileNotFoundError as error:
        print(f"Required file or command not found: {error.filename}", file=sys.stderr)
        return 1

    except KeyboardInterrupt:
        print("\nTrailing command interrupted.", file=sys.stderr)
        return 130


if __name__ == "__main__":
    raise SystemExit(main())