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
    parser = argparse.ArgumentParser(description="Run, sweep, and review the MANTA trailing mission.")
    commands = parser.add_subparsers(dest="command", required=True)

    commands.add_parser("run", help="Run the simulation.")

    sweep_parser = commands.add_parser("sweep", help="Run a trailing experiment sweep.")
    sweep_parser.add_argument("experiment", choices=("comms",))

    review_parser = commands.add_parser("review", help="Review simulation results.")
    review_commands = review_parser.add_subparsers(dest="plot", required=True)

    review_commands.add_parser("tracks", help="Plot vehicle and whale tracks.")

    review_sweep_parser = review_commands.add_parser("sweep", help="Plot sweep results.")
    review_sweep_parser.add_argument("experiment", choices=("comms",))

    args = parser.parse_args()
    config_path = ROOT / "missions" / "trailing" / "config.yaml"

    try:
        config = load_yaml(config_path)

        if config["scenario"]["type"] != "trailing":
            raise ValueError("Expected a trailing scenario.")

        if args.command == "sweep":
            # Run the selected experiment with the current Python interpreter.
            sweep_scripts = {
                "comms": "trailing_sweep_comms.py",
            }
            sweep_path = ROOT / "missions" / "trailing" / "scripts" / sweep_scripts[args.experiment]

            if not sweep_path.is_file():
                raise ValueError(f"Sweep script does not exist: {sweep_path}")

            command = [sys.executable, str(sweep_path), str(config_path)]
            return subprocess.run(command, cwd=ROOT, check=False).returncode

        if args.command == "review":
            # Select the configured track or sweep plotting script.
            review_key = f"sweep_{args.experiment}" if args.plot == "sweep" else "tracks"
            plot_path = (ROOT / config["review"][review_key]).resolve()

            if not plot_path.is_relative_to(ROOT):
                raise ValueError("Review script must be inside MANTA.")
            if not plot_path.is_file():
                raise ValueError(f"Review script does not exist: {plot_path}")

            command = [sys.executable, str(plot_path)]
            return subprocess.run(command, cwd=ROOT, check=False).returncode

        # Prepare services only for a normal simulation run.
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