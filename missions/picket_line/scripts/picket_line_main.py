#!/usr/bin/env python3
"""Command-line entry point for the picket-line mission."""

import argparse
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]

sys.path.insert(0, str(ROOT / "components" / "manta_core" / "scripts"))

from manta_load import load_yaml


def main() -> int:
    parser = argparse.ArgumentParser(description="Run, sweep, and review the MANTA picket-line mission.")
    commands = parser.add_subparsers(dest="command", required=True)

    commands.add_parser("run", help="Run the simulation.")
    commands.add_parser("sweep", help="Sweep the AUV fleet size.")

    review_parser = commands.add_parser("review", help="Review simulation results.")
    review_parser.add_argument("plot", choices=("tracks", "sweep"))

    args = parser.parse_args()
    config_path = ROOT / "missions" / "picket_line" / "config.yaml"

    try:
        config = load_yaml(config_path)

        if config["scenario"]["type"] != "picket_line":
            raise ValueError("Expected a picket_line scenario.")

        if args.command == "sweep":
            # Run the mission's fleet-size sweep using the same interpreter.
            sweep_path = ROOT / "missions" / "picket_line" / "scripts" / "picket_line_sweep.py"

            if not sweep_path.is_file():
                raise ValueError(f"Sweep script does not exist: {sweep_path}")

            command = [sys.executable, str(sweep_path), str(config_path)]
            return subprocess.run(command, cwd=ROOT, check=False).returncode

        if args.command == "review":
            # Plot existing results without preparing a new simulation.
            plot_path = (ROOT / config["review"][args.plot]).resolve()

            if not plot_path.is_relative_to(ROOT):
                raise ValueError("Review script must be inside MANTA.")
            if not plot_path.is_file():
                raise ValueError(f"Review script does not exist: {plot_path}")

            command = [sys.executable, str(plot_path)]
            return subprocess.run(command, cwd=ROOT, check=False).returncode

        # Load simulation helpers only for the run command.
        from picket_line_services import create_services
        from manta_runtime import prepare_runtime

        services = create_services(config)
        compose_path = prepare_runtime(config, config_path, services)
        command = ["docker", "compose", "-f", str(compose_path), "up", "--remove-orphans"]

        return subprocess.run(command, cwd=ROOT, check=False).returncode

    except (ValueError, KeyError) as error:
        print(f"Picket-line configuration error: {error}", file=sys.stderr)
        return 1

    except FileNotFoundError as error:
        print(f"Required file or command not found: {error.filename}", file=sys.stderr)
        return 1

    except KeyboardInterrupt:
        print("\nPicket-line command interrupted.", file=sys.stderr)
        return 130


if __name__ == "__main__":
    raise SystemExit(main())