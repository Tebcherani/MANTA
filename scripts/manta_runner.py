import subprocess
import sys

from pathlib import Path

from manta_runner_old import parse_args, run_review, load_config, print_summary
from manta_runner_old import validate_monte_carlo_config, run_monte_carlo_sweep, generate_scenario_missions
from manta_runner_old import write_runtime_compose, write_runtime_env
from manta_runner_old import run_monte_carlo, run_build


def main() -> int:
    """Parse the command and run the requested MANTA operation."""

    root = Path(__file__).resolve().parent.parent
    args = parse_args(root)

    if args.command == "review":
        return run_review(root, args.plot)

    try:
        config_path = Path(args.config).resolve()
        config = load_config(config_path)

        if args.command == "down":
            compose_path = root / "generated" / "compose.ros.yaml"

            if not compose_path.is_file():
                print("No generated MANTA Compose file exists.")
                return 0

            command = ["docker", "compose", "-f", str(compose_path), "down", "--remove-orphans"]
            return subprocess.run(command, cwd=root, check=False).returncode

        if args.command == "monte-carlo-sweep":
            #validate_vehicle_files(config, root)
            print_summary(config, config_path)

            validate_monte_carlo_config(config)
            sweep = config.sweep

            if not isinstance(sweep, dict):
                raise ValueError("'sweep' must be a YAML mapping.")

            minimum_vehicles = sweep.get("min_vehicles", 1)
            maximum_vehicles = sweep.get("max_vehicles", 10)

            if not isinstance(minimum_vehicles, int) or minimum_vehicles < 1:
                raise ValueError("'sweep.min_vehicles' must be an integer of at least 1.")

            if not isinstance(maximum_vehicles, int) or maximum_vehicles < minimum_vehicles:
                raise ValueError("'sweep.max_vehicles' must be an integer greater than or equal to min_vehicles.")

            return run_monte_carlo_sweep(config, config_path, root, minimum_vehicles=minimum_vehicles, maximum_vehicles=maximum_vehicles)

        generate_scenario_missions(config, config_path, root)
        #validate_vehicle_files(config, root)
        print_summary(config, config_path)

        env_path = write_runtime_env(config, root)
        compose_path = write_runtime_compose(config, root)

        print(f"Generated environment: {env_path}")
        print(f"Generated Compose file: {compose_path}")

        if args.command == "generate":
            return 0

        if args.command == "build":
            return run_build(root, env_path)

        if args.command == "monte-carlo":
            return run_monte_carlo(config, root)

        commands = {
            "dev": ["docker", "compose", "--env-file", str(env_path), "-f", str(root / "compose.dev.yaml"), "run", "--rm", "dev"],
            "run": ["docker", "compose", "-f", str(compose_path), "up", "--remove-orphans"],
        }

        return subprocess.run(commands[args.command], cwd=root, check=False).returncode

    except ValueError as error:
        print(f"MANTA configuration error: {error}", file=sys.stderr)
        return 1

    except FileNotFoundError as error:
        print(f"Required command was not found: {error.filename}", file=sys.stderr)
        return 1

    except KeyboardInterrupt:
        print("\nMANTA command interrupted.", file=sys.stderr)
        return 130


if __name__ == "__main__":
    raise SystemExit(main())