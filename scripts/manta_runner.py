import subprocess
import sys

from manta_core.manta_config import load_yaml, parse_args
from manta_core.manta_down import run_down
from manta_core.manta_review import run_review
from paths import ROOT, CONFIG_PATH

from manta_runner_old import validate_monte_carlo_config, run_monte_carlo_sweep, generate_scenario_missions
from manta_runner_old import write_runtime_compose, write_runtime_env, print_summary
from manta_runner_old import run_monte_carlo, run_build


def main() -> int:
    args = parse_args()

    if args.command == "review": return run_review(plot=args.plot)

    try:
        config = load_yaml(CONFIG_PATH)

        if args.command == "down": run_down()

        if args.command == "monte-carlo-sweep":
            #validate_vehicle_files(config, ROOT)
            print_summary(config, CONFIG_PATH)

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

            return run_monte_carlo_sweep(config, CONFIG_PATH, ROOT, minimum_vehicles=minimum_vehicles, maximum_vehicles=maximum_vehicles)

        generate_scenario_missions(config, CONFIG_PATH, ROOT)
        #validate_vehicle_files(config, ROOT)
        print_summary(config, CONFIG_PATH)

        env_path = write_runtime_env(config, ROOT)
        compose_path = write_runtime_compose(config, ROOT)

        print(f"Generated environment: {env_path}")
        print(f"Generated Compose file: {compose_path}")

        if args.command == "generate":
            return 0

        if args.command == "build":
            return run_build(ROOT, env_path)

        if args.command == "monte-carlo":
            return run_monte_carlo(config, ROOT)

        commands = {
            "dev": ["docker", "compose", "--env-file", str(env_path), "-f", str(ROOT / "compose.dev.yaml"), "run", "--rm", "dev"],
            "run": ["docker", "compose", "-f", str(compose_path), "up", "--remove-orphans"],
        }

        return subprocess.run(commands[args.command], cwd=ROOT, check=False).returncode

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