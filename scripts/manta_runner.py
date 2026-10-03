import subprocess
import sys

from manta_core.manta_build import run_build
from manta_core.manta_config import load_yaml, parse_args
from manta_core.manta_down import run_down
from manta_core.manta_environment import get_host_environment
from manta_core.manta_monte_carlo import run_monte_carlo
from manta_core.manta_review import run_review
from manta_core.manta_run import generate_missions, prepare_runtime
from manta_core.manta_sweep import run_monte_carlo_sweep
from paths import ROOT, DEV_COMPOSE_PATH, DOCKER_COMPOSE_PATH, PICKET_LINE_CONFIG_PATH, PICKET_LINE_MONTE_CARLO_OUTPUT_DIR

def main() -> int:
    args = parse_args()

    scenario_config_paths = {
        "picket_line": PICKET_LINE_CONFIG_PATH,
    }

    scenario_monte_carlo_output_dirs = {
        "picket_line": PICKET_LINE_MONTE_CARLO_OUTPUT_DIR,
    }

    if args.command == "down": return run_down()
    if args.command == "review": return run_review(scenario=args.scenario, plot=args.plot)

    try:
        if args.command == "build": return run_build()

        if args.command == "dev":
            command = ["docker", "compose", "-f", str(DEV_COMPOSE_PATH), "run", "--rm", "dev"]
            return subprocess.run(command, cwd=ROOT, env=get_host_environment(), check=False).returncode

        config_path = scenario_config_paths.get(args.scenario, PICKET_LINE_CONFIG_PATH)
        config = load_yaml(config_path)

        if args.command == "monte_carlo_sweep": return run_monte_carlo_sweep(config)

        prepare_runtime(config, config_path)
        if args.command == "generate": return 0
        if args.command == "monte_carlo": return run_monte_carlo(config, scenario_monte_carlo_output_dirs[args.scenario])

        commands = {
            "run": ["docker", "compose", "-f", str(DOCKER_COMPOSE_PATH), "up", "--remove-orphans"],
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