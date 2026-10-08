import subprocess
import sys

from components.manta_core.scripts.manta_build import run_build
from manta_core.manta_config import load_yaml, parse_args
from components.manta_core.scripts.manta_down import run_down
from components.manta_core.scripts.manta_environment import get_host_environment
from manta_core.manta_monte_carlo import run_monte_carlo
from components.manta_core.scripts.manta_review import run_review
from components.manta_core.scripts.manta_missions import generate_missions
from manta_core.manta_run import prepare_runtime
from manta_core.manta_sweep import run_monte_carlo_sweep
from components.manta_core.scripts.paths import ROOT, DEV_COMPOSE_PATH, DOCKER_COMPOSE_PATH, MISSION_DIR

def main() -> int:
    args = parse_args()

    if args.command == "review": return run_review(scenario=args.scenario, plot=args.plot)

    try:
        if args.command == "build": return run_build()

        if args.command == "dev":
            command = ["docker", "compose", "-f", str(DEV_COMPOSE_PATH), "run", "--rm", "dev"]
            return subprocess.run(command, cwd=ROOT, env=get_host_environment(), check=False).returncode

        config_path = MISSION_DIR / args.scenario / "config.yaml"
        config = load_yaml(config_path)

        if args.command == "monte_carlo_sweep":
            return run_monte_carlo_sweep(config)

        if args.command == "generate":
            generate_missions(config, config_path)
            return 0

        prepare_runtime(config, config_path)

        if args.command == "monte_carlo":
            output_dir = ROOT / "output" / args.scenario / "monte_carlo"
            return run_monte_carlo(config, output_dir)

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