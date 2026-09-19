import subprocess
import sys

from manta_core.manta_build import run_build
from manta_core.manta_config import load_yaml, parse_args
from manta_core.manta_down import run_down
from manta_core.manta_monte_carlo import run_monte_carlo
from manta_core.manta_review import run_review
from manta_core.manta_run import generate_missions, print_config_summary
from manta_core.manta_sweep import run_monte_carlo_sweep
from paths import ROOT, CONFIG_PATH

from manta_runner_old import write_runtime_compose, write_runtime_env

def main() -> int:
    args = parse_args()

    if args.command == "down": run_down()
    if args.command == "review": return run_review(plot=args.plot)

    try:
        config = load_yaml(CONFIG_PATH)
        if args.command == "monte-carlo-sweep": return run_monte_carlo_sweep(config)

        generate_missions(config, CONFIG_PATH)
        print_config_summary(config, CONFIG_PATH)
        env_path = write_runtime_env(config, ROOT)
        compose_path = write_runtime_compose(config, ROOT)
        print(f"Generated environment: {env_path}")
        print(f"Generated Compose file: {compose_path}")

        if args.command == "generate": return 0
        if args.command == "build": return run_build(env_path)
        if args.command == "monte-carlo": return run_monte_carlo(config, ROOT)

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