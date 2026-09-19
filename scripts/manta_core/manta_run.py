import subprocess
import sys

from pathlib import Path
from paths import ROOT, CONFIG_PATH, MISSION_GENERATOR_PATH

from manta_runner_old import write_runtime_compose, write_runtime_env


def generate_missions(config: dict, config_path: Path) -> None:
    """Generate mission file for the picket line scenario with the given configuration."""
    scenario = config.get("scenario", {})
    if scenario.get("type") != "picket_line": return
    result = subprocess.run([sys.executable, str(MISSION_GENERATOR_PATH), str(config_path)], cwd=ROOT, check=False)
    if result.returncode != 0: raise ValueError("Failed to generate picket-line missions.")


def prepare_runtime(sim_config: dict) -> tuple[Path, Path]:
    """Generate missions, runtime environment, and Docker Compose configuration."""

    generate_missions(sim_config, CONFIG_PATH)
    print_config_summary(sim_config)

    env_path = write_runtime_env(sim_config, ROOT)
    compose_path = write_runtime_compose(sim_config, ROOT)

    print(f"Generated environment: {env_path}")
    print(f"Generated Compose file: {compose_path}")

    return env_path, compose_path


def print_config_summary(config: dict) -> None:
    print(f"Loaded configuration: {CONFIG_PATH}")
    print(f"Configured vehicles: {len(config['vehicles'])}")

    for vehicle in config["vehicles"]:
        print(f"  - {vehicle['id']} ({vehicle['type']})")