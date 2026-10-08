"""Shared runtime preparation for MANTA missions."""

import sys

from manta_compose import write_runtime_compose
from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]

from manta_environment import write_runtime_env
from manta_missions import generate_missions
from paths import ENV_PATH

# Support existing helpers during migration.
sys.path.append(str(ROOT / "scripts"))


def prepare_runtime(config: dict, config_path: Path, services: dict, compose_path: Path | None = None, project_name: str = "manta") -> Path:
    """Generate missions, environment settings, and a Compose file."""
    generate_missions(config, config_path)

    print(f"Loaded configuration: {config_path}")
    print(f"Configured vehicles: {len(config['vehicles'])}")

    for vehicle in config["vehicles"]:
        print(f"  - {vehicle['id']} ({vehicle['type']})")

    write_runtime_env(config)

    compose_path = write_runtime_compose(services, compose_path=compose_path, project_name=project_name)

    print(f"Generated environment: {ENV_PATH}")
    print(f"Generated Compose file: {compose_path}")

    return compose_path