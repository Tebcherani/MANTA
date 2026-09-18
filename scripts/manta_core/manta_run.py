import subprocess
import sys

from pathlib import Path
from paths import ROOT, MISSION_GENERATOR_PATH


def generate_missions(config: dict, config_path: Path) -> None:
    """Generate mission file for the picket line scenario with the given configuration."""
    scenario = config.get("scenario", {})
    if scenario.get("type") != "picket_line": return
    result = subprocess.run([sys.executable, str(MISSION_GENERATOR_PATH), str(config_path)], cwd=ROOT, check=False)
    if result.returncode != 0: raise ValueError("Failed to generate picket-line missions.")


def print_config_summary(config: dict, path: Path) -> None:
    print(f"Loaded configuration: {path}")
    print(f"Configured vehicles: {len(config['vehicles'])}")

    for vehicle in config["vehicles"]:
        print(f"  - {vehicle['id']} ({vehicle['type']})")