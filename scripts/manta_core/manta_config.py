import argparse
import yaml

from pathlib import Path
from paths import VEHICLE_DIR


def load_plugin(plugin_id: str) -> dict:
    """Load a vehicle plugin manifest and verify its ID and plugin type."""

    path = VEHICLE_DIR / plugin_id / "manta_plugin.yaml"

    if not path.is_file(): raise ValueError(f"Vehicle plugin '{plugin_id}' was not found: {path}")

    manifest = load_yaml(path)
    plugin = manifest.get("plugin")

    if not isinstance(plugin, dict): raise ValueError(f"Plugin '{plugin_id}' has no 'plugin' section.")
    if plugin.get("id") != plugin_id: raise ValueError(f"Plugin manifest ID '{plugin.get('id')}' does not match configured type '{plugin_id}'.")
    if plugin.get("kind") != "vehicle": raise ValueError(f"Plugin '{plugin_id}' is not a vehicle plugin.")

    return manifest


def load_yaml(path: Path) -> dict:
    """Load a YAML file and require a top-level mapping."""

    if not path.is_file(): raise ValueError(f"YAML file does not exist: {path}")

    try:
        with path.open("r", encoding="utf-8") as file: data = yaml.safe_load(file)
    except yaml.YAMLError as error:
        raise ValueError(f"Invalid YAML in {path}: {error}") from error

    if not isinstance(data, dict): raise ValueError(f"YAML file must contain a top-level mapping: {path}")

    return data


def parse_args() -> argparse.Namespace:
    """Parse the MANTA command and optional review-plot selection."""

    parser = argparse.ArgumentParser(description="Build, run, review, and analyze MANTA simulations.")
    parser.add_argument("command", nargs="?", default="run", choices=("generate", "build", "dev", "run", "down", "review", "monte-carlo", "monte-carlo-sweep"))
    parser.add_argument("plot", nargs="?", default="tracks", choices=("tracks", "sweep",), help="Plot to show with the review command.")

    return parser.parse_args()