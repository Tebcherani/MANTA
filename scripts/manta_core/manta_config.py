import argparse
import yaml

from pathlib import Path
from paths import COMPONENT_DIR


def load_plugin(plugin_id: str) -> dict:
    """Load a vehicle plugin manifest and verify its ID and plugin type."""

    path = COMPONENT_DIR / f"manta_{plugin_id}" / "manta_plugin.yaml"

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
    """Parse the MANTA command, scenario, and review plot."""

    parser = argparse.ArgumentParser(description="Build, run, review, and analyze MANTA simulations.")
    parser.add_argument("command", nargs="?", default="run", choices=("generate", "build", "dev", "run", "down", "review", "monte_carlo", "monte_carlo_sweep"))
    parser.add_argument("scenario", nargs="?", choices=("picket_line", "trailing"), help="Simulation scenario.")
    parser.add_argument("plot", nargs="?", choices=("tracks", "sweep"), help="Scenario plot to show.")

    args = parser.parse_args()

    scenario_commands = {"generate", "run", "monte_carlo", "monte_carlo_sweep"}

    if args.command in scenario_commands and args.scenario is None:
        parser.error(f"{args.command} requires a scenario: manta {args.command} picket_line")

    if args.command == "review" and (args.scenario is None or args.plot is None):
        parser.error("review requires a scenario and plot: manta review picket_line {tracks,sweep}")

    scenario_independent_commands = {"build", "dev", "down"}

    if args.command in scenario_independent_commands and args.scenario is not None:
        parser.error(f"{args.command} does not accept a scenario: manta {args.command}")

    if args.command != "review" and args.plot is not None:
        parser.error("a plot can only be selected with the review command")

    return args