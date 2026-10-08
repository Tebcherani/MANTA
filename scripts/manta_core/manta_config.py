import argparse
import yaml

from pathlib import Path
from paths import COMPONENT_DIR, MISSION_DIR

def get_whale_count(config: dict) -> int:
    """Trailing always uses one whale; other missions configure their count."""
    scenario = config["scenario"]
    whales = scenario["whales"]

    if scenario["type"] == "trailing":
        if "count" in whales:
            raise ValueError(
                "Trailing always uses exactly one whale. "
                "Remove scenario.whales.count from the configuration."
            )
        return 1

    return int(whales["count"])


def parse_args() -> argparse.Namespace:
    """Parse the MANTA command, scenario, and review plot."""

    parser = argparse.ArgumentParser(description="Build, run, review, and analyze MANTA simulations.")
    parser.add_argument("command", nargs="?", default="run", choices=("generate", "build", "dev", "run", "down", "review", "monte_carlo", "monte_carlo_sweep"))

    available_missions = sorted(path.parent.name for path in MISSION_DIR.glob("*/config.yaml"))
    parser.add_argument("scenario", nargs="?", choices=available_missions, help="Mission folder containing config.yaml.")
    
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