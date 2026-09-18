import argparse

from pathlib import Path


def parse_args(root: Path) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Build, run, review, and analyze MANTA simulations.")
    parser.add_argument("command", nargs="?", default="run",
        choices=("generate", "build", "dev", "run", "down", "review", "monte-carlo", "monte-carlo-sweep")
    )
    parser.add_argument("plot", nargs="?", default="tracks", choices=("tracks", "energy", "sweep",),
        help="Plot to show with the review command.",
    )

    parser.add_argument(
        "-c",
        "--config",
        default=str(root / "config" / "simulation.yaml"),
        help="Simulation configuration file.",
    )

    return parser.parse_args()