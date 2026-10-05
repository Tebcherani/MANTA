#!/usr/bin/env python3

"""
Generate one Odyssey whale-trailing mission file per configured vehicle.

Each vehicle begins by patrolling an assigned section of the picket line.
A vehicle can leave the line and follow an assigned whale after detecting
its call.
"""


import sys
import yaml

from dataclasses import dataclass
from pathlib import Path
from textwrap import dedent


if len(sys.argv) != 3:
    raise SystemExit(
        f"Usage: {Path(sys.argv[0]).name} <config_path> <output_directory>"
    )

config_path = Path(sys.argv[1]).resolve()
OUTPUT_DIR = Path(sys.argv[2]).resolve()

@dataclass(frozen=True)
class MissionConfig:
    """Configuration values needed to generate trailing missions."""

    vehicles: list[dict]
    start_n: float
    start_e: float
    mission_duration: float
    vehicle_speed: float
    vehicle_depth: float
    vehicle_capture: float

    @property
    def fleet_size(self) -> int:
        return len(self.vehicles)

    def __post_init__(self) -> None:
        """Validate the configuration immediately after construction."""

        if self.fleet_size == 0:
            raise ValueError("At least one vehicle is required.")

        vehicle_ids = [vehicle.get("id") for vehicle in self.vehicles]

        if any(not vehicle_id for vehicle_id in vehicle_ids):
            raise ValueError("Every vehicle must have a nonempty ID.")
        if len(set(vehicle_ids)) != self.fleet_size:
            raise ValueError("Vehicle IDs must be unique.")
        if self.mission_duration <= 0.0:
            raise ValueError("Scenario duration must be greater than zero.")
        if self.vehicle_speed <= 0.0:
            raise ValueError("Trailing speed must be greater than zero.")
        if self.vehicle_depth <= 0.0:
            raise ValueError("Trailing depth must be greater than zero.")
        if self.vehicle_capture <= 0.0:
            raise ValueError("Trailing capture distance must be greater than zero.")


def generate_mission_file(config: MissionConfig, vehicle_id: str) -> Path:
    """Generate and write the trailing mission for one Odyssey."""

    mission = dedent(
        f"""\
        # Generated MANTA trailing mission for {vehicle_id}
        UseSim 1

        # Start at the configured trailing location.
        sensor: m_pos_n(m) {config.start_n:.6f}
        sensor: m_pos_e(m) {config.start_e:.6f}
        sensor: m_depth(m) {config.vehicle_depth:.6f}
        sensor: real_x(m) {config.start_n:.6f}
        sensor: real_y(m) {config.start_e:.6f}
        sensor: real_z(m) {config.vehicle_depth:.6f}

        state: picket_line

        # Stop after the configured scenario duration.
        behavior: mission_timer 1
          b_arg: time(s) {config.mission_duration:.6f}

        # Maintain safe depth limits.
        behavior: depth_envelope 2
          b_arg: max_depth(m) {config.vehicle_depth + 20.0:.6f}
          b_arg: min_depth(m) 0.5
          b_arg: alt_env_active(bool) 0
          b_arg: min_altitude(m) 2.0
          b_arg: depth_cutoff_active(bool) 0
          b_arg: cutoff_depth(m) {config.vehicle_depth + 25.0:.6f}

        # Move toward the most recently detected whale-call location.
        behavior: whale_trailing 3
          b_arg: depth(m) {config.vehicle_depth:.6f}
          b_arg: speed(m/s) {config.vehicle_speed:.6f}
          b_arg: capture(m) {config.vehicle_capture:.6f}
        """
    )

    mission_path = OUTPUT_DIR / f"{vehicle_id}.mi"
    mission_path.write_text(mission, encoding="utf-8")

    return mission_path


def load_mission_config() -> MissionConfig:
    """Load the picket-line mission settings from the MANTA YAML file."""

    # Load configuration file
    with config_path.open("r", encoding="utf-8") as file:
        config = yaml.safe_load(file)

    # Load top-level sections
    scenario = config["scenario"]
    trailing = scenario["trailing"]
    patrol = scenario["patrol"]

    # Load configuration values
    config = MissionConfig(
        vehicles=config.get("vehicles", []),
        start_n=float(trailing["start"]["north"]),
        start_e=float(trailing["start"]["east"]),
        mission_duration=float(scenario["duration"]),
        vehicle_speed=float(patrol["speed"]),
        vehicle_depth=float(patrol["depth"]),
        vehicle_capture=float(patrol.get("capture", 25.0)),
    )

    return config


def main() -> int:
    """Load the MANTA configuration and generate the Odyssey mission files."""

    config = load_mission_config()
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

    # Generate mission file for each vehicle
    for vehicle in config.vehicles:
        mission_path = generate_mission_file(config=config, vehicle_id=vehicle["id"])
        print(f"Generated {mission_path}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())