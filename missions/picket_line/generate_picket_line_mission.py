#!/usr/bin/env python3

"""
Generate one Odyssey picket-line mission file per configured vehicle.

The picket line is divided into equal-length segments. Each vehicle patrols
between the endpoints of its assigned segment using the Odyssey
`picket_line_patrol` behavior.
"""


import math
import sys
import yaml

from dataclasses import dataclass
from pathlib import Path
from textwrap import dedent


ROOT = Path(__file__).resolve().parents[2]
config_path = Path(sys.argv[1]).resolve() if len(sys.argv) == 2 else CONFIG_PATH
OUTPUT_DIR = ROOT / "generated" / "missions" / "picket_line"


@dataclass(frozen=True)
class MissionConfig:
    """Configuration values needed to generate picket-line missions."""

    vehicles: list[dict]
    line_start_n: float
    line_start_e: float
    line_end_n: float
    line_end_e: float
    mission_duration: float
    vehicle_speed: float
    vehicle_depth: float
    vehicle_capture: float

    @property
    def delta_n(self) -> float:
        """North displacement of the complete picket line."""
        return self.line_end_n - self.line_start_n

    @property
    def delta_e(self) -> float:
        """East displacement of the complete picket line."""
        return self.line_end_e - self.line_start_e

    @property
    def line_length(self) -> float:
        """Total horizontal length of the picket line."""
        return math.hypot(self.delta_n, self.delta_e)

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
        if self.line_length <= 0.0:
            raise ValueError("Picket line must have nonzero length.")
        if self.mission_duration <= 0.0:
            raise ValueError("Scenario duration must be greater than zero.")
        if self.vehicle_speed <= 0.0:
            raise ValueError("Patrol speed must be greater than zero.")
        if self.vehicle_depth <= 0.0:
            raise ValueError("Patrol depth must be greater than zero.")
        if self.vehicle_capture <= 0.0:
            raise ValueError("Patrol capture distance must be greater than zero.")


@dataclass(frozen=True)
class LineSegment:
    """North/east endpoints of one vehicle's picket-line segment."""

    start_n: float
    start_e: float
    end_n: float
    end_e: float


def compute_vehicle_segment(config: MissionConfig, vehicle_index: int) -> LineSegment:
    """Calculate the picket-line segment assigned to one vehicle."""

    if vehicle_index < 0 or vehicle_index >= config.fleet_size:
        raise ValueError("Vehicle index is outside the fleet.")

    start_fraction = vehicle_index / config.fleet_size
    end_fraction = (vehicle_index + 1) / config.fleet_size

    return LineSegment(
        start_n = config.line_start_n + start_fraction * config.delta_n,
        start_e = config.line_start_e + start_fraction * config.delta_e,
        end_n = config.line_start_n + end_fraction * config.delta_n,
        end_e = config.line_start_e + end_fraction * config.delta_e,
    )


def generate_mission_file(config: MissionConfig, vehicle_id: str, segment: LineSegment) -> Path:
    """Generate and write the picket-line mission for one Odyssey."""

    mission = dedent(
        f"""\
        # Generated MANTA picket-line mission for {vehicle_id}
        UseSim 1

        # Start at the beginning of this vehicle's assigned segment.
        sensor: m_pos_n(m) {segment.start_n:.6f}
        sensor: m_pos_e(m) {segment.start_e:.6f}
        sensor: m_depth(m) {config.vehicle_depth:.6f}
        sensor: real_x(m) {segment.start_n:.6f}
        sensor: real_y(m) {segment.start_e:.6f}
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

        # Follow an assigned whale using its exact simulated position.
        behavior: whale_follow 3
          b_arg: depth(m) {config.vehicle_depth:.6f}
          b_arg: speed(m/s) {config.vehicle_speed:.6f}
          b_arg: capture(m) {config.vehicle_capture:.6f}

        # Patrol until a whale is assigned.
        behavior: picket_line_patrol 4
          b_arg: start_n(m) {segment.start_n:.6f}
          b_arg: start_e(m) {segment.start_e:.6f}
          b_arg: end_n(m) {segment.end_n:.6f}
          b_arg: end_e(m) {segment.end_e:.6f}
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
    line = scenario["picket_line"]
    patrol = scenario["patrol"]

    # Load configuration values
    config = MissionConfig(
        vehicles = config.get("vehicles", []),
        line_start_n = float(line["start"]["north"]),
        line_start_e = float(line["start"]["east"]),
        line_end_n = float(line["end"]["north"]),
        line_end_e = float(line["end"]["east"]),
        mission_duration = float(scenario["duration"]),
        vehicle_speed = float(patrol["speed"]),
        vehicle_depth = float(patrol["depth"]),
        vehicle_capture = float(patrol.get("capture", 25.0)),
    )

    return config


def main() -> int:
    """Load the MANTA configuration and generate the Odyssey mission files."""

    config = load_mission_config()
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

    # Generate mission file for each vehicle
    for index, vehicle in enumerate(config.vehicles):
        segment = compute_vehicle_segment(config=config, vehicle_index=index)
        mission_path = generate_mission_file(config=config, vehicle_id=vehicle["id"], segment=segment)
        print(f"Generated {mission_path.relative_to(ROOT)}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())