#!/usr/bin/env python3

import math
import sys
from pathlib import Path

import yaml


def main() -> int:
    root = Path(__file__).resolve().parents[2]

    if len(sys.argv) > 2:
        print(f"Usage: {sys.argv[0]} [configuration-file]")
        return 2

    config_path = (
        Path(sys.argv[1]).resolve()
        if len(sys.argv) == 2
        else root / "config" / "simulation.yaml"
    )

    with config_path.open("r", encoding="utf-8") as file:
        config = yaml.safe_load(file)

    scenario = config["scenario"]
    vehicles = config["vehicles"]
    line = scenario["picket_line"]
    patrol = scenario["patrol"]

    line_start_n = float(line["start"]["north"])
    line_start_e = float(line["start"]["east"])
    line_end_n = float(line["end"]["north"])
    line_end_e = float(line["end"]["east"])

    duration = float(scenario["duration"])
    speed = float(patrol["speed"])
    depth = float(patrol["depth"])
    capture = float(patrol.get("capture", 25.0))

    delta_n = line_end_n - line_start_n
    delta_e = line_end_e - line_start_e
    line_length = math.hypot(delta_n, delta_e)

    if not vehicles:
        raise ValueError("At least one vehicle is required.")

    if line_length <= 0.0:
        raise ValueError("Picket line must have nonzero length.")

    if duration <= 0.0:
        raise ValueError("Scenario duration must be greater than zero.")

    if speed <= 0.0:
        raise ValueError("Patrol speed must be greater than zero.")

    if capture <= 0.0:
        raise ValueError("Patrol capture distance must be greater than zero.")

    output = root / "generated" / "missions"
    output.mkdir(parents=True, exist_ok=True)

    vehicle_count = len(vehicles)

    for index, vehicle in enumerate(vehicles):
        start_fraction = index / vehicle_count
        end_fraction = (index + 1) / vehicle_count

        segment_start_n = line_start_n + start_fraction * delta_n
        segment_start_e = line_start_e + start_fraction * delta_e
        segment_end_n = line_start_n + end_fraction * delta_n
        segment_end_e = line_start_e + end_fraction * delta_e

        vehicle_id = vehicle["id"]

        mission = f"""# Generated MANTA picket-line mission for {vehicle_id}
UseSim 1

# Start at the beginning of this vehicle's assigned segment.
sensor: m_pos_n(m) {segment_start_n:.6f}
sensor: m_pos_e(m) {segment_start_e:.6f}
sensor: m_depth(m) {depth:.6f}
sensor: real_x(m) {segment_start_n:.6f}
sensor: real_y(m) {segment_start_e:.6f}
sensor: real_z(m) {depth:.6f}

state: picket_line

# Stop after the configured scenario duration.
behavior: mission_timer 1
  b_arg: time(s) {duration:.6f}

# Maintain safe depth limits.
behavior: depth_envelope 2
  b_arg: max_depth(m) {depth + 20.0:.6f}
  b_arg: min_depth(m) 0.5
  b_arg: alt_env_active(bool) 0
  b_arg: min_altitude(m) 2.0
  b_arg: depth_cutoff_active(bool) 0
  b_arg: cutoff_depth(m) {depth + 25.0:.6f}

# Patrol using measured position instead of estimated travel time.
behavior: picket_line_patrol 3
  b_arg: start_n(m) {segment_start_n:.6f}
  b_arg: start_e(m) {segment_start_e:.6f}
  b_arg: end_n(m) {segment_end_n:.6f}
  b_arg: end_e(m) {segment_end_e:.6f}
  b_arg: depth(m) {depth:.6f}
  b_arg: speed(m/s) {speed:.6f}
  b_arg: capture(m) {capture:.6f}
"""

        mission_path = output / f"picket_line_{vehicle_id}.mi"
        mission_path.write_text(mission, encoding="utf-8")
        print(f"Generated {mission_path.relative_to(root)}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())