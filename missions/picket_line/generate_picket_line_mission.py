#!/usr/bin/env python3

import math
import sys
import yaml
from pathlib import Path


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

    start_n = float(line["start"]["north"])
    start_e = float(line["start"]["east"])
    end_n = float(line["end"]["north"])
    end_e = float(line["end"]["east"])
    duration = float(scenario["duration"])
    speed = float(patrol["speed"])
    depth = float(patrol["depth"])

    delta_n = end_n - start_n
    delta_e = end_e - start_e
    line_length = math.hypot(delta_n, delta_e)

    if line_length <= 0:
        raise ValueError("Picket line must have nonzero length.")

    if speed <= 0:
        raise ValueError("Patrol speed must be greater than zero.")

    segment_length = line_length / len(vehicles)
    leg_time = segment_length / speed
    cycles = math.ceil(duration / (2 * leg_time))
    heading = math.atan2(delta_e, delta_n) % (2 * math.pi)

    output = root / "generated" / "missions"
    output.mkdir(parents=True, exist_ok=True)

    for index, vehicle in enumerate(vehicles):
        fraction = index / len(vehicles)
        initial_n = start_n + fraction * delta_n
        initial_e = start_e + fraction * delta_e
        vehicle_id = vehicle["id"]

        mission = f"""# Generated MANTA picket-line mission for {vehicle_id}
UseSim 1

# Start at the beginning of this vehicle's assigned segment.
sensor: m_pos_n(m) {initial_n:.6f}
sensor: m_pos_e(m) {initial_e:.6f}
sensor: m_depth(m) {depth:.6f}
sensor: real_x(m) {initial_n:.6f}
sensor: real_y(m) {initial_e:.6f}
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

# Patrol back and forth along this vehicle's segment.
behavior: survey_dead_reckon 3
  b_arg: trans_heading(rad) {heading:.9f}
  b_arg: trans_time(s) 0.0
  b_arg: heading_leg1(rad) {heading:.9f}
  b_arg: time_leg1(s) {leg_time:.6f}
  b_arg: heading_leg2(rad) {heading:.9f}
  b_arg: time_leg2(s) 0.0
  b_arg: cycles(#) {cycles}
  b_arg: depth(m) {depth:.6f}
  b_arg: transit_speed(m/s) {speed:.6f}
  b_arg: survey_speed(m/s) {speed:.6f}
"""

        path = output / f"picket_line_{vehicle_id}.mi"
        path.write_text(mission, encoding="utf-8")
        print(f"Generated {path.relative_to(root)}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())