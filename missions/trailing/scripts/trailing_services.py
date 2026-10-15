"""Compose service definitions for the trailing mission."""

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]

sys.path.insert(0, str(ROOT / "components" / "manta_core" / "scripts"))

# Support container_path until it is migrated into core.
sys.path.append(str(ROOT / "scripts"))

from manta_services import ros_service, vehicle_service
from manta_load import container_path


def create_services(config: dict, ros_domain_id: int = 1) -> dict:
    """Assemble vehicle, whale, detection, and orchestrator services."""
    scenario = config["scenario"]

    if scenario["type"] != "trailing":
        raise ValueError("Expected a trailing scenario.")

    # Read configurable movement, calling, and observation settings.
    vehicles = config["vehicles"]
    output = config["output"]
    simulation = config["simulation"]
    trailing = scenario["trailing"]
    communication_interval = float(trailing.get("communication_interval", 0.0))
    whales = scenario["whales"]
    movement = whales["movement"]
    bouts = whales["calling"]["bouts"]
    monte_carlo = config.get("monte_carlo", {})

    # Translate output paths into container paths.
    data_dir = container_path(output["data_directory"])
    log_dir = container_path(output["log_directory"])
    sample_period = float(output.get("auv_sample_period", 2.0))

    vehicle_ids = [vehicle["id"] for vehicle in vehicles]
    vehicle_list = f"[{', '.join(vehicle_ids)}]"

    # Enable trailing behavior on each configured vehicle.
    services = {
        vehicle["id"]: vehicle_service(
            vehicle=vehicle,
            data_dir=data_dir,
            log_dir=log_dir,
            sample_period=sample_period,
            ros_domain_id=ros_domain_id,
            tracking_enabled=True,
            communication_interval=communication_interval,
        )
        for vehicle in vehicles
    }

    # Trailing uses one whale, active throughout the mission.
    # Fix its initial position and heading at the origin, facing north.
    whale_parameters = {
        "count": 1,
        "mission_duration": scenario["duration"],
        "active_entire_mission": True,
        "initialization_mode": "fixed",
        "initial_north": 0.0,
        "initial_east": 0.0,
        "initial_heading_degrees": 0.0,
        "bout_mean_duration": bouts["mean_duration"],
        "bout_mean_silent_duration": bouts["mean_silent_duration"],
        "bout_active_call_rate_per_hour": bouts["active_call_rate_per_hour"],
        "turn_interval": movement["turn_interval"],
        "heading_correlation": movement["heading_correlation"],
        "heading_std_degrees": movement["heading_std_degrees"],
        "speed_mean": movement["speed_mean"],
        "speed_std": movement["speed_std"],

        # The shared whale simulator still uses group heading data.
        "group_mean_size": 1.0,
        "group_position_std": 0.0,
        "group_start_time_std": 0.0,
        "shared_heading_fraction": 0.75,

        "data_directory": data_dir,
        "random_seed": monte_carlo.get("random_seed", 42),
        "sample_period": output.get("whale_sample_period", 60.0),
    }

    services["whale_sim"] = ros_service(
        "whale_sim", "whale_node",
        whale_parameters, log_dir, ros_domain_id,
    )
    services["whale_sim"]["stop_grace_period"] = "5s"

    # Mission-specific settings. Acoustic settings come from config.yaml.
    detection_parameters = {
        "vehicle_ids": vehicle_list,
        "data_directory": data_dir,
    }

    services["whale_detection_sim"] = ros_service(
        "whale_detection_sim",
        "whale_detection_node",
        detection_parameters,
        log_dir,
        ros_domain_id,
        params_file=("/odyssey/components/manta_acoustics/ros/whale_detection_sim/config.yaml"),
    )

    # Coordinate simulation time and acoustic call processing.
    orchestrator_parameters = {
        "vehicle_ids": vehicle_list,
        "data_directory": data_dir,
        "timestep": simulation["timestep"],
        "control_period": simulation["control_period"],
        "max_steps": simulation["max_steps"],
        "steps_per_message": simulation.get("steps_per_message", 1),
        "wait_for_bearing_processing": True,
        "communication_interval": communication_interval,
    }

    services["sim_orchestrator"] = ros_service(
        "sim_orchestrator", "sim_orchestrator_node",
        orchestrator_parameters, log_dir, ros_domain_id,
        extra_environment={"SIM_LOG_PATH": f"{log_dir}/sim_orchestrator.log"},
    )

    return services