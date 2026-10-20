"""Compose service definitions for the picket-line mission."""

import sys

from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]

sys.path.insert(0, str(ROOT / "components" / "manta_core" / "scripts"))
from manta_load import container_path
from manta_services import ros_service, vehicle_service


def create_services(config: dict, ros_domain_id: int = 1) -> dict:
    """Create vehicle, whale, detection, and orchestrator services."""
    scenario = config["scenario"]

    if scenario["type"] != "picket_line":
        raise ValueError("Expected a picket_line scenario.")

    vehicles = config["vehicles"]
    output = config["output"]
    simulation = config["simulation"]
    monte_carlo = config.get("monte_carlo", {})

    data_dir = container_path(output["data_directory"])
    log_dir = container_path(output["log_directory"])
    sample_period = float(output.get("auv_sample_period", 2.0))
    vehicle_ids = [vehicle["id"] for vehicle in vehicles]
    vehicle_list = f"[{', '.join(vehicle_ids)}]"

    # Vehicles
    services = {
        vehicle["id"]: vehicle_service(
            vehicle=vehicle,
            data_dir=data_dir,
            log_dir=log_dir,
            sample_period=sample_period,
            ros_domain_id=ros_domain_id,
            tracking_enabled=False,
        )
        for vehicle in vehicles
    }

    # Whales
    whale_parameters = {
        "mission_duration": scenario["duration"],
        "data_directory": data_dir,
        "random_seed": monte_carlo.get("random_seed", 42),
    }

    services["whale_sim"] = ros_service(
        "whale_sim", "whale_node",
        whale_parameters, log_dir, ros_domain_id,
        params_file="/odyssey/components/manta_whale_sim/ros/whale_sim/config.yaml",
    )
    services["whale_sim"]["stop_grace_period"] = "5s"

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
        params_file=(
            "/odyssey/components/manta_acoustics/ros/"
            "whale_detection_sim/config.yaml"
        ),
    )

    # Orchestrator
    orchestrator_parameters = {
        "vehicle_ids": vehicle_list,
        "data_directory": data_dir,
        "timestep": simulation["timestep"],
        "control_period": simulation["control_period"],
        "max_steps": simulation["max_steps"],
        "steps_per_message": simulation.get("steps_per_message", 1),
    }

    services["sim_orchestrator"] = ros_service(
        "sim_orchestrator", "sim_orchestrator_node",
        orchestrator_parameters, log_dir, ros_domain_id,
        extra_environment={
            "SIM_LOG_PATH": f"{log_dir}/sim_orchestrator.log"
        },
    )

    return services