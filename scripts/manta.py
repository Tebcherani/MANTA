#!/usr/bin/env python3

import argparse
import os
import subprocess
import sys
from pathlib import Path

import yaml


def load_config(path: Path) -> dict:
    if not path.is_file():
        raise ValueError(f"Configuration file does not exist: {path}")

    try:
        with path.open("r", encoding="utf-8") as file:
            config = yaml.safe_load(file)
    except yaml.YAMLError as error:
        raise ValueError(f"Invalid YAML in {path}: {error}") from error

    if not isinstance(config, dict):
        raise ValueError("The configuration must contain a YAML mapping.")

    return config


def validate_config(config: dict, root: Path) -> None:
    vehicles = config.get("vehicles")
    output = config.get("output")

    if not isinstance(vehicles, list) or not vehicles:
        raise ValueError("'vehicles' must be a nonempty YAML list.")

    if not isinstance(output, dict):
        raise ValueError("'output' must be a YAML mapping.")

    for field in ("data_directory", "log_directory"):
        if not isinstance(output.get(field), str) or not output[field]:
            raise ValueError(f"'output.{field}' must be a nonempty string.")

    vehicle_ids = set()

    for vehicle in vehicles:
        if not isinstance(vehicle, dict):
            raise ValueError("Every vehicle must be a YAML mapping.")

        vehicle_id = vehicle.get("id")
        vehicle_type = vehicle.get("type")

        if not isinstance(vehicle_id, str) or not vehicle_id:
            raise ValueError("Every vehicle must have a nonempty string 'id'.")

        if vehicle_id in vehicle_ids:
            raise ValueError(f"Duplicate vehicle ID: {vehicle_id}")

        vehicle_ids.add(vehicle_id)

        if not isinstance(vehicle_type, str) or not vehicle_type:
            raise ValueError(f"Vehicle '{vehicle_id}' must have a nonempty string 'type'.")

        if vehicle_type != "odyssey":
            raise ValueError(f"Unsupported vehicle type: {vehicle_type}")

        for field in ("mission", "model"):
            value = vehicle.get(field)

            if not isinstance(value, str) or not value:
                raise ValueError(f"Vehicle '{vehicle_id}' must define a nonempty '{field}' path.")

            path = (root / value).resolve()

            if not path.is_file():
                raise ValueError(f"Vehicle '{vehicle_id}' {field} file does not exist: {path}")

        fault = vehicle.get("fault", "")

        if not isinstance(fault, str):
            raise ValueError(f"Vehicle '{vehicle_id}' fault path must be a string.")

        if fault and not (root / fault).resolve().is_file():
            raise ValueError(f"Vehicle '{vehicle_id}' fault file does not exist: {(root / fault).resolve()}")


def print_summary(config: dict, path: Path) -> None:
    print(f"Loaded configuration: {path}")
    print(f"Configured vehicles: {len(config['vehicles'])}")

    for vehicle in config["vehicles"]:
        print(f"  - {vehicle['id']} ({vehicle['type']})")


def container_path(root: Path, configured_path: str) -> str:
    host_path = (root / configured_path).resolve()

    try:
        relative_path = host_path.relative_to(root.resolve())
    except ValueError as error:
        raise ValueError(f"Path must remain inside MANTA: {host_path}") from error

    return f"/odyssey/{relative_path.as_posix()}"


def write_runtime_env(config: dict, root: Path) -> Path:
    generated = root / "generated"
    generated.mkdir(exist_ok=True)

    path = generated / "manta.env"
    vehicle_ids = [vehicle["id"] for vehicle in config["vehicles"]]

    lines = [
        f"HOST_UID={os.getuid()}",
        f"HOST_GID={os.getgid()}",
        f"MANTA_VEHICLE_IDS=[{', '.join(vehicle_ids)}]",
        f"MANTA_DATA_DIR={container_path(root, config['output']['data_directory'])}",
        f"MANTA_LOG_DIR={container_path(root, config['output']['log_directory'])}",
    ]

    for vehicle in config["vehicles"]:
        prefix = "".join(char.upper() if char.isalnum() else "_" for char in vehicle["id"])

        lines.extend([
            f"{prefix}_TYPE={vehicle['type']}",
            f"{prefix}_MISSION={container_path(root, vehicle['mission'])}",
            f"{prefix}_MODEL={container_path(root, vehicle['model'])}",
            f"{prefix}_FAULT={container_path(root, vehicle['fault']) if vehicle.get('fault') else ''}",
        ])

    path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return path


def vehicle_service(vehicle: dict, root: Path, data_dir: str, log_dir: str) -> dict:
    vehicle_id = vehicle["id"]

    return {
        "image": "odyssey_sim_dev",
        "environment": {
            "ROS_DOMAIN_ID": "1",
            "ODYSSEY_VEHICLE_ID": vehicle_id,
            "ODYSSEY_DATA_DIR": data_dir,
            "ROS_LOG_DIR": log_dir,
            "FAULT_PATH": container_path(root, vehicle["fault"]) if vehicle.get("fault") else "",
            "MISSION_PATH": container_path(root, vehicle["mission"]),
            "VEHICLE_PATH": container_path(root, vehicle["model"]),
            "SIM_LOG_PATH": f"{log_dir}/{vehicle_id}.log",
        },
        "volumes": [{"type": "bind", "source": str(root), "target": "/odyssey"}],
        "command": [
            "/bin/bash",
            "-lc",
            "source /opt/ros/jazzy/setup.bash\n"
            "source /odyssey/ros_ws/install/setup.bash\n"
            "ros2 run odyssey_sim odyssey_node",
        ],
    }


def write_runtime_compose(config: dict, root: Path) -> Path:
    generated = root / "generated"
    generated.mkdir(exist_ok=True)

    path = generated / "compose.ros.yaml"
    data_dir = container_path(root, config["output"]["data_directory"])
    log_dir = container_path(root, config["output"]["log_directory"])
    vehicle_ids = [vehicle["id"] for vehicle in config["vehicles"]]

    services = {
        vehicle["id"]: vehicle_service(vehicle, root, data_dir, log_dir)
        for vehicle in config["vehicles"]
    }

    vehicle_list = f"[{', '.join(vehicle_ids)}]"

    services["sim_orchestrator"] = {
        "image": "odyssey_sim_dev",
        "environment": {
            "ROS_DOMAIN_ID": "1",
            "ROS_LOG_DIR": log_dir,
            "SIM_LOG_PATH": f"{log_dir}/sim_orchestrator.log",
        },
        "volumes": [{"type": "bind", "source": str(root), "target": "/odyssey"}],
        "command": [
            "/bin/bash",
            "-lc",
            "source /opt/ros/jazzy/setup.bash\n"
            "source /odyssey/ros_ws/install/setup.bash\n"
            "ros2 run sim_orchestrator sim_orchestrator_node "
            f'--ros-args -p "vehicle_ids:={vehicle_list}"',
        ],
    }

    path.write_text(
        yaml.safe_dump({"name": "manta", "services": services}, sort_keys=False),
        encoding="utf-8",
    )

    return path


def parse_args(root: Path) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Build, run, and review MANTA simulations.")

    parser.add_argument(
        "command",
        nargs="?",
        default="generate",
        choices=("generate", "build", "dev", "run", "down", "review"),
    )

    parser.add_argument(
        "-c",
        "--config",
        default=str(root / "config" / "simulation.yaml"),
        help="Simulation configuration file.",
    )

    return parser.parse_args()


def main() -> int:
    root = Path(__file__).resolve().parent.parent
    args = parse_args(root)

    if args.command == "review":
        return subprocess.run(
            ["./ody", "review2"],
            cwd=root / "vehicles" / "odyssey",
            check=False,
        ).returncode

    try:
        config_path = Path(args.config).resolve()
        config = load_config(config_path)
        validate_config(config, root)
        print_summary(config, config_path)

        env_path = write_runtime_env(config, root)
        compose_path = write_runtime_compose(config, root)

        print(f"Generated environment: {env_path}")
        print(f"Generated Compose file: {compose_path}")

        commands = {
            "build": [
                "docker", "compose", "--env-file", str(env_path),
                "-f", str(root / "compose.dev.yaml"), "build",
            ],
            "dev": [
                "docker", "compose", "--env-file", str(env_path),
                "-f", str(root / "compose.dev.yaml"), "run", "--rm", "dev",
            ],
            "run": [
                "docker", "compose", "-f", str(compose_path),
                "up", "--remove-orphans",
            ],
            "down": [
                "docker", "compose", "-f", str(compose_path),
                "down", "--remove-orphans",
            ],
        }

        if args.command == "generate":
            return 0

        return subprocess.run(commands[args.command], cwd=root, check=False).returncode

    except ValueError as error:
        print(f"MANTA configuration error: {error}", file=sys.stderr)
        return 1

    except FileNotFoundError as error:
        print(f"Required command was not found: {error.filename}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())