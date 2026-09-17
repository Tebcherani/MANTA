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


def load_plugin(root: Path, plugin_id: str) -> dict:
    """Load and validate a MANTA vehicle plugin manifest."""

    path = root / "vehicles" / plugin_id / "manta_plugin.yaml"

    if not path.is_file():
        raise ValueError(f"Vehicle plugin '{plugin_id}' was not found: {path}")

    manifest = load_config(path)
    plugin = manifest.get("plugin")

    if not isinstance(plugin, dict):
        raise ValueError(f"Plugin '{plugin_id}' has no 'plugin' section.")

    if plugin.get("id") != plugin_id:
        raise ValueError(
            f"Plugin manifest ID '{plugin.get('id')}' does not match configured type '{plugin_id}'."
        )

    if plugin.get("kind") != "vehicle":
        raise ValueError(f"Plugin '{plugin_id}' is not a vehicle plugin.")

    return manifest


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

        manifest = load_plugin(root, vehicle_type)
        plugin_config = manifest.get("configuration", {})
        required_fields = plugin_config.get("required", [])
        optional_fields = plugin_config.get("optional", [])

        if not isinstance(required_fields, list) or not isinstance(optional_fields, list):
            raise ValueError(f"Plugin '{vehicle_type}' configuration fields must be YAML lists.")

        for field in required_fields:
            value = vehicle.get(field)

            if not isinstance(value, str) or not value:
                raise ValueError(f"Vehicle '{vehicle_id}' must define a nonempty '{field}' path.")

        for field in required_fields + optional_fields:
            value = vehicle.get(field, "")

            if not isinstance(value, str):
                raise ValueError(f"Vehicle '{vehicle_id}' field '{field}' must be a string.")

            if value:
                path = (root / value).resolve()

                if not path.is_file():
                    raise ValueError(f"Vehicle '{vehicle_id}' {field} file does not exist: {path}")


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
        prefix = "".join(
            character.upper() if character.isalnum() else "_"
            for character in vehicle["id"]
        )

        lines.extend([
            f"{prefix}_TYPE={vehicle['type']}",
            f"{prefix}_MISSION={container_path(root, vehicle['mission'])}",
            f"{prefix}_MODEL={container_path(root, vehicle['model'])}",
            f"{prefix}_FAULT={container_path(root, vehicle['fault']) if vehicle.get('fault') else ''}",
        ])

    path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return path


def vehicle_service(vehicle: dict, root: Path, data_dir: str, log_dir: str, sample_period: float) -> dict:
    """Generate a Compose service from a vehicle plugin manifest."""

    vehicle_id = vehicle["id"]
    vehicle_type = vehicle["type"]
    manifest = load_plugin(root, vehicle_type)
    runtime = manifest.get("runtime", {})
    environment_names = runtime.get("environment", {})
    config_environment = runtime.get("configuration_environment", {})

    try:
        image = runtime["image"]
        ros_package = runtime["ros_package"]
        executable = runtime["executable"]
        vehicle_id_env = environment_names["vehicle_id"]
        data_dir_env = environment_names["data_directory"]
        log_dir_env = environment_names["log_directory"]
        simulation_log_env = environment_names["simulation_log"]
    except KeyError as error:
        raise ValueError(
            f"Plugin '{vehicle_type}' is missing runtime setting: {error.args[0]}"
        ) from error

    environment = {
        "ROS_DOMAIN_ID": "1",
        vehicle_id_env: vehicle_id,
        data_dir_env: data_dir,
        log_dir_env: log_dir,
        simulation_log_env: f"{log_dir}/{vehicle_id}.log",
    }

    for field, environment_name in config_environment.items():
        value = vehicle.get(field, "")
        environment[environment_name] = container_path(root, value) if value else ""

    return {
        "image": image,
        "environment": environment,
        "volumes": [
            {
                "type": "bind",
                "source": str(root),
                "target": "/odyssey",
            }
        ],
        "command": [
            "/bin/bash",
            "-lc",
            "source /opt/ros/jazzy/setup.bash\n"
            "source /odyssey/ros_ws/install/setup.bash\n"
            f"ros2 run {ros_package} {executable} --ros-args -p output_sample_period:={sample_period}",        ],
    }


def whale_service(config: dict, root: Path, log_dir: str) -> dict:
    """Create the whale simulation service."""

    scenario = config["scenario"]
    line = scenario["picket_line"]
    whales = scenario["whales"]
    monte_carlo = scenario.get("monte_carlo", {})

    params = {
        "count": whales["count"],
        "line_start_north": line["start"]["north"],
        "line_start_east": line["start"]["east"],
        "line_end_north": line["end"]["north"],
        "line_end_east": line["end"]["east"],
        "speed_min": whales["speed"],
        "speed_max": whales["speed"],
        "start_distance": whales["start_distance"],
        "depth": whales["depth"],
        "mission_duration": scenario["duration"],
        "call_rate_per_hour": whales["call_rate"],
        "data_directory": container_path(root, config["output"]["data_directory"]),
        "random_seed": monte_carlo.get("random_seed", 42),
        "sample_period": config["output"].get("whale_sample_period", 60.0),
    }

    arguments = " ".join(
        f'-p "{name}:={value}"'
        for name, value in params.items()
    )

    return {
        "image": "odyssey_sim_dev",
        "environment": {
            "ROS_DOMAIN_ID": "1",
            "ROS_LOG_DIR": log_dir,
        },
        "volumes": [
            {
                "type": "bind",
                "source": str(root),
                "target": "/odyssey",
            }
        ],
        "command": [
            "/bin/bash",
            "-lc",
            "source /opt/ros/jazzy/setup.bash\n"
            "source /odyssey/ros_ws/install/setup.bash\n"
            f"ros2 run whale_sim whale_node --ros-args {arguments}",
        ],
    }


def whale_detection_service(config: dict, root: Path, log_dir: str) -> dict:
    """Create the whale-call detection service."""

    vehicle_ids = [vehicle["id"] for vehicle in config["vehicles"]]
    vehicle_list = f"[{', '.join(vehicle_ids)}]"
    detection_range = config["scenario"]["detection"]["radius"]

    data_directory = container_path(
        root,
        config["output"]["data_directory"],
    )

    return {
        "image": "odyssey_sim_dev",
        "environment": {
            "ROS_DOMAIN_ID": "1",
            "ROS_LOG_DIR": log_dir,
        },
        "volumes": [
            {
                "type": "bind",
                "source": str(root),
                "target": "/odyssey",
            }
        ],
        "command": [
            "/bin/bash",
            "-lc",
            "source /opt/ros/jazzy/setup.bash\n"
            "source /odyssey/ros_ws/install/setup.bash\n"
            "ros2 run whale_detection_sim whale_detection_node "
            f'--ros-args -p "vehicle_ids:={vehicle_list}" '
            f'-p "detection_range:={detection_range}"',
            f'-p "detection_range:={detection_range}" '
            f'-p "data_directory:={data_directory}"',
        ],
    }


def write_runtime_compose(config: dict, root: Path) -> Path:
    generated = root / "generated"
    generated.mkdir(exist_ok=True)

    path = generated / "compose.ros.yaml"
    data_dir = container_path(root, config["output"]["data_directory"])
    log_dir = container_path(root, config["output"]["log_directory"])
    vehicle_ids = [vehicle["id"] for vehicle in config["vehicles"]]
    simulation = config["simulation"]
    sample_period = float(config["output"].get("auv_sample_period", 2.0))

    services = {
        vehicle["id"]: vehicle_service(vehicle, root, data_dir, log_dir, sample_period)
        for vehicle in config["vehicles"]
    }

    if config.get("scenario", {}).get("type") == "picket_line":
        services["whale_sim"] = whale_service(config, root, log_dir)
        services["whale_detection_sim"] = whale_detection_service(
            config,
            root,
            log_dir,
        )

    vehicle_list = f"[{', '.join(vehicle_ids)}]"

    services["sim_orchestrator"] = {
        "image": "odyssey_sim_dev",
        "environment": {
            "ROS_DOMAIN_ID": "1",
            "ROS_LOG_DIR": log_dir,
            "SIM_LOG_PATH": f"{log_dir}/sim_orchestrator.log",
        },
        "volumes": [
            {
                "type": "bind",
                "source": str(root),
                "target": "/odyssey",
            }
        ],
        "command": [
            "/bin/bash",
            "-lc",
            "source /opt/ros/jazzy/setup.bash\n"
            "source /odyssey/ros_ws/install/setup.bash\n"
            "ros2 run sim_orchestrator sim_orchestrator_node "
            f'--ros-args -p "vehicle_ids:={vehicle_list}" '
            f'-p "data_directory:={data_dir}" '
            f'-p "timestep:={simulation["timestep"]}" '
            f'-p "control_period:={simulation["control_period"]}" '
            f'-p "max_cycles:={simulation["max_cycles"]}" '
            f'-p "steps_per_message:={simulation.get("steps_per_message", 1)}"',
        ],
    }

    path.write_text(
        yaml.safe_dump(
            {
                "name": "manta",
                "services": services,
            },
            sort_keys=False,
        ),
        encoding="utf-8",
    )

    return path


def parse_args(root: Path) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Build, run, and review MANTA simulations."
    )

    parser.add_argument(
        "command",
        nargs="?",
        default="generate",
        choices=(
            "generate",
            "build",
            "dev",
            "run",
            "down",
            "review",
        ),
    )

    parser.add_argument(
        "plot",
        nargs="?",
        default="tracks",
        choices=(
            "tracks",
            "energy",
        ),
        help="Plot to show with the review command.",
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
        scripts = {
            "tracks": "plot_tracks.py",
            "energy": "plot_energy.py",
        }

        python = root / "review" / ".venv" / "bin" / "python"
        script = root / "review" / "manta_review" / scripts[args.plot]

        if not python.is_file():
            print(f"Review Python is missing: {python}", file=sys.stderr)
            return 1

        if not script.is_file():
            print(f"Review script is missing: {script}", file=sys.stderr)
            return 1

        return subprocess.run(
            [str(python), str(script)],
            cwd=script.parent,
            check=False,
        ).returncode

    try:
        config_path = Path(args.config).resolve()
        config = load_config(config_path)

        # Generate scenario-specific vehicle missions before validating their paths.
        scenario = config.get("scenario", {})

        if scenario.get("type") == "picket_line":
            generator = (
                root
                / "missions"
                / "picket_line"
                / "generate_picket_line_mission.py"
            )

            result = subprocess.run(
                [
                    sys.executable,
                    str(generator),
                    str(config_path),
                ],
                cwd=root,
                check=False,
            )

            if result.returncode != 0:
                raise ValueError(
                    "Failed to generate picket-line missions."
                )

        validate_config(config, root)
        print_summary(config, config_path)

        env_path = write_runtime_env(config, root)
        compose_path = write_runtime_compose(config, root)

        print(f"Generated environment: {env_path}")
        print(f"Generated Compose file: {compose_path}")

        commands = {
            "build": [
                "docker",
                "compose",
                "--env-file",
                str(env_path),
                "-f",
                str(root / "compose.dev.yaml"),
                "build",
            ],
            "dev": [
                "docker",
                "compose",
                "--env-file",
                str(env_path),
                "-f",
                str(root / "compose.dev.yaml"),
                "run",
                "--rm",
                "dev",
            ],
            "run": [
                "docker",
                "compose",
                "-f",
                str(compose_path),
                "up",
                "--remove-orphans",
            ],
            "down": [
                "docker",
                "compose",
                "-f",
                str(compose_path),
                "down",
                "--remove-orphans",
            ],
        }

        if args.command == "generate":
            return 0

        if args.command == "build":
            result = subprocess.run(
                commands["build"],
                cwd=root,
                check=False,
            )

            if result.returncode != 0:
                return result.returncode

            build_inside_container = [
                "docker",
                "compose",
                "--env-file",
                str(env_path),
                "-f",
                str(root / "compose.dev.yaml"),
                "run",
                "--rm",
                "dev",
                "/bin/bash",
                "-lc",
                "source /opt/ros/jazzy/setup.bash && "
                "cd /odyssey/vehicles/odyssey && "
                "make generate_odyssey && "
                "make libraries && "
                "cd /odyssey/ros_ws && "
                "colcon build "
                "--cmake-args -DCMAKE_BUILD_TYPE=Release",
            ]

            return subprocess.run(
                build_inside_container,
                cwd=root,
                check=False,
            ).returncode

        return subprocess.run(
            commands[args.command],
            cwd=root,
            check=False,
        ).returncode

    except ValueError as error:
        print(
            f"MANTA configuration error: {error}",
            file=sys.stderr,
        )
        return 1

    except FileNotFoundError as error:
        print(
            f"Required command was not found: {error.filename}",
            file=sys.stderr,
        )
        return 1


if __name__ == "__main__":
    raise SystemExit(main())