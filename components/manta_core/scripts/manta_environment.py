import os

from manta_load import container_path
from paths import ENV_PATH

def get_host_environment() -> dict[str, str]:
    """Return the process environment with the host user and group IDs."""

    environment = os.environ.copy()
    environment["HOST_UID"] = str(os.getuid())
    environment["HOST_GID"] = str(os.getgid())

    return environment


def write_runtime_env(config: dict) -> None:
    """Write Docker Compose environment variables for the configured MANTA fleet."""
    ENV_PATH.parent.mkdir(parents=True, exist_ok=True)
    vehicle_ids = [vehicle["id"] for vehicle in config["vehicles"]]

    lines = [
        f"HOST_UID={os.getuid()}",
        f"HOST_GID={os.getgid()}",
        f"MANTA_VEHICLE_IDS=[{', '.join(vehicle_ids)}]",
        f"MANTA_DATA_DIR={container_path(config['output']['data_directory'])}",
        f"MANTA_LOG_DIR={container_path(config['output']['log_directory'])}",
    ]

    for vehicle in config["vehicles"]:
        prefix = "".join(character.upper() if character.isalnum() else "_" for character in vehicle["id"])
        fault_path = container_path(vehicle["fault"]) if vehicle.get("fault") else ""

        lines.extend([
            f"{prefix}_TYPE={vehicle['type']}",
            f"{prefix}_MISSION={container_path(vehicle['mission'])}",
            f"{prefix}_MODEL={container_path(vehicle['model'])}",
            f"{prefix}_FAULT={fault_path}",
        ])

    ENV_PATH.write_text("\n".join(lines) + "\n", encoding="utf-8")