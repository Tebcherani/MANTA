"""Shared MANTA configuration utilities."""

from pathlib import Path
from paths import COMPONENT_DIR, MISSION_DIR, ROOT

import importlib.util
import yaml


def container_path(configured_path: str) -> str:
    """Convert a repository-relative host path to its `/odyssey` container path."""
    host_path = (ROOT / configured_path).resolve()

    try:
        relative_path = host_path.relative_to(ROOT.resolve())
    except ValueError as error:
        raise ValueError(f"Path must remain inside MANTA: {host_path}") from error

    return f"/odyssey/{relative_path.as_posix()}"


def load_mission_services(config: dict, ros_domain_id: int = 1) -> dict:
    """Build services using the configured mission's service module."""
    mission_type = config["scenario"]["type"]
    mission_root = (ROOT / "missions").resolve()
    script = (
        mission_root / mission_type / "scripts"
        / f"{mission_type}_services.py"
    ).resolve()

    if not script.is_relative_to(mission_root):
        raise ValueError("Mission service script must be inside missions.")
    if not script.is_file():
        raise ValueError(f"Mission service script does not exist: {script}")

    spec = importlib.util.spec_from_file_location(
        f"{mission_type}_services", script
    )

    if spec is None or spec.loader is None:
        raise ValueError(f"Cannot load mission services: {script}")

    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

    return module.create_services(config, ros_domain_id=ros_domain_id)


def load_plugin(plugin_id: str) -> dict:
    """Load a vehicle plugin manifest and verify its ID and plugin type."""

    path = COMPONENT_DIR / f"manta_{plugin_id}" / "manta_plugin.yaml"

    if not path.is_file(): raise ValueError(f"Vehicle plugin '{plugin_id}' was not found: {path}")

    manifest = load_yaml(path)
    plugin = manifest.get("plugin")

    if not isinstance(plugin, dict): raise ValueError(f"Plugin '{plugin_id}' has no 'plugin' section.")
    if plugin.get("id") != plugin_id: raise ValueError(f"Plugin manifest ID '{plugin.get('id')}' does not match configured type '{plugin_id}'.")
    if plugin.get("kind") != "vehicle": raise ValueError(f"Plugin '{plugin_id}' is not a vehicle plugin.")

    return manifest


def load_yaml(path: Path) -> dict:
    """Load a YAML file and require a top-level mapping."""
    path = Path(path)

    if not path.is_file():
        raise ValueError(f"YAML file does not exist: {path}")

    try:
        with path.open("r", encoding="utf-8") as file:
            data = yaml.safe_load(file)
    except yaml.YAMLError as error:
        raise ValueError(f"Invalid YAML in {path}: {error}") from error

    if not isinstance(data, dict):
        raise ValueError(f"YAML file must contain a top-level mapping: {path}")

    return data