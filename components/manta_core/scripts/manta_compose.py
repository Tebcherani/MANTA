"""Docker Compose file generation for MANTA."""

from pathlib import Path

import yaml

from paths import DOCKER_COMPOSE_PATH


def write_runtime_compose(services: dict, compose_path: Path | None = None, project_name: str = "manta") -> Path:
    """Write a Compose file using mission-supplied services."""
    if compose_path is None:
        compose_path = DOCKER_COMPOSE_PATH

    compose_path = Path(compose_path)
    compose_path.parent.mkdir(parents=True, exist_ok=True)

    document = {"name": project_name, "services": services}
    compose_path.write_text(yaml.safe_dump(document, sort_keys=False), encoding="utf-8")

    return compose_path