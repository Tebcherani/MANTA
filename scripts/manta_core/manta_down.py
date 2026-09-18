import subprocess

from paths import DOCKER_COMPOSE_PATH, ROOT


def run_down() -> int:
    """Stop and remove the generated MANTA Compose services."""

    if not DOCKER_COMPOSE_PATH.is_file():
        print("No generated MANTA Compose file exists.")
        return 0

    command = ["docker", "compose", "-f", str(DOCKER_COMPOSE_PATH), "down", "--remove-orphans"]

    return subprocess.run(command, cwd=ROOT, check=False).returncode