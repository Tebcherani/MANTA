import subprocess

from paths import ROOT, DEV_COMPOSE_PATH
from pathlib import Path
from paths import ENV_PATH


def run_build() -> int:
    build_image = ["docker", "compose", "--env-file", str(ENV_PATH), "-f", str(DEV_COMPOSE_PATH), "build"]

    result = subprocess.run(build_image, cwd=ROOT, check=False)
    if result.returncode != 0: return result.returncode

    build_inside_container = [
        "docker",
        "compose",
        "--env-file",
        str(ENV_PATH),
        "-f",
        str(DEV_COMPOSE_PATH),
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
        "colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release",
    ]

    return subprocess.run(build_inside_container, cwd=ROOT, check=False).returncode