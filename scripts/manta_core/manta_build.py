import subprocess

from manta_core.manta_environment import get_host_environment
from paths import DEV_COMPOSE_PATH, ROOT


def run_build() -> int:
    environment = get_host_environment()

    build_image = ["docker", "compose", "-f", str(DEV_COMPOSE_PATH), "build"]

    result = subprocess.run(build_image, cwd=ROOT, env=environment, check=False)
    if result.returncode != 0: return result.returncode

    build_inside_container = [
        "docker",
        "compose",
        "-f",
        str(DEV_COMPOSE_PATH),
        "run",
        "--rm",
        "dev",
        "/bin/bash",
        "-lc",
        "source /opt/ros/jazzy/setup.bash && "
        "cd /odyssey/components/manta_odyssey && "
        "make generate_odyssey && "
        "make libraries && "
        "cd /odyssey/ros_ws && "
        "colcon build "
        "--base-paths "
        "/odyssey/ros_ws/src "
        "/odyssey/components/manta_acoustics/ros "
        "/odyssey/components/manta_core/ros "
        "/odyssey/components/manta_whale_sim/ros "
        "/odyssey/components/manta_odyssey/ros_ws/src "
        "--cmake-args -DCMAKE_BUILD_TYPE=Release",
    ]

    return subprocess.run(build_inside_container, cwd=ROOT, env=environment, check=False).returncode