import os


def get_host_environment() -> dict[str, str]:
    """Return the process environment with the host user and group IDs."""

    environment = os.environ.copy()
    environment["HOST_UID"] = str(os.getuid())
    environment["HOST_GID"] = str(os.getgid())

    return environment