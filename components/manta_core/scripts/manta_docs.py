"""Build and preview documentation in a dedicated Docker container."""

import argparse
import hashlib
import importlib.util
import os
from pathlib import Path
import shutil
import socket
import subprocess
import time
import uuid


ROOT = Path(__file__).resolve().parents[3]
TOOLING = ROOT / "docs/sphinx"
IMAGE = "manta-docs:local"


def port_number(value):
    port = int(value)
    if not 1 <= port <= 65535:
        raise argparse.ArgumentTypeError("port must be between 1 and 65535")
    return port


def run_docs(argv=None):
    parser = argparse.ArgumentParser(
        prog="./manta docs", description="Build or preview local documentation using Docker."
    )
    parser.add_argument("action", nargs="?", choices=("build",), help="build HTML and exit")
    parser.add_argument("--port", type=port_number, default=8000, help="local preview port (default: 8000)")
    args = parser.parse_args(argv)
    container = None
    name = "manta-docs-" + uuid.uuid4().hex[:12]
    try:
        if not shutil.which("docker"):
            raise RuntimeError("Docker was not found. Install and start Docker, then retry.")
        available = subprocess.run(["docker", "info"], capture_output=True, text=True)
        if available.returncode:
            raise RuntimeError(
                "Docker is unavailable. Start Docker and check your access.\n"
                + available.stderr.strip()
            )
        if args.action != "build":
            with socket.socket() as probe:
                probe.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                try:
                    probe.bind(("127.0.0.1", args.port))
                except OSError as error:
                    raise RuntimeError(
                        f"Port {args.port} is unavailable. Try ./manta docs --port 8001"
                    ) from error

        spec = importlib.util.spec_from_file_location("manta_docs_prepare", TOOLING / "prepare.py")
        preparation = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(preparation)
        preparation.prepare(ROOT)
        fingerprint = hashlib.sha256(
            (TOOLING / "Dockerfile").read_bytes()
            + (TOOLING / "requirements.txt").read_bytes()
        ).hexdigest()
        installed = subprocess.run(
            ["docker", "image", "inspect", "--format",
             '{{ index .Config.Labels "manta.docs.fingerprint" }}', IMAGE],
            capture_output=True, text=True,
        )
        if installed.returncode or installed.stdout.strip() != fingerprint:
            subprocess.run(
                ["docker", "build", "--label", f"manta.docs.fingerprint={fingerprint}",
                 "--tag", IMAGE, str(TOOLING)], check=True,
            )

        build = ROOT / ".build/docs"
        command = [
            "docker", "run", "--rm", "--init", "--name", name,
            "--read-only", "--cap-drop=ALL", "--security-opt=no-new-privileges",
            "--tmpfs", "/tmp:rw,nosuid,nodev,size=128m",
        ]
        if hasattr(os, "getuid"):
            command.extend(["--user", f"{os.getuid()}:{os.getgid()}"])
        for relative, target, readonly in (
            ("source", "/input/source", True), ("config", "/input/config", True),
            ("html", "/output/html", False), ("cache", "/output/cache", False),
        ):
            mount = f"type=bind,source={build / relative},target={target}"
            if readonly:
                mount += ",readonly"
            command.extend(["--mount", mount])
        common = ["-c", "/input/config", "-d", "/output/cache/doctrees", "-W", "--keep-going"]
        if args.action == "build":
            command.extend([
                "--network", "none", IMAGE, "sphinx-build", "-b", "html", *common,
                "/input/source", "/output/html",
            ])
        else:
            command.extend([
                "--publish", f"127.0.0.1:{args.port}:8000", IMAGE, "sphinx-autobuild",
                *common, "--host", "0.0.0.0", "--port", "8000", "--watch", "/input/config",
                "/input/source", "/output/html",
            ])
            print(
                f"Local documentation: http://localhost:{args.port}\n"
                "Leave this command running; press Ctrl+C to stop.\n"
                "README uses the current commit; uncommitted README edits are excluded.",
                flush=True,
            )

        container = subprocess.Popen(command)
        last_error = None
        while container.poll() is None:
            time.sleep(1)
            if args.action != "build":
                try:
                    preparation.prepare(ROOT)
                    last_error = None
                except (OSError, ValueError, subprocess.CalledProcessError) as error:
                    if str(error) != last_error:
                        print(f"Documentation preparation failed: {error}", flush=True)
                        last_error = str(error)
        if container.returncode:
            raise RuntimeError("Documentation build/preview failed; see the errors above.")
        if args.action == "build":
            print(f"HTML: {build / 'html/index.html'}")
        return 0
    except KeyboardInterrupt:
        print("\nStopping documentation preview.", flush=True)
        return 0
    except (OSError, ValueError, RuntimeError, subprocess.CalledProcessError) as error:
        print(f"Documentation error: {error}", flush=True)
        return 1
    finally:
        if container is not None and container.poll() is None:
            subprocess.run(["docker", "stop", "--time", "3", name], capture_output=True)
            try:
                container.wait(timeout=10)
            except subprocess.TimeoutExpired:
                container.terminate()
                container.wait()
