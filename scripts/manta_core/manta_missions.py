import subprocess
import sys
from pathlib import Path

from paths import ROOT

def generate_missions(
    config: dict,
    config_path: Path,
    output_directory: Path | None = None,
) -> None:
    """Run the configured generator with an explicit output directory."""

    settings = config.get("mission_generation")
    if not isinstance(settings, dict):
        raise ValueError("Configuration is missing mission_generation.")

    generator = settings.get("generator")
    if not isinstance(generator, str) or not generator.strip():
        raise ValueError("Configuration is missing mission_generation.generator.")

    generator_path = (ROOT / generator).resolve()

    if not generator_path.is_relative_to(ROOT.resolve()):
        raise ValueError("Mission generator must be inside MANTA.")
    if not generator_path.is_file():
        raise ValueError(f"Mission generator does not exist: {generator_path}")

    # Sweeps can supply their own output directory.
    if output_directory is None:
        configured_output = settings.get("output_directory")
        if not isinstance(configured_output, str) or not configured_output.strip():
            raise ValueError(
                "Configuration is missing mission_generation.output_directory."
            )
        output_directory = ROOT / configured_output

    output_directory = (ROOT / output_directory).resolve()

    if not output_directory.is_relative_to(ROOT.resolve()):
        raise ValueError("Mission output directory must be inside MANTA.")

    command = [
        sys.executable,
        str(generator_path),
        str(config_path.resolve()),
        str(output_directory),
    ]

    result = subprocess.run(command, cwd=ROOT, check=False)

    if result.returncode != 0:
        raise ValueError(f"Mission generation failed: {generator_path}")
    
