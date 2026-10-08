import subprocess
import sys

from manta_load import load_yaml
from paths import ROOT, MISSION_DIR, VENV_EXEC


def run_review(scenario: str, plot: str) -> int:
    """Run the review script selected by mission configuration."""

    try:
        config_path = MISSION_DIR / scenario / "config.yaml"
        config = load_yaml(config_path)

        review = config.get("review")
        if not isinstance(review, dict):
            raise ValueError("Configuration is missing review settings.")

        script_setting = review.get(plot)
        if not isinstance(script_setting, str) or not script_setting.strip():
            raise ValueError(f"Review '{plot}' is not configured.")

        script = (ROOT / script_setting).resolve()

        if not script.is_relative_to(ROOT.resolve()):
            raise ValueError("Review script must be inside MANTA.")
        if not script.is_file():
            raise ValueError(f"Review script does not exist: {script}")
        if not VENV_EXEC.is_file():
            raise ValueError(f"Python environment is missing: {VENV_EXEC}")

        return subprocess.run(
            [str(VENV_EXEC), str(script)],
            cwd=script.parent,
            check=False,
        ).returncode

    except (ValueError, OSError) as error:
        print(f"MANTA review error: {error}", file=sys.stderr)
        return 1