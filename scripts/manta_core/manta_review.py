import subprocess
import sys

from paths import VENV_EXEC, REVIEW_DIR, SWEEP_SUMMARY_PATH


def run_review(plot: str) -> int:
    """Validate the review environment and run the selected plotting script."""
    
    scripts = {"tracks": "plot_tracks.py", "sweep": "plot_sweep_metrics.py"}
    script = REVIEW_DIR / scripts[plot]

    if not VENV_EXEC.is_file():
        print(f"Review venv_dir is missing: {VENV_EXEC}", file=sys.stderr)
        return 1
    if not script.is_file():
        print(f"Review script is missing: {script}", file=sys.stderr)
        return 1

    command = [str(VENV_EXEC), str(script)]

    if plot == "sweep":
        if not SWEEP_SUMMARY_PATH.is_file():
            print(f"Monte Carlo sweep summary is missing: {SWEEP_SUMMARY_PATH}", file=sys.stderr)
            return 1
        
    return subprocess.run(command, cwd=script.parent, check=False).returncode