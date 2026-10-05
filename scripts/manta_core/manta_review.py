import subprocess
import sys

from paths import ROOT, VENV_EXEC, REVIEW_DIR


def run_review(scenario: str, plot: str) -> int:
    """Validate the review environment and run the selected scenario plot."""

    scripts = {
        "picket_line": {
            "tracks": "plot_tracks.py",
            "sweep": "plot_sweep_metrics.py",
        },
        "trailing": {
            "tracks": "plot_tracks.py",
            "sweep": "plot_sweep_metrics.py",
        },
    }

    if scenario not in scripts:
        print(f"Review is not configured for scenario '{scenario}'.", file=sys.stderr)
        return 1

    if plot not in scripts[scenario]:
        print(f"Review plot '{plot}' is not configured for scenario '{scenario}'.", file=sys.stderr)
        return 1

    script = REVIEW_DIR / scenario / "review" / scripts[scenario][plot]

    if not VENV_EXEC.is_file():
        print(f"Review environment is missing: {VENV_EXEC}", file=sys.stderr)
        return 1

    if not script.is_file():
        print(f"Review script is missing: {script}", file=sys.stderr)
        return 1

    if plot == "sweep":
        summary_path = ROOT / "output" / scenario / "monte_carlo_sweep" / "sweep_summary.csv"

        if not summary_path.is_file():
            print(f"Monte Carlo sweep summary is missing: {summary_path}", file=sys.stderr)
            return 1

    return subprocess.run([str(VENV_EXEC), str(script)], cwd=script.parent, check=False).returncode