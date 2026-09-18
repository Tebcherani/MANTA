from pathlib import Path

# ----------------------------------------------------------------- 
# Root
# -----------------------------------------------------------------
ROOT = Path(__file__).resolve().parent.parent

# ----------------------------------------------------------------- 
# Directories
# -----------------------------------------------------------------
REVIEW_DIR = ROOT / "review" / "manta_review"
VEHICLE_DIR = ROOT / "vehicles"

# ----------------------------------------------------------------- 
# Executables
# -----------------------------------------------------------------
VENV_EXEC = ROOT / "review" / ".venv" / "bin" / "python"

# ----------------------------------------------------------------- 
# Files
# -----------------------------------------------------------------

# .yaml
DOCKER_COMPOSE_PATH = ROOT / "generated" / "compose.ros.yaml"
CONFIG_PATH = ROOT / "config" / "simulation.yaml"

# .csv
SWEEP_SUMMARY_PATH = ROOT / "output" / "monte_carlo_sweep" / "sweep_summary.csv"