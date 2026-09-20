from pathlib import Path

# ----------------------------------------------------------------- 
# Root
# -----------------------------------------------------------------
ROOT = Path(__file__).resolve().parent.parent

# ----------------------------------------------------------------- 
# Directories
# -----------------------------------------------------------------
# Code
REVIEW_DIR = ROOT / "review" / "manta_review"
VEHICLE_DIR = ROOT / "vehicles"

# Generated
SWEEP_GENERATED_DIR = ROOT / "generated" / "monte_carlo_sweep"

# Output
SWEEP_OUTPUT_DIR = ROOT / "output" / "monte_carlo_sweep"

# ----------------------------------------------------------------- 
# Executables
# -----------------------------------------------------------------
VENV_EXEC = ROOT / "review" / ".venv" / "bin" / "python"

# ----------------------------------------------------------------- 
# Files
# -----------------------------------------------------------------

# .csv
SWEEP_SUMMARY_PATH = SWEEP_OUTPUT_DIR / "sweep_summary.csv"
MONTE_CARLO_SUMMARY_PATH = ROOT / "output" / "monte_carlo" / "summary.csv"

# .env
ENV_PATH = ROOT / "generated" / "manta.env"

# .py
MISSION_GENERATOR_PATH = ROOT / "missions" / "picket_line" / "generate_picket_line_mission.py"

# .sh
CLEANUP_PATH = ROOT / "scripts" / "cleanup_monte_carlo.sh"

# .yaml
DEV_COMPOSE_PATH = ROOT / "compose.dev.yaml"
DOCKER_COMPOSE_PATH = ROOT / "generated" / "compose.ros.yaml"
CONFIG_PATH = ROOT / "config" / "simulation.yaml"