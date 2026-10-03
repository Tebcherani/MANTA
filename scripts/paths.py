from pathlib import Path

# ----------------------------------------------------------------- 
# Root
# -----------------------------------------------------------------
ROOT = Path(__file__).resolve().parent.parent

# ----------------------------------------------------------------- 
# Mission-specific
# -----------------------------------------------------------------
PICKET_LINE_CONFIG_PATH = ROOT / "config" / "scenarios" / "picket_line.yaml"
PICKET_LINE_MISSION_GENERATOR_PATH = ROOT / "missions" / "picket_line" / "generate_picket_line_mission.py"
PICKET_LINE_MONTE_CARLO_OUTPUT_DIR = ROOT / "output" / "picket_line" / "monte_carlo"
PICKET_LINE_MONTE_CARLO_SUMMARY_PATH = PICKET_LINE_MONTE_CARLO_OUTPUT_DIR / "summary.csv"
PICKET_LINE_SWEEP_GENERATED_DIR = ROOT / "generated" / "picket_line" / "monte_carlo_sweep"
PICKET_LINE_SWEEP_OUTPUT_DIR = ROOT / "output" / "picket_line" / "monte_carlo_sweep"
PICKET_LINE_SWEEP_SUMMARY_PATH = PICKET_LINE_SWEEP_OUTPUT_DIR / "sweep_summary.csv"


# ----------------------------------------------------------------- 
# Directories
# -----------------------------------------------------------------
# Code
REVIEW_DIR = ROOT / "review" / "manta_review"
VEHICLE_DIR = ROOT / "vehicles"

# ----------------------------------------------------------------- 
# Executables
# -----------------------------------------------------------------
VENV_EXEC = ROOT / "review" / ".venv" / "bin" / "python"

# ----------------------------------------------------------------- 
# Files
# -----------------------------------------------------------------

# .env
ENV_PATH = ROOT / "generated" / "manta.env"

# .sh
CLEANUP_PATH = ROOT / "scripts" / "cleanup_monte_carlo.sh"

# .yaml
DEV_COMPOSE_PATH = ROOT / "compose.dev.yaml"
DOCKER_COMPOSE_PATH = ROOT / "generated" / "compose.ros.yaml"