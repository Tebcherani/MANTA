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
SCENARIO_CONFIG_DIR = ROOT / "config"
VEHICLE_DIR = ROOT / "vehicles"

# ----------------------------------------------------------------- 
# Executables
# -----------------------------------------------------------------
VENV_EXEC = ROOT / "review" / ".venv" / "bin" / "python"

# ----------------------------------------------------------------- 
# Files
# -----------------------------------------------------------------

# .env
ENV_PATH = ROOT / "vehicles" / "odyssey" / "manta.env"

# .sh
CLEANUP_PATH = ROOT / "scripts" / "cleanup_monte_carlo.sh"

# .yaml
DEV_COMPOSE_PATH = ROOT / "compose.dev.yaml"
DOCKER_COMPOSE_PATH = ROOT / "vehicles" / "odyssey" / "compose.ros.yaml"