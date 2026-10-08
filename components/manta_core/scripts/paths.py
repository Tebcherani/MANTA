from pathlib import Path

# ----------------------------------------------------------------- 
# Root
# -----------------------------------------------------------------
ROOT = Path(__file__).resolve().parents[3]

# ----------------------------------------------------------------- 
# Directories
# -----------------------------------------------------------------
# Code
MISSION_DIR = ROOT / "missions"
REVIEW_DIR = MISSION_DIR
COMPONENT_DIR = ROOT / "components"

# ----------------------------------------------------------------- 
# Executables
# -----------------------------------------------------------------
VENV_EXEC = ROOT / ".venv" / "bin" / "python"

# ----------------------------------------------------------------- 
# Files
# -----------------------------------------------------------------

# .env
ENV_PATH = ROOT / "manta.env"

# .sh
CLEANUP_PATH = ROOT / "scripts" / "cleanup_monte_carlo.sh"

# .yaml
DEV_COMPOSE_PATH = ROOT / "compose.dev.yaml"
DOCKER_COMPOSE_PATH = ROOT / "compose.ros.yaml"