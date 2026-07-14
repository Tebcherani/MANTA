#!/usr/bin/env bash
set -euo pipefail

# Reproducible Python setup for the Ubuntu development repository.
# May be run from any directory. Set PYTHON to override the interpreter.

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "$SCRIPT_DIR/.." && pwd)"
VENV_DIR="$REPO_ROOT/.venv"
REQ_FILE="$REPO_ROOT/requirements.txt"

if [ -n "${PYTHON-}" ]; then
  PY="$PYTHON"
elif command -v python3 >/dev/null 2>&1; then
  PY=python3
elif command -v python >/dev/null 2>&1; then
  PY=python
else
  echo "ERROR: no Python executable found. Install Python or set PYTHON." >&2
  exit 2
fi

echo "Repository: $REPO_ROOT"
echo "Using Python: $("$PY" --version 2>&1) ($("$PY" -c 'import sys; print(sys.executable)'))"

if ! "$PY" -c 'import sys; raise SystemExit(sys.version_info < (3, 9))'; then
  echo "ERROR: Python 3.9 or newer is required." >&2
  exit 2
fi

if [ -x "$VENV_DIR/bin/python" ]; then
  echo "Virtual environment already exists at $VENV_DIR"
elif [ -e "$VENV_DIR" ]; then
  echo "ERROR: $VENV_DIR exists but is not a valid Ubuntu virtual environment." >&2
  echo "Remove it and rerun: rm -rf '$VENV_DIR'" >&2
  exit 3
else
  echo "Creating virtual environment at $VENV_DIR..."
  if ! "$PY" -m venv "$VENV_DIR"; then
    echo "ERROR: venv creation failed. Run:" >&2
    echo "  sudo apt install python3-venv" >&2
    exit 4
  fi
fi

VENV_PYTHON="$VENV_DIR/bin/python"

if [ -f "$REQ_FILE" ]; then
  echo "Installing workspace dependencies..."
  "$VENV_PYTHON" -m pip install --upgrade pip
  (cd "$REPO_ROOT" && "$VENV_PYTHON" -m pip install -r "$REQ_FILE")
else
  echo "ERROR: dependency manifest not found: $REQ_FILE" >&2
  exit 5
fi

"$VENV_PYTHON" -c 'import matplotlib, numpy, scipy, xarray, PyQt5, ody_review; print("Python dependency check passed")'

echo
echo "Setup complete."
echo "Activate with: source '$VENV_DIR/bin/activate'"
