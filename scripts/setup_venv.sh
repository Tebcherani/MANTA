#!/usr/bin/env bash
set -euo pipefail

# Cross-platform venv setup script for the workspace.
# Usage: ./scripts/setup_venv.sh
# It will use $PYTHON if set, else try python3 then python.

WORKDIR="$(pwd)"
VENV_DIR="$WORKDIR/venv"
REQ_FILE="$WORKDIR/requirements.txt"

# Find python
if [ -n "${PYTHON-}" ]; then
  PY=${PYTHON}
elif command -v python3 >/dev/null 2>&1; then
  PY=python3
elif command -v python >/dev/null 2>&1; then
  PY=python
else
  echo "ERROR: no python executable found. Install Python or set PYTHON env var to the interpreter path." >&2
  exit 2
fi

echo "Using python: $($PY --version 2>&1)"

# Create venv
if [ -d "$VENV_DIR" ]; then
  echo "Virtual environment already exists at $VENV_DIR"
else
  echo "Creating virtual environment..."
  $PY -m venv "$VENV_DIR"
fi

# Ensure pip exists
PIP="$VENV_DIR/bin/pip"
if [ ! -x "$PIP" ]; then
  echo "pip not found in venv; attempting to bootstrap..."
  $PY -m ensurepip --upgrade
fi

# Install requirements if present
if [ -f "$REQ_FILE" ]; then
  echo "Installing requirements from requirements.txt..."
  "$PIP" install --upgrade pip
  "$PIP" install -r "$REQ_FILE"
else
  echo "No requirements.txt found; skipping pip install. Create requirements.txt to install dependencies."
fi

echo "Setup complete. Activate with: source venv/bin/activate"
