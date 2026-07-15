#!/usr/bin/env bash
set -euo pipefail

if ! command -v apt-get >/dev/null 2>&1; then
  echo "ERROR: this helper expects an Ubuntu/Debian system with apt-get." >&2
  exit 2
fi

packages=(
  build-essential
  gcc
  git
  make
  python3
  python3-dev
  python3-pip
  python3-venv
  libgl1
  libxcb-cursor0
  libxcb-xinerama0
  libxkbcommon-x11-0
)

echo "Installing Ubuntu system packages needed for build and GUI work..."
sudo apt-get update
sudo apt-get install -y "${packages[@]}"

echo
echo "System package setup complete."
