#!/usr/bin/env bash

set -e

# Determine the directory containing this script.
PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

sudo docker build \
  --tag odyssey_sim \
  "$PROJECT_DIR"