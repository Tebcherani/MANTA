#!/usr/bin/env bash

set -e

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

sudo docker run --rm \
  --user "$(id -u):$(id -g)" \
  --mount type=bind,source="$PROJECT_DIR/SMission",target=/odyssey/SMission,readonly \
  --mount type=bind,source="$PROJECT_DIR/Vehicle",target=/odyssey/Vehicle,readonly \
  --mount type=bind,source="$PROJECT_DIR/SFault",target=/odyssey/SFault,readonly \
  --mount type=bind,source="$PROJECT_DIR/SEnvironment",target=/odyssey/SEnvironment,readonly \
  --mount type=bind,source="$PROJECT_DIR/SData",target=/odyssey/SData \
  odyssey_sim "$@"