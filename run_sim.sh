#!/usr/bin/env bash

sudo docker run --rm \
  --user "$(id -u):$(id -g)" \
  --mount type=bind,source="$PWD/SMission",target=/odyssey/SMission,readonly \
  --mount type=bind,source="$PWD/Vehicle",target=/odyssey/Vehicle,readonly \
  --mount type=bind,source="$PWD/SFault",target=/odyssey/SFault,readonly \
  --mount type=bind,source="$PWD/SEnvironment",target=/odyssey/SEnvironment,readonly \
  --mount type=bind,source="$PWD/SData",target=/odyssey/SData \
  odyssey_sim "$@"   