#!/usr/bin/env bash

set -u

mapfile -t projects < <(
    docker ps -a --format '{{.Label "com.docker.compose.project"}}' |
        awk '/^manta-mc-[0-9]+$/' |
        sort -u
)

if ((${#projects[@]} == 0)); then
    echo "No MANTA Monte Carlo containers are running."
    exit 0
fi

for project in "${projects[@]}"; do
    echo "Stopping ${project}..."

    mapfile -t containers < <(
        docker ps -aq --filter "label=com.docker.compose.project=${project}"
    )

    if ((${#containers[@]} > 0)); then
        docker rm -f "${containers[@]}" >/dev/null
    fi

    mapfile -t networks < <(
        docker network ls -q --filter "label=com.docker.compose.project=${project}"
    )

    if ((${#networks[@]} > 0)); then
        docker network rm "${networks[@]}" >/dev/null 2>&1 || true
    fi
done

echo "Stopped all MANTA Monte Carlo Docker projects."
