FROM python:3.12-slim-bookworm@sha256:e697c81897302a4c4333bade000d98038fbb21eb064f8ee1fd42d02ebcd9d337

ENV PYTHONDONTWRITEBYTECODE=1 \
    PYTHONUNBUFFERED=1 \
    HOME=/tmp

RUN apt-get update && apt-get install -y --no-install-recommends \
    default-jre-headless graphviz fonts-dejavu-core \
    && rm -rf /var/lib/apt/lists/*

RUN python - <<'PY'
import hashlib
import pathlib
import urllib.request

url = "https://github.com/plantuml/plantuml/releases/download/v1.2025.10/plantuml.jar"
data = urllib.request.urlopen(url, timeout=120).read()
expected = "4a01ea09b317180fb8e7eef712dfdca725409d2ee1919e4b5adfe9d8362b6fe5"
if hashlib.sha256(data).hexdigest() != expected:
    raise SystemExit("PlantUML download checksum mismatch")
pathlib.Path("/opt/plantuml.jar").write_bytes(data)
PY

COPY requirements.txt /tmp/docs-requirements.txt
RUN python -m pip install --no-cache-dir -r /tmp/docs-requirements.txt \
    && rm /tmp/docs-requirements.txt \
    && mkdir -p /input/source /input/config /output/html /output/cache

WORKDIR /input/source
USER 65532:65532
