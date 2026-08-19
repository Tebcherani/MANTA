# Odyssey AUV Simulator (Docker)

This project provides a Dockerized, two-vehicle version of the MIT Sea Grant Odyssey Autonomous Underwater Vehicle (AUV) simulator. Docker allows the simulator to run in a consistent environment without requiring the user to manually install and configure its software dependencies.

## Prerequisites

Docker must be installed before building or running the simulator.

Follow the official Docker installation instructions for your operating system:

https://docs.docker.com/engine/install/

After installation, verify Docker is working by running:

```bash
docker --version
```

## Initial Setup

The Docker Compose configuration uses a `.env` file to determine the user and group IDs that should be used when running the simulator. This ensures that output files created in bind-mounted directories are owned by the current user instead of `root`.

Copy the example environment file:

```bash
cp .env.example .env
```

Determine your user and group IDs:

```bash
id -u
id -g
```

Open `.env` and replace the placeholder values with the values returned by the above commands.

For example:

```text
HOST_UID=1000
HOST_GID=1000
```

This setup only needs to be performed once on each machine.

## Building the Docker Image

The simulator image only needs to be rebuilt when the simulator source code or Dockerfile changes.

Build the image by executing:

```bash
docker compose build
```

## Running the Simulator

Run the simulator container using:

```bash
docker compose up
```

## Analysing Results

Analyze results using:

```bash
./ody review2
```

## Changing the Mission or Faults

In `compose.yaml`, simply set `MISSION_PATH` or `FAULT_PATH` to the mission or fault files, respectively, that you would like to use for the simulation. 

## Input and Output

The simulator uses Docker bind mounts to access input files and write simulation results directly to the host machine.

The following directories are used:

| Directory | Purpose |
|-----------|---------|
| `assets/Missions/` | Mission files |
| `assets/Vehicles/` | Vehicle configuration |
| `assets/Faults/` | Fault configuration |
| `output/Data` | Simulation output |
| `output/Logs` | Simulation logs |
