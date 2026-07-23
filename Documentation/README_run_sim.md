# Odyssey AUV Simulator (Docker)

This project provides a Dockerized version of the MIT Sea Grant Odyssey Autonomous Underwater Vehicle (AUV) simulator. Docker allows the simulator to run in a consistent environment without requiring the user to manually install and configure its software dependencies.

## Prerequisites

Docker must be installed before building or running the simulator.

Follow the official Docker installation instructions for your operating system:

https://docs.docker.com/engine/install/

After installation, verify Docker is working by running:

```bash
docker --version
```

## Building the Docker Image

The simulator image only needs to be rebuilt when the simulator source code or Dockerfile changes.

Build the image by executing:

```bash
./build_sim.sh
```

## Running the Simulator

Run the simulator using:

```bash
./run_sim.sh
```

By default, the simulator executes the mission specified in the Docker image.

To run a different mission file, provide its name as a command-line argument:

```bash
./run_sim.sh <mission_file>
```

For example:

```bash
./run_sim.sh dive240.mi
```

## Input and Output

The simulator uses Docker bind mounts to access input files and write simulation results directly to the host machine.

The following directories are used:

| Directory | Purpose |
|-----------|---------|
| `SMission/` | Mission files |
| `Vehicle/` | Vehicle configuration |
| `SFault/` | Fault configuration |
| `SEnvironment/` | Environment configuration |
| `SData/` | Simulation output |

Simulation output is written directly to the `SData` directory on the host filesystem.

## Typical Workflow

```text
Modify simulator source code
        ↓
./build_sim.sh
        ↓
Edit mission or configuration files
        ↓
./run_sim.sh <mission_file>
        ↓
Analyze results in SData/
```