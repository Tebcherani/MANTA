# Odyssey AUV Simulator

This project provides a Dockerized version of the MIT Sea Grant Odyssey Autonomous Underwater Vehicle (AUV) simulator.

Docker provides a consistent Linux environment for building and running the simulator without requiring the simulator's software dependencies to be installed directly on the host machine.

The repository provides two Docker environments:

- **Runtime environment** — for building and running the Odyssey simulator normally.
- **Development environment** — for interactive development, compiling source code, and building the simulator's static libraries.

The development environment does **not** replace the normal Docker workflow.

---

## Prerequisites

Docker must be installed before building or running the simulator.

Follow the official Docker installation instructions for your operating system:

https://docs.docker.com/engine/install/

After installation, verify Docker is working:

```bash
docker --version
```

---

## Initial Setup

The Docker Compose configuration uses a `.env` file to determine the user and group IDs that should be used when running the simulator.

This ensures that output files created in bind-mounted directories are owned by the current user instead of `root`.

Copy the example environment file:

```bash
cp .env.example .env
```

Determine your user and group IDs:

```bash
id -u
id -g
```

Open `.env` and replace the placeholder values with the values returned by the commands above.

For example:

```text
HOST_UID=1000
HOST_GID=1000
```

This setup only needs to be performed once on each machine.

---

# Running the Simulator

The normal simulator uses:

```text
Dockerfile
compose.yaml
```

Users who only want to build and run the simulator do **not** need the development container.

## Build the Simulator Image

Build the normal runtime image with:

```bash
docker compose build
```

The image should be rebuilt when the simulator source code or runtime Dockerfile changes.

## Run the Simulator

Start the configured simulation with:

```bash
docker compose up
```

The vehicle configuration, mission, fault configuration, and output paths are defined in `compose.yaml`.

So the normal workflow remains:

```bash
docker compose build
docker compose up
```

---

# Development Environment

The repository also provides a separate development container.

It uses:

```text
Dockerfile.dev
compose.dev.yaml
```

This environment provides an interactive Linux shell containing the compiler and native development libraries required by Odyssey.

It is useful for:

- compiling simulator source code;
- building static libraries;
- debugging build errors;
- developing on macOS without installing Linux-specific dependencies locally.

The development environment is separate from the normal runtime image.

## Build the Development Image

Build it with:

```bash
docker compose -f compose.dev.yaml build
```

This normally only needs to be repeated when `Dockerfile.dev` or its installed dependencies change.

## Enter the Development Container

Run:

```bash
docker compose -f compose.dev.yaml run --rm dev
```

This opens an interactive shell inside the Linux development environment.

The repository is bind-mounted at:

```text
/odyssey
```

Therefore, source changes made on the host are immediately visible inside the container, and build products generated inside the container are written back into the host repository.

---

# Building the Simulator Source

Native simulator code should be compiled inside the development container.

Start the container:

```bash
docker compose -f compose.dev.yaml run --rm dev
```

Then, inside the container:

```bash
make clean
make
```

This builds the static libraries and the standalone `sim` executable.

---

# Building the Static Libraries

The simulator source is divided into static libraries so its components can also be consumed by other software.

Inside the development container, run:

```bash
make libraries
```

For a clean rebuild:

```bash
make clean
make libraries
```

The generated libraries are stored under:

```text
lib/
```

and include:

```text
lib/
├── libodyssey.a
├── libsim_common.a
├── libsim_sensors.a
├── libsimulation.a
└── libworld_sim.a
```

| Library | Purpose |
|---|---|
| `libodyssey.a` | Odyssey vehicle, dynamics, autonomy, control, navigation, and vehicle-specific simulation code |
| `libsim_sensors.a` | Reusable simulated sensor models |
| `libsim_common.a` | Common simulator utilities |
| `libsimulation.a` | Simulation configuration, state, and orchestration functionality |
| `libworld_sim.a` | World/environment simulation functionality |

The `lib/` directory contains generated build products and should not be committed to Git.

---

# Runtime vs. Development Commands

The two Docker workflows serve different purposes.

### Run Odyssey normally

```bash
docker compose build
docker compose up
```

This uses:

```text
compose.yaml
    ↓
Dockerfile
    ↓
Odyssey runtime image
```

### Develop or build libraries

```bash
docker compose -f compose.dev.yaml build
docker compose -f compose.dev.yaml run --rm dev
```

Then, inside the container:

```bash
make libraries
```

This uses:

```text
compose.dev.yaml
    ↓
Dockerfile.dev
    ↓
Odyssey development environment
```

---

# Changing Missions and Faults

Runtime vehicle configuration is defined in `compose.yaml`.

Set `MISSION_PATH` to the complete path of the mission file inside the container:

```yaml
MISSION_PATH: /odyssey/assets/Missions/dive240.mi
```

Set `FAULT_PATH` to the complete path of a fault file:

```yaml
FAULT_PATH: /odyssey/assets/Faults/example.fault
```

If no fault file should be used:

```yaml
FAULT_PATH: ""
```

Each vehicle can use its own mission and fault configuration.

---

# Input and Output

Docker bind mounts provide access to simulator inputs and write simulation results directly to the host machine.

| Directory | Purpose |
|---|---|
| `assets/Missions/` | Mission files |
| `assets/Vehicles/` | Vehicle configuration files |
| `assets/Faults/` | Fault configuration files |
| `output/Data/` | Odyssey simulation data (`.obd`) |
| `output/Logs/` | Simulation logs |
| `build/` | Intermediate compiler output |
| `lib/` | Generated static libraries |

---

# Analyzing Results

Analyze simulation results with:

```bash
./ody review2
```

---

# Typical User Workflow

For someone who only wants to run Odyssey:

```bash
docker compose build
docker compose up
```

Then analyze the results:

```bash
./ody review2
```

---

# Typical Developer Workflow

Build the development image once:

```bash
docker compose -f compose.dev.yaml build
```

Enter the development environment:

```bash
docker compose -f compose.dev.yaml run --rm dev
```

Inside the container, rebuild the libraries:

```bash
make clean
make libraries
```

Or build the complete standalone simulator:

```bash
make clean
make
```

Exit the development container with:

```bash
exit
```

The normal runtime simulator can then still be started with:

```bash
docker compose up
```