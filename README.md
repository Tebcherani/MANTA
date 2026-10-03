# MANTA

MANTA is a ROS 2 and Docker-based simulation framework for evaluating coordinated autonomous underwater vehicle (AUV) fleets in a shared world and simulation timeline. The current scenario models a fleet of Odyssey AUVs patrolling a picket line while stochastic whale groups cross the monitored region and produce acoustic calls.

MANTA provides configuration-driven fleet generation, repeatable Docker execution, synchronized simulation stepping, Odyssey vehicle simulation, stochastic whale movement and calling, range-based whale-call detection, Monte Carlo experiments, and fleet-size sweeps.

The current fixed-radius detector is an engineering baseline, not a complete acoustic propagation or detector-performance model.

## Quick start

### Requirements

Install Git, Docker Engine with the Docker Compose plugin, Python 3, and PyYAML. Verify the main dependencies:

```bash
git --version
docker --version
docker compose version
python3 --version
```

### Clone

MANTA is a single repository and does not use Git submodules:

```bash
git clone <MANTA_REPOSITORY_URL>
cd MANTA
```

### Build

```bash
./manta build
```

This command validates `config/simulation.yaml`, generates runtime files, builds the development image, runs `make generate_odyssey`, builds the Odyssey libraries, and builds the ROS 2 workspace in Release mode.

### Run

```bash
./manta run
```

The runner regenerates the missions, environment file, and runtime Compose file before starting the simulation. Stop and remove the generated services with:

```bash
./manta down
```

### Review

Plot trajectories, calls, and detections:

```bash
./manta review tracks
```

Plot fleet-size sweep metrics:

```bash
./manta review sweep
```

The review commands use `review/.venv/bin/python`.

## Command reference

```text
./manta [generate|build|dev|run|down|review|monte-carlo|monte-carlo-sweep] [plot]
```

Running `./manta` without a command is equivalent to `./manta run`.

| Command | Purpose |
| --- | --- |
| `./manta generate` | Generate missions, `generated/manta.env`, and `generated/compose.ros.yaml`. |
| `./manta build` | Generate runtime files, build the Docker image, build Odyssey, and build the ROS workspace. |
| `./manta dev` | Generate runtime files and open an interactive development container. |
| `./manta run` | Generate runtime files and start the configured simulation services. |
| `./manta down` | Stop and remove the generated runtime services and orphans. |
| `./manta review tracks` | Plot vehicle and whale trajectories, calls, and detections. |
| `./manta review sweep` | Plot aggregate fleet-size sweep metrics. |
| `./manta monte-carlo` | Run repeated realizations until convergence or the configured maximum. |
| `./manta monte-carlo-sweep` | Run an adaptive Monte Carlo experiment for each configured fleet size. |

Show command-line help with:

```bash
./manta --help
```

## Configuration

The primary user configuration is `config/simulation.yaml`.

### Output

```yaml
output:
  data_directory: ./output/Data
  log_directory: ./output/Logs
  auv_sample_period: 5.0
  whale_sample_period: 60.0
```

These settings select the output directories and sampling periods for vehicle and whale state logs.

### Simulation

```yaml
simulation:
  timestep: 0.02
  control_period: 10
  max_steps: 12960000
  steps_per_message: 10000
```

- `timestep` is the duration of one physics cycle in seconds.
- `control_period` is the number of physics cycles between vehicle sensor and control updates.
- `max_steps` is the maximum number of physics cycles.
- `steps_per_message` is the normal maximum number of cycles assigned by one `SimStep` message.

The orchestrator shortens a batch when necessary so it stops at the cycle containing the next scheduled whale call.

### Scenario

The current scenario type is `picket_line`. Its configuration defines mission duration, monitoring-line geometry, AUV patrol behavior, detection radius, and whale movement, grouping, and calling parameters.

### Vehicles

Each item in `vehicles` defines one simulated vehicle:

```yaml
vehicles:
  - id: auv_1
    type: odyssey
    mission: ./generated/missions/picket_line_auv_1.mi
    model: ./config/vehicles/Borealis.vd
    fault: ""
```

- `id` uniquely identifies the vehicle.
- `type` selects `vehicles/<type>/manta_plugin.yaml`.
- `mission` identifies the mission file.
- `model` identifies the vehicle model.
- `fault` identifies an optional fault file; an empty string disables it.

Adding or removing entries changes the runtime fleet. The runner generates one Compose service for every configured vehicle.

### Monte Carlo and sweep

The `monte_carlo` section controls realization limits, checkpoint size, workers, timeouts, retries, confidence level, target confidence-interval half-width, stability tolerance, and the base seed. The `sweep` section defines the inclusive fleet-size range.

## Generated files

MANTA regenerates these files from `config/simulation.yaml`:

```text
generated/
├── manta.env
├── compose.ros.yaml
├── missions/
│   ├── picket_line_auv_1.mi
│   └── ...
└── monte_carlo_sweep/
    └── simulation_vehicles_XX.yaml
```

Do not edit generated files as the primary configuration. Change `config/simulation.yaml` and rerun `./manta generate`. Configured paths must remain inside MANTA because the repository is bind-mounted at `/odyssey`.

## Architecture

### ROS 2 packages

```text
ros_ws/src/
├── manta_interfaces/
├── sim_orchestrator/
├── whale_detection_sim/
└── whale_sim/
```

The Odyssey plugin generates and builds its ROS integration during `./manta build`.

### Runtime nodes

#### `SimOrchestratorNode`

The orchestrator initializes the clock and world, publishes startup information, waits for every vehicle, requests scheduled whale calls, publishes batched `SimStep` messages, waits for `SimStepResult`, pauses for detection processing at call cycles, and coordinates shutdown.

#### `OdysseyNode`

One Odyssey node runs per configured vehicle. It receives startup data, initializes its simulator and mission, processes `SimStep`, publishes `VehicleState`, returns `SimStepResult`, and writes vehicle data and final metrics.

#### `WhaleNode`

The whale node generates groups, trajectories, correlated calling activity, and call events. It advances whale lifecycle state, records scheduled whale positions, and supplies the next scheduled call through `GetNextWhaleCall`.

#### `OdysseyDetectionNode`

The detection node stores the latest vehicle states, receives `WhaleCall`, evaluates every applicable vehicle, records detections, and publishes `WhaleCallProcessed` so the orchestrator can continue.

### Synchronization sequence

```text
Orchestrator publishes startup information
    ↓
Each Odyssey initializes and publishes ready
    ↓
Orchestrator requests the next whale call
    ↓
Orchestrator publishes a SimStep batch
    ↓
WhaleNode and Odyssey nodes advance
    ↓
Odyssey nodes publish VehicleState and SimStepResult
    ↓
Orchestrator waits for every active vehicle
    ↓
At a whale-call cycle:
    Orchestrator publishes WhaleCall
    Detection node evaluates the vehicle states
    Detection node publishes WhaleCallProcessed
    Orchestrator requests the following call
    ↓
Simulation continues or shuts down
```

The PlantUML source is `docs/manta_sim_sequence.puml`.

## Vehicle plugins

Vehicle implementations live under `vehicles/<plugin-id>/`. Each `manta_plugin.yaml` manifest defines the runtime image, ROS package, executable, and environment-variable mappings. A configured vehicle's `type` must match the manifest plugin ID.

The current plugin is `vehicles/odyssey/`. It is part of the MANTA repository and is not a Git submodule.

## Development environment

Open the development container with:

```bash
./manta dev
```

The container provides ROS 2 Jazzy, mounts the repository at `/odyssey`, uses the host UID and GID, and uses `ROS_DOMAIN_ID=1`. Host edits are immediately visible inside the container.

After changing native code, ROS code, CMake, package metadata, or the Docker environment, rebuild with:

```bash
./manta build
```

## Output

A normal run writes to the configured directories, normally:

```text
output/
├── Data/
│   ├── whales.csv
│   ├── whale_calls.csv
│   ├── whale_detections.csv
│   ├── <vehicle>.obd
│   └── <vehicle>_metrics.csv
└── Logs/
    ├── sim_orchestrator.log
    └── <node and vehicle logs>
```

Aggregate vehicle metrics include mass, energy consumed, and mean speed. Detection results count unique simulated whale IDs with at least one detected call; this is simulation ground-truth coverage, not acoustic individual identification.

Runtime output, generated files, and build products normally should not be committed.

## Monte Carlo

Run repeated realizations of the configured fleet:

```bash
./manta monte-carlo
```

Each realization receives a deterministic seed, runs in an isolated Compose project and ROS domain, writes its own data and logs, retries failures as configured, and records detection fraction, total fleet mass, total energy, and mean vehicle speed.

Successful realizations are summarized in `output/monte_carlo/summary.csv`. Compatible saved results are loaded so an interrupted experiment can resume. Convergence is checked at batch boundaries using the detection-fraction confidence-interval half-width and cumulative-mean stability.

## Fleet-size sweep

```bash
./manta monte-carlo-sweep
```

For every fleet size, MANTA clones the first configured vehicle as a template, assigns IDs `auv_1` through `auv_N`, generates missions, runs the adaptive Monte Carlo experiment, and calculates fleet-level means and confidence intervals.

```text
output/monte_carlo_sweep/
├── sweep_summary.csv
└── vehicles_XX/
    ├── summary.csv
    └── realization_XXXX/
```

Plot the completed sweep with `./manta review sweep`.

## Repository structure

```text
MANTA/
├── config/
├── docs/
├── generated/
├── missions/
├── output/
├── review/
├── ros_ws/src/
├── scripts/
│   ├── manta_core/
│   ├── manta_runner.py
│   └── paths.py
├── vehicles/odyssey/
├── compose.dev.yaml
├── Dockerfile.dev
├── manta
└── README.md
```

Root-level `build/`, `install/`, and `log/` directories are generated by `colcon` and should not be committed.

## Git workflow

Create a feature branch:

```bash
git switch main
git pull
git switch -c feature/short-description
```

Commit and push it:

```bash
git add <files>
git commit -m "Describe the change"
git push -u origin feature/short-description
```

Open a pull request, merge it into `main`, then update locally with `git switch main && git pull`.

## Troubleshooting

### `install/setup.bash` is missing or ROS packages are not found

```bash
./manta build
```

Generated services automatically source ROS 2 Jazzy and `/odyssey/ros_ws/install/setup.bash`.

### Runtime configuration is stale

```bash
./manta generate
```

### Old containers remain

```bash
./manta down
```

### Review environment is missing

Restore or create `review/.venv/bin/python` before using `./manta review`.

## Documentation

- `docs/manta_sim_sequence.puml` — ROS node sequence diagram.
- `docs/Whale_Model_Memo.pdf` — whale-model equations, assumptions, parameters, and evidence status.
