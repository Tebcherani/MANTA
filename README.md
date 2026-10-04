# MANTA

MANTA is a ROS 2 and Docker-based simulation framework for evaluating coordinated autonomous underwater vehicle (AUV) fleets in a shared world and synchronized simulation timeline.

The current whale-monitoring simulations model Odyssey AUVs operating along a picket line while stochastic groups of North Atlantic right whales cross the monitored region and produce acoustic calls. MANTA supports two related scenarios:

- `picket_line`: AUVs continuously patrol assigned line segments and record whale-call detections without leaving the picket line.
- `trailing`: AUVs initially patrol assigned line segments but may follow a whale after detecting one of its calls.

MANTA provides configuration-driven mission generation, repeatable Docker execution, synchronized simulation stepping, Odyssey vehicle simulation, stochastic whale movement and calling, range-based call detection, Monte Carlo experiments, and fleet-size sweeps.

The current fixed-radius detector is an engineering baseline rather than a complete model of acoustic propagation, ambient noise, receiver performance, or detector uncertainty.

## Quick start

### Requirements

Install:

- Git
- Docker Engine or Docker Desktop
- Docker Compose
- Python 3
- PyYAML

Verify the primary dependencies:

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

This command builds the development image, generates the Odyssey data structures, builds the Odyssey libraries, and builds the ROS 2 workspace in Release mode.

The build command is scenario-independent.

### Generate a scenario

Generate detection-only picket-line missions and runtime files:

```bash
./manta generate picket_line
```

Generate trailing missions and runtime files:

```bash
./manta generate trailing
```

Generation creates the selected Odyssey mission files, `generated/manta.env`, and `generated/compose.ros.yaml`.

### Run a scenario

Run the detection-only picket-line scenario:

```bash
./manta run picket_line
```

Run the whale-trailing scenario:

```bash
./manta run trailing
```

The runner regenerates the selected scenario’s missions and runtime files before starting the simulation.

Stop and remove the generated services with:

```bash
./manta down
```

### Review results

Plot vehicle trajectories, whale trajectories, calls, and detections:

```bash
./manta review picket_line tracks
./manta review trailing tracks
```

Plot fleet-size sweep metrics:

```bash
./manta review picket_line sweep
./manta review trailing sweep
```

Review commands use:

```text
review/.venv/bin/python
```

## Command reference

```text
./manta <command> [scenario] [plot]
```

Scenario-dependent commands accept either `picket_line` or `trailing`:

```text
generate
run
monte_carlo
monte_carlo_sweep
review
```

Scenario-independent commands do not accept a scenario:

```text
build
dev
down
```

| Command | Purpose |
| --- | --- |
| `./manta generate <scenario>` | Generate missions and runtime configuration for a scenario. |
| `./manta build` | Build the Docker image, Odyssey libraries, and ROS 2 workspace. |
| `./manta dev` | Open an interactive development container. |
| `./manta run <scenario>` | Generate and run the selected scenario. |
| `./manta down` | Stop and remove generated runtime services and orphans. |
| `./manta review <scenario> tracks` | Plot vehicle and whale trajectories, calls, and detections. |
| `./manta review <scenario> sweep` | Plot fleet-size sweep statistics. |
| `./manta monte_carlo <scenario>` | Run repeated realizations of the selected scenario. |
| `./manta monte_carlo_sweep <scenario>` | Run Monte Carlo experiments over a range of fleet sizes. |

Show command-line help with:

```bash
./manta --help
```

## Scenario configuration

Scenario configurations are stored under:

```text
config/scenarios/
├── picket_line.yaml
└── trailing.yaml
```

The filename and the configured `scenario.type` must agree.

For example:

```yaml
scenario:
  type: picket_line
```

or:

```yaml
scenario:
  type: trailing
```

### Output configuration

Each scenario writes to its own output directories.

Picket line:

```yaml
output:
  data_directory: ./output/picket_line/Data
  log_directory: ./output/picket_line/Logs
  auv_sample_period: 5.0
  whale_sample_period: 60.0
```

Trailing:

```yaml
output:
  data_directory: ./output/trailing/Data
  log_directory: ./output/trailing/Logs
  auv_sample_period: 5.0
  whale_sample_period: 60.0
```

The sample periods control how frequently vehicle and whale states are written to their respective output files. They do not change the internal physics timestep.

### Simulation configuration

```yaml
simulation:
  timestep: 0.02
  control_period: 10
  max_steps: 12960000
  steps_per_message: 10000
```

- `timestep` is the simulated duration of one physics step, in seconds.
- `control_period` is the number of physics steps between vehicle sensor-processing and control updates.
- `max_steps` is the maximum number of physics steps in the simulation.
- `steps_per_message` is the normal maximum number of physics steps assigned in one `SimStep` message.

The orchestrator shortens a batch when necessary so that the batch ends at the step containing the next scheduled whale call.

### Picket-line scenario

In the `picket_line` scenario, every vehicle receives a mission containing `picket_line` but not `whale_trailing`.

Vehicles:

- patrol assigned sections of the monitored line;
- receive simulated whale-call detections;
- log detections;
- remain on their assigned patrols after a detection.

Tracking is disabled through the Odyssey `tracking_enabled` ROS parameter.

### Trailing scenario

In the `trailing` scenario, vehicle missions contain both:

- `picket_line`;
- `trailing`.

Tracking is enabled through the Odyssey `tracking_enabled` ROS parameter. A vehicle may accept its first detected whale as a target and then follow that whale’s simulated position.

If multiple vehicles are configured, each currently has tracking enabled. A future listener/tracker role configuration would be needed to restrict tracking to one designated vehicle.

### Vehicles

Each `vehicles` entry defines one simulated vehicle:

```yaml
vehicles:
  - id: auv_1
    type: odyssey
    mission: ./generated/missions/picket_line/auv_1.mi
    model: ./config/vehicles/Borealis.vd
    fault: ""
```

For the trailing scenario, the mission path instead uses:

```yaml
mission: ./generated/missions/trailing/auv_1.mi
```

The fields are:

- `id`: Unique vehicle identifier.
- `type`: Vehicle plugin ID.
- `mission`: Generated Odyssey mission file.
- `model`: Odyssey vehicle-model file.
- `fault`: Optional fault file; an empty string disables fault injection.

Adding or removing vehicle entries changes the runtime fleet. The runner creates one Docker Compose service per configured vehicle.

## Mission generation

Mission generators are organized by scenario:

```text
missions/
├── picket_line/
│   └── generate_picket_line_mission.py
└── trailing/
    └── generate_trailing_mission.py
```

The selected `scenario.type` determines which generator is executed:

```text
picket_line → missions/picket_line/generate_picket_line_mission.py
trailing    → missions/trailing/generate_trailing_mission.py
```

Generated missions are stored under:

```text
generated/missions/
├── picket_line/
│   ├── auv_1.mi
│   └── ...
└── trailing/
    ├── auv_1.mi
    └── ...
```

Do not edit generated mission files as the primary configuration. Modify the corresponding scenario configuration or mission generator and regenerate the scenario.

## Generated files

MANTA regenerates runtime files for the most recently generated or executed scenario:

```text
generated/
├── manta.env
├── compose.ros.yaml
├── missions/
│   ├── picket_line/
│   └── trailing/
└── monte_carlo_sweep/
    ├── picket_line/
    └── trailing/
```

`manta.env` and `compose.ros.yaml` are replaced when another scenario is generated. Scenario mission files and simulation outputs remain separated by directory.

Configured paths must remain inside the MANTA repository because the repository is bind-mounted into containers at `/odyssey`.

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

The orchestrator:

- initializes the simulation clock and world;
- publishes startup information;
- waits for every configured vehicle;
- requests the next scheduled whale call;
- publishes batched `SimStep` messages;
- waits for `SimStepResult` messages;
- pauses at whale-call steps for detection processing;
- coordinates normal or failed shutdown.

#### `OdysseyNode`

One Odyssey node runs per configured vehicle. It:

- receives startup information;
- initializes the Odyssey simulator and mission;
- processes `SimStep` messages;
- publishes `VehicleState`;
- returns `SimStepResult`;
- writes vehicle data and final metrics.

In `trailing`, it may also accept a detected whale as a tracking target. In `picket_line`, target assignment is disabled.

#### `WhaleNode`

The whale node generates:

- whale groups;
- crossing locations and activation times;
- correlated group and individual movement;
- complete whale trajectories;
- population-level active and silent calling periods;
- scheduled whale calls.

The population calling model alternates between active bouts and silent periods. Calls occurring during active bouts are assigned to whales that are active at the corresponding time.

The whale node advances lifecycle state, publishes whale states, records scheduled whale positions, and supplies the next scheduled call through `GetNextWhaleCall`.

#### `OdysseyDetectionNode`

The detection node:

- stores the latest vehicle states;
- receives each `WhaleCall`;
- evaluates the applicable vehicles;
- records detected calls;
- publishes whale detections;
- publishes `WhaleCallProcessed` so the orchestrator can continue.

The current baseline declares a detection when the call is within a configured radius of a vehicle.

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
At a whale-call step:
    Orchestrator publishes WhaleCall
    Detection node evaluates vehicle states
    Detection node publishes detections
    Detection node publishes WhaleCallProcessed
    Orchestrator requests the following call
    ↓
Simulation continues or shuts down
```

The PlantUML source is:

```text
docs/manta_sim_sequence.puml
```

## Vehicle plugins

Vehicle implementations live under:

```text
vehicles/<plugin-id>/
```

Each `manta_plugin.yaml` manifest defines the runtime image, ROS package, executable, and environment-variable mappings.

A configured vehicle’s `type` must match the plugin manifest ID.

The current plugin is:

```text
vehicles/odyssey/
```

It is part of the MANTA repository and is not a Git submodule.

## Development environment

Open the development container with:

```bash
./manta dev
```

The container:

- provides ROS 2 Jazzy;
- mounts the repository at `/odyssey`;
- uses the host UID and GID;
- uses `ROS_DOMAIN_ID=1`.

Host edits are immediately visible inside the container.

After changing native code, ROS code, CMake files, package metadata, generated Odyssey structures, or the Docker environment, rebuild with:

```bash
./manta build
```

Python configuration and mission-generator changes generally require regeneration but not a complete rebuild.

## Runtime output

Outputs are separated by scenario:

```text
output/
├── picket_line/
│   ├── Data/
│   │   ├── whales.csv
│   │   ├── whale_calls.csv
│   │   ├── whale_detections.csv
│   │   ├── <vehicle>.obd
│   │   └── <vehicle>_metrics.csv
│   └── Logs/
└── trailing/
    ├── Data/
    │   ├── whales.csv
    │   ├── whale_calls.csv
    │   ├── whale_detections.csv
    │   ├── <vehicle>.obd
    │   └── <vehicle>_metrics.csv
    └── Logs/
```

Aggregate vehicle metrics include:

- total fleet mass;
- energy consumption;
- mean vehicle speed.

Detection results count unique simulated whale IDs with at least one detected call. This represents simulation ground-truth coverage, not acoustic individual identification.

Runtime output, generated files, and build products normally should not be committed.

## Monte Carlo experiments

Run repeated realizations of a scenario:

```bash
./manta monte_carlo picket_line
./manta monte_carlo trailing
```

Each realization:

- receives a deterministic random seed;
- runs in an isolated Compose project and ROS domain;
- writes its own data and logs;
- retries failures as configured;
- records detection fraction, total fleet mass, energy consumption, and mean vehicle speed.

Results are stored under:

```text
output/<scenario>/monte_carlo/
```

For example:

```text
output/picket_line/monte_carlo/summary.csv
output/trailing/monte_carlo/summary.csv
```

Compatible completed results are loaded so interrupted experiments can resume. Convergence is evaluated at batch boundaries using the detection-fraction confidence-interval half-width and cumulative-mean stability.

## Fleet-size sweeps

Run a fleet-size sweep for either scenario:

```bash
./manta monte_carlo_sweep picket_line
./manta monte_carlo_sweep trailing
```

For every fleet size, MANTA:

1. clones the first configured vehicle as a template;
2. assigns IDs `auv_1` through `auv_N`;
3. generates scenario-appropriate missions;
4. runs the adaptive Monte Carlo experiment;
5. calculates fleet-level means and confidence intervals.

Results are separated by scenario:

```text
output/<scenario>/monte_carlo_sweep/
├── sweep_summary.csv
└── vehicles_XX/
    ├── summary.csv
    └── realization_XXXX/
```

Plot completed sweep results with:

```bash
./manta review picket_line sweep
./manta review trailing sweep
```

## Review scripts

Review scripts are organized by scenario:

```text
review/manta_review/
├── picket_line/
│   ├── parse_obd.py
│   ├── plot_tracks.py
│   └── plot_sweep_metrics.py
└── trailing/
    ├── parse_obd.py
    ├── plot_tracks.py
    └── plot_sweep_metrics.py
```

Each script reads from the corresponding scenario output directory.

## Repository structure

```text
MANTA/
├── config/
│   ├── scenarios/
│   └── vehicles/
├── docs/
├── generated/
├── missions/
│   ├── picket_line/
│   └── trailing/
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

Commit and push changes:

```bash
git add <files>
git commit -m "Describe the change"
git push -u origin feature/short-description
```

Open a pull request, merge it into `main`, and then update the local branch:

```bash
git switch main
git pull
```

## Troubleshooting

### `install/setup.bash` is missing or ROS packages are not found

Rebuild the development image and ROS workspace:

```bash
./manta build
```

Generated runtime services automatically source:

```text
/opt/ros/jazzy/setup.bash
/odyssey/ros_ws/install/setup.bash
```

### Runtime configuration is stale

Regenerate the selected scenario:

```bash
./manta generate picket_line
```

or:

```bash
./manta generate trailing
```

### Whale-call scheduling service is unavailable

Confirm the generated Compose file contains both whale services:

```bash
docker compose -f generated/compose.ros.yaml config --services
```

The list should include:

```text
whale_sim
whale_detection_sim
sim_orchestrator
```

Inspect a failed whale node with:

```bash
docker compose -f generated/compose.ros.yaml logs whale_sim
```

### Old containers remain

```bash
./manta down
```

### Review environment is missing

Create or restore:

```text
review/.venv/bin/python
```

before using `./manta review`.

### Review reports incomplete Odyssey files

An Odyssey `.obd` file without the `9999` mission-start marker normally indicates that the simulation shut down before vehicle initialization completed.

Inspect the runtime logs rather than treating this as a plotting error.

## Documentation

- `docs/manta_sim_sequence.puml` — ROS node interaction sequence.
- `docs/Whale_Model_Memo.pdf` — Whale-model equations, assumptions, parameter values, and evidence status.