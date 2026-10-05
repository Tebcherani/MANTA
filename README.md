# MANTA

MANTA is a ROS 2 and Docker-based simulation framework for evaluating autonomous underwater vehicle (AUV) fleets on a shared world and synchronized simulation timeline.

The current missions use Odyssey AUVs, stochastic North Atlantic right whale movement and calling, and simulated acoustic detection:

- **Picket line:** vehicles patrol assigned line segments and record detections without leaving their patrols.
- **Trailing:** vehicles use detected whale-call locations to update their tracking waypoints, according to the configured update interval.

MANTA supports mission generation, synchronized simulation stepping, repeated Monte Carlo realizations, mission-specific sweeps, and result review. The acoustics baseline uses a fixed three-dimensional detection radius; it does not model propagation, noise, receiver performance, or detector uncertainty. The trailing configuration can bypass the range threshold with `receive_all_calls: true`.

## Quick start

### Requirements

- Git
- Docker Engine or Docker Desktop with Docker Compose
- Python 3.10 or newer and virtual-environment support

The development image provides ROS 2 Jazzy and native build dependencies. Host-side Python dependencies are recorded in the root `requirements.txt`.

```bash
git clone <MANTA_REPOSITORY_URL>
cd MANTA
python3 -m venv .venv
.venv/bin/python -m pip install -r requirements.txt
source .venv/bin/activate
```

MANTA currently keeps all components in one repository; cloning does not require submodules.

### Build

```bash
./manta build
```

This builds the development image, generates Odyssey data structures, compiles Odyssey's native libraries, and builds the component ROS packages in Release mode. Shared ROS build output is stored in `.build/ros/`, including its `build/`, `install/`, and `log/` directories. Odyssey also creates native build products inside its component.

The build command does not select a mission.

### Generate mission files

```bash
./manta generate picket_line
./manta generate trailing
```

The configured generator receives the mission config path and output directory. Generated Odyssey `.mi` files go to:

- `output/picket_line/missions/`
- `output/trailing/missions/`

There is one mission file per configured vehicle. `generate` creates mission files only; it does not prepare Docker runtime files or start a simulation.

### Run

```bash
./manta run picket_line
# Or:
./manta run trailing
```

`run` generates the selected mission files, prepares root-level `manta.env` and `compose.ros.yaml`, and starts the runtime services. These runtime files describe the most recently prepared normal run.

```bash
./manta down
```

### Review

```bash
./manta review picket_line tracks
./manta review trailing tracks
./manta review picket_line sweep
./manta review trailing sweep
```

Review scripts are selected from the mission config and run using `.venv/bin/python`. Track plots require simulation results; sweep plots require completed sweep summaries.

## Command reference

```text
./manta <command> [mission] [plot]
```

Mission names are discovered from `missions/*/config.yaml`. The Python CLI currently calls this argument `scenario` internally.

| Command | Purpose |
| --- | --- |
| `./manta build` | Build the Docker image and native/ROS components. |
| `./manta dev` | Open a development container. |
| `./manta down` | Stop and remove the generated runtime services and orphan containers. |
| `./manta generate <mission>` | Generate files using the mission's configured generator. |
| `./manta run <mission>` | Prepare and launch the selected simulation. |
| `./manta review <mission> tracks` | Run its configured track-review script. |
| `./manta review <mission> sweep` | Run its configured sweep-review script. |
| `./manta monte_carlo <mission>` | Run repeated realizations using the current shared experiment machinery. |
| `./manta monte_carlo_sweep <mission>` | Load and execute its configured sweep function. |

```bash
./manta --help
```

## Repository organization

**`components/` contains simulator capabilities. `missions/` contains experiment definitions and mission-specific tools.**

| Location | Contents |
| --- | --- |
| `components/manta_core/ros/manta_interfaces/` | Shared stepping, world-state, and vehicle-state messages. |
| `components/manta_core/ros/sim_orchestrator/` | Shared simulation clock, stepping, whale-call synchronization, and shutdown. |
| `components/manta_odyssey/` | Odyssey native source, models, plugin manifest, and ROS adapter under `ros_ws/src/odyssey_sim/`. |
| `components/manta_whale_sim/ros/` | `whale_interfaces` and `whale_sim`. |
| `components/manta_acoustics/ros/` | `acoustics_interfaces` and `whale_detection_sim`. |
| `missions/picket_line/` | Config, Odyssey generator, fleet-size sweep, and review scripts. |
| `missions/trailing/` | Config, Odyssey generator, call-update-interval sweep, and review scripts. |
| `scripts/manta_core/` | Current Python runner utilities, generation/review/sweep dispatch, and Monte Carlo machinery. |
| `scripts/manta_runner.py` | CLI entry point used by `./manta`. |
| `scripts/paths.py` | Main-project paths. |
| `output/` | Generated mission files, simulation data, logs, and experiment results. |
| `.build/ros/` | Generated shared ROS build workspace. |
| `.venv/` | Shared host Python environment. |
| `requirements.txt` | Python environment dependencies. |
| `compose.dev.yaml`, `Dockerfile.dev` | Development container definition. |
| `compose.ros.yaml`, `manta.env` | Generated normal-run configuration. |

The Python `scripts/manta_core/` directory and the component `components/manta_core/` are distinct locations. The Python-package relocation ZIP is not part of this layout.

## Mission configuration

Each mission owns its config:

```text
missions/picket_line/config.yaml
missions/trailing/config.yaml
```

Paths in these configs are relative to the MANTA repository root, rather than the config's own directory.

### Script selection

Picket line:

```yaml
mission_generation:
  generator: missions/picket_line/generate_odyssey_mission.py
  output_directory: ./output/picket_line/missions
  sweep: missions/picket_line/sweep.py

review:
  tracks: missions/picket_line/review/plot_tracks.py
  sweep: missions/picket_line/review/plot_sweep_metrics.py
```

Trailing:

```yaml
mission_generation:
  generator: missions/trailing/generate_odyssey_mission.py
  output_directory: ./output/trailing/missions
  sweep: missions/trailing/sweep.py

review:
  tracks: missions/trailing/review/plot_tracks.py
  sweep: missions/trailing/review/plot_sweep_metrics.py
```

The top-level `sweep:` section stores sweep parameters. The `mission_generation.sweep` entry selects the script. Both sweep scripts expose `run_monte_carlo_sweep(config)`; no function-name setting is needed.

### Vehicles and output

```yaml
vehicles:
  - id: auv_1
    type: odyssey
    mission: ./output/picket_line/missions/auv_1.mi
    model: ./components/manta_odyssey/config/Borealis.vd
    fault: ""

output:
  data_directory: ./output/picket_line/Data
  log_directory: ./output/picket_line/Logs
  auv_sample_period: 5.0
  whale_sample_period: 60.0
```

For trailing, use the corresponding paths under `output/trailing/`. Vehicle IDs must be unique. The runner creates one service per configured vehicle; `type: odyssey` matches the ID in `components/manta_odyssey/manta_plugin.yaml`. An empty fault path disables fault injection.

Sample periods control recorded trajectory sampling, not the internal physics timestep or event synchronization.

### Simulation timing

```yaml
simulation:
  timestep: 0.02
  control_period: 10
  max_steps: 12960000
  steps_per_message: 10000
```

- `timestep`: simulated seconds per physics step.
- `control_period`: physics steps between vehicle sensor/control updates.
- `max_steps`: maximum simulation cycles.
- `steps_per_message`: normal maximum cycles in one batch.

These example values allow up to three simulated days. The orchestrator shortens batches to reach a scheduled whale-call timestep or the remaining cycle limit. Simulations can end earlier when every vehicle mission completes.

### Mission and component settings

The existing `scenario:` section contains mission geometry, patrol parameters, whale settings, and detection settings. Its `type` still selects supported behavior in parts of the launch and evaluation code. Keep the supported `picket_line` or `trailing` type when using those existing behaviors.

Changing script selection does not by itself make all runtime and evaluation behavior mission-independent.

## Shared simulation architecture

Both current missions use the same orchestrator. It coordinates time and communication; each vehicle's generated `.mi` file and tracking parameters determine what the vehicle does.

| Runtime component | Responsibility |
| --- | --- |
| `SimOrchestratorNode` | Initialize time/world state, wait for vehicle readiness, select batches, coordinate whale calls, and request shutdown. |
| `OdysseyNode` | Execute one vehicle's mission and dynamics, publish state and batch results, and write final metrics. |
| `WhaleNode` | Generate movement/call schedules, update whale state, and answer `GetNextWhaleCall` requests. |
| `WhaleDetectionNode` | Evaluate call-to-vehicle distances, publish detections, log results, and acknowledge processed calls. |

The orchestrator does not switch between picket-line and trailing during a run. “All missions complete” means all vehicle mission files have finished executing.

### Synchronization

1. The orchestrator publishes startup information and waits for all vehicles to report ready.
2. It requests the next scheduled whale call.
3. It publishes a shared `SimStep` batch to vehicle and whale nodes. Those nodes process it asynchronously.
4. Vehicles publish their state and `SimStepResult`; the orchestrator waits for active vehicles to finish.
5. If the call has not been reached, another batch advances toward the same known event. The next call is not requested again after every batch.
6. At the call timestep, the orchestrator publishes `WhaleCall`. The detector waits for matching vehicle-state cycles, evaluates detections, and publishes `WhaleCallProcessed`.
7. After accepting that acknowledgement, the orchestrator requests the following call and continues.

Trailing vehicles also consume detections for tracking. Picket-line vehicles retain their patrol behavior.

The current orchestrator intentionally remains coupled to the vehicle–whale–acoustics timing protocol. No orchestrator rewrite is required to run the two supported missions.

## Monte Carlo and sweeps

```bash
./manta monte_carlo picket_line
./manta monte_carlo trailing
./manta monte_carlo_sweep picket_line
./manta monte_carlo_sweep trailing
```

Realizations use deterministic seeds, separate Compose projects and ROS domains, independent data/log directories, and configured timeout/retry handling. Compatible saved results can be loaded to resume experiments. Current evaluation includes detection coverage and vehicle metrics; trailing also includes tracking-distance/error metrics. Convergence uses mission-dependent metrics.

| Mission | Sweep |
| --- | --- |
| Picket line | Fleet size, using `sweep.min_vehicles` and `sweep.max_vehicles`. |
| Trailing | Call waypoint-update interval, using `sweep.call_update_intervals`. |

Normal Monte Carlo results are written under `output/<mission>/monte_carlo/`. Sweep results are written under `output/<mission>/monte_carlo_sweep/`, including `sweep_summary.csv` and per-setting realization results.

The shared `manta_sweep.py` loads the configured mission sweep function. Each mission owns the sweep implementation; the shared realization/evaluation code still contains mission-specific assumptions.

## Adding a mission

1. Create `missions/<name>/config.yaml`. The CLI discovers it automatically.
2. Add a generator accepting `<config_path> <output_directory>` and select it with `mission_generation.generator`.
3. Configure vehicle mission paths and output directories consistently.
4. Optionally provide a sweep exposing `run_monte_carlo_sweep(config)` and standalone review scripts; select them in the config.
5. Verify generation and review, then test a short run before starting a sweep.

A new folder enables discovery and configured script dispatch. It does **not** guarantee that arbitrary new simulation behavior works: launch, realization evaluation, and convergence still assume the currently supported whale-monitoring workflows. Extending those behaviors requires corresponding Python changes. The orchestrator can stay unchanged while the same synchronization protocol is used.

## Development and generated output

```bash
./manta dev
```

The development container provides ROS 2 Jazzy, mounts the repository at `/odyssey`, and uses the host UID/GID. Host edits are visible inside the container.

Rebuild after native/ROS source, CMake, package metadata, or Docker changes. Python tools and YAML edits generally require regeneration or a new run, without rebuilding ROS.

Runtime services source:

```bash
source /opt/ros/jazzy/setup.bash
source /odyssey/.build/ros/install/setup.bash
```

Typical normal-run data includes `whales.csv`, `whale_calls.csv`, `whale_detections.csv`, vehicle `.obd` files, and vehicle metrics CSVs under `output/<mission>/Data/`, with logs under `Logs/`.

Detection coverage counts unique simulated whale IDs with at least one detected call. It is ground-truth coverage, not acoustic identification of individual whales.

Ignore `.venv/`, `.build/`, component build products, generated runtime files, and generated outputs in Git. Commit mission definitions, source, build manifests, and dependency files. Avoid committing code ZIP archives.

## Troubleshooting

### Missing or stale ROS build output

```bash
./manta build
```

After moving a ROS package, remove its old CMake cache from the active workspace before rebuilding:

```bash
rm -rf .build/ros/build/<package-name>
./manta build
```

For duplicate package names, keep only one source copy in the searched component directories.

### Generated mission files appear in the wrong directory

Check `mission_generation.output_directory` and confirm that the generation wrapper passes it as the second argument to the generator. The generators require explicit config and output paths.

### Runtime configuration is stale

```bash
./manta run trailing
```

`run` regenerates the selected runtime configuration. `generate` only writes mission files.

### Whale-call scheduling service is unavailable

```bash
docker compose -f compose.ros.yaml config --services
docker compose -f compose.ros.yaml logs whale_sim
```

The current runtime needs `whale_sim`, `whale_detection_sim`, `sim_orchestrator`, and the configured vehicle services.

### Review environment or dependencies are missing

```bash
python3 -m venv .venv
.venv/bin/python -m pip install -r requirements.txt
.venv/bin/python -m pip check
```

Both `scripts/paths.py` and the realization metric launcher must use `.venv/bin/python`. Review scripts import their adjacent parser with `from parse_obd import read_obd` when executed directly.

### Incomplete Odyssey output

An `.obd` file without the expected mission-start marker may indicate failed initialization or an interrupted run. Inspect simulation logs before diagnosing the plotting code.

## Documentation

- `docs/manta_sim_sequence.puml`: ROS node interaction sequence. Show repeated stepping batches between call requests and conditional event processing.
- `docs/Whale_Model_Memo.pdf`: whale-model assumptions, equations, parameters, and evidence.
