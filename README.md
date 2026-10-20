# MANTA

MANTA is a ROS 2 and Docker simulation framework for evaluating autonomous underwater vehicle (AUV) missions on a shared world and synchronized simulation timeline.

The current missions use Odyssey AUVs, stochastic North Atlantic right whale movement and calling, and simulated acoustic detection:

- **Picket line:** vehicles patrol assigned line segments and record whale-call detections.
- **Trailing:** a vehicle follows detected whale-call locations, with configurable waypoint-update intervals.

Each mission owns its configuration, service definitions, mission generator, sweeps, and review scripts. Shared build, runtime, service construction, and Monte Carlo utilities live in `components/manta_core/scripts/`.

## Quick start

### Requirements

- Git
- Docker Engine or Docker Desktop with Docker Compose
- Python 3.10 or newer with virtual-environment support

The development image provides ROS 2 Jazzy and native build dependencies. Host Python dependencies are listed in `requirements.txt`.

```bash
git clone <MANTA_REPOSITORY_URL>
cd MANTA

python3 -m venv .venv
.venv/bin/python -m pip install -r requirements.txt

chmod +x manta picket_line trailing
```

Mission launchers use `.venv/bin/python`. All components currently reside in one repository.

### Build

```bash
./manta build
```

This builds the development image, generates Odyssey data structures, compiles native libraries, and builds the ROS packages.

Shared ROS build output is stored in `.build/ros/`. Odyssey native build products remain inside its component.

### Run

```bash
./picket_line run
./trailing run
```

Run one normal mission at a time. Both commands generate mission files, write root-level `manta.env` and `compose.ros.yaml`, and start the configured services.

Stop the normal-run services with:

```bash
./manta down
```

### Sweep

```bash
./picket_line sweep
./trailing sweep comms
```

Sweeps run repeated Monte Carlo realizations internally. There is no separate standalone Monte Carlo CLI command.

### Review

```bash
./picket_line review tracks
./picket_line review sweep

./trailing review tracks
./trailing review sweep comms
```

Review commands plot existing results without preparing a new simulation.

## Commands

| Command | Purpose |
| --- | --- |
| `./manta build` | Build the development image and native/ROS components. |
| `./manta down` | Stop and remove the generated normal-run services. |
| `./picket_line run` | Run the picket-line mission. |
| `./picket_line sweep` | Sweep AUV fleet size. |
| `./picket_line review tracks` | Plot picket-line trajectories and detections. |
| `./picket_line review sweep` | Plot fleet-size sweep results. |
| `./trailing run` | Run the trailing mission. |
| `./trailing sweep comms` | Sweep call waypoint-update intervals. |
| `./trailing review tracks` | Plot trailing trajectories and tracking metrics. |
| `./trailing review sweep comms` | Plot the update-interval sweep. |

```bash
./manta --help
./picket_line --help
./trailing --help
```

The `comms` experiment varies how often detected calls update the tracking waypoint. It does not currently simulate an acoustic communication channel.

## Repository organization

| Location | Contents |
| --- | --- |
| `components/manta_core/scripts/` | Shared command entry point, paths, loading, service construction, runtime preparation, and Monte Carlo utilities. |
| `components/manta_core/ros/manta_interfaces/` | Shared simulation messages. |
| `components/manta_core/ros/sim_orchestrator/` | Simulation clock, stepping, call synchronization, and shutdown. |
| `components/manta_odyssey/` | Odyssey native source, vehicle models, plugin manifest, and ROS adapter. |
| `components/manta_whale_sim/ros/` | Whale simulation and interfaces. |
| `components/manta_acoustics/ros/` | Acoustic detection simulation and interfaces. |
| `missions/picket_line/scripts/` | Entry point, service definitions, mission generation, and fleet-size sweep. |
| `missions/trailing/scripts/` | Entry point, service definitions, mission generation, and trailing sweeps. |
| `missions/<mission>/review/` | Plotting scripts and result parsers. |
| `missions/<mission>/config.yaml` | Mission configuration. |
| `output/` | Generated missions, simulation data, logs, and sweep results. |
| `.build/ros/` | Shared ROS build workspace. |
| `.venv/` | Host Python environment. |
| `compose.dev.yaml`, `Dockerfile.dev` | Development container definition. |
| `compose.ros.yaml`, `manta.env` | Generated normal-run configuration. |

Root launchers dispatch to:

```text
manta       → components/manta_core/scripts/manta_main.py
picket_line → missions/picket_line/scripts/picket_line_main.py
trailing    → missions/trailing/scripts/trailing_main.py
```

### Shared Python utilities

| Module | Responsibility |
| --- | --- |
| `manta_load.py` | Load YAML and plugin manifests, translate container paths, and load mission service builders. |
| `manta_services.py` | Construct reusable ROS and vehicle Compose services. |
| `manta_compose.py` | Write Compose documents from service dictionaries. |
| `manta_environment.py` | Prepare environment settings. |
| `manta_missions.py` | Invoke the configured mission generator. |
| `manta_runtime.py` | Coordinate normal-run preparation. |
| `manta_monte_carlo.py` | Execute repeated realizations and calculate statistics. |
| `manta_realization.py` | Prepare isolated realizations, run containers, and collect results. |
| `manta_dataclasses.py` | Configuration and result data structures. |
| `paths.py` | Repository paths. |

Mission service builders supply their settings to the shared utilities. Monte Carlo realizations use those same builders with realization-specific paths, seeds, and ROS domains.

## Configuration

Each mission owns its configuration:

```text
missions/picket_line/config.yaml
missions/trailing/config.yaml
```

Configured file paths are relative to the repository root.

### Mission generation and review

Picket line:

```yaml
mission_generation:
  generator: missions/picket_line/scripts/picket_line_mission.py
  output_directory: ./output/picket_line/missions

review:
  tracks: missions/picket_line/review/plot_tracks.py
  sweep: missions/picket_line/review/plot_sweep_fleet_size.py
```

Trailing:

```yaml
mission_generation:
  generator: missions/trailing/scripts/trailing_mission.py
  output_directory: ./output/trailing/missions

review:
  tracks: missions/trailing/review/plot_tracks.py
  sweep_comms: missions/trailing/review/plot_sweep_comms_freq.py
```

Mission entry points select sweep scripts directly. The top-level `sweep` section contains experiment parameters; `mission_generation.sweep` is no longer needed.

Generators accept:

```text
<config_path> <output_directory>
```

They create one Odyssey `.mi` file per configured vehicle.

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
```

For trailing, use paths under `output/trailing/`.

Vehicle IDs must be unique. The `type` matches a vehicle plugin manifest. An empty fault path disables fault injection.

Whale behavior and trajectory sampling are configured in
`components/manta_whale_sim/ros/whale_sim/config.yaml`. Acoustic detection settings
are configured in `components/manta_acoustics/ros/whale_detection_sim/config.yaml`.
Both missions load these component files. The shared whale settings use one whale,
active throughout the mission, initialized at the origin heading north, with the
trailing movement and grouping parameters.

Mission services supply runtime output directories, vehicle IDs, mission duration,
and Monte Carlo realization seeds. These runtime values override the corresponding
component defaults; whale behavior and detection settings come from the component files.

Sample periods determine trajectory logging frequency, not the physics timestep.

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
- `steps_per_message`: maximum cycles in a normal stepping batch.

The example permits up to three simulated days. Batches are shortened to reach whale-call events or the cycle limit. Vehicle mission completion can end a run earlier.

### Trailing assumptions

The shared whale component configuration specifies:

- One whale active throughout the mission.
- Initial whale position at the origin, heading north.
- Internal grouping parameters required by the shared whale movement model.

The waypoint-update interval remains configurable.

Calls are received only within the configured acoustic detection range.

Whale motion and acoustic range calculations use the horizontal north/east plane. Odyssey retains depth for vehicle dynamics and control.

The detector models calls as events rather than acoustic waveforms. It does not currently model propagation, multipath, background noise, or hydrophone-array processing. Tracking uses reported call positions; bearing-based cooperative estimation is future work.

## Simulation architecture

Both missions use the same orchestrator.

| Component | Responsibility |
| --- | --- |
| `SimOrchestratorNode` | Initialize the simulation, coordinate stepping and calls, and request shutdown. |
| `OdysseyNode` | Execute one vehicle’s mission, dynamics, and control. |
| `WhaleNode` | Manage all whale trajectories and call schedules. |
| `WhaleDetectionNode` | Evaluate calls against vehicle states and publish detections. |

There is one container per AUV and one whale-simulator container for the entire whale population.

### Synchronization

1. The orchestrator publishes startup information and waits for vehicle readiness.
2. It requests the next scheduled whale call.
3. Vehicle and whale nodes advance on a shared stepping timeline.
4. Vehicles publish states and batch completion results.
5. At the call cycle, the orchestrator publishes the call.
6. The detector evaluates matching vehicle states and acknowledges processing.
7. The orchestrator requests the next call and continues.

Picket-line vehicles retain their patrol behavior. Trailing vehicles use detections to update their target waypoints.

## Sweeps and results

| Experiment | Output directory |
| --- | --- |
| Picket-line fleet size | `output/picket_line/monte_carlo_sweep/` |
| Trailing update interval | `output/trailing/sweep_comms/` |

Each sweep writes `sweep_summary.csv` and per-setting realization results. Realizations use separate Compose projects, ROS domains, data directories, and logs.

Monte Carlo settings control seeds, worker counts, timeout/retry handling, and convergence. Resume behavior depends on saved results and the checks implemented by each sweep.

### Metrics

- **Detection fraction:** proportion of simulated whale IDs with at least one detected call.
- **Mean distance:** time-weighted horizontal AUV–whale separation, in kilometres.
- **Integrated tracking error:** horizontal separation integrated over time, in kilometre-hours.
- **Vehicle metrics:** mass, energy consumption, and speed.

Detection coverage uses simulator whale IDs. It does not represent acoustic identification of individual animals.

Mean distance equals integrated tracking error divided by the evaluated duration in hours.

## Adding a mission

1. Create `missions/<name>/config.yaml`.
2. Add a mission generator accepting configuration and output-directory arguments.
3. Add `<name>_services.py` with `create_services(config, ros_domain_id=1)`.
4. Add a mission entry point and root launcher.
5. Add any experiment sweeps and review scripts.
6. Test a short normal run before repeated experiments.

Core execution utilities are shared, but some result collection and convergence logic still assumes the current whale-monitoring missions. Extend that logic when adding different metrics or mission types.

## Troubleshooting

### Import errors after moving scripts

Mission entry points add the core scripts directory before importing shared modules:

```python
ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "components" / "manta_core" / "scripts"))
```

Core scripts use direct neighboring imports:

```python
from paths import ROOT
from manta_load import load_yaml
```

Remove imports targeting retired modules under `scripts/manta_core/`.

### A sweep exits without running

Confirm the script has a command-line entry point:

```python
if __name__ == "__main__":
    raise SystemExit(main())
```

### Missing generator or plotting script

Check the configured path against the actual file location. Paths are relative to the repository root.

### Configuration snapshot mismatch

Sweeps with snapshot validation reject changed configurations. Preserve the previous experiment by moving its output directory aside before starting a new one.

### AUV initialization failure

Inspect the realization’s console and vehicle logs. Verify that `MISSION_PATH` and `VEHICLE_PATH` point to existing files under the repository mounted at `/odyssey`.

The configured mission path must match the directory passed to the mission generator.

### Missing plotting dependencies

```bash
.venv/bin/python -m pip install -r requirements.txt
.venv/bin/python -m pip check
```

Mission launchers and plotting subprocesses should use the same virtual environment.

### Stale ROS build output

```bash
./manta build
```

Rebuild after native source, ROS interfaces, CMake, or Docker changes. Python and YAML changes generally require a new run rather than a ROS rebuild.

## Generated files and version control

Normal-run data includes whale trajectories, calls, detections, vehicle `.obd` files, and vehicle metrics under `output/<mission>/Data/`. Logs are stored under `output/<mission>/Logs/`.

Exclude virtual environments, build products, generated runtime files, and outputs from Git. Commit source, mission definitions, plugin manifests, and dependency files.