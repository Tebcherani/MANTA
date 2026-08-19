# MANTA

**MANTA** is a ROS 2-based multi-vehicle simulation framework designed to coordinate multiple simulated vehicles within a shared world and simulation timeline.

MANTA provides the simulation orchestration and ROS communication layer while vehicle-specific simulator implementations can live in separate Git repositories.

The Odyssey AUV simulator is currently integrated as a Git submodule under:

```text
vehicles/odyssey/
```

This allows the Odyssey simulator to remain independently maintained while MANTA references a specific known Odyssey revision.

---

# Repository Structure

A typical MANTA checkout looks like:

```text
MANTA/
├── .gitignore
├── .gitattributes
├── .gitmodules
│
├── Dockerfile.dev
├── compose.dev.yaml
├── compose.ros.yaml
│
├── output/
│   ├── Data/
│   └── Logs/
│
├── vehicles/
│   └── odyssey/                  # Git submodule
│       ├── assets/
│       ├── include/
│       ├── src/
│       ├── Makefile
│       └── lib/
│
└── ros_ws/
    └── src/
        ├── odyssey_sim/
        │   ├── CMakeLists.txt
        │   ├── package.xml
        │   └── src/
        │       └── odyssey_node.cpp
        │
        └── sim_orchestrator/
            ├── CMakeLists.txt
            ├── package.xml
            └── src/
                └── sim_orchestrator_node.cpp
```

The major responsibilities are:

```text
MANTA
├── Simulation Orchestrator
├── ROS 2 communication
├── multi-vehicle synchronization
├── shared simulation execution
└── vehicle adapters

vehicles/odyssey
└── Odyssey-specific simulator implementation
```

---

# Prerequisites

Docker must be installed.

Verify Docker with:

```bash
docker --version
```

Git is also required:

```bash
git --version
```

No local ROS 2 installation is required because ROS 2 and the native build dependencies are provided by the MANTA development container.

---

# Cloning MANTA

Because MANTA uses Git submodules, the preferred way to clone it is:

```bash
git clone --recurse-submodules <MANTA_REPOSITORY_URL>
```

This clones both MANTA and the Odyssey dependency.

If MANTA was already cloned without its submodules, initialize them with:

```bash
git submodule update --init --recursive
```

---

# Git Submodules

The Odyssey simulator under:

```text
vehicles/odyssey/
```

is a separate Git repository.

This means there are two independent Git histories:

```text
MANTA/
    .git/

MANTA/vehicles/odyssey/
    separate Git repository
```

MANTA does not track individual Odyssey source files directly.

Instead, MANTA tracks the exact Odyssey commit it expects.

Conceptually:

```text
MANTA commit
    |
    └── Odyssey commit abc123
```

This makes MANTA builds reproducible because a specific MANTA revision always points to a specific Odyssey revision.

---

# Development Environment

MANTA provides a development container containing:

- Ubuntu/Linux
- ROS 2 Jazzy
- GCC/G++
- Make
- colcon
- LAPACK
- LAPACKE
- BLAS
- ROS development tools

The development environment is defined by:

```text
Dockerfile.dev
compose.dev.yaml
```

Build the development image with:

```bash
docker compose -f compose.dev.yaml build
```

This normally only needs to be repeated when `Dockerfile.dev` or the packages installed by it change.

Enter the development environment with:

```bash
docker compose -f compose.dev.yaml run --rm dev
```

Inside the container, the MANTA repository is mounted at:

```text
/odyssey
```

Therefore:

```text
Host MANTA repository
        ↕
Container /odyssey
```

Changes made on the host appear immediately inside the container, and build products generated inside the container appear on the host.

---

# Building MANTA

There are two build stages:

```text
1. Build vehicle-native libraries
2. Build the ROS workspace
```

For the current Odyssey integration:

```text
Odyssey C source
      ↓
make libraries
      ↓
static libraries
      ↓
MANTA ROS nodes
      ↓
colcon build
```

---

# Building the Odyssey Libraries

Enter the MANTA development container:

```bash
docker compose -f compose.dev.yaml run --rm dev
```

Then:

```bash
cd /odyssey/vehicles/odyssey
make libraries
```

This builds the Odyssey simulator libraries under:

```text
vehicles/odyssey/lib/
```

The expected libraries are:

```text
lib/
├── libodyssey.a
├── libsim_common.a
├── libsim_sensors.a
├── libsimulation.a
└── libworld_sim.a
```

For a clean rebuild:

```bash
make clean
make libraries
```

A normal incremental rebuild usually only requires:

```bash
make libraries
```

Make will recompile files whose source has changed.

---

# Building the ROS Workspace

After building the Odyssey libraries:

```bash
cd /odyssey/ros_ws
colcon build
```

For a completely clean ROS rebuild:

```bash
rm -rf build install log
colcon build
```

After a successful build, source the workspace:

```bash
source install/setup.bash
```

You can verify the MANTA executables with:

```bash
ros2 pkg executables odyssey_sim
```

and:

```bash
ros2 pkg executables sim_orchestrator
```

Expected executables include:

```text
odyssey_sim odyssey_node
sim_orchestrator sim_orchestrator_node
```

---

# Typical Full Build

From the MANTA root:

```bash
docker compose -f compose.dev.yaml run --rm dev
```

Then inside the container:

```bash
cd /odyssey/vehicles/odyssey
make libraries

cd /odyssey/ros_ws
colcon build
```

Then exit the development container:

```bash
exit
```

The system is now ready to run.

---

# Running MANTA

The complete ROS simulation is defined by:

```text
compose.ros.yaml
```

It currently launches:

```text
sim_orchestrator
odyssey_1
odyssey_2
```

Start the complete system with:

```bash
docker compose -f compose.ros.yaml up
```

Stop it with:

```text
Ctrl+C
```

and remove the containers/network with:

```bash
docker compose -f compose.ros.yaml down
```

To also remove containers left over from renamed services:

```bash
docker compose -f compose.ros.yaml down --remove-orphans
```

To explicitly recreate the containers:

```bash
docker compose -f compose.ros.yaml up --force-recreate
```

---

# Runtime Startup Sequence

The current startup sequence is:

```text
Docker Compose
      ↓
Simulation Orchestrator starts
      ↓
simulation log initialized
      ↓
simulation configuration initialized
      ↓
world simulator initialized
      ↓
/sim/startup_ready = true
      ↓
┌────────────────────┬────────────────────┐
│                    │                    │
▼                    ▼                    │
Odyssey 1         Odyssey 2               │
initializes       initializes             │
│                    │                    │
▼                    ▼                    │
/odyssey_1/ready  /odyssey_2/ready        │
│                    │                    │
└──────────────┬─────┘                    │
               ▼                          │
      Simulation Orchestrator             │
      confirms all vehicles ready         │
```

The readiness topics use ROS transient-local QoS so nodes that subscribe after readiness was published can still receive the latest state.

---

# Runtime Configuration

Vehicle-specific environment variables are configured in:

```text
compose.ros.yaml
```

For example:

```yaml
environment:
  ROS_DOMAIN_ID: 1

  ODYSSEY_VEHICLE_ID: odyssey_1

  ODYSSEY_DATA_DIR: /odyssey/output/Data

  MISSION_PATH: /odyssey/vehicles/odyssey/assets/Missions/dive240.mi
  FAULT_PATH: ""

  VEHICLE_PATH: /odyssey/vehicles/odyssey/assets/Vehicles/Borealis.vd

  SIM_LOG_PATH: /odyssey/output/Logs/odyssey_1.log
```

A second vehicle can use the same simulator implementation with different configuration:

```yaml
ODYSSEY_VEHICLE_ID: odyssey_2
MISSION_PATH: /odyssey/vehicles/odyssey/assets/Missions/dive240_diff.mi
```

This means:

```text
same Odyssey executable
        ↓
different environment/configuration
        ↓
multiple independent simulated vehicles
```

---

# No Fault File

If an Odyssey vehicle should run without an external fault file:

```yaml
FAULT_PATH: ""
```

The Odyssey simulator treats an empty fault path as no supplied fault configuration.

---

# Output Directories

MANTA runtime output is stored under:

```text
output/
├── Data/
└── Logs/
```

Create them if they do not exist:

```bash
mkdir -p output/Data output/Logs
```

Typical output may include:

```text
output/
├── Data/
│   ├── odyssey_1_....obd
│   └── odyssey_2_....obd
│
└── Logs/
    ├── sim_orchestrator.log
    ├── odyssey_1.log
    └── odyssey_2.log
```

These files are runtime products and normally should not be committed.

---

# What Needs to Be Rebuilt?

Different changes require different rebuilds.

## Odyssey C source changed

For changes under:

```text
vehicles/odyssey/src/
vehicles/odyssey/include/
```

rebuild the Odyssey libraries:

```bash
cd /odyssey/vehicles/odyssey
make libraries
```

Because the ROS executables statically link those libraries, rebuild ROS afterward:

```bash
cd /odyssey/ros_ws
colcon build
```

Then restart the simulation:

```bash
docker compose -f compose.ros.yaml down
docker compose -f compose.ros.yaml up
```

---

## ROS C++ source changed

For changes such as:

```text
ros_ws/src/odyssey_sim/src/odyssey_node.cpp
ros_ws/src/sim_orchestrator/src/sim_orchestrator_node.cpp
```

only the ROS workspace needs rebuilding:

```bash
cd /odyssey/ros_ws
colcon build
```

Then restart the Compose stack.

---

## CMake or package.xml changed

Rebuild the affected ROS package:

```bash
cd /odyssey/ros_ws
colcon build
```

A clean rebuild can be useful after significant build-system changes:

```bash
rm -rf build install log
colcon build
```

---

## Odyssey Makefile changed

Rebuild the native libraries:

```bash
cd /odyssey/vehicles/odyssey
make clean
make libraries
```

Then rebuild ROS:

```bash
cd /odyssey/ros_ws
colcon build
```

---

## Dockerfile.dev changed

Rebuild the development Docker image:

```bash
docker compose -f compose.dev.yaml build
```

Use `--no-cache` when necessary:

```bash
docker compose -f compose.dev.yaml build --no-cache
```

Ordinary C or C++ source changes do **not** require rebuilding the Docker image.

---

## compose.ros.yaml changed

Usually no source rebuild is required.

Recreate the runtime containers:

```bash
docker compose -f compose.ros.yaml down
docker compose -f compose.ros.yaml up --force-recreate
```

---

# Recommended Development Cycle

For a normal Odyssey source change:

```text
edit code
   ↓
make libraries
   ↓
colcon build
   ↓
docker compose down/up
```

Commands:

```bash
docker compose -f compose.dev.yaml run --rm dev
```

Inside:

```bash
cd /odyssey/vehicles/odyssey
make libraries

cd /odyssey/ros_ws
colcon build
```

Exit:

```bash
exit
```

Then:

```bash
docker compose -f compose.ros.yaml down
docker compose -f compose.ros.yaml up
```

---

# Committing MANTA Changes

Changes to the ROS framework should be committed from the MANTA repository root.

Examples include:

```text
ros_ws/
compose.ros.yaml
compose.dev.yaml
Dockerfile.dev
README.md
```

Check status:

```bash
git status
```

Stage the changes:

```bash
git add .
```

Commit:

```bash
git commit -m "Describe the MANTA change"
```

Push:

```bash
git push origin main
```

---

# Committing Odyssey Changes

Odyssey is a separate repository.

If Odyssey code is edited from inside MANTA:

```bash
cd vehicles/odyssey
```

Check the Odyssey repository:

```bash
git status
```

Commit the Odyssey source change:

```bash
git add <changed-files>
git commit -m "Describe the Odyssey change"
git push origin main
```

This updates the actual Odyssey repository.

Then return to MANTA:

```bash
cd ../..
```

MANTA will now report:

```text
modified: vehicles/odyssey (new commits)
```

MANTA must record that it now depends on the new Odyssey revision:

```bash
git add vehicles/odyssey
git commit -m "Update Odyssey dependency"
git push origin main
```

Therefore an Odyssey change normally creates **two commits**:

```text
Odyssey repository
    ↓
commit actual Odyssey source change

MANTA repository
    ↓
commit new Odyssey submodule revision
```

---

# Updating Odyssey Without Editing It

If another developer pushed Odyssey changes and MANTA should use them:

```bash
cd vehicles/odyssey
git pull origin main
```

Then return to MANTA:

```bash
cd ../..
```

Record the updated dependency:

```bash
git add vehicles/odyssey
git commit -m "Update Odyssey dependency"
git push
```

---

# Checking Which Odyssey Version MANTA Uses

From the MANTA root:

```bash
git submodule status
```

Or:

```bash
cd vehicles/odyssey
git log -1 --oneline
```

MANTA records this exact Odyssey revision.

---

# Restoring the Recorded Odyssey Revision

If the Odyssey submodule was changed locally and you want to restore exactly the revision recorded by MANTA:

```bash
git submodule update --init --recursive
```

This checks out the Odyssey commit referenced by the current MANTA commit.

---

# Pulling MANTA Changes

Update MANTA itself with:

```bash
git pull
```

Then synchronize its submodules:

```bash
git submodule update --init --recursive
```

A useful combined command is:

```bash
git pull --recurse-submodules
```

followed by:

```bash
git submodule update --init --recursive
```

---

# Rebase Workflow

When the remote contains new commits and the local repository also contains unpublished commits:

```bash
git pull --rebase origin main
```

If a conflict occurs:

```bash
git status
```

Resolve the affected files, then:

```bash
git add <resolved-files>
git rebase --continue
```

To cancel the rebase:

```bash
git rebase --abort
```

After the rebase completes:

```bash
git push origin main
```

---

# Generated Files

The following are build/runtime products and should normally be ignored by Git:

```text
ros_ws/build/
ros_ws/install/
ros_ws/log/

vehicles/odyssey/build/
vehicles/odyssey/lib/

output/

*.o
*.a
*.obd
*.log
```

Do not commit generated static libraries or colcon build products.

Instead, build them from source.

---

# Clean Rebuild

When build state becomes questionable, use a complete clean rebuild.

Enter the dev environment:

```bash
docker compose -f compose.dev.yaml run --rm dev
```

Clean and rebuild Odyssey:

```bash
cd /odyssey/vehicles/odyssey
make clean
make libraries
```

Clean and rebuild ROS:

```bash
cd /odyssey/ros_ws
rm -rf build install log
colcon build
```

Exit:

```bash
exit
```

Then recreate the runtime stack:

```bash
docker compose -f compose.ros.yaml down --remove-orphans
docker compose -f compose.ros.yaml up --force-recreate
```

---

# Quick Reference

## First checkout

```bash
git clone --recurse-submodules <MANTA_REPOSITORY_URL>
cd MANTA

docker compose -f compose.dev.yaml build
mkdir -p output/Data output/Logs
```

## Build everything

```bash
docker compose -f compose.dev.yaml run --rm dev
```

Inside:

```bash
cd /odyssey/vehicles/odyssey
make libraries

cd /odyssey/ros_ws
colcon build
```

## Run everything

```bash
docker compose -f compose.ros.yaml up
```

## Stop everything

```bash
docker compose -f compose.ros.yaml down
```

## Odyssey source changed

```bash
cd /odyssey/vehicles/odyssey
make libraries

cd /odyssey/ros_ws
colcon build
```

## ROS source changed

```bash
cd /odyssey/ros_ws
colcon build
```

## Commit an Odyssey change

```bash
cd vehicles/odyssey

git add <files>
git commit -m "Odyssey change"
git push origin main

cd ../..

git add vehicles/odyssey
git commit -m "Update Odyssey dependency"
git push origin main
```

## Commit a MANTA-only change

```bash
git add <files>
git commit -m "MANTA change"
git push origin main
```

---

# Architecture Direction

MANTA is intended to support multiple vehicle implementations rather than being tied specifically to Odyssey.

The intended architecture is:

```text
                   MANTA
                     |
        +------------+------------+
        |                         |
 Simulation Orchestrator      World Model
        |
        | ROS interfaces
        |
   +----+---------------------+
   |                          |
Odyssey                    Vehicle B
adapter                    adapter
   |                          |
Odyssey repo              Vehicle B repo
```

Each vehicle implementation can evolve independently while communicating with MANTA through a common ROS-facing interface.

Odyssey is the first integrated vehicle implementation, not a hard-coded requirement of the framework.