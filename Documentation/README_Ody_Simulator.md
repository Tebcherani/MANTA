# Odyssey AUV Simulator — Developer README

MIT Sea Grant Autonomous Underwater Vehicles Laboratory  
Original codebase: RCS revision 2.22–2.24, 1998  
This version: simulator-only extraction, targeting Linux / GCC

---

## Overview

This directory contains the standalone Odyssey AUV vehicle simulator, extracted
from the full vehicle codebase. It runs the complete vehicle control software —
mission loading, layered control, PID dynamic control, navigation filtering, and
data logging — against a physics-based 6-DOF hydrodynamic model in place of real
hardware.

The simulator is the correct starting point for:
- Developing and testing new behaviors before deploying on a vehicle
- Mission planning and rehearsal
- Algorithm development (navigation, control, autonomy)
- Fault injection studies and robustness testing
- Any future MEX / MATLAB integration

The architecture is described in:

> Bellingham, J. G., and J. Leonard. "Task Configuration with Layered Control."
> IARP Workshop on Mobile Robots for Subsea Environments, 1994.

---

## Quick Start

```bash
# First-time Ubuntu setup
./ody setup

# Build both simulator executables
./ody build

# Run the full simulator with a mission from SMission/
./ody sim dive240.mi

# Run the kinematic simulator parser check
./ody ksim

# Run with fault injection
./ody sim dive240.mi cg_shift_demo.fd
```

Output files are written to `./SData/` by default. The system log goes to
`./system.log`. See `RUN_UBUNTU.md` for the concise setup and run guide.

---

## File Inventory

### Entry point

| File | Purpose |
|---|---|
| `sim_main.c` | Main function, outer simulation loop, `exit_gracefully()` |

### Physics model

| File | Purpose |
|---|---|
| `sim.c` | 6-DOF equations of motion: `motion()`, `init_simulation()` |
| `sim_bottom.c` | Simulated bathymetry and altimeter returns |
| `sim_current.c` | Simulated ocean currents |
| `sim_conduct.c` | Simulated water conductivity |
| `sim_usbl.c` | Simulated USBL acoustic navigation returns |
| `sensor_load.c` | Converts simulator true state into sensor readings: `simSensors()`, `sensor_processing()` |
| `get_am.tex` | Hydrodynamic added-mass coefficients — **must be present at runtime** |
| `get_drag.tex` | Drag coefficients — **must be present at runtime** |
| `get_shape.tex` | Vehicle shape parameters — **must be present at runtime** |

`sim.c` reads `get_am.tex` and `get_drag.tex` from the working directory at
startup via `get_hydro()`. If they are missing, the simulator falls back to
built-in Draper-derived coefficients (less accurate). Keep all three `.tex`
files in the same directory as the executable, or in the working directory
from which the simulator is launched.

### Control loop

| File | Purpose |
|---|---|
| `layered_control.c` | Behavior execution loop and command arbitration (`resolve()`) |
| `dynamic_control.c` | PID heading / depth / speed controllers |
| `whoi_traj.c` | Trajectory smoother (first-order filter) used by dynamic controller |
| `whoi_traj.h` | Trajectory smoother types and prototypes |

### Mission loading and data structure

| File | Purpose |
|---|---|
| `load_mission.c` | Parses mission text files into the data structure |
| `vehicle.c` | **Generated** — the data structure instance and template; do not edit by hand |
| `vehicle.h` | **Generated** — the data structure definition and index constants; do not edit by hand |
| `update.c` | `update()`, `update_add()`, `init_data_structure()` |
| `edit_struct.c` | Tool to regenerate `vehicle.c` and `vehicle.h` (see below) |

### Fault injection

| File | Purpose |
|---|---|
| `load_fault.c` | Fault injection engine: `load_faults()`, `apply_faults()` |
| `SFault/` | Directory containing fault definition files (`*.fd`) |
| `SFault/cg_shift_demo.fd` | Example: forward CG shift triggered at depth > 5 m |

### Data logging

| File | Purpose |
|---|---|
| `log_data.c` | Binary data file writer (`log_data()`, `init_data_logging()`) |
| `fileman.h` | `fileman.c` prototypes |
| `parse.c` | **Separate post-processing utility** — reads and converts data files for analysis |

### Navigation

| File | Purpose |
|---|---|
| `lbl.c` | Long-baseline acoustic navigation processing |
| `lbl.h` | LBL types and prototypes |
| `lbl_nav.c` | LBL position solution |
| `ekf_work.c` | Extended Kalman Filter for navigation |
| `ekf.h` | EKF types |
| `nav_math.h` | Navigation math macros (`D2R`, `R2D`, etc.) |

### Behaviors

| File | Behaviors implemented |
|---|---|
| `layered_control.c` | `setpoint`, `setvector`, `acquire_heading`, `depth_envelope`, `mission_timer`, `detect_collision`, and others |
| `waypoint.c` | `waypoint_2d` — navigate to a geographic waypoint |
| `wiggle.c` | `wiggle` — actuator test behavior |
| `yo_yo_depth.c` | Yo-yo depth profiling (depth-triggered) |
| `yo_yo_temperature.c` | Yo-yo profiling (temperature-triggered) |
| `yoyo_new.c` | Updated yo-yo behavior |
| `whoi_homing.c` | USBL acoustic homing behaviors |
| `docking.c` | Docking behavior |

### Utility and support

| File | Purpose |
|---|---|
| `diagnostic.c` | Message routing to terminal and syslog (`write_message()`, `open_syslog()`) |
| `diagnostic.h` | Status codes, `TERM`/`SLOG` flags, prototypes |
| `nrutil.c` | Numerical Recipes support routines (used by `sim.c`) |
| `nrutil.h` | Numerical Recipes prototypes |
| `getopt.c` | POSIX `getopt` implementation |
| `getopt.h` | `getopt` prototypes |

### Headers — core

| File | Contents |
|---|---|
| `vehicle.h` | Generated: `#include`s all headers below; single include for most `.c` files |
| `ody_defs.h` | Constants (`MAX_SENSOR`, `MAX_BEHAVIOR`, etc.), `sensor()` / `sensor_time()` macros, vehicle configuration `#define`s |
| `ody_types.h` | Core type definitions: `struct sensor`, `struct behavior`, `struct command`, `struct vehicle` |
| `ody_var.h` | Global variable declarations (`vvp`, `UseSim`, subsystem flags, etc.) |
| `ody_func.h` | Function prototypes for all major modules |
| `ody_name.h` | Vehicle identity — sets `#define _SIMULATOR` for this build |
| `paths.h` | Runtime path declarations for `SData/`, `SMission/`, `SFault/`, `Vehicle/`, and `system.log` |

### Configuration

| File | Purpose |
|---|---|
| `Sim.cfg` | Simulator vehicle configuration for `edit_struct` |

---

## Build

### Compiler and flags

```bash
make both
```

The current Ubuntu build uses the repository `Makefile`, GNU89 C semantics, and
the simulator identity selected in `ody_name.h`:

```c
#define _SIMULATOR
```

Do not remove or override that vehicle identity. It activates the correct block
in `ody_defs.h`, setting `VEHICLE_NAME "Simulator"` and enabling the right
optional subsystems.

### Recommended Makefile

Use the checked-in `Makefile`. It excludes hardware-only and currently broken
legacy source files, builds `sim`, and builds the Ubuntu kinematic simulator as
`ksim_bin` because `ksim/` is already a directory.

Common targets:

```bash
make both          # build sim and ksim_bin
make run-sim       # run ./sim with MISSION=dive240.mi by default
make run-ksim      # run ./ksim_bin and write a behavior log
make clean         # remove C build products
```

### Known issues for the Linux port

The codebase was written for the OS-9 real-time operating system. Hardware and
OS-9 code is isolated behind `#ifdef _OSK` guards throughout, so it is compiled
out automatically on Linux. However, there are a few issues to be aware of:

**1. `ody_func.h` contains prototypes for removed hardware files.**
Prototypes for `init_thruster()`, `control_fin()`, `read_parosci()`, and others
are declared in `ody_func.h` but their implementations have been deleted. This
will not cause linker errors as long as nothing calls them — and in simulator
mode nothing does — but it will produce warnings. The clean fix is to remove
those prototype blocks from `ody_func.h` or wrap them in
`#ifndef _SIMULATOR ... #endif`.

**2. `ody_var.h` declares globals for removed hardware subsystems.**
Variables like `ErrorKVH_DGC`, `UseThruster`, etc. are declared as `extern` here
and defined in the deleted hardware `.c` files. Under GCC these will cause linker
errors ("undefined reference") if any code in the simulator path actually
references them. In practice they are guarded by `#ifdef` blocks and are
unreachable in simulator mode, but verify by checking linker output carefully
on first build.

**4. `hirestime()` in `sim_main.c`.**
This function is called once to record mission start time. Check whether it is
defined in the remaining source files or needs a simple POSIX replacement using
`clock_gettime()`.

**5. `drand48()` is available on Linux** and does not need the stub declared
in `ody_func.h` under `#ifdef _OSK`. No action needed.

---

## The Data Structure

All software components communicate through a single global data structure
(`struct vehicle`, pointed to by `vvp`). Understanding it is essential before
modifying the code.

### Reading a variable

```c
double depth   = sensor( M_DEPTH );
double heading = sensor( M_YAW );
double kph     = sensor( UO_CONTROL_KPH );
```

The all-caps constants (e.g. `M_DEPTH`, `UO_CONTROL_KPH`) are `#define`d in
`vehicle.h` as integer array indices. They are generated automatically from the
variable names in `edit_struct.c`.

### Writing a variable

Always use `update()` — never write directly to `vvp->s[i].x`:

```c
update( M_DEPTH, new_depth );
update( UO_CONTROL_KPH, 0.75 );
```

`update()` writes the value, sets the timestamp, and updates running statistics
(mean, variance, min, max). The data logger uses the timestamp to decide whether
to record a variable this cycle. Writing directly bypasses the timestamp and the
variable will not be logged.

### Variable naming conventions

| Prefix | Meaning |
|---|---|
| `m_` | Measured or computed by the vehicle |
| `u_` | User-settable parameter (set from mission scripts) |
| `uo_` | User configuration — control gains and limits |
| `c_` | Command variable written by the control system |
| `cc_` | Logged copy of a command variable |
| `e_` | Environmental constant (e.g. latitude) |
| `real_` | Simulator ground-truth state (not accessible to vehicle logic) |
| `wp_` | Waypoint navigation variable |

### Units

All internal values are **MKS and radians**. The one exception is mission script
files, where angles may be specified in degrees — `load_mission.c` converts them
to radians automatically. Always specify units in the variable name string when
adding new variables (e.g. `m_depth(m)`, `m_roll(rad)`).

---

## Modifying the Data Structure

`vehicle.c` and `vehicle.h` are **generated files**. The canonical source of the
data structure definition is the first ~200 lines of `edit_struct.c`, formatted
as specially structured comment lines.

### Workflow

1. Edit `edit_struct.c` — add `sensor:`, `behavior:`, or `b_arg:` lines.
2. Compile and run `edit_struct`:
   ```bash
   gcc -o edit_struct edit_struct.c
   ./edit_struct Sim.cfg
   ```
   This reads its own source file to extract the structure definition and
   generates new `vehicle.h` and `vehicle.c`.
3. Recompile the simulator.

### Syntax

```
sensor: variable_name(unit) default_value

behavior: behavior_name
  b_arg:  argument_name(unit) default_value
  b_arg:  another_arg(bool)   0.0
```

Optional subsystem variables are wrapped in config blocks:
```
config: sbe_25
  sensor: sbe_temp_freq(Hz)
  sensor: sbe_cond_freq(Hz)
endconfig
```
The name in `config:` must match a line in the `.cfg` file for those variables
to be compiled in.

---

## Adding a New Behavior

1. Write the behavior function:
   ```c
   void my_behavior( struct behavior *b, int priority, int level )
   {
       struct command *cmd = &vvp->c[priority];
       /* read b_args, update cmd->heading, cmd->depth, cmd->speed */
   }
   ```

2. Add the behavior definition to `edit_struct.c`:
   ```
   behavior: my_behavior
     b_arg: target_depth(m)  10.0
     b_arg: speed(m/s)        1.0
     b_arg: behavior_active(bool) 0.0
   ```

3. Run `edit_struct` and recompile.

4. Register the function pointer in `layered_control.c` so the mission loader
   can associate the behavior name with the function.

5. Use the behavior in mission scripts without any further code changes:
   ```
   behavior: my_behavior 3
     b_arg: target_depth(m) 25.0
     b_arg: speed(m/s)       1.2
   ```

---

## Mission Scripts

Missions are plain text files passed on the command line:
```bash
./ody sim dive240.mi
```

The wrapper passes `dive240.mi` to `sim`; `load_mission.c` resolves bare mission
names against `SMission/`. You can also pass an explicit relative or absolute
path.

A mission file has three types of entries:

**Sensor initialization** — set starting values for any variable:
```
sensor: uo_control_kph        0.75
sensor: uo_control_pitch_lim(deg)  30
sensor: m_pos_n(m)            0.0
sensor: m_pos_e(m)            0.0
```

**Behavior stack** — list active behaviors with priorities (1 = highest):
```
behavior: mission_timer 1
  b_arg: time(s) 600

behavior: depth_envelope 2
  b_arg: max_depth(m) 100
  b_arg: min_depth(m)   2

behavior: waypoint_2d 3
  b_arg: target_x(m)  500.0
  b_arg: target_y(m)  200.0
  b_arg: speed(m/s)     1.35
```

**Flags** (before the first `behavior:` line):
```
UseSim 1        # always 1 for the simulator
```

---

## The Simulation Loop

`sim_main.c` runs the following sequence every 20 ms:

```
motion( e, p, ... )       — integrate equations of motion one 20ms step
```

Every 10th iteration (i.e., every 200 ms = 5 Hz):
```
simSensors()              — load sim true state into sensor variables
sensor_processing()       — attitude, dead reckoning, nav filtering
apply_faults()            — evaluate and fire fault rules (if any loaded)
layered_control()         — execute behavior stack, resolve commands
dynamic_control()         — PID controllers, compute fin/thruster commands
log_data()                — write changed variables to data file
```

The outer 20 ms loop steps the physics; the inner 200 ms loop steps the control.
This 10:1 ratio gives a more accurate vehicle trajectory than running the
integrator at the control rate.

The simulation state lives in two vectors:
- `e[1..6]` — velocities: surge (u), sway (v), heave (w), roll rate, pitch rate, yaw rate
- `p[1..6]` — positions: x (north), y (east), z (depth), roll, pitch, yaw (psi)

Ground-truth position from `p[]` is written into the `real_*` sensor variables
(`REAL_X`, `REAL_Y`, `REAL_Z`, `REAL_PSI`, etc.) each cycle. These are logged
separately from the estimated navigation variables (`M_POS_N`, `M_POS_E`,
`M_DEPTH`, `M_YAW`) and can be compared post-mission to assess navigation error.

The simulation aborts automatically if the total velocity exceeds 100 m/s
(numerical instability check).

---

## Fault Injection

The simulator includes a scripted fault injection engine (`load_fault.c`) that
lets you corrupt sensor readings or vehicle parameters at runtime according to
rules defined in a plain-text fault definition file.

### Invoking fault injection

Pass the fault file as a second command-line argument:

```bash
./ody sim dive240.mi cg_shift_demo.fd
```

The fault engine looks for bare fault filenames inside the `SFault/` directory
(defined as `SFAULT_DIR` in `paths.h`). If the file is not found, fault
injection is silently skipped - it is always optional. You can also pass an
explicit relative or absolute fault path.

Alternatively, a mission file can specify a fault file by setting
`vvp->fault_file_name` before the simulator starts (e.g. via a `sensor:` line
for a data-structure variable that holds the filename). The command-line argument
takes precedence.

### Fault file format

Fault files are plain text. Lines beginning with `#` are comments. Each rule
occupies one line and has the form:

```
fault: <condition> -> <action> [; <action> ...]
```

#### Conditions

| Form | Meaning |
|---|---|
| `sensor_name op value` | Comparison against a literal |
| `sensor_name op sensor_name` | Comparison between two sensor variables |
| `cond1 && cond2` | Both conditions must be true |
| `cond1 \|\| cond2` | Either condition must be true |
| `prob(rate)` | Fires stochastically; `rate` is probability per second |

Relational operators: `>`, `<`, `>=`, `<=`, `==`, `!=`.  
Up to 8 terms per condition. `&&` binds before `||`, evaluated left to right.

#### Actions

Each action is an assignment:
```
target_sensor = value_or_sensor_name
```

Multiple actions on one rule are separated by semicolons:
```
fault: m_depth > 5.0 -> vs_xg = 0.082 ; vs_yg = 0.010
```

The target and source must both be sensor variables present in the data structure
(i.e. declared in `edit_struct.c`). Literal numeric values are also accepted on
the right-hand side.

#### Example fault file

```
# SFault/cg_shift_demo.fd
# Forward CG shift triggered at depth > 5 m

fault: m_depth > 5.0 -> vs_xg = 0.082
```

This shifts the vehicle's longitudinal center of gravity forward by 0.082 m
(equal to the nominal CG–CB vertical separation) once the vehicle passes 5 m
depth, introducing a nose-down pitch moment the elevator must counteract.

#### Probabilistic fault example

```
# Random depth sensor spike, ~0.5 events per second on average
fault: prob(0.5) -> m_depth = 999.0
```

### How faults integrate with logging

- `apply_faults()` is called after `sensor_processing()` and before
  `layered_control()`, so all behaviors and controllers see the corrupted values.
- The sensor variable `M_FAULT_ACTIVE` is updated every control cycle with the
  count of rules that fired. It is logged automatically, making it easy to see
  exactly when faults were active in post-mission analysis.
- `log_data()` embeds both the fault file name and the full text of the `.fd`
  file in the ODB data file header, so every data file is self-documenting: you
  can reconstruct the exact fault scenario from the data file alone.

### Adding new fault scenarios

Create a new `*.fd` file in `SFault/` — no recompilation required. Any sensor
variable in the data structure can be used in conditions or actions. To make a
new quantity injectable, add it to `edit_struct.c` and regenerate (see
**Modifying the Data Structure** above).

---

## Data Logging and Analysis

`log_data()` runs every 200 ms and writes only variables that have changed since
the last cycle (detected via timestamp). The data file is binary with a
self-documenting ASCII header.

### File locations (from `paths.c` / `paths.h`)

On Linux, runtime paths are rooted at `ODY_ROOT` when that environment variable
is set. Otherwise they are rooted at the executable location. When you use
`./ody`, the wrapper runs from the repository root.

| Path | Location |
|---|---|
| Data files | `./SData/` |
| System log | `./system.log` |
| Mission files | `./SMission/` for bare names; explicit paths are also accepted |
| Fault files | `./SFault/` |
| Vehicle definitions | `./Vehicle/` |

`make setup` and `./ody setup` create `SData/`, `SMission/`, and `SFault/`.

### Data file format

1. **ASCII header** — mission start time, data structure creation date, mission
   file contents, and (if fault injection was used) the fault file name and the
   full text of the `.fd` file.
2. **ASCII index section** — maps integer indices to sensor names, e.g.
   `5 m_depth(m)`. Lines starting with `#` are comments.
3. **Binary data section** — pairs of `(short int index, float value)`.
   Each time group begins with `index > 10000`; the paired float is mission
   elapsed time in seconds.

Because sensor names are embedded in the file, `parse.c` can read and plot data
without access to the current `vehicle.h`. Data files are forward- and
backward-compatible across changes to the data structure.

### Post-mission parsing

```bash
gcc -o parse parse.c -lm
./parse SData/my_data_file
```

Set the environment variable `ODY_AUTO_PARSE=1` to have `parse` run
automatically when the simulation ends. Set `ODY_PARSE_OPTION` to pass
additional flags to `parse`.

---

## File System Setup

Before running for the first time:

```bash
mkdir -p SMission SFault
```

Place mission files in `./SMission/` or provide an explicit path on the command
line. Place fault definition files in `./SFault/` or provide an explicit path.
The hydrodynamic coefficient files must be available from the repository root:

```
./get_am.tex
./get_drag.tex
./get_shape.tex
```

---

## Appendix: LLM Coding Guide — Vehicle Data Structure Discipline

This appendix is written specifically for AI coding assistants (LLMs) working
on this codebase. It explains a design invariant that is easy to miss and costly
to violate.

### The core rule

**Every quantity that matters — sensor readings, configuration parameters,
behavior arguments, computed intermediate values, fault targets, anything that
needs to be observable, configurable, or injectable — must live in the vehicle
data structure and be accessed exclusively through `sensor()` and `update()`.**

This is not a style preference. The data structure is the integration point for
mission loading, fault injection, and data logging. A value that bypasses it is
invisible to all three systems.

### Why this matters: three consumers of every variable

#### 1. Mission configuration (`load_mission.c`)

The mission parser uses `match_sensor()` to look up variable names by string at
load time and write their initial values via `update()`. If a parameter lives as
a local `#define` or a static in a `.c` file, it cannot be set from a mission
script. The operator has no way to adjust it without recompiling.

**Correct pattern** — parameter in the data structure:
```c
// In edit_struct.c:
//   sensor: uo_waypoint_arrival_radius(m)  5.0

double radius = sensor( UO_WAYPOINT_ARRIVAL_RADIUS );
```
Now the mission script can override it:
```
sensor: uo_waypoint_arrival_radius(m)  10.0
```

**Wrong pattern** — hardcoded constant:
```c
#define ARRIVAL_RADIUS  5.0   /* cannot be tuned without recompiling */
```

#### 2. Fault injection (`load_fault.c`)

The fault engine calls `match_sensor()` on both condition operands and action
targets. It can only reference variables in the data structure. If a quantity
(e.g. a thruster gain, a navigation filter coefficient, a sensor bias) lives
outside the data structure, it cannot be faulted.

This matters for realism: real vehicle failures manifest as corrupted sensor
values, saturated actuator commands, or shifted physical parameters — all of
which are data-structure variables. A robust fault injection study requires that
every quantity of interest be in the structure.

**Correct pattern** — injectable parameter:
```c
// In edit_struct.c:
//   sensor: vs_xg(m)  0.0   (longitudinal CG offset)

// In sim.c — reads from data structure every cycle:
double xg = sensor( VS_XG );
```
Fault file can now inject:
```
fault: m_depth > 5.0 -> vs_xg = 0.082
```

**Wrong pattern** — static physical parameter:
```c
static double xg = 0.0;   /* fault injection cannot reach this */
```

#### 3. Data logging (`log_data.c`)

`log_data()` iterates over `vvp->s[]` and records every variable whose timestamp
has advanced since the last cycle. `update()` sets the timestamp; direct writes
do not. A variable written directly is silently dropped from the log.

This means: if you write to `vvp->s[i].x` instead of calling `update(i, val)`,
the value is not logged. If you maintain state in a file-scope `static`, it is
never logged at all. Post-mission analysis is then blind to that quantity.

**Correct pattern**:
```c
update( M_CROSS_TRACK_ERROR, cte );   /* timestamp set; logger will record it */
```

**Wrong pattern**:
```c
vvp->s[M_CROSS_TRACK_ERROR].x = cte;  /* timestamp not set; not logged */
```

### Behavior arguments follow the same rule

Behavior arguments (`b_arg`) are the mechanism by which a behavior exposes its
tunable parameters to the mission system. Every parameter a behavior uses that
an operator might want to tune must be a declared `b_arg` in `edit_struct.c`.

```
behavior: my_behavior
  b_arg: target_depth(m)       10.0
  b_arg: speed(m/s)             1.0
  b_arg: behavior_active(bool)  0.0
```

Inside the behavior function, read arguments with the standard accessor:
```c
double depth = b->arg[ MY_BEHAVIOR_TARGET_DEPTH ].x;
```

Do not read b_args from file-scope variables or pass them as function arguments
from outside. The mission loader populates `b->arg[]` from the mission script;
your function just reads them.

### The `edit_struct.c` → `vehicle.h/c` workflow is mandatory

When you add a new variable, you must:

1. Add a `sensor:` line to `edit_struct.c` with name, unit, and default value.
2. Compile and run `edit_struct` to regenerate `vehicle.h` and `vehicle.c`.
3. Recompile the simulator.

Do not skip step 2 and attempt to add the index constant by hand to `vehicle.h`
or add a slot to `vehicle.c`. The generated files encode array sizes, name
strings for the logger and parser, and the index-to-name mapping used by
`match_sensor()`. Hand-editing them produces inconsistencies that are hard to
diagnose.

### Checklist for adding any new quantity

Before writing code that introduces a new value, ask:

- [ ] Does the operator need to set or tune this from a mission script? → `sensor:` in `edit_struct.c` with a `u_` or `uo_` prefix.
- [ ] Is this a measured or computed vehicle state? → `sensor:` with an `m_` prefix.
- [ ] Do I want to be able to inject a fault into this value? → `sensor:` in `edit_struct.c` (any prefix).
- [ ] Do I need this value in the post-mission data file? → `sensor:` in `edit_struct.c`; write only via `update()`.
- [ ] Is this a tunable parameter of a behavior? → `b_arg:` under the relevant `behavior:` block in `edit_struct.c`.

If any answer is yes, the value belongs in the data structure. The overhead of
adding a sensor variable is negligible; the cost of missing one shows up later
as an untuneable parameter, an unfaultable quantity, or a gap in the data log.

### Summary

| Action | Correct | Wrong |
|---|---|---|
| Read a sensor | `sensor( INDEX )` | `vvp->s[INDEX].x` |
| Write a sensor | `update( INDEX, val )` | `vvp->s[INDEX].x = val` |
| Tunable parameter | `sensor:` in `edit_struct.c` | `#define` or `static` |
| Behavior parameter | `b_arg:` in `edit_struct.c` | function argument or global |
| New variable | run `edit_struct`, recompile | hand-edit `vehicle.h` |
