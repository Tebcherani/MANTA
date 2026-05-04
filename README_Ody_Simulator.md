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
- Any future MEX / MATLAB integration

The architecture is described in:

> Bellingham, J. G., and J. Leonard. "Task Configuration with Layered Control."
> IARP Workshop on Mobile Robots for Subsea Environments, 1994.

---

## Quick Start

```bash
# 1. Compile (see Build section below for full details)
gcc -D_SIMULATOR -o sim sim_main.c sim.c sim_bottom.c sim_current.c \
    sim_conduct.c sim_usbl.c sensor_load.c layered_control.c \
    dynamic_control.c whoi_traj.c log_data.c load_mission.c \
    vehicle.c update.c diagnostic.c fileman.c filters.c nrutil.c \
    lbl.c lbl_nav.c ekf_work.c waypoint.c wiggle.c yo_yo_depth.c \
    yo_yo_temperature.c yoyo_new.c whoi_homing.c docking.c wfparse.c \
    getopt.c -lm

# 2. Run with a mission file
./sim missions/my_mission.txt
```

Output files are written to `./Data/` by default (created automatically).
The system log goes to `./system.log`.

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

### Data logging

| File | Purpose |
|---|---|
| `log_data.c` | Binary data file writer (`log_data()`, `init_data_logging()`) |
| `fileman.c` | Data file management (`set_filesize()`, `truncate_file()`) |
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
| `wfparse.c` | Waypoint file parser (used by `waypoint.c`) |

### Utility and support

| File | Purpose |
|---|---|
| `diagnostic.c` | Message routing to terminal and syslog (`write_message()`, `open_syslog()`) |
| `diagnostic.h` | Status codes, `TERM`/`SLOG` flags, prototypes |
| `filters.c` | Ring-buffer median filter (`RB_Init()`, `RB_AddVal()`, etc.) |
| `filters.h` | Ring-buffer types and prototypes |
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
| `paths.h` | File system paths (resolves to `"."` on non-OS-9 builds) |

### Configuration

| File | Purpose |
|---|---|
| `Sim.cfg` | Simulator vehicle configuration for `edit_struct` |
| `ody_serial.h` | Serial port types — retained because some headers include it; safe to ignore |
| `ody_serio.h` | Serial I/O types — same as above |

---

## Build

### Compiler and flags

```bash
gcc -D_SIMULATOR -O2 -Wall -o sim [source files...] -lm
```

The `-D_SIMULATOR` flag is essential. It activates the correct vehicle
configuration block in `ody_defs.h` and `ody_name.h`, setting
`VEHICLE_NAME "Simulator"` and enabling the right optional subsystems.
Without it the build will fail or produce incorrect behavior.

### Recommended Makefile

```makefile
CC      = gcc
CFLAGS  = -D_SIMULATOR -O2 -Wall -g
LDFLAGS = -lm

SRCS = sim_main.c sim.c sim_bottom.c sim_current.c sim_conduct.c \
       sim_usbl.c sensor_load.c layered_control.c dynamic_control.c \
       whoi_traj.c log_data.c load_mission.c vehicle.c update.c \
       diagnostic.c fileman.c filters.c nrutil.c lbl.c lbl_nav.c \
       ekf_work.c waypoint.c wiggle.c yo_yo_depth.c yo_yo_temperature.c \
       yoyo_new.c whoi_homing.c docking.c wfparse.c getopt.c

OBJS = $(SRCS:.c=.o)

sim: $(OBJS)
	$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS)

%.o: %.c
	$(CC) $(CFLAGS) -c -o $@ $<

clean:
	rm -f $(OBJS) sim
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

**3. `fileman.c` has an OS-9-only `set_filesize()` implementation.**
The `#ifdef _OSK` block around `set_filesize()` means the function compiles to a
no-op on Linux. Data files will grow unbounded rather than being pre-allocated.
This is fine for development but worth noting for long missions.

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
./sim my_mission.txt
```

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

### Interactive keys (if compiled with `-DSIM_MODEM_INTERFACE`)

While running, key presses are processed without blocking:

| Key | Action |
|---|---|
| `p` | Pause simulation |
| `r` | Resume simulation |
| `d` | Print current depth |
| `t` | Print current mission time |
| `x` | Call `exit_gracefully()` and quit |

---

## Data Logging and Analysis

`log_data()` runs every 200 ms and writes only variables that have changed since
the last cycle (detected via timestamp). The data file is binary with a
self-documenting ASCII header.

### File locations (from `paths.h`)

On Linux, all paths resolve relative to the working directory (`.`):

| Path | Location |
|---|---|
| Data files | `./Data/` |
| System log | `./system.log` |
| Mission files | `./Mission/` (can also use absolute paths) |

Create `./Data/` before running if it does not exist, or `log_data()` will fail
to open the output file.

### Data file format

1. **ASCII header** — mission start time and data structure creation date.
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
./parse my_data_file
```

Set the environment variable `ODY_AUTO_PARSE=1` to have `parse` run
automatically when the simulation ends. Set `ODY_PARSE_OPTION` to pass
additional flags to `parse`.

---

## File System Setup

Before running for the first time:

```bash
mkdir -p Data Mission
```

Place mission files in `./Mission/` or provide a full path on the command line.
The hydrodynamic coefficient files must be in the working directory:

```
./get_am.tex
./get_drag.tex
./get_shape.tex
```
