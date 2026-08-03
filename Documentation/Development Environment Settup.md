# Odyssey Simulator — Cheat Sheet

---

## A. Development Environment Setup

### Prerequisites

```bash
sudo apt install build-essential git
```

Python 3.12 is included with Ubuntu 24.04. See `Ubuntu_Setup.md` for
the one-time Python venv and package setup.

### First-time clone and build

```bash
git clone <repo-url>
cd <repo-directory>

make both          # builds sim and ksim_bin; also creates SMission/ SFault/
make edit_struct   # builds the data-structure generation tool
```

### What `make` expects in the working directory

These hydrodynamic coefficient files must be present (they are in the repo):

```
get_am.tex      — added-mass coefficients
get_drag.tex    — drag coefficients
get_shape.tex   — vehicle shape parameters
```

If missing, `sim.c` falls back to built-in Draper coefficients (less accurate).

### Build targets

| Command | Effect |
|---|---|
| `make` | Build `sim` (and run `make setup`) |
| `make both` | Build `sim` and `ksim_bin` |
| `make clean` | Remove `.o` files, `sim`, `ksim_bin`, `edit_struct` |
| `make debug` | Build with AddressSanitizer → produces `sim_debug` |
| `make setup` | Create `SData/` `SMission/` `SFault/` (idempotent) |
| `make edit_struct` | Build the data-structure generation tool |

### Kinematic verification with `ksim_bin`

`ksim_bin` is a separate verification executable that uses the same mission loader
and control pipeline as `sim`, but replaces the hydrodynamic simulator with a
nominal kinematic motion model.

Use it for mission validation without full hydrodynamics:

```bash
./ksim_bin --parse SMission/dive240.mi
./ksim_bin --nolog --dump-behavior behavior_log.json SMission/dive240.mi
```

`--parse` loads the mission and exits; `--nolog` suppresses runtime data output;
`--dump-behavior` writes in-memory behavior activity to JSON.

The mission constructor (`SUtilities/mission_constructor_gui.py` and
`mission_constructor_app.py`) runs `ksim_bin` internally and shows behavior
timeline results in the Results tab.

### Note on `-D_SIMULATOR`

`_SIMULATOR` is defined directly in `ody_name.h` (not via a Makefile flag), so the
Makefile does not need `-D_SIMULATOR`. Do not add it to `CFLAGS` — it would be
doubly defined and harmless but confusing.

---

## B. Running the Simulator — Starting Fresh

### Run a mission

```bash
./sim SMission/dive240.mi                        # plain mission
./sim SMission/dive240.mi cg_shift_demo.fd       # with fault injection
```

The fault file name is relative to `SFault/`; you do not need to write the full path.

### Runtime output

| Location | Contents |
|---|---|
| `SData/S<YYMMDDN>.obd` | Binary data log (auto-named by date/sequence) |
| `SData/Simulator.tab` | Tab file written at session end |
| `system.log` | Diagnostic / syslog messages |

### Mission file format (`.mi` in `SMission/`)

```
UseSim 1

# Set sensor initial values
sensor: m_pos_n(m)          0.0
sensor: m_pos_e(m)          0.0
sensor: uo_control_kph      0.75
sensor: uo_control_pitch_lim(deg)  30   # degrees converted automatically

state: my_state_name

behavior: mission_timer 1           # priority 1 = highest
  b_arg: time(s) 600

behavior: depth_envelope 2
  b_arg: max_depth(m)  100.0
  b_arg: min_depth(m)    0.5

behavior: setpoint 3
  b_arg: heading(rad)  1.57
  b_arg: depth(m)     15.0
  b_arg: speed(m/s)    1.5
  b_arg: time(s)     600.0
```

### Fault file format (`.fd` in `SFault/`)

```
# One rule per line.  Lines starting with # are comments.
fault: <condition> -> <target> = <value> [; <target> = <value> ...]

# Examples:
fault: m_depth > 5.0 -> vs_xg = 0.082
fault: m_depth > 10.0 && m_speed > 1.0 -> m_depth = 999.0
fault: m_depth > 5.0 || m_roll > 0.5   -> uo_control_kph = 0.0
fault: prob(0.5) -> m_depth = 999.0     # fires ~0.5x/s on average
```

Condition operands and action targets must be sensor names present in the data
structure. Literal numeric values are allowed on the right-hand side of actions.

### Reviewing data

**GUI review application:**
```bash
.venv/bin/python3 Review/launch_review.py
```
Select a file from the dropdown (newest first), click **Load**, tick the panels
you want, click **Generate Plots**.

**Review2 — compare two runs side-by-side:**
```bash
.venv/bin/python3 Review/review2.py
```

**C parser** (text dump):
```bash
gcc -o parse parse.c -lm          # build parser once
./parse SData/S2612802.obd        # dump data from a run
```

### Modifying the vehicle data structure

When adding a new sensor, behavior, or behavior argument:

```bash
# 1. Edit the variable definition section at the top of edit_struct.c
#    Add:  sensor: my_new_var(unit)  default_value
#       or behavior: my_behavior / b_arg: arg_name(unit) default

# 2. Rebuild edit_struct and regenerate vehicle.h / vehicle.c
make edit_struct
./edit_struct Sim.cfg

# 3. Recompile
make both
```

`vehicle.c` and `vehicle.h` are generated files, but they are committed to the
repo. After regenerating, commit both files.

### Variable naming conventions (quick ref)

| Prefix | Meaning |
|---|---|
| `m_` | Measured / computed vehicle state |
| `u_` | User parameter (set from mission script) |
| `uo_` | User config (control gains, limits) |
| `c_` | Command output from control system |
| `real_` | Simulator ground-truth (not available to vehicle logic) |
| `vs_` | Vehicle physical / structural parameter |
| `wp_` | Waypoint navigation variable |

### Read / write rule (never bypass)

```c
double v = sensor( MY_VAR );   // READ  — use the macro, not vvp->s[i].x
update( MY_VAR, new_val );     // WRITE — sets timestamp; enables logging
```

Writing directly to `vvp->s[i].x` bypasses the timestamp and the value will
**not** appear in the data log.

---

## C. Git Repository Management

### Remote and branch

```bash
git remote -v
git branch         # currently: main (single branch)
git log --oneline  # see recent commits
```

### What is tracked vs. ignored

**Tracked (in repo):**
- All source: `*.c`, `*.h`, `Makefile`, `.cfg`, `.tex`
- Generated but committed: `vehicle.c`, `vehicle.h`
- Mission files: `SMission/*.mi`
- Fault scenarios: `SFault/*.fd`

**Ignored (not tracked):**
- New `.obd` data files (excluded by `.gitignore`)
- Build products: `*.o`, `sim`, `ksim_bin`, `sim_debug`, `edit_struct`
- `system.log`, `*.tmp`
- `.venv/`

### Typical workflow

```bash
# Check status
git status
git diff

# Stage specific files (avoid git add -A to prevent accidental binary commits)
git add layered_control.c waypoint.c SMission/new_mission.mi

# Commit
git commit -m "Add new_mission with extended depth profile"

# Push
git push origin main
```

### After modifying the data structure

`vehicle.c` and `vehicle.h` must be committed together with the `edit_struct.c`
change that generated them:

```bash
git add edit_struct.c vehicle.c vehicle.h
git commit -m "Add m_cross_track_error to data structure"
```

### Adding a new fault scenario

```bash
# Create the file, test it, then commit
git add SFault/my_new_fault.fd
git commit -m "Add thruster-loss fault scenario"
```

### Useful one-liners

```bash
git log --oneline -10                 # recent history
git diff HEAD~1 -- load_fault.c      # what changed in a file since last commit
git show HEAD:SMission/dive240.mi    # view a committed mission file
git stash                            # shelve uncommitted changes temporarily
git stash pop                        # restore shelved changes
```
