# Ubuntu / Linux Setup Notes

Development environment for the Odyssey AUV Simulator on Ubuntu 24.04 LTS (including WSL2).

> Current quick path: use `./ody setup --system` on a fresh Ubuntu/WSL machine,
> then use `./ody sim`, `./ody ksim`, `./ody review`, `./ody review2`, and
> `./ody mission-constructor` for daily work. See `RUN_UBUNTU.md` for the
> concise command guide. The notes below are retained as background and
> troubleshooting detail.

---

## 1. Prerequisites

```bash
sudo apt install build-essential git
```

Python 3.12 is included with Ubuntu 24.04. The project uses a `.venv` at the repo
root — all Python tools (review, review2, mission constructor) run from it.

---

## 2. First-Time Build

```bash
# Clone the repo (substitute the actual URL)
git clone <repo-url>
cd <repo-directory>

# Build the simulator and kinematic verifier
make both

# Build the data-structure tool
make edit_struct
```

`make both` produces `./sim` and `./ksim_bin`. `make edit_struct` produces `./edit_struct`.
`make` alone builds only `sim`.

---

## 3. Python Virtual Environment

The project `.venv` at the repo root is the canonical Python environment.

### 3.1 Bootstrap pip (first time only)

Ubuntu 24.04 ships Python 3.12 without pip in the venv. Bootstrap it once:

```bash
curl -s https://bootstrap.pypa.io/get-pip.py -o /tmp/get-pip.py
.venv/bin/python3 /tmp/get-pip.py
```

### 3.2 Install packages

```bash
.venv/bin/pip install PyQt5 matplotlib numpy xarray
.venv/bin/pip install -e Review/
```

The `-e` flag installs `ody_review` in editable mode so changes to `Review/ody_review/`
take effect immediately without reinstalling.

### 3.3 System libraries for PyQt5 / xcb

If the review tools crash with `Could not load the Qt platform plugin "xcb"`:

```bash
sudo apt-get install -y \
    libxcb-icccm4 \
    libxcb-image0 \
    libxcb-keysyms1 \
    libxcb-render-util0 \
    libxcb-shape0 \
    libxcb-xinerama0 \
    libxcb-xkb1 \
    libxkbcommon-x11-0
```

To diagnose missing libraries:

```bash
find .venv/lib -name "libqxcb.so" | head -1 | xargs ldd | grep "not found"
```

---

## 4. Running the Tools

### Simulator

```bash
./sim SMission/dive240.mi
./sim SMission/dive240.mi cg_shift_demo.fd    # with fault injection
```

### Kinematic verifier

```bash
./ksim_bin SMission/dive240.mi
./ksim_bin --parse SMission/dive240.mi        # parse only, no run
./ksim_bin --nolog --dump-behavior behavior_log.json SMission/dive240.mi
```

### Review GUI

```bash
.venv/bin/python3 Review/launch_review.py
```

### Review2 comparison tool

```bash
.venv/bin/python3 Review/review2.py
```

### Mission constructor (GUI)

```bash
.venv/bin/python3 SUtilities/mission_constructor_gui.py
```

### Mission constructor (web)

```bash
.venv/bin/python3 SUtilities/mission_constructor_app.py
# then open http://127.0.0.1:8000/ in a browser
```

---

## 5. Modifying the Data Structure

`vehicle.h` and `vehicle.c` are generated from `edit_struct.c`. After any change
to the data structure definition:

```bash
# 1. Edit the sensor/behavior definitions near the top of edit_struct.c
# 2. Rebuild edit_struct and regenerate
make edit_struct
./edit_struct Sim.cfg
# 3. Recompile the simulator
make both
```

Always run `./edit_struct` from the project root (it reads `edit_struct.c` from
the current directory).

---

## 6. Quick-Start Checklist (fresh Ubuntu install)

```bash
# 1. System packages
sudo apt install build-essential git
sudo apt-get install -y \
    libxcb-icccm4 libxcb-image0 libxcb-keysyms1 \
    libxcb-render-util0 libxcb-shape0 libxcb-xinerama0 \
    libxcb-xkb1 libxkbcommon-x11-0

# 2. Clone and build
git clone <repo-url>
cd <repo-directory>
make both
make edit_struct

# 3. Bootstrap pip and install Python packages
curl -s https://bootstrap.pypa.io/get-pip.py -o /tmp/get-pip.py
.venv/bin/python3 /tmp/get-pip.py
.venv/bin/pip install PyQt5 matplotlib numpy xarray
.venv/bin/pip install -e Review/

# 4. Verify
./sim SMission/reference_survey_yoyo.mi
./ksim_bin SMission/reference_survey_yoyo.mi
.venv/bin/python3 Review/launch_review.py
```

---

## 7. Known Port Issues (already fixed in this codebase)

These are recorded for reference — the fixes are already applied.

### Buffer overflow in `ody_defs.h`

`MAX_CHARS` was 30; `vs_thruster_current_limit(Amp)` is 30 characters plus a null
terminator. Increased to 40.

### CRLF line endings in mission files

Mission files authored on Windows use `\r\n`. Added `strip_cr()` to
`load_mission.c`, called after every `fgets`, to strip carriage returns before
parsing.

### `ksim` binary name collides with `ksim/` directory

The Makefile names the kinematic verifier `ksim_bin` on Linux (vs `ksim.exe` on
Windows) to avoid the linker trying to overwrite the source directory.
