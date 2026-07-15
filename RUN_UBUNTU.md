# Ubuntu Setup and Run Guide

This repository is the Ubuntu/Linux development copy of the Odyssey simulator.
Keep it on the Linux filesystem, for example:

```bash
mkdir -p ~/src
cd ~/src
git clone https://github.com/jbellingham01/Odyssey_sim_Ubuntu.git
cd Odyssey_sim_Ubuntu
```

Avoid running the Ubuntu build from `/mnt/c/...` or OneDrive. WSL is much more
reliable when the checkout lives under `~/src`.

## First-Time Setup

On a fresh Ubuntu or WSL machine:

```bash
./ody setup --system
```

That installs Ubuntu packages with `sudo apt`, creates `.venv`, installs the
Python review tools, builds `sim`, builds `ksim_bin`, and runs an environment
check.

If the machine already has compiler and Python system packages:

```bash
./ody setup
```

To check the current environment at any time:

```bash
./ody doctor
```

## Daily Commands

Build both C executables:

```bash
./ody build
```

Run the full simulator:

```bash
./ody sim dive240.mi
```

Run the kinematic simulator parser check:

```bash
./ody ksim
```

Run ksim and write a behavior log:

```bash
./ody ksim --nolog --dump-behavior SUtilities/ksim_runs/latest.json dive240.mi
```

Launch the Review GUI:

```bash
./ody review
```

Launch Review2:

```bash
./ody review2
```

Run Review2 from the command line:

```bash
./ody review2 SData/run1.obd SData/run2.obd --opts all --save Review/compare_out
```

Launch the PyQt Mission Constructor:

```bash
./ody mission-constructor
```

Launch the legacy web Mission Constructor:

```bash
./ody mission-constructor-web --port 8000
```

Then open `http://127.0.0.1:8000/`.

## VS Code

Open this folder in VS Code from Ubuntu/WSL:

```bash
code .
```

The repository includes tasks for setup, build, sim, ksim, Review, Review2,
and both Mission Constructor launch paths.

## Notes

- The Ubuntu ksim executable is `ksim_bin` because `ksim/` is a source/runtime
  directory.
- Python dependencies live in `.venv` at the repository root.
- `./ody review`, `./ody review2`, and `./ody mission-constructor` use `.venv`
  automatically; you do not need to activate it first.
- GUI tools require a working desktop display. On Windows 11 WSL this is
  normally handled by WSLg.
