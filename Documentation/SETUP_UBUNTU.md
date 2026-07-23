# Ubuntu development setup

This repository is the Ubuntu development version. Keep each checkout on the
native Linux filesystem, such as `~/src/Odyssey_sim_Ubuntu`. Do not develop in
a path under `/mnt/c` or in a OneDrive-synchronized directory.

## First-time system setup

```bash
sudo apt update
sudo apt install -y build-essential gdb git python3 python3-pip python3-venv
```

## Project setup

From any directory, run:

```bash
bash ~/src/Odyssey_sim_Ubuntu/scripts/setup_venv.sh
cd ~/src/Odyssey_sim_Ubuntu
source .venv/bin/activate
```

The script:

- finds the repository independently of the caller's current directory;
- requires Python 3.9 or newer;
- creates the single workspace environment at `.venv`;
- installs the root dependency manifest; and
- verifies the application's Python imports.

Virtual environments are machine-specific. Never copy `.venv` between
computers or between Windows and Ubuntu.

## Build and verify

```bash
cd ~/src/Odyssey_sim_Ubuntu
make clean all
source .venv/bin/activate
python -c "import ody_review; print('Python environment OK')"
```

Open `00OdyC.code-workspace` in VS Code. It already selects the portable
`${workspaceFolder}/.venv/bin/python3` interpreter.

## Daily workflow

```bash
cd ~/src/Odyssey_sim_Ubuntu
git pull --ff-only
bash ./scripts/setup_venv.sh
source .venv/bin/activate
```

Use GitHub to synchronize source changes between Ubuntu machines. Build
products, caches, `.venv`, and simulator output remain local and ignored.
