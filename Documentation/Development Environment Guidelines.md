# Development Environment Guidelines

This project builds and runs on Ubuntu Linux (24.04 LTS). The `.venv` at the
repo root is the canonical Python environment for all tools.

## Compiler Requirement

This legacy code uses GNU89 semantics:

```sh
gcc -std=gnu89
```

Do not switch to C99/C11/C17 without an explicit decision to do so.

## Python Environment

All Python tools (review, review2, mission constructor) use the project `.venv`:

```bash
.venv/bin/python3 <script>
```

Do not install packages system-wide or with `--user`. Install into the venv:

```bash
.venv/bin/pip install <package>
```

## Makefile

The Makefile has no machine-specific overrides for Ubuntu — `gcc` and `make`
from `build-essential` are sufficient. The optional include:

```makefile
-include $(HOME)/.ody-auv-sim/Windows_Ody_C.mk
```

is a no-op when that file does not exist, so it can be left in place.

## VS Code

Safe workspace settings (`${workspaceFolder}`-relative, no absolute paths):

```json
{
    "C_Cpp.default.cStandard": "gnu89",
    "python.terminal.activateEnvironment": true
}
```

Set the Python interpreter to `.venv/bin/python3` via the Command Palette
(`Python: Select Interpreter`) rather than hardcoding a path in workspace
settings.
