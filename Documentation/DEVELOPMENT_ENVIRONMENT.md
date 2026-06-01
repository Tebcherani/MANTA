# Development Environment

This project lives in OneDrive and is used from more than one Windows
machine. Keep shared project files portable, and put machine-specific
compiler paths outside the OneDrive workspace.

## Shared Workspace Rules

- Keep `.vscode/settings.json` free of absolute `C:` or `D:` paths.
- Prefer tools from `PATH`: `gcc`, `make`, `python`.
- Do not store per-machine compiler paths in `.vscode/c_cpp_properties.json`,
  `.vscode/tasks.json`, or the project `Makefile`.
- Keep local-only VS Code settings in VS Code User Settings on each machine.

## Makefile Overrides

The project `Makefile` includes this optional per-user file. On Windows, it
checks both `HOME` and `USERPROFILE` because MSYS2 and PowerShell may disagree
about where `HOME` points:

```make
$(HOME)/.ody-auv-sim/Windows_Ody_C.mk
$(USERPROFILE)/.ody-auv-sim/Windows_Ody_C.mk
```

Create that file separately on each machine if the compiler is not on PATH or
if you need local flags. When using MSYS2 `make`, prefer MSYS-style paths
(`/c/...`, `/d/...`) so `PATH` is parsed correctly. For example:

```make
MSYS2_HOME = /d/ProgramsD/msys64
CC = $(MSYS2_HOME)/mingw64/bin/gcc.exe
PATH := $(MSYS2_HOME)/mingw64/bin:$(MSYS2_HOME)/usr/bin:$(PATH)
export PATH
```

or, if `gcc` is already on PATH:

```make
CC = gcc
```

Keep this file outside OneDrive so the work machine and home machine can use
different tool locations without overwriting each other.

## VS Code

Use the same OneDrive workspace on both machines, but keep local paths in VS
Code User Settings, not workspace settings. Good workspace settings are things
that are true everywhere, such as:

```json
{
  "makefile.configureOnOpen": false
}
```

Bad workspace settings are things like:

```json
{
  "C_Cpp.default.compilerPath": "D:/msys64/ucrt64/bin/gcc.exe"
}
```

Put those in User Settings on the machine where they apply.

## Compiler Requirement

This legacy code is intended to build with GNU89 semantics:

```sh
gcc -std=gnu89
```

Do not switch the shared project to C99/C11/C17 just to satisfy one local
toolchain.
