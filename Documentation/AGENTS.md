# Odyssey AUV Simulator — AI Agent Instructions

This is a legacy C/C++ autonomous underwater vehicle simulator and vehicle-control codebase originating in the 1990s.

## Compiler / Build Expectations

The codebase is intentionally compiled using legacy GNU89 semantics.

Use:

    gcc -std=gnu89

or equivalent Makefile flags.

Do NOT modernize the codebase to C99/C11/C17 unless explicitly requested.

Many modules intentionally rely on:
- implicit int behavior
- older function declaration conventions
- legacy GCC compatibility
- OS-9 portability patterns

Avoid introducing unnecessary modernization changes.

Preserve behavior and compatibility over stylistic cleanup.

## Architectural Rules

All meaningful quantities must live in the vehicle data structure.

Never write directly to:

    vvp->s[i].x

Always use:

    update_sensor()

New tunable or logged quantities must:
1. be added to edit_struct.c
2. regenerate vehicle.h and vehicle.c
3. use update_sensor()
4. remain visible to:
   - mission loading
   - fault injection
   - data logging

vehicle.h and vehicle.c are generated files.

After modifying edit_struct.c:

1. rebuild edit_struct
2. run:

       ./edit_struct Sim.cfg

3. rebuild simulator

## Development Philosophy

Prefer:
- conservative changes
- backward compatibility
- minimal diffs
- preserving simulator behavior

Do not rewrite working legacy systems without explicit instruction.