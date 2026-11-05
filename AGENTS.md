# Instructions for working in MANTA

These instructions apply throughout the repository. Read any additional
`AGENTS.md` in the area you are changing for more specific instructions.

## Read before changing code

- Read [README.md](README.md) for setup, commands, and project context.
- Read [docs/code_structure.md](docs/code_structure.md) for component layout,
  state organization, naming, and logging conventions.
- Read the affected component's documentation and initialization and sequence
  diagrams in its `docs/` directory before changing its behavior.
- When available on the current branch, read
  `docs/documentation_rendering_guide.md` before changing documentation rendering.
- Verify relevant details against the current branch's code. If documentation
  conflicts with the implementation, identify the discrepancy rather than
  silently choosing a new architecture.

## Scope of changes

- Change only files necessary to complete the requested task. For component work,
  keep changes within that component unless shared code, interfaces,
  configuration, or associated documentation must also change.
- Before changing files outside the component, establish why they are needed.
  Explain the dependency and ask before proceeding if it materially expands the
  requested scope. Necessary related changes already covered by the request do
  not require another approval.
- Do not perform unrelated cleanup, refactoring, renaming, formatting, or
  documentation edits. Do not reformat untouched code or whole legacy files as
  part of a functional change.
- Inspect the working tree before editing. Preserve unrelated changes, including
  untracked files; do not overwrite, revert, stage, or commit them.
- Review the final diff for accidental edits and unnecessary files. Stage only
  task-related files when a commit is requested.

## Code completeness and reuse

- Search the affected component and relevant shared utilities before adding a
  function. Reuse or extend existing functionality where appropriate; do not
  duplicate implementations or introduce a parallel way to perform the same
  operation without a concrete need.
- Follow existing component responsibilities. Place shared functionality in an
  established shared location only when the task requires it.
- Deliver working functionality. Do not leave empty functions, placeholder
  return values, `pass`, or `NotImplementedError` as substitutes for completing
  the requested work. Intentional no-op callbacks, abstract methods, and similar
  constructs must have a clear purpose documented where it is not obvious.
- Do not leave commented-out implementations or unfinished TODOs in place of
  working code. Report any work that cannot be completed explicitly.
- Remove imports, branches, and helpers made unused or obsolete by your change,
  after checking their callers and uses. Keep this cleanup within the task.
- Introduce abstractions and dependencies only when they serve a concrete need
  in the requested work.

## Coding style

Apply these guidelines to new code and edited sections. Preserve surrounding
conventions in legacy code; do not break interfaces or change behavior solely
to satisfy a style preference. Explicit MANTA conventions take precedence over
the general guides.

### Python

- Follow [PEP 8](https://peps.python.org/pep-0008/) and
  [PEP 257](https://peps.python.org/pep-0257/) for docstrings, with the shared
  MANTA conventions below taking precedence.
- Group standard-library, third-party, and local imports separately. Follow the
  existing package and launcher import structure.
- Maintain compatibility with the Python version supported by the repository.

### C and C++

- For C, follow the
  [MaJerle C style guide](https://github.com/MaJerle/c-code-style/blob/main/CODING_RULES.md),
  subject to the shared MANTA and legacy-code conventions.
- For ROS C++, follow the
  [ROS 2 Jazzy C++ guidelines](https://github.com/ros2/ros2_documentation/blob/jazzy/source/The-ROS2-Project/Contributing/Code-Style-Language-Versions.rst),
  which adapt Google's C++ style. MANTA's four-space indentation, same-line
  opening braces, and naming rules below are explicit project exceptions.
- Always use braces for control statements, including single-line bodies. Use
  same-line opening braces for definitions and control statements in new code.
- Make ownership, resource lifetimes, and error handling explicit. Use idiomatic
  C++ types and resource management in C++ rather than treating it as C.
- Preserve the existing compiler standard and build requirements. Adopting the
  style guide does not authorize changing language standards or build tooling.
- Preserve existing attribution and license notices. Do not invent ownership or
  copyright claims; flag inaccurate headers for maintainer review.

### Shared naming, formatting, and units

- Use four spaces for indentation in Python, C, and C++; do not use tabs.
- Use `snake_case` for functions and variables, `PascalCase` for Python and C++
  classes, and `UPPER_SNAKE_CASE` for constants. Preserve established C type
  naming and existing API conventions.
- Aim for a maximum of 100 characters per source-code line. Wrap earlier when
  that improves readability; do not rewrap untouched code. Long URLs and other
  text that cannot usefully be split may exceed the limit.
- Label physical quantities explicitly with unit suffixes in new or changed
  variable, parameter, state-field, constant, configuration-key, and custom ROS
  message-field names. Examples include `duration_s`, `range_m`, `speed_m_s`,
  `remaining_energy_j`, `heading_rad`, and `MAX_RANGE_M`.
- Use `_b` for Boolean variables and state fields, following
  `docs/code_structure.md`. Give counts and other dimensionless values clear
  names such as `vehicle_count`; do not invent physical units for them.
- Document units for return values, array elements, and fields whose names are
  fixed by an external API. Label units in output column names, plot axes, and
  tables when adding or changing those outputs.
- Prefer SI units for new interfaces unless an established convention requires
  otherwise. Document other units and make conversions explicit at boundaries;
  never silently reinterpret a value or change simulation behavior for style.
- Specify coordinate frames, axis directions, and angle conventions as well as
  units where relevant.
- Preserve existing public names, ROS interfaces, and configuration keys during
  unrelated work. If an existing name lacks units, clarify them in a nearby
  comment or docstring when relevant; renaming a contract requires a coordinated
  change within the requested scope.
- Document public interfaces and non-obvious assumptions, side effects, and
  design decisions. Use Python docstrings and C/C++ documentation comments
  (`/** ... */` or `///`) where appropriate.

### ROS interfaces and behavior

- Follow the affected package's structure and `docs/code_structure.md`.
- Preserve topic, service, parameter, and message contracts unless changing them
  is part of the task. Use compatible QoS settings for communicating endpoints
  and explain non-obvious choices.
- Preserve simulation-time semantics, callback ordering, and synchronization.
  Do not substitute wall-clock time for simulation time without an explicit
  requirement.
- Refer to [REP 103](https://github.com/ros-infrastructure/rep/blob/master/rep-0103.rst)
  for ROS units and coordinate conventions. Document MANTA's actual frames and
  any conversions, including NED/ENU and compass-heading differences; adopting
  the guide does not authorize changing existing coordinate semantics.

### Configuration, build files, and documentation

- Follow the ROS 2 CMake conventions: lowercase commands, two-space indentation,
  and no space between a command name and its opening parenthesis.
- Use two-space indentation in YAML and XML, preserving established schemas and
  key names. Preserve required recipe tabs in Makefiles.
- In shell scripts, quote variable expansions unless splitting is intentional
  and handle command failures explicitly.
- Use consistent Markdown heading levels, language labels on fenced code blocks,
  and readable prose wrapping. Document reasoning and contracts rather than
  restating code. Keep PlantUML names and interactions consistent with the
  component and edit diagram sources rather than rendered output.

## Simulation behavior and documentation

- Preserve simulation behavior unless the requested task changes it. Treat
  timing, random seeds, units, coordinate frames, fault behavior, and output
  formats as behavior that can affect results.
- Make units and modeling assumptions explicit in changed code where relevant.
  Follow the state and variable naming conventions in `docs/code_structure.md`.
- Keep documentation and component diagrams consistent with the behavior changed
  by the task. Explain why a design exists where that reasoning is not obvious.
- Link to existing explanations rather than duplicating them. Edit documentation
  sources, not generated HTML or other rendered output.

## Validation and handoff

- Run checks appropriate to the changed behavior and affected component. Use
  existing checks where available; do not invent test commands or assume a
  repository-wide test suite exists.
- Use `./manta build` for changes requiring the native or ROS build. For runtime
  changes, use a suitably short, relevant mission run when practical. For result
  comparisons, hold configuration, seeds, and other inputs constant and describe
  any observed differences without claiming unverified equivalence.
- Use `./manta docs build` to validate
  documentation changes, following the rendering guide.
- Run `git diff --check` and inspect the final diff. Keep generated outputs,
  build products, logs, and virtual environments out of commits.
- In the handoff, state what changed, what was checked, and any checks not run or
  unresolved limitations. Never claim a check passed unless it was performed.

  ## Purpose and deployment goals

MANTA supports developing fault detection and response methods for use on real
vehicles. Keep vehicle code, these methods, and operational logging suitable for
field deployment with minimal changes.

- Prefer reusing vehicle implementations, with simulation-specific behavior
  separated where practical. Make simulator-only inputs and assumptions clear.
- Consider onboard computing, timing, memory, and storage limits when designing
  methods and logging. Account for imperfect inputs and explain response behavior.
- When discussing design choices with the user, raise material tradeoffs between
  simulation convenience and eventual deployment, including work needed to
  transfer an approach to a real vehicle.
- Apply this goal to new work within the requested scope. Existing simulation
  simplifications can remain; these guidelines do not require a broad rewrite
  or imply that simulation results alone establish field readiness.
