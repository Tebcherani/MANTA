# AUV Simulator Utilities

This folder contains small utilities for comparing historical Odyssey AUV
field runs with the current simulator. The initial focus is extracting mission
scripts from field logs, checking old missions against the simulator data
structure, and preparing trimmed mission files that the simulator can load.

The utilities are intentionally conservative. They do not modify `vehicle.h`,
`vehicle.c`, or simulator source files. They read the generated simulator
metadata and report or remove items that are not supported by the current
build.

## Requirements

- Python 3
- No third-party Python packages are required.
- Run commands from the repository root:

```powershell
cd "C:\Users\jbellin1\OneDrive - Johns Hopkins\Matlab-JHU\AUV_Mission_Sim\AUV_Sim\Windows_Ody_C"
```

Each utility accepts a folder path as its main argument. Use `--help` on any
script for the full option list.

## Shared Parser

`auv_mission_tools.py` is the shared library used by the command-line tools.
It is not normally run directly.

It provides common parsing for:

- `vehicle.h` simulator variables
- the generated `behaviors` table
- `.slg` mission blocks
- `.mi` mission files
- `.obd` ASCII headers

Names are compared the same way the legacy C mission loader does: units are
ignored during matching. For example, `uo_tau(sec)` and `uo_tau(s)` both match
the simulator variable `uo_tau`.

## Utilities

### `launch_mission_constructor.py`

PyQt5 mission constructor application. This is the recommended constructor
path for new work because it can share Python mission parsing, ksim execution,
review tooling, and future mission-automation logic.

Run from the repository root:

```powershell
py SUtilities\launch_mission_constructor.py
```

The current app includes:

- a behavior catalog loaded from the generated `behaviors` table
- mission stack editing with priority controls
- a Header tab for pasting/editing header lines and selecting initial
  `sensor:` values from the generated simulator variable table
- behavior argument editing
- import/export of `.mi` mission files
- direct script editing with "Apply Script Edits"
- a yoyo reference mission template
- a Mission Checks tab for structural and depth-envelope validation
- a Failure Script tab for generating simulator `.fd` fault-injection files
- unit-aware argument entry, including angle degrees and distance/speed/time
  conversions
- `Run Ksim`, which writes runs under `SUtilities\ksim_runs\` and opens
  simulator plots in the Mission Visualization tab

Examples of accepted convenience inputs:

```text
heading(rad): 90 deg
time(s):      2 nm      # converted using the behavior speed
speed(m/s):   3 kn
time(s):      1 hr
depth(m):     0.05 km
```

The mission file still receives simulator-native values such as radians,
meters, meters/second, and seconds.

The first mission checks include:

- unknown behaviors and behavior arguments
- duplicate priorities
- missing `mission_timer` or `depth_envelope`
- yoyo companion placement
- yoyo top/bottom depth consistency
- commanded depths deeper than `depth_envelope.max_depth`
- commanded depths shallower than `depth_envelope.min_depth`
- non-positive setpoint time/speed
- non-positive descend/ascend pitch or speed

The Header tab separates mission setup from behavior editing. Header lines
such as `UseSim`, monitor settings, and other loader flags can be pasted into
the text editor. Initial `sensor:` values are edited in a table whose variable
column is populated from the generated `sensors` file.

The Failure Script tab generates rules such as:

```text
fault: m_depth > 75 -> vs_xg = 0.082
```

It accepts unit-aware trigger and action values, then writes numeric simulator
values in the generated `.fd` text. For example, `75 m`, `8000 s`, and
`8.2 cm` are converted before saving. Trigger and target variable columns are
editable dropdowns: trigger choices prefer measured/runtime variables such as
`m_*`, while target choices prefer vehicle/simulator parameters such as `vs_*`.
The first failure checks include:

- unknown trigger variables
- unknown target variables
- invalid operators
- trigger times beyond `mission_timer`
- trigger depths deeper than the planned mission maximum depth

The implementation is split into:

- `mission_model.py` - mission data model, parser, writer, and templates
- `mission_constructor_gui.py` - PyQt5 user interface and ksim runner
- `launch_mission_constructor.py` - small launcher

### `mission_constructor.html` (legacy)

Standalone browser mission builder for composing simulator `.mi` files from
the current behavior catalog.

Open this file directly in a browser:

```text
SUtilities\mission_constructor.html
```

The constructor includes:

- a behavior catalog built from the current generated `behaviors` table
- editable behavior arguments and mission defaults
- degree / degree-rate editor inputs for angular fields, with mission output
  converted back to radians
- import of existing `.mi` files by file picker or drag/drop
- direct script edits that can be applied back into the behavior editor
- click/select behavior blocks in the script to edit their arguments
- priority controls using simulator terminology: `P1` is highest priority
- highlighting and warning messages for header/use entries, sensors,
  behaviors, and behavior arguments that are not in the current simulator
- a simulator-variable picker for adding initial `sensor:` values from the
  generated vehicle data structure
- copy/download of the current mission script
- live plan-view and vertical-view previews

The previews are planning aids, not a hydrodynamic simulation. They use simple
dead-reckoned estimates from behavior arguments.

### `mission_constructor_app.py` (legacy)

Python wrapper that launches `mission_constructor.html` from a local web server and
invokes `ksim` on demand. It generates mission files and behavior logs under
`SUtilities/ksim_runs/`, then renders simulation results in the app.

Run from the repository root:

```powershell
python SUtilities\mission_constructor_app.py
```

Then open:

```text
http://127.0.0.1:8000/
```

Use the `Run Ksim` button to validate the current mission and view the
behavior timeline in the `Results` tab.

### `extract_mission.py`

Extracts the mission script from every `NAME.slg` file in a folder and writes
`NAME.mi`.

```powershell
python SUtilities\extract_mission.py SData
```

Useful options:

```powershell
python SUtilities\extract_mission.py SData --dry-run
python SUtilities\extract_mission.py SData --output-dir SMission
python SUtilities\extract_mission.py SData --recursive
python SUtilities\extract_mission.py SData --no-overwrite
```

The extractor looks for the mission block after the `mission file:` line and
stops before the vehicle/logging startup output.

### `missing_mi_variables.py`

Scans all `NAME.mi` files in a folder and writes a CSV table showing mission
entries that are not supported by the current simulator.

```powershell
python SUtilities\missing_mi_variables.py SMission
```

Default output:

```text
SMission\missing_mi_variables.csv
```

The report includes:

- missing `sensor:` variables
- missing `behavior:` names
- missing `b_arg:` names for known behaviors

Useful options:

```powershell
python SUtilities\missing_mi_variables.py SMission --output -
python SUtilities\missing_mi_variables.py SMission --output SUtilities\mi_check.csv
python SUtilities\missing_mi_variables.py SMission --recursive
```

By default the script uses:

```text
vehicle.h
behaviors
```

Alternative metadata files can be supplied:

```powershell
python SUtilities\missing_mi_variables.py SMission --vehicle-h path\to\vehicle.h --behaviors path\to\behaviors
```

### `missing_sim_variables.py`

Scans all `NAME.obd` files in a folder and writes a CSV table showing field
data variables that are present in the `.obd` header but missing from the
current simulator `vehicle.h`.

```powershell
python SUtilities\missing_sim_variables.py SData
```

Default output:

```text
SData\missing_sim_variables.csv
```

The script reads only the ASCII header at the front of each `.obd` file. It
ignores behavior log entries when they match known simulator behaviors and
behavior arguments.

Useful options:

```powershell
python SUtilities\missing_sim_variables.py SData --output -
python SUtilities\missing_sim_variables.py SData --output SUtilities\obd_check.csv
python SUtilities\missing_sim_variables.py SData --recursive
```

The `header_terminated` CSV column is `1` when the normal `9999` end-of-header
marker was found. A `0` usually means the file is truncated, malformed, or not
an Odyssey `.obd` file in the expected format.

### `trim_mi.py`

Creates `NAME_trim.mi` files with unsupported simulator entries removed.

```powershell
python SUtilities\trim_mi.py SMission
```

By default, the trimmer removes:

- unsupported `sensor:` lines
- unsupported `behavior:` blocks
- unsupported `b_arg:` lines for known behaviors

Useful options:

```powershell
python SUtilities\trim_mi.py SMission --dry-run
python SUtilities\trim_mi.py SMission --no-overwrite
python SUtilities\trim_mi.py SMission --recursive
python SUtilities\trim_mi.py SMission --keep-missing-behaviors
python SUtilities\trim_mi.py SMission --keep-missing-b-args
```

The original `.mi` files are left unchanged.

## Suggested Workflow

1. Extract missions from field system logs:

```powershell
python SUtilities\extract_mission.py "path\to\field_run_folder"
```

2. Check extracted missions against the simulator:

```powershell
python SUtilities\missing_mi_variables.py "path\to\field_run_folder"
```

3. Check field `.obd` variables against the simulator:

```powershell
python SUtilities\missing_sim_variables.py "path\to\field_run_folder"
```

4. Generate simulator-loadable trimmed missions:

```powershell
python SUtilities\trim_mi.py "path\to\field_run_folder"
```

5. Review the generated CSV files before deciding whether missing quantities
   should be added to `edit_struct.c` and regenerated into `vehicle.h` /
   `vehicle.c`.

## CSV Conventions

The missing-variable reports use one row per input file. Missing names inside a
cell are separated with semicolons.

This keeps the CSV compact while still allowing spreadsheet filtering by file.
For script processing, split fields such as `missing_sensors` or
`missing_variables` on `;`.

## Adding New Utilities

For future scripts, prefer this pattern:

- Put command-line scripts in `SUtilities`.
- Keep reusable parsing logic in `auv_mission_tools.py` or another small shared
  module.
- Accept the folder to process as the first positional argument.
- Add `--dry-run` for utilities that write or modify files.
- Add `--output` for utilities that generate reports.
- Avoid changing simulator source or generated vehicle files from these
  utilities.
- Document the utility in this README when it is added.

When a utility needs simulator data-structure information, read from generated
files such as `vehicle.h` and `behaviors`. Do not infer simulator variables
from unrelated source files.
