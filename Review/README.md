# ody_review - Odyssey `.obd` Review GUI

Tabbed PyQt5 interface for reviewing Odyssey simulator and vehicle `.obd`
logs. It reads the self-documenting binary format produced by `log_data.c`,
conditions the raw signals, and displays selected plot panels in tabs.

## Setup

Run once:

```bat
Review\setup.bat
```

This creates `Review\.venv` and installs the GUI dependencies.

## Launch

From the repository root:

```bat
py Review\launch_review.py
```

Or from inside `Review`:

```bat
py launch_review.py
```

The GUI scans `SData` for `.obd` files newest-first. Select a file, click
`Load`, choose the plot panels you want, then click `Generate Plots`.

## Python API

The lower-level reader and conditioning functions are still available for
notebooks and custom analysis:

```python
import ody_review as od

ds = od.read_obd("../SData/S2615101.obd")
ds = od.condition(ds)
```

## File Format

Documented in `ody_review/obd.py` and unchanged from `log_data.c`:

```text
[ASCII HEADER]
  MIT_SG_AUV_LAB_v<n> <vehicle>
  <datetime>      # OS-9 file creation time
  <datetime>      # Structure creation time
  <idx> <name>(<units>) [initial_value]   one line per sensor
  9999 <mission_start_time> (<date>)
[BINARY SECTION]
  Flat little-endian stream of (int16 idx, float32 value) pairs.
  idx == 10001 marks the start of a new control cycle.
```

## Notes

The old command-line single-run review that opened separate matplotlib
windows has been removed. `launch_review.py` is the supported review entry
point.
