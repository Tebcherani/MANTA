# ody_review — Python framework for reviewing Odyssey `.obd` files

A direct port of the MIT Sea Grant AUV Lab MATLAB review scripts (`review.m`
and friends) into a small, dependency-light Python package. Reads the
self-documenting binary format produced by `log_data.c`, applies the same
data conditioning steps the MATLAB pipeline did, and produces the same
plots — but as `xarray.Dataset` + `matplotlib` instead of the loose
`data(:, m_depth)` matrix idiom.

## Why a rewrite

The MATLAB code blurs three concerns that are worth separating:

1. **Parsing the `.obd` binary** (was `parse.c` / `ody_parse.m`)
2. **Conditioning the raw signals** (was `fix_time.m`, `fix_depth.m`, `fix_ctd.m`)
3. **Plotting** (was `vplane.m`, `tscast.m`, `CascadeCTD.m`, etc.)

The Python version splits them into three modules so each can be reused or
swapped independently — for example, you can parse a file in a notebook and
build your own custom plot without invoking the full review pipeline.

## Install

```bash
pip install numpy xarray matplotlib
pip install gsw          # optional, for TEOS-10 seawater calculations
```

`gsw` is the modern replacement for the CSIRO seawater toolbox the originals
used. If it is not installed, `ody_review.seawater` falls back to a
self-contained UNESCO 1980 / Chen-Millero implementation that agrees to within
sensor noise for normal AUV operating ranges.

## Quick start

```python
import ody_review as od

ds = od.read_obd("Data/X2609501.obd")     # → xarray.Dataset
ds = od.condition(ds)                      # adds salinity, d_rate, yoyo, ...

# Single function, like review.m:
od.review("Data/X2609501.obd", opts=[2, 3, 8, 9])

# Or build your own plots from the Dataset:
import matplotlib.pyplot as plt
plt.plot(ds["time"], ds["m_depth"])
```

From the command line:

```bash
python -m ody_review.review Data/X2609501.obd --opts 2,3,8,9
python -m ody_review.review Data/X2609501.obd --opts all --save report/
```

## How the MATLAB code maps to Python

| MATLAB                     | Python                                                |
|----------------------------|-------------------------------------------------------|
| `parse.c` / `ody_parse.m`  | `ody_review.obd.read_obd(path)` → `xarray.Dataset`    |
| `fix_time.m`               | `conditioning.add_time(ds)` (and auto-applied)        |
| `fix_depth.m`              | `conditioning.add_depth_processing(ds)`               |
| `fix_ctd.m`                | `conditioning.add_ctd_processing(ds)`                 |
| `salsp.m` / `sw_salt`      | `seawater.salinity_and_sound_velocity(c, t, p)`       |
| `sw_dens`                  | `seawater.density(s, t, p)`                           |
| `review.m`                 | `review.review(path, opts=[...])`                     |
| `vplane.m`                 | `plots.vertical_plane(ds)`                            |
| `navplot.m`                | `plots.navplot(ds)`                                   |
| `CTD_process.m`            | `plots.ctd_time_series(ds)` + `plots.ctd_profiles(ds)`|
| `tscast.m`                 | `plots.ts_cast(ds)`                                   |
| `CascadeCTD.m`             | `plots.cascade_ctd(ds)`                               |
| `optics_cast.m`            | `plots.optics_cast(ds)`                               |
| `plt_depth_rate.m`         | `plots.depth_rate_check(ds)`                          |
| `data(:, m_depth)`         | `ds["m_depth"]`                                       |
| `going_up`, `going_down`   | `ds["going_up"]`, `ds["going_down"]` (boolean arrays) |

## Domain knowledge preserved verbatim

These quirks are easy to miss on a clean-room rewrite. The Python ports
keep them:

- **Parosci 10-cycle artifact removal** (`fix_depth.m`). Every 10th depth
  sample is a temperature update of the pressure transducer rather than a
  real depth measurement. Detected by a near-zero offset in `mod(cyc, 10)`
  and turned into NaNs.
- **CTD pump lag** (`CTD_process.m`). Conductivity lags temperature by
  ~5 control cycles through the pumped CTD. Aligned by a circular shift
  before computing salinity.
- **Sensor offset for tilt** (`CTD_process.m`). The CTD cell is `dx=0.8 m`
  forward and `dy=0.25 m` below the pressure transducer; effective CTD
  depth is corrected for vehicle pitch.
- **Yoyo classification at ±0.1 m/s** (`fix_depth.m`) with a >100-cycle
  minimum dive length to suppress noise crossings.
- **Freshwater m → decibars conversion** (JWB 5/2/95 in `CTD_process.m`):
  `dbar = m × 3.281 × 12 × 2.49082e-2`. Kept for backward compatibility,
  though `gsw.p_from_z` is more accurate if available.

## File format reference

Documented in `obd.py` and unchanged from `log_data.c`:

```
[ASCII HEADER]
  MIT_SG_AUV_LAB_v<n> <vehicle>
  <datetime>      # OS-9 file creation time
  <datetime>      # Structure creation time
  <idx> <name>(<units>) [initial_value]   one line per sensor
  9999 <mission_start_time> (<date>)
[BINARY SECTION]
  Flat little-endian stream of (int16 idx, float32 value) pairs.
  idx == 10001 marks the start of a new control cycle; the float
  that follows is debug-only (last cycle's loop time in ms).
  All other (idx, value) pairs are sensor updates within the cycle.
```

## Testing without a real `.obd`

`examples/make_synthetic_obd.py` writes a synthetic file matching the
exact byte layout `sim.exe` produces. Useful for CI:

```python
from examples.make_synthetic_obd import write_synthetic_obd
write_synthetic_obd("test.obd", n_cycles=1500)
import ody_review as od
od.review("test.obd", opts=[2, 9])
```

## Where to take this next

A few directions, in roughly increasing scope:

- **Interactive plots.** Swap `matplotlib` for `plotly` in `plots.py` and
  the same DataSet pipeline produces a self-contained HTML review report.
- **Streamlit / Dash dashboard.** `condition()` + cached `read_obd()` makes
  for a fast in-browser explorer.
- **Multi-file aggregation.** `xr.concat([read_obd(f) for f in files], dim="run")`
  works as long as the sensor sets overlap — useful for mission-series
  comparison plots that the MATLAB code couldn't really do.
- **`netCDF` export.** `ds.to_netcdf("X2609501.nc")` makes the data
  portable to any analysis tool, including back to MATLAB if needed.
- **Behaviour-argument logging.** The MATLAB never plotted the
  `JOHN_VERBOSE` behaviour arguments. The Python parser silently drops
  them today; adding a separate parser path for them would let you
  visualise behaviour arbitration alongside sensor data.
```
