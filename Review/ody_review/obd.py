"""
Parse Odyssey Binary Data (.obd) files into xarray Datasets.

Port of ody_parse.m / parse.c, with the same self-documenting binary format:

  [ASCII HEADER]
    Line 1:  MIT_SG_AUV_LAB_v<n> <vehicle>
    Line 2:  <datetime>  # OS-9 file creation time
    Line 3:  <datetime>  # Structure creation time
    Lines :  <idx> <sensor_name(units)> [initial_value]   one per sensor
    Last  :  9999 <mission_start_time> (<date>)
  [BINARY SECTION]
    Flat stream of 6-byte pairs: int16 index, float32 value (little-endian).
    index == 10001 marks the start of a new control cycle; the float that
    follows is the previous cycle's elapsed-time-in-loop (logger debug).
    All other (index, value) pairs are sensor updates within the cycle.

Because sensor names are embedded in the file, parsing is forward- and
backward-compatible across changes to vehicle.h.
"""
from __future__ import annotations

import re
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import xarray as xr

CYCLE_FLAG = np.int16(10001)
END_OF_HEADER = 9999


@dataclass
class _SensorDef:
    index: int
    name: str
    units: str | None
    init: float


def _parse_header(raw: bytes) -> tuple[list[_SensorDef], int, dict]:
    """Walk the ASCII header line by line until the '9999' marker.

    Returns the sensor table, the byte offset at which the binary section
    starts, and a metadata dict (vehicle name, creation times, mission start).
    """
    sensors: list[_SensorDef] = []
    meta: dict = {}
    binary_offset = -1

    # Cap header search — typical headers are < 8 KB; the C side
    # set_filesize() may pre-allocate a large file padded with NULs, so we
    # don't want to scan all of it as text.
    search_end = min(len(raw), 200_000)
    cursor = 0

    while cursor < search_end:
        nl = raw.find(b"\n", cursor)
        if nl < 0:
            break
        line = raw[cursor:nl].rstrip(b"\r").decode("latin-1", errors="replace").strip()
        next_cursor = nl + 1
        cursor = next_cursor

        if not line:
            continue
        if line.startswith("MIT_SG"):
            parts = line.split(maxsplit=1)
            meta["format"] = parts[0]
            if len(parts) > 1:
                meta["vehicle"] = parts[1].strip()
            continue
        if "#" in line and not line.lstrip().startswith(tuple("0123456789")):
            # Date/comment line: "1998/02/03 04:25:07 GMT     # OS-9 file ..."
            meta.setdefault("header_lines", []).append(line)
            # Extract fault file name written by write_fault_header() in log_data.c:
            #   "# fault file: cg_shift_demo.fd"
            if "fault file:" in line:
                m = re.search(r"fault file:\s*(\S+)", line)
                if m:
                    meta["fault_file"] = m.group(1)
            if "mission file:" in line:
                m = re.search(r"mission file:\s*(\S+)", line)
                if m:
                    meta["mission_file"] = m.group(1)
            if line.startswith("# mi>"):
                meta.setdefault("mission_lines", []).append(line[5:].lstrip())
            if line.startswith("# fd>"):
                meta.setdefault("fault_lines", []).append(line[5:].lstrip())
            continue
        if line.startswith(("*", "%")):
            continue

        # Numeric-led line: sensor entry or 9999 terminator
        tok, *rest = line.split(maxsplit=1)
        try:
            idx = int(tok)
        except ValueError:
            continue
        rest_str = rest[0] if rest else ""

        if idx == END_OF_HEADER:
            # "9999 <mission_start> (<date>)"
            m = re.match(r"\s*([\d.+-eE]+)\s*(?:\((.*)\))?", rest_str)
            if m:
                try:
                    meta["mission_start_time"] = float(m.group(1))
                except ValueError:
                    pass
                if m.group(2):
                    meta["mission_date"] = m.group(2).strip()
            binary_offset = next_cursor
            break

        # Sensor line: "5 m_depth(m) [init_val]"
        # Robustly split off "name(units)" then trailing initial value.
        parts = rest_str.split()
        if not parts:
            continue
        name_units = parts[0]
        units = None
        if "(" in name_units and ")" in name_units:
            name = name_units.split("(", 1)[0]
            units = name_units.split("(", 1)[1].rstrip(")")
        else:
            name = name_units
        name = name.lower().strip()
        if not name:
            continue

        init = np.nan
        if len(parts) >= 2:
            try:
                init = float(parts[1])
            except ValueError:
                pass

        sensors.append(_SensorDef(index=idx, name=name, units=units, init=init))

    if binary_offset < 0:
        # Fallback byte search — handles the set_filesize() NUL-pad case
        m = re.search(rb"9999 +([\d.+-eE]+)", raw[:search_end])
        if m is None:
            raise ValueError("No '9999' mission-start marker found in header.")
        try:
            meta["mission_start_time"] = float(m.group(1))
        except ValueError:
            pass
        nl = raw.find(b"\n", m.end())
        if nl < 0:
            raise ValueError("Found '9999' but no terminating newline after it.")
        binary_offset = nl + 1

    if "mission_lines" in meta:
        meta["mission_text"] = "\n".join(meta["mission_lines"]).rstrip() + "\n"
    if "fault_lines" in meta:
        meta["fault_text"] = "\n".join(meta["fault_lines"]).rstrip() + "\n"

    return sensors, binary_offset, meta


def _decode_binary(buf: bytes) -> tuple[np.ndarray, np.ndarray]:
    """Vectorised decode of the binary section into (indices, values).

    Auto-detects byte order. Modern x86 simulator builds write little-endian.
    The original OS-9 vehicles (PowerPC/68k) wrote big-endian. We try LE
    first and fall back to BE if no cycle marker is found at offset 0.
    """
    n_pairs = len(buf) // 6
    if n_pairs == 0:
        raise ValueError("Binary section is empty.")
    buf = buf[: n_pairs * 6]
    arr = np.frombuffer(buf, dtype=np.uint8).reshape(n_pairs, 6)

    # Try little-endian first
    indices_le = arr[:, 0:2].copy().view(np.int16).reshape(n_pairs)
    if indices_le[0] == CYCLE_FLAG:
        values = arr[:, 2:6].copy().view(np.float32).reshape(n_pairs)
        return indices_le, values

    # Fall back to big-endian (OS-9 vehicle origin)
    indices_be = arr[:, 0:2].copy().view(">i2").reshape(n_pairs)
    if indices_be[0] == CYCLE_FLAG:
        values = arr[:, 2:6].copy().view(">f4").reshape(n_pairs)
        return indices_be, values

    # Neither byte order produces a cycle marker at offset 0; surface the
    # most informative error to help diagnose unknown formats.
    raise ValueError(
        f"No cycle marker (10001) at start of binary section. "
        f"First 6 bytes: {arr[0].tobytes().hex()}; "
        f"LE int16={int(indices_le[0])}, BE int16={int(indices_be[0])}"
    )


def _reconstruct_table(
    indices: np.ndarray,
    values: np.ndarray,
    sensors: list[_SensorDef],
) -> tuple[np.ndarray, list[_SensorDef]]:
    """Rebuild the per-cycle (n_cycles, n_sensors) table.

    Sensor values are sticky: a column carries the last logged value forward
    until a new (index, value) pair arrives — matching how `sensor()` reads
    work in the C runtime.
    """
    cycle_pos = np.flatnonzero(indices == CYCLE_FLAG)
    if cycle_pos.size == 0:
        raise ValueError("No 10001 cycle markers found in binary section.")

    # Column layout: dense over the sensors we know about.
    sensors_sorted = sorted(sensors, key=lambda s: s.index)
    idx_to_col = {s.index: c for c, s in enumerate(sensors_sorted)}
    n_cols = len(sensors_sorted)
    n_cycles = cycle_pos.size

    row = np.full(n_cols, np.nan, dtype=np.float64)
    for s in sensors_sorted:
        if not np.isnan(s.init):
            row[idx_to_col[s.index]] = s.init

    table = np.full((n_cycles, n_cols), np.nan, dtype=np.float64)
    cycle_pos_aug = np.append(cycle_pos, indices.size)

    for c in range(n_cycles):
        start = cycle_pos[c] + 1
        end = cycle_pos_aug[c + 1]
        if end > start:
            seg_idx = indices[start:end]
            seg_val = values[start:end]
            # Drop any unknown indices (e.g. behaviour-arg logging from
            # JOHN_VERBOSE builds), then write the rest in order.
            for ii, vv in zip(seg_idx, seg_val):
                col = idx_to_col.get(int(ii))
                if col is not None:
                    row[col] = float(vv)
        table[c] = row

    return table, sensors_sorted


def read_obd(path: str | Path) -> xr.Dataset:
    """Parse an .obd file and return an xarray Dataset.

    The Dataset has one variable per logged sensor (e.g. ``ds["m_depth"]``),
    indexed by a single ``cycle`` coordinate. Mission elapsed time, if logged
    as ``m_present_time``, is promoted to a coordinate ``time`` (seconds
    relative to first cycle) — matching what fix_time.m did.
    """
    path = Path(path)
    raw = path.read_bytes()

    sensors, binary_offset, meta = _parse_header(raw)
    if not sensors:
        raise ValueError(f"No sensor definitions found in {path}.")

    indices, values = _decode_binary(raw[binary_offset:])
    table, sensors_sorted = _reconstruct_table(indices, values, sensors)

    n_cycles = table.shape[0]
    coords: dict = {"cycle": np.arange(n_cycles)}

    data_vars = {}
    for col, s in enumerate(sensors_sorted):
        attrs = {"sensor_index": s.index}
        if s.units:
            attrs["units"] = s.units
        if not np.isnan(s.init):
            attrs["initial_value"] = s.init
        data_vars[s.name] = xr.DataArray(table[:, col], dims=("cycle",), attrs=attrs)

    ds = xr.Dataset(data_vars=data_vars, coords=coords, attrs=meta)
    ds.attrs["source_file"] = str(path)
    ds.attrs["n_cycles"] = n_cycles
    ds.attrs["n_sensors"] = len(sensors_sorted)

    # Promote mission-elapsed time to a real coordinate
    if "m_present_time" in ds:
        t = ds["m_present_time"].values
        finite = np.isfinite(t)
        if finite.any():
            t0 = t[finite][0]
            ds = ds.assign_coords(time=("cycle", t - t0))
            ds["time"].attrs["units"] = "s"
            ds["time"].attrs["description"] = "mission elapsed time"

    return ds
