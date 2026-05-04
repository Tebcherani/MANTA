"""
Inspect an .obd file: list every sensor with its initial value, write count,
min, and max.

Walks the binary section directly counting actual write events — not the
sticky values seen at each cycle. This means a sensor logged once at
init that never changes shows write_count=1, not write_count=N_cycles.

Usage:
    python -m ody_review.inspect FILE.obd
    python -m ody_review.inspect FILE.obd --csv stats.csv
    python -m ody_review.inspect FILE.obd --sort writes  # writes|name|min|max|index
    python -m ody_review.inspect FILE.obd --filter m_     # only sensors starting with m_
"""
from __future__ import annotations

import argparse
import csv
import sys
from pathlib import Path
from typing import Iterable

import numpy as np

from .obd import _parse_header, _decode_binary, CYCLE_FLAG


def inspect(path: str | Path) -> tuple[list[dict], dict]:
    """Return (rows, meta) where rows is a list of per-sensor dicts and
    meta is the file-level info (vehicle, mission start, n_cycles, ...).

    Each row dict has: index, name, units, initial, writes, min, max, last.
    """
    path = Path(path)
    raw = path.read_bytes()

    sensors, binary_offset, meta = _parse_header(raw)
    if not sensors:
        raise ValueError(f"No sensor definitions found in {path}.")

    indices, values = _decode_binary(raw[binary_offset:])
    n_cycles = int((indices == CYCLE_FLAG).sum())

    # Build per-sensor stats by walking the (idx, value) stream once.
    # Skip the cycle marker rows and any indices not in the sensor table.
    known = {s.index for s in sensors}
    name_by_idx = {s.index: s.name for s in sensors}
    units_by_idx = {s.index: s.units for s in sensors}
    init_by_idx = {s.index: s.init for s in sensors}

    # Vectorised: mask out cycle markers and unknown indices, then groupby.
    valid = (indices != CYCLE_FLAG)
    if valid.any():
        idx_v = indices[valid]
        val_v = values[valid]
        # numpy lacks groupby; use a dict-keyed accumulator. Cheap enough
        # for typical .obd sizes (millions of pairs is fractions of a
        # second on a modern laptop).
        stats: dict[int, dict] = {}
        for ii, vv in zip(idx_v.tolist(), val_v.tolist()):
            if ii not in known:
                continue
            d = stats.get(ii)
            if d is None:
                stats[ii] = {
                    "writes": 1,
                    "min": vv,
                    "max": vv,
                    "first_written": vv,
                    "last": vv,
                }
            else:
                d["writes"] += 1
                if vv < d["min"]:
                    d["min"] = vv
                if vv > d["max"]:
                    d["max"] = vv
                d["last"] = vv
    else:
        stats = {}

    rows: list[dict] = []
    for s in sensors:
        st = stats.get(s.index)
        rows.append({
            "index": s.index,
            "name": s.name,
            "units": s.units or "",
            "initial": init_by_idx[s.index],
            "writes": st["writes"] if st else 0,
            "min": st["min"] if st else float("nan"),
            "max": st["max"] if st else float("nan"),
            "last": st["last"] if st else float("nan"),
        })

    meta = dict(meta)
    meta["n_cycles"] = n_cycles
    meta["n_sensors"] = len(sensors)
    meta["source_file"] = str(path)
    return rows, meta


# --- formatting helpers ----------------------------------------------------

def _fmt_num(x: float, width: int = 12) -> str:
    if x is None or (isinstance(x, float) and (x != x)):  # NaN
        return f"{'--':>{width}}"
    ax = abs(x)
    if ax == 0:
        return f"{0.0:>{width}.4g}"
    if ax < 0.001 or ax >= 1e6:
        return f"{x:>{width}.4e}"
    return f"{x:>{width}.4g}"


def print_table(rows: list[dict], meta: dict, max_name_width: int = 28) -> None:
    """Aligned text table to stdout."""
    # Header / metadata block
    print(f"File:          {meta.get('source_file')}")
    if meta.get("vehicle"):
        print(f"Vehicle:       {meta['vehicle']}")
    if meta.get("format"):
        print(f"Format:        {meta['format']}")
    if meta.get("mission_date"):
        print(f"Mission date:  {meta['mission_date']}")
    print(f"Cycles:        {meta.get('n_cycles', 0):,}")
    print(f"Sensors:       {meta.get('n_sensors', 0)}")
    print()

    # Compute name column width from data
    name_w = max(4, min(max_name_width,
                         max(len(r["name"]) for r in rows) if rows else 4))
    units_w = max(5, min(12,
                          max(len(r["units"]) for r in rows) if rows else 5))

    header = (
        f"{'idx':>4}  {'name':<{name_w}}  {'units':<{units_w}}  "
        f"{'initial':>12}  {'writes':>8}  {'min':>12}  {'max':>12}  {'last':>12}"
    )
    print(header)
    print("-" * len(header))

    for r in rows:
        name = r["name"][:name_w]
        units = r["units"][:units_w]
        print(
            f"{r['index']:>4}  {name:<{name_w}}  {units:<{units_w}}  "
            f"{_fmt_num(r['initial'])}  {r['writes']:>8,}  "
            f"{_fmt_num(r['min'])}  {_fmt_num(r['max'])}  {_fmt_num(r['last'])}"
        )


def write_csv(rows: list[dict], path: str | Path) -> None:
    path = Path(path)
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=[
            "index", "name", "units", "initial", "writes", "min", "max", "last",
        ])
        w.writeheader()
        for r in rows:
            w.writerow(r)


def _sort_rows(rows: list[dict], key: str) -> list[dict]:
    if key == "index":
        return sorted(rows, key=lambda r: r["index"])
    if key == "name":
        return sorted(rows, key=lambda r: r["name"])
    if key == "writes":
        return sorted(rows, key=lambda r: r["writes"], reverse=True)
    if key == "min":
        return sorted(rows, key=lambda r: (r["min"] != r["min"], r["min"]))
    if key == "max":
        return sorted(rows, key=lambda r: (r["max"] != r["max"], -r["max"]))
    return rows


def _filter_rows(rows: list[dict], substr: str) -> list[dict]:
    s = substr.lower()
    return [r for r in rows if s in r["name"].lower()]


def main(argv: list[str] | None = None) -> None:
    p = argparse.ArgumentParser(
        description="Inspect an .obd file: list sensors with stats."
    )
    p.add_argument("path", help="path to .obd file")
    p.add_argument("--csv", help="also write the table to this CSV file")
    p.add_argument("--sort", default="index",
                   choices=["index", "name", "writes", "min", "max"],
                   help="column to sort by (default: index)")
    p.add_argument("--filter", default="",
                   help="show only sensors whose name contains this substring")
    p.add_argument("--only-written", action="store_true",
                   help="hide sensors with zero writes")
    args = p.parse_args(argv)

    rows, meta = inspect(args.path)
    if args.filter:
        rows = _filter_rows(rows, args.filter)
    if args.only_written:
        rows = [r for r in rows if r["writes"] > 0]
    rows = _sort_rows(rows, args.sort)

    print_table(rows, meta)
    if args.csv:
        write_csv(rows, args.csv)
        print(f"\nCSV written to {args.csv}")


if __name__ == "__main__":
    main()
