#!/usr/bin/env python3
"""
SEnvironment/scout.py  —  Chesapeake Bay environmental data scout.

Runs four tasks and writes results to SEnvironment/:

  Task 1  Discover all CO-OPS water-level stations in the Bay region
  Task 2  Discover all CO-OPS current-observation stations in the Bay region
  Task 3  Test CBOFS model data access (THREDDS catalog + OPeNDAP probe)
  Task 4  Download one month of 6-min water-level data from 4 key stations

All output files are written to the same directory as this script.

Usage (from Windows_Ody_C):
    python SEnvironment/scout.py [--month YYYY-MM]

Default sample month: October 2024.
"""

import argparse
import csv
import json
import sys
import time
import urllib.request
import urllib.error
from datetime import datetime, timedelta
from pathlib import Path

# Force UTF-8 on Windows where the default console encoding may be CP1252
if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")

HERE = Path(__file__).parent          # SEnvironment/
COOPS_API   = "https://api.tidesandcurrents.noaa.gov/api/prod/datagetter"
COOPS_MDAPI = "https://api.tidesandcurrents.noaa.gov/mdapi/prod/webapi"

# Chesapeake Bay bounding box (decimal degrees, including major tributaries)
BAY_LAT = (36.8, 39.6)
BAY_LON = (-77.5, -75.5)

# Four reference stations distributed along the Bay — used for Task 4
KEY_STATIONS = [
    {"id": "8638863", "name": "Chesapeake Bay Bridge Tunnel, VA",
     "lat": 36.9633, "lon": -76.1133, "note": "Bay mouth"},
    {"id": "8632200", "name": "Kiptopeke, VA",
     "lat": 37.1667, "lon": -75.9883, "note": "Lower Bay, Eastern Shore"},
    {"id": "8577330", "name": "Solomons Island, MD",
     "lat": 38.3183, "lon": -76.4517, "note": "Mid-Bay, Patuxent entrance"},
    {"id": "8574680", "name": "Baltimore, MD",
     "lat": 39.2667, "lon": -76.5783, "note": "Upper Bay"},
]

# CBOFS THREDDS / OPeNDAP candidate URLs to probe (Task 3)
CBOFS_CANDIDATES = [
    ("THREDDS catalog (CO-OPS)",
     "https://opendap.co-ops.nos.noaa.gov/thredds/catalog/CBOFS/catalog.xml"),
    ("THREDDS catalog (NCEI coastal)",
     "https://www.ncei.noaa.gov/thredds-coastal/catalog/chesapeake_bay_est_mod_r/catalog.xml"),
    ("ERDDAP dataset list (CO-OPS)",
     "https://opendap.co-ops.nos.noaa.gov/erddap/info/index.json?searchFor=cbofs"),
    ("ERDDAP dataset list (coastwatch)",
     "https://coastwatch.pfeg.noaa.gov/erddap/search/index.json?searchFor=cbofs+chesapeake"),
]


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _get(url: str, label: str = "", timeout: int = 20):
    """HTTP GET with basic error handling. Returns body bytes or None."""
    try:
        with urllib.request.urlopen(url, timeout=timeout) as r:
            body = r.read()
        if label:
            print(f"  ✓  {label}")
        return body
    except urllib.error.HTTPError as e:
        if label:
            print(f"  ✗  {label}  →  HTTP {e.code} {e.reason}")
    except urllib.error.URLError as e:
        if label:
            print(f"  ✗  {label}  →  {e.reason}")
    except Exception as e:
        if label:
            print(f"  ✗  {label}  →  {e}")
    return None


def _in_bay(lat: float, lon: float) -> bool:
    return BAY_LAT[0] <= lat <= BAY_LAT[1] and BAY_LON[0] <= lon <= BAY_LON[1]


def _write_csv(path: Path, rows: list[dict], fieldnames: list[str]) -> None:
    with open(path, "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=fieldnames, extrasaction="ignore")
        w.writeheader()
        w.writerows(rows)
    print(f"  → saved {path.name}  ({len(rows)} rows)")


# ---------------------------------------------------------------------------
# Task 1 & 2  —  Station discovery via MDAPI
# ---------------------------------------------------------------------------

def discover_stations(product_type: str) -> list[dict]:
    """
    Query the MDAPI for all stations of a given type, filter to the Bay region.

    product_type: 'waterlevels' or 'currents'
    """
    url = f"{COOPS_MDAPI}/stations.json?type={product_type}&units=metric"
    body = _get(url, label=f"MDAPI station list ({product_type})")
    if body is None:
        return []

    try:
        data = json.loads(body)
    except json.JSONDecodeError as e:
        print(f"  ✗  JSON parse error: {e}")
        return []

    stations = data.get("stations", [])
    bay = []
    for s in stations:
        try:
            lat = float(s.get("lat", 0))
            lon = float(s.get("lng", s.get("lon", 0)))
        except (TypeError, ValueError):
            continue
        if _in_bay(lat, lon):
            bay.append({
                "id":      s.get("id", ""),
                "name":    s.get("name", ""),
                "lat":     lat,
                "lon":     lon,
                "state":   s.get("state", ""),
                "timezone": s.get("timezone", ""),
                "type":    product_type,
            })

    print(f"  → {len(stations)} total {product_type} stations; "
          f"{len(bay)} inside Chesapeake Bay bounding box")
    return bay


def task1_water_level_stations() -> list[dict]:
    print("\n─── Task 1: Water-level stations ───────────────────────────────")
    stations = discover_stations("waterlevels")
    if stations:
        _write_csv(
            HERE / "stations_water_level.csv",
            sorted(stations, key=lambda s: s["lat"], reverse=True),
            ["id", "name", "lat", "lon", "state", "timezone"],
        )
    return stations


def task2_current_stations() -> list[dict]:
    print("\n─── Task 2: Current-observation stations ────────────────────────")
    stations = discover_stations("currents")
    if stations:
        _write_csv(
            HERE / "stations_currents.csv",
            sorted(stations, key=lambda s: s["lat"], reverse=True),
            ["id", "name", "lat", "lon", "state", "timezone"],
        )
    return stations


# ---------------------------------------------------------------------------
# Task 3  —  CBOFS access probe
# ---------------------------------------------------------------------------

def task3_cbofs_probe() -> None:
    print("\n─── Task 3: CBOFS model data access ─────────────────────────────")
    print("  Probing candidate THREDDS / ERDDAP endpoints …")

    results = []
    for label, url in CBOFS_CANDIDATES:
        print(f"\n  [{label}]")
        print(f"  URL: {url}")
        body = _get(url, label="HTTP GET", timeout=30)
        if body is None:
            results.append({"label": label, "url": url, "status": "FAILED", "bytes": 0})
            continue

        nb = len(body)
        snippet = body[:400].decode("utf-8", errors="replace").replace("\n", " ")
        print(f"  Size: {nb:,} bytes")
        print(f"  Content preview: {snippet[:200]} …")
        results.append({"label": label, "url": url, "status": "OK", "bytes": nb})
        time.sleep(0.5)

    # Summary
    print("\n  CBOFS probe summary:")
    for r in results:
        tick = "✓" if r["status"] == "OK" else "✗"
        print(f"  {tick}  {r['label']}")
        print(f"       {r['url']}")

    # Save summary
    summary_path = HERE / "cbofs_probe.json"
    with open(summary_path, "w") as f:
        json.dump(results, f, indent=2)
    print(f"\n  → saved {summary_path.name}")

    print("""
  Next steps for CBOFS:
    • If THREDDS responded: use the catalog XML to enumerate available
      NetCDF files and construct OPeNDAP URLs for xarray.open_dataset().
    • If ERDDAP responded: use the dataset IDs returned to query
      /erddap/griddap/<id>.csv or .nc with time/lat/lon/depth subsets.
    • Install netCDF4 + xarray in this Python env if not present:
        pip install xarray netCDF4
""")


# ---------------------------------------------------------------------------
# Task 4  —  Sample water-level download
# ---------------------------------------------------------------------------

def _coops_water_level(station_id: str, begin: str, end: str) -> list[dict]:
    """
    Download 6-minute verified water level data for one station, one month.

    begin / end: 'YYYYMMDD'
    Returns list of dicts {t, v, s, f, q} or [] on error.
    """
    params = (
        f"?begin_date={begin}&end_date={end}"
        f"&station={station_id}"
        f"&product=water_level"
        f"&datum=MLLW"
        f"&time_zone=gmt"
        f"&units=metric"
        f"&format=csv"
        f"&application=MIT_SeaGrant_ChesapeakeSim"
    )
    url = COOPS_API + params
    body = _get(url, label=f"station {station_id}", timeout=40)
    if body is None:
        return []

    text = body.decode("utf-8", errors="replace")
    lines = text.splitlines()

    # CO-OPS CSV: first line is header, data starts line 2
    # Error responses begin with "Error" or contain no comma on line 1
    if not lines or "," not in lines[0]:
        msg = lines[0][:120] if lines else "(empty response)"
        print(f"    ✗  API error: {msg}")
        return []

    rows = []
    reader = csv.DictReader(lines)
    for row in reader:
        rows.append({k.strip(): v.strip() for k, v in row.items()
                     if k is not None and v is not None})
    return rows


def task4_sample_download(sample_month: str) -> None:
    """
    Download one month of 6-minute water level data from KEY_STATIONS.

    sample_month: 'YYYY-MM'
    """
    print(f"\n─── Task 4: Sample water-level download ({sample_month}) ───────")

    # Build begin/end dates spanning the full calendar month
    year, mon = int(sample_month[:4]), int(sample_month[5:7])
    begin_dt = datetime(year, mon, 1)
    if mon == 12:
        end_dt = datetime(year + 1, 1, 1) - timedelta(days=1)
    else:
        end_dt = datetime(year, mon + 1, 1) - timedelta(days=1)
    begin = begin_dt.strftime("%Y%m%d")
    end   = end_dt.strftime("%Y%m%d")
    print(f"  Period: {begin} → {end}  ({(end_dt - begin_dt).days + 1} days)")

    summary_rows = []

    for st in KEY_STATIONS:
        sid  = st["id"]
        name = st["name"]
        note = st["note"]
        print(f"\n  Station {sid} — {name}  ({note})")

        rows = _coops_water_level(sid, begin, end)
        if not rows:
            summary_rows.append({"id": sid, "name": name, "note": note,
                                  "records": 0, "status": "FAILED"})
            continue

        # Save raw CSV
        out_path = HERE / f"wl_{sid}_{sample_month.replace('-','')}.csv"
        with open(out_path, "w", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=rows[0].keys())
            writer.writeheader()
            writer.writerows(rows)

        # Quick stats
        values = []
        for r in rows:
            # CO-OPS column is ' Water Level' (with leading space)
            val_str = r.get(" Water Level", r.get("Water Level", "")).strip()
            try:
                values.append(float(val_str))
            except ValueError:
                pass

        n_total    = len(rows)
        n_valid    = len(values)
        n_missing  = n_total - n_valid
        pct_ok     = 100 * n_valid / n_total if n_total else 0
        wl_min     = min(values) if values else float("nan")
        wl_max     = max(values) if values else float("nan")
        wl_mean    = sum(values) / len(values) if values else float("nan")
        tidal_range = wl_max - wl_min if values else float("nan")

        print(f"    Records: {n_total:,}  |  valid: {n_valid:,}  "
              f"({pct_ok:.1f}%)  |  missing: {n_missing}")
        print(f"    Water level (m MLLW):  "
              f"min={wl_min:.3f}  max={wl_max:.3f}  "
              f"mean={wl_mean:.3f}  range={tidal_range:.3f}")
        print(f"    → saved {out_path.name}  "
              f"({out_path.stat().st_size / 1024:.1f} KB)")

        summary_rows.append({
            "id": sid, "name": name, "note": note,
            "records": n_total, "valid": n_valid,
            "pct_ok": round(pct_ok, 1),
            "wl_min_m": round(wl_min, 3), "wl_max_m": round(wl_max, 3),
            "wl_mean_m": round(wl_mean, 3),
            "tidal_range_m": round(tidal_range, 3),
            "status": "OK",
        })

        time.sleep(1)   # be polite to the API

    # Summary table
    _write_csv(
        HERE / f"wl_summary_{sample_month.replace('-','')}.csv",
        summary_rows,
        ["id", "name", "note", "records", "valid", "pct_ok",
         "wl_min_m", "wl_max_m", "wl_mean_m", "tidal_range_m", "status"],
    )

    # Annual volume estimate
    print("\n  Annual data volume estimates (extrapolated from this sample):")
    for r in summary_rows:
        if r["status"] == "OK":
            annual = r["records"] * 12
            print(f"    {r['id']}  {r['name'][:35]:35s}  "
                  f"~{annual:,} records/year")
    print("  At ~35 bytes/row (CSV): one station-year ≈ "
          f"~{240 * 365 * 35 / 1e6:.1f} MB")
    print("  Four stations × one year ≈ "
          f"~{4 * 240 * 365 * 35 / 1e6:.0f} MB total (uncompressed CSV)")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--month", default="2024-10",
                        help="Sample month for Task 4  (YYYY-MM, default: 2024-10)")
    parser.add_argument("--tasks", default="1,2,3,4",
                        help="Comma-separated task numbers to run (default: all)")
    args = parser.parse_args()

    tasks = {int(t.strip()) for t in args.tasks.split(",")}

    print("=" * 65)
    print("  Chesapeake Bay Environmental Data Scout")
    print(f"  Bay bounding box: lat {BAY_LAT}, lon {BAY_LON}")
    print(f"  Output directory: {HERE}")
    print("=" * 65)

    if 1 in tasks:
        task1_water_level_stations()
    if 2 in tasks:
        task2_current_stations()
    if 3 in tasks:
        task3_cbofs_probe()
    if 4 in tasks:
        task4_sample_download(args.month)

    print("\n" + "=" * 65)
    print("  Scout complete.  Output files:")
    for f in sorted(HERE.glob("*.csv")) + sorted(HERE.glob("*.json")):
        print(f"    {f.name}")
    print("=" * 65)


if __name__ == "__main__":
    main()
