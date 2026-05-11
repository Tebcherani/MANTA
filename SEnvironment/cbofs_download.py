#!/usr/bin/env python3
"""
SEnvironment/cbofs_download.py  —  CBOFS stations data downloader.

Downloads all available stations.nowcast.nc files from the CO-OPS THREDDS
server and saves them to SEnvironment/cbofs_data/YYYY/MM/DD/.

Each file contains 207 Chesapeake Bay model output stations, 6-minute
intervals, 20 sigma depth layers, for a 6-hour nowcast window.
Four files per day (t00z, t06z, t12z, t18z) give seamless 24-h coverage.

Variables downloaded:
  zeta        [time][station]        free-surface / water level (m)
  u, v        [time][station][layer] horizontal currents (m/s)
  temp        [time][station][layer] potential temperature (deg C)
  ocean_time  [time]                 seconds since 2016-01-01 UTC
  lat_rho     [station]              station latitude
  lon_rho     [station]              station longitude
  h           [station]              station water depth (m)
  s_rho       [layer]                sigma coords (dimensionless, -1..0)
  Cs_r        [layer]                stretching function at RHO-points

Usage:
    python SEnvironment/cbofs_download.py [--data-dir Q:\\cbofs_data] [--dry-run] [--vars zeta,u,v]

Output:
    <data-dir>/YYYY/MM/DD/cbofs.tHHz.YYYYMMDD.stations.nowcast.csv.gz
    <data-dir>/station_coords.csv   (written once)
    <data-dir>/download_log.csv     (one row per file)
"""

import argparse
import csv
import re
import sys
import time
import urllib.request
import urllib.error
from datetime import datetime, timezone, timedelta
from pathlib import Path
from xml.etree import ElementTree as ET

if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")

# ── Constants ────────────────────────────────────────────────────────────────
THREDDS   = "https://opendap.co-ops.nos.noaa.gov/thredds"
MODELS    = f"{THREDDS}/catalog/NOAA/CBOFS/MODELS"
DODC      = f"{THREDDS}/dodsC"
FILESVR   = f"{THREDDS}/fileServer"
EPOCH     = datetime(2016, 1, 1, tzinfo=timezone.utc)
NS        = {"t": "http://www.unidata.ucar.edu/namespaces/thredds/InvCatalog/v1.0",
             "x": "http://www.w3.org/1999/xlink"}
HERE      = Path(__file__).parent

DEFAULT_VARS = ["ocean_time", "lat_rho", "lon_rho", "h", "s_rho", "Cs_r",
                "zeta", "u", "v", "temp"]


# ── HTTP helpers ─────────────────────────────────────────────────────────────

def _get(url, timeout=40, retries=3):
    for attempt in range(retries):
        try:
            with urllib.request.urlopen(url, timeout=timeout) as r:
                return r.read()
        except urllib.error.HTTPError as e:
            if e.code in (404, 400):
                return None
            if attempt < retries - 1:
                time.sleep(2 ** attempt)
        except Exception:
            if attempt < retries - 1:
                time.sleep(2 ** attempt)
    return None


# ── Catalog walk ─────────────────────────────────────────────────────────────

def list_catalog_refs(catalog_url):
    """Return list of (name, absolute_xml_url) from a THREDDS catalog."""
    body = _get(catalog_url)
    if not body:
        return []
    try:
        root = ET.fromstring(body)
    except ET.ParseError:
        return []
    results = []
    for ref in root.findall(".//t:catalogRef", NS):
        name = ref.get("name","") or ref.get("{http://www.w3.org/1999/xlink}title","")
        href = ref.get("{http://www.w3.org/1999/xlink}href","")
        if href.startswith("http"):
            abs_url = href
        elif href.startswith("/"):
            abs_url = f"https://opendap.co-ops.nos.noaa.gov{href}"
        else:
            base = catalog_url.rsplit("/", 1)[0]
            abs_url = f"{base}/{href}"
        results.append((name, abs_url))
    return results


def list_datasets(catalog_url):
    """Return list of (name, urlPath) for leaf datasets in a catalog."""
    body = _get(catalog_url)
    if not body:
        return []
    try:
        root = ET.fromstring(body)
    except ET.ParseError:
        return []
    return [(d.get("name",""), d.get("urlPath",""))
            for d in root.findall(".//t:dataset[@urlPath]", NS)]


def enumerate_available_days():
    """
    Walk MODELS → year → month → day catalogs.
    Returns list of (year, month, day) tuples for all days present.
    """
    days = []
    for yr_name, yr_url in list_catalog_refs(f"{MODELS}/catalog.xml"):
        for mo_name, mo_url in list_catalog_refs(yr_url):
            for dy_name, dy_url in list_catalog_refs(mo_url):
                try:
                    days.append((int(yr_name), int(mo_name), int(dy_name), dy_url))
                except ValueError:
                    pass
    return sorted(days)


# ── OPeNDAP ASCII data reader ─────────────────────────────────────────────────

def _parse_ascii_var(text, varname):
    """
    Parse one variable from an OPeNDAP ASCII response.
    Returns flat list of floats, or None on failure.

    Row labels in the data block use the format  [T], or [T][S],
    (index values in brackets, no variable-name prefix).
    Strip all [N] bracket groups before extracting numbers.
    """
    pattern = re.compile(
        rf"(?:^|\n){re.escape(varname)}\[[\d\]\[]*\]\n(.*?)(?=\n\n|\n[a-zA-Z_]|\Z)",
        re.DOTALL)
    m = pattern.search(text)
    if not m:
        return None
    raw = m.group(1)
    # Remove all row-label brackets like [0], [0][5], etc.
    cleaned = re.sub(r"\[\d+\],?\s*", "", raw)
    nums = re.findall(r"[-+]?(?:\d+\.?\d*|\.\d+)(?:[eE][-+]?\d+)?", cleaned)
    return [float(v) for v in nums]


def fetch_var_ascii(urlpath, varname):
    """Download one variable from a CBOFS stations file via OPeNDAP ASCII."""
    url = f"{DODC}/{urlpath}.ascii?{varname}"
    body = _get(url, timeout=60)
    if body is None:
        return None
    text = body.decode("utf-8", errors="replace")
    return _parse_ascii_var(text, varname)


def fetch_vars_ascii(urlpath, varnames):
    """Download multiple variables in one OPeNDAP ASCII request."""
    query = ",".join(varnames)
    url = f"{DODC}/{urlpath}.ascii?{query}"
    body = _get(url, timeout=120)
    if body is None:
        return None
    text = body.decode("utf-8", errors="replace")
    result = {}
    for v in varnames:
        parsed = _parse_ascii_var(text, v)
        if parsed is not None:
            result[v] = parsed
    return result if result else None


# ── Direct fileServer download (alternative to OPeNDAP) ─────────────────────

def download_nc_direct(urlpath, dest_path):
    """Download raw NetCDF binary via THREDDS fileServer."""
    url = f"{FILESVR}/{urlpath}"
    body = _get(url, timeout=180)
    if body is None:
        return False
    dest_path.parent.mkdir(parents=True, exist_ok=True)
    dest_path.write_bytes(body)
    return True


# ── Per-file processing ───────────────────────────────────────────────────────

def process_station_file(urlpath, dest_dir, dl_vars, dry_run=False):
    """
    Download and save one stations.nowcast file as a numpy .npz archive.

    Arrays saved:
      ocean_time  [n_time]               seconds since 2016-01-01 UTC
      zeta        [n_time, n_sta]         water surface elevation (m)
      u           [n_time, n_sta, n_lyr]  eastward current (m/s)
      v           [n_time, n_sta, n_lyr]  northward current (m/s)
      temp        [n_time, n_sta, n_lyr]  potential temperature (°C)
      lat_rho     [n_sta]
      lon_rho     [n_sta]
      h           [n_sta]                 station depth (m)
      s_rho       [n_lyr]                 sigma coords (−1 → 0)
      Cs_r        [n_lyr]                 stretching function
    """
    import numpy as np

    fname    = urlpath.split("/")[-1]
    out_path = dest_dir / (fname.replace(".nc", ".npz"))

    if out_path.exists():
        return {"file": fname, "status": "skipped", "records": 0}

    if dry_run:
        print(f"    [dry-run] would download {fname}")
        return {"file": fname, "status": "dry-run", "records": 0}

    print(f"    Fetching {fname} ...", end=" ", flush=True)

    data = fetch_vars_ascii(urlpath, dl_vars)
    if data is None:
        print("FAIL")
        return {"file": fname, "status": "failed", "records": 0}

    ocean_time = data.get("ocean_time", [])
    n_time     = len(ocean_time)
    lat_rho    = data.get("lat_rho", [])
    n_sta      = len(lat_rho)
    s_rho      = data.get("s_rho", [])
    n_layer    = len(s_rho)

    if n_time == 0 or n_sta == 0:
        print("FAIL (no data)")
        return {"file": fname, "status": "failed", "records": 0}

    # Reshape flat lists → shaped numpy arrays
    NAN = float("nan")

    def _to_np2(flat, nt, ns):
        a = np.array(flat, dtype=np.float32)
        a[a > 1e29] = np.nan
        return a.reshape(nt, ns)

    def _to_np3(flat, nt, ns, nl):
        a = np.array(flat, dtype=np.float32)
        a[a > 1e29] = np.nan
        return a.reshape(nt, ns, nl)

    arrays = {
        "ocean_time": np.array(ocean_time, dtype=np.float64),
        "lat_rho":    np.array(data.get("lat_rho", []), dtype=np.float32),
        "lon_rho":    np.array(data.get("lon_rho", []), dtype=np.float32),
        "h":          np.array(data.get("h", []),       dtype=np.float32),
        "s_rho":      np.array(data.get("s_rho", []),   dtype=np.float32),
        "Cs_r":       np.array(data.get("Cs_r", []),    dtype=np.float32),
        "zeta":       _to_np2(data.get("zeta", []),     n_time, n_sta),
        "u":          _to_np3(data.get("u", []),         n_time, n_sta, n_layer),
        "v":          _to_np3(data.get("v", []),         n_time, n_sta, n_layer),
        "temp":       _to_np3(data.get("temp", []),      n_time, n_sta, n_layer),
    }

    dest_dir.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(out_path, **arrays)

    kb = out_path.stat().st_size / 1024
    print(f"OK  ({n_time}t × {n_sta}sta × {n_layer}lyr, {kb:.0f} KB npz)")
    return {"file": fname, "status": "ok", "records": n_time * n_sta,
            "n_time": n_time, "n_stations": n_sta, "size_kb": round(kb)}


def save_station_coords(urlpath, data_dir):
    """Download and save station lat/lon/depth once."""
    coord_path = data_dir / "station_coords.csv"
    if coord_path.exists():
        return
    print("  Writing station_coords.csv ...", end=" ", flush=True)
    data = fetch_vars_ascii(urlpath, ["lat_rho", "lon_rho", "h"])
    if data is None:
        print("FAIL")
        return
    lats   = data.get("lat_rho", [])
    lons   = data.get("lon_rho", [])
    depths = data.get("h", [])
    data_dir.mkdir(parents=True, exist_ok=True)
    with open(coord_path, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["station_idx", "lat", "lon", "depth_m"])
        for i, (la, lo, de) in enumerate(zip(lats, lons, depths)):
            w.writerow([i, round(la, 6), round(lo, 6), round(de, 3)])
    print(f"OK  ({len(lats)} stations)")


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description=__doc__,
                formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--data-dir", default=r"Q:\cbofs_data",
                        help="Root directory for downloaded data (default: Q:\\cbofs_data)")
    parser.add_argument("--dry-run", action="store_true",
                        help="List files that would be downloaded, don't download")
    parser.add_argument("--vars", default="ocean_time,lat_rho,lon_rho,h,s_rho,Cs_r,zeta,u,v,temp",
                        help="Comma-separated variable names to download")
    args = parser.parse_args()
    dl_vars  = [v.strip() for v in args.vars.split(",")]
    data_dir = Path(args.data_dir)
    log_path = data_dir / "download_log.csv"

    print("=" * 65)
    print("  CBOFS Stations Data Downloader")
    print(f"  Output: {data_dir}")
    print(f"  Variables: {', '.join(dl_vars)}")
    if args.dry_run:
        print("  *** DRY RUN — no files will be written ***")
    print("=" * 65)

    # Walk the catalog to find all available days
    print("\nEnumerating catalog...")
    days = enumerate_available_days()
    print(f"  Found {len(days)} day(s) in the archive")
    if not days:
        print("  No data found — exiting.")
        return

    # Summary of date range
    first = days[0]
    last  = days[-1]
    print(f"  Range: {first[0]:04d}-{first[1]:02d}-{first[2]:02d}  →  "
          f"{last[0]:04d}-{last[1]:02d}-{last[2]:02d}")

    if not args.dry_run:
        data_dir.mkdir(parents=True, exist_ok=True)

    # Load or init download log
    log_rows = []
    if log_path.exists():
        with open(log_path, newline="", encoding="utf-8") as f:
            existing = {r["file"] for r in csv.DictReader(f)}
    else:
        existing = set()

    coords_saved = False
    total_ok = 0
    total_skip = 0
    total_fail = 0
    total_dryrun = 0

    for yr, mo, dy, day_cat_url in days:
        print(f"\n  {yr:04d}-{mo:02d}-{dy:02d}")
        dest_dir = data_dir / f"{yr:04d}" / f"{mo:02d}" / f"{dy:02d}"

        # List all stations.nowcast files for this day
        dsets = list_datasets(day_cat_url)
        nc_files = [(name, up) for name, up in dsets
                    if "stations.nowcast" in name]
        nc_files.sort()  # t00z → t06z → t12z → t18z

        if not nc_files:
            print("    (no stations.nowcast files found)")
            continue

        for fname, urlpath in nc_files:
            # Save station coords once from the first file
            if not coords_saved and not args.dry_run:
                save_station_coords(urlpath, data_dir)
                coords_saved = True

            result = process_station_file(urlpath, dest_dir, dl_vars, args.dry_run)
            log_rows.append(result)
            st = result["status"]
            if st == "ok":
                total_ok += 1
            elif st == "skipped":
                total_skip += 1
            elif st == "dry-run":
                total_dryrun += 1
            else:
                total_fail += 1

            time.sleep(0.5)  # polite

    # Write log
    if log_rows and not args.dry_run:
        with open(log_path, "w", newline="", encoding="utf-8") as f:
            w = csv.DictWriter(f, fieldnames=["file", "status", "records",
                                               "n_time", "n_stations", "size_kb"],
                               extrasaction="ignore")
            w.writeheader()
            w.writerows(log_rows)

    print("\n" + "=" * 65)
    if args.dry_run:
        print(f"  Dry run complete.  Would download: {total_dryrun} files")
    else:
        print(f"  Done.  OK: {total_ok}  Skipped: {total_skip}  Failed: {total_fail}")
        print(f"  Log: {log_path}")
    print("=" * 65)


if __name__ == "__main__":
    main()
