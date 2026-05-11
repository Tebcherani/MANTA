"""
One-time fetch: download CBOFS fields-grid bathymetry (h, lat_rho, lon_rho,
mask_rho) from one fields.n001.nc file via OPeNDAP ASCII and save as
Q:\cbofs_data\cbofs_bathy.npz.

Run once:
    python SEnvironment/_cbofs_bathy_fetch.py
"""
import sys, re, math
import numpy as np
from pathlib import Path
import urllib.request, urllib.error

if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")

THREDDS  = "https://opendap.co-ops.nos.noaa.gov/thredds"
# h, lat_rho, lon_rho, mask_rho are static — any n001 file works
URLPATH  = "NOAA/CBOFS/MODELS/2026/05/09/cbofs.t12z.20260509.fields.n001.nc"
OUT_PATH = Path(r"Q:\cbofs_data\cbofs_bathy.npz")
VARS     = ["h", "lat_rho", "lon_rho", "mask_rho"]
ETA, XI  = 291, 332   # expected grid size


# ── OPeNDAP fetch ──────────────────────────────────────────────────────────────

def _fetch_ascii(urlpath: str, varnames: list) -> str:
    url = f"{THREDDS}/dodsC/{urlpath}.ascii?{','.join(varnames)}"
    print(f"Fetching {url}")
    req = urllib.request.Request(url, headers={"Accept": "*/*"})
    with urllib.request.urlopen(req, timeout=180) as resp:
        raw = resp.read()
    text = raw.decode("ascii", errors="replace")
    print(f"  received {len(text):,} bytes")
    return text


# ── ASCII parser (same approach as cbofs_download.py) ─────────────────────────

def _parse_var(text: str, varname: str) -> list:
    """Extract all floats for a named variable block in OPeNDAP ASCII output."""
    pattern = re.compile(
        rf"(?:^|\n){re.escape(varname)}\[[\d\]\[]*\]\n(.*?)(?=\n\n|\n[a-zA-Z_]|\Z)",
        re.DOTALL)
    m = pattern.search(text)
    if not m:
        return []
    raw = m.group(1)
    # Strip row-label brackets: [0], [0][5], etc.
    cleaned = re.sub(r"\[\d+\],?\s*", "", raw)
    nums = re.findall(r"[-+]?(?:\d+\.?\d*|\.\d+)(?:[eE][-+]?\d+)?", cleaned)
    return [float(v) for v in nums]


# ── Main ───────────────────────────────────────────────────────────────────────

def main():
    if OUT_PATH.exists():
        print(f"{OUT_PATH} already exists — delete it to re-fetch.")
        _report_existing()
        return

    text = _fetch_ascii(URLPATH, VARS)

    # Show a preview so we can diagnose parse failures
    sep = text.find("-----")
    preview_start = sep + 5 if sep >= 0 else 0
    print("\n--- Data block preview (first 300 chars) ---")
    print(text[preview_start : preview_start + 300])
    print("---\n")

    parsed = {}
    expected = ETA * XI
    for v in VARS:
        vals = _parse_var(text, v)
        if len(vals) != expected:
            print(f"  WARNING {v}: expected {expected} values, got {len(vals)}")
        else:
            print(f"  {v}: {len(vals):,} values OK")
        parsed[v] = np.array(vals, dtype=np.float64).reshape(ETA, XI)

    h       = parsed["h"]
    lat_rho = parsed["lat_rho"]
    lon_rho = parsed["lon_rho"]
    mask    = parsed["mask_rho"]

    # Water cells only (mask_rho == 1); land = 0
    water = mask > 0.5
    n_water = int(water.sum())
    print(f"\nWater cells: {n_water:,} / {ETA*XI:,}")
    print(f"h range (water only): {h[water].min():.2f} – {h[water].max():.2f} m")
    print(f"lat range: {lat_rho[water].min():.4f} – {lat_rho[water].max():.4f}")
    print(f"lon range: {lon_rho[water].min():.4f} – {lon_rho[water].max():.4f}")

    OUT_PATH.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(
        OUT_PATH,
        h       = h.astype(np.float32),
        lat_rho = lat_rho.astype(np.float32),
        lon_rho = lon_rho.astype(np.float32),
        mask_rho= mask.astype(np.uint8),
    )
    print(f"\nSaved: {OUT_PATH}  ({OUT_PATH.stat().st_size/1024:.0f} KB)")
    _report_existing()


def _report_existing():
    if not OUT_PATH.exists():
        return
    d = np.load(OUT_PATH)
    mask = d["mask_rho"] > 0
    h    = d["h"]
    lat  = d["lat_rho"]
    lon  = d["lon_rho"]
    d.close()
    n_water = int(mask.sum())
    print(f"\nGrid summary:")
    print(f"  Total cells : {mask.size:,}  ({mask.shape[0]}x{mask.shape[1]})")
    print(f"  Water cells : {n_water:,}")
    print(f"  h range     : {h[mask].min():.2f} – {h[mask].max():.2f} m")
    print(f"  lat range   : {lat[mask].min():.4f} – {lat[mask].max():.4f} deg")
    print(f"  lon range   : {lon[mask].min():.4f} – {lon[mask].max():.4f} deg")

    # Spot-check a few Bay locations
    lats_flat = lat[mask].ravel()
    lons_flat = lon[mask].ravel()
    hs_flat   = h[mask].ravel()
    cos_lat   = math.cos(math.radians(38.5))

    test_pts = [
        ("Baltimore",    39.27, -76.58),
        ("Mid-Bay",      38.32, -76.45),
        ("Bay mouth",    36.96, -76.11),
        ("Potomac",      38.12, -76.53),
    ]
    print(f"\n  {'Location':20s}  {'depth (m)':>10}")
    for name, qlat, qlon in test_pts:
        dlat = (lats_flat - qlat) * 111_000.0
        dlon = (lons_flat - qlon) * 111_000.0 * cos_lat
        dist2 = dlat**2 + dlon**2
        k = 4
        idx = np.argpartition(dist2, k)[:k]
        d2  = dist2[idx]
        w   = 1.0 / (d2 + 1.0)   # +1 avoids /0
        w  /= w.sum()
        depth = float(np.dot(w, hs_flat[idx]))
        print(f"  {name:20s}  {depth:>10.2f}")


if __name__ == "__main__":
    main()
