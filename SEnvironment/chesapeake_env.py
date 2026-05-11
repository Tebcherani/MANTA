"""
SEnvironment/chesapeake_env.py  —  Chesapeake Bay operating environment.

Loads CBOFS station model data from Q:\\cbofs_data (downloaded by
cbofs_download.py) and provides current vectors and water-level
corrections at arbitrary (lat, lon, depth, time) for AUV simulation.

Quick start
-----------
    from SEnvironment.chesapeake_env import ChesapeakeEnvironment
    env = ChesapeakeEnvironment("Q:/cbofs_data")
    u, v = env.current(lat=38.32, lon=-76.45, depth_m=15.0, posix_time=t)
    zeta  = env.water_level(lat=38.32, lon=-76.45, posix_time=t)

Coordinate conventions
-----------------------
  lat, lon   : decimal degrees (WGS-84)
  depth_m    : positive = below surface (0 = surface, 15 = 15 m deep)
  posix_time : float seconds since Unix epoch (1970-01-01 UTC)
  u          : eastward  velocity  (m/s, positive = flowing east)
  v          : northward velocity  (m/s, positive = flowing north)
  zeta       : water surface elevation above MLLW datum (m)

Spatial interpolation
---------------------
  Inverse-distance weighting (IDW) using the k=6 nearest CBOFS stations.
  Stations with fill values (NaN) at the requested depth are excluded.

Vertical interpolation
----------------------
  ROMS sigma coordinates are converted to physical depth using:
      z(σ, x, t) = Cs(σ) * (h + ζ) + ζ
  Linear interpolation between the two nearest sigma layers.

Temporal interpolation
----------------------
  Linear interpolation between the two bounding 6-minute timesteps.
  Files are loaded lazily by day; the current day's files are cached.
"""

from __future__ import annotations

import math
import os
from datetime import datetime, timezone, timedelta
from pathlib import Path
from typing import Tuple, Optional

import numpy as np

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

CBOFS_EPOCH  = datetime(2016, 1, 1, tzinfo=timezone.utc)
UNIX_EPOCH   = datetime(1970, 1, 1, tzinfo=timezone.utc)
EPOCH_OFFSET = (CBOFS_EPOCH - UNIX_EPOCH).total_seconds()   # 1451606400.0

_IDW_K       = 6      # neighbours for spatial interpolation
_IDW_P       = 2.0    # inverse-distance power


# ---------------------------------------------------------------------------
# File naming helpers
# ---------------------------------------------------------------------------

def _run_label(cbofs_t: float) -> str:
    """Map CBOFS ocean_time (s) → 6-h run label (t00z/t06z/t12z/t18z).

    Files are named by the END of their 6-h nowcast window:
      t06z covers 00:00-06:00, t12z covers 06:00-12:00, etc.
    """
    dt = CBOFS_EPOCH + timedelta(seconds=float(cbofs_t))
    run_hour = ((dt.hour // 6) + 1) * 6
    if run_hour >= 24:
        run_hour = 0
        dt = dt + timedelta(days=1)
    return f"t{run_hour:02d}z"


def _npz_path(data_dir: Path, cbofs_t: float) -> Path:
    """Return the expected .npz path for the file covering cbofs_t."""
    dt = CBOFS_EPOCH + timedelta(seconds=float(cbofs_t))
    run_hour = ((dt.hour // 6) + 1) * 6
    if run_hour >= 24:
        run_hour = 0
        dt = dt + timedelta(days=1)
    fname = f"cbofs.t{run_hour:02d}z.{dt.strftime('%Y%m%d')}.stations.nowcast.npz"
    return data_dir / f"{dt.year:04d}" / f"{dt.month:02d}" / f"{dt.day:02d}" / fname


def posix_to_cbofs(posix_t: float) -> float:
    """Convert Unix epoch seconds → CBOFS epoch seconds."""
    return posix_t - EPOCH_OFFSET


def cbofs_to_posix(cbofs_t: float) -> float:
    """Convert CBOFS epoch seconds → Unix epoch seconds."""
    return cbofs_t + EPOCH_OFFSET


# ---------------------------------------------------------------------------
# ChesapeakeEnvironment
# ---------------------------------------------------------------------------

class ChesapeakeEnvironment:
    """
    Chesapeake Bay current and water-level environment for AUV simulation.

    Parameters
    ----------
    data_dir : str or Path
        Root of the cbofs_data tree (contains station_coords.csv and
        YYYY/MM/DD/*.npz files created by cbofs_download.py).
    verbose : bool
        Print file-load messages (default False).
    """

    def __init__(self, data_dir: str | Path, verbose: bool = False):
        self.data_dir = Path(data_dir)
        self.verbose  = verbose

        # Station coordinates (constant across all files)
        coords_path = self.data_dir / "station_coords.csv"
        if not coords_path.exists():
            raise FileNotFoundError(f"station_coords.csv not found in {self.data_dir}")
        self._load_coords(coords_path)

        # High-res bathymetric grid (optional — from cbofs_bathy.npz)
        self._grid_lat: Optional[np.ndarray] = None
        self._grid_lon: Optional[np.ndarray] = None
        self._grid_h:   Optional[np.ndarray] = None
        bathy_path = self.data_dir / "cbofs_bathy.npz"
        if bathy_path.exists():
            self._load_bathy_grid(bathy_path)

        # Cache: {npz_path_str: loaded_dict}
        self._cache: dict[str, dict] = {}

    # ── Initialisation helpers ───────────────────────────────────────────────

    def _load_coords(self, path: Path) -> None:
        import csv
        lats, lons, depths = [], [], []
        with open(path, newline="", encoding="utf-8") as f:
            for row in csv.DictReader(f):
                lats.append(float(row["lat"]))
                lons.append(float(row["lon"]))
                depths.append(float(row["depth_m"]))
        self._sta_lat   = np.array(lats,   dtype=np.float32)
        self._sta_lon   = np.array(lons,   dtype=np.float32)
        self._sta_depth = np.array(depths, dtype=np.float32)
        self._n_sta     = len(lats)

    def _load_bathy_grid(self, path: Path) -> None:
        """Load the high-res CBOFS fields bathymetric grid (water cells only)."""
        f = np.load(path)
        mask = f["mask_rho"] > 0
        self._grid_lat = f["lat_rho"][mask].astype(np.float64)
        self._grid_lon = f["lon_rho"][mask].astype(np.float64)
        self._grid_h   = f["h"][mask].astype(np.float64)
        f.close()
        if self.verbose:
            print(f"  [bathy] {len(self._grid_h):,} water cells, "
                  f"depth {self._grid_h.min():.1f}–{self._grid_h.max():.1f} m")

    # ── File loading ──────────────────────────────────────────────────────────

    def _load_file(self, npz_path: Path) -> Optional[dict]:
        key = str(npz_path)
        if key in self._cache:
            return self._cache[key]

        if not npz_path.exists():
            return None

        if self.verbose:
            print(f"  [env] loading {npz_path.name}")

        f = np.load(npz_path)
        d = {k: f[k] for k in f.files}
        f.close()

        # Limit cache to 8 files (~21 MB) to avoid unbounded growth
        if len(self._cache) >= 8:
            oldest_key = next(iter(self._cache))
            del self._cache[oldest_key]
        self._cache[key] = d
        return d

    # ── Sigma-coordinate depth conversion ────────────────────────────────────

    @staticmethod
    def _sigma_depths(s_rho: np.ndarray, Cs_r: np.ndarray,
                      h: float, zeta: float) -> np.ndarray:
        """
        Convert sigma coordinates → physical depths below surface (positive down).

        ROMS formula:  z(σ) = Cs(σ)*(h+ζ) + ζ
        Returns depths below *surface* = −z  (so 0 = surface, h = seabed).
        """
        z = Cs_r * (h + zeta) + zeta          # negative = below surface
        return -z                               # positive depth below surface

    # ── Nearest-station IDW weights ───────────────────────────────────────────

    def _idw_weights(self, lat: float, lon: float) -> Tuple[np.ndarray, np.ndarray]:
        """
        Return (indices, weights) for IDW interpolation to (lat, lon).
        Uses flat-earth approximation (fine over Bay scales ~300 km).
        """
        # Degrees → approximate metres
        dlat = (self._sta_lat - lat) * 111_000.0
        dlon = (self._sta_lon - lon) * 111_000.0 * math.cos(math.radians(lat))
        dist2 = dlat**2 + dlon**2

        # k nearest
        k = min(_IDW_K, self._n_sta)
        idx = np.argpartition(dist2, k)[:k]

        d2 = dist2[idx]
        # Handle exact coincidence
        if d2.min() < 1.0:
            exact = np.where(d2 < 1.0)[0]
            w = np.zeros(k)
            w[exact[0]] = 1.0
        else:
            w = 1.0 / (d2 ** (_IDW_P / 2.0))
            w /= w.sum()

        return idx, w

    # ── Core interpolation ────────────────────────────────────────────────────

    def _interpolate(self, lat: float, lon: float, depth_m: float,
                     posix_time: float) -> Tuple[float, float, float]:
        """
        Return (u_east, v_north, zeta) at (lat, lon, depth_m, posix_time).
        Returns (0, 0, 0) when data is unavailable.
        """
        cbofs_t = posix_to_cbofs(posix_time)

        npz_path = _npz_path(self.data_dir, cbofs_t)
        d = self._load_file(npz_path)

        if d is None:
            # Try adjacent run (±6 h) as fallback
            for delta_h in (-6, 6, -12, 12):
                alt_t = cbofs_t + delta_h * 3600
                alt_path = _npz_path(self.data_dir, alt_t)
                d = self._load_file(alt_path)
                if d is not None:
                    break
            if d is None:
                return 0.0, 0.0, 0.0

        ocean_time = d["ocean_time"]   # [n_time]
        s_rho      = d["s_rho"]        # [n_lyr]
        Cs_r       = d["Cs_r"]         # [n_lyr]

        # ── Temporal interpolation ────────────────────────────────────────────
        if cbofs_t <= ocean_time[0]:
            ti0, ti1, alpha = 0, 0, 0.0
        elif cbofs_t >= ocean_time[-1]:
            ti0, ti1, alpha = len(ocean_time)-1, len(ocean_time)-1, 0.0
        else:
            ti1 = int(np.searchsorted(ocean_time, cbofs_t))
            ti0 = ti1 - 1
            dt_frac = (cbofs_t - ocean_time[ti0]) / (ocean_time[ti1] - ocean_time[ti0])
            alpha = float(dt_frac)

        zeta_t0 = d["zeta"][ti0]   # [n_sta]
        zeta_t1 = d["zeta"][ti1]
        u_t0    = d["u"][ti0]      # [n_sta, n_lyr]
        u_t1    = d["u"][ti1]
        v_t0    = d["v"][ti0]
        v_t1    = d["v"][ti1]

        # Temporally interpolated fields
        zeta_sta = zeta_t0 * (1 - alpha) + zeta_t1 * alpha   # [n_sta]
        u_sta    = u_t0    * (1 - alpha) + u_t1    * alpha   # [n_sta, n_lyr]
        v_sta    = v_t0    * (1 - alpha) + v_t1    * alpha

        # ── Spatial IDW ───────────────────────────────────────────────────────
        sta_idx, weights = self._idw_weights(lat, lon)

        # ── Vertical interpolation at each neighbour ──────────────────────────
        u_vals, v_vals, z_vals, w_used = [], [], [], []

        for si, wi in zip(sta_idx, weights):
            ze     = float(zeta_sta[si])
            h_sta  = float(self._sta_depth[si])
            depths = self._sigma_depths(s_rho, Cs_r, h_sta, ze)  # [n_lyr]

            # depth_m is positive below surface; depths[] also positive below surface
            # Layer 0 = near-bottom, layer -1 = near-surface
            d_min, d_max = float(depths.max()), float(depths.min())
            # depths[0] ≈ h_sta (bottom), depths[-1] ≈ 0 (surface)

            u_prof = u_sta[si]   # [n_lyr]
            v_prof = v_sta[si]

            # Clamp to available depth range
            req_depth = max(0.0, min(depth_m, h_sta - 0.1))

            # Interpolate vertically (depths decreasing index = deeper)
            # depths is monotonically DECREASING (index 0 = deepest)
            if req_depth <= depths[-1]:
                # At or above surface layer
                u_interp = float(u_prof[-1]) if not np.isnan(u_prof[-1]) else 0.0
                v_interp = float(v_prof[-1]) if not np.isnan(v_prof[-1]) else 0.0
            elif req_depth >= depths[0]:
                # At or below bottom layer
                u_interp = float(u_prof[0]) if not np.isnan(u_prof[0]) else 0.0
                v_interp = float(v_prof[0]) if not np.isnan(v_prof[0]) else 0.0
            else:
                # Linear interp between two layers
                # depths descends from index 0 (deep) to -1 (shallow)
                # We need the first layer SHALLOWER than req_depth
                li1 = int(np.searchsorted(-depths, -req_depth))  # ascending search
                li0 = li1 - 1
                frac = ((req_depth - depths[li0]) /
                        (depths[li1] - depths[li0]))
                u_interp = float(u_prof[li0] * (1-frac) + u_prof[li1] * frac)
                v_interp = float(v_prof[li0] * (1-frac) + v_prof[li1] * frac)
                if np.isnan(u_interp) or np.isnan(v_interp):
                    u_interp = 0.0
                    v_interp = 0.0

            u_vals.append(u_interp)
            v_vals.append(v_interp)
            z_vals.append(ze)
            w_used.append(wi)

        if not w_used:
            return 0.0, 0.0, 0.0

        w_arr = np.array(w_used)
        w_arr /= w_arr.sum()

        u_out   = float(np.dot(w_arr, u_vals))
        v_out   = float(np.dot(w_arr, v_vals))
        zeta_out= float(np.dot(w_arr, z_vals))

        return u_out, v_out, zeta_out

    # ── Public API ────────────────────────────────────────────────────────────

    def current(self, lat: float, lon: float, depth_m: float,
                posix_time: float) -> Tuple[float, float]:
        """
        Return (u_east_mps, v_north_mps) at the given position and time.

        Parameters
        ----------
        lat, lon   : decimal degrees
        depth_m    : depth below surface (m, positive down)
        posix_time : Unix epoch seconds (float)
        """
        u, v, _ = self._interpolate(lat, lon, depth_m, posix_time)
        return u, v

    def water_level(self, lat: float, lon: float,
                    posix_time: float) -> float:
        """
        Return water surface elevation (m above MLLW) at (lat, lon, time).

        This is the CBOFS modelled zeta, which includes both tidal and
        meteorological (surge) contributions — more accurate than tide tables.
        """
        _, _, zeta = self._interpolate(lat, lon, 0.0, posix_time)
        return zeta

    def depth(self, lat: float, lon: float) -> float:
        """
        Return seafloor depth below MLLW (m) at (lat, lon).

        Uses the CBOFS fields-grid bathymetry (19,772 water cells, ~400 m
        resolution) when cbofs_bathy.npz is present; falls back to IDW over
        the 209 CBOFS station h values otherwise.
        """
        if self._grid_h is not None:
            return self._depth_from_grid(lat, lon)
        idx, weights = self._idw_weights(lat, lon)
        return float(np.dot(weights, self._sta_depth[idx]))

    def _depth_from_grid(self, lat: float, lon: float) -> float:
        """IDW depth from the high-res CBOFS fields grid (k=4 nearest cells)."""
        dlat = (self._grid_lat - lat) * 111_000.0
        dlon = (self._grid_lon - lon) * 111_000.0 * math.cos(math.radians(lat))
        dist2 = dlat**2 + dlon**2
        k = 4
        idx = np.argpartition(dist2, k)[:k]
        d2 = dist2[idx]
        if d2.min() < 1.0:          # query lands exactly on a grid point
            return float(self._grid_h[idx[int(d2.argmin())]])
        w = 1.0 / d2
        w /= w.sum()
        return float(np.dot(w, self._grid_h[idx]))

    def water_column_depth(self, lat: float, lon: float,
                           posix_time: float) -> float:
        """
        Return total instantaneous water column depth (m) = h + zeta.

        Accounts for the tide/surge at the given time.  Useful for AUV
        maximum-depth planning (never command depth_m > water_column_depth).
        """
        return self.depth(lat, lon) + self.water_level(lat, lon, posix_time)

    def summary(self) -> str:
        """Multi-line summary of available data."""
        npz_files = sorted(self.data_dir.glob("**/*.npz"))
        n_data = sum(1 for f in npz_files if "bathy" not in f.name)
        if self._grid_h is not None:
            bathy_src = f"CBOFS fields grid ({len(self._grid_h):,} water cells, ~400 m res)"
        else:
            bathy_src = f"209-station IDW (low-res fallback)"
        return (
            f"ChesapeakeEnvironment\n"
            f"  data_dir  : {self.data_dir}\n"
            f"  CBOFS runs: {n_data} files ({self._n_sta} stations)\n"
            f"  bathymetry: {bathy_src}"
        )


# ---------------------------------------------------------------------------
# Quick self-test
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    import sys

    data_dir = sys.argv[1] if len(sys.argv) > 1 else r"Q:\cbofs_data"
    print(f"Loading environment from {data_dir} ...")

    env = ChesapeakeEnvironment(data_dir, verbose=True)
    print(env.summary())

    # Find a sample time from an available npz file
    npz_files = sorted(Path(data_dir).glob("**/*.npz"))
    if not npz_files:
        print("No .npz files found — run cbofs_download.py first.")
        sys.exit(1)

    sample = np.load(npz_files[len(npz_files)//2])
    t_mid_cbofs = float(sample["ocean_time"][30])
    sample.close()
    t_posix = cbofs_to_posix(t_mid_cbofs)
    t_utc   = datetime.fromtimestamp(t_posix, tz=timezone.utc)

    print(f"\nSample query time: {t_utc.strftime('%Y-%m-%d %H:%M UTC')}")
    print()

    # Test at a few Bay locations
    test_pts = [
        ("Baltimore Harbour",   39.27, -76.58, 5.0),
        ("Mid-Bay (Solomons)",  38.32, -76.45, 10.0),
        ("Bay mouth (CBBT)",    36.96, -76.11, 12.0),
        ("Potomac entrance",    38.12, -76.53, 8.0),
    ]

    print(f"  {'Location':30s}  {'u_E (m/s)':>10}  {'v_N (m/s)':>10}  {'zeta (m)':>9}  {'h (m)':>7}  {'h+zeta (m)':>11}")
    print(f"  {'-'*30}  {'-'*10}  {'-'*10}  {'-'*9}  {'-'*7}  {'-'*11}")
    for name, lat, lon, depth in test_pts:
        u, v = env.current(lat, lon, depth, t_posix)
        z    = env.water_level(lat, lon, t_posix)
        h    = env.depth(lat, lon)
        wcd  = env.water_column_depth(lat, lon, t_posix)
        spd  = math.sqrt(u**2 + v**2)
        print(f"  {name:30s}  {u:>10.4f}  {v:>10.4f}  {z:>9.3f}  {h:>7.1f}  {wcd:>11.3f}  (spd={spd:.3f})")
