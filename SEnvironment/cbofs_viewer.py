"""
SEnvironment/cbofs_viewer.py  —  Interactive CBOFS data viewer  (v3)

Layout
------
  Left   : Bathymetry + station buoy locations + NOAA gauges
  Centre : Water-level / zeta  (pcolormesh on regular grid, land = dark)
  Right  : Current speed (pcolormesh) + direction (quiver arrows)
  Strip  : Dataset / selection info  +  radio buttons
  Bottom : Time series — model prediction + NOAA observed overlay (if gauge nearby)

Coordinate note
---------------
  CBOFS ROMS u,v are in rotated grid coordinates (xi/eta).  The xi axis
  makes ~23° from geographic east at the bay centre.  load_archive() rotates
  all U,V to geographic east-north using the bathy lat/lon grid before storing.

Interactions
------------
  Click time series  → moves time cursor; all three map panels update
  Click bathy map    → selects nearest model station; time series updates.
                       If a NOAA CO-OPS tide-gauge is within 0.08°, its
                       observed water-level is fetched and overlaid in orange.
  Zoom / pan any map → all three stay in sync
  Radio buttons      → switch time-series variable

Usage
-----
    python SEnvironment/cbofs_viewer.py [data_dir]
    default: Q:\\cbofs_data
"""

import sys, csv, math, json, urllib.request
import numpy as np
from pathlib import Path
from datetime import datetime, timezone

import matplotlib
for _be in ["qtagg", "Qt5Agg", "TkAgg"]:
    try:
        matplotlib.use(_be); break
    except Exception:
        pass

import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import matplotlib.dates as mdates
from matplotlib.widgets import RadioButtons
from scipy.interpolate import griddata

if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")

# ── Constants ─────────────────────────────────────────────────────────────────
_OFF = (datetime(2016,1,1,tzinfo=timezone.utc)
        - datetime(1970,1,1,tzinfo=timezone.utc)).total_seconds()

BAY  = (-77.5, -74.7, 36.0, 39.8)   # lon0, lon1, lat0, lat1
LAND_COLOR = "#1a1a1a"
DARK_BG    = "#0a0a12"
PANEL_BG   = "#0d1117"
ACCENT     = "#00d4ff"
RED        = "#ff3344"
ORANGE     = "#ff9900"
GRID_NLO, GRID_NLA = 220, 300       # regular interp grid size

# ── NOAA CO-OPS tide gauges in the Chesapeake / Delaware region ───────────────
_NOAA_GAUGES = [
    dict(id="8638863", name="CB Bridge-Tunnel", lat=36.967, lon=-76.113),
    dict(id="8638610", name="Sewells Point",    lat=36.947, lon=-76.330),
    dict(id="8637624", name="Yorktown",         lat=37.227, lon=-76.479),
    dict(id="8635750", name="Windmill Point",   lat=37.617, lon=-76.291),
    dict(id="8632200", name="Lewisetta",        lat=37.996, lon=-76.466),
    dict(id="8636580", name="Rappahannock Light",lat=37.540, lon=-76.016),
    dict(id="8571892", name="Cambridge MD",     lat=38.573, lon=-76.064),
    dict(id="8573927", name="Bishops Head",     lat=38.220, lon=-76.038),
    dict(id="8575512", name="Annapolis",        lat=38.983, lon=-76.480),
    dict(id="8574680", name="Baltimore",        lat=39.267, lon=-76.580),
    dict(id="8594900", name="Washington DC",    lat=38.873, lon=-77.021),
    dict(id="8551762", name="Delaware City",    lat=39.583, lon=-75.589),
]
_GAUGE_LAT = np.array([g["lat"] for g in _NOAA_GAUGES])
_GAUGE_LON = np.array([g["lon"] for g in _NOAA_GAUGES])
_GAUGE_SNAP = 0.15   # degrees — snap radius for gauge selection (~17 km)


def _to_posix(t): return np.asarray(t, dtype=np.float64) + _OFF
def _to_dt(t):    return datetime.fromtimestamp(float(t), tz=timezone.utc)


# ── Grid angle & u,v rotation ─────────────────────────────────────────────────

def _grid_angle(lat2d, lon2d):
    """Angle (radians) of ROMS xi-axis (array axis=1) from geographic east."""
    dlon = np.gradient(lon2d, axis=1)
    dlat = np.gradient(lat2d, axis=1)
    coslat = np.cos(np.radians(lat2d))
    return np.arctan2(dlat * 110574.0, dlon * 111320.0 * coslat)


def _rotate_uv(U, V, angles):
    """
    Rotate (n_time, n_sta) arrays from ROMS xi/eta to geographic east/north.
    angles : (n_sta,) in radians, angle of xi axis from east.
    """
    ca = np.cos(angles)[None, :]   # (1, n_sta)
    sa = np.sin(angles)[None, :]
    return U * ca - V * sa,  U * sa + V * ca


# ── NOAA CO-OPS fetch ─────────────────────────────────────────────────────────

def _fetch_noaa_wl(gauge_id, t0_posix, t1_posix):
    """
    Fetch 6-min MSL water level from NOAA CO-OPS for the given time window.
    Returns (times_posix_array, wl_m_array) or (None, None) on failure.
    """
    d0 = datetime.fromtimestamp(t0_posix, tz=timezone.utc)
    d1 = datetime.fromtimestamp(t1_posix, tz=timezone.utc)
    # API allows at most 31 days per request
    url = (
        "https://api.tidesandcurrents.noaa.gov/api/prod/datagetter?"
        f"product=water_level&application=cbofs_viewer"
        f"&begin_date={d0:%Y%m%d}&end_date={d1:%Y%m%d}"
        f"&datum=MSL&station={gauge_id}&time_zone=gmt"
        f"&units=metric&interval=6&format=json"
    )
    try:
        with urllib.request.urlopen(url, timeout=20) as r:
            raw = json.loads(r.read().decode())
        if "error" in raw:
            print(f"  NOAA API error: {raw['error']['message']}", flush=True)
            return None, None
        times, wl = [], []
        for pt in raw.get("data", []):
            try:
                t = datetime.strptime(pt["t"], "%Y-%m-%d %H:%M").replace(
                    tzinfo=timezone.utc)
                times.append(t.timestamp())
                wl.append(float(pt["v"]))
            except Exception:
                pass
        if not times:
            return None, None
        return np.array(times, np.float64), np.array(wl, np.float32)
    except Exception as exc:
        print(f"  NOAA fetch failed: {exc}", flush=True)
        return None, None


# ── Data loading ──────────────────────────────────────────────────────────────

def load_archive(data_dir: Path) -> dict:
    data_dir = Path(data_dir)

    with open(data_dir / "station_coords.csv", newline="") as f:
        rows = list(csv.DictReader(f))
    sta_lat = np.array([float(r["lat"])     for r in rows], np.float32)
    sta_lon = np.array([float(r["lon"])     for r in rows], np.float32)
    sta_h   = np.array([float(r["depth_m"]) for r in rows], np.float32)

    b = np.load(data_dir / "cbofs_bathy.npz")
    bathy = dict(lat=b["lat_rho"].copy(), lon=b["lon_rho"].copy(),
                 h=b["h"].copy(), mask=(b["mask_rho"]>0).copy())
    b.close()

    # Compute rotation angle at each station (nearest *water* grid cell)
    ang2d    = _grid_angle(bathy["lat"], bathy["lon"])
    water    = bathy["mask"].ravel()                        # True = water
    flat_lat = bathy["lat"].ravel()[water]
    flat_lon = bathy["lon"].ravel()[water]
    flat_ang = ang2d.ravel()[water]
    # Clip interior boundary artefacts (angles outside ±80° are suspect)
    flat_ang = np.clip(flat_ang, np.radians(-80), np.radians(80))
    dists2   = ((sta_lat[:, None] - flat_lat[None, :])**2
               +(sta_lon[:, None] - flat_lon[None, :])**2)  # (n_sta, n_water)
    nearest  = np.argmin(dists2, axis=1)                     # (n_sta,)
    sta_angles = flat_ang[nearest]                           # radians
    ang_deg    = np.degrees(sta_angles)
    print(f"  Grid angle at stations: "
          f"{ang_deg.min():.1f}° to {ang_deg.max():.1f}° from east "
          f"(mean {ang_deg.mean():.1f}°)", flush=True)

    files = sorted(f for f in data_dir.glob("**/*.npz") if "bathy" not in f.name)
    n = len(files)
    print(f"Loading {n} files …", flush=True)
    times, u_buf, v_buf, z_buf = [], [], [], []
    for i, fp in enumerate(files):
        if i % 20 == 0 or i == n-1:
            print(f"  {i+1}/{n}  {fp.name}", flush=True)
        d = np.load(fp)
        times.extend(_to_posix(d["ocean_time"]))
        u_buf.extend(d["u"][:, :, -1])   # surface layer, shape (61, 209)
        v_buf.extend(d["v"][:, :, -1])
        z_buf.extend(d["zeta"])
        d.close()
    print("  done.", flush=True)

    times = np.array(times, np.float64)
    U_xi  = np.array(u_buf, np.float32);  U_xi[np.abs(U_xi) > 1e10] = np.nan
    V_eta = np.array(v_buf, np.float32);  V_eta[np.abs(V_eta) > 1e10] = np.nan
    Z     = np.array(z_buf, np.float32);  Z[np.abs(Z) > 1e10] = np.nan

    # Rotate from ROMS xi/eta to geographic east/north
    U, V = _rotate_uv(U_xi, V_eta, sta_angles)

    with np.errstate(invalid="ignore"):
        MAG = np.sqrt(U**2 + V**2)

    t0, t1 = _to_dt(times[0]), _to_dt(times[-1])
    print(f"  {len(times)} steps  {t0:%Y-%m-%d} → {t1:%Y-%m-%d}", flush=True)
    return dict(sta_lat=sta_lat, sta_lon=sta_lon, sta_h=sta_h,
                bathy=bathy, times=times, U=U, V=V, Z=Z, MAG=MAG,
                n_files=n, n_sta=len(sta_lat))


# ── Regular interpolation grid ────────────────────────────────────────────────

def make_interp_grid(D: dict):
    """Build a regular lon/lat grid + land mask derived from the bathy npz."""
    lo0, lo1, la0, la1 = BAY
    glon = np.linspace(lo0, lo1, GRID_NLO)
    glat = np.linspace(la0, la1, GRID_NLA)
    glon2, glat2 = np.meshgrid(glon, glat)

    bd      = D["bathy"]
    src_pts = np.column_stack([bd["lon"].ravel(), bd["lat"].ravel()])
    src_msk = bd["mask"].ravel().astype(np.float32)
    msk_gi  = griddata(src_pts, src_msk, (glon2, glat2), method="nearest")
    land_mask = msk_gi < 0.5

    src_h  = np.where(bd["mask"].ravel(), bd["h"].ravel(), np.nan)
    h_gi   = griddata(src_pts, src_h, (glon2, glat2), method="linear")
    h_gi   = np.ma.array(h_gi, mask=land_mask)

    return dict(glon=glon, glat=glat, glon2=glon2, glat2=glat2,
                land=land_mask, h=h_gi)


def interp_field(grid, sta_lon, sta_lat, values):
    """Linear interpolation of station values to regular grid; land masked."""
    valid = np.isfinite(values)
    if valid.sum() < 4:
        return np.ma.array(np.zeros_like(grid["glon2"]), mask=True)
    pts = np.column_stack([sta_lon[valid], sta_lat[valid]])
    gi  = griddata(pts, values[valid],
                   (grid["glon2"], grid["glat2"]), method="linear")
    return np.ma.array(gi, mask=grid["land"] | ~np.isfinite(gi))


# ── Viewer ────────────────────────────────────────────────────────────────────

class CBOFSViewer:

    TS_LABELS = ["u  (E-W m/s)", "v  (N-S m/s)", "|current| m/s", "zeta  m"]

    def __init__(self, D: dict, grid: dict):
        self.D    = D
        self.grid = grid
        self.t_idx   = len(D["times"]) // 2
        self.s_idx   = self._nearest_sta(38.0, -76.4)
        self.ts_var  = "|current| m/s"
        self._syncing = False
        self._obs_times = None   # NOAA obs overlay
        self._obs_wl    = None
        self._obs_gauge = None   # gauge dict

        plt.style.use("dark_background")
        self._build_layout()
        self._init_bathy()
        self._init_zeta()
        self._init_curr()
        self._init_info()
        self._init_ts()
        self._link_zoom()

        self._update_quad()
        self._update_ts()

        self.fig.canvas.mpl_connect("button_press_event", self._on_click)
        plt.show()

    # ── Layout ───────────────────────────────────────────────────────────────

    def _build_layout(self):
        self.fig = plt.figure(figsize=(20, 12), facecolor=DARK_BG)

        outer = gridspec.GridSpec(2, 1, figure=self.fig,
                                   height_ratios=[2, 1], hspace=0.30)

        self._gs_maps = gridspec.GridSpecFromSubplotSpec(
            1, 3, subplot_spec=outer[0], wspace=0.12)

        self._gs_bot = gridspec.GridSpecFromSubplotSpec(
            1, 3, subplot_spec=outer[1],
            width_ratios=[3, 10, 1], wspace=0.08)

        def _ax(spec):
            a = self.fig.add_subplot(spec)
            a.set_facecolor(PANEL_BG)
            return a

        self.ax_bathy = _ax(self._gs_maps[0, 0])
        self.ax_zeta  = _ax(self._gs_maps[0, 1])
        self.ax_curr  = _ax(self._gs_maps[0, 2])

        self.ax_info  = self.fig.add_subplot(self._gs_bot[0])
        self.ax_info.set_facecolor(PANEL_BG); self.ax_info.axis("off")

        self.ax_ts    = _ax(self._gs_bot[1])
        self.ax_rb    = self.fig.add_subplot(self._gs_bot[2])
        self.ax_rb.set_facecolor(PANEL_BG); self.ax_rb.axis("off")

    def _link_zoom(self):
        self._map_axes = [self.ax_bathy, self.ax_zeta, self.ax_curr]
        for ax in self._map_axes:
            ax.callbacks.connect("xlim_changed", self._on_lim_change)
            ax.callbacks.connect("ylim_changed", self._on_lim_change)

    def _on_lim_change(self, src_ax):
        if self._syncing:
            return
        self._syncing = True
        xl, yl = src_ax.get_xlim(), src_ax.get_ylim()
        for ax in self._map_axes:
            if ax is not src_ax:
                ax.set_xlim(xl)
                ax.set_ylim(yl)
        self.fig.canvas.draw_idle()
        self._syncing = False

    # ── Bathymetry (static) ───────────────────────────────────────────────────

    def _init_bathy(self):
        D, g, ax = self.D, self.grid, self.ax_bathy
        ax.set_facecolor(LAND_COLOR)
        pcm = ax.pcolormesh(g["glon"], g["glat"], g["h"],
                            cmap="Blues_r", shading="auto",
                            vmin=0, vmax=float(np.nanmax(g["h"])))
        cb = self.fig.colorbar(pcm, ax=ax, fraction=0.025, pad=0.015)
        cb.set_label("depth (m)", color="white", fontsize=7)
        cb.ax.yaxis.set_tick_params(colors="white", labelsize=6)

        # NOAA gauge markers (triangles)
        ax.scatter(_GAUGE_LON, _GAUGE_LAT, marker="^", s=40,
                   c=ORANGE, alpha=0.9, zorder=6, label="NOAA gauges")
        for g_ in _NOAA_GAUGES:
            ax.annotate(g_["name"], xy=(g_["lon"], g_["lat"]),
                        xytext=(3, 3), textcoords="offset points",
                        fontsize=4.5, color=ORANGE, alpha=0.8, zorder=7)

        # Model station dots
        ax.scatter(D["sta_lon"], D["sta_lat"],
                   s=8, c="#ffdd00", alpha=0.75, zorder=5,
                   label=f"{D['n_sta']} model stations")
        ax.legend(loc="lower left", fontsize=6, framealpha=0.35)

        self.sel_star, = ax.plot(
            D["sta_lon"][self.s_idx], D["sta_lat"][self.s_idx],
            "r*", ms=14, zorder=10)
        self._gauge_marker, = ax.plot([], [], "o", ms=10, mec=ORANGE,
                                      mfc="none", mew=1.5, zorder=9)

        ax.set_title("Bathymetry  —  click to select station",
                     color="white", fontsize=9, pad=6)
        self._map_style(ax)

    # ── Zeta panel ───────────────────────────────────────────────────────────

    def _init_zeta(self):
        g, ax = self.grid, self.ax_zeta
        ax.set_facecolor(LAND_COLOR)
        dummy = np.ma.array(np.zeros((GRID_NLA, GRID_NLO)), mask=g["land"])
        self.pcm_zeta = ax.pcolormesh(g["glon"], g["glat"], dummy,
                                       cmap="RdBu_r", vmin=-0.8, vmax=0.8,
                                       shading="auto")
        cb = self.fig.colorbar(self.pcm_zeta, ax=ax, fraction=0.025, pad=0.015)
        cb.set_label("zeta (m)", color="white", fontsize=7)
        cb.ax.yaxis.set_tick_params(colors="white", labelsize=6)
        self._cb_zeta = cb
        self.title_zeta = ax.set_title("", color="white", fontsize=9, pad=6)
        self._map_style(ax)

    # ── Current panel (speed background + quiver arrows) ─────────────────────

    def _init_curr(self):
        g, ax = self.grid, self.ax_curr
        ax.set_facecolor(LAND_COLOR)
        dummy = np.ma.array(np.zeros((GRID_NLA, GRID_NLO)), mask=g["land"])
        self.pcm_curr = ax.pcolormesh(g["glon"], g["glat"], dummy,
                                       cmap="plasma", vmin=0, vmax=0.5,
                                       shading="auto", alpha=0.85)
        cb = self.fig.colorbar(self.pcm_curr, ax=ax, fraction=0.025, pad=0.015)
        cb.set_label("|current| (m/s)", color="white", fontsize=7)
        cb.ax.yaxis.set_tick_params(colors="white", labelsize=6)
        self._cb_curr = cb
        self._quiver = None
        ax.set_title("Surface current speed + direction  (geo. east-north)",
                     color="white", fontsize=9, pad=6)
        self._map_style(ax)

    # ── Info panel ────────────────────────────────────────────────────────────

    def _init_info(self):
        D = self.D
        t0 = _to_dt(D["times"][0]);  t1 = _to_dt(D["times"][-1])
        span = (D["times"][-1] - D["times"][0]) / 86400.0
        self._hdr = (
            f"Dataset\n"
            f"  Files : {D['n_files']}\n"
            f"  Stns  : {D['n_sta']}\n"
            f"  Start : {t0:%Y-%m-%d}\n"
            f"  End   : {t1:%Y-%m-%d}\n"
            f"  Span  : {span:.1f} days\n"
            f"  Steps : {len(D['times'])} × 6 min\n"
        )
        self.info_txt = self.ax_info.text(
            0.05, 0.97, self._hdr,
            transform=self.ax_info.transAxes,
            va="top", ha="left", fontsize=7.5,
            color="white", family="monospace", linespacing=1.5,
            bbox=dict(boxstyle="round,pad=0.5", fc="#141c28", ec="#304060"),
        )
        self.radio = RadioButtons(self.ax_rb, self.TS_LABELS, active=2,
                                   activecolor=ACCENT)
        for lbl in self.radio.labels:
            lbl.set_fontsize(7.5); lbl.set_color("white")
        self.radio.on_clicked(self._on_radio)

    def _refresh_info(self):
        D, ti, si = self.D, self.t_idx, self.s_idx
        t = _to_dt(D["times"][ti])
        def _f(v): return float(v) if math.isfinite(float(v)) else 0.0
        u_v = _f(D["U"][ti, si]); v_v = _f(D["V"][ti, si])
        z_v = _f(D["Z"][ti, si]); mag = math.sqrt(u_v**2 + v_v**2)
        gauge_str = ""
        if self._obs_gauge:
            gauge_str = (f"\nNOAA gauge\n"
                         f"  {self._obs_gauge['name']}\n"
                         f"  ID: {self._obs_gauge['id']}\n"
                         f"  (zeta overlay in orange)")
        body = (
            f"\nSelected time\n"
            f"  {t:%Y-%m-%d %H:%M UTC}\n"
            f"\nStation #{si}\n"
            f"  lat : {D['sta_lat'][si]:.4f} N\n"
            f"  lon : {D['sta_lon'][si]:.4f} E\n"
            f"  h   : {D['sta_h'][si]:.1f} m\n"
            f"\nSurface values (geo)\n"
            f"  u(E): {u_v:+.4f} m/s\n"
            f"  v(N): {v_v:+.4f} m/s\n"
            f"  mag : {mag:.4f} m/s\n"
            f"  zeta: {z_v:+.4f} m"
            f"{gauge_str}"
        )
        self.info_txt.set_text(self._hdr + body)

    # ── Time series ───────────────────────────────────────────────────────────

    def _init_ts(self):
        ax = self.ax_ts
        t_dt = [_to_dt(t) for t in self.D["times"]]
        self.ts_line, = ax.plot(t_dt, np.zeros(len(t_dt)),
                                 color=ACCENT, lw=0.75, alpha=0.9,
                                 label="CBOFS model", zorder=3)
        self.ts_obs_line, = ax.plot([], [], color=ORANGE, lw=1.0, alpha=0.85,
                                     label="NOAA obs", zorder=4)
        self.ts_vline = ax.axvline(_to_dt(self.D["times"][self.t_idx]),
                                    color=RED, lw=1.5, alpha=0.9, zorder=10)
        ax.xaxis.set_major_formatter(mdates.DateFormatter("%b %d"))
        ax.xaxis.set_major_locator(mdates.DayLocator(interval=3))
        ax.tick_params(colors="white", labelsize=7)
        ax.set_xlabel("Time (UTC)  —  click to select time",
                      color="white", fontsize=8)
        for sp in ax.spines.values(): sp.set_edgecolor("#304060")
        ax.grid(True, alpha=0.12, color="white", lw=0.5)
        self.ts_title  = ax.set_title("", color="white", fontsize=8)
        self.ts_ylabel = ax.set_ylabel("", color="white", fontsize=8)
        self.ts_legend = ax.legend(loc="upper right", fontsize=7,
                                    framealpha=0.4)

    def _update_ts(self):
        D, si = self.D, self.s_idx
        arr_map = {
            "u  (E-W m/s)":  D["U"][:, si],
            "v  (N-S m/s)":  D["V"][:, si],
            "|current| m/s": D["MAG"][:, si],
            "zeta  m":       D["Z"][:, si],
        }
        y = arr_map.get(self.ts_var, D["MAG"][:, si])
        self.ts_line.set_ydata(y)
        self.ax_ts.relim()
        self.ax_ts.autoscale_view(scalex=False, scaley=True)
        self.ts_ylabel.set_text(self.ts_var)
        self.ts_title.set_text(
            f"Station #{si}  ({D['sta_lat'][si]:.3f}°N, "
            f"{D['sta_lon'][si]:.3f}°E)  h={D['sta_h'][si]:.1f} m  "
            f"(near-surface)")

        # Overlay NOAA obs if zeta is selected and obs available
        if (self._obs_times is not None and self.ts_var == "zeta  m"):
            obs_dt = [_to_dt(t) for t in self._obs_times]
            self.ts_obs_line.set_data(obs_dt, self._obs_wl)
        else:
            self.ts_obs_line.set_data([], [])

        self.ax_ts.relim()
        self.ax_ts.autoscale_view(scalex=False, scaley=True)
        self.ts_legend.set_visible(self._obs_times is not None
                                    and self.ts_var == "zeta  m")
        self.fig.canvas.draw_idle()

    # ── Quad update ───────────────────────────────────────────────────────────

    def _update_quad(self):
        D, g = self.D, self.grid
        ti = self.t_idx
        t  = _to_dt(D["times"][ti])

        u = D["U"][ti, :]   # geographic east (rotated)
        v = D["V"][ti, :]   # geographic north (rotated)
        z = D["Z"][ti, :]

        z_gi  = interp_field(g, D["sta_lon"], D["sta_lat"], z)
        u_gi  = interp_field(g, D["sta_lon"], D["sta_lat"], u)
        v_gi  = interp_field(g, D["sta_lon"], D["sta_lat"], v)
        with np.errstate(invalid="ignore"):
            mag_gi = np.ma.sqrt(u_gi**2 + v_gi**2)

        # ── Zeta ──────────────────────────────────────────────────────────
        vmax_z = max(float(np.nanmax(np.abs(z_gi.filled(0)))), 0.05)
        self.pcm_zeta.set_array(z_gi.filled(np.nan).ravel())
        self.pcm_zeta.set_clim(-vmax_z, vmax_z)
        self._cb_zeta.update_normal(self.pcm_zeta)
        self.title_zeta.set_text(f"Water level (zeta)   {t:%Y-%m-%d %H:%M UTC}")

        # ── Current speed background ───────────────────────────────────────
        vmax_m = max(float(np.nanmax(mag_gi.filled(0))), 0.05)
        self.pcm_curr.set_array(mag_gi.filled(np.nan).ravel())
        self.pcm_curr.set_clim(0, vmax_m)
        self._cb_curr.update_normal(self.pcm_curr)

        # ── Quiver direction arrows (geographic east-north) ────────────────
        if self._quiver is not None:
            try: self._quiver.remove()
            except Exception: pass
            self._quiver = None

        ax = self.ax_curr
        sk = 7   # subsample every sk-th grid cell
        glon_q = g["glon2"][::sk, ::sk]
        glat_q = g["glat2"][::sk, ::sk]
        land_q = g["land"][::sk, ::sk]
        u_q    = u_gi.filled(np.nan)[::sk, ::sk]
        v_q    = v_gi.filled(np.nan)[::sk, ::sk]

        valid = ~land_q & np.isfinite(u_q) & np.isfinite(v_q)
        if valid.any():
            mag_q = np.sqrt(u_q[valid]**2 + v_q[valid]**2)
            # Normalise to unit length so arrows show direction only
            # (speed is already shown by background colour)
            safe  = mag_q > 0.02
            u_n   = np.where(safe, u_q[valid] / np.where(safe, mag_q, 1), 0)
            v_n   = np.where(safe, v_q[valid] / np.where(safe, mag_q, 1), 0)
            self._quiver = ax.quiver(
                glon_q[valid], glat_q[valid], u_n, v_n,
                color="white", alpha=0.75, zorder=5,
                scale=28, scale_units="xy",
                width=0.003, headwidth=4, headlength=5, headaxislength=4,
            )

        # ── Time marker ───────────────────────────────────────────────────
        self.ts_vline.set_xdata([t, t])
        self._refresh_info()
        self.fig.canvas.draw_idle()

    # ── Events ────────────────────────────────────────────────────────────────

    def _on_click(self, event):
        if event.inaxes is self.ax_ts and event.xdata is not None:
            tc = mdates.num2date(event.xdata).replace(tzinfo=timezone.utc)
            self.t_idx = int(np.argmin(np.abs(self.D["times"] - tc.timestamp())))
            self._update_quad()
            self._update_ts()

        elif event.inaxes is self.ax_bathy and event.xdata is not None:
            clat, clon = event.ydata, event.xdata

            # Check proximity to a NOAA gauge first
            d2 = (_GAUGE_LAT - clat)**2 + (_GAUGE_LON - clon)**2
            gi = int(np.argmin(d2))
            dist_deg = math.sqrt(float(d2[gi]))
            print(f"  bathy click: lat={clat:.3f} lon={clon:.3f}  "
                  f"nearest gauge={_NOAA_GAUGES[gi]['name']}  "
                  f"dist={dist_deg:.3f}°  snap={_GAUGE_SNAP}°",
                  flush=True)
            if d2[gi] < _GAUGE_SNAP**2:
                self._select_gauge(gi)
            else:
                # Clear gauge overlay when clicking away from gauges
                self._clear_gauge()

            # Always move model station selection to nearest station
            dlat = self.D["sta_lat"] - clat
            dlon = self.D["sta_lon"] - clon
            self.s_idx = int(np.argmin(dlat**2 + dlon**2))
            self.sel_star.set_data([self.D["sta_lon"][self.s_idx]],
                                    [self.D["sta_lat"][self.s_idx]])
            self._update_ts()
            self._refresh_info()
            self.fig.canvas.draw_idle()

    def _select_gauge(self, gi):
        """Select a NOAA gauge: mark it and fetch/display its water-level data."""
        gauge = _NOAA_GAUGES[gi]
        if self._obs_gauge and self._obs_gauge["id"] == gauge["id"]:
            return   # already selected
        self._obs_gauge = gauge
        self._gauge_marker.set_data([gauge["lon"]], [gauge["lat"]])

        # Temporarily show "fetching" in info before blocking network call
        self._refresh_info()
        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()

        print(f"  Fetching NOAA {gauge['name']} (#{gauge['id']}) …",
              flush=True)
        t0 = float(self.D["times"][0])
        t1 = float(self.D["times"][-1])
        times_obs, wl_obs = _fetch_noaa_wl(gauge["id"], t0, t1)
        if times_obs is not None:
            self._obs_times = times_obs
            self._obs_wl    = wl_obs
            print(f"    got {len(times_obs)} obs points", flush=True)
        else:
            self._obs_times = None
            self._obs_wl    = None
            print("    no data returned", flush=True)

        # Auto-switch time series to zeta so the overlay is immediately visible
        self.ts_var = "zeta  m"
        try:
            # set_active also updates the visual state of the radio button
            for i, lbl in enumerate(self.radio.labels):
                if lbl.get_text() == "zeta  m":
                    self.radio.set_active(i)
                    break
        except Exception:
            pass
        self._update_ts()

    def _clear_gauge(self):
        self._obs_gauge = None
        self._obs_times = None
        self._obs_wl    = None
        self._gauge_marker.set_data([], [])

    def _on_radio(self, label):
        self.ts_var = label
        self._update_ts()

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _nearest_sta(self, lat, lon):
        D = self.D
        return int(np.argmin((D["sta_lat"]-lat)**2 + (D["sta_lon"]-lon)**2))

    def _map_style(self, ax):
        lo0, lo1, la0, la1 = BAY
        ax.set_xlim(lo0, lo1); ax.set_ylim(la0, la1)
        ax.set_aspect("equal", adjustable="box")
        ax.set_xlabel("Longitude", color="white", fontsize=7)
        ax.set_ylabel("Latitude",  color="white", fontsize=7)
        ax.tick_params(colors="white", labelsize=6)
        for sp in ax.spines.values(): sp.set_edgecolor("#304060")


# ── Entry point ───────────────────────────────────────────────────────────────

if __name__ == "__main__":
    data_dir = Path(sys.argv[1]) if len(sys.argv) > 1 else Path(r"Q:\cbofs_data")
    D    = load_archive(data_dir)
    print("Building interpolation grid …", flush=True)
    grid = make_interp_grid(D)
    print("  done.", flush=True)
    CBOFSViewer(D, grid)
