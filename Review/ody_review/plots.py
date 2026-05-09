"""
Plotting routines — direct ports of the MATLAB review scripts.

Each function takes a conditioned xarray.Dataset (post conditioning.condition)
and an optional matplotlib Figure or Axes. Returns the Figure for further
customization or saving.

Originals → Python:
  vplane.m       → vertical_plane()
  navplot.m      → navplot()
  CTD_process.m  → ctd_time_series(), ctd_profiles()
  CascadeCTD.m   → cascade_ctd()
  tscast.m       → ts_cast()
  optics_cast.m  → optics_cast()
  plt_depth_rate → depth_rate_check()
"""
from __future__ import annotations

import numpy as np
import xarray as xr
import matplotlib.pyplot as plt


def _need(ds: xr.Dataset, *names: str) -> None:
    missing = [n for n in names if n not in ds]
    if missing:
        raise KeyError(f"Plot requires {missing} which are not in the dataset.")


def _run_name(ds: xr.Dataset) -> str:
    src = ds.attrs.get("source_file", "")
    if src:
        # Make underscores LaTeX-safe (originals replaced _ with .)
        from pathlib import Path
        return Path(src).stem.upper().replace("_", ".")
    return ds.attrs.get("vehicle", "Mission")


# ---------------------------------------------------------------------------
# Clipboard helper — adds a "Copy to clipboard" button to any figure
# ---------------------------------------------------------------------------
def _add_copy_button(fig: plt.Figure) -> None:
    """Add a small 'Copy' button to the bottom-right of the figure.

    Renders the figure to a PNG in memory and puts it on the Windows
    clipboard using the tkinter/PIL approach that works without any
    extra dependencies beyond what the review venv already has.
    Falls back silently on non-Windows platforms.
    """
    import sys
    from matplotlib.widgets import Button

    # Reserve a small axes in the bottom-right corner for the button.
    # [left, bottom, width, height] in figure-fraction coordinates.
    btn_ax = fig.add_axes([0.88, 0.01, 0.10, 0.04])
    btn = Button(btn_ax, "Copy", color="0.85", hovercolor="0.70")

    def _copy(event):
        import io
        buf = io.BytesIO()
        fig.savefig(buf, format="png", dpi=150, bbox_inches="tight")
        buf.seek(0)
        png_bytes = buf.read()

        if sys.platform == "win32":
            try:
                # Try PIL/Pillow first (most reliable on Windows)
                from PIL import Image
                import win32clipboard, win32con
                img = Image.open(io.BytesIO(png_bytes))
                # Convert to DIB (device-independent bitmap) for clipboard
                buf_bmp = io.BytesIO()
                img.convert("RGB").save(buf_bmp, format="BMP")
                bmp_data = buf_bmp.getvalue()[14:]   # strip BMP file header
                win32clipboard.OpenClipboard()
                win32clipboard.EmptyClipboard()
                win32clipboard.SetClipboardData(win32con.CF_DIB, bmp_data)
                win32clipboard.CloseClipboard()
                btn.label.set_text("Copied!")
            except ImportError:
                # Pillow or pywin32 not available — use PowerShell fallback
                try:
                    import subprocess, tempfile, os
                    with tempfile.NamedTemporaryFile(suffix=".png", delete=False) as tmp:
                        tmp.write(png_bytes)
                        tmp_path = tmp.name
                    ps_cmd = (
                        f"Add-Type -AssemblyName System.Windows.Forms; "
                        f"[System.Windows.Forms.Clipboard]::SetImage("
                        f"[System.Drawing.Image]::FromFile('{tmp_path}'))"
                    )
                    subprocess.run(
                        ["powershell", "-NoProfile", "-Command", ps_cmd],
                        check=True, capture_output=True
                    )
                    os.unlink(tmp_path)
                    btn.label.set_text("Copied!")
                except Exception:
                    btn.label.set_text("Failed")
        else:
            # Linux / macOS — xclip or pbcopy
            try:
                import subprocess
                if sys.platform == "darwin":
                    proc = subprocess.Popen(["pbcopy"], stdin=subprocess.PIPE)
                    proc.communicate(png_bytes)
                else:
                    proc = subprocess.Popen(
                        ["xclip", "-selection", "clipboard", "-t", "image/png"],
                        stdin=subprocess.PIPE
                    )
                    proc.communicate(png_bytes)
                btn.label.set_text("Copied!")
            except Exception:
                btn.label.set_text("Failed")

        fig.canvas.draw_idle()

    btn.on_clicked(_copy)
    # Keep a reference so the button isn't garbage-collected
    fig._copy_button = btn



# ---------------------------------------------------------------------------
# vplane.m → vertical_plane
# ---------------------------------------------------------------------------
def vertical_plane(ds: xr.Dataset) -> plt.Figure:
    """Depth and pitch vs time."""
    _need(ds, "m_depth", "m_pitch")
    t = ds["time"].values
    fig, axes = plt.subplots(2, 1, sharex=True, figsize=(9, 6))
    axes[0].plot(t, ds["m_depth"].values, "b-")
    if "m_depth_d" in ds:
        axes[0].plot(t, ds["m_depth_d"].values, "r--", lw=0.8, label="commanded")
        axes[0].legend(loc="best")
    axes[0].set_ylabel("Depth (m)")
    axes[0].invert_yaxis()
    axes[0].grid(True, alpha=0.3)
    axes[0].set_title(f"Vertical plane: {_run_name(ds)}")

    pitch_deg = np.degrees(ds["m_pitch"].values)
    axes[1].plot(t, pitch_deg, "g-")
    axes[1].set_ylabel("Pitch (deg)")
    axes[1].set_xlabel("Mission time (s)")
    axes[1].grid(True, alpha=0.3)
    fig.tight_layout()
    _add_copy_button(fig)
    return fig


# ---------------------------------------------------------------------------
# navplot.m → navplot
# ---------------------------------------------------------------------------
def navplot(ds: xr.Dataset) -> plt.Figure:
    """Track in horizontal plane (north vs east)."""
    _need(ds, "m_pos_n", "m_pos_e")
    fig, ax = plt.subplots(figsize=(7, 7))
    n = ds["m_pos_n"].values
    e = ds["m_pos_e"].values
    ax.plot(e, n, "b-", lw=0.8)
    if "u_veh_init_north" in ds.attrs and "u_veh_init_east" in ds.attrs:
        ax.plot(ds.attrs["u_veh_init_east"], ds.attrs["u_veh_init_north"],
                "go", markersize=10, label="start")
    ax.plot(e[0], n[0], "go", markersize=8, label="track start")
    ax.plot(e[-1], n[-1], "rs", markersize=8, label="track end")
    ax.set_xlabel("East (m)")
    ax.set_ylabel("North (m)")
    ax.set_aspect("equal", adjustable="datalim")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="best")
    ax.set_title(f"Track: {_run_name(ds)}")
    fig.tight_layout()
    _add_copy_button(fig)
    return fig


# ---------------------------------------------------------------------------
# CTD_process.m → ctd_time_series, ctd_profiles
# ---------------------------------------------------------------------------
def ctd_time_series(ds: xr.Dataset) -> plt.Figure:
    """Temperature and salinity vs time (replaces the first CTD_process figure)."""
    _need(ds, "water_temp", "salinity")
    t = ds["time"].values
    fig, axes = plt.subplots(2, 1, sharex=True, figsize=(9, 6))
    axes[0].plot(t, ds["water_temp"].values, "r.", markersize=2)
    axes[0].set_ylabel("Temperature (°C)")
    axes[0].grid(True, alpha=0.3)
    axes[0].set_title(f"Temperature & salinity vs time: {_run_name(ds)}")
    axes[1].plot(t, ds["salinity"].values, "b.", markersize=2)
    axes[1].set_ylabel("Salinity (psu)")
    axes[1].set_xlabel("Mission time (s)")
    axes[1].grid(True, alpha=0.3)
    fig.tight_layout()
    _add_copy_button(fig)
    return fig


def ctd_profiles(ds: xr.Dataset) -> plt.Figure:
    """T, S, density vs depth (replaces the second CTD_process figure)."""
    _need(ds, "water_temp", "salinity", "density", "ctd_depth")
    fig, axes = plt.subplots(1, 3, sharey=True, figsize=(11, 6))
    z = ds["ctd_depth"].values
    axes[0].plot(ds["water_temp"].values, z, "b-", lw=0.6)
    axes[0].set_xlabel("Temperature (°C)")
    axes[0].set_ylabel("Depth (dbar)")
    axes[0].invert_yaxis()
    axes[0].grid(True, alpha=0.3)

    axes[1].plot(ds["salinity"].values, z, "b-", lw=0.6)
    axes[1].set_xlabel("Salinity (psu)")
    axes[1].grid(True, alpha=0.3)
    axes[1].set_title(f"CTD profiles: {_run_name(ds)}")

    axes[2].plot(ds["density"].values, z, "b-", lw=0.6)
    axes[2].set_xlabel("Density (kg/m³)")
    axes[2].grid(True, alpha=0.3)
    fig.tight_layout()
    _add_copy_button(fig)
    return fig


# ---------------------------------------------------------------------------
# tscast.m → ts_cast
# ---------------------------------------------------------------------------
def ts_cast(ds: xr.Dataset) -> plt.Figure:
    """Temperature, salinity, sound velocity vs depth, separating up/down casts."""
    _need(ds, "water_temp", "salinity", "m_depth")
    has_svel = "sound_velocity" in ds
    n_panels = 3 if has_svel else 2
    fig, axes = plt.subplots(1, n_panels, sharey=True, figsize=(4 * n_panels, 6))

    z = ds["m_depth"].values
    up = ds["going_up"].values if "going_up" in ds else np.zeros(z.size, bool)
    down = ds["going_down"].values if "going_down" in ds else np.zeros(z.size, bool)

    def split_plot(ax, x, label):
        x_up = np.where(up, x, np.nan)
        x_dn = np.where(down, x, np.nan)
        ax.plot(x_dn, z, "b-", lw=0.6, label="descending")
        ax.plot(x_up, z, "r-", lw=0.6, label="ascending")
        ax.set_xlabel(label)
        ax.invert_yaxis()
        ax.grid(True, alpha=0.3)

    split_plot(axes[0], ds["water_temp"].values, "Temperature (°C)")
    axes[0].set_ylabel("Depth (m)")
    axes[0].legend(loc="best")
    split_plot(axes[1], ds["salinity"].values, "Salinity (psu)")
    if has_svel:
        split_plot(axes[2], ds["sound_velocity"].values, "Sound velocity (m/s)")
    fig.suptitle(f"T-S cast: {_run_name(ds)}")
    fig.tight_layout()
    _add_copy_button(fig)
    return fig


# ---------------------------------------------------------------------------
# CascadeCTD.m → cascade_ctd
# ---------------------------------------------------------------------------
def cascade_ctd(
    ds: xr.Dataset, cascade_t: float = 0.5, cascade_s: float = 0.2
) -> plt.Figure:
    """Stack each yoyo's up/down cast with a horizontal offset to highlight changes."""
    _need(ds, "water_temp", "salinity", "yoyo", "d_rate", "m_depth")
    yoyo = ds["yoyo"].values
    dr = ds["d_rate"].values
    z = ds["m_depth"].values
    t_w = ds["water_temp"].values
    sal = ds["salinity"].values

    yoyo_max = int(np.nanmax(yoyo)) if np.isfinite(np.nanmax(yoyo)) else 0

    fig, axes = plt.subplots(1, 2, figsize=(11, 6))
    for i in range(1, yoyo_max + 1):
        mask_up = (yoyo == i) & (dr < -0.1)
        mask_dn = (yoyo == i) & (dr > 0.1)
        axes[0].plot(t_w[mask_up] + cascade_t * i, z[mask_up], "r-", lw=0.6)
        axes[0].plot(t_w[mask_dn] + cascade_t * i, z[mask_dn], "b-", lw=0.6)
        axes[1].plot(sal[mask_up] + cascade_s * i, z[mask_up], "r-", lw=0.6)
        axes[1].plot(sal[mask_dn] + cascade_s * i, z[mask_dn], "b-", lw=0.6)

    axes[0].set_xlabel("Temperature (°C, cascaded)")
    axes[0].set_ylabel("Depth (m)")
    axes[0].invert_yaxis()
    axes[0].grid(True, alpha=0.3)
    axes[0].set_title("Temperature cascade")
    axes[1].set_xlabel("Salinity (psu, cascaded)")
    axes[1].invert_yaxis()
    axes[1].grid(True, alpha=0.3)
    axes[1].set_title("Salinity cascade")
    fig.suptitle(f"Cascade CTD: {_run_name(ds)}")
    fig.tight_layout()
    _add_copy_button(fig)
    return fig


# ---------------------------------------------------------------------------
# optics_cast.m → optics_cast
# ---------------------------------------------------------------------------
def optics_cast(ds: xr.Dataset) -> plt.Figure:
    """Temperature, fluorometer, optical backscatter vs depth."""
    _need(ds, "m_depth")
    z = ds["m_depth"].values
    panels = []
    if "water_temp" in ds:
        panels.append(("Temperature (°C)", ds["water_temp"].values, "r"))
    if "m_fl_voltage" in ds:
        panels.append(("Fluorometer (mV)", ds["m_fl_voltage"].values, "g"))
    if "m_bs_voltage" in ds:
        panels.append(("Optical backscatter (mV)", ds["m_bs_voltage"].values, "b"))
    if not panels:
        raise KeyError("None of water_temp, m_fl_voltage, m_bs_voltage in dataset.")

    fig, axes = plt.subplots(1, len(panels), sharey=True, figsize=(4 * len(panels), 6))
    if len(panels) == 1:
        axes = [axes]
    for ax, (label, x, color) in zip(axes, panels):
        ax.plot(x, z, color=color, lw=0.6)
        ax.set_xlabel(label)
        ax.invert_yaxis()
        ax.grid(True, alpha=0.3)
    axes[0].set_ylabel("Depth (m)")
    fig.suptitle(f"Optical sensors: {_run_name(ds)}")
    fig.tight_layout()
    _add_copy_button(fig)
    return fig


# ---------------------------------------------------------------------------
# plt_depth_rate.m → depth_rate_check
# ---------------------------------------------------------------------------
def depth_rate_check(ds: xr.Dataset) -> plt.Figure:
    """Compare depth-derivative depth rate against DVL projected vertical velocity."""
    _need(ds, "d_rate", "m_depth")
    t = ds["time"].values
    z = ds["m_depth"].values
    fig, axes = plt.subplots(3, 1, sharex=True, figsize=(9, 8))
    axes[0].plot(t, ds["d_rate"].values, "r.", markersize=2, label="d/dt(m_depth)")
    if all(v in ds for v in ("m_rdi_z", "m_rdi_y", "m_pitch")):
        proj = (
            ds["m_rdi_z"].values * np.cos(ds["m_pitch"].values)
            - ds["m_rdi_y"].values * np.sin(ds["m_pitch"].values)
        )
        axes[0].plot(t, proj, "bx", markersize=2, alpha=0.5, label="DVL projected")
    axes[0].set_ylabel("Depth rate (m/s)")
    axes[0].legend(loc="best")
    axes[0].grid(True, alpha=0.3)

    axes[1].plot(t, z, "k-", lw=0.6)
    axes[1].set_ylabel("Depth (m)")
    axes[1].invert_yaxis()
    axes[1].grid(True, alpha=0.3)

    if all(v in ds for v in ("m_rdi_x", "m_rdi_y", "m_rdi_z")):
        axes[2].plot(t, ds["m_rdi_x"].values, "r.", markersize=2, label="x")
        axes[2].plot(t, ds["m_rdi_y"].values, "g.", markersize=2, label="y")
        axes[2].plot(t, ds["m_rdi_z"].values, "b.", markersize=2, label="z")
        axes[2].legend(loc="best")
    axes[2].set_ylabel("DVL velocities (m/s)")
    axes[2].set_xlabel("Time (s)")
    axes[2].grid(True, alpha=0.3)
    fig.tight_layout()
    _add_copy_button(fig)
    return fig


# ---------------------------------------------------------------------------
# Vertical-plane diagnostic — depth, pitch, elevator stacked
# ---------------------------------------------------------------------------
def vertical_diagnostic(ds: xr.Dataset) -> plt.Figure:
    """Three stacked panels for diagnosing depth-controller behaviour.

      Panel 1: commanded vs measured depth (m)
      Panel 2: commanded vs measured pitch (deg) + pitch limit envelope
      Panel 3: commanded vs measured elevator angle (deg)

    Reads the cause-and-effect chain top-to-bottom: depth error drives a
    pitch command (outer loop), pitch error drives an elevator command
    (inner loop), elevator deflection drives the vehicle. When something
    oscillates, the panel where commanded and measured first diverge is
    where the loop is misbehaving.

    Missing channels are skipped silently — at minimum we need m_depth,
    m_pitch, and one of m_elevator_angle / c_elevator_angle to draw
    anything useful.
    """
    _need(ds, "m_depth", "m_pitch")
    t = ds["time"].values

    fig, axes = plt.subplots(3, 1, sharex=True, figsize=(10, 9))

    # ---- Panel 1: depth -------------------------------------------------
    axes[0].plot(t, ds["m_depth"].values, "b-", lw=1.0, label="measured")
    if "c_depth" in ds:
        axes[0].plot(t, ds["c_depth"].values, "r--", lw=0.8, label="commanded")
    axes[0].set_ylabel("Depth (m)")
    axes[0].invert_yaxis()
    axes[0].grid(True, alpha=0.3)
    axes[0].legend(loc="best", fontsize=9)
    axes[0].set_title(f"Vertical-plane diagnostic: {_run_name(ds)}")

    # ---- Panel 2: pitch -------------------------------------------------
    pitch_meas_deg = np.degrees(ds["m_pitch"].values)
    axes[1].plot(t, pitch_meas_deg, "g-", lw=1.0, label="measured")
    if "c_pitch" in ds:
        axes[1].plot(t, np.degrees(ds["c_pitch"].values), "r--", lw=0.8,
                     label="commanded")
    # Show the pitch limit envelope if logged — uo_control_pitch_lim is in
    # degrees per edit_struct.c, no conversion needed.
    if "uo_control_pitch_lim" in ds:
        plim = ds["uo_control_pitch_lim"].values
        finite = np.isfinite(plim)
        if finite.any():
            limit = float(plim[finite][-1])  # take latest set value
            axes[1].axhline(limit, color="k", ls=":", lw=0.6, alpha=0.5,
                            label=f"±{limit:.0f}° limit")
            axes[1].axhline(-limit, color="k", ls=":", lw=0.6, alpha=0.5)
    axes[1].set_ylabel("Pitch (deg)")
    axes[1].grid(True, alpha=0.3)
    axes[1].legend(loc="best", fontsize=9)

    # ---- Panel 3: elevator ---------------------------------------------
    have_elev = False
    if "m_elevator_angle" in ds:
        axes[2].plot(t, np.degrees(ds["m_elevator_angle"].values), "m-",
                     lw=1.0, label="measured")
        have_elev = True
    if "c_elevator_angle" in ds:
        axes[2].plot(t, np.degrees(ds["c_elevator_angle"].values), "r--",
                     lw=0.8, label="commanded")
        have_elev = True
    if not have_elev:
        axes[2].text(0.5, 0.5, "No elevator data logged",
                     ha="center", va="center", transform=axes[2].transAxes,
                     color="gray")
    axes[2].set_ylabel("Elevator (deg)")
    axes[2].set_xlabel("Mission time (s)")
    axes[2].grid(True, alpha=0.3)
    if have_elev:
        axes[2].legend(loc="best", fontsize=9)

    fig.tight_layout()
    _add_copy_button(fig)
    return fig


# ---------------------------------------------------------------------------
# Thruster diagnostic — current cmd vs measured, propeller speed
# ---------------------------------------------------------------------------
def thruster(ds: xr.Dataset) -> plt.Figure:
    """Two stacked panels for diagnosing thruster behaviour.

      Panel 1: commanded vs measured thruster current (A)
      Panel 2: measured propeller speed (RPM)

    Note: the simulator commands current, not RPM. Propeller speed is
    what results from current + load. Thrust is computed inside
    thruster_force() and not logged as a sensor — if it becomes useful
    later, we can derive it from current using the same model.
    """
    have_curr = "c_thruster_current" in ds or "m_thruster_current" in ds
    have_rpm  = "m_propeller_speed" in ds
    if not (have_curr or have_rpm):
        raise KeyError("No thruster channels logged "
                       "(c_thruster_current, m_thruster_current, m_propeller_speed).")

    t = ds["time"].values
    n_panels = (1 if have_curr else 0) + (1 if have_rpm else 0)
    fig, axes = plt.subplots(n_panels, 1, sharex=True, figsize=(10, 3.2 * n_panels))
    if n_panels == 1:
        axes = [axes]
    panel = 0

    if have_curr:
        if "m_thruster_current" in ds:
            axes[panel].plot(t, ds["m_thruster_current"].values, "b-",
                             lw=1.0, label="measured")
        if "c_thruster_current" in ds:
            axes[panel].plot(t, ds["c_thruster_current"].values, "r--",
                             lw=0.8, label="commanded")
        axes[panel].set_ylabel("Thruster current (A)")
        axes[panel].grid(True, alpha=0.3)
        axes[panel].legend(loc="best", fontsize=9)
        axes[panel].set_title(f"Thruster: {_run_name(ds)}")
        panel += 1

    if have_rpm:
        axes[panel].plot(t, ds["m_propeller_speed"].values, "g-", lw=1.0,
                         label="measured")
        axes[panel].set_ylabel("Propeller speed (RPM)")
        axes[panel].grid(True, alpha=0.3)
        axes[panel].legend(loc="best", fontsize=9)
        if panel == 0:
            axes[panel].set_title(f"Thruster: {_run_name(ds)}")
        panel += 1

    axes[-1].set_xlabel("Mission time (s)")
    fig.tight_layout()
    _add_copy_button(fig)
    return fig


# ---------------------------------------------------------------------------
# Horizontal-plane diagnostic — plan view + heading/rudder time series
# ---------------------------------------------------------------------------
def horizontal_diagnostic(ds: xr.Dataset) -> plt.Figure:
    """Plan-view track with the heading/rudder time series below it.

      Top:    East vs North position (equal axis), start/end markers.
      Bottom: commanded vs measured heading (deg), and rudder (deg) on
              a secondary y-axis so its smaller range stays readable.

    Asks the same question vertical_diagnostic asks for depth:
      - Is the vehicle going where it was told to go?
      - Is the controller asking for sane rudder?
      - Is the rudder doing what it was asked?
    """
    _need(ds, "m_pos_n", "m_pos_e", "m_yaw")

    t = ds["time"].values
    fig = plt.figure(figsize=(10, 11))
    # gridspec gives us a square-ish track on top, narrower time series below
    gs = fig.add_gridspec(2, 1, height_ratios=[2, 1], hspace=0.25)

    # ---- Plan view ------------------------------------------------------
    ax_top = fig.add_subplot(gs[0])
    n = ds["m_pos_n"].values
    e = ds["m_pos_e"].values
    ax_top.plot(e, n, "b-", lw=0.8, label="track")
    ax_top.plot(e[0], n[0], "go", markersize=10, label="start")
    ax_top.plot(e[-1], n[-1], "rs", markersize=10, label="end")
    ax_top.set_xlabel("East (m)")
    ax_top.set_ylabel("North (m)")
    ax_top.set_aspect("equal", adjustable="datalim")
    ax_top.grid(True, alpha=0.3)
    ax_top.legend(loc="best", fontsize=9)
    ax_top.set_title(f"Horizontal-plane diagnostic: {_run_name(ds)}")

    # ---- Heading + rudder time series ----------------------------------
    ax_bot = fig.add_subplot(gs[1])
    yaw_deg = np.degrees(ds["m_yaw"].values)
    ax_bot.plot(t, yaw_deg, "g-", lw=1.0, label="m_yaw")
    if "c_heading" in ds:
        c_yaw_deg = np.degrees(ds["c_heading"].values)
        ax_bot.plot(t, c_yaw_deg, "r--", lw=0.8, label="commanded")
    ax_bot.set_ylabel("Heading (deg)")
    ax_bot.set_xlabel("Mission time (s)")
    ax_bot.grid(True, alpha=0.3)

    # Rudder on secondary axis (typical range ±15° vs heading 0–360°)
    if "m_rudder_angle" in ds or "c_rudder_angle" in ds:
        ax_rud = ax_bot.twinx()
        if "m_rudder_angle" in ds:
            ax_rud.plot(t, np.degrees(ds["m_rudder_angle"].values), "m-",
                        lw=0.8, alpha=0.7, label="rudder (meas)")
        if "c_rudder_angle" in ds:
            ax_rud.plot(t, np.degrees(ds["c_rudder_angle"].values), "k:",
                        lw=0.8, alpha=0.7, label="rudder (cmd)")
        ax_rud.set_ylabel("Rudder (deg)")
        # Combined legend on the bottom panel
        h1, l1 = ax_bot.get_legend_handles_labels()
        h2, l2 = ax_rud.get_legend_handles_labels()
        ax_bot.legend(h1 + h2, l1 + l2, loc="best", fontsize=9)
    else:
        ax_bot.legend(loc="best", fontsize=9)

    _add_copy_button(fig)
    return fig


# ---------------------------------------------------------------------------
# Battery / power (o2_power.m equivalent)
# ---------------------------------------------------------------------------
def power(ds: xr.Dataset) -> plt.Figure:
    """Battery voltage and current draw vs time."""
    fig, axes = plt.subplots(2, 1, sharex=True, figsize=(9, 6))
    t = ds["time"].values
    has_any = False
    if "m_km_voltage" in ds:
        axes[0].plot(t, ds["m_km_voltage"].values, "b-")
        axes[0].set_ylabel("Voltage (V)")
        has_any = True
    if "m_km_current" in ds:
        axes[1].plot(t, ds["m_km_current"].values, "r-")
        axes[1].set_ylabel("Current (A)")
        has_any = True
    if not has_any:
        raise KeyError("No m_km_voltage / m_km_current in dataset.")
    axes[1].set_xlabel("Mission time (s)")
    axes[0].grid(True, alpha=0.3)
    axes[1].grid(True, alpha=0.3)
    axes[0].set_title(f"Power: {_run_name(ds)}")
    fig.tight_layout()
    _add_copy_button(fig)
    return fig
