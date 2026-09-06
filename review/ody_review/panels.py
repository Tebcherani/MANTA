"""
Extended diagnostic plot panels for the Odyssey AUV review GUI.

Each function accepts a conditioned xarray.Dataset (from conditioning.condition())
and returns a matplotlib Figure suitable for embedding in a Qt tab or standalone
display.  All panels share the fault-overlay helper _shade_faults() which tints
x-axis spans red wherever m_fault_active > 0.

New panels (complement the existing functions in plots.py):
  vertical_control()   depth / altitude / pitch+elevator / roll  (4 rows)
  heading_panel()      heading + rudder time series              (2 rows)
  fault_timeline()     fault-active timeline with depth context  (1-2 rows)
  behavior_axis_waterfall()
                       behavior priority vs time by controlled axis
"""
from __future__ import annotations

import numpy as np
import matplotlib.pyplot as plt
import xarray as xr

from .plots import _add_copy_button, _run_name, _need


_BEHAVIOR_NAMES = {
    0: "mission_timer",
    1: "acquire_heading",
    2: "ascend",
    3: "broadcast",
    4: "chirp_fin",
    5: "depth_envelope",
    6: "descend",
    7: "dock",
    8: "undock",
    9: "whoi_homing",
    10: "edc_homing",
    11: "set_elevator",
    12: "set_rudder",
    13: "setmode",
    14: "setpoint",
    15: "setvector",
    16: "survey_dead_reckon",
    17: "remote_delay",
    18: "waypoint_2d",
    19: "wiggle",
    20: "launch",
    21: "yo_yo",
}


# ---------------------------------------------------------------------------
# Fault-region shading — used by every panel
# ---------------------------------------------------------------------------

def _shade_faults(ax: plt.Axes, t: np.ndarray, ds: xr.Dataset) -> None:
    """Shade x-axis spans where m_fault_active > 0 with a light red tint.

    Adds a single legend entry "fault active" for the first span so the
    label appears exactly once per axes.
    """
    if "m_fault_active" not in ds:
        return
    fa = ds["m_fault_active"].values
    active = np.isfinite(fa) & (fa > 0)
    if not active.any():
        return

    padded = np.concatenate([[False], active, [False]])
    rises = np.where(np.diff(padded.astype(np.int8)) ==  1)[0]
    falls = np.where(np.diff(padded.astype(np.int8)) == -1)[0]
    first = True
    for s, e in zip(rises, falls):
        e = min(e, len(t) - 1)
        ax.axvspan(t[s], t[e], alpha=0.13, color="red",
                   label="fault active" if first else None, zorder=0)
        first = False


# ---------------------------------------------------------------------------
# Vertical-plane control panel  (4 rows)
# ---------------------------------------------------------------------------

def vertical_control(ds: xr.Dataset) -> plt.Figure:
    """Four-panel vertical-plane control overview.

    Row 1  Depth (m)       measured + commanded
    Row 2  Altitude (m)    measured altimeter reading
    Row 3  Pitch (deg)     measured + commanded + error;
                           elevator on secondary y-axis
    Row 4  Roll (deg)      measured

    Fault-active cycles are shaded red on all rows.
    Missing channels are indicated with a text message rather than an error.
    """
    _need(ds, "m_depth")
    if "time" not in ds.coords:
        raise ValueError("No 'time' coordinate — run conditioning.condition() first.")

    t   = ds["time"].values
    run = _run_name(ds)

    fig, axes = plt.subplots(4, 1, sharex=True, figsize=(11, 11))

    # ---- Row 1: Depth -------------------------------------------------------
    ax = axes[0]
    ax.plot(t, ds["m_depth"].values, color="steelblue", lw=1.2, label="measured")
    if "c_depth" in ds:
        ax.plot(t, ds["c_depth"].values, "r--", lw=0.9, label="commanded")
    ax.invert_yaxis()
    ax.set_ylabel("Depth (m)")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="upper right", fontsize=8)
    ax.set_title(f"Vertical-plane control: {run}")
    _shade_faults(ax, t, ds)

    # ---- Row 2: Altitude ----------------------------------------------------
    ax = axes[1]
    if "m_altitude" in ds:
        ax.plot(t, ds["m_altitude"].values, color="seagreen", lw=1.2,
                label="altitude")
        ax.legend(loc="upper right", fontsize=8)
    else:
        ax.text(0.5, 0.5, "No altimeter data logged",
                ha="center", va="center", transform=ax.transAxes,
                color="gray", fontsize=11)
    ax.set_ylabel("Altitude (m)")
    ax.grid(True, alpha=0.3)
    _shade_faults(ax, t, ds)

    # ---- Row 3: Pitch + elevator --------------------------------------------
    ax = axes[2]
    if "m_pitch" in ds:
        pitch_deg = np.degrees(ds["m_pitch"].values)
        ax.plot(t, pitch_deg, color="steelblue", lw=1.2, label="pitch meas")

        if "c_pitch" in ds:
            cmd_deg = np.degrees(ds["c_pitch"].values)
            ax.plot(t, cmd_deg, "r--", lw=0.9, label="pitch cmd")
            ax.plot(t, cmd_deg - pitch_deg, color="darkorange", lw=0.7,
                    ls=":", alpha=0.85, label="error")

        if "uo_control_pitch_lim" in ds:
            plim_arr = ds["uo_control_pitch_lim"].values
            fin = np.isfinite(plim_arr)
            if fin.any():
                plim = float(plim_arr[fin][-1])
                ax.axhline( plim, color="k", ls=":", lw=0.6, alpha=0.5)
                ax.axhline(-plim, color="k", ls=":", lw=0.6, alpha=0.5,
                           label=f"±{plim:.0f}° limit")
    else:
        ax.text(0.5, 0.5, "No pitch data logged",
                ha="center", va="center", transform=ax.transAxes,
                color="gray", fontsize=11)

    ax.axhline(0, color="k", lw=0.4, alpha=0.3)
    ax.set_ylabel("Pitch (deg)")
    ax.grid(True, alpha=0.3)

    # Elevator on secondary axis (range typically ±15° vs pitch ±30°)
    have_elev = "m_elevator_angle" in ds or "c_elevator_angle" in ds
    if have_elev:
        ax2 = ax.twinx()
        if "m_elevator_angle" in ds:
            ax2.plot(t, np.degrees(ds["m_elevator_angle"].values),
                     color="0.50", lw=0.9, label="elevator meas")
        if "c_elevator_angle" in ds:
            ax2.plot(t, np.degrees(ds["c_elevator_angle"].values),
                     color="purple", lw=0.8, ls="--", label="elevator cmd")
        ax2.set_ylabel("Elevator (deg)")
        h1, l1 = ax.get_legend_handles_labels()
        h2, l2 = ax2.get_legend_handles_labels()
        ax.legend(h1 + h2, l1 + l2, loc="upper right", fontsize=8)
    else:
        ax.legend(loc="upper right", fontsize=8)
    _shade_faults(ax, t, ds)

    # ---- Row 4: Roll --------------------------------------------------------
    ax = axes[3]
    if "m_roll" in ds:
        ax.plot(t, np.degrees(ds["m_roll"].values), color="purple", lw=1.2,
                label="roll")
        ax.axhline(0, color="k", lw=0.4, alpha=0.3)
        ax.legend(loc="upper right", fontsize=8)
    else:
        ax.text(0.5, 0.5, "No roll data logged",
                ha="center", va="center", transform=ax.transAxes,
                color="gray", fontsize=11)
    ax.set_ylabel("Roll (deg)")
    ax.set_xlabel("Mission time (s)")
    ax.grid(True, alpha=0.3)
    _shade_faults(ax, t, ds)

    fig.tight_layout()
    _add_copy_button(fig)
    return fig


# ---------------------------------------------------------------------------
# Heading and rudder control panel  (2 rows)
# ---------------------------------------------------------------------------

def heading_panel(ds: xr.Dataset) -> plt.Figure:
    """Two-panel heading control overview.

    Row 1  Heading (deg, 0-360)   measured + commanded
    Row 2  Rudder  (deg)          measured + commanded

    Fault-active cycles are shaded red on both rows.
    """
    _need(ds, "m_yaw")
    if "time" not in ds.coords:
        raise ValueError("No 'time' coordinate — run conditioning.condition() first.")

    t   = ds["time"].values
    run = _run_name(ds)

    fig, axes = plt.subplots(2, 1, sharex=True, figsize=(11, 6))

    # ---- Row 1: Heading -----------------------------------------------------
    ax = axes[0]
    hdg = np.degrees(ds["m_yaw"].values) % 360
    ax.plot(t, hdg, color="steelblue", lw=1.2, label="heading meas")

    # c_heading (layered-control command) or m_yaw_d (setpoint) — try both
    for var, lbl in [("c_heading", "heading cmd"), ("m_yaw_d", "heading setpoint")]:
        if var in ds:
            ax.plot(t, np.degrees(ds[var].values) % 360, "r--", lw=0.9,
                    label=lbl)
            break

    ax.set_ylabel("Heading (deg)")
    ax.set_ylim(-5, 365)
    ax.set_yticks(range(0, 361, 45))
    ax.grid(True, alpha=0.3)
    ax.legend(loc="upper right", fontsize=8)
    ax.set_title(f"Heading control: {run}")
    _shade_faults(ax, t, ds)

    # ---- Row 2: Rudder -------------------------------------------------------
    ax = axes[1]
    have_rudder = False
    if "m_rudder_angle" in ds:
        ax.plot(t, np.degrees(ds["m_rudder_angle"].values),
                color="mediumpurple", lw=1.2, label="rudder meas")
        have_rudder = True
    if "c_rudder_angle" in ds:
        ax.plot(t, np.degrees(ds["c_rudder_angle"].values),
                "r--", lw=0.9, label="rudder cmd")
        have_rudder = True

    if not have_rudder:
        ax.text(0.5, 0.5, "No rudder data logged",
                ha="center", va="center", transform=ax.transAxes,
                color="gray", fontsize=11)
    ax.axhline(0, color="k", lw=0.4, alpha=0.3)
    ax.set_ylabel("Rudder (deg)")
    ax.set_xlabel("Mission time (s)")
    ax.grid(True, alpha=0.3)
    if have_rudder:
        ax.legend(loc="upper right", fontsize=8)
    _shade_faults(ax, t, ds)

    fig.tight_layout()
    _add_copy_button(fig)
    return fig


# ---------------------------------------------------------------------------
# Fault injection timeline  (1-2 rows)
# ---------------------------------------------------------------------------

def fault_timeline(ds: xr.Dataset) -> plt.Figure:
    """Fault injection timeline with depth context.

    Row 1  m_fault_active — count of fault rules that fired each cycle.
           Cycles with active faults are filled red; the fault file name
           is annotated in the upper-left corner.
    Row 2  Depth vs time (for mission context) with matching red shading.

    If m_fault_active is absent (pre-2024 files without the fault engine),
    an explanatory message replaces the plot.
    """
    if "time" not in ds.coords:
        raise ValueError("No 'time' coordinate — run conditioning.condition() first.")

    t   = ds["time"].values
    run = _run_name(ds)

    if "m_fault_active" not in ds:
        fig, ax = plt.subplots(figsize=(11, 3))
        ax.text(0.5, 0.5,
                "m_fault_active not found in this dataset.\n"
                "Fault tracking requires simulator builds from 2024 or later.",
                ha="center", va="center", transform=ax.transAxes,
                color="gray", fontsize=11)
        ax.set_title(f"Fault timeline: {run}")
        ax.axis("off")
        fig.tight_layout()
        return fig

    fa       = ds["m_fault_active"].values
    fa_clean = np.where(np.isfinite(fa), fa, 0.0)
    n_rows   = 2 if "m_depth" in ds else 1

    fig, axes = plt.subplots(n_rows, 1, sharex=True,
                              figsize=(11, 3.5 * n_rows))
    if n_rows == 1:
        axes = [axes]

    # ---- Row 1: m_fault_active ----------------------------------------------
    ax = axes[0]
    ax.fill_between(t, 0, fa_clean, where=(fa_clean > 0),
                    color="red", alpha=0.55, label="fault active")
    ax.fill_between(t, 0, fa_clean, where=(fa_clean == 0),
                    color="steelblue", alpha=0.18)
    ax.step(t, fa_clean, color="darkred", lw=0.9, where="mid")
    ax.set_ylabel("Rules fired (#)")
    ax.set_ylim(bottom=0)
    ax.yaxis.set_major_locator(plt.MaxNLocator(integer=True))
    ax.grid(True, alpha=0.3)
    ax.legend(loc="upper right", fontsize=8)
    ax.set_title(f"Fault injection timeline: {run}")

    fault_file = ds.attrs.get("fault_file", "")
    if fault_file:
        ax.text(0.01, 0.96, f"Fault file: {fault_file}",
                transform=ax.transAxes, fontsize=9, va="top",
                color="darkred",
                bbox=dict(boxstyle="round,pad=0.25", fc="lightyellow",
                          ec="orange", alpha=0.85))

    # ---- Row 2: Depth context -----------------------------------------------
    if n_rows > 1:
        ax2 = axes[1]
        ax2.plot(t, ds["m_depth"].values, color="steelblue", lw=1.0,
                 label="depth")
        ax2.invert_yaxis()
        ax2.set_ylabel("Depth (m)")
        ax2.grid(True, alpha=0.3)
        ax2.legend(loc="upper right", fontsize=8)
        _shade_faults(ax2, t, ds)

    axes[-1].set_xlabel("Mission time (s)")
    fig.tight_layout()
    _add_copy_button(fig)
    return fig


# ---------------------------------------------------------------------------
# Behavior-axis ownership waterfall
# ---------------------------------------------------------------------------

def behavior_axis_waterfall(ds: xr.Dataset) -> plt.Figure:
    """Show which behavior priority owns depth, heading, and speed over time."""
    needed = (
        "c_depth_priority", "c_depth_behavior",
        "c_heading_priority", "c_heading_behavior",
        "c_speed_priority", "c_speed_behavior",
    )
    _need(ds, *needed)
    if "time" not in ds.coords:
        raise ValueError("No 'time' coordinate - run conditioning.condition() first.")

    t = ds["time"].values
    run = _run_name(ds)

    axes = [
        ("Depth", "c_depth_priority", "c_depth_behavior", "tab:blue", -0.18),
        ("Heading", "c_heading_priority", "c_heading_behavior", "tab:orange", 0.0),
        ("Speed", "c_speed_priority", "c_speed_behavior", "tab:green", 0.18),
    ]

    fig, ax = plt.subplots(figsize=(12, 6.5))
    priorities = []
    label_by_priority = {}

    for axis_name, pvar, bvar, color, offset in axes:
        p = ds[pvar].values.astype(float)
        b = ds[bvar].values.astype(float)
        valid = np.isfinite(p) & np.isfinite(b) & (p >= 0) & (b >= 0)
        if not valid.any():
            continue

        y = p[valid]
        ax.scatter(
            t[valid],
            y + offset,
            s=7,
            marker="s",
            color=color,
            alpha=0.85,
            linewidths=0,
            label=axis_name,
        )

        for pri, beh in zip(p[valid], b[valid]):
            pri_i = int(round(pri))
            beh_i = int(round(beh))
            priorities.append(pri_i)
            name = _BEHAVIOR_NAMES.get(beh_i, f"behavior {beh_i}")
            label_by_priority.setdefault(pri_i, f"{pri_i}: {name}")

    if not priorities:
        ax.text(
            0.5,
            0.5,
            "No behavior-axis ownership data in this file.\n"
            "Re-run the simulator with a build that logs c_*_behavior sensors.",
            ha="center",
            va="center",
            transform=ax.transAxes,
            color="gray",
            fontsize=11,
        )
        ax.axis("off")
        fig.tight_layout()
        return fig

    ticks = sorted(set(priorities))
    ax.set_yticks(ticks)
    ax.set_yticklabels([label_by_priority.get(p, str(p)) for p in ticks])
    ax.invert_yaxis()
    ax.set_xlabel("Mission time (s)")
    ax.set_ylabel("Behavior priority")
    ax.set_title(f"Behavior control-axis ownership: {run}")
    ax.grid(True, axis="x", alpha=0.25)
    ax.grid(True, axis="y", alpha=0.15)
    ax.legend(loc="upper right", fontsize=9, ncol=3)

    if "m_depth" in ds:
        ax2 = ax.twinx()
        ax2.plot(t, ds["m_depth"].values, color="0.35", lw=0.8, alpha=0.35)
        ax2.invert_yaxis()
        ax2.set_ylabel("Depth (m)", color="0.35")
        ax2.tick_params(axis="y", colors="0.35")

    fig.tight_layout()
    _add_copy_button(fig)
    return fig
