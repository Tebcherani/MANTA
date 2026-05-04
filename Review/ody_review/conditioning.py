"""
Vehicle data conditioning — ports of fix_time.m, fix_depth.m, fix_ctd.m.

Each function takes an xarray.Dataset (typically from obd.read_obd) and
returns a new Dataset with derived variables added. Functions are pure;
they don't mutate the input.

Domain knowledge preserved verbatim from the MATLAB:
  * Parosci 10-cycle update artifact removal — every 10th sample is
    actually a temperature update of the pressure sensor, not a depth
    sample. Detect by std(offset) low across the run.
  * Yoyo classification at |d_rate| > 0.1 m/s, with a >100-cycle
    minimum dive length to avoid spurious yoyos.
  * CTD depth correction for sensor offset (dx, dy) and pitch.
  * Conductivity-temperature lag from the pumped CTD.
"""
from __future__ import annotations

import numpy as np
import xarray as xr

from . import seawater

YOYO_DISCRIMINATOR = 0.1  # m/s, from fix_depth.m


def _smooth(x: np.ndarray, window: int) -> np.ndarray:
    """Centered moving-average smoothing, matching MATLAB smooth(x, win)."""
    x = np.asarray(x, dtype=float)
    if window <= 1 or x.size == 0:
        return x.copy()
    if window % 2 == 0:
        window += 1
    half = window // 2
    valid = np.isfinite(x)
    xc = np.where(valid, x, 0.0)
    kernel = np.ones(window) / window
    num = np.convolve(xc, kernel, mode="same")
    den = np.convolve(valid.astype(float), kernel, mode="same")
    out = np.full_like(x, np.nan)
    nz = den > 0
    out[nz] = num[nz] / den[nz]
    return out


def add_time(ds: xr.Dataset) -> xr.Dataset:
    """Ensure mission elapsed `time` coordinate is present (fix_time.m)."""
    if "time" in ds.coords:
        return ds
    if "m_present_time" not in ds:
        raise KeyError("Cannot derive time without m_present_time.")
    t = ds["m_present_time"].values.astype(float)
    finite = np.isfinite(t)
    t0 = t[finite][0] if finite.any() else 0.0
    return ds.assign_coords(time=("cycle", t - t0))


def add_depth_processing(
    ds: xr.Dataset,
    yoyo_discriminator: float = YOYO_DISCRIMINATOR,
    smooth_window: int = 11,
    parosci_artifact_removal: bool = True,
) -> xr.Dataset:
    """Port of fix_depth.m. Adds: m_depth_clean, d_rate, yoyo, going_up, going_down.

    yoyo: integer yoyo number (NaN outside the yoyo window)
    going_up: bool array, True where d_rate < -yoyo_discriminator
    going_down: bool array, True where d_rate > yoyo_discriminator
    """
    ds = add_time(ds)
    if "m_depth" not in ds:
        raise KeyError("m_depth missing — cannot do depth processing.")

    d = ds["m_depth"].values.astype(float).copy()
    n = d.size
    # fix_depth.m shifts depth back one cycle to align with measurement
    if n > 1:
        d[: n - 1] = d[1:n]

    if parosci_artifact_removal and n > 100:
        diffs = np.abs(np.diff(d))
        cyc = np.flatnonzero(diffs < 0.0015)
        if cyc.size > 1:
            tens = np.flatnonzero(np.abs(np.diff(cyc) - 10) < 0.003)
            if tens.size > 10:
                offset = cyc[tens] % 10
                if np.std(offset) < 0.2:
                    d_off = int(round(offset.mean()))
                    bad = 10 * np.arange(n // 10) + d_off + 1
                    bad = bad[bad < n]
                    d[bad] = np.nan

    t = ds["time"].values.astype(float)
    valid = np.isfinite(d) & np.isfinite(t)
    d_rate = np.full(n, np.nan)
    if valid.sum() > 1:
        idx = np.flatnonzero(valid)
        dr = np.diff(d[idx]) / np.diff(t[idx])
        # MATLAB pattern: append a trailing 0 so length matches; we pad NaN
        d_rate[idx[:-1]] = dr
        d_rate = _smooth(d_rate, smooth_window)

    # Yoyo classification — only attempt if we ever go below 10 m
    yoyo = np.full(n, np.nan)
    deep = np.flatnonzero(d > 10)
    if deep.size > 1:
        ind10f, ind10l = deep[0], deep[-1]
        # earliest = last sample before first dive where d_rate is below
        # the discriminator (i.e. surface period before descent)
        before = np.flatnonzero(d_rate[:ind10f] < yoyo_discriminator)
        earliest = before[-1] if before.size else 0
        # latest = first sample after last deep point where d_rate >= -disc
        # (i.e. surface period after ascent)
        after = np.flatnonzero(d_rate[ind10l:] > -yoyo_discriminator)
        latest = (ind10l + after[0]) if after.size else (n - 1)
        window = slice(earliest, latest + 1)
        # downward zero-crossings of d_rate within the dive window
        dive_mask = d_rate[window] > yoyo_discriminator
        transitions = np.flatnonzero(np.diff(dive_mask.astype(int)) == 1)
        # require >100 cycles between yoyos to ignore noise
        if transitions.size:
            keep = np.concatenate(([0], np.flatnonzero(np.diff(transitions) > 100) + 1))
            transitions = transitions[keep]
            dwn = transitions + earliest
            if dwn.size >= 2:
                yoyo[: dwn[0] + 1] = 1
                for i in range(1, dwn.size):
                    yoyo[dwn[i - 1] : dwn[i] + 1] = i + 1
                yoyo[dwn[-1] :] = dwn.size + 1

    going_up = (d_rate < -yoyo_discriminator) & np.isfinite(d_rate)
    going_down = (d_rate > yoyo_discriminator) & np.isfinite(d_rate)

    out = ds.copy()
    out["m_depth_clean"] = ("cycle", d)
    out["d_rate"] = ("cycle", d_rate)
    out["d_rate"].attrs["units"] = "m/s"
    out["yoyo"] = ("cycle", yoyo)
    out["going_up"] = ("cycle", going_up)
    out["going_down"] = ("cycle", going_down)
    return out


def add_ctd_processing(
    ds: xr.Dataset,
    s_lag: int = 5,
    sensor_offset_x: float = 0.8,
    sensor_offset_y: float = 0.25,
) -> xr.Dataset:
    """Port of fix_ctd.m + the sensor-offset / lag corrections from CTD_process.m.

    Adds: salinity, sound_velocity, density, ctd_depth (decibars).

    s_lag: cycles of conductivity-vs-temperature lag from the pumped CTD.
    sensor_offset_x, _y: meters from pressure sensor to CTD cell, in vehicle
                         frame. Used to correct CTD depth for pitch.
    """
    needed = ("water_temp", "water_cond", "m_depth")
    for name in needed:
        if name not in ds:
            raise KeyError(f"add_ctd_processing requires {name!r}.")

    pitch = ds["m_pitch"].values if "m_pitch" in ds else np.zeros(ds.sizes["cycle"])
    depth = ds["m_depth"].values
    cond = ds["water_cond"].values.astype(float)
    temp = ds["water_temp"].values.astype(float)

    ctd_depth_m = depth - (
        sensor_offset_x * np.sin(pitch) + sensor_offset_y * np.cos(pitch)
    )
    # CTD pumping lag — shift depth forward by s_lag cycles
    if s_lag > 0 and ctd_depth_m.size > s_lag:
        ctd_depth_m = np.concatenate(
            [np.full(s_lag, ctd_depth_m[s_lag]), ctd_depth_m[: -s_lag]]
        )
    # m of freshwater → decibars (CTD_process.m, JWB 5/2/95)
    ctd_depth_dbar = ctd_depth_m * 3.281 * 12.0 * 2.49082e-2

    if s_lag > 0 and cond.size > s_lag:
        cond_aligned = cond[s_lag:]
        temp_aligned = temp[: -s_lag] if s_lag else temp
        depth_aligned = ctd_depth_dbar[: -s_lag] if s_lag else ctd_depth_dbar
        # pad with NaN at the head so the result stays length-N
        sal = np.full_like(temp, np.nan)
        svel = np.full_like(temp, np.nan)
        dens = np.full_like(temp, np.nan)
        salp, svelp = seawater.salinity_and_sound_velocity(
            cond_aligned, temp_aligned, depth_aligned
        )
        densp = seawater.density(salp, temp_aligned, depth_aligned)
        sal[s_lag:] = salp
        svel[s_lag:] = svelp
        dens[s_lag:] = densp
    else:
        sal, svel = seawater.salinity_and_sound_velocity(cond, temp, ctd_depth_dbar)
        dens = seawater.density(sal, temp, ctd_depth_dbar)

    out = ds.copy()
    out["salinity"] = ("cycle", sal)
    out["salinity"].attrs["units"] = "psu"
    out["sound_velocity"] = ("cycle", svel)
    out["sound_velocity"].attrs["units"] = "m/s"
    out["density"] = ("cycle", dens)
    out["density"].attrs["units"] = "kg/m^3"
    out["ctd_depth"] = ("cycle", ctd_depth_dbar)
    out["ctd_depth"].attrs["units"] = "dbar"
    return out


def condition(ds: xr.Dataset) -> xr.Dataset:
    """Run the standard conditioning pipeline used by review.m."""
    ds = add_time(ds)
    ds = add_depth_processing(ds)
    if all(v in ds for v in ("water_temp", "water_cond")):
        ds = add_ctd_processing(ds)
    return ds
