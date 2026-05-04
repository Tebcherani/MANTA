"""
Main review entry point — port of review.m.

Usage:
    from ody_review import review
    figs = review("Data/X2609501.obd", opts=[2, 3, 8, 9])

Options (matching review.m):
    1   timing
    2   vertical plane (depth, pitch)
    3   heading
    4   navigation
    5   LBL fixes
    6   altimeter
    7   battery / power
    8   CTD time series
    9   T, S, sound velocity vs depth
    10  fluorometer & optical backscatter
    11  RDI / depth-rate cross-check
    12  vertical-plane diagnostic (depth, pitch, elevator — cmd vs measured)
    13  thruster (current cmd vs measured, RPM)
    14  horizontal-plane diagnostic (plan view + heading, rudder vs time)
    "all"   run every applicable option

Plots that need data not present in the file are skipped quietly.
"""
from __future__ import annotations

from pathlib import Path
from typing import Iterable

import matplotlib.pyplot as plt
import xarray as xr

from . import obd, conditioning, plots

DEFAULT_OPTS: tuple[int, ...] = (2, 3)


def review(
    path: str | Path,
    opts: Iterable[int] | str = DEFAULT_OPTS,
    *,
    show: bool = True,
) -> dict[str, plt.Figure]:
    """Read, condition, and plot an .obd file.

    Returns a dict mapping plot name -> matplotlib Figure so callers can
    save or modify them. With show=True (default), figures display
    interactively at the end.
    """
    ds = obd.read_obd(path)
    ds = conditioning.condition(ds)

    if opts == "all":
        opts_set = set(range(1, 15))
    else:
        opts_set = set(opts)

    figs: dict[str, plt.Figure] = {}

    def _try(name: str, fn, *args, **kwargs):
        try:
            figs[name] = fn(*args, **kwargs)
        except (KeyError, ValueError) as e:
            print(f"  [skip] {name}: {e}")

    if 2 in opts_set:
        _try("vertical_plane", plots.vertical_plane, ds)
    if 3 in opts_set and "m_yaw" in ds:
        _try("heading", _heading_plot, ds)
    if 4 in opts_set:
        _try("navplot", plots.navplot, ds)
    if 7 in opts_set:
        _try("power", plots.power, ds)
    if 8 in opts_set:
        _try("ctd_time_series", plots.ctd_time_series, ds)
        _try("ctd_profiles", plots.ctd_profiles, ds)
    if 9 in opts_set:
        _try("ts_cast", plots.ts_cast, ds)
        _try("cascade_ctd", plots.cascade_ctd, ds)
    if 10 in opts_set:
        _try("optics_cast", plots.optics_cast, ds)
    if 11 in opts_set:
        _try("depth_rate_check", plots.depth_rate_check, ds)
    if 12 in opts_set:
        _try("vertical_diagnostic", plots.vertical_diagnostic, ds)
    if 13 in opts_set:
        _try("thruster", plots.thruster, ds)
    if 14 in opts_set:
        _try("horizontal_diagnostic", plots.horizontal_diagnostic, ds)

    if show and figs:
        plt.show()
    return figs


def _heading_plot(ds: xr.Dataset) -> plt.Figure:
    import numpy as np
    fig, ax = plt.subplots(figsize=(9, 4))
    t = ds["time"].values
    ax.plot(t, np.degrees(ds["m_yaw"].values), "b-", lw=0.6, label="m_yaw")
    if "m_yaw_d" in ds:
        ax.plot(t, np.degrees(ds["m_yaw_d"].values), "r--", lw=0.6, label="commanded")
        ax.legend(loc="best")
    ax.set_xlabel("Mission time (s)")
    ax.set_ylabel("Heading (deg)")
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    return fig


def main() -> None:  # CLI entry
    import argparse, sys

    parser = argparse.ArgumentParser(description="Review an Odyssey .obd file.")
    parser.add_argument("path", help="path to .obd file")
    parser.add_argument(
        "--opts",
        default="2,3",
        help="comma-separated options (e.g. '2,3,8,9' or 'all'); see module docstring",
    )
    parser.add_argument("--save", help="directory to save figures (PNG); skips display")
    args = parser.parse_args()

    if args.opts == "all":
        opts: Iterable[int] | str = "all"
    else:
        opts = [int(x) for x in args.opts.split(",") if x.strip()]

    figs = review(args.path, opts=opts, show=args.save is None)
    if args.save:
        out = Path(args.save)
        out.mkdir(parents=True, exist_ok=True)
        stem = Path(args.path).stem
        for name, fig in figs.items():
            target = out / f"{stem}_{name}.png"
            fig.savefig(target, dpi=120, bbox_inches="tight")
            print(f"saved {target}")
        sys.exit(0)


if __name__ == "__main__":
    main()
