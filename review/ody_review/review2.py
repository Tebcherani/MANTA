"""
Compare two Odyssey .obd files side-by-side.

This module reuses the existing `ody_review` conditioning and plotting stack,
but it produces combined figures where run 1 is drawn in blue and run 2 is
drawn in red.

Usage:
    from ody_review.review2 import compare
    figs = compare("run1.obd", "run2.obd", opts=[2, 4, 12])
"""
from __future__ import annotations

from pathlib import Path
from typing import Iterable

import matplotlib.pyplot as plt
import numpy as np
import xarray as xr
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.backends.backend_qt5agg import NavigationToolbar2QT as NavigationToolbar
from PyQt5 import QtCore, QtWidgets

from . import plots
from . import conditioning
from . import obd
from .plots import _add_copy_button, _run_name

DEFAULT_OPTS: tuple[int, ...] = (2, 3)

_PANELS: list[tuple[str, object, bool, int]] = [
    ("Vertical Plane", None, True, 2),
    ("Heading", None, True, 3),
    ("Navigation", None, True, 4),
    ("Power / Battery", None, False, 7),
    ("CTD Time Series", None, False, 8),
    ("T-S Cast", None, False, 9),
    ("Depth Rate", None, False, 11),
    ("Vertical Diag.", None, False, 12),
    ("Thruster", None, False, 13),
    ("Horiz. Diag.", None, False, 14),
]


def _find_sdata() -> Path:
    candidates = [
        Path(__file__).resolve().parent.parent.parent / "output" / "Data",
        Path.cwd() / "output" / "Data",
        Path.cwd().parent / "output" / "Data",
    ]

    for candidate in candidates:
        if candidate.is_dir():
            return candidate

    raise FileNotFoundError(
        "Could not locate output/Data."
    )


def _list_obd_files(sdata: Path) -> list[Path]:
    if not sdata.is_dir():
        return []
    files = list(sdata.glob("*.obd"))
    files.sort(key=lambda p: p.stat().st_mtime, reverse=True)
    return files


class _FigureTab(QtWidgets.QWidget):
    def __init__(self, fig: plt.Figure, parent=None):
        super().__init__(parent)
        self._fig = fig
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)
        canvas = FigureCanvas(fig)
        toolbar = NavigationToolbar(canvas, self)
        layout.addWidget(toolbar)
        layout.addWidget(canvas)


class _CompareSidebar(QtWidgets.QWidget):
    plots_requested = QtCore.pyqtSignal()

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFixedWidth(330)
        root = QtWidgets.QVBoxLayout(self)
        root.setContentsMargins(8, 10, 8, 8)
        root.setSpacing(6)

        root.addWidget(_bold_label("Compare Files"))
        self._combo1 = QtWidgets.QComboBox()
        self._combo2 = QtWidgets.QComboBox()
        self._combo1.setToolTip("First run, plotted in blue")
        self._combo2.setToolTip("Second run, plotted in red")
        root.addWidget(QtWidgets.QLabel("Run 1 (blue):"))
        root.addWidget(self._combo1)
        root.addWidget(QtWidgets.QLabel("Run 2 (red):"))
        root.addWidget(self._combo2)

        btn_row = QtWidgets.QHBoxLayout()
        refresh = QtWidgets.QPushButton("Refresh")
        generate = QtWidgets.QPushButton("Generate")
        generate.setDefault(True)
        generate.setStyleSheet("font-weight: bold;")
        btn_row.addWidget(refresh)
        btn_row.addWidget(generate)
        root.addLayout(btn_row)

        panels_box = QtWidgets.QGroupBox("Plot Panels")
        panel_layout = QtWidgets.QVBoxLayout(panels_box)
        select_row = QtWidgets.QHBoxLayout()
        all_btn = QtWidgets.QPushButton("All")
        none_btn = QtWidgets.QPushButton("None")
        select_row.addWidget(all_btn)
        select_row.addWidget(none_btn)
        panel_layout.addLayout(select_row)
        self._checks: dict[str, QtWidgets.QCheckBox] = {}
        for name, _, default, _ in _PANELS:
            check = QtWidgets.QCheckBox(name)
            check.setChecked(default)
            panel_layout.addWidget(check)
            self._checks[name] = check
        root.addWidget(panels_box)
        root.addStretch(1)

        refresh.clicked.connect(self.refresh)
        generate.clicked.connect(self.plots_requested)
        all_btn.clicked.connect(lambda: self._set_all(True))
        none_btn.clicked.connect(lambda: self._set_all(False))
        self.refresh()

    def refresh(self) -> None:
        files = _list_obd_files(_find_sdata())
        for combo in (self._combo1, self._combo2):
            combo.clear()
            for path in files:
                combo.addItem(path.name, userData=str(path))
        if len(files) > 1:
            self._combo2.setCurrentIndex(1)

    def selected_paths(self) -> tuple[str | None, str | None]:
        return self._combo1.currentData(), self._combo2.currentData()

    def selected_opts(self) -> list[int]:
        opts = []
        for name, _, _, opt in _PANELS:
            if self._checks[name].isChecked():
                opts.append(opt)
        return opts

    def _set_all(self, state: bool) -> None:
        for check in self._checks.values():
            check.setChecked(state)


class CompareMainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Odyssey Review2 Compare")
        self.resize(1440, 920)

        splitter = QtWidgets.QSplitter(QtCore.Qt.Horizontal)
        self.setCentralWidget(splitter)
        self._sidebar = _CompareSidebar()
        self._tabs = QtWidgets.QTabWidget()
        self._tabs.setDocumentMode(True)
        splitter.addWidget(self._sidebar)
        splitter.addWidget(self._tabs)
        splitter.setSizes([330, 1110])
        splitter.setStretchFactor(0, 0)
        splitter.setStretchFactor(1, 1)
        self._sidebar.plots_requested.connect(self._generate_plots)
        self.statusBar().showMessage(f"Ready - scanning {_find_sdata()}")

    def _generate_plots(self) -> None:
        path1, path2 = self._sidebar.selected_paths()
        if not path1 or not path2:
            QtWidgets.QMessageBox.warning(self, "Select files", "Select two OBD files to compare.")
            return
        if path1 == path2:
            QtWidgets.QMessageBox.warning(self, "Select files", "Select two different OBD files.")
            return

        for i in range(self._tabs.count()):
            widget = self._tabs.widget(i)
            if hasattr(widget, "_fig"):
                plt.close(widget._fig)
        self._tabs.clear()

        opts = self._sidebar.selected_opts()
        if not opts:
            return
        try:
            figs = compare(path1, path2, opts=opts, show=False)
        except Exception as exc:
            QtWidgets.QMessageBox.critical(self, "Compare error", str(exc))
            return

        generated = set(figs)
        opt_names = {
            2: ["vertical_plane"],
            3: ["heading"],
            4: ["navplot"],
            7: ["power"],
            8: ["ctd_time_series", "ctd_profiles"],
            9: ["ts_cast", "cascade_ctd"],
            11: ["depth_rate_check"],
            12: ["vertical_diagnostic"],
            13: ["thruster"],
            14: ["horizontal_diagnostic"],
        }
        labels = {name: label for label, names in [
            ("Vertical Plane", ["vertical_plane"]),
            ("Heading", ["heading"]),
            ("Navigation", ["navplot"]),
            ("Power / Battery", ["power"]),
            ("CTD Time Series", ["ctd_time_series", "ctd_profiles"]),
            ("T-S Cast", ["ts_cast", "cascade_ctd"]),
            ("Depth Rate", ["depth_rate_check"]),
            ("Vertical Diag.", ["vertical_diagnostic"]),
            ("Thruster", ["thruster"]),
            ("Horiz. Diag.", ["horizontal_diagnostic"]),
        ] for name in names}

        for opt in opts:
            for name in opt_names.get(opt, []):
                fig = figs.get(name)
                if fig is not None:
                    _add_copy_button(fig)
                    self._tabs.addTab(_FigureTab(fig, self), labels.get(name, name))
                elif name not in generated:
                    holder = QtWidgets.QLabel(
                        f"<center><font color='gray' size='4'>"
                        f"Data not available for this panel<br><br>"
                        f"<small>{name}</small></font></center>"
                    )
                    self._tabs.addTab(holder, f"⊘ {labels.get(name, name)}")
        self.statusBar().showMessage(f"Compared {Path(path1).name} (blue) vs {Path(path2).name} (red)")


def _bold_label(text: str) -> QtWidgets.QLabel:
    label = QtWidgets.QLabel(text)
    font = label.font()
    font.setBold(True)
    label.setFont(font)
    return label


def compare(
    path1: str | Path,
    path2: str | Path,
    opts: Iterable[int] | str = DEFAULT_OPTS,
    *,
    show: bool = True,
) -> dict[str, plt.Figure]:
    """Compare two Odyssey .obd files by plotting both on the same figures."""
    ds1 = conditioning.condition(obd.read_obd(path1))
    ds2 = conditioning.condition(obd.read_obd(path2))

    if opts == "all":
        opts_set = set(range(1, 15))
    else:
        opts_set = set(opts)

    figs: dict[str, plt.Figure] = {}

    def _try(name: str, fn, *args, **kwargs):
        try:
            figs[name] = fn(*args, **kwargs)
        except (KeyError, ValueError) as exc:
            print(f"  [skip] {name}: {exc}")

    if 2 in opts_set:
        _try("vertical_plane", vertical_plane, ds1, ds2)
    if 3 in opts_set:
        _try("heading", heading, ds1, ds2)
    if 4 in opts_set:
        _try("navplot", navplot, ds1, ds2)
    if 7 in opts_set:
        _try("power", power, ds1, ds2)
    if 8 in opts_set:
        _try("ctd_time_series", ctd_time_series, ds1, ds2)
        _try("ctd_profiles", ctd_profiles, ds1, ds2)
    if 9 in opts_set:
        _try("ts_cast", ts_cast, ds1, ds2)
        _try("cascade_ctd", cascade_ctd, ds1, ds2)
    if 10 in opts_set:
        _try("optics_cast", optics_cast, ds1, ds2)
    if 11 in opts_set:
        _try("depth_rate_check", depth_rate_check, ds1, ds2)
    if 12 in opts_set:
        _try("vertical_diagnostic", vertical_diagnostic, ds1, ds2)
    if 13 in opts_set:
        _try("thruster", thruster, ds1, ds2)
    if 14 in opts_set:
        _try("horizontal_diagnostic", horizontal_diagnostic, ds1, ds2)

    if show and figs:
        plt.show()
    return figs


def _need_both(ds1: xr.Dataset, ds2: xr.Dataset, *names: str) -> None:
    missing = [name for name in names if name not in ds1 or name not in ds2]
    if missing:
        raise KeyError(f"Plot requires {missing} in both datasets.")


def _need_one(ds: xr.Dataset, *names: str) -> None:
    missing = [name for name in names if name not in ds]
    if missing:
        raise KeyError(f"Plot requires {missing}.")


def _legend_title(title: str, ds1: xr.Dataset, ds2: xr.Dataset) -> str:
    return f"{title}: {_run_name(ds1)} (blue) vs {_run_name(ds2)} (red)"


def heading(ds1: xr.Dataset, ds2: xr.Dataset) -> plt.Figure:
    _need_both(ds1, ds2, "m_yaw")
    fig, ax = plt.subplots(figsize=(9, 4))
    t1 = ds1["time"].values
    t2 = ds2["time"].values
    ax.plot(t1, np.degrees(ds1["m_yaw"].values), "b-", lw=0.8,
            label=f"{_run_name(ds1)}")
    ax.plot(t2, np.degrees(ds2["m_yaw"].values), "r-", lw=0.8,
            label=f"{_run_name(ds2)}")
    if "m_yaw_d" in ds1:
        ax.plot(t1, np.degrees(ds1["m_yaw_d"].values), "b--", lw=0.6,
                label=f"{_run_name(ds1)} cmd")
    if "m_yaw_d" in ds2:
        ax.plot(t2, np.degrees(ds2["m_yaw_d"].values), "r--", lw=0.6,
                label=f"{_run_name(ds2)} cmd")
    ax.set_xlabel("Mission time (s)")
    ax.set_ylabel("Heading (deg)")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="best", fontsize=9)
    fig.suptitle(_legend_title("Heading", ds1, ds2))
    fig.tight_layout()
    return fig


def vertical_plane(ds1: xr.Dataset, ds2: xr.Dataset) -> plt.Figure:
    _need_both(ds1, ds2, "m_depth", "m_pitch")
    t1 = ds1["time"].values
    t2 = ds2["time"].values

    fig, axes = plt.subplots(2, 1, sharex=True, figsize=(9, 6))
    axes[0].plot(t1, ds1["m_depth"].values, "b-", label=f"{_run_name(ds1)} depth")
    axes[0].plot(t2, ds2["m_depth"].values, "r-", label=f"{_run_name(ds2)} depth")
    if "m_depth_d" in ds1:
        axes[0].plot(t1, ds1["m_depth_d"].values, "b--", lw=0.8,
                     label=f"{_run_name(ds1)} cmd")
    if "m_depth_d" in ds2:
        axes[0].plot(t2, ds2["m_depth_d"].values, "r--", lw=0.8,
                     label=f"{_run_name(ds2)} cmd")
    axes[0].set_ylabel("Depth (m)")
    axes[0].invert_yaxis()
    axes[0].grid(True, alpha=0.3)
    axes[0].legend(loc="best", fontsize=9)

    pitch1 = np.degrees(ds1["m_pitch"].values)
    pitch2 = np.degrees(ds2["m_pitch"].values)
    axes[1].plot(t1, pitch1, "b-", label=f"{_run_name(ds1)} pitch")
    axes[1].plot(t2, pitch2, "r-", label=f"{_run_name(ds2)} pitch")
    axes[1].set_ylabel("Pitch (deg)")
    axes[1].set_xlabel("Mission time (s)")
    axes[1].grid(True, alpha=0.3)
    axes[1].legend(loc="best", fontsize=9)

    fig.suptitle(_legend_title("Vertical plane", ds1, ds2))
    fig.tight_layout()
    return fig


def navplot(ds1: xr.Dataset, ds2: xr.Dataset) -> plt.Figure:
    n1_name, e1_name = ("real_x", "real_y") if "real_x" in ds1 and "real_y" in ds1 else ("m_pos_n", "m_pos_e")
    n2_name, e2_name = ("real_x", "real_y") if "real_x" in ds2 and "real_y" in ds2 else ("m_pos_n", "m_pos_e")
    _need_one(ds1, n1_name, e1_name)
    _need_one(ds2, n2_name, e2_name)
    fig, ax = plt.subplots(figsize=(7, 7))
    n1 = ds1[n1_name].values
    e1 = ds1[e1_name].values
    n2 = ds2[n2_name].values
    e2 = ds2[e2_name].values
    ax.plot(e1, n1, "b-", lw=0.8, label=f"{_run_name(ds1)}")
    ax.plot(e2, n2, "r-", lw=0.8, label=f"{_run_name(ds2)}")
    ax.plot(e1[0], n1[0], "bo", markersize=8)
    ax.plot(e1[-1], n1[-1], "bs", markersize=8)
    ax.plot(e2[0], n2[0], "ro", markersize=8)
    ax.plot(e2[-1], n2[-1], "rs", markersize=8)
    ax.set_xlabel("East (m)")
    ax.set_ylabel("North (m)")
    ax.set_aspect("equal", adjustable="datalim")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="best", fontsize=9)
    fig.suptitle(_legend_title("Track", ds1, ds2))
    fig.tight_layout()
    return fig


def ctd_time_series(ds1: xr.Dataset, ds2: xr.Dataset) -> plt.Figure:
    _need_both(ds1, ds2, "water_temp", "salinity")
    t1 = ds1["time"].values
    t2 = ds2["time"].values
    fig, axes = plt.subplots(2, 1, sharex=True, figsize=(9, 6))
    axes[0].plot(t1, ds1["water_temp"].values, "b.", markersize=2,
                 label=f"{_run_name(ds1)}")
    axes[0].plot(t2, ds2["water_temp"].values, "r.", markersize=2,
                 label=f"{_run_name(ds2)}")
    axes[0].set_ylabel("Temperature (°C)")
    axes[0].grid(True, alpha=0.3)
    axes[0].legend(loc="best", fontsize=9)

    axes[1].plot(t1, ds1["salinity"].values, "b.", markersize=2,
                 label=f"{_run_name(ds1)}")
    axes[1].plot(t2, ds2["salinity"].values, "r.", markersize=2,
                 label=f"{_run_name(ds2)}")
    axes[1].set_ylabel("Salinity (psu)")
    axes[1].set_xlabel("Mission time (s)")
    axes[1].grid(True, alpha=0.3)
    axes[1].legend(loc="best", fontsize=9)

    fig.suptitle(_legend_title("Temperature & salinity vs time", ds1, ds2))
    fig.tight_layout()
    return fig


def ctd_profiles(ds1: xr.Dataset, ds2: xr.Dataset) -> plt.Figure:
    _need_both(ds1, ds2, "water_temp", "salinity", "density", "ctd_depth")
    z1 = ds1["ctd_depth"].values
    z2 = ds2["ctd_depth"].values
    fig, axes = plt.subplots(1, 3, sharey=True, figsize=(11, 6))
    axes[0].plot(ds1["water_temp"].values, z1, "b-", lw=0.6,
                 label=f"{_run_name(ds1)}")
    axes[0].plot(ds2["water_temp"].values, z2, "r-", lw=0.6,
                 label=f"{_run_name(ds2)}")
    axes[0].set_xlabel("Temperature (°C)")
    axes[0].set_ylabel("Depth (dbar)")
    axes[0].invert_yaxis()
    axes[0].grid(True, alpha=0.3)
    axes[0].legend(loc="best", fontsize=8)

    axes[1].plot(ds1["salinity"].values, z1, "b-", lw=0.6)
    axes[1].plot(ds2["salinity"].values, z2, "r-", lw=0.6)
    axes[1].set_xlabel("Salinity (psu)")
    axes[1].grid(True, alpha=0.3)

    axes[2].plot(ds1["density"].values, z1, "b-", lw=0.6)
    axes[2].plot(ds2["density"].values, z2, "r-", lw=0.6)
    axes[2].set_xlabel("Density (kg/m³)")
    axes[2].grid(True, alpha=0.3)

    axes[0].set_title(_legend_title("CTD profiles", ds1, ds2))
    fig.tight_layout()
    return fig


def ts_cast(ds1: xr.Dataset, ds2: xr.Dataset) -> plt.Figure:
    _need_both(ds1, ds2, "water_temp", "salinity", "m_depth")
    has_svel = "sound_velocity" in ds1 and "sound_velocity" in ds2
    n_panels = 3 if has_svel else 2
    fig, axes = plt.subplots(1, n_panels, sharey=True, figsize=(4 * n_panels, 6))
    if n_panels == 1:
        axes = [axes]

    u1 = ds1["going_up"].values if "going_up" in ds1 else np.zeros(ds1["m_depth"].size, bool)
    d1 = ds1["going_down"].values if "going_down" in ds1 else np.zeros(ds1["m_depth"].size, bool)
    u2 = ds2["going_up"].values if "going_up" in ds2 else np.zeros(ds2["m_depth"].size, bool)
    d2 = ds2["going_down"].values if "going_down" in ds2 else np.zeros(ds2["m_depth"].size, bool)

    def split_plot(ax, x1, z1, x2, z2, label):
        ax.plot(np.where(d1, x1, np.nan), z1, "b-", lw=0.6, label=f"{_run_name(ds1)} desc")
        ax.plot(np.where(u1, x1, np.nan), z1, "b--", lw=0.6, label=f"{_run_name(ds1)} asc")
        ax.plot(np.where(d2, x2, np.nan), z2, "r-", lw=0.6, label=f"{_run_name(ds2)} desc")
        ax.plot(np.where(u2, x2, np.nan), z2, "r--", lw=0.6, label=f"{_run_name(ds2)} asc")
        ax.set_xlabel(label)
        ax.invert_yaxis()
        ax.grid(True, alpha=0.3)
        ax.legend(loc="best", fontsize=8)

    split_plot(axes[0], ds1["water_temp"].values, ds1["m_depth"].values,
               ds2["water_temp"].values, ds2["m_depth"].values,
               "Temperature (°C)")
    axes[0].set_ylabel("Depth (m)")
    split_plot(axes[1], ds1["salinity"].values, ds1["m_depth"].values,
               ds2["salinity"].values, ds2["m_depth"].values,
               "Salinity (psu)")
    if has_svel:
        split_plot(axes[2], ds1["sound_velocity"].values, ds1["m_depth"].values,
                   ds2["sound_velocity"].values, ds2["m_depth"].values,
                   "Sound velocity (m/s)")
    fig.suptitle(_legend_title("T-S cast", ds1, ds2))
    fig.tight_layout()
    return fig


def cascade_ctd(ds1: xr.Dataset, ds2: xr.Dataset) -> plt.Figure:
    _need_both(ds1, ds2, "water_temp", "salinity", "yoyo", "d_rate", "m_depth")
    yoyo1 = ds1["yoyo"].values
    yoyo2 = ds2["yoyo"].values
    dr1 = ds1["d_rate"].values
    dr2 = ds2["d_rate"].values
    z1 = ds1["m_depth"].values
    z2 = ds2["m_depth"].values
    t_w1 = ds1["water_temp"].values
    t_w2 = ds2["water_temp"].values
    sal1 = ds1["salinity"].values
    sal2 = ds2["salinity"].values

    yoyo_max = max(
        int(np.nanmax(yoyo1)) if np.isfinite(np.nanmax(yoyo1)) else 0,
        int(np.nanmax(yoyo2)) if np.isfinite(np.nanmax(yoyo2)) else 0,
    )

    fig, axes = plt.subplots(1, 2, figsize=(11, 6))
    for i in range(1, yoyo_max + 1):
        mask_up1 = (yoyo1 == i) & (dr1 < -0.1)
        mask_dn1 = (yoyo1 == i) & (dr1 > 0.1)
        mask_up2 = (yoyo2 == i) & (dr2 < -0.1)
        mask_dn2 = (yoyo2 == i) & (dr2 > 0.1)
        axes[0].plot(t_w1[mask_up1] + 0.5 * i, z1[mask_up1], "b-", lw=0.6)
        axes[0].plot(t_w1[mask_dn1] + 0.5 * i, z1[mask_dn1], "b--", lw=0.6)
        axes[0].plot(t_w2[mask_up2] + 0.5 * i, z2[mask_up2], "r-", lw=0.6)
        axes[0].plot(t_w2[mask_dn2] + 0.5 * i, z2[mask_dn2], "r--", lw=0.6)
        axes[1].plot(sal1[mask_up1] + 0.2 * i, z1[mask_up1], "b-", lw=0.6)
        axes[1].plot(sal1[mask_dn1] + 0.2 * i, z1[mask_dn1], "b--", lw=0.6)
        axes[1].plot(sal2[mask_up2] + 0.2 * i, z2[mask_up2], "r-", lw=0.6)
        axes[1].plot(sal2[mask_dn2] + 0.2 * i, z2[mask_dn2], "r--", lw=0.6)

    axes[0].set_xlabel("Temperature (°C, cascaded)")
    axes[0].set_ylabel("Depth (m)")
    axes[0].invert_yaxis()
    axes[0].grid(True, alpha=0.3)
    axes[0].set_title("Temperature cascade")
    axes[1].set_xlabel("Salinity (psu, cascaded)")
    axes[1].invert_yaxis()
    axes[1].grid(True, alpha=0.3)
    axes[1].set_title("Salinity cascade")
    fig.suptitle(_legend_title("Cascade CTD", ds1, ds2))
    fig.tight_layout()
    return fig


def optics_cast(ds1: xr.Dataset, ds2: xr.Dataset) -> plt.Figure:
    if not any(v in ds1 or v in ds2 for v in ("water_temp", "m_fl_voltage", "m_bs_voltage")):
        raise KeyError("None of water_temp, m_fl_voltage, m_bs_voltage in either dataset.")

    panels = []
    if "water_temp" in ds1 or "water_temp" in ds2:
        panels.append(("Temperature (°C)", "water_temp"))
    if "m_fl_voltage" in ds1 or "m_fl_voltage" in ds2:
        panels.append(("Fluorometer (mV)", "m_fl_voltage"))
    if "m_bs_voltage" in ds1 or "m_bs_voltage" in ds2:
        panels.append(("Optical backscatter (mV)", "m_bs_voltage"))

    fig, axes = plt.subplots(1, len(panels), sharey=True, figsize=(4 * len(panels), 6))
    if len(panels) == 1:
        axes = [axes]

    for ax, (label, var) in zip(axes, panels):
        if var in ds1:
            ax.plot(ds1[var].values, ds1["m_depth"].values, "b-", lw=0.6,
                    label=f"{_run_name(ds1)}")
        if var in ds2:
            ax.plot(ds2[var].values, ds2["m_depth"].values, "r-", lw=0.6,
                    label=f"{_run_name(ds2)}")
        ax.set_xlabel(label)
        ax.invert_yaxis()
        ax.grid(True, alpha=0.3)
        ax.legend(loc="best", fontsize=8)
    axes[0].set_ylabel("Depth (m)")
    fig.suptitle(_legend_title("Optical sensors", ds1, ds2))
    fig.tight_layout()
    return fig


def depth_rate_check(ds1: xr.Dataset, ds2: xr.Dataset) -> plt.Figure:
    _need_both(ds1, ds2, "d_rate", "m_depth")
    t1 = ds1["time"].values
    t2 = ds2["time"].values
    z1 = ds1["m_depth"].values
    z2 = ds2["m_depth"].values
    fig, axes = plt.subplots(3, 1, sharex=True, figsize=(9, 8))
    axes[0].plot(t1, ds1["d_rate"].values, "b.", markersize=2,
                 label=f"{_run_name(ds1)} d/dt")
    axes[0].plot(t2, ds2["d_rate"].values, "r.", markersize=2,
                 label=f"{_run_name(ds2)} d/dt")
    if all(v in ds1 for v in ("m_rdi_z", "m_rdi_y", "m_pitch")):
        proj1 = (ds1["m_rdi_z"].values * np.cos(ds1["m_pitch"].values)
                 - ds1["m_rdi_y"].values * np.sin(ds1["m_pitch"].values))
        axes[0].plot(t1, proj1, "bx", markersize=2, alpha=0.5,
                     label=f"{_run_name(ds1)} DVL")
    if all(v in ds2 for v in ("m_rdi_z", "m_rdi_y", "m_pitch")):
        proj2 = (ds2["m_rdi_z"].values * np.cos(ds2["m_pitch"].values)
                 - ds2["m_rdi_y"].values * np.sin(ds2["m_pitch"].values))
        axes[0].plot(t2, proj2, "rx", markersize=2, alpha=0.5,
                     label=f"{_run_name(ds2)} DVL")
    axes[0].set_ylabel("Depth rate (m/s)")
    axes[0].legend(loc="best", fontsize=8)
    axes[0].grid(True, alpha=0.3)

    axes[1].plot(t1, z1, "b-", lw=0.6, label=f"{_run_name(ds1)}")
    axes[1].plot(t2, z2, "r-", lw=0.6, label=f"{_run_name(ds2)}")
    axes[1].set_ylabel("Depth (m)")
    axes[1].invert_yaxis()
    axes[1].grid(True, alpha=0.3)
    axes[1].legend(loc="best", fontsize=8)

    axes[2].set_ylabel("DVL velocities (m/s)")
    if all(v in ds1 for v in ("m_rdi_x", "m_rdi_y", "m_rdi_z")):
        axes[2].plot(t1, ds1["m_rdi_x"].values, "b.", markersize=2, label=f"{_run_name(ds1)} x")
        axes[2].plot(t1, ds1["m_rdi_y"].values, "b-", markersize=2, label=f"{_run_name(ds1)} y")
        axes[2].plot(t1, ds1["m_rdi_z"].values, "b--", markersize=2, label=f"{_run_name(ds1)} z")
    if all(v in ds2 for v in ("m_rdi_x", "m_rdi_y", "m_rdi_z")):
        axes[2].plot(t2, ds2["m_rdi_x"].values, "r.", markersize=2, label=f"{_run_name(ds2)} x")
        axes[2].plot(t2, ds2["m_rdi_y"].values, "r-", markersize=2, label=f"{_run_name(ds2)} y")
        axes[2].plot(t2, ds2["m_rdi_z"].values, "r--", markersize=2, label=f"{_run_name(ds2)} z")
    axes[2].legend(loc="best", fontsize=8)
    axes[2].set_xlabel("Time (s)")
    axes[2].grid(True, alpha=0.3)

    fig.suptitle(_legend_title("Depth-rate check", ds1, ds2))
    fig.tight_layout()
    return fig


def vertical_diagnostic(ds1: xr.Dataset, ds2: xr.Dataset) -> plt.Figure:
    _need_both(ds1, ds2, "m_depth", "m_pitch")
    t1 = ds1["time"].values
    t2 = ds2["time"].values
    fig, axes = plt.subplots(3, 1, sharex=True, figsize=(10, 9))

    axes[0].plot(t1, ds1["m_depth"].values, "b-", lw=1.0,
                 label=f"{_run_name(ds1)} measured")
    axes[0].plot(t2, ds2["m_depth"].values, "r-", lw=1.0,
                 label=f"{_run_name(ds2)} measured")
    if "c_depth" in ds1:
        axes[0].plot(t1, ds1["c_depth"].values, "b--", lw=0.8,
                     label=f"{_run_name(ds1)} commanded")
    if "c_depth" in ds2:
        axes[0].plot(t2, ds2["c_depth"].values, "r--", lw=0.8,
                     label=f"{_run_name(ds2)} commanded")
    axes[0].set_ylabel("Depth (m)")
    axes[0].invert_yaxis()
    axes[0].grid(True, alpha=0.3)
    axes[0].legend(loc="best", fontsize=8)

    axes[1].plot(t1, np.degrees(ds1["m_pitch"].values), "b-", lw=1.0,
                 label=f"{_run_name(ds1)} measured")
    axes[1].plot(t2, np.degrees(ds2["m_pitch"].values), "r-", lw=1.0,
                 label=f"{_run_name(ds2)} measured")
    if "c_pitch" in ds1:
        axes[1].plot(t1, np.degrees(ds1["c_pitch"].values), "b--", lw=0.8,
                     label=f"{_run_name(ds1)} commanded")
    if "c_pitch" in ds2:
        axes[1].plot(t2, np.degrees(ds2["c_pitch"].values), "r--", lw=0.8,
                     label=f"{_run_name(ds2)} commanded")
    if "uo_control_pitch_lim" in ds1:
        plim1 = ds1["uo_control_pitch_lim"].values
        finite1 = np.isfinite(plim1)
        if finite1.any():
            limit1 = float(plim1[finite1][-1])
            axes[1].axhline(limit1, color="b", ls=":", lw=0.6, alpha=0.5)
            axes[1].axhline(-limit1, color="b", ls=":", lw=0.6, alpha=0.5)
    if "uo_control_pitch_lim" in ds2:
        plim2 = ds2["uo_control_pitch_lim"].values
        finite2 = np.isfinite(plim2)
        if finite2.any():
            limit2 = float(plim2[finite2][-1])
            axes[1].axhline(limit2, color="r", ls=":", lw=0.6, alpha=0.5)
            axes[1].axhline(-limit2, color="r", ls=":", lw=0.6, alpha=0.5)
    axes[1].set_ylabel("Pitch (deg)")
    axes[1].grid(True, alpha=0.3)
    axes[1].legend(loc="best", fontsize=8)

    have_elev1 = "m_elevator_angle" in ds1 or "c_elevator_angle" in ds1
    have_elev2 = "m_elevator_angle" in ds2 or "c_elevator_angle" in ds2
    if have_elev1 or have_elev2:
        if "m_elevator_angle" in ds1:
            axes[2].plot(t1, np.degrees(ds1["m_elevator_angle"].values), "b-", lw=1.0,
                         label=f"{_run_name(ds1)} measured")
        if "m_elevator_angle" in ds2:
            axes[2].plot(t2, np.degrees(ds2["m_elevator_angle"].values), "r-", lw=1.0,
                         label=f"{_run_name(ds2)} measured")
        if "c_elevator_angle" in ds1:
            axes[2].plot(t1, np.degrees(ds1["c_elevator_angle"].values), "b--", lw=0.8,
                         label=f"{_run_name(ds1)} commanded")
        if "c_elevator_angle" in ds2:
            axes[2].plot(t2, np.degrees(ds2["c_elevator_angle"].values), "r--", lw=0.8,
                         label=f"{_run_name(ds2)} commanded")
        axes[2].legend(loc="best", fontsize=8)
    else:
        axes[2].text(0.5, 0.5, "No elevator data logged",
                     ha="center", va="center", transform=axes[2].transAxes,
                     color="gray")
    axes[2].set_ylabel("Elevator (deg)")
    axes[2].set_xlabel("Mission time (s)")
    axes[2].grid(True, alpha=0.3)

    fig.suptitle(_legend_title("Vertical-plane diagnostic", ds1, ds2))
    fig.tight_layout()
    return fig


def thruster(ds1: xr.Dataset, ds2: xr.Dataset) -> plt.Figure:
    velocity_vars = ("m_water_speed", "real_u", "real_v", "real_w")
    if not any(v in ds1 or v in ds2 for v in ("m_thruster_current", "c_thruster_current", "m_propeller_speed") + velocity_vars):
        raise KeyError("No thruster channels logged in either dataset.")

    t1 = ds1["time"].values
    t2 = ds2["time"].values
    panels = []
    if "m_thruster_current" in ds1 or "c_thruster_current" in ds1 or "m_thruster_current" in ds2 or "c_thruster_current" in ds2:
        panels.append("current")
    if "m_propeller_speed" in ds1 or "m_propeller_speed" in ds2:
        panels.append("rpm")
    if any(v in ds1 or v in ds2 for v in velocity_vars):
        panels.append("velocity")

    fig, axes = plt.subplots(len(panels), 1, sharex=True, figsize=(10, 3.2 * len(panels)))
    if len(panels) == 1:
        axes = [axes]

    panel = 0
    if "current" in panels:
        if "m_thruster_current" in ds1:
            axes[panel].plot(t1, ds1["m_thruster_current"].values, "b-", lw=1.0,
                             label=f"{_run_name(ds1)} measured")
        if "m_thruster_current" in ds2:
            axes[panel].plot(t2, ds2["m_thruster_current"].values, "r-", lw=1.0,
                             label=f"{_run_name(ds2)} measured")
        if "c_thruster_current" in ds1:
            axes[panel].plot(t1, ds1["c_thruster_current"].values, "b--", lw=0.8,
                             label=f"{_run_name(ds1)} commanded")
        if "c_thruster_current" in ds2:
            axes[panel].plot(t2, ds2["c_thruster_current"].values, "r--", lw=0.8,
                             label=f"{_run_name(ds2)} commanded")
        axes[panel].set_ylabel("Thruster current (A)")
        axes[panel].grid(True, alpha=0.3)
        axes[panel].legend(loc="best", fontsize=8)
        axes[panel].set_title("Thruster current")
        panel += 1

    if "rpm" in panels:
        if "m_propeller_speed" in ds1:
            axes[panel].plot(t1, ds1["m_propeller_speed"].values, "b-", lw=1.0,
                             label=f"{_run_name(ds1)} measured")
        if "m_propeller_speed" in ds2:
            axes[panel].plot(t2, ds2["m_propeller_speed"].values, "r-", lw=1.0,
                             label=f"{_run_name(ds2)} measured")
        axes[panel].set_ylabel("Propeller speed (RPM)")
        axes[panel].grid(True, alpha=0.3)
        axes[panel].legend(loc="best", fontsize=8)
        panel += 1

    if "velocity" in panels:
        styles = {
            "m_water_speed": ("water speed", "-", 1.0),
            "real_u": ("real_u axial", "-", 0.9),
            "real_v": ("real_v sideways", "--", 0.9),
            "real_w": ("real_w upward", ":", 0.9),
        }
        for name in velocity_vars:
            label, linestyle, width = styles[name]
            if name in ds1:
                axes[panel].plot(t1, ds1[name].values, color="blue",
                                 linestyle=linestyle, lw=width,
                                 label=f"{_run_name(ds1)} {label}")
            if name in ds2:
                axes[panel].plot(t2, ds2[name].values, color="red",
                                 linestyle=linestyle, lw=width,
                                 label=f"{_run_name(ds2)} {label}")
        axes[panel].set_ylabel("Vehicle velocity (m/s)")
        axes[panel].grid(True, alpha=0.3)
        axes[panel].legend(loc="best", fontsize=8)
        panel += 1

    axes[-1].set_xlabel("Mission time (s)")
    fig.suptitle(_legend_title("Thruster", ds1, ds2))
    fig.tight_layout()
    return fig


def horizontal_diagnostic(ds1: xr.Dataset, ds2: xr.Dataset) -> plt.Figure:
    n1_name, e1_name = ("real_x", "real_y") if "real_x" in ds1 and "real_y" in ds1 else ("m_pos_n", "m_pos_e")
    n2_name, e2_name = ("real_x", "real_y") if "real_x" in ds2 and "real_y" in ds2 else ("m_pos_n", "m_pos_e")
    _need_both(ds1, ds2, "m_yaw")
    _need_one(ds1, n1_name, e1_name)
    _need_one(ds2, n2_name, e2_name)
    t1 = ds1["time"].values
    t2 = ds2["time"].values
    fig = plt.figure(figsize=(10, 11))
    gs = fig.add_gridspec(2, 1, height_ratios=[2, 1], hspace=0.25)

    ax_top = fig.add_subplot(gs[0])
    n1 = ds1[n1_name].values
    e1 = ds1[e1_name].values
    n2 = ds2[n2_name].values
    e2 = ds2[e2_name].values
    ax_top.plot(e1, n1, "b-", lw=0.8, label=f"{_run_name(ds1)}")
    ax_top.plot(e2, n2, "r-", lw=0.8, label=f"{_run_name(ds2)}")
    ax_top.plot(e1[0], n1[0], "bo", markersize=10)
    ax_top.plot(e1[-1], n1[-1], "bs", markersize=10)
    ax_top.plot(e2[0], n2[0], "ro", markersize=10)
    ax_top.plot(e2[-1], n2[-1], "rs", markersize=10)
    ax_top.set_xlabel("East (m)")
    ax_top.set_ylabel("North (m)")
    ax_top.set_aspect("equal", adjustable="datalim")
    ax_top.grid(True, alpha=0.3)
    ax_top.legend(loc="best", fontsize=8)

    ax_bot = fig.add_subplot(gs[1])
    yaw1 = np.degrees(ds1["m_yaw"].values) % 360
    yaw2 = np.degrees(ds2["m_yaw"].values) % 360
    ax_bot.plot(t1, yaw1, "b-", lw=1.0, label=f"{_run_name(ds1)} heading")
    ax_bot.plot(t2, yaw2, "r-", lw=1.0, label=f"{_run_name(ds2)} heading")
    if "c_heading" in ds1:
        ax_bot.plot(t1, np.degrees(ds1["c_heading"].values) % 360,
                    "b--", lw=0.8, label=f"{_run_name(ds1)} cmd")
    if "c_heading" in ds2:
        ax_bot.plot(t2, np.degrees(ds2["c_heading"].values) % 360,
                    "r--", lw=0.8, label=f"{_run_name(ds2)} cmd")
    ax_bot.set_ylabel("Heading (deg)")
    ax_bot.set_xlabel("Mission time (s)")
    ax_bot.grid(True, alpha=0.3)

    ax_bot.legend(loc="best", fontsize=8)

    fig.suptitle(_legend_title("Horizontal-plane diagnostic", ds1, ds2))
    fig.tight_layout()
    return fig


def power(ds1: xr.Dataset, ds2: xr.Dataset) -> plt.Figure:
    channels = (
        "m_battery_voltage", "m_battery_current", "real_battery_soc",
        "m_batt_soc_est", "real_battery_power", "m_km_voltage",
        "m_km_current",
    )
    if not any(v in ds1 or v in ds2 for v in channels):
        raise KeyError("No power channels logged in either dataset.")

    v1 = "m_battery_voltage" if "m_battery_voltage" in ds1 else "m_km_voltage"
    v2 = "m_battery_voltage" if "m_battery_voltage" in ds2 else "m_km_voltage"
    i1 = "m_battery_current" if "m_battery_current" in ds1 else "m_km_current"
    i2 = "m_battery_current" if "m_battery_current" in ds2 else "m_km_current"
    s1 = "real_battery_soc" if "real_battery_soc" in ds1 else "m_batt_soc_est"
    s2 = "real_battery_soc" if "real_battery_soc" in ds2 else "m_batt_soc_est"
    fig, axes = plt.subplots(4, 1, sharex=True, figsize=(9, 8))
    t1 = ds1["time"].values
    t2 = ds2["time"].values

    if v1 in ds1 or v2 in ds2:
        if v1 in ds1:
            axes[0].plot(t1, ds1[v1].values, "b-", label=f"{_run_name(ds1)}")
        if v2 in ds2:
            axes[0].plot(t2, ds2[v2].values, "r-", label=f"{_run_name(ds2)}")
        axes[0].set_ylabel("Voltage (V)")
        axes[0].legend(loc="best", fontsize=8)

    if i1 in ds1 or i2 in ds2:
        if i1 in ds1:
            axes[1].plot(t1, ds1[i1].values, "b-", label=f"{_run_name(ds1)}")
        if i2 in ds2:
            axes[1].plot(t2, ds2[i2].values, "r-", label=f"{_run_name(ds2)}")
        axes[1].set_ylabel("Current (A)")
        axes[1].legend(loc="best", fontsize=8)

    if s1 in ds1 or s2 in ds2:
        if s1 in ds1:
            axes[2].plot(t1, ds1[s1].values, "b-", label=f"{_run_name(ds1)}")
        if s2 in ds2:
            axes[2].plot(t2, ds2[s2].values, "r-", label=f"{_run_name(ds2)}")
        axes[2].set_ylabel("SOC")
        axes[2].legend(loc="best", fontsize=8)

    if "real_battery_power" in ds1 or "real_battery_power" in ds2:
        if "real_battery_power" in ds1:
            axes[3].plot(t1, ds1["real_battery_power"].values, "b-", label=f"{_run_name(ds1)}")
        if "real_battery_power" in ds2:
            axes[3].plot(t2, ds2["real_battery_power"].values, "r-", label=f"{_run_name(ds2)}")
        axes[3].set_ylabel("Power (W)")
        axes[3].legend(loc="best", fontsize=8)

    axes[-1].set_xlabel("Mission time (s)")
    for ax in axes:
        ax.grid(True, alpha=0.3)
    fig.suptitle(_legend_title("Power", ds1, ds2))
    fig.tight_layout()
    return fig


def main() -> None:
    import argparse
    import sys

    if len(sys.argv) == 1:
        app = QtWidgets.QApplication(sys.argv)
        win = CompareMainWindow()
        win.show()
        sys.exit(app.exec_())

    parser = argparse.ArgumentParser(
        description="Compare two Odyssey .obd files side by side."
    )
    parser.add_argument("path1", help="first .obd file")
    parser.add_argument("path2", help="second .obd file")
    parser.add_argument(
        "--opts",
        default="2,3",
        help="comma-separated options (e.g. '2,3,8,9' or 'all')",
    )
    parser.add_argument(
        "--save",
        help="directory to save generated figures (PNG) and skip display",
    )
    args = parser.parse_args()

    if args.opts == "all":
        opts: Iterable[int] | str = "all"
    else:
        opts = [int(x) for x in args.opts.split(",") if x.strip()]

    figs = compare(args.path1, args.path2, opts=opts, show=args.save is None)
    if args.save:
        out = Path(args.save)
        out.mkdir(parents=True, exist_ok=True)
        stem1 = Path(args.path1).stem
        stem2 = Path(args.path2).stem
        for name, fig in figs.items():
            target = out / f"{stem1}_{stem2}_{name}.png"
            fig.savefig(target, dpi=120, bbox_inches="tight")
            print(f"saved {target}")
        sys.exit(0)


if __name__ == "__main__":
    main()
