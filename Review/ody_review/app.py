"""
PyQt5 GUI for reviewing Odyssey AUV mission data.

Layout
------
Left sidebar  OBD file selector (newest first), file info panel, plot-panel
              checkboxes with All/None toggles, Generate button.
Right area    QTabWidget — one tab per selected plot panel, each holding a
              matplotlib FigureCanvas with a navigation toolbar.

Entry point
-----------
Run via  Review/launch_review.py  (sets the Qt5Agg backend before importing
this module).  Do not call matplotlib.use() inside this file — the backend
must already be set by the time matplotlib.pyplot is imported.
"""
from __future__ import annotations

import sys
from datetime import datetime
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.backends.backend_qt5agg import NavigationToolbar2QT as NavigationToolbar
from PyQt5 import QtCore, QtWidgets

from . import conditioning, obd, panels, plots


# ---------------------------------------------------------------------------
# SData directory discovery
# ---------------------------------------------------------------------------

def _find_sdata() -> Path:
    """Return the SData directory, searching several candidate locations."""
    candidates = [
        Path(__file__).resolve().parent.parent.parent / "SData",  # …/Windows_Ody_C/SData
        Path.cwd() / "SData",
        Path.cwd().parent / "SData",
    ]
    for c in candidates:
        if c.is_dir():
            return c
    return candidates[0]   # best guess even if it doesn't exist yet


def _list_obd_files(sdata: Path) -> list[Path]:
    """List .obd files in sdata, newest first."""
    if not sdata.is_dir():
        return []
    files = list(sdata.glob("*.obd"))
    files.sort(key=lambda p: p.stat().st_mtime, reverse=True)
    return files


# ---------------------------------------------------------------------------
# Panel registry
# Each entry: (tab label, plot function, default checked)
# ---------------------------------------------------------------------------

_PANELS: list[tuple[str, object, bool]] = [
    ("Vertical Control",  panels.vertical_control,      True),
    ("Heading & Rudder",  panels.heading_panel,         True),
    ("Behavior Axes",     panels.behavior_axis_waterfall, True),
    ("Fault / Behaviors", panels.fault_timeline,        True),
    ("Navigation",        plots.navplot,                True),
    ("Vertical Diag.",    plots.vertical_diagnostic,    False),
    ("Horiz. Diag.",      plots.horizontal_diagnostic,  False),
    ("Power / Battery",   plots.power,                  False),
    ("CTD Time Series",   plots.ctd_time_series,        False),
    ("CTD Profiles",      plots.ctd_profiles,           False),
    ("T-S Cast",          plots.ts_cast,                False),
    ("Thruster",          plots.thruster,               False),
]


# ---------------------------------------------------------------------------
# Single-figure tab
# ---------------------------------------------------------------------------

class _FigureTab(QtWidgets.QWidget):
    """A tab widget holding one matplotlib figure and its navigation toolbar."""

    def __init__(self, fig: plt.Figure, parent=None):
        super().__init__(parent)
        self._fig = fig
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)
        canvas  = FigureCanvas(fig)
        toolbar = NavigationToolbar(canvas, self)
        layout.addWidget(toolbar)
        layout.addWidget(canvas)


# ---------------------------------------------------------------------------
# Left sidebar
# ---------------------------------------------------------------------------

class _Sidebar(QtWidgets.QWidget):
    """Sidebar with file selector, metadata display, and panel checkboxes."""

    load_requested  = QtCore.pyqtSignal(str)   # emits absolute path string
    plots_requested = QtCore.pyqtSignal()
    custom_plot_requested = QtCore.pyqtSignal(str, str)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFixedWidth(295)

        root = QtWidgets.QVBoxLayout(self)
        root.setContentsMargins(8, 10, 8, 8)
        root.setSpacing(6)

        # ---- File selector --------------------------------------------------
        root.addWidget(_bold_label("OBD File:"))
        self._combo = QtWidgets.QComboBox()
        self._combo.setSizePolicy(QtWidgets.QSizePolicy.Expanding,
                                   QtWidgets.QSizePolicy.Fixed)
        self._combo.setToolTip("Select an OBD data file — newest at top")
        root.addWidget(self._combo)

        btn_row = QtWidgets.QHBoxLayout()
        self._btn_refresh = QtWidgets.QPushButton("Refresh")
        self._btn_load    = QtWidgets.QPushButton("Load")
        self._btn_load.setDefault(True)
        self._btn_load.setStyleSheet("font-weight: bold;")
        btn_row.addWidget(self._btn_refresh)
        btn_row.addWidget(self._btn_load)
        root.addLayout(btn_row)

        # ---- File info ------------------------------------------------------
        info_box = QtWidgets.QGroupBox("File Info")
        form = QtWidgets.QFormLayout(info_box)
        form.setLabelAlignment(QtCore.Qt.AlignRight)
        form.setVerticalSpacing(3)
        self._info: dict[str, QtWidgets.QLabel] = {}
        for key in ("Vehicle", "Date", "Duration", "Cycles", "Fault file"):
            lbl = QtWidgets.QLabel("—")
            lbl.setWordWrap(True)
            form.addRow(f"{key}:", lbl)
            self._info[key] = lbl
        root.addWidget(info_box)

        # ---- Plot panel checkboxes ------------------------------------------
        panels_box = QtWidgets.QGroupBox("Plot Panels")
        pv = QtWidgets.QVBoxLayout(panels_box)
        pv.setSpacing(2)

        selrow = QtWidgets.QHBoxLayout()
        btn_all  = QtWidgets.QPushButton("All")
        btn_none = QtWidgets.QPushButton("None")
        for b in (btn_all, btn_none):
            b.setFixedHeight(22)
        selrow.addWidget(btn_all)
        selrow.addWidget(btn_none)
        pv.addLayout(selrow)

        self._checks: dict[str, QtWidgets.QCheckBox] = {}
        for name, _, default in _PANELS:
            cb = QtWidgets.QCheckBox(name)
            cb.setChecked(default)
            pv.addWidget(cb)
            self._checks[name] = cb

        root.addWidget(panels_box)

        # ---- Custom plot ----------------------------------------------------
        custom_box = QtWidgets.QGroupBox("Custom Plot")
        custom_form = QtWidgets.QFormLayout(custom_box)
        custom_form.setLabelAlignment(QtCore.Qt.AlignRight)
        self._combo_x = QtWidgets.QComboBox()
        self._combo_y = QtWidgets.QComboBox()
        self._combo_x.setEnabled(False)
        self._combo_y.setEnabled(False)
        self._combo_x.setToolTip("X-axis variable; defaults to mission time")
        self._combo_y.setToolTip("Y-axis variable")
        custom_form.addRow("X:", self._combo_x)
        custom_form.addRow("Y:", self._combo_y)
        self._btn_custom = QtWidgets.QPushButton("Plot X/Y")
        self._btn_custom.setEnabled(False)
        custom_form.addRow("", self._btn_custom)
        root.addWidget(custom_box)

        # ---- Generate -------------------------------------------------------
        self._btn_gen = QtWidgets.QPushButton("Generate Plots")
        self._btn_gen.setEnabled(False)
        self._btn_gen.setFixedHeight(32)
        self._btn_gen.setStyleSheet("font-weight: bold; font-size: 11pt;")
        root.addWidget(self._btn_gen)

        root.addStretch()

        # ---- Signals --------------------------------------------------------
        self._btn_refresh.clicked.connect(self._on_refresh)
        self._btn_load.clicked.connect(self._on_load)
        self._btn_gen.clicked.connect(self.plots_requested)
        self._btn_custom.clicked.connect(self._on_custom_plot)
        btn_all.clicked.connect(lambda: self._set_all(True))
        btn_none.clicked.connect(lambda: self._set_all(False))

        self._on_refresh()

    # ---- public helpers -----------------------------------------------------

    def selected_panels(self) -> list[tuple[str, object]]:
        return [(name, fn) for name, fn, _ in _PANELS
                if self._checks[name].isChecked()]

    def update_info(self, ds) -> None:
        self._info["Vehicle"].setText(ds.attrs.get("vehicle", "Unknown"))
        self._info["Date"].setText(ds.attrs.get("mission_date", "Unknown"))

        if "time" in ds.coords:
            t   = ds["time"].values
            fin = np.isfinite(t)
            if fin.any():
                dur = float(t[fin][-1] - t[fin][0])
                m, s = int(dur // 60), dur % 60
                self._info["Duration"].setText(f"{m}m {s:.0f}s  ({dur:.0f} s)")
            else:
                self._info["Duration"].setText("—")
        else:
            nc = ds.attrs.get("n_cycles", 0)
            self._info["Duration"].setText(f"~{nc * 0.2:.0f} s (est.)")

        self._info["Cycles"].setText(str(ds.attrs.get("n_cycles", "—")))

        fault = ds.attrs.get("fault_file", "")
        self._info["Fault file"].setText(fault or "(none)")
        if fault:
            self._info["Fault file"].setStyleSheet(
                "color: #cc0000; font-weight: bold;")
        else:
            self._info["Fault file"].setStyleSheet("")

        self._btn_gen.setEnabled(True)
        self.update_variable_choices(ds)

    def update_variable_choices(self, ds) -> None:
        names = ["time"] if "time" in ds.coords else []
        names.extend(sorted(str(name) for name in ds.data_vars))
        self._combo_x.clear()
        self._combo_y.clear()
        self._combo_x.addItems(names)
        self._combo_y.addItems(names)

        x_default = "m_present_time" if "m_present_time" in names else "time"
        y_default = _first_present(
            names,
            "real_battery_power",
            "m_battery_current",
            "m_copley_current",
            "m_thruster_current",
        )
        if x_default in names:
            self._combo_x.setCurrentText(x_default)
        if y_default:
            self._combo_y.setCurrentText(y_default)
        self._combo_x.setEnabled(bool(names))
        self._combo_y.setEnabled(bool(names))
        self._btn_custom.setEnabled(bool(names))

    # ---- private slots ------------------------------------------------------

    def _on_refresh(self) -> None:
        sdata = _find_sdata()
        self._combo.clear()
        files = _list_obd_files(sdata)
        for f in files:
            dt = datetime.fromtimestamp(f.stat().st_mtime).strftime("%Y-%m-%d %H:%M")
            self._combo.addItem(f"{f.name}  [{dt}]", userData=str(f))
        if not files:
            self._combo.addItem(f"(no .obd files in {sdata})")

    def _on_load(self) -> None:
        path = self._combo.currentData()
        if path:
            self.load_requested.emit(path)

    def _set_all(self, state: bool) -> None:
        for cb in self._checks.values():
            cb.setChecked(state)

    def _on_custom_plot(self) -> None:
        x_name = self._combo_x.currentText()
        y_name = self._combo_y.currentText()
        if x_name and y_name:
            self.custom_plot_requested.emit(x_name, y_name)


# ---------------------------------------------------------------------------
# Main window
# ---------------------------------------------------------------------------

class MainWindow(QtWidgets.QMainWindow):
    """Top-level review window: sidebar + tabbed plot area."""

    def __init__(self):
        super().__init__()
        self.setWindowTitle("Odyssey AUV Review")
        self.resize(1440, 920)

        self._ds = None   # current conditioned Dataset

        # Splitter: sidebar (fixed left) | tab area (expanding right)
        splitter = QtWidgets.QSplitter(QtCore.Qt.Horizontal)
        self.setCentralWidget(splitter)

        self._sidebar = _Sidebar()
        splitter.addWidget(self._sidebar)

        self._tabs = QtWidgets.QTabWidget()
        self._tabs.setDocumentMode(True)
        splitter.addWidget(self._tabs)
        splitter.setSizes([295, 1145])
        splitter.setStretchFactor(0, 0)
        splitter.setStretchFactor(1, 1)

        # Status bar
        sdata = _find_sdata()
        self.statusBar().showMessage(f"Ready — scanning {sdata}")

        # Wire up sidebar signals
        self._sidebar.load_requested.connect(self._load_file)
        self._sidebar.plots_requested.connect(self._generate_plots)
        self._sidebar.custom_plot_requested.connect(self._generate_custom_plot)

    # ---- file loading -------------------------------------------------------

    def _load_file(self, path_str: str) -> None:
        path = Path(path_str)
        self.statusBar().showMessage(f"Loading {path.name} …")
        QtWidgets.QApplication.processEvents()
        try:
            ds = obd.read_obd(path)
            ds = conditioning.condition(ds)
            self._ds = ds
            self._sidebar.update_info(ds)
            nc = ds.attrs.get("n_cycles", 0)
            self.statusBar().showMessage(
                f"Loaded {path.name}  ({nc} cycles)")
        except Exception as exc:
            self.statusBar().showMessage(f"Load error: {exc}")
            QtWidgets.QMessageBox.critical(self, "Load Error", str(exc))

    # ---- plot generation ----------------------------------------------------

    def _generate_plots(self) -> None:
        if self._ds is None:
            return

        # Close existing figures before clearing tabs (avoids memory leak)
        for i in range(self._tabs.count()):
            w = self._tabs.widget(i)
            if hasattr(w, "_fig"):
                plt.close(w._fig)
        self._tabs.clear()

        ds       = self._ds
        selected = self._sidebar.selected_panels()
        total    = len(selected)

        for i, (name, fn) in enumerate(selected, 1):
            self.statusBar().showMessage(
                f"Generating {i}/{total}: {name} …")
            QtWidgets.QApplication.processEvents()
            try:
                fig = fn(ds)
                self._tabs.addTab(_FigureTab(fig, self), name)
            except (KeyError, ValueError) as exc:
                holder = QtWidgets.QLabel(
                    f"<center><font color='gray' size='4'>"
                    f"Data not available for this panel<br><br>"
                    f"<small>{exc}</small></font></center>"
                )
                self._tabs.addTab(holder, f"⊘ {name}")

        shown   = sum(1 for i in range(self._tabs.count())
                      if not self._tabs.tabText(i).startswith("⊘"))
        skipped = total - shown
        msg = f"Done — {shown} panel(s) ready"
        if skipped:
            msg += f"  ({skipped} skipped — data not in file)"
        self.statusBar().showMessage(msg)

    def _generate_custom_plot(self, x_name: str, y_name: str) -> None:
        if self._ds is None:
            return
        try:
            fig = _custom_xy_figure(self._ds, x_name, y_name)
            tab_name = f"{y_name} vs {x_name}"
            self._tabs.addTab(_FigureTab(fig, self), tab_name[:40])
            self._tabs.setCurrentIndex(self._tabs.count() - 1)
            self.statusBar().showMessage(f"Custom plot ready: {tab_name}")
        except Exception as exc:
            self.statusBar().showMessage(f"Custom plot error: {exc}")
            QtWidgets.QMessageBox.warning(self, "Custom Plot", str(exc))


# ---------------------------------------------------------------------------
# Small helpers
# ---------------------------------------------------------------------------

def _bold_label(text: str) -> QtWidgets.QLabel:
    lbl = QtWidgets.QLabel(text)
    font = lbl.font()
    font.setBold(True)
    lbl.setFont(font)
    return lbl


def _first_present(names: list[str], *candidates: str) -> str:
    for name in candidates:
        if name in names:
            return name
    return names[1] if len(names) > 1 else (names[0] if names else "")


def _series(ds, name: str):
    if name == "time" and "time" in ds.coords:
        return ds["time"].values
    if name in ds:
        return ds[name].values
    if name in ds.coords:
        return ds.coords[name].values
    raise KeyError(f"{name} is not in the loaded dataset")


def _custom_xy_figure(ds, x_name: str, y_name: str) -> plt.Figure:
    x = np.asarray(_series(ds, x_name), dtype=float)
    y = np.asarray(_series(ds, y_name), dtype=float)
    n = min(x.size, y.size)
    if n == 0:
        raise ValueError("Selected variables have no samples.")
    x = x[:n]
    y = y[:n]
    finite = np.isfinite(x) & np.isfinite(y)
    if not finite.any():
        raise ValueError("Selected variables have no finite overlapping samples.")

    fig, ax = plt.subplots(figsize=(9, 5))
    ax.plot(x[finite], y[finite], "b-", lw=0.9)
    ax.set_xlabel(_axis_label(ds, x_name))
    ax.set_ylabel(_axis_label(ds, y_name))
    ax.set_title(f"{y_name} vs {x_name}: {plots._run_name(ds)}")
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    plots._add_copy_button(fig)
    return fig


def _axis_label(ds, name: str) -> str:
    units = ""
    if name in ds:
        units = ds[name].attrs.get("units", "")
    elif name in ds.coords:
        units = ds.coords[name].attrs.get("units", "")
    return f"{name} ({units})" if units else name
