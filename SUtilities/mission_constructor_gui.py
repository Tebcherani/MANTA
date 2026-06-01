from __future__ import annotations

import datetime as _dt
import json
from pathlib import Path
import subprocess
import sys

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.backends.backend_qt5agg import NavigationToolbar2QT as NavigationToolbar
from PyQt5 import QtCore, QtGui, QtWidgets

try:
    from . import mission_model as mm
except ImportError:  # direct script execution
    import mission_model as mm


ROOT = Path(__file__).resolve().parents[1]
RUN_DIR = Path(__file__).resolve().parent / "ksim_runs"
RUN_DIR.mkdir(parents=True, exist_ok=True)

REVIEW_DIR = ROOT / "Review"
if str(REVIEW_DIR) not in sys.path:
    sys.path.insert(0, str(REVIEW_DIR))

try:
    from ody_review import conditioning, obd, panels, plots
except ImportError:
    conditioning = obd = panels = plots = None


class FigureTab(QtWidgets.QWidget):
    def __init__(self, fig: plt.Figure, parent=None):
        super().__init__(parent)
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        canvas = FigureCanvas(fig)
        toolbar = NavigationToolbar(canvas, self)
        layout.addWidget(toolbar)
        layout.addWidget(canvas)


def _plot_ksim_depth_pitch(ds) -> plt.Figure:
    t = ds["time"].values
    fig, axes = plt.subplots(2, 1, sharex=True, figsize=(11, 7))

    ax = axes[0]
    ax.plot(t, ds["m_depth"].values, color="steelblue", lw=1.3, label="depth")
    if "c_depth" in ds and "cc_depth_mode" in ds:
        mode = ds["cc_depth_mode"].values
        target = np.where(mode == 1, ds["c_depth"].values, np.nan)
        if np.isfinite(target).any():
            ax.plot(t, target, "r--", lw=1.0, label="target depth")
    ax.invert_yaxis()
    ax.set_ylabel("Depth (m)")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="upper right", fontsize=8)
    ax.set_title("Ksim depth and pitch")

    ax = axes[1]
    if "m_pitch" in ds:
        ax.plot(t, np.degrees(ds["m_pitch"].values), color="steelblue",
                lw=1.2, label="pitch")
    if "c_depth" in ds and "cc_depth_mode" in ds:
        mode = ds["cc_depth_mode"].values
        pitch_cmd = np.where(mode == 2, np.degrees(ds["c_depth"].values), np.nan)
        if np.isfinite(pitch_cmd).any():
            ax.plot(t, pitch_cmd, "r--", lw=1.0, label="pitch command")
    ax.axhline(0, color="k", lw=0.5, alpha=0.35)
    ax.set_ylabel("Pitch (deg)")
    ax.set_xlabel("Mission time (s)")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="upper right", fontsize=8)

    fig.tight_layout()
    return fig


def _plot_ksim_heading_speed(ds) -> plt.Figure:
    t = ds["time"].values
    fig, axes = plt.subplots(2, 1, sharex=True, figsize=(11, 6))

    ax = axes[0]
    if "m_yaw" in ds:
        ax.plot(t, np.degrees(ds["m_yaw"].values) % 360, color="steelblue",
                lw=1.2, label="heading")
    if "c_heading" in ds and "cc_heading_mode" in ds:
        mode = ds["cc_heading_mode"].values
        heading_cmd = np.where(mode == 1, np.degrees(ds["c_heading"].values) % 360, np.nan)
        if np.isfinite(heading_cmd).any():
            ax.plot(t, heading_cmd, "r--", lw=1.0, label="heading command")
    ax.set_ylabel("Heading (deg)")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="upper right", fontsize=8)
    ax.set_title("Ksim heading and speed")

    ax = axes[1]
    if "m_water_speed" in ds:
        ax.plot(t, ds["m_water_speed"].values, color="steelblue",
                lw=1.2, label="measured speed")
    elif "real_u" in ds:
        ax.plot(t, ds["real_u"].values, color="steelblue",
                lw=1.2, label="forward speed")
    if "c_speed" in ds:
        ax.plot(t, ds["c_speed"].values, "r--", lw=1.0, label="speed command")
    ax.set_ylabel("Speed (m/s)")
    ax.set_xlabel("Mission time (s)")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="upper right", fontsize=8)

    fig.tight_layout()
    return fig


class MissionConstructor(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.catalog = mm.load_behavior_catalog()
        self.catalog_map = mm.catalog_by_name(self.catalog)
        self.vehicle_variables = mm.load_vehicle_variables()
        self.vehicle_sensor_names = mm.load_vehicle_sensor_names()
        self.mission = mm.Mission()
        self.fault_rules: list[mm.FaultRule] = []
        self.current_path: Path | None = None
        self.arg_widgets: list[tuple[mm.MissionSensor, QtWidgets.QLineEdit]] = []
        self.sensor_table_syncing = False
        self.failure_table_syncing = False
        self.baseline_mission_text = ""
        self.stack_summary_mode = "time"
        self.display_units_mode = "native"

        self.setWindowTitle("Odyssey Mission Constructor")
        self.resize(1320, 820)
        self._build_ui()
        self._new_yoyo_mission(prompt=False)

    def _build_ui(self) -> None:
        toolbar = self.addToolBar("Mission")
        toolbar.setMovable(False)
        for text, slot in [
            ("New", lambda checked=False: self._new_empty_mission()),
            ("YoYo Template", lambda checked=False: self._new_yoyo_mission()),
            ("Open", self._open_mission),
            ("Save", self._save_mission),
            ("Save As", self._save_mission_as),
            ("Save Failure", self._save_failure_script),
            ("Run Ksim", self._run_ksim),
        ]:
            action = QtWidgets.QAction(text, self)
            action.triggered.connect(slot)
            toolbar.addAction(action)

        self.tabs = QtWidgets.QTabWidget()
        self.setCentralWidget(self.tabs)

        constructor_tab = QtWidgets.QWidget()
        constructor_layout = QtWidgets.QVBoxLayout(constructor_tab)
        constructor_layout.setContentsMargins(0, 0, 0, 0)
        central = QtWidgets.QSplitter(QtCore.Qt.Horizontal)
        constructor_layout.addWidget(central)

        left = QtWidgets.QWidget()
        left_layout = QtWidgets.QVBoxLayout(left)
        left_layout.setContentsMargins(8, 8, 8, 8)
        left_layout.addWidget(QtWidgets.QLabel("Behavior Catalog"))
        self.search = QtWidgets.QLineEdit()
        self.search.setPlaceholderText("Filter behaviors")
        self.search.textChanged.connect(self._populate_catalog)
        left_layout.addWidget(self.search)
        self.catalog_list = QtWidgets.QListWidget()
        self.catalog_list.itemDoubleClicked.connect(self._add_catalog_behavior)
        left_layout.addWidget(self.catalog_list, 1)
        add_btn = QtWidgets.QPushButton("Add Behavior")
        add_btn.clicked.connect(self._add_catalog_behavior)
        left_layout.addWidget(add_btn)
        central.addWidget(left)

        middle = QtWidgets.QWidget()
        middle_layout = QtWidgets.QVBoxLayout(middle)
        middle_layout.setContentsMargins(8, 8, 8, 8)
        middle_layout.addWidget(QtWidgets.QLabel("Mission Stack"))

        stack_view_buttons = QtWidgets.QHBoxLayout()
        self.stack_summary_btn = QtWidgets.QPushButton("Show Depth")
        self.stack_summary_btn.setToolTip("Toggle the mission-stack summary between behavior time and depth.")
        self.stack_summary_btn.clicked.connect(self._toggle_stack_summary)
        self.units_btn = QtWidgets.QPushButton("Show hr/km/deg")
        self.units_btn.setToolTip("Toggle display readouts between simulator units and hr/km/deg.")
        self.units_btn.clicked.connect(self._toggle_display_units)
        stack_view_buttons.addWidget(self.stack_summary_btn)
        stack_view_buttons.addWidget(self.units_btn)
        middle_layout.addLayout(stack_view_buttons)

        self.stack_list = QtWidgets.QListWidget()
        stack_font = QtGui.QFontDatabase.systemFont(QtGui.QFontDatabase.FixedFont)
        self.stack_list.setFont(stack_font)
        self.stack_list.currentRowChanged.connect(self._render_details)
        middle_layout.addWidget(self.stack_list, 1)

        stack_buttons = QtWidgets.QHBoxLayout()
        for text, slot in [
            ("Up", lambda: self._move_selected(-1)),
            ("Down", lambda: self._move_selected(1)),
            ("Duplicate", self._duplicate_selected),
            ("Remove", self._remove_selected),
        ]:
            btn = QtWidgets.QPushButton(text)
            btn.clicked.connect(slot)
            stack_buttons.addWidget(btn)
        middle_layout.addLayout(stack_buttons)

        self.details = QtWidgets.QGroupBox("Behavior Arguments")
        self.details_layout = QtWidgets.QFormLayout(self.details)
        middle_layout.addWidget(self.details)
        central.addWidget(middle)

        right = QtWidgets.QSplitter(QtCore.Qt.Vertical)
        script_box = QtWidgets.QWidget()
        script_layout = QtWidgets.QVBoxLayout(script_box)
        script_layout.setContentsMargins(8, 8, 8, 8)

        meta = QtWidgets.QWidget()
        meta_layout = QtWidgets.QFormLayout(meta)
        self.filename_edit = QtWidgets.QLineEdit()
        self.filename_edit.editingFinished.connect(self._sync_meta_from_fields)
        self.state_edit = QtWidgets.QLineEdit()
        self.state_edit.editingFinished.connect(self._sync_meta_from_fields)
        meta_layout.addRow("Filename", self.filename_edit)
        meta_layout.addRow("State", self.state_edit)

        timing_widget = QtWidgets.QWidget()
        timing_row = QtWidgets.QHBoxLayout(timing_widget)
        timing_row.setContentsMargins(0, 0, 0, 0)
        self.mission_timer_label = QtWidgets.QLabel("Mission timer: --")
        self.estimated_time_label = QtWidgets.QLabel("Estimated time: --")
        timing_row.addWidget(self.mission_timer_label)
        timing_row.addSpacing(16)
        timing_row.addWidget(self.estimated_time_label)
        timing_row.addStretch(1)
        meta_layout.addRow("Timing", timing_widget)
        script_layout.addWidget(meta)

        self.script_edit = QtWidgets.QPlainTextEdit()
        self.script_edit.setLineWrapMode(QtWidgets.QPlainTextEdit.NoWrap)
        script_layout.addWidget(self.script_edit, 1)
        script_buttons = QtWidgets.QHBoxLayout()
        apply_btn = QtWidgets.QPushButton("Apply Script Edits")
        apply_btn.clicked.connect(self._apply_script_edits)
        refresh_btn = QtWidgets.QPushButton("Refresh Script")
        refresh_btn.clicked.connect(self._refresh_script)
        script_buttons.addWidget(apply_btn)
        script_buttons.addWidget(refresh_btn)
        script_layout.addLayout(script_buttons)
        right.addWidget(script_box)

        results_box = QtWidgets.QWidget()
        results_layout = QtWidgets.QVBoxLayout(results_box)
        results_layout.setContentsMargins(8, 8, 8, 8)
        results_layout.addWidget(QtWidgets.QLabel("Ksim Results"))
        self.results = QtWidgets.QPlainTextEdit()
        self.results.setReadOnly(True)
        results_layout.addWidget(self.results)
        right.addWidget(results_box)
        central.addWidget(right)

        central.setSizes([270, 430, 620])
        right.setSizes([560, 220])

        self.checks_tab = QtWidgets.QWidget()
        checks_layout = QtWidgets.QVBoxLayout(self.checks_tab)
        checks_layout.setContentsMargins(8, 8, 8, 8)
        checks_layout.addWidget(QtWidgets.QLabel("Mission Checks"))
        self.checks_list = QtWidgets.QTreeWidget()
        self.checks_list.setHeaderLabels(["Severity", "Behavior", "Argument", "Message"])
        self.checks_list.setRootIsDecorated(False)
        checks_layout.addWidget(self.checks_list, 1)
        checks_layout.addWidget(QtWidgets.QLabel("Estimated Behavior Timing"))
        self.timing_table = QtWidgets.QTreeWidget()
        self.timing_table.setHeaderLabels(["Priority", "Behavior", "Estimate", "Depth", "Basis"])
        self.timing_table.setRootIsDecorated(False)
        checks_layout.addWidget(self.timing_table, 1)

        self.visual_tab = QtWidgets.QWidget()
        visual_layout = QtWidgets.QVBoxLayout(self.visual_tab)
        visual_layout.setContentsMargins(8, 8, 8, 8)
        self.visual_tabs = QtWidgets.QTabWidget()
        visual_layout.addWidget(self.visual_tabs, 1)

        self.header_tab = QtWidgets.QWidget()
        header_layout = QtWidgets.QVBoxLayout(self.header_tab)
        header_layout.setContentsMargins(8, 8, 8, 8)
        header_layout.addWidget(QtWidgets.QLabel("Header Lines"))
        self.header_text = QtWidgets.QPlainTextEdit()
        self.header_text.setPlaceholderText("Paste or edit mission header lines here.")
        self.header_text.textChanged.connect(self._sync_header_from_tab)
        header_layout.addWidget(self.header_text, 1)

        sensor_buttons = QtWidgets.QHBoxLayout()
        sensor_buttons.addWidget(QtWidgets.QLabel("Initial sensor values"))
        sensor_buttons.addStretch(1)
        add_sensor = QtWidgets.QPushButton("Add Sensor")
        add_sensor.clicked.connect(self._add_header_sensor)
        remove_sensor = QtWidgets.QPushButton("Remove Sensor")
        remove_sensor.clicked.connect(self._remove_header_sensor)
        sensor_buttons.addWidget(add_sensor)
        sensor_buttons.addWidget(remove_sensor)
        header_layout.addLayout(sensor_buttons)

        self.sensor_table = QtWidgets.QTableWidget(0, 2)
        self.sensor_table.setHorizontalHeaderLabels(["Variable", "Value"])
        self.sensor_table.horizontalHeader().setStretchLastSection(True)
        self.sensor_table.itemChanged.connect(self._sync_header_sensors_from_table)
        header_layout.addWidget(self.sensor_table, 1)

        self.failure_tab = QtWidgets.QWidget()
        failure_layout = QtWidgets.QVBoxLayout(self.failure_tab)
        failure_layout.setContentsMargins(8, 8, 8, 8)
        failure_layout.addWidget(QtWidgets.QLabel("Failure Script"))

        fault_buttons = QtWidgets.QHBoxLayout()
        add_fault = QtWidgets.QPushButton("Add Failure")
        add_fault.clicked.connect(self._add_fault_rule)
        remove_fault = QtWidgets.QPushButton("Remove Failure")
        remove_fault.clicked.connect(self._remove_fault_rule)
        save_fault = QtWidgets.QPushButton("Save Failure Script")
        save_fault.clicked.connect(self._save_failure_script)
        fault_buttons.addWidget(add_fault)
        fault_buttons.addWidget(remove_fault)
        fault_buttons.addStretch(1)
        fault_buttons.addWidget(save_fault)
        failure_layout.addLayout(fault_buttons)

        self.failure_table = QtWidgets.QTableWidget(0, 5)
        self.failure_table.setHorizontalHeaderLabels([
            "Trigger Variable", "Op", "Trigger Value", "Target Variable", "Action Value"
        ])
        self.failure_table.horizontalHeader().setStretchLastSection(True)
        self.failure_table.itemChanged.connect(self._sync_faults_from_table)
        failure_layout.addWidget(self.failure_table, 2)

        self.failure_script = QtWidgets.QPlainTextEdit()
        self.failure_script.setReadOnly(True)
        failure_layout.addWidget(self.failure_script, 1)

        self.failure_checks = QtWidgets.QTreeWidget()
        self.failure_checks.setHeaderLabels(["Severity", "Failure", "Variable", "Message"])
        self.failure_checks.setRootIsDecorated(False)
        failure_layout.addWidget(self.failure_checks, 1)

        self.tabs.addTab(constructor_tab, "Mission Constructor")
        self.tabs.addTab(self.header_tab, "Header")
        self.tabs.addTab(self.failure_tab, "Failure Script")
        self.tabs.addTab(self.checks_tab, "Mission Checks")
        self.tabs.addTab(self.visual_tab, "Mission Visualization")
        self._populate_catalog()

    def _toggle_stack_summary(self) -> None:
        self.stack_summary_mode = "depth" if self.stack_summary_mode == "time" else "time"
        self.stack_summary_btn.setText("Show Time" if self.stack_summary_mode == "depth" else "Show Depth")
        selected = self._selected_behavior()
        self._render_all(selected_behavior=selected)

    def _toggle_display_units(self) -> None:
        self.display_units_mode = "friendly" if self.display_units_mode == "native" else "native"
        self.units_btn.setText("Show MKS/rad" if self.display_units_mode == "friendly" else "Show hr/km/deg")
        selected = self._selected_behavior()
        self._render_all(selected_behavior=selected)

    def _new_empty_mission(self, prompt: bool = True) -> None:
        if prompt and not self._confirm_discard_current_mission():
            return
        self.current_path = None
        self.mission = mm.Mission()
        self.mission.behaviors = []
        self.fault_rules = []
        self._render_all()
        self._mark_saved()

    def _new_yoyo_mission(self, prompt: bool = True) -> None:
        if prompt and not self._confirm_discard_current_mission():
            return
        self.current_path = None
        self.mission = mm.make_yoyo_reference_mission(self.catalog_map)
        self.fault_rules = [
            mm.FaultRule("m_depth", ">", "75 m", "vs_xg", "0.082"),
        ]
        self._render_all()
        self._mark_saved()

    def _populate_catalog(self) -> None:
        query = self.search.text().strip().lower() if hasattr(self, "search") else ""
        self.catalog_list.clear()
        for item in self.catalog:
            if query and query not in item.name.lower():
                continue
            list_item = QtWidgets.QListWidgetItem(item.name)
            list_item.setData(QtCore.Qt.UserRole, item.name)
            self.catalog_list.addItem(list_item)

    def _render_all(self, selected_behavior: mm.MissionBehavior | None = None) -> None:
        if selected_behavior is None:
            selected_behavior = self._selected_behavior()
        self.filename_edit.setText(self.mission.filename)
        self.state_edit.setText(self.mission.state)
        self.stack_list.clear()
        ordered = sorted(self.mission.behaviors, key=lambda item: item.priority)
        checks = mm.run_mission_checks(self.mission, self.catalog_map)
        severities = self._behavior_check_severities(checks)
        timing_estimates = mm.mission_timing_estimates(self.mission)[0]
        timing_by_priority = {
            estimate.priority: estimate
            for estimate in timing_estimates
        }
        timing_by_name: dict[str, mm.TimingEstimate] = {}
        for estimate in timing_estimates:
            for part in estimate.behavior.split(" + "):
                timing_by_name.setdefault(part, estimate)
        selected_row = 0
        for index, behavior in enumerate(ordered):
            summary = self._stack_behavior_summary(behavior, timing_by_priority, timing_by_name)
            item = QtWidgets.QListWidgetItem(f"P{behavior.priority:<2} {behavior.name:<24} {summary:>16}")
            severity = severities.get(behavior.name, "")
            if severity == "error":
                item.setBackground(QtCore.Qt.red)
                item.setForeground(QtCore.Qt.white)
            elif severity == "warning":
                item.setBackground(QtCore.Qt.yellow)
            self.stack_list.addItem(item)
            if behavior is selected_behavior:
                selected_row = index
        if self.mission.behaviors:
            self.stack_list.setCurrentRow(min(selected_row, len(ordered) - 1))
        self._render_details()
        self._render_header_tab()
        self._refresh_script()
        self._render_checks(checks)
        self._render_timing()
        self._render_failure_tab()

    def _stack_behavior_summary(
        self,
        behavior: mm.MissionBehavior,
        timing_by_priority: dict[int, mm.TimingEstimate],
        timing_by_name: dict[str, mm.TimingEstimate],
    ) -> str:
        estimate = timing_by_priority.get(behavior.priority) or timing_by_name.get(behavior.name)
        if self.stack_summary_mode == "time":
            if estimate is None:
                return ""
            return self._format_seconds(estimate.seconds, compact=True)

        depths: list[float] = []
        if estimate is not None:
            depths.extend([estimate.start_depth, estimate.end_depth])
        for arg_name in ("depth", "end_depth", "top_depth", "bottom_depth", "max_depth"):
            value = mm.parse_arg(behavior, arg_name)
            if value is not None:
                depths.append(value)
        if not depths:
            return ""
        return self._format_depth(max(depths), compact=True)

    def _selected_behavior(self) -> mm.MissionBehavior | None:
        row = self.stack_list.currentRow()
        ordered = sorted(self.mission.behaviors, key=lambda item: item.priority)
        if 0 <= row < len(ordered):
            return ordered[row]
        return None

    def _render_details(self) -> None:
        while self.details_layout.rowCount():
            self.details_layout.removeRow(0)
        self.arg_widgets = []
        behavior = self._selected_behavior()
        if not behavior:
            self.details_layout.addRow(QtWidgets.QLabel("No behavior selected."))
            return

        name = QtWidgets.QLineEdit(behavior.name)
        name.editingFinished.connect(lambda: self._rename_behavior(behavior, name.text()))
        self.details_layout.addRow("Behavior", name)

        priority = QtWidgets.QSpinBox()
        priority.setMinimum(1)
        priority.setMaximum(99)
        priority.blockSignals(True)
        priority.setValue(behavior.priority)
        priority.blockSignals(False)
        priority.valueChanged.connect(lambda value: self._set_priority(behavior, value))
        self.details_layout.addRow("Priority", priority)

        for arg in behavior.args:
            edit = QtWidgets.QLineEdit(str(arg.value))
            edit.setPlaceholderText("Examples: 90 deg, 2 nm, 1 hr, 3 kn")
            edit.editingFinished.connect(self._sync_args_from_widgets)
            self.arg_widgets.append((arg, edit))
            row = QtWidgets.QWidget()
            row_layout = QtWidgets.QHBoxLayout(row)
            row_layout.setContentsMargins(0, 0, 0, 0)
            row_layout.addWidget(edit, 1)
            display = QtWidgets.QLabel(self._format_arg_display(arg, behavior))
            display.setMinimumWidth(95)
            display.setAlignment(QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter)
            row_layout.addWidget(display)
            self.details_layout.addRow(arg.name, row)

    def _refresh_script(self) -> None:
        self._sync_header_from_tab()
        self._sync_header_sensors_from_table()
        self._sync_meta_from_fields()
        self._sync_args_from_widgets()
        self.script_edit.setPlainText(mm.build_mission_text(self.mission))
        self._render_checks()
        self._render_timing()

    def _make_sensor_combo(self, value: str, preferred_prefixes: tuple[str, ...] | None = None) -> QtWidgets.QComboBox:
        combo = QtWidgets.QComboBox()
        combo.setEditable(True)
        names = self.vehicle_sensor_names
        if preferred_prefixes:
            preferred = [n for n in names if mm.base_name(n).startswith(preferred_prefixes)]
            rest = [n for n in names if n not in preferred]
            names = preferred + rest
        combo.addItems(names)
        if value:
            index = combo.findText(value)
            if index < 0:
                index = combo.findText(mm.base_name(value), QtCore.Qt.MatchFixedString)
            if index >= 0:
                combo.setCurrentIndex(index)
            else:
                combo.setEditText(value)
        return combo

    def _make_variable_combo(self, value: str, preferred_prefixes: tuple[str, ...] | None = None) -> QtWidgets.QComboBox:
        combo = QtWidgets.QComboBox()
        combo.setEditable(True)
        names = self.vehicle_variables
        if preferred_prefixes:
            preferred = [n for n in names if n.startswith(preferred_prefixes)]
            rest = [n for n in names if n not in preferred]
            names = preferred + rest
        combo.addItems(names)
        if value:
            index = combo.findText(mm.base_name(value))
            if index >= 0:
                combo.setCurrentIndex(index)
            else:
                combo.setEditText(mm.base_name(value))
        return combo

    def _render_header_tab(self) -> None:
        if not hasattr(self, "header_text"):
            return
        self.header_text.blockSignals(True)
        self.header_text.setPlainText(self.mission.header)
        self.header_text.blockSignals(False)

        self.sensor_table_syncing = True
        self.sensor_table.setRowCount(len(self.mission.sensors))
        for row, sensor in enumerate(self.mission.sensors):
            combo = self._make_sensor_combo(sensor.name)
            combo.currentTextChanged.connect(self._sync_header_sensors_from_table)
            self.sensor_table.setCellWidget(row, 0, combo)
            self.sensor_table.setItem(row, 1, QtWidgets.QTableWidgetItem(str(sensor.value)))
        self.sensor_table.resizeColumnsToContents()
        self.sensor_table_syncing = False

    def _sync_header_from_tab(self) -> None:
        if hasattr(self, "header_text"):
            self.mission.header = self.header_text.toPlainText().strip()

    def _sync_header_sensors_from_table(self) -> None:
        if self.sensor_table_syncing or not hasattr(self, "sensor_table"):
            return
        sensors: list[mm.MissionSensor] = []
        for row in range(self.sensor_table.rowCount()):
            combo = self.sensor_table.cellWidget(row, 0)
            name = combo.currentText().strip() if isinstance(combo, QtWidgets.QComboBox) else ""
            item = self.sensor_table.item(row, 1)
            value = item.text().strip() if item else "0"
            if name:
                sensors.append(mm.MissionSensor(name, value))
        self.mission.sensors = sensors
        self._render_checks()

    def _add_header_sensor(self) -> None:
        self._sync_header_sensors_from_table()
        self.mission.sensors.append(mm.MissionSensor("m_depth(m)", "0"))
        self._render_header_tab()
        self._refresh_script()

    def _remove_header_sensor(self) -> None:
        row = self.sensor_table.currentRow()
        self._sync_header_sensors_from_table()
        if 0 <= row < len(self.mission.sensors):
            self.mission.sensors.pop(row)
        self._render_header_tab()
        self._refresh_script()

    def _sync_meta_from_fields(self) -> None:
        if hasattr(self, "filename_edit"):
            self.mission.filename = self.filename_edit.text().strip() or "mission.mi"
            self.mission.state = self.state_edit.text().strip() or "mission"

    def _sync_args_from_widgets(self) -> None:
        behavior = self._selected_behavior()
        for arg, edit in self.arg_widgets:
            raw = edit.text().strip()
            arg.value = raw
            try:
                converted, note = mm.convert_value_expression(arg.name, raw, behavior)
            except ValueError:
                edit.setStyleSheet("background: #fff0f0;")
                continue
            if note:
                arg.value = f"{converted:.8g}"
                edit.setText(arg.value)
                edit.setToolTip(note)
                self.statusBar().showMessage(note, 5000)
            edit.setStyleSheet("")
        self._render_stack_checks()
        self._render_checks()
        self._render_timing()

    def _rename_behavior(self, behavior: mm.MissionBehavior, name: str) -> None:
        behavior.name = name.strip() or behavior.name
        self._render_all()

    def _set_priority(self, behavior: mm.MissionBehavior, value: int) -> None:
        ordered = sorted(self.mission.behaviors, key=lambda item: item.priority)
        if behavior not in ordered:
            return
        ordered.remove(behavior)
        target = max(0, min(value - 1, len(ordered)))
        ordered.insert(target, behavior)
        self.mission.behaviors = ordered
        mm.renumber_behaviors(self.mission)
        self._render_all(selected_behavior=behavior)

    def _add_catalog_behavior(self) -> None:
        item = self.catalog_list.currentItem()
        if not item:
            return
        name = item.data(QtCore.Qt.UserRole)
        priority = len(self.mission.behaviors) + 1
        self.mission.behaviors.append(mm.create_behavior(name, priority, self.catalog_map))
        mm.renumber_behaviors(self.mission)
        self._render_all()
        self.stack_list.setCurrentRow(len(self.mission.behaviors) - 1)

    def _move_selected(self, delta: int) -> None:
        ordered = sorted(self.mission.behaviors, key=lambda item: item.priority)
        row = self.stack_list.currentRow()
        target = row + delta
        if not (0 <= row < len(ordered) and 0 <= target < len(ordered)):
            return
        ordered[row], ordered[target] = ordered[target], ordered[row]
        self.mission.behaviors = ordered
        mm.renumber_behaviors(self.mission)
        self._render_all(selected_behavior=ordered[target])

    def _duplicate_selected(self) -> None:
        behavior = self._selected_behavior()
        if not behavior:
            return
        copy = mm.MissionBehavior(
            behavior.name,
            len(self.mission.behaviors) + 1,
            [mm.MissionSensor(arg.name, arg.value) for arg in behavior.args],
        )
        self.mission.behaviors.append(copy)
        mm.renumber_behaviors(self.mission)
        self._render_all()

    def _remove_selected(self) -> None:
        behavior = self._selected_behavior()
        if not behavior:
            return
        self.mission.behaviors.remove(behavior)
        mm.renumber_behaviors(self.mission)
        self._render_all()

    def _apply_script_edits(self) -> None:
        self.mission = mm.parse_mission_text(
            self.script_edit.toPlainText(), self.mission.filename
        )
        self.current_path = None
        self._render_all()

    def _behavior_check_severities(self, checks: list[mm.MissionCheck]) -> dict[str, str]:
        severities: dict[str, str] = {}
        rank = {"": 0, "ok": 0, "warning": 1, "error": 2}
        for check in checks:
            if not check.behavior:
                continue
            current = severities.get(check.behavior, "")
            if rank.get(check.severity, 0) > rank.get(current, 0):
                severities[check.behavior] = check.severity
        return severities

    def _render_stack_checks(self, checks: list[mm.MissionCheck] | None = None) -> None:
        if not hasattr(self, "stack_list"):
            return
        if checks is None:
            checks = mm.run_mission_checks(self.mission, self.catalog_map)
        severities = self._behavior_check_severities(checks)
        ordered = sorted(self.mission.behaviors, key=lambda item: item.priority)
        for row, behavior in enumerate(ordered):
            item = self.stack_list.item(row)
            if item is None:
                continue
            item.setBackground(QtGui.QBrush())
            item.setForeground(QtGui.QBrush())
            severity = severities.get(behavior.name, "")
            if severity == "error":
                item.setBackground(QtCore.Qt.red)
                item.setForeground(QtCore.Qt.white)
            elif severity == "warning":
                item.setBackground(QtCore.Qt.yellow)

    def _render_checks(self, checks: list[mm.MissionCheck] | None = None) -> None:
        if not hasattr(self, "checks_list"):
            return
        if checks is None:
            checks = mm.run_mission_checks(self.mission, self.catalog_map)
        self.checks_list.clear()
        for check in checks:
            item = QtWidgets.QTreeWidgetItem([
                check.severity.upper(),
                check.behavior,
                check.argument,
                check.message,
            ])
            if check.severity == "error":
                item.setBackground(0, QtCore.Qt.red)
                item.setForeground(0, QtCore.Qt.white)
            elif check.severity == "warning":
                item.setBackground(0, QtCore.Qt.yellow)
            elif check.severity == "ok":
                item.setBackground(0, QtCore.Qt.green)
            self.checks_list.addTopLevelItem(item)
        self.checks_list.resizeColumnToContents(0)
        self.checks_list.resizeColumnToContents(1)
        self.checks_list.resizeColumnToContents(2)

    def _format_seconds(self, seconds: float | None, compact: bool = False) -> str:
        if seconds is None:
            return "--"
        if self.display_units_mode == "friendly":
            if compact:
                return f"{seconds / 3600.0:.2f} hr"
            return f"{seconds:.0f} s ({seconds / 3600.0:.2f} hr)"
        if compact:
            return f"{seconds:.0f} s"
        return f"{seconds:.1f} s"

    def _format_depth(self, meters: float | None, compact: bool = False) -> str:
        if meters is None:
            return "--"
        if self.display_units_mode == "friendly":
            if abs(meters) >= 1000.0:
                return f"{meters / 1000.0:.3g} km"
            return f"{meters:.3g} m"
        return f"{meters:.3g} m"

    def _format_angle(self, radians: float | None) -> str:
        if radians is None:
            return "--"
        if self.display_units_mode == "friendly":
            return f"{np.degrees(radians):.4g} deg"
        return f"{radians:.6g} rad"

    def _format_speed(self, speed: float | None) -> str:
        if speed is None:
            return "--"
        return f"{speed:.4g} m/s"

    def _format_arg_display(self, arg: mm.MissionSensor, behavior: mm.MissionBehavior) -> str:
        try:
            value = mm.convert_value_expression(arg.name, arg.value, behavior)[0]
        except ValueError:
            return "invalid"
        unit = mm.unit_name(arg.name)
        name = mm.base_name(arg.name)
        if "rad" in unit or name in ("heading", "turn", "trans_heading", "heading_leg1", "heading_leg2"):
            return self._format_angle(value)
        if unit in ("s", "sec") or name in ("time", "max_time", "timeout", "duration", "trans_time", "time_leg1", "time_leg2"):
            return self._format_seconds(value, compact=True)
        if unit == "m/s" or name.endswith("speed"):
            return self._format_speed(value)
        if "m" in unit or name in ("depth", "end_depth", "top_depth", "bottom_depth", "max_depth", "min_depth", "cutoff_depth", "min_altitude"):
            return self._format_depth(value, compact=True)
        return f"{value:.6g}"

    def _render_timing(self) -> None:
        if not hasattr(self, "timing_table"):
            return
        estimates, problems = mm.mission_timing_estimates(self.mission)
        total = sum(item.seconds for item in estimates if item.seconds is not None)
        timer = mm.mission_timer_seconds(self.mission)
        self.mission_timer_label.setText(f"Mission timer: {self._format_seconds(timer)}")
        self.estimated_time_label.setText(f"Estimated time: {self._format_seconds(total)}")
        if timer is not None and total > 0.0 and timer < total:
            style = "color: #cc0000; font-weight: bold;"
        elif timer is not None and total > 0.0 and timer < total * 1.1:
            style = "color: #cc0000; font-weight: bold;"
        else:
            style = "color: #008000; font-weight: bold;" if timer is not None and total > 0.0 else ""
        self.mission_timer_label.setStyleSheet(style)
        self.estimated_time_label.setStyleSheet(style)

        self.timing_table.clear()
        for estimate in estimates:
            depth_text = f"{self._format_depth(estimate.start_depth, True)} -> {self._format_depth(estimate.end_depth, True)}"
            item = QtWidgets.QTreeWidgetItem([
                str(estimate.priority),
                estimate.behavior,
                self._format_seconds(estimate.seconds),
                depth_text,
                estimate.note,
            ])
            if estimate.seconds is None:
                item.setBackground(2, QtCore.Qt.yellow)
            self.timing_table.addTopLevelItem(item)
        for problem in problems:
            item = QtWidgets.QTreeWidgetItem(["", "", "--", "", problem])
            item.setBackground(2, QtCore.Qt.yellow)
            self.timing_table.addTopLevelItem(item)
        for column in range(5):
            self.timing_table.resizeColumnToContents(column)

    def _render_failure_tab(self) -> None:
        if not hasattr(self, "failure_table"):
            return
        self.failure_table_syncing = True
        self.failure_table.setRowCount(len(self.fault_rules))
        for row, rule in enumerate(self.fault_rules):
            trigger_combo = self._make_variable_combo(rule.trigger_variable, ("m_", "c_", "real_"))
            trigger_combo.currentTextChanged.connect(self._sync_faults_from_table)
            self.failure_table.setCellWidget(row, 0, trigger_combo)

            op_combo = QtWidgets.QComboBox()
            op_combo.addItems([">", ">=", "<", "<=", "==", "!="])
            op_index = op_combo.findText(rule.operator)
            op_combo.setCurrentIndex(op_index if op_index >= 0 else 0)
            op_combo.currentTextChanged.connect(self._sync_faults_from_table)
            self.failure_table.setCellWidget(row, 1, op_combo)

            self.failure_table.setItem(row, 2, QtWidgets.QTableWidgetItem(str(rule.trigger_value)))

            target_combo = self._make_variable_combo(rule.target_variable, ("vs_", "uo_", "c_"))
            target_combo.currentTextChanged.connect(self._sync_faults_from_table)
            self.failure_table.setCellWidget(row, 3, target_combo)

            self.failure_table.setItem(row, 4, QtWidgets.QTableWidgetItem(str(rule.action_value)))
        self.failure_table_syncing = False
        self.failure_table.resizeColumnsToContents()
        self._refresh_failure_script()

    def _sync_faults_from_table(self) -> None:
        if self.failure_table_syncing:
            return
        rules: list[mm.FaultRule] = []
        for row in range(self.failure_table.rowCount()):
            trigger = self.failure_table.cellWidget(row, 0)
            op = self.failure_table.cellWidget(row, 1)
            trigger_value = self.failure_table.item(row, 2)
            target = self.failure_table.cellWidget(row, 3)
            action_value = self.failure_table.item(row, 4)
            rules.append(mm.FaultRule(
                trigger.currentText().strip() if isinstance(trigger, QtWidgets.QComboBox) else "",
                op.currentText().strip() if isinstance(op, QtWidgets.QComboBox) else ">",
                trigger_value.text().strip() if trigger_value else "0",
                target.currentText().strip() if isinstance(target, QtWidgets.QComboBox) else "",
                action_value.text().strip() if action_value else "0",
            ))
        self.fault_rules = rules
        self._refresh_failure_script()

    def _refresh_failure_script(self) -> None:
        if not hasattr(self, "failure_script"):
            return
        self.failure_script.setPlainText(mm.build_fault_text(self.fault_rules))
        self.failure_checks.clear()
        for check in mm.run_fault_checks(self.fault_rules, self.mission, self.vehicle_variables):
            item = QtWidgets.QTreeWidgetItem([
                check.severity.upper(),
                check.behavior,
                check.argument,
                check.message,
            ])
            if check.severity == "error":
                item.setBackground(0, QtCore.Qt.red)
                item.setForeground(0, QtCore.Qt.white)
            elif check.severity == "warning":
                item.setBackground(0, QtCore.Qt.yellow)
            elif check.severity == "ok":
                item.setBackground(0, QtCore.Qt.green)
            self.failure_checks.addTopLevelItem(item)
        self.failure_checks.resizeColumnToContents(0)
        self.failure_checks.resizeColumnToContents(1)
        self.failure_checks.resizeColumnToContents(2)

    def _add_fault_rule(self) -> None:
        self._sync_faults_from_table()
        self.fault_rules.append(mm.FaultRule())
        self._render_failure_tab()

    def _remove_fault_rule(self) -> None:
        row = self.failure_table.currentRow()
        if 0 <= row < len(self.fault_rules):
            self.fault_rules.pop(row)
            self._render_failure_tab()

    def _open_mission(self) -> None:
        if not self._confirm_discard_current_mission():
            return
        path, _ = QtWidgets.QFileDialog.getOpenFileName(
            self, "Open mission", str(ROOT / "SMission"), "Mission files (*.mi *.txt);;All files (*)"
        )
        if not path:
            return
        self.current_path = Path(path)
        self.mission = mm.parse_mission_text(
            self.current_path.read_text(encoding="latin-1"), self.current_path.name
        )
        self._render_all()
        self._mark_saved()

    def _save_mission(self) -> bool:
        if self.current_path is None:
            return self._save_mission_as()
        self._refresh_script()
        self.current_path.write_text(self.script_edit.toPlainText(), encoding="latin-1")
        self._mark_saved()
        self.statusBar().showMessage(f"Saved {self.current_path}", 4000)
        return True

    def _save_mission_as(self) -> bool:
        path, _ = QtWidgets.QFileDialog.getSaveFileName(
            self, "Save mission", str(ROOT / "SMission" / self.mission.filename),
            "Mission files (*.mi);;All files (*)",
        )
        if not path:
            return False
        self.current_path = Path(path)
        self.mission.filename = self.current_path.name
        return self._save_mission()

    def _current_mission_text(self) -> str:
        self._sync_header_from_tab()
        self._sync_header_sensors_from_table()
        self._sync_meta_from_fields()
        self._sync_args_from_widgets()
        return mm.build_mission_text(self.mission)

    def _has_unsaved_mission_changes(self) -> bool:
        if not hasattr(self, "script_edit"):
            return False
        return self._current_mission_text() != self.baseline_mission_text

    def _mark_saved(self) -> None:
        self.baseline_mission_text = mm.build_mission_text(self.mission)

    def _confirm_discard_current_mission(self) -> bool:
        if not self._has_unsaved_mission_changes():
            return True
        response = QtWidgets.QMessageBox.question(
            self,
            "Save current mission?",
            "Save the current mission before replacing it?",
            QtWidgets.QMessageBox.Save | QtWidgets.QMessageBox.Discard | QtWidgets.QMessageBox.Cancel,
            QtWidgets.QMessageBox.Save,
        )
        if response == QtWidgets.QMessageBox.Save:
            return self._save_mission()
        if response == QtWidgets.QMessageBox.Discard:
            return True
        return False

    def _save_failure_script(self) -> None:
        self._sync_faults_from_table()
        default_name = Path(self.mission.filename).with_suffix(".fd").name
        path, _ = QtWidgets.QFileDialog.getSaveFileName(
            self,
            "Save failure script",
            str(ROOT / "SFault" / default_name),
            "Fault files (*.fd);;All files (*)",
        )
        if not path:
            return
        Path(path).write_text(mm.build_fault_text(self.fault_rules), encoding="latin-1")
        self.statusBar().showMessage(f"Saved failure script {path}", 4000)

    def _run_ksim(self) -> None:
        self._refresh_script()
        ksim = ROOT / ("ksim.exe" if sys.platform.startswith("win") else "ksim")
        if not ksim.exists():
            QtWidgets.QMessageBox.warning(self, "ksim not found", f"Build ksim first:\n{ksim}")
            return

        stamp = _dt.datetime.now().strftime("%Y%m%d_%H%M%S")
        mission_file = RUN_DIR / f"{stamp}_{Path(self.mission.filename).name}"
        behavior_file = RUN_DIR / f"behavior_{stamp}.json"
        mission_file.write_text(self.script_edit.toPlainText(), encoding="latin-1")

        command = [str(ksim), "--dump-behavior", str(behavior_file), str(mission_file)]
        self.results.setPlainText("Running:\n" + " ".join(command))
        QtWidgets.QApplication.processEvents()
        proc = subprocess.run(command, cwd=str(ROOT), capture_output=True, text=True)

        text = []
        text.append(f"Exit code: {proc.returncode}")
        text.append(f"Mission: {mission_file}")
        text.append(f"Behavior log: {behavior_file}")
        if behavior_file.exists():
            try:
                rows = json.loads(behavior_file.read_text(encoding="utf-8"))
                if rows:
                    first, last = rows[0], rows[-1]
                    text.append(
                        f"Behavior samples: {len(rows)} from {first.get('time', 0):.1f}s to {last.get('time', 0):.1f}s"
                    )
            except Exception as exc:
                text.append(f"Could not read behavior log: {exc}")
        if proc.stdout.strip():
            text.append("\nstdout:\n" + proc.stdout.strip())
        if proc.stderr.strip():
            text.append("\nstderr:\n" + proc.stderr.strip())
        self.results.setPlainText("\n".join(text))
        self._populate_visualization(proc.stdout + "\n" + proc.stderr)

    def _datafile_from_output(self, text: str) -> Path | None:
        for line in text.splitlines():
            if "datafile:" in line:
                path = line.split("datafile:", 1)[1].strip()
                if path:
                    return Path(path)
            if "Mission finished -- data file:" in line:
                path = line.split("Mission finished -- data file:", 1)[1].strip()
                if path:
                    return Path(path)
        return None

    def _populate_visualization(self, output: str) -> None:
        self.visual_tabs.clear()
        datafile = self._datafile_from_output(output)
        if not datafile or not datafile.exists():
            msg = QtWidgets.QPlainTextEdit()
            msg.setReadOnly(True)
            msg.setPlainText("No logged .obd data file was found for this run.")
            self.visual_tabs.addTab(msg, "Run Output")
            self.tabs.setCurrentWidget(self.visual_tab)
            return

        if obd is None:
            msg = QtWidgets.QPlainTextEdit()
            msg.setReadOnly(True)
            msg.setPlainText("Review plotting modules are not importable in this Python environment.")
            self.visual_tabs.addTab(msg, "Run Output")
            self.tabs.setCurrentWidget(self.visual_tab)
            return

        try:
            ds = conditioning.condition(obd.read_obd(datafile))
            figure_specs = [
                ("Plan View", plots.navplot),
                ("Depth / Pitch", _plot_ksim_depth_pitch),
                ("Heading / Speed", _plot_ksim_heading_speed),
                ("Behavior Axes", panels.behavior_axis_waterfall),
            ]
            for title, func in figure_specs:
                try:
                    fig = func(ds)
                except Exception as exc:
                    fig = plt.figure(figsize=(8, 4))
                    ax = fig.add_subplot(111)
                    ax.text(0.5, 0.5, f"{title} unavailable:\n{exc}",
                            ha="center", va="center", transform=ax.transAxes)
                    ax.axis("off")
                self._ensure_copy_button(fig)
                self.visual_tabs.addTab(FigureTab(fig), title)
        except Exception as exc:
            msg = QtWidgets.QPlainTextEdit()
            msg.setReadOnly(True)
            msg.setPlainText(f"Could not load {datafile}:\n{exc}")
            self.visual_tabs.addTab(msg, "Run Output")
        self.tabs.setCurrentWidget(self.visual_tab)

    def _ensure_copy_button(self, fig: plt.Figure) -> None:
        if plots is None or hasattr(fig, "_copy_button"):
            return
        add_copy = getattr(plots, "_add_copy_button", None)
        if add_copy is not None:
            add_copy(fig)


def main() -> None:
    app = QtWidgets.QApplication(sys.argv)
    win = MissionConstructor()
    win.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
