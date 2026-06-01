"""ody_review — Python framework for reviewing Odyssey .obd vehicle log files."""
from .obd import read_obd
from .conditioning import (
    add_time,
    add_depth_processing,
    add_ctd_processing,
    condition,
)
from .panels import (
    vertical_control,
    heading_panel,
    fault_timeline,
    behavior_axis_waterfall,
)

__all__ = [
    "read_obd",
    "add_time",
    "add_depth_processing",
    "add_ctd_processing",
    "condition",
    "vertical_control",
    "heading_panel",
    "fault_timeline",
    "behavior_axis_waterfall",
]

__version__ = "0.2.0"
