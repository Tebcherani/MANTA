#!/usr/bin/env python3
"""
Launch the Odyssey AUV Review GUI.

Usage (from the Windows_Ody_C directory or the Review/ directory):
    python Review/launch_review.py
    python launch_review.py          # if already inside Review/

The GUI automatically scans ../SData/ (relative to this script) for .obd files.
Use the Refresh button if you add new data files while the GUI is open.

Requirements: PyQt5, matplotlib (Qt5Agg backend), xarray, numpy
All are installed in the Review/.venv virtual environment.

Activate the venv first if needed:
    .venv\\Scripts\\activate          (Windows)
    source .venv/bin/activate         (Linux / macOS)
"""
from pathlib import Path

# Make the ody_review package importable regardless of where the script is run
import sys
sys.path.insert(0, str(Path(__file__).parent))

try:
    from ody_review.gui import main
except ImportError:
    sys.exit(
        "PyQt5 is not installed in the current Python environment.\n"
        "Activate the Review venv first:\n"
        "  .venv\\Scripts\\activate   (Windows)\n"
        "  source .venv/bin/activate  (Linux / macOS)\n"
        "Then re-run this script."
    )


if __name__ == "__main__":
    main()
