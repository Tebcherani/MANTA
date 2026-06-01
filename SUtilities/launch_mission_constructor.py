#!/usr/bin/env python3
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parent))

try:
    from mission_constructor_gui import main
except ImportError:
    sys.exit(
        "PyQt5 is not installed in the current Python environment.\n"
        "Activate the Review virtual environment or install PyQt5, then re-run."
    )


if __name__ == "__main__":
    main()
