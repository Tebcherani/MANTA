"""Console entry point for the Odyssey AUV Review GUI."""

from __future__ import annotations

import sys

import matplotlib


def main() -> None:
    """Launch the tabbed PyQt review interface."""
    matplotlib.use("Qt5Agg")

    from PyQt5 import QtWidgets

    from .app import MainWindow

    app = QtWidgets.QApplication(sys.argv)
    app.setApplicationName("Odyssey AUV Review")
    app.setOrganizationName("MIT Sea Grant AUV Lab")

    win = MainWindow()
    win.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
