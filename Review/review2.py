#!/usr/bin/env python3
"""Launch the Odyssey review2 compare tool."""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))

from ody_review.review2 import main

if __name__ == "__main__":
    main()
