#!/usr/bin/env python3
# Offline post-processing of the latest rosbag2 recording.
from __future__ import annotations

import sys
from pathlib import Path

_PATH = Path(__file__).resolve().parent
if str(_PATH) not in sys.path:
    sys.path.insert(0, str(_PATH))

from bag_statistics_lib import main

if __name__ == '__main__':
    main()
