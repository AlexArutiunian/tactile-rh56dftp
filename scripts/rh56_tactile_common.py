#!/usr/bin/env python3
"""Compatibility import for scripts copied from the working prototype."""
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from rh56_tactile.common import *  # noqa: F401,F403

