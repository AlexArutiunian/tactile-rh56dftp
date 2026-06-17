#!/usr/bin/env python3
"""Compatibility import for the tactile noise filter module."""
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from rh56_tactile.noise_filter import *  # noqa: F401,F403
