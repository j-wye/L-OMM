#!/usr/bin/env python3
"""Single-tick reduced reference output."""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass
class ReferenceSample:
    pos: np.ndarray         # (3,) [x, y_plane, z], used for diagnostics/gap.
    pitch: float            # gaze pitch (rad)
    r_dot: np.ndarray       # (3,) unit-speed tangent [dx/ds, dz/ds, dpitch/ds]
    wp_index: int           # nearest source-waypoint index
