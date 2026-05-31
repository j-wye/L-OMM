#!/usr/bin/env python3
"""Single line/blend segment carrying dense xz samples (pitch reconstructed later).

Per spec §10, pitch is reconstructed *after* the dense xz table is assembled
(via ``gaze_pitch.GazePitch``).  Segments therefore only carry geometric
data — pitch is no longer a per-segment field.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import numpy as np


@dataclass
class BlendSegment:
    kind: str                          # "line" | "blend"
    xz: np.ndarray                     # (n, 2) dense xz samples
    s_local: Optional[np.ndarray] = None   # (n,) arclength along the segment, starting at 0
    corner_index: int = -1
    radius: float = 0.0
    curvature_lift: bool = False       # spec §6 monitor — True if r_curv > r_geo and lifted
    curvature_overflow: bool = False   # spec §6 monitor — True if r_curv > r_max
