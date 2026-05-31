#!/usr/bin/env python3
"""Internal layer masks used to assemble an active-map snapshot."""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List

import numpy as np

try:
    from control_module.constants import Rect
except ImportError:
    from constants import Rect


@dataclass
class LayeredMaskResult:
    """Internal/debug masks produced by one semantic map-update request.

    The control-facing 2-class contract is exposed through
    ``ActiveMapSnapshot.blocked_mask``, ``final_active_mask``, and
    ``occupied_mask``.  These per-class masks remain available for diagnostics,
    artifact forensics, and paper evidence only.
    """

    occupied_mask: np.ndarray
    target_mask: np.ndarray
    unknown_mask: np.ndarray
    occluded_mask: np.ndarray
    semantic_mask: np.ndarray
    inflated_mask: np.ndarray
    inflation_added_mask: np.ndarray
    sensor_inflated_mask: np.ndarray
    sensor_inflation_added_mask: np.ndarray
    occlusion_inflated_mask: np.ndarray
    occlusion_inflation_added_mask: np.ndarray
    blocked_mask: np.ndarray
    display_rects: List[Rect] = field(default_factory=list)
    generated_occluded_rect_count: int = 0
    total_occluded_rect_count: int = 0

    def as_dict(self) -> Dict[str, np.ndarray]:
        return {
            "occupied": self.occupied_mask,
            "target": self.target_mask,
            "unknown": self.unknown_mask,
            "occluded": self.occluded_mask,
            "semantic": self.semantic_mask,
            "inflated": self.inflated_mask,
            "inflation_added": self.inflation_added_mask,
            "sensor_inflated": self.sensor_inflated_mask,
            "sensor_inflation_added": self.sensor_inflation_added_mask,
            "occlusion_inflated": self.occlusion_inflated_mask,
            "occlusion_inflation_added": self.occlusion_inflation_added_mask,
            "blocked": self.blocked_mask,
        }
