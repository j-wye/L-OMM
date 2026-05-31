#!/usr/bin/env python3
"""Final active-mask snapshot passed into the control module."""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List

import numpy as np

try:  # package import when L-OMM/scripts is on sys.path
    from control_module.constants import Rect
    from control_module.map_handle import MapHandle
except ImportError:  # direct execution with control_module on sys.path
    from constants import Rect
    from map_handle import MapHandle


@dataclass
class ActiveMapSnapshot:
    """Map-update output contract consumed by planning/control.

    Externally exposed planning masks are ``final_active_mask`` and
    ``blocked_mask``.  ``occupied_mask`` is exposed only for sticky-update
    bookkeeping.  ``layer_masks`` is debug-only; new planning/control code
    should not depend on per-class semantic masks.
    """

    handle: MapHandle
    base_feasible_mask: np.ndarray
    final_active_mask: np.ndarray
    blocked_mask: np.ndarray
    occupied_mask: np.ndarray | None = None
    display_rects: List[Rect] = field(default_factory=list)
    stats: Dict[str, object] = field(default_factory=dict)
    layer_masks: Dict[str, np.ndarray] = field(default_factory=dict)
    source: str = "map_update_layer"
    mu_min: float = 0.0

    def __post_init__(self) -> None:
        shape = self.handle.shape
        for name, mask in (
            ("base_feasible_mask", self.base_feasible_mask),
            ("final_active_mask", self.final_active_mask),
            ("blocked_mask", self.blocked_mask),
        ):
            arr = np.asarray(mask)
            if arr.shape != shape:
                raise ValueError(f"{name} shape {arr.shape} does not match map shape {shape}")
        self.base_feasible_mask = np.asarray(self.base_feasible_mask, dtype=bool)
        self.final_active_mask = np.asarray(self.final_active_mask, dtype=bool)
        self.blocked_mask = np.asarray(self.blocked_mask, dtype=bool)
        if self.occupied_mask is not None:
            occupied = np.asarray(self.occupied_mask, dtype=bool)
            if occupied.shape != shape:
                raise ValueError(f"occupied_mask shape {occupied.shape} does not match map shape {shape}")
            self.occupied_mask = occupied
        cleaned_layers: Dict[str, np.ndarray] = {}
        for name, mask in dict(self.layer_masks).items():
            arr = np.asarray(mask, dtype=bool)
            if arr.shape != shape:
                raise ValueError(f"layer mask {name} shape {arr.shape} does not match map shape {shape}")
            cleaned_layers[str(name)] = arr
        self.layer_masks = cleaned_layers
