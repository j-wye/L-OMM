#!/usr/bin/env python3
"""Immutable handle bundling map grids and metadata."""
from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Dict, Tuple

import numpy as np


@dataclass(frozen=True)
class MapHandle:
    map_path: str
    meta_path: str
    mu_grid: np.ndarray
    pitch_grid: np.ndarray
    q_grid: np.ndarray | None
    meta: Dict[str, Any]
    resolution_m: float
    x0: float
    z0: float
    target_y: float
    tag: str

    @property
    def shape(self) -> Tuple[int, int]:
        return int(self.mu_grid.shape[0]), int(self.mu_grid.shape[1])
