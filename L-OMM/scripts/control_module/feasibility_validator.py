#!/usr/bin/env python3
"""Feasibility validation against the same active-manifold mask used by A*."""
from __future__ import annotations

from typing import Tuple

import numpy as np

from map_handle import MapHandle


class FeasibilityValidator:
    """Validate continuous (xz) samples against a pre-computed feasible mask."""

    def __init__(self, handle: MapHandle, feasible_mask: np.ndarray) -> None:
        self.handle = handle
        self.feasible_mask = np.asarray(feasible_mask, dtype=bool)
        if self.feasible_mask.shape != handle.mu_grid.shape:
            raise ValueError("feasible_mask must match map shape")

    def samples_feasible(self, xz: np.ndarray) -> Tuple[bool, int]:
        pts = np.asarray(xz, dtype=np.float64).reshape(-1, 2)
        bad = 0
        n_x, n_z = self.feasible_mask.shape
        for x, z in pts:
            ix = int(round((float(x) - self.handle.x0) / self.handle.resolution_m))
            iz = int(round((float(z) - self.handle.z0) / self.handle.resolution_m))
            if ix < 0 or ix >= n_x or iz < 0 or iz >= n_z or not self.feasible_mask[ix, iz]:
                bad += 1
        return bad == 0, int(bad)
