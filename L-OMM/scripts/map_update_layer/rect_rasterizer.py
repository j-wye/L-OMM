#!/usr/bin/env python3
"""Rectangle normalization, inflation, shadowing, and grid rasterization."""
from __future__ import annotations

import math
from typing import Sequence

import numpy as np

try:
    from control_module.constants import Rect
    from control_module.map_handle import MapHandle
except ImportError:
    from constants import Rect
    from map_handle import MapHandle


class RectRasterizer:
    """Rasterize axis-aligned x-z rectangles onto a reduced active map."""

    def rect_mask(self, handle: MapHandle, rects: Sequence[Rect]) -> np.ndarray:
        n_x, n_z = handle.shape
        mask = np.zeros((n_x, n_z), dtype=bool)
        x_min = float(handle.x0)
        z_min = float(handle.z0)
        x_max = x_min + float(n_x - 1) * float(handle.resolution_m)
        z_max = z_min + float(n_z - 1) * float(handle.resolution_m)
        for rect in rects:
            xl, zt, xr, zb = self.normalize(rect)
            if xr < x_min or xl > x_max or zt < z_min or zb > z_max:
                continue
            ix0 = int(math.floor((xl - handle.x0) / handle.resolution_m))
            ix1 = int(math.ceil((xr - handle.x0) / handle.resolution_m))
            iz0 = int(math.floor((zb - handle.z0) / handle.resolution_m))
            iz1 = int(math.ceil((zt - handle.z0) / handle.resolution_m))
            ix0 = max(0, min(n_x - 1, ix0))
            ix1 = max(0, min(n_x - 1, ix1))
            iz0 = max(0, min(n_z - 1, iz0))
            iz1 = max(0, min(n_z - 1, iz1))
            if ix1 >= ix0 and iz1 >= iz0:
                mask[ix0:ix1 + 1, iz0:iz1 + 1] = True
        return mask

    @staticmethod
    def normalize(rect: Sequence[float]) -> Rect:
        if len(rect) != 4:
            raise ValueError("rectangle must have four values: x_l,z_t,x_r,z_b")
        xl, zt, xr, zb = (float(rect[0]), float(rect[1]), float(rect[2]), float(rect[3]))
        x_lo, x_hi = (xl, xr) if xl <= xr else (xr, xl)
        z_lo, z_hi = (zb, zt) if zb <= zt else (zt, zb)
        return x_lo, z_hi, x_hi, z_lo

    @staticmethod
    def inflate(rect: Rect, margin: float) -> Rect:
        xl, zt, xr, zb = rect
        m = max(0.0, float(margin))
        return xl - m, zt + m, xr + m, zb - m

    @staticmethod
    def shadow(rect: Rect, direction: str, distance: float) -> Rect:
        xl, zt, xr, zb = rect
        d = max(0.0, float(distance))
        if direction == "+x":
            return xr, zt, xr + d, zb
        if direction == "-x":
            return xl - d, zt, xl, zb
        if direction == "+z":
            return xl, zt + d, xr, zt
        if direction == "-z":
            return xl, zb, xr, zb - d
        return rect
