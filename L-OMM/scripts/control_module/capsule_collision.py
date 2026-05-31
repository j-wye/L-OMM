#!/usr/bin/env python3
"""Reduced planar capsule collision primitive for active-manifold planning."""
from __future__ import annotations

import math
from typing import Dict, Optional, Tuple

import numpy as np

from constants import CAPSULE_PROXY_RADII_M, CAPSULE_PROXY_SAMPLE_DS_M
from kinematics import Kinematics
from map_handle import MapHandle


class CapsuleCollision:
    """Fast xz-plane swept-link collision checker.

    The model treats the reduced active arm as a planar serial chain.  Each
    physical link proxy is a line segment between reduced pivots, dilated by a
    conservative radius.  Runtime collision is evaluated against the active-map
    blocked mask; out-of-map link samples are ignored because the active map
    bounds describe EE reference feasibility, not complete robot occupancy.
    """

    def __init__(self,
                 radii_m: Optional[np.ndarray] = None,
                 sample_ds_m: float = CAPSULE_PROXY_SAMPLE_DS_M) -> None:
        radii = CAPSULE_PROXY_RADII_M if radii_m is None else radii_m
        self.radii_m = np.asarray(radii, dtype=np.float64).reshape(-1)
        if self.radii_m.ndim != 1 or self.radii_m.size == 0:
            raise ValueError("radii_m must be a non-empty vector")
        if np.any(~np.isfinite(self.radii_m)) or np.any(self.radii_m < 0.0):
            raise ValueError("capsule radii must be finite and non-negative")
        if sample_ds_m <= 0.0:
            raise ValueError("sample_ds_m must be positive")
        self.sample_ds_m = float(sample_ds_m)
        self.kin = Kinematics()

    def proxy_points_xz(self, q_active: np.ndarray) -> np.ndarray:
        return self.kin.capsule_proxy_points_xz(q_active)

    def build_dilated_masks(self, handle: MapHandle, mask: np.ndarray) -> Dict[float, np.ndarray]:
        base = np.asarray(mask, dtype=bool)
        return {float(r): self._dilate_mask(handle, base, float(r)) for r in np.unique(self.radii_m)}

    def q_hits_mask(self,
                    q_active: np.ndarray,
                    handle: MapHandle,
                    mask: np.ndarray,
                    dilated_masks: Optional[Dict[float, np.ndarray]] = None) -> Tuple[bool, int, int]:
        pts = self.proxy_points_xz(q_active)
        if pts.shape[0] != self.radii_m.size + 1:
            raise ValueError("capsule point/radius count mismatch")
        masks = dilated_masks if dilated_masks is not None else self.build_dilated_masks(handle, mask)
        violations = 0
        checked = 0
        for i, radius in enumerate(self.radii_m):
            inflated = masks[float(radius)]
            hit, hit_count, sample_count = self._segment_hits_mask(handle, pts[i], pts[i + 1], inflated)
            checked += int(sample_count)
            if hit:
                violations += int(hit_count)
        return violations > 0, int(violations), int(checked)

    def trajectory_metrics(self,
                           q_traj: np.ndarray,
                           handle: MapHandle,
                           mask: np.ndarray) -> Dict[str, float]:
        q_arr = np.asarray(q_traj, dtype=np.float64)
        if q_arr.ndim != 2 or q_arr.shape[0] == 0:
            return self._metrics(collision_free=False, violations=0, checked=0)
        dilated = self.build_dilated_masks(handle, mask)
        violations = 0
        checked = 0
        colliding_ticks = 0
        for q in q_arr:
            hit, count, sample_count = self.q_hits_mask(q, handle, mask, dilated)
            checked += int(sample_count)
            if hit:
                colliding_ticks += 1
                violations += int(count)
        return self._metrics(
            collision_free=(violations == 0),
            violations=violations,
            checked=checked,
            colliding_ticks=colliding_ticks,
            total_ticks=int(q_arr.shape[0]),
        )

    def prune_feasible_mask(self,
                            handle: MapHandle,
                            feasible: np.ndarray,
                            blocked_mask: np.ndarray) -> Tuple[np.ndarray, Dict[str, float]]:
        """Remove cells whose stored q-map posture intersects the blocked mask."""
        if handle.q_grid is None:
            return np.asarray(feasible, dtype=bool), {
                "arm_volume_pruning_available": 0.0,
                "arm_volume_checked_cells": 0.0,
                "arm_volume_blocked_cells": 0.0,
            }
        safe = np.asarray(feasible, dtype=bool).copy()
        checked = 0
        blocked = 0
        dilated = self.build_dilated_masks(handle, blocked_mask)
        for ix, iz in np.argwhere(safe):
            checked += 1
            q = np.asarray(handle.q_grid[int(ix), int(iz)], dtype=np.float64).reshape(-1)
            if q.size != 3 or not np.all(np.isfinite(q)):
                safe[int(ix), int(iz)] = False
                blocked += 1
                continue
            if self.q_hits_mask(q, handle, blocked_mask, dilated)[0]:
                safe[int(ix), int(iz)] = False
                blocked += 1
        return safe, {
            "arm_volume_pruning_available": 1.0,
            "arm_volume_proxy": "reduced_planar_capsule",
            "arm_volume_capsule_radius_max_m": float(np.max(self.radii_m)),
            "arm_volume_capsule_sample_ds_m": float(self.sample_ds_m),
            "arm_volume_checked_cells": float(checked),
            "arm_volume_blocked_cells": float(blocked),
            "arm_volume_blocked_ratio_of_checked": float(blocked / max(checked, 1)),
        }

    def _segment_hits_mask(self,
                           handle: MapHandle,
                           p0: np.ndarray,
                           p1: np.ndarray,
                           mask: np.ndarray) -> Tuple[bool, int, int]:
        seg = self._sample_segment(p0, p1, handle.resolution_m)
        count = self._samples_hit_mask(handle, seg, mask)
        return count > 0, int(count), int(seg.shape[0])

    def _sample_segment(self, p0: np.ndarray, p1: np.ndarray, resolution_m: float) -> np.ndarray:
        a = np.asarray(p0, dtype=np.float64).reshape(2)
        b = np.asarray(p1, dtype=np.float64).reshape(2)
        length = float(np.linalg.norm(b - a))
        if length <= 1.0e-12:
            return a.reshape(1, 2)
        ds = min(self.sample_ds_m, max(float(resolution_m) * 0.5, 1.0e-6))
        n = max(2, int(math.ceil(length / ds)) + 1)
        t = np.linspace(0.0, 1.0, n, dtype=np.float64)
        return (1.0 - t[:, None]) * a + t[:, None] * b

    @staticmethod
    def _samples_hit_mask(handle: MapHandle, xz: np.ndarray, mask: np.ndarray) -> int:
        pts = np.asarray(xz, dtype=np.float64).reshape(-1, 2)
        n_x, n_z = mask.shape
        count = 0
        for x, z in pts:
            ix = int(round((float(x) - handle.x0) / handle.resolution_m))
            iz = int(round((float(z) - handle.z0) / handle.resolution_m))
            if ix < 0 or ix >= n_x or iz < 0 or iz >= n_z:
                continue
            if bool(mask[ix, iz]):
                count += 1
        return int(count)

    @staticmethod
    def _dilate_mask(handle: MapHandle, mask: np.ndarray, radius_m: float) -> np.ndarray:
        base = np.asarray(mask, dtype=bool)
        if radius_m <= 0.0 or not np.any(base):
            return base.copy()
        res = max(float(handle.resolution_m), 1.0e-9)
        r_cells = int(math.ceil(float(radius_m) / res))
        offsets = []
        for dx in range(-r_cells, r_cells + 1):
            for dz in range(-r_cells, r_cells + 1):
                if math.hypot(dx * res, dz * res) <= float(radius_m) + 0.5 * res:
                    offsets.append((dx, dz))
        out = np.zeros_like(base, dtype=bool)
        n_x, n_z = base.shape
        xs, zs = np.nonzero(base)
        for dx, dz in offsets:
            nx = xs + dx
            nz = zs + dz
            valid = (nx >= 0) & (nx < n_x) & (nz >= 0) & (nz < n_z)
            out[nx[valid], nz[valid]] = True
        return out

    def _metrics(self,
                 *,
                 collision_free: bool,
                 violations: int,
                 checked: int,
                 colliding_ticks: int = 0,
                 total_ticks: int = 0) -> Dict[str, float]:
        free = 1.0 if collision_free else 0.0
        return {
            "capsule_proxy_collision_free": free,
            "capsule_proxy_violation_count": float(violations),
            "capsule_proxy_checked_samples": float(checked),
            "capsule_proxy_colliding_ticks": float(colliding_ticks),
            "capsule_proxy_total_ticks": float(total_ticks),
            "capsule_proxy_radius_max_m": float(np.max(self.radii_m)),
            "capsule_proxy_sample_ds_m": float(self.sample_ds_m),
        }
