#!/usr/bin/env python3
"""Reference-snapshot spatial diff in the shared x-z map frame."""
from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field
from typing import Dict, Optional, Sequence, Tuple

import numpy as np

from .active_map_snapshot import ActiveMapSnapshot
from .fov_mask import compute_task_plane_fov_mask
from .perception_to_map import CameraIntrinsics


@dataclass(frozen=True)
class DiffReport:
    """Spatial diff report between accepted reference and current observation."""

    new_blocked_mask: np.ndarray
    vanished_blocked_mask: np.ndarray
    new_obstacle_mask: np.ndarray
    vanished_obstacle_mask: np.ndarray
    fov_mask: np.ndarray
    new_blocked_cells_count: int
    vanished_blocked_cells_count: int
    new_obstacle_cells: int
    vanished_obstacle_cells: int
    fov_observed_cells: int
    fov_coverage_ratio: float
    largest_new_cluster_size: int
    largest_reference_corridor_cluster_size: int
    new_cells_in_reference_corridor: int
    new_cells_in_current_capsule: int
    new_cells_in_goal_window: int
    is_corridor_violated: bool
    tau_diff_cells: int
    tau_cluster_cells: int
    reference_valid: bool = True

    def as_dict(self) -> Dict[str, object]:
        return {
            "reference_diff_enabled": True,
            "reference_diff_reference_valid": bool(self.reference_valid),
            "new_blocked_cells_count": int(self.new_blocked_cells_count),
            "vanished_blocked_cells_count": int(self.vanished_blocked_cells_count),
            "new_blocked_cells": int(self.new_blocked_cells_count),
            "vanished_blocked_cells": int(self.vanished_blocked_cells_count),
            "reference_diff_2class_fields": True,
            "reference_diff_per_class_fields_deprecated": True,
            "new_obstacle_cells": int(self.new_obstacle_cells),
            "vanished_obstacle_cells": int(self.vanished_obstacle_cells),
            "fov_observed_cells": int(self.fov_observed_cells),
            "fov_coverage_ratio": float(self.fov_coverage_ratio),
            "largest_new_cluster_size": int(self.largest_new_cluster_size),
            "largest_reference_corridor_cluster_size": int(self.largest_reference_corridor_cluster_size),
            "new_cells_in_reference_corridor": int(self.new_cells_in_reference_corridor),
            "new_cells_in_current_capsule": int(self.new_cells_in_current_capsule),
            "new_cells_in_goal_window": int(self.new_cells_in_goal_window),
            "is_corridor_violated": bool(self.is_corridor_violated),
            "tau_diff_cells": int(self.tau_diff_cells),
            "tau_cluster_cells": int(self.tau_cluster_cells),
        }


@dataclass
class _ReferenceState:
    snapshot: ActiveMapSnapshot
    blocked_mask: np.ndarray
    final_active_mask: np.ndarray
    reference_corridor_mask: np.ndarray
    q_act: object = None
    T_base_cam: Optional[np.ndarray] = None
    metadata: Dict[str, object] = field(default_factory=dict)
    valid: bool = True


class ReferenceSnapshotDiff:
    """Compare current x-z obstacle evidence against an accepted reference."""

    def __init__(
        self,
        *,
        handle,
        intrinsics: CameraIntrinsics,
        y_plane: float,
        d_min: float = 0.05,
        d_max_task: float = 1.0,
        pixel_band_half_width: float | None = None,
        corridor_radius_m: float = 0.015,
        capsule_radius_m: float = 0.015,
        goal_radius_m: float = 0.02,
        tau_diff_cells: int = 3,
        tau_cluster_cells: int = 5,
        require_current_transform: bool = False,
    ) -> None:
        self.handle = handle
        self.intrinsics = intrinsics
        self.y_plane = float(y_plane)
        self.d_min = float(d_min)
        self.d_max_task = float(d_max_task)
        self.pixel_band_half_width = None if pixel_band_half_width is None else float(pixel_band_half_width)
        self.corridor_radius_m = float(corridor_radius_m)
        self.capsule_radius_m = float(capsule_radius_m)
        self.goal_radius_m = float(goal_radius_m)
        self.tau_diff_cells = int(tau_diff_cells)
        self.tau_cluster_cells = int(tau_cluster_cells)
        self.require_current_transform = bool(require_current_transform)
        self._reference: Optional[_ReferenceState] = None

    @property
    def has_reference(self) -> bool:
        return self._reference is not None and bool(self._reference.valid)

    def capture(
        self,
        snapshot: ActiveMapSnapshot,
        q_act: object,
        T_base_cam: np.ndarray | Sequence[Sequence[float]] | None,
        reference_corridor: np.ndarray | Sequence[Sequence[float]] | None = None,
        metadata: Optional[Dict[str, object]] = None,
    ) -> None:
        """Freeze the snapshot that justified the currently accepted path."""
        self._assert_shape(snapshot)
        corridor_mask = self._region_to_mask(reference_corridor, self.corridor_radius_m)
        T = None if T_base_cam is None else np.asarray(T_base_cam, dtype=np.float64)
        if T is not None and T.shape != (4, 4):
            raise ValueError("T_base_cam must have shape 4x4")
        self._reference = _ReferenceState(
            snapshot=snapshot,
            blocked_mask=np.asarray(snapshot.blocked_mask, dtype=bool).copy(),
            final_active_mask=np.asarray(snapshot.final_active_mask, dtype=bool).copy(),
            reference_corridor_mask=corridor_mask,
            q_act=q_act,
            T_base_cam=None if T is None else T.copy(),
            metadata=dict(metadata or {}),
            valid=True,
        )

    def invalidate(self) -> None:
        if self._reference is not None:
            self._reference.valid = False

    def evaluate(
        self,
        current_snapshot: ActiveMapSnapshot,
        current_T_base_cam: np.ndarray | Sequence[Sequence[float]] | None,
        current_corridor: np.ndarray | Sequence[Sequence[float]] | None = None,
        *,
        current_capsule_xz: np.ndarray | Sequence[Sequence[float]] | None = None,
        goal_window_xz: np.ndarray | Sequence[Sequence[float]] | None = None,
        fov_mask_override: Optional[np.ndarray] = None,
    ) -> DiffReport:
        """Return the FOV-aware spatial diff for the current snapshot."""
        self._assert_shape(current_snapshot)
        if self._reference is None:
            raise RuntimeError("ReferenceSnapshotDiff.capture must be called before evaluate")
        ref = self._reference
        current_blocked = np.asarray(current_snapshot.blocked_mask, dtype=bool)
        if fov_mask_override is not None:
            fov_mask = np.asarray(fov_mask_override, dtype=bool)
            if fov_mask.shape != tuple(self.handle.shape):
                raise ValueError("fov_mask_override shape does not match map shape")
        elif current_T_base_cam is None:
            if self.require_current_transform:
                raise ValueError("current_T_base_cam is required when require_current_transform=True")
            fov_mask = np.ones(tuple(self.handle.shape), dtype=bool)
        else:
            fov_mask = compute_task_plane_fov_mask(
                self.handle,
                self.intrinsics,
                current_T_base_cam,
                self.y_plane,
                self.d_min,
                self.d_max_task,
                pixel_band_half_width=self.pixel_band_half_width,
            )

        new_mask = np.logical_and(np.logical_and(~ref.blocked_mask, current_blocked), fov_mask)
        vanished_mask = np.logical_and(np.logical_and(ref.blocked_mask, ~current_blocked), fov_mask)
        corridor_mask = (
            self._region_to_mask(current_corridor, self.corridor_radius_m)
            if current_corridor is not None
            else ref.reference_corridor_mask
        )
        capsule_mask = self._region_to_mask(current_capsule_xz, self.capsule_radius_m)
        goal_mask = self._region_to_mask(goal_window_xz, self.goal_radius_m)

        new_in_corridor = np.logical_and(new_mask, corridor_mask)
        largest_new = _largest_component_size(new_mask)
        largest_corridor = _largest_component_size(new_in_corridor)
        new_corridor_cells = int(np.count_nonzero(new_in_corridor))
        violated = (
            bool(ref.valid)
            and new_corridor_cells >= max(self.tau_diff_cells, 0)
            and largest_corridor >= max(self.tau_cluster_cells, 1)
        )
        fov_cells = int(np.count_nonzero(fov_mask))
        return DiffReport(
            new_blocked_mask=new_mask,
            vanished_blocked_mask=vanished_mask,
            new_obstacle_mask=new_mask,
            vanished_obstacle_mask=vanished_mask,
            fov_mask=fov_mask,
            new_blocked_cells_count=int(np.count_nonzero(new_mask)),
            vanished_blocked_cells_count=int(np.count_nonzero(vanished_mask)),
            new_obstacle_cells=int(np.count_nonzero(new_mask)),
            vanished_obstacle_cells=int(np.count_nonzero(vanished_mask)),
            fov_observed_cells=fov_cells,
            fov_coverage_ratio=float(fov_cells / max(new_mask.size, 1)),
            largest_new_cluster_size=int(largest_new),
            largest_reference_corridor_cluster_size=int(largest_corridor),
            new_cells_in_reference_corridor=new_corridor_cells,
            new_cells_in_current_capsule=int(np.count_nonzero(np.logical_and(new_mask, capsule_mask))),
            new_cells_in_goal_window=int(np.count_nonzero(np.logical_and(new_mask, goal_mask))),
            is_corridor_violated=violated,
            tau_diff_cells=int(self.tau_diff_cells),
            tau_cluster_cells=int(self.tau_cluster_cells),
            reference_valid=bool(ref.valid),
        )

    def _assert_shape(self, snapshot: ActiveMapSnapshot) -> None:
        if tuple(snapshot.handle.shape) != tuple(self.handle.shape):
            raise ValueError("snapshot handle shape does not match ReferenceSnapshotDiff handle")

    def _region_to_mask(
        self,
        region: np.ndarray | Sequence[Sequence[float]] | None,
        radius_m: float,
    ) -> np.ndarray:
        mask = np.zeros(tuple(self.handle.shape), dtype=bool)
        if region is None:
            return mask
        arr = np.asarray(region)
        if arr.dtype == bool and arr.shape == tuple(self.handle.shape):
            return arr.astype(bool, copy=True)
        pts = np.asarray(region, dtype=np.float64).reshape(-1, 2)
        if pts.size == 0:
            return mask
        radius_cells = int(np.ceil(max(float(radius_m), 0.0) / max(float(self.handle.resolution_m), 1.0e-12)))
        n_x, n_z = mask.shape
        for x, z in pts:
            ix = int(round((float(x) - float(self.handle.x0)) / float(self.handle.resolution_m)))
            iz = int(round((float(z) - float(self.handle.z0)) / float(self.handle.resolution_m)))
            for dx in range(-radius_cells, radius_cells + 1):
                for dz in range(-radius_cells, radius_cells + 1):
                    nx, nz = ix + dx, iz + dz
                    if nx < 0 or nx >= n_x or nz < 0 or nz >= n_z:
                        continue
                    ddx = dx * float(self.handle.resolution_m)
                    ddz = dz * float(self.handle.resolution_m)
                    if ddx * ddx + ddz * ddz <= float(radius_m) * float(radius_m) + 1.0e-12:
                        mask[nx, nz] = True
        return mask


def _largest_component_size(mask: np.ndarray) -> int:
    arr = np.asarray(mask, dtype=bool)
    if arr.size == 0 or not np.any(arr):
        return 0
    n_x, n_z = arr.shape
    visited = np.zeros(arr.shape, dtype=bool)
    neighbors = (
        (-1, -1), (-1, 0), (-1, 1),
        (0, -1),           (0, 1),
        (1, -1),  (1, 0),  (1, 1),
    )
    largest = 0
    for start in zip(*np.nonzero(arr)):
        sx, sz = int(start[0]), int(start[1])
        if visited[sx, sz]:
            continue
        q = deque([(sx, sz)])
        visited[sx, sz] = True
        size = 0
        while q:
            ix, iz = q.popleft()
            size += 1
            for dx, dz in neighbors:
                nx, nz = ix + dx, iz + dz
                if nx < 0 or nx >= n_x or nz < 0 or nz >= n_z:
                    continue
                if visited[nx, nz] or not arr[nx, nz]:
                    continue
                visited[nx, nz] = True
                q.append((nx, nz))
        largest = max(largest, size)
    return int(largest)
