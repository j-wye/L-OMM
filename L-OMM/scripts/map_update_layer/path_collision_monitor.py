#!/usr/bin/env python3
"""Capsule-collision trigger wrapper for accepted references."""
from __future__ import annotations

import json
import os
import time
from dataclasses import dataclass
from typing import Any, Mapping, Sequence

import numpy as np


@dataclass(frozen=True)
class CollisionReport:
    """Result of checking an accepted path/reference against a blocked mask."""

    is_collision: bool
    first_collision_point: tuple[float, float] | None
    colliding_cells_count: int
    checked_sample_count: int = 0
    source: str = "path_collision_monitor"

    def as_dict(self) -> dict[str, object]:
        return {
            "path_collision_monitor_enabled": True,
            "path_collision_is_collision": bool(self.is_collision),
            "path_collision_first_collision_point": self.first_collision_point,
            "path_collision_colliding_cells_count": int(self.colliding_cells_count),
            "path_collision_checked_sample_count": int(self.checked_sample_count),
            "path_collision_source": self.source,
        }


class PathCollisionMonitor:
    """Thin adapter from an accepted reference to production capsule checks.

    The monitor does not define a new collision primitive.  It delegates to the
    existing ``CapsuleCollision.q_hits_mask`` / ``trajectory_metrics`` API when
    q-samples are available, or reconstructs q-samples from ``handle.q_grid`` for
    x-z reference samples.
    """

    def __init__(self, capsule: Any, handle: Any | None = None) -> None:
        self.capsule = capsule
        self.handle = handle

    def check(
        self,
        existing_path: Any,
        blocked_mask: np.ndarray,
        *,
        sample_density: float = 0.01,
        handle: Any | None = None,
    ) -> CollisionReport:
        mask = np.asarray(blocked_mask, dtype=bool)
        active_handle = handle if handle is not None else self.handle
        q_traj = self._extract_q_traj(existing_path)
        xz_path = self._extract_xz_path(existing_path)

        if q_traj is None and xz_path is not None:
            if active_handle is None or getattr(active_handle, "q_grid", None) is None:
                raise NotImplementedError("xz path collision check requires handle.q_grid")
            q_traj, xz_path = self._q_samples_from_xz(active_handle, xz_path, sample_density)

        if q_traj is None:
            raise NotImplementedError("PathCollisionMonitor requires q_traj/q_samples or xz_path with handle.q_grid")

        q_arr = np.asarray(q_traj, dtype=np.float64).reshape(-1, 3)
        if q_arr.shape[0] == 0:
            report = CollisionReport(False, None, 0, 0)
            _jetson_sensitivity_log("PathCollisionMonitor.check", existing_path, mask, report, 0, xz_path)
            return report
        if active_handle is None:
            raise NotImplementedError("PathCollisionMonitor requires a map handle for capsule collision checks")

        collisions = 0
        checked = 0
        first_point: tuple[float, float] | None = None
        dilated = None
        if hasattr(self.capsule, "build_dilated_masks"):
            dilated = self.capsule.build_dilated_masks(active_handle, mask)
        for idx, q in enumerate(q_arr):
            hit, hit_count, sample_count = self.capsule.q_hits_mask(q, active_handle, mask, dilated)
            checked += int(sample_count)
            if hit:
                collisions += int(max(hit_count, 1))
                if first_point is None and xz_path is not None and idx < xz_path.shape[0]:
                    first_point = (float(xz_path[idx, 0]), float(xz_path[idx, 1]))
        report = CollisionReport(
            is_collision=collisions > 0,
            first_collision_point=first_point,
            colliding_cells_count=int(collisions),
            checked_sample_count=int(checked),
        )
        _jetson_sensitivity_log("PathCollisionMonitor.check", existing_path, mask, report, int(q_arr.shape[0]), xz_path)
        return report

    @staticmethod
    def _extract_q_traj(existing_path: Any) -> np.ndarray | None:
        for name in ("q_traj", "q_samples", "q_active_samples"):
            value = _field(existing_path, name)
            if value is not None:
                return np.asarray(value, dtype=np.float64).reshape(-1, 3)
        arr = np.asarray(existing_path) if _array_like(existing_path) else None
        if arr is not None and arr.ndim == 2 and arr.shape[1] == 3:
            return arr.astype(np.float64, copy=False)
        return None

    @staticmethod
    def _extract_xz_path(existing_path: Any) -> np.ndarray | None:
        for name in ("xz_path", "xz", "points_xz", "reference_xz"):
            value = _field(existing_path, name)
            if value is not None:
                return np.asarray(value, dtype=np.float64).reshape(-1, 2)
        arr = np.asarray(existing_path) if _array_like(existing_path) else None
        if arr is not None and arr.ndim == 2 and arr.shape[1] == 2:
            return arr.astype(np.float64, copy=False)
        return None

    @staticmethod
    def _q_samples_from_xz(
        handle: Any,
        xz_path: np.ndarray,
        sample_density: float,
    ) -> tuple[np.ndarray, np.ndarray]:
        pts = _densify_xz(np.asarray(xz_path, dtype=np.float64).reshape(-1, 2), sample_density)
        q_grid = np.asarray(handle.q_grid, dtype=np.float64)
        q_samples = []
        kept_xz = []
        n_x, n_z = q_grid.shape[:2]
        for x, z in pts:
            ix = int(round((float(x) - float(handle.x0)) / float(handle.resolution_m)))
            iz = int(round((float(z) - float(handle.z0)) / float(handle.resolution_m)))
            if ix < 0 or ix >= n_x or iz < 0 or iz >= n_z:
                continue
            q = np.asarray(q_grid[ix, iz], dtype=np.float64).reshape(-1)
            if q.size != 3 or not np.all(np.isfinite(q)):
                continue
            q_samples.append(q)
            kept_xz.append((float(x), float(z)))
        return (
            np.asarray(q_samples, dtype=np.float64).reshape(-1, 3),
            np.asarray(kept_xz, dtype=np.float64).reshape(-1, 2),
        )


def _field(obj: Any, name: str) -> Any | None:
    if isinstance(obj, Mapping):
        return obj.get(name)
    return getattr(obj, name, None)


def _array_like(obj: Any) -> bool:
    return isinstance(obj, (np.ndarray, Sequence)) and not isinstance(obj, (str, bytes, Mapping))


def _densify_xz(xz: np.ndarray, sample_density: float) -> np.ndarray:
    pts = np.asarray(xz, dtype=np.float64).reshape(-1, 2)
    if pts.shape[0] <= 1:
        return pts
    ds = max(float(sample_density), 1.0e-9)
    out = [pts[0]]
    for a, b in zip(pts[:-1], pts[1:]):
        length = float(np.linalg.norm(b - a))
        steps = max(1, int(np.ceil(length / ds)))
        for k in range(1, steps + 1):
            t = k / steps
            out.append((1.0 - t) * a + t * b)
    return np.asarray(out, dtype=np.float64).reshape(-1, 2)


# === DIAGNOSTIC LOGGING (REMOVABLE) - JETSON_SENSITIVITY_SWEEP ===
def _jetson_sensitivity_log(
    event: str,
    existing_path: Any,
    blocked_mask: np.ndarray,
    report: CollisionReport,
    q_sample_count: int,
    xz_path: np.ndarray | None,
) -> None:
    path = os.environ.get("JETSON_SENSITIVITY_SWEEP_LOG")
    if not path:
        return
    payload = {
        "timestamp_s": time.time(),
        "event": str(event),
        "existing_path_type": type(existing_path).__name__,
        "blocked_mask_shape": list(np.asarray(blocked_mask).shape),
        "blocked_mask_count": int(np.count_nonzero(np.asarray(blocked_mask, dtype=bool))),
        "q_sample_count": int(q_sample_count),
        "xz_sample_count": int(0 if xz_path is None else np.asarray(xz_path).reshape(-1, 2).shape[0]),
        "report": report.as_dict(),
    }
    with open(path, "a", encoding="utf-8") as f:
        f.write(json.dumps(payload, ensure_ascii=False) + "\n")
# === END DIAGNOSTIC LOGGING ===
