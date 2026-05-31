#!/usr/bin/env python3
"""Depth-derived task-plane geometry evidence for map update.

The module consumes an already-synchronized CameraPreprocessor FrameData depth
array after bridge conversion.  It does not own camera I/O.
"""
from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Any, Sequence, Tuple

import numpy as np

try:
    from control_module.constants import DEFAULT_OBSTACLE_INFLATION_M, Rect, Y_PLANE_FIXED
    from control_module.map_handle import MapHandle
except ImportError:
    from constants import DEFAULT_OBSTACLE_INFLATION_M, Rect, Y_PLANE_FIXED
    from map_handle import MapHandle

from .perception_to_map import CameraIntrinsics, realsense_d435_sigma_depth_m


@dataclass(frozen=True)
class DepthGeometryConfig:
    pixel_band_half_width_px: float = 100.0
    depth_min_m: float = 0.05
    depth_max_m: float = 0.75
    y_plane: float = Y_PLANE_FIXED
    delta_y_static_m: float = 0.091
    k_sigma: float = 2.5
    min_occupied_component_cells: int = 3
    min_unknown_component_pixels: int = 64
    inflation_m: float = DEFAULT_OBSTACLE_INFLATION_M


@dataclass(frozen=True)
class DepthGeometryEvidence:
    occupied_rects: Tuple[Rect, ...] = field(default_factory=tuple)
    unknown_rects: Tuple[Rect, ...] = field(default_factory=tuple)
    occupied_inflation_m: Tuple[float, ...] = field(default_factory=tuple)
    unknown_inflation_m: Tuple[float, ...] = field(default_factory=tuple)
    stats: dict[str, Any] = field(default_factory=dict)


class DepthGeometryEvidenceBuilder:
    """Project task-band depth into occupied/unknown map-update rectangles."""

    def __init__(self, config: DepthGeometryConfig | None = None) -> None:
        self.config = config or DepthGeometryConfig()
        self._pixel_grid_cache: dict[tuple[int, int], tuple[np.ndarray, np.ndarray]] = {}
        self._band_cache: dict[tuple[int, int, float, float], np.ndarray] = {}

    def build(
        self,
        *,
        depth_m: np.ndarray,
        intrinsics: CameraIntrinsics,
        T_base_cam: np.ndarray | Sequence[Sequence[float]],
        handle: MapHandle,
        unknown_candidate_mask: np.ndarray | None = None,
    ) -> DepthGeometryEvidence:
        cfg = self.config
        t0 = time.perf_counter()
        depth = np.asarray(depth_m, dtype=np.float64)
        if depth.ndim != 2:
            raise ValueError("depth_m must be a 2D array in meters")
        T = np.asarray(T_base_cam, dtype=np.float64)
        if T.shape != (4, 4):
            raise ValueError("T_base_cam must have shape 4x4")

        h, w = depth.shape
        uu, vv = self._pixel_grid(h, w)
        band = self._pixel_band(h, w, intrinsics)
        finite_positive = np.isfinite(depth) & (depth > 0.0)
        valid_in_band = band & finite_positive
        invalid_in_band_raw = band & (~finite_positive)
        if unknown_candidate_mask is None:
            unknown_candidate = np.ones(depth.shape, dtype=bool)
        else:
            unknown_candidate = np.asarray(unknown_candidate_mask, dtype=bool)
            if unknown_candidate.shape != depth.shape:
                raise ValueError("unknown_candidate_mask shape must match depth_m")
        invalid_in_band = invalid_in_band_raw & unknown_candidate
        in_depth = (depth >= float(cfg.depth_min_m)) & (depth <= float(cfg.depth_max_m))
        candidates = valid_in_band & in_depth

        filtered_outside_band_count = int(np.count_nonzero((~band) & finite_positive))
        filtered_outside_depth_count = int(np.count_nonzero(valid_in_band & (~in_depth)))
        ignored_invalid_count = int(np.count_nonzero(invalid_in_band_raw & (~unknown_candidate)))
        stats: dict[str, Any] = {
            "depth_geometry_enabled": True,
            "depth_geometry_pixel_band_half_width_px": float(cfg.pixel_band_half_width_px),
            "depth_geometry_depth_min_m": float(cfg.depth_min_m),
            "depth_geometry_depth_max_m": float(cfg.depth_max_m),
            "depth_geometry_valid_pixel_count": int(np.count_nonzero(valid_in_band)),
            "depth_geometry_invalid_pixel_count": int(np.count_nonzero(invalid_in_band)),
            "depth_geometry_in_yband_pixel_count": 0,
            "depth_geometry_occupied_cluster_count": 0,
            "depth_geometry_unknown_cluster_count": 0,
            "depth_geometry_occupied_rect_count": 0,
            "depth_geometry_unknown_rect_count": 0,
            "depth_geometry_filtered_outside_band_count": filtered_outside_band_count,
            "depth_geometry_filtered_outside_depth_count": filtered_outside_depth_count,
            "depth_geometry_ignored_invalid_pixel_count": ignored_invalid_count,
            "depth_geometry_filtered_small_cluster_count": 0,
            "depth_geometry_added_occupied_cells": 0,
            "depth_geometry_added_unknown_cells": 0,
            "depth_geometry_latency_ms": 0.0,
        }

        occupied_mask = np.zeros(handle.shape, dtype=bool)
        if np.any(candidates):
            u = uu[candidates]
            v = vv[candidates]
            d = depth[candidates]
            points_cam = _backproject(u, v, d, intrinsics)
            points_base = (T[:3, :3] @ points_cam.T).T + T[:3, 3][None, :]
            delta = float(cfg.delta_y_static_m) + float(cfg.k_sigma) * realsense_d435_sigma_depth_m(d)
            in_yband = np.abs(points_base[:, 1] - float(cfg.y_plane)) <= delta
            stats["depth_geometry_in_yband_pixel_count"] = int(np.count_nonzero(in_yband))
            if np.any(in_yband):
                occupied_mask = _points_to_cell_mask(handle, points_base[in_yband])

        occupied_rects, occupied_cells, occupied_clusters, filtered_small = _component_rects_from_cell_mask(
            handle,
            occupied_mask,
            min_cells=max(int(cfg.min_occupied_component_cells), 1),
        )
        stats["depth_geometry_filtered_small_cluster_count"] = int(filtered_small)
        stats["depth_geometry_occupied_cluster_count"] = int(occupied_clusters)
        stats["depth_geometry_occupied_rect_count"] = int(len(occupied_rects))
        stats["depth_geometry_added_occupied_cells"] = int(occupied_cells)

        unknown_rects, unknown_cells, unknown_clusters = _unknown_rects_from_invalid_band(
            handle=handle,
            invalid_mask=invalid_in_band,
            intrinsics=intrinsics,
            T_base_cam=T,
            depth_min_m=float(cfg.depth_min_m),
            depth_max_m=float(cfg.depth_max_m),
            min_pixels=max(int(cfg.min_unknown_component_pixels), 1),
        )
        stats["depth_geometry_unknown_cluster_count"] = int(unknown_clusters)
        stats["depth_geometry_unknown_rect_count"] = int(len(unknown_rects))
        stats["depth_geometry_added_unknown_cells"] = int(unknown_cells)
        stats["depth_geometry_latency_ms"] = float((time.perf_counter() - t0) * 1000.0)

        return DepthGeometryEvidence(
            occupied_rects=tuple(occupied_rects),
            unknown_rects=tuple(unknown_rects),
            occupied_inflation_m=tuple(float(cfg.inflation_m) for _ in occupied_rects),
            unknown_inflation_m=tuple(float(cfg.inflation_m) for _ in unknown_rects),
            stats=stats,
        )

    def skipped_stats(self, *, reason: str) -> dict[str, Any]:
        cfg = self.config
        return {
            "depth_geometry_enabled": False,
            "depth_geometry_skip_reason": str(reason),
            "depth_geometry_pixel_band_half_width_px": float(cfg.pixel_band_half_width_px),
            "depth_geometry_depth_min_m": float(cfg.depth_min_m),
            "depth_geometry_depth_max_m": float(cfg.depth_max_m),
            "depth_geometry_valid_pixel_count": 0,
            "depth_geometry_invalid_pixel_count": 0,
            "depth_geometry_in_yband_pixel_count": 0,
            "depth_geometry_occupied_cluster_count": 0,
            "depth_geometry_unknown_cluster_count": 0,
            "depth_geometry_occupied_rect_count": 0,
            "depth_geometry_unknown_rect_count": 0,
            "depth_geometry_filtered_outside_band_count": 0,
            "depth_geometry_filtered_outside_depth_count": 0,
            "depth_geometry_ignored_invalid_pixel_count": 0,
            "depth_geometry_filtered_small_cluster_count": 0,
            "depth_geometry_added_occupied_cells": 0,
            "depth_geometry_added_unknown_cells": 0,
            "depth_geometry_latency_ms": 0.0,
        }

    def _pixel_grid(self, height: int, width: int) -> tuple[np.ndarray, np.ndarray]:
        key = (int(height), int(width))
        cached = self._pixel_grid_cache.get(key)
        if cached is None:
            cached = np.meshgrid(
                np.arange(int(width), dtype=np.float64),
                np.arange(int(height), dtype=np.float64),
            )
            self._pixel_grid_cache[key] = cached
        return cached

    def _pixel_band(self, height: int, width: int, intrinsics: CameraIntrinsics) -> np.ndarray:
        key = (
            int(height),
            int(width),
            round(float(intrinsics.cx), 6),
            round(float(self.config.pixel_band_half_width_px), 6),
        )
        cached = self._band_cache.get(key)
        if cached is None:
            uu, _ = self._pixel_grid(height, width)
            cached = np.abs(uu - float(intrinsics.cx)) <= float(self.config.pixel_band_half_width_px)
            self._band_cache = {key: cached}
        return cached


def _backproject(
    u: np.ndarray,
    v: np.ndarray,
    depth_m: np.ndarray,
    intrinsics: CameraIntrinsics,
) -> np.ndarray:
    x = (u - float(intrinsics.cx)) * depth_m / float(intrinsics.fx)
    y = (v - float(intrinsics.cy)) * depth_m / float(intrinsics.fy)
    z = depth_m
    return np.stack([x, y, z], axis=1)


def _points_to_cell_mask(handle: MapHandle, points_base: np.ndarray) -> np.ndarray:
    pts = np.asarray(points_base, dtype=np.float64).reshape(-1, 3)
    mask = np.zeros(handle.shape, dtype=bool)
    if pts.size == 0:
        return mask
    ix = np.rint((pts[:, 0] - float(handle.x0)) / float(handle.resolution_m)).astype(np.int64)
    iz = np.rint((pts[:, 2] - float(handle.z0)) / float(handle.resolution_m)).astype(np.int64)
    inside = (ix >= 0) & (ix < handle.shape[0]) & (iz >= 0) & (iz < handle.shape[1])
    if np.any(inside):
        mask[ix[inside], iz[inside]] = True
    return mask


def _component_rects_from_cell_mask(
    handle: MapHandle,
    mask: np.ndarray,
    *,
    min_cells: int,
) -> tuple[list[Rect], int, int, int]:
    arr = np.asarray(mask, dtype=bool)
    visited = np.zeros(arr.shape, dtype=bool)
    rects: list[Rect] = []
    kept_cells = 0
    kept_components = 0
    filtered_small = 0
    for ix0, iz0 in zip(*np.nonzero(arr)):
        if visited[ix0, iz0]:
            continue
        cells = _flood_fill(arr, visited, int(ix0), int(iz0))
        if len(cells) < int(min_cells):
            filtered_small += len(cells)
            continue
        xs = [c[0] for c in cells]
        zs = [c[1] for c in cells]
        rects.append(_cell_rect(handle, min(xs), max(xs), min(zs), max(zs)))
        kept_cells += len(cells)
        kept_components += 1
    return rects, kept_cells, kept_components, filtered_small


def _unknown_rects_from_invalid_band(
    *,
    handle: MapHandle,
    invalid_mask: np.ndarray,
    intrinsics: CameraIntrinsics,
    T_base_cam: np.ndarray,
    depth_min_m: float,
    depth_max_m: float,
    min_pixels: int,
) -> tuple[list[Rect], int, int]:
    arr = np.asarray(invalid_mask, dtype=bool)
    v_idx, u_idx = np.nonzero(arr)
    if v_idx.size < int(min_pixels):
        return [], 0, 0
    rect = _image_component_unknown_rect(
        handle=handle,
        u_min=int(np.min(u_idx)),
        u_max=int(np.max(u_idx)),
        v_min=int(np.min(v_idx)),
        v_max=int(np.max(v_idx)),
        intrinsics=intrinsics,
        T_base_cam=T_base_cam,
        depth_min_m=depth_min_m,
        depth_max_m=depth_max_m,
    )
    if rect is None:
        return [], 0, 0
    return [rect], _rect_cell_count(handle, rect), 1


def _flood_fill(mask: np.ndarray, visited: np.ndarray, i0: int, j0: int) -> list[tuple[int, int]]:
    stack = [(int(i0), int(j0))]
    visited[i0, j0] = True
    cells: list[tuple[int, int]] = []
    n_i, n_j = mask.shape
    while stack:
        i, j = stack.pop()
        cells.append((i, j))
        for di in (-1, 0, 1):
            for dj in (-1, 0, 1):
                if di == 0 and dj == 0:
                    continue
                ni = i + di
                nj = j + dj
                if ni < 0 or ni >= n_i or nj < 0 or nj >= n_j:
                    continue
                if visited[ni, nj] or not mask[ni, nj]:
                    continue
                visited[ni, nj] = True
                stack.append((ni, nj))
    return cells


def _image_component_unknown_rect(
    *,
    handle: MapHandle,
    u_min: int,
    u_max: int,
    v_min: int,
    v_max: int,
    intrinsics: CameraIntrinsics,
    T_base_cam: np.ndarray,
    depth_min_m: float,
    depth_max_m: float,
) -> Rect | None:
    samples = []
    for u in (float(u_min), float(u_max)):
        for v in (float(v_min), float(v_max)):
            for d in (float(depth_min_m), float(depth_max_m)):
                samples.append((u, v, d))
    arr = np.asarray(samples, dtype=np.float64)
    points_cam = _backproject(arr[:, 0], arr[:, 1], arr[:, 2], intrinsics)
    points_base = (T_base_cam[:3, :3] @ points_cam.T).T + T_base_cam[:3, 3][None, :]
    x0 = float(np.min(points_base[:, 0]))
    x1 = float(np.max(points_base[:, 0]))
    z0 = float(np.min(points_base[:, 2]))
    z1 = float(np.max(points_base[:, 2]))
    xl = max(min(x0, x1), float(handle.x0))
    xr = min(max(x0, x1), float(handle.x0) + float(handle.shape[0] - 1) * float(handle.resolution_m))
    zb = max(min(z0, z1), float(handle.z0))
    zt = min(max(z0, z1), float(handle.z0) + float(handle.shape[1] - 1) * float(handle.resolution_m))
    if xr < xl or zt < zb:
        return None
    return xl, zt, xr, zb


def _cell_rect(handle: MapHandle, ix_min: int, ix_max: int, iz_min: int, iz_max: int) -> Rect:
    xl = float(handle.x0) + float(ix_min) * float(handle.resolution_m)
    xr = float(handle.x0) + float(ix_max) * float(handle.resolution_m)
    zb = float(handle.z0) + float(iz_min) * float(handle.resolution_m)
    zt = float(handle.z0) + float(iz_max) * float(handle.resolution_m)
    return xl, zt, xr, zb


def _rect_cell_count(handle: MapHandle, rect: Rect) -> int:
    xl, zt, xr, zb = rect
    ix0 = int(np.floor((float(xl) - float(handle.x0)) / float(handle.resolution_m)))
    ix1 = int(np.ceil((float(xr) - float(handle.x0)) / float(handle.resolution_m)))
    iz0 = int(np.floor((float(zb) - float(handle.z0)) / float(handle.resolution_m)))
    iz1 = int(np.ceil((float(zt) - float(handle.z0)) / float(handle.resolution_m)))
    ix0 = max(0, min(handle.shape[0] - 1, ix0))
    ix1 = max(0, min(handle.shape[0] - 1, ix1))
    iz0 = max(0, min(handle.shape[1] - 1, iz0))
    iz1 = max(0, min(handle.shape[1] - 1, iz1))
    if ix1 < ix0 or iz1 < iz0:
        return 0
    return int((ix1 - ix0 + 1) * (iz1 - iz0 + 1))
