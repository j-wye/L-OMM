#!/usr/bin/env python3
"""Cell-driven active-plane occupancy evidence from aligned depth."""
from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Any, Mapping, Sequence, Tuple

import numpy as np

try:
    from control_module.constants import Rect, Y_PLANE_FIXED
    from control_module.map_handle import MapHandle
except ImportError:
    from constants import Rect, Y_PLANE_FIXED
    from map_handle import MapHandle

from .perception_to_map import CameraIntrinsics, realsense_d435_sigma_depth_m


@dataclass(frozen=True)
class CellProjectionOccupancyConfig:
    depth_min_m: float = 0.05
    depth_max_m: float = 0.75
    y_plane: float = Y_PLANE_FIXED
    transform_slack_m: float = 0.006
    k_sigma: float = 2.5
    cell_size_slack_scale: float = 0.5
    footprint_radius_px: int = 1
    min_valid_coverage: float = 0.5
    robust_depth_percentile: float = 20.0
    min_component_cells: int = 1


@dataclass(frozen=True)
class CellProjectionOccupancyEvidence:
    fov_mask: np.ndarray
    free_mask: np.ndarray
    occupied_mask: np.ndarray
    occluded_mask: np.ndarray
    unknown_mask: np.ndarray
    stats: dict[str, Any] = field(default_factory=dict)


class CellProjectionOccupancyEstimator:
    """Classify reduced active-map cells by inverse depth projection."""

    def __init__(self, config: CellProjectionOccupancyConfig | None = None) -> None:
        self.config = config or CellProjectionOccupancyConfig()

    def build(
        self,
        *,
        depth_m: np.ndarray,
        intrinsics: CameraIntrinsics,
        T_base_cam: np.ndarray | Sequence[Sequence[float]],
        handle: MapHandle,
        base_feasible_mask: np.ndarray | None = None,
        unknown_candidate_mask: np.ndarray | None = None,
        transform_slack_m: float | None = None,
    ) -> CellProjectionOccupancyEvidence:
        cfg = self.config
        t0 = time.perf_counter()
        depth = np.asarray(depth_m, dtype=np.float64)
        if depth.ndim != 2:
            raise ValueError("depth_m must be a 2D array in meters")
        T = np.asarray(T_base_cam, dtype=np.float64)
        if T.shape != (4, 4):
            raise ValueError("T_base_cam must have shape 4x4")
        if unknown_candidate_mask is not None:
            unknown_candidate = np.asarray(unknown_candidate_mask, dtype=bool)
            if unknown_candidate.shape != depth.shape:
                raise ValueError("unknown_candidate_mask shape must match depth_m")
        else:
            unknown_candidate = np.ones(depth.shape, dtype=bool)

        shape = tuple(handle.shape)
        if base_feasible_mask is None:
            active = np.ones(shape, dtype=bool)
        else:
            active = np.asarray(base_feasible_mask, dtype=bool)
            if active.shape != shape:
                raise ValueError("base_feasible_mask shape must match map handle")

        fov = np.zeros(shape, dtype=bool)
        free = np.zeros(shape, dtype=bool)
        occupied = np.zeros(shape, dtype=bool)
        occluded = np.zeros(shape, dtype=bool)
        unknown = np.zeros(shape, dtype=bool)
        considered_footprint_pixels = 0
        finite_footprint_pixels = 0
        unknown_candidate_pixels = 0
        no_update_cells = 0
        unknown_from_invalid_cells = 0
        measured_valid_cells = 0

        h, w = depth.shape
        radius = max(int(cfg.footprint_radius_px), 0)
        min_coverage = min(max(float(cfg.min_valid_coverage), 0.0), 1.0)
        percentile = min(max(float(cfg.robust_depth_percentile), 0.0), 50.0)
        transform_slack = (
            float(cfg.transform_slack_m)
            if transform_slack_m is None
            else max(float(cfg.transform_slack_m), float(transform_slack_m))
        )
        resolution = float(getattr(handle, "resolution_m", 0.0))
        cell_slack = max(resolution, 0.0) * max(float(cfg.cell_size_slack_scale), 0.0)

        ix, iz = np.nonzero(active)
        if ix.size:
            xs = float(handle.x0) + ix.astype(np.float64) * resolution
            zs = float(handle.z0) + iz.astype(np.float64) * resolution
            points_base = np.column_stack(
                [xs, np.full(ix.shape, float(cfg.y_plane), dtype=np.float64), zs]
            )
            R = T[:3, :3]
            t = T[:3, 3]
            points_cam = (R.T @ (points_base - t[None, :]).T).T
            expected = points_cam[:, 2]
            projectable = (
                np.isfinite(expected)
                & (expected > 0.0)
                & (expected >= float(cfg.depth_min_m))
                & (expected <= float(cfg.depth_max_m))
            )
            u_float = np.full(expected.shape, np.nan, dtype=np.float64)
            v_float = np.full(expected.shape, np.nan, dtype=np.float64)
            valid_z = expected > 0.0
            u_float[valid_z] = float(intrinsics.fx) * points_cam[valid_z, 0] / expected[valid_z] + float(intrinsics.cx)
            v_float[valid_z] = float(intrinsics.fy) * points_cam[valid_z, 1] / expected[valid_z] + float(intrinsics.cy)
            u_int = np.rint(u_float).astype(np.int64, copy=False)
            v_int = np.rint(v_float).astype(np.int64, copy=False)
            in_image = (
                projectable
                & (u_int >= 0)
                & (u_int < w)
                & (v_int >= 0)
                & (v_int < h)
            )

            for k in np.nonzero(in_image)[0]:
                i = int(ix[k])
                j = int(iz[k])
                u = int(u_int[k])
                v = int(v_int[k])
                if not bool(unknown_candidate[v, u]) and not (np.isfinite(depth[v, u]) and depth[v, u] > 0.0):
                    no_update_cells += 1
                    continue
                fov[i, j] = True
                u0 = max(0, u - radius)
                u1 = min(w - 1, u + radius)
                v0 = max(0, v - radius)
                v1 = min(h - 1, v + radius)
                patch = depth[v0 : v1 + 1, u0 : u1 + 1]
                candidate_patch = unknown_candidate[v0 : v1 + 1, u0 : u1 + 1]
                finite = np.isfinite(patch) & (patch > 0.0)
                considered = finite | candidate_patch
                considered_count = int(np.count_nonzero(considered))
                finite_count = int(np.count_nonzero(finite))
                candidate_count = int(np.count_nonzero(candidate_patch))
                considered_footprint_pixels += considered_count
                finite_footprint_pixels += finite_count
                unknown_candidate_pixels += candidate_count
                if considered_count <= 0:
                    no_update_cells += 1
                    continue
                coverage = float(finite_count / max(considered_count, 1))
                if coverage < min_coverage or not np.any(finite):
                    if bool(candidate_patch[v - v0, u - u0]):
                        unknown[i, j] = True
                        unknown_from_invalid_cells += 1
                    else:
                        no_update_cells += 1
                    continue
                measured_valid_cells += 1
                measured = float(np.percentile(patch[finite], percentile))
                plane_tol = transform_slack + float(cfg.k_sigma) * float(realsense_d435_sigma_depth_m(measured))
                depth_tol = plane_tol + cell_slack
                p_cam = np.array(
                    [
                        (float(u) - float(intrinsics.cx)) * measured / float(intrinsics.fx),
                        (float(v) - float(intrinsics.cy)) * measured / float(intrinsics.fy),
                        measured,
                    ],
                    dtype=np.float64,
                )
                p_base = T[:3, :3] @ p_cam + T[:3, 3]
                near_plane = abs(float(p_base[1]) - float(cfg.y_plane)) <= plane_tol
                expected_depth = float(expected[k])
                if near_plane:
                    if abs(measured - expected_depth) <= depth_tol:
                        occupied[i, j] = True
                    elif measured < expected_depth - depth_tol:
                        occluded[i, j] = True
                    else:
                        free[i, j] = True
                else:
                    if measured < expected_depth - depth_tol:
                        occluded[i, j] = True
                    else:
                        free[i, j] = True

        tolerance = (
            transform_slack
            + float(cfg.k_sigma) * realsense_d435_sigma_depth_m(np.asarray([float(cfg.depth_min_m), float(cfg.depth_max_m)]))
            + cell_slack
        )
        stats = {
            "cell_projection_enabled": True,
            "cell_projection_depth_min_m": float(cfg.depth_min_m),
            "cell_projection_depth_max_m": float(cfg.depth_max_m),
            "cell_projection_y_plane": float(cfg.y_plane),
            "cell_projection_transform_slack_m": float(transform_slack),
            "cell_projection_cell_size_slack_m": float(cell_slack),
            "cell_projection_footprint_radius_px": float(radius),
            "cell_projection_min_valid_coverage": float(min_coverage),
            "cell_projection_robust_depth_percentile": float(percentile),
            "cell_projection_min_component_cells": int(max(int(cfg.min_component_cells), 1)),
            "cell_projection_fov_cells": int(np.count_nonzero(fov)),
            "cell_projection_free_cells": int(np.count_nonzero(free)),
            "cell_projection_occupied_cells": int(np.count_nonzero(occupied)),
            "cell_projection_occluded_cells": int(np.count_nonzero(occluded)),
            "cell_projection_unknown_cells": int(np.count_nonzero(unknown)),
            "cell_projection_considered_footprint_pixels": int(considered_footprint_pixels),
            "cell_projection_finite_footprint_pixels": int(finite_footprint_pixels),
            "cell_projection_unknown_candidate_pixels": int(unknown_candidate_pixels),
            "cell_projection_no_update_cells": int(no_update_cells),
            "cell_projection_unknown_from_invalid_cells": int(unknown_from_invalid_cells),
            "cell_projection_measured_valid_cells": int(measured_valid_cells),
            "cell_projection_depth_tolerance_m_min": float(np.min(tolerance)),
            "cell_projection_depth_tolerance_m_max": float(np.max(tolerance)),
            "cell_projection_latency_ms": float((time.perf_counter() - t0) * 1000.0),
        }
        return CellProjectionOccupancyEvidence(
            fov_mask=fov,
            free_mask=free,
            occupied_mask=occupied,
            occluded_mask=occluded,
            unknown_mask=unknown,
            stats=stats,
        )

    def skipped_stats(self, *, reason: str) -> dict[str, Any]:
        cfg = self.config
        return {
            "cell_projection_enabled": False,
            "cell_projection_skip_reason": str(reason),
            "cell_projection_depth_min_m": float(cfg.depth_min_m),
            "cell_projection_depth_max_m": float(cfg.depth_max_m),
            "cell_projection_fov_cells": 0,
            "cell_projection_free_cells": 0,
            "cell_projection_occupied_cells": 0,
            "cell_projection_occluded_cells": 0,
            "cell_projection_unknown_cells": 0,
            "cell_projection_considered_footprint_pixels": 0,
            "cell_projection_finite_footprint_pixels": 0,
            "cell_projection_unknown_candidate_pixels": 0,
            "cell_projection_no_update_cells": 0,
            "cell_projection_unknown_from_invalid_cells": 0,
            "cell_projection_measured_valid_cells": 0,
            "cell_projection_latency_ms": 0.0,
        }


def component_rects_from_mask(
    handle: MapHandle,
    mask: np.ndarray,
    *,
    min_cells: int = 1,
) -> tuple[Tuple[Rect, ...], dict[str, Any]]:
    arr = np.asarray(mask, dtype=bool)
    if arr.shape != tuple(handle.shape):
        raise ValueError("mask shape must match map handle")
    visited = np.zeros(arr.shape, dtype=bool)
    rects: list[Rect] = []
    source_cells = int(np.count_nonzero(arr))
    rect_cells = 0
    component_count = 0
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
        ix_min, ix_max = min(xs), max(xs)
        iz_min, iz_max = min(zs), max(zs)
        rects.append(_cell_rect(handle, ix_min, ix_max, iz_min, iz_max))
        rect_cells += int((ix_max - ix_min + 1) * (iz_max - iz_min + 1))
        component_count += 1
    return tuple(rects), {
        "source_cells": int(source_cells),
        "rect_cells": int(rect_cells),
        "overcovered_cells": int(max(rect_cells - source_cells, 0)),
        "component_count": int(component_count),
        "filtered_small_cells": int(filtered_small),
    }


def diagnostic_masks_from_evidence(evidence: CellProjectionOccupancyEvidence) -> dict[str, np.ndarray]:
    return {
        "cell_projection_fov": np.asarray(evidence.fov_mask, dtype=bool),
        "cell_projection_free": np.asarray(evidence.free_mask, dtype=bool),
        "cell_projection_occupied": np.asarray(evidence.occupied_mask, dtype=bool),
        "cell_projection_occluded": np.asarray(evidence.occluded_mask, dtype=bool),
        "cell_projection_unknown": np.asarray(evidence.unknown_mask, dtype=bool),
    }


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


def _cell_rect(handle: MapHandle, ix_min: int, ix_max: int, iz_min: int, iz_max: int) -> Rect:
    xl = float(handle.x0) + float(ix_min) * float(handle.resolution_m)
    xr = float(handle.x0) + float(ix_max) * float(handle.resolution_m)
    zb = float(handle.z0) + float(iz_min) * float(handle.resolution_m)
    zt = float(handle.z0) + float(iz_max) * float(handle.resolution_m)
    return xl, zt, xr, zb
