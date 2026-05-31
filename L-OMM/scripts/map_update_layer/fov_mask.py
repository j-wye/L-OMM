#!/usr/bin/env python3
"""Task-plane field-of-view masks for reference snapshot diff."""
from __future__ import annotations

from typing import Mapping, Sequence

import numpy as np

from .perception_to_map import CameraIntrinsics


def compute_task_plane_fov_mask(
    handle,
    intrinsics: CameraIntrinsics | Mapping[str, object] | np.ndarray,
    T_base_cam: np.ndarray | Sequence[Sequence[float]],
    y_plane: float,
    d_min: float,
    d_max_task: float,
    *,
    pixel_band_half_width: float | None = None,
) -> np.ndarray:
    """Rasterize the current camera-visible task-plane support in x-z cells.

    The experiment uses the image-center vertical line as the nominal x-z task
    plane.  We therefore project the bounded center-band frustum support into
    the shared base/map x-z coordinates.  The output is only an observability
    mask: cells outside it must not be interpreted as changed or cleared.
    """
    intr = _coerce_intrinsics(intrinsics)
    T = np.asarray(T_base_cam, dtype=np.float64)
    if T.shape != (4, 4):
        raise ValueError("T_base_cam must have shape 4x4")
    d0 = float(d_min)
    d1 = float(d_max_task)
    if not np.isfinite(d0) or not np.isfinite(d1) or d0 < 0.0 or d1 <= d0:
        raise ValueError("d_min and d_max_task must satisfy 0 <= d_min < d_max_task")

    width = _image_width(intr)
    height = _image_height(intr)
    half_band = 0.0 if pixel_band_half_width is None else max(float(pixel_band_half_width), 0.0)
    u_values = [float(intr.cx)]
    if half_band > 0.0:
        u_values = [
            max(0.0, float(intr.cx) - half_band),
            min(float(width - 1), float(intr.cx) + half_band),
        ]
    v_values = [0.0, float(height - 1)]
    d_values = [d0, d1]

    pts_xz = []
    for u in u_values:
        for v in v_values:
            for d in d_values:
                p_cam = _backproject_pixel(float(u), float(v), float(d), intr)
                p_base = T[:3, :3] @ p_cam + T[:3, 3]
                pts_xz.append((float(p_base[0]), float(p_base[2])))
    polygon = _convex_hull(np.asarray(pts_xz, dtype=np.float64))
    return rasterize_xz_polygon(handle, polygon)


def rasterize_xz_polygon(handle, polygon_xz: np.ndarray) -> np.ndarray:
    """Rasterize an x-z polygon into the grid described by ``handle``."""
    polygon = np.asarray(polygon_xz, dtype=np.float64).reshape(-1, 2)
    mask = np.zeros(tuple(handle.shape), dtype=bool)
    if polygon.shape[0] < 3:
        return mask

    n_x, n_z = mask.shape
    res = float(handle.resolution_m)
    x_min = max(0, int(np.floor((float(np.min(polygon[:, 0])) - float(handle.x0)) / res)) - 1)
    x_max = min(n_x - 1, int(np.ceil((float(np.max(polygon[:, 0])) - float(handle.x0)) / res)) + 1)
    z_min = max(0, int(np.floor((float(np.min(polygon[:, 1])) - float(handle.z0)) / res)) - 1)
    z_max = min(n_z - 1, int(np.ceil((float(np.max(polygon[:, 1])) - float(handle.z0)) / res)) + 1)
    if x_max < x_min or z_max < z_min:
        return mask

    for ix in range(x_min, x_max + 1):
        x = float(handle.x0) + ix * res
        for iz in range(z_min, z_max + 1):
            z = float(handle.z0) + iz * res
            if _point_in_polygon(x, z, polygon):
                mask[ix, iz] = True
    return mask


def _coerce_intrinsics(value: CameraIntrinsics | Mapping[str, object] | np.ndarray) -> CameraIntrinsics:
    if isinstance(value, CameraIntrinsics):
        return value
    if isinstance(value, Mapping):
        data = dict(value)
        return CameraIntrinsics(
            fx=float(data["fx"]),
            fy=float(data["fy"]),
            cx=float(data["cx"]),
            cy=float(data["cy"]),
            width=int(data.get("width", 0)),
            height=int(data.get("height", 0)),
        )
    arr = np.asarray(value, dtype=np.float64)
    if arr.shape == (3, 3):
        return CameraIntrinsics(
            fx=float(arr[0, 0]),
            fy=float(arr[1, 1]),
            cx=float(arr[0, 2]),
            cy=float(arr[1, 2]),
        )
    raise ValueError("intrinsics must be CameraIntrinsics, mapping, or 3x3 K")


def _image_width(intrinsics: CameraIntrinsics) -> int:
    if int(intrinsics.width) > 0:
        return int(intrinsics.width)
    return max(1, int(round(2.0 * float(intrinsics.cx) + 1.0)))


def _image_height(intrinsics: CameraIntrinsics) -> int:
    if int(intrinsics.height) > 0:
        return int(intrinsics.height)
    return max(1, int(round(2.0 * float(intrinsics.cy) + 1.0)))


def _backproject_pixel(u: float, v: float, depth_m: float, intrinsics: CameraIntrinsics) -> np.ndarray:
    d = float(depth_m)
    return np.asarray(
        [
            (float(u) - float(intrinsics.cx)) * d / float(intrinsics.fx),
            (float(v) - float(intrinsics.cy)) * d / float(intrinsics.fy),
            d,
        ],
        dtype=np.float64,
    )


def _convex_hull(points: np.ndarray) -> np.ndarray:
    pts = sorted({(float(x), float(z)) for x, z in np.asarray(points, dtype=np.float64).reshape(-1, 2)})
    if len(pts) <= 1:
        return np.asarray(pts, dtype=np.float64).reshape(-1, 2)

    def cross(o, a, b) -> float:
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    lower = []
    for p in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0.0:
            lower.pop()
        lower.append(p)
    upper = []
    for p in reversed(pts):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0.0:
            upper.pop()
        upper.append(p)
    hull = lower[:-1] + upper[:-1]
    return np.asarray(hull, dtype=np.float64).reshape(-1, 2)


def _point_in_polygon(x: float, z: float, polygon: np.ndarray) -> bool:
    inside = False
    n = int(polygon.shape[0])
    j = n - 1
    for i in range(n):
        xi, zi = float(polygon[i, 0]), float(polygon[i, 1])
        xj, zj = float(polygon[j, 0]), float(polygon[j, 1])
        crosses = (zi > z) != (zj > z)
        if crosses:
            x_cross = (xj - xi) * (z - zi) / max(zj - zi, 1.0e-12) + xi
            if x < x_cross:
                inside = not inside
        j = i
    return inside
