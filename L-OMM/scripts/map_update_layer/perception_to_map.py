#!/usr/bin/env python3
"""RGB-D perception-to-active-map adapter.

This module converts already-produced perception/decision evidence into the
semantic rectangle contract consumed by MapUpdateLayer.  It does not subscribe
to ROS2 topics, open a camera, or run segmentation models.
"""
from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Iterable, Mapping, Sequence, Tuple

import numpy as np

try:
    from control_module.constants import DEFAULT_OBSTACLE_INFLATION_M, Rect, Y_PLANE_FIXED
except ImportError:
    from constants import DEFAULT_OBSTACLE_INFLATION_M, Rect, Y_PLANE_FIXED

from .map_update_request import MapUpdateRequest


@dataclass(frozen=True)
class CameraIntrinsics:
    fx: float
    fy: float
    cx: float
    cy: float
    width: int = 0
    height: int = 0


@dataclass(frozen=True)
class Detection2D:
    semantic_type: str
    bbox: Tuple[float, float, float, float]
    mask: np.ndarray | None = None
    score: float = 1.0
    source_id: str = ""
    fallback_depth_m: float | None = None


def realsense_d435_sigma_depth_m(depth_m: np.ndarray | float) -> np.ndarray:
    d = np.asarray(depth_m, dtype=np.float64)
    return 0.001 + 0.002 * np.square(np.maximum(d, 0.0))


def nominal_t_base_cam(y_plane: float = Y_PLANE_FIXED) -> np.ndarray:
    """Return the nominal gripper-camera transform used by the design note."""
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = np.array(
        [
            [0.0, 0.0, 1.0],
            [-1.0, 0.0, 0.0],
            [0.0, -1.0, 0.0],
        ],
        dtype=np.float64,
    )
    T[:3, 3] = np.array([0.0, float(y_plane), 0.0], dtype=np.float64)
    return T


def _image_yband_prefilter(
    bbox: Sequence[float],
    depth_roi: np.ndarray,
    frame_shape: Tuple[int, int],
    intrinsics: CameraIntrinsics,
    delta_y_m: float,
) -> bool:
    """Conservative image-space y-band gate.

    Invalid depth keeps the detection because the safety-critical discard is
    the later 3D point-level filter, not this fast prefilter.
    """
    u0, _, u1, _ = _normalize_bbox(bbox, frame_shape)
    if u1 < u0:
        return False
    has_depth, d_min, _ = _depth_roi_summary(depth_roi)
    if not has_depth:
        return True
    return _image_yband_prefilter_from_depth_min(bbox, frame_shape, intrinsics, delta_y_m, d_min)


def _image_yband_prefilter_from_depth_min(
    bbox: Sequence[float],
    frame_shape: Tuple[int, int],
    intrinsics: CameraIntrinsics,
    delta_y_m: float,
    depth_min_m: float,
) -> bool:
    u0, _, u1, _ = _normalize_bbox(bbox, frame_shape)
    if u1 < u0:
        return False
    d_min = max(float(depth_min_m), 1.0e-6)
    half_width_px = float(intrinsics.fx) * max(float(delta_y_m), 0.0) / d_min
    return not (float(u1) < intrinsics.cx - half_width_px or float(u0) > intrinsics.cx + half_width_px)


def _point3d_yband_filter(
    points_base: np.ndarray,
    depth_at_point: np.ndarray,
    u_at_point: np.ndarray,
    *,
    y_plane: float,
    tau_cover: float,
    delta_y_static_m: float,
    k_sigma: float,
) -> Tuple[np.ndarray, float]:
    pts = np.asarray(points_base, dtype=np.float64).reshape(-1, 3)
    if pts.size == 0:
        return np.zeros(0, dtype=bool), 0.0
    depth = np.asarray(depth_at_point, dtype=np.float64).reshape(-1)
    _ = np.asarray(u_at_point, dtype=np.float64).reshape(-1)
    delta = float(delta_y_static_m) + float(k_sigma) * realsense_d435_sigma_depth_m(depth)
    in_band = np.abs(pts[:, 1] - float(y_plane)) <= delta
    coverage = float(np.count_nonzero(in_band) / max(pts.shape[0], 1))
    if coverage < float(tau_cover):
        return np.zeros_like(in_band, dtype=bool), coverage
    return in_band, coverage


class PerceptionToMapAdapter:
    """Convert RGB-D detections into MapUpdateRequest.

    The adapter is deterministic and stateless.  Runtime map-change checks are
    handled by reference snapshot diff outside this transform.
    """

    VALID_TYPES = {"occupied", "target", "unknown", "occluded"}

    def __init__(
        self,
        *,
        y_plane: float = Y_PLANE_FIXED,
        delta_y_static_m: float = 0.091,
        tau_cover: float = 0.05,
        sensor_inflation_floor_m: float = DEFAULT_OBSTACLE_INFLATION_M,
        k_sigma: float = 2.5,
        occlusion_extra_inflation_m: float = 0.0,
        robust_percentile: float = 5.0,
        max_points_per_detection: int = 8000,
        max_active_detections: int | None = None,
        require_explicit_transform: bool = False,
    ) -> None:
        self.y_plane = float(y_plane)
        self.delta_y_static_m = float(delta_y_static_m)
        self.tau_cover = float(tau_cover)
        self.sensor_inflation_floor_m = float(sensor_inflation_floor_m)
        self.k_sigma = float(k_sigma)
        self.occlusion_extra_inflation_m = float(occlusion_extra_inflation_m)
        self.robust_percentile = float(robust_percentile)
        self.max_points_per_detection = int(max_points_per_detection)
        if max_active_detections is not None and int(max_active_detections) <= 0:
            raise ValueError("max_active_detections must be positive when provided")
        self.max_active_detections = None if max_active_detections is None else int(max_active_detections)
        self.require_explicit_transform = bool(require_explicit_transform)

    def from_frame(
        self,
        frame: Mapping[str, object] | object,
        perception_result: Iterable[Detection2D | Mapping[str, object]] | Mapping[str, object],
        decision_grasp: Mapping[str, object] | object | None = None,
        T_base_cam: np.ndarray | Sequence[Sequence[float]] | None = None,
        *,
        source: str = "perception_to_map",
        timestamp_s: float | None = None,
        sequence_id: int = 0,
        map_update_hz: float = 10.0,
    ) -> MapUpdateRequest:
        t0 = time.perf_counter()
        depth = np.asarray(_field(frame, "depth", "depth_m"), dtype=np.float64)
        if depth.ndim != 2:
            raise ValueError("frame depth must be a 2D array in meters")
        intrinsics = _coerce_intrinsics(_field(frame, "intrinsics", "K", default=None), depth.shape)
        detections = [_coerce_detection(d) for d in _iter_detections(perception_result)]
        tf_fallback = T_base_cam is None
        if tf_fallback and self.require_explicit_transform:
            raise ValueError("explicit T_base_cam is required when require_explicit_transform=True")
        T = nominal_t_base_cam(self.y_plane) if T_base_cam is None else np.asarray(T_base_cam, dtype=np.float64)
        if T.shape != (4, 4):
            raise ValueError("T_base_cam must have shape 4x4")
        stage_ms = {
            "stage_a_image_prefilter_latency_ms": 0.0,
            "stage_b_depth_pixels_latency_ms": 0.0,
            "stage_c_backproject_transform_latency_ms": 0.0,
            "stage_d_yband_filter_latency_ms": 0.0,
            "stage_e_rect_semantic_latency_ms": 0.0,
            "stage_f_request_assembly_latency_ms": 0.0,
        }

        buckets: dict[str, list[Rect]] = {kind: [] for kind in self.VALID_TYPES}
        inflation_by_kind: dict[str, list[float]] = {kind: [] for kind in self.VALID_TYPES}
        occlusion_specs_by_kind: dict[str, list[Tuple[str, float]]] = {
            "occupied": [],
            "target": [],
        }
        stats = {
            "image_detection_count_in": float(len(detections)),
            "image_detection_count_kept": 0.0,
            "image_detection_count_prefilter_kept": 0.0,
            "image_detection_count_active": 0.0,
            "image_detection_count_pruned_by_topk": 0.0,
            "max_active_detections_used": float(self.max_active_detections or 0),
            "points_total": 0.0,
            "points_in_yband": 0.0,
            "unknown_depth_fallback_count": 0.0,
            "tf_fallback": 1.0 if tf_fallback else 0.0,
        }
        coverages: list[float] = []
        distances: list[float] = []

        prefiltered: list[tuple[int, Detection2D, tuple[float, float, float, float, int]]] = []
        for det_idx, det in enumerate(detections):
            ts = time.perf_counter()
            det_depth_roi = _depth_roi(depth, det.bbox)
            has_depth, depth_min, depth_median = _depth_roi_summary(det_depth_roi)
            if has_depth:
                keep = _image_yband_prefilter_from_depth_min(
                    det.bbox,
                    depth.shape,
                    intrinsics,
                    self.delta_y_static_m,
                    depth_min,
                )
            else:
                keep = True
            rank = self._detection_relevance_rank(
                det,
                depth.shape,
                intrinsics,
                depth_median if has_depth else det.fallback_depth_m,
                det_idx,
            )
            stage_ms["stage_a_image_prefilter_latency_ms"] += (time.perf_counter() - ts) * 1000.0
            if not keep:
                continue
            stats["image_detection_count_prefilter_kept"] += 1.0
            prefiltered.append((det_idx, det, rank))

        active = self._select_active_detections(prefiltered)
        stats["image_detection_count_kept"] = float(len(prefiltered))
        stats["image_detection_count_active"] = float(len(active))
        stats["image_detection_count_pruned_by_topk"] = float(max(len(prefiltered) - len(active), 0))
        stats["active_detection_indices"] = tuple(int(idx) for idx, _, _ in active)

        for _, det, _ in active:
            ts = time.perf_counter()
            pixels_u, pixels_v, pixels_d = self._valid_detection_pixels(det, depth)
            stats["points_total"] += float(pixels_d.size)
            if pixels_d.size == 0:
                if det.fallback_depth_m is None:
                    stats["unknown_depth_fallback_count"] += 1.0
                    stage_ms["stage_b_depth_pixels_latency_ms"] += (time.perf_counter() - ts) * 1000.0
                    continue
                pixels_u, pixels_v, pixels_d = self._bbox_corner_depth_samples(det.bbox, depth.shape, det.fallback_depth_m)
            stage_ms["stage_b_depth_pixels_latency_ms"] += (time.perf_counter() - ts) * 1000.0

            ts = time.perf_counter()
            points_cam = self._backproject(pixels_u, pixels_v, pixels_d, intrinsics)
            points_base = (T[:3, :3] @ points_cam.T).T + T[:3, 3][None, :]
            stage_ms["stage_c_backproject_transform_latency_ms"] += (time.perf_counter() - ts) * 1000.0
            ts = time.perf_counter()
            in_band, coverage = _point3d_yband_filter(
                points_base,
                pixels_d,
                pixels_u,
                y_plane=self.y_plane,
                tau_cover=self.tau_cover,
                delta_y_static_m=self.delta_y_static_m,
                k_sigma=self.k_sigma,
            )
            coverages.append(coverage)
            if not np.any(in_band):
                stage_ms["stage_d_yband_filter_latency_ms"] += (time.perf_counter() - ts) * 1000.0
                continue
            stage_ms["stage_d_yband_filter_latency_ms"] += (time.perf_counter() - ts) * 1000.0

            ts = time.perf_counter()
            filtered = points_base[in_band]
            filtered_depth = pixels_d[in_band]
            stats["points_in_yband"] += float(filtered.shape[0])
            rect = self._xz_rect(filtered)
            kind = self._semantic_kind(det, decision_grasp, rect)
            buckets[kind].append(rect)
            distance = float(np.median(filtered_depth)) if filtered_depth.size else 0.0
            distances.append(distance)
            inflation = self._depth_aware_inflation(distance)
            inflation_by_kind[kind].append(inflation)
            if kind in occlusion_specs_by_kind:
                occlusion_specs_by_kind[kind].append(self._dominant_shadow(points_base=filtered, distance_m=distance))
            stage_ms["stage_e_rect_semantic_latency_ms"] += (time.perf_counter() - ts) * 1000.0

        ts = time.perf_counter()
        per_rect_inflation = (
            inflation_by_kind["occupied"]
            + inflation_by_kind["target"]
            + inflation_by_kind["unknown"]
            + inflation_by_kind["occluded"]
        )
        occlusion_per_rect = occlusion_specs_by_kind["occupied"] + occlusion_specs_by_kind["target"]
        stats["yband_coverage_ratio_per_obj"] = float(np.mean(coverages)) if coverages else 0.0
        stats["camera_to_obstacle_distance_m"] = float(np.mean(distances)) if distances else 0.0
        stats["inflation_used_m_per_obj"] = tuple(float(v) for v in per_rect_inflation)
        stats["shadow_direction_per_obj"] = tuple(direction for direction, _ in occlusion_per_rect)
        stage_ms["stage_f_request_assembly_latency_ms"] += (time.perf_counter() - ts) * 1000.0
        stats.update(stage_ms)
        stats["adapter_latency_ms"] = float((time.perf_counter() - t0) * 1000.0)

        return MapUpdateRequest(
            occupied_rects=tuple(buckets["occupied"]),
            target_rects=tuple(buckets["target"]),
            unknown_rects=tuple(buckets["unknown"]),
            occluded_rects=tuple(buckets["occluded"]),
            inflation_m=float(self.sensor_inflation_floor_m),
            sensor_inflation_m=float(self.sensor_inflation_floor_m),
            occlusion_extra_inflation_m=float(self.occlusion_extra_inflation_m),
            per_rect_inflation_m=tuple(per_rect_inflation),
            occlusion_per_rect=tuple(occlusion_per_rect),
            map_update_hz=float(map_update_hz),
            source=str(source),
            timestamp_s=float(_field(frame, "timestamp_s", default=0.0) if timestamp_s is None else timestamp_s),
            sequence_id=int(sequence_id),
            adapter_stats=stats,
        )

    def _valid_detection_pixels(self, det: Detection2D, depth: np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        if det.mask is None:
            u0, v0, u1, v1 = _normalize_bbox(det.bbox, depth.shape)
            u_range = np.arange(int(u0), int(u1) + 1, dtype=np.int64)
            v_range = np.arange(int(v0), int(v1) + 1, dtype=np.int64)
            uu, vv = np.meshgrid(u_range, v_range)
            u = uu.ravel()
            v = vv.ravel()
        else:
            mask = _detection_mask(det, depth.shape)
            v, u = np.nonzero(mask)
        if u.size > self.max_points_per_detection > 0:
            stride = int(np.ceil(u.size / self.max_points_per_detection))
            u = u[::stride]
            v = v[::stride]
        d = depth[v, u]
        valid = np.isfinite(d) & (d > 0.0)
        return u[valid].astype(np.float64), v[valid].astype(np.float64), d[valid].astype(np.float64)

    @staticmethod
    def _bbox_corner_depth_samples(
        bbox: Sequence[float],
        shape: Tuple[int, int],
        depth_m: float,
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        u0, v0, u1, v1 = _normalize_bbox(bbox, shape)
        u = np.asarray([u0, u1, u0, u1, 0.5 * (u0 + u1)], dtype=np.float64)
        v = np.asarray([v0, v0, v1, v1, 0.5 * (v0 + v1)], dtype=np.float64)
        d = np.full(u.shape, float(depth_m), dtype=np.float64)
        return u, v, d

    @staticmethod
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

    def _xz_rect(self, points_base: np.ndarray) -> Rect:
        pts = np.asarray(points_base, dtype=np.float64).reshape(-1, 3)
        p = min(max(self.robust_percentile, 0.0), 49.0)
        if p <= 0.0:
            x_lo = float(np.min(pts[:, 0]))
            x_hi = float(np.max(pts[:, 0]))
            z_lo = float(np.min(pts[:, 2]))
            z_hi = float(np.max(pts[:, 2]))
        else:
            quantiles = np.percentile(pts[:, (0, 2)], [p, 100.0 - p], axis=0)
            x_lo = float(quantiles[0, 0])
            z_lo = float(quantiles[0, 1])
            x_hi = float(quantiles[1, 0])
            z_hi = float(quantiles[1, 1])
        eps = 1.0e-3
        if x_hi <= x_lo:
            x_lo -= eps
            x_hi += eps
        if z_hi <= z_lo:
            z_lo -= eps
            z_hi += eps
        return x_lo, z_hi, x_hi, z_lo

    def _depth_aware_inflation(self, distance_m: float) -> float:
        sigma = float(realsense_d435_sigma_depth_m(float(distance_m)))
        return max(float(self.sensor_inflation_floor_m), float(self.k_sigma) * sigma)

    @staticmethod
    def _semantic_kind(
        det: Detection2D,
        decision_grasp: Mapping[str, object] | object | None,
        rect: Rect,
    ) -> str:
        kind = str(det.semantic_type).strip().lower()
        aliases = {
            "obstacle": "occupied",
            "object": "occupied",
            "occupied": "occupied",
            "target": "target",
            "unknown": "unknown",
            "occluded": "occluded",
        }
        mapped = aliases.get(kind, "occupied")
        if mapped == "occupied" and _overlaps_decision_target(decision_grasp, rect):
            return "target"
        return mapped

    @staticmethod
    def _dominant_shadow(*, points_base: np.ndarray, distance_m: float) -> Tuple[str, float]:
        pts = np.asarray(points_base, dtype=np.float64).reshape(-1, 3)
        center = np.mean(pts[:, [0, 2]], axis=0)
        if abs(center[0]) >= abs(center[1]):
            direction = "+x" if center[0] >= 0.0 else "-x"
        else:
            direction = "+z" if center[1] >= 0.0 else "-z"
        return direction, min(0.15, 0.4 * max(float(distance_m), 0.0))

    def _select_active_detections(
        self,
        detections: list[tuple[int, Detection2D, tuple[float, float, float, float, int]]],
    ) -> list[tuple[int, Detection2D, tuple[float, float, float, float, int]]]:
        if self.max_active_detections is None or len(detections) <= self.max_active_detections:
            return detections
        return sorted(detections, key=lambda item: item[2])[: self.max_active_detections]

    @staticmethod
    def _detection_relevance_rank(
        det: Detection2D,
        frame_shape: Tuple[int, int],
        intrinsics: CameraIntrinsics,
        depth_median_m: float | None,
        original_index: int,
    ) -> tuple[float, float, float, float, int]:
        u0, _, u1, _ = _normalize_bbox(det.bbox, frame_shape)
        if u0 <= float(intrinsics.cx) <= u1:
            centerline_gap_px = 0.0
        else:
            centerline_gap_px = min(abs(float(intrinsics.cx) - u0), abs(float(intrinsics.cx) - u1))
        center_px = 0.5 * (u0 + u1)
        depth_rank = float(depth_median_m) if depth_median_m is not None and np.isfinite(depth_median_m) else float("inf")
        semantic_rank = {
            "target": 0.0,
            "occupied": 1.0,
            "obstacle": 1.0,
            "object": 1.0,
            "unknown": 2.0,
            "occluded": 3.0,
        }.get(str(det.semantic_type).strip().lower(), 1.0)
        return (
            semantic_rank,
            float(centerline_gap_px),
            abs(float(center_px) - float(intrinsics.cx)),
            depth_rank - float(det.score) * 1.0e-6,
            int(original_index),
        )


def _iter_detections(
    perception_result: Iterable[Detection2D | Mapping[str, object]] | Mapping[str, object],
) -> Iterable[Detection2D | Mapping[str, object]]:
    if isinstance(perception_result, Mapping):
        for key in ("detections", "objects", "results"):
            if key in perception_result:
                return perception_result[key]  # type: ignore[return-value]
        return (perception_result,)
    return perception_result


def _coerce_detection(value: Detection2D | Mapping[str, object]) -> Detection2D:
    if isinstance(value, Detection2D):
        return value
    data = dict(value)
    bbox = data.get("bbox", data.get("box"))
    if bbox is None:
        raise ValueError("detection requires bbox")
    bbox_tuple = tuple(float(v) for v in bbox)  # type: ignore[arg-type]
    if len(bbox_tuple) != 4:
        raise ValueError("detection bbox must be (u0, v0, u1, v1)")
    return Detection2D(
        semantic_type=str(data.get("semantic_type", data.get("type", "occupied"))),
        bbox=bbox_tuple,  # type: ignore[arg-type]
        mask=None if data.get("mask") is None else np.asarray(data.get("mask"), dtype=bool),
        score=float(data.get("score", 1.0)),
        source_id=str(data.get("source_id", "")),
        fallback_depth_m=None if data.get("fallback_depth_m", data.get("depth_m")) is None else float(data.get("fallback_depth_m", data.get("depth_m"))),
    )


def _coerce_intrinsics(value: object, depth_shape: Tuple[int, int]) -> CameraIntrinsics:
    h, w = depth_shape
    if isinstance(value, CameraIntrinsics):
        return value
    if value is None:
        return CameraIntrinsics(fx=615.0, fy=615.0, cx=0.5 * (w - 1), cy=0.5 * (h - 1), width=w, height=h)
    if isinstance(value, Mapping):
        data = dict(value)
        return CameraIntrinsics(
            fx=float(data["fx"]),
            fy=float(data["fy"]),
            cx=float(data["cx"]),
            cy=float(data["cy"]),
            width=int(data.get("width", w)),
            height=int(data.get("height", h)),
        )
    arr = np.asarray(value, dtype=np.float64)
    if arr.shape == (3, 3):
        return CameraIntrinsics(arr[0, 0], arr[1, 1], arr[0, 2], arr[1, 2], width=w, height=h)
    raise ValueError("intrinsics must be CameraIntrinsics, mapping, 3x3 K, or None")


def _detection_mask(det: Detection2D, shape: Tuple[int, int]) -> np.ndarray:
    h, w = shape
    if det.mask is not None:
        arr = np.asarray(det.mask, dtype=bool)
        if arr.shape != shape:
            raise ValueError(f"detection mask shape {arr.shape} does not match depth shape {shape}")
        return arr
    u0, v0, u1, v1 = _normalize_bbox(det.bbox, shape)
    mask = np.zeros(shape, dtype=bool)
    mask[int(v0):int(v1) + 1, int(u0):int(u1) + 1] = True
    return mask


def _depth_roi(depth: np.ndarray, bbox: Sequence[float]) -> np.ndarray:
    u0, v0, u1, v1 = _normalize_bbox(bbox, depth.shape)
    return np.asarray(depth[int(v0):int(v1) + 1, int(u0):int(u1) + 1], dtype=np.float64)


def _depth_roi_summary(depth_roi: np.ndarray) -> Tuple[bool, float, float]:
    finite = np.asarray(depth_roi, dtype=np.float64)
    finite = finite[np.isfinite(finite) & (finite > 0.0)]
    if finite.size == 0:
        return False, float("inf"), float("inf")
    # The image-space prefilter is allowed to be conservative.  Using min depth
    # widens the image band, avoids a per-detection percentile, and never drops
    # evidence that the previous 5th-percentile gate would have kept.
    return True, max(float(np.min(finite)), 1.0e-6), float(np.mean(finite))


def _normalize_bbox(bbox: Sequence[float], shape: Tuple[int, int]) -> Tuple[float, float, float, float]:
    if len(bbox) != 4:
        raise ValueError("bbox must have four values")
    h, w = shape
    u0, v0, u1, v1 = (float(bbox[0]), float(bbox[1]), float(bbox[2]), float(bbox[3]))
    u_lo, u_hi = (u0, u1) if u0 <= u1 else (u1, u0)
    v_lo, v_hi = (v0, v1) if v0 <= v1 else (v1, v0)
    u_lo = min(max(u_lo, 0.0), float(w - 1))
    u_hi = min(max(u_hi, 0.0), float(w - 1))
    v_lo = min(max(v_lo, 0.0), float(h - 1))
    v_hi = min(max(v_hi, 0.0), float(h - 1))
    return u_lo, v_lo, u_hi, v_hi


def _overlaps_decision_target(decision_grasp: Mapping[str, object] | object | None, rect: Rect) -> bool:
    if decision_grasp is None:
        return False
    target_rect = _optional_field(decision_grasp, "target_rect", "target_xz_rect")
    if target_rect is not None:
        try:
            return _rects_intersect(rect, tuple(float(v) for v in target_rect))  # type: ignore[arg-type]
        except Exception:
            return False
    target_xz = _optional_field(decision_grasp, "target_xz", "goal_xz")
    if target_xz is None:
        T = _optional_field(decision_grasp, "T_map_target", "T_base_target")
        if T is not None:
            arr = np.asarray(T, dtype=np.float64)
            if arr.shape == (4, 4):
                target_xz = (float(arr[0, 3]), float(arr[2, 3]))
    if target_xz is None:
        return False
    try:
        x, z = (float(v) for v in target_xz)  # type: ignore[assignment]
    except Exception:
        return False
    xl, zt, xr, zb = rect
    tol = 0.05
    return (xl - tol) <= x <= (xr + tol) and (zb - tol) <= z <= (zt + tol)


def _rects_intersect(a: Rect, b: Sequence[float]) -> bool:
    axl, azt, axr, azb = a
    bxl, bzt, bxr, bzb = (float(b[0]), float(b[1]), float(b[2]), float(b[3]))
    return not (axr < bxl or bxr < axl or azt < bzb or bzt < azb)


def _optional_field(obj: Mapping[str, object] | object, *names: str) -> object | None:
    if isinstance(obj, Mapping):
        for name in names:
            if name in obj:
                return obj[name]
        return None
    for name in names:
        if hasattr(obj, name):
            return getattr(obj, name)
    return None


_MISSING = object()


def _field(obj: Mapping[str, object] | object, *names: str, default: object = _MISSING) -> object:
    if isinstance(obj, Mapping):
        for name in names:
            if name in obj:
                return obj[name]
        if default is not _MISSING:
            return default
        raise KeyError(f"missing required field: {names[0]}")
    for name in names:
        if hasattr(obj, name):
            return getattr(obj, name)
    if default is not _MISSING:
        return default
    raise AttributeError(f"missing required field: {names[0]}")
