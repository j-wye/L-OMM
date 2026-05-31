#!/usr/bin/env python3
"""Bridge CameraPreprocessor FrameData into the map-update contract.

This module deliberately stays below ROS2 and camera ownership.  The only live
RGB-D ingress is ``preprocessing.CameraPreprocessor``; this bridge consumes the
already-synchronized ``FrameData`` object and already-produced perception
evidence, then calls ``PerceptionToMapAdapter`` and ``MapUpdateLayer``.
"""
from __future__ import annotations

import json
import os
import time
from dataclasses import dataclass, replace
from typing import Any, Iterable, Mapping, Sequence

import numpy as np

from .active_map_snapshot import ActiveMapSnapshot
from .cell_projection_occupancy import (
    CellProjectionOccupancyConfig,
    CellProjectionOccupancyEstimator,
    CellProjectionOccupancyEvidence,
    component_rects_from_mask,
    diagnostic_masks_from_evidence,
)
from .depth_geometry_evidence import DepthGeometryConfig, DepthGeometryEvidence, DepthGeometryEvidenceBuilder
from .map_update_layer import MapUpdateLayer
from .map_update_request import MapUpdateRequest
from .perception_to_map import CameraIntrinsics, Detection2D, PerceptionToMapAdapter, nominal_t_base_cam
from .snapshot_validator import SnapshotValidator


DEPTH_RELEVANT_MAX_M = 1.0
CROP_HALF_PIXELS = 200


def lateral_crop_keep_columns(width: int, half_pixels: int) -> np.ndarray:
    width_i = int(width)
    half = max(int(half_pixels), 0)
    if width_i < 0:
        raise ValueError("width must be non-negative")
    if half == 0:
        return np.zeros(width_i, dtype=bool)
    if 2 * half >= width_i:
        return np.ones(width_i, dtype=bool)
    center = width_i // 2
    u_min = max(0, center - half)
    u_max = min(width_i, center + half)
    keep = np.zeros(width_i, dtype=bool)
    keep[u_min:u_max] = True
    return keep


def apply_depth_cap(depth: np.ndarray, max_depth_m: float) -> tuple[np.ndarray, dict[str, float]]:
    arr = np.asarray(depth, dtype=np.float64)
    valid = np.isfinite(arr) & (arr > 0.0) & (arr <= float(max_depth_m))
    out = np.where(valid, arr, 0.0)
    return out, {
        "depth_cap_max_m": float(max_depth_m),
        "depth_cap_keep_ratio": float(np.count_nonzero(valid) / max(arr.size, 1)),
        "depth_cap_drop_ratio": float(1.0 - (np.count_nonzero(valid) / max(arr.size, 1))),
    }


def apply_lateral_crop(image_or_depth: np.ndarray, half_pixels: int) -> tuple[np.ndarray, dict[str, float]]:
    arr = np.asarray(image_or_depth)
    if arr.ndim < 2:
        raise ValueError("apply_lateral_crop expects at least a 2D array")
    width = int(arr.shape[1])
    half = max(int(half_pixels), 0)
    keep = lateral_crop_keep_columns(width, half)
    out = np.array(arr, copy=True)
    out[:, ~keep, ...] = 0
    return out, {
        "lateral_crop_half_pixels": float(half),
        "lateral_crop_keep_ratio": float(np.count_nonzero(keep) / max(width, 1)),
        "lateral_crop_drop_ratio": float(1.0 - (np.count_nonzero(keep) / max(width, 1))),
    }


def speed_prefilter_unknown_candidate_mask(
    depth: np.ndarray,
    *,
    max_depth_m: float = DEPTH_RELEVANT_MAX_M,
    crop_half_pixels: int = CROP_HALF_PIXELS,
) -> np.ndarray:
    """Return pixels where invalid depth should still be treated as unknown.

    Depth cap and lateral crop deliberately ignore pixels for speed. Those
    ignored pixels must not be converted into conservative unknown obstacles.
    True invalid sensor pixels inside the kept lateral region remain unknown
    candidates.
    """

    arr = np.asarray(depth, dtype=np.float64)
    if arr.ndim != 2:
        raise ValueError("depth must be a 2D array")
    lateral_keep = lateral_crop_keep_columns(int(arr.shape[1]), int(crop_half_pixels))[None, :]
    true_invalid = (~np.isfinite(arr)) | (arr <= 0.0)
    within_relevant_depth = true_invalid | (arr <= float(max_depth_m))
    return lateral_keep & true_invalid & within_relevant_depth


def apply_speed_prefilter(
    rgb: np.ndarray | None,
    depth: np.ndarray,
    *,
    max_depth_m: float = DEPTH_RELEVANT_MAX_M,
    crop_half_pixels: int = CROP_HALF_PIXELS,
) -> tuple[np.ndarray | None, np.ndarray, dict[str, float]]:
    depth_before = np.asarray(depth, dtype=np.float64)
    depth_capped, depth_stats = apply_depth_cap(depth_before, max_depth_m=max_depth_m)
    depth_filtered, crop_stats = apply_lateral_crop(depth_capped, half_pixels=crop_half_pixels)
    rgb_filtered = None
    if rgb is not None:
        rgb_filtered, _ = apply_lateral_crop(np.asarray(rgb), half_pixels=crop_half_pixels)
    kept = np.isfinite(depth_filtered) & (depth_filtered > 0.0)
    stats: dict[str, float] = {}
    stats.update(depth_stats)
    stats.update(crop_stats)
    stats["speed_prefilter_total_keep_ratio"] = float(np.count_nonzero(kept) / max(depth_before.size, 1))
    stats["speed_prefilter_total_drop_ratio"] = float(1.0 - stats["speed_prefilter_total_keep_ratio"])
    _jetson_sensitivity_log("apply_speed_prefilter", depth_before.shape, stats)
    return rgb_filtered, depth_filtered, stats


@dataclass(frozen=True)
class CameraBridgeResult:
    request: MapUpdateRequest
    snapshot: ActiveMapSnapshot
    validation: Mapping[str, float]
    stats: Mapping[str, Any]


class CameraFrameMapUpdateBridge:
    """Convert FrameData + perception evidence into an ActiveMapSnapshot."""

    SUPPORT_SURFACE_LABELS = {
        "support",
        "support_surface",
        "supporting_surface",
        "table",
        "table_surface",
        "surface",
    }
    NEARBY_GEOMETRY_LABELS = {
        "nearby",
        "nearby_geometry",
        "adjacent_object",
        "adjacent_geometry",
        "neighbor",
        "neighbour",
    }

    def __init__(
        self,
        *,
        adapter: PerceptionToMapAdapter | None = None,
        updater: MapUpdateLayer | None = None,
        depth_geometry: DepthGeometryEvidenceBuilder | None = None,
        cell_projection: CellProjectionOccupancyEstimator | None = None,
        depth_geometry_enabled: bool = True,
        cell_projection_enabled: bool = True,
        default_semantic_type: str = "target",
        source: str = "camera_frame_bridge",
        require_explicit_transform: bool = False,
        speed_prefilter_enabled: bool = True,
        depth_relevant_max_m: float = DEPTH_RELEVANT_MAX_M,
        crop_half_pixels: int = CROP_HALF_PIXELS,
    ) -> None:
        self.adapter = adapter or PerceptionToMapAdapter(max_active_detections=10)
        self.updater = updater or MapUpdateLayer()
        self.depth_geometry = depth_geometry or DepthGeometryEvidenceBuilder(DepthGeometryConfig())
        self.cell_projection = cell_projection or CellProjectionOccupancyEstimator(CellProjectionOccupancyConfig())
        self.depth_geometry_enabled = bool(depth_geometry_enabled)
        self.cell_projection_enabled = bool(cell_projection_enabled)
        self.default_semantic_type = str(default_semantic_type)
        self.source = str(source)
        self.require_explicit_transform = bool(require_explicit_transform)
        self.speed_prefilter_enabled = bool(speed_prefilter_enabled)
        self.depth_relevant_max_m = float(depth_relevant_max_m)
        self.crop_half_pixels = int(crop_half_pixels)
        self.validator = SnapshotValidator()

    def build_request(
        self,
        frame: Mapping[str, object] | object,
        perception_evidence: Any,
        decision_grasp: Mapping[str, object] | object | None = None,
        T_base_cam: np.ndarray | Sequence[Sequence[float]] | None = None,
        *,
        handle: Any | None = None,
        sequence_id: int = 0,
        map_update_hz: float = 10.0,
    ) -> MapUpdateRequest:
        adapter_frame = frame_to_adapter_frame(frame)
        prefilter_stats: dict[str, Any] = {"speed_prefilter_enabled": bool(self.speed_prefilter_enabled)}
        depth_before = np.asarray(adapter_frame["depth"], dtype=np.float64)
        if self.speed_prefilter_enabled:
            unknown_candidate_mask = speed_prefilter_unknown_candidate_mask(
                depth_before,
                max_depth_m=self.depth_relevant_max_m,
                crop_half_pixels=self.crop_half_pixels,
            )
            _, filtered_depth, prefilter_stats = apply_speed_prefilter(
                None,
                depth_before,
                max_depth_m=self.depth_relevant_max_m,
                crop_half_pixels=self.crop_half_pixels,
            )
            prefilter_stats["speed_prefilter_enabled"] = True
            adapter_frame = dict(adapter_frame)
            adapter_frame["depth"] = filtered_depth
        else:
            unknown_candidate_mask = None
        detections = self.evidence_to_detections(
            perception_evidence,
            default_semantic_type=self.default_semantic_type,
        )
        bridge_stats = self._bridge_stats(frame, adapter_frame, detections)
        if T_base_cam is None and self.require_explicit_transform:
            raise ValueError("explicit T_base_cam is required when require_explicit_transform=True")
        request = self.adapter.from_frame(
            adapter_frame,
            detections,
            decision_grasp=decision_grasp,
            T_base_cam=T_base_cam,
            source=self.source,
            timestamp_s=float(adapter_frame["timestamp_s"]),
            sequence_id=int(sequence_id),
            map_update_hz=float(map_update_hz),
        )
        semantic_request = request
        if self.depth_geometry_enabled and handle is not None:
            T_depth = nominal_t_base_cam(self.adapter.y_plane) if T_base_cam is None else np.asarray(T_base_cam, dtype=np.float64)
            depth_evidence = self.depth_geometry.build(
                depth_m=np.asarray(adapter_frame["depth"], dtype=np.float64),
                intrinsics=adapter_frame["intrinsics"],  # type: ignore[arg-type]
                T_base_cam=T_depth,
                handle=handle,
                unknown_candidate_mask=unknown_candidate_mask,
            )
            request = self._augment_request_with_depth_geometry(request, depth_evidence)
            depth_stats = dict(depth_evidence.stats)
            depth_stats["depth_geometry_tf_fallback"] = 1.0 if T_base_cam is None else 0.0
        elif self.depth_geometry_enabled:
            depth_stats = self.depth_geometry.skipped_stats(reason="handle_not_provided")
        else:
            depth_stats = self.depth_geometry.skipped_stats(reason="disabled")

        if self.cell_projection_enabled and handle is not None and T_base_cam is not None:
            cell_evidence = self.cell_projection.build(
                depth_m=np.asarray(adapter_frame["depth"], dtype=np.float64),
                intrinsics=adapter_frame["intrinsics"],  # type: ignore[arg-type]
                T_base_cam=np.asarray(T_base_cam, dtype=np.float64),
                handle=handle,
                unknown_candidate_mask=unknown_candidate_mask,
            )
            request, cell_stats = self._augment_request_with_cell_projection(
                request,
                cell_evidence,
                handle=handle,
            )
        elif self.cell_projection_enabled and handle is None:
            cell_stats = self.cell_projection.skipped_stats(reason="handle_not_provided")
        elif self.cell_projection_enabled:
            cell_stats = self.cell_projection.skipped_stats(reason="T_base_cam_not_provided")
        else:
            cell_stats = self.cell_projection.skipped_stats(reason="disabled")

        merged_stats = dict(request.adapter_stats)
        merged_stats.update(bridge_stats)
        merged_stats.update(prefilter_stats)
        merged_stats.update(depth_stats)
        merged_stats.update(cell_stats)
        merged_stats.update(self._request_stats(request, detections, semantic_request=semantic_request))
        return replace(request, adapter_stats=merged_stats)

    def process(
        self,
        frame: Mapping[str, object] | object,
        perception_evidence: Any,
        handle: Any,
        *,
        mu_min: float,
        decision_grasp: Mapping[str, object] | object | None = None,
        T_base_cam: np.ndarray | Sequence[Sequence[float]] | None = None,
        sequence_id: int = 0,
        map_update_hz: float = 10.0,
    ) -> CameraBridgeResult:
        request = self.build_request(
            frame,
            perception_evidence,
            decision_grasp=decision_grasp,
            T_base_cam=T_base_cam,
            handle=handle,
            sequence_id=sequence_id,
            map_update_hz=map_update_hz,
        )
        snapshot = self.updater.build(handle, mu_min=float(mu_min), request=request)
        validation = self.validator.validate(snapshot)
        stats = dict(request.adapter_stats)
        stats.update(validation)
        stats.update(
            {
                "target_blocked": bool(float(snapshot.stats.get("target_cells", 0.0)) > 0.0),
                "request_rects_blocked": int(request.semantic_rect_count),
                "snapshot_blocked_cells": float(snapshot.stats.get("blocked_cells", 0.0)),
            }
        )
        return CameraBridgeResult(
            request=request,
            snapshot=snapshot,
            validation=validation,
            stats=stats,
        )

    @classmethod
    def evidence_to_detections(
        cls,
        perception_evidence: Any,
        *,
        default_semantic_type: str = "target",
    ) -> list[Detection2D]:
        out: list[Detection2D] = []
        for item in _iter_evidence(perception_evidence):
            det = cls._coerce_detection(item, default_semantic_type=default_semantic_type)
            if det is not None:
                out.append(det)
        return out

    @classmethod
    def _coerce_detection(cls, item: Any, *, default_semantic_type: str) -> Detection2D | None:
        if isinstance(item, Detection2D):
            return item
        valid = _optional_field(item, "valid")
        if valid is not None and not _as_bool(valid):
            return None
        bbox = _optional_field(item, "bbox", "box")
        if bbox is None:
            return None
        semantic_raw = _optional_field(item, "semantic_type", "type", "label", "class_name")
        semantic_input = str(semantic_raw) if semantic_raw is not None else str(default_semantic_type)
        semantic_type = _map_semantic_type_to_adapter(semantic_input)
        mask_value = _optional_field(item, "mask")
        score_value = _optional_field(item, "score", "confidence")
        source_id = _optional_field(item, "source_id", "id", "track_id")
        return Detection2D(
            semantic_type=semantic_type,
            bbox=tuple(float(v) for v in _to_numpy(bbox).reshape(-1)[:4]),  # type: ignore[arg-type]
            mask=None if mask_value is None else np.asarray(_to_numpy(mask_value), dtype=bool),
            score=1.0 if score_value is None else float(np.asarray(_to_numpy(score_value)).reshape(-1)[0]),
            source_id=str(source_id if source_id is not None else semantic_input),
            fallback_depth_m=None,
        )

    @classmethod
    def _bridge_stats(
        cls,
        frame: Mapping[str, object] | object,
        adapter_frame: Mapping[str, object],
        detections: Sequence[Detection2D],
    ) -> dict[str, Any]:
        raw_depth = np.asarray(_required_field(frame, "depth"))
        depth_m = np.asarray(adapter_frame["depth"], dtype=np.float64)
        invalid = (~np.isfinite(depth_m)) | (depth_m <= 0.0)
        source_labels = [str(det.source_id).strip().lower() for det in detections]
        return {
            "camera_preprocessor_reused": True,
            "new_camera_subscriber_added": False,
            "frame_timestamp_shared": True,
            "frame_timestamp_s": float(adapter_frame["timestamp_s"]),
            "depth_unit": "uint16_mm_to_float_m" if np.issubdtype(raw_depth.dtype, np.integer) else "float_m",
            "depth_dtype_in": str(raw_depth.dtype),
            "depth_dtype_adapter": str(depth_m.dtype),
            "invalid_depth_ratio": float(np.count_nonzero(invalid) / max(depth_m.size, 1)),
            "bridge_detection_count_in": int(len(detections)),
            "target_candidate_count": int(sum(det.semantic_type == "target" for det in detections)),
            "support_surface_candidate_count": int(sum(label in cls.SUPPORT_SURFACE_LABELS for label in source_labels)),
            "nearby_geometry_candidate_count": int(sum(label in cls.NEARBY_GEOMETRY_LABELS for label in source_labels)),
        }

    @classmethod
    def _request_stats(
        cls,
        request: MapUpdateRequest,
        detections: Sequence[Detection2D],
        *,
        semantic_request: MapUpdateRequest | None = None,
    ) -> dict[str, Any]:
        source_labels = [str(det.source_id).strip().lower() for det in detections]
        semantic = request if semantic_request is None else semantic_request
        detections_blocked = int(semantic.semantic_rect_count)
        return {
            "semantic_evidence_rect_count": int(semantic.semantic_rect_count),
            "semantic_occupied_rect_count": int(len(semantic.occupied_rects)),
            "semantic_target_rect_count": int(len(semantic.target_rects)),
            "semantic_unknown_rect_count": int(len(semantic.unknown_rects)),
            "semantic_occluded_rect_count": int(len(semantic.occluded_rects)),
            "total_request_rect_count": int(request.semantic_rect_count),
            "detections_in": int(len(detections)),
            "detections_blocked": detections_blocked,
            "filtered_out_of_task_count": int(max(len(detections) - detections_blocked, 0)),
            "target_blocked": bool(len(request.target_rects) > 0),
            "support_surface_blocked": bool(
                any(label in cls.SUPPORT_SURFACE_LABELS for label in source_labels)
                and len(request.occupied_rects) > 0
            ),
            "nearby_geometry_blocked": bool(
                any(label in cls.NEARBY_GEOMETRY_LABELS for label in source_labels)
                and len(request.occupied_rects) > 0
            ),
        }

    @staticmethod
    def _augment_request_with_depth_geometry(
        request: MapUpdateRequest,
        depth_evidence: DepthGeometryEvidence,
    ) -> MapUpdateRequest:
        depth_occupied = tuple(depth_evidence.occupied_rects)
        depth_unknown = tuple(depth_evidence.unknown_rects)
        if not depth_occupied and not depth_unknown:
            return request

        old_occ = tuple(request.occupied_rects)
        old_target = tuple(request.target_rects)
        old_unknown = tuple(request.unknown_rects)
        old_occluded = tuple(request.occluded_rects)

        per_rect = _split_per_rect_inflation(request)
        occ_vals, target_vals, unknown_vals, occluded_vals = per_rect
        new_per_rect = (
            occ_vals
            + tuple(depth_evidence.occupied_inflation_m)
            + target_vals
            + unknown_vals
            + tuple(depth_evidence.unknown_inflation_m)
            + occluded_vals
        )

        old_occ_specs, old_target_specs = _split_occlusion_specs(request)
        new_occlusion_specs = (
            old_occ_specs
            + tuple(("none", 0.0) for _ in depth_occupied)
            + old_target_specs
        )

        return replace(
            request,
            occupied_rects=old_occ + depth_occupied,
            unknown_rects=old_unknown + depth_unknown,
            per_rect_inflation_m=new_per_rect,
            occlusion_per_rect=new_occlusion_specs,
        )

    @staticmethod
    def _augment_request_with_cell_projection(
        request: MapUpdateRequest,
        cell_evidence: CellProjectionOccupancyEvidence,
        *,
        handle: Any,
    ) -> tuple[MapUpdateRequest, dict[str, Any]]:
        evidence_stats = dict(cell_evidence.stats)
        cfg_min = max(int(evidence_stats.get("cell_projection_min_component_cells", 1)), 1)
        occ_rects, occ_stats = component_rects_from_mask(handle, cell_evidence.occupied_mask, min_cells=cfg_min)
        occluded_rects, occluded_stats = component_rects_from_mask(handle, cell_evidence.occluded_mask, min_cells=cfg_min)
        unknown_rects, unknown_stats = component_rects_from_mask(handle, cell_evidence.unknown_mask, min_cells=cfg_min)

        old_occ = tuple(request.occupied_rects)
        old_target = tuple(request.target_rects)
        old_unknown = tuple(request.unknown_rects)
        old_occluded = tuple(request.occluded_rects)

        per_rect = _split_per_rect_inflation(request)
        occ_vals, target_vals, unknown_vals, occluded_vals = per_rect
        inflation = float(evidence_stats.get("cell_projection_rect_inflation_m", request.sensor_inflation_m))
        new_per_rect = (
            occ_vals
            + tuple(inflation for _ in occ_rects)
            + target_vals
            + unknown_vals
            + tuple(inflation for _ in unknown_rects)
            + occluded_vals
            + tuple(inflation for _ in occluded_rects)
        )

        old_occ_specs, old_target_specs = _split_occlusion_specs(request)
        new_occlusion_specs = (
            old_occ_specs
            + tuple(("none", 0.0) for _ in occ_rects)
            + old_target_specs
        )
        diagnostics = dict(request.diagnostic_masks)
        diagnostics.update(diagnostic_masks_from_evidence(cell_evidence))

        stats = evidence_stats
        for prefix, data in (
            ("cell_projection_occupied_rect", occ_stats),
            ("cell_projection_occluded_rect", occluded_stats),
            ("cell_projection_unknown_rect", unknown_stats),
        ):
            for key, value in data.items():
                stats[f"{prefix}_{key}"] = int(value)
        stats["cell_projection_occupied_rect_count"] = int(len(occ_rects))
        stats["cell_projection_occluded_rect_count"] = int(len(occluded_rects))
        stats["cell_projection_unknown_rect_count"] = int(len(unknown_rects))

        return replace(
            request,
            occupied_rects=old_occ + tuple(occ_rects),
            unknown_rects=old_unknown + tuple(unknown_rects),
            occluded_rects=old_occluded + tuple(occluded_rects),
            per_rect_inflation_m=new_per_rect,
            occlusion_per_rect=new_occlusion_specs,
            diagnostic_masks=diagnostics,
        ), stats


def frame_to_adapter_frame(frame: Mapping[str, object] | object) -> dict[str, object]:
    depth_m = depth_to_meters(_required_field(frame, "depth"))
    intrinsics = intrinsics_to_camera_intrinsics(_required_field(frame, "intrinsics"), depth_m.shape)
    timestamp = _optional_field(frame, "timestamp", "timestamp_s")
    return {
        "depth": depth_m,
        "intrinsics": intrinsics,
        "timestamp_s": 0.0 if timestamp is None else float(timestamp),
    }


def depth_to_meters(depth: object) -> np.ndarray:
    arr = np.asarray(depth)
    if arr.ndim != 2:
        raise ValueError("FrameData.depth must be a 2D array")
    if np.issubdtype(arr.dtype, np.integer):
        return arr.astype(np.float64) * 0.001
    return arr.astype(np.float64, copy=False)


def _split_per_rect_inflation(
    request: MapUpdateRequest,
) -> tuple[tuple[float, ...], tuple[float, ...], tuple[float, ...], tuple[float, ...]]:
    count = request.semantic_rect_count
    fallback = float(request.sensor_inflation_m)
    if request.per_rect_inflation_m is None:
        values = [fallback for _ in range(count)]
    else:
        values = [float(v) for v in request.per_rect_inflation_m]
        values.extend(fallback for _ in range(count - len(values)))
        values = values[:count]
    n_occ = len(request.occupied_rects)
    n_target = len(request.target_rects)
    n_unknown = len(request.unknown_rects)
    n_occluded = len(request.occluded_rects)
    idx = 0
    occ = tuple(values[idx:idx + n_occ])
    idx += n_occ
    target = tuple(values[idx:idx + n_target])
    idx += n_target
    unknown = tuple(values[idx:idx + n_unknown])
    idx += n_unknown
    occluded = tuple(values[idx:idx + n_occluded])
    return occ, target, unknown, occluded


def _split_occlusion_specs(
    request: MapUpdateRequest,
) -> tuple[tuple[tuple[str, float], ...], tuple[tuple[str, float], ...]]:
    count = len(request.occupied_rects) + len(request.target_rects)
    if request.occlusion_per_rect is None:
        values = [(str(request.occlusion_direction), float(request.occlusion_shadow_m)) for _ in range(count)]
    else:
        values = [(str(direction), float(distance)) for direction, distance in request.occlusion_per_rect]
        values.extend(("none", 0.0) for _ in range(count - len(values)))
        values = values[:count]
    n_occ = len(request.occupied_rects)
    return tuple(values[:n_occ]), tuple(values[n_occ:])


def intrinsics_to_camera_intrinsics(value: object, depth_shape: Sequence[int]) -> CameraIntrinsics:
    h, w = int(depth_shape[0]), int(depth_shape[1])
    if isinstance(value, CameraIntrinsics):
        return value
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
    K = np.asarray(value, dtype=np.float64)
    if K.shape != (3, 3):
        raise ValueError("FrameData.intrinsics must be a 3x3 matrix or CameraIntrinsics-compatible mapping")
    return CameraIntrinsics(
        fx=float(K[0, 0]),
        fy=float(K[1, 1]),
        cx=float(K[0, 2]),
        cy=float(K[1, 2]),
        width=w,
        height=h,
    )


def _iter_evidence(perception_evidence: Any) -> Iterable[Any]:
    if perception_evidence is None:
        return ()
    if isinstance(perception_evidence, Mapping):
        if "perception_result" in perception_evidence:
            return _iter_evidence(perception_evidence["perception_result"])
        for key in ("detections", "objects", "results", "evidence"):
            if key in perception_evidence:
                return _iter_evidence(perception_evidence[key])
        return (perception_evidence,)
    if isinstance(perception_evidence, (str, bytes)):
        return ()
    try:
        return tuple(perception_evidence)
    except TypeError:
        return (perception_evidence,)


def _map_semantic_type_to_adapter(label: str) -> str:
    norm = str(label).strip().lower()
    if norm == "target":
        return "target"
    if norm == "unknown":
        return "unknown"
    if norm == "occluded":
        return "occluded"
    return "occupied"


def _to_numpy(value: Any) -> np.ndarray:
    obj = value
    if hasattr(obj, "detach"):
        obj = obj.detach()
    if hasattr(obj, "cpu"):
        obj = obj.cpu()
    if hasattr(obj, "numpy"):
        return np.asarray(obj.numpy())
    return np.asarray(obj)


def _as_bool(value: Any) -> bool:
    arr = np.asarray(_to_numpy(value)).reshape(-1)
    if arr.size == 0:
        return False
    return bool(arr[0])


def _optional_field(obj: Any, *names: str) -> Any | None:
    if isinstance(obj, Mapping):
        for name in names:
            if name in obj:
                return obj[name]
        return None
    for name in names:
        if hasattr(obj, name):
            return getattr(obj, name)
    return None


def _required_field(obj: Any, name: str) -> Any:
    value = _optional_field(obj, name)
    if value is None:
        raise AttributeError(f"missing required FrameData field: {name}")
    return value


# === DIAGNOSTIC LOGGING (REMOVABLE) - JETSON_SENSITIVITY_SWEEP ===
def _jetson_sensitivity_log(event: str, depth_shape: Sequence[int], stats: Mapping[str, Any]) -> None:
    path = os.environ.get("JETSON_SENSITIVITY_SWEEP_LOG")
    if not path:
        return
    payload = {
        "timestamp_s": time.time(),
        "event": str(event),
        "depth_shape": [int(v) for v in depth_shape],
        "stats": dict(stats),
    }
    with open(path, "a", encoding="utf-8") as f:
        f.write(json.dumps(payload, ensure_ascii=False) + "\n")
# === END DIAGNOSTIC LOGGING ===
