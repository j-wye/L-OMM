#!/usr/bin/env python3
"""Offline perception-like input adapter for map-update layer tests.

The adapter accepts stored/synthetic detections that are already projected to
active-map rectangles.  It does not subscribe to camera topics and does not run
segmentation; live RGB-D integration remains outside the current scope.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable, Mapping, Sequence

try:
    from control_module.constants import DEFAULT_MAP_UPDATE_HZ, DEFAULT_OBSTACLE_INFLATION_M, Rect
except ImportError:
    from constants import DEFAULT_MAP_UPDATE_HZ, DEFAULT_OBSTACLE_INFLATION_M, Rect

from .map_update_request import MapUpdateRequest


@dataclass(frozen=True)
class OfflineDetection:
    """Stored detection-like object after offline projection to x-z space."""

    semantic_type: str
    rect: Rect
    score: float = 1.0
    source_id: str = ""


class OfflinePerceptionToMapAdapter:
    """Convert offline detection-like objects into MapUpdateRequest."""

    VALID_TYPES = {"occupied", "target", "unknown", "occluded"}

    def build_request(
        self,
        detections: Iterable[OfflineDetection | Mapping[str, object]],
        *,
        inflation_m: float = DEFAULT_OBSTACLE_INFLATION_M,
        map_update_hz: float = DEFAULT_MAP_UPDATE_HZ,
        source: str = "offline_perception_like",
        timestamp_s: float = 0.0,
        sequence_id: int = 0,
        occlusion_shadow_m: float = 0.0,
        occlusion_direction: str = "none",
    ) -> MapUpdateRequest:
        occupied = []
        target = []
        unknown = []
        occluded = []
        for item in detections:
            det = self._coerce(item)
            kind = det.semantic_type
            if kind == "occupied":
                occupied.append(det.rect)
            elif kind == "target":
                target.append(det.rect)
            elif kind == "unknown":
                unknown.append(det.rect)
            elif kind == "occluded":
                occluded.append(det.rect)
            else:
                raise ValueError(f"unknown offline detection semantic_type: {kind}")
        return MapUpdateRequest(
            occupied_rects=tuple(occupied),
            target_rects=tuple(target),
            unknown_rects=tuple(unknown),
            occluded_rects=tuple(occluded),
            inflation_m=float(inflation_m),
            map_update_hz=float(map_update_hz),
            occlusion_shadow_m=float(occlusion_shadow_m),
            occlusion_direction=str(occlusion_direction),
            source=str(source),
            timestamp_s=float(timestamp_s),
            sequence_id=int(sequence_id),
        )

    def _coerce(self, value: OfflineDetection | Mapping[str, object]) -> OfflineDetection:
        if isinstance(value, OfflineDetection):
            return self._validate(value)
        data = dict(value)
        rect_raw = data.get("rect")
        if rect_raw is None:
            raise ValueError("offline detection requires rect")
        rect_vals = tuple(float(v) for v in rect_raw)  # type: ignore[arg-type]
        if len(rect_vals) != 4:
            raise ValueError("offline detection rect must have four values")
        return self._validate(OfflineDetection(
            semantic_type=str(data.get("semantic_type", data.get("type", "occupied"))),
            rect=rect_vals,  # type: ignore[arg-type]
            score=float(data.get("score", 1.0)),
            source_id=str(data.get("source_id", "")),
        ))

    def _validate(self, det: OfflineDetection) -> OfflineDetection:
        kind = str(det.semantic_type).strip().lower()
        if kind not in self.VALID_TYPES:
            raise ValueError(f"semantic_type must be one of {sorted(self.VALID_TYPES)}")
        x_l, z_t, x_r, z_b = det.rect
        if x_r <= x_l:
            raise ValueError("offline detection rect requires x_r > x_l")
        if z_t <= z_b:
            raise ValueError("offline detection rect requires z_t > z_b")
        return OfflineDetection(kind, (float(x_l), float(z_t), float(x_r), float(z_b)), float(det.score), det.source_id)
