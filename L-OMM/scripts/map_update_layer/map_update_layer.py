#!/usr/bin/env python3
"""Layered active-mask construction outside the control module."""
from __future__ import annotations

from typing import Sequence, Tuple

import numpy as np

try:
    from control_module.constants import Rect
    from control_module.map_handle import MapHandle
except ImportError:
    from constants import Rect
    from map_handle import MapHandle

from .active_map_snapshot import ActiveMapSnapshot
from .layered_mask_result import LayeredMaskResult
from .map_update_request import MapUpdateRequest
from .rect_rasterizer import RectRasterizer
from .snapshot_validator import SnapshotValidator


class MapUpdateLayer:
    """Convert semantic runtime layers into final active/blocked masks."""

    def __init__(self) -> None:
        self.rasterizer = RectRasterizer()
        self.validator = SnapshotValidator()

    def build(
        self,
        handle: MapHandle,
        *,
        mu_min: float,
        request: MapUpdateRequest | None = None,
    ) -> ActiveMapSnapshot:
        req = request or MapUpdateRequest()
        mu_min_eff, mu_min_from_meta = self._effective_mu_min(handle, float(mu_min))
        base = np.asarray(handle.mu_grid > mu_min_eff, dtype=bool)
        layers = self.build_layers(handle, req)
        final = base & (~layers.blocked_mask)
        observed_occupied = layers.occupied_mask | layers.target_mask

        total = float(max(base.size, 1))
        semantic_count = float(np.count_nonzero(layers.semantic_mask))
        inflation_added_count = float(np.count_nonzero(layers.inflation_added_mask))
        sensor_inflation_added_count = float(np.count_nonzero(layers.sensor_inflation_added_mask))
        occlusion_inflation_added_count = float(np.count_nonzero(layers.occlusion_inflation_added_mask))
        stats = {
            "map_update_source": str(req.source),
            "map_update_hz": float(req.map_update_hz),
            "map_update_timestamp_s": float(req.timestamp_s),
            "map_update_sequence_id": float(req.sequence_id),
            "mu_min": float(mu_min_eff),
            "mu_min_requested": float(mu_min),
            "mu_min_runtime_used": float(mu_min_eff),
            "mu_min_runtime_from_meta": 1.0 if mu_min_from_meta else 0.0,
            "inflation_m": float(req.inflation_m),
            "sensor_inflation_m": float(req.sensor_inflation_m),
            "occlusion_extra_inflation_m": float(req.occlusion_extra_inflation_m),
            "occlusion_shadow_m": float(req.occlusion_shadow_m),
            "occlusion_direction_code": self._direction_code(req.occlusion_direction),
            "base_feasible_cells": float(np.count_nonzero(base)),
            "base_feasible_ratio": float(np.count_nonzero(base) / total),
            "blocked_cells": float(np.count_nonzero(layers.blocked_mask)),
            "blocked_ratio": float(np.count_nonzero(layers.blocked_mask) / total),
            "final_feasible_cells": float(np.count_nonzero(final)),
            "final_feasible_ratio": float(np.count_nonzero(final) / total),
            "occupied_cells": float(np.count_nonzero(layers.occupied_mask)),
            "occupied_contract_cells": float(np.count_nonzero(observed_occupied)),
            "target_cells": float(np.count_nonzero(layers.target_mask)),
            "unknown_cells": float(np.count_nonzero(layers.unknown_mask)),
            "occluded_cells": float(np.count_nonzero(layers.occluded_mask)),
            "semantic_blocked_cells": semantic_count,
            "inflated_cells": float(np.count_nonzero(layers.inflated_mask)),
            "inflation_added_cells": inflation_added_count,
            "sensor_inflation_added_cells": sensor_inflation_added_count,
            "occlusion_inflation_added_cells": occlusion_inflation_added_count,
            "inflation_added_ratio_of_semantic": float(inflation_added_count / max(semantic_count, 1.0)),
            "occupied_rect_count": float(len(req.occupied_rects)),
            "target_rect_count": float(len(req.target_rects)),
            "unknown_rect_count": float(len(req.unknown_rects)),
            "occluded_rect_count": float(len(req.occluded_rects)),
            "generated_occluded_rect_count": float(layers.generated_occluded_rect_count),
            "total_occluded_rect_count": float(layers.total_occluded_rect_count),
            "controlled_slow_obstacle_assumption": 1.0,
        }
        if req.adapter_stats:
            stats["adapter_stats"] = dict(req.adapter_stats)
        layer_masks = layers.as_dict()
        for name, mask in dict(req.diagnostic_masks).items():
            layer_masks[str(name)] = np.asarray(mask, dtype=bool)

        snapshot = ActiveMapSnapshot(
            handle=handle,
            base_feasible_mask=base,
            final_active_mask=final,
            blocked_mask=layers.blocked_mask,
            occupied_mask=observed_occupied,
            display_rects=layers.display_rects,
            stats=stats,
            layer_masks=layer_masks,
            source=str(req.source),
            mu_min=float(mu_min_eff),
        )
        stats.update(self.validator.validate(snapshot))
        snapshot.stats = stats
        return snapshot

    def build_layers(self, handle: MapHandle, request: MapUpdateRequest) -> LayeredMaskResult:
        req = request
        occupied_rects = [self.normalize_rect(r) for r in req.occupied_rects]
        target_rects = [self.normalize_rect(r) for r in req.target_rects]
        unknown_rects = [self.normalize_rect(r) for r in req.unknown_rects]
        explicit_occluded_rects = [self.normalize_rect(r) for r in req.occluded_rects]

        base_semantic_count = (
            len(occupied_rects)
            + len(target_rects)
            + len(unknown_rects)
            + len(explicit_occluded_rects)
        )
        base_margins = self._per_rect_inflation_values(req, base_semantic_count)
        idx = 0
        occupied_margins = base_margins[idx:idx + len(occupied_rects)]
        idx += len(occupied_rects)
        target_margins = base_margins[idx:idx + len(target_rects)]
        idx += len(target_rects)
        unknown_margins = base_margins[idx:idx + len(unknown_rects)]
        idx += len(unknown_rects)
        explicit_occluded_margins = base_margins[idx:idx + len(explicit_occluded_rects)]

        generated_occluded_rects = []
        generated_occluded_margins = []
        shadow_sources = occupied_rects + target_rects
        shadow_source_margins = occupied_margins + target_margins
        for rect, margin, (direction, distance) in zip(
            shadow_sources,
            shadow_source_margins,
            self._occlusion_specs(req, len(shadow_sources)),
        ):
            if direction != "none" and distance > 0.0:
                generated_occluded_rects.append(self.shadow_rect(rect, direction, distance))
                generated_occluded_margins.append(float(margin))

        occluded_rects = explicit_occluded_rects + generated_occluded_rects
        occluded_margins = explicit_occluded_margins + generated_occluded_margins
        semantic_rects = occupied_rects + target_rects + unknown_rects + occluded_rects
        semantic_margins = occupied_margins + target_margins + unknown_margins + occluded_margins
        sensor_inflated_rects = [
            self.inflate_rect(r, margin)
            for r, margin in zip(semantic_rects, semantic_margins)
        ]
        occlusion_inflated_rects = [
            self.inflate_rect(r, margin + float(req.occlusion_extra_inflation_m))
            for r, margin in zip(occluded_rects, occluded_margins)
        ]

        occupied_mask = self.rect_mask(handle, occupied_rects)
        target_mask = self.rect_mask(handle, target_rects)
        unknown_mask = self.rect_mask(handle, unknown_rects)
        occluded_mask = self.rect_mask(handle, occluded_rects)
        semantic_mask = occupied_mask | target_mask | unknown_mask | occluded_mask
        sensor_inflated_mask = self.rect_mask(handle, sensor_inflated_rects)
        occlusion_inflated_mask = self.rect_mask(handle, occlusion_inflated_rects)
        inflated_mask = sensor_inflated_mask | occlusion_inflated_mask
        sensor_inflation_added_mask = sensor_inflated_mask & (~semantic_mask)
        occlusion_inflation_added_mask = occlusion_inflated_mask & (~semantic_mask) & (~sensor_inflated_mask)
        inflation_added_mask = inflated_mask & (~semantic_mask)
        blocked = semantic_mask | inflated_mask
        return LayeredMaskResult(
            occupied_mask=occupied_mask,
            target_mask=target_mask,
            unknown_mask=unknown_mask,
            occluded_mask=occluded_mask,
            semantic_mask=semantic_mask,
            inflated_mask=inflated_mask,
            inflation_added_mask=inflation_added_mask,
            sensor_inflated_mask=sensor_inflated_mask,
            sensor_inflation_added_mask=sensor_inflation_added_mask,
            occlusion_inflated_mask=occlusion_inflated_mask,
            occlusion_inflation_added_mask=occlusion_inflation_added_mask,
            blocked_mask=blocked,
            display_rects=semantic_rects + sensor_inflated_rects + occlusion_inflated_rects,
            generated_occluded_rect_count=len(generated_occluded_rects),
            total_occluded_rect_count=len(occluded_rects),
        )

    def rect_mask(self, handle: MapHandle, rects: Sequence[Rect]) -> np.ndarray:
        return self.rasterizer.rect_mask(handle, rects)

    @staticmethod
    def normalize_rect(rect: Sequence[float]) -> Rect:
        return RectRasterizer.normalize(rect)

    @staticmethod
    def inflate_rect(rect: Rect, margin: float) -> Rect:
        return RectRasterizer.inflate(rect, margin)

    @staticmethod
    def shadow_rect(rect: Rect, direction: str, distance: float) -> Rect:
        return RectRasterizer.shadow(rect, direction, distance)

    @staticmethod
    def samples_blocked(handle: MapHandle, xz: np.ndarray, blocked_mask: np.ndarray) -> Tuple[bool, int]:
        pts = np.asarray(xz, dtype=np.float64).reshape(-1, 2)
        n_x, n_z = blocked_mask.shape
        count = 0
        for x, z in pts:
            ix = int(round((float(x) - handle.x0) / handle.resolution_m))
            iz = int(round((float(z) - handle.z0) / handle.resolution_m))
            if ix < 0 or ix >= n_x or iz < 0 or iz >= n_z or bool(blocked_mask[ix, iz]):
                count += 1
        return count > 0, int(count)

    @staticmethod
    def _effective_mu_min(handle: MapHandle, requested_mu_min: float) -> Tuple[float, bool]:
        meta = getattr(handle, "meta", {}) or {}
        if "mu_min_runtime" in meta:
            return float(meta["mu_min_runtime"]), True
        return float(requested_mu_min), False

    @staticmethod
    def _per_rect_inflation_values(request: MapUpdateRequest, count: int) -> list[float]:
        fallback = float(request.sensor_inflation_m)
        values = request.per_rect_inflation_m
        if values is None:
            return [fallback for _ in range(count)]
        out = [float(v) for v in values]
        if len(out) > count:
            raise ValueError("per_rect_inflation_m has more entries than semantic rectangles")
        out.extend(fallback for _ in range(count - len(out)))
        return out

    @staticmethod
    def _occlusion_specs(request: MapUpdateRequest, count: int) -> list[Tuple[str, float]]:
        if request.occlusion_per_rect is None:
            return [(str(request.occlusion_direction), float(request.occlusion_shadow_m)) for _ in range(count)]
        specs = [(str(direction), float(distance)) for direction, distance in request.occlusion_per_rect]
        if len(specs) > count:
            raise ValueError("occlusion_per_rect has more entries than occupied+target rectangles")
        specs.extend(("none", 0.0) for _ in range(count - len(specs)))
        return specs

    @staticmethod
    def _direction_code(direction: str) -> float:
        return {
            "none": 0.0,
            "+x": 1.0,
            "-x": 2.0,
            "+z": 3.0,
            "-z": 4.0,
        }.get(str(direction), -1.0)
