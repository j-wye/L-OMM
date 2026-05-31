#!/usr/bin/env python3
"""Semantic map-update request before conversion to final masks."""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Mapping, Sequence, Tuple

import numpy as np

try:
    from control_module.constants import DEFAULT_MAP_UPDATE_HZ, DEFAULT_OBSTACLE_INFLATION_M, Rect
except ImportError:
    from constants import DEFAULT_MAP_UPDATE_HZ, DEFAULT_OBSTACLE_INFLATION_M, Rect


@dataclass(frozen=True)
class MapUpdateRequest:
    """Input to the external map-update layer.

    This request is intentionally outside the control module.  It can be
    produced by perception, decision, a camera-rate map updater, or an
    experiment wrapper.
    """

    occupied_rects: Sequence[Rect] = field(default_factory=tuple)
    target_rects: Sequence[Rect] = field(default_factory=tuple)
    unknown_rects: Sequence[Rect] = field(default_factory=tuple)
    occluded_rects: Sequence[Rect] = field(default_factory=tuple)
    # Backward-compatible legacy alias.  If sensor_inflation_m is not supplied,
    # this value is used as the sensor-noise inflation margin.
    inflation_m: float = DEFAULT_OBSTACLE_INFLATION_M
    sensor_inflation_m: float | None = None
    occlusion_extra_inflation_m: float = 0.0
    per_rect_inflation_m: Sequence[float] | None = None
    occlusion_per_rect: Sequence[Tuple[str, float]] | None = None
    map_update_hz: float = DEFAULT_MAP_UPDATE_HZ
    occlusion_shadow_m: float = 0.0
    occlusion_direction: str = "none"
    source: str = "external"
    timestamp_s: float = 0.0
    sequence_id: int = 0
    adapter_stats: Mapping[str, object] = field(default_factory=dict)
    diagnostic_masks: Mapping[str, object] = field(default_factory=dict)

    def __post_init__(self) -> None:
        if self.inflation_m < 0.0:
            raise ValueError("inflation_m must be non-negative")
        sensor_inflation = self.inflation_m if self.sensor_inflation_m is None else self.sensor_inflation_m
        if sensor_inflation < 0.0:
            raise ValueError("sensor_inflation_m must be non-negative")
        object.__setattr__(self, "sensor_inflation_m", float(sensor_inflation))
        if self.occlusion_extra_inflation_m < 0.0:
            raise ValueError("occlusion_extra_inflation_m must be non-negative")
        if self.map_update_hz <= 0.0:
            raise ValueError("map_update_hz must be positive")
        if self.occlusion_shadow_m < 0.0:
            raise ValueError("occlusion_shadow_m must be non-negative")
        allowed = {"none", "+x", "-x", "+z", "-z"}
        if self.occlusion_direction not in allowed:
            raise ValueError(f"occlusion_direction must be one of {sorted(allowed)}")
        if self.per_rect_inflation_m is not None:
            per_rect = tuple(float(v) for v in self.per_rect_inflation_m)
            if any(v < 0.0 for v in per_rect):
                raise ValueError("per_rect_inflation_m values must be non-negative")
            object.__setattr__(self, "per_rect_inflation_m", per_rect)
        if self.occlusion_per_rect is not None:
            per_occlusion = tuple((str(direction), float(distance)) for direction, distance in self.occlusion_per_rect)
            for direction, distance in per_occlusion:
                if direction not in allowed:
                    raise ValueError(f"occlusion_per_rect direction must be one of {sorted(allowed)}")
                if distance < 0.0:
                    raise ValueError("occlusion_per_rect distances must be non-negative")
            object.__setattr__(self, "occlusion_per_rect", per_occlusion)
        object.__setattr__(self, "adapter_stats", dict(self.adapter_stats))
        diagnostic_masks = {
            str(name): np.asarray(mask, dtype=bool)
            for name, mask in dict(self.diagnostic_masks).items()
        }
        object.__setattr__(self, "diagnostic_masks", diagnostic_masks)

    @property
    def semantic_rect_count(self) -> int:
        return (
            len(self.occupied_rects)
            + len(self.target_rects)
            + len(self.unknown_rects)
            + len(self.occluded_rects)
        )
