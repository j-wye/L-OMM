#!/usr/bin/env python3
"""Synthetic geometry cases for map-update layer validation."""
from __future__ import annotations

from typing import Dict, Mapping

try:
    from control_module.constants import DEFAULT_MAP_UPDATE_HZ, DEFAULT_OBSTACLE, DEFAULT_OBSTACLE_INFLATION_M, Rect
except ImportError:
    from constants import DEFAULT_MAP_UPDATE_HZ, DEFAULT_OBSTACLE, DEFAULT_OBSTACLE_INFLATION_M, Rect

from .map_update_request import MapUpdateRequest


class SyntheticMapUpdateCases:
    """Factory for camera-free map-update validation requests."""

    def __init__(
        self,
        *,
        inflation_m: float = DEFAULT_OBSTACLE_INFLATION_M,
        map_update_hz: float = DEFAULT_MAP_UPDATE_HZ,
    ) -> None:
        self.inflation_m = float(inflation_m)
        self.map_update_hz = float(map_update_hz)

    def clear(self) -> MapUpdateRequest:
        return self._request(source="synthetic_clear")

    def occupied(self, rect: Rect = DEFAULT_OBSTACLE) -> MapUpdateRequest:
        return self._request(source="synthetic_occupied", occupied_rects=(rect,))

    def target(self, rect: Rect = DEFAULT_OBSTACLE) -> MapUpdateRequest:
        return self._request(source="synthetic_target", target_rects=(rect,))

    def unknown(self, rect: Rect = DEFAULT_OBSTACLE) -> MapUpdateRequest:
        return self._request(source="synthetic_unknown", unknown_rects=(rect,))

    def occluded(self, rect: Rect = DEFAULT_OBSTACLE, *, shadow_m: float = 0.12, direction: str = "+x") -> MapUpdateRequest:
        return self._request(
            source="synthetic_occluded",
            occluded_rects=(rect,),
            occlusion_shadow_m=shadow_m,
            occlusion_direction=direction,
        )

    def inflation_probe(self, rect: Rect = DEFAULT_OBSTACLE, *, inflation_m: float = 0.02) -> MapUpdateRequest:
        return self._request(source="synthetic_inflation", occupied_rects=(rect,), inflation_m=inflation_m)

    def all_cases(self) -> Mapping[str, MapUpdateRequest]:
        return {
            "clear": self.clear(),
            "occupied": self.occupied(),
            "target": self.target(),
            "unknown": self.unknown(),
            "occluded": self.occluded(),
            "inflation": self.inflation_probe(),
        }

    def _request(self, source: str, **kwargs) -> MapUpdateRequest:
        params = {
            "inflation_m": self.inflation_m,
            "map_update_hz": self.map_update_hz,
            "source": source,
        }
        params.update(kwargs)
        return MapUpdateRequest(**params)
