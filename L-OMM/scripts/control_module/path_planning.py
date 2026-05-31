#!/usr/bin/env python3
"""Planner facade selecting the active-map A* edge policy."""
from __future__ import annotations

from typing import Optional

from distance_astar_planner import DistanceAStarPlanner
from map_update_layer.map_update_layer import MapUpdateLayer
from manipulability_astar_planner import ManipulabilityAStarPlanner
from plan_request import PlanRequest
from plan_result import PlanResult


class PathPlanning:
    """Select distance-only or manipulability-cost A* while sharing the downstream control stack."""

    def __init__(self, map_path: Optional[str] = None) -> None:
        self.distance = DistanceAStarPlanner(map_path=map_path)
        self.manipulability = ManipulabilityAStarPlanner(map_path=map_path)
        self.default_map_path = self.distance.default_map_path

    def plan(self, request: PlanRequest) -> PlanResult:
        mode = str(request.cost_mode).strip().lower()
        if mode == "distance":
            return self.distance.plan(request)
        if mode == "manipulability":
            return self.manipulability.plan(request)
        raise ValueError(f"unknown cost_mode: {request.cost_mode}")

    def load_map(self, map_path=None):
        return self.distance.load_map(map_path)

    def path_blocked(self, cells, mask) -> bool:
        return self.distance.path_blocked(cells, mask)

    def samples_blocked(self, handle, xz, mask):
        return MapUpdateLayer.samples_blocked(handle, xz, mask)
