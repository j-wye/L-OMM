#!/usr/bin/env python3
"""Planner output dataclass (path skeleton + feasibility bookkeeping)."""
from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List, Tuple

import numpy as np

from constants import Cell, Rect
from map_handle import MapHandle
from map_update_layer.active_map_snapshot import ActiveMapSnapshot


@dataclass
class PlanResult:
    found: bool
    cells: List[Cell]
    xz: np.ndarray
    pitch: np.ndarray
    length_m: float
    g_cost: float
    nodes_expanded: int
    time_ms: float
    start_cell: Cell
    goal_cell: Cell
    start_requested_xz: Tuple[float, float]
    goal_requested_xz: Tuple[float, float]
    start_xz: Tuple[float, float]
    goal_xz: Tuple[float, float]
    blocked_rects: List[Rect]
    blocked_mask: np.ndarray
    feasible_mask: np.ndarray
    collision_free: bool
    feasibility_stats: Dict[str, float]
    map_update_stats: Dict[str, float]
    map_snapshot: ActiveMapSnapshot
    path_mu_stats: Dict[str, float]
    map_handle: MapHandle
    start_projection_m: float = 0.0
    goal_projection_m: float = 0.0
    projection_tolerance_m: float = 0.0
    valid_grasp: bool = True
    invalid_reason_code: str = ""
    invalid_reason: str = ""
