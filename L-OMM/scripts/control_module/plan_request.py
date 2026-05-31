#!/usr/bin/env python3
"""Planner request dataclass.

PathPlanning consumes a final active-map snapshot, not raw obstacle semantics.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Tuple

from constants import (
    DEFAULT_COST_MODE,
    DEFAULT_GOAL_PROJECTION_FACTOR,
    DEFAULT_MANIP_KAPPA,
    DEFAULT_MANIP_MU_SAFE,
    DEFAULT_MANIP_MU_SAFE_PERCENTILE,
    DEFAULT_MANIP_WEIGHT,
)
from map_update_layer.active_map_snapshot import ActiveMapSnapshot


@dataclass(frozen=True)
class PlanRequest:
    map_snapshot: ActiveMapSnapshot
    start_xz: Optional[Tuple[float, float]] = None
    goal_xz: Optional[Tuple[float, float]] = None
    goal_projection_factor: float = DEFAULT_GOAL_PROJECTION_FACTOR
    cost_mode: str = DEFAULT_COST_MODE  # "distance" | "manipulability"
    manip_weight: float = DEFAULT_MANIP_WEIGHT
    manip_kappa: float = DEFAULT_MANIP_KAPPA
    manip_mu_safe: float = DEFAULT_MANIP_MU_SAFE
    manip_mu_safe_percentile: float = DEFAULT_MANIP_MU_SAFE_PERCENTILE
