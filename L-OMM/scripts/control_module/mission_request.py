#!/usr/bin/env python3
"""Top-level control-module request.

The map-update layer must already have converted semantic obstacle inputs into
active-map snapshots before this request reaches ControlModule.run().
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
    DEFAULT_TRACKING_GAP_M,
)
from map_update_layer.active_map_snapshot import ActiveMapSnapshot


@dataclass
class MissionRequest:
    scenario: str = "clear"
    active_snapshot: Optional[ActiveMapSnapshot] = None
    clear_snapshot: Optional[ActiveMapSnapshot] = None
    start_xz: Optional[Tuple[float, float]] = None
    goal_xz: Optional[Tuple[float, float]] = None
    initial_q_active: Optional[Tuple[float, float, float]] = None
    cost_mode: str = DEFAULT_COST_MODE          # "distance" | "manipulability"
    manip_weight: float = DEFAULT_MANIP_WEIGHT
    manip_kappa: float = DEFAULT_MANIP_KAPPA
    manip_mu_safe: float = DEFAULT_MANIP_MU_SAFE
    manip_mu_safe_percentile: float = DEFAULT_MANIP_MU_SAFE_PERCENTILE
    reference_backend: str = "c2_quintic"   # "linear" | "c2_quintic"
    scheduler_mode: str = "path_following"  # "path_following" | "time_envelope"
    dt: float = 0.01
    v_ref: float = 0.04
    terminal_hold_s: float = 1.0
    t_accel: float = 0.30
    t_decel: float = 0.30
    gap_target_m: float = DEFAULT_TRACKING_GAP_M
    terminal_gap_eps_m: float = DEFAULT_TRACKING_GAP_M
    k_gap_per_s: float = 8.0
    sdot_max_factor: float = 1.25
    sdot_floor_factor: float = 0.05
    goal_projection_factor: float = DEFAULT_GOAL_PROJECTION_FACTOR
    make_plots: bool = True
    write_artifacts: bool = True
