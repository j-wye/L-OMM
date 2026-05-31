#!/usr/bin/env python3
"""Reference timing/progress parameters for arclength tracking."""
from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class ReferenceParams:
    dt: float = 0.01
    v_ref: float = 0.04
    terminal_hold_s: float = 1.0
    table_ds: float = 0.003
    # R2 — t-domain S-curve envelope (start/end ramps)
    t_accel: float = 0.30
    t_decel: float = 0.30
    scheduler_mode: str = "path_following"  # path_following | time_envelope
    gap_target_m: float = 0.01
    terminal_gap_eps_m: float = 0.01
    k_gap_per_s: float = 8.0
    sdot_max_factor: float = 1.25
    sdot_floor_factor: float = 0.05
    max_main_time_s: float = 60.0
    max_main_time_factor: float = 4.0
    min_main_ticks: int = 5

    def __post_init__(self) -> None:
        if self.dt <= 0.0 or self.v_ref <= 0.0 or self.table_ds <= 0.0:
            raise ValueError("dt, v_ref, and table_ds must be positive")
        if self.terminal_hold_s < 0.0:
            raise ValueError("terminal_hold_s must be non-negative")
        if self.t_accel < 0.0 or self.t_decel < 0.0:
            raise ValueError("t_accel/t_decel must be non-negative")
        if self.scheduler_mode not in {"path_following", "time_envelope"}:
            raise ValueError("scheduler_mode must be path_following or time_envelope")
        if self.gap_target_m < 0.0 or self.terminal_gap_eps_m < 0.0:
            raise ValueError("gap tolerances must be non-negative")
        if self.k_gap_per_s < 0.0:
            raise ValueError("k_gap_per_s must be non-negative")
        if self.sdot_max_factor <= 0.0 or self.sdot_floor_factor < 0.0:
            raise ValueError("sdot factors are invalid")
        if self.max_main_time_s <= 0.0 or self.max_main_time_factor <= 0.0:
            raise ValueError("max main time limits must be positive")
        if self.min_main_ticks < 1:
            raise ValueError("min_main_ticks must be >= 1")
