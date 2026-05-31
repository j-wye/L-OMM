#!/usr/bin/env python3
"""DLS controller hyperparameters (validated dataclass)."""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Optional

import numpy as np

from constants import JL_HI_ACTIVE, JL_LO_ACTIVE, N_JOINTS, TASK_DIM


@dataclass
class ControlParams:
    lam_min: float = 1.0e-2
    lam_max: float = 1.0e-1
    sigma_th: float = 5.0e-2
    sigma_hard: float = 1.0e-3
    Kp_pos: float = 4.0
    Kp_pitch: float = 2.5
    Kp_vec: Optional[np.ndarray] = None
    pitch_weight: float = 0.10
    qdot_max: np.ndarray = field(
        default_factory=lambda: np.array([1.5, 1.5, 2.0], dtype=np.float64)
    )
    dq_max_per_tick: float = 0.18
    jl_lo: np.ndarray = field(default_factory=lambda: JL_LO_ACTIVE.copy())
    jl_hi: np.ndarray = field(default_factory=lambda: JL_HI_ACTIVE.copy())

    # Tolerances (design.md §11)
    final_pos_tol: float = 2.0e-2          # success_basic
    final_rot_tol: float = math.radians(5.0)
    strong_pos_tol: float = 1.0e-2         # success_strong (diagnostic)
    strong_rot_tol: float = math.radians(3.0)
    pos_tol: float = 2.0e-3                # waypoint convergence test
    rot_tol: float = math.radians(1.0)

    jl_stuck_k: int = 25
    eps: float = 1.0e-9

    def __post_init__(self) -> None:
        self.qdot_max = np.asarray(self.qdot_max, dtype=np.float64).reshape(-1)
        self.jl_lo = np.asarray(self.jl_lo, dtype=np.float64).reshape(-1)
        self.jl_hi = np.asarray(self.jl_hi, dtype=np.float64).reshape(-1)
        if self.Kp_vec is None:
            self.Kp_vec = np.array(
                [self.Kp_pos, self.Kp_pos, self.Kp_pitch],
                dtype=np.float64,
            )
        else:
            self.Kp_vec = np.asarray(self.Kp_vec, dtype=np.float64).reshape(-1)
        if self.qdot_max.shape[0] != N_JOINTS or self.Kp_vec.shape[0] != TASK_DIM:
            raise ValueError("qdot_max and Kp_vec must match the reduced active model")
        if self.jl_lo.shape[0] != N_JOINTS or self.jl_hi.shape[0] != N_JOINTS:
            raise ValueError("joint limits must match the reduced active model")
        if np.any(self.qdot_max <= 0.0) or np.any(self.jl_lo >= self.jl_hi):
            raise ValueError("invalid joint-space limits")
        if self.lam_min <= 0.0 or self.lam_max < self.lam_min:
            raise ValueError("invalid damping range")
        if self.sigma_hard <= 0.0 or self.sigma_th < self.sigma_hard:
            raise ValueError("invalid singular-value thresholds")
        for tol in (self.final_pos_tol, self.final_rot_tol,
                    self.strong_pos_tol, self.strong_rot_tol,
                    self.pos_tol, self.rot_tol):
            if tol <= 0.0:
                raise ValueError("tolerances must be positive")
