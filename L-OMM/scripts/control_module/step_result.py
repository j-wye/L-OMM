#!/usr/bin/env python3
"""Single-tick DLS controller output."""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from fail_reason import FailReason


@dataclass
class StepResult:
    q_new: np.ndarray
    qdot: np.ndarray
    dq: np.ndarray
    ee_pos: np.ndarray
    e_pos: np.ndarray
    e_rot: np.ndarray
    sigma_min: float
    lambda_used: float
    time_scale: float
    jl_saturation: np.ndarray
    qdot_saturation: np.ndarray
    ok: bool
    fail_reason: FailReason = FailReason.SUCCESS
    fail_detail: str = ""
