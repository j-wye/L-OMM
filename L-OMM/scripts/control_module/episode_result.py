#!/usr/bin/env python3
"""Per-episode rollout result (full trajectories + summary metrics)."""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from fail_reason import FailReason


@dataclass
class EpisodeResult:
    success: bool
    mode: str
    q_traj: np.ndarray
    qdot_traj: np.ndarray
    dq_traj: np.ndarray
    ee_traj: np.ndarray
    e_pos_hist: np.ndarray
    e_rot_hist: np.ndarray
    sigma_min_hist: np.ndarray
    lambda_hist: np.ndarray
    time_scale_hist: np.ndarray
    wp_index: np.ndarray
    s_hist: np.ndarray
    r_dot_ref_hist: np.ndarray
    L_q: float
    L_q_j: np.ndarray
    L_x_sim: float
    L_x_ref: float
    rho_sim: float
    rho_ref: float
    tracking_error_rms: float
    tracking_error_max: float
    tracking_error_pos_rms: float = 0.0
    tracking_error_pos_max: float = 0.0
    tracking_error_rot_rms: float = 0.0
    tracking_error_rot_max: float = 0.0
    final_pos_err: float = 0.0
    final_rot_err: float = 0.0
    peak_qdot: float = 0.0
    peak_qdot_j: np.ndarray = None  # type: ignore[assignment]
    # Spec §12.3 — peak_qdot decomposition by trajectory phase so reviewers can
    # tell at a glance whether a high peak comes from the t=0 homing transient
    # (reference-independent) or from mid-trajectory tracking (reference-dependent).
    peak_qdot_initial: float = 0.0
    peak_qdot_initial_tick: int = 0
    peak_qdot_mid: float = 0.0
    peak_qdot_mid_tick: int = 0
    peak_qdot_terminal: float = 0.0
    peak_qdot_terminal_tick: int = 0
    sdot_hist: np.ndarray = None  # type: ignore[assignment]
    rdot_norm_hist: np.ndarray = None  # type: ignore[assignment]
    dpitch_ds_hist: np.ndarray = None  # type: ignore[assignment]
    gap_norm_hist: np.ndarray = None  # type: ignore[assignment]
    e_along_hist: np.ndarray = None  # type: ignore[assignment]
    gap_max: float = 0.0
    e_along_max: float = 0.0
    reference_progress_ratio: float = 0.0
    reference_progress_final_s: float = 0.0
    terminal_hold_started: bool = False
    min_sigma: float = 0.0
    lambda_active_ratio: float = 0.0
    sigma_hard_breach_ratio: float = 0.0
    completion_time: float = 0.0
    success_basic: bool = False
    success_strong: bool = False
    fail_reason: FailReason = FailReason.SUCCESS
    fail_detail: str = ""
