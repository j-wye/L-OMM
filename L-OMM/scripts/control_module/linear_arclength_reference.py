#!/usr/bin/env python3
"""Piecewise-linear arclength reference with gaze-deterministic pitch.

Ablation baseline (vs SmoothArclengthReference).  Same scheduler contract
(path-following default, time-envelope legacy) and gaze-deterministic pitch
(spec §10) — only the xz geometry differs (no quintic blending, no LOS
shortcut).  This keeps the ablation comparison apples-to-apples on pitch and
feedforward law, isolating the effect of xz C² smoothing.
"""
from __future__ import annotations

import math
from typing import Optional, Tuple

import numpy as np

from constants import Q_HOME_DEFAULT
from feasibility_validator import FeasibilityValidator
from gaze_pitch import GazePitch
from map_handle import MapHandle
from path_following_scheduler import PathFollowingScheduler
from reference_params import ReferenceParams
from reference_sample import ReferenceSample
from time_envelope import TimeEnvelope


class LinearArclengthReference:
    """Linear-in-xz arclength reference with atan2 pitch reconstruction."""

    def __init__(self,
                 xz: np.ndarray,
                 *,
                 y_ref: float,
                 params: ReferenceParams,
                 handle: Optional[MapHandle] = None,
                 feasible_mask: Optional[np.ndarray] = None,
                 gaze: Optional[GazePitch] = None) -> None:
        self.params = params
        self.y_ref = float(y_ref)
        self._xz = np.asarray(xz, dtype=np.float64).reshape(-1, 2)
        if self._xz.shape[0] == 0:
            raise ValueError("xz must be a non-empty (N, 2) array")
        if not np.all(np.isfinite(self._xz)):
            raise ValueError("reference contains non-finite values")

        # Spec §10 — pitch reconstructed deterministically from gaze geometry.
        if gaze is not None:
            self.gaze = gaze
        elif handle is not None:
            self.gaze = GazePitch(handle.meta)
        else:
            raise ValueError("LinearArclengthReference requires `handle` or `gaze` to reconstruct pitch")
        if handle is not None:
            self.q_home = np.asarray(handle.meta.get("q_home_reduced_seed", Q_HOME_DEFAULT), dtype=np.float64).reshape(3)
        else:
            self.q_home = np.asarray(Q_HOME_DEFAULT, dtype=np.float64)
        self._pitch = self.gaze.reconstruct(self._xz)

        if self._xz.shape[0] == 1:
            self.s_nodes = np.zeros(1, dtype=np.float64)
        else:
            seg = np.linalg.norm(np.diff(self._xz, axis=0), axis=1)
            self.s_nodes = np.concatenate(([0.0], np.cumsum(seg)))
        self.length = float(self.s_nodes[-1])

        self._envelope = TimeEnvelope(self.length, self.params.v_ref,
                                      self.params.t_accel, self.params.t_decel)

        recon_metrics: dict = {}
        if handle is not None:
            recon_metrics = self.gaze.reconstruction_error(
                handle.pitch_grid, handle.x0, handle.z0, handle.resolution_m,
                valid_mask=(handle.mu_grid > 0.0),
            )

        ref_feasible = True
        infeasible_count = 0
        feasible_sample_count = int(self._xz.shape[0])
        if handle is not None and feasible_mask is not None:
            dense_check = self._sample_polyline_for_validation(
                min(0.5 * float(handle.resolution_m), float(params.table_ds))
            )
            ref_feasible, infeasible_count = FeasibilityValidator(handle, feasible_mask).samples_feasible(dense_check)
            feasible_sample_count = int(dense_check.shape[0])

        self.metrics = {
            "reference_backend": "linear",
            "original_waypoint_count": int(self._xz.shape[0]),
            "keyframe_count": int(self._xz.shape[0]),
            "corner_count": 0,
            "accepted_blend_count": 0,
            "radius_shrink_count": 0,
            "c0_fallback_corner_count": 0,
            "unsmoothed_corner_count": 0,
            "r_curv_floor_active_corner_count": 0,
            "r_curv_overflow_corner_count": 0,
            "dense_table_sample_count": int(self._xz.shape[0]),
            "smoothed_reference_length": self.length,
            "reference_length_ratio": 1.0,
            "reference_samples_feasible": bool(ref_feasible),
            "reference_infeasible_sample_count": int(infeasible_count),
            "feasibility_sample_count": int(feasible_sample_count),
            "feasibility_violation_count": int(infeasible_count),
            "envelope_total_time_s": float(self._envelope.total_time),
            "envelope_t_accel_s": float(self._envelope.t_accel),
            "envelope_t_decel_s": float(self._envelope.t_decel),
            "scheduler_mode": str(self.params.scheduler_mode),
            "gap_target_m": float(self.params.gap_target_m),
            "k_gap_per_s": float(self.params.k_gap_per_s),
            "sdot_max_factor": float(self.params.sdot_max_factor),
            "sdot_floor_factor": float(self.params.sdot_floor_factor),
            "rdot_table_unit_speed": True,
            "pitch_recon_max_err_rad": float(recon_metrics.get("pitch_recon_max_err_rad", 0.0)),
            "pitch_recon_rms_err_rad": float(recon_metrics.get("pitch_recon_rms_err_rad", 0.0)),
            "pitch_recon_p98_err_rad": float(recon_metrics.get("pitch_recon_p98_err_rad", 0.0)),
            "pitch_recon_sample_count": int(recon_metrics.get("pitch_recon_sample_count", 0)),
            "gaze_x_gaze_m": float(self.gaze.x_gaze),
            "gaze_target_height_m": float(self.gaze.target_height),
        }

    @property
    def dt(self) -> float:
        return self.params.dt

    @property
    def v_ref(self) -> float:
        return self.params.v_ref

    @property
    def terminal_hold_s(self) -> float:
        return self.params.terminal_hold_s

    def n_main_ticks(self) -> int:
        if self.length <= 0.0:
            return 1
        return int(math.ceil(self._envelope.total_time / max(self.dt, 1.0e-12))) + 1

    def n_hold_ticks(self) -> int:
        return int(round(self.terminal_hold_s / self.dt))

    def total_time_s(self) -> float:
        return float((self.n_main_ticks() + self.n_hold_ticks()) * self.dt)

    def max_main_ticks(self) -> int:
        nominal = self._envelope.total_time if self.length > 0.0 else self.dt
        capped = min(float(self.params.max_main_time_s), max(nominal * float(self.params.max_main_time_factor), self.dt))
        return max(int(self.params.min_main_ticks), int(math.ceil(capped / max(self.dt, 1.0e-12))) + 1)

    def make_scheduler(self) -> PathFollowingScheduler:
        return PathFollowingScheduler(
            reference=self,
            v_ref=float(self.params.v_ref),
            gap_target_m=float(self.params.gap_target_m),
            k_gap_per_s=float(self.params.k_gap_per_s),
            sdot_max_factor=float(self.params.sdot_max_factor),
            sdot_floor_factor=float(self.params.sdot_floor_factor),
            accel_time_s=float(self.params.t_accel),
            decel_time_s=float(self.params.t_decel),
        )

    def s_at_tick(self, tick: int) -> float:
        if self.length <= 0.0:
            return 0.0
        n_main = self.n_main_ticks()
        if tick < n_main:
            return float(self._envelope.s_at_time(tick * self.dt))
        return float(self.length)

    def sdot_at_tick(self, tick: int) -> float:
        if self.length <= 0.0:
            return 0.0
        if tick < self.n_main_ticks():
            return float(self._envelope.sdot_at_time(tick * self.dt))
        return 0.0

    def evaluate(self, s: float) -> ReferenceSample:
        if self.length <= 0.0 or self._xz.shape[0] == 1:
            pos = np.array([self._xz[0, 0], self.y_ref, self._xz[0, 1]], dtype=np.float64)
            return ReferenceSample(pos, float(self._pitch[0]), np.zeros(3, dtype=np.float64), 0)
        s_eval = float(np.clip(s, 0.0, self.length))
        idx = int(np.searchsorted(self.s_nodes, s_eval, side="right") - 1)
        idx = max(0, min(idx, self._xz.shape[0] - 2))
        s0 = float(self.s_nodes[idx])
        s1 = float(self.s_nodes[idx + 1])
        denom = max(s1 - s0, 1.0e-12)
        a = (s_eval - s0) / denom
        xz = (1.0 - a) * self._xz[idx] + a * self._xz[idx + 1]
        # Spec §10 — pitch is the reconstructed value at the interpolated xz.
        pitch_reconstructed = self.gaze.reconstruct_scalar(float(xz[0]), float(xz[1]))
        if s_eval >= self.length - 1.0e-12:
            r_dot = np.zeros(3, dtype=np.float64)
            wp = self._xz.shape[0] - 1
        else:
            # Spec §9.4 — unit-speed tangent dγ/ds (no v_ref baked in).
            dxz_ds = (self._xz[idx + 1] - self._xz[idx]) / denom
            dpitch_ds = float((self._pitch[idx + 1] - self._pitch[idx]) / denom)
            r_dot = np.array([dxz_ds[0], dxz_ds[1], dpitch_ds], dtype=np.float64)
            wp = idx + 1
        pos = np.array([xz[0], self.y_ref, xz[1]], dtype=np.float64)
        return ReferenceSample(pos, float(pitch_reconstructed), r_dot, int(wp))

    def table_arrays(self) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        return self.s_nodes.copy(), self._xz.copy(), self._pitch.copy()

    def _sample_polyline_for_validation(self, sample_ds: float) -> np.ndarray:
        if self._xz.shape[0] <= 1:
            return self._xz.copy()
        parts = []
        for i in range(self._xz.shape[0] - 1):
            p0 = self._xz[i]
            p1 = self._xz[i + 1]
            length = float(np.linalg.norm(p1 - p0))
            n = max(2, int(math.ceil(length / max(sample_ds, 1.0e-4))) + 1)
            t = np.linspace(0.0, 1.0, n, dtype=np.float64)
            seg = (1.0 - t[:, None]) * p0 + t[:, None] * p1
            parts.append(seg if not parts else seg[1:])
        return np.vstack(parts)
