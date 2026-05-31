#!/usr/bin/env python3
"""Corridor-Preserving C² Active-Manifold Reference (xz quintic + gaze pitch).

Construction (per spec §3–§10):
  1. KeyframeExtractor reduces A* path → xz keyframes (direction-change + LOS).
  2. C2CornerBlender produces a quintic xz blend per interior keyframe with
     γ″(0)=γ″(1)=0 BC (C² in arclength).
  3. R1: blend radius monitored against a curvature *floor* (not enforced at
     nominal v_ref; flagged via metrics).
  4. R3: each blend is arclength-reparameterized via trapezoid integration
         and monotone interpolation τ(s).
  5. FeasibilityValidator + 4-tier graceful degradation (radius shrink →
     C⁰ corner fallback → C⁰ polyline reference fallback).
  6. Pitch reconstructed by GazePitch on the smoothed dense xz table.
  7. Default runtime scheduling is path-following with adaptive virtual time.
     TimeEnvelope remains as an explicit legacy ablation backend.
  8. Per-tick feedforward built by EpisodeRunner as ṙ_ref = (dγ/ds)·ṡ(t).
"""
from __future__ import annotations

import math
from typing import Dict, List, Tuple

import numpy as np

from blend_segment import BlendSegment
from c2_corner_blender import C2CornerBlender
from feasibility_validator import FeasibilityValidator
from gaze_pitch import GazePitch
from keyframe_extractor import KeyframeExtractor
from map_handle import MapHandle
from path_following_scheduler import PathFollowingScheduler
from reference_params import ReferenceParams
from reference_sample import ReferenceSample
from smoothing_params import SmoothingParams
from time_envelope import TimeEnvelope
from constants import Q_HOME_DEFAULT


class SmoothArclengthReference:
    """Dense arclength table with xz C² construction + gaze-deterministic pitch."""

    def __init__(self,
                 cells,
                 xz: np.ndarray,
                 *,
                 y_ref: float,
                 handle: MapHandle,
                 feasible_mask: np.ndarray,
                 reference_params: ReferenceParams,
                 smoothing_params: SmoothingParams | None = None) -> None:
        self.params = reference_params
        self.smoothing_params = smoothing_params if smoothing_params is not None else SmoothingParams()
        self.y_ref = float(y_ref)
        self._source_xz = np.asarray(xz, dtype=np.float64).reshape(-1, 2)
        if self._source_xz.shape[0] == 0:
            raise ValueError("source xz must be non-empty")

        # Spec §7.1 — sample spacing is min(map.resolution_m/2, params.validation_ds).
        self.validation_ds = float(min(0.5 * float(handle.resolution_m),
                                       float(self.smoothing_params.validation_ds)))
        if self.validation_ds <= 0.0:
            raise ValueError("validation_ds resolved to non-positive value")

        # Spec §10 — gaze geometry (atan2 reconstruction).
        self.gaze = GazePitch(handle.meta)
        self.q_home = np.asarray(handle.meta.get("q_home_reduced_seed", Q_HOME_DEFAULT), dtype=np.float64).reshape(3)

        extractor = KeyframeExtractor(self.smoothing_params)
        key_result = extractor.extract(cells, self._source_xz, feasible_mask)
        self.keyframe_indices = key_result.indices
        key_xz = self._source_xz[self.keyframe_indices]
        blender = C2CornerBlender(self.smoothing_params)
        validator = FeasibilityValidator(handle, feasible_mask)

        accepted: Dict[int, BlendSegment] = {}
        corner_count = 0
        accepted_count = 0
        radius_shrink_count = 0
        c0_fallback_corner_count = 0
        curv_lift_count = 0
        curv_overflow_count = 0

        for i in range(1, len(key_xz) - 1):
            radius, lift, overflow = blender.initial_radius(
                key_xz[i - 1], key_xz[i], key_xz[i + 1], v_ref=self.params.v_ref
            )
            if lift:
                curv_lift_count += 1
            if overflow:
                curv_overflow_count += 1
            if radius < self.smoothing_params.r_min:
                continue
            corner_count += 1
            chosen: BlendSegment | None = None
            r = radius
            local_shrink = 0
            while r >= self.smoothing_params.r_min:
                candidate = blender.corner_candidate(
                    key_xz[i - 1], key_xz[i], key_xz[i + 1],
                    corner_index=int(self.keyframe_indices[i]),
                    radius=r,
                    sample_ds=self.validation_ds,
                    curvature_lift=lift,
                    curvature_overflow=overflow,
                )
                if candidate is None:
                    break
                ok, _ = validator.samples_feasible(candidate.xz)
                if ok:
                    chosen = candidate
                    break
                r *= self.smoothing_params.radius_shrink
                local_shrink += 1
            radius_shrink_count += local_shrink
            if chosen is None:
                c0_fallback_corner_count += 1
            else:
                accepted[i] = chosen
                accepted_count += 1

        xz_table = self._assemble_xz_table(key_xz, accepted, blender)
        all_feasible, bad_count = validator.samples_feasible(xz_table)
        feasibility_sample_count = int(xz_table.shape[0])
        if not all_feasible:
            xz_table = self._polyline_xz_table(self._source_xz)
            all_feasible, bad_count = validator.samples_feasible(xz_table)
            feasibility_sample_count = int(xz_table.shape[0])
            c0_fallback_corner_count = max(c0_fallback_corner_count, corner_count)
            accepted_count = 0

        self.s_table, self.xz_table = self._finalize_xz_table(xz_table)
        # Spec §10 — reconstruct pitch on the smoothed dense xz table.
        self.pitch_table = self.gaze.reconstruct(self.xz_table)
        self.length = float(self.s_table[-1]) if self.s_table.size else 0.0
        self.rdot_table, dpitch_max = self._build_rdot_table()
        self.wp_table = self._nearest_source_wp(self.xz_table)
        source_length = self._path_length(self._source_xz)

        # R2 envelope
        self._envelope = TimeEnvelope(self.length, self.params.v_ref,
                                      self.params.t_accel, self.params.t_decel)

        # Spec §10.5 — pitch slope guard (runtime feedforward only).  The
        # raw pre-clip maximum is preserved as a metric so the user can tell
        # whether the limit is binding or just sitting comfortably under it.
        pitch_slope_pre_clip_max = float(np.max(np.abs(self.rdot_table[:, 2]))) if self.rdot_table.size else 0.0
        slope_violations = int(np.sum(np.abs(self.rdot_table[:, 2]) > self.smoothing_params.pitch_slope_limit))
        if slope_violations:
            self.rdot_table[:, 2] = np.clip(
                self.rdot_table[:, 2],
                -self.smoothing_params.pitch_slope_limit,
                self.smoothing_params.pitch_slope_limit,
            )

        # Spec §10 / §12 — diagnostic: how well does our atan2 match the stored grid?
        recon_metrics = self.gaze.reconstruction_error(
            handle.pitch_grid, handle.x0, handle.z0, handle.resolution_m,
            valid_mask=(handle.mu_grid > 0.0),
        )

        self.metrics = dict(key_result.metrics)
        self.metrics.update({
            "reference_backend": "c2_quintic",
            "corner_count": int(corner_count),
            "accepted_blend_count": int(accepted_count),
            "radius_shrink_count": int(radius_shrink_count),
            "c0_fallback_corner_count": int(c0_fallback_corner_count),
            "unsmoothed_corner_count": int(max(corner_count - accepted_count, 0)),
            "r_curv_floor_active_corner_count": int(curv_lift_count),
            "r_curv_overflow_corner_count": int(curv_overflow_count),
            "dense_table_sample_count": int(self.xz_table.shape[0]),
            "smoothed_reference_length": float(self.length),
            "reference_length_ratio": float(self.length / max(source_length, 1.0e-12)),
            "reference_samples_feasible": bool(all_feasible),
            "reference_infeasible_sample_count": int(bad_count),
            "feasibility_sample_count": int(feasibility_sample_count),
            "feasibility_violation_count": int(bad_count),
            "envelope_total_time_s": float(self._envelope.total_time),
            "envelope_t_accel_s": float(self._envelope.t_accel),
            "envelope_t_decel_s": float(self._envelope.t_decel),
            "scheduler_mode": str(self.params.scheduler_mode),
            "gap_target_m": float(self.params.gap_target_m),
            "k_gap_per_s": float(self.params.k_gap_per_s),
            "sdot_max_factor": float(self.params.sdot_max_factor),
            "sdot_floor_factor": float(self.params.sdot_floor_factor),
            "validation_ds_m": float(self.validation_ds),
            "rdot_table_unit_speed": True,
            "pitch_slope_max_rad_per_m": float(np.max(np.abs(self.rdot_table[:, 2]))) if self.rdot_table.size else 0.0,
            "pitch_slope_pre_clip_max_rad_per_m": float(pitch_slope_pre_clip_max),
            "pitch_slope_violation_count": int(slope_violations),
            "pitch_recon_max_err_rad": float(recon_metrics["pitch_recon_max_err_rad"]),
            "pitch_recon_rms_err_rad": float(recon_metrics["pitch_recon_rms_err_rad"]),
            "pitch_recon_p98_err_rad": float(recon_metrics["pitch_recon_p98_err_rad"]),
            "pitch_recon_sample_count": int(recon_metrics["pitch_recon_sample_count"]),
            "gaze_x_gaze_m": float(self.gaze.x_gaze),
            "gaze_target_height_m": float(self.gaze.target_height),
        })

    # ----- public reference interface ----------------------------------- #

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
        if self.xz_table.shape[0] == 0:
            raise RuntimeError("empty reference table")
        s_eval = float(np.clip(s, 0.0, self.length))
        if self.length <= 0.0 or self.xz_table.shape[0] == 1:
            pos = np.array([self.xz_table[0, 0], self.y_ref, self.xz_table[0, 1]], dtype=np.float64)
            return ReferenceSample(pos, float(self.pitch_table[0]), np.zeros(3, dtype=np.float64),
                                   int(self.wp_table[0]) if self.wp_table.size else 0)
        idx = int(np.searchsorted(self.s_table, s_eval, side="right") - 1)
        idx = max(0, min(idx, self.s_table.size - 2))
        s0 = float(self.s_table[idx])
        s1 = float(self.s_table[idx + 1])
        a = (s_eval - s0) / max(s1 - s0, 1.0e-12)
        xz = (1.0 - a) * self.xz_table[idx] + a * self.xz_table[idx + 1]
        pitch = self.gaze.reconstruct_scalar(float(xz[0]), float(xz[1]))
        if s_eval >= self.length - 1.0e-12:
            r_dot = np.zeros(3, dtype=np.float64)
        else:
            r_dot = (1.0 - a) * self.rdot_table[idx] + a * self.rdot_table[idx + 1]
        wp = int(round((1.0 - a) * self.wp_table[idx] + a * self.wp_table[idx + 1]))
        pos = np.array([xz[0], self.y_ref, xz[1]], dtype=np.float64)
        return ReferenceSample(pos, float(pitch), r_dot, wp)

    def table_arrays(self) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        return self.s_table.copy(), self.xz_table.copy(), self.pitch_table.copy()

    # ----- internals ---------------------------------------------------- #

    def _assemble_xz_table(self,
                           key_xz: np.ndarray,
                           accepted: Dict[int, BlendSegment],
                           blender: C2CornerBlender) -> np.ndarray:
        xz_parts: List[np.ndarray] = []
        for j in range(0, len(key_xz) - 1):
            start_xz = accepted[j].xz[-1] if j in accepted else key_xz[j]
            end_xz = accepted[j + 1].xz[0] if (j + 1) in accepted else key_xz[j + 1]
            line = blender.line_segment(start_xz, end_xz, sample_ds=self.validation_ds)
            self._append(xz_parts, line.xz)
            if (j + 1) in accepted:
                self._append(xz_parts, accepted[j + 1].xz)
        if not xz_parts:
            return key_xz.copy()
        return np.vstack(xz_parts)

    def _polyline_xz_table(self, xz: np.ndarray) -> np.ndarray:
        blender = C2CornerBlender(self.smoothing_params)
        xz_parts: List[np.ndarray] = []
        for i in range(xz.shape[0] - 1):
            seg = blender.line_segment(xz[i], xz[i + 1], sample_ds=self.validation_ds)
            self._append(xz_parts, seg.xz)
        if not xz_parts:
            return xz.copy()
        return np.vstack(xz_parts)

    @staticmethod
    def _append(parts: List[np.ndarray], xz: np.ndarray) -> None:
        if not parts:
            parts.append(xz)
            return
        parts.append(xz[1:] if xz.shape[0] > 1 else xz[:0])

    def _finalize_xz_table(self, xz: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        xz_arr = np.asarray(xz, dtype=np.float64).reshape(-1, 2)
        if xz_arr.shape[0] == 0:
            raise ValueError("empty reference table")
        keep = [0]
        for i in range(1, xz_arr.shape[0]):
            if np.linalg.norm(xz_arr[i] - xz_arr[keep[-1]]) > 1.0e-9:
                keep.append(i)
        xz_arr = xz_arr[keep]
        if xz_arr.shape[0] == 1:
            return np.zeros(1, dtype=np.float64), xz_arr
        seg = np.linalg.norm(np.diff(xz_arr, axis=0), axis=1)
        s = np.concatenate(([0.0], np.cumsum(seg)))
        return s, xz_arr

    def _build_rdot_table(self) -> Tuple[np.ndarray, float]:
        """Unit-speed tangent table dγ/ds (spec §8.4 + §9.4).

        EpisodeRunner multiplies by ṡ(t) from the time envelope so r(t) and
        ṙ(t) share the same motion law.  Pitch slope is from gradient of the
        atan2-reconstructed pitch_table, capped at runtime by §10.5 guard.
        """
        n = self.xz_table.shape[0]
        out = np.zeros((n, 3), dtype=np.float64)
        if n < 2 or self.length <= 0.0:
            return out, 0.0
        dx_ds = np.gradient(self.xz_table[:, 0], self.s_table, edge_order=1)
        dz_ds = np.gradient(self.xz_table[:, 1], self.s_table, edge_order=1)
        dp_ds = np.gradient(self.pitch_table, self.s_table, edge_order=1)
        out[:, 0] = dx_ds
        out[:, 1] = dz_ds
        out[:, 2] = dp_ds
        out[-1, :] = 0.0
        return out, float(np.max(np.abs(dp_ds)))

    def _nearest_source_wp(self, xz_table: np.ndarray) -> np.ndarray:
        out = np.zeros(xz_table.shape[0], dtype=np.int64)
        for i, p in enumerate(xz_table):
            d2 = np.sum((self._source_xz - p.reshape(1, 2)) ** 2, axis=1)
            out[i] = int(np.argmin(d2))
        return out

    @staticmethod
    def _path_length(xz: np.ndarray) -> float:
        arr = np.asarray(xz, dtype=np.float64).reshape(-1, 2)
        if arr.shape[0] < 2:
            return 0.0
        return float(np.sum(np.linalg.norm(np.diff(arr, axis=0), axis=1)))
