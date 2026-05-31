#!/usr/bin/env python3
"""Sigma-adaptive DLS one-tick servo for the reduced active task."""
from __future__ import annotations

import math
from typing import Optional

import numpy as np

from constants import N_JOINTS, TASK_DIM
from control_params import ControlParams
from fail_reason import FailReason
from kinematics import Kinematics
from step_result import StepResult


class DLSController:
    """Damped least-squares controller on (x, z, gaze pitch)."""

    def __init__(self, params: Optional[ControlParams] = None) -> None:
        self.params = params if params is not None else ControlParams()
        self.kin = Kinematics()
        self._I = np.eye(TASK_DIM, dtype=np.float64)

    def step(self,
             q: np.ndarray,
             xz_ref: np.ndarray,
             pitch_ref: float,
             r_dot_ref: Optional[np.ndarray],
             dt: float) -> StepResult:
        try:
            q_cur = np.asarray(q, dtype=np.float64).reshape(-1)
            xz = np.asarray(xz_ref, dtype=np.float64).reshape(-1)
            rdot = np.zeros(TASK_DIM, dtype=np.float64) if r_dot_ref is None else np.asarray(r_dot_ref, dtype=np.float64).reshape(-1)
            dt_f = float(dt)
            if q_cur.shape[0] != N_JOINTS or xz.shape[0] != 2 or rdot.shape[0] != TASK_DIM:
                raise ValueError("invalid vector size")
            if not (np.all(np.isfinite(q_cur)) and np.all(np.isfinite(xz)) and np.all(np.isfinite(rdot))):
                raise ValueError("non-finite vector input")
            if not (math.isfinite(float(pitch_ref)) and math.isfinite(dt_f) and dt_f > 0.0):
                raise ValueError("invalid scalar input")
        except Exception as exc:
            return self._invalid_step(f"{type(exc).__name__}: {exc}")

        p = self.params
        q_clip = np.clip(q_cur, p.jl_lo, p.jl_hi)
        try:
            chain, jac = self.kin.fk_and_jacobian(q_clip)
            t_ee = chain[-1]
            e, _, _ = self.kin.task_error(t_ee, xz, float(pitch_ref))
            ee_pos = t_ee[:3, 3].copy()
            sigma_min = self._sigma_min(jac)
            lam_sq = self._adaptive_lambda_sq(sigma_min)
            u = rdot + p.Kp_vec * e
            a_mat = jac @ jac.T + lam_sq * self._I
            try:
                y = np.linalg.solve(a_mat, u)
            except np.linalg.LinAlgError:
                y = np.linalg.pinv(a_mat) @ u
            qdot_cmd = jac.T @ y
            qdot = np.clip(qdot_cmd, -p.qdot_max, p.qdot_max)
            qdot_sat = np.abs(qdot_cmd) >= (p.qdot_max - p.eps)
            dq = qdot * dt_f
            dq_inf = float(np.max(np.abs(dq))) if dq.size else 0.0
            if dq_inf > p.dq_max_per_tick:
                time_scale = p.dq_max_per_tick / max(dq_inf, p.eps)
                qdot = qdot * time_scale
                dq = qdot * dt_f
            else:
                time_scale = 1.0
            q_raw = q_clip + dq
            q_new = np.clip(q_raw, p.jl_lo, p.jl_hi)
            eps_pos = 1.0e-7
            overshoot = (q_raw < p.jl_lo - p.eps) | (q_raw > p.jl_hi + p.eps)
            at_low = (q_new <= p.jl_lo + eps_pos) & (qdot < -p.eps)
            at_high = (q_new >= p.jl_hi - eps_pos) & (qdot > p.eps)
            jl_sat = overshoot | at_low | at_high
            if not np.all(np.isfinite(q_new)):
                return self._nonfinite_step(e, ee_pos, sigma_min, math.sqrt(lam_sq), "q_new non-finite")
            return StepResult(
                q_new=q_new,
                qdot=qdot,
                dq=dq,
                ee_pos=ee_pos,
                e_pos=e[:2].copy(),
                e_rot=np.array([e[2]], dtype=np.float64),
                sigma_min=sigma_min,
                lambda_used=math.sqrt(lam_sq),
                time_scale=float(time_scale),
                jl_saturation=jl_sat,
                qdot_saturation=qdot_sat,
                ok=True,
            )
        except Exception as exc:
            return self._invalid_step(f"{type(exc).__name__}: {exc}")

    def _adaptive_lambda_sq(self, sigma_min: float) -> float:
        p = self.params
        if sigma_min >= p.sigma_th:
            return p.lam_min * p.lam_min
        ramp = float(np.clip(1.0 - sigma_min / p.sigma_th, 0.0, 1.0))
        return p.lam_min * p.lam_min + (p.lam_max * p.lam_max - p.lam_min * p.lam_min) * ramp * ramp

    @staticmethod
    def _sigma_min(jac: np.ndarray) -> float:
        try:
            svals = np.linalg.svd(jac, compute_uv=False)
            return float(svals[-1]) if svals.size else 0.0
        except np.linalg.LinAlgError:
            return 0.0

    @staticmethod
    def _invalid_step(detail: str) -> StepResult:
        return StepResult(
            q_new=np.full(N_JOINTS, np.nan),
            qdot=np.zeros(N_JOINTS),
            dq=np.zeros(N_JOINTS),
            ee_pos=np.zeros(3),
            e_pos=np.zeros(2),
            e_rot=np.zeros(1),
            sigma_min=0.0,
            lambda_used=0.0,
            time_scale=0.0,
            jl_saturation=np.zeros(N_JOINTS, dtype=bool),
            qdot_saturation=np.zeros(N_JOINTS, dtype=bool),
            ok=False,
            fail_reason=FailReason.INVALID_INPUT,
            fail_detail=detail,
        )

    @staticmethod
    def _nonfinite_step(e: np.ndarray,
                        ee_pos: np.ndarray,
                        sigma_min: float,
                        lam: float,
                        detail: str) -> StepResult:
        return StepResult(
            q_new=np.full(N_JOINTS, np.nan),
            qdot=np.zeros(N_JOINTS),
            dq=np.zeros(N_JOINTS),
            ee_pos=ee_pos,
            e_pos=e[:2].copy(),
            e_rot=np.array([e[2]], dtype=np.float64),
            sigma_min=sigma_min,
            lambda_used=lam,
            time_scale=0.0,
            jl_saturation=np.zeros(N_JOINTS, dtype=bool),
            qdot_saturation=np.zeros(N_JOINTS, dtype=bool),
            ok=False,
            fail_reason=FailReason.NON_FINITE_STATE,
            fail_detail=detail,
        )
