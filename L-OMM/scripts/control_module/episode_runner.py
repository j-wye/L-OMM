#!/usr/bin/env python3
"""Closed-loop DLS rollout over an arclength reference."""
from __future__ import annotations

from typing import List, Optional

import numpy as np

from constants import N_JOINTS, Q_HOME_DEFAULT
from control_params import ControlParams
from dls_controller import DLSController
from episode_result import EpisodeResult
from fail_reason import FailReason


class EpisodeRunner:
    """Run a code-only kinematic closed-loop simulation and accumulate metrics."""

    def __init__(self, controller: Optional[DLSController] = None) -> None:
        self.controller = controller if controller is not None else DLSController()

    def run(self, reference, q0: Optional[np.ndarray] = None) -> EpisodeResult:
        params: ControlParams = self.controller.params
        q_home = getattr(reference, "q_home", Q_HOME_DEFAULT)
        q_cur = np.asarray(q_home if q0 is None else q0, dtype=np.float64).reshape(-1)
        if q_cur.shape[0] != N_JOINTS or not np.all(np.isfinite(q_cur)):
            return self._empty(FailReason.INVALID_INPUT, "invalid q0", getattr(reference, "length", 0.0))
        q_cur = np.clip(q_cur, params.jl_lo, params.jl_hi)

        q_hist: List[np.ndarray] = []
        qdot_hist: List[np.ndarray] = []
        dq_hist: List[np.ndarray] = []
        ee_hist: List[np.ndarray] = []
        ep_hist: List[np.ndarray] = []
        er_hist: List[np.ndarray] = []
        sig_hist: List[float] = []
        lam_hist: List[float] = []
        ts_hist: List[float] = []
        wp_hist: List[int] = []
        s_hist: List[float] = []
        rdot_hist: List[np.ndarray] = []
        sdot_hist: List[float] = []
        gap_hist: List[float] = []
        e_along_hist: List[float] = []

        fail_reason = FailReason.SUCCESS
        fail_detail = ""
        jl_stuck = 0
        reference_reached_end = False
        terminal_hold_started = False
        scheduler_mode = str(getattr(getattr(reference, "params", None), "scheduler_mode", "path_following"))
        n_hold = int(reference.n_hold_ticks())
        n_main_done = 0

        def current_ee(q_state: np.ndarray) -> np.ndarray:
            try:
                chain, _ = self.controller.kin.fk_and_jacobian(q_state)
                return chain[-1][:3, 3].copy()
            except Exception:
                return np.zeros(3, dtype=np.float64)

        def record_tick(sample, s_val: float, sdot_val: float, e_along: float, gap: float) -> bool:
            nonlocal q_cur, fail_reason, fail_detail, jl_stuck
            r_dot_t = sample.r_dot * sdot_val if sample.r_dot is not None else None
            step = self.controller.step(q_cur, sample.pos[[0, 2]], sample.pitch, r_dot_t, reference.dt)
            q_hist.append(step.q_new.copy())
            qdot_hist.append(step.qdot.copy())
            dq_hist.append(step.dq.copy())
            ee_hist.append(step.ee_pos.copy())
            ep_hist.append(step.e_pos.copy())
            er_hist.append(step.e_rot.copy())
            sig_hist.append(float(step.sigma_min))
            lam_hist.append(float(step.lambda_used))
            ts_hist.append(float(step.time_scale))
            wp_hist.append(int(sample.wp_index))
            s_hist.append(float(s_val))
            sdot_hist.append(float(sdot_val))
            gap_hist.append(float(gap))
            e_along_hist.append(float(e_along))
            rdot_hist.append(
                np.asarray(r_dot_t, dtype=np.float64).copy()
                if r_dot_t is not None else np.zeros(3, dtype=np.float64)
            )
            if not step.ok:
                fail_reason = step.fail_reason
                fail_detail = step.fail_detail
                return False
            if np.any(step.jl_saturation) and float(np.linalg.norm(step.dq)) < 1.0e-10:
                jl_stuck += 1
            else:
                jl_stuck = 0
            if jl_stuck >= params.jl_stuck_k:
                fail_reason = FailReason.JOINT_LIMIT_STUCK
                fail_detail = "persistent joint-limit saturation"
                return False
            q_cur = step.q_new.copy()
            return True

        if scheduler_mode == "path_following" and hasattr(reference, "make_scheduler"):
            sched = reference.make_scheduler()
            max_main = int(getattr(reference, "max_main_ticks", reference.n_main_ticks)())
            min_main = int(getattr(reference.params, "min_main_ticks", 5))
            terminal_gap = float(getattr(reference.params, "terminal_gap_eps_m", 0.01))
            ee_for_sched = current_ee(q_cur)
            for tick in range(max_main):
                s, sdot, e_along, gap = sched.update(ee_for_sched, reference.dt)
                sample = reference.evaluate(s)
                if not record_tick(sample, s, sdot, e_along, gap):
                    break
                n_main_done += 1
                ee_for_sched = current_ee(q_cur)
                if s >= float(reference.length) - 1.0e-9 and gap <= terminal_gap and tick + 1 >= min_main:
                    reference_reached_end = True
                    break
            if fail_reason == FailReason.SUCCESS and not reference_reached_end:
                fail_reason = FailReason.REFERENCE_PROGRESS_INCOMPLETE
                fail_detail = (
                    f"reference progress incomplete: s={float(s_hist[-1]) if s_hist else 0.0:.6f} "
                    f"/ L={float(reference.length):.6f}"
                )
        else:
            # Legacy time-envelope ablation mode.
            n_main = int(reference.n_main_ticks())
            has_s_table = hasattr(reference, "s_at_tick")
            has_sdot = hasattr(reference, "sdot_at_tick")
            v_ref_const = float(getattr(reference, "v_ref", 0.0))
            for tick in range(n_main):
                if has_s_table:
                    s = float(reference.s_at_tick(tick))
                else:
                    s = min(float(reference.length), tick * reference.v_ref * reference.dt)
                if has_sdot:
                    sdot = float(reference.sdot_at_tick(tick))
                else:
                    sdot = v_ref_const
                sample = reference.evaluate(s)
                gap = float(np.linalg.norm(sample.pos - current_ee(q_cur)))
                if not record_tick(sample, s, sdot, 0.0, gap):
                    break
                n_main_done += 1
            if fail_reason == FailReason.SUCCESS:
                reference_reached_end = bool(s_hist and s_hist[-1] >= float(reference.length) - 1.0e-9)

        if fail_reason == FailReason.SUCCESS and reference_reached_end:
            terminal_hold_started = True
            for _ in range(n_hold):
                sample = reference.evaluate(float(reference.length))
                gap = float(np.linalg.norm(sample.pos - current_ee(q_cur)))
                if not record_tick(sample, float(reference.length), 0.0, 0.0, gap):
                    break

        return self._pack(
            q_hist, qdot_hist, dq_hist, ee_hist, ep_hist, er_hist,
            sig_hist, lam_hist, ts_hist, wp_hist, s_hist, rdot_hist, sdot_hist,
            gap_hist, e_along_hist,
            float(reference.length), reference.dt,
            int(n_main_done),
            float(getattr(reference, "v_ref", 0.0)),
            float(getattr(getattr(reference, "params", None), "t_accel", 0.0) or 0.0),
            float(getattr(getattr(reference, "params", None), "t_decel", 0.0) or 0.0),
            fail_reason, fail_detail,
            terminal_hold_started,
        )

    def _pack(self,
              q_hist, qdot_hist, dq_hist, ee_hist, ep_hist, er_hist,
              sig_hist, lam_hist, ts_hist, wp_hist, s_hist, rdot_hist, sdot_hist,
              gap_hist, e_along_hist,
              L_x_ref: float,
              dt: float,
              n_main: int,
              v_ref: float,
              t_accel: float,
              t_decel: float,
              fail_reason: FailReason,
              fail_detail: str,
              terminal_hold_started: bool) -> EpisodeResult:
        if not q_hist:
            return self._empty(FailReason.INVALID_INPUT, "empty rollout", L_x_ref)
        q_arr = np.stack(q_hist, axis=0)
        qdot_arr = np.stack(qdot_hist, axis=0)
        dq_arr = np.stack(dq_hist, axis=0)
        ee_arr = np.stack(ee_hist, axis=0)
        ep_arr = np.stack(ep_hist, axis=0)
        er_arr = np.stack(er_hist, axis=0)
        sig_arr = np.asarray(sig_hist, dtype=np.float64)
        lam_arr = np.asarray(lam_hist, dtype=np.float64)
        ts_arr = np.asarray(ts_hist, dtype=np.float64)
        wp_arr = np.asarray(wp_hist, dtype=np.int64)
        s_arr = np.asarray(s_hist, dtype=np.float64)
        rdot_arr = np.stack(rdot_hist, axis=0)
        sdot_arr = np.asarray(sdot_hist, dtype=np.float64)
        gap_arr = np.asarray(gap_hist, dtype=np.float64)
        e_along_arr = np.asarray(e_along_hist, dtype=np.float64)

        params = self.controller.params
        finite = np.all(np.isfinite(q_arr)) and np.all(np.isfinite(qdot_arr)) and np.all(np.isfinite(ee_arr))
        if not finite:
            fail_reason = FailReason.NON_FINITE_STATE
            fail_detail = "non-finite rollout metric"

        final_pos = float(np.linalg.norm(ep_arr[-1]))
        final_rot = float(np.linalg.norm(er_arr[-1]))
        if fail_reason == FailReason.SUCCESS and final_pos > params.final_pos_tol:
            fail_reason = FailReason.POS_TOL_EXCEED
            fail_detail = f"final position residual {final_pos:.6f}"
        if fail_reason == FailReason.SUCCESS and final_rot > params.final_rot_tol:
            fail_reason = FailReason.ROT_TOL_EXCEED
            fail_detail = f"final rotation residual {final_rot:.6f}"

        # Spec §12.3 — peak_qdot decomposition by trajectory phase so a high
        # peak can be attributed to homing (reference-independent), corner /
        # cruise tracking (reference-dependent), or terminal hold.
        qdot_norm = np.linalg.norm(qdot_arr, axis=1)
        n_total = int(qdot_norm.size)
        n_init = max(1, min(n_total, int(np.ceil(max(t_accel, 0.0) / max(dt, 1.0e-12))) + 1))
        hold_ticks = max(0, n_total - n_main)
        decel_ticks = int(np.ceil(max(t_decel, 0.0) / max(dt, 1.0e-12)))
        n_term = max(0, min(n_total - n_init, hold_ticks + decel_ticks))
        init_slice = qdot_norm[:n_init]
        term_slice = qdot_norm[-n_term:] if n_term > 0 else np.zeros(0, dtype=np.float64)
        mid_lo = n_init
        mid_hi = max(mid_lo, n_total - n_term)
        mid_slice = qdot_norm[mid_lo:mid_hi]
        peak_init = float(init_slice.max()) if init_slice.size else 0.0
        peak_init_tick = int(init_slice.argmax()) if init_slice.size else 0
        peak_mid = float(mid_slice.max()) if mid_slice.size else 0.0
        peak_mid_tick = int(mid_lo + mid_slice.argmax()) if mid_slice.size else 0
        peak_term = float(term_slice.max()) if term_slice.size else 0.0
        peak_term_tick = int(n_total - n_term + term_slice.argmax()) if term_slice.size else 0

        rdot_norm = np.linalg.norm(rdot_arr, axis=1)

        L_q_j = np.sum(np.abs(dq_arr), axis=0)
        L_q = float(np.sum(np.linalg.norm(dq_arr, axis=1)))
        L_x_sim = float(np.sum(np.linalg.norm(np.diff(ee_arr, axis=0), axis=1))) if ee_arr.shape[0] > 1 else 0.0

        # Combined (weighted) tracking error — same convention as v1
        err = np.concatenate([ep_arr, params.pitch_weight * er_arr], axis=1)
        err_norm = np.linalg.norm(err, axis=1)
        # Split tracking error (design.md §11)
        pos_norm = np.linalg.norm(ep_arr, axis=1)
        rot_norm = np.linalg.norm(er_arr, axis=1)

        lam_thresh = params.lam_min * (1.0 + 1.0e-4)
        lam_active = int(np.sum(lam_arr > lam_thresh))
        sigma_breach = int(np.sum(sig_arr < params.sigma_hard))

        success_basic = bool(
            fail_reason == FailReason.SUCCESS
            and final_pos <= params.final_pos_tol
            and final_rot <= params.final_rot_tol
        )
        success_strong = bool(
            success_basic
            and final_pos <= params.strong_pos_tol
            and final_rot <= params.strong_rot_tol
        )

        return EpisodeResult(
            success=success_basic,
            mode="arclength",
            q_traj=q_arr,
            qdot_traj=qdot_arr,
            dq_traj=dq_arr,
            ee_traj=ee_arr,
            e_pos_hist=ep_arr,
            e_rot_hist=er_arr,
            sigma_min_hist=sig_arr,
            lambda_hist=lam_arr,
            time_scale_hist=ts_arr,
            wp_index=wp_arr,
            s_hist=s_arr,
            r_dot_ref_hist=rdot_arr,
            L_q=L_q,
            L_q_j=L_q_j,
            L_x_sim=L_x_sim,
            L_x_ref=float(L_x_ref),
            rho_sim=float(L_q / max(L_x_sim, params.eps)),
            rho_ref=float(L_q / max(float(L_x_ref), params.eps)),
            tracking_error_rms=float(np.sqrt(np.mean(err_norm * err_norm))),
            tracking_error_max=float(np.max(err_norm)),
            tracking_error_pos_rms=float(np.sqrt(np.mean(pos_norm * pos_norm))),
            tracking_error_pos_max=float(np.max(pos_norm)),
            tracking_error_rot_rms=float(np.sqrt(np.mean(rot_norm * rot_norm))),
            tracking_error_rot_max=float(np.max(rot_norm)),
            final_pos_err=final_pos,
            final_rot_err=final_rot,
            peak_qdot=float(qdot_norm.max()) if qdot_norm.size else 0.0,
            peak_qdot_j=np.max(np.abs(qdot_arr), axis=0),
            peak_qdot_initial=peak_init,
            peak_qdot_initial_tick=peak_init_tick,
            peak_qdot_mid=peak_mid,
            peak_qdot_mid_tick=peak_mid_tick,
            peak_qdot_terminal=peak_term,
            peak_qdot_terminal_tick=peak_term_tick,
            sdot_hist=sdot_arr,
            rdot_norm_hist=rdot_norm,
            dpitch_ds_hist=rdot_arr[:, 2].copy() if rdot_arr.size else np.zeros(0, dtype=np.float64),
            gap_norm_hist=gap_arr,
            e_along_hist=e_along_arr,
            gap_max=float(np.max(gap_arr)) if gap_arr.size else 0.0,
            e_along_max=float(np.max(e_along_arr)) if e_along_arr.size else 0.0,
            reference_progress_ratio=float(np.max(s_arr) / max(float(L_x_ref), params.eps)) if s_arr.size else 0.0,
            reference_progress_final_s=float(np.max(s_arr)) if s_arr.size else 0.0,
            terminal_hold_started=bool(terminal_hold_started),
            min_sigma=float(np.min(sig_arr)) if sig_arr.size else float("nan"),
            lambda_active_ratio=float(lam_active / max(lam_arr.size, 1)),
            sigma_hard_breach_ratio=float(sigma_breach / max(sig_arr.size, 1)),
            completion_time=float(q_arr.shape[0] * dt),
            success_basic=success_basic,
            success_strong=success_strong,
            fail_reason=fail_reason,
            fail_detail=fail_detail,
        )

    @staticmethod
    def _empty(reason: FailReason, detail: str, L_x_ref: float) -> EpisodeResult:
        return EpisodeResult(
            success=False,
            mode="arclength",
            q_traj=np.zeros((0, N_JOINTS)),
            qdot_traj=np.zeros((0, N_JOINTS)),
            dq_traj=np.zeros((0, N_JOINTS)),
            ee_traj=np.zeros((0, 3)),
            e_pos_hist=np.zeros((0, 3)),
            e_rot_hist=np.zeros((0, 2)),
            sigma_min_hist=np.zeros(0),
            lambda_hist=np.zeros(0),
            time_scale_hist=np.zeros(0),
            wp_index=np.zeros(0, dtype=np.int64),
            s_hist=np.zeros(0),
            r_dot_ref_hist=np.zeros((0, 3)),
            L_q=0.0,
            L_q_j=np.zeros(N_JOINTS),
            L_x_sim=0.0,
            L_x_ref=float(L_x_ref),
            rho_sim=float("nan"),
            rho_ref=float("nan"),
            tracking_error_rms=float("nan"),
            tracking_error_max=float("nan"),
            final_pos_err=float("nan"),
            final_rot_err=float("nan"),
            peak_qdot=0.0,
            peak_qdot_j=np.zeros(N_JOINTS),
            min_sigma=float("nan"),
            lambda_active_ratio=float("nan"),
            sigma_hard_breach_ratio=float("nan"),
            completion_time=0.0,
            peak_qdot_initial=0.0,
            peak_qdot_initial_tick=0,
            peak_qdot_mid=0.0,
            peak_qdot_mid_tick=0,
            peak_qdot_terminal=0.0,
            peak_qdot_terminal_tick=0,
            sdot_hist=np.zeros(0),
            rdot_norm_hist=np.zeros(0),
            dpitch_ds_hist=np.zeros(0),
            gap_norm_hist=np.zeros(0),
            e_along_hist=np.zeros(0),
            gap_max=0.0,
            e_along_max=0.0,
            reference_progress_ratio=0.0,
            reference_progress_final_s=0.0,
            terminal_hold_started=False,
            fail_reason=reason,
            fail_detail=detail,
        )
