#!/usr/bin/env python3
"""Top-level facade — active-mask snapshot → A* → reference → DLS rollout."""
from __future__ import annotations

import os
from typing import Any, Dict, Optional, Tuple

import numpy as np

from artifact_writer import ArtifactWriter
from capsule_collision import CapsuleCollision
from constants import X_TARGET_CONTACT
from control_params import ControlParams
from dls_controller import DLSController
from episode_result import EpisodeResult
from episode_runner import EpisodeRunner
from linear_arclength_reference import LinearArclengthReference
from mission_request import MissionRequest
from path_planning import PathPlanning
from plan_request import PlanRequest
from plan_result import PlanResult
from reference_params import ReferenceParams
from reference_switch_policy import ReferenceSwitchPolicy
from smooth_arclength_reference import SmoothArclengthReference
from smoothing_params import SmoothingParams


class ControlModule:
    """Run-once orchestration after map-update has produced active masks."""

    def __init__(self, output_root: Optional[str] = None) -> None:
        module_dir = os.path.dirname(os.path.abspath(__file__))
        project_root = os.path.abspath(os.path.join(module_dir, "..", "..", ".."))
        self.project_root = project_root
        self.output_root = output_root
        self.planner = PathPlanning()
        self.capsules = CapsuleCollision()
        self.switch_policy = ReferenceSwitchPolicy()

    def run(self, request: MissionRequest, out_dir: Optional[str] = None) -> Dict[str, Any]:
        active_snapshot = self._require_active_snapshot(request)
        clear_snapshot = request.clear_snapshot or active_snapshot
        run_dir = out_dir or self._case_dir(request.scenario, request.reference_backend, request.cost_mode)
        image_root = os.path.join(os.path.dirname(run_dir), "img")
        image_prefix = os.path.basename(os.path.normpath(run_dir))
        writer = ArtifactWriter(make_plots=request.make_plots)
        ref_params = ReferenceParams(
            dt=float(request.dt),
            v_ref=float(request.v_ref),
            terminal_hold_s=float(request.terminal_hold_s),
            t_accel=float(request.t_accel),
            t_decel=float(request.t_decel),
            scheduler_mode=str(request.scheduler_mode),
            gap_target_m=float(request.gap_target_m),
            terminal_gap_eps_m=float(request.terminal_gap_eps_m),
            k_gap_per_s=float(request.k_gap_per_s),
            sdot_max_factor=float(request.sdot_max_factor),
            sdot_floor_factor=float(request.sdot_floor_factor),
        )
        ctrl = DLSController(ControlParams())
        runner = EpisodeRunner(ctrl)

        clear_plan = self.planner.plan(PlanRequest(
            map_snapshot=clear_snapshot,
            start_xz=request.start_xz,
            goal_xz=request.goal_xz,
            goal_projection_factor=float(request.goal_projection_factor),
            cost_mode=str(request.cost_mode),
            manip_weight=float(request.manip_weight),
            manip_kappa=float(request.manip_kappa),
            manip_mu_safe=float(request.manip_mu_safe),
            manip_mu_safe_percentile=float(request.manip_mu_safe_percentile),
        ))
        summary: Dict[str, Any] = {
            "scenario": request.scenario,
            "reference_backend": request.reference_backend,
            "input_contract": "active_map_snapshot",
            "mu_min": float(active_snapshot.mu_min),
            "cost_mode": str(request.cost_mode),
            "manip_weight": float(request.manip_weight),
            "manip_kappa": float(request.manip_kappa),
            "manip_mu_safe": float(request.manip_mu_safe),
            "manip_mu_safe_percentile": float(request.manip_mu_safe_percentile),
            "dt": float(request.dt),
            "v_ref": float(request.v_ref),
            "terminal_hold_s": float(request.terminal_hold_s),
            "t_accel": float(request.t_accel),
            "t_decel": float(request.t_decel),
            "scheduler_mode": str(request.scheduler_mode),
            "gap_target_m": float(request.gap_target_m),
            "active_map_update_source": str(active_snapshot.source),
            "active_map_update_stats": dict(active_snapshot.stats),
            "dynamic_obstacle_scope": "controlled_slow_cube_or_box",
        }
        q0 = None
        if request.initial_q_active is not None:
            q0 = np.asarray(request.initial_q_active, dtype=np.float64).reshape(-1)
            if q0.shape[0] != 3 or not np.all(np.isfinite(q0)):
                raise ValueError("MissionRequest.initial_q_active must be a finite length-3 active joint vector")
            summary["initial_q_active_source"] = "MissionRequest.initial_q_active"
            summary["initial_q_active"] = [float(v) for v in q0.tolist()]
        else:
            summary["initial_q_active_source"] = "EpisodeRunner.default_q_home"

        if request.scenario == "clear":
            case_ref, case_ep = self._simulate(clear_plan, request.reference_backend, ref_params, runner, q0=q0)
            case_summary = self._case_summary("clear", clear_plan, case_ref, case_ep)
            summary["case"] = case_summary
            summary["clear"] = case_summary
            if request.write_artifacts:
                writer.write_case(run_dir, clear_plan, case_ref, case_ep, summary, image_root, image_prefix)
        elif request.scenario == "changed":
            clear_ref, _ = self._simulate(clear_plan, request.reference_backend, ref_params, runner)
            changed_plan = self.planner.plan(PlanRequest(
                map_snapshot=active_snapshot,
                start_xz=request.start_xz,
                goal_xz=request.goal_xz,
                goal_projection_factor=float(request.goal_projection_factor),
                cost_mode=str(request.cost_mode),
                manip_weight=float(request.manip_weight),
                manip_kappa=float(request.manip_kappa),
                manip_mu_safe=float(request.manip_mu_safe),
                manip_mu_safe_percentile=float(request.manip_mu_safe_percentile),
            ))
            changed_ref, changed_ep = self._simulate(changed_plan, request.reference_backend, ref_params, runner, q0=q0)
            changed_summary = self._case_summary("changed", changed_plan, changed_ref, changed_ep)
            changed_summary["clear_path_blocked_by_changed_obstacle"] = bool(
                self.planner.path_blocked(clear_plan.cells, changed_plan.blocked_mask)
            )
            try:
                ref_blocked, ref_blocked_count = self.switch_policy.remaining_reference_blocked(
                    changed_plan, clear_ref, changed_plan.blocked_mask
                )
                changed_summary["clear_reference_blocked_by_changed_obstacle"] = bool(ref_blocked)
                changed_summary["clear_reference_blocked_sample_count"] = int(ref_blocked_count)
            except Exception:
                changed_summary["clear_reference_blocked_by_changed_obstacle"] = False
                changed_summary["clear_reference_blocked_sample_count"] = 0
            summary["case"] = changed_summary
            summary["changed"] = changed_summary
            if request.write_artifacts:
                writer.write_case(run_dir, changed_plan, changed_ref, changed_ep, summary, image_root, image_prefix)
        else:
            raise ValueError(f"unknown scenario: {request.scenario}")
        summary["run_dir"] = run_dir
        return summary

    def _simulate(self,
                  plan: PlanResult,
                  backend: str,
                  ref_params: ReferenceParams,
                  runner: EpisodeRunner,
                  q0: Optional[np.ndarray] = None) -> Tuple[Any, Optional[EpisodeResult]]:
        if not plan.found or not plan.valid_grasp:
            # Empty path — synthesize a single-point linear reference so the
            # runner can still produce an EpisodeResult with metadata.
            ref = LinearArclengthReference(
                np.asarray([plan.start_xz], dtype=np.float64),
                y_ref=plan.map_handle.target_y,
                params=ref_params,
                handle=plan.map_handle,
                feasible_mask=plan.feasible_mask,
            )
            ref.metrics["reference_backend"] = backend
            ref.metrics["reference_generation_skipped"] = True
            ref.metrics["reference_skip_reason"] = "plan_not_found"
            return ref, None
        # Spec §10 — pitch is reconstructed from gaze geometry inside both
        # backends, so only xz waypoints + the map handle are passed in.
        if backend == "linear":
            ref = LinearArclengthReference(
                plan.xz,
                y_ref=plan.map_handle.target_y,
                params=ref_params,
                handle=plan.map_handle,
                feasible_mask=plan.feasible_mask,
            )
        elif backend == "c2_quintic":
            ref = SmoothArclengthReference(
                plan.cells, plan.xz,
                y_ref=plan.map_handle.target_y,
                handle=plan.map_handle,
                feasible_mask=plan.feasible_mask,
                reference_params=ref_params,
                smoothing_params=SmoothingParams(),
            )
        else:
            raise ValueError(f"unknown reference backend: {backend}")
        ep = runner.run(ref, q0=q0)
        return ref, ep

    def _case_summary(self,
                      label: str,
                      plan: PlanResult,
                      reference,
                      episode: Optional[EpisodeResult]) -> Dict[str, Any]:
        if episode is None:
            collision_metrics = {
                "capsule_proxy_collision_free": 0.0,
                "capsule_proxy_violation_count": 0.0,
                "capsule_proxy_checked_samples": 0.0,
            }
            accept_metrics = {
                "candidate_accepted": False,
                "reject_reason": self._plan_reject_reason(plan),
                "reject_diagnostic": plan.invalid_reason_code or plan.invalid_reason,
                "reject_detail": plan.invalid_reason,
            }
            control_summary = self._no_episode_summary(str(accept_metrics["reject_reason"]))
        else:
            collision_metrics = self.capsules.trajectory_metrics(episode.q_traj, plan.map_handle, plan.blocked_mask)
            accept_metrics = self.switch_policy.candidate_accepted(plan, reference, episode, collision_metrics)
            control_summary = self._episode_summary(episode)
        return {
            "label": label,
            "plan": self._plan_summary(plan),
            "reference": dict(getattr(reference, "metrics", {})),
            "control": control_summary,
            "capsule_proxy_validation": collision_metrics,
            "candidate_policy": accept_metrics,
        }

    @staticmethod
    def _plan_summary(plan: PlanResult) -> Dict[str, Any]:
        meta = plan.map_handle.meta
        return {
            "found": bool(plan.found),
            "map_path": plan.map_handle.map_path,
            "map_tag": plan.map_handle.tag,
            "map_resolution_m": float(plan.map_handle.resolution_m),
            "map_target_y": float(plan.map_handle.target_y),
            "map_target_height": float(meta.get("target_height", meta.get("target_z", np.nan))),
            "map_x_ee_goal": float(meta.get("x_opt_ee_goal_m", meta.get("optimal_x", np.nan))),
            "map_x_target_contact": float(meta.get("x_target_contact_m", meta.get("x_gaze", X_TARGET_CONTACT))),
            "valid_grasp": bool(plan.valid_grasp),
            "invalid_reason_code": plan.invalid_reason_code,
            "invalid_reason": plan.invalid_reason,
            "collision_free": bool(plan.collision_free),
            "path_length_m": float(plan.length_m),
            "g_cost": float(plan.g_cost),
            "nodes_expanded": int(plan.nodes_expanded),
            "planning_time_ms": float(plan.time_ms),
            "waypoint_count": int(len(plan.cells)),
            "start_cell": list(plan.start_cell),
            "goal_cell": list(plan.goal_cell),
            "start_requested_xz": list(plan.start_requested_xz),
            "goal_requested_xz": list(plan.goal_requested_xz),
            "start_xz": list(plan.start_xz),
            "goal_xz": list(plan.goal_xz),
            "start_projection_m": float(plan.start_projection_m),
            "goal_projection_m": float(plan.goal_projection_m),
            "projection_tolerance_m": float(plan.projection_tolerance_m),
            "blocked_rects": [list(r) for r in plan.blocked_rects],
            "feasibility_stats": plan.feasibility_stats,
            "map_update_stats": plan.map_update_stats,
            "path_mu_stats": plan.path_mu_stats,
        }

    @staticmethod
    def _episode_summary(ep: EpisodeResult) -> Dict[str, Any]:
        return {
            "success": bool(ep.success),
            "success_basic": bool(ep.success_basic),
            "success_strong": bool(ep.success_strong),
            "fail_reason": ep.fail_reason.name,
            "fail_detail": ep.fail_detail,
            "n_steps": int(ep.q_traj.shape[0]),
            "completion_time_s": float(ep.completion_time),
            "final_pos_err_m": float(ep.final_pos_err),
            "final_rot_err_rad": float(ep.final_rot_err),
            "final_rot_err_deg": float(np.degrees(ep.final_rot_err)),
            "L_q": float(ep.L_q),
            "L_q_j": ep.L_q_j.tolist(),
            "L_q_j_max": float(np.max(ep.L_q_j)) if ep.L_q_j.size else 0.0,
            "L_x_sim": float(ep.L_x_sim),
            "L_x_ref": float(ep.L_x_ref),
            "rho_sim": float(ep.rho_sim),
            "rho_ref": float(ep.rho_ref),
            "tracking_error_rms": float(ep.tracking_error_rms),
            "tracking_error_max": float(ep.tracking_error_max),
            "tracking_error_pos_rms": float(ep.tracking_error_pos_rms),
            "tracking_error_pos_max": float(ep.tracking_error_pos_max),
            "tracking_error_rot_rms": float(ep.tracking_error_rot_rms),
            "tracking_error_rot_max": float(ep.tracking_error_rot_max),
            "gap_max": float(ep.gap_max),
            "e_along_max": float(ep.e_along_max),
            "reference_progress_ratio": float(ep.reference_progress_ratio),
            "reference_progress_final_s": float(ep.reference_progress_final_s),
            "terminal_hold_started": bool(ep.terminal_hold_started),
            "peak_qdot": float(ep.peak_qdot),
            "peak_qdot_j": ep.peak_qdot_j.tolist() if ep.peak_qdot_j is not None else [],
            "peak_qdot_initial": float(ep.peak_qdot_initial),
            "peak_qdot_initial_tick": int(ep.peak_qdot_initial_tick),
            "peak_qdot_mid": float(ep.peak_qdot_mid),
            "peak_qdot_mid_tick": int(ep.peak_qdot_mid_tick),
            "peak_qdot_terminal": float(ep.peak_qdot_terminal),
            "peak_qdot_terminal_tick": int(ep.peak_qdot_terminal_tick),
            "min_sigma": float(ep.min_sigma),
            "lambda_active_ratio": float(ep.lambda_active_ratio),
            "sigma_hard_breach_ratio": float(ep.sigma_hard_breach_ratio),
        }

    @staticmethod
    def _plan_reject_reason(plan: PlanResult) -> str:
        if not plan.valid_grasp or not plan.found or not plan.collision_free:
            return "UNGRASPABLE"
        return "PLAN_INVALID"

    @staticmethod
    def _no_episode_summary(reason: str) -> Dict[str, Any]:
        return {
            "success": False,
            "success_basic": False,
            "success_strong": False,
            "fail_reason": str(reason),
            "fail_detail": "DLS rollout skipped because no valid plan/reference was accepted.",
            "n_steps": 0,
            "completion_time_s": None,
            "final_pos_err_m": None,
            "final_rot_err_rad": None,
            "final_rot_err_deg": None,
            "L_q": None,
            "L_q_j": [],
            "L_q_j_max": None,
            "L_x_sim": None,
            "L_x_ref": None,
            "rho_sim": None,
            "rho_ref": None,
            "tracking_error_rms": None,
            "tracking_error_max": None,
            "tracking_error_pos_rms": None,
            "tracking_error_pos_max": None,
            "tracking_error_rot_rms": None,
            "tracking_error_rot_max": None,
            "gap_max": None,
            "e_along_max": None,
            "reference_progress_ratio": 0.0,
            "reference_progress_final_s": 0.0,
            "terminal_hold_started": False,
            "peak_qdot": None,
            "peak_qdot_j": [],
            "peak_qdot_initial": None,
            "peak_qdot_initial_tick": 0,
            "peak_qdot_mid": None,
            "peak_qdot_mid_tick": 0,
            "peak_qdot_terminal": None,
            "peak_qdot_terminal_tick": 0,
            "min_sigma": None,
            "lambda_active_ratio": None,
            "sigma_hard_breach_ratio": None,
        }

    def _case_dir(self, scenario: str, backend: str, cost_mode: str) -> str:
        suffix = "c2" if backend == "c2_quintic" else backend
        name = f"{scenario}_{suffix}"
        root = self.output_root or os.path.join(self.project_root, "path", self._output_branch_name(cost_mode))
        return os.path.join(root, name)

    @staticmethod
    def _output_branch_name(cost_mode: str) -> str:
        mode = str(cost_mode).strip().lower()
        if mode == "distance":
            return "distance"
        if mode == "manipulability":
            return "manipulability"
        return mode or "unknown"

    @staticmethod
    def _require_active_snapshot(request: MissionRequest):
        if request.active_snapshot is None:
            raise ValueError("MissionRequest.active_snapshot is required; run map_update_layer before ControlModule.run")
        if request.scenario == "changed" and request.clear_snapshot is None:
            raise ValueError("changed scenario requires MissionRequest.clear_snapshot for blockage comparison")
        return request.active_snapshot
