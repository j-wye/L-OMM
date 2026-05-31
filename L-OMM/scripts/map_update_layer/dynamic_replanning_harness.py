#!/usr/bin/env python3
"""Synthetic dynamic map-update and replanning dry-run harness."""
from __future__ import annotations

import csv
import json
import os
from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Tuple

import numpy as np

from control_module.constants import (
    DEFAULT_COST_MODE,
    DEFAULT_MANIP_KAPPA,
    DEFAULT_MANIP_MU_SAFE,
    DEFAULT_MANIP_MU_SAFE_PERCENTILE,
    DEFAULT_MANIP_WEIGHT,
    DEFAULT_MU_MIN,
    DEFAULT_OBSTACLE_INFLATION_M,
)
from control_module.capsule_collision import CapsuleCollision
from control_module.control_module import ControlModule
from control_module.mission_request import MissionRequest
from control_module.path_planning import PathPlanning

from .active_map_snapshot import ActiveMapSnapshot
from .blockage_classifier import ReferenceBlockageClassifier
from .dynamic_frame import SyntheticDynamicFrame, SyntheticDynamicScenario
from .lightweight_supervisor import LightweightSupervisorDryRun
from .map_update_layer import MapUpdateLayer
from .path_collision_monitor import PathCollisionMonitor
from .perception_to_map import CameraIntrinsics, nominal_t_base_cam
from .reference_snapshot_diff import DiffReport, ReferenceSnapshotDiff
from .recovery_contract import RecoveryHandoffContract
from .runtime_integration_checklist import JetsonRuntimeIntegrationChecklist
from .snapshot_validator import SnapshotValidator
from .synthetic_dynamic_sequences import SyntheticDynamicSequences


@dataclass
class ReferenceTrace:
    """Dense reference table used by the dry-run classifier."""

    s: np.ndarray
    xz: np.ndarray
    pitch: np.ndarray

    @property
    def length(self) -> float:
        return float(self.s[-1]) if self.s.size else 0.0


class DynamicReplanningHarness:
    """Run synthetic dynamic snapshot checks without camera or BT integration."""

    def __init__(
        self,
        *,
        output_root: str,
        map_path: Optional[str] = None,
        mu_min: float = DEFAULT_MU_MIN,
        inflation_m: float = DEFAULT_OBSTACLE_INFLATION_M,
        cost_mode: str = DEFAULT_COST_MODE,
        manip_weight: float = DEFAULT_MANIP_WEIGHT,
        manip_kappa: float = DEFAULT_MANIP_KAPPA,
        reference_backend: str = "c2_quintic",
        dt: float = 0.01,
        v_ref: float = 0.04,
        terminal_hold_s: float = 1.0,
        make_plots: bool = False,
        use_reference_diff: bool = False,
        reference_diff_use_task_fov: bool = False,
        reference_diff_pixel_band_px: float = 100.0,
        reference_diff_tau_diff_cells: int = 3,
        reference_diff_tau_cluster_cells: int = 5,
        include_p2_validation_scenarios: bool = False,
    ) -> None:
        self.output_root = os.path.abspath(output_root)
        self.map_path = map_path
        self.mu_min = float(mu_min)
        self.inflation_m = float(inflation_m)
        self.cost_mode = str(cost_mode)
        self.manip_weight = float(manip_weight)
        self.manip_kappa = float(manip_kappa)
        self.reference_backend = str(reference_backend)
        self.dt = float(dt)
        self.v_ref = float(v_ref)
        self.terminal_hold_s = float(terminal_hold_s)
        self.make_plots = bool(make_plots)
        self.reference_diff_use_task_fov = bool(reference_diff_use_task_fov)
        self.include_p2_validation_scenarios = bool(include_p2_validation_scenarios)

        self.planner = PathPlanning(map_path=map_path)
        self.handle = self.planner.load_map(map_path)
        d_max_task = max(
            0.2,
            abs(float(self.handle.x0)) + float(self.handle.shape[0]) * float(self.handle.resolution_m),
        )
        self.reference_T_base_cam = nominal_t_base_cam(float(self.handle.target_y))
        self.reference_diff = (
            ReferenceSnapshotDiff(
                handle=self.handle,
                intrinsics=CameraIntrinsics(
                    fx=615.0,
                    fy=615.0,
                    cx=319.5,
                    cy=239.5,
                    width=640,
                    height=480,
                ),
                y_plane=float(self.handle.target_y),
                d_min=0.05,
                d_max_task=d_max_task,
                pixel_band_half_width=float(reference_diff_pixel_band_px),
                tau_diff_cells=int(reference_diff_tau_diff_cells),
                tau_cluster_cells=int(reference_diff_tau_cluster_cells),
            )
            if use_reference_diff
            else None
        )
        self.updater = MapUpdateLayer()
        self.validator = SnapshotValidator()
        self.path_collision_monitor = PathCollisionMonitor(CapsuleCollision(), handle=self.handle)
        self.classifier = ReferenceBlockageClassifier(path_collision_monitor=self.path_collision_monitor)
        self.supervisor = LightweightSupervisorDryRun()
        self.recovery = RecoveryHandoffContract()
        self.control = ControlModule(output_root=self.output_root)
        self.sequence_factory = SyntheticDynamicSequences(inflation_m=self.inflation_m)

    def run_all(self, scenario_names: Optional[List[str]] = None) -> Dict[str, Any]:
        os.makedirs(self.output_root, exist_ok=True)
        all_scenarios = self.sequence_factory.all_scenarios(
            include_reference_diff_scenarios=self.reference_diff is not None,
            include_p2_validation_scenarios=self.include_p2_validation_scenarios,
        )
        names = scenario_names or list(all_scenarios.keys())
        summary: Dict[str, Any] = {
            "mode": "synthetic_dynamic_replanning_dry_run",
            "explicitly_no_live_camera": True,
            "explicitly_no_ros2_topic": True,
            "explicitly_no_full_bt_supervisor": True,
            "explicitly_no_mobile_base_controller": True,
            "explicitly_no_physical_robot_execution": True,
            "same_snapshot_infinite_retry_disabled": True,
            "universal_same_snapshot_bypass_not_assumed": True,
            "map_path": self.handle.map_path,
            "map_tag": self.handle.tag,
            "mu_min": self.mu_min,
            "inflation_m": self.inflation_m,
            "cost_mode": self.cost_mode,
            "manip_weight": self.manip_weight,
            "manip_kappa": self.manip_kappa,
            "reference_backend": self.reference_backend,
            "reference_diff_enabled": self.reference_diff is not None,
            "reference_diff_use_task_fov": bool(self.reference_diff_use_task_fov),
            "p2_validation_scenarios_enabled": bool(self.include_p2_validation_scenarios),
            "integration_checklist": JetsonRuntimeIntegrationChecklist().as_dict(),
            "scenarios": {},
        }
        for name in names:
            if name not in all_scenarios:
                raise ValueError(f"unknown synthetic dynamic scenario: {name}")
            scenario_summary = self.run_scenario(all_scenarios[name])
            summary["scenarios"][name] = scenario_summary
        sequence_checks = [
            s.get("reference_diff_sequence_checks", {})
            for s in summary["scenarios"].values()
            if s.get("reference_diff_sequence_checks", {}).get("enabled")
        ]
        summary["reference_diff_sequence_check_count"] = int(
            sum(len(c.get("checks", [])) for c in sequence_checks)
        )
        summary["reference_diff_sequence_all_pass"] = bool(
            all(bool(c.get("pass", False)) for c in sequence_checks)
        ) if sequence_checks else True
        self._write_json(os.path.join(self.output_root, "dynamic_replanning_summary.json"), summary)
        return summary

    def run_scenario(self, scenario: SyntheticDynamicScenario) -> Dict[str, Any]:
        scenario_dir = os.path.join(self.output_root, scenario.name)
        os.makedirs(scenario_dir, exist_ok=True)

        clear_snapshot = self.updater.build(
            self.handle,
            mu_min=self.mu_min,
            request=scenario.initial_request,
        )
        self.validator.assert_valid(clear_snapshot)

        initial_dir = os.path.join(scenario_dir, "initial_clear")
        initial_summary = self._run_control(
            active_snapshot=clear_snapshot,
            clear_snapshot=clear_snapshot,
            scenario="clear",
            out_dir=initial_dir,
            start_xz=None,
        )
        current_ref = self._load_reference(os.path.join(initial_dir, "reference.csv"))
        self._capture_reference_diff(clear_snapshot, current_ref)
        previous_runtime_snapshot = self._runtime_snapshot(clear_snapshot)
        previous_blocked = previous_runtime_snapshot.blocked_mask.copy()

        events: List[Dict[str, Any]] = []
        stopped = False
        stop_reason = ""
        pending_recovery = False

        for frame in scenario.frames:
            frame_dir = os.path.join(scenario_dir, f"frame_{frame.frame_index:03d}_{frame.label}")
            os.makedirs(frame_dir, exist_ok=True)
            snapshot = self.updater.build(self.handle, mu_min=self.mu_min, request=frame.request)
            self.validator.assert_valid(snapshot)
            runtime_snapshot = self._runtime_snapshot(snapshot)
            self.validator.assert_valid(runtime_snapshot)
            self._write_snapshot_artifacts(frame_dir, runtime_snapshot, frame)

            current_s = float(frame.current_s_fraction) * current_ref.length
            diff_report = self._reference_diff_report(runtime_snapshot, current_ref, frame)
            report = self.classifier.classify(
                runtime_snapshot,
                s_table=current_ref.s,
                xz_table=current_ref.xz,
                current_s_m=current_s,
                previous_blocked_mask=previous_blocked,
                target_changed_event=bool(frame.target_changed_event),
                capsule_proxy_xz=self._capsule_proxy_override(frame),
                reference_snapshot_diff=diff_report,
                accepted_reference={"xz_path": current_ref.xz},
            )
            row: Dict[str, Any] = {
                "scenario": scenario.name,
                "frame_index": int(frame.frame_index),
                "frame_label": frame.label,
                "expected_event": frame.expected_event,
                "event_matches_expected": self._event_matches(frame.expected_event, report.event_class),
                "snapshot_source": runtime_snapshot.source,
                "snapshot_valid": float(runtime_snapshot.stats.get("snapshot_valid", 0.0)),
                "blocked_cells": int(runtime_snapshot.stats.get("blocked_cells", 0.0)),
                "final_feasible_cells": int(runtime_snapshot.stats.get("final_feasible_cells", 0.0)),
                "control_called": False,
                "candidate_accepted": None,
                "rejection_reason": "",
                "path_length_m": None,
                "reference_length_m": None,
                "final_pos_err_m": None,
                "L_q": None,
                "rho_ref": None,
                "capsule_proxy_collision_free": None,
                "recovery_request": None,
                "synthetic_recovery_response": None,
            }
            row.update(self._ground_truth_fields(frame))
            row.update(report.as_dict())

            if report.event_class == ReferenceBlockageClassifier.CURRENT_POSE_UNSAFE:
                self._invalidate_reference_diff()
                decision = self.supervisor.decide(
                    event_class=report.event_class,
                    replanning_triggered=False,
                    reason=report.reason,
                )
                row.update(decision.as_dict())
                recovery_request = self.recovery.build_request(
                    snapshot=runtime_snapshot,
                    event_class=report.event_class,
                    reject_reason=decision.reject_reason,
                    invalid_reason_code=decision.invalid_reason_code,
                    reference_progress_s=report.current_s_m,
                    current_start_xz=tuple(self._xz_at_s(current_ref, report.current_s_m)),
                )
                row["recovery_request"] = recovery_request.as_dict()
                events.append(row)
                previous_blocked = runtime_snapshot.blocked_mask.copy()
                stopped = True
                stop_reason = "CURRENT_POSE_UNSAFE"
                break

            if report.replanning_triggered:
                self._invalidate_reference_diff()
                pre_decision = self.supervisor.decide(
                    event_class=report.event_class,
                    replanning_triggered=True,
                    reason=report.reason,
                    candidate_metrics=None,
                )
                row.update(pre_decision.as_dict())
                start_xz = self._xz_at_s(current_ref, report.current_s_m)
                control_dir = os.path.join(frame_dir, "candidate_control")
                candidate = self._run_control(
                    active_snapshot=runtime_snapshot,
                    clear_snapshot=clear_snapshot,
                    scenario="changed",
                    out_dir=control_dir,
                    start_xz=(float(start_xz[0]), float(start_xz[1])),
                )
                metrics = self._candidate_metrics(candidate)
                row.update(metrics)
                decision = self.supervisor.decide(
                    event_class=report.event_class,
                    replanning_triggered=True,
                    reason=report.reason,
                    candidate_metrics=metrics,
                )
                row.update(decision.as_dict())
                if bool(metrics.get("candidate_accepted", False)):
                    current_ref = self._load_reference(os.path.join(control_dir, "reference.csv"))
                    self._capture_reference_diff(runtime_snapshot, current_ref)
                    pending_recovery = False
                else:
                    pending_recovery = True
                    recovery_request = self.recovery.build_request(
                        snapshot=runtime_snapshot,
                        event_class=report.event_class,
                        reject_reason=str(decision.reject_reason),
                        invalid_reason_code=str(decision.invalid_reason_code),
                        reference_progress_s=report.current_s_m,
                        current_start_xz=(float(start_xz[0]), float(start_xz[1])),
                    )
                    row["recovery_request"] = recovery_request.as_dict()
                    allow_continue = bool(scenario.metadata.get("allow_continue_after_ungraspable", False))
                    row["synthetic_recovery_response"] = self.recovery.synthetic_response(
                        new_snapshot_available=allow_continue
                    ).as_dict()
                    events.append(row)
                    previous_blocked = runtime_snapshot.blocked_mask.copy()
                    if not allow_continue:
                        stopped = True
                        stop_reason = "UNGRASPABLE"
                        break
                    continue
            else:
                decision = self.supervisor.decide(
                    event_class=report.event_class,
                    replanning_triggered=False,
                    reason=report.reason,
                )
                row.update(decision.as_dict())
                if pending_recovery:
                    row["synthetic_recovery_response"] = self.recovery.synthetic_response(
                        new_snapshot_available=True
                    ).as_dict()

            events.append(row)
            previous_blocked = runtime_snapshot.blocked_mask.copy()
            pending_recovery = False

        scenario_summary = {
            "name": scenario.name,
            "description": scenario.description,
            "metadata": dict(scenario.metadata),
            "initial_reference_source": str(clear_snapshot.source),
            "initial_clear": self._candidate_metrics(initial_summary),
            "event_count": len(events),
            "replan_count": int(sum(1 for e in events if e.get("control_called"))),
            "unnecessary_replan_count": int(sum(1 for e in events if e.get("event_class") == "MASK_CHANGED_NONCRITICAL" and e.get("control_called"))),
            "missed_expected_replan_count": int(sum(
                1 for e in events
                if str(e.get("expected_event")) in {"REFERENCE_BLOCKED", "GOAL_CORRIDOR_BLOCKED"}
                and not bool(e.get("replanning_triggered"))
            )),
            "expected_event_match_count": int(sum(1 for e in events if e.get("event_matches_expected"))),
            "stopped": bool(stopped),
            "stop_reason": stop_reason,
            "events": events,
        }
        scenario_summary["reference_diff_sequence_checks"] = self._reference_diff_sequence_checks(
            scenario,
            events,
        )
        self._write_timeline(os.path.join(scenario_dir, "event_timeline.csv"), events)
        self._write_json(os.path.join(scenario_dir, "scenario_summary.json"), scenario_summary)
        return scenario_summary

    def _runtime_snapshot(self, snapshot: ActiveMapSnapshot) -> ActiveMapSnapshot:
        return snapshot

    def _capture_reference_diff(self, snapshot: ActiveMapSnapshot, reference: ReferenceTrace) -> None:
        if self.reference_diff is None:
            return
        self.reference_diff.capture(
            snapshot,
            q_act=None,
            T_base_cam=self.reference_T_base_cam,
            reference_corridor=reference.xz,
            metadata={"reference_length_m": reference.length},
        )

    def _invalidate_reference_diff(self) -> None:
        if self.reference_diff is not None:
            self.reference_diff.invalidate()

    def _reference_diff_report(
        self,
        snapshot: ActiveMapSnapshot,
        reference: ReferenceTrace,
        frame: SyntheticDynamicFrame,
    ) -> Optional[DiffReport]:
        if self.reference_diff is None or not self.reference_diff.has_reference:
            return None
        fov_override = self._reference_diff_fov_override(frame)
        return self.reference_diff.evaluate(
            snapshot,
            self.reference_T_base_cam,
            current_corridor=reference.xz,
            fov_mask_override=fov_override,
        )

    def _reference_diff_fov_override(self, frame: SyntheticDynamicFrame) -> Optional[np.ndarray]:
        mode = str(frame.metadata.get("reference_diff_fov_override", "")).strip().lower()
        if mode == "task":
            return None
        if mode == "full":
            return np.ones(tuple(self.handle.shape), dtype=bool)
        if mode == "empty":
            return np.zeros(tuple(self.handle.shape), dtype=bool)
        if mode == "rects":
            rects = frame.metadata.get("reference_diff_fov_rects", ())
            return self.updater.rect_mask(self.handle, tuple(rects))
        if self.reference_diff_use_task_fov:
            return None
        return np.ones(tuple(self.handle.shape), dtype=bool)

    @staticmethod
    def _capsule_proxy_override(frame: SyntheticDynamicFrame) -> Optional[np.ndarray]:
        value = frame.metadata.get("capsule_proxy_xz")
        if value is None:
            return None
        return np.asarray(value, dtype=np.float64).reshape(-1, 2)

    def _run_control(
        self,
        *,
        active_snapshot: ActiveMapSnapshot,
        clear_snapshot: ActiveMapSnapshot,
        scenario: str,
        out_dir: str,
        start_xz: Optional[Tuple[float, float]],
    ) -> Dict[str, Any]:
        request = MissionRequest(
            scenario=str(scenario),
            clear_snapshot=clear_snapshot,
            active_snapshot=active_snapshot,
            start_xz=start_xz,
            cost_mode=self.cost_mode,
            manip_weight=self.manip_weight,
            manip_kappa=self.manip_kappa,
            manip_mu_safe=DEFAULT_MANIP_MU_SAFE,
            manip_mu_safe_percentile=DEFAULT_MANIP_MU_SAFE_PERCENTILE,
            reference_backend=self.reference_backend,
            dt=self.dt,
            v_ref=self.v_ref,
            terminal_hold_s=self.terminal_hold_s,
            make_plots=self.make_plots,
            write_artifacts=True,
        )
        return self.control.run(request, out_dir=out_dir)

    @staticmethod
    def _event_matches(expected: str, actual: str) -> bool:
        """Return whether a dry-run event matches the synthetic oracle.

        Some synthetic boxes are intentionally defined in workspace geometry,
        while the distance and manipulability branches can produce slightly
        different references.  The oracle therefore supports a small set of
        acceptable priority classes with ``|`` or ``,`` separators.
        """
        expected_text = str(expected or "").strip()
        if not expected_text:
            return True
        allowed = {
            token.strip()
            for chunk in expected_text.split("|")
            for token in chunk.split(",")
            if token.strip()
        }
        return str(actual) in allowed

    @staticmethod
    def _ground_truth_fields(frame: SyntheticDynamicFrame) -> Dict[str, Any]:
        meta = dict(frame.metadata)
        if "ground_truth_replan_required" in meta:
            gt_value = bool(meta.get("ground_truth_replan_required"))
            gt_status = "defined"
        else:
            gt_value = None
            gt_status = str(meta.get("ground_truth_oracle_status", "missing"))
        return {
            "ground_truth_replan_required": gt_value,
            "ground_truth_event_class": str(meta.get("ground_truth_event_class", "")),
            "ground_truth_use_in_metrics": bool(meta.get("ground_truth_use_in_metrics", gt_value is not None)),
            "ground_truth_oracle_status": gt_status,
        }

    @staticmethod
    def _candidate_metrics(summary: Dict[str, Any]) -> Dict[str, Any]:
        case = summary.get("case", {}) if isinstance(summary, dict) else {}
        plan = case.get("plan", {}) if isinstance(case, dict) else {}
        reference = case.get("reference", {}) if isinstance(case, dict) else {}
        control = case.get("control", {}) if isinstance(case, dict) else {}
        capsule = case.get("capsule_proxy_validation", {}) if isinstance(case, dict) else {}
        policy = case.get("candidate_policy", {}) if isinstance(case, dict) else {}
        return {
            "candidate_accepted": bool(policy.get("candidate_accepted", False)),
            "rejection_reason": str(policy.get("reject_reason", "")),
            "plan_found": bool(plan.get("found", False)),
            "valid_grasp": bool(plan.get("valid_grasp", False)),
            "invalid_reason_code": str(plan.get("invalid_reason_code", "")),
            "path_collision_free": bool(plan.get("collision_free", False)),
            "path_length_m": plan.get("path_length_m"),
            "waypoint_count": plan.get("waypoint_count"),
            "nodes_expanded": plan.get("nodes_expanded"),
            "planning_time_ms": plan.get("planning_time_ms"),
            "reference_length_m": reference.get("smoothed_reference_length"),
            "reference_samples_feasible": reference.get("reference_samples_feasible"),
            "final_pos_err_m": control.get("final_pos_err_m"),
            "final_rot_err_deg": control.get("final_rot_err_deg"),
            "L_q": control.get("L_q"),
            "rho_ref": control.get("rho_ref"),
            "gap_max": control.get("gap_max"),
            "capsule_proxy_collision_free": capsule.get("capsule_proxy_collision_free"),
        }

    @staticmethod
    def _load_reference(path: str) -> ReferenceTrace:
        if not os.path.exists(path):
            raise FileNotFoundError(f"reference.csv not found: {path}")
        s_vals: List[float] = []
        xz_vals: List[Tuple[float, float]] = []
        pitch_vals: List[float] = []
        with open(path, "r", newline="", encoding="utf-8") as f:
            reader = csv.DictReader(f)
            for row in reader:
                s_vals.append(float(row["s"]))
                xz_vals.append((float(row["x"]), float(row["z"])))
                pitch_vals.append(float(row["pitch_ref_rad"]))
        return ReferenceTrace(
            s=np.asarray(s_vals, dtype=np.float64),
            xz=np.asarray(xz_vals, dtype=np.float64).reshape(-1, 2),
            pitch=np.asarray(pitch_vals, dtype=np.float64),
        )

    @staticmethod
    def _xz_at_s(reference: ReferenceTrace, s_value: float) -> np.ndarray:
        if reference.s.size == 0:
            return np.zeros(2, dtype=np.float64)
        s = float(np.clip(s_value, 0.0, reference.length))
        if reference.s.size == 1 or reference.length <= 0.0:
            return reference.xz[0].copy()
        idx = int(np.searchsorted(reference.s, s, side="right") - 1)
        idx = max(0, min(idx, reference.s.size - 2))
        s0 = float(reference.s[idx])
        s1 = float(reference.s[idx + 1])
        a = (s - s0) / max(s1 - s0, 1.0e-12)
        return (1.0 - a) * reference.xz[idx] + a * reference.xz[idx + 1]

    def _write_snapshot_artifacts(self, out_dir: str, snapshot: ActiveMapSnapshot, frame: SyntheticDynamicFrame) -> None:
        os.makedirs(out_dir, exist_ok=True)
        np.save(os.path.join(out_dir, "base_feasible_mask.npy"), snapshot.base_feasible_mask)
        np.save(os.path.join(out_dir, "final_active_mask.npy"), snapshot.final_active_mask)
        np.save(os.path.join(out_dir, "blocked_mask.npy"), snapshot.blocked_mask)
        for layer_name, mask in snapshot.layer_masks.items():
            np.save(os.path.join(out_dir, f"layer_{layer_name}.npy"), mask)
        payload = {
            "frame": {
                "frame_index": frame.frame_index,
                "label": frame.label,
                "current_s_fraction": frame.current_s_fraction,
                "expected_event": frame.expected_event,
                "target_changed_event": frame.target_changed_event,
                "notes": frame.notes,
                "metadata": dict(frame.metadata),
            },
            "snapshot_source": snapshot.source,
            "snapshot_stats": dict(snapshot.stats),
        }
        self._write_json(os.path.join(out_dir, "snapshot_summary.json"), payload)
        if self.make_plots:
            self._write_mask_png(os.path.join(out_dir, "blocked_mask.png"), snapshot.blocked_mask, "blocked mask")
            self._write_mask_png(os.path.join(out_dir, "final_active_mask.png"), snapshot.final_active_mask, "final active mask")

    @staticmethod
    def _write_timeline(path: str, events: List[Dict[str, Any]]) -> None:
        os.makedirs(os.path.dirname(path), exist_ok=True)
        keys: List[str] = []
        for event in events:
            for key in event.keys():
                if key not in keys:
                    keys.append(key)
        with open(path, "w", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=keys)
            writer.writeheader()
            for event in events:
                writer.writerow({k: DynamicReplanningHarness._json_scalar(event.get(k)) for k in keys})

    @staticmethod
    def _write_json(path: str, data: Dict[str, Any]) -> None:
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, "w", encoding="utf-8") as f:
            json.dump(DynamicReplanningHarness._jsonable(data), f, indent=2, ensure_ascii=False)

    @staticmethod
    def _reference_diff_sequence_checks(
        scenario: SyntheticDynamicScenario,
        events: List[Dict[str, Any]],
    ) -> Dict[str, Any]:
        events_by_label = {str(e.get("frame_label")): e for e in events}
        checks: List[Dict[str, Any]] = []
        for frame in scenario.frames:
            expected = frame.metadata.get("reference_diff_expected")
            if not isinstance(expected, dict):
                continue
            event = events_by_label.get(frame.label)
            check: Dict[str, Any] = {
                "frame_label": frame.label,
                "expected": dict(expected),
                "actual": {},
                "pass": event is not None,
                "failures": [],
            }
            if event is None:
                check["failures"].append("frame_event_missing")
                checks.append(check)
                continue
            for key, expected_value in expected.items():
                if key.endswith("_min"):
                    actual_key = key[:-4]
                    actual_value = event.get(actual_key)
                    try:
                        ok = float(actual_value) >= float(expected_value)
                    except (TypeError, ValueError):
                        ok = False
                elif key.endswith("_max"):
                    actual_key = key[:-4]
                    actual_value = event.get(actual_key)
                    try:
                        ok = float(actual_value) <= float(expected_value)
                    except (TypeError, ValueError):
                        ok = False
                else:
                    actual_key = key
                    actual_value = event.get(actual_key)
                    ok = actual_value == expected_value
                check["actual"][actual_key] = actual_value
                if not ok:
                    check["pass"] = False
                    check["failures"].append(
                        {
                            "key": key,
                            "actual": actual_value,
                            "expected": expected_value,
                        }
                    )
            checks.append(check)
        return {
            "enabled": bool(checks),
            "pass": bool(all(bool(c.get("pass")) for c in checks)) if checks else True,
            "checks": checks,
        }

    @staticmethod
    def _json_scalar(value: Any) -> Any:
        if isinstance(value, (dict, list, tuple, np.ndarray)):
            return json.dumps(DynamicReplanningHarness._jsonable(value), ensure_ascii=False)
        if isinstance(value, np.generic):
            return value.item()
        return value

    @staticmethod
    def _jsonable(value: Any) -> Any:
        if isinstance(value, dict):
            return {str(k): DynamicReplanningHarness._jsonable(v) for k, v in value.items()}
        if isinstance(value, (list, tuple)):
            return [DynamicReplanningHarness._jsonable(v) for v in value]
        if isinstance(value, np.ndarray):
            return value.tolist()
        if isinstance(value, np.generic):
            return value.item()
        return value

    @staticmethod
    def _write_mask_png(path: str, mask: np.ndarray, title: str) -> Optional[str]:
        try:
            import matplotlib
            matplotlib.use("Agg")
            import matplotlib.pyplot as plt
        except Exception:
            return None
        try:
            fig, ax = plt.subplots(figsize=(5.0, 5.0), dpi=140)
            ax.imshow(np.asarray(mask, dtype=float).T, origin="lower", cmap="gray_r", aspect="equal")
            ax.set_title(title)
            ax.set_xlabel("x cell")
            ax.set_ylabel("z cell")
            fig.tight_layout()
            os.makedirs(os.path.dirname(path), exist_ok=True)
            fig.savefig(path)
            plt.close(fig)
            return path
        except Exception:
            return None
